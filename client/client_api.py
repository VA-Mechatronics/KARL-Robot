import logging
import threading
import time
import numpy as np
import zmq
import contextlib
from pathlib import Path
from typing import TypeAlias, Literal, Callable, Any
from common.robot_config import RobotConfig
from common.target import Target
from common.mcu_telemetry import RobotTelemetry
from common.kinematics.dh.dh_kinematics import DHKinematics
from client.skills.basic_skills import BasicSkills
from client.skills.homing_skills import HomingSkills
from client.skills.program_skills import ProgramSkills
from client.skills.workspace_skills import WorkspaceSkills

Frame: TypeAlias = Literal["joint", "base", "tool"]
JointName: TypeAlias = Literal["j1", "j2", "j3", "j4", "j5", "j6", "j7", "tool"]
CartesianAxis: TypeAlias = Literal["x", "y", "z", "A", "B", "C"]
JogDistance: TypeAlias = float | Literal["+", "-"]
EventName: TypeAlias = Literal["state", "telemetry", "overwrite", "target", "config", "workspace", "smart_input"]
StateName: TypeAlias = Literal["boot", "error", "guiding", "holding", "idle", "moving", "stopping"]


class RobotAPI(BasicSkills, HomingSkills, ProgramSkills, WorkspaceSkills):
    def __init__(self):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self._last_msg = 0
        self._callbacks = {}
        self._ctx = zmq.Context()
        self._push = None
        self._push_addr = None
        self._sub = None
        self._sub_addr = None
        self._req = None
        self._req_addr = None
        self._recv_thread = None
        self._is_running = False
        self._is_paused = False
        self._state = None
        self._info = None
        self._telemetry = RobotTelemetry.default()
        self._act_target = Target()
        self.set_target = self._act_target.copy()
        self.kinematics = DHKinematics()  # API kinematics check for same in Controller
        self._blocking = False
        self.log.debug("API initialized (Not connected yet)")

    # region *** ZMQ connection ***
    def connect(self,
                host: str = "localhost",
                push_port: int = 5555,
                sub_port: int = 5556,
                req_port: int = 5557) -> bool:
        """Connect API to Controller.

        Args:
            host: Hostname or IP address of the robot controller (e.g. "192.168.1.100").
            push_port: Port for async command queue (default: 5555).
            sub_port: Port for telemetry & state broadcasts (default: 5556).
            req_port: Port for synchronous requests/handshakes (default: 5557).

        Examples:
            >>> robot = RobotAPI()
            >>> # Local host connection
            >>> robot.connect()
            >>> # Remote robot connection
            >>> robot.connect("192.168.1.50")
        """
        host = host.removeprefix("tcp://").strip("/")
        push_addr = f"tcp://{host}:{push_port}"
        sub_addr = f"tcp://{host}:{sub_port}"
        req_addr = f"tcp://{host}:{req_port}"
        if self._push or self._sub or self._req or self._is_running:
            self.disconnect()

        self.log.info(f"Connecting to {host} [PUSH:{push_port}, SUB:{sub_port}, REQ:{req_port}]")
        self._push_addr = push_addr
        self._sub_addr = sub_addr
        self._req_addr = req_addr

        self._push = self._ctx.socket(zmq.PUSH)
        self._push.setsockopt(zmq.LINGER, 0)
        self._push.connect(self._push_addr)

        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.LINGER, 0)
        self._sub.setsockopt(zmq.RCVTIMEO, 2000)  # 2sec Backend not connected timeout
        self._sub.connect(self._sub_addr)
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "")

        self._req_addr = req_addr
        self._req = self._ctx.socket(zmq.REQ)
        self._req.setsockopt(zmq.LINGER, 0)
        self._req.setsockopt(zmq.RCVTIMEO, 2000)  # 2sec Handshake timeout
        self._req.connect(self._req_addr)

        self._last_msg = time.time()

        self._is_running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        self.read_config()
        return True

    def disconnect(self):
        """Disconnect API from Controller"""
        self.log.info("Disconnecting current sockets...")
        self._is_running = False

        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=2.0)
            self._recv_thread = None

        if self._push:
            try:
                self._push.disconnect(self._push_addr)
            except Exception:
                pass
            self._push.close(linger=0)
            self._push = None

        if self._sub:
            try:
                self._sub.disconnect(self._sub_addr)
            except Exception:
                pass
            self._sub.close(linger=0)
            self._sub = None

        if self._req:
            try:
                self._req.disconnect(self._req_addr)
            except Exception:
                pass
            self._req.close(linger=0)
            self._req = None
    # endregion

    # region *** Callbacks ***
    def on(self, event_name: EventName, callback: Callable[..., Any]) -> None:
        """Register a callback function for asynchronous controller events.

            Args:
                event_name: The target event topic to subscribe to.
                callback: Function invoked when a new event payload is received.

            Examples:
                >>> def print_telemetry(payload):
                ...     print(f"Received telemetry: {payload}")
                ...
                >>> robot.on("telemetry", print_telemetry)
            """
        self._callbacks.setdefault(event_name, []).append(callback)

    def _emit(self, event_name, *args):
        for cb in self._callbacks.get(event_name, []):
            cb(*args)

    def _process_msg(self, msg: dict) -> None:
        if not isinstance(msg, dict) or "event" not in msg:
            self.log.warning(f"Unknown message format: {msg}")
            return
        event = msg["event"]
        payload = msg.get("payload")
        match event:
            case "state":
                self._state = payload
                self._info = msg.get("info")
            case "telemetry":
                self._telemetry = RobotTelemetry.from_dict(payload)
            case "target":
                self._act_target = Target.from_dict(payload)
            case "config":
                self.kinematics.config = RobotConfig.from_dict(payload)
        self._emit(event, payload)
        self._last_msg = time.time()

    def _recv_loop(self):
        poller = zmq.Poller()
        if self._sub:
            poller.register(self._sub, zmq.POLLIN)

        while self._is_running:
            if not self._sub:
                break
            try:
                socks = dict(poller.poll(100))  # 100ms
                if self._sub in socks and socks[self._sub] == zmq.POLLIN:
                    while True:
                        try:
                            msg = self._sub.recv_json(flags=zmq.NOBLOCK)
                            self._process_msg(msg)
                        except zmq.Again:
                            break
            except (zmq.error.ZMQError, AttributeError) as e:
                self.log.debug(f"Recv loop interrupted: {e}")
                break
            # Connection Timeout
            if time.time() - self._last_msg > 2.0:
                self._emit("state", "Backend not connected")
    # endregion

    @contextlib.contextmanager
    def blocking_mode(self) -> None:
        """Set API in blocking mode. Scripts will await movement.

        Examples:
            >>> with robot.blocking_mode():
            >>>     robot.moveJ(custom_target)
            >>>     # Program is awaiting reaching custom_target
        """
        old_mode = self._blocking
        self._blocking = True
        try:
            yield
        finally:
            self._blocking = old_mode

    @contextlib.contextmanager
    def non_blocking_mode(self) -> None:
        """Set API in non blocking mode. Scripts will not await movement.
        ATTENTION! You have to handle stop yourself then.

        Examples:
            >>> with robot.non_blocking_mode():
            >>>     robot.jog_joint(joint="j1", distance="+")
            >>>     while robot.telemetry.axes[5].act_pos <= 20:
            >>>         robot.stop()
        """
        old_mode = self._blocking
        self._blocking = False
        try:
            yield
        finally:
            self._blocking = old_mode

    def _send_cmd(self, cmd):
        self._push.send_json(cmd)
        if self._blocking:
            # 1. await Robot to calculate and move
            while self._state not in ["moving", "stopping", "error"]:
                time.sleep(0.1)
            # 2. await Robot to hold again -> motion complete
            while self._state != "holding" or self._is_paused:
                time.sleep(0.1)

    def _send_req_cmd(self, cmd: dict) -> dict:
        try:
            self._req.send_json(cmd)
            response = self._req.recv_json()
            if response.get("status") != "ok":
                self.log.error(f"send_req_cmd failed for {cmd.get('cmd')}: {response.get('message')}")
            return response
        except zmq.error.Again:
            self.log.error(f"send_req_cmd TIMEOUT for command {cmd.get('cmd')}")
            self._req.close(linger=0)
            self._req = self._ctx.socket(zmq.REQ)
            self._req.setsockopt(zmq.LINGER, 0)
            self._req.setsockopt(zmq.RCVTIMEO, 2000)
            self._req.connect(self._req_addr)
            return {"status": "timeout", "message": "Controller did not respond in time"}

    # region *** Getter ***
    @property
    def state(self) -> StateName:
        """Current state of the controller."""
        return self._state

    @property
    def info(self) -> str:
        """Additional status or error details provided by the controller state."""
        return self._info

    @property
    def config(self) -> RobotConfig:
        """Read robot configuration fetched directly from the controller."""
        return self.read_config()

    @property
    def telemetry(self) -> RobotTelemetry:
        """Live motion telemetry including joint positions and velocities."""
        return self._telemetry

    @property
    def act_target(self) -> Target:
        """Current actual target pose and joint values."""
        return self._act_target

    # region *** Commands ***
    def wait(self, sec: float) -> None:
        """Time Delay"""
        start_time = time.time()
        end_time = start_time + sec
        while True:
            remaining = end_time - time.time()
            if remaining <= 0:
                break
            self.log.info(f"Waiting.. {remaining:.2f}sec remaining")
            time.sleep(min(0.1, remaining))

    def stop(self) -> None:
        """Switch controller to stopping state"""
        self._send_cmd({"cmd": "stop"})

    _JOINT_MAP: dict[str, int] = {"j1": 0, "j2": 1, "j3": 2, "j4": 3, "j5": 4, "j6": 5, "j7": 6, "tool": 6}
    _AXIS_MAP: dict[str, int] = {"x": 0, "y": 1, "z": 2, "A": 3, "B": 4, "C": 5}

    def jog(self, frame: Frame, axis: int, distance: JogDistance) -> None:
        """Jog movement of the robot axes. Useful for accessing them via an index.
        Alternatively, you can use jog_joint(), jog_base(), or jog_tool() to make the code more readable.

        Args:
            frame: Coordinate system. "joint" (deg), "base" (mm), or "tool" (mm).
            axis: Index of the axis to move.
                - Joint frame: 0-5 = Joint 6 = Tooling.
                - Base/Tool frame: 0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C.
            distance: Travel distance. Unit depends on frame (deg or mm).
                If "+" or "-" performs a continuous jog.

        Examples:
            >>> # Jog axis 5 = joint 6, continuous in positive direction
            >>> robot.jog(frame="joint", axis=5, distance="+")
            >>> # Jog in Tool frame Z-Axis -5.0mm
            >>> robot.jog(frame="tool", axis=2, distance=-5.0)
        """
        if isinstance(distance, str):
            direction = 1 if distance == "+" else -1
            distance = None
        else:
            direction = 1 if distance >= 0 else -1
            distance = abs(distance)
        self._send_cmd({"cmd": "jog", "frame": frame, "axis": axis, "direction": direction, "distance": distance})

    def jog_joint(self, joint: JointName, distance: JogDistance) -> None:
        """Jog a single joint or tooling axis.

        Args:
            joint: Joint ("j1".."j7" or "tool" for "j7").
            distance: Travel distance in deg, or continuous jog ("+" / "-").

        Examples:
            >>> robot.jog_joint("j1", "+")
            >>> robot.jog_joint("j3", -15.0)
            >>> robot.jog_joint("tool", 10.0) # same as robot.jog_joint("j7", 10.0)
        """
        self.jog(frame="joint", axis=self._JOINT_MAP[joint], distance=distance)

    def jog_base(self, axis: CartesianAxis, distance: JogDistance) -> None:
        """Jog in Cartesian Base coordinate system.

        Args:
            axis: Cartesian axis ("x", "y", "z", "A", "B", "C").
            distance: Travel distance in mm/deg, or continuous jog ("+" / "-").

        Examples:
            >>> robot.jog_base("z", "+")
            >>> robot.jog_base("x", -10.0)
        """
        self.jog(frame="base", axis=self._AXIS_MAP[axis], distance=distance)

    def jog_tool(self, axis: CartesianAxis, distance: JogDistance) -> None:
        """Jog in Cartesian Tool coordinate system relative to active TCP.

        Args:
            axis: Cartesian axis ("x", "y", "z", "A", "B", "C").
            distance: Travel distance in mm/deg, or continuous jog ("+" / "-").

        Examples:
            >>> robot.jog_tool("z", -20.0)
            >>> robot.jog_tool("c", "+")
        """
        self.jog(frame="tool", axis=self._AXIS_MAP[axis], distance=distance)

    def moveJ(self, target: Target | list[float | None] | np.ndarray, gripper: float | None = None) -> None:
        """Move Joints is PTP motion.

        Args:
            target: Target object or list of joint position (default: None joints will remain in actual position).
            gripper: Optional Gripper position (default: None results in no gripper motion).

        Examples:
            >>> # Move to Target "custom1.json" and open Gripper to 50mm
            >>> robot.moveJ(targets["custom1"], gripper=50)
            >>> # Just move J1 and J3 to joint position
            >>> robot.moveJ([10, None, 50])
        """
        if not isinstance(target, Target):
            target = Target(joints=target)
        self._send_cmd({"cmd": "move_j", "target": target.to_dict(), "gripper": gripper})

    def moveL(self, target: Target | list[float | None] | np.ndarray, gripper: float | None = None) -> None:
        """Move Line is Linear motion.

        Args:
            target: Target object or list of joint position (default: None joints will remain in actual position).
            gripper: Optional Gripper position (default: None results in no gripper motion).

        Examples:
            >>> # Move to Target "custom1.json" and open Gripper to 50mm
            >>> robot.moveL(targets["custom1"], gripper=50)
            >>> # Just move J1 and J3 to joint position
            >>> robot.moveL([10, None, 50])
        """
        if not isinstance(target, Target):
            target = Target(joints=target)
        self._send_cmd({"cmd": "move_l", "target": target.to_dict(), "gripper": gripper})

    def movePoly(self, targets: list[Target | list[float]], gripper: float | None = None) -> None:
        """Move Poly spline w/o stopping over list of Targets or "list of joint position"-list.

        Args:
            targets: list of "Target object" or list of "list of joint position" (default: None joints will remain in actual position).
            gripper: Optional Gripper position (default: None results in no gripper motion).

        Examples:
            >>> # Move through list of Target objects
            >>> robot.movePoly(targets=[targets["1"], targets["2"], targets["3"]], gripper=50)
            >>> # Move through list of list[J1...J6]
            >>> robot.movePoly([[150, 200], [160, None, 10]])
        """
        formatted_targets = []
        for t in targets:
            if not isinstance(t, Target):
                t = Target(joints=t)
            formatted_targets.append(t.to_dict())
        self._send_cmd({"cmd": "move_poly", "targets": formatted_targets, "gripper": gripper})

    def gripper(self, payload: float) -> None:
        """Move Gripper to Position.

        Args:
            payload: 0.0 ... 100.0 (mm) depending on Tooling. For picking: set payload = 0. Gripper will stop at resistance.

        Examples:
            >>> # pick part. Gripper stops and holds by resistance
            >>> robot.gripper(0)
            >>> # open gripper to 60.5mm
            >>> robot.gripper(60.5)
        """
        self._send_cmd({"cmd": "gripper", "payload": payload})

    def error(self, info: str | None = None) -> bool:
        """Fault injection. Switch controller to error state.

        Args:
            info: optional custom string.

        Examples:
            >>> robot.error(info="optional custom error message")
        """
        res = self._send_req_cmd({"cmd": "reset", "payload": info})
        return res.get("status") == "ok"

    def reset(self) -> bool:
        """Switch controller to boot state"""
        res = self._send_req_cmd({"cmd": "reset"})
        return res.get("status") == "ok"

    def power_off(self) -> bool:
        """Switch controller to idle state and switch motor power of. ATTENTION! Robot will collapse!"""
        res = self._send_req_cmd({"cmd": "power_off"})
        return res.get("status") == "ok"

    def guide(self) -> bool:
        """Switch controller to guiding state"""
        res = self._send_req_cmd({"cmd": "guide"})
        return res.get("status") == "ok"

    def overwrite(self, payload: float) -> bool:
        """Speed Overwrite.

        Args:
            payload: Speed overwrite factor normally 1.0 or 0.0...2.0. Configuration max speed will limit anyway.
        """
        res = self._send_req_cmd({"cmd": "overwrite", "payload": payload})
        return res.get("status") == "ok"

    def read_config(self) -> RobotConfig | None:
        """Read robot configuration fetched directly from the controller.

        Examples:
            >>> act_config: RobotConfig = robot.read_config()
        """
        res = self._send_req_cmd({"cmd": "read_config"})
        if res.get("status") == "ok":
            self.kinematics.config = RobotConfig.from_dict(res.get("config"))
            return self.kinematics.config
        return None

    def write_config(self, config: RobotConfig | str | Path) -> bool:
        """Write config to MCUs and update in Controller and API.

        Args:
            config: RobotConfig Object or filepath to a config.json - file.

        Examples:
            >>> robot.write_config(WORKSPACE_DIR / "configs" / "custom_config.json")
        """
        if not isinstance(config, RobotConfig):
            config = RobotConfig.read_from_file(config)
        res = self._send_req_cmd({"cmd": "write_config", "payload": config.to_dict()})
        return res.get("status") == "ok"

    def set_tcp_offset(self, x: float=0.0, y: float=0.0, z: float=0.0, A: float=0.0, B: float=0.0, C: float=0.0,
                       matrix: np.ndarray | None = None, overwrite: bool = False) -> "RobotAPI":
        """Set dynamic virtual TCP offset relative to physical tool center point.

                Args:
                    x, y, z: Translation in mm. Translation first.
                    A, B, C: Rotation in degrees (RPY / Euler Angles XYZ). Applied after Translation
                    matrix: Optional explicit 4x4 transformation matrix override.
                    overwrite: If True, replaces existing offset instead of accumulating.

                Examples:
                    >>> # Translation first
                    >>> robot.set_tcp_offset(z=20).set_tcp_offset(A=45) # same as robot.set_tcp_offset(z=20, A=45)
                    >>> # Rotation first
                    >>> robot.set_tcp_offset(A=45).set_tcp_offset(z=20)
                """
        if matrix is not None:
            delta_mat = np.asarray(matrix, dtype=float)
            if delta_mat.shape != (4, 4):
                raise ValueError(f"Explicit matrix must be (4, 4), got {delta_mat.shape}")
        else:
            from scipy.spatial.transform import Rotation as R
            delta_mat = np.eye(4, dtype=float)
            delta_mat[0:3, 3] = [x, y, z]
            if any([A, B, C]):
                rot_mat = R.from_euler("xyz", [A, B, C], degrees=True).as_matrix()
                delta_mat[0:3, 0:3] = rot_mat
        new_config = self.config.copy()
        if overwrite:
            new_config.tcp_offset.matrix = delta_mat
        else:
            new_config.tcp_offset.matrix = new_config.tcp_offset.matrix @ delta_mat
        self.write_config(new_config)
        return self  # Return self to enable Method Chaining!

    def reset_tcp_offset(self) -> "RobotAPI":
        """Reset virtual TCP offset to identity matrix (native tool frame).

        Examples:
            >>> robot.reset_tcp_offset()
        """
        self.set_tcp_offset(matrix=np.eye(4, dtype=float), overwrite=True)
        return self  # Return self to enable Method Chaining!

    def set_zero(self, joint: JointName) -> bool:
        """Overwrite MCU homing offset so act_pos is Zero and update its new home_offset.
        ATTENTION! This overwrites the MCUs Home offset and affects the whole positioning.

        Args:
            joint: Joint ("j1".."j6" or "tool").

        Examples:
            >>> robot.set_zero("j2")
        """
        res = self._send_req_cmd({"cmd": "set_zero", "payload": self._JOINT_MAP[joint]})
        return res.get("status") == "ok"

    def jump_to_bootloader(self, joint: JointName) -> bool:
        """Set MCU in USB DFU Bootloader Mode.

        Args:
            joint: Joint ("j1".."j6" or "tool").

        Examples:
            >>> robot.jump_to_bootloader("j6")
        """
        res = self._send_req_cmd({"cmd": "jump_to_bootloader", "payload": self._JOINT_MAP[joint]})
        return res.get("status") == "ok"

    def set_mcu_homing(self, joint: JointName) -> bool:
        """Set MCU in Homing Mode.
        Only used for j6 (last Wrist Joint with proximity Sensor).
        Move Joint trough proximity Sensor and MCU will update its homing offset.

        Args:
            joint: Joint ("j1".."j6" or "tool").

        Examples:
            >>> def on_homing(payload)
            >>>     node_id, sensor_id, event = payload
            >>>     if mcu_node_id == 6 and sensor_id == 10
            >>>         global homing_state = event
            >>>
            >>> self.robot.jog("j6", "+")
            >>> while homing_state != 3
            >>>     time.sleep(0.1)
            >>> print(f"Axis 5 is homed")
        """
        res = self._send_req_cmd({"cmd": "start_homing", "payload": self._JOINT_MAP[joint]})
        return res.get("status") == "ok"
