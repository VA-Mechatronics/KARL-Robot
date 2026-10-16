import logging
import threading
import time
import zmq

from backend.controller.state_machine import StateMachine
from backend.controller.states.boot_state import BootState
from backend.controller.states.idle_state import IdleState
from backend.controller.states.holding_state import HoldingState
from backend.controller.states.moving_state import MovingState
from backend.controller.states.guiding_state import GuidingState
from backend.controller.states.stopping_state import StoppingState
from backend.controller.states.error_state import ErrorState

from backend.controller.hw_controller import HWController
from backend.controller.io_controller import IOController

from common.workspace_manager import WorkspaceManager
from backend.program_runner import ProgramRunner
from backend.guiding_recorder import GuidingRecorder

from common.trajectories.ptp_trajectory import ptp_trajectory
from common.trajectories.line_trajectory import line_trajectory
from common.trajectories.jog_trajectory import jog_trajectory
from common.trajectories.poly_trajectory import poly_trajectory

from common.robot_config import RobotConfig
from common.kinematics.dh.dh_kinematics import DHKinematics
from common.target import Target


class RobotController:
    def __init__(self):
        self.log = logging.getLogger(__name__)
        self.ctx = zmq.Context.instance()

        self.pull = self.ctx.socket(zmq.PULL)
        self.pull.bind("tcp://*:5555")
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.bind("tcp://*:5556")
        self.rep = self.ctx.socket(zmq.REP)
        self.rep.bind("tcp://*:5557")

        self.states = {
            "boot": BootState(),
            "idle": IdleState(),
            "holding": HoldingState(),
            "moving": MovingState(),
            "guiding": GuidingState(),
            "stopping": StoppingState(),
            "error": ErrorState(),
        }

        self.workspace = WorkspaceManager()
        self.runner = ProgramRunner()
        self.recorder = GuidingRecorder(self.workspace)

        # Controller kinematics check for same in API
        self.kinematics = DHKinematics()

        self.io = IOController(controller=self)
        self.hw = HWController(io_controller=self.io)
        self.sm = StateMachine(controller=self, initial_state=self.states["boot"])

        self.pos_traj = None
        self.vel_traj = None
        self.motion_start_time = 0.0
        self.motion_duration = 0.0
        self.trajectory_time = 0.0
        self.motion_finished_sent = False
        self.overwrite = 1.0
        self._last_motion_cmd = None

        self._running = True
        self._run_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._run_thread.start()
        self.log.info(f"Robot Controller Backend started")

    def _run_loop(self):
        time.sleep(1.0)
        fast_interval = 0.01
        publish_interval = 0.1
        last_publish_time = 0.0
        while self._running:
            fast_start = time.time()

            try:
                # Async PUSH Socket
                while True:
                    try:
                        cmd = self.pull.recv_json(flags=zmq.NOBLOCK)
                        self.handle_cmd(cmd)
                    except zmq.Again:
                        break

                # Sync REQ/REP Socket
                while True:
                    try:
                        cmd = self.rep.recv_json(flags=zmq.NOBLOCK)
                        self.handle_sync_cmd(cmd)
                    except zmq.Again:
                        break

                # motion
                self.io.update()
                self.sm.update()

                # publish interval
                if time.time() - last_publish_time >= publish_interval:
                    last_publish_time = time.time()
                    self.publish_state(self.sm.state.name)
                    self.publish_telemetry()
                    self.publish_target()

            except Exception as e:
                self.log.error(f"Robot Controller Error: {e}")

            # fast interval
            elapsed = time.time() - fast_start
            sleep_time = fast_interval - elapsed
            # self.log.info(f"fast_duration: {elapsed:.4f}")
            if sleep_time > 0:
                time.sleep(sleep_time)

    # region *** Async ***
    def publish_state(self, name: str, info: str = None):
        self.pub.send_json({"event": "state", "payload": name, "info": info})

    def publish_telemetry(self):
        self.pub.send_json({"event": "telemetry", "payload": self.hw.telemetry.to_dict()})

    def publish_overwrite(self):
        self.pub.send_json({"event": "overwrite", "payload": self.overwrite})

    def publish_target(self):
        joints = self.hw.get_joints_position()
        target = Target(
            joints=joints,
            tcp=self.kinematics.joints_to_tcp(joints),
        )
        self.pub.send_json({"event": "target", "payload": target.to_dict()})

    def publish_smart_input(self, node_id, input_id, smart_event):
        self.pub.send_json({"event": "smart_input", "payload": [node_id, input_id, smart_event]})

    def set_state(self, name: str, info: str | None = None):
        self.sm.transition(self.states[name], info=info)
        self.publish_state(name, info)

    # region *** Async Commands***
    def handle_cmd(self, cmd: dict):
        self.log.info(f"handle_cmd: {cmd}")
        c = cmd.get("cmd")

        # jog is excluded because unpredictable relative motion after pause
        if c in ["move_j", "move_l", "gripper"]:
            self._last_motion_cmd = cmd

        match c:
            case "stop":
                self.set_state("stopping")
            case "jog":
                act_target = Target(
                    joints=self.hw.get_joints_position(),
                    tcp=self.kinematics.joints_to_tcp(self.hw.get_joints_position())
                )
                frame = cmd.get("frame")
                axis = cmd.get("axis")
                direction = cmd.get("direction")
                distance = cmd.get("distance")
                self.vel_traj, self.pos_traj, self.motion_duration = jog_trajectory(
                    act_target, frame, axis, direction, self.kinematics, self.hw.config, distance, self.overwrite
                )
                self.motion_start_time = time.time()
                self.set_state("moving")
            case "move_j":
                act_target = Target(
                    joints=self.hw.get_joints_position(),
                    tcp=self.kinematics.joints_to_tcp(self.hw.get_joints_position()),
                )
                set_target = Target.from_dict(cmd.get("target"))
                set_target.joints[6] = cmd.get("gripper")
                self.vel_traj, self.pos_traj, self.motion_duration = ptp_trajectory(
                    act_target, set_target, self.kinematics, self.hw.config, self.overwrite
                )
                self.motion_start_time = time.time()
                self.set_state("moving")
            case "move_l":
                act_target = Target(
                    joints=self.hw.get_joints_position(),
                    tcp=self.kinematics.joints_to_tcp(self.hw.get_joints_position())
                )
                set_target = Target.from_dict(cmd.get("target"))
                set_target.joints[6] = cmd.get("gripper")
                self.vel_traj, self.pos_traj, self.motion_duration = line_trajectory(
                    act_target, set_target, self.kinematics, self.hw.config, self.overwrite
                )
                self.motion_start_time = time.time()
                self.set_state("moving")
            case "move_poly":
                act_target = Target(
                    joints=self.hw.get_joints_position(),
                    tcp=self.kinematics.joints_to_tcp(self.hw.get_joints_position())
                )
                raw_targets = cmd.get("targets", [])
                gripper = cmd.get("gripper")
                set_targets = [Target.from_dict(t) for t in raw_targets]
                for t in set_targets:
                    t.joints[6] = gripper
                self.vel_traj, self.pos_traj, self.motion_duration = poly_trajectory(
                    act_target, set_targets, self.kinematics, self.hw.config, self.overwrite
                )
                self.motion_start_time = time.time()
                self.set_state("moving")
            case "gripper":
                act_target = Target(
                    joints=self.hw.get_joints_position(),
                    tcp=self.kinematics.joints_to_tcp(self.hw.get_joints_position())
                )
                set_target = act_target.copy()
                set_target.joints[6] = cmd.get("payload")
                self.vel_traj, self.pos_traj, self.motion_duration = ptp_trajectory(
                    act_target, set_target, self.kinematics, self.hw.config, self.overwrite
                )
                self.motion_start_time = time.time()
                self.set_state("moving")
            case _:
                self.log.warning(f"Unknown cmd {c}")
    # endregion

    # region *** Sync Commands ***
    def _rep_sync(self, msg: dict, reply: bool = True):
        if reply:
            self.rep.send_json(msg)

    def handle_sync_cmd(self, cmd: dict, reply: bool = True):
        self.log.info(f"handle_sync_cmd: {cmd}")
        c = cmd.get("cmd")

        match c:
            case "error":
                self.set_state("error", info=cmd.get("payload"))
                self._rep_sync({"status": "ok", "state": self.sm.state.name}, reply)
            case "reset":
                self.set_state("boot")
                self._rep_sync({"status": "ok", "state": self.sm.state.name}, reply)
            case "power_off":
                self.set_state("idle")
                self._rep_sync({"status": "ok", "state": self.sm.state.name}, reply)
            case "guide":
                self.set_state("guiding")
                self._rep_sync({"status": "ok", "state": self.sm.state.name}, reply)
            case "overwrite":
                self.overwrite = cmd.get("payload")
                self.publish_overwrite()
                self._rep_sync({"status": "ok", "overwrite": self.overwrite}, reply)
            case "read_config":
                config = self.hw.get_config()
                self.kinematics.config = config
                self.log.info(f"read config from MCU: {config.to_dict()}")
                self.pub.send_json({"event": "config", "payload": config.to_dict()})
                self._rep_sync({"status": "ok", "config": config.to_dict()}, reply)
            case "write_config":
                config = RobotConfig.from_dict(cmd.get("payload"))
                success = self.hw.set_config(config)
                if success:
                    self.kinematics.config = config
                    self.pub.send_json({"event": "config", "payload": config.to_dict()})
                    self._rep_sync({"status": "ok", "message": "Config updated successfully"}, reply)
                else:
                    self.log.warning(f"write config FAILED")
                    self._rep_sync({
                        "status": "error",
                        "message": "Failed to set config on MCUs (CAN ACK Timeout)"
                        }, reply)
            case "set_zero":
                mcu_id = cmd.get("payload")
                self.hw.set_zero(mcu_id)
                config = self.hw.get_config()
                self.kinematics.config = config
                self.pub.send_json({"event": "config", "payload": config.to_dict()})
                self._rep_sync({"status": "ok", "config": config.to_dict()}, reply)
            case "jump_to_bootloader":
                mcu_id = cmd.get("payload")
                self.hw.jump_to_bootloader(mcu_id)
                self.set_state("boot")
                self._rep_sync({"status": "ok"}, reply)
            case "start_homing":
                mcu_id = cmd.get("payload")
                self.hw.start_homing(mcu_id)
                self._rep_sync({"status": "ok"}, reply)

            # --- Program Runner ---
            case "run_program":
                self._last_motion_cmd = None
                prog_name = cmd.get("program")
                repeat = cmd.get("repeat", "1")
                prog_path = self.workspace.get_program_path(prog_name)
                self.runner.run(prog_path, repeat=repeat)
                self._rep_sync({"status": "ok"}, reply)
            case "pause_program":
                self.runner.pause()
                self.handle_cmd({"cmd": "stop"})
                self._rep_sync({"status": "ok"}, reply)
            case "resume_program":
                if self._last_motion_cmd is not None:
                    self.handle_cmd(self._last_motion_cmd)
                else:
                    self.handle_cmd({"cmd": "stop"})
                self.runner.resume()
                self._rep_sync({"status": "ok"}, reply)
            case "cancel_program":
                self._last_motion_cmd = None
                self.runner.stop()
                self.handle_cmd({"cmd": "stop"})
                self._rep_sync({"status": "ok"}, reply)

            # -- Guiding Recorder ---
            case "clear_guide":
                self.recorder.clear()
                self._rep_sync({"status": "ok"}, reply)
            case "record_target":
                cmd_type = cmd.get("type", "moveJ")
                act_target = Target(
                    joints=self.hw.get_joints_position(),
                    tcp=self.kinematics.joints_to_tcp(self.hw.get_joints_position())
                )
                self.recorder.append_cmd(cmd_type, act_target)
                self._rep_sync({"status": "ok"}, reply)

            # --- Workspace Sync Commands ---
            case "push_workspace":
                b64_zip = cmd.get("payload")
                self.workspace.from_b64_zip(b64_zip)
                self._rep_sync({"status": "ok"}, reply)
            case "pull_workspace":
                b64_zip = self.workspace.to_b64_zip()
                self.pub.send_json({"event": "workspace", "payload": b64_zip})
                self._rep_sync({"status": "ok"}, reply)
            case _:
                self.log.warning(f"Unknown sync_cmd {c}")
                self._rep_sync({"status": "error", "message": f"Unknown sync_cmd {c}"}, reply)
    # endregion

    def stop(self):
        self._running = False
