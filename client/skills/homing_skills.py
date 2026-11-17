import time
from typing import TYPE_CHECKING, TypeAlias, Literal
if TYPE_CHECKING:
    from client.client_api import RobotAPI

JointName: TypeAlias = Literal["j1", "j2", "j3", "j4", "j5", "j6", "j7", "tool"]
_JOINT_MAP: dict[str, int] = {"j1": 0, "j2": 1, "j3": 2, "j4": 3, "j5": 4, "j6": 5, "j7": 6, "tool": 6}


class SensorHoming:
    def __init__(self, robot_api: "RobotAPI", joint: JointName):
        self.robot = robot_api
        self.joint = joint
        self.homing_state = None
        self.robot.on("smart_input", self._on_sensor)

    def _on_sensor(self, payload):
        node_id, sensor_id, event = payload
        self.robot.log.info(f"event: {node_id, sensor_id, event}")
        if node_id == _JOINT_MAP[self.joint]+1 and sensor_id == 10 and event == 3:
            self.homing_state = "homed"
            self.robot.log.info(f"MCU homed")

    def home(self, approach, distance):
        with self.robot.blocking_mode():
            self.robot.jog_joint(joint=self.joint, distance=approach)
        self.robot.set_mcu_homing(self.joint)
        self.robot.jog_joint(joint=self.joint, distance=distance)
        while self.robot.state != "moving":
            time.sleep(0.1)
        while (self.homing_state != "homed") and (self.robot.state != "holding"):
            time.sleep(0.1)
        if self.homing_state != "homed":
            self.robot.error(info=f"Homing {self.joint} failed")


class ImpedanceHoming:
    def __init__(self, robot_api: "RobotAPI", joint: JointName):
        self.robot = robot_api
        self.joint = joint
        self.prev_pos = 0

    def home(self):
        self.robot.jog_joint(joint=self.joint, distance="-")
        while self.robot.state != "moving":
            time.sleep(0.1)
        while abs(self.robot.act_target.joints[_JOINT_MAP[self.joint]] - self.prev_pos) > 0.1:
            self.prev_pos = self.robot.act_target.joints[_JOINT_MAP[self.joint]]
            time.sleep(1)
        self.robot.set_zero(self.joint)


class HomingSkills:
    """High-Level Skill Library. Mixin to RobotAPI"""

    def home(self: "RobotAPI") -> None:
        """Homing sequence for Joint 6 and Gripper.

        Examples:
            >>> robot.home()
        """
        with self.blocking_mode():
            self.overwrite(1.0)
            self.moveJ([None, 63, 200, None, 0, None])

        joint6 = SensorHoming(self, "j6")
        joint6.home(approach=-45, distance=200)

        with self.blocking_mode():
            self.moveJ([None, None, None, None, None, 0])

        joint7 = ImpedanceHoming(self, "j7")
        joint7.home()

        with self.blocking_mode():
            self.gripper(50)
