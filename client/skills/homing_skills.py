import time
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from client.client_api import RobotAPI


class HomingManager:
    def __init__(self, robot_api: "RobotAPI"):
        self.robot = robot_api
        self.homing_state = None
        self.prev_pos = 0
        self.robot.on("smart_input", self._on_homing_from_api)

    def _on_homing_from_api(self, payload):
        node_id, sensor_id, event = payload
        self.robot.log.info(f"{node_id, sensor_id, event}")
        if node_id == 6 and sensor_id == 10:
            self.homing_state = event

    def start_homing(self):
        with self.robot.blocking_mode():
            self.robot.log.info("Start Homing...")
            self.robot.overwrite(1.0)
            self.home_mcu_6()
            self.home_gripper()
            self.robot.read_config()
            self.robot.log.info("Homing finished")

    def home_mcu_6(self):
        self.robot.log.info("Approach Homing Position...")
        with self.robot.blocking_mode():
            self.robot.moveJ([None, 63, 200, None, 0, None])
            self.robot.jog_joint(joint="j6", distance=-20.0)
        with self.robot.non_blocking_mode():
            self.robot.set_mcu_homing("j6")
            self.robot.jog_joint(joint="j6", distance=120.0)
            self.homing_state = None
            while self.homing_state != 3 and self.robot.state != "holding":
                time.sleep(0.5)
                if self.robot.state == "holding":
                    self.robot.log.error("Homing failed")
                    self.robot.error(info="Homing failed")
            self.robot.log.info("MCU6 homed")
        with self.robot.blocking_mode():
            self.robot.moveJ([None, None, None, None, None, 0])

    def home_gripper(self):
        with self.robot.non_blocking_mode():
            self.robot.jog_joint(joint="j7", distance="-")
            while abs(self.robot.act_target.joints[6] - self.prev_pos) > 1.0:
                self.prev_pos = self.robot.act_target.joints[6]
                time.sleep(0.5)
            self.robot.log.info("Gripper homed")
        with self.robot.blocking_mode():
            self.robot.stop()
            self.robot.set_zero("j7")
            self.robot.gripper(50)


class HomingSkills:
    """High-Level Skill Library. Mixin to RobotAPI"""

    def home(self: "RobotAPI") -> None:
        """Homing sequence for Joint 6 and Gripper.

        Examples:
            >>> robot.home()
        """
        homing_manager = HomingManager(self)
        homing_manager.start_homing()
