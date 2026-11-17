from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from backend.controller.robot_controller import RobotController
from backend.controller.states.base_state import BaseState

from common.robot_config import RobotConfig


class BootState(BaseState):
    name = "boot"

    def enter(self, controller: RobotController, info=None):
        super().enter(controller, info)
        controller.traj_cleanup()
        try:
            config = RobotConfig.read_from_file(controller.workspace.configs_dir / "config.json")
            controller.handle_sync_cmd({"cmd": "write_config", "payload": config.to_dict()}, False)
        except Exception as e:
            controller.log.warning(f"config.json not found. MCUs not configured. {e}")

    def run(self, controller: RobotController):
        controller.set_state("idle")

    def exit(self, controller):
        pass
    