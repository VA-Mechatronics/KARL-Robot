from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from backend.controller.robot_controller import RobotController
from backend.controller.states.base_state import BaseState


class GuidingState(BaseState):
    name = "guiding"

    def enter(self, controller: RobotController, info=None):
        super().enter(controller, info)
        controller.hw.set_enable(True)
        controller.hw.set_mode(1)

    def run(self, controller: RobotController):
        controller.hw.update_telemetry()

    def exit(self, controller: RobotController):
        controller.hw.set_mode(0)
