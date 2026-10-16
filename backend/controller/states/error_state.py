from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from backend.controller.robot_controller import RobotController
from backend.controller.states.base_state import BaseState


class ErrorState(BaseState):
    name = "error"

    def enter(self, controller: RobotController, info=None):
        super().enter(controller, info)
        controller.hw.hard_stop(set_enable=True)

    def run(self, controller):
        controller.hw.update_telemetry()

    def exit(self, controller):
        pass

