from __future__ import annotations

import logging
import sys
import time
from typing import TYPE_CHECKING
from enum import IntEnum

if TYPE_CHECKING:
    from backend.controller.robot_controller import RobotController
if sys.platform.startswith("linux"):
    import subprocess


class SmartEvent(IntEnum):
    NONE = 0
    SINGLE = 1
    DOUBLE = 2
    LONG = 3
    RELEASE = 4


class IOController:
    def __init__(self, controller: RobotController):
        self.log = logging.getLogger(__name__)
        self.controller = controller
        self.last_event = time.time()

    def update(self) -> None:
        if self._check_e_stop() and self.controller.sm.state.name != "error":
            self.controller.set_state("error", "E-Stop active")

    def _check_e_stop(self) -> bool:
        if sys.platform.startswith("linux"):
            res = subprocess.run(['pinctrl', 'get', '6'], capture_output=True, text=True)   # Raspberrypi GPIO read-in
            return not ('hi' in res.stdout)
        else:
            return False

    def handle_input_event(self, can_node_id: int, input_id: int, smart_event: SmartEvent) -> None:
        self.log.info(f"input event, can_node_id:{can_node_id}, input_id:{input_id}, event:{smart_event}")
        # Special case: Tool MCU
        if can_node_id == 7:
            match input_id:
                case 1:     # FUN
                    self.controller.publish_smart_input(can_node_id, input_id, smart_event)
                    if smart_event == SmartEvent.LONG:
                        if self.controller.sm.state.name == "holding":
                            self.controller.handle_sync_cmd({"cmd": "guide"}, False)
                        else:
                            self.controller.handle_cmd({"cmd": "stop"})
                    if smart_event == SmartEvent.SINGLE:
                        self.controller.handle_sync_cmd({"cmd": "record_target", "type": "moveJ"}, False)
                    if smart_event == SmartEvent.DOUBLE:
                        self.controller.handle_sync_cmd({"cmd": "record_target", "type": "moveL"}, False)
                    self.controller.hw.set_smart_output(can_node_id, 1, smart_event)
                case 2:     # CLS
                    if smart_event == SmartEvent.LONG:
                        self.controller.handle_cmd(
                            {"cmd": "jog", "frame": "joint", "axis": 6, "direction": -1, "distance": None}
                        )
                    if smart_event == SmartEvent.RELEASE:
                        self.controller.handle_cmd({"cmd": "stop"})
                case 3:     # OPN
                    if smart_event == SmartEvent.LONG:
                        self.controller.handle_cmd(
                            {"cmd": "jog", "frame": "joint", "axis": 6, "direction": 1, "distance": None}
                        )
                    if smart_event == SmartEvent.RELEASE:
                        self.controller.handle_cmd({"cmd": "stop"})
        # General
        else:
            self.controller.publish_smart_input(can_node_id, input_id, smart_event)


if __name__ == "__main__":
    pass
