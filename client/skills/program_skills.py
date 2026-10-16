from typing import TYPE_CHECKING, TypeAlias, Literal
if TYPE_CHECKING:
    from client.client_api import RobotAPI


RepeatCount: TypeAlias = int | float    # math.inf / float("inf") for infinite
MoveType: TypeAlias = Literal["moveJ", "moveL"]


class ProgramSkills:
    """High-Level Skill Library. Mixin to RobotAPI"""

    def run_program(self: "RobotAPI", file_name: str, repeat: RepeatCount = 1) -> bool:
        """Start Python Program in workspace.

        Args:
            file_name: file_name in workspace dir
            repeat: integer of repeating times or "inf"

        Examples:
            >>> robot.run_program(file_name="example.py", repeat=3)
            >>> robot.run_program(file_name="example.py", repeat=float("inf"))
        """
        self._is_paused = False
        res = self._send_req_cmd({"cmd": "run_program", "program": file_name, "repeat": repeat})
        return res.get("status") == "ok"

    def pause_program(self: "RobotAPI") -> bool:
        """Pause running Program on controller."""
        self._is_paused = True
        res = self._send_req_cmd({"cmd": "pause_program"})
        return res.get("status") == "ok"

    def resume_program(self: "RobotAPI") -> bool:
        """Resume paused Program on controller."""
        self._is_paused = False
        res = self._send_req_cmd({"cmd": "resume_program"})
        return res.get("status") == "ok"

    def cancel_program(self: "RobotAPI") -> bool:
        """Cancel current Program."""
        self._is_paused = False
        res = self._send_req_cmd({"cmd": "cancel_program"})
        return res.get("status") == "ok"

    def clear_guide(self: "RobotAPI") -> bool:
        """Clears Guiding sequence. run_guide.py will be reset to /system_assets/templates/guide_sequence_template.py."""
        res = self._send_req_cmd({"cmd": "clear_guide"})
        return res.get("status") == "ok"

    def record_guide_target(self: "RobotAPI", move_type: MoveType = "moveJ") -> bool:
        """Save act_target to /workspace/targets and append move command on run_guide.py."""
        res = self._send_req_cmd({"cmd": "record_target", "type": move_type})
        return res.get("status") == "ok"
