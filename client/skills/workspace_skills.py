from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from client.client_api import RobotAPI


class WorkspaceSkills:
    """High-Level Skill Library. Mixin to RobotAPI"""

    def push_workspace(self: "RobotAPI", b64_payload: str | None = None) -> bool:
        """Send local workspace directory to controller.

        Args:
            b64_payload: Optional pre-encoded Base64 zip string. If None, automatically
                compresses the local workspace directory via WorkspaceManager.

        Examples:
            >>> # Standard push of active workspace
            >>> robot.push_workspace()
            >>> # Optional push of a custom Base64 payload
            >>> robot.push_workspace(custom_b64_str)
        """
        if b64_payload is None:
            from common.workspace_manager import WorkspaceManager
            ws = WorkspaceManager()
            b64_payload = ws.to_b64_zip()

        res = self._send_req_cmd({"cmd": "push_workspace", "payload": b64_payload})
        return res.get("status") == "ok"

    def pull_workspace(self: "RobotAPI") -> bool:
        """Requests workspace directory from Controller

        Examples:
            >>> robot.pull_workspace()
        """
        res = self._send_req_cmd({"cmd": "pull_workspace"})
        return res.get("status") == "ok"
