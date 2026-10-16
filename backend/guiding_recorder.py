import logging
import shutil
import time
from common.target import Target
from common.workspace_manager import WorkspaceManager


class GuidingRecorder:
    def __init__(self, workspace: WorkspaceManager):
        self.log = logging.getLogger(__name__)
        self.workspace = workspace
        self.guide_file = self.workspace.programs_dir / "run_guide.py"

    def clear(self):
        template_path = self.workspace.template_file
        if not template_path.exists():
            self.log.error(f"System Template not found at: {template_path}")
            return

        shutil.copy(template_path, self.guide_file)
        self.log.info(f"Reset run_guide.py from system template.")

    def append_cmd(self, cmd_type: str, target: Target):
        if not self.guide_file.exists():
            self.clear()

        target_name = f"guide_{time.time():.0f}"
        self.workspace.save_target(target_name, target)

        cmd_line = f'        robot.{cmd_type}(targets["{target_name}"])\n'

        with open(self.guide_file, "r", encoding="utf-8") as f:
            lines = f.readlines()

        new_lines = []
        for line in lines:
            if "# === GUIDING_COMMANDS_PLACEHOLDER ===" in line:
                new_lines.append(cmd_line)
            new_lines.append(line)

        with open(self.guide_file, "w", encoding="utf-8") as f:
            f.writelines(new_lines)
