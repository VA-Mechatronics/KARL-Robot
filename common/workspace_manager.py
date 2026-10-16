import base64
from datetime import datetime
import io
import logging
import shutil
import zipfile
from pathlib import Path
from common.target import Target

BASE_DIR = Path(__file__).resolve().parents[1]


class WorkspaceManager:
    def __init__(self, workspace_dir: Path | None = None):
        self.log = logging.getLogger(__name__)

        self.base_dir = BASE_DIR

        # System Assets (Read-Only source)
        self.system_dir = BASE_DIR / "system_assets"
        self.system_configs_dir = self.system_dir / "configs"
        self.system_targets_dir = self.system_dir / "targets"
        self.system_programs_dir = self.system_dir / "programs"
        self.template_file = self.system_dir / "templates" / "guide_sequence_template.py"

        # Workspace
        self.workspace_dir = workspace_dir or (BASE_DIR / "workspace")
        self.archive_dir = self.workspace_dir.parent / "workspace_archive"
        self.configs_dir = self.workspace_dir / "configs"
        self.targets_dir = self.workspace_dir / "targets"
        self.programs_dir = self.workspace_dir / "programs"

        self.init_workspace()

    def init_workspace(self):
        self.configs_dir.mkdir(parents=True, exist_ok=True)
        self.targets_dir.mkdir(parents=True, exist_ok=True)
        self.programs_dir.mkdir(parents=True, exist_ok=True)

        # Restore Default Configs
        if self.system_configs_dir.exists():
            for sys_config in self.system_configs_dir.glob("*.json"):
                dest = self.configs_dir / sys_config.name
                if not dest.exists():
                    shutil.copy(sys_config, dest)

        # Restore Default Targets
        if self.system_targets_dir.exists():
            for sys_target in self.system_targets_dir.glob("*.json"):
                dest = self.targets_dir / sys_target.name
                if not dest.exists():
                    shutil.copy(sys_target, dest)

        # Restore Default Programs
        if self.system_programs_dir.exists():
            for sys_prog in self.system_programs_dir.glob("*.py"):
                dest = self.programs_dir / sys_prog.name
                if not dest.exists():
                    shutil.copy(sys_prog, dest)

    def save_target(self, name: str, target: Target) -> Path:
        if not name.endswith(".json"):
            name += ".json"
        target_path = self.targets_dir / name
        target.write_to_file(str(target_path))
        return target_path

    def load_targets(self) -> dict[str, Target]:
        targets = {}
        for file_path in self.targets_dir.glob("*.json"):
            try:
                targets[file_path.stem] = Target.read_from_file(str(file_path))
            except Exception as e:
                self.log.error(f"Error loading target {file_path}: {e}")
        return targets

    def list_programs(self) -> list[str]:
        return [f.name for f in self.programs_dir.glob("*.py")]

    def create_backup_archive(self) -> Path | None:
        """Create a timestamped archive copy of the current workspace before overwriting."""
        if not self.workspace_dir.exists() or not any(self.workspace_dir.iterdir()):
            return None
        self.archive_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        target_archive_path = self.archive_dir / f"{timestamp}_workspace"
        try:
            shutil.copytree(self.workspace_dir, target_archive_path)
            self.log.info(f"Workspace backed up to: {target_archive_path}")
            return target_archive_path
        except Exception as e:
            self.log.error(f"Failed to create workspace archive: {e}")
            return None

    def to_b64_zip(self) -> str:
        self.init_workspace()
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as zip_file:
            for file_path in self.workspace_dir.rglob("*"):
                if file_path.is_file():
                    arcname = file_path.relative_to(self.workspace_dir)
                    zip_file.write(file_path, arcname)
        return base64.b64encode(buffer.getvalue()).decode("utf-8")

    def from_b64_zip(self, b64_payload: str):
        self.create_backup_archive()

        zip_bytes = base64.b64decode(b64_payload)
        buffer = io.BytesIO(zip_bytes)

        shutil.rmtree(self.configs_dir, ignore_errors=True)
        shutil.rmtree(self.targets_dir, ignore_errors=True)
        shutil.rmtree(self.programs_dir, ignore_errors=True)

        with zipfile.ZipFile(buffer, "r") as zip_file:
            zip_file.extractall(self.workspace_dir)

        self.init_workspace()

    def get_program_path(self, program_name: str) -> Path:
        return self.programs_dir / program_name
