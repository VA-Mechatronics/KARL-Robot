import logging
from pathlib import Path
import PySide6.QtWidgets as qtw
import PySide6.QtCore as qtc
from client.client_api import RobotAPI
from common.workspace_manager import WorkspaceManager

BASE_DIR = Path(__file__).resolve().parents[3]


class AutomaticTab(qtw.QWidget):
    def __init__(self, robot_api: RobotAPI):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.robot = robot_api

        self.ws = WorkspaceManager(BASE_DIR / "workspace")

        layout = qtw.QVBoxLayout()

        # region *** Section 1: Program Selection ***
        layout.addWidget(qtw.QLabel("Program Selection"))

        # Program Selector Row
        self.combo_programs = qtw.QComboBox()
        self.btn_refresh_programs = qtw.QPushButton("Refresh")

        prog_select_layout = qtw.QHBoxLayout()
        prog_select_layout.addWidget(self.combo_programs, stretch=2)
        prog_select_layout.addWidget(self.btn_refresh_programs, stretch=1)

        layout.addLayout(prog_select_layout)
        layout.addWidget(qtw.QLabel(""))
        # endregion

        # region *** Section 2: Program Control Bar ***
        layout.addWidget(qtw.QLabel("Program Execution Control"))

        # 1. Start / Restart Controls (Top Row)
        self.btn_restart_once = qtw.QPushButton("Run once")
        self.btn_restart_infinite = qtw.QPushButton("Run infinite")

        start_control_layout = qtw.QHBoxLayout()
        start_control_layout.addWidget(self.btn_restart_once)
        start_control_layout.addWidget(self.btn_restart_infinite)

        # 2. Media-Style Main Controls (Bottom Row)
        self.btn_resume = qtw.QPushButton("Resume")
        self.btn_pause = qtw.QPushButton("Pause")
        self.btn_cancel = qtw.QPushButton("Cancel")

        main_control_layout = qtw.QHBoxLayout()
        main_control_layout.addWidget(self.btn_resume)
        main_control_layout.addWidget(self.btn_pause)
        main_control_layout.addWidget(self.btn_cancel)

        layout.addLayout(start_control_layout)
        layout.addLayout(main_control_layout)
        layout.addWidget(qtw.QLabel(""))
        # endregion

        # region *** Section 3: Guiding / Recorder ***
        layout.addWidget(qtw.QLabel("Guiding / Recorder"))

        self.btn_clear_guide = qtw.QPushButton("Clear run_guide.py")
        self.btn_append_moveJ_guide = qtw.QPushButton("Append Move Joints")
        self.btn_append_moveL_guide = qtw.QPushButton("Append Move Linear")

        layout.addWidget(self.btn_clear_guide)
        layout.addWidget(self.btn_append_moveJ_guide)
        layout.addWidget(self.btn_append_moveL_guide)
        layout.addWidget(qtw.QLabel(""))
        # endregion

        # region *** Section 4: Workspace Sync ***
        layout.addStretch()
        layout.addWidget(qtw.QLabel("Workspace Sync (PC <-> Controller)"))

        self.btn_pull_workspace = qtw.QPushButton("Pull Workspace (PC <- Controller)")
        self.btn_push_workspace = qtw.QPushButton("Push Workspace (PC -> Controller)")

        sync_layout = qtw.QHBoxLayout()
        sync_layout.addWidget(self.btn_pull_workspace)
        sync_layout.addWidget(self.btn_push_workspace)

        layout.addLayout(sync_layout)
        # endregion

        self.setLayout(layout)

        # Initial Program List Populate
        self.refresh_program_list()

        # region *** Signal Connections ***
        # Program Selector Connections
        self.btn_refresh_programs.pressed.connect(self.refresh_program_list)

        # Start / Restart Connections
        self.btn_restart_once.pressed.connect(lambda: self._run_selected_program("1"))
        self.btn_restart_infinite.pressed.connect(lambda: self._run_selected_program("infinite"))

        # Primary Control Bar Connections
        self.btn_resume.pressed.connect(self.resume)
        self.btn_pause.pressed.connect(self.pause)
        self.btn_cancel.pressed.connect(self.cancel)

        # Recorder Connections
        self.btn_clear_guide.pressed.connect(self.clear_guide)
        self.btn_append_moveJ_guide.pressed.connect(self.append_moveJ)
        self.btn_append_moveL_guide.pressed.connect(self.append_moveL)

        # Sync Connections
        self.btn_pull_workspace.pressed.connect(self._on_pull_pressed)
        self.btn_push_workspace.pressed.connect(self._on_push_pressed)

        # Thread-safe ZMQ -> Qt Signal Handling
        self.robot.on("workspace", self._on_zmq_workspace_received)
        # endregion

    # region *** Helper Methods ***
    def resume(self):
        res = self.robot.resume_program()
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"Program resumed" if res else f"Program resume FAILED."
            win.show_message(msg)

    def pause(self):
        res = self.robot.pause_program()
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"Program paused" if res else f"Program pause FAILED."
            win.show_message(msg)

    def cancel(self):
        res = self.robot.cancel_program()
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"Program canceled" if res else f"Program cancel FAILED."
            win.show_message(msg)

    def clear_guide(self):
        res = self.robot.clear_guide()
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"Cleared run_guide.py" if res else f"Clearing run_guide.py FAILED."
            win.show_message(msg)

    def append_moveJ(self):
        res = self.robot.record_guide_target("moveJ")
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"moveJ Target recorded" if res else f"Recording FAILED."
            win.show_message(msg)

    def append_moveL(self):
        res = self.robot.record_guide_target("moveL")
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"moveL Target recorded" if res else f"Recording FAILED."
            win.show_message(msg)

    def refresh_program_list(self) -> None:
        """Refresh local programs list from workspace/programs directory."""
        current_selection = self.combo_programs.currentText()
        self.combo_programs.clear()

        programs = self.ws.list_programs()
        if programs:
            self.combo_programs.addItems(sorted(programs))
            index = self.combo_programs.findText(current_selection)
            if index >= 0:
                self.combo_programs.setCurrentIndex(index)
        win = self.window()
        if win and hasattr(win, "show_message"):
            win.show_message(f"Program list refreshed.")

    def _run_selected_program(self, repeat: str) -> None:
        """Stop running script if active and start selected program."""
        selected_program = self.combo_programs.currentText()
        if selected_program:
            res = self.robot.run_program(selected_program, repeat)
            win = self.window()
            if win and hasattr(win, "show_message"):
                msg = f"Run {selected_program} {repeat} time(s)..." if res else f"Run {selected_program} {repeat} time(s) FAILED."
                win.show_message(msg)
        else:
            win = self.window()
            if win and hasattr(win, "show_message"):
                win.show_message(f"No Program selected.")

    def _on_pull_pressed(self) -> None:
        res = self.robot.pull_workspace()
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"Workspace pulled from controller." if res else f"Workspace pull FAILED."
            win.show_message(msg)

    def _on_push_pressed(self) -> None:
        """Push local workspace to controller."""
        res = self.robot.push_workspace()
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"Workspace pushed to controller." if res else f"Workspace push FAILED."
            win.show_message(msg)

    def _on_zmq_workspace_received(self, b64_payload: str) -> None:
        """ZMQ callback wrapper for thread-safe Qt signal emission."""
        if b64_payload:
            self.ws.from_b64_zip(b64_payload)
            self.refresh_program_list()
            win = self.window()
            if win and hasattr(win, "show_message"):
                win.show_message("Workspace pulled from controller.")
    # endregion
