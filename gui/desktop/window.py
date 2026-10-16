import logging
import PySide6.QtWidgets as qtw
from PySide6.QtGui import QIcon, QRegularExpressionValidator
from PySide6.QtCore import Qt, QRegularExpression
import PySide6.QtCore as qtc

from gui.desktop.gui_tabs.home_tab import HomeTab
from gui.desktop.gui_tabs.model_tab import ModelTab
from gui.desktop.gui_tabs.telemetry_tab import TelemetryTab
from gui.desktop.gui_tabs.configuration_tab import ConfigurationTab
from gui.desktop.gui_tabs.control_tab import ControlTab
from gui.desktop.gui_tabs.automatic_tab import AutomaticTab

from client.client_api import RobotAPI
import platform
import subprocess
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[2]


class MainWindow(qtw.QWidget):
    def __init__(self, robot_api: RobotAPI):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.robot = robot_api
        self.sim_backend = None

        self.setWindowTitle("KARL Control")
        path = BASE_DIR / "gui" / "desktop" / "img" / "icon.png"
        self.setWindowIcon(QIcon(str(path)))
        # self.resize(500, 400)
        layout = qtw.QGridLayout()
        # layout.setColumnMinimumWidth(1, 640)

        # region *** Conn Panel ***
        conn_layout = qtw.QHBoxLayout()
        label_ip = qtw.QLabel("Backend IP:")
        label_ip.setFixedWidth(100)
        conn_layout.addWidget(label_ip)
        self.ip_combo = qtw.QComboBox()
        self.ip_combo.setEditable(True)
        self.ip_combo.addItems(["192.168.77.2", "localhost", "Simulation"])
        # "localhost" or IP-Format (0-255.0-255.0-255.0-255)
        ip_regex = QRegularExpression(
            r"^(Simulation|localhost|((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?))$"
        )
        validator = QRegularExpressionValidator(ip_regex, self.ip_combo)
        self.ip_combo.setValidator(validator)
        conn_layout.addWidget(self.ip_combo)
        self.btn_connect = qtw.QPushButton("Connect")
        self.btn_connect.setFixedWidth(210)
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        conn_layout.addWidget(self.btn_connect)
        # endregion
        layout.addLayout(conn_layout, 0, 0, 1, 1)

        # region *** State Panel ***
        state_panel = qtw.QHBoxLayout()
        label_state = qtw.QLabel("State")
        label_state.setFixedWidth(107)
        state_panel.addWidget(label_state)
        self.label_status = qtw.QLabel("Backend not connected")
        state_panel.addWidget(self.label_status)
        self.alive = qtw.QProgressBar()
        self.alive.setRange(0, 0)
        self.alive.setFixedWidth(100)
        state_panel.addWidget(self.alive)
        # endregion
        layout.addLayout(state_panel, 1, 0, 1, 1)

        # region *** Stop Panel ***
        stop_panel = qtw.QHBoxLayout()
        self.btn_homing = qtw.QPushButton("🏠 Start Homing")
        self.btn_homing.setFixedWidth(210)
        stop_panel.addWidget(self.btn_homing)
        self.btn_guide = qtw.QPushButton("🧤 Guide")
        self.btn_guide.setFixedWidth(210)
        stop_panel.addWidget(self.btn_guide)
        self.btn_pwr_mcu = qtw.QPushButton("⚡ Power off MCU")
        self.btn_pwr_mcu.setFixedWidth(210)
        stop_panel.addWidget(self.btn_pwr_mcu)
        self.btn_stop = qtw.QPushButton("Stop")
        self.btn_stop.setStyleSheet("""
                    QPushButton {
                        background-color: #FF3333;   /* Pastellrot */
                        border: 1px solid #cc6666;   /* dunklerer Rand */
                        border-radius: 5px;
                        padding: 5px;
                    }
                    QPushButton:hover {
                        background-color: #FF5555;   /* Hover-Effekt */
                    }
                    QPushButton:pressed {
                        background-color: #FF1111;   /* gedrückt */
                    }
                """)
        self.btn_stop.setFixedWidth(210)
        stop_panel.addWidget(self.btn_stop)
        self.btn_homing.pressed.connect(lambda: self.robot.run_program("run_homing.py"))
        self.btn_guide.pressed.connect(lambda: self.robot.guide())
        self.btn_pwr_mcu.pressed.connect(lambda: self.robot.power_off())
        self.btn_stop.pressed.connect(lambda: self.robot.pause_program())
        # endregion
        layout.addLayout(stop_panel, 0, 1, 1, 1)

        # region *** Overwrite Panel ***
        overwrite_panel = qtw.QHBoxLayout()
        label_overwrite = qtw.QLabel("Speed Overwrite")
        label_overwrite.setFixedWidth(100)
        overwrite_panel.addWidget(label_overwrite)
        self.overwrite = qtw.QSlider(Qt.Orientation.Horizontal)
        self.overwrite.setMinimum(0)
        self.overwrite.setMaximum(200)
        self.overwrite.setValue(100)
        self.overwrite.setTickPosition(qtw.QSlider.TickPosition.TicksAbove)
        self.overwrite.setTickInterval(50)
        self.overwrite.setFixedWidth(600)
        overwrite_panel.addWidget(self.overwrite)
        self.overwrite.sliderReleased.connect(lambda: self.robot.overwrite(float(self.overwrite.value())/100.))
        # endregion
        layout.addLayout(overwrite_panel, 1, 1, 1, 1)

        # region *** Left Tabs ***
        left_tabs = qtw.QTabWidget()
        left_tabs.addTab(HomeTab(), "🏚️ Home")
        self.model_tab = ModelTab(self.robot)
        left_tabs.addTab(self.model_tab, "🧩 3D Model")
        self.telemetry_tab = TelemetryTab(self.robot)
        left_tabs.addTab(self.telemetry_tab, "📈 Telemetry")
        # endregion
        layout.addWidget(left_tabs, 2, 0, 1, 1)

        # region *** Right Tabs ***
        right_tabs = qtw.QTabWidget()
        self.configuration_tab = ConfigurationTab(self.robot)
        right_tabs.addTab(self.configuration_tab, "🛠️ Config")
        self.control_tab = ControlTab(self.robot)
        right_tabs.addTab(self.control_tab, "🖐️ Control")
        self.automatic_tab = AutomaticTab(self.robot)
        right_tabs.addTab(self.automatic_tab, "💫 Automatic")
        right_tabs.setFixedWidth(855)
        # endregion
        layout.addWidget(right_tabs, 2, 1, 1, 1)

        # region *** Footer Panel ***
        footer_panel = qtw.QHBoxLayout()
        self.footer_info = qtw.QLabel("")
        footer_panel.addWidget(self.footer_info)
        footer_panel.addStretch()
        # endregion
        layout.addLayout(footer_panel, 3, 0, 1, 2)

        self.setLayout(layout)

        self.log.info("GUI started")

        self.robot.on("state", self._on_state_from_api)
        self.robot.on("overwrite", self._on_overwrite_from_api)

        self.control_tab.joint_position_page_object.on_set_positions_changed = self.model_tab.update_set_positions
        self.control_tab.base_position_page_object.on_set_positions_changed = self.model_tab.update_set_positions

    def show_message(self, message: str, timeout_ms: int = 5000) -> None:
        self.log.info(f"{message}")
        if qtc.QThread.currentThread() != qtw.QApplication.instance().thread():
            qtc.QMetaObject.invokeMethod(
                self,
                "_apply_show_message",
                qtc.Qt.QueuedConnection,
                qtc.Q_ARG(str, message),
                qtc.Q_ARG(int, timeout_ms)
            )
            return
        self._apply_show_message(message, timeout_ms)

    @qtc.Slot(str, int)
    def _apply_show_message(self, message: str, timeout_ms: int) -> None:
        self.footer_info.setText(message)
        qtc.QTimer.singleShot(timeout_ms, lambda: self.footer_info.setText(""))

    def _on_state_from_api(self, payload, info=None):
        msg = f"{payload}" if info is None else f"{payload}: {info}"
        self.label_status.setText(msg)

    def _on_overwrite_from_api(self, payload):
        self.overwrite.setValue(payload * 100)

    def _start_sim_backend(self):
        self._stop_sim_backend()
        file_path = BASE_DIR / "backend" / "main.py"
        if platform.system() == "Windows":
            self.sim_backend = subprocess.Popen(
                ["cmd.exe", "/K", sys.executable, str(file_path)],
                creationflags=subprocess.CREATE_NEW_CONSOLE,
            )
        elif platform.system() == "Linux":
            self.sim_backend = subprocess.Popen(
                ["x-terminal-emulator", "-e", sys.executable, str(file_path)],
            )

    def _stop_sim_backend(self):
        if self.sim_backend is None:
            return
        if self.sim_backend.poll() is None:
            if platform.system() == "Windows":
                subprocess.run(
                    ["taskkill", "/PID", str(self.sim_backend.pid), "/T", "/F"],
                )
            elif platform.system() == "Linux":
                try:
                    self.sim_backend.terminate()
                    self.sim_backend.wait(timeout=3)

                except subprocess.TimeoutExpired:
                    self.sim_backend.kill()
                    self.sim_backend.wait()
        self.sim_backend = None

    def _on_connect_clicked(self):
        selected_ip = self.ip_combo.currentText()
        if not selected_ip:
            return
        delay = 100
        if selected_ip in ["Simulation"]:
            selected_ip = "localhost"
            self._start_sim_backend()
            delay = 10000
        self.robot.disconnect()
        self.label_status.setText(f"Connecting to {selected_ip}...")
        qtw.QApplication.processEvents()
        qtc.QTimer.singleShot(delay, lambda: self.robot.connect(selected_ip))
        qtc.QTimer.singleShot(delay+500,  lambda: self.control_tab.joint_position_page_object.set_from_actual())

    def closeEvent(self, event):
        self._stop_sim_backend()
        self.robot.disconnect()
        event.accept()
