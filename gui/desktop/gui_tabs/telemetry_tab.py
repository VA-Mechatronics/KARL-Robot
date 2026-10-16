import logging
import threading
import time
import collections
import subprocess
import sys
from pathlib import Path
import numpy as np
import PySide6.QtWidgets as qtw
import PySide6.QtCore as qtc
from PySide6.QtCore import Qt

import pyqtgraph as pg
from client.client_api import RobotAPI
from common.mcu_telemetry import RobotTelemetry
from tools.telemetry_logger import TelemetryLoggerService

PROJECT_ROOT = Path(__file__).resolve().parents[3]


class TelemetryTab(qtw.QWidget):
    def __init__(self, robot_api: RobotAPI):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.robot = robot_api

        self._logger_service: TelemetryLoggerService | None = None
        self._logger_thread: threading.Thread | None = None
        self._data_buffer = collections.deque(maxlen=1000)

        layout = qtw.QVBoxLayout()
        colors = ['r', 'g', 'b', 'y', 'c', 'm', 'w']

        # region *** Plot 1: Position Live Plot (Actual vs Set) ***
        self.plot_widget_1 = pg.PlotWidget(title="")
        self.plot_widget_1.enableAutoRange(axis='y', enable=False)
        self.plot_widget_1.setYRange(-330, 330)
        axis1 = self.plot_widget_1.getAxis('left')
        axis1.setTickSpacing(major=30, minor=5)

        self.legend_1 = pg.LegendItem(colCount=7)
        self.legend_1.setParentItem(self.plot_widget_1.graphicsItem())
        self.legend_1.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 0))

        self.curves_1_act = []
        self.curves_1_set = []

        # Actual positions (solid lines)
        for i in range(7):
            curve = self.plot_widget_1.plot(pen=pg.mkPen(colors[i], width=2, style=Qt.SolidLine), name=f"J{i+1} Act")
            self.curves_1_act.append(curve)
            self.legend_1.addItem(curve, f"J{i + 1} Act")

        # Target/Set positions (dashed lines)
        for i in range(7):
            curve = self.plot_widget_1.plot(pen=pg.mkPen(colors[i], width=2, style=Qt.DashLine), name=f"J{i+1} Set")
            self.curves_1_set.append(curve)
            self.legend_1.addItem(curve, f"J{i + 1} Set")

        layout.addWidget(self.plot_widget_1)
        # endregion

        # region *** Plot 2: Debug Variable Live Plot ***
        self.plot_widget_2 = pg.PlotWidget(title="")
        self.plot_widget_2.enableAutoRange(axis='y', enable=True)

        self.legend_2 = pg.LegendItem(colCount=7)
        self.legend_2.setParentItem(self.plot_widget_2.graphicsItem())
        self.legend_2.anchor(itemPos=(1, 0), parentPos=(1, 0), offset=(-10, 0))

        self.curves_2 = []
        for i in range(7):
            curve = self.plot_widget_2.plot(pen=pg.mkPen(colors[i], width=2, style=Qt.SolidLine, name=f"J{i+1} val"))
            self.curves_2.append(curve)
            self.legend_2.addItem(curve, f"J{i + 1} val")

        layout.addWidget(self.plot_widget_2)
        # endregion

        # region *** Control Buttons Bar ***
        btn_layout = qtw.QHBoxLayout()

        self.btn_start_logger = qtw.QPushButton("Start Telemetry Logger")
        self.btn_stop_logger = qtw.QPushButton("Stop Telemetry Logger")
        self.btn_analyze = qtw.QPushButton("Analyze Telemetry")

        self.btn_stop_logger.setEnabled(False)

        self.btn_start_logger.clicked.connect(self._start_telemetry_logger)
        self.btn_stop_logger.clicked.connect(self._stop_telemetry_logger)
        self.btn_analyze.clicked.connect(self._analyze_telemetry)

        btn_layout.addWidget(self.btn_start_logger)
        btn_layout.addWidget(self.btn_stop_logger)
        btn_layout.addWidget(self.btn_analyze)

        layout.addLayout(btn_layout)
        # endregion

        self.setLayout(layout)

        # Connect ZMQ callback (Producer)
        self.robot.on("telemetry", self._on_telemetry_from_api)

        # Setup GUI Refresh Loop Timer in Main Thread (Consumer: 20 Hz = 50ms)
        self._render_timer = qtc.QTimer(self)
        self._render_timer.setInterval(100)
        self._render_timer.timeout.connect(self._update_gui_plots)
        self._render_timer.start()

    def _start_telemetry_logger(self) -> None:
        """Instantiate and start logger service directly in a background thread."""
        if self._logger_thread and self._logger_thread.is_alive():
            return
        pub_addr = self.robot.sub_addr
        self._logger_service = TelemetryLoggerService(sub_addr=pub_addr)
        self._logger_thread = threading.Thread(target=self._logger_service.start, daemon=True)
        self._logger_thread.start()
        self.btn_start_logger.setEnabled(False)
        self.btn_stop_logger.setEnabled(True)
        win = self.window()
        if win and hasattr(win, "show_message"):
            win.show_message(f"Telemetry Logger started ({pub_addr})")

    def _stop_telemetry_logger(self) -> None:
        if self._logger_service is not None:
            self.log.info("Stopping telemetry logger service...")
            self._logger_service.stop()

            if self._logger_thread:
                self._logger_thread.join(timeout=2.0)
                self._logger_thread = None
            self._logger_service = None

        self.btn_start_logger.setEnabled(True)
        self.btn_stop_logger.setEnabled(False)

        win = self.window()
        if win and hasattr(win, "show_message"):
            win.show_message("Telemetry Logger stopped and saved Parquet file.")

    def _analyze_telemetry(self) -> None:
        analyzer_script = PROJECT_ROOT / "tools" / "analyze_telemetry.py"
        self.log.info("Launching Plotly telemetry analyzer...")
        subprocess.Popen([sys.executable, str(analyzer_script)])

        win = self.window()
        if win and hasattr(win, "show_message"):
            win.show_message("Opening interactive Plotly trace in browser...")

    def _on_telemetry_from_api(self, payload: dict) -> None:
        try:
            telemetry = RobotTelemetry.from_dict(payload)
            timestamp = time.time()

            act_positions = [axis.act_pos for axis in telemetry.axes]
            set_positions = [axis.set_pos for axis in telemetry.axes]

            """Extract selected attribute for Plot 2"""
            debug_values = [axis.err for axis in telemetry.axes]

            # Append snapshot to thread-safe buffer
            self._data_buffer.append((timestamp, act_positions, set_positions, debug_values))
        except Exception as e:
            self.log.error(f"Error parsing telemetry payload: {e}")

    @qtc.Slot()
    def _update_gui_plots(self) -> None:
        if len(self._data_buffer) < 2:
            return

        # Snapshot current buffer
        snapshot = list(self._data_buffer)
        timestamps = np.array([item[0] for item in snapshot])
        act_data = np.array([item[1] for item in snapshot])  # Shape: (N, 7)
        set_data = np.array([item[2] for item in snapshot])  # Shape: (N, 7)
        debug_data = np.array([item[3] for item in snapshot])  # Shape: (N, 7)

        # Relative time axis in seconds
        t_axis = timestamps - timestamps[0]

        # Update Plot 1: Actual vs Target
        for axis_idx in range(min(7, act_data.shape[1])):
            self.curves_1_act[axis_idx].setData(t_axis, act_data[:, axis_idx])
            self.curves_1_set[axis_idx].setData(t_axis, set_data[:, axis_idx])

        # Update Plot 2: Selected Debug Metric
        for axis_idx in range(min(7, debug_data.shape[1])):
            self.curves_2[axis_idx].setData(t_axis, debug_data[:, axis_idx])
