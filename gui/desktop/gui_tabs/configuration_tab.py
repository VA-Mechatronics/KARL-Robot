import logging
import PySide6.QtWidgets as qtw
from PySide6.QtCore import Qt, QAbstractTableModel
from client.client_api import RobotAPI, RobotConfig
import numpy as np
from gui.desktop.gui_core.control_fields import MCUConfigPage
from pathlib import Path
from common.workspace_manager import WorkspaceManager

BASE_DIR = Path(__file__).resolve().parents[3]


class BaseConfigTableModel(QAbstractTableModel):
    def __init__(self, config):
        super().__init__()
        self.config = config

    def update_config(self, config):
        self.beginResetModel()
        self.config = config
        self._on_config_updated()
        self.endResetModel()

    def _on_config_updated(self):
        pass

    def _cast_value(self, old_val, new_val):
        if isinstance(old_val, int):
            return int(new_val)
        if isinstance(old_val, float):
            return float(new_val)
        return new_val


class RobotConfigTableModel(BaseConfigTableModel):
    def __init__(self, config):
        super().__init__(config)
        self.param_names = list(config.axes[0].__dict__.keys()) if config.axes else []
        self.param_tooltips = {
            "can_node_id": "of MCU",
            "home_offset": "deg",
            "mot_gear_ratio": "used for Feed Forward",
            "enc_gear_ratio": "if encoder on motor instead of gear",
            "nom_velocity": "deg/s",
            "max_velocity": "deg/s",
            "min_pos": "deg",
            "max_pos": "deg",
            "kp": "Proportional gain",
            "ki": "Integral gain",
            "kd": "Derivative gain",
            "kv": "Feed Forward gain",
            "micro_step": "8, 16, 32, 64, ...",
            "max_current": "mA",
            "guiding_gain": "higher = weaker",
            "guiding_deadzone": "mdeg",
            "error_limit": "value",
        }

    def _on_config_updated(self):
        if self.config.axes:
            self.param_names = list(self.config.axes[0].__dict__.keys())

    def rowCount(self, parent=None):
        return len(self.param_names)

    def columnCount(self, parent=None):
        return len(self.config.axes)

    def data(self, index, role):
        if index.isValid() and role in (Qt.DisplayRole, Qt.EditRole):
            axis = self.config.axes[index.column()]
            return getattr(axis, self.param_names[index.row()])
        return None

    def setData(self, index, value, role):
        if role != Qt.EditRole or not index.isValid():
            return False

        axis = self.config.axes[index.column()]
        param = self.param_names[index.row()]
        try:
            old_val = getattr(axis, param)
            setattr(axis, param, self._cast_value(old_val, value))
            self.dataChanged.emit(index, index, [Qt.DisplayRole])
            return True
        except (ValueError, TypeError):
            return False

    def flags(self, index):
        return Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable if index.isValid() else Qt.NoItemFlags

    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            return f"Axis {section + 1}" if orientation == Qt.Horizontal else self.param_names[section]
        if role == Qt.ToolTipRole and orientation == Qt.Vertical:
            return self.param_tooltips.get(self.param_names[section], "")
        return None


class DataclassKeyValTableModel(BaseConfigTableModel):
    def __init__(self, config, target_attr: str, title_prefix: str = ""):
        super().__init__(config)
        self.target_attr = target_attr
        self.title_prefix = title_prefix
        self.keys = []
        self._on_config_updated()

    def _on_config_updated(self):
        obj = getattr(self.config, self.target_attr, None)
        self.keys = list(obj.__dict__.keys()) if obj else []

    def rowCount(self, parent=None):
        return len(self.keys)

    def columnCount(self, parent=None):
        return 1  # Nur noch eine Spalte für Werte

    def data(self, index, role):
        if not index.isValid() or role not in (Qt.DisplayRole, Qt.EditRole):
            return None

        key = self.keys[index.row()]
        return getattr(getattr(self.config, self.target_attr), key)

    def setData(self, index, value, role):
        if role != Qt.EditRole:
            return False

        obj = getattr(self.config, self.target_attr)
        key = self.keys[index.row()]
        try:
            old_val = getattr(obj, key)
            setattr(obj, key, self._cast_value(old_val, value))
            self.dataChanged.emit(index, index, [Qt.DisplayRole])
            return True
        except (ValueError, TypeError):
            return False

    def flags(self, index):
        if not index.isValid():
            return Qt.NoItemFlags
        return Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable

    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return "Value"
            if orientation == Qt.Vertical:
                key = self.keys[section]
                return f"{self.title_prefix} {key}".strip()
        return None


class Matrix4x4TableModel(BaseConfigTableModel):
    def __init__(self, config, target_attr: str, matrix_attr: str = "matrix"):
        super().__init__(config)
        self.target_attr = target_attr
        self.matrix_attr = matrix_attr

    @property
    def matrix(self) -> np.ndarray:
        return getattr(getattr(self.config, self.target_attr), self.matrix_attr)

    def rowCount(self, parent=None):
        return 4

    def columnCount(self, parent=None):
        return 4

    def data(self, index, role):
        if index.isValid() and role in (Qt.DisplayRole, Qt.EditRole):
            val = self.matrix[index.row(), index.column()]
            return f"{val:.3f}" if role == Qt.DisplayRole else float(val)
        return None

    def setData(self, index, value, role):
        if role != Qt.EditRole or not index.isValid():
            return False
        try:
            self.matrix[index.row(), index.column()] = float(value)
            self.dataChanged.emit(index, index, [Qt.DisplayRole])
            return True
        except (ValueError, TypeError):
            return False

    def flags(self, index):
        return Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsEditable if index.isValid() else Qt.NoItemFlags

    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            headers_h = ["X", "Y", "Z", "P"]
            headers_v = ["R1", "R2", "R3", "H"]
            return headers_h[section] if orientation == Qt.Horizontal else headers_v[section]
        return None


class ConfigurationTab(qtw.QWidget):
    def __init__(self, robot_api: RobotAPI):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.robot = robot_api
        self.ws = WorkspaceManager(BASE_DIR / "workspace")
        try:
            self.config = RobotConfig.read_from_file(str(self.ws.configs_dir / "config.json"))
        except Exception as e:
            self.log.warning(f"config.json not found. Default config loaded. {e}")
            self.config = RobotConfig.default()

        layout = qtw.QVBoxLayout(self)

        # MCU Table
        axis_label = qtw.QLabel("MCU")
        self.table = qtw.QTableView()
        self.model = RobotConfigTableModel(self.config)
        self.table.setModel(self.model)
        self.table.setSizePolicy(qtw.QSizePolicy.Expanding, qtw.QSizePolicy.Expanding)
        self.table.setMaximumHeight(500)

        layout.addWidget(axis_label)
        layout.addWidget(self.table, 1)

        self.mcu_config_page = MCUConfigPage(range(1, 8), self.robot)
        self.mcu_config_page.layout.setContentsMargins(115, 0, 0, 0)
        layout.addLayout(self.mcu_config_page.layout)

        # Mechanical Container Layout
        mech_container = qtw.QHBoxLayout()

        # Link Table
        link_box = qtw.QVBoxLayout()
        link_box.addWidget(qtw.QLabel("Link Parameters"))
        self.link_table = qtw.QTableView()
        self.link_model = DataclassKeyValTableModel(self.config, target_attr="link")
        self.link_table.setModel(self.link_model)
        self.link_table.setFixedHeight(180)
        self.link_table.setMaximumWidth(115)
        self.link_table.horizontalHeader().setSectionResizeMode(qtw.QHeaderView.Stretch)
        link_box.addWidget(self.link_table)

        # Tool Physical Matrix Table
        tool_box = qtw.QVBoxLayout()
        tool_box.addWidget(qtw.QLabel("Tool Physical Matrix (4x4)"))
        self.tool_table = qtw.QTableView()
        self.tool_model = Matrix4x4TableModel(self.config, target_attr="tool", matrix_attr="matrix")
        self.tool_table.setModel(self.tool_model)
        self.tool_table.setFixedHeight(180)
        self.tool_table.horizontalHeader().setSectionResizeMode(qtw.QHeaderView.Stretch)
        tool_box.addWidget(self.tool_table)

        # Virtual TCP Offset Matrix Table
        tcp_offset_box = qtw.QVBoxLayout()
        tcp_offset_box.addWidget(qtw.QLabel("Virtual TCP Offset (4x4)"))
        self.tcp_offset_table = qtw.QTableView()
        self.tcp_offset_model = Matrix4x4TableModel(self.config, target_attr="tcp_offset", matrix_attr="matrix")
        self.tcp_offset_table.setModel(self.tcp_offset_model)
        self.tcp_offset_table.setFixedHeight(180)
        self.tcp_offset_table.horizontalHeader().setSectionResizeMode(qtw.QHeaderView.Stretch)
        tcp_offset_box.addWidget(self.tcp_offset_table)

        mech_container.addLayout(link_box, 1)
        mech_container.addLayout(tool_box, 2)
        mech_container.addLayout(tcp_offset_box, 2)
        layout.addLayout(mech_container)

        # Buttons
        control_panel = qtw.QHBoxLayout()
        self.btn_read_from_file = qtw.QPushButton("Read from File")
        self.btn_write_to_file = qtw.QPushButton("Write to File")
        self.btn_read_from_mcu = qtw.QPushButton("Read from MCU")
        self.btn_write_to_mcu = qtw.QPushButton("Write to MCU")
        control_panel.addWidget(self.btn_read_from_file)
        control_panel.addWidget(self.btn_write_to_file)
        control_panel.addWidget(self.btn_read_from_mcu)
        control_panel.addWidget(self.btn_write_to_mcu)

        self.btn_read_from_file.pressed.connect(self.read_from_file)
        self.btn_write_to_file.pressed.connect(self.write_to_file)
        self.btn_read_from_mcu.pressed.connect(self.read_from_mcu)
        self.btn_write_to_mcu.pressed.connect(self.write_to_mcu)
        layout.addLayout(control_panel)

        self.robot.on("config", self._on_config_from_api)

    def _update_all_models(self):
        self.model.update_config(self.config)
        self.link_model.update_config(self.config)
        self.tool_model.update_config(self.config)
        self.tcp_offset_model.update_config(self.config)

    def read_from_file(self):
        path, _ = qtw.QFileDialog.getOpenFileName(self, "Load Config", str(self.ws.configs_dir), "JSON (*.json)")
        if path:
            self.config = RobotConfig.read_from_file(path)
            self._update_all_models()
        win = self.window()
        if win and hasattr(win, "show_message"):
            win.show_message(f"Config loaded from {path}")

    def write_to_file(self):
        path, _ = qtw.QFileDialog.getSaveFileName(self, "Save Config", str(self.ws.configs_dir), "JSON (*.json)")
        if path:
            self.config.write_to_file(path)
        win = self.window()
        if win and hasattr(win, "show_message"):
            win.show_message(f"Config saved to {path}")

    def read_from_mcu(self):
        res = self.robot.read_config()
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"Config read from MCU." if isinstance(res, RobotConfig) else f"Config read from MCU FAILED."
            win.show_message(msg)

    def write_to_mcu(self):
        res = self.robot.write_config(self.config)
        win = self.window()
        if win and hasattr(win, "show_message"):
            msg = f"Config sent to MCU." if res else f"Config sent to MCU FAILED."
            win.show_message(msg)

    def _on_config_from_api(self, payload):
        self.config = RobotConfig.from_dict(payload).copy()
        self._update_all_models()
        win = self.window()
        if win and hasattr(win, "show_message"):
            win.show_message("Config updated")
