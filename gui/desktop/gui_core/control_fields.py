import logging
from common.target import Target
import PySide6.QtWidgets as qtw
from PySide6.QtCore import Signal, QObject
from client.client_api import RobotAPI
from pathlib import Path
from common.workspace_manager import WorkspaceManager

logger = logging.getLogger(__name__)

BASE_DIR = Path(__file__).resolve().parents[3]

_WIDTH = 30
_HEIGHT = 30


# region *** JogPage ***
class JogPage:
    def __init__ (self, frame, field_ids, field_tile, field_names, robot_api: RobotAPI):
        self.frame = frame
        self.page_layout = qtw.QVBoxLayout()
        self.fields = [JogPageField(
            frame=self.frame,
            field_id=field_ids[i], field_name=field_names[i],
            robot_api=robot_api) for i, _ in enumerate(field_ids)]
        self.page_layout.addLayout(JogPageHeader(field_tile).layout)
        for field in self.fields:
            self.page_layout.addLayout(field.layout)
        self.page_layout.addStretch()


class JogPageHeader:
    def __init__(self, title):
        self.layout = qtw.QHBoxLayout()
        self.layout.addWidget(qtw.QLabel(title))
        l1 = qtw.QLabel("Jog")
        l1.setFixedWidth(_WIDTH * 2)
        l1.setFixedHeight(_HEIGHT)
        self.layout.addWidget(l1)


class JogPageField:
    def __init__(self, field_id, field_name, frame, robot_api: RobotAPI):
        self.id = field_id
        self.name = field_name
        self.frame = frame
        self.robot = robot_api

        label_id = qtw.QLabel(f"{self.name}")
        label_id.setFixedHeight(_HEIGHT)
        self.btn_jog_plus = qtw.QPushButton("+")
        self.btn_jog_plus.setFixedWidth(_WIDTH)
        self.btn_jog_plus.setFixedHeight(_HEIGHT)
        self.btn_jog_minus = qtw.QPushButton("-")
        self.btn_jog_minus.setFixedWidth(_WIDTH)

        self.layout = qtw.QHBoxLayout()
        self.layout.addWidget(label_id)
        self.layout.addWidget(self.btn_jog_plus)
        self.layout.addWidget(self.btn_jog_minus)

        # Jog button behaviour (press + hold)
        self.btn_jog_plus.pressed.connect(lambda: (
            self.robot.jog(frame=self.frame, axis=self.id, distance="+"),
        ))
        self.btn_jog_plus.released.connect(lambda: self.robot.stop())

        self.btn_jog_minus.pressed.connect(lambda: (
            self.robot.jog(frame=self.frame, axis=self.id, distance="-"),
        ))
        self.btn_jog_minus.released.connect(lambda: self.robot.stop())
# endregion


# region *** Position Page ***
class PositionPage:
    def __init__(self, frame, field_ids, field_tile, field_names, robot_api: RobotAPI):
        self.frame = frame
        self.robot = robot_api
        self.page_layout = qtw.QVBoxLayout()
        self.fields = [PositionPageField(
            field_id=field_ids[i], field_name=field_names[i], frame=self.frame,
            robot_api=self.robot) for i, _ in enumerate(field_ids)]
        self.page_layout.addLayout(PositionPageHeader(field_tile).layout)
        for field in self.fields:
            self.page_layout.addLayout(field.layout)

        self.page_layout.addStretch()

        self.commands = PositionPageCommands(self.fields, frame, robot_api)

        self.page_layout.addLayout(self.commands.layout)

        # Gui intern event for changed set
        self.on_set_positions_changed = lambda _: None
        for field in self.fields:
            field.set_changed.connect(self._on_set_changed)

    def _on_set_changed(self):
        if self.frame == "joint":
            joints = [f.box_set_pos.value() for f in self.fields]
            self.robot.set_target.joints = joints
            self.robot.set_target.tcp = self.robot.kinematics.joints_to_tcp(joints)
            self.on_set_positions_changed(self.robot.set_target)
        if self.frame == "base":
            pose = [f.box_set_pos.value() for f in self.fields]
            prev_joints = self.robot.act_target.joints
            joints = self.robot.kinematics.pose_to_joints(
                pose, prev_joints,
                shoulder_pref=self.robot.set_target.shoulder,
                elbow_pref=self.robot.set_target.elbow,
                wrist_pref=self.robot.set_target.wrist)
            self.robot.set_target.joints = joints
            self.robot.set_target.tcp = self.robot.kinematics.joints_to_tcp(joints)
            self.on_set_positions_changed(self.robot.set_target)

    def set_from_actual(self):
        for field in self.fields:
            field.set_from_actual()


class PositionPageHeader:
    def __init__(self, title):
        self.layout = qtw.QHBoxLayout()
        l1 = qtw.QLabel(title)
        l1.setFixedHeight(_HEIGHT)
        self.layout.addWidget(l1)
        self.layout.addWidget(qtw.QLabel("Set"))
        self.layout.addWidget(qtw.QLabel("Actual"))


class PositionPageField(QObject):
    set_changed = Signal(int, float)

    def __init__(self, field_id, field_name, frame, robot_api: RobotAPI):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.id = field_id
        self.name = field_name
        self.frame = frame
        self.robot = robot_api

        label_id = qtw.QLabel(f"{self.name}")
        self.box_set_pos = qtw.QDoubleSpinBox()
        self.box_set_pos.setRange(-1000, 1000)
        self.box_set_pos.setFixedHeight(_HEIGHT)
        self.label_act_pos = qtw.QLabel("init")

        self.layout = qtw.QHBoxLayout()
        self.layout.addWidget(label_id)
        self.layout.addWidget(self.box_set_pos)
        self.layout.addWidget(self.label_act_pos)

        # Gui intern event for changed set
        self.box_set_pos.valueChanged.connect(lambda val: self.set_changed.emit(self.id, val))

        self.robot.on("target", self._on_target_from_api)

    def set_from_actual(self):
        self.box_set_pos.setValue(float(self.label_act_pos.text()))

    def _on_target_from_api(self, payload):
        target = Target.from_dict(payload)
        match self.frame:
            case "joint":
                pos = target.joints
                self.label_act_pos.setText(f"{pos[self.id]: .3f}")
            case "base":
                pos = self.robot.kinematics.joints_to_pose(target.joints)
                self.label_act_pos.setText(f"{pos[self.id]: .3f}")


class PositionPageCommands:
    def __init__(self, fields: list[PositionPageField], frame, robot_api: RobotAPI):
        self.fields = fields
        self.frame = frame
        self.robot = robot_api
        self.ws = WorkspaceManager(BASE_DIR / "workspace")

        self.btn_set_from_actual = qtw.QPushButton("Set from Actual")
        self.btn_set_from_actual.setFixedHeight(_HEIGHT)
        self.btn_set_from_file = qtw.QPushButton("Set from File")
        self.btn_set_from_file.setFixedHeight(_HEIGHT)
        self.placeholder = qtw.QLabel("")
        self.placeholder.setFixedHeight(_HEIGHT)
        self.btn_move_joint = qtw.QPushButton("Move Joints")
        self.btn_move_joint.setFixedHeight(_HEIGHT)
        self.btn_move_linear = qtw.QPushButton("Move Linear")
        self.btn_move_linear.setFixedHeight(_HEIGHT)

        self.layout = qtw.QVBoxLayout()
        self.layout.addWidget(self.btn_set_from_actual)
        self.layout.addWidget(self.btn_set_from_file)
        self.layout.addWidget(self.placeholder)
        self.layout.addWidget(self.btn_move_joint)
        self.layout.addWidget(self.btn_move_linear)

        self.btn_set_from_actual.pressed.connect(lambda: [field.set_from_actual() for field in self.fields])

        self.btn_set_from_file.pressed.connect(self.read_from_file)

        self.btn_move_joint.pressed.connect(lambda: (
            self.robot.moveJ(target=self.robot.set_target, gripper=self.robot.set_target.joints[6]),
        ))
        self.btn_move_linear.pressed.connect(lambda: (
            self.robot.moveL(target=self.robot.set_target, gripper=self.robot.set_target.joints[6]),
        ))

    def read_from_file(self):
        path, _ = qtw.QFileDialog.getOpenFileName(
            None, "Load Target", str(self.ws.targets_dir), "JSON (*.json)"
        )
        if not path:
            return
        self.robot.set_target = Target.read_from_file(path)
        self.robot.set_target.joints[:6] = self.robot.kinematics.tcp_to_joints(
            self.robot.set_target.tcp,
            self.robot.set_target.joints,
            shoulder_pref=self.robot.set_target.shoulder,
            elbow_pref=self.robot.set_target.elbow,
            wrist_pref=self.robot.set_target.wrist)
        match self.frame:
            case "joint":
                for field, new_pos in zip(self.fields, self.robot.set_target.joints):
                    field.box_set_pos.setValue(new_pos)
            case "base":
                pose = self.robot.kinematics.joints_to_pose(self.robot.set_target.joints)
                for field, new_pos in zip(self.fields, pose):
                    field.box_set_pos.setValue(new_pos)
# endregion


class ModelPageCommands:
    def __init__(self, robot_api: RobotAPI):
        self.robot = robot_api
        self.ws = WorkspaceManager(BASE_DIR / "workspace")

        self.btn_set_to_file = qtw.QPushButton("Write Set to File")
        self.btn_act_to_file = qtw.QPushButton("Write Actual to File")

        self.layout = qtw.QHBoxLayout()
        self.layout.addWidget(self.btn_set_to_file)
        self.layout.addWidget(self.btn_act_to_file)

        self.btn_set_to_file.pressed.connect(
            lambda: self.write_to_file(self.robot.set_target)
        )
        self.btn_act_to_file.pressed.connect(
            lambda: self.write_to_file(self.robot.act_target)
        )

    def write_to_file(self, target: Target):
        filename, _ = qtw.QFileDialog.getSaveFileName(
            None, "Save Target", str(self.ws.targets_dir), "JSON (*.json)"
        )
        if not filename:
            return
        target.write_to_file(filename)
        win = self.btn_set_to_file.window()
        if win and hasattr(win, "show_message"):
            win.show_message(f"Target saved to {filename}")


class MCUConfigPage:
    def __init__(self, field_ids, robot_api: RobotAPI):
        self.layout = qtw.QHBoxLayout()
        self.fields = [MCUConfigField(
            field_id=field_ids[i],
            robot_api=robot_api) for i, _ in enumerate(field_ids)]
        for field in self.fields:
            self.layout.addLayout(field.layout)

        self.layout.addStretch()


class MCUConfigField:
    def __init__(self, field_id, robot_api: RobotAPI):
        self.field_id = field_id
        self.robot = robot_api

        self.btn_set_zero = qtw.QPushButton("Set Pos 0")
        self.btn_set_zero.setFixedWidth(93)
        # self.btn_set_zero.setFixedHeight(_HEIGHT)
        self.btn_jump_to_bootloader = qtw.QPushButton("Bootloader")
        self.btn_jump_to_bootloader.setFixedWidth(93)
        # self.btn_jump_to_bootloader.setFixedHeight(_HEIGHT)

        self.layout = qtw.QVBoxLayout()
        self.layout.addWidget(self.btn_set_zero)
        self.layout.addWidget(self.btn_jump_to_bootloader)

        self.btn_set_zero.pressed.connect(self.set_zero)

        self.btn_jump_to_bootloader.pressed.connect(self.jump_to_bootloader)

    def set_zero(self):
        res = self.robot.set_zero(f"j{self.field_id+1}")
        win = self.btn_set_zero.window()
        if win and hasattr(win, "show_message"):
            msg = f"MCU {self.field_id} set to zero." if res else f"MCU {self.field_id} set to zero FAILED."

    def jump_to_bootloader(self):
        res = self.robot.jump_to_bootloader(f"j{self.field_id+1}")
        win = self.btn_set_zero.window()
        if win and hasattr(win, "show_message"):
            msg = f"MCU {self.field_id} set to bootloader." if res else f"MCU {self.field_id} set to bootloader FAILED."

# endregion


