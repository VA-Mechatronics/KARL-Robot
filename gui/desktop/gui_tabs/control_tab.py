import logging
import PySide6.QtWidgets as qtw
from gui.desktop.gui_core.control_fields import JogPage, PositionPage
from client.client_api import RobotAPI


class ControlTab(qtw.QWidget):
    def __init__(self, robot_api: RobotAPI):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.robot = robot_api

        layout = qtw.QGridLayout()

        # region *** Frame Selectors ***
        layout.addWidget(qtw.QLabel("Relative Motion Jog"), 0, 0)
        relative_frame_combo = qtw.QComboBox()
        relative_frame_combo.addItems(["Joint Space", "Base Frame", "Tool Frame"])
        layout.addWidget(relative_frame_combo, 1, 0)

        layout.addWidget(qtw.QLabel("Absolute Position"), 0, 1)
        absolute_frame_combo = qtw.QComboBox()
        absolute_frame_combo.addItems(["Joint Space", "Base Frame"])
        layout.addWidget(absolute_frame_combo, 1, 1)

        self.relative_mode = "Joint Space"
        self.absolute_mode = "Joint Space"

        relative_frame_combo.currentTextChanged.connect(self.update_relative_ui)
        absolute_frame_combo.currentTextChanged.connect(self.update_absolute_ui)
        # endregion

        # region *** Relative Stack ***
        self.relative_container = qtw.QWidget()
        self.relative_stack = qtw.QStackedLayout(self.relative_container)

        # region *** Joint Jog Page ***
        joint_jog_page = qtw.QWidget()
        joint_jog_page.setLayout(
            JogPage(
                frame="joint",
                field_ids=[i for i in range(7)],
                field_tile="Joint",
                field_names=["J1 (Base)", "J2 (Shoulder)", "J3 (Elbow)", "J4 (Wrist 1)", "J5 (Wrist 2)",
                             "J6 (Wrist 3)", "J7 (Gripper)"],
                robot_api=self.robot).page_layout
        )
        self.relative_stack.addWidget(joint_jog_page)
        # endregion

        # region *** Base Jog Page ***
        base_jog_page = qtw.QWidget()
        base_jog_page.setLayout(
            JogPage(
                frame="base",
                field_ids=[i for i in range(7)],
                field_tile="Base Coordinate",
                field_names=["X", "Y", "Z", "A", "B", "C", "J7 (Gripper)"],
                robot_api=self.robot).page_layout
        )
        self.relative_stack.addWidget(base_jog_page)
        # endregion

        # region *** Tool Jog Page ***
        tool_jog_page = qtw.QWidget()
        tool_jog_page.setLayout(
            JogPage(
                frame="tool",
                field_ids=[i for i in range(7)],
                field_tile="Tool Coordinate",
                field_names=["X", "Y", "Z", "A", "B", "C", "J7 (Gripper)"],
                robot_api=self.robot).page_layout
        )
        self.relative_stack.addWidget(tool_jog_page)
        # endregion
        # endregion

        # region *** Absolute Stack ***
        self.absolute_container = qtw.QWidget()
        self.absolute_stack = qtw.QStackedLayout(self.absolute_container)

        # region *** Joint Position Page ***
        self.joint_position_page_object = PositionPage(
                frame="joint",
                field_ids=[i for i in range(7)],
                field_tile="Joint",
                field_names=["J1 (Base)", "J2 (Shoulder)", "J3 (Elbow)", "J4 (Wrist 1)", "J5 (Wrist 2)",
                             "J6 (Wrist 3)", "J7 (Gripper)"],
                robot_api=self.robot)

        self.joint_position_page = qtw.QWidget()
        self.joint_position_page.setLayout(self.joint_position_page_object.page_layout)
        self.absolute_stack.addWidget(self.joint_position_page)
        # endregion

        # region *** Base Position Page ***
        self.base_position_page_object = PositionPage(
                frame="base",
                field_ids=[i for i in range(7)],
                field_tile="Base Coordinate",
                field_names=["X", "Y", "Z", "A", "B", "C", "J7 (Gripper)"],
                robot_api=self.robot)

        base_position_page = qtw.QWidget()
        base_position_page.setLayout(self.base_position_page_object.page_layout)
        self.absolute_stack.addWidget(base_position_page)
        # endregion

        layout.addWidget(self.relative_container, 2, 0)
        layout.addWidget(self.absolute_container, 2, 1)

        self.setLayout(layout)

    def update_relative_ui(self, text):
        mapping = {
            "Joint Space": 0,
            "Base Frame": 1,
            "Tool Frame": 2
        }
        self.relative_stack.setCurrentIndex(mapping[text])

    def update_absolute_ui(self, text):
        mapping = {
            "Joint Space": 0,
            "Base Frame": 1
        }
        self.absolute_stack.setCurrentIndex(mapping[text])

        match text:
            case "Joint Space":
                joints = self.robot.set_target.joints
                for x, new_pos in zip(self.joint_position_page_object.fields, joints):
                    x.box_set_pos.setValue(new_pos)
            case "Base Frame":
                pose = self.robot.kinematics.joints_to_pose(self.robot.set_target.joints)
                for x, new_pos in zip(self.base_position_page_object.fields, pose):
                    x.box_set_pos.setValue(new_pos)
