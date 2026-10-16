import logging
import sys
import PySide6.QtWidgets as qtw
from PySide6.QtGui import QPixmap
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[3]


class HomeTab(qtw.QWidget):
    def __init__(self):
        super().__init__()
        layout = qtw.QGridLayout()

        picture_panel = qtw.QVBoxLayout()
        picture = qtw.QLabel()
        path = BASE_DIR / "gui" / "desktop" / "img" / "KARL.png"
        pixmap = QPixmap(path)
        picture.setPixmap(pixmap)
        picture.setFixedSize(726//8*5, 1005//8*5)
        picture.setScaledContents(True)
        picture_panel.addWidget(picture)
        layout.addLayout(picture_panel, 0, 1)

        info_panel = qtw.QVBoxLayout()
        info = qtw.QLabel("Project KARL\n\nKnick-Arm-Roboter-Lite\n\n\nby VA-Mechatronics")
        info.setStyleSheet("font-size: 16px")
        info_panel.addWidget(info)
        layout.addLayout(info_panel, 0, 2)

        self.setLayout(layout)
