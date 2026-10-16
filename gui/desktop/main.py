from common.logger import setup_logging
import sys
from PySide6.QtWidgets import QApplication
from gui.desktop.window import MainWindow
from client.client_api import RobotAPI


def main():
    setup_logging()
    app = QApplication(sys.argv)
    api = RobotAPI()
    window = MainWindow(api)
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
