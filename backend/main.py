from common.logger import setup_logging
import time
from backend.controller.robot_controller import RobotController


def main():
    setup_logging()
    controller = RobotController()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        controller.stop()


if __name__ == "__main__":
    main()
