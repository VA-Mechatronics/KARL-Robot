# region *** Environment ***
import argparse
from common.workspace_manager import WorkspaceManager
from client.client_api import RobotAPI

# --- CLI Arguments ---
parser = argparse.ArgumentParser()
parser.add_argument("--repeat", default="1", help="Number of runs or 'inf' / 'infinite'")
parser.add_argument("--host", default="localhost", help="Robot controller host or IP address")
parser.add_argument("--push-port", type=int, default=5555, help="ZeroMQ Push Port")
parser.add_argument("--sub-port", type=int, default=5556, help="ZeroMQ Sub Port")
parser.add_argument("--req-port", type=int, default=5557, help="ZeroMQ Req Port")
args = parser.parse_args()

workspace = WorkspaceManager()
targets = workspace.load_targets()

robot = RobotAPI()
robot.connect(host=args.host, push_port=args.push_port, sub_port=args.sub_port, req_port=args.req_port)
# endregion


def main():
    with robot.blocking_mode():
        # === GUIDING_COMMANDS_PLACEHOLDER ===
        pass


# region *** Environment ***
if __name__ == "__main__":
    count = float("inf") if args.repeat in ["inf", "infinite", "math.inf"] else int(args.repeat)
    i = 0
    while i < count:
        main()
        i += 1
    print("finished")
# endregion
