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


def poly_move():
    with robot.blocking_mode():
        robot.overwrite(2.0)
        robot.moveJ(targets["home"])
        targets["new_home"] = targets["home"].offset(y=-50)
        robot.movePoly(targets=[targets["new_home"], targets["pos2"], targets["straight"], targets["pose"], targets["new_home"]])

        poly = []
        for _ in range(2):
            poly.append(targets["pos1"].offset(x=60, z=60))
            poly.append(targets["pos1"].offset(z=120))
            poly.append(targets["pos1"].offset(x=-60, z=60))
            poly.append(targets["pos1"])
        robot.moveJ(targets["pos1"])
        robot.movePoly(poly)


def serve_drink():
    with robot.blocking_mode():
        robot.overwrite(2.0)
        robot.pick(target=targets["bottle"], approach=targets["bottle"].offset(y=150), open_gripper=80)
        robot.set_tcp_offset(y=100)
        robot.jog_tool(axis="C", distance=90)
        robot.reset_tcp_offset()
        robot.place(target=targets["bottle"], approach=targets["bottle"].offset(y=150), open_gripper=80)


def palletize():
    feed_in = targets["home"].offset(x=10, y=-100)
    frame = targets["frame"].copy()
    with robot.blocking_mode():
        for dx in [0, 100]:
            for dy in [-0]:
                feed_out = frame.offset(x=dx, y=dy)
                robot.pick(feed_in, feed_in.offset(z=-100))
                robot.place(feed_out, feed_out.offset(z=-100))


def main():
    palletize()
    serve_drink()
    poly_move()

    with robot.blocking_mode():
        robot.moveJ(targets["sleep"], gripper=0)
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
