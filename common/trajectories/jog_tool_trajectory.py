from common.trajectories.line_trajectory import line_trajectory
from common.target import Target
from common.robot_config import RobotConfig
from common.kinematics.base_kinematics import BaseKinematics

import numpy as np


def jog_tool_trajectory(
        act_target: Target,
        axis: int,
        direction: int,
        kinematics: BaseKinematics,
        config: RobotConfig,
        distance: float = None,
        overwrite: float = 1.0,
):
    set_target = act_target.copy()
    if distance is None:
        if 0 <= axis <= 2:
            distance = 100  # mm
        if 3 <= axis <= 5:
            distance = 45   # deg
    direction = direction / abs(direction)
    angle = direction * np.deg2rad(distance)
    delta_rotation = np.eye(3)
    match axis:
        case 3:
            delta_rotation = np.array([
                [1, 0, 0],
                [0, np.cos(angle), -np.sin(angle)],
                [0, np.sin(angle), np.cos(angle)]
        ])
        case 4:
            delta_rotation = np.array([
                [np.cos(angle), 0, np.sin(angle)],
                [0, 1, 0],
                [-np.sin(angle), 0, np.cos(angle)]
            ])
        case 5:
            delta_rotation = np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1]
            ])
        case _:
            translation_local = np.zeros(3)
            translation_local[axis] = direction * distance
            set_target.tcp[0:3, 3] += act_target.tcp[0:3, 0:3] @ translation_local

    set_target.tcp[0:3, 0:3] = act_target.tcp[0:3, 0:3] @ delta_rotation
    U, _, Vt = np.linalg.svd(set_target.tcp[0:3, 0:3])
    set_target.tcp[0:3, 0:3] = U @ Vt
    set_target.frame = "tool"

    vel_trajectory, pos_trajectory, duration = line_trajectory(act_target, set_target, kinematics, config, overwrite)
    return vel_trajectory, pos_trajectory, duration


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from common.kinematics.dh.dh_kinematics import DHKinematics
    from common.robot_config import RobotConfig

    kinematics = DHKinematics()

    act_joints = [1.0, 130.0, 100.0, 1.0, 1.0, 1.0, 1.0]
    act_target = Target(
        joints=act_joints,
        tcp=kinematics.joints_to_tcp(act_joints),
    )

    vel, pos, duration = jog_tool_trajectory(act_target, 1, 1, kinematics, RobotConfig.default())

    time = np.linspace(0, duration, 100)

    vel_ = [[vel[i](t) for i in range(6)] for t in time]
    pos_ = [[pos[i](t) for i in range(6)] for t in time]

    plt.figure()
    plt.plot(time, vel_)
    plt.xlabel("time / s")
    plt.ylabel("vel / deg/s")
    plt.legend([f"j{i}" for i in range(1, 7)])
    plt.grid()

    plt.figure()
    plt.plot(time, pos_)
    plt.xlabel("time / s")
    plt.ylabel("pos / deg")
    plt.legend([f"j{i}" for i in range(1, 7)])
    plt.grid()

    plt.show()
