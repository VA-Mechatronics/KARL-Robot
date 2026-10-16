from common.trajectories.utils import get_quintic_trajectory_functions, get_quintic_const_speed_trajectory_functions
from common.target import Target
from common.robot_config import RobotConfig
from common.kinematics.base_kinematics import BaseKinematics

import numpy as np


def jog_joint_trajectory(
        act_target: Target,
        axis: int,
        direction: int,
        kinematics: BaseKinematics,
        config: RobotConfig,
        distance: float = None,
        overwrite: float = 1.0,
):
    iteration = 0
    while iteration < 100:
        # valid = True
        vel_trajectory = []
        pos_trajectory = []
        act_pos = act_target.joints[axis]
        direction = direction/abs(direction)
        overspeed = 1.0
        duration = 1.0
        for i in range(7):
            if i == axis:
                if distance is None:
                    vel_func, pos_func = get_quintic_const_speed_trajectory_functions(
                        [act_pos, 0, 0, act_pos + direction, min(config.axes[axis].nom_velocity * overwrite, config.axes[axis].max_velocity), 0])
                    duration = 10
                else:
                    while overspeed >= 1.0:
                        duration *= overspeed
                        vel_func, pos_func = get_quintic_trajectory_functions(
                            [act_pos, 0, 0, act_pos + direction * distance, 0, 0], 0, duration)
                        overspeed = abs(vel_func(duration / 2)) / min(config.axes[i].nom_velocity * overwrite, config.axes[i].max_velocity)

                vel_trajectory.append(vel_func)
                pos_trajectory.append(pos_func)
            else:
                vel_trajectory.append(lambda t: 0)
                pos_trajectory.append(lambda t, p0=act_target.joints[i]: p0)
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

    vel, pos, duration = jog_joint_trajectory(act_target, 1, 1, kinematics, RobotConfig.default())

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
