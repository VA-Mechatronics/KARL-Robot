import logging

from common.trajectories.utils import get_quintic_trajectory_functions
from common.target import Target
from common.robot_config import RobotConfig
from common.kinematics.base_kinematics import BaseKinematics

import numpy as np

log = logging.getLogger()


def ptp_trajectory(
        act_target: Target,
        set_target: Target,
        kinematics: BaseKinematics,
        config: RobotConfig,
        overwrite: float = 1.0,
):
    set_target.joints[:7] = np.where(np.isnan(set_target.joints), act_target.joints, set_target.joints)
    if set_target.has_tcp:
        # TCP is source of truth
        set_target.joints[:6] = kinematics.tcp_to_joints(set_target.tcp, set_target.joints,
                                                         shoulder_pref=set_target.shoulder,
                                                         elbow_pref=set_target.elbow,
                                                         wrist_pref=set_target.wrist)
    else:
        set_target.tcp = kinematics.joints_to_tcp(set_target.joints)

    iteration = 0
    duration = 1.0
    while iteration < 100:
        valid = True
        vel_trajectory = []
        pos_trajectory = []
        overspeed: np.ndarray = np.zeros(7)
        for i in range(7):
            vel_func, pos_func = get_quintic_trajectory_functions(
                [act_target.joints[i], 0, 0, set_target.joints[i], 0, 0]
                , 0, duration
            )

            overspeed[i] = abs(vel_func(duration/2)) / min((config.axes[i].nom_velocity * overwrite, config.axes[i].max_velocity))
            if np.max(overspeed) > 1.0:
                valid = False
                break
            if not valid:
                break
            # Success HERE
            vel_trajectory.append(vel_func)
            pos_trajectory.append(pos_func)
        if not valid:
            duration *= np.max(overspeed)
            # duration += 1.0
            iteration += 1
            continue
        else:
            break
    else:
        log.error(f"ptp trajectory iterations exceeded")
        return None, None, 0

    return vel_trajectory, pos_trajectory, duration


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from common.kinematics.dh.dh_kinematics import DHKinematics
    from common.robot_config import RobotConfig

    kinematics = DHKinematics()

    act_joints = [10.0, 120.0, 4.0, 0.0, 0.0, 0.0, 0]
    act_target = Target(
        joints=act_joints,
        tcp=kinematics.joints_to_tcp(act_joints),
    )
    print(act_target.tcp[0:3, 3])

    set_joints = [60.0, 45.0, 45.0, 60.0, 45.0, 45.0, 0]
    set_target = Target(
        joints=set_joints,
        tcp=kinematics.joints_to_tcp(set_joints),
    )
    print(set_target.tcp[0:3, 3])

    vel, pos, duration = ptp_trajectory(act_target, set_target, kinematics, RobotConfig.default())

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
