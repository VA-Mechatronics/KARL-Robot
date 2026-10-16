import logging

from common.trajectories.utils import get_quintic_trajectory_functions
from common.target import Target
from common.robot_config import RobotConfig
from common.kinematics.base_kinematics import BaseKinematics

import numpy as np
from scipy.interpolate import interp1d
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

log = logging.getLogger()


def line_trajectory(
        act_target: Target,
        set_target: Target,
        kinematics: BaseKinematics,
        config: RobotConfig,
        overwrite: float = 1.0,
        max_position_error: float = 200.0,  # Accuracy parameter (e.g. in mm or m)
):
    iteration = 0
    duration = 1.0
    set_target.joints[:7] = np.where(np.isnan(set_target.joints), act_target.joints, set_target.joints)
    if set_target.has_tcp:
        # TCP is source of truth
        pass
    else:
        set_target.tcp = kinematics.joints_to_tcp(set_target.joints)

    distance = set_target.tcp[0:3, 3] - act_target.tcp[0:3, 3]
    abs_distance = np.linalg.norm(distance)
    if abs_distance == 0:
        direction = np.zeros(3)
    else:
        direction = distance / abs_distance

    while iteration < 100:
        valid = True
        vel_func, pos_func = get_quintic_trajectory_functions(
            [0, 0, 0, abs_distance, 0, 0, ],
            0, duration
        )

        act_rotation = R.from_matrix(act_target.tcp[0:3, 0:3])
        set_rotation = R.from_matrix(set_target.tcp[0:3, 0:3])
        key_times = [0, duration]
        key_rotations = R.concatenate([act_rotation, set_rotation])
        slerp = Slerp(key_times, key_rotations)

        steps = 100
        time_series = np.linspace(0, duration, steps)
        dt = time_series[1] - time_series[0]

        tcp_series = [act_target.tcp.copy() for _ in range(steps)]
        joints_pos_series = [act_target.joints.copy() for _ in range(steps)]
        joints_vel_series = [[0.0 for __ in range(7)] for _ in range(steps)]

        for i in range(steps):
            t = time_series[i]

            current_pos_step = pos_func(t)
            tcp_series[i][0:3, 3] += current_pos_step * direction.flatten()

            if abs_distance > 0:
                progress = current_pos_step / abs_distance
            else:
                tau = t / duration
                progress = tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)

            progress = np.clip(progress, 0.0, 1.0)
            rotation_interpolation = slerp([progress * duration])[0]
            tcp_series[i][0:3, 0:3] = rotation_interpolation.as_matrix()

            if i > 0:
                # Gripper interpolation
                _, pos_func_6 = get_quintic_trajectory_functions(
                    [act_target.joints[6], 0, 0, set_target.joints[6], 0, 0], 0, duration)

                joints_pos_series[i] = [
                    *kinematics.tcp_to_joints(
                        tcp_series[i], joints_pos_series[i - 1],
                        shoulder_pref=set_target.shoulder,
                        elbow_pref=set_target.elbow,
                        wrist_pref=set_target.wrist,
                    ),
                    pos_func_6(t)
                ]
                for j in range(7):
                    joints_vel_series[i][j] = (joints_pos_series[i][j] - joints_pos_series[i - 1][j]) / dt

        # -------------------------------------------------------------
        # Check joint velocities & attempt trajectory smoothing
        # -------------------------------------------------------------
        overspeed: np.ndarray = np.zeros(7)
        for i in range(steps):
            for j in range(7):
                overspeed[j] = max(overspeed[j], abs(joints_vel_series[i][j]) / min(
                    (config.axes[j].nom_velocity * overwrite, config.axes[j].max_velocity)))

        max_overspeed = np.max(overspeed)

        if max_overspeed > 1.0:
            valid = False

            # Attempt smoothing if position error tolerance is allowed
            if max_position_error > 0.0:
                # 1. Smooth joint angles using a Gaussian filter (stronger smoothing for higher overspeed)
                sigma = min(5.0, max_overspeed * 1.5)
                smoothed_joints = np.copy(joints_pos_series)

                # Lock start and end positions
                for j in range(7):
                    smoothed_joints[:, j] = gaussian_filter1d(smoothed_joints[:, j], sigma=sigma)
                    smoothed_joints[0, j] = joints_pos_series[0][j]
                    smoothed_joints[-1, j] = joints_pos_series[-1][j]

                # 2. Recalculate joint velocities after smoothing
                smoothed_vel = np.zeros_like(smoothed_joints)
                smoothed_vel[1:] = (smoothed_joints[1:] - smoothed_joints[:-1]) / dt

                smoothed_overspeed = np.zeros(7)
                for i in range(steps):
                    for j in range(7):
                        smoothed_overspeed[j] = max(smoothed_overspeed[j], abs(smoothed_vel[i, j]) / min(
                            (config.axes[j].nom_velocity * overwrite, config.axes[j].max_velocity)))

                # 3. Evaluate maximum TCP position error using forward kinematics
                max_dev = 0.0
                for i in range(steps):
                    actual_tcp = kinematics.joints_to_tcp(smoothed_joints[i, :7])
                    dev = np.linalg.norm(actual_tcp[0:3, 3] - tcp_series[i][0:3, 3])
                    if dev > max_dev:
                        max_dev = dev

                # 4. Accept smoothed trajectory if error and velocities are within limits
                if max_dev <= max_position_error and np.max(smoothed_overspeed) <= 1.0:
                    joints_pos_series = smoothed_joints.tolist()
                    joints_vel_series = smoothed_vel.tolist()
                    valid = True
                    log.info(f"Singularity smoothed: TCP deviation={max_dev:.3f} (tolerance: {max_position_error})")

        if not valid:
            # Scale duration dynamically instead of fixed +1.0 increments
            duration *= max(1.1, max_overspeed)
            iteration += 1
            continue
        else:
            break
    else:
        log.error(f"line trajectory iterations exceeded")
        return None, None, 0

    pos_funcs = [interp1d(time_series, [row[j] for row in joints_pos_series],
                          kind='cubic', fill_value="extrapolate")
                 for j in range(7)]

    vel_funcs = [interp1d(time_series, [row[j] for row in joints_vel_series],
                          kind='cubic', fill_value="extrapolate")
                 for j in range(7)]

    pos_trajectory = [lambda t, f=f: float(f(t)) for f in pos_funcs]
    vel_trajectory = [lambda t, f=f: float(f(t)) for f in vel_funcs]

    return vel_trajectory, pos_trajectory, duration


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from common.kinematics.dh.dh_kinematics import DHKinematics
    from common.robot_config import RobotConfig

    kinematics = DHKinematics()

    act_joints = [60.0, 120.0, 120.0, 60.0, 50.0, 45.0, 0]
    act_target = Target(
        joints=act_joints,
        tcp=kinematics.joints_to_tcp(act_joints),
    )
    print(act_target.tcp)

    set_joints = [60.0, 120.0, 100.0, 50.0, 45.0, 45.0, 0]
    set_target = Target(
        joints=set_joints,
        tcp=kinematics.joints_to_tcp(set_joints),
    )
    print(set_target.tcp)

    vel, pos, duration = line_trajectory(act_target, set_target, kinematics, RobotConfig.default())

    time = np.linspace(0, duration, 100)

    vel_ = [[vel[i](t) for i in range(7)] for t in time]
    pos_ = [[pos[i](t) for i in range(7)] for t in time]

    plt.figure()
    plt.plot(time, vel_)
    plt.xlabel("time / s")
    plt.ylabel("vel / deg/s")
    plt.legend([f"j{i}" for i in range(1, 8)])
    plt.grid()

    plt.figure()
    plt.plot(time, pos_)
    plt.xlabel("time / s")
    plt.ylabel("pos / deg")
    plt.legend([f"j{i}" for i in range(1, 8)])
    plt.grid()

    plt.show()
