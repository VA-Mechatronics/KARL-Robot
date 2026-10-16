import logging
from typing import List
import numpy as np
import scipy.interpolate as interpolate

from common.target import Target
from common.robot_config import RobotConfig
from common.kinematics.base_kinematics import BaseKinematics

log = logging.getLogger(__name__)


def poly_trajectory(
        act_target: Target,
        set_targets: List[Target],
        kinematics: BaseKinematics,
        config: RobotConfig,
        overwrite: float = 1.0,
        blend_radius: float = 5.0  # Transition smooth radius in deg
):
    """Generates a smooth, blended multi-point trajectory across all targets.

    Args:
        act_target: Target Object
        set_targets: List of Target objects [P0, P1, P2, ... Pn]
        kinematics: Robot kinematics
        config: RobotConfig for velocity checks
        overwrite: Velocity override factor
        blend_radius: Smoothing factor for corner rounding. Not implemented
    """
    # 1. Resolve missing joints / kinematics for all targets
    targets = [act_target] + set_targets
    joint_points = []
    for t in targets:
        t.joints[:7] = np.where(np.isnan(t.joints), targets[0].joints, t.joints)
        if t.has_tcp:
            t.joints[:6] = kinematics.tcp_to_joints(
                t.tcp, t.joints,
                shoulder_pref=t.shoulder,
                elbow_pref=t.elbow,
                wrist_pref=t.wrist
            )
        joint_points.append(t.joints[:7])

    joint_points = np.array(joint_points)  # Shape: (N, 7)
    num_points = len(joint_points)

    # 2. Estimate initial time steps based on max joint velocity limits
    max_vels = np.array([min(ax.nom_velocity * overwrite, ax.max_velocity) for ax in config.axes])

    # Calculate distance between consecutive waypoints
    time_steps = [0.0]
    for k in range(num_points - 1):
        dist = np.abs(joint_points[k + 1] - joint_points[k])
        # Duration for segment needed to satisfy joint velocity limits
        seg_duration = np.max(dist / max_vels) * 1.5  # 1.5 safety factor for acceleration
        seg_duration = max(seg_duration, 0.2)  # min segment time 200ms
        time_steps.append(time_steps[-1] + seg_duration)

    time_steps = np.array(time_steps)
    duration = time_steps[-1]

    # 3. Create smooth B-Spline interpolation (C2-continuous: velocity & acceleration)
    pos_funcs = []
    vel_funcs = []

    for axis_idx in range(7):
        y_pts = joint_points[:, axis_idx]

        # Cubic B-Spline representation with smoothed boundary condition (bc_type='clamped' for v=0 at ends)
        # Using CubicSpline ensures continuous position, velocity and acceleration
        cs = interpolate.CubicSpline(time_steps, y_pts, bc_type='clamped')
        cs_vel = cs.derivative(nu=1)

        pos_funcs.append(cs)
        vel_funcs.append(cs_vel)

    # 4. Check velocity limit compliance and iteratively scale duration if needed
    t_eval = np.linspace(0, duration, 200)
    for axis_idx in range(7):
        max_v_reached = np.max(np.abs(vel_funcs[axis_idx](t_eval)))
        if max_v_reached > max_vels[axis_idx]:
            scale_factor = max_v_reached / max_vels[axis_idx]
            time_steps = time_steps * scale_factor
            duration = time_steps[-1]

            # Re-evaluate splines with scaled time
            pos_funcs = []
            vel_funcs = []
            for a_i in range(7):
                cs = interpolate.CubicSpline(time_steps, joint_points[:, a_i], bc_type='clamped')
                pos_funcs.append(cs)
                vel_funcs.append(cs.derivative(nu=1))
            break

    return vel_funcs, pos_funcs, duration