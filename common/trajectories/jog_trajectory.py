from common.target import Target
from common.robot_config import RobotConfig
from common.kinematics.base_kinematics import BaseKinematics

from common.trajectories.jog_joint_trajectory import jog_joint_trajectory
from common.trajectories.jog_base_trajectory import jog_base_trajectory
from common.trajectories.jog_tool_trajectory import jog_tool_trajectory


def jog_trajectory(
        act_target: Target,
        frame: str,
        axis: int,
        direction: int,
        kinematics: BaseKinematics,
        config: RobotConfig,
        distance: float = None,
        overwrite: float = 1.0,
):
    # Gripper Handling
    if axis == 6:
        frame = "joint"

    match frame:
        case "joint":
            return jog_joint_trajectory(
                act_target, axis, direction, kinematics, config, distance, overwrite
            )
        case "base":
            return jog_base_trajectory(
                act_target, axis, direction, kinematics, config, distance, overwrite
            )
        case "tool":
            return jog_tool_trajectory(
                act_target, axis, direction, kinematics, config, distance, overwrite
            )
