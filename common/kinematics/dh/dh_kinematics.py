import logging
from common.kinematics.base_kinematics import BaseKinematics
from common.kinematics.dh.utils import *
import numpy as np
import numpy.typing as npt
from typing import TypeAlias
from common.robot_config import RobotConfig
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[3]

vec6: TypeAlias = [float, float, float, float, float, float] or npt.NDArray[np.float64]
Matrix4x4: TypeAlias = npt.NDArray[np.float64]


class DHKinematics(BaseKinematics):
    def __init__(self):
        self.log = logging.getLogger(__name__)
        self.config = RobotConfig.default()

    def joints_to_transformations(self, joints: vec6) ->\
            [Matrix4x4, Matrix4x4, Matrix4x4, Matrix4x4, Matrix4x4, Matrix4x4, Matrix4x4, Matrix4x4]:
        """All FK transformations j1...j6 to T00...T06"""
        B1 = self.config.link.B1
        A2 = self.config.link.A2
        B4 = self.config.link.B4
        B6 = self.config.link.B6
        j1, j2, j3, j4, j5, j6 = joints[:6]
        # Transformation matrices
        T_00 = dh_to_transformation(link_length=0.0, joint_offset=0.0, twist=0.0, theta=0.0)
        T_01 = dh_to_transformation(link_length=0.0, joint_offset=B1, twist=90.0, theta=j1)
        T_12 = dh_to_transformation(link_length=A2, joint_offset=0.0, twist=0.0, theta=j2)
        T_23 = dh_to_transformation(link_length=0.0, joint_offset=0.0, twist=90.0, theta=j3)
        T_34 = dh_to_transformation(link_length=0.0, joint_offset=B4, twist=-90.0, theta=j4)
        T_45 = dh_to_transformation(link_length=0.0, joint_offset=0.0, twist=90.0, theta=j5)
        # Flange and Tool
        T_56 = dh_to_transformation(link_length=0.0, joint_offset=B6, twist=0.0, theta=j6) @ self.config.tool.matrix

        T_02 = T_01 @ T_12
        T_03 = T_02 @ T_23
        T_04 = T_03 @ T_34
        T_05 = T_04 @ T_45
        T_06 = T_05 @ T_56
        T_07 = T_06 @ self.config.tcp_offset.matrix
        return T_00, T_01, T_02, T_03, T_04, T_05, T_06, T_07

    def joints_to_tcp(self, joints: vec6) -> Matrix4x4:
        """FK transformation j1...j6 to TCP-Matrix4x4"""
        return self.joints_to_transformations(joints)[7]

    def joints_to_pose(self, joints: vec6) -> vec6:
        """FK transformation j1..j6 to X,Y,Z,A,B,C"""
        T = self.joints_to_tcp(joints)

        position = T[0:3, 3]
        R = T[0:3, 0:3]
        rpy = rotation_matrix_to_rpy(R)

        pose = joints
        pose[:6] = [
            float(position[0]),
            float(position[1]),
            float(position[2]),
            float(rpy[0]),
            float(rpy[1]),
            float(rpy[2]),
        ]
        return pose

    def _tcp_to_joints_solutions(self, T_07: Matrix4x4, prev_joints: vec6,
                                 shoulder_pref=None, elbow_pref=None, wrist_pref=None) -> list | None:
        """All IK-Solutions or narrowed down with optional pose preferences.

        Args:
            T_06: TCP-Matrix4x4
            prev_joints: actual j1...j6 or narrowed seed
            shoulder_pref: 'left' oder 'right'
            elbow_pref: 'up' oder 'down'
            wrist_pref: 'flip' oder 'no_flip'

        Returns:
            dynamic length list of [j1...j6] with all IK-Solutions
        """
        prev_joints = prev_joints[:6]
        B1 = self.config.link.B1
        A2 = self.config.link.A2
        B4 = self.config.link.B4
        B6 = self.config.link.B6
        solutions = []

        # Revert virtual TCP offset and physical tool matrix to get WCP
        T_06 = T_07 @ np.linalg.inv(self.config.tcp_offset.matrix) @ np.linalg.inv(self.config.tool.matrix)

        # 2. Flange transform to WCP
        p_06 = T_06[0:3, 3]
        R_06 = T_06[0:3, 0:3]
        p_wc = p_06 - B6 * (R_06 @ np.array([0, 0, 1]))

        if is_shoulder_singularity(p_wc):
            j1_candidates = [prev_joints[0]]
        else:
            # 2 solutions for base rotation
            base = np.degrees(np.arctan2(p_wc[1], p_wc[0]))
            j1_candidates = [base, base + 180.0]
            if shoulder_pref == "right":
                j1_candidates = [j1_candidates[0]]
            if shoulder_pref == "left":
                j1_candidates = [j1_candidates[1]]

        for j1 in j1_candidates:
            # Project wrist center into the planar arm frame (joint2 plane)
            r = np.cos(np.radians(j1)) * p_wc[0] + np.sin(np.radians(j1)) * p_wc[1]
            z = p_wc[2] - B1
            # Law of cosines → D = cos(phi) (elbow angle in triangle)
            D = (r ** 2 + z ** 2 - A2 ** 2 - B4 ** 2) / (2 * A2 * B4)
            D = np.clip(D, -1.0, 1.0)

            if is_elbow_singularity(D):
                j3_candidates = [prev_joints[2]]
            else:
                # 2 solutions for elbow up and down
                j3_geom_candidates = [
                    np.degrees(np.arctan2(+np.sqrt(1 - D ** 2), D)),
                    np.degrees(np.arctan2(-np.sqrt(1 - D ** 2), D))
                ]
                if elbow_pref == "up":
                    j3_geom_candidates = [j3_geom_candidates[1]]
                if elbow_pref == "down":
                    j3_geom_candidates = [j3_geom_candidates[0]]

                # Convert triangle angle phi to DH joint angle j3
                j3_candidates = [90 - j for j in j3_geom_candidates]

            for j3 in j3_candidates:
                # Reapply conversion from DH to triangle angle
                j2 = solve_j2_j3(z, r, j3 - 90, A2, B4)
                # Forward kinematics up to joint3
                T_03 = (
                        dh_to_transformation(0.0, B1, 90.0, j1) @
                        dh_to_transformation(A2, 0.0, 0.0, j2) @
                        dh_to_transformation(0.0, 0.0, 90.0, j3)
                )
                # Rotation that must be produced by wrist joints (4-6)
                R_36 = T_03[0:3, 0:3].T @ R_06

                # Two wrist configurations (wrist flip)
                flip_candidates = [False, True]
                if wrist_pref == "no_flip":
                    flip_candidates = [flip_candidates[0]]
                if wrist_pref == "flip":
                    flip_candidates = [flip_candidates[1]]
                for flip in flip_candidates:
                    j4, j5, j6 = solve_wrist(R_36, flip)

                    if is_wrist_singularity(j5):
                        # extract R_36 (Z-Rotation)
                        j4 = np.degrees(np.arctan2(R_36[1, 0], R_36[0, 0]))
                        j5 = 0.0
                        j6 = 0.0
                        # distribute on current
                        delta = angle_diff(j4, prev_joints[3] + prev_joints[5])
                        j6 = prev_joints[5]
                        j4 = j4 - j6

                    # Base unwrapping for main arm joints (J1..J3)
                    j1 = (j1 + 180.0) % 360.0 - 180.0
                    # j1 = unwrap_angle(j1, prev_joints[0])
                    j2 = unwrap_angle(j2, prev_joints[1])
                    j3 = unwrap_angle(j3, prev_joints[2])

                    # Wrist joints: Normalization to [-180, 180]
                    # Prevent unwrap_angle from continuously stacking +360 deg wraps
                    j4 = (j4 + 180.0) % 360.0 - 180.0
                    j5 = unwrap_angle(j5, prev_joints[4])
                    j6 = (j6 + 180.0) % 360.0 - 180.0

                    solutions.append([j1, j2, j3, j4, j5, j6])

        if not solutions:
            self.log.warning(f"no inverse solutions found")
            return None
        else:
            return solutions

    def tcp_to_joints(self, T06: Matrix4x4, prev_joints: vec6,
                      shoulder_pref: str = None, elbow_pref: str = None, wrist_pref: str = None) -> vec6:
        """IK-Solution with optional pose preferences.

        Args:
            T_06: TCP-Matrix4x4
            prev_joints: actual j1...j6 or narrowed seed
            shoulder_pref: 'left' oder 'right'
            elbow_pref: 'up' oder 'down'
            wrist_pref: 'flip' oder 'no_flip'

        Returns:
            list of [j1...j6]
        """
        prev_joints = prev_joints[:6]
        solutions = self._tcp_to_joints_solutions(T06, prev_joints, shoulder_pref, elbow_pref, wrist_pref)

        if not solutions:
            return None

        limits = [(ax.min_pos, ax.max_pos) for ax in self.config.axes]

        def cost(q):
            # Penalty für Limit-Überschreitungen
            limit_penalty = 0.0
            for val, (q_min, q_max) in zip(q, limits):
                if val < q_min:
                    limit_penalty += (q_min - val) * 1000.0
                elif val > q_max:
                    limit_penalty += (val - q_max) * 1000.0

            return (
                    joint_distance(q, prev_joints) +
                    limit_penalty +
                    0.2 * (1 / (abs(np.sin(np.radians(q[4]))) + 1e-3)) +  # wrist singularity penalty
                    0.1 * (1 / (abs(np.sin(np.radians(q[2]))) + 1e-3))  # elbow penalty
            )
        best = min(solutions, key=cost)
        return best

    def pose_to_joints(self, pose: vec6, prev_joints: vec6,
                       shoulder_pref: str = None, elbow_pref: str = None, wrist_pref: str = None) -> vec6:
        """IK-Solution with optional pose preferences.

        Args:
            pose: [X, Y, Z, A, B, C]
            prev_joints: actual j1...j6 or narrowed seed
            shoulder_pref: 'left' oder 'right'
            elbow_pref: 'up' oder 'down'
            wrist_pref: 'flip' oder 'no_flip'

        Returns:
            list of [j1...j6]
        """
        T06 = np.eye(4)
        T06[:3, :3] = rpy_to_rotation_matrix(pose[3:6])
        T06[:3, 3] = pose[:3]

        joints = pose
        joints[:6] = self.tcp_to_joints(T06, prev_joints, shoulder_pref, elbow_pref, wrist_pref)
        return joints

    def jacobian(self, joints):
        raise NotImplementedError


# region *** Testing ***
def _joints_close(a, b, tol=1e-1):
    diff = np.abs(angle_diff(a, b))
    return np.all(diff < tol), np.sum(diff)


def _unit_test(model: DHKinematics):
    for i in range(1000):
        joints: np.ndarray = np.zeros(6, )
        joints[0] = np.random.uniform(low=-180, high=180)
        joints[1] = np.random.uniform(low=-180, high=180)
        joints[2] = np.random.uniform(low=-180, high=250)
        joints[3] = np.random.uniform(low=-180, high=180)
        joints[4] = np.random.uniform(low=-120, high=120)
        joints[5] = np.random.uniform(low=-180, high=180)
        prev_joints = joints + np.random.uniform(-100, 100, size=6)

        expected_tcp = model.joints_to_tcp(joints)
        solution = model.tcp_to_joints(expected_tcp, prev_joints,
                                       shoulder_pref="left", elbow_pref="up", wrist_pref="no_flip")
        got_tcp = model.joints_to_tcp(solution)

        valid, diff = _joints_close(expected_tcp, got_tcp)
        if not valid:
            logger.warning(f"\n❌ Test failed at iteration {i}")
            logger.warning(f"Expected tcp: \n{np.round(expected_tcp, 3)}")
            logger.warning(f"Got tcp: \n{np.round(got_tcp, 3)}")
            logger.warning(f"set_joints: {np.round(joints, 3)}")
            logger.warning(f"prev_joints: {np.round(prev_joints, 3)}")
            logger.warning(f"solution_joints: {np.round(solution, 3)}")
            logger.warning(f"diff: {diff}")
        else:
            logger.info(f"✅ Test successful at iteration {i}, diff: {diff}")

        assert _joints_close(expected_tcp, got_tcp)
# endregion


if __name__ == "__main__":
    import timeit
    from common.logger import setup_logging
    setup_logging()
    logger = logging.getLogger(__name__)
    np.set_printoptions(suppress=True)

    model = DHKinematics()

    joints_position = [0, 20, 220, 0, 30, 0]
    prev_joints = [0.0, 19.0, 180.0, 0.0, 21.0, 20.0]

    print(f"joint 1 = {joints_position}")
    tcp = model.joints_to_tcp(joints_position)
    print(f"TCP 1 = \n{np.round(tcp, decimals=3)}\n")

    solution = model.tcp_to_joints(tcp, prev_joints, shoulder_pref="left", elbow_pref="up", wrist_pref="no_flip")
    print(f"best Solution = {np.round(solution, decimals=1)}")
    print(f"{np.round(model.joints_to_tcp(solution), decimals=3)}")

    print(f"joints_to_tcp duration: {timeit.timeit(lambda: model.joints_to_tcp(joints_position), number=100)}s")
    print(f"tcp_to_joints duration: {timeit.timeit(lambda: model.tcp_to_joints(tcp, prev_joints), number=100)}s")

    _unit_test(model)
    print("finish")



