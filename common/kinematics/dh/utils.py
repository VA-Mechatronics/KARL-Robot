import numpy as np
from numpy import sin, cos
import numpy.typing as npt
from typing import TypeAlias

Matrix4x4: TypeAlias = npt.NDArray[np.float64]


def dh_to_transformation(link_length: float, joint_offset: float, twist: float, theta: float) -> Matrix4x4:
    """Denavit-Hartenberg-Convention to transformation matrix for one joint to next

    Args:
        link_length: z-shift relative to previous joint
        joint_offset: x-shift relative to previous joint
        twist: x-rotation relative to previous joint
        theta: z-rotation relative to previous joint (actual joint actuator value)

    Returns:
        4x4 homogenous transformation matrix
    """
    a = link_length
    d = joint_offset
    alpha = twist / 180.0 * np.pi
    theta = theta / 180.0 * np.pi
    T = np.array([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
        [0.0, sin(alpha), cos(alpha), d],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return T


def wrap_to_90(angle_deg):
    """Wrap angle to [-90, 90] range"""
    angle_deg = ((angle_deg + 90) % 180) - 90
    return angle_deg


def angle_diff(a, b):
    """Shortest signed angle difference (deg)"""
    return (a - b + 180) % 360 - 180


def unwrap_angle(angle, reference):
    return reference + angle_diff(angle, reference)


def joint_distance(q1, q2, weights=None):
    diff = np.array([angle_diff(q1[i], q2[i]) for i in range(6)])
    if weights is not None:
        diff = diff * weights
    return np.linalg.norm(diff)


def is_wrist_singularity(j5, eps=1e-7):
    return abs(np.sin(np.radians(j5))) < eps


def is_elbow_singularity(D, eps=1e-7):
    return abs(1 - abs(D)) < eps


def is_shoulder_singularity(p_wc, eps=1e-7):
    return np.hypot(p_wc[0], p_wc[1]) < eps


def rotation_matrix_to_rpy(R):
    """Extract RPY-Angles (C=Roll, B=Pitch, A=Yaw in deg) from 4x4-Matrix"""
    eps = 1e-7

    b_rad = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    B = np.degrees(b_rad)

    # Check Gimbal Lock (Pitch close ±90°)
    if np.abs(np.abs(R[2, 0]) - 1.0) < eps:
        C = 0.0
        if R[2, 0] < 0:  # Pitch = +90°
            A = np.degrees(np.arctan2(R[0, 1], R[1, 1]))
        else:  # Pitch = -90°
            A = np.degrees(np.arctan2(-R[0, 1], R[1, 1]))
    else:
        # Roll (C) from R[2,1] and R[2,2]
        C = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        # Yaw (A) from R[1,0] and R[0,0]
        A = np.degrees(np.arctan2(R[1, 0], R[0, 0]))

    return np.array([C, B, A])


def rpy_to_rotation_matrix(rpy):
    """
    Convert RPY-Angles (C=Roll, B=Pitch, A=Yaw in deg) to 4x4-Matrix
    Convention: ZYX (R = Rz(A) * Ry(B) * Rx(C))
    """
    c, b, a = np.radians(rpy)

    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    cc, sc = np.cos(c), np.sin(c)

    R = np.array([
        [ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc],
        [sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc],
        [-sb, cb * sc, cb * cc]
    ])
    return R


def solve_j2_j3(z, r, j3, A2, B4):
    return np.degrees(
        np.arctan2(z, r) -
        np.arctan2(B4 * np.sin(np.radians(j3)),
                   A2 + B4 * np.cos(np.radians(j3)))
    )


def solve_wrist(R, flip=False):
    s5 = np.sqrt(R[2,0]**2 + R[2,1]**2)

    j5 = np.degrees(np.arctan2(s5, R[2,2]))
    j4 = np.degrees(np.arctan2(R[1,2], R[0,2]))
    j6 = np.degrees(np.arctan2(R[2,1], -R[2,0]))

    if flip:
        j5 = -j5
        j4 = j4 + 180
        j6 = j6 + 180

    return j4, j5, j6


if __name__ == "__main__":
    R = rpy_to_rotation_matrix([0, 50.2, 95])
    print(rotation_matrix_to_rpy(R))