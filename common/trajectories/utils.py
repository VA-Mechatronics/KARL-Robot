import numpy as np
from typing import Callable

vec6 = list[float, float, float, float, float, float]


def get_quintic_coeff(Q: vec6, t0: float, tf: float) -> vec6:
    """
    Args:
        t0: start time
        tf: final time
        Q: Start and final conditions [pos0, vel0, acc0, posf, velf, accf]
    """
    # T_matrix * C_vector = Q_vector    ->  C_vector = T_inv_matrix * Q_vector
    T = np.array([
        # c0  c1  c2           c3           c4            c5
        [  1, t0,     t0 ** 2,     t0 ** 3,      t0 ** 4,      t0 ** 5],
        [  0,  1, 2 * t0     , 3 * t0 ** 2,  4 * t0 ** 3,  5 * t0 ** 4],
        [  0,  0, 2          , 6 * t0     , 12 * t0 ** 2, 20 * t0 ** 3],
        [  1, tf,     tf ** 2,     tf ** 3,      tf ** 4,      tf ** 5],
        [  0,  1, 2 * tf     , 3 * tf ** 2,  4 * tf ** 3,  5 * tf ** 4],
        [  0,  0, 2          , 6 * tf     , 12 * tf ** 2, 20 * tf ** 3],
    ])
    C = np.linalg.solve(T, Q)
    return C


def get_quintic_trajectory_functions(Q: vec6, t0: float, tf: float) -> tuple[Callable[[float], float], Callable[[float], float]]:
    coeff = get_quintic_coeff(Q, t0, tf)

    def pos(t):
        return coeff[0] + coeff[1] * t + coeff[2] * t ** 2 + coeff[3] * t ** 3 + coeff[4] * t ** 4 + coeff[5] * t ** 5

    def vel(t):
        return coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t ** 2 + 4 * coeff[4] * t ** 3 + 5 * coeff[5] * t ** 4

    return vel, pos


def get_quintic_const_speed_trajectory_functions(Q: vec6) -> tuple[Callable[[float], float], Callable[[float], float]]:
    start_pos, final_pos, final_vel = Q[0], Q[3], Q[4]
    ramp_up_duration = abs((final_pos - start_pos) / final_vel * 2.5)
    # Corrections for signed velocity
    direction = (final_pos - start_pos) / abs(final_pos - start_pos)
    final_vel = direction * abs(final_vel)
    Q[4] = final_vel

    # Quintic ramp up
    vel_func, pos_func = get_quintic_trajectory_functions(Q, 0, ramp_up_duration)
    const_start_pos = pos_func(ramp_up_duration)

    def pos(t):
        if t <= ramp_up_duration:
            return pos_func(t)
        else:
            return const_start_pos + final_vel * (t - ramp_up_duration)

    def vel(t):
        if t <= ramp_up_duration:
            return vel_func(t)
        else:
            return final_vel

    return vel, pos
