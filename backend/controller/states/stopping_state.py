from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from backend.controller.robot_controller import RobotController
from backend.controller.states.base_state import BaseState

import time
import numpy as np


class StoppingState(BaseState):
    name = "stopping"

    def __init__(self):
        super().__init__()
        self.deceleration_time = 0.5
        self.initial_trajectory_time = None
        self.last_vel = None
        self.last_pos = None

    def enter(self, controller: RobotController, info=None):
        super().enter(controller, info)
        self.initial_trajectory_time = controller.trajectory_time
        if controller.pos_traj and controller.vel_traj:
            self.last_pos = np.array([f(controller.trajectory_time) for f in controller.pos_traj])
            self.last_vel = np.array([f(controller.trajectory_time) for f in controller.vel_traj])
        else:
            self.last_pos = np.array(controller.hw.get_joints_position())
            self.last_vel = np.array(controller.hw.get_joints_velocity())

    def run(self, controller: RobotController):
        controller.hw.update_telemetry()

        current_real_time = time.time() - controller.motion_start_time

        min_limits = np.array([ax.min_pos for ax in controller.hw.config.axes])
        max_limits = np.array([ax.max_pos for ax in controller.hw.config.axes])

        while True:
            lead_time = controller.trajectory_time - current_real_time
            if lead_time >= 0.15:
                break

            timestamp = int(controller.trajectory_time * 1000) & 0xFFFF

            # Linear ramp down
            elapsed_in_ramp = controller.trajectory_time - self.initial_trajectory_time
            s_dot = max(0.0, 1.0 - (elapsed_in_ramp / self.deceleration_time))
            target_vel = self.last_vel * s_dot
            target_pos = self.last_pos + elapsed_in_ramp * (self.last_vel + target_vel) / 2.0

            if (target_pos < min_limits).any() or (target_pos > max_limits).any():
                controller.hw.hard_stop()
                controller.set_state("error")
                self.log.error(f"position limit reached ad t={controller.trajectory_time:.3f}s! Pos: {target_pos}")
                break

            controller.hw.motion(timestamp, target_pos, target_vel)
            controller.trajectory_time += 0.020

            if s_dot == 0.0:
                # Await Buffer last out
                if current_real_time >= (elapsed_in_ramp + 0.15):
                    controller.set_state("holding")
                break

    def exit(self, controller: RobotController):
        controller.pos_traj = None
        controller.vel_traj = None
        # controller.hw.hard_stop(set_enable=True)
