from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from backend.controller.robot_controller import RobotController
from backend.controller.states.base_state import BaseState

import time
import numpy as np


class MovingState(BaseState):
    name = "moving"

    def enter(self, controller: RobotController, info=None):
        super().enter(controller, info)
        controller.hw.set_enable(True)

        controller.motion_start_time = time.time()
        controller.trajectory_time = 0.0
        controller.motion_finished_sent = False

        if controller.pos_traj is None:
            controller.set_state("error")
        if controller.vel_traj is None:
            controller.set_state("error")

    def run(self, controller: RobotController):
        controller.hw.update_telemetry()

        current_real_time = time.time() - controller.motion_start_time

        min_limits = np.array([ax.min_pos for ax in controller.hw.config.axes])
        max_limits = np.array([ax.max_pos for ax in controller.hw.config.axes])
        error_limits = np.array([ax.error_limit for ax in controller.hw.config.axes])

        while True:
            lead_time = controller.trajectory_time - current_real_time
            if lead_time >= 0.15:
                break

            if controller.trajectory_time >= controller.motion_duration:
                if current_real_time >= controller.motion_duration:
                    controller.set_state("holding")
                break

            timestamp = int(controller.trajectory_time * 1000) & 0xFFFF

            # Position limits
            target_pos: np.ndarray = np.array([f(controller.trajectory_time) for f in controller.pos_traj])
            target_vel: np.ndarray = np.array([f(controller.trajectory_time) for f in controller.vel_traj])
            pos_limit = np.where((target_pos < min_limits) | (target_pos > max_limits))[0]
            if len(pos_limit) > 0:
                controller.hw.hard_stop()
                controller.set_state("error", f"Joint {pos_limit + 1} position limit exceeded")
                self.log.error(f"position limit reached at t={controller.trajectory_time:.3f}s! Pos: {target_pos}")
                break

            # PID Error limits
            error: np.ndarray = np.array([ax.err for ax in controller.hw.telemetry.axes])
            err_limit = np.where(np.abs(error) > error_limits)[0]
            if len(err_limit) > 0:
                controller.hw.hard_stop()
                controller.set_state("error", f"Joint {err_limit + 1} error limit exceeded")
                self.log.error(f"error limit reached at t={controller.trajectory_time:.3f}s! Error: {error[err_limit]}")
                break

            controller.hw.motion(timestamp, target_pos, target_vel)
            controller.trajectory_time += 0.020

        if (controller.trajectory_time >= controller.motion_duration) and not controller.motion_finished_sent:
            timestamp = int(controller.motion_duration * 1000) & 0xFFFF
            target_pos = np.array([f(controller.motion_duration) for f in controller.pos_traj])
            target_vel = np.array([0.0 for _ in range(len(target_pos))])

            controller.hw.motion(timestamp, target_pos, target_vel)
            controller.motion_finished_sent = True

    def exit(self, controller: RobotController):
        pass
