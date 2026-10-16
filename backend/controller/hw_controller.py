import logging
import sys
import time
import threading
import struct
from common.robot_config import RobotConfig
from common.mcu_telemetry import RobotTelemetry
from backend.controller.io_controller import IOController

if sys.platform.startswith("linux"):
    import can

vec = [float, float, float, float, float, float, float]


def _create_global_motion_payload(cmd: int, sync_counter: int, timestamp: int, pos: vec, vel: vec):
    payload = bytearray(struct.pack('<BBH', int(cmd), int(sync_counter), int(timestamp)))
    for pos_, vel_ in zip(pos, vel):
        payload.extend(struct.pack('<ii', int(pos_ * 1000), int(vel_ * 1000)))
    return bytes(payload)


class HWController:
    def __init__(self, io_controller: IOController = None):
        self.log = logging.getLogger(__name__)
        self.io_controller = io_controller
        self.config = RobotConfig.default()
        self.telemetry = RobotTelemetry.default()
        self.last_telemetry = 0
        self._ack_lock = threading.Lock()
        self._ack_event = threading.Event()
        self._pending_acks = set()

        if sys.platform.startswith("linux"):
            self.bus = can.interface.Bus(channel="can0", bustype="socketcan", fd=True)
        # ToDo: Implement using sync_counter as roundtrip heartbeat
        self.sync_counter = 0

        if sys.platform.startswith("linux"):
            self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._rx_thread.start()

    def motion(self, timestamp: int, pos: vec, vel: vec):
        for ax, p in zip(self.telemetry.axes, pos):
            ax.set_pos = float(p)

        cmd_run = 1
        motion_bytes = _create_global_motion_payload(cmd_run, self.sync_counter, timestamp, pos, vel)
        if sys.platform.startswith("linux"):
            motion_msg = can.Message(
                arbitration_id=0x100,
                data=motion_bytes,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True
            )
            self.bus.send(motion_msg)
        else:
            for ax, p, v in zip(self.telemetry.axes, pos, vel):
                ax.act_pos = p
                ax.act_vel = v
            # self.log.info(f"t:{timestamp} pos:{[p for p in pos]} vel:{[v for v in vel]}")
        time.sleep(0.001)

    def update_telemetry(self, update_time=0.1):
        if time.time() - self.last_telemetry > update_time:
            self.last_telemetry = time.time()
            for mcu in self.config.axes:
                self._send_request(mcu.can_node_id, 0, 0, 0, 0, 0, 0, 0)
            for mcu in self.config.axes:
                self._send_request(mcu.can_node_id, 1, 0, 0, 0, 0, 0, 0)
            # Additional request on Tooling for Smart-IO
            self._send_request(7, 2, 0, 0, 0, 0, 0, 0)

    def get_joints_position(self):
        return [j.act_pos for j in self.telemetry.axes]

    def get_joints_velocity(self):
        return [j.act_vel for j in self.telemetry.axes]

    def set_config(self, config: RobotConfig, timeout=2) -> bool:
        self.config = config
        msg_types = [3, 4, 5, 6, 7]
        mcu_ids = [mcu.can_node_id for mcu in self.config.axes]

        with self._ack_lock:
            self._pending_acks = {(mcu_id, msg_type) for mcu_id in mcu_ids for msg_type in msg_types}
            self._ack_event.clear()

        for mcu in config.axes:
            self._set_pid(mcu.can_node_id, mcu.kp, mcu.ki, mcu.kd, mcu.kv)
        for mcu in config.axes:
            self._set_mec(mcu.can_node_id, mcu.mot_gear_ratio, mcu.enc_gear_ratio)
        for mcu in config.axes:
            self._set_enc(mcu.can_node_id, mcu.home_offset)
        for mcu in config.axes:
            self._set_drv(mcu.can_node_id, mcu.micro_step, mcu.max_current)
        for mcu in config.axes:
            self._set_gde(mcu.can_node_id, mcu.guiding_gain, mcu.guiding_deadzone, mcu.error_limit)

        if not sys.platform.startswith("linux"):
            return True

        completed = self._ack_event.wait(timeout=timeout)
        if not completed:
            with self._ack_lock:
                missing = list(self._pending_acks)
            self.log.warning(f"set_config CAN ACK timeout! Missing responses: {missing}")
            return False
        return True

    def get_config(self, timeout=2) -> RobotConfig:
        msg_types = [3, 4, 5, 6, 7]
        mcu_ids = [mcu.can_node_id for mcu in self.config.axes]

        with self._ack_lock:
            self._pending_acks = {(mcu_id, msg_type) for mcu_id in mcu_ids for msg_type in msg_types}
            self._ack_event.clear()

        for msg_type in msg_types:
            for mcu in self.config.axes:
                self._send_request(mcu.can_node_id, msg_type, 0, 0, 0, 0, 0, 0)

        if not sys.platform.startswith("linux"):
            return self.config

        completed = self._ack_event.wait(timeout=timeout)
        if not completed:
            with self._ack_lock:
                missing = list(self._pending_acks)
            self.log.warning(f"get_config CAN ACK timeout! Missing responses: {missing}")
        return self.config

    def set_enable(self, en: bool):
        for mcu in self.config.axes:
            self._send_request(mcu.can_node_id, 0, 1, 0, bool(en), 0, 0, 0)

    def start_homing(self, can_node_id):
        self._send_request(can_node_id, 0, 4, 0, 0, 0, 0, 0)

    def set_zero(self, can_node_id):
        self._send_request(can_node_id, 0, 2, 0, 0, 0, 0, 0)

    def set_smart_output(self, can_node_id: int, output_id: int, smart_event: int):
        self._send_request(can_node_id, 2, 1, 0, output_id, smart_event, 0, 0)

    def set_mode(self, mode: int):
        match mode:
            case 0:     # Normal operation
                for mcu in self.config.axes:
                    self._send_request(mcu.can_node_id, 0, 3, 0, int(mode), 0, 0, 0)
            case 1:     # Guiding
                # Base
                self._send_request(1, 0, 3, 0, int(mode), 0, 0, 0)
                self._send_request(1, 0, 1, 0, False, 0, 0, 0)
                # Shoulder
                self._send_request(2, 0, 3, 0, int(mode), 0, 0, 0)
                # Elbow
                self._send_request(3, 0, 3, 0, int(mode), 0, 0, 0)
                # Wrist 1
                self._send_request(4, 0, 3, 0, int(mode), 0, 0, 0)
                self._send_request(4, 0, 1, 0, False, 0, 0, 0)
                # Wrist 2
                self._send_request(5, 0, 3, 0, int(mode), 0, 0, 0)
                self._send_request(5, 0, 1, 0, False, 0, 0, 0)
                # Wrist 3
                self._send_request(6, 0, 3, 0, int(mode), 0, 0, 0)
                self._send_request(6, 0, 1, 0, False, 0, 0, 0)

    def jump_to_bootloader(self, can_node_id):
        self._send_request(can_node_id, 8, 1, 0, 0, 0, 0, 0)

    def hard_stop(self, set_enable: bool = True):
        sub_cmd = 0x01 if set_enable else 0x00
        data = [0x99, sub_cmd]
        if sys.platform.startswith("linux"):
            motion_msg = can.Message(
                arbitration_id=0x000,
                data=data,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True
            )
            self.bus.send(motion_msg)
        else:
            pass
            self.log.info(f"hard stop")
        time.sleep(0.001)

    def _set_pid(self, can_node_id: int, kp: float, ki: float, kd: float, kv: float):
        self._send_request(can_node_id, 3, 1, 0, int(kp * 1000), int(ki * 1000), int(kd * 1000), int(kv * 1000.0))

    def _set_mec(self, can_node_id: int, mot_gear_ratio: float, enc_gear_ratio: float):
        self._send_request(can_node_id, 4, 1, 0, int(mot_gear_ratio * 1000), int(enc_gear_ratio * 1000), 0, 0)

    def _set_enc(self, can_node_id: int, home_offset: float):
        if can_node_id in [6, 7]:
            # Do not override homing related axes
            self._send_request(can_node_id, 5, 0, 0, int(home_offset * 1000), 0, 0, 0)
        else:
            self._send_request(can_node_id, 5, 1, 0, int(home_offset * 1000), 0, 0, 0)

    def _set_drv(self, can_node_id: int, micro_steps: int, max_current: int):
        self._send_request(can_node_id, 6, 1, 0, int(micro_steps), int(max_current), 0, 0)

    def _set_gde(self, can_node_id: int, guiding_gain: int, guiding_deadzone: int, error_limit: int):
        self._send_request(can_node_id, 7, 1, 0, int(guiding_gain), int(guiding_deadzone), int(error_limit), 0)

    def _send_request(self, can_node_id: int, msg_type: int, write: int, echo: int, p1: int, p2: int, p3: int, p4: int):
        request_payload = struct.pack('<BBHiiii',
                                      int(msg_type), int(write), int(echo), int(p1), int(p2), int(p3), int(p4))
        if sys.platform.startswith("linux"):
            request_msg = can.Message(
                arbitration_id=0x200 | can_node_id,
                data=request_payload,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True
            )
            self.bus.send(request_msg)
        else:
            pass
            # self.log.info(f"id:{can_node_id} msg:{msg_type} write:{write} p1:{p1} p2:{p2} p3:{p3} p4:{p4}")
        time.sleep(0.001)

    def _receive_loop(self):
        while True:
            msg = self.bus.recv()
            if not msg:
                continue

            if 0x301 <= msg.arbitration_id <= 0x307:
                can_node_id = msg.arbitration_id & 0x0F
                msg_type, write, echo, p1, p2, p3, p4 = struct.unpack('<BBHiiii', msg.data[:20])
                # self.log.info(f"msg_type:{msg_type} write:{write} echo:{echo} p1:{p1} p2:{p2} p3:{p3} p4:{p4}")

                match msg_type:
                    case 0:  # TELEMETRY 0
                        mcu = self.telemetry.axes[can_node_id - 1]
                        mcu.echo = echo
                        mcu.act_pos = p1 / 1000.0
                        mcu.act_vel = p2 / 1000.0
                        mcu.output = p3 / 1000.0
                        mcu.err = p4 / 1000.0

                    case 1:  # TELEMETRY 1
                        mcu = self.telemetry.axes[can_node_id - 1]
                        mcu.driver_ok = bool(p1)
                        mcu.current = int(p2)
                        mcu.load = int(p3)
                        mcu.over_temperature = bool(p4)

                    case 2:  # Smart IO
                        input_id = p1
                        smart_event = p2
                        self.io_controller.handle_input_event(can_node_id, input_id, smart_event)

                    case 3:  # PID
                        mcu = self.config.axes[can_node_id - 1]
                        mcu.kp = p1 / 1000.0
                        mcu.ki = p2 / 1000.0
                        mcu.kd = p3 / 1000.0
                        mcu.kv = p4 / 1000.0

                    case 4:  # MECHANICS
                        mcu = self.config.axes[can_node_id - 1]
                        mcu.mot_gear_ratio = p1 / 1000.0
                        mcu.enc_gear_ratio = p2 / 1000.0

                    case 5:  # ENCODER
                        mcu = self.config.axes[can_node_id - 1]
                        mcu.home_offset = p1 / 1000.0

                    case 6:  # DRIVER
                        mcu = self.config.axes[can_node_id - 1]
                        mcu.micro_step = p1
                        mcu.max_current = p2

                    case 7:  # GUIDING
                        mcu = self.config.axes[can_node_id - 1]
                        mcu.guiding_gain = p1
                        mcu.guiding_deadzone = p2
                        mcu.error_limit = p3

                # Check and register CAN ACK
                if msg_type in (3, 4, 5, 6, 7):
                    with self._ack_lock:
                        key = (can_node_id, msg_type)
                        if key in self._pending_acks:
                            self._pending_acks.remove(key)
                            if not self._pending_acks:
                                self._ack_event.set()


if __name__ == "__main__":
    import numpy as np
    print("Start")
    hw = HWController()

    trajectory_time = 0.15

    def send_loop():
        start_time = time.time()
        while True:
            global trajectory_time
            current_real_time = time.time() - start_time
            lead_time = trajectory_time - current_real_time
            timestamp = int(trajectory_time * 1000) & 0xFFFF

            pos = 180 * np.sin(2 * 3.141 / 2 * trajectory_time)
            vel = 2 * 3.141 / 2 * 180 * np.cos(2 * 3.141 / 2 * trajectory_time)

            if lead_time < 0.150:
                hw.motion(timestamp, [0, pos, 0, 0, 0, 0], [0, vel, 0, 0, 0, 0])
                trajectory_time += 0.020
            else:
                time.sleep(0.010)

            hw.update_telemetry()

    hw.set_enable(True)
    hw.update_telemetry()
    hw.get_config()
    send_loop()
    print("finish")
