import sys
import time
import signal
import logging
import pandas as pd
import zmq
from pathlib import Path
from common.mcu_telemetry import RobotTelemetry

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(PROJECT_ROOT))

log = logging.getLogger(__name__)

LOG_DIR = PROJECT_ROOT / "workspace" / "telemetry_logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)


class TelemetryLoggerService:
    def __init__(self, sub_addr: str = "tcp://localhost:5556"):
        self.sub_addr = sub_addr
        self.ctx = zmq.Context()
        self.sub = None
        self.running = False
        self.records = []

        signal.signal(signal.SIGINT, self._handle_shutdown)
        signal.signal(signal.SIGTERM, self._handle_shutdown)

    def _handle_shutdown(self, signum, frame):
        log.info("Shutdown signal received. Stopping logging service...")
        self.stop()

    def stop(self):
        self.running = False

    def start(self):
        self.running = True
        self.records.clear()

        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.setsockopt(zmq.RCVTIMEO, 500)
        self.sub.connect(self.sub_addr)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")

        log.info(f"Connected to ZMQ Telemetry stream at {self.sub_addr}")
        log.info("Logging active...")

        start_wall_time = time.time()

        while self.running:
            try:
                msg = self.sub.recv_json()
                if isinstance(msg, dict) and msg.get("event") == "telemetry":
                    payload = msg.get("payload")
                    telemetry = RobotTelemetry.from_dict(payload)
                    now = time.time()

                    row = {"timestamp": now, "rel_time": now - start_wall_time}
                    for i, axis in enumerate(telemetry.axes):
                        prefix = f"j{i+1}_"
                        row[f"{prefix}act_pos"] = axis.act_pos
                        row[f"{prefix}set_pos"] = axis.set_pos
                        row[f"{prefix}act_vel"] = axis.act_vel
                        row[f"{prefix}output"] = axis.output
                        row[f"{prefix}err"] = axis.err
                        row[f"{prefix}current"] = axis.current
                        row[f"{prefix}load"] = axis.load
                        row[f"{prefix}driver_ok"] = axis.driver_ok
                        row[f"{prefix}over_temp"] = axis.over_temperature

                    self.records.append(row)

            except zmq.error.Again:
                pass
            except Exception as e:
                log.error(f"Error reading telemetry: {e}")

        self.save_parquet()

    def save_parquet(self):
        if not self.records:
            log.warning("No records collected. Skipping file export.")
            return

        df = pd.DataFrame(self.records)
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        filename = LOG_DIR / f"telemetry_{timestamp_str}.parquet"

        log.info(f"Saving {len(df)} rows to {filename}...")
        df.to_parquet(filename, engine="pyarrow", compression="snappy")
        log.info(f"Successfully saved: {filename}")


if __name__ == "__main__":
    from common.logger import setup_logging
    setup_logging()

    sub_address = sys.argv[1] if len(sys.argv) > 1 else "tcp://localhost:5556"
    service = TelemetryLoggerService(sub_addr=sub_address)
    service.start()
