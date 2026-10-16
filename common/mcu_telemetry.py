from dataclasses import dataclass, field
from typing import List
from common.dataclass_jsonify import DataclassJsonMixin


@dataclass
class MCUTelemetry(DataclassJsonMixin):
    echo: int = 0
    act_pos: float = 0.0
    set_pos: float = 0.0
    act_vel: float = 0.0
    output: float = 0.0
    err: float = 0.0
    driver_ok: bool = False
    current: float = 0.0
    load: float = 0.0
    over_temperature: bool = False


@dataclass
class RobotTelemetry(DataclassJsonMixin):
    axes: List[MCUTelemetry] = field(default_factory=list)

    @classmethod
    def default(cls) -> "RobotTelemetry":
        default_pos = [0.01, 120.0, 200.0, 0.01, 0.01, 0.01, 0.01]
        return cls(axes=[MCUTelemetry(act_pos=pos) for pos in default_pos])
