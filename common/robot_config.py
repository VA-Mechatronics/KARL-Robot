from dataclasses import dataclass, field
from typing import List
import numpy as np
from common.dataclass_jsonify import DataclassJsonMixin


@dataclass
class MCUConfig(DataclassJsonMixin):
    can_node_id: int = 0
    home_offset: float = 0.0
    mot_gear_ratio: float = 1.0
    enc_gear_ratio: float = 1.0
    nom_velocity: float = 20.0
    max_velocity: float = 20.0
    min_pos: float = -360.0
    max_pos: float = 360.0
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    kv: float = 0.0
    micro_step: int = 16
    max_current: int = 400
    guiding_gain: int = 5000
    guiding_deadzone: int = 2
    error_limit: int = 5000


@dataclass
class ToolParam(DataclassJsonMixin):
    matrix: np.ndarray = field(
        default_factory=lambda: np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 10.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)
    )


@dataclass
class TcpOffsetParam(DataclassJsonMixin):
    matrix: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=float)
    )


@dataclass
class LinkParam(DataclassJsonMixin):
    B1: float = 292.5
    A2: float = 173.0
    B4: float = 218.0
    B6: float = 82.0


@dataclass
class RobotConfig(DataclassJsonMixin):
    axes: List[MCUConfig] = field(default_factory=list)
    tool: ToolParam = field(default_factory=ToolParam)
    tcp_offset: TcpOffsetParam = field(default_factory=TcpOffsetParam)
    link: LinkParam = field(default_factory=LinkParam)

    @classmethod
    def default(cls) -> "RobotConfig":
        return cls(
            axes=[MCUConfig(can_node_id=i) for i in range(1, 8)],
            tool=ToolParam(),
            tcp_offset=TcpOffsetParam(),
            link=LinkParam(),
        )