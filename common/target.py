from dataclasses import dataclass, field
import numpy as np
from typing import Union, Dict, Any
from scipy.spatial.transform import Rotation as R
from common.dataclass_jsonify import DataclassJsonMixin

np.set_printoptions(suppress=True, precision=3, formatter={'float_kind': '{: 0.3f}'.format})


@dataclass
class Target(DataclassJsonMixin):
    frame: str = "reserved"
    joints: np.ndarray = field(default_factory=lambda: np.full(7, np.nan))
    tcp: np.ndarray = field(default_factory=lambda: np.full((4, 4), np.nan))
    shoulder: str = "left"
    elbow: str = "up"
    wrist: str | None = None

    def __post_init__(self):
        # Convert list inputs or missing shapes automatically during init
        self.joints = self._parse_joints(self.joints)
        self.tcp = self._parse_tcp(self.tcp)

    @staticmethod
    def _parse_joints(value) -> np.ndarray:
        if value is None:
            return np.full(7, np.nan)
        val_list = list(value)
        if len(val_list) > 7:
            raise ValueError(f"joints can contain at most 7 values, got {len(val_list)}")
        val_list += [None] * (7 - len(val_list))
        return np.array([np.nan if j is None else float(j) for j in val_list])

    @staticmethod
    def _parse_tcp(value) -> np.ndarray:
        if value is None:
            return np.full((4, 4), np.nan)
        mat = np.asarray(value, dtype=float)
        if mat.shape != (4, 4):
            raise ValueError(f"tcp matrix must have shape (4, 4), got {mat.shape}")
        return mat

    @property
    def has_tcp(self) -> bool:
        return not np.isnan(self.tcp).any()

    # region *** Operator Overloading (@) ***
    def __matmul__(self, other: Union["Target", np.ndarray]) -> "Target":
        if not self.has_tcp:
            raise ValueError("Cannot multiply Target without valid TCP matrix.")
        new_target = self.copy()

        if isinstance(other, Target):
            if not other.has_tcp:
                raise ValueError("Cannot multiply with target without valid TCP matrix.")
            new_target.tcp = self.tcp @ other.tcp
        else:
            mat = np.asarray(other, dtype=float)
            if mat.shape != (4, 4):
                raise ValueError(f"Expected 4x4 matrix, got {mat.shape}")
            new_target.tcp = self.tcp @ mat
        return new_target

    def __rmatmul__(self, other: np.ndarray) -> "Target":
        if not self.has_tcp:
            raise ValueError("Cannot multiply Target without valid TCP matrix.")
        mat = np.asarray(other, dtype=float)
        if mat.shape != (4, 4):
            raise ValueError(f"Expected 4x4 matrix, got {mat.shape}")
        new_target = self.copy()
        new_target.tcp = mat @ self.tcp
        return new_target
    # endregion

    def offset(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
               A: float = 0.0, B: float = 0.0, C: float = 0.0,
               frame: Union[str, "Target"] = "self") -> "Target":
        """Returns a NEW Target shifted by the offset."""
        new_target = self.copy()
        new_target.offset_(x=x, y=y, z=z, A=A, B=B, C=C, frame=frame)
        return new_target

    def offset_(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                A: float = 0.0, B: float = 0.0, C: float = 0.0,
                frame: Union[str, "Target"] = "self") -> "Target":
        """Applies offset IN-PLACE to this Target instance."""
        if not self.has_tcp:
            return self

        delta_M = np.eye(4, dtype=float)
        delta_M[0:3, 3] = [x, y, z]

        if any([A, B, C]):
            rot_mat = R.from_euler("xyz", [A, B, C], degrees=True).as_matrix()
            delta_M[0:3, 0:3] = rot_mat

        if isinstance(frame, Target):
            if not frame.has_tcp:
                raise ValueError("Referenced frame Target does not have a valid TCP.")
            self.tcp = frame.tcp @ delta_M @ np.linalg.inv(frame.tcp) @ self.tcp
        elif isinstance(frame, str):
            f_lower = frame.lower()
            if f_lower in ("self", "tool"):
                self.tcp = self.tcp @ delta_M
            elif f_lower == "base":
                self.tcp = delta_M @ self.tcp
            else:
                raise ValueError(f"Unknown frame option '{frame}'")

        return self

    def __str__(self) -> str:
        return (f"frame: {self.frame}\n"
                f"joints: {self.joints}\n"
                f"tcp:\n{self.tcp}\n"
                f"shoulder/elbow/wrist: {self.shoulder}/{self.elbow}/{self.wrist}")