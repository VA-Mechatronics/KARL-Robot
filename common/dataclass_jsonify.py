import json
import copy
from dataclasses import dataclass, is_dataclass, fields
from pathlib import Path
from typing import TypeVar, Type, Any
import numpy as np

T = TypeVar("T", bound="DataclassJsonMixin")


class DataclassJsonMixin:
    """Central mixin providing unified JSON/Dict serialization, file I/O, and copying."""

    def to_dict(self) -> dict[str, Any]:
        """Convert dataclass or class attributes to dict handling numpy arrays and nested objects."""
        result = {}
        for f in fields(self):
            val = getattr(self, f.name)
            result[f.name] = self._serialize_value(val)
        return result

    @classmethod
    def _serialize_value(cls, val: Any) -> Any:
        if isinstance(val, np.ndarray):
            return val.tolist()
        elif is_dataclass(val) and hasattr(val, "to_dict"):
            return val.to_dict()
        elif isinstance(val, list):
            return [cls._serialize_value(item) for item in val]
        elif isinstance(val, dict):
            return {k: cls._serialize_value(v) for k, v in val.items()}
        return val

    @classmethod
    def from_dict(cls: Type[T], data: dict[str, Any]) -> T:
        """Construct instance from dict with automatic nested conversion."""
        if not isinstance(data, dict):
            return data

        kwargs = {}
        for f in fields(cls):
            if f.name in data:
                val = data[f.name]
                field_type = f.type

                # Nested dataclass handling
                if is_dataclass(field_type) and hasattr(field_type, "from_dict"):
                    kwargs[f.name] = field_type.from_dict(val)
                elif hasattr(field_type, "__origin__") and field_type.__origin__ is list:
                    elem_type = field_type.__args__[0]
                    if is_dataclass(elem_type) and hasattr(elem_type, "from_dict"):
                        kwargs[f.name] = [elem_type.from_dict(item) for item in val]
                    else:
                        kwargs[f.name] = val
                elif field_type == np.ndarray or "ndarray" in str(field_type):
                    kwargs[f.name] = np.array(val, dtype=float) if val is not None else None
                else:
                    kwargs[f.name] = val

        return cls(**kwargs)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=4)

    @classmethod
    def from_json(cls: Type[T], json_str: str) -> T:
        return cls.from_dict(json.loads(json_str))

    def write_to_file(self, filepath: str | Path) -> None:
        path = Path(filepath)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=4)

    @classmethod
    def read_from_file(cls: Type[T], filepath: str | Path) -> T:
        with open(filepath, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    def copy(self: T) -> T:
        """Universal deep copy for all data models."""
        return copy.deepcopy(self)