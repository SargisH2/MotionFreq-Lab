"""Shared motor configuration loader/saver."""

from __future__ import annotations

import json
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Any, Dict


def _config_path() -> Path:
    return Path(__file__).resolve().with_name("motor.json")


@dataclass
class MotorConfig:
    """Represents persisted defaults for motor-related UI."""

    default_axis: str = "X"
    default_speed: int = 100
    default_port: str = ""
    default_baud: int = 115200
    homing_feed: float = 400.0
    homing_clearance: float = 3.0
    homing_long_jog: float = 1000.0
    jog_step: float = 1.0
    jog_feed: float = 600.0
    goto_feed: float = 600.0
    axis_enabled: Dict[str, bool] = field(default_factory=lambda: {"X": True, "Y": False})

    @classmethod
    def from_mapping(cls, data: Dict[str, Any]) -> "MotorConfig":
        axis_enabled = data.get("axis_enabled") or {"X": True, "Y": False}
        axis_enabled = {str(k).upper(): bool(v) for k, v in axis_enabled.items()}
        return cls(
            default_axis=str(data.get("default_axis", "X")).upper() or "X",
            default_speed=int(data.get("default_speed", 100)),
            default_port=str(data.get("default_port", "")),
            default_baud=int(data.get("default_baud", 115200)),
            homing_feed=float(data.get("homing_feed", 400.0)),
            homing_clearance=float(data.get("homing_clearance", 3.0)),
            homing_long_jog=float(data.get("homing_long_jog", 1000.0)),
            jog_step=float(data.get("jog_step", 1.0)),
            jog_feed=float(data.get("jog_feed", 600.0)),
            goto_feed=float(data.get("goto_feed", 600.0)),
            axis_enabled=axis_enabled,
        )

    def to_mapping(self) -> Dict[str, Any]:
        data = asdict(self)
        data["axis_enabled"] = {axis.upper(): bool(enabled) for axis, enabled in self.axis_enabled.items()}
        return data


def load_motor_config() -> MotorConfig:
    """Load motor defaults from disk, falling back to baked-in values."""

    path = _config_path()
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return MotorConfig()
    except Exception:
        return MotorConfig()
    if not isinstance(raw, dict):
        return MotorConfig()
    return MotorConfig.from_mapping(raw)


def save_motor_config(config: MotorConfig) -> None:
    """Persist the motor configuration to disk."""

    path = _config_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    data = config.to_mapping()
    temporary = path.with_suffix(".json.tmp")
    with temporary.open("w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, sort_keys=True)
        fh.write("\n")
    temporary.replace(path)


__all__ = ["MotorConfig", "load_motor_config", "save_motor_config"]
