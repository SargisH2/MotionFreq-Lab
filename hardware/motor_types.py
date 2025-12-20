"""Shared motor-related types and helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Literal, Optional, Tuple, cast

AXES: Tuple[str, ...] = ("X", "Y")
AxisLiteral = Literal["X", "Y"]


class MotorError(RuntimeError):
    """Raised when a motor command cannot be completed."""


@dataclass(frozen=True)
class MotorState:
    """Immutable snapshot of the controller state."""

    connected: bool
    port: Optional[str]
    axis: Optional[str]
    moving: bool
    direction: int
    speed: float
    positions: Dict[str, float]
    last_error: Optional[str]


def normalise_axis(axis: str) -> AxisLiteral:
    """Return the validated, upper-case axis label."""

    if not axis:
        raise MotorError("Axis must be provided.")
    upper = axis.upper()
    if upper not in AXES:
        raise MotorError(f"Unsupported axis '{axis}'. Expected one of {AXES}.")
    return cast(AxisLiteral, upper)


__all__ = ["AXES", "AxisLiteral", "MotorError", "MotorState", "normalise_axis"]
