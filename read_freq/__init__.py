"""Python helper package for the VW frequency controller."""

from typing import Any

try:
    from .controller import (
        ChannelMeasurement,
        ControllerError,
        FrequencyFrame,
        VWFrequencyController,
        format_frame,
    )
except ImportError as exc:  # pragma: no cover - missing dependency
    ControllerError = RuntimeError  # type: ignore[assignment]
    VWFrequencyController = None  # type: ignore[assignment]

    class ChannelMeasurement:  # type: ignore[assignment]
        """Placeholder type when controller dependencies are missing."""

    class FrequencyFrame:  # type: ignore[assignment]
        """Placeholder type when controller dependencies are missing."""

    def format_frame(_: Any) -> str:  # type: ignore[assignment]
        return "Controller unavailable; install 'pyserial'."

__all__ = [
    "ChannelMeasurement",
    "ControllerError",
    "FrequencyFrame",
    "VWFrequencyController",
    "format_frame",
]
