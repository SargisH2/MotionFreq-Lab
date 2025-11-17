"""In-memory storage for measurement samples."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from threading import Lock
from typing import Iterable, Iterator, List, Literal, Sequence

ModeLiteral = Literal["no_motor_time", "pos_from_zero", "neg_from_zero"]
PlotIdLiteral = Literal[1, 2, 3]


@dataclass(frozen=True, slots=True)
class MeasurementRow:
    """Single measurement sample stored by the UI."""

    timestamp: datetime
    elapsed_s: float
    elapsed_ms: int
    axis: Literal["X", "Y"]
    coordinate: float
    ch1_freq: float
    ch2_freq: float
    mode: ModeLiteral
    plot_id: PlotIdLiteral


class MeasurementBuffer:
    """Thread-safe buffer holding measurement rows for a single plot."""

    def __init__(self, plot_id: PlotIdLiteral) -> None:
        self._plot_id = plot_id
        self._rows: List[MeasurementRow] = []
        self._lock = Lock()

    def append(self, row: MeasurementRow) -> None:
        if row.plot_id != self._plot_id:
            raise ValueError(f"Row plot_id {row.plot_id} does not match buffer id {self._plot_id}")
        with self._lock:
            self._rows.append(row)

    def clear(self) -> None:
        with self._lock:
            self._rows.clear()

    def __len__(self) -> int:
        with self._lock:
            return len(self._rows)

    def snapshot(self) -> List[MeasurementRow]:
        with self._lock:
            return list(self._rows)

    def to_dataframe(self):
        """Return the buffer contents as a pandas DataFrame."""

        try:
            import pandas as pd  # type: ignore[import]
        except Exception as exc:  # pragma: no cover - optional dependency
            raise RuntimeError("pandas is required to export measurement buffers") from exc

        with self._lock:
            data = [
                {
                    "timestamp": row.timestamp,
                    "elapsed_ms": row.elapsed_ms,
                    "elapsed_s": row.elapsed_s,
                    "axis": row.axis,
                    "coordinate": row.coordinate,
                    "ch1_freq": row.ch1_freq,
                    "ch2_freq": row.ch2_freq,
                    "mode": row.mode,
                    "plot_id": row.plot_id,
                }
                for row in self._rows
            ]
        return pd.DataFrame(data)


__all__ = ["MeasurementBuffer", "MeasurementRow", "ModeLiteral", "PlotIdLiteral"]
