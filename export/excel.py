"""Helper for exporting measurement buffers to Excel."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Mapping, Sequence, Union

from models.data_buffers import MeasurementBuffer, MeasurementRow

LOGGER = logging.getLogger("measurements.export")

SheetData = Union[MeasurementBuffer, Sequence[MeasurementRow]]


def export_measurements_to_excel(buffers: Mapping[int, SheetData], path: str) -> Path:
    """Write the provided buffers to an Excel workbook.

    Parameters
    ----------
    buffers:
        Mapping whose keys are the plot identifiers (1-3) and values are either
        :class:`MeasurementBuffer` instances or sequences of
        :class:`MeasurementRow`.
    path:
        Destination filename. The parent directory must exist.

    Returns
    -------
    :class:`Path`
        The resolved filesystem path of the newly written workbook.
    """

    try:
        import pandas as pd  # type: ignore[import]
    except Exception as exc:  # pragma: no cover - dependency
        raise RuntimeError("pandas must be installed to export measurements") from exc

    destination = Path(path).expanduser().resolve()
    if not destination.parent.exists():
        raise FileNotFoundError(f"Destination directory does not exist: {destination.parent}")

    LOGGER.info("Exporting measurements to %s", destination)

    with pd.ExcelWriter(destination, engine="openpyxl") as writer:  # type: ignore[arg-type]
        for plot_id in (1, 2, 3):
            sheet_name = f"Plot{plot_id}"
            sheet_data = buffers.get(plot_id, [])
            if isinstance(sheet_data, MeasurementBuffer):
                frame = sheet_data.to_dataframe()
            else:
                rows = list(sheet_data)
                frame = pd.DataFrame(
                    [
                        {
                            "timestamp": row.timestamp,
                            "elapsed_ms": row.elapsed_ms,
                            "axis": row.axis,
                            "coordinate": row.coordinate,
                            "ch1_freq": row.ch1_freq,
                            "ch2_freq": row.ch2_freq,
                            "mode": row.mode,
                            "plot_id": row.plot_id,
                        }
                        for row in rows
                    ]
                )
            if "elapsed_ms" in frame.columns:
                frame.insert(1, "elapsed_ms", frame.pop("elapsed_ms"))
            if "elapsed_s" in frame.columns:
                frame = frame.drop(columns=["elapsed_s"])
            frame.to_excel(writer, index=False, sheet_name=sheet_name)

    return destination


__all__ = ["export_measurements_to_excel"]
