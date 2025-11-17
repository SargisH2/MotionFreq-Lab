"""Shared helpers for motor-related UI components."""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence


SIMULATED_ALIAS = "Simulated Motor"


def dedupe_preserve(values: Iterable[str]) -> List[str]:
    """Return values in original order with duplicates removed."""

    seen: set[str] = set()
    result: List[str] = []
    for item in values:
        if item not in seen:
            seen.add(item)
            result.append(item)
    return result


def prepare_port_list(
    ports: Iterable[str],
    *,
    is_simulation: bool,
    simulated_label: str = SIMULATED_ALIAS,
) -> List[str]:
    """Normalise a raw port list for display in UI controls."""

    normalised = dedupe_preserve(ports)
    if not is_simulation:
        return normalised
    transformed = [simulated_label if p.lower().startswith("simulated") else p for p in normalised]
    if simulated_label not in transformed:
        transformed.append(simulated_label)
    return transformed


def select_port_value(
    values: Sequence[str],
    current: Optional[str],
    preferred: Optional[str] = None,
) -> str:
    """Pick the most appropriate combobox value given current/preferred choices."""

    for candidate in (current, preferred):
        if candidate and candidate in values:
            return candidate
    return values[0] if values else ""


__all__ = [
    "SIMULATED_ALIAS",
    "dedupe_preserve",
    "prepare_port_list",
    "select_port_value",
]

