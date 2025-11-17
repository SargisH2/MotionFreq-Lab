"""Environment loader using python-dotenv for application settings."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

try:  # pragma: no cover - optional dependency guard
    from dotenv import load_dotenv  # type: ignore[import]
except ImportError:  # pragma: no cover
    def load_dotenv(*args, **kwargs):  # type: ignore[override]
        return False

_ENV_INITIALISED = False


def ensure_env_loaded(dotenv_path: Optional[Path] = None) -> None:
    """Load the .env file once."""

    global _ENV_INITIALISED
    if _ENV_INITIALISED:
        return

    if dotenv_path is None:
        dotenv_path = Path(__file__).resolve().parent.parent / ".env"

    if dotenv_path.exists():
        load_dotenv(dotenv_path=dotenv_path, override=False)
    else:
        load_dotenv(override=False)
    _ENV_INITIALISED = True


def env_bool(name: str, default: bool) -> bool:
    """Return an environment variable interpreted as boolean."""

    ensure_env_loaded()
    raw = os.getenv(name)
    if raw is None:
        return default
    value = raw.strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    return default


__all__ = ["env_bool", "ensure_env_loaded"]
