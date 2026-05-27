"""Dialog-settings persistence for the IPC plugin.

Each IPC plugin invocation runs in its own process, so settings have to
land on disk if we want them to survive between launches. Both the
normal exit path (in `ipc_entry`) and the emergency exit path (in
`routing_dialog._kicad_died`) call into here, so the JSON serialisation
lives in exactly one place.
"""

from __future__ import annotations

import json
import os
from typing import Any


_SETTINGS_PATH = os.path.expanduser("~/.kicad_routing_tools_settings.json")


def path() -> str:
    """Filesystem location of the settings JSON."""
    return _SETTINGS_PATH


def load() -> dict:
    """Read previously-persisted dialog settings, or {} on first launch."""
    try:
        with open(_SETTINGS_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except Exception:
        return {}


def _coerce(obj: Any) -> Any:
    """JSON has no tuples or sets — coerce to lists, recursively."""
    if isinstance(obj, tuple):
        return [_coerce(x) for x in obj]
    if isinstance(obj, list):
        return [_coerce(x) for x in obj]
    if isinstance(obj, dict):
        return {k: _coerce(v) for k, v in obj.items()}
    if isinstance(obj, set):
        return [_coerce(x) for x in obj]
    return obj


def save(settings: dict) -> bool:
    """Persist dialog settings to disk. Returns True on success."""
    try:
        with open(_SETTINGS_PATH, "w", encoding="utf-8") as f:
            json.dump(_coerce(settings), f, indent=2, default=str)
        return True
    except Exception:
        return False
