"""KiCad SWIG integration for the API-neutral Track Gloss engine."""

from .adapter import BoardAdapter
from .reader import SelectionSnapshot

__all__ = ("BoardAdapter", "SelectionSnapshot")
