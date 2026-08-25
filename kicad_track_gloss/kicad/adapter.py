"""Small facade joining KiCad readers/writers to the API-neutral engine."""

from __future__ import annotations

from ..engine.model import Segment
from .reader import read_snapshot
from .selection import expand_eligible_keys, expand_seed_keys
from .writer import add_track, apply_plan


def _uuid(item):
    try:
        return item.m_Uuid.AsString()
    except Exception:
        try:
            return item.GetUuid().AsString()
        except Exception:
            return ""


def _net_name(item):
    try:
        return str(item.GetNetname() or "")
    except Exception:
        return ""


class BoardAdapter:
    """Public facade used by the ActionPlugin and headless diagnostics."""

    def __init__(self, pcbnew_module):
        self.pcbnew = pcbnew_module

    def to_mm(self, value):
        try:
            return float(self.pcbnew.ToMM(value))
        except Exception:
            return float(value) / 1_000_000.0

    def point_mm(self, point):
        return self.to_mm(point.x), self.to_mm(point.y)

    def vector(self, point):
        x = int(round(self.pcbnew.FromMM(point[0])))
        y = int(round(self.pcbnew.FromMM(point[1])))
        return self.pcbnew.VECTOR2I(x, y)

    def segment_from_item(self, item):
        start, end = self.point_mm(item.GetStart()), self.point_mm(item.GetEnd())
        try:
            clearance = self.to_mm(item.GetOwnClearance(item.GetLayer()))
        except Exception:
            clearance = -1.0
        return Segment(start[0], start[1], end[0], end[1],
                       self.to_mm(item.GetWidth()), int(item.GetLayer()),
                       int(item.GetNetCode()), _uuid(item), bool(item.IsLocked()),
                       str(item.GetClass()) == "PCB_ARC", _net_name(item),
                       clearance)

    # Compatibility alias kept for existing diagnostic/test callers.
    _segment_from_item = segment_from_item

    def _expand_seed_keys(self, board, straight_by_key, seed_keys, warnings):
        return expand_seed_keys(
            self, board, straight_by_key, seed_keys, warnings)

    def expand_eligible_keys(self, board, straight_by_key, seed_keys, warnings=None):
        return expand_eligible_keys(
            self, board, straight_by_key, seed_keys, warnings)

    def snapshot(self, board, require_selection=True):
        return read_snapshot(self, board, require_selection)

    def apply(self, board, result, rollback_on_error=True):
        return apply_plan(self, board, result, rollback_on_error)

    def _add_track(self, board, start, end, width, layer, net_id):
        return add_track(self, board, start, end, width, layer, net_id)
