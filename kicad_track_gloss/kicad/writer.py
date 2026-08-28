"""Apply immutable Track Gloss plans to a live KiCad board with rollback."""

from __future__ import annotations

from collections import Counter

from ..engine.model import segment_key
from .types import is_straight_track


def _track_map(adapter, board):
    mapping = {}
    for item in board.GetTracks():
        if is_straight_track(adapter.pcbnew, item):
            segment = adapter.segment_from_item(item)
            mapping[segment_key(segment)] = item
    return mapping


def add_track(adapter, board, start, end, width, layer, net_id):
    track = adapter.pcbnew.PCB_TRACK(board)
    track.SetStart(adapter.vector(start))
    track.SetEnd(adapter.vector(end))
    track.SetWidth(adapter.from_mm(width))
    track.SetLayer(layer)
    track.SetNetCode(net_id)
    board.Add(track)
    return track


def _native_copper_signature(item):
    """Return the exact KiCad-IU identity of one straight copper segment."""
    start = (int(item.GetStart().x), int(item.GetStart().y))
    end = (int(item.GetEnd().x), int(item.GetEnd().y))
    first, second = sorted((start, end))
    return (first, second, int(item.GetWidth()), int(item.GetLayer()),
            int(item.GetNetCode()))


def _addition_signature(adapter, addition):
    start = adapter.vector(addition.start)
    end = adapter.vector(addition.end)
    first, second = sorted(((int(start.x), int(start.y)),
                            (int(end.x), int(end.y))))
    return (first, second, adapter.from_mm(addition.width),
            int(addition.layer), int(addition.net_id))


def _verify_applied_plan(adapter, board, result, created):
    """Read the live board back and prove that the requested edit exists."""
    after = _track_map(adapter, board)
    retained_removed = set(result.remove_keys) & set(after)
    if retained_removed:
        raise RuntimeError(
            "Post-apply copper readback found removed tracks still present")

    created_keys = {segment_key(adapter.segment_from_item(item))
                    for item in created}
    missing_created = created_keys - set(after)
    if missing_created:
        raise RuntimeError(
            "Post-apply copper readback could not find newly added tracks")

    expected = Counter(_addition_signature(adapter, addition)
                       for addition in result.additions)
    observed = Counter(_native_copper_signature(after[key])
                       for key in created_keys)
    if observed != expected:
        raise RuntimeError(
            "Post-apply copper readback differs from the requested geometry")


def apply_plan(adapter, board, result, rollback_on_error=True):
    """Apply a prevalidated plan; restore removed copper on any exception."""
    mapping = _track_map(adapter, board)
    missing = [key for key in result.remove_keys if key not in mapping]
    if missing:
        raise RuntimeError("Selected tracks changed before apply; aborting safely.")
    removed_specs = []
    created = []
    try:
        for key in result.remove_keys:
            item = mapping[key]
            removed_specs.append((
                adapter.point_mm(item.GetStart()), adapter.point_mm(item.GetEnd()),
                adapter.to_mm(item.GetWidth()), int(item.GetLayer()),
                int(item.GetNetCode())))
            board.RemoveNative(item)
        for spec in result.additions:
            created.append(add_track(
                adapter, board, spec.start, spec.end, spec.width,
                spec.layer, spec.net_id))
        _verify_applied_plan(adapter, board, result, created)
        return created
    except Exception:
        if rollback_on_error:
            for item in created:
                try:
                    board.RemoveNative(item)
                except Exception:
                    pass
            for start, end, width, layer, net_id in removed_specs:
                add_track(adapter, board, start, end, width, layer, net_id)
        raise
