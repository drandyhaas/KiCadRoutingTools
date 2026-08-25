"""Apply immutable Track Gloss plans to a live KiCad board with rollback."""

from __future__ import annotations

from ..engine.model import segment_key


def _track_map(adapter, board):
    mapping = {}
    for item in board.GetTracks():
        if str(item.GetClass()) == "PCB_TRACK":
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
