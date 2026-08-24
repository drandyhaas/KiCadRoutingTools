"""Convert the current KiCad board and selection into an engine snapshot."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..engine.model import BoardModel, CircleObstacle, segment_key
from .rules import board_bounds, native_rules, track_keepouts
from .selection import expand_eligible_keys, is_probable_diff_pair


@dataclass
class SelectionSnapshot:
    model: BoardModel
    eligible_keys: set
    warnings: list = field(default_factory=list)
    minimum_clearance: float = 0.1
    copper_edge_clearance: float = 0.0
    selection_seed_count: int = 0
    auto_expanded_count: int = 0


def read_snapshot(adapter, board, require_selection=True):
    segments, obstacles, warnings = [], [], []
    straight_by_key = {}
    seed_keys = set()
    for item in board.GetTracks():
        kind = str(item.GetClass())
        if kind == "PCB_VIA":
            x, y = adapter.point_mm(item.GetPosition())
            try:
                diameter = adapter.to_mm(item.GetFrontWidth())
            except Exception:
                diameter = adapter.to_mm(item.GetWidth())
            layers = tuple(range(int(item.TopLayer()), int(item.BottomLayer()) + 1))
            obstacles.append(CircleObstacle(x, y, diameter / 2.0,
                                            int(item.GetNetCode()), layers, "via"))
            if item.IsSelected():
                warnings.append("Selected vias are protected and will not be modified.")
            continue
        if kind not in ("PCB_TRACK", "PCB_ARC"):
            continue
        segment = adapter.segment_from_item(item)
        segments.append(segment)
        key = segment_key(segment)
        if kind == "PCB_TRACK":
            straight_by_key[key] = (item, segment)
        if not item.IsSelected():
            continue
        if kind == "PCB_ARC":
            warnings.append("Selected arcs are protected in this version.")
        elif segment.locked:
            warnings.append("Selected locked tracks are protected.")
        elif is_probable_diff_pair(segment.net_name):
            warnings.append(
                "Probable differential-pair tracks are protected: " + segment.net_name)
        else:
            seed_keys.add(key)

    eligible, expanded, _meanders = expand_eligible_keys(
        adapter, board, straight_by_key, seed_keys, warnings)
    expanded_count = max(0, len(expanded) - len(seed_keys))

    try:
        footprints = board.GetFootprints()
    except Exception:
        footprints = ()
    minimum, edge, net_clearances = native_rules(adapter, board, segments)
    copper_layers = tuple(range(0, 32))
    for footprint in footprints:
        for pad in footprint.Pads():
            x, y = adapter.point_mm(pad.GetPosition())
            size = pad.GetSize()
            radius = max(adapter.to_mm(size.x), adapter.to_mm(size.y)) / 2.0
            try:
                layers = [int(layer) for layer in pad.GetLayerSet().Seq()
                          if str(board.GetLayerName(layer)).endswith(".Cu")]
            except Exception:
                layers = list(copper_layers)
            try:
                local_clearance = adapter.to_mm(pad.GetLocalClearance())
            except Exception:
                local_clearance = 0.0
            obstacles.append(CircleObstacle(
                x, y, radius, int(pad.GetNetCode()), tuple(layers),
                "pad", local_clearance))

    if require_selection and not seed_keys:
        if warnings:
            raise ValueError("No eligible straight track is selected. " +
                             " ".join(sorted(set(warnings))))
        raise ValueError("Select at least two connected straight track segments first.")
    model = BoardModel(segments, obstacles, track_keepouts(adapter, board),
                       net_clearances, minimum, edge, board_bounds(adapter, board))
    return SelectionSnapshot(model, eligible, sorted(set(warnings)), minimum, edge,
                             len(seed_keys), expanded_count)
