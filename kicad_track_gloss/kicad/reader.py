"""Convert the current KiCad board and selection into an engine snapshot."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from ..engine.model import BoardModel, CircleObstacle, PadRegion, segment_key
from .rules import (board_bounds, copper_layers, exact_board_outline,
                    mask_graphic_keepouts, native_rules, track_keepouts,
                    via_track_hole_clearance)
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
    tuned_protected_count: int = 0


def _via_copper_layers(adapter, board, item):
    try:
        return copper_layers(adapter, board, item.GetLayerSet())
    except Exception:
        # Older SWIG builds may not expose the via layer set. Keep the
        # endpoints as a conservative fallback, without inventing a
        # contiguous PCB_LAYER_ID range (internal copper IDs are not
        # contiguous in KiCad).
        return tuple(sorted({int(item.TopLayer()), int(item.BottomLayer())}))


def read_snapshot(adapter, board, require_selection=True):
    segments, obstacles, pad_regions, warnings = [], [], [], []
    straight_by_key = {}
    seed_keys = set()
    try:
        board.InitializeClearanceCache()
    except Exception:
        pass
    via_hole_clearance = via_track_hole_clearance(adapter, board)
    for item in board.GetTracks():
        kind = str(item.GetClass())
        if kind == "PCB_VIA":
            x, y = adapter.point_mm(item.GetPosition())
            try:
                diameter = adapter.to_mm(item.GetFrontWidth())
            except Exception:
                diameter = adapter.to_mm(item.GetWidth())
            layers = _via_copper_layers(adapter, board, item)
            obstacles.append(CircleObstacle(x, y, diameter / 2.0,
                                            int(item.GetNetCode()), layers, "via"))
            if via_hole_clearance > 0.0:
                try:
                    drill_radius = adapter.to_mm(item.GetDrillValue()) / 2.0
                except Exception:
                    drill_radius = 0.0
                if drill_radius > 0.0:
                    obstacles.append(CircleObstacle(
                        x, y, drill_radius, int(item.GetNetCode()), layers,
                        "via", via_hole_clearance))
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

    eligible, expanded, meanders = expand_eligible_keys(
        adapter, board, straight_by_key, seed_keys, warnings)
    expanded_count = max(0, len(expanded) - len(seed_keys))

    try:
        footprints = board.GetFootprints()
    except Exception:
        footprints = ()
    minimum, edge, net_clearances = native_rules(adapter, board, segments)
    fallback_copper_layers = tuple(range(0, 32))
    for footprint in footprints:
        for pad in footprint.Pads():
            x, y = adapter.point_mm(pad.GetPosition())
            size = pad.GetSize()
            half_width = adapter.to_mm(size.x) / 2.0
            half_height = adapter.to_mm(size.y) / 2.0
            shape_names = {
                0: "circle", 1: "rect", 2: "oval", 3: "fallback",
                4: "roundrect", 5: "fallback", 6: "fallback",
            }
            shape = shape_names.get(int(pad.GetShape()), "fallback")
            # Circumscribe the full pad bounding box, including rectangular
            # corners. ``max(width, height) / 2`` is not an enclosing circle.
            radius = math.hypot(half_width, half_height)
            try:
                layers = list(copper_layers(
                    adapter, board, pad.GetLayerSet()))
            except Exception:
                layers = list(fallback_copper_layers)
            # Paste/mask-only apertures are not copper obstacles. An empty
            # layer tuple means "all layers" inside the API-neutral model, so
            # retaining such a pad would incorrectly block every copper layer.
            if not layers:
                continue
            try:
                local_clearance = adapter.to_mm(pad.GetLocalClearance())
            except Exception:
                local_clearance = 0.0
            try:
                corner_radius = adapter.to_mm(pad.GetRoundRectCornerRadius())
            except Exception:
                corner_radius = 0.0
            if shape != "fallback":
                pad_regions.append(PadRegion(
                    x, y, adapter.to_mm(size.x), adapter.to_mm(size.y),
                    float(pad.GetOrientationDegrees()), shape, corner_radius,
                    int(pad.GetNetCode()), tuple(layers), local_clearance))
            else:
                # Unsupported/custom pads retain a conservative enclosing
                # circle, but it must cover their effective primitives rather
                # than only the usually smaller anchor-pad size.
                try:
                    box = pad.GetBoundingBox()
                    box_width = adapter.to_mm(box.GetWidth())
                    box_height = adapter.to_mm(box.GetHeight())
                    obstacle_x = adapter.to_mm(box.GetX()) + box_width / 2.0
                    obstacle_y = adapter.to_mm(box.GetY()) + box_height / 2.0
                    obstacle_radius = math.hypot(
                        box_width / 2.0, box_height / 2.0)
                except Exception:
                    obstacle_x, obstacle_y = x, y
                    obstacle_radius = radius
                obstacles.append(CircleObstacle(
                    obstacle_x, obstacle_y, obstacle_radius,
                    int(pad.GetNetCode()), tuple(layers), "pad",
                    local_clearance))

    if require_selection and not seed_keys:
        if warnings:
            raise ValueError("No eligible straight track is selected. " +
                             " ".join(sorted(set(warnings))))
        raise ValueError("Select at least two connected straight track segments first.")
    keepouts = track_keepouts(adapter, board)
    keepouts.extend(mask_graphic_keepouts(adapter, board))
    model = BoardModel(segments, obstacles, keepouts,
                       net_clearances, minimum, edge, board_bounds(adapter, board),
                       pad_regions, exact_board_outline(adapter, board))
    return SelectionSnapshot(model, eligible, sorted(set(warnings)), minimum, edge,
                             len(seed_keys), expanded_count, len(meanders))
