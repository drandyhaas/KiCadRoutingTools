"""Convert the current KiCad board and selection into an engine snapshot."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from ..engine.model import BoardModel, CircleObstacle, PadRegion, segment_key
from .rules import (board_bounds, copper_layers, enabled_copper_layers,
                    exact_board_outline,
                    mask_graphic_keepouts, native_rules, track_keepouts,
                    via_track_hole_clearance)
from .selection import expand_eligible_keys, is_probable_diff_pair
from .types import is_arc, is_straight_track, is_via


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
        # Older SWIG builds may not expose the via layer set. Ask the via about
        # every enabled copper layer rather than assuming a numeric layer span.
        result = []
        for layer in enabled_copper_layers(adapter, board):
            try:
                if item.IsOnLayer(layer):
                    result.append(layer)
            except Exception:
                continue
        return tuple(result)


def _pad_shape(adapter, pad):
    try:
        value = int(pad.GetShape())
        shapes = (
            (adapter.pcbnew.PAD_SHAPE_CIRCLE, "circle"),
            (adapter.pcbnew.PAD_SHAPE_RECT, "rect"),
            (adapter.pcbnew.PAD_SHAPE_OVAL, "oval"),
            (adapter.pcbnew.PAD_SHAPE_ROUNDRECT, "roundrect"),
        )
        for native_shape, name in shapes:
            if value == int(native_shape):
                return name
    except (AttributeError, TypeError, ValueError):
        pass
    return "fallback"


def _line_chain_points(adapter, chain):
    return tuple(adapter.point_mm(chain.CPoint(index))
                 for index in range(chain.PointCount()))


def _custom_pad_regions(adapter, pad, layers, net_id, clearance):
    """Return exact effective copper polygons, one layer-specific region each."""
    regions = []
    for layer in layers:
        try:
            polyset = pad.GetEffectivePolygon(layer)
            polygons = []
            for outline_index in range(polyset.OutlineCount()):
                outer = _line_chain_points(
                    adapter, polyset.Outline(outline_index))
                holes = tuple(
                    _line_chain_points(
                        adapter, polyset.Hole(outline_index, hole_index))
                    for hole_index in range(polyset.HoleCount(outline_index)))
                if len(outer) >= 3:
                    polygons.append((outer, holes))
            if not polygons:
                continue
            points = [point for outer, _holes in polygons for point in outer]
            x0 = min(point[0] for point in points)
            y0 = min(point[1] for point in points)
            x1 = max(point[0] for point in points)
            y1 = max(point[1] for point in points)
            regions.append(PadRegion(
                (x0 + x1) / 2.0, (y0 + y1) / 2.0,
                x1 - x0, y1 - y0, 0.0, "custom", 0.0,
                net_id, (layer,), clearance, tuple(polygons)))
        except Exception:
            continue
    return regions


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
        if is_via(adapter.pcbnew, item):
            x, y = adapter.point_mm(item.GetPosition())
            try:
                diameter = adapter.to_mm(item.GetWidth(item.TopLayer()))
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
        arc = is_arc(adapter.pcbnew, item)
        if not arc and not is_straight_track(adapter.pcbnew, item):
            continue
        segment = adapter.segment_from_item(item)
        segments.append(segment)
        key = segment_key(segment)
        if not arc:
            straight_by_key[key] = (item, segment)
        if not item.IsSelected():
            continue
        if arc:
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
    for footprint in footprints:
        for pad in footprint.Pads():
            x, y = adapter.point_mm(pad.GetPosition())
            size = pad.GetSize()
            half_width = adapter.to_mm(size.x) / 2.0
            half_height = adapter.to_mm(size.y) / 2.0
            shape = _pad_shape(adapter, pad)
            # Circumscribe the full pad bounding box, including rectangular
            # corners. ``max(width, height) / 2`` is not an enclosing circle.
            radius = math.hypot(half_width, half_height)
            try:
                layers = list(copper_layers(
                    adapter, board, pad.GetLayerSet()))
            except Exception:
                layers = []
                for layer in enabled_copper_layers(adapter, board):
                    try:
                        if pad.IsOnLayer(layer):
                            layers.append(layer)
                    except Exception:
                        continue
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
            custom_shape = False
            try:
                custom_shape = int(pad.GetShape()) == int(
                    adapter.pcbnew.PAD_SHAPE_CUSTOM)
            except (AttributeError, TypeError, ValueError):
                pass
            if custom_shape:
                exact_regions = _custom_pad_regions(
                    adapter, pad, layers, int(pad.GetNetCode()),
                    local_clearance)
                if exact_regions:
                    pad_regions.extend(exact_regions)
                    continue
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
