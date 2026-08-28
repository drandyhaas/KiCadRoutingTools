"""Convert the current KiCad board and selection into an engine snapshot."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..engine.model import BoardModel, CircleObstacle, PadRegion, segment_key
from .authority import native_authority, protection_warnings
from .rules import (copper_layers, exact_board_outline, native_rules,
                    track_keepouts)
from .selection import expand_eligible_scopes
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
    native_protected_count: int = 0
    connection_scopes: tuple = ()


def _via_copper_layers(adapter, board, item):
    return copper_layers(adapter, board, item.GetLayerSet())


def _pad_shape(adapter, pad):
    """Map KiCad 10's native analytic pad shapes to engine primitives."""
    return {
        int(adapter.pcbnew.PAD_SHAPE_CIRCLE): "circle",
        int(adapter.pcbnew.PAD_SHAPE_RECT): "rect",
        int(adapter.pcbnew.PAD_SHAPE_OVAL): "oval",
        int(adapter.pcbnew.PAD_SHAPE_ROUNDRECT): "roundrect",
    }.get(int(pad.GetShape()))


def _line_chain_points(adapter, chain):
    return tuple(adapter.point_mm(chain.CPoint(index))
                 for index in range(chain.PointCount()))


def _pad_regions(adapter, pad, layers, net_id, clearance):
    """Return exact effective copper polygons, one layer-specific region each."""
    regions = []
    for layer in layers:
        polyset = pad.GetEffectivePolygon(layer)
        polygons = []
        for outline_index in range(polyset.OutlineCount()):
            outer = _line_chain_points(adapter, polyset.Outline(outline_index))
            holes = tuple(
                _line_chain_points(adapter, polyset.Hole(
                    outline_index, hole_index))
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
    return regions


def read_snapshot(adapter, board, require_selection=True):
    segments, obstacles, pad_regions, warnings = [], [], [], []
    straight_by_key = {}
    seed_keys = set()
    board.InitializeClearanceCache()
    selected_authorities = {}
    for item in board.GetTracks():
        if is_via(adapter.pcbnew, item):
            x, y = adapter.point_mm(item.GetPosition())
            diameter = adapter.to_mm(item.GetWidth(item.TopLayer()))
            layers = _via_copper_layers(adapter, board, item)
            obstacles.append(CircleObstacle(x, y, diameter / 2.0,
                                            int(item.GetNetCode()), layers, "via"))
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
        else:
            authority = native_authority(adapter.pcbnew, board, item)
            if authority is not None:
                selected_authorities[key] = authority
                continue
            seed_keys.add(key)

    warnings.extend(protection_warnings(selected_authorities))

    eligible, expanded, protected_expanded, connection_scopes = expand_eligible_scopes(
        adapter, board, straight_by_key, seed_keys, warnings)
    expanded_count = max(0, len(expanded) - len(seed_keys))

    footprints = board.GetFootprints()
    minimum, edge, net_clearances = native_rules(adapter, board, segments)
    for footprint in footprints:
        for pad in footprint.Pads():
            x, y = adapter.point_mm(pad.GetPosition())
            size = pad.GetSize()
            layers = list(copper_layers(adapter, board, pad.GetLayerSet()))
            # Paste/mask-only apertures are not copper obstacles. An empty
            # layer tuple means "all layers" inside the API-neutral model, so
            # retaining such a pad would incorrectly block every copper layer.
            if not layers:
                continue
            local_clearance_iu = pad.GetLocalClearance()
            local_clearance = (0.0 if local_clearance_iu is None else
                               adapter.to_mm(local_clearance_iu))
            shape = _pad_shape(adapter, pad)
            if shape is not None:
                pad_regions.append(PadRegion(
                    x, y, adapter.to_mm(size.x), adapter.to_mm(size.y),
                    float(pad.GetOrientationDegrees()), shape,
                    adapter.to_mm(pad.GetRoundRectCornerRadius()),
                    int(pad.GetNetCode()), tuple(layers), local_clearance))
                continue
            exact_regions = _pad_regions(
                adapter, pad, layers, int(pad.GetNetCode()), local_clearance)
            if not exact_regions:
                raise RuntimeError(
                    "KiCad 10 returned no effective copper polygon for pad {}"
                    .format(pad.GetNumber()))
            pad_regions.extend(exact_regions)

    if require_selection and not seed_keys:
        if warnings:
            raise ValueError("No eligible straight track is selected. " +
                             " ".join(sorted(set(warnings))))
        raise ValueError("Select at least two connected straight track segments first.")
    keepouts = track_keepouts(adapter, board)
    model = BoardModel(segments, obstacles, keepouts,
                       net_clearances, minimum, edge, None,
                       pad_regions, exact_board_outline(adapter, board),
                       1.0 / adapter._iu_per_mm())
    return SelectionSnapshot(model, eligible, sorted(set(warnings)), minimum, edge,
                             len(seed_keys), expanded_count,
                             len(selected_authorities) + len(protected_expanded),
                             connection_scopes)
