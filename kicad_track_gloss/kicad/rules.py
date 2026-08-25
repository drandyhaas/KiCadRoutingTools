"""Read board bounds and routing rules exposed through KiCad SWIG."""

from __future__ import annotations

from pathlib import Path
import re

from ..engine.model import BoardOutline, PolygonKeepout


def copper_layers(adapter, board, layer_set):
    """Return enabled copper layer IDs without relying on display names."""
    try:
        enabled = {int(layer) for layer in board.GetEnabledLayers().Seq()}
    except Exception:
        enabled = None
    result = []
    for layer in layer_set.Seq():
        layer_id = int(layer)
        try:
            is_copper = bool(adapter.pcbnew.IsCopperLayer(layer))
        except Exception:
            is_copper = 0 <= layer_id <= 62 and layer_id % 2 == 0
        if is_copper and (enabled is None or layer_id in enabled):
            result.append(layer_id)
    return tuple(sorted(set(result)))


def _chain_points(adapter, chain):
    return tuple(adapter.point_mm(chain.CPoint(index))
                 for index in range(chain.PointCount()))


def exact_board_outline(adapter, board):
    """Extract KiCad's chained Edge.Cuts polygon, including internal holes."""
    try:
        polygons = adapter.pcbnew.SHAPE_POLY_SET()
        if not board.GetBoardPolygonOutlines(
                polygons, True, None, False, True):
            return None
        outlines, holes = [], []
        for index in range(polygons.OutlineCount()):
            points = _chain_points(adapter, polygons.Outline(index))
            if len(points) >= 3:
                outlines.append(points)
            for hole_index in range(polygons.HoleCount(index)):
                points = _chain_points(
                    adapter, polygons.Hole(index, hole_index))
                if len(points) >= 3:
                    holes.append(points)
        if outlines:
            return BoardOutline(tuple(outlines), tuple(holes))
    except Exception:
        pass
    return None


def via_track_hole_clearance(adapter, board):
    """Return a custom via-to-track hole-clearance floor when available.

    KiCad 10's Python bindings expose the board clearance cache but not the
    pairwise hole-clearance evaluator used for hypothetical candidate tracks.
    Read only the narrowly applicable rule from the matching project DRU; all
    other constraints continue to come from KiCad's native objects.
    """
    try:
        rules_path = Path(str(board.GetFileName())).with_suffix(".kicad_dru")
        text = rules_path.read_text(encoding="utf-8")
    except (AttributeError, OSError, UnicodeError):
        return 0.0
    text = re.sub(r"(?m)^\s*#.*$", "", text)
    floor = 0.0
    for match in re.finditer(r"\(rule\b", text, flags=re.IGNORECASE):
        depth = 0
        end = None
        for index in range(match.start(), len(text)):
            if text[index] == "(":
                depth += 1
            elif text[index] == ")":
                depth -= 1
                if depth == 0:
                    end = index + 1
                    break
        if end is None:
            continue
        block = text[match.start():end]
        condition = " ".join(re.findall(
            r"\(condition\s+\"([^\"]*)\"\s*\)", block,
            flags=re.IGNORECASE)).lower()
        if "via" not in condition or "track" not in condition or "!= 'via'" in condition:
            continue
        values = re.findall(
            r"\(constraint\s+hole_clearance\s+\(min\s+"
            r"([0-9]+(?:\.[0-9]+)?)\s*mm\s*\)", block,
            flags=re.IGNORECASE)
        floor = max([floor] + [float(value) for value in values])
    return floor


def board_bounds(adapter, board):
    try:
        box = board.GetBoardEdgesBoundingBox()
        x = adapter.to_mm(box.GetX())
        y = adapter.to_mm(box.GetY())
        return (x, y, x + adapter.to_mm(box.GetWidth()),
                y + adapter.to_mm(box.GetHeight()))
    except Exception:
        return None


def native_rules(adapter, board, segments):
    """Resolve board floors and netclass fallback clearances through KiCad.

    Per-track rule-engine results, including custom ``.kicad_dru`` rules, are
    captured by :meth:`BoardAdapter.segment_from_item` instead.
    """
    minimum = 0.0
    edge = 0.0
    by_net = {}
    try:
        settings = board.GetDesignSettings()
        minimum = adapter.to_mm(settings.m_MinClearance)
        edge = adapter.to_mm(settings.m_CopperEdgeClearance)
        net_settings = settings.m_NetSettings
        names = {s.net_id: s.net_name for s in segments if s.net_id > 0 and s.net_name}
        for net_id, name in names.items():
            try:
                netclass = net_settings.GetEffectiveNetClass(name)
                by_net[net_id] = max(minimum, adapter.to_mm(netclass.GetClearance()))
            except Exception:
                by_net[net_id] = minimum
    except Exception:
        pass
    return max(minimum, 0.0), max(edge, 0.0), by_net


def track_keepouts(adapter, board):
    result = []
    try:
        zones = [board.GetArea(i) for i in range(board.GetAreaCount())]
    except Exception:
        return result
    for zone in zones:
        try:
            if not zone.GetIsRuleArea() or not zone.GetDoNotAllowTracks():
                continue
            layers = tuple(int(layer) for layer in zone.GetLayerSet().Seq())
            outline = zone.Outline()
            for index in range(outline.OutlineCount()):
                chain = outline.Outline(index)
                points = tuple(adapter.point_mm(chain.CPoint(i))
                               for i in range(chain.PointCount()))
                if len(points) >= 3:
                    result.append(PolygonKeepout(points, layers))
        except Exception:
            continue
    return result


def mask_graphic_keepouts(adapter, board):
    """Protect explicit solder-mask graphics from newly routed copper.

    Automatic pad apertures are intentionally excluded: pads already have
    exact copper geometry and are legitimate track terminals.  Explicit mask
    drawings can expose unrelated copper, so moving a track beneath one may
    create a solder-mask bridge even when copper clearance remains valid.
    """
    result = []
    layer_map = {}
    for mask_name, copper_name in (("F.Mask", "F.Cu"), ("B.Mask", "B.Cu")):
        try:
            layer_map[int(board.GetLayerID(mask_name))] = int(
                board.GetLayerID(copper_name))
        except Exception:
            continue
    if not layer_map:
        return result

    drawings = []
    try:
        drawings.extend(board.GetDrawings())
    except Exception:
        pass
    try:
        for footprint in board.GetFootprints():
            drawings.extend(footprint.GraphicalItems())
    except Exception:
        pass
    for item in drawings:
        try:
            if str(item.GetClass()) not in ("PCB_SHAPE", "FP_SHAPE"):
                continue
            copper_layer = layer_map.get(int(item.GetLayer()))
            if copper_layer is None:
                continue
            box = item.GetBoundingBox()
            x0 = adapter.to_mm(box.GetX())
            y0 = adapter.to_mm(box.GetY())
            x1 = x0 + adapter.to_mm(box.GetWidth())
            y1 = y0 + adapter.to_mm(box.GetHeight())
            if x1 <= x0 or y1 <= y0:
                continue
            result.append(PolygonKeepout(
                ((x0, y0), (x1, y0), (x1, y1), (x0, y1)),
                (copper_layer,), "solder_mask"))
        except Exception:
            continue
    return result
