"""Read board bounds and routing rules exposed through KiCad SWIG."""

from __future__ import annotations

from ..engine.model import BoardOutline, PolygonKeepout


def copper_layers(adapter, board, layer_set):
    """Return enabled copper layer IDs without relying on display names."""
    enabled = {int(layer) for layer in board.GetEnabledLayers().Seq()}
    result = []
    for layer in layer_set.Seq():
        layer_id = int(layer)
        if adapter.pcbnew.IsCopperLayer(layer) and layer_id in enabled:
            result.append(layer_id)
    return tuple(sorted(set(result)))


def _chain_points(adapter, chain):
    return tuple(adapter.point_mm(chain.CPoint(index))
                 for index in range(chain.PointCount()))


def exact_board_outline(adapter, board):
    """Extract KiCad's chained Edge.Cuts polygon, including internal holes."""
    polygons = adapter.pcbnew.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(polygons, True, None, False, True):
        return None
    outlines, holes = [], []
    for index in range(polygons.OutlineCount()):
        points = _chain_points(adapter, polygons.Outline(index))
        if len(points) >= 3:
            outlines.append(points)
        for hole_index in range(polygons.HoleCount(index)):
            points = _chain_points(adapter, polygons.Hole(index, hole_index))
            if len(points) >= 3:
                holes.append(points)
    if outlines:
        return BoardOutline(tuple(outlines), tuple(holes))
    return None


def native_rules(adapter, board, segments):
    """Resolve board floors and netclass fallback clearances through KiCad.

    Per-track rule-engine results, including custom ``.kicad_dru`` rules, are
    captured by :meth:`BoardAdapter.segment_from_item` instead.
    """
    minimum = 0.0
    edge = 0.0
    by_net = {}
    settings = board.GetDesignSettings()
    minimum = adapter.to_mm(settings.m_MinClearance)
    edge = adapter.to_mm(settings.m_CopperEdgeClearance)
    net_settings = settings.m_NetSettings
    names = {s.net_id: s.net_name for s in segments if s.net_id > 0 and s.net_name}
    for net_id, name in names.items():
        netclass = net_settings.GetEffectiveNetClass(name)
        by_net[net_id] = max(minimum, adapter.to_mm(netclass.GetClearance()))
    return max(minimum, 0.0), max(edge, 0.0), by_net


def track_keepouts(adapter, board):
    result = []
    zones = [board.GetArea(i) for i in range(board.GetAreaCount())]
    for zone in zones:
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
    return result
