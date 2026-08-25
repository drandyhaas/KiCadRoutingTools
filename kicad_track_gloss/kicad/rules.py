"""Read board bounds and routing rules exposed through KiCad SWIG."""

from __future__ import annotations

from ..engine.model import PolygonKeepout


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
