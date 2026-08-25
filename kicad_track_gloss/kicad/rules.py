"""Read board bounds and routing rules exposed through KiCad SWIG."""

from __future__ import annotations

from pathlib import Path
import re

from ..engine.model import PolygonKeepout


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
