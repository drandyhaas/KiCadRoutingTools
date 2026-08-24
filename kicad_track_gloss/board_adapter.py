"""SWIG pcbnew adapter isolated from the API-neutral gloss engine."""

from __future__ import annotations

from dataclasses import dataclass, field
import re

from .model import BoardModel, CircleObstacle, PolygonKeepout, Segment, geometry_key, segment_key


@dataclass
class SelectionSnapshot:
    model: BoardModel
    eligible_keys: set
    warnings: list = field(default_factory=list)
    minimum_clearance: float = 0.1
    copper_edge_clearance: float = 0.0


def _uuid(item):
    try:
        return item.m_Uuid.AsString()
    except Exception:
        try:
            return item.GetUuid().AsString()
        except Exception:
            return ""


def _net_name(item):
    try:
        return str(item.GetNetname() or "")
    except Exception:
        return ""


def _is_probable_diff_pair(name):
    return bool(re.search(r"(?:[_+.-](?:P|N)|[+-])$", name, re.IGNORECASE))


def _meander_keys(segments):
    """Detect the characteristic A/B/-A direction reversal of tuned meanders."""
    by_group = {}
    for seg in segments:
        by_group.setdefault((seg.net_id, seg.layer, round(seg.width, 6)), []).append(seg)
    protected = set()
    for group in by_group.values():
        adjacency = {}
        for seg in group:
            for point in ((round(seg.start_x, 6), round(seg.start_y, 6)),
                          (round(seg.end_x, 6), round(seg.end_y, 6))):
                adjacency.setdefault(point, []).append(seg)
        starts = [point for point, touching in adjacency.items() if len(touching) == 1]
        if not starts:
            continue
        used, vectors = set(), []
        point = starts[0]
        while True:
            options = [seg for seg in adjacency.get(point, ()) if segment_key(seg) not in used]
            if not options:
                break
            seg = options[0]
            used.add(segment_key(seg))
            start = (round(seg.start_x, 6), round(seg.start_y, 6))
            if start == point:
                nxt = (round(seg.end_x, 6), round(seg.end_y, 6))
                vectors.append((seg.end_x - seg.start_x, seg.end_y - seg.start_y))
            else:
                nxt = start
                vectors.append((seg.start_x - seg.end_x, seg.start_y - seg.end_y))
            point = nxt
        if any(vectors[i][0] * vectors[i + 2][0] + vectors[i][1] * vectors[i + 2][1] < -1e-9
               for i in range(len(vectors) - 2)):
            protected.update(segment_key(seg) for seg in group)
    return protected


class BoardAdapter:
    def __init__(self, pcbnew_module):
        self.pcbnew = pcbnew_module

    def to_mm(self, value):
        try:
            return float(self.pcbnew.ToMM(value))
        except Exception:
            return float(value) / 1_000_000.0

    def point_mm(self, point):
        return self.to_mm(point.x), self.to_mm(point.y)

    def vector(self, point):
        x = int(round(self.pcbnew.FromMM(point[0])))
        y = int(round(self.pcbnew.FromMM(point[1])))
        return self.pcbnew.VECTOR2I(x, y)

    def snapshot(self, board, require_selection=True):
        segments, obstacles, warnings = [], [], []
        eligible = set()
        selected_straight = 0
        for item in board.GetTracks():
            kind = str(item.GetClass())
            if kind == "PCB_VIA":
                x, y = self.point_mm(item.GetPosition())
                try:
                    diameter = self.to_mm(item.GetFrontWidth())
                except Exception:
                    diameter = self.to_mm(item.GetWidth())
                layers = tuple(range(int(item.TopLayer()), int(item.BottomLayer()) + 1))
                obstacles.append(CircleObstacle(x, y, diameter / 2.0,
                                                int(item.GetNetCode()), layers, "via"))
                if item.IsSelected():
                    warnings.append("Selected vias are protected and will not be modified.")
                continue
            if kind not in ("PCB_TRACK", "PCB_ARC"):
                continue
            start, end = self.point_mm(item.GetStart()), self.point_mm(item.GetEnd())
            seg = Segment(start[0], start[1], end[0], end[1],
                          self.to_mm(item.GetWidth()), int(item.GetLayer()),
                          int(item.GetNetCode()), _uuid(item), bool(item.IsLocked()),
                          kind == "PCB_ARC", _net_name(item))
            segments.append(seg)
            if not item.IsSelected():
                continue
            if kind == "PCB_ARC":
                warnings.append("Selected arcs are protected in this version.")
            elif seg.locked:
                warnings.append("Selected locked tracks are protected.")
            elif _is_probable_diff_pair(seg.net_name):
                warnings.append("Probable differential-pair tracks are protected: " + seg.net_name)
            else:
                eligible.add(segment_key(seg))
                selected_straight += 1

        meanders = _meander_keys([s for s in segments if segment_key(s) in eligible])
        if meanders:
            eligible.difference_update(meanders)
            warnings.append("Probable meander/length-tuning tracks are protected.")
            selected_straight = len(eligible)

        try:
            footprints = board.GetFootprints()
        except Exception:
            footprints = ()
        minimum_clearance, edge_clearance, net_clearances = self._native_rules(board, segments)
        copper_layers = tuple(range(0, 32))
        for footprint in footprints:
            for pad in footprint.Pads():
                x, y = self.point_mm(pad.GetPosition())
                size = pad.GetSize()
                radius = max(self.to_mm(size.x), self.to_mm(size.y)) / 2.0
                layers = []
                try:
                    layers = [int(layer) for layer in pad.GetLayerSet().Seq()
                              if str(board.GetLayerName(layer)).endswith(".Cu")]
                except Exception:
                    layers = list(copper_layers)
                try:
                    local_clearance = self.to_mm(pad.GetLocalClearance())
                except Exception:
                    local_clearance = 0.0
                obstacles.append(CircleObstacle(x, y, radius, int(pad.GetNetCode()),
                                                tuple(layers), "pad", local_clearance))

        keepouts = self._keepouts(board)
        if require_selection and selected_straight == 0:
            if warnings:
                raise ValueError("No eligible straight track is selected. " + " ".join(sorted(set(warnings))))
            raise ValueError("Select at least two connected straight track segments first.")
        model = BoardModel(segments, obstacles, keepouts, net_clearances,
                           minimum_clearance, edge_clearance,
                           self._board_bounds(board))
        return SelectionSnapshot(model, eligible, sorted(set(warnings)),
                                 minimum_clearance, edge_clearance)

    def _board_bounds(self, board):
        try:
            box = board.GetBoardEdgesBoundingBox()
            x = self.to_mm(box.GetX())
            y = self.to_mm(box.GetY())
            return (x, y, x + self.to_mm(box.GetWidth()),
                    y + self.to_mm(box.GetHeight()))
        except Exception:
            return None

    def _native_rules(self, board, segments):
        """Resolve board floors and effective netclass clearances through KiCad."""
        minimum = 0.0
        edge = 0.0
        by_net = {}
        try:
            settings = board.GetDesignSettings()
            minimum = self.to_mm(settings.m_MinClearance)
            edge = self.to_mm(settings.m_CopperEdgeClearance)
            net_settings = settings.m_NetSettings
            names = {s.net_id: s.net_name for s in segments if s.net_id > 0 and s.net_name}
            for net_id, name in names.items():
                try:
                    netclass = net_settings.GetEffectiveNetClass(name)
                    by_net[net_id] = max(minimum, self.to_mm(netclass.GetClearance()))
                except Exception:
                    by_net[net_id] = minimum
        except Exception:
            pass
        return max(minimum, 0.0), max(edge, 0.0), by_net

    def _keepouts(self, board):
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
                for idx in range(outline.OutlineCount()):
                    chain = outline.Outline(idx)
                    pts = tuple(self.point_mm(chain.CPoint(i)) for i in range(chain.PointCount()))
                    if len(pts) >= 3:
                        result.append(PolygonKeepout(pts, layers))
            except Exception:
                # Native DRC validation remains authoritative if an older SWIG
                # build cannot expose the rule-area polygon.
                continue
        return result

    def _track_map(self, board):
        mapping = {}
        for item in board.GetTracks():
            if str(item.GetClass()) != "PCB_TRACK":
                continue
            start, end = self.point_mm(item.GetStart()), self.point_mm(item.GetEnd())
            seg = Segment(start[0], start[1], end[0], end[1],
                          self.to_mm(item.GetWidth()), int(item.GetLayer()),
                          int(item.GetNetCode()), _uuid(item), bool(item.IsLocked()),
                          False, _net_name(item))
            mapping[segment_key(seg)] = item
        return mapping

    def apply(self, board, result, rollback_on_error=True):
        """Apply a prevalidated plan; restore removed copper on any exception."""
        mapping = self._track_map(board)
        missing = [key for key in result.remove_keys if key not in mapping]
        if missing:
            raise RuntimeError("Selected tracks changed before apply; aborting safely.")
        removed_specs = []
        created = []
        try:
            for key in result.remove_keys:
                item = mapping[key]
                removed_specs.append((self.point_mm(item.GetStart()), self.point_mm(item.GetEnd()),
                                      self.to_mm(item.GetWidth()), int(item.GetLayer()),
                                      int(item.GetNetCode())))
                board.RemoveNative(item)
            for spec in result.additions:
                created.append(self._add_track(board, spec.start, spec.end, spec.width,
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
                    self._add_track(board, start, end, width, layer, net_id)
            raise

    def _add_track(self, board, start, end, width, layer, net_id):
        track = self.pcbnew.PCB_TRACK(board)
        track.SetStart(self.vector(start))
        track.SetEnd(self.vector(end))
        track.SetWidth(int(round(self.pcbnew.FromMM(width))))
        track.SetLayer(layer)
        track.SetNetCode(net_id)
        board.Add(track)
        return track
