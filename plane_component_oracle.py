"""
Plane connected-component oracle (T6 "mutual-floating-strap" false success).

The plane-tap machinery (plane_pad_tap, route_disconnected_planes) connects a
pad to a power/GND plane by reusing nearby same-net copper -- a via, an
already-tapped pad, a segment. That "same-net copper" can itself be a FLOATING
island or strap of the plane (copper another tap just placed, a disconnected
zone fragment, a stale pre-existing via) that never reaches the main plane.
Two pads can strap to each other's island and both report success while both
stay disconnected; the authoritative zone/fill-aware check_net_connectivity
later grades the net incomplete.

PlaneComponentOracle computes the plane net's connected components ONCE from
the SAME union-find graph the authoritative grader uses
(check_connected.check_net_connectivity with return_graph=True, including its
outline-permissive zone credit), identifies the MAIN plane component -- the
zone-credited component holding the most pads (falling back to the most-pad
component when no zone is credited) -- and answers "is this copper on the main
plane?" for tap-target filtering. It is updated incrementally as verified taps
commit (note_tap_committed), so copper a verified tap just placed is a valid
target for the next pad (transitive credit) without rebuilding the graph.

Because "main" here is exactly "connected to the main component" there, a tap
the oracle accepts is one the grader will credit: honest failure beats false
success.

Scope: route_disconnected_planes' pad-repair pass and final sweep thread an
oracle through plane_pad_tap.try_tap_pad (plane_oracle=...). route_planes'
create_plane does NOT need one: it places taps BEFORE pouring its zone over
the board, so every through via it drops lands under the future fill by
construction. Nets without zones leave the oracle inert (all queries True) --
there is no "plane" to be disconnected from.
"""
from __future__ import annotations

import math
from collections import defaultdict
from typing import Dict, Iterable, List, Optional, Set, Tuple

from geometry_utils import UnionFind

_KEY_DECIMALS = 4
_TOUCH_TOL = 0.02  # mm - same connection tolerance check_net_connectivity uses


def _pt_key(x: float, y: float) -> Tuple[float, float]:
    return (round(x, _KEY_DECIMALS), round(y, _KEY_DECIMALS))


def _pad_key(pad) -> Tuple:
    return (round(pad.global_x, _KEY_DECIMALS), round(pad.global_y, _KEY_DECIMALS),
            getattr(pad, 'component_ref', ''), str(getattr(pad, 'pad_number', '')))


def _seg_key(seg) -> Tuple:
    a = (round(seg.start_x, _KEY_DECIMALS), round(seg.start_y, _KEY_DECIMALS))
    b = (round(seg.end_x, _KEY_DECIMALS), round(seg.end_y, _KEY_DECIMALS))
    return (a, b, seg.layer) if a <= b else (b, a, seg.layer)


class PlaneComponentOracle:
    """Connected components of ONE plane net's copper, main component known.

    Deterministic: built from pcb_data's list order, main-root ties broken by
    pad count then insertion order; no randomness, no set-iteration decisions.
    """

    def __init__(self, pcb_data, net_id: int):
        self.pcb_data = pcb_data
        self.net_id = net_id
        self.rebuild()

    # ------------------------------------------------------------------ build
    def rebuild(self) -> None:
        """(Re)compute components from the current pcb_data copper."""
        from check_connected import check_net_connectivity
        net_id = self.net_id
        segments = [s for s in self.pcb_data.segments if s.net_id == net_id]
        vias = [v for v in self.pcb_data.vias if v.net_id == net_id]
        pads = list(self.pcb_data.pads_by_net.get(net_id, []))
        zones = [z for z in (getattr(self.pcb_data, 'zones', None) or [])
                 if getattr(z, 'net_id', None) == net_id]

        # No zone = no plane to be disconnected from: stay inert (permissive),
        # preserving pre-oracle behavior for pure trace/via repair nets.
        self.inert = not zones
        self._main_via_keys: Set[Tuple[float, float]] = set()
        self._main_pad_keys: Set[Tuple] = set()
        self._main_seg_keys: Set[Tuple] = set()
        self._main_pads: List = []
        self._main_segments: List = []
        self._main_zones: List = []
        self._all_zones: List = zones
        self.n_floating_items = 0
        if self.inert:
            return

        # pcb_data enables the validator-parity fill model: components
        # split per FILL component, so a pinched island's items grade
        # floating and the repair machinery actually repairs them.
        res = check_net_connectivity(net_id, segments, vias, pads, zones,
                                     return_graph=True,
                                     pcb_data=self.pcb_data)
        graph = res.get('graph') or {}
        uf = UnionFind()
        for a, b in graph.get('edges', ()):
            uf.union(a, b)

        pad_repr: Dict[int, int] = graph.get('pad_index_repr', {})
        via_repr: Dict[int, int] = graph.get('via_index_repr', {})
        zone_repr: Dict[int, int] = graph.get('zone_index_repr', {})

        pad_count: Dict[int, int] = defaultdict(int)
        for pad_idx in sorted(pad_repr):
            pad_count[uf.find(pad_repr[pad_idx])] += 1

        # Main root: among zone-credited components, the one with most pads
        # (a floating zone fragment loses to the real plane); if no zone has
        # any credited point, fall back to the most-pad component. Ties break
        # by first occurrence in zone/pad order -- deterministic.
        zone_roots: List[int] = []
        for zone_idx in sorted(zone_repr):
            r = uf.find(zone_repr[zone_idx])
            if r not in zone_roots:
                zone_roots.append(r)
        candidates = zone_roots
        if not candidates:
            seen: List[int] = []
            for pad_idx in sorted(pad_repr):
                r = uf.find(pad_repr[pad_idx])
                if r not in seen:
                    seen.append(r)
            candidates = seen
        if not candidates:
            # Net has zones but no copper points at all: nothing is provably
            # main; taps must place fresh vias (zone outlines stay unfiltered
            # via main_zone_polygons() returning empty -> caller falls back).
            return
        main_root = max(candidates, key=lambda r: pad_count.get(r, 0))

        for via_idx in sorted(via_repr):
            if uf.find(via_repr[via_idx]) == main_root:
                v = vias[via_idx]
                self._main_via_keys.add(_pt_key(v.x, v.y))
            else:
                self.n_floating_items += 1
        for pad_idx in sorted(pad_repr):
            if uf.find(pad_repr[pad_idx]) == main_root:
                p = pads[pad_idx]
                self._main_pad_keys.add(_pad_key(p))
                self._main_pads.append(p)
        for i, s in enumerate(segments):
            # Segment i's endpoints are point ids 2i/2i+1 (see the graph note
            # in check_net_connectivity); both are unioned, either works.
            if uf.find(2 * i) == main_root:
                self._main_seg_keys.add(_seg_key(s))
                self._main_segments.append(s)
            else:
                self.n_floating_items += 1
        for zone_idx in sorted(zone_repr):
            if uf.find(zone_repr[zone_idx]) == main_root:
                self._main_zones.append(zones[zone_idx])

    # ---------------------------------------------------------------- queries
    def via_on_plane(self, x: float, y: float) -> bool:
        """True iff a same-net via at (x, y) belongs to the main component."""
        if self.inert:
            return True
        return _pt_key(x, y) in self._main_via_keys

    def pad_on_plane(self, pad) -> bool:
        """True iff this same-net pad belongs to the main component."""
        if self.inert:
            return True
        return _pad_key(pad) in self._main_pad_keys

    def segment_on_plane(self, seg) -> bool:
        """True iff this same-net segment belongs to the main component."""
        if self.inert:
            return True
        return _seg_key(seg) in self._main_seg_keys

    def zone_on_plane(self, zone) -> bool:
        """True iff this zone's credited copper is the main component."""
        if self.inert:
            return True
        return any(z is zone for z in self._main_zones)

    def main_zone_polygons(self) -> List:
        """Outline polygons of the main component's zones (may be empty when
        no zone has credited copper yet -- callers should then fall back to
        the unfiltered zone list rather than blocking via placement)."""
        return [z.polygon for z in self._main_zones
                if getattr(z, 'polygon', None) and len(z.polygon) >= 3]

    def point_reaches_plane(self, x: float, y: float) -> bool:
        """True iff copper at (x, y) ties to the main component: it coincides
        with main copper (via / pad / segment) or lands inside a main zone
        outline (the grader's outline credit; a through via there is fill-
        connected exactly as check_net_connectivity will grade it)."""
        if self.inert:
            return True
        if _pt_key(x, y) in self._main_via_keys:
            return True
        from check_connected import point_in_polygon
        # When no zone has credited copper yet, nothing is provably main --
        # keep pre-oracle permissiveness for in-outline copper rather than
        # bricking every new-via tap on that degenerate board.
        zone_polys = self.main_zone_polygons() or [
            z.polygon for z in self._all_zones
            if getattr(z, 'polygon', None) and len(z.polygon) >= 3]
        for poly in zone_polys:
            if point_in_polygon(x, y, poly):
                return True
        from routing_utils import point_in_pad_rect
        for p in self._main_pads:
            if point_in_pad_rect(x, y, p, _TOUCH_TOL):
                return True
        from check_connected import point_on_segment
        for s in self._main_segments:
            if point_on_segment(x, y, s.start_x, s.start_y, s.end_x, s.end_y,
                                s.width / 2 + _TOUCH_TOL):
                return True
        return False

    def verify_tap(self, result) -> bool:
        """Oracle-strict terminal rung: does a successful TapResult provably
        tie its pad to the main plane component? A new via must reach the
        plane at its own position; a reuse/trace tap must have landed on
        main copper (its recorded target position)."""
        if self.inert:
            return True
        if getattr(result, 'via', None) is not None:
            return self.point_reaches_plane(result.via['x'], result.via['y'])
        pos = getattr(result, 'reused_via_pos', None)
        if pos is None:
            return False  # trace success with no recorded target: unprovable
        return self.point_reaches_plane(pos[0], pos[1])

    # ---------------------------------------------------------- incremental
    def note_tap_committed(self, pad=None, new_vias: Iterable = (),
                           new_segments: Iterable = ()) -> None:
        """Register a VERIFIED tap's pad and copper as main-connected, so later
        taps may target them (transitive credit) without a graph rebuild.
        new_vias accepts Via objects or tap dicts ({'x','y',...}); new_segments
        accepts Segment objects or tap dicts ({'start','end','width','layer'})."""
        if self.inert:
            return
        if pad is not None and _pad_key(pad) not in self._main_pad_keys:
            self._main_pad_keys.add(_pad_key(pad))
            self._main_pads.append(pad)
        for v in new_vias or ():
            if isinstance(v, dict):
                self._main_via_keys.add(_pt_key(v['x'], v['y']))
            else:
                self._main_via_keys.add(_pt_key(v.x, v.y))
        for s in new_segments or ():
            if isinstance(s, dict):
                from kicad_parser import Segment
                s = Segment(start_x=s['start'][0], start_y=s['start'][1],
                            end_x=s['end'][0], end_y=s['end'][1],
                            width=s['width'], layer=s['layer'],
                            net_id=self.net_id)
            k = _seg_key(s)
            if k not in self._main_seg_keys:
                self._main_seg_keys.add(k)
                self._main_segments.append(s)
