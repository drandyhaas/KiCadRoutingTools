"""Mounting holes and the copper keep-out a screw needs around them (#1442).

KiCad checks a mounting hole against the ordinary hole and pad clearance and
nothing more. The screw head, washer, nut or standoff that sits on the hole is
not in the board file, so a route can lay a track right up to the hole's
copper and every DRC passes while the head lands on it.

This leaf module is the one definition of both questions:

  * ``mounting_holes(pcb)`` -- which footprints are mounting holes;
  * ``hole_copper(pcb)`` -- the other-net tracks and vias inside each hole's
    screw-head ring (``check_mounting_holes.py`` reports it);
  * ``keepout_polygon(hole)`` -- that ring as a polygon, which
    ``add_rule_area.py --mounting-holes`` writes as a copper keep-out the
    router and the plane steps already honour like any rule area.

The ring is derived from the drill: a screw's head reaches about 0.4 x the
drill past the hole's edge for every metric size (ISO 7380 button heads, M2 to
M5: 0.8, 1.3, 1.65 and 2.1 mm past drills of 2.2, 3.2, 4.3 and 5.3 mm). A
washer or standoff reaches further; ``ring`` overrides it.
"""
from __future__ import annotations

import math
import re
from typing import Dict, List, NamedTuple, Optional, Tuple

#: A drill at least this wide is a mounting hole, not a via or a pin.
MOUNT_HOLE_MIN_DRILL = 2.0
#: Footprints with more pads than this are a connector or a module, whatever
#: their drills (KiCad's own ``MountingHole_*_Pad_Via`` is the hole plus 8
#: stitching vias).
MOUNT_HOLE_MAX_PADS = 9
#: How far a screw head reaches past the hole's edge, as a fraction of the drill.
RING_PER_DRILL = 0.4
#: A stitching via-pad beside the hole drills at most this fraction of it.
STITCH_MAX_DRILL_FRACTION = 0.5
#: Two drills (or a drill and drill-less copper) this close are one position.
SAME_CENTRE_MM = 0.01

_NO_SIGNAL_NET = re.compile(r'(^$|^unconnected-|.*GND)', re.I)
#: KiCad's name for a net of one unconnected pin: no net, for a hole.
_UNCONNECTED_NET = re.compile(r'^unconnected-', re.I)


class MountingHole(NamedTuple):
    ref: str            # footprint key
    x: float            # hole centre, board mm
    y: float
    drill: float        # the screw hole's drill (the widest of its pads)
    nets: frozenset     # net ids its pads carry (a GND ring), usually empty;
                        # an ``unconnected-*`` pin net is not one

    def ring(self, override: Optional[float] = None) -> float:
        """How far past the hole's edge the screw head reaches, mm."""
        return override if override is not None else RING_PER_DRILL * self.drill


def _hole_centre(p) -> Tuple[float, float]:
    """Where a pad's drill is, which is not always where its copper is (an
    offset drill)."""
    return (p.hole_x if p.hole_x is not None else p.global_x,
            p.hole_y if p.hole_y is not None else p.global_y)


def _round_drill(p) -> bool:
    """A round hole, not a milled slot (``(drill oval w h)``)."""
    w = getattr(p, 'drill_w', 0.0) or 0.0
    h = getattr(p, 'drill_h', 0.0) or 0.0
    return w <= 0 or h <= 0 or abs(w - h) < 1e-6


def _copper_radius(p) -> float:
    """How far a round pad's copper reaches from its drill: the pad's extent,
    never less than the drill itself (an NPTH's size is its mask opening)."""
    return max(p.size_x or 0.0, p.size_y or 0.0, p.drill or 0.0) / 2.0


def _main_hole(pads):
    """The footprint's screw hole, or None: its widest drilled pad, a ROUND
    drill of MOUNT_HOLE_MIN_DRILL or more, and every other pad one of

      * copper at the same position -- the same hole drawn again (front and
        back pads, KiCad's ``_Pad_TopBottom``: drill-less ``connect``/SMD
        pads on F.Cu and B.Cu over a drilled one), drilled or drill-less;
      * a small PLATED via-pad lying inside the hole's pad copper (KiCad's
        ``_Pad_Via`` stitching ring), at most STITCH_MAX_DRILL_FRACTION of
        the hole's drill and under MOUNT_HOLE_MIN_DRILL.

    A slot, a second large drill anywhere else (a stabilizer, a bracket of
    mixed holes) or loose copper off the hole is not a screw hole."""
    drilled = [p for p in pads if (p.drill or 0.0) > 0]
    if not drilled:
        return None
    p0 = max(drilled, key=lambda p: p.drill)
    if p0.drill < MOUNT_HOLE_MIN_DRILL or not _round_drill(p0):
        return None
    hx, hy = _hole_centre(p0)
    reach = _copper_radius(p0)
    for q in pads:
        if q is p0:
            continue
        drill = q.drill or 0.0
        if drill > 0 and not _round_drill(q):
            return None
        qx, qy = _hole_centre(q) if drill > 0 else (q.global_x, q.global_y)
        d = math.hypot(qx - hx, qy - hy)
        if d <= SAME_CENTRE_MM:
            continue                        # the same hole, drawn again
        if (drill > 0 and q.pad_type == 'thru_hole'
                and drill < MOUNT_HOLE_MIN_DRILL
                and drill <= STITCH_MAX_DRILL_FRACTION * p0.drill
                and d + drill / 2.0 <= reach + 1e-6):
            continue                        # a stitching via in the hole's pad
        return None
    return p0


def mounting_holes(pcb) -> List[MountingHole]:
    """Every footprint that is ONE round screw hole: at most
    MOUNT_HOLE_MAX_PADS pads, the widest drill ROUND and MOUNT_HOLE_MIN_DRILL
    or more, every other pad the same hole drawn again (a hole drawn as front
    and back pads is one hole) or a small plated stitching via inside its pad
    (``_main_hole``), none on a signal net (none, ``unconnected-*`` or a
    ground-named net). A milled slot or cut-out is not a screw hole, and
    neither is a part whose several holes take no screw (a keyboard
    stabilizer's four, a bracket of mixed holes). The hole's centre and drill
    are the screw hole's own, so the head ring is centred on it."""
    out = []
    for ref, fp in sorted(pcb.footprints.items()):
        pads = fp.pads
        if not pads or len(pads) > MOUNT_HOLE_MAX_PADS:
            continue
        p0 = _main_hole(pads)
        if p0 is None:
            continue
        if any(not _NO_SIGNAL_NET.match(p.net_name or '') for p in pads):
            continue
        hx, hy = _hole_centre(p0)
        out.append(MountingHole(
            ref, hx, hy, p0.drill,
            frozenset(p.net_id for p in pads
                      if p.net_id and not _UNCONNECTED_NET.match(p.net_name or ''))))
    return out


def _seg_dist(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def hole_copper(pcb, ring: Optional[float] = None
                ) -> List[Tuple[MountingHole, List[dict]]]:
    """``[(hole, hits)]`` for every mounting hole, ``hits`` being the tracks
    and vias of another net whose copper comes within the screw-head ring of
    the hole's edge, nearest first. Each hit is ``{kind, layer, net, gap}``,
    ``gap`` being the copper-to-hole-edge distance in mm."""
    names = {nid: n.name for nid, n in pcb.nets.items()}
    out = []
    for hole in mounting_holes(pcb):
        reach = hole.drill / 2.0 + hole.ring(ring)
        hits = []
        for s in pcb.segments:
            if getattr(s, 'graphic', False) or s.net_id in hole.nets:
                continue
            d = _seg_dist(hole.x, hole.y, s.start_x, s.start_y, s.end_x, s.end_y)
            if d - s.width / 2.0 < reach:
                hits.append({'kind': 'track', 'layer': s.layer,
                             'net': names.get(s.net_id, str(s.net_id)),
                             'gap': d - s.width / 2.0 - hole.drill / 2.0})
        for v in pcb.vias:
            if v.net_id in hole.nets:
                continue
            d = math.hypot(v.x - hole.x, v.y - hole.y)
            if d - v.size / 2.0 < reach:
                hits.append({'kind': 'via', 'layer': '*',
                             'net': names.get(v.net_id, str(v.net_id)),
                             'gap': d - v.size / 2.0 - hole.drill / 2.0})
        hits.sort(key=lambda h: h['gap'])
        out.append((hole, hits))
    return out


def keepout_polygon(hole: MountingHole, ring: Optional[float] = None,
                    sides: int = 24) -> List[Tuple[float, float]]:
    """The screw-head ring as a regular polygon that CONTAINS the circle (its
    edges sit at the ring, its corners a little past it)."""
    r = (hole.drill / 2.0 + hole.ring(ring)) / math.cos(math.pi / sides)
    return [(hole.x + r * math.cos(2 * math.pi * k / sides),
             hole.y + r * math.sin(2 * math.pi * k / sides))
            for k in range(sides)]
