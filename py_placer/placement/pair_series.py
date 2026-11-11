"""The series parts of a differential pair, and whether they sit as a pair (#1820).

A pair through AC-coupling caps or USB series resistors is two name-detected
pairs joined at a junction: one two-pad part on the P leg, one on the N leg.
The junctions are the ROUTER's own (`diff_xnet.find_ac_coupled_xnets`, which
joins two pairs only when BOTH polarities are bridged, each by its own
non-DNP part with two copper pads, on the same polarity), so a junction named
here is one `route_diff` routes as one signal.

The two parts share no net, so nothing in the placement objective pulls them
together, and a placement that splits them grades as one that does not --
until `route_diff` finds the split side too short to couple and routes it
single-ended. `junction_geometry` measures how the two sit against each other
and `split_reasons` says what departs from a matched pair:

  * the two on different faces;
  * their pad axes turned more than `TURN_TOL_DEG` from parallel;
  * one ahead of the other along the pair by more than `OFFSET_TOL_MM`;
  * their centres more than `GAP_TOL_MM` apart across the pair.

The limits are read off human-routed boards: 544 junctions on 110 boards of
the stress corpus (`boards_set*`). All 544 sit on one face; 539 are parallel
and 5 turned more than 5 degrees; the offset along the pair is 0.022 mm at
the 98th percentile and above 0.5 mm on 5; the centre gap is 2.53 mm at the
98th percentile and above 5 mm on 2. The four limits together name 7 of the
544 junctions, on 6 boards, and a split such as one part moved a few mm and
turned a quarter is named on two counts.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

#: Pad axes further from parallel than this, degrees, are not a matched pair.
TURN_TOL_DEG = 5.0
#: One part's centre this far ahead of the other's along the pair, mm.
OFFSET_TOL_MM = 0.5
#: The two centres this far apart, mm.
GAP_TOL_MM = 5.0


@dataclass(frozen=True)
class Junction:
    """One junction: pairs `pairs[0]` and `pairs[1]` ("POS/NEG" keys, as
    `board_context` names a composed pair), bridged by `p_ref` on the P leg
    and `n_ref` on the N leg; `pair_nets` the (P, N) net ids of each pair."""
    pairs: Tuple[str, str]
    p_ref: str
    n_ref: str
    pair_nets: Tuple[Tuple[int, int], Tuple[int, int]]


def pair_junctions(pcb_data) -> Tuple[List[Junction], List[str]]:
    """Every junction on the board, and the router's warnings (a pair bridged
    on one polarity only, which is not a junction). Pure read."""
    import list_nets
    from diff_xnet import find_ac_coupled_xnets
    from kicad_parser import non_aperture_pads
    from routing_config import DiffPairNet

    name_to_id: Dict[str, int] = {}
    for n in (pcb_data.nets or {}).values():
        if n.name:
            name_to_id.setdefault(n.name, n.net_id)
    dp, member = {}, {}
    for pos, neg in list_nets.find_differential_pairs(pcb_data) or ():
        ids = (name_to_id.get(pos), name_to_id.get(neg))
        if None in ids or 0 in ids:
            continue
        key = f'{pos}/{neg}'
        dp[key] = DiffPairNet(base_name=key, p_net_id=ids[0], n_net_id=ids[1],
                              p_net_name=pos, n_net_name=neg)
        member[ids[0]] = (key, 'P')
        member[ids[1]] = (key, 'N')
    if not dp:
        return [], []
    xnets, warnings = find_ac_coupled_xnets(pcb_data, dp)
    fps = pcb_data.footprints or {}
    out: Dict[frozenset, Dict[str, str]] = {}
    for xn in xnets:
        for ref in xn.bridge_refs:
            p0, p1 = non_aperture_pads(fps[ref])
            k0, pol = member[p0.net_id]
            k1 = member[p1.net_id][0]
            out.setdefault(frozenset((k0, k1)), {})[pol] = ref
    junctions = []
    for keys, pols in out.items():
        if 'P' in pols and 'N' in pols:
            ks = tuple(sorted(keys))
            junctions.append(Junction(
                ks, pols['P'], pols['N'],
                tuple((dp[k].p_net_id, dp[k].n_net_id) for k in ks)))
    junctions.sort(key=lambda j: (j.pairs, j.p_ref, j.n_ref))
    return junctions, list(warnings)


def junction_geometry(pcb_data, j: Junction) -> Optional[Dict]:
    """How the junction's two parts sit against each other, or None when a
    part or one of its pads is not on the board:

      * `sides`: each part's copper face ('F' / 'B');
      * `axis_turn_deg`: the angle between the two parts' pad axes (pad on
        one pair -> pad on the other), folded into [0, 90] -- 0 when they are
        parallel, whichever way round;
      * `offset_mm`: how far one part's centre is AHEAD of the other's along
        the pair -- along the line from the midpoint of the two pads on one
        pair to the midpoint of the two on the other (0 side by side);
      * `gap_mm`: the distance between the two parts' pad centres."""
    from kicad_parser import non_aperture_pads
    fps = pcb_data.footprints or {}
    if j.p_ref not in fps or j.n_ref not in fps:
        return None

    def pad_on(ref, net_id):
        for p in non_aperture_pads(fps[ref]):
            if p.net_id == net_id:
                return p
        return None
    pp = [pad_on(j.p_ref, nets[0]) for nets in j.pair_nets]
    nn = [pad_on(j.n_ref, nets[1]) for nets in j.pair_nets]
    if None in pp or None in nn:
        return None
    mid = [((pp[k].global_x + nn[k].global_x) / 2.0,
            (pp[k].global_y + nn[k].global_y) / 2.0) for k in (0, 1)]
    ux, uy = mid[1][0] - mid[0][0], mid[1][1] - mid[0][1]
    span = math.hypot(ux, uy)
    cp = ((pp[0].global_x + pp[1].global_x) / 2.0,
          (pp[0].global_y + pp[1].global_y) / 2.0)
    cn = ((nn[0].global_x + nn[1].global_x) / 2.0,
          (nn[0].global_y + nn[1].global_y) / 2.0)
    dx, dy = cp[0] - cn[0], cp[1] - cn[1]
    offset = abs(dx * ux + dy * uy) / span if span > 1e-9 else 0.0

    def axis(p):
        return math.degrees(math.atan2(p[1].global_y - p[0].global_y,
                                       p[1].global_x - p[0].global_x))
    turn = abs(axis(pp) - axis(nn)) % 180.0
    turn = min(turn, 180.0 - turn)

    from .legality import footprint_side

    def side(ref):
        return footprint_side(fps[ref])
    return {'sides': {j.p_ref: side(j.p_ref), j.n_ref: side(j.n_ref)},
            'axis_turn_deg': round(turn, 3), 'offset_mm': round(offset, 4),
            'gap_mm': round(math.hypot(dx, dy), 4)}


def split_reasons(geom: Dict, *, turn_deg: float = TURN_TOL_DEG,
                  offset_mm: float = OFFSET_TOL_MM,
                  gap_mm: float = GAP_TOL_MM) -> List[str]:
    """What departs from a matched pair, one phrase each ([] when none)."""
    out = []
    if len(set(geom['sides'].values())) > 1:
        out.append('on different faces')
    if geom['axis_turn_deg'] > turn_deg:
        out.append(f"turned {geom['axis_turn_deg']:g} deg from each other")
    if geom['offset_mm'] > offset_mm:
        out.append(f"{geom['offset_mm']:.2f} mm ahead of each other along "
                   f"the pair")
    if geom['gap_mm'] > gap_mm:
        out.append(f"{geom['gap_mm']:.2f} mm apart")
    return out
