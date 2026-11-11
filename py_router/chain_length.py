"""The length of a lane that runs through series parts (#1799, #1812).

A signal split by a series part -- an AC-coupling capacitor, a series
resistor -- is two nets on the board, and every per-net grade (a `.kicad_dru`
length or skew rule, a `--length-groups` member) measures each net alone.
A limit or a skew stated for the lane end to end needs the nets SUMMED, and
this module is the one measurement every grader of such a lane reads:
check_drc's grade of a recorded AC-coupled chain, board_score's
`--length-groups` chains, and route_diff's shipped-copper verdict on the
chain it matched.

Each net is measured by `net_queries.net_match_length` -- the measure the
per-net graders use (track and via copper plus pad-to-die, and None until the
net's pads are joined by track copper) -- and the chain's `track_mm` is their
sum. A series part's own span (its two copper pads' centres) is NOT track,
so it is reported apart (`span_mm`), never added: a reader comparing with a
hand calculation sees both numbers. Consecutive nets must be joined by one
part with exactly two copper pads, one on each (`series_part_between`); a
pair of nets nothing joins is named in `unbridged`, a finding about the
spec rather than a length.
"""
import math
from typing import Dict, List, Optional, Sequence


def series_part_between(pcb_data, net_a: int, net_b: int) -> Optional[dict]:
    """The part that joins `net_a` to `net_b` in series: a footprint whose
    copper pads (`kicad_parser.non_aperture_pads`, so a paste-only window is
    not a pin) are exactly two, one on each net. {'ref', 'span_mm'} -- the
    distance between the two pads' centres -- or None."""
    from kicad_parser import non_aperture_pads
    for fp in pcb_data.footprints.values():
        pads = [p for p in non_aperture_pads(fp)
                if getattr(p, 'pad_type', '') != 'np_thru_hole']
        if len(pads) != 2:
            continue
        a, b = pads
        if {a.net_id, b.net_id} != {net_a, net_b} or net_a == net_b:
            continue
        return {'ref': fp.reference,
                'span_mm': round(math.hypot(a.global_x - b.global_x,
                                            a.global_y - b.global_y), 4)}
    return None


def measure_chain(pcb_data, nets: Sequence, mode: str = 'total') -> dict:
    """Measure one chain of nets (names or ids, in lane order).

    Returns {'nets': [names], 'missing_nets': [names not on the board],
    'net_mm': {name: length or None}, 'unjoined_nets': [names whose pads are
    not joined by track copper], 'joints': [{'nets': [a, b], 'part': ref or
    None, 'span_mm': float or None}], 'unbridged': ['a -> b', ...],
    'track_mm': the sum, or None unless every net was measured,
    'span_mm': the series parts' spans summed, or None when a joint has no
    part}. `mode` is net_match_length's ('total' or 'pin_pair')."""
    from net_queries import net_match_length
    id_of = {n.name: nid for nid, n in pcb_data.nets.items() if n.name}
    name_of = {nid: n.name for nid, n in pcb_data.nets.items()}
    names, ids, missing = [], [], []
    for x in nets:
        if isinstance(x, int):
            nm, nid = name_of.get(x), x if x in name_of else None
        else:
            nm, nid = x, id_of.get(x)
        names.append(nm if nm is not None else str(x))
        ids.append(nid)
        if nid is None:
            missing.append(names[-1])
    raw: Dict[str, Optional[float]] = {}
    for nm, nid in zip(names, ids):
        if nid is None:
            continue
        raw[nm] = net_match_length(pcb_data, nid, mode=mode)
    net_mm = {nm: None if L is None else round(L, 4) for nm, L in raw.items()}
    unjoined = [nm for nm, L in net_mm.items() if L is None]
    joints, unbridged, spans = [], [], []
    for k in range(len(ids) - 1):
        a, b = ids[k], ids[k + 1]
        part = (series_part_between(pcb_data, a, b)
                if a is not None and b is not None else None)
        joints.append({'nets': [names[k], names[k + 1]],
                       'part': part['ref'] if part else None,
                       'span_mm': part['span_mm'] if part else None})
        if part is None:
            unbridged.append(f'{names[k]} -> {names[k + 1]}')
        else:
            spans.append(part['span_mm'])
    measured = not missing and not unjoined
    return {'nets': names, 'missing_nets': missing, 'net_mm': net_mm,
            'unjoined_nets': unjoined, 'joints': joints, 'unbridged': unbridged,
            'track_mm': (round(math.fsum(raw.values()), 4)
                         if measured else None),
            'span_mm': (round(math.fsum(spans), 4) if not unbridged else None)}


def chain_label(names: List[str]) -> str:
    """How a chain is named in a grade: its nets in lane order."""
    return ' + '.join(names)


def pair_chain_skew(pcb_data, p_nets: Sequence, n_nets: Sequence) -> dict:
    """The end-to-end skew of a differential signal split across series
    parts: the P chain and the N chain measured by `measure_chain`.
    {'p': chain, 'n': chain, 'skew_mm': |P - N| of track or None when either
    side is unmeasured, 'spans_differ': the two sides' series spans differ
    by more than 1 um, so the track skew is not the whole electrical skew}."""
    p = measure_chain(pcb_data, p_nets)
    n = measure_chain(pcb_data, n_nets)
    skew = (round(abs(p['track_mm'] - n['track_mm']), 4)
            if p['track_mm'] is not None and n['track_mm'] is not None else None)
    differ = (p['span_mm'] is not None and n['span_mm'] is not None
              and abs(p['span_mm'] - n['span_mm']) > 1e-3)
    return {'p': p, 'n': n, 'skew_mm': skew, 'spans_differ': differ}
