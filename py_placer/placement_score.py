#!/usr/bin/env python3
"""Placement quality on a COPPER-FREE board -- the terms a lap can be ranked by (#894).

    python3 -X utf8 py_placer/placement_score.py BOARD [--json PATH] [--intent PATH]

Why this exists
---------------
On a copper-free board every placement lap scores the same `blocking` (the
board's unrouted count, which is the ROUTING half's job) and the same `quality`
(`vias 0, copper_mm 0.0, segments 0`). So the ledger cannot rank two placements
and the plateau test has nothing to compare: measured on run 25, seven laps of
one board produced one number, while the layout that shipped had a crossed USB
pair, a connector whose pin order fought the bridge IC's, three nets pushed to
the back cutting the ground pour, and 22 vias for 24 nets. The routing half
paid for all of it and the placement half's instruments said clean.

NO SCALAR. There is deliberately no aggregate over these terms, and no weight
anywhere in this file. #694 is why: a corridor term's measured sign REVERSED
while an aggregate verdict kept printing PASS, because collapsing several
signals into one mark means no reader can say which of its inputs moved. Laps
are compared by `compare_terms`, which is PARETO -- better on every term it can
compare, worse on every term, or `mixed` with both sides named.

Every term CALLS the implementation that already exists rather than mirroring
it (`board_context.pin_order_rows` for pin order and pair span,
`floorplan.grade`'s measured proximity rows for cluster distance,
`board_context.serves_map` for the undeclared half, `legality.pad_rect` /
`rect_area` for pad geometry). A re-implementation of a grader in this repo has
already been measured disagreeing with it 83 times, worst 0.234mm.

Vacuity
-------
A term that could not be measured reports `ran: False` with a `reason` and
`value: None`. It NEVER reports 0. "No differential pair on this board" and
"the pairs are all perfectly short" are different answers, and a score that
cannot tell them apart will rank a board nothing measured above one that was
examined and found wanting.

Exit codes
    0  graded (any values)
    2  bad arguments
    3  board state (missing file, unparseable board)
This tool grades nothing and gates nothing, so there is no "4".
"""
import argparse
import contextlib
import json
import math
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('', 'py_router', 'py_placer', 'py_tools'):
    _p = os.path.join(_ROOT, _d) if _d else _ROOT
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

SCHEMA = 1

#: The terms, IN ORDER, published so a consumer never invents one. There is
#: deliberately NO aggregate over them: see the module docstring.
TERM_ORDER = ('pair_length', 'pin_order_crossings', 'cluster_to_pin',
              'plane_cut_proxy', 'balance')

#: Every term is lower-is-better. Shipped per term rather than assumed, so a
#: future term cannot silently invert `compare_terms` by being added.
_DIRECTION = 'lower-is-better'

#: `plane_cut_proxy` is defined on two layers only -- see its docstring.
PLANE_PROXY_LAYERS = 2

#: Below this the board has no long axis worth calling one, and `balance`
#: abstains rather than picking a side of a coin flip.
SQUARE_TOL = 0.01


def _term(value, unit, basis=None, **extra):
    """A measured term.

    `basis` names the POPULATION the number was taken over, when that
    population is not fixed by the board. Two totals over different
    populations are not larger and smaller versions of each other -- the same
    argument `converge.commensurability` makes about `blocking` -- so
    `term_deltas` refuses to compare a term whose basis moved, rather than
    reporting a change the placement did not cause.

    Measured, and the reason this exists: `plane_cut_proxy` counts nets forced
    around LOCKED parts, and across four laps of one board the blocker set
    went 3 -> 11 -> 3 as parts were frozen. Its value went 6.8 -> 101.4 -> 0.0
    with it. Most of that is bookkeeping about what the operator locked, not a
    fact about the arrangement.
    """
    out = {'ran': True, 'reason': None, 'value': value, 'unit': unit,
           'direction': _DIRECTION, 'basis': basis}
    out.update(extra)
    return out


def _skip(reason, unit, **extra):
    """A term that did not measure. `value` is None, NEVER 0 (see Vacuity)."""
    out = {'ran': False, 'reason': reason, 'value': None, 'unit': unit,
           'direction': _DIRECTION, 'basis': None}
    out.update(extra)
    return out


# --------------------------------------------------------------- pin order

def pin_order_rows(pcb_data, pcb_file, clearance):
    """`board_context.pin_order_rows`, called. Kept as one call site because
    it builds a QuenchState and both pin-order terms read the same rows."""
    import board_context
    return board_context.pin_order_rows(pcb_data, pcb_file, clearance)


def pin_order_crossings(pin_order_doc) -> dict:
    """How many part pairs must cross to be routed at all.

    PURE -- takes what `pin_order_rows` returned, so it can be tested without
    a board. `inversions` is `placement.pair_order`'s answer and a proven
    LOWER BOUND on the crossings any router must pay (Supowit 1987; Leiserson
    & Pinter 1983); this counts the rows whose verdict is CROSSED and does not
    re-derive it.

    UNDETERMINED rows are counted separately and EXCLUDED from the value.
    `board_context._verdict` declares that a tie on the channel axis outranks
    the inversion count, because the tie-break invented an order the geometry
    does not have -- so calling such a row 0 crossings would tell a reader the
    pin order is fine when nothing measured it.
    """
    unit = 'crossed part pairs'
    if not isinstance(pin_order_doc, dict):
        return _skip('pin order was not measured', unit)
    err = pin_order_doc.get('error')
    if err:
        # The Pillow trap: `board_context.pin_order_rows` imports
        # `render_placement`, whose module scope runs a dependency check that
        # RAISES without Pillow -- and its own call site swallows it, so the
        # failure arrives as an empty row list. Empty rows and "no pair
        # crosses" look identical, which is why the error is checked first.
        return _skip(f'board_context.pin_order_rows could not measure: {err}',
                     unit)
    rows = pin_order_doc.get('rows') or []
    iface = [r for r in rows if r.get('scope') == 'interface']
    if not iface:
        return _skip('no part pair on this board shares >= 2 scoring nets, '
                     'so there is no pin order to agree or disagree', unit)
    crossed = [r for r in iface if r.get('verdict') == 'CROSSED']
    undet = [r for r in iface
             if str(r.get('verdict') or '').startswith('UNDETERMINED')]
    if len(undet) == len(iface):
        return _skip(
            f'all {len(iface)} row(s) tie on the channel axis, so every '
            f'inversion count is one the tie-break invented rather than one '
            f'the geometry has', unit, rows=len(iface), undetermined=len(undet))
    return _term(len(crossed), unit, rows=len(iface),
                 undetermined=len(undet),
                 inversions=sum(r.get('inversions') or 0 for r in crossed),
                 pairs=[{'a': r['a'], 'b': r['b'], 'nets': r.get('nets'),
                         'inversions': r.get('inversions'),
                         'span_mm': r.get('span_mm')} for r in crossed])


def pair_length(pin_order_doc) -> dict:
    """The worst straight-line span of a declared differential pair, in mm.

    PURE, over the same rows. `span_mm` is `board_context._pair_span_mm`'s
    number and PR #917 added it for exactly this: over the nets two parts
    share, the MINIMUM pad-centre distance per net, then the MAXIMUM over
    nets. Worst-net, best-pad -- a bus is as long as its longest member.

    Reads the PAIR-scoped rows only. A differential pair's polarity is a
    two-net question and asking it as part of the whole interface hides it:
    on the run-25 fixture one part pair AGREES over three shared nets while
    the two pair nets alone are CROSSED.
    """
    unit = 'mm'
    if not isinstance(pin_order_doc, dict):
        return _skip('pin order was not measured', unit)
    if pin_order_doc.get('error'):
        return _skip(f"board_context.pin_order_rows could not measure: "
                     f"{pin_order_doc['error']}", unit)
    rows = [r for r in (pin_order_doc.get('rows') or [])
            if str(r.get('scope') or '').startswith('pair ')]
    spans = [(r['span_mm'], r) for r in rows
             if isinstance(r.get('span_mm'), (int, float))]
    if not spans:
        return _skip(
            'list_nets.find_differential_pairs found no differential pair on '
            'this board (it is name-based, and 2-terminal resonators are '
            'rejected). --impedance-nets is a route.py glob list, not a pair '
            'source, and is deliberately not read here', unit)
    worst, row = max(spans, key=lambda t: t[0])
    return _term(round(float(worst), 3), unit,
                 pairs=[{'a': r['a'], 'b': r['b'], 'scope': r['scope'],
                         'span_mm': r['span_mm'], 'verdict': r.get('verdict')}
                        for _s, r in sorted(spans, key=lambda t: -t[0])],
                 worst_pair=f"{row['a']}~{row['b']} {row['scope']}")


# ----------------------------------------------------------- cluster to pin

def cluster_to_pin(pcb_data, pcb_file, *, intent=None, clearance=None) -> dict:
    """How far each passive sits from the pin it serves, worst case, in mm.

    TWO channels, because neither covers the board alone:

    * DECLARED -- #902's `proximity` claims, graded by `floorplan.grade` and
      read off `GradeResult.proximity_measured`, which reports a clause that
      PASSES as well as one that fails. This is the only channel that can
      reach a 3-pad regulator's bulk capacitors at all.
    * INFERRED -- `board_context.serves_map`, the decap tether election inside
      its own radius, for the parts nobody declared.

    NOT the raw tether election, and #894 says so explicitly: it is syntactic
    (a ref starting `C` bridging exactly two nets) and its target must carry 4
    copper pads, so a 3-pad SOT89 regulator can never be elected at any radius
    -- which is how a regulator's own bulk capacitors came to be graded
    against a USB socket.
    """
    unit = 'mm'
    rows, declared_n = [], 0
    if intent is not None:
        try:
            from placement import floorplan as fp
            res = fp.grade(intent, pcb_data, pcb_file, clearance=clearance)
            for r in (res.proximity_measured or []):
                rows.append({'ref': r['ref'], 'near': r['near'],
                             'gap_mm': r['gap_mm'], 'limit_mm': r.get('limit_mm'),
                             'passes': r.get('passes'), 'basis': r.get('basis'),
                             'source': 'declared', 'claim': r.get('claim')})
            declared_n = len(rows)
        except Exception as exc:                             # noqa: BLE001
            return _skip(f'floorplan.grade could not measure the declared '
                         f'proximity claims: {type(exc).__name__}: {exc}', unit)
    inferred = {}
    try:
        import board_context
        inferred = board_context.serves_map(pcb_data)
    except Exception:                                        # noqa: BLE001
        inferred = {}
    named = {(r['ref'], r['near']) for r in rows}
    for cap, (ic, dist) in sorted(inferred.items()):
        if (cap, ic) in named:
            continue
        rows.append({'ref': cap, 'near': ic, 'gap_mm': round(float(dist), 4),
                     'limit_mm': None, 'passes': None, 'basis': 'centre',
                     'source': 'inferred', 'claim': None})
    if not rows:
        return _skip(
            'the intent declares no proximity claims and no capacitor sits '
            'within the decap election radius of an elected IC, so nothing '
            'says which part serves which pin', unit,
            declared=0, inferred=0)
    worst = max(rows, key=lambda r: r['gap_mm'])
    # BASIS: which pairs were measured at all. A lap that declares one more
    # proximity claim, or whose election reaches one more capacitor, is taking
    # a maximum over a different population.
    return _term(round(float(worst['gap_mm']), 4), unit,
                 basis=sorted(f"{r['ref']}~{r['near']}" for r in rows),
                 declared=declared_n, inferred=len(rows) - declared_n,
                 worst_pair=f"{worst['ref']}~{worst['near']}",
                 rows=sorted(rows, key=lambda r: -r['gap_mm']))


# ---------------------------------------------------------- plane cut proxy

def plane_cut_proxy(pcb_data, pcb_file=None) -> dict:
    """Total straight-line length of nets forced around a locked part, in mm.

    On two layers a net whose two ends sit on opposite sides of an immovable
    part must cross under it or go around, and either way it removes reference
    copper. `routability.corridor_cut_mm`'s docstring is the authority for why
    LENGTH is the right currency there -- "the geometric lower bound on
    reference-plane copper the crossing removes, which is the quantity
    check_impedance's void-run counter later grades" -- and that argument is
    also why this term is gated on exactly two copper layers.

    It does NOT call `corridor_cut_mm`, for two reasons that function states
    itself: it needs a DECLARED corridor, which a fresh placement board does
    not have; and its chord is POSITION-INVARIANT along the lane, so it cannot
    rank two placements that differ by a nudge, which is this term's whole job.

    "Endpoints" is ill-defined for an N-pad net, so the definition used is the
    DIAMETER PAIR -- the two pads furthest apart -- and each net is counted
    once, against its worst blocker. Said here and in the payload rather than
    left for a reader to infer.

    REPORTED, never optimised: like the corridor term it is a screening
    signal, and #893 (which consumes these terms as a search cost) must not
    put a part-nudging search on it.
    """
    unit = 'mm'
    layers = list(getattr(pcb_data.board_info, 'copper_layers', []) or [])
    if len(layers) != PLANE_PROXY_LAYERS:
        return _skip(
            f'plane_cut_proxy is defined on a {PLANE_PROXY_LAYERS}-layer '
            f'board, where a crossing removes reference copper there is no '
            f'other layer to carry; this board has {len(layers)} copper '
            f'layer(s)', unit, copper_layers=len(layers))
    try:
        from placement import part_class
        from placement import body as body_mod
        from placement import floorplan as fp
        from geometry_utils import segments_intersect_tuple
        from placement import parser as kparser
    except Exception as exc:                                 # noqa: BLE001
        return _skip(f'could not import the geometry this term calls: '
                     f'{type(exc).__name__}: {exc}', unit)
    blockers = set()
    try:
        blockers |= set(part_class.mechanical_parts(pcb_data) or {})
    except Exception:                                        # noqa: BLE001
        pass
    try:
        if pcb_file:
            blockers |= set(kparser.extract_locked_refs(pcb_file) or ())
    except Exception:                                        # noqa: BLE001
        pass
    blockers |= {r for r, f in (pcb_data.footprints or {}).items()
                 if getattr(f, 'locked', False)}
    if not blockers:
        return _skip(
            'no footprint is locked and none classifies as mechanical, so '
            'there is nothing a net can be forced around', unit)
    try:
        bodies = body_mod.board_bodies(pcb_data, pcb_file)
    except Exception as exc:                                 # noqa: BLE001
        return _skip(f'placement.body could not be read: '
                     f'{type(exc).__name__}: {exc}', unit)
    rects = {}
    for ref in sorted(blockers):
        rect, src = fp.drawn_body_rect(bodies.get(ref),
                                       (pcb_data.footprints or {}).get(ref))
        if rect is not None:
            rects[ref] = (rect, src)
    if not rects:
        return _skip('every locked or mechanical part answers body source '
                     '"none", so none of them has geometry to be forced '
                     'around', unit, blockers=len(blockers))
    # Diameter pair per net, over pads NOT belonging to the blocker itself.
    pads_by_net = {}
    for ref, fpo in (pcb_data.footprints or {}).items():
        for pad in (fpo.pads or ()):
            nid = getattr(pad, 'net_id', 0) or 0
            if nid:
                pads_by_net.setdefault(nid, []).append(
                    (ref, float(pad.global_x), float(pad.global_y)))
    total, hits = 0.0, []
    for nid, pads in sorted(pads_by_net.items()):
        if len(pads) < 2:
            continue
        best, pa, pb = -1.0, None, None
        for i, p in enumerate(pads):
            for q in pads[i + 1:]:
                d = (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2
                if d > best:
                    best, pa, pb = d, p, q
        a, b = (pa[1], pa[2]), (pb[1], pb[2])
        for ref, (rect, src) in rects.items():
            if ref in (pa[0], pb[0]):
                continue          # its own net is not forced around it
            x0, y0, x1, y1 = rect
            # "Opposite sides" means the chord CROSSES the body: an endpoint
            # inside the rect is a pad under the part, not a net forced past
            # it.
            if (x0 <= a[0] <= x1 and y0 <= a[1] <= y1) or \
               (x0 <= b[0] <= x1 and y0 <= b[1] <= y1):
                continue
            edges = (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                     ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0)))
            if any(segments_intersect_tuple(a, b, e[0], e[1]) for e in edges):
                length = math.hypot(b[0] - a[0], b[1] - a[1])
                total += length
                hits.append({'net_id': nid,
                             'net': (pcb_data.nets.get(nid).name
                                     if pcb_data.nets.get(nid) else None),
                             'blocker': ref, 'body_source': src,
                             'length_mm': round(length, 3)})
                break             # once per net, against its worst blocker
    # BASIS: the blocker set. Which parts are locked is an operator decision
    # that changes between laps -- measured, 3 -> 11 -> 3 over four laps of
    # one board -- and the total moves with it for reasons the arrangement did
    # not cause. Two laps that froze different parts are not comparable here.
    return _term(round(total, 3), unit, basis=sorted(rects),
                 nets=len(hits), blockers=len(rects),
                 definition='diameter pair per net; counted once per net',
                 rows=sorted(hits, key=lambda r: -r['length_mm'])[:20])


# ------------------------------------------------------------------ balance

def pad_area_balance(pcb_data) -> dict:
    """Pad-copper-area first moment along the board's LONG axis, as a fraction
    of the span.

    #894 asks for "mass distribution along the board's long axis ... pad-area
    first moment", and that is what this is. It deliberately does NOT call
    `check_pockets.census_scalars`, which is a different quantity three ways:
    courtyard-area weighted rather than pad-area, a hypot of BOTH axes with no
    long-axis notion, and per-side with a headline side -- and it costs a
    `congestion_bins` grid pass.

    Never call either number "the centroid" without its weight. The two
    disagree: on the run-25 fixture the count-weighted control reads 13.4% of
    span where the courtyard-area form reads 2.7%.
    """
    unit = 'fraction of span'
    bounds = getattr(pcb_data.board_info, 'board_bounds', None)
    if not bounds:
        return _skip('the board declares no Edge.Cuts outline, so it has no '
                     'span and no long axis', unit)
    x0, y0, x1, y1 = bounds
    span_x, span_y = float(x1 - x0), float(y1 - y0)
    if max(span_x, span_y) <= 0:
        return _skip('the board outline has zero extent', unit)
    if abs(span_x - span_y) / max(span_x, span_y) < SQUARE_TOL:
        return _skip(f'the board is square within {SQUARE_TOL:.0%} '
                     f'(x {span_x:.1f}mm, y {span_y:.1f}mm), so there is no '
                     f'long axis to balance along', unit,
                     span_mm=[round(span_x, 3), round(span_y, 3)])
    from placement import legality as leg
    axis = 'x' if span_x >= span_y else 'y'
    span = span_x if axis == 'x' else span_y
    centre = ((x0 + x1) / 2.0) if axis == 'x' else ((y0 + y1) / 2.0)
    num = area = 0.0
    n = 0
    for fpo in (pcb_data.footprints or {}).values():
        for pad in (fpo.pads or ()):
            try:
                a = leg.rect_area(leg.pad_rect(pad))
            except Exception:                                # noqa: BLE001
                continue
            if a <= 0:
                continue
            u = float(pad.global_x) if axis == 'x' else float(pad.global_y)
            num += a * u
            area += a
            n += 1
    if area <= 0:
        return _skip('no pad on this board has copper area, so there is no '
                     'mass to weigh', unit)
    centroid = num / area
    return _term(round(abs(centroid - centre) / span, 4), unit,
                 axis=axis, span_mm=round(span, 3),
                 centroid_mm=round(centroid, 3), centre_mm=round(centre, 3),
                 weight='pad_copper_area', pad_area_mm2=round(area, 3),
                 pads=n)


# ------------------------------------------------------------- the document

def placement_terms(pcb_data, pcb_file, *, clearance=None, intent=None) -> dict:
    """All five terms for one board."""
    t0 = time.time()
    if clearance is None:
        try:
            import list_nets
            clearance = list_nets.board_floor_knobs(pcb_file, clearance=None)[0]
        except Exception:                                    # noqa: BLE001
            import routing_defaults as _rd
            clearance = _rd.CLEARANCE
    po = pin_order_rows(pcb_data, pcb_file, clearance)
    terms = {
        'pair_length': pair_length(po),
        'pin_order_crossings': pin_order_crossings(po),
        'cluster_to_pin': cluster_to_pin(pcb_data, pcb_file, intent=intent,
                                         clearance=clearance),
        'plane_cut_proxy': plane_cut_proxy(pcb_data, pcb_file),
        'balance': pad_area_balance(pcb_data),
    }
    return {'schema': SCHEMA, 'kind': 'placement-terms',
            'board': os.path.abspath(pcb_file) if pcb_file else None,
            'segments': len(getattr(pcb_data, 'segments', ()) or ()),
            'clearance': clearance,
            'term_order': list(TERM_ORDER),
            'terms': terms,
            'elapsed_s': round(time.time() - t0, 2)}


# --------------------------------------------------------------- comparison

def term_deltas(old_terms, new_terms) -> list:
    """The raw + delta table, as data.

    One computation shared by the printer and the judge below, so a report and
    a verdict can never describe two different comparisons.
    """
    out = []
    for name in TERM_ORDER:
        a = (old_terms or {}).get(name) or {}
        b = (new_terms or {}).get(name) or {}
        av = a.get('value') if a.get('ran') else None
        bv = b.get('value') if b.get('ran') else None
        row = {'term': name, 'old': av, 'new': bv,
               'unit': b.get('unit') or a.get('unit'),
               'direction': b.get('direction') or a.get('direction')
               or _DIRECTION}
        comparable = (isinstance(av, (int, float))
                      and isinstance(bv, (int, float)))
        if comparable and a.get('basis') != b.get('basis'):
            # The two numbers are totals over DIFFERENT populations. Same
            # argument `converge.commensurability` makes about `blocking`: a
            # drop across such a pair is not evidence the placement improved,
            # it may be evidence that less was measured. Named, not silently
            # dropped -- a reader has to be able to see WHY it was not judged.
            row['delta'] = None
            row['judgement'] = 'not-comparable'
            row['why'] = 'the basis moved'
            row['basis_added'] = sorted(set(b.get('basis') or ())
                                        - set(a.get('basis') or ()))
            row['basis_removed'] = sorted(set(a.get('basis') or ())
                                          - set(b.get('basis') or ()))
        elif comparable:
            row['delta'] = round(bv - av, 6)
            row['judgement'] = ('same' if row['delta'] == 0
                                else 'better' if row['delta'] < 0 else 'worse')
        else:
            # NOT 0. A term measured on one side only is excluded from the
            # comparison entirely -- defaulting it would make an unmeasured
            # lap look better or worse than a measured one, which is the shape
            # `_score_key` already refuses for `blocking`.
            row['delta'] = None
            row['judgement'] = 'not-comparable'
            row['why'] = 'measured on one lap only'
        out.append(row)
    return out


def compare_terms(old_terms, new_terms):
    """`(verdict, detail)` -- PARETO, never a scalar.

    `verdict` is one of `better`, `worse`, `mixed`, `same`, `no-common-terms`,
    over the terms BOTH laps measured. Two laps that improve DIFFERENT terms
    are `mixed`, and both sides are named: that is #694's inversion made
    visible instead of arbitrated by a weight nobody can defend.
    """
    detail = term_deltas(old_terms, new_terms)
    judged = [r for r in detail if r['judgement'] != 'not-comparable']
    if not judged:
        return 'no-common-terms', detail
    better = [r for r in judged if r['judgement'] == 'better']
    worse = [r for r in judged if r['judgement'] == 'worse']
    if better and worse:
        return 'mixed', detail
    if better:
        return 'better', detail
    if worse:
        return 'worse', detail
    return 'same', detail


def format_delta(detail) -> str:
    """One line per comparable term, for a human. Never a total."""
    bits = []
    for r in detail:
        if r['judgement'] == 'not-comparable':
            continue
        bits.append(f"{r['term']} {r['old']} -> {r['new']} "
                    f"({r['delta']:+g}, {r['judgement']})")
    return '; '.join(bits) or 'no term was measured on both laps'


# ---------------------------------------------------------------------- CLI

def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('board')
    p.add_argument('--json', metavar='PATH',
                   help='write the document here (stdout otherwise)')
    p.add_argument('--intent', metavar='PATH',
                   help="a floorplan intent. Its `proximity` claims are what "
                        "`cluster_to_pin`'s DECLARED channel grades; without "
                        "one that channel reports only what the decap "
                        "election infers, and says so")
    p.add_argument('--clearance', type=float,
                   help="the board's own floor is read when this is omitted")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if not os.path.isfile(args.board):
        print(f'board not found: {args.board}', file=sys.stderr)
        return 3
    intent = None
    if args.intent:
        try:
            from placement import floorplan as fp
            intent = fp.load_intent(args.intent)
        except Exception as exc:                             # noqa: BLE001
            print(f'--intent: {type(exc).__name__}: {exc}', file=sys.stderr)
            return 2
    # STDOUT IS THE DOCUMENT. The quench and the parser both print warnings,
    # and a caller json.loads() this whole stream -- `board_context.main` does
    # the same for the same reason.
    with contextlib.redirect_stdout(sys.stderr):
        from kicad_parser import parse_kicad_pcb
        try:
            pcb = parse_kicad_pcb(args.board)
        except Exception as exc:                             # noqa: BLE001
            print(f'could not parse {args.board}: {type(exc).__name__}: {exc}',
                  file=sys.stderr)
            return 3
        doc = placement_terms(pcb, args.board, clearance=args.clearance,
                              intent=intent)
    text = json.dumps(doc, indent=2, sort_keys=True)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            fh.write(text + '\n')
    else:
        print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
