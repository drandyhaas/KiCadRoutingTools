"""The intent's decap limits after the anneal: what is still broken, and a
repair that moves the cap (#1382, #1423).

`Model.set_limits` prices `decaps.max_distance_mm` and `max_pin_distance_mm`
per mm past the limit, a soft term: a miss of a tenth of a mm costs less than
the anneal's own noise, so it stops on it, and check_floorplan then grades the
placement with an ERROR the objective had priced. Nothing named the
residual, and nothing mended it.

* `residual(model)` -- every limit term the committed poses leave past its
  limit, as a row naming the rail, the cap, the pin (or chip) and the
  distance reached against the limit. Read the model's own terms, which are
  the grade's pairings measured as the grade measures them.
* `repair(model, movable, legal_at)` -- each broken term's cap (a pin term: the caps that can
  serve the pin, nearest first; a cap term: the cap) goes to the NEAREST legal
  pose, by displacement, that puts the term inside its limit without making any
  other limit term worse. A term no legal pose clears is left where it is and
  reported with the best distance any legal pose reached.
"""

import math
import time
from typing import Callable, Dict, List, Tuple

from .legality import EPS

#: Candidate poses lie on this grid, mm, round the point the limit is measured
#: from -- the step `simple.nearest_legal` walks.
STEP = 0.25
#: A cap term's candidate centre may lie this far (mm) past the limit from the
#: chip's courtyard centre: the cap's own pad reach (a pin term's is exact).
SLACK = 3.0
#: Poses tried and measured per mover before it is given up (legal ones only).
TRIES = 400
#: Candidate poses looked at per mover, legal or not.
LOOKS = 12000
#: A part with at most this many pads may also be tried at the other quarter
#: turns; a bigger one keeps its angle.
TURN_PADS = 3


def _rows_of(model, ls) -> List[Dict]:
    if not ls:
        return []
    T = model._lim._tether_terms
    fps = model.pcb.footprints
    out = []
    for i in ls:
        t = T[i]
        v, thr = model._lval[i], model._lthr[i]
        pairs = {m: d for m, d in model._lpair[i].items() if d is not None}
        near = min(pairs, key=pairs.get) if pairs else None
        if t.kind == 'pin':
            ic = t.data['ic']
            pad = fps[ic].pads[t.data['pad_index']]
            row = {'kind': 'pin', 'ic': ic, 'pin': str(pad.pad_number),
                   'rail': pad.net_name, 'cap': near}
        else:
            cap = t.data['cap']
            from .grouping import is_ground
            rails = sorted({q.net_name for q in fps[cap].pads
                            if q.net_id and not is_ground(q.net_name)})
            row = {'kind': 'decap', 'ic': near, 'pin': None,
                   'rail': '|'.join(rails), 'cap': cap}
        row.update(value=round(v, 3), limit=round(thr, 3), over=round(v - thr, 3), term=i)
        out.append(row)
    return out


def broken_terms(model) -> List[int]:
    """The limit terms the live poses put past their limit (re-measured)."""
    if getattr(model, '_lim', None) is None:
        return []
    model._limits_all()
    return [i for i in model._lterms if model._lval[i] > model._lthr[i] + EPS]


def residual(model) -> List[Dict]:
    """`[{'kind': 'pin'|'decap', 'ic', 'pin', 'rail', 'cap', 'value', 'limit',
    'over', 'term'}]` for every limit term still past its limit, worst first.
    A pin row's `cap` is the nearest cap that can serve the pin; a cap row's
    `ic` is the nearest chip on its rail."""
    rows = _rows_of(model, broken_terms(model))
    return sorted(rows, key=lambda r: (-r['over'], r['term']))


def residual_text(r: Dict) -> str:
    """'U1 pin P12 (/VDD) is 31.65 mm from C20, the nearest decoupling cap
    (limit 8.46 mm)' -- or the cap-to-chip form."""
    if r['kind'] == 'pin':
        return (f"{r['ic']} pin {r['pin']} ({r['rail']}) is {r['value']:.2f} mm from "
                f"{r['cap']}, the nearest decoupling cap (limit {r['limit']:.2f} mm)")
    return (f"{r['cap']} ({r['rail']}) is {r['value']:.2f} mm from {r['ic']}, the chip it "
            f"decouples (limit {r['limit']:.2f} mm)")


def stuck_text(r: Dict) -> str:
    """What the repair found for a row it could not clear: why, in words."""
    w = r.get('would_break')
    if w is not None:
        who = (f"{w['ic']} pin {w['pin']} ({w['rail']})" if w['kind'] == 'pin'
               else f"{w['cap']} ({w['rail']})")
        return (f"; a legal pose inside the limit exists but would put {who} past its "
                f"own limit")
    if r.get('reach') is None:
        return "; no cap on the rail had a legal pose to try"
    return f"; no legal pose of the cap got closer than {r['reach']:.2f} mm"


def _target(model, i, ref) -> Tuple[Tuple[float, float], float]:
    """`((x, y), reach)`: the point term `i`'s limit is measured from, and how
    far `ref`'s centre can lie from it and still come inside the limit. A pin
    term's reach is exact where the model keeps it (the pair's lower bound:
    centre distance less the pin's half diagonal and the cap's pad reach), so a
    pose outside it is never measured."""
    t = model._lim._tether_terms[i]
    thr = model._lthr[i]
    if t.kind == 'pin':
        net, pr = model._lim_pin_rect(i)
        hd = math.hypot(pr[2] - pr[0], pr[3] - pr[1]) / 2.0
        r = model._lrad.get((ref, net))
        return (((pr[0] + pr[2]) / 2.0, (pr[1] + pr[3]) / 2.0),
                thr + hd + (SLACK if r is None else r) + 1e-3)
    pairs = {m: d for m, d in model._lpair[i].items() if d is not None}
    chip = min(pairs, key=pairs.get) if pairs else next(iter(model._lmem[i]))
    x0, y0, x1, y1 = model.state.parts[chip].rect()
    return (((x0 + x1) / 2.0, (y0 + y1) / 2.0),
            thr + SLACK + math.hypot(x1 - x0, y1 - y0) / 2.0)


def _movers(model, i, movable) -> List[str]:
    """The parts whose move can clear term `i`, nearest to the term first."""
    t = model._lim._tether_terms[i]
    if t.kind == 'decap':
        return [t.data['cap']] if t.data['cap'] in movable else []
    pairs = model._lpair[i]
    mem = sorted(model._lmem[i], key=lambda m: (pairs.get(m) is None, pairs.get(m) or 0.0, m))
    return [m for m in mem if m in movable]


def _over(model) -> Dict[int, float]:
    return {i: max(0.0, model._lval[i] - model._lthr[i]) for i in model._lterms}


def _try_mover(model, i, ref, legal_at) -> Dict:
    """Move `ref` to the nearest legal pose that clears term `i` and worsens
    no other limit term. Returns {'moved': (from, to) | None, 'best': the
    nearest any legal pose put term `i`, mm, 'blocks': a term a pose inside
    the limit would have put past its own, else None}."""
    st = model.state
    p = st.parts[ref]
    was = (p.x, p.y, p.rot)
    (tx, ty), reach = _target(model, i, ref)
    thr = model._lthr[i]
    base = _over(model)
    turns = [was[2]]
    if len(model.pcb.footprints[ref].pads) <= TURN_PADS:
        turns += [(was[2] + d) % 360.0 for d in (180.0, 90.0, 270.0)]
    n = int(reach / STEP) + 1
    cand = []
    for a in range(-n, n + 1):
        for b in range(-n, n + 1):
            x, y = tx + a * STEP, ty + b * STEP
            if math.hypot(x - tx, y - ty) <= reach:
                d = math.hypot(x - was[0], y - was[1])
                for k, rot in enumerate(turns):
                    cand.append((d + 0.01 * k, x, y, rot))
    cand.sort()
    best, tried, blocks = None, 0, None
    for _d, x, y, rot in cand[:LOOKS]:
        if (x, y, rot) == was or not legal_at(ref, x, y, rot):
            continue
        st.parts[ref].ensure_rotation(rot)
        st.apply_move(ref, x, y, rot)
        model._limits_all()
        v = model._lval[i]
        best = v if best is None else min(best, v)
        now = _over(model)
        worse = [j for j in now if j != i and now[j] > base[j] + 1e-9]
        if v <= thr + EPS and not worse:
            return {'moved': (was, (round(x, 4), round(y, 4), rot)), 'best': best,
                    'blocks': None}
        if v <= thr + EPS and blocks is None:
            blocks = worse[0]
        st.parts[ref].ensure_rotation(was[2])
        st.apply_move(ref, *was)
        tried += 1
        if tried >= TRIES:
            break
    model._limits_all()
    return {'moved': None, 'best': best, 'blocks': blocks}


def repair(model, movable, legal_at: Callable[[str, float, float, float], bool],
           seats=frozenset(), rounds: int = 3) -> Dict:
    """Mend the limit terms the committed poses leave broken. `movable` are
    the refs that may move (a part seated on an edge never does);
    `legal_at(ref, x, y, rot)` is the full legality of a candidate pose, asked
    before the part is moved. Returns {'moved': [{'ref', 'from', 'to', 'mm', 'rail'}], 'before':
    N broken, 'after': N broken, 'seconds', 'stuck': [residual rows, each with 'reach':
    the nearest distance any legal pose got (None: none was legal) and
    'would_break': the other limit term a pose inside the limit would have put
    past its own (None: no such pose)]}.

    The model is repriced when it ends, so its totals read the mended poses."""
    t0 = time.time()
    movable = set(movable) - set(seats)
    before = broken_terms(model)
    moved, last_best, last_block = [], {}, {}
    for _ in range(rounds):
        bt = broken_terms(model)
        if not bt:
            break
        progress = False
        for i in sorted(bt, key=lambda j: -(model._lval[j] - model._lthr[j])):
            if model._lval[i] <= model._lthr[i] + EPS:
                continue            # an earlier move cleared it too
            for ref in _movers(model, i, movable)[:3]:
                got = _try_mover(model, i, ref, legal_at)
                if got['best'] is not None:
                    last_best[i] = min(last_best.get(i, got['best']), got['best'])
                if got['blocks'] is not None:
                    last_block.setdefault(i, got['blocks'])
                if got['moved']:
                    a, b = got['moved']
                    moved.append({'ref': ref, 'from': [round(v, 4) for v in a],
                                  'to': list(b), 'mm': round(math.hypot(b[0] - a[0], b[1] - a[1]), 3),
                                  'rail': _rows_of(model, [i])[0]['rail']})
                    progress = True
                    break
        if not progress:
            break
    after = broken_terms(model)
    stuck = []
    for r in _rows_of(model, after):
        r['reach'] = None if r['term'] not in last_best else round(last_best[r['term']], 3)
        j = last_block.get(r['term'])
        r['would_break'] = None
        if j is not None:
            o = _rows_of(model, [j])[0]
            r['would_break'] = {k: o[k] for k in ('kind', 'ic', 'pin', 'rail', 'cap')}
        stuck.append(r)
    model.reprice_all()
    model.total()
    return {'moved': moved, 'before': len(before), 'after': len(after), 'stuck': stuck,
            'seconds': round(time.time() - t0, 2)}
