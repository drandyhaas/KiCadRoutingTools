#!/usr/bin/env python3
"""#619 corpus sweep: what the under-pad escape emits into copper the obstacle
map erased, and what closing that costs.

    python3 tests/sweep_619_erased_copper.py off > off.txt
    python3 tests/sweep_619_erased_copper.py all > all.txt
    python3 tests/sweep_619_erased_copper.py --diff off.txt all.txt

NOT named `test_*`, so `run_all.py` never collects it: it drives the whole
tracked board corpus through the fanout engine and takes minutes, not seconds.
It exists in the repo rather than in a scratch directory so the table in the PR
is re-runnable from a clean clone -- the sibling failing of PR #645, whose own
"what I could not verify" section says its 408-footprint table was not.

ONE ARM PER PROCESS, deliberately. `env_knobs` reads the environment once at
import; an in-process arm switch would depend on `refresh()` having reached
every consumer, and a stale read would silently grade one arm as the other.

SELECTION IS BY PAD COUNT, NOT BY PACKAGE NAME. `qfn_fanout.py -c` accepts any
reference and the GUI dropdown lists every footprint with enough pads, so a
QFN/QFP name filter measures a different population than the tool serves --
that is how PR #645's first revision reported 2 changed footprints where the
truth was 7. Under-pad via-drop is the fine-pitch/CSP escape method; a WLCSP-20
is its core case, not an out-of-scope one.

The three residual counters are the point. Each counts EMITTED stubs that sit
inside the clearance floor of PRE-EXISTING copper on a DIFFERENT fanned net --
i.e. copper `nets_to_route` erased from the obstacle map. They are computed
from the engine's returned geometry, graded with `check_drc`'s own predicates
where one exists, so the sweep and the shipped grader cannot drift apart.
"""
import argparse
import contextlib
import io
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522

TRACK_W, CLEARANCE, VIA_SIZE, VIA_DRILL, GRID = 0.1, 0.1, 0.45, 0.25, 0.05
MIN_PADS = 4


def _counters(footprint, pcb, tracks):
    """(via, seg, pad) emitted-stub contacts with erased copper.

    `fanned_nets` is rebuilt the way the ENGINE builds it -- from the pads it
    actually escapes -- rather than from `footprint.pads`. The two differ
    (pad_infos drops net 0, `unconnected-*`, net-filter misses and `center`
    pads), and the engine's set is the one handed to `nets_to_route`, so it is
    the one that defines what got erased.
    """
    from check_drc import check_via_segment_overlap, check_pad_segment_overlap
    from geometry_utils import segment_to_segment_distance
    from kicad_parser import Segment
    import qfn_fanout

    # The set the ENGINE erased, published by the run that just happened --
    # never re-derived from footprint.pads (see LAST_ERASED_SETS' comment).
    fanned = qfn_fanout.LAST_ERASED_SETS.get('nets') or set()
    ev = [v for v in pcb.vias if v.net_id in fanned]
    es = [s for s in pcb.segments if s.net_id in fanned]
    ep = [p for plist in pcb.pads_by_net.values() for p in plist
          if p.net_id in fanned]
    layers = list(pcb.board_info.copper_layers or [])

    n_via = n_seg = n_pad = 0
    for t in tracks:
        seg = Segment(start_x=t['start'][0], start_y=t['start'][1],
                      end_x=t['end'][0], end_y=t['end'][1],
                      width=t['width'], layer=t['layer'], net_id=t['net_id'])
        for v in ev:
            if v.net_id == t['net_id']:
                continue
            if check_via_segment_overlap(v, seg, CLEARANCE)[0]:
                n_via += 1
        for s in es:
            if s.net_id == t['net_id'] or s.layer != t['layer']:
                continue
            if segment_to_segment_distance(
                    seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                    s.start_x, s.start_y, s.end_x, s.end_y) \
                    < s.width / 2 + seg.width / 2 + CLEARANCE - 1e-6:
                n_seg += 1
        for p in ep:
            if p.net_id == t['net_id']:
                continue
            if check_pad_segment_overlap(p, seg, CLEARANCE, layers)[0]:
                n_pad += 1
    return n_via, n_seg, n_pad, len(ev), len(es), len(ep)


def sweep(arm, allow_vip=False, boards=None):
    from run_utils import corpus_boards
    from kicad_parser import parse_kicad_pcb
    from qfn_fanout import generate_qfn_fanout

    rows = []
    paths = boards or corpus_boards()
    if not paths:
        print("SKIP: git could not list the tracked board corpus")
        return None
    for bp in paths:
        name = os.path.splitext(os.path.basename(bp))[0]
        try:
            pcb = parse_kicad_pcb(os.path.join(ROOT, bp))
        except Exception as e:                       # a board we cannot parse
            print(f"  !! {name}: parse failed ({type(e).__name__})")
            continue
        for ref, fp in sorted(pcb.footprints.items()):
            if len(fp.pads) < MIN_PADS:
                continue
            try:
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    tracks, vias, dropped = generate_qfn_fanout(
                        fp, pcb, layer=fp.layer, track_width=TRACK_W,
                        clearance=CLEARANCE, grid_step=GRID,
                        escape_method="underpad", via_size=VIA_SIZE,
                        via_drill=VIA_DRILL, allow_via_in_pad=allow_vip)
            except Exception as e:
                print(f"  !! {name} {ref}: {type(e).__name__}: {e}")
                continue
            nv, ns, npd, ev, es, ep = _counters(fp, pcb, tracks)
            rows.append({
                'board': name, 'ref': ref, 'pads': len(fp.pads),
                'tracks': len(tracks), 'vias': len(vias),
                'dropped': sorted(dropped),
                'erased': [ev, es, ep], 'pairs': [nv, ns, npd],
                # exact emitted geometry, so two arms can be compared for
                # byte-identity rather than merely for equal counts
                'geom': [[list(t['start']), list(t['end']), t['layer'],
                          t['net_id']] for t in tracks]
                        + [[v['x'], v['y'], v['net_id']] for v in vias],
            })
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('arm', nargs='?', default='all',
                    help="KICAD_QFN_UNDERPAD_ERASED_GATE value for this run")
    ap.add_argument('--allow-via-in-pad', action='store_true')
    ap.add_argument('--board', action='append',
                    help="limit to these board paths (repeatable)")
    ap.add_argument('--diff', nargs=2, metavar=('A.json', 'B.json'))
    a = ap.parse_args()

    if a.diff:
        A = {(r['board'], r['ref']): r for r in json.load(open(a.diff[0]))}
        B = {(r['board'], r['ref']): r for r in json.load(open(a.diff[1]))}
        same = [k for k in A if k in B and A[k]['geom'] == B[k]['geom']]
        chg = sorted(k for k in A if k in B and A[k]['geom'] != B[k]['geom'])
        live = [k for k in A if any(A[k]['erased'])]
        print(f"footprints compared          : {len(set(A) & set(B))}")
        print(f"gate-live (erased set non-0) : {len(live)}")
        print(f"byte-identical emitted geom  : {len(same)}")
        print(f"CHANGED                      : {len(chg)}")
        tot = [0, 0, 0, 0, 0, 0]
        print(f"\n{'board / part':42} {'erased v/s/p':>16} "
              f"{'A t/v/d':>12} {'B t/v/d':>12} {'A pairs':>10} {'B pairs':>10}")
        for k in chg:
            x, y = A[k], B[k]
            xa = f"{x['tracks']}/{x['vias']}/{len(x['dropped'])}"
            ya = f"{y['tracks']}/{y['vias']}/{len(y['dropped'])}"
            print(f"{k[0] + ' ' + k[1]:42} "
                  f"{'/'.join(map(str, x['erased'])):>16} "
                  f"{xa:>12} {ya:>12} "
                  f"{'/'.join(map(str, x['pairs'])):>10} "
                  f"{'/'.join(map(str, y['pairs'])):>10}")
        for k in set(A) & set(B):
            for i in range(3):
                tot[i] += A[k]['pairs'][i]
                tot[3 + i] += B[k]['pairs'][i]
        print(f"\nresidual pairs via/seg/pad   A: {tot[0]}/{tot[1]}/{tot[2]}"
              f"   B: {tot[3]}/{tot[4]}/{tot[5]}")
        wa = sum(len(A[k]['dropped']) for k in set(A) & set(B))
        wb = sum(len(B[k]['dropped']) for k in set(A) & set(B))
        print(f"escapes dropped              A: {wa}   B: {wb}"
              f"   (withdrawn by B: {wb - wa})")
        return 0

    os.environ['KICAD_QFN_UNDERPAD_ERASED_GATE'] = a.arm
    import env_knobs
    env_knobs.refresh()
    rows = sweep(a.arm, a.allow_via_in_pad, a.board)
    if rows is None:
        return 77
    out = f"sweep619_{a.arm}{'_vip' if a.allow_via_in_pad else ''}.json"
    with open(out, 'w') as f:
        json.dump(rows, f)
    tv = sum(r['pairs'][0] for r in rows)
    ts = sum(r['pairs'][1] for r in rows)
    tp = sum(r['pairs'][2] for r in rows)
    live = sum(1 for r in rows if any(r['erased']))
    print(f"\narm={a.arm} vip={a.allow_via_in_pad}: {len(rows)} footprints, "
          f"{live} gate-live")
    print(f"residual stub-vs-erased pairs  via={tv}  seg={ts}  pad={tp}")
    print(f"escapes dropped: {sum(len(r['dropped']) for r in rows)}")
    print(f"wrote {out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
