#!/usr/bin/env python3
"""The joint escape plans a differential pair as a pair (joint_escape.plan_array).

  python3 tests/test_622_joint_pairs.py

On two generated buses (synth_bus, shuffled: 12 nets with 4 pairs drawn two columns deep, and 24 nets with 10 pairs
drawn four deep -- with vias, where a leg planned on its own takes B.Cu or another face), the source array's joint
escape, every net a bus net:

1. every ball is planned an escape, and the report counts the pairs and their escapes;
2. each pair's two legs leave by the same face on the same layer, their exits neighbours (within 1.3 ball pitches),
   no other planned exit of that face and layer between them -- planned leg by leg, the first bus had two of its
   three pairs split (one by another net's escape, one by a whole pair's), the second one of its four split and the
   other three with their legs on two faces or two layers;
3. laid, each pair's two teeth stand side by side the same way on the board;
5. a round planning again only balls that are not there (no option among them) gets an empty plan, not a crash.
"""
import contextlib
import io
import math
import os
import subprocess
import sys
import tempfile

try:
    import ortools  # noqa: F401  (the joint escape plans with CP-SAT)
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
sys.path.insert(0, AWX)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
with contextlib.redirect_stdout(io.StringIO()):
    from kicad_parser import parse_kicad_pcb  # noqa: E402
    import escape_moves as em  # noqa: E402
    import joint_escape as je  # noqa: E402
    import pairs as _pairs  # noqa: E402

BAD = []
CASES = (('k12', ['--k', '12', '--pairs', '4', '--depth', '2', '--pattern', 'shuffle', '--seed', '1'], 3),
         ('k24', ['--k', '24', '--pairs', '10', '--depth', '4', '--pattern', 'shuffle', '--seed', '2'], 4))


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


def teeth(board, ref):
    """{net: (face, layer, place along the face)} of each net's stub tip farthest from its ball on `ref`, its face the
    side of the ball box it lies farthest beyond"""
    with contextlib.redirect_stdout(io.StringIO()):
        p = parse_kicad_pcb(board)
    xs = [q.global_x for q in p.footprints[ref].pads]
    ys = [q.global_y for q in p.footprints[ref].pads]
    b = (min(xs), min(ys), max(xs), max(ys))
    out = {}
    for nid, net in p.nets.items():
        pads = [q for q in net.pads if q.component_ref == ref]
        segs = [s for s in p.segments if s.net_id == nid]
        if not pads or not segs:
            continue
        pad = pads[0]
        ends = [(s.start_x, s.start_y, s.layer) for s in segs] + [(s.end_x, s.end_y, s.layer) for s in segs]
        x, y, L = max(ends, key=lambda q: math.hypot(q[0] - pad.global_x, q[1] - pad.global_y))
        d = {'W': b[0] - x, 'E': x - b[2], 'N': b[1] - y, 'S': y - b[3]}
        face = max(d, key=d.get)
        out[net.name.split('/')[-1]] = (face, L, y if face in 'EW' else x)
    return out


def case(tag, args, n_pairs, tmp):
    raw = os.path.join(tmp, f'{tag}.kicad_pcb')
    r = subprocess.run([sys.executable, 'synth_bus.py', raw] + args, cwd=AWX, capture_output=True, text=True)
    if r.returncode or not os.path.isfile(raw):
        print(r.stdout[-2000:], r.stderr[-2000:])
        raise SystemExit(f'BROKEN TEST: synth_bus wrote no board for {tag}')
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(raw)
    names = sorted({q.net_name for q in pcb.footprints['SU1'].pads if q.net_id and q.net_name})
    prs = _pairs.pair_names([n.split('/')[-1] for n in names], admit_all=True)
    if len(prs) != n_pairs:
        raise SystemExit(f'BROKEN TEST: {tag} carries {len(prs)} pairs, not the {n_pairs} it is built with')
    with contextlib.redirect_stdout(io.StringIO()):
        far = je.far_face(pcb, 'SU1', 'SD1')
        hints, rep = je.plan_array(pcb, 'SU1', names, [], je.signal_layers(pcb), far=far, debug=True,
                                   log=lambda *a: None)
    chosen = {k.split('#')[0]: o for k, (kind, o) in rep['debug']['chosen'].items() if kind == 'escape'}

    # 1. every ball planned, the pairs counted
    check(rep['bus_escaped'] == rep['bus_balls'] == len(names),
          f'{tag}: every ball planned an escape ({rep["bus_escaped"]} of {rep["bus_balls"]})')
    check(rep.get('pairs') == len(prs) and rep.get('pairs_escaped') == len(prs),
          f'{tag}: the report counts the pairs and their escapes ({rep.get("pairs")}, {rep.get("pairs_escaped")} of '
          f'{len(prs)})')

    # 2. each pair planned as a pair
    gr = em.grid_of(pcb.footprints['SU1'])
    reach = 1.3 * max(gr.pitch_x, gr.pitch_y)
    for b_, (pn, nn) in sorted(prs.items()):
        a, c = chosen.get(pn), chosen.get(nn)
        if a is None or c is None:
            check(False, f'{tag} {b_}: both legs planned')
            continue
        same = (a.direction, a.layer) == (c.direction, c.layer)
        d = math.hypot(a.exit_pt[0] - c.exit_pt[0], a.exit_pt[1] - c.exit_pt[1])
        ax = 0 if a.direction in ('up', 'down') else 1
        lo, hi = sorted((a.exit_pt[ax], c.exit_pt[ax]))
        between = sorted(n for n, o in chosen.items() if n not in (pn, nn)
                         and (o.direction, o.layer) == (a.direction, a.layer) and lo + 0.02 < o.exit_pt[ax] < hi - 0.02)
        check(same and 0.05 < d <= reach and not between,
              f'{tag} {b_}: planned out of one face on one layer ({a.direction}/{a.layer} and {c.direction}/'
              f'{c.layer}), exits {d:.2f} mm apart (a neighbour within {reach:.2f}), nothing between ({between})')

    # 4. each pair asked the other HAND (`hands`, the other array's): planned with it, still a pair -- or named as having
    # no exit pair of it
    h0 = {pn: _pairs.hand(chosen[pn].direction, chosen[pn].exit_pt, chosen[nn].exit_pt)
          for _b, (pn, nn) in prs.items() if pn in chosen and nn in chosen}
    with contextlib.redirect_stdout(io.StringIO()):
        _h2, rep2 = je.plan_array(pcb, 'SU1', names, [], je.signal_layers(pcb), far=far, debug=True,
                                  log=lambda *a: None, hands={pn: (-h, False) for pn, h in h0.items() if h})
    ch2 = {k.split('#')[0]: o for k, (kind, o) in rep2['debug']['chosen'].items() if kind == 'escape'}
    check(sorted(rep2['hands_held'] + rep2['hands_free']) == sorted(pn for pn, h in h0.items() if h),
          f'{tag}: every pair asked a hand is reported held or free ({rep2["hands_held"]} + {rep2["hands_free"]})')
    turned = 0
    for b_, (pn, nn) in sorted(prs.items()):
        a, c = ch2.get(pn), ch2.get(nn)
        if not h0.get(pn) or pn in rep2['hands_free']:
            continue
        if a is None or c is None:
            check(False, f'{tag} {b_}: both legs planned when asked the other hand')
            continue
        h2 = _pairs.hand(a.direction, a.exit_pt, c.exit_pt)
        d = math.hypot(a.exit_pt[0] - c.exit_pt[0], a.exit_pt[1] - c.exit_pt[1])
        turned += h2 == -h0[pn]
        check(h2 == -h0[pn] and (a.direction, a.layer) == (c.direction, c.layer) and 0.05 < d <= reach,
              f'{tag} {b_}: asked hand {-h0[pn]:+d} (free {h0[pn]:+d}), planned {h2:+d} out of {a.direction}/'
              f'{a.layer} and {c.direction}/{c.layer}, exits {d:.2f} mm apart')
    check(turned > 0, f'{tag}: the hands asked turned {turned} pair(s) (none: the check above checked nothing)')

    # 5. a round planning again balls with no option among them -- here none of the array's -- asks phase 1 nothing:
    # an empty plan, not a crash (zynq U2's third round, its four plane balls held round the passives with no drop
    # left, raised CP-SAT's "solve() has not been called" from the report, and the round's fanout exited 1 four times)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            h3, rep3 = je.plan_array(pcb, 'SU1', names, [], je.signal_layers(pcb), far=far, log=lambda *a: None,
                                     only={'NO_SUCH_NET#Z99'})
        check(not h3 and rep3['unplanned'] == [] and rep3['phase1'] == 'nothing to ask',
              f'{tag}: nothing to plan again -> an empty plan ({len(h3)} hints, phase 1 {rep3["phase1"]})')
    except Exception as e:      # noqa: BLE001
        check(False, f'{tag}: nothing to plan again raised {type(e).__name__}: {e}')

    # 3. laid so
    out = os.path.join(tmp, f'{tag}.laid.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        je.lay(raw, out, 'SU1', names, [], je.signal_layers(pcb), hints)
    T = teeth(out, 'SU1')
    for b_, (pn, nn) in sorted(prs.items()):
        a, c = T.get(pn), T.get(nn)
        if a is None or c is None:
            check(False, f'{tag} {b_}: both legs laid')
            continue
        lo, hi = sorted((a[2], c[2]))
        between = sorted(n for n, t in T.items() if n not in (pn, nn) and t[:2] == a[:2] and lo < t[2] < hi)
        check(a[:2] == c[:2] and not between,
              f'{tag} {b_}: laid side by side ({a[:2]} and {c[:2]}), nothing between ({between})')


with tempfile.TemporaryDirectory() as tmp_:
    for tag_, args_, n_ in CASES:
        case(tag_, args_, n_, tmp_)

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)
