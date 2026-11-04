#!/usr/bin/env python3
"""#1429: a pair coupled over a sliver of its length is NOT coupled.

#1303 called a pair's shipped copper coupled once its coupled run reached one
pair pitch (track width + gap, 0.2 mm here), whatever the pair's length: a
21.5 mm pair with 0.5 mm coupled read 'coupled', stayed in routed_diff_pairs
and kept its #521 protection. Now `diff_pair_custody.measure_pair_copper`
asks for the pitch AND `COUPLED_SHARE_FLOOR` (10%) of the pair's length --
the longer member's track length, the length KiCad's coupling test measures
against -- and discloses the share as `coupled_share`. The same two floors
decide `alongside` (#1361), so a sliver alongside is not protected either.

Fixture: a 2-layer board this test writes, one pair per case. Each long pair
is 21.5 mm (its N member; P is shorter): N runs beside P at the routed gap
for its coupled length, then turns 45 degrees away and finishes 2.1 mm from
P, outside both the coupled and the alongside windows.

  X  sliver     0.5 mm coupled (2%)  -> uncoupled, not alongside, unprotected
  B  below      9% coupled           -> uncoupled
  A  above      11% coupled          -> coupled
  Y  thirty     30% coupled          -> coupled, protected
  T  tight      fully coupled        -> coupled
  Z  short      0.15 mm long, fully coupled: 100% but under the pitch
                                     -> uncoupled

Plus, end to end, a route_diff run on an in-repo board: its JSON_SUMMARY's
pair_reports[].copper carries coupled_share and the share floor.

    python3 tests/test_1429_coupled_share_floor.py
"""
import contextlib
import io
import json
import math
import os
import subprocess
import sys
import tempfile
import uuid
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                        # noqa: E402
from run_utils import check as check_run, evidence, tool        # noqa: E402

failures = []
WIDTH, GAP = 0.1, 0.1
PITCH = WIDTH + GAP
LONG = 21.5
DIAG = 2.0                     # the 45-degree turn-away, in x and in y


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        return fn(*a, **k)


_HEAD = ('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
         '\t(generator_version "10.0")\n\t(general\n\t\t(thickness 1.6)\n'
         '\t\t(legacy_teardrops no)\n\t)\n\t(paper "A4")\n\t(layers\n'
         '\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n\t\t(1 "F.Mask" user)\n'
         '\t\t(3 "B.Mask" user)\n\t\t(25 "Edge.Cuts" user)\n\t)\n'
         '\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n')

# name -> coupled length (mm) of a LONG pair
LONG_PAIRS = {'X': 0.5, 'B': 0.09 * LONG, 'A': 0.11 * LONG, 'Y': 0.30 * LONG}


def fixture_text():
    n = [0]
    refs = [0]
    body = []

    def u():
        n[0] += 1
        return str(uuid.UUID(int=n[0]))

    def seg(x1, y1, x2, y2, net):
        body.append(f'\t(segment\n\t\t(start {x1:.6f} {y1:.6f})\n\t\t(end {x2:.6f} {y2:.6f})\n'
                    f'\t\t(width {WIDTH})\n\t\t(layer "F.Cu")\n\t\t(net "{net}")\n'
                    f'\t\t(uuid "{u()}")\n\t)\n')

    def pad(x, y, net):
        refs[0] += 1
        body.append(f'\t(footprint "TP"\n\t\t(layer "F.Cu")\n\t\t(uuid "{u()}")\n'
                    f'\t\t(at {x:.6f} {y:.6f})\n\t\t(property "Reference" "TP{refs[0]}"\n'
                    f'\t\t\t(at 0 -1 0)\n\t\t\t(layer "F.Fab")\n\t\t\t(uuid "{u()}")\n'
                    f'\t\t\t(effects (font (size 0.5 0.5) (thickness 0.1)))\n\t\t)\n'
                    f'\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 0.1 0.1)\n'
                    f'\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(net "{net}")\n'
                    f'\t\t\t(uuid "{u()}")\n\t\t)\n\t)\n')

    def long_pair(name, y, c):
        # N: c mm beside P at the routed gap, a 45-degree turn away, then
        # straight on until N is LONG mm in all; P runs to N's end x.
        x0, yn = 10.0, y + PITCH
        tail = LONG - c - DIAG * math.sqrt(2)
        xa, xb = x0 + c, x0 + c + DIAG
        xe = xb + tail
        seg(x0, yn, xa, yn, f'{name}_N')
        seg(xa, yn, xb, yn + DIAG, f'{name}_N')
        seg(xb, yn + DIAG, xe, yn + DIAG, f'{name}_N')
        pad(x0, yn, f'{name}_N')
        pad(xe, yn + DIAG, f'{name}_N')
        seg(x0, y, xe, y, f'{name}_P')
        pad(x0, y, f'{name}_P')
        pad(xe, y, f'{name}_P')

    def straight_pair(name, y, length):
        for yy, net in ((y, f'{name}_P'), (y + PITCH, f'{name}_N')):
            seg(10, yy, 10 + length, yy, net)
            pad(10, yy, net)
            pad(10 + length, yy, net)

    for i, (nm, c) in enumerate(LONG_PAIRS.items()):
        long_pair(nm, 10 + 5 * i, c)
    straight_pair('T', 35, 10.0)
    straight_pair('Z', 40, 0.15)
    for (x1, y1, x2, y2) in ((5, 5, 40, 5), (40, 5, 40, 45), (40, 45, 5, 45), (5, 45, 5, 5)):
        body.append(f'\t(gr_line\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n'
                    f'\t\t(stroke (width 0.05) (type default))\n\t\t(layer "Edge.Cuts")\n'
                    f'\t\t(uuid "{u()}")\n\t)\n')
    return _HEAD + ''.join(body) + ')\n'


def unit():
    import diff_pair_custody as dpc
    from diff_pair_custody import (measure_pair_copper, build_pair_reports,
                                   audit_pair_members)
    from route_diff import coupled_protection_pairs

    check('the share floor is one named constant at 10%',
          getattr(dpc, 'COUPLED_SHARE_FLOOR', None) == 0.10,
          str(getattr(dpc, 'COUPLED_SHARE_FLOOR', None)))

    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'pairs.kicad_pcb')
        with open(board, 'w') as fh:
            fh.write(fixture_text())
        pcb = quiet(parse_kicad_pcb, evidence(board, 'fixture board'))
    ids = {nt.name: i for i, nt in pcb.nets.items()}
    names = list(LONG_PAIRS) + ['T', 'Z']
    pairs = [(nm, SimpleNamespace(p_net_id=ids[f'{nm}_P'], n_net_id=ids[f'{nm}_N'],
                                  p_net_name=f'{nm}_P', n_net_name=f'{nm}_N'))
             for nm in names]
    results = {}
    for _nm, pr in pairs:
        res = {'is_diff_pair': True, 'new_segments': [object()]}
        results[pr.p_net_id] = results[pr.n_net_id] = res
    copper = quiet(measure_pair_copper, pcb, pairs, results, lambda _p: (WIDTH, GAP))
    audit = quiet(audit_pair_members, pcb, pairs)

    # Preconditions: the fixture measures as designed, or the rule is untested.
    meas = {k: (round(v['coupled_mm'], 3),
                round(max(v['length_p_mm'], v['length_n_mm']), 3))
            for k, v in copper.items()}
    want = {nm: (round(c, 3), LONG) for nm, c in LONG_PAIRS.items()}
    want.update(T=(10.0, 10.0), Z=(0.15, 0.15))
    check('precondition: coupled run and pair length as built',
          all(abs(meas[k][0] - want[k][0]) < 2e-3 and abs(meas[k][1] - want[k][1]) < 2e-3
              for k in want), str(meas))
    check('precondition: every member is complete (protection is the copper\'s '
          'call, not the audit\'s)',
          all(audit[k] == {'p': True, 'n': True} for k in names), str(audit))
    check('precondition: the sliver passes the old one-pitch floor',
          copper['X']['coupled_mm'] >= copper['X']['coupled_floor_mm'],
          f"{copper['X']['coupled_mm']} vs {copper['X']['coupled_floor_mm']}")

    coupled = {k: v['coupled'] for k, v in copper.items()}
    check('coupled needs one pitch AND 10% of the length: the sliver (2%), 9% and '
          'the 0.15 mm pair do not couple; 11%, 30% and the full run do',
          coupled == {'X': False, 'B': False, 'A': True, 'Y': True, 'T': True,
                      'Z': False}, str(coupled))
    share = {k: v.get('coupled_share') for k, v in copper.items()}
    check('every record discloses coupled_share = coupled / pair length',
          all(isinstance(s, float) for s in share.values())
          and abs(share['X'] - 0.5 / LONG) < 1e-3 and abs(share['Y'] - 0.30) < 1e-3
          and share['T'] == 1.0 and share['Z'] == 1.0
          and all(v.get('coupled_share_floor') == 0.10 for v in copper.values()),
          str(share))
    along = {k: v.get('alongside') for k, v in copper.items() if not v['coupled']}
    check('alongside obeys the same two floors: the sliver runs alongside for '
          '0.5 mm and is not alongside', along == {'X': False, 'B': False, 'Z': False}
          and abs(copper['X']['alongside_mm'] - 0.5) < 2e-3
          and 'alongside_share' in copper['X'],
          str({k: (copper[k].get('alongside_mm'), along[k]) for k in along}))

    state = SimpleNamespace(routed_results=results, diff_pair_single_ended_nets={},
                            pair_diagnostics={})
    reps = quiet(build_pair_reports, state, pairs, audit, pair_copper=copper)
    out = {r['pair']: r['outcome'] for r in reps}
    check("a pair coupled over a sliver of its length is 'uncoupled'",
          out == {'X': 'uncoupled', 'B': 'uncoupled', 'A': 'coupled', 'Y': 'coupled',
                  'T': 'coupled', 'Z': 'uncoupled'}, str(out))
    check('its report carries the share',
          next(r for r in reps if r['pair'] == 'X')['copper'].get('coupled_share')
          == share['X'])
    kept = [nm for nm, _p in coupled_protection_pairs(pairs, copper, audit)]
    check('#521 protects only the pairs coupled over enough of their length',
          kept == ['A', 'Y', 'T'], str(kept))

    # Negative control: the share floor is what decides. At 0 the old
    # one-pitch rule returns and the sliver reads coupled again.
    saved = dpc.COUPLED_SHARE_FLOOR
    try:
        dpc.COUPLED_SHARE_FLOOR = 0.0
        old = quiet(measure_pair_copper, pcb, pairs, results, lambda _p: (WIDTH, GAP))
    finally:
        dpc.COUPLED_SHARE_FLOOR = saved
    check('negative control: with no share floor the sliver and 9% read coupled, '
          'the sub-pitch pair still does not',
          old['X']['coupled'] and old['B']['coupled'] and not old['Z']['coupled'],
          str({k: v['coupled'] for k, v in old.items()}))


BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
OPTS = ['--nets', '*lvds_rx1_1*', '--impedance', '100',
        '--proximity-heuristic-factor', '0.0', '--clearance', '0.1',
        '--via-size', '0.3', '--via-drill', '0.2', '--track-width', '0.1',
        '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'B.Cu']


def end_to_end():
    evidence(BOARD, 'input board')
    with tempfile.TemporaryDirectory(prefix='t1429_') as tmp:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        # copy_board carries the sibling .kicad_pro (#441).
        subprocess.run([sys.executable, tool('copy_board.py'), BOARD, inp],
                       check=True, capture_output=True)
        out = os.path.join(tmp, 'out.kicad_pcb')
        js = os.path.join(tmp, 'route.json')
        check_run([sys.executable, '-X', 'utf8', tool('route_diff.py'), inp, out,
                   *OPTS, '--json-out', js], accept=True)
        doc = json.load(open(evidence(js, '--json-out file')))
    reps = [r for r in doc.get('pair_reports') or [] if 'copper' in r]
    check('end to end: the run reports at least one measured pair', bool(reps),
          str(doc.get('pair_reports')))
    for r in reps:
        c = r['copper']
        length = max(c['length_p_mm'], c['length_n_mm'])
        ok = (c.get('coupled_share_floor') == 0.10
              and isinstance(c.get('coupled_share'), float)
              and abs(c['coupled_share'] - min(1.0, c['coupled_mm'] / length)) < 1e-3
              and c['coupled'] == (c['coupled_mm'] >= c['coupled_floor_mm'] - 1e-9
                                   and c['coupled_share'] >= 0.10 - 1e-3))
        check(f"end to end: {r['pair']}'s copper record carries coupled_share "
              f"and its verdict follows both floors", ok,
              str({k: c.get(k) for k in ('coupled_mm', 'coupled_share',
                                         'coupled_floor_mm', 'coupled')}))


def main():
    unit()
    if '--unit' not in sys.argv:
        end_to_end()
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
