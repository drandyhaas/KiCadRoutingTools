#!/usr/bin/env python3
"""#1361: a pair whose members run alongside, looser than the routed gap,
keeps its #521 protection.

#1303 graded a pair's shipped copper at the gap it was routed at and withdrew
#521 protection from a pair that measure called uncoupled. The #444 seam
re-ask (51ddb747) routes ulx3s GPDI straight from the AC caps to the HDMI
pins, P and N 0.8 mm apart -- the connector's pin pitch -- with 0.17-0.40 mm
of skew. Reported uncoupled, it went unprotected, and the chain's next steps
ripped GPDI_D1 and laid P on In2.Cu beside N on F.Cu.

Now `diff_pair_custody.measure_pair_copper` also measures, for a pair that is
not coupled, how far it runs alongside its partner (KiCad's coupling geometry
up to `ALONGSIDE_GAP_MM`), and `route_diff.coupled_protection_pairs` protects
a pair that still runs alongside when both members are complete. Reporting
does not move: the pair stays 'uncoupled'.

Fixture: a 2-layer board this test writes, one pair per case:
  T  tight      P/N 0.1 mm apart (the routed gap): coupled
  A  alongside  0.8 mm apart: uncoupled, alongside -> protected
  L  layers     P on F.Cu, N on B.Cu: split -> not protected
  F  far        parallel on F.Cu but 20 mm apart (separate connectors): the
                no-maximum-gap measure counts it; not alongside -> not protected
  I  open       as A, with an N pad left unconnected (#1107) -> not protected

    python3 tests/test_1361_loose_pair_protection.py
"""
import contextlib
import io
import os
import sys
import tempfile
import uuid
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                        # noqa: E402
from run_utils import evidence                                  # noqa: E402

failures = []
WIDTH, GAP = 0.1, 0.1


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


def fixture_text():
    n = [0]
    refs = [0]
    body = []

    def u():
        n[0] += 1
        return str(uuid.UUID(int=n[0]))

    def seg(x1, y1, x2, y2, net, layer):
        body.append(f'\t(segment\n\t\t(start {x1:.6f} {y1:.6f})\n\t\t(end {x2:.6f} {y2:.6f})\n'
                    f'\t\t(width {WIDTH})\n\t\t(layer "{layer}")\n\t\t(net "{net}")\n'
                    f'\t\t(uuid "{u()}")\n\t)\n')

    def pad(x, y, net, layer):
        refs[0] += 1
        body.append(f'\t(footprint "TP"\n\t\t(layer "{layer}")\n\t\t(uuid "{u()}")\n'
                    f'\t\t(at {x:.6f} {y:.6f})\n\t\t(property "Reference" "TP{refs[0]}"\n'
                    f'\t\t\t(at 0 -1 0)\n\t\t\t(layer "F.Fab")\n\t\t\t(uuid "{u()}")\n'
                    f'\t\t\t(effects (font (size 0.5 0.5) (thickness 0.1)))\n\t\t)\n'
                    f'\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 0.1 0.1)\n'
                    f'\t\t\t(layers "{layer}" "{layer[0]}.Mask")\n\t\t\t(net "{net}")\n'
                    f'\t\t\t(uuid "{u()}")\n\t\t)\n\t)\n')

    def pair(name, y_p, y_n, lp='F.Cu', ln='F.Cu'):
        for y, net, layer in ((y_p, f'{name}_P', lp), (y_n, f'{name}_N', ln)):
            seg(10, y, 20, y, net, layer)
            pad(10, y, net, layer)
            pad(20, y, net, layer)

    # centre spacing = width + edge gap
    pair('T', 10, 10 + WIDTH + GAP)
    pair('A', 15, 15 + WIDTH + 0.8)
    pair('L', 20, 20 + WIDTH + GAP, 'F.Cu', 'B.Cu')
    pair('F', 25, 45)
    pair('I', 50, 50 + WIDTH + 0.8)
    pad(30, 50 + WIDTH + 0.8, 'I_N', 'F.Cu')          # left open
    for (x1, y1, x2, y2) in ((5, 5, 60, 5), (60, 5, 60, 60), (60, 60, 5, 60), (5, 60, 5, 5)):
        body.append(f'\t(gr_line\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n'
                    f'\t\t(stroke (width 0.05) (type default))\n\t\t(layer "Edge.Cuts")\n'
                    f'\t\t(uuid "{u()}")\n\t)\n')
    return _HEAD + ''.join(body) + ')\n'


def main():
    import diff_pair_custody as dpc
    from diff_pair_custody import (measure_pair_copper, build_pair_reports,
                                   audit_pair_members)
    from diff_pair_routing import seam_coupling
    from route_diff import coupled_protection_pairs, protection_candidates

    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'pairs.kicad_pcb')
        with open(board, 'w') as fh:
            fh.write(fixture_text())
        pcb = quiet(parse_kicad_pcb, evidence(board, 'fixture board'))
    ids = {nt.name: i for i, nt in pcb.nets.items()}
    names = ('T', 'A', 'L', 'F', 'I')
    pairs = [(nm, SimpleNamespace(p_net_id=ids[f'{nm}_P'], n_net_id=ids[f'{nm}_N'],
                                  p_net_name=f'{nm}_P', n_net_name=f'{nm}_N'))
             for nm in names]
    results = {}
    for _nm, pr in pairs:
        res = {'is_diff_pair': True, 'new_segments': [object()]}
        results[pr.p_net_id] = results[pr.n_net_id] = res
    copper = quiet(measure_pair_copper, pcb, pairs, results, lambda _p: (WIDTH, GAP))
    audit = quiet(audit_pair_members, pcb, pairs)

    # Preconditions: the fixture must put every rescued-or-refused pair below
    # the routed-gap measure, or the rule below is never exercised.
    coupled = {k: v['coupled'] for k, v in copper.items()}
    check('precondition: only T couples at the routed gap',
          coupled == {'T': True, 'A': False, 'L': False, 'F': False, 'I': False},
          str(coupled))
    check('precondition: the member audit sees I_N left open, every other member '
          'complete', audit.get('I') == {'p': True, 'n': False}
          and all(audit[k] == {'p': True, 'n': True} for k in ('T', 'A', 'L', 'F')),
          str(audit))
    far = [s for s in pcb.segments if s.net_id in (ids['F_P'], ids['F_N'])]
    al, _ln = seam_coupling(far, ids['F_P'], ids['F_N'])
    check("precondition: the re-ask gate's no-maximum-gap measure counts F's "
          "members, 20 mm apart, as alongside", al > 9.9, f'{al:.2f} mm')

    along = {k: v.get('alongside') for k, v in copper.items() if not v['coupled']}
    check('alongside within 1 mm: A and I yes; split layers (L) and 20 mm apart '
          '(F) no', along == {'A': True, 'L': False, 'F': False, 'I': True}, str(along))
    check('the alongside record carries its length and window',
          abs(copper['A']['alongside_mm'] - 10.0) < 1e-6
          and copper['A']['alongside_window_mm'] == [None, dpc.ALONGSIDE_GAP_MM]
          and 'alongside_mm' not in copper['T'],
          str({k: copper['A'].get(k) for k in ('alongside_mm', 'alongside_window_mm')}))

    state = SimpleNamespace(routed_results=results, diff_pair_single_ended_nets={},
                            pair_diagnostics={})
    reps = quiet(build_pair_reports, state, pairs, audit, pair_copper=copper)
    out = {r['pair']: r['outcome'] for r in reps}
    check('reporting does not move: A is still uncoupled (KiCad measure at the '
          'routed gap), I incomplete',
          out == {'T': 'coupled', 'A': 'uncoupled', 'L': 'uncoupled',
                  'F': 'uncoupled', 'I': 'incomplete'}, str(out))

    kept = [nm for nm, _p in coupled_protection_pairs(pairs, copper, audit)]
    check('#521 keeps T (coupled) and A (alongside, complete); drops the split L, '
          'the far F and the open I', kept == ['T', 'A'], str(kept))
    prot = protection_candidates(results, pcb, pairs=coupled_protection_pairs(
        pairs, copper, audit))
    check('protection_candidates records both members of T and A, nothing else',
          sorted(prot) == ['A_N', 'A_P', 'T_N', 'T_P'], str(sorted(prot)))
    kept = [nm for nm, _p in coupled_protection_pairs(pairs, copper)]
    check('without the member audit no uncoupled pair is rescued', kept == ['T'],
          str(kept))

    # Negative control: the window decides. At 0.5 mm, A (0.8 mm apart) no
    # longer runs alongside and loses protection like any split pair.
    saved = dpc.ALONGSIDE_GAP_MM
    try:
        dpc.ALONGSIDE_GAP_MM = 0.5
        narrow = quiet(measure_pair_copper, pcb, pairs, results, lambda _p: (WIDTH, GAP))
    finally:
        dpc.ALONGSIDE_GAP_MM = saved
    kept = [nm for nm, _p in coupled_protection_pairs(pairs, narrow, audit)]
    check('negative control: a 0.5 mm window drops A', kept == ['T']
          and narrow['A']['alongside'] is False, str(kept))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
