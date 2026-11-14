#!/usr/bin/env python3
"""#1869: a differential pair matched, recorded and graded on its lane.

`route_diff.py --diff-pair-intra-match` measures each member as KiCad's skew
DRC does, all the copper of the net. A pair with a branch to a third pad
cannot be matched in both senses: matched pad to pad, its total copper is
apart by the branches and check_drc fails it against the tolerance the pair
step recorded; matched in total copper, its signal paths are apart.

`--diff-pair-intra-lane auto | P_NET=REF.PAD:REF.PAD ...` matches the path
between two pads (`intra_pair_lane`, on `net_queries.pin_pair_pads`), records
the pads with the tolerance, and check_drc grades that recorded tolerance on
the path between them, reporting each member's branch as a stub. Without the
option nothing changes, and a `.kicad_dru` skew rule is always graded on
total copper, KiCad's measure.

The board is the bundled lvds_converter_dualclk: `/DATA+` and `/DATA-` run
J1 -> IC4, each with a third pad on R4. The chain is the pair step
(`--diff-pair-intra-match --length-match-tolerance 0.15`), then `route.py`,
which joins the branches to R4.

Rows:
  1. control, no lane: the record names no pads, and check_drc fails the
     pair on total copper, `/DATA-` 2.2846 mm shorter than `/DATA+`;
  2. `auto`: the record names IC4.10 / J1.5 and IC4.9 / J1.4; check_drc
     grades the lane at 0.105 mm, inside the 0.15 mm recorded, with no skew
     violation, and reports the two branches as stubs (about 5.45 and
     3.27 mm); board_score's pair record carries the lane and its skew;
  3. a `.kicad_dru` skew rule over the same two nets is graded on total
     copper, lane or no lane, as KiCad grades it;
  4. a named lane records the pads named, the N member's on the same parts;
  5. a named lane whose pads are not on the pair's nets, and one for a net no
     pair of the step has, refuse with the reason and exit 2;
  6. a recorded manifest carrying the flag converts to a plan step with it.
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence, tool  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
PAIR = ['--nets', '/DATA+', '/DATA-', '--diff-pair-intra-match',
        '--length-match-tolerance', '0.15', '--force-reroute']
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def record(board):
    with open(evidence(board[:-len('.kicad_pcb')] + '.kicad_pro', 'the project'),
              encoding='utf-8') as fh:
        return (json.load(fh).get('kicad_routing_tools') or {}).get(
            'intra_pair_tolerance') or {}


def chain(td, tag, lane):
    """The pair step (with `lane` when given), then route.py. Returns the
    shipped board."""
    s1, s2 = (os.path.join(td, f'{tag}_{n}.kicad_pcb') for n in ('s1', 's2'))
    check(PY + [tool('route_diff.py'), os.path.join(td, 'in.kicad_pcb'), s1] + PAIR
          + (['--diff-pair-intra-lane'] + lane if lane else []), accept=True, timeout=1800)
    check(PY + [tool('route.py'), evidence(s1, 'the pair step board'), s2,
                '--nets', '/DATA+', '/DATA-', '--clearance-ceiling', '0.1'],
          accept=True, timeout=1800)
    return evidence(s2, 'the shipped board')


def drc(board, flagged=None):
    argv = PY + [tool('check_drc.py'), board]
    r = (check(argv, refuse=flagged, code=1) if flagged else
         __import__('subprocess').run(argv, capture_output=True, text=True,
                                      encoding='utf-8', errors='replace', cwd=REPO))
    out = (r.stdout or '') + (r.stderr or '')
    if 'Traceback' in out:
        raise AssertionError('BROKEN TEST: check_drc crashed\n' + out[-1200:])
    return out


def main():
    with tempfile.TemporaryDirectory() as td:
        check(PY + [tool('copy_board.py'), evidence(BOARD, 'the bundled board'),
                    os.path.join(td, 'in.kicad_pcb')], accept=True)

        print("1. control: no lane")
        b = chain(td, 'total', None)
        rec = record(b).get('/DATA+') or {}
        ok(rec.get('tolerance_mm') == 0.15 and 'lane' not in rec,
           f"the record names no pads: {rec}")
        out = drc(b, flagged='2.2846mm shorter than /DATA+')
        ok('SKEW-OUT-OF-RANGE violations (1)' in out and 'Recorded lane' not in out,
           "check_drc fails the pair on total copper, 2.2846 mm")

        print("2. --diff-pair-intra-lane auto")
        b = chain(td, 'auto', ['auto'])
        rec = record(b).get('/DATA+') or {}
        lane = rec.get('lane') or {}
        ok(sorted(lane.get('p') or []) == ['IC4.10', 'J1.5']
           and sorted(lane.get('n') or []) == ['IC4.9', 'J1.4'],
           f"the record names the lane's pads: {lane}")
        out = drc(b)
        line = next((l for l in out.splitlines() if 'Recorded lane' in l), '')
        ok('SKEW-OUT-OF-RANGE' not in out and 'skew 0.1050 mm on the lane' in line,
           f"check_drc grades the lane, 0.105 mm, with no skew violation: {line.strip()[:120]}")
        js = os.path.join(td, 'auto.json')
        check(PY + [tool('check_drc.py'), b, '--json', js], accept=True)
        with open(evidence(js, 'check_drc --json'), encoding='utf-8') as fh:
            doc = json.load(fh)

        lg = (doc.get('recorded_lanes') or [{}])[0]
        stubs = lg.get('stub_mm') or {}
        ok(abs((lg.get('skew_mm') or 0) - 0.105) < 0.001
           and abs((stubs.get('/DATA+') or 0) - 5.45) < 0.02
           and abs((stubs.get('/DATA-') or 0) - 3.27) < 0.02,
           f"--json carries the lane skew and the branches as stubs: {lg.get('skew_mm')}, {stubs}")
        sj = os.path.join(td, 'score.json')
        __import__('subprocess').run(
            PY + [os.path.join(REPO, 'py_tools', 'board_score.py'), b, '--json', sj],
            capture_output=True, text=True, cwd=REPO)
        with open(evidence(sj, 'the score'), encoding='utf-8') as fh:
            pr = json.load(fh)['components']['diff_pairs']['pairs']['/DATA+']
        ok(pr.get('lane_skew_mm') == 0.105 and pr.get('skew_mm') == 2.2846,
           f"board_score: lane skew {pr.get('lane_skew_mm')}, total-copper skew {pr.get('skew_mm')}")

        print("3. a .kicad_dru skew rule stays on total copper")
        dru = b[:-len('.kicad_pcb')] + '.kicad_dru'
        with open(dru, 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n(rule "data skew"\n  (condition "A.NetName == \'/DATA+\' '
                     '|| A.NetName == \'/DATA-\'")\n  (constraint skew (max 0.15mm)))\n')
        out = drc(b, flagged='2.2846mm shorter than /DATA+')
        ok("'data skew'" in out and 'Recorded lane' not in out,
           "the file's rule binds the pair and is graded on all its copper")
        os.remove(dru)

        print("4. a named lane")
        named = os.path.join(td, 'named.kicad_pcb')
        check(PY + [tool('route_diff.py'), os.path.join(td, 'in.kicad_pcb'), named] + PAIR
              + ['--diff-pair-intra-lane', '/DATA+=J1.5:IC4.10'], accept=True, timeout=1800)
        lane = (record(named).get('/DATA+') or {}).get('lane') or {}
        ok(lane == {'p': ['J1.5', 'IC4.10'], 'n': ['J1.4', 'IC4.9']},
           f"the pads named, and the N member's on the same parts: {lane}")

        print("5. lanes that cannot be")
        for spec, why in (('/DATA+=J1.5:R9.1', 'expected two different copper pads of /DATA+'),
                          ('/NOPE+=J1.5:IC4.10', 'is not a member of a pair this step routes'),
                          ('/DATA+', 'expected P_NET=REF.PAD:REF.PAD or auto')):
            check(PY + [tool('route_diff.py'), os.path.join(td, 'in.kicad_pcb'),
                        os.path.join(td, 'bad.kicad_pcb')] + PAIR
                  + ['--diff-pair-intra-lane', spec], refuse=why, code=2, timeout=900)
            ok(True, f"{spec}: refused, exit 2 ({why})")

        print("6. the flag survives into a plan")
        sys.path.insert(0, os.path.join(TESTS, 'stress'))
        import manifest_to_plan as m2p
        ok(m2p.LIST_FLAGS.get('--diff-pair-intra-lane') == 'diff_pair_intra_lane',
           "manifest_to_plan carries --diff-pair-intra-lane as diff_pair_intra_lane")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
