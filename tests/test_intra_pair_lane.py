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
  6. a recorded manifest carrying the flag converts to a plan step with it;
  7. a `.kicad_dru` skew rule binding the pair at the PAIR step: the lane is
     ignored, the pair is matched on total copper as KiCad grades the rule,
     and the step says so on the console, in the record and in its JSON;
  8. the lane flag without intra-pair matching says the lane is unused and
     records nothing;
  9. unit, `intra_pair_lane.resolve_auto`: the lane is the two terminals the
     pair step routed -- a 2-terminal pair's two, a multi-point pair's
     coupled legs' ends even where the copper already joins a farther
     branch -- and a pair whose legs do not form one path, or whose P and N
     end on different parts, gets no lane and says why.
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
from types import SimpleNamespace  # noqa: E402

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


def units():
    """Row 9: resolve_auto on synthetic pairs."""
    from synth import make_pad, make_seg, make_net, make_pcb
    from intra_pair_lane import AUTO, resolve_auto
    P, N = 1, 2

    def board(refs_p, refs_n, joined_p=()):
        pads = {P: [make_pad(P, x, 0.0, ref=r, num='1') for r, x in refs_p],
                N: [make_pad(N, x, 0.5, ref=r, num='2') for r, x in refs_n]}
        segs = [make_seg(a, 0.0, b, 0.0, net_id=P) for a, b in joined_p]
        return make_pcb(nets={P: make_net(P, '/X+'), N: make_net(N, '/X-')},
                        segments=segs, vias=[], pads_by_net=pads, zones=[])
    pair = SimpleNamespace(p_net_id=P, n_net_id=N, p_net_name='/X+', n_net_name='/X-')
    pairs = [('/X', pair)]

    def term(pcb, ref):
        return tuple(next(q for q in pcb.pads_by_net[n] if q.component_ref == ref)
                     for n in (P, N))

    pcb = board([('J1', 0.0), ('U1', 10.0)], [('J1', 0.0), ('U1', 10.0)])
    lanes, why = resolve_auto({P: AUTO, N: AUTO}, pairs, {P: {'is_diff_pair': True}}, pcb)
    ok(lanes == {P: ('J1.1', 'U1.1'), N: ('J1.2', 'U1.2')} and why == {},
       f"a 2-terminal pair: its two terminals {lanes}")

    pcb = board([('J1', 0.0), ('U1', 10.0), ('R1', 20.0)],
                [('J1', 0.0), ('U1', 10.0), ('R1', 20.0)],
                joined_p=((0.0, 10.0), (10.0, 20.0)))
    res = {'leg_results': [{'_leg_terms': (term(pcb, 'J1'), term(pcb, 'U1'))}]}
    lanes, why = resolve_auto({P: AUTO, N: AUTO}, pairs, {P: res}, pcb)
    ok(lanes == {P: ('J1.1', 'U1.1'), N: ('J1.2', 'U1.2')},
       f"a multi-point pair: the routed leg J1 -> U1, not the joined branch to R1: {lanes}")
    res = {'leg_results': [{'_leg_terms': (term(pcb, 'J1'), term(pcb, 'U1'))},
                           {'_leg_terms': (term(pcb, 'U1'), term(pcb, 'R1'))}]}
    lanes, _ = resolve_auto({P: AUTO, N: AUTO}, pairs, {P: res}, pcb)
    ok(lanes == {P: ('J1.1', 'R1.1'), N: ('J1.2', 'R1.2')},
       f"two contiguous legs: the chain's ends {lanes}")

    pcb = board([('J1', 0.0), ('U1', 10.0), ('R1', 20.0), ('R2', 30.0)],
                [('J1', 0.0), ('U1', 10.0), ('R1', 20.0), ('R2', 30.0)])
    res = {'leg_results': [{'_leg_terms': (term(pcb, 'J1'), term(pcb, 'U1'))},
                           {'_leg_terms': (term(pcb, 'R1'), term(pcb, 'R2'))}]}
    lanes, why = resolve_auto({P: AUTO, N: AUTO}, pairs, {P: res}, pcb)
    ok(lanes is None and 'do not form one path' in why.get(P, ''),
       f"legs that do not form one path: no lane, {why}")

    pcb = board([('J1', 0.0), ('U1', 10.0)], [('J2', 0.0), ('U1', 10.0)])
    lanes, why = resolve_auto({P: AUTO, N: AUTO}, pairs, {P: {'is_diff_pair': True}}, pcb)
    ok(lanes is None and 'P and N end on different parts' in why.get(P, ''),
       f"P and N on different parts: no lane, {why}")


def main():
    print("9. resolve_auto (unit)")
    units()
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
        for spec, why in (('/DATA+=J1.5:R9.1', 'R9.1 names 0 of its copper pads'),
                          ('/NOPE+=J1.5:IC4.10', 'is not a member of a pair this step routes'),
                          ('/DATA+', 'expected P_NET=REF.PAD:REF.PAD or auto')):
            check(PY + [tool('route_diff.py'), os.path.join(td, 'in.kicad_pcb'),
                        os.path.join(td, 'bad.kicad_pcb')] + PAIR
                  + ['--diff-pair-intra-lane', spec], refuse=why, code=2, timeout=900)
            ok(True, f"{spec}: refused, exit 2 ({why})")

        print("7. a .kicad_dru skew rule binds the pair at the pair step")
        rb = os.path.join(td, 'ruled.kicad_pcb')
        check(PY + [tool('copy_board.py'), os.path.join(td, 'in.kicad_pcb'), rb],
              accept=True)
        with open(rb[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n(rule "data skew"\n  (condition "A.inDiffPair'
                     '(\'/DATA*\')")\n  (constraint skew (max 0.15mm) '
                     '(within_diff_pairs)))\n')
        rs1, rjs = (os.path.join(td, n) for n in ('ruled_s1.kicad_pcb', 'ruled.json'))
        r = check(PY + [tool('route_diff.py'), rb, rs1] + PAIR
                  + ['--diff-pair-intra-lane', 'auto', '--json-out', rjs],
                  accept=True, timeout=1800)
        log = (r.stdout or '') + (r.stderr or '')
        ok("/DATA is matched on total copper, not on its lane: .kicad_dru skew rule "
           "'data skew' binds the pair" in log
           and 'are matched, recorded and graded on the path' not in log,
           "the console says the lane is ignored")
        rec = record(rs1).get('/DATA+') or {}
        ok('lane' not in rec and "'data skew'" in (rec.get('lane_ignored') or ''),
           f"the record says so, and names no lane: {rec}")
        with open(evidence(rjs, '--json-out'), encoding='utf-8') as fh:
            row = (json.load(fh).get('intra_pair_matching') or [{}])[0]
        ok("'data skew'" in (row.get('lane_ignored') or '') and 'lane' not in row,
           f"and so does the step's JSON row: {row.get('lane_ignored')}")

        print("8. the lane without intra-pair matching")
        um = os.path.join(td, 'unmatched.kicad_pcb')
        r = check(PY + [tool('route_diff.py'), os.path.join(td, 'in.kicad_pcb'), um,
                        '--nets', '/DATA+', '/DATA-', '--force-reroute',
                        '--diff-pair-intra-lane', 'auto'], accept=True, timeout=1800)
        log = (r.stdout or '') + (r.stderr or '')
        ok('Intra-pair lane (#1869): unused for 1 pair(s) this step does not '
           'intra-match (/DATA)' in log
           and 'are matched, recorded and graded on the path' not in log
           and not record(um), "the step says the lane is unused and records nothing")

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
