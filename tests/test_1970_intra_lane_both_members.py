#!/usr/bin/env python3
"""#1970: `--diff-pair-intra-lane` takes a lane for each member of a pair
whose two members end on different parts.

`intra_pair_lane.resolve` derived the N member's lane from the P member's
entry, as its pads on the same two parts, and raised when the N member has
not exactly one pad on each. On `kicad_files/cap_chain.kicad_pcb` `DPA_P` runs
C1.1 - J1.1 and `DPA_N` C2.1 - J1.2 (a series capacitor per member, one
connector), so `DPA_N` has no pad on C1, and the refusal said "name that
member's lane too". Naming it too was refused with the same line, in either
order of the entries, because each entry was derived before the next one was
read. A member named now keeps the pads named; the derivation is only for a
member no entry names.

Rows (route_diff.py on cap_chain, `--nets DPA_P DPA_N
--diff-pair-intra-match`, then check_drc.py and board_score.py on the
output):
  1. control: `DPA_P=C1.1:J1.1` alone still refuses, with the text that asks
     for the N member's lane;
  2. `DPA_P=C1.1:J1.1 DPA_N=C2.1:J1.2`: the step says the pair is matched on a
     named lane, JSON_SUMMARY's `intra_pair_matching` row and the project's
     record carry both members' pads, check_drc grades the recorded tolerance
     on that lane (`Recorded lane ... DPA_N C2.1 -> J1.2`, no skew violation,
     `recorded_lanes` in --json) and board_score's pair record carries the
     lane and its skew;
  3. the same two entries in the other order record the same lane;
  4. control: without the option the pair routes (`Diff pairs:    1/1
     routed`) and the record names no lane;
  5. both named, then the P member's lane again spelled the other way round
     (`DPA_P=J1.1:C1.1`): the lane is taken, in the order of the member's
     last entry, as main orders a member named twice;
  6. both members named, one of them with a pad its net lacks, refuses naming
     that entry, in either order;
  7. control, unit, on `kicad_files/lvds_converter_dualclk.kicad_pcb`, where
     `/DATA-` shares J1 and IC4 with `/DATA+`: one entry still derives the
     other member's pads as before (`/DATA+=J1.5:IC4.10` gives `/DATA-`
     J1.4 / IC4.9), both named keep the pads named, and `/DATA+` named twice,
     the second time reversed, resolves as on main;
  8. route_diff.py on lvds_converter_dualclk (the step of
     tests/test_intra_pair_lane.py): `/DATA+` named twice with other pads
     (`/DATA+=J1.5:IC4.10 /DATA+=J1.5:R4.1`) refuses, naming both entries;
     main took /DATA+ J1.5 - R4.1 and /DATA- J1.4 - IC4.9 from it. Control:
     the same lane spelled twice, reversed, routes and records the lane main
     records (/DATA+ IC4.10 - J1.5, /DATA- J1.4 - IC4.9).

Run: python3 -X utf8 tests/test_1970_intra_lane_both_members.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence, tool  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
CAP = os.path.join(REPO, 'kicad_files', 'cap_chain.kicad_pcb')
LVDS = os.path.join(REPO, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
STEP = ['--nets', 'DPA_P', 'DPA_N', '--diff-pair-intra-match']
BOTH = ['DPA_P=C1.1:J1.1', 'DPA_N=C2.1:J1.2']
LANE = {'p': ['C1.1', 'J1.1'], 'n': ['C2.1', 'J1.2']}
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def route(td, tag, lane):
    """The pair step on cap_chain; (board, console) when it routed, else
    (None, the reason it did not)."""
    out = os.path.join(td, tag + '.kicad_pcb')
    argv = (PY + [tool('route_diff.py'), os.path.join(td, 'in.kicad_pcb'), out] + STEP
            + (['--diff-pair-intra-lane'] + lane if lane else []))
    try:
        r = check(argv, accept=True, timeout=900)
    except AssertionError as e:
        return None, _reason(e)
    return evidence(out, 'the pair step board'), (r.stdout or '') + (r.stderr or '')


def _reason(err):
    """The `Error:` line of a failed run, else the assertion's first line."""
    lines = str(err).splitlines()
    return next((l for l in lines if l.startswith('Error:')), lines[0])[:220]


def refused(td, lane, why):
    """True when the pair step refuses `lane` for the stated reason, exit 2."""
    try:
        check(PY + [tool('route_diff.py'), os.path.join(td, 'in.kicad_pcb'),
                    os.path.join(td, 'bad.kicad_pcb')] + STEP
              + ['--diff-pair-intra-lane'] + lane, refuse=why, code=2, timeout=900)
        return True, ''
    except AssertionError as e:
        return False, _reason(e)


def close(a, b):
    """Two tools' 4-decimal millimetres for one length: route_diff measures
    its copper in memory and check_drc / board_score re-read the written
    file, and each rounds to 4 decimals, so the same skew may print one unit
    apart in the last place."""
    return a is not None and b is not None and abs(a - b) <= 1e-4 + 1e-9


def summary(console):
    for line in console.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            return json.loads(line[len('JSON_SUMMARY:'):])
    return {}


def recorded(board):
    """The lane the project records for DPA, as the graders read it."""
    from kicad_dru import recorded_intra_pair_lanes
    got = recorded_intra_pair_lanes(board)
    return {'p': got.get('DPA_P'), 'n': got.get('DPA_N')} if got else {}


def cap_rows(td):
    print("1. control: the P member's lane alone")
    good, why = refused(td, ['DPA_P=C1.1:J1.1'],
                        "Error: --diff-pair-intra-lane 'DPA_P=C1.1:J1.1': DPA_N has 0 "
                        "pad(s) on C1 (none), so its end of the lane is not decided; "
                        "name that member's lane too")
    ok(good, f"refuses, asking for DPA_N's lane {why}")

    print("2. both members' lanes named")
    b, log = route(td, 'both', BOTH)
    ok(b is not None and 'Intra-pair lane (#1869): 1 pair(s) are matched, recorded and '
       'graded on the path between two pads (named), not on total copper' in log,
       f"the step takes the named lane {'' if b else log}")
    if b is not None:
        row = (summary(log).get('intra_pair_matching') or [{}])[0]
        ok(row.get('lane') == LANE and row.get('status') == 'within-tolerance',
           f"JSON_SUMMARY intra_pair_matching: {row.get('status')}, lane {row.get('lane')}")
        ok(recorded(b) == LANE, f"the project records the lane: {recorded(b)}")
        js = os.path.join(td, 'both_drc.json')
        r = subprocess.run(PY + [tool('check_drc.py'), b, '--json', js],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=REPO)
        out = (r.stdout or '') + (r.stderr or '')
        line = next((l.strip() for l in out.splitlines() if 'Recorded lane' in l), '')
        ok('Traceback' not in out and 'SKEW-OUT-OF-RANGE' not in out
           and 'NO DRC VIOLATIONS FOUND' in out
           and line.startswith('Recorded lane (route_diff, #1869): DPA_P C1.1 -> J1.1 ')
           and '/ DPA_N C2.1 -> J1.2 ' in line,
           f"check_drc grades the recorded tolerance on the lane: {line[:140]}")
        with open(evidence(js, 'check_drc --json'), encoding='utf-8') as fh:
            lg = (json.load(fh).get('recorded_lanes') or [{}])[0]
        ok(lg.get('lane') == {'DPA_P': LANE['p'], 'DPA_N': LANE['n']}
           and close(lg.get('skew_mm'), row.get('delta_mm')),
           f"check_drc --json recorded_lanes: {lg.get('lane')}, skew {lg.get('skew_mm')} mm")
        sj = os.path.join(td, 'both_score.json')
        subprocess.run(PY + [os.path.join(REPO, 'py_tools', 'board_score.py'), b,
                             '--json', sj], capture_output=True, text=True, cwd=REPO)
        with open(evidence(sj, 'the score'), encoding='utf-8') as fh:
            pr = json.load(fh)['components']['diff_pairs']['pairs'].get('DPA_P') or {}
        ok(pr.get('lane') == {'DPA_P': LANE['p'], 'DPA_N': LANE['n']}
           and close(pr.get('lane_skew_mm'), lg.get('skew_mm')),
           f"board_score: lane {pr.get('lane')}, lane skew {pr.get('lane_skew_mm')} mm")

    print("3. the same entries in the other order")
    b, log = route(td, 'reversed', BOTH[::-1])
    ok(b is not None and recorded(b) == LANE,
       f"the same lane is recorded: {recorded(b) if b else log}")

    print("4. control: no lane")
    b, log = route(td, 'none', None)
    ok(b is not None and 'Diff pairs:    1/1 routed' in log
       and 'Intra-pair lane (#1869)' not in log and recorded(b) == {},
       f"the pair routes and records no lane {'' if b else log}")

    print("5. the P member's lane named again, reversed")
    b, log = route(td, 'again', BOTH + ['DPA_P=J1.1:C1.1'])
    ok(b is not None and recorded(b) == {'p': ['J1.1', 'C1.1'], 'n': LANE['n']},
       f"the lane is taken, P in its last entry's order: {recorded(b) if b else log}")

    print("6. both named, one with a pad its net lacks")
    bad = 'DPA_N=C2.1:J1.9'
    for lane in (['DPA_P=C1.1:J1.1', bad], [bad, 'DPA_P=C1.1:J1.1']):
        good, why = refused(td, lane,
                            f"Error: --diff-pair-intra-lane '{bad}': net DPA_N: J1.9 "
                            f"names 0 of its copper pads (C2.1, J1.2)")
        ok(good, f"{' '.join(lane)}: refuses, naming {bad} {why}")


def lvds_twice(td):
    """Row 8: a member named twice, through the pair step on lvds."""
    board = os.path.join(td, 'lvds.kicad_pcb')
    check(PY + [tool('copy_board.py'), evidence(LVDS, 'the bundled board'), board],
          accept=True)
    step = [board, os.path.join(td, 'lvds_out.kicad_pcb'), '--nets', '/DATA+', '/DATA-',
            '--diff-pair-intra-match', '--length-match-tolerance', '0.15',
            '--force-reroute', '--diff-pair-intra-lane']
    try:
        check(PY + [tool('route_diff.py')] + step + ['/DATA+=J1.5:IC4.10', '/DATA+=J1.5:R4.1'],
              refuse="Error: --diff-pair-intra-lane '/DATA+=J1.5:R4.1': /DATA+'s lane is "
                     "already named by '/DATA+=J1.5:IC4.10', with other pads; name one "
                     "lane per member", code=2, timeout=900)
        ok(True, "/DATA+ named twice with other pads: refused, naming both entries")
    except AssertionError as e:
        ok(False, f"/DATA+ named twice with other pads: refused, naming both entries "
                  f"{_reason(e)}")
    try:
        check(PY + [tool('route_diff.py')] + step + ['/DATA+=J1.5:IC4.10', '/DATA+=IC4.10:J1.5'],
              accept=True, timeout=900)
        from kicad_dru import recorded_intra_pair_lanes
        got = recorded_intra_pair_lanes(evidence(step[1], 'the pair step board'))
        why = {'/DATA+': got.get('/DATA+'), '/DATA-': got.get('/DATA-')}
    except AssertionError as e:
        got, why = {}, _reason(e)
    ok(got.get('/DATA+') == ['IC4.10', 'J1.5'] and got.get('/DATA-') == ['J1.4', 'IC4.9'],
       f"control: the same lane twice, reversed, routes and records main's lane: {why}")


def lvds_rows():
    """Row 7: `resolve` on the bundled board, where the derivation works."""
    from kicad_parser import parse_kicad_pcb
    from net_queries import find_differential_pairs
    from intra_pair_lane import resolve
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(LVDS, 'the bundled board'))
    pairs = sorted(find_differential_pairs(pcb, ['/DATA*']).items())

    def named(spec):
        got = resolve(spec, pairs, pcb)
        return {pcb.nets[i].name: v for i, v in got.items()}
    for spec, want in (
            (['/DATA+=J1.5:IC4.10'], {'/DATA+': ('J1.5', 'IC4.10'),
                                      '/DATA-': ('J1.4', 'IC4.9')}),
            (['/DATA-=J1.4:IC4.9'], {'/DATA-': ('J1.4', 'IC4.9'),
                                     '/DATA+': ('J1.5', 'IC4.10')}),
            (['/DATA+=J1.5:IC4.10', '/DATA-=J1.4:R4.2'],
             {'/DATA+': ('J1.5', 'IC4.10'), '/DATA-': ('J1.4', 'R4.2')}),
            # main's order for a member named twice: its last entry's, and
            # the twin's pads derived from its first
            (['/DATA+=J1.5:IC4.10', '/DATA+=IC4.10:J1.5'],
             {'/DATA+': ('IC4.10', 'J1.5'), '/DATA-': ('J1.4', 'IC4.9')})):
        got = named(spec)
        ok(got == want and list(got) == list(want), f"{' '.join(spec)}: {got}")


def main():
    print("7. control: lvds_converter_dualclk /DATA (unit)")
    lvds_rows()
    with tempfile.TemporaryDirectory() as td:
        check(PY + [tool('copy_board.py'), evidence(CAP, 'the bundled board'),
                    os.path.join(td, 'in.kicad_pcb')], accept=True)
        cap_rows(td)
        print("8. a member named twice, on lvds_converter_dualclk")
        lvds_twice(td)
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
