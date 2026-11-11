#!/usr/bin/env python3
"""#1832: a `.kicad_dru` clearance or track_width rule scoped by a courtyard
term is graded by where each track and pad stands, as #1805 grades a via.

`(rule ... (condition "A.insideCourtyard('U1')") (constraint clearance ...))`
was NOT GRADED: its copper was graded at the class value, so a tightening
rule missed KiCad's errors inside the part, a relaxing one counted pairs
KiCad passes there, and every pair -- far from the part too -- was tagged
"may bind" (#1528). A track_width rule of that kind was NOT GRADED (size).

Every expectation below is kicad-cli 10.0.6's verdict on the same board
(Default class clearance 0.2 mm, every track 0.15 mm wide):
  1. width: which tracks a `track_width (min 0.3mm)` rule binds, per
     courtyard predicate -- a track whose capsule overlaps the courtyard
     binds (partly inside, or reaching in by its half-width), one tangent to
     it does not, and the track's own layer does not matter;
  2. clearance: which N/M pairs a `clearance (min 0.5mm)` rule flags, per
     condition -- KiCad tries a condition both ways, so one item inside is
     enough for `A.insideCourtyard()`, and a pad binds by its copper (one
     straddling the edge is inside);
  3. the issue's relaxing rule: of two pairs at 0.15 mm under a 0.1 mm rule,
     only the one outside U1 is flagged, untagged;
  4. none of these rules is NOT GRADED, on the console or in --json, and
     board_score / check_complete do not list the relaxing rule ungraded.

    python3 tests/test_1832_courtyard_track_pad_rules.py
"""
import json
import os
import re
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FAILS = []
_U = [0]


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def _uid():
    _U[0] += 1
    return f"a0000000-0000-0000-0000-{_U[0]:012d}"


def _fp(ref, x, y, side, pads=(), crt=(-2, -2, 2, 2), other=True):
    """A part with a courtyard on its own side and, with ``other``, a second
    4x4 one on the other side 5 mm lower; SMD pads (number, x, y, size, net)."""
    o = 'B' if side == 'F' else 'F'
    x0, y0, x1, y1 = crt
    s = (f'(footprint "lib:{ref}_fp" (layer "{side}.Cu") (uuid "{_uid()}")\n  (at {x} {y} 0)\n'
         f'  (property "Reference" "{ref}" (at 0 -3 0) (layer "{side}.SilkS") (uuid "{_uid()}") '
         f'(effects (font (size 1 1) (thickness 0.15))))\n'
         f'  (fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.05) (type solid)) '
         f'(fill no) (layer "{side}.CrtYd") (uuid "{_uid()}"))\n')
    if other:
        s += (f'  (fp_rect (start -2 3) (end 2 7) (stroke (width 0.05) (type solid)) (fill no) '
              f'(layer "{o}.CrtYd") (uuid "{_uid()}"))\n')
    for num, px, py, sz, net in pads:
        s += (f'  (pad "{num}" smd rect (at {px} {py}) (size {sz} {sz}) (layers "{side}.Cu") '
              f'(net {net} "{"NM"[net - 1]}") (uuid "{_uid()}"))\n')
    return s + ')\n'


def _seg(x0, y0, x1, y1, net, layer='F.Cu'):
    return (f' (segment (start {x0} {y0}) (end {x1} {y1}) (width 0.15) (layer "{layer}") '
            f'(net {net}) (uuid "{_uid()}"))\n')


HEAD = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
        ' (general (thickness 1.6) (legacy_teardrops no))\n (paper "A4")\n'
        ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user) '
        '(31 "F.CrtYd" user "F.Courtyard") (29 "B.CrtYd" user "B.Courtyard") '
        '(5 "F.SilkS" user "F.Silkscreen") (7 "B.SilkS" user "B.Silkscreen"))\n'
        ' (setup (pad_to_mask_clearance 0))\n (net 0 "") (net 1 "N") (net 2 "M")\n'
        ' (gr_rect (start 90 90) (end 130 110) (stroke (width 0.05) (type default)) (fill no) '
        '(layer "Edge.Cuts") (uuid "e0000000-0000-0000-0000-000000000001"))\n')
PRO = {'board': {'design_settings': {'rules': {'min_clearance': 0.0, 'min_track_width': 0.1}}},
       'net_settings': {'meta': {'version': 4},
                        'classes': [{'name': 'Default', 'clearance': 0.2, 'track_width': 0.15}],
                        'netclass_patterns': []}}

#: U1 (F.Cu, at 100,100): F.CrtYd (98,98)-(102,102), B.CrtYd (98,103)-(102,107).
#: U2 (B.Cu, at 120,100): B.CrtYd (118,98)-(122,102), F.CrtYd (118,103)-(122,107).
#: One net-N track per entry, (x0, y0, x1, y1, layer).
WIDTH_TRACKS = {
    'in_F': (99, 100, 101, 100, 'F.Cu'),
    'partial_F': (101.5, 99, 104, 99, 'F.Cu'),
    'side_overlap': (102.05, 100.2, 102.05, 100.8, 'F.Cu'),     # reaches 0.025 in
    'side_clear': (102.1, 101.0, 102.1, 101.6, 'F.Cu'),         # 0.025 short
    'side_tangent': (102.075, 101.9, 102.075, 102.5, 'F.Cu'),   # touches the edge
    'end_overlap': (102.05, 98.4, 103.5, 98.4, 'F.Cu'),         # its end cap reaches in
    'B_under_F': (99, 101, 101, 101, 'B.Cu'),
    'F_in_Bcy': (99, 105, 101, 105, 'F.Cu'),
    'B_in_Bcy': (99, 106, 101, 106, 'B.Cu'),
    'u2_B_in': (119, 100, 121, 100, 'B.Cu'),
    'u2_F_in': (119, 101, 121, 101, 'F.Cu'),
    'u2_F_other': (119, 105, 121, 105, 'F.Cu'),
    'u2_B_other': (119, 106, 121, 106, 'B.Cu'),
    'far': (110, 100, 112, 100, 'F.Cu'),
}
WIDTH_BOARD = (HEAD + _fp('U1', 100, 100, 'F') + _fp('U2', 120, 100, 'B')
               + ''.join(_seg(x0, y0, x1, y1, 1, l)
                         for x0, y0, x1, y1, l in WIDTH_TRACKS.values()) + ')\n')
_U1F = {'in_F', 'partial_F', 'side_overlap', 'end_overlap', 'B_under_F'}
_U1B = {'F_in_Bcy', 'B_in_Bcy'}
#: {condition: the tracks kicad-cli 10.0.6 flags under track_width (min 0.3mm)}
KICAD_WIDTH = {
    "A.insideCourtyard('U1')": _U1F | _U1B,
    "A.intersectsCourtyard('U1')": _U1F | _U1B,
    "A.insideFrontCourtyard('U1')": _U1F,
    "A.insideBackCourtyard('U1')": _U1B,
    "A.insideCourtyard('U2')": {'u2_B_in', 'u2_F_in', 'u2_F_other', 'u2_B_other'},
    "A.insideFrontCourtyard('U2')": {'u2_B_in', 'u2_F_in'},
    "A.insideBackCourtyard('U2')": {'u2_F_other', 'u2_B_other'},
}

#: N/M pairs 0.30 mm apart (0.375 for 'pad_out'); a pad is (x, y) of its
#: centre, a track (x, y) of its start, as kicad-cli reports them. U1 stands
#: at (100, 101) here, its F.CrtYd (98,99)-(102,103): the 'one_in' pair
#: straddles y = 103, not an even millimetre, where check_drc's 2 mm
#: segment cells would part two tracks before any rule is asked.
CLEAR_PAIRS = {
    'both_in': ((99, 101), (99, 101.45)),
    'one_in': ((99, 102.8), (99, 103.25)),          # N inside, M outside
    'both_out': ((109, 101), (109, 101.45)),
    'pad_in': ((100, 99.5), (100.625, 99.5)),       # U1.1 against an M track inside
    'pad_edge': ((102, 101), (102.625, 100.7)),     # U5.1 straddles U1's edge; track out
    'pad_out': ((97.5, 101.5), (98.2, 101.2)),      # U6.1 outside; track inside
}
CLEAR_BOARD = (HEAD
               + _fp('U1', 100, 101, 'F', pads=[('1', 0, -1.5, 0.5, 1)])
               + _fp('U5', 102, 101, 'F', pads=[('1', 0, 0, 0.5, 1)],
                     crt=(-0.4, -0.4, 0.4, 0.4), other=False)
               + _fp('U6', 97.5, 101.5, 'F', pads=[('1', 0, 0, 0.5, 1)],
                     crt=(-0.4, -0.4, 0.4, 0.4), other=False)
               + _seg(99, 101, 101, 101, 1) + _seg(99, 101.45, 101, 101.45, 2)
               + _seg(99, 102.8, 101, 102.8, 1) + _seg(99, 103.25, 101, 103.25, 2)
               + _seg(109, 101, 111, 101, 1) + _seg(109, 101.45, 111, 101.45, 2)
               + _seg(100.625, 99.5, 101.5, 99.5, 2)
               + _seg(102.625, 100.7, 102.625, 101.3, 2)
               + _seg(98.2, 101.2, 98.2, 101.8, 2)
               + ')\n')
#: {condition: the pairs kicad-cli 10.0.6 flags under clearance (min 0.5mm)}
KICAD_CLEAR = {
    "A.insideCourtyard('U1')": {'both_in', 'one_in', 'pad_in', 'pad_edge', 'pad_out'},
    "B.insideCourtyard('U1')": {'both_in', 'one_in', 'pad_in', 'pad_edge', 'pad_out'},
    "A.insideCourtyard('U1') && B.insideCourtyard('U1')": {'both_in', 'pad_in'},
    "A.insideCourtyard('U1') && A.NetName == 'M'": {'both_in', 'pad_in', 'pad_out'},
    "A.insideCourtyard('U1') && A.Type == 'Pad'": {'pad_in', 'pad_edge'},
    "!A.insideCourtyard('U1')": {'one_in', 'both_out', 'pad_edge', 'pad_out'},
}


def stage(td, name, board, rule):
    d = os.path.join(td, name)
    os.makedirs(d)
    pcb = os.path.join(d, 'b.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as fh:
        fh.write(board)
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(PRO, fh)
    with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(f'(version 1)\n{rule}\n')
    return evidence(pcb, 'the probe board')


def grade(pcb):
    """(console, the --json document) of a check_drc run, which exits 0
    (nothing found) or 1 (found), never by an accident."""
    out = os.path.splitext(pcb)[0] + '.drc.json'
    argv = PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), pcb, '--json', out,
                 '--max-print', '0']
    try:
        r = check(argv, code=1, refuse='DRC VIOLATIONS')
    except AssertionError as e:
        if 'got exit 0' not in str(e):
            raise
        r = check(argv, accept=True, refuse=None)
    with open(evidence(out, 'the --json document'), encoding='utf-8') as fh:
        return r.stdout, json.load(fh)


def _pt(x, y):
    return (round(float(x), 3), round(float(y), 3))


def flagged_pairs(doc):
    names = {frozenset(map(lambda p: _pt(*p), v)): k for k, v in CLEAR_PAIRS.items()}
    got, other = set(), []
    for v in doc['items']:
        if v.get('accepted'):
            continue
        if v['type'] == 'segment-segment':
            key = frozenset((_pt(*v['loc1'][:2]), _pt(*v['loc2'][:2])))
        elif v['type'] == 'pad-segment':
            key = frozenset((_pt(*v['pad_loc']), _pt(*v['seg_loc'][:2])))
        else:
            other.append(v['type'])
            continue
        got.add(names.get(key, f"unknown {sorted(key)}"))
    return got, other


def width(td):
    print("1. track_width scoped by a courtyard term, per track")
    starts = {_pt(x0, y0): n for n, (x0, y0, _x1, _y1, _l) in WIDTH_TRACKS.items()}
    for i, (cond, want) in enumerate(KICAD_WIDTH.items()):
        pcb = stage(td, f'w{i}', WIDTH_BOARD,
                    f'(rule "cw" (condition "{cond}") (constraint track_width (min 0.3mm)))')
        out, doc = grade(pcb)
        got = {starts.get(_pt(*v['seg_loc'][:2]), '?') for v in doc['items']
               if v['type'] == 'track-width' and not v.get('accepted')}
        ok(got == want, f"{cond}: {sorted(got)}" + ('' if got == want else f" want {sorted(want)}"))
        ok('NOT GRADED' not in out and not doc.get('not_graded'),
           f"{cond}: not NOT GRADED")


def clearance(td):
    print("2. clearance scoped by a courtyard term, per pair")
    for i, (cond, want) in enumerate(KICAD_CLEAR.items()):
        pcb = stage(td, f'c{i}', CLEAR_BOARD,
                    f'(rule "cc" (condition "{cond}") (constraint clearance (min 0.5mm)))')
        out, doc = grade(pcb)
        got, other = flagged_pairs(doc)
        ok(got == want and not other,
           f"{cond}: {sorted(got)}" + ('' if got == want else f" want {sorted(want)}")
           + (f" and {other}" if other else ''))
        ok('NOT GRADED' not in out and not doc.get('not_graded')
           and 'UNVERIFIED' not in out and doc.get('unverified') == 0,
           f"{cond}: not NOT GRADED, nothing tagged unverified")
        rows = [v for v in doc['items'] if v.get('courtyard_rule')]
        # every pair is 0.30+ apart, over the class: only the rule flags one
        ok(all(v['courtyard_rule'] == 'cc' and v['required_mm'] == 0.5 for v in rows)
           and len(rows) == len(want),
           f"{cond}: each flagged row names the rule, at 0.5")


def relaxing(td):
    print("3. the issue's relaxing rule")
    board = (HEAD + _fp('U1', 100, 100, 'F')
             + _seg(99, 100.5, 101, 100.5, 1) + _seg(99, 100.8, 101, 100.8, 2)
             + _seg(109, 100.5, 111, 100.5, 1) + _seg(109, 100.8, 111, 100.8, 2) + ')\n')
    pcb = stage(td, 'relax', board, '(rule "relax_clearance" (condition '
                "\"A.insideCourtyard('U1')\") (constraint clearance (min 0.1mm)))")
    out, doc = grade(pcb)
    rows = [v for v in doc['items'] if v['type'] == 'segment-segment' and not v.get('accepted')]
    # kicad-cli 10.0.6: one error, the pair outside U1 (netclass 0.2, actual 0.15)
    ok([_pt(*v['loc1'][:2]) for v in rows] == [(109.0, 100.5)],
       f"only the pair outside U1 is flagged: {[v['loc1'][:2] for v in rows]}")
    ok(not any(v.get('unverified_rule') for v in rows) and 'may bind' not in out,
       "it is not tagged 'may bind'")
    ok(re.search(r"Courtyard-scoped clearance rules .*'relax_clearance':0\.1", out)
       is not None, "the console names the rule as graded by position")
    # the board has a DRC error, so board_score's verdict is blocking (exit 4)
    r = check(PY + [os.path.join(REPO, 'py_tools', 'board_score.py'), pcb, '--quiet'],
              code=4, refuse='SCORE_JSON=')
    line = next((x for x in r.stdout.splitlines() if x.startswith('SCORE_JSON=')), '')
    drc = ((json.loads(line[len('SCORE_JSON='):]) if line else {}).get('components')
           or {}).get('drc') or {}
    ok(bool(line) and drc.get('count') == 1 and not drc.get('ungraded_rules')
       and not drc.get('unverified'),
       f"board_score: drc 1, the rule not ungraded, nothing unverified: {json.dumps(drc)[:300]}")
    r = check(PY + [os.path.join(REPO, 'check_complete.py'), pcb, '--skip-slow'],
              code=4, refuse='VERDICT')
    ok('nothing grades' not in r.stdout,
       "check_complete does not call the rule ungraded")


def main():
    with tempfile.TemporaryDirectory() as td:
        width(td)
        clearance(td)
        relaxing(td)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
