#!/usr/bin/env python3
"""#1064: `place_pose` refuses a PAD STACK, any net, the way check_assembly does.

`place_pose` graded a pose with `grade_pad_legality`, whose pair loop skips a
same-net pad pair before it measures anything. Two parts' pads stacked on ONE
net were therefore no pair at all: esp_prog's C4 put on Y1 (both pads are
`Net-(C4-Pad1)`) exited 0 with `legal: true, no_worse: true`, while
check_assembly graded the written board `C4 <-> Y1 pad_intersection 0.0412mm2
side F BLOCKING`, NOT BUILDABLE. Run 38 hit it again after #1100 (C15 on C19,
both pad pairs same-net): #1100's new-PAIR arm compared two empty sets.

The stack is now measured by check_assembly's own channel
(`legality.pad_intersection_pairs`, lifted verbatim out of
`grade_body_overlap`), published as `pad_stack_count` / `pad_stack_area` /
`pad_stack_pairs`, and gated by the count, magnitude and new-pair arms
`pose_ops.worsened` already has. `check_floorplan` PRINTS the exact count
(no rule).

The synthetic arms rebuild run 38's geometry: two identical 0.65 x 1.2 two-pad
caps (+5V / GND), one offset (0.222, 0.321) from the other, so each pad pair
overlaps 0.428 x 0.879 = 0.3762 mm2 and only same-net pads touch.

Run: python3 -X utf8 tests/test_1064_place_pose_same_net_stack.py [case ...]
"""
import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 1500

PLACE_POSE = os.path.join(ROOT, 'py_placer', 'place_pose.py')
ASSEMBLY = os.path.join(ROOT, 'py_tools', 'check_assembly.py')
FLOORPLAN = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')
ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
PILE = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
DAMAGED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                       'tigard_damaged.kicad_pcb')

#: esp_prog C4 at rot 0: its pad 1 and Y1's pad 1 are one net, and the pad
#: edges meet at x = 136.138 (measured: 136.137 stacks 0.0006 mm2).
C4_Y = 104.100
C4_STACK_X = 136.063
C4_KISS_X = 136.188
#: A rotated near-touch the bounding-box census calls a stack and the exact
#: grader does not: C4 at -45 degrees beside Y1's corner. Found by a 0.01 mm
#: sweep at 16c096b1, 0.04-0.05 mm inside every boundary measured (exact pad
#: gap about 0.05-0.075 mm); quench's `pad_intersection_pairs` reads 1 here
#: and check_assembly reads 0. The issue's own proposal (gate on the
#: box-based `PairShortfall.stack`) would refuse it.
C4_NEAR = (136.06, 103.62, 315)

#: The census on the inputs this test leans on, measured at 16c096b1 with
#: check_assembly's own channel (grade_body_overlap's pad_intersection
#: pairs). The tracked corpus has none; the two fixtures are why arm 15 is
#: not an equality of empty lists.
PINNED_STACKS = {'run29_pile': 83, 'tigard_damaged': 29}
#: ... and their sides: front copper, and '' for a through-hole pair (all
#: layers), as check_assembly prints them.
PINNED_SIDES = {'run29_pile': {'F': 66, '': 17},
                'tigard_damaged': {'F': 28, '': 1}}

CAP = ('  (footprint "t:C" (layer "F.Cu") (at {x} {y} {rot}){lock}\n'
       '    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
       '    (fp_rect (start -1.3 -0.8) (end 1.3 0.8) (stroke (width 0.05)'
       ' (type default)) (layer "F.CrtYd"))\n'
       '    (pad "1" smd rect (at -0.75 0 {rot}) (size 0.65 1.2)'
       ' (layers "F.Cu") (net 1 "+5V"))\n'
       '    (pad "2" smd rect (at 0.75 0 {rot}) (size 0.65 1.2)'
       ' (layers "F.Cu") (net 2 "GND")))\n')

R = ('  (footprint "t:R" (layer "F.Cu") (at {x} {y})\n'
     '    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
     '    (fp_rect (start -1 -0.5) (end 1 0.5) (stroke (width 0.05)'
     ' (type default)) (layer "F.CrtYd"))\n'
     '    (pad "1" smd rect (at -0.5 0) (size 0.6 0.6) (layers "F.Cu")'
     ' (net {a} "N{a}"))\n'
     '    (pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu")'
     ' (net {b} "N{b}")))\n')

#: C19 and C20 are the two "victims"; the run-38 offset puts C15 on one.
C19 = (20.0, 15.0)
C20 = (30.0, 15.0)
OFF = (0.222, 0.321)


def _board(td, caps, rs=(), name='b'):
    """`caps`: [(ref, x, y, rot, locked)], `rs`: [(ref, x, y, net_a, net_b)]."""
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "+5V") (net 2 "GND") (net 3 "N3")'
            ' (net 4 "N4")\n'
            '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n')
    for ref, x, y, rot, locked in caps:
        text += CAP.format(ref=ref, x=x, y=y, rot=rot,
                           lock=' (locked yes)' if locked else '')
    for ref, x, y, a, b in rs:
        text += R.format(ref=ref, x=x, y=y, a=a, b=b)
    text += ')\n'
    p = os.path.join(td, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return p


def _on(victim, dy=OFF[1]):
    return (round(victim[0] + OFF[0], 3), round(victim[1] + dy, 3))


def _summary(r):
    for line in reversed((r.stdout or '').splitlines()):
        if line.startswith('JSON_SUMMARY: '):
            return json.loads(line[len('JSON_SUMMARY: '):])
    raise AssertionError('no JSON_SUMMARY line:\n' + (r.stdout or '')[-1500:])


def _pose(board, out, *op, refuse=None, code=None, accept=False):
    return run_utils.check([sys.executable, '-X', 'utf8', PLACE_POSE, board,
                            out] + [str(a) for a in op],
                           refuse=refuse, code=code, accept=accept)


def _assembly_stacks(board, td):
    """check_assembly's own pad_intersection pairs on a WRITTEN board."""
    j = os.path.join(td, 'ca.json')
    subprocess.run([sys.executable, '-X', 'utf8', ASSEMBLY,
                    run_utils.evidence(board, 'written board'), '--json', j],
                   capture_output=True, text=True, cwd=ROOT)
    with open(run_utils.evidence(j, 'check_assembly json'),
              encoding='utf-8') as fh:
        doc = json.load(fh)
    return sorted([p['a'], p['b'], p['area_mm2'], p['side']]
                  for p in doc['blocking_pairs']
                  if p['kind'] == 'pad_intersection')


def _footprint_xy(board, ref):
    from kicad_parser import parse_kicad_pcb
    fp = parse_kicad_pcb(board).footprints[ref]
    return round(fp.x, 3), round(fp.y, 3)


# --------------------------------------------------------------------------
# esp_prog: the issue's own repro
# --------------------------------------------------------------------------

def test_the_issue_repro_is_refused():
    """Arm 1. Exit 4, nothing written, the stack named by its NEW pair."""
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'o.kicad_pcb')
        r = _pose(ESP, out, 'set', 'C4', C4_STACK_X, C4_Y, '--rot', '0',
                  refuse='pad_stack_pairs new: C4/Y1', code=4)
        assert not os.path.exists(out), 'a refusal wrote a board'
        s = _summary(r)
        assert s['no_worse'] is False, s['no_worse']
        assert (s['pad_stack_count_before'],
                s['pad_stack_count_after']) == (0, 1), s
        assert s['pad_stack_pairs_after'] == [['C4', 'Y1', 0.0412, 'F']], \
            s['pad_stack_pairs_after']
        assert 'NOT BUILDABLE' in s['refused'], s['refused']
        assert s['pad_stack_basis'].startswith(
            "check_assembly's pad_intersection channel"), s['pad_stack_basis']
        assert "pads are stacked" in s['legal_basis'], s['legal_basis']
    print("  C4 on Y1 refused at exit 4, nothing written")


def test_force_writes_and_check_assembly_agrees():
    """Arm 2. `--force` writes; the summary's rows ARE check_assembly's."""
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'o.kicad_pcb')
        r = _pose(ESP, out, 'set', 'C4', C4_STACK_X, C4_Y, '--rot', '0',
                  '--force', accept=True)
        s = _summary(r)
        assert s['forced'] is True, s
        assert 'C4/Y1' in r.stdout, r.stdout[-1500:]
        assert _assembly_stacks(out, td) == s['pad_stack_pairs_after'] == [
            ['C4', 'Y1', 0.0412, 'F']], s['pad_stack_pairs_after']
    print("  forced write: check_assembly reports the same C4/Y1 row")


def test_a_kiss_is_legal():
    """Arm 3. Pads 0.05 mm apart: no stack, legal, no_worse."""
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'o.kicad_pcb')
        r = _pose(ESP, out, 'set', 'C4', C4_KISS_X, C4_Y, '--rot', '0',
                  accept=True)
        s = _summary(r)
        assert s['legal'] is True and s['no_worse'] is True, s
        assert s['pad_stack_count_after'] == 0, s['pad_stack_pairs_after']
        assert _assembly_stacks(out, td) == [], 'check_assembly disagrees'
    print("  a 0.05 mm kiss stays legal")


def test_the_boundary_is_check_assemblys():
    """Arm 4. 1 um of overlap refuses; touching edges do not."""
    with tempfile.TemporaryDirectory() as td:
        _pose(ESP, os.path.join(td, 'a.kicad_pcb'), 'set', 'C4', 136.137,
              C4_Y, '--rot', '0', refuse='pad_stack_pairs new: C4/Y1',
              code=4)
        _pose(ESP, os.path.join(td, 'b.kicad_pcb'), 'set', 'C4', 136.138,
              C4_Y, '--rot', '0', accept=True)
    print("  136.137 refused, 136.138 accepted")


def _near_touch_board(td):
    out = os.path.join(td, 'near.kicad_pcb')
    r = _pose(ESP, out, 'set', 'C4', *C4_NEAR[:2], '--rot', C4_NEAR[2],
              accept=True)
    return out, _summary(r)


def test_a_rotated_near_touch_is_not_a_stack():
    """Arm 5. The exact grader decides, not the bounding boxes."""
    with tempfile.TemporaryDirectory() as td:
        out, s = _near_touch_board(td)
        assert s['pad_stack_count_after'] == 0, s['pad_stack_pairs_after']
        assert not s.get('refused'), s.get('refused')
        assert _assembly_stacks(out, td) == [], 'check_assembly disagrees'
    print("  C4 at -45 beside Y1: no stack, accepted")


# --------------------------------------------------------------------------
# the run-38 shape, synthetic
# --------------------------------------------------------------------------

def test_run38_shape_is_refused():
    """Arm 6. C15 onto C19, both pad pairs same-net, on a board that also
    carries an inherited different-net conflict (R1/R2) -- so `legal` was
    already False and only `no_worse` could have refused."""
    with tempfile.TemporaryDirectory() as td:
        b = _board(td, [('C15', 10, 25, 180, False),
                        ('C19', C19[0], C19[1], 180, False)],
                   rs=[('R1', 30, 25, 3, 4), ('R2', 30.4, 25, 1, 2)])
        x, y = _on(C19)
        r = _pose(b, os.path.join(td, 'o.kicad_pcb'), 'set', 'C15', x, y,
                  '--rot', '180', refuse='pad_stack_pairs new: C15/C19',
                  code=4)
        s = _summary(r)
        assert s['pad_conflicts_before'] >= 1, 'the inherited conflict'
        # R1/R2's inherited overlap is a stack too (any net); C15/C19 is new.
        assert ['C15', 'C19', 0.3762, 'F'] in s['pad_stack_pairs_after'], \
            s['pad_stack_pairs_after']
        assert ['R1', 'R2'] == [p[:2] for p in s['pad_stack_pairs_after']
                                if p[0] == 'R1'][0], s['pad_stack_pairs_after']
    print("  run-38 C15 onto C19 refused (0.3762 mm2)")


def test_near_snaps_off_the_stack():
    """Arm 7. `--near` at the stacked spot snaps to a pose that stacks
    nothing, and check_assembly agrees on the written board."""
    with tempfile.TemporaryDirectory() as td:
        b = _board(td, [('C15', 10, 25, 180, False),
                        ('C19', C19[0], C19[1], 180, False)])
        x, y = _on(C19)
        out = os.path.join(td, 'o.kicad_pcb')
        # The courtyards (2.6 x 1.6) veto every pose within 1 mm of the
        # stack, so the snap gets a 2.5 mm radius (it lands 2.5 mm away).
        r = _pose(b, out, 'set', 'C15', '--near', x, y, '--rot', '180',
                  '--radius', '2.5', accept=True)
        s = _summary(r)
        assert s.get('snapped'), s.get('snapped')
        assert s['pad_stack_count_after'] == 0, s['pad_stack_pairs_after']
        assert _assembly_stacks(out, td) == [], 'check_assembly disagrees'
        assert _footprint_xy(out, 'C15') != (x, y)
    print("  --near snapped off the stack")


def _stacked(td, dy=OFF[1], extra=()):
    x, y = _on(C19, dy)
    return _board(td, [('C15', x, y, 180, False),
                       ('C19', C19[0], C19[1], 180, False),
                       ('C20', C20[0], C20[1], 180, False)] + list(extra),
                  rs=[('R3', 10, 25, 3, 4)])


def test_a_mixed_refusal_still_explains_the_stack():
    """R2 moved onto R1 (different nets): a new pad conflict AND a new
    stack. The stack sentence must survive beside the other category."""
    with tempfile.TemporaryDirectory() as td:
        b = _board(td, [], rs=[('R1', 30, 25, 3, 4), ('R2', 10, 10, 1, 2)])
        r = _pose(b, os.path.join(td, 'o.kicad_pcb'), 'set', 'R2', 30.4, 25,
                  '--rot', '0', refuse='NOT BUILDABLE', code=4)
        reason = _summary(r)['refused']
        assert 'pad_conflict' in reason and 'pad_stack' in reason, reason
        assert "A pad stack is two parts' pad copper" in reason, reason
    print("  pad conflict + stack: both named, the stack explained")


def test_an_inherited_stack_is_not_charged_but_is_not_legal():
    """Arm 8. An unrelated move on a board that already stacks: no_worse
    (inherited damage is not counted), and legal is False (it is there)."""
    with tempfile.TemporaryDirectory() as td:
        r = _pose(_stacked(td), os.path.join(td, 'o.kicad_pcb'), 'set',
                  'R3', 12, 25, '--rot', '0', accept=True)
        s = _summary(r)
        assert s['no_worse'] is True, s.get('refused')
        assert s['legal'] is False, 'a stacked board read legal'
        assert s['pad_stack_count_after'] == 1, s['pad_stack_pairs_after']
    print("  inherited stack: no_worse true, legal false")


def test_a_shrinking_stack_is_accepted():
    """Arm 9. 0.3762 -> 0.2568 mm2 is an improvement."""
    with tempfile.TemporaryDirectory() as td:
        x, y = _on(C19, 0.6)
        r = _pose(_stacked(td), os.path.join(td, 'o.kicad_pcb'), 'set',
                  'C15', x, y, '--rot', '180', accept=True)
        s = _summary(r)
        assert s['pad_stack_area_after'] < s['pad_stack_area_before'], s
    print("  0.3762 -> 0.2568 accepted")


def test_a_deepening_stack_is_refused():
    """Arm 10. Same pair, same count, MORE overlap: the magnitude arm."""
    with tempfile.TemporaryDirectory() as td:
        x, y = _on(C19)
        r = _pose(_stacked(td, dy=0.6), os.path.join(td, 'o.kicad_pcb'),
                  'set', 'C15', x, y, '--rot', '180',
                  refuse='pad_stack_area', code=4)
        assert 'pad_stack_pairs new' not in _summary(r)['refused']
    print("  0.2568 -> 0.3762 refused on pad_stack_area")


def test_the_area_is_summed_not_maxed():
    """Arm 11. Two stacks; the SMALLER one deepens while the larger holds.
    A max would read the larger both times and accept. Each stack covers
    BOTH pad pairs, so the totals are twice the per-pair areas: C15/C19 2 x
    0.3762, C16/C20 2 x 0.1284 -> 2 x 0.2568."""
    with tempfile.TemporaryDirectory() as td:
        c16_at = _on(C20, 0.9)
        b = _stacked(td, extra=[('C16', c16_at[0], c16_at[1], 180, False)])
        x, y = _on(C20, 0.6)
        r = _pose(b, os.path.join(td, 'o.kicad_pcb'), 'set', 'C16', x, y,
                  '--rot', '180', refuse='pad_stack_area', code=4)
        s = _summary(r)
        assert (s['pad_stack_area_before'],
                s['pad_stack_area_after']) == (1.0092, 1.266), s
    print("  summed 1.0092 -> 1.266 refused")


def test_a_stack_that_spreads_to_more_pads_is_refused():
    """The code reviewer's repro: C15 turned 90 degrees stacks ONE pad on C19
    (0.4225 mm2); moved to the run-38 offset it stacks BOTH (2 x 0.3762).
    Its deepest overlap SHRINKS while more copper is stacked -- the area
    arm sums every stacked pad pair, so it is refused."""
    with tempfile.TemporaryDirectory() as td:
        b = _board(td, [('C15', 19.25, 14.25, 90, False),
                        ('C19', C19[0], C19[1], 0, False)])
        x, y = _on(C19)
        r = _pose(b, os.path.join(td, 'o.kicad_pcb'), 'set', 'C15', x, y,
                  '--rot', '0', refuse='pad_stack_area', code=4)
        s = _summary(r)
        assert s['pad_stack_area_after'] > s['pad_stack_area_before'], s
        assert 'pad_stack_pairs new' not in s['refused'], s['refused']
        assert [p[2] for p in s['pad_stack_pairs_after']] == [0.3762], \
            s['pad_stack_pairs_after']
    print("  one pad 0.4225 -> both pads %.4f refused"
          % s['pad_stack_area_after'])


def test_a_new_stack_is_refused_when_the_totals_tie():
    """Arm 12, #1100's lesson: C15 leaves C19's stack for the same stack on
    C20. Count and area tie; only the PAIR arm can see it."""
    with tempfile.TemporaryDirectory() as td:
        x, y = _on(C20)
        r = _pose(_stacked(td), os.path.join(td, 'o.kicad_pcb'), 'set',
                  'C15', x, y, '--rot', '180',
                  refuse='pad_stack_pairs new: C15/C20', code=4)
        reason = _summary(r)['refused']
        assert 'pad_stack_count' not in reason, reason
        assert 'pad_stack_area' not in reason, reason
    print("  C19 -> C20 refused on the new pair alone")


def test_leaving_a_stack_is_accepted():
    """Arm 13."""
    with tempfile.TemporaryDirectory() as td:
        r = _pose(_stacked(td), os.path.join(td, 'o.kicad_pcb'), 'set',
                  'C15', 10, 20, '--rot', '180', accept=True)
        s = _summary(r)
        assert (s['pad_stack_count_before'],
                s['pad_stack_count_after']) == (1, 0), s
    print("  leaving the stack accepted, 1 -> 0")


# --------------------------------------------------------------------------
# the arms, in process
# --------------------------------------------------------------------------

def test_the_three_arms_and_their_defaults():
    """Arm 14."""
    from placement.pose_ops import worsened, is_clean
    zero = {'pad_stack_count': 0, 'pad_stack_area': 0.0,
            'pad_stack_pairs': []}
    one = {'pad_stack_count': 1, 'pad_stack_area': 0.04,
           'pad_stack_pairs': [['C4', 'Y1', 0.04, 'F']]}
    assert worsened(zero, one) == ['pad_stack_count', 'pad_stack_area',
                                   'pad_stack_pairs'], worsened(zero, one)
    deeper = dict(one, pad_stack_area=0.05,
                  pad_stack_pairs=[['C4', 'Y1', 0.05, 'F']])
    assert worsened(one, deeper) == ['pad_stack_area'], worsened(one, deeper)
    moved = dict(one, pad_stack_pairs=[['C4', 'Y2', 0.04, 'F']])
    assert worsened(one, moved) == ['pad_stack_pairs'], worsened(one, moved)
    # An older report without the pair list leaves the pair arm off, and a
    # report without any stack key reads as zero.
    older = {'pad_stack_count': 1, 'pad_stack_area': 0.04}
    assert worsened(older, moved) == [], worsened(older, moved)
    assert worsened({}, {}) == []
    assert is_clean({'pad_stack_count': 1}) is False
    assert is_clean({'pad_stack_area': 0.04}) is False
    assert is_clean(zero) is True
    print("  count, area and pair arms; missing keys read as zero")


def test_the_census_rows():
    """Two stacks that share a first ref are TWO stacks, summed, in a fixed
    order -- the channel yields pairs sharing a first ref in hash-seed order,
    and no board fixture here puts two such stacks side by side."""
    from unittest.mock import patch
    from placement import legality
    rows = [legality.BodyOverlapPair(a='A', b=b, kind='pad_intersection',
                                     area_mm2=area, side='F', waived=False,
                                     waiver='')
            for b, area in (('C', 0.2), ('B', 0.1))]
    def channel(pcb, clr, totals=None):
        # every stacked pad pair: A/C stacks two pads, A/B one
        totals.update({('A', 'C'): 0.35, ('A', 'B'): 0.1})
        return list(rows)
    with patch.object(legality, 'pad_intersection_pairs', channel):
        c = legality.pad_stack_census(None, 0.2)
    assert c['pad_stack_count'] == 2, c
    assert c['pad_stack_area'] == 0.45, c
    assert c['pad_stack_pairs'] == [['A', 'B', 0.1, 'F'],
                                    ['A', 'C', 0.2, 'F']], c
    print("  A/C and A/B: two stacks, 0.45 mm2 summed over pads, sorted")


def test_opposite_sides_are_not_a_stack():
    """A part on the back under one on the front shares no side: no stack,
    as check_assembly says (`_sides_interact`)."""
    with tempfile.TemporaryDirectory() as td:
        b = _board(td, [('C19', C19[0], C19[1], 180, False),
                        ('C21', 10, 25, 180, False)])
        with open(b, encoding='utf-8') as fh:
            text = fh.read()
        # C21 on the back: its footprint and both pads on B.Cu.
        head, tail = text.split('"C21"', 1)
        head = head[:head.rfind('(footprint')] + head[head.rfind('(footprint'):] \
            .replace('(layer "F.Cu")', '(layer "B.Cu")', 1)
        tail = tail.replace('(layers "F.Cu")', '(layers "B.Cu")', 2)
        with open(b, 'w', encoding='utf-8') as fh:
            fh.write(head + '"C21"' + tail)
        out = os.path.join(td, 'o.kicad_pcb')
        x, y = _on(C19)
        r = _pose(b, out, 'set', 'C21', x, y, '--rot', '180', accept=True)
        s = _summary(r)
        assert s['pad_stack_count_after'] == 0, s['pad_stack_pairs_after']
        assert _assembly_stacks(out, td) == [], 'check_assembly disagrees'
    print("  C21 on B.Cu under C19 on F.Cu: no stack")


def test_the_census_is_check_assemblys():
    """Arm 15. `pad_intersection_pairs` IS grade_body_overlap's
    pad_intersection channel on every board, and the fixtures' counts are
    the ones measured before the lift (an equality of the function with its
    own caller alone would hold for any lift)."""
    from kicad_parser import parse_kicad_pcb
    from placement.legality import (grade_body_overlap,
                                    pad_intersection_pairs,
                                    pad_stack_census)
    from placement.parser import extract_locked_refs
    import routing_defaults
    clr = routing_defaults.CLEARANCE
    boards = [b for b in run_utils.corpus_boards()]
    assert boards, 'git could not list the corpus'
    total = 0
    for b in boards + [run_utils.evidence(PILE), run_utils.evidence(DAMAGED)]:
        pcb = parse_kicad_pcb(b)
        # Whole NamedTuples, compared as tuples (every field, in order).
        lifted = sorted(tuple(p) for p in pad_intersection_pairs(
            pcb, clr, set(extract_locked_refs(b) or ())))
        graded = sorted(tuple(p) for p in grade_body_overlap(
            pcb, clr, pcb_file=b)['pairs'] if p.kind == 'pad_intersection')
        assert lifted == graded, b
        rows = pad_stack_census(pcb, clr)['pad_stack_pairs']
        assert len(rows) == len(lifted), b
        name = os.path.splitext(os.path.basename(b))[0]
        if name in PINNED_STACKS:
            assert len(lifted) == PINNED_STACKS[name], (name, len(lifted))
            sides = {}
            for r in rows:
                sides[r[3]] = sides.get(r[3], 0) + 1
            assert sides == PINNED_SIDES[name], (name, sides)
        else:
            assert len(lifted) == 0, (name, len(lifted))
        total += len(lifted)
    assert total >= 100, total
    print("  %d boards, %d stacks, lift == grade_body_overlap"
          % (len(boards) + 2, total))


def test_scope_names_it_and_main_does_not_grade():
    """Arm 16. `legal_scope` says what it now covers; the measurement stays
    in the engine (test_892_registries' rule)."""
    import ast
    from placement.pose_ops import LEGAL_SCOPE
    assert any('pad stack' in s for s in LEGAL_SCOPE), LEGAL_SCOPE
    from placement.pose_ops import LEGAL_UNMEASURED
    assert any('coincident_origins' in s for s in LEGAL_UNMEASURED), \
        LEGAL_UNMEASURED
    with open(PLACE_POSE, encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    main = next(n for n in tree.body
                if isinstance(n, ast.FunctionDef) and n.name == 'main')
    names = {getattr(c.func, 'id', getattr(c.func, 'attr', None))
             for c in ast.walk(main) if isinstance(c, ast.Call)}
    assert not names & {'pad_intersection_pairs', 'pad_stack_census'}, names
    print("  legal_scope names pad stacks; main() measures nothing itself")


def test_check_floorplan_prints_the_exact_count():
    """Arm 17. Printed, not a rule; the JSON count is check_assembly's."""
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'o.kicad_pcb')
        _pose(ESP, out, 'set', 'C4', C4_STACK_X, C4_Y, '--rot', '0',
              '--force', accept=True)
        intent = os.path.join(td, 'i.json')
        run_utils.check([sys.executable, '-X', 'utf8', FLOORPLAN, ESP,
                         '--emit-intent', intent], accept=True)
        j = os.path.join(td, 'g.json')
        r = run_utils.check([sys.executable, '-X', 'utf8', FLOORPLAN, out,
                             '--intent', run_utils.evidence(intent),
                             '--exit-zero', '--json', j], accept=True)
        assert 'pad stacks: 1 ' in r.stdout, r.stdout[-2000:]
        assert 'C4 <-> Y1' in r.stdout, r.stdout[-2000:]
        s = _summary(r)
        assert s['pad_stack_count'] == len(_assembly_stacks(out, td)) == 1
        assert not any('stack' in k for k in s['violations_by_rule']), s
        with open(run_utils.evidence(j), encoding='utf-8') as fh:
            doc = json.load(fh)
        assert doc['pad_stacks']['pad_stack_pairs'] == [
            ['C4', 'Y1', 0.0412, 'F']], doc['pad_stacks']
    print("  check_floorplan prints 'pad stacks: 1' and C4 <-> Y1")


def test_check_floorplan_is_exact_where_the_box_census_is_not():
    """Arm 18. On the near-touch board the printed count and
    `pad_stack_count` are 0 while the bounding-box `pad_intersection_pairs`
    beside them is 1 -- the line prints the exact census, not the box one,
    and the box key keeps its meaning."""
    with tempfile.TemporaryDirectory() as td:
        out, _s = _near_touch_board(td)
        intent = os.path.join(td, 'i.json')
        run_utils.check([sys.executable, '-X', 'utf8', FLOORPLAN, ESP,
                         '--emit-intent', intent], accept=True)
        r = run_utils.check([sys.executable, '-X', 'utf8', FLOORPLAN, out,
                             '--intent', run_utils.evidence(intent),
                             '--exit-zero'], accept=True)
        assert 'pad stacks: 0 ' in r.stdout, r.stdout[-2000:]
        s = _summary(r)
        assert s['pad_stack_count'] == 0, s['pad_stack_count']
        assert s['pad_intersection_pairs'] == 1, (
            'the fixture no longer separates the two censuses: %r'
            % s['pad_intersection_pairs'])
    print("  near-touch: printed 0, box census 1")


def test_check_floorplan_lists_five_and_counts_the_rest():
    """tigard_damaged carries 29 stacks: five are listed, the rest counted,
    and the line says check_assembly grades them NOT BUILDABLE."""
    with tempfile.TemporaryDirectory() as td:
        intent = os.path.join(td, 'i.json')
        run_utils.check([sys.executable, '-X', 'utf8', FLOORPLAN, DAMAGED,
                         '--emit-intent', intent], accept=True)
        r = run_utils.check([sys.executable, '-X', 'utf8', FLOORPLAN,
                             DAMAGED, '--intent', run_utils.evidence(intent),
                             '--exit-zero'], accept=True)
    lines = r.stdout.splitlines()
    (i,) = [k for k, ln in enumerate(lines) if 'pad stacks: 29 ' in ln]
    assert lines[i].endswith('check_assembly grades these NOT BUILDABLE'), \
        lines[i]
    assert all(' <-> ' in ln for ln in lines[i + 1:i + 6]), lines[i:i + 7]
    assert lines[i + 6].strip() == '... 24 more', lines[i + 6]
    print("  29 stacks: five listed, '... 24 more', NOT BUILDABLE said")


TESTS = [
    test_the_issue_repro_is_refused,
    test_force_writes_and_check_assembly_agrees,
    test_a_kiss_is_legal,
    test_the_boundary_is_check_assemblys,
    test_a_rotated_near_touch_is_not_a_stack,
    test_run38_shape_is_refused,
    test_near_snaps_off_the_stack,
    test_a_mixed_refusal_still_explains_the_stack,
    test_an_inherited_stack_is_not_charged_but_is_not_legal,
    test_a_shrinking_stack_is_accepted,
    test_a_deepening_stack_is_refused,
    test_the_area_is_summed_not_maxed,
    test_a_stack_that_spreads_to_more_pads_is_refused,
    test_a_new_stack_is_refused_when_the_totals_tie,
    test_leaving_a_stack_is_accepted,
    test_the_three_arms_and_their_defaults,
    test_the_census_rows,
    test_opposite_sides_are_not_a_stack,
    test_the_census_is_check_assemblys,
    test_scope_names_it_and_main_does_not_grade,
    test_check_floorplan_prints_the_exact_count,
    test_check_floorplan_is_exact_where_the_box_census_is_not,
    test_check_floorplan_lists_five_and_counts_the_rest,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        # A filter that names no case passes nothing: a mutation battery
        # witness spelled wrong would otherwise read every row as SURVIVED.
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
