#!/usr/bin/env python3
"""A rip retracts and its replacement grows (#946 item 13, #1022).

A rip was two frames of red at `rip_hold` and then the copper was gone -- the
least legible way to show the single event the movie exists to explain.

The channel a movie has and a still does not is **time**, and direction
survives every colour deficiency there is: copper retracting toward its anchor
and copper growing out of it are opposite motions, readable with no colour at
all. This is not an alternative to the dash (#1013) -- the dash is the floor,
it makes a STILL frame legible, and it ships regardless.

What this file pins:

  * **frame COUNT goes up, frame SIZE does not.** That is the whole risk of a
    motion feature in this subsystem: Pillow does not raise on a GIF whose
    frames differ in size, it silently resizes every later frame to the first;
  * **it retracts from the FAR END, monotonically.** Shrinking every doomed
    segment toward its own midpoint is much easier and reads as the copper
    dissolving, which is not what happened. Asserted on LENGTH: each stage
    holds strictly less than the one before, the last holds none, and every
    stage's copper lies ON the original segments;
  * **growth is the mirror image** -- strictly increasing, ending at the whole
    thing, so the live state and the final frame cannot disagree;
  * **`--rip-hold 0` still cuts.** The old behaviour is a documented arm, not
    an accident, and it is what a caller asks for when a film must not grow;
  * **only RESTORES grow.** A plain `new` add happens thousands of times in a
    film; animating each one would make every movie four times longer for an
    event that has no counterpart to be confused with;
  * **`marks` still bracket each step**, because a composer reads beat
    boundaries off them and a motion that ran outside its own beat would
    caption the wrong frames.
"""
import os
import sys
import tempfile

RUN_ALL_TIMEOUT = 600

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image                                       # noqa: F401
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import animate_route as A          # noqa: E402
import copper_motion as CM         # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')

#: A four-segment chain running away from the origin, all on layer 0 --
#: DELIBERATELY SCRAMBLED and written end-for-end.
#:
#: The tidy version of this fixture is a trap: rows already in anchor order
#: with their near end first make `order_from` a no-op, so a mutant that drops
#: the ordering entirely passes. Measured -- `copper-dissolves-instead-of-
#: retracting` survived the first version of this file. A file order is
#: whatever the writer emitted, so this is also the realistic case.
#: The lengths are 7 / 11 / 13 / 9, which do NOT divide the total evenly at
#: n = 4 -- so every keep-length falls INSIDE a segment and the frontier CUT
#: runs. A 4x10 chain puts every boundary on a joint, so the cut never
#: executes and a mutant that drops it passes: the phase-12 verifier measured
#: exactly that, on the two properties this module argues hardest for.
CHAIN = [[18.0, 0.0, 7.0, 0.0, 0.2, 0],
         [0.0, 0.0, 7.0, 0.0, 0.2, 0],
         [40.0, 0.0, 31.0, 0.0, 0.2, 0],
         [18.0, 0.0, 31.0, 0.0, 0.2, 0]]
#: Copper that stays, touching the chain's near end.
LIVE = [[-5.0, 0.0, 0.0, 0.0, 0.2, 0]]

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _len(rows):
    return sum(((r[2] - r[0]) ** 2 + (r[3] - r[1]) ** 2) ** 0.5 for r in rows)


def _on_chain(rows, tol=1e-6):
    """Every endpoint lies on the original chain (here, the x axis)."""
    return all(abs(r[1]) < tol and abs(r[3]) < tol
               and -tol <= min(r[0], r[2]) and max(r[0], r[2]) <= 40 + tol
               for r in rows)


def test_it_retracts_from_the_far_end():
    _mark = len(_FAIL)
    plan = CM.stages(CHAIN, live=LIVE, n=4, grow=False)
    if len(plan) != 4:
        fail('asked for 4 stages, got %d' % len(plan))
        return
    lens = [_len(st) for st in plan]
    total = _len(CHAIN)
    if any(b >= a for a, b in zip(lens, lens[1:])):
        fail('a retraction stage did not shrink: %s' % lens)
    if lens[-1] > 1e-9:
        fail('the last stage still holds %.3f mm -- the copper is gone, and '
             'that frame IS the event' % lens[-1])
    if abs(lens[0] - total * 0.75) > 1e-6:
        fail('stage 0 holds %.3f mm, expected %.3f' % (lens[0], total * 0.75))
    # THE FRONTIER IS CUT, not dropped: at least one stage must end inside a
    # segment rather than on a joint, or the cut is never executed and a
    # mutant that removes it passes.
    joints = {0.0, 7.0, 18.0, 31.0, 40.0}
    cuts = [i for i, st in enumerate(plan) if st
            and round(max(max(r[0], r[2]) for r in st), 6) not in joints]
    if not cuts:
        fail('BROKEN FIXTURE: every stage ends on a segment joint %s, so the '
             'frontier cut never runs' % sorted(joints))
    else:
        print('    stage(s) %s end mid-segment, so the frontier is CUT' % cuts)
    # the FAR end goes first: the anchor is at x=0 (LIVE touches it there), so
    # what survives is the near part
    for i, st in enumerate(plan):
        if not _on_chain(st):
            fail('stage %d left the original copper: %s' % (i, st))
        far = max([max(r[0], r[2]) for r in st] or [0.0])
        near = min([min(r[0], r[2]) for r in st] or [0.0])
        if st and near > 1e-6:
            fail('stage %d starts at x=%.2f -- it retracted from the NEAR end'
                 % (i, near))
        if st and far > 40.0 * (1 - i / 4.0) + 1e-6:
            fail('stage %d still reaches x=%.2f' % (i, far))
    print('    lengths %s of %.0f mm, all on the original chain'
          % ([round(v, 1) for v in lens], total))
    if len(_FAIL) == _mark:
        print('  PASS: it pulls back toward the copper that stays')


def test_growth_is_the_mirror_image():
    _mark = len(_FAIL)
    plan = CM.stages(CHAIN, live=LIVE, n=4, grow=True)
    lens = [_len(st) for st in plan]
    total = _len(CHAIN)
    if any(b <= a for a, b in zip(lens, lens[1:])):
        fail('a growth stage did not grow: %s' % lens)
    if abs(lens[-1] - total) > 1e-6:
        fail('growth ends at %.3f mm, not the whole %.3f -- the live state '
             'and the final frame would disagree' % (lens[-1], total))
    for i, st in enumerate(plan):
        if not _on_chain(st):
            fail('growth stage %d left the original copper' % i)
    # and the two are opposites, which is the claim that makes the motion
    # readable without colour
    back = [_len(st) for st in CM.stages(CHAIN, live=LIVE, n=4, grow=False)]
    if [round(a + b, 6) for a, b in zip(lens, back)] != [round(total, 6)] * 4:
        fail('retract and grow are not complementary: %s + %s is not %.1f at '
             'every stage' % (lens, back, total))
    print('    lengths %s of %.0f mm' % ([round(v, 1) for v in lens], total))
    if len(_FAIL) == _mark:
        print('  PASS: opposite motions, same geometry')


def test_the_degenerate_cases_refuse_rather_than_invent():
    _mark = len(_FAIL)
    if CM.stages([], n=4) != []:
        fail('empty copper produced stages')
    if CM.stages(CHAIN, n=1) != []:
        fail('n=1 produced a "motion" -- one stage is not a motion')
    if CM.stages(CHAIN, n=0) != []:
        fail('n=0 produced stages')
    # zero-length copper has no direction to move in
    stitch = [[5.0, 5.0, 5.0, 5.0, 0.2, 0]]
    if CM.stages(stitch, n=4) != []:
        fail('coincident endpoints were given a direction they do not have')
    # no live copper at all: an anchor is still chosen, deterministically
    a1 = CM.anchor_for(CHAIN)
    a2 = CM.anchor_for(CHAIN)
    if a1 is None or a1 != a2:
        fail('the anchor is not deterministic without live copper: %r / %r'
             % (a1, a2))
    if CM.anchor_for([]) is not None:
        fail('an anchor was invented for no copper')
    # the anchor IS the end nearest the surviving copper
    if CM.anchor_for(CHAIN, LIVE) != (0.0, 0.0):
        fail('the anchor is not the end nearest the live copper: %r'
             % (CM.anchor_for(CHAIN, LIVE),))
    if len(_FAIL) == _mark:
        print('  PASS: nothing is invented where there is no direction')


def _film(rip_hold, marks=None):
    """A two-board chain whose second board RIPS copper, so `reveal_delta`
    takes the remove path."""
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(BOARD)
    rows_s, rows_v = A._board_rows(pcb, list(pcb.board_info.copper_layers))
    r, layers = A._renderer(BOARD, None, 200, 1, 150)
    m = A.Movie(r, layers, rip_hold=rip_hold)
    m.snapshot('input')
    m.reveal_delta(rows_s, rows_v, 'step1 route', chunks=2)
    # now take a slice of it away -- a rip
    m.reveal_delta(rows_s[:-60], rows_v, 'step2 reroute', chunks=2)
    # and put it back, as a RESTORE
    m.add(rows_s[-60:], [], 'reroute', 'step3 reroute')
    return m


def test_frame_count_goes_up_and_frame_size_does_not():
    _mark = len(_FAIL)
    cut = _film(0)
    moved = _film(2)
    if len(moved.frames) <= len(cut.frames):
        fail('motion added no frames: %d vs %d (cut)'
             % (len(moved.frames), len(cut.frames)))
    for name, m in (('cut', cut), ('motion', moved)):
        sizes = {f.size for f in m.frames}
        if len(sizes) != 1:
            fail('%s: %d frame sizes %s -- Pillow does NOT raise on this, it '
                 'silently resizes every later frame to the first'
                 % (name, len(sizes), sizes))
    print('    --rip-hold 0 -> %d frames, --rip-hold 2 -> %d, one size each'
          % (len(cut.frames), len(moved.frames)))
    if len(_FAIL) == _mark:
        print('  PASS: more frames, same size')


def test_rip_hold_zero_still_cuts():
    """The old behaviour is a documented arm, not an accident."""
    _mark = len(_FAIL)
    cut = _film(0)
    if cut.motion:
        fail('--rip-hold 0 left motion on')
    # and the motion path is genuinely what makes the difference: same film,
    # motion forced off by the same switch
    moved = _film(2)
    if not moved.motion:
        fail('--rip-hold 2 did not turn motion on')
    if len(cut.frames) >= len(moved.frames):
        fail('the cut arm is not shorter: %d vs %d'
             % (len(cut.frames), len(moved.frames)))
    if len(_FAIL) == _mark:
        print('  PASS: the cut is still available and still cuts')


def test_only_restores_grow():
    _mark = len(_FAIL)
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(BOARD)
    rows_s, _rv = A._board_rows(pcb, list(pcb.board_info.copper_layers))
    r, layers = A._renderer(BOARD, None, 200, 1, 150)
    plain = A.Movie(r, layers, rip_hold=2)
    plain.add(rows_s[:80], [], 'route', 'new copper')
    rest = A.Movie(r, layers, rip_hold=2)
    rest.add(rows_s[:80], [], 'reroute', 'restored copper')
    if len(plain.frames) != 1:
        fail('a plain add drew %d frames -- animating every add makes every '
             'movie four times longer' % len(plain.frames))
    if len(rest.frames) <= 1:
        fail('a restore drew %d frame(s) -- it did not grow'
             % len(rest.frames))
    print('    plain add -> %d frame, restore -> %d frames'
          % (len(plain.frames), len(rest.frames)))
    if len(_FAIL) == _mark:
        print('  PASS: the event with a counterpart moves; the rest cuts')


def test_marks_still_bracket_each_step():
    """A composer reads beat boundaries off `marks`; a motion running outside
    its own beat would caption the wrong frames."""
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        import shutil
        b1 = os.path.join(td, 'step1.kicad_pcb')
        shutil.copyfile(BOARD, b1)
        steps = [('step1 route', BOARD, None), ('step2 route', b1, None)]
        marks = []
        frames = A.build_boards(steps, b1, 200, 1, 150, 2, 4, marks=marks)
        if not frames:
            fail('no frames')
            return
        if [m[1] for m in marks] != [s[1] for s in steps]:
            fail('marks do not name the steps in order')
        for label, _board, first, last in marks:
            if not (0 <= first <= last <= len(frames)):
                fail('%s: mark [%d, %d) outside 0..%d'
                     % (label, first, last, len(frames)))
        if len({f.size for f in frames}) != 1:
            fail('the film has %d sizes' % len({f.size for f in frames}))
        print('    %d frames, %d marks, all in range, one size'
              % (len(frames), len(marks)))
    if len(_FAIL) == _mark:
        print('  PASS: the beats still bracket their own frames')


def test_a_growth_stage_is_not_drawn_over_its_finished_self():
    """`_frame(base_s=...)` exists for exactly one reason, and it is invisible
    to every count-based check.

    A growth stage draws the partial copper as a HIGHLIGHT over a base. If the
    base is the live state -- which already holds the whole restored net,
    because `add` inserts before it draws -- then the finished copper sits
    under every stage and the growth is not visible at all. The frame count,
    the frame size and the reported stages are identical either way, so this is
    asserted on INK: each growth frame must carry strictly more than the one
    before it.
    """
    _mark = len(_FAIL)
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(BOARD)
    layers = list(pcb.board_info.copper_layers)
    rows_s, _rv = A._board_rows(pcb, layers)
    # ONE layer, so the ink being counted is unambiguous.
    one = [r for r in rows_s if int(r[5]) == 0][:300]
    if len(one) < 40:
        fail('BROKEN FIXTURE: only %d segment(s) on the first layer'
             % len(one))
        return
    r, ls = A._renderer(BOARD, None, 260, 1, 150)
    m = A.Movie(r, ls, rip_hold=2)
    m.add(one, [], 'reroute', 'restored')
    if len(m.frames) < 3:
        fail('a restore drew %d frame(s); there is nothing to compare'
             % len(m.frames))
        return
    col = m.theme.rgb('event_restored')
    ink = []
    for f in m.frames:
        px = f.convert('RGB')
        ink.append(sum(n for n, c in px.getcolors(1 << 20) if c == col))
    if any(b <= a for a, b in zip(ink, ink[1:])):
        fail('the growth stages do not gain ink: %s -- the finished copper is '
             'drawn UNDER every stage, so the growth is invisible' % ink)
    else:
        print('    restore ink per frame: %s' % ink)
    if ink[0] * 2 > ink[-1]:
        fail('the first stage already carries %d of the final %d -- it is not '
             'growing from an anchor' % (ink[0], ink[-1]))
    # AND THE BASE. The highlight ink above grows either way; what `base_s`
    # decides is whether the FINISHED copper is already drawn underneath in
    # its LAYER colour. With the base excluded it appears only in the final
    # frame; without it, it is there from stage one and the growth is a
    # highlight crawling over a net that is visibly already complete.
    lay = r.palette.get(ls[0])
    base_ink = [sum(n for n, c in f.convert('RGB').getcolors(1 << 20)
                    if c == lay) for f in m.frames]
    if base_ink[0] * 2 > max(base_ink):
        fail('the restored net is already drawn in its layer colour at stage '
             '0 (%s) -- the growth stages have their finished self underneath '
             'them' % base_ink)
    else:
        print('    layer-colour ink per frame: %s' % base_ink)
    if len(_FAIL) == _mark:
        print('  PASS: each stage is drawn on what existed, not on what will')


def test_the_rows_are_oriented_near_end_first():
    """A fraction of an ORIENTED segment is the part still attached to the
    anchor; a fraction of an unoriented one is a stub floating wherever the
    file happened to spell the endpoints.

    Asserted directly, because the fixture used to be spelled near-end-first
    already and the flip never ran -- a mutant that removed it passed every
    gate.
    """
    _mark = len(_FAIL)
    a = CM.anchor_for(CHAIN, LIVE)
    ordered = CM.order_from(CHAIN, a)
    if len(ordered) != len(CHAIN):
        fail('order_from lost a row: %d of %d' % (len(ordered), len(CHAIN)))
    flipped = sum(1 for r in ordered if list(r) not in [list(c) for c in CHAIN])
    if not flipped:
        fail('BROKEN FIXTURE: nothing was flipped, so the orientation rule is '
             'not exercised at all')
    ax, ay = a
    for r in ordered:
        d0 = ((r[0] - ax) ** 2 + (r[1] - ay) ** 2) ** 0.5
        d1 = ((r[2] - ax) ** 2 + (r[3] - ay) ** 2) ** 0.5
        if d0 > d1 + 1e-9:
            fail('a row is spelled far-end-first: %s (anchor %s)' % (r, a))
    # and nearest-first overall
    ds = [((r[0] - ax) ** 2 + (r[1] - ay) ** 2) ** 0.5 for r in ordered]
    if ds != sorted(ds):
        fail('the rows are not nearest-first from the anchor: %s'
             % [round(v, 1) for v in ds])
    else:
        print('    %d of %d rows flipped; distances %s'
              % (flipped, len(CHAIN), [round(v, 1) for v in ds]))
    if len(_FAIL) == _mark:
        print('  PASS: near end first, nearest first')


def test_a_trunk_passing_through_is_not_filtered_away():
    """`_near_live` bounds the anchor search, and its bound must not change
    the answer.

    The first version admitted a live segment only when one of its ENDPOINTS
    fell in the neighbourhood, while the justification is about DISTANCE. A
    T-junction -- a long trunk with a stub branching mid-span -- then had its
    trunk dropped, the anchor fell back to the centroid rule and landed on the
    STUB'S OWN MIDDLE, and the stub retracted from both ends leaving a
    detached floating piece.
    """
    _mark = len(_FAIL)
    r, layers = A._renderer(BOARD, None, 160, 1, 150)
    m = A.Movie(r, layers, rip_hold=1)
    trunk = [-30.0, 0.0, 30.0, 0.0, 0.2, 0]          # both ends far outside
    stub = [[0.0, 0.0, 0.0, 3.0, 0.2, 0],
            [0.0, 3.0, 0.0, 6.0, 0.2, 0]]
    from route_trace import _Seg, seg_key_row
    for row in [trunk] + stub:
        m.live_s[seg_key_row(row)] = _Seg(row, layers)
    doomed = [seg_key_row(row) for row in stub]
    near = m._near_live([m._row(m.live_s[k]) for k in doomed],
                        exclude=set(doomed))
    if not near:
        fail('the trunk was filtered away -- it passes straight through the '
             'stub\'s neighbourhood and both its ends are outside the box')
        return
    anchor = CM.anchor_for(stub, near)
    if anchor != (0.0, 0.0):
        fail('the anchor is %s, not the junction (0.0, 0.0) -- the stub would '
             'retract to its own middle and leave a floating piece' % (anchor,))
    else:
        print('    trunk kept (%d row(s)); anchor at the junction %s'
              % (len(near), anchor))
    # and the filter still BOUNDS: copper far away is not admitted
    far = [900.0, 900.0, 910.0, 900.0, 0.2, 0]
    m.live_s[seg_key_row(far)] = _Seg(far, layers)
    near2 = m._near_live([m._row(m.live_s[k]) for k in doomed],
                         exclude=set(doomed))
    if any(abs(row[0] - 900.0) < 1e-6 for row in near2):
        fail('the filter admitted copper 900 mm away -- it bounds nothing')
    if len(_FAIL) == _mark:
        print('  PASS: a trunk through the neighbourhood is in, distant '
              'copper is out')


TESTS = (
    test_it_retracts_from_the_far_end,
    test_the_rows_are_oriented_near_end_first,
    test_a_trunk_passing_through_is_not_filtered_away,
    test_growth_is_the_mirror_image,
    test_the_degenerate_cases_refuse_rather_than_invent,
    test_frame_count_goes_up_and_frame_size_does_not,
    test_rip_hold_zero_still_cuts,
    test_only_restores_grow,
    test_marks_still_bracket_each_step,
    test_a_growth_stage_is_not_drawn_over_its_finished_self,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for m in _FAIL:
            print('  - %s' % m)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
