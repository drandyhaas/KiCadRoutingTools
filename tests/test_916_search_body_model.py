#!/usr/bin/env python3
"""#916: the SEARCH seats against `placement.body`, behind `body_model`.

#896 wired every GRADING consumer to `placement/body.py` and deliberately left the
SEARCH on an inlined `courtyard -> pad bbox` ladder. So on a library that draws no
courtyard the optimizer seated parts against PAD BOXES while every instrument that
graded the result saw drawn bodies -- and `pose_ok` reads those baked bounds, so the
divergence moves which basin the anneal lands in, not merely what a report says.

Two things have to be true and they pull in opposite directions, which is why both
are asserted here:

* **OFF is the OLD CODE.** `body_model=False` must take the inlined ladder, not a
  re-derivation through `placement.body` that happens to agree. The A/B's control arm
  is only a control if it is untouched, and "it agrees on the corpus I tried" is not
  the same claim.
* **ON must actually MOVE something**, on a board where #896 measured that it should.
  A flag that changes no rectangle would sail through an A/B as "no regression" while
  fixing nothing -- the failure `test_896_one_body_implementation.py` calls a
  ride-along.

Run: python3 -X utf8 tests/test_916_search_body_model.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 900
RUN_ALL_FAST_OK = True

#: esp_prog is the board #896 was filed from: 0 of its 21 footprints draw a
#: courtyard, so today EVERY part is seated against a pad box. tigard draws
#: courtyards widely and is the control for "a courtyard board barely moves".
BOARDS = ('esp_prog.kicad_pcb', 'tigard.kicad_pcb')

TESTS = []


def _board(name):
    return run_utils.evidence(os.path.join(ROOT, 'kicad_files', name),
                              'corpus board')


def _state(name, body_model):
    from kicad_parser import parse_kicad_pcb
    from placement.quench import QuenchState
    path = _board(name)
    pcb = parse_kicad_pcb(path)
    return QuenchState(pcb, path, 0.2, 0.55, 30.0, 0.5, 0.15, 2.0, 2.0, 2.0,
                       0.1, 0.3, body_model=body_model)


def _bounds(state):
    return {ref: dict(p.bounds_by_rot) for ref, p in state.parts.items()}


def test_off_is_the_inlined_ladder_not_a_rederivation():
    """OFF must equal the ladder computed directly, rect for rect."""
    from kicad_parser import parse_kicad_pcb
    from placement.parser import courtyard_for_side, extract_courtyard_sides
    from placement.utility import compute_footprint_bbox_local
    from placement.legality import rotate_local_bounds
    from placement.quench import ROTATIONS
    for name in BOARDS:
        path = _board(name)
        pcb = parse_kicad_pcb(path)
        courtyards = extract_courtyard_sides(path)
        st = _state(name, False)
        bad = []
        for ref, part in st.parts.items():
            fp = pcb.footprints.get(ref)
            if fp is None:
                continue
            lb = courtyard_for_side(courtyards.get(ref), part.side)
            if lb is None:
                lb = compute_footprint_bbox_local(fp)
            for r in ROTATIONS:
                want = rotate_local_bounds(*lb, r)
                got = part.bounds_by_rot.get(r)
                if got is None or max(abs(a - b) for a, b in zip(got, want)) > 1e-9:
                    bad.append((ref, r, got, want))
        assert not bad, (
            '%s: body_model=False is NOT the inlined ladder on %d (ref, rot); '
            'e.g. %r' % (name, len(bad), bad[:3]))
        print('  %-22s OFF == inlined ladder for %d parts'
              % (name, len(st.parts)))


TESTS.append(test_off_is_the_inlined_ladder_not_a_rederivation)


def test_on_grows_boxes_on_a_courtyardless_board_and_shrinks_none():
    """`occupancy_local` is monotone: it may grow a box, never shrink one.

    Monotonicity is the whole safety argument for arming this in a search --
    a shrinking box would admit poses the grader then flags.
    """
    off = _state('esp_prog.kicad_pcb', False)
    on = _state('esp_prog.kicad_pcb', True)
    grew, shrank = [], []
    for ref, p_off in off.parts.items():
        p_on = on.parts.get(ref)
        if p_on is None:
            continue
        a = p_off.bounds_by_rot[0.0]
        b = p_on.bounds_by_rot[0.0]
        area_a = (a[2] - a[0]) * (a[3] - a[1])
        area_b = (b[2] - b[0]) * (b[3] - b[1])
        if area_b > area_a + 1e-9:
            grew.append((ref, round(area_a, 3), round(area_b, 3)))
        elif area_b < area_a - 1e-9:
            shrank.append((ref, round(area_a, 3), round(area_b, 3)))
    assert not shrank, (
        'esp_prog: occupancy_local SHRANK %d part(s) -- it is meant to be '
        'monotone, and a shrinking seat box admits poses the grader flags: %r'
        % (len(shrank), shrank[:5]))
    assert grew, (
        'esp_prog: body_model=True changed NO part box. On the board #896 was '
        'filed from (0 of 21 footprints draw a courtyard) that means the flag '
        'is inert and an A/B on it would measure nothing.')
    print('  esp_prog: %d part(s) grew, 0 shrank; largest %s %s -> %s mm2'
          % (len(grew), *max(grew, key=lambda t: t[2] - t[1])))


TESTS.append(test_on_grows_boxes_on_a_courtyardless_board_and_shrinks_none)


def test_on_is_occupancy_not_the_bare_body():
    """The seat box must be `occupancy_local`, never `body_local`.

    They differ exactly where a pad sticks out past the drawn body, which is
    the case that matters: `legality.part_local_bounds` takes occupancy for
    the same reason, so taking the bare body here would re-open the
    grader/enforcer divergence #916 exists to close -- in the SHRINKING
    direction.
    """
    from kicad_parser import parse_kicad_pcb
    from placement import body as B
    path = _board('esp_prog.kicad_pcb')
    pcb = parse_kicad_pcb(path)
    bodies = B.board_bodies(pcb, path)
    on = _state('esp_prog.kicad_pcb', True)
    differ = 0
    for ref, geom in bodies.items():
        part = on.parts.get(ref)
        if part is None or geom.occupancy_local is None:
            continue
        got = part.bounds_by_rot[0.0]
        want = geom.occupancy_local
        assert max(abs(a - b) for a, b in zip(got, want)) <= 1e-9, (
            'esp_prog %s: seat box %r is not occupancy_local %r'
            % (ref, got, want))
        if (geom.body_local is not None
                and max(abs(a - b) for a, b
                        in zip(geom.body_local, geom.occupancy_local)) > 1e-9):
            differ += 1
    assert differ >= 1, (
        'on esp_prog no part has body_local != occupancy_local, so this test '
        'cannot tell the two apart and asserts nothing about which was used')
    print('  esp_prog: seat box is occupancy_local (%d part(s) where it '
          'differs from body_local)' % differ)


TESTS.append(test_on_is_occupancy_not_the_bare_body)


def test_fab_rect_is_unchanged_by_the_flag():
    """Containment is a DIFFERENT question and must not follow the seat box.

    `QuenchState.fab_rect`'s own docstring: a courtyard is body + margin +
    shell overhang, the corpus ships frac-1.0 courtyard containment on four
    healthy boards against zero non-exempt fab containment, and "a
    courtyard-based containment test is a false-veto machine".
    `occupancy_local` IS courtyard-based whenever a courtyard is drawn, so
    #916 deliberately stops at the seat box.
    """
    for name in BOARDS:
        off = _state(name, False)
        on = _state(name, True)
        for ref in sorted(off.parts):
            a = off.fab_rect(ref)
            b = on.fab_rect(ref)
            if a is None and b is None:
                continue
            assert a is not None and b is not None, (
                '%s %s: fab_rect presence changed with body_model (%r vs %r)'
                % (name, ref, a, b))
            assert max(abs(x - y) for x, y in zip(a, b)) <= 1e-9, (
                '%s %s: fab_rect moved with body_model: %r -> %r'
                % (name, ref, a, b))
        print('  %-22s fab_rect identical under both arms' % name)


TESTS.append(test_fab_rect_is_unchanged_by_the_flag)


def main():
    failures = 0
    for fn in TESTS:
        print('%s ...' % fn.__name__)
        try:
            fn()
        except AssertionError as exc:
            failures += 1
            print('FAIL %s: %s' % (fn.__name__, exc))
    if failures:
        print('%d/%d checks FAILED' % (failures, len(TESTS)))
        return 1
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
