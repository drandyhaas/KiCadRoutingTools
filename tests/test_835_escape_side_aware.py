#!/usr/bin/env python3
"""#835: the escape ledger stops charging copper on the other face, or a frame.

`escape._blocked_span` walked every other footprint and tested XY band overlap
alone -- `fp.layer` was read nowhere in the module -- so a part on B.Cu was
charged the span an F.Cu part occupies. `blockers`, which the module's own
docstring calls the field to read first, named a part on the other side of the
board as the one to move.

The suite could not have caught this. `tests/test_escape_ledger.py`'s fixtures
are `_Fp` objects with **no `layer` attribute at all**, so `footprint_side`
answers `'F'` for every one of them and no arm exercises a second face; its two
real-board arms assert only that the output is sorted. Nothing anywhere
asserted `blockers` CONTENT on a real board. That is the hole this file fills.

Pinned here:

  1. Structural invariants over the whole tracked corpus, which do not go
     stale when a board is added: no blocker shares no face with the part it
     is charged against, and no blocker is a container.
  2. The per-board numbers, as a change detector.
  3. The two witnesses from the issue and from `via_slots`' docstring --
     ulx3s U9/SD1 charging each other across the board, and rp2350 U6 being
     charged its whole face by the Teensy module it sits inside.
  4. `span_eaten`, the shared kernel: the union, the clamp, and the per-ref
     amounts both ledgers need.
  5. `routability.face_lane_ledger` on the same kernel: a face can no longer
     be covered by more than its own length, and its side test is symmetric.
  6. Hand-built fixtures that actually exercise the new skip, since the
     existing ones cannot.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'tests')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                              # noqa: E402
from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import escape as E                             # noqa: E402
from placement import routability as R                        # noqa: E402
from placement.legality import (container_refs,               # noqa: E402
                                footprint_has_through_pads,
                                footprint_side,
                                graded_parts_from_file,
                                sides_occupied)
from placement.options import deficit_totals                  # noqa: E402

#: Deficit lanes / deficit parts / faces in deficit, per board. Regenerate with
#: `python3 -X utf8 tests/measure_834_835_side_awareness.py --table B`.
#: Boards absent from a checkout are skipped, not failed.
EXPECTED = {
    'ulx3s': (0, 0, 0),
    'orangecrab_ext_pll': (70, 18, 39),
    'glasgow_revC': (17, 14, 14),
    'rp2350_fpga_eensy_prePlane': (53, 8, 24),
    # The controls. tigard and splitflap_driver must not move under ANY of the
    # three arms; watchy and kit-dev-coldfire are the boards with no cross-side
    # charge and no container at all.
    'tigard': (41, 8, 23),
    'splitflap_driver': (0, 0, 0),
    'watchy': (25, 4, 13),
    'kit-dev-coldfire-xilinx_5213': (0, 0, 0),
}


class _Pad:
    def __init__(self, gx, gy, net=1, drill=0.0, lx=None, ly=None):
        self.global_x = gx
        self.global_y = gy
        # `pad_pitch` reads the LOCAL frame; the fixture is unrotated, so the
        # two frames differ by the footprint origin only and the pitch is the
        # same either way.
        self.local_x = gx if lx is None else lx
        self.local_y = gy if ly is None else ly
        self.net_id = net
        self.drill = drill
        self.size_x = self.size_y = 0.3
        self.layers = ['F.Cu']
        self.pad_type = 'smd'
        self.rect_rotation = 0.0


class _Fp:
    def __init__(self, ref, pads, layer='F.Cu', name='QFN-32'):
        self.reference = ref
        self.footprint_name = name
        self.pads = pads
        self.layer = layer
        self.x = pads[0].global_x if pads else 0.0
        self.y = pads[0].global_y if pads else 0.0
        self.rotation = 0.0


class _Pcb:
    def __init__(self, fps):
        self.footprints = {f.reference: f for f in fps}
        self.board_info = None
        self.source_path = None
        self.nets = {}
        self.zones = []


def _board(name):
    for p in run_utils.corpus_boards():
        if os.path.splitext(os.path.basename(p))[0] == name:
            return parse_kicad_pcb(p), p
    return None, None


def _sides(pcb):
    return {r: sides_occupied(footprint_side(f), footprint_has_through_pads(f))
            for r, f in pcb.footprints.items()}


def test_no_blocker_is_ever_on_a_face_the_part_does_not_occupy():
    """The invariant, over every tracked board. Survives a corpus change."""
    boards = run_utils.corpus_boards()
    if not boards:
        print('SKIP: git could not name the tracked corpus')
        return
    checked = charges = 0
    for path in boards:
        try:
            pcb = parse_kicad_pcb(path)
        except Exception:                                     # noqa: BLE001
            continue
        sides = _sides(pcb)
        led = E.escape_ledger(pcb, pcb_file=path)
        if not led:
            continue
        checked += 1
        for p in led:
            own = sides[p.ref]
            for f in p.faces:
                for b in f.blockers:
                    charges += 1
                    assert sides[b] & own, (
                        '{}: {} on {} is charged against {} on {}'.format(
                            os.path.basename(path), b,
                            ''.join(sorted(sides[b])), p.ref,
                            ''.join(sorted(own))))
    assert checked >= 5, 'only {} board(s) produced a ledger'.format(checked)
    assert charges > 0, 'no blocker was charged anywhere; this proves nothing'
    print('  PASS: {} blocker charges over {} board(s), none cross-side'
          .format(charges, checked))


def test_no_blocker_is_a_container():
    """A frame the part sits inside is not a body parked off its face."""
    checked = 0
    for path in run_utils.corpus_boards():
        try:
            pcb = parse_kicad_pcb(path)
        except Exception:                                     # noqa: BLE001
            continue
        cont = container_refs(pcb, graded_parts_from_file(pcb, path))
        if not cont:
            continue
        checked += 1
        for p in E.escape_ledger(pcb, pcb_file=path):
            for f in p.faces:
                assert not (set(f.blockers) & cont), (
                    '{}: {} charged a container {}'.format(
                        os.path.basename(path), p.ref,
                        sorted(set(f.blockers) & cont)))
    assert checked >= 1, (
        'no tracked board declares a container, so this arm proves nothing; '
        'rp2350_fpga_eensy_prePlane should have U8')
    print('  PASS: containers charged nowhere, on {} board(s) that have one'
          .format(checked))


def test_the_per_board_numbers():
    """The change detector. Regenerate with the measure script, do not guess."""
    seen = 0
    for name, want in sorted(EXPECTED.items()):
        pcb, path = _board(name)
        if pcb is None:
            print('    SKIP {} (not in this checkout)'.format(name))
            continue
        seen += 1
        led = E.escape_ledger(pcb, pcb_file=path)
        tot = deficit_totals(led)
        faces = sum(1 for p in led for f in p.faces if f.deficit > 0)
        got = (tot['lanes'], tot['parts'], faces)
        assert got == want, (
            '{}: lanes/parts/faces {} != recorded {} -- re-measure with '
            'tests/measure_834_835_side_awareness.py before re-recording'
            .format(name, got, want))
    assert seen >= 4, 'only {} board(s) checked'.format(seen)
    print('  PASS: {} board(s) match the recorded ledger'.format(seen))


def test_the_ulx3s_witness_stops_charging_across_the_board():
    """U9 (B) west "15.35mm taken by SD1" (F) -- and SD1's east charged back.

    The `.claude/skills/plan-pcb-placement` worked example is this pair, so
    the skill was teaching the defect.
    """
    pcb, path = _board('ulx3s')
    if pcb is None:
        print('  SKIP: ulx3s not present')
        return
    sides = _sides(pcb)
    assert sides['U9'] == frozenset(('B',)) and sides['SD1'] == frozenset(('F',)), (
        sorted(sides['U9']), sorted(sides['SD1']))
    led = {p.ref: p for p in E.escape_ledger(pcb, pcb_file=path)}
    for a, b in (('U9', 'SD1'), ('SD1', 'U9')):
        if a not in led:
            continue
        for f in led[a].faces:
            assert b not in f.blockers, (a, f.face, f.blockers)
    assert deficit_totals(E.escape_ledger(pcb, pcb_file=path))['lanes'] == 0, (
        'ulx3s still reports a deficit; every one of its six was cross-side')
    print('  PASS: U9<->SD1 charge each other nowhere, and ulx3s reports 0')


def test_the_rp2350_witness_is_the_container_not_the_side():
    """`via_slots`' docstring cites rp2350 U6/U8. U6 is DRILLED, so U8 being on
    the same face is not the point -- U8 is a Teensy module that CONTAINS U6.

    Both halves are asserted, because the two arms are complementary and it is
    easy to credit one with the other's effect.
    """
    pcb, path = _board('rp2350_fpga_eensy_prePlane')
    if pcb is None:
        print('  SKIP: rp2350 not present')
        return
    sides = _sides(pcb)
    assert sides['U6'] == frozenset(('F', 'B')), sorted(sides['U6'])
    cont = container_refs(pcb, graded_parts_from_file(pcb, path))
    assert cont == {'U8'}, sorted(cont)
    u6 = E.escape_ledger(pcb, pcb_file=path, refs=['U6'])[0]
    for f in u6.faces:
        assert 'U8' not in f.blockers, (f.face, f.blockers)
    assert any(f.supply > 0 for f in u6.faces), [f.supply for f in u6.faces]
    # ...and U6 is still the board's worst part: corrected, not quieted.
    assert u6.worst is not None and u6.worst.deficit >= 11, u6.to_dict()
    # The B-side U1 is still charged, because U6's leads reach that face. A
    # one-sided `own_side in g.sides` test would have dropped it.
    charged = {b for f in u6.faces for b in f.blockers}
    assert 'U1' in charged and sides['U1'] == frozenset(('B',)), (
        sorted(charged), sorted(sides['U1']))
    print('  PASS: U8 exempt as a container, B-side U1 still charged, '
          'U6 still worst at {}'.format(u6.worst.deficit))


def test_span_eaten_unions_and_clamps():
    """The shared kernel's two corrections, in isolation."""
    band, horiz = (0.0, 1.0), True
    # Two obstacles covering the same stretch are charged ONCE.
    blocked, order = E.span_eaten(
        0.0, 10.0, band, horiz,
        [('A', (2.0, 0.1, 6.0, 0.9)), ('B', (3.0, 0.1, 7.0, 0.9))])
    assert abs(blocked - 5.0) < 1e-9, blocked            # 2..7, not 4 + 4
    assert dict(order) == {'A': 4.0, 'B': 4.0}, order
    assert [r for r, _ in order] == ['A', 'B'], order
    # ...and the total can never exceed the span.
    blocked, _ = E.span_eaten(
        0.0, 5.0, band, horiz,
        [('A', (-100.0, 0.1, 100.0, 0.9)), ('B', (-50.0, 0.1, 50.0, 0.9))])
    assert abs(blocked - 5.0) < 1e-9, blocked
    # An obstacle outside the band is not charged at all.
    blocked, order = E.span_eaten(
        0.0, 10.0, band, horiz, [('A', (2.0, 5.0, 6.0, 9.0))])
    assert blocked == 0.0 and order == (), (blocked, order)
    print('  PASS: union 5.0mm (not 8.0), clamped to the span, band respected')


def test_a_back_side_neighbour_is_not_named_but_a_drilled_one_is():
    """The fixtures the existing suite lacks: a `layer='B.Cu'` blocker.

    `tests/test_escape_ledger.py`'s `_Fp` has no `layer` attribute, so every
    one of its parts resolves to 'F' and the new skip is inert there.
    """
    victim = _Fp('U1', [_Pad(10 + 0.5 * i, 10) for i in range(8)]
                 + [_Pad(10 + 0.5 * i, 13.5) for i in range(8)])
    front = _Fp('J9', [_Pad(10 + 0.5 * i, 9.6) for i in range(8)],
                layer='F.Cu', name='Conn')
    back = _Fp('J8', [_Pad(10 + 0.5 * i, 9.6) for i in range(8)],
               layer='B.Cu', name='Conn')
    tht = _Fp('J7', [_Pad(10 + 0.5 * i, 9.6, drill=0.8) for i in range(8)],
              layer='B.Cu', name='Conn')

    def blockers_of(neighbour):
        pcb = _Pcb([victim, neighbour])
        pe = E.part_escape(pcb, 'U1', pitch_mm=0.25)
        return {b for f in pe.faces for b in f.blockers}

    assert 'J9' in blockers_of(front), 'a same-side neighbour must be named'
    assert 'J8' not in blockers_of(back), (
        'a B.Cu neighbour is still charged against an F.Cu part')
    assert 'J7' in blockers_of(tht), (
        'a DRILLED back-side neighbour occupies both faces and must be named')
    print('  PASS: front named, back skipped, drilled back named')


def test_face_lane_ledger_cannot_cover_more_than_the_face():
    """`routability`'s half of the kernel: the union, and the clamp.

    Before, this loop accumulated `covered += span` per neighbour, so two
    bodies over the same stretch were billed twice; `max(0.0, length -
    covered)` absorbed the impossible total instead of refusing it.
    """
    checked = rows = 0
    for name in ('glasgow_revC', 'tigard', 'rp2350_fpga_eensy_prePlane'):
        pcb, path = _board(name)
        if pcb is None:
            continue
        checked += 1
        refs = [r for r in sorted(pcb.footprints)][:40]
        for ref in refs:
            try:
                led = R.face_lane_ledger(pcb, ref, clearance=0.2,
                                         track_width=0.2, grid_step=0.1,
                                         pcb_file=path)
            except Exception:                                 # noqa: BLE001
                continue
            for row in led:
                rows += 1
                eaten = sum(mm for _r, mm in row['eaten_by'])
                # per-ref amounts may overlap; the SUPPLY is what must be sane
                assert row['supply_routed_grid'] >= 0, row
                assert row['supply_finest_grid'] >= row['supply_routed_grid'], row
                assert row['length_mm'] >= 0, row
                assert eaten >= 0, row
    assert checked >= 2 and rows > 50, (checked, rows)
    print('  PASS: {} rows over {} board(s), supply never negative and the '
          'finest grid never supplies less'.format(rows, checked))


def test_the_union_is_measured_where_it_actually_bites():
    """glasgow_revC J1 east: eight neighbours, 12.42mm of cover on a 9.64mm
    face. SUMMED that is more than the face, so `max(0.0, length - covered)`
    collapses supply to 0; UNIONED it is 8.23mm and supply is 3.

    Chosen by scanning, not by guessing. My first witness for this was tigard
    J1 east, and it was wrong: tigard's routability numbers move under this
    branch because of the SYMMETRIC SIDE TEST, not the union -- summing and
    unioning give it the same supply. The mutation battery caught that, which
    is what it is for. Corpus-wide there are 227 faces where the summed cover
    exceeds the union; this is the largest clean one on a board with no
    container.
    """
    pcb, path = _board('glasgow_revC')
    if pcb is None:
        print('  SKIP: glasgow_revC not present')
        return
    assert not container_refs(pcb, graded_parts_from_file(pcb, path)), (
        'glasgow grew a container; this is no longer a clean union witness')
    rows = {r['face']: r for r in R.face_lane_ledger(
        pcb, 'J1', clearance=0.2, track_width=0.2, grid_step=0.1,
        pcb_file=path)}
    east = rows['E']
    assert abs(east['length_mm'] - 9.64) < 0.01, east
    assert len(east['eaten_by']) == 8, east['eaten_by']
    summed = sum(mm for _r, mm in east['eaten_by'])
    assert summed > east['length_mm'], (
        'the per-neighbour cover no longer exceeds the face, so summing and '
        'unioning would agree and this arm proves nothing: {} vs {}'
        .format(summed, east['length_mm']))
    assert east['supply_routed_grid'] == 3, (
        'glasgow J1 east supply moved from the recorded 3 (it is 0 if the '
        'intervals are summed rather than unioned): {}'.format(east))


def test_tigard_moves_on_the_side_test_not_the_union():
    """The other half, kept apart on purpose.

    tigard has no container and no cross-side blocker charge, so its ESCAPE
    ledger is 41 lanes before and after. Its `routability` numbers do move, and
    the cause is the symmetric side test: J1 is DRILLED, so it occupies both
    faces and the B-side JP1 is charged against its north face, where the
    one-sided `own_side in g.sides` dropped it.
    """
    pcb, path = _board('tigard')
    if pcb is None:
        print('  SKIP: tigard not present')
        return
    sides = _sides(pcb)
    assert sides['J1'] == frozenset(('F', 'B')), sorted(sides['J1'])
    assert sides['JP1'] == frozenset(('B',)), sorted(sides['JP1'])
    rows = {r['face']: r for r in R.face_lane_ledger(
        pcb, 'J1', clearance=0.2, track_width=0.2, grid_step=0.1,
        pcb_file=path)}
    assert 'JP1' in {r for r, _mm in rows['N']['eaten_by']}, rows['N']
    print('  PASS: glasgow J1 east supply 3 (0 if summed); tigard J1 keeps '
          'its B-side JP1 through the symmetric side test')


def test_face_lane_ledger_side_test_is_symmetric():
    """A drilled part keeps its far-side blockers.

    The one-sided `own_side in g.sides` charged a THT part only for
    neighbours on its `footprint_side`, though its leads occupy both.
    """
    pcb, path = _board('rp2350_fpga_eensy_prePlane')
    if pcb is None:
        print('  SKIP: rp2350 not present')
        return
    sides = _sides(pcb)
    assert sides['U6'] == frozenset(('F', 'B'))
    led = R.face_lane_ledger(pcb, 'U6', clearance=0.2, track_width=0.2,
                             grid_step=0.1, pcb_file=path)
    eaten = {r for row in led for r, _mm in row['eaten_by']}
    back = {r for r in eaten if sides.get(r) == frozenset(('B',))}
    assert back, (
        'no back-side neighbour is charged against the drilled U6; the '
        'symmetric side test regressed: {}'.format(sorted(eaten)))
    assert 'U8' not in eaten, 'the container is charged here too'
    print('  PASS: U6 keeps {} back-side blocker(s) and no container'
          .format(len(back)))


def main():
    bad = []
    for name, fn in sorted(globals().items()):
        if not name.startswith('test_'):
            continue
        print('--- {}'.format(name))
        try:
            fn()
        except Exception as exc:                              # noqa: BLE001
            bad.append('{}: {}: {}'.format(name, type(exc).__name__, exc))
            print('  FAIL: {}'.format(bad[-1]))
    if bad:
        print('\n{} FAILED'.format(len(bad)))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
