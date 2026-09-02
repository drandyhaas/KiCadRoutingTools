#!/usr/bin/env python3
"""#841: the ONE rectangle a lane ledger charges a neighbour.

`escape` charged the bbox of pad CENTRES; `routability.face_lane_ledger`
charged the COURTYARD. The kernel they share since #835 takes its rects from
the caller, so the two answered the same question differently -- by 2 to 7x on
five boards.

This file pins the rect ITSELF, before either consumer uses it, so a later
change to a ledger cannot quietly redefine what "obstruction" means.

What is asserted, and why each one is here rather than implied:

  1. It is the pad-COPPER box, and it is `PartPads.extent` -- CALLED, not
     re-derived. A second copy of that geometry is how the two channels came
     to disagree in the first place.
  2. It is strictly WIDER than the pad-centre box wherever a part has more
     than one pad. The old rect is the degenerate case, not a conservative
     one: a two-terminal passive contributes a zero-width obstruction.
  3. It is NOT the courtyard, on a board where the two differ. Without this,
     a well-meaning edit could satisfy every other assertion here by handing
     back `GradedPart.rect`.
  4. A footprint with no pads is ABSENT, not present with a fiction. This is
     the `synthetic` hazard #841 names, closed by construction.
  5. `clearance` reaches the result. It is easy to read this parameter as
     decoration -- `extent` is a copper box -- but NPTH growth is folded into
     it, so a hardcoded clearance would price a mounting hole wrongly and
     nothing else in the file would notice.

Run: python3 -X utf8 tests/test_841_obstruction_rect.py
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
from placement.legality import (build_part_pads,              # noqa: E402
                                graded_parts_from_file)

RUN_ALL_FAST_OK = True

#: Parts whose pad-copper extent MOVES with the clearance argument, and by how
#: much. All NPTH mounting holes: `PartPads` folds the growth
#: `max(0, npth_floor - clearance)` into `holes_extent`. Regenerate with
#: `tests/measure_834_835_side_awareness.py --table D`.
NPTH_MOVERS = {
    'tigard': {'H1', 'H2', 'H3', 'H4'},
    'rp2350_fpga_eensy_prePlane': {'J2'},
}
NPTH_DELTA_MM = 0.200


def _board(name):
    for p in run_utils.corpus_boards():
        if os.path.splitext(os.path.basename(p))[0] == name:
            return parse_kicad_pcb(p), p
    return None, None


def test_it_is_PartPads_extent_called_not_copied():
    """Rule 1. The grader is CALLED; this test would pass on a copy that
    happened to agree today and drift tomorrow, so it compares object for
    object over every part of three boards rather than sampling."""
    seen = 0
    for name in ('tigard', 'glasgow_revC', 'rp2350_fpga_eensy_prePlane'):
        pcb, path = _board(name)
        if pcb is None:
            print('  SKIP {} absent'.format(name))
            continue
        rects = E.board_obstruction_rects(pcb, 0.2)
        parts = build_part_pads(pcb.footprints, 0.2)
        for ref, pp in parts.items():
            fp = pcb.footprints[ref]
            want = pp.extent(fp.x, fp.y, fp.rotation or 0.0)
            if want is None:
                assert ref not in rects, (
                    '{}: {} has no extent but is in the map'.format(name, ref))
                continue
            assert rects.get(ref) == want, (
                '{}: {} obstruction rect {} != PartPads.extent {}'
                .format(name, ref, rects.get(ref), want))
            seen += 1
    assert seen > 300, 'only {} parts compared; the corpus did not load'.format(seen)
    print('  PASS: {} parts, rect is PartPads.extent exactly'.format(seen))


def test_it_is_wider_than_the_pad_centre_box():
    """Rule 2. The old rect spans pad CENTRES, so it is contained in this one
    on every part, and STRICTLY smaller wherever a pad has area -- which is
    every real part. A part measured at its centres is not conservative; it is
    a part whose terminals do not exist."""
    strict = 0
    total = 0
    for name in ('tigard', 'glasgow_revC', 'watchy'):
        pcb, _p = _board(name)
        if pcb is None:
            print('  SKIP {} absent'.format(name))
            continue
        rects = E.board_obstruction_rects(pcb, 0.2)
        for ref, rect in rects.items():
            fp = pcb.footprints[ref]
            if not fp.pads:
                continue
            c = E._part_rect(fp)
            total += 1
            assert (rect[0] <= c[0] + 1e-9 and rect[1] <= c[1] + 1e-9
                    and rect[2] >= c[2] - 1e-9 and rect[3] >= c[3] - 1e-9), (
                '{}: {} copper rect {} does not contain the centre rect {}'
                .format(name, ref, rect, c))
            if (rect[2] - rect[0]) > (c[2] - c[0]) + 1e-6:
                strict += 1
    assert total > 200, 'only {} parts examined'.format(total)
    assert strict > 0.9 * total, (
        'only {} of {} parts are STRICTLY wider than their pad-centre box; '
        'the rect has collapsed back to centres'.format(strict, total))
    print('  PASS: {} of {} parts strictly wider than the pad-centre box'
          .format(strict, total))


def test_it_is_not_the_courtyard():
    """Rule 3. The change detector for the other wrong answer. Asserting only
    "wider than centres" would be satisfied by the courtyard too."""
    pcb, path = _board('glasgow_revC')
    if pcb is None:
        print('  SKIP glasgow_revC absent')
        return
    rects = E.board_obstruction_rects(pcb, 0.2)
    court = {g.ref: g.rect for g in graded_parts_from_file(pcb, path)}
    differ = sum(1 for r, v in rects.items()
                 if r in court and max(abs(a - b) for a, b in zip(v, court[r])) > 1e-6)
    assert differ > 0.5 * len(rects), (
        'only {} of {} rects differ from the courtyard; the map is charging '
        'assembly keep-out, not copper'.format(differ, len(rects)))
    print('  PASS: {} of {} rects differ from the courtyard'
          .format(differ, len(rects)))


def test_a_padless_footprint_is_absent():
    """Rule 4. `legality.part_local_bounds` invents a +/-0.5mm box for a
    footprint with no courtyard AND no pads, and flags it `synthetic`.
    `grade_body_overlap` refuses to gate on those. A lane SUPPLY is a number
    `options.move_blocker` turns into "move this part", so the fiction must
    not reach it -- here it cannot, because no pads means no entry."""
    checked = 0
    for name in ('tigard', 'glasgow_revC'):
        pcb, _p = _board(name)
        if pcb is None:
            print('  SKIP {} absent'.format(name))
            continue
        rects = E.board_obstruction_rects(pcb, 0.2)
        padless = [r for r, f in pcb.footprints.items() if not (f.pads or [])]
        assert padless, '{}: no pad-less footprint to test with'.format(name)
        for r in padless:
            assert r not in rects, (
                '{}: pad-less {} is charged an obstruction'.format(name, r))
        checked += len(padless)
    assert checked >= 3, 'only {} pad-less footprints seen'.format(checked)
    print('  PASS: {} pad-less footprints, none charged'.format(checked))


def test_clearance_reaches_the_result_and_only_via_drilled_holes():
    """Rule 5. Two halves, and the second is the one that makes it a test
    rather than a demonstration: the clearance must move the NPTH parts AND
    must move nothing else. A blanket inflation would satisfy the first half
    alone."""
    for name, expect in sorted(NPTH_MOVERS.items()):
        pcb, _p = _board(name)
        if pcb is None:
            print('  SKIP {} absent'.format(name))
            continue
        tight = E.board_obstruction_rects(pcb, 0.0)
        loose = E.board_obstruction_rects(pcb, 0.5)
        moved = {r for r in tight
                 if max(abs(a - b) for a, b in zip(tight[r], loose[r])) > 1e-6}
        assert moved == expect, (
            '{}: parts moving with clearance {} != expected {}'
            .format(name, sorted(moved), sorted(expect)))
        for r in moved:
            d = max(abs(a - b) for a, b in zip(tight[r], loose[r]))
            assert abs(d - NPTH_DELTA_MM) < 1e-6, (
                '{}: {} moved {:.4f}mm, not the recorded {}mm'
                .format(name, r, d, NPTH_DELTA_MM))
        print('  PASS: {} -- {} moved by {}mm, nothing else'
              .format(name, sorted(moved), NPTH_DELTA_MM))


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
