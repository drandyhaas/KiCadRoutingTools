#!/usr/bin/env python3
"""#1664: an unlocked part's edge row on a pile is a staging reading, not an ERROR.

Every conjunct of `rule_edge_connector` but the mating face's evidence reads
the part's pose, and on a pile an unlocked part's pose is staging. A correct
row read 2 ERRORs on one edge and 4 on another with exit 4, and read none on the
first placed seed, so no edit could clear them, and its overhang and model end
were numbers the skill told the reader to act on. The rule asks the pile test
the emitter, board_brief, cover_facts and reconcile ask (`PlacementState.pile`,
and a KiCad lock is a decision).

Pinned here, with an edge-mount jack the fixture's own placement seats:
  1. on the pile a correct row, on either edge, raises no `edge_connector`
     finding; its evidence row says `staging`, with the declared band and the
     face's evidence and no overhang or facing verdict, the printed row says so,
     the row abstains rather than passes, and `--plan-only` exits 0 on it;
  2. what the footprint alone says still reads on the pile: a declared face with
     no source warns;
  3. a LOCKED part on a pile still grades: a wrong edge is an ERROR there;
  4. the board that is not a pile grades as before: the same wrong row is an
     ERROR with its overhang read, and `.staged` is False for every part.

    python3 tests/test_1664_edge_row_on_a_pile.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_tools', 'py_placer', 'py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import floorplan                                # noqa: E402
from placement.placement_state import assess_placement         # noqa: E402
from run_utils import check, evidence, tool                    # noqa: E402

PY = [sys.executable, '-B', '-X', 'utf8']
FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', '1603', 'edge_sma.kicad_pcb')
MAKE_UNPLACED = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                             'scripts', 'make_unplaced.py')
PLACE_POSE = tool('place_pose.py')
BAND = {'min': 11.719, 'max': 12.43}


def stage(src, dst, *flags):
    check(PY + [MAKE_UNPLACED, src, dst, *flags], accept=True, allow=('WARNING',))
    return evidence(dst, 'the staged pile')


def graded(board, rows):
    doc = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
           'edge_connectors': rows}
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    return pcb, floorplan.grade(floorplan.intent_from_dict(doc, ''), pcb, board)


def of_rule(res, severity=None):
    return [v for v in res.violations if v.rule == 'edge_connector'
            and (severity is None or v.severity == severity)]


def test_pile_row_is_staging(d):
    pile = stage(FIXTURE, os.path.join(d, 'pile.kicad_pcb'))
    with contextlib.redirect_stdout(io.StringIO()):
        assert assess_placement(parse_kicad_pcb(pile), pile).pile, 'the fixture must stage to a pile'
    for edge in ('east', 'north'):
        _, res = graded(pile, [{'ref': 'J3', 'edge': edge, 'overhang_mm': BAND}])
        assert not of_rule(res), [v.message for v in of_rule(res)]
        row = res.edge_connector_evidence[0]
        assert row['staging'] is True and row['overhang_limit_mm'] == BAND, row
        assert 'overhang_mm' not in row and 'model_end_mm' not in row, row
        assert row['facing']['disposition'] == 'staging' and 'facing' not in row['facing'], row
        assert row['facing']['evidence'], f'the evidence label is what a pile can answer: {row}'
        text = floorplan.format_text(res)
        assert f'J3 {edge}: staging pose on a pile' in text, text
        assert f"[{row['facing']['basis']}, {row['facing']['evidence']}]" in text, text
        # neither a violation nor a pass: what a brief's clause on it reads
        assert 'edge_connectors[J3].pose' in res.budget_abstained, res.budget_abstained
        assert not res.complete, 'a staged row is an absent verdict, not a clean one'

        intent = os.path.join(d, f'{edge}.json')
        with open(intent, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                       'edge_connectors': [{'ref': 'J3', 'edge': edge,
                                            'overhang_mm': BAND}]}, fh)
        check(PY + [tool('check_floorplan.py'), pile, '--intent', intent, '--plan-only'],
              accept=True, allow=('WARNING',))
    print('  ok: a correct row on a pile raises nothing, says `staging`, and --plan-only passes')


def test_pile_keeps_what_the_footprint_says(d):
    pile = os.path.join(d, 'pile.kicad_pcb')
    _, res = graded(pile, [{'ref': 'J3', 'edge': 'east', 'overhang_mm': BAND,
                            'mating_face': '-y'}])
    assert not of_rule(res, 'error'), [v.message for v in of_rule(res, 'error')]
    warns = [v.message for v in of_rule(res, 'warn')]
    assert any("J3's mating face" in m for m in warns), warns
    print('  ok: the warning about a declared face still reads on the pile')


def test_locked_part_on_a_pile_grades(d):
    locked = os.path.join(d, 'locked.kicad_pcb')
    check(PY + [PLACE_POSE, FIXTURE, locked, 'lock', 'J3', '--force', '--allow-routed'],
          accept=True, allow=('WARNING',))
    pile = stage(evidence(locked), os.path.join(d, 'locked_pile.kicad_pcb'), '--keep-locked')
    pcb, res = graded(pile, [{'ref': 'J3', 'edge': 'north', 'overhang_mm': BAND}])
    assert assess_placement(pcb, pile).pile, 'the rest must stage to a pile, or this tests nothing'
    assert 'J3' in {r for r, fp in pcb.footprints.items() if fp.locked}
    row = res.edge_connector_evidence[0]
    assert 'staging' not in row and 'overhang_mm' in row, row
    assert any('sits nearest' in v.message and v.severity == 'error' for v in of_rule(res)), \
        [v.message for v in of_rule(res)]
    print('  ok: a locked part on a pile still grades: a wrong edge is an ERROR')


def test_placed_board_grades(d):
    pcb, res = graded(FIXTURE, [{'ref': 'J3', 'edge': 'north', 'overhang_mm': BAND}])
    assert not assess_placement(pcb, FIXTURE).pile, 'the fixture itself must not be a pile'
    row = res.edge_connector_evidence[0]
    assert 'staging' not in row and 'overhang_mm' in row, row
    assert any('sits nearest' in v.message and v.severity == 'error' for v in of_rule(res)), \
        [v.message for v in of_rule(res)]
    print('  ok: the placed board grades the same row as ever')


def main():
    d = tempfile.mkdtemp(prefix='t1664_')
    try:
        test_pile_row_is_staging(d)
        test_pile_keeps_what_the_footprint_says(d)
        test_locked_part_on_a_pile_grades(d)
        test_placed_board_grades(d)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
