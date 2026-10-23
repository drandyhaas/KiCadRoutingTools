#!/usr/bin/env python3
"""#959 / #999: pad-less blocks are counted, answered for, and never crash.

Run 29's P1 printed "6 zoned block(s) cover all 13 movable part(s)" on a board
of 21 footprint blocks. The denominator was `if fp.pads` -- what the seeder
moves -- and so it hid exactly the three blocks the seeder never touches: the
pad-less logos. One of them sat at the pile origin for 12 laps printing silk
across CON2's apertures, found by eye rather than by any gate. And asking
for a pose for one of them died in a KeyError traceback at exit 1, the same
code as the verdict "no legal pose".

Traps written against:

  * the old advice was wrong for exactly these blocks -- "add each to a block
    with a zone" does nothing to a block the seeder never places, so an arm
    asserts a zoned pad-less block is REFUSED as inert rather than accepted;
  * `must_lock` looks like an answer and is not one -- it stamps a lock the
    seeder writes after seating, and this block is never seated;
  * a non-zero exit is not evidence: every CLI arm asserts the reason, and
    that there is NO traceback.

The P1 pad-less census and its refusals lived in the retired placement_driver
and left with it (its `dispositions.refs` judgement survives as
`floorplan.stale_dispositions`, pinned in test_959_rule_roster). What stays is
the board fact, the grade, and the pose ranker and CLI that must refuse,
not crash.
"""
import json
import os
import shutil
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _sub in ('', 'py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(REPO, _sub) if _sub else REPO
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402

RUN_ALL_TIMEOUT = 900

ESP = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
LOGOS = ['#00000000-0000-0000-0000-00005a3b5201',
         '#00000000-0000-0000-0000-00005d8c51dd',
         '#00000000-0000-0000-0000-00005e7dd057']


def _tiny_board(path, refs, locked=(), padless=(), courtyard=()):
    """An outline and one part per ref, each with one connected pad -- except
    the refs in `padless`, which carry none (a logo); those in `courtyard`
    also draw an F.CrtYd rectangle, which is what lets `zone_containment`
    grade a pad-less block at all. Refs in `locked` carry `(locked yes)`.
    (Was placement_driver._tiny_board, retired with that driver.)"""
    def _body(r):
        if r in padless:
            return ('    (fp_rect (start -1 -1) (end 1 1) (layer "F.CrtYd") '
                    '(width 0.05))\n' if r in courtyard else '')
        return (f'    (pad "1" smd rect (at 0 0) (size 0.6 0.8) '
                f'(layers "F.Cu") (net 1 "/A") (uuid "p1-{r}"))\n')
    fps = ''.join(
        f'  (footprint "test:FP" (layer "F.Cu") (uuid "fp-{r}") '
        f'(at {2 + 3 * i} 2){" (locked yes)" if r in locked else ""}\n'
        f'    (property "Reference" "{r}" (at 0 0))\n'
        + _body(r) + '  )\n' for i, r in enumerate(refs))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "test")\n'
                 '  (net 0 "")\n  (net 1 "/A")\n'
                 '  (gr_rect (start 0 0) (end 20 10) (layer "Edge.Cuts") '
                 '(uuid "e1"))\n' + fps + ')\n')
    return path


def test_esp_prog_has_21_blocks_and_three_are_padless():
    pcb = parse_kicad_pcb(ESP)
    assert len(pcb.footprints) == 21, len(pcb.footprints)
    padless = sorted(k for k, fp in pcb.footprints.items() if not fp.pads)
    assert padless == LOGOS, padless
    print(f"  PASS: 21 blocks, pad-less: {len(padless)}")


def test_a_padless_block_with_a_courtyard_is_graded():
    """A pad-less block that draws a courtyard IS graded by its zone: move
    the locked logo outside the zone and zone_containment names it."""
    with tempfile.TemporaryDirectory() as tmp:
        blocks = [{'name': 'all', 'refs': ['U*', 'LOGO1'],
                   'zone': [0, 0, 10, 10], 'note': 'ICs and the logo'}]
        plan = os.path.join(tmp, 'p.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent',
                       'units': 'mm', 'blocks': blocks}, fh)
        locked = _tiny_board(os.path.join(tmp, 'b.kicad_pcb'),
                             ('U1', 'U2', 'LOGO1'), padless=('LOGO1',),
                             courtyard=('LOGO1',), locked=('LOGO1',))
        from placement import floorplan as fp_
        it = fp_.load_intent(plan)
        far = os.path.join(tmp, 'c.kicad_pcb')
        text = open(locked, encoding='utf-8').read().replace(
            '(uuid "fp-LOGO1") (at 8 2)', '(uuid "fp-LOGO1") (at 18 8)')
        open(far, 'w', encoding='utf-8').write(text)
        res = fp_.grade(it, parse_kicad_pcb(far), far)
        assert any(v.rule == 'zone_containment' and v.ref == 'LOGO1'
                   for v in res.violations), res.violations
        # The control: in its zone, the same logo draws no containment.
        res = fp_.grade(it, parse_kicad_pcb(locked), locked)
        assert not any(v.rule == 'zone_containment' and v.ref == 'LOGO1'
                       for v in res.violations), res.violations
    print("  PASS: a courtyard pad-less block is graded by its zone")


def test_rank_poses_refuses_instead_of_raising_a_bare_keyerror():
    import pose_score
    pcb = parse_kicad_pcb(ESP)
    for key, code in ((LOGOS[0], 4), ('NOPE1', 2)):
        try:
            pose_score.rank_poses(pcb, ESP, key)
        except pose_score.PoseUnrankable as exc:
            assert exc.code == code, (key, exc.code)
            assert isinstance(exc, KeyError)
            assert str(exc) == exc.reason and not str(exc).startswith("'")
        else:
            raise AssertionError(f'{key} was ranked')
    print("  PASS: PoseUnrankable carries code 4 (a real block) / 2 (none)")


def test_place_pose_snap_on_a_padless_block_is_a_refusal():
    """`--strict-legal` on a board that is not already clean sends the pose
    down the snap path, which is where `rank_poses` is asked about the block.
    Run 29's own pile is that board (every part stacked at the centre); on the
    base commit this died in a KeyError traceback at exit 1."""
    pile = os.path.join(REPO, 'tests', 'fixtures', '959',
                        'run29_pile.kicad_pcb')
    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'pile.kicad_pcb')
        shutil.copy(pile, board)
        shutil.copy(pile[:-len('.kicad_pcb')] + '.kicad_pro',
                    board[:-len('.kicad_pcb')] + '.kicad_pro')
        before = open(board, 'rb').read()
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('place_pose.py'), board, board,
                             'set', LOGOS[1], '--near', '120', '95',
                             '--strict-legal'],
                            refuse='cannot be snapped', code=4)
        assert 'Traceback' not in (r.stdout + r.stderr)
        assert 'give it an exact pose instead' in r.stdout, r.stdout[-800:]
        assert open(board, 'rb').read() == before, 'the board was written'
        # The refusal rides in the full summary: the op it refused is named.
        line = [x for x in r.stdout.splitlines()
                if x.startswith('JSON_SUMMARY:')][-1]
        summ = json.loads(line.split('JSON_SUMMARY: ', 1)[1])
        assert summ['exit_code'] == 4 and summ.get('knobs'), summ
        assert summ['ops'], summ
        # Nothing was written, so the summary names no output (round 2).
        assert summ['output'] is None, summ['output']
    print("  PASS: place_pose's snap path refuses a pad-less block at exit "
          "4 and writes nothing")


TESTS = [
    test_esp_prog_has_21_blocks_and_three_are_padless,
    test_a_padless_block_with_a_courtyard_is_graded,
    test_rank_poses_refuses_instead_of_raising_a_bare_keyerror,
    test_place_pose_snap_on_a_padless_block_is_a_refusal,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
