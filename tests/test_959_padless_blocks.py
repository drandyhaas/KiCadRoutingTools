#!/usr/bin/env python3
"""#959 / #999: pad-less blocks are counted, answered for, and never crash.

Run 29's P1 printed "6 zoned block(s) cover all 13 movable part(s)" on a board
of 21 footprint blocks. The denominator was `if fp.pads` -- what the seeder
moves -- and so it hid exactly the three blocks the seeder never touches: the
pad-less logos. One of them sat at the pile origin for 12 laps printing silk
across CON2's apertures, found by eye rather than by any gate. And asking
`converge poses` about one of them died in a KeyError traceback at exit 1, the
same code as the verdict "no legal pose".

Traps written against:

  * the old advice was wrong for exactly these blocks -- "add each to a block
    with a zone" does nothing to a block the seeder never places, so an arm
    asserts a zoned pad-less block is REFUSED as inert rather than accepted;
  * `must_lock` looks like an answer and is not one -- it stamps a lock the
    seeder writes after seating, and this block is never seated;
  * a non-zero exit is not evidence: every CLI arm asserts the reason, and
    the converge arm asserts there is NO traceback.
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
PLAN_975 = os.path.join(REPO, 'tests', 'fixtures', '975', 'esp_prog_run27',
                        'zone_plan.json')
DRIVER = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement',
                      'scripts', 'placement_driver.py')
LOGOS = ['#00000000-0000-0000-0000-00005a3b5201',
         '#00000000-0000-0000-0000-00005d8c51dd',
         '#00000000-0000-0000-0000-00005e7dd057']


def _driver():
    sys.path.insert(0, os.path.dirname(DRIVER))
    import importlib
    return importlib.import_module('placement_driver')


def _p1(board, plan, *extra, waive=True):
    return ([sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
             '--board', board, '--zone-plan', plan]
            + (['--waive', 'seed-connectors:the probe hands them over']
               if waive else []) + list(extra))


def _plan(tmp, name, **edits):
    with open(PLAN_975, encoding='utf-8') as fh:
        doc = json.load(fh)
    doc.update(edits)
    p = os.path.join(tmp, name)
    with open(p, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return p


def test_esp_prog_has_21_blocks_and_three_are_padless():
    pcb = parse_kicad_pcb(ESP)
    assert len(pcb.footprints) == 21, len(pcb.footprints)
    padless = sorted(k for k, fp in pcb.footprints.items() if not fp.pads)
    assert padless == LOGOS, padless
    print(f"  PASS: 21 blocks, pad-less: {len(padless)}")


def test_p1_names_the_three_logos():
    """The plan zones every pad-bearing part (fixture 975, run 27's own), and
    P1 still refuses -- naming the three blocks the old count left out."""
    r = run_utils.check(_p1(ESP, PLAN_975),
                        refuse='3 pad-less block(s) are answered for by '
                               'nothing', code=4)
    for key in LOGOS:
        assert key in r.stdout, (key, r.stdout[-1500:])
    assert 'place_pose.py' in r.stdout and "lock '<KEY>'" in r.stdout
    print("  PASS: P1 refuses naming all three #uuid logo blocks")


def test_the_padless_check_keeps_the_older_refusals_first():
    """Precedence: every refusal P1 had before #959 still comes first. On
    esp_prog with run 27's plan and NO seed-connectors waiver the reader is
    told about the free connectors -- the run-27 refusal -- not the logos;
    with the waiver, the logos."""
    r = run_utils.check(_p1(ESP, PLAN_975, waive=False),
                        refuse='carry no `(locked yes)` in the board', code=4)
    assert 'pad-less' not in r.stdout, r.stdout[-800:]
    run_utils.check(_p1(ESP, PLAN_975),
                    refuse='pad-less block(s) are answered for by nothing',
                    code=4)
    print("  PASS: the seed-connectors refusal precedes the pad-less one")


def test_must_lock_does_not_answer_a_padless_block():
    with tempfile.TemporaryDirectory() as tmp:
        plan = _plan(tmp, 'ml.json',
                     must_lock=['USB1', 'Ref*', 'Ref*~2'] + LOGOS)
        run_utils.check(_p1(ESP, plan),
                        refuse='3 pad-less block(s) are answered for by '
                               'nothing', code=4)
    print("  PASS: must_lock does not excuse a block the seeder never seats")


def test_a_zoned_padless_block_is_refused_as_inert():
    with tempfile.TemporaryDirectory() as tmp:
        with open(PLAN_975, encoding='utf-8') as fh:
            blocks = json.load(fh)['blocks']
        blocks = blocks + [{'name': 'logos', 'refs': [LOGOS[0]],
                            'zone': [120, 95, 125, 100],
                            'note': 'the recycle logo, back side'}]
        plan = _plan(tmp, 'inert.json', blocks=blocks)
        run_utils.check(_p1(ESP, plan),
                        refuse='sit in a zoned block and draw no courtyard',
                        code=4)
        # LOCKED does not rescue it: no rule grades a block with no pads and
        # no courtyard, so the zone would be a claim nothing checks (the
        # Phase-2 verifier measured a locked logo 13 mm outside its zone
        # grading PASS). 0 of 30 corpus pad-less blocks draw a courtyard.
        board = os.path.join(tmp, 'esp.kicad_pcb')
        shutil.copy(ESP, board)
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('place_pose.py'), board, board,
                         'lock', LOGOS[0]], accept=True)
        run_utils.check(_p1(board, plan),
                        refuse='sit in a zoned block and draw no courtyard',
                        code=4)
    print("  PASS: a zone around a courtyard-less pad-less block is refused, "
          "locked or not")


def test_a_padless_block_with_a_courtyard_is_graded_once_locked():
    drv = _driver()
    with tempfile.TemporaryDirectory() as tmp:
        blocks = [{'name': 'all', 'refs': ['U*', 'LOGO1'],
                   'zone': [0, 0, 10, 10], 'note': 'ICs and the logo'}]
        plan = os.path.join(tmp, 'p.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump(drv._zone_plan_doc(blocks), fh)
        unlocked = drv._tiny_board(os.path.join(tmp, 'a.kicad_pcb'),
                                   ('U1', 'U2', 'LOGO1'), padless=('LOGO1',),
                                   courtyard=('LOGO1',))
        argv = [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
                '--zone-plan', plan, '--board']
        run_utils.check(argv + [unlocked],
                        refuse='the zone DOES grade it once it is placed',
                        code=4)
        locked = drv._tiny_board(os.path.join(tmp, 'b.kicad_pcb'),
                                 ('U1', 'U2', 'LOGO1'), padless=('LOGO1',),
                                 courtyard=('LOGO1',), locked=('LOGO1',))
        run_utils.check(argv + [locked], accept=True)
        # ...and the grade really does see it: move the locked logo outside
        # the zone and zone_containment names it.
        from placement import floorplan as fp_
        it = fp_.load_intent(plan)
        far = os.path.join(tmp, 'c.kicad_pcb')
        text = open(locked, encoding='utf-8').read().replace(
            '(uuid "fp-LOGO1") (at 8 2)', '(uuid "fp-LOGO1") (at 18 8)')
        open(far, 'w', encoding='utf-8').write(text)
        res = fp_.grade(it, parse_kicad_pcb(far), far)
        assert any(v.rule == 'zone_containment' and v.ref == 'LOGO1'
                   for v in res.violations), res.violations
    print("  PASS: a courtyard pad-less block in a zone must be locked, and "
          "is then graded")


def test_a_disposition_or_a_file_lock_answers_it():
    with tempfile.TemporaryDirectory() as tmp:
        plan = _plan(tmp, 'disp.json', dispositions={'refs': {
            k: 'a back-side logo; its position is cosmetic' for k in LOGOS}})
        # Answered: P1 moves on to its NEXT question -- the rule roster --
        # which is asserted BY ITS OWN TEXT: "the pad-less text is absent" is
        # also true of a refusal that rejected the disposition itself.
        run_utils.check(_p1(ESP, plan),
                        refuse='rule(s) this plan leaves dark', code=4)
        # The file lock answers it too, and is what the refusal tells the
        # reader to write.
        board = os.path.join(tmp, 'esp.kicad_pcb')
        shutil.copy(ESP, board)
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('place_pose.py'), board, board,
                         'lock'] + LOGOS, accept=True)
        run_utils.check(_p1(board, PLAN_975),
                        refuse='rule(s) this plan leaves dark', code=4)
        # A disposition for a block already locked answers nothing.
        plan = _plan(tmp, 'twice.json', dispositions={'refs': {
            LOGOS[0]: 'answered twice'}})
        run_utils.check(_p1(board, plan),
                        refuse='already locked in the board', code=4)
    print("  PASS: dispositions.refs and a file lock both answer it; both "
          "at once is refused as stale")


def test_disposition_keys_are_exact_and_padless_only():
    with tempfile.TemporaryDirectory() as tmp:
        plan = _plan(tmp, 'glob.json', dispositions={'refs': {
            '#*': 'every logo, by pattern'}})
        run_utils.check(_p1(ESP, plan),
                        refuse='names 1 block(s) this board does not have',
                        code=4)
        plan = _plan(tmp, 'padded.json', dispositions={'refs': {
            'C1': 'not a logo'}})
        run_utils.check(_p1(ESP, plan), refuse='answers PAD-LESS blocks only',
                        code=4)
    print("  PASS: a glob key and a pad-bearing key are both refused")


def test_the_pass_message_counts_blocks():
    """On a board whose every block is answered, the census the stage prints
    has BLOCKS as its denominator."""
    drv = _driver()
    with tempfile.TemporaryDirectory() as tmp:
        board = drv._tiny_board(os.path.join(tmp, 'b.kicad_pcb'),
                                ('U1', 'U2', 'LOGO1', 'LOGO2'),
                                padless=('LOGO1', 'LOGO2'),
                                locked=('LOGO2',))
        plan = os.path.join(tmp, 'p.json')
        doc = drv._zone_plan_doc(
            [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
              'note': 'both ICs'}])
        doc['dispositions']['refs'] = {'LOGO1': 'cosmetic'}
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh)
        r = run_utils.check([sys.executable, '-X', 'utf8', DRIVER, '--stage',
                             'P1', '--board', board, '--zone-plan', plan],
                            accept=True)
        out = r.stdout
        assert 'all 4 footprint(s) accounted for' in out, out[:900]
        assert ('2 pad-less, the seeder never moves them (1 locked, '
                '1 dispositioned)') in ' '.join(out.split()), out[:900]
    print("  PASS: the census counts 4 blocks, 2 of them pad-less")


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


def test_converge_poses_exits_4_with_json_and_no_traceback():
    for key, kind in ((LOGOS[2], 'unrankable'), ('NOPE1', 'not_on_board')):
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('converge.py'), 'poses', ESP,
                             '--ref', key], refuse='"refused"', code=4)
        assert 'Traceback' not in (r.stdout + r.stderr)
        start = r.stdout.index('{')
        doc = json.loads(r.stdout[start:r.stdout.rindex('}') + 1])
        assert doc['refused_kind'] == kind and doc['poses'] == [], doc
        assert key in doc['refused'] and 'knobs' in doc, doc
    print("  PASS: converge poses refuses at exit 4 with a reason, both kinds")


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
    print("  PASS: place_pose's snap path refuses a pad-less block at exit "
          "4 and writes nothing")


TESTS = [
    test_esp_prog_has_21_blocks_and_three_are_padless,
    test_p1_names_the_three_logos,
    test_the_padless_check_keeps_the_older_refusals_first,
    test_must_lock_does_not_answer_a_padless_block,
    test_a_zoned_padless_block_is_refused_as_inert,
    test_a_padless_block_with_a_courtyard_is_graded_once_locked,
    test_a_disposition_or_a_file_lock_answers_it,
    test_disposition_keys_are_exact_and_padless_only,
    test_the_pass_message_counts_blocks,
    test_rank_poses_refuses_instead_of_raising_a_bare_keyerror,
    test_converge_poses_exits_4_with_json_and_no_traceback,
    test_place_pose_snap_on_a_padless_block_is_a_refusal,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
