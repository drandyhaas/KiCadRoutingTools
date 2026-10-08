#!/usr/bin/env python3
"""#1113: `rank_rotations.py` end to end on the committed run-29 pile.

The ranker is only as honest as its arms, so each one is checked against an
independent control: for every angle, a hand-written intent declaring U1 at
that angle and a direct `place_seed` run must reproduce the ranker's row --
crossings, hpwl, unseated, grade errors, and the pose digest of the written
board. Then:

* a second run with the angles in another order reproduces every row (the
  arms are independent and deterministic);
* `--write-intent` declares the winning angle, and `--write-best` copies the
  winning board WITH its project sibling;
* the arms differ: at least two distinct crossings values, so the ranking is
  not a tie that the ladder order decided (non-vacuity).

    python3 tests/test_1113_rank_rotations_e2e.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb            # noqa: E402
from placement import floorplan as fp               # noqa: E402
from placement.provenance import file_pose_digest   # noqa: E402
import run_utils                                    # noqa: E402

RUN_ALL_TIMEOUT = 1800

PILE = os.path.join(TESTS_DIR, 'fixtures', '959', 'run29_pile.kicad_pcb')
ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
TOOL = run_utils.tool('rank_rotations.py')
PLACE_SEED = run_utils.tool('place_seed.py')
KEYS = ('crossings', 'hpwl', 'unseated', 'grade_errors')


def _run(argv):
    r = subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    return r


def _summary(stdout):
    m = re.search(r'^JSON_SUMMARY: (.*)$', stdout, re.M)
    assert m, stdout[-1500:]
    return json.loads(m.group(1))


def test_each_arm_is_a_hand_run_seed_and_the_ranking_is_stable():
    run_utils.evidence(PILE, 'the run-29 pile')
    with tempfile.TemporaryDirectory() as td:
        doc = fp.emit_intent(parse_kicad_pcb(PILE), PILE, decaps_from=ESP)
        ipath = os.path.join(td, 'intent.json')
        with open(ipath, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1)
        out1 = os.path.join(td, 'r1')
        best = os.path.join(td, 'best.kicad_pcb')
        wi = os.path.join(td, 'intent_rot.json')
        r = _run([TOOL, PILE, '--intent', ipath, '--ref', 'U1', '--seeds',
                  '0', '--out-dir', out1, '--write-best', best,
                  '--write-intent', wi])
        assert r.returncode == 0, r.stdout[-1500:] + r.stderr[-1500:]
        with open(os.path.join(out1, 'rotations.json'), encoding='utf-8') as fh:
            res = json.load(fh)
        rows = {row['rotation']: row for row in res['rows']}
        assert sorted(rows) == [0.0, 90.0, 180.0, 270.0], sorted(rows)
        xs = {rows[a]['crossings'] for a in rows}
        assert len(xs) >= 2, xs          # the arms really differ

        # the control: U1 declared by hand, place_seed run directly
        for ang, row in sorted(rows.items()):
            hand = json.loads(json.dumps(doc))
            hand['blocks'] = list(hand.get('blocks') or []) + [
                {'name': 'control', 'refs': ['U1'], 'rotation': ang}]
            hand['min_reader'] = max(int(hand.get('min_reader') or 0), 5)
            hp = os.path.join(td, f'hand_{ang:g}.json')
            with open(hp, 'w', encoding='utf-8') as fh:
                json.dump(hand, fh, indent=1)
            hb = os.path.join(td, f'hand_{ang:g}.kicad_pcb')
            h = _run([PLACE_SEED, PILE, hb, '--intent', hp, '--seed', '0',
                      '--group-by', 'auto'])
            assert h.returncode in (0, 4), h.stdout[-800:] + h.stderr[-800:]
            s = _summary(h.stdout)
            got = {k: row[k] for k in KEYS}
            want = {k: s.get(k) for k in KEYS}
            assert got == want, (ang, got, want)
            assert row['pose_digest'] == file_pose_digest(hb), ang
            assert row['rotation_applied'] and row['hard_fail'] is None, row

        # the CONTROL is the undeclared seed: a plain place_seed run
        ctl = res['control']
        hc = os.path.join(td, 'plain_0.kicad_pcb')
        h = _run([PLACE_SEED, PILE, hc, '--intent', ipath, '--seed', '0',
                  '--group-by', 'auto'])
        assert h.returncode in (0, 4), h.stdout[-800:]
        assert ctl['rows'][0]['pose_digest'] == file_pose_digest(hc), ctl
        assert ctl['crossings'] == _summary(h.stdout)['crossings'], ctl
        assert res['separated'] is None          # one seed: no spread

        # the same arms in another order: the same rows
        out2 = os.path.join(td, 'r2')
        r2 = _run([TOOL, PILE, '--intent', ipath, '--ref', 'U1', '--seeds',
                   '0', '--rotations', '270', '90', '--out-dir', out2])
        assert r2.returncode == 0, r2.stdout[-800:] + r2.stderr[-800:]
        with open(os.path.join(out2, 'rotations.json'), encoding='utf-8') as fh:
            res2 = json.load(fh)
        for row in res2['rows']:
            a = row['rotation']
            assert {k: row[k] for k in KEYS + ('pose_digest',)} == \
                {k: rows[a][k] for k in KEYS + ('pose_digest',)}, a
        # the input angle (0) was not ranked there: no input baseline
        assert _summary(r2.stdout)['input_crossings'] is None, \
            r2.stdout[-400:]

        # the written outputs
        win = res['best_rotation']
        assert win == res['ranking'][0] and win is not None, res
        assert os.path.isfile(best) and os.path.isfile(
            os.path.splitext(best)[0] + '.kicad_pro'), best
        assert file_pose_digest(best) == rows[win]['pose_digest']
        with open(wi, encoding='utf-8') as fh:
            wdoc = json.load(fh)
        intent = fp.intent_from_dict(wdoc, PILE)
        blocks, _ = fp.resolve_blocks(intent, parse_kicad_pcb(PILE),
                                      ('kicad', 'sheet'))
        assert fp.rotations_for_ref(intent, blocks)['U1'][0] == win
        assert _summary(r.stdout)['best_rotation'] == win
    print(f"  PASS: 4 arms each equal a hand-run seed; reordered arms "
          f"reproduce; best {win:g} written (crossings "
          f"{ {a: rows[a]['crossings'] for a in sorted(rows)} })")


TESTS = [
    test_each_arm_is_a_hand_run_seed_and_the_ranking_is_stable,
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
