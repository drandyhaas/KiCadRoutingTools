#!/usr/bin/env python3
"""A fanout's `drc_grazes` counts the grazes of the copper IT laid, not the board's.

  python3 tests/test_fanout_grazes_scope.py

Both fanout CLIs DRC their written board for the JSON_SUMMARY's
`drc_grazes`, and the routing skill re-runs THAT fanout with a thinner track
or a smaller via while a class is > 0. They used to count every violation on
the board, so each step re-reported every earlier step's: on eis the U4 QFN
fanout at 0.2 reported 616 grazes, all three BGA fanouts', none its own, and
a planner following the skill would chase copper no re-run of U4 can move.
Pinned, for qfn_fanout (haasoscope U2) and bga_fanout (interf_u U9):

1. a graze the INPUT board already had (two foreign tracks 0.05 mm apart,
   well away from the part) leaves `drc_grazes` exactly as on the clean
   board, and the summary says what it counts (`scope`);
2. the graze is really on the written board (a whole-board DRC finds it), so
   check 1 is not passing on a board that lost it;
3. the scoping counts what is new: the same written board scored against
   the CLEAN input's copper (as if the step had laid those tracks) counts
   the graze -- so check 1 is not a scope that counts nothing.
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from run_utils import tool  # noqa: E402

# (tool, board, part, clearance the CLI grades at, (x0, x1, y) of the graze, nets)
LEGS = [
    ('qfn_fanout.py', 'haasoscope_pro_max_test.kicad_pcb', 'U2', 0.1,
     (135.0, 145.0, 75.0), (2, 3)),
    ('bga_fanout.py', 'interf_u_unrouted_placed.kicad_pcb', 'U9', 0.25,
     (90.0, 100.0, 40.0), (1, 2)),
]


def _inject(src, dst, x0, x1, y, nets):
    """`src` plus two foreign F.Cu tracks 0.25 apart (0.05 mm of air)."""
    with open(src, encoding='utf-8') as fh:
        text = fh.read().rstrip()
    assert text.endswith(')')
    segs = ''.join(
        '\t(segment (start %s %s) (end %s %s) (width 0.2) (layer "F.Cu") '
        '(net %d) (uuid "graze-%d"))\n' % (x0, y + 0.25 * i, x1, y + 0.25 * i, n, i)
        for i, n in enumerate(nets))
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(text[:-1] + segs + ')\n')


def _summary(cli, board, part, out):
    r = subprocess.run([sys.executable, tool(cli), board, '--component', part,
                        '--output', out], capture_output=True, text=True,
                       cwd=ROOT, timeout=600)
    rows = [json.loads(l.split('JSON_SUMMARY: ', 1)[1])
            for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]
    if r.returncode != 0 or not rows:
        raise RuntimeError(f"{cli} exited {r.returncode} with "
                           f"{len(rows)} JSON_SUMMARY line(s): {r.stderr[-400:]}")
    return rows[-1]


def _quiet(fn):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn()


def leg(cli, board, part, clearance, at, nets, d, fails):
    src = os.path.join(ROOT, 'kicad_files', board)
    if not os.path.isfile(src):
        fails.append(f"missing board {src}")
        return
    from kicad_parser import parse_kicad_pcb
    from check_drc import run_drc
    clean = os.path.join(d, 'clean_' + board)
    dirty = os.path.join(d, 'dirty_' + board)
    shutil.copyfile(src, clean)
    _inject(src, dirty, *at, nets)
    try:
        s0 = _summary(cli, clean, part, os.path.join(d, 'out_clean_' + board))
        out = os.path.join(d, 'out_dirty_' + board)
        s1 = _summary(cli, dirty, part, out)
    except Exception as e:
        fails.append(f"{cli} {part}: {e}")
        return
    g0, g1 = s0.get('drc_grazes') or {}, s1.get('drc_grazes') or {}
    if 'error' in g0 or 'error' in g1:
        fails.append(f"{cli} {part}: the graze audit errored ({g0} / {g1})")
        return
    # 1. the input's graze is not the step's
    keys = ('pad_via', 'via_segment', 'pad_segment', 'segment_segment', 'total')
    if [g1.get(k) for k in keys] != [g0.get(k) for k in keys]:
        fails.append(f"{cli} {part}: a graze the input already had changed "
                     f"drc_grazes ({g0.get('total')} on the clean board, "
                     f"{g1.get('total')} with it)")
    if not g1.get('scope'):
        fails.append(f"{cli} {part}: drc_grazes does not say what it counts (no 'scope')")
    # 2. the graze is on the written board
    whole = _quiet(lambda: run_drc(out, clearance=clearance, quiet=True,
                                   max_print=0, check_sizes=False))
    ss = [r for r in whole if r.get('type') == 'segment-segment'
          and abs(r['loc1'][1] - at[2]) < 0.5]
    if not ss:
        fails.append(f"{cli} {part}: the injected graze is not on the written board "
                     f"-- check 1 measured nothing")
    # 3. the same board scored as if the step had laid the tracks
    try:
        from check_drc import copper_keys, fanout_drc_grazes
    except ImportError as e:
        fails.append(f"{cli} {part}: {e}")
        return
    base = copper_keys(_quiet(lambda: parse_kicad_pcb(clean)))
    g2 = fanout_drc_grazes(out, clearance, base)
    if g2.get('segment_segment', 0) < g0.get('segment_segment', 0) + len(ss):
        fails.append(f"{cli} {part}: scored against the clean input, the new "
                     f"tracks' graze is not counted ({g2})")


def main():
    print('=' * 60)
    print("fanout drc_grazes: the copper this step laid, not the whole board")
    print('=' * 60)
    fails = []
    d = tempfile.mkdtemp(prefix='grazes_scope_')
    try:
        for args in LEGS:
            leg(*args, d, fails)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print("PASS: on qfn_fanout and bga_fanout a graze the input already had leaves "
          "drc_grazes unchanged (it is on the written board, and the same board "
          "scored against the clean input counts it); the summary names its scope")
    return 0


if __name__ == '__main__':
    sys.exit(main())
