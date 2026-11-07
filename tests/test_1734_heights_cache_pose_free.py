#!/usr/bin/env python3
"""#1734: a heights measurement does not depend on where the parts stand.

The measurement cache key leaves part poses out on purpose, so every seed of
a placement reads one measurement. But `_measure` exported the board at its
own poses and read them twice:
  * the export's frame was fit to the poses by the median miss. On a pile,
    where parts are staged on a grid and more than half share one x, a frame
    that read the board's face as a plan axis tied and won: every part was
    measured on its side (an edge jack 4.34 mm tall read 15.24 mm), cached
    under the placed board's key, and read back by the next run on the
    placed board, which then graded a body through the board that is not
    there;
  * the export tessellates a curved model where it stands, so one part
    measured at two poses differed in a few raster cells.
`_canonical_poses` now stages every part for the export at a position that
depends on file order alone, its angle and side kept.

Cases:
  1. (no KiCad) the staged text of a board and of the same board with its
     parts moved and piled is the same, and keeps each part's angle;
  2. (kicad-cli) on a fresh cache, the pile measured FIRST and the placed
     board measured second share one cache entry, and the placed board reads
     its own heights and depth from it: its jack 4.34 mm tall, its body at
     board depth on the parts it is on the placed board; and a measurement
     made straight from the pile equals one made straight from the board.

Case 2 exits 77 (a skip, not a pass) when kicad-cli is absent.

    python3 tests/test_1734_heights_cache_pose_free.py
"""
import contextlib
import io
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import run_utils                                              # noqa: E402

RUN_ALL_TIMEOUT = 600
FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', '1603', 'edge_sma.kicad_pcb')
MAKE_UNPLACED = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                             'scripts', 'make_unplaced.py')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _pile(d):
    out = os.path.join(d, 'pile.kicad_pcb')
    run_utils.check([sys.executable, MAKE_UNPLACED, FIXTURE, out],
                    accept=True, allow=('WARNING',))
    return run_utils.evidence(out, 'the staged pile')


def _read(path):
    with open(path, encoding='utf-8') as fh:
        return fh.read()


def _angles(text):
    from kicad_parser import footprint_at_match, iter_footprint_blocks
    return [footprint_at_match(fp).group(3)
            for _s, _e, fp, _r, _k in iter_footprint_blocks(text)]


def test_staged_text_is_pose_free(d):
    from placement import heights as H
    run_utils.evidence(FIXTURE, 'tracked fixture')
    board, pile = _read(FIXTURE), _read(_pile(d))
    check('the fixture and its pile differ in their poses', board != pile)
    _canonical_poses = getattr(H, '_canonical_poses', None)
    check('the export is staged at poses of its own', _canonical_poses)
    if _canonical_poses is None:
        return
    a, b = _canonical_poses(board), _canonical_poses(pile)
    from kicad_parser import footprint_at_match, iter_footprint_blocks
    from placement.heights import _key

    def blocks(t):
        return [fp for _s, _e, fp, _r, _k in iter_footprint_blocks(t)]
    check('their footprints stage at the same poses for the export',
          [footprint_at_match(fp).group(0) for fp in blocks(a)]
          == [footprint_at_match(fp).group(0) for fp in blocks(b)])
    check('...and the two boards share one cache key',
          _key(board, 'cli', iter_footprint_blocks)
          == _key(pile, 'cli', iter_footprint_blocks))
    poses = [footprint_at_match(fp).group(1, 2) for fp in blocks(a)]
    check('every part stands at its own x and its own y',
          len({p[0] for p in poses}) == len(poses)
          and len({p[1] for p in poses}) == len(poses), poses)
    check('each part keeps its angle', _angles(a) == _angles(board),
          (_angles(a), _angles(board)))


def test_pile_first_then_board(d):
    from kicad_oracle import find_kicad_cli
    if not find_kicad_cli(warn=False):
        print('SKIP: kicad-cli not found')
        sys.exit(77)
    from kicad_parser import parse_kicad_pcb
    from model_export import model_dirs, resolve_cli, stage_models
    from placement import heights as H
    pile = os.path.join(d, 'pile.kicad_pcb')
    cache = os.path.join(d, 'cache')
    os.makedirs(cache)
    old = os.environ.get('XDG_CACHE_HOME')
    os.environ['XDG_CACHE_HOME'] = cache
    try:
        H._MEMO.clear()
        got = {}
        for name, path in (('pile', pile), ('board', FIXTURE)):
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                pcb = parse_kicad_pcb(path)
                got[name] = H.for_board(pcb, path)
            H._MEMO.clear()
    finally:
        if old is None:
            os.environ.pop('XDG_CACHE_HOME', None)
        else:
            os.environ['XDG_CACHE_HOME'] = old
    entries = os.listdir(os.path.join(cache, 'krt_heights'))
    check('the pile and the board share one cache entry', len(entries) == 1,
          entries)
    raw = got['board'].raw or {}
    check('the board read after the pile reads its jack 4.34 mm tall',
          (raw.get('top') or {}).get('J3') == 4.34, raw.get('top'))
    check('...and its bodies at board depth where they are',
          sorted(raw.get('depth') or {}) == ['J1', 'J2', 'J3'],
          sorted(raw.get('depth') or {}))

    def measure(path):
        text = _read(path)
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            pcb = parse_kicad_pcb(path)
        cli, _ = resolve_cli()
        bd = os.path.dirname(os.path.abspath(path))
        staged = stage_models(text, model_dirs(cli, bd), bd)[0]
        return H._measure(staged, cli, pcb, H.board_thickness(text))
    a, b = measure(FIXTURE), measure(pile)
    check('a measurement of the pile equals one of the board, raster for '
          'raster', a == b,
          sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k)))


def main():
    with tempfile.TemporaryDirectory() as d:
        for t in (test_staged_text_is_pose_free, test_pile_first_then_board):
            print(t.__name__)
            t(d)
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASSED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
