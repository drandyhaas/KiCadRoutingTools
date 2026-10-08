#!/usr/bin/env python3
"""Five defects seen in PR C's own run-32 stills, each pinned (#1036, #946).

  1. **The board keeps its share.** A 16:9 `split` film left the board a
     1400x342 strip. A landscape frame with a band now puts its panel in a
     side column and every lower box is capped so board + attempts band keep
     `STAGE3D_BOARD_H_FRAC` of the height -- checked over ratio x band.
  2. (The iso view's containment check went with the iso panel: stage3d is
     the only film layout.)
  3./4. (The glide inventory and the attempts band's labels went with the
     retired layouts: the stage3d layer column shows no inventory, and the
     attempts band is not drawn. The glide's landing frame is pinned by
     test_1042's placement panels.)
  5. **The Python heap does not grow with frames x segments.** `Movie.chrome`
     kept a tuple of the live copper per frame; it now keeps a position in an
     edit log. The slope is measured against the old storage as a control,
     and every frame's resolved copper is checked equal to what it was.

Needs Pillow; renders small in-repo boards.
"""
import json
import os
import shutil
import sys
import tempfile
import tracemalloc

# stage3d is the only film layout, so an unnamed layout is a stage3d
# frame. These tests grade the 2D board, not the Node/Chromium 3D
# render: set before env_knobs is read.
os.environ.setdefault('KICAD_MOVIE_BOARD3D', '2d')

RUN_ALL_TIMEOUT = 900

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import importlib.util                                           # noqa: E402
if importlib.util.find_spec('PIL') is None:
    print('SKIP: needs Pillow')
    sys.exit(77)

import animate_route as A          # noqa: E402
import frame_layout as FL          # noqa: E402

KF = os.path.join(ROOT, 'kicad_files')
ROUTED = os.path.join(KF, 'routed_output.kicad_pcb')
SEED = os.path.join(KF, 'interf_u_unrouted.kicad_pcb')
PLACED = os.path.join(KF, 'interf_u_unrouted_placed.kicad_pcb')

_FAIL = []
_NOTES = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


# --------------------------------------------------------------------------
def test_the_board_keeps_its_share_at_every_ratio():
    """On the stage3d frame (the only layout) -- the sweep used to run
    over every layout; the retired ones are gone."""
    _mark = len(_FAIL)
    bb = (50.0, 71.0, 130.0, 120.0)            # glasgow's 1.63:1 outline
    n, worst = 0, (9.0, None)
    for lk in ('stage3d',):
        for rk, ratio in FL.RATIOS.items():
            for band in (0, 1):
                n += 1
                g0 = FL.plan_frame(bb, ratio=ratio, size=1400)
                tp = int(0.16 * g0.frame.h) if band else 0
                g = FL.plan_frame(bb, ratio=ratio, size=1400, track_px=tp)
                tr = g.track.h if g.track else 0
                share = (g.board.h + tr) / float(g.frame.h)
                wshare = g.board.w / float(g.frame.w)
                if share < worst[0]:
                    worst = (share, (lk, rk, band))
                if share < FL.STAGE3D_BOARD_H_FRAC - 0.01:
                    fail('%s/%s band=%s: board+band %.2f of the '
                         'height (floor %.2f)'
                         % (lk, rk, band, share, FL.STAGE3D_BOARD_H_FRAC))
                if wshare < 0.5:
                    fail('%s/%s: board %.2f of the width'
                         % (lk, rk, wshare))
    # the case the stills showed
    g = FL.plan_frame(bb, ratio=16 / 9.0, size=1400, track_px=126)
    if g.board.h < 0.7 * g.frame.h:
        fail('16:9 + band: board %dx%d in a %dx%d frame -- still a '
             'strip' % (g.board.w, g.board.h, g.frame.w, g.frame.h))
    if g.panel is None or g.panel.x < g.board.w:
        fail('16:9 + band: the panel is not in a side column')
    if len(_FAIL) == _mark:
        print('  PASS: %d plans; worst board+band share %.2f (%s); 16:9 '
              '+ band board is %dx%d' % (n, worst[0], worst[1],
                                         g.board.w, g.board.h))


# --------------------------------------------------------------------------
def _trace(board, n, path):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(board)
    layers = list(pcb.board_info.copper_layers)
    segs, _v = A._board_rows(pcb, layers)
    ev = [{'event': 'route', 'net_name': 'n%d' % i, 'add_s': [r]}
          for i, r in enumerate(segs[:n])]
    with open(path, 'w') as f:
        json.dump({'layers': layers, 'events': ev}, f)


def _peak(n, tmp, old_storage=False, check=False):
    import frame_spool
    board = os.path.join(tmp, 'b.kicad_pcb')
    if not os.path.isfile(board):
        shutil.copy(ROUTED, board)
    tr = os.path.join(tmp, 'b_%d.json' % n)
    _trace(board, n, tr)
    orig = A.Movie._note_chrome
    mirror = []

    def _old(self, label):
        orig(self, label)
        # the pre-fix storage: a full copy of the live copper per frame
        self.chrome[-1]['live'] = tuple(self.live_s.values())
        self.chrome[-1]['live_v'] = tuple(self.live_v.values())

    def _chk(self, label):
        orig(self, label)
        mirror.append((len(self.chrome) - 1,
                       tuple(id(x) for x in self.live_s.values())))
    if old_storage:
        A.Movie._note_chrome = _old
    elif check:
        A.Movie._note_chrome = _chk
    held = {}
    spy = A._compose_into_frame

    def _keep(frames, geom, r, chrome=None):
        held['chrome'] = chrome
        return spy(frames, geom, r, chrome)
    A._compose_into_frame = _keep
    tracemalloc.start()
    try:
        with frame_spool.FrameSpool() as sp:
            A.build_boards([('route', board, tr)], board, 200, 1, None, 2, 6,
                           aspect='16:9', frames_sink=sp, board3d='2d',
                           max_frames=0)
            nf = len(sp)
        _c, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
        A.Movie._note_chrome = orig
        A._compose_into_frame = spy
    bad = 0
    if check:
        ch = held.get('chrome') or []
        for i, ids in mirror:
            got = tuple(id(x) for x in A._live(ch[i]['live']))
            if got != ids:
                bad += 1
    return nf, peak, bad


def test_the_heap_does_not_grow_with_frames_x_segments():
    _mark = len(_FAIL)
    tmp = tempfile.mkdtemp(prefix='t1036r_')
    try:
        f1, p1, _ = _peak(300, tmp)
        f2, p2, _ = _peak(900, tmp)
        c1, q1, _ = _peak(300, tmp, old_storage=True)
        c2, q2, _ = _peak(900, tmp, old_storage=True)
        _n, _p, bad = _peak(200, tmp, check=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    slope = (p2 - p1) / float(max(1, f2 - f1))
    cslope = (q2 - q1) / float(max(1, c2 - c1))
    print('    edit log  : %d -> %d frames, peak %.1f -> %.1f MB, %.0f B/frame'
          % (f1, f2, p1 / 1e6, p2 / 1e6, slope))
    print('    CONTROL (per-frame copies): %d -> %d frames, peak %.1f -> '
          '%.1f MB, %.0f B/frame' % (c1, c2, q1 / 1e6, q2 / 1e6, cslope))
    # What the old storage added on top of the log is the frames x segments
    # term, so it must grow SUPERLINEARLY with the frame count (each frame
    # copies a live set that is itself growing): 3x the frames here is ~9x
    # the copies. If it does not, this measurement cannot see the defect.
    x1, x2 = q1 - p1, q2 - p2
    ratio = x2 / float(max(1, x1))
    frames_ratio = f2 / float(max(1, f1))
    print('    old-storage excess: %.2f MB at %d frames, %.2f MB at %d '
          '(x%.1f for x%.1f the frames)' % (x1 / 1e6, f1, x2 / 1e6, f2,
                                            ratio, frames_ratio))
    if ratio < 2 * frames_ratio:
        fail('BROKEN: the old-storage control grew only x%.1f for x%.1f the '
             'frames, so it does not show the frames x segments term'
             % (ratio, frames_ratio))
    if (p2 - p1) * 2 > (q2 - q1):
        fail('the edit log grew %.1f MB against the old storage %.1f MB -- '
             'not clearly flatter' % ((p2 - p1) / 1e6, (q2 - q1) / 1e6))
    if bad:
        fail('%d frame(s) resolved different copper than they had' % bad)
    if len(_FAIL) == _mark:
        print('  PASS: %.0f B/frame against the old %.0f B/frame; every '
              'frame resolves the copper it had' % (slope, cslope))


TESTS = (
    test_the_board_keeps_its_share_at_every_ratio,
    test_the_heap_does_not_grow_with_frames_x_segments,
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
    print('all %d checks passed%s' % (len(TESTS), ('; NOTE: ' + '; '.join(
        _NOTES)) if _NOTES else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
