#!/usr/bin/env python3
"""Five defects seen in PR C's own run-32 stills, each pinned (#1036, #946).

  1. **The board keeps its share.** A 16:9 `split` film with the iso view
     left the board a 1400x342 strip. Landscape iso now goes in a side column
     and every lower box is capped so board + attempts band keep
     `BOARD_MIN_SHARE` of the height -- checked over layout x ratio x iso x
     band.
  2. **The iso view is contained, not cropped.** kicad-cli frames the board by
     the canvas HEIGHT and clips across, so a portrait request (the 9:16
     stacked iso half) cut the board at both sides. Checked end to end through
     `compose_two_panel` with a stand-in renderer that clips the same way,
     with the OLD request shape as the control.
  3. **The inventory follows the glide.** Mid-glide the box read the
     destination's "272 of 272 placed" over parts still in the pile.
  4. **No label overprints another** in the attempts band -- ticks and caption
     included.
  5. **The Python heap does not grow with frames x segments.** `Movie.chrome`
     kept a tuple of the live copper per frame; it now keeps a position in an
     edit log. The slope is measured against the old storage as a control,
     and every frame's resolved copper is checked equal to what it was.

Needs Pillow; renders small in-repo boards. The run-32 value check in (3)
runs only when wk/run32 is present and says so when it is not.
"""
import json
import os
import shutil
import sys
import tempfile
import tracemalloc

RUN_ALL_TIMEOUT = 900

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import animate_route as A          # noqa: E402
import frame_layout as FL          # noqa: E402

KF = os.path.join(ROOT, 'kicad_files')
ROUTED = os.path.join(KF, 'routed_output.kicad_pcb')
SEED = os.path.join(KF, 'interf_u_unrouted.kicad_pcb')
PLACED = os.path.join(KF, 'interf_u_unrouted_placed.kicad_pcb')
RUN32 = os.path.join(ROOT, 'wk', 'run32')

_FAIL = []
_NOTES = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


# --------------------------------------------------------------------------
def test_the_board_keeps_its_share_at_every_layout_and_ratio():
    _mark = len(_FAIL)
    bb = (50.0, 71.0, 130.0, 120.0)            # glasgow's 1.63:1 outline
    n, worst = 0, (9.0, None)
    for lk in FL.LAYOUTS:
        for rk, ratio in FL.RATIOS.items():
            for iso in (False, True):
                for band in (0, 1):
                    n += 1
                    g0 = FL.plan_frame(bb, layout=lk, ratio=ratio, size=1400,
                                       panel=True, iso=iso)
                    tp = int(0.16 * g0.frame.h) if band else 0
                    g = FL.plan_frame(bb, layout=lk, ratio=ratio, size=1400,
                                      panel=True, iso=iso, track_px=tp)
                    tr = g.track.h if g.track else 0
                    share = (g.board.h + tr) / float(g.frame.h)
                    wshare = g.board.w / float(g.frame.w)
                    if share < worst[0]:
                        worst = (share, (lk, rk, iso, band))
                    if share < FL.BOARD_MIN_SHARE - 0.01:
                        fail('%s/%s iso=%s band=%s: board+band %.2f of the '
                             'height (floor %.2f)'
                             % (lk, rk, iso, band, share, FL.BOARD_MIN_SHARE))
                    if wshare < 0.5:
                        fail('%s/%s iso=%s: board %.2f of the width'
                             % (lk, rk, iso, wshare))
    # the case the stills showed
    g = FL.plan_frame(bb, layout='split', ratio=16 / 9.0, size=1400,
                      panel=True, iso=True, track_px=126)
    if g.board.h < 0.7 * g.frame.h:
        fail('16:9 split + iso: board %dx%d in a %dx%d frame -- still a strip'
             % (g.board.w, g.board.h, g.frame.w, g.frame.h))
    if not g.panel_split or g.panel_split[0].x < g.board.w:
        fail('16:9 split + iso: the 3D view is not in a side column')
    if len(_FAIL) == _mark:
        print('  PASS: %d plans; worst board+band share %.2f (%s); 16:9 split '
              '+ iso board is now %dx%d' % (n, worst[0], worst[1],
                                            g.board.w, g.board.h))


# --------------------------------------------------------------------------
BOARD_ASPECT = 1.6


def _fake_kicad(board, out_png, cli, width, height, rotate=None, **kw):
    """kicad-cli's framing, reproduced: fit the board to the canvas HEIGHT
    with a margin and CLIP whatever does not fit across."""
    im = Image.new('RGBA', (width, height), (0, 0, 0, 0))
    bh = int(height * 0.7)
    bw = int(bh * BOARD_ASPECT)
    x0 = (width - bw) // 2
    y0 = (height - bh) // 2
    ImageDraw.Draw(im).rectangle([x0, y0, x0 + bw - 1, y0 + bh - 1],
                                 fill=(200, 60, 60, 255))
    im.save(out_png)
    return out_png, ''


def _iso_content_aspect(min_aspect=None):
    import kicad_iso_render as kir
    import movie_panels as MP
    saved = (kir.resolve_cli, kir.resolve_models, kir.render_iso,
             kir.render_many, MP.ISO_MIN_REQUEST_ASPECT)
    kir.resolve_cli = lambda _e=None: ('fake-cli', '')
    kir.resolve_models = lambda b, dirs=None: {'total': 10, 'found': 10}
    kir.render_iso = _fake_kicad

    def _many(jobs, cli, workers=None, **kw):
        return {k: _fake_kicad(b, o, cli, kw['width'], kw['height'])
                for k, b, o, _r in jobs}
    kir.render_many = _many
    if min_aspect is not None:
        MP.ISO_MIN_REQUEST_ASPECT = min_aspect
    try:
        box = FL.Box(0, 400, 330, 392)       # the 9:16 stacked iso half
        frames = [Image.new('RGB', (788, 1400), (240, 240, 240))
                  for _ in range(4)]
        marks = [('s', ROUTED, 0, 4)]
        out, rep = MP.compose_two_panel(frames, marks, ROUTED,
                                        MP.IsoOpts(max_renders=1,
                                                   theme='light'), box=box)
        if rep.get('state') != 'ran':
            return None, rep
        f = out[0].crop((box.x, box.y, box.x + box.w, box.y + box.h))
        red = f.convert('RGB').point(lambda v: 255 if v > 150 else 0)
        # the stand-in board is the only saturated red in the panel
        px = f.convert('RGB').load()
        xs, ys = [], []
        for y in range(f.height):
            for x in range(f.width):
                r_, g_, b_ = px[x, y]
                if r_ > 150 and g_ < 120 and b_ < 120:
                    xs.append(x)
                    ys.append(y)
        del red
        if not xs:
            return None, rep
        bw = max(xs) - min(xs) + 1
        bh = max(ys) - min(ys) + 1
        inside = (min(xs) >= 0 and max(xs) < box.w and min(ys) >= 0
                  and max(ys) < box.h)
        return (bw / float(bh), inside), rep
    finally:
        (kir.resolve_cli, kir.resolve_models, kir.render_iso,
         kir.render_many, MP.ISO_MIN_REQUEST_ASPECT) = saved


def test_the_iso_view_is_contained_not_cropped():
    _mark = len(_FAIL)
    got, rep = _iso_content_aspect()
    if got is None:
        fail('BROKEN: the iso panel did not run (%r)' % rep)
        return
    aspect, inside = got
    if not inside:
        fail('the iso content leaves its box')
    if abs(aspect - BOARD_ASPECT) > 0.1 * BOARD_ASPECT:
        fail('the board came out %.2f:1, not its own %.2f:1 -- clipped'
             % (aspect, BOARD_ASPECT))
    # CONTROL: the old request shape (the box's own, portrait) must be SEEN
    # to clip, or this check proves nothing.
    old, _r = _iso_content_aspect(min_aspect=0.0)
    if old is None or abs(old[0] - BOARD_ASPECT) <= 0.1 * BOARD_ASPECT:
        fail('BROKEN: the portrait-request control did not clip (%r), so '
             'the check cannot see the defect' % (old,))
    if len(_FAIL) == _mark:
        print('  PASS: board %.2f:1 inside a 330x392 box (control, the old '
              'portrait request: %.2f:1, clipped)' % (aspect, old[0]))


# --------------------------------------------------------------------------
def _glide_inventories(a, b):
    """(label, inventory) per frame for a stage glide from a to b, split
    layout so the lower box exists."""
    import movie_camera as MC
    seen = []
    orig = A.Movie._note_chrome

    def _spy(self, label):
        seen.append((label, self.inventory))
        return orig(self, label)
    A.Movie._note_chrome = _spy
    try:
        st = MC.Stage(MC.synth_rounds([a, b]), '', tween=4)
        A.build_boards([('a', a, None), ('b', b, None)], b, 240, 1, None, 2,
                       6, stage=st, layout='split', aspect='16:9')
    finally:
        A.Movie._note_chrome = orig
    return seen


def _placed(inv):
    return sum(x for x, _y in (inv or {}).values())


def test_the_inventory_follows_the_glide():
    _mark = len(_FAIL)
    seen = _glide_inventories(SEED, PLACED)
    moving = [i for i, (lb, _v) in enumerate(seen) if 'moving' in lb]
    if len(moving) < 2:
        fail('BROKEN: no glide frames (%r)' % [lb for lb, _v in seen][:12])
        return
    before = seen[moving[0] - 1][1]
    for i in moving[:-1]:
        if seen[i][1] is not before:
            fail('frame %d (%s) mid-glide already reads the destination '
                 'inventory' % (i, seen[i][0]))
            break
    if seen[moving[-1]][1] is before:
        fail('the landing frame still reads the source inventory')
    # the VALUES, on the board the stills showed, when it is here
    ua = os.path.join(RUN32, 'glasgow_unplaced.kicad_pcb')
    ub = os.path.join(RUN32, 'placed_v2.kicad_pcb')
    if os.path.isfile(ua) and os.path.isfile(ub):
        s2 = _glide_inventories(ua, ub)
        mv = [i for i, (lb, _v) in enumerate(s2) if 'moving' in lb]
        mid = _placed(s2[mv[len(mv) // 2]][1])
        land = _placed(s2[mv[-1]][1])
        if mid >= land:
            fail('run 32 mid-glide reads %d placed, the landing %d' % (mid,
                                                                     land))
        else:
            print('    run 32: mid-glide %d placed, landing %d' % (mid, land))
    else:
        _NOTES.append('run-32 value check not run: wk/run32 absent')
        print('    (run-32 value check not run: wk/run32 absent)')
    if len(_FAIL) == _mark:
        print('  PASS: %d glide frames read the source board; the landing '
              'frame reads the destination' % (len(moving) - 1))


# --------------------------------------------------------------------------
class _Rec(object):
    """An ImageDraw stand-in that records every text box it draws."""

    def __init__(self, d):
        self._d = d
        self.boxes = []

    def text(self, xy, txt, *a, **kw):
        font = kw.get('font')
        anchor = kw.get('anchor')
        self.boxes.append((txt, self._d.textbbox(xy, txt, font=font,
                                                 anchor=anchor)))
        return self._d.text(xy, txt, *a, **kw)

    def __getattr__(self, k):
        return getattr(self._d, k)


def test_no_band_label_overprints_another():
    _mark = len(_FAIL)
    import movie_attempts as MA
    # run 32's opening: a pile at 12703, then a fast run of drops
    scores = [12703, 267, 251, 239, 239, 239, 238, 604, 41, 40, 38, 33, 32,
              31, 30, 30, 29, 28, 28, 27, 25, 23, 21, 21, 20, 19]
    rows = tuple(MA.Attempt(i, 'l%d' % i, 'completion', i - 1 if i else None,
                            True, False, float(s), s == 0, None)
                 for i, s in enumerate(scores))
    tracks = [('fixture', MA.Track(rows, 'blocking (lower better)',
                                   'converge', 'fixture'))]
    led = os.path.join(RUN32, 'ledger.jsonl')
    if os.path.isfile(led):
        tracks.append(('run32 ledger', MA.attempts_from_converge_ledger(led)))
    else:
        _NOTES.append('run-32 ledger absent: labels checked on the fixture')

    def _over(a, b):
        return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])
    n = placed_n = wanted_n = 0
    for name, track in tracks:
        for w, h in ((1400, 126), (788, 224), (600, 90)):
            im = Image.new('RGB', (w, h))
            rec = _Rec(ImageDraw.Draw(im))
            dbg = {}
            if not MA.draw_track(rec, FL.Box(0, 0, w, h), track, debug=dbg):
                fail('%s: the band declined at %dx%d' % (name, w, h))
                continue
            # every text box, read INDEPENDENTLY of the drawer's own list
            bx = rec.boxes
            n += len(bx)
            for i in range(len(bx)):
                for j in range(i + 1, len(bx)):
                    if _over(bx[i][1], bx[j][1]):
                        fail('%s %dx%d: text %r overprints %r'
                             % (name, w, h, bx[i][0], bx[j][0]))
            # ...and every record label against every mark: node, kept
            # ring, ungraded rail tick, tick label, caption
            for txt, rect in dbg.get('labels', ()):
                for o in dbg.get('obstacles', ()):
                    if _over(rect, o):
                        fail('%s %dx%d: label %r sits on a mark at %r'
                             % (name, w, h, txt, o))
                        break
            placed_n += len(dbg.get('labels', ()))
            wanted_n += len(dbg.get('wanted', ()))
    if len(_FAIL) == _mark:
        print('  PASS: %d text boxes over %d band(s), none overprinting; '
              '%d of %d record labels found a free spot, none on a mark'
              % (n, 3 * len(tracks), placed_n, wanted_n))


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

    def _keep(frames, geom, r, chrome=None, iso_in_panel=False):
        held['chrome'] = chrome
        return spy(frames, geom, r, chrome, iso_in_panel=iso_in_panel)
    A._compose_into_frame = _keep
    tracemalloc.start()
    try:
        with frame_spool.FrameSpool() as sp:
            A.build_boards([('route', board, tr)], board, 200, 1, None, 2, 6,
                           layout='split', aspect='16:9', frames_sink=sp,
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
    test_the_board_keeps_its_share_at_every_layout_and_ratio,
    test_the_iso_view_is_contained_not_cropped,
    test_the_inventory_follows_the_glide,
    test_no_band_label_overprints_another,
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
