#!/usr/bin/env python3
"""Every frame of a film goes through ONE path, flipped or not (#1082-#1085).

A Stage flip to the back side used to hand its frames to a second render path
(`Stage._snap`'s mirrored branch, `_emit_flip`) that appended straight to
`movie.frames`. Four defects came out of that one bypass, all measured on the
chain in `film_chain_1081.py` (a flip at frames 35..41 of 73):

  * **#1082** -- no chrome record for those frames: 73 frames, 42 records, so
    the rail and the layer strip of 38 frames were drawn from ANOTHER frame's
    record (the flip read the copper of a step that had not happened yet);
  * **#1083** -- the caption stamped over the board on a layout whose rail
    already carries it (31 stamps);
  * **#1084** -- no `overlays=`, so a back-side glide drew no ghost or arrow;
  * **#1085** -- the copper a routing step reveals after the flip went
    through Movie's own path, which never mirrored: the hook meant to
    (`Stage.exit_step`) had no caller, so the film read B, F, B.

The fix mirrors inside `Movie._push_frame`, the one place a frame joins the
film. This file pins each symptom on the REAL `build_boards` + `Stage` path,
plus the stage-state record #1081's 3D board is rebuilt from, which must be
exactly as long as the film.
"""
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

try:
    from PIL import ImageChops, ImageOps
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import film_chain_1081 as FC                                    # noqa: E402
import animate_route as A                                       # noqa: E402
import route_render as RR                                       # noqa: E402

_FAIL = []


def _check(ok, msg):
    print('  %s %s' % ('ok  ' if ok else 'FAIL', msg))
    if not ok:
        _FAIL.append(msg)


def _shots(st, kind):
    return [(a, b) for k, a, b in st.frame_log() if k == kind]


def test_one_chrome_record_and_one_stage_record_per_frame():
    """#1082: the rail and the strip of frame i are frame i's."""
    with FC.Chain() as c:
        out = {}
        frames, m, st, _g = FC.film(c.boards, stage_out=out)
        n = len(frames)
        _check(bool(_shots(st, 'flip')), 'the fixture flips (%s)'
               % [k for k, _a, _b in st.frame_log()])
        _check(len(m.chrome) == n,
               'one chrome record per frame: %d records, %d frames'
               % (len(m.chrome), n))
        _check(len(out.get('log') or ()) == n,
               'one stage record per frame: %d records, %d frames'
               % (len(out.get('log') or ()), n))
        # The flip happens on a copper-free board (S1 -> S2 are stripped), so
        # a flip frame's layer strip must show NO copper. Before the fix it
        # read the records the later copper reveal wrote: 151, then 604.
        a, b = _shots(st, 'flip')[0]
        live = [len(A._live(m.chrome[i]['live'])) for i in range(a, b)]
        _check(live and not any(live),
               'the flip frames\' strip copper is the board\'s (0): %s' % live)
        kinds = [r['kind'] for r in out['log'][a:b]]
        _check(kinds == ['flip'] * (b - a),
               'the stage record calls the flip frames flips: %s' % kinds)
        # The poses the 3D board reads. `moving` names what glides on each
        # glide frame (it was never set: 0 of 73 records), and every epoch's
        # table holds RESTING poses -- a glide mutates the footprints in
        # place, so the first table of a glide board used to catch C26 at a
        # pose no board has (the phase-1 verifier's finding).
        from kicad_parser import parse_kicad_pcb
        glides = [i for s in _shots(st, 'action') for i in range(*s)]
        named = [i for i in glides if out['log'][i]['moving']]
        _check(glides and named == glides,
               'every glide frame names what moves (%d of %d)'
               % (len(named), len(glides)))
        rest = set()
        for bd in c.boards:
            fps = parse_kicad_pcb(bd).footprints
            for ref in ('C26', 'U1'):
                fp = fps[ref]
                rest.add((ref, round(fp.x, 6), round(fp.y, 6)))
        bad = [(e, ref, tab[ref][:2])
               for e, tab in enumerate(out['epochs'])
               for ref in ('C26', 'U1')
               if (ref, round(tab[ref][0], 6), round(tab[ref][1], 6))
               not in rest]
        _check(not bad, 'every epoch holds a board\'s resting poses %s'
               % (bad[:3],))


def test_no_caption_over_the_board_when_a_rail_carries_it():
    """#1083: 0 over-board stamps on a film -- every film frame has a rail
    (stage3d is the only layout) -- and a frame with NO rail still gets its
    caption, so the stamp was not just deleted. The rail-less control is
    `build_single`, the one path that plans no frame; the flip-caption
    control this used to run was the retired legacy frame's."""
    calls = []
    orig = RR.BoardRenderer._label

    def _spy(self, img, text, *a, **k):
        calls.append(text)
        return orig(self, img, text, *a, **k)
    RR.BoardRenderer._label = _spy
    try:
        with FC.Chain() as c:
            FC.film(c.boards)
            rail = list(calls)
            del calls[:]
            A.build_single({'events': []}, c.boards[-1], 320, 1, None, 2)
            bare = list(calls)
    finally:
        RR.BoardRenderer._label = orig
    _check(rail == [], 'stage3d (rail): no over-board caption (%d stamps: %s)'
           % (len(rail), rail[:3]))
    _check(any('routed' in (t or '') for t in bare),
           'no rail (build_single): the frame is still captioned (%s)'
           % bare[:3])


def test_a_back_side_glide_draws_its_ghost():
    """#1084: the ghost/arrow reaches the renderer on the back as on the
    front, and it changes pixels there (a control with the ghost disabled)."""
    import movie_camera as MC
    import place_motion as PM
    seen = []
    orig_frame = RR.BoardRenderer.frame
    orig_snap = MC.Stage._snap
    last = {}

    def _frame(self, *a, **k):
        last['ov'] = k.get('overlays')
        return orig_frame(self, *a, **k)

    def _snap(self, label):
        last.clear()
        ov = self.movie.overlay is not None
        mirrored = self._mirror
        out = orig_snap(self, label)
        if ov:
            seen.append((mirrored, last.get('ov') is not None))
        return out
    with FC.Chain() as c:
        RR.BoardRenderer.frame, MC.Stage._snap = _frame, _snap
        try:
            frames, _m, st, _g = FC.film(c.boards)
        finally:
            RR.BoardRenderer.frame, MC.Stage._snap = orig_frame, orig_snap
        back = [got for mir, got in seen if mir]
        front = [got for mir, got in seen if not mir]
        _check(front and all(front), 'front glide: ghost passed %d/%d'
               % (sum(front), len(front)))
        _check(back and all(back), 'back glide: ghost passed %d/%d'
               % (sum(back), len(back)))
        og = PM.ghost_overlay
        PM.ghost_overlay = lambda *a, **k: None
        try:
            plain, _m2, _st2, _g2 = FC.film(c.boards)
        finally:
            PM.ghost_overlay = og
        flip_at = _shots(st, 'flip')[0][0]
        a, b = [s for s in _shots(st, 'action') if s[0] >= flip_at][0]
        changed = sum(1 for i in range(a, b)
                      if ImageChops.difference(frames[i].convert('RGB'),
                                               plain[i].convert('RGB'))
                      .getbbox())
        _check(changed > 0, 'the back glide\'s ghost changes %d of %d frames'
               % (changed, b - a))


def test_copper_revealed_after_the_flip_is_mirrored():
    """#1085: the copper reveal after a flip to B is seen from the back. The
    last reveal frame and the first outro frame show one state at one view
    (the settle already brought the camera home), so their board boxes are
    EQUAL -- before the fix the outro frame equalled the reveal's MIRROR.

    Those two checks are RELATIVE: a film that stopped mirroring after the
    flip altogether is self-consistent and passes them (on the stage3d
    frame, measured: `mutate_1081`'s copper-after-the-flip-unmirrored
    survived). So the renderer's own calls are counted too -- every frame
    recorded as seen from the back must have been rendered with
    `mirror=True`."""
    calls = []
    orig = RR.BoardRenderer.frame

    def _frame(self, *a, **k):
        calls.append(bool(k.get('mirror')))
        return orig(self, *a, **k)
    with FC.Chain() as c:
        out = {}
        RR.BoardRenderer.frame = _frame
        try:
            frames, _m, st, g = FC.film(c.boards, stage_out=out)
        finally:
            RR.BoardRenderer.frame = orig
        outro = _shots(st, 'outro')[0][0]
        flip_end = _shots(st, 'flip')[0][1]
        bx = g.board
        box = (bx.x, bx.y + bx.h // 3, bx.x + bx.w, bx.y + bx.h)
        rev = frames[outro - 1].convert('RGB').crop(box)
        first = frames[outro].convert('RGB').crop(box)
        same = ImageChops.difference(rev, first).getbbox()
        mirr = ImageChops.difference(ImageOps.mirror(rev), first).getbbox()
        _check(same is None and mirr is not None,
               'reveal frame %d and outro frame %d agree (diff %s; vs the '
               'mirror %s)' % (outro - 1, outro, same, mirr))
        # ...and the COPPER-STEP frames themselves (kind 'frame', Movie's
        # own path) -- the ones #1085 was about. Frame `outro - 1` is a
        # reconcile SNAPSHOT, which a Stage-side mirror alone would already
        # fix (the phase-1 verifier reverted `_frame`'s mirror and the checks
        # above still passed). A reveal frame differs from the snapshot after
        # it only by its highlight, so it must sit far closer to that
        # snapshot than to its mirror image.
        log = out['log']
        reveal = [i for i in range(flip_end, outro - 1)
                  if log[i]['kind'] == 'frame']
        _check(bool(reveal), 'the fixture reveals copper through Movie\'s '
               'own path after the flip (%d frames)' % len(reveal))
        if reveal:
            k = reveal[-1]
            fk = frames[k].convert('RGB').crop(box)
            snap = frames[outro - 1].convert('RGB').crop(box)

            def _n(a, b):
                d = ImageChops.difference(a, b).convert('L')
                return d.point(lambda v: 255 if v else 0).histogram()[255]
            near, far = _n(fk, snap), _n(ImageOps.mirror(fk), snap)
            _check(near * 4 < far,
                   'copper-step frame %d is seen from the back: %d px off '
                   'the mirrored snapshot, %d px off its mirror image'
                   % (k, near, far))
        n_back = sum(1 for r in out['log'] if r['mirror'])
        _check(n_back and sum(calls) >= n_back,
               'every back-side frame is rendered mirrored (%d mirror=True '
               'renders for %d back-side frames)' % (sum(calls), n_back))
        flags = [r['mirror'] for r in out['log'][flip_end:]]
        _check(flags and all(flags),
               'every frame after the flip is recorded mirrored (%d of %d)'
               % (sum(flags), len(flags)))


def test_the_stage3d_column_is_always_the_layer_strip():
    """#1081's layer column is ONE thing on every frame: the per-layer strip
    (with the board's numbers under it). It used to switch by phase between
    the strip, a placement bar chart and a stats table (the phase-7
    verification), which read as three widgets beside one 3D board."""
    import animate_route as A2
    import render_panels as RP
    calls = {'strip': 0}
    o_strip = RP.draw_layer_strip

    def _strip(*a, **k):
        calls['strip'] += 1
        return o_strip(*a, **k)
    RP.draw_layer_strip = _strip
    try:
        with FC.Chain() as c:
            geom = []
            steps = [('step %d' % i, b, None) for i, b in enumerate(c.boards)]
            import movie_camera as MC
            st = MC.Stage(MC.synth_rounds(c.boards), '', tween=4, quiet=True)
            frames = A2.build_boards(steps, c.boards[-1], 480, 1, None, 2, 6,
                                     stage=st,
                                     geom_out=geom, board3d='2d')
            list(frames)
    finally:
        RP.draw_layer_strip = o_strip
    _check(calls['strip'] == len(frames),
           'stage3d: the layer strip on all %d frames (%s)'
           % (len(frames), calls))
    _check(not any(hasattr(RP, n) for n in ('draw_inventory', 'phase_for')),
           'and nothing else the column could switch to is left')


TESTS = (
    test_one_chrome_record_and_one_stage_record_per_frame,
    test_no_caption_over_the_board_when_a_rail_carries_it,
    test_a_back_side_glide_draws_its_ghost,
    test_copper_revealed_after_the_flip_is_mirrored,
    test_the_stage3d_column_is_always_the_layer_strip,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for msg in _FAIL:
            print('  - %s' % msg)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
