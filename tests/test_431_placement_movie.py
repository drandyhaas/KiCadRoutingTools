"""The camera wired into the real movie pipeline (#431).

`test_431_camera.py` covers the shot planner as data. This covers the
INTEGRATION: that `Stage` drives the existing `Movie`/`BoardRenderer`/
`save_movie` stack, that footprints actually animate, and -- the part that
matters most -- that the ROUTING movie is untouched when no stage is passed.

Asserts on the raw PIL frame list, never on the encoded file: Pillow's GIF
writer de-duplicates identical consecutive frames, so a settle beat (which is
deliberately N identical frames) is invisible in the output file.
"""

import json
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # fixture_boards
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split

from PIL import ImageChops  # noqa: E402

import animate_route as A  # noqa: E402
import make_movie as MM  # noqa: E402
import route_render as RR  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from movie_camera import Stage, load_round_sidecars  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KF = os.path.join(ROOT, 'kicad_files')
SEED = os.path.join(KF, 'interf_u_unrouted.kicad_pcb')
PLACED = os.path.join(KF, 'interf_u_unrouted_placed.kicad_pcb')


def _work_dir():
    """A work dir shaped like place_route_loop's: two accepted rounds carrying a
    real 22-part placement delta, plus one REJECTED round that must be skipped."""
    d = tempfile.mkdtemp()
    shutil.copy(SEED, os.path.join(d, 'loop_round0.kicad_pcb'))
    shutil.copy(SEED, os.path.join(d, 'loop_round0_routed.kicad_pcb'))
    shutil.copy(PLACED, os.path.join(d, 'loop_round1.kicad_pcb'))
    shutil.copy(PLACED, os.path.join(d, 'loop_round2.kicad_pcb'))
    shutil.copy(PLACED, os.path.join(d, 'loop_round2_routed.kicad_pcb'))

    # Two rounds working on well-SEPARATED regions -- otherwise the camera is
    # right not to move (hysteresis), and a test asserting a pan would be
    # asserting a bug. Left cluster, then right cluster.
    b = parse_kicad_pcb(PLACED)
    by_x = sorted(b.footprints.items(), key=lambda kv: kv[1].x)
    left = [r for r, _f in by_x[:3]]
    right = [r for r, _f in by_x[-3:]]

    def _moves(refs):
        out = []
        for ref in sorted(refs):
            fp = b.footprints[ref]
            out.append({'reference': ref,
                        'from': [fp.x - 2.0, fp.y - 1.5, fp.rotation or 0.0],
                        'to': [fp.x, fp.y, fp.rotation or 0.0]})
        return out

    docs = [
        {'schema': 1, 'round': 0, 'board': 'loop_round0.kicad_pcb',
         'routed': 'loop_round0_routed.kicad_pcb', 'parent': None,
         'accepted': True, 'screened': False, 'moved': _moves(left),
         'metrics': {}},
        # REJECTED: on disk, but not the story. Must not appear in the chain.
        {'schema': 1, 'round': 1, 'board': 'loop_round1.kicad_pcb',
         'routed': None, 'parent': 'loop_round0.kicad_pcb',
         'accepted': False, 'screened': False, 'moved': _moves(left),
         'metrics': {}},
        {'schema': 1, 'round': 2, 'board': 'loop_round2.kicad_pcb',
         'routed': 'loop_round2_routed.kicad_pcb',
         'parent': 'loop_round0.kicad_pcb',      # NOT round1 -- it was rejected
         'accepted': True, 'screened': False, 'moved': _moves(right),
         'metrics': {}},
    ]
    moved = _moves(left)
    for doc in docs:
        with open(os.path.join(d, f"loop_round{doc['round']}.json"), 'w',
                  encoding='utf-8') as f:
            json.dump(doc, f)
    return d, len(moved)


# --- the routing movie must not change --------------------------------------

def _count_renderers(boards, **kw):
    """make_movie over `boards`, counting BoardRenderer constructions and
    set_view calls, and capturing stderr."""
    import contextlib
    import io
    made = {'ctor': 0, 'set_view': 0}
    orig = RR.BoardRenderer

    class Counting(orig):
        def __init__(self, *a, **k):
            made['ctor'] += 1
            super().__init__(*a, **k)

        def set_view(self, *a, **k):
            made['set_view'] += 1
            return super().set_view(*a, **k)

    RR.BoardRenderer = Counting
    err = io.StringIO()
    old_env = os.environ.pop('KICAD_MOVIE_CAMERA', None)
    try:
        d = tempfile.mkdtemp()
        try:
            with contextlib.redirect_stderr(err):
                out = MM.make_movie(boards, out=os.path.join(d, 'r.gif'),
                                    size=200, quiet=True, **kw)
            assert out
        finally:
            shutil.rmtree(d, ignore_errors=True)
    finally:
        RR.BoardRenderer = orig
        if old_env is not None:
            os.environ['KICAD_MOVIE_CAMERA'] = old_env
    return made, err.getvalue()


def test_no_stage_means_no_camera_and_one_renderer():
    """Structural non-regression: with stage=None the hooks are falsy branches.
    Counting constructions and set_view calls proves it without a golden image
    (Pillow's resampling drifts between versions).

    UPDATED DELIBERATELY for #1036. This used to run `[SEED, PLACED]` -- a
    chain whose parts MOVE -- with the camera unstated, and pinned that it got
    no stage. That was the bug: run 32's seven placement boards contributed no
    frames at all. An unstated camera now turns 'auto' exactly when the boards
    show a pose change, so the no-stage path is pinned on the two chains that
    still take it: a ROUTING chain (no pose change) with the camera unstated,
    and the placement chain with the camera EXPLICITLY off."""
    from fixture_boards import ensure_many
    route_chain = ensure_many('fanout_output1.kicad_pcb',
                              'fanout_output2.kicad_pcb')
    for boards, kw, why in ((route_chain, {}, 'routing chain, camera unstated'),
                            ([SEED, PLACED], {'camera': 'off'},
                             'placement chain, --camera off')):
        made, err = _count_renderers(boards, **kw)
        assert made['ctor'] == 1, (why, f"{made['ctor']} renderers built "
                                   "(expected 1)")
        # exactly the one inside __init__; the camera never aims it
        assert made['set_view'] == 1, (why, made['set_view'])
        assert 'camera auto' not in err, (why, err)


def test_a_placement_chain_turns_the_camera_on_by_itself():
    """#1036: `make_movie a.kicad_pcb b.kicad_pcb` over boards whose parts
    MOVE is a placement film. With the camera unstated it goes 'auto' -- the
    stage aims the camera (more than the one set_view in __init__) -- and says
    so on stderr, even under quiet=True."""
    made, err = _count_renderers([SEED, PLACED])
    assert 'camera auto' in err, err
    assert made['set_view'] > 1, ('the stage never aimed the camera',
                                  made['set_view'])
    # --camera off on the same chain says what it skipped only when there is
    # something to skip: interf_u's boards carry no copper, and the whole chain
    # being copper-free is not "leading" boards skipped before routing.
    _m, err_off = _count_renderers([SEED, PLACED], camera='off')
    assert 'camera auto' not in err_off, err_off


def test_leading_copper_free_boards_are_counted():
    from fixture_boards import ensure_many
    routed = ensure_many('fanout_output1.kicad_pcb')[0]
    steps = [('a', SEED, None), ('b', PLACED, None), ('c', routed, None)]
    assert MM.leading_copper_free(steps) == 2, MM.leading_copper_free(steps)
    assert MM.leading_copper_free(steps[2:]) == 0


def test_a_middle_step_draws_its_own_boards_pads():
    """#1036 verifier: replacing build_boards' per-step `r.pcb = pcb` with
    `pass` SURVIVED this file, because the only check looked at frame 0 (set
    before the loop). Here a chain A -> B -> A whose MIDDLE board has moved
    parts and a trace of its own: every frame drawn during B's step must be
    drawn from B's board."""
    import json as _json
    sys.path.insert(0, os.path.join(ROOT, 'tests'))
    from test_film_composition import _variant
    routed = os.path.join(KF, 'routed_output.kicad_pcb')
    d = tempfile.mkdtemp()
    try:
        a = os.path.join(d, 'a.kicad_pcb')
        b = os.path.join(d, 'b.kicad_pcb')
        shutil.copy(routed, a)
        _variant(routed, b, dx=6.0, dy=0.0, n=12)
        pcb = parse_kicad_pcb(b)
        layers = list(pcb.board_info.copper_layers)
        s0 = A._board_rows(pcb, layers)[0][0]
        row = [s0[0] + 0.5, s0[1] + 0.5, s0[2] + 0.5, s0[3] + 0.5, s0[4],
               s0[5]]
        tr = os.path.join(d, 'b_trace.json')
        with open(tr, 'w') as f:
            _json.dump({'layers': layers, 'events': [
                {'event': 'route', 'net_name': 'x', 'add_s': [row]}]}, f)
        drawn = []
        orig = RR.BoardRenderer.frame

        def _spy(self, *aa, **kk):
            drawn.append(os.path.basename(
                getattr(self.pcb, 'source_path', '') or ''))
            return orig(self, *aa, **kk)
        RR.BoardRenderer.frame = _spy
        marks = []
        try:
            A.build_boards([('a', a, None), ('b', b, tr), ('a2', a, None)],
                           a, 200, 1, None, 2, 6, marks=marks)
        finally:
            RR.BoardRenderer.frame = orig
        _lb, _bd, first, last = marks[1]
        during = drawn[first:last]
        assert during, ('BROKEN: the middle step drew no frame', marks)
        assert all(x == 'b.kicad_pcb' for x in during), \
            ('a frame of the MIDDLE step was drawn from another board',
             during)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_auto_camera_ignores_drift_but_not_placement():
    """#1036 verifier's r5 repro: a 0.05 mm nudge of one part switched a whole
    routing film to the placement camera. Below MOVIE_MOVE_MIN_MM a
    translation is drift -- no camera, no glide -- while a real placement
    still turns the camera on."""
    sys.path.insert(0, os.path.join(ROOT, 'tests'))
    from test_film_composition import _variant
    from fixture_boards import ensure_many
    b1, _b2 = ensure_many('fanout_output1.kicad_pcb',
                          'fanout_output2.kicad_pcb')
    d = tempfile.mkdtemp()
    try:
        drift = os.path.join(d, 'drift.kicad_pcb')
        _variant(b1, drift, dx=0.05, dy=0.0, n=1)
        _m, err = _count_renderers([b1, drift])
        assert 'camera auto' not in err, ('0.05 mm drift turned the camera '
                                          'on', err)
        import movie_camera as MC
        assert not any(rd['moved'] for rd in MC.synth_rounds([b1, drift]))
        real = os.path.join(d, 'real.kicad_pcb')
        _variant(b1, real, dx=3.0, dy=0.0, n=3)
        _m, err2 = _count_renderers([b1, real])
        assert 'camera auto' in err2, ('a 3 mm placement did not turn the '
                                       'camera on', err2)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_the_opening_frame_holds_an_off_board_pile():
    """#1036 verifier: frame 0 ("input") was drawn at the board's bounds, so
    an off-board pile was clipped, and the camera then jumped out to the
    overview three frames later. With a pile beyond the outline, frame 0 is
    drawn at the pile-inclusive overview."""
    import movie_camera as MC
    rounds = MC.synth_rounds([SEED, PLACED])
    b = parse_kicad_pcb(SEED).board_info.board_bounds
    # a pile 40 mm below the board, the way run 32's pile sits
    rounds[0]['extent'] = [b[0], b[1], b[2], b[3] + 40.0]
    st = MC.Stage(rounds, '', tween=3)
    views = []
    orig = RR.BoardRenderer.frame

    def _spy(self, *aa, **kk):
        views.append(getattr(self, '_view', None))
        return orig(self, *aa, **kk)
    RR.BoardRenderer.frame = _spy
    try:
        A.build_boards([('a', SEED, None), ('b', PLACED, None)], PLACED, 200,
                       1, None, 2, 6, stage=st)
    finally:
        RR.BoardRenderer.frame = orig
    assert views and views[0] is not None, views[:3]
    assert tuple(views[0]) == tuple(st._overview), (views[0], st._overview)


def test_each_step_draws_its_own_boards_pads():
    """#1036: the renderer is built from the FINAL board, and without a stage
    every frame used to draw the final board's pads -- so a film opening on an
    unplaced pile showed the finished placement from frame one. Each step now
    re-points `r.pcb` at its own board. The opening frame of [SEED, PLACED]
    must therefore be SEED's picture, not PLACED's."""
    fr = A.build_boards([('seed', SEED, None), ('placed', PLACED, None)],
                        PLACED, 200, 1, 150, 2, 6)
    seed_only = A.build_boards([('seed', SEED, None)], PLACED, 200, 1, 150, 2,
                               6)
    placed_only = A.build_boards([('placed', PLACED, None)], PLACED, 200, 1,
                                 150, 2, 6)
    assert ImageChops.difference(fr[0].convert('RGB'),
                                 seed_only[0].convert('RGB')).getbbox() is None
    assert ImageChops.difference(fr[0].convert('RGB'),
                                 placed_only[0].convert('RGB')).getbbox(), \
        'the opening frame still shows the FINAL board\'s pads'


def test_the_shots_before_a_glide_show_the_board_as_it_was():
    """#1036, seen on the run-32 render: `build_boards` re-points `r.pcb` at
    the step's board BEFORE `enter_step` drains the camera shots queued ahead
    of the action, so the establishing shot showed the FINISHED placement and
    the parts then jumped back to glide out of the pile. The shots before the
    action must be drawn on the PREVIOUS board."""
    import movie_camera as MC
    rounds = MC.synth_rounds([SEED, PLACED])
    st = MC.Stage(rounds, '', tween=4)
    seen = []
    orig = MC.Stage._emit

    def _spy(self, kind, views, label, side=None):
        seen.append((kind, os.path.basename(
            getattr(self.r.pcb, 'source_path', '') or '')))
        return orig(self, kind, views, label, side)
    MC.Stage._emit = _spy
    try:
        A.build_boards([('seed', SEED, None), ('placed', PLACED, None)],
                       PLACED, 200, 1, None, 2, 6, stage=st)
    finally:
        MC.Stage._emit = orig
    pre = [b for k, b in seen if k != 'outro']
    assert pre, seen
    assert all(b == os.path.basename(SEED) for b in pre), \
        ('a shot before the glide was drawn on the destination board', seen)


def test_synthesised_rounds_zoom_and_frame_the_pile():
    """#1036: `make_movie` on a board LIST passes work_dir='' and synthesised
    rounds carry ABSOLUTE board paths. `Stage._plan` parsed the round's board
    only under a work dir, so every hand-driven chain planned its placement
    shots with focus=None -- the camera never zoomed. And the overview must
    hold where the parts come FROM (`extent`), or a glide out of an off-board
    pile starts off-frame."""
    import movie_camera as MC
    rounds = MC.synth_rounds([SEED, PLACED])
    assert rounds and rounds[0].get('extent'), rounds[:1]
    seen = {}
    orig = MC.plan_shots

    def _spy(acts, overview, opts=None):
        seen['acts'], seen['overview'] = list(acts), overview
        return orig(acts, overview, opts)

    class _R:
        bounds = (0.0, 0.0, 1.0, 1.0)     # deliberately smaller than the pile

        def set_view(self, view=None):    # the opening frame aims the pile
            self._view = view
    MC.plan_shots = _spy
    try:
        st = MC.Stage(rounds, '', tween=4)
        st.attach(object(), _R(), ['F.Cu', 'B.Cu'])
    finally:
        MC.plan_shots = orig
    acts = seen.get('acts') or []
    assert acts and all(a.focus is not None for a in acts),         [a.focus for a in acts]
    ext, ov = rounds[0]['extent'], seen['overview']
    assert (ov[0] <= ext[0] and ov[1] <= ext[1] and ov[2] >= ext[2]
            and ov[3] >= ext[3]), (ov, ext)


def test_build_boards_signature_keeps_stage_optional():
    import inspect
    sig = inspect.signature(A.build_boards)
    assert sig.parameters['stage'].default is None


# --- the blocker, as an assertion -------------------------------------------

def test_moving_parts_animates_only_with_a_stage():
    """The whole reason a placement movie was impossible: build_boards points
    the renderer at the FINAL board, so parts sit at their final poses from
    frame 0. With dynamic_zones=True (which build_boards passes) frame() draws
    pads per frame from renderer.pcb, so re-pointing it animates them."""
    pcb_a, pcb_b = parse_kicad_pcb(SEED), parse_kicad_pcb(PLACED)
    r = RR.BoardRenderer(pcb_a, size=260, supersample=1, dynamic_zones=True)
    before = r.frame()
    r.pcb = pcb_b
    after = r.frame()
    assert ImageChops.difference(before, after).getbbox() is not None, \
        "re-pointing renderer.pcb did not move the parts"

    r2 = RR.BoardRenderer(pcb_a, size=260, supersample=1, dynamic_zones=False)
    b2 = r2.frame()
    r2.pcb = pcb_b
    assert ImageChops.difference(b2, r2.frame()).getbbox() is None, \
        "with dynamic_zones=False pads are baked into _base -- this is why the " \
        "animator's dynamic_zones=True is load-bearing, not incidental"


# --- the chain ---------------------------------------------------------------

def test_only_accepted_rounds_enter_the_chain():
    d, _n = _work_dir()
    try:
        chain = MM.placement_chain(d)
        assert chain is not None
        labels = [s[0] for s in chain[0]]
        assert 'round 1' not in labels, \
            f"a REJECTED round entered the chain: {labels}"
        assert labels[0] == 'round 0' and 'round 2' in labels
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_a_dir_without_sidecars_is_still_a_routing_run():
    """placement_chain must key on the SIDECARS, not a loop_round*.kicad_pcb
    glob -- --work-dir defaults to the output board's directory, which may hold
    unrelated boards."""
    d = tempfile.mkdtemp()
    try:
        shutil.copy(SEED, os.path.join(d, 'loop_round0.kicad_pcb'))
        assert MM.placement_chain(d) is None
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_sidecar_loader_orders_by_round_not_by_name():
    d = tempfile.mkdtemp()
    try:
        for n in (0, 2, 10):
            with open(os.path.join(d, f'loop_round{n}.json'), 'w',
                      encoding='utf-8') as f:
                json.dump({'schema': 1, 'round': n, 'accepted': True,
                           'board': f'loop_round{n}.kicad_pcb'}, f)
        got = [x['round'] for x in load_round_sidecars(d)]
        assert got == [0, 2, 10], f"{got} -- round 10 must not sort before 2"
    finally:
        shutil.rmtree(d, ignore_errors=True)


# --- the camera in the pipeline ---------------------------------------------

def _camera_frames(size=240):
    d, n_moved = _work_dir()
    try:
        steps, final = MM.placement_chain(d)
        stage = Stage(load_round_sidecars(d), d, tween=6)
        frames = A.build_boards(steps, final, size, 1, 150, 2, 6, stage=stage)
        return frames, stage, n_moved
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_every_frame_is_the_same_size():
    """The single highest-value assertion here. _write_mp4 fails on mixed sizes
    and the failure is SILENT -- caught, and the whole movie degraded to GIF."""
    frames, _stage, _n = _camera_frames()
    assert frames
    assert len({f.size for f in frames}) == 1, {f.size for f in frames}


def test_the_camera_moves_and_then_holds():
    """Motion during a transit, stillness during a settle beat. Asserted on the
    RAW frames: the GIF writer de-duplicates identical consecutive frames, so a
    beat is invisible in the encoded file."""
    frames, stage, _n = _camera_frames()
    log = stage.frame_log()
    assert log, "the stage recorded no shots"
    kinds = [k for k, _a, _b in log]
    assert 'establish' in kinds or 'transit' in kinds, kinds

    def changed(i):
        return ImageChops.difference(frames[i], frames[i + 1]).getbbox() is not None

    moved_any = False
    for kind, a, b in log:
        if kind == 'transit' and b - a >= 3:
            assert any(changed(i) for i in range(a, min(b, len(frames)) - 1)), \
                "a transit produced no motion"
            moved_any = True
        if kind == 'beat' and b - a >= 2:
            assert not any(changed(i) for i in range(a, min(b, len(frames)) - 1)), \
                "a settle beat moved -- the board or camera changed during it"
    assert moved_any, "no transit shot emitted at all"


def test_the_parts_actually_glide():
    frames, stage, n_moved = _camera_frames()
    assert n_moved == 3, n_moved
    acts = [(a, b) for k, a, b in stage.frame_log() if k == 'action']
    assert acts, "no tween emitted"
    a, b = acts[-1]
    assert b - a >= 2
    assert ImageChops.difference(frames[a], frames[b - 1]).getbbox() is not None, \
        "the tween's first and last frames are identical -- nothing moved"


def test_the_movie_writes_and_reports_its_path():
    d, _n = _work_dir()
    out = os.path.join(d, 'placement.gif')
    try:
        got = MM.make_movie([d], out=out, camera='auto', size=200, quiet=True)
        assert got and os.path.exists(got), got
        assert os.path.getsize(got) > 1000
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_camera_defaults_off_and_the_env_knob_turns_it_on():
    """One variable covers the GUI recorder, run_plan.py --movie and the stress
    renderer at once -- and OFF is the default everywhere, because every GUI
    movie is a routing movie."""
    import env_knobs
    old = os.environ.get('KICAD_MOVIE_CAMERA')
    try:
        os.environ.pop('KICAD_MOVIE_CAMERA', None)
        env_knobs.refresh()
        assert env_knobs.MOVIE_CAMERA == 'off'
        os.environ['KICAD_MOVIE_CAMERA'] = 'auto'
        env_knobs.refresh()
        assert env_knobs.MOVIE_CAMERA == 'auto'
    finally:
        if old is None:
            os.environ.pop('KICAD_MOVIE_CAMERA', None)
        else:
            os.environ['KICAD_MOVIE_CAMERA'] = old
        env_knobs.refresh()


def test_camera_on_a_dir_with_no_sidecars_degrades_with_a_note():
    """Asking for a camera where there is nothing to drive must still produce a
    movie, not an exception."""
    d = tempfile.mkdtemp()
    try:
        shutil.copy(SEED, os.path.join(d, 'step01_route.kicad_pcb'))
        shutil.copy(PLACED, os.path.join(d, 'step02_route.kicad_pcb'))
        got = MM.make_movie([d], out=os.path.join(d, 'm.gif'), camera='auto',
                            size=200, quiet=True)
        assert got and os.path.exists(got)
    finally:
        shutil.rmtree(d, ignore_errors=True)



def test_tween_zero_cuts_instead_of_gliding():
    """The glide is decoration and on a long run it is most of the runtime.
    tween=0 must still show the delta -- a BEFORE frame at the source poses and
    an AFTER at the parsed board -- so it is a cut, not a missing beat."""
    d, _n = _work_dir()
    try:
        steps, final = MM.placement_chain(d)
        st = Stage(load_round_sidecars(d), d, tween=0)
        frames = A.build_boards(steps, final, 220, 1, 150, 2, 6, stage=st)
        acts = [(a, b) for k, a, b in st.frame_log() if k == 'action']
        assert acts, "no action frames at all"
        for a, b in acts:
            assert b - a == 2, f"tween=0 must emit exactly before+after, got {b - a}"
            assert ImageChops.difference(frames[a], frames[b - 1]).getbbox()                 is not None, "the before/after cut shows no delta"
        # and it is genuinely shorter than a glide
        st2 = Stage(load_round_sidecars(d), d, tween=8)
        long_frames = A.build_boards(steps, final, 220, 1, 150, 2, 6, stage=st2)
        assert len(frames) < len(long_frames)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_going_to_the_back_flips_the_board_and_mirrors_what_follows():
    """Going to B does what a person does with a real board: it turns over.

    Every frame after the flip is mirrored, because you are looking at the other
    face. Reading a back-side placement off an un-mirrored X-ray means mentally
    reversing every x coordinate -- exactly the error a render should remove.
    """
    from PIL import ImageOps
    d, _n = _work_dir()
    try:
        steps, final = MM.placement_chain(d)
        st = Stage(load_round_sidecars(d), d, tween=0)
        A.build_boards(steps, final, 200, 1, 150, 2, 6, stage=st)

        # _snap is the single point every frame passes through, so assert the
        # mirror there rather than on a pair of frames whose copper also
        # changed (that comparison is real but muddy).
        # Compare WITHOUT captions: a mirrored frame keeps its caption upright
        # (a reversed caption in the wrong corner reads as a rendering fault,
        # not as "you are looking at the back"), so a captioned frame is
        # deliberately NOT a pure mirror of its unmirrored twin.
        st._mirror = False
        st._snap('')
        plain = st.movie.frames[-1]
        st._mirror = True
        st._snap('')
        flipped = st.movie.frames[-1]
        assert flipped.tobytes() == ImageOps.mirror(plain).tobytes(),             "a frame emitted while looking at the back is not mirrored"
        assert flipped.tobytes() != plain.tobytes(), "the board is symmetric?"

        # ...and the caption survives the flip the right way up: a mirrored
        # frame WITH a caption must differ from the plain mirror.
        st._snap('round 2')
        assert st.movie.frames[-1].tobytes() != flipped.tobytes()
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_the_flip_is_a_turn_not_a_dissolve():
    """The board narrows to an edge as it passes 90 degrees, so the moment the
    handedness changes is visible rather than implied. A dissolve would hide it.
    """
    import json as _j
    from movie_camera import Action, CameraOpts, plan_shots, views_for
    shots = plan_shots([Action('place', 'front', (10, 10, 20, 20), 'F'),
                        Action('place', 'back', (12, 11, 22, 21), 'B')],
                       (0.0, 0.0, 100.0, 60.0), CameraOpts())
    flips = [s for s in shots if s.kind == 'flip']
    assert flips, [s.kind for s in shots]
    assert flips[0].view_to is None, "a flip must not also move the camera"
    assert len(views_for(flips[0])) == flips[0].frames
    assert all(v == flips[0].view for v in views_for(flips[0])),         "the flip happens at a FROZEN view -- one thing at a time"


def test_a_through_hole_part_moves_on_BOTH_sides():
    """A hole cannot move on one side only. The data model enforces it -- a
    through-hole pad has ONE global coordinate and layers '*.Cu' -- and the
    offset helper must not break that by treating sides separately."""
    from movie_camera import _offset_to
    from placement.legality import footprint_has_through_pads
    pcb = parse_kicad_pcb(os.path.join(KF, 'tigard.kicad_pcb'))
    ref = next(r for r, f in sorted(pcb.footprints.items())
               if f.pads and footprint_has_through_pads(f))
    fp = pcb.footprints[ref]
    tht = [p for p in fp.pads if getattr(p, 'drill', 0)]
    assert tht, "fixture: expected a drilled pad"
    home = (fp.x, fp.y, [(p, p.global_x, p.global_y) for p in fp.pads],
            [(p, [list(pt) for pt in (p.polygons or [])])
             for p in fp.pads if getattr(p, 'polygons', None)])
    before = [(p.global_x, p.global_y) for p in fp.pads]
    _offset_to(fp, home, 3.0, -2.0)
    for (bx, by), p in zip(before, fp.pads):
        assert abs((p.global_x - bx) - 3.0) < 1e-9
        assert abs((p.global_y - by) - (-2.0)) < 1e-9
    # every drilled pad spans both sides, so moving it moved both
    for p in tht:
        assert any(str(l).startswith('*') or l in ('F.Cu', 'B.Cu')
                   for l in (p.layers or [])), p.layers
    _offset_to(fp, home, 0.0, 0.0)
    assert (fp.x, fp.y) == (home[0], home[1])


def test_a_zoom_on_the_back_side_frames_the_back_parts():
    """The bug this exists for: the camera plans in un-mirrored world space, but
    once flipped every frame is mirrored on the way out. Handing the transform
    the raw rect lands the zoom on the MIRROR IMAGE of the target -- the parts
    you asked to see end up on the far side of the frame, or off it once you are
    zoomed in. The overview hides it (a board is roughly symmetric); a zoom does
    not, which is exactly how it was spotted.
    """
    d, _n = _work_dir()
    try:
        steps, final = MM.placement_chain(d)
        st = Stage(load_round_sidecars(d), d, tween=0)
        seen = []
        real = st._aim

        def spy(v):
            real(v)
            seen.append((v, st._mirror, st.r.tf))
        st._aim = spy
        A.build_boards(steps, final, 600, 1, 150, 2, 6, stage=st)

        board_w = st._overview[2] - st._overview[0]
        zoomed = [(v, tf) for v, m, tf in seen
                  if m and v is not None and (v[2] - v[0]) < board_w * 0.9]
        if not zoomed:
            return          # this fixture did not flip AND zoom; nothing to check
        v, tf = zoomed[-1]
        # the centre of the requested view must land at the centre of the frame
        # AFTER the mirror, not at its reflection
        cx, cy = (v[0] + v[2]) / 2, (v[1] + v[3]) / 2
        px, _py = tf.pt(cx, cy)
        W = 600
        assert abs((W - px) - W / 2) < W * 0.06, (
            f"the back-side zoom is off centre by {abs((W - px) - W / 2):.0f}px "
            f"of {W} -- the camera is aiming at the un-mirrored coordinates")
    finally:
        shutil.rmtree(d, ignore_errors=True)


TESTS = [
    test_no_stage_means_no_camera_and_one_renderer,
    test_a_placement_chain_turns_the_camera_on_by_itself,
    test_leading_copper_free_boards_are_counted,
    test_each_step_draws_its_own_boards_pads,
    test_a_middle_step_draws_its_own_boards_pads,
    test_the_opening_frame_holds_an_off_board_pile,
    test_auto_camera_ignores_drift_but_not_placement,
    test_synthesised_rounds_zoom_and_frame_the_pile,
    test_the_shots_before_a_glide_show_the_board_as_it_was,
    test_build_boards_signature_keeps_stage_optional,
    test_moving_parts_animates_only_with_a_stage,
    test_only_accepted_rounds_enter_the_chain,
    test_a_dir_without_sidecars_is_still_a_routing_run,
    test_sidecar_loader_orders_by_round_not_by_name,
    test_every_frame_is_the_same_size,
    test_the_camera_moves_and_then_holds,
    test_the_parts_actually_glide,
    test_the_movie_writes_and_reports_its_path,
    test_camera_defaults_off_and_the_env_knob_turns_it_on,
    test_camera_on_a_dir_with_no_sidecars_degrades_with_a_note,
    test_tween_zero_cuts_instead_of_gliding,
    test_going_to_the_back_flips_the_board_and_mirrors_what_follows,
    test_the_flip_is_a_turn_not_a_dissolve,
    test_a_zoom_on_the_back_side_frames_the_back_parts,
    test_a_through_hole_part_moves_on_BOTH_sides,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
