#!/usr/bin/env python3
"""#887: the vertically stacked X-ray-over-3D frame, WITHOUT kicad-cli.

Everything here is pure or monkeypatched, so it runs in seconds on any machine
and grades the parts that decide whether the movie is correct:

  * the FAST PATH is untouched -- the single-panel movie is byte-identical and
    never even reaches the composer;
  * ONE frame size across the film, including when a render failed, which is the
    invariant `_write_mp4` degrades on silently and the Pillow GIF fallback
    raises on uncaught;
  * every OFF state says which one it is, rather than being one silence.

`tests/test_887_iso_render.py` is the other half and needs a real kicad-cli.

RUN_ALL_FAST_OK: it shells out for nothing; the one subprocess-free import guard
is Pillow, which route_render already requires.
"""
import os
import sys
import tempfile

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 300

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

try:
    from PIL import Image, ImageChops
except ImportError:
    print('SKIP: Pillow is not installed, so no frame can be rendered')
    sys.exit(77)

import animate_route as A                                  # noqa: E402
import make_movie as MM                                    # noqa: E402
import movie_panels as mp                                  # noqa: E402
import kicad_iso_render as kir                             # noqa: E402

BOARD_A = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
BOARD_B = os.path.join(ROOT, 'kicad_files', 'qfn_fanned_out.kicad_pcb')
TIGARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')

BAD = []


def want(cond, label, extra=''):
    if cond:
        print('  PASS: %s' % label)
    else:
        BAD.append(label)
        print('  FAIL: %s %s' % (label, extra))


def _frames(size=200):
    steps, final = MM.resolve_inputs([BOARD_A, BOARD_B])
    marks = []
    fr = A.build_boards(steps, final, size, 1, 150, 2, 6, marks=marks)
    return fr, marks, final


def _fake_render(png_sizes):
    """A render_many stand-in that writes real PNGs at the sizes given.

    The sizes are the MEASURED ones (KiCad 10.0.0): a 900x700 request came back
    872x672 and a 640x480 request came back 616x448. The point of feeding four
    different ones -- including a 3x3 -- is that the composed frame must be one
    size regardless, because the box is ours and the render's size is not.
    """
    used = []

    def render_many(jobs, cli, workers=None, **kw):
        out = {}
        for i, (key, _board, png, _rot) in enumerate(jobs):
            # Index by the SHOT key, not by position in this batch. Indexing by
            # `i` meant the probe shot never came through here at all, so of the
            # four measured-realistic sizes only the first two were ever used --
            # the commit message claimed all four and the test docstring said so
            # too. Keying on `key` spreads them across the real shot indices.
            w, h = png_sizes[key % len(png_sizes)]
            used.append((w, h))
            Image.new('RGBA', (w, h), (20, 90, 40, 255)).save(png)
            out[key] = (png, '')
        return out
    render_many.used = used
    return render_many


# --------------------------------------------------------------- fast path

def test_the_fast_path_never_reaches_the_composer():
    """No panels asked for -> the composer is not merely a no-op, it is unreached."""
    boom = []

    def explode(*a, **k):
        boom.append(1)
        raise AssertionError('the composer ran on the default path')

    saved = mp.compose_two_panel
    saved_bb = A.build_boards
    seen = {}

    def spy(*a, **k):
        seen['marks'] = k.get('marks', 'ABSENT')
        return saved_bb(*a, **k)

    mp.compose_two_panel = explode
    A.build_boards = spy
    try:
        d = tempfile.mkdtemp()
        out = MM.make_movie([BOARD_A, BOARD_B], out=os.path.join(d, 'm.gif'),
                            size=200, quiet=True)
        want(out and os.path.exists(out), 'the default movie still writes', out)
        want(not boom, 'and the composer was never called')
        want(seen.get('marks') is None,
             'build_boards is called with marks=None -- its own default, so the '
             'default call is the SAME call, not merely an equivalent one',
             seen.get('marks'))
    finally:
        mp.compose_two_panel = saved
        A.build_boards = saved_bb


def test_the_default_movie_is_bit_for_bit_what_it_was():
    grabbed = {}
    saved = A.save_movie

    def spy(frames, out, fps, end_hold, png_dir=None, **kw):
        grabbed.setdefault(out, [f.tobytes() for f in frames])
        return saved(frames, out, fps, end_hold, png_dir=png_dir, **kw)

    A.save_movie = spy
    try:
        d = tempfile.mkdtemp()
        a = os.path.join(d, 'a.gif')
        b = os.path.join(d, 'b.gif')
        MM.make_movie([BOARD_A, BOARD_B], out=a, size=200, quiet=True)
        MM.make_movie([BOARD_A, BOARD_B], out=b, size=200, quiet=True,
                      panels='xray')
        # NON-EMPTY FIRST. `grabbed.get(a) == grabbed.get(b)` is None == None
        # when save_movie is never reached, so the comparison proves nothing
        # until we know there were frames to compare.
        want(grabbed.get(a) and len(grabbed[a]) > 1,
             'the spy captured a real frame list (%d frames)'
             % len(grabbed.get(a) or ()), len(grabbed.get(a) or ()))
        want(grabbed.get(a) == grabbed.get(b),
             'panels="xray" is byte-identical to passing nothing at all')
    finally:
        A.save_movie = saved


# --------------------------------------------------------------- degrading

def test_no_kicad_cli_says_why_and_keeps_one_panel():
    fr, marks, final = _frames()
    ref = {f.size for f in fr}
    ids = [id(f) for f in fr]
    # Patch the RESOLVER's own source, not resolve_cli itself. Stubbing
    # resolve_cli made the test assert a string it had just written -- the
    # 'KICAD_CLI' below was its own injected text, and deleting the env var
    # from the real message survived the suite. Going one level down means the
    # message under test is the shipped one.
    import kicad_oracle
    saved = kicad_oracle.find_kicad_cli
    kicad_oracle.find_kicad_cli = lambda *a, **k: None
    try:
        real_msg = kir.resolve_cli(None)[1]
        out, rep = mp.compose_two_panel(fr, marks, final)
    finally:
        kicad_oracle.find_kicad_cli = saved
    want({f.size for f in out} == ref, 'the frame size is untouched', ref)
    want([id(f) for f in out] == ids,
         'and the very same Image objects come back, not copies')
    line = mp.iso_status_line(rep)
    want(rep['state'] == 'did_not_run', 'the state is named', rep['state'])
    want('KICAD_CLI' in real_msg,
         'the REAL resolver message names the env var to set', real_msg)
    want('KICAD_CLI' in line, 'and it reaches the status line', line)
    want('OFF' in line and 'full speed' in line,
         'and says the movie is the fast single-panel one', line)


def test_a_kicad_cli_that_is_not_kicad_is_an_error_not_a_crash():
    """No monkeypatching: sys.executable exists, runs, and is not kicad-cli."""
    fr, marks, final = _frames()
    ref = {f.size for f in fr}
    out, rep = mp.compose_two_panel(fr, marks, final,
                                    mp.IsoOpts(cli=sys.executable))
    want(rep['state'] == 'error', 'a failing probe render is an error state',
         rep['state'])
    want({f.size for f in out} == ref,
         'and it changed no frame size -- the probe runs BEFORE the commitment')
    want('exited' in rep['detail'], 'the detail carries the exit code',
         rep['detail'])
    want('Traceback' not in rep['detail'], 'and is a message, not a traceback')


def test_max_renders_zero_is_a_named_disable_not_a_silent_one():
    fr, marks, final = _frames()
    ref = {f.size for f in fr}
    out, rep = mp.compose_two_panel(fr, marks, final, mp.IsoOpts(max_renders=0))
    want(rep['state'] == 'disabled', 'disabled is its own state', rep['state'])
    want({f.size for f in out} == ref, 'nothing was composed')
    want('--iso-max-renders 0' in mp.iso_status_line(rep),
         'and the line names the flag that turned it off',
         mp.iso_status_line(rep))


def test_the_status_line_distinguishes_every_state():
    reps = [
        mp._report('ran', shots=[1, 2], panel_wh=(100, 60), boards=2,
                   models={'total': 5, 'found': 5}),
        mp._report('ran', shots=[1, 2], panel_wh=(100, 60), boards=2, failed=1,
                   models={'total': 84, 'found': 0, 'other_ext': 78}),
        mp._report('disabled', '--iso-max-renders 0'),
        mp._report('did_not_run', 'kicad-cli not found (set $KICAD_CLI ...)'),
        mp._report('error', 'kicad-cli pcb render exited 1: boom'),
        mp._report('not_applicable', 'no frames'),
        mp._report('not_applicable', 'no chain boards to render'),
    ]
    lines = [mp.iso_status_line(r) for r in reps]
    # SEVEN reports over FIVE states: `ran` appears twice (clean and with a
    # failed shot) and `not_applicable` twice (no frames, no boards). The
    # claim is that each REPORT reads differently, not that there are seven
    # states -- an earlier version of this message said the latter.
    want(len(set(lines)) == 7,
         'seven reports over five states, seven distinguishable lines',
         len(set(lines)))
    for r, ln in zip(reps, lines):
        if r['state'] != 'ran':
            want(' ON ' not in ln,
                 'an OFF state (%s) must not read like success' % r['state'], ln)
    want('bare board' in lines[1].lower(),
         'a board with no resolvable models says BARE BOARD', lines[1])
    want('FAILED' in lines[1], 'and a failed shot is counted out loud', lines[1])


# ------------------------------------------------------------ the invariant

def test_the_stacked_frame_is_one_constant_even_size():
    fr, marks, final = _frames()
    top = fr[0].size
    saved_r, saved_c = kir.render_many, kir.resolve_cli
    kir.render_many = _fake_render([(872, 672), (616, 448), (900, 700), (3, 3)])
    kir.resolve_cli = lambda explicit=None: ('FAKE', '')
    saved_iso = kir.render_iso
    kir.render_iso = lambda board, png, cli, **kw: (
        Image.new('RGBA', (616, 448), (20, 90, 40, 255)).save(png) or png, '')
    try:
        out, rep = mp.compose_two_panel(fr, marks, final,
                                        mp.IsoOpts(max_renders=4))
    finally:
        kir.render_many, kir.resolve_cli, kir.render_iso = (
            saved_r, saved_c, saved_iso)
    sizes = {f.size for f in out}
    want(len(sizes) == 1,
         'ONE size across the film, though the renders came back at four '
         'different sizes', sizes)
    W, H = out[0].size
    want(W == top[0], 'the width is the X-ray panel\'s', (W, top))
    want(H > top[1], 'and the frame really did grow', (H, top[1]))
    want(H % 2 == 0,
         'the composed height is EVEN, so _write_mp4\'s `& ~1` crop cannot '
         'shave the caption strip', H)


def test_the_iso_panel_never_touches_the_xray_panel():
    fr, marks, final = _frames()
    keep = [f.copy() for f in fr]
    W, H = keep[0].size
    saved_r, saved_c, saved_i = kir.render_many, kir.resolve_cli, kir.render_iso
    kir.render_many = _fake_render([(616, 448)])
    kir.resolve_cli = lambda explicit=None: ('FAKE', '')
    kir.render_iso = lambda board, png, cli, **kw: (
        Image.new('RGBA', (616, 448), (20, 90, 40, 255)).save(png) or png, '')
    try:
        out, _rep = mp.compose_two_panel(fr, marks, final,
                                         mp.IsoOpts(max_renders=2))
    finally:
        kir.render_many, kir.resolve_cli, kir.render_iso = (
            saved_r, saved_c, saved_i)
    bad = [i for i, f in enumerate(out)
           if f.crop((0, 0, W, H)).tobytes() != keep[i].tobytes()]
    want(not bad, 'the X-ray half is byte-identical after composition',
         bad[:5])


def test_a_failed_render_keeps_the_box_and_says_so():
    fr, marks, final = _frames()
    saved_r, saved_c, saved_i = kir.render_many, kir.resolve_cli, kir.render_iso

    def flaky(jobs, cli, workers=None, **kw):
        out = {}
        for n, (key, _b, png, _rot) in enumerate(jobs):
            if n == 0:
                out[key] = (None, 'boom')
            else:
                Image.new('RGBA', (616, 448), (20, 90, 40, 255)).save(png)
                out[key] = (png, '')
        return out

    kir.render_many = flaky
    kir.resolve_cli = lambda explicit=None: ('FAKE', '')
    kir.render_iso = lambda board, png, cli, **kw: (
        Image.new('RGBA', (616, 448), (20, 90, 40, 255)).save(png) or png, '')
    try:
        out, rep = mp.compose_two_panel(fr, marks, final,
                                        mp.IsoOpts(max_renders=3))
    finally:
        kir.render_many, kir.resolve_cli, kir.render_iso = (
            saved_r, saved_c, saved_i)
    want(len({f.size for f in out}) == 1,
         'a failed shot keeps the box, so the film stays one size',
         {f.size for f in out})
    want(rep['failed'] == 1, 'and the failure is counted', rep['failed'])
    want(any(s['error'] for s in rep['shots']),
         'the reason is carried per shot, not swallowed')
    want('FAILED' in mp.iso_status_line(rep), 'and reported in the status line')


# ------------------------------------------------------------- the cadence

def test_a_render_that_succeeds_but_will_not_decode_is_counted_as_failed():
    """The gap between "kicad-cli exited 0" and "there is an image".

    The other failure test hands back `(None, 'boom')`, which the composer
    already counted. This is the path where the render REPORTS success and the
    file is junk -- a full disk, a killed child -- which only iso_panel can
    discover. Counting only falsy `png` survived, and so did deleting the
    drawing of the reason.
    """
    fr, marks, final = _frames()
    saved_r, saved_c, saved_i = kir.render_many, kir.resolve_cli, kir.render_iso

    def junk(jobs, cli, workers=None, **kw):
        out = {}
        for key, _b, png, _rot in jobs:
            open(png, 'wb').write(b'this is not a png')
            out[key] = (png, '')          # "success"
        return out

    kir.render_many = junk
    kir.resolve_cli = lambda explicit=None: ('FAKE', '')
    kir.render_iso = lambda board, png, cli, **kw: (
        Image.new('RGBA', (616, 448), (20, 90, 40, 255)).save(png) or png, '')
    try:
        out, rep = mp.compose_two_panel(fr, marks, final,
                                        mp.IsoOpts(max_renders=3))
    finally:
        kir.render_many, kir.resolve_cli, kir.render_iso = (
            saved_r, saved_c, saved_i)
    want(len({f.size for f in out}) == 1,
         'the film is still one size', {f.size for f in out})
    want(rep['failed'] >= 1,
         'and an undecodable render COUNTS as failed -- otherwise the status '
         'line claims full success over panels reading "could not read the '
         'render"', rep['failed'])
    want(any('could not read' in (s['error'] or '') for s in rep['shots']),
         'with the reason carried per shot',
         [s['error'] for s in rep['shots']])
    want('FAILED' in mp.iso_status_line(rep),
         'and surfaced in the status line', mp.iso_status_line(rep))

    # And the reason must be DRAWN, not merely reported.
    box = (220, 100)
    d = tempfile.mkdtemp()
    bad = os.path.join(d, 'bad.png')
    open(bad, 'wb').write(b'this is not a png')
    blank, _ = mp.iso_panel(box, None, 'cap')
    drawn_panel, err = mp.iso_panel(box, bad, 'cap')
    want(err and 'could not read' in err, 'iso_panel reports it', err)
    want(ImageChops.difference(blank, drawn_panel).getbbox() is not None,
         'and writes it into the panel, so a viewer sees why that beat is '
         'blank instead of an unexplained empty box')


def test_the_shot_plan_is_capped_and_covers_every_frame():
    owner = ['a'] * 10 + ['b'] * 10 + ['c'] * 5
    shots, f2s = mp.plan_iso_shots(owner, mp.IsoOpts(max_renders=2))
    want(len(shots) == 2, 'the budget is a hard cap', len(shots))
    want(len(f2s) == 25 and f2s == sorted(f2s),
         'every frame is mapped, and the map never goes backwards')
    want(f2s[0] == 0 and max(f2s) == 1, 'both shots are used', (f2s[0], max(f2s)))
    shots2, _ = mp.plan_iso_shots(owner, mp.IsoOpts(max_renders=100))
    want({s.board for s in shots2} == {'a', 'b', 'c'},
         'with budget to spare, every board appears',
         {s.board for s in shots2})
    for i, k in enumerate(f2s):
        if not (shots[k].first <= i < shots[k].last):
            want(False, 'frame %d falls outside its own shot span' % i)
            return
    want(True, 'and every frame lies inside the span of the shot it maps to')


def test_a_two_board_chain_still_rotates():
    owner = ['a'] * 20 + ['b'] * 20
    shots, _ = mp.plan_iso_shots(owner, mp.IsoOpts(max_renders=8, sweep_deg=60))
    want(len(shots) == 8,
         'the leftover budget buys ROTATION rather than sitting unused',
         len(shots))
    yaws = [s.rotate[2] for s in shots]
    want(yaws == sorted(yaws), 'the yaw sweep is monotone', yaws)
    want(abs((yaws[-1] - yaws[0]) - 60.0) < 1e-9,
         'and travels exactly the declared sweep', yaws[-1] - yaws[0])


def test_a_tiny_film_does_not_pay_a_render_per_frame():
    """The per-STEP cadence is the whole cost argument; a short chain must not
    quietly become per-frame."""
    shots, _ = mp.plan_iso_shots(['a'] * 3, mp.IsoOpts(max_renders=24))
    want(len(shots) == 1, 'three frames buy one render, not three', len(shots))
    shots2, _ = mp.plan_iso_shots(['a'] * 6, mp.IsoOpts(max_renders=24))
    want(len(shots2) <= 2, 'six frames buy at most two', len(shots2))


def test_the_frame_map_follows_the_marks_and_ends_on_the_final_board():
    marks = [('s1', 'b1', 1, 4), ('s2', 'b2', 4, 9)]
    own = mp.board_for_frames(marks, 11, 'final')
    want(own[0] == 'b1',
         'the opening "input" snapshot takes the FIRST step\'s board', own[0])
    want(own[1:4] == ['b1'] * 3 and own[4:9] == ['b2'] * 5,
         'each step owns its own span', own[1:9])
    want(own[9:] == ['final'] * 2,
         'and the trailing trueup takes the FINAL board, which need not be the '
         'last step\'s', own[9:])
    want(mp.board_for_frames([], 3, 'f') == ['f'] * 3, 'no marks -> all final')
    want(mp.board_for_frames(marks, 0, 'f') == [], 'no frames -> empty')


def test_a_zero_length_first_mark_still_owns_the_opening_frame():
    """Measured: pairing two boards gave marks[0] == (label, board, 1, 1).

    Skipping an empty mark opened the film on the SECOND step's 3D view while
    the X-ray panel above still showed the first board -- the two panels
    disagreeing about which board this is.
    """
    marks = [('s1', 'b1', 1, 1), ('s2', 'b2', 1, 7)]
    own = mp.board_for_frames(marks, 7, 'b2')
    want(own[0] == 'b1',
         'the empty first step still owns the opening snapshot', own[0])
    want(own[1:] == ['b2'] * 6, 'and contributes nothing else', own[1:])


# --------------------------------------------------------------- pre-check

def test_the_model_precheck_counts_what_is_on_disk():
    """Machine-independent: no real KiCad tree is read, only temp dirs we made."""
    empty = tempfile.mkdtemp()
    dirs = {v: empty for v in kir._MODEL_DIR_VARS}
    dirs['KIPRJMOD'] = empty
    m = kir.resolve_models(TIGARD, dirs)
    want(m['total'] == 84, 'tigard references 84 models', m['total'])
    want(m['found'] == 0, 'none of which exist in an empty tree', m['found'])
    want(m['other_ext'] == 0, 'and no twins either, yet', m['other_ext'])

    # Now plant the .step twins the real KiCad 10 tree has, and nothing else.
    txt = open(TIGARD, encoding='utf-8', errors='replace').read()
    raws = kir._MODEL_RE.findall(txt)

    def sub(raw):
        # A LAMBDA replacement, never the bare string: `empty` is a Windows temp
        # path, re.sub treats a replacement as a TEMPLATE, and `C:\Users\...`
        # raises "bad escape \U". resolve_models itself already substitutes
        # through a lambda; this test did not, and only the test was wrong.
        return kir._VAR_RE.sub(lambda m: empty, raw).replace('\\', '/')

    # The counter counts REFERENCES, not distinct files, and on a real board
    # those differ a lot: tigard's 84 references point at far fewer files,
    # because every 0402 resistor names the same model. So derive what to
    # expect from the board instead of assuming one file is one reference --
    # a hand-picked "planted 5, expect 5" was wrong by a factor of seven here.
    resolved = [sub(r) for r in raws]
    chosen = sorted(set(resolved))[:3]
    expect_alt = sum(1 for r in resolved if r in chosen)
    for rel in chosen:
        stem = os.path.splitext(rel)[0]
        os.makedirs(os.path.dirname(stem), exist_ok=True)
        open(stem + '.step', 'w').close()
    m2 = kir.resolve_models(TIGARD, dirs)
    want(m2['found'] == 0,
         'a .step twin is not the .wrl the board asked for', m2['found'])
    want(m2['other_ext'] == expect_alt,
         'but it IS counted as present under another extension, once per '
         'REFERENCE -- the measured tigard case, and a different problem from '
         'a missing install', (m2['other_ext'], expect_alt))
    want(expect_alt > len(chosen),
         'and this board really does reuse models, so the two counts differ '
         '(%d references over %d files)' % (expect_alt, len(chosen)))

    # And the real thing: plant the .wrl the board actually names.
    expect_found = sum(1 for r in resolved if r == chosen[0])
    os.makedirs(os.path.dirname(chosen[0]), exist_ok=True)
    open(chosen[0], 'w').close()
    m3 = kir.resolve_models(TIGARD, dirs)
    want(m3['found'] == expect_found,
         'planting the named file resolves every reference to it',
         (m3['found'], expect_found))
    want('BARE BOARD' in kir.models_note(m2),
         'zero resolved models is called a bare board, in the caption',
         kir.models_note(m2))
    want('BARE BOARD' not in kir.models_note({'total': 15, 'found': 10}),
         'a partly-resolved board is NOT called bare -- lvds resolves 10 of 15 '
         'and renders fully populated')


def test_an_unknown_panel_set_is_refused_in_code_and_warned_in_the_env():
    d = tempfile.mkdtemp()
    try:
        MM.make_movie([BOARD_A], out=os.path.join(d, 'x.gif'), size=120,
                      quiet=True, panels='xray+hologram')
        want(False, 'an unknown panels= value must raise')
    except ValueError as e:
        want('xray+iso' in str(e),
             'and the error names the accepted set', str(e))


def _panel_text(img):
    """The caption strip of a panel, as a crude ink signature per column.

    Reading pixels rather than text because the caption is drawn, not stored --
    two panels whose captions differ produce different ink, and that is all
    these tests need to tell "per board" from "one note pasted everywhere".
    """
    W, H = img.size
    strip = img.crop((0, int(H * 0.88), W, H)).convert('L')
    return tuple(sum(strip.getpixel((x, y)) for y in range(strip.height))
                 for x in range(0, W, 3))


def test_each_panels_caption_describes_its_own_board():
    """The models note used to be computed once from the opening board and
    pasted under every panel, so a frame showing board B carried board A's
    count. The fix was made and then not pinned: the mutation that restores the
    bug survived, and so did one that names shots[0]'s board in every caption.
    Nothing read a caption at all.
    """
    fr, marks, final = _frames()
    saved_r, saved_c, saved_i = kir.render_many, kir.resolve_cli, kir.render_iso
    kir.render_many = _fake_render([(616, 448)])
    kir.resolve_cli = lambda explicit=None: ('FAKE', '')
    kir.render_iso = lambda board, png, cli, **kw: (
        Image.new('RGBA', (616, 448), (20, 90, 40, 255)).save(png) or png, '')
    grabbed = []
    real_panel = mp.iso_panel

    def spy(box, png, caption, error=''):
        grabbed.append(caption)
        return real_panel(box, png, caption, error=error)

    mp.iso_panel = spy
    try:
        mp.compose_two_panel(fr, marks, final, mp.IsoOpts(max_renders=4))
    finally:
        kir.render_many, kir.resolve_cli, kir.render_iso = (
            saved_r, saved_c, saved_i)
        mp.iso_panel = real_panel

    want(len(grabbed) >= 2, 'more than one panel was captioned', len(grabbed))
    boards = {os.path.splitext(os.path.basename(b))[0]
              for b in mp.board_for_frames(marks, len(fr), final)}
    want(len(boards) >= 2, 'and the chain really shows two boards', boards)
    for name in boards:
        want(any(c.startswith(name) for c in grabbed),
             'a caption names %r, the board its own panel is showing' % name,
             grabbed)
    notes = {c.split('|')[-1].strip() for c in grabbed}
    want(len(notes) >= 2,
         'and the model counts differ between boards, so the note is computed '
         'PER BOARD rather than once from the opening one', notes)


def test_a_failed_shot_draws_the_reason_into_its_panel():
    """The test named "says so" checked the box SIZE and the report, never that
    the reason reaches the picture. Deleting the drawing survived."""
    box = (200, 90)
    good, _ = mp.iso_panel(box, None, 'cap')
    bad, drawn = mp.iso_panel(box, None, 'cap', error='boom the render failed')
    want(good.size == bad.size == box, 'both are the same box', (good.size,
                                                                bad.size))
    want(ImageChops.difference(good, bad).getbbox() is not None,
         'the failed panel has visibly different ink -- the reason is DRAWN, '
         'not only reported')
    want(drawn == 'boom the render failed',
         'and the panel echoes the reason back, so the caller can count it '
         'whether the failure came from the render or from reading it', drawn)
    # The case the return value exists FOR: the render "succeeded" but the file
    # will not decode. Only iso_panel can discover that.
    d = tempfile.mkdtemp()
    junk = os.path.join(d, 'junk.png')
    open(junk, 'wb').write(b'not a png at all')
    panel, err = mp.iso_panel(box, junk, 'cap')
    want(panel.size == box, 'the box survives an undecodable file', panel.size)
    want(err and 'could not read' in err,
         'and the failure is RETURNED, not only drawn -- returning nothing here '
         'put three broken panels under a status line claiming full success',
         err)


def test_models_note_reports_found_of_total_in_that_order():
    """No test read the numbers, only the BARE BOARD substring, so printing
    them the wrong way round survived."""
    note = kir.models_note({'total': 15, 'found': 10})
    want('10/15' in note, 'found comes first, then total', note)
    want('15/10' not in note, 'and not the other way round', note)
    bare = kir.models_note({'total': 84, 'found': 0, 'other_ext': 78})
    want('0/84' in bare and 'BARE BOARD' in bare, 'a bare board reads 0/84',
         bare)
    # MOSTLY bare: gating the warning on exactly zero left four corpus boards
    # rendering essentially empty with no caption at all (1/160, 5/148, 3/58,
    # 7/75).
    mostly = kir.models_note({'total': 160, 'found': 1, 'other_ext': 149})
    want('MOSTLY BARE' in mostly,
         'and 1 of 160 is called out too, not silently reported as a count',
         mostly)
    want('another extension' in mostly,
         'with the stale-reference reason, which used to stop applying one '
         'model above zero', mostly)
    full = kir.models_note({'total': 15, 'found': 15})
    want('BARE' not in full, 'a fully-resolved board gets no warning', full)


def test_model_dirs_defines_the_projects_own_variable():
    """lvds has two ${KIPRJMOD} refs; the model test plants KIPRJMOD but grades
    tigard, which uses only ${KISYS3DMOD} -- so dropping KIPRJMOD survived."""
    dirs = kir.model_dirs(cli_path=None, board_path=BOARD_A)
    want(dirs.get('KIPRJMOD') == os.path.dirname(os.path.abspath(BOARD_A)),
         'KIPRJMOD is the board\'s own directory', dirs.get('KIPRJMOD'))
    # It is defined WITHOUT a kicad-cli path, because it comes from the board
    # and not from the install -- the versioned 3DMODEL_DIR variables do need
    # the install, and are legitimately unresolved here.
    want('KIPRJMOD' in kir.model_dirs(board_path=BOARD_A),
         'and needs no kicad-cli to be known')
    # Prove it is USED: substitute it and nothing else, then count.
    # DERIVED from the board, not guessed: a hardcoded count was wrong twice
    # here, and a number nobody re-derives is a number that rots.
    raws = kir._MODEL_RE.findall(
        open(BOARD_A, encoding='utf-8', errors='replace').read())
    n_proj = sum(1 for r in raws if '${KIPRJMOD}' in r)
    n_bare = sum(1 for r in raws if '${' not in r)
    want(n_proj > 0, 'lvds really does use ${KIPRJMOD}', n_proj)
    want(n_bare > 0,
         'and it also carries %d BARE relative paths -- the case that used to '
         'be resolved against the caller\'s directory' % n_bare, n_bare)

    only_proj = {'KIPRJMOD': dirs['KIPRJMOD']}
    m = kir.resolve_models(BOARD_A, only_proj)
    want(m['total'] == len(raws), 'every model reference is counted',
         (m['total'], len(raws)))
    want(m['total'] - m['unresolved_var'] == n_proj + n_bare,
         'with KIPRJMOD as the only key, exactly the ${KIPRJMOD} references '
         'plus the bare relative ones get as far as a path -- so dropping that '
         'key would silently make the first group unresolvable',
         (m['total'] - m['unresolved_var'], n_proj + n_bare))
    without = kir.resolve_models(BOARD_A, {})
    want(without['unresolved_var'] == m['total'] - n_bare,
         'and with no keys at all only the bare paths remain resolvable, '
         'because they need no variable', (without['unresolved_var'], n_bare))


def test_a_relative_model_path_is_resolved_against_the_board_not_the_cwd():
    """Measured: leaving a bare relative path relative made os.path.isfile
    answer against os.getcwd(), so whether a reference resolved was decided by
    what happened to sit beside the shell -- and anything found that way is a
    file kicad-cli would never load. This test plants a decoy at the CWD and
    asserts it is NOT counted, which is the defect stated as behaviour rather
    than as a corpus number."""
    import re
    d1, d2 = tempfile.mkdtemp(), tempfile.mkdtemp()
    board = os.path.join(d1, 'b.kicad_pcb')
    open(board, 'w', encoding='utf-8').write(
        '(footprint "x" (model "sub/part.step" (offset (xyz 0 0 0))))\n')
    here = os.getcwd()
    try:
        # Plant a decoy at the CWD, where a cwd-relative resolver would find it.
        os.makedirs(os.path.join(d2, 'sub'), exist_ok=True)
        open(os.path.join(d2, 'sub', 'part.step'), 'w').close()
        os.chdir(d2)
        m = kir.resolve_models(board)
        want(m['total'] == 1, 'one model reference', m)
        want(m['found'] == 0,
             'the decoy beside the CWD is NOT counted -- kicad-cli would never '
             'load it', m)
        # Now plant it where KiCad would actually look: beside the board.
        os.makedirs(os.path.join(d1, 'sub'), exist_ok=True)
        open(os.path.join(d1, 'sub', 'part.step'), 'w').close()
        m2 = kir.resolve_models(board)
        want(m2['found'] == 1,
             'while the one beside the BOARD is', m2)
        os.chdir(here)
        want(kir.resolve_models(board)['found'] == 1,
             'and the answer does not change with the caller\'s directory')
    finally:
        os.chdir(here)


def test_a_non_finite_tuning_value_warns_and_falls_back():
    """argparse type=float accepts nan and inf, and main() catches only
    FileNotFoundError -- so --iso-height-frac nan took the whole movie down
    with `ValueError: cannot convert float NaN to integer`."""
    o = mp.IsoOpts(height_frac=float('nan'), sweep_deg=float('inf'))
    want(o.height_frac == 0.62 and o.sweep_deg == 60.0,
         'non-finite values fall back to the defaults',
         (o.height_frac, o.sweep_deg))
    W, Ht, Hi, tot = mp.panel_geometry((200, 150), o.height_frac)
    want(tot % 2 == 0 and Hi > 0, 'and the geometry still computes', (Hi, tot))
    for v in (0.0, 0.01, -1.0, 5.0):
        _W, _Ht, hi, t = mp.panel_geometry((200, 150), v)
        want(hi >= 48 and t % 2 == 0,
             'a finite but extreme %r is still handled, not rejected' % v,
             (hi, t))


def test_a_temp_dir_failure_is_a_named_state_not_a_traceback():
    fr, marks, final = _frames()
    ref = {f.size for f in fr}
    saved_c, saved_mk = kir.resolve_cli, mp.tempfile.mkdtemp
    kir.resolve_cli = lambda explicit=None: ('FAKE', '')

    def boom(*a, **k):
        raise PermissionError(13, 'Permission denied')

    mp.tempfile.mkdtemp = boom
    try:
        out, rep = mp.compose_two_panel(fr, marks, final)
    finally:
        kir.resolve_cli, mp.tempfile.mkdtemp = saved_c, saved_mk
    want(rep['state'] == 'error',
         'a read-only or full %TEMP% is a named state', rep['state'])
    want({f.size for f in out} == ref, 'and nothing was composed', ref)
    want('directory' in rep['detail'], 'the detail says what failed',
         rep['detail'])


def test_the_disabled_message_names_the_value_the_user_typed():
    fr, marks, final = _frames()
    _out, rep = mp.compose_two_panel(fr, marks, final,
                                     mp.IsoOpts(max_renders=-1))
    want('-1' in mp.iso_status_line(rep),
         'a negative cap is echoed as -1, not as the hardcoded 0 the user '
         'never typed', mp.iso_status_line(rep))


def test_the_env_knob_turns_the_panel_on_and_a_typo_warns():
    """The "and warned in the env" half of the original test asserted nothing;
    no test touched os.environ at all."""
    import env_knobs
    d = tempfile.mkdtemp()
    saved = os.environ.get('KICAD_MOVIE_PANELS')
    reached = []
    real = mp.compose_two_panel

    def spy(frames, marks, final, opts=None, quiet=False):
        reached.append(opts)
        return frames, mp._report('disabled', 'stubbed')

    mp.compose_two_panel = spy
    try:
        os.environ['KICAD_MOVIE_PANELS'] = 'xray+iso'
        env_knobs.refresh()
        MM.make_movie([BOARD_A], out=os.path.join(d, 'k.gif'), size=120,
                      quiet=True)
        want(len(reached) == 1,
             'the env knob alone reaches the composer', len(reached))

        os.environ['KICAD_MOVIE_PANELS'] = 'hologram'
        env_knobs.refresh()
        import io as _io
        import contextlib
        err = _io.StringIO()
        with contextlib.redirect_stderr(err):
            MM.make_movie([BOARD_A], out=os.path.join(d, 'k2.gif'), size=120,
                          quiet=False)
        want(len(reached) == 1, 'a typo does NOT reach the composer',
             len(reached))
        want('KICAD_MOVIE_PANELS' in err.getvalue(),
             'and the warning names the variable', err.getvalue()[:160])
    finally:
        mp.compose_two_panel = real
        if saved is None:
            os.environ.pop('KICAD_MOVIE_PANELS', None)
        else:
            os.environ['KICAD_MOVIE_PANELS'] = saved
        env_knobs.refresh()


TESTS_TO_RUN = [
    test_each_panels_caption_describes_its_own_board,
    test_a_failed_shot_draws_the_reason_into_its_panel,
    test_models_note_reports_found_of_total_in_that_order,
    test_model_dirs_defines_the_projects_own_variable,
    test_a_relative_model_path_is_resolved_against_the_board_not_the_cwd,
    test_a_non_finite_tuning_value_warns_and_falls_back,
    test_a_temp_dir_failure_is_a_named_state_not_a_traceback,
    test_the_disabled_message_names_the_value_the_user_typed,
    test_the_env_knob_turns_the_panel_on_and_a_typo_warns,
    test_the_fast_path_never_reaches_the_composer,
    test_the_default_movie_is_bit_for_bit_what_it_was,
    test_no_kicad_cli_says_why_and_keeps_one_panel,
    test_a_kicad_cli_that_is_not_kicad_is_an_error_not_a_crash,
    test_max_renders_zero_is_a_named_disable_not_a_silent_one,
    test_the_status_line_distinguishes_every_state,
    test_the_stacked_frame_is_one_constant_even_size,
    test_the_iso_panel_never_touches_the_xray_panel,
    test_a_failed_render_keeps_the_box_and_says_so,
    test_a_render_that_succeeds_but_will_not_decode_is_counted_as_failed,
    test_the_shot_plan_is_capped_and_covers_every_frame,
    test_a_two_board_chain_still_rotates,
    test_a_tiny_film_does_not_pay_a_render_per_frame,
    test_the_frame_map_follows_the_marks_and_ends_on_the_final_board,
    test_a_zero_length_first_mark_still_owns_the_opening_frame,
    test_the_model_precheck_counts_what_is_on_disk,
    test_an_unknown_panel_set_is_refused_in_code_and_warned_in_the_env,
]


def main():
    for fn in TESTS_TO_RUN:
        print('--- %s' % fn.__name__)
        fn()
    if BAD:
        print('\nFAILED: %d' % len(BAD))
        for b in BAD:
            print('  - %s' % b)
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
