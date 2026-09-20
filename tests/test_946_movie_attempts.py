#!/usr/bin/env python3
"""The attempts band: three producers, one record type, one axis (#946, #1021).

`place_route_loop` writes every round it tried -- kept and dropped -- and
`make_movie.placement_chain` then skips every non-accepted one. Two docstrings
in `movie_camera` say the opposite about the same files ("`make_film` wants the
dropped ones precisely because they are the search"), and `write_round_sidecar`
carries a `parent` field that exists ONLY because a rejected round sits between
two accepted ones in both name and mtime order. The search is on disk in full.
Nothing drew it.

What this file pins, and every claim here is one a plausible-looking band could
otherwise make falsely:

  * **the axis is the run's own accept rule, chosen ONCE.** `better()` ranks
    "failures first, then iterations" and annotates `vias` report-only, so a
    `vias` axis would draw a staircase pointing one way beside accept/reject
    rings pointing the other. `--accept-cmd` switches the whole axis, and the
    label says which;
  * **a screened round keeps its node and is not scored zero.** Its sidecar has
    `metrics: {}` on purpose -- a consumer "cannot tell 'screened' from
    'crashed'" -- and `None` plotted at the axis floor reads as a perfect
    board;
  * **a converge row with `score: null` is not dropped.** `_score_key`:
    "`blocking == None` is NOT zero -- it means a component that was asked for
    could not answer". Dropping such rows is a MEASURED bug: the placement half
    once read *plateau* from two accepted laps while five accepted laps were
    invisible for having a null score;
  * **nothing is synthesised.** `movie_camera.synth_rounds` forbids exactly
    this extension in its own docstring;
  * **the degradation arm returns the SAME list of the SAME images**, not a
    copy that happens to look the same -- `compose_two_panel`'s contract, and
    its rule that no OFF state may read like success;
  * **the shared record rule did not change `awx`'s line.** `best_so_far` is
    now one function with two policy flags; the control is an independent
    re-implementation of `Ribbon`'s original loop, compared row for row;
  * **`make_film` attaches the band BEFORE its badge loop**, asserted by pixel:
    `_badge` borders the frame it is given, so an after-attach band would leave
    the border around the board only.
"""
import json
import os
import sys
import tempfile

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

import frame_layout as FL          # noqa: E402
import movie_attempts as MA        # noqa: E402
import render_theme as RT          # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

#: round, accepted, screened, failures  (None = no metrics at all)
LOOP_ROWS = [(0, True, False, 14), (1, False, False, 17), (2, True, False, 11),
             (3, False, True, None), (4, True, False, 6), (5, False, False, 9),
             (6, True, False, 2), (7, False, False, 4), (8, True, False, 0)]

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _loop_dir(d, accept_cmd=False):
    last = 'input.kicad_pcb'
    for rnd, acc, scr, fails in LOOP_ROWS:
        met = {}
        if fails is not None:
            met = {'failures': fails, 'iterations': 9000 - rnd,
                   'vias': 400 - rnd}
            if accept_cmd:
                # DELIBERATELY anti-correlated with `failures`: if the axis
                # silently fell back, the staircase would be the other one and
                # this fixture is the only thing that could tell.
                met['accept_score'] = float(fails * -1 + 100)
        doc = {'schema': 1, 'round': rnd,
               'board': None if scr else 'loop_round%d.kicad_pcb' % rnd,
               'routed': None if scr else 'loop_round%d_routed.kicad_pcb' % rnd,
               'parent': last, 'accepted': acc, 'screened': scr,
               'targets': [], 'groups': {}, 'moved': [], 'metrics': met}
        with open(os.path.join(d, 'loop_round%d.json' % rnd), 'w',
                  encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
        if acc:
            last = 'loop_round%d_routed.kicad_pcb' % rnd
    return d


def _ledger(path):
    rows = [(0, 'completion', None, 'a' * 8, True, {'blocking': 9}),
            (1, 'placement', 'a' * 8, 'b' * 8, False, {'blocking': 12}),
            # blocking: null -- a component that could not answer
            (2, 'completion', 'a' * 8, 'c' * 8, True, {'blocking': None}),
            (3, 'systemic', 'c' * 8, 'd' * 8, True, {'blocking': 4}),
            (4, 'completion', 'd' * 8, 'e' * 8, False, {'blocking': 7})]
    with open(path, 'w', encoding='utf-8') as f:
        for it, kind, par, res, acc, score in rows:
            f.write(json.dumps({'iteration': it, 'kind': kind,
                                'parent_sha': par, 'result_sha': res,
                                'lever': '%s lever' % kind, 'accepted': acc,
                                'score': score}) + '\n')
    return path


def _evolve(path):
    doc = {'tag': 'ev', 'K': 51,
           'pop0': [{'name': 's0', 'origin': 'seed x/r0/c0',
                     'grade': [[], 0, 98]},
                    {'name': 's1', 'origin': 'seed x/r0/c1',
                     'grade': [['/A'], 0, 91]}],
           'gens': [{'gen': 1,
                     'new': [{'name': 'g1a', 'origin': 'descend<s0>',
                              'grade': [[], 0, 90]},
                             {'name': 'g1b', 'origin': 'jump<s1; bans [1]>',
                              'grade': [['/B'], 0, 80]}],
                     'pop': [{'name': 'g1a', 'origin': 'descend<s0>',
                              'grade': [[], 0, 90]}]},
                    {'gen': 2,
                     'new': [{'name': 'g2a', 'origin': 'cross<g1a x s0; 4 from B>',
                              'grade': [[], 0, 83]}],
                     'pop': [{'name': 'g2a',
                              'origin': 'cross<g1a x s0; 4 from B>',
                              'grade': [[], 0, 83]}]}]}
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    return path


# --------------------------------------------------------------------------
def test_three_producers_one_record_type():
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t1 = MA.attempts_from_loop_dir(_loop_dir(td))
        t2 = MA.attempts_from_converge_ledger(_ledger(
            os.path.join(td, 'ledger.jsonl')))
        t3 = MA.attempts_from_evolve_ledger(_evolve(
            os.path.join(td, 'evolve_k51.json')))
        for name, t, n in (('loop', t1, len(LOOP_ROWS)), ('converge', t2, 5),
                           ('evolve', t3, 5)):
            if t is None:
                fail('%s: adapter returned None on its own fixture' % name)
                continue
            if len(t.attempts) != n:
                fail('%s: %d attempts, expected %d'
                     % (name, len(t.attempts), n))
            bad = [a for a in t.attempts if not isinstance(a, MA.Attempt)]
            if bad:
                fail('%s: %d row(s) are not an Attempt' % (name, len(bad)))
            print('    %-9s %-28s %s' % (name, t.metric, t.note))
        # the three metrics are DIFFERENT, and each is its own run's rule
        metrics = {t.metric for t in (t1, t2, t3) if t}
        if len(metrics) != 3:
            fail('two producers share an axis label: %s' % sorted(metrics))
    if len(_FAIL) == _mark:
        print('  PASS: one record type, three producers, three honest axes')


def test_the_axis_is_the_accept_rule_and_is_never_mixed():
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        plain = MA.attempts_from_loop_dir(_loop_dir(td))
    with tempfile.TemporaryDirectory() as td2:
        acc = MA.attempts_from_loop_dir(_loop_dir(td2, accept_cmd=True))
    if 'failures' not in plain.metric:
        fail('a plain loop did not rank on failures: %r' % plain.metric)
    if 'accept score' not in acc.metric:
        fail('--accept-cmd did not switch the axis: %r' % acc.metric)
    # ONE metric over the WHOLE list. The fixture's accept_score is
    # anti-correlated with failures, so a per-row fallback shows up as a
    # different staircase rather than as a tie.
    want = [None if f is None else float(-f + 100)
            for _r, _a, _s, f in LOOP_ROWS]
    got = [a.score for a in acc.attempts]
    if got != want:
        fail('the accept-score axis is mixed: %s vs %s' % (got, want))
    # and the admissibility gate follows the metric, not the producer
    if plain.gate_record or not acc.gate_record:
        fail('gate_record did not follow the metric (plain=%s, accept=%s)'
             % (plain.gate_record, acc.gate_record))
    if len(_FAIL) == _mark:
        print('    plain=%r  accept-cmd=%r' % (plain.metric, acc.metric))
        print('  PASS: one axis per film, named, following the accept rule')


def test_an_ungraded_attempt_is_not_a_zero():
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t = MA.attempts_from_loop_dir(_loop_dir(td))
        scr = [a for a in t.attempts if a.screened]
        if len(scr) != 1:
            fail('the screened round did not survive: %d of %d'
                 % (len(scr), len(t.attempts)))
        elif scr[0].score is not None:
            fail('a screened round was scored %r rather than None'
                 % (scr[0].score,))
        elif scr[0].admissible:
            fail('an ungraded round was called admissible')
        lg = MA.attempts_from_converge_ledger(
            _ledger(os.path.join(td, 'l.jsonl')))
        nulls = [a for a in lg.attempts if a.score is None]
        if len(nulls) != 1:
            fail('the null-scored converge row was dropped (%d rows of 5 kept '
                 'a None score)' % len(nulls))
        if any(a.score == 0 for a in lg.attempts):
            fail('a null blocking was coerced to 0 -- _score_key: "blocking '
                 '== None is NOT zero"')
        # it is still COUNTED, which is the half a drop would hide
        if 'ungraded' not in t.note:
            fail('the note does not disclose the ungraded attempt: %r' % t.note)
    if len(_FAIL) == _mark:
        print('    loop: 1 screened kept, score None; converge: 1 null kept')
        print('  PASS: None is a value, not a zero and not a reason to drop')


def test_the_staircase_is_the_loops_own_best():
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t = MA.attempts_from_loop_dir(_loop_dir(td))
    rec = MA.best_so_far(t.attempts, require_admissible=t.gate_record)
    # the accepted rounds' failures, monotonically non-increasing
    want_final = min(f for _r, a, _s, f in LOOP_ROWS
                     if a and f is not None)
    if not rec:
        fail('no record line at all')
    elif rec[-1][1] != want_final:
        fail('the record ends at %r, the best accepted round is %r'
             % (rec[-1][1], want_final))
    vals = [v for _i, v in rec]
    if any(b > a for a, b in zip(vals, vals[1:])):
        fail('the record line went UP: %s' % vals)
    # A REJECTED round cannot set a record -- the loop rejects exactly what
    # `better()` says is not better.
    probe = list(t.attempts)
    i = next(k for k, a in enumerate(probe) if not a.accepted and
             a.score is not None)
    probe[i] = probe[i]._replace(score=-999.0)
    if MA.best_so_far(probe, require_admissible=t.gate_record) != rec:
        fail('a rejected attempt scoring -999 moved the record')
    if len(_FAIL) == _mark:
        print('    record %s' % [v for _i, v in rec])
        print('  PASS: the staircase is the run\'s own best, kept only')


def test_the_shared_record_rule_did_not_change_awx():
    """THE CONTROL. `best_so_far` replaced `Ribbon`'s own loop, so the claim
    "the algorithm is shared, the policy is each caller's own" needs an
    independent re-implementation of the original to compare against."""
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t = MA.attempts_from_evolve_ledger(_evolve(
            os.path.join(td, 'evolve_k51.json')))
    rows = list(t.attempts)

    # the ORIGINAL rule, rewritten from `Ribbon.__init__` as it stood: best
    # ADMISSIBLE score over every world born at or before each instant, with
    # no accepted/kept condition at all.
    ref, best = [], None
    for a in rows:
        for b in rows:
            if b.index <= a.index and b.admissible and b.score is not None:
                if best is None or b.score < best:
                    best = b.score
        if best is not None:
            ref.append((a.index, best))
    got = MA.best_so_far(rows, require_accepted=False, require_admissible=True)
    if [v for _i, v in got] != [v for _i, v in ref]:
        fail('the shared rule disagrees with the original: %s vs %s'
             % (got, ref))
    else:
        print('    awx policy reproduces %s' % [v for _i, v in ref])
    # and the two policies are genuinely different, or the control is vacuous
    other = MA.best_so_far(rows, require_accepted=True,
                           require_admissible=False)
    if [v for _i, v in other] == [v for _i, v in ref]:
        fail('BROKEN CONTROL: both policies give the same line on this '
             'fixture, so it cannot tell them apart')
    # the hollow rule itself: an open-net world never sets a record
    opens = [a for a in rows if not a.admissible]
    if not opens:
        fail('BROKEN FIXTURE: no world with open nets, so "not admissible '
             'however few vias" is untested')
    elif min(a.score for a in opens) >= min(v for _i, v in ref):
        fail('BROKEN FIXTURE: the open-net world is not the lowest via count, '
             'so it could not have set a phantom record anyway')
    if len(_FAIL) == _mark:
        print('  PASS: one algorithm, two policies, neither changed')


def test_nothing_is_synthesised():
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        # boards, no sidecars, no ledger
        for n in ('a', 'b', 'c'):
            with open(os.path.join(td, '%s.kicad_pcb' % n), 'w',
                      encoding='utf-8') as f:
                f.write('(kicad_pcb)\n')
        if MA.discover(td) is not None:
            fail('a chain of boards produced attempts -- synth_rounds forbids '
                 'exactly this extension in its own docstring')
        if MA.discover(os.path.join(td, 'a.kicad_pcb')) is not None:
            fail('discovery from a board path synthesised a search')
        if MA.attempts_from_loop_dir(td) is not None:
            fail('the loop adapter invented rounds from nothing')
    if len(_FAIL) == _mark:
        print('  PASS: no sidecars, no ledger, no band')


def test_the_off_arm_returns_the_same_objects():
    _mark = len(_FAIL)
    frames = [Image.new('RGB', (120, 80), (7, 7, 7)) for _ in range(4)]
    ids = [id(f) for f in frames]
    back, rep = MA.attach(frames, None, theme=RT.DARK)
    if back is not frames:
        fail('the OFF arm returned a different list object')
    if [id(f) for f in back] != ids:
        fail('the OFF arm replaced the Image objects')
    if rep.get('drawn'):
        fail('the OFF arm reported drawn=True')
    if 'one attempt' not in MA.status_line(rep):
        fail('the OFF arm does not say WHY: %r' % MA.status_line(rep))
    # one attempt is also an OFF arm, and says something different
    one = MA.Track((MA.Attempt(0, 'r0', 'round', None, True, False, 3.0,
                               False, None),), 'failures', 'loop', '')
    back2, rep2 = MA.attach(frames, one, theme=RT.DARK)
    if back2 is not frames or rep2.get('drawn'):
        fail('a single attempt drew a band')
    if 'one attempt on disk' not in (rep2.get('why') or ''):
        fail('the single-attempt refusal is not named: %r' % rep2.get('why'))
    if len(_FAIL) == _mark:
        print('    %s' % MA.status_line(rep))
        print('  PASS: no OFF state reads like success')


def test_the_band_keeps_the_frame_invariant():
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t = MA.attempts_from_loop_dir(_loop_dir(td))
    for w, h in ((320, 200), (160, 160), (901, 309)):
        frames = [Image.new('RGB', (w, h), (7, 7, 7)) for _ in range(5)]
        n_before = len(frames)
        back, rep = MA.attach(frames, t, theme='dark')
        sizes = {f.size for f in back}
        if len(sizes) != 1:
            fail('%dx%d: the band left %d sizes: %s' % (w, h, len(sizes),
                                                        sizes))
        if len(back) != n_before:
            fail('%dx%d: the band changed the frame COUNT %d -> %d'
                 % (w, h, n_before, len(back)))
        got = sizes.pop()
        if got[0] != w or got[1] <= h:
            fail('%dx%d: became %s -- the band grows the HEIGHT only'
                 % (w, h, got))
        if got[1] - h < MA.BAND_MIN_PX:
            fail('%dx%d: band is %d px, below the %d px legibility floor'
                 % (w, h, got[1] - h, MA.BAND_MIN_PX))
        try:
            FL.assert_frames_uniform([f.size for f in back])
        except FL.FrameSizeError as exc:
            fail('%dx%d: %s' % (w, h, exc))
        print('    %4dx%-4d -> %s  (+%d px band)' % (w, h, got, got[1] - h))
    if len(_FAIL) == _mark:
        print('  PASS: one size, same count, height only')


def test_the_band_actually_draws():
    """A drawer wrapped in `except Exception: pass` returns None whether it
    drew or not. Count the ink."""
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t = MA.attempts_from_loop_dir(_loop_dir(td))
    bg = RT.DARK.rgb('ground')
    img = Image.new('RGB', (700, 130), bg)
    MA.draw_track(ImageDraw.Draw(img), FL.Box(0, 0, 700, 130), t,
                  theme=RT.DARK)
    cols = {c for _n, c in img.getcolors(1 << 20)}
    if len(cols) < 4:
        fail('the band drew %d colour(s) -- it swallowed an exception'
             % len(cols))
    for role in ('status_best', 'status_kept', 'op_descend'):
        if RT.DARK.rgb(role) not in cols:
            fail('%s never reached the canvas' % role)
    if RT.DARK.rgb('status_dropped') not in cols:
        fail('the ungraded round left no tick')
    # an EMPTY track must leave the canvas untouched, or the check above is
    # only measuring that something was drawn
    blank = Image.new('RGB', (700, 130), bg)
    MA.draw_track(ImageDraw.Draw(blank), FL.Box(0, 0, 700, 130), None,
                  theme=RT.DARK)
    if {c for _n, c in blank.getcolors(1 << 20)} != {bg}:
        fail('draw_track put ink on the canvas with no track')
    if len(_FAIL) == _mark:
        print('    %d colours, record + kept + operator + ungraded all present'
              % len(cols))
        print('  PASS: the band draws, and an empty one draws nothing')


def test_the_horizon_grows_with_the_film():
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t = MA.attempts_from_loop_dir(_loop_dir(td))
    bg = RT.DARK.rgb('ground')
    ink = []
    for up in (0, 4, 8):
        img = Image.new('RGB', (700, 130), bg)
        MA.draw_track(ImageDraw.Draw(img), FL.Box(0, 0, 700, 130), t,
                      upto=up, theme=RT.DARK)
        ink.append(sum(n for n, c in img.getcolors(1 << 20)
                       if c == RT.DARK.rgb('op_descend')))
    if not (ink[0] < ink[1] < ink[2]):
        fail('the graph does not grow with the horizon: %s' % ink)
    else:
        print('    operator ink at horizon 0/4/8: %s' % ink)
    if len(_FAIL) == _mark:
        print('  PASS: it grows with the film rather than spoiling it')


def test_make_film_attaches_before_it_badges():
    """THE ORDERING CLAIM, asserted by pixel rather than by source order.

    `_badge` draws nested rectangles around the WHOLE frame it is given. Attach
    the band afterwards and the border encloses only the board, so the bottom
    rows of a badged frame stop being badge colour -- which is exactly the trap
    `movie_panels.py:40-44` documents for panels.
    """
    _mark = len(_FAIL)
    try:
        import make_film as mf
    except Exception as exc:                                  # noqa: BLE001
        print('  SKIP: %s' % exc)
        return
    with tempfile.TemporaryDirectory() as td:
        sys.path.insert(0, os.path.join(ROOT, 'tests'))
        from test_film_composition import _variant
        good = os.path.join(td, 'good.kicad_pcb')
        bad = os.path.join(td, 'bad.kicad_pcb')
        _variant(BOARD, good, dx=1.0, dy=1.0, n=4)
        _variant(BOARD, bad, dx=-4.0, dy=3.0, n=4)
        shots = mf.parse_positional([BOARD, good, bad],
                                    [os.path.basename(bad)])
        t = MA.attempts_from_loop_dir(_loop_dir(td))
        # camera='auto' and this size deliberately: with the camera off a
        # placement-only attempt changes NO copper, so its beat is one frame
        # and nothing is badged -- the probe would then pass vacuously on an
        # unbadged film. `test_film_composition` uses the same arm.
        off = mf.build_film(shots, size=400, fps=6.0, camera='auto',
                            quiet=True, attempts_from='')
        on = mf.build_film(shots, size=400, fps=6.0, camera='auto',
                           quiet=True, attempts=t)
        if not off or not on:
            fail('no frames')
            return
        if on[0].height <= off[0].height:
            fail('the band did not reach build_film (%s vs %s)'
                 % (on[0].size, off[0].size))
        if len({f.size for f in on}) != 1:
            fail('the banded film has %d sizes' % len({f.size for f in on}))
        want = RT.default_theme().rgb('status_tried')
        badged = [f for f in on if f.convert('RGB').getpixel((0, 0)) == want]
        if not badged:
            fail('no frame is badged at all -- the probe or the fixture moved')
        else:
            # the badge must enclose the BAND too: probe the bottom-left of a
            # badged frame, which is inside the band's rows.
            f = badged[0].convert('RGB')
            if f.getpixel((0, f.height - 1)) != want:
                fail('the badge stops above the band -- attach() ran AFTER '
                     '_badge, so the border encloses only the board')
            else:
                print('    %d/%d frames badged, border reaches row %d'
                      % (len(badged), len(on), f.height - 1))
    if len(_FAIL) == _mark:
        print('  PASS: band first, badge second, border around the whole frame')


TESTS = (
    test_three_producers_one_record_type,
    test_the_axis_is_the_accept_rule_and_is_never_mixed,
    test_an_ungraded_attempt_is_not_a_zero,
    test_the_staircase_is_the_loops_own_best,
    test_the_shared_record_rule_did_not_change_awx,
    test_nothing_is_synthesised,
    test_the_off_arm_returns_the_same_objects,
    test_the_band_keeps_the_frame_invariant,
    test_the_band_actually_draws,
    test_the_horizon_grows_with_the_film,
    test_make_film_attaches_before_it_badges,
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
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
