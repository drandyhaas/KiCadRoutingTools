#!/usr/bin/env python3
"""The search behind a film: three producers, one record type, one axis
(#946, #1021).

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
  * **the shared record rule did not change `awx`'s line.** `best_so_far` is
    now one function with two policy flags; the control is an independent
    re-implementation of `Ribbon`'s original loop, compared row for row;
  * **`make_film` attaches the band BEFORE its badge loop**, asserted by pixel:
    `_badge` borders the frame it is given, so an after-attach band would leave
    the border around the board only. The band is the stage3d frame's
    benchmark band now: the attempts band's drawing (`attach`, `draw_track`)
    was retired with every film layout but stage3d, and its tests with it.
"""
import json
import os
import sys
import tempfile

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
    print('SKIP: needs Pillow (the film tests render)')
    sys.exit(77)

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
                   'vias': 400 - rnd,
                   # The placement-only PROXIES a real loop also records.
                   # They are a SCREEN, not the judge -- deliberately moving
                   # the OPPOSITE way from `failures`, so an axis that picked
                   # one of them would draw a staircase pointing the wrong way
                   # and this fixture is what notices.
                   'ratsnest_crossings': 100 + rnd * 5,
                   'ratsnest_hpwl': 1000 + rnd * 40,
                   'ratsnest_length': 2000 + rnd * 30}
            # DELIBERATELY anti-correlated with `failures`: if the axis
            # silently fell back, the staircase would be the other one and
            # this fixture is the only thing that could tell.
            #
            # AND ROUND 0 CARRIES NONE, which is how the real producer writes
            # it: `place_route_loop` keeps the baseline judge's answer in a
            # local, not in `metrics`. Without that hole a per-ROW fallback is
            # indistinguishable from a per-LIST choice, because every row has
            # both keys -- the phase-11 verifier measured exactly that, and the
            # row it built from the real `write_round_sidecar` is what this
            # reproduces.
            if accept_cmd and rnd > 0:
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
        # converge: 4, not 5 -- the fixture's one PLACEMENT row is off the
        # verdict axis since #1042 (it scores the copper-free board), and
        # the note says so.
        if t2 is not None and 'placement lap' not in t2.note:
            fail('the converge note does not say a placement lap was taken '
                 'off the axis: %r' % t2.note)
        for name, t, n in (('loop', t1, len(LOOP_ROWS)), ('converge', t2, 4),
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
    want = [None if (f is None or r == 0) else float(-f + 100)
            for r, _a, _s, f in LOOP_ROWS]
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


def test_the_lineage_is_resolved_from_the_parent_board():
    """`parent` names a BOARD basename and the x-axis is the round number, so
    the edge has to be resolved back through the board a round produced.

    Unpinned, `parent=None` for every row passed all eleven checks -- the band
    then draws a scatter with no lineage at all, which is the one thing the
    `parent` field exists to make drawable. A rejected round's parent is the
    last ACCEPTED board, which is exactly why the field exists.
    """
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t = MA.attempts_from_loop_dir(_loop_dir(td))
    got = {a.index: a.parent for a in t.attempts}
    # round 0's parent is the chain INPUT, which no round wrote -- it is the
    # root, and None there is correct rather than missing.
    want = {0: None, 1: 0, 2: 0, 3: 2, 4: 2, 5: 4, 6: 4, 7: 6, 8: 6}
    if got != want:
        fail('lineage %s, expected %s' % (got, want))
    if all(v is None for v in got.values()):
        fail('no attempt has a parent at all -- the band would draw a scatter')
    # and a REJECTED round hangs off the last ACCEPTED board, not off N-1
    if got[5] == 4 and got[7] == 6:
        print('    %d edges; rejected rounds hang off the last ACCEPTED board'
              % sum(1 for v in got.values() if v is not None))
    else:
        fail('a rejected round is parented on N-1: %s' % got)
    if len(_FAIL) == _mark:
        print('  PASS: the search is a tree, and the tree is on disk')


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

    # THE ORIGINAL RULE, transcribed from `Ribbon.__init__` as it stood at
    # de52e40c5 -- ONE ENTRY PER UNIQUE `born` INSTANT, not per world:
    #
    #     times = sorted({w.born for w in reg.worlds.values() if w.grade})
    #     for t in times:
    #         for w in reg.worlds.values():
    #             if w.born <= t and w.grade and not w.grade[0]: best = min(...)
    #         if best is not None: record.append((t, best))
    #
    # The first version of this control looped per ROW, which is a THIRD
    # algorithm: it agreed with neither the original nor the code it was
    # guarding, so it could not fail on the change it existed to detect. The
    # verifier measured the real divergence by loading both Ribbon classes
    # side by side: 2 gold record labels at the parent, 4 at HEAD.
    ref, best = [], None
    for t in sorted({a.index for a in rows}):
        for b in rows:
            if b.index <= t and b.admissible and b.score is not None:
                if best is None or b.score < best:
                    best = b.score
        if best is not None:
            ref.append((t, best))
    got = MA.best_so_far(rows, require_accepted=False, require_admissible=True)
    # WHOLE PAIRS, not just the values: the defect was an extra entry at an
    # index that already had one, and comparing values alone would have let
    # `[(1, 90), (1, 90)]` pass as `[(1, 90)]`.
    if got != ref:
        fail('the shared rule disagrees with the original: %s vs %s'
             % (got, ref))
    if len({i for i, _v in got}) != len(got):
        fail('the record has two entries at one index: %s -- Ribbon.draw '
             'labels each new record once, so that is a duplicate gold number '
             'for a value that was never the record at the end of an instant'
             % got)
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
        # A MALFORMED LEDGER must not take the film down. `make_film.main()`
        # calls this adapter UNGUARDED, and a line that parses as a scalar or
        # a list raises AttributeError on `.get` -- one stray line, no film.
        bad = os.path.join(td, 'bad.jsonl')
        with open(bad, 'w', encoding='utf-8') as f:
            for junk in ('42', '["not", "a", "row"]', '"a string"', 'null',
                         '{oops'):
                f.write(junk + '\n')
            f.write(json.dumps({'iteration': 0, 'kind': 'completion',
                                'result_sha': 'a' * 8, 'accepted': True,
                                'score': {'blocking': 3}}) + '\n')
        try:
            t = MA.attempts_from_converge_ledger(bad)
        except Exception as exc:                              # noqa: BLE001
            fail('a malformed ledger raised %s -- make_film calls this '
                 'unguarded' % exc.__class__.__name__)
            t = None
        if t is not None and len(t.attempts) != 1:
            fail('the malformed lines were read as %d row(s)'
                 % len(t.attempts))
    if len(_FAIL) == _mark:
        print('  PASS: no sidecars, no ledger, no track')


def test_make_film_attaches_before_it_badges():
    """THE ORDERING CLAIM, asserted by pixel rather than by source order.

    `_badge` draws nested rectangles around the WHOLE frame it is given. Attach
    the band afterwards and the border encloses only the board, so the bottom
    rows of a badged frame stop being badge colour. The band is the stage3d
    frame's benchmark band (the only film layout; it folded the attempts band
    in), discovered from the loop sidecars.
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
        loop = _loop_dir(td)
        # camera='auto' deliberately: with the camera off a placement-only
        # attempt changes NO copper, so its beat is one frame and nothing is
        # badged -- the probe would then pass vacuously on an unbadged film.
        # `test_film_composition` uses the same arm. Size 1000: the band is
        # declined under the stage3d board's 70% height floor below that.
        off = mf.build_film(shots, size=1000, fps=6.0, camera='auto',
                            quiet=True, attempts_from='',
                            placement={'board3d': '2d'})
        on = mf.build_film(shots, size=1000, fps=6.0, camera='auto',
                           quiet=True, attempts_from=loop,
                           placement={'board3d': '2d'})
        if not off or not on:
            fail('no frames')
            return
        # #946/C4: a DECLARED layout reserves the band INSIDE its frame, so
        # the banded film is the SAME size as the unbanded one -- it used to
        # grow, which is how `--aspect 16:9` came out taller than 16:9. The
        # band reached build_film when the pixels differ, not the size.
        if on[0].size != off[0].size:
            fail('the band changed a declared frame size (%s vs %s)'
                 % (on[0].size, off[0].size))
        from PIL import ImageChops
        if ImageChops.difference(on[-1].convert('RGB'),
                                 off[-1].convert('RGB')).getbbox() is None:
            fail('the band did not reach build_film: banded and unbanded '
                 'films are pixel-identical')
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


def test_a_card_and_a_band_in_one_film_are_one_size():
    """`make_film` cuts its spliced cards at `frames[0].size`, so the band has
    to be attached BEFORE that size is read.

    Hoisting the read above the band passes every other check in this file and
    in `test_film_composition` while producing a demonstrably two-size film --
    which Pillow does not raise on. This is the case that has both.
    """
    _mark = len(_FAIL)
    try:
        import make_film as mf
        from PIL import Image as _I
    except Exception as exc:                                  # noqa: BLE001
        print('  SKIP: %s' % exc)
        return
    with tempfile.TemporaryDirectory() as td:
        sys.path.insert(0, os.path.join(ROOT, 'tests'))
        from test_film_composition import _variant
        good = os.path.join(td, 'g.kicad_pcb')
        _variant(BOARD, good, n=3)
        png = os.path.join(td, 'why.png')
        _I.new('RGB', (1234, 200), (10, 90, 160)).save(png)
        loop = _loop_dir(td)
        shots = ([mf.card_shot(png, 'the delta that motivated this')] +
                 mf.parse_positional([BOARD, good], []))
        # size 1000: below it the stage3d frame declines the band
        frames = mf.build_film(shots, size=1000, fps=6.0, camera='off',
                               quiet=True, attempts_from=loop,
                               placement={'board3d': '2d'})
        bare = mf.build_film(shots, size=1000, fps=6.0, camera='off',
                             quiet=True, attempts_from='',
                             placement={'board3d': '2d'})
        if not frames or not bare:
            fail('no frames')
            return
        from PIL import ImageChops
        if ImageChops.difference(frames[-1].convert('RGB'),
                                 bare[-1].convert('RGB')).getbbox() is None:
            fail('BROKEN: no band was drawn, so the card/band size claim '
                 'is vacuous')
        sizes = {f.size for f in frames}
        if len(sizes) != 1:
            fail('a film with a card AND a band has %d sizes: %s -- the card '
                 'was cut before the band was attached' % (len(sizes), sizes))
        else:
            print('    %d frames (card + band), one size %s'
                  % (len(frames), sizes.pop()))
    if len(_FAIL) == _mark:
        print('  PASS: the band is attached before the cards are cut')


def test_a_placement_run_is_graded_on_the_ROUTED_result():
    """The question every reader of a placement film asks, pinned.

    `place_route_loop` is a place-AND-route loop: a round moves parts, routes,
    and is kept or thrown away on `better()`, whose leading term is `failures`
    -- copper, from the route summary. So the y-axis of a PLACEMENT film is the
    routed result, and that is the run's own accept rule rather than a
    placement score.

    The sidecar also carries `ratsnest_crossings` / `hpwl` / `length`, which
    are placement-only proxies and a SCREEN rather than the judge. This fixture
    moves them the OPPOSITE way from `failures`, so an axis that picked one
    would draw a staircase pointing the wrong way -- and it would still look
    perfectly plausible.
    """
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t = MA.attempts_from_loop_dir(_loop_dir(td))
    if 'failures' not in t.metric:
        fail('a place-and-route loop is not graded on failures: %r' % t.metric)
    got = [a.score for a in t.attempts]
    want = [None if f is None else float(f) for _r, _a, _s, f in LOOP_ROWS]
    if got != want:
        fail('the axis is not `failures`: %s vs %s' % (got, want))
    # the proxies move the other way, so the record would INVERT on them
    rec = [v for _i, v in MA.best_so_far(t.attempts)]
    if rec != sorted(rec, reverse=True):
        fail('the record does not fall: %s' % rec)
    for bad in ('crossings', 'hpwl', 'length', 'ratsnest'):
        if bad in t.metric:
            fail('a placement PROXY reached the axis label: %r' % t.metric)
    print('    axis %r; record %s (the proxies rise while this falls)'
          % (t.metric, rec))
    if len(_FAIL) == _mark:
        print('  PASS: the judge is the routed result, not the screen')


def test_place_and_route_is_one_graph():
    """#946/C4: a place-and-route run leaves a converge ledger AND loop
    sidecars; `discover` joins them into ONE track whose x-axis is laps across
    both halves -- the first half keeps its indices, the second is shifted
    past it, and the second half's root descends from the first half's last
    accepted attempt."""
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        _loop_dir(td)
        _ledger(os.path.join(td, 'ledger.jsonl'))
        t = MA.discover(td)
    if t is None or t.source != 'converge+loop':
        fail('discover did not join the halves: %r' % (t and t.source))
        return
    # 4 ledger laps on the axis: the fixture's placement row is off it (#1042)
    n_led, n_loop = 4, len(LOOP_ROWS)
    idx = [a.index for a in t.attempts]
    # the ledger half keeps its OWN iteration numbers (lap 1, the placement
    # lap, is off the axis and leaves its gap), the loop half follows it
    want = [0, 2, 3, 4] + list(range(5, 5 + n_loop))
    if idx != want:
        fail('the joined x-axis is not %r: %r' % (want, idx))
    first_loop = t.attempts[n_led]
    last_acc_ledger = max(a.index for a in t.attempts[:n_led] if a.accepted)
    if first_loop.parent != last_acc_ledger:
        fail('the loop half does not descend from the ledger-half last kept '
             'attempt: parent %r, expected %r'
             % (first_loop.parent, last_acc_ledger))
    shifted = [a.parent for a in t.attempts[n_led + 1:] if a.parent is not None]
    if any(p < n_led for p in shifted):
        fail('a loop parent was not shifted into the loop half: %r' % shifted)
    if 'blocking' not in t.metric or 'failures' not in t.metric:
        fail('the joined axis does not say it is both terms: %r' % t.metric)
    if len(_FAIL) == _mark:
        print('  PASS: %d ledger laps + %d loop rounds -> one axis 0..%d (%s)'
              % (n_led, n_loop, idx[-1], t.note))


def test_lineage_falls_back_to_last_accepted_and_says_so():
    """Until `converge.py record` takes a parent explicitly (#1034), a row with
    no `parent_sha` is drawn from the LAST ACCEPTED row before it -- the
    loop's own rule -- and the note COUNTS those guesses, because under
    parallel lineages the guess can be wrong."""
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'ledger.jsonl')
        rows = [(0, None, 'a', True, 9), (1, 'a', 'b', False, 12),
                (2, 'a', 'c', True, 8), (3, None, 'd', True, 4),
                (4, 'zzz-not-a-row', 'e', False, 7)]
        with open(p, 'w', encoding='utf-8') as f:
            for it, par, res, acc, b in rows:
                f.write(json.dumps({'iteration': it, 'kind': 'completion',
                                    'parent_sha': par, 'result_sha': res,
                                    'accepted': acc,
                                    'score': {'blocking': b}}) + '\n')
        t = MA.attempts_from_converge_ledger(p)
    par = {a.index: a.parent for a in t.attempts}
    if par[0] is not None:
        fail('the first row is not the root: %r' % par[0])
    if par[1] != 0 or par[2] != 0:
        fail('a recorded parent_sha was not followed: %r' % par)
    if par[3] != 2:
        fail('no parent_sha -> expected the last accepted row 2, got %r'
             % par[3])
    if par[4] != 3:
        fail('an unresolvable parent_sha -> expected last accepted 3, got %r'
             % par[4])
    if '#1034' not in t.note or '2 parent(s)' not in t.note:
        fail('the fallback is not disclosed and counted: %r' % t.note)
    if len(_FAIL) == _mark:
        print('  PASS: parent_sha followed; 2 fallbacks, said: %s' % t.note)


#: `blocking` values that are not a count (#1077), one row each after a
#: graded row -- the shape of #1071's tigard ledger, plus the scalars that
#: `float()` accepted without a word.
_NOT_A_COUNT = ({'a': 1}, [1], 'abc', '10', True, False, -1,
                float('nan'), float('inf'), 10 ** 400)


def test_a_blocking_that_is_not_a_count_is_ungraded_not_raised():
    """#1077. A per-term dict raised `float(b)` inside `make_film.main()`,
    which calls this adapter unguarded; `false` was drawn at 0.0 ADMISSIBLE;
    `"10"` was plotted at 10; a null `iteration` raised `int(None)`."""
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'l.jsonl')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(json.dumps({'iteration': 0, 'kind': 'completion',
                                'result_sha': 'r0', 'accepted': True,
                                'score': {'blocking': 3}}) + '\n')
            for i, b in enumerate(_NOT_A_COUNT, 1):
                f.write(json.dumps({'iteration': i, 'kind': 'completion',
                                    'parent_sha': 'r0',
                                    'result_sha': 'r%d' % i, 'accepted': False,
                                    'score': {'blocking': b}}) + '\n')
        try:
            t = MA.attempts_from_converge_ledger(p)
        except Exception as exc:                            # noqa: BLE001
            fail('the adapter raised on a non-count blocking: %s: %s'
                 % (type(exc).__name__, exc))
            return
        got = {a.index: a for a in t.attempts}
        if len(t.attempts) != len(_NOT_A_COUNT) + 1:
            fail('rows were dropped: %d of %d kept'
                 % (len(t.attempts), len(_NOT_A_COUNT) + 1))
        if got.get(0) is None or got[0].score != 3.0:
            fail('the graded row lost its score: %r' % (got.get(0),))
        for i, b in enumerate(_NOT_A_COUNT, 1):
            a = got.get(i)
            if a is None:
                fail('the %r row is missing' % (b,))
            elif a.score is not None or a.admissible:
                fail('blocking %r was drawn (score %r, admissible %r) rather '
                     'than ungraded' % (b, a.score, a.admissible))
        if 'not a count' not in t.note or \
                '%d with a blocking' % len(_NOT_A_COUNT) not in t.note:
            fail('the note does not count the non-count rows: %r' % t.note)
        note = t.note

        # The row's OTHER fields: a null or boolean `iteration` falls back to
        # the row position (`int(None)` raised; `true` read as 1), and a
        # parent_sha / result_sha that is not a string is no lineage key (a
        # list raised `unhashable`).
        s = os.path.join(td, 'i.jsonl')
        with open(s, 'w', encoding='utf-8') as f:
            f.write(json.dumps({'iteration': 0, 'kind': 'completion',
                                'result_sha': ['r0'], 'accepted': True,
                                'score': {'blocking': 3}}) + '\n')
            f.write(json.dumps({'iteration': None, 'kind': 'completion',
                                'parent_sha': ['r0'], 'accepted': False,
                                'score': {'blocking': 2}}) + '\n')
            f.write(json.dumps({'iteration': True, 'kind': 'completion',
                                'accepted': False,
                                'score': {'blocking': 1}}) + '\n')
        try:
            u = MA.attempts_from_converge_ledger(s)
        except Exception as exc:                            # noqa: BLE001
            fail('a malformed iteration / sha raised: %s: %s'
                 % (type(exc).__name__, exc))
        else:
            idx = [(a.index, a.score) for a in u.attempts]
            if idx != [(0, 3.0), (1, 2.0), (2, 1.0)]:
                fail('a null or boolean iteration did not fall back to the '
                     'row position: %r' % (idx,))

        # `_graded` decides whether placement laps leave the axis. A routing
        # row whose blocking is not a count is NOT a routed verdict, so a
        # placement-only ledger keeps its laps on the axis.
        q = os.path.join(td, 'g.jsonl')
        with open(q, 'w', encoding='utf-8') as f:
            f.write(json.dumps({'iteration': 0, 'kind': 'placement',
                                'result_sha': 'p0', 'accepted': True,
                                'score': {'blocking': 4}}) + '\n')
            f.write(json.dumps({'iteration': 1, 'kind': 'completion',
                                'result_sha': 'c1', 'accepted': True,
                                'score': {'blocking': False}}) + '\n')
        t = MA.attempts_from_converge_ledger(q)
        if 'placement lap(s) off this axis' in t.note or \
                not any(a.score == 4.0 for a in t.attempts):
            fail('a `false` routing blocking counted as a routed verdict and '
                 'took the placement lap off the axis: %r' % t.note)
    if len(_FAIL) == _mark:
        print('  PASS: %d non-count blockings drawn ungraded and counted: %s'
              % (len(_NOT_A_COUNT), note))


def test_the_film_and_the_verdict_agree_on_what_a_blocking_is():
    """#1077/#1088. The film's `_blocking_value` IS converge's
    `blocking_value` -- one function, imported from `ledger_score` -- so the
    two cannot drift. It was a hand mirror pinned by the table below; the
    table stays as a behaviour check on the one rule."""
    _mark = len(_FAIL)
    import converge
    import ledger_score
    if not (MA._blocking_value is converge.blocking_value
            is ledger_score.blocking_value):
        fail('the film and the verdict use different blocking rules again: '
             '%r / %r' % (MA._blocking_value, converge.blocking_value))
    nan, inf = float('nan'), float('inf')
    table = (0, 3, 3.0, 2.5, -0.0, 10 ** 20, 10 ** 400, -10 ** 400,
             -1, -0.5, True, False, None,
             'abc', '10', '', {}, {'a': 1}, [], [1], nan, inf, -inf)
    for v in table:
        try:
            a, b = MA._blocking_value(v), converge.blocking_value(v)
        except Exception as exc:                            # noqa: BLE001
            fail('the rule raised on %r: %s: %s'
                 % (v, type(exc).__name__, exc))
            continue
        same = (a is None and b is None) or (
            a is not None and b is not None and a == b
            and type(a) is type(b))
        if not same:
            fail('film and verdict disagree on %r: film %r, verdict %r'
                 % (v, a, b))
    if len(_FAIL) == _mark:
        print('  PASS: film and verdict agree on all %d values' % len(table))


TESTS = (
    test_three_producers_one_record_type,
    test_the_axis_is_the_accept_rule_and_is_never_mixed,
    test_a_placement_run_is_graded_on_the_ROUTED_result,
    test_the_lineage_is_resolved_from_the_parent_board,
    test_an_ungraded_attempt_is_not_a_zero,
    test_the_staircase_is_the_loops_own_best,
    test_the_shared_record_rule_did_not_change_awx,
    test_nothing_is_synthesised,
    test_make_film_attaches_before_it_badges,
    test_a_card_and_a_band_in_one_film_are_one_size,
    test_place_and_route_is_one_graph,
    test_lineage_falls_back_to_last_accepted_and_says_so,
    test_a_blocking_that_is_not_a_count_is_ungraded_not_raised,
    test_the_film_and_the_verdict_agree_on_what_a_blocking_is,
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
