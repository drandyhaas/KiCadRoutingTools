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
    # the SAME images too -- asserted on BOTH off arms, not only on the
    # no-track one. A copy that happens to look the same is still a copy, and
    # the contract is that the caller's list comes back untouched.
    if [id(f) for f in back2] != ids:
        fail('the single-attempt arm replaced the Image objects')
    if 'one attempt on disk' not in (rep2.get('why') or ''):
        fail('the single-attempt refusal is not named: %r' % rep2.get('why'))
    if len(_FAIL) == _mark:
        print('    %s' % MA.status_line(rep))
        print('  PASS: no OFF state reads like success')


def test_the_band_keeps_the_frame_invariant():
    _mark = len(_FAIL)
    with tempfile.TemporaryDirectory() as td:
        t = MA.attempts_from_loop_dir(_loop_dir(td))
    # A SHORT frame has no room for one, and says so rather than taking most
    # of the picture: measured at 74% of an 86 px frame before the ceiling.
    short = [Image.new('RGB', (560, 86), (7, 7, 7)) for _ in range(3)]
    ids = [id(f) for f in short]
    back, rep = MA.attach(short, t, theme='dark')
    if rep.get('drawn'):
        fail('a 560x86 frame drew a band anyway')
    if [id(f) for f in back] != ids:
        fail('the too-short arm rebuilt the frames')
    if 'no room' not in (rep.get('why') or ''):
        fail('the too-short refusal does not say why: %r' % rep.get('why'))
    else:
        print('    560x86: %s' % MA.status_line(rep))
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
        if got[0] != w:
            fail('%dx%d: became %s -- the band grows the HEIGHT only'
                 % (w, h, got))
        if not rep.get('drawn'):
            # A frame too short for a legible band declines, and both arms
            # are correct -- what is NOT correct is growing by a band nobody
            # can read, or shrinking.
            if got != (w, h):
                fail('%dx%d: declined but the frame still changed to %s'
                     % (w, h, got))
            print('    %4dx%-4d -> declined: %s'
                  % (w, h, (rep.get('why') or '')[:54]))
            continue
        if got[1] <= h:
            fail('%dx%d: drawn but the frame did not grow: %s' % (w, h, got))
        if got[1] - h < MA.BAND_MIN_PX:
            fail('%dx%d: band is %d px, below the %d px legibility floor'
                 % (w, h, got[1] - h, MA.BAND_MIN_PX))
        if (got[1] - h) > h * MA.BAND_MAX_FRAC:
            fail('%dx%d: the band is %.0f%% of the frame -- a time series '
                 'about the run must not dwarf the film it annotates'
                 % (w, h, 100.0 * (got[1] - h) / h))
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
        # `layout='split'` because the band now REFUSES a frame too short to
        # carry it, and this board is 6.5:1 -- at size 400 its legacy frame is
        # 400x62, where a legible band would be over a third of the picture.
        # A declared layout gives the frame its own aspect and the band room.
        off = mf.build_film(shots, size=400, fps=6.0, camera='auto',
                            quiet=True, attempts_from='', layout='split')
        on = mf.build_film(shots, size=400, fps=6.0, camera='auto',
                           quiet=True, attempts=t, layout='split')
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
        t = MA.attempts_from_loop_dir(_loop_dir(td))
        shots = ([mf.card_shot(png, 'the delta that motivated this')] +
                 mf.parse_positional([BOARD, good], []))
        frames = mf.build_film(shots, size=300, fps=6.0, camera='off',
                               quiet=True, attempts=t, layout='split')
        if not frames:
            fail('no frames')
            return
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
    n_led, n_loop = 5, len(LOOP_ROWS)
    idx = [a.index for a in t.attempts]
    if idx != list(range(n_led + n_loop)):
        fail('the joined x-axis is not laps 0..%d: %r'
             % (n_led + n_loop - 1, idx))
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
    # and the joined track DRAWS, record line and all
    im = Image.new('RGB', (800, 160))
    if not MA.draw_track(ImageDraw.Draw(im), FL.Box(0, 0, 800, 160), t):
        fail('the joined track declined to draw')
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


TESTS = (
    test_three_producers_one_record_type,
    test_the_axis_is_the_accept_rule_and_is_never_mixed,
    test_a_placement_run_is_graded_on_the_ROUTED_result,
    test_the_lineage_is_resolved_from_the_parent_board,
    test_an_ungraded_attempt_is_not_a_zero,
    test_the_staircase_is_the_loops_own_best,
    test_the_shared_record_rule_did_not_change_awx,
    test_nothing_is_synthesised,
    test_the_off_arm_returns_the_same_objects,
    test_the_band_keeps_the_frame_invariant,
    test_the_band_actually_draws,
    test_the_horizon_grows_with_the_film,
    test_make_film_attaches_before_it_badges,
    test_a_card_and_a_band_in_one_film_are_one_size,
    test_place_and_route_is_one_graph,
    test_lineage_falls_back_to_last_accepted_and_says_so,
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
