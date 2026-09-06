#!/usr/bin/env python3
"""#887: which instant of the RUN a movie frame is showing, and how it says so.

The movie path records no time at all -- a frame carries a burned-in label and
nothing else, and the GIF's `duration=` is playback speed. The run DOES record
time, one JSON row per wrapped command. This file grades the bridge.

Three things it must never do, each of which has a test:

  * count DOWN. An exact countdown was possible -- the movie is built after the
    run, so t1 - instant is arithmetic over recorded facts -- and it was
    removed anyway, because it reads as "time left in this video" and means
    "time that remained in the run";
  * present a corrected instant as a measured one -- the monotone clamp marks
    itself in the basis;
  * change a frame's size, which would silently degrade the whole movie to GIF.

Every frame also carries an ABSOLUTE UTC instant, so it is placeable in time
once it is out of the movie and away from the ledger.

No boards and no rendering here beyond a handful of tiny PIL images, so it runs
in a second on any machine.
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
    print('SKIP: Pillow is not installed, so no frame can be stamped')
    sys.exit(77)

import cmd_timing as ct                                    # noqa: E402

FIX = os.path.join(TESTS, 'fixtures', 'cmd_timing', 'synthetic_run.jsonl')

BAD = []


def want(cond, label, extra=''):
    if cond:
        print('  PASS: %s' % label)
    else:
        BAD.append(label)
        print('  FAIL: %s %s' % (label, extra))


def _rows(specs):
    """specs: (label, t_start, wall, exit, [board written])."""
    out = []
    for label, t0, wall, code, board in specs:
        argv = ['python3', 'py_router/route.py']
        if board:
            argv.append('work/' + board)
        out.append({'label': label, 't_start': float(t0),
                    't_end': float(t0 + wall), 'wall_s': float(wall),
                    'exit': code, 'iso_start': '2026-01-01T00:00:00',
                    'argv': argv, 'cmdline': ' '.join(argv)})
    return out


def _ledger(rows):
    import json
    d = os.path.join(tempfile.mkdtemp(), 'wd')
    os.makedirs(d)
    p = os.path.join(d, 'cmd_timing.jsonl')
    with open(p, 'w', encoding='utf-8') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')
    return p


# ------------------------------------------------------------- the mapping

def test_the_mtime_witness_names_the_command_that_wrote_the_board():
    rows = _rows([('P0-driver', 0, 1, 0, None),
                  ('R1-pour', 100, 10, 0, 'r1_pour.kicad_pcb'),
                  ('R3-route', 200, 20, 0, 'r3_route.kicad_pcb')])
    marks = [('a', '/w/r1_pour.kicad_pcb', 0, 5),
             ('b', '/w/r3_route.kicad_pcb', 5, 10)]
    mt = {'/w/r1_pour.kicad_pcb': 105.0, '/w/r3_route.kicad_pcb': 215.0}
    anc = ct.anchor_steps(marks, rows, mtimes=mt)
    want([a.stage for a in anc] == ['R1-pour', 'R3-route'],
         'each board resolves to the row whose window contains its mtime',
         [a.stage for a in anc])
    want([a.basis for a in anc] == ['mtime', 'mtime'], 'by the mtime witness',
         [a.basis for a in anc])
    want([a.t for a in anc] == [105.0, 215.0],
         'and the instant IS the mtime, not the row boundary', [a.t for a in anc])


def test_exit_zero_rows_win_the_argv_tie():
    """The measured pathology: on run 24 `r4` is first mentioned by a DRY run
    that never wrote it (exit 4), and `r5_prune` by a step that exited 1."""
    rows = _rows([('P5-seat-dry', 10, 1, 4, 'r4.kicad_pcb'),
                  ('P5-seat', 50, 1, 0, 'r4.kicad_pcb')])
    marks = [('a', '/w/r4.kicad_pcb', 0, 5)]
    anc = ct.anchor_steps(marks, rows, mtimes={'/w/r4.kicad_pcb': None})
    want(anc[0].stage == 'P5-seat',
         'the exit-0 mention wins over an earlier failing one', anc[0].stage)
    want(anc[0].basis == 'argv', 'and the basis says it came from argv',
         anc[0].basis)

    only_bad = _rows([('P5-dry', 10, 1, 4, 'r4.kicad_pcb'),
                      ('P5-dry2', 50, 1, 2, 'r4.kicad_pcb')])
    anc2 = ct.anchor_steps(marks, only_bad, mtimes={'/w/r4.kicad_pcb': None})
    want(anc2[0].basis == 'argv?',
         'when EVERY mention failed, the basis says so with a question mark',
         anc2[0].basis)
    want(anc2[0].stage == 'P5-dry', 'and takes the first mention', anc2[0].stage)


def test_a_board_matched_by_name_is_matched_by_BASENAME_not_substring():
    rows = _rows([('R3', 100, 1, 0, 'xr3.kicad_pcb')])
    marks = [('a', '/w/r3.kicad_pcb', 0, 5)]
    anc = ct.anchor_steps(marks, rows, mtimes={'/w/r3.kicad_pcb': None})
    want(anc[0].basis == 'none',
         'r3.kicad_pcb does not match xr3.kicad_pcb -- a substring match would',
         anc[0].basis)


def test_the_instant_is_monotone_across_the_chain_and_says_when_it_was_fixed():
    """Reproduces the run-24 shape where a dry run would place step 3 before
    step 2 if nothing clamped it."""
    rows = _rows([('P-late', 300, 1, 0, 'b1.kicad_pcb'),
                  ('P-early', 100, 1, 0, 'b2.kicad_pcb')])
    marks = [('s1', '/w/b1.kicad_pcb', 0, 5), ('s2', '/w/b2.kicad_pcb', 5, 10)]
    anc = ct.anchor_steps(marks, rows,
                          mtimes={'/w/b1.kicad_pcb': None,
                                  '/w/b2.kicad_pcb': None})
    ts = [a.t for a in anc]
    want(ts == sorted(ts),
         'the chain order is authoritative, so instants never go backwards', ts)
    want('clamped' in anc[1].basis,
         'and a corrected instant SAYS it was corrected, rather than passing '
         'as a measurement', anc[1].basis)


def test_a_step_with_no_row_at_all_gets_no_number():
    rows = _rows([('P0', 0, 1, 0, None)])
    marks = [('s1', '/w/nowhere.kicad_pcb', 0, 5)]
    anc = ct.anchor_steps(marks, rows, mtimes={'/w/nowhere.kicad_pcb': None})
    want(anc[0].t is None and anc[0].basis == 'none',
         'an unmatched beat carries no instant', (anc[0].t, anc[0].basis))
    clock = ct.RunClock(anc, ct.totals(rows), 5)
    r = clock.at(2)
    want(r.elapsed_s is None and r.instant is None,
         'and no elapsed and no instant are invented for it',
         (r.elapsed_s, r.instant))
    lines = clock.lines(2)
    want(any('not in the ledger' in ln for ln in lines),
         'the frame says why, instead of showing a number from the last beat',
         lines)


def test_a_seed_board_older_than_the_run_clamps_to_the_run_start():
    """Measured on run 24: board.kicad_pcb's mtime precedes the first wrapped
    command by 101 s, which is the right answer, not a miss."""
    rows = _rows([('staging', 1000, 1, 0, None), ('P0', 1100, 1, 0, None)])
    marks = [('s1', '/w/seed.kicad_pcb', 0, 5)]
    anc = ct.anchor_steps(marks, rows, mtimes={'/w/seed.kicad_pcb': 900.0})
    want(anc[0].basis == 'pre-run', 'the basis names the situation', anc[0].basis)
    want(anc[0].t == 1000.0,
         'and the instant is the run start, not the older file time', anc[0].t)


# ----------------------------------------------------- counting up, in UTC

def _full_clock():
    rows = _rows([('P0', 0, 10, 0, 'b1.kicad_pcb'),
                  ('R1', 100, 10, 0, 'b2.kicad_pcb'),
                  ('V1', 200, 10, 0, 'b3.kicad_pcb')])
    marks = [('s1', '/w/b1.kicad_pcb', 0, 4),
             ('s2', '/w/b2.kicad_pcb', 4, 8),
             ('s3', '/w/b3.kicad_pcb', 8, 12)]
    mt = {'/w/b1.kicad_pcb': 0.0, '/w/b2.kicad_pcb': 105.0,
          '/w/b3.kicad_pcb': 210.0}
    anc = ct.anchor_steps(marks, rows, mtimes=mt)
    return ct.RunClock(anc, ct.totals(rows), 12), ct.totals(rows)


def test_the_clock_counts_up_and_never_counts_down():
    """The clock is a POSITION in the run, not a time-until.

    A countdown was implemented here and was exact -- `t1 - instant` over two
    recorded facts, verified against run 24. It came out because exact is not
    the same as legible: a countdown reads as "time left in this video" and
    means "time that remained in the run", and a 25-frame GIF that ends in four
    seconds while showing "remaining 0:15:57" invites precisely that misreading.
    `+elapsed of total` says the same thing unambiguously.
    """
    clock, tot = _full_clock()
    elapsed = [clock.at(i).elapsed_s for i in range(12)]
    want(all(e is not None for e in elapsed),
         'every frame has an elapsed reading', elapsed)
    want(all(a <= b + 1e-9 for a, b in zip(elapsed, elapsed[1:])),
         'and it is monotone NON-DECREASING across the film -- a clock that '
         'went backwards would be the visible symptom of a bad mapping',
         elapsed)
    want(elapsed[0] == 0.0, 'the film opens at the run start', elapsed[0])
    want(elapsed[-1] <= tot.run_s + 1e-9,
         'and never exceeds the run it is measuring', (elapsed[-1], tot.run_s))

    joined = ' '.join(clock.lines(3)).lower()
    for word in ('remaining', 'countdown', 'time left', 'eta'):
        want(word not in joined,
             'no frame offers a %r figure' % word, joined)
    want(clock.lines(3)[0].startswith('RUN CLOCK  +'),
         'the reading leads with a PLUS, which is what counting up looks like',
         clock.lines(3)[0])
    want(' of ' in clock.lines(3)[0],
         'and carries the total, so the viewer can subtract if they want the '
         'other number', clock.lines(3)[0])


def test_every_frame_carries_an_absolute_utc_instant():
    """What makes a frame self-describing once it leaves the movie.

    The ledger's own `iso_start` is LOCAL time with no offset (tee_cmd writes
    `time.localtime`), so it means different things on different machines.
    `t_start` is epoch, so UTC is a total function of a recorded fact.
    """
    clock, tot = _full_clock()
    for i in (0, 5, 11):
        r = clock.at(i)
        want(r.instant is not None, 'frame %d has an absolute instant' % i)
        m = clock.meta(i)
        want(m.get('krt:utc', '').endswith('Z'),
             'the metadata carries UTC with a Z', m.get('krt:utc'))
        # The instant and the elapsed must agree, or one of them is lying.
        want(abs((r.instant - tot.t0) - r.elapsed_s) < 1e-6,
             'and utc - run_start == elapsed, so the two readings are the same '
             'fact in two forms', (r.instant, tot.t0, r.elapsed_s))
        want(ct.utc_iso(r.instant) == m['krt:utc'],
             'the drawn instant and the stored one are the same')
    want(clock.meta(5).get('krt:run_started_utc', '').endswith('Z'),
         'and the run start is recorded in UTC too, so elapsed is checkable '
         'from the metadata alone', clock.meta(5).get('krt:run_started_utc'))
    at_lines = [ln for ln in clock.lines(5) if ln.startswith('at ')]
    want(len(at_lines) == 1 and at_lines[0].endswith('Z'),
         'and the frame DRAWS it', clock.lines(5))


def test_utc_iso_is_utc_and_not_local():
    want(ct.utc_iso(0) == '1970-01-01T00:00:00Z', 'the epoch is the epoch',
         ct.utc_iso(0))
    want(ct.utc_iso(None) == '', 'None is empty, not a crash')
    # The point of the function: on a machine that is NOT at UTC, this must not
    # follow the local zone. Compare against a value computed independently.
    import calendar
    import time
    epoch = 1787220830.761
    want(ct.utc_iso(epoch)
         == time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(epoch)),
         'it is gmtime, not localtime', ct.utc_iso(epoch))
    want(calendar.timegm(time.strptime(ct.utc_iso(epoch),
                                       '%Y-%m-%dT%H:%M:%SZ')) == int(epoch),
         'and it round-trips back to the epoch it came from', ct.utc_iso(epoch))


def test_a_step_reports_what_its_own_command_cost():
    """The other reading of "how long did this take": elapsed says WHERE in the
    run the frame sits, wall_s says how long that step itself ran."""
    rows = _rows([('P0', 0, 10, 0, 'b1.kicad_pcb'),
                  ('R1', 100, 42.5, 0, 'b2.kicad_pcb')])
    marks = [('s1', '/w/b1.kicad_pcb', 0, 4), ('s2', '/w/b2.kicad_pcb', 4, 8)]
    mt = {'/w/b1.kicad_pcb': 5.0, '/w/b2.kicad_pcb': 120.0}
    clock = ct.RunClock(ct.anchor_steps(marks, rows, mtimes=mt),
                        ct.totals(rows), 8)
    want(clock.anchors[1].wall_s == 42.5,
         'the anchor carries its own row\'s wall_s', clock.anchors[1].wall_s)
    want(clock.meta(5).get('krt:step_wall_s') == 42.5,
         'and the frame records it', clock.meta(5).get('krt:step_wall_s'))
    want(clock.meta(1).get('krt:step_wall_s') == 10.0,
         'per step, not one figure for the film',
         clock.meta(1).get('krt:step_wall_s'))


def test_an_unmapped_beat_is_named_rather_than_gated():
    """`covered` used to withhold the countdown for the whole film when one
    beat was unmapped. With no countdown there is nothing to withhold, so the
    unmapped beats are simply DISCLOSED -- per frame, and as a list."""
    rows = _rows([('P0', 0, 10, 0, 'b1.kicad_pcb'),
                  ('R1', 100, 10, 0, 'b2.kicad_pcb')])
    marks = [('s1', '/w/b1.kicad_pcb', 0, 4),
             ('s2', '/w/b2.kicad_pcb', 4, 8),
             ('s3', '/w/missing.kicad_pcb', 8, 12)]
    mt = {'/w/b1.kicad_pcb': 0.0, '/w/b2.kicad_pcb': 105.0,
          '/w/missing.kicad_pcb': None}
    clock = ct.RunClock(ct.anchor_steps(marks, rows, mtimes=mt),
                        ct.totals(rows), 12)
    want(clock.unmapped() == ['s3'],
         'the unmapped beat is named', clock.unmapped())
    want(clock.at(1).elapsed_s is not None,
         'and the beats that DID map still read normally -- one hole no longer '
         'silences the whole film', clock.at(1).elapsed_s)
    want(clock.at(9).elapsed_s is None,
         'while the hole itself carries no number', clock.at(9).elapsed_s)
    want(any('not in the ledger' in ln for ln in clock.lines(9)),
         'and says why', clock.lines(9))


def test_the_frame_names_its_basis_and_never_says_eta():
    clock, _ = _full_clock()
    for i in (0, 5, 11):
        lines = clock.lines(i)
        want(lines[0].startswith('RUN CLOCK'),
             'frame %d leads with what the number IS' % i, lines[0])
        want(any(ln.startswith('basis') for ln in lines),
             'frame %d names its basis' % i, lines)
        low = ' '.join(lines).lower()
        for bad in ('eta', 'estimat', 'projected', 'at this rate', 'should '
                    'finish'):
            if bad in low:
                want(False, 'frame %d says %r' % (i, bad), low)
                return
    want(True, 'and no frame says ETA, estimate, projected or "at this rate"')


def test_an_interpolated_reading_admits_it():
    clock, _ = _full_clock()
    marked = [i for i in range(12) if clock.at(i).interpolated]
    want(marked, 'some frames inside a beat are interpolated', marked)
    lines = clock.lines(marked[0])
    want('~' in lines[0],
         'an interpolated elapsed carries a ~, because copper reveal is not '
         'uniform in time', lines[0])
    want(any('interpolated within' in ln for ln in lines),
         'and the basis line says so in words', lines)
    boundary = clock.at(0)
    want(not boundary.interpolated,
         'while a frame on a beat boundary is a measurement, unmarked')


# ------------------------------------------------------------- the overlay

def test_the_overlay_never_changes_the_frame_size():
    """The highest-value assertion here, mirroring test_431_placement_movie."""
    clock, _ = _full_clock()
    frames = [Image.new('RGB', (240, 180), (10, 10, 10)) for _ in range(12)]
    before = {f.size for f in frames}
    for i, f in enumerate(frames):
        ct.stamp_run_clock(f, clock.lines(i))
    want({f.size for f in frames} == before,
         'stamping changes no size', {f.size for f in frames})
    want(len({f.size for f in frames}) == 1, 'and they are all still one size')


def test_a_long_overlay_wraps_instead_of_running_off_the_frame():
    """The defect a mock-up caught before any of this was written: one line of
    clock text overflowed a 700 px frame, and PIL clips in silence."""
    long_lines = ['RUN CLOCK  +0:51:23 of 1:17:39',
                  'basis  cmd_timing.jsonl - 153 wrapped commands, mapped by '
                  'mtime inside a wrapped command window']
    # COMPARE against the same text as short lines. A bare "the box is more
    # than 20 rows tall" survived the mutation battery, because two unwrapped
    # lines already clear that. The claim is that the long text occupies MORE
    # vertical space than it would if each line stayed on one row -- which is
    # exactly what wrapping means and what clipping would not do.
    tall = Image.new('RGB', (200, 150), (10, 10, 10))
    ct.stamp_run_clock(tall, long_lines)
    short = Image.new('RGB', (200, 150), (10, 10, 10))
    ct.stamp_run_clock(short, ['RUN CLOCK', 'basis  x'])

    def box_h(img):
        b = ImageChops.difference(img, Image.new('RGB', (200, 150),
                                                 (10, 10, 10))).getbbox()
        return 0 if b is None else b[3] - b[1]

    want(box_h(tall) > box_h(short) + 4,
         'the long text takes MORE rows than the same number of short lines, '
         'i.e. it wrapped instead of being clipped at the frame edge',
         (box_h(tall), box_h(short)))
    # And nothing may be drawn outside the frame or lost off the right edge.
    b = ImageChops.difference(tall, Image.new('RGB', (200, 150),
                                              (10, 10, 10))).getbbox()
    want(b[2] <= 200 and b[3] <= 150, 'and stays inside the frame', b)


def test_the_overlay_draws_bottom_left_and_leaves_the_top_alone():
    frame = Image.new('RGB', (240, 180), (10, 10, 10))
    keep = frame.copy()
    ct.stamp_run_clock(frame, ['RUN CLOCK  +0:01:00 of 0:10:00', 'stage  R1'])
    box = ImageChops.difference(frame, keep).getbbox()
    want(box is not None, 'something was drawn', box)
    want(box[3] > 180 * 0.5,
         'the clock sits in the BOTTOM half, opposite _label\'s top-left', box)
    want(box[1] > 180 * 0.25,
         'and does not reach up into the caption area', box)


# ---------------------------------------------------------- the PNG metadata

def test_the_png_block_is_facts_and_carries_no_prediction():
    clock, tot = _full_clock()
    m = clock.meta(5)
    for k in ('krt:frame', 'krt:frames', 'krt:clock_basis', 'krt:ledger_rows',
              'krt:elapsed_s', 'krt:run_total_s', 'krt:tool_s',
              'krt:outside_s', 'krt:t_epoch', 'krt:stage', 'krt:step',
              'krt:utc', 'krt:run_started_utc', 'krt:step_wall_s'):
        want(k in m, 'the block carries %s' % k, sorted(m))
    want(m['krt:run_total_s'] == round(tot.run_s, 1),
         'and the total matches the ledger', m['krt:run_total_s'])
    bad = [k for k in m if any(w in k.lower()
                               for w in ('eta', 'progress', 'estimate',
                                         'forecast', 'predict', 'remaining'))]
    want(not bad,
         'and no eta, progress or remaining key -- a percentage and a countdown '
         'both invite being read as forecasts, and everything they would say is '
         'derivable from elapsed and the total', bad)
    # A frame is placeable in time with NO other input: the two UTC stamps and
    # the elapsed figure must be mutually consistent on their own.
    import calendar
    import time as _t
    started = calendar.timegm(_t.strptime(m['krt:run_started_utc'],
                                          '%Y-%m-%dT%H:%M:%SZ'))
    at = calendar.timegm(_t.strptime(m['krt:utc'], '%Y-%m-%dT%H:%M:%SZ'))
    want(abs((at - started) - float(m['krt:elapsed_s'])) <= 1.0,
         'krt:utc - krt:run_started_utc == krt:elapsed_s, checkable from the '
         'PNG alone with no ledger present',
         (at - started, m['krt:elapsed_s']))


def test_the_png_block_omits_what_it_cannot_know():
    rows = _rows([('P0', 0, 10, 0, 'b1.kicad_pcb')])
    marks = [('s1', '/w/none.kicad_pcb', 0, 4)]
    clock = ct.RunClock(ct.anchor_steps(marks, rows,
                                        mtimes={'/w/none.kicad_pcb': None}),
                        ct.totals(rows), 4)
    m = clock.meta(1)
    want('krt:remaining_s' not in m,
         'no remaining key at all -- the countdown was removed, see the '
         'RunClock docstring', sorted(m))
    want('krt:utc' not in m,
         'and no UTC instant for a beat that has none', sorted(m))
    want('krt:elapsed_s' not in m,
         'and no elapsed key for a beat that is not in the ledger', sorted(m))
    want(m['krt:clock_basis'] == 'none',
         'while the basis key still says WHY it is missing', m['krt:clock_basis'])


def test_make_movie_actually_draws_the_clock_when_a_ledger_is_beside_the_chain():
    """THE INTEGRATION, which is the one path nothing exercised.

    Every other test here builds a RunClock directly, and the wiring shipped
    broken: the gate was `str(timing).lower() not in ('off', 'none', '0')`, and
    `str(None).lower()` is 'none' -- so the DEFAULT disabled the clock and the
    feature never ran once, in a movie that otherwise looked perfect. Found by
    running it on a real work dir and finding no krt: keys in the PNGs.
    """
    import glob
    import shutil
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    import make_movie as MM

    src = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
    wd = os.path.join(tempfile.mkdtemp(), 'run')
    os.makedirs(wd)
    b1 = os.path.join(wd, 'step1.kicad_pcb')
    b2 = os.path.join(wd, 'step2.kicad_pcb')
    shutil.copy(src, b1)
    shutil.copy(src, b2)
    os.utime(b1, (1000.0, 1000.0))
    os.utime(b2, (1200.0, 1200.0))
    import json
    with open(os.path.join(wd, 'cmd_timing.jsonl'), 'w', encoding='utf-8') as f:
        for r in _rows([('P1-place', 990, 20, 0, 'step1.kicad_pcb'),
                        ('R1-route', 1190, 20, 0, 'step2.kicad_pcb')]):
            f.write(json.dumps(r) + '\n')

    png_dir = os.path.join(wd, 'frames')
    out = MM.make_movie([b1, b2], out=os.path.join(wd, 'm.gif'), size=160,
                        quiet=True, png_dir=png_dir)
    want(out and os.path.exists(out), 'the movie is written', out)
    frames = sorted(glob.glob(os.path.join(png_dir, '*.png')))
    want(frames, 'and PNG frames were dumped', len(frames))
    text = Image.open(frames[len(frames) // 2]).text
    krt = {k: v for k, v in text.items() if k.startswith('krt:')}
    want(krt, 'the frames carry the krt: timing block -- WITHOUT ASKING, '
              'because a ledger sits beside the chain', sorted(text))
    want(krt.get('krt:ledger_rows') == '2',
         'read from that ledger', krt.get('krt:ledger_rows'))
    want('krt:elapsed_s' in krt and 'krt:stage' in krt,
         'with an elapsed and a stage', sorted(krt))

    # And the OFF switch really switches it off.
    png2 = os.path.join(wd, 'frames_off')
    MM.make_movie([b1, b2], out=os.path.join(wd, 'm2.gif'), size=160,
                  quiet=True, png_dir=png2, timing='off')
    f2 = sorted(glob.glob(os.path.join(png2, '*.png')))
    off = {k for k in Image.open(f2[len(f2) // 2]).text if k.startswith('krt:')}
    want(not off, 'timing="off" writes no timing block at all', off)


def test_a_movie_with_no_ledger_beside_it_carries_no_timing_block():
    """The presence gate, from the other side: every existing movie is untouched."""
    import glob
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    import make_movie as MM
    src = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
    d = os.path.join(tempfile.mkdtemp(), 'plain')
    os.makedirs(d)
    png_dir = os.path.join(d, 'frames')
    MM.make_movie([src], out=os.path.join(d, 'm.gif'), size=160, quiet=True,
                  png_dir=png_dir)
    frames = sorted(glob.glob(os.path.join(png_dir, '*.png')))
    want(frames, 'frames were dumped', len(frames))
    krt = {k for k in Image.open(frames[0]).text if k.startswith('krt:')}
    want(not krt,
         'a chain with no cmd_timing.jsonl beside it gets no clock and no '
         'metadata -- the trigger is the ledger, not a mode', krt)


def test_clock_for_returns_none_without_a_ledger():
    want(ct.clock_for([('a', 'b', 0, 1)], None, 1) is None, 'no path, no clock')
    empty = os.path.join(tempfile.mkdtemp(), 'nope.jsonl')
    want(ct.clock_for([('a', 'b', 0, 1)], empty, 1) is None,
         'a missing ledger yields no clock rather than an empty one')
    want(ct.clock_for([], FIX, 1) is None, 'and no marks, no clock')


TESTS_TO_RUN = [
    test_the_mtime_witness_names_the_command_that_wrote_the_board,
    test_exit_zero_rows_win_the_argv_tie,
    test_a_board_matched_by_name_is_matched_by_BASENAME_not_substring,
    test_the_instant_is_monotone_across_the_chain_and_says_when_it_was_fixed,
    test_a_step_with_no_row_at_all_gets_no_number,
    test_a_seed_board_older_than_the_run_clamps_to_the_run_start,
    test_the_clock_counts_up_and_never_counts_down,
    test_every_frame_carries_an_absolute_utc_instant,
    test_utc_iso_is_utc_and_not_local,
    test_a_step_reports_what_its_own_command_cost,
    test_an_unmapped_beat_is_named_rather_than_gated,
    test_the_frame_names_its_basis_and_never_says_eta,
    test_an_interpolated_reading_admits_it,
    test_the_overlay_never_changes_the_frame_size,
    test_a_long_overlay_wraps_instead_of_running_off_the_frame,
    test_the_overlay_draws_bottom_left_and_leaves_the_top_alone,
    test_the_png_block_is_facts_and_carries_no_prediction,
    test_the_png_block_omits_what_it_cannot_know,
    test_make_movie_actually_draws_the_clock_when_a_ledger_is_beside_the_chain,
    test_a_movie_with_no_ledger_beside_it_carries_no_timing_block,
    test_clock_for_returns_none_without_a_ledger,
]


def main():
    for fn in TESTS_TO_RUN:
        print('--- %s' % fn.__name__)
        try:
            fn()
        except Exception as exc:                            # noqa: BLE001
            import traceback
            BAD.append('%s RAISED %s' % (fn.__name__, exc))
            traceback.print_exc()
    if BAD:
        print('\nFAILED: %d' % len(BAD))
        for b in BAD:
            print('  - %s' % b)
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
