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
import subprocess
import sys
import tempfile

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 300

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

try:
    from PIL import Image, ImageChops, ImageDraw
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


def test_the_opening_frame_does_not_borrow_the_first_steps_basis():
    """Frame 0 is `build_boards`' own `m.snapshot("input")`, before the loop.

    No wrapped command produced it, so it has no stage, and its instant is the
    run's start. It used to return the FIRST ANCHOR's stage and basis, which
    made the opening frame announce `stage P0 / mapped by mtime` about a board
    P0 never wrote and no mtime ever witnessed -- a false provenance on the one
    frame every viewer sees first. An approximate NUMBER would be fine here; a
    false BASIS is not, because the basis is the frame's claim about where its
    number came from.
    """
    rows = _rows([('P0', 1000, 10, 0, 'b1.kicad_pcb')])
    # `first=2`, so frames 0 and 1 precede every mark.
    marks = [('s1', '/w/b1.kicad_pcb', 2, 6)]
    anc = ct.anchor_steps(marks, rows, mtimes={'/w/b1.kicad_pcb': 1005.0})
    clock = ct.RunClock(anc, ct.totals(rows), 6)

    want(anc[0].basis == 'mtime',
         'the first STEP is still mapped by mtime -- without this the test '
         'below proves nothing about frame 0', anc[0].basis)
    r0 = clock.at(0)
    want(r0.basis == 'run-start',
         'frame 0 names its own basis rather than the step it precedes',
         r0.basis)
    want(r0.basis != anc[0].basis,
         "and that basis is NOT the first anchor's -- the whole defect",
         r0.basis)
    want(r0.stage is None,
         'with no stage, because no wrapped command produced this frame',
         r0.stage)
    want(r0.elapsed_s == 0.0 and r0.instant == 1000.0,
         'while the instant is still the run start, which is true',
         (r0.elapsed_s, r0.instant))

    lines = ' | '.join(clock.lines(0))
    want('the board the run started from' in lines,
         'and the overlay says so in words rather than "unlabelled", which '
         'would read as a broken ledger', lines)
    want('mapped by run-start' in lines, 'the basis line carries it too', lines)
    want(clock.at(2).basis == 'mtime' and clock.at(2).stage == 'P0',
         'the first REAL beat is untouched',
         (clock.at(2).basis, clock.at(2).stage))
    want('krt:stage' not in clock.meta(0),
         'and the PNG block omits a stage it does not have', clock.meta(0))


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

    # EVERY frame, not frame 3. Reading one frame missed that frames 0 and
    # 8-11 are not interpolated while 3 is, so a countdown added under
    # `if not r.interpolated` would have shipped on 5 of 12 frames with this
    # test green.
    for i in range(12):
        joined = ' '.join(clock.lines(i)).lower()
        for word in ('remaining', 'countdown', 'time left', 'eta', 'to go'):
            if word in joined:
                want(False, 'frame %d offers a %r figure' % (i, word), joined)
                return
    want(True, 'no frame of the film offers a remaining/countdown figure')

    for i in (0, 3, 11):
        head = clock.lines(i)[0]
        want(head.startswith('RUN CLOCK  +'),
             'frame %d leads with a PLUS, which is what counting up looks like'
             % i, head)
    # The TOTAL, by value. `' of ' in head` passed while the number was
    # anything at all: swapping run_s for tool_s survived, and on run 24 that
    # would have drawn "+0:51:23 of 0:04:13".
    want(clock.lines(3)[0].endswith(' of %s' % ct.fmt_hms(tot.run_s)),
         'and the total it carries is the RUN span, not some other duration',
         (clock.lines(3)[0], ct.fmt_hms(tot.run_s)))
    want(ct.fmt_hms(tot.tool_s) not in clock.lines(3)[0],
         'and specifically not the tool time, which is 5.4%% of it on run 24',
         (clock.lines(3)[0], ct.fmt_hms(tot.tool_s)))


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
    # krt:t_epoch BY VALUE. Presence-only let it revert to the run start --
    # the very thing the count-up commit says it changed -- undetected.
    for i in (0, 5, 11):
        r = clock.at(i)
        want(abs(float(clock.meta(i)['krt:t_epoch']) - r.instant) < 1e-3,
             'frame %d: krt:t_epoch is THIS frame, not the run start' % i,
             (clock.meta(i)['krt:t_epoch'], r.instant, tot.t0))
    want(len({clock.meta(i)['krt:t_epoch'] for i in range(12)}) > 1,
         'and it varies across the film, so a constant cannot satisfy it')
    at_lines = [ln for ln in clock.lines(5) if ln.startswith('at ')]
    want(len(at_lines) == 1 and at_lines[0].endswith('Z'),
         'and the frame DRAWS it', clock.lines(5))


def test_utc_iso_is_utc_and_not_local():
    """HARDCODED literals, because every other formulation is inert on CI.

    The first version of this test compared `utc_iso` against `time.gmtime`,
    round-tripped through `calendar.timegm`, and checked `utc_iso(0)` -- all
    three of which a LOCALTIME implementation satisfies when the machine's zone
    IS UTC, which is the CI default. Measured: dropping `datetime.timezone.utc`
    from `cmd_timing.utc_iso` was killed in Europe/Amsterdam and SURVIVED, all
    green, under `TZ=UTC`. Since this is the only UTC check in the branch, the
    headline claim -- every frame carries an absolute UTC instant -- was
    untested exactly where it runs.

    A literal at a non-midnight hour cannot be satisfied by a local clock in any
    zone but UTC, and asserts the format at the same time.
    """
    want(ct.utc_iso(0) == '1970-01-01T00:00:00Z', 'the epoch is the epoch',
         ct.utc_iso(0))
    want(ct.utc_iso(None) == '', 'None is empty, not a crash')
    # 1787220830.761 -> 2026-08-20T10:13:50Z. Verified against run 24's own
    # ledger: its first row's local iso_start is 11:12:09 on a UTC+2 machine,
    # and this instant's frame reported 10:13:50Z.
    want(ct.utc_iso(1787220830.761) == '2026-08-20T10:13:50Z',
         'a known instant formats to its known UTC string -- a hardcoded '
         'literal, so a localtime implementation fails here in every zone but '
         'UTC itself', ct.utc_iso(1787220830.761))
    want(ct.utc_iso(1000000000) == '2001-09-09T01:46:40Z',
         'and a second one, at a different hour', ct.utc_iso(1000000000))
    want(ct.utc_iso(1787220830.761).endswith('Z'),
         'the Z is part of the value, not decoration')
    # Sub-second input must not shift the second.
    want(ct.utc_iso(1787220830.999) == '2026-08-20T10:13:50Z',
         'fractional seconds truncate rather than round up into the next '
         'second', ct.utc_iso(1787220830.999))

    # STRUCTURE, not output, for the one property output cannot show.
    #
    # On a machine whose zone IS UTC -- the CI default -- localtime and gmtime
    # agree on every input, so NO assertion over this function's return value
    # can distinguish them. Measured: a `fromtimestamp(epoch)` mutant with the
    # tzinfo dropped is killed under Europe/Amsterdam and America/Los_Angeles
    # and SURVIVES under TZ=UTC, and that is not a weak assertion, it is a
    # logical impossibility. `time.tzset()` would let a test force a zone, but
    # it does not exist on Windows, which is this repo's primary platform.
    #
    # So pin the shape: the conversion must name a UTC frame of reference.
    import ast
    src = open(os.path.join(ROOT, 'py_router', 'cmd_timing.py'),
               encoding='utf-8').read()
    fn = next(n for n in ast.walk(ast.parse(src))
              if isinstance(n, ast.FunctionDef) and n.name == 'utc_iso')
    # DROP THE DOCSTRING before dumping. utc_iso's docstring explains why it
    # does not use localtime -- and contains the word, so a dump including it
    # failed the `'localtime' not in body` check on correct code. Prose that
    # quotes the thing a check forbids is this repo's own recurring trap.
    stmts = fn.body[1:] if (fn.body and isinstance(fn.body[0], ast.Expr)
                            and isinstance(fn.body[0].value, ast.Constant)
                            and isinstance(fn.body[0].value.value, str)
                            ) else fn.body
    body = ' '.join(ast.dump(st) for st in stmts)
    want('timezone' in body and 'utc' in body,
         'utc_iso names a UTC frame of reference in its own source -- the only '
         'check that can fail on a UTC machine, where localtime and gmtime are '
         'indistinguishable by output', body[:160])
    want('localtime' not in body,
         'and never calls localtime, which is what tee_cmd already wrote and '
         'what this function exists to avoid')


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

    # And on the ARGV path, which is the one used exactly when mtime is gone --
    # a copied work dir, or make_film materialising boards from its store. The
    # mtime branch alone left the argv branch's `wall = r0.get('wall_s')`
    # deletable with this test green.
    argv_anchors = ct.anchor_steps(marks, rows,
                                   mtimes={m[1]: None for m in marks})
    want([a.basis for a in argv_anchors] == ['argv', 'argv'],
         'the control really did fall through to argv',
         [a.basis for a in argv_anchors])
    argv_clock = ct.RunClock(argv_anchors, ct.totals(rows), 8)
    want(argv_clock.meta(5).get('krt:step_wall_s') == 42.5,
         'and the argv path carries the step cost too',
         argv_clock.meta(5).get('krt:step_wall_s'))


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
    # More than one, so a [:1] truncation cannot pass. make_movie prints
    # `', '.join(unmapped[:3])`, so the list itself must be complete.
    many = [('s1', '/w/b1.kicad_pcb', 0, 4)] + [
        ('gap%d' % i, '/w/gone%d.kicad_pcb' % i, 4 + i, 5 + i)
        for i in range(4)]
    mt2 = {'/w/b1.kicad_pcb': 0.0}
    mt2.update({'/w/gone%d.kicad_pcb' % i: None for i in range(4)})
    c2 = ct.RunClock(ct.anchor_steps(many, rows, mtimes=mt2),
                     ct.totals(rows), 8)
    want(c2.unmapped() == ['gap0', 'gap1', 'gap2', 'gap3'],
         'and ALL of them are named, not the first', c2.unmapped())
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

def _band_all(frames, clock):
    lines = [clock.lines(i) for i in range(len(frames))]
    bh = ct.clock_band_height(lines, frames[0].width, frames[0].height)
    return [ct.add_clock_band(f, ln, bh) for f, ln in zip(frames, lines)], bh


def test_every_banded_frame_is_still_one_size():
    """The highest-value assertion here, mirroring test_431_placement_movie.

    The clock grows the frame now instead of drawing over it, so the invariant
    is no longer "the size does not change" but "every frame changes by the
    SAME amount" -- which is the property `save_movie` actually needs, and the
    one a per-frame band would break.
    """
    clock, _ = _full_clock()
    frames = [Image.new('RGB', (240, 180), (10, 10, 10)) for _ in range(12)]
    out, bh = _band_all(frames, clock)
    want(bh > 0, 'a band was reserved', bh)
    want(len({f.size for f in out}) == 1,
         'and every banded frame is one size', {f.size for f in out})
    want(out[0].size == (240, 180 + bh),
         'which is the original plus the band', (out[0].size, bh))


def test_the_clock_never_draws_on_the_board():
    """THE defect this band replaced, and it shipped in a published image.

    The clock used to be stamped bottom-left ON the frame, mirroring
    `_label`'s top-left corner. `_label` is one short line; the clock is four,
    so its black box covered a corner of the X-ray panel including copper --
    which a reviewer spotted immediately in the first still published for this
    PR. The board region must now come back byte-identical.
    """
    clock, _ = _full_clock()
    board = Image.new('RGB', (240, 180), (10, 10, 10))
    d = ImageDraw.Draw(board)
    # Paint the whole frame, bottom-left corner INCLUDED, so anything drawn
    # over the board shows up as a difference wherever it lands.
    for x in range(0, 240, 8):
        d.line([(x, 0), (x, 180)], fill=(40, 90, 140), width=3)
    keep = board.copy()

    out, bh = _band_all([board.copy()], clock)
    got = out[0]
    want(got.size[1] == 180 + bh, 'the frame grew by the band', got.size)
    want(ImageChops.difference(got.crop((0, 0, 240, 180)), keep).getbbox()
         is None,
         'and the ORIGINAL frame region is byte-identical -- the clock is '
         'beside the board, never on it')
    band = got.crop((0, 180, 240, 180 + bh))
    want(ImageChops.difference(band, Image.new('RGB', band.size,
                                               (0, 0, 0))).getbbox()
         is not None,
         'while the band itself carries the text')


def test_the_band_is_sized_for_the_WORST_frame_not_each_one():
    """A per-frame band is how the frames end up different sizes.

    `clock_band_height` is handed every frame's lines at once for exactly this
    reason: a step whose stage name wraps needs one more row than its
    neighbour, and sizing each frame to its own text is the mixed-size defect
    that `_write_mp4` fails on and the Pillow GIF fallback absorbs by silently
    resizing every later frame.
    """
    short = ['RUN CLOCK  +0:01:00 of 0:10:00']
    long = ['RUN CLOCK  +0:51:23 of 1:17:39',
            'basis  cmd_timing.jsonl - 153 wrapped commands, mapped by mtime '
            'inside a wrapped command window that keeps going for a while']
    bh = ct.clock_band_height([short, long], 200, 150)
    only_short = ct.clock_band_height([short], 200, 150)
    want(bh > only_short,
         'the movie-wide band is taller than the shortest frame needs',
         (bh, only_short))
    a = ct.add_clock_band(Image.new('RGB', (200, 150), (10, 10, 10)), short, bh)
    b = ct.add_clock_band(Image.new('RGB', (200, 150), (10, 10, 10)), long, bh)
    want(a.size == b.size,
         'so a short-text frame and a long-text frame come out the SAME size',
         (a.size, b.size))


def test_a_long_clock_wraps_instead_of_running_off_the_frame():
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
    tall = ct.clock_band_height([long_lines], 200, 150)
    short = ct.clock_band_height([['RUN CLOCK', 'basis  x']], 200, 150)
    want(tall > short + 4,
         'the long text reserves MORE rows than the same number of short '
         'lines, i.e. it wrapped instead of being clipped at the frame edge',
         (tall, short))
    # And every wrapped row must actually be drawn inside the band.
    img = ct.add_clock_band(Image.new('RGB', (200, 150), (10, 10, 10)),
                            long_lines, tall)
    band = img.crop((0, 150, 200, 150 + tall))
    b = ImageChops.difference(band, Image.new('RGB', band.size,
                                              (0, 0, 0))).getbbox()
    want(b is not None and b[2] <= 200 and b[3] <= tall,
         'and it stays inside the band, nothing lost off the right edge', b)


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

    # TWO DIFFERENT boards. Copying one board twice produces a chain with no
    # copper delta and therefore ONE frame -- the opening "input" snapshot,
    # which belongs to no step. This test then asserted `krt:stage` on that
    # frame, and it was green only because the opening frame used to BORROW the
    # first anchor's stage. A chain with real steps is what the assertion below
    # was always meant to be about.
    wd = os.path.join(tempfile.mkdtemp(), 'run')
    os.makedirs(wd)
    b1 = os.path.join(wd, 'step1.kicad_pcb')
    b2 = os.path.join(wd, 'step2.kicad_pcb')
    shutil.copy(os.path.join(ROOT, 'kicad_files',
                             'lvds_converter_dualclk.kicad_pcb'), b1)
    shutil.copy(os.path.join(ROOT, 'kicad_files',
                             'routed_output.kicad_pcb'), b2)
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
    want(len(frames) > 1,
         'the chain produced more than the opening snapshot -- without this '
         'the stage assertion below is about a frame no step owns',
         len(frames))
    text = Image.open(frames[-1]).text
    krt = {k: v for k, v in text.items() if k.startswith('krt:')}
    want(krt, 'the frames carry the krt: timing block -- WITHOUT ASKING, '
              'because a ledger sits beside the chain', sorted(text))
    want(krt.get('krt:ledger_rows') == '2',
         'read from that ledger', krt.get('krt:ledger_rows'))
    want('krt:elapsed_s' in krt and 'krt:stage' in krt,
         'with an elapsed and a stage', sorted(krt))
    first = {k: v for k, v in Image.open(frames[0]).text.items()
             if k.startswith('krt:')}
    want('krt:stage' not in first and first.get('krt:clock_basis') == 'run-start',
         'while the OPENING frame carries no stage and says run-start, '
         'because no wrapped command produced it', sorted(first))

    # And the OFF switch really switches it off.
    png2 = os.path.join(wd, 'frames_off')
    MM.make_movie([b1, b2], out=os.path.join(wd, 'm2.gif'), size=160,
                  quiet=True, png_dir=png2, timing='off')
    f2 = sorted(glob.glob(os.path.join(png2, '*.png')))
    off = {k for k in Image.open(f2[len(f2) // 2]).text if k.startswith('krt:')}
    want(not off, 'timing="off" writes no timing block at all', off)


def test_a_named_ledger_that_is_missing_is_refused_not_replaced():
    """`--timing-ledger /typo.jsonl` must NOT quietly use a different ledger.

    The auto-discovery fallback ran whenever the named path was not a file, so
    a mistyped `--timing-ledger` stamped the movie with whatever ledger
    happened to sit beside the chain -- another run's clock, drawn into every
    frame, with numbers that look entirely reasonable and no way for a viewer
    to tell. The test needs a REAL discoverable ledger present, or the silent
    fallback would have produced no clock either and passed for free.
    """
    import glob
    import json
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
    with open(os.path.join(wd, 'cmd_timing.jsonl'), 'w', encoding='utf-8') as f:
        for r in _rows([('P1-place', 990, 20, 0, 'step1.kicad_pcb'),
                        ('R1-route', 1190, 20, 0, 'step2.kicad_pcb')]):
            f.write(json.dumps(r) + '\n')

    # The control: auto-discovery finds that ledger and the clock is drawn.
    png_ok = os.path.join(wd, 'auto')
    MM.make_movie([b1, b2], out=os.path.join(wd, 'a.gif'), size=160,
                  quiet=True, png_dir=png_ok)
    fa = sorted(glob.glob(os.path.join(png_ok, '*.png')))
    got = {k for k in Image.open(fa[len(fa) // 2]).text if k.startswith('krt:')}
    want(got, 'CONTROL: a discoverable ledger IS found, so the silent '
              'fallback had something to fall back TO', sorted(got))

    missing = os.path.join(wd, 'not_here.jsonl')
    raised = None
    try:
        MM.make_movie([b1, b2], out=os.path.join(wd, 'b.gif'), size=160,
                      quiet=True, timing=missing)
    except FileNotFoundError as exc:
        raised = str(exc)
    want(raised is not None,
         'a named ledger that is not there is REFUSED, not replaced by the '
         'one auto-discovery would have found', raised)
    want(raised and missing in raised,
         'and the refusal names the path that was asked for', raised)

    # The CLI turns it into an argparse error rather than a traceback.
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_router', 'make_movie.py'), b1, b2,
         '-o', os.path.join(wd, 'c.gif'), '--size', '160', '--quiet',
         '--timing-ledger', missing],
        cwd=ROOT, capture_output=True, text=True)
    want(r.returncode == 2, 'the CLI exits 2', r.returncode)
    want('no such file' in r.stderr and 'Traceback' not in r.stderr,
         'with a stated reason and no traceback', r.stderr.strip()[-200:])


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
    test_every_banded_frame_is_still_one_size,
    test_the_clock_never_draws_on_the_board,
    test_the_band_is_sized_for_the_WORST_frame_not_each_one,
    test_a_long_clock_wraps_instead_of_running_off_the_frame,
    test_the_png_block_is_facts_and_carries_no_prediction,
    test_the_png_block_omits_what_it_cannot_know,
    test_make_movie_actually_draws_the_clock_when_a_ledger_is_beside_the_chain,
    test_a_movie_with_no_ledger_beside_it_carries_no_timing_block,
    test_clock_for_returns_none_without_a_ledger,
    test_the_opening_frame_does_not_borrow_the_first_steps_basis,
    test_a_named_ledger_that_is_missing_is_refused_not_replaced,
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
