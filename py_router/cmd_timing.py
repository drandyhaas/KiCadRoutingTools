#!/usr/bin/env python3
"""Read a run's ``cmd_timing.jsonl`` and say what the run spent its time on (#887).

``tests/stress/tee_cmd.py`` wraps every tool a recorded run invokes and appends
one JSON row per command to ``<workdir>/cmd_timing.jsonl``. Until now that file
was consumed by a WATCH SUBAGENT doing arithmetic by hand at the end of a run --
a mandate that lived only inside a work dir (``wk/run24/esp_prog/watch/timing.md``)
and died with it. The step table, the stage subtotals, the totals and the three
longest steps are a deterministic script's job; this is that script, and the
movie overlay reads the same numbers, so there is ONE reader and two consumers.

    python3 -X utf8 py_router/cmd_timing.py <WORKDIR|LEDGER.jsonl> [--json]

**The clock here is DESCRIPTIVE, inherited from the writer.** ``tee_cmd`` says it
plainly: "no code path in this repo compares it to a limit. There is no budget,
no cap and no timeout." So this module declares no threshold, takes no deadline
argument, and never exits non-zero because a run was slow. It reports.

**It never predicts.** Elapsed and the run total are both recorded facts, so any
"remaining" figure a consumer derives from them is subtraction, not a forecast.
There is no rate, no projection and no ETA anywhere in here, and a test walks
this module's AST to keep it that way.

Why it lives in py_router/: ``make_movie.py`` puts only its own directory on
``sys.path`` and is imported in-process by the GUI recorder, ``run_plan.py``,
``place_route_loop.py`` and ``render_run.py``, so a sibling import costs nothing
and a path bootstrap would cost every one of them. ``py_tools/_path.py`` already
inserts ``../py_router``, so ``make_film.py`` gets this for free. Shipped code
must not import from ``tests/``, which is why the reader is not beside its
writer -- an asymmetry that is correct, since only the harness WRITES the ledger
while the movie, the film, the loop and the audit pass all READ it.

``import cmd_timing`` touches no third-party module at all -- not numpy, not
Pillow -- so the report CLI runs on a machine that has none. A test asserts it
by inspecting ``sys.modules`` after a bare import in a fresh interpreter. Any
future drawing helper here must keep its PIL import inside the function, the
way ``py_tools/make_film._badge`` does.
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['combined'], 'kind': 'instrument'}

import argparse
import collections
import json
import math
import os
import sys

LEDGER_NAME = 'cmd_timing.jsonl'


class LedgerError(ValueError):
    """A ledger line that is not a JSONL row, named by file and line number.

    A ValueError, NOT a SystemExit, and the difference is not cosmetic. This is
    a LIBRARY function with two consumers: the report CLI, where exiting is
    right, and the movie, where a clock is decoration that "may never take the
    movie down" -- its own words at make_movie.py. SystemExit derives from
    BaseException, so it walked straight through every `except Exception` guard
    meant to contain it:

      * place_route_loop's end-of-run movie block would have killed the whole
        PLACEMENT RUN, after the routing was done;
      * the GUI recorder's worker thread would have died with no log line;
      * make_movie's CLI, which catches only FileNotFoundError, would have died
        with a bare message.

    And the triggering input is ordinary, not exotic: tee_cmd APPENDS a row per
    command, so any run killed mid-write -- run 22 died to a token limit exactly
    that way -- leaves a truncated last line. `main()` turns this into the exit
    it always was.
    """

OTHER = 'other'

#: Bucket rules, first match wins, encoding the footnotes at
#: wk/run24/esp_prog/watch/timing.md:33-40 -- which were the only place they
#: existed.
#:
#: **The ORDER of these rules is INERT, and the first version of this file was
#: wrong to say otherwise.** No prefix here is a prefix of another (pinned by
#: `test_no_rule_prefix_shadows_another`), so at most one rule can match any
#: label and first-match-wins never arbitrates anything -- reversing the whole
#: tuple changes no answer. The two footnote outcomes are real, but they come
#: from the PREFIX TEST, not from precedence:
#:
#:   * `fence-audit-start` buckets under close-out because a lowercase `f`
#:     starts no letter rule -- not because `fence` is tested early;
#:   * `Pclose-q-render` buckets under `P*` because `close` is not a prefix OF
#:     `Pclose-q-render` (`P` is) -- these test the head of the LABEL, not
#:     whether the label contains the word.
#:
#: The distinction is what the next person needs: adding a rule whose prefix
#: SHADOWS an existing one (say `'Pc'`) would make order suddenly decide the
#: answer, and the guard named above is what catches that.
#:
#: Matching is case-SENSITIVE. Nothing in the published run-24 report forces
#: that choice, so it is a decision: case-insensitivity would swallow `route4`
#: -- the RUNBOOK's own example label -- into `R*`, and a lowercase tool name is
#: not a stage. What happens when a whole run ignores this scheme is
#: `unmatched_note()`, not silence.
STAGE_RULES = (
    ('staging',   ('staging',)),
    ('close-out', ('fence', 'close')),
    ('P*',        ('P',)),
    ('L*',        ('L',)),
    ('R*',        ('R',)),
    ('V*',        ('V',)),
)

STAGE_TITLES = {
    'staging': 'staging',
    'P*': 'P* placement',
    'L*': 'L* driver',
    'R*': 'R* route',
    'V*': 'V* verification',
    'close-out': 'close-out',
    OTHER: 'other',
}

#: Report order. `other` is ALWAYS present, even at zero: the run-24 footnote
#: claims "the other bucket is empty -- every label matched a named prefix", and
#: that is only a checkable claim if the bucket is printed. A bucketer that
#: silently drops unmatched labels makes the subtotals disagree with the tool
#: total with nothing saying why.
STAGE_ORDER = ('staging', 'P*', 'L*', 'R*', 'V*', 'close-out', OTHER)


# --------------------------------------------------------------------------
# reading
# --------------------------------------------------------------------------

def find_ledger(start, max_up=3):
    """Absolute path to the ``cmd_timing.jsonl`` governing ``start``, or None.

    ``start`` may be the ledger itself, a directory, or a board inside a work
    dir. Walks that directory then up to ``max_up`` parents, which is what lets
    BOTH ``make_movie.py <workdir>`` and the board-sequence form
    (``make_movie.py .../frozen.kicad_pcb .../r1_pour.kicad_pcb ...``, which is
    literally what run 24's own R7-movie step ran) find the same file.

    A ``.jsonl`` file given directly is taken AS the ledger whatever it is
    called. Requiring the canonical basename here rejected every ledger that is
    not in a work dir -- the test fixtures, a copy saved for a bug report, a
    second ledger kept beside the first -- and the failure looked like "no
    ledger" rather than "wrong name", which is the least useful of the two.
    """
    if not start:
        return None
    p = os.path.abspath(start)
    if os.path.isfile(p) and p.lower().endswith('.jsonl'):
        return p
    d = p if os.path.isdir(p) else os.path.dirname(p)
    for _ in range(max(0, int(max_up or 0)) + 1):
        cand = os.path.join(d, LEDGER_NAME)
        if os.path.isfile(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def _num(v):
    """A FINITE float, or None -- a row with a missing/garbage clock is a row.

    Non-finite is rejected as garbage, not passed through. ``json.loads``
    accepts bare ``NaN`` and ``Infinity`` by default and ``float()`` is happy
    with both, so a single such value used to reach ``fmt_hms`` and die there
    with ``ValueError: cannot convert float NaN to integer`` -- a whole report
    lost to one bad cell, which is exactly what this function exists to prevent.
    """
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def load_rows(path):
    """Every row in the ledger, SORTED BY ``t_start``. ``[]`` when absent.

    Not a reuse of ``tests/stress/predictor_study.read_jsonl``: that one lives
    under tests/, does not sort, does not coerce, and reports a parse failure as
    a bare JSONDecodeError with no line number. This takes its shape (skip
    blanks, [] when missing) and its sibling ``load_rows``' line-numbered error.

    Sorting matters because the file is append-only and nothing guarantees
    order; the totals below use min()/max() for the same reason.
    """
    rows = []
    if not path or not os.path.isfile(path):
        return rows
    # utf-8-SIG, not utf-8: this is a Windows-primary repo where PowerShell's
    # `>` and Out-File default to UTF-8 WITH BOM, so a ledger that has been
    # copied or filtered through a shell arrives with one. Under plain utf-8
    # the BOM made line 1 unparseable and the whole ledger read as a refusal.
    # The -sig codec is a no-op when there is no BOM.
    with open(path, encoding='utf-8-sig') as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError as e:
                raise LedgerError('%s:%d: not a JSONL row: %s' % (path, n, e))
            if not isinstance(row, dict):
                raise LedgerError('%s:%d: not a JSONL row: not an object'
                                  % (path, n))
            row = dict(row)
            row.setdefault('label', 'unlabelled')     # tee_cmd's own default
            for k in ('t_start', 't_end', 'wall_s'):
                row[k] = _num(row.get(k))
            rows.append(row)
    rows.sort(key=lambda r: (r['t_start'] is None, r['t_start'] or 0.0))
    return rows


# --------------------------------------------------------------------------
# bucketing
# --------------------------------------------------------------------------

def stage_of(label):
    """Which stage bucket a tee_cmd label belongs to. See STAGE_RULES."""
    s = label if isinstance(label, str) else str(label or '')
    for stage, prefixes in STAGE_RULES:
        for pre in prefixes:
            if s.startswith(pre):
                return stage
    return OTHER


def bucket_rows(rows):
    """{stage: [row, ...]} in STAGE_ORDER, empty buckets kept.

    Nothing here groups by label, anywhere. The run-24 footnote's "labels that
    repeat are counted once per entry" is implemented as that ABSENCE, so please
    do not "fix" this into a group-by: three `R5-prune` rows are three steps.

    `exit` is carried and never judged. Run 24's own start anchor
    `staging-assembly0` exited 4 and is row 1 of the published table.
    """
    out = collections.OrderedDict((s, []) for s in STAGE_ORDER)
    for r in rows:
        out[stage_of(r.get('label'))].append(r)
    return out


def subtotals(rows):
    """[(stage, n_steps, wall_s), ...] in report order, `other` always present.

    The sum of the wall_s column equals ``totals(rows).tool_s`` by construction;
    a test asserts it, which is what makes a silently-dropped label impossible.
    """
    buckets = bucket_rows(rows)
    return [(stage, len(rs), round(sum((r.get('wall_s') or 0.0) for r in rs), 3))
            for stage, rs in buckets.items()]


#: ``last_iso_start`` is NOT an end time, and is named so nobody reads it as one.
#: tee_cmd records ``iso_start`` and nothing else, so the only honest ISO string
#: for the end of a run is the START of the row that ended last. Calling that
#: field `iso_end` (as the first version did) exported a value 400 s adrift of
#: `t1` on a ledger with an overlapping row, under a name that promised the
#: opposite -- and `report_data` hands these fields to other consumers.
Totals = collections.namedtuple(
    'Totals', 'n tool_s t0 t1 run_s outside_s iso_start last_iso_start '
              'first_label last_label')


def totals(rows):
    """Counts and spans. Never raises; an empty ledger gives n=0 and None times.

    The span is ``max(t_end) - min(t_start)`` across ALL rows, not the first and
    last row's. On run 24 the two coincide (tee_cmd runs serially and blocks, so
    the rows are disjoint and already ordered), but the file is append-only and
    max() costs nothing.

    ``outside_s`` is reported as-is and never clamped, even if negative: a
    negative value means two wrapped commands overlapped, which is information,
    not an error to hide.
    """
    n = len(rows)
    tool_s = round(sum((r.get('wall_s') or 0.0) for r in rows), 3)
    starts = [r['t_start'] for r in rows if r.get('t_start') is not None]
    ends = [r['t_end'] for r in rows if r.get('t_end') is not None]
    if not starts or not ends:
        return Totals(n, tool_s, None, None, None, None, None, None, None, None)
    t0, t1 = min(starts), max(ends)
    first = min((r for r in rows if r.get('t_start') is not None),
                key=lambda r: r['t_start'])
    last = max((r for r in rows if r.get('t_end') is not None),
               key=lambda r: r['t_end'])
    run_s = round(t1 - t0, 3)
    return Totals(n, tool_s, t0, t1, run_s, round(run_s - tool_s, 3),
                  first.get('iso_start'), last.get('iso_start'),
                  first.get('label'), last.get('label'))


def longest(rows, n=3):
    """The n rows with the largest wall_s, ties broken by t_start (run order)."""
    ranked = sorted(rows,
                    key=lambda r: (-(r.get('wall_s') or 0.0),
                                   r.get('t_start') if r.get('t_start') is not None
                                   else float('inf')))
    return ranked[:max(0, int(n))]


def utc_iso(epoch):
    """``2026-08-20T10:03:32Z`` for an epoch instant, or '' for None.

    UTC, and the `Z` is part of the value. The ledger's own `iso_start` is
    LOCAL time with no offset (``time.strftime(..., time.localtime(t0))`` at
    tee_cmd.py:179), which is unambiguous only on the machine that wrote it --
    so it is echoed verbatim in the report, where the reader is that machine's
    owner, and never used for a frame, which travels. `t_start` is epoch
    seconds, so this is a total function of a recorded fact.
    """
    if epoch is None:
        return ''
    import datetime
    return (datetime.datetime.fromtimestamp(float(epoch),
                                            datetime.timezone.utc)
            .strftime('%Y-%m-%dT%H:%M:%SZ'))


def fmt_hms(seconds):
    """``H:MM:SS`` for a duration, rounded to the NEAREST second.

    ``floor(s + 0.5)``, deliberately, and the two alternatives fail differently:

      * truncation is wrong ON THIS DATA -- it misses four of the nine H:MM:SS
        values in the hand-written run-24 report (P* 0:00:54 not 0:00:55, L*
        0:00:06 not 0:00:07, close-out 0:00:23 not 0:00:24, and the run span
        1:17:38 not 1:17:39);
      * Python's ``round`` is wrong IN PRINCIPLE but not on this data: it
        reproduces all nine, because none of them lands on an exact .5. It is
        banker's rounding, so ``round(0.5) == 0`` and ``round(1.5) == 2``, and
        the day a bucket does land on a half-second it would disagree with the
        report for a reason nobody would look for. Stated separately because an
        earlier version of this comment said "both alternatives are wrong"
        against the nine values, and only one of them is.

    Not shared with ``kicad_routing_plugin/placement_gui._fmt_elapsed``: that one
    lives in the wx plugin package (unimportable from here without dragging wx
    onto sys.path) and spells the same duration "1h 17m 39s".
    """
    if seconds is None:
        return '-'
    total = int(math.floor(float(seconds) + 0.5))
    sign = '-' if total < 0 else ''
    total = abs(total)
    h, rem = divmod(total, 3600)
    m, s = divmod(rem, 60)
    return '%s%d:%02d:%02d' % (sign, h, m, s)


# --------------------------------------------------------------------------
# the report the watch subagent used to write by hand
# --------------------------------------------------------------------------

def unmatched_note(rows):
    """A warning when labels fall outside the scheme, or '' when none do.

    The mandate at timing.md:17-22 defined close-out as a CATCH-ALL
    ("everything after the last route step / remaining labels"); these rules
    make it a `fence`/`close` prefix and send everything unrecognised to
    `other`. On run 24 the two agree, because `other` came out empty -- but they
    are not the same rule, and a run that labels its steps differently is not
    exotic: the RUNBOOK's own worked example uses `route4`, which lands in
    `other`, and a lowercase convention (`p0-driver`, `r3-route`) would put
    100% of a run there.

    Printing an empty `other` row is not enough to notice that. This says so in
    words, and names the labels, so a misbucketed run is a visible fact instead
    of a zero someone has to spot.
    """
    odd = [r.get('label') for r in rows if stage_of(r.get('label')) == OTHER]
    if not odd:
        return ''
    uniq = sorted(set(odd))
    shown = ', '.join('`%s`' % u for u in uniq[:6])
    more = '' if len(uniq) <= 6 else ' (+%d more)' % (len(uniq) - 6)
    pct = 100.0 * len(odd) / max(1, len(rows))
    return ('**%d of %d steps (%.0f%%) matched no stage prefix** and are '
            'counted under `other`: %s%s. The stage subtotals describe the '
            'rest. If this run labels its steps by another convention, the '
            'buckets below are not the ones its author had in mind.'
            % (len(odd), len(rows), pct, shown, more))


def _rules_note():
    """The bucketing note, generated FROM STAGE_RULES so prose cannot drift."""
    parts = []
    for stage, prefixes in STAGE_RULES:
        parts.append('%s -> %s' % ('/'.join(p + '*' for p in prefixes), stage))
    return ('Bucketing by label PREFIX, case-sensitive: '
            + '; '.join(parts) + '; anything else -> ' + OTHER
            + '. These prefixes are pairwise incomparable, so their order does '
              'not arbitrate anything. `fence-audit-start` buckets under '
              'close-out because a lowercase `f` starts no letter rule (it ran '
              'at run START, not at the end); `Pclose-*` counts under P* '
              'because `close` is not a prefix OF `Pclose-*`. Repeated labels '
              'are counted once per entry.')


def report_markdown(rows, ledger_path=None):
    """The step table / subtotals / totals / three-longest report, as markdown."""
    tot = totals(rows)
    out = []
    where = (' -- `%s`' % ledger_path) if ledger_path else ''
    out.append('## Timing audit%s' % where)
    out.append('')
    out.append('%d wrapped command%s.' % (tot.n, '' if tot.n == 1 else 's'))
    out.append('')
    out.append(_rules_note())
    out.append('')
    odd = unmatched_note(rows)
    if odd:
        out.append(odd)
        out.append('')
    out.append('This clock is DESCRIPTIVE: `wall_s` is a record, not a limit. '
               'There is no budget, no cap and no timeout, and nothing here '
               'fails a run for being slow.')
    out.append('')

    out.append('### 1. Step table (run order)')
    out.append('')
    out.append('| # | label | started | wall s | exit |')
    out.append('|---:|---|---|---:|---:|')
    for i, r in enumerate(rows, 1):
        # `started` echoes iso_start VERBATIM and never re-derives it from
        # t_start: tee_cmd writes LOCAL time with no zone offset, so a reader in
        # another zone would silently print a different clock than the run's own
        # record.
        out.append('| %d | %s | %s | %s | %s |'
                   % (i, r.get('label'), r.get('iso_start') or '-',
                      ('%.3f' % r['wall_s']) if r.get('wall_s') is not None else '-',
                      r.get('exit', '-')))
    out.append('')

    out.append('### 2. Stage subtotals')
    out.append('')
    out.append('| stage | steps | wall s | H:MM:SS |')
    out.append('|---|---:|---:|---:|')
    for stage, n, wall in subtotals(rows):
        # ROUND ONCE, then use that value for both columns. Formatting the raw
        # value at %.1f while handing fmt_hms the unrounded one let the two
        # disagree: a 26.46 s bucket printed `| 26.5 | 0:00:26 |`.
        shown = round(wall, 1)
        out.append('| %s | %d | %.1f | %s |'
                   % (STAGE_TITLES[stage], n, shown, fmt_hms(shown)))
    out.append('')

    out.append('### 3. Totals')
    out.append('')
    out.append('- Entries counted: %d' % tot.n)
    out.append('- Tool time (sum of wall_s): %.1f s = **%s**'
               % (tot.tool_s, fmt_hms(tot.tool_s)))
    if tot.run_s is None:
        out.append('- Total run time: not recoverable (no usable t_start/t_end)')
        out.append('- **Time outside the tools**: not recoverable')
    else:
        # The last row's t_end has no recorded ISO -- tee_cmd writes iso_start
        # only -- so this names the row and its START rather than printing a
        # clock nobody wrote down. The hand-written run-24 report re-derived
        # that timestamp; doing so on a machine in another zone would print a
        # different clock than the run's own record, which is the whole reason
        # `started` is echoed verbatim everywhere else in this file.
        out.append('- Total run time (first t_start %s -> last t_end, from row '
                   '`%s` started %s): %.1f s = **%s**'
                   % (tot.iso_start or '-', tot.last_label,
                      tot.last_iso_start or '-', tot.run_s, fmt_hms(tot.run_s)))
        out.append('- **Time outside the tools** (agent/orchestration): '
                   '%.1f s = **%s**' % (tot.outside_s, fmt_hms(tot.outside_s)))
    out.append('')

    top = longest(rows, 3)
    out.append('### 4. Three longest steps')
    out.append('')
    if not top:
        out.append('- (none)')
    for r in top:
        out.append('- **%s** -- %.1f s (started %s, exit %s)'
                   % (r.get('label'), r.get('wall_s') or 0.0,
                      r.get('iso_start') or '-', r.get('exit', '-')))
    out.append('')
    if tot.run_s is not None:
        out.append('BOTTOM LINE: total run %s, tool time %s, outside-tools %s.'
                   % (fmt_hms(tot.run_s), fmt_hms(tot.tool_s),
                      fmt_hms(tot.outside_s)))
    return '\n'.join(out) + '\n'


def report_data(rows, ledger_path=None):
    """The same report as plain data, for --json and for any other consumer."""
    tot = totals(rows)
    return {
        'ledger': ledger_path,
        'totals': tot._asdict(),
        'subtotals': [{'stage': s, 'title': STAGE_TITLES[s], 'steps': n,
                       'wall_s': w} for s, n, w in subtotals(rows)],
        'longest': [{'label': r.get('label'), 'wall_s': r.get('wall_s'),
                     'iso_start': r.get('iso_start'), 'exit': r.get('exit')}
                    for r in longest(rows, 3)],
        'steps': [{'label': r.get('label'), 'iso_start': r.get('iso_start'),
                   'wall_s': r.get('wall_s'), 'exit': r.get('exit')}
                  for r in rows],
    }


# --------------------------------------------------------------------------
# the run clock: which instant a movie frame is showing
# --------------------------------------------------------------------------

#: ``wall_s`` is what the command that produced this board COST -- the other
#: reading of "how long did this take". `t` says where in the run the beat sits;
#: `wall_s` says how long its own step ran, and only the matched row knows that.
Anchor = collections.namedtuple(
    'Anchor', 'label board first last t stage basis wall_s')


def _basenames(row):
    """Every argv entry's basename, backslashes normalised.

    The ledger carries the argv as the OS gave it, so on Windows a board
    appears as `work\\r1.kicad_pcb`. Comparing basenames is what makes the
    match a match; comparing substrings would let `r3.kicad_pcb` find
    `xr3.kicad_pcb`.
    """
    out = []
    for a in (row.get('argv') or ()):
        try:
            out.append(os.path.basename(str(a).replace('\\', '/')))
        except Exception:                                       # noqa: BLE001
            continue
    return out


def anchor_steps(marks, rows, mtimes=None):
    """One ``Anchor`` per mark: when that step's board came into existence.

    ``t`` is a run-clock epoch instant or None, and ``basis`` NAMES HOW IT WAS
    FOUND so a frame can print it rather than implying a precision the match
    does not have.

    The primary witness is the board's MTIME landing inside a row's
    ``[t_start, t_end]``. tee_cmd writes ``t0 = time.time()`` and a file's mtime
    is the same epoch clock, and it runs commands serially and blocks on each --
    so at most one row can contain an mtime, and the resolution is unique by
    construction. Measured on run 24: zero overlapping rows in 153, and 16 of
    17 chain boards resolved to exactly one row (r1_pour -> R1-pour,
    r3_route -> R3-route, r7_lc -> R7-layercosts). The 17th is the seed board,
    whose mtime precedes the run by 101 s -- the right answer, not a miss.

    What mtime identifies is WHEN THE CONTENT CAME INTO EXISTENCE, which is
    usually but not always the command that wrote the file: `copy_board.py` uses
    `shutil.copy2`, which preserves mtime, so a copied board resolves to the row
    that produced its SOURCE. Two of run 24's sixteen are copies. That is the
    right instant for a movie -- the frame shows that content -- but it is not
    the same claim as "the command that wrote this path".

    argv matching is the FALLBACK, not the primary, because mtime is destroyed
    by copying a work dir and by `make_film --from-ledger`, which materialises
    boards out of a content-addressed store.

    It is a fallback rather than the rule because it is wrong FAR more often
    than it looks. Graded against the mtime answer over run 24's 17 chain
    boards: raw first-mention picks the wrong command 10 times, and the rule
    actually implemented below -- prefer an exit-0 mention -- still picks wrong
    9 times, five of them by more than a minute (worst: `routed`, off by 470 s).
    The exit-0 preference does fix `r4`, landing 0.1 s from the truth.

    (An earlier version of this docstring said "wrong three times" and named
    r4, routed and r5_prune. Each anecdote is accurate, but three was the number
    of examples looked at, not the number of failures -- r1, r2 and r3 are three
    MORE instances of the same dry-run category the text presented as happening
    once. Understating a fallback's error rate by 3x is how it stops being
    treated as a fallback.)

    ``mtimes`` overrides ``os.path.getmtime`` (tests, and any caller that knows
    better).
    """
    rows = [r for r in (rows or [])
            if r.get('t_start') is not None and r.get('t_end') is not None]
    out = []
    for m in (marks or []):
        label, board, first, last = m[0], m[1], m[2], m[3]
        t, stage, basis, wall = None, None, 'none', None
        mt = None
        if mtimes and board in mtimes:
            mt = mtimes[board]
        else:
            try:
                mt = os.path.getmtime(board)
            except OSError:
                mt = None
        if mt is not None:
            inside = [r for r in rows if r['t_start'] <= mt <= r['t_end']]
            if inside:
                t, stage, basis = mt, inside[0].get('label'), 'mtime'
                wall = inside[0].get('wall_s')
        if t is None and mt is not None and rows and mt < min(
                r['t_start'] for r in rows):
            # A file OLDER than the run's first wrapped command was not written
            # by any of them, so this is checked BEFORE argv. Measured on run
            # 24: board.kicad_pcb predates the run by 101 s, and argv-first gave
            # it `L1-driver` at t+107 -- the first successful command that
            # merely READ it, which is a worse answer than "it was already
            # there". Clamped to t0, because the film opens at the run's start.
            t, basis = min(r['t_start'] for r in rows), 'pre-run'
        if t is None:
            base = os.path.basename(str(board).replace('\\', '/'))
            hits = [r for r in rows if base in _basenames(r)]
            clean = [r for r in hits if r.get('exit') in (0, '0')]
            pick = (clean or hits)
            if pick:
                r0 = min(pick, key=lambda r: r['t_start'])
                t, stage = r0['t_end'], r0.get('label')
                wall = r0.get('wall_s')
                basis = 'argv' if clean else 'argv?'
        if t is None and mt is not None and rows:
            # No `elif mt < t0: pre-run` here. That branch used to exist and
            # became UNREACHABLE when the pre-run test moved above the argv
            # fallback: it fires on exactly `mt < t0`, so nothing could ever
            # reach a second copy. The mutation battery is what found it --
            # deleting the live branch left the dead one answering, so the row
            # survived and pointed straight at the duplicate.
            t0 = min(r['t_start'] for r in rows)
            t1 = max(r['t_end'] for r in rows)
            if t0 <= mt <= t1:
                t, basis = mt, 'mtime-loose'
        out.append(Anchor(label, board, first, last, t, stage, basis, wall))

    # Monotone clamp in MARK order. The movie's step order is the chain and is
    # authoritative, so an earlier-looking instant is a mapping error, not time
    # running backwards. A clamped anchor's basis gains '+clamped', because a
    # corrected number must never be presented as a measured one.
    prev = None
    fixed = []
    for a in out:
        if a.t is not None and prev is not None and a.t < prev:
            a = a._replace(t=prev, basis=a.basis + '+clamped')
        if a.t is not None:
            prev = a.t
        fixed.append(a)
    return fixed


Reading = collections.namedtuple(
    'Reading', 'elapsed_s stage basis instant interpolated')


class RunClock(object):
    """Frame index -> where that frame sits on the RUN's wall clock.

    **It counts UP, and there is deliberately no countdown.**

    A countdown was possible and would have been exact -- the movie is built
    after the run, so `t1 - instant` is the subtraction of two recorded facts,
    not a forecast. It was implemented, measured against run 24, and then
    removed, because exact is not the same as legible: a countdown READS as
    "time left in this video", and it means "time that remained in the run".
    A 25-frame GIF that finishes in four seconds while showing
    "remaining 0:15:57" is inviting exactly that misreading. `+1:01:42 of
    1:17:39` carries the same information with no way to misread it, and the
    viewer can subtract if they want the other number.

    Taking it out also deleted the machinery it needed: a `covered` predicate
    over whether the ledger spanned the film, its shortfall message, and an
    exact-or-absent branch in both the overlay and the metadata. None of that
    was wrong; all of it existed only to make one redundant line safe.

    The basis is the run clock and nothing else. Tool time is not it: in run 24
    the wrapped commands account for 253.3 s of a 4658.7 s run -- 5.4% -- so a
    clock driven by tool time would sit near zero for over an hour.

    Within a step's frame span the instant is INTERPOLATED between two measured
    endpoints (this step's and the next resolved one's) and never projected past
    the last. Copper reveal is not uniform in time, so an interpolated figure is
    a smoothing rather than a measurement, and the overlay says so.
    """

    def __init__(self, anchors, tot, n_frames):
        self.anchors = list(anchors or [])
        self.tot = tot
        self.n = int(n_frames)
        self.resolved = [a for a in self.anchors if a.t is not None]

    def unmapped(self):
        """The beats with no instant, or [] when every one resolved.

        What `covered` used to gate is now just disclosure: the frames that
        could not be placed say so individually, and this names them for a
        caller that wants a one-line summary.
        """
        return [a.label for a in self.anchors if a.t is None]

    def at(self, i):
        """The ``Reading`` for frame ``i``. ``instant`` is an ABSOLUTE epoch."""
        t = self.tot
        if not self.anchors or t is None or t.t0 is None:
            return Reading(None, None, 'no ledger', None, False)
        k = None
        for j, a in enumerate(self.anchors):
            if a.first <= i < a.last:
                k = j
                break
        if k is None:
            # Before the first mark: `build_boards` opens with its own
            # `m.snapshot("input")` BEFORE the step loop, so frame 0 is always
            # here. Its instant is the run's start, and it borrowed the first
            # anchor's stage and basis until #887's pre-push review -- which
            # made frame 0 announce, say, `stage R1-pour / mapped by mtime`
            # about a board R1-pour never touched and mtime never witnessed.
            # A number is allowed to be approximate here; a BASIS is a claim
            # about where the number came from, and that one was false. Its own
            # name, and no stage, because no wrapped command produced this
            # frame.
            if i < self.anchors[0].first:
                return Reading(0.0, None, 'run-start', t.t0, False)
            k = len(self.anchors) - 1
        a = self.anchors[k]
        if a.t is None:
            return Reading(None, a.stage, 'none', None, False)
        nxt = next((b for b in self.anchors[k + 1:] if b.t is not None), None)
        interp = False
        inst = a.t
        if nxt is not None and nxt.first > a.first:
            frac = (i - a.first) / float(max(1, nxt.first - a.first))
            frac = min(1.0, max(0.0, frac))
            inst = a.t + (nxt.t - a.t) * frac
            interp = frac not in (0.0,)
        return Reading(max(0.0, inst - t.t0), a.stage, a.basis, inst, interp)

    def lines(self, i):
        """The overlay text for frame ``i``. Three lines, four when licensed.

        The first token is literally RUN CLOCK -- never ETA, never a bare "time
        left" -- because the frame must say what the number is before it says
        the number. It counts UP, `+elapsed of total`, and the `at` line gives
        the absolute UTC instant, so a frame lifted out of the movie is still
        placeable in time. There is no countdown; see the class docstring for
        why an exact one was removed rather than kept.
        """
        r = self.at(i)
        t = self.tot
        if r.elapsed_s is None:
            out = ['RUN CLOCK  --']
            if r.basis == 'none':
                out.append('stage  not in the ledger')
                out.append('basis  cmd_timing.jsonl - this beat has no wrapped '
                           'command')
            else:
                out.append('basis  no cmd_timing.jsonl beside this chain')
            return out
        total = fmt_hms(t.run_s) if t and t.run_s is not None else '--'
        out = ['RUN CLOCK  +%s%s of %s'
               % (fmt_hms(r.elapsed_s), ' ~' if r.interpolated else '', total)]
        # A `pre-run` or `run-start` beat has no stage BECAUSE no wrapped
        # command produced it -- it is the board the run started from. Saying
        # "unlabelled" there reads as a defect in the ledger rather than as the
        # fact it is. (The two differ in what is known: `pre-run` is a board
        # whose mtime PREDATES the run, `run-start` is the opening snapshot,
        # which has no board of its own at all.)
        stage = r.stage or ('the board the run started from'
                            if (r.basis or '').startswith(('pre-run',
                                                           'run-start'))
                            else 'unlabelled')
        out.append('stage  %s' % stage)
        how = 'interpolated within %s' % stage if r.interpolated else \
            'mapped by %s' % (r.basis or 'nothing')
        out.append('basis  cmd_timing.jsonl - %d wrapped commands, %s'
                   % (t.n if t else 0, how))
        # UTC, from the epoch instant. NOT the ledger's `iso_start`, which
        # tee_cmd writes as LOCAL time with no offset -- unambiguous only on
        # the machine that produced it. `t_start` is epoch seconds, so UTC is a
        # total function of it and means the same thing everywhere the movie is
        # watched. The `Z` is part of the value, not decoration.
        if r.instant is not None:
            out.append('at  %s' % utc_iso(r.instant))
        return out

    def meta(self, i):
        """The PNG text block for frame ``i``. Every value a recorded fact.

        **`krt:utc` is the one to read.** An absolute UTC instant makes a frame
        self-describing: it needs no ledger, no run start and no knowledge of
        the machine that produced it to be placed in time, so the timeline
        survives outside the movie -- which is the whole reason the block exists.
        `krt:t_epoch` is the same instant unrounded, for arithmetic.

        Deliberately NOT here: the ledger's own `iso_start`, which is local time
        with no offset and therefore ambiguous the moment the PNG leaves the
        machine; `krt:eta` and `krt:progress`, because a prediction and a
        percentage both invite being read as forecasts; and `krt:remaining_s`,
        which was exact but read as "time left in this video" -- see the class
        docstring.
        """
        r = self.at(i)
        t = self.tot
        m = {
            'krt:frame': i,
            'krt:frames': self.n,
            'krt:clock_basis': r.basis or 'none',
            'krt:ledger_rows': t.n if t else 0,
        }
        if r.stage:
            m['krt:stage'] = r.stage
        if r.instant is not None:
            m['krt:utc'] = utc_iso(r.instant)
            m['krt:t_epoch'] = round(r.instant, 3)
        if r.elapsed_s is not None:
            m['krt:elapsed_s'] = round(r.elapsed_s, 1)
            m['krt:elapsed_hms'] = fmt_hms(r.elapsed_s)
        if t and t.run_s is not None:
            m['krt:run_total_s'] = round(t.run_s, 1)
            m['krt:run_total_hms'] = fmt_hms(t.run_s)
            m['krt:run_started_utc'] = utc_iso(t.t0)
            m['krt:tool_s'] = round(t.tool_s, 1)
            m['krt:outside_s'] = round(t.outside_s, 1)
        for k, a in enumerate(self.anchors):
            if a.first <= i < a.last:
                m['krt:step'] = a.label
                # What that step COST, which is the other reading of "how long
                # did this take": the elapsed figure says where in the run this
                # frame sits, and this says how long its own command ran.
                if a.wall_s is not None:
                    m['krt:step_wall_s'] = round(a.wall_s, 3)
                break
        return m


def clock_for(marks, ledger_path, n_frames, mtimes=None):
    """A ``RunClock`` for a movie, or None when there is no ledger to read."""
    if not ledger_path or not marks:
        return None
    rows = load_rows(ledger_path)
    if not rows:
        return None
    return RunClock(anchor_steps(marks, rows, mtimes=mtimes),
                    totals(rows), n_frames)


#: Padding inside the clock band, in pixels.
_CLOCK_PAD = 6


def _clock_font(frame_h):
    from route_render import load_font
    return load_font(max(11, frame_h // 55))


def _wrap_clock(lines, avail, measure):
    """``lines`` broken on spaces to fit ``avail`` px. Wraps, never clips.

    A one-line clock overflowed a 700 px frame the first time it was drawn, and
    PIL clips at the edge in silence -- the same trap ``_label`` documents,
    where a strip ending at a plausible-looking field reads as the whole story.
    """
    out = []
    for ln in lines or ():
        cur = ''
        for word in str(ln).split(' '):
            cand = (cur + ' ' + word) if cur else word
            if cur and measure(cand) > avail:
                out.append(cur)
                cur = word
            else:
                cur = cand
        out.append(cur)
    return out


def clock_band_height(all_lines, width, frame_h):
    """The band height for a WHOLE movie: one number, from every frame's text.

    It is computed across all frames on purpose. Frames carry different numbers
    of wrapped lines -- a step whose stage name wraps has one more than its
    neighbour -- so a per-frame height would make the frames different sizes,
    which is the one thing `save_movie` cannot take: `_write_mp4` fails and the
    Pillow GIF fallback silently RESIZES every later frame to the first.
    """
    from PIL import Image, ImageDraw

    if not all_lines:
        return 0
    probe = ImageDraw.Draw(Image.new('RGB', (8, 8)))
    font = _clock_font(frame_h)

    def measure(s):
        try:
            bb = probe.textbbox((0, 0), s, font=font)
            return bb[2] - bb[0]
        except Exception:                                       # noqa: BLE001
            return 8 * len(s)

    try:
        bb = probe.textbbox((0, 0), 'Ag', font=font)
        lh = (bb[3] - bb[1]) + 4
    except Exception:                                           # noqa: BLE001
        lh = 16
    avail = max(60, int(width) - 2 * _CLOCK_PAD - 6)
    worst = max((len(_wrap_clock(ln, avail, measure)) for ln in all_lines
                 if ln), default=0)
    return (lh * worst + 2 * _CLOCK_PAD) if worst else 0


def add_clock_band(frame, lines, band_h):
    """``frame`` with a clock BAND grown underneath it. Returns a NEW image.

    **The band exists instead of an overlay, and that is the whole point.** The
    clock used to be drawn bottom-left ON the board, mirroring
    ``BoardRenderer._label``'s top-left corner -- but ``_label`` is one short
    line and the clock is four, so its black box covered a quarter of the X-ray
    panel, including copper the movie exists to show. Reviewers of #887 said so
    about the first published still, and they were right: an instrument that
    hides the measurement is not an instrument.

    Growing the frame instead means the clock can never occlude anything. Every
    frame grows by the SAME ``band_h`` -- take it from ``clock_band_height``
    over the whole movie, never per frame -- so the constant-frame-size
    invariant holds by construction.

    PIL is imported HERE, the ``make_film._badge`` way, so ``import cmd_timing``
    stays free of third-party modules.
    """
    if not lines or not band_h:
        return frame
    from PIL import Image, ImageDraw

    W, H = frame.size
    out = Image.new('RGB', (W, H + int(band_h)), (0, 0, 0))
    out.paste(frame, (0, 0))
    d = ImageDraw.Draw(out)
    font = _clock_font(H)

    def measure(s):
        try:
            bb = d.textbbox((0, 0), s, font=font)
            return bb[2] - bb[0]
        except Exception:                                       # noqa: BLE001
            return 8 * len(s)

    try:
        bb = d.textbbox((0, 0), 'Ag', font=font)
        lh = (bb[3] - bb[1]) + 4
    except Exception:                                           # noqa: BLE001
        lh = 16
    avail = max(60, W - 2 * _CLOCK_PAD - 6)
    for i, ln in enumerate(_wrap_clock(lines, avail, measure)):
        y = H + _CLOCK_PAD + i * lh
        if y + lh > H + band_h:
            # The band was sized for the worst frame in the movie, so this
            # cannot happen -- but a clipped clock is a wrong clock, not a
            # cosmetic problem, so it stops rather than drawing off the end.
            break
        d.text((_CLOCK_PAD, y), ln, fill=(240, 240, 240), font=font)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path', help='a run work dir, or the cmd_timing.jsonl itself')
    ap.add_argument('--json', action='store_true',
                    help='emit the report as JSON instead of markdown')
    args = ap.parse_args(argv)

    ledger = find_ledger(args.path)
    if not ledger:
        print('cmd_timing: no %s under %s' % (LEDGER_NAME, args.path),
              file=sys.stderr)
        return 2
    # The CLI is where exiting on a malformed ledger is the right answer; the
    # library raises so its other consumer, the movie, can keep going.
    try:
        rows = load_rows(ledger)
    except LedgerError as exc:
        print('cmd_timing: %s' % exc, file=sys.stderr)
        return 2
    try:
        rel = os.path.relpath(ledger)
    except ValueError:
        # Windows: relpath raises across drives ("path is on mount 'D:', start
        # on mount 'C:'"). A work dir on a second drive or a UNC share is
        # ordinary, and cosmetics must not take the report down with them.
        rel = ledger
    if args.json:
        print(json.dumps(report_data(rows, rel), indent=2, sort_keys=True))
    else:
        sys.stdout.write(report_markdown(rows, rel))
    return 0


if __name__ == '__main__':
    sys.exit(main())
