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

import argparse
import collections
import json
import math
import os
import sys

LEDGER_NAME = 'cmd_timing.jsonl'

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
                raise SystemExit('%s:%d: not a JSONL row: %s' % (path, n, e))
            if not isinstance(row, dict):
                raise SystemExit('%s:%d: not a JSONL row: not an object' % (path, n))
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


def fmt_hms(seconds):
    """``H:MM:SS`` for a duration, rounded to the NEAREST second.

    ``floor(s + 0.5)``, deliberately, and both alternatives are wrong:

      * truncation misses four of the nine H:MM:SS values in the hand-written
        run-24 report (6.8 -> 0:00:06 not 0:00:07; 4658.7 -> 1:17:38 not
        1:17:39);
      * Python's ``round`` is banker's, so ``round(0.5) == 0``.

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

Anchor = collections.namedtuple('Anchor', 'label board first last t stage basis')


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
    17 chain boards resolved to exactly one row, every one the command that
    semantically wrote it (r1_pour -> R1-pour, r3_route -> R3-route,
    r7_lc -> R7-layercosts). The 17th is the seed board, whose mtime precedes
    the run by 101 s -- which is the right answer, not a miss.

    argv matching is the FALLBACK, not the primary, because mtime is destroyed
    by copying a work dir and by `make_film --from-ledger`, which materialises
    boards out of a content-addressed store. It is a fallback rather than the
    rule because on the same run it is wrong three times: `r4` is first
    mentioned by a DRY run that never wrote it (exit 4), `routed` by a checker
    that only READ it, and `r5_prune` by a step that exited 1.

    ``mtimes`` overrides ``os.path.getmtime`` (tests, and any caller that knows
    better).
    """
    rows = [r for r in (rows or [])
            if r.get('t_start') is not None and r.get('t_end') is not None]
    out = []
    for m in (marks or []):
        label, board, first, last = m[0], m[1], m[2], m[3]
        t, stage, basis = None, None, 'none'
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
                basis = 'argv' if clean else 'argv?'
        if t is None and mt is not None and rows:
            t0 = min(r['t_start'] for r in rows)
            t1 = max(r['t_end'] for r in rows)
            if t0 <= mt <= t1:
                t, basis = mt, 'mtime-loose'
            elif mt < t0:
                # The seed board, written before the run started. Clamping to
                # t0 is honest -- the film opens at the run's beginning -- and
                # the basis says the instant is the run's start, not the file's.
                t, basis = t0, 'pre-run'
        out.append(Anchor(label, board, first, last, t, stage, basis))

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
    'Reading', 'elapsed_s stage basis remaining_s covered interpolated')


class RunClock(object):
    """Frame index -> where that frame sits on the RUN's wall clock.

    The basis is the run clock and nothing else. Tool time is not it: in run 24
    the wrapped commands account for 253.3 s of a 4658.7 s run -- 5.4% -- so a
    countdown driven by tool time would read "nearly done" for over an hour.

    Within a step's frame span the instant is INTERPOLATED between two measured
    endpoints (this step's and the next resolved one's) and never projected past
    the last. Copper reveal is not uniform in time, so an interpolated figure is
    a smoothing rather than a measurement, and the overlay says so.

    ``remaining_s`` is offered ONLY when ``covered`` -- see ``_covered`` -- and
    it is then EXACT: the movie is built after the run, so the total is a
    recorded fact and the subtraction is arithmetic. It is never an estimate,
    and when the ledger falls short the field is absent rather than guessed.
    """

    def __init__(self, anchors, tot, n_frames):
        self.anchors = list(anchors or [])
        self.tot = tot
        self.n = int(n_frames)
        self.resolved = [a for a in self.anchors if a.t is not None]
        self.covered = self._covered()

    def _covered(self):
        """Does the ledger demonstrably span the whole film?

        All of: at least two rows with a real span; EVERY mark resolved; and the
        film's first and last anchors bracketing the run's own first and last
        wrapped commands. Anything less and no remaining figure is offered.
        """
        t = self.tot
        if not t or not t.n or t.run_s is None or t.run_s <= 0 or t.n < 2:
            return False
        if not self.anchors or len(self.resolved) != len(self.anchors):
            return False
        return (self.resolved[0].t <= t.t0 + 1e-6
                and self.resolved[-1].t >= t.t1 - 1e-6 - (t.run_s * 0.0))

    def shortfall(self):
        """Why `covered` is False, in words, or '' when it is True."""
        if self.covered:
            return ''
        t = self.tot
        if not t or t.run_s is None or t.n < 2:
            return 'the ledger has no usable span'
        missing = len(self.anchors) - len(self.resolved)
        if missing:
            return ('ledger covers %d of %d beats'
                    % (len(self.resolved), len(self.anchors)))
        return 'the film does not span the whole run'

    def at(self, i):
        """The ``Reading`` for frame ``i``."""
        t = self.tot
        if not self.anchors or t is None or t.t0 is None:
            return Reading(None, None, 'no ledger', None, False, False)
        k = None
        for j, a in enumerate(self.anchors):
            if a.first <= i < a.last:
                k = j
                break
        if k is None:
            # Before the first mark: build_boards' own "input" snapshot, which
            # is the run's beginning.
            if i < (self.anchors[0].first if self.anchors else 0):
                a0 = self.anchors[0]
                return Reading(0.0, a0.stage, a0.basis, self._rem(t.t0),
                               self.covered, False)
            k = len(self.anchors) - 1
        a = self.anchors[k]
        if a.t is None:
            return Reading(None, a.stage, 'none', None, self.covered, False)
        nxt = next((b for b in self.anchors[k + 1:] if b.t is not None), None)
        interp = False
        inst = a.t
        if nxt is not None and nxt.first > a.first:
            frac = (i - a.first) / float(max(1, nxt.first - a.first))
            frac = min(1.0, max(0.0, frac))
            inst = a.t + (nxt.t - a.t) * frac
            interp = frac not in (0.0,)
        return Reading(max(0.0, inst - t.t0), a.stage, a.basis,
                       self._rem(inst), self.covered, interp)

    def _rem(self, inst):
        if not self.covered:
            return None
        return max(0.0, self.tot.t1 - inst)

    def lines(self, i):
        """The overlay text for frame ``i``. Three lines, four when licensed.

        The first token is literally RUN CLOCK -- never ETA, never a bare "time
        left" -- because the frame must say what the number is before it says
        the number. The remaining line carries its qualifier inside one string
        so a later edit cannot drop the parenthetical and leave a bare countdown
        on screen.
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
        stage = r.stage or 'unlabelled'
        out.append('stage  %s' % stage)
        how = 'interpolated within %s' % stage if r.interpolated else \
            'mapped by %s' % (r.basis or 'nothing')
        out.append('basis  cmd_timing.jsonl - %d wrapped commands, %s'
                   % (t.n if t else 0, how))
        if r.remaining_s is not None:
            out.append('remaining  %s  (exact, post-hoc: the run is over; this '
                       'is a recorded total)' % fmt_hms(r.remaining_s))
        return out

    def meta(self, i):
        """The PNG text block for frame ``i``. Every value a recorded fact.

        No `eta` key and no `progress` key: a percentage invites being read as a
        prediction, and elapsed/total is derivable from two fields already here.
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
        if r.elapsed_s is not None:
            m['krt:elapsed_s'] = round(r.elapsed_s, 1)
            m['krt:elapsed_hms'] = fmt_hms(r.elapsed_s)
            m['krt:t_epoch'] = round(t.t0 + r.elapsed_s, 3)
        if t and t.run_s is not None:
            m['krt:run_total_s'] = round(t.run_s, 1)
            m['krt:run_total_hms'] = fmt_hms(t.run_s)
            m['krt:tool_s'] = round(t.tool_s, 1)
            m['krt:outside_s'] = round(t.outside_s, 1)
        if r.remaining_s is not None:
            m['krt:remaining_s'] = round(r.remaining_s, 1)
            m['krt:remaining_basis'] = 'exact-post-hoc'
        for k, a in enumerate(self.anchors):
            if a.first <= i < a.last:
                m['krt:step'] = a.label
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


def stamp_run_clock(frame, lines):
    """Draw the run clock BOTTOM-LEFT, in place. Never changes ``frame.size``.

    In place because every frame handed to ``save_movie`` must be one size:
    ``animate_route._write_mp4`` raises on a change mid-stream, catches it, and
    degrades the whole movie to GIF silently.

    Bottom-left mirrors ``route_render.BoardRenderer._label``'s top-left, using
    the same font helper, black box and text colour, so the two read as one
    instrument rather than two. PIL is imported HERE, the ``make_film._badge``
    way, so ``import cmd_timing`` stays free of third-party modules.
    """
    if not lines:
        return frame
    from PIL import ImageDraw
    from route_render import load_font

    d = ImageDraw.Draw(frame)
    W, H = frame.size
    font = load_font(max(11, H // 55))
    pad = 6
    avail = max(60, W - 2 * pad - 6)

    def _w(s):
        try:
            bb = d.textbbox((0, 0), s, font=font)
            return bb[2] - bb[0]
        except Exception:                                       # noqa: BLE001
            return 8 * len(s)

    # WRAP, do not clip. A one-line clock overflowed a 700 px frame the first
    # time it was drawn, and PIL clips at the edge in silence -- the same trap
    # _label documents, where a strip that ends at a plausible-looking field
    # reads as the whole story.
    wrapped = []
    for ln in lines:
        cur = ''
        for word in ln.split(' '):
            cand = (cur + ' ' + word) if cur else word
            if cur and _w(cand) > avail:
                wrapped.append(cur)
                cur = word
            else:
                cur = cand
        wrapped.append(cur)
    try:
        bb = d.textbbox((0, 0), 'Ag', font=font)
        lh = (bb[3] - bb[1]) + 4
    except Exception:                                           # noqa: BLE001
        lh = 16
    box_h = lh * len(wrapped) + 6
    top = H - box_h - pad
    box_w = max(_w(x) for x in wrapped) if wrapped else 0
    d.rectangle([pad - 3, top - 3, pad + box_w + 3, H - pad + 3],
                fill=(0, 0, 0))
    for i, ln in enumerate(wrapped):
        d.text((pad, top + i * lh), ln, fill=(240, 240, 240), font=font)
    return frame


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
    rows = load_rows(ledger)
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
