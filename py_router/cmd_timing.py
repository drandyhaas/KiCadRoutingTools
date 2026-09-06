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
