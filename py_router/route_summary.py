#!/usr/bin/env python3
"""One honest tally from route.py's one-or-two JSON_SUMMARY emissions.

route.py runs an end-of-run reconciliation pass exactly when the first pass left
failures. It self-invokes batch_route one level deep on the written board and
prints a SECOND JSON_SUMMARY, scoped to the retried nets, whose failure lists
route.py itself calls "the honest still-open set". Reading only the first summary
counts every reconciliation recovery as a failure.

This module owns that reduction, in two forms:

    merge_summaries(summaries, aborted)  -- from dicts, for an in-process caller
    merge_route_summaries(log)           -- from a log, for a subprocess caller

`route.py --json-out` uses the first (it has the dicts and knows whether the
reconciliation raised, so it needs neither a regex nor a marker string).
`place_route_loop.py` uses the second, because it only ever sees a log file.
Both were the same 110 lines in two places until they were not.
"""
import json
import os
import re
from typing import Dict, List, Optional

__all__ = ['merge_summaries', 'merge_route_summaries', 'summary_min',
           'write_summary_file', 'SUMMARY_RE', 'SUMMARY_MIN_RE',
           'RECONCILE_ABORTED', 'EFFORT_KEYS']

SUMMARY_RE = re.compile(r'JSON_SUMMARY: (\{.*\})')

# The one-line compact tally route.py prints at the end of every OUTERMOST
# run (CLI and GUI alike): the merged verdict in <1KB, where the big
# JSON_SUMMARY lines run 6-20KB each with scope semantics the log has to warn
# about. The trailing colon-space differs from SUMMARY_RE's subject, so
# neither regex can eat the other.
SUMMARY_MIN_RE = re.compile(r'JSON_SUMMARY_MIN: (\{.*\})')

# route.py prints this from the except around its reconciliation self-invoke.
# The sub-run prints its JSON_SUMMARY BEFORE the board is written, so a summary
# followed by this marker advertises recoveries that may never have reached the
# file on disk.
RECONCILE_ABORTED = 'final reconciliation pass failed:'

# Counters that measure WORK DONE, so they add across passes. Everything else in
# a summary is state, and state is whatever the LAST pass measured.
EFFORT_KEYS = ('total_iterations', 'total_vias', 'total_time')


def merge_summaries(summaries: List[Dict], aborted: bool = False) -> Optional[Dict]:
    """Reduce one-or-more summary dicts to a single tally.

    Per field class:

    * FAILURE STATE (failed_single / open_single / failed_multipoint /
      multipoint_pads_*) comes from the LAST summary, and is exact rather
      than a delta. Every net
      with a nonzero failure term is in the retry set by construction, and the
      sub-run re-derives each retried net's pad counts over ALL of that net's
      pads from the final-board union-find, so the last summary's numbers are
      absolute. Summing pad counts would double-count.
    * EFFORT is SUMMED: both passes are work this run cost the router, and an
      iteration tiebreak should see all of it. Taking effort from the last
      summary would make a badly failing candidate look cheap, since the
      reconciliation pass only re-routes a handful of nets.
    * The pad-pair keys are REBUILT, because they are whole-board in the first
      summary but reconcile-subset-scoped in the sub-run's.
    * Anything else is last-wins.

    `aborted` means the reconciliation raised AFTER printing its summary: it
    claims recoveries the board write may never have committed, so fall back to
    the first pass, which is what is definitely on disk.

    Degrades to the single-summary case unchanged. Returns None for an empty
    list.
    """
    if not summaries:
        return None
    merged = dict(summaries[0] if aborted else summaries[-1])

    for key in EFFORT_KEYS:
        merged[key] = sum(s.get(key, 0) for s in summaries)

    # INCOMPLETENESS IS STICKY. `complete: false` marks a run that did not
    # finish, and merging is last-wins for everything not named
    # above -- so a complete second pass would erase a partial first pass's
    # disclosure and the merged tally would read as a whole-board result built
    # partly on numbers nobody finished computing. Same reasoning as `aborted`
    # just below: what matters is what is definitely on disk. `.get(...,
    # True)` leaves an ordinary log untouched.
    if any(not s.get('complete', True) for s in summaries):
        merged['complete'] = False
        _p = next((s for s in summaries if not s.get('complete', True)), {})
        for k in ('status', 'stopped_in', 'deadline_s', 'elapsed_s'):
            if _p.get(k) is not None:
                merged[k] = _p[k]

    # THE ORACLE'S ANSWER IS STICKY IN THE OTHER DIRECTION. `oracle_check` is
    # not a last-wins field: the reconciliation sub-run never reaches the
    # oracle block, so it emits the initialiser `'skipped'`, and last-wins then
    # threw away a real answer from pass 1. Measured: a log carrying 10+
    # `ORACLE CHECK:` lines where KiCad contradicted in-process grading merged
    # to `oracle_check: 'skipped'` -- so the run's verdict rested on the
    # router's own tally while the summary said the authority had not been
    # consulted at all.
    #
    # Precedence, worst-news-first: a contradiction outranks agreement, which
    # outranks "could not ask", which outranks "did not ask".
    _ORACLE_RANK = {'failed': 4, 'ok': 3, 'unavailable': 2, 'disabled': 1}

    def _orank(v):
        return _ORACLE_RANK.get(str(v).split(' ')[0], 0)

    _oracles = [s.get('oracle_check') for s in summaries
                if s.get('oracle_check') is not None]
    if _oracles:
        merged['oracle_check'] = max(_oracles, key=_orank)

    # Rebuild the pad-pair tallies and the blockers key, which last-wins would
    # silently narrow to the reconcile subset (a 50/40 whole-board reading
    # becomes the sub-run's 3/2, or vanishes entirely). Skipped when aborted:
    # merged is already pass 1 wholesale, which is what is on disk.
    if len(summaries) > 1 and not aborted:
        first, last = summaries[0], summaries[-1]
        if 'pad_pairs_total' in first:
            if 'pad_pairs_total' in last:
                # Denominator: pass 1's whole board. Connected: that total minus
                # what is STILL open at end of run. A net the reconciliation
                # itself broke that pass 1 never graded subtracts its deficit
                # without widening the denominator -- conservative, same spirit
                # as the coverage-gate widening below.
                _deficit = sum(
                    e.get('pairs_total', 0) - e.get('pairs_connected', 0)
                    for e in (last.get('pad_pairs_open') or []))
                _total = first['pad_pairs_total']
                merged['pad_pairs_total'] = _total
                merged['pad_pairs_connected'] = max(0, _total - _deficit)
            else:
                # The sub-run printed a summary without pad-pair keys (its
                # emission is defensively try/except'd): pass 1's numbers are
                # the newest that exist.
                merged['pad_pairs_total'] = first['pad_pairs_total']
                merged['pad_pairs_connected'] = first.get('pad_pairs_connected', 0)
                if 'pad_pairs_open' in first:
                    merged['pad_pairs_open'] = first['pad_pairs_open']
        # `blockers` and `boxed_in` are both first-pass attribution: the
        # reconcile sub-run re-routes a SUBSET and its summary carries neither,
        # so without this a merged summary loses the evidence for nets that are
        # still failing. Filtered to the nets that ARE still failing, so a net
        # the reconcile fixed does not carry a stale accusation.
        _failed = None
        for _k in ('blockers', 'boxed_in'):
            if _k in first and _k not in merged:
                if _failed is None:
                    _failed = set(merged.get('failed_single') or [])
                    _failed |= {d.get('net_name') if isinstance(d, dict) else d
                                for d in (merged.get('failed_multipoint') or [])}
                merged[_k] = [e for e in first[_k] if e.get('net') in _failed]
        # `finalize_excluded_nets` carries WHOLE, not through the
        # still-failing filter: it is a list of net NAMES rather than per-net
        # records, and it states what the finalize declined to do BY PLAN --
        # something the reconcile sub-run neither repeats nor revokes. It is
        # stamped on the summary AFTER the JSON_SUMMARY line printed, so it
        # exists only on `first`; without this carry, last-wins drops it on
        # exactly the runs that reconciled -- i.e. the failing ones, where
        # telling "declined by plan" from "failed to" is the whole point.
        if ('finalize_excluded_nets' in first
                and 'finalize_excluded_nets' not in merged):
            merged['finalize_excluded_nets'] = first['finalize_excluded_nets']

    # DISTURBED-BUT-UNOWNED NETS ARE STICKY (#622 yw1: SA1 shipped with ZERO
    # copper, SA2/SA6 open, and the merged MIN said failed:2 deficit:0). A
    # middle pass's coverage_gate_nets / ripped_open_uncounted name rip
    # victims OUTSIDE that pass's --nets scope, verified broken against real
    # copper at emission time -- and a later, narrower sub-run's summary
    # carries neither key, so last-wins erased the only record of them.
    # Union them across all passes, dropping any net a LATER summary
    # CLASSIFIED (routed_single = recovered; failed/open/multipoint = that
    # pass took ownership and already counts it), so nothing double-counts.
    # terminal_restores merges the same way (per-net) so summary_min's
    # terminal_restores_broken survives the merge -- and a restore mark can
    # be superseded WITHIN its own pass: the reroute loop re-routes the
    # victim after the stub restore, and the pass-end routed_single
    # (re-derived from the final-board union-find) is the truth (yt1:
    # SDQ7/SDQ6/SA4 marked stub, same-pass routed, board grades clean; yv3:
    # single-summary form of the same). So a mark survives only while its
    # net is in neither its own pass's routed_single nor any later pass's
    # classification. This applies to SINGLE-summary logs too. When aborted,
    # only pass 1 (what is on disk) participates.
    _use = summaries[:1] if aborted else summaries

    def _classified_names(s):
        names = set(s.get('routed_single') or [])
        names |= set(s.get('failed_single') or [])
        names |= set(s.get('open_single') or [])
        names |= {d.get('net_name') if isinstance(d, dict) else d
                  for d in (s.get('failed_multipoint') or [])}
        return names

    _gate_all: List[str] = []
    _tr_merged: Dict = {}
    for _i, _s in enumerate(_use):
        _later: set = set()
        for _t in _use[_i + 1:]:
            _later |= _classified_names(_t)
        for _n in (list(_s.get('coverage_gate_nets') or [])
                   + list(_s.get('ripped_open_uncounted') or [])):
            if _n not in _later and _n not in _gate_all:
                _gate_all.append(_n)
        _own_routed = set(_s.get('routed_single') or [])
        for _n, _v in (_s.get('terminal_restores') or {}).items():
            if _v == 'full' or (_n not in _later
                                and _n not in _own_routed):
                _tr_merged[_n] = _v
    if _gate_all or 'coverage_gate_nets' in merged:
        merged['coverage_gate_nets'] = _gate_all
    if _tr_merged or 'terminal_restores' in merged:
        merged['terminal_restores'] = _tr_merged

    # Coverage-gate nets have NO routed result, so their pads never reach
    # multipoint_pads_total and a caller's
    # failures = len(failed_single) + pad-deficit weighs them ZERO, though they
    # ship at broken copper. Give each one weight 1, matching what failed_single
    # gives a net that produced no result at all, by widening the pad
    # denominator. They are in neither failed_single nor the pad tallies, so
    # this cannot double-count. It matters most on the LAST summary: those are
    # nets the reconciliation pass ITSELF broke through its rip escalation, and
    # without this a loop can read failures=0 on a board shipping disconnected
    # copper and stop.
    gate = merged.get('coverage_gate_nets') or []
    if gate:
        merged['multipoint_pads_total'] = (
            merged.get('multipoint_pads_total', 0) + len(gate))
    return merged


def merge_route_summaries(log: str) -> Optional[Dict]:
    """`merge_summaries` for a caller that only has the log text.

    Returns None when the log carries no summary at all.
    """
    raw = SUMMARY_RE.findall(log)
    if not raw:
        return None
    summaries = [json.loads(s) for s in raw]
    aborted = log.rfind(RECONCILE_ABORTED) > log.rfind(raw[-1])
    return merge_summaries(summaries, aborted)


def summary_min(merged: Dict, name_cap: int = 20) -> Dict:
    """The <1KB verdict an agent reads INSTEAD of the big summaries.

    Every value here is derived from the MERGED tally, so it carries the
    "run-scope plus recoveries" semantics automatically -- the trap the log
    warns about ("never scrape the LAST JSON_SUMMARY") cannot be re-imported
    through this line. Name lists are capped at `name_cap` with an explicit
    '+N more' marker, never silently truncated.

    `finalize_excluded_nets` (plane nets outside the route's --nets scope,
    excluded from the finalize by plan) is set on the summary AFTER the
    `JSON_SUMMARY:` line was printed, so it reaches `--json-out`, the dict
    `batch_route` returns, and this line -- not the printed big summary. It
    is included here only when present.

    Deliberately ABSENT: power_widths, ampacity, stacked copper --
    forensics that stay in the big summaries / --json-out. And the DRC-floor
    writeback verdict, which does not exist yet when this prints: the
    writeback runs afterwards and reports on its own, so a consumer must
    read both -- this line says nothing about whether the floors held.
    """
    def _names(vals) -> List[str]:
        names = [str(v) for v in (vals or [])]
        if len(names) > name_cap:
            return names[:name_cap] + [f'+{len(names) - name_cap} more']
        return names

    pairs = merged.get('pad_pairs_open') or []
    tr = merged.get('terminal_restores') or {}
    broken_restores = sorted(n for n, v in tr.items()
                             if v in ('full_open', 'stub'))
    total = merged.get('multipoint_pads_total') or 0
    conn = merged.get('multipoint_pads_connected') or 0
    out = {
        'scope': 'merged',
        'routed': merged.get('successful'),
        'failed': merged.get('failed'),
        'failed_single': _names(merged.get('failed_single')),
        'open_single': _names(merged.get('open_single')),
        'multipoint_deficit': max(0, total - conn),
        'pad_pairs_open': {
            'count': len(pairs),
            'nets': _names(sorted({p.get('net') for p in pairs
                                   if p.get('net')}))},
        'terminal_restores_broken': _names(broken_restores),
        'min_clearance_used': merged.get('min_clearance_used'),
        'vias': merged.get('total_vias'),
        # NOT wall clock, and named for what it actually counts. `total_time`
        # is the single-ended loop plus the reroute loop and nothing else --
        # phase-3 taps, rescues, the plane finalize, and parse/write all sit
        # outside it. Measured on splitflap_driver: 0.35 against 20.72 s of
        # real wall time, a 59x under-report. The big summary can afford to
        # call it `total_time` among thirty other keys; a one-line verdict an
        # agent reads INSTEAD of those cannot call it `duration_s` without
        # asserting the run took that long.
        'main_loop_time_s': merged.get('total_time'),
    }
    if merged.get('finalize_excluded_nets'):
        out['finalize_excluded_nets'] = _names(
            merged['finalize_excluded_nets'])
    return out


def write_summary_file(path: str, merged: Optional[Dict]) -> None:
    """Publish `merged` at `path` ALL-OR-NOTHING. Raises if it cannot.

    `route.py --json-out` is a published contract with readers this repo does
    not own -- `place_route_loop --accept-cmd` hands the path straight to an
    arbitrary external judge -- and every reader opens it the same way:
    ``if os.path.isfile(js): json.load(...)``. So a file that EXISTS and does
    not parse is the worst artifact this can produce: the existence check
    passes and the parse raises, out of loops that catch only
    subprocess.TimeoutExpired.

    The call this replaced could not avoid producing exactly that. It was
    ``open(path, 'w')`` followed by ``json.dump(...)``: the open TRUNCATES the
    destination before the first chunk is encoded, and json.dump then streams
    -- it calls ``iterencode(o)`` with ``_one_shot=False``, so the C encoder is
    never used, with or without `indent`, and the pure-Python generator writes
    chunk by chunk straight into the handle. Any failure partway therefore
    published a valid PREFIX of the document. route.py's `except Exception`
    around the call cannot undo that: the truncation already happened, and it
    only prints a WARNING, after which route.py exits 0 like any other run.

    So: serialise FIRST (a payload that will not encode opens nothing at all),
    write to a sibling temp file, and publish with os.replace, which is atomic
    on POSIX and Windows alike. A reader sees the whole previous file or the
    whole new one, never half of either. Same idiom as board_store.put and the
    .kicad_pro writeback, and the one tests/stress/predictor_study.py adopted
    after a study run died on a JSONDecodeError that "read like a study failure
    and was a file-system race" -- this bug, in another file.

    Deliberately NO ``default=str``: route.py prints the same dict through a
    bare ``json.dumps`` outside any try, before this is ever reached, so an
    unserialisable summary is already a loud failure of the whole run.
    Coercing here would let the FILE carry a ``<obj at 0x...>`` repr that the
    stdout line refused -- non-deterministic, and silently divergent from
    ``merge_route_summaries(log)``, which a test compares against this file.

    Deliberately NO fallback document either. `converge.route_verdict` scores a
    truthy dict carrying none of the failure keys as `(0, 'clean')`, so an
    ``{'complete': false}`` placeholder would rank a broken candidate FIRST.
    Failing closed -- no file -- lands every caller in the `summary = {}` /
    "no summary" case they all already handle, where a candidate without a
    verdict ranks last and never best.
    """
    # indent=1, no sort_keys, default ensure_ascii: byte-identical to what the
    # streaming writer produced, so the on-disk format does not change.
    text = json.dumps({} if merged is None else merged, indent=1)
    # pid-suffixed: several routes can share one output directory.
    tmp = f'{path}.{os.getpid()}.tmp'
    published = False
    try:
        with open(tmp, 'w', encoding='utf-8') as fh:
            fh.write(text)
        # Windows refuses os.replace while another process holds the
        # destination open (PermissionError: [WinError 5]). Letting that
        # propagate is correct: the caller prints its WARNING and the
        # destination keeps whatever it already had.
        os.replace(tmp, path)
        published = True
    finally:
        if not published:
            try:
                os.remove(tmp)
            except OSError:
                pass
