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

THE FINAL RE-GRADE (#1069). A merge of summaries can only report what some
summary recorded, and each summary grades ITS OWN scope at ITS OWN moment:
pass 1 before the plane finalize, a finalize repair sub-run its casualties, a
reconciliation lap its retry set. A net one of them left broken that no later
one looked at fell out of the merged tally (glasgow run34: 19 reported, 40
disconnected on disk). So the outermost route grades the board it SHIPS, once,
over every net the run owns (`regrade_record`), prints the result as a
`JSON_REGRADE:` line, and both merges apply it on top of the summaries: the
failure state then describes the final board rather than the last lap.
"""
import json
import os
import re
from typing import Dict, Iterable, List, Optional

__all__ = ['merge_summaries', 'merge_route_summaries', 'summary_min',
           'write_summary_file', 'SUMMARY_RE', 'SUMMARY_MIN_RE', 'REGRADE_RE',
           'RECONCILE_ABORTED', 'EFFORT_KEYS', 'SUMMARY_SINK',
           'RECONCILE_RAISED', 'FINAL_REGRADE', 'reset_run_state',
           'named_nets', 'regrade_record', 'GATE_RE',
           'apply_improvement_gate']

SUMMARY_RE = re.compile(r'JSON_SUMMARY: (\{.*\})')
# #962: `fab_notes`' ship-time via_in_pad record, printed after the summaries
VIA_IN_PAD_RE = re.compile(r'VIA_IN_PAD_JSON: (\{.*\})')
# #1069: the outermost run's final-board re-grade, printed once, after every
# JSON_SUMMARY of that run and before --json-out is written.
REGRADE_RE = re.compile(r'JSON_REGRADE: (\{.*\})')
# #1173: the outermost run's improvement-gate verdict, printed once after every
# JSON_SUMMARY of that run. route.py publishes it on --json-out as well.
GATE_RE = re.compile(r'JSON_IMPROVEMENT_GATE: (\{.*\})')


def apply_improvement_gate(merged: Optional[Dict], gate: Optional[Dict]):
    """Set the improvement gate's verdict on a merged document (#1173): the
    report under ``improvement_gate`` and, when it REVERTED the output to the
    input board, ``shipped`` / ``shipped_note`` -- the tallies then describe
    the rejected attempt, not the board that ships. route.py's --json-out
    writer and `merge_route_summaries` both call this, so the file and the log
    stay one document (#830)."""
    if merged is None or gate is None:
        return merged
    merged['improvement_gate'] = gate
    if gate.get('verdict') == 'reject':
        merged['shipped'] = 'input board'
        merged['shipped_note'] = (
            'the improvement gate REJECTED this run and the output is the '
            'input board; every tally in this file describes the rejected '
            'attempt, not the shipped board')
    return merged

# ONE summary sink per PROCESS (#1069). It used to live in route.py, which is
# `__main__` when run from the CLI -- and the plane finalize reaches its repair
# sub-runs through `repair_planes`' `from route import batch_route`, which
# loads a SECOND copy of route.py with a sink of its own. Those sub-runs
# printed a JSON_SUMMARY that `merge_route_summaries(log)` folded in while
# `--json-out` never saw it, so the file and the log merge disagreed (#830's
# one-document rule), and the second copy's empty sink also stamped them
# scope='run' and reset the outer run's protected-net refusal record. This
# module is imported under ONE name by every copy, so its lists are shared.
# Mutate them in place only (`.clear()`, `.append()`, `[0] = ...`): a
# rebinding would split them again.
SUMMARY_SINK: List[dict] = []
RECONCILE_RAISED = [False]
FINAL_REGRADE: List[Optional[dict]] = [None]


def reset_run_state() -> None:
    """Start an OUTERMOST run clean (a nested sub-run must never call this)."""
    SUMMARY_SINK.clear()
    RECONCILE_RAISED[0] = False
    FINAL_REGRADE[0] = None

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


def merge_summaries(summaries: List[Dict], aborted: bool = False,
                    regrade: Optional[Dict] = None) -> Optional[Dict]:
    """Reduce one-or-more summary dicts to a single tally.

    Per field class:

    * FAILURE STATE (routed_single / failed_single / open_single /
      failed_multipoint / multipoint_pads_* / successful / failed) comes from
      `regrade` when there is one -- the outermost run's grade of the board it
      shipped (`regrade_record`), which is the only reading that covers every
      summary's scope at once. Without one (a log from before #1069, or a
      re-grade that raised) it falls back to the LAST summary. That fallback
      is exact only for the nets the last summary graded: a reconciliation lap
      retries every net pass 1 left failing and re-derives their pad counts
      over the final union-find, but a net a plane-finalize sub-run or an
      earlier lap left broken outside that retry set is invisible to it --
      which is the hole the re-grade closes. Summing pad counts would
      double-count either way.
    * EFFORT is SUMMED: both passes are work this run cost the router, and an
      iteration tiebreak should see all of it. Taking effort from the last
      summary would make a badly failing candidate look cheap, since the
      reconciliation pass only re-routes a handful of nets.
    * The pad-pair keys are REBUILT, because they are whole-board in the first
      summary but reconcile-subset-scoped in the sub-run's.
    * Anything else is last-wins.

    `scope` is stamped 'merged' whenever the result is more than one
    summary's word -- several summaries, or a re-grade.

    `aborted` means the reconciliation raised AFTER printing its summary: it
    claims recoveries the board write may never have committed, so fall back to
    the first pass, which is what is definitely on disk. A re-grade still
    applies: it read the board that is on disk.

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
        # `power_widths` (#1033) carries WHOLE for the same reason: the
        # outermost run measures it on the board it SHIPS, after the
        # reconciliation sub-run returned, and stamps it on `first` -- the
        # sub-run's own summary has none. A sub-run that ever measured one
        # would be measuring a slice, so first always wins.
        for _k in ('power_widths', 'power_widths_measured_on',
                   'power_widths_run_scope'):
            if _k in first:
                merged[_k] = first[_k]

    # DISTURBED-BUT-UNOWNED NETS ARE STICKY (#622 yw1: SA1 shipped with ZERO
    # copper, SA2/SA6 open, and the merged MIN said failed:2 deficit:0). A
    # middle pass's coverage_gate_nets / ripped_open_uncounted name rip
    # victims OUTSIDE that pass's --nets scope, verified broken against real
    # copper at emission time -- and a later, narrower sub-run's summary
    # carries neither key, so last-wins erased the only record of them.
    # Union them across all passes, dropping a net only when its LAST
    # classification after the flag makes the flag redundant: routed_single
    # there (recovered), or a failure bucket of the FINAL summary (last-wins
    # already counts it). A failure bucket of a MIDDLE summary does not: that
    # summary's buckets are overwritten by last-wins, so dropping the flag on
    # its word lost the net entirely (#1069: gate in lap 1, failed_single in
    # a middle sub-run, a last lap that never looked at it -> in no bucket).
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
    _last_i = len(_use) - 1

    def _flag_is_redundant(name, after):
        """Is a sticky flag on `name`, raised in summary `after`, carried by
        a later classification? Walks backwards to the LAST summary that
        classified the net."""
        for _j in range(_last_i, after, -1):
            _c = _classify(_use[_j]).get(name)
            if _c is None:
                continue
            return _c == 'routed' or _j == _last_i
        return False

    _gate_all: List[str] = []
    _tr_merged: Dict = {}
    for _i, _s in enumerate(_use):
        _later: set = set()
        for _t in _use[_i + 1:]:
            _later |= set(_classify(_t))
        for _n in (list(_s.get('coverage_gate_nets') or [])
                   + list(_s.get('ripped_open_uncounted') or [])):
            if _n not in _gate_all and not _flag_is_redundant(_n, _i):
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

    if len(summaries) > 1 or regrade:
        merged['scope'] = 'merged'

    if regrade:
        _apply_regrade(merged, regrade, summaries)
        return merged

    # FALLBACK CARRY (no re-grade). A MIDDLE summary -- a plane-finalize
    # repair sub-run, printed between pass 1 and the reconciliation laps --
    # can leave a net failing that no later summary classifies: the laps
    # retry pass 1's failures, not the finalize's. Last-wins overwrote its
    # buckets, so carry each such net in the bucket it was left in. Pass 1 is
    # deliberately NOT carried: the laps retry every net it left failing, so
    # one they did not classify was already connected when they started. The
    # sticky flags above already carry theirs, so they are skipped here.
    if not aborted and len(_use) > 2:
        _gate_set = set(_gate_all)
        _fs = list(merged.get('failed_single') or [])
        _os = list(merged.get('open_single') or [])
        _fm = list(merged.get('failed_multipoint') or [])
        _carried = False
        for _i in range(1, _last_i):
            _s = _use[_i]
            _later = set()
            for _t in _use[_i + 1:]:
                _later |= set(_classify(_t))
            _entries = {_fm_name(d): d for d in (_s.get('failed_multipoint')
                                                 or [])}
            for _n, _b in _classify(_s).items():
                if _b == 'routed' or _n in _later or _n in _gate_set:
                    continue
                _carried = True
                if _b == 'failed_single':
                    _fs.append(_n)
                    continue
                if _b == 'open_single':
                    _os.append(_n)
                if _n in _entries:
                    _fm.append(_entries[_n])
                if _b == 'multipoint':
                    _k = len((_entries.get(_n) or {}).get('failed_pads')
                             or []) or 1
                    merged['multipoint_pads_total'] = (
                        merged.get('multipoint_pads_total', 0) + _k)
        if _carried:
            merged['failed_single'] = _fs
            merged['open_single'] = _os
            merged['failed_multipoint'] = _fm

    # Coverage-gate nets have NO routed result, so their pads never reach
    # multipoint_pads_total and a caller's
    # failures = len(failed_single) + pad-deficit weighs them ZERO, though they
    # ship at broken copper. Give each one weight 1, matching what failed_single
    # gives a net that produced no result at all, by widening the pad
    # denominator. They are in neither failed_single nor the pad tallies, so
    # this cannot double-count. It matters most on the LAST summary: those are
    # nets the reconciliation pass ITSELF broke through its rip escalation, and
    # without this a loop can read failures=0 on a board shipping disconnected
    # copper and stop. (A re-graded merge returned above: its pad tallies are
    # rebuilt from the board, so the gate nets are already in them.)
    gate = merged.get('coverage_gate_nets') or []
    if gate:
        merged['multipoint_pads_total'] = (
            merged.get('multipoint_pads_total', 0) + len(gate))
    return merged


def _fm_name(entry) -> str:
    return entry.get('net_name') if isinstance(entry, dict) else entry


def _classify(summary: Dict) -> Dict[str, str]:
    """{net: bucket} for every net `summary` CLASSIFIED, one bucket per net.

    Precedence follows the emitter: an open_single net is also listed in
    failed_multipoint, and a coverage-gate net only there. 'multipoint' means
    failed_multipoint and neither single bucket.
    """
    out: Dict[str, str] = {}
    for n in summary.get('routed_single') or []:
        out[n] = 'routed'
    for d in summary.get('failed_multipoint') or []:
        out[_fm_name(d)] = 'multipoint'
    for n in summary.get('open_single') or []:
        out[n] = 'open_single'
    for n in summary.get('failed_single') or []:
        out[n] = 'failed_single'
    return out


def named_nets(summaries: Iterable[Dict]) -> List[str]:
    """Every net any summary NAMES -- classified, flagged or disclosed.

    The re-grade's net set: a summary names exactly the nets its pass worked
    on or found broken, so their union is what the run owns -- pass 1's scope,
    every finalize casualty sub-run's, every lap's, and the out-of-scope
    victims each one disclosed. Order: first appearance.
    """
    seen: Dict[str, None] = {}

    def _add(names):
        for n in names or []:
            if isinstance(n, str) and n:
                seen.setdefault(n, None)

    for s in summaries:
        _add(s.get('routed_single'))
        _add(s.get('failed_single'))
        _add(s.get('open_single'))
        _add([_fm_name(d) for d in (s.get('failed_multipoint') or [])])
        for k in ('coverage_gate_nets', 'ripped_open',
                  'ripped_open_uncounted', 'fragmented_nets'):
            _add(s.get(k))
        for k in ('terminal_restores', 'preexisting_rips', 'oracle_open'):
            v = s.get(k)
            if isinstance(v, dict):
                _add(list(v))
            elif isinstance(v, list):
                _add([e.get('net') if isinstance(e, dict) else e for e in v])
        _add([e.get('net') for e in (s.get('pad_pairs_open') or [])
              if isinstance(e, dict)])
    return list(seen)


def regrade_record(summaries: List[Dict], grades: Dict[str, Dict],
                   routing_scope: Iterable[str], *, board: str,
                   seconds: float = 0.0,
                   disturbed_only: Iterable[str] = ()) -> Dict:
    """The final failure state, rebuilt from a grade of the shipped board.

    `grades` is {net: {'pads': P, 'broken': bool, 'copper': bool,
    'failed_pads': [{x, y, component_ref, pad_number}, ...]}} for every net the
    run owns (`named_nets` of its summaries, plus nets whose copper it
    changed). `routing_scope` is the outermost pass's routing scope (its
    single-ended net list), which `successful` counts. `failed` counts every
    net the run ships broken (#1215): the distinct nets of failed_single,
    open_single and failed_multipoint -- a net this run ripped and could not
    re-route is outside pass 1's list, and `len(scope) - successful` read 0
    for it. `disturbed_only` names the nets graded ONLY because the
    run changed their copper; the caller has already dropped those that were
    no worse than on the input board.

    Each broken net keeps the bucket meanings route.py emits (CLAUDE.md, "Read
    the failure buckets by their real definitions"), decided by the LAST
    summary that classified it:

    * failed_single there -> failed_single ("no result at all", weight 1).
    * open_single there -> open_single (weight 1), plus a failed_multipoint
      entry carrying its pads, as the emitter lists it.
    * failed_multipoint only there, or a net of 3+ pads the last word on
      which was 'routed' or nothing -> failed_multipoint, its disconnected
      pads priced in the multipoint pad deficit.
    * otherwise a two-pad net: open_single if it still has copper of its own,
      failed_single if it has none.

    So `len(failed_single) + len(open_single) + pad deficit` counts every
    broken net exactly once. The pad tallies cover every graded multipoint
    net, connected ones included, so they are a whole-run denominator.
    """
    last: Dict[str, str] = {}
    for s in summaries:
        last.update(_classify(s))
    scope = list(dict.fromkeys(routing_scope or []))
    failed_single: List[str] = []
    open_single: List[str] = []
    failed_mp: List[Dict] = []
    mp_total = mp_conn = 0
    for name in sorted(grades):
        g = grades[name]
        pads = int(g.get('pads') or 0)
        cls = last.get(name)
        if not g.get('broken'):
            if cls == 'multipoint' or (pads >= 3 and cls != 'failed_single'):
                mp_total += pads
                mp_conn += pads
            continue
        fp = list(g.get('failed_pads') or [])
        entry = {'net_name': name, 'failed_pads': fp}
        if cls == 'failed_single':
            failed_single.append(name)
        elif cls == 'open_single':
            open_single.append(name)
            failed_mp.append(entry)
        elif cls == 'multipoint' or pads >= 3:
            failed_mp.append(entry)
            mp_total += pads
            mp_conn += max(0, pads - max(1, len(fp)))
        elif g.get('copper'):
            open_single.append(name)
            failed_mp.append(entry)
        else:
            failed_single.append(name)
    broken = {n for n, g in grades.items() if g.get('broken')}
    routed = [n for n in scope if n in grades and n not in broken]
    was_failing = set()
    for s in summaries:
        was_failing |= {n for n, c in _classify(s).items() if c != 'routed'}
        was_failing |= set(s.get('coverage_gate_nets') or [])
    return {
        'board': board,
        'graded_nets': len(grades),
        'seconds': round(float(seconds), 2),
        'routed_single': routed,
        'failed_single': failed_single,
        'open_single': open_single,
        'failed_multipoint': failed_mp,
        'multipoint_pads_total': mp_total,
        'multipoint_pads_connected': mp_conn,
        'successful': len(routed),
        'failed': len(set(failed_single) | set(open_single)
                      | {d['net_name'] for d in failed_mp}),
        # Disclosure: broken nets NO summary classified (caught only because
        # the run changed their copper), and nets some summary left failing
        # that the shipped board has connected.
        'unowned_broken': sorted(broken & set(disturbed_only) - set(last)),
        'recovered': sorted(was_failing & set(grades) - broken),
    }


def _apply_regrade(merged: Dict, regrade: Dict, summaries: List[Dict]) -> None:
    """Overwrite the merged failure state with the re-grade (in place)."""
    for k in ('routed_single', 'failed_single', 'open_single',
              'failed_multipoint', 'multipoint_pads_total',
              'multipoint_pads_connected', 'successful', 'failed'):
        if k in regrade:
            merged[k] = json.loads(json.dumps(regrade[k]))
    broken = (set(merged.get('failed_single') or [])
              | set(merged.get('open_single') or [])
              | {_fm_name(d) for d in (merged.get('failed_multipoint') or [])})
    # Disclosure lists keep their meaning, narrowed to what still ships broken.
    for k in ('coverage_gate_nets', 'ripped_open', 'ripped_open_uncounted'):
        if k in merged:
            merged[k] = [n for n in (merged.get(k) or []) if n in broken]
    if merged.get('terminal_restores'):
        merged['terminal_restores'] = {
            n: v for n, v in merged['terminal_restores'].items()
            if v == 'full' or n in broken}
    # Per-net attribution, newest entry per net across every summary, kept for
    # the nets that are still failing (the old path kept pass 1's only).
    for k, key in (('blockers', 'net'), ('boxed_in', 'net'),
                   ('pad_pairs_open', 'net')):
        if not any(k in s for s in summaries):
            continue
        by_net: Dict[str, Dict] = {}
        for s in summaries:
            for e in s.get(k) or []:
                if isinstance(e, dict) and e.get(key):
                    by_net[e[key]] = e
        merged[k] = [e for n, e in by_net.items() if n in broken]
    if 'pad_pairs_total' in merged and 'pad_pairs_open' in merged:
        _deficit = sum(e.get('pairs_total', 0) - e.get('pairs_connected', 0)
                       for e in merged['pad_pairs_open'])
        merged['pad_pairs_connected'] = max(
            0, merged['pad_pairs_total'] - _deficit)
    merged['regrade'] = {k: regrade.get(k) for k in (
        'board', 'graded_nets', 'seconds', 'unowned_broken', 'recovered',
        'broken_by_run')}


def merge_route_summaries(log: str) -> Optional[Dict]:
    """`merge_summaries` for a caller that only has the log text.

    Returns None when the log carries no summary at all.
    """
    raw = SUMMARY_RE.findall(log)
    if not raw:
        return None
    summaries = [json.loads(s) for s in raw]
    aborted = log.rfind(RECONCILE_ABORTED) > log.rfind(raw[-1])
    # #1069: the re-grade belongs to this run only if it follows the run's
    # last summary; one printed before it describes an earlier run.
    regrade = None
    rg = list(REGRADE_RE.finditer(log))
    if rg and rg[-1].start() > log.rfind(raw[-1]):
        regrade = json.loads(rg[-1].group(1))
    merged = merge_summaries(summaries, aborted, regrade)
    # #962: the ship-time Type VII record runs after every JSON_SUMMARY line,
    # so it is printed on its own line; route.py sets the same record on the
    # merged `--json-out` document. The last one is the shipped board's.
    vip = VIA_IN_PAD_RE.findall(log)
    if merged is not None and vip:
        merged['via_in_pad'] = json.loads(vip[-1])
    # #1173: the gate's verdict, when it follows the run's last summary.
    gt = list(GATE_RE.finditer(log))
    if gt and gt[-1].start() > log.rfind(raw[-1]):
        apply_improvement_gate(merged, json.loads(gt[-1].group(1)))
    return merged


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
    # #1069: how many nets the final-board re-grade read. Absent when the
    # tally rests on the summaries alone (no re-grade ran).
    if isinstance(merged.get('regrade'), dict):
        out['regraded_nets'] = merged['regrade'].get('graded_nets')
    return out


def _discard(path: str) -> None:
    """Best-effort unlink. A file we cannot remove is reported by the caller,
    never silently tolerated."""
    try:
        os.remove(path)
    except OSError:
        pass


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
    "no summary" case they all already handle. Stated precisely, because the
    three handle it differently: compare_seeds ranks such a row last and never
    picks it as `best`; cmd_poses only annotates; place_portfolio DROPS it from
    the routed ranking and falls through to the static one, so with every probe
    unreadable it still names a best on crossings/HPWL alone (`probe_kind:
    null` is the only tell). That last one is pre-existing and is not made
    worse here -- but it is not "ranks last", and saying so would be the
    comfortable version.

    FAILING CLOSED MEANS DELETING THE DESTINATION, and that is not fussiness.
    An atomic publish that merely declines to overwrite leaves the PREVIOUS
    run's summary standing at `path` -- a complete, parseable document that
    every reader accepts as this run's, with no signal anywhere: the reader's
    guard sees valid JSON, so it reports no error, and the verdict is simply
    about the wrong run. That is worse than the truncated file this replaced,
    which at least announces itself. Measured: with os.replace forced to fail,
    a destination holding {"failed_single": ["OLD_RUN"]} still held it after a
    write of ["NEW_RUN"] -- and `place_route_loop` reuses
    `work/loop_round{N}_route.json` across runs and hands it to `--accept-cmd`,
    so the judge would score this round against the last one's tally.

    If the destination cannot be removed either -- the same lock that blocked
    the rename -- the stale file survives and the raised message SAYS SO, so
    the caller's warning is true rather than reassuring. Note the in-repo
    precedent at predictor_study.py:270 CATCHES this PermissionError instead.
    It can, because it then RE-READS the survivor and compares its argv_sha,
    raising SystemExit on disagreement -- it verifies rather than assumes.
    Nothing here can verify a route summary that way, so it re-raises.
    """
    tmp = f'{path}.{os.getpid()}.tmp'   # several routes can share a directory
    try:
        # indent=1, no sort_keys, default ensure_ascii: byte-identical to what
        # the streaming writer produced, so the on-disk format does not change.
        text = json.dumps({} if merged is None else merged, indent=1)
        with open(tmp, 'w', encoding='utf-8') as fh:
            fh.write(text)
        # Windows refuses os.replace while another process holds the
        # destination open (PermissionError: [WinError 5]).
        os.replace(tmp, path)
    except Exception as exc:
        _discard(tmp)
        _discard(path)
        stale = os.path.exists(path)
        raise OSError(
            f'could not publish {path}: {type(exc).__name__}: {exc}'
            + ('; a PREVIOUS run\'s summary is still there and could NOT be '
               'removed -- do not read it as this run\'s' if stale else
               '; no file was left behind')) from exc
