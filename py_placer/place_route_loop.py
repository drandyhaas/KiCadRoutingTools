"""
Router-in-the-loop placement repair.

Routes the board, reads the failure diagnostics (failed nets + the blocker
nets named in the router's frontier analysis), and micro-quenches ONLY the
parts that could help those routes succeed:

  - parts owning pads of the failed nets (move the endpoint out of the
    congested pocket), and
  - parts owning pads of the blocker nets (move the anchor so the blocking
    wall re-routes),

with the failed nets given extra weight in the quench cost: their airwire
length and any crossing they take part in.
High-pin-count parts are excluded from targeting (--max-target-pins) - moving
a resistor that anchors a blocker net is low-risk; dragging a 144-pin QFP to
fix one net is how placements get destroyed.

The new placement is accepted only if a real re-route improves
(failures, router iterations), otherwise it is reverted and the next round
widens the displacement cap.

Usage:
  python place_route_loop.py input.kicad_pcb output.kicad_pcb \
      --route-args '--nets "/*" "Net-*" --track-width 0.2 ...' \
      [quench options]
"""
from __future__ import annotations
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)

import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys

from kicad_parser import parse_kicad_pcb
import routing_defaults as defaults
from placement.portfolio import copy_siblings
from placement.groups import GroupError, derive_groups, describe, parse_sources
from placement.cli_gates import (add_board_state_args, add_intent_arg,
                                 add_lock_advisor_args, add_tidiness_args)
from placement.quench import quench
from placement.writer import write_placed_output
from placement.diagnosis import (TOP_K as DIAGNOSIS_TOP_K,
                                 format_text as diagnosis_format,
                                 to_json as diagnosis_to_json)
from placement.relocate import format_text as relocate_format

# The loop shells out to the route.py sitting NEXT TO THIS FILE. A bare
# relative 'route.py' only resolved when the caller's cwd happened to be the
# repo root, and the failure surfaced as the misleading "produced no
# JSON_SUMMARY" instead of "no such file" (#458).
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# route.py is in the SIBLING py_router/ since the placement split.
_ROUTE_PY = os.path.join(os.path.dirname(_SCRIPT_DIR), 'py_router', 'route.py')

# The one-or-two-summary reduction now lives in route_summary.py, so route.py
# --json-out and this loop cannot drift apart on what "the tally" means.
from route_summary import (merge_route_summaries, SUMMARY_RE as _SUMMARY_RE,
                           RECONCILE_ABORTED as _RECONCILE_ABORTED,
                           EFFORT_KEYS as _EFFORT_KEYS)

# route.py's own "I stopped on my budget" code. Imported, not literal 7, so
# this loop cannot drift from the tool it shells.


def _log_tail(log: str, lines: int = 15) -> str:
    """Last few log lines, so an error names the real failure instead of
    burying it behind a path the operator has to go open."""
    return ''.join(f"  | {ln}\n" for ln in log.splitlines()[-lines:])


def _run_route_cmd(cmd, log_file):
    """Launch route.py with stdout and stderr captured to log_file; return its
    exit code. Split out from run_route so the whole tally can be driven
    against a canned log without launching a child process."""
    with open(log_file, 'w', encoding='utf-8') as f:
        return subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode


def accept_score(cmd: str, placed: str, routed: str, json_file: str):
    """(score, note) from an external, spec-aware accept test. None = reject.

    better() compares failures then iterations, and that is all it CAN compare:
    it has no way to see a max-length rule, a via ban, a required track width or
    a decap-proximity limit. On a board with a real specification those are
    exactly the constraints that decide whether a placement got better, and a
    loop blind to them will accept a round that broke one.

    Reworking better() is out of scope by its own comment, so this does not
    touch it -- it BYPASSES it when the operator supplies a judge:

        CMD <placed.kicad_pcb> <routed.kicad_pcb> <route.json>

    The judge prints one line `SCORE=<float>`, LOWER IS BETTER. A non-zero exit,
    a missing SCORE line, or an unparseable number all mean REJECT: a judge that
    cannot answer is not evidence that the round was good.

    Post-route mirror of _ratsnest_screen's pre-route veto -- same shape, same
    contract of reporting the numbers behind every decision.
    """
    argv = shlex.split(cmd) + [placed, routed, json_file or '']
    try:
        r = subprocess.run(argv, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
    except OSError as e:
        return None, f"accept-cmd could not run: {e}"
    if r.returncode != 0:
        detail = (r.stdout or r.stderr or '').strip().splitlines()
        return None, (f"accept-cmd exit {r.returncode}"
                      + (f": {detail[-1][:160]}" if detail else ''))
    m = re.search(r'^SCORE=([-+0-9.eE]+)', r.stdout or '', re.M)
    if not m:
        return None, 'accept-cmd printed no SCORE= line'
    try:
        val = float(m.group(1))
    except ValueError:
        return None, f"accept-cmd SCORE= unparseable: {m.group(1)!r}"
    return val, f"score {val:g}"


def run_route(pcb_file: str, routed_file: str, route_args: str, log_file: str,
              json_file: str = None, extra_targets=None, *,
              keep_blocker_cells: bool = False):
    """Run route.py and return its metrics dict.

    `json_file` asks route.py to also write its MERGED summary there
    (route.py --json-out). The loop does not read it back -- it already has the
    metrics -- but --accept-cmd is handed the path, so an external judge sees
    the same tally the loop is using instead of re-scraping the log. Skipped
    when the operator already put --json-out in --route-args: that
    destination is theirs to choose.
    """
    # Absolute path to the sibling route.py, so the loop runs from any cwd.
    # No cwd= override: route.py resolves its own assets from __file__, and
    # relative paths inside --route-args (--net-clearances foo.json) must keep
    # resolving against the OPERATOR's cwd, which a cwd= would silently break.
    # -X utf8 mirrors how the test suite invokes route.py.
    _extra = shlex.split(route_args)
    if json_file and '--json-out' not in _extra:
        _extra += ['--json-out', json_file]
    cmd = [sys.executable, '-X', 'utf8', _ROUTE_PY, pcb_file, routed_file] + _extra
    rc = _run_route_cmd(cmd, log_file)
    # errors='replace': route.py forces its own stream to UTF-8, but a cp1252
    # default locale on the READING side would raise on the first non-ASCII
    # glyph and lose the whole round.
    with open(log_file, encoding='utf-8', errors='replace') as f:
        log = f.read()
    if rc != 0:
        # route.py exits 0 even when nets fail; its other deliberate non-zero
        # exit is "No nets matched the given patterns!". So non-zero means a
        # crash, an unreadable board or a --route-args typo, none of which is
        # a routing result.
        raise RuntimeError(f"route.py exited {rc} (see {log_file})\n"
                           + _log_tail(log))

    summary = merge_route_summaries(log)
    if summary is None:
        raise RuntimeError(
            f"route.py produced no JSON_SUMMARY (see {log_file})\n"
            + _log_tail(log))
    # A PARTIAL run is exactly as fatal as no summary: a `complete: false`
    # summary PARSES, so without this the loop would consume an unfinished
    # run's numbers as if they were a whole board's -- a quieter and worse
    # failure than a missing summary. Nothing produces one today (route.py has
    # no self-budget), but a partial that ever appears must not pass silently.
    # `.get('complete', True)` so ordinary logs are unaffected.
    if not summary.get('complete', True):
        raise RuntimeError(
            f"route.py produced a PARTIAL JSON_SUMMARY "
            f"(status={summary.get('status')!r}) -- its tallies are not a "
            f"whole-board result.\nsee {log_file}\n"
            + _log_tail(log))

    return metrics_from_summary(summary, log, extra_targets,
                                keep_blocker_cells=keep_blocker_cells)


def _copy_blocker_report(jb):
    """Detach `summary['blockers']` from the caller's dict without reshaping it.

    Two levels deep, which is how deep the structure goes: the per-failed-net
    entry and its `blocked_by` items. An entry that carries no `blocked_by`
    key does NOT gain an empty one -- adding it would be normalisation, and a
    consumer must be able to tell "attributed nothing" from "did not say".
    """
    if jb is None:
        return None
    out = []
    for e in jb:
        if not isinstance(e, dict):
            out.append(e)
            continue
        c = dict(e)
        bb = c.get('blocked_by')
        if isinstance(bb, list):
            c['blocked_by'] = [dict(b) if isinstance(b, dict) else b
                               for b in bb]
        out.append(c)
    return out


def metrics_from_summary(summary: dict, log: str = '',
                         extra_targets=None, *,
                         keep_blocker_cells: bool = False) -> dict:
    """Round metrics from an already-merged JSON_SUMMARY.

    Split out of run_route (#431) so a renderer can caption a recorded round
    from its `loop_roundN_route.log` using THIS arithmetic rather than a second
    implementation that drifts. Pure: no subprocess, no file IO. `log` is only
    the pre-#409 blocker fallback.

    `keep_blocker_cells` (#553) adds ONE key, `blocker_report`, and changes
    nothing else. The seven-key form is what `write_round_sidecar` serialises
    verbatim into `loop_round{N}.json`, so an unconditional key would change
    the sidecar bytes of every run that never asked for #553. Off by default;
    `--target-select diagnosis` is what turns it on.

    `blocker_report` is a COPY of `summary['blockers']`, or **None** when that
    key was absent. It is deliberately raw: this function does no arithmetic on
    it, because `placement.diagnosis.blocker_evidence` already folds it and a
    second fold here is the drift #431 split this function out to prevent. Do
    not add a count alongside it -- ask `blocker_evidence`.

    WHAT `None` ACTUALLY MEANS, which is not what it looks like. `route.py`
    writes the key only `if blockers_report:`, so it is OMITTED both on a
    pre-#409 log AND on a modern run that attributed nothing. So `None` reads
    "no structured attribution reached us" -- and it is exactly then that
    `blockers` above comes from the whole-log regex, which scrapes TRANSIENT
    blocker lines for nets that later routed. `None` is therefore the signal
    that those names are not evidence, which is why the diagnosis skips the
    signal outright rather than ranking them. `[]` is a narrower case
    (`route_summary.merge_summaries` after reconciliation cleared every
    failure) and stays distinguishable.

    A COPY, not the caller's list: `write_round_sidecar` serialises this dict,
    and a consumer that sorted or annotated the report in place would write
    through into the summary the loop is still holding.

    Why carry it rather than re-read `work/loop_round{N}_route.json`: run_route
    adds `--json-out` only when the operator did not put their own in
    `--route-args`, so on a perfectly legal invocation that file is never
    written. The merged summary is in hand here.
    """
    failed_nets = list(summary.get('failed_single', []))
    # Nets the CALLER named as the thing to work on, whether or not the router
    # failed them (#549). Without this the loop's entire target set comes from
    # `failed_single` + `failed_multipoint`, so a board where EVERY NET ROUTES
    # but a spec clause is violated -- a maximum length, a via ban, a required
    # width -- yields an empty target list and the loop has nothing to move. It
    # is not that it moves the wrong parts; it does not run. Pair with
    # `--accept-cmd`, which is the other half: one supplies the targets, the
    # other supplies the gradient, and neither alone lets the loop chase a
    # requirement the router is happy with.
    failed_nets += [n for n in (extra_targets or []) if n not in failed_nets]
    # failed_multipoint entries are dicts {net_name, failed_pads}; keep just the
    # name so failed_nets is uniformly net-name strings (downstream uses them as
    # dict keys -> a dict here raises "unhashable type: 'dict'").
    failed_nets += [d['net_name'] if isinstance(d, dict) else d
                    for d in summary.get('failed_multipoint', [])]
    mp_deficit = (summary.get('multipoint_pads_total', 0)
                  - summary.get('multipoint_pads_connected', 0))
    # open_single: kept-result nets with disconnected pads. Their names are
    # already in failed_nets via failed_multipoint; the COUNT needs its own
    # term because a non-multipoint open net contributes nothing to either
    # failed_single or the pad deficit (the emitter excludes multipoint nets
    # from the key, so this cannot double-count against mp_deficit).
    failures = (len(summary.get('failed_single', []))
                + len(summary.get('open_single', []))
                + mp_deficit)

    # Blocker nets from frontier diagnostics. Prefer the structured
    # JSON_SUMMARY 'blockers' key (#409): the last-wins attribution of nets
    # still failed at END of run, capped 10/net -- a narrower, more targeted
    # move-candidate set. Fallback for older logs: scrape every transient
    # "  1. /MD1: 46 (31.7%) ..." line in the whole log (includes blockers of
    # nets that later routed and every N-retry re-analysis).
    jb = summary.get('blockers')
    if jb is not None:
        # An explicit empty list means "structured emitter, nothing left to
        # attribute" (e.g. the reconciliation recovered every failure) -- do
        # NOT regress to the whole-log regex, which would scrape transient
        # blocks of nets that later routed.
        blockers = {b['net'] for e in jb for b in e.get('blocked_by', [])}
    else:
        blockers = set(re.findall(r'^\s+\d+\.\s+(\S+?):\s+\d+\s+\(', log, re.M))

    out = {
        'failures': failures,
        'failed_nets': failed_nets,
        'blockers': sorted(blockers),
        'iterations': summary.get('total_iterations', 0),
        'vias': summary.get('total_vias', 0),
        # #409 follow-up: pad-pair routability tallies (PRR = connected/total
        # downstream); 0/0 on logs from routers without the keys. Report-only
        # here -- better() still compares failures/iterations (#458's scope).
        'pad_pairs_connected': summary.get('pad_pairs_connected', 0),
        'pad_pairs_total': summary.get('pad_pairs_total', 0),
    }
    if keep_blocker_cells:
        # The cell counts the seven-key form discards above, carried raw. No
        # coordinates exist anywhere in this JSON
        # (blocking_analysis.blocking_info_to_dict serialises counts only), and
        # none is invented here.
        out['blocker_report'] = _copy_blocker_report(jb)
    return out


def write_round_sidecar(work: str, rnd: int, *, board: str, routed: str,
                        parent: str, accepted: bool, screened: bool = False,
                        targets=None, groups=None, moved=None, metrics=None,
                        diagnosis=None, relocation=None) -> str:
    """Record one round as `loop_round{N}.json` next to its board (#431).

    The boards alone are NOT a chain: a REJECTED loop_round2.kicad_pcb sits
    between rounds 1 and 3 in both name and mtime order, and anything that
    walks the directory would animate it as though it had been kept. `parent`
    is the field that fixes that -- it names the board this round was actually
    derived from, which is the last ACCEPTED one, not N-1.

    An artifact rather than a callback, deliberately: it is how the rest of this
    repo communicates between tools (make_movie reads directories), it needs no
    hook in this monolithic main(), and both the renderer and the movie camera
    can consume it without either importing the other.
    """
    path = os.path.join(work, f'loop_round{rnd}.json')
    doc = {
        'schema': 1,
        'round': rnd,
        'board': os.path.basename(board) if board else None,
        'routed': os.path.basename(routed) if routed else None,
        'parent': os.path.basename(parent) if parent else None,
        'accepted': bool(accepted),
        'screened': bool(screened),
        'targets': sorted(targets or []),
        'groups': {k: sorted(v) for k, v in sorted((groups or {}).items())},
        'moved': moved or [],
        'metrics': metrics or {},
    }
    # #554: same rule -- a run without --relocate keeps the key set it had.
    if relocation is not None:
        doc['relocation'] = relocation
    # #553: added CONDITIONALLY, never as a None placeholder -- a `pins` run's
    # sidecar keeps the eleven keys it has always had, byte for byte.
    if diagnosis is not None:
        doc['diagnosis'] = diagnosis
    try:
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
    except OSError as e:      # best-effort: never lose a round over a sidecar
        print(f"  (round sidecar not written: {e})")
        return ''
    return path


def relocate_round(pcb_data, pcb_file, blocks, *, block=None, refs=None,
                   ignore_nets=None, max_corridor_mm=None, clearance=None):
    """Propose ONE bounded block relocation on this round's board (#554).

    Returns a `relocate.Relocation`, always -- a refusal is a result and carries
    its own named reason, never a count.

    Two things this has to get right, and both are invisible if it does not:

    * The state is built with **`move_refs=None`**. `QuenchState.__init__` locks
      every ref OUTSIDE `move_refs`, and this loop passes its target selection as
      `move_refs` a few lines later. Handed such a state the solve would have no
      free neighbours, would answer `no_room_at_any_dose` on every board, and
      would look exactly like a correct negative result.
    * `build_neighbor_lists` is never called, so the travel budget stays infinite
      and the outline gate's cached reach stays conservative -- which is why the
      pruning caveat in `quench._group_offsets` does not apply here, and why
      #554's three named invariants are not engaged rather than "unlocked".
    """
    import fnmatch
    import pose_score
    from placement import diagnosis as _diag, relocate as _rel
    from placement import routability as _routab

    pcb = pcb_data
    state = pose_score.make_state(pcb, pcb_file, clearance=clearance or 0.25)
    ids = _diag.ignore_net_ids(pcb, ignore_nets or [])
    units_src = dict(blocks or {})
    if refs:
        named = sorted({r for pat in refs
                        for r in fnmatch.filter(sorted(pcb.footprints), pat)})
        if len(named) < 2:
            return _rel.Relocation(
                refusal='no_diagnosed_block: --relocate-refs matched %d part(s)'
                        % len(named))
        # MERGE, never replace. Replacing the derived dict left every OTHER
        # block as loose singletons, so the pass tore them apart while its own
        # docstring promised "so does every untouched decap cluster": measured
        # on tigard, the refs arm split `decap:U5` in half and bought a 2x
        # larger dose partly by doing so. The named refs are removed from any
        # block they were in -- they are the unit now -- and the rest keep
        # their rigidity.
        chosen = 'refs:%s' % ','.join(refs)
        named_set = set(named)
        units_src = {k: [r for r in v if r not in named_set]
                     for k, v in units_src.items()}
        units_src = {k: v for k, v in units_src.items() if len(v) >= 2}
        units_src[chosen] = named
        block = chosen
    if block is None:
        ranked = sorted(_routab.block_displacements(state, units_src,
                                                    ignore_net_ids=ids),
                        key=lambda d: (-d.distance_mm, d.block))
        if not ranked:
            return _rel.Relocation(
                refusal='block_has_no_target: no derived block has a foreign-pad '
                        'centroid to move toward')
        block = ranked[0].block
    disp = {d.block: d for d in _routab.block_displacements(
        state, units_src, ignore_net_ids=ids)}.get(block)
    if disp is None:
        return _rel.Relocation(
            block=block,
            refusal='block_has_no_target: %s connects to nothing outside itself, '
                    'so it has no direction to move in' % block)
    direction = (disp.net_centroid[0] - disp.centroid[0],
                 disp.net_centroid[1] - disp.centroid[1])
    return _rel.relocate_block(state, units_src, block, direction,
                               want_mm=disp.distance_mm,
                               max_corridor_mm=max_corridor_mm,
                               clearance=clearance)


def moves_from_placements(parent_pcb, placements) -> list:
    """`[{reference, from:[x,y,rot], to:[x,y,rot]}]` for the movie's tween.

    quench() returns only the destination pose, so the source is read off the
    board the round started from. Also the degraded path when a work dir has no
    sidecars: `moves_between(a, b)` is this with both poses read from files.
    """
    out = []
    for p in placements or []:
        fp = parent_pcb.footprints.get(p['reference']) if parent_pcb else None
        if fp is None:
            continue
        out.append({'reference': p['reference'],
                    'from': [round(fp.x, 4), round(fp.y, 4),
                             round(fp.rotation or 0.0, 3)],
                    'to': [round(p['new_x'], 4), round(p['new_y'], 4),
                           round(p.get('new_rotation') or 0.0, 3)]})
    return sorted(out, key=lambda m: m['reference'])


def nets_to_refs(pcb_data, net_names, max_pins, locked_patterns):
    """Map net names to the movable component refs that own their pads."""
    import fnmatch
    name_to_id = {net.name: nid for nid, net in pcb_data.nets.items()}
    refs = set()
    for name in net_names:
        nid = name_to_id.get(name)
        if nid is None:
            continue
        for pad in pcb_data.nets[nid].pads:
            refs.add(pad.component_ref)
    out = set()
    for ref in refs:
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            continue
        pins = len([p for p in fp.pads if p.net_id > 0])
        if pins > max_pins:
            continue
        if locked_patterns and any(fnmatch.fnmatch(ref, p)
                                   for p in locked_patterns):
            continue
        # #829: never offer a footprint that draws the board outline as a
        # target. QuenchState would lock it anyway, so this stops the loop
        # naming a target it cannot move rather than preventing a move.
        if getattr(fp, 'owns_board_outline', False):
            continue
        out.add(ref)
    return out


def _locked_out(pcb_data, locked_patterns):
    """Refs an operator's --lock globs exclude. The LOCK rule only.

    Deliberately NOT `nets_to_refs`'s whole filter: that also drops anything
    over `--max-target-pins`, and bypassing the pin cap is the entire point of
    #553 -- "the part that needs to move is never a passive". So a diagnosis
    may offer a 100-pin IC the pin selector would have refused, and that is the
    feature. It may not offer a part the operator locked, and that is the rule.
    The ranking says what SHOULD move; the lock says what MAY.

    #829 adds a second source to the same "what MAY" question: a footprint that
    draws the board's own outline. Nothing here can move it -- QuenchState locks
    it downstream, and its lock is monotone -- so this is about HONESTY rather
    than safety: without it the loop names a target it will then silently fail
    to move, and the reader is left looking for a lock that is not in the file.
    """
    import fnmatch
    if not locked_patterns:
        return set()
    return {r for r in (pcb_data.footprints or {})
            if any(fnmatch.fnmatch(r, p) for p in locked_patterns)}


def _outline_owners(pcb_data):
    """Refs excluded because they draw the board outline (#829).

    A SEPARATE set from `_locked_out`, deliberately. Folding the two together
    made the caller's message -- "every one of the N diagnosed part(s) matches
    --lock" -- lie on a run with no `--lock` at all, sending the reader hunting
    for a flag that was never passed. This addition is about honesty; spending
    it on a wrong message would be self-defeating.
    """
    return {r for r, fp in (pcb_data.footprints or {}).items()
            if getattr(fp, 'owns_board_outline', False)}


def block_census(pcb_data) -> str:
    """One line per group source and what it derives on THIS board.

    Printed once, before round 0, because `--group-by auto` derives NOTHING on
    most of the tracked corpus -- 5 of the 6 boards `docs/placement-predictors.md`
    grades on are flat schematics whose `(path ...)` entries are all distinct
    top-level uuids, and no corpus board carries a KiCad `(group ...)` at all.
    An operator who asked for a signal that cannot run on their board should
    read that here rather than infer it from a selection that never changes.
    """
    from placement.groups import SOURCES
    out = []
    for src in SOURCES:
        try:
            got = derive_groups(pcb_data, (src,))
        except Exception as e:                             # noqa: BLE001
            out.append(f'    {src:<10} unavailable ({type(e).__name__})')
            continue
        n = sum(len(v) for v in got.values())
        out.append(f'    {src:<10} {len(got):>3} block(s), {n:>4} part(s)'
                   + ('' if got else '   <- nothing on this board'))
    return '\n'.join(out)


def diagnose_round(pcb_data, pcb_file, blocks, metrics, *, ignore_nets=None,
                   budget=None, top_k=None):
    """One round's diagnosis. Always a `placement.diagnosis.Diagnosis`.

    Split out of main() so it can be exercised without a router, and so the
    expensive parts -- a QuenchState and two legality graders -- are visible in
    one place. Returns a `placement.diagnosis.Diagnosis`.
    """
    from placement import diagnosis as _diag
    state = _diag.make_state(pcb_data, pcb_file)
    legality = _diag.legality_defects(pcb_data, pcb_file=pcb_file)
    return _diag.diagnose(
        state, pcb_data, blocks,
        blocker_report=metrics.get('blocker_report'),
        legality=legality,
        ignore_net_ids=sorted(_diag.ignore_net_ids(pcb_data, ignore_nets)),
        budget=budget,
        top_k=_diag.TOP_K if top_k is None else top_k)


def better(a, b):
    """Is metrics a better than b? Failures first, then iterations."""
    if a['failures'] != b['failures']:
        return a['failures'] < b['failures']
    return a['iterations'] < b['iterations'] * 0.95


def _ratsnest_screen(before, after, pct):
    """(skip, note) -- should this candidate skip its routing run? (#504)

    Routing is the honest judge but an expensive one, often minutes per round.
    A candidate whose ratsnest got clearly WORSE than the board it came from is
    very unlikely to route better, so it is not worth paying for.

    Screens on `crossings` and `hpwl` only. Both are unweighted -- crossings is
    a raw count by contract, hpwl is pure pad geometry -- whereas `length` and
    `total` are scaled by the per-round net_weights this loop sets from
    --failed-net-weight. (before/after share the weights within one quench call,
    so length is safe to REPORT; it just is not a stable thing to threshold on.)

    pct <= 0 disables the screen, which is the default: skipping a placement
    that would in fact have won is a real cost, so this is opt-in and every
    decision is logged with its numbers.
    """
    if not pct or pct <= 0 or not before or not after:
        return False, ""
    worse = []
    note = []
    for key, label in (('crossings', 'crossings'), ('hpwl', 'hpwl')):
        b, a = before.get(key), after.get(key)
        if b is None or a is None:
            continue
        delta = (a - b) / b * 100.0 if b else (100.0 if a else 0.0)
        note.append(f"{label} {b:g}->{a:g} ({delta:+.1f}%)")
        if delta > pct:
            worse.append(f"{label} {delta:+.1f}%")
    txt = ", ".join(note)
    if worse:
        return True, f"{txt}  [regressed > {pct:g}%: {', '.join(worse)}]"
    return False, txt


def main():
    parser = argparse.ArgumentParser(
        description="Router-in-the-loop placement repair.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input_file")
    # Optional so the report-only --suggest-locks mode does not demand an
    # output board it never writes; required post-parse for a real run.
    parser.add_argument("output_file", nargs="?")
    parser.add_argument("--route-args", default=None,
                        help="Arguments passed to route.py (quoted string)")
    parser.add_argument("--target-nets", nargs="*", default=None, metavar="NET",
                        help="Net names to treat as targets EVEN IF the router "
                             "routed them. The loop's move candidates otherwise "
                             "come only from failed nets, so a board where "
                             "everything routes but a SPEC clause is violated "
                             "(a maximum length, a via ban, a required width) "
                             "gives it an empty target list and it does nothing. "
                             "Pair with --accept-cmd: this supplies the targets, "
                             "that supplies the gradient.")
    parser.add_argument("--rounds", type=int, default=5,
                        help="Max repair rounds (default: 5)")
    parser.add_argument("--max-displacement", type=float, default=3.0,
                        help="Initial displacement cap per round in mm "
                             "(default: 3; widened 1.5x after a rejected "
                             "round - nudges only, never swaps)")
    parser.add_argument("--swap-max-displacement", type=float, default=None,
                        help="Displacement cap for same-footprint swaps in mm; "
                             "must not exceed --max-displacement and is NOT "
                             "widened between rounds (default: the initial "
                             "--max-displacement)")
    parser.add_argument("--max-target-pins", type=int, default=40,
                        help="Don't move parts with more connected pins than "
                             "this (default: 40)")
    parser.add_argument("--failed-net-weight", type=float, default=3.0,
                        help="Cost multiplier for failed nets: scales their "
                             "airwire length and any crossing they take part "
                             "in (default: 3.0)")
    parser.add_argument("--target-select", choices=('pins', 'diagnosis'),
                        default='pins',
                        help="How to choose which parts the quench may move. "
                             "'pins' (default, unchanged): the pad owners of "
                             "the failed and blocker nets, minus anything over "
                             "--max-target-pins. 'diagnosis' (#553): rank "
                             "blocks and loose parts on three signals -- "
                             "connectivity-centroid displacement, blocked "
                             "cells owned, and legality defect pairs -- and "
                             "take the round-robin union of their top-k. "
                             "It also changes what --group-by hands the "
                             "quench as a rigid body: diagnosis mode "
                             "FREEZES the blocks before round 0, because "
                             "netprefix and decap membership is a "
                             "function of poses the optimizer is moving. "
                             "NO MEASUREMENT SHOWS 'diagnosis' ROUTES BETTER "
                             "THAN 'pins'; see py_placer/placement/"
                             "diagnosis.py. Requires --group-by for the "
                             "displacement signal; the other two need none.")
    parser.add_argument("--diagnosis-top-k", type=int, default=DIAGNOSIS_TOP_K,
                        metavar="K",
                        help=f"How many candidates each diagnosis signal "
                             f"offers per sweep (default: {DIAGNOSIS_TOP_K}). "
                             f"A report size, not a threshold. NOT usually "
                             f"what bounds the move set: the loop budgets "
                             f"each round at the number of parts the pin "
                             f"filter would have offered, and that budget "
                             f"is normally the binding constraint. And a "
                             f"BLOCK IS ADDED WHOLE, so the budget is a floor "
                             f"rather than a cap: measured at budget 8, "
                             f"glasgow_revC selects 67 parts under --group-by "
                             f"auto, because its one derivable block has 68 "
                             f"members. The overshoot is reported")
    parser.add_argument("--group-by", default="none",
                        help="Move blocks of parts as one rigid body. Comma "
                             "list of: kicad, sheet, netprefix, decap; 'auto' = "
                             "kicad,sheet (default: none). A targeted part pulls "
                             "in its whole block, so a block is no longer "
                             "half-frozen because its IC exceeds "
                             "--max-target-pins")
    parser.add_argument("--relocate", action="store_true",
                        help="Before each round's quench, propose ONE bounded "
                             "block relocation (#554): move the diagnosed block "
                             "toward the centroid of what it connects to, "
                             "letting neighbours yield while their relative "
                             "order is held as a hard constraint, minimising "
                             "total displacement. DEFAULT OFF, AND THE ROUTED "
                             "A/B WENT AGAINST IT: over 3 evidence cells it "
                             "recovered damage on all 3 (median delta +0.57), "
                             "but on only 2 boards where the acceptance rule "
                             "counts 3, and place_route_loop with the pin gate "
                             "lifted reached a STRICTLY BETTER routed result on "
                             "2 of those 3 (tests/stress/"
                             "block_relocation_study.py). What IS measured is "
                             "the mechanism: over 24 blocks on the 9 corpus "
                             "boards that have a measurable block, letting "
                             "neighbours yield bought >= 1mm more travel than "
                             "freezing them on 11 of them, spanning 6 boards, "
                             "max 16.66mm -- though the MEDIAN cell reaches "
                             "1.03mm against a median want of 10.36mm (tests/"
                             "stress/relocation_reach.py). The round's own "
                             "re-route is the judge, and a rejected round "
                             "reverts byte for byte")
    parser.add_argument("--relocate-block", default=None, metavar="NAME",
                        help="Relocate this derived block by name instead of "
                             "the most displaced one. Names come from "
                             "--list-groups / the group-source census")
    parser.add_argument("--relocate-refs", nargs="+", default=None,
                        metavar="GLOB",
                        help="Relocate an EXPLICIT set of refs as one rigid "
                             "body, bypassing block derivation entirely. This "
                             "is what separates 'the relocation worked' from "
                             "'the diagnosis picked right' in a study, since "
                             "--target-select diagnosis is itself a measured "
                             "null (#553)")
    parser.add_argument("--relocate-max-corridor", type=float, default=None,
                        metavar="MM",
                        help="Refuse a relocation whose neighbours must travel "
                             "more than this in total, and try a shorter dose. "
                             "Unset = no budget, and the corridor is reported "
                             "rather than bounded. Worth setting: measured on "
                             "sonde_u, relocating a 3-part block 8.96mm moved "
                             "17 neighbours a combined 53.92mm -- legal, "
                             "minimal for that dose, and a bigger disturbance "
                             "than #554's 'move only diagnosed blocks' asks for")
    parser.add_argument("--accept-cmd", default=None, metavar="CMD",
                        help="External accept test, run after each round's route as "
                             "CMD <placed> <routed> <route.json>; print one line "
                             "SCORE=<float>, LOWER IS BETTER. Replaces the built-in "
                             "failures-then-iterations comparison, which cannot see a "
                             "max-length rule, a via ban, a required width or a "
                             "decap-proximity limit -- on a spec'd board those are what "
                             "decide whether a placement got better. A non-zero exit or "
                             "a missing SCORE line REJECTS the round.")
    parser.add_argument("--ratsnest-screen", type=float, default=0.0,
                        help="Skip the routing run when a candidate placement's "
                             "airwire crossings or HPWL regress by more than "
                             "this percent against the board it came from -- a "
                             "cheap pre-route filter (0 = disabled, the default)")
    parser.add_argument("--step", type=float, default=0.5,
                        help="Candidate grid step in mm (default: 0.5)")
    parser.add_argument("--length-weight", type=float, default=0.3)
    parser.add_argument("--crossing-penalty", type=float, default=30.0)
    parser.add_argument("--halo-base", type=float, default=0.5)
    parser.add_argument("--halo-coef", type=float, default=0.15)
    parser.add_argument("--halo-weight", type=float, default=2.0)
    parser.add_argument("--edge-halo", type=float, default=2.0)
    parser.add_argument("--edge-weight", type=float, default=2.0)
    parser.add_argument("--clearance", type=float, default=defaults.CLEARANCE)
    parser.add_argument("--board-edge-clearance", type=float, default=0.55)
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP)
    parser.add_argument("--ignore-nets", nargs="+", default=None)
    parser.add_argument("--lock", nargs="+", default=None)
    parser.add_argument("--no-rotate", action="store_true",
                        help="Disable rotation moves")
    parser.add_argument("--no-swap", action="store_true",
                        help="Disable same-footprint swap moves")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print each accepted quench move and the "
                             "per-pass swap-capped count")
    parser.add_argument("--work-dir", default=None,
                        help="Directory for intermediate files "
                             "(default: alongside output)")
    # ON BY DEFAULT. The loop already writes every round's board and sidecar,
    # so the movie is the only artifact that shows WHY a round was taken -- and
    # a flag nobody remembers to pass is a flag nobody sees. It renders from
    # what is already on disk after the loop has finished, so it cannot change
    # the routing result, and a failure to render is caught and reported rather
    # than failing the run.
    parser.add_argument("--movie", nargs='?', const='', default='',
                        metavar="OUT",
                        help="render a movie of the whole repair when the loop "
                             "finishes, with the placement camera on (#431): "
                             "overview -> zoom to each round's moved parts -> "
                             "pan when the work moves -> then play the moves. "
                             "ON BY DEFAULT; pass --no-movie to skip it. "
                             "Default path: <work-dir>/placement.mp4")
    parser.add_argument("--no-movie", dest='movie', action='store_const',
                        const=None,
                        help="skip the end-of-run movie (see --movie)")
    parser.add_argument("--movie-tween", type=int, default=10, metavar="N",
                        help="frames per placement glide in --movie; 0 = no "
                             "glide, cut straight to the new placement")
    add_board_state_args(parser)
    add_intent_arg(parser)
    add_lock_advisor_args(parser)
    add_tidiness_args(parser)

    # #597: nets named like negative rails (-3V3) must parse as VALUES.
    # argparse's negative-number matcher differs by CPython version, so a
    # manifest that replays on a dev machine dies in a 3.12 container.
    args = __import__("cli_nets").pin_dash_digit_values(parser).parse_args()

    # --suggest-locks is report-only, so it must not demand a route spec it
    # never uses. Every other run still requires one.
    if not args.suggest_locks and not args.route_args:
        parser.error("--route-args is required (except with --suggest-locks)")
    if not args.suggest_locks and not args.output_file:
        parser.error("output_file is required (except with --suggest-locks)")

    # #702: the intent is loaded BEFORE record_invocation, for the reason
    # place_optimize gives -- an exit 2 on an unreadable intent touches no
    # file, and a manifest carrying a command that produced nothing leaves the
    # next step's input made by nothing. It needs only `args`, so it is safe
    # this early; the board-dependent RESOLVE stays down beside the
    # board-state gates.
    from placement.cli_gates import load_intent_or_exit
    _intent, _rc = load_intent_or_exit(args)
    if _rc:
        return _rc

    # BEFORE record_invocation, for the reason stated just above for the
    # intent: an exit 2 must touch no file, and a manifest carrying a command
    # that produced nothing leaves the next step's input made by nothing.
    # These two need only `args`, so nothing forced them to sit after it.
    #
    # The numeric checks BELOW still do, and that is pre-existing rather than
    # made worse here: moving them has its own blast radius, because a manifest
    # replaying a bad --max-displacement gets a recorded line today and would
    # stop getting one. So the rule as stated is not yet what the whole
    # function does -- said here rather than left for a reader to notice.
    try:
        group_sources = parse_sources(args.group_by)
    except GroupError as exc:
        parser.error(str(exc))
    # #553. Only the DISPLACEMENT signal needs blocks; the other two rank loose
    # parts, so a source that derives NOTHING on this board is a warning about
    # a lost signal (printed by `block_census` before round 0), not a refusal.
    # `--group-by none` with `--target-select diagnosis` is different: it is a
    # contradiction the operator can only have typed by accident.
    #
    # `--suggest-locks` is report-only -- it advises, prints and exits without
    # deriving a group, running a diagnosis or routing anything -- so refusing
    # it for the shape of a flag it never reads is a refusal with no subject.
    if (args.target_select == 'diagnosis' and not group_sources
            and not args.suggest_locks):
        parser.error(
            "--target-select diagnosis ranks derived BLOCKS on connectivity "
            "displacement, and --group-by none derives none. Pass a source "
            "list -- 'auto,netprefix,decap' is the one that derives something "
            "on most tracked boards; bare 'auto' (kicad,sheet) derives NOTHING "
            "on five of the six this repo grades placement on -- or drop "
            "--target-select. (The other two signals, blocked cells and "
            "legality pairs, do rank loose parts, so a source that derives "
            "nothing is a warning printed before round 0, not this refusal.)")
    if args.diagnosis_top_k < 1:
        parser.error("--diagnosis-top-k must be >= 1")

    # #554. Refused BEFORE round 0's route, so a bad invocation costs no routing.
    if args.relocate and not group_sources and not args.relocate_refs \
            and not args.suggest_locks:
        parser.error(
            "--relocate moves a derived BLOCK, and --group-by none derives "
            "none. Pass a source list -- 'auto,netprefix,decap' derives "
            "something on most tracked boards, while bare 'auto' (kicad,sheet) "
            "derives NOTHING on five of the six this repo grades placement on "
            "-- or name the parts yourself with --relocate-refs.")
    for _flag, _val in (('--relocate-block', args.relocate_block),
                        ('--relocate-refs', args.relocate_refs),
                        ('--relocate-max-corridor',
                         args.relocate_max_corridor)):
        if _val is not None and not args.relocate:
            parser.error("%s only applies with --relocate" % _flag)
    if args.relocate_block and args.relocate_refs:
        parser.error("--relocate-block names a DERIVED block and "
                     "--relocate-refs bypasses derivation; pass one or neither")
    if args.relocate_max_corridor is not None and args.relocate_max_corridor < 0:
        parser.error("--relocate-max-corridor is a total travel in mm; "
                     "negative is not a smaller budget")

    if not args.suggest_locks:
        try:
            from redo_record import record_invocation
            record_invocation()
        except Exception:
            pass

    if args.max_displacement < 0:
        parser.error("--max-displacement must be >= 0")
    if args.ratsnest_screen < 0:
        parser.error("--ratsnest-screen must be >= 0 (0 disables it)")
    if args.swap_max_displacement is not None:
        if args.swap_max_displacement < 0:
            parser.error("--swap-max-displacement must be >= 0")
        if args.swap_max_displacement > args.max_displacement:
            parser.error("--swap-max-displacement must not exceed "
                         "--max-displacement")

    # Report-only: advise, print, exit. Writes no board, routes nothing, and
    # deliberately runs BEFORE the expensive round-0 route.
    if args.suggest_locks:
        from placement.lock_advisor import advise_locks, format_text, to_json
        advice = advise_locks(parse_kicad_pcb(args.input_file), args.input_file,
                              lock_patterns=args.lock,
                              min_confidence=args.lock_confidence,
                              edge_margin=args.lock_edge_margin,
                              use_globs=args.suggest_locks_globs)
        print(format_text(advice))
        if args.suggest_locks_json:
            with open(args.suggest_locks_json, 'w', encoding='utf-8') as f:
                json.dump(to_json(advice), f, indent=1, sort_keys=True)
            print(f"Wrote {args.suggest_locks_json}")
        print("JSON_SUMMARY: " + json.dumps(advice.tally(), sort_keys=True))
        return 0

    # Board-state gates (#431), BEFORE round 0. Round 0 routes the whole board,
    # so refusing here saves minutes-to-hours of A* that would fail everything
    # and then quench a pile.
    from placement.placement_state import gate_or_exit
    _pcb0 = parse_kicad_pcb(args.input_file)
    gate_or_exit(_pcb0, args.input_file,
                 'place_route_loop.py',
                 allow_unplaced=args.allow_unplaced,
                 allow_routed=args.allow_routed)

    # #702: resolved ONCE, here, and FROZEN for the whole run.
    #
    # Here, because a malformed intent has the same cost profile as the
    # board-state gates just above -- round 0 routes the whole board, and
    # refusing after that has already run wastes exactly what those gates
    # exist to save.
    #
    # Frozen, because block membership is a function of refs, globs and
    # derived groups, not of poses, so no round can legitimately change it --
    # and re-deriving under the optimizer's own moves is precisely the
    # non-stationarity that made `corridor_weight` NOT ADOPTED (quench.py:
    # "the optimizer minimises against corridors frozen at construction while
    # the grader re-derives them from the final poses"). A gate that re-elects
    # its own identity mid-run is not monotone.
    intent_gate = None
    if _intent is not None:
        from placement.cli_gates import resolve_intent_gate_for_cli
        intent_gate, _ = resolve_intent_gate_for_cli(
            _intent, _pcb0, group_sources, args.intent)

    # #553: in diagnosis mode the block universe is resolved ONCE, here, and
    # frozen -- the same non-stationarity argument the intent gate makes just
    # above. `netprefix` and `decap` derivation both read POSES, so a universe
    # re-derived each round is re-elected by the optimizer's own moves.
    #
    # The `pins` path deliberately keeps its per-round derivation: freezing it
    # would change the default, and the default must stay byte-identical.
    _keep_cells = args.target_select == 'diagnosis'
    frozen_blocks = None
    diagnosis_rounds = 0
    fallback_rounds = 0
    select_overlap = []
    fallback_reasons = []
    if args.target_select == 'diagnosis':
        print("Group sources on this board:")
        print(block_census(_pcb0))
        frozen_blocks = derive_groups(_pcb0, group_sources)
        if frozen_blocks:
            print(f"  frozen for the run: {describe(frozen_blocks)}")
        else:
            print("  WARNING: --group-by "
                  f"{args.group_by} derives NO block on this board, so the "
                  "connectivity-displacement signal cannot run. The blocked-"
                  "cell and legality signals rank loose parts and still can.")

    work = args.work_dir or os.path.dirname(os.path.abspath(args.output_file))
    os.makedirs(work, exist_ok=True)

    cur_file = os.path.join(work, 'loop_round0.kicad_pcb')
    shutil.copy(args.input_file, cur_file)
    # #441. This one is load-bearing rather than cosmetic: round 0 ROUTES this
    # copy, and a board without its .kicad_pro resolves the DRC floor from the
    # STOCK netclass instead of the one the board was built to. Every round of
    # the loop -- and the accept/reject decision that reads their failure counts
    # -- would then be measured at the wrong clearance.
    copy_siblings(args.input_file, cur_file)

    screened = 0
    print("Round 0: routing initial placement...")
    best = run_route(cur_file, os.path.join(work, 'loop_round0_routed.kicad_pcb'),
                     args.route_args, os.path.join(work, 'loop_round0_route.log'),
                     json_file=os.path.join(work, 'loop_round0_route.json'),
                     extra_targets=args.target_nets,
                     keep_blocker_cells=_keep_cells)
    # Round 0 is the unconditional baseline, so the judge is not a gate here --
    # it is scored only so later rounds have an incumbent to beat.
    best_score = None
    if args.accept_cmd:
        best_score, _note = accept_score(
            args.accept_cmd, cur_file,
            os.path.join(work, 'loop_round0_routed.kicad_pcb'),
            os.path.join(work, 'loop_round0_route.json'))
        print(f"  baseline accept-cmd: {_note}")
        if best_score is None:
            print('  WARNING: the judge could not score the BASELINE. Every round '
                  'it can score will now be accepted on its first success.')
    print(f"  failures={best['failures']} iterations={best['iterations']:,}"
          f" vias={best['vias']}")
    # Round 0 is the baseline: its own parent, nothing moved, always "accepted".
    write_round_sidecar(work, 0, board=cur_file,
                        routed=os.path.join(work, 'loop_round0_routed.kicad_pcb'),
                        parent=None, accepted=True, metrics=best)
    # Kept separately from `best`, which advances on every accept. Without it
    # the run cannot report what it started from, which is half of any delta.
    baseline = dict(best)
    accepted_rounds = 0
    # #554. `relocate_applied == 0` beside a non-zero `relocate_refused` is the
    # machine-readable statement that the flag did nothing this run -- the same
    # anti-degeneration device --target-select carries.
    relocate_applied = 0
    relocate_refused = 0
    relocate_reasons: list = []
    relocate_records: list = []
    rounds_run = 0

    max_disp = args.max_displacement
    # The swap cap is pinned to the BASE displacement and never widened (#458).
    # A rejected round widens the NUDGE radius, a local search around each
    # part's own seed that a real re-route then validates. Widening the swap
    # cap in lockstep turns a 3mm budget into a 15mm teleport after four
    # rejections, which is the #430 stranding failure coming back through the
    # loop. Swaps get one fixed budget for the whole run.
    swap_cap = (args.max_displacement if args.swap_max_displacement is None
                else args.swap_max_displacement)

    for rnd in range(1, args.rounds + 1):
        if best['failures'] == 0:
            print("No failures left - stopping.")
            break

        pcb_data = parse_kicad_pcb(cur_file)
        # ALWAYS computed, in both modes. In `pins` it is the selection; in
        # `diagnosis` it is the BUDGET -- the diagnosis is offered exactly the
        # number of parts the pin filter would have spent, so the two selectors
        # are the same size by construction -- and it is the change detector
        # that says whether the flag did anything at all.
        pins_targets = nets_to_refs(pcb_data,
                                    best['failed_nets'] + best['blockers'],
                                    args.max_target_pins, args.lock)
        targets = set(pins_targets)
        diag = None

        if args.target_select == 'diagnosis' and pins_targets:
            diag = diagnose_round(
                pcb_data, cur_file, frozen_blocks or {}, best,
                ignore_nets=args.ignore_nets, budget=len(pins_targets),
                top_k=args.diagnosis_top_k)
            print(diagnosis_format(diag))
            # An operator's --lock is never overridden by a diagnosis: the
            # ranking says what SHOULD move, the lock says what MAY. Resolved
            # ONCE per round -- putting the call in a comprehension condition
            # re-scans every footprint against every glob per selected ref.
            locked = _locked_out(pcb_data, args.lock)
            owners = _outline_owners(pcb_data)
            picked = {r for r in diag.selected if r not in locked | owners}
            why = ''
            if diag.degenerate:
                why = diag.fallback_reason()
            elif not picked:
                # A selection wholly inside --lock is a FALLBACK, not a
                # decision. Without this the round reached the "no movable
                # target parts" stop and ended the whole run, while the verdict
                # reported rounds_diagnosis=1 and rounds_fallback=0 -- the run
                # where the flag did the most damage reading as the run where
                # it worked.
                #
                # NAME THE SOURCE (#829). The two exclusions are reported
                # apart because the reader's next move differs: a --lock hit is
                # theirs to change, a board-outline owner is not, and blaming
                # --lock on a run that passed none sends them looking for a
                # flag that was never there.
                _byl = [r for r in diag.selected if r in locked]
                _byo = [r for r in diag.selected if r in owners
                        and r not in locked]
                parts = []
                if _byl:
                    parts.append(f'{len(_byl)} match --lock')
                if _byo:
                    parts.append(f"{len(_byo)} draw the board outline and are "
                                 f"not this tool's to move (#829): "
                                 f"{', '.join(sorted(_byo))}")
                why = (f'all {len(diag.selected)} diagnosed part(s) are '
                       f'excluded -- ' + '; '.join(parts))
            if why:
                fallback_rounds += 1
                fallback_reasons.append(f'round {rnd}: {why}')
                print(f"  target-select: FALLING BACK to pins this round -- "
                      f"{why}")
            else:
                diagnosis_rounds += 1
                targets = picked
            select_overlap.append({'round': rnd, 'pins': len(pins_targets),
                                   'diagnosis': len(targets),
                                   'overlap': len(targets & pins_targets),
                                   # Without this a fallback round and a round
                                   # where the diagnosis independently agreed
                                   # with pins carry identical numbers.
                                   'fallback': bool(why)})

        if not targets:
            print("No movable target parts - stopping.")
            break
        rounds_run = rnd

        # #459: a targeted part pulls in its whole block, so the block moves as
        # one body instead of its members fighting each other -- and a block is
        # no longer half-frozen because its IC exceeds --max-target-pins.
        blocks = {}
        if group_sources:
            # In diagnosis mode the universe was frozen before round 0 (poses
            # move, and `netprefix`/`decap` derivation reads poses); in pins
            # mode it is derived per round, exactly as before.
            blocks = (dict(frozen_blocks) if frozen_blocks is not None
                      else derive_groups(pcb_data, group_sources))
            pulled = {r for refs in blocks.values() for r in refs
                      if any(m in targets for m in refs)}
            if pulled - targets:
                print(f"  blocks pull in {len(pulled - targets)} more part(s)")
            targets |= pulled
            blocks = {n: r for n, r in blocks.items()
                      if any(m in targets for m in r)}

        # #554: ONE bounded block relocation, before the quench, on its own
        # board. It has to be its own board rather than a merge into the
        # quench's placements: `_candidate_positions` generates around each
        # part's SEED, so a block relocated inside one QuenchState would be
        # frozen out of every local refinement that follows it. Re-seeding from
        # the written board is what lets the quench polish the new pose.
        # A rejected round still reverts to `cur_file` byte for byte, because
        # `cur_file` is not reassigned unless the round is accepted.
        quench_base = cur_file
        parent_pcb = pcb_data          # before the relocation re-parses it
        reloc = None
        if args.relocate:
            # `blocks` is already filtered to the ones with a targeted member,
            # and that is the right scope rather than an accident: #554 says
            # "move only DIAGNOSED blocks", and a block none of whose members
            # owns a failed or blocker net is not implicated in this round's
            # failure. `--relocate-refs` bypasses the whole derivation for a
            # caller who has decided otherwise.
            reloc = relocate_round(
                pcb_data, cur_file, blocks,
                block=args.relocate_block, refs=args.relocate_refs,
                ignore_nets=args.ignore_nets,
                max_corridor_mm=args.relocate_max_corridor,
                clearance=args.clearance)
            print('  ' + relocate_format(reloc).replace('\n', '\n  '))
            if reloc.refusal:
                relocate_refused += 1
                relocate_reasons.append('round %d: %s' % (rnd, reloc.refusal))
            else:
                relocate_applied += 1
                # `applied` is the field a machine consumer filters on, and it
                # was dead: nothing set it, so three written relocations all
                # serialised as "applied": false in the same JSON that said
                # rounds_applied: 3.
                reloc.applied = True
                relocate_records.append(dict(reloc.to_dict(), round=rnd))
                quench_base = os.path.join(work,
                                           f'loop_round{rnd}_relocated.kicad_pcb')
                write_placed_output(cur_file, quench_base,
                                    [dict(m) for m in reloc.moves])
                copy_siblings(cur_file, quench_base)
                pcb_data = parse_kicad_pcb(quench_base)
                # The block moved, so every part it may now pull in moved with
                # it; the targets were resolved against the OLD poses and stay
                # valid as a REF SET, which is all `move_refs` consumes.
                targets |= set(reloc.members)

        def _round_moved(placements):
            """Every pose this ROUND changed, against the board it started from.

            Two bugs this closes, both in the movie's only input. `pcb_data` is
            the RE-PARSED relocated board by the time the sidecar is written, so
            measuring against it recorded each `from` as the pose the part had
            AFTER the relocation -- disagreeing with the `parent` the same
            sidecar names, on 6 of 10 refs. And a part the relocation moved but
            the quench did not was absent entirely, so `movie_camera` framed the
            shot off the wrong set and never animated the relocation leg at all.

            The quench's entry wins where both moved the same ref: it is the
            final pose, and the relocation's is an intermediate.
            """
            rows = moves_from_placements(parent_pcb, list(reloc.moves) if
                                         (reloc is not None and not reloc.refusal)
                                         else [])
            by_ref = {r['reference']: r for r in rows}
            for r in moves_from_placements(parent_pcb, placements or []):
                by_ref[r['reference']] = r
            return [by_ref[k] for k in sorted(by_ref)]

        name_to_id = {net.name: nid for nid, net in pcb_data.nets.items()}
        net_weights = {name_to_id[n]: args.failed_net_weight
                       for n in best['failed_nets'] if n in name_to_id}

        print(f"Round {rnd}: failed={best['failed_nets']}")
        print(f"  blockers={best['blockers'][:8]}"
              f"{'...' if len(best['blockers']) > 8 else ''}")
        print(f"  targeting {len(targets)} parts"
              f" (max_disp={max_disp:.1f}mm, swap cap={swap_cap:.1f}mm):"
              f" {', '.join(sorted(targets))}")

        ratsnest = {}
        placements = quench(
            pcb_data, pcb_file=quench_base,
            max_displacement=max_disp,
            swap_max_displacement=swap_cap,
            step=args.step,
            grid_step=args.grid_step, clearance=args.clearance,
            board_edge_clearance=args.board_edge_clearance,
            crossing_penalty=args.crossing_penalty,
            length_weight=args.length_weight,
            halo_base=args.halo_base, halo_coef=args.halo_coef,
            halo_weight=args.halo_weight,
            edge_halo=args.edge_halo, edge_weight=args.edge_weight,
            allow_rotations=not args.no_rotate,
            allow_swaps=not args.no_swap,
            ignore_nets=args.ignore_nets, lock_refs=args.lock,
            move_refs=targets, net_weights=net_weights,
            align_weight=args.align_weight,
            align_radius=args.align_radius,
            align_span=args.align_span,
            orient_weight=args.orient_weight,
            metrics_out=ratsnest,
            groups=blocks,
            verbose=args.verbose,
            intent_gate=intent_gate,
        )

        if not placements and (reloc is None or reloc.refusal):
            # #702: name the declared gate when it is what refused, or
            # widening the radius reads as "try harder" while every
            # extra pose it offers is one the gate also refuses.
            _ref = (ratsnest.get("intent_gate") or {}).get("rejected", 0)
            print(f"  Quench found no improving moves - widening the nudge cap"
                  f" (swap cap stays {swap_cap:.1f}mm)."
                  + (f" NOTE: the declared-intent gate refused {_ref} pose(s)"
                     f" this round; a wider radius offers more poses it also"
                     f" refuses. Relax the zone tolerance or drop --intent if"
                     f" the loop stalls here." if _ref else ""))
            max_disp *= 1.5
            continue

        # #504: quench was handed the CURRENT best board, so its own 'before' IS
        # that board's ratsnest and 'after' is the candidate's -- the screen gets
        # its baseline for free, with no second QuenchState (which would re-read
        # the board for courtyards and locked refs) and no round-0 bootstrap.
        rn_before = ratsnest.get('before', {})
        rn_after = ratsnest.get('after', {})
        # A relocation-only round has no quench delta to screen on, and the
        # screen exists to skip a ROUTE that the ratsnest says cannot help. With
        # no `before`/`after` it would compare two empty dicts and skip the one
        # round the relocation was made for.
        skip, why = ((False, '') if not placements else
                     _ratsnest_screen(rn_before, rn_after, args.ratsnest_screen))
        if why:
            print(f"  ratsnest: {why}")
        if skip:
            print(f"  SCREENED - skipping the routing run, widening the nudge cap"
                  f" (swap cap stays {swap_cap:.1f}mm).")
            screened += 1
            # No board is written for a screened round, so without this record a
            # consumer sees a gap in the numbering and cannot tell "screened"
            # from "crashed".
            write_round_sidecar(work, rnd, board=None, routed=None,
                                parent=cur_file, accepted=False, screened=True,
                                targets=targets, groups=blocks,
                                moved=_round_moved(placements),
                                # A screened round writes no board, so this
                                # sidecar is the ONLY record of what the
                                # diagnosis chose -- the one round that cannot
                                # afford to lose it.
                                diagnosis=(diagnosis_to_json(diag)
                                           if diag is not None else None),
                                relocation=(reloc.to_dict()
                                            if reloc is not None else None))
            max_disp *= 1.5
            continue

        cand_file = os.path.join(work, f'loop_round{rnd}.kicad_pcb')
        write_placed_output(quench_base, cand_file, placements)

        metrics = run_route(
            cand_file, os.path.join(work, f'loop_round{rnd}_routed.kicad_pcb'),
            args.route_args, os.path.join(work, f'loop_round{rnd}_route.log'),
            json_file=os.path.join(work, f'loop_round{rnd}_route.json'),
            extra_targets=args.target_nets,
            keep_blocker_cells=_keep_cells)
        # Report-only, exactly like the pad_pairs_* keys above: every round now
        # records placement quality next to the routing result. better() is
        # deliberately untouched -- reworking the comparator is #458's scope.
        metrics['ratsnest_crossings'] = rn_after.get('crossings')
        metrics['ratsnest_hpwl'] = rn_after.get('hpwl')
        metrics['ratsnest_length'] = rn_after.get('length')
        print(f"  -> failures={metrics['failures']}"
              f" iterations={metrics['iterations']:,} vias={metrics['vias']}")

        # Record BEFORE the accept/revert bookkeeping mutates cur_file, so
        # `parent` names the board this candidate was actually derived from.
        if args.accept_cmd:
            _score, _note = accept_score(
                args.accept_cmd, cand_file,
                os.path.join(work, f'loop_round{rnd}_routed.kicad_pcb'),
                os.path.join(work, f'loop_round{rnd}_route.json'))
            metrics['accept_score'] = _score
            # Strictly better, and a judge that could not answer never wins.
            _accepted = _score is not None and (best_score is None
                                                or _score < best_score)
            print(f"  accept-cmd: {_note}"
                  + (f" vs incumbent {best_score:g}" if best_score is not None
                     else " (no incumbent score)"))
        else:
            _accepted = better(metrics, best)
        write_round_sidecar(work, rnd, board=cand_file,
                            routed=os.path.join(
                                work, f'loop_round{rnd}_routed.kicad_pcb'),
                            parent=cur_file, accepted=_accepted,
                            targets=targets, groups=blocks,
                            moved=_round_moved(placements),
                            metrics=metrics,
                            diagnosis=(diagnosis_to_json(diag)
                                       if diag is not None else None),
                            relocation=(reloc.to_dict()
                                        if reloc is not None else None))
        if _accepted:
            print(f"  ACCEPTED (was failures={best['failures']},"
                  f" iterations={best['iterations']:,})")
            best = metrics
            if args.accept_cmd:
                best_score = metrics.get('accept_score')
            cur_file = cand_file
            max_disp = args.max_displacement
            accepted_rounds += 1
        else:
            print(f"  REJECTED - reverting, widening the nudge cap"
                  f" (swap cap stays {swap_cap:.1f}mm).")
            max_disp *= 1.5

    shutil.copy(cur_file, args.output_file)
    copy_siblings(cur_file, args.output_file)          # #441, as at round 0
    if args.ratsnest_screen > 0:
        print(f"Ratsnest screen: {screened} round(s) skipped the routing run"
              f" at {args.ratsnest_screen:g}% regression.")
    print(f"Final: failures={best['failures']} iterations={best['iterations']:,}"
          f" vias={best['vias']}")
    print(f"Wrote {args.output_file}")

    # A machine-readable verdict. Before this the ONLY structured output of a
    # loop run was the per-round `loop_round{N}.json` sidecars: the normal path
    # printed a text `Final:` line and nothing else, and `main()` returned None
    # so the process exited 0 whether `failures` was 0 or 400. Any harness that
    # wanted to know how the run went had to scrape stdout or reassemble the
    # sidecars. Same key shape as the other placement CLIs (before/after pairs).
    summary = {
        'rounds': args.rounds,
        'rounds_run': rounds_run,
        'rounds_accepted': accepted_rounds,
        'rounds_screened': screened,
        'failures_before': baseline['failures'],
        'failures_after': best['failures'],
        'iterations_before': baseline['iterations'],
        'iterations_after': best['iterations'],
        'vias_before': baseline['vias'],
        'vias_after': best['vias'],
        'failed_nets_before': sorted(baseline.get('failed_nets') or []),
        'failed_nets_after': sorted(best.get('failed_nets') or []),
        'max_displacement': args.max_displacement,
        'max_target_pins': args.max_target_pins,
        'group_by': args.group_by,
        # #553. `target_select` is an additive echo, in the same class as
        # `group_by` above. The rest appear ONLY in diagnosis mode, and
        # `rounds_diagnosis == 0` beside a non-zero `rounds_fallback` is the
        # machine-readable statement that the flag did nothing this run.
        'target_select': args.target_select,
        'relocate': bool(args.relocate),
        'work_dir': work,
        'output': args.output_file,
    }
    if args.relocate:
        from placement.relocate import NO_EFFICACY_CLAIM as _NEC
        summary.update({
            'relocate_rounds_applied': relocate_applied,
            'relocate_rounds_refused': relocate_refused,
            'relocate_refusals': relocate_reasons,
            'relocate_proposals': relocate_records,
            'relocate_max_corridor_mm': args.relocate_max_corridor,
            # A relocation of a block the diagnosis picked inherits the
            # diagnosis's own measured null, and the presence of a solver does
            # not launder it. A machine consumer cannot read the numbers above
            # without also reading this.
            'relocate_efficacy': _NEC,
        })
    if args.target_select == 'diagnosis':
        from placement.diagnosis import NO_EFFICACY_CLAIM
        summary.update({
            'target_select_rounds_diagnosis': diagnosis_rounds,
            'target_select_rounds_fallback': fallback_rounds,
            'target_select_fallback_reasons': fallback_reasons,
            'target_select_overlap': select_overlap,
            'target_select_top_k': args.diagnosis_top_k,
            'target_select_blocks': len(frozen_blocks or {}),
            # Not a courtesy. A machine consumer reading this verdict cannot
            # get the numbers without also getting the sentence that says no
            # measurement supports them.
            'target_select_efficacy': NO_EFFICACY_CLAIM,
        })
    print("JSON_SUMMARY: " + json.dumps(summary, sort_keys=True))

    if args.movie is not None:
        # The work dir already holds every round board plus the loop_round{N}.json
        # sidecars, so this needs no extra plumbing -- make_movie reads a
        # directory, and the sidecars are what make the set a chain.
        try:
            from make_movie import make_movie
            out = args.movie or os.path.join(work, 'placement.mp4')
            got = make_movie([work], out=out, camera='auto', quiet=False)
            print(f"Movie: {got}" if got else "Movie: nothing to animate")
        except Exception as e:
            print(f"  (movie skipped: {e})")

    # 0 = the loop RAN. Deliberately not "0 = the board routes": every other
    # tool in this family reserves non-zero for "this tool could not do its
    # job" (2 argparse, 3 board-state refusal), and route.py does not exit
    # non-zero for unrouted nets either. Making a remaining routing failure a
    # process failure would make this the only tool that conflates the two, and
    # would break every `set -e` caller in tests/stress the first time a hard
    # board did not close. The verdict is the JSON_SUMMARY above.
    return 0


if __name__ == "__main__":
    # Declare the lever for the WHOLE run, so every pose this CLI
    # writes carries its name. Nothing called declare_lever outside
    # tests, so the unaided instrument had no armed state at all:
    # unarmed it is silent, and armed by hand it refused the engine.
    from placement.provenance import declare_lever
    with declare_lever('place_route_loop.py', sys.argv):
        # The return value used to be dropped here, so a refusal deeper in main()
        # still exited 0 (the #551 family). Propagate it.
        sys.exit(main() or 0)
