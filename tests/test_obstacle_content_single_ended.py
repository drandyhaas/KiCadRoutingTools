#!/usr/bin/env python3
"""Invariant E (map CONTENT) on the route.py front -- the gap that let four
copper-commit sites drift (#806 follow-up).

WHY THIS FILE EXISTS. `tests/test_obstacle_map_balance.py` already drives
route.py on a churn recipe, and it asserts:

    route.py: working map BALANCED
    route.py: cache-object ledger balanced

Those are invariants A/B/C -- BALANCE. A stale-but-balanced cache entry passes
them by construction, which #806 states in its own words: "The ref-count
invariants A/B/C HELD throughout -- the stale entry was removed and re-added in
balance -- while the map's CONTENT was wrong." The CONTENT check (invariant E,
`run_obstacle_content_audit`) was built by #806 and wired ONLY into the
route_diff front. Nothing ever asserted it on route.py.

Measured cost of that gap, 2026-09-16, on the recipe test_obstacle_map_balance
ALREADY RUNS (kicad_files/flat_hierarchy.kicad_pcb, single layer, fat geometry):

    HEAD                     48785 cells NOT blocked, 18 stale entries
    with the four fixes      31456 cells NOT blocked, 15 stale entries

So the existing fixture reproduced the defect all along; only the assertion was
missing. On cparti_fpga's retry step the same audit read 13 stale entries /
12064 under-blocked cells at HEAD and 0 / 9 after the fixes.

THE FOUR SITES (each a copper commit that did not tell the map):
  * route.py "Issue #134 last resort"          -- no refresh AND no registration
  * diff_pair_custody casualty branch D        -- refreshed, never registered
  * diff_pair_custody casualty REROUTE branch  -- `record_single_ended_success`
                                                  takes no map argument
  * net_rescue commit                          -- registered, never refreshed
The last two are mirror images, which is why no single pattern finds them.

TWO THINGS THIS GATE GETS RIGHT THAT A NAIVE VERSION WOULD NOT:

  1. It reads EVERY `[OBSTACLE CONTENT]` line, not the last. At HEAD the FINAL
     audit on this fixture reads 0 stale while an earlier one reads 18 -- a
     last-line check passes on broken code.
  2. It REPORTS which at-risk commit paths the recipe actually fired. This
     fixture fires `rescue` (16x) but NOT the #134 last resort or the casualty
     branches, so a green run here does not mean those are covered. Saying so
     is the point (#923: every gate says what it cannot see); the STRUCTURAL
     rows below are what guard the paths this board never reaches.

THE FIFTH SITE, AND WHY THE BASELINE IS NOW ZERO. Four fixes left 15 stale
entries on this fixture while every per-stage audit INSIDE batch_route read 0 --
so the leak was after them. Bracketing the post-route cleanup found it:

    7a-before-cleanup:     42 cells NOT blocked,     0 via,  0 stale
    7b-after-cleanup:   31456 cells NOT blocked, 30997 via, 15 stale

`run_post_route_cleanup` MOVES and STRIPS copper (the graze nudges, the via
nudge, the prunes and sweeps, the #536 smoother) by mutating pcb_data directly,
outside the rip/commit choke points, and never told the map. It was the single
largest source of invariant-E error in the run. With route.py refreshing the
scope nets after it, the fixture reads 0 stale / 0 under-blocked via cells, so
the baseline is 0: a real invariant, not a tolerated number.

That fix also closed the DRC it was causing. cparti_fpga's retry step went
drc_real 30 -> 10 with connectivity IMPROVING (15 -> 10 nets incomplete), and
the 11mm collinear SPIs_MISO/SPIs_SCK short -- which a correct-looking map had
not prevented -- disappeared entirely.
"""
import ast
import glob
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tests'))

#: Worst stale-entry count across all audits on the flat_hierarchy recipe.
#: 18 at HEAD -> 15 with four fixes -> 0 once the cleanup pipeline refreshed
#: too (2026-09-16). The invariant HOLDS, so this is 0 and any regression is a
#: failure, not a tolerated number.
BASELINE_WORST_STALE = 0

#: Commit sites with no obstacle refresh in their innermost block. Each entry
#: is either MEASURED CLEAR (the audit read 0 stale on a run where the site
#: demonstrably fired) or UNVERIFIED (tolerated, not yet exercised). A site not
#: on this list and not refreshing is new drift and fails.
ALLOWLIST = {
    # Keyed by file:FUNCTION, never file:line -- a line-number key shifts on any
    # edit above the site and fires "unlisted" + "orphan" together on an
    # unrelated commit. A gate that cries wolf gets disabled, which is how the
    # original invariant-E gap survived in the first place.
    'py_router/diff_pair_multipoint.py:_route_chain_attempt':
        'MEASURED CLEAR: hybrid leg + per-leg commits fired on cparti route_diff, audit 0/0/0',
    'py_router/diff_pair_routing.py:seam_reask_chain_leg':
        'MEASURED CLEAR: #444 seam re-ask fired 2x on cparti route_diff, audit 0/0/0',
    'py_router/routing_context.py:record_single_ended_success':
        'BY DESIGN: this IS the recorder; its callers own the refresh',
    'py_router/routing_context.py:restore_ripped_net':
        'BY DESIGN: delegates to restore_net, which refreshes',
    'py_router/rip_up_reroute.py:restore_net':
        'UNVERIFIED: partial_leg_rip arm; not yet exercised by a measured run',
    'py_router/layer_swap_fallback.py:try_fallback_layer_swap':
        'UNVERIFIED: victim-reroute map is local to this function',
    'py_router/net_rescue.py:rescue_failed_nets':
        'COVERED: the refresh is at the SHARED commit point after both\n         branches, not inside each commit\'s own block',
    'py_router/net_rescue.py:_attempt_net_at_geometry':
        'COVERED: as rescue_failed_nets -- rescue rebuilds its own pristine\n         window maps (copper-epoch keyed) and the shared refresh covers the\n         persistent map',
}

#: Commit paths this gate would like to exercise. Reported, not required --
#: a board that does not reach one is honest coverage information.
AT_RISK = {
    '#134 last resort': r"Issue #134 last resort",
    'casualty partial': r"PARTIAL .*: reroute failed",
    'casualty reroute': r"REROUTED .*: casualty re-routed",
    'rescue commit': r"Rescuing ",
}

CONTENT_RE = re.compile(
    r"\[OBSTACLE CONTENT[^\]]*\] (?P<nets>\d+) nets recomputed from pcb_data: "
    r"(?P<cells>\d+) cells checked, (?P<missing>\d+) NOT blocked in working "
    r"map; (?P<vias>\d+) via cells checked, (?P<vmissing>\d+) NOT via-blocked; "
    r"(?P<stale_cells>\d+) cells absent from their net's cache entry "
    r"\((?P<stale>\d+) stale entries\)")

FAILS = []


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


# ---------------------------------------------------------------------------
# Structural: every copper commit tells the map, or is allowlisted WITH A REASON
# ---------------------------------------------------------------------------
REFRESH = {'refresh_net_obstacles', 'update_net_obstacles_after_routing',
           'restore_obstacles_inplace', 'add_net_obstacles_from_cache',
           'record_single_ended_success', 'precompute_net_obstacles'}


def _commit_sites():
    """{'file:function': refreshes?} for every add_route_to_pcb_data call,
    judged in its INNERMOST enclosing statement block.

    Innermost matters: batch_route is thousands of lines and mentions the
    refresh names elsewhere, so a function- or ancestor-scoped test marks the
    #134 site 'refreshed' and reports nothing. Two earlier cuts of this audit
    did exactly that and returned a FALSE NEGATIVE on the very bug they were
    written for -- hence t_the_structural_detector_is_not_vacuous below.

    A function with several commit sites is one key: it refreshes only if EVERY
    one of its sites does, so a newly added unrefreshed commit inside an
    already-allowlisted function still has to be justified by a human reading
    the reason string.
    """
    best = {}
    for f in sorted(glob.glob(os.path.join(ROOT, 'py_router', '*.py'))):
        try:
            tree = ast.parse(open(f).read())
        except SyntaxError:
            continue
        rel = os.path.relpath(f, ROOT)
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            for node in ast.walk(fn):
                for fld in ('body', 'orelse', 'finalbody'):
                    body = getattr(node, fld, None)
                    if not isinstance(body, list):
                        continue
                    for i, stmt in enumerate(body):
                        hits = [n for n in ast.walk(stmt)
                                if isinstance(n, ast.Call)
                                and isinstance(n.func, ast.Name)
                                and n.func.id == 'add_route_to_pcb_data']
                        if not hits:
                            continue
                        names = set()
                        for s2 in body[i:]:
                            for n in ast.walk(s2):
                                if isinstance(n, ast.Call) and isinstance(n.func, ast.Name):
                                    names.add(n.func.id)
                                if isinstance(n, ast.ImportFrom):
                                    names.update(a.name for a in n.names)
                        key = f"{rel}:{fn.name}"
                        ok = bool(REFRESH & names)
                        best[key] = ok if key not in best else (best[key] and ok)
    return best


def t_every_commit_site_tells_the_map():
    sites = _commit_sites()
    check('t_commit_sites_found', len(sites) >= 10,
          f'{len(sites)} function(s) commit copper via add_route_to_pcb_data')
    unlisted = sorted(k for k, ok in sites.items()
                      if not ok and k not in ALLOWLIST)
    check('t_no_unlisted_commit_site_skips_the_refresh',
          not unlisted,
          'every non-refreshing commit site is allowlisted with a reason'
          if not unlisted else
          'commit site(s) with no obstacle refresh and no allowlist entry -- '
          'copper committed here is invisible to every map built afterwards: '
          + ', '.join(unlisted))
    # An allowlist entry for a site that no longer exists is stale bookkeeping
    # and hides the next one: fail it rather than let the list rot.
    orphans = sorted(k for k in ALLOWLIST if k not in sites)
    check('t_allowlist_has_no_orphans', not orphans,
          'every allowlist entry names a real commit site'
          if not orphans else f'allowlist names sites that are gone: {orphans}')


def t_the_structural_detector_is_not_vacuous():
    """It must FAIL on a synthetic unrefreshed site, or the rows above prove
    nothing. (Both earlier cuts of this audit passed on the broken tree.)"""
    broken = ast.parse("def f():\n"
                       "    if x:\n"
                       "        add_route_to_pcb_data(pcb, r)\n"
                       "        results.append(r)\n")
    found = []
    for node in ast.walk(broken):
        for fld in ('body', 'orelse'):
            body = getattr(node, fld, None)
            if not isinstance(body, list):
                continue
            for i, stmt in enumerate(body):
                if any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                       and n.func.id == 'add_route_to_pcb_data'
                       for n in ast.walk(stmt)):
                    names = {n.func.id for s2 in body[i:] for n in ast.walk(s2)
                             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
                    found.append(bool(REFRESH & names))
    check('t_the_structural_detector_is_not_vacuous',
          found and not any(found),
          'a synthetic commit-without-refresh IS detected')


# ---------------------------------------------------------------------------
# Behavioural: invariant E on the route.py front, across EVERY audit
# ---------------------------------------------------------------------------
def t_invariant_e_on_the_route_py_front():
    board = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')
    if not os.path.isfile(board) or os.path.getsize(board) == 0:
        check('t_invariant_e_on_the_route_py_front', False,
              f'fixture missing: {board} -- this gate graded NOTHING')
        return
    env = dict(os.environ, KICAD_OBSTACLE_AUDIT='1', KICAD_OBSTACLE_LEDGER='1',
               KICAD_PLANE_FRAGILITY_COST='0')
    with tempfile.TemporaryDirectory(prefix='obs_content_') as d:
        out = os.path.join(d, 'churn.kicad_pcb')
        argv = [sys.executable, '-X', 'utf8',
                os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                '--nets', '*', '!GND', '--track-width', '0.8',
                '--clearance', '0.7', '--via-size', '1.0', '--via-drill', '0.6',
                '--layers', 'F.Cu', '--max-ripup', '10']
        try:
            p = subprocess.run(argv, cwd=ROOT, env=env, capture_output=True,
                               text=True, timeout=2400,
                               encoding='utf-8', errors='replace')
        except subprocess.TimeoutExpired:
            check('t_invariant_e_on_the_route_py_front', False,
                  'the churn recipe timed out -- no verdict, not a pass')
            return
    log = (p.stdout or '') + (p.stderr or '')
    check('t_run_completed', p.returncode == 0 and 'Traceback' not in log,
          f'rc={p.returncode}')
    rows = list(CONTENT_RE.finditer(log))
    check('t_content_audit_ran', bool(rows),
          f'{len(rows)} [OBSTACLE CONTENT] audit line(s)'
          if rows else 'no content audit line -- KICAD_OBSTACLE_AUDIT did not '
                       'engage, so invariant E graded nothing')
    if not rows:
        return
    pops = [int(m.group('cells')) for m in rows]
    check('t_population_non_vacuous', min(pops) > 0,
          f'smallest audited population {min(pops)} cells')
    rips = len(re.findall(r"[Rr]ipping|Extending to N", log))
    check('t_churn_engaged', rips >= 1,
          f'{rips} rip/extend event(s) -- the recipe still churns, so the '
          f'commit paths under test are reachable')

    worst = max(int(m.group('stale')) for m in rows)
    worst_cells = max(int(m.group('missing')) for m in rows)
    print(f"    invariant E across {len(rows)} audit(s): worst stale={worst}, "
          f"worst NOT-blocked cells={worst_cells}")
    worst_via = max(int(m.group('vmissing')) for m in rows)
    check('t_no_cells_under_blocked', worst_cells == 0,
          'every cell the board\'s copper owns is blocked in the working map'
          if worst_cells == 0 else
          f'{worst_cells} cell(s) NOT blocked -- copper a later search on this '
          f'map will route straight through (invariant E). NB the audit measures '
          f'on a clone_fresh() copy, so these are NOT source/target exemptions')
    check('t_no_via_cells_under_blocked', worst_via == 0,
          'every via cell is via-blocked' if worst_via == 0 else
          f'{worst_via} via cell(s) NOT via-blocked')
    check('t_stale_entries_no_worse_than_baseline',
          worst <= BASELINE_WORST_STALE,
          f'worst stale {worst} <= baseline {BASELINE_WORST_STALE} '
          f'(target 0)' if worst <= BASELINE_WORST_STALE else
          f'worst stale {worst} EXCEEDS baseline {BASELINE_WORST_STALE} -- a '
          f'copper commit stopped telling the obstacle map')
    if worst < BASELINE_WORST_STALE:
        print(f"    NOTE: improved to {worst}; lower BASELINE_WORST_STALE "
              f"to {worst} in the same commit as the fix")

    # COVERAGE, reported not required: which at-risk paths this board reached.
    print("    at-risk commit paths exercised by this fixture:")
    for label, pat in AT_RISK.items():
        n = len(re.findall(pat, log))
        print(f"      {label:<20} {'fired ' + str(n) + 'x' if n else 'NOT EXERCISED (structural rows only)'}")


def main():
    t_the_structural_detector_is_not_vacuous()
    t_every_commit_site_tells_the_map()
    t_invariant_e_on_the_route_py_front()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
