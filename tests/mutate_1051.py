#!/usr/bin/env python3
"""The #1051 / #1052 / #1053 / #1054 / #1043 mutation battery: does anything
notice when the arrays, fixed poses, rigid groups or tethers stop doing what
the PR says they do?

One row per load-bearing mechanism, across the eight places the PR put one:

  ar  `placement/arrays.py`: the formation predicate (axis, order, rotation
      incl. the 2-pad modulo-180, pitch; an order with < 2 resolved members
      is UNCHECKED), the pin order's single-pad preference, and the pose-
      blind detector (host floor, connector / jumper exclusion, rail filter,
      bridge decline, one role per part, the paste-ready `suggestion_row`).
  fp  `placement/floorplan.py`: the `arrays[]` / `fixed_poses[]` load
      refusals, the board-aware `array_problems`, the fixed-pose grade (brief
      vs mechanical), `rule_array_formation` and its arming, the gate bundle.
  sd  `placement/seeder.py`: stage 0's exact seat and its KiCad-style
      courtyard rule (abutting is legal, overlapping is not, judged pairwise
      over every DECLARED pose), the held / anchors-first exclusion,
      hosts-only-first for rows, a row member
      never seated alone, `_seat_block`'s pose cap / sibling re-check / band
      order, the partner-centroid target, the decap NOTE on a zero claim, and
      the ref tie-break keys that make a seed hash-seed independent.
  qu  `placement/quench.py`: the rigid-group merge and dedupe, the nudge
      exclusion, `_rigid_swap_ok`, release / rejoin (trigger, end of pass,
      hysteresis, ONE exclusion set), the tether's live election, the group
      move's partner poses, `swap_intent_ok`'s tether conjunct, the shortcut
      soundness guards, `clusters_dropped`.
  ps  `place_seed.py`: exit 4 on an unhonoured fixed pose; the disclosure
      judged at the WRITTEN poses.
  pd  (RETIRED) the staged placement driver's P1 fixed-pose rows, removed
      with the driver when that skill was retired for pcb-free-agent.
  rl  `place_route_loop.py`: each round's quench disclosure reaches the
      JSON_SUMMARY.
  rc  `placement/reconcile.py`: a brief array member with a mechanical pose
      is a contradiction row.

Every row carries an EXPECTATION, and a verdict that does not match it is
reported as WRONG. An anchor that does not match its target EXACTLY ONCE is
BROKEN, checked before anything is written (`preflight`, #877).

A killer is `path` or `path::test_name[::test_name...]`: the named tests
alone, through the file's own substring filter. A filter that matches
nothing runs nothing and exits 0 -- so the UNMUTATED baseline asserts that
every name it asked for printed its `--- name` line, and the battery exits 2
before any mutation when one did not (or when any killer fails unmutated).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- run it in a checkout nothing else
is reading. It refuses to start while a target has uncommitted changes,
because it restores by overwriting. Do not import it: the tree's rule for
every battery, whatever this one's module scope happens to do.

    python3 -X utf8 tests/mutate_1051.py
    python3 -X utf8 tests/mutate_1051.py --row fixed-abutting-refused
    python3 -X utf8 tests/mutate_1051.py --list

`_uncache` is carried over from `tests/mutate_983.py`: several rows are
same-size edits, and CPython trusts a `.pyc` on (mtime seconds, size).

EXPECTED SURVIVORS, with the reason, rather than deleted rows: none.
`zoned-row-zone-packed-first` was one until a verifier reached the double
failure it needs (esp_prog, U1 pinned by a fixed pose, R3/R4 a row zoned to
U1's pads: the row caps and the one-by-one zone fallback finds nothing);
`test_1051_hardening`'s zoned-row test now kills it.

THE MEASURED RESULT is recorded below from the run, never predicted.

MEASURED on the tree of `test_1051_hardening: a plain string where the
f-string had no placeholder` (7857a2455, 2026-09-26, Windows; every killer
run unmutated first, every selected test name seen to run, all green):
**154 rows, 153 KILLED, 1 SURVIVED -- the then-expected
`zoned-row-zone-packed-first` -- 0 broken, 1340 s wall.** Superseded below.

The table is now **159 rows**: the phase-7 verifier added 2 (156), and the
final re-review 3 -- `fixed-pad-short-unnamed` and
`route-loop-disclosure-dropped` (two survivors it found, each killed by a new
assertion) and `mechanical-array-member-not-a-contradiction`.

MEASURED on e91f019f3 (2026-09-26, Windows; every killer run unmutated
first, every selected test name seen to run, all green): **159 rows, 159
KILLED, 0 SURVIVED, 0 broken, 1467 s wall.**
`mechanical-array-member-not-a-contradiction` is killed through run_utils'
"failed, but NOT for the stated reason": without the contradiction row P1
falls back to its unlocked-anchor refusal, which is the mutant's effect.
Superseded below.

The #1059 review removed `decaps.seat_owners_first` and the three rows that
guarded it (`owners-first-armed-without-the-key`,
`rows-seat-the-whole-tier-first`, `rows-after-every-part`): **156 rows**.

MEASURED, the run of record, on 21b3de07 (2026-09-27, Windows, beside a full
suite run; every killer run unmutated first, every selected test name seen
to run, all green): **156 rows, 156 KILLED, 0 SURVIVED, 0 broken, 2018 s
wall.**

The earlier run, which changed the tests rather than the table:
- 154 rows at b0dfc488f (1525 s): 124 KILLED, 30 SURVIVED, none expected.
  29 were holes in the PR's tests -- the pin order's shared-net and
  lowest-pad rules, the grid host, an absent member, place_seed's written-
  pose verdicts, an anchored rigid group, six row-seat mechanisms (courtyard
  offsets, pin order over a shuffled list, the self-check, the mod-180
  dedupe, the pin-side axis, the revert), two row refusals, stage 0's pad
  clearance / eviction freeze / copper-less courtyard, 2.4's row rank and
  the decap stage's claim on row caps, five release/rejoin rules, and four
  tether rules. `tests/test_1051_hardening.py` -- 24 tests at 8de18a378,
  not the 25 its commit message says -- kills each (named in its
  docstring); the thirtieth was then taken for an equivalent survivor.

Phase-7 verification (one fresh verifier reproduced every row): the
survivor above was reachable (now KILLED, one more hardening test, 25);
`array-formation-no-geometry-passes` killed only through the KeyError its
mutant causes further down the rule, so it is renamed for what it shows
(`array-formation-no-geometry-reaches-the-measurement`) and the abstention
and its disclosed row get rows of their own; and the baseline's "did the
named test run" check matches the exact `--- name` line, no longer a
substring of it.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys
import time

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

ARRAYS = os.path.join(_ROOT, 'py_placer', 'placement', 'arrays.py')
FLOORPLAN = os.path.join(_ROOT, 'py_placer', 'placement', 'floorplan.py')
SEEDER = os.path.join(_ROOT, 'py_placer', 'placement', 'seeder.py')
QUENCH = os.path.join(_ROOT, 'py_placer', 'placement', 'quench.py')
PLACE_SEED = os.path.join(_ROOT, 'py_placer', 'place_seed.py')
ROUTE_LOOP = os.path.join(_ROOT, 'py_placer', 'place_route_loop.py')
RECONCILE = os.path.join(_ROOT, 'py_placer', 'placement', 'reconcile.py')
TARGETS = {'ar': ARRAYS, 'fp': FLOORPLAN, 'sd': SEEDER, 'qu': QUENCH,
           'ps': PLACE_SEED, 'rl': ROUTE_LOOP, 'rc': RECONCILE}

T_AS = os.path.join(_TESTS, 'test_1051_arrays_schema.py')
T_SA = os.path.join(_TESTS, 'test_1051_suggest_arrays.py')
T_SEED = os.path.join(_TESTS, 'test_1051_seed_arrays.py')
T_QB = os.path.join(_TESTS, 'test_1051_quench_blocks.py')
T_DET = os.path.join(_TESTS, 'test_1051_determinism.py')
T_959 = os.path.join(_TESTS, 'test_959_reconcile.py')
T_H = os.path.join(_TESTS, 'test_1051_hardening.py')

#: (name, target, old, new, killers, expect)
ROWS = [
    # ==== arrays.formation ==================================================
    ('formation-axis-never-fails', 'ar',
     "        failed.append('axis')",
     "        pass",
     (T_AS + '::test_the_formation_predicate',), 'KILLED'),
    ('formation-axis-tolerance-ignored', 'ar',
     "    ok = dev <= tol['axis_mm'] + 1e-9",
     "    ok = True",
     (T_AS + '::test_the_formation_predicate',), 'KILLED'),
    ('formation-auto-axis-picks-the-shorter-extent', 'ar',
     "        axis = 'x' if (max(xs) - min(xs)) >= (max(ys) - min(ys)) else 'y'",
     "        axis = 'x' if (max(xs) - min(xs)) < (max(ys) - min(ys)) else 'y'",
     (T_AS + '::test_the_formation_predicate',
      T_AS + '::test_array_formation_grades_the_as_built_board'), 'KILLED'),
    ('formation-order-never-fails', 'ar',
     "            failed.append('order')",
     "            pass",
     (T_AS + '::test_the_formation_predicate',), 'KILLED'),
    ('formation-order-reversed-row-refused', 'ar',
     "        ok = seen == want or seen == list(reversed(want))",
     "        ok = seen == want",
     (T_AS + '::test_the_formation_predicate',
      T_AS + '::test_array_formation_grades_the_as_built_board'), 'KILLED'),
    ('formation-order-over-one-member-is-checked', 'ar',
     "    elif len(want) < 2:",
     "    elif len(want) < 1:",
     (T_AS + '::test_an_order_nobody_resolves_is_unchecked_not_passed',),
     'KILLED'),
    ('formation-two-pad-not-modulo-180', 'ar',
     "    return (180.0 if pads is not None and int(pads) <= SYMMETRIC_MAX_PADS",
     "    return (360.0 if pads is not None and int(pads) <= SYMMETRIC_MAX_PADS",
     (T_AS + '::test_two_pad_parts_are_the_same_turned_180',), 'KILLED'),
    ('formation-declared-rotation-never-off', 'ar',
     "                     if _ang_diff(r, want_rot, t) > tol['rotation_deg'])",
     "                     if _ang_diff(r, want_rot, t) > 1e9)",
     (T_AS + '::test_the_formation_predicate',), 'KILLED'),
    ('formation-shared-rotation-looser-period', 'ar',
     "        spread = max(_ang_diff(rots[i], rots[j], max(per[i], per[j]))",
     "        spread = max(_ang_diff(rots[i], rots[j], min(per[i], per[j]))",
     (T_AS + '::test_two_pad_parts_are_the_same_turned_180',), 'KILLED'),
    ('formation-shared-rotation-never-fails', 'ar',
     "        ok = spread <= tol['rotation_deg']",
     "        ok = True",
     (T_AS + '::test_the_formation_predicate',), 'KILLED'),
    ('formation-declared-pitch-ignored', 'ar',
     "               if abs(g - float(pitch_spec)) > tol['pitch_mm'] + 1e-9]",
     "               if abs(g - float(pitch_spec)) > 1e9]",
     (T_AS + '::test_the_formation_predicate',), 'KILLED'),
    ('formation-auto-pitch-spread-ignored', 'ar',
     "        ok = (spread <= tol['pitch_spread_mm'] + 1e-9",
     "        ok = (True",
     (T_AS + '::test_the_formation_predicate',), 'KILLED'),
    ('formation-stacked-members-are-a-row', 'ar',
     "              and min(gaps) > tol['pitch_spread_mm'])",
     "              and True)",
     (T_AS + '::test_the_formation_predicate',
      T_AS + '::test_a_missing_member_skips_the_formation_and_stacked_reads_so',
      T_AS + '::test_stacked_members_still_report_the_other_gaps'), 'KILLED'),

    # ==== arrays.pin_order ==================================================
    ('pin-order-single-pad-preference-off', 'ar',
     "        use = single or own",
     "        use = own",
     (T_AS + '::test_the_single_pad_net_decides_the_pin_order',), 'KILLED'),
    ('pin-order-shared-net-orders', 'ar',
     "        own = [n for n in nets if reach.get(n) == 1 and n in host_pads]",
     "        own = [n for n in nets if reach.get(n) >= 1 and n in host_pads]",
     (T_H + '::test_pin_order_ignores_shared_nets_and_keys_on_the_lowest_pad',
      T_AS + '::test_pin_order_is_read_off_pads_and_nets',), 'KILLED'),
    ('pin-order-highest-pad', 'ar',
     "        pad = min((p for n in use for p in host_pads[n]), key=natural_key)",
     "        pad = max((p for n in use for p in host_pads[n]), key=natural_key)",
     (T_H + '::test_pin_order_ignores_shared_nets_and_keys_on_the_lowest_pad',
      T_AS + '::test_pin_order_is_read_off_pads_and_nets',), 'KILLED'),

    # ==== arrays.suggest_arrays =============================================
    ('suggest-host-floor-off', 'ar',
     "                      if _copper_pad_count(fps[h]) >= floor])",
     "                      if True])",
     (T_SA + '::test_host_floor_and_cascade_on_splitflap',), 'KILLED'),
    ('suggest-host-floor-ratio-zero', 'ar',
     "                MIN_HOST_TO_MEMBER_PADS * _copper_pad_count(fps[refs[0]])",
     "                0 * _copper_pad_count(fps[refs[0]])",
     (T_SA + '::test_host_floor_and_cascade_on_splitflap',), 'KILLED'),
    ('suggest-floored-host-not-declined', 'ar',
     "                and score[best][0] >= MIN_MEMBERS and best not in floored):",
     "                and False):",
     (T_SA + '::test_host_floor_and_cascade_on_splitflap',
      T_SA + '::test_decline_wording'), 'KILLED'),
    ('suggest-connector-is-a-member', 'ar',
     '        return f"part_class classifies it {cls}"',
     '        pass',
     (T_SA + '::test_connectors_testpoints_jumpers_are_never_members',),
     'KILLED'),
    ('suggest-jumper-is-a-member', 'ar',
     "    if any(k in name for k in JUMPER_FP):",
     "    if False:",
     (T_SA + '::test_connectors_testpoints_jumpers_are_never_members',),
     'KILLED'),
    ('suggest-rail-is-an-own-signal', 'ar',
     "            if _is_rail(name, p.net_id, rails):",
     "            if False:",
     (T_SA,), 'KILLED'),
    ('suggest-supply-pin-rails-ignored', 'ar',
     "    return (bool(net_id) and net_id in rails) or _rail_by_name(name)",
     "    return _rail_by_name(name)",
     (T_SA + '::test_the_supply_pin_rail_path',), 'KILLED'),
    ('suggest-two-part-net-is-a-rail', 'ar',
     "                if len(parts) < RAIL_MIN_PARTS:",
     "                if False:",
     (T_SA,), 'KILLED'),
    ('suggest-digit-leaf-always-a-rail', 'ar',
     "    return not leaf[:1].isdigit() or bool(_VOLTAGE_LEAF.match(leaf))",
     "    return True",
     (T_SA + '::test_rail_names_the_detector_reads_locally',), 'KILLED'),
    ('suggest-sheet-bank-rail-is-own', 'ar',
     "                and not _is_rail(_net_label(pcb, net_id), net_id, rails))",
     "                and True)",
     (T_SA + '::test_rails_are_not_own_nets_in_a_sheet_bank',), 'KILLED'),
    ('suggest-bridge-not-declined', 'ar',
     "        if other is not None:",
     "        if False:",
     (T_SA + '::test_glasgow_resistor_arrays_and_buffers',), 'KILLED'),
    ('suggest-passive-series-bridges', 'ar',
     "    if any(_copper_pad_count(fps[m]) <= SYMMETRIC_MAX_PADS for m in members):",
     "    if False:",
     (T_SA + '::test_series_passives_are_rows_not_bridges',), 'KILLED'),
    ('suggest-bridge-on-the-host-net', 'ar',
     "               and any(lk['net'] not in host_nets for lk in ls)}",
     "               and True}",
     (T_SA,), 'KILLED'),
    ('suggest-host-may-be-a-member', 'ar',
     "        if c['serves'] and c['serves'] in member_of:",
     "        if False:",
     (T_SA + '::test_no_part_is_host_and_member_and_no_big_ic_member',),
     'KILLED'),
    ('suggest-member-may-be-a-host', 'ar',
     "                if m in host_of:",
     "                if False:",
     (T_SA + '::test_no_part_is_host_and_member_and_no_big_ic_member',),
     'KILLED'),
    ('suggest-grid-host-gets-pin-order', 'ar',
     "        grid = _grid_named(fps[host])",
     "        grid = False",
     (T_H + '::test_a_ball_grid_host_suggests_no_order',
      T_SA + '::test_ulx3s_549r_on_u1',), 'KILLED'),
    ('suggest-decap-row-on-a-shared-rail', 'ar',
     "        if len(caps) < MIN_MEMBERS or len(chips) != 1:",
     "        if len(caps) < MIN_MEMBERS or len(chips) < 1:",
     (T_SA,), 'KILLED'),
    ('suggest-regulator-counts-as-a-rail-chip', 'ar',
     "            if 'power_out' in toks and 'power_in' not in toks:",
     "            if False:",
     (T_SA,), 'KILLED'),
    ('suggest-partition-ignores-the-sheet', 'ar',
     "               groups_mod._sheet_of(fp))",
     "               '')",
     (T_SA + '::test_glasgow_resistor_arrays_and_buffers',), 'KILLED'),
    ('suggestion-row-carries-criterion', 'ar',
     "    row: Dict[str, object] = {'name': c['name'],",
     "    row: Dict[str, object] = {'criterion': c.get('criterion'), "
     "'name': c['name'],",
     (T_SA + '::test_every_suggestion_row_loads_as_pasted',), 'KILLED'),
    ('suggestion-row-without-why', 'ar',
     "                'why': suggestion_why(c)})",
     "                })",
     (T_SA + '::test_every_suggestion_row_loads_as_pasted',), 'KILLED'),

    # ==== floorplan: load refusals ==========================================
    ('load-fixed-pose-duplicate', 'fp',
     "        if ref in seen:",
     "        if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-fixed-pose-basis-unchecked', 'fp',
     "        if basis not in _FIXED_POSE_BASES:",
     "        if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-fixed-pose-and-edge-connector', 'fp',
     "        if ref in edge_refs:",
     "        if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-fixed-pose-and-must-lock', 'fp',
     "        pat = _must_lock_hit(ref, must_lock)",
     "        pat = None",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-array-of-one', 'fp',
     "                              f\"references, got {members!r}\")\n"
     "        if len(members) < 2:",
     "                              f\"references, got {members!r}\")\n"
     "        if len(members) < 1:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-array-member-in-two-rows', 'fp',
     "            if m in owner:",
     "            if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-array-member-fixed', 'fp',
     "            if m in fixed_refs:",
     "            if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-array-member-must-lock', 'fp',
     "            pat = _must_lock_hit(m, must_lock)",
     "            pat = None",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-array-member-edge-connector', 'fp',
     "            if m in edge_refs:",
     "            if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-array-serves-a-member', 'fp',
     "        if serves in members:",
     "        if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-pin-order-without-serves', 'fp',
     "        if order == 'pin' and (serves is None or serves == 'unknown'):",
     "        if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-array-pitch-unbounded', 'fp',
     "            if not math.isfinite(v) or v <= 0:",
     "            if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),
    ('load-array-duplicate-name', 'fp',
     "        if name in names:",
     "        if False:",
     (T_AS + '::test_every_load_refusal_carries_its_reason',), 'KILLED'),

    # ==== floorplan: array_problems, the grade and arming ===================
    ('problems-locked-member-allowed', 'fp',
     "            if getattr(fps[m], 'locked', False) or m in locked:",
     "            if False:",
     (T_AS + '::test_board_aware_findings_name_the_member',), 'KILLED'),
    ('problems-mixed-footprints-allowed', 'fp',
     "                if names[m] != first:",
     "                if False:",
     (T_AS + '::test_board_aware_findings_name_the_member',), 'KILLED'),
    ('problems-block-rotation-conflict-ignored', 'fp',
     "                if not any(arr._ang_diff(x, want) < 1e-9 for x in ok_set):",
     "                if False:",
     (T_AS + '::test_board_aware_findings_name_the_member',), 'KILLED'),
    ('problems-shared-rotation-conflict-ignored', 'fp',
     "            if not common:",
     "            if False:",
     (T_AS + '::test_a_shared_rotation_no_member_can_take_is_a_conflict',),
     'KILLED'),
    ('problems-split-zones-allowed', 'fp',
     "                if in_zone[m] != zones_used or len(zones_used) > 1:",
     "                if False:",
     (T_AS + '::test_board_aware_findings_name_the_member',), 'KILLED'),
    ('problems-not-raised-by-grade', 'fp',
     "    violations.extend(array_problems(intent, pcb_data, blocks,",
     "    (array_problems(intent, pcb_data, blocks,",
     (T_AS + '::test_board_aware_findings_name_the_member',), 'KILLED'),
    ('problems-not-raised-by-the-gate', 'fp',
     "    problems.extend(_aprobs)",
     "    problems.extend([])",
     (T_AS + '::test_board_aware_findings_name_the_member',), 'KILLED'),
    ('array-formation-never-armed', 'fp',
     "        return bool(intent.arrays)",
     "        return False",
     (T_AS + '::test_the_new_keys_load_and_land_on_the_intent',
      T_AS + '::test_array_formation_grades_the_as_built_board'), 'KILLED'),
    ('array-formation-always-armed', 'fp',
     "        return bool(intent.arrays)",
     "        return True",
     (T_AS + '::test_the_new_keys_load_and_land_on_the_intent',), 'KILLED'),
    ('array-formation-grades-a-row-missing-a-part', 'fp',
     "        if absent:",
     "        if False:",
     (T_H + '::test_a_row_naming_a_missing_ref_is_skipped_as_absent',
      T_AS + '::test_a_missing_member_skips_the_formation_and_stacked_reads_so',),
     'KILLED'),
    # Killed by the KeyError the measurement below raises for a member with
    # no geometry -- a crash, which is what this row shows: the guard is
    # all that stands between such a member and it.
    ('array-formation-no-geometry-reaches-the-measurement', 'fp',
     "        missing = [m for m in spec['members'] if m not in ctx.parts]",
     "        missing = []",
     (T_AS + '::test_a_member_with_no_geometry_gets_a_measured_row',),
     'KILLED'),
    ('array-formation-no-geometry-not-abstained', 'fp',
     "            ctx.abstained[f\"arrays[{name}]\"] = why",
     "            pass",
     (T_AS + '::test_a_member_with_no_geometry_gets_a_measured_row',),
     'KILLED'),
    ('array-formation-no-geometry-row-not-disclosed', 'fp',
     "            ctx.array_measured.append({'name': name, 'formed': None,\n"
     "                                       'skipped': why})",
     "            pass",
     (T_AS + '::test_a_member_with_no_geometry_gets_a_measured_row',),
     'KILLED'),
    ('array-formation-unresolved-order-not-abstained', 'fp',
     "        if order_refs is not None and 'order' in v['unchecked']:",
     "        if False:",
     (T_AS + '::test_an_order_nobody_resolves_is_unchecked_not_passed',),
     'KILLED'),
    ('array-formation-grade-strict-rotation', 'fp',
     "                          'pads': arr._copper_pad_count(fps[m])})",
     "                          'pads': None})",
     (T_AS + '::test_two_pad_parts_are_the_same_turned_180',), 'KILLED'),
    ('rigid-true-blocks-not-in-the-bundle', 'fp',
     "        if z.rigid:",
     "        if False:",
     (T_AS + '::test_the_gate_bundle_carries_the_new_data',
      T_QB + '::test_rigid_true_block_moves_as_one'), 'KILLED'),

    # ==== floorplan: the fixed-pose grade ===================================
    ('fixed-grade-unknown-ref-clean', 'fp',
     "    for ref in sorted(poses):",
     "    for ref in []:",
     (T_AS + '::test_every_fixed_pose_is_graded',), 'KILLED'),
    ('fixed-grade-skips-any-mechanical-ref', 'fp',
     "        if fp_ is not None and _rc.same_pose(",
     "        if fp_ is not None and (lambda *_a: True)(",
     (T_AS + '::test_every_fixed_pose_is_graded',), 'KILLED'),
    ('fixed-grade-grades-the-files-pose-twice', 'fp',
     "        if fp_ is not None and _rc.same_pose(",
     "        if False and _rc.same_pose(",
     (T_AS + '::test_every_fixed_pose_is_graded',
      T_AS + '::test_an_agreeing_brief_pose_corroborates_the_mechanical_one'),
     'KILLED'),
    ('fixed-grade-lost-value-ungraded', 'fp',
     "            if ref not in lost:",
     "            if True:",
     (T_AS + '::test_the_lost_error_is_for_a_mechanical_entry_only',),
     'KILLED'),
    ('fixed-grade-lost-error-on-declared-too', 'fp',
     "            if f.get('basis') == 'mechanical':",
     "            if True:",
     (T_AS + '::test_the_lost_error_is_for_a_mechanical_entry_only',),
     'KILLED'),
    ('fixed-grade-not-called', 'fp',
     "    violations.extend(fixed_pose_violations(",
     "    (fixed_pose_violations(",
     (T_AS + '::test_every_fixed_pose_is_graded',), 'KILLED'),

    # ==== seeder: stage 0, the fixed poses ==================================
    ('fixed-abutting-refused', 'sd',
     "    ra, ta = pa.rect(*pose_a), pa.tht_rect(*pose_a)",
     "    ra, ta = tuple(v + d for v, d in zip(pa.rect(*pose_a), "
     "(-0.02, -0.02, 0.02, 0.02))), pa.tht_rect(*pose_a)",
     (T_SEED + '::test_abutting_fixed_poses_seat_and_overlapping_ones_both_refuse',),
     'KILLED'),
    ('fixed-overlap-allowed', 'sd',
     "            if area > FIXED_OVERLAP_EPS_MM2:",
     "            if area > 1e9:",
     (T_SEED + '::test_abutting_fixed_poses_seat_and_overlapping_ones_both_refuse',
      T_SEED + '::test_a_real_overlap_is_refused_with_its_measurement'),
     'KILLED'),
    # ---- #1060: fixed_poses[].accept_courtyard_overlap ---------------------
    ('waiver-waives-the-copper-too', 'sd',
     "        opose = obstacles[other]",
     "        opose = obstacles[other]\n"
     "        if frozenset((ref, other)) in waived:\n"
     "            continue",
     (T_SEED + '::test_the_waiver_covers_courtyards_only_both_ways',), 'KILLED'),
    ('waiver-is-one-directional', 'sd',
     "                if frozenset((ref, other)) in waived:",
     "                if frozenset((ref, other)) in waived and ref < other:",
     (T_SEED + '::test_the_waiver_covers_courtyards_only_both_ways',), 'KILLED'),
    ('waiver-of-an-absent-ref-seats', 'sd',
     "    if absent:",
     "    if False:",
     (T_SEED + '::test_a_waiver_the_board_cannot_honour_is_refused',), 'KILLED'),
    ('waiver-not-disclosed', 'sd',
     "            seated[ref]['courtyard_waived'] = waived_hits[ref]",
     "            pass",
     (T_SEED + '::test_a_named_courtyard_waiver_seats_u30_exactly',), 'KILLED'),
    ('waiver-needs-no-why', 'fp',
     "            if acc and not str(f.get('why') or '').strip():",
     "            if False:",
     (T_SEED + '::test_a_waiver_the_board_cannot_honour_is_refused',), 'KILLED'),
    ('waiver-never-reaches-waiver-pairs', 'fp',
     "            for other in f.get('accept_courtyard_overlap') or ():",
     "            for other in ():",
     (T_SEED + '::test_a_named_courtyard_waiver_seats_u30_exactly',), 'KILLED'),
    ('waiver-waives-the-holes-too', 'sd',
     "                    _dh = _drill_conflict(state, ref, pose, other, opose)",
     "                    _dh = None",
     (T_SEED + '::test_the_waiver_does_not_waive_stacked_holes',), 'KILLED'),
    ('waiver-not-graded', 'fp',
     "    out: List[Violation] = list(_fixed_pose_waiver_findings(",
     "    out: List[Violation] = [] and list(_fixed_pose_waiver_findings(",
     (T_SEED + '::test_a_waiver_is_graded_even_where_the_mechanical_anchor_grades_the_pose',
      T_SEED + '::test_a_named_courtyard_waiver_seats_u30_exactly',
      T_SEED + '::test_a_waiver_the_board_cannot_honour_is_refused'),
     'KILLED'),
    ('fixed-declared-poses-not-each-others-obstacles', 'sd',
     "        obstacles.update({o: p for o, p in declared.items() if o != ref})",
     "        pass",
     (T_SEED + '::test_abutting_fixed_poses_seat_and_overlapping_ones_both_refuse',),
     'KILLED'),
    ('fixed-declared-poses-judged-in-ref-order', 'sd',
     "        obstacles.update({o: p for o, p in declared.items() if o != ref})",
     "        obstacles.update({o: p for o, p in declared.items() if o < ref})",
     (T_SEED + '::test_abutting_fixed_poses_seat_and_overlapping_ones_both_refuse',),
     'KILLED'),
    ('fixed-clash-not-named', 'sd',
     "        clash = sorted(o for o in conflicts if o in declared)",
     "        clash = []",
     (T_SEED + '::test_abutting_fixed_poses_seat_and_overlapping_ones_both_refuse',),
     'KILLED'),
    ('fixed-pad-clearance-unchecked', 'sd',
     "            if what:",
     "            if False:",
     (T_H + '::test_declared_poses_whose_courtyards_clear_but_pads_collide_are_refused',
      T_SEED + '::test_illegal_fixed_pose_is_refused_not_nudged',
      T_SEED + '::test_a_real_overlap_is_refused_with_its_measurement'),
     'KILLED'),
    ('fixed-outline-unchecked', 'sd',
     "    outside = state.edge_gate.rect_outside_amount(r) > 1e-9",
     "    outside = False",
     (T_SEED + '::test_illegal_fixed_pose_is_refused_not_nudged',
      T_SEED + '::test_padless_fixed_pose_is_judged_by_its_hole'), 'KILLED'),
    ('fixed-holes-unchecked', 'sd',
     "        if holes_out:",
     "        if False:",
     (T_SEED + '::test_padless_fixed_pose_is_judged_by_its_hole',), 'KILLED'),
    ('fixed-padless-courtyard-unchecked', 'sd',
     "        if not pad_boxes(geometry, ref) and not n_holes:",
     "        if False:",
     (T_H + '::test_a_copperless_fixed_pose_past_the_outline_is_refused',
      T_SEED + '::test_padless_fixed_pose_is_judged_by_its_hole',), 'KILLED'),
    ('fixed-pose-not-exact', 'sd',
     "    x, y = round(float(f['x']), 3), round(float(f['y']), 3)",
     "    x, y = round(float(f['x']) + 0.05, 3), round(float(f['y']), 3)",
     (T_SEED + '::test_fixed_pose_exact_locked_and_survives_repair_and_force',),
     'KILLED'),
    ('fixed-seat-not-locked', 'sd',
     "            'lock_refs': sorted(set(lock_refs) | fixed_lock),",
     "            'lock_refs': sorted(set(lock_refs)),",
     (T_SEED + '::test_fixed_pose_exact_locked_and_survives_repair_and_force',),
     'KILLED'),
    ('fixed-declared-side-ignored', 'sd',
     "    if not side_kept and side_decl != side_now:",
     "    if False:",
     (T_SEED + '::test_illegal_fixed_pose_is_refused_not_nudged',), 'KILLED'),
    ('fixed-file-lock-off-pose-counts-as-there', 'sd',
     "        at = (math.hypot(part.x - x, part.y - y) <= FIXED_POSE_EPS_MM",
     "        at = (True",
     (T_SEED + '::test_every_unhonoured_fixed_pose_fails_the_gate',), 'KILLED'),
    ('fixed-refused-not-unseated', 'sd',
     "    unseated = list(unseated) + sorted(held)",
     "    unseated = list(unseated)",
     (T_SEED + '::test_illegal_fixed_pose_is_refused_not_nudged',), 'KILLED'),
    ('held-seated-by-a-later-stage', 'sd',
     "        return sorted((r for r in refs if r in unplaced and r not in held),",
     "        return sorted((r for r in refs if r in unplaced),",
     (T_SEED + '::test_illegal_fixed_pose_is_refused_not_nudged',
      T_SEED + '::test_refused_fixed_pose_stays_unwritten_under_anchors_first'),
     'KILLED'),
    ('held-is-an-anchors-first-anchor', 'sd',
     "                          if r not in held\n"
     "                          and part_extent_mm(state, r) >= thr),",
     "                          if True\n"
     "                          and part_extent_mm(state, r) >= thr),",
     (T_SEED + '::test_refused_fixed_pose_stays_unwritten_under_anchors_first',),
     'KILLED'),
    ('fixed-seated-evictable', 'sd',
     "        immovable.update({r: 'fixed_pose' for r in fixed_seated",
     "        immovable.update({r: 'fixed_pose' for r in ()",
     (T_H + '::test_a_seated_fixed_pose_is_frozen_to_the_eviction_rung',
      T_SEED + '::test_fixed_pose_exact_locked_and_survives_repair_and_force',),
     'KILLED'),

    # ==== seeder: stage 2.4 / 2.45, the rows ================================
    ('row-member-seated-alone-first', 'sd',
     "        want24 -= array_members",
     "        pass",
     (T_SEED + '::test_a_row_member_is_never_seated_alone_before_its_row',),
     'KILLED'),
    ('row-refused-by-the-intent-check-still-seated', 'sd',
     "            if _an in _aprobs:",
     "            if False:",
     (T_H + '::test_a_row_the_intent_check_refuses_or_with_a_placed_member_is_not_seated',
      T_SEED + '::test_unseatable_row_is_disclosed_and_falls_through',),
     'KILLED'),
    ('row-with-a-placed-member-still-seated', 'sd',
     "            elif any(m not in unplaced or m in held for m in _am):",
     "            elif False:",
     (T_H + '::test_a_row_the_intent_check_refuses_or_with_a_placed_member_is_not_seated',
      T_SEED + '::test_unseatable_row_is_disclosed_and_falls_through',),
     'KILLED'),
    ('row-member-claimed-by-the-decap-stage', 'sd',
     "            decap_scope -= array_members",
     "            pass",
     (T_H + '::test_a_declared_row_of_caps_is_not_the_decap_stages_to_claim',
      T_SEED + '::test_decaps_armed_claims_caps_once_the_owners_are_seated',),
     'KILLED'),
    ('zoned-row-zone-packed-first', 'sd',
     "                   if r not in decap_scope and r not in array_zoned_members]",
     "                   if r not in decap_scope]",
     (T_H + '::test_a_zoned_row_that_seats_nowhere_is_not_zone_packed_again',
      T_SEED + '::test_glasgow_resistor_pair_and_buffer_bank_are_formed'),
     'KILLED'),
    ('seat-block-pose-cap-off', 'sd',
     "                        if tried >= cap:",
     "                        if False:",
     (T_SEED + '::test_pose_cap_trips_and_says_so',), 'KILLED'),
    ('seat-block-sibling-recheck-off', 'sd',
     "                        if _siblings_ok(state, poses, rot,",
     "                        if True or _siblings_ok(state, poses, rot,",
     (T_SEED + '::test_sibling_recheck_reverts_a_row_whose_pads_collide',),
     'KILLED'),
    ('seat-block-sibling-recheck-excludes-the-row', 'sd',
     "                                        exclude - set(order)):",
     "                                        exclude):",
     (T_SEED + '::test_sibling_recheck_reverts_a_row_whose_pads_collide',),
     'KILLED'),
    ('seat-block-failed-row-not-reverted', 'sd',
     "                            state.apply_move(m, bx, by, br)",
     "                            pass",
     (T_H + '::test_a_refused_row_is_put_back_where_it_was',
      T_SEED + '::test_sibling_recheck_reverts_a_row_whose_pads_collide',),
     'KILLED'),
    ('seat-block-fine-rings-before-the-sweep', 'sd',
     "            for bname in ('ring', 'sweep', 'fine', 'xfine'):",
     "            for bname in ('ring', 'fine', 'xfine', 'sweep'):",
     (T_SEED + '::test_row_seat_reaches_the_sweep_before_the_fine_rings',),
     'KILLED'),
    ('seat-block-pitch-precheck-off', 'sd',
     "                if pitch - ext < clr + 1e-6:",
     "                if False:",
     (T_SEED + '::test_sibling_recheck_reverts_a_row_whose_pads_collide',
      T_SEED + '::test_unseatable_row_is_disclosed_and_falls_through'),
     'KILLED'),
    ('seat-block-pitch-margin-zero', 'sd',
     "ROW_PITCH_MARGIN_MM = 0.01",
     "ROW_PITCH_MARGIN_MM = 0.0",
     (T_SEED + '::test_splitflap_u4_row_is_formed',
      T_SEED + '::test_glasgow_resistor_pair_and_buffer_bank_are_formed'),
     'KILLED'),
    ('row-offsets-by-origin-not-courtyard', 'sd',
     "        out.append((s - cx, -cy) if axis == 'x' else (-cx, s - cy))",
     "        out.append((s, 0.0) if axis == 'x' else (0.0, s))",
     (T_H + '::test_row_offsets_line_up_courtyard_centres_not_origins',
      T_SEED + '::test_splitflap_u4_row_is_formed',
      T_SEED + '::test_glasgow_resistor_pair_and_buffer_bank_are_formed'),
     'KILLED'),
    ('row-target-the-host-pins', 'sd',
     "    if cs:\n        tx = sum(c[0] for c in cs) / len(cs)",
     "    if False:\n        tx = sum(c[0] for c in cs) / len(cs)",
     (T_SEED + '::test_row_target_is_the_partner_centroid_not_the_host_pins',),
     'KILLED'),
    ('row-direction-never-reversed', 'sd',
     "        return len(ends) >= 2 and ends[0] > ends[-1]",
     "        return False",
     (T_SEED + '::test_row_runs_the_way_its_host_pins_run',), 'KILLED'),
    ('row-axis-perpendicular-to-the-pins', 'sd',
     "            first = 'y' if abs(dx) >= abs(dy) else 'x'",
     "            first = 'x' if abs(dx) >= abs(dy) else 'y'",
     (T_H + '::test_the_row_runs_parallel_to_the_side_its_pins_are_on',
      T_SEED + '::test_row_runs_the_way_its_host_pins_run',), 'KILLED'),
    ('row-order-ignores-the-pins', 'sd',
     "    if spec['order'] in ('pin', 'declared') and order_refs:",
     "    if False:",
     (T_H + '::test_a_shuffled_declaration_is_seated_in_pin_order',
      T_SEED + '::test_splitflap_u4_row_is_formed',), 'KILLED'),
    ('row-rotation-no-mod-180-dedupe', 'sd',
     "        if all(arr.rotation_period(pads[m]) == 180.0 for m in members):",
     "        if False:",
     (T_H + '::test_a_two_pad_row_tries_each_angle_once_modulo_180',
      T_SEED + '::test_splitflap_u4_row_is_formed',
      T_SEED + '::test_glasgow_resistor_pair_and_buffer_bank_are_formed'),
     'KILLED'),
    ('row-self-check-always-formed', 'sd',
     "        'verdict': 'formed' if v['formed'] else 'broken',",
     "        'verdict': 'formed',",
     (T_H + '::test_the_seeded_verdict_is_the_self_checks',
      T_SEED + '::test_splitflap_u4_row_is_formed',
      T_SEED + '::test_glasgow_resistor_pair_and_buffer_bank_are_formed'),
     'KILLED'),
    ('formed-row-evictable', 'sd',
     "        immovable.update({m: f\"array:{n}\" for n, rec in arrays_formed.items()",
     "        immovable.update({m: f\"array:{n}\" for n, rec in {}.items()",
     (T_SEED + '::test_formed_rows_are_immovable_to_the_eviction_rung',),
     'KILLED'),
    ('formed-row-in-the-anchor-rounds', 'sd',
     "                        or ref in row_members):",
     "                        or False):",
     (T_SEED + '::test_anchor_rounds_leave_a_formed_row_whole',), 'KILLED'),
    ('decap-zero-claim-silent', 'sd',
     "        if decap_scope and not decap_claimed:",
     "        if False:",
     (T_SEED + '::test_zero_claim_reports_why',
      T_SEED + '::test_the_decap_stage_says_why_it_claims_nothing'),
     'KILLED'),
    ('decap-zero-claim-reason-missing', 'sd',
     "        elif not decap_claimed:",
     "        elif False:",
     (T_SEED + '::test_zero_claim_reports_why',
      T_SEED + '::test_the_decap_stage_says_why_it_claims_nothing'),
     'KILLED'),
    ('anchors-queue-ties-by-hash', 'sd',
     "                          and part_extent_mm(state, r) >= thr),\n"
     "                         key=lambda r: (-part_extent_mm(state, r), r))",
     "                          and part_extent_mm(state, r) >= thr),\n"
     "                         key=lambda r: -part_extent_mm(state, r))",
     (T_DET + '::test_seed_and_polish_are_hash_seed_independent',), 'KILLED'),
    ('anchor-rounds-ties-by-hash', 'sd',
     "                            key=lambda r: (-part_extent_mm(state, r), r))",
     "                            key=lambda r: -part_extent_mm(state, r))",
     (T_DET + '::test_seed_and_polish_are_hash_seed_independent',), 'KILLED'),

    # ==== quench: rigid groups ==============================================
    ('merge-ref-in-two-groups', 'qu',
     "            if owner is None:",
     "            if True:",
     (T_QB + '::test_a_ref_in_two_groups_is_deduped_and_disclosed',),
     'KILLED'),
    ('merge-dedupe-not-disclosed', 'qu',
     "                dropped.setdefault(ref, []).append(name)",
     "                pass",
     (T_QB + '::test_a_ref_in_two_groups_is_deduped_and_disclosed',),
     'KILLED'),
    ('merge-partly-locked-row-translates', 'qu',
     "            if fixed:\n                info['anchored'][name] = fixed",
     "            if False:\n                info['anchored'][name] = fixed",
     (T_H + '::test_a_partly_locked_rigid_group_is_anchored_not_translated',
      T_QB + '::test_rigid_true_block_moves_as_one',
      T_QB + '::test_a_ref_in_two_groups_is_deduped_and_disclosed'), 'KILLED'),
    ('merge-members-not-held', 'qu',
     "                info['held'][r] = name",
     "                pass",
     (T_QB + '::test_member_leaves_only_through_a_disclosed_release',
      T_QB + '::test_rigid_true_block_moves_as_one'), 'KILLED'),
    ('merge-icless-cluster-kept', 'qu',
     "        elif kind == 'tether' and name.split(':', 1)[1] not in refs:",
     "        elif False:",
     (T_QB + '::test_a_ref_in_two_groups_is_deduped_and_disclosed',),
     'KILLED'),
    ('nudge-moves-a-held-member', 'qu',
     "            if ref in held:",
     "            if False:",
     (T_QB + '::test_member_leaves_only_through_a_disclosed_release',
      T_QB + '::test_seeded_row_keeps_formation_through_the_polish'), 'KILLED'),
    ('rigid-swap-across-groups', 'qu',
     "    if ga is None or ga != gb:",
     "    if ga is None:",
     (T_QB + '::test_rigid_swap_rule_stays_inside_one_group',), 'KILLED'),
    ('rigid-swap-positioned-members', 'qu',
     "    return ra not in order and rb not in order",
     "    return True",
     (T_QB + '::test_rigid_swap_rule_stays_inside_one_group',), 'KILLED'),
    ('rigid-swap-block-never', 'qu',
     "        return True                 # a rigid block",
     "        return False                # a rigid block",
     (T_QB + '::test_rigid_swap_rule_stays_inside_one_group',), 'KILLED'),
    ('rigid-swap-rule-not-consulted', 'qu',
     "                        if held and (ra in held or rb in held) and not \\",
     "                        if False and (ra in held or rb in held) and not \\",
     (T_QB + '::test_rigid_swap_rule_stays_inside_one_group',
      T_QB + '::test_seeded_row_keeps_formation_through_the_polish'), 'KILLED'),

    # ==== quench: release and rejoin ========================================
    ('release-never', 'qu',
     "        clause = _release_clause(state, ref,",
     "        clause = None and _release_clause(state, ref,",
     (T_QB + '::test_member_leaves_only_through_a_disclosed_release',),
     'KILLED'),
    ('release-although-a-block-move-clears-it', 'qu',
     "                if _clause_failing(state, ref, shifted,",
     "                if False and _clause_failing(state, ref, shifted,",
     (T_H + '::test_a_member_a_block_move_can_clear_is_not_released',
      T_QB + '::test_member_leaves_only_through_a_disclosed_release',),
     'KILLED'),
    ('release-judges-the-formation-pairs', 'qu',
     "    clause = _clause_failing(state, ref, exclude=members)",
     "    clause = _clause_failing(state, ref, exclude=None)",
     (T_H + '::test_a_formation_tighter_than_the_clearance_is_its_own_business',
      T_QB + '::test_member_leaves_only_through_a_disclosed_release',
      T_QB + '::test_release_and_rejoin_do_not_oscillate'), 'KILLED'),
    ('release-stops-before-the-member-moves', 'qu',
     "        if moves == 0 and not changed:",
     "        if moves == 0:",
     (T_H + '::test_a_release_in_a_pass_that_moved_nothing_still_gets_its_pass',
      T_QB + '::test_member_leaves_only_through_a_disclosed_release',),
     'KILLED'),
    ('rejoin-without-hysteresis', 'qu',
     "        if pass_num < rec['pass'] + 2 or rec['_anchor'] is None:",
     "        if pass_num < rec['pass'] + 1 or rec['_anchor'] is None:",
     (T_QB + '::test_a_released_member_rejoins_when_clean_and_in_its_slot',),
     'KILLED'),
    ('rejoin-out-of-its-slot', 'qu',
     "        if _slot_of(state, ref, rec['_anchor']) != rec['_slot']:",
     "        if False:",
     (T_H + '::test_a_released_member_that_moved_away_clean_stays_released',
      T_QB + '::test_a_released_member_rejoins_when_clean_and_in_its_slot',),
     'KILLED'),
    ('rejoin-with-a-different-exclusion-set', 'qu',
     "        if _clause_failing(state, ref, exclude=_formation(",
     "        if _clause_failing(state, ref, exclude=set() and _formation(",
     (T_H + '::test_a_formation_tighter_than_the_clearance_is_its_own_business',
      T_QB + '::test_release_and_rejoin_do_not_oscillate',
      T_QB + '::test_a_released_member_rejoins_when_clean_and_in_its_slot'),
     'KILLED'),
    ('formation-keeps-released-members', 'qu',
     "    out = {r['ref'] for r in released if r['group'] == name}",
     "    out = set()",
     (T_QB + '::test_release_and_rejoin_do_not_oscillate',), 'KILLED'),

    # ==== quench: tethers ===================================================
    ('tether-frozen-election', 'qu',
     "                    cands = [(r, self._chip_bounds(r)) for r in rail]",
     "                    cands = [(r, self._chip_bounds(r)) for r in rail "
     "if r == t.data['ic']]",
     (T_QB + '::test_a_cap_elected_beyond_the_radius_may_not_walk_into_another_ics',),
     'KILLED'),
    ('tether-ungraded-pair-never-counts', 'qu',
     "            if ((grade_view or not t.data['graded'])\n"
     "                    and d > t.data['radius'] + legality.EPS):",
     "            if (d > t.data['radius'] + legality.EPS):",
     (T_QB + '::test_a_cap_elected_beyond_the_radius_may_not_walk_into_another_ics',
      T_QB + '::test_tether_terms_equal_the_grader_on_every_pair'), 'KILLED'),
    ('tether-group-move-partners-at-home', 'qu',
     "            override = dict(self._tether_override, **override)",
     "            pass",
     (T_QB + '::test_an_ic_that_would_strand_its_caps_is_refused_the_cluster_moves',),
     'KILLED'),
    ('tether-group-move-no-override', 'qu',
     "            self._tether_override = {",
     "            _unused_override = {",
     (T_QB + '::test_an_ic_that_would_strand_its_caps_is_refused_the_cluster_moves',),
     'KILLED'),
    ('tether-swap-checks-one-half', 'qu',
     "        if len(override) == 1:",
     "        if True:",
     (T_H + '::test_a_swap_is_checked_on_both_halves_in_either_order',
      T_QB + '::test_a_swap_that_strands_a_cap_is_refused_on_the_tether',),
     'KILLED'),
    ('tether-not-monotone', 'qu',
     "            if c > u + legality.EPS:",
     "            if True:",
     (T_H + '::test_a_term_already_past_its_limit_may_improve',
      T_QB + '::test_tether_terms_equal_the_grader_on_every_pair',
      T_QB + '::test_an_ic_that_would_strand_its_caps_is_refused_the_cluster_moves'),
     'KILLED'),
    ('tether-not-in-candidate-valid', 'qu',
     "            return self._tether_gate(ref, x, y, rot)",
     "            return True",
     (T_QB + '::test_an_ic_that_would_strand_its_caps_is_refused_the_cluster_moves',),
     'KILLED'),
    ('swap-intent-ok-without-tethers', 'qu',
     "                and (not self._tether_active",
     "                and (True",
     (T_QB + '::test_a_swap_that_strands_a_cap_is_refused_on_the_tether',),
     'KILLED'),
    # Re-anchored for #1117, which added `or state.declared_rotations` to
    # this condition; the mutation still drops only the tether term.
    ('swap-gate-tethers-only-not-asked', 'qu',
     "                        if ((state._intent_active or state._tether_active\n"
     "                             or state.declared_rotations)",
     "                        if ((state._intent_active\n"
     "                             or state.declared_rotations)",
     (T_H + '::test_a_tethers_only_quench_gates_swaps_and_drops_icless_clusters',
      T_QB + '::test_a_swap_that_strands_a_cap_is_refused_on_the_tether',),
     'KILLED'),
    ('tether-decap-shortcut-unsound', 'qu',
     "                if static is not None and static <= t.threshold + legality.EPS:",
     "                if static is not None:",
     (T_QB + '::test_the_tether_shortcuts_change_no_decision',), 'KILLED'),
    ('tether-pin-shortcut-unsound', 'qu',
     "            if (best is not None and best <= t.threshold + legality.EPS",
     "            if (best is not None",
     (T_QB + '::test_the_tether_shortcuts_change_no_decision',), 'KILLED'),
    ('tether-cluster-without-its-ic', 'qu',
     "            if ic in refs:              # a cluster without its IC moves caps",
     "            if True:                    # a cluster without its IC moves caps",
     (T_H + '::test_a_tethers_only_quench_gates_swaps_and_drops_icless_clusters',
      T_QB + '::test_an_ic_that_would_strand_its_caps_is_refused_the_cluster_moves',),
     'KILLED'),

    # ==== place_seed ========================================================
    ('place-seed-unhonoured-fixed-pose-exits-0', 'ps',
     "        _reason = fixed_pose_reason(summary)",
     "        _reason = None",
     (T_SEED + '::test_every_unhonoured_fixed_pose_fails_the_gate',), 'KILLED'),
    ('place-seed-moved-fixed-pose-honoured', 'ps',
     "                    if not rec.get('at_written_pose', True)})",
     "                    if False})",
     (T_SEED + '::test_every_unhonoured_fixed_pose_fails_the_gate',), 'KILLED'),
    ('place-seed-fixed-pose-judged-at-the-seed', 'ps',
     "        at = (f is not None and abs(f.x - rec['x']) <= 1e-3",
     "        at = (True",
     (T_H + '::test_place_seed_judges_rows_and_fixed_poses_at_the_written_board',
      T_SEED + '::test_every_unhonoured_fixed_pose_fails_the_gate',), 'KILLED'),
    ('place-seed-row-verdict-at-the-seed', 'ps',
     "        m = measured.get(name) or {}",
     "        m = {'formed': rec.get('verdict') == 'formed'}",
     (T_H + '::test_place_seed_judges_rows_and_fixed_poses_at_the_written_board',
      T_QB + '::test_seeded_row_keeps_formation_through_the_polish',),
     'KILLED'),

    # ==== the P1 driver: RETIRED =========================================
    # Five rows (a fixed pose at another pose / without rot / on a locked
    # part / excusing the lock / owing a zone) mutated the staged placement
    # driver's P1 stage, killed by test_959_reconcile's `test_p1_*` driver
    # tests. Driver and tests left the tree when the placement skill was
    # retired for pcb-free-agent.
    # --- the re-review's two survivors (both reached by new assertions) ---
    ('fixed-pad-short-unnamed', 'sd',
     "            if sf.pad_overlap:",
     "            if False:",
     (T_H + '::test_declared_poses_whose_courtyards_clear_but_pads_collide_are_refused',),
     'KILLED'),
    ('route-loop-disclosure-dropped', 'rl',
     "        summary['quench_disclosure'] = list(sink)",
     "        pass",
     (T_QB + '::test_route_loop_summary_carries_each_rounds_quench_disclosure',),
     'KILLED'),
    # --- the re-review's contradiction row (#1051/#1054) ---
    ('mechanical-array-member-not-a-contradiction', 'rc',
     "        if ref not in members or ref not in pcb.footprints:",
     "        if True:",
     (T_959 + '::test_a_brief_array_member_with_a_mechanical_pose_is_a_contradiction',),
     'KILLED'),
]

from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _uncache(path):
    """Delete the target's cached bytecode -- see this module's docstring."""
    import importlib
    import importlib.util
    try:
        cached = importlib.util.cache_from_source(path)
        if os.path.exists(cached):
            os.remove(cached)
    except (OSError, ValueError, NotImplementedError):
        pass
    importlib.invalidate_caches()


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _argv(spec):
    """`(path, [names])` for a killer spec `path[::name...]`."""
    path, _sep, rest = spec.partition('::')
    return path, [n for n in rest.split('::') if n] if rest else []


def _run(spec):
    path, names = _argv(spec)
    p = subprocess.run([sys.executable, '-X', 'utf8', path] + names,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=1800, cwd=_ROOT)
    return p, names


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit or stash '
                  'first -- this battery restores by overwriting.'
                  % os.path.basename(path))
            return 2

    t0 = time.time()
    # The battery is only evidence if every killer passes UNMUTATED first,
    # and only if every name it selects is a test that RAN: a substring
    # filter that matches nothing runs nothing and exits 0.
    for path in TARGETS.values():
        _uncache(path)
    for spec in sorted({t for row in rows for t in row[4]}):
        p, names = _run(spec)
        shown = os.path.basename(spec)
        if p.returncode:
            print('BROKEN: %s fails on the UNMUTATED tree (exit %d); every '
                  'verdict below would be meaningless.' % (shown, p.returncode))
            return 2
        lines = set(p.stdout.splitlines())
        ran = [n for n in names if ('--- ' + n) in lines]
        if len(ran) != len(names) or 'ALL PASS' not in p.stdout:
            print('BROKEN: %s selected %s but ran %s -- a killer that runs '
                  'nothing kills nothing.' % (shown, names, ran))
            return 2
        print('  unmutated %-70s passes' % shown[:70])

    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path, base = TARGETS[tgt], orig[tgt]
            n = base.count(old)
            if n != 1:
                results.append((name, 'BROKEN', expect,
                                'anchor matched %d times' % n, []))
                print('  ran %-52s BROKEN' % name)
                continue
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(old, new, 1))
            _uncache(path)
            killed, failed = False, []
            for t in tests:
                p, _names = _run(t)
                out = (p.stderr or '') + (p.stdout or '')
                failed += [l.strip()[:90] for l in out.splitlines()
                           if l.strip().startswith(('FAIL', 'AssertionError'))]
                if 'Traceback' in out:
                    failed.append('raised: '
                                  + out.strip().splitlines()[-1][:80])
                if p.returncode:
                    killed = True
                    break           # one kill is the verdict; save the rest
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            _uncache(path)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed[:2]))
            print('  ran %-52s %s' % (name, results[-1][1]), flush=True)
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
            _uncache(v)

    print()
    w = max(len(r[0]) for r in results)
    wrong = 0
    for name, verdict, expect, cnt, failed in results:
        mark = ''
        if verdict != expect:
            mark = '   <-- WRONG, expected %s' % expect
            wrong += 1
        print('%-*s  %-9s  %-3s%s' % (w, name, verdict, cnt, mark))
        for f in failed:
            print('%s      %s' % (' ' * w, f))
    killed = sum(1 for r in results if r[1] == 'KILLED')
    survived = sum(1 for r in results if r[1] == 'SURVIVED')
    broken = sum(1 for r in results if r[1] == 'BROKEN')
    print('\n%d rows: %d killed, %d survived (%d of them expected), %d broken'
          % (len(results), killed, survived,
             sum(1 for r in results if r[1] == r[2] == 'SURVIVED'), broken))
    print('wall time %.0fs' % (time.time() - t0))
    if wrong or broken:
        print('%d row(s) did not match their expectation' % (wrong + broken))
    return 1 if (wrong or broken) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', default=None, help='run one row by name')
    ap.add_argument('--list', action='store_true', help='list the row names')
    a = ap.parse_args()
    if a.list:
        for r in ROWS:
            print('%-52s %-4s %s' % (r[0], r[1], r[5]))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
