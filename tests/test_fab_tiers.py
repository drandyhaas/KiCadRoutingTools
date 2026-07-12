#!/usr/bin/env python3
"""Issue #237: JLC fab floors as selectable cost tiers (standard / advanced) with
an optional override file, plus the floor ladder / escalation / param-floor guards.

Covers fab_tiers.py end to end:
  1. Tier table + ladder shape (standard escalates to advanced; advanced is hard).
  2. fab_floors = nominal, fab_floor_min = deepest; the advanced<=standard via invariant.
  3. Override file parsing (separators, comments, unknown keys, non-positive) and
     overlay semantics (only listed keys; collapses the ladder to one hard rung).
  4. Process-default tier set/get round-trip and that fab_floors(n) honours it.
  5. add_fab_tier_args / fab_tier_from_args (incl. missing-file error).
  6. Param-floor checks: check_param_floors / enforce_fab_floors (CLI), and that a
     bigger override floor is enforced.

Run:  python3 tests/test_fab_tiers.py [-v]
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import fab_tiers as ft


def _vias(ladder):
    return [(round(f['via_diameter'], 4), round(f['via_drill'], 4)) for f in ladder]


def test_tiers_and_ladder(v):
    fails = []
    # standard ladder = [standard, (fine-via rung on 4+ layers), advanced];
    # advanced ladder = [advanced]. The 0.30/0.15 fine-via rung is 4+-layer only
    # (fine BGA-pitch escape), restored on the standard ladder in commit 926bf3f.
    expected_standard = {
        2: [(0.45, 0.20), (0.25, 0.15)],
        4: [(0.45, 0.20), (0.30, 0.15), (0.25, 0.15)],
    }
    for ncu in (2, 4):
        lad = ft.fab_floor_ladder(ncu, 'standard')
        if _vias(lad) != expected_standard[ncu]:
            fails.append(f"{ncu}L standard ladder vias {_vias(lad)} != "
                         f"{expected_standard[ncu]}")
        adv = ft.fab_floor_ladder(ncu, 'advanced')
        if len(adv) != 1 or _vias(adv) != [(0.25, 0.15)]:
            fails.append(f"{ncu}L advanced ladder {_vias(adv)} unexpected")
    # nominal vs deepest
    if ft.fab_floors(4, 'standard')['via_diameter'] != 0.45:
        fails.append("standard nominal via != 0.45")
    if ft.fab_floor_min(4, 'standard')['via_diameter'] != 0.25:
        fails.append("standard deepest via != 0.25")
    # advanced via must never exceed standard via (the escalation invariant)
    for ncu in (2, 4):
        if ft.fab_floor_min(ncu, 'advanced')['via_diameter'] > ft.fab_floors(ncu, 'standard')['via_diameter']:
            fails.append(f"{ncu}L advanced via > standard via (breaks invariant)")
    # track floors per the spec (standard preserves historical values)
    if ft.fab_floors(2, 'standard')['track_width'] != 0.127:
        fails.append("2L standard track != 0.127 (must preserve current default)")
    if abs(ft.fab_floors(4, 'standard')['track_width'] - 0.0889) > 1e-9:
        fails.append("4L standard track != 0.0889")
    # pad hole-to-hole modelled separately from via hole-to-hole
    f = ft.fab_floors(4, 'standard')
    if f['hole_to_hole'] != 0.20 or f['pad_hole_to_hole'] != 0.45:
        fails.append("via/pad hole-to-hole split wrong")
    if v and not fails:
        print("  tiers/ladder OK")
    return fails


def test_overrides(v):
    fails = []
    p = os.path.join(ROOT, "tests", "_tmp_fab_ovr.txt")
    with open(p, "w") as fh:
        fh.write("# fab override\nvia_drill = 0.18\nclearance: 0.08\ntrack_width 0.07\n"
                 "bogus 1.0\nvia_diameter -1\n")
    try:
        ovr = ft.parse_fab_overrides(p)
        if ovr != {'via_drill': 0.18, 'clearance': 0.08, 'track_width': 0.07}:
            fails.append(f"parsed overrides wrong: {ovr}")
        # overlay: only listed keys change; ladder collapses to ONE hard rung
        lad = ft.fab_floor_ladder(4, 'standard', ovr)
        if len(lad) != 1:
            fails.append(f"override ladder not single-rung: len {len(lad)}")
        only = lad[0]
        if only['via_drill'] != 0.18 or only['clearance'] != 0.08 or only['track_width'] != 0.07:
            fails.append("override values not applied")
        if only['via_diameter'] != 0.45:
            fails.append("override changed an UNLISTED key (via_diameter)")
        # overlay onto advanced base keeps advanced's unlisted keys
        adv = ft.fab_floor_ladder(4, 'advanced', ovr)[0]
        if adv['via_diameter'] != 0.25:
            fails.append("override on advanced lost the advanced via")
    finally:
        os.remove(p)
    if v and not fails:
        print("  overrides OK")
    return fails


def test_default_tier_state(v):
    fails = []
    prev = ft.get_default_fab_tier()
    try:
        ft.set_default_fab_tier('advanced')
        if ft.fab_floors(4)['via_diameter'] != 0.25:
            fails.append("set_default advanced not honoured by fab_floors(n)")
        if len(ft.fab_floor_ladder(4)) != 1:
            fails.append("advanced default ladder not single-rung")
        ft.set_default_fab_tier('standard', {'via_drill': 0.30})
        t, o = ft.get_default_fab_tier()
        if t != 'standard' or o != {'via_drill': 0.30}:
            fails.append(f"get_default round-trip wrong: {t} {o}")
        # default overrides collapse the ladder too
        if len(ft.fab_floor_ladder(4)) != 1:
            fails.append("default override ladder not single-rung")
    finally:
        ft.set_default_fab_tier(*prev)
    if v and not fails:
        print("  default-tier state OK")
    return fails


def test_args(v):
    fails = []
    p = argparse.ArgumentParser()
    ft.add_fab_tier_args(p)
    t, o = ft.fab_tier_from_args(p.parse_args(['--fab-tier', 'advanced']))
    if t != 'advanced' or o != {}:
        fails.append(f"advanced args wrong: {t} {o}")
    # missing override file -> clean SystemExit
    try:
        ft.fab_tier_from_args(p.parse_args(['--fab-overrides', '/no/such/file.txt']))
        fails.append("missing override file did not raise")
    except SystemExit:
        pass
    if v and not fails:
        print("  args OK")
    return fails


def test_param_floors(v):
    fails = []
    # 4-layer deepest floors: track 0.0762, clearance 0.09, via 0.25, drill 0.15, h2h 0.20
    viols = ft.check_param_floors(4, 'standard', track_width=0.05, clearance=0.2,
                                  via_size=0.2, via_drill=0.1, hole_to_hole_clearance=0.1)
    names = {n for n, _, _ in viols}
    if names != {'track_width', 'via_size', 'via_drill', 'hole_to_hole_clearance'}:
        fails.append(f"check_param_floors flagged {names}")
    # at/above floor -> no violation
    if ft.check_param_floors(4, 'standard', via_size=0.25, track_width=0.0762):
        fails.append("at-floor values wrongly flagged")
    # enforce pins up to the floor and reports the clamp (issue #237: warn + pin,
    # don't abort the run -- the fab can't make sub-floor, so clamp and continue).
    pinned = ft.enforce_fab_floors(4, 'standard', via_drill=0.05)
    if pinned.get('via_drill') != ft.fab_floor_min(4, 'standard')['via_drill']:
        fails.append(f"enforce_fab_floors did not pin sub-floor via_drill up to "
                     f"the floor (got {pinned})")
    # at/above-floor params return no clamps
    if ft.enforce_fab_floors(4, 'standard', via_drill=0.2):
        fails.append("enforce_fab_floors clamped an at/above-floor via_drill")
    # an override lowering the floor lets the value through
    if ft.check_param_floors(4, 'standard', {'via_drill': 0.04}, via_drill=0.05):
        fails.append("override-lowered floor still flagged 0.05 drill")
    if v and not fails:
        print("  param floors OK")
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    fails = []
    for name, fn in (("tiers/ladder", test_tiers_and_ladder),
                     ("overrides", test_overrides),
                     ("default-tier state", test_default_tier_state),
                     ("args", test_args),
                     ("param floors", test_param_floors)):
        print(f"=== {name} ===")
        fails += fn(args.verbose)
    if fails:
        print("\nFAIL:\n  " + "\n  ".join(fails))
        return 1
    print("\nPASS: fab tiers, ladder, overrides, default state, args, and param floors")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
