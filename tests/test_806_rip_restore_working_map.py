#!/usr/bin/env python3
"""#806: a rip, a restore and a recorded reroute refresh the working map for
EVERY net the moved copper is on -- the pair's GND return vias included -- and
a partial (leg-level) restore refreshes its own net at all.

WHAT WAS WRONG. The commit sites in the diff-pair loop were fixed to refresh
`obstacle_cache.committed_net_ids(result, own)` (the pair, then every other
net the result carries), but the SHARED rip/restore primitives kept refreshing
the pair's own ids only:

  * rip_up_net took a pair's copper off the board -- its GND return vias with
    it -- and recomputed P and N only, so GND's entry kept blocking vias that
    were gone (measured on scalenode_cm4's diff step at two layers: after
    ripping HDMI0_D1, GND's entry held 1928 cells and 698 via cells of copper
    no longer on the board);
  * restore_net put them back and recomputed P and N only, so whenever GND's
    entry had been refreshed in between, the restored vias were not blocked;
  * routing_context.record_diff_pair_success (the reroute loop's and the
    casualty reconcile's pair commit) refreshed P and N only -- the same
    defect as the loop's own commits before e318cb19f, which a rerouted pair's
    new return vias went straight through;
  * try_fallback_layer_swap's inline rip / reroute / restore did the same;
  * restore_net's partial-leg branch (#510, KICAD_LEG_RIP=1) put a ripped
    branch back on the board WITHOUT refreshing the net at all: the partial rip
    had recomputed the entry without the branch, so the restored branch was
    unblocked until something else refreshed that net. Measured on the
    flat_hierarchy churn recipe with KICAD_LEG_RIP=1: ten partial restores,
    595 to 58744 cells and 554 to 62129 via cells of the restored branch absent
    from the net's entry each time.

The end-of-run [OBSTACLE CONTENT] audit could see none of this: it reads only
UNDER-blocks, and only at the end, by which time a later refresh of the same
net had healed each entry.

FOUR CHECKS, on the real primitives over the in-repo esp_prog board (no
routing, about a second):
  1. rip_up_net of a pair whose result carries a GND via: GND's entry equals a
     fresh recompute and the via's site is free again.
  2. restore_net of that pair after GND's entry was refreshed meanwhile: the
     via is via-blocked again and GND's entry equals a fresh recompute.
  3. record_diff_pair_success with a GND via: blocked, entry fresh.
  4. a partial-leg rip and restore of a single-ended net: the branch's cells
     are blocked again and the net's entry equals a fresh recompute.
Every check also asserts the ref-counted maps balance (working - caches ==
base), so a fix that double-adds cannot pass.

NEGATIVE CONTROL (measured, the three engine files at the parent commit):
1 fails on "GND entry == fresh after the rip" (874 cells / 553 via cells of
the ripped via still in GND's entry) and "the ripped return via's site is free
again"; 2 on "the restored GND via is via-blocked"; 3 on
"record_diff_pair_success blocks the GND via"; 4 on "the restored branch is
blocked in the working map" (2024 of 2057 branch cells unblocked). The balance
rows PASS on the broken code too -- a stale entry is removed and re-added in
balance -- which is why every check here grades CONTENT.

NOT COVERED HERE: try_fallback_layer_swap's inline rip / reroute / restore got
the same one-line change (committed_net_ids over the moved result) and has no
row of its own; its map is reached only inside a diff-pair layer-swap rip-up.

Run:  python3 tests/test_806_rip_restore_working_map.py
"""
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from run_utils import evidence

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name
          + (f"  [{detail}]" if detail and not cond else ""))
    if not cond:
        FAILS.append(name)


def _cells(cd):
    from obstacle_cache import _cache_cells_and_vias
    c, v = _cache_cells_and_vias(cd)
    return set(map(tuple, c.tolist())), set(map(tuple, v.tolist()))


def _entry_is_fresh(pcb, config, cache, nid):
    """(fresh?, detail): the net's cache entry == a recompute from the board."""
    from obstacle_cache import precompute_net_obstacles
    ec, ev = _cells(cache[nid])
    fc, fv = _cells(precompute_net_obstacles(pcb, nid, config))
    return (ec == fc and ev == fv,
            f"entry-only {len(ec - fc)} cells / {len(ev - fv)} via cells, "
            f"board-only {len(fc - ec)} cells / {len(fv - ev)} via cells")


def _balanced(base, working, cache):
    from obstacle_cache import remove_net_obstacles_from_cache
    probe = working.clone_fresh()
    for cd in cache.values():
        remove_net_obstacles_from_cache(probe, cd)
    bs, ps = base.get_stats(), probe.get_stats()
    return bs[:2] == ps[:2] and bs[7] == ps[7], f"base {bs} probe {ps}"


def _setup():
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig, GridCoord, DiffPairNet
    from obstacle_map import GridObstacleMap
    from obstacle_cache import precompute_net_obstacles, build_working_obstacle_map
    evidence(BOARD, 'esp_prog board')
    pcb = parse_kicad_pcb(BOARD)
    ids = {n.name: nid for nid, n in pcb.nets.items()}
    p_id, n_id, gnd_id = ids['/D_P'], ids['/D_N'], ids['GND']
    config = GridRouteConfig(layers=list(pcb.board_info.copper_layers),
                             grid_step=0.05, track_width=0.2, clearance=0.1778,
                             via_size=0.45, via_drill=0.3)
    base = GridObstacleMap(len(config.layers))
    cache = {nid: precompute_net_obstacles(pcb, nid, config)
             for nid in (p_id, n_id, gnd_id)}
    working = build_working_obstacle_map(base, cache)
    pair = DiffPairNet(base_name='/D', p_net_id=p_id, n_net_id=n_id,
                       p_net_name='/D_P', n_net_name='/D_N')
    return dict(pcb=pcb, config=config, coord=GridCoord(config.grid_step),
                p=p_id, n=n_id, gnd=gnd_id, base=base, cache=cache,
                working=working, pair=pair)


def _legs(ctx, nid, drop):
    """A pad-to-pad run for `nid`: a short stub off its U1 pad, then the long
    leg to its USB1 pad. Pad-to-pad, so the commit's dead-end prune keeps it."""
    from kicad_parser import Segment
    pcb = ctx['pcb']
    u = next(p for p in pcb.nets[nid].pads if p.component_ref == 'U1')
    c = next(p for p in pcb.nets[nid].pads if p.component_ref == 'USB1')

    def seg(x0, y0, x1, y1):
        return Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1, width=0.2,
                       layer='F.Cu', net_id=nid)
    return (seg(u.global_x, u.global_y, u.global_x, u.global_y + drop),
            seg(u.global_x, u.global_y + drop, c.global_x, c.global_y))


def _pair_result(ctx):
    """A routed pair result whose copper includes a GND return via, at a site
    nothing else on the board blocks."""
    from kicad_parser import Via
    pcb, p_id, n_id = ctx['pcb'], ctx['p'], ctx['n']
    pu = next(p for p in pcb.nets[p_id].pads if p.component_ref == 'U1')
    gvia = Via(x=pu.global_x + 4.0, y=pu.global_y + 4.0, size=0.45, drill=0.3,
               layers=['F.Cu', 'B.Cu'], net_id=ctx['gnd'])
    return {'new_segments': [*_legs(ctx, p_id, 0.9), *_legs(ctx, n_id, 1.1)],
            'new_vias': [gvia]}, gvia


def t_rip_and_restore_pair_with_return_via():
    print("\n[1] rip_up_net of a pair carrying a GND return via")
    print("[2] restore_net of it after GND's entry was refreshed meanwhile")
    from pcb_modification import add_route_to_pcb_data
    from obstacle_cache import refresh_net_obstacles, committed_net_ids
    from rip_up_reroute import rip_up_net, restore_net
    ctx = _setup()
    pcb, config, cache, working = ctx['pcb'], ctx['config'], ctx['cache'], ctx['working']
    p_id, n_id, gnd_id = ctx['p'], ctx['n'], ctx['gnd']
    res, gvia = _pair_result(ctx)
    gx, gy = ctx['coord'].to_grid(gvia.x, gvia.y)
    check("precondition: the return via's site is free before the commit",
          not working.is_via_blocked(gx, gy))
    # The routed state the loop leaves (the commit sites are already fixed).
    add_route_to_pcb_data(pcb, res)
    refresh_net_obstacles(working, cache, pcb, config, committed_net_ids(res, (p_id, n_id)))
    check("precondition: the committed GND via is via-blocked",
          working.is_via_blocked(gx, gy))
    routed_net_ids, routed_paths = [p_id, n_id], {}
    routed_results = {p_id: res, n_id: res}
    dp = {p_id: ('/D', ctx['pair']), n_id: ('/D', ctx['pair'])}
    remaining, results = [], [res]

    saved, ripped_ids, was_in = rip_up_net(
        p_id, pcb, routed_net_ids, routed_paths, routed_results, dp, remaining,
        results, config, None, working, cache, None, None, None)
    check("rip_up_net ripped the pair", saved is res and sorted(ripped_ids) == sorted([p_id, n_id]),
          f"saved is res: {saved is res}, ids {ripped_ids}")
    fresh, why = _entry_is_fresh(pcb, config, cache, gnd_id)
    check("GND entry == fresh after the rip (the removed return via is no "
          "longer stamped)", fresh, why)
    check("the ripped return via's site is free again",
          not working.is_via_blocked(gx, gy))
    ok, why = _balanced(ctx['base'], working, cache)
    check("ref-counts balanced after the rip", ok, why)

    # Some other commit refreshes GND while the pair is out (another pair's
    # return vias do exactly this through committed_net_ids).
    refresh_net_obstacles(working, cache, pcb, config, [gnd_id])
    restore_net(p_id, saved, ripped_ids, was_in, pcb, routed_net_ids,
                routed_paths, routed_results, dp, remaining, results, config,
                None, None, working, cache, None, None)
    check("restore_net put the pair back",
          gvia in pcb.vias and routed_results.get(p_id) is res)
    check("the restored GND via is via-blocked", working.is_via_blocked(gx, gy))
    fresh, why = _entry_is_fresh(pcb, config, cache, gnd_id)
    check("GND entry == fresh after the restore", fresh, why)
    ok, why = _balanced(ctx['base'], working, cache)
    check("ref-counts balanced after the restore", ok, why)


def t_record_diff_pair_success_refreshes_return_net():
    print("\n[3] record_diff_pair_success with a GND return via")
    from routing_context import record_diff_pair_success
    ctx = _setup()
    pcb, config, cache, working = ctx['pcb'], ctx['config'], ctx['cache'], ctx['working']
    p_id, n_id, gnd_id = ctx['p'], ctx['n'], ctx['gnd']
    res, gvia = _pair_result(ctx)
    gx, gy = ctx['coord'].to_grid(gvia.x, gvia.y)
    check("precondition: the return via's site is free before the commit",
          not working.is_via_blocked(gx, gy))
    dp = {p_id: ('/D', ctx['pair']), n_id: ('/D', ctx['pair'])}
    record_diff_pair_success(pcb, res, ctx['pair'], '/D', config, [p_id, n_id],
                             [], {}, {}, dp, {}, {},
                             working_obstacles=working, net_obstacles_cache=cache)
    check("record_diff_pair_success blocks the GND via",
          working.is_via_blocked(gx, gy))
    fresh, why = _entry_is_fresh(pcb, config, cache, gnd_id)
    check("GND entry == fresh after the recorded commit", fresh, why)
    ok, why = _balanced(ctx['base'], working, cache)
    check("ref-counts balanced", ok, why)


def t_partial_leg_restore_refreshes_its_net():
    print("\n[4] a partial-leg rip and restore of a single-ended net")
    from pcb_modification import add_route_to_pcb_data
    from obstacle_cache import refresh_net_obstacles, precompute_net_obstacles
    from rip_up_reroute import rip_up_net, restore_net
    ctx = _setup()
    pcb, config, cache, working = ctx['pcb'], ctx['config'], ctx['cache'], ctx['working']
    p_id = ctx['p']   # routed single-ended here: the partial path is per net
    trunk, branch = _legs(ctx, p_id, 0.9)
    res = {'new_segments': [trunk, branch], 'new_vias': []}
    add_route_to_pcb_data(pcb, res)
    refresh_net_obstacles(working, cache, pcb, config, [p_id])
    routed_net_ids, routed_paths, routed_results = [p_id], {}, {p_id: res}
    remaining, results = [], [res]

    partial, ids, was_in = rip_up_net(
        p_id, pcb, routed_net_ids, routed_paths, routed_results, {}, remaining,
        results, config, None, working, cache, None, None, None,
        only_segments=[branch])
    check("rip_up_net took the partial (leg) path",
          bool(partial and partial.get('partial_leg_rip'))
          and branch not in pcb.segments and trunk in pcb.segments)
    ripped_entry = _cells(cache[p_id])[0]
    restore_net(p_id, partial, ids, was_in, pcb, routed_net_ids, routed_paths,
                routed_results, {}, remaining, results, config, None, None,
                working, cache, None, None)
    check("restore_net put the branch back", branch in pcb.segments)
    # The branch's footprint: what the board now says the net owns beyond
    # what its entry held while the branch was out.
    restored = _cells(precompute_net_obstacles(pcb, p_id, config))[0] - ripped_entry
    clone = working.clone_fresh()   # no source/target exemptions
    unblocked = [c for c in restored if not clone.is_blocked(*map(int, c))]
    check("the restored branch is blocked in the working map",
          bool(restored) and not unblocked,
          f"{len(unblocked)} of {len(restored)} branch cells unblocked")
    fresh, why = _entry_is_fresh(pcb, config, cache, p_id)
    check("the net's entry == fresh after the partial restore", fresh, why)
    ok, why = _balanced(ctx['base'], working, cache)
    check("ref-counts balanced", ok, why)


def main():
    t_rip_and_restore_pair_with_return_via()
    t_record_diff_pair_success_refreshes_return_net()
    t_partial_leg_restore_refreshes_its_net()
    print()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s): " + "; ".join(FAILS))
        return 1
    print("PASS: rip / restore / recorded pair commit / partial restore keep "
          "every touched net's map entry fresh")
    return 0


if __name__ == '__main__':
    sys.exit(main())
