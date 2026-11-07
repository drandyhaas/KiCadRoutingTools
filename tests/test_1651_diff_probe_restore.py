#!/usr/bin/env python3
"""#1651: undoing a diff-pair rip in the main diff-pair loop puts back what
the rip took.

`route_diff_pairs` rips a routed blocker to clear a probe (or a connector
graze), and when the retry does not route clean it restores the blocker.
`rip_up_net` drops the blocker's routed paths and swaps its working-map entry
for a stubs-only one; the restore used to be a second, partial inverse
(`restore_ripped_net`) that put back the copper, routed_net_ids and
routed_results but neither of those. So after an undone probe rip:

  * the restored pair was missing from routed_net_paths for the rest of the
    run -- blocker analysis could no longer name it, and an emptied map is
    what reached the reroute loop's unbound-variable crash (#1650);
  * the working map still held the stubs-only entry, so a later search on
    that map saw no restored copper (measured with KICAD_OBSTACLE_AUDIT on
    the issue's board: the restored pair's two nets were the only stale
    entries before the end-of-run sync).

Every undo now goes through `restore_net`, rip_up_net's own inverse. This
test drives the real loop over the probe rip -> failed retry -> restore path
(router and obstacle builders stubbed, rip_up_net / restore_net real) and
stops it at the failure classification that follows the restore.

    python3 tests/test_1651_diff_probe_restore.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from routing_config import GridRouteConfig, DiffPairNet     # noqa: E402
from routing_state import RoutingState                     # noqa: E402
from blocking_analysis import BlockingInfo                 # noqa: E402
from synth import make_pcb, make_net, make_seg, make_pad   # noqa: E402
import diff_pair_loop                                      # noqa: E402
import rip_up_reroute                                      # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{': ' + detail if detail else ''}")
    if not ok:
        failures.append(label)


class _Stop(Exception):
    """Raised at the first step after the probe ladder's restore."""


def pair(base, p_id, n_id):
    return DiffPairNet(base_name=base, p_net_id=p_id, n_net_id=n_id,
                       p_net_name=f"{base}_P", n_net_name=f"{base}_N")


def build():
    a, b = pair('A', 1, 2), pair('B', 3, 4)
    nets = {i: make_net(i, n) for i, n in
            ((1, 'A_P'), (2, 'A_N'), (3, 'B_P'), (4, 'B_N'))}
    b_segs = [make_seg(0, 0, 10, 0, net_id=3), make_seg(0, 0.4, 10, 0.4, net_id=4)]
    cfg = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                          via_drill=0.3, layers=['F.Cu', 'B.Cu'],
                          grid_step=0.05)
    # Pads at both ends of each track, so the commit's dead-end prune keeps it.
    pads = {3: [make_pad(3, 0, 0, ref='U1', num='1'), make_pad(3, 10, 0, ref='U2', num='1')],
            4: [make_pad(4, 0, 0.4, ref='U1', num='2'), make_pad(4, 10, 0.4, ref='U2', num='2')]}
    st = RoutingState(pcb_data=make_pcb(nets=nets, segments=list(b_segs),
                                        pads_by_net=pads),
                      config=cfg)
    st.layer_map = {'F.Cu': 0, 'B.Cu': 1}
    p_path, n_path = [(0, 0, 0), (200, 0, 0)], [(0, 8, 0), (200, 8, 0)]
    b_res = {'new_segments': list(b_segs), 'new_vias': [], 'iterations': 1,
             'p_path': p_path, 'n_path': n_path}
    for nid in (3, 4):
        st.routed_net_ids.append(nid)
        st.routed_results[nid] = b_res
        st.diff_pair_by_net_id[nid] = ('B', b)
    st.routed_net_paths.update({3: p_path, 4: n_path})
    st.results.append(b_res)
    st.remaining_net_ids.extend([1, 2])
    st.total_routes = 1
    # A persistent working map is present (route_diff always builds one); its
    # contents are recorded through the stubbed cache functions below.
    st.working_obstacles = object()
    return st, a, (p_path, n_path)


def drive(st, a):
    calls = {'route': 0}
    pcb = st.pcb_data

    def precompute(pcb_data, nid, config):
        # The entry records how much of the net's copper was on the board
        # when it was (re)computed.
        return ('entry', nid, sum(1 for s in pcb_data.segments if s.net_id == nid))

    def route(*a_, **k):
        calls['route'] += 1
        if calls['route'] == 1:
            return {'probe_blocked': True, 'blocked_cells': [(5, 5, 0)],
                    'direction': 'forward', 'blocked_at': 'probe', 'iterations': 1}
        return {'failed': True, 'iterations': 1}

    def stop(*a_, **k):
        raise _Stop()

    for nid in (3, 4):
        st.net_obstacles_cache[nid] = precompute(pcb, nid, st.config)

    patches = [
        (diff_pair_loop, 'get_diff_pair_terminals', lambda *a_, **k: ['t0', 't1']),
        (diff_pair_loop, 'leg_electrically_short', lambda *a_, **k: False),
        (diff_pair_loop, 'build_diff_pair_obstacles', lambda *a_, **k: (None, [])),
        (diff_pair_loop, 'get_diff_pair_endpoints', lambda *a_, **k: ([], [], None)),
        (diff_pair_loop, 'route_diff_pair_with_obstacles', route),
        (diff_pair_loop, 'analyze_frontier_blocking',
         lambda *a_, **k: [BlockingInfo(net_id=3, net_name='B_P', blocked_count=5,
                                        track_cells=5, via_cells=0, unique_cells=5,
                                        near_target_cells=0, near_source_cells=0,
                                        details='')]),
        (diff_pair_loop, 'classify_diff_pair_failure', stop),
        (rip_up_reroute, 'precompute_net_obstacles', precompute),
        (rip_up_reroute, 'add_net_obstacles_from_cache', lambda *a_, **k: None),
        (rip_up_reroute, 'remove_net_obstacles_from_cache', lambda *a_, **k: None),
    ]
    saved = [(m, n, getattr(m, n)) for m, n, _ in patches]
    for m, n, v in patches:
        setattr(m, n, v)
    stopped = False
    try:
        diff_pair_loop.route_diff_pairs(st, [('A', a)])
    except _Stop:
        stopped = True
    finally:
        for m, n, v in saved:
            setattr(m, n, v)
    return stopped, calls['route']


def t_probe_restore():
    print("probe rip of a routed pair, retry fails, the pair is restored")
    st, a, (p_path, n_path) = build()
    stopped, routes = drive(st, a)
    if not stopped or routes != 2:
        raise AssertionError(
            f"BROKEN TEST: the loop did not take the probe rip -> retry -> "
            f"restore path (stopped={stopped}, router called {routes}x, want 2)")
    check('the pair is routed again', all(n in st.routed_results for n in (3, 4))
          and set(st.routed_net_ids) >= {3, 4})
    check('its copper is back on the board',
          sum(1 for s in st.pcb_data.segments if s.net_id in (3, 4)) == 2)
    check('routed_net_paths holds its paths again (blocker analysis can name it)',
          st.routed_net_paths.get(3) == p_path and st.routed_net_paths.get(4) == n_path,
          f"keys={sorted(st.routed_net_paths)}")
    ents = {nid: st.net_obstacles_cache.get(nid) for nid in (3, 4)}
    check('the working-map entry was recomputed with the restored copper',
          all(e is not None and e[2] == 1 for e in ents.values()),
          f"entries={ents} (segments counted at recompute; 0 = the rip's "
          f"stubs-only entry)")


if __name__ == '__main__':
    t_probe_restore()
    if failures:
        print(f"FAILED: {failures}")
        sys.exit(1)
    print("PASS")
