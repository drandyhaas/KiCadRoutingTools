#!/usr/bin/env python3
"""A pair routed by the hybrid escape is a blocker the next pair can name.

The blocking analysis names blockers from `routed_net_paths`, and every pair
success site recorded a member there only from its grid path (`p_path` /
`n_path`). A hybrid escape (#766) carries none, so its members were never
recorded: a later pair whose launch the hybrid's legs sealed was told "no
rippable blockers found" and went single-ended. Measured with the #1736 pad
stamps on a connector row: the sealed pair coupled once it could rip the
hybrid.

Rows:
  - `record_diff_pair_success` (the reconcile and reroute commit path)
    records both members of a hybrid result, with an empty path, and keeps a
    coupled result's grid paths as they are;
  - the main loop's inline hybrid commit records its members too.

    python3 tests/test_1736_hybrid_is_a_blocker.py
"""
import inspect
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

from kicad_parser import PCBData, BoardInfo, Net, Segment            # noqa: E402
from routing_config import GridRouteConfig                           # noqa: E402
from routing_context import record_diff_pair_success                 # noqa: E402
import diff_pair_loop                                                # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _commit(result, p, n):
    cfg = GridRouteConfig(track_width=0.2, clearance=0.2, grid_step=0.1,
                          layers=['F.Cu', 'B.Cu'])
    pcb = PCBData(board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                                       copper_layers=['F.Cu', 'B.Cu'],
                                       board_bounds=(0.0, 0.0, 20.0, 20.0)),
                  nets={p: Net(net_id=p, name='D_P'), n: Net(net_id=n, name='D_N')},
                  footprints={}, vias=[], segments=[], pads_by_net={}, zones=[])
    pair = SimpleNamespace(p_net_id=p, n_net_id=n, p_net_name='D_P', n_net_name='D_N')
    paths = {}
    record_diff_pair_success(pcb, result, pair, 'D', cfg, [p, n], [], paths, {}, {},
                             {}, {'F.Cu': 0, 'B.Cu': 1})
    return paths


def main():
    segs = lambda: [Segment(start_x=2.0, start_y=y, end_x=12.0, end_y=y, width=0.2,
                            layer='F.Cu', net_id=nid)
                    for nid, y in ((1, 5.0), (2, 5.4))]
    hybrid = {'new_segments': segs(), 'new_vias': [], 'hybrid_escape': True}
    paths = _commit(hybrid, 1, 2)
    check("a hybrid result's members are recorded for the blocking analysis",
          set(paths) == {1, 2}, f"recorded {sorted(paths)}")

    coupled = {'new_segments': segs(), 'new_vias': [],
               'p_path': [(20, 50, 0), (120, 50, 0)], 'n_path': [(20, 54, 0), (120, 54, 0)]}
    paths = _commit(coupled, 1, 2)
    check("a coupled result keeps its grid paths",
          paths.get(1) == coupled['p_path'] and paths.get(2) == coupled['n_path'])

    src = inspect.getsource(diff_pair_loop.route_diff_pairs)
    i = src.find('HYBRID ESCAPE: direct coupled middle')
    tail = src[i:i + 2500] if i >= 0 else ''
    check("the main loop's hybrid commit records its members",
          'record_pair_paths(routed_net_paths, pair, hyb)' in tail)

    if FAILS:
        print(f"\nFAIL: {len(FAILS)} row(s): {FAILS}")
        return 1
    print("\na hybrid-routed pair is recorded as a routed pair a blocker search can name. OK")
    return 0


if __name__ == '__main__':
    sys.exit(main())
