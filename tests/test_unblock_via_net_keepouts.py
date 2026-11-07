#!/usr/bin/env python3
"""Pad taps (tap_pad_with_escalation) must honour the net-filtered keep-outs
(net_keepouts) that bind the net they place for.

A tap places on maps of its own, which the per-route region stamp
(net_keepouts.stamp) never reaches, and its callers lay the copper outside any
stamped route: the #189 via-in-pad unblock (and its #535 off-pad escape)
registers its site as a free via in an allowed block, the #666 bare-ball
escape of the net rescue appends it to the board. Measured beside a mask
opening only GND may cross: a boxed power pad got an unblock via whose ring
reached 0.13 mm into the region, and a bottom-side power pad under the region
got an in-pad dogbone whose barrel reached its top-side copper. The end-of-run
audit reported both; KiCad graded them items_not_allowed / solder_mask_bridge.

    python3 tests/test_unblock_via_net_keepouts.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import net_keepouts as nk  # noqa: E402
from kicad_parser import BoardInfo, Net, Pad, PCBData  # noqa: E402
from routing_config import GridCoord, GridRouteConfig  # noqa: E402
from plane_pad_tap import tap_pad_with_escalation  # noqa: E402
from single_ended_routing import _place_shrunk_via_in_pad  # noqa: E402

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def _pcb(region_nets, pad_layer='F.Cu'):
    """A lone SMD pad of net PWR on `pad_layer` at (10, 10) under a 1 x 1 mm
    F.Cu region that keeps `region_nets` out (tracks and vias)."""
    pad = Pad(pad_number='1', net_id=5, net_name='PWR', global_x=10.0,
              global_y=10.0, size_x=0.6, size_y=0.6, shape='rect',
              layers=[pad_layer], drill=0, component_ref='C1',
              local_x=0.0, local_y=0.0)
    info = BoardInfo(layers={}, copper_layers=list(LAYERS),
                     board_bounds=(0.0, 0.0, 20.0, 20.0))
    pcb = PCBData(board_info=info, nets={5: Net(net_id=5, name='PWR')},
                  footprints={}, vias=[], segments=[], pads_by_net={5: [pad]})
    nk.attach(pcb, [{'name': 'window', 'polygon': [(9.5, 9.5), (10.5, 9.5), (10.5, 10.5), (9.5, 10.5)],
                     'holes': [], 'layers': {'F.Cu'}, 'tracks_allowed': False,
                     'vias_allowed': False, 'nets': frozenset(region_nets)}])
    return pcb, pad


def _seg(sd):
    """A tap's segment dict as a Segment of net PWR."""
    from kicad_parser import Segment
    return Segment(start_x=sd['start'][0], start_y=sd['start'][1],
                   end_x=sd['end'][0], end_y=sd['end'][1],
                   width=sd['width'], layer=sd['layer'], net_id=5)


def _config():
    return GridRouteConfig(grid_step=0.05, clearance=0.1, track_width=0.12,
                           via_size=0.45, via_drill=0.2, layers=list(LAYERS))


def run():
    fails = []

    def check(name, cond):
        print(('  PASS  ' if cond else '  FAIL  ') + name)
        if not cond:
            fails.append(name)

    cfg = _config()
    coord = GridCoord(cfg.grid_step)

    # Control: a region that does not bind PWR leaves the unblock as it was.
    pcb, pad = _pcb({'GND'})
    r = _place_shrunk_via_in_pad(pad, None, cfg, pcb, 5, coord, LAYERS)
    check("exempt net: the unblock still places its via", r is not None)

    # The region binds PWR: whatever the unblock returns must be clean.
    pcb, pad = _pcb({'PWR'})
    r = _place_shrunk_via_in_pad(pad, None, cfg, pcb, 5, coord, LAYERS)
    found = [] if r is None else nk.audit(pcb, r[3], [r[0]])
    if r is not None:
        print(f"    (via at {r[0].x:.3f},{r[0].y:.3f} size {r[0].size}, "
              f"{len(r[3])} stub segment(s); audit {found})")
    check("bound net: no unblock copper inside the region", found == [])

    # A bottom-side pad under a top-side region: the bare-ball escape's tap
    # (in-pad dogbone, the rescue's arguments) must not drill through it.
    for nets, want in (({'GND'}, True), ({'PWR'}, False)):
        pcb, pad = _pcb(nets, pad_layer='B.Cu')
        t = tap_pad_with_escalation(pad, 'B.Cu', 5, pcb, cfg, max_search_radius=1.5,
                                    via_size=cfg.via_size, via_drill=cfg.via_drill,
                                    verbose=False, fine_for_all=True)
        vias = []
        if t.success and t.via is not None:
            from kicad_parser import Via
            v = t.via
            vias = [Via(x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                        layers=v.get('layers', ['F.Cu', 'B.Cu']), net_id=5)]
        found = nk.audit(pcb, [], vias)
        if want:
            check("bottom pad, exempt net: the tap still places its via",
                  t.success and bool(vias))
        else:
            check("bottom pad, bound net: no tap via inside the region", found == [])
            # The search skips the region's sites and goes on to the next one
            # (net_keepouts.via_site_filter) rather than failing: there is
            # room outside the region well within the 1.5 mm search.
            check("bottom pad, bound net: the tap relocates its via outside the region",
                  t.success and bool(vias) and found == []
                  and nk.audit(pcb, [_seg(sd) for sd in t.segments], []) == [])

    # The site predicate itself: None when nothing binds the net's vias, and
    # it agrees with the audit on both sides of the region's edge.
    pcb, _ = _pcb({'GND'})
    check("via_site_filter: None for a net no region binds",
          nk.via_site_filter(pcb, 5, 0.45) is None)
    pcb, _ = _pcb({'PWR'})
    ok = nk.via_site_filter(pcb, 5, 0.45)
    check("via_site_filter: refuses the region's centre", ok is not None and not ok(10.0, 10.0))
    check("via_site_filter: refuses a ring that grazes the edge", not ok(10.7, 10.0))
    check("via_site_filter: allows a ring clear of the edge", ok(10.75, 10.0))
    check("via_site_filter: allows a far site", ok(15.0, 15.0))

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(run())
