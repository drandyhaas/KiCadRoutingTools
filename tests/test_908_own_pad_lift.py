#!/usr/bin/env python3
"""#908: modelling a footprint's own copper must not SEAL its own pad.

A footprint's copper carries no net, so it is foreign copper to every net --
including the net of the pad it was drawn around. esp_prog's U2 tab notches
around pad 2 (`Net-(C1-Pad1)`) with its west edge exactly coincident with the
pad's, so stamping the shape whole pinches the pad's approach: the failure
mode of sibling #907, manufactured by the fix for #908.

The lift is per SEGMENT and own-footprint only. The whole-CLUSTER answer
(`graphic_effective_nets`, which is what the CHECKER uses) is much wider:
watchy's twelve antenna polys form ONE cluster touching both the feed pad and
a GND pad, so a cluster-wide lift would let a GND route cross the whole
antenna -- graded clean by that same reasoning, and a destroyed part.

Invariants gated here:
  1. The subset chain own-pad <= pads-only <= checker, on synthetic data and
     on every affected corpus board. This is the safety direction: a
     GENERATOR that is never more permissive than the CHECKER cannot lay
     copper the checker will flag.
  2. `include_mutable=False` really does drop the track/via arms (a track
     touching a graphic grants its net in the checker's answer and NOT in the
     obstacle map's -- because a rip can delete that track later).
  3. The lift is LOCAL: on watchy only a handful of the antenna's 48 edges
     lift, not all of them.
  4. It works: esp_prog U2 pad 2's approach corridor is free with the lift
     and pinched without it.
  5. A FOREIGN net is still blocked by the same copper.
  6. Reporting: a graphic is named `Polygon(<owner>)`, matching KiCad's own
     "Polygon [<no net>] of U2 on F.Cu"; a routed track gets no label.

Run:
    python3 tests/test_908_own_pad_lift.py
"""

import contextlib
import io
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb, Segment, Pad, Net, PCBData, Footprint
from check_drc import (graphic_effective_nets, graphic_own_pad_nets,
                       graphic_item_label)

RUN_ALL_TIMEOUT = 900


def _synthetic():
    """A 2-pad part whose own graphic touches pad 1, plus a foreign track that
    also touches the graphic (the track arm is what `include_mutable` gates)."""
    pad1 = Pad(pad_number='1', net_id=1, net_name='/A',
               global_x=0.0, global_y=0.0, local_x=0, local_y=0,
               size_x=1.0, size_y=1.0, shape='rect', layers=['F.Cu'],
               drill=0, pad_type='smd', component_ref='U1')
    pad2 = Pad(pad_number='2', net_id=2, net_name='/B',
               global_x=8.0, global_y=0.0, local_x=8, local_y=0,
               size_x=1.0, size_y=1.0, shape='rect', layers=['F.Cu'],
               drill=0, pad_type='smd', component_ref='U1')
    art = Segment(start_x=0.5, start_y=0.0, end_x=3.0, end_y=0.0,
                  width=0.3, layer='F.Cu', net_id=0, graphic=True,
                  owner_ref='U1')
    trk = Segment(start_x=3.0, start_y=0.0, end_x=5.0, end_y=0.0,
                  width=0.3, layer='F.Cu', net_id=3)
    fp = Footprint(reference='U1', footprint_name='L:P', x=0.0, y=0.0,
                   rotation=0.0, layer='F.Cu', pads=[pad1, pad2])
    return PCBData(board_info=None,
                   nets={1: Net(1, '/A'), 2: Net(2, '/B'), 3: Net(3, '/C')},
                   footprints={'U1': fp}, vias=[], segments=[art, trk],
                   pads_by_net={1: [pad1], 2: [pad2]}), art


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    # --- 1/2 on synthetic data --------------------------------------------
    pcb, art = _synthetic()
    own = graphic_own_pad_nets(pcb)
    pads_only = graphic_effective_nets(pcb, include_mutable=False)
    checker = graphic_effective_nets(pcb, include_mutable=True)
    check('own-pad lift names the touched pad\'s net',
          own.get(id(art)) == frozenset({1}), f'{own.get(id(art))}')
    check('pads-only answer has the pad net and NOT the track net',
          pads_only.get(id(art)) == frozenset({0, 1}),
          f'{pads_only.get(id(art))}')
    check('the checker\'s answer DOES include the touching track\'s net',
          3 in (checker.get(id(art)) or set()), f'{checker.get(id(art))}')
    check('subset chain holds: own <= pads-only <= checker',
          own[id(art)] <= pads_only[id(art)] <= checker[id(art)])

    # --- 1 again, on the real boards --------------------------------------
    boards = ['esp_prog', 'tigard', 'watchy', 'ulx3s']
    seen = 0
    for b in boards:
        path = os.path.join(ROOT_DIR, 'kicad_files', f'{b}.kicad_pcb')
        if not os.path.isfile(path):
            check(f'{b}: board present', False, path)
            continue
        with contextlib.redirect_stdout(io.StringIO()):
            rp = parse_kicad_pcb(path)
        o = graphic_own_pad_nets(rp)
        po = graphic_effective_nets(rp, include_mutable=False)
        ck = graphic_effective_nets(rp, include_mutable=True)
        seen += 1
        check(f'{b}: subset chain holds for every lifted segment',
              bool(o) and all(o[k] <= po[k] <= ck[k] for k in o),
              f'{len(o)} lifted')
        # 3: the lift is LOCAL, not the whole shape
        n_graphic = len([s for s in rp.segments
                         if getattr(s, 'graphic', False)])
        if b == 'watchy':
            check('watchy: only a few antenna edges lift, not all 48',
                  0 < len(o) < n_graphic / 2, f'{len(o)}/{n_graphic}')
    check('the corpus section actually ran', seen == len(boards),
          f'{seen}/{len(boards)}')

    # --- 4/5: the obstacle map ---------------------------------------------
    from routing_config import GridRouteConfig
    from obstacle_map import build_base_obstacle_map, GridCoord
    import check_drc as _cd

    path = os.path.join(ROOT_DIR, 'kicad_files', 'esp_prog.kicad_pcb')
    if not os.path.isfile(path):
        check('esp_prog present for the obstacle probe', False, path)
    else:
        with contextlib.redirect_stdout(io.StringIO()):
            pcb2 = parse_kicad_pcb(path)
        pad = [p for p in pcb2.footprints['U2'].pads if p.pad_number == '2'][0]
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
        coord = GridCoord(cfg.grid_step)
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
        corridor = [(gx - k, gy) for k in range(1, 9)]

        def free_count(nets):
            with contextlib.redirect_stdout(io.StringIO()):
                obs = build_base_obstacle_map(pcb2, cfg, nets)
            return sum(0 if obs.is_blocked(x, y, 0) else 1
                       for x, y in corridor)

        with_lift = free_count([pad.net_id])
        real = _cd.graphic_own_pad_nets
        try:
            _cd.graphic_own_pad_nets = lambda _p: {}
            without_lift = free_count([pad.net_id])
        finally:
            _cd.graphic_own_pad_nets = real
        check('esp_prog U2.2: the lift frees the west approach corridor',
              with_lift == len(corridor) and without_lift < with_lift,
              f'with={with_lift} without={without_lift} of {len(corridor)}')

        # 5: a FOREIGN net must still be blocked by that same copper
        foreign = [n for n in pcb2.nets
                   if n and n != pad.net_id][:1]
        if foreign:
            check('a foreign net is still blocked by the same tab copper',
                  free_count(foreign) < len(corridor),
                  f'{free_count(foreign)}/{len(corridor)} free')

            # ...INCLUDING when both nets are in the same batch. The base map
            # is built for a whole call, so a lift keyed on "is the own net
            # anywhere in nets_to_route" drops the copper for EVERY net in the
            # run: measured on route.py's default all-nets call, tigard 4/4
            # and ulx3s 24/24 footprint-copper edges went unmodelled. Routing
            # the two nets SEPARATELY cannot see that, which is why this row
            # exists.
            # Compared against the SAME batch with the lift disabled, not
            # against a foreign-only run: routing the own net also stops its
            # PADS being stamped, which moves the corridor for reasons that
            # have nothing to do with this feature.
            batch = [pad.net_id] + foreign
            both = free_count(batch)
            try:
                _cd.graphic_own_pad_nets = lambda _p: {}
                both_nolift = free_count(batch)
            finally:
                _cd.graphic_own_pad_nets = real
            check('a batch containing the own net does NOT lift it in the '
                  'base map',
                  both == both_nolift,
                  f'batch={both} same-batch-without-the-lift={both_nolift}')

        # ...and the per-net lift still reaches the own net inside a batch,
        # through prepare_obstacles_inplace rather than the base map. This is
        # the arm that proves the batch fix actually delivers the lift; a
        # skipped version of it would assert nothing.
        from routing_context import (prepare_obstacles_inplace,
                                     restore_obstacles_inplace)
        from obstacle_map import build_layer_map
        import numpy as _np
        batch = [pad.net_id] + (foreign or [])
        with contextlib.redirect_stdout(io.StringIO()):
            obs = build_base_obstacle_map(pcb2, cfg, batch)
        blocked_before = sum(1 for x, y in corridor if obs.is_blocked(x, y, 0))
        cache = {}
        with contextlib.redirect_stdout(io.StringIO()):
            _snv, _ = prepare_obstacles_inplace(
                obs, pcb2, cfg, pad.net_id, batch, [], {},
                build_layer_map(cfg.layers), cache)
        prepared = sum(0 if obs.is_blocked(x, y, 0) else 1
                       for x, y in corridor)
        check('inside a batch, prepare lifts it for the OWN net',
              prepared == len(corridor), f'{prepared}/{len(corridor)}')
        with contextlib.redirect_stdout(io.StringIO()):
            restore_obstacles_inplace(
                obs, pad.net_id, cache,
                _snv if isinstance(_snv, _np.ndarray)
                else _np.empty((0, 2), dtype=_np.int32))
        check('and restore puts every lifted row back',
              sum(1 for x, y in corridor
                  if obs.is_blocked(x, y, 0)) == blocked_before,
              f'after={sum(1 for x, y in corridor if obs.is_blocked(x, y, 0))} '
              f'before={blocked_before}')

    # --- 6: naming ---------------------------------------------------------
    check('a graphic with an owner is named Polygon(owner)',
          graphic_item_label(art) == 'Polygon(U1)')
    check('an ownerless graphic is named Graphic',
          graphic_item_label(Segment(0, 0, 1, 1, 0.2, 'F.Cu', 0,
                                     graphic=True)) == 'Graphic')
    check('a routed track gets no label',
          graphic_item_label(Segment(0, 0, 1, 1, 0.2, 'F.Cu', 5)) == '')

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
