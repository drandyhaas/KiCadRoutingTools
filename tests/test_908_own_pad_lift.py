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
  7. EXACTLY ONE lift per map (#977): the base build lifts for nobody, and
     each routing path lifts on the map it routes -- neither zero times (the
     pad seals itself) nor twice (a cell two obstacles blocked goes 2 -> 0
     instead of 2 -> 1, and the restore hands it back at 1, so the map stops
     describing the copper it stands for).

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
    pcb2 = pad = None
    foreign = []
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

        # Counted on a map a net would actually be ROUTED on, not on the
        # bare base: the base map lifts for nobody (#977), so the lift is
        # delivered by the routing path -- here prepare_obstacles_inplace,
        # the one the main loop uses. Counting the base alone measures
        # whether the base lifts, which is a different question and the one
        # #977 answered "never".
        def free_count(nets, prepare_for=None):
            from routing_context import prepare_obstacles_inplace
            from obstacle_map import build_layer_map
            with contextlib.redirect_stdout(io.StringIO()):
                obs = build_base_obstacle_map(pcb2, cfg, nets)
                if prepare_for is not None:
                    prepare_obstacles_inplace(
                        obs, pcb2, cfg, prepare_for, nets, [], {},
                        build_layer_map(cfg.layers), {})
            return sum(0 if obs.is_blocked(x, y, 0) else 1
                       for x, y in corridor)

        with_lift = free_count([pad.net_id], prepare_for=pad.net_id)
        real = _cd.graphic_own_pad_nets
        try:
            _cd.graphic_own_pad_nets = lambda _p: {}
            without_lift = free_count([pad.net_id], prepare_for=pad.net_id)
        finally:
            _cd.graphic_own_pad_nets = real
        check('esp_prog U2.2: the lift frees the west approach corridor',
              with_lift == len(corridor) and without_lift < with_lift,
              f'with={with_lift} without={without_lift} of {len(corridor)}')

        # 5: a FOREIGN net must still be blocked by that same copper -- even
        # when that foreign net is the one being routed and prepared for.
        foreign = [n for n in pcb2.nets
                   if n and n != pad.net_id][:1]
        if foreign:
            check('a foreign net is still blocked by the same tab copper',
                  free_count(foreign, prepare_for=foreign[0]) < len(corridor),
                  f'{free_count(foreign, prepare_for=foreign[0])}/'
                  f'{len(corridor)} free')

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

    # --- 7: exactly ONE lift site per map (#977) --------------------------
    # The own-pad rows are lifted by whatever routes the net, on the map it
    # routes on: prepare_obstacles_inplace (restored afterwards),
    # build_single_ended_obstacles (its own clone), ensure_own_pad_lift
    # (multipoint phase 3) and net_rescue's pristine clone. The BASE build
    # lifts for nobody -- so there is exactly one lifter per map and nothing
    # to arbitrate.
    #
    # It used to lift too, when built for a single net, and record which net
    # in a table keyed by `id(map)`. An address identifies an object only
    # while that object is alive: measured on macOS, twenty maps built and
    # dropped in a row all landed on ONE address, so a BATCH map built after
    # a single-net map read the dead map's entry and skipped the lift. Rows
    # 7d/7e reproduce exactly that.
    #
    # Both directions are gated, and neither is visible to a plain
    # "is this cell free" count. A missing lift leaves own-pad cells blocked;
    # a DOUBLE lift takes a cell two obstacles blocked to 0 where one lift
    # leaves it at 1, so it frees strictly MORE cells than the rows call for.
    # Each arm is therefore compared against a reference map that had the
    # recorded rows removed exactly once, by hand -- an equality, so it needs
    # no threshold and no re-tuning when the stamp geometry changes.
    if pcb2 is not None and pad is not None:
        import gc
        from routing_context import prepare_obstacles_inplace
        from obstacle_map import build_layer_map

        def _base(nets):
            with contextlib.redirect_stdout(io.StringIO()):
                return build_base_obstacle_map(pcb2, cfg, nets)

        def _lift_rows(net):
            return (getattr(pcb2, '_graphic_own_pad_lift', None) or {}).get(net)

        def _lift_cells(net):
            out = []
            for r in (_lift_rows(net) if _lift_rows(net) is not None else []):
                gx, lo, hi, layer = int(r[0]), int(r[1]), int(r[2]), int(r[3])
                out.extend((gx, gy, layer) for gy in range(lo, hi + 1))
            return out

        def _freed(m, cells):
            """The own-pad cells this map leaves UNBLOCKED."""
            return frozenset(c for c in cells if not m.is_blocked(*c))

        def _freed_by_one_lift(nets, cells):
            """Reference: the same base with the rows removed exactly once."""
            m = _base(nets)
            m.remove_blocked_cell_spans_batch(_lift_rows(pad.net_id))
            return _freed(m, cells)

        def _freed_by_prepare(m, nets, cells):
            with contextlib.redirect_stdout(io.StringIO()):
                prepare_obstacles_inplace(m, pcb2, cfg, pad.net_id, nets, [],
                                          {}, build_layer_map(cfg.layers), {})
            return _freed(m, cells)

        solo_nets = [pad.net_id]
        batch_nets = [pad.net_id] + (foreign or [])

        solo = _base(solo_nets)
        cells = _lift_cells(pad.net_id)
        check('7a: this board HAS own-pad rows (else section 7 proves nothing)',
              len(cells) > 0, f'{len(cells)} cell(s)')
        check('7b: a base built for ONE net lifts for nobody',
              len(_freed(solo, cells)) == 0,
              f'{len(_freed(solo, cells))} of {len(cells)} already free')

        want_solo = _freed_by_one_lift(solo_nets, cells)
        check('7c: on a single-net base, prepare lifts exactly once',
              _freed_by_prepare(solo, solo_nets, cells) == want_solo,
              f'{len(_freed_by_prepare(_base(solo_nets), solo_nets, cells))} '
              f'freed, one lift frees {len(want_solo)}')

        want_batch = _freed_by_one_lift(batch_nets, cells)
        check('7d: on a batch base, prepare lifts exactly once',
              _freed_by_prepare(_base(batch_nets), batch_nets, cells)
              == want_batch and len(want_batch) > 0,
              f'{len(_freed_by_prepare(_base(batch_nets), batch_nets, cells))} '
              f'freed, one lift frees {len(want_batch)}')

        # 7e: the #977 regression itself. Build a single-net map, drop it, and
        # build the BATCH map onto its address -- the shape that made
        # test_908 flake. The lift must not depend on what died there.
        collided = None
        for _ in range(64):
            dead = id(_base(solo_nets))
            gc.collect()
            cand = _base(batch_nets)
            if id(cand) == dead:
                collided = cand
                break
            del cand
        if collided is None:
            # Never silently: a row whose SETUP did not happen proves nothing,
            # and this one depends on the platform allocator recycling the
            # address at all.
            print('  NOT RUN: 7e needs a batch map to land on a dead '
                  'single-net map address; 64 attempts did not collide')
        else:
            check('7e: a batch map at a DEAD single-net map\'s address is '
                  'lifted the same',
                  _freed_by_prepare(collided, batch_nets, cells) == want_batch,
                  f'{len(_freed_by_prepare(collided, batch_nets, cells))} freed '
                  f'at the recycled address, {len(want_batch)} on a fresh map')

        # 7f: net_rescue is the one routing path that reaches neither prepare
        # nor build_single_ended_obstacles, so the map it hands out has to
        # arrive already lifted. Both arms: the fresh build and the cache hit
        # (a hit re-clones the SAME pristine map, and the rows travel with it
        # rather than being re-read off a board a later build has overwritten).
        from net_rescue import _pristine_rescue_map
        rescue_a = _pristine_rescue_map(pcb2, pcb2, cfg, pad.net_id, {},
                                        ('t977',))
        rescue_b = _pristine_rescue_map(pcb2, pcb2, cfg, pad.net_id, {},
                                        ('t977',))
        check('7f: the rescue map arrives lifted (fresh build)',
              _freed(rescue_a, cells) == want_solo,
              f'{len(_freed(rescue_a, cells))} freed, want {len(want_solo)}')
        check('7g: ...and on a cache hit',
              _freed(rescue_b, cells) == want_solo,
              f'{len(_freed(rescue_b, cells))} freed, want {len(want_solo)}')

        # 7h: the rows a build records must describe the map that build
        # returns. The per-net via rungs (#530 decision 4) are stamped by
        # SUB-BUILDS on the same pcb_data, at their own via geometry, and
        # every build rebinds these records on the board it is handed -- so
        # the last rung's answer was what prepare then lifted off the parent's
        # map. Same defect class as #977 in a different place: a record read
        # against a map it does not describe. Needs a board with a per-net via
        # size, or no sub-build runs at all (the #568 small rung is skipped).
        from dataclasses import replace as _dc_replace
        from obstacle_cache import via_rungs as _via_rungs
        rung_cfg = _dc_replace(
            cfg, net_via_sizes={(foreign or [pad.net_id])[0]:
                                (cfg.via_size * 2.0, cfg.via_drill * 2.0)})
        check('7h-setup: the probe config really has a rung to sub-build',
              len(_via_rungs(rung_cfg, pcb2)) > 1,
              f'{_via_rungs(rung_cfg, pcb2)}')
        with contextlib.redirect_stdout(io.StringIO()):
            build_base_obstacle_map(pcb2, rung_cfg, solo_nets)
        with_rungs = (pcb2._graphic_own_pad_via_lift or {}).get(pad.net_id)
        with contextlib.redirect_stdout(io.StringIO()):
            build_base_obstacle_map(pcb2, rung_cfg, solo_nets, _rung_pass=True)
        parent_only = (pcb2._graphic_own_pad_via_lift or {}).get(pad.net_id)
        check('7h: a via-rung sub-build does not overwrite the parent build\'s '
              'recorded rows',
              with_rungs is not None and parent_only is not None
              and len(with_rungs) == len(parent_only)
              and (with_rungs == parent_only).all(),
              f'{0 if with_rungs is None else len(with_rungs)} row(s) recorded '
              f'with the rung pass, {0 if parent_only is None else len(parent_only)} '
              f'without it')

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
