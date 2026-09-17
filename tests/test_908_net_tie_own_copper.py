"""A net tie's own copper must never obstruct the nets it ties (#908).

A footprint declaring `(net_tie_pad_groups ...)` shorts those pads THROUGH its
own copper -- the bridge IS the intended conductor between them. #908 made
footprint copper block routing, correctly, but that copper carries no net, so
it is foreign to every net including the two it exists to join. Left unlifted,
a tie seals its own pads: measured on cparti_fpga, `check_reachability` called
the tie pad CAGED for any track width and all 8 pads of its four ties shipped
unconnected.

THE INVARIANT THIS PINS, and the reason it is written as a comparison rather
than as thresholds: **routing a tied net must see exactly the map it would see
if the tie's copper were not copper at all.** The control board is the same
board with the bridge moved to silkscreen, so any divergence is the tie's own
copper obstructing its own net -- which is precisely the bug. A threshold on
"how many cells are free" would have to be re-tuned by whoever next changes the
stamp geometry; an equality against the control never does.

It is a comparison test for a second reason: the four defects behind this each
left the lift LOOKING correct. The rows were computed, recorded and removed,
and its own bookkeeping was self-consistent every time --

  1. the per-pad rule lifted each edge only for the pad it touched, so a
     two-pad tie's end caps lifted for one net each;
  2. the via half of the stamp was recorded nowhere and lifted for no one;
  3. `_StaticStampProxy` (#422) redirects add_blocked_* to the static bitmap
     but not remove_blocked_*, so the lift removed from a layer that never
     held the rows -- a silent no-op;
  4. the `baked` marker lived on pcb_data, so a nested single-net build left
     its marker behind and prepare skipped the lift on the BATCH map.

Every one of those passes a test that asks "did we call the lift". Only a test
that asks "is the map the same as if the copper were absent" catches them.
"""
import contextlib
import io
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []

#: The bridge is a FILLED fp_poly with stroke width 0, which is how KiCad
#: writes a net tie -- and how cparti_fpga's NT1-NT4 are written.
_TIE_POLY = ('(fp_poly (pts (xy -0.4 -0.1) (xy 0.4 -0.1) (xy 0.4 0.1) '
             '(xy -0.4 0.1)) (stroke (width 0) (type solid)) (fill yes) '
             '(layer "{layer}") (uuid "aaaaaaaa-0000-4000-8000-000000000001"))')

BOARD = """(kicad_pcb (version 20241229) (generator "pcbnew")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (2 "B.Cu" signal)
    (25 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "TIED_A")
  (net 2 "TIED_B")
  (net 3 "FOREIGN")
  (footprint "test:net_tie" (layer "F.Cu")
    (at 10 10)
    (property "Reference" "NT1" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (net_tie_pad_groups "1, 2")
    %s
    (pad "1" smd rect (at -0.4 0) (size 0.3 0.2) (layers "F.Cu") (net 1 "TIED_A"))
    (pad "2" smd rect (at 0.4 0) (size 0.3 0.2) (layers "F.Cu") (net 2 "TIED_B"))
  )
  (footprint "test:res" (layer "F.Cu")
    (at 16 10)
    (property "Reference" "R2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "TIED_A"))
    (pad "2" smd rect (at 0 2) (size 0.5 0.5) (layers "F.Cu") (net 3 "FOREIGN"))
  )
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)))
)
"""


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def _board(tie_layer):
    """Parse the fixture with the tie bridge on `tie_layer`.

    "F.Cu" is the real board; "F.SilkS" is the CONTROL -- the identical board
    with the bridge demoted off copper, i.e. what the map would be if #908 had
    never modelled it.
    """
    from kicad_parser import parse_kicad_pcb
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD % _TIE_POLY.format(layer=tie_layer))
        path = f.name
    try:
        return parse_kicad_pcb(path)
    finally:
        os.unlink(path)


def _net(pcb, name):
    for n in pcb.nets.values():
        if n.name == name:
            return n.net_id
    raise AssertionError(f'no net {name}')


def _blocked_window(pcb, net_id, foreign_ids, cx=10.0, cy=10.0, r=14):
    """is_blocked over a window around the tie, after a real prepare."""
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map, build_layer_map
    from routing_context import prepare_obstacles_inplace
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    with contextlib.redirect_stdout(io.StringIO()):
        obs = build_base_obstacle_map(pcb, cfg, [net_id] + list(foreign_ids))
        prepare_obstacles_inplace(obs, pcb, cfg, net_id,
                                  [net_id] + list(foreign_ids), [], {},
                                  build_layer_map(cfg.layers), {})
    gx, gy = GridCoord(cfg.grid_step).to_grid(cx, cy)
    return {(dx, dy): bool(obs.is_blocked(gx + dx, gy + dy, 0))
            for dx in range(-r, r + 1) for dy in range(-r, r + 1)}


def t_the_fixture_really_has_modelled_tie_copper():
    """Non-vacuity: every check below is empty if #908 does not see this."""
    pcb = _board('F.Cu')
    fp = pcb.footprints.get('NT1')
    graphic = [s for s in pcb.segments
               if getattr(s, 'graphic', False)
               and getattr(s, 'owner_ref', '') == 'NT1']
    check('t_the_fixture_really_has_modelled_tie_copper',
          fp is not None and fp.net_tie_groups == [['1', '2']] and len(graphic) >= 3,
          f'net_tie_groups={fp and fp.net_tie_groups}, {len(graphic)} graphic segment(s)')
    ctl = _board('F.SilkS')
    ctl_graphic = [s for s in ctl.segments
                   if getattr(s, 'graphic', False)
                   and getattr(s, 'owner_ref', '') == 'NT1']
    check('t_the_control_has_none_of_it', not ctl_graphic,
          f'{len(ctl_graphic)} graphic segment(s) on the control')


def t_the_tie_copper_lifts_for_both_tied_nets():
    from check_drc import graphic_own_pad_nets
    pcb = _board('F.Cu')
    a, b, foreign = _net(pcb, 'TIED_A'), _net(pcb, 'TIED_B'), _net(pcb, 'FOREIGN')
    own = graphic_own_pad_nets(pcb)
    segs = [s for s in pcb.segments
            if getattr(s, 'graphic', False) and getattr(s, 'owner_ref', '') == 'NT1']
    both = [s for s in segs if {a, b} <= set(own.get(id(s), ()))]
    check('t_the_tie_copper_lifts_for_both_tied_nets',
          len(both) == len(segs),
          f'{len(both)} of {len(segs)} tie edge(s) lift for BOTH tied nets')
    # The other direction, or "lift for everyone" would pass the row above.
    leaked = [s for s in segs if foreign in own.get(id(s), ())]
    check('t_the_tie_copper_still_blocks_a_foreign_net',
          not leaked,
          f'{len(leaked)} tie edge(s) wrongly lifted for FOREIGN')


def t_a_tied_net_sees_the_map_it_would_see_without_the_tie_copper():
    """THE regression guard: equality against the copper-free control."""
    real, ctl = _board('F.Cu'), _board('F.SilkS')
    a_r, a_c = _net(real, 'TIED_A'), _net(ctl, 'TIED_A')
    f_r, f_c = _net(real, 'FOREIGN'), _net(ctl, 'FOREIGN')
    w_real = _blocked_window(real, a_r, [f_r])
    w_ctl = _blocked_window(ctl, a_c, [f_c])
    diff = [k for k in w_real if w_real[k] != w_ctl[k]]
    check('t_a_tied_net_sees_the_map_it_would_see_without_the_tie_copper',
          not diff,
          f'{len(diff)} cell(s) differ from the copper-free control'
          + (f' e.g. {diff[:4]}' if diff else ''))
    # Non-vacuity for the comparison itself: the window must contain blocked
    # cells at all, or "identical" is two empty maps agreeing.
    check('t_the_window_is_not_trivially_empty',
          any(w_ctl.values()),
          f'{sum(w_ctl.values())} of {len(w_ctl)} cell(s) blocked on the control')


def t_a_foreign_net_is_still_blocked_by_the_tie_copper():
    """The half that must NOT converge: the tie is real copper to everyone else."""
    real, ctl = _board('F.Cu'), _board('F.SilkS')
    f_r, f_c = _net(real, 'FOREIGN'), _net(ctl, 'FOREIGN')
    a_r, a_c = _net(real, 'TIED_A'), _net(ctl, 'TIED_A')
    w_real = _blocked_window(real, f_r, [a_r])
    w_ctl = _blocked_window(ctl, f_c, [a_c])
    extra = [k for k in w_real if w_real[k] and not w_ctl[k]]
    check('t_a_foreign_net_is_still_blocked_by_the_tie_copper',
          len(extra) > 0,
          f'{len(extra)} cell(s) blocked for FOREIGN that the control leaves free')


def t_the_short_gate_does_not_see_the_tie_as_foreign():
    """The geometric terminal-short gate, which reads copper, not the map.

    The obstacle map can be perfectly lifted and the net still fail: the rescue
    pass finds a route to the tie pad, then `_neck_terminal_grazes` measures the
    terminal against "foreign" copper, finds the tie's own net-0 bridge under it
    and rejects the route as a shipped short. Measured on cparti_fpga, that was
    the LAST thing standing between the fixed map and a routed net -- repeated
    "terminal copper on F.Cu would OVERLAP a foreign track/via (edge dist
    -0.147mm)" while the copper-free control rescued the same gap and
    reconnected.
    """
    from single_ended_routing import _seg_foreign_seg_dist
    real = _board('F.Cu')
    a, foreign = _net(real, 'TIED_A'), _net(real, 'FOREIGN')
    # A terminal segment lying straight along the tie bridge.
    x1, y1, x2, y2 = 9.6, 10.0, 10.4, 10.0
    d_tied = _seg_foreign_seg_dist(real, a, x1, y1, x2, y2, 'F.Cu')
    d_foreign = _seg_foreign_seg_dist(real, foreign, x1, y1, x2, y2, 'F.Cu')
    check('t_the_short_gate_does_not_see_the_tie_as_foreign',
          d_tied > 0.0,
          f'a tied net measures {d_tied:.4f}mm to "foreign" copper (must be clear)')
    # The control direction: the same copper IS foreign to an unrelated net, or
    # the exemption would be "never a short", which ships real ones.
    check('t_the_short_gate_still_sees_it_for_a_foreign_net',
          d_foreign <= 0.0,
          f'a foreign net measures {d_foreign:.4f}mm (must overlap)')


def main():
    t_the_fixture_really_has_modelled_tie_copper()
    t_the_tie_copper_lifts_for_both_tied_nets()
    t_a_tied_net_sees_the_map_it_would_see_without_the_tie_copper()
    t_a_foreign_net_is_still_blocked_by_the_tie_copper()
    t_the_short_gate_does_not_see_the_tie_as_foreign()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
