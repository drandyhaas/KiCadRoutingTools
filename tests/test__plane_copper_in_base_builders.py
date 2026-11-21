"""#424 tap relocation: armed plane nets must stay obstacles in the from-base builders.

With KICAD_TAP_RELOCATION=1, route.py takes zone-backed plane nets out of the
base map (so single-tap surgery is an exact cache recompute) and deliberately
keeps them out of routed_net_ids.  The incremental working map carries them via
the net cache, but `build_single_ended_obstacles` / `build_diff_pair_obstacles`
start from a clone of the base and only add routed nets, remaining nets and the
GND net's vias.  A net routed through either builder (casualty reconcile after a
failed rip restore, phase-3 retries, reroute loops) then sees no plane copper
and lays tracks straight through the plane net's vias and stubs, which the
cleanup drops as shorts.

Fixture: net C is the plane net, with one via and one stub, excluded from the
base map exactly as route.py does when relocation is armed.  Both builders,
building for net B, must block C's via and stub cells.
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))

FAILS = []
VIA = (20.0, 20.0)                 # net C via
STUB = ((10.0, 26.0), (16.0, 26.0))  # net C stub on F.Cu


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def main():
    try:
        import grid_router  # noqa: F401
    except ImportError:
        print('SKIP: grid_router not importable')
        return 0
    import constraint_agreement as ca
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map, build_layer_map
    from routing_context import build_single_ended_obstacles, build_diff_pair_obstacles

    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'plane.kicad_pcb')
        fp = lambda ref, x, y, n, name: dict(ref=ref, x=x, y=y, net_id=n, net_name=name)
        ca.write_board(
            board,
            footprints=[fp('U1', 5, 5, 1, 'A'), fp('U2', 35, 5, 1, 'A'),
                        fp('U3', 5, 35, 2, 'B'), fp('U4', 35, 35, 2, 'B'),
                        fp('U5', 8, 26, 3, 'C')],
            vias=[(VIA[0], VIA[1], 0.6, 0.3, 3)],
            segments=[(STUB[0][0], STUB[0][1], STUB[1][0], STUB[1][1], 0.25, 'F.Cu', 3)])
        pcb = parse_kicad_pcb(board)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.15, clearance=0.15,
                          grid_step=0.05, via_size=0.6, via_drill=0.3)
    coord = GridCoord(cfg.grid_step)
    layer_map = build_layer_map(cfg.layers)
    with contextlib.redirect_stdout(io.StringIO()):
        # route.py with relocation armed: plane net C is base-map-excluded
        base = build_base_obstacle_map(pcb, cfg, [1, 2, 3], static_base=True)
    pcb._tap_relocation_plane_ids = (3,)

    vx, vy = coord.to_grid(*VIA)
    sx, sy = coord.to_grid((STUB[0][0] + STUB[1][0]) / 2, STUB[0][1])
    check('fixture: the base map leaves net C open (as route.py builds it)',
          not base.is_blocked(vx, vy, 0) and not base.is_blocked(sx, sy, 0))

    with contextlib.redirect_stdout(io.StringIO()):
        se, _ = build_single_ended_obstacles(
            base, pcb, cfg, routed_net_ids=[1], remaining_net_ids=[2],
            all_unrouted_net_ids=[2], net_id=2, gnd_net_id=None,
            track_proximity_cache={}, layer_map=layer_map)
    check('single-ended: plane via blocks both layers',
          se.is_blocked(vx, vy, 0) and se.is_blocked(vx, vy, 1))
    check('single-ended: plane via blocks via placement', se.is_via_blocked(vx, vy))
    check('single-ended: plane stub blocks its layer', se.is_blocked(sx, sy, 0))

    with contextlib.redirect_stdout(io.StringIO()):
        dp = build_diff_pair_obstacles(
            base, pcb, cfg, routed_net_ids=[1], remaining_net_ids=[2],
            all_unrouted_net_ids=[2], p_net_id=2, n_net_id=2, gnd_net_id=None,
            track_proximity_cache={}, layer_map=layer_map, extra_clearance=0.0)
    dp = dp[0] if isinstance(dp, tuple) else dp
    check('diff pair: plane via blocks via placement', dp.is_via_blocked(vx, vy))
    check('diff pair: plane stub blocks its layer', dp.is_blocked(sx, sy, 0))

    # Disarmed (no plane ids recorded): a base-excluded net stays the caller's business
    pcb._tap_relocation_plane_ids = ()
    with contextlib.redirect_stdout(io.StringIO()):
        off, _ = build_single_ended_obstacles(
            base, pcb, cfg, routed_net_ids=[1], remaining_net_ids=[2],
            all_unrouted_net_ids=[2], net_id=2, gnd_net_id=None,
            track_proximity_cache={}, layer_map=layer_map)
    check('disarmed: builder output unchanged (net C not stamped)',
          not off.is_blocked(sx, sy, 0))

    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    sys.exit(main())
