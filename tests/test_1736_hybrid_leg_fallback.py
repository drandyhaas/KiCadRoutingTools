#!/usr/bin/env python3
"""#1736: a hybrid's legs that cannot attach on layer-width pad stamps retry
on call-width ones.

route_diff's maps stamp pads and the board edge at each layer's width
(`stamp_track_width`). On a fine-pitch part whose layer width is wider than
its pads, the one legal leg runs along its own pad's centreline, and the pad
stamp's corner keep-out (half a cell at a rectangular corner) can close it.
The hybrid escape then retries its legs once on a map whose pads and edge are
stamped at the call's track width; the assembled-route gate still grades each
leg at its real width.

Rows:
  * a 0.35 mm-pitch row of 0.18 mm pads, layer width 0.2337 mm: the own pad's
    centreline is closed at the pad's corner rows on the layer-width stamps
    and open on the call-width ones (`stamp_pads_at_call_width`);
  * the fallback base map is built once, on first ask, and the run's lift
    records are put back after it; no builder when no layer is wider;
  * `hybrid_leg_maps` hands back no fallback without a builder, and a
    fallback built from the call-width base with the call-width config;
  * the hybrid retries only on unattached legs, without a second fallback,
    and every hybrid call site passes its fallback.

    python3 tests/test_1736_hybrid_leg_fallback.py
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence  # noqa: E402

TW, GAP, CLR, STEP, WIDE = 0.1, 0.1, 0.1, 0.1, 0.2337

# Three pads 0.35 mm apart, the middle one the routed net's; its centre sits a
# quarter cell off the grid, as a connector's pads usually do.
BOARD = """(kicad_pcb (version 20240108) (generator "t")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "") (net 1 "A") (net 2 "OWN") (net 3 "B")
  (gr_rect (start 0 0) (end 10 10) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts"))
  (footprint "T:J" (layer "F.Cu") (at 5.125 5.215)
    (pad "1" smd rect (at -0.35 0) (size 0.18 0.33) (layers "F.Cu") (net 1 "A"))
    (pad "2" smd rect (at 0 0) (size 0.18 0.33) (layers "F.Cu") (net 2 "OWN"))
    (pad "3" smd rect (at 0.35 0) (size 0.18 0.33) (layers "F.Cu") (net 3 "B")))
)
"""

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _cfg(widths, **kw):
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=TW, diff_pair_gap=GAP,
                           clearance=CLR, grid_step=STEP, board_edge_clearance=0.2,
                           via_size=0.45, via_drill=0.2, layer_widths=widths,
                           reserve_layer_widths=True, **kw)


def geometry_rows(pcb):
    from obstacle_map import build_base_obstacle_map
    try:
        narrow_cfg = _cfg({'F.Cu': WIDE}, stamp_pads_at_call_width=True)
    except TypeError as e:
        check("the config takes stamp_pads_at_call_width", False, str(e))
        return
    with contextlib.redirect_stdout(io.StringIO()):
        layer = build_base_obstacle_map(pcb, _cfg({'F.Cu': WIDE}), [2])
        call = build_base_obstacle_map(pcb, narrow_cfg, [2])

    def g(mm):
        return int(round(mm / STEP))
    # x = 5.1 is the grid column on the own pad's centreline; a track there at
    # the layer width clears both neighbours' copper by 0.118 mm.
    column = [(5.1, y) for y in (4.9, 5.0, 5.1, 5.2, 5.3, 5.4, 5.5)]
    corner_rows = [(x, y) for x, y in column if layer.is_blocked(g(x), g(y), 0)]
    check("on the layer-width stamps the own pad's centreline is closed at a corner row",
          bool(corner_rows), f"{corner_rows}")
    check("on the call-width stamps the whole centreline is open",
          not any(call.is_blocked(g(x), g(y), 0) for x, y in column))


def builder_rows(pcb):
    import obstacle_map
    import routing_context as rc
    if not hasattr(rc, 'call_width_leg_base'):
        check("routing_context has call_width_leg_base", False)
        return
    check("no fallback builder when no layer is wider than the call's width",
          rc.call_width_leg_base(pcb, _cfg({}), [2], {}) is None)

    built = []
    real = obstacle_map.build_base_obstacle_map

    def counting(pcb_data, config, nets, **kw):
        built.append(config.stamp_pads_at_call_width)
        pcb_data._net_tie_lift = 'the fallback build'
        return real(pcb_data, config, nets, **kw)
    obstacle_map.build_base_obstacle_map = counting
    try:
        pcb._net_tie_lift = 'the run'
        build = rc.call_width_leg_base(pcb, _cfg({'F.Cu': WIDE}), [2], {})
        with contextlib.redirect_stdout(io.StringIO()):
            first = build() if build else None
            second = build() if build else None
    finally:
        obstacle_map.build_base_obstacle_map = real
    check("the fallback base map is built once, on the first ask, at call width",
          built == [True] and first is not None and first is second, f"{built}")
    check("the run's lift records are put back after the fallback build",
          pcb._net_tie_lift == 'the run', f"{pcb._net_tie_lift}")

    seen = []
    real_legs = rc.build_diff_pair_leg_obstacles

    def legs(base, pcb_data, config, *a):
        seen.append((base, config.stamp_pads_at_call_width))
        return ('legs', len(seen))
    rc.build_diff_pair_leg_obstacles = legs
    try:
        args = ([], [], [], 1, 3, None, {}, {'F.Cu': 0, 'B.Cu': 1})
        pcb._call_width_leg_base = None
        m, fb = rc.hybrid_leg_maps('run base', pcb, _cfg({'F.Cu': WIDE}), *args)
        check("without a builder the leg map comes with no fallback",
              fb is None and seen == [('run base', False)], f"{seen}")
        pcb._call_width_leg_base = build
        seen.clear()
        m, fb = rc.hybrid_leg_maps('run base', pcb, _cfg({'F.Cu': WIDE}), *args)
        fb_map = fb() if fb else None
        check("the fallback leg map is built on the call-width base and config",
              fb_map is not None and seen == [('run base', False), (first, True)],
              f"{seen}")
    finally:
        rc.build_diff_pair_leg_obstacles = real_legs


def wiring_rows():
    src = open(os.path.join(ROOT, 'py_router', 'diff_pair_routing.py'), encoding='utf-8').read()
    m = re.search(r"if leg_fallback is not None and _LEGS_UNATTACHED in _hyb_rej:(.*?)\n    return None\n",
                  src, re.S)
    check("the hybrid retries only when its legs did not attach",
          m is not None and 'leg_fallback()' in m.group(1), "")
    check("the retry carries no fallback of its own",
          m is not None and 'leg_fallback=' not in m.group(1))
    sites = 0
    for f in ('diff_pair_loop.py', 'diff_pair_multipoint.py', 'reroute_loop.py'):
        s = open(os.path.join(ROOT, 'py_router', f), encoding='utf-8').read()
        sites += len(re.findall(r"_route_direct_coupled_middle\((?:[^()]|\([^()]*\))*"
                                r"leg_fallback=leg_fallback", s))
    check("all four hybrid call sites pass their fallback", sites == 4, f"{sites}")


def main():
    from kicad_parser import parse_kicad_pcb
    td = tempfile.mkdtemp(prefix='t1736lf_')
    try:
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(BOARD)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(evidence(path, 'the staged board'))
    finally:
        shutil.rmtree(td, ignore_errors=True)
    geometry_rows(pcb)
    builder_rows(pcb)
    wiring_rows()
    if FAILS:
        print(f"\nFAIL: {len(FAILS)} row(s): {FAILS}")
        return 1
    print("\na hybrid's legs retry on call-width pad stamps when they cannot attach. OK")
    return 0


if __name__ == '__main__':
    sys.exit(main())
