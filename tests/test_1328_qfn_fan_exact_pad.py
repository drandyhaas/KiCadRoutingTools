#!/usr/bin/env python3
"""qfn_fanout cuts a fan back only for a pad it really grazes (found
replaying #1328's chains).

The stub graze test asked two things of every foreign pad: the obstacle map
(`check_line_clearance`) and its own per-pad loop. The map is built with
extra_clearance = track_width / 2 on top of stamps that already carry the
track's half-width -- the guard its nearest-cell sampling needs -- so every
pad was held a further half track width (0.05-0.075 mm) away, and the per-pad
loop's inflated box squared off round corners. uhk_trackball's /NCS fan was
cut back at a tip 0.188 mm from R2.1 with 0.175 mm needed.

Pad copper is now left off that map (build_base_obstacle_map(stamp_pads=
False)); the per-pad loop measures it exactly (check_drc.segment_pad_gap)
at the larger of the fan clearance and the clearance the map stamped.

Checks:
  1. The builder: stamp_pads=False leaves pad copper off the map and keeps
     the NPTH hole keep-out; the default still stamps the pad.
  2. On tigard U3 (QFN-64, 0.5 mm), a foreign 0.8 mm pad whose flat side
     /AD1's full fan meets obliquely at its tip, 20 um beyond its reach (the
     /NCS geometry). Precondition: the pad-stamped map the stub test used
     refuses that fan, and accepts it without the pad.
     The fan now ships at full length, clearing the pad by check_drc's
     measure, DRC-clean.
  3. Negative control: the same pad 20 um INSIDE the reach -- the fan is cut
     back, and the copper it ships clears the pad.

    python3 tests/test_1328_qfn_fan_exact_pad.py
"""
import copy
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'rust_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import parse_kicad_pcb, BoardInfo, Net                # noqa: E402
from routing_config import GridRouteConfig, GridCoord                   # noqa: E402
from obstacle_map import (build_base_obstacle_map, build_layer_map,     # noqa: E402
                          check_line_clearance)
from kicad_dru import install_layer_clearances                          # noqa: E402
from check_drc import segment_pad_gap                                   # noqa: E402
from qfn_fanout import generate_qfn_fanout                              # noqa: E402
from synth import make_pad, make_pcb                                    # noqa: E402

RUN_ALL_TIMEOUT = 300

BOARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
TW, CLR = 0.1, 0.1
NEED = CLR + TW / 2
NET = '/AD1'
fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL') + f': {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        fails.append(name)


# ---- 1. the builder -------------------------------------------------------
print("1. build_base_obstacle_map(stamp_pads=False)")
smd = make_pad(2, 5.0, 5.0, num='1', size_x=1.0, size_y=1.0)
npth = make_pad(0, 8.0, 5.0, num='MH', size_x=1.0, size_y=1.0, shape='circle',
                layers=('*.Cu', '*.Mask'), drill=1.0, pad_type='np_thru_hole')
syn = make_pcb(pads_by_net={2: [smd], 0: [npth]},
               nets={0: Net(0, ''), 1: Net(1, 'A'), 2: Net(2, 'B')},
               board_info=BoardInfo(layers={0: 'F.Cu'}, board_bounds=(0, 0, 20, 10),
                                    copper_layers=['F.Cu']))
scfg = GridRouteConfig(layers=['F.Cu'], track_width=TW, clearance=CLR, grid_step=0.1)
sc = GridCoord(0.1)
on = build_base_obstacle_map(syn, scfg, nets_to_route=[1])
off = build_base_obstacle_map(syn, scfg, nets_to_route=[1], stamp_pads=False)
check("the default stamps the pad's copper", on.is_blocked(*sc.to_grid(5.0, 5.0), 0))
check("stamp_pads=False leaves it off", not off.is_blocked(*sc.to_grid(5.0, 5.0), 0))
check("...and keeps the NPTH hole keep-out", off.is_blocked(*sc.to_grid(8.0, 5.0), 0)
      and off.is_blocked(*sc.to_grid(8.0 + 0.55, 5.0), 0))

# ---- 2/3. tigard U3 -------------------------------------------------------
if not os.path.exists(BOARD):
    print(f"FAIL: board missing ({BOARD})")
    sys.exit(1)


def fanout(pcb):
    """(tracks, tip of /AD1's stub, dropped): the stub is pad -> corner -> tip,
    and a fan cut back to nothing leaves only the pad -> corner leg."""
    u3 = pcb.footprints['U3']
    nets = sorted({p.net_name for p in u3.pads if p.net_name and 'GND' not in p.net_name})
    tracks, _v, dropped = generate_qfn_fanout(
        u3, pcb, net_filter=nets, layer='F.Cu', track_width=TW, clearance=CLR,
        grid_step=0.05, escape_method='stub')
    nid = next(i for i, n in pcb.nets.items() if n.name == NET)
    legs = [t for t in tracks if t['net_id'] == nid]
    starts = {tuple(t['start']) for t in legs}
    tips = [tuple(t['end']) for t in legs if tuple(t['end']) not in starts]
    assert len(tips) == 1, legs
    return tracks, tips[0], dropped


def old_map_accepts(pcb, a, b):
    """The stub test's map as it was built: pads stamped, extra = TW/2."""
    u3 = pcb.footprints['U3']
    cfg = GridRouteConfig(layers=list(pcb.board_info.copper_layers), track_width=TW,
                          clearance=CLR)
    install_layer_clearances(cfg, None, None, pcb)
    obs = build_base_obstacle_map(pcb, cfg, nets_to_route=[p.net_id for p in u3.pads
                                                           if p.net_id],
                                  extra_clearance=TW / 2)
    return check_line_clearance(obs, a[0], a[1], b[0], b[1],
                                build_layer_map(cfg.layers)['F.Cu'], cfg)


pcb0 = parse_kicad_pcb(BOARD)
_t0, TIP, _d = fanout(pcb0)
_nid = next(i for i, n in pcb0.nets.items() if n.name == NET)
CORNER = next(tuple(t['start']) for t in _t0 if t['net_id'] == _nid and tuple(t['end']) == TIP)
L0 = math.hypot(TIP[0] - CORNER[0], TIP[1] - CORNER[1])
dx, dy = (TIP[0] - CORNER[0]) / L0, (TIP[1] - CORNER[1]) / L0
check(f"{NET}'s fan is a full 45-degree leg on the clean board, heading -x",
      abs(abs(dx) - abs(dy)) < 1e-6 and dx < 0 and L0 > 1.0, f"{CORNER} -> {TIP}")
H = 0.4


def with_pad(slack):
    """tigard with a 0.8 mm pad of a net foreign to U3 (the map leaves every
    U3 net off) just left of /AD1's tip, its right side `NEED + slack` from
    it: the fan meets that flat side obliquely and the tip is its nearest
    point -- uhk_trackball's /NCS fan against R2.1."""
    pcb = parse_kicad_pcb(BOARD)
    nid = max(pcb.nets) + 1
    pcb.nets[nid] = Net(nid, '/X')
    pad = make_pad(nid, TIP[0] - NEED - slack - H, TIP[1], ref='R99', num='1',
                   net_name='/X', size_x=2 * H, size_y=2 * H)
    pcb.pads_by_net.setdefault(nid, []).append(pad)
    return pcb, pad


print("2. a pad 20 um beyond the full fan's reach")
pcb2, pad2 = with_pad(+0.02)
g = segment_pad_gap(CORNER[0], CORNER[1], TIP[0], TIP[1], pad2) - TW / 2
check("the full fan clears it by 20 um (check_drc measure)", abs(g - CLR - 0.02) < 1e-6,
      f"gap {g:.4f}")
check("precondition: the old pad-stamped map refuses the full fan",
      not old_map_accepts(pcb2, CORNER, TIP))
check("...and accepts it without the pad", old_map_accepts(pcb0, CORNER, TIP))
tracks2, tip2, dropped2 = fanout(pcb2)
check("the fan ships at full length",
      math.hypot(tip2[0] - TIP[0], tip2[1] - TIP[1]) < 1e-6, f"tip {tip2} vs {TIP}")
worst2 = min(segment_pad_gap(t['start'][0], t['start'][1], t['end'][0], t['end'][1],
                             pad2) - t['width'] / 2 for t in tracks2)
check("no track grazes the pad", worst2 >= CLR - 1e-9, f"min gap {worst2:.4f}")
check("nothing dropped", not dropped2, str(dropped2))

print("3. negative control: the pad 20 um inside the reach")
pcb3, pad3 = with_pad(-0.02)
tracks3, tip3, _d3 = fanout(pcb3)
L3 = math.hypot(tip3[0] - CORNER[0], tip3[1] - CORNER[1])
check("the fan is cut back", L3 < L0 - 1e-6, f"{L3:.3f} of {L0:.3f} mm")
worst3 = min(segment_pad_gap(t['start'][0], t['start'][1], t['end'][0], t['end'][1],
                             pad3) - t['width'] / 2 for t in tracks3)
check("and what ships clears the pad", worst3 >= CLR - 1e-9, f"min gap {worst3:.4f}")

if fails:
    print(f"\nFAILED: {fails}")
    sys.exit(1)
print("\nALL PASSED")
