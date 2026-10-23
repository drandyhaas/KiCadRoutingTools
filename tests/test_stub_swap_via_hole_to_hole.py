#!/usr/bin/env python3
"""A stub layer-switch pad via must respect hole-to-hole clearance
against EVERY existing hole -- including the same net's own via.

Wavelet RevB attempt-004: a STUB-SWAP RESCUE moved GNSS_V_BCKP's F.Cu stub to
B.Cu and drilled a pad via at R202.2 (-25.7,-14.59), 0.16 mm from the net's
existing 0.45/0.20 via at (-25.7,-14.75): overlapping drills. The rescue
paths (single_ended_loop stub-swap rescue, switch_boxed_stub_near, the
diff-pair multipoint switch) applied the swap without the via-fit funnel
(_swap_vias_fit_or_shrink) every layer_swap_optimization site runs.

Checks, one per site (each with a control that the fixture reaches it):
  1. switch_boxed_stub_near: refused next to the same-net via, the board
     left exactly as it was; a hole a smaller via can clear keeps the switch
     with the via shrunk, disclosed in the fab ledger like every other
     layer-swap pad via.
  2. diff_pair_multipoint._switch_blocked_terminal_stubs: refused next to a
     same-net via, board unchanged.
  3. route.py's STUB-SWAP RESCUE (single_ended_loop._stub_swap_rescue), on
     #1261's rescue board with a NET_C via 0.66 mm from the pad and
     --hole-to-hole-clearance 0.5: no rescue swap, no pad via inside the
     hole-to-hole floor, the board DRC-clean.
  4. The same with a rippable NET_B under the stub: the rescue rips NET_B,
     the funnel refuses the via, and NET_B's input copper ships
     byte-identical (the revert restores the rips it made).

    python3 tests/test_stub_swap_via_hole_to_hole.py
"""
import contextlib
import io
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, 'py_router'), os.path.join(_ROOT, 'py_tools'),
           os.path.join(_ROOT, 'tests')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import PCBData, BoardInfo, Segment, Pad, Via  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from stub_layer_switching import switch_boxed_stub_near  # noqa: E402
from run_utils import check as run_check, evidence, tool  # noqa: E402
from synth import RoutingBoard  # noqa: E402
import fab_tiers  # noqa: E402

NET = 12
H2H = 0.25
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + str(detail)) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def drill_gap(a, b):
    return math.hypot(a.x - b.x, a.y - b.y) - a.drill / 2 - b.drill / 2


def _board(via_offset=0.16):
    pad = Pad(component_ref='R202', pad_number='2', global_x=-25.7, global_y=-14.59,
              local_x=0, local_y=0, size_x=0.64, size_y=0.54, shape='roundrect',
              layers=['F.Cu', 'F.Mask', 'F.Paste'], net_id=NET, net_name='GNSS_V_BCKP')
    old_via = Via(x=-25.7, y=-14.59 - via_offset, size=0.45, drill=0.2,
                  layers=['F.Cu', 'B.Cu'], net_id=NET)
    tp = Pad(component_ref='TP206', pad_number='1', global_x=-25.7, global_y=-17.5,
             local_x=0, local_y=0, size_x=1.0, size_y=1.0, shape='circle',
             layers=['F.Cu', 'F.Mask'], net_id=NET, net_name='GNSS_V_BCKP')
    # The escape stub off R202.2 (free end at y=-12.59) and an unconnected
    # second island, so the net has two copper groups and a stub free end.
    stub = [Segment(start_x=-25.7, start_y=-14.59, end_x=-25.7, end_y=-13.59,
                    width=0.2, layer='F.Cu', net_id=NET),
            Segment(start_x=-25.7, start_y=-13.59, end_x=-25.7, end_y=-12.59,
                    width=0.2, layer='F.Cu', net_id=NET)]
    island = Segment(start_x=-25.7, start_y=-17.5, end_x=-25.7, end_y=-16.8,
                     width=0.2, layer='F.Cu', net_id=NET)
    pcb = PCBData(board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                       board_bounds=None),
                  nets={}, footprints={}, vias=[old_via] if via_offset else [],
                  segments=stub + [island], pads_by_net={NET: [pad, tp]})
    cfg = GridRouteConfig(grid_step=0.05, clearance=0.2, via_size=0.45,
                          via_drill=0.2, track_width=0.2,
                          hole_to_hole_clearance=H2H, layers=['F.Cu', 'B.Cu'])
    return pcb, cfg, old_via


def boxed_stub():
    print("1. switch_boxed_stub_near")
    pcb, cfg, _ = _board(via_offset=0)
    if quiet(switch_boxed_stub_near, pcb, NET, cfg, -25.7, -12.59) is None:
        check("BROKEN FIXTURE: with no via nearby the stub switches", False)
        return
    check("control: with no via nearby the stub switches", True)

    pcb, cfg, old_via = _board()
    res = quiet(switch_boxed_stub_near, pcb, NET, cfg, -25.7, -12.59)
    new_vias = [v for v in pcb.vias if v is not old_via]
    gaps = [round(drill_gap(v, old_via), 3) for v in new_vias]
    check("no new via inside hole-to-hole of the same-net via",
          all(g >= H2H - 1e-9 for g in gaps), f"drill gaps {gaps}")
    check("refused, and the board is exactly as it was",
          res is None and new_vias == [] and all(s.layer == 'F.Cu' for s in pcb.segments),
          f"switch {'kept' if res else 'refused'}, {len(new_vias)} new via(s)")

    # 0.42 mm away the nominal 0.45/0.20 via cannot clear the drill, a
    # 0.25/0.15 rung can: the switch stays, the via shrinks, and the shrink
    # is disclosed exactly as at the optimizer's apply sites.
    pcb, cfg, old_via = _board(via_offset=0.42)
    fab_tiers.reset_ledger()
    res = quiet(switch_boxed_stub_near, pcb, NET, cfg, -25.7, -12.59)
    new_vias = [v for v in pcb.vias if v is not old_via]
    rows = [r for r in fab_tiers._LEDGER['narrowed']
            if r['site'] == 'layer-swap pad via' and r['net'] == NET]
    check("a hole a smaller via clears: the switch is kept with the via shrunk",
          res is not None and len(new_vias) == 1 and new_vias[0].size < cfg.via_size,
          [(v.size, v.drill) for v in new_vias])
    check("and the shrink is in the fab ledger as a layer-swap pad via", bool(rows), rows)
    fab_tiers.reset_ledger()


class _WalledMap:
    """F.Cu (index 0) fully walled, B.Cu open."""
    def is_blocked(self, gx, gy, layer):
        return layer == 0


def diff_terminal():
    print("2. diff-pair multipoint terminal stub switch")
    from synth import make_net, make_pad, make_pcb, make_seg
    import diff_pair_multipoint as dpm

    def run(hole):
        pp = make_pad(1, 10.0, 10.0, ref='J1', num='1', size_x=0.6, size_y=0.6)
        nn = make_pad(2, 11.0, 10.0, ref='J1', num='2', size_x=0.6, size_y=0.6)
        old = Via(x=10.0, y=9.84, size=0.45, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=1)
        pcb = make_pcb(nets={}, vias=[old] if hole else [],
                       segments=[make_seg(10.0, 10.0, 10.0, 11.0, net_id=1),
                                 make_seg(11.0, 10.0, 11.0, 11.0, net_id=2)],
                       pads_by_net={1: [pp], 2: [nn]},
                       board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                                            copper_layers=['F.Cu', 'B.Cu'],
                                            board_bounds=(0.0, 0.0, 30.0, 30.0)))
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.1, track_width=0.2,
                              clearance=0.15, via_size=0.45, via_drill=0.2,
                              hole_to_hole_clearance=H2H)
        state = SimpleNamespace(config=cfg, pcb_data=pcb,
                                diff_pair_base_obstacles=_WalledMap())
        switched = quiet(dpm._switch_blocked_terminal_stubs, state, None, [(pp, nn)])
        return switched, sorted(s.layer for s in pcb.segments), len(pcb.vias)

    switched, layers, nvias = run(hole=False)
    if not switched:
        check("BROKEN FIXTURE: with no via nearby the terminal switches", False, layers)
        return
    check("control: with no via nearby the terminal switches", True, layers)
    switched, layers, nvias = run(hole=True)
    check("next to the same-net via: refused, board unchanged",
          not switched and layers == ['F.Cu', 'F.Cu'] and nvias == 1,
          f"{len(switched)} switch(es), stubs {layers}, {nvias} via(s)")


def rescue_board(rip, hole):
    """#1261's STUB-SWAP RESCUE board: NET_A's F.Cu stub inside a locked B.Cu
    ring of NET_C that admits a via only at P1's off-grid centre, the route
    restricted to In1/In2. `hole` adds a NET_C via on the ring 0.66 mm from
    P1 (P1 is 0.5 mm so the via clears the pad); `rip` puts an unlocked
    NET_B under the stub on both inner layers (test_1261_stub_swap_rescue_rip)."""
    b = RoutingBoard(layers=('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'))
    px, py, ex = 10.05, 15.05, 14.05
    b.pad('P1', px, py, 'NET_A', sx=0.5, sy=0.5)
    b.seg(px, py, ex, py, 'NET_A')
    if rip:
        b.pad('P2', 30, 15, 'NET_A', sx=1.6, sy=1.6, tht=True)
        b.seg(30, 15, 26, 15, 'NET_A', layer='In2.Cu')
        b.pad('B1', 12, 8, 'NET_B', sx=1.6, sy=1.6, tht=True)
        b.pad('B2', 12, 22, 'NET_B', sx=1.6, sy=1.6, tht=True)
        b.seg(12, 8, 12, 22, 'NET_B', layer='In1.Cu', uid='netb-in1')
        b.seg(12, 8, 12, 22, 'NET_B', layer='In2.Cu', uid='netb-in2')
    else:
        b.pad('P2', 30, 15, 'NET_A')
        b.seg(30, 15, 26, 15, 'NET_A')
    d = 0.25 + 0.25 + 0.15 + 0.01
    b.ring([(round(px - d, 4), round(py - d, 4)), (round(ex + d, 4), round(py - d, 4)),
            (round(ex + d, 4), round(py + d, 4)), (round(px - d, 4), round(py + d, 4))],
           'NET_C', w=0.3, layer='B.Cu', locked=True)
    b.pad('C1', 4, 15, 'NET_C', sx=1.6, sy=1.6, tht=True)
    b.pad('C2', 4, 25, 'NET_C', sx=1.6, sy=1.6, tht=True)
    b.seg(4, 15, round(px - d, 4), round(py - d, 4), 'NET_C', layer='B.Cu', locked=True)
    b.seg(4, 15, 4, 25, 'NET_C', layer='B.Cu', locked=True)
    if hole:
        b.via(round(px - d, 4), py, 'NET_C', size=0.3, drill=0.15, locked=True)
    return b.text()


SWAP = 'STUB-SWAP RESCUE: NET_A'
RIP = "PRE-EXISTING rip: 'NET_B'"
H2H_ROUTE = 0.5


def _vias(text):
    return [(float(x), float(y), float(d), int(n)) for x, y, d, n in re.findall(
        r'\(via\s+\(at ([-\d.]+) ([-\d.]+)\)\s+\(size [-\d.]+\)\s+\(drill ([-\d.]+)\)'
        r'.*?\(net (\d+)\)', text, re.S)]


def _route(tmp, tag, text):
    inp = os.path.join(tmp, f'{tag}_in.kicad_pcb')
    out = os.path.join(tmp, f'{tag}_out.kicad_pcb')     # fresh path per run
    with open(inp, 'w', encoding='utf-8') as fh:
        fh.write(text)
    evidence(inp, 'synthetic input board')
    env = dict(os.environ, KICAD_PLANE_FINALIZE='0')
    r = run_check([sys.executable, '-X', 'utf8', tool('route.py'), inp, out,
                   '--nets', '*', '--layers', 'In1.Cu', 'In2.Cu',
                   '--keep-input-copper', '--escalation', 'off',
                   '--no-fix-drc-settings',
                   '--hole-to-hole-clearance', str(H2H_ROUTE)], accept=True, env=env)
    evidence(out, 'routed output')
    with open(out, encoding='utf-8') as fh:
        return r.stdout + r.stderr, out, fh.read()


def _h2h_ok(board):
    """Every NET_A via clears the NET_C hole by the routed floor."""
    vias = _vias(board)
    net_c = [v for v in vias if abs(v[0] - 9.39) < 1e-6 and abs(v[1] - 15.05) < 1e-6]
    if len(net_c) != 1:
        return False, f"the NET_C hole is not in the output: {vias}"
    hx, hy, hd, hn = net_c[0]
    gaps = [round(math.hypot(x - hx, y - hy) - d / 2 - hd / 2, 3)
            for x, y, d, n in vias if n != hn]
    return all(g >= H2H_ROUTE - 1e-6 for g in gaps), f"drill gaps {gaps}"


def rescue_routes():
    tmp = tempfile.mkdtemp(prefix='t1271_')
    try:
        print("3. route.py STUB-SWAP RESCUE")
        con, _, _ = _route(tmp, 'control', rescue_board(rip=False, hole=False))
        if SWAP not in con:
            check("BROKEN FIXTURE: with no hole the rescue swaps NET_A's stub", False)
            return
        check("control: with no hole the rescue swaps NET_A's stub", True)
        con, out, board = _route(tmp, 'hole', rescue_board(rip=False, hole=True))
        check("next to the hole: no rescue swap",
              SWAP not in con, '; '.join(l.strip() for l in con.splitlines() if SWAP in l))
        ok, why = _h2h_ok(board)
        check("no NET_A via inside the hole-to-hole floor", ok, why)
        # check_drc exits 1 on a violation; read its verdict line, so a run
        # that died before grading fails here instead of reading as clean.
        d = subprocess.run([sys.executable, '-X', 'utf8', tool('check_drc.py'), out,
                            '--clearance', '0.25',
                            '--hole-to-hole-clearance', str(H2H_ROUTE)],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=_ROOT)
        verdict = re.search(r'NO DRC VIOLATIONS|FOUND \d+ DRC VIOLATIONS', d.stdout)
        check("the board is DRC-clean at the routed floors",
              verdict is not None and verdict.group(0) == 'NO DRC VIOLATIONS',
              verdict.group(0) if verdict else (d.stdout + d.stderr)[-300:])

        print("4. the rescue's rips are restored when the via is refused")
        con, out, board = _route(tmp, 'rip', rescue_board(rip=True, hole=True))
        check("the rescue rips the swap blocker NET_B", RIP in con)
        check("no rescue swap", SWAP not in con)
        netb = sorted(re.findall(r'\(segment [^\n]*"netb-in[12]"\)', board))
        with open(os.path.join(tmp, 'rip_in.kicad_pcb'), encoding='utf-8') as fh:
            netb_in = sorted(re.findall(r'\(segment [^\n]*"netb-in[12]"\)', fh.read()))
        check("NET_B's input copper ships byte-identical",
              len(netb_in) == 2 and netb == netb_in, f"{netb_in} -> {netb}")
        check("and is reported restored", "'NET_B': original copper restored" in con)
        ok, why = _h2h_ok(board)
        check("no NET_A via inside the hole-to-hole floor", ok, why)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    boxed_stub()
    diff_terminal()
    rescue_routes()
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
