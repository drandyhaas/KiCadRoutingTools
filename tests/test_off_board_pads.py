#!/usr/bin/env python3
"""Off-board pads must be skipped, never routed toward (issue #291).

The edge keep-out only blocks a few grid cells past the outline bbox, so an
edge between TWO off-board pads used to route entirely in the unblocked space
beyond it, drawing real copper 20-50mm off the board (framework_dock /
pico_disp / apple1 corpus boards). Pads clearly outside the Edge.Cuts outline
are now dropped from every routing endpoint set:

- check_drc.make_off_board_test: the shared test (outline + cutouts, bbox
  fallback, edge-hugging tolerance)
- connectivity.get_net_endpoints / get_multipoint_net_pads: single-ended and
  multipoint routing (route.py, GUI, rip/reroute loops)
- plane_obstacle_builder.identify_target_pads: route_planes.py taps
- plane_pad_tap.find_unconnected_plane_pads: route_disconnected_planes.py

    python3 tests/test_off_board_pads.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb
from check_drc import make_off_board_test


def _rect_edge(x1, y1, x2, y2):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    out = []
    for i in range(4):
        a, b = pts[i], pts[(i + 1) % 4]
        out.append(f' (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
                   f'(layer "Edge.Cuts") (width 0.1))')
    return "\n".join(out)


def _fp(ref, x, y, pads):
    """pads: list of (pad_number, local_x, local_y, net_id, net_name)"""
    body = "\n".join(
        f'  (pad "{n}" smd rect (at {lx} {ly}) (size 1 1) (layers "F.Cu") '
        f'(net {nid} "{nname}"))'
        for n, lx, ly, nid, nname in pads)
    return (f' (footprint "test:{ref}" (layer "F.Cu") (at {x} {y})\n'
            f'  (property "Reference" "{ref}" (at 0 -2) (layer "F.SilkS"))\n'
            f'{body}\n )')


def _board(footprints, nets):
    net_decls = "\n".join(f' (net {nid} "{name}")' for nid, name in nets)
    return ('(kicad_pcb (version 20221018) (generator test)\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            ' (net 0 "")\n'
            f'{net_decls}\n'
            f'{_rect_edge(0, 0, 30, 30)}\n'
            + "\n".join(footprints) + '\n)')


def run():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            fails.append(name)

    # --- make_off_board_test unit checks ---
    board = _board([_fp('R1', 5, 5, [('1', 0, 0, 1, 'A')])], [(1, 'A')])
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(board)
        path = f.name
    try:
        pcb = parse_kicad_pcb(path)
        off = make_off_board_test(pcb.board_info)
        check("test exists when outline parsed", off is not None)
        check("inside point on board", not off(15, 15))
        check("point 20mm out is off-board", off(50, 15))
        check("point ON the outline stays routable", not off(30, 15))
        check("point 0.3mm out (edge-hugging pad) stays routable", not off(30.3, 15))
        check("point 1mm out is off-board", off(31.0, 15))
    finally:
        os.unlink(path)

    # No outline at all -> no test possible
    from types import SimpleNamespace
    check("no outline/bounds -> None",
          make_off_board_test(SimpleNamespace(board_outline=[], board_cutouts=[],
                                              board_bounds=None)) is None)
    # bbox fallback
    off_bb = make_off_board_test(SimpleNamespace(board_outline=[], board_cutouts=[],
                                                 board_bounds=(0, 0, 30, 30)))
    check("bbox fallback: inside", not off_bb(15, 15))
    check("bbox fallback: outside", off_bb(35, 15))

    # --- routing must draw no copper toward off-board pads ---
    # Net 1 "OFF2": 2-pad net, far pad 30mm off-board -> skipped, fails fast.
    # Net 2 "OFFMP": multipoint, 2 pads on-board + 2 clustered off-board ->
    #   the off-board cluster must NOT be connected off-board (the #291 bug);
    #   the on-board pair must still route.
    footprints = [
        _fp('R1', 5, 5, [('1', 0, 0, 1, 'OFF2')]),
        _fp('R2', 60, 15, [('1', 0, 0, 1, 'OFF2')]),
        _fp('U1', 10, 25, [('1', 0, 0, 2, 'OFFMP'), ('2', 3, 0, 2, 'OFFMP')]),
        _fp('U2', 60, 40, [('1', 0, 0, 2, 'OFFMP'), ('2', 3, 0, 2, 'OFFMP')]),
    ]
    board = _board(footprints, [(1, 'OFF2'), (2, 'OFFMP')])
    tmpdir = tempfile.mkdtemp()
    inp = os.path.join(tmpdir, 'in.kicad_pcb')
    outp = os.path.join(tmpdir, 'out.kicad_pcb')
    with open(inp, 'w') as f:
        f.write(board)
    from route import batch_route
    batch_route(inp, outp, ['OFF2', 'OFFMP'], layers=['F.Cu', 'B.Cu'],
                track_width=0.2, clearance=0.2, grid_step=0.1)
    routed = parse_kicad_pcb(outp)
    bb = routed.board_info.board_bounds

    def seg_off(s):
        return not (bb[0] - 0.001 <= s.start_x <= bb[2] + 0.001 and
                    bb[1] - 0.001 <= s.start_y <= bb[3] + 0.001 and
                    bb[0] - 0.001 <= s.end_x <= bb[2] + 0.001 and
                    bb[1] - 0.001 <= s.end_y <= bb[3] + 0.001)

    off_segs = [s for s in routed.segments if seg_off(s)]
    off_vias = [v for v in routed.vias
                if not (bb[0] <= v.x <= bb[2] and bb[1] <= v.y <= bb[3])]
    check("no segments outside the outline", not off_segs)
    check("no vias outside the outline", not off_vias)
    onboard_mp = [s for s in routed.segments if s.net_id == 2]
    check("on-board multipoint pair still routed", len(onboard_mp) >= 1)

    # --- plane tools skip off-board pads ---
    from plane_obstacle_builder import identify_target_pads
    from plane_pad_tap import find_unconnected_plane_pads
    pcb2 = parse_kicad_pcb(inp)
    tp = identify_target_pads(pcb2, 2, 'B.Cu')
    off_types = [p['type'] for p in tp
                 if not (bb[0] <= p['pad'].global_x <= bb[2])]
    check("identify_target_pads marks off-board pads",
          off_types and all(t == 'off_board' for t in off_types))
    check("off-board pads need no via/trace",
          all(not p['needs_via'] and not p['needs_trace']
              for p in tp if p['type'] == 'off_board'))
    unconn = find_unconnected_plane_pads(pcb2, 2, {'B.Cu'})
    check("find_unconnected_plane_pads skips off-board pads",
          all(bb[0] <= pad.global_x <= bb[2] for pad, _ in unconn))

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
