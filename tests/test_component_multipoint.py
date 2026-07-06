#!/usr/bin/env python3
"""Regression test for issue #317: multipoint routing must be COMPONENT-based.

The old pad-position MST filtered edges with a 0.02mm endpoint-coincidence
grouping, so copper joined only by cap overlap (a soft joint) was treated as
disconnected: the router re-tapped already-connected islands with redundant
loops while the genuinely missing edge could still fail (butterstick DQ5).

Now the terminals are grouped by the authoritative overlap-aware definition
(check_net_connectivity) and an MST over the COMPONENTS routes exactly N-1
connections between nearest terminal pairs:

  1. two islands soft-jointed by cap overlap + one far pad -> ONE routed edge
     joining the far pad, never a redundant island-to-island tap;
  2. all islands already overlap-connected -> nothing routed, net reported
     successful (not failed);
  3. unit level: get_copper_connected_terminal_groups groups soft-jointed
     islands; compute_component_mst_edges yields N_components - 1 edges.

    python3 tests/test_component_multipoint.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from routing_config import GridRouteConfig
from connectivity import (
    get_multipoint_net_pads,
    get_copper_connected_terminal_groups,
    compute_component_mst_edges,
)
from check_connected import check_net_connectivity


def _board(connect_c=False):
    """3 single-pad footprints on net /MULTI plus existing copper:
      island1: pad A (2,2) -> (5,2)
      island2: (5.05,2) -> pad B (8,2)     [0.05mm gap, width 0.2 -> caps
                                            OVERLAP: a soft joint, connected
                                            by the authoritative definition]
      pad C at (14,2): unconnected -- unless connect_c, which adds
      island3: (8.05,2) -> pad C, soft-jointed onto island2, so ALL copper
      is one overlap-connected component (3 coincidence islands).
    """
    extra = ''
    if connect_c:
        extra = ('\t(segment (start 8.05 2) (end 14 2) (width 0.2) '
                 '(layer "F.Cu") (net 1) (uuid "s3"))\n')
    return f'''(kicad_pcb
\t(version 20221018)
\t(net 0 "")
\t(net 1 "/MULTI")
\t(gr_line (start 0 0) (end 20 0) (layer "Edge.Cuts") (width 0.1))
\t(gr_line (start 20 0) (end 20 8) (layer "Edge.Cuts") (width 0.1))
\t(gr_line (start 20 8) (end 0 8) (layer "Edge.Cuts") (width 0.1))
\t(gr_line (start 0 8) (end 0 0) (layer "Edge.Cuts") (width 0.1))
\t(footprint "L:A" (layer "F.Cu") (at 2 2)
\t\t(property "Reference" "U1")
\t\t(pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/MULTI")))
\t(footprint "L:B" (layer "F.Cu") (at 8 2)
\t\t(property "Reference" "U2")
\t\t(pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/MULTI")))
\t(footprint "L:C" (layer "F.Cu") (at 14 2)
\t\t(property "Reference" "U3")
\t\t(pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/MULTI")))
\t(segment (start 2 2) (end 5 2) (width 0.2) (layer "F.Cu") (net 1) (uuid "s1"))
\t(segment (start 5.05 2) (end 8 2) (width 0.2) (layer "F.Cu") (net 1) (uuid "s2"))
{extra})'''


def _write_board(text):
    f = tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False)
    f.write(text)
    f.close()
    return f.name


def _route(board_path):
    out = tempfile.mktemp(suffix=".kicad_pcb")
    cmd = [sys.executable, "route.py", board_path,
           "--output", out,
           "--nets", "/MULTI",
           "--layers", "F.Cu", "B.Cu",
           "--clearance", "0.1", "--via-size", "0.3",
           "--via-drill", "0.2", "--track-width", "0.2"]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    txt = r.stdout + r.stderr
    return txt, out


def _json_summary(txt):
    m = re.search(r'JSON_SUMMARY: (\{.*\})', txt)
    return json.loads(m.group(1)) if m else None


def run():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            fails.append(name)

    # --- unit level: grouping + component MST -------------------------------
    board = _write_board(_board())
    try:
        pcb = parse_kicad_pcb(board)
        net_id = next(nid for nid, n in pcb.nets.items() if n.name == '/MULTI')
        config = GridRouteConfig(track_width=0.2, clearance=0.1, via_size=0.3,
                                 via_drill=0.2, layers=['F.Cu', 'B.Cu'])
        pad_info = get_multipoint_net_pads(pcb, net_id, config)
        check("net detected as multipoint (3 terminals)",
              pad_info is not None and len(pad_info) == 3)
        comps = get_copper_connected_terminal_groups(pcb, net_id, pad_info)
        n_comps = len(set(comps.values()))
        check("soft-jointed islands grouped as ONE component (2 total)",
              n_comps == 2)
        positions = [(pi[3], pi[4]) for pi in pad_info]
        edges = compute_component_mst_edges(positions, comps)
        check("component MST has exactly N-1 = 1 edge", len(edges) == 1)
        if edges:
            a, b, _ = edges[0]
            check("MST edge joins the two components",
                  comps[a] != comps[b])
    finally:
        os.unlink(board)

    # --- end to end: one soft joint + one far pad -> 1 routed edge ----------
    board = _write_board(_board())
    txt, out = _route(board)
    try:
        summary = _json_summary(txt)
        check("route.py reports success", summary and summary.get('failed') == 0)
        check("terminals collapsed to 2 groups in the log",
              "into 2 group(s)" in txt)
        check("exactly 1 multipoint edge routed",
              summary and summary.get('multipoint_edges_routed') == 1)
        if os.path.exists(out):
            pcb2 = parse_kicad_pcb(out)
            nid2 = next(nid for nid, n in pcb2.nets.items() if n.name == '/MULTI')
            res = check_net_connectivity(
                nid2,
                [s for s in pcb2.segments if s.net_id == nid2],
                [v for v in pcb2.vias if v.net_id == nid2],
                pcb2.pads_by_net.get(nid2, []),
                [z for z in pcb2.zones if z.net_id == nid2])
            check("output net authoritatively connected", res['connected'])
        else:
            check("output written", False)
    finally:
        os.unlink(board)
        for ext in (".kicad_pcb", ".kicad_pro"):
            p = out[:-len(".kicad_pcb")] + ext
            if os.path.exists(p):
                os.remove(p)

    # --- everything overlap-connected -> nothing to route -------------------
    # End to end, route.py's upstream already-routed detection skips the net
    # before multipoint even runs; assert that skip (and no failure). The
    # route_multipoint_main early return is the safety net for when the two
    # definitions disagree -- exercise it directly (it returns before ever
    # touching the obstacle map, so obstacles=None is fine).
    board = _write_board(_board(connect_c=True))
    txt, out = _route(board)
    try:
        summary = _json_summary(txt)
        check("already-connected net skipped or successful, never failed",
              ("Already fully connected" in txt and "0 failed" not in txt)
              or (summary and summary.get('failed') == 0))

        from single_ended_routing import route_multipoint_main
        pcb3 = parse_kicad_pcb(board)
        nid3 = next(nid for nid, n in pcb3.nets.items() if n.name == '/MULTI')
        config3 = GridRouteConfig(track_width=0.2, clearance=0.1, via_size=0.3,
                                  via_drill=0.2, layers=['F.Cu', 'B.Cu'])
        pad_info3 = get_multipoint_net_pads(pcb3, nid3, config3)
        check("connect_c board still detected as multipoint (3 islands)",
              pad_info3 is not None and len(pad_info3) == 3)
        result = route_multipoint_main(pcb3, nid3, config3, None, pad_info3)
        check("route_multipoint_main returns already_connected success",
              result is not None and result.get('already_connected')
              and not result.get('failed'))
        check("already_connected result marks every terminal routed",
              result is not None
              and result.get('routed_pad_indices') == set(range(len(pad_info3))))
    finally:
        os.unlink(board)
        for ext in (".kicad_pcb", ".kicad_pro"):
            p = out[:-len(".kicad_pcb")] + ext
            if os.path.exists(p):
                os.remove(p)

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("\nall checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
