#!/usr/bin/env python3
"""Compare a board's net-class design rules against the geometry actually used
in its real (downloaded) routing: track widths, via sizes/drills, per-class
usage, and the project's clearance rule. Reveals whether the Default net class
is a faithful baseline for the autorouter, whether non-Default classes matter,
and whether real routing uses per-track widths the class params don't capture.

Usage: python3 measure_routing.py <routed.kicad_pcb> [...]
"""
import sys
import os
import re
from collections import Counter, defaultdict

_here = os.path.dirname(os.path.abspath(__file__))
for _cand in (os.path.abspath(os.path.join(_here, "..", "..")),
              os.path.expanduser("~/Documents/KiCadRoutingTools")):
    if os.path.exists(os.path.join(_cand, "kicad_parser.py")):
        sys.path.insert(0, _cand)
        break
from kicad_parser import parse_kicad_pcb
from list_nets import read_design_rules


def pro_clearance(pcb_path):
    """Pull min clearance / track / via from the sibling .kicad_pro design rules."""
    pro = pcb_path.rsplit(".kicad_pcb", 1)[0] + ".kicad_pro"
    if not os.path.exists(pro):
        return {}
    txt = open(pro, errors="replace").read()
    out = {}
    for key, pat in (("min_clearance", r'"min_clearance"\s*:\s*([0-9.]+)'),
                     ("min_track_width", r'"min_track_width"\s*:\s*([0-9.]+)'),
                     ("min_via_diameter", r'"min_via_diameter"\s*:\s*([0-9.]+)'),
                     ("min_through_hole_diameter", r'"min_through_hole_diameter"\s*:\s*([0-9.]+)')):
        m = re.search(pat, txt)
        if m:
            out[key] = float(m.group(1))
    return out


def q(v):
    return round(v, 3)


def top(counter, n=6):
    return ", ".join(f"{w}mm×{c}" for w, c in counter.most_common(n))


def main():
    for path in sys.argv[1:]:
        name = os.path.basename(path).replace(".kicad_pcb", "")
        pcb = parse_kicad_pcb(path)
        dr = read_design_rules(path)
        classes = dr.get("classes", {}) if dr else {}
        assigns = dr.get("assignments", {}) if dr else {}

        # net_id -> class name
        netid_to_class = {}
        for nid, net in pcb.nets.items():
            netid_to_class[nid] = assigns.get(net.name, "Default")

        # actual track widths
        w_all = Counter()
        w_by_layer = defaultdict(Counter)
        w_by_class = defaultdict(Counter)
        for s in pcb.segments:
            w = q(s.width)
            w_all[w] += 1
            w_by_layer[s.layer][w] += 1
            w_by_class[netid_to_class.get(s.net_id, "Default")][w] += 1

        # actual vias
        via_c = Counter()
        for v in pcb.vias:
            via_c[(q(v.size), q(v.drill))] += 1

        pc = pro_clearance(path)

        print(f"\n{'='*78}\n{name}  ({len(pcb.board_info.copper_layers)} layers, "
              f"{len(pcb.segments)} segs, {len(pcb.vias)} vias, {len(pcb.nets)} nets)")
        print(f"  project rules: " + (", ".join(f"{k}={v}" for k, v in pc.items()) or "none in .kicad_pro"))
        print(f"  net classes ({len(classes)}): " + (", ".join(
            f"{cn}[cl={cd.get('clearance','?')},tw={cd.get('track_width','?')},"
            f"via={cd.get('via_diameter','?')}/{cd.get('via_drill','?')}"
            + (f",dpg={cd['diff_pair_gap']}" if 'diff_pair_gap' in cd else "")
            + (f",dpw={cd['diff_pair_width']}" if 'diff_pair_width' in cd else "")
            + "]" for cn, cd in classes.items()) or "none"))
        n_assigned = sum(1 for c in netid_to_class.values() if c != "Default")
        print(f"  nets assigned to non-Default class: {n_assigned}/{len(pcb.nets)}")
        print(f"  ACTUAL track widths (overall): {top(w_all)}")
        # per layer (copper only)
        for layer in pcb.board_info.copper_layers:
            if w_by_layer.get(layer):
                print(f"    {layer:8s}: {top(w_by_layer[layer], 5)}")
        if len([c for c in w_by_class if c != "Default"]):
            for cn, wc in w_by_class.items():
                print(f"    class {cn:14s}: {top(wc, 5)}")
        print(f"  ACTUAL vias (size/drill): " +
              ", ".join(f"{s}/{d}mm×{c}" for (s, d), c in via_c.most_common(6)))


if __name__ == "__main__":
    main()
