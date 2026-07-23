#!/usr/bin/env python3
"""#478: footprint-nested zones (keep-outs AND copper pours) must be parsed.

`_iter_zone_blocks` anchored on exactly one tab of indentation, so zones
owned by footprints (written at two tabs, KiCad 7+) were invisible to both
`extract_zones` and `extract_keepouts`: the router routed through footprint
keep-outs and the fill model over-credited across them. Footprint-zone
polygon points are stored in BOARD coordinates (no transform needed).

Plane creation must treat footprint pours as the footprint's business:
never a conflict veto, never a replace target, never excised by
remove_zones_from_content.

Run:
    python3 tests/test_footprint_zones.py
"""

import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import extract_zones, extract_keepouts, parse_kicad_pcb


FP_BOARD = '''(kicad_pcb
\t(version 20241229)
\t(net 0 "")
\t(net 1 "GND")
\t(net 2 "+3V3")
\t(footprint "lib:ufl"
\t\t(layer "F.Cu")
\t\t(at 10 5)
\t\t(zone
\t\t\t(net 0)
\t\t\t(net_name "")
\t\t\t(layers "F.Cu" "In1.Cu")
\t\t\t(name "fp_koz")
\t\t\t(hatch edge 0.5)
\t\t\t(keepout
\t\t\t\t(tracks not_allowed)
\t\t\t\t(vias not_allowed)
\t\t\t\t(copperpour not_allowed)
\t\t\t)
\t\t\t(polygon
\t\t\t\t(pts
\t\t\t\t\t(xy 9 -1) (xy 11 -1) (xy 11 11) (xy 9 11)
\t\t\t\t)
\t\t\t)
\t\t)
\t\t(zone
\t\t\t(net 2)
\t\t\t(net_name "+3V3")
\t\t\t(layer "F.Cu")
\t\t\t(name "fp_pour")
\t\t\t(hatch edge 0.5)
\t\t\t(min_thickness 0.25)
\t\t\t(polygon
\t\t\t\t(pts
\t\t\t\t\t(xy 14 2) (xy 16 2) (xy 16 4) (xy 14 4)
\t\t\t\t)
\t\t\t)
\t\t)
\t)
\t(zone
\t\t(net 1)
\t\t(net_name "GND")
\t\t(layer "F.Cu")
\t\t(hatch edge 0.5)
\t\t(connect_pads
\t\t\t(clearance 0.3)
\t\t)
\t\t(min_thickness 0.25)
\t\t(polygon
\t\t\t(pts
\t\t\t\t(xy 0 0) (xy 20 0) (xy 20 10) (xy 0 10)
\t\t\t)
\t\t)
\t)
)
'''


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    print("#478: footprint-nested zone parsing")
    kos = extract_keepouts(FP_BOARD)
    check("footprint keep-out extracted (was invisible)",
          len(kos) == 1 and kos[0]['layers'] == {'F.Cu', 'In1.Cu'})
    check("footprint keep-out flags parsed",
          kos and not kos[0]['copper_pour_allowed'] and kos[0]['in_footprint'])

    zones = extract_zones(FP_BOARD)
    board_z = [z for z in zones if not z.in_footprint]
    fp_z = [z for z in zones if z.in_footprint]
    check("board pour still parsed, in_footprint=False",
          len(board_z) == 1 and board_z[0].net_id == 1)
    check("footprint pour extracted, in_footprint=True",
          len(fp_z) == 1 and fp_z[0].net_id == 2
          and fp_z[0].polygon[0] == (14.0, 2.0))

    # Legacy formats keep parsing: single-tab v5 single-line zone (369 A5)
    # and a space-indented zone (defensive; pre-v7 files had no fp zones).
    v5 = ('(kicad_pcb\n'
          '\t(zone (net 1) (net_name "GND") (layer F.Cu) (tstamp 0)\n'
          '\t\t(polygon (pts (xy 0 0) (xy 5 0) (xy 5 5) (xy 0 5)))\n'
          '\t)\n)\n')
    check("single-tab v5 zone still parses (#369 A5)",
          len(extract_zones(v5)) == 1)
    sp = v5.replace('\n\t(zone', '\n  (zone')
    check("space-indented zone parses, not footprint-flagged",
          len(extract_zones(sp)) == 1 and not extract_zones(sp)[0].in_footprint)

    print("\n#478: plane creation ignores footprint pours")
    from plane_io import (ZoneInfo, check_existing_zones,
                          filter_zones_from_content)
    fp_pour = ZoneInfo(net_id=2, net_name='+3V3', layer='In2.Cu',
                       in_footprint=True)
    board_pour = ZoneInfo(net_id=2, net_name='+3V3', layer='In2.Cu')
    ok, cont, repl = check_existing_zones([fp_pour], 'In2.Cu', 'GND', 1)
    check("different-net FOOTPRINT pour does not veto the plane",
          ok and cont and repl is None)
    ok, cont, repl = check_existing_zones([board_pour], 'In2.Cu', 'GND', 1)
    check("different-net BOARD pour still vetoes", not ok and not cont)
    ok, cont, repl = check_existing_zones(
        [ZoneInfo(net_id=1, net_name='GND', layer='In2.Cu',
                  in_footprint=True)], 'In2.Cu', 'GND', 1)
    check("same-net footprint pour is not a replace target", repl is None)

    out = filter_zones_from_content(FP_BOARD, [(2, 'F.Cu')], None)
    check("filter_zones keeps the footprint-owned pour",
          '"fp_pour"' in out)
    out = filter_zones_from_content(FP_BOARD, [(1, 'F.Cu')], None)
    check("filter_zones still removes the board pour",
          '(net 1)' not in out.replace('(net 1 "GND")', ''))

    print("\n#478: end-to-end -- footprint copperpour keep-out splits the fill model")
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'fp.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(FP_BOARD)
        pcb = parse_kicad_pcb(p)
        check("parsed board carries the footprint keep-out",
              len(pcb.board_info.keepouts) == 1)
        from plane_fill_model import get_fill_models
        models = get_fill_models(pcb, 1)
        m = models.get('F.Cu', [None])[0]
        left = m.query_component(4.0, 5.0) if m else None
        right = m.query_component(18.0, 8.0) if m else None
        check("GND fill split by the footprint keep-out strip",
              bool(left) and bool(right) and left != right)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
