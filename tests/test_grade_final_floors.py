"""grade_final.redo_floors must extract the SMALLEST value each DRC-floor flag
took across the recorded chain, so the shipped board's .kicad_pro is written at
the tightest clearance actually routed (not a later step's looser nominal). A
bare `cp <step> <final>` strands the sibling .kicad_pro, so without this the
delivered final grades at a step default and reads phantom violations in KiCad
(#403/#326).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tests", "stress"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "stress"))

from grade_final import redo_floors, redo_final_and_clearance


MANIFEST = """\
# cwd=/x
python3 -X utf8 /r/route.py in.kicad_pcb step3_signal.kicad_pcb --clearance 0.09 --track-width 0.1 --via-size 0.45 --via-drill 0.25
python3 -X utf8 /r/route_disconnected_planes.py step3_signal.kicad_pcb step5_repair.kicad_pcb --clearance 0.1 --hole-to-hole-clearance 0.2 --via-size 0.35 --via-drill 0.2
cp step5_repair.kicad_pcb board_final.kicad_pcb
python3 -X utf8 /r/route_disconnected_planes.py board_final.kicad_pcb board_step7.kicad_pcb --clearance 0.1 --via-size 0.35 --board-edge-clearance 0.15
python3 -X utf8 /r/check_drc.py board_step7.kicad_pcb --clearance 0.5
"""


def main():
    fails = 0
    floors = redo_floors(MANIFEST)

    # clearance: min(0.09, 0.1, 0.1) = 0.09 -- the check_drc 0.5 line is ignored.
    if floors.get('clearance') != 0.09:
        print(f"FAIL: clearance floor = {floors.get('clearance')}, want 0.09"); fails += 1
    else:
        print("PASS: clearance takes the chain minimum (0.09), ignoring check_ lines")

    # via_diameter: min(0.45, 0.35, 0.35) = 0.35
    if floors.get('via_diameter') != 0.35:
        print(f"FAIL: via_diameter = {floors.get('via_diameter')}, want 0.35"); fails += 1
    else:
        print("PASS: via_size -> via_diameter minimum (0.35)")

    # via_drill: min(0.25, 0.2) = 0.2 ; track_width: 0.1 ; hole_to_hole: 0.2 ; edge: 0.15
    for kw, want in (('via_drill', 0.2), ('track_width', 0.1),
                     ('hole_to_hole', 0.2), ('edge_clearance', 0.15)):
        if floors.get(kw) != want:
            print(f"FAIL: {kw} = {floors.get(kw)}, want {want}"); fails += 1
        else:
            print(f"PASS: {kw} minimum ({want})")

    # the check_drc line's --clearance 0.5 must NOT leak into the floors
    if floors.get('clearance', 0) > 0.09:
        print("FAIL: a check_ command's --clearance leaked into the floors"); fails += 1

    # redo_final_and_clearance still resolves the final board + min clearance
    final, clr = redo_final_and_clearance(MANIFEST)
    if final != 'board_step7.kicad_pcb' or abs(clr - 0.09) > 1e-9:
        print(f"FAIL: redo_final_and_clearance = ({final}, {clr}), want (board_step7.kicad_pcb, 0.09)"); fails += 1
    else:
        print("PASS: redo_final_and_clearance resolves final + chain-min clearance")

    if fails:
        print(f"\n{fails} test(s) FAILED"); return 1
    print("\nAll grade_final floor-parse tests passed"); return 0


if __name__ == "__main__":
    sys.exit(main())
