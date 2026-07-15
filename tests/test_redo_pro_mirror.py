"""A `cp`/`mv` of a routed board in a replayed manifest must carry its sibling
`.kicad_pro` (the recorded DRC floor), or the renamed board grades at the board's
looser design default and manufactures phantom clearance/hole violations on
legitimately fine-tapped copper (core1106_cam: clean at the routed 0.09, graded
at the 0.2 default when `board_final.kicad_pro` went missing; #403/#326).

Covers `mirror_project_sibling` in tests/stress/redo_stress_test.py.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tests", "stress"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "stress"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "stress"))

from redo_stress_test import mirror_project_sibling


def _mk(d, name, body="(kicad_pcb)"):
    with open(os.path.join(d, name), "w") as f:
        f.write(body)


def main():
    fails = 0
    with tempfile.TemporaryDirectory() as d:
        # 1) cp <src> <dst>: mirrors the .kicad_pro, content byte-preserved.
        _mk(d, "step5c.kicad_pcb")
        _mk(d, "step5c.kicad_pro", '{"floor":0.09}')
        mirror_project_sibling(["cp", "step5c.kicad_pcb", "final.kicad_pcb"], d)
        dst = os.path.join(d, "final.kicad_pro")
        if not (os.path.isfile(dst) and open(dst).read() == '{"floor":0.09}'):
            print("FAIL: cp did not mirror .kicad_pro with preserved content"); fails += 1
        else:
            print("PASS: cp mirrors sibling .kicad_pro")

        # 2) mv moves the .kicad_pro (source removed).
        _mk(d, "a.kicad_pcb"); _mk(d, "a.kicad_pro", "{}")
        mirror_project_sibling(["mv", "a.kicad_pcb", "b.kicad_pcb"], d)
        if not (os.path.isfile(os.path.join(d, "b.kicad_pro"))
                and not os.path.isfile(os.path.join(d, "a.kicad_pro"))):
            print("FAIL: mv did not move .kicad_pro"); fails += 1
        else:
            print("PASS: mv moves sibling .kicad_pro")

        # 3) a routing command (not cp/mv) is a no-op even when both args are boards.
        _mk(d, "x.kicad_pcb")
        mirror_project_sibling(["route.py", "x.kicad_pcb", "y.kicad_pcb"], d)
        if os.path.isfile(os.path.join(d, "y.kicad_pro")):
            print("FAIL: non-cp command mirrored a .kicad_pro"); fails += 1
        else:
            print("PASS: non-cp command is a no-op")

        # 4) cp whose source has no .kicad_pro: no-op, no crash.
        _mk(d, "nopro.kicad_pcb")
        mirror_project_sibling(["cp", "nopro.kicad_pcb", "z.kicad_pcb"], d)
        if os.path.isfile(os.path.join(d, "z.kicad_pro")):
            print("FAIL: mirrored a .kicad_pro that did not exist"); fails += 1
        else:
            print("PASS: missing source .kicad_pro is a no-op")

        # 5) absolute paths (cwd=None) resolve correctly.
        _mk(d, "s2.kicad_pcb"); _mk(d, "s2.kicad_pro", "{}")
        mirror_project_sibling(
            ["cp", os.path.join(d, "s2.kicad_pcb"), os.path.join(d, "f2.kicad_pcb")], None)
        if not os.path.isfile(os.path.join(d, "f2.kicad_pro")):
            print("FAIL: absolute-path cp did not mirror"); fails += 1
        else:
            print("PASS: absolute-path cp mirrors")

        # 6) cp with extra flags / not a plain 2-board rename: left alone.
        _mk(d, "m1.kicad_pcb"); _mk(d, "m1.kicad_pro", "{}"); _mk(d, "m2.kicad_pcb")
        mirror_project_sibling(["cp", "m1.kicad_pcb", "m2.kicad_pcb", "adir.kicad_pcb"], d)
        if os.path.isfile(os.path.join(d, "adir.kicad_pro")):
            print("FAIL: 3-board cp should be skipped (ambiguous dst)"); fails += 1
        else:
            print("PASS: non 2-board cp is skipped")

    if fails:
        print(f"\n{fails} test(s) FAILED")
        return 1
    print("\nAll redo .kicad_pro-mirror tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
