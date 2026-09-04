#!/usr/bin/env python3
"""Issue #295 / #439: the standalone fix_kicad_drc_settings writeback clamps EVERY
net class's CLEARANCE (Default and the impedance design's HDMI/USB/Zxx classes)
DOWN to the routed floor so KiCad's per-net-class DRC does not storm copper
legitimately routed at the smaller run clearance. Since 6b971f1 the non-Default
clamp is restricted to _NONDEFAULT_CLAMP_FIELDS (clearance + the diff-pair
readback fields): per-class track_width/via_diameter/via_drill are declared
geometry (an impedance class's width IS its spec) and are preserved. #439
removed the old --no-clamp-netclasses flag: the standalone fixer always clamps
(the clearance clamp only ever lowers a class to the copper actually routed, so
it is always DRC-safe). To PRESERVE a class spec in full, route with route.py
and OMIT --clearance -- the router then honors each class and the writeback
keeps it. The .kicad_pcb is never touched.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(ROOT, 'py_router', 'fix_kicad_drc_settings.py')
BOARD = os.path.join(ROOT, "kicad_files", "qfn_underpad_coupling.kicad_pcb")


def _project():
    # Default at a stock-stricter 0.2, plus an impedance class at 0.125 / wide
    # track+via -- the shape a routed board inherits from an impedance design.
    return {
        "board": {"design_settings": {"rules": {}, "rule_severities": {}}},
        "net_settings": {"meta": {"version": 0}, "classes": [
            {"name": "Default", "clearance": 0.2, "track_width": 0.2,
             "via_diameter": 0.6, "via_drill": 0.3, "priority": 2147483647,
             "microvia_diameter": 0.3, "diff_pair_gap": 0.25, "wire_width": 6},
            {"name": "Z100_inner", "clearance": 0.125, "track_width": 0.162,
             "via_diameter": 0.6, "via_drill": 0.25},
        ]},
        "meta": {"version": 1},
    }


def _run(tmp, *extra):
    pcb = os.path.join(tmp, "board.kicad_pcb")
    pro = os.path.join(tmp, "board.kicad_pro")
    shutil.copyfile(BOARD, pcb)
    json.dump(_project(), open(pro, "w"), indent=2)
    r = subprocess.run(
        [sys.executable, SCRIPT, pcb, "--clearance", "0.09", "--track-width", "0.0889",
         "--via-size", "0.25", "--via-drill", "0.15", *extra],
        capture_output=True, text=True, cwd=ROOT)
    assert r.returncode == 0, r.stdout + r.stderr
    proj = json.load(open(pro))
    cls = {c["name"]: c for c in proj["net_settings"]["classes"]}
    return cls


def main():
    if not os.path.exists(BOARD):
        print(f"FAIL: test board missing ({BOARD})")
        return 1
    fails = []

    def near(a, b):
        return abs((a if a is not None else -1) - b) <= 1e-9

    # DEFAULT behaviour (clamp on): every class's CLEARANCE drops to the routed
    # floor (KiCad's one DRC-enforced per-class field). Since 6b971f1, a
    # non-Default class's track/via values are PRESERVED: they are declared
    # geometry, not DRC constraints -- clamping them destroyed impedance specs
    # (a QFN fanout rewrote USB_FS_DIFF's 0.8mm width to 0.15 on nets it never
    # routed). Z100_inner's 0.162 track_width IS its 100-ohm spec. The Default
    # class still clamps in full (it is the writeback's own floor record).
    with tempfile.TemporaryDirectory() as tmp:
        cls = _run(tmp)
        if not near(cls["Default"]["clearance"], 0.09):
            fails.append(f"[clamp] Default.clearance = {cls['Default'].get('clearance')}, expected 0.09")
        # #842: the Default class's track_width is a DRAW DEFAULT too and is
        # PRESERVED (it used to be lowered to the routed 0.0889, which the
        # next run then read back as "the board's own width").
        if not near(cls["Default"]["track_width"], 0.2):
            fails.append(f"[clamp] Default.track_width = {cls['Default'].get('track_width')}, expected 0.2 (draw default preserved, #842)")
        if not near(cls["Z100_inner"]["clearance"], 0.09):
            fails.append(f"[clamp] Z100_inner.clearance = {cls['Z100_inner'].get('clearance')}, expected 0.09 (should clamp)")
        # track_width is a DRAW DEFAULT, not a DRC floor: the non-Default clamp
        # deliberately PRESERVES it (fix_kicad_drc_settings doctrine -- lowering
        # it prevents no violation and overwrote a hard impedance spec figure,
        # USB_FS_DIFF 0.8 -> 0.15). Only clearance clamps.
        if not near(cls["Z100_inner"]["track_width"], 0.162):
            fails.append(f"[clamp] Z100_inner.track_width = {cls['Z100_inner'].get('track_width')}, expected 0.162 (draw default preserved)")

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print("PASS: the standalone writeback clamps every class's CLEARANCE to the "
          "routed floor and preserves non-Default declared geometry (6b971f1); "
          "--no-clamp-netclasses is gone (#439)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
