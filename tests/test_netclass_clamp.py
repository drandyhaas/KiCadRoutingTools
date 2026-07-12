#!/usr/bin/env python3
"""Issue #295 addendum + --no-clamp-netclasses flag: after routing, the NON-Default
net classes (an impedance design's HDMI/USB/Zxx classes at e.g. 0.125mm) are, BY
DEFAULT, clamped down to the routed floor so KiCad's per-net-class DRC does not
storm copper legitimately routed at the smaller run clearance. The
``--no-clamp-netclasses`` flag disables that, for a final impedance-controlled
board whose class spec must survive. The Default class is always clamped either
way; the .kicad_pcb is never touched.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(ROOT, "fix_kicad_drc_settings.py")
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

    # DEFAULT behaviour (clamp on): both Default and the impedance class drop to
    # the routed floor.
    with tempfile.TemporaryDirectory() as tmp:
        cls = _run(tmp)
        if not near(cls["Default"]["clearance"], 0.09):
            fails.append(f"[clamp] Default.clearance = {cls['Default'].get('clearance')}, expected 0.09")
        if not near(cls["Z100_inner"]["clearance"], 0.09):
            fails.append(f"[clamp] Z100_inner.clearance = {cls['Z100_inner'].get('clearance')}, expected 0.09 (should clamp)")
        if not near(cls["Z100_inner"]["track_width"], 0.0889):
            fails.append(f"[clamp] Z100_inner.track_width = {cls['Z100_inner'].get('track_width')}, expected 0.0889")

    # --no-clamp-netclasses: Default still clamps, impedance class UNTOUCHED.
    with tempfile.TemporaryDirectory() as tmp:
        cls = _run(tmp, "--no-clamp-netclasses")
        if not near(cls["Default"]["clearance"], 0.09):
            fails.append(f"[no-clamp] Default.clearance = {cls['Default'].get('clearance')}, expected 0.09 (Default always clamps)")
        if not near(cls["Z100_inner"]["clearance"], 0.125):
            fails.append(f"[no-clamp] Z100_inner.clearance = {cls['Z100_inner'].get('clearance')}, expected 0.125 (must survive)")
        if not near(cls["Z100_inner"]["track_width"], 0.162):
            fails.append(f"[no-clamp] Z100_inner.track_width = {cls['Z100_inner'].get('track_width')}, expected 0.162 (must survive)")

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print("PASS: non-Default netclasses clamped to the routed floor by default; "
          "--no-clamp-netclasses preserves the impedance class spec; Default always clamped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
