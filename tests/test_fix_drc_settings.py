#!/usr/bin/env python3
"""Issue #160: fix_kicad_drc_settings.py makes a routed board's KiCad DRC
constraints consistent with the routed floors -- lowering them toward the fab
floor (never raising), creating a Default net class if the project has none, and
ignoring non-routing severities -- while leaving the .kicad_pcb byte-for-byte
untouched (so a KiCad-9 board stays KiCad-9).
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(ROOT, "fix_kicad_drc_settings.py")
BOARD = os.path.join(ROOT, "kicad_files", "qfn_underpad_coupling.kicad_pcb")


def _md5(path):
    return hashlib.md5(open(path, "rb").read()).hexdigest()


def main():
    if not os.path.exists(BOARD):
        print(f"FAIL: test board missing ({BOARD})")
        return 1

    fails = []
    with tempfile.TemporaryDirectory() as tmp:
        pcb = os.path.join(tmp, "board.kicad_pcb")
        pro = os.path.join(tmp, "board.kicad_pro")
        shutil.copyfile(BOARD, pcb)
        # Stock-stricter project, plus one rule already LOOSER than the routed
        # floor (min_track_width 0.05) to prove we never tighten, and one
        # severity already at 'ignore' (courtyards_overlap) to prove we never
        # RAISE a severity back up.
        json.dump({
            "board": {"design_settings": {
                "rules": {"min_clearance": 0.2, "min_track_width": 0.05,
                          "min_via_diameter": 0.6, "min_hole_clearance": 0.25,
                          "min_through_hole_diameter": 0.3, "min_copper_edge_clearance": 0.5},
                "rule_severities": {"solder_mask_bridge": "error",
                                    "courtyards_overlap": "ignore"}}},
            "net_settings": {"classes": []},
            "meta": {"version": 1},
        }, open(pro, "w"), indent=2)

        md5_before = _md5(pcb)
        r = subprocess.run(
            [sys.executable, SCRIPT, pcb,
             "--clearance", "0.15", "--track-width", "0.15", "--via-size", "0.4",
             "--via-drill", "0.3", "--hole-to-hole", "0.2", "--edge-clearance", "0.0"],
            capture_output=True, text=True, cwd=ROOT)
        if r.returncode != 0:
            print("FAIL: script errored\n" + r.stdout + r.stderr)
            return 1

        proj = json.load(open(pro))
        rules = proj["board"]["design_settings"]["rules"]
        sev = proj["board"]["design_settings"]["rule_severities"]
        classes = proj["net_settings"]["classes"]
        default = next((c for c in classes if c.get("name") == "Default"), None)

        # Loosened toward the routed floor.
        expect_rules = {"min_clearance": 0.15, "min_via_diameter": 0.4,
                        "min_hole_clearance": 0.15, "min_through_hole_diameter": 0.3,
                        "min_hole_to_hole": 0.2, "min_copper_edge_clearance": 0.0}
        for k, v in expect_rules.items():
            if abs(rules.get(k, -1) - v) > 1e-9:
                fails.append(f"rules.{k} = {rules.get(k)}, expected {v}")
        # Never tightened: a rule already looser than the floor stays.
        if abs(rules.get("min_track_width", -1) - 0.05) > 1e-9:
            fails.append(f"min_track_width was raised to {rules.get('min_track_width')} "
                         f"(should stay 0.05 -- only loosen)")
        # Default net class created COMPLETE (a sparse stub is ignored by KiCad,
        # which then falls back to the stock 0.2 mm default -- issue #160 v9), and
        # set to the clearance floor; net_settings.meta present.
        if default is None:
            fails.append("Default net class was not created")
        else:
            if abs(default.get("clearance", -1) - 0.15) > 1e-9:
                fails.append(f"net_class[Default].clearance = {default.get('clearance')}, expected 0.15")
            for required in ("priority", "microvia_diameter", "diff_pair_gap", "wire_width"):
                if required not in default:
                    fails.append(f"created Default class missing '{required}' (KiCad won't honour a sparse class)")
        if "meta" not in proj["net_settings"]:
            fails.append("net_settings.meta missing (KiCad needs it to read classes)")
        # Non-routing severities ignored.
        for cat in ("solder_mask_bridge", "lib_footprint_mismatch", "courtyards_overlap",
                    "annular_width"):
            if sev.get(cat) != "ignore":
                fails.append(f"severity[{cat}] = {sev.get(cat)}, expected ignore")
        # Thermal-relief shortfall demoted error -> warning (still visible).
        if sev.get("starved_thermal") != "warning":
            fails.append(f"severity[starved_thermal] = {sev.get('starved_thermal')}, expected warning")
        # The board file must be byte-for-byte unchanged (version preserved).
        if _md5(pcb) != md5_before:
            fails.append("the .kicad_pcb was modified (must only edit the .kicad_pro)")

        # Idempotent: a second run reports nothing to change.
        r2 = subprocess.run([sys.executable, SCRIPT, pcb, "--clearance", "0.15",
                             "--track-width", "0.15", "--via-size", "0.4", "--via-drill", "0.3",
                             "--hole-to-hole", "0.2", "--edge-clearance", "0.0"],
                            capture_output=True, text=True, cwd=ROOT)
        if "already consistent" not in r2.stdout:
            fails.append("second run was not idempotent (expected 'already consistent')")

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print("PASS: constraints loosened to the routed floor (never tightened), Default "
          "net class created, non-routing severities ignored, .kicad_pcb untouched, idempotent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
