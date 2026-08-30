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
SCRIPT = os.path.join(ROOT, 'py_router', 'fix_kicad_drc_settings.py')
BOARD = os.path.join(ROOT, "kicad_files", "qfn_underpad_coupling.kicad_pcb")


def _md5(path):
    return hashlib.md5(open(path, "rb").read()).hexdigest()



def _check_in_run_floor_sync():
    """#650: the in-run audit must grade at the floors the board will SHIP with.

    ``apply_routed_floors`` lowers the sibling project's COPPER floors mid-run,
    before the plane finalize's oracle grades the board -- until it existed, that
    sibling was still the INPUT's (seeded by ``seed_project_for_output``) and its
    looser declared floors made the zone filler pull the pours back further than
    the shipped board's would, so the audit over-reported opens. Measured on
    orangecrab: 57 reported vs the 46 that ship (GND 19 vs 11), all of it
    ``rules.min_hole_clearance`` 0.25 -> 0.0889.
    """
    sys.path.insert(0, os.path.join(ROOT, "py_router"))
    from fix_kicad_drc_settings import apply_routed_floors
    fails = []
    with tempfile.TemporaryDirectory() as td:
        pcb = os.path.join(td, "b.kicad_pcb")
        pro = os.path.join(td, "b.kicad_pro")
        shutil.copyfile(BOARD, pcb)

        # No sibling project -> no-op, and none is created (a project-less
        # board must stay project-less; #441 warns about it, we must not
        # silently invent one).
        if apply_routed_floors(pcb, clearance=0.1) != []:
            fails.append("#650: reported changes with no sibling .kicad_pro")
        if os.path.exists(pro):
            fails.append("#650: created a .kicad_pro where the board had none")

        # A board declaring LOOSER floors than the run routed to: hole/copper
        # comes down to the routed clearance (the measured fill driver), and so
        # does the Default class.
        json.dump({"board": {"design_settings": {
                      "rules": {"min_hole_clearance": 0.25, "min_clearance": 0.2}}},
                   "net_settings": {"classes": [
                       {"name": "Default", "clearance": 0.2},
                       {"name": "HS", "clearance": 0.3}]}},
                  open(pro, "w"))
        changes = apply_routed_floors(pcb, clearance=0.0889)
        rules = json.load(open(pro))["board"]["design_settings"]["rules"]
        if abs(rules.get("min_hole_clearance", 9) - 0.0889) > 1e-9:
            fails.append(f"#650: min_hole_clearance = {rules.get('min_hole_clearance')}, "
                         f"expected 0.0889")
        if abs(rules.get("min_clearance", 9) - 0.0889) > 1e-9:
            fails.append(f"#650: min_clearance = {rules.get('min_clearance')}, expected 0.0889")
        classes = {c["name"]: c for c in json.load(open(pro))["net_settings"]["classes"]}
        if abs(classes["Default"].get("clearance", 9) - 0.0889) > 1e-9:
            fails.append("#650: the Default class was not lowered to the routed clearance")
        # Non-Default classes are the WRITEBACK's call (#439 -- it depends on the
        # caller having passed a --clearance ceiling, a main() fact). Clamping
        # them here could ship a tightened class on a run that meant to honor them.
        if abs(classes["HS"].get("clearance", 0) - 0.3) > 1e-9:
            fails.append("#650: a non-Default class was clamped by default")
        if not changes:
            fails.append("#650: reported no changes on a project that needed lowering")

        # Idempotent -- it runs before every in-run grade.
        if apply_routed_floors(pcb, clearance=0.0889) != []:
            fails.append("#650: not idempotent (second call reported changes)")

        # ONLY-loosen: a board already declaring a TIGHTER floor keeps it, so
        # this can never manufacture a violation on correct copper.
        json.dump({"board": {"design_settings": {
                      "rules": {"min_hole_clearance": 0.05}}}}, open(pro, "w"))
        apply_routed_floors(pcb, clearance=0.0889)
        rules = json.load(open(pro))["board"]["design_settings"]["rules"]
        if abs(rules.get("min_hole_clearance", 9) - 0.05) > 1e-9:
            fails.append(f"#650: RAISED a tighter declared floor to "
                         f"{rules.get('min_hole_clearance')} (must only loosen)")
    return fails


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

        # Loosened toward the routed floor. Edge (#338): --edge-clearance 0.0
        # means "not enforced this step", NOT "lower the rule to zero" -- the
        # board's own min_copper_edge_clearance must SURVIVE (KiCad grades
        # copper_edge_clearance from it and the routers now route to it).
        expect_rules = {"min_clearance": 0.15, "min_via_diameter": 0.4,
                        "min_hole_clearance": 0.15, "min_through_hole_diameter": 0.3,
                        "min_hole_to_hole": 0.2, "min_copper_edge_clearance": 0.5}
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

    fails += _check_in_run_floor_sync()

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print("PASS: constraints loosened to the routed floor (never tightened), Default "
          "net class created, non-routing severities ignored, .kicad_pcb untouched, idempotent; "
          "in-run floor sync (#650) lowers hole/copper to the routed floor, only-loosens, "
          "is idempotent, and no-ops without a sibling project")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
