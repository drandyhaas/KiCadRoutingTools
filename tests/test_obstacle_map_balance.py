#!/usr/bin/env python3
"""Obstacle-map ref-count balance on churn boards, for all four front-ends
(issues #208 / #309).

Every front-end that maintains an incrementally-updated obstacle map must end
its run with the map exactly equal to a fresh rebuild - any residual is a
ref-count leak (over-blocking that silently costs routability) or an
over-decrement (under-blocking that ships DRC violations). The #309 leaks
(phase-3 tap-via removes missing diagonal_margin; in-progress-via rings on the
persistent working map) only appear under RIP/REROUTE CHURN, so each stage
here is pinned to a recipe measured to actually churn:

  1. route.py     - flat_hierarchy at deliberately fat geometry: forces
                    rip-ups and a phase-3 temporary tap-obstacle window.
                    This exact recipe showed the pre-fix #309 leak
                    (+616 blocked_cells at 5436c08). KICAD_OBSTACLE_AUDIT
                    asserts working == base + sum(caches); the
                    KICAD_OBSTACLE_LEDGER cross-checks per cache object.
  2. route_diff   - lvds_converter_dualclk CLK/DATA pairs, same audit on
                    route_diff's own working map.
  3. route_planes - kit-dev quick chain (/AN* signals, then 4-layer
                    +3.3V/GND planes with --rip-blocker-nets): the per-net
                    via-placement map audit vs a fresh rebuild, with real
                    rip churn.
  4. route_disconnected_planes - interf_u chain (planes -> U9 fanout ->
                    signal route -> repair): the repair pass runs under
                    TAP_MAP_VERIFY=1, whose built-in asserts compare the
                    shared via maps against fresh builds on every rip and
                    per-tap window.

If a stage's churn assertion starts failing after an engine change, the
recipe needs re-tuning (more congestion), NOT deletion - without churn the
balance check is vacuous.

    python3 tests/test_obstacle_map_balance.py
"""
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
KF = os.path.join(ROOT_DIR, "kicad_files")

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  [{detail}]" if detail and not cond else ""))
    if not cond:
        FAILS.append(name)


def run_cmd(args, audit=True, ledger=False, tap_verify=False):
    env = dict(os.environ)
    env.pop("KICAD_OBSTACLE_AUDIT", None)
    env.pop("KICAD_OBSTACLE_LEDGER", None)
    env.pop("TAP_MAP_VERIFY", None)
    if audit:
        env["KICAD_OBSTACLE_AUDIT"] = "1"
    if ledger:
        env["KICAD_OBSTACLE_LEDGER"] = "1"
    if tap_verify:
        env["TAP_MAP_VERIFY"] = "1"
    p = subprocess.run([sys.executable, "-X", "utf8"] + args, cwd=ROOT_DIR,
                       env=env, capture_output=True, text=True, timeout=1200)
    return p.returncode, p.stdout + p.stderr


def main():
    tmp = tempfile.mkdtemp(prefix="map_balance_")

    # ---- 1. route.py: signal churn ------------------------------------------
    out1 = os.path.join(tmp, "route_churn.kicad_pcb")
    rc, log = run_cmd(["route.py", os.path.join(KF, "flat_hierarchy.kicad_pcb"), out1,
                       "--nets", "*", "!GND",
                       "--track-width", "0.8", "--clearance", "0.7",
                       "--via-size", "1.0", "--via-drill", "0.6",
                       "--layers", "F.Cu", "B.Cu", "--max-ripup", "10"],
                      ledger=True)
    check("route.py: run completed", rc == 0, f"rc={rc}")
    rips = len(re.findall(r"[Rr]ipping|Extending to N", log))
    check("route.py: rip churn engaged (recipe still churns)", rips >= 1, f"rips={rips}")
    temp_windows = len(re.findall(r"tap segments.*to obstacles", log))
    check("route.py: phase-3 temp tap-obstacle window exercised",
          temp_windows >= 1, f"windows={temp_windows}")
    check("route.py: working map BALANCED",
          "BALANCED: ref-counted maps" in log and "LEAK/DESYNC" not in log)
    check("route.py: cache-object ledger balanced",
          "cache-object ledger BALANCED" in log and "UNBALANCED serial" not in log)

    # ---- 2. route_diff -------------------------------------------------------
    out2 = os.path.join(tmp, "diff_bal.kicad_pcb")
    rc, log = run_cmd(["route_diff.py", os.path.join(KF, "lvds_converter_dualclk.kicad_pcb"),
                       out2, "--nets", "/CLK+", "/CLK-", "/DATA+", "/DATA-",
                       "/CLK_P", "/CLK_N",
                       "--diff-pair-gap", "0.25", "--track-width", "0.2",
                       "--clearance", "0.2", "--max-ripup", "10"],
                      ledger=True)
    check("route_diff: run completed", rc == 0, f"rc={rc}")
    check("route_diff: pairs routed", "Routing diff pair" in log)
    check("route_diff: working map BALANCED",
          "BALANCED: ref-counted maps" in log and "LEAK/DESYNC" not in log)
    check("route_diff: cache-object ledger balanced",
          "cache-object ledger BALANCED" in log and "UNBALANCED serial" not in log)

    # ---- 3. route_planes: rip-blocker churn ----------------------------------
    kit_out = os.path.join(tmp, "kit_out.kicad_pcb")
    kit_geom = ["--track-width", "0.2", "--clearance", "0.2", "--via-size", "0.5",
                "--via-drill", "0.4", "--hole-to-hole-clearance", "0.3"]
    rc, log = run_cmd(["route.py", os.path.join(KF, "kit-dev-coldfire-xilinx_5213.kicad_pcb"),
                       kit_out, "--nets", "/AN*",
                       "--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
                       "--max-ripup", "10"] + kit_geom, audit=False)
    check("route_planes: kit signal pre-route completed", rc == 0, f"rc={rc}")
    kit_plane = os.path.join(tmp, "kit_plane.kicad_pcb")
    rc, log = run_cmd(["route_planes.py", kit_out, kit_plane,
                       "--nets", "+3.3V", "GND", "+3.3V", "GND",
                       "--plane-layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
                       "--max-via-reuse-radius", "3", "--rip-blocker-nets"] + kit_geom)
    check("route_planes: run completed", rc == 0, f"rc={rc}")
    rips = len(re.findall(r"[Rr]ipping", log))
    check("route_planes: rip churn engaged (recipe still churns)", rips >= 1, f"rips={rips}")
    audits = re.findall(r"OBSTACLE AUDIT route_planes:\S+\] via map (\w+)", log)
    # 3, not 4: the second GND pass finds all its pads already processed by the
    # earlier layer pass (pads_need_via == 0), so no via map is built there.
    check("route_planes: per-net via-map audits ran", len(audits) >= 3, f"audits={len(audits)}")
    check("route_planes: every via map BALANCED vs fresh rebuild",
          bool(audits) and all(a == "BALANCED" for a in audits)
          and "DIVERGED" not in log)

    # ---- 4. route_disconnected_planes under TAP_MAP_VERIFY -------------------
    iu_plane = os.path.join(tmp, "iu_plane.kicad_pcb")
    iu_fan = os.path.join(tmp, "iu_fanout.kicad_pcb")
    iu_routed = os.path.join(tmp, "iu_routed.kicad_pcb")
    iu_fixed = os.path.join(tmp, "iu_repair.kicad_pcb")
    rc, _ = run_cmd(["route_planes.py", os.path.join(KF, "interf_u_unrouted.kicad_pcb"),
                     iu_plane, "--nets", "VCC", "GND",
                     "--plane-layers", "F.Cu", "B.Cu"], audit=False)
    check("repair: interf_u planes step completed", rc == 0, f"rc={rc}")
    rc, _ = run_cmd(["bga_fanout.py", iu_plane, "--component", "U9",
                     "--output", iu_fan, "--nets", "/*"], audit=False)
    check("repair: interf_u fanout step completed", rc == 0, f"rc={rc}")
    rc, _ = run_cmd(["route.py", iu_fan, iu_routed, "--no-bga-zone",
                     "--layer-costs", "1", "1", "--max-ripup", "10",
                     "--stub-proximity-radius", "10", "--stub-proximity-cost", "3.0",
                     "--max-iterations", "1000000",
                     "--board-edge-clearance", "0.55"], audit=False)
    check("repair: interf_u signal route completed", rc == 0, f"rc={rc}")
    rc, log = run_cmd(["route_disconnected_planes.py", iu_routed, iu_fixed,
                       "--board-edge-clearance", "0.6"], audit=False, tap_verify=True)
    check("repair: run completed under TAP_MAP_VERIFY", rc == 0, f"rc={rc}")
    check("repair: no shared-map divergence",
          "TAP_MAP_VERIFY" not in log or "divergence" not in log)
    m = re.search(r'"total_routes": (\d+)', log)
    check("repair: repairs actually performed (recipe still has work)",
          m is not None and int(m.group(1)) >= 1,
          f"total_routes={m.group(1) if m else 'none'}")

    print()
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s): {FAILS}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
