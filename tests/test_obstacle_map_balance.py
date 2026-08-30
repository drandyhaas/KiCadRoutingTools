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
                    +3.3V/GND planes): the per-net via-placement map audit
                    vs a fresh rebuild. NO churn since #562 -- the pour step
                    defers every via-needed pad to the route step, so it
                    cannot rip; stage 4 carries the churn coverage.
  4. repair_planes - interf_u chain (planes -> U9 fanout ->
                    signal route -> repair): the repair pass runs under
                    TAP_MAP_VERIFY=1, whose built-in asserts compare the
                    shared via maps against fresh builds on every rip and
                    per-tap window.

If a stage's churn assertion starts failing after an engine change, the
recipe needs re-tuning (more congestion), NOT deletion - without churn the
balance check is vacuous. The ONE exception is a front-end that can no longer
churn by construction (route_planes since #562): there, re-tuning is
impossible and the churn coverage must MOVE to a front-end that still rips,
never be quietly dropped.

    python3 tests/test_obstacle_map_balance.py
"""
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
#: #716: matches the 1200 s this test already puts on its own subprocess
#: (below). Under run_all's 600 s default it was killed before reaching its
#: final checks, so a REAL failing check ("repairs actually performed")
#: presented as a timeout and was dismissed as slowness. Measured in #691: it
#: passes ALONE in 681 s with all 18 checks green, so this is a budget for the
#: contended case, not cover for a hung test.
#:
#: Raised to 2400 s because 1200 was still not a contended-case budget on a
#: slower box. Measured here: ALONE 994 s, all checks green -- 1.46x the #691
#: figure -- while the same test TIMED OUT inside a full `run_all.py` (which
#: runs 4 at a time) on that same machine. A budget with 1.2x headroom over
#: the solo time is not a budget for contention, and the failure mode it
#: produces is the one this comment already warns about: a real result
#: presented as slowness. The subprocess timeout below is raised to match, or
#: the inner kill would just move the same problem one level down.
RUN_ALL_TIMEOUT = 2400

ROOT_DIR = os.path.dirname(TESTS_DIR)
KF = os.path.join(ROOT_DIR, "kicad_files")

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  [{detail}]" if detail and not cond else ""))
    if not cond:
        FAILS.append(name)


def run_cmd(args, audit=True, ledger=False, tap_verify=False,
            fragility_off=False):
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
    # #522 reorg: the CLI scripts moved from the repo root into py_router/
    # (py_tools/ for the utilities). Resolve the bare script name the
    # recipes use against its current home so the test survives moves.
    for sub in ("py_router", "py_tools"):
        cand = os.path.join(ROOT_DIR, sub, args[0])
        if os.path.exists(cand):
            args = [cand] + list(args[1:])
            break
    if fragility_off:
        # The stage-1 churn recipe's last routing failure is cost-surface
        # sensitive: with the #424 plane-fragility field priced from the
        # EXACT KiCad fill (which depends on whether a KiCad python is
        # discoverable on the machine), the failing MST edge routes and the
        # recipe stops churning -- making the balance checks vacuous and the
        # test environment-dependent. Pin the field off for that stage; the
        # audit under churn is what this test exists to exercise.
        env["KICAD_PLANE_FRAGILITY_COST"] = "0"
    # Kept equal to RUN_ALL_TIMEOUT above: an inner kill lower than the outer
    # budget just relocates the same "a real result presented as slowness"
    # failure one level down, where it reads as a router hang instead.
    p = subprocess.run([sys.executable, "-X", "utf8"] + args, cwd=ROOT_DIR,
                       env=env, capture_output=True, text=True, timeout=2400)
    return p.returncode, p.stdout + p.stderr


def main():
    tmp = tempfile.mkdtemp(prefix="map_balance_")

    # ---- 1. route.py: signal churn ------------------------------------------
    out1 = os.path.join(tmp, "route_churn.kicad_pcb")
    rc, log = run_cmd(["py_router/route.py", os.path.join(KF, "flat_hierarchy.kicad_pcb"), out1,
                       "--nets", "*", "!GND",
                       "--track-width", "0.8", "--clearance", "0.7",
                       "--via-size", "1.0", "--via-drill", "0.6",
                       # 2026-08-05 re-tune (per this header: more congestion,
                       # never delete): the two-layer recipe stopped churning
                       # once the engine gained pre-existing rip candidacy +
                       # the plain-net via rung -- it now routes clean at ANY
                       # fatness on two layers. Single-layer restores real
                       # churn: 64 rips / 13 tap windows, maps still BALANCED.
                       "--layers", "F.Cu", "--max-ripup", "10"],
                      ledger=True, fragility_off=True)
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
    rc, log = run_cmd(["py_router/route_diff.py", os.path.join(KF, "lvds_converter_dualclk.kicad_pcb"),
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
    # Pre-route a WIDE slice of U102's signal nets, not just /AN*. Per this
    # file's own header ("re-tune the recipe, NOT delete the assertion"): with
    # only /AN* routed, the plane step still hit blocked pads but every rip-up
    # attempt failed with "blocked by +3.3V (protected, cannot rip)" -- the
    # blockers were plane copper, which is never rippable, so zero rips ever
    # happened and the balance assertion below went vacuous. These nets put
    # RIPPABLE signal copper around U102's plane pads; the stage now logs 3
    # real rips out of 36 rip-up attempts.
    rc, log = run_cmd(["py_router/route.py", os.path.join(KF, "kit-dev-coldfire-xilinx_5213.kicad_pcb"),
                       kit_out, "--nets", "/AN*", "/IRQ*", "/QSPI*", "/DTIN*",
                       "/PWM*", "/GPT*", "/PST*", "/DDAT*",
                       "--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
                       "--max-ripup", "10"] + kit_geom, audit=False)
    check("route_planes: kit signal pre-route completed", rc == 0, f"rc={rc}")
    kit_plane = os.path.join(tmp, "kit_plane.kicad_pcb")
    rc, log = run_cmd(["py_router/route_planes.py", kit_out, kit_plane,
                       "--nets", "+3.3V", "GND", "+3.3V", "GND",
                       "--plane-layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
                      + kit_geom)
    check("route_planes: run completed", rc == 0, f"rc={rc}")
    # NO rip-churn assertion here any more, and re-tuning cannot bring one
    # back: since #562 the pour step neither taps nor rips (every via-needed
    # pad is deferred to the route step), so route_planes can no longer churn
    # BY CONSTRUCTION. The churn half of this stage now lives in stage 4,
    # which exercises the same via maps in the engine that still rips. What
    # remains here is still worth asserting: the maps the pour DOES build
    # (thermal-via arrays) must balance against a fresh rebuild.
    audits = re.findall(r"OBSTACLE AUDIT route_planes:\S+\] via map (\w+)", log)
    check("route_planes: every via map BALANCED vs fresh rebuild",
          all(a == "BALANCED" for a in audits) and "DIVERGED" not in log,
          f"audits={audits}")

    # ---- 4. repair_planes under TAP_MAP_VERIFY -------------------
    iu_plane = os.path.join(tmp, "iu_plane.kicad_pcb")
    iu_fan = os.path.join(tmp, "iu_fanout.kicad_pcb")
    iu_routed = os.path.join(tmp, "iu_routed.kicad_pcb")
    iu_fixed = os.path.join(tmp, "iu_repair.kicad_pcb")
    rc, _ = run_cmd(["py_router/route_planes.py", os.path.join(KF, "interf_u_unrouted.kicad_pcb"),
                     iu_plane, "--nets", "VCC", "GND",
                     "--plane-layers", "F.Cu", "B.Cu"], audit=False)
    check("repair: interf_u planes step completed", rc == 0, f"rc={rc}")
    rc, _ = run_cmd(["py_router/bga_fanout.py", iu_plane, "--component", "U9",
                     "--output", iu_fan, "--nets", "/*"], audit=False)
    check("repair: interf_u fanout step completed", rc == 0, f"rc={rc}")
    # Plane nets are EXCLUDED from the route step's --nets so its in-run
    # finalize (#562) skips the pours BY PLAN and the signal step's pour
    # damage survives for repair_planes. With the plane nets in scope the
    # recipe only churned while the finalize happened to MISS some damage --
    # a band the 0808-09 engine wave closed (total_routes went to 0 and this
    # stage's TAP_MAP_VERIFY coverage silently went vacuous).
    rc, _ = run_cmd(["py_router/route.py", iu_fan, iu_routed, "--no-bga-zone",
                     "--nets", "*", "!VCC", "!GND",
                     "--layer-costs", "1", "1", "--max-ripup", "10",
                     "--stub-proximity-radius", "10", "--stub-proximity-cost", "3.0",
                     "--max-iterations", "1000000",
                     "--board-edge-clearance", "0.55"], audit=False)
    check("repair: interf_u signal route completed", rc == 0, f"rc={rc}")
    rc, log = run_cmd(["py_router/repair_planes.py", iu_routed, iu_fixed,
                       "--board-edge-clearance", "0.6"], audit=False, tap_verify=True)
    check("repair: run completed under TAP_MAP_VERIFY", rc == 0, f"rc={rc}")
    check("repair: no shared-map divergence",
          "TAP_MAP_VERIFY" not in log or "divergence" not in log)
    # The vacuity guard for this stage: TAP_MAP_VERIFY's asserts run SILENTLY
    # (plane_pad_tap._TAP_MAP_VERIFY, read at import; it speaks only on
    # divergence), so an empty log is indistinguishable from a stage that did
    # nothing. Something must therefore prove work happened.
    #
    # #716: this asserted `total_routes >= 1` alone and went red at
    # total_routes=0 -- read as "the repair step stopped performing repairs".
    # It had not. Measured on this exact recipe (2026-08-22): repair_planes
    # reports `Pad repair: 2 pad(s) with no connection to the VCC plane` and
    # `Pad repair: 2/2 unconnected pad(s) reconnected`, with
    # `total_regions: 0, total_routes: 0`. The pour damage now takes the form
    # of STRANDED ISLANDS ("1 anchored region plus 2 stranded island(s)
    # (60.2 mm^2) with no pad, via or track on them") which repair_planes
    # correctly does not route -- there is nothing on them to reconnect --
    # while the real work went down the PAD-REPAIR path.
    #
    # Both paths place taps, which is what exercises the shared via maps this
    # stage exists to verify, so the guard accepts either. Do NOT narrow this
    # back to total_routes without re-tuning the recipe to produce genuinely
    # routable regions: a guard that cannot see the work that happened reports
    # a healthy engine as broken, which is exactly what #716 caught.
    m = re.search(r'"total_routes": (\d+)', log)
    routes = int(m.group(1)) if m else 0
    mp = re.search(r'Pad repair: (\d+)/(\d+) unconnected pad\(s\) reconnected', log)
    pads = int(mp.group(1)) if mp else 0
    check("repair: repairs actually performed (recipe still has work)",
          routes >= 1 or pads >= 1,
          f"total_routes={routes}, pads_reconnected={pads} -- neither path did "
          f"work, so the TAP_MAP_VERIFY coverage above is vacuous; re-tune the "
          f"recipe for more congestion rather than relaxing this check")

    print()
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s): {FAILS}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
