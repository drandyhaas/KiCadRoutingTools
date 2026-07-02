#!/usr/bin/env python3
"""Issue #195: QFN under-pad diff-pair escape must reach a connector 2/2, fully
connected and DRC-clean.

Reporter's synthetic repro (kicad_files/qfn_csi_underpad_diff.kicad_pcb): a
QFN-48 0.5mm (U1) with two CSI diff pairs on its right edge and a 1x04 pin
header (J1) ~12mm away, on a 4-layer board. The board outline is sized so the
connector is fully in bounds (the reporter's original board had the lower pins
hanging over the edge -- a separate bug; here we test the routable scenario).

This guards FOUR fixes that all have to hold for the chain to succeed:
  1. qfn_fanout under-pad escape draws the pad->via stub on the pad's own copper
     layer (F.Cu), so the through-via actually ties the QFN pad to the In1 net
     -- otherwise the pads float and the final board is disconnected even though
     route_diff "routes".
  2. route_diff launches the coupled pair on an inner layer reachable through the
     escape via when the stub layer's corridor is jammed (the QFN pad field on
     F.Cu) -- otherwise the CLK pair can only launch rotated sideways and fails,
     giving 1/2.
  3. check_connected credits a track ending inside the large J1 through-hole pad
     even when it lands off-centre -- otherwise the landing reads as a false
     disconnect.
  4. (implicitly) the whole pipeline writes valid boards through each step.

The board ships checked in (built once with KiCad's pcbnew, QFN-48 + 1x04
header), so this test needs no pcbnew. Commands mirror the reporter's exactly.

Run:
    python3 tests/test_qfn_csi_underpad_diff.py [-v]
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT_DIR, "kicad_files", "qfn_csi_underpad_diff.kicad_pcb")
CLEARANCE = "0.1"  # route_diff default; grade DRC at the routed clearance


def _run(argv, verbose):
    r = subprocess.run([sys.executable, *argv], cwd=ROOT_DIR,
                       capture_output=True, text=True)
    txt = r.stdout + r.stderr
    if verbose:
        print(txt)
    return r.returncode, txt


def _json_summary(txt):
    m = re.search(r"JSON_SUMMARY:\s*(\{.*\})", txt)
    return json.loads(m.group(1)) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(BOARD):
        print(f"FAIL: board not found: {BOARD}")
        return 1

    fails = []
    d = tempfile.mkdtemp(prefix="qfn_csi_diff_")
    fanned = os.path.join(d, "fanned.kicad_pcb")
    routed = os.path.join(d, "routed.kicad_pcb")
    try:
        # 1) Under-pad escape the QFN's CSI pairs to the inner layer.
        rc, txt = _run(["qfn_fanout.py", BOARD, "-o", fanned,
                        "--component", "U1", "--nets", "CSI_*",
                        "--escape-method", "underpad", "--layer", "In1.Cu",
                        "--width", "0.127", "--via-size", "0.5",
                        "--via-drill", "0.2", "--allow-via-in-pad"], args.verbose)
        if rc != 0:
            fails.append(f"qfn_fanout exited {rc}")
        fan = _json_summary(txt)
        if not fan:
            fails.append("qfn_fanout printed no JSON_SUMMARY")
        elif fan.get("escaped") != 4 or fan.get("failed") != 0:
            fails.append(f"qfn_fanout escaped {fan.get('escaped')}/failed "
                         f"{fan.get('failed')}, expected 4/0")

        # 2) Coupled-route both pairs from the In1 stubs to J1.
        rc, txt = _run(["route_diff.py", fanned, routed,
                        "--nets", "CSI_D0*", "CSI_CLK*",
                        "--layers", "F.Cu", "In1.Cu",
                        "--track-width", "0.127", "--diff-pair-gap", "0.18",
                        "--diff-pair-intra-match"], args.verbose)
        if rc != 0:
            fails.append(f"route_diff exited {rc}")
        rd = _json_summary(txt)
        if not rd:
            fails.append("route_diff printed no JSON_SUMMARY")
        elif rd.get("successful") != 2 or rd.get("failed") != 0:
            fails.append(f"route_diff routed {rd.get('successful')}/2 pairs "
                         f"(failed {rd.get('failed')}), expected 2/0 "
                         f"[failed: {rd.get('failed_diff_pairs')}]")
        # Informational: this scenario only routes because the pair launches on
        # In1 through the escape via. Not a hard assert -- a future engine that
        # routes 2/2 another way is fine -- but flag if it silently stops firing.
        if rd and rd.get("successful") == 2 and "reachable through the endpoint via" not in txt:
            print("NOTE: routed 2/2 without the via-reachable multilayer launch "
                  "firing (escape geometry changed?) -- still a pass.")

        # 3) Fully connected (guards the floating-pad and in-pad-landing fixes).
        rc, txt = _run(["check_connected.py", routed], args.verbose)
        if "ALL NETS FULLY CONNECTED" not in txt:
            detail = "\n".join(
                f"      {ln}" for ln in txt.strip().splitlines()
                if re.search(r"net \d|Disconnected|Segments:|\(net", ln))[:2000]
            fails.append("check_connected did NOT report all nets connected "
                         "(floating QFN pad or false J1 disconnect?)\n" + detail)

        # 4) DRC-clean at the routed clearance.
        rc, txt = _run(["check_drc.py", routed, "-c", CLEARANCE], args.verbose)
        if "NO DRC VIOLATIONS FOUND" not in txt:
            m = re.search(r"FOUND (\d+) DRC VIOLATIONS", txt)
            detail = "\n".join(f"      {ln}" for ln in txt.splitlines()
                               if "violations (" in ln)
            fails.append(f"check_drc found {m.group(1) if m else '?'} violation(s) "
                         f"at clearance {CLEARANCE}\n" + detail)
    finally:
        for f in (fanned, routed):
            try:
                os.remove(f)
            except OSError:
                pass
        try:
            os.rmdir(d)
        except OSError:
            pass

    if fails:
        print("FAIL:")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("PASS: QFN CSI under-pad diff pairs route 2/2, fully connected, DRC-clean (#195)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
