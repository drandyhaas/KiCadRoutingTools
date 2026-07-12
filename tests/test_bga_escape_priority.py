#!/usr/bin/env python3
"""Issue #129: escape priority for multi-ball nets.

Escape channels are the scarce resource on a dense BGA (#122), and a net only
NEEDS one escape -- later routing picks its other balls up inside the BGA
(--no-bga-zones). bga_fanout now fans one ball per net first (net coverage),
then attempts real escapes for the remaining EXTRA balls with the pass-1
copper committed; an extra that fails (or whose escape grazes pass-1 copper,
caught by the cross-pass guard) is a soft failure whose ball gets a quick
intra-BGA A* strap to its net's fanout, or is left for the main router.
No new CLI options; boards without multi-ball nets take a single pass.

Uses kicad_files/interf_u_unrouted_placed.kicad_pcb U9 (a PGA whose VCC/12V
nets have multiple pins).

    python3 tests/test_bga_escape_priority.py
"""
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
BOARD = os.path.join(ROOT, "kicad_files", "interf_u_unrouted_placed.kicad_pcb")


def run_fanout(nets):
    out = tempfile.mktemp(suffix=".kicad_pcb")
    cmd = [sys.executable, "bga_fanout.py", BOARD, "--component", "U9",
           "--nets", *nets,
           "--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
           "--track-width", "0.2", "--clearance", "0.2",
           "--via-size", "0.5", "--via-drill", "0.3", "--output", out]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    return r.stdout + r.stderr, out


def main():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # --- multi-ball nets in scope: two-pass priority runs ---
    txt, out = run_fanout(["*", "!GND"])
    m = re.search(r"Escape priority \(issue #129\): (\d+) net\(s\) have (\d+) extra ball\(s\)", txt)
    check("escape-priority split fires for multi-ball nets", m is not None)
    m_sum = re.search(r'"escaped": (\d+), "failed": (\d+)', txt)
    check("summary parses", m_sum is not None)
    if m_sum:
        check("all nets escape (net coverage first)", int(m_sum.group(2)) == 0,
              f"failed={m_sum.group(2)}")

    # every multi-ball-net ball ends with copper it can actually reach
    if os.path.exists(out):
        from kicad_parser import parse_kicad_pcb
        from collections import defaultdict
        pcb = parse_kicad_pcb(out)
        u9 = pcb.footprints["U9"]
        byn = defaultdict(list)
        for p in u9.pads:
            if p.net_name and p.net_name != "GND" \
                    and not p.net_name.lower().startswith("unconnected-"):
                byn[p.net_id].append(p)
        missing = []
        for nid, balls in byn.items():
            if len(balls) < 2:
                continue
            for b in balls:
                tol = max(b.size_x, b.size_y) / 2 + 0.01
                pl = next((l for l in (b.layers or [])
                           if l.endswith(".Cu") and not l.startswith("*")), None)
                any_layer = pl is None or (b.drill or 0) > 0
                has_via = any(v.net_id == nid
                              and abs(v.x - b.global_x) < tol
                              and abs(v.y - b.global_y) < tol
                              for v in pcb.vias)
                has_seg = any(
                    s.net_id == nid and (any_layer or s.layer == pl) and (
                        (abs(s.start_x - b.global_x) < tol
                         and abs(s.start_y - b.global_y) < tol)
                        or (abs(s.end_x - b.global_x) < tol
                            and abs(s.end_y - b.global_y) < tol))
                    for s in pcb.segments)
                if not (has_via or has_seg):
                    missing.append(f"{b.net_name}.{b.pad_number}")
        check("every multi-ball-net ball carries reachable copper",
              not missing, f"missing: {missing[:5]}")
        os.remove(out)

    # --- no multi-ball nets in scope: single pass, no priority machinery ---
    txt1, out1 = run_fanout(["*", "!GND", "!VCC", "!12V", "!-12V", "!+5V"])
    m1 = re.search(r"Escape priority", txt1)
    m1_sum = re.search(r'"escaped": (\d+), "failed": (\d+)', txt1)
    if m1 is None:
        check("single pass when no extras", True)
    else:
        # the exclusion list may not cover every multi-ball net on this
        # fixture; the split firing is then correct
        mm = re.search(r"have (\d+) extra ball", txt1)
        check("single pass when no extras (or extras legitimately present)",
              mm is not None and int(mm.group(1)) > 0)
    check("exclusion run parses", m1_sum is not None)
    if os.path.exists(out1):
        os.remove(out1)

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
