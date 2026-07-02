#!/usr/bin/env python3
"""Validate one candidate .kicad_pcb for the stress corpus.

Prints a one-line JSON verdict: loads in kicad_parser, has real routable content,
loads in pcbnew (so prep will work), plus a difficulty tier guess.

Usage: python3 validate_candidate.py <file.kicad_pcb>
PASS criteria: parser loads; 2<=copper_layers<=8; footprints>=10; routable_nets>=20.
"""
import json, sys, subprocess, os, re
REPO = "/Users/andy/Documents/KiCadRoutingTools"
sys.path.insert(0, REPO)
KPY = "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3"


def classify(layers, fps, routable, big_bga):
    if big_bga or layers >= 6 or fps >= 120 or routable >= 250:
        return "hard"
    if layers >= 4 or fps >= 45 or routable >= 80:
        return "medium"
    return "easy"


def main():
    f = sys.argv[1]
    v = {"file": os.path.basename(f), "pass": False}
    try:
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(f)
        layers = len(pcb.board_info.copper_layers)
        fps = len(pcb.footprints)
        routable = sum(1 for n in pcb.nets.values() if len(n.pads) >= 2 and n.net_id != 0)
        # biggest footprint pad count -> BGA/dense hint
        big = max((len(fp.pads) for fp in pcb.footprints.values()), default=0)
        v.update(copper_layers=layers, footprints=fps, routable_nets=routable, max_pads=big)
    except Exception as e:
        v["error"] = f"parser: {type(e).__name__}: {e}"[:160]
        print(json.dumps(v)); return
    # KiCad version token
    try:
        head = open(f, "r", errors="replace").read(400)
        m = re.search(r"\(version\s+(\d+)\)", head)
        v["kicad_version"] = int(m.group(1)) if m else None
    except Exception:
        v["kicad_version"] = None
    # pcbnew load (prep will use this)
    try:
        r = subprocess.run([KPY, "-c",
            f"import pcbnew,sys; b=pcbnew.LoadBoard(sys.argv[1]); print(len(b.GetFootprints()))", f],
            capture_output=True, text=True, timeout=90)
        v["pcbnew_load"] = (r.returncode == 0 and r.stdout.strip().isdigit())
        if not v["pcbnew_load"]:
            v["pcbnew_err"] = (r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "rc=%d" % r.returncode)[:120]
    except Exception as e:
        v["pcbnew_load"] = False
        v["pcbnew_err"] = str(e)[:120]
    v["tier"] = classify(v["copper_layers"], v["footprints"], v["routable_nets"], v["max_pads"] >= 200)
    v["pass"] = (2 <= v["copper_layers"] <= 8 and v["footprints"] >= 10
                 and v["routable_nets"] >= 20 and v["pcbnew_load"])
    print(json.dumps(v))


if __name__ == "__main__":
    main()
