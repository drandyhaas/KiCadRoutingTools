#!/usr/bin/env python3
"""Validate one candidate .kicad_pcb for the stress corpus.

Metrics come from **pcbnew** (KiCad's C++ loader), not the pure-Python parser:
pcbnew loads even a 62MB / 800-footprint / 8-layer board in ~2-3s (the Python
parser needs 30-100s+ on the same file, and hung unbounded on the biggest), and
"it loads in pcbnew" is exactly what prep needs anyway. Prints a one-line JSON
verdict + a difficulty tier.

NB: a full DRC run is the WRONG check here -- an UNROUTED source board would report
thousands of unconnected-net "violations". Validation is a STRUCTURAL load: copper
layers, footprints INSIDE the board outline, and routable nets.

Usage: python3 validate_candidate.py <file.kicad_pcb>
PASS: pcbnew loads; 2<=copper_layers<=8; footprints>=10; routable_nets>=20; has a
board outline with <= max(2, 10%) footprints outside it (off-board = curation defect).
"""
import json, sys, subprocess, os, re
KPY = "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3"

# Runs inside KiCad's python. Emits one JSON line of raw metrics (or an error).
PCBNEW_METRICS = r'''
import pcbnew, json, sys
from collections import defaultdict
try:
    b = pcbnew.LoadBoard(sys.argv[1])
except Exception as e:
    print(json.dumps({"error": ("pcbnew load: %s: %s" % (type(e).__name__, e))[:160]})); sys.exit(0)
layers = b.GetCopperLayerCount()
fps = list(b.GetFootprints())
bbox = b.GetBoardEdgesBoundingBox()
has_outline = bbox.GetWidth() > 0 and bbox.GetHeight() > 0
x0, y0, x1, y1 = bbox.GetLeft(), bbox.GetTop(), bbox.GetRight(), bbox.GetBottom()
off, maxpads = 0, 0
padcount = defaultdict(int)
for fp in fps:
    p = fp.GetPosition()
    if has_outline and (p.x < x0 or p.x > x1 or p.y < y0 or p.y > y1):
        off += 1
    pads = fp.Pads()
    maxpads = max(maxpads, len(pads))
    for pad in pads:
        nc = pad.GetNetCode()
        if nc > 0:
            padcount[nc] += 1
routable = sum(1 for n in padcount.values() if n >= 2)
print(json.dumps({"copper_layers": layers, "footprints": len(fps), "off_board_fps": off,
                  "has_outline": bool(has_outline), "routable_nets": routable, "max_pads": maxpads}))
'''


def classify(layers, fps, routable, big_bga):
    if big_bga or layers >= 6 or fps >= 120 or routable >= 250:
        return "hard"
    if layers >= 4 or fps >= 45 or routable >= 80:
        return "medium"
    return "easy"


def main():
    f = sys.argv[1]
    v = {"file": os.path.basename(f), "pass": False}
    # KiCad version token (cheap, from the file header)
    try:
        m = re.search(r"\(version\s+(\d+)\)", open(f, errors="replace").read(400))
        v["kicad_version"] = int(m.group(1)) if m else None
    except Exception:
        v["kicad_version"] = None
    # metrics via pcbnew
    try:
        r = subprocess.run([KPY, "-c", PCBNEW_METRICS, f], capture_output=True, text=True, timeout=180)
        line = (r.stdout or "").strip().splitlines()[-1] if r.stdout.strip() else ""
        m = json.loads(line) if line else {"error": (r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "no output")[:160]}
    except Exception as e:
        m = {"error": f"{type(e).__name__}: {e}"[:160]}
    v.update(m)
    if "error" in m:
        v["reject_reason"] = m["error"]; print(json.dumps(v)); return
    v["tier"] = classify(v["copper_layers"], v["footprints"], v["routable_nets"], v["max_pads"] >= 200)
    off = v["off_board_fps"]
    off_ok = v["has_outline"] and off <= max(2, 0.1 * v["footprints"])
    if not off_ok:
        v["reject_reason"] = ("no board outline" if not v["has_outline"]
                              else f"{off} footprints off-board (curation defect)")
    v["pass"] = (2 <= v["copper_layers"] <= 8 and v["footprints"] >= 10
                 and v["routable_nets"] >= 20 and off_ok)
    print(json.dumps(v))


if __name__ == "__main__":
    main()
