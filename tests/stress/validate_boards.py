#!/usr/bin/env python3
"""Validate candidate boards parse with kicad_parser; collect complexity stats."""
import json
import sys
import traceback
from pathlib import Path
import os
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from kicad_parser import parse_kicad_pcb  # noqa: E402

SRC = STRESS / "sources/github"
manifest = json.loads((SRC / "manifest.json").read_text())

results = []
for entry in manifest:
    f = entry["file"]
    if entry["version"] < 20211014:
        continue  # pre-KiCad-6 format, parser doesn't support
    try:
        pcb = parse_kicad_pcb(f)
        copper = pcb.board_info.copper_layers
        n_pads = sum(len(fp.pads) for fp in pcb.footprints.values())
        # connected nets (>= 2 pads) are what the router would route
        routable = sum(1 for n in pcb.nets.values() if len(n.pads) >= 2 and n.net_id != 0)
        big_fps = sorted(
            ((ref, fp.footprint_name, len(fp.pads)) for ref, fp in pcb.footprints.items()
             if len(fp.pads) > 40), key=lambda x: -x[2])[:5]
        results.append({
            **entry,
            "parse": "ok",
            "copper_layers": len(copper),
            "nets": len(pcb.nets),
            "routable_nets": routable,
            "footprints": len(pcb.footprints),
            "pads": n_pads,
            "segments": len(pcb.segments),
            "vias": len(pcb.vias),
            "bounds": pcb.board_info.board_bounds,
            "big_footprints": big_fps,
        })
        print(f"OK   {Path(f).name[:60]:62s} layers={len(copper)} nets={len(pcb.nets):4d} "
              f"routable={routable:4d} fps={len(pcb.footprints):3d} pads={n_pads:5d}")
    except Exception as e:
        results.append({**entry, "parse": "FAIL", "error": f"{type(e).__name__}: {e}"})
        print(f"FAIL {Path(f).name[:60]:62s} {type(e).__name__}: {e}")
        traceback.print_exc(limit=2)

out = SRC / "validated.json"
out.write_text(json.dumps(results, indent=2))
print(f"\n{sum(1 for r in results if r['parse']=='ok')} parseable -> {out}")
