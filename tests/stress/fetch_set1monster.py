#!/usr/bin/env python3
"""Fetch extreme set1monster .kicad_pcb sources listed in manifest_set1monster.json (raw download).

Mirrors fetch_set4.py but reads manifest_set1monster.json and downloads into
$STRESS_DIR/sources/github_set1monster/. Set 5 is a 15-board mix of easy/medium/hard
open-hardware boards (MCU/USB dev boards, synth/audio modules, FPGA/SoC/SDR).
After fetching, run `bash prep_set1monster.sh` (needs KiCad's bundled python / pcbnew)
to produce boards_set1monster/ (routed reference) + boards_unrouted_set1monster/ (stripped).

  python3 fetch_set1monster.py
  bash prep_set1monster.sh
"""
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
MANIFEST = HERE / "manifest_set1monster.json"


def main():
    boards = json.loads(MANIFEST.read_text())
    out_dir = STRESS / "sources/github_set1monster"
    out_dir.mkdir(parents=True, exist_ok=True)
    ok = 0
    for b in boards:
        dest = out_dir / Path(b["file"]).name
        r = subprocess.run(["curl", "-sL", "--fail", b["raw_url"], "-o", str(dest)],
                           capture_output=True)
        if r.returncode != 0 or not dest.exists() or dest.stat().st_size == 0:
            print(f"  FAIL {b['repo']}  <- {b['raw_url']}")
            continue
        ok += 1
        print(f"  OK  {b['repo']:42} {dest.stat().st_size // 1024}KB  [{b.get('tier','?')}]")
    print(f"\n{ok}/{len(boards)} extreme set1monster sources -> {out_dir}")
    return 0 if ok == len(boards) else 1


if __name__ == "__main__":
    sys.exit(main())
