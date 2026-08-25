#!/usr/bin/env python3
"""Fetch set-4 .kicad_pcb sources listed in manifest_set4.json (raw download).

Mirrors fetch_set3.py but reads manifest_set4.json and downloads into
$STRESS_DIR/sources/github_set4/. Set 4 starts as a single board
(Rahul9-spb/FPGA-1 -- an FPGA + SDRAM design, BGA/QFN, 4-layer, KiCad 10);
append more entries to the manifest to grow it. Run:

  python3 fetch_set4.py
  bash prep_set4.sh          # needs KiCad's bundled python (pcbnew)
"""
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
MANIFEST = HERE / "manifest_set4.json"


def main():
    boards = json.loads(MANIFEST.read_text())
    out_dir = STRESS / "sources/github_set4"
    out_dir.mkdir(parents=True, exist_ok=True)
    ok = 0
    for b in boards:
        # The manifest pins an absolute "file"; honor $STRESS_DIR by re-deriving
        # the destination from the safe basename so a relocated corpus still works.
        dest = out_dir / Path(b["file"]).name
        r = subprocess.run(["curl", "-sL", "--fail", b["raw_url"], "-o", str(dest)],
                           capture_output=True)
        if r.returncode != 0 or not dest.exists() or dest.stat().st_size == 0:
            print(f"  FAIL {b['repo']}  <- {b['raw_url']}")
            continue
        ok += 1
        print(f"  OK  {b['repo']:42} v{b.get('version', '?'):<9} {dest.stat().st_size // 1024}KB")
    print(f"\n{ok}/{len(boards)} set-4 sources -> {out_dir}")
    return 0 if ok == len(boards) else 1


if __name__ == "__main__":
    sys.exit(main())
