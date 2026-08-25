#!/usr/bin/env python3
"""Fetch set-9 .kicad_pcb sources listed in manifest_set9.json (raw download).

Mirrors fetch_set5.py but reads manifest_set9.json and downloads into
$STRESS_DIR/sources/github_set9/. Set 9 is a 15-board mix of easy/medium/hard
open-hardware boards themed around RF/wireless/networking (LoRa/BLE/Zigbee
nodes, an SDR, a ham-radio handheld) and power electronics (USB-PD chargers,
a PoE injector, a solar/battery node, a multi-rail robotics PSU).
After fetching, run `bash prep_set9.sh` (needs KiCad's bundled python / pcbnew)
to produce boards_set9/ (routed reference) + boards_unrouted_set9/ (stripped).

  python3 fetch_set9.py
  bash prep_set9.sh
"""
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
MANIFEST = HERE / "manifest_set9.json"


def main():
    boards = json.loads(MANIFEST.read_text())
    out_dir = STRESS / "sources/github_set9"
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
    print(f"\n{ok}/{len(boards)} set-9 sources -> {out_dir}")
    return 0 if ok == len(boards) else 1


if __name__ == "__main__":
    sys.exit(main())
