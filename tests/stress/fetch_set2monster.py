#!/usr/bin/env python3
"""Fetch set2monster .kicad_pcb sources listed in manifest_set2monster.json (raw download).

Mirrors fetch_set5.py but reads manifest_set2monster.json and downloads into
$STRESS_DIR/sources/github_set2monster/. set2monster is a 15-board EXTREME
"monster" corpus (huge, dense, many-layer open-hardware boards) with a
different flavor than set1monster: large AI-accelerator / compute-carrier
baseboards (Jetson Orin/Nano, Snapdragon 625, CM4), server/telecom adapters
(OCuLink 10GbE/PCIe), a GPU cluster backplane, an LPDDR5 testbed, and
camera/broadcast (SDI-MIPI, GMSL) boards. After fetching, run
`bash prep_set2monster.sh` (needs KiCad's bundled python / pcbnew) to produce
boards_set2monster/ (routed reference) + boards_unrouted_set2monster/
(stripped).

  python3 fetch_set2monster.py
  bash prep_set2monster.sh
"""
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
MANIFEST = HERE / "manifest_set2monster.json"


def main():
    boards = json.loads(MANIFEST.read_text())
    out_dir = STRESS / "sources/github_set2monster"
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
    print(f"\n{ok}/{len(boards)} set2monster sources -> {out_dir}")
    return 0 if ok == len(boards) else 1


if __name__ == "__main__":
    sys.exit(main())
