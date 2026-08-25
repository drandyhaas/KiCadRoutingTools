#!/usr/bin/env python3
"""Fetch set14 .kicad_pcb sources listed in manifest_set14.json (raw download).

Mirrors fetch_set10.py. Downloads each board (and its sibling .kicad_pro, which
carries the DRC floor) into $STRESS_DIR/sources/github_set14/.
After fetching, run `bash prep_set14.sh` (needs KiCad's bundled python /
pcbnew) to produce boards_set14/ (routed reference) +
boards_unrouted_set14/ (stripped).

  python3 fetch_set14.py
  bash prep_set14.sh
"""
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
MANIFEST = HERE / "manifest_set14.json"


def main():
    boards = json.loads(MANIFEST.read_text())
    out_dir = STRESS / "sources/github_set14"
    out_dir.mkdir(parents=True, exist_ok=True)
    ok = 0
    for b in boards:
        dest = out_dir / Path(b["file"]).name
        r = subprocess.run(["curl", "-sL", "--fail", b["raw_url"], "-o", str(dest)],
                           capture_output=True)
        if r.returncode != 0 or not dest.exists() or dest.stat().st_size == 0:
            print(f"  FAIL {b['repo']}  <- {b['raw_url']}")
            continue
        # sibling .kicad_pro: never drop it (see CLAUDE.md #441 -- a board without
        # its project file resolves its DRC floor from the STOCK netclass).
        pro_url = b["raw_url"][: -len(".kicad_pcb")] + ".kicad_pro"
        subprocess.run(["curl", "-sL", "--fail", pro_url, "-o",
                        str(dest.with_suffix(".kicad_pro"))], capture_output=True)
        ok += 1
        print(f"  OK  {b['repo']:42} {dest.stat().st_size // 1024}KB  [{b.get('tier','?')}]")
    print(f"\n{ok}/{len(boards)} set14 sources -> {out_dir}")
    return 0 if ok == len(boards) else 1


if __name__ == "__main__":
    sys.exit(main())
