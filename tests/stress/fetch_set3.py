#!/usr/bin/env python3
"""Fetch set-3 .kicad_pcb sources listed in manifest_set3.json (raw download).

Mirrors fetch_boards.py but is manifest-driven: every board's raw_url and
destination are fixed in tests/stress/manifest_set3.json, so a checkout
reproduces the exact corpus. Downloads into $STRESS_DIR/sources/github_set3/.

Set 3 is intentionally heavy on OLD KiCad formats (v4 / v20171130 .. v8): the
text parser cannot read pre-v6 boards directly, but the pcbnew round-trip in
prep_set3.sh upgrades them to the current format before stripping. Run:

  python3 fetch_set3.py
  bash prep_set3.sh          # needs KiCad's bundled python (pcbnew)
"""
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
MANIFEST = HERE / "manifest_set3.json"


def main():
    boards = json.loads(MANIFEST.read_text())
    out_dir = STRESS / "sources/github_set3"
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
        print(f"  OK  {b['repo']:42} v{b['version']:<9} {dest.stat().st_size // 1024}KB")
    print(f"\n{ok}/{len(boards)} set-3 sources -> {out_dir}")
    return 0 if ok == len(boards) else 1


if __name__ == "__main__":
    sys.exit(main())
