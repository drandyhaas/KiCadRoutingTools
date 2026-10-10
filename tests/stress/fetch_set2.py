"""Fetch set-2 .kicad_pcb sources listed in manifest_set2.json (raw download).

Mirrors fetch_set10.py but reads manifest_set2.json and downloads into
$STRESS_DIR/sources/github_set2/. Set 2 is the 15 newer boards described in
README.md: FPGA/BGA, DDR4/DDR5 test beds, USB diff-pair boards, QFN/QFP
carriers and simple keyboards.
After fetching, run `bash prep_set2.sh` (needs KiCad's bundled python /
pcbnew) to produce boards_set2/ (routed reference) + boards_unrouted_set2/
(stripped).

  python3 fetch_set2.py
  bash prep_set2.sh
"""
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
MANIFEST = HERE / "manifest_set2.json"


def main():
    boards = json.loads(MANIFEST.read_text())
    out_dir = STRESS / "sources/github_set2"
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
    print(f"\n{ok}/{len(boards)} set-2 sources -> {out_dir}")
    return 0 if ok == len(boards) else 1


if __name__ == "__main__":
    sys.exit(main())
