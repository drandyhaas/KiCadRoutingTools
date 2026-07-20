#!/usr/bin/env python3
"""Stage set-11 board sources listed in manifest_set11.json.

Set 11 boards are published as ZIP archives rather than raw .kicad_pcb
files (unlike fetch_set5..10's downloads) -- e.g. Adiuvo Engineering's
RP2350_FPGA_eensy board lives in the forgix_public Bitbucket repo as
Kicad_Project/RP2350_FPGA_eensy-main.zip): each entry's `raw_url` zip is
downloaded and its `zip_member` .kicad_pcb (+ sibling .kicad_pro) extracted
into $STRESS_DIR/sources/local_set11/. If the download fails and the entry
has a `local_path`, that file is copied instead. After staging, run
`bash prep_set11.sh` (needs KiCad's bundled python / pcbnew) to produce
boards_set11/ (routed reference) + boards_unrouted_set11/ (stripped).

  python3 fetch_set11.py
  bash prep_set11.sh
"""
import io
import json
import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
MANIFEST = HERE / "manifest_set11.json"


def _stage_from_zip(b, dest):
    # curl, not urllib: matches fetch_set5..10 and avoids macOS pythons
    # that ship without SSL root certificates.
    r = subprocess.run(["curl", "-sL", "--fail", b["raw_url"]],
                       capture_output=True, timeout=300)
    if r.returncode != 0 or not r.stdout:
        raise RuntimeError(f"curl rc={r.returncode}")
    zf = zipfile.ZipFile(io.BytesIO(r.stdout))
    member = b["zip_member"]
    dest.write_bytes(zf.read(member))
    pro_member = member[: -len(".kicad_pcb")] + ".kicad_pro"
    if pro_member in zf.namelist():
        dest.with_suffix(".kicad_pro").write_bytes(zf.read(pro_member))
        return True
    return False


def _stage_from_raw(b, dest):
    """Plain raw .kicad_pcb download (entries with no `zip_member`).

    Set 11 was originally all zip-published sources; the boards added to fill
    it out are ordinary raw GitHub URLs, like fetch_set5..10. Also grabs the
    sibling .kicad_pro -- without it the route step resolves its DRC floor from
    the STOCK netclass and KiCad grades phantom clearance violations (#441).
    """
    r = subprocess.run(["curl", "-sL", "--fail", b["raw_url"], "-o", str(dest)],
                       capture_output=True, timeout=300)
    if r.returncode != 0 or not dest.exists() or dest.stat().st_size == 0:
        raise RuntimeError(f"curl rc={r.returncode}")
    pro_url = b["raw_url"][: -len(".kicad_pcb")] + ".kicad_pro"
    p = subprocess.run(["curl", "-sL", "--fail", pro_url, "-o",
                        str(dest.with_suffix(".kicad_pro"))], capture_output=True, timeout=300)
    return p.returncode == 0


def _stage_from_local(b, dest):
    src = Path(b["local_path"])
    if not src.exists():
        return None
    shutil.copyfile(src, dest)
    pro = src.with_suffix(".kicad_pro")
    if pro.exists():
        shutil.copyfile(pro, dest.with_suffix(".kicad_pro"))
        return True
    return False


def main():
    boards = json.loads(MANIFEST.read_text())
    out_dir = STRESS / "sources/local_set11"
    out_dir.mkdir(parents=True, exist_ok=True)
    ok = 0
    for b in boards:
        dest = out_dir / Path(b["file"]).name
        got_pro = None
        origin = None
        if b.get("raw_url"):
            stage = _stage_from_zip if b.get("zip_member") else _stage_from_raw
            try:
                got_pro = stage(b, dest)
                origin = "zip" if b.get("zip_member") else "raw"
            except Exception as e:
                print(f"  WARN {b['short_name']}: download failed ({e}); trying local_path")
        if origin is None and b.get("local_path"):
            got_pro = _stage_from_local(b, dest)
            origin = "local" if got_pro is not None else None
        if origin is None:
            print(f"  MISS {b['short_name']}: no usable source "
                  f"(raw_url failed / local_path missing)")
            continue
        ok += 1
        print(f"  OK  {b['short_name']:24} {dest.stat().st_size // 1024}KB  "
              f"[{b.get('tier', '?')}] via {origin}"
              f"{'' if got_pro else '  (no .kicad_pro)'}")
    print(f"\n{ok}/{len(boards)} set-11 sources -> {out_dir}")
    return 0 if ok == len(boards) else 1


if __name__ == "__main__":
    sys.exit(main())
