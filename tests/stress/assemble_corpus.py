#!/usr/bin/env python3
"""Assemble a corpus set from candidates/curation.json:
  - copy each source .kicad_pcb into sources/github_setN/<slug>.kicad_pcb
  - try to fetch the sibling .kicad_pro from the repo (best effort)
  - write manifest_setN.json (pre-existing entries preserved + new)
  - regenerate fetch_setN.py and prep_setN.sh with a full MAP array
Run with plain python3 (the prep step itself uses KiCad's python separately).

`set` in curation.json is the set's NAME, so any set works -- "set26", and the
extreme sets "set3monster" too, not just an integer.

Re-running is idempotent: a pre-existing manifest entry survives unless the new
curation re-adds the same `short_name`. (This used to be "keep entries WITHOUT a
short_name", which was really a stand-in for set4's one hand-written board and
would have silently dropped every entry in an already-populated set.)"""
import json, os, shutil, subprocess
from pathlib import Path

STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
CAND = STRESS / "sources/candidates"
# This script's own directory IS tests/stress -- never a hardcoded home dir.
STRESS_TESTS = Path(__file__).resolve().parent
cur = json.load(open(CAND / "curation.json"))

TIER_PKG = {"easy": "2-layer,MCU", "medium": "4-layer,USB,MCU", "hard": "BGA,FPGA/SoC,high-speed",
            "monster": "BGA/SoC,high-speed,high-layer"}

# What a set IS, carried into the generated fetch_setN.py docstring. Without this
# the generator overwrites any hand-written description every time it re-runs.
SET_BLURB = {
    "set28": (
        "set28 began as the hwidvorakinfo (Daniel Dvorak) batch -- a single-designer\n"
        "survey of that author's 22 public repos, of which only 3 ship a\n"
        ".kicad_pcb at all (the rest are firmware-only STM32 Eclipse projects).\n"
        "BEASTH7_01 was rejected by validate_candidate.py as pre-KiCad-6 format\n"
        "(20171130, ships a v5 .pro rather than a .kicad_pro), leaving two:\n"
        "a 2 GHz active scope probe (RF/analog, few nets but tight geometry) and\n"
        "a 218-footprint smart agricultural switch (mains-side power + MCU).\n"
        "\n"
        "mez_rx joined later from a DIFFERENT source: a user-submitted board\n"
        "attached to issue #614 by ughstudios (Daniel Gleason), who reported the\n"
        "router could not complete it. It is the first ARCHIVE-sourced entry\n"
        "(a .zip attachment, not a raw repo URL -- see archive_url/archive_member)\n"
        "and the corpus's hardest board to date: 8 layers, a 400-ball 0.8mm-pitch\n"
        "FPGA, a real .kicad_dru DFM ruleset, and 91.1% completion against a\n"
        "corpus median of 100%. It has NEVER been human-routed, so boards_set28/\n"
        "mez_rx.kicad_pcb is a DEGENERATE reference (0 segments/0 vias):\n"
        "compare_to_original and the DRC-delta-vs-original are meaningless for it.\n"
        "\n"
        "storm_tracker (solderable/storm-tracker-hardware) joined 2026-08-13 at\n"
        "Andy's request: a 4-layer 36.5x69mm ESP32-C6-MINI-1U lightning detector\n"
        "(AS3935 + 1.54\" e-paper over a 24-pin 0.5mm FFC, USB-C, battery charger).\n"
        "Small but dense -- 342 vias in 2500mm2, a 0.13mm fab floor -- and one of\n"
        "only two corpus boards with a real .kicad_dru; unlike mez_rx's DFM\n"
        "ruleset, its single rule is CONDITIONED on a netclass\n"
        "(A.NetClass == '90R_DP' && B.NetName == 'GND'), and its USB pair is\n"
        "netclass-assigned by PATTERN (*USB_D* -> 90R_DP). Fully human-routed, so\n"
        "unlike mez_rx it is a usable compare_to_original reference."
    ),
    "set3monster": (
        "set3monster is the \"extreme / intractable\" monster batch: boards whose\n"
        "size or stackup puts a single route step near (or past) the RUNBOOK 3h/command\n"
        "cap. lora_cubesat_cm (485 nets / 6 layers) was moved here out of set11 for\n"
        "exactly that reason. The rest are 8-14 copper layer Antmicro boards -- the\n"
        "first >8-layer boards in the corpus, admitted once validate_candidate.py's\n"
        "upper layer bound was removed in favour of the `monster` tier."
    ),
}

def fetch_pro(raw_url, dest):
    """Fetch the board's sibling project files from the repo beside the board.

    Both matter: the `.kicad_pro` carries the DRC floor (#441) and the
    `.kicad_dru` the per-layer/conditioned clearance rules (#498), which
    OUTRANK --clearance and which check_drc reads back at grading time. A
    curl --fail 404 can still leave a zero-byte file, which would read as
    "rules present, none defined" -- unlink it. Returns whether the .kicad_pro
    (the one recorded as `has_kicad_pro`) was obtained.
    """
    if not raw_url or not raw_url.endswith(".kicad_pcb"): return False
    got_pro = False
    for ext in (".kicad_pro", ".kicad_dru"):
        out = dest if ext == ".kicad_pro" else dest.with_suffix(ext)
        url = raw_url[:-len(".kicad_pcb")] + ext
        r = subprocess.run(["curl", "-sL", "--fail", url, "-o", str(out)], capture_output=True)
        if r.returncode == 0 and out.exists() and out.stat().st_size > 20:
            got_pro = got_pro or ext == ".kicad_pro"
            continue
        if out.exists(): out.unlink()
    return got_pro


def copy_local_siblings(src_path, src_dir, slug):
    """Copy the candidate's OWN sibling project files into the set's source dir.

    For an ARCHIVE-sourced board (a .zip attached to a GitHub issue) there is no
    raw .kicad_pcb URL to derive a sibling .kicad_pro from, so fetch_pro cannot
    work -- but the archive already carried the siblings next to the board. Take
    them from there. `.kicad_dru` matters as much as `.kicad_pro` here (#498:
    per-layer clearance lives in the dru and OUTRANKS --clearance), and
    prep_set2.py copies both onward to the routed + stripped outputs.
    """
    got = {}
    for ext in (".kicad_pro", ".kicad_dru"):
        sib = os.path.splitext(str(src_path))[0] + ext
        if os.path.exists(sib):
            shutil.copy(sib, src_dir / f"{slug}{ext}")
            got[ext] = True
    return got.get(".kicad_pro", False)

by_set = {}
for b in cur: by_set.setdefault(str(b["set"]), []).append(b)

for s in sorted(by_set):
    src_dir = STRESS / f"sources/github_{s}"
    src_dir.mkdir(parents=True, exist_ok=True)
    manifest = []
    # Preserve any board already curated into this set that we are NOT re-adding.
    incoming = {b["name"] for b in by_set[s]}
    prev = STRESS_TESTS / f"manifest_{s}.json"
    if prev.exists():
        manifest.extend([e for e in json.load(open(prev))
                         if e.get("short_name") not in incoming])
    mapping = []  # (short_name, filename_fragment) for prep MAP
    pros = 0
    for b in sorted(by_set[s], key=lambda x: (x["tier"], x["name"])):
        slug = b["slug"]
        fn = f"{slug}.kicad_pcb"
        dst = src_dir / fn
        src_path = b["src"] if os.path.isabs(b["src"]) else str(CAND / b["src"])
        shutil.copy(src_path, dst)
        if b.get("archive_url"):
            got_pro = copy_local_siblings(src_path, src_dir, slug)
        else:
            got_pro = fetch_pro(b["raw_url"], src_dir / f"{slug}.kicad_pro")
        pros += 1 if got_pro else 0
        manifest.append({
            "set": s, "repo": b["repo"], "path": b["path"], "branch": b.get("branch", "main"),
            "file": fn, "raw_url": b["raw_url"], "github_url": b["github_url"],
            # Archive-sourced board (e.g. a .zip attached to a GitHub issue):
            # fetch_setN.py downloads the archive and extracts these members.
            "archive_url": b.get("archive_url", ""),
            "archive_member": b.get("archive_member", ""),
            "size_kb": dst.stat().st_size // 1024, "layers_est": b["layers"],
            "footprints": b["footprints"], "routable_nets": b["routable_nets"],
            "max_pads": b.get("max_pads"), "kicad_version": b.get("kicad_version"),
            "has_kicad_pro": got_pro,
            "tier": b["tier"], "packages": TIER_PKG.get(b["tier"], "?"),
            "lane": b.get("lane", ""),
            "license": "see repo", "short_name": b["name"], "note": b["note"] or b["repo"],
        })
        mapping.append((b["name"], slug))
    json.dump(manifest, open(STRESS_TESTS / f"manifest_{s}.json", "w"), indent=2)
    # regenerate prep_setN.sh with the MAP. The MAP covers the WHOLE manifest, not
    # just this run's additions -- fetch_setN.py downloads every manifest entry, so
    # a MAP of only the new boards would leave the pre-existing ones un-prepped on
    # a from-scratch rebuild.
    known = {n for n, _ in mapping}
    for e in manifest:
        sn = e.get("short_name")
        if sn and sn not in known:
            mapping.append((sn, Path(e["file"]).stem))
    mapping.sort()
    map_lines = "\n".join(f'  "{n}|{frag}"' for n, frag in mapping)
    prep = f"""#!/bin/bash
# Normalize+strip every {s} board (one pcbnew process each, segfault-safe).
# Reuses prep_set2.py (generic: <src> <routed_dst> <stripped_dst>).
#   stripped, unrouted boards -> boards_unrouted_{s}/
#   normalized routed reference + .kicad_pro -> boards_{s}/
# Auto-generated by assemble_corpus.py. Sources are modern KiCad (v6+).
set -u
SELF="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)"
STRESS="${{STRESS_DIR:-$HOME/Documents/kicad_stress_test}}"
SRC="$STRESS/sources/github_{s}"
KPY="${{KICAD_PYTHON:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3}}"
PREP="$SELF/prep_set2.py"
mkdir -p "$STRESS/boards_unrouted_{s}" "$STRESS/boards_{s}"

# short-name | source-filename-fragment (unique within github_{s}/)
MAP=(
{map_lines}
)

for entry in "${{MAP[@]}}"; do
  IFS='|' read -r name frag <<< "$entry"
  srcfile=$(find "$SRC" -maxdepth 1 -name '*.kicad_pcb' -name "*${{frag}}*" | head -1)
  if [ -z "$srcfile" ]; then echo "MISS $name (frag '$frag')"; continue; fi
  echo "== $name <- $(basename "$srcfile")"
  "$KPY" "$PREP" "$srcfile" "$STRESS/boards_{s}/$name.kicad_pcb" \\
         "$STRESS/boards_unrouted_{s}/$name.kicad_pcb" 2>/dev/null || echo "  FAIL $name"
  profile="${{srcfile%.kicad_pcb}}.kicad_pro"
  [ -f "$profile" ] && cp "$profile" "$STRESS/boards_{s}/$name.kicad_pro"
done
echo "Done. stripped -> boards_unrouted_{s}/ ; routed reference -> boards_{s}/"
"""
    (STRESS_TESTS / f"prep_{s}.sh").write_text(prep)
    os.chmod(STRESS_TESTS / f"prep_{s}.sh", 0o755)

    # regenerate fetch_setN.py -- pure boilerplate over manifest_setN.json, but
    # without it the set cannot be re-downloaded from a clean checkout.
    blurb = SET_BLURB.get(s, "")
    fetch = f'''#!/usr/bin/env python3
"""Fetch {s} .kicad_pcb sources listed in manifest_{s}.json (raw download).
{(chr(10) + blurb + chr(10)) if blurb else ""}
Downloads each board and its sibling project files -- the .kicad_pro (DRC floor,
#441) and the .kicad_dru (per-layer clearance rules, #498) --
into $STRESS_DIR/sources/github_{s}/. After fetching, run `bash prep_{s}.sh`
(needs KiCad's bundled python / pcbnew) to produce boards_{s}/ (routed
reference) + boards_unrouted_{s}/ (stripped).

  python3 fetch_{s}.py
  bash prep_{s}.sh

Auto-generated by assemble_corpus.py.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
MANIFEST = HERE / "manifest_{s}.json"


def fetch_archive(b, dest):
    """Board shipped as a .zip (e.g. attached to a GitHub issue) rather than a
    raw .kicad_pcb URL. Download the archive, extract `archive_member` as the
    board, and take its siblings from the SAME directory inside the archive --
    the .kicad_pro (DRC floor, #441) and .kicad_dru (per-layer clearance, #498)
    travel with it. Returns True on success.
    """
    with tempfile.TemporaryDirectory() as td:
        zp = Path(td) / "src.zip"
        r = subprocess.run(["curl", "-sL", "--fail", b["archive_url"], "-o", str(zp)],
                           capture_output=True)
        if r.returncode != 0 or not zp.exists() or zp.stat().st_size == 0:
            return False
        try:
            with zipfile.ZipFile(zp) as z:
                z.extractall(Path(td) / "x")
        except zipfile.BadZipFile:
            return False
        member = Path(td) / "x" / b["archive_member"]
        if not member.exists():
            return False
        shutil.copy(member, dest)
        for ext in (".kicad_pro", ".kicad_dru"):
            sib = member.with_suffix(ext)
            if sib.exists():
                shutil.copy(sib, dest.with_suffix(ext))
        return True


def main():
    boards = json.loads(MANIFEST.read_text())
    out_dir = STRESS / "sources/github_{s}"
    out_dir.mkdir(parents=True, exist_ok=True)
    ok = 0
    for b in boards:
        dest = out_dir / Path(b["file"]).name
        if b.get("archive_url"):
            if fetch_archive(b, dest):
                ok += 1
                print(f"  OK  {{b['repo']:42}} {{dest.stat().st_size // 1024}}KB  "
                      f"[{{b.get('tier','?')}}] (archive)")
            else:
                print(f"  FAIL {{b['repo']}}  <- {{b['archive_url']}}")
            continue
        r = subprocess.run(["curl", "-sL", "--fail", b["raw_url"], "-o", str(dest)],
                           capture_output=True)
        if r.returncode != 0 or not dest.exists() or dest.stat().st_size == 0:
            print(f"  FAIL {{b['repo']}}  <- {{b['raw_url']}}")
            continue
        # siblings: never drop them. Without the .kicad_pro a board resolves its
        # DRC floor from the STOCK netclass (#441); without the .kicad_dru every
        # routing step and check_drc lose the per-layer/conditioned clearance
        # rules that OUTRANK --clearance (#498). A 404 under --fail can still
        # leave a zero-byte file, which reads as "rules present, none" -- drop it.
        for ext in (".kicad_pro", ".kicad_dru"):
            sib = dest.with_suffix(ext)
            subprocess.run(["curl", "-sL", "--fail",
                            b["raw_url"][: -len(".kicad_pcb")] + ext,
                            "-o", str(sib)], capture_output=True)
            if sib.exists() and sib.stat().st_size == 0:
                sib.unlink()
        ok += 1
        print(f"  OK  {{b['repo']:42}} {{dest.stat().st_size // 1024}}KB  [{{b.get('tier','?')}}]")
    print(f"\\n{{ok}}/{{len(boards)}} {s} sources -> {{out_dir}}")
    return 0 if ok == len(boards) else 1


if __name__ == "__main__":
    sys.exit(main())
'''
    (STRESS_TESTS / f"fetch_{s}.py").write_text(fetch)
    os.chmod(STRESS_TESTS / f"fetch_{s}.py", 0o755)
    n_new = len(by_set[s])
    print(f"{s}: {n_new} new boards -> github_{s}/ ; sibling .kicad_pro fetched: "
          f"{pros}/{n_new} ; manifest={len(manifest)} entries ; MAP={len(mapping)} ; "
          f"prep_{s}.sh + fetch_{s}.py regenerated")
print("done")
