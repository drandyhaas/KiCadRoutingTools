#!/usr/bin/env python3
"""Fetch .kicad_pcb files from GitHub repos (raw download, no cloning)."""
import json
import subprocess
import sys
import re
from pathlib import Path
import os
STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))

REPOS = [
    # (owner/repo, note)
    ("beekeeb/piantor", "split keyboard, simple 2-layer"),
    ("duckyb/urchin", "34-key keyboard"),
    ("gcormier/megadesk", "IKEA desk controller, simple"),
    ("tjhorner/upsy-desky", "standing desk ESP32"),
    ("cardonabits/haxo-hw", "haxophone MCU board"),
    ("scottbez1/splitflap", "split-flap display controller"),
    ("scottbez1/smartknob", "BLDC haptic knob, medium"),
    ("skot/bitaxe", "Bitcoin ASIC miner, medium-complex"),
    ("Ottercast/OtterCastAudioV2", "Allwinner SoC + DDR, complex"),
    ("wntrblm/Castor_and_Pollux", "SAMD21 synth module"),
    ("jakkra/ZSWatch", "nRF5340 smartwatch, dense"),
    ("opulo-inc/lumenpnp", "pick-and-place machine boards"),
    ("GlasgowEmbedded/glasgow", "FPGA BGA interface tool"),
    ("OLIMEX/Neo6502", "retro computer RP2040"),
    ("OLIMEX/RP2040-PICO-PC", "RP2040 mini PC"),
    ("greatscottgadgets/hackrf", "SDR RF board"),
    ("tillitis/tillitis-key1", "security key"),
    ("Ottercast/OtterCastAmp", "amplifier board"),
    ("psychogenic/riffpga", "FPGA board"),
    ("antmicro/lpddr4-testbed", "6-layer BGA LPDDR4 - set-1 replacement for dropped spirit_cm5"),
]

OUT = STRESS / "sources/github_set1"
OUT.mkdir(parents=True, exist_ok=True)
MAX_PER_REPO = 4  # avoid keyboard repos with dozens of variants


def gh_json(args):
    r = subprocess.run(["gh", "api"] + args, capture_output=True, text=True)
    if r.returncode != 0:
        return None
    return json.loads(r.stdout)


def main():
    manifest = []
    for repo, note in REPOS:
        info = gh_json([f"repos/{repo}"])
        if not info:
            print(f"SKIP {repo}: repo not found")
            continue
        branch = info["default_branch"]
        tree = gh_json([f"repos/{repo}/git/trees/{branch}?recursive=1"])
        if not tree:
            print(f"SKIP {repo}: tree fetch failed")
            continue
        pcbs = [e for e in tree.get("tree", [])
                if e["path"].endswith(".kicad_pcb")]
        # Prefer larger files (main boards) over tiny test/panel files
        pcbs.sort(key=lambda e: e.get("size", 0), reverse=True)
        pcbs = pcbs[:MAX_PER_REPO]
        if not pcbs:
            print(f"SKIP {repo}: no .kicad_pcb files")
            continue
        for e in pcbs:
            path = e["path"]
            safe = repo.replace("/", "__") + "__" + path.replace("/", "_")
            dest = OUT / safe
            url = f"https://raw.githubusercontent.com/{repo}/{branch}/{path}"
            r = subprocess.run(["curl", "-sL", "--fail", url, "-o", str(dest)],
                               capture_output=True)
            if r.returncode != 0:
                print(f"  FAIL download {repo}/{path}")
                continue
            head = dest.read_text(errors="replace")[:2000]
            m = re.search(r"\(version\s+(\d+)\)", head)
            ver = int(m.group(1)) if m else 0
            size_kb = dest.stat().st_size // 1024
            print(f"  OK {repo}/{path}  version={ver} size={size_kb}KB")
            manifest.append({"repo": repo, "path": path, "file": str(dest),
                             "version": ver, "size_kb": size_kb, "note": note,
                             "url": f"https://github.com/{repo}"})
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"\n{len(manifest)} files downloaded -> {OUT}/manifest.json")


if __name__ == "__main__":
    main()
