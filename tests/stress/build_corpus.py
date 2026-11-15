#!/usr/bin/env python3
"""Build the stress-test corpus in STRESS_DIR from the recipes in this repo, so
anyone can run the corpus A/B (#1918).

    python3 tests/stress/build_corpus.py                      # sets 1-28 + set1arc
    python3 tests/stress/build_corpus.py --sets set10,set11   # some sets
    python3 tests/stress/build_corpus.py --chains-only        # boards already prepped

For each set it runs the set's own tracked recipe -- `fetch_setN.py` to
download the source boards listed in `manifest_setN.json` from their upstream
repos, then `prep_setN.sh` to strip them into `boards_unrouted_<set>/`
(set1arc: `prep_set1arc.py --stage-only`, from boards the other sets
produce) -- and then
installs the recorded routing chains from `tests/stress/corpus_chains/<set>/
<board>/` into `runs_<set>/<board>/`, where `cloud_replay_sets.py` and
`ab_replay_grade.py` look for them.

The boards are NOT redistributed: they are fetched from their authors' repos.
So the corpus you build is close to the maintainer's but not identical to it:
an upstream repo may have changed or disappeared since the chains were
recorded, a different KiCad writes a slightly different prepped board, and
every prep gives the board fresh random uuids. Compare two arms you ran
yourself on ONE corpus (a control at the base commit and your branch), never
your numbers against someone else's.

A chain is installed only when its input board was produced; the boards whose
input is missing are listed at the end and are simply absent from the corpus.
Prep needs KiCad's bundled python (`--kicad-python`, else $KICAD_PYTHON, else
the platform's KiCad install, as `py_router/kicad_locate.py` finds it).
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CHAINS = HERE / "corpus_chains"
DEFAULT_SETS = [f"set{i}" for i in range(1, 29)] + ["set1arc"]
SIBLINGS = (".kicad_pcb", ".kicad_pro", ".kicad_dru", ".kicad_prl")


def expand_sets(spec: str) -> list:
    out = []
    for part in spec.split(","):
        part = part.strip()
        m = re.fullmatch(r"set(\d+)-set(\d+)", part)
        if m:
            out += [f"set{i}" for i in range(int(m.group(1)), int(m.group(2)) + 1)]
        elif part:
            out.append(part)
    return out


def kicad_python(explicit):
    """The first candidate interpreter that can import pcbnew."""
    if explicit:
        return explicit
    sys.path.insert(0, str(REPO / "py_router"))
    from kicad_locate import kicad_python_candidates
    for cand in kicad_python_candidates(this_interpreter=sys.executable):
        if os.path.exists(cand) and subprocess.run(
                [cand, "-c", "import pcbnew"], capture_output=True).returncode == 0:
            return cand
    return None


def run(cmd, env, label):
    print(f"  $ {' '.join(str(c) for c in cmd)}", flush=True)
    r = subprocess.run([str(c) for c in cmd], env=env, cwd=str(HERE))
    if r.returncode != 0:
        print(f"  WARNING: {label} exited {r.returncode} (boards it did not produce are skipped)")


def fetch_and_prep(s, env):
    if s == "set1arc":
        run([sys.executable, "prep_set1arc.py", "--stage-only"], env, "prep set1arc")
    else:
        fetch, prep = HERE / f"fetch_{s}.py", HERE / f"prep_{s}.sh"
        if not fetch.exists() or not prep.exists():
            print(f"  {s}: no fetch_{s}.py / prep_{s}.sh in the repo -- skipped")
            return
        run([sys.executable, fetch.name], env, f"fetch {s}")
        run(["bash", prep.name], env, f"prep {s}")


def chain_input(manifest: Path, stress: Path):
    """The unrouted input board a chain starts from, mapped into `stress`."""
    m = re.search(r"(/\S*?/boards_unrouted_[^/\s'\"]+/[^/\s'\"]+\.kicad_pcb)", manifest.read_text())
    if not m:
        return None
    p = Path(m.group(1))
    return stress / p.parent.name / p.name


def install_chains(s, stress):
    src_set = CHAINS / s
    if not src_set.is_dir():
        print(f"  {s}: no recorded chains in {src_set.relative_to(REPO)}")
        return [], []
    installed, missing = [], []
    for bdir in sorted(p for p in src_set.iterdir() if p.is_dir()):
        man = bdir / "redo_commands.sh"
        board = bdir.name
        inp = chain_input(man, stress) or stress / f"boards_unrouted_{s}" / f"{board}.kicad_pcb"
        if not inp.exists():
            missing.append(f"{s}/{board}")
            continue
        dst = stress / f"runs_{s}" / board
        dst.mkdir(parents=True, exist_ok=True)
        for f in bdir.iterdir():
            shutil.copy2(f, dst / f.name)
        # A chain that starts from a copy of its input inside the run dir
        # (`route_planes.py <board>.kicad_pcb ...`) gets that copy, siblings too.
        if re.search(rf"(?<![/\w]){re.escape(board)}\.kicad_pcb\b", man.read_text()):
            for ext in SIBLINGS:
                sib = inp.with_suffix(ext)
                if sib.exists():
                    shutil.copy2(sib, dst / f"{board}{ext}")
        installed.append(f"{s}/{board}")
    return installed, missing


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--sets", default=",".join(DEFAULT_SETS),
                    help="comma list, ranges allowed (set1-set28,set1arc)")
    ap.add_argument("--stress-dir", default=os.environ.get(
        "STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
    ap.add_argument("--kicad-python", default=None)
    ap.add_argument("--chains-only", action="store_true",
                    help="skip fetch and prep; only install the chains")
    args = ap.parse_args()

    sets = expand_sets(args.sets)
    # set1arc is staged from boards the other sets produce, so it goes last.
    sets = [s for s in sets if s != "set1arc"] + (["set1arc"] if "set1arc" in sets else [])
    stress = Path(args.stress_dir).expanduser()
    stress.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, STRESS_DIR=str(stress))
    kpy = None
    if not args.chains_only:
        kpy = kicad_python(args.kicad_python)
        if not kpy:
            sys.exit("no KiCad python found: pass --kicad-python or set KICAD_PYTHON")
        env["KICAD_PYTHON"] = kpy
        print(f"STRESS_DIR={stress}\nKiCad python: {kpy}")

    all_in, all_miss = [], []
    for s in sets:
        print(f"== {s}", flush=True)
        if not args.chains_only:
            fetch_and_prep(s, env)
        got, miss = install_chains(s, stress)
        print(f"  {s}: {len(got)} chain(s) installed, {len(miss)} without an input board")
        all_in += got
        all_miss += miss
    print(f"\n{len(all_in)} boards ready in {stress}")
    if all_miss:
        print(f"{len(all_miss)} recorded chain(s) have no input board (fetch or prep failed, "
              f"or the upstream repo is gone) and are left out:")
        for b in all_miss:
            print(f"  {b}")


if __name__ == "__main__":
    main()
