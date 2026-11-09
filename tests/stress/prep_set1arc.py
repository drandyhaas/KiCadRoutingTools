#!/usr/bin/env python3
"""Build the arc-intact replay set, `set1arc`, from boards the corpus already has.

    python3 tests/stress/prep_set1arc.py              # stage and record every board
    python3 tests/stress/prep_set1arc.py --stage-only # no replay
    python3 tests/stress/prep_set1arc.py --boards a_arc,b_arc

`normalize_boards.py` and `prep_set2.py` rebuild Edge.Cuts as chained lines, so an
ordinary corpus input has almost no arc left on its outline, and a corpus replay
cannot see a change to how an arc outline is read. Changing them would void every
recorded baseline, so this set ADDS a few chains instead and touches nothing else.

For each row of `manifest_set1arc.json`, naming a `source_set` and `source_board`:

  boards_set1arc/<short>.kicad_pcb           the routed reference, arcs intact
  boards_unrouted_set1arc/<short>.kicad_pcb  `strip_copper_only.py` of it: no routing
                                             output, outline bit-identical, pour
                                             outlines kept (the ordinary prep drops
                                             those), siblings from the source's
                                             unrouted twin (what its chain read)
  runs_set1arc/<short>/redo_commands.sh      the board's RECORDED chain, repointed at
                                             that input, with the side files it names
  runs_set1arc/<short>/...                   the chain's boards, replayed with no LLM
  results_set1arc/<short>.json               the replay's wall time: `plan` prices the
                                             board from it, and it is what marks the
                                             board done for `run_queue.sh`, which would
                                             otherwise send an LLM worker at it

The source's own run, input and baseline are read and never written. A short name
ends `_arc` because several tools key a board by its bare name across sets
(`sweep_lib.discover_boards`, `upload_corpus.collect`, the cost table), so a twin
sharing its source's name would silently vanish from `--sets set17,set1arc`.

Everything but the manifest and this file lives under the stress dir and does not
travel with the repo. `cloud_replay_sets.py --only upload` puts it on the corpus
volume; another machine rebuilds it by running this.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO / "py_router"))
sys.path.insert(0, str(HERE / "modal_sweep"))

from copy_board import copy_board, copy_siblings  # noqa: E402
import kicad_parser as kp  # noqa: E402
import sweep_lib  # noqa: E402
sys.path.insert(0, str(HERE))
from ab_replay_grade import final_output_name  # noqa: E402

SET = "set1arc"
MANIFEST = HERE / f"manifest_{SET}.json"
DEFAULT_STRESS = Path(os.environ.get("STRESS_DIR",
                                     os.path.expanduser("~/Documents/kicad_stress_test")))
#: what the chain itself produces, or its project carries, is not a side file
_BOARD_FILES = (".kicad_pcb", ".kicad_pro", ".kicad_prl", ".kicad_dru")


def edge_arc_count(text: str) -> int:
    """Arcs and circles the board draws on Edge.Cuts, at board or footprint level."""
    n, depth, start, in_str, i = 0, 0, [], False, 0
    while i < len(text):
        c = text[i]
        if in_str:
            if c == '\\':
                i += 1
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == '(':
            start.append(i)
        elif c == ')' and start:
            s = start.pop()
            if text.startswith(('(gr_arc', '(fp_arc', '(gr_circle', '(fp_circle'), s) \
                    and '"Edge.Cuts"' in text[s:i]:
                n += 1
        i += 1
    return n


def repoint_chain(text: str, stress, src_set: str, src_board: str, short: str) -> str:
    """The recorded chain's text with its input and run dir moved onto `set1arc`.

    Only the three paths that name the source change. A manifest that never named
    the source's input is refused rather than written: its first step would read
    some other board, and the set would quietly stop being arc-intact."""
    old = sweep_lib.recorded_corpus_prefix(text) or str(stress)
    old_in = f"{old}/boards_unrouted_{src_set}/{src_board}.kicad_pcb"
    if old_in not in text:
        raise SystemExit(f"{src_set}/{src_board}: the recorded chain never reads {old_in}")
    out = text.replace(old_in, f"{stress}/boards_unrouted_{SET}/{short}.kicad_pcb")
    # `runs_set1_llm0801` etc. record under a suffixed dir, and this set's has none
    old_run = re.compile(re.escape(old) + rf"/runs_{re.escape(src_set)}[^/\s'\"]*/{re.escape(src_board)}"
                         + r"(?=[/\s'\"]|$)", re.MULTILINE)
    return old_run.sub(f"{stress}/runs_{SET}/{short}", out)


def side_files(text: str) -> list:
    """Bare file names the chain's own lines mention (`--net-clearances x.json`).
    An alphabetic extension, as `redo_stress_test.seed_side_files` asks, tells them
    from flag values and net names (`0.25`, `+3.3V`)."""
    named = set()
    for ln in text.splitlines():
        if ln.lstrip().startswith("#"):
            continue
        named.update(t.strip("'\"") for t in ln.split())
    return sorted(n for n in named
                  if "/" not in n and re.fullmatch(r"[\w.+-]+\.[A-Za-z]+", n)
                  and not n.endswith(_BOARD_FILES) and not n.endswith(".py"))


def stage(row: dict, stress: Path) -> Path:
    """Reference, input and repointed chain for one row -> the new run dir."""
    src_set, src_board, short = row["source_set"], row["source_board"], row["short_name"]
    routed = stress / f"boards_{src_set}" / f"{src_board}.kicad_pcb"
    twin = stress / f"boards_unrouted_{src_set}" / f"{src_board}.kicad_pcb"
    run_dirs = [d / src_board for d in sweep_lib.run_dirs_for(stress, src_set)
                if (d / src_board / "redo_commands.sh").exists()]
    if not routed.exists() or not twin.exists() or not run_dirs:
        raise SystemExit(f"{src_set}/{src_board}: needs {routed}, {twin} and a recorded "
                         f"run under runs_{src_set}; fetch and prep that set first")
    src_run = run_dirs[0]

    ref = stress / f"boards_{SET}" / f"{short}.kicad_pcb"
    inp = stress / f"boards_unrouted_{SET}" / f"{short}.kicad_pcb"
    run = stress / f"runs_{SET}" / short
    shutil.rmtree(run, ignore_errors=True)
    # the result is what says the recording finished: an old one must not outlive it
    (stress / f"results_{SET}" / f"{short}.json").unlink(missing_ok=True)
    for d in (ref.parent, inp.parent, run):
        d.mkdir(parents=True, exist_ok=True)
    copy_board(str(routed), str(ref))
    subprocess.run([sys.executable, "-X", "utf8", str(HERE / "strip_copper_only.py"),
                    str(ref), str(inp)], check=True, capture_output=True)
    copy_siblings(str(twin), str(inp))

    with open(inp, encoding="utf-8", errors="replace") as f:
        board = f.read()
    arcs = edge_arc_count(board)
    if not arcs:
        raise SystemExit(f"{short}: no arc on Edge.Cuts in {inp}; it does not belong in this set")

    text = (src_run / "redo_commands.sh").read_text(encoding="utf-8")
    (run / "redo_commands.sh").write_text(
        repoint_chain(text, stress, src_set, src_board, short), encoding="utf-8")
    for name in side_files(text):
        if (src_run / name).is_file():
            shutil.copy2(src_run / name, run / name)

    old, fine = kp.extract_board_bounds(board), kp.extract_board_bounds(board, kp.EDGE_CHORD_EPS)
    moved = max(abs(a - b) for a, b in zip(old, fine)) * 1000
    print(f"{short}: {arcs} Edge.Cuts arcs; bounds at the 5 um and 0.5 um sag differ by "
          f"{moved:.2f} um", flush=True)
    return run


def record(row: dict, stress: Path, run: Path) -> int:
    """Replay the repointed chain into its run dir, with this checkout's tools."""
    short = row["short_name"]
    text = (run / "redo_commands.sh").read_text(encoding="utf-8")
    recorded = sweep_lib.recorded_repo_prefix(text)
    remap = ["--remap", f"{recorded.rstrip('/')}:{REPO}"] if recorded and recorded != str(REPO) else []
    t0 = time.time()
    with open(run / "_record.log", "w", encoding="utf-8") as log:
        rc = subprocess.run([sys.executable, str(HERE / "redo_stress_test.py"),
                             str(run / "redo_commands.sh"), "--workdir", str(run), *remap,
                             "--skip-checks", "--continue-on-error",
                             "--timings-out", str(run / "timings.json")],
                            stdout=log, stderr=subprocess.STDOUT).returncode
    secs = time.time() - t0
    # --continue-on-error keeps the replay's own exit at 0 past a failed step, so
    # the recording stands on the chain's last board and each step's own exit
    final = run / (final_output_name(text) or "")
    steps = json.loads((run / "timings.json").read_text(encoding="utf-8")).get("commands", [])
    failed = sum(1 for c in steps if c.get("returncode"))
    if rc or not final.is_file() or failed:
        print(f"{short}: NOT RECORDED: replay rc {rc}, {failed} step(s) failed, final board "
              f"{final.name} {'present' if final.is_file() else 'missing'}; see {run / '_record.log'}",
              flush=True)
        return rc or 1
    results = stress / f"results_{SET}"
    results.mkdir(parents=True, exist_ok=True)
    (results / f"{short}.json").write_text(json.dumps(
        {"board": short, "source": f"{row['source_set']}/{row['source_board']}",
         "recorded_by": "prep_set1arc.py, no LLM",
         "timing": {"tool_exec_s": round(secs, 1)}}, indent=1), encoding="utf-8")
    with open(final, encoding="utf-8", errors="replace") as f:
        arcs = edge_arc_count(f.read())
    print(f"{short}: recorded in {secs:.0f} s; final board {final.name} has {arcs} Edge.Cuts arcs",
          flush=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stress-dir", default=str(DEFAULT_STRESS))
    ap.add_argument("--boards", default="", help="short names, comma-separated (default: all)")
    ap.add_argument("--stage-only", action="store_true", help="write the inputs and chains, replay nothing")
    a = ap.parse_args()
    stress = Path(a.stress_dir).expanduser()
    rows = json.loads(MANIFEST.read_text(encoding="utf-8"))
    want = {b.strip() for b in a.boards.split(",") if b.strip()}
    unknown = want - {r["short_name"] for r in rows}
    if unknown:
        raise SystemExit(f"not in {MANIFEST.name}: {', '.join(sorted(unknown))}")
    bad = 0
    for row in rows:
        if want and row["short_name"] not in want:
            continue
        run = stage(row, stress)
        if not a.stage_only:
            bad += 1 if record(row, stress, run) else 0
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
