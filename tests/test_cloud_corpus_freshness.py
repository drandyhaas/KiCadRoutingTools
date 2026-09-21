#!/usr/bin/env python3
"""cloud_replay_sets' upload stage, the two checks that run BEFORE money is
spent (2026-09-19):

  manifest_preflight   a LOCAL manifest without a `# cwd=<stress>/runs_<set>/
                       <board>` line is refused by name -- the cloud placer
                       stages the corpus at that path, and butterstick's repair
                       (re-run from a scratchpad) raised inside its container
                       after the arm was launched;
  stale_manifests      a board whose manifest on the volume is a different
                       SIZE than the local one is reported stale, one the
                       volume lacks absent, everything else fresh -- the
                       volume is listed once per set, never read per board.

Both are exercised on a temporary stress dir; the volume side is the pure
parser over (path, size) pairs, so no Modal account is touched.
"""
import os
import sys
import tempfile
from pathlib import Path

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress', 'modal_sweep'))
import cloud_replay_sets as crs  # noqa: E402

FAILS = []


def check(cond, msg):
    print(('  PASS  ' if cond else '  FAIL  ') + msg)
    if not cond:
        FAILS.append(msg)


def _manifest(stress: Path, run_dir: str, board: str, cwd: str, n_cmds: int = 2) -> Path:
    d = stress / run_dir / board
    d.mkdir(parents=True, exist_ok=True)
    lines = ["#!/bin/bash", "set -e"]
    for i in range(n_cmds):
        lines.append(f"# cwd={cwd}")
        lines.append(f"python3 route.py in{i}.kicad_pcb out{i}.kicad_pcb")
    m = d / "redo_commands.sh"
    m.write_text("\n".join(lines) + "\n")
    return m


def main():
    with tempfile.TemporaryDirectory() as td:
        stress = Path(td)
        good = _manifest(stress, "runs_set9", "good_board",
                         f"{stress}/runs_set9/good_board")
        bad = _manifest(stress, "runs_set9", "scratch_board",
                        "/private/tmp/somewhere/scratchpad/route_scratch_board")
        # an older run dir of the same set holds a stale copy of good_board and
        # a board of its own; the PREFERRED (plain-named) dir wins for good_board
        _manifest(stress, "runs_set9_llm0801", "good_board",
                  f"{stress}/runs_set9_llm0801/good_board", n_cmds=9)
        only_old = _manifest(stress, "runs_set9_llm0801", "old_only",
                             f"{stress}/runs_set9_llm0801/old_only")
        os.utime(stress / "runs_set9_llm0801", (1, 1))   # older than the plain dir
        (stress / "runs_set9" / "not_a_board").mkdir()   # no manifest: not a board

        print("manifest_preflight")
        bad_rows = crs.manifest_preflight(["set9"], stress)
        check([(s, b) for s, b, _m in bad_rows] == [("set9", "scratch_board")],
              f"only the scratchpad-cwd board is refused (got {[(s, b) for s, b, _ in bad_rows]})")
        check(bad_rows and bad_rows[0][2] == str(bad),
              "the refusal names the manifest path to fix")
        check(crs.manifest_preflight(["set8"], stress) == [],
              "a set with no run dir refuses nothing")

        print("local_manifests")
        local = crs.local_manifests(["set9"], stress)
        check(set(local) == {("set9", "runs_set9", "good_board"),
                             ("set9", "runs_set9", "scratch_board"),
                             ("set9", "runs_set9_llm0801", "old_only")},
              f"one manifest per board, preferred run dir first (got {sorted(local)})")
        check(local[("set9", "runs_set9", "good_board")] == good,
              "good_board comes from the plain run dir, not the older copy")

        print("manifest_sizes_from_entries")
        entries = [("runs_set9/good_board/redo_commands.sh", good.stat().st_size),
                   ("runs_set9/good_board/attempt_1", 424),
                   ("runs_set9/good_board/attempt_1/redo_commands.sh", 999),   # depth 4: ignored
                   ("/runs_set9/scratch_board/redo_commands.sh", 1853),
                   ("runs_set9_llm0801/old_only/redo_commands.sh", only_old.stat().st_size)]
        sizes = crs.manifest_sizes_from_entries(entries)
        check(sizes == {("runs_set9", "good_board"): good.stat().st_size,
                        ("runs_set9", "scratch_board"): 1853,
                        ("runs_set9_llm0801", "old_only"): only_old.stat().st_size},
              f"manifests at depth run_dir/board only, leading slash tolerated (got {sizes})")

        print("stale_manifests")
        stale, absent = crs.stale_manifests(local, sizes)
        check([(s, b, n, h) for s, b, n, h in stale] == [("set9", "scratch_board", bad.stat().st_size, 1853)],
              f"the size mismatch is stale, with both sizes (got {stale})")
        check(absent == [], f"nothing absent when every board is listed (got {absent})")
        stale2, absent2 = crs.stale_manifests(local, {})
        check(stale2 == [] and sorted(b for _s, b, _n in absent2) == ["good_board", "old_only", "scratch_board"],
              f"an empty listing makes every board absent, none stale (got {absent2})")
        stale3, absent3 = crs.stale_manifests(
            local, {(rd, b): m.stat().st_size for (_s, rd, b), m in local.items()})
        check(stale3 == [] and absent3 == [], "sizes that all match report fresh")
        # a NEGATIVE control: a board fresh by size, then one byte appended
        # locally -> stale, with the two sizes one apart
        key = ("set9", "runs_set9", "scratch_board")
        fresh_remote = {("runs_set9", "scratch_board"): bad.stat().st_size}
        check(crs.stale_manifests({key: bad}, fresh_remote) == ([], []),
              "the control is fresh before the edit")
        bad.write_text(bad.read_text() + "\n")
        stale4, _ = crs.stale_manifests({key: bad}, fresh_remote)
        check(len(stale4) == 1 and stale4[0][2] == stale4[0][3] + 1,
              f"a one-byte local edit is reported as stale (got {stale4})")

    if FAILS:
        print(f"\nFAIL: {len(FAILS)} check(s)")
        for f in FAILS:
            print("  - " + f)
        return 1
    print("\nall checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
