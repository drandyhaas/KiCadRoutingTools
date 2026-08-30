#!/usr/bin/env python3
"""Merge independently-launched cloud_replay_sets waves into ONE sweep json,
so `modal_sweep/rank_arms.py` can score them as arms of a single experiment.

    python3 tests/stress/cloud_arms_to_sweep.py \
        <ctrl_wave_dir> <arm_wave_dir> [...] --out sweep_dirs.json
    python3 tests/stress/modal_sweep/rank_arms.py sweep_dirs.json --drop-rescue-clean

Why this exists
---------------
`modal_app` writes a sweep json holding every arm's rows, and `rank_arms` is
the only tool that turns those into the verdict the RUNBOOK actually trusts:
paired on boards that replayed an IDENTICAL chain, scored on `nets_incomplete`
alone, with the rescue-clean cell disclosed (RUNBOOK rules 3, 4, 5).

A knob screened with `cloud_replay_sets.py` does NOT produce that file. Each
arm is a separate invocation landing in its own wave dir, and the tools that
read wave dirs (`ab_wave_report.py`, `--compare`) do a two-wave DRC/incomplete
roll-up with no chain pairing, no rescue cell, and no `--hard` split. This
bridges the two: same rows, arranged the way rank_arms expects.

The merge is not a concatenation, because the two halves of a wave carry
different fields:

* `<wave>/<set>/summary.json` holds the GRADING after harvest re-graded the
  kept boards locally (`drc_real` via kicad-cli, the corrected net census).
* `<wave>/_raw/<arm>/<set>__<board>.json` holds the PROVENANCE the regrade
  drops on the floor -- `arm`, `steps`, `rescue_steps`, `patched_defaults`,
  `env_overrides`, and the perf counters.

Take grading from the first and provenance from the second. Feeding rank_arms
the regraded rows alone would silently disable its chain-identity guard (no
`steps`), its arm identification (no `arm`/`patched_defaults`) and its rescue
cell (no `rescue_steps`) -- i.e. exactly the three checks that make the number
mean anything.

Grader mixing
-------------
Harvest re-grades locally, but a board whose artifact was not kept is re-added
to summary.json marked `regraded: False` and keeps its CLOUD grade -- and the
cloud image has no kicad-cli, so its `drc_real` falls back to raw DRC. Pairing
a locally-graded row in one arm against a cloud-graded row in another measures
the GRADER, which is RUNBOOK rule 2 and has produced a sign error before. Such
rows are dropped by default (`--keep-cloud-graded` opts out); the count is
always printed.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Provenance fields the local regrade drops; rank_arms needs the first four.
CARRY = ("arm", "steps", "rescue_steps", "patched_defaults", "env_overrides",
         "set", "git", "total_seconds", "cpu_seconds", "total_iterations",
         "peak_rss_mb")


def arm_of(wave: Path) -> str:
    """The arm name this wave was launched under (cloud_replay_sets writes it)."""
    stamp = wave / "arm.txt"
    if stamp.exists() and stamp.read_text().strip():
        return stamp.read_text().strip()
    # Fall back to the _raw subdir, which is named after the arm.
    raw = wave / "_raw"
    subs = [p.name for p in raw.iterdir() if p.is_dir()] if raw.is_dir() else []
    if len(subs) == 1:
        return subs[0]
    raise SystemExit(f"{wave}: cannot determine the arm name (no arm.txt, "
                     f"_raw holds {subs or 'nothing'})")


def raw_rows(wave: Path, arm: str) -> dict:
    """{(set, board): raw row} for this wave, or {} if _raw was not kept."""
    out = {}
    raw = wave / "_raw"
    for root in (raw / arm, raw):
        if not root.is_dir():
            continue
        for jf in sorted(root.glob("*.json")):
            try:
                row = json.loads(jf.read_text())
            except Exception:
                continue
            row = row[0] if isinstance(row, list) else row
            board = row.get("board")
            s = row.get("set") or jf.name.split("__")[0]
            if board:
                out.setdefault((s, board), row)
    return out


def collect(wave: Path, keep_cloud_graded: bool) -> tuple:
    arm = arm_of(wave)
    raws = raw_rows(wave, arm)
    rows, dropped_cloud, missing_prov = [], [], []
    for sdir in sorted(p for p in wave.iterdir()
                       if p.is_dir() and p.name.startswith("set")):
        summ = sdir / "summary.json"
        if not summ.exists():
            continue
        for r in json.loads(summ.read_text()):
            board = r.get("board")
            if not board:
                continue
            if r.get("regraded") is False and not keep_cloud_graded:
                dropped_cloud.append(f"{sdir.name}/{board}")
                continue
            merged = dict(r)
            prov = raws.get((sdir.name, board))
            if prov:
                for k in CARRY:
                    if k in prov and merged.get(k) is None:
                        merged[k] = prov[k]
            else:
                missing_prov.append(f"{sdir.name}/{board}")
            merged["arm"] = arm
            merged.setdefault("set", sdir.name)
            rows.append(merged)
    return arm, rows, dropped_cloud, missing_prov


def main():
    ap = argparse.ArgumentParser(
        description="Merge cloud_replay_sets wave dirs into a rank_arms sweep json")
    ap.add_argument("waves", nargs="+", help="wave dirs; the CONTROL may be any of them "
                                             "(rank_arms finds it by empty overrides)")
    ap.add_argument("--out", required=True, help="sweep json to write")
    ap.add_argument("--keep-cloud-graded", action="store_true",
                    help="keep rows the local regrade could not re-score. They were "
                         "graded WITHOUT kicad-cli, so pairing them against a "
                         "locally-graded arm compares graders, not engines.")
    a = ap.parse_args()

    all_rows, arms = [], []
    for w in a.waves:
        wave = Path(w).expanduser()
        if not wave.is_dir():
            raise SystemExit(f"not a directory: {wave}")
        arm, rows, dropped, missing = collect(wave, a.keep_cloud_graded)
        arms.append(arm)
        all_rows += rows
        note = ""
        if dropped:
            note += f", {len(dropped)} cloud-graded DROPPED ({', '.join(dropped[:3])}" \
                    f"{'...' if len(dropped) > 3 else ''})"
        if missing:
            note += f", {len(missing)} without _raw provenance " \
                    f"({', '.join(missing[:3])}{'...' if len(missing) > 3 else ''})"
        print(f"{arm:28} {len(rows):4} rows from {wave}{note}")

    if len(set(arms)) != len(arms):
        raise SystemExit(f"two waves report the same arm name ({arms}) -- rank_arms "
                         f"would merge them into one arm")
    no_steps = [r for r in all_rows if r.get("steps") is None]
    if no_steps:
        print(f"WARN {len(no_steps)} row(s) carry no step list; rank_arms cannot "
              f"chain-pair those and will drop them", file=sys.stderr)

    Path(a.out).expanduser().write_text(
        json.dumps({"rows": all_rows, "arms": arms}, indent=1))
    print(f"\nwrote {a.out}: {len(all_rows)} rows, {len(arms)} arms\n"
          f"  python3 tests/stress/modal_sweep/rank_arms.py {a.out} --drop-rescue-clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
