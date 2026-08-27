"""Run the Track Gloss CLI sequentially over a directory of KiCad projects.

The benchmark keeps the canonical CLI JSON and process logs for every project,
then writes a compact CSV/Markdown table suitable for later A/B comparisons.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


REPORT_COLUMNS = (
    "project",
    "planned_saved_mm",
    "planned_angle_corrections",
    "planned_segments_reduced",
    "elapsed_seconds",
    "planning_seconds",
    "native_seconds",
    "passes",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_version(cli: Path) -> dict[str, object]:
    repository = cli.parent.parent

    def git(*arguments: str) -> str:
        process = subprocess.run(
            ["git", *arguments], cwd=repository, capture_output=True,
            text=True, encoding="utf-8", check=True,
        )
        return process.stdout.strip()

    try:
        return {
            "repository": str(repository),
            "commit": git("rev-parse", "HEAD"),
            "branch": git("branch", "--show-current"),
            "describe": git("describe", "--always", "--dirty"),
            "dirty": bool(git("status", "--porcelain")),
        }
    except (OSError, subprocess.CalledProcessError):
        return {"repository": str(repository), "available": False}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("corpus", type=Path, help="directory containing .kicad_pro projects")
    parser.add_argument("output", type=Path, help="new directory for benchmark artifacts")
    parser.add_argument("--kicad-python", type=Path, required=True)
    parser.add_argument(
        "--cli",
        type=Path,
        default=Path(__file__).with_name("score_track_gloss.py"),
    )
    parser.add_argument("--max-passes", type=int, default=16)
    parser.add_argument(
        "--minimum-saved-length-mm", type=float, default=None, metavar="MM",
        help="override the shared Track Gloss minimum saving for this run",
    )
    parser.add_argument(
        "--exclude", action="append", default=[], metavar="PROJECT",
        help="same-stem project name to omit; repeat for multiple projects",
    )
    return parser.parse_args()


def markdown_table(rows: list[dict[str, object]]) -> str:
    labels = ("Projet", "mm planifies", "Angles corriges",
              "Segments reduits", "Temps (s)", "Planning (s)",
              "DRC (s)", "Passes")
    lines = [
        "# Benchmark Track Gloss\n",
        "| " + " | ".join(labels) + " |",
        "| " + " | ".join("---" for _ in labels) + " |",
    ]
    for row in rows:
        lines.append(
            "| {project} | {planned_saved_mm:.6f} | "
            "{planned_angle_corrections} | {planned_segments_reduced} | "
            "{elapsed_seconds:.3f} | {planning_seconds:.3f} | "
            "{native_seconds:.3f} | {passes} |".format(**row)
        )
    if rows:
        lines.append(
            "| **TOTAL** | **{:.6f}** | **{}** | **{}** | **{:.3f}** | "
            "**{:.3f}** | **{:.3f}** | — |".format(
                sum(float(row["planned_saved_mm"]) for row in rows),
                sum(int(row["planned_angle_corrections"]) for row in rows),
                sum(int(row["planned_segments_reduced"]) for row in rows),
                sum(float(row["elapsed_seconds"]) for row in rows),
                sum(float(row["planning_seconds"]) for row in rows),
                sum(float(row["native_seconds"]) for row in rows),
            )
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    corpus = args.corpus.resolve()
    output = args.output.resolve()
    cli = args.cli.resolve()
    kicad_python = args.kicad_python.resolve()
    if output.exists():
        raise SystemExit(f"output directory already exists: {output}")
    if args.max_passes < 1:
        raise SystemExit("--max-passes must be at least one")
    if (args.minimum_saved_length_mm is not None and
            (not math.isfinite(args.minimum_saved_length_mm) or
             args.minimum_saved_length_mm < 0.0)):
        raise SystemExit(
            "--minimum-saved-length-mm must be finite and non-negative")

    excluded = set(args.exclude)
    projects = sorted(
        (path for path in corpus.glob("*.kicad_pro") if path.stem not in excluded),
        key=lambda path: path.name.lower(),
    )
    if not projects:
        raise SystemExit(f"no .kicad_pro projects found in: {corpus}")
    missing_boards = [path.with_suffix(".kicad_pcb") for path in projects
                      if not path.with_suffix(".kicad_pcb").is_file()]
    if missing_boards:
        raise SystemExit("missing same-stem boards: " + ", ".join(map(str, missing_boards)))

    raw = output / "raw"
    raw.mkdir(parents=True)
    rows: list[dict[str, object]] = []
    failures: list[dict[str, object]] = []
    resolved_minimums: set[float] = set()
    started_utc = datetime.now(timezone.utc)

    for index, project in enumerate(projects, 1):
        result_json = raw / f"{project.stem}.json"
        command = [
            str(kicad_python), str(cli), str(project),
            "--max-passes", str(args.max_passes),
            "--json-out", str(result_json),
        ]
        if args.minimum_saved_length_mm is not None:
            command.extend([
                "--minimum-saved-length-mm",
                str(args.minimum_saved_length_mm),
            ])
        print(f"[{index}/{len(projects)}] {project.stem}", flush=True)
        started = time.perf_counter()
        process = subprocess.run(command, capture_output=True, text=True, encoding="utf-8")
        elapsed = time.perf_counter() - started
        (raw / f"{project.stem}.stdout.txt").write_text(process.stdout, encoding="utf-8")
        (raw / f"{project.stem}.stderr.txt").write_text(process.stderr, encoding="utf-8")

        if process.returncode != 0 or not result_json.is_file():
            failures.append({
                "project": project.stem,
                "returncode": process.returncode,
                "elapsed_seconds": round(elapsed, 6),
                "command": command,
            })
            continue
        payload = json.loads(result_json.read_text(encoding="utf-8"))
        timings = payload.get("timings_ms", {})
        native_timings = payload.get("native_drc_timings_ms", {})
        resolved_minimum = payload.get(
            "minimum_saved_length_mm", args.minimum_saved_length_mm)
        if resolved_minimum is not None:
            resolved_minimums.add(float(resolved_minimum))
        measured_native_ms = (
            float(timings.get("native_portfolio", 0.0)) +
            float(timings.get("native_primary", 0.0)) +
            float(timings.get("native_fallback", 0.0)))
        if measured_native_ms <= 0.0:
            # Backward compatibility with CLI payloads predating phase-level
            # timings. Their single native report is still the best measure.
            measured_native_ms = float(native_timings.get("total", 0.0))
        rows.append({
            "project": project.stem,
            "planned_saved_mm": float(payload.get(
                "planned_saved_mm", payload["potential_saved_mm"])),
            "planned_angle_corrections": int(payload.get(
                "planned_angle_corrections", 0)),
            "planned_segments_reduced": int(
                payload.get("planned_removed", 0) -
                payload.get("planned_added", 0)
                if "planned_removed" in payload else payload["segments_saved"]),
            "elapsed_seconds": round(elapsed, 6),
            "planning_seconds": round(
                (float(timings.get("planning_primary", 0.0)) +
                 float(timings.get("planning_fallback", 0.0))) / 1000.0, 6),
            "native_seconds": round(
                measured_native_ms / 1000.0, 6),
            "passes": int(payload["convergence_passes"]),
        })

    with (output / "report.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=REPORT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    (output / "report.md").write_text(markdown_table(rows), encoding="utf-8")
    metadata = {
        "schema": 1,
        "started_utc": started_utc.isoformat(),
        "finished_utc": datetime.now(timezone.utc).isoformat(),
        "corpus": str(corpus),
        "cli": str(cli),
        "engine_git": git_version(cli),
        "kicad_python": str(kicad_python),
        "driver_python": sys.version,
        "platform": platform.platform(),
        "max_passes": args.max_passes,
        "minimum_saved_length_mm": (
            next(iter(resolved_minimums))
            if len(resolved_minimums) == 1 else sorted(resolved_minimums)),
        "time_budget_seconds": None,
        "board_execution": "sequential",
        "engine_parallelism": "enabled",
        "project_count": len(projects),
        "successful_projects": len(rows),
        "excluded_projects": sorted(excluded),
        "input_sha256": {
            project.stem: {
                "kicad_pro": sha256(project),
                "kicad_pcb": sha256(project.with_suffix(".kicad_pcb")),
            }
            for project in projects
        },
        "failures": failures,
    }
    (output / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"report: {output / 'report.csv'}", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
