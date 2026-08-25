#!/usr/bin/env python3
"""Run Track Gloss and the independent Codex oracle on the stress corpus.

The pipeline is resumable.  It downloads exactly the boards named by the
versioned ``manifest_set*.json`` files, scores every routed board with the
Track Gloss CLI, and runs the expensive oracle only when the CLI saves at
least the configured percentage of straight-track copper.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
DEFAULT_ROOT = Path.home() / "Documents" / "kicad_track_gloss_stress"


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    result.add_argument("--threshold", type=float, default=5.0)
    result.add_argument("--jobs", type=int, default=2,
                        help="parallel board scores; default 2")
    result.add_argument("--download-jobs", type=int, default=12)
    result.add_argument("--timeout", type=int, default=1800,
                        help="per CLI score timeout in seconds")
    result.add_argument("--oracle-timeout", type=int, default=7200)
    result.add_argument("--oracle-limit", type=int, default=0,
                        help="maximum new oracle runs; 0 means all qualifying")
    result.add_argument("--kicad-python", default=r"D:\kicad\bin\python.exe")
    result.add_argument("--kicad-cli", default=r"D:\kicad\bin\kicad-cli.exe")
    result.add_argument("--codex-command", default="codex")
    result.add_argument("--board-id", action="append", default=[],
                        help="limit the run to an exact set/file id; repeatable")
    result.add_argument("--skip-fetch", action="store_true")
    result.add_argument("--skip-score", action="store_true")
    result.add_argument("--skip-oracle", action="store_true")
    return result


def atomic_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8")
    temporary.replace(path)


def corpus_rows():
    rows = []
    for manifest in sorted(HERE.glob("manifest_set*.json")):
        document = json.loads(manifest.read_text(encoding="utf-8"))
        if not isinstance(document, list):
            raise ValueError("manifest must contain an array: " + str(manifest))
        for index, row in enumerate(document):
            item = dict(row)
            item["manifest"] = manifest.name
            item["manifest_index"] = index
            item["set"] = str(item.get("set") or manifest.stem.removeprefix(
                "manifest_"))
            item["file"] = Path(item["file"]).name
            item["id"] = item["set"] + "/" + item["file"]
            rows.append(item)
    identifiers = [row["id"] for row in rows]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("duplicate set/file identifiers in stress manifests")
    return rows


def source_paths(root, row):
    board = root / "sources" / row["set"] / row["file"]
    return board, board.with_suffix(".kicad_pro"), board.with_suffix(".kicad_dru")


def _download(url, destination):
    request = urllib.request.Request(
        url, headers={"User-Agent": "KiCadTrackGlossStress/0.3.24"})
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".part")
    with urllib.request.urlopen(request, timeout=120) as response, \
            temporary.open("wb") as output:
        shutil.copyfileobj(response, output)
    if temporary.stat().st_size == 0:
        raise OSError("empty download")
    temporary.replace(destination)


def fetch_one(root, row):
    board, project, rules = source_paths(root, row)
    if not board.is_file() or board.stat().st_size == 0:
        _download(row["raw_url"], board)
    sidecars = {}
    base_url = row["raw_url"][:-len(".kicad_pcb")]
    for label, suffix, destination in (
            ("project", ".kicad_pro", project),
            ("rules", ".kicad_dru", rules)):
        if destination.is_file():
            sidecars[label] = True
            continue
        try:
            _download(base_url + suffix, destination)
            sidecars[label] = True
        except (OSError, urllib.error.URLError, urllib.error.HTTPError):
            sidecars[label] = False
            part = destination.with_suffix(destination.suffix + ".part")
            if part.exists():
                part.unlink()
    return {
        "id": row["id"], "ok": True, "bytes": board.stat().st_size,
        **sidecars,
    }


def fetch_corpus(root, rows, jobs):
    results = []
    with ThreadPoolExecutor(max_workers=max(1, jobs)) as executor:
        futures = {executor.submit(fetch_one, root, row): row for row in rows}
        for completed, future in enumerate(as_completed(futures), 1):
            row = futures[future]
            try:
                outcome = future.result()
            except Exception as error:
                outcome = {"id": row["id"], "ok": False,
                           "error": "{}: {}".format(type(error).__name__, error)}
            results.append(outcome)
            if completed % 25 == 0 or not outcome["ok"]:
                print("FETCH {}/{} {}".format(
                    completed, len(rows), outcome["id"]), flush=True)
    results.sort(key=lambda row: row["id"])
    atomic_json(root / "fetch-report.json", {"boards": results})
    return results


def project_argument(project):
    return ["--project", str(project)] if project.is_file() else []


def run_process(command, timeout, cwd=REPO):
    started = time.monotonic()
    completed = subprocess.run(
        command, cwd=cwd, capture_output=True, text=True, encoding="utf-8",
        errors="replace", timeout=timeout)
    return completed, time.monotonic() - started


def parse_score(stdout):
    prefix = "GLOSS_SCORE_JSON="
    for line in stdout.splitlines():
        if line.startswith(prefix):
            return json.loads(line[len(prefix):])
    raise ValueError("Track Gloss JSON marker missing from stdout")


def score_one(args, root, row):
    result_path = root / "scores" / row["set"] / (row["file"] + ".json")
    if result_path.is_file():
        cached = json.loads(result_path.read_text(encoding="utf-8"))
        if cached.get("status") == "scored":
            cached["qualifies"] = (
                float(cached["cli_saved_percent"]) + 1e-12 >= args.threshold)
        return cached
    board, project, _rules = source_paths(root, row)
    command = [args.kicad_python, str(REPO / "tools" / "score_track_gloss.py")]
    command += project_argument(project)
    command.append(str(board))
    try:
        completed, seconds = run_process(command, args.timeout)
        if completed.returncode != 0:
            raise RuntimeError((completed.stderr or completed.stdout)[-1000:])
        payload = parse_score(completed.stdout)
        outcome = {
            "id": row["id"], "status": "scored", "seconds": seconds,
            "board": str(board), "project": str(project) if project.is_file() else None,
            "before_mm": payload["before_mm"],
            "cli_after_mm": payload["after_mm"],
            "cli_saved_mm": payload["potential_saved_mm"],
            "cli_saved_percent": payload["potential_saved_percent"],
            "cli_segments_saved": payload["segments_saved"],
            "cli_convergence_passes": payload["convergence_passes"],
            "qualifies": payload["potential_saved_percent"] + 1e-12 >= args.threshold,
        }
    except Exception as error:
        outcome = {"id": row["id"], "status": "score_failed",
                   "error": "{}: {}".format(type(error).__name__, error)}
    atomic_json(result_path, outcome)
    return outcome


def score_corpus(args, root, rows):
    results = []
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as executor:
        futures = {executor.submit(score_one, args, root, row): row for row in rows}
        for completed, future in enumerate(as_completed(futures), 1):
            outcome = future.result()
            results.append(outcome)
            print("SCORE {}/{} {} {}".format(
                completed, len(rows), outcome["id"], outcome["status"]),
                flush=True)
    return sorted(results, key=lambda row: row["id"])


def run_oracle_one(args, root, row, score):
    result_path = root / "oracle-results" / row["set"] / (row["file"] + ".json")
    if result_path.is_file():
        cached = json.loads(result_path.read_text(encoding="utf-8"))
        if (cached.get("status") == "oracle_complete" and
                abs(float(cached.get("oracle_saved_mm", 0.0))) <= 1e-9):
            cached["status"] = "oracle_invalid_no_change"
            cached["error"] = (
                "Oracle returned an unchanged board despite a qualifying CLI "
                "gain; its transcript must be inspected before retrying.")
            atomic_json(result_path, cached)
        return cached, False
    board, project, rules = source_paths(root, row)
    oracle_board = root / "oracle" / row["set"] / row["file"]
    oracle_board.parent.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable, str(REPO / "codex" / "run_ai_gloss.py"),
        "--board", str(board), "--scope-file",
        str(REPO / "codex" / "examples" / "scope-all.json"),
        "--output", str(oracle_board), "--codex-command", args.codex_command,
        "--kicad-cli", args.kicad_cli,
        "--kicad-python", args.kicad_python,
    ]
    if project.is_file():
        command += ["--project", str(project)]
    if rules.is_file():
        command += ["--rules", str(rules)]
    try:
        completed, seconds = run_process(command, args.oracle_timeout)
        if completed.returncode != 0:
            raise RuntimeError((completed.stderr or completed.stdout)[-2000:])
        score_command = [
            args.kicad_python, str(REPO / "tools" / "score_track_gloss.py")]
        score_command += project_argument(project)
        score_command.append(str(oracle_board))
        scored, score_seconds = run_process(score_command, args.timeout)
        if scored.returncode != 0:
            raise RuntimeError((scored.stderr or scored.stdout)[-1000:])
        oracle_score = parse_score(scored.stdout)
        oracle_mm = oracle_score["before_mm"]
        oracle_saved_mm = score["before_mm"] - oracle_mm
        outcome = {
            **score, "status": "oracle_complete",
            "oracle_seconds": seconds, "oracle_score_seconds": score_seconds,
            "oracle_board": str(oracle_board), "oracle_mm": oracle_mm,
            "oracle_saved_mm": oracle_saved_mm,
            "oracle_saved_percent": (
                100.0 * (score["before_mm"] - oracle_mm) / score["before_mm"]
                if score["before_mm"] else 0.0),
            "oracle_vs_cli_mm": oracle_mm - score["cli_after_mm"],
        }
        if abs(oracle_saved_mm) <= 1e-9:
            outcome["status"] = "oracle_invalid_no_change"
            outcome["error"] = (
                "Oracle returned an unchanged board despite a qualifying CLI "
                "gain; inspect the Codex JSONL transcript.")
    except Exception as error:
        outcome = {**score, "status": "oracle_failed",
                   "error": "{}: {}".format(type(error).__name__, error)}
    atomic_json(result_path, outcome)
    return outcome, True


def write_summary(root, rows, threshold):
    document = {
        "schema": 1, "threshold_percent": threshold,
        "corpus_boards": len(rows), "boards": rows,
    }
    atomic_json(root / "track-gloss-oracle-summary.json", document)
    columns = [
        "id", "status", "before_mm", "cli_after_mm", "cli_saved_mm",
        "cli_saved_percent", "cli_segments_saved", "qualifies", "oracle_mm",
        "oracle_saved_mm", "oracle_saved_percent", "oracle_vs_cli_mm", "error",
    ]
    csv_path = root / "track-gloss-oracle-summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as output:
        writer = csv.DictWriter(output, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    args = parser().parse_args(argv)
    if args.threshold < 0.0 or args.threshold > 100.0:
        raise SystemExit("--threshold must be between 0 and 100")
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    rows = corpus_rows()
    if args.board_id:
        requested = set(args.board_id)
        known = {row["id"] for row in rows}
        missing = sorted(requested - known)
        if missing:
            raise SystemExit("unknown --board-id: " + ", ".join(missing))
        rows = [row for row in rows if row["id"] in requested]
    print("CORPUS {} manifest boards -> {}".format(len(rows), root), flush=True)

    if not args.skip_fetch:
        fetched = fetch_corpus(root, rows, args.download_jobs)
        print("FETCHED {}/{}".format(
            sum(bool(row.get("ok")) for row in fetched), len(rows)), flush=True)
    available = [row for row in rows if source_paths(root, row)[0].is_file()]
    if not args.skip_score:
        scores = score_corpus(args, root, available)
    else:
        scores = []
        for row in available:
            path = root / "scores" / row["set"] / (row["file"] + ".json")
            if path.is_file():
                scores.append(json.loads(path.read_text(encoding="utf-8")))
    by_id = {row["id"]: row for row in rows}
    qualifying = [row for row in scores if row.get("qualifies")]
    print("QUALIFYING {} / {} at >= {:.3f}%".format(
        len(qualifying), len(scores), args.threshold), flush=True)

    outcomes = {row["id"]: row for row in scores}
    launched = 0
    if not args.skip_oracle:
        for score in sorted(qualifying, key=lambda row: (
                -row["cli_saved_percent"], row["id"])):
            if args.oracle_limit and launched >= args.oracle_limit:
                break
            outcome, was_new = run_oracle_one(
                args, root, by_id[score["id"]], score)
            outcomes[score["id"]] = outcome
            launched += int(was_new)
            print("ORACLE {} {}".format(score["id"], outcome["status"]),
                  flush=True)
            write_summary(root, sorted(outcomes.values(), key=lambda row: row["id"]),
                          args.threshold)
    final_rows = sorted(outcomes.values(), key=lambda row: row["id"])
    write_summary(root, final_rows, args.threshold)
    print("SUMMARY " + str(root / "track-gloss-oracle-summary.json"))
    return 0 if all(row.get("status") != "score_failed" for row in scores) else 1


if __name__ == "__main__":
    raise SystemExit(main())
