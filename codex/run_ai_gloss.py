#!/usr/bin/env python3
"""Run an independent Codex PCB-gloss oracle in an isolated workspace."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


def resource_path(relative):
    root = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    return root / relative


def parser():
    result = argparse.ArgumentParser(
        description="Run the isolated Codex KiCad-gloss oracle.")
    result.add_argument("--board", required=True)
    result.add_argument("--project")
    result.add_argument("--rules")
    result.add_argument("--scope-file", required=True)
    result.add_argument("--output", required=True)
    result.add_argument("--codex-command", default="codex")
    result.add_argument("--kicad-cli", default="kicad-cli")
    result.add_argument("--kicad-python", default="")
    result.add_argument(
        "--prompt", default=str(resource_path("prompts/kicad_ai_gloss.md")))
    return result


def existing_file(value, suffix, label):
    path = Path(value).resolve()
    if path.suffix.lower() != suffix or not path.is_file():
        raise ValueError("{} must be an existing {} file: {}".format(
            label, suffix, path))
    return path


def load_scope(path):
    path = existing_file(path, ".json", "scope manifest")
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError("invalid scope JSON: " + str(error))
    scopes = document.get("scopes") if isinstance(document, dict) else None
    if not isinstance(scopes, list) or not scopes:
        raise ValueError('scope manifest must contain a non-empty "scopes" array')
    pattern = re.compile(r"^(ALL|net:.+|segment:[0-9A-Fa-f-]+)$")
    if any(not isinstance(value, str) or not pattern.fullmatch(value)
           for value in scopes):
        raise ValueError("invalid scope entry")
    if "ALL" in scopes and len(scopes) != 1:
        raise ValueError("ALL cannot be combined with narrower scopes")
    if len(scopes) != len(set(scopes)):
        raise ValueError("scope entries must be unique")
    return document


def copy_project_family(board, project, rules, directory, stem):
    directory.mkdir(parents=True)
    target_board = directory / (stem + ".kicad_pcb")
    shutil.copy2(board, target_board)
    target_project = None
    target_rules = None
    if project:
        target_project = directory / (stem + ".kicad_pro")
        shutil.copy2(project, target_project)
    if rules:
        target_rules = directory / (stem + ".kicad_dru")
        shutil.copy2(rules, target_rules)
    return target_board, target_project, target_rules


def file_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def render_prompt(template_path, replacements):
    text = Path(template_path).read_text(encoding="utf-8")
    for key, value in replacements.items():
        text = text.replace("{{" + key + "}}", str(value or "not supplied"))
    unresolved = re.findall(r"\{\{[A-Z_]+\}\}", text)
    if unresolved:
        raise ValueError("unresolved prompt placeholders: " + str(unresolved))
    return text


def drc_summary(kicad_cli, board, report):
    command = [
        str(kicad_cli), "pcb", "drc", "--format", "json",
        "--all-track-errors", "--output", str(report), str(board),
    ]
    completed = subprocess.run(
        command, capture_output=True, text=True, encoding="utf-8",
        errors="replace")
    if completed.returncode != 0 or not report.is_file():
        detail = (completed.stdout + "\n" + completed.stderr).strip()
        raise RuntimeError("KiCad DRC could not run: " + detail[-500:])
    document = json.loads(report.read_text(encoding="utf-8"))
    categories = Counter(
        row.get("type", "unknown") for row in document.get("violations", []))
    return {
        "violations": sum(categories.values()),
        "unconnected_items": len(document.get("unconnected_items", [])),
        "categories": dict(sorted(categories.items())),
    }


def new_drc_categories(before, after):
    keys = set(before["categories"]) | set(after["categories"])
    return {
        key: after["categories"].get(key, 0) - before["categories"].get(key, 0)
        for key in sorted(keys)
        if after["categories"].get(key, 0) > before["categories"].get(key, 0)
    }


def run(args):
    board = existing_file(args.board, ".kicad_pcb", "board")
    project = (existing_file(args.project, ".kicad_pro", "project")
               if args.project else None)
    rules = (existing_file(args.rules, ".kicad_dru", "design rules")
             if args.rules else None)
    scope = load_scope(args.scope_file)
    output = Path(args.output).resolve()
    if output.suffix.lower() != ".kicad_pcb":
        raise ValueError("output must end in .kicad_pcb")
    if output == board:
        raise ValueError("output cannot overwrite the input board")
    if output.exists():
        raise ValueError("output already exists: " + str(output))
    if not output.parent.is_dir():
        raise ValueError("output directory does not exist: " + str(output.parent))

    transcript_path = Path(str(output) + ".codex.jsonl")
    result_path = Path(str(output) + ".codex-result.json")
    occupied = [path for path in (transcript_path, result_path) if path.exists()]
    if occupied:
        raise ValueError("sidecar already exists: " + str(occupied[0]))
    with tempfile.TemporaryDirectory(prefix="kicad-ai-gloss-") as temporary:
        work = Path(temporary)
        source_board, source_project, source_rules = copy_project_family(
            board, project, rules, work / "source", "source")
        source_digest = file_digest(source_board)
        candidate_board, candidate_project, candidate_rules = copy_project_family(
            board, project, rules, work / "candidate", "candidate")
        scope_path = work / "scope.json"
        scope_path.write_text(
            json.dumps(scope, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        prompt = render_prompt(args.prompt, {
            "BOARD_FILE": source_board,
            "OUTPUT_FILE": candidate_board,
            "PROJECT_FILE": candidate_project,
            "RULES_FILE": candidate_rules,
            "SCOPE_FILE": scope_path,
            "KICAD_CLI": args.kicad_cli,
            "KICAD_PYTHON": args.kicad_python,
        })
        subprocess.run(
            ["git", "init", "--quiet"], cwd=work, check=True,
            capture_output=True)
        command = [
            args.codex_command, "exec", "--ephemeral",
            "--sandbox", "workspace-write", "--ignore-user-config",
            "--ignore-rules", "--json", "-",
        ]
        completed = subprocess.run(
            command, cwd=work, input=prompt, capture_output=True, text=True,
            encoding="utf-8", errors="replace")
        transcript_path.write_text(completed.stdout, encoding="utf-8")
        if completed.returncode != 0:
            raise RuntimeError(
                "Codex agent failed with exit {}: {}".format(
                    completed.returncode, completed.stderr.strip()[-500:]))
        if file_digest(source_board) != source_digest:
            raise RuntimeError("agent modified the isolated source-board copy")
        if not candidate_board.is_file() or not candidate_board.read_text(
                encoding="utf-8", errors="strict").lstrip().startswith("(kicad_pcb"):
            raise RuntimeError("agent did not produce a valid KiCad board file")

        before = drc_summary(
            args.kicad_cli, source_board, work / "drc-before.json")
        after = drc_summary(
            args.kicad_cli, candidate_board, work / "drc-after.json")
        new_categories = new_drc_categories(before, after)
        unconnected_delta = (
            after["unconnected_items"] - before["unconnected_items"])
        valid = not new_categories and unconnected_delta <= 0
        report = {
            "valid": valid,
            "input": str(board),
            "output": str(output),
            "scope": scope,
            "agent_exit_code": completed.returncode,
            "drc_before": before,
            "drc_after": after,
            "new_drc_categories": new_categories,
            "unconnected_delta": unconnected_delta,
        }
        result_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        if not valid:
            raise RuntimeError(
                "candidate rejected by deterministic DRC comparison; see " +
                str(result_path))
        shutil.copy2(candidate_board, output)
        return report


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        report = run(args)
    except Exception as error:
        print("kicad-ai-gloss: {}: {}".format(
            type(error).__name__, error), file=sys.stderr)
        return 1
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
