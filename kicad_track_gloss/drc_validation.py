"""Official KiCad DRC comparison performed on temporary board copies."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from .board_adapter import BoardAdapter


@dataclass
class DrcComparison:
    baseline_count: int
    candidate_count: int
    new_violations: list


def find_kicad_cli():
    found = shutil.which("kicad-cli")
    if found:
        return found
    if os.name == "nt":
        root = Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "KiCad"
        for version in ("10.0", "9.0", "8.0"):
            candidate = root / version / "bin" / "kicad-cli.exe"
            if candidate.exists():
                return str(candidate)
    return None


def _violation_records(data):
    records = []
    if isinstance(data, dict):
        for key, value in data.items():
            if key.lower() in ("violations", "unconnected_items", "schematic_parity") and isinstance(value, list):
                records.extend(item for item in value if isinstance(item, dict))
            else:
                records.extend(_violation_records(value))
    elif isinstance(data, list):
        for value in data:
            records.extend(_violation_records(value))
    return records


def _signature(record):
    items = []
    for item in record.get("items", ()) or ():
        if isinstance(item, dict):
            # New replacement tracks necessarily receive new KIID values. Use
            # semantic item descriptions instead, otherwise an unchanged
            # pre-existing violation is falsely classified as new.
            items.append(str(item.get("description") or item.get("ref") or
                             item.get("type") or ""))
    pos = record.get("pos") or record.get("position") or {}
    if isinstance(pos, dict):
        try:
            position = (round(float(pos.get("x", 0.0)), 3),
                        round(float(pos.get("y", 0.0)), 3))
        except (TypeError, ValueError):
            position = ()
    else:
        position = ()
    return (str(record.get("type") or record.get("rule") or record.get("code") or ""),
            str(record.get("severity") or ""),
            str(record.get("description") or record.get("message") or ""),
            tuple(sorted(items)), position)


def _run(cli, board_path, report_path):
    cmd = [cli, "pcb", "drc", "--format", "json", "--output", str(report_path),
           "--refill-zones", "--exit-code-violations", str(board_path)]
    completed = subprocess.run(cmd, capture_output=True, text=True, timeout=180,
                               creationflags=(subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0))
    if not report_path.exists():
        detail = (completed.stderr or completed.stdout or "no DRC report").strip()
        raise RuntimeError("kicad-cli DRC failed: " + detail)
    try:
        data = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise RuntimeError("KiCad produced an unreadable JSON DRC report") from exc
    return [_signature(record) for record in _violation_records(data)]


def validate_on_copy(pcbnew, board, result):
    """Raise if applying ``result`` adds any official KiCad DRC violation."""
    cli = find_kicad_cli()
    if not cli:
        raise RuntimeError("kicad-cli was not found; disable DRC validation only for an explicit manual test.")
    adapter = BoardAdapter(pcbnew)
    with tempfile.TemporaryDirectory(prefix="kicad-track-gloss-") as tmp:
        tmp = Path(tmp)
        baseline_board = tmp / "baseline.kicad_pcb"
        candidate_board = tmp / "candidate.kicad_pcb"
        pcbnew.SaveBoard(str(baseline_board), board)
        # KiCad resolves project settings and custom .kicad_dru rules by board
        # basename. Preserve them for both temporary copies.
        source_name = str(board.GetFileName() or "")
        if source_name:
            source = Path(source_name)
            for suffix in (".kicad_pro", ".kicad_dru"):
                sidecar = source.with_suffix(suffix)
                if sidecar.exists():
                    shutil.copy2(sidecar, baseline_board.with_suffix(suffix))
                    shutil.copy2(sidecar, candidate_board.with_suffix(suffix))
        candidate = pcbnew.LoadBoard(str(baseline_board))
        adapter.apply(candidate, result, rollback_on_error=False)
        pcbnew.SaveBoard(str(candidate_board), candidate)
        before = _run(cli, baseline_board, tmp / "baseline-drc.json")
        after = _run(cli, candidate_board, tmp / "candidate-drc.json")
        delta = Counter(after) - Counter(before)
        new = [signature for signature, count in delta.items() for _ in range(count)]
        if new:
            raise ValueError("Gloss rejected: official KiCad DRC reports {} new violation(s).".format(len(new)))
        return DrcComparison(len(before), len(after), new)
