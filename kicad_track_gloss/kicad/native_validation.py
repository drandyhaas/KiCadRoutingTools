"""One-shot KiCad-native DRC gate for a composed Track Gloss plan."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


@dataclass
class NativeDrcResult:
    allowed: bool
    before: dict = field(default_factory=dict)
    after: dict = field(default_factory=dict)
    increases: dict = field(default_factory=dict)
    error: str = ""


def _copy_project_files(source_board, target_board):
    source = Path(str(source_board.GetFileName()))
    for suffix in (".kicad_pro", ".kicad_dru"):
        candidate = source.with_suffix(suffix)
        if candidate.is_file():
            shutil.copy2(candidate, target_board.with_suffix(suffix))


def _json_report_counts(text):
    return _json_report_summary(text)[0]


def _normalized_text(value):
    value = re.sub(r"\blength\s+[0-9.]+\s+mm\b", "length", str(value))
    return re.sub(r"\s+", " ", value).strip()


def _json_report_summary(text):
    document = json.loads(text)
    records = list(document.get("violations", ())) + list(
        document.get("unconnected_items", ()))
    counts = Counter()
    fingerprints = Counter()
    for record in records:
        kind = record.get("type", "unknown")
        counts[kind] += 1
        items = []
        for item in record.get("items", ()):
            position = item.get("pos") or {}
            items.append((
                _normalized_text(item.get("description", "")),
                round(float(position.get("x", 0.0)), 3),
                round(float(position.get("y", 0.0)), 3),
            ))
        fingerprints[(kind, _normalized_text(
            record.get("description", "")), tuple(sorted(items)))] += 1
    return counts, fingerprints


def _kicad_cli(adapter):
    suffix = ".exe" if os.name == "nt" else ""
    candidates = []
    module_path = getattr(adapter.pcbnew, "__file__", "")
    if module_path:
        module = Path(module_path).resolve()
        candidates.extend(parent / ("kicad-cli" + suffix)
                          for parent in list(module.parents)[:5])
    resolved = shutil.which("kicad-cli")
    if resolved:
        candidates.append(Path(resolved))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RuntimeError("kicad-cli was not found beside pcbnew or on PATH")


def _kicad_python(adapter):
    suffix = ".exe" if os.name == "nt" else ""
    module_path = getattr(adapter.pcbnew, "__file__", "")
    candidates = []
    if module_path:
        module = Path(module_path).resolve()
        for directory in list(module.parents)[:5]:
            candidates.extend((directory / ("python" + suffix),
                               directory / ("python3" + suffix)))
    candidates.append(Path(sys.executable))
    for candidate in candidates:
        if candidate.is_file() and candidate.name.lower() not in (
                "kicad.exe", "pcbnew.exe"):
            return candidate
    raise RuntimeError("KiCad's Python interpreter was not found")


def _write_plan(path, plan):
    if any(str(key).startswith("geom:") for key in plan.remove_keys):
        raise RuntimeError("native DRC requires KiCad UUID track identities")
    path.write_text(json.dumps({
        "remove_keys": list(plan.remove_keys),
        "additions": [{
            "start": list(item.start), "end": list(item.end),
            "width": item.width, "layer": item.layer,
            "net_id": item.net_id,
        } for item in plan.additions],
    }), encoding="utf-8")


def _apply_plan_process(adapter, baseline_path, candidate_path, plan_path):
    process = subprocess.run([
        str(_kicad_python(adapter)), str(Path(__file__).resolve()),
        "--apply-plan", str(baseline_path), str(candidate_path),
        str(plan_path),
    ], capture_output=True, text=True, timeout=180)
    if process.returncode != 0 or not candidate_path.is_file():
        detail = (process.stderr or process.stdout).strip()[:500]
        raise RuntimeError(
            "candidate snapshot failed (exit {}): {}".format(
                process.returncode, detail))


def _run_drc(adapter, board_path, report_path):
    command = [
        str(_kicad_cli(adapter)), "pcb", "drc", "--format", "json",
        "--severity-all", "--units", "mm", "--refill-zones",
        "--output", str(report_path), str(board_path),
    ]
    process = subprocess.run(
        command, capture_output=True, text=True, timeout=300)
    if process.returncode != 0 or not report_path.is_file():
        detail = (process.stderr or process.stdout).strip()[:500]
        raise RuntimeError(
            "native DRC failed (exit {}): {}".format(
                process.returncode, detail))
    return _json_report_summary(report_path.read_text(
        encoding="utf-8", errors="replace"))


def validate_native_plan(adapter, board, plan):
    """Reject a plan if KiCad reports any increased DRC category.

    Temporary boards and reports are private implementation artifacts and are
    removed before returning.  The current PCB and its zone fills are never
    mutated by this validation.
    """
    try:
        with tempfile.TemporaryDirectory(prefix="kicad-track-gloss-drc-") as name:
            root = Path(name)
            baseline_path = root / "baseline.kicad_pcb"
            candidate_path = root / "candidate.kicad_pcb"
            plan_path = root / "plan.json"
            if not adapter.pcbnew.SaveBoard(str(baseline_path), board):
                raise RuntimeError("KiCad could not snapshot the current board")
            _copy_project_files(board, baseline_path)
            _copy_project_files(board, candidate_path)
            _write_plan(plan_path, plan)
            _apply_plan_process(
                adapter, baseline_path, candidate_path, plan_path)
            before, before_fingerprints = _run_drc(
                adapter, baseline_path, root / "before.rpt")
            after, after_fingerprints = _run_drc(
                adapter, candidate_path, root / "after.rpt")
            new_records = after_fingerprints - before_fingerprints
            increases = Counter()
            for (kind, _description, _items), count in new_records.items():
                increases[kind] += count
            return NativeDrcResult(
                not increases, dict(before), dict(after), dict(increases))
    except Exception as error:
        return NativeDrcResult(False, error=str(error))


def _item_uuid(item):
    try:
        return item.m_Uuid.AsString()
    except Exception:
        return item.GetUuid().AsString()


def _headless_apply(baseline_path, candidate_path, plan_path):
    try:
        import wx
        wx.Log.SetActiveTarget(wx.LogStderr())
    except Exception:
        pass
    import pcbnew

    board = pcbnew.LoadBoard(str(baseline_path))
    plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
    tracks = {
        _item_uuid(item): item for item in board.GetTracks()
        if str(item.GetClass()) == "PCB_TRACK"
    }
    missing = [key for key in plan["remove_keys"] if key not in tracks]
    if missing:
        raise RuntimeError("candidate snapshot is missing planned tracks")
    for key in plan["remove_keys"]:
        board.RemoveNative(tracks[key])
    for addition in plan["additions"]:
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(pcbnew.VECTOR2I(
            int(round(addition["start"][0] * 1_000_000)),
            int(round(addition["start"][1] * 1_000_000))))
        track.SetEnd(pcbnew.VECTOR2I(
            int(round(addition["end"][0] * 1_000_000)),
            int(round(addition["end"][1] * 1_000_000))))
        track.SetWidth(int(round(addition["width"] * 1_000_000)))
        track.SetLayer(int(addition["layer"]))
        track.SetNetCode(int(addition["net_id"]))
        board.Add(track)
    if not pcbnew.SaveBoard(str(candidate_path), board):
        raise RuntimeError("KiCad could not save the candidate snapshot")


if __name__ == "__main__":
    if len(sys.argv) == 5 and sys.argv[1] == "--apply-plan":
        _headless_apply(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        raise SystemExit("expected --apply-plan BASELINE CANDIDATE PLAN")
