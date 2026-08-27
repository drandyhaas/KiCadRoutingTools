from collections import Counter
from pathlib import Path
import threading

import pytest

from kicad_track_gloss.engine.model import AddedSegment, GlossResult
from kicad_track_gloss.kicad import native_validation


class _FakePcbnew:
    @staticmethod
    def SaveBoard(path, _board):
        Path(path).write_text("baseline", encoding="utf-8")
        return True


class _FakeAdapter:
    pcbnew = _FakePcbnew()


def _plan(label):
    return GlossResult(
        remove_keys=[label],
        additions=[AddedSegment((0.0, 0.0), (1.0, 0.0), 0.2, 0, 1)])


def test_native_ladder_rejects_more_than_two_candidates():
    with pytest.raises(ValueError, match="one or two"):
        native_validation.validate_native_plan_ladder(
            None, None, [_plan("a"), _plan("b"), _plan("c")],
            skip_native=True)


def test_native_ladder_validates_both_candidates_in_one_parallel_wave(
        monkeypatch):
    native_validation._baseline_cache.clear()
    native_validation._validation_cache.clear()
    candidate_barrier = threading.Barrier(2)
    calls = []

    monkeypatch.setattr(
        native_validation, "_is_strict_removal_only_plan",
        lambda *_args: False)
    monkeypatch.setattr(
        native_validation, "_copy_project_files", lambda *_args: None)
    monkeypatch.setattr(
        native_validation, "_state_digest", lambda *_args: b"baseline")

    def apply_plan(_adapter, _baseline, candidate, _plan_path,
                   timeout_seconds=None):
        Path(candidate).write_text("candidate", encoding="utf-8")

    def run_drc(_adapter, board_path, _report_path, timeout_seconds=None,
                cancel_event=None):
        name = Path(board_path).stem
        calls.append(name)
        if name.startswith("candidate-"):
            candidate_barrier.wait(timeout=1.0)
        counts = Counter(clearance=1) if name == "candidate-0" else Counter()
        return counts, Counter()

    monkeypatch.setattr(native_validation, "_apply_plan_process", apply_plan)
    monkeypatch.setattr(native_validation, "_run_drc", run_drc)
    monkeypatch.setattr(
        native_validation, "_drc_increases",
        lambda _before, after, _before_fp, _after_fp:
        ({"clearance": 1} if after.get("clearance") else {}))

    results = native_validation.validate_native_plan_ladder(
        _FakeAdapter(), object(), [_plan("primary"), _plan("fallback")],
        timeout_seconds=2.0)

    assert [result.allowed for result in results] == [False, True]
    assert all(result.validation_mode == "native_portfolio"
               for result in results)
    assert set(calls) == {"baseline", "candidate-0", "candidate-1"}
    assert results[0].before == results[1].before == {}


def test_native_ladder_cancels_fallback_when_primary_is_allowed(monkeypatch):
    native_validation._baseline_cache.clear()
    native_validation._validation_cache.clear()
    fallback_started = threading.Event()
    fallback_cancelled = threading.Event()

    monkeypatch.setattr(
        native_validation, "_is_strict_removal_only_plan",
        lambda *_args: False)
    monkeypatch.setattr(
        native_validation, "_copy_project_files", lambda *_args: None)
    monkeypatch.setattr(
        native_validation, "_state_digest", lambda *_args: b"baseline")

    def apply_plan(_adapter, _baseline, candidate, _plan_path,
                   timeout_seconds=None):
        Path(candidate).write_text("candidate", encoding="utf-8")

    def run_drc(_adapter, board_path, _report_path, timeout_seconds=None,
                cancel_event=None):
        name = Path(board_path).stem
        if name == "candidate-1":
            fallback_started.set()
            assert cancel_event.wait(timeout=1.0)
            fallback_cancelled.set()
            raise native_validation._NativeDrcCancelled()
        if name == "candidate-0":
            assert fallback_started.wait(timeout=1.0)
        return Counter(), Counter()

    monkeypatch.setattr(native_validation, "_apply_plan_process", apply_plan)
    monkeypatch.setattr(native_validation, "_run_drc", run_drc)

    results = native_validation.validate_native_plan_ladder(
        _FakeAdapter(), object(), [_plan("primary"), _plan("fallback")],
        timeout_seconds=2.0)

    assert results[0].allowed
    assert results[1].validation_mode == "not_needed"
    assert fallback_cancelled.is_set()
