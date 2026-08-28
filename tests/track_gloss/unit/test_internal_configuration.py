import json
from pathlib import Path

import pytest

from kicad_track_gloss.configuration import (
    CONFIG, get_session_config, load_internal_config, reset_session_config,
    update_session_config)
from kicad_track_gloss.kicad.native_validation import validate_native_plan


def test_current_internal_policy_preserves_release_behavior():
    assert CONFIG.schema_version == 1
    assert CONFIG.gloss.minimum_saved_length_mm == pytest.approx(0.2)
    assert CONFIG.convergence.interactive_group_max_passes == 2
    assert CONFIG.convergence.cli_max_passes == 16
    assert CONFIG.timing.interactive_total_time_budget_seconds == 20.0
    assert CONFIG.timing.interactive_planning_time_budget_seconds == 10.0
    assert CONFIG.timing.interactive_cancellation_grace_seconds == 1.0
    assert CONFIG.timing.cli_total_time_budget_seconds is None
    assert CONFIG.safety.kicad_drc_for_single_track is True


def test_internal_policy_rejects_wrong_types(tmp_path):
    document = {
        "schema_version": 1,
        "gloss": {"minimum_saved_length_mm": 0.01},
        "convergence": {
            "interactive_group_max_passes": 2,
            "cli_max_passes": 16,
        },
        "timing": {
            "interactive_total_time_budget_seconds": 10.0,
            "interactive_planning_time_budget_seconds": 5.0,
            "interactive_cancellation_grace_seconds": 1.0,
            "cli_total_time_budget_seconds": None,
        },
        "safety": {"kicad_drc_for_single_track": "yes"},
    }
    path = tmp_path / "policy.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="must be a boolean"):
        load_internal_config(path)


def test_single_track_policy_can_explicitly_skip_native_drc():
    result = validate_native_plan(None, None, None, skip_native=True)
    assert result.allowed
    assert result.validation_mode == "single_track_drc_disabled"


def test_native_drc_policy_rejects_conflicting_modes():
    with pytest.raises(ValueError, match="both forced and skipped"):
        validate_native_plan(
            None, None, None, force_native=True, skip_native=True)


def test_internal_policy_rejects_planning_budget_above_total(tmp_path):
    config_path = Path("kicad_track_gloss/internal_config.json")
    document = json.loads(config_path.read_text(encoding="utf-8"))
    document["timing"]["interactive_planning_time_budget_seconds"] = 21.0
    path = tmp_path / "policy.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="cannot exceed"):
        load_internal_config(path)


def test_session_policy_changes_are_validated_and_not_persisted():
    try:
        changed = update_session_config(
            minimum_saved_length_mm=0.025,
            interactive_group_max_passes=3,
            interactive_total_time_budget_seconds=20.0,
            interactive_planning_time_budget_seconds=12.0,
            interactive_cancellation_grace_seconds=1.5,
            kicad_drc_for_single_track=False)
        assert get_session_config() is changed
        assert changed.gloss.minimum_saved_length_mm == pytest.approx(0.025)
        assert changed.convergence.interactive_group_max_passes == 3
        assert changed.timing.interactive_total_time_budget_seconds == 20.0
        assert not changed.safety.kicad_drc_for_single_track
        assert CONFIG.gloss.minimum_saved_length_mm == pytest.approx(0.2)
        with pytest.raises(ValueError, match="cannot exceed"):
            update_session_config(
                minimum_saved_length_mm=0.01,
                interactive_group_max_passes=2,
                interactive_total_time_budget_seconds=5.0,
                interactive_planning_time_budget_seconds=6.0,
                interactive_cancellation_grace_seconds=1.0,
                kicad_drc_for_single_track=True)
    finally:
        reset_session_config()
