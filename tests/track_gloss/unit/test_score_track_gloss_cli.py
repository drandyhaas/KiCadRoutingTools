import importlib.util
import json
from pathlib import Path

import pytest

from kicad_track_gloss.engine.model import Segment


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "score_track_gloss.py"
SPEC = importlib.util.spec_from_file_location("score_track_gloss_cli", SCRIPT)
CLI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLI)


def test_direct_mode_accepts_one_board():
    board, placed, route_json = CLI.resolve_inputs(["candidate.kicad_pcb"])
    assert board == Path("candidate.kicad_pcb")
    assert placed is None
    assert route_json is None


def test_place_route_loop_uses_routed_board():
    board, placed, route_json = CLI.resolve_inputs(
        ["placed.kicad_pcb", "routed.kicad_pcb", "route.json"], True)
    assert board == Path("routed.kicad_pcb")
    assert placed == Path("placed.kicad_pcb")
    assert route_json == Path("route.json")


@pytest.mark.parametrize("paths,loop", [([], False), (["a", "b"], False),
                                         (["a", "b"], True)])
def test_cli_rejects_wrong_positional_count(paths, loop):
    with pytest.raises(ValueError):
        CLI.resolve_inputs(paths, loop)


def test_project_path_resolves_its_sibling_board():
    board, project = CLI.resolve_board_project("design.kicad_pro")
    assert board == Path("design.kicad_pcb")
    assert project == Path("design.kicad_pro")


def test_explicit_project_can_grade_a_differently_named_candidate():
    board, project = CLI.resolve_board_project(
        "candidate.kicad_pcb", "reference.kicad_pro")
    assert board == Path("candidate.kicad_pcb")
    assert project == Path("reference.kicad_pro")


def test_stdout_has_json_then_place_route_loop_score_line():
    output = CLI.score_stdout({"score": 87.5, "changed": True})
    json_line, score_line = output.splitlines()
    assert json.loads(json_line.removeprefix("GLOSS_SCORE_JSON=")) == {
        "changed": True, "score": 87.5}
    assert score_line == "SCORE=87.500000000"


def test_scope_defaults_to_all_and_rejects_ambiguous_mix():
    assert CLI.resolve_scopes() == ["ALL"]
    assert CLI.resolve_scopes([" net:VCC ", "segment:abc"]) == [
        "net:VCC", "segment:abc"]
    with pytest.raises(ValueError):
        CLI.resolve_scopes(["ALL", "net:VCC"])


def test_scope_manifest_is_merged_with_command_line(tmp_path):
    manifest = tmp_path / "scope.json"
    manifest.write_text('{"scopes":["segment:abc"]}', encoding="utf-8")
    assert CLI.resolve_scopes(["net:VCC"], manifest) == [
        "net:VCC", "segment:abc"]


def test_scope_resolves_exact_nets_and_segments():
    records = {
        "a": (None, Segment(0, 0, 1, 0, 0.2, 0, 1, "a", net_name="VCC")),
        "b": (None, Segment(0, 1, 1, 1, 0.2, 0, 2, "b", net_name="GND")),
    }
    not_diff = lambda _name: False
    assert CLI.seed_keys_for_scopes(records, ["net:VCC"], not_diff) == {"a"}
    assert CLI.seed_keys_for_scopes(
        records, ["segment:b"], not_diff) == {"b"}
    with pytest.raises(ValueError):
        CLI.seed_keys_for_scopes(records, ["net:vcc"], not_diff)


def test_cli_exposes_convergence_pass_limit_and_trace():
    args = CLI._parser().parse_args([
        "--max-passes", "7", "--trace-passes", "candidate.kicad_pcb"])
    assert args.max_passes == 7
    assert args.trace_passes
