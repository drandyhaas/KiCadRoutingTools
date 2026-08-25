import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "codex" / "run_ai_gloss.py"
SPEC = importlib.util.spec_from_file_location("run_ai_gloss", SCRIPT)
AGENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AGENT)


def test_agent_scope_manifest_contract(tmp_path):
    manifest = tmp_path / "scope.json"
    manifest.write_text('{"scopes":["net:VCC","segment:abc-def"]}',
                        encoding="utf-8")
    assert AGENT.load_scope(manifest)["scopes"] == [
        "net:VCC", "segment:abc-def"]

    manifest.write_text('{"scopes":["ALL","net:VCC"]}', encoding="utf-8")
    with pytest.raises(ValueError):
        AGENT.load_scope(manifest)


def test_agent_drc_gate_reports_only_increases():
    before = {"categories": {"clearance": 2, "edge": 1}}
    after = {"categories": {"clearance": 2, "edge": 0, "shorting": 1}}
    assert AGENT.new_drc_categories(before, after) == {"shorting": 1}


def test_agent_prompt_has_no_unresolved_contract_tokens(tmp_path):
    template = tmp_path / "prompt.md"
    template.write_text("{{BOARD_FILE}} -> {{OUTPUT_FILE}}", encoding="utf-8")
    rendered = AGENT.render_prompt(template, {
        "BOARD_FILE": "A0.kicad_pcb", "OUTPUT_FILE": "Aia.kicad_pcb"})
    assert rendered == "A0.kicad_pcb -> Aia.kicad_pcb"


def test_agent_uses_bounded_native_sandbox_with_local_policy():
    command = AGENT.codex_exec_command("codex")
    assert command == [
        "codex", "--ask-for-approval", "never", "exec", "--ephemeral",
        "--sandbox", "workspace-write", "--json", "-",
    ]
    assert "--ignore-user-config" not in command
    assert "--ignore-rules" not in command


def test_versioned_prompt_forbids_track_gloss_reuse():
    prompt = (ROOT / "codex" / "prompts" / "kicad_ai_gloss.md").read_text(
        encoding="utf-8")
    assert "Do not call, import, inspect, reproduce, or search" in prompt
    assert "{{SCOPE_FILE}}" in prompt
    assert "fixed point" in prompt


def test_agent_json_schemas_are_valid_documents():
    for name in ("scope.schema.json", "result.schema.json"):
        document = json.loads((ROOT / "codex" / "schemas" / name).read_text(
            encoding="utf-8"))
        assert document["$schema"].endswith("2020-12/schema")
