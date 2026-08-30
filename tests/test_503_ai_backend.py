"""#503: AI backend abstraction (Claude Code / opencode).

Headless tests (no wx, no CLI): per-backend argv construction, skill-prompt
phrasing, streaming-event parsing on recorded-shape fixtures for BOTH wire
formats, the RESULT= output contract, auth hints, and extract_narrative's
dual-schema transcript support.
"""
import json
import os
import sys
import types

sys.modules.setdefault('wx', types.ModuleType('wx'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_tools'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'kicad_routing_plugin'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'stress'))

import ai_backend  # noqa: E402
from ai_backend import (  # noqa: E402
    BACKENDS, BACKEND_IDS, DEFAULT_BACKEND_ID, get_backend,
    extract_result_line, summarize_tool_use,
)
import extract_narrative  # noqa: E402

FAILURES = []


def check(name, cond, detail=""):
    status = "ok" if cond else "FAIL"
    print(f"[{status}] {name}" + (f" -- {detail}" if detail and not cond else ""))
    if not cond:
        FAILURES.append(name)


# ------------------------------------------------------------------ registry

check("registry ids", BACKEND_IDS == ("claude", "opencode"))
check("default backend", DEFAULT_BACKEND_ID == "claude")
check("get_backend known", get_backend("opencode").id == "opencode")
check("get_backend unknown falls back", get_backend("gpt-cli").id == "claude")
check("get_backend None falls back", get_backend(None).id == "claude")

claude = BACKENDS["claude"]
oc = BACKENDS["opencode"]

# ---------------------------------------------------------------------- argv

cmd = claude.build_cmd("/usr/bin/claude", "PROMPT", model="opus", effort="high")
check("claude argv shape",
      cmd[:3] == ["/usr/bin/claude", "-p", "PROMPT"]
      and "--output-format" in cmd and "stream-json" in cmd
      and "--allowedTools" in cmd
      and cmd[cmd.index("--model") + 1] == "opus"
      and cmd[cmd.index("--effort") + 1] == "high", str(cmd))
cmd = claude.build_cmd("claude", "P")
check("claude argv omits default model/effort",
      "--model" not in cmd and "--effort" not in cmd, str(cmd))

cmd = oc.build_cmd("/opt/homebrew/bin/opencode", "PROMPT",
                   model="anthropic/claude-sonnet-4-5", effort="high")
check("opencode argv shape",
      cmd[:2] == ["/opt/homebrew/bin/opencode", "run"]
      and cmd[cmd.index("--format") + 1] == "json"
      and cmd[cmd.index("--agent") + 1] == ai_backend.OPENCODE_ANALYSIS_AGENT
      and cmd[cmd.index("--model") + 1] == "anthropic/claude-sonnet-4-5"
      and cmd[cmd.index("--variant") + 1] == "high"
      and cmd[-2:] == ["--", "PROMPT"], str(cmd))
cmd = oc.build_cmd("opencode", "P")
check("opencode argv omits default model/variant",
      "--model" not in cmd and "--variant" not in cmd, str(cmd))

# The analysis agent the GUI selects must actually exist in the repo config.
oc_cfg = json.load(open(os.path.join(os.path.dirname(__file__), "..",
                                     "opencode.json")))
agent_cfg = oc_cfg.get("agent", {}).get(ai_backend.OPENCODE_ANALYSIS_AGENT)
check("opencode.json defines the analysis agent", agent_cfg is not None)
check("analysis agent denies edits",
      agent_cfg and agent_cfg.get("permission", {}).get("edit") == "deny")
check("analysis agent allows bash (skills run checkers)",
      agent_cfg and agent_cfg.get("permission", {}).get("bash") == "allow")

# #552: the agent's description used to RESTATE the Claude allowlist as a
# literal ("Read,Glob,Grep,Bash,WebSearch"). Adding Task to the constant made
# that sentence false with nothing to notice, so the literal is banned outright
# -- a description that names the constant cannot go stale.
_desc = (agent_cfg or {}).get("description", "")
#: Two or more allowlist names comma-joined -- the shape of a restated list.
#: Matching a single `Tool,` would fail on ordinary prose ("run checkers with
#: Bash, and fetch datasheets"), which is a gate that cries wolf.
_names = ai_backend.CLAUDE_ALLOWED_TOOLS.split(",")
_restated = [f"{a},{b}" for a in _names for b in _names
             if a != b and f"{a},{b}" in _desc]
check("opencode.json does not restate the allowlist as a literal",
      not _restated,
      f"{_restated} appear as a comma-joined list in the description")
check("opencode.json names the constant instead",
      "CLAUDE_ALLOWED_TOOLS" in _desc, _desc)

# #552 item 4: refuse rather than silently drop. The read-only ask still builds.
try:
    oc.build_cmd("opencode", "P", allowed_tools="Read,Write")
    _oc_refused = ""
except ValueError as _e:
    _oc_refused = str(_e)
check("opencode refuses an allowlist its agent cannot grant",
      "Write" in _oc_refused, _oc_refused or "no ValueError raised")
check("opencode still accepts the read-only default",
      "--agent" in oc.build_cmd("opencode", "P",
                                allowed_tools=ai_backend.CLAUDE_ALLOWED_TOOLS))

# -------------------------------------------------------------- skill prompt

p = claude.skill_prompt("plan-pcb-routing", "/tmp/b.kicad_pcb", "analysis only")
check("claude skill prompt uses slash syntax",
      p.startswith("/plan-pcb-routing /tmp/b.kicad_pcb"), p)
p = oc.skill_prompt("plan-pcb-routing", "/tmp/b.kicad_pcb", "analysis only")
check("opencode skill prompt names the skill tool",
      "plan-pcb-routing" in p and "skill" in p.lower() and not p.startswith("/"), p)

# ------------------------------------------------------- claude stream state

state = claude.stream_state()
t = state.feed({"type": "system", "subtype": "init", "model": "opus",
                "claude_code_version": "2.1", "cwd": "/repo",
                "skills": ["plan-pcb-routing"]})
check("claude init event formatted", t and "model: opus" in t and "skills discovered" in t)
t = state.feed({"type": "assistant", "message": {"content": [
    {"type": "text", "text": "Looking at the board."},
    {"type": "tool_use", "name": "Bash",
     "input": {"description": "run DRC", "command": "python3 py_router/check_drc.py b"}}]}})
check("claude assistant event formatted",
      t and "Looking at the board." in t and "Bash: run DRC" in t)
t = state.feed({"type": "user", "message": {"content": [
    {"type": "tool_result", "is_error": False, "content": "0 violations"}]}})
check("claude tool result formatted", t and "[ok] 0 violations" in t)
check("claude result event swallowed",
      state.feed({"type": "result", "is_error": False,
                  "result": "report\nRESULT=4"}) is None)
result, error = state.finish(0, "")
check("claude finish returns result", result == "report\nRESULT=4" and error is None)
check("claude RESULT extracted", extract_result_line(result) == "4")

state = claude.stream_state()
state.feed({"type": "result", "is_error": True, "result": "Not logged in"})
result, error = state.finish(1, "")
check("claude error result", result is None and error == "Not logged in")
check("claude auth hint fires", "/login" in claude.auth_hint("Not logged in"))
check("claude auth hint quiet on other errors", claude.auth_hint("boom") == "")

state = claude.stream_state()
result, error = state.finish(2, "  crashed  ")
check("claude no-result uses stderr", result is None and error == "crashed")
state = claude.stream_state()
result, error = state.finish(3, "")
check("claude no-result no-stderr names exit code",
      result is None and "exited with code 3" in error)

# ----------------------------------------------------- opencode stream state

state = oc.stream_state()
check("opencode step events silent",
      state.feed({"type": "step_start", "part": {"type": "step-start"}}) is None)
t = state.feed({"type": "tool_use", "part": {
    "type": "tool", "tool": "bash",
    "state": {"status": "completed",
              "input": {"command": "python3 py_router/check_drc.py b",
                        "description": "run DRC"}}}})
check("opencode tool event formatted", t and "bash: run DRC" in t)
t = state.feed({"type": "tool_use", "part": {
    "type": "tool", "tool": "read",
    "state": {"status": "error", "error": "no such file",
              "input": {"filePath": "/x"}}}})
check("opencode tool error marked", t and "read: /x" in t and "[x] no such file" in t)
t = state.feed({"type": "text", "part": {"type": "text",
                                         "text": "The board has 4 layers.\nRESULT=4"}})
check("opencode text event formatted", t and "RESULT=4" in t)
result, error = state.finish(0, "")
check("opencode finish joins texts",
      error is None and extract_result_line(result) == "4")

state = oc.stream_state()
state.feed({"type": "text", "part": {"text": "partial"}})
state.feed({"type": "error", "error": {"name": "ProviderAuthError",
                                       "data": {"message": "no credentials for anthropic"}}})
result, error = state.finish(1, "")
check("opencode error event wins", result is None
      and error == "no credentials for anthropic")
check("opencode auth hint fires",
      "opencode auth login" in oc.auth_hint("no credentials for anthropic"))

state = oc.stream_state()
result, error = state.finish(1, "")
check("opencode empty run errors", result is None and "exited with code 1" in error)

# ----------------------------------------------------------- misc primitives

check("extract_result_line last wins",
      extract_result_line("RESULT=1\ntext\nRESULT=2\n") == "2")
check("extract_result_line missing", extract_result_line("no result") is None)
check("summarize unknown tool dumps json",
      "x" in summarize_tool_use("mystery", {"x": 1}))
check("summarize truncates",
      len(summarize_tool_use("bash", {"command": "y" * 500})) < 140)
# #552 item 3: a subagent dispatch must render as its DESCRIPTION, or the live
# transcript goes quiet through the longest part of a run -- the fan-out. The
# case has existed since #633 and nothing asserted it. Both spellings, because
# Claude sends "Task" and opencode sends "task".
check("summarize names a Task by its description",
      summarize_tool_use("Task", {"description": "verify the placement",
                                  "prompt": "a very long prompt"})
      == "Task: verify the placement")
check("summarize names a lowercase task too",
      summarize_tool_use("task", {"description": "verify"}) == "task: verify")

# ------------------------------------------- extract_narrative dual schemas

claude_recs = [
    {"type": "system", "subtype": "init", "timestamp": "2026-07-29T01:02:03Z"},
    {"type": "assistant", "timestamp": "2026-07-29T01:02:04Z", "message": {"content": [
        {"type": "text", "text": "Routing signal nets now."},
        {"type": "tool_use", "name": "Bash",
         "input": {"command": "python3 -X utf8 /repo/route.py board.kicad_pcb --clearance 0.1"}}]}},
    {"type": "assistant", "message": {"content": [
        {"type": "tool_use", "name": "Bash",
         "input": {"command": "python3 /repo/check_drc.py board.kicad_pcb"}}]}},
]
md, turns, cmds = extract_narrative.build(claude_recs, board="b1")
check("narrative claude: narration kept", "Routing signal nets now." in md)
# NOT "py_router/route.py": the narrative's _routing_cmd slices from the TOOL
# BASENAME (any leading path is dropped by design), so the md carries
# "route.py ..." regardless of where the tool lives. The #522 sweep (8b5b692)
# rewrote this assertion as if it were a file path and broke it.
check("narrative claude: routing cmd kept", "route.py" in md and cmds == 1)
check("narrative claude: check_drc dropped", "check_drc" not in md)

oc_recs = [
    {"type": "step_start", "timestamp": 1753750000000, "part": {"type": "step-start"}},
    {"type": "text", "timestamp": 1753750001000,
     "part": {"type": "text", "text": "Fanning out the BGA."}},
    {"type": "tool_use", "timestamp": 1753750002000, "part": {
        "type": "tool", "tool": "bash",
        "state": {"status": "completed",
                  "input": {"command": "python3 -X utf8 /repo/bga_fanout.py board.kicad_pcb"}}}},
    {"type": "tool_use", "part": {
        "type": "tool", "tool": "bash",
        "state": {"status": "completed",
                  "input": {"command": "python3 /repo/list_nets.py board.kicad_pcb"}}}},
]
md, turns, cmds = extract_narrative.build(oc_recs, board="b2")
check("narrative opencode: narration kept", "Fanning out the BGA." in md)
check("narrative opencode: routing cmd kept", "bga_fanout.py" in md and cmds == 1)
check("narrative opencode: list_nets dropped", "list_nets" not in md)
check("narrative opencode: ms timestamps parsed", "UTC" in md)

# -------------------------------------------------------------------- wrapup

if FAILURES:
    print(f"\n{len(FAILURES)} FAILURE(S): {FAILURES}")
    sys.exit(1)
print("\nAll #503 ai_backend checks passed.")
