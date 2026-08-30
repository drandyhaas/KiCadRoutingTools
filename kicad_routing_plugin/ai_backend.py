"""
KiCad Routing Tools - AI backend abstraction (issue #503).

The GUI's AI features and the stress harness drive an agent CLI headless.
Historically that CLI was Claude Code only; this module makes the backend
pluggable so opencode (https://opencode.ai) can be used as a configurable
alternative with any model provider it supports.

Each backend knows how to:
  - locate its CLI (`find_cli`), including the common install spots KiCad
    misses when launched from Finder/desktop without a shell PATH;
  - phrase a skill invocation (`skill_prompt`) - Claude Code uses the
    /skill-name slash syntax, opencode loads skills through its `skill`
    tool (both discover this repo's .claude/skills/ directory);
  - build the headless argv (`build_cmd`);
  - parse its streaming JSON events into transcript text and a final
    result (`stream_state`); the RESULT=<value> output contract itself is
    backend-agnostic;
  - recognize an auth failure and tell the user how to log in
    (`auth_hint`).

This module is intentionally wx-free so tests can import it headless.
"""

import json
import os
import shutil

# Read-only analysis tools: the DEFAULT allowlist for the "Ask AI" analysis
# skills, which never need write access to the board. Callers that drive a
# board-mutating skill headless (the Placement tab) pass their own
# write-capable allowlist via build_cmd(allowed_tools=...).
#
# The subagent-dispatch tool is here (#552) so an analysis skill can buy a
# SECOND OPINION: a verifier that re-measures on its own channel, rather than
# the same model re-reading its own work inline.
#
# BOTH spellings, deliberately. Claude Code renamed the tool: on 2.1.251 the
# dispatch event carries `"name":"Agent"` and `Task` is the older token, which
# is what this repo has shipped since #633. A permission rule matches the
# canonical name only, so listing one name risks granting nothing on the CLI
# the user happens to have. Listing both costs nothing on either.
#
# WHAT THIS LIST IS, AND IS NOT. `--allowedTools` AUTO-APPROVES the tools it
# names; it does not remove the others. Measured on 2.1.251 with exactly the
# argv build_cmd emits: the run's `system/init` event reports
# `permissionMode: auto` (from the USER's settings, which this module neither
# sets nor overrides) and lists `Write`/`Edit` among its tools regardless of
# what is written here. So this is a statement of INTENT, not a sandbox.
# Anyone reasoning about what a run can do to the board should note that
# `Bash` is on this list, and `Bash` writes files.
CLAUDE_ALLOWED_TOOLS = "Read,Glob,Grep,Bash,Agent,Task,WebSearch"

# The behavioural half of the contract. Stated once here because it was
# hand-copied into eight prompts across four GUI modules, which is exactly how
# a contract drifts -- one site gets edited and the other seven quietly mean
# something else.
#
# NEITHER HALF IS ENFORCED, and an earlier draft of this comment claimed
# otherwise. `--allowedTools` auto-approves rather than restricts (see above),
# and this is prompt text. What follows is the contract the run is ASKED to
# honour.
#
# WHY PROMPT TEXT, given that enforceable levers exist. They do: `claude
# --help` on 2.1.251 offers `--agents <json>` (which can define a subagent and
# its tools outright), `--append-system-prompt`, `--permission-mode` and
# `--settings`. `--agents` is the enforceable form of #552 item 2 and is the
# right eventual answer. It is not this change: it means designing the child
# agents themselves, and it applies to Claude Code only, while this sentence
# has to reach opencode too. What ships here is the repo's EXISTING convention
# for bounding a child -- `<subagent_prompt>` blocks quoted verbatim (see
# .claude/skills/plan-pcb-placement/SKILL.md) -- stated once instead of eight
# times. Treat the enforced version as owed, not done.
#
# VERDICT= rather than RESULT= for the child's answer, because
# extract_result_line() below takes the LAST `RESULT=` line of the parent's
# final message as that run's result. A child whose text the parent echoes
# would land on the parent's contract.
#
# "analysis and planning only" rather than "analysis only": that was the plan
# button's wording and it is the widest of the eight, so unifying on it loosens
# nothing that matters. The clause after the colon is identical to what all
# eight said before.
ANALYSIS_CONSTRAINT = (
    "analysis and planning only: do not execute any routing commands and do "
    "not modify any files. If you dispatch a subagent, copy this sentence into "
    "its prompt verbatim and give it no tool this run does not have; have it "
    "answer with a line beginning VERDICT= (never RESULT=, which this GUI "
    "reads as the run's own result line)."
)

# opencode has no per-run tool allowlist flag; the repo's opencode.json
# defines this agent (edit denied, bash/webfetch allowed) as the equivalent
# of the Claude allowlist above.
OPENCODE_ANALYSIS_AGENT = "pcb-analysis"

# Claude-Code tool names whose intent the pinned pcb-analysis agent cannot
# serve. opencode.json's `permission` block denies exactly one thing --
# `"edit": "deny"` -- so only `edit` is that file's own word; the rest is this
# module mapping Claude's write-tool names onto it, and an earlier draft of
# this comment wrongly called the whole set fact.
#
# OpencodeBackend.build_cmd REFUSES an allowed_tools naming any of these rather
# than silently dropping the argument (#552 item 4): a caller that believes it
# has Write and does not is worse off than one that will not start.
#
# THIS IS A GUARD, NOT A SANDBOX, and the difference is measurable: it matches
# tool NAMES, so `Write(/tmp/x)` (a specifier form `claude --help` documents),
# `MultiEdit`, an `mcp__*` write tool, and plain `Bash` all pass it. It catches
# the mistake a caller is actually likely to make -- handing over a
# write-capable allowlist wholesale, as PLACEMENT_ALLOWED_TOOLS would -- and
# claims nothing beyond that.
#
# `--add-dir` has no opencode flag either, but the agent grants
# `external_directory`, so dropping it costs the run nothing; it is not
# refused.
OPENCODE_DENIED_TOOLS = frozenset({"write", "edit", "notebookedit", "multiedit"})


def _denied_opencode_tools(allowed_tools):
    """The names in `allowed_tools` the pcb-analysis agent cannot serve.

    Accepts what `--allowedTools` itself accepts and what callers actually
    pass: a comma string, a SPACE-separated string (the CLI takes both), a
    list/tuple, or None. It also strips a `Tool(specifier)` suffix -- the form
    `claude --help` documents as `"Bash(git *) Edit"` -- because `Write(/tmp/x)`
    is a write request however it is spelled.

    Returns clean, stripped names so the refusal message cannot echo a raw tab
    or newline back at the operator.
    """
    if not allowed_tools:
        return set()
    if isinstance(allowed_tools, str):
        items = allowed_tools.replace(",", " ").split()
    else:
        items = [str(t) for t in allowed_tools]
    out = set()
    for raw in items:
        name = raw.strip().split("(", 1)[0].strip()
        if name.lower() in OPENCODE_DENIED_TOOLS:
            out.add(name)
    return out


class AIBackend:
    """One headless agent CLI. Subclasses fill in the specifics."""

    id = None            # settings key, e.g. "claude"
    label = None         # UI label, e.g. "Claude Code"
    cli_name = None      # binary name looked up on PATH
    install_url = None   # where to get it
    login_hint = None    # one-liner shown on auth failures
    candidates = ()      # absolute fallback paths (no shell PATH in KiCad)
    model_suggestions = ("Default",)   # editable-combo suggestions
    effort_suggestions = ("Default",)
    model_tooltip = ""
    effort_tooltip = ""
    # Substrings (lowercased) that mark an error as an auth problem.
    _auth_markers = ()

    def find_cli(self):
        """Return the CLI path, or None if not installed."""
        path = shutil.which(self.cli_name)
        if path:
            return path
        for candidate in self.candidates:
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
        return None

    def not_found_message(self):
        return (f"{self.label} CLI not found. Install it ({self.install_url}) "
                f"and make sure `{self.cli_name}` is on your PATH.")

    def auth_hint(self, error):
        """Extra guidance when a run failed because the CLI isn't logged in."""
        if error and any(m in error.lower() for m in self._auth_markers):
            return "\n" + self.login_hint
        return ""

    def skill_prompt(self, skill, args, instructions):
        """Compose the prompt that invokes one of the repo's skills."""
        raise NotImplementedError

    def build_cmd(self, cli_path, prompt, model=None, effort=None,
                  allowed_tools=None, add_dirs=()):
        """The headless argv streaming one JSON event per stdout line.

        allowed_tools/add_dirs are honored by Claude Code (per-run tool
        allowlist and extra directories). opencode has no per-run allowlist
        flag -- its agent config in opencode.json is the equivalent -- so it
        ignores add_dirs and REFUSES an allowed_tools its pinned agent cannot
        serve rather than dropping it silently (#552).
        """
        raise NotImplementedError

    def stream_state(self):
        """A fresh per-run stream parser (see _StreamState)."""
        raise NotImplementedError


class _StreamState:
    """Per-run stream parser state.

    feed(event) returns transcript text to display (or None), and finish()
    resolves the run into (result_text, error) - exactly one is non-None.
    """

    def feed(self, event):
        raise NotImplementedError

    def finish(self, returncode, stderr):
        raise NotImplementedError


# --------------------------------------------------------------------- Claude


def summarize_tool_use(name, tool_input, _KEYS={
        # tool name (lowercased) -> input keys to try, in order
        "bash": ("description", "command"),
        "read": ("file_path", "filePath"),
        "write": ("file_path", "filePath"),
        "edit": ("file_path", "filePath"),
        "glob": ("pattern",),
        "grep": ("pattern",),
        "list": ("path",),
        "websearch": ("query",),
        "webfetch": ("url",),
        "skill": ("name", "skill"),
        "task": ("description",)}):
    """One-line human-readable summary of a tool call (both backends'
    tool/input names are covered; unknown tools dump their input JSON)."""
    detail = None
    for key in _KEYS.get(str(name).lower(), ()):
        if tool_input.get(key):
            detail = tool_input[key]
            break
    if detail is None:
        detail = json.dumps(tool_input)
    detail = " ".join(str(detail).split())
    if len(detail) > 120:
        detail = detail[:120] + "..."
    return f"{name}: {detail}"


def tool_result_text(block, max_len=120):
    """First line of a Claude tool result, truncated."""
    content = block.get("content", "")
    if isinstance(content, list):
        content = " ".join(
            c.get("text", "") for c in content
            if isinstance(c, dict) and c.get("type") == "text")
    first_line = str(content).strip().splitlines()[0] if str(content).strip() else "(no output)"
    if len(first_line) > max_len:
        first_line = first_line[:max_len] + "..."
    return first_line


def format_claude_stream_event(event):
    """Format one Claude stream-json event as transcript text, or None."""
    etype = event.get("type")
    if etype == "system" and event.get("subtype") == "init":
        model = event.get("model", "unknown")
        version = event.get("claude_code_version", "unknown")
        lines = [f"Claude Code {version} | model: {model}",
                 f"cwd: {event.get('cwd', '?')}"]
        skills = event.get("skills", [])
        if skills:
            shown = ", ".join(skills[:8]) + (", ..." if len(skills) > 8 else "")
            lines.append(f"skills discovered: {len(skills)} ({shown})")
        return "\n".join(lines) + "\n\n"
    if etype == "assistant":
        lines = []
        for block in event.get("message", {}).get("content", []):
            btype = block.get("type")
            if btype == "text" and block.get("text", "").strip():
                lines.append(block["text"].rstrip())
            elif btype == "tool_use":
                summary = summarize_tool_use(block.get("name", "?"), block.get("input", {}))
                lines.append(f"  -> {summary}")
        return "\n".join(lines) + "\n" if lines else None
    if etype == "user":
        content = event.get("message", {}).get("content", [])
        lines = []
        if isinstance(content, list):
            for block in content:
                if isinstance(block, dict) and block.get("type") == "tool_result":
                    mark = "x" if block.get("is_error") else "ok"
                    lines.append(f"     [{mark}] {tool_result_text(block)}")
        return "\n".join(lines) + "\n" if lines else None
    return None


class _ClaudeStreamState(_StreamState):
    def __init__(self):
        self._final_event = None

    def feed(self, event):
        if event.get("type") == "result":
            self._final_event = event
            return None
        return format_claude_stream_event(event)

    def finish(self, returncode, stderr):
        if self._final_event is None:
            # claude died before emitting a result event
            return None, (stderr or "").strip() or f"claude exited with code {returncode}"
        if self._final_event.get("is_error"):
            return None, str(self._final_event.get("result", "unknown error from claude"))
        return str(self._final_event.get("result", "")), None


class ClaudeBackend(AIBackend):
    id = "claude"
    label = "Claude Code"
    cli_name = "claude"
    install_url = "https://claude.com/claude-code"
    login_hint = ("Claude Code is installed but not logged in: open a "
                  "terminal, run `claude`, complete /login, then retry.")
    candidates = (
        os.path.expanduser("~/.claude/local/claude"),
        os.path.expanduser("~/.local/bin/claude"),
        "/opt/homebrew/bin/claude",
        "/usr/local/bin/claude",
        "/usr/bin/claude",
        # Windows: native installer and npm -g locations (KiCad launched from
        # the desktop misses the shell PATH there too). An unset env var
        # leaves the literal %VAR% in the path, which simply fails isfile().
        os.path.expanduser("~/.local/bin/claude.exe"),
        os.path.expandvars(r"%LOCALAPPDATA%\Programs\claude\claude.exe"),
        os.path.expandvars(r"%APPDATA%\npm\claude.cmd"),
    )
    # ALIASES, not pinned version IDs: the CLI resolves each to the newest
    # model of its tier, so the list never goes stale.
    model_suggestions = ("Default", "fable", "opus", "sonnet", "haiku")
    effort_suggestions = ("Default", "low", "medium", "high", "xhigh", "max")
    model_tooltip = (
        "Model for the headless run (--model). Default = your claude CLI "
        "default. Bigger models give deeper analysis; haiku is fastest/cheapest.")
    effort_tooltip = (
        "Reasoning effort (--effort): low/medium/high/xhigh/max. Higher = more "
        "thorough but slower and costlier. Not supported on haiku.")
    _auth_markers = ("invalid api key", "/login", "not logged in",
                     "authentication", "oauth")

    def skill_prompt(self, skill, args, instructions):
        return f"/{skill} {args} — {instructions}"

    def build_cmd(self, cli_path, prompt, model=None, effort=None,
                  allowed_tools=None, add_dirs=()):
        cmd = [
            cli_path, "-p", prompt,
            # stream-json (requires --verbose in -p mode) emits one JSON
            # event per line as the agent works, for live progress.
            "--output-format", "stream-json", "--verbose",
            "--allowedTools", allowed_tools or CLAUDE_ALLOWED_TOOLS,
        ]
        for d in add_dirs:
            cmd += ["--add-dir", d]
        if model:
            cmd += ["--model", model]
        if effort:
            cmd += ["--effort", effort]
        return cmd

    def stream_state(self):
        return _ClaudeStreamState()


# ------------------------------------------------------------------- opencode


class _OpencodeStreamState(_StreamState):
    """Parses `opencode run --format json` events (one JSON per line).

    Event shape (packages/opencode/src/cli/cmd/run.ts): {"type": ...,
    "timestamp", "sessionID", "part"|"error"}. Text/reasoning parts are
    emitted once completed (part.time.end); tool parts once completed or
    errored. There is no terminal "result" event: the final reply is the
    accumulated text parts, and errors arrive as {"type": "error"} events.
    """

    def __init__(self):
        self._texts = []
        self._errors = []

    def feed(self, event):
        etype = event.get("type")
        part = event.get("part") or {}
        if etype == "text":
            text = (part.get("text") or "").strip()
            if text:
                self._texts.append(text)
                return text + "\n"
            return None
        if etype == "tool_use":
            state = part.get("state") or {}
            summary = summarize_tool_use(part.get("tool", "?"), state.get("input") or {})
            line = f"  -> {summary}\n"
            if state.get("status") == "error":
                err = str(state.get("error", "")).strip().splitlines()
                line += f"     [x] {err[0] if err else 'tool error'}\n"
            return line
        if etype == "error":
            error = event.get("error") or {}
            message = ""
            if isinstance(error, dict):
                message = str((error.get("data") or {}).get("message", "")
                              or error.get("name", ""))
            self._errors.append(message or "unknown error from opencode")
            return None
        return None  # step_start / step_finish / reasoning: no transcript line

    def finish(self, returncode, stderr):
        if self._errors:
            return None, "; ".join(self._errors)
        if not self._texts:
            return None, (stderr or "").strip() or f"opencode exited with code {returncode}"
        # No terminal result event: the reply is the accumulated text parts
        # (RESULT= extraction scans backwards, so joining them all is safe).
        return "\n".join(self._texts), None


class OpencodeBackend(AIBackend):
    id = "opencode"
    label = "opencode"
    cli_name = "opencode"
    install_url = "https://opencode.ai"
    login_hint = ("opencode is installed but has no working provider "
                  "credentials: open a terminal and run `opencode auth login`.")
    candidates = (
        os.path.expanduser("~/.opencode/bin/opencode"),
        os.path.expanduser("~/.local/bin/opencode"),
        "/opt/homebrew/bin/opencode",
        "/usr/local/bin/opencode",
        "/usr/bin/opencode",
    )
    model_suggestions = ("Default",)
    effort_suggestions = ("Default", "minimal", "low", "medium", "high", "max")
    model_tooltip = (
        "Model as provider/model (opencode -m), e.g. anthropic/claude-sonnet-4-5 "
        "or openai/gpt-5.2-codex. Default = your opencode default model. "
        "Note: the skills' output contracts are tuned on Claude models; "
        "smaller models may not follow them reliably.")
    effort_tooltip = (
        "Model variant (opencode --variant): provider-specific reasoning "
        "effort, e.g. high, max, minimal. Default = none.")
    _auth_markers = ("auth", "credential", "api key", "apikey", "unauthorized",
                     "no providers", "provider not found", "not logged in")

    def skill_prompt(self, skill, args, instructions):
        # opencode has no slash syntax for skills; it loads them on demand
        # through its `skill` tool (discovering this repo's .claude/skills/).
        return (f"Load the '{skill}' skill with your skill tool and follow "
                f"it for: {args} — {instructions}")

    def build_cmd(self, cli_path, prompt, model=None, effort=None,
                  allowed_tools=None, add_dirs=()):
        # opencode has no per-run allowlist flag: the agent pinned below IS the
        # allowlist. So a request this agent cannot grant is REFUSED, not
        # dropped (#552 item 4). Before this, the two kwargs were bound and
        # never read, and `PLACEMENT_SUPPORTED_BACKENDS = ("claude",)` was the
        # only thing standing between a write-capable caller and a run that
        # would fail deep inside the skill instead of at launch.
        #
        # add_dirs is deliberately NOT refused: opencode grants the agent
        # `external_directory`, so having no --add-dir costs the run nothing.
        denied = sorted(_denied_opencode_tools(allowed_tools))
        if denied:
            raise ValueError(
                "the opencode backend cannot grant " + ", ".join(denied)
                + f": it runs the read-only '{OPENCODE_ANALYSIS_AGENT}' agent, "
                "whose opencode.json permissions deny edits. Use the Claude "
                "backend for a skill that writes.")
        cmd = [
            cli_path, "run",
            "--format", "json",       # one JSON event per line
            "--agent", OPENCODE_ANALYSIS_AGENT,  # read-only agent (opencode.json)
        ]
        if model:
            cmd += ["--model", model]
        if effort:
            cmd += ["--variant", effort]
        cmd += ["--", prompt]
        return cmd

    def stream_state(self):
        return _OpencodeStreamState()


# ------------------------------------------------------------------ registry

BACKENDS = {b.id: b for b in (ClaudeBackend(), OpencodeBackend())}
BACKEND_IDS = tuple(BACKENDS)          # ("claude", "opencode")
DEFAULT_BACKEND_ID = "claude"


def get_backend(backend_id):
    """The backend for a settings/UI id; unknown ids fall back to Claude."""
    return BACKENDS.get(backend_id, BACKENDS[DEFAULT_BACKEND_ID])


def extract_result_line(text):
    """Return the value of the last RESULT=<value> line, or None."""
    for line in reversed(text.strip().splitlines()):
        line = line.strip()
        if line.startswith("RESULT="):
            return line[len("RESULT="):].strip()
    return None
