"""
KiCad Routing Tools - AI backend abstraction (issue #503).

The GUI's AI features and the stress harness drive an agent CLI headless.
Historically that CLI was Claude Code only; this module makes the backend
pluggable so opencode (https://opencode.ai) or OpenAI Codex can be used as
configurable alternatives.  Codex reuses the ChatGPT login saved by its CLI,
so it does not require an API key.

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
CLAUDE_ALLOWED_TOOLS = "Read,Glob,Grep,Bash,WebSearch"

# opencode has no per-run tool allowlist flag; the repo's opencode.json
# defines this agent (edit denied, bash/webfetch allowed) as the equivalent
# of the Claude allowlist above.
OPENCODE_ANALYSIS_AGENT = "pcb-analysis"


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
    # Some native CLIs accept a prompt argument safely, while others are more
    # robust when a potentially long prompt is supplied on stdin.
    prompt_via_stdin = False

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
        allowlist and extra writable directories); opencode ignores them
        (its agent config in opencode.json is the equivalent).
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
        # allowed_tools/add_dirs are Claude-specific (accepted for signature
        # parity; the Placement tab pins the Claude backend anyway).
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


# --------------------------------------------------------------------- Codex


def _codex_error_message(event):
    """Extract a useful message from Codex's top-level/turn error shapes."""
    error = event.get("error")
    if isinstance(error, dict):
        return str(error.get("message") or error.get("detail")
                   or error.get("type") or "Codex error")
    if error:
        return str(error)
    return str(event.get("message") or "Codex error")


class _CodexStreamState(_StreamState):
    """Parse the JSONL event stream produced by ``codex exec --json``."""

    def __init__(self):
        self._texts = []
        self._errors = []

    def feed(self, event):
        etype = event.get("type")
        item = event.get("item") or {}
        itype = item.get("type")

        if etype == "thread.started":
            thread_id = event.get("thread_id")
            return f"Codex | thread: {thread_id}\n\n" if thread_id else None

        if etype == "item.completed" and itype == "agent_message":
            text = str(item.get("text") or "").strip()
            if text:
                self._texts.append(text)
                return text + "\n"
            return None

        # Show tool progress once, when it starts.  Agent messages are emitted
        # only on completion above because their text is not stable earlier.
        if etype == "item.started":
            if itype == "command_execution":
                command = " ".join(str(item.get("command") or "").split())
                if len(command) > 120:
                    command = command[:120] + "..."
                return f"  -> Bash: {command}\n" if command else None
            if itype == "web_search":
                query = " ".join(str(item.get("query") or "web search").split())
                return f"  -> WebSearch: {query}\n"
            if itype == "mcp_tool_call":
                name = item.get("tool") or item.get("name") or "MCP tool"
                return f"  -> {name}\n"

        if etype == "item.completed" and itype == "command_execution" \
                and item.get("status") == "failed":
            output = str(item.get("aggregated_output") or "command failed")
            first = output.strip().splitlines()[0] if output.strip() else "command failed"
            return f"     [x] {first[:120]}\n"

        if etype in ("turn.failed", "error"):
            message = _codex_error_message(event)
            self._errors.append(message)
            return None
        return None

    def finish(self, returncode, stderr):
        if self._errors:
            return None, "; ".join(self._errors)
        if returncode:
            return None, (stderr or "").strip() or \
                f"codex exited with code {returncode}"
        if not self._texts:
            return None, (stderr or "").strip() or \
                "codex completed without an agent message"
        # Keeping all messages makes RESULT= extraction robust if a model
        # emits an informational message before its final answer.
        return "\n".join(self._texts), None


class CodexBackend(AIBackend):
    id = "codex"
    label = "OpenAI Codex"
    cli_name = "codex"
    install_url = "https://learn.chatgpt.com/docs/codex/cli"
    login_hint = (
        "Codex CLI is installed but not logged in: open a terminal, run "
        "`codex login`, choose Sign in with ChatGPT, then retry. This uses "
        "your ChatGPT subscription; no API key is required.")
    candidates = (
        os.path.expandvars(r"%APPDATA%\npm\codex.cmd"),
        os.path.expanduser("~/.local/bin/codex"),
        "/opt/homebrew/bin/codex",
        "/usr/local/bin/codex",
        "/usr/bin/codex",
    )
    model_suggestions = ("Default",)
    effort_suggestions = ("Default", "minimal", "low", "medium", "high", "xhigh")
    model_tooltip = (
        "Codex model (--model). Default uses your Codex CLI model and ChatGPT "
        "subscription. The field is editable for model names supported by "
        "your account.")
    effort_tooltip = (
        "Codex reasoning effort: minimal/low/medium/high/xhigh. Default uses "
        "your Codex CLI configuration; xhigh depends on the selected model.")
    _auth_markers = ("not logged in", "codex login", "sign in", "unauthorized",
                     "authentication", "refresh token", "401")
    prompt_via_stdin = True

    def find_cli(self):
        """Find the public CLI, not the desktop app's private executable.

        Recent Windows desktop packages contain an internal ``codex.exe`` in
        Program Files\\WindowsApps.  It may appear on PATH but Windows denies
        direct launches from other applications, so treating it as the CLI
        leaves the GUI enabled only to fail with Access Denied.
        """
        paths = [shutil.which("codex.cmd"), shutil.which(self.cli_name)]
        paths.extend(self.candidates)
        for path in paths:
            if not path:
                continue
            normalized = os.path.normcase(os.path.abspath(path))
            if ("windowsapps" in normalized
                    and "openai.codex_" in normalized):
                continue
            if os.path.isfile(path) and os.access(path, os.X_OK):
                return path
        return None

    def not_found_message(self):
        return (
            "OpenAI Codex CLI not found. Install the public CLI with "
            "`npm install -g @openai/codex` (the Codex desktop app's internal "
            "executable is not a CLI launcher), run `codex login`, then reopen "
            "this dialog.")

    def skill_prompt(self, skill, args, instructions):
        # Codex does not discover this repository's Claude-format skills
        # reliably, so pass the selected skill directly on stdin.
        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        skill_path = os.path.join(root_dir, ".claude", "skills", skill,
                                  "SKILL.md")
        try:
            with open(skill_path, "r", encoding="utf-8",
                      errors="replace") as skill_file:
                skill_text = skill_file.read()
        except OSError as error:
            skill_text = (f"[Could not embed local skill {skill_path}: "
                          f"{error}]")
        prompt = (f"Follow the embedded local {skill} skill for: "
                  f"{args} — {instructions}"
                  "\n\n<LOCAL_SKILL>\n" + skill_text + "\n</LOCAL_SKILL>")
        if (skill == "plan-pcb-routing" and isinstance(args, str)
                and os.path.isfile(args)):
            try:
                from .codex_plan_context import collect_plan_context
                board_context = collect_plan_context(args)
            except Exception as error:
                board_context = json.dumps({"error": str(error)})
            prompt += ("\n<LOCAL_BOARD_CONTEXT>\n" + board_context +
                       "\n</LOCAL_BOARD_CONTEXT>\n"
                       "<EXECUTION_RULES>\n"
                       "The plugin already performed the complete read-only "
                       "board analysis. Ignore command examples in the skill: "
                       "do not call "
                       "any shell, local tool, MCP, connector, or repository "
                       "inspection tool. Derive the report and final RESULT line "
                       "from the embedded skill and context. Web search may be "
                       "used for datasheets. Do not modify files.\n"
                       "</EXECUTION_RULES>")
        elif os.name == "nt":
            prompt += (
                "\nOn Windows, the Codex shell already wraps commands with "
                "`cmd.exe`; do not invoke a nested shell. Work from the "
                "requested work directory, use relative artifact paths, and "
                "invoke the repository's `python3.cmd` directly for Python "
                "scripts. Do not install Python packages during the run."
            )
        return prompt

    def build_cmd(self, cli_path, prompt, model=None, effort=None,
                  allowed_tools=None, add_dirs=()):
        # Generic analysis calls omit allowed_tools and remain read-only.
        # Claude/OpenCode placement may pass an allowlist and isolated workdir;
        # controlled Codex placement is self-contained and remains read-only.
        sandbox = "workspace-write" if allowed_tools else "read-only"
        self_contained = (not allowed_tools and
                          ("<LOCAL_BOARD_CONTEXT>" in prompt or
                           "<LOCAL_PLACEMENT_CONTEXT>" in prompt))
        cmd = [cli_path, "--ask-for-approval", "never"]
        if "<LOCAL_PLACEMENT_CONTEXT>" not in prompt:
            cmd += ["--search"]
        if self_contained:
            cmd += ["--disable", "shell_tool"]
        for directory in add_dirs:
            cmd += ["--add-dir", directory]
        cmd += [
            "exec", "--json", "--ephemeral", "--skip-git-repo-check",
            "--sandbox", sandbox,
        ]
        if self_contained:
            # Auth is still loaded, while user plugins, MCP servers, hooks and
            # notifications are omitted from this self-contained analysis.
            cmd += ["--ignore-user-config"]
        if model:
            cmd += ["--model", model]
        if effort:
            # A bare value is accepted as a literal string by Codex and also
            # survives Windows npm .cmd shims without nested-quote hazards.
            cmd += ["--config", f"model_reasoning_effort={effort}"]
        # AISkillRunner supplies the actual prompt on stdin.
        cmd += ["-"]
        return cmd

    def stream_state(self):
        return _CodexStreamState()


# ------------------------------------------------------------------ registry

BACKENDS = {b.id: b for b in (ClaudeBackend(), OpencodeBackend(), CodexBackend())}
BACKEND_IDS = tuple(BACKENDS)          # ("claude", "opencode", "codex")
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
