"""Controlled Codex placement loop: Codex decides, the plugin executes.

Codex runs without shell, web, MCP, or repository access.  Each turn returns
one small JSON action.  This module validates that action and, for ``run``,
launches one repository Python script directly (never through a shell) inside
the isolated placement work directory.
"""

import json
import os
import re
import shutil
import subprocess
import sys
import threading

from .ai_backend import extract_result_line
from .placement_run import PLACEMENT_RESULT_SCHEMA


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SCRIPT_ROOTS = (
    os.path.join(ROOT_DIR, ".claude", "skills",
                 "plan-pcb-placement-and-routing", "scripts"),
    os.path.join(ROOT_DIR, ".claude", "skills", "plan-pcb-placement",
                 "scripts"),
    os.path.join(ROOT_DIR, "py_placer"),
    os.path.join(ROOT_DIR, "py_router"),
    os.path.join(ROOT_DIR, "py_tools"),
)
_RESOURCES = {
    "placement": os.path.join(ROOT_DIR, ".claude", "skills",
                              "plan-pcb-placement", "SKILL.md"),
    "routing": os.path.join(ROOT_DIR, ".claude", "skills",
                            "plan-pcb-routing", "SKILL.md"),
    "placement-routing": os.path.join(
        ROOT_DIR, ".claude", "skills", "plan-pcb-placement-and-routing",
        "SKILL.md"),
}
_TEXT_EXTENSIONS = {".json", ".jsonl", ".log", ".txt", ".md",
                    ".kicad_pro", ".kicad_dru"}
_ENV_NAME = re.compile(r"^KICAD_[A-Z0-9_]+$")
_NET_VALUE_FLAGS = {"--nets", "--power-nets", "--ignore-nets",
                    "--target-nets", "--pairs", "--net"}
_MAX_TURNS = 160
_MAX_OUTPUT = 40_000
_MAX_CONTEXT = 500_000


def _inside(path, root):
    try:
        root = os.path.normcase(os.path.abspath(root))
        path = os.path.normcase(os.path.abspath(path))
        return os.path.commonpath((path, root)) == root
    except (OSError, ValueError):
        return False


def _read_text(path, limit=_MAX_OUTPUT):
    with open(path, "r", encoding="utf-8", errors="replace") as stream:
        text = stream.read(limit + 1)
    return text[:limit] + ("\n...[truncated]" if len(text) > limit else "")


def _manifest(workdir):
    rows = []
    for current, directories, files in os.walk(workdir):
        directories.sort()
        files.sort()
        for name in directories + files:
            path = os.path.join(current, name)
            try:
                stat = os.stat(path)
            except OSError:
                continue
            relative = os.path.relpath(path, workdir).replace(os.sep, "/")
            rows.append({"name": relative, "size": stat.st_size,
                         "directory": os.path.isdir(path)})
            if len(rows) >= 300:
                return rows
    return rows


def _python_executable():
    """Return a real Python executable even inside KiCad's host process."""
    executable = sys.executable or ""
    if os.path.basename(executable).lower().startswith("python"):
        return executable
    prefix = sys.prefix or sys.base_prefix
    candidates = ([os.path.join(prefix, "python.exe")]
                  if os.name == "nt" else
                  [os.path.join(prefix, "bin", "python3")])
    candidates.extend(filter(None, (shutil.which("python3"),
                                    shutil.which("python"))))
    return next((path for path in candidates if os.path.isfile(path)), None)


def _decision_contract(mode, workdir, extra):
    result_schema = PLACEMENT_RESULT_SCHEMA.replace("RESULT=", "")
    return f"""You are the decision engine for a controlled KiCad {mode} run.
You have NO tools. Do not ask to use shell, GitHub, MCP, web search, or inspect
the repository. The plugin owns all reads and execution. Return exactly one
single-line RESULT=<JSON> action and no other RESULT line.

Allowed actions:
1. {{"action":"run","script":"<repository-relative .py>",
    "args":["..."],"env":{{"KICAD_NAME":"value"}}}}
2. {{"action":"read_artifact","path":"<relative workdir text file>"}}
3. {{"action":"read_resource","name":"placement"|"routing"|
    "placement-routing"}}
4. {{"action":"finish","result":{result_schema}}}

Rules:
- Use run for one command only. The script must be below py_placer, py_router,
  py_tools, or the two placement skill scripts directories.
- Use only relative board/artifact paths. Never use '..' or an absolute path.
- Never use python -c, pipes, redirections, shell commands, package installs,
  rg, type, more, dir, findstr, or where.
- Driver output is authoritative. Follow one emitted stage at a time and
  satisfy a guard rather than bypassing it.
- When a driver names a JSON artifact for a checker, pass that exact path with
  the checker's --json option; never rely on stdout redirection.
- Keep every artifact in {workdir.replace(os.sep, '/')}. Never overwrite
  input.kicad_pcb. Use ledger.jsonl for convergence state.
- For place_route, every outer loop_driver stage must include --no-delegate.
- Before finish, create final.kicad_pcb, REPORT.md, and the requested movie
  when the workflow can produce them. Paths in finish.result must be relative
  to the workdir; the plugin converts them to absolute paths.
User instructions: {extra.strip() or '(none)'}
"""


class CodexPlacementRunner:
    """AISkillRunner-compatible controlled placement orchestrator."""

    def __init__(self, cli_path, backend, on_transcript, on_done,
                 dispatch=lambda fn, *args: fn(*args)):
        self.cli_path = cli_path
        self.backend = backend
        self.on_transcript = on_transcript
        self.on_done = on_done
        self.dispatch = dispatch
        self._thread = None
        self._process = None
        self._cancel_requested = False

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def run(self, workdir, staged_board, mode, model=None, effort=None,
            extra=""):
        if self.is_running():
            raise RuntimeError("a controlled Codex placement run is active")
        self._cancel_requested = False
        self._thread = threading.Thread(
            target=self._work,
            args=(os.path.abspath(workdir), os.path.basename(staged_board),
                  mode, model, effort, extra), daemon=True)
        self._thread.start()

    def cancel(self):
        self._cancel_requested = True
        process = self._process
        if process is not None:
            try:
                process.terminate()
            except OSError:
                pass

    def _emit(self, text):
        self.dispatch(self.on_transcript, text)

    def _done(self, result, error):
        self.dispatch(self.on_done, result, error)

    def _work(self, workdir, board, mode, model, effort, extra):
        try:
            os.makedirs(os.path.join(workdir, "wk"), exist_ok=True)
            history = []
            resources = {}
            if mode == "place_route":
                initial = {
                    "action": "run",
                    "script": (".claude/skills/plan-pcb-placement-and-routing/"
                               "scripts/loop_driver.py"),
                    "args": ["--stage", "L1", "--board", board,
                             "--ledger", "ledger.jsonl", "--no-delegate"],
                }
            else:
                initial = {
                    "action": "run",
                    "script": (".claude/skills/plan-pcb-placement/scripts/"
                               "placement_driver.py"),
                    "args": ["--stage", "P0", "--board", board],
                }
            output = self._execute(initial, workdir)
            history.append({"request": initial, "result": output})

            contract = _decision_contract(mode, workdir, extra)
            for turn in range(1, _MAX_TURNS + 1):
                if self._cancel_requested:
                    self._done(None, "Cancelled.")
                    return
                prompt = self._prompt(contract, turn, workdir, history,
                                      resources)
                response, error = self._ask_codex(prompt, model, effort)
                if error:
                    self._done(None, error)
                    return
                decision, error = self._parse_decision(response)
                if error:
                    history.append({"decision_error": error,
                                    "model_response": response[-4000:]})
                    self._emit(f"  [decision rejected] {error}\n")
                    continue
                action = decision.get("action")
                if action == "finish":
                    result, error = self._finish_result(
                        decision.get("result"), workdir)
                    if error:
                        history.append({"request": decision,
                                        "decision_error": error})
                        self._emit(f"  [finish rejected] {error}\n")
                        continue
                    self._done("RESULT=" + json.dumps(
                        result, ensure_ascii=False, separators=(",", ":")), None)
                    return
                try:
                    output = self._execute(decision, workdir,
                                           resources=resources)
                except (OSError, ValueError) as exc:
                    output = {"ok": False, "error": str(exc)}
                    self._emit(f"  [action rejected] {exc}\n")
                history.append({"request": decision, "result": output})
                history = history[-6:]
            self._done(None, f"Codex placement exceeded {_MAX_TURNS} decisions")
        except Exception as exc:
            self._done(None, f"Controlled Codex placement failed: {exc}")

    def _prompt(self, contract, turn, workdir, history, resources):
        context = {
            "turn": turn,
            "artifacts": _manifest(workdir),
            "recent_actions": history,
            "loaded_resources": resources,
        }
        encoded = json.dumps(context, ensure_ascii=False, separators=(",", ":"))
        if len(encoded) > _MAX_CONTEXT:
            context["loaded_resources"] = {
                name: (text if len(text) <= 120_000 else
                       text[:60_000] + "\n...[middle omitted]...\n" +
                       text[-60_000:])
                for name, text in resources.items()}
            while len(context["recent_actions"]) > 1:
                encoded = json.dumps(context, ensure_ascii=False,
                                     separators=(",", ":"))
                if len(encoded) <= _MAX_CONTEXT:
                    break
                context["recent_actions"].pop(0)
            encoded = json.dumps(context, ensure_ascii=False,
                                 separators=(",", ":"))
        return ("<LOCAL_PLACEMENT_CONTEXT>\n" + encoded +
                "\n</LOCAL_PLACEMENT_CONTEXT>\n" + contract)

    def _ask_codex(self, prompt, model, effort):
        cmd = self.backend.build_cmd(self.cli_path, prompt, model=model,
                                     effort=effort)
        state = self.backend.stream_state()
        kwargs = {"cwd": ROOT_DIR, "stdin": subprocess.PIPE,
                  "stdout": subprocess.PIPE, "stderr": subprocess.PIPE,
                  "text": True, "encoding": "utf-8", "errors": "replace",
                  "bufsize": 1}
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW
        self._process = subprocess.Popen(cmd, **kwargs)
        self._process.stdin.write(prompt)
        self._process.stdin.close()
        stderr = []
        drain = threading.Thread(
            target=lambda: stderr.append(self._process.stderr.read()),
            daemon=True)
        drain.start()
        for line in self._process.stdout:
            try:
                event = json.loads(line)
            except (json.JSONDecodeError, ValueError):
                continue
            text = state.feed(event)
            if text and event.get("type") == "item.completed":
                self._emit("Codex decision: " + text)
        self._process.wait()
        drain.join(timeout=5)
        code = self._process.returncode
        self._process = None
        if self._cancel_requested:
            return None, "Cancelled."
        return state.finish(code, "".join(stderr))

    @staticmethod
    def _parse_decision(response):
        value = extract_result_line(response or "")
        if value is None:
            return None, "no RESULT=<JSON> decision"
        try:
            decision = json.loads(value)
        except (json.JSONDecodeError, ValueError) as exc:
            return None, f"decision is not JSON: {exc}"
        if not isinstance(decision, dict):
            return None, "decision is not an object"
        if decision.get("action") not in {
                "run", "read_artifact", "read_resource", "finish"}:
            return None, f"unknown action {decision.get('action')!r}"
        return decision, None

    def _execute(self, decision, workdir, resources=None):
        action = decision.get("action")
        if action == "read_resource":
            if resources is None:
                raise ValueError("resource storage is unavailable")
            name = str(decision.get("name") or "")
            path = _RESOURCES.get(name)
            if not path:
                raise ValueError(f"unknown resource {name!r}")
            text = _read_text(path, _MAX_CONTEXT)
            resources[name] = text
            self._emit(f"  -> resource: {name}\n")
            return {"ok": True, "resource": name, "characters": len(text)}
        if action == "read_artifact":
            relative = self._safe_relative(decision.get("path"))
            path = os.path.join(workdir, relative)
            if not _inside(path, workdir) or not os.path.isfile(path):
                raise ValueError(f"artifact not found: {relative}")
            if os.path.splitext(path)[1].lower() not in _TEXT_EXTENSIONS:
                raise ValueError("only text artifacts may be read")
            text = _read_text(path)
            self._emit(f"  -> read: {relative}\n")
            return {"ok": True, "path": relative, "content": text}
        if action != "run":
            raise ValueError(f"cannot execute action {action!r}")

        script_name = self._safe_relative(decision.get("script"))
        script = os.path.abspath(os.path.join(ROOT_DIR, script_name))
        if (not script.lower().endswith(".py") or not os.path.isfile(script)
                or not any(_inside(script, root) for root in _SCRIPT_ROOTS)):
            raise ValueError(f"script is not allowed: {script_name}")
        args = decision.get("args") or []
        if not isinstance(args, list) or len(args) > 200:
            raise ValueError("args must be a list of at most 200 strings")
        clean_args = []
        value_flag = None
        for value in args:
            text = str(value)
            if text.startswith("--"):
                value_flag = text.split("=", 1)[0]
            clean_args.append(self._safe_arg(
                value, workdir, allow_slash=value_flag in _NET_VALUE_FLAGS))
        requested_env = decision.get("env") or {}
        if not isinstance(requested_env, dict):
            raise ValueError("env must be an object")
        env = os.environ.copy()
        python_paths = [ROOT_DIR, os.path.join(ROOT_DIR, "py_router"),
                        os.path.join(ROOT_DIR, "py_placer"),
                        os.path.join(ROOT_DIR, "py_tools")]
        env["PYTHONPATH"] = os.pathsep.join(
            python_paths + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
        for name, value in requested_env.items():
            if not _ENV_NAME.match(str(name)):
                raise ValueError(f"environment name is not allowed: {name!r}")
            env[str(name)] = str(value)
        python = _python_executable()
        if not python or not os.path.isfile(python):
            raise OSError("KiCad Python executable was not found")
        command = [python, "-X", "utf8", script, *clean_args]
        shown = " ".join([os.path.relpath(script, ROOT_DIR), *clean_args])
        self._emit(f"  -> local Python: {shown}\n")
        kwargs = {"cwd": workdir, "stdout": subprocess.PIPE,
                  "stderr": subprocess.STDOUT, "text": True,
                  "encoding": "utf-8", "errors": "replace", "env": env,
                  "bufsize": 1}
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW
        self._process = subprocess.Popen(command, **kwargs)
        chunks = []
        for line in self._process.stdout:
            chunks.append(line)
            self._emit(line)
            if self._cancel_requested:
                self._process.terminate()
                break
        self._process.wait()
        code = self._process.returncode
        self._process = None
        output = "".join(chunks)
        return {"ok": code == 0, "exit_code": code,
                "output": output[-_MAX_OUTPUT:]}

    @staticmethod
    def _safe_relative(value):
        if not isinstance(value, str) or not value.strip():
            raise ValueError("a non-empty relative path is required")
        value = value.replace("\\", "/")
        if re.match(r"^[A-Za-z]:", value) or value.startswith("/"):
            raise ValueError("absolute paths are forbidden")
        parts = [part for part in value.split("/") if part not in ("", ".")]
        if ".." in parts:
            raise ValueError("parent path traversal is forbidden")
        return os.path.join(*parts)

    @staticmethod
    def _safe_arg(value, workdir, allow_slash=False):
        if not isinstance(value, (str, int, float)):
            raise ValueError("arguments must be strings or numbers")
        value = str(value)
        if any(char in value for char in ("\x00", "\r", "\n")):
            raise ValueError("control characters are forbidden in arguments")
        normalized = value.replace("\\", "/")
        if ".." in normalized.split("/"):
            raise ValueError("parent path traversal is forbidden in arguments")
        if (re.match(r"^[A-Za-z]:", normalized) or normalized.startswith("//")
                or (normalized.startswith("/") and not allow_slash)):
            candidate = os.path.abspath(value)
            if not _inside(candidate, workdir):
                raise ValueError("absolute argument path is outside the workdir")
            return os.path.relpath(candidate, workdir)
        return value

    @staticmethod
    def _finish_result(result, workdir):
        if not isinstance(result, dict):
            return None, "finish.result must be an object"
        status = result.get("status")
        if status not in ("complete", "residue", "refused"):
            return None, f"invalid finish status {status!r}"
        normalized = dict(result)
        for key in ("board", "movie", "report"):
            value = normalized.get(key)
            if not value:
                normalized[key] = None
                continue
            if not isinstance(value, str):
                return None, f"{key} path must be a string or null"
            path = value if os.path.isabs(value) else os.path.join(workdir, value)
            path = os.path.abspath(path)
            if not _inside(path, workdir) or not os.path.isfile(path):
                return None, f"{key} does not exist inside the workdir: {value}"
            normalized[key] = path
        if status != "refused" and normalized.get("board") is None:
            return None, "complete/residue requires an existing board"
        blocking = normalized.get("blocking")
        if blocking is not None and not isinstance(blocking, int):
            return None, "blocking must be an integer or null"
        normalized["summary"] = str(normalized.get("summary") or "")
        return normalized, None
