#!/usr/bin/env python3
"""#925: a quoting-heavy prompt must never reach cmd.exe, on EITHER backend.

    python3 tests/test_925_windows_shim_prompt.py

The bug: on Windows `shutil.which("opencode")` resolves to the npm shim
`opencode.cmd`, Popen launches a `.cmd` through an implicit cmd.exe, and
`list2cmdline` escapes embedded quotes as `\\"` -- which cmd's tokenizer does
not understand. The first quote in the ~3 kB skill prompt ends the quoted
region and every `|` after it becomes a shell pipe, so the launch dies with
"The system cannot find the file specified." and no session is ever created.

The runner already moved the prompt to stdin for a `.cmd`, but it did so by
searching the argv for Claude's `-p`. No opencode command line contains one:
the lookup raised, an `except ValueError: pass` swallowed it, and opencode kept
passing the prompt through cmd.exe. That is why the split now lives on the
BACKEND -- and why the negative control below drives the OLD rule against the
opencode argv and asserts it finds nothing.

Also covers #926, the hole found while verifying #925: `opencode run --agent
<unknown>` does not fail. It warns on stderr and runs on the DEFAULT agent at
exit 0, which silently voids the read-only pin that `build_cmd`'s write refusal
rests on.
"""
import json
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_routing_plugin import ai_backend as ab  # noqa: E402

FAILURES = []


def check(cond, name, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}"
          + (f" -- {detail}" if detail and not cond else ""))
    if not cond:
        FAILURES.append(name)


# A prompt with the three characters that break cmd.exe tokenizing, plus the
# em dash the real skill_prompt() puts between args and instructions.
PROMPT = 'Load the skill for: ' + '"' * 4 + "'" * 4 + '|' * 4 + ' \u2014 go'


def _patch(monkey):
    """Swap module globals, returning a restore callable."""
    old = {k: getattr(ab, k) if hasattr(ab, k) else None for k in monkey}
    for k, v in monkey.items():
        setattr(ab, k, v)
    return lambda: [setattr(ab, k, v) for k, v in old.items()]


class _FakeOs:
    """os with a chosen `name`; everything else is the real module."""
    def __init__(self, name):
        self.name = name

    def __getattr__(self, item):
        return getattr(os, item)


def test_the_mechanism_this_whole_fix_exists_for():
    """cmd.exe cannot read what list2cmdline writes."""
    print("--- test_the_mechanism_this_whole_fix_exists_for")
    line = subprocess.list2cmdline(["opencode.CMD", "run", "--", PROMPT])
    check('\\"' in line,
          "list2cmdline escapes the prompt's quotes as \\\" (cmd cannot parse)",
          line[:80])
    # The pipes land INSIDE what cmd would treat as unquoted text once that
    # first \" has closed the region -- which is the actual failure.
    check(line.count('|') == 4, "the pipes survive into the command line")


def test_find_cli_prefers_a_real_exe_over_a_windows_shim():
    print("--- test_find_cli_prefers_a_real_exe_over_a_windows_shim")
    with tempfile.TemporaryDirectory() as tmp:
        exe = os.path.join(tmp, "opencode.exe")
        with open(exe, "w") as fh:
            fh.write("")
        os.chmod(exe, 0o755)
        b = ab.OpencodeBackend()
        b.candidates = (exe,)
        restore = _patch({
            "os": _FakeOs("nt"),
            "shutil": type("S", (), {"which": staticmethod(
                lambda _n: r"C:\Users\x\AppData\Roaming\npm\opencode.CMD")})(),
        })
        try:
            check(b.find_cli() == exe,
                  "a .CMD on PATH loses to a real .exe candidate", b.find_cli())
        finally:
            restore()


def test_the_shim_is_still_returned_when_nothing_better_exists():
    """The reorder must not turn a working install into 'not found'."""
    print("--- test_the_shim_is_still_returned_when_nothing_better_exists")
    shim = r"C:\Users\x\AppData\Roaming\npm\opencode.CMD"
    b = ab.OpencodeBackend()
    b.candidates = (os.path.join(tempfile.gettempdir(), "no-such-opencode"),)
    restore = _patch({
        "os": _FakeOs("nt"),
        "shutil": type("S", (), {"which": staticmethod(lambda _n: shim)})(),
    })
    try:
        check(b.find_cli() == shim,
              "with no real exe, the shim is still found (no regression)",
              str(b.find_cli()))
    finally:
        restore()
    # ...and genuinely-absent stays absent.
    restore = _patch({
        "os": _FakeOs("nt"),
        "shutil": type("S", (), {"which": staticmethod(lambda _n: None)})(),
    })
    try:
        check(b.find_cli() is None, "a CLI that is not installed is still None")
    finally:
        restore()


def test_posix_resolution_is_untouched():
    print("--- test_posix_resolution_is_untouched")
    b = ab.OpencodeBackend()
    restore = _patch({
        "os": _FakeOs("posix"),
        "shutil": type("S", (), {"which": staticmethod(
            lambda _n: "/opt/homebrew/bin/opencode")})(),
    })
    try:
        check(b.find_cli() == "/opt/homebrew/bin/opencode",
              "on POSIX, which() still wins outright")
    finally:
        restore()


def test_the_old_rule_finds_nothing_in_an_opencode_argv():
    """The NEGATIVE CONTROL: this is exactly why the bug shipped."""
    print("--- test_the_old_rule_finds_nothing_in_an_opencode_argv")
    cmd = ab.OpencodeBackend().build_cmd("opencode.CMD", PROMPT)
    check("-p" not in cmd,
          "no opencode argv contains Claude's -p, so the old lookup raised")
    check(cmd[-1] == PROMPT and cmd[-2] == "--",
          "the prompt is the trailing argv element after --", str(cmd[-2:])[:80])


def test_each_backend_splits_its_own_prompt_onto_stdin():
    print("--- test_each_backend_splits_its_own_prompt_onto_stdin")
    for backend, tail in ((ab.OpencodeBackend(), ["--"]),
                          (ab.ClaudeBackend(), None)):
        cmd = backend.build_cmd("cli.CMD", PROMPT)
        out, payload = backend.stdin_prompt(cmd, PROMPT)
        check(payload == PROMPT,
              f"{backend.id}: the prompt is handed to stdin")
        check(PROMPT not in out,
              f"{backend.id}: no argv element still carries the prompt")
        check(all('"' not in a and '|' not in a for a in out),
              f"{backend.id}: the remaining argv is free of cmd metacharacters")
        if tail is not None:
            check(out[-1] == "--",
                  f"{backend.id}: the bare -- is left in place (it still parses)",
                  str(out[-2:]))
    # The base class declares "argv only" rather than raising, so an unknown
    # backend degrades to today's behaviour instead of crashing the launch.
    base_cmd = ["cli.CMD", "--", PROMPT]
    check(ab.AIBackend().stdin_prompt(base_cmd, PROMPT) == (base_cmd, None),
          "the base backend declares no stdin channel")


def test_the_split_does_not_mutate_the_caller_s_list():
    print("--- test_the_split_does_not_mutate_the_caller_s_list")
    for backend in (ab.OpencodeBackend(), ab.ClaudeBackend()):
        cmd = backend.build_cmd("cli.CMD", PROMPT)
        before = list(cmd)
        backend.stdin_prompt(cmd, PROMPT)
        check(cmd == before,
              f"{backend.id}: stdin_prompt leaves the caller's argv alone")


# ------------------------------------------------- the agent-fallback hole


def test_an_undeclared_agent_is_refused_before_any_model_call():
    print("--- test_an_undeclared_agent_is_refused_before_any_model_call")
    check(ab.opencode_agent_declared(),
          "the repo's own opencode.json really does declare the agent")
    with tempfile.TemporaryDirectory() as tmp:
        gone = os.path.join(tmp, "opencode.json")
        with open(gone, "w") as fh:
            json.dump({"agent": {"something-else": {}}}, fh)
        restore = _patch({"OPENCODE_CONFIG": gone})
        try:
            check(not ab.opencode_agent_declared(),
                  "a config without the pinned agent is detected")
            try:
                ab.OpencodeBackend().build_cmd("opencode", PROMPT)
                check(False, "build_cmd refuses an undeclared agent")
            except ValueError as e:
                check("DEFAULT agent" in str(e),
                      "build_cmd refuses, naming the fallback", str(e)[:90])
        finally:
            restore()
    # An unreadable config must NOT be turned into a refusal of our own.
    restore = _patch({"OPENCODE_CONFIG": os.path.join(tempfile.gettempdir(),
                                                      "nope-925.json")})
    try:
        check(ab.opencode_agent_declared(),
              "an unreadable config defers to opencode's own error")
    finally:
        restore()


def test_a_run_that_fell_back_to_the_default_agent_is_discarded():
    print("--- test_a_run_that_fell_back_to_the_default_agent_is_discarded")
    # Exactly what opencode 1.18.9 writes, ANSI codes and all.
    stderr = ('\x1b[93m\x1b[1m! \x1b[0m agent "pcb-analysis" not found. '
              'Falling back to default agent\n')
    st = ab._OpencodeStreamState()
    st.feed({"type": "text", "part": {"text": "the board looks fine"}})
    result, error = st.finish(0, stderr)
    check(result is None,
          "a successful reply is DISCARDED when the pin did not hold", str(result))
    check(error and "default agent" in error.lower(),
          "the error says the read-only pin was not in force", str(error)[:90])
    # And the ordinary path is untouched.
    st2 = ab._OpencodeStreamState()
    st2.feed({"type": "text", "part": {"text": "RESULT=ok"}})
    r2, e2 = st2.finish(0, "")
    check(r2 == "RESULT=ok" and e2 is None,
          "a normal run still returns its reply", f"{r2!r} {e2!r}")


def main():
    for fn in sorted([v for k, v in globals().items()
                      if k.startswith("test_") and callable(v)],
                     key=lambda f: f.__code__.co_firstlineno):
        fn()
    print()
    if FAILURES:
        print(f"FAIL: #925 windows shim/prompt, {len(FAILURES)} failure(s): "
              + ", ".join(FAILURES))
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
