#!/usr/bin/env python3
"""A fake `claude` CLI for the placement fake-run gate.

Receives the EXACT argv ClaudeBackend.build_cmd produced (the test prepends
this script to it), asserts the placement-run contract (write-capable
allowlist, --add-dir, a /plan-pcb-placement prompt), then plays a canned run:
stream-json init, seed 0 in the skill's layout (s0.kicad_pcb + s0.json,
the fast-routed s0_fast.kicad_pcb and its s0_fast.score.json),
a milestone `converge.py record` tool line + ledger row, left to sit long
enough for the monitor to fill the candidates table and render the row's
preview, then REPORT.md, final.kicad_pcb (the kept seed), and a terminal
result event ending with a valid RESULT= line.
"""
import json
import os
import shutil
import sys
import time


def emit(event):
    sys.stdout.write(json.dumps(event) + "\n")
    sys.stdout.flush()


def main():
    argv = sys.argv[1:]
    if "-p" not in argv or "--add-dir" not in argv:
        sys.stderr.write("fake_claude: missing -p or --add-dir\n")
        return 2
    i = argv.index("-p")
    nxt = argv[i + 1] if i + 1 < len(argv) else ""
    if not nxt or nxt.startswith("--"):
        # The npm .cmd path: AISkillRunner strips the prompt from argv (cmd.exe
        # mangles embedded quotes) and pipes it on stdin, like the real CLI
        # accepts. Read it there.
        prompt = sys.stdin.read()
    else:
        prompt = nxt
    workdir = argv[argv.index("--add-dir") + 1]
    allowed = argv[argv.index("--allowedTools") + 1] if "--allowedTools" in argv else ""
    for tool in ("Write", "Edit", "Bash"):
        if tool not in allowed.split(","):
            sys.stderr.write(f"fake_claude: {tool} missing from allowlist: {allowed}\n")
            return 2
    if ("input.kicad_pcb" not in prompt or "--no-delegate" in prompt
            or "/plan-pcb-placement " not in prompt):
        # Both tab modes run the placement skill on the board staged into
        # the workdir; no retired loop-driver flag.
        sys.stderr.write(f"fake_claude: unexpected prompt: {prompt[:200]}\n")
        return 2
    staged = os.path.join(workdir, "input.kicad_pcb")
    if not os.path.isfile(staged):
        sys.stderr.write(f"fake_claude: staged board missing: {staged}\n")
        return 2
    # Dump the received argv (prompt elided) so the test can assert flags
    # like --model/--effort actually reached the CLI.
    with open(os.path.join(workdir, "argv.json"), "w") as f:
        json.dump([a if a is not prompt else "<prompt>" for a in argv], f)

    emit({"type": "system", "subtype": "init", "model": "fake-model",
          "claude_code_version": "0.0-test", "cwd": workdir,
          "skills": ["plan-pcb-placement"]})
    # Seed 0, in the skill's run layout (SKILL.md section 2).
    emit({"type": "assistant", "message": {"content": [
        {"type": "tool_use", "name": "Bash",
         "input": {"description": "Place seed 0",
                   "command": "python3 -X utf8 py_placer/place_simple.py "
                              "fixed.kicad_pcb s0.kicad_pcb --seed 0 "
                              "--json s0.json"}}]}})
    shutil.copyfile(staged, os.path.join(workdir, "s0.kicad_pcb"))
    with open(os.path.join(workdir, "s0.json"), "w") as f:
        json.dump({"final": {"cost": 1.0}, "illegal": []}, f)
    shutil.copyfile(staged, os.path.join(workdir, "s0_fast.kicad_pcb"))
    with open(os.path.join(workdir, "s0_fast.score.json"), "w") as f:
        json.dump({"blocking": 0, "quality": {
            "vias": 12, "copper_mm": 345.6, "segments": 210}}, f)

    # Both keys, like real Claude Code events. The skill records each
    # milestone in the converge ledger, and the ledger row written below is
    # the stage line's last milestone.
    emit({"type": "assistant", "message": {"content": [
        {"type": "tool_use", "name": "Bash",
         "input": {"description": "Record the milestone board",
                   "command": "python3 -X utf8 py_placer/converge.py record "
                              "--ledger ledger.jsonl --board s0.kicad_pcb "
                              "--kind quench --lever nudge"}}]}})

    # Then sit quietly long enough for the monitor (1 s ticks, candidates
    # rescanned every 2nd tick) to fill the table and render the preview.
    with open(os.path.join(workdir, "ledger.jsonl"), "a") as f:
        f.write(json.dumps({"iteration": 1, "kind": "quench",
                            "lever": "nudge"}) + "\n")
    emit({"type": "assistant", "message": {"content": [
        {"type": "text", "text": "Seed 0 recorded."}]}})
    time.sleep(8)

    with open(os.path.join(workdir, "REPORT.md"), "w") as f:
        f.write("# Fake placement report\n\nblocking 0 -> 0 (canned).\n")
    final = os.path.join(workdir, "final.kicad_pcb")
    shutil.copyfile(os.path.join(workdir, "s0.kicad_pcb"), final)   # the kept seed

    result = json.dumps({
        "status": "complete", "board": final.replace("\\", "/"),
        "report": os.path.join(workdir, "REPORT.md").replace("\\", "/"),
        "blocking": 0, "summary": "canned run"})
    emit({"type": "result", "is_error": False,
          "result": f"Canned run complete.\nRESULT={result}"})
    return 0


if __name__ == "__main__":
    sys.exit(main())
