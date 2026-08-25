#!/usr/bin/env python3
"""Cloud plan+route: upload a KiCad board, an LLM agent plans and routes it
on Modal, download the routed board.

    modal run tests/stress/modal_route/route_app.py --board my_board.kicad_pcb

Prereqs (one-time):
  1. modal deploy tests/stress/modal_route/vllm_app.py     (the LLM half)
  2. modal secret create kicad-llm-token KICAD_LLM_TOKEN=<same token>

The board (plus its .kicad_pro/.kicad_prl/.kicad_dru siblings, if present) is
shipped as bytes into a CPU container built from a clean checkout of HEAD --
the same image discipline as the parameter sweep. Inside, a headless
`opencode run` agent (the stress harness's #503 backend) pointed at the vLLM
endpoint follows the repo's own skills: plan-pcb-routing to produce the
plan, then executes the chain (fanout / diff pairs / planes / signal), and
verifies with check_drc + check_connected, consulting
diagnose-routing-failures on retries. Every board-mutating tool
self-records to a redo manifest (REDO_MANIFEST), so the run comes back
REPLAYABLE, exactly like a stress-corpus board.

Returned and written next to the input (or --out DIR):
  <name>.routed.kicad_pcb / .kicad_pro    the routed board + its project
  redo_commands.sh                        deterministic replay manifest
  SUMMARY.md                              the agent's plan + outcome report
  agent.log                               full agent transcript
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import modal

REPO = "/opt/kicad-routing-tools"
WORK = "/work"

app = modal.App("kicad-route-board")

# --- image: the sweep's clean-checkout recipe + the opencode CLI ------------
_here = Path(__file__).resolve().parent
_repo_root = _here.parent.parent.parent
sys.path.insert(0, str(_repo_root / "tests" / "stress" / "modal_sweep"))

if modal.is_local():
    from modal_app import _image_source  # reuse the sweep's pinned-commit source
    _src_dir, GIT_SHA = _image_source()
else:
    _src_dir, GIT_SHA = "/tmp", os.environ.get("KICAD_SWEEP_GIT", "unknown")

image = (
    modal.Image.debian_slim(python_version="3.12")
    .pip_install("numpy>=1.21.0", "scipy>=1.7.0", "shapely>=1.8.0")
    .apt_install("curl", "procps", "git", "unzip")
    .env({"KICAD_SWEEP_GIT": GIT_SHA})
    # opencode: the OpenAI-compatible-agent CLI the stress harness already
    # supports as a backend (#503). Standalone installer, no node needed.
    .run_commands("curl -fsSL https://opencode.ai/install | bash",
                  "ln -sf /root/.opencode/bin/opencode /usr/local/bin/opencode")
    .add_local_dir(_src_dir, REPO, copy=True, ignore=[
        "**/.git/**", "**/__pycache__/**", "**/target/**", "**/.claude/worktrees/**",
    ])
    .run_commands(
        f"cd {REPO} && python3 build_router.py",
        f"cd {REPO} && python3 -c \"import sys; sys.path.insert(0,'rust_router'); "
        f"import grid_router; print('grid_router', grid_router.__version__)\"",
    )
)

PROMPT = """You are a non-interactive PCB routing agent. Route ONE user-supplied
KiCad board: {board_path}

Rules:
- You cannot ask questions. Decide and act.
- FIRST read {repo}/.claude/skills/plan-pcb-routing/SKILL.md and follow it to
  analyze this board and produce a concrete step plan (fanout needs, diff
  pairs, power/plane nets, then signal routing). Write the plan to
  {work}/out/SUMMARY.md before executing it.
- Execute the plan with this repo's CLI tools (python3 {repo}/py_router/...).
  Work in {work}/ so intermediates stay out of the repo.
- Verify with check_connected.py AND check_drc.py at the clearance you routed
  (a DRC-clean board can still be disconnected -- run both). On failures,
  read {repo}/.claude/skills/diagnose-routing-failures/SKILL.md and retry
  with its guidance. Prefer retry-and-keep-better.
- Finish by copying the best board to {work}/out/routed.kicad_pcb using
  python3 {repo}/py_router/copy_board.py (NEVER bare cp -- the .kicad_pro
  sibling carries the DRC floor), and append the final check_drc /
  check_connected numbers to {work}/out/SUMMARY.md.
{notes}"""


@app.function(image=image, cpu=4.0, memory=8192, timeout=4 * 3600,
              secrets=[modal.Secret.from_name("kicad-llm-token")])
def route_board(files: dict, endpoint: str, model: str, notes: str = "") -> dict:
    inp = Path(WORK) / "in"
    out = Path(WORK) / "out"
    inp.mkdir(parents=True, exist_ok=True)
    out.mkdir(parents=True, exist_ok=True)
    board_path = None
    for name, data in files.items():
        p = inp / name
        p.write_bytes(data)
        if name.endswith(".kicad_pcb"):
            board_path = p

    # opencode provider config -> the vLLM endpoint
    cfg_dir = Path.home() / ".config" / "opencode"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    (cfg_dir / "opencode.json").write_text(json.dumps({
        "provider": {
            "modalvllm": {
                "npm": "@ai-sdk/openai-compatible",
                "options": {"baseURL": endpoint.rstrip("/") + "/v1",
                            "apiKey": os.environ["KICAD_LLM_TOKEN"]},
                "models": {model: {"name": model}},
            }
        },
    }, indent=1))

    env = dict(os.environ)
    env["REDO_MANIFEST"] = f"{WORK}/redo_commands.sh"
    env["REDO_RUNDIR"] = WORK
    prompt = PROMPT.format(board_path=board_path, repo=REPO, work=WORK,
                           notes=(f"- Extra instructions from the user: {notes}"
                                  if notes else ""))
    t0 = time.time()
    proc = subprocess.run(
        ["opencode", "run", "-m", f"modalvllm/{model}", prompt],
        cwd=REPO, env=env, capture_output=True, text=True,
        timeout=3 * 3600 + 1800)
    wall = time.time() - t0

    result = {"agent_rc": proc.returncode, "wall_s": round(wall, 1),
              "files": {}}
    result["files"]["agent.log"] = (proc.stdout + "\n--- stderr ---\n"
                                    + proc.stderr).encode()
    for p in sorted(out.glob("*")):
        if p.is_file():
            result["files"][p.name] = p.read_bytes()
    man = Path(env["REDO_MANIFEST"])
    if man.exists():
        result["files"]["redo_commands.sh"] = man.read_bytes()
    return result


@app.local_entrypoint()
def main(board: str, endpoint: str = "", model: str = "kicad-router-llm",
         out: str = "", notes: str = ""):
    src = Path(board)
    if not src.exists():
        raise SystemExit(f"no such board: {board}")
    if not endpoint:
        # default: the deployed vllm_app's web URL
        fn = modal.Function.from_name("kicad-route-llm", "serve")
        endpoint = fn.get_web_url()
        print(f"using deployed LLM endpoint: {endpoint}")
    files = {src.name: src.read_bytes()}
    for suffix in (".kicad_pro", ".kicad_prl", ".kicad_dru"):
        sib = src.with_suffix(suffix)
        if sib.exists():
            files[sib.name] = sib.read_bytes()
    print(f"uploading {len(files)} file(s), {sum(len(v) for v in files.values())/1e6:.1f} MB")
    res = route_board.remote(files, endpoint, model, notes)
    dest = Path(out) if out else src.parent / f"{src.stem}_routed"
    dest.mkdir(parents=True, exist_ok=True)
    for name, data in res["files"].items():
        (dest / name).write_bytes(data)
    print(f"agent rc={res['agent_rc']}  wall={res['wall_s']}s")
    print(f"wrote {len(res['files'])} file(s) -> {dest}/")
    summary = dest / "SUMMARY.md"
    if summary.exists():
        print("\n--- SUMMARY.md (tail) ---")
        print("\n".join(summary.read_text().splitlines()[-25:]))
