"""Placement tab headless contracts (placement_run.py + ai_backend kwargs).

Headless tests (no wx, no CLI, no pcbnew): workdir creation/staging, the
instruction + RESULT= contracts, the monitor's pollers (ledger tail, board
artifacts, stage derivation), the scavenge fallback, and the build_cmd
allowed_tools/add_dirs extension the placement runs depend on.
"""
import json
import os
import shutil
import sys
import tempfile
import types

sys.modules.setdefault('wx', types.ModuleType('wx'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..',
                                'kicad_routing_plugin'))

import ai_backend  # noqa: E402
import placement_run  # noqa: E402
from placement_run import (  # noqa: E402
    PLACEMENT_ALLOWED_TOOLS, PLACEMENT_RESULT_SCHEMA, PLACEMENT_SKILLS,
    build_placement_instructions, build_placement_prompt, create_workdir,
    derive_stage, list_board_artifacts, newest_stable_board,
    parse_placement_result, read_ledger_tail, scan_workdir_outputs,
    stage_inputs,
)

FAILURES = []


def check(name, cond, detail=""):
    status = "ok" if cond else "FAIL"
    print(f"[{status}] {name}" + (f" -- {detail}" if detail and not cond else ""))
    if not cond:
        FAILURES.append(name)


TMP = tempfile.mkdtemp(prefix="krt_placement_run_test_")

# ------------------------------------------------------- workdir + staging

board_dir = os.path.join(TMP, "boards")
os.makedirs(board_dir)
board = os.path.join(board_dir, "demo.kicad_pcb")
open(board, "w").write("(kicad_pcb (version 20260206))\n")
open(os.path.join(board_dir, "demo.kicad_pro"), "w").write("{}\n")
open(os.path.join(board_dir, "demo.kicad_dru"), "w").write("(version 1)\n")

wk = create_workdir(board, "place")
check("workdir under board dir",
      os.path.isdir(wk) and wk.startswith(os.path.join(board_dir, "krt_placement")),
      wk)
check("workdir name carries mode", os.path.basename(wk).endswith("_place"), wk)
wk_tmp = create_workdir(None, "place_route")
check("workdir tempdir fallback",
      os.path.isdir(wk_tmp)
      and wk_tmp.startswith(os.path.join(tempfile.gettempdir(), "krt_placement")),
      wk_tmp)
wk_again = create_workdir(board, "place")
check("same-second restart gets a FRESH workdir (no dirty reuse)",
      wk_again != wk and os.path.isdir(wk_again), wk_again)

snapshot = os.path.join(TMP, "snapshot.kicad_pcb")
open(snapshot, "w").write("(kicad_pcb (version 20260206) snapshot)\n")
staged = stage_inputs(wk, snapshot, board)
check("staged board path", staged == os.path.join(wk, "input.kicad_pcb"))
check("staged board content", "snapshot" in open(staged).read())
check("staged .kicad_pro sibling",
      os.path.isfile(os.path.join(wk, "input.kicad_pro")))
check("staged .kicad_dru sibling",
      os.path.isfile(os.path.join(wk, "input.kicad_dru")))

# ------------------------------------------------------------- instructions

instr = build_placement_instructions(wk, "place")
check("instructions name the workdir", wk.replace("\\", "/") in instr)
check("instructions carry the RESULT schema", PLACEMENT_RESULT_SCHEMA in instr)
check("place mode has no --no-delegate", "--no-delegate" not in instr)
check("instructions refuse copper stripping", '"refused"' in instr)
instr_pr = build_placement_instructions(wk, "place_route", extra="Focus on U1.")
check("place_route mode has --no-delegate", "--no-delegate" in instr_pr)
check("extra instructions included", "Focus on U1." in instr_pr)

prompt = build_placement_prompt(ai_backend.BACKENDS["claude"], wk, staged, "place")
check("prompt invokes the placement skill",
      prompt.startswith("/" + PLACEMENT_SKILLS["place"] + " "), prompt[:60])
check("prompt names the staged board", staged.replace("\\", "/") in prompt)

# ------------------------------------------------------------------- RESULT

final_board = os.path.join(wk, "final.kicad_pcb")
open(final_board, "w").write("(kicad_pcb final)\n")
report_md = os.path.join(wk, "REPORT.md")
open(report_md, "w").write("# report\n")

ok_payload = json.dumps({
    "status": "complete", "board": final_board.replace("\\", "/"),
    "movie": None, "report": report_md, "blocking": 0, "summary": "done"})
out, errs = parse_placement_result(ok_payload)
check("result parses", out is not None and not errs, str(errs))
check("result board normalized",
      out and os.path.samefile(out["board"], final_board))
check("result blocking int", out and out["blocking"] == 0)
check("result report found", out and out["report"] and os.path.isfile(out["report"]))

out, errs = parse_placement_result(None)
check("no RESULT line rejected", out is None and errs)
out, errs = parse_placement_result("not json {")
check("bad JSON rejected", out is None and errs)
out, errs = parse_placement_result(json.dumps({"status": "done", "board": final_board}))
check("unknown status rejected", out is None and errs)
out, errs = parse_placement_result(json.dumps(
    {"status": "complete", "board": os.path.join(wk, "missing.kicad_pcb")}))
check("missing board rejected", out is None and errs)
out, errs = parse_placement_result(json.dumps(
    {"status": "refused", "board": None, "summary": "routed copper"}))
check("refused needs no board", out is not None and out["status"] == "refused",
      str(errs))
out, errs = parse_placement_result(json.dumps(
    {"status": "complete", "board": final_board,
     "movie": os.path.join(wk, "nope.mp4"), "blocking": "zero"}))
check("bad movie/blocking non-fatal",
      out is not None and out["movie"] is None and out["blocking"] is None
      and len(errs) == 2, str(errs))

# ------------------------------------------------------- artifacts + stable

art_wk = os.path.join(TMP, "artifacts")
os.makedirs(art_wk)


def put(name, content="x", mtime=None):
    p = os.path.join(art_wk, name)
    open(p, "w").write(content)
    if mtime is not None:
        os.utime(p, (mtime, mtime))
    return p


put("input.kicad_pcb", "in", mtime=1000)
put("loop_round0.kicad_pcb", "r0", mtime=2000)
p_r1 = put("loop_round1.kicad_pcb", "r1-partial", mtime=3000)
put("notes.txt", "not a board")
os.makedirs(os.path.join(art_wk, "boards"))
open(os.path.join(art_wk, "boards", "deadbeef.kicad_pcb"), "w").write("cas")

arts = list_board_artifacts(art_wk)
check("artifact scan finds top-level boards only",
      [os.path.basename(p) for p, _m, _s in arts]
      == ["input.kicad_pcb", "loop_round0.kicad_pcb", "loop_round1.kicad_pcb"],
      str(arts))

check("first scan has no stable board (nothing to compare)",
      newest_stable_board(arts, None) is None)
arts2 = list_board_artifacts(art_wk)
check("size-stable newest wins",
      newest_stable_board(arts2, arts) == p_r1)
put("loop_round1.kicad_pcb", "r1-grown-since-last-scan", mtime=3001)
arts3 = list_board_artifacts(art_wk)
check("grown file skipped, older stable board returned",
      os.path.basename(newest_stable_board(arts3, arts2) or "")
      == "loop_round0.kicad_pcb")

# ------------------------------------------------------------- ledger tail

ledger = os.path.join(TMP, "ledger.jsonl")
with open(ledger, "w") as f:
    f.write(json.dumps({"iteration": 1, "kind": "quench", "lever": "nudge"}) + "\n")
    f.write(json.dumps({"iteration": 2, "kind": "reseat", "lever": "COL4"}) + "\n")
    f.write('{"iteration": 3, "kind": "torn"')  # no newline: torn last line
rows, off = read_ledger_tail(ledger, 0)
check("ledger reads complete rows", [r["iteration"] for r in rows] == [1, 2])
rows2, off2 = read_ledger_tail(ledger, off)
check("torn line not consumed", rows2 == [] and off2 == off)
with open(ledger, "a") as f:
    f.write(', "lever": "x"}\n')
rows3, off3 = read_ledger_tail(ledger, off2)
check("completed line picked up next tick",
      len(rows3) == 1 and rows3[0]["iteration"] == 3 and off3 > off2)
rows4, off4 = read_ledger_tail(os.path.join(TMP, "no-ledger.jsonl"), 0)
check("missing ledger tolerated", rows4 == [] and off4 == 0)

# ------------------------------------------------------------ derive_stage

tail = ["  -> Bash: python3 .../placement_driver.py --stage P4 --board x",
        "     [ok] stage emitted"]
row = {"iteration": 7, "kind": "reseat", "lever": "COL4"}
check("transcript stage wins over ledger",
      derive_stage(tail, row, "loop_round3.kicad_pcb")
      == placement_run.STAGE_LABELS["P4"])
tail_l2 = ["Bash: loop_driver.py --stage L2 --no-delegate"]
check("loop stages mapped", derive_stage(tail_l2, None, None)
      == placement_run.STAGE_LABELS["L2"])
check("--stage= form matched",
      derive_stage(["placement_driver.py --stage=P2"], None, None)
      == placement_run.STAGE_LABELS["P2"])
check('--stage "quoted" form matched',
      derive_stage(['placement_driver.py --stage "L3" --board x'], None, None)
      == placement_run.STAGE_LABELS["L3"])
check("P-close mapped",
      derive_stage(["--stage P-close"], None, None)
      == placement_run.STAGE_LABELS["P-close"])
check("ledger row formatted",
      derive_stage([], row, None) == "lap 7: reseat/COL4")
check("artifact heuristic route log",
      derive_stage([], None, "loop_round2_route.log") == "routing round 2")
check("artifact heuristic placing",
      derive_stage([], None, "loop_round5.kicad_pcb") == "placing round 5")
check("artifact heuristic lap board",
      derive_stage([], None, "lap3.kicad_pcb") == "lap 3")
check("fallback", derive_stage([], None, None) == "working...")

# ---------------------------------------------------------------- scavenge

scav = scan_workdir_outputs(wk)
check("scavenge prefers final.kicad_pcb",
      scav["board"] and os.path.basename(scav["board"]) == "final.kicad_pcb")
check("scavenge finds report", scav["report"] == report_md)
check("scavenge status", scav["status"] == "scavenged")
os.remove(final_board)
put_gif = os.path.join(wk, "placement.gif")
open(put_gif, "wb").write(b"GIF89a")
lap = os.path.join(wk, "lap1.kicad_pcb")
open(lap, "w").write("(kicad_pcb lap1)\n")
scav2 = scan_workdir_outputs(wk)
check("scavenge falls back to newest non-input board",
      scav2["board"] == lap, str(scav2))
check("scavenge gif fallback", scav2["movie"] == put_gif)
empty_wk = os.path.join(TMP, "empty")
os.makedirs(empty_wk)
scav3 = scan_workdir_outputs(empty_wk)
check("scavenge on empty dir is all-None",
      scav3["board"] is None and scav3["movie"] is None and scav3["report"] is None)

# ----------------------------------------------- ai_backend build_cmd kwargs

claude = ai_backend.BACKENDS["claude"]
cmd = claude.build_cmd("claude", "P", allowed_tools=PLACEMENT_ALLOWED_TOOLS,
                       add_dirs=(wk,))
check("placement allowlist in argv",
      cmd[cmd.index("--allowedTools") + 1] == PLACEMENT_ALLOWED_TOOLS, str(cmd))
check("add-dir in argv", "--add-dir" in cmd and cmd[cmd.index("--add-dir") + 1] == wk,
      str(cmd))
cmd_default = claude.build_cmd("claude", "P")
check("default argv unchanged (read-only allowlist, no add-dir)",
      cmd_default[cmd_default.index("--allowedTools") + 1]
      == ai_backend.CLAUDE_ALLOWED_TOOLS
      and "--add-dir" not in cmd_default, str(cmd_default))
oc = ai_backend.BACKENDS["opencode"]
# #552 item 4: opencode has no per-run allowlist flag, so a request it cannot
# grant is REFUSED rather than dropped. It used to bind both kwargs and read
# neither, which meant a write-capable caller got a read-only run and found out
# deep inside the skill. Read-only asks still build, and add_dirs is still
# ignored on purpose -- the agent is granted `external_directory`, so having no
# --add-dir costs the run nothing.
cmd_oc = oc.build_cmd("opencode", "P",
                      allowed_tools=ai_backend.CLAUDE_ALLOWED_TOOLS,
                      add_dirs=("Y",))
check("opencode still builds for a read-only allowlist",
      "--add-dir" not in cmd_oc and "Y" not in cmd_oc, str(cmd_oc))
try:
    oc.build_cmd("opencode", "P", allowed_tools=PLACEMENT_ALLOWED_TOOLS)
    _refused = ""
except ValueError as _e:
    _refused = str(_e)
check("opencode REFUSES a write-capable allowlist",
      "Write" in _refused and ai_backend.OPENCODE_ANALYSIS_AGENT in _refused,
      _refused or "no ValueError raised")

check("placement allowlist can write",
      all(t in PLACEMENT_ALLOWED_TOOLS.split(",") for t in ("Write", "Edit", "Bash")))

# #552: Task is what makes a skill's close-out verification a SECOND agent
# rather than the same model re-reading its own work. placement_run.py's own
# comment has claimed it is load-bearing since #633 with nothing asserting it.
check("placement allowlist carries Task",
      "Task" in PLACEMENT_ALLOWED_TOOLS.split(","), PLACEMENT_ALLOWED_TOOLS)
_analysis = ai_backend.CLAUDE_ALLOWED_TOOLS.split(",")
check("analysis allowlist carries Task", "Task" in _analysis,
      ai_backend.CLAUDE_ALLOWED_TOOLS)
check("analysis allowlist stays write-free (the property Task must not cost)",
      not ({"Write", "Edit", "NotebookEdit"} & set(_analysis)),
      ai_backend.CLAUDE_ALLOWED_TOOLS)

win_candidates = [c for c in claude.candidates
                  if c.endswith((".exe", ".cmd"))]
check("Windows CLI candidates present", len(win_candidates) >= 2,
      str(claude.candidates))

# -------------------------------------------------------------------- done

shutil.rmtree(TMP, ignore_errors=True)
shutil.rmtree(wk_tmp, ignore_errors=True)
print()
if FAILURES:
    print(f"{len(FAILURES)} FAILURES: {FAILURES}")
    sys.exit(1)
print("all ok")
