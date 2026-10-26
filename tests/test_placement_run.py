"""Placement tab headless contracts (placement_run.py + ai_backend kwargs).

Headless tests (no wx, no CLI, no pcbnew): workdir creation/staging, the
instruction + RESULT= contracts, the monitor's pollers (ledger tail, board
artifacts, the candidates table's scan, stage derivation), the scavenge
fallback, and the build_cmd allowed_tools/add_dirs extension the placement
runs depend on.
"""
import json
import os
import shutil
import stat
import subprocess
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
    PLACEMENT_ALLOWED_TOOLS, PLACEMENT_RESULT_SCHEMA, PLACEMENT_SKILL_MODES,
    PLACEMENT_SKILLS, RUN_MARKER, RUNS_DIRNAME, active_candidates,
    build_placement_instructions, build_placement_prompt, create_workdir,
    derive_stage, format_prune_note, list_board_artifacts, mark_kept,
    newest_run_board, newest_stable_board, parse_placement_result,
    prune_runs, read_ledger_tail, scan_candidates, scan_workdir_outputs,
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
check("no mechanical.json staged when the board dir has none",
      not os.path.exists(os.path.join(wk, "mechanical.json")))

# mechanical.json is a DIRECTORY file beside the board, not a stem sibling,
# and discover_mechanical reads it from the run board's own directory -- so it
# must be staged into the workdir under the same name or the run never sees it.
open(os.path.join(board_dir, "mechanical.json"), "w").write('{"anchors": []}\n')
wk_mech = create_workdir(board, "place")
staged_mech = stage_inputs(wk_mech, snapshot, board)
check("staged mechanical.json beside input.kicad_pcb",
      os.path.isfile(os.path.join(wk_mech, "mechanical.json"))
      and '"anchors"' in open(os.path.join(wk_mech, "mechanical.json")).read())
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_placer'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))
from placement.reconcile import discover_mechanical  # noqa: E402
check("discover_mechanical finds it from the STAGED board",
      os.path.normcase(discover_mechanical(staged_mech))
      == os.path.normcase(os.path.join(wk_mech, "mechanical.json")),
      discover_mechanical(staged_mech))

# EVERY input a CLI tool discovers beside a board reaches the staged board:
# the stem siblings copy_board.SIBLING_EXTS lists (.kicad_pro, .kicad_prl,
# .kicad_dru, .design-brief.json -- each read as `<stem><ext>`) and the one
# DIRECTORY file (mechanical.json). On BOTH of stage_inputs' import paths:
# the GUI's (placement_gui puts py_router on sys.path, so the real tuple) and
# the guarded fallback literal (test_711 waives it as a copy) -- the stagings
# above ran before py_router was on sys.path, so they saw only the fallback.
# Equal staged sets ARE the fallback-equals-canonical assertion.
from copy_board import SIBLING_EXTS  # noqa: E402
from placement.design_brief import discover_brief  # noqa: E402
full_dir = os.path.join(TMP, "boards_full")
os.makedirs(full_dir)
full_board = os.path.join(full_dir, "full.kicad_pcb")
open(full_board, "w").write("(kicad_pcb (version 20260206))\n")
for _ext in SIBLING_EXTS:
    open(os.path.join(full_dir, "full" + _ext), "w").write("{}\n")
open(os.path.join(full_dir, "mechanical.json"), "w").write('{"anchors": []}\n')


def _staged_set(import_ok):
    saved = sys.modules.get("copy_board", None)
    if not import_ok:
        sys.modules["copy_board"] = None       # `from copy_board import` raises
    try:
        wk_full = create_workdir(full_board, "place")
        st = stage_inputs(wk_full, snapshot, full_board)
    finally:
        if saved is None:
            sys.modules.pop("copy_board", None)
        else:
            sys.modules["copy_board"] = saved
    stem = os.path.splitext(st)[0]
    return st, {e for e in SIBLING_EXTS if os.path.isfile(stem + e)}


for _label, _ok in (("the GUI's import path (copy_board.SIBLING_EXTS)", True),
                    ("the guarded fallback tuple", False)):
    st, got = _staged_set(_ok)
    check(f"{_label}: every SIBLING_EXTS sibling staged",
          got == set(SIBLING_EXTS), f"missing {sorted(set(SIBLING_EXTS) - got)}")
    check(f"{_label}: discover_brief finds the staged brief",
          os.path.normcase(discover_brief(st)) == os.path.normcase(
              os.path.splitext(st)[0] + ".design-brief.json"), discover_brief(st))
    check(f"{_label}: discover_mechanical finds the staged mechanical.json",
          bool(discover_mechanical(st)), st)

# The project's schematic tree reaches the staged board too: board_context
# reads `<board>.kicad_sch` and its sub-sheets, by paths relative to the root,
# so the root is staged as input.kicad_sch and a sub-sheet keeps its folder.
from placement.schematic import discover as discover_schematic  # noqa: E402
from placement.schematic import tree_files  # noqa: E402
sch_dir = os.path.join(TMP, "boards_sch")
os.makedirs(os.path.join(sch_dir, "sheets"))
sch_board = os.path.join(sch_dir, "proj.kicad_pcb")
open(sch_board, "w").write("(kicad_pcb (version 20260206))\n")
open(os.path.join(sch_dir, "proj.kicad_sch"), "w").write(
    '(kicad_sch (version 20231120) (uuid "r")\n'
    '  (sheet (at 0 0) (size 10 10) (uuid "s1")\n'
    '    (property "Sheetname" "Power" (at 0 0 0))\n'
    '    (property "Sheetfile" "sheets/power.kicad_sch" (at 0 0 0))))\n')
open(os.path.join(sch_dir, "sheets", "power.kicad_sch"), "w").write(
    '(kicad_sch (version 20231120) (uuid "p"))\n')
wk_sch = create_workdir(sch_board, "place")
st_sch = stage_inputs(wk_sch, snapshot, sch_board)
root_sch, _looked = discover_schematic(st_sch)
check("the staged board finds its schematic as input.kicad_sch",
      root_sch is not None and os.path.normcase(root_sch)
      == os.path.normcase(os.path.join(wk_sch, "input.kicad_sch")), root_sch)
inside, outside = tree_files(root_sch) if root_sch else ([], ["unreadable"])
check("the staged hierarchy resolves whole (the sub-sheet in its folder)",
      sorted(r.replace("\\", "/") for _p, r in inside)
      == ["input.kicad_sch", "sheets/power.kicad_sch"] and not outside,
      (inside, outside))
check("a board with no schematic stages none",
      not os.path.exists(os.path.join(wk, "input.kicad_sch")))

# ----------------------------------------------- retention + .gitignore (#1057)

runs_root = os.path.join(board_dir, RUNS_DIRNAME)
gi = os.path.join(runs_root, ".gitignore")
check("krt_placement carries a .gitignore that ignores everything",
      os.path.isfile(gi) and "*" in open(gi).read().split())
def read_marker(run_dir):
    try:
        with open(os.path.join(run_dir, RUN_MARKER)) as f:
            return json.load(f)
    except (OSError, ValueError) as e:
        return {"unreadable": str(e)}


_marker = read_marker(wk)
check("run marker names the board and mode",
      os.path.normcase(_marker.get("board") or "") == os.path.normcase(board)
      and _marker.get("mode") == "place", str(_marker))
_marker_tmp = read_marker(wk_tmp)
check("unsaved board's marker names no board",
      "board" in _marker_tmp and _marker_tmp["board"] is None, str(_marker_tmp))

# A user's own .gitignore is never overwritten.
own = os.path.join(TMP, "own")
os.makedirs(os.path.join(own, RUNS_DIRNAME))
open(os.path.join(own, RUNS_DIRNAME, ".gitignore"), "w").write("mine\n")
open(os.path.join(own, "o.kicad_pcb"), "w").write("(kicad_pcb)\n")
create_workdir(os.path.join(own, "o.kicad_pcb"), "place")
check("existing .gitignore left alone",
      open(os.path.join(own, RUNS_DIRNAME, ".gitignore")).read() == "mine\n")

# The claim the .gitignore exists for, asked of git itself.
if shutil.which("git"):
    repo = os.path.join(TMP, "repo")
    os.makedirs(repo)
    subprocess.run(["git", "init", "-q", repo], check=True)
    rb = os.path.join(repo, "r.kicad_pcb")
    open(rb, "w").write("(kicad_pcb)\n")
    rwk = create_workdir(rb, "place")
    open(os.path.join(rwk, "final.kicad_pcb"), "w").write("(kicad_pcb)\n")
    st = subprocess.run(["git", "status", "--porcelain",
                         "--untracked-files=all"], cwd=repo,
                        capture_output=True, text=True, check=True).stdout
    check("git status sees the board but nothing under krt_placement",
          "r.kicad_pcb" in st and RUNS_DIRNAME not in st, st)
else:
    print("[skip] git not on PATH: .gitignore not checked against git")


def mk_run(root, name, board_path=None, marker=True, payload=2000):
    """A fabricated run directory, so the test controls its stamp."""
    d = os.path.join(root, name)
    os.makedirs(os.path.join(d, "boards", "cafe"))
    open(os.path.join(d, "boards", "cafe", "b.kicad_pcb"), "w").write(
        "x" * payload)
    if marker:
        with open(os.path.join(d, RUN_MARKER), "w") as f:
            json.dump({"board": board_path, "mode": "place"}, f)
    return d


# Two boards share one folder: A's runs, B's runs, one run from before the
# marker existed (whose board cannot be told), and things that are not runs.
two = os.path.join(TMP, "two")
os.makedirs(two)
ba = os.path.join(two, "a.kicad_pcb")
bb = os.path.join(two, "b.kicad_pcb")
for _b in (ba, bb):
    open(_b, "w").write("(kicad_pcb)\n")
root2 = os.path.join(two, RUNS_DIRNAME)
a_runs = [mk_run(root2, f"20260101_00000{i}_place", ba) for i in (1, 2, 3)]
a_runs.append(mk_run(root2, "20260101_000003_place_2", ba))   # same second
a_runs.append(mk_run(root2, "20260101_000004_place_route", ba))
b_runs = [mk_run(root2, f"20260101_00000{i}_place", bb) for i in (5, 6)]
legacy2 = mk_run(root2, "20250101_000000_place", marker=False)
os.makedirs(os.path.join(root2, "notes"))
open(os.path.join(root2, "20250101_000001_place"), "w").write("a file")
# A read-only file inside a run that must go (Windows rmtree trips on it).
_ro = os.path.join(a_runs[0], "boards", "cafe", "b.kicad_pcb")
os.chmod(_ro, stat.S_IREAD)
new_a = create_workdir(ba, "place")

res0 = prune_runs(new_a, ba, 0)
check("keep 0 keeps every run",
      not res0["pruned"] and all(os.path.isdir(d) for d in a_runs))

res = prune_runs(new_a, ba, 3)
check("keep 3 = the new run + this board's two newest",
      [os.path.basename(p) for p, _b in res["kept"]]
      == [os.path.basename(new_a), "20260101_000004_place_route",
          "20260101_000003_place_2"], str(res["kept"]))
check("this board's older runs removed (read-only file included)",
      sorted(os.path.basename(p) for p, _b in res["pruned"])
      == ["20260101_000001_place", "20260101_000002_place",
          "20260101_000003_place"]
      and not any(os.path.exists(p) for p, _b in res["pruned"])
      and not res["failed"], str(res))
check("pruned sizes measured", all(b >= 2000 for _p, b in res["pruned"]),
      str(res["pruned"]))
check("the other board's runs untouched", all(os.path.isdir(d) for d in b_runs))
check("an unattributable run in a two-board folder untouched + counted",
      os.path.isdir(legacy2) and res["unattributed"] == 1, str(res))
check("non-run entries untouched",
      os.path.isdir(os.path.join(root2, "notes"))
      and os.path.isfile(os.path.join(root2, "20250101_000001_place")))
check("the new run is never pruned", os.path.isdir(new_a))
note = format_prune_note(res)
check("prune note says what was removed and what is kept",
      "removed 3 older run(s)" in note and "keeping the newest 3" in note
      and "never pruned" in note, note)

new_b = create_workdir(bb, "place")
res1 = prune_runs(new_b, bb, 1)
check("keep 1 removes every other run of the board, and only that board's",
      not any(os.path.isdir(d) for d in b_runs)
      and os.path.isdir(new_b) and os.path.isdir(new_a), str(res1))
_quiet = format_prune_note(prune_runs(new_a, ba, 99))
check("nothing pruned -> the note is only the unattributed-run line",
      _quiet.count("\n") == 1 and "never pruned" in _quiet, _quiet)

if os.name == "nt":
    # A file held open (a report in an editor) blocks deletion on Windows, and
    # rmtree removes .krt_run.json first there. The half-deleted run must stay
    # this board's so the next run retries it, as the note promises -- in a
    # two-board folder a marker-less run would be nobody's.
    busy = mk_run(root2, "20260101_000000_place", ba)
    held = open(os.path.join(busy, "REPORT.md"), "w")
    try:
        res_busy = prune_runs(new_a, ba, 1)
        check("an open file fails the prune, disclosed",
              os.path.isdir(busy) and busy in res_busy["failed"]
              and "could not fully remove" in format_prune_note(res_busy),
              str(res_busy))
        check("...and the half-deleted run is still this board's",
              read_marker(busy).get("board") is not None
              and res_busy["unattributed"] == 1, str(read_marker(busy)))
    finally:
        held.close()
    res_retry = prune_runs(new_a, ba, 1)
    check("the next run's prune finishes it",
          not os.path.exists(busy) and not res_retry["failed"], str(res_retry))

# One board in the folder: a marker-less run can only have been its run.
one = os.path.join(TMP, "one")
os.makedirs(one)
bo = os.path.join(one, "o.kicad_pcb")
open(bo, "w").write("(kicad_pcb)\n")
open(os.path.join(one, "_autosave-o.kicad_pcb"), "w").write("(kicad_pcb)\n")
root1 = os.path.join(one, RUNS_DIRNAME)
legacy1 = [mk_run(root1, f"20250101_00000{i}_place", marker=False)
           for i in (1, 2)]
new_o = create_workdir(bo, "place_route")
res_o = prune_runs(new_o, bo, 2)
check("marker-less runs of a one-board folder are that board's",
      not os.path.exists(legacy1[0]) and os.path.isdir(legacy1[1])
      and res_o["unattributed"] == 0, str(res_o))

if os.name == "nt":
    other_case = mk_run(root1, "20250101_000003_place", bo.upper())
    prune_runs(new_o, bo, 1)
    check("board paths compare case-insensitively on Windows",
          not os.path.exists(other_case))

# A link named like a run is never followed or removed.
lnk_target = os.path.join(TMP, "lnk_target")
os.makedirs(lnk_target)
open(os.path.join(lnk_target, "precious.txt"), "w").write("keep")
lnk = os.path.join(root1, "20250101_000009_place")
try:
    os.symlink(lnk_target, lnk, target_is_directory=True)
except (OSError, NotImplementedError):
    print("[skip] cannot create a directory symlink here")
else:
    res_l = prune_runs(new_o, bo, 1)
    # Not even attempted: rmtree refusing a link is not the guard.
    check("a link named like a run is left alone",
          os.path.islink(lnk) and not res_l["failed"]
          and os.path.isfile(os.path.join(lnk_target, "precious.txt")),
          str(res_l))
    if os.name == "nt":
        os.rmdir(lnk)       # a directory symlink is removed as a directory
    else:
        os.remove(lnk)

# ------------------------------------------------------------- instructions

instr = build_placement_instructions(wk, "place")
check("instructions name the workdir", wk.replace("\\", "/") in instr)
check("instructions carry the RESULT schema", PLACEMENT_RESULT_SCHEMA in instr)
check("place mode has no --no-delegate", "--no-delegate" not in instr)
check("instructions refuse copper stripping", '"refused"' in instr)
instr_pr = build_placement_instructions(wk, "place_route", extra="Focus on U1.")
# The placement skill has no staged driver to run inline, so neither mode
# passes the old loop driver's --no-delegate any more.
check("place_route mode has no --no-delegate either",
      "--no-delegate" not in instr_pr)
check("extra instructions included", "Focus on U1." in instr_pr)

# Both tab modes run the one placement skill; the skill's own mode argument
# is what tells them apart.
check("both tab modes run plan-pcb-placement",
      PLACEMENT_SKILLS == {"place": "plan-pcb-placement",
                           "place_route": "plan-pcb-placement"},
      str(PLACEMENT_SKILLS))
check("tab modes map to the skill's place / full modes",
      PLACEMENT_SKILL_MODES == {"place": "place", "place_route": "full"},
      str(PLACEMENT_SKILL_MODES))
_board_fwd = os.path.abspath(staged).replace("\\", "/")
prompt = build_placement_prompt(ai_backend.BACKENDS["claude"], wk, staged, "place")
check("place prompt is /plan-pcb-placement place <board>",
      prompt.startswith(f"/plan-pcb-placement place {_board_fwd}"), prompt[:120])
check("prompt names the staged board", staged.replace("\\", "/") in prompt)
prompt_pr = build_placement_prompt(ai_backend.BACKENDS["claude"], wk, staged,
                                   "place_route")
check("place_route prompt is /plan-pcb-placement full <board>",
      prompt_pr.startswith(f"/plan-pcb-placement full {_board_fwd}"),
      prompt_pr[:120])

# ------------------------------------------------------------------- RESULT

final_board = os.path.join(wk, "final.kicad_pcb")
open(final_board, "w").write("(kicad_pcb final)\n")
report_md = os.path.join(wk, "REPORT.md")
open(report_md, "w").write("# report\n")

ok_payload = json.dumps({
    "status": "complete", "board": final_board.replace("\\", "/"),
    "report": report_md, "blocking": 0, "summary": "done"})
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
     "report": os.path.join(wk, "nope.md"), "blocking": "zero"}))
check("bad report/blocking non-fatal",
      out is not None and out["report"] is None and out["blocking"] is None
      and len(errs) == 2, str(errs))
# #1075: `isinstance(True, int)` holds, so a bool (and a negative) passed.
# The REFUSAL is asserted, not its wording: 3.0 and "0" were refused before
# #1075 too, and must keep failing only if they stop being refused.
for _bad in (True, False, -1, 3.0, "0"):
    out, errs = parse_placement_result(json.dumps(
        {"status": "complete", "board": final_board, "blocking": _bad}))
    check("blocking %r is refused" % (_bad,),
          out is not None and out["blocking"] is None
          and any(e.startswith("blocking ") for e in errs), str(errs))
for _ok in (0, 3):
    out, errs = parse_placement_result(json.dumps(
        {"status": "complete", "board": final_board, "blocking": _ok}))
    check("blocking %r is kept" % (_ok,),
          out is not None and out["blocking"] == _ok and not errs, str(errs))

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

# The newest recognised tool command says what the run is doing now, the
# newest ledger row its last milestone (both shown when both exist); without
# either, the newest board's name (relative to the workdir) answers.
tail = ["  -> Bash: python3 .../placement_driver.py --stage P4 --board x",
        "     [ok] stage emitted"]
row = {"iteration": 7, "kind": "reseat", "lever": "COL4"}
check("ledger row wins over an unrecognised command and an artifact",
      derive_stage(tail, row, "s3.kicad_pcb") == "lap 7: reseat/COL4")
check("a stage-like transcript alone is not a stage",
      derive_stage(["Bash: loop_driver.py --stage L2"], None, None)
      == "working...")
check("artifact wins over an unrecognised command when no row exists",
      derive_stage(tail, None, "s3.kicad_pcb") == "seed 3 placed")
check("the old STAGE_LABELS table is gone",
      not hasattr(placement_run, "STAGE_LABELS"))
check("placing a seed",
      derive_stage(["python3 py_placer/place_simple.py wk/fixed.kicad_pcb "
                    "wk/s3.kicad_pcb --seed 3 --json wk/s3.json"],
                   None, None) == "placing seed 3")
check("fast-routing a seed, with the last milestone",
      derive_stage(["cd wk/s1_route && python3 py_router/route.py a.kicad_pcb "
                    "b.kicad_pcb --nets '*' --quick"], row, None)
      == "fast-routing seed 1 (lap 7: reseat/COL4)")
check("scoring a seed",
      derive_stage(["python3 py_tools/quick_score.py wk/s2_fast.kicad_pcb "
                    "--json wk/s2_fast.score.json"], None, None)
      == "scoring seed 2")
check("the parallel form names its seeds",
      derive_stage(["python3 py_placer/place_simple.py wk/fixed.kicad_pcb "
                    "wk/s{seed}.kicad_pcb --seeds 0-3 --jobs 4 "
                    "--json wk/s{seed}.json"], None, None)
      == "placing seeds 0-3")
check("an s{seed} board with no seed named places the default seeds",
      derive_stage(["python3 py_placer/place_simple.py wk/fixed.kicad_pcb "
                    "'wk/s{seed}.kicad_pcb' --jobs 4 "
                    "--json 'wk/s{seed}.json'"], None, None)
      == "placing the seeds")
check("a --seed run with no seed board still names the seed",
      derive_stage(["python3 py_placer/place_simple.py a b --seed 2"], None,
                   None) == "placing seed 2")

check("ledger row formatted",
      derive_stage([], row, None) == "lap 7: reseat/COL4")
check("artifact heuristic fast-routed seed",
      derive_stage([], None, "s0_fast.kicad_pcb") == "seed 0 fast-routed")
check("artifact heuristic fixed board",
      derive_stage([], None, "fixed.kicad_pcb") == "fixed parts locked")
check("artifact heuristic final board",
      derive_stage([], None, "final.kicad_pcb") == "final board written")
check("fallback", derive_stage([], None, None) == "working...")

# -------------------------------------------------------------- candidates

cand_wk = os.path.join(TMP, "candidates")


def cput(rel, content="(kicad_pcb)\n", mtime=None):
    p = os.path.join(cand_wk, rel)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    open(p, "w").write(content)
    if mtime is not None:
        os.utime(p, (mtime, mtime))
    return p


def cscore(rel, blocking, vias, copper, segments):
    cput(rel, json.dumps({"blocking": blocking, "quality": {
        "vias": vias, "copper_mm": copper, "segments": segments}}))


# s0 scored with blocking nets, s1 scored clean, s2 placed and routing, s3's
# board written but its report not yet, s4 scored clean with fewer vias
# (best), newest.
cput("fixed.kicad_pcb", "(kicad_pcb fixed)\n")
for s in (0, 1, 2, 4):
    cput(f"s{s}.kicad_pcb", f"(kicad_pcb s{s})\n")
    cput(f"s{s}.json", "{}")
cput("s3.kicad_pcb", "(kicad_pcb s3)\n")
for s in (0, 1):
    cput(f"s{s}_fast.kicad_pcb", f"(kicad_pcb s{s} fast)\n")
cscore("s0_fast.score.json", 2, 40, 300.0, 200)
cscore("s1_fast.score.json", 0, 44, 310.0, 210)
cput("s0_route/step4_route.kicad_pcb")          # working files: no row
cput("s5_route/step4_route.kicad_pcb")          # a route folder alone: no row
_later = os.path.getmtime(os.path.join(cand_wk, "s4.json")) + 60
cput("s4_fast.kicad_pcb", "(kicad_pcb s4 fast)\n", mtime=_later)
cscore("s4_fast.score.json", 0, 30, 280.0, 190)

rows_c = scan_candidates(cand_wk)
by = {r["seed"]: r for r in rows_c}
check("one row per seed, in order",
      [r["seed"] for r in rows_c] == [0, 1, 2, 3, 4], str(list(by)))
check("scored row with blocking nets", by[0]["status"] == "routed"
      and by[0]["vias"] == 40 and by[0]["copper_mm"] == 300.0)
check("a clean seed that is not the best", by[1]["status"] == "clean")
check("the best is the clean seed with fewest vias", by[4]["status"] == "best")
check("placed, not yet scored", by[2]["status"] == "routing"
      and by[2]["fast"] is None and by[2]["blocking"] is None)
check("board without its report is still placing",
      by[3]["status"] == "placing")
check("a seed's route dir adds no row", 5 not in by)
rows_a = scan_candidates(cand_wk, active=[6])
check("the seed being placed shows before its board exists",
      any(r["seed"] == 6 and r["status"] == "placing" for r in rows_a))
check("active_candidates reads the newest place_simple command",
      active_candidates(["python3 py_placer/place_simple.py f wk/s0.kicad_pcb",
                         "python3 py_placer/place_simple.py f wk/s1.kicad_pcb"])
      == [1])
check("the parallel form names every seed it places",
      active_candidates(["python3 py_placer/place_simple.py f "
                         "wk/s{seed}.kicad_pcb --seeds 0-3 --jobs 4 "
                         "--json wk/s{seed}.json"])
      == [0, 1, 2, 3])
check("the default seeds: its scattered and clustered ones",
      active_candidates(["python3 py_placer/place_simple.py f "
                         "'wk/s{seed}.kicad_pcb' --scattered 2 --clustered 1"])
      == [0, 1, 2])
check("one --seed into an s{seed} board is that seed",
      active_candidates(["python3 py_placer/place_simple.py f "
                         "'wk/s{seed}.kicad_pcb' --seed 7"]) == [7])
check("an s{seed} board with --seed 3 is placing seed 3, not the default seeds",
      derive_stage(["python3 py_placer/place_simple.py f 'wk/s{seed}.kicad_pcb' --seed 3"],
                   None, None) == "placing seed 3")
check("no place_simple command, no active candidate",
      active_candidates(["python3 py_tools/board_score.py x"]) == [])
check("newest board, relative to the workdir",
      newest_run_board(cand_wk) == "s4_fast.kicad_pcb",
      str(newest_run_board(cand_wk)))

# KEPT: the final board is matched to a candidate by content; a final board
# that is no candidate's falls back to the best scored row of the run.
shutil.copyfile(os.path.join(cand_wk, "s1.kicad_pcb"),
                os.path.join(cand_wk, "final.kicad_pcb"))
kept = mark_kept(scan_candidates(cand_wk), cand_wk)
check("final.kicad_pcb matched to its candidate by sha256",
      kept is not None and kept["seed"] == 1 and kept["status"] == "KEPT")
cput("final.kicad_pcb", "(kicad_pcb touched up)\n")
kept2 = mark_kept(scan_candidates(cand_wk), cand_wk)
check("an unmatched final falls back to the run's best scored row",
      kept2["seed"] == 4, str(kept2))
os.remove(os.path.join(cand_wk, "final.kicad_pcb"))
scav_c = scan_workdir_outputs(cand_wk)
check("scavenge without final takes the best candidate's PLACEMENT",
      scav_c["board"] == os.path.join(cand_wk, "s4.kicad_pcb"),
      str(scav_c["board"]))
for name in os.listdir(cand_wk):
    if name.endswith(".score.json"):
        os.remove(os.path.join(cand_wk, name))
scav_n = scan_workdir_outputs(cand_wk)
check("scavenge with no score never takes a --quick board",
      scav_n["board"] and not os.path.basename(scav_n["board"]).endswith("_fast.kicad_pcb"),
      str(scav_n["board"]))

# ---------------------------------------------------------------- scavenge

scav = scan_workdir_outputs(wk)
check("scavenge prefers final.kicad_pcb",
      scav["board"] and os.path.basename(scav["board"]) == "final.kicad_pcb")
check("scavenge finds report", scav["report"] == report_md)
check("scavenge status", scav["status"] == "scavenged")
os.remove(final_board)
lap = os.path.join(wk, "lap1.kicad_pcb")
open(lap, "w").write("(kicad_pcb lap1)\n")
scav2 = scan_workdir_outputs(wk)
check("scavenge falls back to newest non-input board",
      scav2["board"] == lap, str(scav2))
empty_wk = os.path.join(TMP, "empty")
os.makedirs(empty_wk)
scav3 = scan_workdir_outputs(empty_wk)
check("scavenge on empty dir is all-None",
      scav3["board"] is None and scav3["report"] is None)

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
