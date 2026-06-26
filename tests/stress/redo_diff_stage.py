#!/usr/bin/env python3
"""Re-run the fanout + route_diff stages of recorded stress boards, then flag any
board with deferred/failed differential pairs or DRC errors (issue: diff-stage
regression sweep).

For every recorded stress run that contains a route_diff.py step, this:
  1. truncates that board's redo_commands.sh manifest through the LAST non-help
     route_diff.py command (so only fanout + diff-pair routing replay),
  2. replays the truncated chain into a fresh work dir via redo_stress_test.py
     --remap (the engine is whatever is checked out in the repo -- so this A/Bs
     a router change against the recorded diff stage),
  3. parses route_diff's JSON_SUMMARY for failed/deferred (single-ended) pairs,
  4. runs check_drc.py on the diff-stage board at that step's own --clearance
     (per the "grade DRC at routed clearance" rule), and
  5. copies the kicad_* files of any FLAGGED board (deferred OR failed pairs OR
     DRC violations OR no output produced) to the output dir for inspection.

Boards are auto-detected by scanning <stress-root>/runs*/*/redo_commands.sh, so
no board list is needed; pass board names (or set/board) as positional filters to
restrict the run.

Usage:
  redo_diff_stage.py                       # all diff boards under the default root
  redo_diff_stage.py hackrf_one glasgow_revC
  redo_diff_stage.py --stress-root ~/Documents/kicad_stress_test \
                     --out-dir ~/Documents/diff2
"""

import argparse
import glob
import json
import os
import re
import shlex
import shutil
import subprocess
import sys

# repo root = two levels up from this file (tests/stress/redo_diff_stage.py)
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REDO = os.path.join(REPO, "tests", "stress", "redo_stress_test.py")
CHECK_DRC = os.path.join(REPO, "check_drc.py")
DEFAULT_ROOT = os.path.expanduser("~/Documents/kicad_stress_test")
DEFAULT_OUT = os.path.expanduser("~/Documents/diff2")


def find_diff_manifests(root):
    """Return [(set_board, manifest_path), ...] for every run dir whose manifest
    has at least one non-help route_diff.py command. set_board is 'runs_setN/board'."""
    out = []
    for man in sorted(glob.glob(os.path.join(root, "runs*", "*", "redo_commands.sh"))):
        txt = open(man, encoding="utf-8", errors="replace").read()
        if any("route_diff.py" in ln and "--help" not in ln and not ln.lstrip().startswith("#")
               for ln in txt.splitlines()):
            rundir = os.path.dirname(man)
            setname = os.path.basename(os.path.dirname(rundir))
            out.append((f"{setname}/{os.path.basename(rundir)}", man))
    return out


def last_route_diff_index(lines):
    last = None
    for i, ln in enumerate(lines):
        if "route_diff.py" in ln and "--help" not in ln and not ln.lstrip().startswith("#"):
            last = i
    return last


def resolve_diff_output(argv):
    """Replicate route_diff.py's output-file resolution: --output, else the second
    positional, else input_routed.kicad_pcb (or input itself with --overwrite).
    Also return the --clearance value. Returns (output_path_str, clearance_str)."""
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("input_file")
    p.add_argument("output_file", nargs="?")
    p.add_argument("net_patterns", nargs="*")
    p.add_argument("--output")
    p.add_argument("--nets", "-n", nargs="+")
    p.add_argument("--clearance", default="0.2")
    p.add_argument("--overwrite", action="store_true")
    ns, _ = p.parse_known_args(argv[1:])  # drop the python script path token
    out = ns.output or ns.output_file
    if out is None:
        if ns.overwrite:
            out = ns.input_file
        else:
            base, ext = os.path.splitext(ns.input_file)
            out = base + "_routed" + ext
    return out, str(ns.clearance)


def diffstage_min_clearance(kept_lines, route_diff_clearance):
    """The DRC floor a diff-stage board must meet is the MIN --clearance among the
    steps that actually laid copper on it (fanout + route_diff) -- NOT the
    route_diff step's own value. Grading at the route_diff clearance over-counts
    when that step used a looser (numerically larger) clearance than the fanout
    (e.g. eis/lily58 routed diff pairs at 0.2 over a 0.1/0.127 board). Falls back
    to the route_diff clearance if no step set one explicitly."""
    cls = []
    for ln in kept_lines:
        if ln.lstrip().startswith("#") or "--clearance" not in ln:
            continue
        try:
            a = shlex.split(ln)
        except ValueError:
            continue
        if "--clearance" in a:
            i = a.index("--clearance")
            try:
                cls.append(float(a[i + 1]))
            except (IndexError, ValueError):
                pass
    if route_diff_clearance is not None:
        try:
            cls.append(float(route_diff_clearance))
        except ValueError:
            pass
    return f"{min(cls):g}" if cls else str(route_diff_clearance)


def parse_json_summary(text):
    summary = None
    for m in re.finditer(r"JSON_SUMMARY:\s*(\{.*\})", text):
        try:
            summary = json.loads(m.group(1))
        except Exception:
            pass
    return summary


def parse_drc_count(text):
    m = re.search(r"FAILED\s+\((\d+)\s+violation", text)
    return int(m.group(1)) if m else 0  # PASSED / no FAILED line -> 0


def seed_missing_inputs(kept_lines, olddir, wdir):
    """Some manifests read a relative board (e.g. <board>_input.kicad_pcb or an
    intermediate) that an UNRECORDED copy/rename step produced -- the chain never
    regenerates it, so a fresh replay would FileNotFoundError. Copy any such
    relative input that no earlier command produces from the original run dir.
    Returns the list of seeded basenames."""
    produced = set()
    seeded = []
    for ln in kept_lines:
        if not ln.strip() or ln.lstrip().startswith("#"):
            continue
        try:
            argv = shlex.split(ln)
        except ValueError:
            continue
        pcbs = [a for a in argv if a.endswith(".kicad_pcb")]
        if not pcbs:
            continue
        # consumed input = first .kicad_pcb token; outputs = --output value + any
        # later positional .kicad_pcb token
        inp = pcbs[0]
        outs = set(pcbs[1:])
        if "--output" in argv:
            i = argv.index("--output")
            if i + 1 < len(argv):
                outs.add(argv[i + 1])
        base = os.path.basename(inp)
        if not os.path.isabs(inp) and base not in produced:
            src = os.path.join(olddir, base)
            dst = os.path.join(wdir, base)
            if os.path.isfile(src) and not os.path.isfile(dst):
                shutil.copy(src, dst)
                seeded.append(base)
        produced.update(os.path.basename(o) for o in outs)
    return seeded


def process_board(set_board, manifest, work_root, out_dir, drc_size_checks):
    board = set_board.split("/")[-1]
    wdir = os.path.join(work_root, board)
    if os.path.isdir(wdir):
        shutil.rmtree(wdir)
    os.makedirs(wdir)

    lines = open(manifest, encoding="utf-8", errors="replace").read().splitlines()
    last = last_route_diff_index(lines)
    if last is None:
        return None
    kept = lines[:last + 1]
    tman = os.path.join(wdir, "trunc_redo.sh")
    open(tman, "w", encoding="utf-8").write("\n".join(kept) + "\n")

    olddir = os.path.dirname(manifest)  # recorded cwd / absolute-path prefix
    seeded = seed_missing_inputs(kept, olddir, wdir)
    if seeded:
        print(f"  (seeded unrecorded inputs: {', '.join(seeded)})")

    rd_argv = shlex.split(lines[last])
    # strip leading interpreter tokens so resolve_diff_output sees the route_diff argv
    while rd_argv and (rd_argv[0].endswith(("python", "python3")) or rd_argv[0] in ("-X", "utf8")):
        rd_argv.pop(0)
    out_board, rd_clearance = resolve_diff_output(rd_argv)
    # grade DRC at the floor the board must meet (min over fanout + route_diff),
    # not the route_diff step's own clearance
    clearance = diffstage_min_clearance(kept, rd_clearance)

    log = os.path.join(wdir, "replay.log")
    print(f"\n===== {set_board} (DRC clearance {clearance}, out {os.path.basename(out_board)}) =====",
          flush=True)
    with open(log, "w") as lf:
        # --workdir forces every command into wdir (so relative inputs -- incl.
        # seeded ones -- chain there even when a manifest's recorded `# cwd=`
        # points elsewhere, as the _prev_orig dirs do); --remap additionally
        # rewrites absolute-path arguments (e.g. an absolute --output) into wdir.
        rc = subprocess.run(
            ["python3", "-X", "utf8", REDO, tman, "--workdir", wdir,
             "--remap", f"{olddir}:{wdir}", "--skip-checks", "--continue-on-error"],
            stdout=lf, stderr=subprocess.STDOUT).returncode

    text = open(log, encoding="utf-8", errors="replace").read()
    summary = parse_json_summary(text)
    failed = summary.get("failed_diff_pairs", []) if summary else None
    deferred = summary.get("single_ended_diff_pairs", []) if summary else None
    routed = summary.get("routed_diff_pairs", []) if summary else []

    out_path = os.path.join(wdir, os.path.basename(out_board))
    drc_log = os.path.join(wdir, "drc.log")
    if os.path.isfile(out_path):
        cmd = ["python3", "-X", "utf8", CHECK_DRC, out_path, "-c", clearance, "--quiet"]
        if not drc_size_checks:
            cmd.append("--no-size-checks")
        with open(drc_log, "w") as df:
            subprocess.run(cmd, stdout=df, stderr=subprocess.STDOUT)
        drc = parse_drc_count(open(drc_log, encoding="utf-8", errors="replace").read())
    else:
        drc = "NO_OUTPUT"

    flagged = (bool(failed) or bool(deferred) or summary is None
               or drc == "NO_OUTPUT" or (isinstance(drc, int) and drc > 0))
    rec = dict(board=set_board, routed=len(routed), failed=failed, deferred=deferred,
               drc=drc, flagged=flagged, replay_rc=rc, out=os.path.basename(out_board))
    print(f"  routed={len(routed)} failed={failed} deferred={deferred} "
          f"drc={drc} flagged={flagged}")

    if flagged and os.path.isfile(out_path):
        dest = os.path.join(out_dir, set_board)
        os.makedirs(dest, exist_ok=True)
        base = os.path.splitext(os.path.basename(out_board))[0]
        for ext in (".kicad_pcb", ".kicad_pro", ".kicad_prl"):
            p = os.path.join(wdir, base + ext)
            if os.path.isfile(p):
                shutil.copy(p, os.path.join(dest, board + "_diff" + ext))
        shutil.copy(log, os.path.join(dest, "replay.log"))
        if os.path.isfile(drc_log):
            shutil.copy(drc_log, os.path.join(dest, "drc.log"))
    return rec


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("boards", nargs="*",
                    help="Optional board-name or set/board filters (default: all diff boards)")
    ap.add_argument("--stress-root", default=DEFAULT_ROOT)
    ap.add_argument("--out-dir", default=DEFAULT_OUT,
                    help="Where flagged boards' kicad_* files are copied")
    ap.add_argument("--work-root", default=None,
                    help="Scratch dir for replays (default: <stress-root>/diff_redo_work)")
    ap.add_argument("--drc-size-checks", action="store_true",
                    help="Include fab min-width/size checks in DRC (default: clearance/geometry only)")
    ap.add_argument("--list", action="store_true", help="List detected diff boards and exit")
    args = ap.parse_args()

    work_root = args.work_root or os.path.join(args.stress_root, "diff_redo_work")
    os.makedirs(work_root, exist_ok=True)
    os.makedirs(args.out_dir, exist_ok=True)

    boards = find_diff_manifests(args.stress_root)
    if args.boards:
        want = set(args.boards)
        boards = [(sb, m) for (sb, m) in boards
                  if sb in want or sb.split("/")[-1] in want]
    if args.list:
        for sb, _ in boards:
            print(sb)
        print(f"\n{len(boards)} diff board(s)")
        return 0
    if not boards:
        print("No matching diff boards found.")
        return 1

    print(f"Replaying diff stage for {len(boards)} board(s) from {args.stress_root}")
    report = []
    for sb, man in boards:
        try:
            rec = process_board(sb, man, work_root, args.out_dir, args.drc_size_checks)
            if rec:
                report.append(rec)
        except Exception as e:
            print(f"  ERROR on {sb}: {e}")
            report.append(dict(board=sb, error=str(e), flagged=True))

    with open(os.path.join(work_root, "report.json"), "w") as f:
        json.dump(report, f, indent=2)

    flagged = [r for r in report if r.get("flagged")]
    print("\n\n========== SUMMARY ==========")
    print(f"{len(report)} boards, {len(flagged)} flagged (deferred/failed pairs or DRC). "
          f"Flagged boards copied under {args.out_dir}")
    for r in sorted(report, key=lambda x: not x.get("flagged")):
        print(json.dumps(r))
    return 0


if __name__ == "__main__":
    sys.exit(main())
