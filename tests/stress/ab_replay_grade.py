#!/usr/bin/env python3
"""Replay a whole stress-test set into a fresh wave dir and grade it -- the A/B
harness on top of redo_stress_test.py (single-manifest replay).

`redo_stress_test.py` replays ONE board's recorded command manifest. This driver
replays every board in a set in parallel, then grades each board's *final* board
for DRC (at the route step's actual --clearance) and connectivity, and writes a
JSON summary. Run it once per code version ("wave") to A/B an engine change.

Two modes:

  # Grade one wave (the working tree decides which code runs):
  ab_replay_grade.py --set ~/Documents/kicad_stress_test/runs_set3 \
                     --out ~/Documents/kicad_stress_test/ab_run/old --label old

  # Compare two wave summaries:
  ab_replay_grade.py --compare .../old/summary.json .../new/summary.json

Typical A/B recipe (the engine change is uncommitted in the working tree):

  git stash push file1.py file2.py            # baseline = HEAD
  ab_replay_grade.py --set runs_set3 --out ab/old --label old
  git stash pop                               # candidate = HEAD + change
  ab_replay_grade.py --set runs_set3 --out ab/new --label new
  ab_replay_grade.py --compare ab/old/summary.json ab/new/summary.json

Notes / gotchas (see memory: rerun-stress-boards, grade-drc-at-routed-clearance):
- Manifests reference tools by ABSOLUTE repo path, so a replay always runs
  whatever is checked out -- the two waves MUST be sequential (shared git state),
  but boards WITHIN a wave run in parallel.
- A board whose chain breaks (e.g. a diff pair fails -> route_diff writes no
  output) reports chain_complete=False and is excluded from the DRC/conn
  comparison. Compare only counts boards complete in BOTH waves.
- DRC is graded at each board's own routed --clearance, parsed from its manifest.
"""
import argparse
import concurrent.futures
import json
import os
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # for _gitver when run as a script
from _gitver import write_git_version, load_git_version, format_version

REPO = Path(__file__).resolve().parent.parent.parent  # tests/stress/ -> repo root


def route_clearance(manifest_txt, default="0.1"):
    """DRC must be graded at the routed clearance. A board may route different
    steps at different --clearance (e.g. a signal retry at 0.15 over a 0.1 base,
    plus planes at 0.1); grade at the MINIMUM so copper laid at the tightest
    clearance isn't phantom-flagged at a looser one (see grade-drc-at-routed-
    clearance: grading tigard at 0.15 vs its real 0.1 invents ~600 violations)."""
    vals = [float(v) for v in re.findall(r"--clearance\s+(\d[\d.]*)", manifest_txt)]
    return str(min(vals)) if vals else default


def final_output_name(manifest_txt):
    """Final board = last .kicad_pcb token of the last non-check command.

    Output naming varies per board (<board>_stepN, <board>_signal, step1_signal,
    ...), so detect it from the manifest rather than globbing a fixed pattern.
    For route*.py the output is the 2nd positional and for fanout it is --output;
    in both, the LAST .kicad_pcb token on the line is the produced board.
    """
    last = None
    for line in manifest_txt.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "check_" in line:
            continue
        toks = [t.strip("'\"") for t in line.split() if t.strip("'\"").endswith(".kicad_pcb")]
        if toks:
            last = toks[-1]
    return os.path.basename(last) if last else None


def _drc_count(text):
    m = re.search(r"FAILED \((\d+) violations?\)", text) or re.search(r"FOUND (\d+) DRC VIOLATION", text)
    return int(m.group(1)) if m else 0  # no match == PASSED / clean


def _conn_count(text):
    m = re.search(r"Connectivity issues \((\d+)\)", text)
    return int(m.group(1)) if m else 0


def _completion(text):
    """Routing completion from check_connected output: a net is complete if it is
    neither unrouted nor has a connectivity issue. Returns (total, incomplete, pct)
    or (None, None, None) if the net total can't be parsed."""
    tm = re.search(r"Checking (\d+) [\w ]*nets", text)
    if not tm:
        return None, None, None
    total = int(tm.group(1))
    um = re.search(r"Unrouted nets \((\d+)\)", text)
    im = re.search(r"Connectivity issues \((\d+)\)", text)
    incomplete = min(total, (int(um.group(1)) if um else 0) + (int(im.group(1)) if im else 0))
    pct = round(100.0 * (total - incomplete) / total, 1) if total else None
    return total, incomplete, pct


def _tool_of(argv):
    """Tool name (basename of the first .py arg) for per-tool timing aggregation."""
    for a in argv:
        if a.endswith(".py"):
            return os.path.basename(a)
    return argv[-1] if argv else "?"


def _diff_pair_stats(log_text):
    """Coupled diff-pair completion, from route_diff's JSON_SUMMARY lines in the
    replay log. route_diff classifies each pair as coupled (routed_diff_pairs),
    deferred-to-single-ended (single_ended_diff_pairs), or failed. Across multiple
    route_diff calls a pair's FINAL status wins (a retry can couple one that first
    failed). 'Completion' here = coupled / total pairs -- the rate of pairs actually
    routed as coupled diff pairs (single-ended fallback does NOT count). Returns
    None pct when the board has no diff pairs."""
    status = {}
    for m in re.finditer(r"JSON_SUMMARY:\s*(\{.*\})", log_text):
        try:
            d = json.loads(m.group(1))
        except Exception:
            continue
        if "routed_diff_pairs" not in d:   # skip non-diff summaries (e.g. bga_fanout)
            continue
        for p in d.get("routed_diff_pairs", []):       status[p] = "coupled"
        for p in d.get("single_ended_diff_pairs", []): status[p] = "single_ended"
        for p in d.get("failed_diff_pairs", []):       status[p] = "failed"
    total = len(status)
    coupled = sum(1 for v in status.values() if v == "coupled")
    return {"diff_pairs_total": total,
            "diff_pairs_coupled": coupled,
            "diff_pairs_single_ended": sum(1 for v in status.values() if v == "single_ended"),
            "diff_pairs_failed": sum(1 for v in status.values() if v == "failed"),
            "diff_coupled_pct": round(100.0 * coupled / total, 1) if total else None}


def _kicad_grade(pcb, clearance):
    """KiCad's own DRC verdict on the final board (#316): copper-class
    violation count + the two-sided diff vs check_drc, graded with the
    netclass clearance equalized to the routed clearance (kicad_drc_compare's
    staging). Returns Nones when kicad-cli is unavailable so replay grading
    still works everywhere."""
    try:
        sys.path.insert(0, str(REPO / "tests" / "stress"))
        from kicad_drc_compare import (KICAD_CLI, _staged_copy, run_kicad_drc,
                                       run_check_drc, match)
        if not os.path.exists(KICAD_CLI):
            return {"kicad_drc": None, "kicad_only": None, "checkdrc_only": None}
        tmpd, staged = _staged_copy(pcb, float(clearance))
        try:
            kicad, err = run_kicad_drc(staged)
        finally:
            import shutil
            shutil.rmtree(tmpd, ignore_errors=True)
        if err:
            return {"kicad_drc": None, "kicad_only": None, "checkdrc_only": None}
        cd = run_check_drc(pcb, float(clearance))
        _, kicad_only, cd_only = match(kicad, cd)
        return {"kicad_drc": len(kicad), "kicad_only": len(kicad_only),
                "checkdrc_only": len(cd_only)}
    except Exception:
        return {"kicad_drc": None, "kicad_only": None, "checkdrc_only": None}


def grade(pcb, clearance):
    # Grade at the board's OWN recorded floors when the sibling .kicad_pro
    # exists (check_drc auto-grades from it -- the Stage-C clearance ledger
    # writes the true minimum every step used, including legitimate fine-tap
    # escalations BELOW the manifest --clearance). Grading such copper at the
    # manifest clearance manufactures phantom violations: nrfmicro's plane
    # repair fine-tapped R5.2 at a recorded 0.127 floor and the 0.2 manifest
    # grade counted +13 "regressions" on a board whose auto-grade is clean
    # (#347 A/B). The manifest clearance stays as the fallback for finals
    # shipped without a .kicad_pro (the daisho gap, #217).
    pro = Path(pcb).with_suffix(".kicad_pro")
    drc_args = [sys.executable, "-X", "utf8", str(REPO / "check_drc.py"), pcb, "--quiet"]
    # Auto-grade only when the .kicad_pro actually RECORDS a clearance: a
    # stock/minimal project copied through the chain has none, and check_drc
    # would silently fall back to 0.2 -- manufacturing phantom sub-clearance
    # violations on a 0.1-routed board (the tigard class, again).
    _recorded = None
    if pro.exists():
        try:
            import json as _json
            sys.path.insert(0, str(REPO))
            from fix_kicad_drc_settings import project_copper_clearance
            with open(pro) as _f:
                _recorded = project_copper_clearance(_json.load(_f))
        except Exception:
            _recorded = None
    if _recorded is None:
        drc_args[-1:-1] = ["-c", clearance]
    drc = subprocess.run(drc_args, capture_output=True, text=True)
    # NOT --quiet: the "Checking N routed nets" total (needed for completion %)
    # only prints in non-quiet mode; the unrouted/connectivity-issue counts print
    # either way.
    conn = subprocess.run([sys.executable, "-X", "utf8", str(REPO / "check_connected.py"), pcb],
                          capture_output=True, text=True)
    ctext = conn.stdout + conn.stderr
    total, incomplete, pct = _completion(ctext)
    out = {"drc": _drc_count(drc.stdout + drc.stderr), "conn": _conn_count(ctext),
           "nets_total": total, "nets_incomplete": incomplete, "completion_pct": pct}
    out.update(_kicad_grade(pcb, clearance))
    return out


def do_board(set_dir, out_dir, label, board):
    manifest = set_dir / board / "redo_commands.sh"
    src = str(set_dir / board)
    dst = str(out_dir / board)
    Path(dst).mkdir(parents=True, exist_ok=True)
    txt = manifest.read_text()
    clr = route_clearance(txt)
    timings_path = f"{dst}/timings.json"
    with open(f"{dst}/_replay.log", "w") as log:
        rc = subprocess.run([sys.executable, str(REPO / "tests/stress/redo_stress_test.py"),
                             str(manifest), "--remap", f"{src}:{dst}",
                             "--skip-checks", "--continue-on-error",
                             "--timings-out", timings_path],
                            stdout=log, stderr=subprocess.STDOUT).returncode
    # Per-step wall-clock (for A/B timing comparison): keep the raw per-command
    # list and a per-tool sum (route.py / route_planes.py / ... -- where the
    # vectorization speedups land), plus the total.
    steps, time_by_tool, peak_by_tool, total_s, peak_board = [], {}, {}, 0.0, 0.0
    if os.path.exists(timings_path):
        for c in json.loads(Path(timings_path).read_text()).get("commands", []):
            tool = _tool_of(c.get("argv", []))
            sec = c.get("seconds", 0.0)
            pk = c.get("peak_rss_mb", 0.0)
            steps.append({"tool": tool, "seconds": sec, "peak_rss_mb": pk, "rc": c.get("returncode")})
            time_by_tool[tool] = round(time_by_tool.get(tool, 0.0) + sec, 3)
            peak_by_tool[tool] = round(max(peak_by_tool.get(tool, 0.0), pk), 1)  # max, not sum
            total_s += sec
            peak_board = max(peak_board, pk)
    # Coupled diff-pair completion is parsed from route_diff's JSON_SUMMARY in the
    # replay log (captured above), so it reflects what actually coupled-routed.
    log_path = f"{dst}/_replay.log"
    dp = _diff_pair_stats(Path(log_path).read_text(errors="replace") if os.path.exists(log_path) else "")
    fname = final_output_name(txt)
    final = os.path.join(dst, fname) if fname else None
    done = bool(final) and os.path.exists(final)
    res = {"board": board, "clearance": clr, "replay_rc": rc,
           "final": fname if done else None, "chain_complete": done,
           "drc": None, "conn": None, "nets_total": None, "nets_incomplete": None,
           "completion_pct": None,
           "total_seconds": round(total_s, 3), "peak_rss_mb": round(peak_board, 1),
           "time_by_tool": time_by_tool, "peak_by_tool": peak_by_tool, "steps": steps,
           **dp}
    if done:
        res.update(grade(final, clr))
    dps = f"{res['diff_pairs_coupled']}/{res['diff_pairs_total']}" if res['diff_pairs_total'] else "-"
    print(f"[{label}] {board}: chain={'ok' if done else 'BROKEN'} "
          f"drc={res['drc']} kdrc={res.get('kicad_drc')} conn={res['conn']} compl={res['completion_pct']}% "
          f"dpair={dps} t={res['total_seconds']}s peak={res['peak_rss_mb']}MB final={res['final']}", flush=True)
    return res


def run_wave(set_dir, out_dir, label, jobs):
    # Skip dot-prefixed dirs: those are headless-worker artifacts (a paused /
    # NORESULT retry copy of a real board, e.g. `.framework_dock_noresult_...`),
    # not corpus boards -- the clean board dir is present separately (see memory
    # stress-headless-worker-noresult).
    boards = sorted(d.name for d in set_dir.iterdir()
                    if not d.name.startswith(".") and (d / "redo_commands.sh").exists())
    out_dir.mkdir(parents=True, exist_ok=True)
    ver = write_git_version(out_dir, REPO, label=label)  # provenance: which code produced this wave
    print(f"[{label}] code under test: {format_version(ver)}  (commit {ver.get('commit')})")
    print(f"[{label}] replaying {len(boards)} boards from {set_dir} -> {out_dir} ({jobs} parallel)")
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = [ex.submit(do_board, set_dir, out_dir, label, b) for b in boards]
        for f in concurrent.futures.as_completed(futs):  # report as boards finish
            results.append(f.result())
    results.sort(key=lambda r: r["board"])
    summary = out_dir / "summary.json"
    summary.write_text(json.dumps(results, indent=2))
    complete = sum(1 for r in results if r["chain_complete"])
    print(f"[{label}] wrote {summary}: {complete}/{len(results)} chains complete")
    return results


def regrade(out_dir, set_dir):
    """Re-grade an existing wave's final boards (no re-routing) and rewrite its
    summary.json -- e.g. after a grading fix like the route_clearance change, or
    to reuse a prior wave as a baseline."""
    out_dir = Path(out_dir); set_dir = Path(set_dir)
    results = []
    for bdir in sorted(p for p in out_dir.iterdir() if p.is_dir()):
        b = bdir.name
        man = set_dir / b / "redo_commands.sh"
        if not man.exists():
            continue
        txt = man.read_text(); clr = route_clearance(txt)
        fname = final_output_name(txt)
        final = bdir / fname if fname else None
        done = bool(final) and final.exists()
        lp = bdir / "_replay.log"
        dp = _diff_pair_stats(lp.read_text(errors="replace") if lp.exists() else "")
        res = {"board": b, "clearance": clr, "replay_rc": 0,
               "final": fname if done else None, "chain_complete": done,
               "drc": None, "conn": None, "nets_total": None, "nets_incomplete": None,
               "completion_pct": None, **dp}  # regrade re-runs no commands, so no timing
        if done:
            res.update(grade(str(final), clr))
        print(f"[regrade] {b}: chain={'ok' if done else 'BROKEN'} drc={res['drc']} "
              f"conn={res['conn']} compl={res['completion_pct']}%")
        results.append(res)
    (out_dir / "summary.json").write_text(json.dumps(results, indent=2))
    print(f"[regrade] rewrote {out_dir/'summary.json'}")
    return results


def _fmt(v):
    return "-" if v is None else str(v)


def compare(old_json, new_json):
    old = {r["board"]: r for r in json.loads(Path(old_json).read_text())}
    new = {r["board"]: r for r in json.loads(Path(new_json).read_text())}
    ov, nv = load_git_version(old_json), load_git_version(new_json)
    if ov or nv:
        print(f"code: old = {format_version(ov)}   new = {format_version(nv)}")
    boards = sorted(set(old) | set(new))
    # Connectivity is graded on TOTAL INCOMPLETE nets (unrouted + connectivity-issue
    # nets), NOT the "conn" (connectivity-issue) count alone: a net that loses its
    # copper entirely leaves the conn bucket for the unrouted bucket, so conn can
    # DROP while the board got worse. incomplete = nets_incomplete (falls back to
    # conn for pre-#-schema summaries that lack it).
    def _incompl(r):
        ni = r.get("nets_incomplete")
        return ni if ni is not None else r.get("conn")
    print(f"{'board':14} {'drc o->n':>11} {'incompl o->n':>13} {'compl% o->n':>13} "
          f"{'dpair o->n':>13} {'time(s) o->n':>16} {'peakMB o->n':>15}  note")
    print("-" * 128)
    drc_delta = incompl_delta = dpair_delta = 0
    t_old = t_new = 0.0
    time_old, time_new = {}, {}   # per-tool wall-clock summed across boards (speedup view)
    pk_old, pk_new = {}, {}       # per-tool peak RSS = max across boards (memory view)
    for b in boards:
        o, n = old.get(b), new.get(b)
        oc = o and o["chain_complete"]
        nc = n and n["chain_complete"]
        if not (oc and nc):
            print(f"{b:14} {'-':>11} {'-':>13} {'-':>13} {'-':>13} {'-':>16} {'-':>15}  chain incomplete "
                  f"(old={'ok' if oc else 'broken'}, new={'ok' if nc else 'broken'}) -- excluded")
            continue
        oi, ni = _incompl(o), _incompl(n)
        dd = n["drc"] - o["drc"]
        cd = (ni - oi) if (oi is not None and ni is not None) else 0  # incomplete-net delta
        drc_delta += dd; incompl_delta += cd
        # coupled diff-pair count (fewer coupled = quality regression)
        ocp, ncp = o.get("diff_pairs_coupled"), n.get("diff_pairs_coupled")
        odt, ndt = o.get("diff_pairs_total"), n.get("diff_pairs_total")
        dpd = (ncp - ocp) if (ocp is not None and ncp is not None) else 0
        dpair_delta += dpd
        dp_o = f"{ocp}/{odt}" if odt else "-"
        dp_n = f"{ncp}/{ndt}" if ndt else "-"
        to, tn = o.get("total_seconds"), n.get("total_seconds")
        if to is not None and tn is not None:
            t_old += to; t_new += tn
        for src, dst in ((o.get("time_by_tool", {}), time_old), (n.get("time_by_tool", {}), time_new)):
            for k, v in src.items():
                dst[k] = round(dst.get(k, 0.0) + v, 3)
        for src, dst in ((o.get("peak_by_tool", {}), pk_old), (n.get("peak_by_tool", {}), pk_new)):
            for k, v in src.items():
                dst[k] = round(max(dst.get(k, 0.0), v), 1)
        op, npc = o.get("completion_pct"), n.get("completion_pct")
        po, pn = o.get("peak_rss_mb"), n.get("peak_rss_mb")
        flag = ""
        if dd > 0 or cd > 0 or dpd < 0:    flag = "  <-- REGRESSION"
        elif dd < 0 or cd < 0 or dpd > 0:  flag = "  improved"
        speed = f"  ({to/tn:.2f}x)" if (to and tn) else ""
        print(f"{b:14} {o['drc']:>4} -> {n['drc']:<4} {_fmt(oi):>5} -> {_fmt(ni):<5} "
              f"{_fmt(op):>5} -> {_fmt(npc):<5} {dp_o:>5} -> {dp_n:<5} "
              f"{_fmt(to):>6} -> {_fmt(tn):<6}{speed:>9} {_fmt(po):>6} -> {_fmt(pn):<6}{flag}")
    print("-" * 128)
    verdict = "REGRESSION" if (drc_delta > 0 or incompl_delta > 0 or dpair_delta < 0) else "no regression"
    print(f"net delta: drc {drc_delta:+d}, incomplete nets {incompl_delta:+d} "
          f"(unrouted + connectivity-issue), coupled diff-pairs {dpair_delta:+d}"
          f"  ==>  {verdict}")
    if t_old and t_new:
        print(f"total replay wall-clock: {t_old:.1f}s -> {t_new:.1f}s  ({t_old/t_new:.2f}x)")
        print("\nper-tool: wall-clock (summed) and peak RSS (max) over boards complete in both:")
        print(f"  {'tool':28} {'t_old(s)':>9} {'t_new(s)':>9} {'speedup':>8}   "
              f"{'peak_old':>9} {'peak_new':>9}")
        for tool in sorted(set(time_old) | set(time_new), key=lambda k: -time_new.get(k, 0)):
            ot, nt = time_old.get(tool, 0.0), time_new.get(tool, 0.0)
            sp = f"{ot/nt:.2f}x" if nt else "-"
            print(f"  {tool:28} {ot:>9.1f} {nt:>9.1f} {sp:>8}   "
                  f"{pk_old.get(tool, 0):>8.0f}MB {pk_new.get(tool, 0):>8.0f}MB")
    else:
        print("(timing/peak not available in one/both summaries -- re-run waves with the "
              "current ab_replay_grade to capture per-step timing + peak memory)")
    return drc_delta <= 0 and incompl_delta <= 0 and dpair_delta >= 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", help="Set runs dir, e.g. ~/Documents/kicad_stress_test/runs_set3")
    ap.add_argument("--out", help="Wave output dir (per code version)")
    ap.add_argument("--label", default="wave", help="Label for log lines (e.g. old / new)")
    ap.add_argument("--jobs", type=int, default=4, help="Boards in parallel (default 4)")
    ap.add_argument("--compare", nargs=2, metavar=("OLD.json", "NEW.json"),
                    help="Compare two wave summaries and print a regression table")
    ap.add_argument("--regrade", metavar="WAVE_DIR",
                    help="Re-grade an existing wave's finals (no re-routing) and rewrite its summary.json")
    args = ap.parse_args()

    if args.compare:
        ok = compare(*args.compare)
        return 0 if ok else 1
    if args.regrade:
        if not args.set:
            ap.error("--regrade needs --set (for per-board manifests)")
        regrade(args.regrade, Path(args.set).expanduser())
        return 0
    if not args.set or not args.out:
        ap.error("--set and --out are required (or use --compare/--regrade)")
    run_wave(Path(args.set).expanduser(), Path(args.out).expanduser(), args.label, args.jobs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
