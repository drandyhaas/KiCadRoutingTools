#!/usr/bin/env python3
"""Replay a stress-test board run deterministically, with no LLM (issue #132).

The original board run is driven by an LLM agent following RUNBOOK.md, but every
routing/fanout/plane/check command goes through tests/stress/run_limited.sh,
which records each command (fully quoted argv + the cwd it ran in) to a manifest
(default <run-dir>/redo_commands.sh). This script replays that manifest to
reproduce the exact final board in seconds with no API calls -- which makes runs
reproducible and lets an engine change be A/B'd cleanly (replay the same manifest
with the change on vs off).

By default the replay is PRUNED to the file-dependency chain that produces the
final board (issue #231): the agent's superseded retries (a route command it ran
5x, only the last feeding downstream) and dead-end branches (an output nothing
consumes) are skipped. This is much faster and deterministic -- each kept command
re-reads its own input board, so dropping the overwritten writes can't change its
result. Pass --verbatim to replay every recorded command literally.

Usage:
  redo_stress_test.py <manifest> [--remap OLD:NEW ...] [--verbatim] [--skip-checks] [--dry-run]

--remap rewrites a path prefix in every argument, so a run whose intermediates
were written with absolute paths under one run dir can be replayed into a fresh
directory (the source board, referenced by its own absolute path, still resolves):
  redo_stress_test.py runs_set1/ottercast/redo_commands.sh \
      --remap /...kicad_stress_test/runs_set1/ottercast:/tmp/redo_A
"""

import argparse
import os
import json
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # for _gitver when run as a script
from _gitver import write_git_version, format_version

REPO = Path(__file__).resolve().parent.parent.parent  # tests/stress/ -> repo root


def _tree_rss_kb(pid):
    """Resident set size (KB) of a process plus its direct children, via ps --
    same tree-RSS method run_limited.sh uses for its memory watchdog."""
    total = 0
    try:
        out = subprocess.run(["ps", "-o", "rss=", "-p", str(pid)],
                             capture_output=True, text=True).stdout
        total += sum(int(x) for x in out.split())
        kids = subprocess.run(["pgrep", "-P", str(pid)],
                              capture_output=True, text=True).stdout.split()
        for k in kids:
            o2 = subprocess.run(["ps", "-o", "rss=", "-p", k],
                                capture_output=True, text=True).stdout
            total += sum(int(x) for x in o2.split())
    except Exception:
        pass
    return total


def _footprint_mb(pid):
    """phys_footprint (MB) of one pid via /usr/bin/footprint (darwin only).

    macOS RSS (ps) under-reports the true memory a routing process holds:
    mimalloc-retained pages and GPU-driver / IOAccelerator-tagged regions
    (issue #419) are part of phys_footprint but not of the RSS ps reports.
    `footprint` is the authoritative number the OS uses for memory pressure.
    Returns 0.0 on any error or non-darwin."""
    if sys.platform != "darwin":
        return 0.0
    try:
        out = subprocess.run(["/usr/bin/footprint", str(pid)],
                             capture_output=True, text=True, timeout=15).stdout
        for line in out.splitlines():
            if "Footprint:" in line and "phys_footprint" not in line:
                parts = line.split("Footprint:")[1].strip().split()
                val = float(parts[0])
                unit = parts[1] if len(parts) > 1 else "MB"
                if unit == "GB":
                    val *= 1024
                elif unit == "KB":
                    val /= 1024
                return val
    except Exception:
        pass
    return 0.0


def _tree_footprint_mb(pid):
    """Sum phys_footprint (MB) of a process plus its direct children (darwin
    only). The footprint tool is ~10x costlier than `ps` (it forks and reads
    the VM map), so callers sample it coarsely (every few seconds), unlike the
    0.5s RSS cadence."""
    if sys.platform != "darwin":
        return 0.0
    total = _footprint_mb(pid)
    try:
        kids = subprocess.run(["pgrep", "-P", str(pid)],
                              capture_output=True, text=True).stdout.split()
        for k in kids:
            total += _footprint_mb(k)
    except Exception:
        pass
    return total


def run_with_peak_rss(argv, cwd, interval=0.5, timeout=None, footprint_interval=5.0):
    """Run a command (inheriting stdout/stderr) while sampling its process-tree
    RSS; return (returncode, peak_rss_kb, timed_out, peak_footprint_mb). Lets the
    replay record per-step peak memory so an engine change's memory effect is
    comparable, not just its time.

    On darwin also samples /usr/bin/footprint (phys_footprint, MB) every
    ``footprint_interval`` seconds -- the authoritative memory number that
    captures mimalloc-retained + IOAccelerator-tagged memory RSS misses
    (issue #419). ``peak_footprint_mb`` is 0.0 on non-darwin.

    If timeout (seconds) is given and the command runs past it, kill the process
    tree and return timed_out=True. The original LLM run has a per-command cap
    (run_board.sh's 3-hour/command budget); the replay had none, so a board that
    wedges in rip-up (issue #211: ulx3s' 40-min+ non-terminating final route)
    could hang a whole sweep forever. A generous cap (default 3h) lets slow-but-
    finishing boards complete while still bailing a true non-termination."""
    p = subprocess.Popen(argv, cwd=cwd)
    peak = 0
    peak_fp = 0.0
    start = time.time()
    # Sample footprint first at start+interval (not at t~0, which would catch
    # only the pre-allocation startup value); commands shorter than one interval
    # get no footprint sample and report 0 (honest -- they aren't the memory
    # concern #419 targets, which are the minutes-long heavy route.py steps).
    last_fp = start
    timed_out = False
    while p.poll() is None:
        peak = max(peak, _tree_rss_kb(p.pid))
        now = time.time()
        if sys.platform == "darwin" and now - last_fp >= footprint_interval:
            peak_fp = max(peak_fp, _tree_footprint_mb(p.pid))
            last_fp = now
        if timeout is not None and now - start > timeout:
            timed_out = True
            # Kill the children first (the heavy router workers), then the wrapper.
            for k in subprocess.run(["pgrep", "-P", str(p.pid)],
                                    capture_output=True, text=True).stdout.split():
                try:
                    os.kill(int(k), 9)
                except (ValueError, ProcessLookupError):
                    pass
            try:
                p.kill()
            except ProcessLookupError:
                pass
            p.wait()
            break
        time.sleep(interval)
    return p.returncode, peak, timed_out, peak_fp


def parse_manifest(path):
    """Yield (cwd, argv) for each recorded command.

    A `# cwd=` line sets the working directory for every following command until
    the next `# cwd=` -- shell-faithful (a `cd` persists), NOT one-shot. run_limited.sh
    records a `# cwd=` before each command it wraps, so normally every command
    carries its own and the stickiness is a no-op. It matters for hand-spliced
    steps that share the preceding command's cwd: the de-hole `cp <step> <final>`
    + follow-up pair (#334/#345), where the `cp` consumes the `# cwd=` and the
    follow-up command has none of its own. Resetting to None there made the
    follow-up run in the launcher's cwd, breaking the relative-path chain
    (FileNotFoundError on the just-cp'd board) and falsely marking the board
    chain-broken under --remap."""
    cmds = []
    current_cwd = None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("# cwd="):
                # The cwd was recorded with %q quoting on the value after 'cwd='.
                current_cwd = " ".join(shlex.split(line[len("# cwd="):]))
                continue
            if not line or line.startswith("#") or line in ("set -e",):
                continue
            if line.startswith("#!"):
                continue
            argv = shlex.split(line)
            if argv:
                cmds.append((current_cwd, argv))
    return cmds


def apply_remaps(argv, remaps):
    out = []
    for a in argv:
        for old, new in remaps:
            if old in a:
                a = a.replace(old, new)
        out.append(a)
    return out


def is_check_cmd(argv):
    return any(os.path.basename(a).startswith("check_") for a in argv)


def mirror_project_sibling(argv, cwd):
    """After a `cp`/`mv` of a .kicad_pcb, carry its sibling .kicad_pro too.

    Routing steps record the run's true DRC floor (the smallest clearance / via /
    hole-to-hole actually used, incl. fine-tap escalations below nominal) in the
    output board's sibling `.kicad_pro`; `check_drc` and the KiCad grader read it
    back so legitimately-tight copper is graded at the floor it was routed to, not
    the board's looser design netclass. A bare `cp step.kicad_pcb final.kicad_pcb`
    (the de-hole rename, #334/#345) copies only the .kicad_pcb, stranding
    `final.kicad_pcb` with NO .kicad_pro -- so grading silently falls back to the
    board's default netclass and manufactures phantom clearance/hole violations on
    fine-tapped copper (core1106_cam: clean at the routed 0.09, but graded at the
    0.2 default; #403/#326). Mirror the .kicad_pro so the recorded floor travels
    with the board it describes."""
    if not argv or argv[0] not in ("cp", "mv"):
        return
    pcbs = [a for a in argv[1:] if a.endswith(".kicad_pcb")]
    if len(pcbs) != 2:  # only a plain <src> <dst> rename/copy
        return
    def _resolve(p):
        return p if os.path.isabs(p) else os.path.join(cwd or ".", p)
    src_pro = _resolve(pcbs[0])[:-len(".kicad_pcb")] + ".kicad_pro"
    dst_pro = _resolve(pcbs[1])[:-len(".kicad_pcb")] + ".kicad_pro"
    if (not os.path.isfile(src_pro)
            or os.path.abspath(src_pro) == os.path.abspath(dst_pro)):
        return
    try:
        if argv[0] == "mv":
            shutil.move(src_pro, dst_pro)
        else:
            shutil.copy2(src_pro, dst_pro)
        print(f"    mirrored DRC floor -> {os.path.basename(dst_pro)}")
    except OSError as e:
        print(f"    (could not mirror .kicad_pro: {e})")


def board_io(argv):
    """Return (input_basenames, output_basename) for a command's .kicad_pcb args.

    Convention for every tool here: the LAST .kicad_pcb token is the output and
    the earlier ones are inputs -- this covers both positional `<in> <out>`
    (route*.py, place_fanout_clearance.py) and `<in> --output <out>`
    (bga_fanout.py, qfn_fanout.py), because the --output value still appears after
    the input in argv. Boards are keyed by BASENAME so the dependency analysis is
    invariant under --remap / --workdir path rewriting (issue #231). Returns
    ([], None) when the command names no board (e.g. `--help`)."""
    toks = [a for a in argv if a.endswith(".kicad_pcb")]
    if not toks:
        return [], None
    return [os.path.basename(t) for t in toks[:-1]], os.path.basename(toks[-1])


def compute_prune_keep(cmds):
    """Indices (0-based) of the commands needed to reproduce the final board.

    Builds the file-dependency DAG (issue #231) and keeps only the transitive
    chain feeding the final board, dropping the agent's superseded retries and
    dead-end branches:

    - Forward pass tracks `producer[board] = last command index that wrote it`, so
      a command's inputs bind to the THEN-CURRENT producer -- i.e. the last write
      before that read. A board rewritten N times therefore links only its last
      writer to whoever consumes it; the earlier writes gain no dependents.
    - The final target is the last non-check command that writes a board.
    - keep = the final command + everything transitively reachable through those
      input->producer edges. Non-chain non-check commands (superseded writes,
      never-consumed dead ends like `*_retry`) fall out.
    - A check command is kept only if every board it reads is still produced by
      the kept chain or is an external/seeded input (never produced by any
      command) -- so we never check a board no kept command makes.

    Returns (keep:set[int], info:dict) where info has final_index, final_board,
    and the dropped non-check indices (for a transparent report).

    Caveat (issue #231): keeping the last writer of each consumed board assumes
    that writer SUCCEEDED in the original run -- true when the agent only advanced
    after a success, the normal case. The manifest records no exit status, so a
    rare "earlier attempt succeeded, later attempt failed" ordering would keep a
    failing command; rerun without --prune (verbatim) if a pruned replay fails."""
    n = len(cmds)
    producer = {}
    deps = [set() for _ in range(n)]
    out_of = [None] * n
    in_of = [[] for _ in range(n)]
    final = None
    for i, (_cwd, argv) in enumerate(cmds):
        ins, out = board_io(argv)
        in_of[i] = ins
        if is_check_cmd(argv):
            continue  # reads boards but produces none; not part of the producer chain
        out_of[i] = out
        deps[i] = {producer[b] for b in ins if b in producer}
        if out is not None:
            producer[out] = i
            final = i

    if final is None:
        return set(range(n)), {"final_index": None, "final_board": None, "dropped": []}

    keep = set()
    stack = [final]
    while stack:
        j = stack.pop()
        if j in keep:
            continue
        keep.add(j)
        stack.extend(deps[j])

    produced_any = {o for o in out_of if o}
    produced_kept = {out_of[i] for i in keep if out_of[i]}
    for i, (_cwd, argv) in enumerate(cmds):
        if not is_check_cmd(argv):
            continue
        # keep a check iff every board it reads exists after pruning
        if all(b in produced_kept or b not in produced_any for b in in_of[i]):
            keep.add(i)

    dropped = [i for i in range(n) if i not in keep and not is_check_cmd(cmds[i][1])]

    # CHAIN-HOLE detection (zynq final_board lesson): a kept command reading a
    # board that NO recorded command produces is normally just the seed input
    # (first command). Any OTHER unproduced input means the agent hand-created
    # a board mid-run (cp/mv is not recorded), so the pruner cannot see past
    # it -- the replay would silently run current code on STALE original-run
    # copper and the A/B verdict is contaminated. Callers must warn loudly.
    first_nc = next((i for i in range(n) if not is_check_cmd(cmds[i][1])), None)
    seed_ok = set(in_of[first_nc]) if first_nc is not None else set()
    holes = sorted({b for i in keep if not is_check_cmd(cmds[i][1])
                    for b in in_of[i]
                    if b not in produced_any and b not in seed_ok
                    and str(b).endswith('.kicad_pcb')})
    return keep, {"final_index": final, "final_board": out_of[final],
                  "dropped": dropped, "chain_holes": holes}


def compute_contamination(cmds):
    """Which outputs of a replay carry STALE original-run copper (issue #315
    post-mortem: zynq's step5d read `final_board.kicad_pcb`, a file the worker
    cp'd mid-run that no manifest command regenerates, so a --verbatim replay
    silently grafted the ORIGINAL run's copper into every later step and the
    'fix still broken at step5e' verdict was contamination, not code).

    A command is TAINTED if it reads a chain-hole file (a .kicad_pcb no command
    produces, other than the first command's seed inputs) or reads a board whose
    latest producer is tainted. Returns (holes, tainted_outputs, last_clean_out):
    the hole files, every tainted output board, and the last board written by a
    clean command -- the artifact a grader should use."""
    produced_any = set()
    for _cwd, argv in cmds:
        if not is_check_cmd(argv):
            _ins, out = board_io(argv)
            if out:
                produced_any.add(out)

    holes = set()
    tainted_boards = set()
    tainted_outputs = []
    last_clean_out = None
    seen_output = False
    seed_ok = set()
    for _cwd, argv in cmds:
        if is_check_cmd(argv):
            continue
        ins, out = board_io(argv)
        # An unproduced input is a legit external seed when it's an absolute
        # path (corpus source board -- identical in both runs) or when it is
        # first read before ANY output exists (a hand-copied `*_input` seed).
        # Only an unproduced RELATIVE input first read mid-chain is original-
        # run copper the replay can't regenerate.
        cmd_holes = set()
        for b in ins:
            if not str(b).endswith('.kicad_pcb') or b in produced_any or b in seed_ok:
                continue
            if os.path.isabs(str(b)) or not seen_output:
                seed_ok.add(b)
            else:
                cmd_holes.add(b)
        holes |= cmd_holes
        tainted = bool(cmd_holes) or any(b in tainted_boards for b in ins)
        if out:
            seen_output = True
            if tainted:
                tainted_boards.add(out)
                tainted_outputs.append(out)
            else:
                tainted_boards.discard(out)  # a clean rewrite launders the name
                last_clean_out = out
    return sorted(holes), tainted_outputs, last_clean_out


def warn_contamination(cmds):
    """Print the replay-contamination warning (both pruned and verbatim modes).
    Returns True if any contamination was found."""
    holes, tainted, last_clean = compute_contamination(cmds)
    if not holes:
        return False
    print("\n" + "!" * 70)
    print("WARNING: REPLAY CONTAMINATION -- these input board(s) are produced by")
    print("NO recorded command (the agent cp/mv'd them mid-run), so the replay")
    print("seeds them from the ORIGINAL run dir with the ORIGINAL code's copper:")
    for h in holes:
        print(f"    {h}")
    if tainted:
        print("Every output downstream of them is part original-run copper -- do NOT")
        print("grade these to judge current code:")
        for t in tainted:
            print(f"    {t}")
    if last_clean:
        print(f"Last output with fully current-code ancestry (grade THIS): {last_clean}")
    print("!" * 70 + "\n")
    return True


def seed_input_boards(manifest, cmds, dest_dir):
    """Copy 'input-only' relative .kicad_pcb files into the replay dest dir.

    Some runs start from a board the agent renamed/copied into the run dir (e.g.
    `<board>_input.kicad_pcb`) WITHOUT that copy being a recorded command, then
    reference it by a RELATIVE path. On replay into a fresh dir those files don't
    exist, so the very first command fails and the whole chain breaks (e.g.
    usb_sniffer). Such files are 'input-only': referenced as an argument but never
    produced as any command's output. Copy them from the manifest's own dir (the
    original run dir) into dest so relative-path inputs resolve.

    Absolute-path inputs (the source board under boards_unrouted*/) already resolve
    and are skipped. Returns the list of basenames seeded."""
    def kpcb(argv):
        return [t for t in argv if t.endswith(".kicad_pcb")]
    produced, referenced = set(), set()
    for _cwd, argv in cmds:
        if is_check_cmd(argv):
            continue
        toks = kpcb(argv)
        if toks:
            produced.add(os.path.basename(toks[-1]))  # last .kicad_pcb = output
        for t in toks:
            referenced.add(t)
    src_dir = os.path.dirname(os.path.abspath(manifest))
    seeded = []
    for t in sorted(referenced):
        base = os.path.basename(t)
        if os.path.isabs(t) or base in produced:
            continue  # absolute inputs resolve already; produced files get written
        src = os.path.join(src_dir, base)
        dst = os.path.join(dest_dir, base)
        if os.path.exists(src) and not os.path.exists(dst):
            import shutil
            shutil.copy2(src, dst)
            seeded.append(base)
            # Carry the sibling .kicad_pro too (same custody rule as
            # mirror_project_sibling): the seed board's project holds the
            # design's real DRC rules -- min_copper_edge_clearance above all
            # (#338: ecp5_mini's 0.01 edge rule lost at seed -> planes step
            # wrote its 0.5 zone-pullback default as the constraint -> 82
            # phantom kicad edge items; rp2350_dev's 0.5 rule lost -> route
            # steps never honored the band). Floors applied downstream are
            # lower-only, so carrying the project is always safe.
            src_pro = src[:-len(".kicad_pcb")] + ".kicad_pro"
            dst_pro = dst[:-len(".kicad_pcb")] + ".kicad_pro"
            # Stem-mismatch fallback (the openstint gap): stress workers
            # RENAMED the pristine board into the run dir (board0.kicad_pcb)
            # but left the project under the ORIGINAL board name
            # (<run_dir_name>.kicad_pro), so no same-stem sibling exists and
            # the design's DRC rules (0.3 edge on openstint) silently vanish
            # for the whole chain. The run dir is named after the board, so
            # that pro is the pristine project -- carry it under the seeded
            # stem.
            if not os.path.exists(src_pro):
                board_named = os.path.join(src_dir,
                                           os.path.basename(os.path.normpath(src_dir)) + ".kicad_pro")
                if os.path.exists(board_named):
                    print(f"    (seed {base}: no same-stem .kicad_pro; carrying "
                          f"{os.path.basename(board_named)} as {os.path.basename(dst_pro)})")
                    src_pro = board_named
            if os.path.exists(src_pro) and not os.path.exists(dst_pro):
                shutil.copy2(src_pro, dst_pro)
                seeded.append(os.path.basename(src_pro))
    return seeded


def main():
    ap = argparse.ArgumentParser(description="Replay a recorded stress-test manifest (no LLM).")
    ap.add_argument("manifest", help="Path to redo_commands.sh manifest")
    ap.add_argument("--remap", action="append", default=[],
                    help="OLD:NEW path-prefix rewrite applied to every argument (repeatable)")
    ap.add_argument("--workdir", metavar="DIR",
                    help="Run EVERY command in DIR (created if needed), ignoring each "
                         "command's recorded `# cwd=`. Use this to replay into a fresh "
                         "directory: the manifest's relative output paths then chain "
                         "correctly inside DIR. Without it, a command runs in its recorded "
                         "cwd -- which silently OVERWRITES the original run's boards if "
                         "that cwd isn't covered by --remap. The source board (absolute "
                         "path) still resolves either way.")
    ap.add_argument("--skip-checks", action="store_true",
                    help="Skip check_*.py commands (they do not mutate the board)")
    ap.add_argument("--verbatim", action="store_true",
                    help="Replay EVERY recorded command in order, including the agent's "
                         "superseded retries and dead-end branches. By default the replay "
                         "PRUNES to the file-dependency chain that produces the final "
                         "board (issue #231) -- much faster on dense boards (e.g. a "
                         "signal-route command recorded 5x runs once) and deterministic "
                         "(each kept command re-reads its own input, independent of the "
                         "dropped writes). Use --verbatim to reproduce the literal "
                         "sequence, or if a pruned replay fails on a failed-retry "
                         "ordering (see compute_prune_keep()).")
    ap.add_argument("--dry-run", action="store_true", help="Print the plan, run nothing")
    ap.add_argument("--skip-validate", action="store_true",
                    help="Skip the parser-parity validation (validate_pcb_data.py: "
                         "pcbnew-built PCBData vs text parse) of the final board. "
                         "The validation needs KiCad's bundled python and adds one "
                         "board load; it is non-fatal either way.")
    ap.add_argument("--continue-on-error", action="store_true",
                    help="Keep going if a command fails (default: replicate the agent's "
                         "sequence, where a failed step is followed by its retry)")
    ap.add_argument("--timings-out", metavar="PATH",
                    help="Write per-command wall-clock timings to PATH (JSON) for "
                         "comparison across code versions")
    ap.add_argument("--timeout", type=float, default=10800, metavar="SECONDS",
                    help="Per-command wall-clock cap (default 10800 = 3h). A command "
                         "that runs past it is killed (process tree) and counted as a "
                         "failure, so a non-terminating board (issue #211) can't wedge "
                         "the whole replay forever. Pass 0 to disable the cap.")
    args = ap.parse_args()

    # Record a per-copper route trace for each replayed route.py / route_diff.py
    # step (#482) so render_run can splice fine rip/restore animation into the
    # whole-run movie. Default-on; set KICAD_ROUTE_TRACE=0 to skip.
    os.environ.setdefault("KICAD_ROUTE_TRACE", "1")

    remaps = []
    for r in args.remap:
        if ":" not in r:
            ap.error(f"--remap needs OLD:NEW, got {r!r}")
        old, new = r.split(":", 1)
        remaps.append((old, new))
        os.makedirs(new, exist_ok=True)

    if args.workdir:
        os.makedirs(args.workdir, exist_ok=True)

    # Provenance: record which checkout produced this replay's outputs. Prefer the
    # workdir (where the boards land), else the timings-out dir, else cwd.
    if not args.dry_run:
        ver_dir = args.workdir or (os.path.dirname(args.timings_out) if args.timings_out else ".") or "."
        ver = write_git_version(ver_dir, REPO)
        print(f"Code under test: {format_version(ver)}  (commit {ver.get('commit')}) "
              f"-> {os.path.join(ver_dir, 'git_version.txt')}")

    cmds = parse_manifest(args.manifest)
    if not cmds:
        print(f"No commands found in {args.manifest}")
        return 1

    print(f"Replaying {len(cmds)} recorded command(s) from {args.manifest}")
    if remaps:
        print("Remaps: " + ", ".join(f"{o} -> {n}" for o, n in remaps))

    prune_keep = None
    if not args.verbatim:
        prune_keep, info = compute_prune_keep(cmds)
        dropped = info["dropped"]
        print(f"Pruning to file-dependency chain: running {len(prune_keep)} of "
              f"{len(cmds)} command(s), dropping {len(dropped)} superseded/dead-end "
              f"non-check command(s). Final board: {info['final_board']}")
        warn_contamination(cmds)
        for i in dropped:
            _ins, out = board_io(cmds[i][1])
            tool = next((os.path.basename(a) for a in cmds[i][1] if a.endswith(".py")), "?")
            print(f"    drop [{i+1}] {tool} -> {out}")
    else:
        # Verbatim replays inherit hole files just the same -- the zynq #315
        # "still broken" verdict came from grading a verbatim replay's tail
        # that descended from a recorded artifact. Warn here too.
        warn_contamination(cmds)

    # Seed input-only relative boards (e.g. <board>_input.kicad_pcb) into the dest
    # so the first command finds them -- without this the chain breaks (issue: a
    # relative seed file the original run dir had but no recorded command produces).
    dest_dirs = [args.workdir] if args.workdir else [n for _o, n in remaps]
    for d in dest_dirs:
        seeded = seed_input_boards(args.manifest, cmds, d)
        if seeded:
            print(f"Seeded input board(s) into {d}: {', '.join(seeded)}")

    failures = 0
    timings = []   # per-command (index, seconds, returncode, argv) for later comparison
    t0 = time.time()
    for i, (cwd, argv) in enumerate(cmds, 1):
        if prune_keep is not None and (i - 1) not in prune_keep:
            print(f"[{i}/{len(cmds)}] prune: {' '.join(map(shlex.quote, argv[:3]))} ...")
            continue
        argv = apply_remaps(argv, remaps)
        # --workdir forces every command into one directory (chains relative outputs
        # correctly); otherwise use the command's recorded cwd, remapped.
        if args.workdir:
            cwd = args.workdir
        else:
            remapped = apply_remaps([cwd], remaps)[0] if cwd else None
            if remaps and cwd and remapped == cwd:
                print(f"    WARNING: cwd {cwd} not covered by --remap; this command will "
                      f"run in (and may overwrite) the original run dir. Use --workdir.")
            cwd = remapped
        # SAFETY GUARD -- silent-clobber prevention. When confinement is intended
        # (--workdir or --remap given), a PRODUCING command's OUTPUT board must land
        # inside an intended destination, never the original recorded run dir. If the
        # remap did not cover the output path -- e.g. the manifest was recorded under a
        # different dir than --set/--remap names (a COPIED or MOVED run dir, whose
        # redo_commands.sh still carries the original absolute paths) -- the write would
        # silently OVERWRITE the original run's boards. Refuse loudly instead of
        # clobbering. Producers have >= 2 .kicad_pcb tokens (in -> out); board_image /
        # check_* name a single read-only board and are exempt.
        _pcbs = [a for a in argv if a.endswith(".kicad_pcb")]
        if dest_dirs and len(_pcbs) >= 2 and not is_check_cmd(argv):
            _out = _pcbs[-1]
            _res = os.path.realpath(_out if os.path.isabs(_out)
                                    else os.path.join(cwd or ".", _out))
            _dests = [os.path.realpath(d) for d in dest_dirs]
            if not any(_res == d or _res.startswith(d + os.sep) for d in _dests):
                sys.exit(
                    f"\nABORT (clobber guard): command {i} would write its output board to\n"
                    f"  {_res}\n"
                    f"which is OUTSIDE the intended destination(s):\n"
                    f"  {chr(10).join('  ' + d for d in _dests)}\n"
                    f"The manifest's baked paths were not covered by --remap (a copied or moved\n"
                    f"run dir?). Pass --remap <the-manifest's-own-recorded-dir>:<dest>, or\n"
                    f"--workdir <dest>. Refusing to overwrite the original run.\n")
        if args.skip_checks and is_check_cmd(argv):
            print(f"[{i}/{len(cmds)}] skip check: {' '.join(map(shlex.quote, argv[:3]))} ...")
            continue
        label = " ".join(shlex.quote(a) for a in argv)
        print(f"[{i}/{len(cmds)}] {label}", flush=True)
        if args.dry_run:
            continue
        if cwd and not os.path.isdir(cwd):
            os.makedirs(cwd, exist_ok=True)
        cmd_t0 = time.time()
        timeout = args.timeout if args.timeout and args.timeout > 0 else None
        rc, peak_kb, timed_out, peak_fp_mb = run_with_peak_rss(argv, cwd, timeout=timeout)
        if rc == 0 and not timed_out:
            # A `cp`/`mv` of a board must carry its .kicad_pro (the recorded DRC
            # floor) or the renamed board grades at the looser design default.
            mirror_project_sibling(argv, cwd)
        dt = time.time() - cmd_t0
        rec = {"index": i, "seconds": round(dt, 3), "returncode": rc,
               "peak_rss_mb": round(peak_kb / 1024, 1),
               "timed_out": timed_out, "argv": argv}
        # peak_footprint_mb (darwin only): the authoritative memory number that
        # captures mimalloc-retained + IOAccelerator-tagged pages RSS misses
        # (issue #419). Omitted on platforms where it's unavailable (0.0).
        if peak_fp_mb:
            rec["peak_footprint_mb"] = round(peak_fp_mb, 1)
        timings.append(rec)
        suffix = f"  exit {rc}" if rc != 0 else ""
        if timed_out:
            suffix = f"  TIMED OUT after {timeout:.0f}s (killed)"
        print(f"    -> {dt:.2f}s" + suffix)
        if rc != 0 or timed_out:
            failures += 1
            # check tools return non-zero when they find issues -- not a real error;
            # a timeout is always a real (non-termination) failure.
            real = timed_out or not is_check_cmd(argv)
            if real:
                print("    (timed out)" if timed_out else "    (non-check command failed)")
            if real and not args.continue_on_error:
                print(f"\nStopping at command {i}; rerun with --continue-on-error to push through.")
                return 2

    total = time.time() - t0
    if prune_keep is not None:
        ran = sum(1 for i in range(1, len(cmds) + 1) if (i - 1) in prune_keep)
        print(f"\nReplayed {ran} of {len(cmds)} command(s) (pruned to the dependency "
              f"chain) in {total:.1f}s ({failures} non-zero exits).")
    else:
        print(f"\nReplayed {len(cmds)} command(s) in {total:.1f}s ({failures} non-zero exits).")
    # Per-command timing breakdown (slowest first) -- comparable across code versions.
    if timings:
        print("Per-command time (slowest first):")
        for t in sorted(timings, key=lambda x: -x["seconds"]):
            tool = next((os.path.basename(a) for a in t["argv"] if a.endswith(".py")), "?")
            fp = t.get("peak_footprint_mb")
            fp_col = f" {fp:7.0f}MBfp" if fp is not None else ""
            print(f"  {t['seconds']:7.2f}s  {t.get('peak_rss_mb', 0):7.0f}MBrss{fp_col}  [{t['index']}] {tool}")
    if args.timings_out:
        with open(args.timings_out, "w") as f:
            json.dump({"manifest": args.manifest, "total_seconds": round(total, 3),
                       "commands": timings}, f, indent=2)
        print(f"Wrote per-command timings to {args.timings_out}")

    # Resolve the FINAL board path (used by the PNG snapshot and validation).
    fb_path = None
    try:
        _keep, _info = compute_prune_keep(cmds)
        fb = _info.get("final_board")
    except Exception:
        fb = None
    if fb:
        base = args.workdir or apply_remaps([cmds[-1][0] or "."], remaps)[0]
        fb_path = fb if os.path.isabs(fb) else os.path.join(base or ".", fb)
        fb_path = apply_remaps([fb_path], remaps)[0]
        if not os.path.isfile(fb_path):
            fb_path = None

    # Final-board snapshot (combined + per-layer PNGs, #296/#424) AND the
    # whole-run routing movie (#482), via the fast geometry renderer
    # (render_run -> route_render + animate_route), replacing the old kicad-cli
    # + headless-Chrome board_layer_images path. Drops <board>.png,
    # <board>_<layer>.png, and <rundir>/routing.gif. Non-fatal.
    if fb_path:
        try:
            from render_run import render_final_snapshot, render_run_movie
            render_final_snapshot(fb_path)
            render_run_movie(os.path.dirname(os.path.abspath(fb_path)))
        except Exception as e:
            print(f"board render/movie skipped ({e})")

    # Parser-parity validation of the FINAL board (the headless twin of the
    # GUI's "Validate PCB Data" button): the pcbnew-built PCBData and the text
    # parse must describe the same board, or the CLI and GUI route different
    # worlds. Non-fatal: a FAIL is a parser bug to file, not a replay failure.
    if not args.skip_validate and fb_path:
        print(f"\nParser-parity validation of final board: {fb_path}")
        try:
            vp = subprocess.run(
                [sys.executable, str(REPO / "validate_pcb_data.py"), fb_path],
                capture_output=True, text=True, timeout=900)
            lines = [l for l in vp.stdout.splitlines()
                     if 'assert' not in l and l.strip()]
            for l in lines[:25]:
                print(f"  {l}")
            if len(lines) > 25:
                print(f"  ... ({len(lines) - 25} more line(s))")
            if vp.returncode != 0:
                print("  WARNING: parser-parity validation FAILED -- one of the "
                      "two parsers mis-models this board; file a parser issue. "
                      "(Not counted as a replay failure.)")
        except Exception as e:
            print(f"  validation skipped ({e})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
