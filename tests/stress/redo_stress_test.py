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


def run_with_peak_rss(argv, cwd, interval=0.5, timeout=None):
    """Run a command (inheriting stdout/stderr) while sampling its process-tree
    RSS; return (returncode, peak_rss_kb, timed_out). Lets the replay record
    per-step peak memory so an engine change's memory effect is comparable, not
    just its time.

    If timeout (seconds) is given and the command runs past it, kill the process
    tree and return timed_out=True. The original LLM run has a per-command cap
    (run_board.sh's 3-hour/command budget); the replay had none, so a board that
    wedges in rip-up (issue #211: ulx3s' 40-min+ non-terminating final route)
    could hang a whole sweep forever. A generous cap (default 3h) lets slow-but-
    finishing boards complete while still bailing a true non-termination."""
    p = subprocess.Popen(argv, cwd=cwd)
    peak = 0
    start = time.time()
    timed_out = False
    while p.poll() is None:
        peak = max(peak, _tree_rss_kb(p.pid))
        if timeout is not None and time.time() - start > timeout:
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
    return p.returncode, peak, timed_out


def parse_manifest(path):
    """Yield (cwd, argv) for each recorded command. cwd is None if not recorded."""
    cmds = []
    pending_cwd = None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("# cwd="):
                # The cwd was recorded with %q quoting on the value after 'cwd='.
                pending_cwd = " ".join(shlex.split(line[len("# cwd="):]))
                continue
            if not line or line.startswith("#") or line in ("set -e",):
                continue
            if line.startswith("#!"):
                continue
            argv = shlex.split(line)
            if argv:
                cmds.append((pending_cwd, argv))
            pending_cwd = None
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


def tool_of(argv):
    """Basename of the .py script a command runs ('' if none)."""
    return next((os.path.basename(a) for a in argv if a.endswith(".py")), "")


def cap_opt_injection_index(cmds, keep):
    """Index of the command after which to auto-run place_fanout_clearance.

    Cap-opt is part of the intended pipeline after a BGA fanout, but chains
    recorded before it existed (or where the agent skipped it) leave caps
    grazing/shorting fanout vias all the way to the final board (#267:
    orangecrab's 16 PAD-VIA). Returns the index of the LAST kept bga_fanout
    command -- mirroring the agent-recorded pattern (cap-opt once, after the
    BGA fanout phase, before qfn fanouts) -- or None when the kept chain
    already runs place_fanout_clearance anywhere (trust the recorded
    placement), has no bga_fanout, or the bga_fanout writes no board
    (--help). qfn-only chains get no injection: cap-opt keys on BGAs and
    would no-op."""
    kept = [k for k in range(len(cmds)) if keep is None or k in keep]
    if any(tool_of(cmds[k][1]) == "place_fanout_clearance.py" for k in kept):
        return None
    bgas = [k for k in kept
            if tool_of(cmds[k][1]) == "bga_fanout.py" and board_io(cmds[k][1])[1]]
    return bgas[-1] if bgas else None


def build_cap_opt_argv(fanout_argv):
    """place_fanout_clearance argv for a just-run fanout command: same python
    prefix, IN-PLACE on the fanout's output (downstream commands read that
    filename, and cap-opt writes nothing when no cap moves -- a distinct
    output path would then not exist), reusing the fanout's --clearance when
    it had one."""
    script_i = next(j for j, a in enumerate(fanout_argv) if a.endswith(".py"))
    out = [a for a in fanout_argv if a.endswith(".kicad_pcb")][-1]
    argv = list(fanout_argv[:script_i]) + [
        str(REPO / "place_fanout_clearance.py"), out, out]
    for j, a in enumerate(fanout_argv):
        if a == "--clearance" and j + 1 < len(fanout_argv):
            argv += ["--clearance", fanout_argv[j + 1]]
            break
    return argv


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
    return keep, {"final_index": final, "final_board": out_of[final], "dropped": dropped}


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
    ap.add_argument("--no-auto-cap-opt", action="store_true",
                    help="Do not auto-insert a place_fanout_clearance step after the "
                         "last bga_fanout when the manifest lacks one (#267). By "
                         "default the replay injects it (in-place on that fanout's "
                         "output, at the fanout's --clearance) so chains recorded "
                         "without cap-opt still get caps moved off fanout vias; pass "
                         "this for a literal replay of the recorded commands.")
    ap.add_argument("--dry-run", action="store_true", help="Print the plan, run nothing")
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
        for i in dropped:
            _ins, out = board_io(cmds[i][1])
            tool = next((os.path.basename(a) for a in cmds[i][1] if a.endswith(".py")), "?")
            print(f"    drop [{i+1}] {tool} -> {out}")

    inject_after = None
    if not args.no_auto_cap_opt:
        inject_after = cap_opt_injection_index(cmds, prune_keep)
        if inject_after is not None:
            print(f"Auto cap-opt: will run place_fanout_clearance after command "
                  f"[{inject_after + 1}] (chain has bga_fanout but no recorded "
                  f"cap-opt, #267; --no-auto-cap-opt disables)")

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
        if args.skip_checks and is_check_cmd(argv):
            print(f"[{i}/{len(cmds)}] skip check: {' '.join(map(shlex.quote, argv[:3]))} ...")
            continue
        label = " ".join(shlex.quote(a) for a in argv)
        print(f"[{i}/{len(cmds)}] {label}", flush=True)
        if args.dry_run:
            if (i - 1) == inject_after:
                cargv = build_cap_opt_argv(argv)
                print(f"[{i}b/{len(cmds)}] auto cap-opt: "
                      f"{' '.join(map(shlex.quote, cargv))}")
            continue
        if cwd and not os.path.isdir(cwd):
            os.makedirs(cwd, exist_ok=True)
        cmd_t0 = time.time()
        timeout = args.timeout if args.timeout and args.timeout > 0 else None
        rc, peak_kb, timed_out = run_with_peak_rss(argv, cwd, timeout=timeout)
        dt = time.time() - cmd_t0
        timings.append({"index": i, "seconds": round(dt, 3), "returncode": rc,
                        "peak_rss_mb": round(peak_kb / 1024, 1),
                        "timed_out": timed_out, "argv": argv})
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

        # Auto cap-opt after the last bga_fanout (see cap_opt_injection_index).
        # Runs only when the fanout itself succeeded; in-place, so the chain's
        # file names are untouched and the dependency DAG is unaffected.
        if (i - 1) == inject_after and rc == 0 and not timed_out:
            cargv = build_cap_opt_argv(argv)
            print(f"[{i}b/{len(cmds)}] auto cap-opt: "
                  f"{' '.join(map(shlex.quote, cargv))}", flush=True)
            cmd_t0 = time.time()
            rc2, peak_kb2, to2 = run_with_peak_rss(cargv, cwd, timeout=timeout)
            dt2 = time.time() - cmd_t0
            timings.append({"index": i, "injected": "cap_opt",
                            "seconds": round(dt2, 3), "returncode": rc2,
                            "peak_rss_mb": round(peak_kb2 / 1024, 1),
                            "timed_out": to2, "argv": cargv})
            suffix2 = f"  exit {rc2}" if rc2 != 0 else ""
            if to2:
                suffix2 = f"  TIMED OUT after {timeout:.0f}s (killed)"
            print(f"    -> {dt2:.2f}s" + suffix2)
            if rc2 != 0 or to2:
                failures += 1
                print("    (auto cap-opt failed)")
                if not args.continue_on_error:
                    print(f"\nStopping at auto cap-opt after command {i}; rerun "
                          f"with --continue-on-error to push through.")
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
            print(f"  {t['seconds']:7.2f}s  {t.get('peak_rss_mb', 0):7.0f}MB  [{t['index']}] {tool}")
    if args.timings_out:
        with open(args.timings_out, "w") as f:
            json.dump({"manifest": args.manifest, "total_seconds": round(total, 3),
                       "commands": timings}, f, indent=2)
        print(f"Wrote per-command timings to {args.timings_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
