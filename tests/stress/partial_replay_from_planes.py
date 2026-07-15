#!/usr/bin/env python3
"""Partial replay: re-run each board's recorded manifest FROM a chosen mid-chain
step (default the first ``route_planes.py``), reusing an EXISTING wave's earlier
boards for everything before that step.

Use this when a code change only affects the later stages of the chain (e.g. a
plane-routing or grading fix, not ``route.py`` / fanout / diff), so the expensive
upstream routing can be skipped and only the affected tail re-run. Pairs with a
prior full wave (from ``ab_replay_grade.py``) that supplies the pre-cutover
boards.

Per board it: finds the first REAL ``--from-script`` step (one that produces a
board -- ``--help``/``--version`` probes are skipped), stages the input board(s)
that command chain reads but that are produced UPSTREAM of the cutover (with
their ``.kicad_pro`` siblings, so the tail steps still see the recorded DRC
floors / edge rule), and hands a partial manifest to ``redo_stress_test.py``
with ``--remap`` into a fresh out dir. A pre-cutover board missing from the seed
wave falls back to the original recorded run dir (retry-branch intermediates)
then the stress source dirs (``boards_setN`` / ``boards_unrouted_setN``).
Grading is left to the caller (run ``ab_replay_grade.py`` /
``kicad_drc_compare.py`` on the finals).

Usage:
  partial_replay_from_planes.py --set <runs_setN> --seed <prior_wave/setN> \
        --out <fresh_wave/setN> [--only b1,b2] [--from-script route_planes.py] [--dry-run]
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _HERE)
import redo_stress_test as rst  # noqa: E402  (sibling module in tests/stress)


def script_of(argv):
    for a in argv:
        if a.endswith(".py"):
            return os.path.basename(a)
    return None


def _board_dirs(run_dir):
    """Fallback dirs for a pre-cutover board the seed wave lacks, in priority
    order: the ORIGINAL recorded run dir (``runs_setN/<board>`` -- holds the real
    intermediates, including retry-branch boards a lean seed replay pruned), then
    the stress source board dirs (``boards_setN`` / ``boards_unrouted_setN`` --
    for a first plane step that reads the original source directly)."""
    dirs = [run_dir]
    set_dir = os.path.dirname(run_dir)               # .../runs_setN
    root = os.path.dirname(set_dir)                   # stress root
    sn = os.path.basename(set_dir)
    if sn.startswith("runs_"):
        tail = sn[len("runs_"):]                      # setN
        dirs += [os.path.join(root, f"boards_{tail}"),
                 os.path.join(root, f"boards_unrouted_{tail}")]
    return dirs


def _locate(b, seed_dir, run_dir):
    """Find pre-cutover board ``b``: prefer the seed wave, else fall back through
    the original run dir and the stress source dirs. Returns (path, origin_label)
    or (None, None)."""
    cand = os.path.join(seed_dir, b)
    if os.path.isfile(cand):
        return cand, "seed"
    for d in _board_dirs(run_dir):
        cand = os.path.join(d, b)
        if os.path.isfile(cand):
            return cand, os.path.basename(d)
    return None, None


def build_partial(manifest, seed_dir, run_dir, from_script):
    """Return (kept_manifest_text, [(src, basename, origin), ...]) or (None, reason).

    Staging is returned for the CALLER to copy straight into the out dir (immune
    to the ``run_dir`` -> out ``--remap``), not emitted as cp lines -- a fallback
    board found under ``run_dir`` would otherwise be remapped in BOTH its source
    and destination and self-copy."""
    cmds = rst.parse_manifest(manifest)              # [(cwd, argv), ...]
    # Cut over at the first REAL from_script step -- one that produces a board.
    # Probe invocations (route_planes.py --help / --version) share the script
    # name but have no board IO; matching those lands the cutover too early and
    # drags in source-reading steps whose inputs the seed wave never produced.
    first = next((i for i, (_, a) in enumerate(cmds)
                  if script_of(a) == from_script and rst.board_io(a)[1]), None)
    if first is None:
        return None, f"no {from_script} step with board output"
    kept = cmds[first:]
    # boards PRODUCED by kept commands (their own outputs) -- do NOT stage these
    produced = set()
    for _, a in kept:
        _inp, out = rst.board_io(a)
        if out:
            produced.add(os.path.basename(out))
    # boards READ by kept commands but produced UPSTREAM of the cutover -> stage
    stage, seen = [], set()
    for _, a in kept:
        inp, _out = rst.board_io(a)
        for i in inp:
            b = os.path.basename(i)
            if b in produced or b in seen or not b.endswith(".kicad_pcb"):
                continue
            seen.add(b)
            src, origin = _locate(b, seed_dir, run_dir)
            if src is None:
                return None, f"{b} not found in seed / run dir / source dirs"
            stage.append((src, b, origin))
    text = "\n".join(f"# cwd={cwd}\n" + " ".join(rst.shlex.quote(x) for x in a)
                     for cwd, a in kept) + "\n"
    return text, stage


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", required=True, help="runs_setN dir (source of the recorded manifests)")
    ap.add_argument("--seed", required=True, help="prior wave's setN dir supplying the pre-cutover boards")
    ap.add_argument("--out", required=True, help="fresh out dir for the partial-replay finals")
    ap.add_argument("--only", default=None, help="comma-separated board subset")
    ap.add_argument("--from-script", default="route_planes.py",
                    help="cut over at the first command running this script (default: route_planes.py)")
    ap.add_argument("--dry-run", action="store_true", help="print the partial manifest, do not run")
    args = ap.parse_args()
    # Absolute paths: redo_stress_test runs every command in --workdir, and the
    # staging copies + fallback source dirs are resolved relative to these.
    args.set = os.path.abspath(args.set)
    args.seed = os.path.abspath(args.seed)
    args.out = os.path.abspath(args.out)
    boards = sorted(d for d in os.listdir(args.set)
                    if os.path.isfile(os.path.join(args.set, d, "redo_commands.sh")))
    if args.only:
        only = set(args.only.split(","))
        boards = [b for b in boards if b in only]
    for b in boards:
        manifest = os.path.join(args.set, b, "redo_commands.sh")
        run_dir = os.path.join(args.set, b)              # original recorded cwd prefix (remapped below)
        seed_dir = os.path.join(args.seed, b)
        dst_dir = os.path.join(args.out, b)
        text, stage = build_partial(manifest, seed_dir, run_dir, args.from_script)
        if text is None:
            print(f"[skip] {b}: {stage}")
            continue
        staged_desc = [f"{bb}[{origin}]" for _src, bb, origin in stage]
        print(f"[{b}] staged={staged_desc}  (from {args.from_script} onward)")
        if args.dry_run:
            print("  --- partial manifest ---\n  " + text.replace("\n", "\n  "))
            continue
        os.makedirs(dst_dir, exist_ok=True)
        # Stage pre-cutover boards straight into the out dir (+ their .kicad_pro
        # siblings, so tail steps see the recorded DRC floors / edge rule). Done
        # here rather than as remapped cp lines so a fallback board sourced from
        # run_dir isn't rewritten in both src and dest by --remap (self-copy).
        for src, bb, _origin in stage:
            shutil.copy2(src, os.path.join(dst_dir, bb))
            pro = src[:-len('.kicad_pcb')] + '.kicad_pro'
            if os.path.isfile(pro):
                shutil.copy2(pro, os.path.join(dst_dir, bb[:-len('.kicad_pcb')] + '.kicad_pro'))
        with tempfile.NamedTemporaryFile("w", suffix=".sh", delete=False, dir=dst_dir) as tf:
            tf.write(text)
            pm = tf.name
        rc = subprocess.run([sys.executable, os.path.join(_HERE, "redo_stress_test.py"),
                             pm, "--remap", f"{run_dir}:{dst_dir}", "--workdir", dst_dir,
                             "--continue-on-error"],
                            capture_output=True, text=True)
        with open(os.path.join(dst_dir, "_partial_replay.log"), "w") as lf:
            lf.write(rc.stdout + rc.stderr)
        tail = "\n".join((rc.stdout + rc.stderr).splitlines()[-3:])
        print(f"  rc={rc.returncode}\n  {tail}")


if __name__ == "__main__":
    main()
