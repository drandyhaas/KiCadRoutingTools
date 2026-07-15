#!/usr/bin/env python3
"""Partial replay: re-run each board's recorded manifest FROM a chosen mid-chain
step (default the first ``route_planes.py``), reusing an EXISTING wave's earlier
boards for everything before that step.

Use this when a code change only affects the later stages of the chain (e.g. a
plane-routing or grading fix, not ``route.py`` / fanout / diff), so the expensive
upstream routing can be skipped and only the affected tail re-run. Pairs with a
prior full wave (from ``ab_replay_grade.py``) that supplies the pre-cutover
boards.

Per board it: finds the first command whose script is ``--from-script``, stages
(``cp``) the input board(s) that command chain reads but that are produced
UPSTREAM of the cutover (with their ``.kicad_pro`` siblings, so the tail steps
still see the recorded DRC floors / edge rule), and hands a partial manifest to
``redo_stress_test.py`` with ``--remap`` into a fresh out dir. Grading is left to
the caller (run ``ab_replay_grade.py`` / ``kicad_drc_compare.py`` on the finals).

Usage:
  partial_replay_from_planes.py --set <runs_setN> --seed <prior_wave/setN> \
        --out <fresh_wave/setN> [--only b1,b2] [--from-script route_planes.py] [--dry-run]
"""
import argparse
import os
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


def build_partial(manifest, seed_dir, run_dir, from_script):
    """Return (partial_manifest_text, staged_boards) or (None, reason)."""
    cmds = rst.parse_manifest(manifest)              # [(cwd, argv), ...]
    first = next((i for i, (_, a) in enumerate(cmds) if script_of(a) == from_script), None)
    if first is None:
        return None, f"no {from_script} step"
    kept = cmds[first:]
    # boards PRODUCED by kept commands (their own outputs) -- do NOT stage these
    produced = set()
    for _, a in kept:
        _inp, out = rst.board_io(a)
        if out:
            produced.add(os.path.basename(out))
    # boards READ by kept commands but produced UPSTREAM of the cutover -> stage from seed
    need = []
    for _, a in kept:
        inp, _out = rst.board_io(a)
        for i in inp:
            b = os.path.basename(i)
            if b not in produced and b not in need and b.endswith(".kicad_pcb"):
                need.append(b)
    lines, staged = [], []
    for b in need:
        src_pcb = os.path.join(seed_dir, b)
        if not os.path.isfile(src_pcb):
            return None, f"seed missing {b}"
        # stage board + its .kicad_pro sibling (tail steps read the DRC floors / edge rule from it)
        lines += [f"# cwd={run_dir}", f"cp {src_pcb} {os.path.join(run_dir, b)}"]
        pro = src_pcb[:-len('.kicad_pcb')] + '.kicad_pro'
        if os.path.isfile(pro):
            lines += [f"# cwd={run_dir}",
                      f"cp {pro} {os.path.join(run_dir, b[:-len('.kicad_pcb')] + '.kicad_pro')}"]
        staged.append(b)
    for cwd, a in kept:
        lines += [f"# cwd={cwd}", " ".join(rst.shlex.quote(x) for x in a)]
    return "\n".join(lines) + "\n", staged


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
    # Absolute paths: redo_stress_test runs every command in --workdir, so any
    # relative path in the staged cp commands would resolve against the workdir.
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
        text, info = build_partial(manifest, seed_dir, run_dir, args.from_script)
        if text is None:
            print(f"[skip] {b}: {info}")
            continue
        print(f"[{b}] staged={info}  (from {args.from_script} onward)")
        if args.dry_run:
            print("  --- partial manifest ---\n  " + text.replace("\n", "\n  "))
            continue
        os.makedirs(dst_dir, exist_ok=True)
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
