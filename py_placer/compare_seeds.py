#!/usr/bin/env python3
"""Rank place_seed seeds by ONE identical full-board probe each.

Run 7 hand-rolled exactly this: three seeds, each graded against the intent,
then compared -- and the comparison that actually discriminated was a
FULL-BOARD probe route per seed, not the portfolio's shared-window probe and
not crossings (crossings ranked seed 0 first; the probe measured seed 2 at
39 failures vs seed 0's 53 and run 7 shipped on seed 2). The skill's ladder
now calls for identical full-board probes on the seed axis; this CLI is that
step as one command instead of an afternoon of bookkeeping.

Per seed: place_seed subprocess (the seed must GRADE CLEAN against its own
intent -- a seed failing its gate is recorded but never ranked) -> one
full-board probe via converge.scoped_route, every seed routed with the SAME
net patterns and the SAME route args, verdicts via route_verdict (which
counts routed-but-OPEN nets). Output: a ranked table, seeds.json in
--out-dir, and a JSON_SUMMARY line with best_seed.

    python compare_seeds.py board.kicad_pcb --intent floorplan.json \\
        --seeds 0 1 2 --out-dir seedcmp --ignore-nets GND VCC

Exit 0 when at least one seed was probed; 4 when every seed failed its
intent gate (nothing rankable); 2 for usage errors.
"""
import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)
import argparse
import json
import os
import shlex
import subprocess
import sys

# REPO root (this script lives in py_placer/): subprocesses run with cwd=ROOT.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _split_forwarded(tokens):
    """Normalize --seed-args/--route-args values into an argv list.

    argparse's nargs="+" refuses dash-prefixed values, so forwarded flags must
    arrive as ONE quoted string via the = form
    (--seed-args='--clearance 0.15 --max-displacement 2'); split those on
    shell rules. Plain tokens pass through untouched.
    """
    out = []
    for t in tokens or []:
        if any(ch.isspace() for ch in t):
            out.extend(shlex.split(t))
        else:
            out.append(t)
    return out


def build_parser():
    p = argparse.ArgumentParser(
        description="Rank place_seed seeds by one identical full-board "
                    "probe each.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog="""
Examples:
  python compare_seeds.py board.kicad_pcb --intent fp.json --seeds 0 1 2 \\
      --out-dir seedcmp --ignore-nets GND VCC
  python compare_seeds.py board.kicad_pcb --intent fp.json --seeds 0 5 \\
      --out-dir seedcmp --route-args='--clearance 0.15'
""")
    p.add_argument("input_file", help="Unplaced board (outline + parts pile)")
    p.add_argument("--intent", required=True,
                   help="Floorplan intent, forwarded to place_seed")
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2],
                   help="Seed values to generate and rank (default: 0 1 2)")
    p.add_argument("--out-dir", required=True,
                   help="Directory for seed boards + seeds.json")
    p.add_argument("--ignore-nets", nargs="+", default=None,
                   help="Net patterns excluded from the probe (plane-routed "
                        "rails) and forwarded to place_seed's polish")
    p.add_argument("--seed-args", nargs="+", default=None,
                   help="Extra args forwarded to every place_seed call. "
                        "Dash-prefixed flags must ride in ONE quoted = value: "
                        "--seed-args='--clearance 0.15 --max-displacement 2'")
    p.add_argument("--route-args", nargs="+", default=None,
                   help="Extra args for every probe route -- identical "
                        "across seeds by construction. Same quoting rule as "
                        "--seed-args: --route-args='--clearance 0.15'")
    # NO --route-timeout (#713 item 2). It defaulted to 1800 here and 900 in
    # place_portfolio -- two clocks, two defaults, neither covered by a test or
    # named in any doc -- and a timed-out seed became `failures: None`, which
    # `_key` sorts into tier 2 and `best` then never selects. A comparison one
    # of whose arms a clock erased is not one. The bound is SCOPE: the net
    # patterns already passed below.
    p.add_argument("--skip-gated", action="store_true", default=True,
                   help=argparse.SUPPRESS)
    p.add_argument("--probe-gated", action="store_true",
                   help="Probe seeds that FAILED their intent gate too "
                        "(recorded but still never ranked above a clean "
                        "seed)")
    return p


def _run_place_seed(args, seed, out_board):
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_placer', 'place_seed.py'), args.input_file, out_board,
            '--intent', args.intent, '--seed', str(seed)]
    if args.ignore_nets:
        argv += ['--ignore-nets'] + list(args.ignore_nets)
    if args.seed_args:
        argv += list(args.seed_args)
    r = subprocess.run(argv, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', cwd=ROOT)
    summary = {}
    for line in r.stdout.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            try:
                summary = json.loads(line.split(':', 1)[1])
            except ValueError:
                pass
    return r, summary


def main():
    parser = build_parser()
    args = parser.parse_args()
    # Subprocesses run with cwd=ROOT: a caller-relative path silently breaks
    # every one of them (the place_portfolio lesson, 8bd1344). Absolutize at
    # the door.
    args.input_file = os.path.abspath(args.input_file)
    args.intent = os.path.abspath(args.intent)
    args.out_dir = os.path.abspath(args.out_dir)
    args.seed_args = _split_forwarded(args.seed_args)
    args.route_args = _split_forwarded(args.route_args)
    if len(set(args.seeds)) != len(args.seeds):
        parser.error("--seeds has duplicates")
    os.makedirs(args.out_dir, exist_ok=True)
    try:
        from redo_record import record_invocation
        record_invocation()
    except Exception:
        pass

    from converge import route_verdict, scoped_route

    full_nets = ['*'] + [f'!{p}' for p in (args.ignore_nets or [])]
    rows = []
    for seed in args.seeds:
        out_board = os.path.join(args.out_dir, f'seed_{seed}.kicad_pcb')
        print(f"[seed {seed}] place_seed -> {os.path.basename(out_board)}")
        r, s = _run_place_seed(args, seed, out_board)
        row = {'seed': seed, 'board': out_board,
               'place_seed_rc': r.returncode,
               'grade_errors': s.get('grade_errors'),
               'unseated': s.get('unseated'),
               'crossings': s.get('crossings'), 'hpwl': s.get('hpwl'),
               'probe': None}
        if r.returncode not in (0, 4):
            row['note'] = ('place_seed failed: '
                           + (r.stderr or r.stdout)[-300:].strip())
            print(f"[seed {seed}] FAILED (rc={r.returncode})")
            rows.append(row)
            continue
        row['gated'] = (r.returncode == 4)
        if row['gated']:
            print(f"[seed {seed}] fails its intent gate "
                  f"({s.get('grade_errors')} error(s), "
                  f"{s.get('unseated')} unseated)"
                  + ("" if args.probe_gated else " -- not probed"))
            if not args.probe_gated:
                rows.append(row)
                continue
        print(f"[seed {seed}] probing full board "
              f"({len(full_nets)} pattern(s))")
        from converge import probe_route
        row['probe'] = probe_route(out_board, full_nets,
                                   extra_args=args.route_args or [])
        row['probe']['probe_kind'] = 'full'
        if row['probe']['failures'] is None:
            # Previously silent: the only print sat on the success path, so a
            # seed that produced no verdict vanished from the console AND from
            # the ranking.
            print(f"[seed {seed}] NO VERDICT ({row['probe']['status']}: "
                  f"{row['probe']['note']})")
        else:
            print(f"[seed {seed}] failures={row['probe']['failures']} "
                  f"({row['probe']['note']})")
        rows.append(row)

    # Rank: clean-gated probed seeds first by (failures, iterations, seed);
    # probed-but-gated after them; everything else last, unranked.
    def _key(row):
        pr = row.get('probe') or {}
        f = pr.get('failures')
        return (0 if (f is not None and not row.get('gated')) else
                1 if f is not None else 2,
                f if f is not None else 1 << 30,
                pr.get('iterations') or 0,
                row['seed'])

    ranked = sorted(rows, key=_key)
    print("\nseed  gate   failures  iterations  crossings  hpwl")
    for row in ranked:
        pr = row.get('probe') or {}
        gate = ('FAIL' if row.get('gated') else
                'ok' if row.get('place_seed_rc') == 0 else 'ERR')
        print(f"{row['seed']:>4}  {gate:<5}  "
              f"{str(pr.get('failures', '-')):>8}  "
              f"{str(pr.get('iterations', '-')):>10}  "
              f"{str(row.get('crossings', '-')):>9}  "
              f"{str(row.get('hpwl', '-')):>8}")

    best = next((r for r in ranked
                 if (r.get('probe') or {}).get('failures') is not None
                 and not r.get('gated')), None)
    doc = {'input': args.input_file, 'intent': args.intent,
           'seeds': args.seeds, 'probe_nets': full_nets,
           'route_args': args.route_args or [], 'rows': rows,
           'ranking': [r['seed'] for r in ranked],
           'best_seed': best['seed'] if best else None}
    doc_path = os.path.join(args.out_dir, 'seeds.json')
    with open(doc_path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1, sort_keys=True)
    print(f"Wrote {doc_path}")
    print("JSON_SUMMARY: " + json.dumps({
        'seeds': len(args.seeds), 'probed': sum(1 for r in rows if r['probe']),
        'gated': sum(1 for r in rows if r.get('gated')),
        'best_seed': best['seed'] if best else None,
        'best_failures': ((best.get('probe') or {}).get('failures')
                          if best else None),
        'out_dir': args.out_dir}, sort_keys=True))
    if best is not None:
        return 0
    if rows and all(r.get('gated') for r in rows):
        print("compare_seeds: every seed failed its intent gate -- nothing "
              "rankable. Fix the intent or the pile first.", file=sys.stderr)
        return 4
    # `no winner` is `nothing rankable`, full stop (#713 item 2). The old form
    # was `4 if not any(r['probe'])`, and a no-verdict row is a truthy dict --
    # so a run where every probe failed returned 0 with `best_seed: null`,
    # contradicting docs/utilities.md's own contract ("exit 0 with a ranked
    # winner, 4 when nothing was rankable"). A caller branching on the exit
    # code read a total failure as a success.
    _no_verdict = [r for r in rows
                   if (r.get('probe') or {}).get('failures') is None]
    if _no_verdict:
        print(f"compare_seeds: {len(_no_verdict)} of {len(rows)} seed(s) "
              f"produced NO VERDICT and none was rankable -- "
              + '; '.join(f"seed {r['seed']}: "
                          f"{(r.get('probe') or {}).get('status', 'not probed')}"
                          for r in _no_verdict[:6]), file=sys.stderr)
    return 4


if __name__ == "__main__":
    sys.exit(main())
