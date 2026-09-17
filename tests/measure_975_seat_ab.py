#!/usr/bin/env python3
"""#975: the edge-seat floor preference, commit against commit, on real boards.

    # one arm per engine tree (each a clean checkout, grid_router built in):
    python3 -B -X utf8 tests/measure_975_seat_ab.py --repo <base-tree>   --out base.json
    python3 -B -X utf8 tests/measure_975_seat_ab.py --repo <branch-tree> --out branch.json
    python3 -B -X utf8 tests/measure_975_seat_ab.py --diff base.json branch.json

    # the run-27 esp_prog 10-seed pass rate, per tree (needs grid_router):
    python3 -B -X utf8 tests/measure_975_seat_ab.py --repo <tree> --seeds-out seeds.json
    python3 -B -X utf8 tests/measure_975_seat_ab.py --diff-seeds base_seeds.json branch_seeds.json

    # test_706's wall clock, interleaved over trees (idle machine):
    python3 -B -X utf8 tests/measure_975_seat_ab.py --time706 <main-tree> <base-tree> <branch-tree>

NOT named `test_*`, so `run_all.py` never collects it: it needs two checkouts
and minutes of place_seed per board.

INPUTS come from THIS file's repo and are the same bytes for every arm, with
their sha256 recorded: the emitted intent of each board (`floorplan.
emit_intent`), a copper-stripped copy where a board carries copper
(`tests/stress/strip_copper_only.py`), and run 27's esp_prog input rebuilt from
`tests/fixtures/975/esp_prog_run27/` (the emitted esp_prog intent declares no
edge connector, so an A/B on it would measure nothing). Only the ENGINE comes
from `--repo`: `place_seed.py` and `compare_seeds.py` run there, with cwd there
and PYTHONHASHSEED=0. The GRADERS come from this repo for both arms
(`check_assembly.py` for `blocking`, the parser for poses), so both sides are
graded on the same terms.

Per board, two arms: `--no-polish`, and the default polish. Each emitted board
runs `--force --seed 0 --group-by kicad,sheet --clearance 0.2
--board-edge-clearance 0.55`; the run-27 fixture runs with neither clearance
flag, so it resolves 0.15 / 0.3 from its project as run 27 did.

THE RULE, written before any arm ran:

  GUARD, every board x arm: the branch is never worse than the base on
    `blocking` (check_assembly), the number of `unseated_refs`, rc (0 -> not
    0), `grade_errors`, `pad_conflicts_seeded`, or `hole_conflicts_added`.
    Any one of these is a REGRESSION, whatever the signal says.
  SIGNAL: `pad_edge_after` findings on the intent's declared edge refs are
    <= the base on every row, and < on at least one board.
  CONSISTENCY, branch rows: every `edge_floor_fallback` ref has a finding in
    `pad_edge_after`, and every declared edge ref that the run SEATED with a
    finding has a record. SEATED means the run moved it: not in
    `unseated_refs` AND at a pose other than its input pose. (Corrected after
    the first run, which read "not unseated" as seated and so flagged
    rp2350's U8 -- `(locked yes)` in the input, never seated by anything.)
  Rows where nothing moved are printed as NULL rows, never dropped.

MEASURED: see the #975 PR, which carries the table this prints.
"""
import argparse
import hashlib
import json
import os
import shutil
import statistics
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (HERE, os.path.join(HERE, 'py_router'), os.path.join(HERE, 'py_placer'),
           os.path.join(HERE, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

BOARDS = ('tigard', 'splitflap_driver', 'ulx3s', 'rp2350_fpga_eensy_prePlane', 'esp_prog_run27')
FIXTURE = os.path.join(HERE, 'tests', 'fixtures', '975', 'esp_prog_run27')
SEEDS = list(range(10))
GUARDS = ('blocking', 'unseated', 'rc_fail', 'grade_errors', 'pad_conflicts_seeded',
          'hole_conflicts_added')


def sha(path):
    with open(path, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def git(root, *args):
    return subprocess.run(['git', '-C', root, *args], capture_output=True, text=True,
                          check=True).stdout.strip()


def env():
    return dict(os.environ, PYTHONHASHSEED='0', PYTHONIOENCODING='utf-8', KRT_NO_BANNER='1')


def summary_of(stdout):
    found = None
    for line in (stdout or '').splitlines():
        if line.startswith('JSON_SUMMARY: '):
            found = json.loads(line[len('JSON_SUMMARY: '):])
    return found


def prepare(tmp):
    """{board: (board_path, intent_path, flags)} built from THIS repo's bytes."""
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    from placement.placement_state import assess_placement
    from placement.writer import write_placed_output
    out = {}
    for name in BOARDS:
        d = os.path.join(tmp, name)
        os.makedirs(d)
        board = os.path.join(d, 'board.kicad_pcb')
        if name == 'esp_prog_run27':
            with open(os.path.join(FIXTURE, 'pile.json'), encoding='utf-8') as stream:
                pile = json.load(stream)
            x, y, rot = pile['pose']
            write_placed_output(os.path.join(HERE, pile['source']), board,
                                [{'reference': r, 'new_x': x, 'new_y': y, 'new_rotation': rot}
                                 for r in pile['refs']])
            shutil.copyfile(os.path.join(FIXTURE, 'board.kicad_pro'),
                            os.path.join(d, 'board.kicad_pro'))
            intent = os.path.join(d, 'intent.json')
            shutil.copyfile(os.path.join(FIXTURE, 'zone_plan.json'), intent)
            out[name] = (board, intent, [])
            continue
        source = os.path.join(HERE, 'kicad_files', name + '.kicad_pcb')
        shutil.copyfile(source, board)
        for ext in ('.kicad_pro', '.kicad_dru'):
            sibling = os.path.splitext(source)[0] + ext
            if os.path.exists(sibling):
                shutil.copyfile(sibling, os.path.splitext(board)[0] + ext)
        if assess_placement(parse_kicad_pcb(board)).has_copper:
            stripped = os.path.join(d, 'stripped.kicad_pcb')
            subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(HERE, 'tests', 'stress', 'strip_copper_only.py'),
                            board, stripped], check=True, capture_output=True, env=env())
            os.replace(stripped, board)
        doc = floorplan.emit_intent(parse_kicad_pcb(board), board)
        intent = os.path.join(d, 'intent.json')
        with open(intent, 'w', encoding='utf-8') as stream:
            json.dump(doc, stream, indent=1, sort_keys=True)
        out[name] = (board, intent, ['--group-by', 'kicad,sheet', '--clearance', '0.2',
                                     '--board-edge-clearance', '0.55'])
    return out


def edge_refs(intent_path):
    from placement import floorplan
    return sorted(c['ref'] for c in floorplan.load_intent(intent_path).edge_claims())


def poses(board, refs):
    from kicad_parser import parse_kicad_pcb
    fps = parse_kicad_pcb(board).footprints
    return {r: [round(fps[r].x, 4), round(fps[r].y, 4), fps[r].rotation]
            for r in refs if r in fps}


def run_arm(repo, board, intent, flags, polish, work):
    out = os.path.join(work, 'seed.kicad_pcb')
    argv = [sys.executable, '-X', 'utf8', os.path.join(repo, 'py_placer', 'place_seed.py'),
            board, out, '--intent', intent, '--force', '--seed', '0', *flags]
    if not polish:
        argv.append('--no-polish')
    t0 = time.perf_counter()
    proc = subprocess.run(argv, capture_output=True, text=True, cwd=repo, env=env())
    seconds = round(time.perf_counter() - t0, 1)
    summary = summary_of(proc.stdout)
    row = {'rc': proc.returncode, 'seconds': seconds}
    if summary is None:
        row['error'] = (proc.stdout[-1500:] + proc.stderr[-1500:])
        return row
    refs = edge_refs(intent)
    after = summary.get('pad_edge_after') or {}
    edge_findings = [f for f in after.get('findings') or []
                     if f['pad_ref'].split('.')[0] in refs]
    other = [f for f in after.get('findings') or [] if f not in edge_findings]
    report = os.path.join(work, 'assembly.json')
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join(HERE, 'py_tools', 'check_assembly.py'),
                    out, '--json', report], capture_output=True, text=True, env=env())
    assembly = {}
    if os.path.exists(report):
        with open(report, encoding='utf-8') as stream:
            assembly = json.load(stream)
    row.update({
        'unseated_refs': summary.get('unseated_refs'),
        'rotation_unseated': summary.get('rotation_unseated'),
        'grade_errors': summary.get('grade_errors'),
        'grade_errors_pinned': summary.get('grade_errors_pinned'),
        'pad_conflicts_seeded': summary.get('pad_conflicts_seeded'),
        'hole_conflicts_added': summary.get('hole_conflicts_added'),
        'pad_edge_before_findings': len((summary.get('pad_edge_before') or {}).get('findings') or []),
        'edge_findings': sorted((f['pad_ref'], f['pad_index'], f['shortfall_mm'])
                                for f in edge_findings),
        'edge_findings_n': len(edge_findings),
        'edge_shortfall_sum': round(sum(f['shortfall_mm'] for f in edge_findings), 6),
        'edge_shortfall_max': round(max((f['shortfall_mm'] for f in edge_findings), default=0.0), 6),
        'other_findings_n': len(other),
        'edge_floor_fallback': {r: {k: rec.get(k) for k in ('why', 'kept', 'shortfall_mm')}
                                for r, rec in (summary.get('edge_floor_fallback') or {}).items()},
        'has_edge_floor_key': 'edge_floor_fallback' in summary,
        'blocking': assembly.get('blocking'),
        'buildable': assembly.get('buildable'),
        'edge_poses': poses(out, refs),
        'output_sha256': sha(out),
    })
    return row


def collect(repo, out_path):
    repo = os.path.abspath(repo)
    if git(repo, 'status', '--porcelain', '--untracked-files=no'):
        sys.exit(f'REFUSED: {repo} has uncommitted changes')
    tmp = tempfile.mkdtemp(prefix='ab975_')
    try:
        inputs = prepare(tmp)
        doc = {'repo': repo, 'engine_sha': git(repo, 'rev-parse', 'HEAD'),
               'inputs_sha': git(HERE, 'rev-parse', 'HEAD'), 'python': sys.version.split()[0],
               'boards': {}}
        for name, (board, intent, flags) in inputs.items():
            doc['boards'][name] = {'board_sha256': sha(board), 'intent_sha256': sha(intent),
                                   'edge_refs': edge_refs(intent),
                                   'input_poses': poses(board, edge_refs(intent)), 'arms': {}}
            for polish in (False, True):
                arm = 'polish' if polish else 'no-polish'
                work = os.path.join(tmp, name, arm)
                os.makedirs(work)
                row = run_arm(repo, board, intent, flags, polish, work)
                doc['boards'][name]['arms'][arm] = row
                print(f'{name:28s} {arm:9s} rc {row["rc"]} edge findings '
                      f'{row.get("edge_findings_n")} blocking {row.get("blocking")} '
                      f'records {sorted(row.get("edge_floor_fallback") or {})} '
                      f'{row["seconds"]}s', flush=True)
        with open(out_path, 'w', encoding='utf-8') as stream:
            json.dump(doc, stream, indent=1, sort_keys=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _guard_values(row):
    return {'blocking': row.get('blocking'), 'unseated': len(row.get('unseated_refs') or []),
            'rc_fail': int(row.get('rc') != 0), 'grade_errors': row.get('grade_errors'),
            'pad_conflicts_seeded': row.get('pad_conflicts_seeded'),
            'hole_conflicts_added': row.get('hole_conflicts_added')}


def diff(a_path, b_path):
    with open(a_path, encoding='utf-8') as stream:
        base = json.load(stream)
    with open(b_path, encoding='utf-8') as stream:
        branch = json.load(stream)
    print(f'base {base["engine_sha"][:10]}  branch {branch["engine_sha"][:10]}')
    regressions, improved_boards, null_rows, inconsistent = [], set(), [], []
    print(f'{"board":28s} {"arm":9s} {"edge findings":>14s} {"sum mm":>15s} '
          f'{"blocking":>9s} {"unseated":>9s} {"rc":>5s} {"grade":>7s}  records / moved')
    for name in sorted(set(base['boards']) | set(branch['boards'])):
        b0, b1 = base['boards'].get(name), branch['boards'].get(name)
        if not b0 or not b1:
            print(f'{name}: missing on one side')
            regressions.append((name, 'missing'))
            continue
        if (b0['board_sha256'], b0['intent_sha256']) != (b1['board_sha256'], b1['intent_sha256']):
            print(f'{name}: INPUTS DIFFER between arms -- not comparable')
            regressions.append((name, 'inputs'))
            continue
        for arm in ('no-polish', 'polish'):
            r0, r1 = b0['arms'][arm], b1['arms'][arm]
            if 'error' in r0 or 'error' in r1:
                print(f'{name:28s} {arm:9s} ERROR base={"error" in r0} branch={"error" in r1}')
                regressions.append((name, arm, 'error'))
                continue
            g0, g1 = _guard_values(r0), _guard_values(r1)
            for key in GUARDS:
                if g0[key] is None or g1[key] is None:
                    continue
                if g1[key] > g0[key]:
                    regressions.append((name, arm, key, g0[key], g1[key]))
            if r1['edge_findings_n'] > r0['edge_findings_n']:
                regressions.append((name, arm, 'edge_findings', r0['edge_findings_n'],
                                    r1['edge_findings_n']))
            elif r1['edge_findings_n'] < r0['edge_findings_n']:
                improved_boards.add(name)
            moved = sorted(r for r in r1['edge_poses'] if r1['edge_poses'][r] != r0['edge_poses'].get(r))
            if r0['output_sha256'] == r1['output_sha256'] or (
                    not moved and r0['edge_findings'] == r1['edge_findings'] and g0 == g1):
                null_rows.append((name, arm))
            findings_refs = {p.split('.')[0] for p, _, _ in r1['edge_findings']}
            seated = {r for r in set(b1['edge_refs']) - set(r1['unseated_refs'] or [])
                      if r1['edge_poses'].get(r) != b1.get('input_poses', {}).get(r)}
            for ref in r1['edge_floor_fallback']:
                if ref not in findings_refs:
                    inconsistent.append((name, arm, ref, 'record without a finding'))
            for ref in (findings_refs & seated) - set(r1['edge_floor_fallback']):
                inconsistent.append((name, arm, ref, 'finding without a record'))
            print(f'{name:28s} {arm:9s} {r0["edge_findings_n"]:>6d} -> {r1["edge_findings_n"]:<5d} '
                  f'{r0["edge_shortfall_sum"]:>6.3f} -> {r1["edge_shortfall_sum"]:<6.3f} '
                  f'{str(g0["blocking"]):>3s} -> {str(g1["blocking"]):<3s} '
                  f'{g0["unseated"]:>3d} -> {g1["unseated"]:<3d} {g0["rc_fail"]}->{g1["rc_fail"]} '
                  f'{str(g0["grade_errors"]):>2s}->{str(g1["grade_errors"]):<2s}  '
                  f'{ {r: v["why"] for r, v in r1["edge_floor_fallback"].items()} } {moved}')
    print()
    print(f'NULL rows (nothing moved): {null_rows}')
    print(f'boards improved on the signal: {sorted(improved_boards)}')
    print(f'consistency problems: {inconsistent}')
    print(f'REGRESSIONS: {regressions}')
    ok = not regressions and len(improved_boards) >= 1 and not inconsistent
    print('VERDICT:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


def seeds(repo, out_path):
    repo = os.path.abspath(repo)
    if git(repo, 'status', '--porcelain', '--untracked-files=no'):
        sys.exit(f'REFUSED: {repo} has uncommitted changes')
    tmp = tempfile.mkdtemp(prefix='seeds975_')
    try:
        from placement.writer import write_placed_output
        d = os.path.join(tmp, 'esp_prog')
        os.makedirs(d)
        with open(os.path.join(FIXTURE, 'pile.json'), encoding='utf-8') as stream:
            pile = json.load(stream)
        board = os.path.join(d, 'board.kicad_pcb')
        x, y, rot = pile['pose']
        write_placed_output(os.path.join(HERE, pile['source']), board,
                            [{'reference': r, 'new_x': x, 'new_y': y, 'new_rotation': rot}
                             for r in pile['refs']])
        shutil.copyfile(os.path.join(FIXTURE, 'board.kicad_pro'), os.path.join(d, 'board.kicad_pro'))
        intent = os.path.join(d, 'zone_plan.json')
        shutil.copyfile(os.path.join(FIXTURE, 'zone_plan.json'), intent)
        outdir = os.path.join(d, 'seedcmp')
        t0 = time.perf_counter()
        proc = subprocess.run([sys.executable, '-X', 'utf8',
                               os.path.join(repo, 'py_placer', 'compare_seeds.py'), board,
                               '--intent', intent, '--seeds', *map(str, SEEDS),
                               '--out-dir', outdir, '--ignore-nets', 'GND'],
                              capture_output=True, text=True, cwd=repo, env=env())
        with open(os.path.join(outdir, 'seeds.json'), encoding='utf-8') as stream:
            result = json.load(stream)
        rows = {}
        for row in result['rows']:
            probe = row.get('probe') or {}
            rows[str(row['seed'])] = {
                'gated': row.get('gated'), 'place_seed_rc': row.get('place_seed_rc'),
                'grade_errors': row.get('grade_errors'), 'unseated': row.get('unseated'),
                'probe_failures': probe.get('failures'), 'probe_status': probe.get('status'),
                'passes': (not row.get('gated') and row.get('place_seed_rc') == 0
                           and probe.get('failures') == 0),
                'board_sha256': sha(row['board']) if os.path.exists(row['board']) else None}
        doc = {'engine_sha': git(repo, 'rev-parse', 'HEAD'), 'input_sha256': sha(board),
               'rc': proc.returncode, 'seconds': round(time.perf_counter() - t0, 1),
               'passes': sum(r['passes'] for r in rows.values()), 'rows': rows}
        with open(out_path, 'w', encoding='utf-8') as stream:
            json.dump(doc, stream, indent=1, sort_keys=True)
        print(f'{doc["engine_sha"][:10]}: {doc["passes"]}/{len(rows)} seeds pass '
              f'({doc["seconds"]}s)')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def diff_seeds(a_path, b_path):
    with open(a_path, encoding='utf-8') as stream:
        a = json.load(stream)
    with open(b_path, encoding='utf-8') as stream:
        b = json.load(stream)
    if a['input_sha256'] != b['input_sha256']:
        print('INPUTS DIFFER -- not comparable')
        return 1
    lost = []
    for seed in sorted(a['rows'], key=int):
        ra, rb = a['rows'][seed], b['rows'].get(seed, {})
        same = ra.get('board_sha256') == rb.get('board_sha256')
        print(f'seed {seed}: base {"pass" if ra["passes"] else "FAIL"} '
              f'branch {"pass" if rb.get("passes") else "FAIL"} '
              f'{"same board" if same else "board differs"}')
        if ra['passes'] and not rb.get('passes'):
            lost.append(seed)
    print(f'{a["engine_sha"][:10]} {a["passes"]}/{len(a["rows"])}  ->  '
          f'{b["engine_sha"][:10]} {b["passes"]}/{len(b["rows"])};  lost: {lost}')
    return 1 if lost else 0


def time706(trees, rounds):
    names = [os.path.abspath(t) for t in trees]
    samples = {t: [] for t in names}
    cmd = [sys.executable, '-X', 'utf8', os.path.join('tests', 'test_706_seat_edge_target.py')]
    for t in names:                                   # one discarded warm-up each
        subprocess.run(cmd, cwd=t, capture_output=True, env=env())
    for r in range(rounds):
        order = names[r % len(names):] + names[:r % len(names)]
        for t in order:
            t0 = time.perf_counter()
            proc = subprocess.run(cmd, cwd=t, capture_output=True, env=env())
            dt = time.perf_counter() - t0
            if proc.returncode != 0:
                sys.exit(f'test_706 failed in {t}')
            samples[t].append(dt)
            print(f'round {r} {git(t, "rev-parse", "--short", "HEAD")} {dt:.2f}s', flush=True)
    med = {t: statistics.median(v) for t, v in samples.items()}
    first = names[0]
    for t in names:
        print(f'{git(t, "rev-parse", "--short", "HEAD")}: median {med[t]:.2f}s  min '
              f'{min(samples[t]):.2f}s  ratio to {git(first, "rev-parse", "--short", "HEAD")} '
              f'{med[t] / med[first]:.3f}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repo', default=HERE)
    ap.add_argument('--out')
    ap.add_argument('--diff', nargs=2)
    ap.add_argument('--seeds-out')
    ap.add_argument('--diff-seeds', nargs=2)
    ap.add_argument('--time706', nargs='+')
    ap.add_argument('--rounds', type=int, default=5)
    args = ap.parse_args()
    if args.diff:
        return diff(*args.diff)
    if args.diff_seeds:
        return diff_seeds(*args.diff_seeds)
    if args.time706:
        time706(args.time706, args.rounds)
        return 0
    if args.seeds_out:
        seeds(args.repo, args.seeds_out)
        return 0
    if not args.out:
        ap.error('one of --out, --diff, --seeds-out, --diff-seeds, --time706 is required')
    collect(args.repo, args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
