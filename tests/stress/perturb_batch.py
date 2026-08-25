#!/usr/bin/env python3
"""First-batch runner for the #411 perturbed-corpus rig.

Walks the `plan-pcb-routing` skill's Step 0 ladder on a PERTURBED board and
scores the result against the original, which is free ground truth.

    Step 0    decision table -- a perturbed board is the "rough / imported /
              auto-generated placement" row, so placement is warranted
    Step 0b   place_optimize --suggest-locks -> the refs to lock
    Step 0c   place_optimize --max-displacement 3 --lock <those refs>
              (and a second arm at a cap capable of the dose, which the skill
              does NOT prescribe -- that difference is the measurement)
    Step 0d   render the delta
    Step 0e   check_floorplan --intent --health

Scores are APPENDED to a scoreboard JSONL, never rewritten, so a later round
adds rows rather than replacing them and nothing has to be reverted to re-run.

    python3 -X utf8 tests/stress/perturb_batch.py --out-dir wk/batch1 \\
        --boards glasgow_revC splitflap_driver tigard
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

_here = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(_here, '..', '..'))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split

BOARDS_DIR = os.path.join(ROOT, 'kicad_files')
# The manufacturing floor each board is actually routed and graded at, from
# `list_nets.py --design-rules`. Grading at anything else manufactures phantom
# violations or hides real ones.
CLEARANCE = {'tigard': 0.09, 'splitflap_driver': 0.10, 'glasgow_revC': 0.09}
WANT_ARMS = ('quench', 'loop')
# Rails, so block displacement does not degenerate into "distance from the
# middle of the board" (GND owns 96 of ulx3s's parts).
IGNORE_NETS = ['GND', 'VCC', 'VDD', '+3V3', '+5V', '+1V8', '+1V1', 'GNDA']


def _run(argv, log_path, timeout=3600):
    with open(log_path, 'w', encoding='utf-8') as fh:
        r = subprocess.run(argv, stdout=fh, stderr=subprocess.STDOUT,
                           text=True, encoding='utf-8', errors='replace',
                           cwd=ROOT, timeout=timeout)
    with open(log_path, encoding='utf-8') as fh:
        return r.returncode, fh.read()


def _json_summary(text):
    for line in reversed(text.splitlines()):
        if line.startswith('JSON_SUMMARY: '):
            try:
                return json.loads(line[len('JSON_SUMMARY: '):])
            except ValueError:
                return {}
    return {}


def suggest_locks(board, work):
    """Step 0b. Report-only; returns the refs the advisor says to lock.

    Reads `lock_patterns`, which is the advisor's OWN paste-ready answer -- the
    same list it prints as `--lock ...`. An earlier version of this function
    rebuilt the list from `findings[].reference`, and the field is called `ref`:
    every lookup returned None, so the quench ran with NO locks at all. That is
    precisely the Step 0a failure the skill warns about ("a lock you forgot to
    re-pass is silent"), and it produced a plausible-looking run. Reading the
    tool's own output instead of re-deriving it is the fix and the lesson.
    """
    js = os.path.join(work, 'lock_advice.json')
    rc, out = _run([sys.executable, '-X', 'utf8',
                    os.path.join(ROOT, 'py_placer', 'place_optimize.py'), board,
                    '--suggest-locks', '--suggest-locks-json', js],
                   os.path.join(work, 'locks.log'))
    tally = _json_summary(out)
    refs = []
    if os.path.exists(js):
        with open(js, encoding='utf-8') as fh:
            doc = json.load(fh)
        refs = list(doc.get('lock_patterns') or ())
        if not refs:                       # older/None schema: rebuild by hand
            refs = [f['ref'] for f in doc.get('findings', [])
                    if f.get('confidence') == 'high' and f.get('ref')
                    and not f.get('already_locked')]
    if not refs:
        print('  WARNING: the lock advisor returned no refs to lock; the '
              'quench arm will run unlocked. Treat that as "nothing detected", '
              'not "nothing to lock".', flush=True)
    return sorted(set(refs)), tally


def quench_arm(board, out_board, work, label, max_disp, locks,
               max_passes=4, timeout=1800):
    """Step 0c, at a stated displacement cap.

    `--step` is scaled with the cap, and that is not a detail: the candidate
    grid is `n = int(max_disp / step)` on a side, so cost grows as
    `(cap/step)^2`. Measured on tigard (20 movable refs, max_passes=2): cap 3
    -> 0.6 s, cap 10 -> 2.6 s, cap 20 -> 8.6 s, cap 40 -> 17.8 s; at cap 20
    with `--step 3.0` it is 1.5 s for an IDENTICAL crossings result. Holding
    step at 1.0 for a cap-matched arm is how a batch that should take minutes
    takes hours, which is a budget failure disguised as a slow board.
    """
    step = max(0.5, round(max_disp / 6.0, 3))
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_placer', 'place_optimize.py'), board, out_board,
            '--max-displacement', str(max_disp),
            '--step', str(step), '--max-passes', str(max_passes),
            # The guidance weights the skill's Step 0c command uses.
            '--length-weight', '0.3', '--crossing-penalty', '30',
            '--halo-coef', '0.15', '--halo-weight', '2', '--edge-halo', '2',
            '--ignore-nets'] + IGNORE_NETS
    if locks:
        argv += ['--lock'] + locks
    t0 = time.time()
    try:
        rc, out = _run(argv, os.path.join(work, f'{label}.log'),
                       timeout=timeout)
    except subprocess.TimeoutExpired:
        # A timeout is `timed_out`, NEVER "did not recover". Conflating the two
        # is how a budget failure gets published as a negative result.
        return {'rc': None, 'timed_out': True,
                'seconds': round(time.time() - t0, 1), 'argv': argv,
                'summary': {}}
    return {'rc': rc, 'timed_out': False, 'seconds': round(time.time() - t0, 1),
            'step': step, 'max_passes': max_passes,
            'argv': argv, 'summary': _json_summary(out)}


def causal_nets(control_board, members, ignore_nets):
    """The nets the perturbed unit owns, computed on the ORIGINAL board.

    Frozen before either arm runs and identical for every arm, so a recovery
    cannot choose its own exam -- the same causal-scoping design as
    tests/test_placement_probe.py, and for the same reason: scoping by "nets
    touching parts that moved" is circular, because an arm that moves nothing
    then scores a perfect null.
    """
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(control_board)
    ig = {n.upper() for n in ignore_nets}
    want = set()
    for ref in members:
        fp = pcb.footprints.get(ref)
        if not fp:
            continue
        for pad in fp.pads:
            if pad.net_id and pad.net_name:
                nm = pad.net_name
                base = nm.split('/')[-1].upper()
                if base in ig or nm.upper() in ig:
                    continue
                if len(pcb.pads_by_net.get(pad.net_id, ())) >= 2:
                    want.add(nm)
    return sorted(want)


# The two loop configurations. Running ONLY the shipped defaults would make the
# answer a tautology: `nets_to_refs` drops any footprint over --max-target-pins,
# and --group-by defaults to none, so L0 structurally cannot move a block and
# "recovers nothing" would be the documented behaviour of the stop-loss rather
# than a measurement. L3 removes both limits, so a null there is a real null.
LOOP_ARMS = {
    # The shipped command, on a FULL-board route. Scoping the route to the
    # unit's own nets is what the cheap tier does, and it is wrong here:
    # measured on splitflap_driver/translate, a 63-net scoped route of the
    # PERTURBED board returned failures=0, so the loop printed "No failures
    # left - stopping" with rounds_run=0 and handed back its input byte for
    # byte. Scoping makes routing EASIER, and the loop only ever moves parts
    # that own a failed or blocking net.
    'loop@shipped': dict(max_disp=3.0, step=0.5, pins=40, group='none',
                         targeted=False),
    # Cap matched to the dose, pin gate lifted, blocks enabled. Without this
    # arm a null is the documented behaviour of the stop-loss rather than a
    # measurement.
    'loop@allon':   dict(max_disp=None, step=None, pins=100000,
                         group='sheet,decap', targeted=False),
    # The skill's own answer to "everything routes but the floorplan is wrong":
    # --target-nets supplies targets even at zero failures. Shipped caps, so the
    # ONLY difference from loop@shipped is whether the loop engages at all --
    # which separates "the caps are too small" from "it never got to targeting".
    'loop@targeted': dict(max_disp=3.0, step=0.5, pins=40, group='none',
                          targeted=True),
}


def loop_arm(board, out_board, work, label, cfg, nets, clearance, applied,
             rounds=5, timeout=5400):
    """place_route_loop -- the only arm that re-routes and accept/reverts."""
    max_disp = cfg['max_disp'] if cfg['max_disp'] else max(3.0, round(applied * 1.5, 2))
    step = cfg['step'] if cfg['step'] else max(0.5, round(max_disp / 6.0, 3))
    excl = ' '.join(f'"!{n}"' for n in IGNORE_NETS)
    route_args = f'--nets "*" {excl} --clearance {clearance}'
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_placer', 'place_route_loop.py'), board, out_board,
            '--route-args', route_args,
            '--work-dir', os.path.join(work, label.replace('@', '_')),
            '--rounds', str(rounds),
            '--max-displacement', str(max_disp), '--step', str(step),
            '--max-target-pins', str(cfg['pins']),
            '--group-by', cfg['group'],
            '--no-movie']
    if cfg.get('targeted'):
        argv += ['--target-nets'] + nets
    argv += ['--ignore-nets'] + IGNORE_NETS
    t0 = time.time()
    try:
        rc, out = _run(argv, os.path.join(work, f'{label}.log'), timeout=timeout)
    except subprocess.TimeoutExpired:
        return {'rc': None, 'timed_out': True, 'max_displacement': max_disp,
                'seconds': round(time.time() - t0, 1), 'argv': argv,
                'summary': {}, 'nets': len(nets)}
    # The analytic ceiling on how far this arm could POSSIBLY travel: the cap
    # widens 1.5x per rejected round and resets on accept, so a dose beyond
    # this is out of reach by arithmetic, not by any property of the search.
    ceiling = max_disp * (1.5 ** max(0, rounds - 1))
    return {'rc': rc, 'timed_out': False, 'seconds': round(time.time() - t0, 1),
            'max_displacement': max_disp, 'step': step, 'rounds': rounds,
            'cap_ceiling_mm': round(ceiling, 3),
            'cap_infeasible': bool(applied > ceiling),
            'nets': len(nets), 'argv': argv, 'summary': _json_summary(out)}


def grade_floorplan(board, intent, work, label):
    """Step 0e -- an INDEPENDENT check: not produced by the thing being graded."""
    rc, out = _run([sys.executable, '-X', 'utf8',
                    os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), board,
                    '--intent', intent, '--health', '--exit-zero', '-q'],
                   os.path.join(work, f'{label}_floorplan.log'))
    return _json_summary(out)


def do_board(name, kind, out_dir, doses):
    from placement import perturb as P
    from placement import recovery as R

    board = os.path.join(BOARDS_DIR, f'{name}.kicad_pcb')
    work = os.path.join(out_dir, f'{name}__{kind}')
    os.makedirs(work, exist_ok=True)
    rows = []

    for dose in doses:
        cell = os.path.join(work, f'd{dose:g}')
        os.makedirs(cell, exist_ok=True)
        # GROUND TRUTH LIVES OUTSIDE THE CELL. The cell is the work dir every
        # arm runs in, and the control is the human placement pose-for-pose --
        # left beside `perturbed.kicad_pcb` (perturb's compatibility default)
        # any glob, any `--before`, any tool taking a directory could read it,
        # so the experiment would be fenced on paper and open in fact. `_truth/`
        # is a sibling of the d<dose> cells, so `fence_audit --workdir <cell>`
        # never walks it.
        truth = os.path.join(work, '_truth', f'd{dose:g}')
        os.makedirs(truth, exist_ok=True)
        perturbed = os.path.join(cell, 'perturbed.kicad_pcb')
        try:
            rec = P.perturb(board, perturbed, kind=kind, dose_mm=dose,
                            ignore_nets=IGNORE_NETS,
                            control_out=os.path.join(truth,
                                                     'control.kicad_pcb'))
        except Exception as exc:                            # noqa: BLE001
            rows.append({'board': name, 'kind': kind, 'dose_mm_requested': dose,
                         'status': 'error', 'reason': f'{type(exc).__name__}: {exc}'})
            continue
        if rec.get('status') != 'ok':
            rows.append({'board': name, 'kind': kind, 'dose_mm_requested': dose,
                         'status': rec.get('status'), 'reason': rec.get('reason')})
            continue

        control = rec['control_board']
        applied = rec['dose_mm_applied']
        members = rec['block']['members']

        # Step 0e: the ground-truth intent comes off the CONTROL, plus a zone
        # per perturbed unit at its ORIGINAL bbox. Emitted intent alone is
        # vacuous on boards with no sheets (tigard/splitflap: 0 zones), so
        # without this the grader would PASS a wildly perturbed board.
        # It is DERIVED from truth (the unit's original bbox), so it belongs in
        # `_truth/` with the control -- it is the grader's input, never an arm's.
        intent = os.path.join(truth, 'truth_intent.json')
        _emit_truth_intent(control, members, intent)

        # Step 0b on the perturbed board -- what the skill would actually see.
        locks, lock_tally = suggest_locks(perturbed, cell)

        arms = {'none': perturbed}
        arms_meta = {}
        if 'quench' in WANT_ARMS:
            # The cap the skill PRESCRIBES, and a cap CAPABLE of the dose. The
            # difference between them is the finding.
            for label, cap in (('quench@3mm', 3.0),
                               ('quench@dose', max(3.0, round(applied * 1.5, 2)))):
                ob = os.path.join(cell, f'{label.replace("@", "_")}.kicad_pcb')
                arms_meta[label] = quench_arm(perturbed, ob, cell, label, cap,
                                              locks)
                if os.path.exists(ob):
                    arms[label] = ob
        if 'loop' in WANT_ARMS:
            nets = causal_nets(control, members, IGNORE_NETS)
            if not nets:
                print(f'  {name}/{kind}: no causal nets for the unit; '
                      f'skipping the loop arms', flush=True)
            else:
                print(f'  {name}/{kind}: {len(nets)} causal net(s), frozen from '
                      f'the original', flush=True)
                for label, cfg in LOOP_ARMS.items():
                    ob = os.path.join(cell, f'{label.replace("@", "_")}.kicad_pcb')
                    meta = loop_arm(perturbed, ob, cell, label, cfg, nets,
                                    CLEARANCE.get(name, 0.1), applied)
                    arms_meta[label] = meta
                    print(f'    {label:14s} {meta["seconds"]:8.1f}s '
                          f'cap {meta["max_displacement"]}mm '
                          f'ceiling {meta.get("cap_ceiling_mm")}mm '
                          f'infeasible={meta.get("cap_infeasible")} '
                          f'timed_out={meta["timed_out"]}', flush=True)
                    if os.path.exists(ob):
                        arms[label] = ob

        for label, b in arms.items():
            score = R.score_board(b, rec)
            d0, d1 = applied, score['displacement'].get('perturbed_pad_rms')
            fp = grade_floorplan(b, intent, cell, label.replace('@', '_'))
            rows.append({
                'board': name, 'kind': kind,
                'unit': rec['block']['name'], 'unit_source': rec['block']['source'],
                'unit_parts': len(members),
                'dose_mm_requested': dose, 'dose_mm_applied': applied,
                'dose_norm': rec.get('dose_norm'), 'clipped': rec.get('clipped'),
                'max_feasible_dose_mm': rec.get('max_feasible_dose_mm'),
                'direction_source': rec.get('direction_source'),
                'arm': label,
                'recovery_fraction': R.recovery_fraction(d0, d1),
                'displacement': score['displacement'],
                'unit_spread_ratio': score.get('unit_spread_ratio'),
                'unit_gyration_orig_mm': score.get('unit_gyration_orig_mm'),
                'HPWL': score['HPWL'], 'NC': score['NC'],
                'OO': score['OO'], 'OoB': score['OoB'],
                'block_displacement_mm': score['block_displacement_mm'],
                'state': score['state'],
                'floorplan': fp,
                'locks_advised': locks, 'lock_tally': lock_tally,
                'arm_meta': arms_meta.get(label),
                'board_path': os.path.abspath(b),
                'control_path': control,
                'status': 'ok',
            })
        # Baseline row for the control, so every delta has something to be
        # measured against and a degenerate reference is visible.
        ctl = R.score_board(control, rec)
        rows.append({
            'board': name, 'kind': kind, 'arm': 'control',
            'unit': rec['block']['name'], 'unit_parts': len(members),
            'dose_mm_requested': dose, 'dose_mm_applied': applied,
            'recovery_fraction': None,
            'displacement': ctl['displacement'],
            'HPWL': ctl['HPWL'], 'NC': ctl['NC'], 'OO': ctl['OO'],
            'OoB': ctl['OoB'],
            'block_displacement_mm': ctl['block_displacement_mm'],
            'state': ctl['state'],
            'floorplan': grade_floorplan(control, intent, cell, 'control'),
            'board_path': control, 'control_path': control, 'status': 'ok',
        })
    return rows


def _emit_truth_intent(control_board, members, out_path):
    """emit_intent from the control, plus a zone at the unit's ORIGINAL bbox.

    Measured: `--emit-intent` claims zones for 4 of 10 blocks on ulx3s, 1 of 3
    on glasgow_revC, and **0 on tigard and splitflap_driver**. On those boards a
    grade against the plain emitted intent PASSES a wildly perturbed placement,
    because there is no zone to violate. The added zone comes from the
    known-good placement, never from the recovery, so it is ground truth rather
    than a target fitted to the answer.
    """
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    pcb = parse_kicad_pcb(control_board)
    doc = floorplan.emit_intent(pcb, control_board)
    xs, ys = [], []
    for r in members:
        fp = pcb.footprints.get(r)
        if fp:
            xs.append(fp.x)
            ys.append(fp.y)
    if xs:
        pad = 2.0
        doc.setdefault('blocks', []).append({
            'name': 'perturbed-unit',
            'refs': sorted(members),
            'zone': [round(min(xs) - pad, 3), round(min(ys) - pad, 3),
                     round(max(xs) - min(xs) + 2 * pad, 3),
                     round(max(ys) - min(ys) + 2 * pad, 3)],
            'note': 'the unit bbox on the KNOWN-GOOD placement (#411 ground truth)',
        })
    with open(out_path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=1, sort_keys=True)


def append_scoreboard(path, rows, code_version):
    """Append-only. A re-run adds rows; nothing is rewritten, so nothing has to
    be reverted to run again."""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'a', encoding='utf-8') as fh:
        for r in rows:
            r = dict(r)
            r['code_version'] = code_version
            fh.write(json.dumps(r, sort_keys=True) + '\n')


def git_version():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, cwd=ROOT,
                              timeout=30).stdout.strip() or 'unknown'
    except Exception:                                       # noqa: BLE001
        return 'unknown'


def table(rows):
    hdr = (f"{'board':17s} {'kind':9s} {'dose':>6s} {'appl':>7s} {'arm':>12s} "
           f"{'recov':>7s} {'disp':>7s} {'HPWL':>9s} {'NC':>6s} {'OO':>8s} "
           f"{'oob':>4s} {'intent':>7s}")
    out = [hdr, '-' * len(hdr)]
    for r in rows:
        if r.get('status') != 'ok':
            out.append(f"{r['board']:17s} {r['kind']:9s} "
                       f"{r.get('dose_mm_requested', 0):6.1f} "
                       f"{'--':>7s} {'--':>12s}  {r.get('status')}: "
                       f"{str(r.get('reason'))[:40]}")
            continue
        d = r.get('displacement') or {}
        rf = r.get('recovery_fraction')
        fp = r.get('floorplan') or {}
        out.append(
            f"{r['board']:17s} {r['kind']:9s} {r.get('dose_mm_requested', 0):6.1f} "
            f"{r.get('dose_mm_applied') or 0:7.3f} {r['arm']:>12s} "
            f"{('--' if rf is None else f'{rf:+.3f}'):>7s} "
            f"{d.get('perturbed_pad_rms', 0):7.3f} {r['HPWL']:9.1f} {r['NC']:6d} "
            f"{r['OO']:8.2f} {r['OoB']['count']:4d} "
            f"{fp.get('errors', '--'):>7}")
    return '\n'.join(out)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out-dir', required=True)
    p.add_argument('--boards', nargs='+', default=['glasgow_revC', 'tigard'])
    p.add_argument('--kinds', nargs='+', default=['swap', 'translate'])
    p.add_argument('--doses', nargs='+', type=float, default=[10.0])
    p.add_argument('--scoreboard', default=None,
                   help='append-only JSONL (default <out-dir>/scoreboard.jsonl)')
    p.add_argument('--arms', nargs='+', default=['quench', 'loop'],
                   choices=['quench', 'loop'],
                   help='arm families to run (loop re-routes; hours, not minutes)')
    a = p.parse_args(argv)
    global WANT_ARMS
    WANT_ARMS = tuple(a.arms)

    os.makedirs(a.out_dir, exist_ok=True)
    sb = a.scoreboard or os.path.join(a.out_dir, 'scoreboard.jsonl')
    ver = git_version()
    all_rows = []
    for name in a.boards:
        for kind in a.kinds:
            print(f"[batch] {name} / {kind} ...", flush=True)
            rows = do_board(name, kind, a.out_dir,
                            [0.0] if kind in ('swap', 'pile') else a.doses)
            append_scoreboard(sb, rows, ver)
            all_rows.extend(rows)
    print('\n' + table(all_rows))
    print(f"\nscoreboard: {sb}  ({ver})")
    return 0


if __name__ == '__main__':
    sys.exit(main())
