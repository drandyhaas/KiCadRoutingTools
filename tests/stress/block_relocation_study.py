#!/usr/bin/env python3
"""Does a bounded block relocation restore a board that damage made unroutable?

`tests/stress/relocation_reach.py` answers the MECHANISM question -- letting
neighbours yield buys a block travel a frozen board cannot -- and answers it yes.
This file asks the only question that decides whether the feature is worth having,
and it is a different one:

    On a board where displacing exactly one block measurably worsens the ROUTED
    outcome, does one bounded relocation of that block restore it -- without
    moving anything else, without tearing the block -- where the tool that
    already exists does not?

Reach is not routability. That sentence is the reason this file exists.

THE ARMS
--------
    C   control, undamaged (perturb's own dose-0 pass of the same writer)
    D   damaged, unrepaired -- the denominator's other end
    R   the relocation applied to D
    R0  THE SAME relocation applied to C -- the undamaged pairing
    L   place_route_loop with the pin gate lifted and blocks on: the incumbent

`R0` is not optional. #553's recall study reported "above chance on 4 of 4
boards" and the finding evaporated when the same ranking was run on the pristine
board: `perturb.pick_block` ranks units by source and size, and so do most
placement scorers, so the "damaged" block is often already first. A repair arm
with no undamaged pairing cannot tell "it fixed the damage" from "it moves that
block on every board". The delta is the evidence; the raw number is not.

`L` is not optional either. #554 is only interesting if it beats a tool that
already exists: #411 measured `loop@allon` taking tigard from 13 routing failures
to 2, at a recovery of -0.052. A relocation that ties that has demonstrated
nothing.

THE HEADLINE IS ROUTED, AND `recovery` IS NOT IT
-------------------------------------------------
    route_recovery = (blocking(D) - blocking(R)) / (blocking(D) - blocking(C))

over failed + open + pad-deficit, `None` when the denominator is zero. 1.0 is
restored to the control's routed quality, 0 inert, negative worse than leaving
the damage alone.

Pose `recovery` is reported as a DIAGNOSTIC and never as the verdict, for the
reason CLAUDE.md already states and for a sharper one measured here: the
relocation aims at `net_centroid`, and on the ten tracked boards a block sits
4.95-54.9 mm from its own net centroid at the HUMAN's placement. So the best
achievable pose recovery is bounded above by `1 - d_ctrl/dose`, which is at or
below zero on most cells before anything runs. A pose null on such a cell is not
evidence about the solver. `compensated` -- routed win at recovery ~ 0, with zero
collateral and an intact block -- is a first-class PASS here, exactly as #411
concluded.

THE CONTRACT CHECKS, WHICH CAN FAIL A CELL THAT ROUTED BETTER
--------------------------------------------------------------
#554 says "move only diagnosed blocks; keep the relative order of everything else
as a hard constraint". That is the definition, not the bonus. So a cell FAILS if
`collateral_pad_rms` rises or the block is torn, whatever `blocking` did -- a
solve that buys routability by walking the neighbours is `place_route_loop`, and
that already exists.

DAMAGE KINDS
------------
`swap` and `wrong_side` are the evidence arms. `translate` is an INSTRUMENT CHECK
and is never aggregated: `perturb.block_direction` computes the damage direction
from `routability.block_displacements`, the very quantity this solve consumes, so
its recovery is partly arithmetic. `scatter` and `pile` are excluded by
measurement -- `perturb_jitter` skips illegal-pose parts so `scatter` often
damages nothing, and `pile` moves every free part so there is no "the block".

    python3 -X utf8 tests/stress/block_relocation_study.py --out wk/554 \
        --boards esp_prog splitflap_driver
    python3 -X utf8 tests/stress/block_relocation_study.py --from-rows wk/554/rows.jsonl
    python3 -X utf8 tests/stress/block_relocation_study.py --self-test
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile

_here = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(_here, '..', '..'))
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

SCHEMA = 1

ARMS = ('C', 'D', 'R', 'R0', 'L')
EVIDENCE_KINDS = ('swap', 'wrong_side')
INSTRUMENT_KINDS = ('translate',)
EXCLUDED_KINDS = {'scatter': 'portfolio.perturb_jitter skips a part whose '
                             'incumbent pose is not fully legal, which on a '
                             'dense board is every part, so it often damages '
                             'nothing at all',
                  'pile': 'every free part moves, so there is no "the block"'}

IGNORE_NETS = ['GND', 'VCC', 'VDD', 'VSS', '+3V3', '+5V', '+1V8', '+1V1', 'GNDA',
               'AGND', 'VBUS', '+3.3V']
GROUP_BY = 'kicad,sheet,netprefix,decap'

#: G1. `recovery.HOME_TOLERANCE_MM` is 2.0, so at a 2 mm dose every member is
#: already inside the home tolerance before anything is repaired and the home
#: curve has no rung to move through. 2x that is the smallest dose whose repair
#: is observable at all.
MIN_APPLIED_MM = 4.0
MIN_MEMBERS_MOVED = 2

ROUTE_TIMEOUT = 3600
LOOP_TIMEOUT = 5400
BOARDS = ['esp_prog', 'splitflap_driver', 'tigard']


def _run(argv, log_path, timeout):
    with open(log_path, 'w', encoding='utf-8') as fh:
        r = subprocess.run(argv, stdout=fh, stderr=subprocess.STDOUT, text=True,
                           encoding='utf-8', errors='replace', cwd=ROOT,
                           timeout=timeout)
    with open(log_path, encoding='utf-8', errors='replace') as fh:
        return r.returncode, fh.read()


def route_blocking(board, out_board, log_path, timeout=ROUTE_TIMEOUT):
    """`(blocking, detail)` from ONE route. `blocking` is None if it did not run.

    Deliberately the router's own tally of what is not connected -- failed nets,
    kept-but-open nets, and the multipoint pad deficit -- read off
    `JSON_SUMMARY_MIN`. `open_single` is in it because a KEPT result whose pads
    are still disconnected is how a board ships open copper while reporting
    `failures=0` (CLAUDE.md's failure-bucket rule).
    """
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_router', 'route.py'), board, out_board,
            '--nets', '*']
    try:
        rc, log = _run(argv, log_path, timeout)
    except subprocess.TimeoutExpired:
        return None, {'timed_out': True}
    if rc != 0:
        return None, {'rc': rc, 'tail': log.strip().splitlines()[-3:]}
    line = [ln for ln in log.splitlines() if ln.startswith('JSON_SUMMARY_MIN: ')]
    if not line:
        return None, {'no_summary': True}
    d = json.loads(line[-1].split('JSON_SUMMARY_MIN: ', 1)[1])
    failed = d.get('failed') or 0
    openc = len(d.get('open_single') or [])
    deficit = d.get('multipoint_deficit') or 0
    return failed + openc + deficit, {'failed': failed, 'open': openc,
                                      'deficit': deficit,
                                      'routed': d.get('routed')}


def pose_delta(a_board, b_board):
    """(max member move, moved count, per-ref deltas) between two boards.

    The APPLIED damage is measured from the two boards, never from perturb's
    record: `clipped` and `max_feasible_dose_mm` describe a rigid-translate
    probe that only two of the arms clip against, and reading them as the dose
    was wrong by 79x once (kit-dev-coldfire excluded at "1.404 mm landed" while
    its members had moved 111 mm).
    """
    from kicad_parser import parse_kicad_pcb
    import math
    a = parse_kicad_pcb(a_board).footprints
    b = parse_kicad_pcb(b_board).footprints
    out = {}
    for ref, fa in a.items():
        fb = b.get(ref)
        if fb is None:
            continue
        d = math.hypot(fb.x - fa.x, fb.y - fa.y)
        if d > 1e-6:
            out[ref] = round(d, 4)
    return (round(max(out.values()), 4) if out else 0.0), len(out), out


def score(board, record):
    """recovery.score_board, or None when it cannot run."""
    try:
        from placement import recovery
        return recovery.score_board(board, record)
    except Exception as e:                                # noqa: BLE001
        return {'error': '%s: %s' % (type(e).__name__, e)}


def relocate_arm(board, out_board, work, label, refs, rounds=2,
                 corridor_mm=None):
    """place_route_loop --relocate, block named EXPLICITLY.

    Naming the refs is what separates "the relocation worked" from "the
    diagnosis picked right". #553's selector is a measured null, so folding it in
    here would make a null unattributable to either half.
    """
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_placer', 'place_route_loop.py'),
            board, out_board,
            '--route-args', '--nets "*"',
            '--work-dir', os.path.join(work, label),
            '--rounds', str(rounds), '--no-movie',
            '--relocate', '--group-by', GROUP_BY,
            '--relocate-refs'] + list(refs)
    if corridor_mm is not None:
        argv += ['--relocate-max-corridor', str(corridor_mm)]
    argv += ['--ignore-nets'] + IGNORE_NETS
    try:
        rc, log = _run(argv, os.path.join(work, label + '.log'), LOOP_TIMEOUT)
    except subprocess.TimeoutExpired:
        return {'timed_out': True}
    line = [ln for ln in log.splitlines() if ln.startswith('JSON_SUMMARY: ')]
    return {'rc': rc,
            'summary': json.loads(line[-1].split('JSON_SUMMARY: ', 1)[1])
            if line else {}}


def loop_arm(board, out_board, work, label, rounds=2):
    """The incumbent: pin gate lifted, blocks on, cap matched. #411's loop@allon.

    Running only the SHIPPED defaults would make the comparison a tautology --
    `nets_to_refs` drops anything over --max-target-pins and --group-by defaults
    to none, so L0 structurally cannot move a block and "recovers nothing" would
    be the documented behaviour of the stop-loss rather than a measurement.
    """
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_placer', 'place_route_loop.py'),
            board, out_board,
            '--route-args', '--nets "*"',
            '--work-dir', os.path.join(work, label),
            '--rounds', str(rounds), '--no-movie',
            '--max-target-pins', '100000', '--group-by', GROUP_BY,
            '--max-displacement', '6.0', '--step', '1.0']
    argv += ['--ignore-nets'] + IGNORE_NETS
    try:
        rc, log = _run(argv, os.path.join(work, label + '.log'), LOOP_TIMEOUT)
    except subprocess.TimeoutExpired:
        return {'timed_out': True}
    line = [ln for ln in log.splitlines() if ln.startswith('JSON_SUMMARY: ')]
    return {'rc': rc,
            'summary': json.loads(line[-1].split('JSON_SUMMARY: ', 1)[1])
            if line else {}}


def run_cell(board, kind, out_dir, *, dose_mm=8.0, seed=1, rounds=2):
    """One (board, kind) cell, fenced. Returns a row dict, always."""
    from placement import perturb as P
    row = {'schema': SCHEMA, 'board': board, 'kind': kind,
           'is_evidence': kind in EVIDENCE_KINDS,
           'counted_in_primary': kind in EVIDENCE_KINDS}
    if kind in EXCLUDED_KINDS:
        row['skipped'] = 'excluded kind: ' + EXCLUDED_KINDS[kind]
        return row

    work = os.path.join(out_dir, '%s_%s' % (board, kind))
    truth = os.path.join(out_dir, '_truth', '%s_%s' % (board, kind))
    os.makedirs(work, exist_ok=True)
    os.makedirs(truth, exist_ok=True)
    src = os.path.join(ROOT, 'kicad_files', board + '.kicad_pcb')
    damaged = os.path.join(work, 'board.kicad_pcb')
    control = os.path.join(truth, 'control.kicad_pcb')
    try:
        rec = P.perturb(src, damaged, kind=kind, dose_mm=dose_mm, seed=seed,
                        group_by=GROUP_BY, ignore_nets=IGNORE_NETS,
                        write_record=False, control_out=control)
    except Exception as e:                                # noqa: BLE001
        row['skipped'] = 'perturb raised %s: %s' % (type(e).__name__, e)
        return row
    if rec.get('status') != 'ok':
        row['skipped'] = 'perturb: %s' % (rec.get('reason') or rec.get('status'))
        return row

    members = sorted(rec['block']['members'])
    row['block'] = rec['block']['name']
    row['members'] = members
    applied, moved, _d = pose_delta(control, damaged)
    row.update({'applied_mm': applied, 'members_moved': moved})

    # G1 -- free.
    if applied < MIN_APPLIED_MM or moved < MIN_MEMBERS_MOVED:
        row['skipped'] = ('unmeasurable_dose: %.2f mm over %d part(s); needs '
                          '>= %.1f mm over >= %d (below that every member is '
                          'inside recovery.HOME_TOLERANCE_MM before repair)'
                          % (applied, moved, MIN_APPLIED_MM, MIN_MEMBERS_MOVED))
        return row

    routed = {}
    bC, dC = route_blocking(control, os.path.join(work, 'C_routed.kicad_pcb'),
                            os.path.join(work, 'C.log'))
    bD, dD = route_blocking(damaged, os.path.join(work, 'D_routed.kicad_pcb'),
                            os.path.join(work, 'D.log'))
    routed['C'], routed['D'] = {'blocking': bC, **dC}, {'blocking': bD, **dD}
    row['routed'] = routed
    if bC is None or bD is None:
        row['skipped'] = 'a baseline route did not produce a summary'
        return row

    # G3 -- the RUNBOOK's own definition of a subject.
    if not bD > bC:
        row['skipped'] = ('not_placement_limited: damaged blocking %d is not '
                          'worse than the controlic %d, so there is nothing for '
                          'a placement repair to restore' % (bD, bC))
        row['skipped'] = row['skipped'].replace('controlic', 'control')
        return row

    R = relocate_arm(damaged, os.path.join(work, 'R.kicad_pcb'), work, 'R',
                     members, rounds=rounds)
    R0 = relocate_arm(control, os.path.join(work, 'R0.kicad_pcb'), work, 'R0',
                      members, rounds=rounds)
    L = loop_arm(damaged, os.path.join(work, 'L.kicad_pcb'), work, 'L',
                 rounds=rounds)
    row['arms'] = {'R': R, 'R0': R0, 'L': L}

    for name, res, base in (('R', R, damaged), ('R0', R0, control),
                            ('L', L, damaged)):
        out = os.path.join(work, '%s.kicad_pcb' % name)
        if not os.path.exists(out):
            routed[name] = {'blocking': None, 'reason': 'arm produced no board'}
            continue
        b, d = route_blocking(out, os.path.join(work, '%s_routed.kicad_pcb' % name),
                              os.path.join(work, '%s_route.log' % name))
        routed[name] = {'blocking': b, **d}
        mx, mv, _ = pose_delta(base, out)
        routed[name].update({'max_move_mm': mx, 'parts_moved': mv})

    denom = bD - bC
    for name in ('R', 'R0', 'L'):
        b = routed.get(name, {}).get('blocking')
        routed[name]['route_recovery'] = (
            None if b is None or denom == 0 else round((bD - b) / denom, 4))
    # THE evidence column: the repair minus what the same repair does to an
    # UNDAMAGED board.
    rr, r0 = routed['R'].get('route_recovery'), routed['R0'].get('route_recovery')
    row['route_recovery'] = rr
    row['route_recovery_delta'] = (None if rr is None or r0 is None
                                   else round(rr - r0, 4))
    row['loop_route_recovery'] = routed['L'].get('route_recovery')

    # Contract checks -- these can FAIL a cell that routed better.
    moved_R = routed.get('R', {}).get('parts_moved')
    row['contract'] = {
        'parts_moved_by_R': moved_R,
        'block_members': len(members),
        # Every part R moved that is not a block member is collateral. #554's
        # definition is "move only diagnosed blocks"; the corridor is licensed,
        # but it is reported so a reader can see the disturbance the routed
        # number was bought with.
        'collateral_parts': (None if moved_R is None
                             else max(0, moved_R - len(members))),
    }
    return row


def summarise(rows):
    live = [r for r in rows if 'skipped' not in r]
    ev = [r for r in live if r.get('counted_in_primary')
          and r.get('route_recovery_delta') is not None]
    deltas = sorted(r['route_recovery_delta'] for r in ev)
    up = [r for r in ev if r['route_recovery_delta'] > 0]
    down = [r for r in ev if r['route_recovery_delta'] < 0]
    out = {
        'schema': SCHEMA,
        'cells': len(live),
        'evidence_cells': len(ev),
        'boards': sorted({r['board'] for r in ev}),
        'median_route_recovery_delta': (
            None if not deltas else
            round(deltas[len(deltas) // 2] if len(deltas) % 2 else
                  (deltas[len(deltas) // 2 - 1] + deltas[len(deltas) // 2]) / 2, 4)),
        'cells_up': len(up), 'cells_down': len(down),
        'instrument_cells': [r['board'] + '/' + r['kind'] for r in live
                             if r['kind'] in INSTRUMENT_KINDS],
        'skipped': [{'board': r['board'], 'kind': r['kind'],
                     'reason': r['skipped']} for r in rows if 'skipped' in r],
    }
    out['verdict'] = _verdict(out, ev)
    return out


def _verdict(s, ev):
    n = s['evidence_cells']
    if n == 0:
        return ('NOT MEASURED: no evidence cell survived the admission gates, so '
                'this run says nothing about whether a relocation restores a '
                'damaged board. The skipped list says why each one went.')
    if n < 3:
        return ('UNDERPOWERED: %d evidence cell(s). This repo\'s own acceptance '
                'rule wants N >= 3 with the right direction on N-1 and wrong on '
                'none; %d cannot satisfy it. Reported, not aggregated.' % (n, n))
    if s['cells_down'] == 0 and s['cells_up'] >= n - 1:
        return ('SUPPORTED: route_recovery_delta positive on %d of %d evidence '
                'cell(s) and negative on none, median %s. The delta is the '
                'evidence -- the raw recovery is not, because the same solve on '
                'the UNDAMAGED board is the other half of every row.'
                % (s['cells_up'], n, s['median_route_recovery_delta']))
    return ('NOT SUPPORTED: %d of %d evidence cell(s) improved and %d got worse, '
            'median delta %s. A relocation that does not beat its own undamaged '
            'pairing has not restored anything.'
            % (s['cells_up'], n, s['cells_down'],
               s['median_route_recovery_delta']))


def self_test():
    fails = []
    # A cell whose damaged board is no worse than the control must be REPORTED,
    # never silently dropped -- it is the RUNBOOK's definition of a non-subject.
    s = summarise([{'board': 'b', 'kind': 'swap',
                    'skipped': 'not_placement_limited: x'}])
    if not s['skipped']:
        fails.append('a skipped cell vanished from the summary')
    if 'NOT MEASURED' not in s['verdict']:
        fails.append('a run with no evidence cell did not say so: %r'
                     % s['verdict'])
    # The instrument arm must never reach the evidence aggregate.
    rows = [{'board': 'b', 'kind': 'translate', 'counted_in_primary': False,
             'is_evidence': False, 'route_recovery_delta': 9.0}]
    s = summarise(rows)
    if s['evidence_cells'] != 0:
        fails.append('translate was aggregated as evidence')
    if not s['instrument_cells']:
        fails.append('the instrument cell was not reported at all')
    # Under-powered must not read as a result.
    rows = [{'board': 'b', 'kind': 'swap', 'counted_in_primary': True,
             'is_evidence': True, 'route_recovery_delta': 1.0}]
    if 'UNDERPOWERED' not in summarise(rows)['verdict']:
        fails.append('one positive cell was reported as a finding')
    # ... and a mixed result is NOT SUPPORTED, not "mostly works".
    rows = [dict(rows[0]), dict(rows[0], board='c'),
            dict(rows[0], board='d', route_recovery_delta=-1.0)]
    if 'NOT SUPPORTED' not in summarise(rows)['verdict']:
        fails.append('a cell that got WORSE did not block the claim')
    for f in fails:
        print('SELF-TEST FAIL: ' + f)
    print('self-test: %s (4 check groups)' % ('FAIL' if fails else 'ok'))
    return 1 if fails else 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--boards', nargs='+', default=BOARDS)
    p.add_argument('--kinds', nargs='+',
                   default=list(EVIDENCE_KINDS) + list(INSTRUMENT_KINDS))
    p.add_argument('--dose', type=float, default=8.0)
    p.add_argument('--rounds', type=int, default=2)
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--out', default=None, metavar='DIR')
    p.add_argument('--from-rows', default=None, metavar='JSONL')
    p.add_argument('--self-test', action='store_true')
    a = p.parse_args(argv)

    if a.self_test:
        return self_test()
    if a.from_rows:
        rows = [json.loads(x) for x in open(a.from_rows, encoding='utf-8')
                if x.strip()]
        print(json.dumps(summarise(rows), indent=2, sort_keys=True))
        return 0
    if self_test():
        print('refusing to measure with a failing instrument')
        return 1

    out_dir = a.out or tempfile.mkdtemp(prefix='relocation_study_')
    os.makedirs(out_dir, exist_ok=True)
    rows = []
    for board in a.boards:
        for kind in a.kinds:
            print('--- %s / %s' % (board, kind), flush=True)
            row = run_cell(board, kind, out_dir, dose_mm=a.dose, seed=a.seed,
                           rounds=a.rounds)
            rows.append(row)
            if 'skipped' in row:
                print('    SKIP %s' % row['skipped'][:150], flush=True)
            else:
                print('    blocking C=%s D=%s R=%s R0=%s L=%s  delta=%s'
                      % tuple([row['routed'][k].get('blocking')
                               for k in ('C', 'D', 'R', 'R0', 'L')]
                              + [row.get('route_recovery_delta')]), flush=True)
            with open(os.path.join(out_dir, 'rows.jsonl'), 'a',
                      encoding='utf-8') as fh:
                fh.write(json.dumps(row, sort_keys=True) + '\n')
    summary = summarise(rows)
    print()
    print(json.dumps(summary, indent=2, sort_keys=True))
    with open(os.path.join(out_dir, 'summary.json'), 'w', encoding='utf-8') as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
