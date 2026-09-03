#!/usr/bin/env python3
"""#846 A/B: how far may `--allow-via-in-pad`'s escape-axis ladder reach?

The ladder was called `_onpad` and documented as "on-pad (via-in-pad) offsets
along the escape axis, 0 == via centred on the pad". Its increment is
``step = max(stagger, grid_step, 0.05)`` where ``stagger`` is the INTER-NET
centre-to-centre a via needs from a DIFFERENT net's via at this pitch -- so on a
fine-pitch part it exceeds the pad and only ``k = 0`` is guaranteed on it.

The issue asks for a corpus A/B before the ladder changes, because those long
offsets are load-bearing for escape counts. This is that A/B. Three arms, one
process each (module knobs are read once at import, so in-process arms poison
each other):

    full     today's behaviour, k = 0..8 either side, no limit
    pad      |k*step| <= pad_width/2          -- the via CENTRE stays on the pad
    barrel   |k*step| <= pad_width/2 - via/2  -- the whole BARREL stays on it

Grade paired and directional, per CLAUDE.md's A/B doctrine: escapes are the
headline (`escaped` up, `failed` down), `drc_grazes.total` is the guard.

    python3 tests/sweep_846_onpad_ladder.py [--out results.json]

Not named test_* on purpose: it is an instrument, not a gate, and run_all.py
should not spend its budget on it.
"""
import argparse
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

# board, component, net filter, layer, track, clearance, via, drill
CASES = [
    ('tigard', 'U3', None, 'F.Cu', 0.1, 0.10, 0.45, 0.25),
    ('qfn_underpad_coupling', 'U1', 'SIG*', 'F.Cu', 0.1, 0.12, 0.45, 0.20),
    ('qfn_diffpair_escape', 'U1', 'DP1*', 'F.Cu', 0.1, 0.15, 0.45, 0.20),
    ('qfn_interior_pads', 'U1', None, 'F.Cu', 0.1, 0.10, 0.45, 0.20),
    # The board the issue measured on, and the one that discriminates HARDEST:
    # not one of its selected offsets overlaps a pad, yet confining the ladder
    # to the pad costs it 5 escapes. That is the finding -- the "on-pad" ladder
    # earns its keep doing OFF-pad work.
    ('routed_output', 'U2', 'Net-(U2A-*)', 'B.Cu', 0.1, 0.10, 0.45, 0.25),
]
ARMS = ('full', 'pad', 'barrel')


def one(case, arm, workdir):
    board, ref, nets, layer, tw, clr, vs, vd = case
    src = os.path.join(ROOT, 'kicad_files', board + '.kicad_pcb')
    if not os.path.exists(src):
        return None
    out = os.path.join(workdir, f'{board}_{arm}.kicad_pcb')
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_router', 'qfn_fanout.py'), src,
            '--component', ref, '--output', out,
            '--escape-method', 'underpad', '--allow-via-in-pad',
            '--layer', layer, '--width', str(tw), '--clearance', str(clr),
            '--via-size', str(vs), '--via-drill', str(vd),
            '--grid-step', '0.05']
    if nets:
        argv += ['--nets', nets]
    env = dict(os.environ)
    env['KICAD_QFN_ONPAD_REACH'] = arm
    env['PYTHONPATH'] = os.pathsep.join(
        [ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools')])
    p = subprocess.run(argv, capture_output=True, text=True, env=env, cwd=ROOT)
    row = None
    for line in p.stdout.splitlines():
        if line.startswith('JSON_SUMMARY: '):
            row = json.loads(line[len('JSON_SUMMARY: '):])
    if row is None:
        return {'board': board, 'arm': arm, 'error': (p.stderr or
                                                      p.stdout)[-400:]}
    return {'board': board, 'arm': arm,
            'escaped': row.get('escaped'), 'failed': row.get('failed'),
            'via_in_pad': row.get('via_in_pad'),
            'via_in_pad_offcentre': row.get('via_in_pad_offcentre'),
            'max_stub_mm': row.get('max_stub_mm'),
            'grazes': (row.get('drc_grazes') or {}).get('total')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=None)
    ap.add_argument('--workdir', default=None)
    args = ap.parse_args()
    workdir = args.workdir or os.path.join(ROOT, '.sweep846')
    os.makedirs(workdir, exist_ok=True)

    rows = []
    for case in CASES:
        for arm in ARMS:
            r = one(case, arm, workdir)
            if r is None:
                print(f'  SKIP {case[0]} (board absent)')
                continue
            rows.append(r)
            if 'error' in r:
                print(f"  {r['board']:<22} {arm:<7} ERROR {r['error'][:120]}")
            else:
                print(f"  {r['board']:<22} {arm:<7} escaped={r['escaped']:<4}"
                      f" failed={r['failed']:<4} via_in_pad={r['via_in_pad']}"
                      f" offcentre={r['via_in_pad_offcentre']}"
                      f" max_stub={r['max_stub_mm']} grazes={r['grazes']}")

    print('\n=== paired, vs the `full` arm ===')
    by = {}
    for r in rows:
        by.setdefault(r['board'], {})[r['arm']] = r
    verdict = {}
    for board, arms in sorted(by.items()):
        base = arms.get('full')
        if not base or 'error' in base:
            continue
        for arm in ('pad', 'barrel'):
            cur = arms.get(arm)
            if not cur or 'error' in cur:
                continue
            d_esc = (cur['escaped'] or 0) - (base['escaped'] or 0)
            d_gra = (cur['grazes'] or 0) - (base['grazes'] or 0)
            mark = 'same' if d_esc == 0 else ('BETTER' if d_esc > 0
                                              else 'WORSE')
            verdict.setdefault(arm, []).append(d_esc)
            print(f'  {board:<22} {arm:<7} escaped {d_esc:+d}  grazes '
                  f'{d_gra:+d}   {mark}')
    print()
    for arm, deltas in sorted(verdict.items()):
        worse = sum(1 for d in deltas if d < 0)
        better = sum(1 for d in deltas if d > 0)
        print(f'  {arm}: {better} improved, {worse} regressed, '
              f'{len(deltas) - better - worse} unchanged over {len(deltas)} '
              f'boards -- '
              + ('ADOPTABLE' if worse == 0 and better >= max(1, len(deltas) - 1)
                 else 'NOT ADOPTABLE (the doctrine wants improve on >= N-1, '
                      'regress on none)'))

    if args.out:
        with open(args.out, 'w') as f:
            json.dump(rows, f, indent=1)
        print('\nwrote', args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
