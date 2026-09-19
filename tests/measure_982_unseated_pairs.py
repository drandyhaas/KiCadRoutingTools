#!/usr/bin/env python3
"""How many charged pad conflicts are against a part the seed could not seat?

#982. `place_seed` used to charge `pad_conflicts_seeded` for a pair between a
part it moved and a part it could NOT seat, which was written at the pose it
came in with. This script is the measurement behind that claim and behind the
numbers in `place_seed.split_pad_pairs`: it prepares each board the way the
issue does, runs the seed, and counts the pairs on each side of that line.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it seeds real
boards and takes minutes per arm.

    python3 -X utf8 tests/measure_982_unseated_pairs.py --boards ulx3s --seeds 0-9
    python3 -X utf8 tests/measure_982_unseated_pairs.py            # the table below
    python3 -X utf8 tests/measure_982_unseated_pairs.py --out rows.json

THE RECIPE MATTERS, and these numbers do not reproduce without it. Each board
is copied with its siblings, STRIPPED OF COPPER when it carries any
(`tests/stress/strip_copper_only.py` -- `place_seed` refuses a board with
copper), and given its own `floorplan.emit_intent`; the run is
`--force --group-by kicad,sheet --clearance 0.2 --board-edge-clearance 0.55
--no-polish`, `PYTHONHASHSEED=0`. A board prepared some other way -- the
unstripped file, or an intent from `check_floorplan` with different flags --
seeds differently and reports different pairs, which is how one review read
this table as unreproducible. The esp_prog arm is the run-27 fixture under
`tests/fixtures/975/` when that directory is present, with no clearance flags,
and is skipped when it is not.

THE MEASURED RESULT is recorded here from the run, never predicted.

MEASURED on Windows, 2026-09-17, on this branch (so the pairs are already in
the `pad_conflicts_unseated` bucket; `charged` below is both buckets together,
which is what the old accounting put in `pad_conflicts_seeded`):

  board                       unseated     charged  of those, vs an unseated part
  ulx3s, seeds 0..9           2 (J1,J2)          5  5
  rp2350_fpga_eensy_prePlane  1 (U6)            16  15
  orangecrab_ext_pll          4 (H1..H4)         3  2
  tigard                      2 (H1,H4)          0  0
  splitflap_driver            none               0  0
  watchy                      none               0  0
  interf_u_unrouted           none               0  0
  esp_prog_run27              SKIPPED -- the run-27 fixture is not in this tree

Seed 0 for every board but ulx3s. Over 7 boards at seed 0: 19 charged pairs,
17 of them against a part the seed could not seat.

ulx3s per seed, which is the issue's own table, reproduced: seed 1 `H4-J1
0.400`; seed 7 `H1-J2 0.191`, `H2-J1 0.1558`, `H3-J2 0.1466`; seed 8 `H1-J1
0.1558`; nothing on the other seven. rc 4 in every row, for the unseated parts.

rp2350 is the worst case: fifteen of its sixteen charged pairs are the ONE
unseated U6, whose 75 pads sit at the designer's pose. Only `J1-J3 0.400` there
is between two parts the seed placed, and on orangecrab only `J1-J5 0.400`.

The instability the issue reports reproduces too: at seed 0, ulx3s has no pair
at all, and `--sham 0.386` -- which displaces AUDIO1's first seat by the same
0.386 mm #975's board-edge fix does, on an unmutated engine -- produces
`H4-J2 0.191`. That is one pair appearing because a mounting hole landed on an
unseated connector, with nothing about the seed itself any different.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in (_ROOT, os.path.join(_ROOT, 'py_router'), os.path.join(_ROOT, 'py_placer'),
           os.path.join(_ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

SEED = os.path.join(_ROOT, 'py_placer', 'place_seed.py')
STRIP = os.path.join(_TESTS, 'stress', 'strip_copper_only.py')
FIXTURE = os.path.join(_TESTS, 'fixtures', '975', 'esp_prog_run27')

BOARDS = ['ulx3s', 'rp2350_fpga_eensy_prePlane', 'orangecrab_ext_pll', 'tigard',
          'splitflap_driver', 'watchy', 'interf_u_unrouted', 'esp_prog_run27']

FLAGS = ['--group-by', 'kicad,sheet', '--clearance', '0.2',
         '--board-edge-clearance', '0.55']


def _env():
    return dict(os.environ, PYTHONHASHSEED='0', PYTHONIOENCODING='utf-8')


def _sha(path):
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def prepare(name, into):
    """The board, its intent and its flags -- the recipe in the docstring."""
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    from placement.placement_state import assess_placement
    from placement.writer import write_placed_output
    os.makedirs(into, exist_ok=True)
    board = os.path.join(into, 'board.kicad_pcb')
    if name == 'esp_prog_run27':
        if not os.path.isdir(FIXTURE):
            return None
        with open(os.path.join(FIXTURE, 'pile.json'), encoding='utf-8') as fh:
            pile = json.load(fh)
        x, y, rot = pile['pose']
        write_placed_output(os.path.join(_ROOT, pile['source']), board,
                            [{'reference': r, 'new_x': x, 'new_y': y,
                              'new_rotation': rot} for r in pile['refs']])
        shutil.copyfile(os.path.join(FIXTURE, 'board.kicad_pro'),
                        os.path.join(into, 'board.kicad_pro'))
        intent = os.path.join(into, 'intent.json')
        shutil.copyfile(os.path.join(FIXTURE, 'zone_plan.json'), intent)
        return {'board': board, 'intent': intent, 'flags': []}
    source = os.path.join(_ROOT, 'kicad_files', name + '.kicad_pcb')
    if not os.path.exists(source):
        return None
    shutil.copyfile(source, board)
    for ext in ('.kicad_pro', '.kicad_dru'):
        sibling = os.path.splitext(source)[0] + ext
        if os.path.exists(sibling):
            shutil.copyfile(sibling, os.path.splitext(board)[0] + ext)
    if assess_placement(parse_kicad_pcb(board)).has_copper:
        tmp = os.path.join(into, 'stripped.kicad_pcb')
        subprocess.run([sys.executable, '-X', 'utf8', STRIP, board, tmp],
                       check=True, capture_output=True, env=_env())
        os.replace(tmp, board)
    intent = os.path.join(into, 'intent.json')
    doc = floorplan.emit_intent(parse_kicad_pcb(board), board)
    with open(intent, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=1, sort_keys=True)
    return {'board': board, 'intent': intent, 'flags': list(FLAGS)}


def run_seed(spec, out, seed, sham=0.0):
    """One `place_seed`, returned as its JSON_SUMMARY plus the exit code.

    `sham` displaces the FIRST stage-1 pose of the board's first declared edge
    connector by that many mm in y, on the unmutated engine -- the control that
    shows the count moves with any displacement, not with a particular fix.
    """
    argv = [sys.executable, '-X', 'utf8']
    if sham:
        argv += [os.path.join(_TESTS, 'measure_982_sham.py'), str(sham)]
    argv += [SEED, spec['board'], out, '--intent', spec['intent'],
             '--force', '--seed', str(seed), '--no-polish'] + spec['flags']
    proc = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=_ROOT, env=_env())
    summary = None
    for line in (proc.stdout or '').splitlines():
        if line.startswith('JSON_SUMMARY: '):
            summary = json.loads(line[len('JSON_SUMMARY: '):])
    return proc.returncode, summary, (proc.stdout or '') + (proc.stderr or '')


def row(summary, rc):
    """The two numbers the issue is about, plus what they are made of."""
    if summary is None:
        return {'rc': rc, 'error': 'no JSON_SUMMARY'}
    unseated = set(summary.get('unseated_refs') or ())
    # Both spellings, so one script reads a base tree and a fixed one.
    pairs = list(summary.get('pad_conflicts_seeded_pairs') or ())
    pairs += list(summary.get('pad_conflicts_unseated_pairs') or ())
    return {
        'rc': rc,
        'unseated': sorted(unseated),
        'charged_pairs': len(pairs),
        'vs_unseated': sum(1 for p in pairs
                           if p[0] in unseated or p[1] in unseated),
        'pairs': [[p[0], p[1], round(float(p[2]), 4)] for p in pairs],
        'seeded_key': summary.get('pad_conflicts_seeded'),
        'unseated_key': summary.get('pad_conflicts_unseated'),
        'inherited_key': summary.get('pad_conflicts_inherited'),
        'total_key': summary.get('pad_conflicts_after'),
        'grade_errors': summary.get('grade_errors'),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--boards', default=','.join(BOARDS),
                    help='comma-separated; default every board in the table')
    ap.add_argument('--seeds', default='0',
                    help='"0", "0-9" or "0,3,7"')
    ap.add_argument('--sham', type=float, default=0.0,
                    help='displace the first edge connector by N mm in y')
    ap.add_argument('--out', default=None, help='write the rows as JSON')
    ap.add_argument('--workdir', default=None,
                    help='keep the prepared boards here instead of a tempdir')
    args = ap.parse_args()

    if '-' in args.seeds:
        lo, hi = args.seeds.split('-')
        seeds = list(range(int(lo), int(hi) + 1))
    else:
        seeds = [int(s) for s in args.seeds.split(',') if s != '']

    import tempfile
    tmp = args.workdir or tempfile.mkdtemp(prefix='m982_')
    os.makedirs(tmp, exist_ok=True)
    rows = {}
    for name in [b for b in args.boards.split(',') if b]:
        spec = prepare(name, os.path.join(tmp, name))
        if spec is None:
            print(f'{name:28s} SKIPPED (not in this tree)', flush=True)
            continue
        print(f'{name:28s} board {_sha(spec["board"])[:12]} '
              f'intent {_sha(spec["intent"])[:12]}', flush=True)
        for seed in seeds:
            out = os.path.join(tmp, name, f'seed{seed}.kicad_pcb')
            rc, summary, text = run_seed(spec, out, seed, args.sham)
            r = row(summary, rc)
            # Which ref the sham displaced, from the run's own line, so the
            # control names its actor instead of leaving it to be assumed.
            sham_line = [ln.strip() for ln in text.splitlines() if 'SHAM:' in ln]
            if sham_line:
                r['sham'] = sham_line[0]
                print(f'  {sham_line[0]}', flush=True)
            rows[f'{name}:{seed}'] = r
            print(f'  seed {seed}: rc {r.get("rc")} unseated '
                  f'{r.get("unseated")} charged {r.get("charged_pairs")} '
                  f'of which vs an unseated part {r.get("vs_unseated")} '
                  f'{r.get("pairs") or ""}', flush=True)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(rows, fh, indent=1, sort_keys=True)
        print(f'wrote {args.out}')
    tot = sum(r.get('charged_pairs') or 0 for r in rows.values())
    vs = sum(r.get('vs_unseated') or 0 for r in rows.values())
    print(f'\n{len(rows)} run(s): {tot} charged pair(s), {vs} of them against '
          f'a part the seed could not seat')
    return 0


if __name__ == '__main__':
    sys.exit(main())
