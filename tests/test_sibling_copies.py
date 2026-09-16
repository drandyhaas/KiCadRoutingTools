#!/usr/bin/env python3
"""A board's project siblings reach every board that is routed or delivered,
and carrying them in place is not a crash.

Two defects in one helper's reach, both #441's hazard -- a board without its
`.kicad_pro` resolves its DRC floor and net classes from the stock defaults:

  1. IN PLACE. `portfolio.copy_siblings(X, X)` handed each sibling to
     `shutil.copyfile` as its own destination, which raises SameFileError
     (PermissionError through copy2 on Windows). Every in-place run with a
     sibling crashed after writing its board: `place_seed X X --repair` or
     `--reseat` with nothing to move, the seed path, `place_optimize X X`,
     `beautify_labels X X`.
  2. ROUNDS AND PROBES. `place_route_loop` copied the siblings to round 0 only;
     every later candidate was written without them and ROUTED that way, so
     rounds >= 1 were judged at the stock floor (measured: a board's 0.4 USB
     class became the 0.25 fallback and the round that bought 0 failures with
     it was accepted), and an accepted round delivered OUT with no project.
     `converge poses --route` routed its probe candidates the same way.

Each case drives the real code path and checks the files, not a return code.
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_tools', 'py_placer', 'tests'):
    _q = os.path.join(ROOT, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

EP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
SF = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


if not (os.path.isfile(EP) and os.path.isfile(SF)):
    print('SKIP: fixture missing')
    sys.exit(77)

import subprocess                                             # noqa: E402
from copy_board import SIBLING_EXTS                           # noqa: E402
from placement.portfolio import copy_siblings                 # noqa: E402

#: A project whose Default class is NOT the stock 0.25 fallback, and a rules
#: file, so a board that lost them is a board with different rules.
PRO = json.dumps({
    'board': {'design_settings': {'rules': {'min_clearance': 0.15,
                                            'min_copper_edge_clearance': 0.3},
                                  'rule_severities': {}}},
    'meta': {'filename': 'board.kicad_pro', 'version': 1},
    'net_settings': {'classes': [{'name': 'Default', 'clearance': 0.15,
                                  'track_width': 0.2, 'via_diameter': 0.6,
                                  'via_drill': 0.3}],
                     'meta': {'version': 3}}}, indent=2)
DRU = '(version 1)\n(rule inner_tight (layer inner) (constraint clearance (min 0.12mm)))\n'


def with_siblings(src, d, name='board.kicad_pcb'):
    """Copy `src` into `d` with a synthesised .kicad_pro and .kicad_dru."""
    os.makedirs(d, exist_ok=True)
    b = os.path.join(d, name)
    shutil.copyfile(src, b)
    base = os.path.splitext(b)[0]
    with open(base + '.kicad_pro', 'w', encoding='utf-8') as f:
        f.write(PRO)
    with open(base + '.kicad_dru', 'w', encoding='utf-8') as f:
        f.write(DRU)
    return b


def siblings_of(board):
    """{ext: bytes} for every sibling present next to `board`."""
    base = os.path.splitext(board)[0]
    out = {}
    for ext in SIBLING_EXTS:
        if os.path.isfile(base + ext):
            with open(base + ext, 'rb') as f:
                out[ext] = f.read()
    return out


# ==========================================================================
# 1. the helper
# ==========================================================================
print('1. copy_siblings')
d = tempfile.mkdtemp(prefix='sib_')
X = with_siblings(SF, d)
before = siblings_of(X)
try:
    copy_siblings(X, X)
    err = None
except Exception as e:                                        # noqa: BLE001
    err = e
check('in place (the same board) is not an error', err is None, repr(err))
check('...and leaves every sibling byte-identical',
      siblings_of(X) == before and set(before) == {'.kicad_pro', '.kicad_dru'},
      str(sorted(siblings_of(X))))

swapped = os.path.join(os.path.dirname(X), os.path.basename(X).upper())
if os.path.exists(swapped):               # a case-insensitive disk
    try:
        copy_siblings(X, swapped)
        err = None
    except Exception as e:                                    # noqa: BLE001
        err = e
    check('a case-only respelling of the same board is in place too', err is None,
          repr(err))

Y = os.path.join(tempfile.mkdtemp(prefix='sib_y_'), 'other.kicad_pcb')
shutil.copyfile(X, Y)
copy_siblings(X, Y)
check('to another board it still carries every sibling',
      siblings_of(Y) == before, str(sorted(siblings_of(Y))))


# ==========================================================================
# 2. the CLIs that crashed in place
# ==========================================================================
print('2. in-place CLIs')
from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement.floorplan import emit_intent                   # noqa: E402

PY = [sys.executable, '-X', 'utf8', '-B']


def in_place(tag, script, extra, want_codes=(0,)):
    d = tempfile.mkdtemp(prefix='sib_cli_')
    X = with_siblings(EP, d)
    intent = os.path.join(d, 'intent.json')
    with open(intent, 'w', encoding='utf-8') as f:
        json.dump(emit_intent(parse_kicad_pcb(EP), EP), f)
    before = siblings_of(X)
    argv = PY + [os.path.join(ROOT, 'py_placer', script), X, X] + [
        a.replace('@INTENT@', intent) for a in extra]
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=1200)
    out = (r.stdout or '') + (r.stderr or '')
    # Exit 4 is a place_seed VERDICT (a violation it reports), not a crash;
    # what must never appear is the traceback the in-place copy produced.
    check(f'{tag}: completes in place with a sibling project',
          r.returncode in want_codes and 'Traceback' not in out,
          f'rc={r.returncode}' + ('' if r.returncode in want_codes else f' {out[-400:]}'))
    check(f'{tag}: ...and reports its result', 'JSON_SUMMARY' in (r.stdout or ''))
    check(f'{tag}: ...and the siblings are untouched', siblings_of(X) == before)


in_place('place_seed --repair, nothing to move', 'place_seed.py',
         ['--intent', '@INTENT@', '--repair'], want_codes=(0, 4))
in_place('place_seed --reseat, nothing to move', 'place_seed.py',
         ['--intent', '@INTENT@', '--reseat'], want_codes=(0, 4))
in_place('place_seed --force (the seed path)', 'place_seed.py',
         ['--intent', '@INTENT@', '--force', '--no-polish'], want_codes=(0, 4))
in_place('place_optimize', 'place_optimize.py', ['--max-passes', '1'])
in_place('beautify_labels', 'beautify_labels.py', [])


# ==========================================================================
# 3. place_route_loop: every routed round, every quenched board, and OUT
# ==========================================================================
print('3. place_route_loop rounds')
import place_route_loop as prl                                # noqa: E402
from test_554_loop_relocate import _board as _reloc_board, _proposal  # noqa: E402


def run_loop(relocate):
    src, _w = _reloc_board()
    d = tempfile.mkdtemp(prefix='sib_loop_')
    board = with_siblings(src, d)
    os.unlink(src)
    want = siblings_of(board)
    work = os.path.join(d, 'work')
    OUT = os.path.join(d, 'out.kicad_pcb')
    seen = {'route': [], 'quench': []}
    fails = iter([3, 2, 2, 2])

    def fake_route(pcb_file, routed_file, route_args, log_file, **kw):
        seen['route'].append((os.path.basename(pcb_file), siblings_of(pcb_file)))
        shutil.copyfile(pcb_file, routed_file)
        return {'failures': next(fails), 'failed_nets': ['NA'], 'blockers': [],
                'iterations': 1000, 'vias': 0, 'blocker_report': None}

    def fake_quench(pcb_data, **kw):
        pf = kw.get('pcb_file')
        seen['quench'].append((os.path.basename(pf or ''), siblings_of(pf) if pf else {}))
        return [{'reference': 'R1', 'new_x': 161.0 + len(seen['quench']),
                 'new_y': 100.0, 'new_rotation': 0.0}]

    argv = ['place_route_loop.py', board, OUT, '--route-args', '--nets "*"',
            '--rounds', '2', '--max-displacement', '3.0', '--work-dir', work,
            '--no-movie']
    if relocate:
        argv += ['--relocate', '--group-by', 'decap']
    saved = (prl.quench, prl.run_route, prl.relocate_round, sys.argv)
    prl.quench, prl.run_route, sys.argv = fake_quench, fake_route, argv
    if relocate:
        prl.relocate_round = lambda *a, **k: _proposal()
    try:
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            prl.main()
    finally:
        prl.quench, prl.run_route, prl.relocate_round, sys.argv = saved
    relocated = sorted(n for n in os.listdir(work) if n.endswith('_relocated.kicad_pcb'))
    return want, seen, OUT, work, relocated


for relocate in (False, True):
    tag = 'with --relocate' if relocate else 'plain'
    want, seen, OUT, work, relocated = run_loop(relocate)
    check(f'{tag}: rounds 0-2 were routed (round 1 accepted, round 2 rejected)',
          [n for n, _s in seen['route']] == ['loop_round0.kicad_pcb',
                                             'loop_round1.kicad_pcb',
                                             'loop_round2.kicad_pcb'],
          str([n for n, _s in seen['route']]))
    bad = [n for n, s in seen['route'] if s != want]
    check(f'{tag}: every routed board carries the input\'s project and rules',
          not bad, f'missing on {bad}')
    bad = [n for n, s in seen['quench'] if s != want]
    check(f'{tag}: every quenched board carries them, after an accept too',
          len(seen['quench']) == 2 and not bad, f'{len(seen["quench"])} quench(es), missing on {bad}')
    if relocate:
        bad = [n for n in relocated if siblings_of(os.path.join(work, n)) != want]
        check(f'{tag}: every relocation board carries them, after an accept too',
              len(relocated) == 2 and not bad, f'{relocated}, missing on {bad}')
    check(f'{tag}: the delivered board carries them after an accepted round',
          siblings_of(OUT) == want, str(sorted(siblings_of(OUT))))


# ==========================================================================
# 4. converge poses --route: the probe candidates are routed with the project
# ==========================================================================
print('4. converge poses --route')
import converge                                               # noqa: E402

d = tempfile.mkdtemp(prefix='sib_conv_')
board = with_siblings(SF, d)
want = siblings_of(board)
probed = []


def fake_scoped_route(cand, nets, **kw):
    probed.append(siblings_of(cand))
    return {'returncode': 0, 'summary': {'failed_single': [], 'total_iterations': 1,
                                         'total_vias': 0},
            'board': cand, 'json': '', 'argv': [], 'stdout_tail': ''}


_real = converge.scoped_route
converge.scoped_route = fake_scoped_route
try:
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        rc = converge.main(['poses', board, '--ref', 'C1', '--radius', '0.5', '--step', '0.5',
                            '--limit', '2', '--route', '--route-top', '2',
                            '--affected', '/A'])
finally:
    converge.scoped_route = _real
check('converge probed at least one candidate', len(probed) >= 1 and rc == 0,
      f'rc={rc} probes={len(probed)}')
check('...and every probe candidate carried the board\'s project and rules',
      probed and all(p == want for p in probed),
      str([sorted(p) for p in probed]))


print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
