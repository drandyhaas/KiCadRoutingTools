#!/usr/bin/env python3
"""#1216: --routing-clearance-margin says what it does -- diff-pair via
spacing only -- everywhere it is offered.

route.py's help called it a "Multiplier on track-via clearance", docs listed
it as a general algorithm option, and the single-ended GUI tab offered it as
"Extra clearance margin multiplier for safety". Only diff_pair_routing.py
reads it (the P/N via offset and the centerline's via keep-out), so a campaign
run spent 10 minutes at 1.7 for byte-identical single-ended copper.

Checks:
  1. Readers: `config.routing_clearance_margin` is read only in
     diff_pair_routing.py (argparse `args.` and the GUI's widget are the
     plumbing, not readers). A new reader elsewhere means the
     texts below are stale -- update them with it.
  2. route.py --help, route_diff.py --help and the GUI tooltip say so.
  3. route.py on tracked splitflap_driver at 1.0 and 2.0: the run with 2.0
     prints the "diff-pair via spacing only" note, and both lay identical
     segments and vias.

    python3 tests/test_1216_clearance_margin_scope.py
"""
import ast
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def readers():
    out = set()
    for tree in ('py_router', 'py_tools', 'kicad_routing_plugin'):
        for dirpath, _d, files in os.walk(os.path.join(ROOT, tree)):
            for fn in files:
                if not fn.endswith('.py'):
                    continue
                path = os.path.join(dirpath, fn)
                try:
                    mod = ast.parse(open(path, encoding='utf-8').read())
                except SyntaxError:
                    continue
                for node in ast.walk(mod):
                    if (isinstance(node, ast.Attribute)
                            and node.attr == 'routing_clearance_margin'
                            and isinstance(node.ctx, ast.Load)
                            and not (isinstance(node.value, ast.Name)
                                     and node.value.id in ('args', 'self',
                                                           'dialog'))):
                        out.add(os.path.relpath(path, ROOT))
    return out


def copper(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        p = parse_kicad_pcb(path)
    r = lambda v: round(v, 4)
    return (sorted((r(s.start_x), r(s.start_y), r(s.end_x), r(s.end_y), s.layer,
                    s.net_id, r(s.width)) for s in p.segments),
            sorted((r(v.x), r(v.y), v.net_id) for v in p.vias))


def main():
    # 1. Readers.
    got = readers()
    check('only diff_pair_routing reads the margin',
          got == {os.path.join('py_router', 'diff_pair_routing.py')}, str(sorted(got)))

    # 2. Texts.
    for script in ('route.py', 'route_diff.py'):
        r = subprocess.run([sys.executable, '-X', 'utf8', f'py_router/{script}', '--help'],
                           cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        flat = ' '.join(r.stdout.split())
        i = flat.rfind('--routing-clearance-margin')     # the option, not usage
        frag = flat[i:i + 330] if i >= 0 else ''
        check(f'{script} --help names the P/N via offset', 'P/N via offset' in frag, frag)
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', '--help'],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    flat = ' '.join(r.stdout.split())
    check('route.py --help says single-ended copper ignores it',
          'Diff pairs only' in flat and 'Single-ended tracks and vias do not read it' in flat)
    # ipc-migration: swig_gui.py is routing_dialog.py here.
    gui = open(os.path.join(ROOT, 'kicad_routing_plugin', 'routing_dialog.py'),
               encoding='utf-8').read()
    line = next((l for l in gui.splitlines() if "('routing_clearance_margin'," in l), '')
    check('the GUI tooltip says diff pairs only', 'Diff pairs only' in line, line.strip()[:160])

    # 3. Behaviour.
    tmp = tempfile.mkdtemp(prefix='t1216_')
    try:
        from copy_board import copy_board
        outs, logs = {}, {}
        for m in ('1.0', '2.0'):
            inp = os.path.join(tmp, f'in_{m}.kicad_pcb')
            with contextlib.redirect_stdout(io.StringIO()):
                copy_board(BOARD, inp)
            out = os.path.join(tmp, f'out_{m}.kicad_pcb')
            r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py',
                                inp, out, '--nets', '*',
                                '--routing-clearance-margin', m],
                               cwd=ROOT, capture_output=True, text=True,
                               encoding='utf-8', errors='replace')
            outs[m], logs[m] = out, r.stdout + r.stderr
            check(f'route.py at {m} wrote a board', os.path.isfile(out),
                  logs[m][-300:] if not os.path.isfile(out) else '')
        note = 'diff-pair via spacing only'
        check('the 2.0 run says what the margin reaches',
              note in logs['2.0'] and note not in logs['1.0'])
        if all(os.path.isfile(o) for o in outs.values()):
            a, b = copper(outs['1.0']), copper(outs['2.0'])
            check('single-ended copper is identical at 1.0 and 2.0', a == b,
                  f'{len(a[0])}/{len(b[0])} segments, {len(a[1])}/{len(b[1])} vias')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
