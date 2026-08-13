#!/usr/bin/env python3
"""check_assembly must grade a coincident-origin stack NOT BUILDABLE.

Run 19, measured twice: SW17+SW34+REF_PUCK_R all at (128.399, 61.001) graded
`buildable (blocking 0)` -- the pair currency counts pad INTERSECTIONS, and
the rotated pads happened to interleave. A stack of parts at one origin is
unbuildable whatever the pads do, so it is now its own blocking channel:
`coincident_origin_groups` / `coincident_origins` in the JSON, COINCIDENT
ORIGINS in the findings, and the verdict/exit-code flip.

Three cases, on one fixture board:

  * a stack whose pads DON'T intersect (the run-19 shape: blocking stays 0)
    flips the verdict through the new channel alone -- exit 4, NOT BUILDABLE,
    and the JSON group names both refs;
  * the unmodified board still grades clean, exit 0;
  * a marker pair (two mounting holes moved onto one point) is exonerated by
    the stack channel -- markers are co-located by design (mouse-bites,
    fiducials), exactly placement_state's `_suspect_in` prior art.

Run: python3 -X utf8 tests/test_assembly_coincident_stack.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ.setdefault('KRT_NO_BANNER', '1')

FIXTURE = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
TOOL = os.path.join(ROOT, 'py_tools', 'check_assembly.py')

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def run_tool(board, json_out):
    r = subprocess.run([sys.executable, '-X', 'utf8', TOOL, board,
                        '--json', json_out],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600, cwd=ROOT)
    doc = None
    if os.path.isfile(json_out):
        with open(json_out, encoding='utf-8') as f:
            doc = json.load(f)
    return r.returncode, r.stdout + r.stderr, doc


def rewrite(text, old_at, new_at, out_path):
    """Move one footprint by rewriting its `(at x y ...)` line, and REFUSE to
    write a board the substitution did not change -- a drifted fixture must
    fail loudly here, not downstream as a mystery-clean grade."""
    moved = text.replace(old_at, new_at, 1)
    if moved == text:
        raise AssertionError(f'fixture drift: {old_at!r} not found in '
                             f'{FIXTURE}')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(moved)
    return out_path


def main():
    text = open(FIXTURE, encoding='utf-8').read()
    tmp = tempfile.mkdtemp(prefix='coincident_stack_')
    try:
        print('a stack whose pads INTERLEAVE is caught by the new channel')
        # C1 (an 0603) moved onto C3's origin (an 8x10 elec cap whose pads sit
        # ~3mm out): no pad intersects, so `blocking` stays 0 -- exactly the
        # run-19 shape the pair currency graded `buildable (blocking 0)`.
        board = rewrite(text, '(at 200.66 34.29 90)', '(at 144.78 31.75 90)',
                        os.path.join(tmp, 'stacked.kicad_pcb'))
        code, out, doc = run_tool(board, os.path.join(tmp, 'stacked.json'))
        check('exit code 4', code == 4, f'exit {code}\n{out[-600:]}')
        check('verdict NOT BUILDABLE', 'NOT BUILDABLE' in out, out[-600:])
        check('...through the stack channel alone (blocking 0)',
              doc is not None and doc.get('blocking') == 0,
              str(doc and doc.get('blocking')))
        check('JSON buildable False', doc is not None
              and doc.get('buildable') is False, str(doc and doc.get('buildable')))
        groups = (doc or {}).get('coincident_origin_groups') or []
        check('one JSON group naming both refs',
              len(groups) == 1 and sorted(groups[0]['refs']) == ['C1', 'C3'],
              str(groups))
        check('scalar coincident_origins is the group count',
              (doc or {}).get('coincident_origins') == 1,
              str((doc or {}).get('coincident_origins')))
        check('the findings block prints the group',
              'COINCIDENT ORIGINS' in out and '@ (144.78, 31.75)' in out,
              out[-600:])

        print('the unmodified board still grades clean')
        clean = os.path.join(tmp, 'clean.kicad_pcb')
        shutil.copyfile(FIXTURE, clean)
        code, out, doc = run_tool(clean, os.path.join(tmp, 'clean.json'))
        check('exit code 0', code == 0, f'exit {code}\n{out[-600:]}')
        check('buildable, zero groups', doc is not None
              and doc.get('buildable') is True
              and doc.get('coincident_origins') == 0
              and doc.get('coincident_origin_groups') == [], str(doc)[:300])

        print('a marker pair at one point is exonerated by the stack channel')
        # H1 moved onto H2 (both M3 mounting holes): co-located markers are
        # by-design (mouse-bites, fiducials) and must NOT flag as a stack.
        # Their annular pads DO physically intersect, so the OLD blocking
        # channel still fires -- correctly, and that is not under test here.
        board = rewrite(text, '(at 28.702 28.702)', '(at 220.218 28.702)',
                        os.path.join(tmp, 'markers.kicad_pcb'))
        code, out, doc = run_tool(board, os.path.join(tmp, 'markers.json'))
        check('no COINCIDENT ORIGINS finding', 'COINCIDENT ORIGINS' not in out,
              out[-600:])
        check('zero groups in the JSON', doc is not None
              and doc.get('coincident_origins') == 0
              and doc.get('coincident_origin_groups') == [],
              str(doc and doc.get('coincident_origin_groups')))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
