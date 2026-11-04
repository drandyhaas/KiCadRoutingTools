"""#1493: route.py writes the copper-to-hole floor it HELD, not the copper
clearance -- route.py's half of #1369 (c).

route.py passed no hole clearance to its #650 in-run floor sync or to its
writeback, so a board declaring min_hole_clearance 0.25 shipped at 0.2 after a
route at 0.2 although no copper needed it, outside the design_rules ledger
(--strict-sizes exit 0) and with a banner calling it "unchanged by this
step". Now:

  1. route.py on flat_hierarchy (min_hole_clearance 0.25) keeps 0.25.
  2. a floor written UNDER the one held is a design_rules ledger row
     (`hole_clearance`), once: the in-run sync passes ledger=False.
  3. the writeback banner credits a floor the in-run sync already lowered
     to this step ("0.25 -> 0.2"), not "unchanged by this step".

    python3 tests/test_1493_route_hole_floor.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

import fix_kicad_drc_settings as fk  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def rules(pcb):
    with open(os.path.splitext(pcb)[0] + '.kicad_pro') as fh:
        return json.load(fh)['board']['design_settings']['rules']


def route(td):
    print('1. route.py keeps the hole floor it held')
    src = os.path.join(td, 'in.kicad_pcb')
    out = os.path.join(td, 'r.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'copy_board.py'),
                    os.path.join('kicad_files', 'flat_hierarchy.kicad_pcb'), src],
                   capture_output=True, cwd=REPO)
    run_utils.evidence(src)
    check('the input declares 0.25', rules(src).get('min_hole_clearance') == 0.25)
    r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'route.py'),
                        src, out, '--nets', '/pic_programmer/*', 'Net-(D*', '--strict-sizes'],
                       capture_output=True, text=True, cwd=REPO)
    run_utils.evidence(out)
    got = rules(out).get('min_hole_clearance')
    check('routed at 0.2, the output keeps min_hole_clearance 0.25', got == 0.25, f'{got}')
    check('...and --strict-sizes exits 0: nothing was narrowed', r.returncode == 0,
          f'rc {r.returncode}')


def ledger():
    print('2. a floor written under the one held is ledgered, once')
    import fab_tiers
    saved = (fk.drawn_copper_items, fk.step_copper_to_hole)
    import obstacle_map
    saved_d = obstacle_map.declared_hole_clearance
    try:
        obstacle_map.declared_hole_clearance = lambda pcb, path: (0.25, 'test')
        fk.drawn_copper_items = lambda a, b: (object(), [], [])
        fk.step_copper_to_hole = lambda out, segs, vias, held, clr: 0.22
        for flag, want in ((True, 1), (False, 0)):
            fab_tiers._LEDGER['narrowed'].clear()
            with contextlib.redirect_stdout(io.StringIO()):
                v = fk.step_hole_clearance('a.kicad_pcb', 'b.kicad_pcb', 0.2, ledger=flag)
            rows = [r for r in fab_tiers._LEDGER['narrowed'] if r['kind'] == 'hole_clearance']
            check(f'ledger={flag}: floor 0.22, {want} hole_clearance row(s)',
                  v == 0.22 and len(rows) == want
                  and all((r['requested'], r['delivered']) == (0.25, 0.22) for r in rows),
                  json.dumps(rows))
    finally:
        fk.drawn_copper_items, fk.step_copper_to_hole = saved
        obstacle_map.declared_hole_clearance = saved_d
        fab_tiers._LEDGER['narrowed'].clear()


def banner(td):
    print('3. the banner credits an in-run lowering to this step')
    src = os.path.join(td, 'b_in.kicad_pcb')
    out = os.path.join(td, 'b_out.kicad_pcb')
    for dst in (src, out):
        subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'copy_board.py'),
                        os.path.join('kicad_files', 'flat_hierarchy.kicad_pcb'), dst],
                       capture_output=True, cwd=REPO)
        run_utils.evidence(dst)
    # the #650 in-run sync, as route.py runs it mid-run (it seeds the
    # floor origin, then lowers the output's project to 0.2)
    with contextlib.redirect_stdout(io.StringIO()):
        fk.apply_routed_floors(out, clearance=0.2, hole_clearance=0.2)
    check('the in-run sync lowered the output to 0.2',
          rules(out).get('min_hole_clearance') == 0.2, json.dumps(rules(out)))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fk.fix_project_for_output(out, input_pcb=src, clearance=0.2, hole_clearance=0.2)
    txt = buf.getvalue()
    line = next((ln for ln in txt.splitlines() if 'copper-to-hole' in ln.lower()
                 or 'hole clearance' in ln.lower()), '')
    check('reads 0.25 -> 0.2, not "unchanged by this step"',
          '0.25 -> 0.2' in line and 'unchanged by this step' not in line, line or txt[-600:])


def main():
    td = tempfile.mkdtemp(prefix='krt1493_')
    try:
        route(td)
        ledger()
        banner(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
