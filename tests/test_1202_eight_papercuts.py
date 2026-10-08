#!/usr/bin/env python3
"""#1202: eight papercuts from the fa10 campaign, and the comment's one ask.

  1. rank_rotations' control baseline counted seeds with parts still in the
     pile (ecc83's median control seed had C1 10 mm off the outline), and its
     lines printed no unseated count.
  2. rank_rotations ran its arms one after another (--jobs).
  3. beautify_labels never said it does not model silk graphics.
  4. "Override with --allow-routed" printed by tools that always override.
  5. board_score's poured_nets was [] on a fully connected board with a zone.
  6. make_film --from-ledger titled every film `_film`.
  7. converge record refused stop condition 3 beside a FAIL lens while its
     synonym STUCK passed.
  8. route.py's progress line hid nets sitting ripped in the reroute queue.
  +  board_score records per-tool wall seconds (`tool_seconds`).

    python3 tests/test_1202_eight_papercuts.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402

PILE = os.path.join(TESTS_DIR, 'fixtures', '959', 'run29_pile.kicad_pcb')
ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def rank_rotations_run(tmp, jobs):
    """rank_rotations main in-process, place_seed faked: every arm copies the
    pile; control seed 1 leaves a part in the pile and is the old median."""
    import rank_rotations as rr
    from placement import floorplan as fp
    ipath = os.path.join(tmp, 'pile.intent.json')
    if not os.path.isfile(ipath):
        doc = quiet(fp.emit_intent, parse_kicad_pcb(PILE), PILE, decaps_from=ESP)
        json.dump(doc, open(ipath, 'w'))
    live = {'n': 0, 'max': 0}
    lock = threading.Lock()

    def fake(input_file, intent, seed, out, *, ignore_nets=None, seed_args=None):
        with lock:
            live['n'] += 1
            live['max'] = max(live['max'], live['n'])
        time.sleep(0.15)
        quiet(copy_board, PILE, out)
        control = intent == ipath
        crossings = ({0: 2, 1: 4, 2: 12}[seed] if control
                     else 20 + seed + int('90' in os.path.basename(intent)))
        with lock:
            live['n'] -= 1
        return (SimpleNamespace(returncode=0, stdout='', stderr=''),
                {'crossings': crossings, 'hpwl': 100.0 + crossings,
                 'unseated': 1 if (control and seed == 1) else 0,
                 'grade_errors': 0})
    out_dir = os.path.join(tmp, f'rr_{jobs}')
    saved_run, saved_argv = rr.run_place_seed, sys.argv
    rr.run_place_seed = fake
    sys.argv = ['rank_rotations.py', PILE, '--intent', ipath, '--ref', 'U1',
                '--seeds', '0', '1', '2', '--rotations', '0', '90',
                '--out-dir', out_dir, '--jobs', str(jobs)]
    buf, err = io.StringIO(), io.StringIO()
    try:
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(err):
            rc = rr.main()
    except SystemExit as exc:             # an argparse refusal: say which
        rc = f'SystemExit({exc.code}): {err.getvalue().strip()[-200:]}'
    finally:
        rr.run_place_seed, sys.argv = saved_run, saved_argv
    path = os.path.join(out_dir, 'rotations.json')
    doc = json.load(open(path)) if os.path.isfile(path) else {'rows': []}
    return rc, doc, buf.getvalue(), live['max']


def main():
    tmp = tempfile.mkdtemp(prefix='t1202_')
    try:
        # 1./2. rank_rotations.
        rc1, doc1, out1, conc1 = rank_rotations_run(tmp, 1)
        check('precondition: rank_rotations ran (jobs 1)', rc1 == 0, str(rc1))
        ctl = doc1.get('control') or {}
        check('control lines print unseated',
              all('unseated' in l for l in out1.splitlines()
                  if l.strip().startswith('crossings')), out1[:300])
        check('the control median skips the seed with a part in the pile',
              ctl.get('crossings') == 7 and ctl.get('baseline_seeds') == [0, 2]
              and ctl.get('unseated_seeds') == [1], json.dumps(
                  {k: ctl.get(k) for k in ('crossings', 'baseline_seeds',
                                           'unseated_seeds')}))
        check('the verdict line names the excluded seed',
              'left parts in the pile' in out1)
        rc3, doc3, _out3, conc3 = rank_rotations_run(tmp, 3)
        check('precondition: rank_rotations ran (jobs 3)', rc3 == 0, str(rc3))
        strip = lambda d: [(r['rotation'], r['seed'], r['crossings'], r['unseated'])
                           for r in d['rows']]
        check('--jobs 3 ran arms at once', conc1 == 1 and conc3 > 1,
              f'max concurrent {conc1} -> {conc3}')
        check('--jobs 3 changes no result',
              rc1 == rc3 and strip(doc1) == strip(doc3)
              and doc1.get('ranking') == doc3.get('ranking'), str(strip(doc3)))

        # 3./4. beautify_labels on a routed board.
        ro = os.path.join(tmp, 'ro.kicad_pcb')
        quiet(copy_board, os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb'), ro)
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_placer/beautify_labels.py',
                            ro, os.path.join(tmp, 'ro_lab.kicad_pcb')],
                           cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        log = r.stdout + r.stderr
        summ = next((l for l in log.splitlines() if l.startswith('JSON_SUMMARY:')), '')
        check('beautify_labels says silk graphics are not checked',
              'silk graphics not checked' in log and summ
              and json.loads(summ.split(':', 1)[1]).get('silk_graphics_checked') is False,
              log[-300:])
        check('no "Override with --allow-routed" from a tool that overrides',
              'Override with --allow-routed' not in log
              and 'proceeding with the copper left as it is' in log)
        from placement import placement_state as ps
        st = ps.assess_placement(parse_kicad_pcb(ro), ro)
        check('format_report still advises the override when not given',
              'Override with --allow-routed' in ps.format_report(st, 't'))

        # 5. + tool_seconds: board_score.
        import board_score as bs
        saved = bs.run_tool
        bs.run_tool = lambda root, tool, *a: (0, 'ALL NETS FULLY CONNECTED\n')
        try:
            conn = bs.score_connectivity(ROOT, os.path.join(ROOT, 'kicad_files',
                                                            'sonde_u.kicad_pcb'))
        finally:
            bs.run_tool = saved
        check('a fully connected board still reports its poured nets',
              conn.get('poured_nets') == ['GND'] and conn.get('poured_nets_meaning'),
              str(conn.get('poured_nets')))
        bs.TOOL_SECONDS.clear()
        bs.run_tool(ROOT, 'check_connected.py', ro)
        check('run_tool records per-tool seconds',
              bs.TOOL_SECONDS.get('check_connected.py', 0) > 0, str(bs.TOOL_SECONDS))

        # 6. make_film --from-ledger titles the film after the ledger's run.
        import make_film
        import animate_route
        from board_store import BoardStore
        run_dir = os.path.join(tmp, '10_glasgow')
        os.makedirs(run_dir)
        store = BoardStore(os.path.join(run_dir, 'boards'))
        sha = store.put(ro)
        led = os.path.join(run_dir, 'ledger.jsonl')
        open(led, 'w').write(json.dumps({'iteration': 0, 'kind': 'placement',
                                         'result_sha': sha, 'accepted': True,
                                         'score': {'blocking': 3}}) + '\n')
        seen = {}
        real = animate_route.build_boards

        def spy(*a, **k):
            seen['title'] = k.get('title')
            return []
        animate_route.build_boards = spy
        try:
            quiet(make_film.main, ['--from-ledger', led, '-o',
                                   os.path.join(tmp, 'f.gif'), '--board-3d', '2d'])
        finally:
            animate_route.build_boards = real
        check('a ledger film is titled after its run directory',
              seen.get('title') == '10_glasgow', str(seen))

        # 7. converge.
        import converge
        check("stop condition 3 sits beside a FAIL lens like its synonym STUCK",
              '3' in converge.FAIL_COMPATIBLE_STOPS
              and 'STUCK' in converge.FAIL_COMPATIBLE_STOPS
              and '1' not in converge.FAIL_COMPATIBLE_STOPS)

        # 8. route.py's progress line counts ripped nets (splitflap rips one).
        sf = os.path.join(tmp, 'sf.kicad_pcb')
        quiet(copy_board, os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb'), sf)
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', sf,
                            os.path.join(tmp, 'sf_out.kicad_pcb'), '--nets', '*'],
                           cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        lines = [l for l in r.stdout.splitlines() if l.startswith('[') and 'Routing' in l]
        check('precondition: the run rips a net',
              any('REROUTE' in l for l in r.stdout.splitlines()))
        check('the main pass says how many nets wait ripped',
              any('ripped)]' in l for l in lines),
              next((l for l in lines if 'ripped' in l), lines[-1] if lines else ''))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
