#!/usr/bin/env python3
"""#1122 on the REAL headless dialog: an `optimize_caps` step whose intent
declares a cap's rotation holds it on the LIVE board, lands where the CLI's
`--intent` run lands, and a contradictory declaration stops the step.

    python3 tests/gui_parity/test_1122_cap_rotation_gui.py

(re-execs into KiCad's bundled python automatically, like its siblings)

ipc-migration: the board is the fake-IPC harness's (fake_ipc_board), the
dialog is routing_dialog's, and the moves are read back from the file the
fake flushes them to. The GUI builds PCBData from the board
(`build_pcb_data_from_board`), and
`declared_cap_rotations` resolves the intent's blocks against THAT data, so
the CLI test (tests/test_1122_fanout_declared_rotation.py) cannot speak for
this front. The fixture is the CLI test's: the U30 crop at clearance 0.1 with
a 0.6 / 2.0 mm budget, 2.0 mm decap limits and C24 declared at 270 -- the
configuration in which the run KEPT is the one without the decap gate.

Exits 2 when no python with pcbnew+wx exists: it is a killer gate for
tests/mutate_1120_1121_1122.py, and a gate that exits 0 when it did not run
reports every row it guards as SURVIVED.
"""
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
           key=path_version_key, reverse=True),
]
FIX = os.path.join(REPO, 'tests', 'fixtures', '1067', 'u30_crop.kicad_pcb')
TOOL = os.path.join(REPO, 'py_placer', 'place_fanout_clearance.py')
HOST_PYTHON = os.environ.get('KRT_HOST_PYTHON') or sys.executable
TIGHT = {'clearance': 0.1, 'cap_max_displacement': 0.6,
         'cap_max_displacement_cap': 2.0}


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx, kipy'],
                          capture_output=True).returncode == 0:
            env = dict(os.environ, KRT_HOST_PYTHON=sys.executable)
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:], env=env).returncode)
    print('NOT RUN: no python with pcbnew+wx+kipy found -- this gate measures the '
          'GUI cap pass with declared rotations, and nothing else does')
    sys.exit(2)


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
        import kipy  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    sys.path.insert(0, REPO)
    for sub in ('py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    sys.path.insert(0, os.path.dirname(REPO))
    sys.path.insert(0, os.path.join(REPO, 'kicad_routing_plugin'))
    from kicad_parser import parse_kicad_pcb
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from fake_ipc_board import install as _install_fake_board, build_pcb_data_like_ipc
    from kicad_routing_plugin.routing_dialog import RoutingDialog
    from kicad_routing_plugin import ai_plan
    from placement import floorplan as fp

    failures = []

    def check(name, ok, detail=''):
        if not ok:
            failures.append(name)
        print(('  PASS ' if ok else '  FAIL ') + name
              + (('  ' + detail) if detail and not ok else ''))

    td = tempfile.mkdtemp(prefix='t1122gui_')
    board = os.path.join(td, 'u30_crop.kicad_pcb')
    shutil.copyfile(FIX, board)
    shutil.copyfile(os.path.splitext(FIX)[0] + '.kicad_pro',
                    os.path.splitext(board)[0] + '.kicad_pro')

    def intent(blocks, name):
        doc = fp.emit_intent(parse_kicad_pcb(board), board)
        doc['blocks'] = blocks
        doc['decaps'] = {'max_distance_mm': 2.0, 'max_pin_distance_mm': 2.0}
        path = os.path.join(td, name + '.json')
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1)
        return path
    held = intent([{'name': 'rot_C24', 'refs': ['C24'], 'rotation': 270.0}],
                  'held')
    contra = intent([{'name': 'a', 'refs': ['C24'], 'rotation': 270.0},
                     {'name': 'b', 'refs': ['C2*'], 'rotation': 180.0}],
                    'contra')

    # the CLI arm, on the host python (see test_1067_cap_intent_gui for why
    # its PYTHON* settings are dropped)
    cli_out = os.path.join(td, 'cli.kicad_pcb')
    host_env = {k: v for k, v in os.environ.items()
                if not k.startswith('PYTHON')}
    r = subprocess.run([HOST_PYTHON, '-X', 'utf8', TOOL, board, cli_out,
                        '--clearance', '0.1', '--max-displacement', '0.6',
                        '--max-displacement-cap', '2.0', '--intent', held],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO, env=host_env)
    check('the CLI arm ran', r.returncode == 0, r.stderr[-600:])
    m = re.search(r'^JSON_SUMMARY: (.*)$', r.stdout, re.M)
    cli_sum = json.loads(m.group(1)) if m else {}
    check('the CLI kept the run WITHOUT the decap gate (the fixture '
          'exercises #1122\'s trap)',
          ((cli_sum.get('decap') or {}).get('compared') or {}).get('kept')
          == 'ungated', str((cli_sum.get('decap') or {}).get('compared')))
    cli_poses = ({k: (f.x, f.y, (f.rotation or 0.0) % 360)
                  for k, f in parse_kicad_pcb(cli_out).footprints.items()}
                 if r.returncode == 0 else {})
    seed = {k: (f.x, f.y, (f.rotation or 0.0) % 360)
            for k, f in parse_kicad_pcb(board).footprints.items()}

    app = wx.App(False)  # noqa: F841
    _install_fake_board(board)
    dlg = RoutingDialog(None, build_pcb_data_like_ipc(board), board)

    def _bytes():
        # this front writes to the .kicad_pro, so "unmodified" is both files
        return tuple(open(p, 'rb').read() for p in
                     (board, os.path.splitext(board)[0] + '.kicad_pro'))

    def drive(params):
        logs = []
        ex = ai_plan.PlanExecutor(
            dlg, [{'action': 'optimize_caps', 'params': params}], [0],
            lambda i, s: None, lambda c, a: None, log=logs.append)
        ex._queue = [0]
        ex._next_step()
        return '\n'.join(str(x) for x in logs)

    def live_poses():
        # The fake flushes every footprint move to its file on push.
        return {k: (f.x, f.y, (f.rotation or 0.0) % 360)
                for k, f in parse_kicad_pcb(board).footprints.items()}

    # == 1. a contradictory declaration: NOT run, nothing moves =============
    dlg.reset_params_to_defaults()
    before = live_poses()
    before_bytes = _bytes()
    log = drive(dict(TIGHT, cap_intent_path=contra))
    check('a contradictory declaration stops the step',
          'Cap optimization NOT run' in log and 'different rotations' in log,
          log[-600:])
    check('...moves nothing', live_poses() == before)
    check('...and leaves the board unmodified', _bytes() == before_bytes)

    # == 2. the real pass, C24 declared ======================================
    dlg.reset_params_to_defaults()
    log = drive(dict(TIGHT, cap_intent_path=held))
    after = live_poses()
    check('the live C24 keeps its declared angle',
          abs((after['C24'][2] - 270.0 + 180) % 360 - 180) < 1e-6,
          str(after.get('C24')))
    moved_gui = {k for k in after if k in seed and any(
        abs(a - b) > 1e-3 for a, b in zip(after[k], seed[k]))}
    moved_cli = {k for k, p in cli_poses.items() if k in seed and any(
        abs(a - b) > 1e-3 for a, b in zip(p, seed[k]))}
    check('the GUI moved caps', bool(moved_gui), log[-600:])
    check('the GUI moved exactly the caps the CLI moved',
          moved_gui == moved_cli,
          'gui-only %s cli-only %s' % (sorted(moved_gui - moved_cli),
                                       sorted(moved_cli - moved_gui)))
    off = [k for k in moved_gui & moved_cli
           if any(abs(a - b) > 1e-3 for a, b in zip(after[k], cli_poses[k]))]
    check('...to the same poses', not off,
          '; '.join('%s gui %s cli %s' % (k, after[k], cli_poses[k])
                    for k in off[:4]))
    dlg.Destroy()

    print()
    if failures:
        print('FAILED (%d): %s' % (len(failures), ', '.join(failures)))
        return 1
    print('ALL PASS -- the GUI cap pass holds a declared rotation in the run '
          'it keeps and lands where the CLI lands')
    return 0


if __name__ == '__main__':
    _rc = main()
    for _d in glob.glob(os.path.join(tempfile.gettempdir(), 't1122gui_*')):
        shutil.rmtree(_d, ignore_errors=True)
    sys.exit(_rc)
