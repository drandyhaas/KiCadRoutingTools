#!/usr/bin/env python3
"""#1067 on the REAL headless dialog: an `optimize_caps` plan step carrying
`cap_intent_path` runs the cap pass with the decap gate, on the LIVE board,
and lands where the CLI's `--intent` run lands.

    python3 tests/gui_parity/test_1067_cap_intent_gui.py

(re-execs into KiCad's bundled python automatically, like its siblings)

ipc-migration: the board is the fake-IPC harness's (fake_ipc_board), the
dialog is routing_dialog's, and the cap pass's moves are read back from the
file the fake flushes them to. The GUI builds PCBData from the board
(`build_pcb_data_from_board`) rather than parsing the file, and the decap gate reads pin TYPES and nets off that data,
so "the param reaches the engine" (test_772_cap_params_reach_engine) is not
enough: this drives the real engine on the U30 crop
(tests/fixtures/1067/u30_crop.kicad_pcb) and compares the live poses with a
CLI run on the same board. Also: a path that does not load stops the step
before the engine runs (the CLI's exit 2), moving nothing; and the control
round-trips through settings_persistence.

Exits 2 when no python with pcbnew+wx exists: it is a killer gate for
tests/mutate_1067.py, and a gate that exits 0 when it did not run reports
every row it guards as SURVIVED.
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
          'GUI cap pass with an intent, and nothing else does')
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
    # as its siblings do (test_962_paste_parity, test_footprint_position_sync):
    # the cap pass's apply loop imports `gui_utils` top-level
    sys.path.insert(0, os.path.join(REPO, 'kicad_routing_plugin'))
    from kicad_parser import parse_kicad_pcb
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from fake_ipc_board import install as _install_fake_board, build_pcb_data_like_ipc
    from kicad_routing_plugin.routing_dialog import RoutingDialog
    from kicad_routing_plugin import ai_plan, settings_persistence as sp
    from placement import floorplan as fp

    failures = []

    def check(name, ok, detail=''):
        if not ok:
            failures.append(name)
        print(('  PASS ' if ok else '  FAIL ') + name
              + (('  ' + detail) if detail and not ok else ''))

    td = tempfile.mkdtemp(prefix='t1067gui_')
    board = os.path.join(td, 'u30_crop.kicad_pcb')
    shutil.copyfile(FIX, board)
    shutil.copyfile(os.path.splitext(FIX)[0] + '.kicad_pro',
                    os.path.splitext(board)[0] + '.kicad_pro')
    doc = fp.emit_intent(parse_kicad_pcb(board), board)
    doc['blocks'] = []
    doc['decaps'] = {'max_distance_mm': 2.5, 'max_pin_distance_mm': 2.5}
    intent_path = os.path.join(td, 'crop.intent.json')
    with open(intent_path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=1)

    # the CLI arm, on the host python (the one that launched this gate)
    cli_out = os.path.join(td, 'cli.kicad_pcb')
    # KiCad's python exports its own PYTHON* settings -- PYTHONUSERBASE points
    # at KiCad's 3rdparty dir, which hides the host's user site-packages
    # (numpy) -- so the host interpreter gets none of them
    host_env = {k: v for k, v in os.environ.items()
                if not k.startswith('PYTHON')}
    r = subprocess.run([HOST_PYTHON, '-X', 'utf8', TOOL, board, cli_out,
                        '--clearance', '0.1', '--intent', intent_path],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO, env=host_env)
    check('the CLI arm ran', r.returncode == 0, r.stderr[-600:])
    m = re.search(r'^JSON_SUMMARY: (.*)$', r.stdout, re.M)
    cli_sum = json.loads(m.group(1)) if m else {}
    cli_moved = ({k: (f.x, f.y, (f.rotation or 0.0) % 360)
                  for k, f in parse_kicad_pcb(cli_out).footprints.items()}
                 if r.returncode == 0 else {})
    seed = {k: (f.x, f.y, (f.rotation or 0.0) % 360)
            for k, f in parse_kicad_pcb(board).footprints.items()}

    app = wx.App(False)  # noqa: F841
    _install_fake_board(board)
    dlg = RoutingDialog(None, build_pcb_data_like_ipc(board), board)

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

    # == 1. an unreadable path: NOT run, nothing moves ======================
    dlg.reset_params_to_defaults()
    before = live_poses()
    log = drive({'cap_intent_path': os.path.join(td, 'missing.json'),
                 'clearance': 0.1})
    check('an unreadable intent stops the step',
          'Cap optimization NOT run: cannot load intent' in log, log[-600:])
    check('...and moves nothing', live_poses() == before)

    # == 1b. a refused step owes no writeback (phase-3 verifier: the
    # net-class clamp still ran, Wide 0.4 -> 0.3, board modified) ===========
    # (ipc-migration: this front writes floors and classes to the .kicad_pro,
    # so "unmodified" is both files byte-identical.)
    flat_src = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
    flat = os.path.join(td, 'flat_hierarchy.kicad_pcb')
    shutil.copyfile(flat_src, flat)
    shutil.copyfile(os.path.splitext(flat_src)[0] + '.kicad_pro',
                    os.path.splitext(flat)[0] + '.kicad_pro')

    def _bytes(path):
        return tuple(open(p, 'rb').read() for p in
                     (path, os.path.splitext(path)[0] + '.kicad_pro'))
    flat_before = _bytes(flat)
    _install_fake_board(flat)
    dlg.reset_params_to_defaults()
    log = drive({'cap_intent_path': os.path.join(td, 'missing.json'),
                 'clearance': 0.3})
    check('a refused step on a board with a wider class leaves it '
          'unmodified', _bytes(flat) == flat_before, log[-600:])
    _install_fake_board(board)

    # == 2. the real pass with the intent ===================================
    dlg.reset_params_to_defaults()
    log = drive({'cap_intent_path': intent_path, 'clearance': 0.1})
    after = live_poses()
    moved_gui = {r for r in after if r in seed and any(
        abs(a - b) > 1e-3 for a, b in zip(after[r], seed[r]))}
    moved_cli = {r for r, p in cli_moved.items() if r in seed and any(
        abs(a - b) > 1e-3 for a, b in zip(p, seed[r]))}
    check('the GUI moved caps', bool(moved_gui), log[-600:])
    check('the GUI moved exactly the caps the CLI moved',
          moved_gui == moved_cli,
          'gui-only %s cli-only %s' % (sorted(moved_gui - moved_cli),
                                       sorted(moved_cli - moved_gui)))
    off = [r for r in moved_gui & moved_cli
           if any(abs(a - b) > 1e-3 for a, b in zip(after[r], cli_moved[r]))]
    check('...to the same poses', not off,
          '; '.join('%s gui %s cli %s' % (r, after[r], cli_moved[r])
                    for r in off[:4]))
    broken = sorted((cli_sum.get('decap') or {}).get('broken') or {})
    added = ((cli_sum.get('decap') or {}).get('grade') or {}).get('added')
    check('the CLI broke a claim for a cap (the fixture exercises the '
          'ladder)', bool(broken), str(cli_sum.get('decap'))[:300])
    check('the GUI summary names the cap that broke a decap limit',
          all(r in log for r in broken) and 'broke a decap limit' in log,
          log[-800:])
    check('the GUI summary reports the NEW decap error(s) the CLI added',
          ('NEW decap error' in log) == bool(added), log[-800:])

    # == 3. the control round-trips through settings ========================
    dlg.fanout_tab.bga_options.cap_intent_path.SetValue(intent_path)
    saved = sp.get_dialog_settings(dlg)
    check('settings save the intent path',
          saved.get('fanout_bga_cap_intent_path') == intent_path,
          repr(saved.get('fanout_bga_cap_intent_path')))
    dlg2 = RoutingDialog(None, build_pcb_data_like_ipc(board), board)
    sp.restore_dialog_settings(dlg2, saved)
    check('...and restore it',
          dlg2.fanout_tab.bga_options.cap_intent_path.GetValue()
          == intent_path)
    for d in (dlg, dlg2):
        d.Destroy()

    print()
    if failures:
        print('FAILED (%d): %s' % (len(failures), ', '.join(failures)))
        return 1
    print('ALL PASS -- the GUI cap pass holds the intent\'s decap limits and '
          'lands where the CLI lands')
    return 0


if __name__ == '__main__':
    sys.exit(main())
