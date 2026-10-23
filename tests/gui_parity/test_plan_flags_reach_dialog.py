#!/usr/bin/env python3
"""A recorded flag reaches the REAL dialog through a converted plan, and the
next step that omits it runs at the CLI default again.

test_manifest_plan_parity.py proves by AST that each flag converts to a param
naming a settable, reset control. What an AST cannot see is the run itself:
whether the executor's _set_control really sets that widget (a RadioBox has
no SetValue, so bga_fanout's --primary-escape was dropped while every table
named it), whether the control HOLDS the value (a SpinCtrlDouble rounds to its
digits), and whether the value is the one the tab hands its engine.

So, per flag: a recorded manifest line carrying it, followed by the same step
without it, is converted by tests/stress/manifest_to_plan.py's own CLI; the
plan step must carry the param; then the plan runs on the REAL PlanExecutor
over the REAL headless RoutingDialog, with each step's action replaced by a
probe (nothing routes, nothing is written). The probe reads the control AND
the tab config its engine call reads. The flagged step must see the recorded
value in both; the following step must see the CLI default in both -- the
reset_params_to_defaults half, without which a plan leaks one step's value
into every later step.

Needs KiCad's python (re-execs into it); exits 77 when there is none. Seconds.

Run: python3 tests/gui_parity/test_plan_flags_reach_dialog.py
"""
import contextlib
import glob
import io
import json
import os
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


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew+wx found")
    sys.exit(77)


# (label, tool + args BEFORE the boards' flags, the flag and its value, the
# plan param and value it must convert to, probe name, value with the flag,
# value without it). Every probe returns (control value, engine config value).
# A step's boards are filled in by the chain below, so the converter's pruning
# keeps every step.
_PLANES = ('python3 py_router/route_planes.py {src} {dst} --nets GND '
           '--plane-layers B.Cu')
# splitflap_driver's U2 is mounted on F.Cu (the QFN engine is a spy below, so
# its package does not matter). Item 0 of the QFN panel's escape-layer choice:
_QFN_REF, _MOUNTED = 'U2', 'Mounted layer (default)'
# A --net-clearances file beside the manifest. splitflap_driver is an
# all-Default board, so its class-derived map is EMPTY and any entry here is
# the file's doing.
_NCL_FILE = 'ncl.json'
_NCL = {'/CLOCK_IN': 0.111, '+12V': 0.321}
# A --fab-overrides file beside the manifest (a comment only: no override).
_FAB_FILE = 'fab.txt'
CASES = [
    # (override box, spin, the setback the engine gets). Unticked = auto: the
    # CLI's omitted flag, None to the engine.
    dict(label='route_diff --diff-pair-centerline-setback',
         cmd='python3 py_router/route_diff.py {src} {dst} "*"',
         flag='--diff-pair-centerline-setback 0.35',
         param=('centerline_setback', 0.35), probe='centerline_setback',
         want=(True, 0.35, 0.35), default=(False, 0.0, None)),
    # An EXPLICIT 0 is a setback of zero to the engine, not auto -- the GUI's
    # old "0 = auto" spin could not say it.
    dict(label='route_diff --diff-pair-centerline-setback 0 (explicit zero)',
         cmd='python3 py_router/route_diff.py {src} {dst} "*"',
         flag='--diff-pair-centerline-setback 0',
         param=('centerline_setback', 0), probe='centerline_setback',
         want=(True, 0.0, 0.0), default=(False, 0.0, None)),
    dict(label='bga_fanout --primary-escape',
         cmd='python3 py_router/bga_fanout.py {src} --output {dst} '
             '--component U1',
         flag='--primary-escape vertical',
         param=('escape_direction', 'vertical'), probe='escape_direction',
         want=('Vertical', 'vertical'), default=('Horizontal', 'horizontal')),
    dict(label='route_planes --min-thickness',
         cmd=_PLANES, flag='--min-thickness 0.0762',
         param=('min_thickness', 0.0762), probe='min_thickness',
         want=(0.0762, 0.0762), default=(0.1, 0.1)),
    dict(label='route_planes --plane-max-iterations',
         cmd=_PLANES, flag='--plane-max-iterations 50000',
         param=('plane_max_iterations', 50000), probe='plane_max_iterations',
         want=(50000, 50000), default=(200000, 200000)),
    dict(label='route_planes --voronoi-seed-interval',
         cmd=_PLANES, flag='--voronoi-seed-interval 3.25',
         param=('voronoi_seed_interval', 3.25), probe='voronoi_seed_interval',
         want=(3.25, 3.25), default=(2.0, 2.0)),
    dict(label='route_planes --skip-existing-zones',
         cmd=_PLANES, flag='--skip-existing-zones',
         param=('skip_existing_zones', True), probe='skip_existing_zones',
         want=(True, True), default=(False, False)),
    # A RELATIVE file, as the corpus records it: the plan must carry it
    # absolute, resolved against the command's folder ({tmp}).
    dict(label='route --net-clearances',
         cmd='python3 py_router/route.py {src} {dst} --nets "*"',
         flag='--net-clearances ' + _NCL_FILE,
         param=('net_clearances_path', '{tmp}/' + _NCL_FILE),
         probe='net_clearances_path',
         want=('{tmp}/' + _NCL_FILE,) * 2, default=('', '')),
    dict(label='route_diff --net-clearances',
         cmd='python3 py_router/route_diff.py {src} {dst} "*"',
         flag='--net-clearances ' + _NCL_FILE,
         param=('net_clearances_path', '{tmp}/' + _NCL_FILE),
         probe='net_clearances_path_diff',
         want=('{tmp}/' + _NCL_FILE,) * 2, default=('', '')),
    dict(label='route_planes --net-clearances',
         cmd=_PLANES,
         flag='--net-clearances ' + _NCL_FILE,
         param=('net_clearances_path', '{tmp}/' + _NCL_FILE),
         probe='net_clearances_path_planes',
         want=('{tmp}/' + _NCL_FILE,) * 2, default=('', '')),
    dict(label='route --fab-overrides (relative file)',
         cmd='python3 py_router/route.py {src} {dst} --nets "*"',
         flag='--fab-overrides ' + _FAB_FILE,
         param=('fab_overrides_path', '{tmp}/' + _FAB_FILE),
         probe='fab_overrides_path',
         want=('{tmp}/' + _FAB_FILE,) * 2, default=('', '')),
    dict(label='qfn_fanout --layer',
         cmd='python3 py_router/qfn_fanout.py {src} --output {dst} '
             '--component ' + _QFN_REF,
         flag='--layer B.Cu',
         param=('qfn_layer', 'B.Cu'), probe='qfn_layer',
         want=('B.Cu', 'B.Cu'), default=(_MOUNTED, None)),
]


def _probes(dlg):
    """{probe name: (control value, engine config value)} right now."""
    d = dlg.differential_tab
    bga = dlg.fanout_tab.bga_options
    po = dlg.planes_tab.create_options
    # What the planes tab hands create_plane: its ONE assembly point.
    plane_cfg = dlg.planes_tab._build_mode_config()
    planes = {name: (round(getattr(po, name).GetValue(), 6),
                     plane_cfg.get(name))
              for name in ('min_thickness', 'plane_max_iterations',
                           'voronoi_seed_interval')}
    planes['skip_existing_zones'] = (po.skip_existing_zones.GetValue(),
                                     plane_cfg.get('skip_existing_zones'))
    qo = dlg.fanout_tab.qfn_options
    # The Route tab's config, and the Differential tab's merged one.
    route_cfg = dlg._build_routing_config([], ['F.Cu', 'B.Cu'])
    diff_cfg = {**d.get_routing_config(), **d.get_config()}
    return {
        **planes,
        'qfn_layer': (qo.qfn_layer.GetStringSelection(),
                      qo.get_config().get('layer')),
        'net_clearances_path': (dlg.net_clearances_path.GetValue(),
                                route_cfg.get('net_clearances_path')),
        'fab_overrides_path': (dlg.fab_overrides_path.GetValue(),
                               route_cfg.get('fab_overrides_path')),
        'net_clearances_path_diff': (dlg.net_clearances_path.GetValue(),
                                     diff_cfg.get('net_clearances_path')),
        'net_clearances_path_planes': (dlg.net_clearances_path.GetValue(),
                                       plane_cfg.get('net_clearances_path')),
        'centerline_setback': (
            d.centerline_setback_check.GetValue(),
            round(d.centerline_setback.GetValue(), 6),
            d.get_config().get('diff_pair_centerline_setback')),
        'escape_direction': (
            bga.escape_direction.GetStringSelection(),
            bga.get_config().get('primary_escape')),
    }


def _sub(value, tmp):
    """A case value with its {tmp} placeholder filled in. A placeholder
    marks a path, normalised as the converter normalises the one it joins
    (`os.path.normpath`), so '{tmp}/x' compares on Windows too."""
    if isinstance(value, str):
        return (os.path.normpath(value.replace('{tmp}', tmp))
                if '{tmp}' in value else value)
    if isinstance(value, tuple):
        return tuple(_sub(v, tmp) for v in value)
    return value


def _worker_net_clearances(dlg, path):
    """{'route' | 'diff' | 'planes': the per-net clearance map, BY NET NAME,
    that the Route, Differential and Planes tabs' REAL workers hand
    batch_route / batch_route_diff_pairs / create_plane} with the Net
    Clearances File set to `path`. Each
    engine is a spy that records its kwargs and stops the run, so nothing
    routes; wx.CallAfter is held so no completion handler fires later."""
    import wx
    import route
    import route_diff
    import route_planes

    class _Stop(Exception):
        pass

    seen = {}

    def _spy(tag):
        def _engine(*a, **kw):
            seen[tag] = kw.get('net_clearances')
            raise _Stop('engine reached')
        return _engine

    names = {nid: n.name for nid, n in dlg.pcb_data.nets.items()}
    real = (route.batch_route, route_diff.batch_route_diff_pairs, wx.CallAfter,
            route_planes.create_plane)
    d = dlg.differential_tab
    p = dlg.planes_tab
    try:
        route.batch_route = _spy('route')
        route_diff.batch_route_diff_pairs = _spy('diff')
        route_planes.create_plane = _spy('planes')
        wx.CallAfter = lambda *a, **k: None
        dlg.net_clearances_path.SetValue(path)
        dlg._run_routing(dlg._build_routing_config(['/CLOCK_IN'],
                                                   ['F.Cu', 'B.Cu']))
        # (the diff worker print_exc()s the stop; it is not a failure)
        with contextlib.redirect_stderr(io.StringIO()):
            d._run_diff_routing({**d.get_routing_config(), **d.get_config()},
                                ['/CLOCK_IN'])
            # (the planes worker reports the stop as its plane error)
            with contextlib.redirect_stdout(io.StringIO()):
                p._run_create_planes(dict(p._build_mode_config(),
                                          assignments=[(['GND'], ['B.Cu'])]))
    finally:
        (route.batch_route, route_diff.batch_route_diff_pairs, wx.CallAfter,
         route_planes.create_plane) = real
        dlg.net_clearances_path.SetValue('')
    return {tag: ({names.get(i, i): v for i, v in m.items()}
                  if isinstance(m, dict) else m)
            for tag, m in seen.items()}


def _qfn_engine_layers(dlg, run_until):
    """{'mounted' | 'B.Cu': (layer the tab hands generate_qfn_fanout, the
    warning lines it logged)} from the FanoutTab's REAL QFN run, with the
    engine replaced by a spy that returns no copper and the apply stubbed."""
    import qfn_fanout
    tab = dlg.fanout_tab
    qo = tab.qfn_options
    fp = dlg.pcb_data.footprints[_QFN_REF]
    nets = sorted({p.net_name for p in fp.pads if p.net_id and p.net_name})
    real_engine, real_apply = qfn_fanout.generate_qfn_fanout, tab._apply_fanout_results
    real_log = tab.append_log
    out = {}
    try:
        for label, item in (('mounted', _MOUNTED), ('B.Cu', 'B.Cu')):
            seen, logged = {}, []

            def _spy(footprint, pcb, **kw):
                seen['layer'] = kw.get('layer')
                return [], [], []

            qfn_fanout.generate_qfn_fanout = _spy
            tab._apply_fanout_results = lambda *a, **k: None
            tab.append_log = logged.append
            qo.qfn_layer.SetSelection(qo.qfn_layer.FindString(item))
            tab._run_qfn_fanout(fp, nets, qo.get_config())
            run_until(lambda: not getattr(tab, '_running', False), 120)
            out[label] = (seen.get('layer'),
                          [w for w in logged if 'WARNING' in w])
    finally:
        qfn_fanout.generate_qfn_fanout = real_engine
        tab._apply_fanout_results = real_apply
        tab.append_log = real_log
        qo.qfn_layer.SetSelection(0)
    return out


def main():
    try:
        import wx  # noqa: F401
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')
    import wx
    import pcbnew
    for p in (REPO, os.path.join(REPO, 'py_router'),
              os.path.join(REPO, 'py_tools'),
              os.path.join(REPO, 'tests', 'gui_parity')):
        sys.path.insert(0, p)
    app = wx.App(False)  # noqa: F841  (before any wx object)
    wx.MessageBox = lambda *a, **k: wx.OK

    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.swig_gui import RoutingDialog
    from kicad_routing_plugin import ai_plan, gui_utils
    import fix_kicad_drc_settings
    from copy_board import copy_board
    from wx_pump import run_until

    tmp = tempfile.mkdtemp(prefix='plan_flags_')
    results = []

    def check(label, ok, detail):
        results.append(ok)
        print(f"  {'PASS' if ok else 'FAIL'}  {label}: {detail}")

    try:
        # 1. A manifest with each case's flagged step and then its plain step,
        #    chained board to board so pruning keeps them all.
        lines, n = ['#!/bin/bash'], 0
        for case in CASES:
            for with_flag in (True, False):
                cmd = case['cmd'].format(src=f'b{n}.kicad_pcb',
                                         dst=f'b{n + 1}.kicad_pcb')
                lines.append(cmd + (' ' + case['flag'] if with_flag else ''))
                n += 1
        manifest = os.path.join(tmp, 'redo_commands.sh')
        plan_path = os.path.join(tmp, 'plan.json')
        ncl_path = os.path.join(tmp, _NCL_FILE)
        with open(ncl_path, 'w', encoding='utf-8') as f:
            json.dump(_NCL, f)
        with open(os.path.join(tmp, _FAB_FILE), 'w', encoding='utf-8') as f:
            f.write('# no override: the path is what is under test\n')
        with open(manifest, 'w', encoding='utf-8', newline='\n') as f:
            f.write('\n'.join(lines) + '\n')
        conv = subprocess.run(
            [sys.executable, os.path.join(REPO, 'tests', 'stress',
                                          'manifest_to_plan.py'),
             manifest, plan_path], capture_output=True, text=True)
        if conv.returncode != 0 or not os.path.isfile(plan_path):
            print(f"BROKEN TEST: manifest_to_plan exited {conv.returncode}:\n"
                  f"{conv.stdout}{conv.stderr}")
            return 2
        with open(plan_path, encoding='utf-8') as f:
            steps = json.load(f)['steps']
        if len(steps) != 2 * len(CASES):
            print(f"BROKEN TEST: {len(steps)} plan step(s) for "
                  f"{2 * len(CASES)} manifest lines:\n{conv.stdout}")
            return 2

        print("[1] manifest_to_plan: the flagged step carries the param, the "
              "plain step does not")
        for k, case in enumerate(CASES):
            key, val = case['param']
            val = _sub(val, tmp)
            on, off = steps[2 * k], steps[2 * k + 1]
            got = (on.get('params') or {}).get(key, '<absent>')
            check(case['label'], got == val and key not in
                  (off.get('params') or {}),
                  f"flagged step params[{key!r}] = {got!r} (want {val!r}); "
                  f"plain step {'omits it' if key not in (off.get('params') or {}) else 'CARRIES it'}"
                  f" -- {lines[2 * k + 1]}")

        # 2. The plan through the real executor; each action is a probe.
        board_path = os.path.join(tmp, 'board.kicad_pcb')
        copy_board(os.path.join(REPO, 'kicad_files',
                                'splitflap_driver.kicad_pcb'), board_path)
        board = pcbnew.LoadBoard(board_path)
        pcbnew.GetBoard = lambda: board
        # Nothing is written: the end-of-plan floor writers are spies.
        gui_utils.update_live_drc_floors = lambda *a, **k: []
        fix_kicad_drc_settings.fix_project_for_output = lambda *a, **k: None
        dlg = RoutingDialog(None, parse_kicad_pcb(board_path), board_path)
        try:
            seen, done = [], []
            ex = ai_plan.PlanExecutor(dlg, steps, range(len(steps)),
                                      on_status=lambda i, s: None,
                                      on_finished=lambda n, r: done.append(r))
            ex.POLL_MS = 10
            def _probe_step(action):
                seen.append(_probes(dlg))
                # Report a produced result, as each tab's apply does: a step
                # that marks none stops the plan (ai_plan._step_refusal).
                owner = ex._action_owner(action)
                if owner is not None:
                    gui_utils.mark_plan_step_produced(owner)
            ex._action_parts = lambda action: (
                lambda: _probe_step(action), lambda: False)
            ex.start()
            run_until(lambda: bool(done), 120)
            qfn_runs = _qfn_engine_layers(dlg, run_until)
            ncl_runs = {'file': _worker_net_clearances(dlg, ncl_path),
                        'none': _worker_net_clearances(dlg, '')}
        finally:
            dlg.Destroy()
        if len(seen) != len(steps):
            print(f"BROKEN TEST: the executor ran {len(seen)} of "
                  f"{len(steps)} steps (finished: {done})")
            return 2

        print("\n[2] the real dialog: the flagged step's control and engine "
              "config hold the value; the next step is back at the default")
        for k, case in enumerate(CASES):
            on = seen[2 * k][case['probe']]
            off = seen[2 * k + 1][case['probe']]
            want = _sub(tuple(case['want']), tmp)
            default = _sub(tuple(case['default']), tmp)
            check(case['label'] + ' (flagged step)', on == want,
                  f"(control, engine config) = {on!r}, want {want!r}")
            check(case['label'] + ' (next step, flag omitted)', off == default,
                  f"(control, engine config) = {off!r}, want the CLI default "
                  f"{default!r}")

        print("\n[3] the QFN tab's own fanout run: the escape layer reaches "
              "generate_qfn_fanout, with the CLI's #96 warning off the "
              "mounted layer")
        mounted = qfn_runs['mounted']
        b_cu = qfn_runs['B.Cu']
        check("default choice: the engine gets the part's mounted layer, no "
              "warning", mounted[0] == 'F.Cu' and not mounted[1],
              f"layer={mounted[0]!r}, warnings={mounted[1]}")
        check("B.Cu chosen: the engine gets B.Cu and the log warns",
              b_cu[0] == 'B.Cu' and any('differs from' in w and 'mounted layer'
                                        in w for w in b_cu[1]),
              f"layer={b_cu[0]!r}, warnings={b_cu[1]}")

        print("\n[4] the centerline setback: the dialog's explicit 0 and its "
              "auto are the CLI's explicit 0 and omitted flag to the engine, "
              "and the two are different runs")
        from routing_config import GridRouteConfig
        from diff_pair_multipoint import diff_pair_min_coupled_length

        def _min_coupled(setback):
            # 0.3/0.3 geometry: auto's 4x half-spacing is 1.2 mm, so its 5x
            # term (6 mm) clears the 3 mm floor and an explicit 0's does not.
            return diff_pair_min_coupled_length(GridRouteConfig(
                track_width=0.3, diff_pair_gap=0.3,
                diff_pair_centerline_setback=setback))
        k0 = next(k for k, c in enumerate(CASES) if 'explicit zero' in c['label'])
        gui_zero = seen[2 * k0]['centerline_setback'][2]
        gui_auto = seen[2 * k0 + 1]['centerline_setback'][2]
        cli_zero, cli_auto = _min_coupled(0.0), _min_coupled(None)
        check("the engine tells an explicit 0 from auto",
              cli_zero != cli_auto,
              f"min coupled length {cli_zero} mm (0) vs {cli_auto} mm (auto)")
        check("the dialog's explicit-0 step routes as the CLI's explicit 0",
              _min_coupled(gui_zero) == cli_zero,
              f"setback {gui_zero!r} -> {_min_coupled(gui_zero)} mm")
        check("the dialog's next step routes as the CLI's omitted flag",
              _min_coupled(gui_auto) == cli_auto,
              f"setback {gui_auto!r} -> {_min_coupled(gui_auto)} mm")

        print("\n[5] the Net Clearances File reaches the engines: the Route, "
              "Differential and Planes workers hand them the file's map, as "
              "route.py / route_diff.py / route_planes.py --net-clearances do")
        from kicad_parser import parse_kicad_pcb as _parse
        from list_nets import load_net_clearances, net_clearance_map_by_name
        _cli = _parse(board_path)
        cli_map = {_cli.nets[i].name: v for i, v in net_clearance_map_by_name(
            _cli.nets, load_net_clearances(ncl_path)).items()}
        check("the CLI's map from the file names both nets",
              cli_map == _NCL, f"{cli_map}")
        for tag in ('route', 'diff', 'planes'):
            got = ncl_runs['file'].get(tag, '<engine not reached>')
            check(f"{tag} worker with the file: the engine gets the CLI's map",
                  got == cli_map, f"{got}")
            none = ncl_runs['none'].get(tag, '<engine not reached>')
            check(f"{tag} worker with the field empty: the net classes' map "
                  f"(empty on this all-Default board), not the file's",
                  none == {}, f"{none}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    n_ok = sum(results)
    print(f"\n{n_ok}/{len(results)} checks passed")
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
