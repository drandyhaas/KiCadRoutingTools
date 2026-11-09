#!/usr/bin/env python3
"""#1698: a plan's net-class map and written project match the CLI's.

    python3 tests/gui_parity/test_1698_plan_clearance_parity_gui.py

Needs KiCad's python (re-execs into it); exits 2 without it. On a board with a
wider second class, the same flags go through the CLI and through the real dialog
and plan executor, and the gate compares what the engine was handed, the project
written and the copper:

  A. `clearance` alone: the other class stays as declared. The end-of-plan write
     used to read the Min Clearance box, which a bare `clearance` also ticks, and
     lowered it on the board and the project.
  B. a ceiling alone: the project is written. It used to return early, as no step
     carried `clearance`, and manifest_to_plan never emits one beside a ceiling.
  C. a ceiling above the Default class: each tab caps its map at the ceiling, not
     at the run's clearance, so the other class routes at what it is graded at.
  D. the differential and planes tabs, the same.
  E. the steps that RAN decide: an earlier step's ceiling holds when the last step
     has none, and a step the run did not reach writes nothing.

The rules of the project are compared too: the copper-to-hole floor the steps held
must survive the plan-end write, which otherwise defaults it to the copper
clearance. `min_via_drill` is left out: KiCad's own save drops that repo-private
rule from the project, and tool_records carries it.

ipc-migration: the dialog runs over the fake-IPC board
(`tests/gui_parity/fake_ipc_board.py`, which `headless_plan.run_plan` installs).
kipy cannot write live design settings, so this front's class writeback is the
sibling .kicad_pro only (each tab's `gui_utils.apply_drc_settings_fix` and the
plan executor's end-of-plan `fix_project_for_output`). The SWIG gate also
compared the live board's net settings with the CLI's project; there is no live
class store on this front, so that check is dropped and the written project is
compared.
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
for sub in ('py_router', 'py_tools', 'py_placer'):
    sys.path.insert(0, os.path.join(REPO, sub))
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

BOARD = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
HOST_PYTHON = os.environ.get('KRT_HOST_PYTHON', sys.executable)
NETS = ['VCC', 'VPP', 'VCC_PIC', 'VPP-MCLR', 'DATA-RB7', 'CLOCK-RB6', 'Net-(Q1-B)', 'Net-(Q2-B)',
        'Net-(Q2-C)', 'Net-(R8-Pad1)']
VIA = {'via_size': 0.5, 'via_drill': 0.3}
VIA_FLAGS = ('--via-size', '0.5', '--via-drill', '0.3')
ROUTE = {'track_width': 0.15, **VIA}
ROUTE_FLAGS = ('--track-width', '0.15', *VIA_FLAGS)
PAIR = {'diff_pair_width': 0.15, 'diff_pair_gap': 0.2, **VIA}
PAIR_FLAGS = ('--track-width', '0.15', '--diff-pair-gap', '0.2', *VIA_FLAGS)
failures = []


def check(name, ok, detail=''):
    if not ok:
        failures.append(name)
    print(('  PASS ' if ok else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def reexec_into_kicad():
    from kicad_locate import kicad_python_candidates
    for py in kicad_python_candidates():
        if py == sys.executable or not os.path.exists(py):
            continue
        if subprocess.run([py, '-c', 'import wx, kipy'], capture_output=True).returncode == 0:
            env = dict(os.environ, KRT_HOST_PYTHON=sys.executable)
            sys.exit(subprocess.run([py, '-X', 'utf8', os.path.abspath(__file__)],
                                    env=env).returncode)
    print('NOT RUN: no python with wx + kipy found')
    sys.exit(2)


def stage(path, pair=False):
    from copy_board import copy_board
    os.makedirs(os.path.dirname(path), exist_ok=True)
    copy_board(BOARD, path)
    if pair:
        # Two nets the board's `V*` pattern puts in the wider class, named as a pair
        with open(path, encoding='utf-8') as f:
            text = f.read()
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text.replace('"DATA-RB7"', '"VD_P"').replace('"CLOCK-RB6"', '"VD_N"'))
    return path


def project(board):
    with open(os.path.splitext(board)[0] + '.kicad_pro', encoding='utf-8') as f:
        doc = json.load(f)
    rules = dict(doc['board']['design_settings']['rules'])
    rules.pop('min_via_drill', None)
    return {c['name']: c['clearance'] for c in doc['net_settings']['classes']}, rules


def copper(board):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(board)
    name = {n.net_id: n.name for n in pcb.nets.values()}
    segs = sorted((name.get(s.net_id) or '', s.layer, sorted([(round(s.start_x, 6), round(s.start_y, 6)),
                                                              (round(s.end_x, 6), round(s.end_y, 6))]),
                   round(s.width, 6)) for s in pcb.segments)
    vias = sorted((name.get(v.net_id) or '', round(v.x, 6), round(v.y, 6), round(v.size, 6), round(v.drill, 6))
                  for v in pcb.vias)
    return segs, vias


def engine_calls(path):
    """What the engine was handed, per call: (the map's values, the ceiling kwarg)."""
    out = []
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as f:
            for line in f:
                d = json.loads(line)
                m = d.get('net_clearances')
                out.append((sorted({round(v, 6) for v in m.values()}) if isinstance(m, dict) else None,
                            d.get('clearance_ceiling')))
    return out


def cli_chain(td, name, *commands, pair=False):
    """The CLI's steps in turn, each reading the last one's output and its project."""
    prev = stage(os.path.join(td, name, 'in.kicad_pcb'), pair)
    dump = os.path.join(td, name, 'engine.jsonl')
    env = {k: v for k, v in os.environ.items() if not k.startswith('PYTHON')}
    env.update(KICAD_DUMP_BATCH_KWARGS=dump, KICAD_DUMP_BATCH_KWARGS_CONTINUE='1')
    for n, (tool, *flags) in enumerate(commands):
        out = os.path.join(td, name, f'step{n}.kicad_pcb')
        r = subprocess.run([HOST_PYTHON, '-X', 'utf8', os.path.join(REPO, 'py_router', tool), prev, out, *flags],
                           capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=REPO, env=env)
        if not os.path.isfile(os.path.splitext(out)[0] + '.kicad_pro'):
            raise AssertionError(f'BROKEN TEST: {tool} wrote no project: exit {r.returncode} '
                                 f'{(r.stdout + r.stderr)[-600:]}')
        prev = out
    return prev, engine_calls(dump)


def plan(td, name, steps, indices=None, pair=False):
    """The plan through the real dialog and executor, on a fresh copy of the board."""
    from headless_plan import run_plan
    path = stage(os.path.join(td, name, 'plan.kicad_pcb'), pair)
    dump = os.environ['KICAD_DUMP_BATCH_KWARGS']
    if os.path.isfile(dump):
        os.remove(dump)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        result = run_plan(path, steps, indices=indices, timeout=900)
    ran = indices if indices is not None else range(len(steps))
    statuses = [s.get('status') for s in result.get('steps') or ()]
    if statuses != ['done'] * len(ran):
        raise AssertionError(f'BROKEN TEST: the plan did not run: {statuses} {buf.getvalue()[-800:]}')
    return path, engine_calls(dump)

def same(label, cli_board, cli_calls, gui_board, gui_calls, *, map_values=None):
    """The fronts agree: engine map, written classes and rules, copper."""
    cc, cr = project(cli_board)
    gc, gr = project(gui_board)
    check(f'{label}: the written classes match the CLI', cc == gc, f'cli {cc} plan {gc}')
    diff = {k: (cr.get(k), gr.get(k)) for k in set(cr) | set(gr) if cr.get(k) != gr.get(k)}
    check(f'{label}: the written rules match the CLI', not diff, f'(cli, plan) {diff}' if diff else '')
    # The SWIG gate compared the live board's classes here as well. kipy has no
    # live class store or writer, so the written project above is the only one.
    if map_values is not None:
        check(f'{label}: the CLI engine got the map [precondition]',
              cli_calls and cli_calls[0][0] == map_values, f'{cli_calls[:1]}')
        check(f'{label}: the plan engine got the same map',
              gui_calls and gui_calls[0][0] == map_values, f'{gui_calls[:1]}')
    a, b = copper(cli_board), copper(gui_board)
    check(f'{label}: the copper matches the CLI', a == b,
          f'{len(a[0])} segments, {len(a[1])} vias' if a == b else
          f'cli {len(a[0])}/{len(a[1])} plan {len(b[0])}/{len(b[1])}')


def route_step(params, nets=NETS):
    return {'action': 'route', 'nets': list(nets), 'params': {**ROUTE, **params}}


def main():
    try:
        import wx  # noqa: F401
        import kipy  # noqa: F401
    except ImportError:
        reexec_into_kicad()
    os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

    td = tempfile.mkdtemp(prefix='krt1698_')
    # read at import, so before the engines are
    os.environ['KICAD_DUMP_BATCH_KWARGS'] = os.path.join(td, 'plan_engine.jsonl')
    os.environ['KICAD_DUMP_BATCH_KWARGS_CONTINUE'] = '1'
    try:
        nets = ('--nets', *NETS)
        print('A. `clearance` alone: the other class stays as declared')
        cli = cli_chain(td, 'a_cli', ('route.py', *nets, *ROUTE_FLAGS, '--clearance', '0.15'))
        check('precondition: the CLI keeps Wide and lowers Default',
              project(cli[0])[0] == {'Default': 0.15, 'Wide': 0.4}, f'{project(cli[0])[0]}')
        same('clearance', *cli, *plan(td, 'a', [route_step({'clearance': 0.15})]), map_values=[0.4])

        print('B. a ceiling alone: the project is written')
        cli = cli_chain(td, 'b_cli', ('route.py', *nets, *ROUTE_FLAGS, '--clearance-ceiling', '0.15'))
        check('precondition: the CLI writes both at the ceiling',
              project(cli[0])[0] == {'Default': 0.15, 'Wide': 0.15}, f'{project(cli[0])[0]}')
        check('precondition: the CLI holds the copper-to-hole floor above that',
              project(cli[0])[1].get('min_hole_clearance', 0) > 0.15, f"{project(cli[0])[1].get('min_hole_clearance')}")
        same('ceiling below Default', *cli, *plan(td, 'b', [route_step({'clearance_ceiling': 0.15})]),
             map_values=[0.15])

        print('C. a ceiling above the Default class: the map caps at the ceiling')
        cli = cli_chain(td, 'c_cli', ('route.py', *nets, *ROUTE_FLAGS, '--clearance-ceiling', '0.3'))
        check('precondition: the CLI writes Wide at the ceiling, Default as it was',
              project(cli[0])[0] == {'Default': 0.2, 'Wide': 0.3}, f'{project(cli[0])[0]}')
        same('ceiling above Default', *cli, *plan(td, 'c', [route_step({'clearance_ceiling': 0.3})]),
             map_values=[0.3])

        print('D. the differential and planes tabs')
        cli = cli_chain(td, 'd_cli', ('route_diff.py', '--nets', 'VD_*', *PAIR_FLAGS, '--clearance-ceiling', '0.3'),
                        pair=True)
        step = {'action': 'route_diff', 'pairs': ['VD'], 'params': {**PAIR, 'clearance_ceiling': 0.3}}
        same('differential', *cli, *plan(td, 'd', [step], pair=True), map_values=[0.3])
        step['params'] = {**PAIR, 'clearance': 0.15}
        cli = cli_chain(td, 'd2_cli', ('route_diff.py', '--nets', 'VD_*', *PAIR_FLAGS, '--clearance', '0.15'),
                        pair=True)
        same('differential, `clearance` alone', *cli, *plan(td, 'd2', [step], pair=True), map_values=[0.4])

        cli = cli_chain(td, 'e_cli', ('route_planes.py', '--nets', 'GND', '--plane-layers', 'B.Cu', *VIA_FLAGS,
                                      '--clearance-ceiling', '0.3'))
        step = {'action': 'route_planes', 'assignments': [{'nets': ['GND'], 'layer': 'B.Cu'}],
                'params': {**VIA, 'clearance_ceiling': 0.3}}
        gui = plan(td, 'e', [step])
        # The CLI's create_plane reads the classes itself and caps them at this kwarg
        check('planes: the CLI engine got the ceiling [precondition]', cli[1] and cli[1][0][1] == 0.3, f'{cli[1][:1]}')
        check('planes: the plan engine got the same ceiling', gui[1] and gui[1][0][1] == 0.3, f'{gui[1][:1]}')
        check('planes: and the map its taps and GND vias read caps there',
              gui[1] and gui[1][0][0] == [0.3], f'{gui[1][:1]}')
        same('planes', *cli, *gui)

        print('E. the steps that ran decide')
        first, second = NETS[:5], NETS[5:]
        cli = cli_chain(td, 'f_cli', ('route.py', '--nets', *first, *ROUTE_FLAGS, '--clearance-ceiling', '0.3'),
                        ('route.py', '--nets', *second, *ROUTE_FLAGS))
        steps = [route_step({'clearance_ceiling': 0.3}, first), route_step({}, second)]
        path, _ = plan(td, 'f', steps)
        check('a ceiling on an earlier step holds in the project when the last step has none',
              project(path)[0] == project(cli[0])[0] == {'Default': 0.2, 'Wide': 0.3},
              f'cli {project(cli[0])[0]} plan {project(path)[0]}')

        steps = [route_step({'clearance_ceiling': 0.3}, first), route_step({'clearance_ceiling': 0.25}, second)]
        path, _ = plan(td, 'g', steps, indices=[0])
        check('a step the run did not reach writes nothing',
              project(path)[0] == {'Default': 0.2, 'Wide': 0.3}, f'{project(path)[0]}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print()
    if failures:
        print(f'FAILED: {len(failures)} check(s)')
        for f_ in failures:
            print(f'  {f_}')
        return 1
    print("PASS: #1698 a plan's map and written project match the CLI's")
    return 0


if __name__ == '__main__':
    sys.exit(main())
