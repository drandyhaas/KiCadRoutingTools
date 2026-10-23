#!/usr/bin/env python3
"""--layer-costs must give one value per routing layer; any other count is
refused, never shifted onto the wrong layers (found reviewing #1269).

The costs are positional (entry i prices layers[i]) and every engine then
APPENDS the board's unrequested copper layers at FORBIDDEN cost (-1) after
them. A short list moved each appended -1 one layer early: `batch_route(
layers=[F.Cu, B.Cu], layer_costs=[1.0])` on a 4-layer board built the config
[F.Cu, B.Cu, In1.Cu, In2.Cu] at [1000, -1, -1, 1000] -- the requested B.Cu
forbidden, the unrequested In2.Cu routable. A long list priced the
unrequested layers instead of forbidding them. bga_fanout already refused
both; route.py, route_diff.py, route_planes.py and repair_planes.py now do.

Checks:
  1. Each engine refuses a short and a long list, naming the layers and the
     values (batch_route, batch_route_diff_pairs and repair_planes raise
     LayerCostsCountError; create_plane prints the ERROR and writes nothing).
     No engine reaches its GridRouteConfig.
  2. Controls: an equal-length list and no list at all build exactly the
     config they did before -- requested layers keep their costs and the
     unrequested ones are appended at -1.
  3. The CLIs refuse with a one-line ERROR (no traceback) and exit 1.
  4. The GUI's alignment (gui_utils.layer_costs_for_layers): the Layer Costs
     field's values are placed BY NAME on a tab's layers, from one value per
     copper layer or per selected layer; a count matching neither passes
     through for the engine to judge.

    python3 tests/test_layer_costs_count.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool  # noqa: E402

BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'qfn_csi_underpad_diff.kicad_pcb'),
                 '4-layer fixture board')
NET = 'CSI_CLK_N'
REASON = 'needs one value per routing layer'
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + str(detail)) if detail else ''}")
    if not ok:
        failures.append(label)


class _Stop(Exception):
    pass


def _engine(module, call):
    """Run `call` stopped where the engine first holds its routing config:
    at `module.GridRouteConfig` construction, or -- for repair_planes, which
    sets layer_costs on the config after building it -- at the
    install_layer_clearances call that first reads it. Returns ('config',
    {layers, layer_costs}) when the engine got that far, ('refused', message)
    when it refused, ('error', repr) otherwise."""
    import kicad_dru
    seen = {}
    if module.__name__ == 'repair_planes':
        patched, attr = kicad_dru, 'install_layer_clearances'

        def capture(config, *a, **kw):
            seen.update(layers=config.layers, layer_costs=config.layer_costs)
            raise _Stop
    else:
        patched, attr = module, 'GridRouteConfig'

        def capture(*a, **kw):
            seen.update(kw)
            raise _Stop
    real = getattr(patched, attr)
    setattr(patched, attr, capture)
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            res = call()
    except _Stop:
        return 'config', seen
    except Exception as e:                      # noqa: BLE001
        from routing_exceptions import LayerCostsCountError
        if isinstance(e, LayerCostsCountError):
            return 'refused', str(e)
        return 'error', f"{type(e).__name__}: {e}"
    finally:
        setattr(patched, attr, real)
    text = out.getvalue()
    line = next((l for l in text.splitlines() if REASON in l), None)
    if line is not None:
        return 'refused', line
    return 'error', f"returned {res!r} without a config or a refusal"


def _cfg(kw):
    return kw.get('layers'), kw.get('layer_costs')


def engines():
    import route
    import route_diff
    import route_planes
    import repair_planes
    devnull = os.devnull

    runs = {
        'batch_route': (route, lambda lc, layers: route.batch_route(
            BOARD, devnull, [], layers=layers, layer_costs=lc)),
        'batch_route_diff_pairs': (route_diff, lambda lc, layers:
                                   route_diff.batch_route_diff_pairs(
                                       BOARD, devnull, [], layers=layers, layer_costs=lc)),
        'create_plane': (route_planes, lambda lc, layers: route_planes.create_plane(
            input_file=BOARD, output_file='', net_names=[NET], plane_layers=['In1.Cu'],
            all_layers=layers, layer_costs=lc, dry_run=True, return_results=True)),
        'repair_planes': (repair_planes, lambda lc, layers: repair_planes.repair_planes(
            BOARD, '', [NET], ['In1.Cu'], dry_run=True, routing_layers=layers,
            layer_costs=lc)),
    }
    print("1. every engine refuses a short and a long list")
    for name, (module, call) in runs.items():
        layers = ['F.Cu', 'In1.Cu', 'B.Cu'] if name == 'create_plane' else ['F.Cu', 'B.Cu']
        n = len(layers)
        for kind, lc in (('short', [1.0] * (n - 1)), ('long', [1.0] * n + [3.0])):
            got, what = _engine(module, lambda: call(lc, layers))
            ok = got == 'refused' and f"{n} layer(s)" in what and f"got {len(lc)}" in what
            check(f"{name}: a {kind} list ({len(lc)} for {n} layers) is refused",
                  ok, what if got != 'config' else
                  f"built the config anyway: layers {_cfg(what)[0]} at {_cfg(what)[1]}")

    print("2. controls: equal-length and no list build the config they always did")
    want = {
        ('batch_route', 'equal'): (['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu'], [1.0, 5.0, -1, -1]),
        ('batch_route', 'none'): (['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu'], [1.0, 3.0, -1, -1]),
        ('batch_route', 'full'): (['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], [1.0, 2.0, 3.0, 4.0]),
        ('batch_route_diff_pairs', 'equal'): (['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu'],
                                              [1.0, 5.0, -1, -1]),
        ('batch_route_diff_pairs', 'none'): (['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu'],
                                             [1.0, 1.0, -1, -1]),
        ('create_plane', 'equal'): (['F.Cu', 'In1.Cu', 'B.Cu', 'In2.Cu'], [1.0, 5.0, 2.0, -1]),
        ('create_plane', 'none'): (['F.Cu', 'In1.Cu', 'B.Cu', 'In2.Cu'], [1.0, 3.0, 3.0, -1]),
        ('repair_planes', 'equal'): (['F.Cu', 'B.Cu'], [1.0, 5.0]),
    }
    args = {
        ('batch_route', 'equal'): (['F.Cu', 'B.Cu'], [1.0, 5.0]),
        ('batch_route', 'none'): (['F.Cu', 'B.Cu'], []),
        ('batch_route', 'full'): (['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], [1.0, 2.0, 3.0, 4.0]),
        ('batch_route_diff_pairs', 'equal'): (['F.Cu', 'B.Cu'], [1.0, 5.0]),
        ('batch_route_diff_pairs', 'none'): (['F.Cu', 'B.Cu'], None),
        ('create_plane', 'equal'): (['F.Cu', 'In1.Cu', 'B.Cu'], [1.0, 5.0, 2.0]),
        ('create_plane', 'none'): (['F.Cu', 'In1.Cu', 'B.Cu'], []),
        ('repair_planes', 'equal'): (['F.Cu', 'B.Cu'], [1.0, 5.0]),
    }
    for (name, kind), (layers, lc) in args.items():
        module, call = runs[name]
        got, what = _engine(module, lambda: call(lc, layers))
        built = (list(what.get('layers') or []), list(what.get('layer_costs') or [])) \
            if got == 'config' else None
        check(f"{name}: {kind} costs {lc} on {layers}",
              built == want[(name, kind)], built if got == 'config' else what)


def clis():
    print("3. the CLIs refuse with a clean ERROR line")
    tmp = tempfile.mkdtemp(prefix='t_lcount_')
    try:
        out = os.path.join(tmp, 'out.kicad_pcb')
        cases = [
            ('route.py', [BOARD, out, '--nets', NET, '--layers', 'F.Cu', 'B.Cu',
                          '--layer-costs', '1.0']),
            ('route.py', [BOARD, out, '--nets', NET, '--layer-costs', '1.0', '3.0']),
            ('route_diff.py', [BOARD, out, '--nets', '*CLK*', '--layers', 'F.Cu', 'B.Cu',
                               '--layer-costs', '1.0', '3.0', '3.0']),
            ('route_planes.py', [BOARD, out, '--nets', NET, '--plane-layers', 'In1.Cu',
                                 '--layer-costs', '1.0', '3.0']),
            ('repair_planes.py', [BOARD, out, '--nets', NET, '--plane-layers', 'In1.Cu',
                                  '--layers', 'F.Cu', 'B.Cu', '--layer-costs', '1.0']),
        ]
        for script, argv in cases:
            label = f"{script} {' '.join(a for a in argv[2:] if a not in (BOARD, out))}"
            try:
                r = run_check([sys.executable, '-X', 'utf8', tool(script)] + argv,
                              refuse=REASON, code=1)
                line = next((l for l in (r.stdout + r.stderr).splitlines() if REASON in l), '')
                check(label, line.startswith('ERROR'), line.strip()[:160])
            except AssertionError as e:
                check(label, False, str(e).splitlines()[0][:200])
    finally:
        import shutil
        shutil.rmtree(tmp, ignore_errors=True)


def gui_alignment():
    print("4. GUI: the Layer Costs field placed on a tab's layers by name")
    try:
        from kicad_routing_plugin.gui_utils import layer_costs_for_layers
    except ImportError as e:
        check("gui_utils.layer_costs_for_layers exists", False, e)
        return
    copper = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    typed = [1.0, 6.0, 1.5, 2.0]                     # one per copper layer
    plane_layers = ['F.Cu', 'In1.Cu', 'B.Cu']        # the planes tab's own list
    rows = [
        ("copper-ordered values onto the planes tab's layers",
         (typed, plane_layers, copper, copper), [1.0, 6.0, 2.0]),
        ("copper-ordered values onto a selected subset",
         (typed, ['F.Cu', 'B.Cu'], ['F.Cu', 'B.Cu'], copper), [1.0, 2.0]),
        ("selected-ordered values onto the selected layers (as typed)",
         ([1.0, 4.0], ['F.Cu', 'B.Cu'], ['F.Cu', 'B.Cu'], copper), [1.0, 4.0]),
        ("selected-ordered values onto the planes tab's layers",
         ([1.0, 5.0, 4.0], plane_layers, ['F.Cu', 'In1.Cu', 'B.Cu'], copper),
         [1.0, 5.0, 4.0]),
        ("a count matching neither passes through for the engine",
         ([1.0, 2.0, 3.0], ['F.Cu', 'B.Cu'], ['F.Cu', 'B.Cu'], copper), [1.0, 2.0, 3.0]),
        ("an empty field", ([], plane_layers, copper, copper), []),
    ]
    for label, a, want in rows:
        got = layer_costs_for_layers(*a)
        check(label, got == want, got)
    try:
        got = layer_costs_for_layers([1.0, 2.0], plane_layers, ['F.Cu', 'B.Cu'], copper)
        check("selected-ordered values cannot price an unselected pour layer",
              False, f"returned {got}")
    except ValueError as e:
        check("selected-ordered values cannot price an unselected pour layer",
              'In1.Cu' in str(e), e)


def main():
    engines()
    clis()
    gui_alignment()
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
