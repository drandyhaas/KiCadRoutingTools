"""#1355: the bus step names a missing ortools on the console, in a second.

Without ortools the bus step's fanout, ends model and whole solve each died in a
stage log: route_bus ran minutes, printed `fanout exit 1` / `laid nothing`,
exited 1, and never named the library on the console. The gate is
`startup_checks.check_bus_dependencies`, called by route_bus.py and
whole_route.py before they start, exit 2 with the pip command.

The control arm runs the same command with ortools importable and asserts the
gate does NOT fire (the run is stopped at an argument error right after it, so
no routing is paid for).
"""
import os
import sys
import tempfile
import textwrap

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_utils import ROOT_DIR, check, evidence

BLOCKER = textwrap.dedent('''
    import os, runpy, sys

    class _NoOrtools:
        def find_spec(self, name, path=None, target=None):
            if name == 'ortools' or name.startswith('ortools.'):
                raise ModuleNotFoundError(f"No module named '{name}'", name=name)
            return None

    sys.meta_path.insert(0, _NoOrtools())
    target = os.path.abspath(sys.argv[1])
    sys.argv = sys.argv[1:]
    sys.path.insert(0, os.path.dirname(target))
    runpy.run_path(target, run_name='__main__')
''')

BOARD = os.path.join(ROOT_DIR, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _run_with_blocker(script, tmp, *args, **kw):
    blocker = os.path.join(tmp, 'no_ortools.py')
    with open(blocker, 'w', encoding='utf-8') as f:
        f.write(BLOCKER)
    evidence(blocker, 'the ortools blocker')
    evidence(BOARD, 'the input board')
    return check([sys.executable, '-B', '-X', 'utf8', blocker,
                  os.path.join(ROOT_DIR, 'awx', script), BOARD,
                  os.path.join(tmp, 'out.kicad_pcb'), '--bus', 'U1:U2', *args],
                 **kw)


def test_route_bus_names_ortools_and_exits_2():
    with tempfile.TemporaryDirectory() as tmp:
        r = _run_with_blocker('route_bus.py', tmp, refuse='ortools', code=2,
                              timeout=120)
        assert 'pip install' in (r.stdout + r.stderr), r.stderr[-600:]


def test_whole_route_names_ortools_and_exits_2():
    with tempfile.TemporaryDirectory() as tmp:
        blocker = os.path.join(tmp, 'no_ortools.py')
        with open(blocker, 'w', encoding='utf-8') as f:
            f.write(BLOCKER)
        evidence(blocker, 'the ortools blocker')
        evidence(BOARD, 'the input board')
        check([sys.executable, '-B', '-X', 'utf8', blocker,
               os.path.join(ROOT_DIR, 'awx', 'whole_route.py'), '--help'],
              accept=True, timeout=120)
        r = check([sys.executable, '-B', '-X', 'utf8', blocker,
                   os.path.join(ROOT_DIR, 'awx', 'whole_route.py'), BOARD,
                   os.path.join(tmp, 'out.kicad_pcb')],
                  refuse='ortools', code=2, timeout=120)
        assert 'pip install' in (r.stdout + r.stderr)


def test_gate_passes_when_ortools_is_importable():
    try:
        import ortools  # noqa: F401
    except ImportError:
        print('SKIP: ortools is not installed here')
        return
    sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
    from startup_checks import check_bus_dependencies
    check_bus_dependencies()


if __name__ == '__main__':
    test_route_bus_names_ortools_and_exits_2()
    test_whole_route_names_ortools_and_exits_2()
    test_gate_passes_when_ortools_is_importable()
    print('ok')
