"""route_planes.py accepts --net-clearances <json> like route.py and route_diff.py.

Before: a pipeline handing every step the same per-net clearance file had
route.py and route_diff.py open it, while route_planes.py exited 2 with
"unrecognized arguments: --net-clearances" -- the plane step of a chain never
ran. The file's map is used AS GIVEN (no net class, no ceiling), as the two
routing mains use it: it reaches create_plane's plane taps and fill model,
and create_plane caps only the map it auto-reads itself. (The --add-gnd-vias
pass is held to the file by test_gnd_vias_net_class_clearance.py.)
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'py_router'))

BOARD = ROOT / 'tests' / 'fixtures' / 'island_semantics' / 'two_islands_tht.kicad_pcb'


class _Stop(Exception):
    pass


def _nc_file(d, mapping):
    path = os.path.join(d, 'nc.json')
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(mapping, fh)
    return path


def test_cli_accepts_and_loads_the_file():
    with tempfile.TemporaryDirectory() as d:
        path = _nc_file(d, {'SIG': 0.3, 'NOT_ON_BOARD': 0.9})
        res = subprocess.run([sys.executable, str(ROOT / 'py_router' / 'route_planes.py'),
                              str(BOARD), '--nets', 'GND', '--plane-layers', 'B.Cu',
                              '--dry-run', '--net-clearances', path],
                             capture_output=True, text=True, timeout=600)
    assert res.returncode == 0, res.stdout[-2000:] + res.stderr[-2000:]
    assert 'Loaded per-net clearances for 1/' in res.stdout, res.stdout[-2000:]


def test_main_hands_the_map_to_create_plane_by_net_id():
    import route_planes
    from kicad_parser import parse_kicad_pcb
    sig = next(nid for nid, n in parse_kicad_pcb(str(BOARD)).nets.items() if n.name == 'SIG')
    seen = {}

    def spy(**kw):
        seen.update(kw)
        raise _Stop()
    real, argv = route_planes.create_plane, sys.argv
    with tempfile.TemporaryDirectory() as d:
        path = _nc_file(d, {'SIG': 0.3})
        route_planes.create_plane = spy
        sys.argv = ['route_planes.py', str(BOARD), os.path.join(d, 'o.kicad_pcb'),
                    '--nets', 'GND', '--plane-layers', 'B.Cu',
                    '--clearance-ceiling', '0.1', '--net-clearances', path]
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                route_planes.main()
        except _Stop:
            pass
        finally:
            route_planes.create_plane, sys.argv = real, argv
    assert seen.get('net_clearances') == {sig: 0.3}, seen.get('net_clearances')


def test_create_plane_does_not_cap_a_passed_map():
    """--clearance-ceiling caps the class map create_plane auto-reads; a map
    the caller passes is used as given (route.py's --net-clearances contract)."""
    import plane_fill_model
    import route_planes
    got = []

    def stop(pcb_data, net_clearances):
        got.append(net_clearances)
        raise _Stop()
    real = plane_fill_model.set_board_net_clearances
    plane_fill_model.set_board_net_clearances = stop
    try:
        with tempfile.TemporaryDirectory() as d, \
                contextlib.redirect_stdout(io.StringIO()):
            try:
                route_planes.create_plane(
                    str(BOARD), os.path.join(d, 'o.kicad_pcb'), ['GND'], ['B.Cu'],
                    dry_run=True, net_clearances={1: 0.3},
                    clamp_netclasses=True, clearance_ceiling=0.1)
            except _Stop:
                pass
    finally:
        plane_fill_model.set_board_net_clearances = real
    assert got == [{1: 0.3}], got


TESTS = [test_cli_accepts_and_loads_the_file,
         test_main_hands_the_map_to_create_plane_by_net_id,
         test_create_plane_does_not_cap_a_passed_map]

if __name__ == '__main__':
    for t in TESTS:
        t()
        print(f'ok {t.__name__}')
    print('ALL PASS')
