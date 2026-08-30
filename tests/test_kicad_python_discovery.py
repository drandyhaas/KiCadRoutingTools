"""KiCad-python discovery + the exact-fill "unavailable" contract (#647).

Two defects this pins, both of which made every routing step after
route_planes.py silently route against a WRONG model of the pours:

1. `find_kicad_python()` looked for Windows' interpreter at the unversioned
   `C:\\Program Files\\KiCad\\bin\\python.exe`. No KiCad >= 6 installs there
   (it is `...\\KiCad\\10.0\\bin\\python.exe`), so the finder returned None for
   every Windows user and EVERY exact-fill consumer -- plane fragility, the
   oracle, check_connected, route_planes' stitch validation -- degraded to its
   raster approximation. It also trusted that a path EXISTS rather than that it
   can import pcbnew, so a KiCad-less macOS box "found" /usr/bin/python3.

2. `refill_islands()` returns None by contract when it is unavailable;
   plane_fragility called `.items()` on it regardless. The AttributeError was
   caught and printed as "exact refill unavailable ('NoneType' object has no
   attribute 'items')" -- a swallowed exception that read like a per-board
   quirk while it was really a whole-platform capability gap.

No pcbnew required: the platform layouts are simulated.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

import kicad_exact_fill as kef  # noqa: E402
import plane_fragility as pf  # noqa: E402


def test_windows_versioned_layout_sorts_newest_first():
    """KiCad 10 must outrank KiCad 9 -- string sort puts "9.0" above "10.0"."""
    paths = [r'C:\Program Files\KiCad\9.0\bin\python.exe',
             r'C:\Program Files\KiCad\10.0\bin\python.exe',
             r'C:\Program Files\KiCad\8.0\bin\python.exe']
    best = sorted(paths, key=kef._win_version_key, reverse=True)[0]
    assert '10.0' in best, best
    # The unversioned path is the LAST resort, never a winner.
    assert kef._win_version_key(r'C:\Program Files\KiCad\bin\python.exe') \
        < kef._win_version_key(paths[0])
    # Separator-agnostic on purpose: an os.path-based key collapses to a
    # constant off-Windows, which is untestable anywhere it could regress.
    assert kef._win_version_key('C:/Program Files/KiCad/10.0/bin/python.exe') \
        == (10, 0)


def test_candidates_cover_the_real_windows_install():
    """The versioned layout and both Program Files roots are searched.

    This asserted the module SOURCE contained 'Program Files (x86)' and a
    `'*', 'bin'` glob. That is a prose scan: it passes just as happily when the
    strings sit in a comment, and it broke the moment #763 moved the platform
    knowledge into `kicad_locate` -- while the behaviour it named was intact.
    Asserted behaviourally now, against a simulated Windows tree, which also
    covers the drives the old wording could not describe.
    """
    import kicad_locate as kl

    tree = {r'C:\Program Files', r'C:\Program Files\KiCad',
            r'C:\Program Files\KiCad\10.0', r'C:\Program Files\KiCad\10.0\bin',
            r'C:\Program Files (x86)', r'C:\Program Files (x86)\KiCad',
            r'C:\Program Files (x86)\KiCad\9.0'}
    got = kl.kicad_python_candidates(
        system='Windows', environ={'ProgramFiles': r'C:\Program Files',
                                   'ProgramFiles(x86)': r'C:\Program Files (x86)'},
        globber=lambda pat: sorted(
            d for d in tree
            if len(d.split('\\')) == len(pat.split('\\'))
            and all(a == b or b == '*'
                    for a, b in zip(d.split('\\'), pat.split('\\')))),
        isdir=lambda p: p in tree, drives=[], winreg_mod=False)
    assert r'C:\Program Files\KiCad\10.0\bin\python.exe' in got, got
    assert r'C:\Program Files (x86)\KiCad\9.0\bin\python.exe' in got, got
    # ...and the newest version comes first, or a KiCad 10 user is handed 9.
    assert got.index(r'C:\Program Files\KiCad\10.0\bin\python.exe') < \
        got.index(r'C:\Program Files (x86)\KiCad\9.0\bin\python.exe'), got
    # sys.executable is a candidate: under the GUI plugin, THIS interpreter is
    # KiCad's python and needs no search at all.
    assert sys.executable in kef.kicad_python_candidates()


def test_gui_host_binary_is_never_a_candidate():
    """Inside KiCad's embedded python (the GUI plugin) `sys.executable` is the
    pcbnew HOST BINARY, not python3. It must never reach the candidate list:
    subprocess.run([pcbnew, script, board, out, dir]) does not run a script --
    pcbnew opens every argument as a FILE, so the exact-fill refill re-launched
    the APPLICATION once per call and KiCad reported
    "Pcbnew:OpenProjectFiles() takes a single filename" (the `out` path in that
    argv does not exist yet -- the empty filename in the message)."""
    import types
    memo, real = list(kef._KICAD_PYTHON_MEMO), sys.executable
    had_pcbnew = 'pcbnew' in sys.modules
    try:
        kef._KICAD_PYTHON_MEMO.clear()
        sys.modules.setdefault('pcbnew', types.ModuleType('pcbnew'))
        for host in ("/Applications/KiCad/KiCad.app/Contents/MacOS/pcbnew",
                     r"C:\Program Files\KiCad\10.0\bin\pcbnew.exe",
                     "/usr/bin/pcbnew"):
            sys.executable = host
            assert host not in kef.kicad_python_candidates(), \
                f"host binary {host} leaked into the candidate list"
            kef._KICAD_PYTHON_MEMO.clear()
            got = kef.find_kicad_python()
            assert got != host, f"find_kicad_python() returned the host binary {host}"
            assert got is None or \
                os.path.basename(got).lower().startswith('python'), \
                f"find_kicad_python() returned a non-python: {got}"
    finally:
        sys.executable = real
        if not had_pcbnew:
            sys.modules.pop('pcbnew', None)
        kef._KICAD_PYTHON_MEMO.clear()
        kef._KICAD_PYTHON_MEMO.extend(memo)
    print("  PASS: the pcbnew host binary never reaches a subprocess spawn")


def test_finder_verifies_pcbnew_rather_than_file_existence():
    """A path that exists but cannot import pcbnew is not an answer."""
    memo, env = list(kef._KICAD_PYTHON_MEMO), os.environ.get('KICAD_PYTHON')
    try:
        kef._KICAD_PYTHON_MEMO.clear()
        # A real, executable python that certainly lacks pcbnew: this test's
        # own interpreter would qualify only if pcbnew were importable here.
        os.environ['KICAD_PYTHON'] = os.path.abspath(__file__)  # not a python
        got = kef.find_kicad_python()
        assert got != os.path.abspath(__file__), \
            "returned an unverified path"
    finally:
        kef._KICAD_PYTHON_MEMO.clear()
        kef._KICAD_PYTHON_MEMO.extend(memo)
        if env is None:
            os.environ.pop('KICAD_PYTHON', None)
        else:
            os.environ['KICAD_PYTHON'] = env


def test_fragility_survives_unavailable_refill(capsys_print=None):
    """The #647 crash: refill_islands -> None must not reach .items().

    Asserts the degraded path is reported as a CAUSE the operator can act on,
    not as an AttributeError string.
    """
    import io
    import contextlib

    import numpy as np
    from kicad_parser import PCBData, BoardInfo, Net, Zone
    from routing_config import GridRouteConfig

    zone = Zone(net_id=1, net_name='GND', layer='In1.Cu',
                polygon=[(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)])
    pcb = PCBData(
        board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'In1.Cu'],
                             board_bounds=(0, 0, 20, 20)),
        nets={1: Net(net_id=1, name='GND')}, footprints={}, vias=[],
        segments=[], pads_by_net={}, zones=[zone])
    # A source_path that EXISTS sends the code down the refill branch, which
    # is the branch that crashed.
    pcb.source_path = os.path.abspath(__file__)
    cfg = GridRouteConfig(track_width=0.2, clearance=0.2, via_size=0.6,
                          via_drill=0.3, grid_step=0.1,
                          layers=['F.Cu', 'In1.Cu'])

    memo = list(kef._KICAD_PYTHON_MEMO)
    try:
        kef._KICAD_PYTHON_MEMO.clear()
        kef._KICAD_PYTHON_MEMO.append(None)   # simulate: nothing found
        kef._REFILL_MEMO.clear()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            cells = pf.compute_plane_fragility_cells(pcb, cfg)
        out = buf.getvalue()
    finally:
        kef._KICAD_PYTHON_MEMO.clear()
        kef._KICAD_PYTHON_MEMO.extend(memo)
        kef._REFILL_MEMO.clear()

    assert 'NoneType' not in out, f"swallowed AttributeError is back: {out}"
    assert 'no python with pcbnew found' in out, out
    assert 'KICAD_PYTHON' in out, "message must name the operator's remedy"
    # Still degrades usefully rather than dying: outlines produce a field.
    assert isinstance(cells, np.ndarray) and len(cells) > 0


def main():
    test_windows_versioned_layout_sorts_newest_first()
    test_candidates_cover_the_real_windows_install()
    test_gui_host_binary_is_never_a_candidate()
    test_finder_verifies_pcbnew_rather_than_file_existence()
    test_fragility_survives_unavailable_refill()
    print("PASS: KiCad-python discovery + exact-fill unavailable contract")


if __name__ == '__main__':
    main()
