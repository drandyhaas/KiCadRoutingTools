#!/usr/bin/env python3
"""#962 GUI gate: the dialog's pre-step re-sync moves the paste openings with
the parts.

The GUI keeps one PCBData across plan steps and re-reads the live board
before each (RoutingDialog._sync_pcb_data_from_board): copper, zones,
footprint and pad poses, `sense:` groups. It never re-read
`pcb.paste_apertures`, so after a part moved in the session (optimize_caps,
the placement Apply, the user) its openings stayed where it had been, and
the via-in-paste keep-out (obstacle_map) and the Type VII stamping
(fab_notes) read them there. The CLI re-parses the board every step.

On esp_prog, U2 (whose F.Paste tab is a GRAPHIC opening) moves +3/+2 mm and
C3 (two pad openings) +1 mm and 90 degrees on the live board. The dialog's
REAL sync then runs, and its openings must equal a fresh
build_pcb_data_from_board of the moved board, opening for opening (owner,
layer, source, pad, geometry). The negative control: before the sync they
differ, so the moves do move openings.

Needs pcbnew and wx; re-execs into KiCad's python automatically.

    python3 -X utf8 tests/gui_parity/test_962_paste_resync.py
"""
import contextlib
import glob
import io
import os
import subprocess
import sys
from collections import Counter
from types import SimpleNamespace

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
           key=path_version_key, reverse=True),
]
BOARD = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
MOVES = (('U2', 3.0, 2.0, 0.0), ('C3', 1.0, 0.0, 90.0))
FAILURES = []


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            argv = [cand, '-X', 'utf8', os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew found")
    sys.exit(0)


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + detail) if detail else ''))
        FAILURES.append(what)


def _q(v):
    return round(float(v), 4)


def _sig(ap):
    """An opening as a consumer reads it: whose, where, and its shape."""
    if ap.shape_pad is not None:
        p = ap.shape_pad
        geom = (_q(p.global_x), _q(p.global_y), _q(p.size_x), _q(p.size_y),
                _q(getattr(p, 'rect_rotation', 0.0) or 0.0))
    elif ap.circle is not None:
        geom = tuple(_q(v) for v in ap.circle)
    else:
        geom = tuple(tuple((_q(x), _q(y)) for x, y in ring) for ring in ap.rings)
    return (ap.owner_ref, ap.layer, ap.source, ap.pad_number, geom)


def _openings(pcb):
    return Counter(_sig(a) for a in pcb.paste_apertures or [])


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import pcbnew
    for _p in ('', 'py_router', 'py_placer', 'py_tools'):
        _d = os.path.join(REPO, _p)
        if _d not in sys.path:
            sys.path.insert(0, _d)
    from kicad_parser import build_pcb_data_from_board
    import kicad_routing_plugin.swig_gui as swig_gui
    print('KiCad build: %s' % pcbnew.GetBuildVersion())
    if not os.path.exists(BOARD):
        print('FAIL: fixture missing: %s' % BOARD)
        return 1

    with contextlib.redirect_stdout(io.StringIO()):
        board = pcbnew.LoadBoard(BOARD)
        live = build_pcb_data_from_board(board)        # the dialog's PCBData
    moved_refs = {ref for ref, *_ in MOVES}
    own = [a for a in live.paste_apertures if a.owner_ref in moved_refs]
    check(any(a.source == 'graphic' for a in own) and any(a.source == 'pad' for a in own),
          'the moved parts own a graphic opening and pad openings',
          'sources %s' % sorted({a.source for a in own}))

    for ref, dx, dy, drot in MOVES:
        fp = board.FindFootprintByReference(ref)
        p = fp.GetPosition()
        fp.SetPosition(pcbnew.VECTOR2I(p.x + pcbnew.FromMM(dx), p.y + pcbnew.FromMM(dy)))
        if drot:
            fp.SetOrientationDegrees(fp.GetOrientationDegrees() + drot)
    with contextlib.redirect_stdout(io.StringIO()):
        fresh = build_pcb_data_from_board(board)       # what the CLI would read

    stale = sum((_openings(fresh) - _openings(live)).values())
    check(stale > 0, 'before the sync the openings are stale (the moves move openings)',
          'no opening differs')

    fake = SimpleNamespace(pcb_data=live, _connectivity_cache={})
    real_get_board = pcbnew.GetBoard
    pcbnew.GetBoard = lambda: board
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            swig_gui.RoutingDialog._sync_pcb_data_from_board(fake)
    finally:
        pcbnew.GetBoard = real_get_board
    check('Warning' not in out.getvalue(), 'the sync ran without a warning',
          out.getvalue()[-600:])
    a, b = _openings(live), _openings(fresh)
    check(a == b, 'after the sync the openings equal a fresh build (%d openings, '
          '%d had moved)' % (sum(b.values()), stale),
          'only synced %s; only fresh %s' % (sorted((a - b).elements())[:3],
                                             sorted((b - a).elements())[:3]))

    print('\n%d failure(s)' % len(FAILURES))
    for f in FAILURES:
        print('  FAILED: %s' % f)
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
