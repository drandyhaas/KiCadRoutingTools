#!/usr/bin/env python3
"""Placement tab Apply (Place + Route) lays TRACKS, never the result board's
copper GRAPHICS.

The parser puts copper graphics into `pcb.segments` as `graphic=True`
(#337 board `gr_*`, #908 footprint `fp_*` -- pad outlines, tabs, antennas).
`PlacementTab._apply_copper` replaces the live board's tracks with the
result's segments, and it used to add every one of them as a PCB_TRACK: each
graphic became a net-0 track over its own footprint's pads (KiCad DRC flags
it) on top of the graphic the live board still draws.

Through the REAL method (ipc-migration: `_apply_ipc`, poses and copper in one
kipy commit) on the fake-IPC board staged from ulx3s, whose RP1/RP3 draw B.Cu
polygons, with the laid tracks read back from the file the fake writes (its
own get_tracks() re-seeds graphics as tracks, which kipy does not): the
tracks added are exactly the non-graphic segments, the
fixture carries graphics (asserted, so the gate cannot go vacuous), and no
added track sits on a graphic's endpoints.

Needs pcbnew; re-execs into KiCad's python automatically. Does not
self-skip: a gate that exits 0 without running reports what it guards as
checked.

    python3 tests/gui_parity/test_placement_apply_graphics.py
"""
import glob
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

from kicad_locate import path_version_key  # noqa: E402
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/"
    "Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
            key=path_version_key, reverse=True),
]

BOARD = os.path.join(REPO, 'kicad_files', 'ulx3s.kicad_pcb')


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        r = subprocess.run([cand, '-c', 'import pcbnew, wx, kipy'], capture_output=True)
        if r.returncode == 0:
            # os.execv re-splits argv on spaces on Windows ("Program Files").
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew+wx+kipy found; this gate does not self-skip")
    sys.exit(2)


def run():
    import shutil
    import tempfile
    import wx
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from fake_ipc_board import install as _install_fake_board
    from kicad_parser import parse_kicad_pcb
    from kicad_routing_plugin.placement_gui import PlacementTab

    app = wx.App(False)  # noqa: F841  (before any wx object)
    failures = []

    def check(label, ok, detail=""):
        print(f"  [{'ok' if ok else 'FAIL'}] {label}" + (f" ({detail})" if detail else ""))
        if not ok:
            failures.append(label)

    result = parse_kicad_pcb(BOARD)
    # ulx3s ships unrouted, so add two real tracks: the method must still lay
    # those (no in-repo board carries both tracks and copper graphics).
    from kicad_parser import Segment
    net_id = next(nid for nid, n in result.nets.items() if nid and n.name)
    x0, y0 = result.board_info.board_bounds[:2]
    result.segments += [
        Segment(start_x=x0 + 5, start_y=y0 + 5, end_x=x0 + 8, end_y=y0 + 5,
                width=0.2, layer='F.Cu', net_id=net_id),
        Segment(start_x=x0 + 8, start_y=y0 + 5, end_x=x0 + 8, end_y=y0 + 9,
                width=0.2, layer='B.Cu', net_id=net_id)]
    graphics = [s for s in result.segments if getattr(s, 'graphic', False)]
    tracks = [s for s in result.segments if not getattr(s, 'graphic', False)]
    check("fixture carries copper graphics and tracks", graphics and tracks,
          f"{len(graphics)} graphic, {len(tracks)} track segments")

    work = tempfile.mkdtemp(prefix='t_apply_graphics_')
    staged = os.path.join(work, os.path.basename(BOARD))
    shutil.copyfile(BOARD, staged)       # the fake writes its commits here
    board = _install_fake_board(staged)
    try:
        added, _vias = PlacementTab._apply_ipc(None, board, result, [], True)
        check("tracks added == non-graphic segments", added == len(tracks),
              f"added {added}, tracks {len(tracks)}, all {len(result.segments)}")
        live = [s for s in parse_kicad_pcb(staged).segments
                if not getattr(s, 'graphic', False)]
    finally:
        shutil.rmtree(work, ignore_errors=True)
    check("live board tracks == non-graphic segments", len(live) == len(tracks),
          f"{len(live)} vs {len(tracks)}")

    def key(x0, y0, x1, y1):
        a = (round(x0, 4), round(y0, 4))
        b = (round(x1, 4), round(y1, 4))
        return (a, b) if a <= b else (b, a)
    graphic_keys = {key(s.start_x, s.start_y, s.end_x, s.end_y) for s in graphics}
    track_keys = {key(s.start_x, s.start_y, s.end_x, s.end_y) for s in tracks}
    on_graphic = [t for t in live
                  if key(t.start_x, t.start_y, t.end_x, t.end_y)
                  in graphic_keys - track_keys]
    check("no track laid on a graphic's endpoints", not on_graphic,
          f"{len(on_graphic)} found")

    if failures:
        print(f"\nVERDICT: FAIL ({len(failures)}): " + "; ".join(failures))
        return 1
    print("\nVERDICT: placement Apply lays tracks only")
    return 0


def main():
    try:
        import pcbnew  # noqa: F401
        import wx  # noqa: F401
        import kipy  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(run())


if __name__ == '__main__':
    main()
