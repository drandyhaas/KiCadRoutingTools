#!/usr/bin/env python3
"""#1556 on every GUI strip site: the live strip is counted and width-preferring.

    python3 tests/gui_parity/test_1556_live_strip_counted.py

(re-execs into KiCad's bundled python automatically, like its siblings, and
does NOT self-skip without it: exit 2)

The CLI writer strips a listed segment through kicad_writer.match_stack_members:
by span, layer and net, COUNTED, the listed width first. #1556 moved the route
and differential tabs onto it; three live strip sites still matched a SET of
(endpoints, net) keys -- the Planes tab's cleanup strips and cleanup delta,
and the oracle's stranded-fragment delete in gui_utils -- so a strip listing
ONE member of a same-span stack removed every member, the keeper too, and
matched across layers. All five now call gui_utils.remove_listed_segments.

Checks, on a real pcbnew board:
  1. A same-span stack of a 0.0762 and a 0.1 track, the 0.1 listed: only the
     0.1 goes (on the old set match both did).
  2. The same span on B.Cu, listed on F.Cu: the B.Cu track stays.
  3. Two listings of a two-member stack: both go; a listing with no member:
     nothing goes.
  4. Every strip site calls the helper (AST over the plugin): no live strip
     matches tracks by a set of endpoint keys any more.
"""

# ---------------------------------------------------------------------------
# NOT RUNNABLE ON ipc-migration. It drives gui_utils.remove_listed_segments on
# a pcbnew BOARD and counts its calls in swig_gui / differential_gui /
# planes_gui / gui_utils. On this branch no tab strips the live board itself:
# every strip goes through the IPC adapter, and that SWIG helper is not here.
#
# WHERE THE CLAIM IS GRADED HERE: tests/test_ipc_live_apply_ports.py (G):
# kicad_ipc_adapter.strip_listed_tracks, through the routing apply and the
# Planes apply's finalize strips, on real kipy tracks -- the listed member of
# a same-span stack, no cross-layer match, counted, and no endpoint-set strip
# left in the adapter.
#
# NOT COVERED: a running KiCad's answer to the remove commit.
#
# main's body is kept below, unreached, so a later sync merges into it.
if __name__ == '__main__':
    import sys as _sys
    print("SKIP: #1556's live strip drives the SWIG helper; ipc-migration grades "
          "kicad_ipc_adapter.strip_listed_tracks in tests/test_ipc_live_apply_ports.py (G); "
          "a live KiCad's remove commit is NOT covered.")
    _sys.exit(77)
# ---------------------------------------------------------------------------
import ast
import glob
import os
import subprocess
import sys

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
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with pcbnew found; this gate does not self-skip, "
          "because a gate that exits 0 without running reports everything it "
          "guards as checked.")
    sys.exit(2)


def main():
    try:
        import pcbnew
    except ImportError:
        _reexec_into_kicad()
    sys.path.insert(0, REPO)
    for sub in ('py_router', 'py_placer', 'py_tools'):
        sys.path.insert(0, os.path.join(REPO, sub))
    sys.path.insert(0, os.path.dirname(REPO))
    from kicad_parser import Segment
    from kicad_routing_plugin.gui_utils import remove_listed_segments

    failures = []

    def check(name, cond, info=""):
        if not cond:
            failures.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {info}" if info else ""))

    def board_with(tracks):
        b = pcbnew.BOARD()
        net = pcbnew.NETINFO_ITEM(b, "N")
        b.Add(net)
        code = net.GetNetCode()
        for (x0, y0, x1, y1, w, layer) in tracks:
            t = pcbnew.PCB_TRACK(b)
            t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(x0), pcbnew.FromMM(y0)))
            t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(x1), pcbnew.FromMM(y1)))
            t.SetWidth(pcbnew.FromMM(w))
            t.SetLayer(layer)
            t.SetNetCode(code)
            b.Add(t)
        return b, code

    def widths(b):
        return sorted((round(pcbnew.ToMM(t.GetWidth()), 4), t.GetLayer())
                      for t in b.GetTracks())

    def seg(code, w, layer='F.Cu', x1=12.0):
        return Segment(start_x=10.0, start_y=10.0, end_x=x1, end_y=10.0,
                       width=w, layer=layer, net_id=code)

    F, B = pcbnew.F_Cu, pcbnew.B_Cu
    print("1. a same-span stack, the 0.1 member listed")
    b, n = board_with([(10, 10, 12, 10, 0.0762, F), (12, 10, 10, 10, 0.1, F)])
    gone = remove_listed_segments(b, pcbnew, [seg(n, 0.1)])
    check("one removed", gone == 1, f"removed {gone}")
    check("the 0.0762 keeper stays", widths(b) == [(0.0762, F)], f"{widths(b)}")

    print("2. the same span on B.Cu, listed on F.Cu")
    b, n = board_with([(10, 10, 12, 10, 0.1, B)])
    gone = remove_listed_segments(b, pcbnew, [seg(n, 0.1, 'F.Cu')])
    check("nothing removed", gone == 0 and widths(b) == [(0.1, B)], f"{widths(b)}")

    print("3. counted")
    b, n = board_with([(10, 10, 12, 10, 0.1, F), (10, 10, 12, 10, 0.1, F)])
    gone = remove_listed_segments(b, pcbnew, [seg(n, 0.1), seg(n, 0.1)])
    check("two listings of a two-member stack remove both", gone == 2 and not widths(b))
    b, n = board_with([(10, 10, 12, 10, 0.1, F)])
    gone = remove_listed_segments(b, pcbnew, [seg(n, 0.1, x1=13.0)])
    check("a listing with no member removes nothing", gone == 0 and len(widths(b)) == 1)

    print("4. every live strip site calls the helper")
    plug = os.path.join(REPO, 'kicad_routing_plugin')
    calls = {}
    setkey = []
    for f in sorted(glob.glob(os.path.join(plug, '*.py'))):
        src = open(f, encoding='utf-8').read()
        tree = ast.parse(src)
        name = os.path.basename(f)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == \
                    'remove_listed_segments':
                calls[name] = calls.get(name, 0) + 1
        # The old shape: a track's (start, end, net) key looked up in a set,
        # with the reversed-ends retry.
        if 'k[2], k[3], k[0], k[1], k[4]' in src or 'a[2], a[3], a[0], a[1], a[4]' in src:
            setkey.append(name)
    want = {'swig_gui.py': 1, 'differential_gui.py': 1, 'planes_gui.py': 2,
            'gui_utils.py': 1}
    check("swig_gui, differential_gui, planes_gui (x2) and gui_utils' oracle "
          "strip call it", all(calls.get(k, 0) >= v for k, v in want.items()),
          f"{calls}")
    check("no endpoint-set strip is left", not setkey, f"{setkey}")

    print()
    if failures:
        print(f"FAILED ({len(failures)}): " + ", ".join(failures))
        return 1
    print("PASS: every live strip is counted, layer-keyed and width-preferring")
    return 0


if __name__ == '__main__':
    sys.exit(main())
