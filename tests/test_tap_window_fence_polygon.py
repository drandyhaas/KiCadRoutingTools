#!/usr/bin/env python3
"""A tap window's trace search stays inside the window on a polygonal board.

`try_tap_pad` (plane taps, and the net rescue's bare-pad dogbone escape) builds
its obstacle maps on `make_local_window`, a copy of the board restricted to a
square around the pad: it loads only the copper within the window plus a 2 mm
item margin and installs the window as the board's bounds. On a rectangular
board the track edge band then fences the window. A polygonal board keeps its
outline, and the outline band blocks only what lies outside the BOARD, so a
window side inside the board was open: past the stamped band the map holds
nothing at all, and the via->pad trace walked out of the window into copper
the window never loaded. Measured on a corpus board: a rescue dogbone of 145
segments crossed a foreign B.Cu track lying beyond the window and its margin.

The board here is a chamfered square with the pad near its middle, so every
side of the window is inside the board.

  A. the cells just past each window side are blocked on the window's track
     map (before the fix they were free -- the open window edge itself);
  B. a via boxed in by a foreign wall whose only opening leads out of the
     window, into a channel a foreign track closes beyond the item margin,
     gets NO trace (before the fix the A* left the window and its trace
     crossed that unloaded track: a short);
  C. control: with the wall between via and pad removed the same window
     routes the trace, inside the window -- so B's refusal is the fence,
     not a sealed map.

    python3 tests/test_tap_window_fence_polygon.py
"""
import contextlib
import io
import math
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence                         # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


_N = [0]


def _uid():
    _N[0] += 1
    return '00000000-0000-4000-8000-%012d' % _N[0]


_HEAD = ('(kicad_pcb\n\t(version 20260206)\n\t(generator "test_window_fence")\n'
         '\t(generator_version "10.0")\n\t(general\n\t\t(thickness 1.6)\n\t)\n'
         '\t(paper "A4")\n\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
         '\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n\t\t(25 "Edge.Cuts" user)\n'
         '\t)\n\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n')

# A chamfered square, 50 mm across: not rectangular, so the window keeps it.
OUTLINE = [(32, 25), (68, 25), (75, 32), (75, 68), (68, 75), (32, 75),
           (25, 68), (25, 32)]
PAD = (50.0, 50.0)
VIA = (53.0, 50.0)
HALF = 4.5            # try_tap_pad's window for a 1.5 mm via search radius
# The box around the via (net WALL): open to the east, where the channel runs
# past the window edge (54.5) and its item margin (56.5) to x = 57.5.
WEST_WALL = (51.0, 49.0, 51.0, 51.0)
CHANNEL = [(51.0, 49.0, 57.5, 49.0), (51.0, 51.0, 57.5, 51.0)]
# Net FAR closes the channel at x = 57, beyond the margin: never loaded.
FAR = (57.0, 49.25, 57.0, 50.75)
W = 0.2


def _seg(s, net):
    return ('\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n'
            '\t\t(layer "F.Cu")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)\n'
            % (s[0], s[1], s[2], s[3], W, net, _uid()))


def _board(west_wall=True):
    pad = ('\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 0.6 0.6)\n'
           '\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(net "P")\n\t\t\t(uuid "%s")\n'
           '\t\t)\n' % _uid())
    fp = ('\t(footprint "TestPoint:TP"\n\t\t(layer "F.Cu")\n\t\t(uuid "%s")\n'
          '\t\t(at %s %s)\n\t\t(property "Reference" "TP1"\n\t\t\t(at 0 -2 0)\n'
          '\t\t\t(layer "F.Mask")\n\t\t\t(uuid "%s")\n\t\t)\n%s\t)\n'
          % (_uid(), PAD[0], PAD[1], _uid(), pad))
    walls = ([WEST_WALL] if west_wall else []) + CHANNEL
    edges = ''.join('\t(gr_line\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(stroke\n'
                    '\t\t\t(width 0.05)\n\t\t\t(type default)\n\t\t)\n'
                    '\t\t(layer "Edge.Cuts")\n\t\t(uuid "%s")\n\t)\n'
                    % (a[0], a[1], b[0], b[1], _uid())
                    for a, b in zip(OUTLINE, OUTLINE[1:] + OUTLINE[:1]))
    return (_HEAD + fp + ''.join(_seg(s, 'WALL') for s in walls)
            + _seg(FAR, 'FAR') + edges + ')\n')


def _seg_seg_d(a, b):
    from net_rescue import _seg_seg_d as d
    return d(*a, *b)


def _window_and_map(pcb):
    from plane_obstacle_builder import build_routing_obstacle_map
    from plane_pad_tap import make_local_window
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(track_width=0.15, clearance=0.15, via_size=0.45,
                          via_drill=0.2, grid_step=0.1, layers=['F.Cu', 'B.Cu'],
                          board_edge_clearance=0.3)
    nid = next(n for n, net in pcb.nets.items() if net.name == 'P')
    win = make_local_window(pcb, PAD[0], PAD[1], HALF)
    obs = quiet(build_routing_obstacle_map, win, cfg, nid, 'F.Cu',
                skip_pad_blocking=False, verbose=False)
    return cfg, nid, win, obs


def _trace(pcb):
    from kicad_parser import Pad
    from route_planes import route_via_to_pad
    cfg, nid, win, obs = _window_and_map(pcb)
    pad = next(p for p in pcb.pads_by_net[nid])
    assert isinstance(pad, Pad)
    # Generous iterations: the question is where the search may go, not how
    # long it may take.
    res = quiet(route_via_to_pad, VIA, pad, 'F.Cu', nid, obs, cfg,
                max_iterations=200000, return_blocked_cells=True)
    return win, res


def main():
    from kicad_parser import parse_kicad_pcb
    from plane_obstacle_builder import _is_rectangular_outline

    d = tempfile.mkdtemp(prefix='twf_')
    try:
        boards = {}
        for name, west in (('boxed', True), ('open', False)):
            path = os.path.join(d, name + '.kicad_pcb')
            with open(path, 'w', encoding='utf-8', newline='\n') as f:
                f.write(_board(west))
            boards[name] = quiet(parse_kicad_pcb, evidence(path, 'staged board'))
        pcb = boards['boxed']
        bi = pcb.board_info

        print('preconditions')
        cfg, nid, win, obs = _window_and_map(pcb)
        wb = win.board_info.board_bounds
        check('the outline is not rectangular (the polygon branch runs)',
              len(bi.board_outline or []) == len(OUTLINE)
              and not _is_rectangular_outline(bi.board_outline, bi.board_bounds),
              str(bi.board_outline))
        check('the window keeps the outline', bool(win.board_info.board_outline))
        check('every window side is inside the board',
              all(abs(a - b) > 1 for a, b in zip(wb, bi.board_bounds)),
              f'window {wb} board {bi.board_bounds}')
        far_net = next(n for n, net in pcb.nets.items() if net.name == 'FAR')
        wall_net = next(n for n, net in pcb.nets.items() if net.name == 'WALL')
        check('the window loads the wall but not the track beyond its margin',
              any(s.net_id == wall_net for s in win.segments)
              and not any(s.net_id == far_net for s in win.segments))

        print('A. the window edge is fenced')
        g = cfg.grid_step
        lo_x, lo_y = round(wb[0] / g), round(wb[1] / g)
        hi_x, hi_y = round(wb[2] / g), round(wb[3] / g)
        ring = ([(lo_x - 1, gy) for gy in range(lo_y, hi_y + 1)]
                + [(hi_x + 1, gy) for gy in range(lo_y, hi_y + 1)]
                + [(gx, lo_y - 1) for gx in range(lo_x, hi_x + 1)]
                + [(gx, hi_y + 1) for gx in range(lo_x, hi_x + 1)])
        open_cells = [(round(x * g, 2), round(y * g, 2)) for x, y in ring
                      if not obs.is_blocked(x, y, 0)]
        check('every cell just past the window sides is blocked',
              not open_cells, f'{len(open_cells)} of {len(ring)} free, '
                              f'e.g. {open_cells[:4]}')

        print('B. a via boxed in toward the window edge')
        win, res = _trace(pcb)
        pts = [p for s in (res.segments or []) for p in (s['start'], s['end'])]
        outside = [p for p in pts if not (wb[0] - 1e-6 <= p[0] <= wb[2] + 1e-6
                                          and wb[1] - 1e-6 <= p[1] <= wb[3] + 1e-6)]
        shorts = [s for s in (res.segments or [])
                  if _seg_seg_d((*s['start'], *s['end']), FAR)
                  < (s['width'] + W) / 2 - 1e-6]
        check('no trace leaves the window', not outside,
              f'{len(outside)} trace point(s) outside, e.g. {outside[:3]}')
        check('no trace touches the unloaded foreign track', not shorts,
              f'{len(shorts)} segment(s) on net FAR')
        check('the boxed via gets no trace (its only way out is the fence)',
              not res.success,
              f'{len(res.segments or [])} segment(s) routed')

        print('C. control: the same window with the via open toward the pad')
        win, res = _trace(boards['open'])
        pts = [p for s in (res.segments or []) for p in (s['start'], s['end'])]
        check('the open via is traced to the pad', res.success and bool(pts),
              f'{len(res.segments or [])} segment(s)')
        check('inside the window',
              all(wb[0] <= p[0] <= wb[2] and wb[1] <= p[1] <= wb[3] for p in pts))
    finally:
        shutil.rmtree(d, ignore_errors=True)

    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s): {FAILS}')
        return 1
    print('PASS: a polygonal board\'s tap window is fenced at its bounds')
    return 0


if __name__ == '__main__':
    sys.exit(main())
