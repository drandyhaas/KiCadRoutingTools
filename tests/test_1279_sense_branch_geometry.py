#!/usr/bin/env python3
"""The geometry behind a `sense:<REF>.<PAD>` branch is the board's real copper (#1279).

tests/test_1279_sense_branches.py proves the rule on the sense_star fixture
(one straight 0.2 mm trace into the middle of a 1 x 1 mm rect join pad).
This file holds the rule to KiCad's own geometry where the fixture does not
reach, each case built in-test from the fixture's text:

- a load track crossing the sense trace in an X is a tap (KiCad's
  connectivity joins crossing copper, not only endpoints);
- a same-net pad the sense trace runs across off-centre is a sense pad, so
  no load is routed through it onto the trace;
- a round (or roundrect) join pad is its copper, not its bounding box;
- a member drawn across the join pad with both ends outside it touches it;
- a blind member via has copper on its span only: it neither touches nor
  blocks copper on the layers it does not reach;
- a join pad no wider than the track (an 0402 cap pad under a 0.5 mm power
  track, off the routing grid) is still landable: the run routes at its
  clearance, narrows nothing and writes no lower clearance to the project;
- a member leaving a tilted join pad obliquely is held to the rule from where
  its COPPER leaves the pad, not its centreline;
- a same-net pour taps the branch where KiCad can fill: not inside the zone's
  cut-out or a pour keep-out around the trace, but over a strip of a sense
  pad whose centre it misses;
- a member uuid matching no item is dropped, as KiCad drops it on load, and
  named in the branch's warnings;
- a contact is judged where the copper meets, not at a track's end: a 1 mm
  track landing on a 0.8 mm join pad and meeting the member on it is the join.

tests/test_1279_sense_branch_geometry_pcbnew.py holds the parse paths to the
same answers under KiCad's python.

Run:
    python3 tests/test_1279_sense_branch_geometry.py
"""
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import traceback

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import sense_branch as sb  # noqa: E402
from kicad_parser import parse_kicad_pcb, Segment  # noqa: E402
from run_utils import evidence, tool  # noqa: E402

#: A handful of route.py runs on a 30 x 20 mm board, ~5 s each here.
RUN_ALL_TIMEOUT = 900

BOARD = os.path.join(TESTS_DIR, 'fixtures', 'sense_branch', 'sense_star.kicad_pcb')
PRO = BOARD[:-len('.kicad_pcb')] + '.kicad_pro'
MEMBER = '11111111-0000-0000-0000-000000000001'
#: The fixture's member track, the anchor every variant inserts before.
ANCHOR = '  (segment (start 3 5)'

_TMP = []


def _tmpdir():
    d = tempfile.mkdtemp(prefix='t1279g_')
    _TMP.append(d)
    return d


def _src():
    with open(BOARD, encoding='utf-8') as f:
        text = f.read()
    assert text.count(ANCHOR) == 1, 'fixture member track moved'
    return text


def _variant(text, name='v'):
    """`text` written as a board in a fresh dir, with the fixture's .kicad_pro."""
    d = _tmpdir()
    path = os.path.join(d, f'{name}.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    shutil.copy(PRO, os.path.join(d, f'{name}.kicad_pro'))
    return path


def _footprint(ref, x, y, pad='(pad "1" smd rect (at 0 0) (size 1 1) '
                              '(layers "F.Cu" "F.Paste" "F.Mask") (net 1 "VDD"))'):
    return (f'  (footprint "T:P" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
            f'(effects (font (size 1 1))))\n    {pad})\n')


def _track(x1, y1, x2, y2, uuid, width=0.3, layer='F.Cu'):
    return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width {width}) '
            f'(layer "{layer}") (net 1) (uuid "{uuid}"))\n')


def _branch(board):
    p = parse_kicad_pcb(board)
    assert len(p.sense_branches) == 1, p.sense_branches
    return p, p.sense_branches[0]


def _run(script, args, timeout=600):
    r = subprocess.run([sys.executable, tool(script)] + args, cwd=ROOT,
                       capture_output=True, text=True, timeout=timeout)
    assert r.returncode == 0, f'{script} exit {r.returncode}:\n' + r.stdout[-3000:] + r.stderr[-3000:]
    return r.stdout


def _route(board, extra=()):
    """route.py VDD to a FRESH output path; (output board, console)."""
    out = os.path.join(_tmpdir(), 'out.kicad_pcb')
    log = _run('route.py', [board, out, '--nets', 'VDD', '--layers', 'F.Cu', 'B.Cu',
                            '--track-width', '0.3', '--clearance', '0.2'] + list(extra))
    evidence(out, 'route.py output')
    return out, log


def _tapped(board):
    """Pads of VDD reached from the member trace without crossing C1.1
    (the test_1279_sense_branches grader, on sense_branch._touch)."""
    p = parse_kicad_pcb(board)
    join = next(q for q in p.footprints['C1'].pads if q.pad_number == '1')
    items = ([('seg', s) for s in p.segments if s.net_id == join.net_id]
             + [('via', v) for v in p.vias if v.net_id == join.net_id]
             + [('pad', q) for q in p.pads_by_net[join.net_id] if q is not join])
    seen = {i for i, (k, it) in enumerate(items) if k == 'seg' and it.uuid == MEMBER}
    assert seen, 'member trace missing'
    todo = list(seen)
    while todo:
        i = todo.pop()
        for j, other in enumerate(items):
            if j not in seen and sb._touch(items[i], other, join):
                seen.add(j)
                todo.append(j)
    return sorted(f'{it.component_ref}.{it.pad_number}' for i, (k, it) in enumerate(items)
                  if i in seen and k == 'pad')


#: C1.1, the fixture's join pad: the FIRST pad block in the file.
JOIN_PAD = '(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") (net 1 "VDD"))'
MEMBER_LINE = ('  (segment (start 3 5) (end 27 5) (width 0.2) (layer "F.Cu") (net 1) '
               f'(uuid "{MEMBER}"))\n')
MEMBER2 = '11111111-0000-0000-0000-000000000002'


def _with_join_pad(text, pad):
    """`text` with C1.1 replaced by the `(pad ...)` s-expression `pad`."""
    i = text.index(JOIN_PAD)
    assert text.index('"C1"') < i < text.index('"U1"'), 'C1.1 is no longer the first pad'
    return text[:i] + pad + text[i + len(JOIN_PAD):]


def _diagonal(text):
    """The member leaving C1 (3, 5) at 45 degrees: (3, 5)-(4.5, 3.5)-(27, 3.5),
    with U1 moved to (27, 3.5) and both tracks in the group."""
    assert text.count(MEMBER_LINE) == 1, 'fixture member track moved'
    return (text.replace(MEMBER_LINE, MEMBER_LINE.replace('(end 27 5)', '(end 4.5 3.5)')
                         + _track(4.5, 3.5, 27, 3.5, MEMBER2, width=0.2))
            .replace('(at 27 5)\n', '(at 27 3.5)\n')
            .replace(f'(members "{MEMBER}")', f'(members "{MEMBER}" "{MEMBER2}")'))


def _disc(x, y, r, n=24):
    """Sample points of a copper disc: its centre and three rings."""
    import numpy as np
    a = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    rr = np.repeat([r / 3, 2 * r / 3, r], n)
    return (np.concatenate([[x], x + rr * np.tile(np.cos(a), 3)]),
            np.concatenate([[y], y + rr * np.tile(np.sin(a), 3)]))


def _in_member(b, xs, ys, layer='F.Cu'):
    """Mask of points lying in the branch's member TRACK copper on `layer`."""
    import numpy as np
    hit = np.zeros(len(xs), dtype=bool)
    for s in b.segments:
        if s.layer != layer:
            continue
        ex, ey = s.end_x - s.start_x, s.end_y - s.start_y
        l2 = ex * ex + ey * ey
        t = np.clip(((xs - s.start_x) * ex + (ys - s.start_y) * ey) / l2, 0.0, 1.0) if l2 else 0.0
        hit |= np.hypot(xs - s.start_x - t * ex, ys - s.start_y - t * ey) <= s.width / 2 - 1e-6
    return hit


def _off_pad(pad, xs, ys):
    """Distance from each point to the pad's real copper (0 on it): the
    checker's own measure (check_drc.point_to_pad_distance), not the code
    under test."""
    import numpy as np
    from check_drc import point_to_pad_distance
    return np.array([point_to_pad_distance(float(x), float(y), pad) for x, y in zip(xs, ys)])


def _free_cells_on_member(board, track_width=0.3, clearance=0.2):
    """Cells near the join pad the branch's stamp leaves free for VDD where a
    `track_width` track would put copper OFF the join pad's real copper
    (more than the 0.01 mm contact slack, sense_branch._EPS) onto member
    copper off it -- a tap the obstacle map allows."""
    from routing_config import GridRouteConfig, GridCoord
    _p, b = _branch(board)
    assert b.protected, b.reason
    cfg = GridRouteConfig(track_width=track_width, clearance=clearance, via_size=0.6,
                          grid_step=0.05, layers=['F.Cu', 'B.Cu'])
    coord = GridCoord(cfg.grid_step)
    cells, _vias = sb.sense_obstacle_rows(b, cfg, coord, {'F.Cu': 0, 'B.Cu': 1}, clearance)
    blocked = {(int(x), int(y)) for x, y, l in cells if l == 0}
    j, bad = b.join, []
    c = coord.to_grid(j.global_x, j.global_y)
    for gx in range(c[0] - 40, c[0] + 41):
        for gy in range(c[1] - 40, c[1] + 41):
            if (gx, gy) in blocked:
                continue
            xs, ys = _disc(gx * cfg.grid_step, gy * cfg.grid_step, track_width / 2)
            if (_in_member(b, xs, ys) & (_off_pad(j, xs, ys) > 0.01)).any():
                bad.append((round(gx * cfg.grid_step, 3), round(gy * cfg.grid_step, 3)))
    return bad


def _off_join_contacts(board, slack=0.01):
    """Points where the routed board's non-member VDD tracks overlap member
    copper OFF the join pad's real copper by more than `slack` (raster
    0.01 mm)."""
    import numpy as np
    p, b = _branch(board)
    mem = set(b.member_uuids)
    out = []
    for s in p.segments:
        if s.net_id != b.net_id or s.uuid in mem or s.layer != 'F.Cu':
            continue
        r = s.width / 2
        gx, gy = np.meshgrid(
            np.arange(min(s.start_x, s.end_x) - r, max(s.start_x, s.end_x) + r + 0.01, 0.01),
            np.arange(min(s.start_y, s.end_y) - r, max(s.start_y, s.end_y) + r + 0.01, 0.01))
        xs, ys = gx.ravel(), gy.ravel()
        ex, ey = s.end_x - s.start_x, s.end_y - s.start_y
        l2 = ex * ex + ey * ey
        t = np.clip(((xs - s.start_x) * ex + (ys - s.start_y) * ey) / l2, 0.0, 1.0) if l2 else 0.0
        on = np.hypot(xs - s.start_x - t * ex, ys - s.start_y - t * ey) <= r
        xs, ys = xs[on], ys[on]
        hit = _in_member(b, xs, ys) & (_off_pad(b.join, xs, ys) > slack)
        out += [(round(float(x), 2), round(float(y), 2)) for x, y in zip(xs[hit], ys[hit])]
    return out


def _off_join_gap_routed(board):
    """The smallest gap (raster 0.01 mm) between the routed board's
    non-member VDD tracks and the member tracks, both measured only where
    they are OFF the join pad's real copper (more than 0.01 mm)."""
    import numpy as np

    def raster(segs):
        pts = []
        for s in segs:
            r = s.width / 2
            gx, gy = np.meshgrid(
                np.arange(min(s.start_x, s.end_x) - r, max(s.start_x, s.end_x) + r + 0.01, 0.01),
                np.arange(min(s.start_y, s.end_y) - r, max(s.start_y, s.end_y) + r + 0.01, 0.01))
            xs, ys = gx.ravel(), gy.ravel()
            ex, ey = s.end_x - s.start_x, s.end_y - s.start_y
            l2 = ex * ex + ey * ey
            t = np.clip(((xs - s.start_x) * ex + (ys - s.start_y) * ey) / l2, 0.0, 1.0) if l2 else 0.0
            on = np.hypot(xs - s.start_x - t * ex, ys - s.start_y - t * ey) <= r
            pts.append(np.column_stack([xs[on], ys[on]]))
        pts = np.concatenate(pts)
        return pts[_off_pad(b.join, pts[:, 0], pts[:, 1]) > 0.01]

    p, b = _branch(board)
    mem = set(b.member_uuids)
    m = raster([s for s in b.segments if s.layer == 'F.Cu'])
    n = raster([s for s in p.segments if s.net_id == b.net_id and s.uuid not in mem
                and s.layer == 'F.Cu'])
    from scipy.spatial import cKDTree
    return float(cKDTree(m).query(n)[0].min()) if len(m) and len(n) else float('inf')


def _seg(x1, y1, x2, y2, width=0.2, layer='F.Cu'):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=width,
                   layer=layer, net_id=1)


# --- contact ----------------------------------------------------------------

def test_x_crossing_is_a_tap():
    """A U2.1-U4.1 load track crossing the sense trace at (15, 5), both its
    ends 3 mm away: KiCad joins it to the trace (measured on pcbnew 10.0.3,
    GetConnectedItems of the member track lists U2.1 and U4.1), so the input
    already taps the branch. The same crossing ON the join pad is the join."""
    p, b = _branch(_variant(_src().replace(
        ANCHOR, _footprint('U4', 15, 2) + _track(15, 8, 15, 2, '44444444-0000-0000-0000-000000000001')
        + ANCHOR), 'xcross'))
    assert not b.protected and 'input copper already taps it (track (15, 8)-(15, 2)' in b.reason, \
        b.reason
    member = next(s for s in p.segments if s.uuid == MEMBER)
    join = b.join
    assert sb._touch(('seg', _seg(15, 0, 15, 10)), ('seg', member), join)
    # Crossing the member where it lies on C1.1 (3, 5): that contact is the join.
    assert not sb._touch(('seg', _seg(3.2, 4, 3.2, 6)), ('seg', member), join)


def test_pad_crossed_off_centre_is_a_sense_pad():
    """VDD pad U5.1 (2 x 2 mm at (10, 5.4)) with the sense trace running
    through it 0.4 mm off its centre, neither trace end on it: KiCad joins it
    to the trace (pcbnew 10.0.3), so it is a sense pad and never a place to
    attach a load. Before, it stayed a routable pad and the route ran C1.1 ->
    U5.1 -> U2.1, loading the trace from C1 to U5. (0.4 mm, not more: past
    0.6 mm check_connected's #479 through-pad credit no longer joins U5.1 to
    the trace although KiCad does, and the run reports the pad open.)"""
    board = _variant(_src().replace(
        ANCHOR, _footprint('U5', 10, 5.4, '(pad "1" smd rect (at 0 0) (size 2 2) '
                                          '(layers "F.Cu" "F.Paste" "F.Mask") (net 1 "VDD"))')
        + ANCHOR), 'thru')
    _p, b = _branch(board)
    assert b.protected, b.reason
    pads = sorted(f'{q.component_ref}.{q.pad_number}' for q in b.sense_pads)
    assert pads == ['U1.1', 'U5.1'], f'sense pads {pads}: U5.1 on the trace is not one'
    out, log = _route(board)
    assert re.search(r'"multipoint_pads_connected": 5, "multipoint_pads_total": 5', log), log[-2000:]
    assert _tapped(out) == ['U1.1', 'U5.1'], f'a load reached the sense copper off C1.1: {_tapped(out)}'


ROUND = '(pad "1" smd circle (at 0 0) (size 2 2) (layers "F.Cu" "F.Paste" "F.Mask") (net 1 "VDD"))'
ROUNDRECT = ('(pad "1" smd roundrect (at 0 0) (size 2 2) (roundrect_rratio 0.25) '
             '(layers "F.Cu" "F.Paste" "F.Mask") (net 1 "VDD"))')


def test_round_join_pad_is_its_copper():
    """A 2 mm ROUND join pad is a circle, not its bounding box.

    - A member starting at (3.95, 4.05) -- inside the box, 0.24 mm off the
      circle's copper -- does not touch the join pad (KiCad: the member joins
      U1.1 only). Before, it read as joined and protected, `routable` hid the
      branch, and the routed net shipped open.
    - With the member leaving the pad at 45 degrees, no cell the stamp leaves
      free lets a 0.3 mm VDD track put copper off the circle onto member copper
      (before: 39 such cells for the circle, 6 for a 0.25 roundrect), and a
      load routed in from the upper right lands on the pad, not on the trace
      beyond it (before: contact at (3.73, 4.13), 0.14 mm off the copper)."""
    gap = _with_join_pad(_src(), ROUND).replace('(segment (start 3 5) (end 27 5)',
                                               '(segment (start 3.95 4.05) (end 27 5)')
    _p, b = _branch(_variant(gap, 'gap'))
    assert not b.protected and 'does not touch the join pad C1.1' in (b.reason or ''), b.reason
    for name, pad in (('round', ROUND), ('roundrect', ROUNDRECT)):
        bad = _free_cells_on_member(_variant(_diagonal(_with_join_pad(_src(), pad)), name))
        assert not bad, f'{name}: {len(bad)} free cells put copper on the member off the pad: {bad[:4]}'
    board = _variant(_diagonal(_with_join_pad(_src(), ROUND)).replace(
        '(footprint "T:P" (layer "F.Cu") (at 15 8)', '(footprint "T:P" (layer "F.Cu") (at 5.5 1)'),
        'round_route')
    out, log = _route(board)
    assert re.search(r'"multipoint_pads_connected": 4, "multipoint_pads_total": 4', log), log[-2000:]
    hits = _off_join_contacts(out)
    assert not hits, f'new copper overlaps the member off the round join pad at {hits[:3]}'


def test_member_across_the_join_pad_touches_it():
    """A member drawn straight across C1.1, (1, 5)-(27, 5) with both ends
    off the pad, is joined to it (before: refused 'member copper does not
    touch the join pad C1.1'); the stub past the pad stays foreign copper."""
    board = _variant(_src().replace('(segment (start 3 5) (end 27 5)',
                                    '(segment (start 1 5) (end 27 5)'), 'across')
    _p, b = _branch(board)
    assert b.protected, b.reason
    assert [f'{q.component_ref}.{q.pad_number}' for q in b.sense_pads] == ['U1.1']
    assert [tuple(round(v, 6) for v in xy) for xy in sb._outside_join(b.segments[0], b.join)] \
        == [(1.0, 5.0, 2.6, 5.0), (3.4, 5.0, 27.0, 5.0)]


FOUR_LAYERS = ('(layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) '
               '(2 "B.Cu" signal)')
BLIND = '11111111-0000-0000-0000-0000000000a1'


def _blind(text, extra=''):
    """The fixture on four copper layers with a blind F.Cu-In1.Cu member via
    at (15, 5) on the sense trace, plus `extra` board items."""
    assert text.count('(layers (0 "F.Cu" signal) (2 "B.Cu" signal)') == 1
    via = (f'  (via blind (at 15 5) (size 0.6) (drill 0.3) (layers "F.Cu" "In1.Cu") '
           f'(net 1) (uuid "{BLIND}"))\n')
    return (text.replace('(layers (0 "F.Cu" signal) (2 "B.Cu" signal)', FOUR_LAYERS)
            .replace(ANCHOR, via + extra + ANCHOR)
            .replace(f'(members "{MEMBER}")', f'(members "{MEMBER}" "{BLIND}")'))


def test_blind_member_via_is_on_its_span_only():
    """A blind F.Cu-In1.Cu member via at (15, 5) has no copper on In2.Cu or
    B.Cu. A B.Cu VDD track passing under it is not a tap (before: refused,
    'input copper already taps it (track (15, 3)-(15, 7) on B.Cu)'; pcbnew
    10.0.3 joins the track to nothing); the stamp leaves In2.Cu and B.Cu free
    (before: 553 cells on each); the terminal gate and the oracle-link
    retarget see nothing there; it does not touch a B.Cu pad above it."""
    from routing_config import GridRouteConfig, GridCoord
    _p, b = _branch(_variant(_blind(_src(), _track(15, 3, 15, 7, '66666666-0000-0000-0000-000000000001',
                                                   layer='B.Cu')), 'blind_tap'))
    assert b.protected, b.reason
    p, b = _branch(_variant(_blind(_src()), 'blind'))
    assert b.protected and len(b.vias) == 1, b.reason
    layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    cfg = GridRouteConfig(track_width=0.3, clearance=0.2, via_size=0.6, grid_step=0.05,
                          layers=layers)
    cells, _vias = sb.sense_obstacle_rows(b, cfg, GridCoord(0.05),
                                          {n: i for i, n in enumerate(layers)}, 0.2)
    per_layer = {n: int((cells[:, 2] == i).sum()) for i, n in enumerate(layers)}
    assert per_layer['In1.Cu'] > 0 and per_layer['In2.Cu'] == per_layer['B.Cu'] == 0, per_layer
    assert sb.terminal_sense_dist(p, 1, 14.0, 5.0, 16.0, 5.0, 'B.Cu', 0.15) == 1e9
    assert sb.join_for_point(p, 1, 15.0, 5.0, 'B.Cu') is None
    via = b.vias[0]
    pad_b = next(q for q in p.footprints['U1'].pads)
    pad_b = type(pad_b)(**{**pad_b.__dict__, 'global_x': 15.0, 'global_y': 5.0, 'layers': ['B.Cu']})
    assert not sb._touch(('via', via), ('pad', pad_b), b.join)
    assert sb._touch(('via', via), ('pad', type(pad_b)(**{**pad_b.__dict__, 'layers': ['F.Cu']})),
                     b.join)


def test_join_pad_narrower_than_the_track_is_landable():
    """C1.1 is 0.5 x 0.5 mm at (3.02, 5.01) -- off the 0.05 mm grid -- and
    VDD routes at 0.5 mm. The cut-out was the pad shrunk by the track's
    half-width, a point no grid cell hits, so the join pad was walled in by
    its own branch: the run reached it only by rescue, descending clearance
    0.2 -> 0.1818 (written into the output .kicad_pro) and clamping vias to
    0.45/0.5 in the pad. Now only the copper that sticks out of the pad is held
    to the rule, and the pad's own cell is always free for tracks and vias."""
    from routing_config import GridRouteConfig, GridCoord
    text = (_src().replace('(at 3 5)\n', '(at 3.02 5.01)\n')
            .replace('(segment (start 3 5)', '(segment (start 3.02 5.01)'))
    board = _variant(_with_join_pad(text, JOIN_PAD.replace('(size 1 1)', '(size 0.5 0.5)')),
                     'small')
    _p, b = _branch(board)
    assert b.protected and (b.join.global_x, b.join.size_x) == (3.02, 0.5), b.reason
    cfg = GridRouteConfig(track_width=0.5, clearance=0.2, via_size=0.6, grid_step=0.05,
                          layers=['F.Cu', 'B.Cu'])
    coord = GridCoord(0.05)
    cells, vias = sb.sense_obstacle_rows(b, cfg, coord, {'F.Cu': 0, 'B.Cu': 1}, 0.2)
    c = coord.to_grid(b.join.global_x, b.join.global_y)
    assert not ((cells[:, 0] == c[0]) & (cells[:, 1] == c[1]) & (cells[:, 2] == 0)).any(), \
        'the join pad\'s own cell is blocked for tracks'
    assert not ((vias[:, 0] == c[0]) & (vias[:, 1] == c[1])).any(), \
        'the join pad\'s own cell is blocked for vias'
    out, log = _route(board, ['--track-width', '0.5'])
    summary = json.loads(re.search(r'^JSON_SUMMARY: (\{.*\})$', log, re.M).group(1))
    dr = summary.get('design_rules') or {}
    assert not dr.get('narrowed') and not dr.get('count'), dr.get('narrowed')
    assert summary['min_clearance_used'] == 0.2, summary['min_clearance_used']
    assert re.search(r'"multipoint_pads_connected": 4, "multipoint_pads_total": 4', log), log[-2000:]
    with open(out[:-len('.kicad_pcb')] + '.kicad_pro', encoding='utf-8') as f:
        doc = json.load(f)
    default = next(k for k in doc['net_settings']['classes'] if k['name'] == 'Default')
    assert (default['clearance'], doc['board']['design_settings']['rules']['min_clearance']) \
        == (0.2, 0.2), (default['clearance'], doc['board']['design_settings']['rules'])
    assert _tapped(out) == ['U1.1'], f'a load reached the sense copper off C1.1: {_tapped(out)}'
    hits = _off_join_contacts(out)
    assert not hits, f'new copper overlaps the member off the join pad at {hits[:3]}'


def test_oblique_member_is_held_from_where_its_copper_leaves():
    """C1.1 is a 2.4 x 0.6 mm pad turned 30 degrees; the member leaves it
    horizontally, so its lower edge crosses the pad edge ~0.2 mm before its
    centreline does. Clipped at the centreline, that copper was outside the
    rule and the load route ran 0.108 mm from it (0.156 before the off-pad
    measure) at a 0.2 mm clearance. Every off-pad point of the member is
    within its half-width of a stamped piece, and the routed gap holds."""
    import numpy as np
    text = _src().replace('(footprint "T:P" (layer "F.Cu") (at 3 5)\n',
                          '(footprint "T:P" (layer "F.Cu") (at 3 5 30)\n')
    board = _variant(_with_join_pad(text, JOIN_PAD.replace('(at 0 0) (size 1 1)',
                                                           '(at 0 0 30) (size 2.4 0.6)')), 'r30')
    _p, b = _branch(board)
    assert b.protected and abs(b.join.rect_rotation) == 30.0, (b.reason, b.join.rect_rotation)
    m = b.segments[0]
    xs, ys = np.meshgrid(np.arange(2.0, 5.0, 0.005), np.arange(4.85, 5.15, 0.005))
    xs, ys = xs.ravel(), ys.ravel()
    on = _in_member(b, xs, ys) & (_off_pad(b.join, xs, ys) > 1e-9)
    pieces = sb._outside_join(m, b.join)
    d = np.full(on.sum(), np.inf)
    for ax, ay, bx, by in pieces:
        ex, ey = bx - ax, by - ay
        t = np.clip(((xs[on] - ax) * ex + (ys[on] - ay) * ey) / (ex * ex + ey * ey), 0.0, 1.0)
        d = np.minimum(d, np.hypot(xs[on] - ax - t * ex, ys[on] - ay - t * ey))
    assert (d <= m.width / 2 + 1e-6).all(), \
        f'{int((d > m.width / 2 + 1e-6).sum())} off-pad member points outside the stamped pieces {pieces}'
    out, log = _route(board)
    assert re.search(r'"multipoint_pads_connected": 4, "multipoint_pads_total": 4', log), log[-2000:]
    assert '"narrowed": []' in log, 'the run narrowed copper'
    gap = _off_join_gap_routed(out)
    assert gap >= 0.2 - 0.015, f'new copper off the pad {gap:.3f} mm from member copper off it'


def _zone(polys, uuid='55555555-0000-0000-0000-000000000001'):
    """A VDD pour on F.Cu: its first ring the outline, the rest cut-outs."""
    rings = ''.join('    (polygon (pts ' + ' '.join(f'(xy {x} {y})' for x, y in r) + '))\n'
                    for r in polys)
    return (f'  (zone (net 1) (net_name "VDD") (layer "F.Cu") (uuid "{uuid}") (hatch edge 0.5)\n'
            '    (connect_pads (clearance 0.2)) (min_thickness 0.2) (filled_areas_thickness no)\n'
            '    (fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3))\n' + rings + '  )\n')


def _pour_keepout(ring, uuid='55555555-0000-0000-0000-000000000002'):
    return (f'  (zone (net 0) (net_name "") (layer "F.Cu") (uuid "{uuid}") (hatch edge 0.5)\n'
            '    (connect_pads (clearance 0)) (min_thickness 0.25)\n'
            '    (keepout (tracks allowed) (vias allowed) (pads allowed) (copperpour not_allowed) '
            '(footprints allowed))\n'
            '    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
            '    (polygon (pts ' + ' '.join(f'(xy {x} {y})' for x, y in ring) + '))\n  )\n')


BOARD_RING = [(0.5, 0.5), (29.5, 0.5), (29.5, 19.5), (0.5, 19.5)]
#: Around the sense trace and U1.1, clear of C1.1's copper (x <= 3.5).
SLOT = [(3.55, 4.2), (29.4, 4.2), (29.4, 5.8), (3.55, 5.8)]


def test_pour_taps_only_where_it_can_fill():
    """A VDD pour over the whole board on F.Cu, the sense trace's layer:
    - cut around the trace by a zone CUT-OUT, or by a copper-pour keep-out,
      KiCad's fill never touches the trace off C1.1 (pcbnew 10.0.3 ZONE_FILLER:
      0 samples) and the branch is protected -- before, the outline alone was
      read and both were refused 'pours over its F.Cu track';
    - a pour over y >= 5.3 feeding U2/U3 overlaps a 0.2 mm strip of the sense
      pad U1.1 but not its centre: pcbnew fills 150/2500 samples of U1.1, so
      it is a tap -- before, U1.1 was judged by its centre and protected.
    The parser keeps the cut-out (Zone.holes); the outline is unchanged."""
    def resolve(extra, name):
        return _branch(_variant(_src().replace(ANCHOR, extra + ANCHOR), name))

    p, b = resolve(_zone([BOARD_RING, SLOT]), 'hole')
    assert [len(z.polygon) for z in p.zones] == [4] and p.zones[0].holes == [SLOT], \
        [(z.polygon, z.holes) for z in p.zones]
    assert b.protected, b.reason
    _p, b = resolve(_zone([BOARD_RING]) + _pour_keepout(SLOT), 'keepout')
    assert b.protected, b.reason
    _p, b = resolve(_zone([[(14, 5.3), (29.5, 5.3), (29.5, 10), (14, 10)]]), 'padedge')
    assert not b.protected and 'pours over sense pad U1.1' in (b.reason or ''), b.reason
    # Not cut around the trace: still a tap (test_1279_sense_branches has the rest).
    _p, b = resolve(_zone([BOARD_RING]), 'whole')
    assert not b.protected and 'pours over its F.Cu track' in (b.reason or ''), b.reason


DANGLING = '99999999-0000-0000-0000-000000000009'


def test_dangling_member_is_ignored_as_kicad_does():
    """The group lists the member track and a uuid no item on the board has.
    KiCad leaves the latter out of the group on load (pcbnew 9.0.6 and 10.0.3:
    GetItems() lists the track only), so the text path does too: the branch
    is protected on its one track, with a warning naming the uuid -- before,
    the text path refused it ('are not tracks or vias on the board') while
    the pcbnew path protected it."""
    p, b = _branch(_variant(_src().replace(
        f'(members "{MEMBER}")', f'(members "{MEMBER}" "{DANGLING}")'), 'dangling'))
    assert b.protected, b.reason
    assert b.member_uuids == [MEMBER] and b.dangling_uuids == [DANGLING], \
        (b.member_uuids, getattr(b, 'dangling_uuids', None))
    entry = b.summary(p)
    assert entry['protected'] and DANGLING in ' '.join(entry.get('warnings', ())), entry
    raw = sb.sense_groups_from_text(_src())
    assert raw == [('sense:C1.1', '33333333-0000-0000-0000-000000000001', [MEMBER])], raw


def test_pad_signed_distance_is_the_pad_copper():
    """sense_branch._pad_sd -- the signed distance every rule test above reads
    -- in closed form for a rect, roundrect, circle or oval pad equals the
    polygon walk over the same shape (check_drc._pad_copper_core) to 1e-12,
    inside and out, tilted or not; and outside it equals
    check_drc.point_to_pad_distance."""
    import numpy as np
    from check_drc import _pad_copper_core, point_to_pad_distance
    from kicad_parser import Pad
    rng = np.random.default_rng(1279)
    for shape, sx, sy, tilt, ratio in (('rect', 2.4, 0.6, 30.0, 0.0), ('rect', 1.0, 1.0, 0.0, 0.0),
                                       ('circle', 2.0, 2.0, 0.0, 0.0),
                                       ('oval', 2.5, 1.0, -20.0, 0.0),
                                       ('roundrect', 2.0, 1.2, 15.0, 0.25)):
        pad = Pad(component_ref='C1', pad_number='1', global_x=3.0, global_y=5.0, local_x=0.0,
                  local_y=0.0, size_x=sx, size_y=sy, shape=shape, layers=['F.Cu'], net_id=1,
                  net_name='VDD', rect_rotation=tilt, roundrect_rratio=ratio)
        xs, ys = 3.0 + rng.uniform(-2, 2, 2000), 5.0 + rng.uniform(-2, 2, 2000)
        closed = sb._pad_sd(pad, xs, ys)
        polys, radius = _pad_copper_core(pad)
        walked = sb._pad_sd(Pad(**{**pad.__dict__, 'polygons': polys}), xs, ys) - radius
        assert np.abs(closed - walked).max() < 1e-12, (shape, float(np.abs(closed - walked).max()))
        out = closed > 1e-9
        ref = np.array([point_to_pad_distance(x, y, pad) for x, y in zip(xs[out], ys[out])])
        assert np.abs(closed[out] - ref).max() < 1e-12, shape


def test_contact_is_judged_where_the_copper_meets():
    """C1.1 is 0.8 x 0.95 mm at (3.02, 5) and VDD routes at 1.0 mm. The route
    lands with a track ending at (3.1, 5.5), 0.025 mm past the pad's edge,
    whose copper meets the member at x = 3.1 -- ON the pad. _touch judged the
    contact at the track's END, so the grader and the run's own
    tapped_by_run called it a tap (before: _tapped U1.1, U2.1, U3.1 and a
    WARNING). It is judged where the copper meets, with the overhang a 1 mm
    track centred on a 0.8 mm pad must make (0.1 mm) counting as on the pad;
    a track meeting the member 0.5 mm past the pad is still a tap."""
    text = (_src().replace('(at 3 5)\n', '(at 3.02 5)\n')
            .replace('(segment (start 3 5)', '(segment (start 3.02 5)'))
    board = _variant(_with_join_pad(text, JOIN_PAD.replace('(size 1 1)', '(size 0.8 0.95)')),
                     'p0603')
    p, b = _branch(board)
    assert b.protected, b.reason
    member = b.segments[0]
    assert not sb._touch(('seg', _seg(3.0, 5.4, 3.1, 5.5, width=1.0)), ('seg', member), b.join)
    assert sb._touch(('seg', _seg(3.9, 5.5, 3.9, 6.5, width=1.0)), ('seg', member), b.join)
    out, log = _route(board, ['--track-width', '1.0'])
    summary = json.loads(re.search(r'^JSON_SUMMARY: (\{.*\})$', log, re.M).group(1))
    entry = summary['sense_branches'][0]
    assert entry['protected'] and 'tapped_by_run' not in entry, entry
    assert not (summary.get('design_rules') or {}).get('narrowed'), summary['design_rules']
    assert _tapped(out) == ['U1.1'], f'a load reached the sense copper off C1.1: {_tapped(out)}'
    hits = _off_join_contacts(out, slack=0.1 + 0.01)
    assert not hits, f'new copper overlaps the member more than 0.1 mm off the pad at {hits[:3]}'


def test_pad_contact_is_the_checkers_measure():
    """sense_branch._seg_meets_pad asks check_connected's question exactly
    (check_drc.segment_pad_gap, which track_reaches_pad asks), not on 0.05 mm
    samples along the track.

    - A 0.05 mm track grazing a 1 x 1 mm pad's corner 0.023 mm off its
      centreline overlaps the pad by 2 um, and KiCad joins them. Its samples,
      0.054 mm apart on a 0.54 mm track, stood 0.027 mm either side of the
      contact and 0.0354 mm off the copper, past the 0.035 mm reach (half
      width plus the 0.01 mm contact slack): before, no contact.
    - At the checker's own reach (no slack: the half width less 1 nm) the
      answer equals track_reaches_pad on 3000 tracks ending or passing near
      rect, tilted, round, oval, roundrect and custom pads, and a contact's
      point lies on the track as near the copper as the track comes."""
    import numpy as np
    from check_connected import track_reaches_pad
    from check_drc import segment_pad_gap
    from kicad_parser import Pad

    def pad_of(shape, sx, sy, tilt=0.0, ratio=0.0, polygons=None):
        return Pad(component_ref='C1', pad_number='1', global_x=3.0, global_y=5.0,
                   local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape=shape,
                   layers=['F.Cu'], net_id=1, net_name='VDD', rect_rotation=tilt,
                   roundrect_rratio=ratio, polygons=polygons)

    square = pad_of('rect', 1.0, 1.0)
    h = 0.5 ** 0.5
    cx, cy = 3.5 + 0.023 * h, 5.5 + 0.023 * h        # off the corner (3.5, 5.5)
    ux, uy = h, -h
    a = 0.54 * 0.55                                    # the contact between two samples
    graze = _seg(cx - ux * a, cy - uy * a, cx + ux * (0.54 - a), cy + uy * (0.54 - a),
                 width=0.05)
    assert track_reaches_pad(graze, square), 'the fixture is no contact for KiCad'
    hit, pt = sb._seg_meets_pad(graze, square)
    assert hit, 'a 2 um corner graze between two samples read as no contact'
    assert math.hypot(pt[0] - cx, pt[1] - cy) < 1e-9, pt

    tri = [(3.0, 4.0), (4.2, 5.6), (1.9, 5.5)]
    pads = [square, pad_of('rect', 2.4, 0.6, tilt=30.0), pad_of('circle', 2.0, 2.0),
            pad_of('oval', 2.5, 1.0, tilt=-20.0),
            pad_of('roundrect', 2.0, 1.2, tilt=15.0, ratio=0.25),
            pad_of('custom', 2.4, 1.6, polygons=[tri])]
    rng = np.random.default_rng(12790)
    agreed = hits = 0
    for pad in pads:
        for _ in range(500):
            x1, y1 = 3.0 + rng.uniform(-2.2, 2.2), 5.0 + rng.uniform(-2.2, 2.2)
            ang, length = rng.uniform(0, 2 * math.pi), rng.uniform(0.05, 3.0)
            seg = _seg(x1, y1, x1 + length * math.cos(ang), y1 + length * math.sin(ang),
                       width=rng.uniform(0.1, 0.4))
            touch, pt = sb._seg_meets_pad(seg, pad, grow=-1e-6)
            assert touch == track_reaches_pad(seg, pad), (pad.shape, seg)
            agreed += 1
            if touch:
                hits += 1
                gap = segment_pad_gap(seg.start_x, seg.start_y, seg.end_x, seg.end_y, pad)
                on = sb._dist_seg(pt[0], pt[1], seg.start_x, seg.start_y, seg.end_x, seg.end_y)
                assert on < 1e-9, (pad.shape, seg, pt)
                assert max(float(sb._pad_sd(pad, pt[0], pt[1])[0]), 0.0) <= gap + 1e-9, \
                    (pad.shape, seg, pt, gap)
    assert hits > 300, f'only {hits} of {agreed} tracks touch their pad: the sample is not near the copper'


TESTS = [
    test_x_crossing_is_a_tap,
    test_pad_crossed_off_centre_is_a_sense_pad,
    test_round_join_pad_is_its_copper,
    test_member_across_the_join_pad_touches_it,
    test_blind_member_via_is_on_its_span_only,
    test_join_pad_narrower_than_the_track_is_landable,
    test_oblique_member_is_held_from_where_its_copper_leaves,
    test_pour_taps_only_where_it_can_fill,
    test_dangling_member_is_ignored_as_kicad_does,
    test_pad_signed_distance_is_the_pad_copper,
    test_contact_is_judged_where_the_copper_meets,
    test_pad_contact_is_the_checkers_measure,
]


def main():
    evidence(BOARD, 'the sense_star fixture')
    evidence(PRO, "the sense_star fixture's .kicad_pro")
    failed = []
    try:
        for t in TESTS:
            try:
                t()
                print(f'  PASS  {t.__name__}')
            except Exception:  # noqa: BLE001
                failed.append(t.__name__)
                print(f'  FAIL  {t.__name__}')
                traceback.print_exc()
    finally:
        for d in _TMP:
            shutil.rmtree(d, ignore_errors=True)
    print(f'{len(TESTS) - len(failed)}/{len(TESTS)} passed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
