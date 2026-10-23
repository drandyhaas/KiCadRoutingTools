#!/usr/bin/env python3
"""Every path that lays new copper keeps off a protected sense branch (#1279).

The fixture tests/fixtures/sense_branch/sense_star.kicad_pcb (+ .kicad_pro):
net VDD, join pad C1.1 (3, 5), sense pad U1.1 (27, 5), one 0.2 mm F.Cu sense
trace between them declared by `(group "sense:C1.1" ...)`. The boards below
are built from its text in a temp dir, so each case is a few lines of copper
added to that board. tests/test_1279_sense_branches.py covers the main route;
these cover the OTHER paths that lay copper of the branch's own net:

(a) plane_region_connector.build_base_obstacles -- the map region joins,
    oracle links and the guaranteed join route on -- leaves the routed net's
    copper out, so the branch is stamped back in as foreign copper with the
    join pad cut out.
(b) kicad_oracle: a link end on the branch off its join pad moves to the
    join pad (sense_branch.join_link_end, shared with route.py's #572
    links), an end's cluster expansion never offers member copper, and the
    width upgrades (wide_route_clear) and the direct sliver weld -- foreign
    copper only, by geometry -- measure the branch too. End to end: a load
    whose only way out is onto the sense pad, left to the route step's
    in-run plane finalize oracle.
(c) the bridging passes (close_soft_joints, snap_stub_gaps) join same-net
    copper by geometry and never use the branch as a bridge end: a dead end
    within a stub gap of the trace (in memory), and a via legally beside a
    sense pad (route.py, end to end). The region-join board (repair_planes,
    end to end) holds (a), (b) and (c) together: the join crosses at C1.1.
(d) route_planes names every protected branch a pour it creates covers
    (KiCad's fill joins same-net copper: the next route step refuses the
    branch), as a WARNING from the engine -- the Planes tab's log -- and as
    `poured_over` in JSON_SUMMARY.sense_branches; repair_planes reports the
    branches, judged on the board it wrote.

Run:
    python3 tests/test_1279_sense_branch_paths.py
"""
import json
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
from kicad_parser import parse_kicad_pcb  # noqa: E402
from run_utils import tool, evidence  # noqa: E402

BOARD = os.path.join(TESTS_DIR, 'fixtures', 'sense_branch', 'sense_star.kicad_pcb')
PRO = BOARD[:-len('.kicad_pcb')] + '.kicad_pro'
MEMBER = '11111111-0000-0000-0000-000000000001'
#: The fixture's member trace line; extra copper is inserted before it.
ANCHOR = '  (segment (start 3 5) (end 27 5)'
#: The clearance every case routes at (the fixture's Default class).
CLEARANCE = 0.2

_TMP = []


def _tmpdir():
    d = tempfile.mkdtemp(prefix='t1279p_')
    _TMP.append(d)
    return d


def _src():
    with open(BOARD, encoding='utf-8') as f:
        return f.read()


def _variant(text, name='v'):
    """`text` written as a board in a fresh dir, with the fixture's .kicad_pro."""
    d = _tmpdir()
    path = os.path.join(d, f'{name}.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    shutil.copy(PRO, os.path.join(d, f'{name}.kicad_pro'))
    return path


def _seg(x1, y1, x2, y2, w, layer, net, uuid):
    """A KiCad-LOCKED track: the router may not rip it (#521), so it walls."""
    return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width {w}) '
            f'(layer "{layer}") (net {net}) (locked yes) (uuid "{uuid}"))\n')


def _with_gnd(text, copper):
    """`text` with a GND net and `copper` (locked GND walls) added."""
    text = text.replace('(net 0 "") (net 1 "VDD")', '(net 0 "") (net 1 "VDD") (net 2 "GND")')
    assert ANCHOR in text
    return text.replace(ANCHOR, copper + ANCHOR)


def _run(script, args, timeout=600):
    r = subprocess.run([sys.executable, tool(script)] + args, cwd=ROOT,
                       capture_output=True, text=True, timeout=timeout)
    assert r.returncode == 0, f'{script} exit {r.returncode}:\n' + r.stdout[-3000:] + r.stderr[-3000:]
    return r.stdout


def _branch(board_or_pcb):
    p = parse_kicad_pcb(board_or_pcb) if isinstance(board_or_pcb, str) else board_or_pcb
    assert len(p.sense_branches) == 1, p.sense_branches
    return p.sense_branches[0]


def _vdd(p):
    return next(i for i, n in p.nets.items() if n.name == 'VDD')


def _summary(log):
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', log, re.M)
    assert m, 'no JSON_SUMMARY printed'
    return json.loads(m.group(1))


def _connected(board):
    r = subprocess.run([sys.executable, tool('check_connected.py'), board, '--nets', 'VDD'],
                       cwd=ROOT, capture_output=True, text=True)
    assert 'ALL NETS FULLY CONNECTED' in r.stdout, r.stdout[-2000:] + r.stderr[-1000:]


def _hits(src, out):
    """New VDD copper of `out` within CLEARANCE of `src`'s protected branch
    off the join pad (sense_branch.terminal_sense_dist against the INPUT's
    branch: a tapped output re-parses as refused and measures nothing).
    Also asserts the member trace survived."""
    pin = parse_kicad_pcb(src)
    b = _branch(pin)
    assert b.protected, b.reason
    pout = parse_kicad_pcb(out)
    assert any(s.uuid == MEMBER for s in pout.segments), 'the member trace is gone'
    net, vdd_out = b.net_id, _vdd(pout)
    old = {s.uuid for s in pin.segments} | {v.uuid for v in pin.vias}
    hits = []
    for s in pout.segments:
        if s.net_id != vdd_out or s.uuid in old:
            continue
        half = s.width / 2.0
        gap = sb.terminal_sense_dist(pin, net, s.start_x, s.start_y, s.end_x, s.end_y,
                                     s.layer, half) - half
        if gap < CLEARANCE - 0.01:
            hits.append(f'track ({s.start_x:g}, {s.start_y:g})-({s.end_x:g}, {s.end_y:g}) '
                        f'{s.layer} w={s.width:g} gap {gap:.3f}')
    for v in pout.vias:
        if v.net_id != vdd_out or v.uuid in old:
            continue
        half = v.size / 2.0
        gap = min(sb.terminal_sense_dist(pin, net, v.x, v.y, v.x, v.y, layer, half)
                  for layer in pin.board_info.copper_layers) - half
        if gap < CLEARANCE - 0.01:
            hits.append(f'via ({v.x:g}, {v.y:g}) gap {gap:.3f}')
    return hits


# --- (a) the plane connector's base map --------------------------------------

def test_plane_base_map_stamps_the_branch():
    """The map region joins and oracle links route VDD on: the sense trace
    and U1.1 are blocked for tracks and vias off C1.1, C1.1 itself is not."""
    from plane_region_connector import build_base_obstacles
    from routing_config import GridRouteConfig, GridCoord
    p = parse_kicad_pcb(BOARD)
    assert _branch(p).protected
    vdd = _vdd(p)
    cfg = GridRouteConfig(clearance=CLEARANCE, track_width=0.3, via_size=0.6,
                          via_drill=0.3, grid_step=0.1, layers=['F.Cu', 'B.Cu'])
    obs, layer_map = build_base_obstacles({vdd}, ['F.Cu', 'B.Cu'], p, cfg,
                                          track_width=0.3, track_via_clearance=0.8,
                                          hole_to_hole_clearance=0.3)
    coord = GridCoord(cfg.grid_step)
    f = layer_map['F.Cu']
    for x, y, what in ((15.0, 5.0, 'on the trace'), (15.0, 5.4, 'beside the trace'),
                       (27.0, 5.0, 'on U1.1')):
        gx, gy = coord.to_grid(x, y)
        assert obs.is_blocked(gx, gy, f), f'F.Cu {what} ({x}, {y}) is free to VDD'
        assert obs.is_via_blocked(gx, gy), f'a VDD via {what} ({x}, {y}) is allowed'
    gx, gy = coord.to_grid(3.0, 5.0)
    assert not obs.is_blocked(gx, gy, f), 'the join pad C1.1 is blocked to its own net'
    # Another net's map is unchanged: the branch is ordinary VDD copper to it.
    obs2, _ = build_base_obstacles(set(), ['F.Cu', 'B.Cu'], p, cfg, track_width=0.3,
                                   track_via_clearance=0.8, hole_to_hole_clearance=0.3)
    gx, gy = coord.to_grid(3.0, 5.0)
    assert obs2.is_blocked(gx, gy, f), 'foreign VDD pad C1.1 free to another net'
    # A sense pad never anchors a region (nor carries one across layers).
    from plane_region_connector import _collect_anchor_points
    pts, _ = _collect_anchor_points(vdd, (0, 0, 30, 20), p, coord, {'F.Cu'},
                                    ['F.Cu', 'B.Cu'])
    assert (3.0, 5.0) in pts and (27.0, 5.0) not in pts, pts


# --- (b) the width upgrades and the sliver weld --------------------------------

def test_widening_checks_measure_the_branch():
    """wide_route_clear (every region-join / oracle width upgrade and the
    oracle's emitted-copper gate) and the oracle's direct sliver weld check
    foreign copper by geometry; the branch is foreign to its own net there."""
    from plane_region_connector import wide_route_clear
    from kicad_oracle import _direct_sliver_weld
    from routing_config import GridRouteConfig
    p = parse_kicad_pcb(BOARD)
    vdd = _vdd(p)
    cfg = GridRouteConfig(clearance=CLEARANCE, track_width=0.3, via_size=0.6,
                          via_drill=0.3, grid_step=0.1, layers=['F.Cu', 'B.Cu'])
    leg = lambda *xyxy: [(xyxy[0], xyxy[1], 'F.Cu'), (xyxy[2], xyxy[3], 'F.Cu')]  # noqa: E731
    assert not wide_route_clear(leg(15, 3, 15, 7), 0.3, p, vdd, cfg), 'a leg across the trace'
    assert not wide_route_clear(leg(15, 3, 15, 4.6), 0.3, p, vdd, cfg), 'a leg 0.05 mm off it'
    assert wide_route_clear(leg(15, 3, 15, 4.5), 0.3, p, vdd, cfg), 'a leg at clearance refused'
    assert wide_route_clear(leg(3, 2, 3, 5), 0.3, p, vdd, cfg), 'a leg onto the join pad refused'
    assert not wide_route_clear(leg(3, 2, 3, 5), 1.2, p, vdd, cfg), \
        'a 1.2 mm leg onto C1.1 reaches past the pad onto the trace'
    assert _direct_sliver_weld(p, vdd, 15, 4.7, 15, 5.3, 'F.Cu', cfg) is None, \
        'a sliver weld across the trace'
    assert _direct_sliver_weld(p, vdd, 15, 6.7, 15, 7.3, 'F.Cu', cfg) is not None, \
        'a sliver weld clear of the branch refused'


def test_oracle_link_ends_never_on_the_branch():
    """An oracle link end on the trace or on U1.1 lands on C1.1, and an end
    expanded to its copper cluster never offers member copper."""
    from kicad_oracle import _net_track_components, _cluster_points
    p = parse_kicad_pcb(BOARD)
    vdd = _vdd(p)
    for end in ((15.0, 5.0, 'F.Cu', 'track'), (27.0, 5.0, 'F.Cu', 'pad'),
                (27.0, 5.0, None, 'pad')):
        assert sb.join_link_end(p, vdd, end) == (3.0, 5.0, 'F.Cu', end[3]), end
    off = (15.0, 8.0, 'F.Cu', 'pad')
    assert sb.join_link_end(p, vdd, off) == off
    comps = _net_track_components(p, vdd)
    pts, _root = _cluster_points(p, vdd, 3.0, 5.0, 'F.Cu', comps)
    assert all(sb.join_for_point(p, vdd, q[0], q[1], q[2] if len(q) > 2 else None) is None
               for q in pts), f'the cluster of C1.1 offers sense copper: {pts}'


def _board_oracle():
    """Load U2.1 moves to (27, 6.7), inside a locked GND fence (F.Cu, plus a
    B.Cu slab under it) that also holds the sense pad U1.1: the only way out
    of the fence for U2.1 is onto U1.1. The routers refuse that, so U2.1 is
    left for the plane finalize's oracle, whose links used to land there."""
    t = _src().replace('(at 15 8)\n    (property "Reference" "U2"',
                       '(at 27 6.7)\n    (property "Reference" "U2"')
    fence = ''.join(_seg(*xyw, 'F.Cu' if i < 5 else 'B.Cu', 2,
                         f'aaaaaaaa-0000-0000-0000-{i:012d}')
                    for i, xyw in enumerate([(25.9, 5.4, 25.9, 7.8, 0.2), (25.9, 7.8, 28.1, 7.8, 0.2),
                                             (28.1, 7.8, 28.1, 4.0, 0.2), (28.1, 4.0, 25.9, 4.0, 0.2),
                                             (25.9, 4.0, 25.9, 4.6, 0.2), (27.0, 4.9, 27.0, 6.9, 3.0)]))
    return _variant(_with_gnd(t, fence), 'oracle')


class _Skip(Exception):
    """A test whose precondition this machine lacks: reported, not failed."""


def test_plane_finalize_oracle_leaves_the_branch_alone():
    """route_planes (B.Cu VDD pour) then route.py: the in-run plane
    finalize's oracle leg (#562) routes the links KiCad's fill reports
    missing; none may land on U1.1 or the trace. U2.1 stays open. The
    oracle is kicad-cli, so without it this case cannot run."""
    from kicad_oracle import find_kicad_cli
    if find_kicad_cli(warn=False) is None:
        raise _Skip('kicad-cli not found: the oracle leg cannot run')
    src = _board_oracle()
    d = _tmpdir()
    pour, out = os.path.join(d, 'pour.kicad_pcb'), os.path.join(d, 'out.kicad_pcb')
    _run('route_planes.py', [src, pour, '--nets', 'VDD', '--plane-layers', 'B.Cu',
                             '--clearance', str(CLEARANCE), '--track-width', '0.3'])
    evidence(pour, 'route_planes.py output')
    log = _run('route.py', [pour, out, '--nets', 'VDD', '--layers', 'F.Cu', 'B.Cu',
                            '--track-width', '0.3', '--clearance-ceiling', str(CLEARANCE)])
    evidence(out, 'route.py output')
    assert 'KiCad-oracle recheck round 1' in log, \
        'the oracle had no link to route; the case proves nothing\n' + log[-3000:]
    hits = _hits(src, out)
    assert not hits, f'new VDD copper on the sense branch off C1.1: {hits}'
    assert _branch(out).protected, 'the routed board taps its own branch'


# --- (c) the geometry bridging passes -----------------------------------------

def test_stub_gap_snap_never_targets_the_branch():
    """snap_stub_gaps extends a dead end to the nearest same-net copper
    within 1.5x its width. Measured in the plane cleanup (repair_planes on
    the region-join board below, before the width upgrade measured the
    branch): a 0.8 mm strap whose via end sat 0.9 mm from the trace's
    centreline was snapped onto the trace. The same geometry, in memory."""
    from kicad_parser import Segment, Via
    from pcb_modification import snap_stub_gaps
    from routing_config import GridRouteConfig
    p = parse_kicad_pcb(BOARD)
    vdd = _vdd(p)
    strap = Segment(start_x=7.9, start_y=4.1, end_x=3.2, end_y=4.1, width=0.8,
                    layer='F.Cu', net_id=vdd)
    p.segments.append(strap)
    p.vias.append(Via(x=7.9, y=4.1, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'],
                      net_id=vdd))
    cfg = GridRouteConfig(clearance=CLEARANCE, track_width=0.3, via_size=0.6,
                          via_drill=0.3, grid_step=0.1, layers=['F.Cu', 'B.Cu'])
    results = []
    snap_stub_gaps(results, p, {vdd}, cfg)
    new = [s for r in results for s in r.get('new_segments', [])]
    taps = [s for s in new
            if sb.terminal_sense_dist(p, vdd, s.start_x, s.start_y, s.end_x, s.end_y,
                                      s.layer, s.width / 2.0) < s.width / 2.0]
    assert not taps, f'a stub-gap connector onto the sense branch: '         f'{[(s.start_x, s.start_y, s.end_x, s.end_y) for s in taps]}'


def _board_soft_joint():
    """Load U2 is a B.Cu pad at (27, 5.6), just below the sense pad U1.1,
    inside a locked GND B.Cu ring: the VDD via that serves it lands 0.2 mm
    off U1.1's edge -- legal, and inside close_soft_joints' via->pad graze
    band (barrel edge within a track width of a same-net SMD pad)."""
    t = _src().replace(
        '(footprint "T:P" (layer "F.Cu") (at 15 8)\n'
        '    (property "Reference" "U2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1))))\n'
        '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") (net 1 "VDD")))',
        '(footprint "T:P" (layer "B.Cu") (at 27 5.6)\n'
        '    (property "Reference" "U2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1))))\n'
        '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "B.Cu" "B.Paste" "B.Mask") (net 1 "VDD")))')
    t = t.replace('(layers (0 "F.Cu" signal) (2 "B.Cu" signal)',
                  '(layers (0 "F.Cu" signal) (2 "B.Cu" signal) (14 "B.Paste" user) (3 "B.Mask" user)')
    ring = ''.join(_seg(*xy, 0.2, 'B.Cu', 2, f'cccccccc-0000-0000-0000-{i:012d}')
                   for i, xy in enumerate([(25.8, 6.7, 28.2, 6.7), (25.8, 4.8, 25.8, 6.7),
                                           (28.2, 4.8, 28.2, 6.7), (25.8, 4.8, 28.2, 4.8)]))
    return _variant(_with_gnd(t, ring), 'softjoint')


def test_soft_joint_never_bridges_onto_a_sense_pad():
    """route.py: the via beside U1.1 is legal; the cleanup's via->pad graze
    bridge used to stub it onto U1.1 (and the late collapse then dropped the
    now-redundant sense trace)."""
    src = _board_soft_joint()
    out = os.path.join(_tmpdir(), 'out.kicad_pcb')
    _run('route.py', [src, out, '--nets', 'VDD', '--layers', 'F.Cu', 'B.Cu',
                      '--track-width', '0.3', '--clearance', str(CLEARANCE)])
    evidence(out, 'route.py output')
    hits = _hits(src, out)
    assert not hits, f'new VDD copper on the sense branch off C1.1: {hits}'


def _board_region_join():
    """A locked GND B.Cu wall right under the sense trace splits a B.Cu VDD
    pour into a top and a bottom island; locked GND F.Cu stubs close both
    ends, so a region join must cross y=5 on F.Cu -- legally only over C1.1.
    U3 moves to the top island so both islands carry a load."""
    t = _src().replace('(at 20 8)\n    (property "Reference" "U3"',
                       '(at 20 2)\n    (property "Reference" "U3"')
    wall = (_seg(0.5, 5, 29.5, 5, 0.2, 'B.Cu', 2, 'dddddddd-0000-0000-0000-000000000001')
            + _seg(0.6, 5, 2.0, 5, 0.2, 'F.Cu', 2, 'dddddddd-0000-0000-0000-000000000002')
            + _seg(28.0, 5, 29.4, 5, 0.2, 'F.Cu', 2, 'dddddddd-0000-0000-0000-000000000003'))
    return _variant(_with_gnd(t, wall), 'regionjoin')


def test_region_join_crosses_only_at_the_join_pad():
    """route_planes then repair_planes: the region join (its map stamps the
    branch, (a)) and the plane cleanup's stub-gap snap after it."""
    src = _board_region_join()
    d = _tmpdir()
    pour, rep = os.path.join(d, 'pour.kicad_pcb'), os.path.join(d, 'repair.kicad_pcb')
    _run('route_planes.py', [src, pour, '--nets', 'VDD', '--plane-layers', 'B.Cu',
                             '--clearance', str(CLEARANCE), '--track-width', '0.3'])
    evidence(pour, 'route_planes.py output')
    log = _run('repair_planes.py', [pour, rep, '--nets', 'VDD', '--plane-layers', 'B.Cu',
                                    '--clearance', str(CLEARANCE), '--track-width', '0.3'])
    evidence(rep, 'repair_planes.py output')
    assert 'Joining 2 regions' in log, 'no region join ran; the case proves nothing\n' + log[-2000:]
    hits = _hits(src, rep)
    assert not hits, f'new VDD copper on the sense branch off C1.1: {hits}'
    assert _branch(rep).protected, 'the repaired board taps its own branch'
    _connected(rep)
    # (d) repair_planes reports the branch, judged on the board it wrote.
    entry = _summary(log)['sense_branches']
    assert [(e['group'], e['protected'], 'tapped_by_run' in e) for e in entry] == \
        [('sense:C1.1', True, False)], entry


# --- (d) a pour over the branch is warned -------------------------------------

def _board_th_sense_pad():
    """The fixture with its sense pad U1.1 a THROUGH-HOLE pad: a pour of VDD
    on ANY layer floods it."""
    old = ('    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") '
           '(net 1 "VDD")))\n  (footprint "T:P" (layer "F.Cu") (at 15 8)')
    new = ('    (pad "1" thru_hole circle (at 0 0) (size 1.2 1.2) (drill 0.6) '
           '(layers "*.Cu" "*.Mask") (net 1 "VDD")))\n  (footprint "T:P" (layer "F.Cu") (at 15 8)')
    t = _src()
    assert t.count(old) == 1
    return _variant(t.replace(old, new), 'thpad')


def _pour(board, layer):
    out = os.path.join(_tmpdir(), 'pour.kicad_pcb')
    log = _run('route_planes.py', [board, out, '--nets', 'VDD', '--plane-layers', layer,
                                   '--clearance', str(CLEARANCE), '--track-width', '0.3'])
    evidence(out, 'route_planes.py output')
    return log


def test_pour_over_the_branch_is_warned():
    """route_planes: a VDD pour on the trace's layer, and one on B.Cu over a
    through-hole sense pad, each name the branch (WARNING + `poured_over`);
    the fixture's own B.Cu pour, which touches no sense copper, does not."""
    for board, layer in ((BOARD, 'F.Cu'), (_board_th_sense_pad(), 'B.Cu')):
        log = _pour(board, layer)
        assert (f'WARNING: the VDD pour on {layer} covers sense branch sense:C1.1'
                in log), log[-3000:]
        entry = _summary(log)['sense_branches']
        assert [(e['group'], e.get('poured_over')) for e in entry] == \
            [('sense:C1.1', [layer])], entry
    log = _pour(BOARD, 'B.Cu')
    assert 'covers sense branch' not in log, log[-3000:]
    entry = _summary(log)['sense_branches']
    assert [(e['group'], e['protected'], e.get('poured_over')) for e in entry] == \
        [('sense:C1.1', True, None)], entry


def test_pour_warning_on_the_gui_call_shape():
    """The Planes tab's call (an in-memory PCBData, return_results=True,
    dry_run): the warning is the engine's, so the tab's log shows it."""
    import contextlib
    import io
    import route_planes as rp
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rp.create_plane(BOARD, '', ['VDD'], ['F.Cu'], clearance=CLEARANCE,
                        track_width=0.3, dry_run=True, return_results=True,
                        pcb_data=parse_kicad_pcb(BOARD))
    assert 'WARNING: the VDD pour on F.Cu covers sense branch sense:C1.1' in buf.getvalue(), \
        buf.getvalue()[-3000:]
    assert [e.get('poured_over') for e in rp.consume_sense_branches()] == [['F.Cu']]


TESTS = [
    test_plane_base_map_stamps_the_branch,
    test_widening_checks_measure_the_branch,
    test_oracle_link_ends_never_on_the_branch,
    test_plane_finalize_oracle_leaves_the_branch_alone,
    test_stub_gap_snap_never_targets_the_branch,
    test_soft_joint_never_bridges_onto_a_sense_pad,
    test_region_join_crosses_only_at_the_join_pad,
    test_pour_over_the_branch_is_warned,
    test_pour_warning_on_the_gui_call_shape,
]


def main():
    evidence(BOARD, 'the sense_star fixture')
    evidence(PRO, "the sense_star fixture's .kicad_pro")
    failed, skipped = [], []
    try:
        for t in TESTS:
            try:
                t()
                print(f'  PASS  {t.__name__}')
            except _Skip as e:
                skipped.append(t.__name__)
                print(f'  SKIP  {t.__name__}: {e}')
            except Exception:  # noqa: BLE001
                failed.append(t.__name__)
                print(f'  FAIL  {t.__name__}')
                traceback.print_exc()
    finally:
        for d in _TMP:
            shutil.rmtree(d, ignore_errors=True)
    print(f'{len(TESTS) - len(failed) - len(skipped)}/{len(TESTS)} passed'
          + (f', {len(skipped)} skipped (asserted nothing)' if skipped else ''))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
