#!/usr/bin/env python3
"""A track may not cut a pour that nothing in the run will repair (#1260).

#1260's board: a 40 x 30 mm two-layer board, SIG's two pads at (5, 15) and
(35, 15), and a ZONE_NET pour 12 mm wide filling the whole height between
them. `route.py --nets SIG --layers F.Cu` reported SUCCESS and drew SIG
straight through the pour. KiCad's refill then cut the pour at the track and
deleted the half that no longer touched a ZONE_NET pad; with a second
ZONE_NET pad on the far side, the refill split the pour into two islands and
left ZONE_NET open. Nothing in the run repairs ZONE_NET -- it is outside
--nets, so the plane finalize excludes it by plan.

The pour guard (py_router/pour_guard.py) blocks such a pour island as an
obstacle when no routed net has a terminal inside it, so SIG fails on F.Cu.
Two controls keep ordinary routing over pours as it was: a pour that does
not span the route, and a pour that holds one of SIG's own pads (a track
leaving a pad inside a pour opens a slot from a hole to the edge, which
divides nothing).

The boards are written here from s-expressions, without pcbnew (the suite
image has no KiCad). Rows:

  * route.py end to end: the issue's board (one ZONE_NET pad) and the
    two-pad variant fail SIG, and the reconciliation lap, which inherits the
    run's barriers, fails it again rather than routing through;
  * the controls route SIG exactly as before (a straight track, and a track
    through the pour that holds J1);
  * KICAD_POUR_GUARD=0 restores the old behaviour on the issue's board;
  * compute_pour_barriers' classification, in process: barrier / crossable
    (a routed terminal inside) / minor (one anchor, below the minimum island
    area) / no anchor, an in-scope zone net, and no net filter at all;
  * the obstacle stamp: the island's layer only, tracks and vias, at the
    zone's own clearance when it is wider than the pair clearance;
  * the pad taps, which place on maps of their own: the #535 off-pad escape
    via of the #189 via-in-pad unblock moves to a site clear of a barrier
    rather than land in it.

    python3 tests/test_1260_pour_guard.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import run_utils                                              # noqa: E402

RUN_ALL_TIMEOUT = 900
ROUTE = run_utils.tool('route.py')
PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}"
          + (f" ({detail})" if detail else ''))
    if not ok:
        failures.append(label)


# --------------------------------------------------------------- the boards
_HEAD = '''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(13 "F.Paste" user)
\t\t(1 "F.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(25 "Edge.Cuts" user)
\t\t(35 "F.Fab" user)
\t)
\t(setup
\t\t(pad_to_mask_clearance 0)
\t)
'''


def _pad_fp(ref, x, y, net):
    return ('\t(footprint ""\n\t\t(layer "F.Cu")\n\t\t(at %g %g)\n'
            '\t\t(property "Reference" "%s"\n\t\t\t(at 0 0 0)\n'
            '\t\t\t(layer "F.SilkS")\n\t\t\t(effects\n\t\t\t\t(font\n'
            '\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
            '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1 1)\n'
            '\t\t\t(layers "F.Cu" "F.Mask" "F.Paste")\n\t\t\t(net "%s")\n'
            '\t\t)\n\t)\n' % (x, y, ref, net))


def _edge(x1, y1, x2, y2):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n'
            '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
            '\t\t(layer "Edge.Cuts")\n\t)\n' % (x1, y1, x2, y2))


def _zone(net, pts):
    xy = ' '.join('(xy %g %g)' % p for p in pts)
    return ('\t(zone\n\t\t(net "%s")\n\t\t(layer "F.Cu")\n\t\t(name "POUR")\n'
            '\t\t(hatch edge 0.5)\n\t\t(connect_pads\n\t\t\t(clearance 0.5)\n'
            '\t\t)\n\t\t(min_thickness 0.25)\n\t\t(fill yes\n'
            '\t\t\t(thermal_gap 0.5)\n\t\t\t(thermal_bridge_width 0.5)\n'
            '\t\t\t(island_removal_mode 0)\n\t\t)\n'
            '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t%s\n\t\t\t)\n\t\t)\n\t)\n'
            % (net, xy))


#: name -> (ZONE_NET pads, pour outline). SIG's pads are always J1 (5, 15)
#: and J2 (35, 15).
BOARDS = {
    # the issue's board: the pour spans the full height between J1 and J2
    'full': ([(20, 15)], [(14, 1), (26, 1), (26, 29), (14, 29)]),
    # ZONE_NET's two pads are joined only by the pour
    'twopad': ([(20, 5), (20, 25)], [(14, 1), (26, 1), (26, 29), (14, 29)]),
    # control: the pour stops short of SIG's straight line
    'partial': ([(20, 6)], [(14, 1), (26, 1), (26, 12), (14, 12)]),
    # control: the pour holds J1, so SIG leaves it from a hole
    'inside': ([(20, 15)], [(2, 1), (26, 1), (26, 29), (2, 29)]),
}


def write_board(path, name):
    zpads, outline = BOARDS[name]
    parts = [_HEAD, _pad_fp('J1', 5, 15, 'SIG'), _pad_fp('J2', 35, 15, 'SIG')]
    for i, (x, y) in enumerate(zpads):
        parts.append(_pad_fp('J%d' % (3 + i), x, y, 'ZONE_NET'))
    corners = [(0, 0), (40, 0), (40, 30), (0, 30)]
    for i in range(4):
        parts.append(_edge(*corners[i], *corners[(i + 1) % 4]))
    parts.append(_zone('ZONE_NET', outline))
    parts.append(')\n')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(''.join(parts))
    return run_utils.evidence(path, f'{name} board')


# ------------------------------------------------------------- end to end
def route(tmp, name, env_extra=None):
    """Route SIG on F.Cu to a FRESH output; return (min summary, SIG tracks
    as (x1, y1, x2, y2), log)."""
    src = write_board(os.path.join(tmp, f'{name}.kicad_pcb'), name)
    tag = 'off' if env_extra else 'on'
    out = os.path.join(tmp, f'{name}_{tag}_out.kicad_pcb')
    env = dict(os.environ)
    env.update(env_extra or {})
    r = run_utils.check(PY + [ROUTE, src, out, '--nets', 'SIG',
                              '--layers', 'F.Cu', '--escalation', 'off'],
                        accept=True, env=env)
    log = r.stdout + r.stderr
    m = re.findall(r'JSON_SUMMARY_MIN: (\{.*\})', log)
    if not m:
        raise AssertionError(f'BROKEN TEST: no JSON_SUMMARY_MIN in the {name} '
                             f'log\n{log[-1500:]}')
    text = open(run_utils.evidence(out, f'{name} output'),
                encoding='utf-8').read()
    tracks = [tuple(float(v) for v in t) for t in re.findall(
        r'\(segment\s+\(start ([-\d.]+) ([-\d.]+)\)\s+'
        r'\(end ([-\d.]+) ([-\d.]+)\)', text)]
    return json.loads(m[-1]), tracks, log


def _crosses(tracks, x0, x1):
    """Some track runs over the band x0 < x < x1."""
    return any(max(a, c) > x0 and min(a, c) < x1 for a, _b, c, _d in tracks)


def test_end_to_end(tmp):
    for name in ('full', 'twopad'):
        mins, tracks, log = route(tmp, name)
        check(f"{name}: SIG fails on F.Cu instead of cutting ZONE_NET's pour",
              mins.get('failed_single') == ['SIG'] and mins.get('routed') == 0,
              f"failed_single={mins.get('failed_single')} "
              f"routed={mins.get('routed')}")
        check(f"{name}: no track crosses the pour", not _crosses(tracks, 14, 26),
              str(tracks))
        check(f"{name}: the guard said what it blocked",
              "Pour guard (#1260): ZONE_NET pour on F.Cu is outside this "
              "run's nets" in log)
        check(f"{name}: the reconciliation lap retried SIG and inherited the "
              f"barrier (it would otherwise route through)",
              'Final reconciliation: retrying 1' in log
              and log.count('Pour guard (#1260): ZONE_NET pour on F.Cu is') == 1)
    mins, tracks, log = route(tmp, 'full', {'KICAD_POUR_GUARD': '0'})
    check("KICAD_POUR_GUARD=0 restores the old route through the pour",
          mins.get('routed') == 1 and _crosses(tracks, 14, 26)
          and 'Pour guard' not in log, str(tracks))

    mins, tracks, log = route(tmp, 'partial')
    check("control: a pour short of SIG's line leaves SIG one straight track",
          mins.get('routed') == 1 and len(tracks) == 1
          and sorted([tracks[0][0], tracks[0][2]]) == [5.0, 35.0]
          and tracks[0][1] == tracks[0][3] == 15.0, str(tracks))
    mins, tracks, log = route(tmp, 'inside')
    check("control: a pour holding J1 stays crossable and SIG routes through it",
          mins.get('routed') == 1 and _crosses(tracks, 14, 26)
          and 'hold a routed net\'s terminal' in log, str(tracks))


# --------------------------------------------------------------- in process
def _parsed(tmp, name):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(write_board(os.path.join(tmp, f'{name}.kicad_pcb'),
                                      name))
    pcb.source_path = ''        # outline geometry: no KiCad subprocess here
    return pcb


def _ids(pcb):
    return {n.name: nid for nid, n in pcb.nets.items()}


def test_classification(tmp):
    import pour_guard
    pcb = _parsed(tmp, 'full')
    ids = _ids(pcb)
    sig = [ids['SIG']]
    layers = ['F.Cu', 'B.Cu']
    bar, rep = pour_guard.compute_pour_barriers(pcb, layers, sig, ['SIG'], 0.1)
    check("an out-of-scope pour with no routed terminal is a barrier",
          len(bar) == 1 and bar[0]['net'] == 'ZONE_NET'
          and bar[0]['layer'] == 'F.Cu' and bar[0]['clearance'] == 0.5
          and rep['pours'][0]['blocked_islands'] == 1, str(rep))
    check("the geometry falls back to the zone outline without a board file",
          rep['source'] == 'zone_outlines', rep['source'])
    bar, rep = pour_guard.compute_pour_barriers(
        pcb, layers, sig, ['SIG', 'ZONE_NET'], 0.1)
    check("a zone net matched by the net filter is in scope: no barrier",
          bar == [] and rep is None)
    bar, rep = pour_guard.compute_pour_barriers(pcb, layers, sig, [], 0.1)
    check("no net filter means every net is in scope: no barrier",
          bar == [] and rep is None)
    bar, rep = pour_guard.compute_pour_barriers(pcb, ['B.Cu'], sig, ['SIG'],
                                                0.1)
    check("a pour on a layer the run does not route is ignored", bar == [])

    pcb = _parsed(tmp, 'inside')
    bar, rep = pour_guard.compute_pour_barriers(
        pcb, layers, [_ids(pcb)['SIG']], ['SIG'], 0.1)
    check("a pour holding a routed pad is crossable",
          bar == [] and rep['pours'][0]['crossable_islands'] == 1, str(rep))

    # Islands chosen directly: a small one on J3 alone, a large empty one,
    # and a small one joining J3 and J4.
    pcb = _parsed(tmp, 'twopad')
    zid = _ids(pcb)['ZONE_NET']
    small_one = [(19, 4), (21, 4), (21, 6), (19, 6)]          # 4 mm^2, J3
    empty = [(30, 20), (38, 20), (38, 28), (30, 28)]           # 64 mm^2, none
    joins = [(19, 4), (21, 4), (21, 26), (19, 26)]             # 44 mm^2, J3+J4
    orig = pour_guard.fill_islands
    try:
        pour_guard.fill_islands = lambda p, w: (
            {k: [small_one, empty, joins] for k in w}, 'test', '')
        bar, rep = pour_guard.compute_pour_barriers(
            pcb, layers, [_ids(pcb)['SIG']], ['SIG'], 0.1)
    finally:
        pour_guard.fill_islands = orig
    row = rep['pours'][0]
    check("only the island joining two of its net's pads is blocked",
          len(bar) == 1 and bar[0]['polygons'] == [joins]
          and row['blocked_islands'] == 1 and row['minor_islands'] == 2,
          str(row))
    check("the minimum island area is KiCad's default unless the zone sets "
          "its own", pour_guard._min_island_area(pcb.zones) == 10.0)
    z = [zz for zz in pcb.zones if zz.net_id == zid][0]
    z.island_removal_mode, z.island_area_min = 2, 3.0
    check("a zone removing islands below its own area uses that area",
          pour_guard._min_island_area([z]) == 3.0)


def test_stamp(tmp):
    from obstacle_map import build_base_obstacle_map
    from routing_config import GridCoord, GridRouteConfig
    pcb = _parsed(tmp, 'full')
    sig = _ids(pcb)['SIG']
    poly = [(14, 1), (26, 1), (26, 29), (14, 29)]
    coord = GridCoord(0.1)

    def build(barriers):
        cfg = GridRouteConfig(track_width=0.2, clearance=0.2, via_size=0.6,
                              via_drill=0.3, grid_step=0.1,
                              layers=['F.Cu', 'B.Cu'])
        cfg.pour_barriers = barriers
        return build_base_obstacle_map(pcb, cfg, [sig])

    obs = build([{'net': 'ZONE_NET', 'layer': 'F.Cu', 'polygons': [poly],
                  'clearance': 0.5}])
    bare = build([])
    mid = coord.to_grid(20.0, 25.0)
    near = coord.to_grid(13.5, 25.0)    # 0.5 off: inside 0.5 + 0.1, not 0.2 + 0.1
    far = coord.to_grid(13.4, 25.0)     # 0.6 off: the keep-out's edge, open
    check("the barrier blocks tracks inside the island on its layer",
          obs.is_blocked(mid[0], mid[1], 0) and not bare.is_blocked(mid[0], mid[1], 0))
    check("...and not on another layer", not obs.is_blocked(mid[0], mid[1], 1))
    check("...and vias inside it", obs.is_via_blocked(mid[0], mid[1])
          and not bare.is_via_blocked(mid[0], mid[1]))
    check("the zone's own clearance (0.5) sets the keep-out, not the pair "
          "clearance (0.2)", obs.is_blocked(near[0], near[1], 0)
          and not obs.is_blocked(far[0], far[1], 0))
    obs2 = build([{'net': 'ZONE_NET', 'layer': 'F.Cu', 'polygons': [poly],
                   'clearance': None}])
    check("with no zone clearance the pair clearance applies",
          not obs2.is_blocked(near[0], near[1], 0))


def test_tap(tmp):
    import contextlib
    import io
    import single_ended_routing as ser
    from check_drc import _point_to_polys_distance, _segment_to_polys_distance
    from routing_config import GridCoord, GridRouteConfig
    # just above J1, where the escape via lands with no barrier
    poly = [(3, 12), (7, 12), (7, 13.7), (3, 13.7)]

    def escape(barriers):
        pcb = _parsed(tmp, 'full')
        sig = _ids(pcb)['SIG']
        pad = pcb.footprints['J1'].pads[0]
        cfg = GridRouteConfig(track_width=0.2, clearance=0.2, via_size=0.6,
                              via_drill=0.3, grid_step=0.05,
                              layers=['F.Cu', 'B.Cu'])
        cfg.same_net_pad_clearance = 0.1    # no in-pad via: the escape runs
        cfg.pour_barriers = barriers
        with contextlib.redirect_stdout(io.StringIO()):
            r = ser._place_shrunk_via_in_pad_impl(pad, None, cfg, pcb, sig,
                                                  GridCoord(0.05), cfg.layers)
        if r is None:
            return None, float('inf')
        via, stub = r[0], r[3]
        gap = min([_point_to_polys_distance(via.x, via.y, [poly]) - via.size / 2]
                  + [_segment_to_polys_distance(s.start_x, s.start_y, s.end_x,
                                                s.end_y, [poly])[0] - s.width / 2
                     for s in stub])
        return via, gap

    via, gap = escape([])
    check("control: with no barrier the escape via lands next to the island",
          via is not None and gap < 0.2, f"{gap:.3f} mm")
    via, gap = escape([{'net': 'ZONE_NET', 'layer': 'F.Cu', 'polygons': [poly],
                        'clearance': None}])
    check("with the barrier the escape via and its stub keep the clearance "
          "from the island", via is not None and gap >= 0.2 - 1e-6,
          f"via {via and (via.x, via.y)}, {gap:.3f} mm")


def main():
    tmp = tempfile.mkdtemp(prefix='t1260_')
    try:
        print("route.py end to end")
        test_end_to_end(tmp)
        try:
            import pour_guard  # noqa: F401
        except ImportError as e:
            check("py_router/pour_guard.py exists", False, str(e))
        else:
            print("in process: classification")
            test_classification(tmp)
            print("in process: obstacle stamp")
            test_stamp(tmp)
            print("in process: pad taps")
            test_tap(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAILED {len(failures)}: " + '; '.join(failures))
        return 1
    print("\nall #1260 pour-guard checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
