#!/usr/bin/env python3
"""The plane fill model a route step reads sees copper laid earlier in the
same run (#1555).

A zone's fill model is cached on the board object and was built the first
time anything asked -- in a `route.py --nets '*'` step, before a single net
routed (the already-routed filter grades every net, plane nets included).
Every later reader got that model: the multipoint grouping of a plane net
(which pads its fill already joins), the pour-launch ladder (which fill
cells a straggler may launch from) and the in-commit prune. Copper the run
laid for other nets in the meantime was not in it, so a plane net routed
late graded pads plane-connected through fill a fresh track had cut, and
was offered launch cells on fill KiCad no longer pours there. The plane
finalize -- which reads a REBUILT model -- then found those pads on
"pinched fill islands" and repaired them by rip-up.

A cached model now carries the foreign copper it was stamped from and is
rebuilt when the board's foreign copper changes (plane_fill_model.
_model_is_current); the ladder's memo keeps the models its cells came from.

The board (built here): a B.Cu GND pour strip with GND test pads TP1 (left
end) and TP4 (right end) in it and TP2 on bare board right of it, and a
foreign SIG net. The run then lays a SIG track across the middle of the
strip between its two pads, which cuts the pour in two: TP4's half no
longer reaches TP1.

Checks:
  A. grouping: TP1 and TP4 share a fill component before the track, and
     not after it
  B. pour-launch ladder, read with the fresh grouping as Phase 3 reads it:
     after the track, no cell launching TP1's component lies in the half
     the track cut off from TP1; and a memo primed before the track (same
     grouping object) answers what a fresh scan answers
  C. the zone's OWN copper does not rebuild the model (cost), a rip of the
     foreign track does (and the pour is whole again)
  D. a model handed KiCad's own fill (install_kicad_fill) is kept as it is

    python3 tests/test_1555_fill_model_sees_inrun_copper.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence                          # noqa: E402

FAILS = []
LAYERS = ['F.Cu', 'B.Cu']
CUT_X = 155.0                 # the SIG track's x: it crosses the strip here
TP1, TP4, TP2 = (151.0, 101.0), (159.0, 101.0), (163.0, 101.0)


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


def board_text():
    def tp_fp(ref, x, y, net='GND'):
        return (f'\t(footprint "test:TP" (layer "B.Cu") (uuid "{_uid()}") (at {x} {y})\n'
                f'\t\t(property "Reference" "{ref}" (at 0 -1 0) (layer "B.SilkS") '
                f'(uuid "{_uid()}") (effects (font (size 0.5 0.5) (thickness 0.1)) '
                '(justify mirror)))\n'
                '\t\t(pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "B.Cu" "B.Mask") '
                f'(net "{net}") (uuid "{_uid()}"))\n\t)\n')
    return ('(kicad_pcb\n\t(version 20260206)\n\t(generator "test_1555")\n'
            '\t(generator_version "10.0")\n'
            '\t(general (thickness 1.6) (legacy_teardrops no))\n\t(paper "A4")\n'
            '\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
            '\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n\t\t(25 "Edge.Cuts" user)\n\t)\n'
            '\t(setup (pad_to_mask_clearance 0))\n'
            '\t(net 0 "")\n\t(net 1 "GND")\n\t(net 2 "SIG")\n'
            + tp_fp('TP1', *TP1) + tp_fp('TP4', *TP4) + tp_fp('TP2', *TP2)
            # SIG's two pads, above and below the strip: the track between
            # them is what the run lays (a padless track would be pruned as a
            # dead end on commit)
            + tp_fp('TP5', CUT_X, 99.0, 'SIG') + tp_fp('TP6', CUT_X, 103.0, 'SIG')
            + f'\t(gr_rect (start 145 95) (end 168 107) (stroke (width 0.05) (type default)) '
            f'(fill no) (layer "Edge.Cuts") (uuid "{_uid()}"))\n'
            f'\t(zone (net "GND") (layer "B.Cu") (uuid "{_uid()}") (hatch edge 0.5)\n'
            '\t\t(connect_pads yes (clearance 0.2)) (min_thickness 0.2)\n'
            '\t\t(fill yes (thermal_gap 0.2) (thermal_bridge_width 0.2) '
            '(island_removal_mode 0))\n'
            '\t\t(polygon (pts (xy 150 100) (xy 160 100) (xy 160 102) (xy 150 102)))\n'
            '\t)\n)\n')


def _cfg():
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=list(LAYERS), track_width=0.2, clearance=0.2)


def _zone(pcb, gnd):
    return next(z for z in pcb.zones if z.net_id == gnd)


def _cut_track(pcb, sig):
    from kicad_parser import Segment
    return Segment(start_x=CUT_X, start_y=99.0, end_x=CUT_X, end_y=103.0,
                   width=0.2, layer='B.Cu', net_id=sig)


def _components(pcb, gnd):
    """{pad position: component} -- the grouping a multipoint route reads."""
    from connectivity import get_multipoint_net_pads, get_terminal_component_info
    pad_info = quiet(get_multipoint_net_pads, pcb, gnd, _cfg())
    comps = get_terminal_component_info(pcb, gnd, pad_info)[0]
    return pad_info, comps, {(round(p[3], 2), round(p[4], 2)): comps.get(i, i)
                             for i, p in enumerate(pad_info)}


def _ladder(pcb, gnd, pad_info, comps, memo=True):
    """{component: [weld point, ...]} the pour-launch ladder offers."""
    from routing_config import GridCoord
    from single_ended_routing import _pour_launch_region_cells
    cfg = _cfg()
    if not memo:
        pcb._pour_launch_memo = None
    cells = quiet(_pour_launch_region_cells, pcb, gnd, pad_info, comps, list(LAYERS),
                  GridCoord(cfg.grid_step), {}, cfg)
    return {c: sorted(cmap.values()) for c, cmap in cells.items()}


def t_grouping_and_ladder(pcb, gnd, sig):
    from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
    from plane_fill_model import get_zone_model
    print('A. grouping sees the track the run laid')
    zone = _zone(pcb, gnd)
    pad_info, comps, by_pos = _components(pcb, gnd)
    check('precondition: TP1 and TP4 share the pour before the track',
          by_pos[TP1] == by_pos[TP4], f'{by_pos}')
    # Prime the ladder's memo with this grouping and the pre-track model.
    before = _ladder(pcb, gnd, pad_info, comps, memo=False)
    check('precondition: before the track, TP1\'s component is offered cells '
          'in the right half of the pour',
          any(fx > CUT_X for fx, _ in before.get(by_pos[TP1], ())),
          f'{sum(map(len, before.values()))} cell(s)')
    m_before = get_zone_model(pcb, zone)

    track = {'new_segments': [_cut_track(pcb, sig)], 'new_vias': []}
    add_route_to_pcb_data(pcb, track)
    check('precondition: the track was committed',
          track['new_segments'] and track['new_segments'][0] in pcb.segments)
    # The memo primed above holds `comps`: asked again with that object, it
    # must answer what a scan of the fill as it is now answers, not hand back
    # cells read off the fill as it was before the track.
    via_memo = _ladder(pcb, gnd, pad_info, comps)
    rescanned = _ladder(pcb, gnd, pad_info, comps, memo=False)

    m_after = get_zone_model(pcb, zone)
    check('the model is rebuilt after foreign copper is committed',
          m_after is not m_before)
    check('the model no longer pours under the track',
          m_after is not None and not m_after.query_component(CUT_X, 101.0),
          f'component {m_after.query_component(CUT_X, 101.0) if m_after else None}')
    _, comps2, by_pos2 = _components(pcb, gnd)
    check('TP1 and TP4 are on different fill islands after the track',
          by_pos2[TP1] != by_pos2[TP4], f'{by_pos2}')

    print('B. pour-launch ladder sees the track the run laid')
    fresh = _ladder(pcb, gnd, pad_info, comps2, memo=False)
    cut_off = [(round(fx, 3), round(fy, 3))
               for fx, fy in fresh.get(by_pos2[TP1], ()) if fx > CUT_X]
    check("no cell launching TP1's component lies in the half the track cut "
          'off', not cut_off, f'{len(cut_off)}: {cut_off[:3]}')
    check('a memo primed before the track answers what a rescan answers',
          via_memo == rescanned,
          f'memo {sum(map(len, via_memo.values()))} cell(s), rescan '
          f'{sum(map(len, rescanned.values()))}')

    print("C. the zone's own copper keeps the model; a rip rebuilds it")
    from kicad_parser import Segment
    own = {'new_segments': [Segment(start_x=151.0, start_y=101.0, end_x=153.0,
                                    end_y=101.0, width=0.2, layer='B.Cu',
                                    net_id=gnd)], 'new_vias': []}
    add_route_to_pcb_data(pcb, own)
    check('own-net copper does not rebuild the model',
          get_zone_model(pcb, zone) is m_after)
    remove_route_from_pcb_data(pcb, track)
    m_rip = get_zone_model(pcb, zone)
    check('a rip of the foreign track rebuilds the model', m_rip is not m_after)
    check('the pour is whole again after the rip',
          m_rip is not None and m_rip.query_component(CUT_X, 101.0) > 0)


def t_kicad_fill_kept(pcb, gnd, sig):
    print("D. a model holding KiCad's own fill is kept")
    from pcb_modification import add_route_to_pcb_data
    from plane_fill_model import get_zone_model, install_kicad_fill
    zone = _zone(pcb, gnd)
    poly = [(150.0, 100.0), (160.0, 100.0), (160.0, 102.0), (150.0, 102.0)]
    install_kicad_fill(pcb, {('GND', 'B.Cu'): [poly]})
    m_kicad = get_zone_model(pcb, zone)
    add_route_to_pcb_data(pcb, {'new_segments': [_cut_track(pcb, sig)],
                                'new_vias': []})
    check('install_kicad_fill model survives a later commit',
          get_zone_model(pcb, zone) is m_kicad)


if __name__ == '__main__':
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory(prefix='t1555_') as d:
        board = os.path.join(d, 'strip.kicad_pcb')
        with open(board, 'w', encoding='utf-8', newline='\n') as f:
            f.write(board_text())
        pcb = quiet(parse_kicad_pcb, evidence(board, 'staged board'))
        gnd = next(n for n, net in pcb.nets.items() if net.name == 'GND')
        sig = next(n for n, net in pcb.nets.items() if net.name == 'SIG')
        t_grouping_and_ladder(pcb, gnd, sig)
        pcb2 = quiet(parse_kicad_pcb, board)
        t_kicad_fill_kept(pcb2, gnd, sig)
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')
