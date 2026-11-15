"""#1930: copper a net may lift is stamped through the area composition.

A board with a clearance rule scoped by a rule area is stamped twice (#1843):
at the class values outside the areas and at each area's value inside it, and
the lift records are cut the same way. The rows a net may LIFT -- a net tie's
bridge, a part's own tab -- are stamped on the real map, past the static
proxy, so the lift can remove them. They went past the area composition too:
stamped whole at the outside value, lifted at the area's. In an area that
relaxes the clearance a ring of the wider stamp stayed around the tie and its
pads, and the tied net could not leave; in an area that tightens it the lift
took off rows the map never held for that copper, which opens cells another
net's copper blocks.

The board is #908's net tie (tests/test_908_net_tie_own_copper.py) in a rule
area `ROOM`, and the invariant is #908's: **a tied net sees exactly the map it
would see if the tie's copper were not copper at all**, here with the area
composition live. The control is the same board with the bridge on
silkscreen.

  1. the fixture has modelled tie copper and the area rule is installed;
  2. relaxing area (class 0.5, ROOM 0.15): the tied net's map equals the
     control's;
  3. tightening area (class 0.2, ROOM 0.4), a foreign track beside the tie:
     the tied net's map equals the control's;
  4. the bridge still blocks a foreign net inside the area;
  5. route.py routes the tied net on one layer in the relaxing area with no
     escalation, and check_drc and check_connected are clean.
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
import test_908_net_tie_own_copper as t908  # noqa: E402

PY = [sys.executable]
FAILS = []

ZONE = '''  (zone (layer "F.Cu") (uuid "bbbbbbbb-0000-4000-8000-000000000001") (name "ROOM") (hatch edge 0.5)
    (connect_pads (clearance 0)) (min_thickness 0.25)
    (keepout (tracks allowed) (vias allowed) (pads allowed) (copperpour allowed) (footprints allowed))
    (placement (enabled no) (sheetname ""))
    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 7 7) (xy 13 7) (xy 13 13) (xy 7 13))))
'''
#: A FOREIGN track 0.6 mm below the tie's centre line, inside ROOM.
FOREIGN_TRACK = ('  (segment (start 9 10.7) (end 11 10.7) (width 0.2) (layer "F.Cu") '
                 '(net 3) (uuid "cccccccc-0000-4000-8000-000000000001"))\n')


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def stage(td, tag, tie_layer, cls, room, foreign_track=False):
    """#908's board with the bridge on `tie_layer`, in ROOM, with the Default
    class at `cls` mm and ROOM's rule at `room` mm."""
    text = t908.BOARD % t908._TIE_POLY.format(layer=tie_layer)
    k = text.rindex('(gr_rect')
    text = text[:k] + ZONE + (FOREIGN_TRACK if foreign_track else '') + '  ' + text[k:]
    base = os.path.join(td, tag)
    with open(base + '.kicad_pcb', 'w', encoding='utf-8') as fh:
        fh.write(text)
    with open(base + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {'min_clearance': 0.0}}},
                   'net_settings': {'meta': {'version': 4}, 'classes': [
                       {'name': 'Default', 'clearance': cls, 'track_width': 0.2,
                        'via_diameter': 0.6, 'via_drill': 0.3,
                        'priority': 2147483647}]},
                   'meta': {'version': 3}}, fh)
    with open(base + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n(rule "room"\n'
                 "  (condition \"A.intersectsArea('ROOM')\")\n"
                 f'  (constraint clearance (min {room}mm)))\n')
    return run_utils.evidence(base + '.kicad_pcb')


def window(board, cls, net_name, r=16):
    """(has an area composition, {cell: blocked}) around the tie for
    `net_name`, after the rules are installed and a real prepare."""
    from kicad_parser import parse_kicad_pcb
    from kicad_dru import install_layer_clearances
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map, build_layer_map
    from routing_context import prepare_obstacles_inplace
    pcb = parse_kicad_pcb(board)
    ids = {n.name: n.net_id for n in pcb.nets.values()}
    nid = ids[net_name]
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=cls, track_width=0.2)
    with contextlib.redirect_stdout(io.StringIO()):
        install_layer_clearances(cfg, None, board, pcb, net_keepouts=True)
        obs = build_base_obstacle_map(pcb, cfg, [nid])
        prepare_obstacles_inplace(obs, pcb, cfg, nid, [nid], [], {},
                                  build_layer_map(cfg.layers), {})
    gx, gy = GridCoord(cfg.grid_step).to_grid(10.0, 10.0)
    return (getattr(cfg, 'area_clearance', None) is not None,
            {(dx, dy): bool(obs.is_blocked(gx + dx, gy + dy, 0))
             for dx in range(-r, r + 1) for dy in range(-r, r + 1)})


def same_as_control(td, tag, cls, room, foreign_track=False):
    real = stage(td, tag, 'F.Cu', cls, room, foreign_track)
    ctl = stage(td, tag + '_ctl', 'F.SilkS', cls, room, foreign_track)
    area_r, w_real = window(real, cls, 'TIED_A')
    area_c, w_ctl = window(ctl, cls, 'TIED_A')
    kept = sorted(k for k in w_real if w_real[k] and not w_ctl[k])
    opened = sorted(k for k in w_real if w_ctl[k] and not w_real[k])
    return area_r and area_c, kept, opened, sum(w_ctl.values())


def fixture(td):
    print('1. the fixture')
    from kicad_parser import parse_kicad_pcb
    board = stage(td, 'fix', 'F.Cu', 0.5, 0.15)
    pcb = parse_kicad_pcb(board)
    graphic = [s for s in pcb.segments if getattr(s, 'graphic', False)
               and getattr(s, 'owner_ref', '') == 'NT1']
    check('the tie has modelled copper', len(graphic) >= 3, f'{len(graphic)} segment(s)')
    area, _w = window(board, 0.5, 'TIED_A')
    check("ROOM's rule is installed as an area composition", area)


def relaxing(td):
    print('2. an area that relaxes the clearance')
    area, kept, opened, n = same_as_control(td, 'relax', 0.5, 0.15)
    check('the area composition is live on both boards', area)
    check('the control blocks cells in the window (non-vacuity)', n > 0, f'{n} cell(s)')
    check('no cell of the tie\'s own copper stays blocked for the tied net',
          not kept, f'{len(kept)} cell(s) blocked that the control leaves free: {kept[:6]}')
    check('no other cell is opened', not opened, f'{len(opened)}: {opened[:6]}')


def tightening(td):
    print('3. an area that tightens the clearance, a foreign track beside the tie')
    area, kept, opened, n = same_as_control(td, 'tight', 0.2, 0.4, foreign_track=True)
    check('the area composition is live on both boards', area)
    check('the control blocks cells in the window (non-vacuity)', n > 0, f'{n} cell(s)')
    check('the lift opens no cell the foreign track blocks',
          not opened, f'{len(opened)} cell(s) free that the control blocks: {opened[:6]}')
    check('no cell of the tie\'s own copper stays blocked', not kept,
          f'{len(kept)}: {kept[:6]}')


def foreign(td):
    print('4. the bridge still blocks a foreign net in the area')
    real = stage(td, 'frn', 'F.Cu', 0.5, 0.15)
    ctl = stage(td, 'frn_ctl', 'F.SilkS', 0.5, 0.15)
    _a, w_real = window(real, 0.5, 'FOREIGN')
    _a, w_ctl = window(ctl, 0.5, 'FOREIGN')
    check('the bridge\'s centre is blocked for FOREIGN and free without the copper',
          w_real[(0, 0)] and not w_ctl[(0, 0)],
          f'real={w_real[(0, 0)]} control={w_ctl[(0, 0)]}')


def routed(td):
    print('5. routed: the tied net leaves the tie on one layer')
    board = stage(td, 'route', 'F.Cu', 0.5, 0.15)
    out = os.path.join(td, 'route_o.kicad_pcb')
    r = subprocess.run(PY + [os.path.join('py_router', 'route.py'), board, out,
                             '--nets', 'TIED_A', '--layers', 'F.Cu'],
                       capture_output=True, text=True, cwd=REPO)
    line = next((ln for ln in r.stdout.splitlines()
                 if ln.startswith('JSON_SUMMARY_MIN:')), None)
    doc = json.loads(line[len('JSON_SUMMARY_MIN:'):]) if line else {}
    check('route.py routes TIED_A', doc.get('routed') == 1 and not doc.get('failed_single'),
          f"routed={doc.get('routed')} failed={doc.get('failed_single')} "
          f"{(r.stdout + r.stderr)[-300:] if not line else ''}")
    # Walled in, the net was recovered only by the rescue pass at a narrower
    # clearance than the board's.
    check('...at the board\'s own rules, with no escalation',
          doc.get('escalations') == 0, f"escalations={doc.get('escalations')} "
          f"min_clearance_used={doc.get('min_clearance_used')}")
    run_utils.evidence(out)
    for tool, extra in (('check_drc.py', []), ('check_connected.py', ['--nets', 'TIED_A'])):
        c = subprocess.run(PY + [os.path.join('py_router', tool), out] + extra,
                           capture_output=True, text=True, cwd=REPO)
        check(f'{tool} is clean on the routed board', c.returncode == 0, c.stdout[-300:])


def main():
    td = tempfile.mkdtemp(prefix='krt1930_')
    try:
        for t in (fixture, relaxing, tightening, foreign, routed):
            t(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
