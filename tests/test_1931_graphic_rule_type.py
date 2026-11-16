"""A footprint's copper graphic is a Graphic to a type-scoped clearance rule,
never a Track (#1931).

A class/type-scoped `.kicad_dru` clearance rule (#1491) reads an item's
`Type`. KiCad calls a footprint's copper shape -- a net tie's bridge, a tab --
`Graphic`, so a rule on `B.Type == 'Track'` does not bind it and a rule on
`B.Type == 'Graphic'` does. The scoped channel used to ask for the types
`track` and `via` whenever the obstacle was not a pad, so both the router's
stamps and check_drc's grade read the graphic as a track: a track-typed rule
kept a group's tracks that far from every net tie (and check_drc reported
phantom rows), while a graphic-typed rule was neither routed to nor graded.
The #735 track-rule channel (`A.Type == 'track' && B.Type == 'track'`) had the
same blind spot.

The board is #908's net tie (a filled F.Cu bridge between two tied pads) with
a FOREIGN track 0.4 mm from the bridge, FOREIGN in net class GRP, every class
at 0.1 mm, and one 0.6 mm rule. kicad-cli 10.0.6:

    rule binds                          kicad-cli
    GRP vs a Track or Arc               no clearance violation
    GRP vs a Graphic                    clearance 0.6000 mm; actual 0.4000 mm
    GRP track vs track (#735 shape)     no clearance violation

A `.kicad_dru` hole_clearance rule binds the graphic as well: #1935's tied via
0.20 mm off the bridge under `A.NetName != B.NetName` at 0.3 mm is "Hole
clearance violation (rule 'hole to copper' clearance 0.3000 mm; actual 0.2000
mm)" to kicad-cli, and check_drc used to grade it at the declared floor.

    python3 tests/test_1931_graphic_rule_type.py
"""
import json
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

PY = [sys.executable, '-X', 'utf8']
FAILS = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        FAILS.append(label)


BOARD = """(kicad_pcb (version 20241229) (generator "pcbnew")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (2 "B.Cu" signal)
    (25 "Edge.Cuts" user)
  )
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "TIED_A")
  (net 2 "TIED_B")
  (net 3 "FOREIGN")
  (footprint "test:net_tie" (layer "F.Cu")
    (uuid "aaaaaaaa-0000-4000-8000-000000000010")
    (at 10 10)
    (property "Reference" "NT1" (at 0 -2 0) (layer "F.SilkS") (uuid "aaaaaaaa-0000-4000-8000-000000000011") (effects (font (size 1 1) (thickness 0.15))))
    (net_tie_pad_groups "1, 2")
    %(bridge)s
    (pad "1" smd rect (at -0.4 0) (size 0.3 0.2) (layers "F.Cu") (net 1 "TIED_A") (uuid "aaaaaaaa-0000-4000-8000-000000000012"))
    (pad "2" smd rect (at 0.4 0) (size 0.3 0.2) (layers "F.Cu") (net 2 "TIED_B") (uuid "aaaaaaaa-0000-4000-8000-000000000013"))
  )
  (segment (start 8 10.6) (end 12 10.6) (width 0.2) (layer "F.Cu") (net 3) (uuid "aaaaaaaa-0000-4000-8000-000000000020"))
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)) (uuid "aaaaaaaa-0000-4000-8000-000000000030"))
)
"""

#: the bridge as KiCad writes a net tie: a FILLED fp_poly of stroke width 0
POLY = ('(fp_poly (pts (xy -0.4 -0.1) (xy 0.4 -0.1) (xy 0.4 0.1) (xy -0.4 0.1)) '
        '(stroke (width 0) (type solid)) (fill yes) (layer "F.Cu") '
        '(uuid "aaaaaaaa-0000-4000-8000-000000000001"))')
#: and as a stroked line: the same 0.2 mm wide copper, edge at y 10.1
LINE = ('(fp_line (start -0.4 0) (end 0.4 0) (stroke (width 0.2) (type solid)) '
        '(layer "F.Cu") (uuid "aaaaaaaa-0000-4000-8000-000000000002"))')

RULES = {
    'none': '',
    'track': "(rule \"grp\" (constraint clearance (min 0.6mm)) "
             "(condition \"A.hasNetclass('GRP') && (B.Type == 'Track' || B.Type == 'Arc')\"))",
    'graphic': "(rule \"grp\" (constraint clearance (min 0.6mm)) "
               "(condition \"A.hasNetclass('GRP') && B.Type == 'Graphic'\"))",
    # #735's track-pair shape, read by the TrackRule channel
    'track735': "(rule \"grp\" (constraint clearance (min 0.6mm)) "
                "(condition \"A.NetClass == 'GRP' && A.Type == 'track' && B.Type == 'track'\"))",
}


def write(td, name, rule, bridge=POLY):
    b = os.path.join(td, name + '.kicad_pcb')
    with open(b, 'w') as f:
        f.write(BOARD % {'bridge': bridge})
    cls = {'clearance': 0.1, 'track_width': 0.1, 'via_diameter': 0.45, 'via_drill': 0.2,
           'diff_pair_gap': 0.1, 'diff_pair_width': 0.1}
    pro = {'board': {'design_settings': {'rules': {
               'min_clearance': 0.0, 'min_track_width': 0.1, 'min_hole_clearance': 0.1,
               'min_hole_to_hole': 0.1, 'min_copper_edge_clearance': 0.0}}},
           'net_settings': {'meta': {'version': 4}, 'classes': [
               dict(cls, name='Default', priority=2147483647),
               dict(cls, name='GRP', priority=0)],
               'netclass_patterns': [{'netclass': 'GRP', 'pattern': 'FOREIGN'}]},
           'meta': {'version': 3}}
    with open(os.path.splitext(b)[0] + '.kicad_pro', 'w') as f:
        json.dump(pro, f)
    if rule:
        with open(os.path.splitext(b)[0] + '.kicad_dru', 'w') as f:
            f.write(f'(version 1)\n{rule}\n')
    return b


def drc_rows(b):
    """check_drc's clearance rows between FOREIGN and the bridge."""
    out = os.path.splitext(b)[0] + '_drc.json'
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), b,
                             '--clearance', '0.1', '--json', out],
                       capture_output=True, text=True, cwd=REPO)
    if not os.path.isfile(out):
        raise RuntimeError(f'check_drc wrote no {out}: {r.stdout[-600:]}{r.stderr[-600:]}')
    doc = json.load(open(out))
    rows = [v for v in doc.get('items', [])
            if 'FOREIGN' in (v.get('net1'), v.get('net2'))
            and 'NT1' in f"{v.get('item1')} {v.get('item2')}"]
    return rows, r


def drc_rows_any(b, net):
    """check_drc's rows that touch `net`."""
    out = os.path.splitext(b)[0] + '_drc.json'
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), b,
                             '--clearance', '0.1', '--json', out],
                       capture_output=True, text=True, cwd=REPO)
    if not os.path.isfile(out):
        raise RuntimeError(f'check_drc wrote no {out}: {r.stdout[-600:]}{r.stderr[-600:]}')
    return [v for v in json.load(open(out)).get('items', [])
            if net in (v.get('net1'), v.get('net2'))], r


def router(b, routed='FOREIGN'):
    """(the stamp of the bridge's graphic segments on F.Cu, blocked F.Cu cells
    near the bridge) for a run routing `routed`."""
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from kicad_dru import install_layer_clearances, install_track_clearances
    from obstacle_map import build_base_obstacle_map
    from routing_config import GridCoord
    pcb = parse_kicad_pcb(b)
    nid = next(n.net_id for n in pcb.nets.values() if n.name == routed)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    cfg.clearance = 0.1
    cfg.track_width = 0.1
    cfg.grid_step = 0.05
    cfg.set_net_clearances({}, [nid])     # every class is at 0.1 mm
    install_layer_clearances(cfg, None, b, pcb)
    install_track_clearances(cfg, None, b, pcb, routed_net_ids=[nid])
    graphics = [s for s in pcb.segments if getattr(s, 'graphic', False)]
    assert graphics, 'the bridge parsed as no graphic copper'
    try:
        stamp = max(cfg.track_obstacle_clearance(
            s.net_id, cfg.layer_clearance(s.layer, cfg.obstacle_clearance(s.net_id, graphic=True),
                                          s.net_id, graphic=True), graphic=True)
            for s in graphics)
    except TypeError:
        stamp = None    # a config with no notion of a graphic obstacle
    if stamp is None:
        stamp = float('nan')
    obs = build_base_obstacle_map(pcb, cfg, [nid])
    coord = GridCoord(cfg.grid_step)
    (x0, y0), (x1, y1) = coord.to_grid(9.0, 10.1), coord.to_grid(11.0, 10.5)
    # the strip between the bridge (y 10.1) and the FOREIGN track, x over the bridge
    blocked = sum(1 for ix in range(x0, x1 + 1) for iy in range(y0, y1 + 1)
                  if obs.is_blocked(ix, iy, 0))
    return stamp, blocked


def main():
    td = tempfile.mkdtemp(prefix='krt1931_')
    print('1. check_drc grades the graphic as KiCad types it')
    for bridge_name, bridge in (('polygon', POLY), ('line', LINE)):
        got = {}
        for key in ('none', 'track', 'graphic', 'track735'):
            rows, r = drc_rows(write(td, f'{bridge_name}_{key}', RULES[key], bridge))
            got[key] = rows
        check(f'{bridge_name}: no rule -> clean', not got['none'], f"{len(got['none'])} rows")
        check(f'{bridge_name}: a Track/Arc rule does not bind the graphic',
              not got['track'], f"{len(got['track'])} rows")
        check(f'{bridge_name}: a Graphic rule binds it at 0.6 mm',
              bool(got['graphic'])
              and all(abs((v.get('required_mm') or 0) - 0.6) < 1e-6 for v in got['graphic']),
              f"{[(v.get('type'), v.get('required_mm')) for v in got['graphic']]}")
        check(f'{bridge_name}: a #735 track-pair rule does not bind it',
              not got['track735'], f"{len(got['track735'])} rows")

    print('2. the router stamps the graphic as KiCad types it')
    st = {k: router(write(td, f'route_{k}', RULES[k])) for k in RULES}
    for k, (stamp, blocked) in st.items():
        print(f'     {k:9s} stamp {stamp:.3f} mm, {blocked} cells blocked between bridge and track')
    check('a Track/Arc rule leaves the graphic at the class 0.1 mm',
          abs(st['track'][0] - 0.1) < 1e-9 and st['track'][1] == st['none'][1])
    check('a #735 track-pair rule leaves it at 0.1 mm too',
          abs(st['track735'][0] - 0.1) < 1e-9 and st['track735'][1] == st['none'][1])
    check('a Graphic rule stamps it at 0.6 mm, and blocks more of the strip',
          abs(st['graphic'][0] - 0.6) < 1e-9 and st['graphic'][1] > st['none'][1])

    print('3. the rule still binds a real track as before')
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from kicad_dru import install_track_clearances
    b = write(td, 'trk', RULES['track'])
    pcb = parse_kicad_pcb(b)
    fid = next(n.net_id for n in pcb.nets.values() if n.name == 'FOREIGN')
    aid = next(n.net_id for n in pcb.nets.values() if n.name == 'TIED_A')
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    cfg.clearance = 0.1
    install_track_clearances(cfg, None, b, pcb, routed_net_ids=[fid])
    v = cfg.layer_clearance('F.Cu', cfg.obstacle_clearance(aid), aid)
    check("TIED_A's track is stamped at the Track rule's 0.6 mm", abs(v - 0.6) < 1e-9, f'{v}')

    print('4. a hole_clearance rule binds the graphic too (#1935\'s board)')
    hole_rule = ('(rule "hole to copper" (constraint hole_clearance (min 0.3mm)) '
                 '(condition "A.NetName != B.NetName"))')
    via = ('  (via (at 9.8 10.4) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") (net 1) '
           '(uuid "aaaaaaaa-0000-4000-8000-000000000040"))\n')
    b = write(td, 'hole', hole_rule)
    txt = open(b).read().rstrip()
    with open(b, 'w') as f:
        f.write(txt[:-1] + via + ')\n')
    rows, _r = drc_rows_any(b, 'TIED_A')
    hole = [v for v in rows if v.get('type') == 'track-hole']
    # kicad-cli: "Hole clearance violation (rule 'hole to copper' clearance
    # 0.3000 mm; actual 0.2000 mm)", Polygon [<no net>] of NT1 and the via
    check("TIED_A's via beside the bridge is graded at the rule's 0.3 mm",
          bool(hole) and all(abs((v.get('required_mm') or 0) - 0.3) < 1e-6
                             and v.get('rule') == 'hole to copper' for v in hole),
          f"{[(v.get('required_mm'), v.get('rule')) for v in hole]}")

    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        sys.exit(1)
    print('PASS')


if __name__ == '__main__':
    main()
