#!/usr/bin/env python3
"""A pad whose zone connection is NONE is not served by the pour.

KiCad resolves how a zone joins a pad of its own net as: the pad's own
`(zone_connect N)`, else its footprint's, else the zone's `connect_pads`
(DRC_ENGINE::EvalZoneConnection; PAD::GetZoneConnectionOverrides is the first
two). NONE makes the filler knock the pad out like foreign copper, so the pad
reaches its net only through a track, a via or another pad.

The parser did not read the pad/footprint override, so the fill model and
check_net_connectivity credited pour contact KiCad never makes. Measured:
solder jumper J10's GND half (B.Cu, `(zone_connect 0)`, under
J12's RST half) sat inside the B.Cu GND pour; KiCad left it open with the fill
ending 0.45 mm away, every route step and check_connected's copper grade
called GND complete, and the open shipped.

The board below is the case in miniature: J1.1 opts out, J1.2 inherits the
zone's solid connection, J2 opts out at the FOOTPRINT and its pad 2 opts back
in. tests/gui_parity/test_pad_zone_connect_parity.py runs the same board
through pcbnew (both parse paths, and KiCad's own fill as the oracle).

    python3 -X utf8 tests/test_pad_zone_connect.py
"""
import os
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

#: GND pour on B.Cu (solid, clearance 0.3) under two footprints of GND pads.
BOARD = '''(kicad_pcb (version 20241229) (generator "pcbnew") (general (thickness 1.6))
 (paper "A4")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (34 "B.Paste" user) (36 "B.SilkS" user "B.Silkscreen")
  (37 "F.SilkS" user "F.Silkscreen") (38 "B.Mask" user) (39 "F.Mask" user) (44 "Edge.Cuts" user))
 (setup)
 (net 0 "") (net 1 "GND")
 (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts"))
 (footprint "L:J" (layer "F.Cu") (at 10 10)
  (property "Reference" "J1" (at 0 -3 0) (layer "F.SilkS") (hide yes) (effects (font (size 1 1) (thickness 0.15))))
  (pad "1" smd rect (at -2 0) (size 1 1) (layers "B.Cu" "B.Mask") (net 1 "GND") (zone_connect 0))
  (pad "2" smd rect (at 2 0) (size 1 1) (layers "B.Cu" "B.Mask") (net 1 "GND")))
 (footprint "L:J" (layer "F.Cu") (at 10 20)
  (property "Reference" "J2" (at 0 -3 0) (layer "F.SilkS") (hide yes) (effects (font (size 1 1) (thickness 0.15))))
  (zone_connect 0)
  (attr smd)
  (pad "1" smd rect (at -2 0) (size 1 1) (layers "B.Cu" "B.Mask") (net 1 "GND"))
  (pad "2" smd rect (at 2 0) (size 1 1) (layers "B.Cu" "B.Mask") (net 1 "GND") (zone_connect 2)))
 (zone (net 1) (net_name "GND") (layer "B.Cu") (uuid "00000000-0000-0000-0000-0000000000aa") (hatch edge 0.5)
  (connect_pads yes (clearance 0.3)) (min_thickness 0.2) (filled_areas_thickness no)
  (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
  (polygon (pts (xy 1 1) (xy 39 1) (xy 39 29) (xy 1 29))))
)
'''

#: Pad -> the zone connection KiCad resolves for it against the pour above.
EXPECTED = {('J1', '1'): 'none', ('J1', '2'): 'solid',
            ('J2', '1'): 'none', ('J2', '2'): 'solid'}

FAILURES = []


def check(cond, what, detail=''):
    print(('  ok    ' if cond else '  FAIL  ') + what
          + ('' if cond or not detail else '  -- ' + detail))
    if not cond:
        FAILURES.append(what)


def write_board(directory):
    path = os.path.join(directory, 'zone_connect.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(BOARD)
    return path


def main():
    from kicad_parser import (parse_kicad_pcb, Segment, pad_zone_connection,
                              pad_joins_zone)
    from plane_fill_model import ZoneFillModel
    from check_connected import check_net_connectivity

    path = write_board(tempfile.mkdtemp(prefix='krt_zone_connect_'))
    pcb = parse_kicad_pcb(path)
    gnd = next(n for n, v in pcb.nets.items() if v.name == 'GND')
    zone = next(z for z in pcb.zones if z.net_id == gnd)
    pads = {(p.component_ref, p.pad_number): p for p in pcb.pads_by_net[gnd]}

    print('parse')
    check(zone.pad_connection == 'solid', "the zone's connect_pads yes reads solid",
          repr(zone.pad_connection))
    check(pads['J1', '1'].zone_connection == 'none'
          and pads['J1', '2'].zone_connection is None,
          "a pad's own (zone_connect 0) is read; a pad without one has no override",
          repr((pads['J1', '1'].zone_connection, pads['J1', '2'].zone_connection)))
    check(pcb.footprints['J2'].zone_connection == 'none',
          "a footprint's (zone_connect 0) is read (not cut off by the '(zone' child bound)",
          repr(pcb.footprints['J2'].zone_connection))
    check(pads['J2', '1'].zone_connection == 'none'
          and pads['J2', '2'].zone_connection == 'solid',
          "a pad inherits its footprint's override unless it has its own",
          repr((pads['J2', '1'].zone_connection, pads['J2', '2'].zone_connection)))
    got = {k: pad_zone_connection(p, zone) for k, p in pads.items()}
    check(got == EXPECTED, 'pad_zone_connection resolves pad > footprint > zone', repr(got))

    print('\nresolution rules')
    th = SimpleNamespace(zone_connection=None, pad_type='thru_hole')
    smd = SimpleNamespace(zone_connection=None, pad_type='smd')
    tho = SimpleNamespace(pad_connection='thru_hole_only')
    check(pad_zone_connection(th, tho) == 'thermal'
          and pad_zone_connection(smd, tho) == 'solid',
          'thermal-for-PTH: a plated through-hole pad gets a relief, an SMD pad a solid joint')
    check(pad_zone_connection(smd, SimpleNamespace(pad_connection=None)) == 'thermal',
          "a zone that does not say connects by thermal relief (KiCad's default)")
    check(not pad_joins_zone(smd, SimpleNamespace(pad_connection='none'))
          and pad_joins_zone(SimpleNamespace(zone_connection='solid', pad_type='smd'),
                             SimpleNamespace(pad_connection='none')),
          "a zone's connect_pads no refuses a pad unless the pad overrides it")

    print('\nfill model')
    m = ZoneFillModel(pcb, zone)
    check(m.ok, 'the model builds')
    p11, p12 = pads['J1', '1'], pads['J1', '2']
    check(m.query_component(p11.global_x, p11.global_y) == 0,
          'no fill on a pad the zone does not join (knocked out like foreign copper)')
    check((m.query_component(p11.global_x - 0.5 - 0.3 - 0.15, p11.global_y) or 0) > 0,
          'the knock-out is the zone clearance: fill resumes past pad edge + 0.3')
    check((m.query_component(p12.global_x, p12.global_y) or 0) > 0,
          'a solid-connected pad sits in the fill')

    print('\nconnectivity')
    pad_list = list(pcb.pads_by_net[gnd])
    zones = [zone]

    def open_pads(segs, plist=None, zlist=None, **kw):
        r = check_net_connectivity(gnd, segs, [], plist or pad_list, zlist or zones, **kw)
        hits = set()
        for (x, y, _l, ref) in (r.get('disconnected_pads') or []):
            for (cref, num), p in pads.items():
                if cref == ref and abs(p.global_x - x) < 1e-6 and abs(p.global_y - y) < 1e-6:
                    hits.add((cref, num))
        return hits

    # Outline credit FIRST, on a fresh parse: no fill model has been built for
    # those zone objects, so lookup_zone_model finds none and the legacy
    # outline credit answers (the removal passes' path without pcb_data).
    import plane_fill_model
    bare = parse_kicad_pcb(path)
    bare_zones = [z for z in bare.zones if z.net_id == gnd]
    check(all(plane_fill_model.lookup_zone_model(z) is None for z in bare_zones),
          'witness: no fill model is registered for the fresh parse')
    got = open_pads([], plist=list(bare.pads_by_net[gnd]), zlist=bare_zones)
    check(got == {('J1', '1'), ('J2', '1')},
          'outline credit: exactly the two opted-out pads are open', repr(sorted(got)))
    got = open_pads([], pcb_data=pcb)
    check(got == {('J1', '1'), ('J2', '1')},
          'fill model: exactly the two opted-out pads are open', repr(sorted(got)))
    # The opt-out, not the geometry, decides: lift it and the pad is served.
    # A fresh parse, because fill models are cached on the board object.
    lifted = parse_kicad_pcb(path)
    lp = [p for p in lifted.pads_by_net[gnd]]
    next(p for p in lp if (p.component_ref, p.pad_number) == ('J1', '1')).zone_connection = None
    r = check_net_connectivity(gnd, [], [], lp, [z for z in lifted.zones if z.net_id == gnd],
                               pcb_data=lifted)
    refs = {(ref, round(x, 3)) for (x, y, _l, ref) in (r.get('disconnected_pads') or [])}
    check(('J1', round(p11.global_x, 3)) not in refs,
          'the same pad without the override is credited to the pour', repr(sorted(refs)))
    # The pad needs copper of its own: a track from it into the fill joins it.
    strap = Segment(start_x=p11.global_x, start_y=p11.global_y,
                    end_x=p11.global_x, end_y=p11.global_y - 2.0,
                    width=0.25, layer='B.Cu', net_id=gnd)
    check(('J1', '1') not in open_pads([strap], pcb_data=pcb),
          'a track from the opted-out pad into the fill connects it')

    print()
    if FAILURES:
        print('FAILED: %d check(s): %s' % (len(FAILURES), FAILURES))
        return 1
    print('PASS: zone connection NONE keeps the pour off the pad')
    return 0


if __name__ == '__main__':
    sys.exit(main())
