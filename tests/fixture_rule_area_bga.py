"""A synthetic 8x8 BGA in a rule area, for the fan-out's area rules.

U1: 64 balls, 0.8 mm pitch, 0.4 mm pads on F.Cu, each on its own net with a
second pad on J1 (so no ball is a single-pad net). The nets of every other
pair of columns are in class GRP_A, the rest in GRP_B, so each ball has
balls of the other class beside it. Default, GRP_A and GRP_B are all 0.1 mm.
A rule area BGA_ROOM on every copper layer surrounds the ball field, its
edge `margin` + 0.2 mm past the outer balls' copper (below -0.6 it leaves
the outer balls out).

`WIDE` is the spacing rule between the two classes (0.5 mm, one rule per
`(layer outer)` / `(layer inner)`), `ROOM` the area rule (0.1 mm).

    python3 tests/fixture_rule_area_bga.py DIR   # writes the variants
"""
import json
import os
import sys
import uuid

N, PITCH, CX, CY = 8, 0.8, 15.0, 15.0

WIDE = ''.join(f'''(rule "inter-group {lc}" (layer {lc})
  (constraint clearance (min 0.5mm))
  (condition "A.NetClass == 'GRP_A' && B.NetClass == 'GRP_B'"))
''' for lc in ('outer', 'inner'))
ROOM = '''(rule "BGA Tight Clearance"
  (constraint clearance (min 0.1mm))
  (condition "A.intersectsArea('BGA_ROOM')"))
'''


def _u(i):
    return str(uuid.UUID(int=0x1000 + i))


def _net(r, c):
    return f"{'A' if (c // 2) % 2 == 0 else 'B'}{r}{c}"


def board_text(margin=1.0):
    balls, pins = [], []
    for k in range(N * N):
        r, c = divmod(k, N)
        name = _net(r, c)
        balls.append(f'''		(pad "{chr(65 + r)}{c + 1}" smd circle (at {(c - (N - 1) / 2) * PITCH:g} {(r - (N - 1) / 2) * PITCH:g})
			(size 0.4 0.4) (layers "F.Cu" "F.Mask" "F.Paste") (net "{name}") (uuid "{_u(k)}"))''')
        pins.append(f'''		(pad "{k + 1}" smd rect (at {(k % 16) * 1.0:g} {(k // 16) * 1.0:g})
			(size 0.5 0.5) (layers "F.Cu" "F.Mask") (net "{name}") (uuid "{_u(1000 + k)}"))''')
    h = N * PITCH / 2 + margin
    nl = chr(10)
    return f'''(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(generator_version "10.0")
	(general (thickness 1.6) (legacy_teardrops no))
	(paper "A4")
	(layers
		(0 "F.Cu" signal)
		(4 "In1.Cu" signal)
		(6 "In2.Cu" signal)
		(2 "B.Cu" signal)
		(13 "F.Paste" user)
		(15 "B.Paste" user)
		(5 "F.SilkS" user "F.Silkscreen")
		(7 "B.SilkS" user "B.Silkscreen")
		(1 "F.Mask" user)
		(3 "B.Mask" user)
		(25 "Edge.Cuts" user)
		(31 "F.CrtYd" user "F.Courtyard")
		(29 "B.CrtYd" user "B.Courtyard")
		(35 "F.Fab" user)
		(33 "B.Fab" user)
	)
	(setup (pad_to_mask_clearance 0) (allow_soldermask_bridges_in_footprints no))
	(footprint "test:BGA64_P0.8"
		(layer "F.Cu")
		(uuid "{_u(800)}")
		(at {CX:g} {CY:g})
		(property "Reference" "U1" (at 0 -5 0) (layer "F.SilkS") (uuid "{_u(801)}") (effects (font (size 1 1))))
		(property "Value" "BGA64" (at 0 5 0) (layer "F.Fab") (uuid "{_u(802)}") (effects (font (size 1 1))))
		(attr smd)
{nl.join(balls)}
	)
	(footprint "test:PADS64"
		(layer "F.Cu")
		(uuid "{_u(810)}")
		(at 7.5 24)
		(property "Reference" "J1" (at 0 -2 0) (layer "F.SilkS") (uuid "{_u(811)}") (effects (font (size 1 1))))
		(property "Value" "PADS64" (at 0 5 0) (layer "F.Fab") (uuid "{_u(812)}") (effects (font (size 1 1))))
		(attr smd)
{nl.join(pins)}
	)
	(gr_rect (start 0 0) (end 30 30) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts") (uuid "{_u(901)}"))
	(zone
		(layers "F.Cu" "In1.Cu" "In2.Cu" "B.Cu")
		(uuid "{_u(900)}")
		(name "BGA_ROOM")
		(hatch edge 0.5)
		(connect_pads (clearance 0))
		(min_thickness 0.25)
		(keepout (tracks allowed) (vias allowed) (pads allowed) (copperpour allowed) (footprints allowed))
		(placement (enabled no) (sheetname ""))
		(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
		(polygon (pts (xy {CX - h:g} {CY - h:g}) (xy {CX + h:g} {CY - h:g}) (xy {CX + h:g} {CY + h:g}) (xy {CX - h:g} {CY + h:g})))
	)
)
'''


def project(name):
    base = {"name": "Default", "clearance": 0.1, "track_width": 0.1,
            "via_diameter": 0.3, "via_drill": 0.15, "microvia_diameter": 0.25,
            "microvia_drill": 0.1, "diff_pair_width": 0.1, "diff_pair_gap": 0.1,
            "diff_pair_via_gap": 0.25, "priority": 2147483647}
    return {"board": {"design_settings": {"rules": {
                "min_clearance": 0.1, "min_track_width": 0.1, "min_via_diameter": 0.3,
                "min_via_annular_width": 0.075, "min_through_hole_diameter": 0.15,
                "min_hole_clearance": 0.1, "min_hole_to_hole": 0.2,
                "min_copper_edge_clearance": 0.3}}},
            "net_settings": {
                "classes": [base, dict(base, name="GRP_A", priority=0),
                            dict(base, name="GRP_B", priority=1)],
                "meta": {"version": 4},
                "netclass_patterns": [{"netclass": "GRP_A", "pattern": "A*"},
                                      {"netclass": "GRP_B", "pattern": "B*"}]},
            "meta": {"filename": name + ".kicad_pro", "version": 3}}


def write(directory, name, dru, margin=1.0):
    """`directory/name.kicad_pcb` with its .kicad_pro and a .kicad_dru
    holding `dru`. Returns the board path."""
    os.makedirs(directory, exist_ok=True)
    stem = os.path.join(directory, name)
    with open(stem + '.kicad_pcb', 'w', encoding='utf-8') as fh:
        fh.write(board_text(margin))
    with open(stem + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(project(name), fh, indent=2)
    with open(stem + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n' + dru)
    return stem + '.kicad_pcb'


if __name__ == '__main__':
    out = sys.argv[1]
    for nm, d in (('none', ''), ('wide', WIDE), ('room', WIDE + ROOM),
                  ('room_first', ROOM + WIDE)):
        print(write(out, nm, d))
