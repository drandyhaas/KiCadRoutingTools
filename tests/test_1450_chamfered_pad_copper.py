#!/usr/bin/env python3
"""A chamfered-rect pad's cut corners are not copper (#1450).

KiCad writes a chamfered-rect pad -- its Eagle importer's octagon pad -- as
`roundrect` plus `(chamfer <corners>)` and `(chamfer_ratio r)`. The parser
read the roundrect and dropped the chamfer, so a cut corner counted as
copper: a route's last turn landed there, the end-of-run strict collapse
(#1063) removed the centre-to-corner piece as lying inside the pad, and KiCad
then reported the pad unconnected while route.py, check_drc and
check_connected all passed. The pad now carries its outline in
`Pad.polygons`, which every copper consumer reads exactly.

Checks:
  A. the one-pad board's octagon parses as the chamfered outline, not the
     roundrect: its eight vertices, a point in a cut corner off the copper,
     shape and ratio as pcbnew loads them
  B. chamfered_rect_polygon on one corner, a rotated pad, a rounded
     unchamfered corner, a chamfered `rect` (rounded at 0.25, as pcbnew
     loads it) and a per-layer padstack (left to its box)
  C. a paste opening: a positive margin is the grown box, zero margin the
     outline
  D. route.py on the one-pad board: the shipped track reaches the pad's
     copper, and check_drc and check_connected pass
  E. KiCad's own DRC reports no unconnected item (skipped, and said, when
     kicad-cli is not installed)

    python3 tests/test_1450_chamfered_pad_copper.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence, run_step, tool          # noqa: E402
from check_drc import point_to_pad_distance              # noqa: E402
from kicad_parser import parse_kicad_pcb                 # noqa: E402

RUN_ALL_TIMEOUT = 600
FAILS = []

PAD = '''		(pad "1" thru_hole roundrect
			(at 0.0000 0.0000)
			(size 2.4 2.4)
			(drill 1.2)
			(layers "*.Cu" "*.Mask")
			(remove_unused_layers no)
			(roundrect_rratio 0.25)
			(chamfer_ratio 0.2928932188)
			(chamfer top_left top_right bottom_left bottom_right)
			(net 1 "N1")
			(uuid "00000000-0000-4000-8000-000000000001")
		)
'''

BOARD = '''(kicad_pcb
	(version 20241229)
	(generator "synth")
	(generator_version "9.0")
	(general (thickness 1.6) (legacy_teardrops no))
	(paper "A4")
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(1 "F.Mask" user)
		(3 "B.Mask" user)
		(5 "F.SilkS" user "F.Silkscreen")
		(25 "Edge.Cuts" user)
	)
	(setup
		(pad_to_mask_clearance 0)
	)
	(net 0 "")
	(net 1 "N1")
	(gr_rect
		(start -16.0 -6.0)
		(end 18.54 6.0)
		(stroke (width 0.1) (type default))
		(fill no)
		(layer "Edge.Cuts")
		(uuid "00000000-0000-4000-8000-000000000003")
	)
	(footprint "test:J1"
		(layer "F.Cu")
		(uuid "00000000-0000-4000-8000-000000000004")
		(at 0 0)
		(property "Reference" "J1"
			(at 0 -3 0)
			(layer "F.SilkS")
			(uuid "00000000-0000-4000-8000-000000000005")
			(effects (font (size 1 1) (thickness 0.15)))
		)
''' + PAD + '''	)
	(footprint "test:P1"
		(layer "F.Cu")
		(uuid "00000000-0000-4000-8000-000000000006")
		(at 0 0)
		(property "Reference" "P1"
			(at 0 -3 0)
			(layer "F.SilkS")
			(uuid "00000000-0000-4000-8000-000000000007")
			(effects (font (size 1 1) (thickness 0.15)))
		)
		(pad "1" smd rect
			(at -9.0000 1.0000)
			(size 1.0 1.0)
			(layers "F.Cu" "F.Mask")
			(net 1 "N1")
			(uuid "00000000-0000-4000-8000-000000000002")
		)
	)
)
'''

#: The octagon: a 2.4 mm square cut 0.2928932188 * 2.4 along each side.
CUT = 0.2928932188 * 2.4
OCTAGON = [(-1.2, -1.2 + CUT), (-1.2 + CUT, -1.2), (1.2 - CUT, -1.2), (1.2, -1.2 + CUT),
           (1.2, 1.2 - CUT), (1.2 - CUT, 1.2), (-1.2 + CUT, 1.2), (-1.2, 1.2 - CUT)]


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  [{detail}]' if detail and not cond else ''))


def same_ring(a, b, tol=1e-6):
    return len(a) == len(b) and all(math.hypot(p[0] - q[0], p[1] - q[1]) <= tol
                                    for p, q in zip(a, b))


def t_parse(board):
    print('A. the one-pad board parses as the chamfered outline')
    pad = parse_kicad_pcb(board).footprints['J1'].pads[0]
    polys = pad.polygons or []
    check('the octagon carries one outline', len(polys) == 1, f'polygons={pad.polygons}')
    check('the outline is the octagon, not the roundrect',
          bool(polys) and same_ring(polys[0], OCTAGON), f'{polys}')
    # The issue's track end: |x|+|y| = 2.0, past the octagon's 1.697, and
    # inside the old rounded-rect model (radius 0.6 at that corner).
    d = point_to_pad_distance(-1.0, 1.0, pad)
    check('a point in a cut corner is off the copper (0.21 mm)',
          abs(d - (2.0 - (2.4 - CUT)) / math.sqrt(2)) < 1e-6, f'{d:.4f}')
    check('a point on a flat side is copper', point_to_pad_distance(1.19, 0.0, pad) == 0.0)
    check('shape, size and ratio as pcbnew loads them',
          (pad.shape, pad.size_x, pad.size_y, pad.roundrect_rratio)
          == ('roundrect', 2.4, 2.4, 0.25), f'{pad.shape} {pad.size_x} {pad.roundrect_rratio}')
    check('the pad still discloses the chamfer', 'chamfered pad' in pad.geometry_approximations)


def t_outline():
    print('B. chamfered_rect_polygon')
    try:
        from kicad_parser import chamfered_pad_from_text, chamfered_rect_polygon
    except ImportError as e:
        check('the chamfered-outline builders exist', False, str(e))
        return
    one = chamfered_rect_polygon(0, 0, 2.0, 1.0, 0.0, ['top_left'], 0.2, 0.0)
    check('one sharp chamfered corner: five vertices, the cut 0.2 * min(w, h)',
          same_ring(one, [(-1.0, -0.3), (-0.8, -0.5), (1.0, -0.5), (1.0, 0.5), (-1.0, 0.5)]),
          f'{one}')
    rot = chamfered_rect_polygon(5, 3, 2.0, 1.0, 90.0, ['top_left'], 0.2, 0.0)
    # KiCad's +90 turns the pad frame's (x, y) to board (y, -x).
    check('a pad at 90 degrees turns about the copper centre',
          same_ring(rot, [(5 + y, 3 - x) for x, y in one]), f'{rot}')
    rnd = chamfered_rect_polygon(0, 0, 2.0, 2.0, 0.0, ['top_left'], 0.25, 0.5)
    corner = [(x, y) for x, y in rnd if x > 0.5 + 1e-9 and y > 0.5 + 1e-9]
    check('an unchamfered corner is rounded by the radius',
          len(corner) > 2 and all(abs(math.hypot(x - 0.5, y - 0.5) - 0.5) < 1e-9
                                  for x, y in corner), f'{corner}')
    check('the chamfered corner is cut, not rounded',
          same_ring(rnd[:2], [(-1.0, -0.5), (-0.5, -1.0)]), f'{rnd[:2]}')
    text = PAD.replace('thru_hole roundrect', 'thru_hole rect').replace(
        '\t\t\t(roundrect_rratio 0.25)\n', '').replace(
        'top_left top_right bottom_left bottom_right', 'top_left')
    got = chamfered_pad_from_text(text, 'rect', 2.4, 1.6, 0.0, 0.0, 0.0)
    check('a chamfered `rect` rounds its other corners at 0.25, as pcbnew loads it',
          got is not None and got[1] == 0.25
          and same_ring(got[0][0], chamfered_rect_polygon(
              0, 0, 2.4, 1.6, 0.0, ['top_left'], 0.2928932188, 0.4)), f'{got}')
    stack = PAD.replace('(remove_unused_layers no)', '(padstack (mode custom))')
    check('a per-layer padstack keeps its box',
          chamfered_pad_from_text(stack, 'roundrect', 2.4, 2.4, 0, 0, 0) is None)
    plain = PAD.replace('\t\t\t(chamfer top_left top_right bottom_left bottom_right)\n', '')
    check('a pad naming no corner is not chamfered',
          chamfered_pad_from_text(plain, 'roundrect', 2.4, 2.4, 0, 0, 0) is None)


def t_paste(board):
    print('C. the paste opening')
    from paste_apertures import _inflated_pad
    pad = parse_kicad_pcb(board).footprints['J1'].pads[0]
    grown = _inflated_pad(pad, 0.1, 0.1)
    check('a positive margin is the grown box (a superset), not the outline',
          grown.polygons is None and abs(grown.size_x - 2.6) < 1e-9)
    exact = _inflated_pad(pad, 0.0, 0.0)
    check('zero margin is the copper outline', exact.polygons == pad.polygons)


def _kicad_drc(cli, path, report):
    subprocess.run([cli, 'pcb', 'drc', '--refill-zones', '--severity-all',
                    '--all-track-errors', '--format', 'json', '-o', report, path],
                   capture_output=True, text=True, timeout=300)
    with open(evidence(report, 'kicad-cli DRC report'), encoding='utf-8') as f:
        return json.load(f)


def t_route(d, board):
    print('D. route.py on the one-pad board')
    out = os.path.join(d, 'out.kicad_pcb')
    _rc, log = run_step([tool('route.py'), board, out, '--nets', '*'], outputs=[out])
    pcb = parse_kicad_pcb(out)
    pad = pcb.footprints['J1'].pads[0]
    segs = [s for s in pcb.segments if s.net_id == pad.net_id]
    check('the net is routed', bool(segs))
    ends = [(x, y) for s in segs for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y))]
    # Graded on the octagon itself, not on the parser's model of it: the pad
    # sits at the origin, so its copper is |x|, |y| <= 1.2, |x| + |y| <= 2.4 - CUT.
    inside = [(x, y) for x, y in ends
              if max(abs(x), abs(y)) <= 1.2 + 1e-6 and abs(x) + abs(y) <= 2.4 - CUT + 1e-6]
    check('the shipped track reaches the pad copper', bool(inside),
          f'track ends {ends}; none on the octagon')
    run_step([tool('check_drc.py'), out])
    run_step([tool('check_connected.py'), out])
    print('  PASS check_drc and check_connected pass')
    return out


def t_kicad(d, out):
    print('E. KiCad DRC')
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        print("  SKIP: no kicad-cli found; KiCad's own unconnected-items grade is unchecked here")
        return
    rep = _kicad_drc(cli, out, os.path.join(d, 'drc.json'))
    unc = rep.get('unconnected_items', [])
    check('KiCad reports no unconnected item', not unc,
          '; '.join(i.get('description', '') + ' ' + str([it.get('description') for it in i.get('items', [])])
                    for i in unc))


def main():
    with tempfile.TemporaryDirectory(prefix='t1450_') as d:
        board = os.path.join(d, 'octagon_pad.kicad_pcb')
        with open(board, 'w', encoding='utf-8', newline='\n') as f:
            f.write(BOARD)
        evidence(board, 'the one-pad board')
        t_parse(board)
        t_outline()
        t_paste(board)
        out = t_route(d, board)
        t_kicad(d, out)
    print(f'\n{len(FAILS)} failure(s)' + (': ' + '; '.join(FAILS) if FAILS else ''))
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
