#!/usr/bin/env python3
"""#1800: check_join moves a join end typed within a micron of a same-net
track end, via centre or pad anchor ONTO it, and says so -- or, with
--no-snap, refuses it by name.

KiCad merges two tracks into one run only when their ends are equal to the
nanometre, and cuts a run at a pad's outline only when the run ends exactly on
the pad's anchor. A join 40 nm off either connects copper to copper, prints
CLEAN, and is measured with its in-pad stretch counted in full: 0.12 mm longer
here, more than a 0.1 mm intra-pair skew tolerance.

Fixture: net N1 has a round 1.7 mm pad at (105,105); the TRACK board also has
an N1 track from that anchor to (104.7,105.6), a point inside the pad.
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import check, evidence  # noqa: E402

TOOL = os.path.join(ROOT, 'py_tools', 'check_join.py')

TRACK = '''	(segment
		(start 105 105)
		(end 104.7 105.6)
		(width 0.25)
		(layer "F.Cu")
		(net 1)
		(uuid "11111111-2222-3333-4444-555555555555")
	)
'''

BOARD = '''(kicad_pcb
	(version 20241229)
	(generator "test_1800")
	(general
		(thickness 1.6)
	)
	(layers
		(0 "F.Cu" signal)
		(31 "B.Cu" signal)
		(44 "Edge.Cuts" user)
	)
	(net 0 "")
	(net 1 "N1")
	(footprint "test:PAD"
		(layer "F.Cu")
		(at 105 105)
		(property "Reference" "J1"
			(at 0 -2 0)
			(layer "F.SilkS")
			(effects (font (size 1 1) (thickness 0.15)))
		)
		(pad "1" smd circle
			(at 0 0)
			(size 1.7 1.7)
			(layers "F.Cu")
			(net 1 "N1")
		)
	)
	(gr_rect
		(start 100 100)
		(end 120 120)
		(stroke (width 0.1) (type default))
		(layer "Edge.Cuts")
	)
%s)
'''


def _board(d, with_track):
    p = os.path.join(d, 'track.kicad_pcb' if with_track else 'pad.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as f:
        f.write(BOARD % (TRACK if with_track else ''))
    return p


def _join(board, kept, *tokens, extra=()):
    r = check([sys.executable, '-X', 'utf8', TOOL, board, 'N1', *tokens,
               '--keep-staged', kept, *extra], accept=True)
    evidence(kept, 'the kept board')
    return r.stdout


def _net(path):
    from kicad_parser import parse_kicad_pcb
    from net_queries import kicad_net_length_nm, _nm
    b = parse_kicad_pcb(path)
    nid = next(i for i, n in b.nets.items() if n.name == 'N1')
    segs = [s for s in b.segments if s.net_id == nid]
    length = kicad_net_length_nm(b, segs, [v for v in b.vias if v.net_id == nid],
                                 b.nets[nid].pads)
    ends = sorted(((_nm(s.start_x), _nm(s.start_y)), (_nm(s.end_x), _nm(s.end_y)))
                  for s in segs)
    return length, ends


def test_near_miss_of_a_track_end_is_snapped():
    with tempfile.TemporaryDirectory() as d:
        b = _board(d, True)
        exact = os.path.join(d, 'exact.kicad_pcb')
        off = os.path.join(d, 'off.kicad_pcb')
        out = _join(b, exact, '104.7,105.6,F.Cu', '102,105.6,F.Cu')
        assert 'snapped' not in out, out
        out = _join(b, off, '104.69996,105.6,F.Cu', '102,105.6,F.Cu')
        assert ('snapped (104.699960,105.600000) -> (104.700000,105.600000)'
                in out and 'track end' in out), out
        le, ee = _net(exact)
        lo, eo = _net(off)
        assert eo == ee, f"kept board's ends differ from the exact join: {eo} vs {ee}"
        assert lo == le, f"KiCad length {lo} nm != the exact join's {le} nm"
    print("  PASS: a join 40 nm off a same-net track end lands on it; length = exact join's")


def test_near_miss_of_a_pad_anchor_is_snapped():
    with tempfile.TemporaryDirectory() as d:
        b = _board(d, False)
        exact = os.path.join(d, 'exact.kicad_pcb')
        off = os.path.join(d, 'off.kicad_pcb')
        _join(b, exact, '105,105,F.Cu', '104.7,105.6,F.Cu', '102,105.6,F.Cu')
        out = _join(b, off, '105.00004,105,F.Cu', '104.7,105.6,F.Cu',
                    '102,105.6,F.Cu')
        assert 'snapped (105.000040,105.000000) -> (105.000000,105.000000)' in out \
            and 'pad anchor J1.1' in out, out
        le, ee = _net(exact)
        lo, eo = _net(off)
        assert eo == ee and lo == le, (lo, le, eo, ee)
    print("  PASS: a join 40 nm off a same-net pad anchor lands on it; length = exact join's")


def test_no_snap_refuses_by_name():
    with tempfile.TemporaryDirectory() as d:
        b = _board(d, True)
        check([sys.executable, '-X', 'utf8', TOOL, b, 'N1',
               '104.69996,105.6,F.Cu', '102,105.6,F.Cu', '--no-snap'],
              refuse='near-miss-endpoint', code=1)
    print("  PASS: --no-snap refuses the near miss as near-miss-endpoint, exit 1")


def test_five_um_off_is_neither_snapped_nor_refused():
    with tempfile.TemporaryDirectory() as d:
        b = _board(d, True)
        kept = os.path.join(d, 'k.kicad_pcb')
        out = _join(b, kept, '104.695,105.6,F.Cu', '102,105.6,F.Cu',
                    extra=('--no-snap',))
        assert 'snapped' not in out and 'near-miss' not in out, out
        _l, ends = _net(kept)
        assert ((104695000, 105600000), (102000000, 105600000)) in ends, ends
    print("  PASS: 5 um off a same-net end is a separate end: not moved, not refused")


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"--- {fn.__name__}")
        fn()
    print("ALL PASS")
