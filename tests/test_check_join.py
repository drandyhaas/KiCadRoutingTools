#!/usr/bin/env python3
"""check_join.py: the hand-join verifier, promoted from run 7's prototype.

The prototype (wk7/join_check.py) modeled pads as axis-aligned boxes at two
hardcoded constants, never checked zones, dropped layer transitions on the
floor, and skipped own-net vias entirely. This version stages the candidate
onto a copy of the board and diffs the REAL check_drc engine, so the cases
below include exactly what the prototype missed:

  * a 45-degree-rotated pad whose true corner sticks out of its AABB;
  * a layer change with no via (missing-via);
  * a candidate via stacked on an existing own-net via (KiCad permits
    same-net stacks -- only this tool will ever flag one).
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

BOARD_TEXT = '''(kicad_pcb
	(version 20241229)
	(generator "test_check_join")
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
	(net 2 "VICTIM")
	(footprint "test:R45"
		(layer "F.Cu")
		(at 110 110 45)
		(property "Reference" "R1"
			(at 0 -2 0)
			(layer "F.SilkS")
			(effects (font (size 1 1) (thickness 0.15)))
		)
		(pad "1" smd rect
			(at 0 0 45)
			(size 2 1)
			(layers "F.Cu")
			(net 2 "VICTIM")
		)
	)
	(gr_rect
		(start 100 100)
		(end 120 120)
		(stroke (width 0.1) (type default))
		(layer "Edge.Cuts")
	)
	(via
		(at 115 115)
		(size 0.5)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(net 1)
	)
)
'''


def _board(d):
    p = os.path.join(d, 'j.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as f:
        f.write(BOARD_TEXT)
    return p


def _run(args):
    return subprocess.run([sys.executable, '-X', 'utf8',
                           os.path.join(ROOT, 'py_tools', 'check_join.py')] + args,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT)


def test_clean_join_is_clean():
    with tempfile.TemporaryDirectory() as d:
        b = _board(d)
        r = _run([b, 'N1', '104,116,F.Cu', '107,116,F.Cu'])
        assert 'CLEAN' in r.stdout, r.stdout + r.stderr
        assert r.returncode == 0
    print("  PASS: a join in free space is CLEAN, exit 0")


def test_rotated_pad_corner_is_caught():
    """Track at y=111.2 near the 45-degree pad at (110,110), size 2x1: the
    TRUE rotated corner reaches y=111.06 (0.064mm gap to the track edge, a
    real violation at 0.2 clearance); the axis-aligned box ends at y=110.5
    (0.625mm gap -- 'clean'). The prototype's AABB model passed this."""
    with tempfile.TemporaryDirectory() as d:
        b = _board(d)
        r = _run([b, 'N1', '108,111.2,F.Cu', '112,111.2,F.Cu'])
        assert r.returncode == 1, (
            "the rotated pad's corner violation was missed -- the AABB "
            "model is back:\n" + r.stdout + r.stderr)
        assert 'VICTIM' in r.stdout, r.stdout
    print("  PASS: 45-degree pad corner caught (the AABB blind spot)")


def test_straight_short_is_caught():
    with tempfile.TemporaryDirectory() as d:
        b = _board(d)
        r = _run([b, 'N1', '108,110,F.Cu', '112,110,F.Cu'])
        assert r.returncode == 1 and 'VICTIM' in r.stdout, (
            r.stdout + r.stderr)
    print("  PASS: a track through a foreign pad is a violation")


def test_layer_change_without_via_is_missing_via():
    with tempfile.TemporaryDirectory() as d:
        b = _board(d)
        r = _run([b, 'N1', '104,116,F.Cu', '105,116,F.Cu', '105,116,B.Cu',
                  '105,118,B.Cu'])
        assert r.returncode == 1 and 'missing-via' in r.stdout, (
            "a layer transition with no via: token must be a violation "
            "(the prototype silently dropped these pairs):\n" + r.stdout)
        # ... and WITH the via it is clean
        r = _run([b, 'N1', '104,116,F.Cu', '105,116,F.Cu', '105,116,B.Cu',
                  '105,118,B.Cu', 'via:105,116'])
        assert r.returncode == 0 and 'CLEAN' in r.stdout, r.stdout + r.stderr
    print("  PASS: missing-via flagged; the same join with via: is CLEAN")


def test_candidate_via_on_existing_own_net_via_is_stacked():
    with tempfile.TemporaryDirectory() as d:
        b = _board(d)
        r = _run([b, 'N1', 'via:115,115'])
        assert r.returncode == 1 and 'stacked-copper' in r.stdout, (
            "an own-net via stack must be flagged -- KiCad's DRC permits "
            "it, so nothing else ever will:\n" + r.stdout + r.stderr)
    print("  PASS: own-net via stack flagged")


def test_usage_errors_exit_2():
    with tempfile.TemporaryDirectory() as d:
        b = _board(d)
        r = _run([b, 'NOSUCHNET', '104,116,F.Cu', '107,116,F.Cu'])
        assert r.returncode == 2 and 'no net named' in r.stderr, r.stderr
        r = _run([b, 'N1', '104,116,In1.Cu', '107,116,In1.Cu'])
        assert r.returncode == 2 and 'not a copper layer' in r.stderr
        r = _run([b, 'N1', 'garbage'])
        # Not a bare `== 2`: argparse, an ImportError and a parse crash all
        # exit 2, so the code alone cannot say the guard held.
        assert r.returncode == 2 and 'Traceback' not in r.stderr, r.stderr[-400:]
    print("  PASS: unknown net / bad layer / bad token all exit 2")


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"--- {fn.__name__}")
        fn()
    print("ALL PASS")
