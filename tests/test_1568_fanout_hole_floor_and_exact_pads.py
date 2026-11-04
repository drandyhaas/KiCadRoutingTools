#!/usr/bin/env python3
"""#1568 follow-ups: the fanouts hold the copper-to-hole rule the way the
route steps do, and bga_fanout measures a via or stub against a pad on
check_drc's own pad geometry.

Copper-to-hole, the route steps' policy (#1217, #1493): an NPTH hole holds a
via's copper off its wall at check_drc's requirement (the pad's override above
the NPTH floor, else max(pair clearance, the board's declared floor)); plated
holes and vias are held at copper clearance; and the writeback writes the
floor the step held, lowered only to what its own copper needs, with a
design_rules row when it is lowered. The fanouts used to test an NPTH pad as
a pad shape at the pair clearance, and wrote the copper clearance as the hole
floor with nothing in the ledger.

Checks:
  1. QFN under-pad, a 0.5 mm NPTH hole 0.185 mm from where a via lands, on a
     board declaring min_hole_clearance 0.25: check_drc finds no VIA-HOLE row
     (was 1), every pad escapes.
     Its copper then clears the floor, and 0.25 is the floor written.
  2. The same board without the hole: the written min_hole_clearance is the
     floor held, lowered to what the step's own copper needs (0.2157 mm, two
     of its vias' copper against each other's drills; was the 0.1 copper
     clearance), with a hole_clearance row in JSON_SUMMARY.design_rules.
  3. bga_fanout's via test against a pad: a via clear of a ROUND pad's copper
     but inside its bounding square is accepted (the rectangle refused it);
     an NPTH hole holds the via at the declared floor, and at the copper
     clearance with none declared.
  4. bga_fanout's segment test against a pad: a segment clear of a ball's
     circle but inside its square corner is no hit at margin 0, and a hit
     only within the margin of the circle.

    python3 tests/test_1568_fanout_hole_floor_and_exact_pads.py
"""
import contextlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from copy_board import copy_board                                  # noqa: E402
from run_utils import check as run_check, evidence                 # noqa: E402
from synth import make_pad                                         # noqa: E402

QFN = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
ARGS = ['--component', 'U2', '--nets', '*', '!GND', '--escape-method', 'underpad',
        '--allow-via-in-pad', '--width', '0.1', '--clearance', '0.1',
        '--via-size', '0.45', '--via-drill', '0.25']
# 0.66 mm from the via a 0.2 mm footprint override puts DATA_31 at
# (212.30, 106.50): copper-to-hole 0.66 - 0.225 - 0.25 = 0.185.
HOLE_AT = (211.64, 106.5)
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stage(dst, with_hole=True):
    """The fixture with a 0.2 mm clearance override on the QFN, an NPTH hole
    at HOLE_AT (unless `with_hole` is False), and a project declaring
    min_hole_clearance 0.25."""
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(QFN, dst)
    with open(dst, encoding='utf-8', newline='') as f:
        text = f.read().rstrip()
    ref = text.index('(property "Reference" "U2"')
    tag = '(tags "QFN NoLead")'
    at = text.rindex(tag, 0, ref) + len(tag)
    text = text[:at] + '\n\t\t(clearance 0.2)' + text[at:]
    hole = f'''
	(footprint "MountingHole:Test_NPTH"
		(layer "F.Cu")
		(uuid "0b6c2f5e-1568-4b00-8000-000000000001")
		(at {HOLE_AT[0]} {HOLE_AT[1]})
		(property "Reference" "H99"
			(at 0 -1 0)
			(layer "F.SilkS")
			(uuid "0b6c2f5e-1568-4b00-8000-000000000002")
			(effects (font (size 1 1) (thickness 0.15)))
		)
		(attr exclude_from_pos_files exclude_from_bom)
		(pad "" np_thru_hole circle
			(at 0 0)
			(size 0.5 0.5)
			(drill 0.5)
			(layers "*.Cu" "*.Mask")
			(uuid "0b6c2f5e-1568-4b00-8000-000000000003")
		)
	)
'''
    assert text.endswith(')')
    with open(dst, 'w', encoding='utf-8', newline='') as f:
        f.write(text[:-1] + (hole if with_hole else '') + ')\n')
    proj = {'board': {'design_settings': {'rules': {
                'min_clearance': 0.0, 'min_track_width': 0.1, 'min_via_diameter': 0.4,
                'min_through_hole_diameter': 0.2, 'min_hole_clearance': 0.25}}},
            'net_settings': {'meta': {'version': 4}, 'classes': [{
                'name': 'Default', 'clearance': 0.1, 'track_width': 0.2,
                'via_diameter': 0.6, 'via_drill': 0.3, 'priority': 2147483647}]}}
    with open(os.path.splitext(dst)[0] + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)
    return evidence(dst)


def fan(inp, out):
    """Run the under-pad escape: (written min_hole_clearance, the
    hole_clearance rows of JSON_SUMMARY.design_rules)."""
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/qfn_fanout.py', inp,
                   '--output', out] + ARGS, accept=True)
    evidence(out, 'qfn_fanout output')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout, re.M)
    if not m:
        raise AssertionError(f"BROKEN TEST: no JSON_SUMMARY\n{r.stdout[-1500:]}")
    js = json.loads(m.group(1))
    check("every pad escaped", js.get('escaped') == 47 and js.get('failed') == 0,
          f"escaped={js.get('escaped')} failed={js.get('failed')}")
    with open(os.path.splitext(out)[0] + '.kicad_pro', encoding='utf-8') as f:
        written = json.load(f)['board']['design_settings']['rules'].get('min_hole_clearance')
    rows = [n for n in (js.get('design_rules') or {}).get('narrowed', [])
            if n.get('kind') == 'hole_clearance']
    return written, rows


def qfn_case(tmp):
    print("1. QFN under-pad beside an NPTH hole, min_hole_clearance 0.25 declared")
    inp = stage(os.path.join(tmp, 'q.kicad_pcb'))
    out = os.path.join(tmp, 'q_fan.kicad_pcb')
    written, rows = fan(inp, out)
    d = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py', out,
                        '--no-size-checks', '--max-print', '0'],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600)
    txt = d.stdout + d.stderr
    if 'Traceback' in txt or not ('NO DRC VIOLATIONS' in txt or 'DRC VIOLATION' in txt):
        raise AssertionError(f"BROKEN TEST: no verdict from check_drc.py\n{txt[-1500:]}")
    if 'Copper-to-hole clearance 0.25' not in txt:
        raise AssertionError("BROKEN TEST: check_drc did not grade at the declared "
                             f"0.25 copper-to-hole floor\n{txt[-1500:]}")
    hits = re.search(r'^VIA-HOLE violations \((\d+)\)', txt, re.M)
    check("check_drc finds no via within the declared floor of the NPTH hole (was 1)",
          hits is None, hits.group(0) if hits else '')
    check("...and the floor it held is written (was the 0.1 copper clearance)",
          written == 0.25 and not rows, f"{written}, rows {rows}")

    print("2. the same board without the hole: the writeback")
    inp = stage(os.path.join(tmp, 'q2.kicad_pcb'), with_hole=False)
    written, rows = fan(inp, os.path.join(tmp, 'q2_fan.kicad_pcb'))
    check("the written copper-to-hole floor is the one held, lowered to the step's "
          "own copper (was the 0.1 copper clearance)",
          written is not None and abs(written - 0.215685) < 1e-4, f"{written}")
    check("...and the lowering is a design_rules row",
          len(rows) == 1 and rows[0].get('requested') == 0.25, f"{rows}")


def bga_via_case():
    print("3. bga_fanout via against a pad: check_drc's pad, and an NPTH hole's floor")
    from bga_fanout.geometry import via_clears_pad_rects
    ball = make_pad(1, 0.0, 0.0, size_x=0.4, size_y=0.4, shape='circle')
    # A 0.3 via on the diagonal: 0.2 + 0.15 + 0.1 + 0.01 from the centre,
    # clear of the circle, inside the bounding square grown by 0.25.
    d = (0.2 + 0.15 + 0.1 + 0.01) / math.sqrt(2)
    check("a via clear of a round pad's copper is accepted (the rectangle refused it)",
          via_clears_pad_rects(d, d, 0.15, 0.1, [ball]) is True)
    check("...and one inside its copper's clearance is refused",
          via_clears_pad_rects(0.2 + 0.15 + 0.09, 0.0, 0.15, 0.1, [ball]) is False)
    hole = make_pad(0, 0.0, 0.0, size_x=0.5, size_y=0.5, shape='circle',
                    layers=('*.Cu', '*.Mask'), drill=0.5, pad_type='np_thru_hole')
    x = 0.25 + 0.15 + 0.15                    # copper-to-hole 0.15
    try:
        held = via_clears_pad_rects(x, 0.0, 0.15, 0.1, [hole], hole_clearance=0.25)
    except TypeError as e:          # the test took no copper-to-hole floor
        held = f"no hole floor: {e}"
    check("an NPTH hole holds the via at the declared 0.25 floor", held is False,
          '' if held is False else f"{held}")
    check("...and at the copper clearance with no floor declared",
          via_clears_pad_rects(x, 0.0, 0.15, 0.1, [hole]) is True)


def bga_seg_case():
    print("4. bga_fanout segment against a ball: the circle, not its square")
    from bga_fanout.reroute import _seg_hits_pad
    ball = make_pad(1, 0.0, 0.0, size_x=0.4, size_y=0.4, shape='circle')
    # A segment through (0.17, 0.17): inside the 0.2 square, 0.04 off the circle.
    p = (0.17, 0.17)
    a, b = (p[0] + 1, p[1] - 1), (p[0] - 1, p[1] + 1)
    check("a centreline past the ball's square corner does not cross the ball",
          _seg_hits_pad(*a, *b, ball) is False)
    gap = math.hypot(*p) - 0.2
    check("...and comes within a margin just over its gap to the circle",
          _seg_hits_pad(*a, *b, ball, margin=gap + 0.001) is True
          and _seg_hits_pad(*a, *b, ball, margin=gap - 0.001) is False,
          f"gap {gap:.4f}")


def main():
    if not os.path.isfile(QFN):
        print(f"SKIP: {os.path.relpath(QFN, ROOT)} missing")
        return 77
    tmp = tempfile.mkdtemp(prefix='t1568b_')
    try:
        qfn_case(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    bga_via_case()
    bga_seg_case()
    print()
    if failures:
        print(f"FAILED: {len(failures)} check(s): " + '; '.join(failures))
        return 1
    print("PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
