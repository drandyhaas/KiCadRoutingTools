#!/usr/bin/env python3
"""#962: the placement channel for footprint GRAPHIC copper past the outline.

`grade_pad_legality` gains `oob_graphic_copper_*`. check_assembly publishes
them. render_placement adds `checklist.a_off_outline.graphic_copper`, and its
--gate fails on it. All three CALL check_drc's `footprint_graphic_outline_census`,
so this pins that they AGREE, rather than re-deriving the geometry a fourth time.

Invariants:
1. The three U2 poses: count, amount, refs and the non-gating edge-floor
   shortfall disclosure (116.70 is 0.25 mm inside, so 0.30 short of 0.55).
2. U2 at 115.34 x rotations 0/90/180/270/45, and flipped to the B side: legality
   == check_drc == render_placement, to 1 um. At least one pose is off the board
   and at least one is on it, so the comparison is not vacuous.
3. The stroke counts AS DRAWN: a filled poly whose outline sits 0.2 mm inside the
   edge is off the board at stroke 0.5 (reach 0.25) and on it at stroke 0.3.
4. A chamfered outline: copper across the chamfer but inside the bounding box
   is caught. The bbox alone would miss it.
5. check_assembly publishes the keys, and its buildable VERDICT does not move
   (the #937 contract for the off-outline channels). render_placement --gate
   fails on the graphic channel.
6. A part moved IN MEMORY is re-posed from its parse pose, and grades the
   same as the written-and-reparsed board.

Run:
    python3 tests/test_962_graphic_oob_channel.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from copy_board import copy_board  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from check_drc import run_drc, footprint_graphic_outline_census  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402
from placement.legality import grade_pad_legality  # noqa: E402
from run_utils import check as run_check  # noqa: E402

FAILS = []
ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def stage(work, name, placements):
    out = os.path.join(work, name + '.kicad_pcb')
    copy_board(ESP, out)
    if placements:
        write_placed_output(out, out, placements)
    return out


def u2(x, rot=90, side=None):
    d = dict(reference='U2', new_x=x, new_y=93.6, new_rotation=rot)
    if side:
        d['new_side'] = side
    return [d]


def three_ways(work, board):
    """U2's overrun by legality, by check_drc and by render_placement."""
    leg = dict(grade_pad_legality(parse_kicad_pcb(board), 0.25)['oob_graphic_copper_refs'])
    viol = run_drc(board, clearance=0.25, clearance_margin=0.0, quiet=True,
                   print_summary=False)
    drc = {v['owner_ref']: v['overrun_mm'] for v in viol
           if v.get('type') == 'graphic-off-board'}
    js = os.path.join(work, 'rp.json')
    run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_tools', 'render_placement.py'),
               board, '--json-out', js, '-o', os.path.join(work, 'rp.png')], accept=True)
    rp = dict(json.load(open(js, encoding='utf-8'))['checklist']['a_off_outline']['graphic_copper'])
    return leg.get('U2'), drc.get('U2'), rp.get('U2')


def main():
    work = tempfile.mkdtemp(prefix='krt962c_')
    try:
        # 1
        rows = {}
        for nm, pl in (('orig', []), ('bad', u2(115.34)), ('ok', u2(116.70))):
            rows[nm] = grade_pad_legality(parse_kicad_pcb(stage(work, nm, pl)), 0.25)
        b = rows['bad']
        check('1. 115.34: count 1, amount 1.11, refs [[U2, 1.11]]',
              b['oob_graphic_copper_count'] == 1
              and abs(b['oob_graphic_copper_amount'] - 1.11) <= 0.005
              and b['oob_graphic_copper_refs'] == [['U2', b['oob_graphic_copper_amount']]],
              str({k: b[k] for k in b if 'graphic' in k and 'basis' not in k}))
        for nm in ('orig', 'ok'):
            check('1. %s: count 0, amount 0.0' % nm,
                  rows[nm]['oob_graphic_copper_count'] == 0
                  and rows[nm]['oob_graphic_copper_amount'] == 0.0)
        sf = dict(rows['ok']['graphic_edge_shortfall_refs'])
        check('1. 116.70: the 0.30 mm edge-floor shortfall is DISCLOSED (not gated)',
              abs(sf.get('U2', 0) - 0.30) <= 0.005, str(sf))
        check('1. every key carries its basis', bool(b.get('oob_graphic_copper_basis')))

        # 2
        seen_off = seen_on = False
        for rot, side in ((0, None), (90, None), (180, None), (270, None), (45, None), (90, 'B')):
            bd = stage(work, 'r%s%s' % (rot, side or ''), u2(115.34, rot, side))
            leg, drc, rp = three_ways(work, bd)
            if leg or drc or rp:
                seen_off = True
            else:
                seen_on = True
            agree = ((leg is None and drc is None and rp is None)
                     or (None not in (leg, drc, rp)
                         and abs(leg - drc) <= 1e-3 and abs(leg - rp) <= 1e-3))
            check('2. rot %s%s: legality == check_drc == render_placement'
                  % (rot, ' B-side' if side else ''), agree, f'{leg} {drc} {rp}')
        check('2. the sweep has an off-board pose AND an on-board one (not vacuous)',
              seen_off and seen_on)

        # 3 / 4 -- synthetic outline and stroke
        def board(body, name, outline=None):
            p = os.path.join(work, name + '.kicad_pcb')
            edge = outline or ('(gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) '
                               '(type solid)) (fill no) (layer "Edge.Cuts"))')
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write('(kicad_pcb (version 20240108) (generator "t")\n'
                         ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                         ' (setup)\n (net 0 "")\n (net 1 "/A")\n %s\n%s\n)' % (edge, body))
            return p

        def tab(width, x_end):
            return ('(footprint "L:P" (layer "F.Cu") (at 30 10) (property "Reference" "U9")\n'
                    ' (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A"))\n'
                    ' (fp_poly (pts (xy 0 -0.5) (xy %s -0.5) (xy %s 0.5) (xy 0 0.5)) '
                    '(stroke (width %s) (type solid)) (fill yes) (layer "F.Cu") (uuid "t")))'
                    % (x_end, x_end, width))
        for w, want in ((0.5, True), (0.3, False)):
            g = grade_pad_legality(parse_kicad_pcb(board(tab(w, 9.8), 'w%s' % w)), 0.25)
            check('3. outline 0.2 mm inside the edge, stroke %s: off=%s' % (w, want),
                  (g['oob_graphic_copper_count'] == 1) == want,
                  str(g['oob_graphic_copper_refs']))
        chamfer = ('(gr_poly (pts (xy 0 0) (xy 35 0) (xy 40 5) (xy 40 30) (xy 0 30)) '
                   '(stroke (width 0.1) (type solid)) (fill no) (layer "Edge.Cuts"))')
        corner = ('(footprint "L:P" (layer "F.Cu") (at 37 2) (property "Reference" "U8")\n'
                  ' (pad "1" smd rect (at -2 1) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A"))\n'
                  ' (fp_poly (pts (xy 0 0) (xy 2 0) (xy 2 1) (xy 0 1)) '
                  '(stroke (width 0.1) (type solid)) (fill yes) (layer "F.Cu") (uuid "k")))')
        pc = parse_kicad_pcb(board(corner, 'chamfer', outline=chamfer))
        g = grade_pad_legality(pc, 0.25)
        bb = pc.board_info.board_bounds
        check('4. copper across a chamfer, inside the bounding box, is caught',
              g['oob_graphic_copper_count'] == 1 and bb and 37 + 2 <= bb[2],
              f"{g['oob_graphic_copper_refs']} bounds={bb}")

        # 5 -- check_assembly and the render_placement gate
        bad = stage(work, 'bad5', u2(115.34))
        js = os.path.join(work, 'asm.json')
        run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_tools', 'check_assembly.py'),
                   bad, '--json', js], accept=True)
        a = json.load(open(js, encoding='utf-8'))
        check('5. check_assembly publishes oob_graphic_copper_* keys',
              a.get('oob_graphic_copper_count') == 1
              and a.get('oob_graphic_copper_refs', [[None]])[0][0] == 'U2', str(a.get('oob_graphic_copper_refs')))
        check('5. ... and its buildable verdict is not moved by them (#937 contract)',
              a.get('buildable') is True, str(a.get('verdict')))
        r = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_tools', 'render_placement.py'),
                       bad, '--gate', '-o', os.path.join(work, 'g.png')],
                      refuse='a_off_outline.graphic_copper')
        check('5. render_placement --gate names a_off_outline.graphic_copper', r.returncode != 0)

        # 6 -- in-memory re-pose == written board
        mem = parse_kicad_pcb(ESP)
        fp = mem.footprints['U2']
        fp.x, fp.y, fp.rotation = 115.34, 93.6, 90.0
        m = [r['overrun_mm'] for r in footprint_graphic_outline_census(mem)['rows']
             if r['owner_ref'] == 'U2']
        w = [r['overrun_mm'] for r in footprint_graphic_outline_census(
            parse_kicad_pcb(bad))['rows'] if r['owner_ref'] == 'U2']
        check('6. a part moved in memory grades as the written board does',
              m and w and abs(max(m) - max(w)) <= 1e-6, f'{m} vs {w}')
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
