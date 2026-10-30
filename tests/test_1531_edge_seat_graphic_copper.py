"""Edge seat: an overhang never licenses graphic copper past the outline.

A declared pose (and the edge seat, whose legality is a declared pose's) may
leave the COURTYARD past the outline -- an edge part overhangs by design --
when every pad and drill hole stays on the board. The copper a footprint
DRAWS (#962: a tab, an antenna) was never asked on that path, and the
search's cached bound skipped it whenever the courtyard enclosed the copper,
which an estimated courtyard (#1300) does. So the edge seat put a part's
F.Cu tab past the outline, and the grade's oob exemption excused the part.

Pinned here on a synthetic board: one part with no courtyard, whose Fab
body (made unmeasurable by a pin-1 circle) reaches 4.5 mm past its pads and
carries a filled F.Cu tab out to 4.0 mm:
  1. a declared pose whose courtyard overhangs with the tab off the board is
     refused, with the tab's measurement, which equals the census the grade
     reads (`legality._graphic_copper_channel`) on the board written there;
  2. a pose whose courtyard overhangs with the tab still on the board is
     accepted, and the census lists nothing (no over-refusal);
  3. the edge seat finds no seat for a band that needs the tab off the
     board, and says why;
  4. the search measures the tab of a part whose courtyard is off the board;
  5. the grade's oob exemption does not excuse a declared edge part whose
     courtyard is inside its band but whose tab is past the outline.

    python3 tests/test_1531_edge_seat_graphic_copper.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
import pose_score                                             # noqa: E402
from placement import edge_seat, floorplan, legality, seeder  # noqa: E402

FAILS = []
REF = 'U1'
EDGE = 'north'

#: a 40 x 30 board; turned 180, U1's body and tab run NORTH of its pads:
#: the body to 4.5 mm, the tab to 4.0 mm
BOARD = '''(kicad_pcb
 (version 20241229)
 (generator "test")
 (general (thickness 1.6))
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (36 "B.SilkS" user)
  (37 "F.SilkS" user) (44 "Edge.Cuts" user) (46 "F.CrtYd" user)
  (49 "F.Fab" user))
 (net 0 "")
 (net 1 "/A")
 (net 2 "/B")
 (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type solid))
  (fill no) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "test:PART" (layer "F.Cu") (uuid "fp-u1") (at %s)
  (property "Reference" "U1" (at 0 -3 0) (layer "F.SilkS") (uuid "t1"))
  (fp_rect (start -2 -1.5) (end 2 4.5) (stroke (width 0.1) (type solid))
   (fill no) (layer "F.Fab") (uuid "b1"))
  (fp_circle (center -1.5 -1) (end -1.3 -1) (stroke (width 0.1)
   (type solid)) (fill no) (layer "F.Fab") (uuid "b2"))
%s  (fp_poly (pts (xy -1 2.5) (xy 1 2.5) (xy 1 4) (xy -1 4))
   (stroke (width 0) (type solid)) (fill solid) (layer "F.Cu") (uuid "g1"))
  (pad "1" smd rect (at -1 0) (size 1 1) (layers "F.Cu") (net 1 "/A")
   (uuid "p1"))
  (pad "2" smd rect (at 1 0) (size 1 1) (layers "F.Cu") (net 2 "/B")
   (uuid "p2")))
 (footprint "test:R" (layer "F.Cu") (uuid "fp-r1") (at 10 20)
  (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS") (uuid "t2"))
  (pad "1" smd rect (at -1 0) (size 1 1) (layers "F.Cu") (net 1 "/A")
   (uuid "p3"))
  (pad "2" smd rect (at 1 0) (size 1 1) (layers "F.Cu") (net 2 "/B")
   (uuid "p4")))
)
'''
#: the tab 1.5 mm past the north edge, the body 2.0 mm, pads 2.0 mm inside
OFF = (20.0, 2.5, 180.0)
#: the body 0.3 mm past, the tab 0.2 mm inside, pads 4.2 mm inside
ON = (20.0, 4.2, 180.0)


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **kw)


#: a drawn courtyard round body and pads, for the grade's exemption
COURTYARD = ('  (fp_rect (start -2.25 -1.75) (end 2.25 4.75) (stroke (width 0.05)'
             ' (type solid)) (fill no) (layer "F.CrtYd") (uuid "c1"))\n')


def _write(tmp, name, pose, courtyard=False):
    path = os.path.join(tmp, name)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(BOARD % (' '.join(f'{v:g}' for v in pose),
                          COURTYARD if courtyard else ''))
    return path


def _census(path):
    """The grade's graphic-copper reading on the board written at a pose."""
    pcb = _quiet(parse_kicad_pcb, path)
    ch = legality._graphic_copper_channel(pcb, 0.0)
    return dict((r, a) for r, a in ch['oob_graphic_copper_refs'])


def _intent():
    return floorplan.intent_from_dict({
        'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
        'edge_connectors': [{'ref': REF, 'edge': EDGE, 'class': 'connector',
                             'overhang_mm': {'min': 1.5, 'max': 2.5}}]}, '')


def main():
    tmp = tempfile.mkdtemp(prefix='t_gfx_seat_')
    try:
        base = _write(tmp, 'base.kicad_pcb', (20.0, 15.0, 0.0))
        pcb = _quiet(parse_kicad_pcb, base)
        st = _quiet(pose_score.make_state, pcb, base, estimate_courtyards=True)
        check('the part is seated on an estimate that encloses its tab',
              REF in st.courtyard_estimates
              and seeder._graphic_copper_overhang(st, REF) == 0.0,
              seeder._graphic_copper_overhang(st, REF))

        # 1. courtyard and tab past the outline
        how, reasons, _c = seeder._fixed_pose_check(st, REF, OFF, {})
        census = _census(_write(tmp, 'off.kicad_pcb', OFF))
        check('a declared pose with the tab off the board is refused',
              how is None and any('graphic copper' in r for r in reasons),
              (how, reasons))
        check('...the census the grade reads lists the part there',
              REF in census, census)
        got = seeder._graphic_copper_off_board(st, REF, *OFF)
        check('...and the seat measures what the census measures',
              REF in census and abs(got - census[REF]) < 1e-3,
              (got, census.get(REF)))

        # 2. courtyard past the outline, tab on the board
        how, reasons, _c = seeder._fixed_pose_check(st, REF, ON, {})
        census = _census(_write(tmp, 'on.kicad_pcb', ON))
        check('a courtyard overhang with the tab on the board is accepted',
              how == 'overhang', (how, reasons))
        check('...and the census lists nothing there', REF not in census,
              census)

        # 3. the edge seat
        it = _intent()
        clr, edge = floorplan.resolved_floors(base, None, None)
        sst = _quiet(edge_seat.seat_state, pcb, base, it, {}, clr, edge)
        out, _notes, unseated = _quiet(edge_seat.seats, sst, pcb, base, it,
                                       set(pcb.footprints))
        check('the edge seat finds no seat that puts the tab off the board',
              REF not in out, out.get(REF))
        check('...and says the graphic copper stopped it',
              'graphic copper' in (unseated.get(REF) or ''),
              unseated.get(REF))

        # 4. the search, for a part whose courtyard is off the board
        check('the search measures the tab of a part off the board',
              st._graphic_copper_past(REF, *OFF) > 1.0,
              st._graphic_copper_past(REF, *OFF))

        # 5. the grade's exemption
        path = _write(tmp, 'graded.kicad_pcb', OFF, courtyard=True)
        ppcb = _quiet(parse_kicad_pcb, path)
        it2 = floorplan.intent_from_dict({
            'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'edge_connectors': [{'ref': REF, 'edge': EDGE,
                                 'class': 'connector',
                                 'overhang_mm': {'min': 1.5,
                                                 'max': 2.5}}]}, path)
        (ctx, *_r) = _quiet(floorplan._grade_ctx, it2, ppcb, path)
        check('the grade does not exempt a part whose tab is off the board',
              REF not in ctx.oob_exempt(), ctx.oob_exempt())
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
