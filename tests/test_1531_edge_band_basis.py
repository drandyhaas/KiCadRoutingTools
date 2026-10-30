"""#1531: an edge part's overhang band is read on ONE basis, seat and grade.

A declared `edge_connectors[]` band falls back to the part's courtyard when
its drawn body cannot be measured (`legacy_occupancy@margin=0`). For a part
that draws no courtyard, place_simple's edge seat read that courtyard on the
ESTIMATE it seats the part on, while `rule_edge_connector` read the PAD BOX:
the seat put the estimate exactly at a positive `overhang_mm.min`, and the
grade then read 0.00 there and failed it -- on the pad box a positive minimum
asks the pads to leave the board, so no pose could meet it. In place_simple's
#1300 fallback the seat itself read the pad box and found no room.

Pinned here on a synthetic board: one part with no courtyard, whose Fab body
reaches 4 mm past its pads and carries a pin-1 circle (so the body is not
measured and the band falls back to the courtyard), declared on the north
edge with a band of 2-3 mm:
  1. the edge seat seats it -- on the estimating state AND on the fallback
     state that seats it on its pad box -- at a pose the grade reads inside
     the band;
  2. `rule_edge_connector` grades the board written at that pose clean, and
     reads the band in the same currency (`legacy_occupancy@margin=0`);
  3. end to end: place_simple seats it and check_floorplan passes the board;
  4. the emitter's observed band grades the board it was read off clean.

    python3 tests/test_1531_edge_band_basis.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
import pose_score                                             # noqa: E402
from placement import edge_seat, floorplan                    # noqa: E402
from placement.connector_geometry import (                    # noqa: E402
    courtyard_overhang_past, geometry_for)
from run_utils import check as run_check, tool                # noqa: E402

FAILS = []
BAND = {'min': 2.0, 'max': 3.0}
EDGE = 'north'
REF = 'U1'

#: a 40 x 30 board; U1's pads sit at y = 0 in its own frame, its Fab body
#: runs from y = -1.5 to y = 4.5 -- 4 mm past the pads -- with a pin-1
#: circle, so `connector_geometry` measures no body and `body` estimates
#: a courtyard from the Fab drawing
BOARD = '''(kicad_pcb
 (version 20241229)
 (generator "test")
 (general (thickness 1.6))
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (36 "B.SilkS" user)
  (37 "F.SilkS" user) (44 "Edge.Cuts" user) (49 "F.Fab" user))
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
 (footprint "test:R" (layer "F.Cu") (uuid "fp-r2") (at 30 20)
  (property "Reference" "R2" (at 0 -2 0) (layer "F.SilkS") (uuid "t3"))
  (pad "1" smd rect (at -1 0) (size 1 1) (layers "F.Cu") (net 1 "/A")
   (uuid "p5"))
  (pad "2" smd rect (at 1 0) (size 1 1) (layers "F.Cu") (net 2 "/B")
   (uuid "p6")))
)
'''


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **kw)


def _write(path, pose='20 15 0'):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(BOARD % pose)
    return path


def _intent_doc(base=None):
    doc = dict(base or {'schema': 1, 'kind': 'floorplan-intent',
                        'units': 'mm'})
    doc['edge_connectors'] = [{
        'ref': REF, 'edge': EDGE, 'class': 'connector',
        'overhang_mm': dict(BAND),
        'why': 'its body reaches past the edge by design'}]
    return doc


def _band_read(path, pose):
    """The grade's band reading for REF at `pose` (`zone_parts`)."""
    pcb = _quiet(parse_kicad_pcb, path)
    st = _quiet(pose_score.make_state, pcb, path)
    zp = floorplan.zone_parts(st, pcb, path)[REF]
    return courtyard_overhang_past(geometry_for(st, pcb, path),
                                   zp.rect(*pose), EDGE)[0]


def test_fixture(tmp):
    path = _write(os.path.join(tmp, 'fixture.kicad_pcb'))
    pcb = _quiet(parse_kicad_pcb, path)
    st = _quiet(pose_score.make_state, pcb, path)
    est = _quiet(pose_score.make_state, pcb, path, estimate_courtyards=True)
    geo = geometry_for(st, pcb, path)
    check('the part draws no courtyard and is seated on an estimate',
          REF in est.courtyard_estimates, sorted(est.courtyard_estimates))
    check('...its body is not measured, so the band reads the courtyard',
          not geo.measure(REF, EDGE)['body_measured'],
          geo.measure(REF, EDGE))
    a, b = est.parts[REF].rect(), st.parts[REF].rect()
    check('...and the estimate reaches 4 mm past the pad box',
          abs((a[3] - b[3]) - 4.0) < 0.1, (a, b))


def test_seat_and_grade(tmp):
    path = _write(os.path.join(tmp, 'fixture.kicad_pcb'))
    pcb = _quiet(parse_kicad_pcb, path)
    it = floorplan.intent_from_dict(_intent_doc(), '')
    clr, edge = floorplan.resolved_floors(path, None, None)
    for label, skip in (('estimating', ()), ('fallback', (REF,))):
        st = _quiet(edge_seat.seat_state, pcb, path, it, {}, clr, edge,
                    estimate_skip=skip)
        out, _notes, unseated = _quiet(edge_seat.seats, st, pcb, path, it,
                                       set(pcb.footprints))
        check(f'the {label} state seats the part on its band', REF in out,
              unseated.get(REF))
        if REF not in out:
            continue
        p = st.parts[REF]
        pose = (p.x, p.y, p.rot % 360)
        read = _band_read(path, pose)
        check(f'...where the grade reads the {label} seat inside the band',
              BAND['min'] - 1e-6 <= read <= BAND['max'] + 1e-6, read)
        # the rule itself, on the board written at that pose
        placed = _write(os.path.join(tmp, f'{label}.kicad_pcb'),
                        f'{pose[0]:.6f} {pose[1]:.6f} {pose[2]:g}')
        ppcb = _quiet(parse_kicad_pcb, placed)
        res = _quiet(floorplan.grade, floorplan.intent_from_dict(
            _intent_doc(), placed), ppcb, placed)
        bad = [v.message for v in res.violations
               if v.rule == 'edge_connector' and v.severity == 'error']
        check(f'rule_edge_connector grades the {label} seat clean', not bad,
              bad)
        ev = [e for e in res.edge_connector_evidence if e['ref'] == REF]
        check('...reading the band on the courtyard',
              ev and str(ev[0].get('overhang_basis', '')).startswith(
                  'legacy_occupancy'), ev)


def test_place_then_grade(tmp):
    board = _write(os.path.join(tmp, 'b.kicad_pcb'))
    ipath = os.path.join(tmp, 'intent.json')
    with open(ipath, 'w') as fh:
        json.dump(_intent_doc(), fh)
    groups = os.path.join(tmp, 'groups.json')
    with open(groups, 'w') as fh:
        json.dump({'schema': 1, 'groups': [{
            'name': 'all', 'role': 'core', 'refs': [REF, 'R1', 'R2'],
            'why': 'one group'}]}, fh)
    out = os.path.join(tmp, 's0.kicad_pcb')
    r = run_check([sys.executable, tool('place_simple.py'), board, out,
                   '--groups', groups, '--intent', ipath, '--seed', '0'],
                  accept=True)
    check(f'place_simple seats the part on the {EDGE} edge',
          f'edge connector {REF} on the {EDGE} edge' in r.stdout,
          r.stdout[-800:])
    g = subprocess.run([sys.executable, tool('check_floorplan.py'), out,
                        '--intent', ipath], capture_output=True, text=True,
                       timeout=900)
    text = g.stdout + g.stderr
    bad = [ln.strip() for ln in text.splitlines()
           if f'edge_connector: {REF} ' in ln and '[ERROR]' in ln]
    check('check_floorplan grades the seated band clean', not bad, bad)


def test_emitted_band_grades_clean(tmp):
    """With its pads at the edge and its body 3.6 mm past it, the emitter's
    observed band covers the overhang the grade reads, so the board grades
    clean against what was read off it."""
    # turned 180, the body runs north of the pads; pads 0.4 mm inside
    placed = _write(os.path.join(tmp, 'emit.kicad_pcb'), '20 0.9 180')
    read = _band_read(placed, (20.0, 0.9, 180.0))
    ppcb = _quiet(parse_kicad_pcb, placed)
    doc = _quiet(floorplan.emit_intent, ppcb, placed)
    rows = [c for c in doc.get('edge_connectors') or () if c['ref'] == REF]
    check('the emitter observes the part at the edge', bool(rows),
          doc.get('edge_connectors'))
    check('...with a band covering the overhang the grade reads',
          rows and float(rows[0]['overhang_mm']['max']) >= read - 1e-6,
          (rows, read))
    res = _quiet(floorplan.grade, floorplan.intent_from_dict(doc, placed),
                 ppcb, placed)
    bad = [v.message for v in res.violations
           if v.rule == 'edge_connector' and v.severity == 'error']
    check('the emitted band grades the board it was read off clean', not bad,
          bad)


def main():
    tmp = tempfile.mkdtemp(prefix='t1531_')
    try:
        test_fixture(tmp)
        test_seat_and_grade(tmp)
        test_emitted_band_grades_clean(tmp)
        test_place_then_grade(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
