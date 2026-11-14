#!/usr/bin/env python3
"""#1843: route.py necks a track to the width a rule area allows.

A `.kicad_dru` `track_width` rule conditioned on `A.insideArea('ROOM')` was
graded by position (check_drc) but not routed to: the keep-outs in the room
reserved the run's track width and every track was drawn at it, so a gap that
only the room's narrower track fits stayed closed. Now the room's keep-outs
reserve the room's width, and a track is drawn at it on the stretches where
its own width does not fit -- not for its whole run through the room.

The fixture (tests/fixtures/1843/room_width) is synthetic, the walls and nets
of tests/test_1843_area_clearance_routing.py: Default class clearance 0.2 mm
and track 0.25 mm, Board Setup `min_track_width` 0.2 mm, gaps 0.4 mm, and the
rule `clearance (min 0.1mm)` + `track_width (min 0.1mm) (opt 0.1mm)` in ROOM.
Through the gap a 0.25 mm track needs 0.45 mm at the room's clearance and a
0.1 mm track 0.3 mm, so only the room's track passes. kicad-cli 10.0.6 grades
the routed board at 0 violations with the board minimum still 0.2 mm: a
track_width rule replaces Board Setup's minimum for the tracks it binds.

Checks:
  1. ROOM_NET routes and OPEN_NET does not; ROOM_NET's track is 0.1 mm
     through the wall's gap, on less than half of its run through the room,
     and 0.25 mm elsewhere; check_drc is clean; the run names the rule; the
     project's `min_track_width` stays 0.2; the neck is disclosed (console
     and JSON_SUMMARY `area_necks`) under --escalation off and is no
     `design_rules` narrowing; check_complete's --authored-from grade holds
     the neck at the room's rule, not at Board Setup's minimum;
  2. `narrow_tracks` without a board to measure against splits a track at
     the room's edge and narrows all of it inside, leaves the rest at its
     width -- a track that starts or ends inside the room too, in either
     direction --, and is stable when run on its own output; a call on a
     SUBSET of a route's list leaves that list whole;
  3. a net wider than the run's width is not narrowed, and its search
     margin grows by the room's narrower reserve;
  4. the board-minimum scan: a track a room's rule binds is not a statement
     about Board Setup's minimum, by KiCad's reading (its copper meets the
     room); a room rule that cannot be read is an error, never a quiet "no
     rule", and the scan then leaves the track minimum unmeasured;
  5. the gap widened to 0.47 mm: a 0.25 mm track keeps the room's 0.1 mm
     clearance there with 0.01 mm to spare, so it ships at full width and
     nothing is necked.
"""
import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1843')
ROOM = (8.0, 4.0, 12.0, 8.0)
WALL = (9.5, 10.5)      # the x extent of the wall of pads in the room
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def route_fixture(td, gap_shift=0.0):
    """Route the fixture in ``td`` (its wall pads moved ``gap_shift`` mm
    apart each); (summary, log, routed board, input board)."""
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        shutil.copy(evidence(os.path.join(FIX, 'room_width' + ext), 'the fixture'),
                    os.path.join(td, 'in' + ext))
    board = os.path.join(td, 'in.kicad_pcb')
    if gap_shift:
        with open(board, encoding='utf-8') as fh:
            text = fh.read()
        moved = (text.replace('(at 0 3.15)', f'(at 0 {3.15 - gap_shift:g})', 1)
                 .replace('(at 0 8.85)', f'(at 0 {8.85 + gap_shift:g})', 1))
        ok(moved != text, f"the wall's gap is widened by {2 * gap_shift:g} mm")
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write(moved)
    out = os.path.join(td, 'out.kicad_pcb')
    summary = os.path.join(td, 'out.json')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out,
                    '--nets', 'ROOM_NET', 'OPEN_NET', '--layers', 'F.Cu',
                    '--escalation', 'off', '--json-out', summary],
              accept=True, timeout=900)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return doc, (r.stdout or '') + (r.stderr or ''), out, board


def routed():
    print("1. the fixture, routed")
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as td:
        doc, log, out, board = route_fixture(td)
        ok(doc.get('routed_single') == ['ROOM_NET'] and doc.get('failed_single') == ['OPEN_NET'],
           f"the net in the room routes, the other does not: "
           f"{doc.get('routed_single')} / {doc.get('failed_single')}")
        pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
        spans = sorted((round(min(s.start_x, s.end_x), 3), round(max(s.start_x, s.end_x), 3),
                        s.width) for s in pcb.segments)
        necks = [(a, b) for a, b, w in spans if abs(w - 0.1) < 1e-9]
        ok(len(necks) == 1 and necks[0][0] <= WALL[0] and necks[0][1] >= WALL[1],
           f"its track is 0.1 mm through the wall's gap: {necks}")
        ok(necks and necks[0][0] > ROOM[0] and necks[0][1] < ROOM[2]
           and necks[0][1] - necks[0][0] < (ROOM[2] - ROOM[0]) / 2,
           f"on less than half of its run through the room: "
           f"{necks and round(necks[0][1] - necks[0][0], 3)} of {ROOM[2] - ROOM[0]} mm")
        ok(all(abs(w - 0.25) < 1e-9 for a, b, w in spans if (a, b) not in necks)
           and len(spans) > len(necks),
           f"and 0.25 mm elsewhere: {spans}")
        d = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                        '--clearance-margin-mm', '0.0005'], accept=True)
        ok('NO DRC VIOLATIONS FOUND' in (d.stdout or ''), "check_drc is clean")
        ok("track_width 0.1 mm: a track no wider than the run's may neck to it there"
           in log, "the run names the rule as routed to")
        with open(os.path.join(td, 'out.kicad_pro'), encoding='utf-8') as fh:
            rules = json.load(fh)['board']['design_settings']['rules']
        ok(rules.get('min_track_width') == 0.2,
           f"the board's min_track_width is kept: {rules.get('min_track_width')}")
        rows = doc.get('area_necks') or []
        ok(len(rows) == 1 and rows[0].get('net') == 'ROOM_NET'
           and rows[0].get('area') == 'ROOM' and rows[0].get('width_mm') == 0.1
           and necks and abs(rows[0].get('length_mm', 0) - (necks[0][1] - necks[0][0])) < 1e-3,
           f"the neck is disclosed in JSON_SUMMARY area_necks: {rows}")
        ok('Rule-area necks: ROOM_NET' in log, "and on the console")
        narrowed = [x for x in ((doc.get('design_rules') or {}).get('narrowed') or [])
                    if x.get('kind') == 'track_width']
        ok(not narrowed, f"and it is the board's rule, not a narrowing: {narrowed}")
        from fix_kicad_drc_settings import size_values_below_authored
        got = size_values_below_authored(out, {'min_track_width': 0.2}, board)
        tw = got.get('min_track_width') or {}
        ok(tw.get('below') == [] and tw.get('held_by_rule', 0) >= 1
           and tw.get('rules') == ['room'],
           f"the authored grade holds the neck at the room's rule: {tw}")
        cj = os.path.join(td, 'complete.json')
        check(PY + [os.path.join(REPO, 'check_complete.py'), out, '--authored-from', board,
                    '--json', cj], refuse='INCOMPLETE', code=4)   # OPEN_NET cannot route
        with open(evidence(cj, "check_complete's report"), encoding='utf-8') as fh:
            ff = json.load(fh).get('fab_floors') or {}
        kept = [r for r in ff.get('measured_kept') or [] if r.get('key') == 'min_track_width']
        elsewhere = [r for k in ('relaxed', 'vacuous', 'reference_too')
                     for r in ff.get(k) or [] if r.get('key') == 'min_track_width']
        ok(len(kept) == 1 and kept[0].get('rules') == ['room'] and kept[0].get('on_board') == 0.1
           and not elsewhere,
           f"and check_complete reads the track floor as kept by the room's rule: "
           f"{kept} / {elsewhere}")


def wider_gap():
    print("5. the wall's gap at 0.47 mm")
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as td:
        doc, log, out, _board = route_fixture(td, gap_shift=0.035)
        pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
        widths = sorted({s.width for s in pcb.segments})
        ok(doc.get('routed_single') == ['ROOM_NET'] and widths == [0.25],
           f"ROOM_NET ships at its full width: {widths}")
        ok(not doc.get('area_necks'), f"and nothing is necked: {doc.get('area_necks')}")
        d = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                        '--clearance-margin-mm', '0.0005'], accept=True)
        ok('NO DRC VIOLATIONS FOUND' in (d.stdout or ''), "check_drc is clean")


def scan():
    print("4. the board-minimum scan")
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    import area_clearance
    import design_rules
    import fix_kicad_drc_settings as fk
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(os.path.join(FIX, 'room_width.kicad_pcb'))
    held = area_clearance.room_width_test(pcb)
    ok(held is not None and held(9.0, 6.0, 11.0, 6.0, 'F.Cu', 0.1),
       "a 0.1 mm track in the room is graded at the room's rule")
    ok(held is not None and held(7.0, 6.0, 7.9, 6.0, 'F.Cu', 0.25)
       and not held(7.0, 6.0, 7.85, 6.0, 'F.Cu', 0.25),
       "a track whose copper reaches into the room is too, one that stops at its edge is not")
    real = design_rules.AreaIndex

    class Broken(real):
        def __init__(self, *a, **k):
            raise RuntimeError("the area index failed")
    design_rules.AreaIndex = Broken
    try:
        try:
            area_clearance.room_width_test(pcb)
            raised = False
        except RuntimeError:
            raised = True
        ok(raised, "a room rule that cannot be read raises, it is not read as no rule")
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            widths = fk.board_size_values(pcb)['min_track_width']
        ok(widths == [] and 'ERROR' in err.getvalue(),
           f"and the scan says so and measures no track: {widths} / {err.getvalue().strip()}")
    finally:
        design_rules.AreaIndex = real


def _config(area, power=None):
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(track_width=0.25, clearance=0.2, grid_step=0.05,
                          layers=['F.Cu', 'B.Cu'])
    cfg.area_clearance = area
    if power:
        cfg.power_net_widths = dict(power)
    return cfg


def narrowing():
    print("2. narrow_tracks")
    from shapely.geometry import box
    from area_clearance import AreaClearance, Region, narrow_tracks
    from kicad_parser import Segment
    area = AreaClearance([Region('r', 'ROOM', None, box(*ROOM), frozenset(['F.Cu']),
                                 0.0, width=0.1)], ['F.Cu', 'B.Cu'])
    cfg = _config(area)

    def seg(layer='F.Cu'):
        return Segment(start_x=6.0, start_y=6.0, end_x=14.0, end_y=6.0, width=0.25,
                       layer=layer, net_id=1)
    segs = [seg()]
    n = narrow_tracks(segs, cfg, 1)
    got = [(round(s.start_x, 3), round(s.end_x, 3), s.width) for s in segs]
    ok(n == 1 and got == [(6.0, 7.925, 0.25), (7.925, 12.075, 0.1), (12.075, 14.0, 0.25)],
       f"a crossing track is cut 1.5 grid steps outside the room: {got}")
    again = narrow_tracks(segs, cfg, 1)
    ok(again == 0 and [(round(s.start_x, 3), round(s.end_x, 3), s.width) for s in segs] == got,
       "and a second pass changes nothing")
    other = [seg('B.Cu')]
    ok(narrow_tracks(other, cfg, 1) == 0 and other[0].width == 0.25,
       "a track on a layer the room is not drawn on keeps its width")
    for a_, b_, want in (((10.0, 6.0), (16.0, 6.0), [(10.0, 12.075, 0.1), (12.075, 16.0, 0.25)]),
                         ((16.0, 6.0), (10.0, 6.0), [(16.0, 12.075, 0.25), (12.075, 10.0, 0.1)])):
        segs = [Segment(start_x=a_[0], start_y=a_[1], end_x=b_[0], end_y=b_[1], width=0.25,
                        layer='F.Cu', net_id=1)]
        narrow_tracks(segs, cfg, 1)
        got = [(round(s.start_x, 3), round(s.end_x, 3), s.width) for s in segs]
        ok(got == want, f"a track from {a_} to {b_} is narrow in the room only: {got}")
    from single_ended_routing import _neck_terminal_grazes
    from types import SimpleNamespace as NS
    whole = [seg()]
    _neck_terminal_grazes([whole[0]], [], NS(segments=[], vias=[], pads_by_net={},
                                             footprints={}, nets={}), 1, cfg, floor=0.1)
    got = [(round(s.start_x, 3), round(s.end_x, 3)) for s in whole]
    ok(got == [(6.0, 14.0)], f"a call on a subset of a route's list leaves the list whole: {got}")

    print("3. a net wider than the run's width")
    wide = _config(area, power={7: 0.4})
    segs = [seg()]
    segs[0].width = 0.4
    ok(narrow_tracks(segs, wide, 7) == 0 and len(segs) == 1 and segs[0].width == 0.4,
       "is not narrowed in the room")
    plain = _config(None, power={7: 0.4})
    m_room = wide.track_margins_for_net(7)[0]
    m_plain = plain.track_margins_for_net(7)[0]
    ok(m_room > m_plain > 0, f"and its search margin covers the room's reserve: "
                             f"{m_room} > {m_plain} cells")
    ok(wide.track_margins_for_net(1)[0] == 0.0,
       "a net at the run's width has no margin (it is narrowed instead)")


def main():
    narrowing()
    scan()
    routed()
    wider_gap()
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
