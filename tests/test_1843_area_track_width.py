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
0.1 mm track 0.3 mm, so only the room's track passes. kicad-cli 10.0.8 grades
the routed board at 0 violations with the board minimum still 0.2 mm: a
track_width rule replaces Board Setup's minimum for the tracks it binds.

Checks:
  1. ROOM_NET routes and OPEN_NET does not; ROOM_NET's track is 0.1 mm
     through the wall's gap, on less than half of its run through the room,
     and 0.25 mm elsewhere; check_drc is clean; the run names the rule; the
     project's `min_track_width` stays 0.2;
  2. `narrow_tracks` without a board to measure against splits a track at
     the room's edge and narrows all of it inside, leaves the rest at its
     width, and is stable when run on its own output;
  3. a net wider than the run's width is not narrowed, and its search
     margin grows by the room's narrower reserve.
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


def routed():
    print("1. the fixture, routed")
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as td:
        for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
            shutil.copy(evidence(os.path.join(FIX, 'room_width' + ext), 'the fixture'),
                        os.path.join(td, 'in' + ext))
        out = os.path.join(td, 'out.kicad_pcb')
        summary = os.path.join(td, 'out.json')
        r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'),
                        os.path.join(td, 'in.kicad_pcb'), out,
                        '--nets', 'ROOM_NET', 'OPEN_NET', '--layers', 'F.Cu',
                        '--escalation', 'off', '--json-out', summary],
                  accept=True, timeout=900)
        log = (r.stdout or '') + (r.stderr or '')
        with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
            doc = json.load(fh)
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
    routed()
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
