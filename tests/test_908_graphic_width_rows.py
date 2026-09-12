#!/usr/bin/env python3
"""#908 follow-up: a footprint's GRAPHIC copper is not a track, so its stroke
width is not a track width.

A filled `fp_poly` on a copper layer parses as net-0 `graphic=True` segments
(#908), one per perimeter edge, carrying the poly's STROKE width. The copper
is the fill; the stroke is an outline. KiCad's `track_width` constraint grades
`PCB_TRACK` only and never a shape, so two consumers that walked
`pcb.segments` without excluding graphics manufactured findings out of a
SOT-89 tab outline:

  * `check_drc` graded the eight 0.1 mm perimeter segments of esp_prog's U2
    against the 0.15 mm fab floor -> 8 permanent `track-width` rows on a
    board with NO tracks (run 26: `board_score` counted them as `undersized 8`
    on every placement lap, and `blocking 0` was unreachable);
  * `fix_kicad_drc_settings.scan_board_minima` took `min(width)` over the same
    segments -> every chain step's writeback wrote `rules.min_track_width
    0.15 -> 0.1`, and `check_complete --authored-from` then read the board as
    UNSOUND ("track width 0.15 -> 0.1");
  * `fix_kicad_drc_settings._fab_floor_disclosure`'s census counted them too,
    so the FAB FLOOR RELAXED banner told its reader that N tracks sit under
    the original floor on a board whose only sub-floor copper is an outline.
    That one writes nothing, but it is the line a human reads to decide
    whether to re-route, and a wrong denominator there is a wrong decision.

All three now skip `seg.graphic`. This file pins each on the tracked fixture that
produced them, with a control board carrying ONE real 0.1 mm track so that a
checker which reports nothing at all cannot pass.

Run:
    python3 tests/test_908_graphic_width_rows.py
"""

import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from check_drc import run_drc
from fix_kicad_drc_settings import _fab_floor_disclosure, scan_board_minima
from kicad_parser import parse_kicad_pcb

RUN_ALL_FAST_OK = True

FIXTURE = os.path.join(ROOT_DIR, 'kicad_files', 'esp_prog.kicad_pcb')
FLOOR = 0.15
STROKE = 0.1

#: The control: the same SOT-89-style tab (a filled poly with a 0.1 stroke)
#: beside ONE real 0.1 mm track on a net. Only the track is a track.
CONTROL = '''(kicad_pcb
 (version 20221018)
 (net 0 "")
 (net 1 "/A")
 (footprint "L:P" (layer "F.Cu") (at 10 10)
   (property "Reference" "U1")
   (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "/A"))
   (fp_poly (pts (xy 0.4 -0.5) (xy 3.4 -0.5) (xy 3.4 0.5) (xy 0.4 0.5))
     (stroke (width 0.1) (type solid)) (fill yes)
     (layer "F.Cu") (uuid "poly1")))
 (segment (start 20 20) (end 24 20) (width 0.1) (layer "F.Cu") (net 1) (uuid "t1"))
)
'''


def _quiet_drc(path):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        return run_drc(path, clearance=0.25, min_track_width=FLOOR, quiet=True)


def _width_rows(violations):
    return [v for v in violations if v.get('type') == 'track-width']


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    # --- 0: the fixture is what this file says it is (anti-vacuity) --------
    pcb = parse_kicad_pcb(FIXTURE)
    graphics = [s for s in pcb.segments if getattr(s, 'graphic', False)]
    check('esp_prog carries exactly 8 segments, all graphic (U2 tab outline)',
          len(pcb.segments) == 8 and len(graphics) == 8,
          f'{len(pcb.segments)} segments, {len(graphics)} graphic')
    check('every one is the 0.1 mm stroke of U2',
          all(abs(s.width - STROKE) < 1e-9 for s in graphics)
          and {getattr(s, 'owner_ref', None) for s in graphics} == {'U2'},
          f'widths {sorted({s.width for s in graphics})}, '
          f'owners {sorted({getattr(s, "owner_ref", None) for s in graphics})}')
    check('the fixture has no real track at all (so a width row can only be a graphic)',
          not [s for s in pcb.segments if not getattr(s, 'graphic', False)])

    # --- 1: check_drc grades no graphic against the track-width floor -------
    rows = _width_rows(_quiet_drc(FIXTURE))
    check('check_drc reports 0 track-width rows on esp_prog at the 0.15 floor',
          rows == [], f'{len(rows)} rows: {rows[:2]}')

    # --- 2: scan_board_minima does not take a stroke as the board's track ---
    minima = scan_board_minima(FIXTURE)
    check('scan_board_minima reports no min_track_width on a track-less board',
          'min_track_width' not in minima, f'{minima}')

    # --- 3: the control -- a REAL 0.1 mm track beside the same kind of poly --
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False,
                                     encoding='utf-8') as fh:
        fh.write(CONTROL)
        control = fh.name
    try:
        cpcb = parse_kicad_pcb(control)
        cg = [s for s in cpcb.segments if getattr(s, 'graphic', False)]
        real = [s for s in cpcb.segments if not getattr(s, 'graphic', False)]
        check('the control parses as 4 graphic segments plus 1 real track',
              len(cg) == 4 and len(real) == 1 and abs(real[0].width - STROKE) < 1e-9,
              f'{len(cg)} graphic, {len(real)} real')
        crow = _width_rows(_quiet_drc(control))
        check('check_drc reports exactly 1 track-width row on the control (the track)',
              len(crow) == 1, f'{len(crow)} rows: {crow[:3]}')
        check('and that row is the track, not a poly edge',
              bool(crow) and tuple(round(c, 3) for c in crow[0].get('seg_loc', ())) ==
              (20.0, 20.0, 24.0, 20.0),
              f'{crow[0].get("seg_loc") if crow else None}')
        cmin = scan_board_minima(control)
        check('scan_board_minima reads the control\'s track width 0.1',
              abs(cmin.get('min_track_width', 0.0) - STROKE) < 1e-9, f'{cmin}')

        # --- 4: the FAB FLOOR RELAXED census counts tracks, not outlines ---
        # The control carries 1 real track and 4 poly edges, every one of them
        # 0.1 mm and so every one of them under a declared 0.15. The banner
        # must say `1 of 1`, never `5 of 5`: the denominator is the population
        # a reader would have to re-route.
        lines = _fab_floor_disclosure(
            control, {'min_track_width': FLOOR},
            {'board': {'design_settings': {'rules': {'min_track_width': STROKE}}}})
        tw = [ln for ln in lines if 'track width' in ln]
        check('the relaxation banner fires on the control at all',
              len(tw) == 1, f'{lines}')
        check('...and its census counts the 1 real track, not the 4 poly edges',
              bool(tw) and '1 of 1 object(s)' in tw[0], f'{tw[0] if tw else None}')

        # And on the fixture, whose only sub-floor copper IS an outline: there
        # is no track to count, so the census abstains rather than reporting
        # eight.
        flines = _fab_floor_disclosure(
            FIXTURE, {'min_track_width': FLOOR},
            {'board': {'design_settings': {'rules': {'min_track_width': STROKE}}}})
        ftw = [ln for ln in flines if 'track width' in ln]
        check('on esp_prog the census reports no object at all (nothing is a track)',
              len(ftw) == 1 and 'object(s)' not in ftw[0],
              f'{ftw[0] if ftw else flines}')
    finally:
        os.unlink(control)

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
