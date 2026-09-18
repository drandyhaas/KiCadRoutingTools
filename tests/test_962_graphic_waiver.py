#!/usr/bin/env python3
"""#962: check_drc grades footprint GRAPHIC copper past the outline, and scopes the #908 waiver.

`accepted: immutable-graphic` used to waive every footprint graphic at the edge,
with no off-board guard and no owner check. `place_pose set U2 115.34 93.6
--rot 90` put esp_prog U2's F.Cu tab 1.11 mm past the outline, and check_drc
read "NO DRC VIOLATIONS FOUND" (three accepted rows).

Invariants, on staged copies of real boards and on synthetic fixtures:

1. **The three U2 poses** from the issue comment, with both the comment's flags
   and the placement skill's plain command:
   - 115.34 gives exactly ONE `graphic-off-board` row (one per shape, not per
     segment): overrun 1.11 +- 0.005 mm, owner_state `movable`, CLI exit 1;
   - the original pose and 116.70 give no graphic row and no accepted row.
2. **watchy AE1**'s inherited grazes (inside the outline) stay ACCEPTED as
   `immutable-graphic`: origin `unverified` without --baseline and `inherited`
   with it. They are never counted.
3. **AE1 moved OFF the board** is a `graphic-off-board` violation.
4. **AE1 moved to graze harder** (still inside) with --baseline is a
   `graphic-board-edge` violation with origin `placement`: the waiver cannot
   erase a placement-created change.
5. **A LOCKED U2 at 115.34** is still flagged: a lock is not a waiver, because
   placement stamps locks itself.
6. **Waivers:** a footprint that owns the board outline is waived, and so is a
   board-level `gr_poly` off the board.
7. **The grade runs where others are switched off:** with copper_edge_clearance
   severity `ignore`, with edge clearance 0, and with a `--nets` filter.
8. **Ring ownership (#628):** a part's copper circle on its OWN milled window
   is not flagged; the same copper on a real cutout it does not own is.
9. **Unmodelled copper** (orangecrab's pad-less G*** logos) is listed as
   unmeasured.

Run:
    python3 tests/test_962_graphic_waiver.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from copy_board import copy_board  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from check_drc import run_drc, footprint_graphic_outline_census  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402
from placement.seeder import stamp_locked  # noqa: E402
from run_utils import check as run_check  # noqa: E402

FAILS = []
ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
WATCHY = os.path.join(ROOT, 'kicad_files', 'watchy.kicad_pcb')


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def graphic_rows(viol, typ=None):
    return [v for v in viol if v.get('type') in ((typ,) if typ else
                                                 ('graphic-off-board', 'graphic-board-edge'))
            and not v.get('accepted')]


def accepted_graphic(viol):
    return [v for v in viol if v.get('accepted') == 'immutable-graphic']


def stage(work, src, name, placements, locked=()):
    out = os.path.join(work, name + '.kicad_pcb')
    copy_board(src, out)
    if placements:
        write_placed_output(out, out, placements)
    if locked:
        stamp_locked(out, set(locked))
    return out


def drc(board, **kw):
    kw.setdefault('clearance', 0.25)
    kw.setdefault('clearance_margin', 0.0)
    return run_drc(board, quiet=True, print_summary=False, **kw)


def main():
    work = tempfile.mkdtemp(prefix='krt962w_')
    try:
        U2 = lambda x, rot=90: [dict(reference='U2', new_x=x, new_y=93.6, new_rotation=rot)]
        orig = stage(work, ESP, 'orig', [])
        bad = stage(work, ESP, 'bad', U2(115.34))
        ok = stage(work, ESP, 'ok', U2(116.70))

        # 1 -- the comment's own flags
        flags = dict(check_pad_edge=True, board_edge_clearance=0.25)
        vb = drc(bad, **flags)
        g = graphic_rows(vb, 'graphic-off-board')
        check('1. 115.34: exactly ONE graphic-off-board row (per shape)', len(g) == 1,
              str([(v['item1'], v['overrun_mm']) for v in g]))
        if g:
            check('1. ... overrun 1.11 +- 0.005 mm, owner U2, state movable',
                  abs(g[0]['overrun_mm'] - 1.11) <= 0.005 and g[0]['owner_ref'] == 'U2'
                  and g[0]['owner_state'] == 'movable', str(g[0]))
            check('1. ... carries a location (seg_loc) for --render / kicad_drc_compare',
                  len(g[0].get('seg_loc') or ()) == 4)
        check('1. 115.34: none of its segments is ALSO an accepted row (no double publish)',
              not [v for v in accepted_graphic(vb) if 'U2' in str(v.get('item1'))])
        for nm, bd in (('original', orig), ('116.70', ok)):
            vv = drc(bd, **flags)
            check('1. %s: no graphic row and no accepted graphic row' % nm,
                  not graphic_rows(vv) and not accepted_graphic(vv),
                  str([(v['type'], v.get('item1')) for v in graphic_rows(vv) + accepted_graphic(vv)]))
        # the skill's plain command, and the CLI exit code
        r = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                       bad, '--clearance', '0.25', '--clearance-margin', '0',
                       '--json', os.path.join(work, 'bad.json')], code=1)
        d = json.load(open(os.path.join(work, 'bad.json'), encoding='utf-8'))
        check('1. the skill\'s plain command: exit 1 and graphic-off-board in by_type',
              d.get('by_type', {}).get('graphic-off-board') == 1, str(d.get('by_type')))

        # 2 -- watchy AE1: inherited grazes stay accepted
        vw = drc(WATCHY)
        acc = accepted_graphic(vw)
        check('2. watchy AE1 grazes are ACCEPTED, never counted',
              len(acc) == 9 and not graphic_rows(vw),
              f'{len(acc)} accepted, counted {len(graphic_rows(vw))}')
        check('2. ... origin unverified without --baseline',
              {v.get('origin') for v in acc} == {'unverified'})
        vwb = drc(WATCHY, baseline=WATCHY)
        check('2. ... origin inherited WITH --baseline, still accepted',
              {v.get('origin') for v in accepted_graphic(vwb)} == {'inherited'}
              and not graphic_rows(vwb))

        # 3 / 4 -- AE1 moved: off the board, and into a harder graze
        wp = parse_kicad_pcb(WATCHY)
        ae = wp.footprints['AE1']
        base_ov = max(r['overrun_mm'] for r in footprint_graphic_outline_census(wp)['rows']
                      if r['owner_ref'] == 'AE1')
        worst = max((r for r in footprint_graphic_outline_census(wp)['rows']
                     if r['owner_ref'] == 'AE1'), key=lambda r: r['overrun_mm'])
        # step toward the nearest edge: the direction from the part to its worst point
        wx, wy = worst['point']
        L = ((wx - ae.x) ** 2 + (wy - ae.y) ** 2) ** 0.5 or 1.0
        ux, uy = (wx - ae.x) / L, (wy - ae.y) / L
        off = stage(work, WATCHY, 'ae1_off',
                    [dict(reference='AE1', new_x=ae.x + ux * 2.0, new_y=ae.y + uy * 2.0,
                          new_rotation=ae.rotation)])
        vo = drc(off)
        check('3. AE1 moved 2 mm toward its edge: graphic-off-board',
              any(v['owner_ref'] == 'AE1' for v in graphic_rows(vo, 'graphic-off-board')))
        step = min(0.06, max(0.01, -base_ov / 2.0))
        graze = stage(work, WATCHY, 'ae1_graze',
                      [dict(reference='AE1', new_x=ae.x + ux * step, new_y=ae.y + uy * step,
                            new_rotation=ae.rotation)])
        vg = drc(graze, baseline=WATCHY, board_edge_clearance=0.3)
        pl = graphic_rows(vg, 'graphic-board-edge')
        check('4. AE1 nudged into a harder graze, with --baseline: graphic-board-edge '
              'origin placement', bool(pl) and all(v.get('origin') == 'placement' for v in pl),
              str([(v['type'], v.get('origin')) for v in graphic_rows(vg)]))
        check('4. ... and without --baseline the same graze is accepted as unverified',
              not graphic_rows(drc(graze, board_edge_clearance=0.3), 'graphic-board-edge'))

        # 5 -- a lock is not a waiver
        locked = stage(work, ESP, 'locked', U2(115.34), locked=('U2',))
        vl = drc(locked)
        gl = graphic_rows(vl, 'graphic-off-board')
        check('5. a LOCKED U2 at 115.34 is still graphic-off-board (state locked)',
              len(gl) == 1 and gl[0]['owner_state'] == 'locked', str(gl))

        # 6 / 7 / 8 -- synthetic fixtures
        def board(body, name, pro=None):
            p = os.path.join(work, name + '.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write('(kicad_pcb (version 20240108) (generator "t")\n'
                         ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                         ' (setup)\n (net 0 "")\n (net 1 "/A")\n'
                         ' (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type solid)) '
                         '(fill no) (layer "Edge.Cuts"))\n%s\n)' % body)
            if pro is not None:
                with open(p[:-10] + '.kicad_pro', 'w', encoding='utf-8') as fh:
                    json.dump(pro, fh)
            return p

        tab_off = ('(fp_poly (pts (xy 0 0) (xy 3 0) (xy 3 1) (xy 0 1)) (stroke (width 0.1) '
                   '(type solid)) (fill yes) (layer "F.Cu") (uuid "t1"))')
        pad = '(pad "1" smd rect (at 0 0.5) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A"))'
        # the tab starts inside and ends 1.5 mm past the east edge (x=40)
        fp_off = ('(footprint "L:P" (layer "F.Cu") (at 38.5 10) (property "Reference" "U9")\n'
                  ' %s\n %s)' % (pad, tab_off))
        v = drc(board(fp_off, 'movable_off'))
        check('6. control: a movable part\'s tab 1.5 mm past the edge is flagged',
              any(r['owner_ref'] == 'U9' for r in graphic_rows(v, 'graphic-off-board')))
        owner = ('(footprint "L:EDGE" (layer "F.Cu") (at 38.5 10) (property "Reference" "J9")\n'
                 ' %s\n %s\n (fp_line (start 1.5 -3) (end 1.5 3) (stroke (width 0.1) '
                 '(type solid)) (layer "Edge.Cuts")))' % (pad, tab_off))
        po = board(owner, 'owner_off')
        pj = parse_kicad_pcb(po)
        v = drc(po)
        check('6. a footprint that OWNS the board outline is waived',
              pj.footprints['J9'].owns_board_outline
              and not any(r['owner_ref'] == 'J9' for r in graphic_rows(v)),
              f"owns_board_outline={pj.footprints['J9'].owns_board_outline}")
        gr = ('(gr_poly (pts (xy 39 5) (xy 42 5) (xy 42 6) (xy 39 6)) (stroke (width 0.1) '
              '(type solid)) (fill yes) (layer "F.Cu"))')
        v = drc(board(gr, 'boardlevel_off'))
        check('6. a board-level gr_poly off the board is waived (board art)',
              not graphic_rows(v, 'graphic-off-board'))

        pro_ignore = {'board': {'design_settings': {'rule_severities': {
            'copper_edge_clearance': 'ignore'}}}}
        v = drc(board(fp_off, 'ignored', pro=pro_ignore))
        check('7. copper_edge_clearance severity ignore: STILL graphic-off-board',
              any(r['owner_ref'] == 'U9' for r in graphic_rows(v, 'graphic-off-board')))
        v = drc(board(fp_off, 'zero_edge'), board_edge_clearance=0.0, clearance=0.0)
        check('7. edge clearance 0: STILL graphic-off-board',
              any(r['owner_ref'] == 'U9' for r in graphic_rows(v, 'graphic-off-board')))
        v = drc(board(fp_off, 'netsfilter'), net_patterns=['/NOSUCH*'])
        check('7. a --nets filter that drops net 0: STILL graphic-off-board',
              any(r['owner_ref'] == 'U9' for r in graphic_rows(v, 'graphic-off-board')))

        # 8 -- ring ownership: a window the part's pads sit inside
        win = ('(gr_circle (center 20 15) (end 22 15) (stroke (width 0.1) (type solid)) '
               '(fill no) (layer "Edge.Cuts"))')
        pads2 = ('(pad "1" smd rect (at -0.5 0) (size 0.4 0.4) (layers "F.Cu") (net 1 "/A"))\n'
                 ' (pad "2" smd rect (at 0.5 0) (size 0.4 0.4) (layers "F.Cu") (net 1 "/A"))')
        ring_cu = ('(fp_circle (center 0 0) (end 2 0) (stroke (width 0.2) (type solid)) '
                   '(fill no) (layer "F.Cu") (uuid "c1"))')
        own = board(win + '\n(footprint "L:W" (layer "F.Cu") (at 20 15) '
                    '(property "Reference" "W1")\n %s\n %s)' % (pads2, ring_cu), 'own_window')
        v = drc(own)
        check('8. copper on the part\'s OWN milled window (#628) is not flagged',
              not any(r['owner_ref'] == 'W1' for r in graphic_rows(v, 'graphic-off-board')),
              str(graphic_rows(v)))
        foreign = board(win + '\n(footprint "L:W" (layer "F.Cu") (at 30 15) '
                        '(property "Reference" "W2")\n %s\n %s)'
                        % (pads2.replace('(at -0.5 0)', '(at 0 0)').replace('(at 0.5 0)', '(at 0.9 0)'),
                           ring_cu.replace('(center 0 0) (end 2 0)',
                                           '(center -10 0) (end -8.1 0)')),
                        'foreign_window')
        v = drc(foreign)
        check('8. the same copper crossing a window the part does NOT own is flagged',
              any(r['owner_ref'] == 'W2' for r in graphic_rows(v, 'graphic-off-board')),
              str([(r['type'], r.get('owner_ref')) for r in v if 'graphic' in r['type']]))

        # 9 -- unmodelled copper is listed
        oc = footprint_graphic_outline_census(
            parse_kicad_pcb(os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')))
        check('9. orangecrab\'s pad-less G*** logo copper is listed unmeasured',
              {u['owner_ref'] for u in oc['unmeasured']} >= {'G***'}
              and all(u['kind'] == 'logo' for u in oc['unmeasured']),
              str(oc['unmeasured']))
    finally:
        shutil.rmtree(work, ignore_errors=True)

    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
