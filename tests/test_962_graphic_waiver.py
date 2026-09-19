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
            # kicad_drc_compare pairs by net; KiCad names this item <no net>
            check('1. ... net1 is the net-0 name (pairable), and no overlap_mm (not a '
                  'clearance quantity, so "in CONTACT" cannot count it)',
                  g[0]['net1'] == '' and 'overlap_mm' not in g[0], str(g[0]))
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
        # The severity leg is the one that switches the edge pass OFF: an edge
        # clearance of 0 is pinned up to the fab floor (fab_edge_floor), so
        # the edge pass still runs there and cannot show the grade is outside
        # it. That leg was dropped as vacuous (#962 phase-2 verification).
        ign = board(fp_off, 'ignored', pro=pro_ignore)
        v = drc(ign)
        check('7. copper_edge_clearance severity ignore: STILL graphic-off-board',
              any(r['owner_ref'] == 'U9' for r in graphic_rows(v, 'graphic-off-board')))
        import io
        import contextlib
        _buf = io.StringIO()
        with contextlib.redirect_stdout(_buf):
            run_drc(ign, clearance=0.25, clearance_margin=0.0, quiet=False,
                    print_summary=False)
        check('7. ... and that board really skips the edge pass (the leg is not vacuous)',
              "Skipping board edge clearances" in _buf.getvalue())
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
              {u['owner_ref'] for u in oc['unmeasured'] if u['kind'] == 'logo'} >= {'G***'},
              str(oc['unmeasured']))
        txt = board('(footprint "L:T" (layer "F.Cu") (at 10 10) (property "Reference" "T1")\n'
                    ' %s\n (fp_text user "HI" (at 0 2) (layer "F.Cu") (uuid "x1") '
                    '(effects (font (size 1 1) (thickness 0.15)))))\n'
                    '(footprint "L:T" (layer "F.Cu") (at 20 10) (property "Reference" "T2")\n'
                    ' %s\n (fp_text user "HI" (at 0 2) (layer "F.Cu") (hide yes) (uuid "x2") '
                    '(effects (font (size 1 1) (thickness 0.15)))))'
                    % (pad, pad), 'copper_text')
        un = footprint_graphic_outline_census(parse_kicad_pcb(txt))['unmeasured']
        check('9. VISIBLE copper text is listed unmeasured (kind text); hidden text is not',
              [(u['owner_ref'], u['kind']) for u in un] == [('T1', 'text')], str(un))
        # the forms the cheap pre-skip must not drop (phase-2 verification, round 2)
        odd = board(
            '(footprint "L:G" (layer "F.Cu") (at 5 5) (property "Reference" "G1")\n'
            ' (fp_poly (pts (xy 0 0) (xy 1 0) (xy 1 1)) (stroke (width 0) (type solid)) '
            '(fill yes) (layers "F.Cu" "F.Mask")))\n'
            '(footprint "L:Q" (layer "F.Cu") (at 15 5) (property "Reference" "Q1")\n'
            ' %s\n (pad "2" smd rect (at 2 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A"))\n'
            ' (fp_curve (pts (xy 0 1) (xy 1 2) (xy 2 2) (xy 3 1)) (stroke (width 0.2) '
            '(type solid)) (layers "F.Cu" "F.Mask")))\n'
            '(footprint "L:K" (layer "F.Cu") (at 25 5) (property "Reference" "K1")\n'
            ' %s\n (fp_text user "KO" (at 0 2) (layer "F.Cu" knockout) '
            '(effects (font (size 1 1) (thickness 0.15)))))\n'
            '(footprint "L:H" (layer "F.Cu") (at 35 5) (property "Reference" "H2")\n'
            ' %s\n (fp_text user "do not hide me" (at 0 2) (layer "F.Cu") '
            '(effects (font (size 1 1) (thickness 0.15)))))'
            % (pad, pad, pad), 'odd_unmodelled')
        un = footprint_graphic_outline_census(parse_kicad_pcb(odd))['unmeasured']
        check('9. plural-layer logo and curve, knockout text and text SAYING "hide" '
              'are all listed', sorted((u['owner_ref'], u['kind']) for u in un)
              == [('G1', 'logo'), ('H2', 'text'), ('K1', 'text'), ('Q1', 'curve')], str(un))
        nob = os.path.join(work, 'no_outline.kicad_pcb')
        with open(nob, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb (version 20240108) (generator "t")\n'
                     ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                     ' (setup)\n (net 0 "")\n (net 1 "/A")\n%s\n)' % fp_off)
        cen = footprint_graphic_outline_census(parse_kicad_pcb(nob))
        check('9. a board with NO outline: no rows, and its graphic copper is listed '
              'unmeasured (no-outline)', not cen['rows']
              and ('U9', 'no-outline') in [(u['owner_ref'], u['kind']) for u in cen['unmeasured']],
              str(cen))

        # 10 -- the B side, graded absolutely (not only "the three agree")
        fp_b = ('(footprint "L:P" (layer "B.Cu") (at 38.5 10) (property "Reference" "U6")\n'
                ' %s\n %s)' % (pad.replace('"F.Cu"', '"B.Cu"'),
                               tab_off.replace('"F.Cu"', '"B.Cu"')))
        v = graphic_rows(drc(board(fp_b, 'bside_off')), 'graphic-off-board')
        check('10. a B.Cu tab 1.5 mm past the edge: graphic-off-board, overrun 1.55 '
              '(1.5 + half the 0.1 stroke), layer B.Cu',
              len(v) == 1 and abs(v[0]['overrun_mm'] - 1.55) <= 0.005
              and v[0]['layer'] == 'B.Cu', str(v))

        # 11 / 12 -- --baseline compares the WHOLE pose: rotation and side too.
        # A tab symmetric about the part origin looks identical after 180
        # degrees and after a top-bottom flip, so only the pose says it moved.
        sym = ('(fp_poly (pts (xy -1.4 -0.5) (xy 1.4 -0.5) (xy 1.4 0.5) (xy -1.4 0.5)) '
               '(stroke (width 0.1) (type solid)) (fill yes) (layer "%s") (uuid "s1"))')
        pad0 = '(pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "%s") (net 1 "/A"))'

        def symfp(side, rot):
            cu = side + '.Cu'
            # copper reach 38.45 + 1.4 + 0.05 = 39.9: 0.1 mm inside x=40
            return ('(footprint "L:S" (layer "%s") (at 38.45 10 %s) '
                    '(property "Reference" "S1")\n %s\n %s)'
                    % (cu, rot, pad0 % cu, sym % cu))
        b_f0 = board(symfp('F', 0), 'sym_f0')
        for nm, cur in (('rotated 180', board(symfp('F', 180), 'sym_f180')),
                        ('flipped to the B side', board(symfp('B', 0), 'sym_b0'))):
            pl = graphic_rows(drc(cur, baseline=b_f0, board_edge_clearance=0.3),
                              'graphic-board-edge')
            check('11. the same copper, %s against --baseline: origin placement' % nm,
                  pl and all(r.get('origin') == 'placement' and r['owner_ref'] == 'S1'
                             for r in pl), str(pl))
        vi = drc(b_f0, baseline=b_f0, board_edge_clearance=0.3)
        check('12. control: unmoved against itself, the graze is accepted inherited',
              not graphic_rows(vi)
              and {r.get('origin') for r in accepted_graphic(vi)} == {'inherited'},
              str([(r['type'], r.get('origin')) for r in vi if 'graphic' in str(r.get('item1'))]))

        # 13 / 14 -- the placement channel: waivers never count, an owner the
        # board cannot resolve always does
        from placement.legality import _graphic_copper_channel
        for nm, p, key in (('owner of the outline', po, 'J9'),
                           ('board-level art', board(gr, 'boardlevel_leg'), '<board>')):
            ch = _graphic_copper_channel(parse_kicad_pcb(p), 0.0)
            check('13. %s: count 0, listed in _waived instead' % nm,
                  ch['oob_graphic_copper_count'] == 0 and ch['oob_graphic_copper_amount'] == 0
                  and key in [w[0] for w in ch['oob_graphic_copper_waived']],
                  str({k: ch[k] for k in ch if k != 'oob_graphic_copper_basis'}))
        pu = parse_kicad_pcb(board(fp_off, 'unresolved'))
        pu.footprints.pop('U9')
        cen = footprint_graphic_outline_census(pu)
        ch = _graphic_copper_channel(pu, 0.0)
        check('14. an owner with no footprint is `unresolved` and COUNTED, not waived',
              [r['owner_state'] for r in cen['rows'] if r['owner_ref'] == 'U9'] == ['unresolved']
              and ch['oob_graphic_copper_count'] == 1, str(ch['oob_graphic_copper_refs']))

        # 15 -- a side change in memory is not graded on the old side's copper
        pm = parse_kicad_pcb(board(fp_off, 'memflip'))
        pm.footprints['U9'].layer = 'B.Cu'
        cen = footprint_graphic_outline_census(pm)
        check('15. a part flipped IN MEMORY: no row graded from stale copper, listed '
              'unmeasured (moved-side)',
              not [r for r in cen['rows'] if r['owner_ref'] == 'U9']
              and ('U9', 'moved-side') in [(u['owner_ref'], u['kind']) for u in cen['unmeasured']],
              str(cen['unmeasured']))

        # 16 -- a circle is measured on its TRUE curve. At 11.25 degrees the
        # 16-gon's chord midpoint faces the edge, 1.9% of r short of it.
        circ = ('(footprint "L:C" (layer "F.Cu") (at 37.97 15 11.25) (property "Reference" "C7")\n'
                ' %s\n (fp_circle (center 0 0) (end 2 0) (stroke (width 0.1) (type solid)) '
                '(fill no) (layer "F.Cu") (uuid "c7")))' % pad0 % 'F.Cu')
        rows = [r for r in footprint_graphic_outline_census(
            parse_kicad_pcb(board(circ, 'true_circle')))['rows'] if r['owner_ref'] == 'C7']
        check('16. the true circle reaches 0.02 mm past the edge (the 16-gon would not)',
              len(rows) == 1 and abs(rows[0]['overrun_mm'] - 0.02) <= 0.001, str(rows))

        # 17 -- a board cutout lying wholly inside a FILLED tab
        hole = ('(gr_circle (center 20 15) (end 20.5 15) (stroke (width 0.1) (type solid)) '
                '(fill no) (layer "Edge.Cuts"))')
        filled_tab = ('(footprint "L:H" (layer "F.Cu") (at 20 15) (property "Reference" "H1")\n'
                      ' (pad "1" smd rect (at -1.5 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A"))\n'
                      ' (fp_poly (pts (xy -2 -1) (xy 2 -1) (xy 2 1) (xy -2 1)) (stroke (width 0.1) '
                      '(type solid)) (fill %s) (layer "F.Cu") (uuid "h1")))')
        vh = graphic_rows(drc(board(hole + '\n' + filled_tab % 'yes', 'hole_filled')),
                          'graphic-off-board')
        check('17. a FILLED tab over a cutout wholly inside it: graphic-off-board, '
              'depth 1.05 (1.0 in the copper + half the stroke)',
              len(vh) == 1 and vh[0]['owner_ref'] == 'H1'
              and abs(vh[0]['overrun_mm'] - 1.05) <= 0.01, str(vh))
        vh = graphic_rows(drc(board(hole + '\n' + filled_tab % 'no', 'hole_unfilled')),
                          'graphic-off-board')
        check('17. control: the same outline UNFILLED is not over the cutout',
              not vh, str(vh))
        # a filled KEYHOLE (a +-2 square with a slit into a +-1 hole) whose
        # outline starts on its bridge vertex: it revisits vertex 0 mid-outline,
        # and splitting it there read the copper-free hole as copper
        key = ('(footprint "L:H" (layer "F.Cu") (at 20 15) (property "Reference" "H3")\n'
               ' (pad "1" smd rect (at -1.5 1.5) (size 0.4 0.4) (layers "F.Cu") (net 1 "/A"))\n'
               ' (fp_poly (pts (xy 1 0) (xy 2 0) (xy 2 2) (xy -2 2) (xy -2 -2) (xy 2 -2) '
               '(xy 2 0) (xy 1 0) (xy 1 -1) (xy -1 -1) (xy -1 1) (xy 1 1)) '
               '(stroke (width 0.1) (type solid)) (fill yes) (layer "F.Cu") (uuid "k1")))')
        pk = parse_kicad_pcb(board(hole + '\n' + key, 'keyhole'))
        kr = [r for r in footprint_graphic_outline_census(pk)['rows'] if r['owner_ref'] == 'H3']
        check('17. a filled keyhole around a cutout: ONE shape, graded inside the '
              'board (the hole is not copper)',
              len(kr) == 1 and kr[0]['overrun_mm'] < 0, str(kr))

        # 18 -- the placement driver implicates the owner of a COUNTED row only
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            'pd962', os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                                  'scripts', 'placement_driver.py'))
        pd = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(pd)
        js = os.path.join(work, 'drc_items.json')
        with open(js, 'w', encoding='utf-8') as fh:
            json.dump({'items': [
                {'type': 'graphic-off-board', 'owner_ref': 'U2', 'item1': 'x'},
                {'type': 'segment-board-edge', 'owner_ref': 'AE1',
                 'accepted': 'immutable-graphic'}]}, fh)
        got = pd._implicated_refs([js])
        check('18. _implicated_refs names U2 (counted) and not AE1 (accepted)',
              'U2' in got and 'AE1' not in got, str(got))

        # 19 -- kicad_drc_compare carries both types and the baseline
        sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
        import kicad_drc_compare as kdc
        check('19. kicad_drc_compare pairs both new types with copper_edge_clearance',
              {'graphic-off-board', 'graphic-board-edge'} <= kdc.EDGE_CD_TYPES)
        import check_drc as _cd
        seen = {}
        _real = _cd.run_drc

        def _spy(*a, **kw):
            seen.update(kw)
            return []
        _cd.run_drc = _spy
        try:
            kdc.run_check_drc(ign, baseline=b_f0)
        finally:
            _cd.run_drc = _real
        check('19. run_check_drc hands --baseline to run_drc', seen.get('baseline') == b_f0,
              str(seen))
    finally:
        shutil.rmtree(work, ignore_errors=True)

    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
