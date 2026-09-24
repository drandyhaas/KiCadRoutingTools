#!/usr/bin/env python3
"""#1031: a board's rule-area keep-out band is a pad-copper legality term.

The router blocks every track cell inside a `(keepout (tracks not_allowed))`
rule area and within `clearance + track_width/2` of it. No placement
instrument modelled that, so a part could be seated with signal pads in the
band while check_assembly, place_pose, render_placement and check_reachability
all stayed green (glasgow_revC, run 32).

Built on a SYNTHETIC 4-layer board, so it always runs. The board carries a
2 mm keep-out band (F.Cu + B.Cu, tracks and vias forbidden) round its edge:

  R1   SMD, both pads IN the band                  -> illegal
  R3   SMD, pad copper reaches the band with its far edge only; a track can
       still land on its inward half               -> legal (the discriminator:
       "any copper within the band" would flag it, the router routes it)
  J1   through-hole, in the band, reachable on In1/In2 -> reported, not failed
  FID1 net-less SMD in the band                    -> exempt (needs no track)
  R2/R4 the far ends of the nets, well inside      -> clean

Invariants:
1. grade_pad_legality names exactly R1 (both pads), R3 is not named, J1 is
   in keepout_copper_tht_refs, FID1 in keepout_copper_exempt; the basis,
   band and track-width source travel with the numbers.
2. check_assembly publishes the same keys with the same values, and its
   verdict does not move (#937: not a not_buildable conjunct).
3. render_placement's checklist.a_off_outline.keepout_copper equals the
   grade (same helper, proposed-pose path), is ALWAYS emitted (a board with
   no keep-out emits []), and --gate fails on it by name.
4. place_pose refuses a move of R2 into the band (exit 4, nothing written)
   and accepts the same-sized move that stays clear.
5. The search conjunct (LegalityContext.keepout_ok, which
   quench.candidate_valid calls): a clean part may not enter the band; a part
   the seed already seats there may move out, and may not go deeper.
6. check_reachability: R1.1 is CAGED, R3.1 is PASSABLE.
7. Inert: a board with no keep-out grades 0 and the context carries None.

The run-32 repro lives in tests/test_1031_keepout_run32_repro.py (it needs
the gitignored wk/run32 boards and self-skips without them).

Run:
    python3 tests/test_1031_keepout_legality.py
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

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import legality  # noqa: E402
from placement.legality import grade_pad_legality  # noqa: E402
from run_utils import check as run_check, evidence  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


KEEPOUT = '''  (zone (net 0) (net_name "") (layers "F.Cu" "B.Cu") (uuid "ko1")
    (name "edge band") (hatch edge 0.5) (connect_pads (clearance 0))
    (min_thickness 0.25)
    (keepout (tracks not_allowed) (vias not_allowed) (pads allowed)
      (copperpour allowed) (footprints allowed))
    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 0 0) (xy 40 0) (xy 40 30) (xy 0 30)))
    (polygon (pts (xy 2 2) (xy 38 2) (xy 38 28) (xy 2 28))))
'''


def _res(ref, x, y, n1, n2, uid, rot=0):
    return ('  (footprint "R:R_0603" (layer "F.Cu") (uuid "%s") (at %s %s %s)\n'
            '    (property "Reference" "%s" (at 0 -1.2 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -1.5 -0.7) (end 1.5 0.7) (stroke (width 0.05) '
            '(type solid)) (fill none) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -0.8 0) (size 0.8 0.9) '
            '(layers "F.Cu" "F.Mask" "F.Paste") (net %d "%s"))\n'
            '    (pad "2" smd rect (at 0.8 0) (size 0.8 0.9) '
            '(layers "F.Cu" "F.Mask" "F.Paste") (net %d "%s")))\n'
            % (uid, x, y, rot, ref, n1, NETS[n1], n2, NETS[n2]))


NETS = {1: '/A', 2: '/B', 3: '/C', 4: '/D', 5: '/E'}


def board_text(parts, keepout=True):
    nets = ''.join(' (net %d "%s")\n' % (k, v) for k, v in NETS.items())
    return ('(kicad_pcb (version 20240108) (generator "t")\n'
            ' (general (thickness 1.6))\n'
            ' (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) '
            '(2 "In2.Cu" signal) (31 "B.Cu" signal)\n'
            '  (44 "Edge.Cuts" user) (37 "F.SilkS" user) (39 "F.Mask" user)\n'
            '  (35 "F.Paste" user) (49 "F.Fab" user) (47 "F.CrtYd" user))\n'
            ' (setup)\n (net 0 "")\n' + nets +
            ' (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) '
            '(type solid)) (fill no) (layer "Edge.Cuts"))\n'
            + (KEEPOUT if keepout else '') + ''.join(parts) + ')\n')


def default_parts(r2_at=(20, 15)):
    return [
        _res('R1', 1.6, 15, 1, 2, 'u1', 90),      # both pads in the band
        _res('R2', r2_at[0], r2_at[1], 1, 2, 'u2'),
        _res('R3', 3.15, 10, 3, 4, 'u3'),         # far edge grazes the band
        _res('R4', 20, 20, 3, 4, 'u4'),
        # J1: through-hole in the band, net /E, far end on R5
        '  (footprint "C:PinHeader" (layer "F.Cu") (uuid "u5") (at 1.2 5)\n'
        '    (property "Reference" "J1" (at 0 -2 0) (layer "F.SilkS"))\n'
        '    (pad "1" thru_hole circle (at 0 0) (size 1.7 1.7) (drill 1.0) '
        '(layers "*.Cu" "*.Mask") (net 5 "/E"))\n'
        # a SECOND copper pad under the same number (a shield's two tabs,
        # a mounting hole's ring): one finding row, not two
        '    (pad "1" thru_hole circle (at 0 2) (size 1.7 1.7) (drill 1.0) '
        '(layers "*.Cu" "*.Mask") (net 5 "/E")))\n',
        '  (footprint "C:PinHeader" (layer "F.Cu") (uuid "u6") (at 25 8)\n'
        '    (property "Reference" "J2" (at 0 -2 0) (layer "F.SilkS"))\n'
        '    (pad "1" thru_hole circle (at 0 0) (size 1.7 1.7) (drill 1.0) '
        '(layers "*.Cu" "*.Mask") (net 5 "/E")))\n',
        # FID1: net-less fiducial in the band
        '  (footprint "F:Fiducial" (layer "F.Cu") (uuid "u7") (at 1 28)\n'
        '    (property "Reference" "FID1" (at 0 -2 0) (layer "F.SilkS"))\n'
        '    (pad "" smd circle (at 0 0) (size 1 1) '
        '(layers "F.Cu" "F.Mask")))\n',
        # R6: a WIDE pad whose CENTRE is inside the band (0.2 from the
        # region, band 0.275) while its inward edge landing clears it
        '  (footprint "R:R_wide" (layer "F.Cu") (uuid "u8") (at 3.2 23 0)\n'
        '    (property "Reference" "R6" (at 0 -1.2 0) (layer "F.SilkS"))\n'
        '    (pad "1" smd rect (at -1.0 0) (size 1.2 0.9) '
        '(layers "F.Cu" "F.Mask" "F.Paste") (net 3 "/C"))\n'
        '    (pad "2" smd rect (at 1.0 0) (size 1.2 0.9) '
        '(layers "F.Cu" "F.Mask" "F.Paste") (net 4 "/D")))\n',
    ]


def zone(net, x0, y0, x1, y1, layer='F.Cu', uid='z1'):
    """A same-net copper zone outline (no fill needed: the channel reads the
    outline)."""
    return ('  (zone (net %d) (net_name "%s") (layer "%s") (uuid "%s")\n'
            '    (hatch edge 0.5) (connect_pads (clearance 0.2))\n'
            '    (min_thickness 0.25) (fill (thermal_gap 0.5) '
            '(thermal_bridge_width 0.5))\n'
            '    (polygon (pts (xy %s %s) (xy %s %s) (xy %s %s) (xy %s %s))))\n'
            % (net, NETS[net], layer, uid, x0, y0, x1, y0, x1, y1, x0, y1))


def write_board(work, name, parts=None, keepout=True):
    p = os.path.join(work, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(board_text(parts if parts is not None else default_parts(),
                            keepout))
    return p


def _pairs(rows):
    return sorted((str(a), round(float(b), 3)) for a, b in rows)


def main():
    work = tempfile.mkdtemp(prefix='krt1031_')
    try:
        bd = write_board(work, 'ko')
        pcb = parse_kicad_pcb(bd)
        check('0. the synthetic board parses its keep-out',
              len(pcb.board_info.keepouts) == 1
              and not pcb.board_info.keepouts[0]['tracks_allowed'],
              str(pcb.board_info.keepouts))

        # 1 -- the grade
        g = grade_pad_legality(pcb, 0.2, pcb_file=bd)
        pads = {(r[0], r[1]) for r in g['keepout_copper_pads']}
        check('1. exactly R1 is illegal, both pads',
              g['oob_keepout_copper_count'] == 1
              and [r[0] for r in g['oob_keepout_copper_refs']] == ['R1']
              and pads == {('R1', '1'), ('R1', '2')},
              str(g['oob_keepout_copper_refs']) + str(sorted(pads)))
        check('1. R3 (far edge in the band, inward landing open) is NOT named',
              not any(r[0] == 'R3' for r in g['keepout_copper_pads']
                      + g['keepout_copper_tht_refs'] + g['keepout_copper_exempt']))
        check('1. J1 (through-hole, In1/In2 uncovered) is reported, not failed',
              [r[0] for r in g['keepout_copper_tht_refs']] == ['J1'],
              str(g['keepout_copper_tht_refs']))
        check('1. FID1 (net-less) is exempt with its reason',
              [(r[0], r[-1]) for r in g['keepout_copper_exempt']]
              == [('FID1', 'no_connection')], str(g['keepout_copper_exempt']))
        check('1. amount is the sum of the per-part worst reach',
              abs(g['oob_keepout_copper_amount']
                  - g['oob_keepout_copper_refs'][0][1]) < 1e-9)
        check('1. band = clearance + track/2, track width sourced',
              abs(g['keepout_copper_band_mm'] - (0.2 + 0.15 / 2)) < 1e-9
              and g['keepout_copper_track_width']['source'] == 'fixed default'
              and 'router' in g['keepout_copper_basis'],
              str((g['keepout_copper_band_mm'], g['keepout_copper_track_width'])))

        # 2 -- check_assembly publishes the SAME values, verdict unchanged
        js = os.path.join(work, 'asm.json')
        run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'check_assembly.py'), bd,
                   '--clearance', '0.2', '--json', js], accept=True)
        evidence(js, 'check_assembly json')
        doc = json.load(open(js, encoding='utf-8'))
        for k in ('oob_keepout_copper_count', 'oob_keepout_copper_amount',
                  'oob_keepout_copper_refs', 'keepout_copper_pads',
                  'keepout_copper_tht_refs', 'keepout_copper_exempt',
                  'keepout_copper_unmeasured', 'keepout_copper_basis'):
            check('2. check_assembly publishes %s == the grade' % k,
                  json.loads(json.dumps(g[k])) == doc.get(k),
                  '%r vs %r' % (g[k], doc.get(k)))
        check('2. its verdict does not move (not a not_buildable conjunct)',
              str(doc.get('verdict', '')).startswith('buildable'),
              str(doc.get('verdict')))

        # 3 -- render_placement: same helper at the model's poses, and --gate
        rj = os.path.join(work, 'rp.json')
        r = run_check([sys.executable, '-X', 'utf8',
                       os.path.join(ROOT, 'py_tools', 'render_placement.py'),
                       bd, '--clearance', '0.2', '--json-out', rj, '-o',
                       os.path.join(work, 'rp.png'), '--gate'],
                      refuse='a_off_outline.keepout_copper=1', code=4)
        evidence(rj, 'render_placement json')
        chk = json.load(open(rj, encoding='utf-8'))['checklist']['a_off_outline']
        check('3. render_placement keepout_copper == grade_pad_legality',
              _pairs(chk['keepout_copper'])
              == _pairs(g['oob_keepout_copper_refs']),
              '%r vs %r' % (chk['keepout_copper'], g['oob_keepout_copper_refs']))
        check('3. ...and names the same pads',
              {(p[0], p[1]) for p in chk['keepout_copper_pads']} == pads)
        check('3. --gate fails, naming a_off_outline.keepout_copper',
              r.returncode == 4
              and 'a_off_outline.pad_copper' not in (r.stdout + r.stderr),
              'rc=%s' % r.returncode)
        clean = write_board(work, 'nokeep', keepout=False)
        rj2 = os.path.join(work, 'rp2.json')
        run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'render_placement.py'),
                   clean, '--clearance', '0.2', '--json-out', rj2, '-o',
                   os.path.join(work, 'rp2.png')], accept=True)
        evidence(rj2, 'render_placement json (no keep-out)')
        chk2 = json.load(open(rj2, encoding='utf-8'))['checklist']['a_off_outline']
        check('3. the key is ALWAYS emitted ([] on a board with no keep-out)',
              chk2.get('keepout_copper') == [], str(chk2.get('keepout_copper')))

        # 4 -- place_pose refuses a move into the band
        out = os.path.join(work, 'posed.kicad_pcb')
        r = run_check([sys.executable, '-X', 'utf8',
                       os.path.join(ROOT, 'py_placer', 'place_pose.py'), bd,
                       out, '--clearance', '0.2', 'set', 'R2', '37.6', '15'],
                      refuse='oob_keepout_copper_count 1 -> 2', code=4)
        # 37.6 puts pad 2 in the band and 1.2 mm from the edge: the keep-out
        # arm is the ONLY one that moves, so the refusal is this channel's.
        _why = r.stdout.split('WORSE (', 1)[-1].split(')', 1)[0]
        check('4. place_pose refuses R2 into the band: exit 4, nothing '
              'written, for the keep-out alone',
              r.returncode == 4 and not os.path.exists(out)
              and 'pad_edge' not in _why and 'oob_pad' not in _why,
              'rc=%s %s' % (r.returncode, _why[-600:]))
        r = run_check([sys.executable, '-X', 'utf8',
                       os.path.join(ROOT, 'py_placer', 'place_pose.py'), bd,
                       out, '--clearance', '0.2', 'set', 'R2', '30', '15'],
                      accept=True)
        check('4. ...and accepts a move that stays clear (exit 0, written)',
              r.returncode == 0 and os.path.exists(out),
              'rc=%s %s' % (r.returncode, r.stdout[-600:]))

        # 5 -- the search conjunct quench.candidate_valid calls
        fps = pcb.footprints
        parts = legality.build_part_pads(fps, 0.2)
        seeds = {k: (fp.x, fp.y, fp.rotation or 0.0) for k, fp in fps.items()}
        ctx = legality.LegalityContext(
            parts, None, 0.2, pose_of=lambda r: seeds[r],
            seed_of=lambda r: seeds[r],
            keepouts=legality.RuleAreaKeepouts.for_board(pcb, 0.2, bd))
        check('5. a clean part may not enter the band',
              ctx.keepout_ok('R2', 20, 15, 0)
              and not ctx.keepout_ok('R2', 38.6, 15, 0))
        check('5. a part the seed seats in the band may move out, not deeper',
              ctx.keepout_ok('R1', 4.0, 15, 90)
              and ctx.keepout_ok('R1', 1.6, 15, 90)
              and ctx.keepout_ok('R1', 1.8, 15, 90)
              and not ctx.keepout_ok('R1', 1.0, 15, 90))
        check('5. the through-hole part is not gated by the search',
              ctx.keepout_ok('J1', 0.8, 5, 0))

        # 6 -- check_reachability sees the band
        def reach(pad, **kw):
            rr = run_check([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_tools',
                                         'check_reachability.py'), bd,
                            '--pad', pad], **kw)
            return rr.stdout
        # exit 1 is reserved for the geometry verdict (its --help)
        o1 = reach('R1.1', refuse='VERDICT    CAGED', code=1)
        check('6. check_reachability R1.1 is CAGED', 'CAGED' in o1, o1[-400:])
        # R3's far edge grazes the band; its partner R4 is inside, so the
        # net is reachable -- the discriminator again, on the other tool.
        o2 = reach('R3.1', accept=True)
        check('6. check_reachability R3.1 is PASSABLE', 'PASSABLE' in o2,
              o2[-400:])

        # 8 -- EVERY search move is gated, through the real callers rather
        # than a hand-built context: candidate_valid, the swap phase and
        # relocate's exact re-check all reach LegalityContext.pads_ok, which
        # carries the keep-out conjunct.
        from placement.quench import QuenchState
        from placement import relocate as RL
        st = QuenchState(parse_kicad_pcb(bd), bd, 0.2, 0.55, 10.0, 0.5, 0.25,
                         2.0, 2.0, 2.0, 0.1, 1.0)
        check('8. candidate_valid refuses R2 into the band, accepts clear',
              not st.candidate_valid('R2', 37.6, 15, 0.0)
              and st.candidate_valid('R2', 30.0, 15, 0.0))
        check('8. the swap phase refuses R2 onto R1\'s in-band pose',
              not st.swap_pads_ok('R1', 'R2'))
        units = RL.rigid_units(st, None)
        why = RL.exact_refusal(st, units, {units.of_ref['R2']: (17.6, 0.0)})
        check('8. relocate refuses a block shift into the band, naming the '
              'keep-out', why.startswith('rule_area_keepout_refused_a_shift:R2'), why)
        check('8. ...and accepts the same shift that stays clear',
              RL.exact_refusal(st, units,
                               {units.of_ref['R2']: (10.0, 0.0)}) == '')
        m = st.pad_legality_metrics()
        check('8. the quench tallies the in-band part for the portfolio gate',
              m.get('keepout_pad_parts') == 1
              and m.get('keepout_pad_amount', 0) > 0, str(m))

        # 9 -- the portfolio hard gate reads that tally
        from placement import portfolio
        qm = portfolio._quench_metrics({'legality': m})
        check('9. _quench_metrics carries keepout_pad_parts',
              qm.get('keepout_pad_parts') == 1, str(qm))

        def _score(parts_in, base):
            c = portfolio.Candidate(index=1, strategy='t', board=bd,
                                    metrics={'keepout_pad_parts': parts_in})
            portfolio.score_candidate(
                c, free=[], baseline_overlap=1e9, baseline_oob=10 ** 6,
                baseline_pad_pairs=10 ** 6, baseline_hole_shortfall=1e9,
                baseline_keepout_parts=base, clearance=0.2,
                board_edge_clearance=0.55, grid_step=0.1, ignore_nets=None)
            return c
        c = _score(1, 0)
        check('9. score_candidate gates a candidate with MORE parts in the '
              'band than the baseline',
              c.gates.get('passed') is False and 'keep-out band' in
              ' '.join(c.gates.get('reasons') or []), str(c.gates))
        check('9. ...and not one that only keeps the baseline\'s',
              _score(1, 1).gates.get('passed') is True)

        # 10 -- pour-served means a same-net zone REACHES the pad
        def ko_rows(parts, name, keepout_text=None):
            p = os.path.join(work, name + '.kicad_pcb')
            txt = board_text(parts)
            if keepout_text is not None:
                txt = txt.replace(KEEPOUT, keepout_text)
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write(txt)
            gg = grade_pad_legality(parse_kicad_pcb(p), 0.2, pcb_file=p)
            return ({(r[0], r[1]) for r in gg['keepout_copper_pads']},
                    {(r[0], r[1], r[-1]) for r in gg['keepout_copper_exempt']})
        far = default_parts() + [zone(1, 20, 5, 24, 9)]
        ill, ex = ko_rows(far, 'far_zone')
        check('10. a same-net zone 20 mm away does NOT exempt R1.1',
              ('R1', '1') in ill and ('R1', '1', 'pour_served') not in ex,
              str((sorted(ill), sorted(ex))))
        over = default_parts() + [zone(1, 0.5, 13, 3.0, 17)]
        ill, ex = ko_rows(over, 'over_zone')
        check('10. a same-net zone covering R1.1 exempts it (pour allowed)',
              ('R1', '1') not in ill and ('R1', '1', 'pour_served') in ex
              and ('R1', '2') in ill, str((sorted(ill), sorted(ex))))
        ill, ex = ko_rows(over, 'over_zone_nopour',
                          KEEPOUT.replace('(copperpour allowed)',
                                          '(copperpour not_allowed)'))
        check('10. ...but not where the rule area forbids pour',
              ('R1', '1') in ill, str((sorted(ill), sorted(ex))))
        # ...nor where a SECOND rule area -- tracks allowed, pour forbidden --
        # covers the contact point while the band itself allows pour. This is
        # the `_pour_reaches` ban loop; the arm above never reaches it,
        # because the band's own pour flag short-circuits first.
        nopour = ('  (zone (net 0) (net_name "") (layer "F.Cu") (uuid "ko2")\n'
                  '    (hatch edge 0.5) (connect_pads (clearance 0))\n'
                  '    (min_thickness 0.25)\n'
                  '    (keepout (tracks allowed) (vias allowed) '
                  '(pads allowed) (copperpour not_allowed) '
                  '(footprints allowed))\n'
                  '    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
                  '    (polygon (pts (xy 0.2 12.5) (xy 3.5 12.5) (xy 3.5 17.5)'
                  ' (xy 0.2 17.5))))\n')
        ill, ex = ko_rows(over + [nopour], 'over_zone_second_nopour')
        check('10. ...nor where a second pour-forbidding rule area covers '
              'the contact point',
              ('R1', '1') in ill and ('R1', '1', 'pour_served') not in ex,
              str((sorted(ill), sorted(ex))))

        # 11 -- the landing is ANY free point of the pad, not its centre:
        # R6.1's centre sits 0.2 from the region (band 0.275) and its
        # inward edge clears it, so it is legal; a centre-only reading
        # would name it.
        ko = legality.RuleAreaKeepouts.for_board(pcb, 0.2, bd)
        r6 = parts['R6'].pad_rects(*seeds['R6'])[0]
        cx = (r6[0] + r6[2]) / 2.0
        check('11. R6.1 centre is INSIDE the band (the case is live)',
              0.0 < ko._clear(ko.areas[0], cx, (r6[1] + r6[3]) / 2.0)
              < ko.band)
        check('11. ...yet an edge landing clears it: not named',
              ko.rect_amount(0, r6) == 0.0
              and not any(r[0] == 'R6' for r in g['keepout_copper_pads']))

        # 12 -- one row per (part, pad number): J1-style duplicates collapse
        check('12. no duplicate (ref, pad, area) rows in any list',
              all(len({(r[0], r[1], r[4]) for r in g[k]}) == len(g[k])
                  for k in ('keepout_copper_pads', 'keepout_copper_tht_refs')))

        # 13 -- P-close ECHOES and PERSISTS a keepout-band waiver, merged into
        # the waivers.json beside the render so P3's own keys survive
        import importlib.util
        _spec = importlib.util.spec_from_file_location(
            'pdrv_1031', os.path.join(ROOT, '.claude', 'skills',
                                      'plan-pcb-placement', 'scripts',
                                      'placement_driver.py'))
        pdrv = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(pdrv)
        ftmp = os.path.join(work, 'drv')
        os.makedirs(ftmp)
        dargv = pdrv._fixture_argv(pdrv._next_line_fixture(ftmp))
        da = pdrv._args(dargv + ['--waive', 'X:checked', '--waive',
                                 'keepout-band:R9 reached on In1 by design'])
        wpath = os.path.join(os.path.dirname(os.path.abspath(da.render_json)),
                             'waivers.json')
        with open(wpath, 'w', encoding='utf-8') as fh:
            json.dump({'unlocked_high': 1, 'waivers': {'U1': 'p3'}}, fh)
        out = pdrv.STAGES['P-close'](da)
        wdoc = json.load(open(wpath, encoding='utf-8'))
        check('13. P-close echoes the keepout-band waiver with its reason',
              'GATE WAIVERS: --waive keepout-band: R9 reached on In1 by '
              'design' in out and not out.startswith('<error>'), out[:300])
        check('13. ...and persists it, keeping P3\'s keys',
              wdoc.get('closeout', {}).get('waivers')
              == {'keepout-band': 'R9 reached on In1 by design'}
              and wdoc.get('waivers') == {'U1': 'p3'}, str(wdoc))
        out0 = pdrv.STAGES['P-close'](pdrv._args(dargv + ['--waive',
                                                          'X:checked']))
        check('13. with no gate waiver it says none',
              'GATE WAIVERS: none' in out0, out0[:300])

        # 14 -- a census that could not be BUILT is not a clean one: the
        # driver refuses it, and render_placement --gate fails on it
        rj3 = os.path.join(ftmp, 'r_ko_err.json')
        rdoc = json.load(open(da.render_json, encoding='utf-8'))
        rdoc['checklist']['a_off_outline'] = {
            'pad_copper': [], 'courtyard': [], 'keepout_copper': [],
            'keepout_copper_unmeasured': [['*', 'error', 'ValueError: x']]}
        with open(rj3, 'w', encoding='utf-8') as fh:
            json.dump(rdoc, fh)
        a3 = pdrv._args(dargv + ['--waive', 'X:checked'])
        a3.render_json = rj3
        out3 = pdrv.STAGES['P-close'](a3)
        check('14. P-close refuses a render whose keep-out census errored',
              out3.startswith('<error>')
              and 'could not build its rule-area keep-out census' in out3,
              out3[:300])

        import render_placement
        from placement import legality as _leg

        def _boom(*_a, **_k):
            raise ValueError('census exploded (test)')
        _orig = _leg.keepout_pad_findings
        _leg.keepout_pad_findings = _boom
        rj4 = os.path.join(work, 'rp_err.json')
        import contextlib
        import io
        try:
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
                try:
                    rc4 = render_placement.main(
                        [bd, '--clearance', '0.2', '--json-out', rj4, '-o',
                         os.path.join(work, 'rp_err.png'), '--gate'])
                except SystemExit as e:
                    rc4 = e.code
        finally:
            _leg.keepout_pad_findings = _orig
        chk4 = json.load(open(rj4, encoding='utf-8'))['checklist'][
            'a_off_outline']
        check('14. render_placement records the failure as an error row',
              any(u[1] == 'error' for u in chk4['keepout_copper_unmeasured'])
              and chk4['keepout_copper'] == [], str(chk4))
        check('14. ...and --gate FAILS on it rather than passing an empty list',
              rc4 == 4 and 'keepout_copper_unmeasured(error)=1'
              in buf.getvalue(), 'rc=%s %s' % (rc4, buf.getvalue()[-300:]))

        # 15 -- the landing set is the ROUTER's: a round pad lands only inside
        # its inscribed ellipse, a tilted pad only at its centre. Against a
        # 45-degree band edge the old 9-point box accepted both (0.0).
        import math
        from types import SimpleNamespace as NS
        tri = [(-50.0, 50.0), (50.0, -50.0), (50.0, 50.0)]      # x + y > 0
        ko45 = {'polygon': tri, 'holes': [], 'layers': {'F.Cu'},
                'tracks_allowed': False, 'vias_allowed': True,
                'copper_pour_allowed': True, 'in_footprint': False}

        def _npad(num, x, y, net, shape, rr=0.0, sx=2.0, sy=2.0,
                  layers=('F.Cu',), drill=0, ptype='smd'):
            return NS(pad_number=num, global_x=x, global_y=y, local_x=x,
                      local_y=y, size_x=sx, size_y=sy, shape=shape,
                      layers=list(layers), drill=drill, net_id=net,
                      net_name='/N', pad_type=ptype, rect_rotation=rr,
                      local_clearance=0.0, hole_x=None, hole_y=None)

        def _amount45(shape, rr, sx, depth):
            c = depth / math.sqrt(2)
            fp_ = NS(reference='U1', pads=[_npad('1', c, c, 1, shape, rr,
                                                 sx, sx)],
                     x=0.0, y=0.0, rotation=0.0, layer='F.Cu')
            fq_ = NS(reference='U2', pads=[_npad('1', 30, -40, 1, 'rect')],
                     x=30, y=-40, rotation=0.0, layer='F.Cu')
            pcb_ = NS(board_info=NS(copper_layers=['F.Cu', 'B.Cu'],
                                    keepouts=[ko45]),
                      footprints={'U1': fp_, 'U2': fq_}, zones=[])
            k_ = legality.RuleAreaKeepouts(pcb_, 0.2, 0.15)
            return k_.part_amount('U1', legality.PartPads(fp_, 0.2)
                                  .pad_rects(0.0, 0.0, 0.0))
        a_round = _amount45('circle', 0.0, 2.0, 0.9)
        a_tilt = _amount45('rect', 45.0, 1.4, 0.3)
        check('15. a 2 mm round pad across a 45-degree edge: 0.25 (the '
              'router\'s ellipse), not 0', abs(a_round - 0.25) < 1e-3,
              str(a_round))
        check('15. a 1.4 mm square tilted 45 degrees: 0.575 (centre only)',
              abs(a_tilt - 0.575) < 1e-3, str(a_tilt))

        # 16 -- parity with the router's own landing cells
        # (single_ended_routing._free_on_pad_cells) on sample pads: every
        # landing we sample is a cell the router could use, and the router's
        # cells span the same extent -- so the sampled set is the router's
        # region, not a guess at it.
        from single_ended_routing import _free_on_pad_cells
        from routing_config import GridCoord
        cfg = NS(track_width=0.15)
        free_obs = NS(is_blocked=lambda gx, gy, li: False)
        step = 0.01
        coord = GridCoord(step)
        kpar = legality.RuleAreaKeepouts.for_board(pcb, 0.2, bd)
        for shape, sx, sy, rr in (('circle', 2.0, 2.0, 0.0),
                                  ('oval', 2.0, 1.0, 0.0),
                                  ('rect', 1.2, 0.8, 0.0),
                                  ('roundrect', 1.0, 0.6, 0.0),
                                  ('rect', 1.4, 1.4, 30.0)):
            P = _npad('1', 10.0, 10.0, 1, shape, rr, sx, sy)
            cells = _free_on_pad_cells(P, 0, cfg, free_obs, coord)
            lands = kpar.landings((10 - sx / 2, 10 - sy / 2, 10 + sx / 2,
                                   10 + sy / 2), P, 0.0)
            if rr:
                ok = cells == [] and lands == [(10.0, 10.0)]
            else:
                xs = [g[0] * step for g in cells]
                ys = [g[1] * step for g in cells]
                hx = sx / 2 - 0.075
                hy = sy / 2 - 0.075
                rnd = shape in ('circle', 'oval')
                inside = all(
                    (((qx - 10) / hx) ** 2 + ((qy - 10) / hy) ** 2
                     <= 1 + 1e-9) if rnd else
                    (abs(qx - 10) <= hx + 1e-9 and abs(qy - 10) <= hy + 1e-9)
                    for qx, qy in lands)
                span = (abs(min(xs) - min(q[0] for q in lands)) <= step
                        and abs(max(xs) - max(q[0] for q in lands)) <= step
                        and abs(min(ys) - min(q[1] for q in lands)) <= step
                        and abs(max(ys) - max(q[1] for q in lands)) <= step)
                ok = bool(cells) and inside and span
            check('16. landings match the router\'s cells: %s %sx%s rot %s'
                  % (shape, sx, sy, rr), ok)
        # 16b -- the end-of-run reconciliation hands the router
        # connectivity._EndpointStub terminals: zero size and no `shape`.
        # They must yield no cells, not raise (a raise skipped watchy's
        # final reconciliation and changed its copper).
        from connectivity import _EndpointStub
        try:
            stub_cells = _free_on_pad_cells(_EndpointStub(10.0, 10.0, 'F.Cu'), 0,
                                            cfg, free_obs, coord)
            check('16b. an _EndpointStub terminal yields no cells',
                  stub_cells == [], str(stub_cells))
        except Exception as e:
            check('16b. an _EndpointStub terminal yields no cells', False,
                  '%s: %s' % (type(e).__name__, e))

        # 17 -- two keep-out areas 0.2 mm apart act TOGETHER: each alone
        # leaves the pad a landing, the gap between them does not
        def _two(kos):
            fp_ = NS(reference='U1', pads=[_npad('1', 0, 0, 1, 'rect', 0.0,
                                                 2.0, 0.5)],
                     x=0.0, y=0.0, rotation=0.0, layer='F.Cu')
            fq_ = NS(reference='U2', pads=[_npad('1', 0, 30, 1, 'rect')],
                     x=0, y=30, rotation=0.0, layer='F.Cu')
            pcb_ = NS(board_info=NS(copper_layers=['F.Cu', 'B.Cu'],
                                    keepouts=kos),
                      footprints={'U1': fp_, 'U2': fq_}, zones=[])
            k_ = legality.RuleAreaKeepouts(pcb_, 0.2, 0.15)
            return k_.part_amount('U1', legality.PartPads(fp_, 0.2)
                                  .pad_rects(0, 0, 0))

        def _half(poly):
            return {'polygon': poly, 'holes': [], 'layers': {'F.Cu'},
                    'tracks_allowed': False, 'vias_allowed': True,
                    'copper_pour_allowed': True, 'in_footprint': False}
        A_ = _half([(-10, -10), (-0.1, -10), (-0.1, 10), (-10, 10)])
        B_ = _half([(0.1, -10), (10, -10), (10, 10), (0.1, 10)])
        check('17. two adjacent areas: each alone 0, together illegal',
              _two([A_]) == 0 and _two([B_]) == 0 and _two([A_, B_]) > 0.1,
              str((_two([A_]), _two([B_]), _two([A_, B_]))))

        # 18 -- a through-hole pad escapes only on a layer a track can USE:
        # an uncovered OUTER layer, or an inner one no foreign-net zone
        # covers. J1 (/E) sits in the F&B band; foreign /A planes on In1
        # and In2 over it leave no escape, one plane leaves In2.
        def _tht_kind(extra, name):
            p_ = os.path.join(work, name + '.kicad_pcb')
            with open(p_, 'w', encoding='utf-8') as fh:
                fh.write(board_text(default_parts() + extra))
            gg = grade_pad_legality(parse_kicad_pcb(p_), 0.2, pcb_file=p_)
            return ('illegal' if any(r[0] == 'J1'
                                     for r in gg['keepout_copper_pads'])
                    else 'tht' if any(r[0] == 'J1'
                                      for r in gg['keepout_copper_tht_refs'])
                    else 'none')
        both = [zone(1, 0, 3, 3, 9, 'In1.Cu', 'zi1'),
                zone(1, 0, 3, 3, 9, 'In2.Cu', 'zi2')]
        check('18. J1 with foreign planes on BOTH inner layers fails like SMD',
              _tht_kind(both, 'tht_planes') == 'illegal')
        check('18. ...with one inner layer free it is reported, not failed',
              _tht_kind(both[:1], 'tht_one_plane') == 'tht')

        # 19 -- inherited band pads: render_placement --before carries the
        # before board's own census, and the driver judges NEW copper
        # against it (a human reference board is not refused for its own
        # pads), absolute without it
        rj5 = os.path.join(work, 'rp_before.json')
        run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'render_placement.py'),
                   bd, '--before', bd, '--clearance', '0.2', '--json-out',
                   rj5, '-o', os.path.join(work, 'rp_before.png')],
                  accept=True)
        c5 = json.load(open(evidence(rj5), encoding='utf-8'))['checklist'][
            'a_off_outline']
        check('19. --before carries the before board\'s keep-out census',
              _pairs(c5.get('keepout_copper_before') or [])
              == _pairs(c5['keepout_copper']) and c5['keepout_copper'],
              str(c5.get('keepout_copper_before')))
        rdoc6 = json.load(open(da.render_json, encoding='utf-8'))
        for label, before, refused in (
                ('inherited, no deeper', [['R1', 0.5]], False),
                ('inherited but DEEPER now', [['R1', 0.1]], True),
                ('no --before census (absolute)', None, True)):
            rdoc6['checklist']['a_off_outline'] = {
                'pad_copper': [], 'courtyard': [],
                'keepout_copper': [['R1', 0.3]],
                'keepout_copper_before': before}
            rj6 = os.path.join(ftmp, 'r_before_%d.json' % int(refused))
            with open(rj6, 'w', encoding='utf-8') as fh:
                json.dump(rdoc6, fh)
            a6 = pdrv._args(dargv + ['--waive', 'X:checked'])
            a6.render_json = rj6
            o6 = pdrv.STAGES['P-close'](a6)
            got = 'seat pads inside a rule-area KEEP-OUT band' in o6
            check('19. P-close keep-out arm: %s -> %s'
                  % (label, 'refused' if refused else 'passes'),
                  got == refused, o6[:200])

        # 7 -- inert without a keep-out
        pcb0 = parse_kicad_pcb(clean)
        g0 = grade_pad_legality(pcb0, 0.2, pcb_file=clean)
        ko0 = legality.RuleAreaKeepouts.for_board(pcb0, 0.2, clean)
        ctx0 = legality.LegalityContext(parts, None, 0.2,
                                        pose_of=lambda r: seeds[r],
                                        seed_of=lambda r: seeds[r],
                                        keepouts=ko0)
        check('7. no keep-out: count 0, nothing listed, context inert',
              g0['oob_keepout_copper_count'] == 0
              and not g0['keepout_copper_pads']
              and not g0['keepout_copper_tht_refs']
              and ctx0.keepouts is None and not ko0.active)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    if FAILS:
        print('\nFAILED: %d check(s): %s' % (len(FAILS), FAILS))
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
