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
    ]


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
