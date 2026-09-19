#!/usr/bin/env python3
"""#959 / #998: the zone plan checked against itself BEFORE the first pose.

Run 29 found its zone plan's three ERRORs at lap 5, because the one caller of
the self-consistency check was `grade`, which place_seed runs after writing
the seed. `floorplan.plan_check` runs the checks that need no pose of a
movable part, and P1 / place_seed refuse on its ERRORs.

The bar every ERROR here has to clear is SOUNDNESS: a satisfiable plan is
never refused. So the tests carry the counterexamples the plan reviewers
measured against the first drafts:

  * run 29's own lap-5 plan (recovered from the session record) overlaps by
    1.60 and 0.6325 mm2 and its board satisfied both zones -- WARN, not ERROR;
  * a tight zone around ulx3s U9 / orangecrab J4 holding the parts UNDER them
    on the other face -- satisfied by the shipped boards, so it must pass the
    per-face area bound;
  * sonde_u J1, a DSUB whose courtyard is longer than its edge because its
    flange overhangs both corners -- a shipping board, so the edge bound must
    read PADS, not courtyards;
  * an exclusive overlap the member can still avoid -- satisfiable.

And the corpus control: `plan_check` on every tracked board's emitted intent
refuses nothing.
"""
import hashlib
import json
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _sub in ('', 'py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(REPO, _sub) if _sub else REPO
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402

RUN_ALL_TIMEOUT = 1800

FIX = os.path.join(REPO, 'tests', 'fixtures', '959')
PILE = os.path.join(FIX, 'run29_pile.kicad_pcb')
LAP5 = os.path.join(FIX, 'zone_plan_r2_lap5.json')
ESP = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
PLAN_975 = os.path.join(REPO, 'tests', 'fixtures', '975', 'esp_prog_run27',
                        'zone_plan.json')
DRIVER = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement',
                      'scripts', 'placement_driver.py')


def _raw(**extra):
    d = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'}
    d.update(extra)
    return d


def _board(tmp, name, parts, w=20, h=10):
    """A minimal board: an outline and one footprint per (ref, x, y, pads),
    each pad `(dx, dy, sx, sy)` in the footprint's frame."""
    fps = []
    for ref, x, y, pads, *rest in parts:
        locked = ' (locked yes)' if rest and rest[0] else ''
        body = ''.join(
            f'    (pad "{i + 1}" smd rect (at {dx} {dy}) (size {sx} {sy}) '
            f'(layers "F.Cu") (net 1 "/A") (uuid "p{i}-{ref}"))\n'
            for i, (dx, dy, sx, sy) in enumerate(pads))
        fps.append(f'  (footprint "t:FP" (layer "F.Cu") (uuid "fp-{ref}") '
                   f'(at {x} {y}){locked}\n'
                   f'    (property "Reference" "{ref}" (at 0 0))\n{body}  )\n')
    path = os.path.join(tmp, name)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "test")\n'
                 '  (net 0 "")\n  (net 1 "/A")\n'
                 f'  (gr_rect (start 0 0) (end {w} {h}) (layer "Edge.Cuts") '
                 '(uuid "e1"))\n' + ''.join(fps) + ')\n')
    return path


def _check(intent_raw, board):
    it = fp.intent_from_dict(intent_raw, '')
    found, measured = fp.plan_check(it, parse_kicad_pcb(board), board)
    return found, measured


def subprocess_run(argv):
    import subprocess
    return subprocess.run(argv, capture_output=True, text=True,
                          encoding='utf-8', errors='replace',
                          cwd=REPO, timeout=900)


def _errors(found):
    return [v for v in found if v.severity == 'error']


PAD = [(0, 0, 0.6, 0.8)]
#: What P1 and check_floorplan resolve blocks with. Without them an emitted
#: sheet-group block resolves to nothing -- which is how this control first
#: caught P1 passing none.
SOURCES = ('kicad', 'sheet')


def test_run29_lap5_overlaps_are_warnings_not_errors():
    found, _ = _check(json.load(open(LAP5, encoding='utf-8')), PILE)
    ov = [v for v in found if v.rule == 'intent_zone_overlap']
    areas = sorted(round(v.measured['overlap_area_mm2'], 4) for v in ov)
    assert areas == [0.6325, 1.6], areas
    assert all(v.severity == 'warn' for v in ov), ov
    assert 'may be placed there' in ov[0].message, ov[0].message
    assert not _errors(found), _errors(found)
    print("  PASS: run 29's lap-5 overlaps (1.60, 0.6325 mm2) are WARNs; "
          "the plan has no error")


def _fab_board(tmp):
    """The Phase-4 verifier's synthetic board: P1 draws no courtyard (a
    4x4 pad box is what the grade reads) but a 4x12 .Fab body (what the
    occupancy reads); M1 is a small part with a courtyard."""
    def fpblock(ref, x, y, pads, fab=None, crt=None):
        body = ''.join(
            f'    (pad "{i + 1}" smd rect (at {dx} {dy}) (size {sx} {sy}) '
            f'(layers "F.Cu") (net 1 "/A") (uuid "p{i}-{ref}"))\n'
            for i, (dx, dy, sx, sy) in enumerate(pads))
        g = ''
        for layer, r in (('F.Fab', fab), ('F.CrtYd', crt)):
            if r:
                g += (f'    (fp_rect (start {r[0]} {r[1]}) (end {r[2]} '
                      f'{r[3]}) (stroke (width 0.1) (type solid)) (fill '
                      f'none) (layer "{layer}") (uuid "{layer}-{ref}"))\n')
        return (f'  (footprint "t:FP" (layer "F.Cu") (uuid "fp-{ref}") '
                f'(at {x} {y})\n    (property "Reference" "{ref}" (at 0 0))'
                f'\n{g}{body}  )\n')
    path = os.path.join(tmp, 'fab.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "test")\n'
                 '  (net 0 "")\n  (net 1 "/A")\n'
                 '  (gr_rect (start 0 0) (end 40 40) (layer "Edge.Cuts") '
                 '(uuid "e1"))\n'
                 + fpblock('P1', 10, 10, [(-1.5, -1.5, 1, 1),
                                          (1.5, 1.5, 1, 1)],
                           fab=(-2, -6, 2, 6))
                 + fpblock('M1', 10, 25, [(0, 0, 1, 1)], crt=(-1, -1, 1, 1))
                 + ')\n')
    return path


def test_the_exclusive_check_reads_the_grades_geometry():
    """Phase-4 verifier B1: the check read `part_local_bounds` (occupancy,
    .Fab included), the grade reads the courtyard / pad box, so a board every
    zone rule passes failed its own grade. And the pile rotation (SF6), the
    side filter (M4) and the zone's own members (M3)."""
    with tempfile.TemporaryDirectory() as tmp:
        b = _fab_board(tmp)
        blocks = [{'name': 'B', 'refs': ['P1'],
                   'zone': [7.5, 7.5, 12.5, 20.5], 'tolerance_mm': 0},
                  {'name': 'A', 'refs': ['M1'], 'zone': [5, 13, 15, 30],
                   'exclusive': True, 'tolerance_mm': 0}]
        it = fp.intent_from_dict(_raw(blocks=blocks), '')
        res = fp.grade(it, parse_kicad_pcb(b), b)
        assert not res.errors, res.errors
        found, _ = _check(_raw(blocks=blocks), b)
        assert not [v for v in found
                    if v.rule == 'plan_zone_exclusive_unsatisfiable'], found
        ov = [v for v in found if v.rule == 'intent_zone_overlap']
        assert ov and "EXCLUSIVE zone 'A'" in ov[0].message, ov
        # Made genuinely infeasible (B entirely inside A) it IS the ERROR --
        # unless A is on the other face (M4) or P1 is also A's member (M3).
        inside = [dict(blocks[0], zone=[8, 14, 12, 29]), blocks[1]]
        found, _ = _check(_raw(blocks=inside), b)
        assert [v.ref for v in found
                if v.rule == 'plan_zone_exclusive_unsatisfiable'] == ['P1']
        found, _ = _check(_raw(blocks=[inside[0], dict(inside[1],
                                                        side='B')]), b)
        assert not [v for v in found
                    if v.rule == 'plan_zone_exclusive_unsatisfiable'], found
        found, _ = _check(_raw(blocks=[inside[0], dict(
            inside[1], refs=['M1', 'P1'])]), b)
        assert not [v for v in found
                    if v.rule == 'plan_zone_exclusive_unsatisfiable'], found
        # A part on the pile at 45 degrees has the 0-degree lattice too.
        text = open(b, encoding='utf-8').read().replace(
            '(uuid "fp-P1") (at 10 10)', '(uuid "fp-P1") (at 10 10 45)')
        open(b, 'w', encoding='utf-8').write(text)
        tight = [dict(blocks[0], zone=[7.9, 7.9, 12.1, 12.6]), blocks[1]]
        found, _ = _check(_raw(blocks=tight), b)
        assert not [v for v in found
                    if v.rule == 'plan_zone_exclusive_unsatisfiable'], found
    print("  PASS: the grade's geometry; nested-inside is the ERROR; the "
          "other face, a shared member and a 45-degree pile rotation are "
          "not")


def test_exclusive_infeasibility_is_the_one_error():
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(tmp, 'x.kicad_pcb', [('U1', 3, 3, PAD), ('U2', 6, 3, PAD)])
        blocks_bad = [
            {'name': 'A', 'refs': ['U1'], 'zone': [0, 0, 10, 10],
             'exclusive': True},
            {'name': 'B', 'refs': ['U2'], 'zone': [5, 2, 7, 4],
             'tolerance_mm': 0}]
        found, _ = _check(_raw(blocks=blocks_bad), b)
        e = [v for v in found
             if v.rule == 'plan_zone_exclusive_unsatisfiable']
        assert e and e[0].severity == 'error' and e[0].ref == 'U2', found
        # grade raises it too
        it = fp.intent_from_dict(_raw(blocks=blocks_bad), '')
        res = fp.grade(it, parse_kicad_pcb(b), b)
        assert any(v.rule == 'plan_zone_exclusive_unsatisfiable'
                   for v in res.errors), res.violations
        # satisfiable: B reaches past A, so U2 has room outside it
        blocks_ok = [dict(blocks_bad[0]),
                     {'name': 'B', 'refs': ['U2'], 'zone': [8, 2, 14, 4],
                      'tolerance_mm': 0}]
        found, _ = _check(_raw(blocks=blocks_ok), b)
        assert not [v for v in found
                    if v.rule == 'plan_zone_exclusive_unsatisfiable'], found
        assert any(v.rule == 'intent_zone_overlap' and v.severity == 'warn'
                   for v in found), found
    print("  PASS: an exclusive zone a member cannot avoid is an ERROR (in "
          "plan_check and grade); one it can avoid is only an overlap WARN")


def test_a_literal_glob_is_an_error_only_when_it_double_zones():
    pcb = parse_kicad_pcb(ESP)
    fid = pcb.footprints['Ref*']
    fid2 = pcb.footprints['Ref*~2']
    raw = _raw(blocks=[
        {'name': 'fid', 'refs': ['Ref*'],
         'zone': [fid.x - 2, fid.y - 2, fid.x + 2, fid.y + 2]},
        {'name': 'fid2', 'refs': ['Ref[*]~2'],
         'zone': [fid2.x - 2, fid2.y - 2, fid2.x + 2, fid2.y + 2]}])
    found, _ = _check(raw, ESP)
    g = [v for v in found if v.rule == 'block_glob_literal']
    assert g and g[0].severity == 'error', found
    assert "'Ref[*]'" in g[0].message, g[0].message
    # Nested zones are satisfiable: the board-wide `Ref*` block and a tight
    # `Ref[*]~2` one share area, so Ref*~2 can sit in both (SF1).
    nested = _raw(blocks=[
        {'name': 'all', 'refs': ['Ref*'], 'zone': [0, 0, 400, 400]},
        {'name': 'fid2', 'refs': ['Ref[*]~2'],
         'zone': [fid2.x - 2, fid2.y - 2, fid2.x + 2, fid2.y + 2]}])
    found, _ = _check(nested, ESP)
    assert not [v for v in found if v.rule == 'block_glob_literal'
                and v.severity == 'error'], found
    # One zone only: the over-match puts Ref*~2 in no second zone, so it is
    # a lint, not a contradiction (M7) ...
    single = _raw(blocks=[{'name': 'fid', 'refs': ['Ref*'],
                           'zone': [fid.x - 2, fid.y - 2, fid.x + 2,
                                    fid.y + 2]}])
    found, _ = _check(single, ESP)
    g = [v for v in found if v.rule == 'block_glob_literal']
    assert [v.severity for v in g] == ['warn'], g
    # ... and a list that also names the swept block meant it (M8).
    named = _raw(blocks=[{'name': 'fid', 'refs': ['Ref*', 'Ref[*]~2'],
                          'zone': [fid.x - 2, fid.y - 2, fid.x + 2,
                                   fid.y + 2]}])
    found, _ = _check(named, ESP)
    assert not [v for v in found if v.rule == 'block_glob_literal'], found
    # Fixture 975's must_lock names both blocks: the over-match is intended.
    found, _ = _check(json.load(open(PLAN_975, encoding='utf-8')), ESP)
    g = [v for v in found if v.rule == 'block_glob_literal']
    assert not [v for v in g if v.severity == 'error'], g
    print("  PASS: `Ref*` putting Ref*~2 in a second zone is an ERROR; "
          "fixture 975's intended over-match is not")


def test_a_locked_member_outside_its_zone():
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(tmp, 'l.kicad_pcb', [('U1', 3, 3, PAD, True),
                                        ('U2', 6, 3, PAD)])
        # A plan that demoted zone_containment is not refused for it (SF2).
        demoted = _raw(blocks=[{'name': 'far', 'refs': ['U1'],
                                'zone': [10, 5, 12, 7], 'tolerance_mm': 0}],
                       severity={'zone_containment': 'warn'})
        found, _ = _check(demoted, b)
        f = [v for v in found if v.rule == 'plan_fixed_outside_zone']
        assert f and f[0].severity == 'warn', found
        raw = _raw(blocks=[{'name': 'far', 'refs': ['U1'],
                            'zone': [14, 5, 18, 9]}])
        found, _ = _check(raw, b)
        v = [x for x in found if x.rule == 'plan_fixed_outside_zone']
        assert v and v[0].ref == 'U1' and v[0].severity == 'error', found
        # unlocked, it is the seeder's to move: no finding
        b2 = _board(tmp, 'u.kicad_pcb', [('U1', 3, 3, PAD),
                                         ('U2', 6, 3, PAD)])
        found, _ = _check(raw, b2)
        assert not [x for x in found
                    if x.rule == 'plan_fixed_outside_zone'], found
    print("  PASS: a LOCKED member outside its zone is named; an unlocked "
          "one is not")


def _tight_zone_around(pcb, board, ref, margin=1.0):
    """A zone of `ref`'s rect + margin holding every other-face part fully
    under it -- the arrangement the shipped board already has."""
    import pose_score
    st = pose_score.make_state(pcb, board)
    r = st.parts[ref].rect()
    zone = [r[0] - margin, r[1] - margin, r[2] + margin, r[3] + margin]
    side = st.parts[ref].side
    under = [k for k, p in st.parts.items()
             if k != ref and p.side != side
             and p.rect()[0] >= zone[0] and p.rect()[1] >= zone[1]
             and p.rect()[2] <= zone[2] and p.rect()[3] <= zone[3]]
    return zone, [ref] + under


def test_two_locked_parts_that_overlap():
    """Row 8. Locked parts do not move, so their overlap is in every
    placement. A WARN per pair -- the grade counts courtyard overlap only
    against a declared budget, and run 29's shipped board carried a 1.0 mm2
    fiducial-in-connector overlap -- and an ERROR only when the locked pairs
    ALONE exceed a declared `overlap_area`."""
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(tmp, 'f.kicad_pcb', [('U1', 3, 3, PAD, True),
                                        ('U2', 3.3, 3, PAD, True),
                                        ('U3', 3.1, 3, PAD)])
        found, meas = _check(_raw(), b)
        pairs = [v for v in found if v.rule == 'plan_fixed_overlap']
        assert len(pairs) == 1 and pairs[0].severity == 'warn', found
        assert pairs[0].measured['pair'] == ['U1', 'U2'], pairs[0]
        # 0.3 mm x 0.8 mm of shared pad bbox, and U3 (unlocked) is not in it.
        assert abs(meas['fixed_overlap']['total_mm2'] - 0.24) < 1e-6, meas
        assert not [v for v in found if v.rule == 'plan_fixed_overlap_budget']
        found, _ = _check(_raw(legality_budget={'overlap_area': 0.2}), b)
        err = [v for v in found if v.rule == 'plan_fixed_overlap_budget']
        assert err and err[0].severity == 'error', found
        # As strong as `legality`: demoted, the grade passes this overlap,
        # so the plan finding is a WARN (Phase-7 fact-check).
        found, _ = _check(_raw(legality_budget={'overlap_area': 0.2},
                               severity={'legality': 'warn'}), b)
        err = [v for v in found if v.rule == 'plan_fixed_overlap_budget']
        assert err and err[0].severity == 'warn', found
        found, _ = _check(_raw(legality_budget={'overlap_area': 0.3}), b)
        assert not [v for v in found
                    if v.rule == 'plan_fixed_overlap_budget'], found
    print("  PASS: locked U1/U2 overlap 0.24 mm2 -> a WARN; over a declared "
          "0.2 budget -> an ERROR; under 0.3 -> none; unlocked U3 ignored")


def test_the_area_bound_is_per_face_and_passes_shipped_boards():
    for name, ref in (('ulx3s', 'U9'), ('ulx3s', 'U2'),
                      ('orangecrab_ext_pll', 'J4')):
        board = os.path.join(REPO, 'kicad_files', f'{name}.kicad_pcb')
        pcb = parse_kicad_pcb(board)
        zone, members = _tight_zone_around(pcb, board, ref)
        assert len(members) > 1, (name, ref, members)
        raw = _raw(blocks=[{'name': 'tight', 'refs': members, 'zone': zone,
                            'tolerance_mm': 0}])
        found, meas = _check(raw, board)
        bad = [v for v in found if v.rule == 'plan_zone_overfull']
        assert not bad, (name, ref, bad)
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(tmp, 'o.kicad_pcb', [('U1', 3, 3, PAD), ('U2', 6, 3, PAD)])
        blocks = [{'name': 'all', 'refs': ['U*'],
                   'zone': [1.5, 1.5, 2.4, 2.4], 'tolerance_mm': 0}]
        # 0.96 mm2 into 0.81 forces >= 0.15 mm2 of courtyard overlap: an
        # ERROR against a declared budget of 0.1, not against 0.2 ...
        for budget, want in ((0.1, 'error'), (0.2, None)):
            found, _ = _check(_raw(blocks=blocks, legality_budget={
                'overlap_area': budget}), b)
            v = [x for x in found if x.rule == 'plan_zone_overfull']
            assert [x.severity for x in v] == ([want] if want else []), (
                budget, found)
        assert abs(v[0].measured['members_area_mm2'] - 0.96) < 1e-6 \
            if v else True
        # ... and with no budget nothing bounds the overlap: the WARN.
        found, _ = _check(_raw(blocks=blocks), b)
        assert not [x for x in found if x.rule == 'plan_zone_overfull']
        c = [x for x in found if x.rule == 'plan_zone_crowded']
        assert c and c[0].measured['forced_overlap_mm2'] > 0.14, found
        # 1.05 mm2 holds 0.96 by courtyard but not with 0.25 mm around
        # each part ((0.85 x 1.05) x 2 = 1.785): the WARN half.
        raw = _raw(blocks=[{'name': 'all', 'refs': ['U*'],
                            'zone': [1.5, 1.5, 2.55, 2.5],
                            'tolerance_mm': 0}])
        found, _ = _check(raw, b)
        assert not [x for x in found if x.rule == 'plan_zone_overfull']
        c = [x for x in found if x.rule == 'plan_zone_crowded']
        assert c and c[0].severity == 'warn', found
    print("  PASS: ulx3s U9/U2 and orangecrab J4 with the parts under them "
          "pass the per-face bound; 0.96 mm2 in a 0.81 mm2 zone is an ERROR "
          "only past a declared overlap budget")


def test_the_area_bound_is_sound_for_the_grade():
    """The Phase-4 verifier's counterexamples: shipped boards whose tight
    zones the grade SATISFIES were refused, because the bound charged locked
    parts and ignored that the grade allows declared overlap. And the
    skips that keep false ERRORs out, each pinned (M4, M12, M13 survived)."""
    glas = os.path.join(REPO, 'kicad_files', 'glasgow_revC.kicad_pcb')
    gpcb = parse_kicad_pcb(glas)
    # Every member counts, locked ones too -- the grade counts a locked
    # pair's overlap -- so glasgow's MK1 + FID1 read as the board has them:
    # clean at the board's own budget, an ERROR at 0 (they DO overlap).
    own = fp.emit_intent(gpcb, glas)['legality_budget'].get('overlap_area')
    for budget, want in ((own, []), (0, ['error'])):
        if budget is None:
            continue
        raw = _raw(blocks=[{'name': 'mk', 'refs': ['MK1', 'FID1'],
                            'zone': [49, 111, 59, 121], 'tolerance_mm': 0}],
                   legality_budget={'overlap_area': budget})
        found, meas = _check(raw, glas)
        got = [v.severity for v in found if v.rule == 'plan_zone_overfull']
        assert got == want, (budget, got, meas['zones'])
    # ulx3s GPDI1 in the zone of its own courtyard, with the B-side parts
    # under its lead field (the verifier's counterexample): the far-face
    # charge is real overlap the grade counts, so at budget 0 it is an
    # ERROR on B (M11), and a budget covering the forced overlap clears it.
    ul = os.path.join(REPO, 'kicad_files', 'ulx3s.kicad_pcb')
    pcb = parse_kicad_pcb(ul)
    zone, members = _tight_zone_around(pcb, ul, 'GPDI1', margin=0.0)
    blk = [{'name': 'gpdi', 'refs': members, 'zone': zone,
            'tolerance_mm': 0}]
    found, meas = _check(_raw(blocks=blk,
                              legality_budget={'overlap_area': 0}), ul)
    err = [v for v in found if v.rule == 'plan_zone_overfull']
    assert err and err[0].measured['face'] == 'B', (found, meas['zones'])
    forced = err[0].measured['forced_overlap_mm2']
    found, _ = _check(_raw(blocks=blk, legality_budget={
        'overlap_area': forced + 0.01}), ul)
    assert not [v for v in found if v.rule == 'plan_zone_overfull'], (
        forced, found)
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(tmp, 'w.kicad_pcb', [('U1', 3, 3, PAD), ('U2', 6, 3, PAD)])
        blocks = [{'name': 'all', 'refs': ['U*'],
                   'zone': [1.5, 1.5, 2.4, 2.4], 'tolerance_mm': 0}]
        # A waived pair's zone is skipped (M13).
        found, _ = _check(_raw(blocks=blocks,
                               legality_budget={'overlap_area': 0},
                               overlap_waivers=[{'pair': ['U1', 'U2'],
                                                 'reason': 'a stacked pair'}]),
                          b)
        assert not [v for v in found if v.rule.startswith('plan_zone_')], (
            found)
        # A zone smaller than a member's courtyard grades it on its centre,
        # so it is not charged (M12).
        found, meas = _check(_raw(blocks=[{'name': 'pin', 'refs': ['U1'],
                                           'zone': [2.9, 2.9, 3.1, 3.1],
                                           'tolerance_mm': 0}],
                                  legality_budget={'overlap_area': 0}), b)
        row = meas['zones'][0]
        assert row['members'] == [] and not [
            v for v in found if v.rule == 'plan_zone_overfull'], (row, found)
    print("  PASS: glasgow's locked MK1+FID1 pass at the board's own "
          "budget; GPDI1's far face is an ERROR at budget 0 and not at a "
          "covering budget; waived and anchor-graded members are skipped")


def test_the_edge_bound_reads_pads_not_courtyards():
    board = os.path.join(REPO, 'kicad_files', 'sonde_u.kicad_pcb')
    raw = _raw(edge_connectors=[{'ref': 'J1', 'edge': 'west',
                                 'class': 'edge_receptacle'}])
    found, meas = _check(raw, board)
    assert not [v for v in found if v.rule == 'plan_edge_overfull'], found
    with tempfile.TemporaryDirectory() as tmp:
        # A 13 x 13 pad array: 12.6 x 12.8 mm whichever way it is turned, on
        # a 10 mm edge. (A single ROW would turn 90 degrees and fit -- the
        # bound is per part at its best rotation, which is what makes it
        # sound.)
        block = [(dx, dy, 0.6, 0.8) for dx in range(13) for dy in range(13)]
        b = _board(tmp, 'e.kicad_pcb', [('J1', 2, 2, block),
                                        ('U1', 18, 5, PAD)], w=40, h=10)
        raw = _raw(edge_connectors=[{'ref': 'J1', 'edge': 'west',
                                     'class': 'edge_receptacle'}])
        found, _ = _check(raw, b)
        v = [x for x in found if x.rule == 'plan_edge_overfull']
        assert v and v[0].ref == 'J1', found
        # A 9 x 9 array (8.6 mm) turned 45 degrees measures 11.9 mm in the
        # board frame; in its own frame it fits a 10 mm edge.
        nine = [(dx, dy, 0.6, 0.6) for dx in range(9) for dy in range(9)]
        b = _board(tmp, 'r.kicad_pcb', [('J1', 5, 5, nine),
                                        ('U1', 30, 5, PAD)], w=40, h=10)
        text = open(b, encoding='utf-8').read().replace(
            '(uuid "fp-J1") (at 5 5)', '(uuid "fp-J1") (at 5 5 45)')
        open(b, 'w', encoding='utf-8').write(text)
        found, _ = _check(raw, b)
        assert not [x for x in found if x.rule == 'plan_edge_overfull'], (
            found)
        # Two claimed parts that each fit the edge but not side by side:
        # the WARN, never the ERROR (M20).
        six = [(dx, dy, 0.6, 0.6) for dx in range(7) for dy in range(7)]
        b = _board(tmp, 'two.kicad_pcb', [('J1', 5, 3, six),
                                          ('J2', 5, 7, six),
                                          ('U1', 30, 5, PAD)], w=40, h=10)
        raw2 = _raw(edge_connectors=[
            {'ref': 'J1', 'edge': 'west', 'class': 'edge_receptacle'},
            {'ref': 'J2', 'edge': 'west', 'class': 'edge_receptacle'}])
        found, meas = _check(raw2, b)
        assert not [x for x in found if x.rule == 'plan_edge_overfull']
        w = [x for x in found if x.rule == 'plan_edge_crowded']
        assert w and w[0].severity == 'warn', (found, meas['edges'])
    print("  PASS: sonde_u's DSUB passes (pads fit, its flange overhangs); a "
          "12.6 x 12.8 mm pad array on a 10 mm edge does not; a 45-degree "
          "part is measured in its own frame")


def test_the_board_area_bound_is_per_face():
    for name in ('orangecrab_ext_pll', 'ulx3s'):
        board = os.path.join(REPO, 'kicad_files', f'{name}.kicad_pcb')
        found, meas = _check(_raw(), board)
        assert not [v for v in found if v.rule == 'plan_board_overfull'], (
            name, meas)
    with tempfile.TemporaryDirectory() as tmp:
        big = [(dx, dy, 0.9, 0.9) for dx in range(4) for dy in range(4)]
        parts = [(f'U{i}', 2 + 4 * (i % 4), 2 + 4 * (i // 4) % 8, big)
                 for i in range(16)]
        b = _board(tmp, 'b.kicad_pcb', parts)
        found, _ = _check(_raw(legality_budget={'overlap_area': 0,
                                                'oob_count': 0}), b)
        e = [v for v in found if v.rule == 'plan_board_overfull']
        assert e and e[0].measured['utilisation'] > 1.0, found
        assert 'None' not in e[0].message, e[0].message
        # No budget: the WARN, which says the number (it said "None").
        found, meas = _check(_raw(), b)
        assert not [v for v in found if v.rule == 'plan_board_overfull']
        w = [v for v in found if v.rule == 'plan_board_crowded']
        assert w and 'None' not in w[0].message, found
        assert meas['utilisation_per_face_clearance0'] > 1.0, meas
    print("  PASS: orangecrab and ulx3s fit per face; 16 x 14.4 mm2 on a "
          "200 mm2 board is an ERROR past a declared budget, a WARN with "
          "none, and both print the utilisation")


def test_place_seed_refuses_before_writing_and_repair_only_reports():
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(tmp, 'o.kicad_pcb', [('U1', 3, 3, PAD), ('U2', 6, 3, PAD)])
        plan = os.path.join(tmp, 'p.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump(_raw(blocks=[{'name': 'all', 'refs': ['U*'],
                                    'zone': [1.5, 1.5, 2.4, 2.4],
                                    'tolerance_mm': 0}],
                           legality_budget={'overlap_area': 0}), fh)
        out = os.path.join(tmp, 'seed.kicad_pcb')
        before = hashlib.sha256(open(b, 'rb').read()).hexdigest()
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('place_seed.py'), b, out,
                             '--intent', plan],
                            refuse='the zone plan is refused before anything '
                                   'is written', code=5)
        assert not os.path.exists(out), 'a refused plan still wrote a seed'
        assert hashlib.sha256(open(b, 'rb').read()).hexdigest() == before
        line = [x for x in r.stdout.splitlines()
                if x.startswith('JSON_SUMMARY:')][-1]
        s = json.loads(line.split('JSON_SUMMARY: ', 1)[1])
        assert s['refused'] == 'plan_check' and s['exit_code'] == 5, s
        assert s['output'] is None and s['written'] is False, s
        # compare_seeds says it once, names the cause, and ranks nothing.
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('compare_seeds.py'), b,
                             '--intent', plan, '--seeds', '0', '1', '2',
                             '--out-dir', os.path.join(tmp, 'cmp')],
                            refuse='the zone plan was refused before any '
                                   'seed was written', code=4)
        assert r.stdout.count('place_seed exit 5') == 1, r.stdout
    # --repair works from a PLACEMENT, not a plan: it reports and proceeds.
    with tempfile.TemporaryDirectory() as tmp:
        pcb = parse_kicad_pcb(ESP)
        u1 = pcb.footprints['U1']
        plan = os.path.join(tmp, 'p.json')
        # C2, R1, R2 are 1.516 x 0.55 mm each: every one fits a 1.6 x 1.2
        # zone alone, and the three need 2.50 mm2 of its 1.92.
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump(_raw(blocks=[{'name': 'tiny',
                                    'refs': ['C2', 'R1', 'R2'],
                                    'zone': [u1.x, u1.y, u1.x + 1.6,
                                             u1.y + 1.2],
                                    'tolerance_mm': 0}],
                           legality_budget={'overlap_area': 0}), fh)
        r = subprocess_run([sys.executable, '-X', 'utf8',
                            run_utils.tool('place_seed.py'), ESP,
                            os.path.join(tmp, 'out.kicad_pcb'), '--intent',
                            plan, '--repair', '--dry-run'])
        assert 'PLAN [ERROR] plan_zone_overfull' in r.stdout, r.stdout[-1500:]
        assert r.returncode != 5, (r.returncode, r.stdout[-800:])
        assert 'Traceback' not in r.stdout + r.stderr
    # The refusal set is exactly `PLAN_SEED_REFUSES`: a zone a keep-out
    # covers is an ERROR that P1 refuses, and place_seed PRINTS it and seeds,
    # so the seeder can name the member it could not seat (#701's contract).
    assert fp.PLAN_SEED_REFUSES == {
        'plan_zone_overfull', 'plan_edge_overfull', 'plan_board_overfull',
        'block_glob_literal'}, fp.PLAN_SEED_REFUSES
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(tmp, 'k.kicad_pcb', [('U1', 3, 3, PAD)])
        plan = os.path.join(tmp, 'p.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump(_raw(blocks=[{'name': 'b', 'refs': ['U1'],
                                    'zone': [2.0, 2.0, 5.0, 5.0],
                                    'tolerance_mm': 0}],
                           keepouts=[{'name': 'hot',
                                      'rect': [1.0, 1.0, 6.0, 6.0],
                                      'sides': ['F', 'B']}]), fh)
        r = subprocess_run([sys.executable, '-X', 'utf8',
                            run_utils.tool('place_seed.py'), b,
                            os.path.join(tmp, 'seed.kicad_pcb'),
                            '--intent', plan, '--force'])
        assert 'PLAN [ERROR] intent_zone_in_keepout' in r.stdout, \
            r.stdout[-1500:]
        assert r.returncode != 5, (r.returncode, r.stdout[-800:])
        assert 'Traceback' not in r.stdout + r.stderr
        assert os.path.exists(os.path.join(tmp, 'seed.kicad_pcb'))
    print("  PASS: place_seed exits 5 with nothing written and the source "
          "unchanged; compare_seeds says so once; --repair only reports; a "
          "keep-out-covered zone is printed and seeded, not refused")


def test_plan_only_on_a_pile():
    r = run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('check_floorplan.py'), PILE,
                         '--intent', LAP5, '--plan-only', '--no-mechanical'],
                        accept=True)
    line = [x for x in r.stdout.splitlines()
            if x.startswith('JSON_SUMMARY:')][-1]
    s = json.loads(line.split('JSON_SUMMARY: ', 1)[1])
    assert s['plan_only'] and s['plan_errors'] == 0, s
    assert s['plan_findings_by_rule'].get('intent_zone_overlap') == 2, s
    assert 'pending' in s['ledger_status'], s['ledger_status']
    assert 'decap_distance' in s['rules_dark_undispositioned'], s
    print("  PASS: --plan-only reads a pile, reports 2 overlap WARNs, the "
          "ledger's pending rows and what the plan owes")


def _true_pile(tmp):
    """esp_prog with EVERY footprint at the board centre -- a board
    `assess_placement` calls unplaced, which run 29's own pile is not (its
    mechanical refs sit at their source poses)."""
    from kicad_parser import iter_footprint_blocks
    pcb = parse_kicad_pcb(ESP)
    bx = pcb.board_info.board_bounds
    cx, cy = round((bx[0] + bx[2]) / 2, 3), round((bx[1] + bx[3]) / 2, 3)
    text = open(ESP, encoding='utf-8').read()
    for start, end, _t, _r, key in reversed(list(
            iter_footprint_blocks(text))):
        block = text[start:end]
        i = block.index('(at ')
        j = block.index(')', i)
        block = block[:i] + f'(at {cx} {cy}' + block[j:]
        text = text[:start] + block + text[end:]
    out = os.path.join(tmp, 'pile.kicad_pcb')
    with open(out, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return out


def test_plan_only_on_a_true_pile_and_its_exit_4():
    """Phase-4 verifier SF8: the pile fixture above is not one
    `assess_placement` calls unplaced, so the `--plan-only` allowance for an
    unplaced board was untested, and so was its exit 4."""
    with tempfile.TemporaryDirectory() as tmp:
        pile = _true_pile(tmp)
        plan = os.path.join(tmp, 'p.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump(_raw(blocks=[{'name': 'u', 'refs': ['U1'],
                                    'zone': [0, 0, 400, 400]}]), fh)
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('check_floorplan.py'), pile,
                         '--intent', plan, '--no-brief', '--no-mechanical'],
                        refuse='unplaced', code=3)
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), pile,
                             '--intent', plan, '--plan-only', '--no-brief',
                             '--no-mechanical'], accept=True)
        bad = os.path.join(tmp, 'bad.json')
        # C2, R1, R2 are 1.516 x 0.55 mm: each fits a 1.6 x 1.2 zone alone
        # (so none is anchor-graded), the three need 2.50 of its 1.92 mm2.
        c = parse_kicad_pcb(pile).footprints['C2']
        with open(bad, 'w', encoding='utf-8') as fh:
            json.dump(_raw(blocks=[{'name': 'tiny',
                                    'refs': ['C2', 'R1', 'R2'],
                                    'zone': [c.x, c.y, c.x + 1.6,
                                             c.y + 1.2],
                                    'tolerance_mm': 0}],
                           legality_budget={'overlap_area': 0}), fh)
        r = run_utils.check([sys.executable, '-X', 'utf8',
                             run_utils.tool('check_floorplan.py'), pile,
                             '--intent', bad, '--plan-only', '--no-brief',
                             '--no-mechanical'],
                            refuse='plan_zone_overfull', code=4)
        line = [x for x in r.stdout.splitlines()
                if x.startswith('JSON_SUMMARY:')][-1]
        s = json.loads(line.split('JSON_SUMMARY: ', 1)[1])
        assert s['plan_errors'] >= 1, s
    print("  PASS: a true pile grades exit 3 but plans exit 0; an "
          "unsatisfiable plan is --plan-only exit 4")


def test_plan_check_refuses_no_emitted_corpus_intent():
    """The control: every tracked board's own emitted intent is a plan the
    board satisfies, so plan_check must refuse none of them."""
    from types import SimpleNamespace
    sys.path.insert(0, os.path.dirname(DRIVER))
    import importlib
    drv = importlib.import_module('placement_driver')
    refused = {}
    n = grouped = 0
    for board in run_utils.corpus_boards():
        pcb = parse_kicad_pcb(board)
        try:
            doc = fp.emit_intent(pcb, board, group_sources=SOURCES,
                                 declare_classes=True)
            it = fp.intent_from_dict(doc, '')
            found, _ = fp.plan_check(it, pcb, board, group_sources=SOURCES)
        except fp.UntrustworthyOutline:
            continue
        n += 1
        errs = _errors(found)
        if errs:
            refused[os.path.basename(board)] = [(v.rule, v.ref, v.block)
                                                for v in errs]
        # ...and P1's own call, on the boards whose plan names a sheet or
        # kicad GROUP: without the group sources `_plan_owed` passes, those
        # blocks resolve to nothing and P1 refuses every such plan (D2).
        if any(b.get('group') for b in doc.get('blocks') or ()):
            grouped += 1
            ok, why = drv._plan_owed(SimpleNamespace(board=board), it, pcb,
                                     fp)
            if not ok:
                refused[os.path.basename(board) + ' (P1)'] = why[:300]
    assert n >= 15, n
    assert grouped >= 1, 'no corpus plan names a group -- D2 is untested'
    assert not refused, refused
    print(f"  PASS: plan_check refuses none of {n} emitted corpus intents")




def test_round2_the_plan_errors_track_the_grade():
    """Round-2 verifier on Phase 4: an exclusive zone whose owners the grade
    cannot see (BLOCKING), a lead field reaching past its courtyard, a
    demoted zone rule, a part allowed off the board, and a locked part that
    already fills a zone -- each was a plan ERROR the grade disagreed with,
    or the reverse."""
    with tempfile.TemporaryDirectory() as tmp:
        # A zone exclusive to a courtyard-less logo grades nobody.
        b = os.path.join(tmp, 'logo.kicad_pcb')
        text = open(_fab_board(tmp), encoding='utf-8').read().replace(
            '(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 '
            '"/A") (uuid "p0-M1"))', '').replace(
            '(fp_rect (start -1 -1) (end 1 1) (stroke (width 0.1) (type '
            'solid)) (fill none) (layer "F.CrtYd") (uuid "F.CrtYd-M1"))', '')
        open(b, 'w', encoding='utf-8').write(text)
        assert not parse_kicad_pcb(b).footprints['M1'].pads
        found, _ = _check(_raw(blocks=[
            {'name': 'B', 'refs': ['P1'], 'zone': [7, 7, 13, 13],
             'tolerance_mm': 0},
            {'name': 'A', 'refs': ['M1'], 'zone': [0, 0, 40, 40],
             'exclusive': True, 'tolerance_mm': 0}]), b)
        assert not [v for v in found
                    if v.rule == 'plan_zone_exclusive_unsatisfiable'], found
        # A demoted zone rule demotes the plan finding that stands for it.
        fb = _fab_board(tmp)
        inside = [{'name': 'B', 'refs': ['P1'], 'zone': [8, 14, 12, 29],
                   'tolerance_mm': 0},
                  {'name': 'A', 'refs': ['M1'], 'zone': [5, 13, 15, 30],
                   'exclusive': True, 'tolerance_mm': 0}]
        for sev_map, want in (({}, 'error'),
                              ({'zone_exclusive': 'warn'}, 'warn'),
                              ({'zone_containment': 'warn'}, 'warn'),
                              ({'zone_exclusive': 'warn',
                                'plan_zone_exclusive_unsatisfiable': 'error'},
                               'error')):
            found, _ = _check(_raw(blocks=inside, severity=sev_map), fb)
            got = [v.severity for v in found
                   if v.rule == 'plan_zone_exclusive_unsatisfiable']
            assert got == [want], (sev_map, got)
        # A 45-degree pile rotation reaches the check and passes on the
        # 0-degree lattice (the round-1 arm never overlapped A).
        text = open(fb, encoding='utf-8').read().replace(
            '(uuid "fp-P1") (at 10 10)', '(uuid "fp-P1") (at 10 10 45)')
        open(fb, 'w', encoding='utf-8').write(text)
        room = [{'name': 'B', 'refs': ['P1'], 'zone': [7.9, 8.0, 12.1, 15.0],
                 'tolerance_mm': 0},
                {'name': 'A', 'refs': ['M1'], 'zone': [5, 13, 15, 30],
                 'exclusive': True, 'tolerance_mm': 0}]
        found, _ = _check(_raw(blocks=room), fb)
        assert any(v.rule == 'intent_zone_overlap' for v in found), found
        assert not [v for v in found
                    if v.rule == 'plan_zone_exclusive_unsatisfiable'], found
        # A locked part that already fills a zone, plus a free member: no
        # arrangement fits both at budget 0 -- the ERROR the locked
        # exclusion had lost.
        lb = _board(tmp, 'fill.kicad_pcb',
                    [('L1', 10, 7.5, [(0, 0, 10, 5)], True),
                     ('U2', 30, 5, [(0, 0, 10, 5)])], w=40, h=20)
        found, _ = _check(_raw(blocks=[{'name': 'z', 'refs': ['L1', 'U2'],
                                        'zone': [5, 5, 15, 10],
                                        'tolerance_mm': 0}],
                               legality_budget={'overlap_area': 0}), lb)
        assert [v.severity for v in found
                if v.rule == 'plan_zone_overfull'] == ['error'], found
        # The board bound needs every part kept ON the board.
        big = [(dx, dy, 0.9, 0.9) for dx in range(4) for dy in range(4)]
        parts = [(f'U{i}', 2 + 4 * (i % 4), 2 + 4 * (i // 4) % 8, big)
                 for i in range(16)]
        bb = _board(tmp, 'bb.kicad_pcb', parts)
        for budget, want in (({'overlap_area': 0}, []),
                             ({'overlap_area': 0, 'oob_count': 3}, []),
                             ({'overlap_area': 0, 'oob_count': 0},
                              ['error']),
                             ({'overlap_area': 1e6, 'oob_count': 0}, [])):
            found, _ = _check(_raw(legality_budget=budget), bb)
            got = [v.severity for v in found
                   if v.rule == 'plan_board_overfull']
            assert got == want, (budget, got)
        # One-face assembly: the other WARN branch (M19).
        found, _ = _check(_raw(assembly={'sides': 'F'}), bb)
        w = [v for v in found if v.rule == 'plan_board_crowded']
        assert any('assembly policy' in v.message for v in w), found
        # A WARN half honours the severity map (N11/N16).
        found, _ = _check(_raw(severity={'plan_board_crowded': 'error'}), bb)
        assert 'error' in [v.severity for v in found
                           if v.rule == 'plan_board_crowded'], found
    print("  PASS: an invisible-owner exclusive zone, demoted zone rules, a "
          "45-degree pile, a locked fill, off-board allowance and the "
          "one-face WARN each read as the grade would")


def test_round2_the_far_face_charge_is_what_the_courtyard_confines():
    """A drilled-pad rect reaching past its courtyard may sit outside the
    zone, so only its part inside the courtyard is charged."""
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'tht.kicad_pcb')
        pads = ''.join(
            f'    (pad "{i + 1}" thru_hole circle (at {x} 0) (size 1 1) '
            f'(drill 0.6) (layers "*.Cu") (net 1 "/A") (uuid "t{i}"))\\n'
            for i, x in enumerate((-5.5, 5.5)))
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(
                '(kicad_pcb (version 20241229) (generator "test")\\n'
                '  (net 0 "")\\n  (net 1 "/A")\\n'
                '  (gr_rect (start 0 0) (end 40 40) (layer "Edge.Cuts") '
                '(uuid "e1"))\\n'
                '  (footprint "t:T" (layer "F.Cu") (uuid "fp-P1") '
                '(at 20 20)\\n    (property "Reference" "P1" (at 0 0))\\n'
                '    (fp_rect (start -5 -2) (end 5 2) (stroke (width 0.05) '
                '(type solid)) (fill none) (layer "F.CrtYd") (uuid "c1"))\\n'
                + pads + '  )\\n'
                + ''.join(
                    f'  (footprint "t:Q" (layer "B.Cu") (uuid "fp-{q}") '
                    f'(at {x} 20)\\n    (property "Reference" "{q}" '
                    f'(at 0 0))\\n    (pad "1" smd rect (at 0 0) (size 5 3) '
                    f'(layers "B.Cu") (net 1 "/A") (uuid "{q}p"))\\n  )\\n'
                    for q, x in (('Q1', 17.5), ('Q2', 22.5)))
                + ')\\n')
        # The zone IS P1's 10 x 4 courtyard. On B, Q1 + Q2 take 30 mm2 and
        # the leads 10 of the 12 their drill rect spans: 40 of 40, which
        # fits. Charging the whole 12 x 1 drill rect read 42.
        found, meas = _check(_raw(blocks=[{'name': 'z',
                                           'refs': ['P1', 'Q1', 'Q2'],
                                           'zone': [15, 18, 25, 22],
                                           'tolerance_mm': 0}],
                                  legality_budget={'overlap_area': 0}),
                             path)
        row = meas['zones'][0]
        assert sorted(row['members']) == ['P1', 'Q1', 'Q2'], row
        assert not [v for v in found if v.rule == 'plan_zone_overfull'], (
            found, meas['zones'])
    print("  PASS: a lead field wider than its courtyard is charged only "
          "inside it")


def test_p1_itself_refuses_a_plan_error():
    """The DRIVER's P1, not only `plan_check`: run 27's plan with the logos
    answered and one zone shrunk so its members cannot fit under a declared
    overlap budget of 0. Each member still fits alone -- a zone smaller than
    a member is an anchor, which is not charged. Without this, P1 could stop
    asking and every other test would still pass (the #959 battery's
    `p1-never-checks-the-plan` survived)."""
    logos = ['#00000000-0000-0000-0000-00005a3b5201',
             '#00000000-0000-0000-0000-00005d8c51dd',
             '#00000000-0000-0000-0000-00005e7dd057']
    with open(PLAN_975, encoding='utf-8') as fh:
        plan = json.load(fh)
    plan['dispositions'] = {'refs': {k: 'a back-side logo; cosmetic'
                                     for k in logos}}
    plan['legality_budget'] = {'overlap_area': 0.0, 'oob_count': 0}
    for b in plan['blocks']:
        if b['name'] == 'ldo':          # U2, C1, C3
            b['zone'] = [114.5, 91.5, 118.4, 93.1]
            b['tolerance_mm'] = 0.0
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, 'overfull.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(plan, fh)
        r = run_utils.check(
            [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
             '--board', ESP, '--zone-plan', p, '--waive',
             'seed-connectors:the probe hands them over'],
            refuse='in the zone plan that no arrangement can satisfy',
            code=4)
    assert 'plan_zone_overfull' in r.stdout and "'ldo'" in r.stdout, \
        r.stdout[-1500:]
    assert '--plan-only' in r.stdout, r.stdout[-800:]
    print("  PASS: the driver's P1 refuses the overfull plan by name, and "
          "points at --plan-only")


TESTS = [
    test_run29_lap5_overlaps_are_warnings_not_errors,
    test_round2_the_plan_errors_track_the_grade,
    test_round2_the_far_face_charge_is_what_the_courtyard_confines,
    test_the_exclusive_check_reads_the_grades_geometry,
    test_exclusive_infeasibility_is_the_one_error,
    test_a_literal_glob_is_an_error_only_when_it_double_zones,
    test_a_locked_member_outside_its_zone,
    test_two_locked_parts_that_overlap,
    test_the_area_bound_is_per_face_and_passes_shipped_boards,
    test_the_area_bound_is_sound_for_the_grade,
    test_the_edge_bound_reads_pads_not_courtyards,
    test_the_board_area_bound_is_per_face,
    test_place_seed_refuses_before_writing_and_repair_only_reports,
    test_plan_only_on_a_pile,
    test_plan_only_on_a_true_pile_and_its_exit_4,
    test_plan_check_refuses_no_emitted_corpus_intent,
    test_p1_itself_refuses_a_plan_error,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
