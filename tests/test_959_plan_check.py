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
        raw = _raw(blocks=[{'name': 'all', 'refs': ['U*'],
                            'zone': [1.5, 1.5, 2.4, 2.4], 'tolerance_mm': 0}])
        found, _ = _check(raw, b)
        v = [x for x in found if x.rule == 'plan_zone_overfull']
        assert v and v[0].severity == 'error', found
        assert abs(v[0].measured['members_area_mm2'] - 0.96) < 1e-6, v[0]
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
          "pass the per-face bound; 0.96 mm2 in a 0.81 mm2 zone does not")


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
    print("  PASS: sonde_u's DSUB passes (pads fit, its flange overhangs); a "
          "12.6 x 12.8 mm pad array on a 10 mm edge does not")


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
        found, _ = _check(_raw(), b)
        assert [v for v in found if v.rule == 'plan_board_overfull'], found
    print("  PASS: orangecrab and ulx3s fit per face; 16 x 14.4 mm2 on a "
          "200 mm2 board does not")


def test_place_seed_refuses_before_writing_and_repair_only_reports():
    with tempfile.TemporaryDirectory() as tmp:
        b = _board(tmp, 'o.kicad_pcb', [('U1', 3, 3, PAD), ('U2', 6, 3, PAD)])
        plan = os.path.join(tmp, 'p.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump(_raw(blocks=[{'name': 'all', 'refs': ['U*'],
                                    'zone': [1.5, 1.5, 2.4, 2.4],
                                    'tolerance_mm': 0}]), fh)
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
                                    'tolerance_mm': 0}]), fh)
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


def test_plan_check_refuses_no_emitted_corpus_intent():
    """The control: every tracked board's own emitted intent is a plan the
    board satisfies, so plan_check must refuse none of them."""
    refused = {}
    n = 0
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
    assert n >= 15, n
    assert not refused, refused
    print(f"  PASS: plan_check refuses none of {n} emitted corpus intents")


TESTS = [
    test_run29_lap5_overlaps_are_warnings_not_errors,
    test_exclusive_infeasibility_is_the_one_error,
    test_a_literal_glob_is_an_error_only_when_it_double_zones,
    test_a_locked_member_outside_its_zone,
    test_two_locked_parts_that_overlap,
    test_the_area_bound_is_per_face_and_passes_shipped_boards,
    test_the_edge_bound_reads_pads_not_courtyards,
    test_the_board_area_bound_is_per_face,
    test_place_seed_refuses_before_writing_and_repair_only_reports,
    test_plan_only_on_a_pile,
    test_plan_check_refuses_no_emitted_corpus_intent,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
