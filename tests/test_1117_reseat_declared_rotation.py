#!/usr/bin/env python3
"""#1117: `place_seed`'s post-polish re-seat holds a DECLARED rotation.

The re-seat (#701/#797) puts a part the polish walked out of its zone or into
a keep-out back, with the seeder's own `_try_place`. It passed no
`rotations=`, so `_try_place` searched its fallback lattice -- the polished
angle, then each quarter turn -- and could turn a part whose angle the intent
declares (`blocks[].rotation`, `rotation_candidates`, or the `rotation:<ref>`
block `rank_rotations --write-intent` hands to the next seed). It was the one
production `_try_place` call that did, and a failed re-seat printed nothing.

The rig is test_701's: a sitecustomize patches `placement.quench.quench` to
return a forced move list, because the quench on a fixture this small leaves
everything alone and the re-seat would never run -- a test that passes in
both directions. The geometry makes the turn the ONLY way back in:

    zone `b` = U1 only, [4, 4, 14, 14], tolerance 0.5
    U1: courtyard 6 x 2              R1: courtyard 7 x 11 (the strip blocker)
    the polish moves U1 to (30, 12) and R1 to (7.5, 9), which covers the
    zone's whole height from x 4 to 11 and leaves a 3.3 mm strip: U1 fits
    there at 90 or 270, never at 0 or 180.

`test_the_fixture_admits_only_a_turned_pose` proves that geometry in-process
before any arm relies on it. Every CLI arm asserts on the RE-PARSED WRITTEN
BOARD; the JSON is checked too, for what it claims. The declared arms A and C
each have an undeclared control on the same board showing the re-seat does
turn a part that declares nothing, so "not turned" is not satisfied by a
re-seat that never ran; B's three arms are each other's controls.

The quench's SWAP phase had the same hole one step earlier (#1117's
verifier): it exchanges full poses, angles included, so two parts of one
footprint declared at different angles traded them, and nothing graded it.
`test_a_swap_does_not_trade_declared_angles` pins that.

Run: python3 -X utf8 tests/test_1117_reseat_declared_rotation.py [case ...]
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 900

SIZE = (40.0, 24.0)
ZONE = (4.0, 4.0, 14.0, 14.0)
TOL = 0.5
CLEARANCE = 0.2
EDGE = 0.5
#: The refusal `place_seed` prints when its own grade fails (gate_reason).
GATE = 'does NOT satisfy its intent'

#: The polish the sitecustomize forces, read from this variable as JSON.
_INJECT = '''
import json
import os

import placement.quench as _q
_real = _q.quench


def _forced(*a, **kw):
    _real(*a, **kw)
    return json.loads(os.environ['T1117_POLISH'])


_q.quench = _forced
'''

#: U1 out of its zone at 0, R1 into the zone as the strip blocker.
STRIP = [{'reference': 'U1', 'new_x': 30.0, 'new_y': 12.0, 'new_rotation': 0.0},
         {'reference': 'R1', 'new_x': 7.5, 'new_y': 9.0, 'new_rotation': 0.0}]


def _part(ref, x, y, half_w, half_h, rot=0.0):
    """A footprint with a (2*half_w x 2*half_h) courtyard and two SMD pads
    on its long axis. NOT square: a square part reads the same at every
    angle, and this test is about which angle was written."""
    pads = ''.join(
        f'\t\t(pad "{i + 1}" smd rect\n'
        f'\t\t\t(at {dx} 0)\n'
        f'\t\t\t(size 0.6 0.6)\n\t\t\t(layers "F.Cu")\n'
        f'\t\t\t(net {i + 1} "N{i + 1}")\n'
        f'\t\t\t(uuid "p{i}-{ref}")\n\t\t)\n'
        for i, dx in enumerate((-1.5, 1.5)))
    return f'''\t(footprint "test:P{ref}"
\t\t(layer "F.Cu")
\t\t(uuid "fp-{ref}")
\t\t(at {x} {y} {rot:g})
\t\t(property "Reference" "{ref}"
\t\t\t(at 0 0)
\t\t)
\t\t(fp_rect
\t\t\t(start {-half_w} {-half_h})
\t\t\t(end {half_w} {half_h})
\t\t\t(layer "F.CrtYd")
\t\t\t(uuid "cy-{ref}")
\t\t)
{pads}\t)
'''


def _board(path, u1=(30.0, 18.0, 0.0), r1=(34.0, 8.0, 0.0), r1_half=(3.5, 5.5)):
    body = ('(kicad_pcb\n\t(version 20241229)\n'
            '\t(net 0 "")\n\t(net 1 "N1")\n\t(net 2 "N2")\n'
            '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end {} {})\n'
            '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n'.format(*SIZE)
            + _part('U1', u1[0], u1[1], 3.0, 1.0, u1[2])
            + _part('R1', r1[0], r1[1], r1_half[0], r1_half[1], r1[2])
            + ')\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)
    return path


def _intent_doc(claim=None):
    """Block `b` = U1 alone with the zone, so the seat target is the zone
    centre exactly; `claim` is merged into the block (a `rotation` or a
    `rotation_candidates`)."""
    block = {'name': 'b', 'refs': ['U1'], 'zone': list(ZONE),
             'tolerance_mm': TOL}
    block.update(claim or {})
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'min_reader': 5,
            'envelope': {'rect': [0.0, 0.0, SIZE[0], SIZE[1]],
                         'tolerance_mm': 0.5},
            'blocks': [block]}


def _env(wd, polish):
    inj = os.path.join(wd, 'inj')
    os.makedirs(inj, exist_ok=True)
    with open(os.path.join(inj, 'sitecustomize.py'), 'w',
              encoding='utf-8') as f:
        f.write(_INJECT)
    return dict(os.environ, PYTHONHASHSEED='0', PYTHONIOENCODING='utf-8',
                T1117_POLISH=json.dumps(polish),
                PYTHONPATH=inj + os.pathsep + os.pathsep.join(
                    os.path.join(ROOT, d)
                    for d in ('py_placer', 'py_router', 'py_tools')))


def _seed(tag, claim, polish, *, refuse=None, r1_half=(3.5, 5.5)):
    """Seed the fixture under a forced polish; returns (summary, U1 pose,
    stdout). `refuse` None = assert exit 0; otherwise assert exit 4 for
    the gate's own reason."""
    wd = tempfile.mkdtemp(prefix=f't1117_{tag}_')
    bpath = _board(os.path.join(wd, 'in.kicad_pcb'), r1_half=r1_half)
    ipath = os.path.join(wd, 'fp.json')
    with open(ipath, 'w', encoding='utf-8') as f:
        json.dump(_intent_doc(claim), f)
    out = os.path.join(wd, 'out.kicad_pcb')
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_placer', 'place_seed.py'), bpath, out,
            '--intent', ipath, '--clearance', str(CLEARANCE),
            '--board-edge-clearance', str(EDGE), '--force']
    env = _env(wd, polish)
    if refuse is None:
        r = run_utils.check(argv, accept=True, env=env)
    else:
        r = run_utils.check(argv, refuse=refuse, code=4, env=env)
    summary = None
    for line in r.stdout.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            summary = json.loads(line.split(':', 1)[1])
    assert summary is not None, f"no JSON_SUMMARY line\n{r.stdout[-1500:]}"
    from kicad_parser import parse_kicad_pcb
    fp = parse_kicad_pcb(run_utils.evidence(out, 'written seed')
                         ).footprints['U1']
    return summary, (round(fp.x, 3), round(fp.y, 3),
                     round((fp.rotation or 0.0) % 360, 1)), r.stdout


def _in_zone(pose):
    """U1's courtyard (6 x 2, turned by its rotation) inside the zone plus
    its tolerance."""
    x, y, rot = pose
    hw, hh = (1.0, 3.0) if rot in (90.0, 270.0) else (3.0, 1.0)
    return (x - hw >= ZONE[0] - TOL - 1e-6 and x + hw <= ZONE[2] + TOL + 1e-6
            and y - hh >= ZONE[1] - TOL - 1e-6
            and y + hh <= ZONE[3] + TOL + 1e-6)


def test_the_fixture_admits_only_a_turned_pose():
    """The arms below rest on this: on the post-polish board U1 has NO pose in
    its zone at 0 or 180 and one at 90. Proven with the seat search itself,
    at the same clearances the CLI uses, so a fixture edit that quietly
    widens the strip refuses here instead of turning every arm vacuous."""
    import pose_score
    from kicad_parser import parse_kicad_pcb
    from placement import seeder
    wd = tempfile.mkdtemp(prefix='t1117_fx_')
    path = _board(os.path.join(wd, 'post.kicad_pcb'), u1=(30.0, 12.0, 0.0),
                  r1=(7.5, 9.0, 0.0))
    got = {}
    for lad in ([0.0], [180.0], [90.0]):
        st = pose_score.make_state(parse_kicad_pcb(path), path,
                                   clearance=CLEARANCE,
                                   board_edge_clearance=EDGE)
        cx, cy = (ZONE[0] + ZONE[2]) / 2, (ZONE[1] + ZONE[3]) / 2
        clr = seeder._try_place(st, 'U1', cx, cy, set(), constraint=ZONE,
                                tol=TOL, rotations=lad)
        got[lad[0]] = None if clr is None else (st.parts['U1'].x,
                                                st.parts['U1'].y)
    assert got[0.0] is None and got[180.0] is None and got[90.0] is not None, (
        f"the fixture no longer forces a turn: {got}")
    print(f"  only 90 fits on the post-polish board: {got}")


def test_a_declared_rotation_is_not_traded_for_a_seat():
    """A: `rotation: 0`. Before #1117 the re-seat wrote U1 at 90 inside the
    zone and the seed exited 0 -- a declared angle silently replaced. Now it
    keeps 0, stays where the polish left it, names the decline and exits 4.
    A': the same board with nothing declared is re-seated at 90 and exits 0,
    so the declared arm is not passing on a re-seat that never ran."""
    s, pose, out = _seed('A', {'rotation': 0}, STRIP, refuse=GATE)
    assert pose == (30.0, 12.0, 0.0), f"U1 written at {pose}"
    assert 'NOT repaired, U1:' in out and 'declared rotation 0' in out, (
        out[-1500:])
    rec = (s.get('reseat_declined') or {}).get('U1')
    assert rec and rec['rotation'] == 0.0 and rec['zone'] == 'b' and (
        'zone_containment' in rec['rules']), f"reseat_declined={rec}"
    print(f"  declared 0: written {pose}, decline named, exit 4")
    s2, pose2, out2 = _seed('A2', None, STRIP)
    assert pose2[2] in (90.0, 270.0) and _in_zone(pose2), (
        f"control: undeclared U1 written at {pose2}")
    assert 'polish walked U1' in out2 and s2.get('reseat_declined') == {}, (
        f"control: reseat_declined={s2.get('reseat_declined')}")
    print(f"  undeclared control: re-seated at {pose2}, exit 0")


def test_a_candidate_set_bounds_the_reseat():
    """B1: `[0, 180]` has no angle that fits the strip, so the re-seat
    declines rather than leaving the set. B2: `[0, 90]` does, and the
    re-seat takes 90. B3: the seeder seats a set in the author's ORDER --
    `[180, 0]` seats at 180, which a sorted ladder would turn to 0. B4: the
    re-seat tries the angle the polish chose first when it is in the set."""
    s, pose, out = _seed('B1', {'rotation_candidates': [0, 180]}, STRIP,
                         refuse=GATE)
    assert pose[2] in (0.0, 180.0) and not _in_zone(pose), f"B1 at {pose}"
    rec = (s.get('reseat_declined') or {}).get('U1') or {}
    assert rec.get('rotation_candidates') == [0.0, 180.0], f"B1 {rec}"
    assert 'rotation_candidates [0, 180]' in out, out[-1500:]
    print(f"  [0, 180]: declined, written {pose}")
    _s, pose, _o = _seed('B2', {'rotation_candidates': [0, 90]}, STRIP)
    assert pose[2] == 90.0 and _in_zone(pose), f"B2 at {pose}"
    print(f"  [0, 90]: re-seated at {pose}")
    # B3: the seeder seats a candidate set in the AUTHOR's order (the first
    # that fits); the polish here moves only R1, so U1 stays where it seated.
    r1_away = [{'reference': 'R1', 'new_x': 22.0, 'new_y': 12.0,
                'new_rotation': 0.0}]
    _s, pose, _o = _seed('B3', {'rotation_candidates': [180, 0]}, r1_away)
    assert pose[2] == 180.0 and _in_zone(pose), f"B3 at {pose}"
    print(f"  [180, 0]: seated at {pose} (author order)")
    # B4: the re-seat tries the angle the polish chose FIRST when it is in
    # the set -- [0, 90], polished out of the zone at 90, comes back at 90,
    # not at the author's first 0 (the nudge picked 90 as an improvement).
    turned = [{'reference': 'U1', 'new_x': 30.0, 'new_y': 12.0,
               'new_rotation': 90.0}] + r1_away
    _s, pose, out = _seed('B4', {'rotation_candidates': [0, 90]}, turned)
    assert pose[2] == 90.0 and _in_zone(pose), f"B4 at {pose}"
    assert 'polish walked U1' in out, out[-1500:]
    print(f"  [0, 90], polished to 90: re-seated at {pose} (its own angle)")


def test_a_polish_turn_is_turned_back():
    """C: the polish moves U1 out of its zone AND turns it to 90, with the
    zone empty. The fallback lattice starts at the polished angle, so the
    old re-seat wrote 90; the declared ladder writes 0. C': undeclared, the
    re-seat keeps the polished 90 -- the same board, the other answer."""
    turned = [{'reference': 'U1', 'new_x': 30.0, 'new_y': 12.0,
               'new_rotation': 90.0},
              {'reference': 'R1', 'new_x': 22.0, 'new_y': 12.0,
               'new_rotation': 0.0}]
    _s, pose, _o = _seed('C', {'rotation': 0}, turned)
    assert pose[2] == 0.0 and _in_zone(pose), f"C at {pose}"
    print(f"  declared 0, polished to 90: re-seated at {pose}")
    _s, pose, _o = _seed('C2', None, turned)
    assert pose[2] == 90.0 and _in_zone(pose), f"C' at {pose}"
    print(f"  undeclared control: re-seated at {pose}")


def test_an_undeclared_decline_is_named_too():
    """D: nothing declared, and an 11 x 11 R1 fills the zone, so no angle
    fits. The re-seat used to fail silently -- the grade error appeared with
    no repair line. Now the decline is named, quarter turns and all."""
    full = [dict(STRIP[0]),
            {'reference': 'R1', 'new_x': 9.0, 'new_y': 9.0,
             'new_rotation': 0.0}]
    s, pose, out = _seed('D', None, full, refuse=GATE, r1_half=(5.5, 5.5))
    assert not _in_zone(pose), f"D at {pose}"
    assert 'NOT repaired, U1:' in out and 'any quarter turn' in out, (
        out[-1500:])
    rec = (s.get('reseat_declined') or {}).get('U1') or {}
    assert rec.get('rotation') is None and rec.get(
        'rotation_candidates') is None, f"D {rec}"
    print(f"  undeclared, zone full: decline named, written {pose}")


def test_repair_placement_holds_the_declaration():
    """E: `seeder.repair_placement` (place_seed --repair) seats a zone
    violator with the same ladder -- its closure now delegates to
    `floorplan.declared_ladder`. U1 sits just past the zone's east edge with
    R1 as the strip blocker: declared 0 it is NOT turned; undeclared it is
    re-seated at a quarter turn inside the zone."""
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan, seeder
    wd = tempfile.mkdtemp(prefix='t1117_E_')
    path = _board(os.path.join(wd, 'E.kicad_pcb'), u1=(17.0, 9.0, 0.0),
                  r1=(7.5, 9.0, 0.0))
    res = {}
    for tag, claim in (('declared', {'rotation': 0}), ('undeclared', None)):
        intent = floorplan.intent_from_dict(_intent_doc(claim))
        out = seeder.repair_placement(parse_kicad_pcb(path), path, intent,
                                      clearance=CLEARANCE,
                                      board_edge_clearance=EDGE)
        mv = {m['reference']: m for m in out.get('moves') or ()}
        res[tag] = mv.get('U1')
    assert res['declared'] is None or (
        res['declared']['new_rotation'] % 360 == 0.0), (
        f"declared U1 turned by repair: {res['declared']}")
    u = res['undeclared']
    assert u is not None and u['new_rotation'] % 360 in (90.0, 270.0) and (
        _in_zone((u['new_x'], u['new_y'], u['new_rotation'] % 360))), (
        f"control: undeclared repair move {u}")
    print(f"  repair: declared {res['declared']}, undeclared {u}")


def test_one_ladder_and_its_decline_line():
    """F: `floorplan.declared_ladder` is the mapping every seat search uses;
    it agrees with the quench's own declared branch (which normalises with
    `% 360` and stays separate on its hot path), and the decline line names
    each kind of claim."""
    from placement import floorplan
    from placement.quench import _candidate_rotations
    import place_seed
    assert floorplan.declared_ladder(None) is None
    for claim in ((0.0, None), (270.0, None), (None, (90.0, 0.0)),
                  (None, (180.0, 0.0, 90.0))):
        lad = floorplan.declared_ladder(claim)
        q = _candidate_rotations(None, True, claim)
        assert lad == q, f"{claim}: seat ladder {lad} vs quench {q}"
    # The record: only THIS part's errors, only the rules the re-seat
    # repairs, and the claim it was held to.
    from types import SimpleNamespace as NS
    errs = [NS(ref='U1', rule='zone_containment'), NS(ref='U1', rule='keepout'),
            NS(ref='U1', rule='decap_pin_distance'),
            NS(ref='R1', rule='zone_exclusive')]
    rec = place_seed.reseat_decline_record(
        'U1', errs, ('zone_containment', 'keepout', 'zone_exclusive'),
        NS(name='b'), (None, (0.0, 180.0)))
    assert rec == {'rules': ['keepout', 'zone_containment'], 'zone': 'b',
                   'rotation': None, 'rotation_candidates': [0.0, 180.0]}, rec
    line = place_seed.reseat_decline_line
    rec = {'rules': ['zone_containment'], 'zone': 'b'}
    one = line('U1', dict(rec, rotation=0.0, rotation_candidates=None))
    many = line('U1', dict(rec, rotation=None,
                           rotation_candidates=[0.0, 180.0]))
    none = line('U1', dict(rec, rotation=None, rotation_candidates=None))
    ko = line('U2', {'rules': ['keepout'], 'zone': None, 'rotation': 90.0,
                     'rotation_candidates': None})
    both = line('U3', {'rules': ['keepout', 'zone_containment',
                                 'zone_exclusive'], 'zone': 'b',
                       'rotation': None, 'rotation_candidates': None})
    bare = line('U4', {'rules': [], 'zone': None, 'rotation': None,
                       'rotation_candidates': None})
    excl = line('U5', {'rules': ['zone_exclusive'], 'zone': None,
                       'rotation': None, 'rotation_candidates': None})
    assert "in zone 'b' at its declared rotation 0 " in one, one
    assert 'rotation_candidates [0, 180]' in many, many
    assert 'any quarter turn' in none and 'declared' not in none, none
    assert ('no legal pose clear of the declared keep-out at its declared '
            'rotation 90 ') in ko, ko
    assert ("in zone 'b' clear of the declared keep-out and exclusive zone "
            "of another block") in both, both
    assert 'no legal pose anywhere on the board' in bare, bare
    assert ('no legal pose clear of the declared exclusive zone of another '
            'block at its current angle') in excl, excl
    print('  declared_ladder == quench for 4 claims; the record keeps only '
          "this part's repairable rules; 7 decline phrasings")


#: Two parts of one footprint between two connectors, C1 at 0 and C2 at 90,
#: sharing one zone (found by #1117's verifier: on 457959b7 the polish's swap
#: phase left C2 written at 0 with `rotation: 90` declared, exit 0).
def _swap_board(path, c2_first=False, crossed=False, c2_rot=90):
    """C1 at 0 and C2 at `c2_rot`, one footprint, between J1 (N1/N2) and J2
    (N3/N4). `c2_first` writes C2's block FIRST: the swap phase pairs parts
    in FILE order, so this is what flips which half of the swap gate (`ra`
    or `rb`) a one-sided declaration exercises -- renaming them does not
    (#1117's second verifier). `crossed` puts each part next to the OTHER
    part's connector, so on an already placed board a swap pays 16 mm."""
    def part(ref, fpname, x, rot, nets):
        pads = ''.join(
            f'\t\t(pad "{i + 1}" smd rect (at {dx} 0) (size 0.6 0.6) '
            f'(layers "F.Cu") (net {n} "N{n}") (uuid "p{i}-{ref}"))\n'
            for i, (dx, n) in enumerate(zip((-1.0, 1.0), nets)))
        return (f'\t(footprint "{fpname}" (layer "F.Cu") (uuid "fp-{ref}") '
                f'(at {x} 3.0 {rot:g})\n'
                f'\t\t(property "Reference" "{ref}" (at 0 0))\n'
                f'\t\t(fp_rect (start -1.5 -1.0) (end 1.5 1.0) '
                f'(layer "F.CrtYd") (uuid "cy-{ref}"))\n{pads}\t)\n')
    nets = ''.join(f'\t(net {i} "N{i}")\n' for i in range(1, 5))
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n' + nets
                + '\t(gr_rect (start 0 0) (end 40.0 6.0) (layer "Edge.Cuts") '
                '(uuid "e1"))\n'
                + part('J1', 'test:J', 4.0, 0, (1, 2))
                + part('J2', 'test:J', 36.0, 0, (3, 4))
                + ''.join(sorted(
                    (part('C1', 'test:CAP', 22.0 if crossed else 18.0, 0,
                          (1, 2)),
                     part('C2', 'test:CAP', 18.0 if crossed else 22.0, c2_rot,
                          (3, 4))),
                    key=lambda t: ('fp-C2' not in t) if c2_first else
                    ('fp-C1' not in t)))
                + ')\n')
    return path


def _swap_seed(tag, blocks, c2_first=False):
    wd = tempfile.mkdtemp(prefix=f't1117_{tag}_')
    bpath = _swap_board(os.path.join(wd, 'in.kicad_pcb'), c2_first)
    ipath = os.path.join(wd, 'fp.json')
    with open(ipath, 'w', encoding='utf-8') as f:
        json.dump({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                   'min_reader': 5,
                   'envelope': {'rect': [0, 0, 40, 6], 'tolerance_mm': 0.5},
                   'must_lock': ['J*'], 'blocks': blocks}, f)
    out = os.path.join(wd, 'out.kicad_pcb')
    r = run_utils.check(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_placer', 'place_seed.py'), bpath, out,
         '--intent', ipath, '--seed', '0', '--force', '--clearance', '0.2',
         '--board-edge-clearance', '0.3', '--max-displacement', '5'],
        accept=True)
    from kicad_parser import parse_kicad_pcb
    fps = parse_kicad_pcb(run_utils.evidence(out, 'written seed')).footprints
    return {ref: round((fps[ref].rotation or 0.0) % 360, 1)
            for ref in ('C1', 'C2')}, r.stdout


def test_a_swap_does_not_trade_declared_angles():
    """G: C1 declared 0 and C2 declared 90, one footprint, one zone. The
    polish's swap traded their poses -- angles included -- and wrote C2 at 0
    with exit 0 and no grade error (measured on 457959b7). Now the swap is
    refused, and counted as the intent refusing it (`swap-intent=` on the
    pass line).

    ONE side declaring is enough, and each side is its own half of the gate:
    C2 declared 90 alone, written second in the file (`rb`) and then first
    (`ra`), is held at 90 both times. Measured on 457959b7: written at 0 in
    every one of these.

    No undeclared control: with nothing declared the nudge turns the parts
    freely, so their written angles cannot say whether a swap ran. What
    proves the arm is not vacuous is that it FAILS with the swap gate
    removed -- `mutate_1117`'s `swap-*` rows."""
    zone = [14.0, 0.5, 26.0, 5.5]
    got, out = _swap_seed('G', [
        {'name': 'c1', 'refs': ['C1'], 'zone': zone, 'rotation': 0},
        {'name': 'c2', 'refs': ['C2'], 'zone': zone, 'rotation': 90}])
    assert got == {'C1': 0.0, 'C2': 90.0}, f"declared angles traded: {got}"
    assert 'swap-intent=' in out, out[-1500:]
    for c2_first in (False, True):
        one, _o = _swap_seed('G1%d' % c2_first, [
            {'name': 'a', 'refs': ['C1'], 'zone': zone},
            {'name': 'b', 'refs': ['C2'], 'zone': zone, 'rotation': 90}],
            c2_first=c2_first)
        assert one['C2'] == 90.0, (
            f"C2 declared 90 alone (first in file: {c2_first}), written at "
            f"{one['C2']}")
    print(f"  declared both: {got}; declared one, either half: held at 90")


def _optimize(tag, blocks, c2_rot, *extra):
    """place_optimize on the CROSSED board (a swap pays 16 mm): the
    written angles and positions, the JSON_SUMMARY and stdout."""
    wd = tempfile.mkdtemp(prefix=f't1117_{tag}_')
    bpath = _swap_board(os.path.join(wd, 'in.kicad_pcb'), crossed=True,
                        c2_rot=c2_rot)
    ipath = os.path.join(wd, 'fp.json')
    with open(ipath, 'w', encoding='utf-8') as f:
        json.dump({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                   'min_reader': 5,
                   'envelope': {'rect': [0, 0, 40, 6], 'tolerance_mm': 0.5},
                   'must_lock': ['J*'], 'blocks': blocks}, f)
    out = os.path.join(wd, 'out.kicad_pcb')
    r = run_utils.check(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_placer', 'place_optimize.py'), bpath, out,
         '--intent', ipath, '--clearance', '0.2', '--board-edge-clearance',
         '0.3', '--max-displacement', '4'] + list(extra), accept=True)
    summary = next(json.loads(ln.split(':', 1)[1])
                   for ln in r.stdout.splitlines()
                   if ln.startswith('JSON_SUMMARY:'))
    from kicad_parser import parse_kicad_pcb
    fps = parse_kicad_pcb(run_utils.evidence(out, 'optimized board')).footprints
    return ({ref: (round(fps[ref].x, 2), round((fps[ref].rotation or 0) % 360, 1))
             for ref in ('C1', 'C2')}, summary, r.stdout)


def test_the_optimizer_refuses_only_a_swap_that_turns_a_declared_part():
    """I: place_optimize on a placed board, parts crossed so a swap pays.
    (i) A ROTATION-ONLY intent -- no zone, so nothing else arms the swap
    gate -- with C1 at 0 and C2 at 90 declared: the swap is refused and
    disclosed under rule `rotation`, with the refs it binds (an earlier
    version of this fix reported `refs_bound: 0, rules_enforced: []` while
    refusing). On 457959b7 the swap was simply taken and C2 written at 0.
    (ii) Both parts at 0 and C1 declared 90: the swap changes NO angle, and
    it is taken, with and without --no-rotate (that earlier version refused
    it, losing 16 mm)."""
    held, s1, out1 = _optimize('Ii', [
        {'name': 'c1', 'refs': ['C1'], 'rotation': 0},
        {'name': 'c2', 'refs': ['C2'], 'rotation': 90}], c2_rot=90)
    assert held == {'C1': (22.0, 0.0), 'C2': (18.0, 90.0)}, held
    assert 'swap-intent=' in out1, out1[-1500:]
    assert s1.get('intent_moves_refused_by_site', {}).get('swap'), s1
    assert s1.get('intent_moves_refused_by_rule', {}).get('rotation'), s1
    assert s1.get('intent_rules_enforced') == ['rotation'], s1
    assert s1.get('intent_refs_bound') == 2, s1
    for extra in ((), ('--no-rotate',)):
        moved, s2, out2 = _optimize('Iii', [
            {'name': 'c1', 'refs': ['C1'], 'rotation': 90}], 0, *extra)
        assert moved['C1'][0] == 18.0 and moved['C2'][0] == 22.0, (extra,
                                                                    moved)
        assert 'swap-intent=' not in out2, (extra, out2[-1500:])
    print(f"  turning swap refused {held}; angle-neutral swap taken {moved}")


def test_rank_rotations_fails_a_declined_arm():
    """H: rank_rotations ranks an arm by what its seed produced. An arm whose
    seed's re-seat DECLINED the ranked part -- the angle held, the zone did
    not -- would rank on the crossings the polish bought by walking the part
    out of its zone. It ranks AFTER every angle whose seeds held the part,
    and still ranks (a zone too full for any angle is not about rotation):
    the ranker's row carries the record it decided on."""
    wd = tempfile.mkdtemp(prefix='t1117_H_')
    bpath = _board(os.path.join(wd, 'in.kicad_pcb'))
    ipath = os.path.join(wd, 'fp.json')
    with open(ipath, 'w', encoding='utf-8') as f:
        json.dump(_intent_doc(None), f)
    rj = os.path.join(wd, 'rot.json')
    run_utils.check(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_placer', 'rank_rotations.py'), bpath,
         '--intent', ipath, '--ref', 'U1', '--rotations', '0', '90',
         '--out-dir', os.path.join(wd, 'rot'), '--json-out', rj,
         '--seed-args=--force --clearance 0.2 --board-edge-clearance 0.5'],
        accept=True, env=_env(wd, STRIP))
    with open(run_utils.evidence(rj, 'ranking'), encoding='utf-8') as f:
        doc = json.load(f)
    rows = {r['rotation']: r for r in doc['rows']}
    assert 'U1' in rows[0.0]['reseat_declined'], rows[0.0]
    assert rows[0.0]['reseat_declined_ref'] is True, rows[0.0]
    assert rows[0.0]['hard_fail'] is None, rows[0.0]
    assert rows[90.0]['hard_fail'] is None, rows[90.0]
    assert doc['ranking'] == [90.0, 0.0], doc['ranking']
    print("  arm 0 declined, ranked after arm 90: %s" % doc['ranking'])


TESTS = [
    test_the_fixture_admits_only_a_turned_pose,
    test_a_declared_rotation_is_not_traded_for_a_seat,
    test_a_candidate_set_bounds_the_reseat,
    test_a_polish_turn_is_turned_back,
    test_an_undeclared_decline_is_named_too,
    test_repair_placement_holds_the_declaration,
    test_one_ladder_and_its_decline_line,
    test_a_swap_does_not_trade_declared_angles,
    test_the_optimizer_refuses_only_a_swap_that_turns_a_declared_part,
    test_rank_rotations_fails_a_declined_arm,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        # A filter that names no case passes nothing: a mutation battery
        # witness spelled wrong would otherwise read every row as SURVIVED.
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
