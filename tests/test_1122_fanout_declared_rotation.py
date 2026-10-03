#!/usr/bin/env python3
"""#1122: `place_fanout_clearance --intent` turns a cap only within its declaration.

The cap optimizer tried `ROTATIONS` -- every quarter turn -- for a cap whose
budget had escalated, and its via-clear fallback tried them for any cap
still grazing a via. Neither read a rotation declaration; `--intent` was read
for its decap limits only. So a cap whose angle a block declares was turned,
and no grade rule catches it (floorplan has no `rule_rotation`, by design).

The trap the fix must not fall into: with an intent, `repair_fanout_clearance`
may run the pass a second time WITHOUT the decap gate (`intent=None`) and ship
that run when it ends better (#1067). A hold read from `intent` inside the
pass would be dropped in exactly that run. The claims are therefore resolved
once and passed beside the intent (`declared_rotations`), and the end-to-end
arm below is chosen so that the run KEPT is the ungated one.

Fixture: test_1067's U30 crop at --clearance 0.1 with a 0.6 / 2.0 mm budget
(TIGHT) and that test's 2.0 mm decap limits, measured on this tree: with no
block the ungated run is kept and C24 270 -> 180, C33 180 -> 0 and C63 0 -> 180
are all turned. Declaring C24 at 270 keeps the ungated run, holds C24, and
leaves C33 and C63 turning -- so "held" is the declaration's doing.

Run: python3 -X utf8 tests/test_1122_fanout_declared_rotation.py [case ...]
"""
import ast
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

RUN_ALL_TIMEOUT = 1200

FIX = os.path.join(ROOT, 'tests', 'fixtures', '1067', 'u30_crop.kicad_pcb')
CLI = os.path.join(ROOT, 'py_placer', 'place_fanout_clearance.py')
ENGINE = os.path.join(ROOT, 'py_placer', 'placement', 'fanout_clearance.py')
COPY = os.path.join(ROOT, 'py_router', 'copy_board.py')
TIGHT = ['--clearance', '0.1', '--max-displacement', '0.6',
         '--max-displacement-cap', '2.0']


def _intent(td, blocks, name):
    """test_1067's tight document (the board's own emitted intent, decap
    limits 2.0 mm) plus `blocks`."""
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan as fp
    doc = fp.emit_intent(parse_kicad_pcb(FIX), FIX)
    doc['decaps'] = {'max_distance_mm': 2.0, 'max_pin_distance_mm': 2.0}
    doc['blocks'] = list(blocks)
    path = os.path.join(td, name + '.json')
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return path


def _board(td):
    b = os.path.join(td, 'in.kicad_pcb')
    run_utils.check([sys.executable, COPY, FIX, b], accept=True)
    return b


def _run(td, blocks, name, **kw):
    """(summary, {ref: rotation} turned, stdout) of one CLI run."""
    from kicad_parser import parse_kicad_pcb
    b = _board(td)
    out = os.path.join(td, name + '.kicad_pcb')
    r = run_utils.check([sys.executable, '-X', 'utf8', CLI, b, out] + TIGHT
                        + ['--intent', _intent(td, blocks, name)], accept=True)
    s = json.loads([ln for ln in r.stdout.splitlines()
                    if ln.startswith('JSON_SUMMARY: ')][-1][14:])
    src = parse_kicad_pcb(b).footprints
    got = parse_kicad_pcb(run_utils.evidence(out)).footprints
    turned = {k: (src[k].rotation % 360, got[k].rotation % 360) for k in src
              if k in got
              and abs((src[k].rotation - got[k].rotation) % 360) > 1e-6}
    poses = {k: (round(got[k].x, 4), round(got[k].y, 4), got[k].rotation % 360)
             for k in got}
    return s, turned, r.stdout, poses


def test_the_fixture_turns_caps():
    """The control: the same intent without a block turns C24, and keeps
    the ungated run."""
    with tempfile.TemporaryDirectory() as td:
        s, turned, _o, _p = _run(td, [], 'none')
    assert turned.get('C24') == (270.0, 180.0), turned
    assert s['decap']['compared']['kept'] == 'ungated', s['decap']['compared']
    # A run whose intent declares no rotation prints what it printed before.
    assert 'declared_rotations' not in s, s.get('declared_rotations')
    print("  no block: turned %r, kept ungated" % (turned,))


def test_the_rotation_list():
    from placement.fanout_clearance import ROTATIONS, _cap_rotations

    class Cap:
        def __init__(self, rot, seed=None):
            self.rot, self.seed_rot = rot, rot if seed is None else seed
    c270 = Cap(270.0)
    assert _cap_rotations(c270, None, True, True) == ROTATIONS
    assert _cap_rotations(c270, None, True, False) == [270.0]
    assert _cap_rotations(c270, None, False, True) == [270.0]
    assert _cap_rotations(c270, (270.0, None), True, True) == [270.0]
    assert _cap_rotations(Cap(90.0), (0.0, None), True, True) == [90.0, 0.0]
    assert _cap_rotations(c270, (None, (180.0, 90.0, 270.0)), True, True) \
        == [270.0, 180.0, 90.0]
    assert _cap_rotations(c270, (None, (45.0, 90.0)), True, True) \
        == [270.0, 90.0], 'an off-lattice member is offered'
    assert _cap_rotations(c270, (None, (90.0,)), False, True) == [270.0]
    # Not armed: a declared cap does not turn either (#1122 verifier, M4).
    assert _cap_rotations(c270, (None, (270.0, 90.0)), True, False) == [270.0]
    # The first angle is where the cap IS, not where it was seeded: a cap
    # that turned into its declaration stays offered it (M3).
    assert _cap_rotations(Cap(0.0, seed=90.0), (0.0, None), True, True) \
        == [0.0]
    print("  undeclared unchanged; declared turns only within the claim")


def test_rotations_are_confined_to_the_helper():
    """Every read of `ROTATIONS` in the engine goes through `_cap_rotations`
    -- a new turning site that reads the lattice directly is what #1122 was."""
    with open(ENGINE, encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    outside = []

    def walk(node, fn):
        for ch in ast.iter_child_nodes(node):
            name = (ch.name if isinstance(ch, (ast.FunctionDef,
                                               ast.AsyncFunctionDef)) else fn)
            if (isinstance(ch, ast.Name) and ch.id == 'ROTATIONS'
                    and isinstance(ch.ctx, ast.Load) and fn != '_cap_rotations'):
                outside.append((ch.lineno, fn))
            walk(ch, name)
    walk(tree, None)
    assert not outside, outside
    print("  ROTATIONS is read only inside _cap_rotations")


def test_a_declared_cap_is_not_turned_in_the_run_that_ships():
    """C24 declared at 270: held, while the run kept is the UNGATED one."""
    with tempfile.TemporaryDirectory() as td:
        s, turned, out, _p = _run(td, [{'name': 'rot_C24', 'refs': ['C24'],
                                        'rotation': 270.0}], 'c24')
    assert 'C24' not in turned, turned
    assert s['decap']['compared']['kept'] == 'ungated', (
        'the fixture no longer exercises the ungated arm: %r'
        % s['decap']['compared'])
    assert s['declared_rotations'] == {'C24': 270.0}, s['declared_rotations']
    assert 'Declared rotations (intent): 1 cap(s): C24 at 270' in out
    assert 'C33' in turned and 'C63' in turned, (
        'an undeclared cap stopped turning: %r' % (turned,))
    print("  C24 held at 270 in the kept (ungated) run; still turned: %r"
          % (turned,))


def test_a_candidate_set_turns_only_within_it():
    with tempfile.TemporaryDirectory() as td:
        s24, t24, o24, _p = _run(td, [{'name': 'a', 'refs': ['C24'],
                                       'rotation_candidates': [270.0, 90.0]}],
                                 'set24')
        _s, t33, _o, _p = _run(td, [{'name': 'b', 'refs': ['C33'],
                                     'rotation_candidates': [180.0, 0.0]}],
                               'set33')
    # Without a claim C24 goes to 180; with {270, 90} it may only reach 90.
    assert t24.get('C24') == (270.0, 90.0), t24
    # A set is disclosed as a set, in the line and the JSON (M8, M10).
    assert 'C24 within [270.0, 90.0]' in o24, o24[-1500:]
    assert s24['declared_rotations'] == {'C24': [270.0, 90.0]}, s24
    # A turn the set admits is still made.
    assert t33.get('C33') == (180.0, 0.0), t33
    print("  C24 {270, 90} -> %r; C33 {180, 0} -> %r"
          % (t24.get('C24'), t33.get('C33')))


def test_the_ungated_arm_is_handed_the_claims():
    """Both runs -- the gated one and the comparison without the gate --
    receive the same claims, whichever is kept."""
    from unittest.mock import patch
    from kicad_parser import parse_kicad_pcb
    from placement import fanout_clearance as fc
    from placement import floorplan as fp
    calls = []
    real = fc._repair_one_arm

    def spy(pcb_data, **kw):
        calls.append((kw.get('intent') is not None,
                      kw.get('declared_rotations')))
        return real(pcb_data, **kw)
    with tempfile.TemporaryDirectory() as td:
        b = _board(td)
        intent = fp.load_intent(_intent(td, [{'name': 'rot_C24', 'refs':
                                              ['C24'], 'rotation': 270.0}],
                                        'spy'))
        with patch.object(fc, '_repair_one_arm', spy):
            fc.repair_fanout_clearance(
                parse_kicad_pcb(b), b, clearance=0.1, netclass_ceiling=0.1,
                max_displacement=0.6, max_displacement_cap=2.0, intent=intent)
    assert [g for g, _ in calls] == [True, False], calls
    assert all(d == {'C24': (270.0, None)} for _, d in calls), calls
    print("  gated and ungated runs both carry %r" % (calls[0][1],))


def test_a_declaration_on_a_part_that_is_not_a_cap_changes_nothing():
    from kicad_parser import parse_kicad_pcb
    rot = parse_kicad_pcb(FIX).footprints['U30'].rotation % 360
    with tempfile.TemporaryDirectory() as td:
        _s, _t, out, p_u30 = _run(td, [{'name': 'u', 'refs': ['U30'],
                                        'rotation': rot}], 'u30')
        _s, _t, _o, p_none = _run(td, [], 'none')
    assert p_u30 == p_none
    assert 'Declared rotations' not in out
    print("  U30 declared at its own angle: every pose identical")


def test_a_contradiction_writes_and_records_nothing():
    with tempfile.TemporaryDirectory() as td:
        b = _board(td)
        out = os.path.join(td, 'o.kicad_pcb')
        manifest = os.path.join(td, 'redo_commands.sh')
        ip = _intent(td, [{'name': 'a', 'refs': ['C24'], 'rotation': 270.0},
                          {'name': 'b', 'refs': ['C2*'], 'rotation': 180.0}],
                     'contra')
        run_utils.check([sys.executable, '-X', 'utf8', CLI, b, out] + TIGHT
                        + ['--intent', ip], refuse='different rotations',
                        code=2, env=dict(os.environ, REDO_MANIFEST=manifest))
        assert not os.path.exists(out), 'a refusal wrote a board'
        assert not (os.path.exists(manifest) and os.path.getsize(manifest)), \
            'a refusal was recorded'
        # A candidate SET against a single angle is a contradiction too, and
        # the pre-check reads sets as well as single angles (M6).
        ip2 = _intent(td, [{'name': 'a', 'refs': ['C24'],
                            'rotation_candidates': [270.0, 90.0]},
                           {'name': 'b', 'refs': ['C24'], 'rotation': 180.0}],
                      'contra2')
        run_utils.check([sys.executable, '-X', 'utf8', CLI, b, out] + TIGHT
                        + ['--intent', ip2], refuse='different rotations',
                        code=2, env=dict(os.environ, REDO_MANIFEST=manifest))
        assert not os.path.exists(out), 'a refusal wrote a board'
    print("  two blocks, two angles for C24 (single or set): exit 2, nothing "
          "written or recorded")


def test_an_off_lattice_member_is_named_not_offered():
    """C24 declared {45, 90}: 45 is off its quarter-turn lattice, so it is
    not offered, and the run says so by name (M5)."""
    with tempfile.TemporaryDirectory() as td:
        _s, turned, out, _p = _run(td, [{'name': 'a', 'refs': ['C24'],
                                         'rotation_candidates': [45.0, 90.0]}],
                                   'offl')
    assert '(C24: 45 not offered -- off its quarter-turn lattice)' in out, \
        out[-1500:]
    assert turned.get('C24', (270.0, 270.0))[1] in (270.0, 90.0), turned
    print("  C24 {45, 90}: 45 named and not offered; C24 -> %r"
          % (turned.get('C24'),))


def test_the_escalation_guard_decides_an_outcome():
    """#1122's verifier measured the guard (a cap whose declaration leaves
    no other angle is not armed to turn) flip which cap the crop leaves
    grazing: C63 declared at 0 resolves every graze WITH it, and leaves C63
    grazing without it. Pinned so the guard has a witness."""
    with tempfile.TemporaryDirectory() as td:
        s, _t, _o, _p = _run(td, [{'name': 'a', 'refs': ['C63'],
                                   'rotation': 0.0}], 'c63')
    assert s['unresolved'] == [], s['unresolved']
    assert s['decap']['compared']['kept'] == 'ungated', s['decap']['compared']
    print("  C63 declared 0: every graze resolved, ungated run kept")


def test_an_unresolved_rotation_block_is_said():
    """A block that declares a rotation and resolves to no part holds
    nothing; it is printed rather than run silently (#1122's verifier)."""
    with tempfile.TemporaryDirectory() as td:
        _s, turned, out, _p = _run(td, [{'name': 'ghost',
                                         'group': 'NO_SUCH_GROUP',
                                         'rotation': 270.0}], 'ghost')
    assert '[block_unresolved]' in out and 'holds no part' in out, \
        out[-1500:]
    # Once per block, whatever number of problems it has.
    assert sum('(#1122)' in ln for ln in out.splitlines()) == 1, out[-1500:]
    print("  a rotation block naming no group: INTENT WARN printed, once")


def test_a_partly_resolved_block_is_not_said_to_hold_nothing():
    """Block a's `refs` resolve while its `group` does not: it still holds
    C24, so it is warned without "holds no part". Block b declares no
    rotation, so its problem is not #1122's to print. (#1122's round-2
    verifier: every rotation block's problem read "holds no part".)"""
    import contextlib
    import io as _io
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan as fp
    from placement.fanout_clearance import declared_cap_rotations
    with tempfile.TemporaryDirectory() as td:
        ip = _intent(td, [{'name': 'a', 'refs': ['C24'],
                           'group': 'NO_SUCH_GROUP', 'rotation': 270.0},
                          {'name': 'b', 'group': 'NO_SUCH_GROUP'},
                          {'name': 'ghost', 'group': 'NO_SUCH_GROUP',
                           'rotation': 90.0}], 'part')
        buf = _io.StringIO()
        with contextlib.redirect_stdout(buf):
            held = declared_cap_rotations(fp.load_intent(ip),
                                          parse_kicad_pcb(FIX), report=True)
    lines = [ln for ln in buf.getvalue().splitlines() if '(#1122)' in ln]
    assert len(lines) == 2, buf.getvalue()
    (a,) = [ln for ln in lines if "block 'a'" in ln]
    (g,) = [ln for ln in lines if "block 'ghost'" in ln]
    assert 'holds only C24' in a and 'holds no part' not in a, a
    assert 'holds no part' in g, g
    assert held.get('C24') == (270.0, None), held
    print("  a: holds only C24; ghost: holds no part; b: not #1122's")


TESTS = [
    test_the_fixture_turns_caps,
    test_the_rotation_list,
    test_rotations_are_confined_to_the_helper,
    test_a_declared_cap_is_not_turned_in_the_run_that_ships,
    test_a_candidate_set_turns_only_within_it,
    test_the_ungated_arm_is_handed_the_claims,
    test_a_declaration_on_a_part_that_is_not_a_cap_changes_nothing,
    test_a_contradiction_writes_and_records_nothing,
    test_an_off_lattice_member_is_named_not_offered,
    test_the_escalation_guard_decides_an_outcome,
    test_an_unresolved_rotation_block_is_said,
    test_a_partly_resolved_block_is_not_said_to_hold_nothing,
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
