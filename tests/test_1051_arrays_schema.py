#!/usr/bin/env python3
"""#1051 / #1052 / #1054 phase 1: the `arrays`, `fixed_poses` and
`blocks[].rigid` schema, the brief compile, the formation predicate and the
pin order.

What this pins, and why each is a separate case rather than one round trip:

* every LOAD refusal carries its REASON and names the member -- a refusal is
  not evidence on its own (a malformed fixture once "passed" against a
  message about a different key);
* the BOARD-aware findings (`array_unresolved`, `array_conflict`) are raised
  by both `grade` and `resolve_intent_gate`, each naming its member;
* "unknown" and an absent key stay two different things in the brief report;
* a reader-6 intent still loads, and a reader-7 claim refuses on an older
  build;
* `arrays.formation` fails for each of its four reasons and passes a clean
  row -- and the as-built splitflap pull-ups R6..R8 ARE a clean pin-order row
  (pins 11-13 of U4, 19.05 mm apart), which is the positive control that the
  predicate is not failing everything;
* `arrays.pin_order` on splitflap's 47k pull-ups, read off pads and nets;
* every `fixed_poses[]` entry is GRADED (#1054 correction), whatever its
  source, and a ref the mechanical file itself anchors is graded once.

    python3 tests/test_1051_arrays_schema.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from kicad_parser import parse_kicad_pcb           # noqa: E402
from placement import arrays as arr                # noqa: E402
from placement import design_brief as db           # noqa: E402
from placement import floorplan as fp              # noqa: E402
from run_utils import check                        # noqa: E402

RUN_ALL_TIMEOUT = 900

SPLITFLAP = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
GLASGOW = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
ESP_BRIEF = os.path.join(ROOT, 'tests', 'fixtures', '959', 'asbuilt',
                         'esp_prog.design-brief.json')
ESP_MECH = os.path.join(ROOT, 'tests', 'fixtures', '959',
                        'run29_mechanical.json')
CHECK_FLOORPLAN = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')

#: splitflap's 47k pull-ups to U4 and the U4 pad each one's own net lands
#: on (+3V3 is shared by six of them, so it orders nothing). R14 is a
#: pull-DOWN: its GND lands on U4 pads 8 and 15, its signal on pad 10 alone,
#: so the single-pad net decides.
PULLUPS = ['R6', 'R7', 'R8', 'R9', 'R11', 'R12', 'R14']
PIN_ORDER = ['R11', 'R12', 'R14', 'R6', 'R7', 'R8', 'R9']

_PCB = {}


def _pcb(path=SPLITFLAP):
    if path not in _PCB:
        _PCB[path] = parse_kicad_pcb(path)
    return _PCB[path]


def _fresh(path=SPLITFLAP):
    return parse_kicad_pcb(path)


def _base(**over):
    d = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'}
    d.update(over)
    return d


def _row(**over):
    r = {'name': 'pullups', 'members': ['R6', 'R7', 'R8'], 'serves': 'U4',
         'order': 'pin', 'rotation': 'shared'}
    r.update(over)
    return r


def _rejects(raw, why):
    try:
        fp.intent_from_dict(raw)
    except fp.IntentError as exc:
        msg = str(exc)
        assert why in msg, (why, msg)
        return msg
    raise AssertionError(f"NOT REFUSED, expected {why!r}: {raw!r}")


# --------------------------------------------------------------------------
# load: the schema and its refusals
# --------------------------------------------------------------------------

def test_the_new_keys_load_and_land_on_the_intent():
    raw = _base(min_reader=7,
                blocks=[{'name': 'mcu', 'refs': ['U4'], 'rigid': True},
                        {'name': 'rest', 'refs': ['C1']}],
                arrays=[_row(pitch_mm=19.05, axis='x', allow_mixed=False,
                             why='pull-ups on U4', note='n', source='brief',
                             context={'k': 'v'})],
                fixed_poses=[{'ref': 'J1', 'x': 1.0, 'y': 2.0, 'rot': 90,
                              'side': 'F', 'basis': 'declared', 'why': 'w',
                              'context': {}}])
    it = fp.intent_from_dict(raw)
    assert it.blocks[0].rigid is True and it.blocks[1].rigid is False
    assert it.arrays[0]['members'] == ['R6', 'R7', 'R8'], it.arrays
    assert it.fixed_poses[0]['ref'] == 'J1', it.fixed_poses
    # JSON round trip: the dumped document loads to the same claims.
    again = fp.intent_from_dict(json.loads(json.dumps(raw)))
    assert again.arrays == it.arrays and again.fixed_poses == it.fixed_poses
    # An intent declaring none of it behaves as before.
    plain = fp.intent_from_dict(_base())
    assert plain.arrays == () and plain.fixed_poses == ()
    assert not fp._wants(plain, 'array_formation')
    assert fp._wants(it, 'array_formation')
    print("  PASS: arrays / fixed_poses / rigid load, round-trip, and arm "
          "array_formation only when declared")


#: (raw, the reason the refusal must carry). Each names the member.
_REFUSALS = [
    (_base(arrays=[_row(members=['R6'])]), 'at least two'),
    (_base(arrays=[_row(members=['R6', 'R6'])]), 'R6 listed twice'),
    (_base(arrays=[_row(), _row(name='b', members=['R8', 'R9'])]),
     "member R8 is also in array 'pullups'"),
    (_base(arrays=[_row()], must_lock=['R*']),
     "member R6 is named by must_lock 'R*'"),
    (_base(arrays=[_row()],
           fixed_poses=[{'ref': 'R7', 'x': 0, 'y': 0, 'basis': 'declared'}]),
     'member R7 has a `fixed_poses` entry'),
    (_base(arrays=[_row()], edge_connectors=[{'ref': 'R8', 'edge': 'west'}]),
     'member R8 is also an `edge_connectors` entry'),
    (_base(arrays=[_row(serves=None)]), '`serves` is absent'),
    (_base(arrays=[_row(serves='unknown')]), '`serves` is unknown'),
    (_base(arrays=[_row(serves='R6')]), 'is a member of the row it serves'),
    (_base(arrays=[_row(order='bus')]), "order: 'bus'"),
    (_base(arrays=[_row(rotation='any')]), 'expected a number of degrees'),
    (_base(arrays=[_row(pitch_mm=0)]), "expected 'auto' or a positive"),
    (_base(arrays=[_row(axis='z')]), "axis: 'z'"),
    (_base(arrays=[_row(allow_mixed='yes')]), 'allow_mixed: expected true'),
    (_base(arrays=[_row(membres=['R1'])]), 'unknown key(s) membres'),
    (_base(arrays=[_row(), _row(members=['R9', 'R11'])]),
     "duplicate array name 'pullups'"),
    (_base(fixed_poses=[{'ref': 'J1', 'x': 0, 'y': 0}]), 'basis'),
    (_base(fixed_poses=[{'ref': 'J1', 'y': 0, 'basis': 'declared'}]),
     'needs `x`'),
    (_base(fixed_poses=[{'ref': 'J1', 'x': 0, 'y': 0, 'basis': 'declared',
                         'side': 'top'}]), "side: 'top'"),
    (_base(fixed_poses=[{'ref': 'J1', 'x': 0, 'y': 0, 'basis': 'declared'},
                        {'ref': 'J1', 'x': 1, 'y': 1, 'basis': 'declared'}]),
     'duplicate fixed pose'),
    (_base(fixed_poses=[{'ref': 'J1', 'x': 0, 'y': 0, 'basis': 'declared'}],
           edge_connectors=[{'ref': 'J1', 'edge': 'west'}]),
     'J1 is also an `edge_connectors` entry'),
    (_base(fixed_poses=[{'ref': 'MH1', 'x': 0, 'y': 0, 'basis': 'declared'}],
           must_lock=['MH*']),
     "MH1 is also named by must_lock 'MH*'"),
    (_base(fixed_poses=[{'ref': 'J1', 'x': 0, 'y': 0, 'basis': 'guess'}]),
     "basis: 'guess'"),
    (_base(blocks=[{'name': 'b', 'refs': ['U1'], 'rigid': 'yes'}]),
     "rigid 'yes'"),
    (_base(blocks=[{'name': 'fixed:U1', 'refs': ['U1']}]), 'is reserved'),
    # Phase-1 verifier, finding 4: NaN / infinity loaded and then passed
    # every rotation check.
    (_base(arrays=[_row(rotation=float('nan'))]), 'not a finite angle'),
    (_base(arrays=[_row(rotation=float('inf'))]), 'not a finite angle'),
    (_base(blocks=[{'name': 'b', 'refs': ['U1'],
                    'rotation': float('nan')}]), 'not a finite angle'),
    (_base(blocks=[{'name': 'b', 'refs': ['U1'],
                    'rotation_candidates': [0, float('inf')]}]),
     'not a finite angle'),
    (_base(fixed_poses=[{'ref': 'J1', 'x': 0, 'y': 0, 'basis': 'declared',
                         'rot': float('nan')}]), 'not a finite angle'),
    (_base(fixed_poses=[{'ref': 'J1', 'x': float('nan'), 'y': 0,
                         'basis': 'declared'}]), 'not a finite coordinate'),
    (_base(fixed_poses=[{'ref': 'J1', 'x': 0, 'y': float('inf'),
                         'basis': 'declared'}]), 'not a finite coordinate'),
    (_base(arrays=[_row(pitch_mm=float('inf'))]),
     "expected 'auto' or a positive"),
    (_base(arrays=[_row(pitch_mm=float('nan'))]),
     "expected 'auto' or a positive"),
]


def test_every_load_refusal_carries_its_reason():
    for raw, why in _REFUSALS:
        _rejects(raw, why)
    print(f"  PASS: {len(_REFUSALS)} malformed arrays / fixed_poses / rigid "
          f"refused, each for its stated reason")


def test_a_reader_6_file_still_loads_and_a_reader_7_claim_refuses_old():
    # 8 since #1142 (`decaps.within_radius_refs`); arrays still need 7.
    assert fp.READER_VERSION == 8, fp.READER_VERSION
    fp.intent_from_dict(_base(min_reader=6,
                              blocks=[{'name': 'b', 'refs': ['U1']}]))
    saved = fp.READER_VERSION
    try:
        # A reader-6 build handed a document that says it needs reader 7.
        fp.READER_VERSION = 6
        _rejects(_base(min_reader=7, arrays=[_row()]),
                 'this build is reader 6')
    finally:
        fp.READER_VERSION = saved
    print("  PASS: a reader-6 intent loads at 7; min_reader 7 refuses on a "
          "reader-6 build")


# --------------------------------------------------------------------------
# the predicate and the pin order
# --------------------------------------------------------------------------

def _poses(spec):
    return [{'ref': r, 'x': x, 'y': y, 'rot': rot} for r, x, y, rot in spec]


def test_the_formation_predicate():
    clean = _poses([('A', 0, 0, 90), ('B', 2, 0, 90), ('C', 4, 0, 90)])
    v = arr.formation(clean, order_key=['A', 'B', 'C'],
                      rotation_spec='shared')
    assert v['formed'] and v['axis'] == 'x', v
    # Either direction along the axis is the same row.
    v = arr.formation(clean, order_key=['C', 'B', 'A'], rotation_spec=90)
    assert v['formed'], v
    # Shuffled order.
    v = arr.formation(clean, order_key=['B', 'A', 'C'])
    assert v['failed'] == ['order'], v
    # Mixed rotation, both as `shared` and against a declared angle.
    mixed = _poses([('A', 0, 0, 90), ('B', 2, 0, 0), ('C', 4, 0, 90)])
    assert arr.formation(mixed, rotation_spec='shared')['failed'] == \
        ['rotation']
    v = arr.formation(mixed, rotation_spec=90)
    assert v['failed'] == ['rotation'] and v['checks']['rotation']['off'] \
        == ['B'], v
    # Off the axis.
    off = _poses([('A', 0, 0, 0), ('B', 2, 1.0, 0), ('C', 4, 0, 0)])
    v = arr.formation(off, axis_spec='x')
    assert 'axis' in v['failed'], v
    # Uneven pitch, and a declared pitch the row does not have.
    uneven = _poses([('A', 0, 0, 0), ('B', 2, 0, 0), ('C', 7, 0, 0)])
    assert arr.formation(uneven)['failed'] == ['pitch']
    assert arr.formation(clean, pitch_spec=3.0)['failed'] == ['pitch']
    assert arr.formation(clean, pitch_spec=2.0)['formed']
    # Two members on one spot are even, and not a row.
    stacked = _poses([('A', 0, 0, 0), ('B', 0, 0, 0)])
    assert 'pitch' in arr.formation(stacked)['failed']
    # Not declared is UNCHECKED, never passed silently.
    v = arr.formation(clean)
    assert set(v['unchecked']) == {'order', 'rotation'}, v
    print("  PASS: formation passes a clean row (either direction) and fails "
          "shuffled order, mixed rotation, off-axis and uneven pitch")


def test_pin_order_is_read_off_pads_and_nets():
    order, unresolved = arr.pin_order(_pcb(), 'U4', PULLUPS)
    assert order == PIN_ORDER and not unresolved, (order, unresolved)
    # Pose-blind: shuffling every member's pose changes nothing.
    pcb = _fresh()
    for i, r in enumerate(PULLUPS):
        f = pcb.footprints[r]
        f.x, f.y, f.rotation = 10.0 * i, 3.0 * (i % 2), 90.0 * (i % 4)
    assert arr.pin_order(pcb, 'U4', PULLUPS) == (PIN_ORDER, {})
    # A member with no net of its own on the served part is unresolved,
    # named, and never ordered; an absent served part resolves nothing.
    # R5 (an LED resistor) shares no net with U4.
    _o, un = arr.pin_order(_pcb(), 'U4', ['R6', 'R5'])
    assert un == {'R5': 'shares no net with U4'} and _o == ['R6'], (_o, un)
    _o, un = arr.pin_order(_pcb(), 'ZZ9', ['R6', 'R7'])
    assert _o == [] and set(un) == {'R6', 'R7'}, un
    assert sorted(['A10', 'B1', 'A2', '10', '2'], key=arr.natural_key) == \
        ['2', '10', 'A2', 'A10', 'B1']
    print(f"  PASS: pin order {PIN_ORDER} off U4's pad numbering, identical "
          f"with every member's pose shuffled")


# --------------------------------------------------------------------------
# grade: array_formation and the board-aware findings
# --------------------------------------------------------------------------

def _grade(raw, pcb=None, **kw):
    pcb = pcb or _pcb()
    return fp.grade(fp.intent_from_dict(raw), pcb, SPLITFLAP, **kw)


def test_array_formation_grades_the_as_built_board():
    r = _grade(_base(arrays=[_row()]))
    assert 'array_formation' in r.rules_run, r.rules_run
    assert not [v for v in r.violations if v.rule.startswith('array')], \
        [v.message for v in r.violations]
    # R9 sits 66 mm further on: the pitch breaks, and the finding names it.
    r = _grade(_base(arrays=[_row(members=['R6', 'R7', 'R8', 'R9'])]))
    found = [v for v in r.violations if v.rule == 'array_formation']
    assert len(found) == 1 and found[0].block == 'pullups', found
    assert found[0].measured['failed'] == ['pitch'], found[0].measured
    # A declared order the board does not have.
    r = _grade(_base(arrays=[_row(order='declared',
                                  members=['R7', 'R6', 'R8'])]))
    found = [v for v in r.violations if v.rule == 'array_formation']
    assert found and 'order' in found[0].measured['failed'], found
    # Dark when nothing is declared.
    r = _grade(_base())
    assert r.rules_skipped.get('array_formation') == \
        'the intent declares no arrays', r.rules_skipped
    print("  PASS: R6..R8 grade as a formed pin-order row; adding R9 fails "
          "the pitch; a wrong declared order fails; dark when undeclared")


def test_board_aware_findings_name_the_member():
    pcb = _fresh()
    pcb.footprints['R7'].locked = True
    raw = _base(
        blocks=[{'name': 'z', 'refs': ['R6'], 'zone': [150, 40, 200, 50]},
                {'name': 'rot', 'refs': ['R8'], 'rotation': 90}],
        arrays=[_row(rotation=0),
                {'name': 'mixed', 'members': ['R9', 'C1', 'ZZ9'],
                 'serves': 'ZZ8', 'order': 'declared'}])
    it = fp.intent_from_dict(raw)
    blocks, _ = fp.resolve_blocks(it, pcb)
    probs = fp.array_problems(it, pcb, blocks)
    got = {(v.rule, v.ref) for v in probs}
    # R7 is locked AND outside the zone R6's block puts the row in; R8's
    # block turns it to 90 against the row's 0 AND leaves it outside the
    # zone; C1 is a capacitor in a row of resistors.
    want = {('array_unresolved', None), ('array_unresolved', 'ZZ9'),
            ('array_conflict', 'R7'), ('array_conflict', 'C1'),
            ('array_conflict', 'R8')}
    assert want <= got, sorted(got ^ want)
    msgs = ' | '.join(v.message for v in probs)
    for frag in ('member R7 is locked', 'member C1 is', "member R8's block "
                 'declares rotation 90', 'serves ZZ8', 'member ZZ9 is not on',
                 'in zoned block'):
        assert frag in msgs, (frag, msgs)
    # BOTH reach points raise them: the gate the quenching CLIs run...
    _bundle, gate_probs = fp.resolve_intent_gate(it, pcb, ())
    assert {(v.rule, v.ref) for v in gate_probs} >= want, gate_probs
    # ...and the grade (the file lock here is the in-memory one).
    r = fp.grade(it, pcb, SPLITFLAP)
    assert {(v.rule, v.ref) for v in r.violations} >= want, r.violations
    # allow_mixed lifts only the footprint finding.
    raw2 = _base(arrays=[{'name': 'mixed', 'members': ['R9', 'C1'],
                          'order': 'declared', 'allow_mixed': True}])
    it2 = fp.intent_from_dict(raw2)
    assert not fp.array_problems(it2, _pcb(), {}), 'allow_mixed ignored'
    print(f"  PASS: {len(want)} board-aware findings, each naming its member, "
          f"from both grade and resolve_intent_gate")


def test_the_gate_bundle_carries_the_new_data():
    raw = _base(blocks=[{'name': 'mcu', 'refs': ['U4', 'C1'], 'rigid': True},
                        {'name': 'free', 'refs': ['C2']}],
                arrays=[_row()],
                fixed_poses=[{'ref': 'J3', 'x': 1, 'y': 2,
                              'basis': 'declared'}])
    it = fp.intent_from_dict(raw)
    b, _ = fp.resolve_intent_gate(it, _pcb(), ())
    assert b['rigid_blocks'] == {'array:pullups': ['R6', 'R7', 'R8'],
                                 'block:mcu': ['C1', 'U4']}, b['rigid_blocks']
    assert b['arrays'][0]['order_refs'] == ['R6', 'R7', 'R8'], b['arrays']
    assert b['fixed_poses'][0]['ref'] == 'J3'
    # The keys every existing consumer reads are unchanged.
    plain, _ = fp.resolve_intent_gate(fp.intent_from_dict(_base()), _pcb(),
                                      ())
    assert plain['arrays'] == () and plain['fixed_poses'] == () \
        and plain['rigid_blocks'] == {} and plain['tethers'] == {}
    # #1043 (phase 4) added `tethers`: the declared tether limits, `{}` when
    # none is declared.
    assert set(plain) == {'rotations', 'zones', 'keepouts', 'lock_refs',
                          'arrays', 'fixed_poses', 'rigid_blocks', 'tethers'}
    print("  PASS: resolve_intent_gate carries arrays, fixed_poses and "
          "rigid_blocks as data")


# --------------------------------------------------------------------------
# fixed poses are graded, whatever their source (#1054 correction)
# --------------------------------------------------------------------------

def _u4_pose(dx=0.0):
    u = _pcb().footprints['U4']
    return {'ref': 'U4', 'x': u.x + dx, 'y': u.y, 'rot': u.rotation,
            'basis': 'declared', 'why': 'test'}


def test_every_fixed_pose_is_graded():
    r = _grade(_base(fixed_poses=[_u4_pose()]))
    assert not [v for v in r.violations
                if (v.block or '').startswith('fixed:')], r.violations
    r = _grade(_base(fixed_poses=[_u4_pose(5.0)]))
    hit = [v for v in r.violations if v.block == 'fixed:U4']
    assert len(hit) == 1 and hit[0].rule == 'zone_containment' \
        and hit[0].ref == 'U4' and hit[0].severity == 'error', r.violations
    # A ref the board does not have is an error, never a clean pass.
    r = _grade(_base(fixed_poses=[{'ref': 'ZZ9', 'x': 0, 'y': 0,
                                   'basis': 'declared'}]))
    assert [(v.rule, v.severity) for v in r.violations
            if v.ref == 'ZZ9'] == [('fixed_pose_unresolved', 'error')]
    # A pad-less part cannot be anchored: a WARN that says why.
    pcb = _fresh()
    pcb.footprints['H6'].pads = []
    h = pcb.footprints['H6']
    r = _grade(_base(fixed_poses=[{'ref': 'H6', 'x': h.x, 'y': h.y,
                                   'basis': 'mechanical'}]), pcb=pcb)
    w = [v for v in r.violations if v.ref == 'H6']
    assert [(v.rule, v.severity) for v in w] == [
        ('fixed_pose_unresolved', 'warn')] and 'pad-less' in w[0].message, w
    # A ref the mechanical file anchors is graded ONCE, by the file.
    mech = {'path': 'm.json', 'poses': {'U4': {
        'x': _u4_pose(5.0)['x'], 'y': _u4_pose()['y'], 'rot': 0.0,
        'reason': 'r'}}}
    r = _grade(_base(fixed_poses=[_u4_pose(5.0)]), mechanical=mech)
    blocks = sorted(v.block for v in r.violations
                    if v.rule == 'zone_containment')
    assert blocks == ['mech:U4'], blocks
    _brief_pose_contradicting_mechanical_is_graded()
    print("  PASS: a fixed pose grades clean at its pose, fails 5 mm off, "
          "refuses a missing ref, warns on a pad-less one, is graded once "
          "when the mechanical file anchors the SAME pose, and a brief pose "
          "contradicting mechanical.json is a contradiction AND graded")


def _brief_pose_contradicting_mechanical_is_graded():
    """Phase-1 verifier, finding 1, as reported: on esp_prog mechanical.json
    puts U1 at its board pose and the brief's `fixed[U1].pose` 13 mm away.
    The emit reported 0 contradictions and the grade PASSED -- the brief
    pose was skipped because the file named the ref, whatever its pose."""
    from placement import reconcile as rc
    pcb = _pcb(ESP)
    u1 = pcb.footprints['U1']
    here = (u1.x, u1.y, (u1.rotation or 0.0) % 360.0)
    with tempfile.TemporaryDirectory() as tmp:
        mpath = os.path.join(tmp, 'mechanical.json')
        with open(mpath, 'w', encoding='utf-8') as fh:
            json.dump({'kind': 'mechanical-declaration', 'schema': 1,
                       'refs': {'U1': list(here)},
                       'reasons': {'U1': 'test: mechanical says here'}}, fh)
        mech = rc.load_mechanical(mpath)
        frag, _rep = db.compile_brief(_brief(fixed=[{
            'ref': 'U1', 'why': 'brief says elsewhere',
            'pose': {'x': here[0] + 13.0, 'y': here[1] - 10.0, 'rot': 0,
                     'side': 'F'}}]), board_refs=sorted(pcb.footprints))
        rows = rc.reconcile(pcb, ESP, brief_fragment=frag,
                            brief_source='b.design-brief.json',
                            mechanical=mech)
        con = [r for r in rows if r['id'] == 'U1:pose']
        assert con and con[0]['kind'] == 'contradiction' \
            and 'brief' in con[0]['values'], rows
        lost = rc.lost_mechanical_refs(rows)
        it = fp.intent_from_dict(_base(fixed_poses=frag['fixed_poses']))
        for skip in (lost, ()):
            # Whichever value wins, the brief's pose is graded: U1 sits at
            # the FILE's pose, 13 mm from the brief's.
            r = fp.grade(it, pcb, ESP, mechanical=mech, mechanical_skip=skip)
            hit = [v for v in r.violations if v.block == 'fixed:U1']
            assert hit and hit[0].severity == 'error', (skip, r.violations)
        # A basis:mechanical entry carrying the file's pose, for a ref whose
        # file value LOST, is an error: nothing else grades it.
        stale = fp.intent_from_dict(_base(fixed_poses=[{
            'ref': 'U1', 'x': here[0], 'y': here[1], 'rot': here[2],
            'basis': 'mechanical'}]))
        r = fp.grade(stale, pcb, ESP, mechanical=mech, mechanical_skip=['U1'])
        bad = [v for v in r.violations if v.rule == 'fixed_pose_unresolved']
        assert bad and bad[0].severity == 'error' \
            and 'LOST a contradiction' in bad[0].message, r.violations


# --------------------------------------------------------------------------
# the brief: compile, unknown vs absent, merge, drift, coverage
# --------------------------------------------------------------------------

BRIEF = {'schema': 1, 'kind': 'design-brief', 'units': 'mm'}


def _brief(**over):
    raw = dict(BRIEF)
    raw.update(over)
    return db.brief_from_dict(raw, 'b.design-brief.json')


def _brief_rejects(why, **over):
    try:
        _brief(**over)
    except db.BriefError as exc:
        assert why in str(exc), (why, str(exc))
        return
    raise AssertionError(f"NOT REFUSED, expected {why!r}: {over!r}")


def test_the_brief_compiles_arrays_and_fixed_poses():
    b = _brief(arrays=[_row(requirement='one bus, one row'),
                       {'name': 'r2', 'members': ['R9', 'R11'],
                        'order': 'unknown', 'serves': 'unknown'}],
               fixed=[{'ref': 'U4', 'why': 'datum',
                       'pose': {'x': 1.5, 'y': 2.5, 'rot': 'unknown'}},
                      {'ref': 'H1', 'why': 'boss'}])
    refs = sorted(_pcb().footprints)
    frag, rep = db.compile_brief(b, board_refs=refs)
    assert frag['min_reader'] == 7, frag
    a0 = frag['arrays'][0]
    assert a0['source'] == 'brief' and 'requirement' not in a0 \
        and a0['context']['requirement'] == 'one bus, one row', a0
    assert frag['fixed_poses'] == [{'ref': 'U4', 'x': 1.5, 'y': 2.5,
                                    'rot': 'unknown', 'basis': 'declared',
                                    'why': 'datum'}], frag['fixed_poses']
    # "unknown" is reported; an ABSENT key is neither declared nor unknown.
    assert 'arrays[r2].order' in rep['unknown'] \
        and 'arrays[r2].serves' in rep['unknown'], rep['unknown']
    assert 'fixed[U4].rot' in rep['unknown'], rep['unknown']
    assert 'arrays[pullups].order' in rep['declared'], rep['declared']
    for cid in ('arrays[r2].rotation', 'arrays[pullups].pitch_mm',
                'fixed[U4].side'):
        assert cid not in rep['declared'] and cid not in rep['unknown'], cid
    assert rep['counts']['arrays'] == 2 and rep['counts']['fixed_poses'] == 1
    assert 'H1' not in {f['ref'] for f in frag['fixed_poses']}
    # Every clause id the compiler emits parses back.
    for cid in rep['declared'] + [u for u in rep['unknown'] if '[' in u]:
        assert db.parse_clause_id(cid) is not None, cid
    # The compiled fragment loads as an intent.
    it = fp.intent_from_dict(_base(**{k: v for k, v in frag.items()}))
    assert len(it.arrays) == 2 and len(it.fixed_poses) == 1
    # A brief with neither keeps the counts it always had.
    _f, rep0 = db.compile_brief(_brief(), board_refs=refs)
    assert 'arrays' not in rep0['counts'] and 'fixed_poses' not in \
        rep0['counts'] and 'min_reader' not in _f, (rep0['counts'], _f)
    print("  PASS: arrays and fixed[].pose compile 1:1 at min_reader 7; "
          "unknown and absent reported apart")


def test_the_brief_carries_a_courtyard_waiver_and_drift_sees_it():
    """#1060: `fixed[].accept_courtyard_overlap` compiles through to the
    intent entry as written, drift compares it, and a row with NO pose that
    carries one is refused (there is no pose for it to waive at)."""
    b = _brief(fixed=[{'ref': 'U4', 'why': 'datum', 'pose': {'x': 1.5,
                                                             'y': 2.5},
                       'accept_courtyard_overlap': ['R6']}])
    frag, _rep = db.compile_brief(b, board_refs=sorted(_pcb().footprints))
    assert frag['fixed_poses'][0]['accept_courtyard_overlap'] == ['R6'], frag
    it = fp.intent_from_dict(_base(**frag))
    assert ('U4', 'R6') in it.courtyard_waiver_pairs(), it
    # A COURTYARD waiver only: it is not an overlap_waivers[] pair, whose
    # consumers exempt the drawn-body containment gate too (phase-4 verifier).
    assert ('U4', 'R6') not in it.waiver_pairs(), it.waiver_pairs()
    doc = dict(_base(**frag))
    doc['fixed_poses'] = [dict(frag['fixed_poses'][0],
                               accept_courtyard_overlap=['R7'])]
    lines = db.drift(doc, frag)
    assert any('accept_courtyard_overlap' in ln for ln in lines), lines
    _brief_rejects('declares none',
                   fixed=[{'ref': 'H1', 'why': 'w',
                           'accept_courtyard_overlap': ['R6']}])
    print("  PASS: the waiver compiles 1:1, reaches courtyard_waiver_pairs "
          "(not waiver_pairs), drifts, and "
          "a pose-less row carrying one is refused")


def test_the_brief_refuses_what_the_intent_refuses():
    _brief_rejects('member R8 is also in array',
                   arrays=[_row(), _row(name='b', members=['R8', 'R9'])])
    _brief_rejects('also declared in interfaces[]',
                   interfaces=[{'ref': 'J1', 'edge': 'west'}],
                   fixed=[{'ref': 'J1', 'pose': {'x': 0, 'y': 0}}])
    _brief_rejects('member R6 has a `fixed_poses` entry',
                   arrays=[_row()],
                   fixed=[{'ref': 'R6', 'pose': {'x': 0, 'y': 0}}])
    _brief_rejects('needs `y`', fixed=[{'ref': 'J1', 'pose': {'x': 0}}])
    _brief_rejects("side: 'top'",
                   fixed=[{'ref': 'J1', 'pose': {'x': 0, 'y': 0,
                                                 'side': 'top'}}])
    _brief_rejects('unknown key(s) reqs', arrays=[_row(reqs='x')])
    _brief_rejects('member R7 is also an `edge_connectors` entry',
                   arrays=[_row()], interfaces=[{'ref': 'R7',
                                                 'edge': 'north'}])
    print("  PASS: the brief refuses a doubled member, a pose on an "
          "interface, a posed array member and malformed poses, by name")


def test_merge_drift_and_coverage():
    b = _brief(arrays=[_row()],
               fixed=[{'ref': 'U4', 'pose': {'x': 144.78, 'y': 48.26,
                                             'rot': 0}}])
    frag, rep = db.compile_brief(b, board_refs=sorted(_pcb().footprints))
    emitted = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
               'must_lock': ['U4'],
               'edge_connectors': [{'ref': 'R6', 'edge': 'north',
                                    'source': 'observed'}]}
    merged = db.merge_into_intent(emitted, frag, rep)
    assert merged['arrays'] == frag['arrays']
    assert merged['fixed_poses'] == frag['fixed_poses']
    assert merged['min_reader'] == 7
    # The emitter's inferences for the same parts are DROPPED, and said so.
    assert merged['edge_connectors'] == [] and merged['must_lock'] == [], \
        merged
    joined = ' | '.join(rep['contradictions'])
    assert 'R6: the brief puts it in array' in joined \
        and "must_lock 'U4' names U4" in joined, joined
    it = fp.intent_from_dict(merged)
    assert it.arrays and it.fixed_poses
    # No drift against itself; drift when the intent lost or changed a row.
    assert db.drift_pairs(merged, frag) == []
    lost = dict(merged, arrays=[], fixed_poses=[])
    ids = db.drifted_clause_ids(lost, frag)
    assert ids == ['arrays[pullups].members', 'fixed[U4].pose'], ids
    moved = json.loads(json.dumps(merged))
    moved['arrays'][0]['order'] = 'declared'
    moved['fixed_poses'][0]['x'] = 150.0
    ids = db.drifted_clause_ids(moved, frag)
    assert ids == ['arrays[pullups].order', 'fixed[U4].pose'], ids
    # Coverage: graded when carried and graded, uncovered when dropped.
    res = fp.grade(it, _pcb(), SPLITFLAP)
    cov = db.clause_coverage(rep, merged, rules_run=res.rules_run,
                             abstained=res.budget_abstained)
    st = {c['id']: c['state'] for c in cov['clauses']}
    assert st['arrays[pullups].members'] == 'graded', st
    assert st['fixed[U4].pose'] == 'graded', st
    cov = db.clause_coverage(rep, lost, rules_run=res.rules_run)
    st = {c['id']: c['state'] for c in cov['clauses']}
    assert st['arrays[pullups].members'] == 'uncovered' \
        and st['fixed[U4].pose'] == 'uncovered', st
    # The ledger attributes a formation failure to the ARRAY clause.
    bad = json.loads(json.dumps(merged))
    bad['arrays'][0]['members'] = ['R6', 'R7', 'R8', 'R9']
    b2 = _brief(arrays=[_row(members=['R6', 'R7', 'R8', 'R9'])])
    frag2, rep2 = db.compile_brief(b2, board_refs=sorted(_pcb().footprints))
    it2 = fp.intent_from_dict(bad)
    res2 = fp.grade(it2, _pcb(), SPLITFLAP, with_roster=True,
                    brief_fragment=frag2)
    cov2 = db.clause_coverage(rep2, bad, rules_run=res2.rules_run)
    led = fp.declaration_ledger(it2, res2.roster, result=res2, coverage=cov2)
    row = [x for x in led if x['id'] == 'arrays[pullups].members']
    assert row and row[0]['status'] == 'graded_fail', row
    print("  PASS: merge appends and drops the emitter's colliding claims; "
          "drift and coverage see both keys; the ledger fails the array "
          "clause")


# --------------------------------------------------------------------------
# emit: nothing by default; the rigid path; mechanical poses compile
# --------------------------------------------------------------------------

def test_emit_intent_writes_none_of_it_by_default():
    doc = fp.emit_intent(_pcb(), SPLITFLAP)
    for key in ('arrays', 'fixed_poses', 'min_reader'):
        assert key not in doc, key
    assert not any('rigid' in b for b in doc['blocks'])
    # Phase 2 made 'auto' real (tests/test_1051_suggest_arrays.py grades
    # it); a value that is neither still refuses, by name.
    try:
        fp.emit_intent(_pcb(), SPLITFLAP, derive_arrays='yes')
    except ValueError as exc:
        assert "derive_arrays 'yes'" in str(exc), exc
    else:
        raise AssertionError("derive_arrays='yes' was accepted")
    # splitflap emits no blocks; glasgow emits sheet blocks.
    gl = _pcb(GLASGOW)
    base = fp.emit_intent(gl, GLASGOW)
    assert not any('rigid' in b for b in base['blocks'])
    assert 'min_reader' not in base
    name = base['blocks'][0]['name']
    ron = fp.emit_intent(gl, GLASGOW, rigid_blocks=(name,))
    assert [b['name'] for b in ron['blocks'] if b.get('rigid')] == [name]
    assert ron['min_reader'] == 7
    rigid = [z.name for z in fp.intent_from_dict(ron).blocks if z.rigid]
    assert rigid == [name], rigid
    try:
        fp.emit_intent(_pcb(), SPLITFLAP, rigid_blocks=('no-such-block',))
    except ValueError as exc:
        assert 'no-such-block' in str(exc), exc
    else:
        raise AssertionError('an unknown rigid block name was accepted')
    print(f"  PASS: emit_intent writes no arrays/fixed_poses/rigid by "
          f"default; rigid_blocks=({name!r},) marks exactly that block")


def test_mechanical_poses_compile_into_fixed_poses():
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'emitted.json')
        check([sys.executable, '-X', 'utf8', CHECK_FLOORPLAN, ESP,
               '--emit-intent', out, '--brief', ESP_BRIEF,
               '--mechanical', ESP_MECH, '-q'], accept=True)
        with open(out, encoding='utf-8') as fh:
            doc = json.load(fh)
        mech = doc['context']['mechanical']
        assert mech['fixed_poses'] == ['Ref*', 'Ref*~2'], mech
        assert set(mech['fixed_skipped']) == {'USB1'}, mech['fixed_skipped']
        rows = {f['ref']: f for f in doc['fixed_poses']}
        assert rows['Ref*']['basis'] == 'mechanical' \
            and (rows['Ref*']['x'], rows['Ref*']['y']) == (141.2, 95.9), rows
        assert doc['min_reader'] == 7
        # It loads and grades: the as-built fiducials sit at their poses.
        check([sys.executable, '-X', 'utf8', CHECK_FLOORPLAN, ESP,
               '--intent', out, '--brief', ESP_BRIEF, '--no-mechanical',
               '-q'], accept=True)
    print("  PASS: mechanical.json anchors compile to fixed_poses (USB1, an "
          "edge connector, skipped by name) and the intent grades")


# --------------------------------------------------------------------------
# Phase-1 verifier, findings 2, 3, 5 and 6
# --------------------------------------------------------------------------

def test_an_order_nobody_resolves_is_unchecked_not_passed():
    """Finding 2: with fewer than two members resolved the order check
    compared [] to [] and passed; on splitflap `serves: J3` graded clean."""
    v = arr.formation(_poses([('A', 0, 0, 0), ('B', 2, 0, 0)]),
                      order_key=['B'])
    assert 'order' in v['unchecked'] and v['checks']['order']['ok'] is None, v
    raw = _base(arrays=[_row(serves='J3')])
    r = _grade(raw)
    assert r.budget_abstained.get('arrays[pullups].order', '').startswith(
        "order 'pin' places only 0 member(s)"), r.budget_abstained
    assert not r.complete, 'an unresolved order read as a complete grade'
    row = r.array_measured[0]
    assert set(row['order_unresolved']) == {'R6', 'R7', 'R8'}, row
    # Partial resolution is disclosed on a PASS too: R8's pads taken off
    # every net, R6 and R7 still resolve and still form a row.
    pcb = _fresh()
    for pd in pcb.footprints['R8'].pads:
        pd.net_id = 0
    r = _grade(_base(arrays=[_row()]), pcb=pcb)
    assert not [v for v in r.violations if v.rule == 'array_formation'], \
        r.violations
    row = r.array_measured[0]
    assert row['formed'] and row['order_expected'] == ['R6', 'R7'] \
        and row['order_unresolved'] == {'R8': 'shares no net with U4'}, row
    assert 'order unresolved for R8' in fp.format_text(r)
    assert fp.to_json(r)['array_formation'][0]['name'] == 'pullups'
    print("  PASS: an order that places < 2 members is unchecked and "
          "abstains; partial resolution is disclosed on a passing grade")


def test_the_single_pad_net_decides_the_pin_order():
    """Finding 3: the splitflap case cannot tell `single or own` from `own`
    (R14's GND lands on U4 8 and 15, its signal on 10 -- both sort between
    pins 4 and 11). Here member A's ground lands on host pads 1 and 9 and
    its signal on 5; B's signal on 3. The signal decides: B (3) then A (5).
    Deciding on every own net would put A first, on its ground pin 1."""
    from types import SimpleNamespace as NS

    def pad(num, net):
        return NS(pad_number=num, net_id=net)
    pcb = NS(footprints={
        'H': NS(pads=[pad('1', 9), pad('9', 9), pad('5', 1), pad('3', 2)]),
        'A': NS(pads=[pad('1', 1), pad('2', 9)]),
        'B': NS(pads=[pad('1', 2), pad('2', 7)]),
    })
    order, un = arr.pin_order(pcb, 'H', ['A', 'B'])
    assert order == ['B', 'A'] and not un, (order, un)
    print("  PASS: a net landing on one pad of the served part outranks a "
          "multi-pad (ground) net")


def test_a_shared_rotation_no_member_can_take_is_a_conflict():
    """Finding 5: `shared` compared only DECIDED angles, so a member
    allowed a candidate set passed against another's decided angle although
    no angle suits both. (R6 {0, 90} against R7 {180}, the verifier's
    example, is NOT a conflict since round 2: both are two-pad resistors,
    the same turned 180 -- `test_two_pad_parts_are_the_same_turned_180`.)
    R6 {0} against R7 {90} is one, modulo 180 too."""
    raw = _base(blocks=[{'name': 'a', 'refs': ['R6'],
                         'rotation_candidates': [0]},
                        {'name': 'b', 'refs': ['R7'], 'rotation': 90}],
                arrays=[_row()])
    it = fp.intent_from_dict(raw)
    blocks, _ = fp.resolve_blocks(it, _pcb())
    probs = [v for v in fp.array_problems(it, _pcb(), blocks)
             if v.rule == 'array_conflict']
    assert {v.ref for v in probs} == {'R6', 'R7'} and \
        'no angle is allowed to every member' in probs[0].message, probs
    # A common angle clears it.
    raw['blocks'][1]['rotation'] = 0
    it = fp.intent_from_dict(raw)
    blocks, _ = fp.resolve_blocks(it, _pcb())
    assert not fp.array_problems(it, _pcb(), blocks)
    print("  PASS: shared rotation with an empty common angle set is an "
          "array_conflict on each member; a common angle clears it")


def test_a_missing_member_skips_the_formation_and_stacked_reads_so():
    """Finding 6: the any-member-missing guard, pinned. R8, R6 and R9 alone
    fail the pitch (gaps 38.1 and 66.04 mm); with ZZ9 missing the rule must
    NOT grade the survivors -- `array_unresolved` owns the finding."""
    r = _grade(_base(arrays=[_row(members=['R8', 'R6', 'R9'],
                                  order='declared')]))
    assert [v for v in r.violations if v.rule == 'array_formation'], \
        'control: the survivors alone should fail'
    r = _grade(_base(arrays=[_row(members=['R8', 'R6', 'R9', 'ZZ9'],
                                  order='declared')]))
    assert not [v for v in r.violations if v.rule == 'array_formation'], \
        r.violations
    assert [v.ref for v in r.violations
            if v.rule == 'array_unresolved'] == ['ZZ9'], r.violations
    assert r.array_measured[0]['formed'] is None \
        and 'ZZ9' in r.array_measured[0]['skipped'], r.array_measured
    # Two members on one spot read as stacked, not as a list of zero gaps.
    pcb = _fresh()
    pcb.footprints['R7'].x = pcb.footprints['R6'].x
    r = _grade(_base(arrays=[_row(members=['R6', 'R7'])]), pcb=pcb)
    msg = [v.message for v in r.violations if v.rule == 'array_formation']
    assert msg and 'members stacked: R6, R7' in msg[0], msg
    print("  PASS: a missing member skips formation (array_unresolved "
          "reports it); stacked members are named as stacked")


# --------------------------------------------------------------------------
# Phase-1 re-verifier (round 2)
# --------------------------------------------------------------------------

def _mech_for(pcb, ref, pose, tmp):
    from placement import reconcile as rc
    mpath = os.path.join(tmp, 'mechanical.json')
    with open(mpath, 'w', encoding='utf-8') as fh:
        json.dump({'kind': 'mechanical-declaration', 'schema': 1,
                   'refs': {ref: list(pose)},
                   'reasons': {ref: 'test'}}, fh)
    return rc.load_mechanical(mpath)


def test_an_agreeing_brief_pose_corroborates_the_mechanical_one():
    """Round 2, findings 1 and 2: brief and mechanical.json agree on U1 at a
    pose 3 mm off the board part. The row is drift the BOARD loses, so
    before a grade it is `pending` -- not `graded_fail` because the agreeing
    mechanical channel was counted as a loser. And the winner (the ledger's
    basis) stays `mechanical`: the physical fact, which the brief
    corroborates."""
    from placement import reconcile as rc
    pcb = _pcb(ESP)
    u1 = pcb.footprints['U1']
    pose = (u1.x + 3.0, u1.y, (u1.rotation or 0.0) % 360.0)
    with tempfile.TemporaryDirectory() as tmp:
        mech = _mech_for(pcb, 'U1', pose, tmp)
        frag, _r = db.compile_brief(_brief(fixed=[{
            'ref': 'U1', 'pose': {'x': pose[0], 'y': pose[1],
                                  'rot': pose[2]}}]),
            board_refs=sorted(pcb.footprints))
        rows = rc.reconcile(pcb, ESP, brief_fragment=frag,
                            brief_source='b.design-brief.json',
                            mechanical=mech)
    row = [r for r in rows if r['id'] == 'U1:pose'][0]
    assert row['kind'] == 'drift' and row['winner'] == 'mechanical' \
        and 'corroborating' in row['why'], row
    assert not rc.lost_mechanical_refs(rows)
    it = fp.intent_from_dict(_base(fixed_poses=frag['fixed_poses']))
    led = fp.declaration_ledger(it, [], result=None, reconciliation=rows)
    lrow = [x for x in led if x['id'] == 'reconcile:U1:pose'][0]
    assert lrow['status'] == 'pending' and lrow['basis'] == 'mechanical', \
        lrow
    print("  PASS: an agreeing brief pose keeps the row pending before a "
          "grade and mechanical as its winner and basis")


def test_the_lost_error_is_for_a_mechanical_entry_only():
    """Round 2, finding 3: the lost-contradiction error named "the
    mechanical.json pose" for ANY entry at the file's pose. A DECLARED entry
    there is the plan's own claim, and since the file value lost, no file
    anchor grades it: it is graded like any other entry."""
    pcb = _pcb(ESP)
    u1 = pcb.footprints['U1']
    pose = (u1.x + 13.0, u1.y, (u1.rotation or 0.0) % 360.0)
    with tempfile.TemporaryDirectory() as tmp:
        mech = _mech_for(pcb, 'U1', pose, tmp)
    for basis, want in (('mechanical', 'fixed_pose_unresolved'),
                        ('declared', 'zone_containment')):
        it = fp.intent_from_dict(_base(fixed_poses=[{
            'ref': 'U1', 'x': pose[0], 'y': pose[1], 'rot': pose[2],
            'basis': basis}]))
        out = fp.fixed_pose_violations(it, pcb, ESP, mechanical=mech,
                                       mechanical_skip=['U1'])
        assert [v.rule for v in out] == [want], (basis, out)
    print("  PASS: only a basis:mechanical entry gets the lost error; a "
          "declared one at the same pose is graded by its own anchor")


def test_a_member_with_no_geometry_gets_a_measured_row():
    """Round 2, finding 4."""
    it = fp.intent_from_dict(_base(arrays=[_row()]))
    ctx = fp._grade_ctx(it, _pcb(), SPLITFLAP)[0]
    del ctx.parts['R8']
    assert list(fp.rule_array_formation(ctx)) == []
    assert ctx.array_measured == [{
        'name': 'pullups', 'formed': None,
        'skipped': ('no placement geometry for R8, so the row cannot be '
                    'measured')}], ctx.array_measured
    assert 'arrays[pullups]' in ctx.abstained
    print("  PASS: a member with no geometry abstains AND is disclosed as a "
          "measured row")


def test_stacked_members_still_report_the_other_gaps():
    """Round 2, finding 5: R7 stacked on R6, and R9 66 mm beyond R8."""
    pcb = _fresh()
    pcb.footprints['R7'].x = pcb.footprints['R6'].x
    r = _grade(_base(arrays=[_row(members=['R6', 'R7', 'R8', 'R9'],
                                  order='declared')]), pcb=pcb)
    msg = [v.message for v in r.violations if v.rule == 'array_formation']
    assert msg and 'members stacked: R6, R7' in msg[0] \
        and 'gaps between the rest [66.04, 38.1]mm' in msg[0], msg
    print("  PASS: a stacked row still names the uneven gaps of the rest")


def test_two_pad_parts_are_the_same_turned_180():
    """Round 2, finding 6 (a design correction from Phase 2's human-board
    measurement): a part with <= 2 copper pads compares its rotation modulo
    180, in the grader AND in the load-time conflict check, so the two
    agree. A part with more pads keeps the exact comparison."""
    two = _poses([('A', 0, 0, 0), ('B', 2, 0, 180), ('C', 4, 0, 0)])
    for p in two:
        p['pads'] = 2
    assert arr.formation(two, rotation_spec='shared')['formed']
    assert arr.formation(two, rotation_spec=0)['formed']
    assert arr.formation(two, rotation_spec=180)['formed']
    three = [dict(p, pads=3) for p in two]
    assert arr.formation(three, rotation_spec='shared')['failed'] == \
        ['rotation']
    assert arr.formation(three, rotation_spec=0)['failed'] == ['rotation']
    # Mixed: a pair is compared at the stricter period.
    mixed = [dict(two[0], pads=2), dict(two[1], pads=3)]
    assert arr.formation(mixed, rotation_spec='shared')['failed'] == \
        ['rotation']
    # Unknown pad count is strict.
    assert not arr.formation(_poses([('A', 0, 0, 0), ('B', 2, 0, 180)]),
                             rotation_spec='shared')['formed']
    # The conflict check agrees: resistors R6 at 0 and R7 at 180 are one
    # shared rotation, and R8's block at 180 does not contradict a row at 0.
    raw = _base(blocks=[{'name': 'a', 'refs': ['R6'], 'rotation': 0},
                        {'name': 'b', 'refs': ['R7'], 'rotation': 180},
                        {'name': 'c', 'refs': ['R8'],
                         'rotation_candidates': [180]}],
                arrays=[_row()])
    it = fp.intent_from_dict(raw)
    blocks, _ = fp.resolve_blocks(it, _pcb())
    assert not fp.array_problems(it, _pcb(), blocks)
    raw['arrays'] = [_row(rotation=0)]
    it = fp.intent_from_dict(raw)
    blocks, _ = fp.resolve_blocks(it, _pcb())
    assert not fp.array_problems(it, _pcb(), blocks)
    # ...and so does the grade, on the board: R7 turned 180 in memory.
    pcb = _fresh()
    pcb.footprints['R7'].rotation = 180.0
    r = _grade(_base(arrays=[_row()]), pcb=pcb)
    assert not [v for v in r.violations if v.rule == 'array_formation'], \
        r.violations
    print("  PASS: two-pad members 180 apart share a rotation (grade and "
          "conflict check); three-pad members do not")


def test_emit_never_fixes_an_array_member_from_mechanical_json():
    """Review item 2: `--emit-intent` compiled a mechanical.json pose into
    `fixed_poses[]` for a ref the brief declares as an ARRAY member, and the
    tool's own loader then refused the intent ("member R3 has a
    `fixed_poses` entry") -- emit exit 0, grade exit 2. A member is now a
    CLAIMED ref: skipped by name in `fixed_skipped`, and the emitted intent
    loads and grades."""
    pcb = _pcb(ESP)
    f = pcb.footprints
    with tempfile.TemporaryDirectory() as tmp:
        mpath = os.path.join(tmp, 'mechanical.json')
        with open(mpath, 'w', encoding='utf-8') as fh:
            json.dump({'kind': 'mechanical-declaration', 'schema': 1,
                       'refs': {r: [f[r].x, f[r].y, f[r].rotation or 0]
                                for r in ('R3', 'R4')},
                       'reasons': {'R3': 't', 'R4': 't'}}, fh)
        bpath = os.path.join(tmp, 'brief.json')
        with open(bpath, 'w', encoding='utf-8') as fh:
            json.dump(dict(BRIEF, arrays=[{
                'name': 'uart', 'members': ['R3', 'R4'], 'serves': 'U1',
                'order': 'pin', 'rotation': 'shared'}]), fh)
        out = os.path.join(tmp, 'emitted.json')
        check([sys.executable, '-X', 'utf8', CHECK_FLOORPLAN, ESP,
               '--emit-intent', out, '--brief', bpath, '--mechanical', mpath,
               '--quiet'], accept=True)
        with open(out, encoding='utf-8') as fh:
            doc = json.load(fh)
        refs = {x['ref'] for x in doc.get('fixed_poses') or ()}
        assert not refs & {'R3', 'R4'}, refs
        # Since the re-review this is a CONTRADICTION the brief wins
        # (`R3:array`), so the mechanical value LOST and neither is anchored
        # at all; a member the file still anchors (a brief the run wrote,
        # which loses) is skipped by the `_claimed` guard instead.
        rows = {r['id']: r for r in doc['context']['reconciliation']}
        mech = doc['context']['mechanical']
        for r in ('R3', 'R4'):
            row = rows.get(f'{r}:array')
            assert row and row['kind'] == 'contradiction' \
                and row['winner'] == 'brief', row
            assert r not in mech['anchored'], mech['anchored']
            assert 'lost a contradiction' in mech['skipped'].get(r, ''), \
                mech['skipped']
        fp.load_intent(out)          # the loader accepts what emit wrote
        check([sys.executable, '-X', 'utf8', CHECK_FLOORPLAN, ESP,
               '--intent', out, '--quiet'], accept=True)
    print("  PASS: R3/R4 (array members) are skipped from mechanical "
          "fixed_poses by name, and the emitted intent loads and grades")


def test_the_ledger_fails_a_fixed_pose_only_on_its_anchor():
    """Review item 5. The ledger's `fixed[REF].pose` clause failed on ANY
    zone_containment error for the ref -- a zone its block declares is a
    different clause. U4 fixed at its own board pose (the anchor holds) and
    in a zoned block far away (the zone fails): the pose clause PASSES and
    the zone's finding stays the zone's. plan_check also WARNS, before any
    write, that the fixed pose lies outside its own block's zone."""
    u = _pcb().footprints['U4']
    b = _brief(fixed=[{'ref': 'U4', 'pose': {'x': u.x, 'y': u.y,
                                             'rot': u.rotation or 0}}])
    frag, rep = db.compile_brief(b, board_refs=sorted(_pcb().footprints))
    raw = _base(fixed_poses=frag['fixed_poses'], min_reader=7,
                blocks=[{'name': 'far', 'refs': ['U4'],
                         'zone': [0.0, 0.0, 5.0, 5.0]}])
    it = fp.intent_from_dict(raw)
    res = fp.grade(it, _pcb(), SPLITFLAP, with_roster=True,
                   brief_fragment=frag)
    zc = [v for v in res.violations if v.rule == 'zone_containment'
          and v.ref == 'U4']
    assert [v.block for v in zc] == ['far'], [(v.block, v.message)
                                             for v in zc]
    cov = db.clause_coverage(rep, raw, rules_run=res.rules_run)
    led = fp.declaration_ledger(it, res.roster, result=res, coverage=cov)
    row = [x for x in led if x['id'] == 'fixed[U4].pose']
    assert row and row[0]['status'] == 'graded_pass', row
    # Control: the pose itself moved 5mm -- the anchor fails, and so does
    # the clause.
    moved = json.loads(json.dumps(raw))
    moved['fixed_poses'][0]['x'] = u.x + 5.0
    it2 = fp.intent_from_dict(moved)
    res2 = fp.grade(it2, _pcb(), SPLITFLAP, with_roster=True,
                    brief_fragment=frag)
    led2 = fp.declaration_ledger(it2, res2.roster, result=res2,
                                 coverage=db.clause_coverage(
                                     rep, moved, rules_run=res2.rules_run))
    row2 = [x for x in led2 if x['id'] == 'fixed[U4].pose']
    assert row2 and row2[0]['status'] == 'graded_fail', row2
    # plan_check: the declared pose is outside its own block's zone.
    found, _meas = fp.plan_check(it, _pcb(), SPLITFLAP)
    w = [v for v in found if v.rule == 'plan_fixed_outside_zone'
         and v.ref == 'U4']
    assert len(w) == 1 and w[0].severity == fp.WARN, [
        (v.rule, v.ref, v.severity) for v in found]
    assert "fixed pose" in w[0].message and "'far'" in w[0].message, \
        w[0].message
    print("  PASS: a zone failure is not the fixed-pose clause's; a moved "
          "pose is; plan_check warns the pose is outside its own zone")


def test_plan_check_measures_a_fixed_pose_on_its_declared_face():
    """Re-review item 3: plan_check's fixed-pose zone check measured the
    part's courtyard on the face it is on NOW. A declared `side` flips it
    (the writer mirrors local y), so a courtyard that extends only +y on F
    extends -y on B. The zone holds the F outline and not the B one: the
    WARN must follow the DECLARED side."""
    board = """(kicad_pcb (version 20241229) (net 0 "") (net 1 "/A")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
 (gr_rect (start 0 0) (end 30 30) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "t:P" (layer "F.Cu") (uuid "fp-P1") (at 5 5)
  (property "Reference" "P1" (at 0 0 0))
  (fp_rect (start -1 0) (end 1 4) (layer "F.CrtYd") (uuid "c1"))
  (pad "1" smd rect (at 0 1) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A") (uuid "p1")))
 (footprint "t:Q" (layer "F.Cu") (uuid "fp-Q1") (at 20 20)
  (property "Reference" "Q1" (at 0 0 0))
  (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A") (uuid "q1")))
)
"""
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(board)
        pcb = parse_kicad_pcb(path)
        got = {}
        for side in ('F', 'B'):
            it = fp.intent_from_dict(_base(
                min_reader=7,
                blocks=[{'name': 'z', 'refs': ['P1'],
                         'zone': [8.0, 9.5, 12.0, 14.5]}],
                fixed_poses=[{'ref': 'P1', 'x': 10.0, 'y': 10.0, 'rot': 0,
                              'side': side, 'basis': 'declared'}]))
            found, _m = fp.plan_check(it, pcb, path)
            got[side] = [v for v in found
                         if v.rule == 'plan_fixed_outside_zone']
        assert got['F'] == [], [v.message for v in got['F']]
        assert len(got['B']) == 1 and got['B'][0].ref == 'P1', got['B']
    print("  PASS: the fixed pose fits its zone on F and not mirrored on B, "
          "and plan_check warns only for the declared B")


TESTS = [
    test_plan_check_measures_a_fixed_pose_on_its_declared_face,
    test_the_ledger_fails_a_fixed_pose_only_on_its_anchor,
    test_emit_never_fixes_an_array_member_from_mechanical_json,
    test_the_new_keys_load_and_land_on_the_intent,
    test_every_load_refusal_carries_its_reason,
    test_a_reader_6_file_still_loads_and_a_reader_7_claim_refuses_old,
    test_the_formation_predicate,
    test_pin_order_is_read_off_pads_and_nets,
    test_array_formation_grades_the_as_built_board,
    test_board_aware_findings_name_the_member,
    test_the_gate_bundle_carries_the_new_data,
    test_every_fixed_pose_is_graded,
    test_the_brief_compiles_arrays_and_fixed_poses,
    test_the_brief_carries_a_courtyard_waiver_and_drift_sees_it,
    test_the_brief_refuses_what_the_intent_refuses,
    test_merge_drift_and_coverage,
    test_emit_intent_writes_none_of_it_by_default,
    test_mechanical_poses_compile_into_fixed_poses,
    test_an_order_nobody_resolves_is_unchecked_not_passed,
    test_the_single_pad_net_decides_the_pin_order,
    test_a_shared_rotation_no_member_can_take_is_a_conflict,
    test_a_missing_member_skips_the_formation_and_stacked_reads_so,
    test_an_agreeing_brief_pose_corroborates_the_mechanical_one,
    test_the_lost_error_is_for_a_mechanical_entry_only,
    test_a_member_with_no_geometry_gets_a_measured_row,
    test_stacked_members_still_report_the_other_gaps,
    test_two_pad_parts_are_the_same_turned_180,
]


if __name__ == '__main__':
    # A name filter (substring), so a mutation battery can run one test;
    # the battery checks each name it asked for printed its `---` line.
    only = sys.argv[1:]
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
    print('ALL PASS')
