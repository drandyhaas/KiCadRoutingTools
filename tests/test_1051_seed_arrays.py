#!/usr/bin/env python3
"""#1051 / #1053 / #1054 phase 3: the SEEDER seats declared rows, seats each
row's served part first, and seats fixed poses exactly.

What each case pins, and why:

* splitflap U4:47k (the detector's row, `emit_intent(derive_arrays='auto')`):
  seated as ONE row -- one axis, U4's pin order (either direction), one
  rotation, a pitch that clears the board clearance -- and the GRADER, on a
  written copy of the seeded poses, calls it formed too (`array_formation`,
  the rule's own measurement, not the seeder's self-check). Every member
  passes `pose_ok` against its siblings at the written poses.
* glasgow: a resistor-array pair (U30:33R~2, RN7+RN8) and a buffer bank
  (U30:SN74LVC1T45DCKR~2, eight SOT-363) form, graded the same way.
* an UNSEATABLE row (esp_prog R3+R4 at a declared pitch below the courtyard)
  is disclosed in `array_unseated` with its reason, and its members are still
  seated one by one -- not unseated.
* the POSE CAP trips and says so (`array_pose_cap=1`), against a control at
  the default cap in which the same row forms -- so "capped" is not how that
  row always ends.
* `decaps.max_distance_mm = 3`: splitflap and watchy claim > 0 caps at a
  supply pin once their owner ICs are seated first (#1053: declared as fixed
  poses; unzoned and undeclared they claim 0 and say why), and the forced
  0-claim case (the only cap's owner is not U-prefixed) returns the reason
  and prints the note.
* fixed poses (#1054): seated EXACTLY, stamped `(locked yes)`, and unmoved by
  `place_seed --repair` and `--force` (CLI). The repair arm is not vacuous: a
  control intent WITHOUT the fixed pose moves the same part on the same board.
* an illegal fixed pose is REFUSED, never nudged: off the board (pad copper
  past the outline), colliding with an earlier fixed pose (named), and on a
  side the part is not on (no flip move). Refused refs are unseated, unwritten
  and unlocked.
* stage 1 treats a stage-0 part as an obstacle: run 27's fixture, where a
  declared south-edge header's band midpoint lands on FIX1 -- with FIX1 a
  fixed pose, the header slides clear of it (control: without the fixed pose
  the header takes the midpoint).
* an intent declaring none of `decaps.max_distance_mm`, `arrays`,
  `fixed_poses` seeds IDENTICALLY to the pre-phase-3 seeder
  (tests/fixtures/1051/seed_unarmed_baseline.json, recorded at 754e7f419).

    python3 tests/test_1051_seed_arrays.py
"""
import json
import os
import random
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb           # noqa: E402
from placement import arrays as arr                # noqa: E402
from placement import floorplan as fp              # noqa: E402
from placement import seeder                       # noqa: E402
from placement.legality import grade_pad_legality  # noqa: E402
from placement.writer import write_placed_output   # noqa: E402

RUN_ALL_TIMEOUT = 900

BOARDS = os.path.join(ROOT, 'kicad_files')
SPLITFLAP = os.path.join(BOARDS, 'splitflap_driver.kicad_pcb')
GLASGOW = os.path.join(BOARDS, 'glasgow_revC.kicad_pcb')
WATCHY = os.path.join(BOARDS, 'watchy.kicad_pcb')
ESP = os.path.join(BOARDS, 'esp_prog.kicad_pcb')
PLACE_SEED = os.path.join(ROOT, 'py_placer', 'place_seed.py')
BASELINE = os.path.join(TESTS_DIR, 'fixtures', '1051',
                        'seed_unarmed_baseline.json')
SOURCES = ('kicad', 'sheet')
CLEARANCE = 0.2

#: esp_prog's R3 and R4 are the series resistors on U1 pins 3 and 4 (U0RXD /
#: U0TXD), one 0402 footprint -- a two-member row with a served IC.
ESP_ROW = {'name': 'u1_uart', 'members': ['R4', 'R3'], 'serves': 'U1',
           'order': 'pin', 'rotation': 'shared', 'pitch_mm': 'auto',
           'axis': 'auto', 'why': 'test row'}


def _intent(doc, td, name='intent.json'):
    path = os.path.join(td, name)
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=1)
    return fp.load_intent(path), path


def _seed(board, intent, seed='0', clearance=CLEARANCE, **kw):
    pcb = parse_kicad_pcb(board)
    res = seeder.seed_from_intent(pcb, board, intent, random.Random(seed),
                                  group_sources=SOURCES, clearance=clearance,
                                  **kw)
    return pcb, res


def _write(board, res, td, name):
    out = os.path.join(td, name)
    write_placed_output(board, out, res['placements'])
    return out


def _graded_rows(intent, out):
    """{name: array_measured row} from the GRADER on the written board."""
    g = fp.grade(intent, parse_kicad_pcb(out), out, group_sources=SOURCES,
                 clearance=CLEARANCE)
    return {str(a['name']): a for a in g.array_measured}


def _check_row(pcb, res, name, out, graded):
    """The row `name` is formed by the seeder AND the grader, lies in its pin
    order, at one rotation, and every member is legal against its siblings
    at the WRITTEN poses."""
    rec = res['arrays_formed'].get(name)
    assert rec is not None, (name, res['array_unseated'].get(name))
    assert rec['verdict'] == 'formed', rec
    assert graded[name]['formed'] is True, graded[name]
    members = rec['members']
    exp = graded[name].get('order_expected')
    if exp:
        seen = [m for m in members if m in exp]
        assert seen in (exp, list(reversed(exp))), (members, exp)
    written = parse_kicad_pcb(out)
    rots = {round((written.footprints[m].rotation or 0.0) % 360.0, 6)
            for m in members}
    assert len(rots) == 1 and next(iter(rots)) == rec['rot'] % 360.0, rots
    import pose_score
    st = pose_score.make_state(written, out, clearance=CLEARANCE)
    others = set(st.parts) - set(members)
    for m in members:
        p = st.parts[m]
        assert seeder.pose_ok(st, m, p.x, p.y, p.rot, others), (
            f"{m} is not legal against its siblings at its written pose")
    # The pitch clears the board clearance between neighbours.
    b = st.parts[members[0]].rect(0.0, 0.0, rec['rot'])
    ext = (b[2] - b[0]) if rec['axis'] == 'x' else (b[3] - b[1])
    assert rec['pitch_mm'] >= ext + CLEARANCE, (rec['pitch_mm'], ext)
    return rec


def test_splitflap_u4_row_is_formed():
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(SPLITFLAP)
        doc = fp.emit_intent(pcb, SPLITFLAP, derive_arrays='auto')
        names = [a['name'] for a in doc['arrays']]
        assert 'U4:47k' in names, names
        intent, _p = _intent(doc, td)
        pcb, res = _seed(SPLITFLAP, intent)
        out = _write(SPLITFLAP, res, td, 'sf.kicad_pcb')
        graded = _graded_rows(intent, out)
        rec = _check_row(pcb, res, 'U4:47k', out, graded)
        assert sorted(rec['members']) == sorted(
            next(a['members'] for a in doc['arrays']
                 if a['name'] == 'U4:47k'))
        assert not res['unseated'], res['unseated']
        # The seeder's self-check and the grader agree on every row.
        for n, r in res['arrays_formed'].items():
            assert (r['verdict'] == 'formed') == bool(graded[n]['formed']), n
    print(f"  PASS: splitflap U4:47k is one row along {rec['axis']} at "
          f"{rec['rot']:g}deg, pitch {rec['pitch_mm']}mm, in U4's pin order, "
          f"after {rec['poses_tried']} pose(s); the grader agrees on all "
          f"{len(res['arrays_formed'])} seated rows")


def test_glasgow_resistor_pair_and_buffer_bank_are_formed():
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(GLASGOW)
        doc = fp.emit_intent(pcb, GLASGOW, derive_arrays='auto')
        # The buffer bank is a DECLINED bridge since phase-3 fix round 1;
        # a reader may still declare it by hand, which is this row.
        declined = []
        arr.suggest_arrays(pcb, declined=declined)
        bank = next(d for d in declined
                    if d['why'] == 'bridges U30 and RN1')
        doc['arrays'].append({'name': 'U30:SN74LVC1T45DCKR~2',
                              'members': list(bank['members']),
                              'serves': 'U30', 'order': 'unknown',
                              'rotation': 'shared', 'pitch_mm': 'auto',
                              'axis': 'auto', 'why': 'declared by hand'})
        intent, _p = _intent(doc, td)
        pcb, res = _seed(GLASGOW, intent)
        out = _write(GLASGOW, res, td, 'gl.kicad_pcb')
        graded = _graded_rows(intent, out)
        pair = _check_row(pcb, res, 'U30:33R~2', out, graded)
        assert sorted(pair['members']) == ['RN7', 'RN8'], pair['members']
        bank = _check_row(pcb, res, 'U30:SN74LVC1T45DCKR~2', out, graded)
        assert len(bank['members']) == 8, bank['members']
        n_formed = sum(1 for r in res['arrays_formed'].values()
                       if r['verdict'] == 'formed')
    print(f"  PASS: glasgow RN7+RN8 and the 8-buffer bank are formed rows "
          f"(grader agrees); {n_formed} of {len(doc['arrays'])} detected "
          f"rows formed, {len(res['array_unseated'])} not seated as rows")


def _host_pin_along(pcb, host, members, axis):
    """{member: coordinate ALONG `axis` of the host pads it reaches by its
    own nets} on a written board -- the same own-net reading `_seat_array`
    uses, re-derived here from the file."""
    fps = pcb.footprints
    nets = {m: {p.net_id for p in fps[m].pads if p.net_id} for m in members}
    out = {}
    for m in members:
        others = set().union(*(nets[o] for o in members if o != m))
        own = nets[m] - others
        pts = [(p.global_x, p.global_y) for p in fps[host].pads
               if p.net_id and p.net_id in own]
        if pts:
            k = 0 if axis == 'x' else 1
            out[m] = sum(q[k] for q in pts) / len(pts)
    return out


def test_row_runs_the_way_its_host_pins_run():
    """The DIRECTION, asserted explicitly (`_check_row` accepts either, so a
    `_reverse` that never flips, or flips the wrong way, survived it --
    phase-3 verifier). On watchy the detector's pin-order rows include
    both a row whose pin order already runs with the axis and one that must
    be flipped, so both mutations are reachable: for every formed pin-order
    row, member centres increase in the listed order (the row is laid that
    way) AND the first member's host pins lie no further along the axis
    than the last member's."""
    import pose_score
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(WATCHY)
        doc = fp.emit_intent(pcb, WATCHY, derive_arrays='auto')
        intent, _p = _intent(doc, td)
        _pcb, res = _seed(WATCHY, intent)
        out = _write(WATCHY, res, td, 'w.kicad_pcb')
        written = parse_kicad_pcb(out)
        st = pose_score.make_state(written, out, clearance=CLEARANCE)
        spec = {a['name']: a for a in fp.resolved_arrays(intent, pcb)}
        kinds = set()
        checked = 0
        for name, rec in res['arrays_formed'].items():
            sp = spec[name]
            if sp['order'] != 'pin' or not sp['order_refs']:
                continue
            members = rec['members']
            k = 0 if rec['axis'] == 'x' else 1
            cen = [(st.parts[m].rect()[k] + st.parts[m].rect()[k + 2]) / 2
                   for m in members]
            assert cen == sorted(cen), (name, members, cen)
            pins = _host_pin_along(written, sp['serves'], members,
                                   rec['axis'])
            ends = [pins[m] for m in members if m in pins]
            assert len(ends) >= 2, (name, pins)
            assert ends[0] <= ends[-1] + 1e-6, (
                f"{name}: laid against its host pins {ends}")
            ref_order = [m for m in sp['order_refs'] if m in members]
            kinds.add('flipped' if members == ref_order[::-1]
                      and members != ref_order else 'kept')
            checked += 1
        assert kinds == {'flipped', 'kept'}, (kinds, "the board no longer "
                                              "exercises both directions")
    print(f"  PASS: {checked} pin-order row(s) on watchy run with their "
          f"host pins, both a kept and a flipped one among them")


def test_sibling_recheck_reverts_a_row_whose_pads_collide():
    """`_seat_block` checks each member with its unplaced siblings EXCLUDED,
    so only the seated re-check (`_siblings_ok`) sees sibling PADS. A part
    whose pads reach past its courtyard passes the pitch pre-check (which
    is courtyard-based) and every per-member check, and must still be
    refused when seated. Forcing the re-check True (phase-3 verifier:
    survived) would ship two different-net pads on top of each other."""
    board = '''(kicad_pcb
 (version 20241229)
 (net 0 "") (net 1 "/A") (net 2 "/B") (net 3 "/C") (net 4 "/D")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
 (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (uuid "e1"))
%s)
'''
    fp_t = ('''  (footprint "t:R" (layer "F.Cu") (uuid "fp-%(r)s") (at %(x)s 10)
   (property "Reference" "%(r)s" (at 0 0 0))
   (fp_rect (start -0.2 -0.2) (end 0.2 0.2) (layer "F.CrtYd") (uuid "c-%(r)s"))
   (pad "1" smd rect (at -0.9 0) (size 0.8 0.8) (layers "F.Cu") (net %(a)s "/%(na)s") (uuid "%(r)s1"))
   (pad "2" smd rect (at 0.9 0) (size 0.8 0.8) (layers "F.Cu") (net %(b)s "/%(nb)s") (uuid "%(r)s2"))
  )
''')
    # Apart in the INPUT: pad legality is baseline-relative to the input
    # poses, so two parts stacked in the file would license any overlap.
    parts = (fp_t % dict(r='R1', x=25, a=1, na='A', b=2, nb='B')
             + fp_t % dict(r='R2', x=5, a=3, na='C', b=4, nb='D'))
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(board % parts)
        doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm',
               'arrays': [{'name': 'pads_out', 'members': ['R1', 'R2'],
                           'order': 'declared', 'rotation': 0,
                           'pitch_mm': 'auto', 'axis': 'x'}]}
        intent = fp.intent_from_dict(doc, path)
        pcb = parse_kicad_pcb(path)
        res = seeder.seed_from_intent(pcb, path, intent, random.Random('0'),
                                      group_sources=(), clearance=0.2,
                                      array_pose_cap=300)
        assert 'pads_out' in res['array_unseated'], res['arrays_formed']
        assert not res['arrays_formed'], res['arrays_formed']
        out = _write(path, res, td, 'o.kicad_pcb')
        g = grade_pad_legality(parse_kicad_pcb(out), 0.2, pcb_file=out)
        assert g['pad_conflicts'] == 0, g
    print(f"  PASS: a row whose pads collide only when seated is reverted "
          f"at every anchor ({res['array_unseated']['pads_out']['poses_tried']}"
          f" tried) and the members are seated apart, pad-clean")


def test_formed_rows_are_immovable_to_the_eviction_rung():
    """Since 406113056 a formed row's member is `immovable` to stage 3c,
    labelled `array:<name>`, so the rung never lifts one member out of its
    row. Observed through the census the rung records: a part that cannot
    be seated beside a formed row names the members as FROZEN by the row,
    not as liftable blockers."""
    board = '''(kicad_pcb
 (version 20241229)
 (net 0 "") (net 1 "/A") (net 2 "/B") (net 3 "/C")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
 (gr_rect (start 0 0) (end 9 4) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "t:R" (layer "F.Cu") (uuid "fp-R1") (at 4 2)
  (property "Reference" "R1" (at 0 0 0))
  (fp_rect (start -0.8 -0.5) (end 0.8 0.5) (layer "F.CrtYd") (uuid "c1"))
  (pad "1" smd rect (at -0.4 0) (size 0.5 0.6) (layers "F.Cu") (net 1 "/A") (uuid "a1"))
  (pad "2" smd rect (at 0.4 0) (size 0.5 0.6) (layers "F.Cu") (net 3 "/C") (uuid "a2")))
 (footprint "t:R" (layer "F.Cu") (uuid "fp-R2") (at 4 2)
  (property "Reference" "R2" (at 0 0 0))
  (fp_rect (start -0.8 -0.5) (end 0.8 0.5) (layer "F.CrtYd") (uuid "c2"))
  (pad "1" smd rect (at -0.4 0) (size 0.5 0.6) (layers "F.Cu") (net 2 "/B") (uuid "b1"))
  (pad "2" smd rect (at 0.4 0) (size 0.5 0.6) (layers "F.Cu") (net 3 "/C") (uuid "b2")))
 (footprint "t:BIG" (layer "F.Cu") (uuid "fp-X1") (at 4 2)
  (property "Reference" "X1" (at 0 0 0))
  (fp_rect (start -3.6 -1.6) (end 3.6 1.6) (layer "F.CrtYd") (uuid "c3"))
  (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 3 "/C") (uuid "x1")))
)
'''
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(board)
        doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm',
               'arrays': [{'name': 'r', 'members': ['R1', 'R2'],
                           'order': 'declared', 'rotation': 0,
                           'pitch_mm': 'auto', 'axis': 'x'}]}
        intent = fp.intent_from_dict(doc, path)
        res = seeder.seed_from_intent(parse_kicad_pcb(path), path, intent,
                                      random.Random('0'), group_sources=(),
                                      clearance=0.2, board_edge_clearance=0.1,
                                      evict_depth=1)
        assert 'r' in res['arrays_formed'], res['array_unseated']
        assert 'X1' in res['unseated'], res['unseated']
        frozen = (res['no_pose_census'].get('X1') or {}).get('frozen') or {}
        assert frozen.get('R1') == 'array:r' and frozen.get('R2') == 'array:r', \
            res['no_pose_census'].get('X1')
        assert not [e for e in res['evictions'] if e.get('accepted')], \
            res['evictions']
    print(f"  PASS: the rung reports the row's members frozen by the row "
          f"({frozen}) and evicts neither")


def test_anchor_rounds_leave_a_formed_row_whole():
    """Since 406113056 the `--anchors-first` rounds skip a formed row's
    members: the rounds re-seat ONE part at a time toward its partners,
    which pulls a row apart. Graded by the GRADER on the written board.
    The rounds must actually REACH the board -- a reverted round leaves
    nothing to test -- and whether round 2 is kept depends on the seed and
    on every earlier stage, so the arm takes the first watchy seed (of 8)
    whose round 2 is kept, and asserts one exists (seeds 0 and 5 at the
    time of writing; seed 1 before stage 2.4 went opt-in)."""
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(WATCHY)
        doc = fp.emit_intent(pcb, WATCHY, derive_arrays='auto')
        intent, _p = _intent(doc, td)
        res = rounds = None
        for seed in range(8):
            _pcb, res = _seed(WATCHY, intent, seed=str(seed),
                              anchors_first=True, anchor_rounds=3)
            rounds = [n for n in res['notes']
                      if n.startswith('anchor round')]
            if rounds and 'REVERTED' not in rounds[0]:
                break
        assert rounds and 'REVERTED' not in rounds[0], rounds
        moved = int(rounds[0].split(':')[1].split('part')[0])
        assert moved > 0, rounds      # the rounds DID move parts
        out = _write(WATCHY, res, td, 'w.kicad_pcb')
        graded = _graded_rows(intent, out)
        assert res['arrays_formed']
        for name in res['arrays_formed']:
            assert graded[name]['formed'] is True, (name, graded[name])
    print(f"  PASS: {len(res['arrays_formed'])} rows stay formed through "
          f"the anchor rounds ({rounds[0]})")


def test_rows_seat_only_their_hosts_first():
    """Declared rows: stage 2.4 seats each non-zoned row's `serves` and the
    rows, nothing else. Every row comes after its own host."""
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(SPLITFLAP)
        doc = fp.emit_intent(pcb, SPLITFLAP, derive_arrays='auto')
        assert (doc.get('decaps') or {}).get('max_distance_mm') is None
        intent, _p = _intent(doc, td)
        _pcb, res = _seed(SPLITFLAP, intent)
        order = res['early_order']
        rows = {a['name']: a for a in doc['arrays']}
        hosts = {a['serves'] for a in rows.values() if a.get('serves')}
        assert set(order) <= hosts | {f"array:{n}" for n in rows}, order
        for name, a in rows.items():
            key = f"array:{name}"
            assert key in order, (key, order)
            if a.get('serves'):
                assert order.index(a['serves']) < order.index(key), order
        assert res['decap_stage']['armed'] is False
    print(f"  PASS: 2.4 seats only {sorted(hosts)} and the "
          f"{len(rows)} rows, each row after its host")


def test_a_row_member_is_never_seated_alone_before_its_row():
    """A part that is a member of one row and the `serves` of another is
    seated WITH its row, never alone in stage 2.4 ahead of it. glasgow's
    unzoned buffer bank declared by hand (a declined bridge, serving U30),
    and a second row, RN11+RN12, declared as serving one of the bank's
    buffers: that buffer is a 2.4 host, yet no bank member appears alone in
    2.4's seat order, and the bank forms."""
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(GLASGOW)
        doc = fp.emit_intent(pcb, GLASGOW)
        declined = []
        arr.suggest_arrays(pcb, declined=declined)
        # The UNZONED bank (RN7's side): the RN1-side bank lies in a zoned
        # sheet block and is seated in stage 2, never by 2.4.
        bank = next(d for d in declined if d['why'] == 'bridges U30 and RN7')
        host = bank['members'][0]
        doc['arrays'] = [
            {'name': 'bank', 'members': list(bank['members']),
             'serves': 'U30', 'order': 'unknown', 'rotation': 'shared',
             'pitch_mm': 'auto', 'axis': 'auto', 'why': 'declared by hand'},
            {'name': 'rn', 'members': ['RN11', 'RN12'], 'serves': host,
             'order': 'unknown', 'rotation': 'shared', 'pitch_mm': 'auto',
             'axis': 'auto', 'why': 'serves a bank member'}]
        intent, _p = _intent(doc, td)
        _pcb, res = _seed(GLASGOW, intent)
        order = res['early_order']
        assert 'array:bank' in order and 'array:rn' in order, order
        assert not set(bank['members']) & set(order), (host, order)
        assert 'bank' in res['arrays_formed'], res['array_unseated']
    print(f"  PASS: {host} serves row rn and is a bank member; no bank "
          f"member is seated before the bank row")


#: A row whose only free strip is > 30mm from its target: the locked B1
#: covers x 0..45 of a 100 x 10mm board, and the row's partner L1 sits
#: inside it at x=5. The 1mm ring (radius 30) finds nothing; the sweep does.
SWEEP_BOARD = """(kicad_pcb
 (version 20241229)
 (net 0 "") (net 1 "/A") (net 2 "/B")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
 (gr_rect (start 0 0) (end 100 10) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "t:L" (layer "F.Cu") (uuid "fp-L1") (at 5 5) (locked yes)
  (property "Reference" "L1" (at 0 0 0))
  (fp_rect (start -1 -1) (end 1 1) (layer "F.CrtYd") (uuid "cL"))
  (pad "1" smd rect (at -0.5 0) (size 0.4 0.4) (layers "F.Cu") (net 1 "/A") (uuid "l1"))
  (pad "2" smd rect (at 0.5 0) (size 0.4 0.4) (layers "F.Cu") (net 2 "/B") (uuid "l2")))
 (footprint "t:B" (layer "F.Cu") (uuid "fp-B1") (at 22.5 5) (locked yes)
  (property "Reference" "B1" (at 0 0 0))
  (fp_rect (start -22.5 -5) (end 22.5 5) (layer "F.CrtYd") (uuid "cB"))
  (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net 0 "") (uuid "b1")))
 (footprint "t:R" (layer "F.Cu") (uuid "fp-R1") (at 80 5)
  (property "Reference" "R1" (at 0 0 0))
  (fp_rect (start -0.8 -0.5) (end 0.8 0.5) (layer "F.CrtYd") (uuid "c1"))
  (pad "1" smd rect (at -0.4 0) (size 0.4 0.5) (layers "F.Cu") (net 1 "/A") (uuid "r11")))
 (footprint "t:R" (layer "F.Cu") (uuid "fp-R2") (at 90 5)
  (property "Reference" "R2" (at 0 0 0))
  (fp_rect (start -0.8 -0.5) (end 0.8 0.5) (layer "F.CrtYd") (uuid "c2"))
  (pad "1" smd rect (at -0.4 0) (size 0.4 0.5) (layers "F.Cu") (net 2 "/B") (uuid "r21")))
)
"""


def test_row_seat_reaches_the_sweep_before_the_fine_rings():
    """The band order's sweep-before-fine half (02ba1cb1a). With the fine
    rings first, one angle's fine ring (16641 positions) plus the 1mm ring
    (3721) is 20362 -- past the 20000 cap -- so a row whose only room is
    beyond the rings' reach is refused as capped. Sweep-first seats it
    beyond x=45 in a few thousand poses. (phase-3 re-verifier: the
    ring,fine,xfine,sweep mutation survived every other test.)"""
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(SWEEP_BOARD)
        doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm',
               'arrays': [{'name': 'far', 'members': ['R1', 'R2'],
                           'order': 'declared', 'rotation': 0,
                           'pitch_mm': 'auto', 'axis': 'x'}]}
        res = seeder.seed_from_intent(
            parse_kicad_pcb(path), path, fp.intent_from_dict(doc, path),
            random.Random('0'), group_sources=(), clearance=0.2,
            board_edge_clearance=0.1)
        rec = res['arrays_formed'].get('far')
        assert rec is not None, res['array_unseated']
        assert rec['anchor'][0] > 45.0, rec
        assert rec['poses_tried'] < seeder.ARRAY_SEAT_POSE_CAP, rec
    print(f"  PASS: the row seats at x={rec['anchor'][0]} after "
          f"{rec['poses_tried']} poses, via the sweep")


def test_row_target_is_the_partner_centroid_not_the_host_pins():
    """19917b2d7: a row aims at the mean of its members' placed partners
    (host AND far side). On splitflap at least one row's partner centroid
    differs from its host-pin centroid by > 2mm (U5:220R since stage 2.4
    went opt-in; U4:47k too before), and every such row is seated nearer
    its target than its host pins; with the host-pins target the two
    coincide and no such row exists."""
    import math
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(SPLITFLAP)
        doc = fp.emit_intent(pcb, SPLITFLAP, derive_arrays='auto')
        intent, _p = _intent(doc, td)
        _pcb, res = _seed(SPLITFLAP, intent)
        w = parse_kicad_pcb(_write(SPLITFLAP, res, td, 'sf.kicad_pcb'))
        differ = []
        for name, rec in res['arrays_formed'].items():
            host = rec['serves']
            if not host:
                continue
            mem = rec['members']
            nets = {m: {p.net_id for p in w.footprints[m].pads if p.net_id}
                    for m in mem}
            pts = []
            for m in mem:
                own = nets[m] - set().union(*(nets[o] for o in mem if o != m))
                q = [(p.global_x, p.global_y) for p in w.footprints[host].pads
                     if p.net_id in own]
                if q:
                    pts.append((sum(a for a, _ in q) / len(q),
                                sum(b for _, b in q) / len(q)))
            hp = (sum(a for a, _ in pts) / len(pts),
                  sum(b for _, b in pts) / len(pts))
            if math.dist(rec['target'], hp) > 2.0:
                differ.append(name)
                assert (math.dist(rec['anchor'], rec['target'])
                        < math.dist(rec['anchor'], hp)), (name, rec, hp)
        assert differ, differ
    print(f"  PASS: {', '.join(sorted(differ))} aim at their partner "
          f"centroid, away from their host pins, and sit nearer it")


def test_padless_fixed_pose_is_judged_by_its_hole():
    """Stage 0's overhang exemption rested on "every pad's copper is on the
    board", vacuous for a part with no copper: tigard's NPTH-only H1 at
    (300, 300) read "seated exactly ... (pads on the board)". The hole now
    decides: its whole drill circle on the board. The control is H1 at its
    own human pose (33, 33), whose COURTYARD crosses the margin gate and
    which must still seat."""
    tig = os.path.join(BOARDS, 'tigard.kicad_pcb')
    got = {}
    with tempfile.TemporaryDirectory() as td:
        doc = fp.emit_intent(parse_kicad_pcb(tig), tig)
        for pose in ((300.0, 300.0), (31.0, 33.0), (33.0, 33.0)):
            d = dict(doc, fixed_poses=[{'ref': 'H1', 'x': pose[0],
                                        'y': pose[1], 'rot': 0,
                                        'basis': 'declared', 'why': 't'}])
            intent, _p = _intent(d, td)
            _pcb, res = _seed(tig, intent)
            got[pose] = res
        for pose in ((300.0, 300.0), (31.0, 33.0)):
            r = got[pose]
            assert 'H1' in r['fixed_refused'], (pose, r['fixed_seated'])
            assert 'drill hole(s) past the outline' in                 r['fixed_refused']['H1']['reason'], r['fixed_refused']
            assert 'H1' in r['unseated']
        ok = got[(33.0, 33.0)]
        assert ok['fixed_seated']['H1']['how'] in ('contained', 'overhang'),             ok['fixed_refused']
    print("  PASS: H1 off the board and straddling the edge are refused by "
          "the hole; at its human pose it seats")


def test_unseatable_row_is_disclosed_and_falls_through():
    with tempfile.TemporaryDirectory() as td:
        doc = fp.emit_intent(parse_kicad_pcb(ESP), ESP)
        doc['arrays'] = [dict(ESP_ROW, pitch_mm=0.3)]
        intent, _p = _intent(doc, td)
        pcb, res = _seed(ESP, intent)
        un = res['array_unseated'].get('u1_uart')
        assert un is not None and not res['arrays_formed'], res
        assert 'pitch 0.3mm is below the courtyard extent' in un['reason'], un
        assert un['capped'] is False and un['poses_tried'] == 0, un
        written = {p['reference'] for p in res['placements']}
        assert {'R3', 'R4'} <= written, written
        assert not {'R3', 'R4'} & set(res['unseated']), res['unseated']
        assert any('array u1_uart: NOT seated as a row' in n
                   for n in res['notes']), res['notes']
    print(f"  PASS: a row at a pitch below its courtyard is disclosed "
          f"({un['reason'][:60]}...) and R3/R4 are seated one by one")


def test_pose_cap_trips_and_says_so():
    with tempfile.TemporaryDirectory() as td:
        doc = fp.emit_intent(parse_kicad_pcb(ESP), ESP)
        doc['arrays'] = [dict(ESP_ROW)]
        intent, _p = _intent(doc, td)
        # CONTROL first: at the default cap the same row forms, so "capped"
        # below is the cap's doing and not the row's fate.
        _pcb, ok = _seed(ESP, intent)
        rec = ok['arrays_formed'].get('u1_uart')
        assert rec is not None and rec['verdict'] == 'formed', ok
        assert rec['poses_tried'] > 1, rec
        _pcb, res = _seed(ESP, intent, array_pose_cap=1)
        un = res['array_unseated'].get('u1_uart')
        assert un is not None, res['arrays_formed']
        assert un['capped'] is True and un['poses_tried'] == 1, un
        assert 'the pose cap (1) was reached' in un['reason'], un
        assert any('pose cap (1)' in n for n in res['notes'])
        assert not {'R3', 'R4'} & set(res['unseated'])
    print(f"  PASS: cap 1 trips after 1 pose and says so; the default cap "
          f"forms the same row after {rec['poses_tried']} pose(s)")


def _owner_fixed_poses(pcb):
    """A `fixed_poses[]` entry at its own pose for every U-prefixed IC a cap
    elects a tether to -- the owners the decap pin stage reads its pins off
    (U-prefixed without `decap_owner_chips`), seated by stage 0."""
    from placement import groups as groups_mod
    near, beyond, _o = groups_mod.decap_populations(pcb)
    fps = pcb.footprints
    owners = sorted(r for r in set(near) | {ic for _c, ic, _d in beyond}
                    if r.startswith('U'))
    return owners, [{'ref': r, 'x': fps[r].x, 'y': fps[r].y,
                     'rot': fps[r].rotation or 0, 'basis': 'declared',
                     'why': 'the owner IC, seated first'} for r in owners]


def test_decaps_armed_claims_caps_once_the_owners_are_seated():
    """#1053: the pin stage claims caps at the supply pins of ICs already
    seated. With the owner ICs declared as fixed poses (stage 0), splitflap
    and watchy claim caps; unzoned and undeclared they claim 0 (the next
    test)."""
    got = {}
    with tempfile.TemporaryDirectory() as td:
        for board in (SPLITFLAP, WATCHY):
            pcb = parse_kicad_pcb(board)
            doc = fp.emit_intent(pcb, board)
            doc['decaps'] = dict(doc.get('decaps') or {}, max_distance_mm=3.0)
            owners, doc['fixed_poses'] = _owner_fixed_poses(pcb)
            assert owners, board
            intent, _p = _intent(doc, td, os.path.basename(board) + '.json')
            _pcb, res = _seed(board, intent)
            assert not res['fixed_refused'], res['fixed_refused']
            ds = res['decap_stage']
            assert ds['armed'] and ds['scope'] > 0, ds
            assert ds['claimed'] > 0, ds
            got[os.path.basename(board)] = (ds['claimed'], ds['scope'])
    print(f"  PASS: with max_distance_mm 3 the pin stage claims caps "
          f"(claimed, scope): {got}")


def test_the_decap_stage_says_why_it_claims_nothing():
    """#1053's "or say loudly why it did not": on an unzoned seed with no
    owner IC seated before the pin stage, splitflap and watchy claim 0 --
    and the NOTE and `decap_stage.reason` say that no owner IC was seated,
    and what would seat one."""
    with tempfile.TemporaryDirectory() as td:
        for board in (SPLITFLAP, WATCHY):
            doc = fp.emit_intent(parse_kicad_pcb(board), board)
            doc['decaps'] = dict(doc.get('decaps') or {}, max_distance_mm=3.0)
            intent, _p = _intent(doc, td, os.path.basename(board) + '.json')
            _pcb, res = _seed(board, intent)
            ds = res['decap_stage']
            assert ds['armed'] and ds['scope'] > 0, ds
            assert ds['claimed'] == 0, ds
            why = 'no owner IC is seated before this stage'
            assert why in (ds['reason'] or ''), ds
            note = [n for n in res['notes']
                    if n.startswith('decap stage 2.5:')]
            assert note and why in note[0], note
    print("  PASS: the stage claims 0 and says no owner IC was seated "
          "before it")


def test_zero_claim_reports_why():
    import test_792_decap_seeding as t792
    with tempfile.TemporaryDirectory() as wd:
        # Only C3 in scope; its owner IC1 is not U-prefixed, so neither 2.4
        # nor 2.5 treats it as an owner (decap_owner_chips is off).
        res, _poses, _pcb = t792._seed(
            wd, {'max_distance_mm': 3.0, 'exempt': ['C1', 'C2', 'C5']})
        ds = res['decap_stage']
        assert ds['armed'] and ds['scope'] == 1 and ds['claimed'] == 0, ds
        assert 'pins 0' in (ds['reason'] or ''), ds
        note = [n for n in res['notes'] if n.startswith('decap stage 2.5:')]
        assert note and '1 cap(s) in scope, 0 claimed' in note[0], res['notes']
    print(f"  PASS: a non-empty scope claiming 0 says why: {ds['reason']}")


def _summary(r):
    m = re.search(r'^JSON_SUMMARY: (.*)$', r.stdout, re.M)
    assert m, r.stdout[-1500:]
    return json.loads(m.group(1))


def _run_seed(args):
    r = subprocess.run([sys.executable, '-X', 'utf8', PLACE_SEED] + args,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=600)
    out = r.stdout + r.stderr
    assert 'Traceback' not in out, out[-2000:]
    # 0, or 4 for a grade finding on this board -- never an argparse or
    # load failure. The fixed-pose contract is read off the summary and the
    # written file, not off the exit code.
    assert r.returncode in (0, 4), (r.returncode, out[-2000:])
    return r


def _fp_pose(path, ref):
    f = parse_kicad_pcb(path).footprints[ref]
    return (round(f.x, 3), round(f.y, 3), round((f.rotation or 0.0) % 360, 3),
            bool(f.locked))


def test_fixed_pose_exact_locked_and_survives_repair_and_force():
    from placement.portfolio import copy_siblings
    with tempfile.TemporaryDirectory() as td:
        doc = fp.emit_intent(parse_kicad_pcb(ESP), ESP)
        # R1 at its own (human) pose: small, so the repair control below has
        # room to move it -- a large part on this dense board has none.
        fixed = {'ref': 'R1', 'x': 136.4, 'y': 98.8, 'rot': 270, 'side': 'F',
                 'basis': 'declared', 'why': 'test datum'}
        doc_fixed = dict(doc, fixed_poses=[fixed])
        _i, ip = _intent(doc_fixed, td, 'fixed.json')
        _i, ip_ctl = _intent(doc, td, 'control.json')
        seeded = os.path.join(td, 'seeded.kicad_pcb')
        s = _summary(_run_seed([ESP, seeded, '--intent', ip, '--force',
                                '--clearance', str(CLEARANCE)]))
        assert s['fixed_seated']['R1']['how'] == 'contained', s['fixed_seated']
        assert s['fixed_seated']['R1']['at_written_pose'] is True
        assert _fp_pose(seeded, 'R1') == (136.4, 98.8, 270.0, True), \
            _fp_pose(seeded, 'R1')

        # --force on the seeded board: R1 is file-locked AT its pose.
        forced = os.path.join(td, 'forced.kicad_pcb')
        s = _summary(_run_seed([seeded, forced, '--intent', ip, '--force',
                                '--clearance', str(CLEARANCE)]))
        assert s['fixed_seated']['R1']['how'] == 'already_there', s
        assert _fp_pose(forced, 'R1') == (136.4, 98.8, 270.0, True)

        # --repair, with a real violation ON R1: C3 dropped onto it, the
        # stamp on R1 REMOVED (so the file lock is not what protects it) and
        # C3 locked (so the pair's only mover is R1). The control intent,
        # without the fixed pose, must move R1 -- or this arm is vacuous.
        bad = os.path.join(td, 'bad.kicad_pcb')
        write_placed_output(seeded, bad, [{'reference': 'C3', 'new_x': 136.4,
                                           'new_y': 98.8,
                                           'new_rotation': 270.0}])
        copy_siblings(seeded, bad)
        assert seeder.stamp_unlocked(bad, ['R1']) == 1
        seeder.stamp_locked(bad, ['C3'])
        assert _fp_pose(bad, 'R1')[3] is False
        ctl = os.path.join(td, 'repair_ctl.kicad_pcb')
        _run_seed([bad, ctl, '--intent', ip_ctl, '--repair',
                   '--clearance', str(CLEARANCE)])
        assert _fp_pose(ctl, 'R1')[:3] != (136.4, 98.8, 270.0), (
            "control: the repair did not move R1 without the fixed pose, so "
            "the fixed-pose arm below would prove nothing")
        rep = os.path.join(td, 'repair.kicad_pcb')
        _run_seed([bad, rep, '--intent', ip, '--repair',
                   '--clearance', str(CLEARANCE)])
        assert _fp_pose(rep, 'R1')[:3] == (136.4, 98.8, 270.0), \
            _fp_pose(rep, 'R1')
        # And the stamped board is untouched by --repair too.
        rep2 = os.path.join(td, 'repair2.kicad_pcb')
        _run_seed([seeded, rep2, '--intent', ip, '--repair',
                   '--clearance', str(CLEARANCE)])
        assert _fp_pose(rep2, 'R1') == (136.4, 98.8, 270.0, True)
    print("  PASS: R1 seated exactly and stamped; --force keeps it "
          "(already_there); --repair leaves it, stamped or not, where the "
          "control intent moves it")


def test_every_unhonoured_fixed_pose_fails_the_gate():
    """Phase-5 fact-check: a fixed pose refused because the part is locked
    in the FILE off its pose was in `fixed_refused` but not in `unseated`,
    and its grade error sits on a locked part (`grade_errors_pinned`), so
    place_seed exited 0 with the declared pose unmet. Every `fixed_refused`
    entry now fails the gate. CONTROL first: the same seeded board with the
    pose it is locked AT exits 0, so the refusal is what moves the exit."""
    import place_seed as _ps
    assert _ps.fixed_pose_reason({'fixed_refused': {}, 'fixed_seated': {
        'R1': {'at_written_pose': True}}}) is None
    assert 'R1' in _ps.fixed_pose_reason({'fixed_refused': {'R1': {}}})
    assert 'C1' in _ps.fixed_pose_reason({'fixed_seated': {
        'C1': {'at_written_pose': False}}})
    with tempfile.TemporaryDirectory() as td:
        doc = fp.emit_intent(parse_kicad_pcb(ESP), ESP)
        at = {'ref': 'R1', 'x': 136.4, 'y': 98.8, 'rot': 270,
              'basis': 'declared', 'why': 'test datum'}
        _i, ip = _intent(dict(doc, fixed_poses=[at]), td, 'at.json')
        seeded = os.path.join(td, 'seeded.kicad_pcb')
        _run_seed([ESP, seeded, '--intent', ip, '--force', '--no-polish',
                   '--clearance', str(CLEARANCE)])
        assert _fp_pose(seeded, 'R1') == (136.4, 98.8, 270.0, True)
        ctl = _run_seed([seeded, os.path.join(td, 'ctl.kicad_pcb'),
                         '--intent', ip, '--force', '--no-polish',
                         '--clearance', str(CLEARANCE)])
        s = _summary(ctl)
        assert s['fixed_seated']['R1']['how'] == 'already_there', s
        assert ctl.returncode == 0, (
            "control: the at-pose run must exit 0, or the arm below cannot "
            "attribute its exit 4 to the refusal -- "
            + (ctl.stdout + ctl.stderr)[-1500:])
        off = dict(at, x=137.4)
        _i, ip_off = _intent(dict(doc, fixed_poses=[off]), td, 'off.json')
        r = _run_seed([seeded, os.path.join(td, 'off.kicad_pcb'),
                       '--intent', ip_off, '--force', '--no-polish',
                       '--clearance', str(CLEARANCE)])
        s = _summary(r)
        assert 'R1' in s['fixed_refused'], s['fixed_refused']
        assert 'R1' not in s['unseated_refs'], s['unseated_refs']
        assert r.returncode == 4, (r.returncode, r.stderr[-1500:])
        assert 'declared fixed pose(s) NOT honoured (R1)' in r.stderr, \
            r.stderr[-1500:]
    print("  PASS: a fixed pose refused on a file-locked part exits 4 and "
          "says so (the at-pose control exits 0)")


def test_illegal_fixed_pose_is_refused_not_nudged():
    """Off the board, on the wrong side, and two DECLARATIONS on one spot.
    The pair clash refuses BOTH C1 and C3, each naming the other and the
    measured overlap -- neither declaration outranks the other, and keeping
    the one whose ref sorts first would decide by name. Q1, at its human
    pose, seats exactly and is locked."""
    with tempfile.TemporaryDirectory() as td:
        doc = fp.emit_intent(parse_kicad_pcb(ESP), ESP)
        doc['fixed_poses'] = [
            {'ref': 'R1', 'x': 100.0, 'y': 100.0, 'rot': 0,
             'basis': 'declared', 'why': 'off the board'},
            {'ref': 'C1', 'x': 138.46, 'y': 96.23, 'rot': 90,
             'basis': 'declared', 'why': 'clashes with C3'},
            {'ref': 'C3', 'x': 138.46, 'y': 96.23, 'rot': 90,
             'basis': 'declared', 'why': 'on top of C1'},
            {'ref': 'R2', 'x': 136.4, 'y': 101.6, 'rot': 90, 'side': 'B',
             'basis': 'declared', 'why': 'wrong side'},
            {'ref': 'Q1', 'x': 139.0, 'y': 99.7, 'rot': 180,
             'basis': 'declared', 'why': 'its human pose'},
        ]
        intent, _p = _intent(doc, td)
        pcb, res = _seed(ESP, intent)
        ref_ = res['fixed_refused']
        assert set(ref_) == {'R1', 'C1', 'C3', 'R2'}, ref_
        assert 'past the outline' in ref_['R1']['reason'], ref_['R1']
        for a, b in (('C1', 'C3'), ('C3', 'C1')):
            assert f"courtyard overlaps {b} by" in ref_[a]['reason'], ref_[a]
            assert ref_[a]['conflicts_with_declared'] == [b], ref_[a]
        assert 'no flip move' in ref_['R2']['reason'], ref_['R2']
        assert res['fixed_seated']['Q1']['how'] == 'contained'
        written = {p['reference']: p for p in res['placements']}
        assert (written['Q1']['new_x'], written['Q1']['new_y'],
                written['Q1']['new_rotation']) == (139.0, 99.7, 180.0)
        decl = {f['ref']: f for f in doc['fixed_poses']}
        disp = res.get('unseated_disposition') or {}
        for r in ('R1', 'C1', 'C3', 'R2'):
            assert r in res['unseated'], (r, res['unseated'])
            # #1151: a refused part may be STAGED off the board, which
            # writes it at its staging slot -- never at the refused pose.
            if r in written:
                assert disp[r]['disposition'] == 'staged', (r, disp.get(r))
                assert (written[r]['new_x'], written[r]['new_y']) != (
                    decl[r]['x'], decl[r]['y']), r
            assert r not in res['lock_refs'], r
        assert 'Q1' in res['lock_refs']
        assert sum('REFUSED' in n for n in res['notes']) == 4, res['notes']
    print("  PASS: off-board, wrong-side, and BOTH halves of a declared "
          "clash are refused (each naming the other); Q1 is exact and "
          "locked")


def _glasgow_human_fixed(td, extra=()):
    """glasgow's RN banks and SN74LVC1T45 buffers (plus `extra`) declared as
    fixed poses AT THEIR HUMAN POSES, seeded on the human board itself --
    every unlocked part is the pile there, so this is the unplaced board's
    question without depending on wk/."""
    pcb = parse_kicad_pcb(GLASGOW)
    fps = pcb.footprints
    rn = sorted(r for r in fps if r.startswith('RN'))
    buf = sorted(r for r, f in fps.items()
                 if 'SN74LVC1T45' in str(getattr(f, 'value', '')))
    assert len(rn) == 12 and len(buf) == 17, (rn, buf)
    doc = fp.emit_intent(pcb, GLASGOW)
    doc['fixed_poses'] = [{'ref': r, 'x': fps[r].x, 'y': fps[r].y,
                           'rot': fps[r].rotation or 0, 'basis': 'declared',
                           'why': 'the human pose'}
                          for r in rn + buf + list(extra)]
    intent, _p = _intent(doc, td)
    _pcb, res = _seed(GLASGOW, intent)
    return rn + buf, res


def test_human_glasgow_rows_seat_as_fixed_poses():
    """Stage 0 judges courtyards the way KiCad does: an OVERLAP is illegal,
    ABUTTING is not. glasgow's human RN banks and buffers abut at exactly
    0.000mm (kicad-cli's DRC accepts them) and ALL 29 seat. Before this,
    under the searched seat's 0.02mm floor and checked against only the
    poses already seated in ref order, 15 of the 29 were refused, in an
    alternating pattern, each "within 0.02mm"."""
    with tempfile.TemporaryDirectory() as td:
        refs, res = _glasgow_human_fixed(td)
        assert not res['fixed_refused'], res['fixed_refused']
        assert set(refs) <= set(res['fixed_seated']), sorted(
            set(refs) - set(res['fixed_seated']))
        written = {p['reference']: p for p in res['placements']}
        fps = parse_kicad_pcb(GLASGOW).footprints
        for r in refs:
            assert (written[r]['new_x'], written[r]['new_y']) == (
                round(fps[r].x, 3), round(fps[r].y, 3)), r
    print(f"  PASS: all {len(refs)} human RN/buffer poses seat exactly")


def test_a_real_overlap_is_refused_with_its_measurement():
    """U30 at its human pose overlaps the file-locked FID8 by 1.15 x 1.15mm
    (kicad-cli flags that pair). The refusal states the measured overlap,
    not a clearance floor it was never judged at."""
    with tempfile.TemporaryDirectory() as td:
        _refs, res = _glasgow_human_fixed(td, extra=('U30',))
        why = res['fixed_refused']['U30']['reason']
        assert 'courtyard overlaps FID8 by 1.15x1.15mm' in why, why
        assert 'within' not in why, why
        assert set(res['fixed_refused']) == {'U30'}, res['fixed_refused']
    print(f"  PASS: U30 refused -- {why}")


def _pair_board(td, xb, waive=None, why='the design', xa=10.0):
    """Two 2 x 2mm courtyards on a 30 x 20 board, A at x=`xa`, B at x=`xb`,
    pads 0.6mm from centre (so abutting courtyards keep pad clearance).
    `waive`, when given, is A's `accept_courtyard_overlap` (#1060)."""
    part = """ (footprint "t:P" (layer "F.Cu") (uuid "fp-%(r)s") (at %(x)s 10)
  (property "Reference" "%(r)s" (at 0 0 0))
  (fp_rect (start -1 -1) (end 1 1) (layer "F.CrtYd") (uuid "c-%(r)s"))
  (pad "1" smd rect (at -0.6 0) (size 0.4 0.4) (layers "F.Cu") (net %(a)s "/N%(a)s") (uuid "%(r)s1"))
  (pad "2" smd rect (at 0.6 0) (size 0.4 0.4) (layers "F.Cu") (net %(b)s "/N%(b)s") (uuid "%(r)s2")))
"""
    body = ('(kicad_pcb\n (version 20241229)\n (net 0 "") (net 1 "/N1") '
            '(net 2 "/N2") (net 3 "/N3") (net 4 "/N4")\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
            ' (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") '
            '(uuid "e1"))\n'
            + part % dict(r='A', x=5, a=1, b=2)
            + part % dict(r='B', x=25, a=3, b=4) + ')\n')
    path = os.path.join(td, f'pair_{xb}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(body)
    a = {'ref': 'A', 'x': xa, 'y': 10.0, 'rot': 0, 'basis': 'declared'}
    if waive is not None:
        a.update(accept_courtyard_overlap=list(waive), why=why)
    doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm', 'fixed_poses': [
        a, {'ref': 'B', 'x': xb, 'y': 10.0, 'rot': 0, 'basis': 'declared'}]}
    return seeder.seed_from_intent(
        parse_kicad_pcb(path), path, fp.intent_from_dict(doc, path),
        random.Random('0'), group_sources=(), clearance=0.2,
        board_edge_clearance=0.1)


def test_abutting_fixed_poses_seat_and_overlapping_ones_both_refuse():
    """Synthetic, so the answer is arithmetic: courtyards touching at x=11
    (gap 0) are legal, as KiCad has them, and both seat; B 0.1mm further in
    overlaps A by 0.10 x 2.00mm and BOTH declarations are refused, each
    naming the other -- the verdict cannot depend on which ref sorts first."""
    with tempfile.TemporaryDirectory() as td:
        ok = _pair_board(td, 12.0)
        assert set(ok['fixed_seated']) == {'A', 'B'}, ok['fixed_refused']
        bad = _pair_board(td, 11.9)
        assert set(bad['fixed_refused']) == {'A', 'B'}, bad['fixed_seated']
        for a, b in (('A', 'B'), ('B', 'A')):
            rec = bad['fixed_refused'][a]
            assert f"courtyard overlaps {b} by 0.10x2.00mm" in rec['reason'], \
                rec
            assert rec['conflicts_with_declared'] == [b], rec
    print("  PASS: abutting courtyards seat; a 0.1mm overlap refuses both, "
          "each naming the other")


def test_a_named_courtyard_waiver_seats_u30_exactly():
    """#1060: U30 at its human pose overlaps FID8's courtyard 1.15 x 1.15mm,
    and `accept_courtyard_overlap: ["FID8"]` (with its `why`) seats it
    exactly, locked, with the measured overlap disclosed in its record. The
    unwaived pose is still refused (the arm above)."""
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(GLASGOW)
        u = pcb.footprints['U30']
        doc = fp.emit_intent(pcb, GLASGOW)
        doc['fixed_poses'] = [{'ref': 'U30', 'x': u.x, 'y': u.y,
                               'rot': u.rotation or 0, 'basis': 'declared',
                               'why': 'the human pose, FID8 sits in its '
                                      'courtyard by design',
                               'accept_courtyard_overlap': ['FID8']}]
        intent, _p = _intent(doc, td)
        _pcb, res = _seed(GLASGOW, intent)
        assert not res['fixed_refused'], res['fixed_refused']
        rec = res['fixed_seated']['U30']
        assert rec['how'] == 'contained', rec
        cw = rec['courtyard_waived']['FID8']
        assert (cw['w_mm'], cw['h_mm']) == (1.15, 1.15), cw
        assert 'U30' in res['lock_refs'], res['lock_refs']
        written = {p['reference']: p for p in res['placements']}
        assert (written['U30']['new_x'], written['U30']['new_y']) == (
            round(u.x, 3), round(u.y, 3))
        out = _write(GLASGOW, res, td, 'u30.kicad_pcb')
        g = fp.grade(intent, parse_kicad_pcb(out), out, group_sources=SOURCES,
                     clearance=CLEARANCE)
        w = [v for v in g.violations if v.rule == 'fixed_pose_overlap_waived']
        assert [(v.ref, v.severity, v.measured['waives']) for v in w] == [
            ('U30', 'warn', 'FID8')], w
        assert w[0].measured['overlap_mm2'] > 1.3, w[0].measured
    print(f"  PASS: U30 seated exactly; waived FID8 {cw}; graded "
          f"{w[0].message}")


def test_the_waiver_covers_courtyards_only_both_ways():
    """Synthetic, so the answer is arithmetic. A waiver on A naming B:
    * B at 11.9 overlaps A's courtyard only -> BOTH seat (the pair is
      unordered, so B's own check against A is waived too), disclosed;
    * B at 10.0 stacks its pads on A's -> BOTH still refused, naming the
      pad short (the waiver is a claim about courtyards, not copper);
    * A at x=0.3 puts its pad copper past the outline -> refused.
    """
    with tempfile.TemporaryDirectory() as td:
        ok = _pair_board(td, 11.9, waive=['B'])
        assert set(ok['fixed_seated']) == {'A', 'B'}, ok['fixed_refused']
        assert 'B' in ok['fixed_seated']['A']['courtyard_waived'], ok
        assert 'A' in ok['fixed_seated']['B']['courtyard_waived'], ok
        bad = _pair_board(td, 10.0, waive=['B'])
        assert set(bad['fixed_refused']) == {'A', 'B'}, bad['fixed_seated']
        why = bad['fixed_refused']['A']['reason']
        assert 'pads' in why and 'courtyard' not in why, why
        off = _pair_board(td, 25.0, waive=['B'], xa=0.3)
        assert 'A' in off['fixed_refused'], off['fixed_seated']
        assert 'past the outline' in off['fixed_refused']['A']['reason'], off
    print(f"  PASS: courtyard-only waiver seats both; a pad stack still "
          f"refuses ({why}); the outline still refuses")


def _npth_pair(td, xb, waive=True):
    """Two 2 x 2mm courtyards, each with ONE non-plated hole (drill 0.5) at
    its origin, A at x=10 and B at x=`xb`; A waives B's courtyard."""
    part = """ (footprint "t:H" (layer "F.Cu") (uuid "fp-%(r)s") (at %(x)s 10)
  (property "Reference" "%(r)s" (at 0 0 0))
  (fp_rect (start -1 -1) (end 1 1) (layer "F.CrtYd") (uuid "c-%(r)s"))
  (pad "" np_thru_hole circle (at 0 0) (size 0.5 0.5) (drill 0.5) (layers "*.Cu" "*.Mask") (uuid "%(r)sh")))
"""
    body = ('(kicad_pcb\n (version 20241229)\n (net 0 "")\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))\n'
            ' (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") '
            '(uuid "e1"))\n'
            + part % dict(r='A', x=5) + part % dict(r='B', x=25) + ')\n')
    path = os.path.join(td, f'npth_{xb}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(body)
    a = {'ref': 'A', 'x': 10.0, 'y': 10.0, 'rot': 0, 'basis': 'declared'}
    if waive:
        a.update(accept_courtyard_overlap=['B'], why='the design')
    doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm', 'fixed_poses': [
        a, {'ref': 'B', 'x': xb, 'y': 10.0, 'rot': 0, 'basis': 'declared'}]}
    return seeder.seed_from_intent(
        parse_kicad_pcb(path), path, fp.intent_from_dict(doc, path),
        random.Random('0'), group_sources=(), clearance=0.2,
        board_edge_clearance=0.1)


def test_the_waiver_does_not_waive_stacked_holes():
    """The courtyard check was the only one that saw two holes on top of
    each other (`pair_shortfall` checks a hole against COPPER), so a waived
    pair re-asks it: coincident NPTH drills are refused even under the
    waiver, holes 1.2mm apart (courtyards overlapping 0.8mm) seat."""
    with tempfile.TemporaryDirectory() as td:
        stacked = _npth_pair(td, 10.0)
        assert set(stacked['fixed_refused']) == {'A', 'B'}, stacked
        why = stacked['fixed_refused']['A']['reason']
        assert "drill A's hole" in why and "from B's hole" in why, why
        apart = _npth_pair(td, 11.2)
        assert set(apart['fixed_seated']) == {'A', 'B'}, apart['fixed_refused']
    print(f"  PASS: stacked holes refused under the waiver ({why}); apart "
          f"they seat")


def test_a_waiver_is_graded_even_where_the_mechanical_anchor_grades_the_pose():
    """`fixed_pose_violations` skips an entry whose pose the mechanical
    file's own anchor grades; its waiver must be graded anyway."""
    pcb = parse_kicad_pcb(ESP)
    r1 = pcb.footprints['R1']
    doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm',
           'fixed_poses': [{'ref': 'R1', 'x': r1.x, 'y': r1.y,
                            'rot': r1.rotation or 0, 'basis': 'declared',
                            'why': 'w', 'accept_courtyard_overlap': ['NOPE']}]}
    it = fp.intent_from_dict(doc)
    mech = {'poses': {'R1': {'x': r1.x, 'y': r1.y, 'rot': r1.rotation or 0}}}
    got = fp.fixed_pose_violations(it, pcb, ESP, mechanical=mech)
    assert any(v.rule == 'fixed_pose_unresolved' and 'NOPE' in v.message
               for v in got), [(v.rule, v.message) for v in got]
    print("  PASS: the waiver is graded though the mechanical anchor "
          "grades the pose")


def test_a_waiver_the_board_cannot_honour_is_refused():
    """A waiver naming a ref the board does not have refuses the pose at
    stage 0 and is `fixed_pose_unresolved` (error) in the grade; the loader
    refuses a malformed one outright."""
    with tempfile.TemporaryDirectory() as td:
        res = _pair_board(td, 25.0, waive=['NOPE'])
        assert 'A' in res['fixed_refused'], res['fixed_seated']
        assert 'NOPE' in res['fixed_refused']['A']['reason'], res
        base = {'ref': 'A', 'x': 1.0, 'y': 1.0, 'basis': 'declared',
                'why': 'w'}
        for acc, why, frag in (('B', 'w', 'expected a list'),
                               (['B*'], 'w', 'literal references'),
                               (['A'], 'w', 'names A itself'),
                               (['B', 'B'], 'w', 'repeats'),
                               (['B'], '', 'needs a `why`')):
            doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm',
                   'fixed_poses': [dict(base, why=why,
                                        accept_courtyard_overlap=acc)]}
            try:
                fp.intent_from_dict(doc)
            except fp.IntentError as exc:
                assert frag in str(exc), (acc, str(exc))
            else:
                raise AssertionError(f"loader accepted {acc!r}")
        pcb = parse_kicad_pcb(ESP)
        doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm',
               'fixed_poses': [{'ref': 'R1', 'x': pcb.footprints['R1'].x,
                                'y': pcb.footprints['R1'].y,
                                'basis': 'declared', 'why': 'w',
                                'accept_courtyard_overlap': ['NOPE']}]}
        g = fp.grade(fp.intent_from_dict(doc), pcb, ESP)
        hit = [v for v in g.errors if v.rule == 'fixed_pose_unresolved'
               and 'NOPE' in v.message]
        assert hit, [(v.rule, v.message) for v in g.violations]
    print("  PASS: an absent waiver ref refuses the pose and errors the "
          "grade; five malformed waivers refused at load")


def test_fixed_pose_obeys_the_keepout_band_and_pad_stacks_absolutely():
    """Review item 3: stage 0 must refuse what `pads_ok` refuses, ABSOLUTE
    rather than seed-relative -- #1031's rule-area keep-out band and a
    cross-part pad STACK were missing. (a) test_1031's R2 at (37.6, 15) puts
    pad 2 in the band (place_pose refuses it; grade_pad_legality reports it)
    and was seated 'contained'; (30, 15) is the clear control. (b) Two
    declared parts whose courtyards do not touch but whose SAME-net pads
    lie on each other: no clearance shortfall (same net), but a stack, and
    both declarations are refused."""
    import test_1031_keepout_legality as t1031
    with tempfile.TemporaryDirectory() as td:
        bd = os.path.join(td, 'ko.kicad_pcb')
        with open(bd, 'w', encoding='utf-8') as fh:
            fh.write(t1031.board_text(t1031.default_parts()))
        got = {}
        for x in (37.6, 30.0):
            doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm',
                   'fixed_poses': [{'ref': 'R2', 'x': x, 'y': 15.0,
                                    'rot': 0, 'basis': 'declared'}]}
            got[x] = seeder.seed_from_intent(
                parse_kicad_pcb(bd), bd, fp.intent_from_dict(doc, bd),
                random.Random('0'), group_sources=(), clearance=0.2)
        why = got[37.6]['fixed_refused']['R2']['reason']
        assert 'mm into a rule-area keep-out band' in why, why
        assert got[30.0]['fixed_seated']['R2']['how'] == 'contained', got[30.0]

        part = """ (footprint "t:P" (layer "F.Cu") (uuid "fp-%(r)s") (at %(x)s 10)
  (property "Reference" "%(r)s" (at 0 0 0))
  (fp_rect (start -0.2 -0.2) (end 0.2 0.2) (layer "F.CrtYd") (uuid "c-%(r)s"))
  (pad "1" smd rect (at -0.9 0) (size 0.8 0.8) (layers "F.Cu") (net %(a)s "/N%(a)s") (uuid "%(r)s1"))
  (pad "2" smd rect (at 0.9 0) (size 0.8 0.8) (layers "F.Cu") (net %(b)s "/N%(b)s") (uuid "%(r)s2")))
"""
        nets = ''.join(' (net %d "/N%d")' % (i, i) for i in (1, 2, 3))
        body = ('(kicad_pcb (version 20241229) (net 0 "")' + nets
                + ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal))'
                + ' (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts")'
                + ' (uuid "e1"))\n'
                + part % dict(r='A', x=5, a=1, b=2)
                + part % dict(r='B', x=25, a=2, b=3) + ')\n')
        sb = os.path.join(td, 'stack.kicad_pcb')
        with open(sb, 'w', encoding='utf-8') as fh:
            fh.write(body)
        # A pad 2 (net 2) at 15.9; B pad 1 (net 2) at 16.8 - 0.9 = 15.9.
        doc = {'schema': 1, 'kind': fp.KIND, 'units': 'mm', 'fixed_poses': [
            {'ref': 'A', 'x': 15.0, 'y': 10.0, 'rot': 0, 'basis': 'declared'},
            {'ref': 'B', 'x': 16.8, 'y': 10.0, 'rot': 0,
             'basis': 'declared'}]}
        res = seeder.seed_from_intent(
            parse_kicad_pcb(sb), sb, fp.intent_from_dict(doc, sb),
            random.Random('0'), group_sources=(), clearance=0.2,
            board_edge_clearance=0.1)
        assert set(res['fixed_refused']) == {'A', 'B'}, res['fixed_seated']
        assert "pads stack on B's copper" in \
            res['fixed_refused']['A']['reason'], res['fixed_refused']
    print(f"  PASS: R2 into the band refused ({why}); the clear pose seats; "
          f"a same-net pad stack refuses both declarations")


def test_refused_fixed_pose_stays_unwritten_under_anchors_first():
    """`--anchors-first` builds its anchor queue from `unplaced` directly,
    not through `_order`, so a REFUSED fixed pose (held in `unplaced`) was
    seated there and written anyway (phase-3 verifier). CON2 is esp_prog's
    largest free part, so it IS an anchor candidate -- the control arm
    without the fixed pose shows it in the anchor list."""
    with tempfile.TemporaryDirectory() as td:
        doc = fp.emit_intent(parse_kicad_pcb(ESP), ESP)
        _i, _p = _intent(doc, td, 'ctl.json')
        _pcb, ctl = _seed(ESP, _i, anchors_first=True)
        anote = [n for n in ctl['notes'] if n.startswith('anchors-first:')]
        assert anote and 'CON2' in anote[0], anote
        doc['fixed_poses'] = [{'ref': 'CON2', 'x': 100.0, 'y': 100.0,
                               'rot': 0, 'basis': 'declared',
                               'why': 'off the board'}]
        intent, _p = _intent(doc, td)
        _pcb, res = _seed(ESP, intent, anchors_first=True)
        assert 'CON2' in res['fixed_refused'], res['fixed_refused']
        assert 'CON2' in res['unseated'], res['unseated']
        # #1151: a refused part may be STAGED off the board (a row at its
        # staging slot) -- never seated by the anchors queue.
        if 'CON2' in {p['reference'] for p in res['placements']}:
            assert (res['unseated_disposition']['CON2']['disposition']
                    == 'staged'), res['unseated_disposition'].get('CON2')
        anote = [n for n in res['notes'] if n.startswith('anchors-first:')]
        assert anote and 'CON2' not in anote[0], anote
    print("  PASS: a refused fixed pose is no anchor and is not written "
          "under anchors_first (the control lists CON2 as an anchor)")


def test_stage1_treats_a_stage0_part_as_an_obstacle():
    import test_run27_edge_seat_clears_placed as t27
    with tempfile.TemporaryDirectory() as td:
        path = t27._board(td, 'b.kicad_pcb', locked=False)
        base = dict(t27.INTENT)

        def seed(doc):
            pcb = parse_kicad_pcb(path)
            res = seeder.seed_from_intent(
                pcb, path, fp.intent_from_dict(doc, path),
                random.Random('27'), group_sources=(), clearance=0.2,
                board_edge_clearance=0.3, grid_step=0.1)
            out = os.path.join(td, 'o.kicad_pcb')
            write_placed_output(path, out, res['placements'])
            pose = {p['reference']: (round(p['new_x'], 3),
                                     round(p['new_y'], 3))
                    for p in res['placements']}
            return res, pose, grade_pad_legality(parse_kicad_pcb(out), 0.2,
                                                 pcb_file=out)

        # CONTROL: FIX1 in the pile, so the header takes the band midpoint --
        # exactly where FIX1's fixed pose will be.
        _r, ctl, _g = seed(base)
        assert abs(ctl['J1'][0] - 15.0) < 0.5, ctl['J1']
        doc = dict(base, fixed_poses=[{'ref': 'FIX1', 'x': 15.0, 'y': 18.4,
                                       'rot': 0, 'basis': 'declared',
                                       'why': 'datum'}])
        res, pose, g = seed(doc)
        assert pose['FIX1'] == (15.0, 18.4), pose['FIX1']
        assert 'FIX1' in res['fixed_seated'], res['fixed_refused']
        assert g['pad_conflicts'] == 0, g
        assert abs(pose['J1'][0] - 15.0) > 1.0, pose['J1']
    print(f"  PASS: with FIX1 seated at stage 0 the header slides to "
          f"x={pose['J1'][0]} (control: {ctl['J1'][0]}), no pad conflict")


def test_unarmed_seeds_are_identical_to_the_pre_phase3_seeder():
    with open(BASELINE, encoding='utf-8') as fh:
        base = json.load(fh)['runs']
    boards = {'watchy': WATCHY, 'esp_prog': ESP}
    n = 0
    with tempfile.TemporaryDirectory() as td:
        for key, want in sorted(base.items()):
            board, s = key.split(':')
            path = boards[board]
            doc = fp.emit_intent(parse_kicad_pcb(path), path)
            assert not doc.get('arrays') and not doc.get('fixed_poses')
            assert (doc.get('decaps') or {}).get('max_distance_mm') is None
            intent, _p = _intent(doc, td, f'{board}.json')
            _pcb, res = _seed(path, intent, seed=s)
            got = {p['reference']: [p['new_x'], p['new_y'],
                                    p['new_rotation'], p.get('new_side')]
                   for p in res['placements']}
            assert got == want['placements'], (
                key, sorted(r for r in set(got) | set(want['placements'])
                            if got.get(r) != want['placements'].get(r)))
            assert res['unseated'] == want['unseated'], key
            assert res['lock_refs'] == want['lock_refs'], key
            assert res['decap_stage']['armed'] is False
            assert not res['arrays_formed'] and not res['fixed_seated']
            n += len(got)
    print(f"  PASS: {len(base)} unarmed seeds ({n} placements) are identical "
          f"to the pre-phase-3 seeder's")


TESTS = [
    test_splitflap_u4_row_is_formed,
    test_glasgow_resistor_pair_and_buffer_bank_are_formed,
    test_row_runs_the_way_its_host_pins_run,
    test_sibling_recheck_reverts_a_row_whose_pads_collide,
    test_formed_rows_are_immovable_to_the_eviction_rung,
    test_anchor_rounds_leave_a_formed_row_whole,
    test_rows_seat_only_their_hosts_first,
    test_a_row_member_is_never_seated_alone_before_its_row,
    test_row_seat_reaches_the_sweep_before_the_fine_rings,
    test_row_target_is_the_partner_centroid_not_the_host_pins,
    test_padless_fixed_pose_is_judged_by_its_hole,
    test_unseatable_row_is_disclosed_and_falls_through,
    test_pose_cap_trips_and_says_so,
    test_decaps_armed_claims_caps_once_the_owners_are_seated,
    test_the_decap_stage_says_why_it_claims_nothing,
    test_zero_claim_reports_why,
    test_fixed_pose_exact_locked_and_survives_repair_and_force,
    test_every_unhonoured_fixed_pose_fails_the_gate,
    test_illegal_fixed_pose_is_refused_not_nudged,
    test_human_glasgow_rows_seat_as_fixed_poses,
    test_a_real_overlap_is_refused_with_its_measurement,
    test_abutting_fixed_poses_seat_and_overlapping_ones_both_refuse,
    test_a_named_courtyard_waiver_seats_u30_exactly,
    test_the_waiver_covers_courtyards_only_both_ways,
    test_a_waiver_the_board_cannot_honour_is_refused,
    test_the_waiver_does_not_waive_stacked_holes,
    test_a_waiver_is_graded_even_where_the_mechanical_anchor_grades_the_pose,
    test_fixed_pose_obeys_the_keepout_band_and_pad_stacks_absolutely,
    test_refused_fixed_pose_stays_unwritten_under_anchors_first,
    test_stage1_treats_a_stage0_part_as_an_obstacle,
    test_unarmed_seeds_are_identical_to_the_pre_phase3_seeder,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
    print('ALL PASS')
