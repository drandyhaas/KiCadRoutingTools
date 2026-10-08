#!/usr/bin/env python3
"""#1105: the per-supply-pin decap claim, run again once the owner ICs are
seated (seeder stage 3.5), and an emitter that promises only what the seed
can claim.

On a pile or a flat board stage 2.5 finds no placed owner IC, so it claims
nothing, while `check_floorplan --emit-intent` used to promise "will seat 38
cap(s) per supply pin" (run 38, StickHub). Stage 3.5 runs the same claim
inside stage 3, after the queue's last owner IC. What each case pins:

* every part with three or more connected pads -- every IC -- is seated
  exactly as with the stage off, on esp_prog and watchy (the claim draws no
  RNG and runs after them); a cap does move, and the stage claimed some.
* a flat board (esp_prog, splitflap) and the committed run-29 pile claim caps
  at stage 3.5 where 2.5 claimed none, every claim note tagged, and each
  claimed cap written where its note says it landed.
* a seat that lands past the decap limit is undone and the cap keeps its own
  centroid turn (`DECAP_LATE_WITHIN_LIMIT`): on the run-29 pile two of U1's
  caps are declined and say so.
* no supply pin is served twice: with one owner declared as a fixed pose, the
  owners 2.5 served and the owners 3.5 served are disjoint.
* a cap whose only owner is not U-prefixed is still never claimed (control:
  `decap_owner_chips=True` claims it).
* the stage OFF is the pre-#1105 seeder (tests/fixtures/1105/
  seed_off_baseline.json, recorded at 4aa33a88 with `--record`).
* the emitter's line and its `seeder_forecast` say which caps 2.5 claims,
  which only 3.5 can claim (and that 3.5 is off unless asked), and which no
  stage can claim; the forecast agrees with what the seed did.
* place_seed's `--decap-claim-after-ics` / `--no-decap-claim-after-ics`
  reach the seeder and print what the stage did.
* `DECAP_CLAIM_AFTER_ICS_DEFAULT` is what tests/test_placement_ab.py's
  `decap-*` rows measured: ON only if a variant passed the gate.

    python3 tests/test_1105_decap_claim_after_ics.py [name-substring ...]
    python3 tests/test_1105_decap_claim_after_ics.py --record <repo root>
"""
import json
import os
import random
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
#: `--record <root>` seeds with ANOTHER checkout's code (the pre-#1105 one)
#: and rewrites the OFF-arm fixture from it.
_REC = (sys.argv.index('--record')
        if '--record' in sys.argv[1:] else None)
ROOT = (os.path.abspath(sys.argv[_REC + 1]) if _REC is not None
        else os.path.dirname(TESTS_DIR))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb           # noqa: E402
from placement import floorplan as fp              # noqa: E402
from placement import seeder                       # noqa: E402
from placement.writer import write_placed_output   # noqa: E402

RUN_ALL_TIMEOUT = 900

REPO = os.path.dirname(TESTS_DIR)
BOARDS = os.path.join(REPO, 'kicad_files')
ESP = os.path.join(BOARDS, 'esp_prog.kicad_pcb')
WATCHY = os.path.join(BOARDS, 'watchy.kicad_pcb')
SPLITFLAP = os.path.join(BOARDS, 'splitflap_driver.kicad_pcb')
PILE = os.path.join(TESTS_DIR, 'fixtures', '959', 'run29_pile.kicad_pcb')
BASELINE = os.path.join(TESTS_DIR, 'fixtures', '1105',
                        'seed_off_baseline.json')
PLACE_SEED = os.path.join(REPO, 'py_placer', 'place_seed.py')
CHECK_FLOORPLAN = os.path.join(REPO, 'py_tools', 'check_floorplan.py')
SOURCES = ('kicad', 'sheet')
CLEARANCE = 0.2
LIMIT = 3.0
TAG = '(stage 3.5)'


def _doc(board):
    """The board's own intent, armed with a 3 mm decap limit. The pile has
    no limit of its own (a pile withholds it), so it gets esp_prog's."""
    if board == PILE:
        return fp.emit_intent(parse_kicad_pcb(PILE), PILE, decaps_from=ESP)
    doc = fp.emit_intent(parse_kicad_pcb(board), board)
    doc['decaps'] = dict(doc.get('decaps') or {}, max_distance_mm=LIMIT)
    return doc


def _intent(doc, td, name):
    path = os.path.join(td, name)
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=1)
    return fp.load_intent(path), path


def _seed(board, intent, seed='0', **kw):
    pcb = parse_kicad_pcb(board)
    return pcb, seeder.seed_from_intent(
        pcb, board, intent, random.Random(seed), group_sources=SOURCES,
        clearance=CLEARANCE, **kw)


class _knobs:
    """Set seeder module knobs for one block, and restore them."""

    def __init__(self, **kw):
        self.kw, self.saved = kw, {}

    def __enter__(self):
        for k, v in self.kw.items():
            self.saved[k] = getattr(seeder, k)
            setattr(seeder, k, v)

    def __exit__(self, *exc):
        for k, v in self.saved.items():
            setattr(seeder, k, v)
        return False


def _poses(res):
    return {p['reference']: (p['new_x'], p['new_y'], p['new_rotation'])
            for p in res['placements']}


def _net_pads(pcb, ref):
    return sum(1 for p in pcb.footprints[ref].pads if p.net_id > 0)


def _late_notes(res):
    return [n for n in res['notes'] if n.endswith(TAG) and 'decap for' in n]


def test_every_ic_is_seated_exactly_as_with_the_stage_off():
    """The claim runs after the last owner IC and draws no RNG, so every part
    stage 3 seats before it is the stage-off seed's. Pinned as: every part
    with 3+ connected pads has the same pose in both arms."""
    with tempfile.TemporaryDirectory() as td:
        for board in (ESP, WATCHY):
            intent, _p = _intent(_doc(board), td, os.path.basename(board))
            pcb, off = _seed(board, intent, decap_claim_after_ics=False)
            _pcb, on = _seed(board, intent, decap_claim_after_ics=True)
            p_off, p_on = _poses(off), _poses(on)
            big = sorted(r for r in p_off if _net_pads(pcb, r) >= 3)
            assert big, board
            moved = [r for r in big if p_off[r] != p_on.get(r)]
            assert not moved, (board, moved)
            late = on['decap_stage']['late']
            assert late['armed'] and late['claimed'] > 0, late
            # anti-vacuity: the stage changed something, and only small parts
            assert any(p_off[r] != p_on.get(r) for r in late['caps']), board
            assert all(_net_pads(pcb, r) <= 2 for r in late['caps']), late
    print(f"  PASS: {len(big)} IC/3+-pad pose(s) on watchy identical in both "
          f"arms; the stage claimed {late['claimed']} cap(s) there")


def test_a_flat_board_claims_after_its_owner_ics():
    with tempfile.TemporaryDirectory() as td:
        got = {}
        for board in (ESP, SPLITFLAP):
            intent, out = _intent(_doc(board), td, os.path.basename(board))
            pcb, res = _seed(board, intent, decap_claim_after_ics=True)
            ds = res['decap_stage']
            assert ds['claimed'] == 0, ds        # nothing placed before 2.5
            late = ds['late']
            assert late['claimed'] > 0 and late['reason'] is None, late
            notes = _late_notes(res)
            assert sorted(n.split(':')[0] for n in notes) == \
                sorted(late['caps']), (notes, late['caps'])
            # each claimed cap is WRITTEN where its note says it landed
            wb = os.path.join(td, 'w_' + os.path.basename(board))
            write_placed_output(board, wb, res['placements'])
            written = parse_kicad_pcb(wb).footprints
            for n in notes:
                m = re.match(r'(\S+): decap for (\S+) pad\(s\) near '
                             r'\(([-\d.]+), ([-\d.]+)\).*landed ([\d.]+)mm',
                             n)
                assert m, n
                ref = m.group(1)
                tx, ty, d = (float(m.group(3)), float(m.group(4)),
                             float(m.group(5)))
                f = written[ref]
                assert abs(((f.x - tx) ** 2 + (f.y - ty) ** 2) ** 0.5 - d) \
                    < 0.006, (n, f.x, f.y)
            got[os.path.basename(board)] = late['claimed']
    print(f"  PASS: stage 2.5 claims 0 and stage 3.5 claims {got}; every "
          f"claim is noted and written where the note says")


def test_the_pile_claims_after_its_owner_ics():
    with tempfile.TemporaryDirectory() as td:
        intent, _p = _intent(_doc(PILE), td, 'pile.json')
        _pcb, res = _seed(PILE, intent, decap_claim_after_ics=True)
        ds = res['decap_stage']
        # Seeded whole (no seed_refs): nothing is placed before 2.5, so every
        # owner -- U1, and USB1 too, a U-prefixed connector -- is stage 3's.
        assert ds['claimed'] == 0 and ds['pins'] == 0, ds
        late = ds['late']
        assert 'U1' in late['owners'] and late['claimed'] >= 1, late
    print(f"  PASS: run 29's pile: 2.5 claims {ds['claimed']}, 3.5 claims "
          f"{late['claimed']} at {late['owners']}")


def test_a_seat_past_the_limit_is_undone():
    """Two adjacent supply pins on different rails send their caps to one
    spot; the second lands millimetres from the pin it claimed. Under
    DECAP_LATE_WITHIN_LIMIT a seat farther from its IC than the limit AS THE
    GRADE MEASURES IT (`seeder.decap_graded_distance`: pad centroid to the
    elected chip's pad box) is undone and the cap keeps its own centroid
    turn; a seat far from its pin target but within the limit of its IC is
    kept, because the grade accepts it (it used to be declined too, on the
    distance to the pin target). Control: with the decline off the same caps
    are claimed at the far seats."""
    with tempfile.TemporaryDirectory() as td:
        intent, _p = _intent(_doc(PILE), td, 'pile.json')
        lim = float(intent.decaps['max_distance_mm'])
        with _knobs(DECAP_LATE_WITHIN_LIMIT=True,
                    DECAP_LATE_AT='after_last_owner'):
            _pcb, on = _seed(PILE, intent, decap_claim_after_ics=True)
        with _knobs(DECAP_LATE_WITHIN_LIMIT=False,
                    DECAP_LATE_AT='after_last_owner'):
            _pcb, ctl = _seed(PILE, intent, decap_claim_after_ics=True)
        late, lctl = on['decap_stage']['late'], ctl['decap_stage']['late']
        assert late['declined'], late
        assert not set(late['declined']) & set(late['caps']), late
        for ref in late['declined']:
            note = [n for n in on['notes']
                    if n.startswith(f"{ref}: stage 3.5 declined its seat")]
            assert note and f"{lim:g}mm decap limit" in note[0], note
            assert float(re.search(r'landed ([\d.]+)mm from \S+ as the '
                                   r'grade measures it', note[0]).group(1)
                         ) > lim, note
        far_but_graded_in = []
        for n in _late_notes(on):
            graded = float(re.search(r', ([\d.]+)mm from \S+ as graded',
                                     n).group(1))
            assert graded <= lim + 1e-9, n
            if float(re.search(r'landed ([\d.]+)mm', n).group(1)) > lim:
                far_but_graded_in.append(n.split(':')[0])
        # the defect the graded measure fixes: a seat past the limit from its
        # PIN TARGET but inside it from its IC's pad box is the grade's pass,
        # and is kept (the pin-target measure declined C2 here, 7.07 mm from
        # its pin and 0.82 mm from U1)
        assert far_but_graded_in, _late_notes(on)
        # control: the same caps are CLAIMED when the decline is off
        assert set(late['declined']) <= set(lctl['caps']), (late, lctl)
        assert lctl['declined'] == [], lctl
    print(f"  PASS: {late['declined']} declined past the {lim:g} mm limit "
          f"(claimed at the far seat when the decline is off)")


def test_the_decline_measures_as_the_grade():
    """`seeder.decap_graded_distance` -- what the within-limit check reads --
    is the grade's own election on a board at its file poses: for every
    graded tether on esp_prog and watchy it returns the chip and distance
    `groups.decap_populations` (the population `rule_decap_distance` grades)
    holds. A chip that is not yet placed is no candidate (it still sits at
    its staging pose): with nothing placed there is no tether to measure."""
    import pose_score
    from placement import groups as groups_mod
    n = 0
    for board in (ESP, WATCHY):
        pcb = parse_kicad_pcb(board)
        state = pose_score.make_state(pcb, board)
        near, beyond, _o = groups_mod.decap_populations(pcb)
        pairs = [(cap, ic, d) for ic, caps in near.items()
                 for cap, d in caps] + list(beyond)
        assert pairs, board
        for cap, ic, d in pairs:
            chips = groups_mod.rail_chips(pcb, cap)
            got = seeder.decap_graded_distance(pcb, state, cap, chips,
                                               set(chips))
            assert got[0] == ic and abs(got[1] - d) < 1e-6, (cap, got, ic, d)
            assert seeder.decap_graded_distance(
                pcb, state, cap, chips, set()) == (None, None), cap
            n += 1
    print(f"  PASS: {n} tether(s) on esp_prog and watchy measured exactly as "
          f"the grade elects them; no placed chip, no tether")


def test_the_decline_reads_live_poses_and_the_rail():
    """Every measure the within-limit check takes during a real seed, re-taken
    beside it: the cap and its chips POSED where the seed has them at that
    moment (not at their file poses), and the chips the call is handed are
    exactly the cap's rail chips (`groups.rail_chips`). A measure read at the
    file pose flipped run 29's C3 from declined to kept ("0.00mm from U1")
    with every other case here still passing."""
    from placement import groups as groups_mod
    from placement.legality import footprint_at_pose
    real = seeder.decap_graded_distance
    seen = []

    def spy(pcb_data, state, cap, chips, placed):
        got = real(pcb_data, state, cap, chips, placed)

        def posed(r):
            p = state.parts[r]
            return footprint_at_pose(pcb_data.footprints[r],
                                     (p.x, p.y, p.rot))
        assert list(chips) == groups_mod.rail_chips(pcb_data, cap), (
            cap, chips)
        cands = [(c, groups_mod.chip_bounds_of(posed(c)))
                 for c in chips if c in placed]
        want = groups_mod.elect_live(posed(cap),
                                     [x for x in cands if x[1] is not None])
        seen.append((cap, got, want))
        return got
    with tempfile.TemporaryDirectory() as td:
        intent, _p = _intent(_doc(PILE), td, 'pile.json')
        seeder.decap_graded_distance = spy
        try:
            with _knobs(DECAP_LATE_WITHIN_LIMIT=True,
                        DECAP_LATE_AT='after_last_owner'):
                _seed(PILE, intent, decap_claim_after_ics=True)
        finally:
            seeder.decap_graded_distance = real
    assert seen, 'the within-limit check never measured anything'
    bad = [s for s in seen if s[1][0] != s[2][0]
           or abs((s[1][1] or 0.0) - (s[2][1] or 0.0)) > 1e-9]
    assert not bad, bad
    print(f"  PASS: {len(seen)} within-limit measure(s) on run 29's pile, "
          f"each the cap's live-pose distance to its rail's placed chips")


def test_no_supply_pin_is_served_twice():
    """One owner declared as a fixed pose (2.5 serves it), the rest left to
    stage 3: the owners each stage served are disjoint, and no cap is
    claimed twice."""
    from placement import groups as groups_mod
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(SPLITFLAP)
        near, beyond, _o = groups_mod.decap_populations(pcb)
        owners = sorted(r for r in set(near) | {ic for _c, ic, _d in beyond}
                        if r.startswith('U'))
        assert len(owners) >= 2, owners
        first = owners[0]
        f = pcb.footprints[first]
        doc = _doc(SPLITFLAP)
        doc['fixed_poses'] = [{'ref': first, 'x': f.x, 'y': f.y,
                               'rot': f.rotation or 0, 'basis': 'declared',
                               'why': 'one owner seated before 2.5'}]
        intent, _p = _intent(doc, td, 'one_fixed.json')
        _pcb, res = _seed(SPLITFLAP, intent, decap_claim_after_ics=True)
        ds = res['decap_stage']
        assert ds['claimed'] > 0 and ds['late']['claimed'] > 0, ds
        early = {re.match(r'\S+: decap for (\S+) ', n).group(1)
                 for n in res['notes']
                 if 'decap for' in n and not n.endswith(TAG)}
        late = {re.match(r'\S+: decap for (\S+) ', n).group(1)
                for n in _late_notes(res)}
        assert early and late and not early & late, (early, late)
        assert not early & set(ds['late']['owners']), (early, ds['late'])
        caps = [n.split(':')[0] for n in res['notes'] if 'decap for' in n]
        assert len(caps) == len(set(caps)), caps
    print(f"  PASS: 2.5 served {sorted(early)}, 3.5 served {sorted(late)}; "
          f"disjoint, {len(caps)} cap(s) each claimed once")


def test_a_non_u_owner_is_still_not_claimed():
    import test_792_decap_seeding as t792
    with tempfile.TemporaryDirectory() as wd:
        # Only C3 in scope; its owner IC1 is not U-prefixed.
        res, _poses_, _pcb = t792._seed(
            wd, {'max_distance_mm': 3.0, 'exempt': ['C1', 'C2', 'C5']},
            decap_claim_after_ics=True)
        ds = res['decap_stage']
        assert ds['claimed'] == 0 and ds['late']['claimed'] == 0, ds
        assert 'U-prefixed' in (ds['late']['reason'] or ''), ds['late']
    # ...and where the rule is observable: IC1 seated BEFORE stage 2.5 (a
    # fixed pose). Its pins are on the board, and still no stage claims C3.
    with tempfile.TemporaryDirectory() as wd:
        path = t792._board(os.path.join(wd, 'b.kicad_pcb'))
        pcb = parse_kicad_pcb(path)
        doc = t792._intent({'max_distance_mm': 3.0,
                            'exempt': ['C1', 'C2', 'C5']})
        ic1 = pcb.footprints['IC1']
        doc['fixed_poses'] = [{'ref': 'IC1', 'x': ic1.x, 'y': ic1.y,
                               'rot': ic1.rotation or 0, 'basis': 'declared',
                               'why': 'IC1 placed before the pin stage'}]
        doc['min_reader'] = 7
        res2 = seeder.seed_from_intent(
            pcb, path, fp.intent_from_dict(doc, path), random.Random(11),
            decap_claim_after_ics=True)
        ds2 = res2['decap_stage']
        assert 'IC1' in res2['fixed_seated'], res2['fixed_seated']
        assert ds2['claimed'] == 0 and ds2['late']['claimed'] == 0, ds2
    with tempfile.TemporaryDirectory() as wd:
        ctl, _poses_, _pcb = t792._seed(
            wd, {'max_distance_mm': 3.0, 'exempt': ['C1', 'C2', 'C5']},
            decap_claim_after_ics=True, decap_owner_chips=True)
        got = ctl['decap_stage']['claimed'] + \
            ctl['decap_stage']['late']['claimed']
        assert got == 1, ctl['decap_stage']
    print(f"  PASS: IC1's cap is never claimed ({ds['late']['reason']}); "
          f"decap_owner_chips claims it")


#: (board, seed) pairs the OFF-arm fixture records.
_OFF_RUNS = (('esp_prog', '0'), ('esp_prog', '1'), ('watchy', '0'),
             ('pile', '0'))
_OFF_BOARDS = {'esp_prog': ESP, 'watchy': WATCHY, 'pile': PILE}


def _off_run(board, seed, td, **kw):
    path = _OFF_BOARDS[board]
    intent, _p = _intent(_doc(path), td, f'{board}.json')
    _pcb, res = _seed(path, intent, seed=seed, **kw)
    return {'placements': {r: list(v) for r, v in _poses(res).items()},
            'unseated': res['unseated'], 'notes': res['notes'],
            'decap_stage': res['decap_stage']}


def test_the_stage_off_is_the_pre_1105_seeder():
    with open(BASELINE, encoding='utf-8') as fh:
        base = json.load(fh)
    assert base['recorded_at'], base
    n = 0
    with tempfile.TemporaryDirectory() as td:
        for board, seed in _OFF_RUNS:
            want = base['runs'][f'{board}:{seed}']
            got = _off_run(board, seed, td, decap_claim_after_ics=False)
            assert got['placements'] == want['placements'], (
                board, seed, sorted(r for r in got['placements']
                                    if got['placements'][r]
                                    != want['placements'].get(r)))
            assert got['unseated'] == want['unseated'], (board, seed)
            assert got['notes'] == want['notes'], (board, seed)
            old = want['decap_stage']
            assert {k: got['decap_stage'][k] for k in old} == old, (
                board, seed, got['decap_stage'])
            assert got['decap_stage']['late']['armed'] is False
            n += len(got['placements'])
    print(f"  PASS: {len(_OFF_RUNS)} armed seeds ({n} placements, notes, "
          f"decap_stage) identical to the seeder at {base['recorded_at']}")


def _emit(td, extra=()):
    out = os.path.join(td, 'emitted.json')
    r = subprocess.run([sys.executable, '-X', 'utf8', CHECK_FLOORPLAN, PILE,
                        '--allow-unplaced', '--emit-intent', out,
                        '--decaps-from', ESP] + list(extra),
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO)
    assert r.returncode == 0, r.stdout[-800:] + r.stderr[-800:]
    with open(out, encoding='utf-8') as fh:
        return r.stdout, json.load(fh)


def test_the_emitter_promises_only_what_the_seed_can_claim():
    with tempfile.TemporaryDirectory() as td:
        stdout, doc = _emit(td)
        line = [ln for ln in stdout.splitlines() if 'READS this key' in ln]
        assert len(line) == 1, stdout[-1500:]
        f = doc['context']['decap_census']['seeder_forecast']
        assert f['early'] == ['C1'] and f['early_owners'] == ['USB1'], f
        assert f['late'] and 'U1' in f['late_owners'], f
        assert f['late_armed'] is seeder.DECAP_CLAIM_AFTER_ICS_DEFAULT, f
        if f['late_armed']:
            assert 'at stage 3.5' in line[0], line[0]
        else:
            assert 'are NOT claimed' in line[0], line[0]
            assert '--decap-claim-after-ics' in line[0], line[0]
        n = len(f['early']) + (len(f['late']) if f['late_armed'] else 0)
        assert f"up to {n} of {f['scope']}" in line[0], line[0]
        # control: U1 declared as a fixed pose moves its caps to stage 2.5
        pcb = parse_kicad_pcb(PILE)
        u1 = pcb.footprints['U1']
        doc2 = dict(doc, fixed_poses=[{'ref': 'U1', 'x': u1.x, 'y': u1.y,
                                       'rot': u1.rotation or 0,
                                       'basis': 'declared', 'why': 'ctl'}])
        intent2 = fp.intent_from_dict(doc2, PILE)
        blocks, _ = fp.resolve_blocks(intent2, pcb, SOURCES)
        f2 = seeder.decap_pin_forecast(pcb, intent2, blocks,
                                       standing=f['standing'])
        assert not set(f['late']) - set(f2['early']), (f, f2)
        assert 'U1' in f2['early_owners'] and not f2['late'], f2
    print(f"  PASS: the pile's line names {len(f['early'])} cap(s) at stage "
          f"2.5 and {len(f['late'])} that only stage 3.5 can claim; a fixed "
          f"pose for U1 moves them to 2.5")


def test_the_forecast_agrees_with_the_seed():
    """watchy has caps no U-prefixed part can own: the forecast names them,
    and neither stage claims one. Every owner stage 3.5 served is one the
    forecast listed."""
    with tempfile.TemporaryDirectory() as td:
        doc = _doc(WATCHY)
        intent, _p = _intent(doc, td, 'watchy.json')
        pcb = parse_kicad_pcb(WATCHY)
        blocks, _ = fp.resolve_blocks(intent, pcb, SOURCES)
        f = seeder.decap_pin_forecast(pcb, intent, blocks,
                                      claim_after_ics=True)
        assert f['ownerless'], f
        _pcb, res = _seed(WATCHY, intent, decap_claim_after_ics=True)
        claimed = set(re.match(r'(\S+): decap for', n).group(1)
                      for n in res['notes'] if 'decap for' in n)
        assert claimed and not claimed & set(f['ownerless']), (claimed, f)
        # The forecast assumes every declared early seat succeeds; a stage-1
        # seat that fails leaves its owner (and its caps) to stage 3.5, so
        # the seed's claims are bounded by early + late, never ownerless.
        late = res['decap_stage']['late']
        owners = (set(f['early_owners']) | set(f['late_owners'])
                  | set(f['backup_owners']))
        assert set(late['owners']) <= owners, (late, f)
        assert set(late['caps']) <= set(f['early']) | set(f['late']), (late, f)
    print(f"  PASS: {len(f['ownerless'])} ownerless cap(s) on watchy, none "
          f"claimed; 3.5's {len(late['caps'])} claim(s) are all forecast")


def _summary(stdout):
    m = re.search(r'^JSON_SUMMARY: (.*)$', stdout, re.M)
    assert m, stdout[-1500:]
    return json.loads(m.group(1))


def test_place_seed_flag_pair():
    h = subprocess.run([sys.executable, '-X', 'utf8', PLACE_SEED, '--help'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO)
    assert '--decap-claim-after-ics' in h.stdout, h.stdout[-500:]
    assert '--no-decap-claim-after-ics' in h.stdout, h.stdout[-500:]
    with tempfile.TemporaryDirectory() as td:
        _i, ipath = _intent(_doc(PILE), td, 'pile.json')
        got = {}
        for flag in ('--no-decap-claim-after-ics', '--decap-claim-after-ics'):
            out = os.path.join(td, flag.strip('-') + '.kicad_pcb')
            r = subprocess.run([sys.executable, '-X', 'utf8', PLACE_SEED,
                                PILE, out, '--intent', ipath, '--seed', '0',
                                '--no-polish', flag],
                               capture_output=True, text=True,
                               encoding='utf-8', errors='replace', cwd=REPO)
            assert r.returncode in (0, 4), r.stdout[-800:] + r.stderr[-800:]
            got[flag] = (_summary(r.stdout)['decap_stage']['late'], r.stdout)
        late_off, so_off = got['--no-decap-claim-after-ics']
        late_on, so_on = got['--decap-claim-after-ics']
        assert late_off['armed'] is False and late_off['claimed'] == 0
        assert 'stage 3.5 is off (--decap-claim-after-ics arms it)' in so_off
        assert late_on['armed'] is True and late_on['claimed'] > 0, late_on
        assert '(3.5: ' in so_on and 'decap stage 3.5:' in so_on, so_on[-800:]
    print(f"  PASS: the flag pair reaches the seeder (3.5 claimed "
          f"{late_on['claimed']} on, 0 off) and the NOTE says which")


def test_the_default_is_the_measured_one():
    """Default ON only when a `decap-*` A/B family was adopted (no row of it
    `rejected`); every family the table rejected keeps the stage opt-in."""
    import test_placement_ab as ab
    fams = {}
    for row in ab.ROWS:
        m = re.match(r'(decap-(?:after-ics|within-limit|after-queue))-',
                     row['name'])
        if m:
            fams.setdefault(m.group(1), []).append(row)
    assert set(fams) == {'decap-after-ics', 'decap-within-limit',
                         'decap-after-queue'}, sorted(fams)
    assert all(len(v) >= 3 for v in fams.values()), fams
    adopted = [k for k, v in fams.items()
               if not any(r.get('rejected') for r in v)]
    assert seeder.DECAP_CLAIM_AFTER_ICS_DEFAULT is bool(adopted), adopted
    print(f"  PASS: DECAP_CLAIM_AFTER_ICS_DEFAULT is "
          f"{seeder.DECAP_CLAIM_AFTER_ICS_DEFAULT}; adopted families: "
          f"{adopted or 'none'}")


def _record():
    """Rewrite the OFF-arm fixture with the code under `--record <root>`."""
    import subprocess as _sp
    sha = _sp.run(['git', '-C', ROOT, 'rev-parse', '--short=8', 'HEAD'],
                  capture_output=True, text=True).stdout.strip()
    kw = {}
    import inspect
    if 'decap_claim_after_ics' in inspect.signature(
            seeder.seed_from_intent).parameters:
        kw['decap_claim_after_ics'] = False
    runs = {}
    with tempfile.TemporaryDirectory() as td:
        for board, seed in _OFF_RUNS:
            runs[f'{board}:{seed}'] = _off_run(board, seed, td, **kw)
    os.makedirs(os.path.dirname(BASELINE), exist_ok=True)
    with open(BASELINE, 'w', encoding='utf-8', newline='\n') as fh:
        json.dump({'recorded_at': sha, 'root': os.path.basename(ROOT),
                   'clearance': CLEARANCE, 'limit_mm': LIMIT, 'runs': runs},
                  fh, indent=1, sort_keys=True)
        fh.write('\n')
    print(f"wrote {BASELINE} from {ROOT} at {sha}")


TESTS = [
    test_every_ic_is_seated_exactly_as_with_the_stage_off,
    test_a_flat_board_claims_after_its_owner_ics,
    test_the_pile_claims_after_its_owner_ics,
    test_a_seat_past_the_limit_is_undone,
    test_the_decline_measures_as_the_grade,
    test_the_decline_reads_live_poses_and_the_rail,
    test_no_supply_pin_is_served_twice,
    test_a_non_u_owner_is_still_not_claimed,
    test_the_stage_off_is_the_pre_1105_seeder,
    test_the_emitter_promises_only_what_the_seed_can_claim,
    test_the_forecast_agrees_with_the_seed,
    test_place_seed_flag_pair,
    test_the_default_is_the_measured_one,
]


if __name__ == '__main__':
    if _REC is not None:
        _record()
        sys.exit(0)
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
