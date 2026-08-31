"""The first tests stage 2.5 has ever had (#792).

Before this file, `seeder.py`'s 160-line per-supply-pin stage was executed by NO
test: `decap_scope` appeared only in `seeder.py`, the note string "decap for ...
pad(s) near" appeared nowhere under `tests/`, and no test anywhere constructed
an intent with `decaps.max_distance_mm` and called `seed_from_intent`. Every
claim in its comments -- "all ten caps claimed U1 pads, U3.8 graded 3.5mm",
"twelve graded pins over ten caps is the DESIGNED shape" -- was unverified prose.

A HAND-BUILT BOARD, not a corpus one, following `test_630_seeder_eviction`'s
argument: on a corpus board the answer depends on two hundred other parts, and
what this file needs to assert are theorems. The board is:

    U1    16 pads, 2-D extent, U-prefixed  -- two VCC pins 4mm apart, two GND
    IC1    8 pads, 2-D extent, NOT U-prefixed -- one VCC2 pin
    CN1   10 pads in ONE ROW (collinear), carrying VCC -- a castellated row,
          which `groups._pads_are_collinear` refuses and a ref prefix does not
    C1 C2 C5 on (VCC, GND)     -- THREE caps for U1's TWO VCC pins, so the
          pin stage must decline one and the put-back must catch it
    C3    on (VCC2, GND)       -- IC1's decap; its owner is NOT U-prefixed
    C4    on (VBULK, GND)      -- VBULK is touched only by C4 and L1, a 2-pad
          inductor, so NO chip carries it: the orphan case
    L1    2 pads (VBULK, VIN)  -- the LC filter that makes C4 an orphan

Every arm asserts on the WRITTEN placement and on the stage's own notes, never
on a count of seated parts -- `test_630`'s recorded lesson is that "a count of
seated parts is not a board".
"""
import json
import os
import random
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('', 'py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402
from placement import groups as groups_mod                  # noqa: E402
from placement import seeder                                # noqa: E402

SIZE = (60.0, 40.0)
#: net id -> name. 1 GND, 2 VCC, 3 VCC2, 4 VBULK, 5 VIN.
NETS = {1: 'GND', 2: 'VCC', 3: 'VCC2', 4: 'VBULK', 5: 'VIN'}


def _pads(spec, layer='F.Cu'):
    """spec: [(number, net_id, dx, dy)]."""
    return ''.join(
        '\t\t(pad "{n}" smd rect\n\t\t\t(at {dx} {dy})\n'
        '\t\t\t(size 0.6 0.6)\n\t\t\t(layers "{L}")\n'
        '\t\t\t(net {net} "{nm}")\n\t\t\t(uuid "p{n}-{u}")\n\t\t)\n'.format(
            n=num, dx=dx, dy=dy, L=layer, net=nid, nm=NETS.get(nid, ''),
            u=uid)
        for num, nid, dx, dy, uid in spec)


def _fp(ref, x, y, hw, hh, spec):
    return ('\t(footprint "test:{r}"\n\t\t(layer "F.Cu")\n\t\t(uuid "fp-{r}")\n'
            '\t\t(at {x} {y})\n\t\t(property "Reference" "{r}"\n'
            '\t\t\t(at 0 0)\n\t\t)\n'
            '\t\t(fp_rect\n\t\t\t(start {a} {b})\n\t\t\t(end {c} {d})\n'
            '\t\t\t(layer "F.CrtYd")\n\t\t\t(uuid "cy-{r}")\n\t\t)\n'
            '{p}\t)\n').format(r=ref, x=x, y=y, a=-hw, b=-hh, c=hw, d=hh,
                               p=_pads(spec))


def _board(path):
    """The fixture. Pin coordinates are chosen so the answers are arithmetic."""
    parts = [
        # U1: 16 pads, two-dimensional extent. VCC at (-2,-2) and (+2,-2) --
        # 4mm apart, so two distinct pins rather than one collapsed cluster.
        _fp('U1', 30, 20, 3.0, 3.0,
            [('1', 2, -2, -2, 'a'), ('2', 2, 2, -2, 'b'),
             ('3', 1, -2, 2, 'c'), ('4', 1, 2, 2, 'd')]
            + [(str(5 + i), 0, -2.5 + 0.3 * i, 0.0, 'e%d' % i)
               for i in range(12)]),
        # IC1: a chip by the grouper's definition (8 pads, 2-D) and NOT
        # U-prefixed, which is the whole point of arm T4.
        _fp('IC1', 12, 12, 2.0, 2.0,
            [('1', 3, -1, -1, 'f'), ('2', 1, 1, -1, 'g')]
            + [(str(3 + i), 0, -1 + 0.4 * i, 1.0, 'h%d' % i)
               for i in range(6)]),
        # CN1: 10 pads in ONE ROW carrying VCC -- collinear, so the grouper
        # refuses it as an IC and a ref-prefix test has no opinion.
        _fp('CN1', 48, 32, 3.0, 0.8,
            [(str(1 + i), 2 if i == 0 else 0, -2.7 + 0.6 * i, 0.0, 'i%d' % i)
             for i in range(10)]),
        _fp('C1', 5, 5, 0.7, 0.5, [('1', 2, -0.4, 0, 'j'),
                                   ('2', 1, 0.4, 0, 'k')]),
        _fp('C2', 8, 5, 0.7, 0.5, [('1', 2, -0.4, 0, 'l'),
                                   ('2', 1, 0.4, 0, 'm')]),
        _fp('C3', 5, 8, 0.7, 0.5, [('1', 3, -0.4, 0, 'n'),
                                   ('2', 1, 0.4, 0, 'o')]),
        _fp('C4', 8, 8, 0.7, 0.5, [('1', 4, -0.4, 0, 'p'),
                                   ('2', 1, 0.4, 0, 'q')]),
        # A THIRD cap on VCC. U1 has only TWO VCC pins, so pass 2 forms two
        # clusters and its `zip` pairs two caps -- the third is never
        # claimed and must reach the put-back. Without it, arm T3 asserted
        # over a run in which the pin stage claimed everything, i.e. it
        # tested nothing (it printed 'put back [], declined []').
        _fp('C5', 11, 5, 0.7, 0.5, [('1', 2, -0.4, 0, 't'),
                                    ('2', 1, 0.4, 0, 'u')]),
        _fp('L1', 11, 8, 0.7, 0.5, [('1', 4, -0.4, 0, 'r'),
                                    ('2', 5, 0.4, 0, 's')]),
    ]
    nets = ''.join('\t(net {} "{}")\n'.format(i, n) for i, n in NETS.items())
    body = ('(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n' + nets
            + '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end {} {})\n'
              '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n'.format(*SIZE)
            + ''.join(parts) + ')\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)
    return path


def _intent(decaps, zones=()):
    doc = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
           "envelope": {"rect": [0.0, 0.0, SIZE[0], SIZE[1]],
                        "tolerance_mm": 0.5},
           "blocks": [dict(z) for z in zones]}
    if decaps is not None:
        doc["decaps"] = dict(decaps)
    return doc


def _seed(wd, decaps, zones=(), **kw):
    path = _board(os.path.join(wd, 'b.kicad_pcb'))
    pcb = parse_kicad_pcb(path)
    doc = _intent(decaps, zones)
    res = seeder.seed_from_intent(pcb, path, fp.intent_from_dict(doc, path),
                                  random.Random(11), **kw)
    poses = {p['reference']: (p['new_x'], p['new_y'])
             for p in res['placements']}
    return res, poses, pcb


def _notes(res, needle):
    return [n for n in res.get('notes') or [] if needle in n]


def test_the_fixture_is_the_board_the_arms_below_assume():
    """Every arm rests on WHICH parts the grouper calls chips. Asserted first,
    so a fixture that drifts fails here rather than making an arm below pass
    for the wrong reason."""
    with tempfile.TemporaryDirectory() as wd:
        pcb = parse_kicad_pcb(_board(os.path.join(wd, 'b.kicad_pcb')))
        chips = groups_mod.chip_refs(pcb)
        assert chips == {'U1', 'IC1'}, sorted(chips)      # CN1 is collinear
        near, beyond, orphans = groups_mod.decap_populations(pcb)
        tethered = ({c for caps in near.values() for c, _d in caps}
                    | {c for c, _i, _d in beyond})
        assert tethered == {'C1', 'C2', 'C3', 'C5'}, sorted(tethered)
        assert orphans == ['C4'], orphans                 # VBULK: only C4 + L1
    print("  PASS: chips {U1, IC1} (CN1 is a collinear row), tethered "
          "{C1, C2, C3, C5}, orphan {C4}")


def test_the_pin_stage_runs_and_seats_ONE_cap_PER_PIN():
    """T1. U1's two VCC pins are 4mm apart, so a pin-first stage seats C1 and
    C2 at DIFFERENT pins -- a cap-first greedy or a single cluster would put
    both at one."""
    with tempfile.TemporaryDirectory() as wd:
        zones = [{"name": "u", "refs": ["U1"], "zone": [24, 14, 36, 26]}]
        on, poses, pcb = _seed(wd, {'max_distance_mm': 3.0}, zones)
        seats = _notes(on, 'decap for U1')
        # TWO seats for TWO pins, from the three VCC caps -- which two
        # is the stage's ordering to decide and not this arm's claim.
        assert len(seats) == 2, on['notes']
        got = {n.split(':')[0] for n in seats}
        assert got <= {'C1', 'C2', 'C5'}, seats
        # DIFFERENT pins, not the same one twice. Asserted as "each cap is
        # nearest a DIFFERENT VCC pin" rather than "each cap is within N mm of
        # one": U1's courtyard is 6x6 and the caps cannot overlap it, so the
        # seat lands at the nearest legal pose OUTSIDE it and an absolute
        # distance bound would be a statement about the courtyard, not about
        # the stage. Measured here: C1 (26.0, 18.0), C2 (34.0, 18.0), U1
        # (30.0, 20.0) -- pins at (28, 18) and (32, 18), one cap each.
        assert len({poses[r] for r in got}) == 2, poses
        pins = [(poses['U1'][0] + dx, poses['U1'][1] - 2) for dx in (-2, 2)]
        picked = {}
        for ref in sorted(got):
            cx, cy = poses[ref]
            picked[ref] = min(range(2), key=lambda i: (
                (cx - pins[i][0]) ** 2 + (cy - pins[i][1]) ** 2))
        assert len(set(picked.values())) == 2, (picked, poses)

        # ANTI-TAUTOLOGY: with no decap key the stage must not run at all, so
        # "the stage did something" cannot be satisfied by stage 3 doing it.
        off, off_poses, _p = _seed(wd, None, zones)
        assert not _notes(off, 'decap for'), off['notes']
        moved = [r for r in got if off_poses[r] != poses[r]]
        assert moved, "both arms produced the same poses -- the key is inert"
    print(f"  PASS: {sorted(got)} are seated at U1's two DISTINCT VCC pins "
          f"out of three candidates, and none is seated at all when the key "
          f"is absent")


def test_a_cap_with_no_rail_carrying_chip_is_NOT_evicted_from_its_zone():
    """T2, the #792 fix. C4's rail (VBULK) is touched only by C4 and L1, so no
    chip carries it and stage 2.5 can never seat it. Before the narrowing it
    was still stripped from zone packing and fell to the centroid stage."""
    with tempfile.TemporaryDirectory() as wd:
        zones = [{"name": "u", "refs": ["U1"], "zone": [24, 14, 36, 26]},
                 {"name": "bulk", "refs": ["C4"], "zone": [44, 4, 56, 16],
                  "tolerance_mm": 0.5}]
        res, poses, pcb = _seed(wd, {'max_distance_mm': 3.0}, zones)
        assert not _notes(res, 'C4: decap for'), res['notes']
        # THE NARROWING ITSELF, not just its outcome. Reverting the
        # scope to the old syntactic test and re-running this file
        # killed NOTHING: C4 still ends up in its zone, because the
        # 2.6 put-back catches it either way. The two changes are
        # distinguishable only by WHICH stage seats it -- narrowed, C4
        # is never evicted and stage 2 packs it with its block, in
        # priority order; un-narrowed it is evicted, declined by the
        # pin stage, and appended afterwards. So assert the absence of
        # a put-back note, which is the observable difference.
        assert not _notes(res, 'C4: zone-packed into'), (
            'C4 reached its zone via the put-back, so it WAS evicted: '
            'the scope was not narrowed', res['notes'])
        assert not _notes(res, 'C4: the pin stage declined'), res['notes']
        x, y = poses['C4']
        # Inside its declared zone, asserted through the grader's own
        # predicate rather than against a coordinate, so the arm survives a
        # change to the packing order.
        assert 44.0 - 0.5 <= x <= 56.0 + 0.5, poses['C4']
        assert 4.0 - 0.5 <= y <= 16.0 + 0.5, poses['C4']
        assert 'C4' not in (res.get('unseated') or [])
    print(f"  PASS: C4 (rail carried by no chip) is zone-packed at "
          f"{poses['C4']}, inside its declared zone, and never claimed by the "
          f"pin stage")


def test_a_cap_the_pin_stage_DECLINES_is_put_back_and_named():
    """T3. Give U1 a zone so tight that `_try_place` refuses the pin seat, and
    the cap must still land in its own zone with a note saying why -- not at
    the board centre, and not unseated.

    Asserts the note COUNT, because a version that seats silently satisfies an
    "all seated" assertion, and that silent path is the very bug #792 fixes in
    pass 2."""
    with tempfile.TemporaryDirectory() as wd:
        # C1 and C2 both declared into a zone far from U1's pins: the pin stage
        # tries the zone-constrained seat first, then the unconstrained one.
        zones = [{"name": "u", "refs": ["U1"], "zone": [24, 14, 36, 26]},
                 {"name": "caps", "refs": ["C1", "C2", "C5"],
                  "zone": [2, 30, 14, 38], "tolerance_mm": 0.5}]
        res, poses, _p = _seed(wd, {'max_distance_mm': 3.0}, zones)
        assert not (res.get('unseated') or []), res['unseated']
        # Every scope cap ends up somewhere, and every one that the pin stage
        # did not claim carries a note saying so.
        claimed = {n.split(':')[0] for n in _notes(res, 'decap for')}
        put_back = {n.split(':')[0] for n in _notes(res, 'zone-packed into')}
        scope = {'C1', 'C2', 'C5'}
        assert len(claimed) == 2, (claimed, res['notes'])
        # THE PUT-BACK, NAMED. Asserted as an EQUALITY against the caps
        # the pin stage did not take, never as membership in a union of
        # note kinds: the union version accepted the put-back being
        # DELETED, because the pin stage's own truncation note supplied
        # the coverage. A battery row that had been KILLED started
        # surviving and this arm stayed green -- found in review, and it
        # is why the two notes now have distinct wording.
        assert put_back == scope - claimed, (put_back, claimed)
        # ...and the cap really is INSIDE its declared zone, not merely
        # mentioned. Without the put-back it lands near the board centre.
        for ref in put_back:
            x, y = poses[ref]
            assert 2.0 - 0.5 <= x <= 14.0 + 0.5, (ref, poses[ref])
            assert 30.0 - 0.5 <= y <= 38.0 + 0.5, (ref, poses[ref])
        for ref in sorted(scope):
            assert ref in poses, ref
        declined = set()
    print(f"  PASS: claimed {sorted(claimed)}, put back {sorted(put_back)}, "
          f"declined {sorted(declined)} -- every scope cap is accounted for by "
          f"a note, none unseated")


def test_a_declined_cap_with_NO_zone_is_still_named_by_the_pin_stage():
    """The pass-2 note, on the only path where it is the ONLY report.

    A battery row suppressing that note SURVIVED: in T3 the declined cap has a
    declared zone, so the 2.6 put-back catches it and emits its own note, and
    the arm's count is satisfied either way. The pass-2 note is therefore
    untested by T3 -- it is masked by the very fix that follows it.

    Here the three VCC caps are declared into NO zone, so the put-back has
    nothing to put them back INTO and skips them. The cap the pin stage
    passes over then reaches the generic stage, and a pass-2 note is the only
    thing that says so.

    Writing this arm found a SECOND silent path, distinct from the `_seat`
    failure: `zip(clusters, caps_r)` truncates to the shorter list, so a cap
    past the cluster count is never reached by the loop at all and was
    dropped without a word. Both are named now, which is what the
    surrounding comment has always claimed ("caps no pin wanted fall through
    to the generic stage, which reports honestly") and did not deliver.
    """
    with tempfile.TemporaryDirectory() as wd:
        zones = [{"name": "u", "refs": ["U1"], "zone": [24, 14, 36, 26]}]
        res, poses, _p = _seed(wd, {'max_distance_mm': 3.0}, zones)
        claimed = {n.split(':')[0] for n in _notes(res, 'decap for')}
        # The two pin-stage notes are matched SEPARATELY. Unioning them
        # is what let the put-back arm above accept its own deletion.
        truncated = {n.split(':')[0] for n in
                     _notes(res, 'pin cluster left for it')}
        refused = {n.split(':')[0] for n in
                   _notes(res, 'no legal pose at')}
        declined = truncated | refused
        put_back = {n.split(':')[0] for n in _notes(res, 'zone-packed into')}
        scope = {'C1', 'C2', 'C5'}
        # Two pins, three caps: one is left over, and it has no zone to be put
        # back into, so the pin stage's own note must name it.
        assert len(claimed) == 2, (claimed, res['notes'])
        assert not put_back, (put_back, "these caps have no zone; nothing "
                                        "should have been put back")
        left = scope - claimed
        assert len(left) == 1, left
        # Named by the TRUNCATION path specifically -- three caps, two
        # clusters, `zip` reaches two. The `_seat`-refusal note is a
        # different path and no fixture here reaches it (see the
        # battery's recorded survivor).
        assert left == truncated, (left, truncated, res['notes'])
        assert not refused, refused
        # And it is still SEATED, by the generic stage, not lost.
        for ref in scope:
            assert ref in poses, ref
        assert not (res.get('unseated') or []), res['unseated']
    print(f"  PASS: {sorted(left)} is declined by the pin stage with no zone "
          f"to fall back to, is NAMED by pass 2 rather than dropped silently, "
          f"and is still seated")


def test_the_owner_test_is_the_GROUPERS_answer_not_a_ref_prefix():
    """T4, both directions in one arm, which is what stops the fix being
    "delete the U test":

      IC1 is a chip and is NOT U-prefixed -> C3 must be seated at its pin;
      CN1 is a 10-pad collinear ROW carrying VCC -> no cap may be seated at it.
    """
    with tempfile.TemporaryDirectory() as wd:
        zones = [{"name": "u", "refs": ["U1"], "zone": [24, 14, 36, 26]},
                 {"name": "ic", "refs": ["IC1"], "zone": [6, 6, 18, 18]},
                 {"name": "cn", "refs": ["CN1"], "zone": [42, 28, 54, 36]}]
        off, _p, _pc = _seed(wd, {'max_distance_mm': 3.0}, zones)
        on, poses, _pc = _seed(wd, {'max_distance_mm': 3.0}, zones,
                               decap_owner_chips=True)
        # The prefix test cannot see IC1; the grouper's answer can.
        assert not _notes(off, 'C3: decap for IC1'), off['notes']
        assert _notes(on, 'C3: decap for IC1'), on['notes']
        # And NEITHER may seat a cap at the collinear row.
        assert not _notes(off, 'decap for CN1'), off['notes']
        assert not _notes(on, 'decap for CN1'), on['notes']
    print("  PASS: with the grouper's answer C3 is seated at IC1's pin, and "
          "the collinear CN1 row is refused as an owner under BOTH settings")


def test_the_control_arm_is_untouched_when_no_decap_key_is_declared():
    """The change must be scoped to armed intents. An intent with no `decaps`
    key must seed byte-identically to one graded before #792 -- which is what
    the corpus-scale control arm asserts, and this is its unit form."""
    with tempfile.TemporaryDirectory() as wd:
        zones = [{"name": "u", "refs": ["U1"], "zone": [24, 14, 36, 26]}]
        a, pa, _p = _seed(wd, None, zones)
        b, pb, _p = _seed(wd, {}, zones)
        assert pa == pb, [k for k in pa if pa[k] != pb.get(k)]
        assert not _notes(a, 'decap for') and not _notes(b, 'decap for')
        assert not _notes(a, 'zone-packed into')
    print(f"  PASS: absent and empty `decaps` seed identically over "
          f"{len(pa)} part(s), and neither runs the pin stage")


TESTS = [
    test_the_fixture_is_the_board_the_arms_below_assume,
    test_the_pin_stage_runs_and_seats_ONE_cap_PER_PIN,
    test_a_cap_with_no_rail_carrying_chip_is_NOT_evicted_from_its_zone,
    test_a_cap_the_pin_stage_DECLINES_is_put_back_and_named,
    test_a_declined_cap_with_NO_zone_is_still_named_by_the_pin_stage,
    test_the_owner_test_is_the_GROUPERS_answer_not_a_ref_prefix,
    test_the_control_arm_is_untouched_when_no_decap_key_is_declared,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
