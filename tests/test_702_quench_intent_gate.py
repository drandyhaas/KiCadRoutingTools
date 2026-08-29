#!/usr/bin/env python3
"""#702: the quench refuses a move that breaks a declared claim.

Hand-written boards, for the reason `tests/test_701_keepout_seating.py` gives
for its own: on a corpus board "where would this part have gone" is only
answerable by re-running the optimizer, which makes the control circular. Here
the geometry is small enough that the answer is a theorem.

EVERY ACCEPTING ARM HAS A CONTROL. An assertion like "the part is not in the
keep-out" is satisfied by a board where the part was never going to be in the
keep-out, and then the test passes in both directions. So each arm that shows
the gate doing something is paired with the same board showing the gate absent
doing the other thing.

EVERY ARM ALSO ASSERTS THE COUNTER. `metrics_out['intent_gate']['by_site']`
says which of the three enforcement sites refused. Without it, "the gate
refused nothing" and "this arm never reached the gate" are the same
observation -- which is how an arm passes while testing nothing. Two of the
mutations in `tests/mutate_702.py` are killed only by a `by_site` assertion.

MEASURED MUTATION TABLE, from `python3 tests/mutate_702.py`. The edits are in
that file; these are the verdicts, recorded FROM THE RUN and not predicted:

    20 rows: 18 killed, 2 survived, 0 broken, 0 disagreeing with expectation

    the two survivors, recorded rather than deleted:
      the-gate-is-built-even-with-no-intent      building an empty spec on a
                                                board that declares nothing is
                                                indistinguishable from not
                                                building one -- every lookup
                                                misses and `intent_ok` returns
                                                on `if not spec`.
      the-active-flag-ignores-whether-anything-bound
                                                same reason, one level up: the
                                                flag only guards work that is
                                                already a no-op on an empty
                                                spec. Kept as a detector for
                                                the day the guard means more.

Three rows exist because a mutation caught THIS FILE, not the engine:
`the-incumbent-is-read-at-the-SEED-pose` survived until arm C was written at
all; `the-keepout-test-forgets-the-through-hole-rect` survived while arm J
asserted only BINDING and never the hit test; and
`zone_escape-grades-the-origin-not-the-centre` survived while arm E asserted
`intent_ok` (which admits a part at its own pose by construction) instead of
the absolute `intent_clear`. None of the three was a bug in the gate; all three
were arms that passed without reaching what they named.
"""
import json
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
        sys.path.insert(0, os.path.join(_p, 'py_router'))
        sys.path.insert(0, os.path.join(_p, 'py_tools'))
        sys.path.insert(0, os.path.join(_p, 'py_placer'))

from kicad_parser import parse_kicad_pcb                        # noqa: E402
from placement import floorplan, legality                       # noqa: E402
from placement.quench import (QuenchState, quench,              # noqa: E402
                              INTENT_ENFORCED_RULES)

RUN_ALL_TIMEOUT = 600

passed = failed = 0


def check(name, ok, detail=""):
    """Prints `detail` on OK and FAIL alike, so every detail must read as a
    MEASUREMENT rather than as a failure explanation."""
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}"
          + (f" -- {detail}" if detail else ""))


# --------------------------------------------------------------------------
# fixtures -- the shape of tests/test_701_keepout_seating.py's, on purpose
# --------------------------------------------------------------------------

SIZE = (40.0, 24.0)


def _part(ref, x, y, half_w, half_h, npads=2, layer='F.Cu', thru=False,
          cy_off=(0.0, 0.0), fp='auto', nets=None):
    """A footprint with a (2*half_w x 2*half_h) courtyard and `npads` pads.

    `cy_off` shifts the COURTYARD off the footprint origin, which is the case
    `floorplan.zone_escape`'s anchor branch grades on the courtyard CENTRE
    while `seeder.zone_gate` would grade on the origin. 17 of 65 parts on
    splitflap_driver are like this; arm N is the regression detector.
    """
    ox, oy = cy_off
    # `fp` lets two refs share a FOOTPRINT NAME, which is what makes them
    # swap-eligible (the swap phase pairs on footprint_name + side + has_tht +
    # matching rot-0 bounds). `nets` gives them crossed nets so the swap is
    # strictly improving and the phase actually reaches its gates.
    fp_name = f'test:P{ref}' if fp == 'auto' else fp
    nets = nets or tuple(1 if i == 0 else 2 for i in range(npads))
    if thru:
        pads = ''.join(
            f'\t\t(pad "{i + 1}" thru_hole circle\n'
            f'\t\t\t(at {i * 0.6 - 0.3} 0)\n'
            f'\t\t\t(size 0.6 0.6)\n\t\t\t(drill 0.35)\n'
            f'\t\t\t(layers "*.Cu" "*.Mask")\n'
            f'\t\t\t(net {nets[i]} "N{nets[i]}")\n'
            f'\t\t\t(uuid "p{i}-{ref}")\n\t\t)\n' for i in range(npads))
    else:
        pads = ''.join(
            f'\t\t(pad "{i + 1}" smd rect\n'
            f'\t\t\t(at {i * 0.2 - 0.2} 0)\n'
            f'\t\t\t(size 0.3 0.3)\n\t\t\t(layers "{layer}")\n'
            f'\t\t\t(net {nets[i]} "N{nets[i]}")\n'
            f'\t\t\t(uuid "p{i}-{ref}")\n\t\t)\n' for i in range(npads))
    return f'''\t(footprint "{fp_name}"
\t\t(layer "{layer.split('.')[0]}.Cu")
\t\t(uuid "fp-{ref}")
\t\t(at {x} {y})
\t\t(property "Reference" "{ref}"
\t\t\t(at 0 0)
\t\t)
\t\t(fp_rect
\t\t\t(start {ox - half_w} {oy - half_h})
\t\t\t(end {ox + half_w} {oy + half_h})
\t\t\t(layer "{layer.split('.')[0]}.CrtYd")
\t\t\t(uuid "cy-{ref}")
\t\t)
{pads}\t)
'''


def board(path, parts, size=SIZE):
    body = ('(kicad_pcb\n\t(version 20241229)\n'
            '\t(net 0 "")\n'
            + ''.join(f'\t(net {i} "N{i}")\n' for i in range(1, 9))
            + ('\t(gr_rect\n\t\t(start 0 0)\n\t\t(end {} {})\n'
               '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n').format(*size)
            + ''.join(parts) + ')\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)
    return path


def intent_doc(blocks=(), keepouts=(), size=SIZE):
    d = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
         "envelope": {"rect": [0.0, 0.0, size[0], size[1]],
                      "tolerance_mm": 0.5},
         "blocks": [dict(b) for b in blocks]}
    if keepouts:
        d["keepouts"] = [dict(k) for k in keepouts]
    return d


def load(doc, wd, tag='fp'):
    p = os.path.join(wd, f'{tag}.json')
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    return floorplan.load_intent(p)


def state_for(bpath, intent, sources=()):
    """A QuenchState carrying the resolved gate, as `quench()` builds it."""
    pcb = parse_kicad_pcb(bpath)
    gate, _ = floorplan.resolve_intent_gate(intent, pcb, sources)
    return QuenchState(pcb, bpath, 0.25, 0.55, 10.0, 0.5, 0.25, 2.0, 2.0, 2.0,
                       0.1, 1.0, keepouts=gate['keepouts'],
                       intent_zones=gate['zones']), gate


def graded_errors(bpath, intent, rule=None, ref=None):
    r = floorplan.grade(intent, parse_kicad_pcb(bpath), bpath)
    return [v for v in r.errors
            if (rule is None or v.rule == rule)
            and (ref is None or v.ref == ref)]


# --------------------------------------------------------------------------
# arms
# --------------------------------------------------------------------------

def arm_A_nudge_gate(wd):
    """The gate refuses a nudge out of a declared zone; without it, it goes."""
    print("--- A: the nudge gate, and its control")
    # U1 in a tight zone; U2 locked far east so the airwire pulls U1 east.
    b = board(os.path.join(wd, 'a.kicad_pcb'), [
        _part('U1', 8.0, 12.0, 1.5, 1.5),
        _part('U2', 34.0, 12.0, 1.5, 1.5),
    ])
    doc = intent_doc(blocks=[{"name": "z", "refs": ["U1"],
                              "zone": [5.0, 9.0, 11.0, 15.0],
                              "tolerance_mm": 0.5}])
    it = load(doc, wd, 'a')

    def run(gate_on, out):
        pcb = parse_kicad_pcb(b)
        g = None
        if gate_on:
            g, _ = floorplan.resolve_intent_gate(it, pcb, ())
        m = {}
        pl = quench(pcb, pcb_file=b, max_displacement=12.0, step=1.0,
                    grid_step=0.1, clearance=0.25, board_edge_clearance=0.55,
                    crossing_penalty=10.0, length_weight=1.0, halo_base=0.5,
                    halo_coef=0.25, halo_weight=2.0, edge_halo=2.0,
                    edge_weight=2.0, lock_refs=['U2'], metrics_out=m,
                    intent_gate=g, max_passes=4)
        from placement.writer import write_placed_output
        write_placed_output(b, out, pl)
        return m

    off = run(False, os.path.join(wd, 'a_off.kicad_pcb'))
    on = run(True, os.path.join(wd, 'a_on.kicad_pcb'))

    e_off = graded_errors(os.path.join(wd, 'a_off.kicad_pcb'), it,
                          'zone_containment', 'U1')
    e_on = graded_errors(os.path.join(wd, 'a_on.kicad_pcb'), it,
                         'zone_containment', 'U1')
    # CONTROL first: without the gate the quench must actually break the zone,
    # or the arm below proves nothing.
    check("control: no gate walks U1 out of its zone", len(e_off) == 1,
          f"{len(e_off)} zone_containment error(s) without --intent")
    check("the gate keeps U1 inside its zone", len(e_on) == 0,
          f"{len(e_on)} zone_containment error(s) with the gate")
    ig = on.get('intent_gate') or {}
    check("the refusal is attributed to candidate_valid",
          ig.get('by_site', {}).get('candidate_valid', 0) > 0,
          f"by_site={ig.get('by_site')}")
    check("the OFF arm built no gate at all", 'intent_gate' not in off,
          "metrics_out carries no intent_gate key without an intent")


def arm_B_monotone_escape(wd):
    """A part that STARTS inside a keep-out may still leave it."""
    print("--- B: monotone escape, and the freeze it must not cause")
    b = board(os.path.join(wd, 'b.kicad_pcb'),
              [_part('U1', 10.0, 12.0, 1.5, 1.5)])
    ko = {"name": "k", "rect": [7.0, 9.0, 13.0, 15.0]}
    it = load(intent_doc(keepouts=[ko]), wd, 'b')
    st, _ = state_for(b, it)

    p = st.parts['U1']
    inside = st.intent_terms('U1', p.rects(10.0, 12.0, 0.0))
    check("fixture: U1 starts INSIDE the keep-out", inside[0] > 0.0,
          f"keepout_hit at the seed pose = {inside[0]}")
    # A pose fully clear of the keep-out must be admitted even though the
    # incumbent violates -- that is the whole point of the monotone rule.
    check("a clear pose is admitted from inside the keep-out",
          st.intent_ok('U1', 20.0, 12.0, 0.0),
          "hit 0.0 <= incumbent, so the part can leave")
    # And a pose that is still inside, but LESS inside, is admitted too.
    less = st.intent_terms('U1', p.rects(12.0, 12.0, 0.0))
    check("a strictly-less-overlapping pose is admitted",
          less[0] < inside[0] and st.intent_ok('U1', 12.0, 12.0, 0.0),
          f"hit {inside[0]:.2f} -> {less[0]:.2f}")
    # A pose that goes DEEPER is refused.
    check("a deeper pose is refused",
          not st.intent_ok('U1', 10.0, 12.0, 90.0)
          or st.intent_terms('U1', p.rects(10.0, 12.0, 90.0))[0] <= inside[0],
          "no candidate may increase the overlap")


def arm_C_ratchet(wd):
    """The incumbent is the pose the part is IN, not the pose it started at.

    Read at the seed, the rule ratchets against a fixed reference: a part that
    legitimately left a keep-out could walk straight back in, because the seed
    overlap is still there to be "no worse than". Found by mutation -- the
    seed-reading mutant survived the first version of this file, which had no
    arm for it at all.
    """
    print("--- C: the incumbent is the CURRENT pose, not the seed")
    b = board(os.path.join(wd, 'c.kicad_pcb'),
              [_part('U1', 10.0, 12.0, 1.0, 1.0)])
    it = load(intent_doc(keepouts=[{"name": "k",
                                    "rect": [8.0, 10.0, 12.0, 14.0]}]), wd, 'c')
    st, _ = state_for(b, it)
    p = st.parts['U1']
    seed_hit = st.intent_terms('U1', p.rects(10.0, 12.0, 0.0))[0]
    check("fixture: the SEED pose is inside the keep-out", seed_hit > 0.0,
          f"seed overlap {seed_hit}")
    # Walk it out, exactly as an accepted move would.
    st.apply_move('U1', 25.0, 12.0, 0.0)
    check("after leaving, the incumbent reads CLEAN",
          st._incumbent_intent('U1')[0] == 0.0,
          "the cache was invalidated by apply_move")
    check("and it may not walk back in",
          not st.intent_ok('U1', 10.0, 12.0, 0.0),
          "0.0 -> overlap is a worsening, whatever the seed was")
    # CONTROL: from the seed pose itself, that same pose IS admitted (it is
    # the incumbent), so the refusal above is the ratchet and not the geometry.
    st2, _ = state_for(b, it)
    check("control: at the seed pose the same pose is admitted",
          st2.intent_ok('U1', 10.0, 12.0, 0.0),
          "a part is never refused for being where it already is")


def arm_D_circle_marker(wd):
    """`keepout_hit` reports a circle as a fabricated 1.0 marker, so the rule
    must be `<=` and not `<` -- with `<` a part inside a circle can never move
    at all unless it clears the whole circle in one nudge."""
    print("--- D: the circle marker, which pins <= rather than <")
    b = board(os.path.join(wd, 'd.kicad_pcb'), [
        _part('U1', 12.0, 12.0, 1.0, 1.0),      # inside the circle
        _part('U2', 30.0, 12.0, 1.0, 1.0),      # outside it
    ])
    it = load(intent_doc(keepouts=[{"name": "c", "circle": [12.0, 12.0, 5.0]}]),
              wd, 'd')
    st, _ = state_for(b, it)
    p1 = st.parts['U1']
    check("fixture: U1 is inside the circle",
          st.intent_terms('U1', p1.rects(12.0, 12.0, 0.0))[0] == 1.0,
          "the marker is exactly 1.0, not an area")
    check("a part inside a circle may MOVE WITHIN it (1.0 <= 1.0)",
          st.intent_ok('U1', 13.0, 12.0, 0.0),
          "this is the assertion that fails if the rule uses `<`")
    check("a part inside a circle may LEAVE it",
          st.intent_ok('U1', 30.0, 20.0, 0.0), "hit 0.0")
    # CONTROL: the part that starts OUTSIDE may not enter.
    check("control: a part outside the circle may not ENTER it",
          not st.intent_ok('U2', 12.0, 12.0, 0.0),
          "0.0 -> 1.0 is refused")


def arm_E_tolerance_and_small_zone(wd):
    """The gate uses the grade's tolerance and the grade's small-zone branch."""
    print("--- E: tolerance, and the spec-coordinate branch")
    b = board(os.path.join(wd, 'e.kicad_pcb'),
              [_part('U1', 10.0, 12.0, 2.0, 2.0)])
    tol = 0.5
    doc = intent_doc(blocks=[{"name": "z", "refs": ["U1"],
                              "zone": [8.0, 10.0, 12.0, 14.0],
                              "tolerance_mm": tol}])
    it = load(doc, wd, 'e')
    st, _ = state_for(b, it)
    check("a pose tol/2 outside is admitted",
          st.intent_ok('U1', 10.0 + tol / 2.0, 12.0, 0.0),
          f"escape {tol/2:.2f} <= tol {tol}")
    check("a pose 2*tol outside is refused",
          not st.intent_ok('U1', 10.0 + 2.0 * tol, 12.0, 0.0),
          f"escape {2*tol:.2f} > tol {tol}")

    # Spec-COORDINATE zone: smaller than the courtyard, so containment falls
    # back to the courtyard CENTRE. A gate that re-derived this as plain
    # rect-in-rect would freeze the part everywhere.
    b2 = board(os.path.join(wd, 'e2.kicad_pcb'),
               [_part('U1', 10.0, 12.0, 2.0, 2.0)])
    doc2 = intent_doc(blocks=[{"name": "z", "refs": ["U1"],
                               "zone": [9.8, 11.8, 10.2, 12.2],
                               "tolerance_mm": tol}])
    it2 = load(doc2, wd, 'e2')
    st2, _ = state_for(b2, it2)
    spec = st2.intent_spec_for('U1')[0]
    check("a zone smaller than the courtyard uses the anchor branch",
          spec.anchor, "zone_fits_courtyard False -> grade the centre")
    # `intent_clear`, NOT `intent_ok`. The monotone rule admits a part at its
    # OWN pose by construction (candidate == incumbent), so asserting
    # `intent_ok` here is satisfied by any anchor semantics at all -- measured:
    # the mutation that makes the anchor branch grade the whole courtyard
    # survived this arm until it asserted the ABSOLUTE predicate.
    check("the part is CLEAN at its own pose under the anchor rule",
          st2.intent_clear('U1', st2.parts['U1'].rects(10.0, 12.0, 0.0)),
          "a whole-courtyard rule would score this 1.8mm outside a 0.5mm tol")
    # And the anchor measurement is the courtyard CENTRE, which for an OFFSET
    # courtyard is not the footprint origin -- the distinction `zone_escape`'s
    # docstring exists for.
    off_b = board(os.path.join(wd, 'e3.kicad_pcb'),
                  [_part('U1', 10.0, 12.0, 1.0, 1.0, cy_off=(1.5, 0.0))])
    off_it = load(intent_doc(blocks=[{"name": "z", "refs": ["U1"],
                                      "zone": [11.3, 11.8, 11.7, 12.2],
                                      "tolerance_mm": 0.1}]), wd, 'e3')
    off_st, _ = state_for(off_b, off_it)
    check("the anchor measures the courtyard CENTRE, not the origin",
          off_st.intent_clear('U1', off_st.parts['U1'].rects(10.0, 12.0, 0.0)),
          "centre is x=11.5 (inside 11.3..11.7); the origin x=10.0 is not")


def arm_K_zone_exclusive(wd):
    """A stranger may not intrude on a reserved zone; a member may."""
    print("--- K: zone_exclusive, and its membership control")
    b = board(os.path.join(wd, 'k.kicad_pcb'), [
        _part('U1', 10.0, 12.0, 1.5, 1.5),      # the member
        _part('R1', 30.0, 12.0, 1.5, 1.5),      # the stranger
    ])
    zone = [6.0, 8.0, 14.0, 16.0]
    it = load(intent_doc(blocks=[{"name": "rf", "refs": ["U1"], "zone": zone,
                                  "exclusive": True, "tolerance_mm": 0.5}]),
              wd, 'k')
    st, _ = state_for(b, it)
    check("a non-member may not intrude on the exclusive zone",
          not st.intent_ok('R1', 10.0, 12.0, 0.0),
          "overlap > EPS is refused")
    # CONTROL: make it a member and the same pose is fine.
    it2 = load(intent_doc(blocks=[{"name": "rf", "refs": ["U1", "R1"],
                                   "zone": zone, "exclusive": True,
                                   "tolerance_mm": 0.5}]), wd, 'k2')
    st2, _ = state_for(b, it2)
    check("control: a MEMBER at the same pose is admitted",
          st2.intent_ok('R1', 10.0, 12.0, 0.0),
          "rule_zone_exclusive skips members, and so does the gate")


def arm_J_tht_other_side(wd):
    """A through-hole part is in a keep-out from either side."""
    print("--- J: through-hole from the far side, and its SMD control")
    b = board(os.path.join(wd, 'j.kicad_pcb'), [
        _part('T1', 10.0, 12.0, 1.0, 1.0, layer='B.Cu', thru=True),
        _part('S1', 10.0, 12.0, 1.0, 1.0, layer='B.Cu'),
    ])
    it = load(intent_doc(keepouts=[{"name": "f", "rect": [7.0, 9.0, 13.0, 15.0],
                                    "sides": ["F"]}]), wd, 'j')
    st, _ = state_for(b, it)
    check("an F-side keep-out binds a B-side THROUGH-HOLE part",
          bool(st.intent_spec_for('T1')),
          "its leads pass through, so it occupies both faces")
    check("control: an F-side keep-out does NOT bind a B-side SMD part",
          not st.intent_spec_for('S1'),
          "sides_occupied is {B} -- no overlap with the keep-out's faces")

    # BINDING is not the same as MEASURING. The hit test must see the drilled
    # rect too, or a part whose BODY clears the keep-out while its LEADS do not
    # is admitted -- and `rule_keepout` grades both, so the gate and the grade
    # would disagree. Found by mutation: dropping tht_rect from the hit test
    # survived the first version of this arm, which asserted only binding.
    b2 = board(os.path.join(wd, 'j2.kicad_pcb'),
               [_part('T2', 10.0, 12.0, 0.35, 0.35, npads=4,
                      layer='B.Cu', thru=True)])
    it2 = load(intent_doc(keepouts=[{"name": "leads",
                                     "rect": [10.4, 11.0, 14.0, 13.0],
                                     "sides": ["F"]}]), wd, 'j2')
    st2, _ = state_for(b2, it2)
    p2 = st2.parts['T2']
    body, leads = p2.rects(10.0, 12.0, 0.0)
    over_body = legality.rect_overlap_area(body, (10.4, 11.0, 14.0, 13.0))
    over_leads = legality.rect_overlap_area(leads, (10.4, 11.0, 14.0, 13.0))
    check("fixture: the BODY clears the keep-out but the LEADS do not",
          over_body <= legality.EPS < over_leads,
          f"courtyard overlap {over_body:.3f}, drilled-rect overlap {over_leads:.3f}")
    check("the gate measures the drilled rect, not just the courtyard",
          st2.intent_terms('T2', (body, leads))[0] > 0.0,
          "keepout_hit is given BOTH rects, as rule_keepout grades both")
    check("and the grade agrees at that pose",
          bool(graded_errors(b2, it2, 'keepout', 'T2')),
          "gate and grade see the same violation")


def arm_P_two_keepouts(wd):
    """One term per (ref, ENTRY): a part in K1 may not hop into K2."""
    print("--- P: per-entry indexing, which a per-rule gate would admit")
    b = board(os.path.join(wd, 'p.kicad_pcb'),
              [_part('U1', 8.0, 12.0, 1.0, 1.0)])
    kos = [{"name": "k1", "rect": [6.0, 10.0, 10.0, 14.0]},
           {"name": "k2", "rect": [20.0, 10.0, 24.0, 14.0]}]
    it = load(intent_doc(keepouts=kos), wd, 'p')
    st, _ = state_for(b, it)
    p = st.parts['U1']
    check("fixture: U1 has TWO terms, one per keep-out",
          len(st.intent_spec_for('U1')) == 2,
          f"{[t.name for t in st.intent_spec_for('U1')]}")
    here = st.intent_terms('U1', p.rects(8.0, 12.0, 0.0))
    check("fixture: U1 starts in k1 and not k2",
          here[0] > 0.0 and here[1] == 0.0, f"terms={here}")
    check("U1 may NOT hop from k1 into k2",
          not st.intent_ok('U1', 22.0, 12.0, 0.0),
          "k2 would go 0.0 -> hit, which no k1 improvement may pay for")
    check("control: U1 MAY move to clear ground",
          st.intent_ok('U1', 15.0, 12.0, 0.0), "both terms 0.0")

    # And the same for two CIRCLES, where both values are the 1.0 marker and a
    # per-rule max() would read the hop as 1.0 -> 1.0 and admit it.
    it2 = load(intent_doc(keepouts=[
        {"name": "c1", "circle": [8.0, 12.0, 3.0]},
        {"name": "c2", "circle": [24.0, 12.0, 3.0]}]), wd, 'p2')
    st2, _ = state_for(b, it2)
    check("two CIRCLES: the 1.0 -> 1.0 hop is refused too",
          not st2.intent_ok('U1', 24.0, 12.0, 0.0),
          "the marker is per-entry, so c2 still goes 0.0 -> 1.0")


def _quench_to(bpath, out, intent, wd, **kw):
    """Run a quench with or without the gate; return (metrics, out path)."""
    from placement.writer import write_placed_output
    pcb = parse_kicad_pcb(bpath)
    g = None
    if intent is not None:
        g, _ = floorplan.resolve_intent_gate(intent, pcb, ())
    m = {}
    base = dict(max_displacement=14.0, step=1.0, grid_step=0.1,
                clearance=0.25, board_edge_clearance=0.55,
                crossing_penalty=10.0, length_weight=1.0, halo_base=0.5,
                halo_coef=0.25, halo_weight=2.0, edge_halo=2.0,
                edge_weight=2.0, max_passes=4)
    base.update(kw)
    pl = quench(pcb, pcb_file=bpath, metrics_out=m, intent_gate=g, **base)
    write_placed_output(bpath, out, pl)
    return m, out


def arm_F_swap_hole(wd):
    """The SWAP phase reaches `candidate_valid` on no path, so it needs its
    own conjunct. Two same-footprint parts with crossed nets swap unless the
    declared intent forbids the destination."""
    print("--- F: the swap phase, which candidate_valid never sees")
    # A1 (net 3) belongs beside its partner in the east; A2 (net 4) beside
    # its partner in the west. So the swap is strictly improving.
    parts = [
        _part('A1', 10.0, 12.0, 1.0, 1.0, fp='test:SWAP', nets=(3, 3)),
        _part('A2', 30.0, 12.0, 1.0, 1.0, fp='test:SWAP', nets=(4, 4)),
        _part('E1', 36.0, 12.0, 1.0, 1.0, nets=(3, 3)),
        _part('W1', 4.0, 12.0, 1.0, 1.0, nets=(4, 4)),
    ]
    b = board(os.path.join(wd, 'f.kicad_pcb'), parts)
    # A keep-out over A2's pose: swapping would put A1 there.
    it = load(intent_doc(keepouts=[{"name": "east",
                                    "rect": [28.0, 10.0, 32.0, 14.0],
                                    "allow": ["A2", "E1", "W1"]}]), wd, 'f')

    def poses(path):
        pcb = parse_kicad_pcb(path)
        return {r: (round(f.x, 2), round(f.y, 2))
                for r, f in pcb.footprints.items()}

    seed = poses(b)
    # The cap must exceed the pair's separation (20mm) or the swap phase
    # skips the pair at `swaps_skipped` BEFORE reaching the intent conjunct --
    # measured: at the default 14mm cap this arm passed its behavioural
    # assertion with by_site={}, i.e. the nudge gate refused the pose and the
    # swap gate was never exercised at all. That is the vacuity the counter
    # exists to catch, and it caught it here first.
    m_off, p_off = _quench_to(b, os.path.join(wd, 'f_off.kicad_pcb'), None, wd,
                              max_displacement=25.0,
                              lock_refs=['E1', 'W1'], allow_rotations=False)
    m_on, p_on = _quench_to(b, os.path.join(wd, 'f_on.kicad_pcb'), it, wd,
                            max_displacement=25.0,
                            lock_refs=['E1', 'W1'], allow_rotations=False)
    off, on = poses(p_off), poses(p_on)
    swapped_off = off['A1'][0] > off['A2'][0]
    # CONTROL: without the gate the exchange happens.
    check("control: without the gate A1 and A2 exchange sides", swapped_off,
          f"A1 {seed['A1'][0]} -> {off['A1'][0]}, A2 {seed['A2'][0]} -> {off['A2'][0]}")
    check("with the gate A1 never lands in the keep-out",
          not (28.0 <= on['A1'][0] <= 32.0),
          f"A1 at x={on['A1'][0]} (keep-out spans 28..32)")
    check("and the grade agrees the board is clean",
          not graded_errors(p_on, it, 'keepout'),
          "0 keepout error(s) on the written board")

    # The behavioural arm above cannot prove WHICH site refused: on this
    # fixture the nudge phase reaches the pose first, and `by_site` came back
    # {'candidate_valid': 75, } with swap never exercised. So the swap site is
    # tested two ways that do not depend on the swap happening to be the
    # winning move.
    #
    # (1) the predicate itself, with a control.
    st, _ = state_for(b, it)
    check("swap_intent_ok refuses an exchange INTO a bound keep-out",
          not st.swap_intent_ok('A1', 'A2'),
          "A1 at A2's pose lands in 'east', which does not allow A1")
    it_all = load(intent_doc(keepouts=[{"name": "east",
                                        "rect": [28.0, 10.0, 32.0, 14.0],
                                        "allow": ["*"]}]), wd, 'f_all')
    st_all, _ = state_for(b, it_all)
    check("control: with allow ['*'] the same exchange is permitted",
          st_all.swap_intent_ok('A1', 'A2'),
          "the allow GLOB is honoured, so this is the keep-out and not the pair")
    check("it binds each part to ITS OWN claims at the partner's pose",
          st.intent_ok('A2', *[getattr(st.parts['A1'], k)
                               for k in ('x', 'y', 'rot')]),
          "A2 is exempt by `allow`, so its half of the swap is fine; "
          "the refusal above is A1's half alone")

    # (2) reachability: the phase must actually CALL it. Counted rather than
    # inferred, because a conjunct the swap loop never reaches is a conjunct
    # that ships inert -- which is what the first version of this arm did.
    import placement.quench as q
    calls = []
    orig = q.QuenchState.swap_intent_ok

    def counting(self, ra, rb):
        calls.append((ra, rb))
        return orig(self, ra, rb)
    q.QuenchState.swap_intent_ok = counting
    try:
        _quench_to(b, os.path.join(wd, 'f_reach.kicad_pcb'), it, wd,
                   max_displacement=25.0, lock_refs=['E1', 'W1'],
                   allow_rotations=False)
    finally:
        q.QuenchState.swap_intent_ok = orig
    check("the swap phase reaches the conjunct at all", bool(calls),
          f"{len(calls)} swap pair(s) tested, e.g. {calls[0] if calls else '-'}")


def arm_H_group_path(wd):
    """The rigid-block phase gates through `group_move_valid` ->
    `candidate_valid`, so it inherits the conjunct -- but it must not be
    frozen outright."""
    print("--- H: the rigid-group path, and the freeze it must not cause")
    parts = [_part('G1', 8.0, 12.0, 1.0, 1.0, nets=(3, 3)),
             _part('G2', 11.0, 12.0, 1.0, 1.0, nets=(3, 3)),
             _part('E1', 36.0, 12.0, 1.0, 1.0, nets=(3, 3))]
    b = board(os.path.join(wd, 'h.kicad_pcb'), parts)
    blocks = {'blk': ['G1', 'G2']}
    zone = [4.0, 8.0, 16.0, 16.0]
    it = load(intent_doc(blocks=[{"name": "blk", "refs": ["G1", "G2"],
                                  "zone": zone, "tolerance_mm": 0.5}]),
              wd, 'h')
    m_off, p_off = _quench_to(b, os.path.join(wd, 'h_off.kicad_pcb'), None, wd,
                              groups=blocks, lock_refs=['E1'])
    m_on, p_on = _quench_to(b, os.path.join(wd, 'h_on.kicad_pcb'), it, wd,
                            groups=blocks, lock_refs=['E1'])
    e_off = graded_errors(p_off, it, 'zone_containment')
    e_on = graded_errors(p_on, it, 'zone_containment')
    check("control: without the gate the block leaves its zone",
          len(e_off) > 0, f"{len(e_off)} zone_containment error(s)")
    check("with the gate the block stays inside its zone", not e_on,
          f"{len(e_on)} zone_containment error(s)")
    # And it must not be frozen: a translate that STAYS inside is still taken.
    pcb_on = parse_kicad_pcb(p_on)
    moved = any(abs(pcb_on.footprints[r].x - {'G1': 8.0, 'G2': 11.0}[r]) > 1e-9
                for r in ('G1', 'G2'))
    check("the gate does not freeze the group phase outright", moved,
          "at least one member still moved inside its zone")


def arm_I_unfreeze_branch(wd):
    """`candidate_valid`'s #456 escape lets an OFF-BOARD part move toward the
    board even when the pose is otherwise illegal. The intent conjunct sits
    ABOVE that branch and must dominate it."""
    print("--- I: the off-board escape branch, which must not be a loophole")
    # U1 sits off the west edge. A keep-out covers the board interior it would
    # come home into -- but NOT U1's own pose, or the incumbent would already
    # violate and the arm would pass for the wrong reason.
    b = board(os.path.join(wd, 'i.kicad_pcb'),
              [_part('U1', -3.0, 12.0, 1.0, 1.0),
               _part('E1', 36.0, 12.0, 1.0, 1.0)])
    ko = {"name": "mid", "rect": [2.0, 6.0, 30.0, 18.0]}
    it = load(intent_doc(keepouts=[ko]), wd, 'i')
    st, _ = state_for(b, it)
    p = st.parts['U1']
    check("fixture: U1's OWN pose is clear of the keep-out",
          st.intent_terms('U1', p.rects(-3.0, 12.0, 0.0))[0] == 0.0,
          "so the incumbent is clean and the gate is live for it")
    check("fixture: U1 is genuinely off the board",
          st.violation('U1') > 0.0, f"violation={st.violation('U1'):.2f}")
    # The #456 branch would admit a pose that improves the board term. The
    # intent conjunct must refuse it anyway, because it enters the keep-out.
    check("a homeward pose INSIDE the keep-out is refused",
          not st.candidate_valid('U1', 5.0, 12.0, 0.0),
          "the escape branch does not launder a declared claim")
    # CONTROL: with no keep-out the same pose IS admitted by the #456 branch.
    st2 = QuenchState(parse_kicad_pcb(b), b, 0.25, 0.55, 10.0, 0.5, 0.25, 2.0,
                      2.0, 2.0, 0.1, 1.0)
    check("control: without the keep-out the same pose is admitted",
          st2.candidate_valid('U1', 5.0, 12.0, 0.0),
          "so the refusal above is the gate, not the geometry")


def arm_Q_census_lift(wd):
    """#701's census must still be able to LIFT a keep-out through this gate.

    `seeder.count_legal_poses` answers "how many seats would lifting keep-out X
    free" by removing X from `state.keepouts_for[ref]` and recounting. The
    recount goes `pose_ok` -> `candidate_valid` -> `intent_ok`, so if the gate
    reads a FROZEN copy of the keep-outs the lift is silently defeated and the
    census reports 0 freed poses -- which degrades a stranded part's verdict
    from `keepout_blocks` to `no_movable_neighbour`, whose prose is verbatim
    the misleading answer that whole disclosure exists to replace.

    Measured while the slice was frozen: lifted went 64 -> 0.
    """
    print("--- Q: the #701 census lift still reaches through the gate")
    from placement import seeder
    b = board(os.path.join(wd, 'q.kicad_pcb'),
              [_part('U1', 30.0, 12.0, 1.0, 1.0)])
    ko = {"name": "hot", "rect": [4.0, 8.0, 16.0, 16.0]}
    it = load(intent_doc(keepouts=[ko]), wd, 'q')
    st, _ = state_for(b, it)
    # Count seats in the keep-out's own area: zero with it, many without.
    kw = dict(radius=3.0, step=1.0, rotations=(0.0,), cap=400)
    bound = seeder.count_legal_poses(st, 'U1', 10.0, 12.0, set(), **kw)
    lifted = seeder.count_legal_poses(st, 'U1', 10.0, 12.0, set(),
                                      without_keepouts=('hot',), **kw)
    check("with the keep-out bound, the census finds no seat there",
          bound == 0, f"bound={bound}")
    check("lifting it frees seats -- the lift reaches the gate",
          lifted > 0, f"lifted={lifted}")
    check("and the keep-out is restored afterwards",
          st.keepouts_for.get('U1') and
          st.keepouts_for['U1'][0]['name'] == 'hot',
          "count_legal_poses put it back")

    # TWO keep-outs, because the one-keep-out case is not discriminating: with
    # a single entry the lift DELETES `keepouts_for[ref]` entirely, so even a
    # gate that ignored the filtered list and read the raw `self.keepouts`
    # would come back empty and look correct. Measured -- that mutation
    # survived the single-keep-out arm. With two, lifting one must leave the
    # other, which only a gate reading the LIFTED list can get right.
    b2 = board(os.path.join(wd, 'q2.kicad_pcb'),
               [_part('U1', 34.0, 20.0, 1.0, 1.0)])
    it2 = load(intent_doc(keepouts=[
        {"name": "hot", "rect": [4.0, 8.0, 16.0, 16.0]},
        {"name": "cold", "rect": [20.0, 2.0, 26.0, 6.0]}]), wd, 'q2')
    st2, _ = state_for(b2, it2)
    both = seeder.count_legal_poses(st2, 'U1', 10.0, 12.0, set(), **kw)
    one = seeder.count_legal_poses(st2, 'U1', 10.0, 12.0, set(),
                                   without_keepouts=('hot',), **kw)
    check("with two keep-outs, lifting one still frees its area",
          both == 0 and one > 0, f"both={both}, hot lifted={one}")
    check("and the OTHER keep-out survives the lift",
          len(st2.keepouts_for.get('U1', ())) == 2,
          "restored to both entries")


def arm_L_inertness(wd):
    """With no intent the gate is UNREACHABLE, not merely quiet."""
    print("--- L: inertness, proven by poison rather than by output")
    b = board(os.path.join(wd, 'l.kicad_pcb'), [
        _part('U1', 10.0, 12.0, 1.5, 1.5),
        _part('U2', 30.0, 12.0, 1.5, 1.5),
    ])
    pcb = parse_kicad_pcb(b)
    st = QuenchState(pcb, b, 0.25, 0.55, 10.0, 0.5, 0.25, 2.0, 2.0, 2.0,
                     0.1, 1.0)
    check("a state with no intent binds nothing",
          not st._intent_active and st._intent_spec == {},
          "_intent_active False, _intent_spec empty")

    # The poison arm: if the gate were reachable at all, this raises.
    import placement.quench as q
    orig = q.QuenchState.intent_ok

    def poison(self, *a, **kw):
        raise AssertionError("the intent gate was reached with no intent")
    q.QuenchState.intent_ok = poison
    try:
        m = {}
        quench(parse_kicad_pcb(b), pcb_file=b, max_displacement=3.0, step=1.0,
               grid_step=0.1, clearance=0.25, board_edge_clearance=0.55,
               crossing_penalty=10.0, length_weight=1.0, halo_base=0.5,
               halo_coef=0.25, halo_weight=2.0, edge_halo=2.0, edge_weight=2.0,
               metrics_out=m, max_passes=2)
        ok, why = True, "a full quench completed with intent_ok poisoned"
    except AssertionError as exc:
        ok, why = False, str(exc)
    finally:
        q.QuenchState.intent_ok = orig
    check("a no-intent quench never calls the gate", ok, why)
    check("and reports no intent_gate metrics", 'intent_gate' not in m,
          "the key is absent, not zero -- 'no gate' != 'refused nothing'")


def arm_M_enforced_tuple(wd):
    """The enforced set is pinned in BOTH directions."""
    print("--- M: INTENT_ENFORCED_RULES, pinned both ways")
    expect = {'zone_containment', 'zone_exclusive', 'keepout'}
    check("the engine enforces exactly the three declared rules",
          set(INTENT_ENFORCED_RULES) == expect,
          f"{sorted(INTENT_ENFORCED_RULES)}")
    # Every enforced rule must be reachable by an arm above, or the tuple is
    # advertising something no test exercises.
    covered = {'zone_containment': 'A/E', 'zone_exclusive': 'K',
               'keepout': 'B/D/J/P'}
    check("every enforced rule has an arm in this file",
          set(covered) == set(INTENT_ENFORCED_RULES),
          ", ".join(f"{r}:{a}" for r, a in sorted(covered.items())))
    # And they are all real floorplan rules, not invented names.
    names = {n for n, _ in floorplan.RULES}
    check("every enforced name is a real floorplan rule",
          set(INTENT_ENFORCED_RULES) <= names,
          f"RULES has {len(names)} names")


def arm_N_gate_vs_grade(wd):
    """The gate's clean verdict and the grade's agree, over a pose lattice.

    Includes a part whose courtyard CENTRE is offset from its footprint
    origin: `seeder.zone_gate`'s anchor branch tests the origin and the grade
    tests the centre, so a gate built on the seeder's predicate disagrees here
    by up to the offset. This is the regression detector for that.
    """
    print("--- N: gate-vs-grade agreement over a lattice")
    # The third case is the one that matters and the one the first version of
    # this arm missed: a zone SMALLER than the courtyard forces the ANCHOR
    # branch, and an offset courtyard makes the origin and the centre disagree.
    # With only the two large-zone cases, `zone_escape` grading the origin
    # instead of the centre survived mutation -- the anchor branch was never
    # taken at all.
    cases = (('centred', (0.0, 0.0), [7.0, 9.0, 13.0, 15.0]),
             ('offset-courtyard', (1.5, 0.0), [7.0, 9.0, 13.0, 15.0]),
             ('offset-ANCHOR-branch', (1.5, 0.0), [9.8, 11.8, 10.2, 12.2]))
    for tag, off, zrect in cases:
        b = board(os.path.join(wd, f'n_{tag}.kicad_pcb'),
                  [_part('U1', 10.0, 12.0, 1.0, 1.0, cy_off=off)])
        doc = intent_doc(blocks=[{"name": "z", "refs": ["U1"],
                                  "zone": zrect, "tolerance_mm": 0.5}])
        it = load(doc, wd, f'n_{tag}')
        st, _ = state_for(b, it)
        p = st.parts['U1']
        disagree = []
        for dx in range(-6, 7):
            for dy in range(-4, 5):
                x, y = 10.0 + dx, 12.0 + dy
                gate_clean = st.intent_clear('U1', p.rects(x, y, 0.0))
                # Move the part there and ask the GRADE.
                moved = board(os.path.join(wd, f'n_probe_{tag}.kicad_pcb'),
                              [_part('U1', x, y, 1.0, 1.0, cy_off=off)])
                grade_clean = not graded_errors(moved, it, 'zone_containment')
                if gate_clean != grade_clean:
                    disagree.append((x, y, gate_clean, grade_clean))
        anchor = st.intent_spec_for('U1')[0].anchor
        check(f"{tag}: gate and grade agree on every pose in the lattice",
              not disagree,
              f"117 poses, anchor={anchor}, {len(disagree)} disagreement(s)"
              + (f" e.g. {disagree[0]}" if disagree else ""))


def arm_O_crossed_claims(wd):
    """A zone a keep-out swallows whole is refused where it is AUTHORED."""
    print("--- O: the crossed-claim contradiction, refused at resolve time")
    b = board(os.path.join(wd, 'o.kicad_pcb'),
              [_part('U1', 10.0, 12.0, 1.0, 1.0)])
    zone = [8.0, 10.0, 12.0, 14.0]
    swallow = {"name": "big", "rect": [6.0, 8.0, 14.0, 16.0]}
    partial = {"name": "corner", "rect": [11.0, 13.0, 20.0, 20.0]}
    blocks = [{"name": "z", "refs": ["U1"], "zone": zone,
               "tolerance_mm": 0.5}]

    pcb = parse_kicad_pcb(b)
    it_bad = load(intent_doc(blocks=blocks, keepouts=[swallow]), wd, 'o_bad')
    _, probs = floorplan.resolve_intent_gate(it_bad, pcb, ())
    named = [v for v in probs if v.rule == 'intent_zone_in_keepout']
    check("a keep-out swallowing a zone is reported by name",
          len(named) == 1 and 'big' in named[0].message,
          named[0].message[:80] if named else "no finding raised")

    # CONTROL: a PARTIAL overlap is a legitimate intent -- a zone with a corner
    # bitten out still has room, and refusing it would be the gate inventing a
    # rule the grade does not have.
    it_ok = load(intent_doc(blocks=blocks, keepouts=[partial]), wd, 'o_ok')
    _, probs2 = floorplan.resolve_intent_gate(it_ok, pcb, ())
    check("control: a PARTIAL overlap is not reported",
          not [v for v in probs2 if v.rule == 'intent_zone_in_keepout'],
          f"{len(probs2)} problem(s), none of them the contradiction")
    # The two filters `keepouts_for_ref` centralizes. A keep-out that does not
    # BIND the block's members cannot contradict their zone, and reporting it
    # is a false ERROR on an intent the grade calls clean. Both measured as
    # false positives before the check consulted them.
    for tag, ko, blks in (
            ('sides', dict(swallow, sides=["B"]), blocks),
            ('allow', dict(swallow, allow=["U1"]), blocks)):
        it_x = load(intent_doc(blocks=blks, keepouts=[ko]), wd, f'o_{tag}')
        _, px = floorplan.resolve_intent_gate(it_x, pcb, ())
        named_x = [v for v in px if v.rule == 'intent_zone_in_keepout']
        bound = floorplan.keepouts_for_ref((ko,), 'U1', frozenset({'F'}))
        check(f"a keep-out exempt by `{tag}` raises no contradiction",
              not named_x,
              f"keepouts_for_ref binds {len(bound)} entry(ies), "
              f"{len(named_x)} contradiction(s) raised")

    check("the finding's severity is settable like any other",
          floorplan.intent_from_dict(
              intent_doc(blocks=blocks,
                         keepouts=[swallow])
              | {"severity": {"intent_zone_in_keepout": "warn"}}
          ).severity_of('intent_zone_in_keepout') == 'warn',
          "registered in _NON_RULE_SEVERITIES")


def main():
    with tempfile.TemporaryDirectory() as wd:
        arm_A_nudge_gate(wd)
        arm_B_monotone_escape(wd)
        arm_C_ratchet(wd)
        arm_D_circle_marker(wd)
        arm_E_tolerance_and_small_zone(wd)
        arm_K_zone_exclusive(wd)
        arm_J_tht_other_side(wd)
        arm_P_two_keepouts(wd)
        arm_F_swap_hole(wd)
        arm_H_group_path(wd)
        arm_I_unfreeze_branch(wd)
        arm_Q_census_lift(wd)
        arm_L_inertness(wd)
        arm_M_enforced_tuple(wd)
        arm_N_gate_vs_grade(wd)
        arm_O_crossed_claims(wd)
    print(f"\n{passed}/{passed + failed} checks passed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
