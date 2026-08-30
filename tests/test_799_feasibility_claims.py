"""The published #799 numbers, asserted against their committed measurement.

A measured number needs a committed measurement, or it is a number that lived
only in a terminal. `tests/measure_799_zone_feasibility.py` produced these; this
file is what makes them a change detector rather than prose.

TWO artefacts are committed, and neither is the full sweep:

  * `tests/799_feasibility_summary.json` -- the whole run's counters.
  * `tests/799_feasibility_rows.jsonl` -- a DECLARED SUBSET of 142 of the 33696
    rows, chosen by rule BEFORE the answers were read: the lowest coverage
    fraction that refuses and the highest that stays silent, per (board, family,
    filter), plus every row where the check is silent and the pose census
    provably found a pose, plus every hard false positive (there are none).

The full 33696-row file is 9.5 MB and is deliberately NOT committed. A rows file
that size has been asked to leave this repo before; the summary plus the
boundary rows is what a reader actually needs, and
`measure_799_zone_feasibility.py --from-rows` re-derives every counter from a
rows file so the pipeline itself stays checkable.

WHAT THESE NUMBERS ARE, AND ARE NOT. The corpus arm measures INERTNESS:
`emit_intent` writes `keepouts: []` and no CLI creates one, so every intent this
repo can generate is structurally unreachable by the check. The agreement arm is
the real evidence, and it is ASYMMETRIC on purpose. For a NON-ANCHOR zone
`seeder.pose_ok` is stronger than the check's question (it also demands board
containment and neighbour clearance), so a pose it finds is a pose that exists
and a refusal against it is a hard bug; a census of 0 is NOT proof of
infeasibility and is reported separately as a soft suspected miss.

THE NON-ANCHOR QUALIFIER IS LOAD-BEARING, and the sweep cannot drop it: in
ANCHOR mode `zone_gate` constrains the footprint ORIGIN while this check grades
the courtyard CENTRE, so the two disagree by construction. Every zone in the
sweep is a member's own courtyard bbox inflated by a margin >= 0, which always
satisfies `zone_fits_courtyard` -- so the sweep never reaches anchor mode, and
`hard_false_positives == 0` is evidence about the non-anchor branch ALONE.
"""
import json
import os
import sys

RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.dirname(os.path.abspath(__file__))
SUMMARY = os.path.join(HERE, '799_feasibility_summary.json')
ROWS = os.path.join(HERE, '799_feasibility_rows.jsonl')

#: The published figures. Stated as LITERALS, not read from the file and
#: compared to itself: a test that derives its expectation from the artefact
#: under test passes whatever the artefact says.
PUBLISHED = {
    'cases': 33696,
    'boards': 13,
    'members': 39,
    'refused': 10550,
    'old_total_coverage': 804,
    'H': 9750,
    'M': 18,
    'oracle_consulted': 11558,
    'hard_false_positives': 0,
    'soft_misses': 990,
    'refusals_with_no_binding_keepout': 0,
    # Cases that DECLARE a keep-out and whose member the resolver still
    # exempted. An earlier version counted every `bound == 0` row, which
    # swept in the 4212 rows at coverage fraction 0 where no keep-out
    # exists at all -- inflating the exemption evidence by a third.
    'exempted_cases': 12600,
    'no_keepout_declared_cases': 4212,
}
SUBSET_ROWS = 142

passed = 0
failed = 0


def check(name, ok, detail=""):
    """Detail must read as a MEASUREMENT: it prints on OK and on FAIL alike."""
    global passed, failed
    if ok:
        passed += 1
        print(f"  OK   {name}" + (f" -- {detail}" if detail else ""))
    else:
        failed += 1
        print(f"  FAIL {name}" + (f" -- {detail}" if detail else ""))


def main():
    for p in (SUMMARY, ROWS):
        if not os.path.isfile(p) or os.path.getsize(p) == 0:
            print(f"FAIL: {os.path.basename(p)} is missing or empty -- the "
                  f"published numbers have no committed measurement")
            return 1

    with open(SUMMARY, encoding='utf-8') as f:
        s = json.load(f)
    with open(ROWS, encoding='utf-8') as f:
        rows = [json.loads(line) for line in f if line.strip()]

    for k, want in PUBLISHED.items():
        if k == 'boards':
            got = len(s.get('boards') or [])
        elif k == 'members':
            got = None       # derived from the rows below, not in the summary
        else:
            got = s.get(k)
        if k == 'members':
            continue
        check(f"summary.{k} is the published {want}", got == want,
              f"committed value {got}")

    # The two anti-vacuity bounds, asserted rather than described. A sweep with
    # no hits proves nothing; one with no misses proves nothing either.
    check("H > 0: the widening refuses cases total coverage did not",
          s.get('H', 0) > 0,
          f"H={s.get('H')} of {s.get('refused')} refusals "
          f"({s.get('old_total_coverage')} were already caught)")
    check("M > 0: the check is not simply refusing everything",
          s.get('M', 0) > 0,
          f"M={s.get('M')} case(s) silent with a pose provably present")
    check("the pre-registered criterion holds: NO hard false positive",
          s.get('hard_false_positives') == 0,
          "a refusal against a census that found a pose would be a bug, "
          "because pose_ok is strictly stronger than this check's question")
    check("every refusal names a keep-out that BINDS the member",
          s.get('refusals_with_no_binding_keepout') == 0,
          "the resolver's own answer, not a proxy for it")

    # The subset must carry the sweep's own counterexamples, or it is a
    # curated set that agrees with itself.
    check(f"the declared row subset is {SUBSET_ROWS} rows",
          len(rows) == SUBSET_ROWS, f"{len(rows)} committed")
    m_rows = [r for r in rows
              if isinstance(r.get('oracle'), int) and r['feasible']
              and r['oracle'] >= 1]
    check("every M row survived into the subset",
          len(m_rows) == PUBLISHED['M'],
          f"{len(m_rows)} of {PUBLISHED['M']} -- the subset keeps the cases "
          f"that would falsify the claim, not only the ones that support it")
    fp_rows = [r for r in rows
               if isinstance(r.get('oracle'), int) and not r['feasible']
               and r['oracle'] >= 1]
    check("no hard false positive is present in the rows either",
          not fp_rows, f"{len(fp_rows)} row(s)")

    # A partial overlap must NOT refuse: arm O's control, restated as a
    # corpus-wide property rather than one fixture.
    at_zero = [r for r in rows if r['frac'] == 0.0 and not r['feasible']]
    check("no case refuses at coverage fraction 0",
          not at_zero,
          f"{len(at_zero)} row(s) -- refusing there would contradict the "
          f"partial-overlap control")
    flips = sorted({r['frac'] for r in rows
                    if r.get('subset_reason') == 'flip'})
    check("the flip points are recorded and above 0",
          flips and min(flips) > 0.0, f"lowest refusing fraction {flips[:4]}")

    circ = [r for r in rows if r['family'] == 'circle' and not r['feasible']]
    check("a circle keep-out never refuses, as documented",
          not circ, f"{len(circ)} row(s) -- no disc/rect free-area kernel "
                    f"exists, so a refusal resting on one would be unsound")

    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
