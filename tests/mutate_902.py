#!/usr/bin/env python3
"""Mutation battery for #902 / #895.

Green tests are not evidence of coverage. Every row below breaks one thing the
test files CLAIM to hold down; a row that survives is a hole in the tests, and
a row recorded as an expected survivor is a finding rather than a convenience.

    python3 tests/mutate_902.py            # every row
    python3 tests/mutate_902.py --list
    python3 tests/mutate_902.py --row min-becomes-max

A row is KILLED by a FAILURE or an ERROR. An anchor that does not match EXACTLY
ONCE is reported BROKEN, never skipped: a mutation that silently edited nothing
would be recorded as a surviving row, which is the opposite of what it means.

Refuses to start on a dirty target tree, because it restores the ORIGINAL text
from disk and would write committed text over uncommitted work.

THE MEASURED TABLE GOES IN THE HEADER OF THE TEST FILE IT DEFENDS, FROM THE
RUN -- never predicted here and never edited afterwards to match.

Every row here has a scar. Nine of them are branches that ALREADY survived a
battery once: five found by the verifier of the rule (the `min` that is the
stated invariant, both abstentions, the partial-pad miss, the courtyard-vs-body
read) and four by the verifier of the close-out gate (the bool check, longest-
match resolution, the `+ ':'` suffix guard, the whole DRIFTED arm). They are
recorded here so the next change has to get past them rather than past a
memory.
"""
import argparse
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

KILLED, SURVIVED, BROKEN = 'KILLED', 'SURVIVED', 'BROKEN'

TARGETS = {
    'fp': os.path.join(REPO, 'py_placer', 'placement', 'floorplan.py'),
    'db': os.path.join(REPO, 'py_placer', 'placement', 'design_brief.py'),
    'cf': os.path.join(REPO, 'py_tools', 'check_floorplan.py'),
    'bc': os.path.join(REPO, 'py_tools', 'board_context.py'),
}

T902 = 'tests/test_902_proximity.py'
T895 = 'tests/test_895_boundary_criteria.py'
T891 = 'tests/test_891_board_context.py'
TSCH = 'tests/test_549_floorplan_schema.py'

#: (name, target, old, new, tests that must notice, expectation)
ROWS = [
    # ---- the rule's own arithmetic -----------------------------------------
    # THE INVARIANT. Every declared case gives a subject pad exactly one
    # candidate partner, so min and max were indistinguishable until a case
    # gave one pad six partners on one net.
    ('min-becomes-max', 'fp',
     "        if best is None or g < best[0]:",
     "        if best is None or g > best[0]:",
     (T902,), KILLED),
    # The narrowing that makes a claim about the RAIL leg rather than the
    # nearest pad. Dropping it reports a number about a different pin.
    ('net-narrowing-dropped', 'fp',
     "    same = ([q for q in partners if q.net_id == pad.net_id and pad.net_id > 0]\n"
     "            if net_match else [])",
     "    same = []",
     (T902,), KILLED),
    # A pad NUMBER is not unique: a crystal has two ground tabs both named
    # `3`. First-hit makes the answer depend on file order.
    ('pads-named-takes-the-first', 'fp',
     "    return [p for p in (fp_obj.pads or ()) if p.pad_number in want]",
     "    got = [p for p in (fp_obj.pads or ()) if p.pad_number in want]\n"
     "    return got[:1]",
     (T902,), KILLED),
    # A partially-wrong pad list graded CLEAN on the survivors: the guard
    # fired only when EVERY name missed.
    ('partial-pad-miss-ignored', 'fp',
     "            missing = [n for n in names if n not in have]",
     "            missing = [] if have else [n for n in names if n not in have]",
     (T902,), KILLED),
    # Both abstentions could be deleted with the whole suite green, and a
    # claim naming a padless part then yielded NOTHING while `proximity`
    # stayed in `rules_run` -- the vacuous pass `_ARM` was dropped on.
    ('pads-abstention-deleted', 'fp',
     "            ctx.abstain(\n                f\"{akey}.pads\",",
     "            _unused = (\n                f\"{akey}.pads\",",
     (T902,), KILLED),
    ('body-abstention-deleted', 'fp',
     "                ctx.abstain(f\"{akey}.basis\", why)\n                continue",
     "                continue",
     (T902,), KILLED),
    # `body_local` is COURTYARD-first, and #896 says a courtyard is not a
    # body. Reading it under-states every gap by the assembly margin.
    ('body-reads-the-courtyard', 'fp',
     "        rect_local = None if geom is None else (geom.drawn_local\n"
     "                                                or geom.body_local)",
     "        rect_local = None if geom is None else geom.body_local",
     (T902,), KILLED),
    # A missing ref must be a FINDING, not silence: a typo would grade clean,
    # which is `block_unresolved`'s failure one level over.
    ('missing-ref-is-silent', 'fp',
     "        missing = [r for r, f in ((ref, a_fp), (near, b_fp)) if f is None]",
     "        missing = []",
     (T902,), KILLED),
    # `_wants` ends in a bare `return True`, so a rule with no branch runs on
    # every board and lands in `rules_run` having measured nothing.
    ('wants-branch-deleted', 'fp',
     "    if rule == 'proximity':\n        return bool(intent.proximity)",
     "    if rule == 'proximity':\n        return True",
     (T902,), KILLED),

    # ---- the loaders --------------------------------------------------------
    # inf and nan pass both `_number(lo=)` and a `<= 0` guard, and json.load
    # accepts the literals: a declared limit nothing can ever violate.
    ('brief-finite-check-deleted', 'db',
     "            if not math.isfinite(float(limit)):",
     "            if False:",
     (T902,), KILLED),
    ('intent-finite-check-deleted', 'fp',
     "        if not math.isfinite(limit) or limit <= 0.0:",
     "        if limit <= 0.0:",
     (T902,), KILLED),
    # Three spellings of "unpadded" defeated the reverse guard, so a brief
    # could declare 5mm one way and 9mm the other.
    ('reverse-guard-keys-on-none', 'db',
     "            if isinstance(pads, dict) and pads.get(ref_one):\n                continue",
     "            if pads is not None:\n                continue",
     (T902,), KILLED),
    ('intent-reverse-guard-deleted', 'fp',
     "        if not (p.get('pads') or {}).get(ref) and (near, ref) in seen:",
     "        if False:",
     (T902,), KILLED),
    # A reference may contain `~`, so a bare `ref~near` id collides and one of
    # two declared unknowns disappears into a set union.
    ('claim-id-drops-the-row', 'db',
     "    return f\"proximity[{row}:{ref}~{near}]\"",
     "    return f\"proximity[{ref}~{near}]\"",
     (T902,), KILLED),
    # An integer pad number matches nothing, measures nothing, grades clean.
    ('pad-number-type-check-gone', 'db',
     "            if not isinstance(n, str):",
     "            if False:",
     (T902,), KILLED),

    # ---- clause coverage ----------------------------------------------------
    ('coverage-uncovered-reads-graded', 'db',
     "        return ('uncovered', f\"`{rule}` did not run on this grade\", rule)",
     "        return ('graded', '', rule)",
     (T902,), KILLED),
    ('coverage-abstention-ignored', 'db',
     "        if _abstention_is_about(akey, kind, ref, near, intent_doc):",
     "        if False:",
     (T902,), KILLED),
    # Trusting the row index alone charged an abstention to whatever claim sat
    # at that row -- a finding about an innocent clause.
    ('abstention-matches-on-index-only', 'db',
     "    if not akey.startswith(f\"proximity[{row}:{ref}~{near}]\"):\n        return False",
     "    if False:\n        return False",
     (T902,), KILLED),
    # `not_claimed` and `carried` must never block, or declaring honestly is
    # punished and people stop declaring.
    ('unknown-clauses-block', 'db',
     "    out['complete'] = (counts['uncovered'] == 0 and counts['abstained'] == 0\n"
     "                       and counts['drifted'] == 0)",
     "    out['complete'] = (counts['uncovered'] == 0 and counts['abstained'] == 0\n"
     "                       and counts['drifted'] == 0\n"
     "                       and counts['not_claimed'] == 0)",
     (T902,), KILLED),
    # The absence message must name what is actually absent, not fabricate a
    # brief that was never found.
    ('absence-reason-keys-on-coverage', 'cf',
     "                  + (_brief_absence_reason(args, brief) if not brief_fragment",
     "                  + (_brief_absence_reason(args, brief) if not coverage",
     (T902,), KILLED),

    # ---- #895's instrument --------------------------------------------------
    # The span is the WORST shared-net pad distance: a bus is as long as its
    # longest member, and the nearest pair says nothing about whether it fits.
    ('span-takes-the-nearest', 'bc',
     "        if worst is None or near > worst:",
     "        if worst is None or near < worst:",
     (T891, T895), KILLED),
]


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _dirty():
    r = subprocess.run(['git', 'status', '--porcelain'] +
                       sorted(set(TARGETS.values())),
                       cwd=REPO, capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if ln.strip()]


def run_row(row, keep=False):
    name, target, old, new, tests, _expect = row
    path = TARGETS[target]
    with open(path, encoding='utf-8') as fh:
        original = fh.read()
    if original.count(old) != 1:
        return BROKEN, f'anchor matched {original.count(old)} time(s)'
    try:
        with open(path, 'w', encoding='utf-8', newline='\n') as fh:
            fh.write(original.replace(old, new, 1))
        for t in tests:
            r = subprocess.run([sys.executable, os.path.join(REPO, t)],
                               cwd=REPO, capture_output=True, text=True)
            if r.returncode != 0:
                return KILLED, f'{t} exit {r.returncode}'
        return SURVIVED, ''
    finally:
        if not keep:
            with open(path, 'w', encoding='utf-8', newline='\n') as fh:
                fh.write(original)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--row', action='append', default=[])
    a = ap.parse_args()

    if a.list:
        for name, target, _o, _n, tests, expect in ROWS:
            print(f"  {name:38} {target:3} {expect:8} {' '.join(tests)}")
        return 0

    dirty = _dirty()
    if dirty:
        print('REFUSING: the target tree is dirty. This restores the ORIGINAL '
              'text from disk and would write committed text over uncommitted '
              'work.')
        for ln in dirty:
            print(f'  {ln}')
        return 2

    rows = [r for r in ROWS if not a.row or r[0] in a.row]
    if a.row and not rows:
        print(f'no row matches {a.row}')
        return 2
    counts = {KILLED: 0, SURVIVED: 0, BROKEN: 0}
    disagreed = 0
    for row in rows:
        got, detail = run_row(row)
        counts[got] += 1
        flag = 'ok  ' if got == row[5] else 'DISAGREES'
        if got != row[5]:
            disagreed += 1
        print(f'  {row[0]:38} {got:9} {flag} {detail}')
    print(f"\n{len(rows)} rows: {counts[KILLED]} killed, "
          f"{counts[SURVIVED]} survived, {counts[BROKEN]} broken, "
          f"{disagreed} disagreeing with expectation")
    return 1 if (counts[BROKEN] or disagreed) else 0


if __name__ == '__main__':
    sys.exit(main())
