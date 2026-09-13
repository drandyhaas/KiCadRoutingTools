#!/usr/bin/env python3
"""Every BLOCKING component has a lens that can refute a PASS about it (#904).

`converge record --lens` stores a verifier's verdict, and `lens_contradictions`
is the only thing that ever checks one against a number. It looks the lens up in
`LENS_COMPONENTS`, so a lens missing from that table CANNOT be contradicted by
anything -- the check does not fail, it does not run.

Measured (run 25, esp_prog): `spec` was absent from the table, so two `--final`
rows carrying `VERDICT=PASS:lens=spec` were written beside a score reporting
`impedance 1`. The end-to-end verifier returned `FAIL:lens=spec` on that same
clause two hours later and the run's terminal row had to be re-recorded.

The table is now complete, and this file is what keeps it complete. It does NOT
hard-code the component list -- that is the trap it exists to avoid, since a
copied list agrees with the code exactly until board_score grows a tenth
component. It re-derives the names from `board_score.py`'s OWN SOURCE, from the
`parts = {...}` literal whose values `blocking` sums, using `ast` rather than an
import (importing board_score pulls in the routing engine).

`assembly` is the one deliberate exemption, and it is declared here with its
reason rather than silently skipped: assembly is graded at the PLACEMENT
boundaries by the boundary verifier's check 5 (references/verifier-prompts.md,
"Check 5 addendum"), which answers `VERDICT=...:check=<1-5>` -- a grammar
`_LENS_RE` refuses on purpose. The routed-board lenses are 7-9 and none of them
was ever asked to run check_assembly.

Also pinned here, because it is the same claim from the other side: the
docstring formula in board_score.py must list exactly the components it actually
sums. That formula was stale by one term (`assembly`) for as long as the term
existed, which is how "the three lenses cover blocking exactly" looked true.
"""
import ast
import io
import os
import sys

RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
import converge                                               # noqa: E402

BOARD_SCORE = os.path.join(
    ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing', 'scripts',
    'board_score.py')

#: Graded somewhere other than a routed-board lens, with WHERE. Not a bare
#: skip-list: an exemption with no reason is how a real gap gets filed under
#: "known".
EXEMPT = {
    'assembly': ('boundary verifier check 5 (verifier-prompts.md, "Check 5 '
                 'addendum"), whose VERDICT= line carries check=<1-5> and not '
                 'lens=<name>'),
}


def blocking_components():
    """The keys of board_score's `parts` dict, read from its source.

    `blocking = sum(c for c in counts if c)` over `parts.values()`, so the keys
    of that literal ARE the blocking components -- no more and no less.
    """
    tree = ast.parse(io.open(BOARD_SCORE, encoding='utf-8').read())
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(t, ast.Name) and t.id == 'parts'
                   for t in node.targets):
            continue
        if not isinstance(node.value, ast.Dict):
            continue
        keys = [k.value for k in node.value.keys
                if isinstance(k, ast.Constant) and isinstance(k.value, str)]
        if len(keys) == len(node.value.keys):
            found.append(keys)
    assert len(found) == 1, (
        "expected exactly one `parts = {...}` literal in %s, found %d. If "
        "board_score was refactored, this test has to learn the new shape -- "
        "it must NEVER fall back to a hard-coded list, which is the failure it "
        "exists to prevent." % (BOARD_SCORE, len(found)))
    return found[0]


def test_every_blocking_component_has_a_lens_or_a_declared_exemption():
    comps = blocking_components()
    assert len(comps) >= 8, comps
    # AT LEAST ONE lens, not exactly one. Two lenses legitimately grade one
    # component from different sides -- `floorplan` is lens 1 (`intent`, the
    # placement half's floorplan check) and part of lens 9 (`spec`, "connector
    # positions") -- and a row carrying a PASS on both while the count is
    # non-zero is wrong twice, which is what the check should say. The table is
    # lens -> components, not a partition.
    claimed = {}
    for lens, keys in converge.LENS_COMPONENTS.items():
        for k in keys:
            claimed.setdefault(k, []).append(lens)
    homeless = [c for c in comps if c not in claimed and c not in EXEMPT]
    assert not homeless, (
        f"blocking components with no lens and no declared exemption: "
        f"{homeless}. A PASS verdict about one of these cannot be contradicted "
        f"by its own row's score -- which is #904's defect, one component "
        f"further along. Map it in converge.LENS_COMPONENTS, or add it to "
        f"EXEMPT here WITH the name of the check that does grade it.")
    stale = [k for k in claimed if k not in comps]
    assert not stale, (
        f"LENS_COMPONENTS maps {stale}, which board_score does not sum into "
        f"`blocking`. score_component() returns None for a key nothing "
        f"measures, so such an entry can never fire: it reads as coverage and "
        f"is not.")
    for name in EXEMPT:
        assert name in comps, (
            f"{name!r} is exempted here but is no longer a blocking "
            f"component. Delete the exemption rather than leaving a waiver "
            f"for a check that no longer exists.")
    # ...and every lens the reference DEFINES must be in the table or be
    # deliberately absent. This is the direction the count above cannot see:
    # the table can cover all nine components while a lens a verifier actually
    # answers with is missing from it, so a PASS on that lens is unrefutable.
    # `floorplan` was covered by `spec` while `intent` -- the lens that
    # literally grades check_floorplan -- had no entry at all.
    for lens in ('connectivity', 'drc', 'spec', 'intent'):
        assert lens in converge.LENS_COMPONENTS, (
            f"lens {lens!r} grades a blocking component and is not in "
            f"LENS_COMPONENTS, so a PASS on it contradicts nothing")
    print(f"  PASS: {len(comps)} blocking components, "
          f"{len(claimed)} lens-covered by "
          f"{len(converge.LENS_COMPONENTS)} lenses, "
          f"{len(EXEMPT)} exempt by declaration")


def test_the_spec_lens_refutes_a_measured_count_and_only_a_measured_one():
    """The conservative half is the whole safety argument for the mapping."""
    measured = {'blocking': 1,
                'blocking_by': {'unrouted': 0, 'broken': 0, 'drc': 0,
                                'undersized': 0, 'floorplan': 0,
                                'impedance': 1, 'length': 0, 'net_widths': 0}}
    got = converge.lens_contradictions(['VERDICT=PASS:lens=spec'], measured)
    assert got == [('spec', 'impedance', 1)], got

    # the FLOORPLAN component, graded and violated.
    fp = {'blocking': 2, 'blocking_by': {'floorplan': 2}}
    assert converge.lens_contradictions(['VERDICT=PASS:lens=spec'], fp) == \
        [('spec', 'floorplan', 2)]

    # UNGRADED must stay silent, in every spelling board_score emits.
    for ungraded in ({'blocking': 0,
                      'components': {'impedance': {'ran': False,
                                                   'count': None}}},
                     {'blocking': 0, 'blocking_by': {'impedance': None}},
                     {'blocking': 0}):
        assert converge.lens_contradictions(
            ['VERDICT=PASS:lens=spec'], ungraded) == [], ungraded

    # A FAIL is honest about any number at all.
    assert converge.lens_contradictions(
        ['VERDICT=FAIL:lens=spec;finding=x;evidence=y'], measured) == []

    # And the case that bypassed the gate with a shift key.
    assert converge.lens_contradictions(
        ['VERDICT=PASS:lens=Spec'], measured) == [('Spec', 'impedance', 1)]
    print("  PASS: spec contradicts a measured count, never an ungraded one")


def test_board_scores_own_formula_lists_what_it_sums():
    src = io.open(BOARD_SCORE, encoding='utf-8').read()
    head = src[:src.index('def ')] if 'def ' in src else src
    i = head.find('blocking = (')
    assert i > 0, "board_score.py's docstring no longer states the formula"
    formula = head[i:head.index(')', i) + 1]
    named = {w for w in formula.replace('+', ' ').replace('(', ' ')
             .replace(')', ' ').replace('=', ' ').split()
             if w.isidentifier()} - {'blocking'}
    comps = set(blocking_components())
    assert named == comps, (
        f"board_score's docstring formula and its `parts` dict disagree: "
        f"only in the docstring {sorted(named - comps)}, only summed "
        f"{sorted(comps - named)}. The formula is what a reader completes the "
        f"lens table from.")
    print(f"  PASS: the stated formula lists all {len(comps)} summed "
          f"components")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
