"""ONE predicate for "does this score grade this board" (#963).

#963's contributor measured the symptom from outside the repo: `converge
record --kind systemic --score-file <a stale score>` returned exit 0 and wrote
`accepted: true` with only a stderr WARNING, while `loop_driver --stage L5`
handed the SAME pair refused at exit 4. They called it "different enforcement
strengths", and the strengths are fine -- a baseline row legitimately attaches a
parent score to a rejected candidate, a close-out legitimately does not. What
was not fine is that the two answers came from two different implementations of
one question, and there were FIVE of them:

    py_placer/converge.py    _grades_another_board          a bool, used once
    py_placer/converge.py    cmd_record, inline             warns; re-parsed
                                                            a.score behind a
                                                            bare except, a
                                                            SECOND read of
                                                            bytes already
                                                            parsed 200 lines up
    loop_driver.py           _score_board_mismatch          refuses (L3)
    loop_driver.py           _verdict, inline               refuses (L5)
    loop_driver.py           _close_out, inline             refuses, about the
                                                            close-out document

THE ONLY TEST HERE THAT CATCHES A REINTRODUCED COPY IS THE AST ONE. Every
behavioural assertion below passes just as happily with five implementations as
with one -- that is precisely how five of them arrived. So the first test walks
the syntax tree and refuses a sixth, in the shape of `tests/test_711_sibling_
lists.py`, which refuses a tenth sibling list for the same reason.

The scan asserts it FOUND the expected sites before it asserts it found nothing
else: a silently-empty walk reads exactly like a pass.
"""
import ast
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _pkg in ('py_placer', 'py_router', 'py_tools'):
    _d = os.path.join(ROOT, _pkg)
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.insert(0, _d)
sys.path.insert(0, ROOT)

DRIVER = os.path.join(ROOT, '.claude', 'skills',
                      'plan-pcb-placement-and-routing', 'scripts',
                      'loop_driver.py')
CONVERGE = os.path.join(ROOT, 'py_placer', 'converge.py')

#: The two functions allowed to compare a freshly computed digest against a
#: payload's `board_sha`: converge's canonical predicate, and the driver's
#: local fallback for the case where converge itself cannot be imported.
ALLOWED_COMPARE_SITES = {
    ('py_placer/converge.py', 'score_board_binding'),
    ('loop_driver.py', '_score_board_mismatch'),
}

#: The driver's own FIXTURE builders hash a board and write its digest into a
#: payload they then hand to a stage -- that is building the evidence, not
#: answering the question, and the weaker scan below cannot tell the two apart
#: from syntax alone. Exempt by name, PRINTED rather than silently skipped, and
#: deliberately short: a new name here should have to be argued for.
FIXTURE_BUILDERS = {
    ('loop_driver.py', '_self_test'),
    ('loop_driver.py', '_refusal_scenarios'),
    ('loop_driver.py', '_dump_all'),
}


def _tree(path):
    return ast.parse(io.open(path, encoding='utf-8').read(), filename=path)


def _enclosing_functions(tree):
    """{node: name of the innermost def containing it} for every node."""
    owner = {}
    for fn in ast.walk(tree):
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for n in ast.walk(fn):
                owner[n] = fn.name          # innermost wins: walk is top-down
    return owner


def _is_sha_call(node):
    """A direct call to board_store.sha256_file, under any import alias."""
    return (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
            and node.func.id.lstrip('_').startswith('sha256_file'))


def test_one_place_compares_a_board_digest_to_a_payload():
    """A `sha256_file(...)` call INSIDE a comparison is the copy's fingerprint.

    Every one of the three sites this change removed had exactly this shape --
    `if sha256_file(a.board) != _psha:` -- so this is the assertion that would
    have caught them being written, and the one that catches the fourth.
    """
    found = set()
    for rel, path in (('py_placer/converge.py', CONVERGE),
                      ('loop_driver.py', DRIVER)):
        tree = _tree(path)
        owner = _enclosing_functions(tree)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Compare):
                continue
            parts = [node.left] + list(node.comparators)
            if any(_is_sha_call(p) for p in parts):
                found.add((rel, owner.get(node, '<module>')))
    assert found, ("no digest comparison found at all -- the scan matched "
                   "nothing, which reads like a pass and is not one")
    extra = found - ALLOWED_COMPARE_SITES
    assert not extra, (
        f"a new copy of the score-to-board compare: {sorted(extra)}.\n"
        f"Call converge.score_board_binding() instead -- #963 removed three "
        f"of these and the whole point is that there is now one.")
    missing = ALLOWED_COMPARE_SITES - found
    assert not missing, (f"the expected site(s) {sorted(missing)} no longer "
                         f"compare a digest -- this scan now guards nothing")
    print(f"  PASS: {len(found)} digest comparison(s), both expected")


def test_no_other_function_reads_board_sha_and_hashes_a_board():
    """The weaker half of the same fingerprint, for a copy spelled differently.

    A copy that assigns the digest first (`sha = sha256_file(b)` then
    `if sha != psha`) has no Call inside its Compare and slips past the test
    above. Holding "hashes a board AND reads a payload's board_sha" to the same
    two functions catches that spelling too.
    """
    hits = set()
    for rel, path in (('py_placer/converge.py', CONVERGE),
                      ('loop_driver.py', DRIVER)):
        tree = _tree(path)
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            body = ast.walk(fn)
            hashes = reads = False
            for n in body:
                if _is_sha_call(n):
                    hashes = True
                elif isinstance(n, ast.Constant) and n.value == 'board_sha':
                    reads = True
            if hashes and reads:
                hits.add((rel, fn.name))
    assert hits, "scan matched nothing"
    exempt = sorted(hits & FIXTURE_BUILDERS)
    for site in exempt:
        print(f"    exempt (builds fixtures, does not decide): {site[1]}")
    extra = hits - ALLOWED_COMPARE_SITES - FIXTURE_BUILDERS
    assert not extra, (
        f"{sorted(extra)} both hash a board and read a payload's board_sha -- "
        f"that is the predicate, and it lives in score_board_binding")
    print(f"  PASS: {len(hits) - len(exempt)} deciding function(s), "
          f"{len(exempt)} exempt")


def test_the_driver_asks_the_one_function_everywhere_it_used_to_inline():
    """L3, L5's verdict and the close-out all route through one helper."""
    tree = _tree(DRIVER)
    owner = _enclosing_functions(tree)
    callers = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) \
                and node.func.id == '_score_board_mismatch':
            callers.add(owner.get(node, '<module>'))
    for want in ('l3', '_verdict', '_close_out'):
        assert want in callers, (
            f"{want} no longer asks _score_board_mismatch -- the three "
            f"refusing sites are exactly what #963 folded together "
            f"(callers seen: {sorted(callers)})")
    print(f"  PASS: {sorted(callers)} all go through one helper")


def test_the_binding_is_four_valued():
    """`unbound` must stay distinguishable from `other`.

    Collapsing it into a bool deletes cmd_record's pre-B4 "payload carries no
    board_sha" disclosure, which is a different operator action from "grades a
    different board" -- and the bool is what four of the five copies were.
    """
    import converge
    with tempfile.TemporaryDirectory() as tmp:
        b = os.path.join(tmp, 'b.kicad_pcb')
        io.open(b, 'w', encoding='utf-8').write('(kicad_pcb)\n')
        from board_store import sha256_file
        sha = sha256_file(b)
        assert converge.score_board_binding(b, {'board_sha': sha}) \
            == ('this', sha)
        assert converge.score_board_binding(b, {'board_sha': 'f' * 64}) \
            == ('other', 'f' * 64)
        assert converge.score_board_binding(b, {'blocking': 0}) \
            == ('unbound', None)
        assert converge.score_board_binding(b, None) == ('unbound', None)
        missing = os.path.join(tmp, 'gone.kicad_pcb')
        assert converge.score_board_binding(missing, {'board_sha': 'a' * 64}) \
            == ('unknown', 'a' * 64)
        assert set(converge.SCORE_BINDINGS) == {'this', 'other', 'unbound',
                                                'unknown'}
    print("  PASS: this / other / unbound / unknown are four distinct answers")


def test_unknown_never_reads_as_a_mismatch():
    """"I could not tell" must not switch a check ON.

    A check that refused because it could not answer would be the same class of
    mistake it exists to catch, and both retired docstrings said so.
    """
    import converge
    sys.path.insert(0, os.path.dirname(DRIVER))
    import loop_driver as L
    with tempfile.TemporaryDirectory() as tmp:
        missing = os.path.join(tmp, 'gone.kicad_pcb')
        payload = {'board_sha': 'a' * 64}
        assert converge.score_board_binding(missing, payload)[0] == 'unknown'
        assert converge._grades_another_board(missing, payload) is False
        assert L._score_board_mismatch(missing, payload) is None
    print("  PASS: an unanswerable question refuses nothing")


def test_a_caller_supplied_digest_is_used_instead_of_rehashing():
    """cmd_record hands in the sha `store.put` just computed.

    Two reads of one file is not only wasted work: the mismatch used to be
    judged on a SECOND, weaker read of bytes already parsed, and between the
    two reads the file can change. Proven by deleting the board and checking
    the answer still comes back.
    """
    import converge
    with tempfile.TemporaryDirectory() as tmp:
        gone = os.path.join(tmp, 'gone.kicad_pcb')
        assert converge.score_board_binding(
            gone, {'board_sha': 'b' * 64}, board_sha='b' * 64) \
            == ('this', 'b' * 64)
        assert converge.score_board_binding(
            gone, {'board_sha': 'b' * 64}, board_sha='c' * 64) \
            == ('other', 'b' * 64)
    print("  PASS: a supplied digest answers without touching the file")


def test_the_local_fallback_agrees_with_the_shared_predicate():
    """A shared predicate behind a bare `except` is worse than five copies.

    One unimportable module would then switch off every gate at once, silently
    -- which is what the neighbouring `lens_contradictions` import does today.
    So the driver keeps a local fallback; this pins that the two paths give the
    same answer, and that the blind case is DISCLOSED rather than silent.
    """
    sys.path.insert(0, os.path.dirname(DRIVER))
    import loop_driver as L
    from board_store import sha256_file
    with tempfile.TemporaryDirectory() as tmp:
        b = os.path.join(tmp, 'b.kicad_pcb')
        io.open(b, 'w', encoding='utf-8').write('(kicad_pcb)\n')
        sha = sha256_file(b)
        cases = [{'board_sha': sha}, {'board_sha': 'd' * 64}, {'blocking': 0}]
        shared = [L._score_board_mismatch(b, p) for p in cases]
        real, L._converge_module = L._converge_module, lambda: None
        try:
            fallback = [L._score_board_mismatch(b, p) for p in cases]
        finally:
            L._converge_module = real
        assert shared == fallback == [None, 'd' * 64, None], \
            f"shared {shared} != fallback {fallback}"
        assert L._BINDING_BLIND is None, \
            "the fallback answered, so nothing should be disclosed as blind"
        assert L._binding_note() == ''
    print("  PASS: fallback agrees, and answering is not reported as blind")


def test_a_driver_that_cannot_answer_says_so():
    """The disclosure exists and reaches a stage's text."""
    sys.path.insert(0, os.path.dirname(DRIVER))
    import loop_driver as L
    real_blind, L._BINDING_BLIND = L._BINDING_BLIND, 'a simulated import error'
    try:
        note = L._binding_note()
        assert 'could NOT check' in note, note
        assert 'a simulated import error' in note, note
    finally:
        L._BINDING_BLIND = real_blind
    src = io.open(DRIVER, encoding='utf-8').read()
    assert src.count('{_binding_note()}') >= 3, (
        "the disclosure is defined but barely called -- an instrument with no "
        "production caller reports nothing")
    print("  PASS: the blind case is disclosed, and the note has callers")


TESTS = [
    test_one_place_compares_a_board_digest_to_a_payload,
    test_no_other_function_reads_board_sha_and_hashes_a_board,
    test_the_driver_asks_the_one_function_everywhere_it_used_to_inline,
    test_the_binding_is_four_valued,
    test_unknown_never_reads_as_a_mismatch,
    test_a_caller_supplied_digest_is_used_instead_of_rehashing,
    test_the_local_fallback_agrees_with_the_shared_predicate,
    test_a_driver_that_cannot_answer_says_so,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
