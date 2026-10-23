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

WHAT THIS GATE CANNOT SEE, named rather than implied:

  * It scans ONE FILE, converge.py. The three loop_driver.py sites, and
    the driver's local fallback that the driver-only tests here pinned, were
    retired with that driver.
    `py_tools/render_placement.py` (~:570, ~:641-651)
    holds a sixth site of the same shape -- hand-rolled on `hashlib`, matching
    / mismatch-skips / no-sha-notes -- and degrades the same way. #963 does not
    touch it, and widening the scan to the whole tree would make every
    legitimate content hash in the repo a finding.
  * It is syntax, not semantics. A copy that routes the compare through a
    helper of its own, or through a dict lookup, is invisible.
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

CONVERGE = os.path.join(ROOT, 'py_placer', 'converge.py')

#: The one function allowed to compare a freshly computed digest against a
#: payload's `board_sha`: converge's canonical predicate.
ALLOWED_COMPARE_SITES = {
    ('py_placer/converge.py', 'score_board_binding'),
}

#: Sites that compare a digest they hashed themselves and are NOT this
#: predicate, each with the question it actually answers. A list like this is
#: where a guard usually fails, so two things hold it honest: every entry is
#: PRINTED with its reason on every run, and a new name fails the test until
#: somebody writes that reason down. "Does this board appear in the ledger" and
#: "do two ledgers describe the same work" are different questions from "does
#: this score grade this board", and folding them together would be the
#: opposite of what #963 is about.
NOT_THE_PREDICATE = {}

#: A copy of the predicate in the assign-then-compare spelling, scanned as a
#: POSITIVE CONTROL: with no such site left in converge, a scan that matched
#: nothing would otherwise read exactly like a pass.
_CONTROL_COPY = """
def copy(board, payload):
    sha = sha256_file(board)
    if sha != payload.get('board_sha'):
        return 'other'
"""


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
    """A call that produces a board digest, in any spelling used in this repo.

    Three, and the third was a measured blind spot in the first draft of this
    file: `sha256_file(p)` under any import alias, `hashlib.sha256(...)`, and
    `....hexdigest()`. The hashlib pair is not hypothetical -- it is exactly
    how `py_tools/render_placement.py` and
    `py_tools/board_score.py` already spell it, so a copy
    pasted from either would have walked straight past a scan that only knew
    `sha256_file`.
    """
    if not isinstance(node, ast.Call):
        return False
    if isinstance(node.func, ast.Name):
        return node.func.id.lstrip('_').startswith('sha256_file')
    if isinstance(node.func, ast.Attribute):
        return node.func.attr in ('sha256_file', 'sha256', 'hexdigest')
    return False


def test_one_place_compares_a_board_digest_to_a_payload():
    """A `sha256_file(...)` call INSIDE a comparison is the copy's fingerprint.

    Every one of the three sites this change removed had exactly this shape --
    `if sha256_file(a.board) != _psha:` -- so this is the assertion that would
    have caught them being written, and the one that catches the fourth.
    """
    found = set()
    for rel, path in (('py_placer/converge.py', CONVERGE),):
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
    print(f"  PASS: {len(found)} digest comparison(s), all expected")


def _names_from_sha(fn):
    """Names bound DIRECTLY to a `sha256_file(...)` result inside `fn`."""
    out = set()
    for n in ast.walk(fn):
        if not isinstance(n, ast.Assign):
            continue
        pairs = []
        if isinstance(n.value, ast.Tuple):
            for tgt in n.targets:
                if isinstance(tgt, ast.Tuple) and \
                        len(tgt.elts) == len(n.value.elts):
                    pairs += list(zip(tgt.elts, n.value.elts))
        else:
            pairs = [(t, n.value) for t in n.targets]
        for tgt, val in pairs:
            if isinstance(tgt, ast.Name) and _is_sha_call(val):
                out.add(tgt.id)
    return out


def test_no_other_function_compares_a_stored_digest():
    """The other spelling of the same copy, caught by dataflow not by keywords.

    A copy that assigns the digest first -- `sha = sha256_file(b)` then
    `if sha != psha:` -- has no Call inside its Compare and walks straight past
    the test above.

    The obvious cheaper scan, "this function hashes a board AND mentions
    `board_sha`", is what an earlier draft of this file did, and it was wrong
    in both directions on the real tree: `cmd_record` hashes a LENS FILE for
    `lens_source` and mentions `board_sha` in a dict it writes, and
    `cmd_verdict` hashes `--board` and reads the score's key without ever
    comparing the two. Neither is a copy of the predicate. Requiring the
    COMPARISON is what separates deciding from mentioning.
    """
    def _scan(rel, tree):
        out = set()
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            held = _names_from_sha(fn)
            if not held:
                continue
            for n in ast.walk(fn):
                if isinstance(n, ast.Compare) and any(
                        isinstance(p, ast.Name) and p.id in held
                        for p in [n.left] + list(n.comparators)):
                    out.add((rel, fn.name))
        return out

    assert _scan('<control>', ast.parse(_CONTROL_COPY)) == {
        ('<control>', 'copy')}, (
        "the dataflow scan does not see a copy of the predicate at all, so "
        "its silence about converge.py is not evidence")
    hits = _scan('py_placer/converge.py', _tree(CONVERGE))
    exempt = sorted(hits & set(NOT_THE_PREDICATE))
    for site in exempt:
        print(f"    not the predicate -- {site[1]}: {NOT_THE_PREDICATE[site]}")
    extra = hits - ALLOWED_COMPARE_SITES - set(NOT_THE_PREDICATE)
    assert not extra, (
        f"{sorted(extra)} compare a digest they hashed themselves. If that is "
        f"'does this score grade this board', call score_board_binding; if it "
        f"is a different question, say which in NOT_THE_PREDICATE.")
    missing = set(NOT_THE_PREDICATE) - hits
    assert not missing, (
        f"{sorted(missing)} no longer compare a digest -- an exemption for a "
        f"site that does not exist is a note nobody will ever re-read")
    print(f"  PASS: {len(hits) - len(exempt)} assign-then-compare site(s) in "
          f"scope, {len(exempt)} answering another question")


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
    print("  PASS: this / other / unbound / unknown are four distinct answers")


def test_unknown_never_reads_as_a_mismatch():
    """"I could not tell" must not switch a check ON.

    A check that refused because it could not answer would be the same class of
    mistake it exists to catch, and both retired docstrings said so.
    """
    import converge
    with tempfile.TemporaryDirectory() as tmp:
        missing = os.path.join(tmp, 'gone.kicad_pcb')
        payload = {'board_sha': 'a' * 64}
        assert converge.score_board_binding(missing, payload)[0] == 'unknown'
        assert converge._grades_another_board(missing, payload) is False
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


TESTS = [
    test_one_place_compares_a_board_digest_to_a_payload,
    test_no_other_function_compares_a_stored_digest,
    test_the_binding_is_four_valued,
    test_unknown_never_reads_as_a_mismatch,
    test_a_caller_supplied_digest_is_used_instead_of_rehashing,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
