#!/usr/bin/env python3
"""#726: two footprint blocks, one reference, and only one of them parsed.

`PCBData.footprints` was a dict keyed by reference, so the second block with a
given reference silently REPLACED the first (`kicad_parser.py`, text path and
pcbnew path alike). Measured on the 22 git-tracked boards: 5 of them carry a
duplicate, 12 footprints were lost, and watchy's TP4/TP5 are real test points
rather than unannotated placeholders.

HOW THE BUG SURVIVED THE OLD TESTS. Nothing counted. `compare_pcb_data` --
the harness whose whole job is to prove the two parse paths agree -- compares
the two dicts' KEY SETS, so both paths losing the same block reads as perfect
parity. `check_assembly`'s `coincident_origins`, the one check that exists to
find two parts at one point, iterates the dict and therefore cannot form a
pair out of two blocks that are one entry. And no test anywhere asserted
`len(pcb.footprints)` against the file's own block count; the two numbers were
simply never compared. The defect was invisible by construction, not by
oversight.

SIX THINGS THIS FILE PINS, each easy to get wrong:

1. **Every block survives, with its own pose.** Not "the count is right" --
   the two entries must carry the two DIFFERENT positions. A fix that keeps a
   count by storing the same footprint twice passes a length assert.

2. **The key follows the FILE, not the alphabet and not the uuid.** Both parse
   paths compute the ordinal from their own ordered list, so "deterministic"
   has to mean "determined by file order" or the two paths can disagree while
   each is internally consistent. Pinned with a fixture whose duplicates are
   in the opposite order.

3. **A non-colliding reference is untouched.** The scheme must be inert on 17
   of 22 tracked boards; over-reach here would rename half the corpus.

4. **`duplicate_references` counts OCCURRENCES, not extras**, and this file
   re-derives that from the fixture rather than asserting a literal --
   `check_assembly` reports the board's block total from these values, so an
   off-by-one here ships a wrong number in a JSON report.

5. **The warning goes to STDERR.** 9 shipped call sites across 4 files emit
   bare JSON on stdout, where a WARNING line in front is a `JSONDecodeError`
   at char 0.

6. **The four adversarial shapes an independent review built** and this file
   did not have: a literal `~2` beside two bare twins, two EMPTY references,
   the KiCad 6/7 and 8+ forms claiming one name, and a reference-less block
   between two twins. All four passed on the first run, which is why they are
   pinned rather than left alone.

The corpus arms carry their own counterexamples: `t_every_pad_is_reachable`
FAILED on esp_prog and watchy before the fix and held on the other 20, and
`t_the_duplicate_witnesses_are_still_in_the_corpus` refuses to let the corpus
arms go quietly vacuous if a board is ever re-exported.

`RUN_ALL_FAST_OK`: in-process parses of small synthetic boards plus one pass
over the tracked corpus. It imports `run_utils` only for `corpus_boards`
(a `git ls-files`), which is what the auto-classifier flags; no chain runs
here.

    python3 -X utf8 tests/test_726_duplicate_reference_keys.py
"""
import io
import os
import re
import shutil
import subprocess
import sys
import tempfile
from contextlib import redirect_stdout

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 300

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_utils import corpus_boards                              # noqa: E402
from synth import board_text, footprint_text, write_board        # noqa: E402
from kicad_parser import (parse_kicad_pcb,                       # noqa: E402
                          disambiguate_references,
                          duplicate_reference_counts,
                          iter_footprint_blocks,
                          footprint_raw_reference,
                          find_matching_paren,
                          DUP_REF_SEP)

#: The five tracked boards that carry a duplicate reference. Named so an arm
#: below can refuse to pass when a board stops being a witness -- see
#: `t_the_duplicate_witnesses_are_still_in_the_corpus`.
WITNESSES = ('esp_prog', 'glasgow_revC', 'orangecrab_ext_pll', 'ulx3s',
             'watchy')

FAILURES = []
_TMP = []


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + detail) if detail else ''))
        FAILURES.append(what)


def _board(*fps, **kw):
    p = write_board(board_text(''.join(fps), **kw))
    _TMP.append(p)
    return p


def _parse(path):
    """Parse with stderr silenced -- the warning is arm 7's business."""
    err = io.StringIO()
    old, sys.stderr = sys.stderr, err
    try:
        return parse_kicad_pcb(path)
    finally:
        sys.stderr = old


def _block_count(path):
    """The file's own footprint-block count, by the string-aware matcher.

    Re-derived here rather than read off the parser: an arm that compares the
    parser against itself proves nothing.
    """
    with open(path, encoding='utf-8', errors='replace') as fh:
        content = fh.read()
    n = 0
    for m in re.finditer(r'\(footprint\s+"', content):
        find_matching_paren(content, m.start())   # raises if unbalanced
        n += 1
    return n


# --- 1. every block survives -------------------------------------------------

def t_two_blocks_one_reference_both_survive():
    """Both U7 blocks are present AND carry their own two poses."""
    p = _board(footprint_text('U7', 10, 10, uuid='a'),
               footprint_text('U7', 30, 10, uuid='b'),
               footprint_text('R1', 20, 10, uuid='c'))
    pcb = _parse(p)
    check(len(pcb.footprints) == 3,
          'three blocks, three entries', str(sorted(pcb.footprints)))
    poses = sorted((round(f.x, 3), round(f.y, 3))
                   for k, f in pcb.footprints.items() if k.startswith('U7'))
    # The point of the arm: NOT that there are two entries, but that they are
    # two DIFFERENT parts. Storing one footprint twice keeps the count.
    check(poses == [(10.0, 10.0), (30.0, 10.0)],
          'the two U7 entries carry the two DIFFERENT poses', str(poses))
    check(all(len(f.pads) == 1 for f in pcb.footprints.values()),
          'each entry keeps its own pad')
    total_pads = sum(len(f.pads) for f in pcb.footprints.values())
    check(total_pads == 3, 'no pad is lost with the block', str(total_pads))


def t_the_key_reaches_footprint_reference_and_pad_component_ref():
    """The disambiguated name is the part's identity, not just a dict key.

    `Pad.component_ref` is compared as a same-part test in 7 pad-vs-pad places
    across `check_drc`, `plane_pad_tap`, `route_diff` and
    `diff_pair_multipoint`. If
    the key lived only in the dict, two pads on two different parts would go on
    comparing equal.
    """
    p = _board(footprint_text('U7', 10, 10, uuid='a'),
               footprint_text('U7', 30, 10, uuid='b'))
    pcb = _parse(p)
    for key, fp in pcb.footprints.items():
        check(fp.reference == key,
              'Footprint.reference is the key for %r' % key, fp.reference)
        check(all(pad.component_ref == key for pad in fp.pads),
              'Pad.component_ref is the key for %r' % key,
              str([pad.component_ref for pad in fp.pads]))
    a, b = pcb.footprints['U7'], pcb.footprints['U7' + DUP_REF_SEP + '2']
    check(a.pads[0].component_ref != b.pads[0].component_ref,
          'two pads on two different parts no longer compare equal')


# --- 2. the key follows the file ---------------------------------------------

def t_the_key_is_file_ordered_and_deterministic():
    """Parsing twice gives the same mapping, and the mapping tracks the file."""
    fwd = _board(footprint_text('TP4', 10, 10, uuid='first'),
                 footprint_text('TP4', 90, 90, uuid='second'))
    a, b = _parse(fwd), _parse(fwd)
    check({k: (v.x, v.y) for k, v in a.footprints.items()}
          == {k: (v.x, v.y) for k, v in b.footprints.items()},
          'the same file parses to the same mapping twice')
    check((a.footprints['TP4'].x, a.footprints['TP4'].y) == (10.0, 10.0),
          'the FIRST block in the file keeps the bare name')

    # The same two parts, swapped in the file. If the key were derived from the
    # uuid (sorted or otherwise) or from the alphabet, the mapping would not
    # move. It must.
    rev = _board(footprint_text('TP4', 90, 90, uuid='second'),
                 footprint_text('TP4', 10, 10, uuid='first'))
    c = _parse(rev)
    check((c.footprints['TP4'].x, c.footprints['TP4'].y) == (90.0, 90.0),
          'reversing the file order reverses which block is the bare name')
    check(c.footprints['TP4'].uuid == 'second',
          'and it follows the BLOCK, not the uuid sort order',
          c.footprints['TP4'].uuid)


def t_an_existing_ordinal_name_is_not_shadowed():
    """A board that genuinely contains TP4, TP4 and TP4~2 gets three keys.

    The ordinal is issued against the file's own names as well as the ones
    already handed out, so the synthetic name can never land on a real one.
    """
    for order in ((0, 1, 2), (2, 0, 1), (1, 2, 0)):
        blocks = [footprint_text('TP4', 10, 10, uuid='a'),
                  footprint_text('TP4', 20, 20, uuid='b'),
                  footprint_text('TP4' + DUP_REF_SEP + '2', 30, 30, uuid='c')]
        pcb = _parse(_board(*[blocks[i] for i in order]))
        check(len(pcb.footprints) == 3,
              'three keys survive whatever the file order %s' % (order,),
              str(sorted(pcb.footprints)))
        check(len({f.uuid for f in pcb.footprints.values()}) == 3,
              'and they are three DIFFERENT blocks, order %s' % (order,))


def t_the_scheme_is_a_pure_function_of_the_ordered_list():
    """`disambiguate_references` is where both parse paths agree; unit-pin it."""
    check(disambiguate_references(['A', 'B', 'C']) == ['A', 'B', 'C'],
          'no duplicates -> identity')
    check(disambiguate_references(['A', 'A', 'A'])
          == ['A', 'A' + DUP_REF_SEP + '2', 'A' + DUP_REF_SEP + '3'],
          'ordinals count occurrences from the first')
    check(disambiguate_references([]) == [], 'empty list -> empty list')
    out = disambiguate_references(['A', 'A' + DUP_REF_SEP + '2', 'A'])
    check(len(set(out)) == 3 and out[0] == 'A' and out[1] == 'A' + DUP_REF_SEP + '2',
          'a real ordinal-shaped name is never overwritten', str(out))


# --- 3. inert on a non-colliding name ----------------------------------------

def t_a_unique_reference_is_untouched():
    p = _board(footprint_text('R1', 1, 1, uuid='a'),
               footprint_text('C2', 2, 2, uuid='b'),
               footprint_text('U3', 3, 3, uuid='c'))
    pcb = _parse(p)
    check(sorted(pcb.footprints) == ['C2', 'R1', 'U3'],
          'a board with no duplicate keeps exactly its own names',
          str(sorted(pcb.footprints)))
    check(pcb.duplicate_references == {},
          'and reports no duplicates', str(pcb.duplicate_references))


# --- 4. the reference-less and empty-reference blocks ------------------------

def t_referenceless_uuid_keys_survive():
    """`#uuid` keying (thunderscope's 86 NPTH dots) is unchanged by #726."""
    p = _board(footprint_text(None, 1, 1, uuid='u1'),
               footprint_text(None, 2, 2, uuid='u2'),
               footprint_text('', 3, 3, uuid='u3'),      # esp_prog's shape
               footprint_text('R1', 4, 4, uuid='u4'))
    pcb = _parse(p)
    hashed = sorted(k for k in pcb.footprints if k.startswith('#'))
    check(hashed == ['#u1', '#u2', '#u3'],
          'three reference-less blocks keep three distinct #uuid keys',
          str(hashed))
    check(all(len(pcb.footprints[k].pads) == 1 for k in hashed),
          'and their pads with them')
    check(pcb.duplicate_references == {},
          'distinct uuids are not a duplicate reference',
          str(pcb.duplicate_references))


def t_two_referenceless_blocks_sharing_a_uuid():
    """The `cap_chain` shape: repeated uuids, which forbid a uuid key.

    `kicad_files/cap_chain.kicad_pcb` shares one footprint uuid across C1/C2
    and another across J1/J2, so uuids are demonstrably not unique in this
    corpus. Two reference-LESS blocks that also share one would have collapsed
    onto a single `#uuid` key; the ordinal pass runs after the `#uuid` pass, so
    they no longer do.
    """
    p = _board(footprint_text(None, 1, 1, uuid='same'),
               footprint_text(None, 2, 2, uuid='same'))
    pcb = _parse(p)
    check(len(pcb.footprints) == 2,
          'two reference-less blocks sharing a uuid stay two parts',
          str(sorted(pcb.footprints)))
    poses = sorted((f.x, f.y) for f in pcb.footprints.values())
    check(poses == [(1.0, 1.0), (2.0, 2.0)],
          'with their own poses', str(poses))
    check(pcb.duplicate_references == {'#same': 2},
          'and it is reported under the name the FILE effectively gives them',
          str(pcb.duplicate_references))


def t_the_kicad_6_7_reference_form_is_still_read():
    """`(fp_text reference ...)` -- no tracked board uses it (issue #78)."""
    p = _board(footprint_text('R1', 1, 1, uuid='a', ref_property=False),
               footprint_text('R1', 2, 2, uuid='b', ref_property=False))
    pcb = _parse(p)
    check(sorted(pcb.footprints) == ['R1', 'R1' + DUP_REF_SEP + '2'],
          'the 6/7 form disambiguates the same way',
          str(sorted(pcb.footprints)))


# --- 5. duplicate_references semantics ---------------------------------------

def t_duplicate_references_value_is_the_occurrence_count():
    """RE-DERIVED from the fixture, never a literal.

    `check_assembly` prints these as "TP4 x2" and reports the board's block
    total from them, so "extras" instead of "occurrences" is a wrong number in
    a shipped JSON report.
    """
    spec = {'U7': 4, 'R1': 1, 'C9': 2}
    fps = []
    n = 0
    for ref, count in spec.items():
        for i in range(count):
            n += 1
            fps.append(footprint_text(ref, n, n, uuid='u%d' % n))
    p = _board(*fps)
    pcb = _parse(p)
    expected = {r: c for r, c in spec.items() if c > 1}
    check(pcb.duplicate_references == expected,
          'the value is the OCCURRENCE count, re-derived from the fixture',
          '%s != %s' % (pcb.duplicate_references, expected))
    blocks = _block_count(p)
    distinct = len(spec)
    check(sum(pcb.duplicate_references.values())
          - len(pcb.duplicate_references) == blocks - distinct,
          'sum(values) - len(values) == blocks - distinct references')
    check(len(pcb.footprints) == blocks,
          'and every block is a key', '%d != %d' % (len(pcb.footprints), blocks))


def t_duplicate_references_is_keyed_by_the_FILE_spelling():
    """`{'U7': 2}`, not `{'U7~2': 1}` -- it is what to tell a human to rename."""
    p = _board(footprint_text('U7', 1, 1, uuid='a'),
               footprint_text('U7', 2, 2, uuid='b'))
    pcb = _parse(p)
    check(list(pcb.duplicate_references) == ['U7'],
          'keyed by the reference as the board spells it',
          str(pcb.duplicate_references))
    check(duplicate_reference_counts(['U7', 'U7', 'R1']) == {'U7': 2},
          'and the helper says the same thing standalone')


# --- 6. the warning ----------------------------------------------------------

def t_the_warning_is_on_stderr_and_stdout_stays_clean():
    """9 shipped call sites across 4 files emit bare JSON on stdout; a
    WARNING there is a JSONDecodeError at char 0."""
    p = _board(footprint_text('U7', 1, 1, uuid='a'),
               footprint_text('U7', 2, 2, uuid='b'))
    err = io.StringIO()
    out = io.StringIO()
    old, sys.stderr = sys.stderr, err
    try:
        with redirect_stdout(out):
            parse_kicad_pcb(p)
    finally:
        sys.stderr = old
    check(out.getvalue() == '',
          'stdout is untouched by the duplicate warning',
          repr(out.getvalue()[:120]))
    check('U7 x2' in err.getvalue(),
          'stderr names the reference and its count', repr(err.getvalue()[:160]))

    clean = _board(footprint_text('R1', 1, 1, uuid='a'))
    err2 = io.StringIO()
    old, sys.stderr = sys.stderr, err2
    try:
        parse_kicad_pcb(clean)
    finally:
        sys.stderr = old
    check(err2.getvalue() == '',
          'and a board with no duplicate says nothing at all',
          repr(err2.getvalue()[:120]))


# --- 7. corpus ---------------------------------------------------------------

def t_every_pad_is_reachable_from_exactly_one_footprint():
    """The internal inconsistency, gone.

    Pads are appended to `pads_by_net` BEFORE the footprint is stored, so a
    dropped block left its copper in the net model with no reachable
    Footprint: the router's view and the placement view of one board disagreed
    about how many pads existed. Measured before the fix: 3 orphans (watchy 2,
    on +3V3 and GND; esp_prog 1). This arm FAILED on those two boards and held
    on the other 20 -- it carries its own counterexamples.
    """
    boards = corpus_boards()
    if not boards:
        print('  SKIP: corpus_boards() is empty (git could not answer)')
        return
    bad = []
    for b in boards:
        pcb = _parse(b)
        via_fp = sum(len(f.pads) for f in pcb.footprints.values())
        via_net = sum(len(v) for v in pcb.pads_by_net.values())
        if via_fp != via_net:
            bad.append((os.path.basename(b), via_fp, via_net))
    check(not bad,
          'every pad in pads_by_net is reachable from a footprint, %d boards'
          % len(boards), str(bad))


def t_block_count_equals_key_count():
    boards = corpus_boards()
    if not boards:
        print('  SKIP: corpus_boards() is empty (git could not answer)')
        return
    bad = []
    for b in boards:
        pcb = _parse(b)
        n = _block_count(b)
        if len(pcb.footprints) != n:
            bad.append((os.path.basename(b), n, len(pcb.footprints)))
    check(not bad,
          'len(pcb.footprints) == the file\'s own block count, %d boards'
          % len(boards), str(bad))


def t_the_duplicate_witnesses_are_still_in_the_corpus():
    """Without this the corpus arms can go vacuous without anyone noticing.

    If a board is re-exported with its duplicates renamed, every arm above
    passes on a set that can no longer fail. REPLACE the witness; do not
    delete the assert.
    """
    boards = {os.path.basename(b).replace('.kicad_pcb', ''): b
              for b in corpus_boards()}
    if not boards:
        print('  SKIP: corpus_boards() is empty (git could not answer)')
        return
    missing = []
    for name in WITNESSES:
        if name not in boards:
            missing.append('%s (board gone)' % name)
            continue
        if not _parse(boards[name]).duplicate_references:
            missing.append('%s (no longer carries a duplicate)' % name)
    check(not missing,
          'the five duplicate-reference witnesses still carry duplicates',
          '%s -- if a board legitimately changed, REPLACE the witness rather '
          'than deleting this arm, or the corpus arms above become vacuous'
          % missing)


def t_iter_footprint_blocks_agrees_with_the_parser_on_every_board():
    """One implementation of the naming, used by the parser and every writer."""
    boards = corpus_boards()
    if not boards:
        print('  SKIP: corpus_boards() is empty (git could not answer)')
        return
    bad = []
    for b in boards:
        with open(b, encoding='utf-8', errors='replace') as fh:
            content = fh.read()
        keys = [k for _, _, _, _, k in iter_footprint_blocks(content)]
        pcb = _parse(b)
        if sorted(keys) != sorted(pcb.footprints):
            bad.append(os.path.basename(b))
    check(not bad,
          'iter_footprint_blocks names exactly what the parser keys, %d boards'
          % len(boards), str(bad))


def t_the_adversarial_shapes_a_reviewer_found():
    """Four fixtures an independent review built and this file did not have.

    All four passed on the first run, which is the point: they are cheap, they
    are exactly where a future refactor of the ordinal breaks, and "it already
    works" is a reason to PIN a shape, not a reason to leave it unpinned.
    """
    # A: a literal ordinal-shaped name sitting BESIDE two bare twins. The
    #    ordinal must step over it rather than collide with it.
    two = DUP_REF_SEP + '2'
    pcb = _parse(_board(footprint_text('TP4', 1, 1, uuid='a'),
                        footprint_text('TP4', 2, 2, uuid='b'),
                        footprint_text('TP4' + two, 3, 3, uuid='c')))
    check(sorted(pcb.footprints) == sorted(['TP4', 'TP4' + DUP_REF_SEP + '3',
                                            'TP4' + two]),
          'A: a real `%s2` beside two bare twins keeps all three'
          % DUP_REF_SEP, str(sorted(pcb.footprints)))
    check(pcb.footprints['TP4' + two].uuid == 'c',
          'A: and the literal name still belongs to the block that spells it')

    # B: two EMPTY Reference properties -- esp_prog's three-logo shape.
    pcb = _parse(_board(footprint_text('', 1, 1, uuid='e1'),
                        footprint_text('', 2, 2, uuid='e2'),
                        footprint_text('R1', 3, 3, uuid='r')))
    check(sorted(pcb.footprints) == ['#e1', '#e2', 'R1'],
          'B: two EMPTY references key by uuid, not onto each other',
          str(sorted(pcb.footprints)))

    # C: the KiCad 6/7 form and the modern one claiming ONE name. No tracked
    #    board mixes them, so this can only be covered synthetically.
    pcb = _parse(_board(footprint_text('J9', 1, 1, uuid='m', ref_property=True),
                        footprint_text('J9', 2, 2, uuid='l',
                                       ref_property=False)))
    check(sorted(pcb.footprints) == ['J9', 'J9' + two],
          'C: the 6/7 and 8+ forms disambiguate against EACH OTHER',
          str(sorted(pcb.footprints)))
    check(pcb.footprints['J9'].uuid == 'm'
          and pcb.footprints['J9' + two].uuid == 'l',
          'C: in file order, whichever form each block uses')

    # D: a reference-LESS block BETWEEN two twins. The ordinal counts
    #    OCCURRENCES of the name, not positions in the file, so the block in
    #    between must not shift it.
    pcb = _parse(_board(footprint_text('C7', 1, 1, uuid='c1'),
                        footprint_text(None, 2, 2, uuid='c2'),
                        footprint_text('C7', 3, 3, uuid='c3')))
    check(sorted(pcb.footprints) == ['#c2', 'C7', 'C7' + two],
          'D: a reference-less block between two twins does not shift the '
          'ordinal', str(sorted(pcb.footprints)))
    check(pcb.footprints['C7' + two].uuid == 'c3',
          'D: and the ordinal still names the second BEARER of the name')


TESTS = (
    t_two_blocks_one_reference_both_survive,
    t_the_key_reaches_footprint_reference_and_pad_component_ref,
    t_the_key_is_file_ordered_and_deterministic,
    t_an_existing_ordinal_name_is_not_shadowed,
    t_the_scheme_is_a_pure_function_of_the_ordered_list,
    t_a_unique_reference_is_untouched,
    t_referenceless_uuid_keys_survive,
    t_two_referenceless_blocks_sharing_a_uuid,
    t_the_kicad_6_7_reference_form_is_still_read,
    t_duplicate_references_value_is_the_occurrence_count,
    t_duplicate_references_is_keyed_by_the_FILE_spelling,
    t_the_warning_is_on_stderr_and_stdout_stays_clean,
    t_every_pad_is_reachable_from_exactly_one_footprint,
    t_block_count_equals_key_count,
    t_the_duplicate_witnesses_are_still_in_the_corpus,
    t_iter_footprint_blocks_agrees_with_the_parser_on_every_board,
    t_the_adversarial_shapes_a_reviewer_found,
)


def main():
    try:
        for t in TESTS:
            print('\n%s' % t.__name__)
            t()
    finally:
        for p in _TMP:
            try:
                os.remove(p)
            except OSError:
                pass
    print('\n%d arm(s), %d failure(s)' % (len(TESTS), len(FAILURES)))
    for f in FAILURES:
        print('  FAILED: %s' % f)
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
