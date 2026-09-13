#!/usr/bin/env python3
"""#726 on the IPC read path: `build_pcb_data_from_board` must key a footprint
the way the file does, not by its bare reference.

#726 fixed this for both of main's parse paths -- the text scan and the SWIG
pcbnew walk. This branch's `build_pcb_data_from_board` is neither: it walks a
live kipy board, and it kept `footprints[_fp_reference(fp)] = footprint`. On a
board where two blocks claim one reference, the second overwrote the first, so
the IPC model carried one fewer part AND that part's pads were absent from
`pads_by_net` -- copper the router would route straight through, with nothing
saying so.

WHY THIS FILE AND NOT THE PARITY GATE. `tests/gui_parity/test_726_parse_path_parity.py`
is the real two-path gate, and it cannot run here: it compares against a live
`pcbnew` BOARD, which the port removed, and the IPC read path needs a RUNNING
KiCad (`fake_ipc_board` deliberately serves reads by re-parsing the file, so
driving the kipy builder through it would grade the text parser twice). So the
keying itself is graded here, off the one thing that decides it:
`kipy_raw_references` -- which exists as a function precisely so this test can
reach it -- composed with `disambiguate_references`, exactly as the builder
composes them.

THE ORACLE IS THE FILE, NOT A RESTATEMENT OF THE CODE. The expected keys come
from `iter_footprint_blocks`, the text parser's own namer. A test that rebuilt
the ordinal rule here would pass whatever the rule became.

    python3 -X utf8 tests/test_726_kipy_reference_keys.py
"""
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from kicad_parser import (disambiguate_references,        # noqa: E402
                          duplicate_reference_counts,
                          iter_footprint_blocks,
                          kipy_raw_references)
from run_utils import corpus_boards                        # noqa: E402

FAILS = []


def check(cond, msg):
    if not cond:
        FAILS.append(msg)
    return cond


class _StubFp:
    """The slice of a kipy FootprintInstance `kipy_raw_references` reads.

    `.reference` is what `_fp_reference` probes first, and `.id.value` is the
    uuid it falls back to. Nothing else is touched, which is the point of
    reading the references through a function rather than inline in a builder
    that needs a socket.
    """

    class _Id:
        def __init__(self, value):
            self.value = value

    def __init__(self, reference, uuid):
        self.reference = reference
        self.id = _StubFp._Id(uuid)


def _board_footprints(path):
    """(stub footprints in file order, expected keys) for one board."""
    txt = open(path, encoding='utf-8', errors='replace').read()
    stubs, keys, raws = [], [], []
    for _s, _e, fp_text, raw, key in iter_footprint_blocks(txt):
        # A block whose raw name is '#<uuid>' had no Reference property at all;
        # the live footprint reports '' for it, and the uuid is how both paths
        # recover a unique name.
        uuid = raw[1:] if raw.startswith('#') else ''
        stubs.append(_StubFp('' if raw.startswith('#') else raw, uuid))
        keys.append(key)
        raws.append(raw)
    return stubs, keys, raws


def test_the_ipc_keying_matches_the_file_on_every_corpus_board():
    boards = corpus_boards()
    if not boards:
        print("SKIP: git could not list the corpus")
        return
    graded = 0
    with_dups = []
    for path in boards:
        stubs, expected, raws = _board_footprints(path)
        if not stubs:
            continue
        graded += 1
        got = disambiguate_references(kipy_raw_references(stubs))
        name = os.path.basename(path)
        if not check(got == expected,
                     f"{name}: the IPC keying disagrees with the file's own "
                     f"blocks; first divergence at index "
                     f"{next((i for i, (a, b) in enumerate(zip(got, expected)) if a != b), len(got))}"):
            continue
        check(len(set(got)) == len(got),
              f"{name}: the IPC keying handed out a repeated key, so a "
              f"footprints dict built from it loses a block")
        if duplicate_reference_counts(raws):
            with_dups.append(name)

    check(graded >= 20,
          f"only {graded} corpus board(s) carried footprints; the claim is "
          f"about the tracked corpus, so a short set is not the same test")
    # The denominator that matters: a run where no board carries a duplicate
    # grades the ordinal rule on nothing at all.
    check(len(with_dups) >= 3,
          f"only {len(with_dups)} corpus board(s) actually carry a duplicated "
          f"reference ({with_dups}); this test would be vacuous about #726")
    print(f"  graded {graded} board(s); {len(with_dups)} carry duplicates: "
          f"{', '.join(sorted(with_dups))}")


def test_a_reference_less_footprint_is_keyed_by_uuid_not_collapsed():
    """esp_prog carries three blocks with no Reference property. Keyed by '',
    all three become one dict entry and two parts' pads leave the model."""
    board = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
    if not os.path.isfile(board):
        print("SKIP: esp_prog.kicad_pcb not present")
        return
    stubs, expected, raws = _board_footprints(board)
    refless = [i for i, s in enumerate(stubs) if not s.reference]
    check(len(refless) >= 3,
          f"esp_prog no longer carries reference-less footprints "
          f"({len(refless)}); this test is about a board that has them")
    got = kipy_raw_references(stubs)
    for i in refless:
        check(got[i].startswith('#') and len(got[i]) > 1,
              f"reference-less footprint {i} keyed {got[i]!r}, not by uuid")
    check(len({got[i] for i in refless}) == len(refless),
          "the reference-less footprints share a key, so they collapse onto "
          "one entry and the rest's pads leave pads_by_net")


def test_a_footprint_that_cannot_name_itself_still_gets_a_unique_key():
    """No reference AND no uuid: '?' is not unique on its own, so the ordinal
    rule is what keeps two of them apart. A silently shared key here is the
    same lost-block defect with a different cause."""
    stubs = [_StubFp('', ''), _StubFp('', ''), _StubFp('R1', 'u1')]
    raws = kipy_raw_references(stubs)
    check(raws[:2] == ['?', '?'],
          f"a nameless footprint should report '?', got {raws[:2]}")
    keys = disambiguate_references(raws)
    check(len(set(keys)) == len(keys),
          f"two nameless footprints collapsed onto one key: {keys}")


TESTS_TO_RUN = [
    test_the_ipc_keying_matches_the_file_on_every_corpus_board,
    test_a_reference_less_footprint_is_keyed_by_uuid_not_collapsed,
    test_a_footprint_that_cannot_name_itself_still_gets_a_unique_key,
]


def run():
    for t in TESTS_TO_RUN:
        print(f"--- {t.__name__}")
        t()
    if FAILS:
        for f in FAILS:
            print(f"  FAIL  {f}")
        print(f"\n{len(FAILS)} check(s) FAILED")
        return False
    print("ALL PASS  #726 keying on the IPC read path")
    return True


if __name__ == '__main__':
    sys.exit(0 if run() else 1)
