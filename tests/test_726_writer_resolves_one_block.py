#!/usr/bin/env python3
"""#726: one placement must move one block, not every block sharing a name.

`placement/writer.py:write_placed_output` located footprint blocks in the
.kicad_pcb TEXT and matched each block's own `(property "Reference" "XX")`
against a dict keyed by reference. Nothing consumed an entry, so ONE placement
rewrote EVERY block carrying that string.

HOW THE BUG SURVIVED THE OLD TEST. `tests/test_457_writer_precision.py` covers
this writer in ten arms -- coordinate precision, fractional rotation, pad-angle
folding, and the #113 string-aware scan -- and every one of them uses distinct
references (`C1`/`C2`). Not one fixture put two blocks under one name, so the
loop's "look the string up and keep going" was never once exercised against a
second bearer. The writer's own `Modified N footprint positions` line counted
BLOCKS rather than placements and was the only observable, and nothing read it.

THE MEASUREMENT THIS FILE TURNS INTO A GATE. `placement/perturb.py`'s
`_all_at_current` hands this writer every footprint at the pose it already has
-- its docstring says "Poses are unchanged for non-members -- only their
formatting is", and `perturb.py` builds the stress harness's ground-truth
CONTROL board with it. Measured over the 22 git-tracked boards before the fix,
that call relocated **12 footprint blocks across 5 boards, up to 70.66 mm**
(four of glasgow's six casualties are `kikit:Tab` panel tabs). After: 0 on
22 of 22. `tests/measure_726_writer_teleport.py` produces the table; the
`t_an_identity_write_moves_nothing` arm below is that table as an assertion,
and the 17 boards that always read 0 are its built-in negative controls.

SIX THINGS THIS FILE PINS, each easy to get wrong:

1. **Exactly one block moves, and the twin is BYTE-IDENTICAL.** Compared as a
   block SUBSTRING, not as a whole file: the writer also runs
   `move_copper_text_to_silkscreen` over the whole text, so a file-level
   comparison fails on four tracked boards for a reason that has nothing to do
   with #726.

2. **POSES, not bytes, for the identity write.** The writer reformats every
   `(at)` it touches to `:.6f`, so `(at 141.2 95.9)` becomes
   `(at 141.200000 95.900000)` with the part in the same place. A byte
   comparison calls that a move.

3. **The OTHER twin is addressable too.** A fix that always resolves to block 0
   passes arm 1 and is still wrong.

4. **`Modified N` counts placements applied, not blocks rewritten** -- anchored
   on the full phrase, because `writer.py` prints `Modified {n} reference
   label(s)` in the same module and `tests/stress/stage_blind.py` quotes the
   phrase in PROSE. A bare `Modified ` grep matches three things, two of which
   are not this code.

5. **A placement that matches nothing is REPORTED.** It used to be discarded in
   silence, so a `--ref` typo produced a successful-looking write that did
   nothing.

6. **A reference-LESS `#uuid` block is addressable.** The parser has always
   handed those keys out and the writer could never act on one, so the placer
   believed it had moved a part the file disagreed about.

WHAT THE BATTERY MEASURED (`python3 -X utf8 tests/mutate_726.py`, 18 rows over
`kicad_parser.py`, `placement/writer.py`, `placement/parser.py`,
`placement/seeder.py`, `check_assembly.py` and `gui_utils.py`):

    text-path-reverts-to-last-wins                       KILLED
    pcbnew-path-reverts-to-last-wins                     KILLED
    only-the-text-path-disambiguates                     KILLED
    the-ordinal-starts-at-the-wrong-block                KILLED
    duplicate-references-counts-extras-not-occurrences   KILLED
    the-key-is-uuid-sorted-not-file-ordered              KILLED
    the-ordinal-can-shadow-a-real-name                   KILLED
    the-warning-goes-to-stdout                           KILLED
    the-key-never-reaches-pad-component_ref              KILLED
    writer-matches-by-name-again                         KILLED
    writer-resolves-every-duplicate-to-block-zero        KILLED
    writer-counts-blocks-not-placements                  KILLED
    writer-drops-an-unmatched-placement-in-silence       KILLED
    the-label-writer-keeps-its-own-ref-lookup            KILLED
    side-maps-keep-the-last-block                        KILLED
    stamp_locked-locks-every-namesake                    KILLED
    footprint-blocks-uses-the-old-formula                KILLED
    gui-sync-matches-by-bare-reference                   KILLED

    18 rows: 18 killed, 0 survived, 0 broken, 0 disagreeing with expectation

The FIRST run was 16 killed and 2 survived, and both non-kills were real:

  * `the-key-never-reaches-pad-component_ref` rebound the key AFTER the value
    had been taken from it, so the row mutated nothing and could only survive.
    A hole in the battery, not in the tests -- and one a declared expectation
    launders, since it read as a deliberate non-kill.
  * `gui-sync-matches-by-bare-reference` survived because NOTHING covered
    `gui_utils.sync_footprint_positions_from_board` on a board with duplicate
    references. `tests/gui_parity/test_footprint_position_sync.py` covers that
    function and stays GREEN through the mutation, because it runs on a
    61-block board with 61 distinct references -- a passing gate on a board
    that cannot express the defect. `tests/gui_parity/test_726_gui_sync.py`
    exists because that row said so.

`--verify-anchors` checks each anchor repo-wide rather than only in its target,
and caught three of the eighteen before the first run: `file=sys.stderr)` alone
reaches 31 tracked files, `matched.add(key)` appears in both writers, and the
ordinal's `while` line is also in the measurement script's copy of the scheme.

`RUN_ALL_TIMEOUT` is generous because the identity-write arm writes all 22
tracked boards; there is no chain run here.

    python3 -X utf8 tests/test_726_writer_resolves_one_block.py
"""
import io
import os
import re
import shutil
import sys
import tempfile
from contextlib import redirect_stdout

RUN_ALL_TIMEOUT = 900

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_utils import corpus_boards                              # noqa: E402
from synth import board_text, footprint_text, write_board        # noqa: E402
from kicad_parser import (parse_kicad_pcb, iter_footprint_blocks,  # noqa: E402
                          DUP_REF_SEP)
from kicad_writer import move_copper_text_to_silkscreen          # noqa: E402
from placement.writer import write_placed_output, write_label_output  # noqa: E402
from placement.labels import LabelResult                         # noqa: E402
from placement.seeder import stamp_locked                        # noqa: E402

TOL = 1e-6      # mm; both sides are nm-quantized, so a real move is huge

FAILURES = []
_TMPDIR = tempfile.mkdtemp(prefix='t726_')


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + detail) if detail else ''))
        FAILURES.append(what)


def _board(*fps):
    return write_board(board_text(''.join(fps)),
                       os.path.join(_TMPDIR, 'in%d.kicad_pcb' % len(os.listdir(_TMPDIR))))


def _out():
    fd, p = tempfile.mkstemp(suffix='.kicad_pcb', dir=_TMPDIR)
    os.close(fd)
    return p


def _write(src, placements, quiet=True):
    """Run the writer; return (out_path, its stdout)."""
    out = _out()
    buf = io.StringIO()
    with redirect_stdout(buf):
        write_placed_output(src, out, placements)
    return out, buf.getvalue()


def _read(p):
    with open(p, encoding='utf-8', errors='replace') as fh:
        return fh.read()


def _blocks_by_key(path):
    """{key: block text} -- the substring the twin-identity arm compares."""
    return {k: t for _, _, t, _, k in iter_footprint_blocks(_read(path))}


def _poses(path):
    """{key: (x, y, rot)} straight off the file."""
    at = re.compile(r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)')
    out = {}
    for _, _, t, _, k in iter_footprint_blocks(_read(path)):
        m = at.search(t)
        out[k] = (float(m.group(1)), float(m.group(2)),
                  float(m.group(3)) if m and m.group(3) else 0.0)
    return out


def _moved(before, after):
    """Keys whose pose changed. Requires the same key set on both sides."""
    assert set(before) == set(after), (
        'the writer changed the block set: only-before=%s only-after=%s'
        % (sorted(set(before) - set(after))[:4],
           sorted(set(after) - set(before))[:4]))
    out = []
    for k, (x0, y0, r0) in before.items():
        x1, y1, r1 = after[k]
        d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        if d > TOL or abs(r1 - r0) > TOL:
            out.append((k, d))
    return out


# --- 1 & 3: exactly one block, and either one --------------------------------

def t_one_placement_moves_exactly_one_block():
    src = _board(footprint_text('U7', 10, 10, uuid='a'),
                 footprint_text('U7', 30, 10, uuid='b'),
                 footprint_text('R1', 20, 10, uuid='c'))
    before = _blocks_by_key(src)
    out, _ = _write(src, [{'reference': 'U7', 'new_x': 15.0, 'new_y': 15.0,
                           'new_rotation': 0.0}])
    after = _blocks_by_key(out)
    check(set(before) == set(after), 'the block set is unchanged',
          str(sorted(set(before) ^ set(after))))
    check(_poses(out)['U7'] == (15.0, 15.0, 0.0),
          'the named block moved', str(_poses(out)['U7']))
    twin = 'U7' + DUP_REF_SEP + '2'
    check(after[twin] == before[twin],
          'the TWIN block is BYTE-IDENTICAL', 'twin text changed')
    check(after['R1'] == before['R1'],
          'an unrelated block is BYTE-IDENTICAL')


def t_the_other_twin_is_addressable_too():
    """A fix that always resolves to block 0 passes the arm above."""
    src = _board(footprint_text('U7', 10, 10, uuid='a'),
                 footprint_text('U7', 30, 10, uuid='b'))
    before = _blocks_by_key(src)
    twin = 'U7' + DUP_REF_SEP + '2'
    out, _ = _write(src, [{'reference': twin, 'new_x': 55.0, 'new_y': 55.0,
                           'new_rotation': 0.0}])
    after = _blocks_by_key(out)
    check(_poses(out)[twin] == (55.0, 55.0, 0.0),
          'the SECOND twin moves when it is the one named',
          str(_poses(out)[twin]))
    check(after['U7'] == before['U7'],
          'and the first twin is BYTE-IDENTICAL')


def t_the_twins_are_independent_in_one_call():
    """Both named at once -> both move, each to its own pose."""
    src = _board(footprint_text('U7', 10, 10, uuid='a'),
                 footprint_text('U7', 30, 10, uuid='b'))
    twin = 'U7' + DUP_REF_SEP + '2'
    out, txt = _write(src, [
        {'reference': 'U7', 'new_x': 1.0, 'new_y': 2.0, 'new_rotation': 0.0},
        {'reference': twin, 'new_x': 3.0, 'new_y': 4.0, 'new_rotation': 0.0}])
    p = _poses(out)
    check(p['U7'] == (1.0, 2.0, 0.0) and p[twin] == (3.0, 4.0, 0.0),
          'two placements, two blocks, two different poses', str(p))
    check('Modified 2 footprint positions' in txt,
          'and the count is 2', txt.strip().splitlines()[0] if txt else '')


# --- 2: the identity write, over the tracked corpus --------------------------

def t_an_identity_write_moves_nothing():
    """The headline. Before the fix: 12 blocks across 5 boards, up to 70.66 mm.

    POSES, not bytes -- the writer reformats every `(at)` it is handed to
    `:.6f`, which is not a move.
    """
    boards = corpus_boards()
    if not boards:
        print('  SKIP: corpus_boards() is empty (git could not answer)')
        return
    bad = []
    for b in boards:
        pcb = parse_kicad_pcb(b)
        placements = [{'reference': r, 'new_x': f.x, 'new_y': f.y,
                       'new_rotation': f.rotation}
                      for r, f in pcb.footprints.items()]
        before = _poses(b)
        out, _ = _write(b, placements)
        moved = _moved(before, _poses(out))
        os.remove(out)
        if moved:
            bad.append((os.path.basename(b), moved[:4]))
    check(not bad,
          'handing the writer every part at its own pose moves NOTHING, '
          '%d boards' % len(boards), str(bad))


def t_a_zero_placement_write_changes_only_the_copper_text_pass():
    """The zero-offset control.

    HONEST NOTE: this arm passes before AND after the fix. It is a control
    against collateral damage, not a discriminator -- it exists because the
    obvious version of it ("a no-placement write is byte-identical to the
    input") is FALSE, on 4 of the 22 tracked boards, for a reason unrelated to
    #726: `write_placed_output` runs `move_copper_text_to_silkscreen` over the
    whole file unconditionally. Asserting plain byte-identity produces four
    spurious failures that read exactly like the defect. The expected text is
    re-derived by calling that pass, not hardcoded.
    """
    boards = corpus_boards()
    if not boards:
        print('  SKIP: corpus_boards() is empty (git could not answer)')
        return
    bad = []
    moved_text = []
    for b in boards:
        src = _read(b)
        expected = move_copper_text_to_silkscreen(src)
        out, _ = _write(b, [])
        got = _read(out)
        os.remove(out)
        if got != expected:
            bad.append(os.path.basename(b))
        if expected != src:
            moved_text.append(os.path.basename(b))
    check(not bad,
          'a no-placement write is exactly the copper-text pass, %d boards'
          % len(boards), str(bad))
    check(moved_text,
          'and that pass is NOT a no-op everywhere -- %d board(s) move copper '
          'text, which is why plain byte-identity would be the wrong assert: %s'
          % (len(moved_text), moved_text),
          'if this is empty the arm above has become vacuous')


# --- 4: the count ------------------------------------------------------------

def t_the_modified_count_is_placements_not_blocks():
    src = _board(footprint_text('U7', 10, 10, uuid='a'),
                 footprint_text('U7', 30, 10, uuid='b'),
                 footprint_text('R1', 20, 10, uuid='c'))
    _, txt = _write(src, [{'reference': 'U7', 'new_x': 1.0, 'new_y': 1.0,
                           'new_rotation': 0.0}])
    # The FULL phrase: `writer.py` also prints "Modified N reference label(s)",
    # and tests/stress/stage_blind.py quotes "Modified " in prose.
    check('Modified 1 footprint positions' in txt,
          'one placement over two same-named blocks reports 1',
          repr([l for l in txt.splitlines() if l.startswith('Modified')]))
    check('Modified 2 footprint positions' not in txt,
          'and never 2')


# --- 5: an unmatched placement is reported -----------------------------------

def t_a_placement_matching_no_block_is_reported_not_dropped():
    src = _board(footprint_text('R1', 1, 1, uuid='a'))
    before = _poses(src)
    out, txt = _write(src, [{'reference': 'NOPE7', 'new_x': 9.0, 'new_y': 9.0,
                             'new_rotation': 0.0}])
    check(_moved(before, _poses(out)) == [],
          'a placement for an absent reference moves nothing')
    check("'NOPE7'" in txt and 'matched no footprint block' in txt,
          'and the writer SAYS so rather than dropping it', repr(txt))


def t_a_bare_name_no_longer_reaches_the_ordinal_twin():
    """The old spelling is now a miss, and a LOUD one.

    A caller that hardcodes `TP4` on a board with two of them used to move
    both. It now moves one -- and a caller that names something the board does
    not have gets told.
    """
    src = _board(footprint_text('U7', 10, 10, uuid='a'),
                 footprint_text('U7', 30, 10, uuid='b'))
    out, txt = _write(src, [{'reference': 'U7', 'new_x': 1.0, 'new_y': 1.0,
                             'new_rotation': 0.0}])
    check('matched no footprint block' not in txt,
          'the bare name still resolves (to the FIRST block)', repr(txt))
    out2, txt2 = _write(src, [{'reference': 'U7' + DUP_REF_SEP + '9',
                               'new_x': 1.0, 'new_y': 1.0,
                               'new_rotation': 0.0}])
    check('matched no footprint block' in txt2,
          'an ordinal the board does not have is reported, not silently '
          'dropped', repr(txt2))


# --- 6: the reference-less block ---------------------------------------------

def t_a_referenceless_block_is_addressable():
    """`#uuid` keys are handed out by the parser; the writer can now act on one.

    Before #726 `write_placed_output` required a `(property "Reference" ...)`
    node and skipped everything else, so a placement for one of esp_prog's
    three reference-less logo blocks wrote nothing and still returned True --
    the placer's model and the file disagreed and neither said so.
    """
    src = _board(footprint_text(None, 1, 1, uuid='u1'),
                 footprint_text(None, 2, 2, uuid='u2'),
                 footprint_text('R1', 3, 3, uuid='u3'))
    before = _blocks_by_key(src)
    out, txt = _write(src, [{'reference': '#u1', 'new_x': 40.0, 'new_y': 40.0,
                             'new_rotation': 0.0}])
    after = _blocks_by_key(out)
    check(_poses(out)['#u1'] == (40.0, 40.0, 0.0),
          'a #uuid-keyed placement moves its block', str(_poses(out)['#u1']))
    check(after['#u2'] == before['#u2'] and after['R1'] == before['R1'],
          'and nothing else')
    check('matched no footprint block' not in txt,
          'without a warning, because it DID match')


# --- the sibling writers -----------------------------------------------------

def t_stamp_locked_locks_one_block():
    """`seeder.stamp_locked` used to stamp every block sharing the name."""
    src = _board(footprint_text('TP4', 10, 10, uuid='a'),
                 footprint_text('TP4', 30, 10, uuid='b'))
    n = stamp_locked(src, ['TP4'])
    blocks = _blocks_by_key(src)
    locked = {k: '(locked yes)' in t for k, t in blocks.items()}
    check(n == 1, 'stamp_locked reports one block locked', str(n))
    check(locked['TP4'] and not locked['TP4' + DUP_REF_SEP + '2'],
          'and it is the one that was named', str(locked))


def t_the_label_writer_edits_one_block():
    """`write_label_output` carried the identical defect."""
    src = _board(footprint_text('TP4', 10, 10, uuid='a'),
                 footprint_text('TP4', 30, 10, uuid='b'))
    before = _blocks_by_key(src)
    res = LabelResult(reference='TP4', action='placed', at_x=1.5, at_y=-2.5,
                      file_rotation=0.0, size=0.8, thickness=0.12)
    out = _out()
    buf = io.StringIO()
    with redirect_stdout(buf):
        write_label_output(src, out, [res])
    after = _blocks_by_key(out)
    check('Modified 1 reference label(s)' in buf.getvalue(),
          'one label result edits one block',
          repr([l for l in buf.getvalue().splitlines()
                if l.startswith('Modified')]))
    check(after['TP4'] != before['TP4'], 'the named block changed')
    check(after['TP4' + DUP_REF_SEP + '2'] == before['TP4' + DUP_REF_SEP + '2'],
          'the TWIN block is BYTE-IDENTICAL')


TESTS = (
    t_one_placement_moves_exactly_one_block,
    t_the_other_twin_is_addressable_too,
    t_the_twins_are_independent_in_one_call,
    t_an_identity_write_moves_nothing,
    t_a_zero_placement_write_changes_only_the_copper_text_pass,
    t_the_modified_count_is_placements_not_blocks,
    t_a_placement_matching_no_block_is_reported_not_dropped,
    t_a_bare_name_no_longer_reaches_the_ordinal_twin,
    t_a_referenceless_block_is_addressable,
    t_stamp_locked_locks_one_block,
    t_the_label_writer_edits_one_block,
)


def main():
    try:
        for t in TESTS:
            print('\n%s' % t.__name__)
            t()
    finally:
        shutil.rmtree(_TMPDIR, ignore_errors=True)
    print('\n%d arm(s), %d failure(s)' % (len(TESTS), len(FAILURES)))
    for f in FAILURES:
        print('  FAILED: %s' % f)
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
