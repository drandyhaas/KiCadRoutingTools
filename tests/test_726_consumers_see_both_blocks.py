#!/usr/bin/env python3
"""#726 consumers: a check cannot see a part the parser dropped.

`check_assembly`'s `coincident_origins` exists to catch two parts sitting at
one point. It read **0** on run 20's board while TWO coincident pairs sat on
it, because `pcb.footprints` was keyed by reference and each pair was a single
dict entry. The check that exists for exactly this case was structurally blind
to it -- and said so in its own comment, which is the whole reason this file
starts there.

HOW THE HOLE SURVIVED. `check_assembly.py` already read
`getattr(pcb, 'duplicate_references', None)` (added by the #628 PR), printed an
advisory from it and emitted it in its JSON. `PCBData` never had that attribute,
so `getattr` returned `None` every time, the advisory never once fired, and
`footprint_blocks` silently reported the PART count while claiming to report
the BLOCK count. Dead code written against a shape that did not exist is
invisible to a test suite: nothing fails, nothing prints, nothing is wrong
enough to notice.

FIVE THINGS THIS FILE PINS:

1. **`footprint_blocks` is RE-DERIVED**, not read back. Its old formula was
   `len(footprints) + sum(dup.values()) - len(dup)`, which reconstructed the
   block total from a dict of survivors. That formula is now wrong in the other
   direction -- on watchy it computes 86 + 4 - 2 = **88** for a board with 86
   blocks -- so an arm that trusted the field would pass while the JSON shipped
   a wrong number.

2. **The isolating fixture is `U7`/`U7` against `U7`/`U8`** -- the same two
   parts, the same point, the same classes, differing only in the NAME. A
   fixture built from `TP*` refs would "prove" the blindness for the wrong
   reason: `coincident_origins` exempts marker classes (fiducial, mount_hole,
   testpoint) BY DESIGN, so a TP-only stack is invisible whatever the keying
   does. That correction cost a round the first time this was written.

3. **The side maps agree with the dict.** `placement/parser.py` re-reads the
   board and builds its OWN reference-keyed courtyard and fab maps, and
   `placement/labels.py` looks a courtyard up as `courtyard_sides.get(
   fp.reference)`. A `TP4~2` against a `TP4`-keyed map finds nothing and falls
   back silently to the pad bbox -- a third, independent collapse hiding behind
   the first two.

4. **`(locked yes)` is per BLOCK, not per name.** glasgow's seven `REF**` are
   3 locked logos and 4 unlocked `kikit:Tab` panel tabs. They used to be one
   set entry.

5. **And the consequence of 4 is measured, not assumed.** Making the tabs
   visible and correctly-unlocked does NOT make them movable: `quench.py`
   admits a zero-pad footprint to `state.parts` only with a drawn courtyard,
   and all seven are zero-pad with none. This arm exists because the obvious
   worry ("four panel tabs just became movable") is wrong, and a reader who
   assumes it will go looking for a lock that should not be added.

    python3 -X utf8 tests/test_726_consumers_see_both_blocks.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

RUN_ALL_TIMEOUT = 900

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_utils import check as run_check, evidence, tool, tool_env  # noqa: E402
from synth import board_text, footprint_text, write_board        # noqa: E402
from kicad_parser import (parse_kicad_pcb, find_matching_paren,   # noqa: E402
                          DUP_REF_SEP)
from placement.parser import (extract_locked_refs,               # noqa: E402
                              extract_courtyard_sides,
                              extract_fab_sides)

FAILURES = []
_TMPDIR = tempfile.mkdtemp(prefix='t726c_')


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + detail) if detail else ''))
        FAILURES.append(what)


def _board_path(name):
    return os.path.join(_ROOT, 'kicad_files', name + '.kicad_pcb')


def _grep_block_count(path):
    """The block count by a method the parser does not use."""
    with open(path, encoding='utf-8', errors='replace') as fh:
        content = fh.read()
    return len(re.findall(r'\(footprint\s+"', content))


def _assembly_json(board, args=(), refuse=None):
    """Run check_assembly and read its JSON.

    `refuse` names the REASON a non-zero exit is expected for -- a stacked
    fixture is legitimately NOT BUILDABLE (exit 4), and asserting only the code
    would let an ImportError or an argparse accident read as the finding.
    `run_utils.check` reports those as a BROKEN TEST instead.
    """
    out = os.path.join(_TMPDIR, os.path.basename(board) + '.json')
    argv = [sys.executable, '-X', 'utf8', tool('check_assembly.py'), board,
            '--json', out] + list(args)
    if refuse is None:
        run_check(argv, accept=True, timeout=600)
    else:
        run_check(argv, refuse=refuse, code=4, timeout=600)
    evidence(out, 'check_assembly JSON')
    with open(out, encoding='utf-8') as fh:
        return json.load(fh)


# --- 1. footprint_blocks -----------------------------------------------------

def t_footprint_blocks_is_the_block_count_re_derived():
    for name, in (('watchy',), ('esp_prog',), ('glasgow_revC',), ('tigard',)):
        b = _board_path(name)
        doc = _assembly_json(b)
        want = _grep_block_count(b)
        check(doc['footprint_blocks'] == want,
              '%s: footprint_blocks == the file\'s own block count (%d)'
              % (name, want), 'JSON says %s' % doc['footprint_blocks'])
        # The OLD formula, computed here so the arm cannot silently pass if
        # someone restores it: on watchy it gives 88 for an 86-block board.
        old = (len(parse_kicad_pcb(b).footprints)
               + sum(doc['duplicate_references'].values())
               - len(doc['duplicate_references']))
        if doc['duplicate_references']:
            check(old != want,
                  '%s: and the OLD formula would have said %d, not %d'
                  % (name, old, want))


def t_duplicate_references_reaches_the_json():
    for name, want in (('watchy', {'TP4': 2, 'TP5': 2}),
                       ('esp_prog', {'Ref*': 2}),
                       ('glasgow_revC', {'REF**': 7}),
                       ('tigard', {})):
        doc = _assembly_json(_board_path(name))
        check(doc['duplicate_references'] == want,
              '%s: duplicate_references == %s' % (name, want),
              str(doc['duplicate_references']))


def t_the_stale_explanation_is_gone():
    """A wrong explanation that survives a fix ships green forever."""
    src = os.path.join(_ROOT, 'py_tools', 'check_assembly.py')
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    for dead in ('Only the LAST block of each is parsed',
                 'cannot form a coincident pair',
                 'are a single '):
        check(dead not in text,
              'check_assembly no longer claims %r' % dead[:40])
    doc = _assembly_json(_board_path('watchy'))
    check('every footprint BLOCK' in doc['coincident_origins_basis'],
          'and the JSON basis states the new one',
          doc['coincident_origins_basis'][:80])


# --- 2. the isolating fixture ------------------------------------------------

def _coincident_pair(name_a, name_b):
    """Two pad-bearing parts at ONE point, named as asked."""
    p = write_board(board_text(
        footprint_text(name_a, 40, 40, uuid='p1', name='test:SOIC8')
        + footprint_text(name_b, 40, 40, uuid='p2', name='test:SOIC8')
        + footprint_text('R1', 10, 10, uuid='p3')),
        os.path.join(_TMPDIR, 'coin_%s_%s.kicad_pcb' % (name_a, name_b)))
    return p


def t_two_parts_at_one_point_are_seen_even_sharing_a_name():
    distinct = _assembly_json(_coincident_pair('U7', 'U8'),
                              refuse='COINCIDENT ORIGINS')
    same = _assembly_json(_coincident_pair('U7', 'U7'),
                          refuse='COINCIDENT ORIGINS')
    check(distinct['coincident_origins'] == 1,
          'U7 + U8 stacked at one point: caught (the positive control)',
          str(distinct['coincident_origins']))
    check(same['coincident_origins'] == 1,
          'U7 + U7 stacked at one point: caught TOO -- this read 0 before '
          '#726, on the same two parts at the same point',
          str(same['coincident_origins']))
    check(same['duplicate_references'] == {'U7': 2},
          'and the naming is reported separately',
          str(same['duplicate_references']))
    # Why the fixture is not built from TP*: coincident_origins exempts marker
    # classes by design, so a TP-only stack proves nothing about the keying.
    # Not buildable either, but for a DIFFERENT reason -- the pad-intersection
    # channel. `coincident_origins` waives it as `marker_class`, which is the
    # whole point: a TP-only fixture reads 0 here whatever the keying does.
    markers = _assembly_json(_coincident_pair('TP1', 'TP2'),
                             refuse='pad_intersection')
    check(markers['coincident_origins'] == 0,
          'a TESTPOINT stack is exempt from coincident_origins by design -- '
          'which is why the fixture above uses U7/U8 and not TP4/TP5',
          str(markers['coincident_origins']))


# --- 3. the side maps --------------------------------------------------------

def t_the_side_maps_are_keyed_like_the_dict():
    for name in ('watchy', 'glasgow_revC', 'esp_prog', 'ulx3s',
                 'orangecrab_ext_pll', 'tigard'):
        b = _board_path(name)
        pcb = parse_kicad_pcb(b)
        keys = set(pcb.footprints)
        with open(b, encoding='utf-8', errors='replace') as fh:
            content = fh.read()
        cy = extract_courtyard_sides(b)
        fb = extract_fab_sides(b)
        stray = (set(cy) | set(fb)) - keys
        check(not stray,
              '%s: courtyard/fab maps use only keys the dict has' % name,
              str(sorted(stray)[:5]))
        # And the map REACHES the twins: every block that draws a courtyard
        # must have an entry, including a `~n` one.
        want = set()
        for m in re.finditer(r'\(footprint\s+"', content):
            s = m.start()
            t = content[s:find_matching_paren(content, s)]
            if '.CrtYd"' in t:
                want.add(s)
        check(len(cy) == len(want),
              '%s: one courtyard entry per block that draws one (%d)'
              % (name, len(want)), 'map has %d' % len(cy))


# --- 4 & 5. locked is per block, and what that does NOT mean -----------------

def t_locked_is_per_block_not_per_name():
    b = _board_path('glasgow_revC')
    locked = extract_locked_refs(b)
    ref_keys = sorted(k for k in parse_kicad_pcb(b).footprints
                      if k.startswith('REF**'))
    got = sorted(k for k in locked if k.startswith('REF**'))
    check(len(ref_keys) == 7,
          'glasgow carries seven REF** blocks', str(ref_keys))
    check(got == ['REF**', 'REF**' + DUP_REF_SEP + '2',
                  'REF**' + DUP_REF_SEP + '7'],
          'exactly the three LOCKED ones are locked -- the four kikit:Tab '
          'panel tabs are not, and used to be one set entry with them',
          str(got))


def t_the_unlocked_tabs_are_still_not_movable():
    """The obvious worry, measured and refuted.

    Four panel tabs becoming visible and correctly-unlocked sounds like four
    parts the placer may now walk around the board. It is not:
    `quench.py:815-825` admits a zero-pad footprint to `state.parts` only when
    the board draws it a courtyard, and `portfolio.free_refs` /
    `portfolio.py:318` filter on `fp.pads` outright. All seven glasgow `REF**`
    are zero-pad with no courtyard. Stated here so a reader does not go and add
    a lock that would be wrong.
    """
    import io
    from contextlib import redirect_stdout
    from pose_score import make_state
    from placement.portfolio import free_refs
    b = _board_path('glasgow_revC')
    pcb = parse_kicad_pcb(b)
    refs = [k for k in pcb.footprints if k.startswith('REF**')]
    check(all(not pcb.footprints[k].pads for k in refs),
          'all seven REF** blocks are zero-pad')
    cy = extract_courtyard_sides(b)
    check(not any(k in cy for k in refs),
          'and none of them draws a courtyard')
    with redirect_stdout(io.StringIO()):
        st = make_state(pcb, b)
        free = set(free_refs(pcb, b))
    inpart = [k for k in refs if k in st.parts]
    check(not inpart,
          'so none of them enters QuenchState.parts -- the quench cannot move '
          'a part it has no entry for', str(inpart))
    check(not (set(refs) & free),
          'and portfolio.free_refs excludes them too',
          str(sorted(set(refs) & free)))


TESTS = (
    t_footprint_blocks_is_the_block_count_re_derived,
    t_duplicate_references_reaches_the_json,
    t_the_stale_explanation_is_gone,
    t_two_parts_at_one_point_are_seen_even_sharing_a_name,
    t_the_side_maps_are_keyed_like_the_dict,
    t_locked_is_per_block_not_per_name,
    t_the_unlocked_tabs_are_still_not_movable,
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
