#!/usr/bin/env python3
"""What the skills SAY about `blocking` must be what the code does (#918).

Two claims, both re-derived from `board_score.py`'s own source rather than from
a list copied into this file -- a copied list agrees with the code exactly
until the code changes, which is the trap
`tests/test_904_lens_components_cover_blocking.py` exists to avoid and the
reason the stale formula below survived for as long as the term did.

1. **No prose gates on `check_assembly`'s `blocking == 0`.** `check_assembly`
   decides NOT BUILDABLE on FIVE conjuncts and publishes `buildable` so nobody
   has to re-derive them; `blocking` is the first conjunct alone and means
   "pad intersections". A gate written against it passes a board that is
   unbuildable through the other four.

   This IS #918's acceptance, which asks for `grep -rn "blocking == 0"` over
   the three skill dirs to return "nothing **that is a gate**" -- the last
   three words are load-bearing and an earlier draft of this file dropped
   them. Zero occurrences was never the ask, and could not be: of the 30
   occurrences in those directories, 24 are about **board_score's** `blocking`,
   a nine-component total, where the phrase is exactly right. So the check
   resolves each hit's SUBJECT -- a `blocking == 0` is flagged only when
   `check_assembly` is named within a line of it AND the sentence reads as a
   gate.

2. **Every stated `blocking` FORMULA lists the nine components board_score
   sums.** Measured at the time this was written, SEVEN sites were stale: the
   combined SKILL.md and evidence-map.md each stated seven members (no
   `assembly`, no `net_widths`), loop_driver.py stated six in FOUR places, and
   review-routed-board's sample line showed five -- while board_score's own
   docstring and the 9.1 table were right.

Run: python3 -X utf8 tests/test_918_gate_wording.py
"""
import ast
import io
import os
import re
import sys

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 120

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD_SCORE = os.path.join(
    ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing', 'scripts',
    'board_score.py')
SKILL_DIRS = [
    os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement'),
    os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing'),
    os.path.join(ROOT, '.claude', 'skills', 'review-routed-board'),
]

#: Lines within this many of a `blocking == 0` that are read for the word
#: `check_assembly`. ONE: the two have to be in the same sentence for the
#: scalar to be what the sentence gates on, and a wrapped sentence spans two
#: lines. Three was too wide and it showed immediately -- adding an
#: `components.assembly.*` row to evidence-map.md put `check_assembly` three
#: lines from a CORRECT statement about board_score's own `blocking == 0`
#: tie-break, and the checker called it a gate.
WINDOW = 1

#: A sentence that DECIDES something. A `blocking == 0` in an explanatory
#: aside is not a gate and is not flagged.
GATE_WORDS = re.compile(
    r'\b(until|unless|gate|gates|gated|FAIL|FAILS|refuse|refuses|'
    r'blocks|stop|stops|requires?|must)\b')

#: The ONE way to keep `blocking == 0` next to `check_assembly` in a gating
#: sentence: say explicitly that it is not the thing to gate on. The corrected
#: sites do exactly that, because naming what was wrong is how the next reader
#: learns why the key changed -- a checker that forced the fix to delete its
#: own explanation would be trading one silent hazard for another.
#:
#: It is a NARROW literal on purpose. The first version of this rule suppressed
#: any window containing the word `buildable`, which is the single word most
#: likely to appear in the prose being gated: an adversarial review broke it
#: with `"The board is buildable only when check_assembly reports
#: `blocking == 0`"` -- a real gate, silenced -- and again with an unrelated
#: parenthetical on a neighbouring line. Both are in `_self_test` below.
PROHIBITION = re.compile(r'NOT\s+`blocking == 0`|not on that count')

#: A stated formula, in either spelling the repo uses: four or more
#: `+`-joined identifiers (`unrouted + broken + drc + ...`), or the sample
#: BLOCKING= line's `k=v` run (`unrouted=0 broken=0 drc=0 ...`).
#:
#: Read over a ROLLING MULTI-LINE WINDOW, not per line. Every correct copy of
#: this formula in the tree wraps -- nine names do not fit on one line -- so a
#: per-line scan reads the first half of a right answer as a wrong one, and
#: reads a stale list that wraps as nothing at all. The shape to assert is the
#: sentence, not the line.
FORMULA = re.compile(r'\b([a-z_]+(?:\s*\+\s*[a-z_]+){3,})\b')
FORMULA_KV = re.compile(r'\b([a-z_]+=\d+(?:\s+[a-z_]+=\d+){3,})\b')

#: Lines joined before the formula scan. Three spans every wrapped list in the
#: tree today. Windows overlap, so one sentence yields several matches and the
#: BEST (largest) one at each start wins -- see `_stated_formulas`.
FORMULA_WINDOW = 3

passed = 0
failed = 0


def check(name, ok, detail=''):
    global passed, failed
    if ok:
        passed += 1
        print(f'  OK   {name}' + (f' -- {detail}' if detail else ''))
    else:
        failed += 1
        print(f'  FAIL {name}' + (f' -- {detail}' if detail else ''))


def blocking_components():
    """The keys of board_score's `parts` dict, from its source.

    `blocking = sum(c for c in counts if c)` over `parts.values()`, so those
    keys ARE the blocking components. Same derivation as
    `test_904_lens_components_cover_blocking.py`, and for the same reason.
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
        f'expected exactly one `parts = {{...}}` literal in {BOARD_SCORE}, '
        f'found {len(found)}. If board_score was refactored this test has to '
        f'learn the new shape -- it must NEVER fall back to a hard-coded '
        f'list, which is the failure it exists to prevent.')
    return set(found[0])


def sources():
    for d in SKILL_DIRS:
        for base, dirs, files in os.walk(d):
            dirs[:] = [x for x in dirs if x != '__pycache__']
            for f in sorted(files):
                if f.endswith(('.md', '.py')):
                    p = os.path.join(base, f)
                    yield p, io.open(p, encoding='utf-8',
                                     errors='replace').read().splitlines()


def test_no_prose_gates_on_check_assembly_blocking():
    hits = []
    for path, lines in sources():
        hits += gating_sites(lines, os.path.relpath(path, ROOT))
    check('no site gates on check_assembly\'s `blocking == 0`', not hits,
          '; '.join(hits) if hits else
          'checked every .md and .py under the three skill dirs')


def _stated_formulas(lines, comps):
    """(line_no, names) for each blocking-formula-shaped list in `lines`.

    Windows of up to FORMULA_WINDOW lines are joined with a space, after the
    leading comment marker and indentation are stripped, so a list that wraps
    is one sentence to the scanner.

    Two suppressions, both needed, and both learned by getting this wrong:

    * only the LARGEST name set found at each window start is kept; and
    * a match that reaches the END of its joined window is discarded, because
      the window boundary may be what ended it rather than the author.

    The windows overlap by construction, so one correct nine-name sentence
    also yields truncated four-, five- and six-name matches from the windows
    that clip it. Reporting one of those as "missing net_widths", against a
    line that says `net_widths` two words later, is a false alarm
    indistinguishable from a real one -- and a checker whose false alarms look
    like its true ones teaches the next person to ignore it.

    The FIRST attempt suppressed any set that was a subset of one found within
    a few lines. That is wrong in the one direction that matters: a genuinely
    stale five-name list sitting two lines under a correct nine-name one is
    also a subset of it, and vanished. `_self_test` carries that exact text.
    Truncation is a property of the MATCH, not of the neighbourhood, so it is
    tested as one.
    """
    for i in range(len(lines)):
        best = set()
        for w in range(1, FORMULA_WINDOW + 1):
            # Quotes are DELETED, not just the indent stripped. In Python
            # source a wrapped formula is a run of adjacent string literals,
            # so the line boundary reads `floorplan + " "assembly` -- and the
            # `" "` breaks the `+`-chain exactly where the list continues,
            # which is how a nine-name sentence in loop_driver.py measured as
            # a stale five-name one. No component name contains a quote.
            joined = ' '.join(re.sub(r'^\s*#*\s*', '', x).replace('"', '')
                              .replace("'", '') for x in lines[i:i + w])
            for rx, sep in ((FORMULA, '+'), (FORMULA_KV, None)):
                for m in rx.finditer(joined):
                    if sep:
                        names = {t.strip() for t in m.group(1).split(sep)}
                    else:
                        names = {t.split('=')[0] for t in m.group(1).split()}
                    # `len(names & comps)`, NOT `names <= comps`. A stale list
                    # carrying one name that is not a component --
                    # `unrouted + broken + drc + undersized + shorts` -- was
                    # invisible to the subset test, which is the wrong way for
                    # a staleness checker to fail: the more wrong the list,
                    # the less it saw.
                    # A match with nothing but whitespace and `+` after it may
                    # have been CUT OFF by the window boundary rather than by
                    # the author. Such a match is not evidence of anything: it
                    # is the same sentence seen through too small a hole. Skip
                    # it -- a wider window at this start will produce the real
                    # one. The `+` matters: a formula that wraps does so AFTER
                    # its operator, so the joined window ends `... impedance +`
                    # and a plain end-of-string test says the match ended
                    # naturally when the list plainly continues.
                    if (re.fullmatch(r'[\s+]*', joined[m.end():])
                            and i + w < len(lines)):
                        continue
                    if (len(names & comps) >= 4
                            and len(names & comps) > len(best & comps)):
                        best = names
        if best:
            yield i + 1, best


#: A sample `BLOCKING=` line names only the components that GRADED: an
#: ungraded one has `count: None` and board_score's bits line skips it. So a
#: seven-name sample sitting beside `UNGRADED: impedance, length` is correct
#: output, not a stale formula -- and demanding all nine there would teach the
#: opposite of this skill set's own rule, "report as unexamined, never as
#: clean". The exemption is narrow: the missing names must be exactly the ones
#: an adjacent UNGRADED line accounts for.
UNGRADED_LINE = re.compile(r'UNGRADED[^:]*:\s*(.+)')


def _ungraded_nearby(lines, lineno):
    named = set()
    for x in lines[max(0, lineno - 1 - FORMULA_WINDOW):
                   lineno + FORMULA_WINDOW]:
        m = UNGRADED_LINE.search(x)
        if m:
            named |= {t.strip(' `.,') for t in m.group(1).split(',')}
    return named


def stale_formulas(lines, comps, label='<text>'):
    """Every formula-shaped list in `lines` that does not name all of `comps`.

    Shared by the real scan and by `_self_test`, so the rule the self-test
    proves is the rule the tree is graded by -- a self-test over a
    reimplementation proves nothing about the checker.
    """
    out, seen = [], set()
    for lineno, names in _stated_formulas(lines, comps):
        got = names & comps
        # Only a list that is TRYING to be the blocking formula: it must name
        # the two that every stale copy shares (`unrouted`, `broken`) and be
        # long enough to be a claim about the total.
        if not {'unrouted', 'broken'} <= got or len(got) < 4:
            continue
        if got == comps:
            continue
        if (comps - got) <= _ungraded_nearby(lines, lineno):
            continue          # a sample line, with its UNGRADED line beside it
        key = tuple(sorted(comps - got))
        if key in seen:
            continue              # the rolling windows overlap
        seen.add(key)
        out.append(f'{label}:{lineno}: missing {sorted(comps - got)}')
    return out


def gating_sites(lines, label='<text>'):
    """Every `blocking == 0` in `lines` whose SUBJECT is `check_assembly`.

    Shared with `_self_test` for the same reason as `stale_formulas`.
    """
    hits = []
    for i, line in enumerate(lines):
        if 'blocking == 0' not in line:
            continue
        lo, hi = max(0, i - WINDOW), min(len(lines), i + WINDOW + 1)
        window = '\n'.join(lines[lo:hi])
        if 'check_assembly' not in window:
            continue
        if not GATE_WORDS.search(window):
            continue
        if PROHIBITION.search(window):
            continue
        hits.append(f'{label}:{i + 1}: {line.strip()[:90]}')
    return hits


def _self_test():
    """The two rules, proven against texts that must and must not fire.

    Every exploit below broke an earlier version of this file. A checker whose
    own suppressions are untested is a checker that quietly stops checking:
    the `buildable` suppression this replaced passed a real gate the moment
    the word appeared anywhere within three lines of it.
    """
    comps = {'unrouted', 'broken', 'drc', 'undersized', 'floorplan',
             'assembly', 'impedance', 'length', 'net_widths'}
    cases = [
        # (name, text, must_fire, which)
        ('a real gate on check_assembly',
         'Gate: FAIL unless `check_assembly` reports `blocking == 0`.',
         True, 'gate'),
        ('...still caught when the word `buildable` appears nearby',
         'The board is buildable only when `check_assembly` reports\n'
         '`blocking == 0`; the loop must not proceed until that is true.',
         True, 'gate'),
        ('...and when a neighbouring line merely mentions it',
         'Gate: FAIL unless `check_assembly`\'s `blocking == 0`.\n'
         '(A board that passes this is buildable.)',
         True, 'gate'),
        ('the corrected wording, which names what was wrong',
         'FAILS unless `check_assembly` reports `buildable: true`\n'
         '(NOT `blocking == 0` -- that is 1 of its 5 conjuncts).',
         False, 'gate'),
        ('board_score\'s own blocking == 0 is not this rule\'s business',
         '`quality` is a tie-break only, compared once `blocking == 0`.',
         False, 'gate'),
        ('a stale seven-member formula',
         'It is unrouted + broken + drc + undersized + floorplan +\n'
         'impedance + length.',
         True, 'formula'),
        ('...even carrying a name that is not a component at all',
         'It is unrouted + broken + drc + undersized + shorts.',
         True, 'formula'),
        ('...and even sitting two lines under a correct one',
         'blocking is unrouted + broken + drc + undersized + floorplan +\n'
         'assembly + impedance + length + net_widths.\n'
         '\n'
         'Older note: it is unrouted + broken + drc + undersized + floorplan.',
         True, 'formula'),
        ('the correct nine-member formula, however it wraps',
         'It is unrouted + broken + drc + undersized + floorplan +\n'
         'assembly + impedance + length + net_widths.',
         False, 'formula'),
        ('a sample BLOCKING= line omitting exactly its UNGRADED components',
         'BLOCKING=0  (unrouted=0 broken=0 drc=0 undersized=0 floorplan=0 '
         'assembly=0 net_widths=0)\n'
         'UNGRADED (not scored, not passed): impedance, length',
         False, 'formula'),
        ('...but not one omitting a component nothing accounts for',
         'BLOCKING=0  (unrouted=0 broken=0 drc=0 undersized=0 floorplan=0 '
         'assembly=0 net_widths=0)\n'
         'UNGRADED (not scored, not passed): impedance',
         True, 'formula'),
        ('...and not one with no UNGRADED line at all',
         'BLOCKING=0  (unrouted=0 broken=0 drc=0 undersized=0 floorplan=0)',
         True, 'formula'),
    ]
    bad = []
    for name, text, must_fire, which in cases:
        lines = text.splitlines()
        got = (gating_sites(lines) if which == 'gate'
               else stale_formulas(lines, comps))
        if bool(got) != must_fire:
            bad.append(f'{name}: expected '
                       f'{"a finding" if must_fire else "silence"}, got {got}')
    check('the checker fires on what it must and stays silent on what it must not',
          not bad, '; '.join(bad) if bad else f'{len(cases)} cases')


def test_every_stated_formula_lists_the_nine():
    comps = blocking_components()
    wrong = []
    for path, lines in sources():
        if os.path.abspath(path) == os.path.abspath(BOARD_SCORE):
            continue          # its own docstring is pinned by test_904
        wrong += stale_formulas(lines, comps, os.path.relpath(path, ROOT))
    check('every stated `blocking` formula lists all '
          f'{len(comps)} components', not wrong,
          '; '.join(wrong) if wrong else f'components: {sorted(comps)}')


def main():
    # The self-test runs FIRST and unconditionally: if the checker's own rules
    # have stopped discriminating, its verdict about the tree is worthless and
    # a green run would be the most misleading possible outcome.
    print('--- _self_test')
    _self_test()
    for name in sorted(k for k in globals() if k.startswith('test_')):
        print(f'--- {name}')
        globals()[name]()
    print(f'\n{passed} passed, {failed} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
