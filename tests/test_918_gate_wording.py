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

   Note what this does NOT flag, and why #918's literal acceptance
   (`grep -rn "blocking == 0"` returns nothing) cannot be met as written: about
   thirty sites in these directories say `blocking == 0` about **board_score's**
   `blocking`, which is a nine-component total and where the phrase is exactly
   right. So the check is scoped: a `blocking == 0` is flagged only when
   `check_assembly` is named within a few lines of it AND the sentence reads as
   a gate.

2. **Every stated `blocking` FORMULA lists the nine components board_score
   sums.** Measured at the time this was written: the combined SKILL.md and
   evidence-map.md each stated seven (no `assembly`, no `net_widths`),
   loop_driver.py stated six in three places, and review-routed-board's sample
   line showed five -- while board_score's own docstring and the 9.1 table were
   right. Three places correct, five wrong.

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

#: Lines within this many of a `blocking == 0` that are read for the words
#: `check_assembly`. Three is enough to span a wrapped sentence and short
#: enough that an unrelated mention two paragraphs away does not fire.
WINDOW = 3

#: A sentence that DECIDES something. A `blocking == 0` in an explanatory
#: aside is not a gate and is not flagged.
GATE_WORDS = re.compile(
    r'\b(until|unless|gate|gates|gated|FAIL|FAILS|refuse|refuses|'
    r'blocks|stop|stops|requires?|must)\b')

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
        for i, line in enumerate(lines):
            if 'blocking == 0' not in line:
                continue
            lo, hi = max(0, i - WINDOW), min(len(lines), i + WINDOW + 1)
            window = '\n'.join(lines[lo:hi])
            if 'check_assembly' not in window:
                continue
            if not GATE_WORDS.search(window):
                continue
            # A window that ALSO names `buildable` is the corrected wording,
            # not a gate: the four fixed sites each keep `blocking == 0` in a
            # clause saying NOT to gate on it, and a checker that cannot tell
            # a prohibition from the thing prohibited would force the fix to
            # delete its own explanation. Naming what was wrong is how the
            # next reader learns why the key changed.
            if 'buildable' in window:
                continue
            hits.append(f'{os.path.relpath(path, ROOT)}:{i + 1}: '
                        f'{line.strip()[:90]}')
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
    * a set that is a STRICT SUBSET of one found within FORMULA_WINDOW lines
      of it is dropped.

    The windows overlap by construction, so one correct nine-name sentence
    also yields truncated four-, five- and six-name matches from every window
    that clips its head or its tail. Reporting one of those as "missing
    net_widths", against a line that says `net_widths` two words later, is a
    false alarm indistinguishable from a real one -- and a checker whose false
    alarms look like its true ones teaches the next person to ignore it.
    A genuinely stale list has no longer neighbour to be a subset of.
    """
    found = []
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
                    if names <= comps and len(names) > len(best):
                        best = names
        if best:
            found.append((i + 1, best))
    for lineno, names in found:
        if any(names < other and abs(ln - lineno) <= FORMULA_WINDOW
               for ln, other in found):
            continue
        yield lineno, names


def test_every_stated_formula_lists_the_nine():
    comps = blocking_components()
    wrong, seen = [], set()
    for path, lines in sources():
        if os.path.abspath(path) == os.path.abspath(BOARD_SCORE):
            continue          # its own docstring is pinned by test_904
        for lineno, names in _stated_formulas(lines, comps):
            # Only a list that is TRYING to be the blocking formula: it must
            # name the two that every stale copy shares (`unrouted`,
            # `broken`) and be long enough to be a claim about the total.
            if not {'unrouted', 'broken'} <= names or len(names) < 4:
                continue
            if names == comps:
                continue
            key = (path, tuple(sorted(comps - names)))
            if key in seen:
                continue          # the rolling windows overlap
            seen.add(key)
            wrong.append(f'{os.path.relpath(path, ROOT)}:{lineno}: missing '
                         f'{sorted(comps - names)}')
    check('every stated `blocking` formula lists all '
          f'{len(comps)} components', not wrong,
          '; '.join(wrong) if wrong else f'components: {sorted(comps)}')


def main():
    for name in sorted(k for k in globals() if k.startswith('test_')):
        print(f'--- {name}')
        globals()[name]()
    print(f'\n{passed} passed, {failed} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
