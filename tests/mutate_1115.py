"""The #1115 mutation battery: a cited command runs as written.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore the defect #1115 measured, or the reason it went unseen:

  * `skill-cites-json-without-a-path` -- the free-agent skill's mode test,
    `board_brief.py <board> --json`, exited 2 as written;
  * `summary-drops-has_copper` -- the skill read `has_copper` off a line that
    never carried it;
  * `arity-reads-every-flag-as-boolean` / `discovery-ignores-bare-spans` --
    test_431 checked a flag's NAME only, and never enrolled board_brief.py.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1115.py
    python3 tests/mutate_1115.py --row skill-cites-json-without-a-path

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending (the skill is CRLF, and every anchor on it is ONE line, so
the translation never has to guess). A witness is `(test file, case-name
substring...)`: test_431 runs only the cases whose names contain one of them;
test_1109 is unittest, so its cases are full `Class.method` names.

Not covered by a row, and why:
  * `_min_values` reading the LAST alias's metavar: on the Python this suite
    runs (3.13) argparse prints the metavar only once, after the last alias,
    so reading the first alias that carries one is the same answer -- an
    equivalent mutant here; it differs only on Python <= 3.12;
  * `_arity`'s per-subcommand `min()`: no discovered tool defines one flag
    with two different arities across subcommands today, so no witness can
    see it;
  * `_min_values`' loop that strips NESTED optional groups: on Python 3.13
    no option head nests one optional group inside another before the
    count, so stripping only the outer level gives the same count -- an
    equivalent mutant here;
  * `_subcommand_names` reading only the positional section's brace group:
    reading an option's choices too re-prints the top-level help, so the
    mutant changes the runtime (130 of 182 `--help` calls), never an answer.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

TARGETS = {
    'fa': os.path.join(_ROOT, '.claude', 'skills', 'pcb-free-agent',
                       'SKILL.md'),
    'bb': os.path.join(_ROOT, 'py_tools', 'board_brief.py'),
    't431': os.path.join(_TESTS, 'test_431_skill_commands.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


T431 = 'test_431_skill_commands.py'
VALUE_SCAN = _t(T431, 'value_scan')
EVERY_VALUE = _t(T431, 'every_value_flag')
T1109 = 'test_1109_pile_and_film.py'
RING = _t(T1109, 'TestPilePredicate.test_board_brief_publishes_pile_on_a_staging_ring')
COPPER = _t(T1109, 'TestPilePredicate.test_board_brief_publishes_has_copper')
PLACED = _t(T1109, 'TestPilePredicate.test_a_placed_board_is_not_a_pile')

# (name, target, old, new, tests, expect)
ROWS = [
    # -- the skill: the command as the executor runs it ----------------------
    ('skill-cites-json-without-a-path', 'fa',
     "`python3 -X utf8 py_tools/board_brief.py <board> --json wk/<run>/brief.json`:",
     "`board_brief.py <board> --json`:",
     (EVERY_VALUE,), 'KILLED'),
    ('pointer-row-drops-the-path', 'fa',
     "| read the board | `py_tools/board_brief.py <board> --json <out>`, ",
     "| read the board | `py_tools/board_brief.py <board> --json`, ",
     (EVERY_VALUE,), 'KILLED'),
    # -- board_brief: the line the skill reads -------------------------------
    ('summary-drops-has_copper', 'bb',
     "         'has_copper': (brief.get('state') or {}).get('has_copper'),",
     "",
     (RING, COPPER, PLACED), 'KILLED'),
    ('summary-has_copper-is-a-constant', 'bb',
     "         'has_copper': (brief.get('state') or {}).get('has_copper'),",
     "         'has_copper': False,",
     (COPPER,), 'KILLED'),
    ('text-state-ignores-pile', 'bb',
     "                'PILE' if st.get('pile') else",
     "                'PILE' if False else",
     (RING,), 'KILLED'),
    # -- test_431: the arity gate --------------------------------------------
    ('arity-reads-every-flag-as-boolean', 't431',
     "    return len([t for t in tail.replace('...', ' ').split() if t])",
     "    return 0",
     (VALUE_SCAN,), 'KILLED'),
    ('missing-values-never-reported', 't431',
     "            if got < need:",
     "            if False:",
     (VALUE_SCAN,), 'KILLED'),
    ('invocation-needs-python', 't431',
     "                    if ran or (nxt and _is_value(nxt)):",
     "                    if ran:",
     (VALUE_SCAN,), 'KILLED'),
    ('pointer-read-as-invocation', 't431',
     "                    if ran or (nxt and _is_value(nxt)):",
     "                    if True:",
     (VALUE_SCAN,), 'KILLED'),
    ('discovery-ignores-bare-spans', 't431',
     "    bare = [m.group(1) for m in _BARE_SPAN_RE.finditer(text)]",
     "    bare = []",
     (VALUE_SCAN,), 'KILLED'),
    ('payload-blanked', 't431',
     "        return m.group(0) if '.py' in m.group(2) else ' %s ' % _QUOTED",
     "        return m.group(0) if '.py' in m.group(2) else ' '",
     (VALUE_SCAN,), 'KILLED'),
    ('quotes-pair-across-spans', 't431',
     "    block = _ROUTE_ARGS_RE.sub(' --route-args %s ' % _QUOTED, block)\n"
     "    spans = re.findall(r'`([^`]+)`', block)",
     "    block = _ROUTE_ARGS_RE.sub(' --route-args %s ' % _QUOTED, block)\n"
     "    block = re.sub(r\"\"\"(['\"])(.*?)\\1\"\"\", ' QUOTED ', block, flags=re.S)\n"
     "    spans = re.findall(r'`([^`]+)`', block)",
     (VALUE_SCAN,), 'KILLED'),
    # -- the verifier's unwitnessed lines (#1115 phase 3) ---------------------
    ('comment-is-a-value', 't431',
     "    if not tok or tok in _SHELL_STOP or tok.startswith('#'):",
     "    if not tok or tok in _SHELL_STOP:",
     (VALUE_SCAN,), 'KILLED'),
    ('python-prefix-ignored', 't431',
     "                    ran = any(re.match(r'python[0-9.]*(\\.exe)?$', t)",
     "                    ran = False and any(re.match(r'python[0-9.]*(\\.exe)?$', t)",
     (VALUE_SCAN,), 'KILLED'),
    ('outside-backticks-unread', 't431',
     "    if '.py' in outside:\n        spans.append(outside)\n\n    def quoted",
     "    if False:\n        spans.append(outside)\n\n    def quoted",
     (VALUE_SCAN,), 'KILLED'),
    ('one-value-is-enough', 't431',
     "            need = arity.get(m.group(1), 0)",
     "            need = min(1, arity.get(m.group(1), 0))",
     (VALUE_SCAN,), 'KILLED'),
    ('subcommand-help-unread', 't431',
     "    texts.extend(_sub_help_text(tool, s) for s in _subcommand_names(text))",
     "    pass",
     (VALUE_SCAN,), 'KILLED'),
    ('usage-synopses-unread', 't431',
     "            if m.group(1) not in out:",
     "            if False:",
     (VALUE_SCAN,), 'KILLED'),
    ('unreadable-help-is-empty', 't431',
     "    if '--help' not in text:\n"
     "        raise RuntimeError(f'{tool} --help produced no option list '",
     "    if False:\n"
     "        raise RuntimeError(f'{tool} --help produced no option list '",
     (VALUE_SCAN,), 'KILLED'),
    ('unbalanced-span-not-joined', 't431',
     "                   or '\\n'.join(cur).count('`') % 2)",
     "                   or False)",
     (VALUE_SCAN,), 'KILLED'),
    ('operator-is-a-value', 't431',
     "    if not tok or tok in _SHELL_STOP or tok.startswith('#'):",
     "    if not tok or tok.startswith('#'):",
     (VALUE_SCAN,), 'KILLED'),
    ('negative-number-is-an-option', 't431',
     "    if tok.startswith('-') and not re.match(r'-[0-9.]', tok):",
     "    if tok.startswith('-'):",
     (VALUE_SCAN,), 'KILLED'),
]

sys.path.insert(0, _TESTS)
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _run_tests(tests):
    failed = []
    for t in tests:
        p = subprocess.run([sys.executable, '-X', 'utf8', t[0]] + list(t[1:]),
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=2400, cwd=_ROOT)
        if p.returncode != 0:
            failed.append((os.path.basename(t[0]) + ':' + ','.join(t[1:]),
                           p.returncode,
                           [ln.strip()[:90] for ln in
                            ((p.stdout or '') + (p.stderr or '')).splitlines()
                            if 'FAIL' in ln or 'Error' in ln][:2]))
    return failed


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit or stash '
                  'first -- this battery restores by overwriting.'
                  % os.path.basename(path))
            return 2
    # THE UNMUTATED BASELINE: every witness must pass as the code stands,
    # or a row it "kills" proves nothing.
    witnesses = sorted({t for r in rows for t in r[4]})
    base_fail = _run_tests(witnesses)
    if base_fail:
        print('REFUSING: witnesses fail UNMUTATED -- %s' % base_fail)
        return 2
    print('baseline: %d witnesses pass unmutated' % len(witnesses))
    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path = TARGETS[tgt]
            base = orig[tgt]
            o, n = old, new
            if '\r\n' in base:
                o, n = o.replace('\n', '\r\n'), n.replace('\n', '\r\n')
            if base.count(o) != 1 or o == n:
                results.append((name, 'BROKEN', expect,
                                ['anchor matched %d times' % base.count(o)]))
                continue
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(o, n, 1))
            try:
                failed = _run_tests(tests)
            finally:
                io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if failed else 'SURVIVED',
                            expect, [str(f)[:150] for f in failed[:2]]))
            print('%-38s %s' % (name, results[-1][1]), flush=True)
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
    wrong = [r for r in results if r[1] != r[2]]
    print('')
    for name, verdict, expect, why in results:
        print('%-38s %-9s%s' % (name, verdict, '' if verdict == expect else
                                '   <-- WRONG, expected %s' % expect))
        for w in why:
            print('      %s' % w)
    print('\n%d rows: %d killed, %d survived, %d broken'
          % (len(results), sum(r[1] == 'KILLED' for r in results),
             sum(r[1] == 'SURVIVED' for r in results),
             sum(r[1] == 'BROKEN' for r in results)))
    return 1 if wrong else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--row', default=None, help='run only this row')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
