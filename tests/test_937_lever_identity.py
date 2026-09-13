#!/usr/bin/env python3
"""#937: which LEVER a ledger row pulled, derived rather than guessed.

`loop_driver.py:4-9` records the one documented reason the two drivers are
separate: "placement accepts a lap when the named finding it aimed at is gone,
routing accepts an iteration when `blocking` strictly decreased... The driver
never emits both." That argues for an accept rule stated PER LEVER rather than
per half -- and the first thing such a rule needs is to know which lever a row
pulled.

`_HALF` keys on `kind`, which is the half. `lever` is free prose and cannot be
read by a gate: `run_watch.py` is explicit that it is "NEVER matched", because
a gate that reads a disclosure punishes disclosing. `lever_argv` is the
structured channel, and this pins what it can and cannot answer.

REPORTED, NOT GATED, and these cases say so. Making the accept rule per-lever
moves DONE and STUCK verdicts, and this repo grades a verdict-moving change by
a corpus A/B rather than by reasoning about it.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, ROOT)

import converge as C                                        # noqa: E402

RUN_ALL_TIMEOUT = 120
RUN_ALL_FAST_OK = True


def t_a_recorded_command_yields_its_tool():
    cases = [
        (['python3', '-X', 'utf8', 'py_placer/place_optimize.py', 'b.kicad_pcb'],
         'place_optimize.py'),
        (['python3', 'py_router/route.py', 'a', 'b'], 'route.py'),
        # The tool, not the interpreter: a bare argv[0] of `python3` names the
        # language, which is true of every row and therefore useless.
        (['python3', 'py_tools/render_placement.py', 'b'],
         'render_placement.py'),
        # ...and when no .py token exists at all, the command itself. 31 of
        # the recorded rows in this tree are `bash <script>`, which IS the
        # honest answer for them.
        (['bash', 'wk/run19/arrange.sh'], 'bash'),
        (['cp', 'a.kicad_pcb', 'b.kicad_pcb'], 'cp'),
    ]
    for argv, want in cases:
        got = C.lever_identity({'lever_argv': argv})
        assert got == want, f'{argv} -> {got!r}, expected {want!r}'
    print(f'  PASS: {len(cases)} argv shapes resolve to their tool')


def t_a_row_with_no_command_is_None_not_a_guess():
    """None is a first-class answer. Most rows without a lever_argv have NO
    COMMAND to record -- an L2 freeze stamp, an `--exhausted` declaration, an
    L5 close-out -- and inventing one for them is how a ledger becomes prose.
    `board_store.replay_command` raises for the same reason."""
    for row in ({'lever': 'moved R12 off the edge'},
                {'lever_argv': None},
                {'lever_argv': []},
                {},
                {'lever_argv': 'not a list'}):
        got = C.lever_identity(row)
        assert got is None, f'{row} -> {got!r}, expected None'
    print('  PASS: a row recording no command answers None, never a guess')


def t_the_prose_field_is_never_read():
    """The prose field must not become a second, weaker identity channel: a
    gate that matched on `lever` would punish the row that described itself
    best."""
    rich = {'lever': 'ran place_optimize.py on R12 with --max-displacement 3'}
    assert C.lever_identity(rich) is None, (
        'lever_identity read the free-text `lever` field. That field exists '
        'for a human, and matching on it penalises disclosure.')
    print('  PASS: a richly-worded `lever` still yields no identity')


def t_the_histogram_is_stable_and_counts_only_identified_rows():
    rows = [{'lever_argv': ['python3', 'py_placer/place_seed.py', 'b']},
            {'lever_argv': ['python3', 'py_placer/place_seed.py', 'b']},
            {'lever_argv': ['python3', 'py_router/route.py', 'b']},
            {'lever': 'no argv here'}]
    h = C._lever_histogram(rows)
    assert h == {'place_seed.py': 2, 'route.py': 1}, h
    assert list(h) == ['place_seed.py', 'route.py'], (
        f'the histogram must be ordered for a stable document, got {list(h)}')
    print('  PASS: histogram counts identified rows only, in a stable order')


def t_status_publishes_both_the_histogram_and_what_it_could_not_read():
    """The coverage number has to travel with the histogram. A histogram
    alone reads as complete."""
    import tempfile
    import json
    import subprocess
    with tempfile.TemporaryDirectory() as tmp:
        led = os.path.join(tmp, 'ledger.jsonl')
        with open(led, 'w', encoding='utf-8') as fh:
            for row in ({'iteration': 0, 'kind': 'placement',
                         'lever': 'seeded',
                         'lever_argv': ['python3', 'py_placer/place_seed.py',
                                        'b.kicad_pcb']},
                        {'iteration': 1, 'kind': 'placement',
                         'lever': 'froze the poses'}):
                fh.write(json.dumps(row) + '\n')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_placer', 'converge.py'), 'status',
             '--ledger', led],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=120)
    assert r.returncode == 0, f'status exited {r.returncode}\n{r.stderr[-600:]}'
    doc = json.loads(r.stdout)
    assert doc.get('lever_identities') == {'place_seed.py': 1}, doc.get(
        'lever_identities')
    assert doc.get('unreplayable') == 1, (
        f'unreplayable is {doc.get("unreplayable")!r}; one of the two rows '
        f'records no command and the document must say so')
    print('  PASS: status carries the histogram AND the unreadable count')


TESTS = (t_a_recorded_command_yields_its_tool,
         t_a_row_with_no_command_is_None_not_a_guess,
         t_the_prose_field_is_never_read,
         t_the_histogram_is_stable_and_counts_only_identified_rows,
         t_status_publishes_both_the_histogram_and_what_it_could_not_read)


def _every_case_is_registered():
    defined = {n for n in globals() if n.startswith('t_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(f'--- {fn.__name__}')
        fn()
    print('\nALL PASS')
