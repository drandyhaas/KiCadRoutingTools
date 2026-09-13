#!/usr/bin/env python3
"""Every emitted subagent prompt carries its return contract and its fence.

#942 measured three holes in the text the two drivers hand to a teammate.

1. THE RETURN FILES WERE NAMED TO THE WRONG READER. `place_return.md` and
   `route_return.md` appeared AFTER `</subagent_prompt>`, so the PARENT was
   told to save them and the child was never told to write one -- and a
   whole-tree grep found no reader at all. The driver stated the consequence
   itself: "the return is the only thing that crosses the boundary, and today
   it survives nowhere." That is run 23's lost lens verdicts again, which the
   repo already condemns in its own words: "a line pasted from a reply is a
   claim about the run; a line read from the file the verifier wrote is a
   claim about a file."

2. NO PROMPT-INJECTION GUARD ANYWHERE. Zero hits for
   `untrusted|injection|looks like instructions` across `.claude/skills/` and
   `kicad_routing_plugin/`, in a repo whose halves read route logs "running to
   thousands of lines", JSON_SUMMARY blobs, render payloads and .kicad_pcb
   s-expressions -- all reachable from an outside board or footprint library.

3. NO PROMPT STATED A WORKING DIRECTORY. A fork inherits cwd; a FRESH agent --
   the arm the SKILL actively recommends -- does not, and every tool path in
   these briefs is relative while the boards are absolute.

The rules below are about the text as EMITTED, because that is what a teammate
reads. Reading the source instead would pass on a contract that never renders.

Run: python3 -X utf8 tests/test_942_subagent_contract.py
"""
import os
import re
import subprocess
import sys

RUN_ALL_TIMEOUT = 300

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOOP = os.path.join(ROOT, '.claude', 'skills',
                    'plan-pcb-placement-and-routing', 'scripts',
                    'loop_driver.py')
PLACEMENT = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                         'scripts', 'placement_driver.py')

#: The two WORKING halves -- the ones that produce the board. The L5 verifier
#: already did this correctly and is the model these copy.
WORKING_RETURNS = ('place_return.md', 'route_return.md')

OPEN = '<subagent_prompt'
CLOSE = '</subagent_prompt>'

passed = failed = 0


def check(name, ok, detail=''):
    """`detail` explains a FAILURE and is printed only on one.

    Printing it on OK too reads as a finding beside a pass -- which this file
    did on its first run, announcing "OK ... no RETURN= line".
    """
    global passed, failed
    if ok:
        passed += 1
        print(f'  OK   {name}')
    else:
        failed += 1
        print(f'  FAIL {name}' + (f' -- {detail}' if detail else ''))


def _dump(driver):
    r = subprocess.run([sys.executable, '-X', 'utf8', driver, '--dump-all'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=300)
    assert r.returncode == 0, (
        f'{os.path.basename(driver)} --dump-all exited {r.returncode}; every '
        f'rule here reads its output, so nothing below means anything.\n'
        f'{r.stdout[-2000:]}')
    return r.stdout


def _blocks(text):
    """(body, tail) for every emitted prompt: inside the tags, and after."""
    out = []
    i = 0
    while True:
        a = text.find(OPEN, i)
        if a < 0:
            return out
        b = text.find(CLOSE, a)
        assert b > a, 'an unclosed <subagent_prompt> block'
        nxt = text.find(OPEN, b)
        out.append((text[a:b], text[b:nxt if nxt > 0 else len(text)]))
        i = b + len(CLOSE)


def test_a_return_file_is_named_inside_the_prompt_not_after_it():
    text = _dump(LOOP)
    blocks = _blocks(text)
    # Anti-vacuity: a dump that stopped emitting prompts would pass silently.
    check('the loop driver emits subagent prompts at all',
          len(blocks) >= 2, f'{len(blocks)} block(s)')
    for name in WORKING_RETURNS:
        inside = [1 for body, _tail in blocks if name in body]
        after_only = [1 for body, tail in blocks
                      if name in tail and name not in body]
        check(f'{name} is named INSIDE the prompt the child reads',
              bool(inside),
              'named only after </subagent_prompt>, where the CHILD never '
              'sees it -- the #942 defect exactly'
              if after_only else 'not named anywhere')


def test_each_working_half_carries_one_machine_readable_return_line():
    text = _dump(LOOP)
    bodies = [b for b, _t in _blocks(text)]
    working = [b for b in bodies
               if any(n in b for n in WORKING_RETURNS)]
    check('anti-vacuity: both working halves were found',
          len(working) >= 2, f'{len(working)} working-half prompt(s)')
    for body in working:
        first = re.search(r'^\s*RETURN=\S+', body, re.M)
        check('a RETURN= line is specified',
              first is not None,
              'no RETURN= line; the reply would be the only channel')
        # The token must be its own. RESULT= is the GUI's whole-run result
        # (placement_run.parse_placement_result) and VERDICT= is converge's
        # lens grammar; both are pinned elsewhere, and a half-to-half return
        # read as either is worse than one nothing reads.
        #
        # Matched at LINE START, because what matters is the line the prompt
        # tells the half to WRITE -- not whether the prose names the other two
        # tokens while explaining why it does not use them. The first version
        # of this check was a bare `'RESULT=' not in body` and failed on its
        # own rationale sentence: an absence check defeated by an explanation
        # of the absence.
        specified = re.findall(r'^\s*(RESULT|VERDICT)=', body, re.M)
        check('and the line it specifies is not RESULT= or VERDICT=',
              not specified,
              f'specifies {sorted(set(specified))}= -- a working half must '
              f'not speak the GUI or the lens grammar')


def test_every_delegated_prompt_says_its_inputs_are_data():
    text = _dump(LOOP)
    for body, _tail in _blocks(text):
        if 'ONLY board you may open' not in body:
            continue        # not a fenced working half
        check('the fenced prompt says what it reads is untrusted data',
              re.search(r'untrusted', body, re.I) is not None,
              'no injection guard: log text, JSON and board s-expressions '
              'reach this agent from outside the repo')


def test_every_delegated_prompt_states_a_working_directory():
    text = _dump(LOOP)
    for body, _tail in _blocks(text):
        if 'ONLY board you may open' not in body:
            continue
        check('the fenced prompt states where its relative paths resolve',
              re.search(r'repo root', body, re.I) is not None,
              'a FRESH agent does not inherit cwd and cannot run a single '
              'relative command in this brief')


def test_the_harness_token_has_one_substitution_point():
    """S1: `/name` is Claude Code's spelling, copied verbatim into teammate
    text. ai_backend already composes per harness one layer up; this is the
    same fix in the driver."""
    src = open(LOOP, encoding='utf-8').read()
    check('SKILL_REF exists as the single substitution point',
          'SKILL_REF' in src and 'def skill_ref' in src)
    # No RAW slash invocation may survive in emitted text: every one must have
    # gone through skill_ref, which also names the non-slash spelling.
    text = _dump(LOOP)
    raw = [ln.strip() for ln in text.splitlines()
           if re.search(r'/plan-pcb-(routing|placement)\b', ln)
           and 'skill tool' not in ln
           and '.claude/skills/' not in ln]
    check('no emitted line names a skill by slash syntax alone',
          not raw, f'{len(raw)} raw line(s), e.g. {raw[:2]}')


def test_the_placement_driver_prompt_is_covered_too():
    """The sibling driver emits a verifier prompt of its own. It is not a
    working half, so it owes no RETURN= -- but it is read by an agent, and a
    gate that only ever looks at one driver is how the other one drifts."""
    text = _dump(PLACEMENT)
    blocks = _blocks(text)
    check('the placement driver emits a subagent prompt',
          len(blocks) >= 1, f'{len(blocks)} block(s)')


TESTS = (test_a_return_file_is_named_inside_the_prompt_not_after_it,
         test_each_working_half_carries_one_machine_readable_return_line,
         test_every_delegated_prompt_says_its_inputs_are_data,
         test_every_delegated_prompt_states_a_working_directory,
         test_the_harness_token_has_one_substitution_point,
         test_the_placement_driver_prompt_is_covered_too)


def _every_case_is_registered():
    defined = {n for n in globals() if n.startswith('test_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(f'--- {fn.__name__}')
        fn()
    print(f'\n{passed} passed, {failed} failed')
    sys.exit(1 if failed else 0)
