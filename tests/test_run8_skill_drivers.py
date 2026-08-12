#!/usr/bin/env python3
"""The drivers ARE the workflow; the skill files are their reference.

A skill document is read all at once, which is how a 4,900-line one produced
executors that skimmed the gate and improvised the ladder. The driver is the
tape head instead: one stage per invocation, and a guard that WITHHOLDS the
next stage until the evidence the previous one owed actually exists.

The refusal is the mechanism. A gate written in prose is a sentence someone
skims; a gate that will not print the next instructions cannot be skimmed past.

This test runs the driver's own --self-test (33 checks: every stage emits a
tagged block, says what comes next, stays under 80 lines, no hedging phrases,
every guard refuses without its evidence and proceeds with it) and pins the
contract the skill file promises.

Run: python3 -X utf8 tests/test_run8_placement_driver.py
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                      'scripts', 'placement_driver.py')
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                     'SKILL.md')
FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def run(args):
    p = subprocess.run([sys.executable, '-X', 'utf8', DRIVER] + args,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


ROUTING_SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing',
                             'SKILL.md')






LOOP_DRIVER = os.path.join(ROOT, '.claude', 'skills',
                           'plan-pcb-placement-and-routing', 'scripts',
                           'loop_driver.py')
LOOP_SKILL = os.path.join(ROOT, '.claude', 'skills',
                          'plan-pcb-placement-and-routing', 'SKILL.md')


def run_loop(args):
    p = subprocess.run([sys.executable, '-X', 'utf8', LOOP_DRIVER] + args,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def test_loop_driver():
    """The loop BETWEEN the halves -- the one that was prose until run 8."""
    check('the loop driver ships with its skill', os.path.isfile(LOOP_DRIVER))
    code, out = run_loop(['--self-test'])
    check('its self-test passes', code == 0 and out.strip().endswith('OK'),
          out[-400:])

    code, out = run_loop(['--stage', 'L2', '--board', 'b.kicad_pcb'])
    check('routing refuses to start without a placement close-out', code == 4)
    check('...and says how to produce one', 'check_assembly' in out, out[:300])

    # The other end of the same asymmetry: L2 refuses to START routing without
    # a placement close-out, and until run 13 nothing refused to FINISH. A run
    # reached the terminal artifact having never entered the routing half's own
    # V1-V5 loop, and shipped a board carrying a power-to-signal short.
    with tempfile.TemporaryDirectory() as td:
        bd = os.path.join(td, 'b.kicad_pcb')
        open(bd, 'w', encoding='utf-8').close()
        lp = os.path.join(td, 'l.jsonl')
        # The rows carry the board's REAL content hash. L5 now checks that the
        # board it is about to ship is in the ledger -- on a clean route the
        # loop goes L2 -> L5 with no L3 between, so that was the one path where
        # nothing verified the routing half had recorded anything. A fixture
        # with no result_sha is a fixture no real ledger produces.
        import hashlib
        _sha = hashlib.sha256(open(bd, 'rb').read()).hexdigest()
        rows = ([{'kind': 'placement', 'accepted': True, 'result_sha': _sha,
                  'score': {'blocking': 0, 'quality': {}}}] * 6
                + [{'kind': 'completion', 'accepted': True, 'result_sha': _sha,
                    'score': {'blocking': 0, 'quality': {}}}] * 6)
        with open(lp, 'w', encoding='utf-8') as fh:
            for i, r in enumerate(rows):
                fh.write(json.dumps(dict(r, iteration=i)) + '\n')
        sp = os.path.join(td, 's.json')
        with open(sp, 'w', encoding='utf-8') as fh:
            json.dump({'blocking': 0}, fh)
        code, out = run_loop(['--stage', 'L5', '--board', bd,
                              '--ledger', lp, '--score', sp])
        check('closing out refuses without a routing close-out', code == 4,
              out[:300])
        check('...and says which command produces one',
              'check_complete' in out and '--authored-from' in out, out[:400])

    code, out = run_loop(['--stage', 'L4', '--board', 'b.kicad_pcb'])
    check('a re-entry refuses without a measured shape', code == 4)
    check('...and names the asymmetry that makes it matter',
          'wastes' in out or 'throws away' in out or 'no parameter can fix' in out,
          out[:400])

    code, out = run_loop(['--stage', 'L4', '--board', 'b.kicad_pcb',
                          '--shape', 'placement'])
    check('a placement-shaped re-entry marks routed boards stale',
          code == 0 and 'stale' in out, out[:300])

    code, out = run_loop(['--stage', 'L1', '--board', 'b.kicad_pcb',
                          '--delegate'])
    check('delegation dispatches a TEAMMATE, and names the agent-type rule',
          'TEAMMATE' in out and 'Agent tool' in out, out[:400])
    # The retired claim: "a subagent cannot spawn one" is false in this harness
    # (`claude` and `general-purpose` carry the Agent tool; `Explore` and `Plan`
    # do not), so the rule is about the TYPE, not about subagents as such.
    check('...and no longer claims a subagent cannot spawn',
          'cannot spawn one' not in out, out[:400])

    # Delegation is the DEFAULT now, at every size -- run 14 ran both halves
    # inline at 191 parts / 150 nets and the routing half then absorbed the
    # outer loop's classify stage.
    code, out = run_loop(['--stage', 'L1', '--board', 'b.kicad_pcb'])
    check('both halves delegate by default, with no flag',
          'DELEGATING:' in out and '<subagent_prompt' in out, out[:400])
    code, out = run_loop(['--stage', 'L1', '--board', 'b.kicad_pcb',
                          '--no-delegate'])
    check('--no-delegate is still the escape hatch',
          'INLINE:' in out and '<subagent_prompt' not in out, out[:400])

    text = open(LOOP_SKILL, encoding='utf-8').read()
    check('the combined skill points at its driver',
          'loop_driver.py' in text and '--stage L1' in text)
    check('...and says delegation is now a correctness decision',
          'CORRECTNESS one' in text)


def main():
    check('the driver ships with the skill', os.path.isfile(DRIVER))

    code, out = run(['--self-test'])
    check('its own self-test passes', code == 0 and out.strip().endswith('OK'),
          out[-400:])

    code, out = run(['--list'])
    check('it lists its stages', code == 0 and 'P0' in out and 'P-close' in out)

    print('one stage at a time')
    code, out = run(['--stage', 'P0', '--board', 'b.kicad_pcb'])
    check('a stage emits and exits 0', code == 0)
    check('...tagged as instructions for the reader',
          out.startswith('<stage_instructions'))
    check('...naming only its own stage',
          out.count('<stage_instructions') == 1 and 'stage="P0"' in out)
    check('...and short enough to act on', len(out.splitlines()) <= 80,
          str(len(out.splitlines())))

    print('guards withhold, and say what is missing')
    code, out = run(['--stage', 'P4', '--board', 'b.kicad_pcb'])
    check('a stage whose evidence is absent refuses', code == 4, out[:200])
    check('the refusal is tagged', out.startswith('<error>'))
    check('it names what to produce', '--before' in out)

    code, out = run(['--stage', 'P3', '--board', 'b.kicad_pcb'])
    check('reconstruct refuses without the copper-free measurement', code == 4)
    check('...and gives the command that makes it', 'check_drc.py' in out)

    print('the skill file points at the driver')
    text = open(SKILL, encoding='utf-8').read()
    check('the skill tells the reader to drive, not to improvise',
          'placement_driver.py' in text and '--stage P0' in text)
    check('it explains the three tags',
          '<stage_instructions>' in text and '<subagent_prompt>' in text
          and '<error>' in text)
    check('it says a subagent prompt is NOT the reader\'s instructions',
          'Do NOT read it as your own instructions' in text)
    check('it says an error means a gate is holding',
          'not a malfunction' in text)


    print('the loop driver')
    test_loop_driver()

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
