#!/usr/bin/env python3
"""Mutation battery for tests/test_713_refill_status.py.

A gate that would stay green with the fix reverted is not a gate. Each row
below breaks ONE thing the gate claims to defend; a row that survives is
reported, because a surviving row means the claim next to it is unpinned.

Two hazards this script is built around, both measured previously in this repo:

- **A stale .pyc outlives the mutation.** A size-preserving edit inside one
  second can leave every later import reading the pre-mutation bytecode, so the
  run reports "survived" about code that never loaded. Every row therefore runs
  in a fresh subprocess with PYTHONDONTWRITEBYTECODE=1 and every __pycache__
  removed first.
- **git checkout -- eats uncommitted work.** Nothing here calls git. The file
  is restored from an in-memory copy of its own bytes.
"""
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GATE = os.path.join(ROOT, 'tests', 'test_713_refill_status.py')

#: (label, file, old, new, why the gate must catch it)
ROWS = [
    ('timeout re-reported as a refill failure (THE item)',
     'py_router/kicad_exact_fill.py',
     "'timeout': 'the KiCad refill TIMED OUT',",
     "'timeout': 'the KiCad refill failed',",
     'the misattribution #713 item 4 is about'),

    ('plane_fragility re-derives the reason itself again',
     'py_router/plane_fragility.py',
     "print(f\"Plane fragility: exact fill unavailable ({_st.why()}); \"",
     "print(f\"Plane fragility: exact fill unavailable (the KiCad refill \"\n"
     "                      f\"failed); \"",
     'the caller must not restate a reason it was handed'),

    ('is_timeout stops distinguishing the machine-dependent arm',
     'py_router/kicad_exact_fill.py',
     "        return self.reason == 'timeout'\n\n    def why",
     "        return False\n\n    def why",
     'the one arm that varies with CPU speed must stay nameable'),

    ('a timeout gets memoised as a failure',
     'py_router/kicad_exact_fill.py',
     "        return None, RefillStatus('timeout',\n"
     "                                  f'limit {timeout}s', _dt)",
     "        if _mk is not None:\n            _REFILL_FAILED.add(_mk)\n"
     "        return None, RefillStatus('timeout',\n"
     "                                  f'limit {timeout}s', _dt)",
     'a budget verdict must not be remembered as a board fact'),

    ('refill_failed stops being memoised',
     'py_router/kicad_exact_fill.py',
     "                _REFILL_FAILED.add(_mk)\n            return None, RefillStatus('refill_failed',",
     "                pass\n            return None, RefillStatus('refill_failed',",
     'the pre-existing failure memo must survive this change'),

    ('the wrapper leaks the pair to its 6 call sites',
     'py_router/kicad_exact_fill.py',
     "                             project_from=project_from)[0]",
     "                             project_from=project_from)",
     'refill_islands must keep its original return type'),

    ('a timeout is called uniform, so the strip looks fair',
     'py_placer/plane_score.py',
     "        return self.reason in ('no_bounds', 'no_named_net', 'no_kicad_python')",
     "        return self.reason in ('no_bounds', 'no_named_net', "
     "'no_kicad_python', 'timeout')",
     'item 1 branches on this; a wrong True restores the defect'),

    ('plane_score restates the refill wording instead of forwarding it',
     'py_placer/plane_score.py',
     "        if base is None:\n            return _refill_why(self)",
     "        if base is None:\n            return 'the plane score was "
     "unavailable'",
     'two wordings for one cause drift apart'),

    ('the GUI provider goes back to falling through silently',
     'py_router/kicad_parser.py',
     "            _warn_live_fill_fallback(_st.why())",
     "            pass",
     'a silent stale-clearance fallback is the #627 divergence'),

    ('an empty fill stops being distinguished from a successful one',
     'py_router/plane_fragility.py',
     "            elif not fills:",
     "            elif False:",
     'the sixth outcome must not read as an ordinary success'),

    # The three rows below survived a first battery and were found by an
    # independent adversarial pass, not by this file. They are the shapes an
    # AST "is it called?" check and a stubbed-return arm cannot see, so they
    # are kept permanently rather than fixed and forgotten.
    ('the {} guard becomes `is not None`, which looks MORE precise',
     'py_router/kicad_parser.py',
     "            if islands:",
     "            if islands is not None:",
     'a correct-looking tightening silently restores the quiet fall-through'),

    ('the warner is still called from both arms but prints nothing',
     'py_router/kicad_parser.py',
     '        print(f"WARNING: live-board staged refill did not yield a fill "',
     '        _unused = (f"WARNING: live-board staged refill did not yield a fill "',
     'an AST check that a warner is CALLED cannot see that it is silent'),

    ('the empty-fill clause is deleted at its SOURCE',
     'py_router/kicad_exact_fill.py',
     "    detail = '' if out else 'the refill ran and produced no islands'",
     "    detail = ''",
     'an arm that stubs the return asserts on a string the test wrote'),

    ('an empty fill is scored as the best possible rank again',
     'py_placer/plane_score.py',
     "        if not per_net:",
     "        if False:",
     'islands=0 is the OPTIMUM of a term that sorts before hpwl'),

    ('the guard goes back to testing the WHOLE fill dict',
     'py_placer/plane_score.py',
     "        if not per_net:",
     "        if not fills:",
     'the shipped-then-corrected form: `fills` is non-empty on the 29 boards '
     'with pre-existing zones, so a candidate whose WANTED pour is empty '
     'still scored islands=0 and outranked every real one'),

    ('plane_score.why() drops its own detail again',
     'py_placer/plane_score.py',
     "        return f'{base} ({self.detail})' if self.detail else base",
     "        return base",
     'no_named_net builds the net list the reader needs, then discards it'),

    ('an empty map passes the stitch validator again',
     'py_router/route_planes.py',
     "    if not islands_map:\n        return None",
     "    if islands_map is None:\n        return None",
     'an absent verdict must not be reported as a fact about the board'),
]


def _clear_pyc():
    for base, dirs, _ in os.walk(ROOT):
        for d in list(dirs):
            if d == '__pycache__':
                shutil.rmtree(os.path.join(base, d), ignore_errors=True)
                dirs.remove(d)


def run_gate():
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1',
               PYTHONIOENCODING='utf-8')
    r = subprocess.run([sys.executable, '-X', 'utf8', GATE],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, env=env, timeout=1800)
    return r.returncode, (r.stdout + r.stderr)


_clear_pyc()
rc, out = run_gate()
if rc != 0:
    print("BROKEN: the gate does not pass on the UNMUTATED tree; "
          "every row below would be meaningless.")
    print(out[-3000:])
    sys.exit(1)
print(f"baseline: gate passes clean ({out.strip().splitlines()[-1]})\n")

killed = survived = broken = 0
for label, rel, old, new, why in ROWS:
    path = os.path.join(ROOT, rel)
    with open(path, encoding='utf-8') as f:
        original = f.read()
    if original.count(old) != 1:
        print(f"BROKEN {label}\n    anchor matched {original.count(old)} times "
              f"in {rel} (need exactly 1) -- the row tests nothing")
        broken += 1
        continue
    try:
        with open(path, 'w', encoding='utf-8') as f:
            f.write(original.replace(old, new))
        _clear_pyc()
        rc, out = run_gate()
    finally:
        with open(path, 'w', encoding='utf-8') as f:
            f.write(original)
        _clear_pyc()
    if rc != 0:
        killed += 1
        print(f"KILLED   {label}")
    else:
        survived += 1
        print(f"SURVIVED {label}\n    unpinned claim: {why}")

print(f"\n{len(ROWS)} rows: {killed} killed, {survived} survived, "
      f"{broken} broken")
sys.exit(1 if (survived or broken) else 0)
