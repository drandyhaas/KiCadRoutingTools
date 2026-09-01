#!/usr/bin/env python3
"""Mutation battery for tests/test_713_wallclock_census.py.

The census exists to catch a NEW wall-clock decision site. A gate that cannot
be shown to catch one is a comment. Each row below reintroduces a #713-shaped
defect, in a form the gate might plausibly miss, and reports any that survives.

Same two hazards as tests/mutate_713_phase1.py: a fresh subprocess with
PYTHONDONTWRITEBYTECODE=1 and every __pycache__ cleared (a size-preserving edit
inside one second can leave a stale .pyc), and restore from an in-memory copy
rather than `git checkout --` (which has eaten uncommitted work here twice).
"""
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GATE = os.path.join(ROOT, 'tests', 'test_713_wallclock_census.py')

ROWS = [
    ('a brand-new budget in a registered reporting-only file',
     'py_router/route.py',
     "    base_start = time.time()",
     "    base_start = time.time()\n"
     "    if time.time() - base_start > 300.0:\n        pass",
     'the exact shape of all five #713 survivors'),

    ('a budget hidden behind an ALIASED import',
     'py_router/phase3_routing.py',
     "    net_start_time = time.time()",
     "    from time import monotonic as _mono\n"
     "    net_start_time = time.time()\n"
     "    if _mono() - 0.0 > 300.0:\n        pass",
     'a symbol grep for time.time alone would miss this'),

    ('a clock stashed in a dict, then compared later',
     'py_router/phase3_routing.py',
     "    start_time = time.time()",
     "    start_time = time.time()\n"
     "    _b = {}\n    _b['t'] = time.time()\n"
     "    if _b['t'] > 1.0:\n        pass",
     'the value reaches a decision through a container'),

    ('a NEW file with a clock, registered nowhere',
     'py_router/__wallclock_probe.py',
     None,   # created, not patched
     "import time\n_t = time.time()\nif time.time() - _t > 60:\n    pass\n",
     'an unregistered site must fail rather than be assumed benign'),

    ('--plane-score-budget quietly re-added',
     'py_placer/place_portfolio.py',
     '    p.add_argument("--plane-score", nargs="+", default=None,',
     '    p.add_argument("--plane-score-budget", type=float, default=300.0)\n'
     '    p.add_argument("--plane-score", nargs="+", default=None,',
     'the flag this PR deleted must not come back'),

    ('--route-timeout quietly re-added to compare_seeds',
     'py_placer/compare_seeds.py',
     '    p.add_argument("--probe-gated", action="store_true",',
     '    p.add_argument("--route-timeout", type=int, default=1800)\n'
     '    p.add_argument("--probe-gated", action="store_true",',
     'the other deleted flag'),

    ('a registry entry kept after that file ONLY clock is gone',
     'py_placer/board_store.py',
     "        entry.setdefault('t', time.time())",
     "        entry.setdefault('t', 0.0)",
     'a stale registry rots into folklore; board_store has exactly one clock, '
     'so removing it must make the entry stale'),

    ('a budget added to a HANG_DETECTOR file, exempt from the comparison rule',
     'py_router/kicad_oracle.py',
     "    t0 = time.monotonic()",
     "    t0 = time.monotonic()\n"
     "    if time.monotonic() - t0 > 300.0:\n        pass",
     'the comparison rule covers `reporting` files only, so a budget added '
     'inside a hang-detector file is NOT caught. A deliberate limit -- those '
     'files legitimately compare a clock -- recorded as an expected survivor '
     'rather than discovered later'),
]


#: Rows that MUST survive, with the reason. A recorded expected survivor is a
#: stated limit of the gate; an unrecorded one is a hole nobody looked at.
#: The repo's convention (PR #823: "2 survived, both expected, with the
#: reason") -- never delete the row, never pretend it was killed.
EXPECTED_SURVIVORS = {
    'a budget added to a HANG_DETECTOR file, exempt from the comparison rule',
}


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
                       errors='replace', cwd=ROOT, env=env, timeout=600)
    return r.returncode, (r.stdout + r.stderr)


_clear_pyc()
rc, out = run_gate()
if rc != 0:
    print("BROKEN: the gate does not pass on the UNMUTATED tree.")
    print(out[-2000:])
    sys.exit(1)
print(f"baseline: {out.strip().splitlines()[-1]}\n")

killed = survived = broken = expected_survivors = 0
unexpected_kills = []
for label, rel, old, new, why in ROWS:
    path = os.path.join(ROOT, rel)
    created = old is None
    original = None
    if created:
        if os.path.exists(path):
            print(f"BROKEN {label}\n    {rel} already exists")
            broken += 1
            continue
    else:
        with open(path, encoding='utf-8') as f:
            original = f.read()
        if original.count(old) != 1:
            print(f"BROKEN {label}\n    anchor matched "
                  f"{original.count(old)} times in {rel} (need 1)")
            broken += 1
            continue
    try:
        with open(path, 'w', encoding='utf-8') as f:
            f.write(new if created else original.replace(old, new))
        _clear_pyc()
        rc, out = run_gate()
    finally:
        if created:
            os.unlink(path)
        else:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(original)
        _clear_pyc()
    expected = label in EXPECTED_SURVIVORS
    if rc != 0:
        killed += 1
        if expected:
            unexpected_kills.append(label)
            print(f"KILLED   {label}\n    BUT IT WAS RECORDED AS AN EXPECTED "
                  f"SURVIVOR -- the gate got stronger; update "
                  f"EXPECTED_SURVIVORS")
        else:
            print(f"KILLED   {label}")
    elif expected:
        expected_survivors += 1
        print(f"SURVIVED {label} (EXPECTED)\n    stated limit: {why}")
    else:
        survived += 1
        print(f"SURVIVED {label}\n    unpinned: {why}")

print(f"\n{len(ROWS)} rows: {killed} killed, {survived} survived, "
      f"{expected_survivors} expected survivor(s), {broken} broken")
sys.exit(1 if (survived or broken or unexpected_kills) else 0)
