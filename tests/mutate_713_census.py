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

# This battery's runner is at MODULE SCOPE, so `import mutate_713_census`
# RUNS THE GATE AND REWRITES ENGINE FILES. #877 was filed after a census did
# exactly that to six batteries, rewrote 13 files under py_router/ and
# py_placer/, and then reported numbers measured against its own damage.
# Refusing the import outright, rather than hiding the runner behind
# `if __name__`, keeps the reason visible and names the API that answers the
# question the importer actually had.
if __name__ != '__main__':                                 # pragma: no cover
    raise ImportError(
        'tests/mutate_713_census.py is a SCRIPT, not a module: importing it '
        'runs the battery and rewrites engine files in place. To read its '
        'rows, use tests/mutation_anchors.resolve_static(path), which parses '
        'the file instead of executing it.')

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

    # Targets the REGISTRY rather than a source file. Removing a call no
    # longer makes a file undiscovered -- since discovery widened, a bare
    # `import time` still matches, which is deliberate (an import is a latent
    # site). So the staleness rule is exercised the way it actually fires: an
    # entry naming a file that carries no clock at all.
    ('a REGISTRY entry for a file with no clock in it',
     'tests/test_713_wallclock_census.py',
     "REGISTRY = {\n",
     "REGISTRY = {\n    'py_router/routing_defaults.py': ('reporting', 'x'),\n",
     'a stale registry rots into folklore'),

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

# Every anchor must match its target exactly once BEFORE anything is
# rewritten. A stale anchor otherwise reports BROKEN mid-run, after the
# witnesses have been paid for; this is the one second (#877).
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


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
        # RAW BYTES for the restore, decoded text for the match (#877).
        # Writing without `newline=''` translates every '\n' to os.linesep, so
        # on Windows one run rewrote the whole target in CRLF and left it
        # permanently "modified" -- which then tripped a dirty-tree refusal on
        # the next run. `.gitattributes` pins `*.py text eol=lf`, so that was a
        # real corruption, not a preference. `mutate_711.py:296-321` has the
        # same pair and records the other half: matching a multi-line anchor
        # against a RAW decode silently found nothing in three rows.
        raw = open(path, 'rb').read()
        with open(path, encoding='utf-8') as f:
            original = f.read()
        if original.count(old) != 1:
            print(f"BROKEN {label}\n    anchor matched "
                  f"{original.count(old)} times in {rel} (need 1)")
            broken += 1
            continue
    try:
        with open(path, 'w', encoding='utf-8', newline='') as f:
            f.write(new if created else original.replace(old, new))
        _clear_pyc()
        rc, out = run_gate()
    finally:
        if created:
            os.unlink(path)
        else:
            open(path, 'wb').write(raw)      # byte-exact, from what was read
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
