#!/usr/bin/env python3
"""#622 `awx/rules.py`: a run's supplied rules reach every module the whole route loads, the same way twice.

The bus step (awx/route_bus.py) resolves the chain's sizes as route.py resolves its own and supplies them to every
stage through one setting (rules.SETTING). A stage in a process of its own imports the chain's modules under it: each
module initializes its constants from rules.active(), the supplied rules. A stage in the driver's process -- or a
GUI's, where the modules were imported long before -- meets modules already imported: rules.install() rewrites them.
The two must leave every module the same, bit for bit, or the same run plans differently in-process.

What this asserts, over every awx module the whole route loads (whole_route.py's stages' imports, any depth; the stage
scripts themselves read these modules' constants at each run) and every module route_bus.py imports:

1. FRESH (the chain's modules imported with the rules supplied) == INSTALLED (imported with nothing supplied, then
   rules.install of the same rules): every module-level numeric constant, compared by float.hex.
2. The check is live: with nothing supplied the constants differ from FRESH (braid.TRACK among them), and an install of
   DEFAULT leaves them as a plain import has them.
"""
import ast
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
AWX = os.path.join(os.path.dirname(HERE), 'awx')
FAIL = []

# the recorded zynq chain's sizes: lanes 0.15, escapes 0.12, clearance 0.09, via 0.45/0.3
SIZES = dict(clearance=0.09, track_width=0.15, via_size=0.45, via_drill=0.3)
FAN_TRACK = 0.12

CHILD = r'''
import contextlib, io, json, os, sys, types
AWX, mods, mode, sizes, fan = sys.argv[1], sys.argv[2].split(','), sys.argv[3], json.loads(sys.argv[4]), float(sys.argv[5])
sys.path.insert(0, AWX); sys.path.insert(0, os.path.join(AWX, '..', 'py_router'))
os.chdir(AWX)
import importlib
import rules
with contextlib.redirect_stdout(io.StringIO()):
    for m in mods:
        importlib.import_module(m)
if mode == 'installed':
    rules.install(rules.Rules.from_router_config(types.SimpleNamespace(**sizes), fan_track=fan))
elif mode == 'default':
    rules.install(rules.DEFAULT)
out = {}
for m in mods:
    mod = sys.modules[m]
    out[m] = {k: float(v).hex() for k, v in vars(mod).items()
              if k.lstrip('_').isupper() and isinstance(v, (int, float)) and not isinstance(v, bool)}
print(json.dumps(out, sort_keys=True))
'''


def check(ok, what):
    print(('  ok   ' if ok else '  FAIL ') + what)
    if not ok:
        FAIL.append(what)


def closure(roots):
    """the awx modules `roots` import, any depth, the roots included"""
    local = {f[:-3] for f in os.listdir(AWX) if f.endswith('.py')}
    seen, todo = set(), sorted(roots)
    while todo:
        m = todo.pop()
        if m in seen or m not in local:
            continue
        seen.add(m)
        for n in ast.walk(ast.parse(open(os.path.join(AWX, m + '.py')).read())):
            if isinstance(n, ast.Import):
                todo += [a.name.split('.')[0] for a in n.names]
            elif isinstance(n, ast.ImportFrom) and n.module:
                todo.append(n.module.split('.')[0])
    return seen


def library_modules():
    """the awx modules the stages whole_route.py names import, any depth (the stage scripts themselves left out: they
    do their work at import, and read these modules' constants at each run), and every module route_bus.py -- the
    step, in its own process -- imports, any depth (make_bench, fanout_from_plan as a module, ...)"""
    driver = ast.parse(open(os.path.join(AWX, 'whole_route.py')).read())
    stages = {n.value[:-3] for n in ast.walk(driver) if isinstance(n, ast.Constant) and isinstance(n.value, str)
              and re.fullmatch(r'[a-z_]+\.py', n.value) and os.path.isfile(os.path.join(AWX, n.value))}
    return sorted((closure(stages) - stages) | closure(['route_bus']))


def run(mods, mode, supplied):
    sys.path.insert(0, AWX)
    import rules
    env = dict(os.environ)
    env.pop(rules.SETTING, None)
    if supplied:
        env[rules.SETTING] = rules.as_setting(rules.Rules.from_router_config(
            __import__('types').SimpleNamespace(**SIZES), fan_track=FAN_TRACK))
    r = subprocess.run([sys.executable, '-c', CHILD, AWX, ','.join(mods), mode, json.dumps(SIZES), str(FAN_TRACK)],
                       capture_output=True, text=True, env=env)
    if r.returncode:
        print(r.stderr[-2000:])
        raise SystemExit(f'BROKEN TEST: the {mode} import died (exit {r.returncode})')
    return json.loads(r.stdout.strip().splitlines()[-1])


def diff(a, b):
    out = []
    for m in sorted(set(a) | set(b)):
        for k in sorted(set(a.get(m, {})) | set(b.get(m, {}))):
            if a.get(m, {}).get(k) != b.get(m, {}).get(k):
                out.append(f'{m}.{k}')
    return out


def main():
    mods = library_modules()
    print(f'{len(mods)} modules the whole route loads: {", ".join(mods)}')
    fresh = run(mods, 'plain', supplied=True)
    installed = run(mods, 'installed', supplied=False)
    plain = run(mods, 'plain', supplied=False)
    default = run(mods, 'default', supplied=False)
    d = diff(fresh, installed)
    check(not d, 'an install leaves every module as a fresh import under the supplied rules does'
          + (f' -- differ: {", ".join(d[:12])}' if d else ''))
    live = diff(plain, fresh)
    check('braid.TRACK' in live and 'braid.VIA_NEED' in live and 'source_realize.FAN_TRACK' in live,
          f'the supplied rules move the constants ({len(live)} of them, braid.TRACK among them)')
    d = diff(plain, default)
    check(not d, 'an install of DEFAULT leaves a plain import as it is' + (f' -- differ: {", ".join(d[:12])}' if d else ''))
    print('all passed' if not FAIL else f'{len(FAIL)} FAILED')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
