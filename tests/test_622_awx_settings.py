#!/usr/bin/env python3
"""#622 `awx/awx_settings.py`: the whole route reads its settings through one module, never os.environ.

The bus step runs the whole route inside KiCad's own process, where os.environ is every module's: a routing call there
may not write it. So every awx module the whole route loads asks `awx_settings.get(...)` (its own default kept), and a
caller running the route in its own process gives the values for the call (`awx_settings.given`). With nothing given
the environment answers, so the harness's scripts and the stages run as processes of their own read what they always
read.

What this asserts:

1. given() answers for every read inside its block -- get, req (KeyError as os.environ[name] raises), environ() -- the
   innermost of nested blocks first, and puts the environment back after; os.environ is neither read nor written
   inside it.
2. No module the whole route loads reads os.environ: the stages whole_route.py names and every awx module they import
   (any depth), save the two lines below that the whole route never runs.
"""
import ast
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
AWX = os.path.join(os.path.dirname(HERE), 'awx')
sys.path.insert(0, AWX)
import awx_settings  # noqa: E402

FAIL = []

# (module, the line's start): os.environ uses the whole route never runs
NOT_THE_ROUTE = {
    ('probe_memo', "    os.environ['PROBE_MEMO_DIR'] = d"),               # its own self-test
    ('whole_ends', "    os.environ.setdefault('PLAN_JUDGE', 'ends')"),     # its own __main__, a research CLI
}


def check(ok, what):
    print(('  ok   ' if ok else '  FAIL ') + what)
    if not ok:
        FAIL.append(what)


def chain_modules():
    """the stages whole_route.py names and the awx modules they import, any depth"""
    local = {f[:-3] for f in os.listdir(AWX) if f.endswith('.py')}
    driver = ast.parse(open(os.path.join(AWX, 'whole_route.py')).read())
    stages = {n.value[:-3] for n in ast.walk(driver) if isinstance(n, ast.Constant) and isinstance(n.value, str)
              and re.fullmatch(r'[a-z_]+\.py', n.value) and os.path.isfile(os.path.join(AWX, n.value))}
    seen, todo = set(), sorted(stages)
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
    return stages, seen


def main():
    print('1. given')
    key = 'AWX_SETTINGS_TEST_KNOB'
    os.environ.pop(key, None)
    snapshot = dict(os.environ)
    check(awx_settings.get(key, 'd') == 'd', 'nothing given: the environment answers (absent -> the default)')
    with awx_settings.given({key: 'outer'}):
        check(awx_settings.get(key) == 'outer' and awx_settings.req(key) == 'outer', 'given: get and req answer')
        check(awx_settings.get('HOME') is None, 'given is the WHOLE of it: a variable it does not name is absent')
        try:
            awx_settings.req('HOME')
            check(False, 'req of a name not given raises KeyError')
        except KeyError:
            check(True, 'req of a name not given raises KeyError')
        with awx_settings.given({key: 'inner'}):
            check(awx_settings.get(key) == 'inner', 'nested: the innermost answers')
        check(awx_settings.environ() == {key: 'outer'}, 'environ() is a copy of what is given')
    check(awx_settings.get(key, 'd') == 'd', 'after the block the environment answers again')
    check(dict(os.environ) == snapshot, 'os.environ untouched throughout')

    print('2. no module the whole route loads reads os.environ')
    stages, mods = chain_modules()
    check(len(stages) >= 10 and len(mods) >= 30, f'{len(stages)} stages, {len(mods)} modules (the sweep is not empty)')
    bad = []
    for m in sorted(mods - {'awx_settings'}):          # (the one module that reads the environment, by design)
        src = open(os.path.join(AWX, m + '.py')).read()
        lines = src.split('\n')
        # the tree, not the text: `__import__('os').environ[...]` spells no "os.environ" (whole_polish's once did),
        # and a comment or a docstring that names the environment is not a read
        for n in ast.walk(ast.parse(src)):
            hit = ((isinstance(n, ast.Attribute) and n.attr in ('environ', 'getenv', 'putenv', 'unsetenv')
                    and not (isinstance(n.value, ast.Name) and n.value.id == 'awx_settings'))
                   or (isinstance(n, ast.ImportFrom) and n.module == 'os'
                       and any(a.name in ('environ', 'getenv', 'putenv') for a in n.names)))
            if hit and not any(m == km and lines[n.lineno - 1].startswith(kl) for km, kl in NOT_THE_ROUTE):
                bad.append(f'{m}.py:{n.lineno}')
    check(not bad, 'every read goes through awx_settings' + ('' if not bad else ': ' + ', '.join(bad[:8])))
    for km, kl in sorted(NOT_THE_ROUTE):
        src = open(os.path.join(AWX, km + '.py')).read()
        check(('\n' + kl) in src, f'{km}: the exception still exists (else take it off the list)')

    print(f'\n{"FAILED: " + str(len(FAIL)) if FAIL else "all passed"}')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
