#!/usr/bin/env python3
"""#1702: the placement A/B image carries KiCad's stock 3D models and proves at
build time that the body gate reads them, because an image without them does
not fail: the gate reads as unmeasured, the seat goes without it, and a score
calls the board buildable.

  1. the image is the plain release plus that release's models, and nothing else
     of its -full image; the proof runs at build, is staged by the deploy, and
     is covered by the app's name;
  2. a tree with no body gate is asked nothing, and says so;
  3. prove_models.py refuses a library that is empty, and one that holds enough
     files but not the models the gate reads, each for that reason;
  4. on this machine's KiCad, it passes and names the depth it read.

Items 1 and 2 ask nothing of KiCad. Without kicad-cli the rest exits 77 with a
`SKIP:` line.

    python3 tests/test_1702_placement_ab_models.py
"""
import os
import re
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import model_export  # noqa: E402
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 300
SKIP_EXIT = 77
AB = os.path.join(TESTS, 'stress', 'placement_ab')
PROVE = [sys.executable, '-X', 'utf8', '-B', os.path.join(AB, 'prove_models.py')]
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def read(name):
    with open(os.path.join(AB, name), encoding='utf-8') as f:
        return f.read()


def child_env(**extra):
    env = {k: v for k, v in os.environ.items() if not re.fullmatch(r'KICAD\d*_3DMODEL_DIR|KISYS3DMOD', k)}
    env.update(extra)
    return env


def main():
    print('1. the image')
    app, route = read('modal_app.py'), read('route_modal.py')
    copies = re.findall(r"COPY --from=(\S+) (\S+) ([^\s']+)", app)
    check('it copies one directory out of the -full image of the release it runs on',
          copies == [('{KICAD}-full', '/usr/share/kicad/3dmodels', '/usr/share/kicad/3dmodels')], str(copies))
    check('its base is still the plain release',
          "'KRT_PAB_KICAD_IMAGE', 'kicad/kicad:10.0.0')" in app and 'from_registry(KICAD,' in app)
    check('the proof runs in the build, on the shipped tree',
          "f'python3 /opt/prove_models.py {REPO}'" in app and "'/opt/prove_models.py', copy=True" in app)
    hashed = re.search(r"for f in \(([^)]*)\):\s+h\.update", route)
    staged = re.search(r"for f in \(([^)]*)\):\s+shutil\.copy2", route)
    check("the app's name covers the proof", bool(hashed) and "'prove_models.py'" in hashed.group(1))
    check('the deploy stages the proof', bool(staged) and "'prove_models.py'" in staged.group(1))

    with tempfile.TemporaryDirectory() as tmp:
        print('2. a tree with no body gate')
        old = os.path.join(tmp, 'old_tree')
        os.makedirs(old)
        r = run_utils.check(PROVE + [old], accept=True, env=child_env(), allow=('ModuleNotFoundError',))
        check('it is asked nothing and says so', 'nothing to prove' in r.stdout, r.stdout.strip()[:200])

        cli, why = model_export.resolve_cli()
        if not cli:
            run_utils.check(PROVE + [REPO], refuse='NO MODELS PROVED', code=1, env=child_env())
            if failures:
                print('FAILED:', failures)
                return 1
            print(f'SKIP: no kicad-cli ({why})')
            return SKIP_EXIT

        print('3. a short library')
        empty = os.path.join(tmp, 'models')
        os.makedirs(empty)
        cache = os.path.join(tmp, 'cache')
        # every variable: a model missing from one is looked up by name in the others
        env = child_env(XDG_CACHE_HOME=cache, **{v: empty for v in model_export._MODEL_DIR_VARS})
        run_utils.check(PROVE + [REPO], refuse='holds 0 models, fewer than 7000', code=1, env=env)
        check('an empty library is refused by its count', True)
        with open(os.path.join(empty, 'other_part.step'), 'w') as f:
            f.write('not one of the models the gate reads\n')
        r = run_utils.check(PROVE + [REPO, '--min-models', '1'], refuse='the body gate read', code=1, env=env)
        check('a library without the fixture\'s models is refused by the gate, with what it read',
              "no part's 3D model resolved" in (r.stdout + r.stderr), (r.stdout + r.stderr).strip()[-200:])

        print('4. this KiCad')
        r = run_utils.check(PROVE + [REPO], accept=True, env=child_env(XDG_CACHE_HOME=cache))
        check('it passes and names the depth it read',
              re.search(r'body gate read: J3 \d+\.\d+ mm inside', r.stdout) is not None, r.stdout.strip()[-200:])
    if failures:
        print('FAILED:', failures)
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
