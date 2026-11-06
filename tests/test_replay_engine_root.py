#!/usr/bin/env python3
"""replay_plan_vs_run's CLI leg runs the checkout it is launched from, and run_queue.sh
leaves a named set alone.

A manifest bakes the absolute path of the repo that recorded it, and redo_stress_test
keeps a recorded path that still exists, so from a worktree the "CLI at HEAD" leg ran the
main checkout's engine, and a recording from a deleted tree did not run at all.
"""
import os
import subprocess
import sys
import tempfile
from unittest.mock import patch

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(REPO, 'tests', 'gui_parity'), os.path.join(REPO, 'tests'),
                os.path.join(REPO, 'py_router')]
import replay_plan_vs_run as rpvr  # noqa: E402
from replay_plan_vs_run import recorded_engine_remaps  # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {name}" + (f': {detail}' if detail and not ok else ''))
    if not ok:
        FAILS.append(f'{name}: {detail}' if detail else name)


def main():
    with tempfile.TemporaryDirectory() as td:
        here = os.path.join(td, 'here')
        for sub in ('py_router', 'py_placer'):
            os.makedirs(os.path.join(here, sub))
        for rel in ('py_router/route.py', 'py_placer/place_simple.py', 'py_router/old_top.py'):
            open(os.path.join(here, rel), 'w').close()
        manifest = os.path.join(td, 'm.sh')
        with open(manifest, 'w', encoding='utf-8') as fh:
            fh.write('# cwd=/rec/run\n'
                     'python3 -u -X utf8 /main/KRT/py_router/route.py /rec/run/a.kicad_pcb b.kicad_pcb\n'
                     'python3 -u /gone/tree/py_placer/place_simple.py b.kicad_pcb c.kicad_pcb\n'
                     "python3 '/older/KRT/old_top.py' c.kicad_pcb d.kicad_pcb\n"
                     f'python3 {here}/py_router/route.py d.kicad_pcb e.kicad_pcb\n'
                     'python3 /rec/run/my_own_script.py e.kicad_pcb\n')
        got = dict(recorded_engine_remaps(manifest, here))
        check('a recorded main checkout maps to this one', got.get('/main/KRT') == here, str(got))
        check('so does a tree that no longer exists', got.get('/gone/tree') == here, str(got))
        check('so does a script recorded before the folders', got.get('/older/KRT') == here, str(got))
        check('this checkout needs no remap', here not in got, str(got))
        check("a run's own script is not an engine", '/rec/run' not in got, str(got))

        # Through the real replayer, on the driver's own arguments.
        rec = os.path.join(td, 'rec')
        os.makedirs(rec)
        open(os.path.join(rec, 'a.kicad_pcb'), 'w').close()
        real = os.path.join(rec, 'redo_commands.sh')
        with open(real, 'w', encoding='utf-8') as fh:
            fh.write(f'python3 -u -X utf8 /elsewhere/KRT/py_router/route.py {rec}/a.kicad_pcb '
                     f"{rec}/b.kicad_pcb --nets '*'\n")
        calls = []

        class Done:
            returncode = 0

        def spy(cmd, **kw):
            calls.append(cmd)
            return Done()
        dest_root = os.path.join(td, 'out')
        with patch.object(rpvr.subprocess, 'run', spy):
            rpvr.build_cli_reference({'manifest': real, 'rundir': rec}, dest_root)
        cmd = calls[0]
        check("the driver's CLI leg remaps the recorded engine",
              f'/elsewhere/KRT:{REPO}' in cmd, str(cmd))
        check('before the run dir, so a destination is never rewritten',
              cmd.index(f'/elsewhere/KRT:{REPO}') < cmd.index(f"{rec}:{os.path.join(dest_root, 'cli_head')}"),
              str(cmd))
        r = subprocess.run([sys.executable, '-X', 'utf8', *cmd[3:], '--dry-run'],
                           capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=REPO)
        ran = [ln for ln in r.stdout.splitlines() if 'route.py' in ln and 'Remaps' not in ln]
        check('the replay runs this checkout',
              ran and all(os.path.join(REPO, 'py_router', 'route.py') in ln for ln in ran),
              (r.stdout + r.stderr)[-500:])

        # run_queue's set filter, as the script defines it.
        with open(os.path.join(REPO, 'tests', 'stress', 'run_queue.sh'), encoding='utf-8') as fh:
            text = fh.read()
        start = text.index('in_scope(){')
        body = text[start:text.index('\n}\n', start) + 3]
        for scope, s, want in (('', '12', 0), ('', '1arc', 1), ('', '3monster', 1),
                               ('11 12', '12', 0), ('11 12', '13', 1), ('1arc', '1arc', 0),
                               ('1arc', '12', 1)):
            rc = subprocess.run(['bash', '-c', f'QUEUE_SETS="{scope}"\n{body}\nin_scope "{s}"'],
                                capture_output=True).returncode
            check(f'QUEUE_SETS="{scope}", set {s}: {"in" if want == 0 else "out"}', rc == want, str(rc))
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
