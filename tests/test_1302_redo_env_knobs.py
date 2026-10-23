#!/usr/bin/env python3
"""#1302: a recorded step replays with the env knobs it ran with.

redo_record wrote `python3 -u -X utf8 <argv>` and never the environment, so
a step run under KICAD_PLANE_FRAGILITY_COST=0 replayed at the defaults (gp14
ecc83: 0 vias / 131.72 mm as run, 2 vias / 146.10 mm replayed). A prefix
added by hand did not help: redo_stress_test handed it to Popen as argv[0]
and died (`PermissionError: 'KICAD_PLANE_FRAGILITY_COST=0'`), and
manifest_to_plan dropped it without a word.

Asserts:
  1. env_knobs.replay_env_knobs keeps the routing knobs, whole, and leaves
     out what locates the recording machine (interpreters, kicad-cli,
     directories) or a harness;
  2. record_invocation leads the line with `NAME=value`, quoted on the value
     only (bash's assignment syntax), and the timings record carries it;
  3. split_env_prefix reads it back from the parsed manifest;
  4. redo_stress_test replays a prefixed command with the knob in the
     child's environment (it crashed before);
  5. manifest_to_plan converts the step and NAMES the knob it cannot carry.

    python3 tests/test_1302_redo_env_knobs.py
"""
import json
import os
import shlex
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
sys.path.insert(0, TESTS_DIR)

import env_knobs                                                 # noqa: E402
import redo_record                                               # noqa: E402
from redo_stress_test import parse_manifest, split_env_prefix    # noqa: E402
from run_utils import check, evidence                            # noqa: E402

failures = []
KNOB = 'KICAD_PLANE_FRAGILITY_COST'
LAYERS = 'KICAD_PLANE_FRAGILITY_LAYERS'


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


class env_set:
    """Set/unset environment variables for a block, then restore them."""
    def __init__(self, **kv):
        self.kv, self.old = kv, {}

    def __enter__(self):
        for k, v in self.kv.items():
            self.old[k] = os.environ.get(k)
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

    def __exit__(self, *a):
        for k, v in self.old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


def clean_env():
    """This process's environment with no KICAD_*/KRT_* knob and no REDO_*."""
    return {k: v for k, v in os.environ.items()
            if not k.startswith(('KICAD', 'KRT_', 'REDO_'))}


def main():
    tmp = tempfile.mkdtemp(prefix='t1302_')
    saved = dict(os.environ)
    try:
        os.environ.clear()
        os.environ.update(clean_env())

        print("1: replay_env_knobs")
        long_val = 'x' * 300
        with env_set(**{KNOB: '0', LAYERS: long_val, 'KICAD_PYTHON': '/a/py',
                        'KICAD_CLI': '/a/kicad-cli', 'KICAD_SWEEP_DIRTY': '1',
                        'KRT_DATASHEET_DIR': '/a/ds', 'KICAD9_3DMODEL_DIR': '/m',
                        'KICAD_CONFIG_HOME': '/c'}):
            k = env_knobs.replay_env_knobs()
        ok("keeps the routing knobs", k.get(KNOB) == '0' and LAYERS in k, str(k)[:120])
        ok("a long value is kept whole (the log line truncates it)",
           k.get(LAYERS) == long_val)
        ok("leaves out interpreter / kicad-cli / sweep / dir / install names",
           not ({'KICAD_PYTHON', 'KICAD_CLI', 'KICAD_SWEEP_DIRTY',
                 'KRT_DATASHEET_DIR', 'KICAD9_3DMODEL_DIR',
                 'KICAD_CONFIG_HOME'} & set(k)), str(sorted(k)))

        print("2: record_invocation")
        man = os.path.join(tmp, 'run', 'redo_commands.sh')
        os.makedirs(os.path.dirname(man))
        argv0 = sys.argv
        sys.argv = ['py_router/route.py', 'in.kicad_pcb', 'out.kicad_pcb',
                    '--nets', '*']
        try:
            with env_set(REDO_MANIFEST=man, REDO_RUNDIR=os.path.dirname(man),
                         **{KNOB: '0', LAYERS: 'GND@F.Cu=0, GND@B.Cu=0'}):
                redo_record.record_invocation()
        finally:
            sys.argv = argv0
        line = open(evidence(man, 'the manifest'), encoding='utf-8').read() \
            .strip().splitlines()[-1]
        ok("the line leads with the knobs",
           line.startswith(f"{KNOB}=0 {LAYERS}='GND@F.Cu=0, GND@B.Cu=0' python3 "),
           line)
        toks = shlex.split(line)
        ok("bash reads them as assignments, value intact",
           toks[:2] == [f"{KNOB}=0", f"{LAYERS}=GND@F.Cu=0, GND@B.Cu=0"]
           and toks[2] == 'python3', str(toks[:3]))
        # The timings record is written at exit by an atexit hook; call the
        # registered writer through a fresh process instead of exiting this one.
        with env_set(REDO_MANIFEST=man, REDO_RUNDIR=os.path.dirname(man),
                     **{KNOB: '0'}):
            check([sys.executable, '-c',
                   'import sys; sys.path.insert(0, "py_router"); '
                   'sys.argv = ["py_router/route.py", "a.kicad_pcb", '
                   '"b.kicad_pcb"]; import redo_record; '
                   'redo_record.record_invocation()'], accept=True)
        tim = man[:-3] + '.timings.jsonl'
        rec = json.loads(open(evidence(tim, 'the timings log'),
                              encoding='utf-8').read().strip().splitlines()[-1])
        ok("the timings record carries env", rec.get('env') == {KNOB: '0'},
           str(rec.get('env')))
        with env_set(REDO_MANIFEST=man, REDO_RUNDIR=os.path.dirname(man)):
            sys.argv = ['py_router/route.py', 'x.kicad_pcb', 'y.kicad_pcb']
            try:
                redo_record.record_invocation()
            finally:
                sys.argv = argv0
        last = open(man, encoding='utf-8').read().strip().splitlines()[-1]
        ok("no knob, no prefix", last.startswith('python3 -u -X utf8 '), last)

        print("3: parse_manifest + split_env_prefix")
        cmds = parse_manifest(man)
        env, argv = split_env_prefix(cmds[0][1])
        ok("the prefix reads back as the environment",
           env == {KNOB: '0', LAYERS: 'GND@F.Cu=0, GND@B.Cu=0'}
           and argv[:2] == ['python3', '-u'], f"{env} {argv[:3]}")
        env, argv = split_env_prefix(['python3', 'a.py', 'X=1'])
        ok("an assignment AFTER the command is an argument", env == {}
           and argv == ['python3', 'a.py', 'X=1'])

        print("4: redo_stress_test applies the prefix")
        wd = os.path.join(tmp, 'work')
        os.makedirs(wd)
        # A stand-in tool that names no board: the replay then has no final
        # board, so it renders nothing afterwards.
        tool = os.path.join(tmp, 'echo_tool.py')
        with open(tool, 'w', encoding='utf-8') as f:
            f.write("import os\n"
                    f"print('CHILD SAW ' + repr(os.environ.get('{KNOB}')))\n")
        hand = os.path.join(tmp, 'hand.sh')
        with open(hand, 'w', encoding='utf-8') as f:
            f.write(f"# cwd={shlex.quote(wd)}\n{KNOB}=0 "
                    f"{shlex.quote(sys.executable)} {shlex.quote(tool)}\n")
        r = check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'tests', 'stress', 'redo_stress_test.py'),
                   hand, '--verbatim', '--skip-validate'], accept=True,
                  cwd=wd, env=clean_env())
        ok("the child ran with the knob", "CHILD SAW '0'" in r.stdout,
           r.stdout[-200:])

        print("5: manifest_to_plan names the knob it cannot carry")
        plan_man = os.path.join(tmp, 'plan.sh')
        with open(plan_man, 'w', encoding='utf-8') as f:
            f.write(f"# cwd={shlex.quote(ROOT)}\n{KNOB}=0 python3 -u -X utf8 "
                    f"py_router/route.py kicad_files/cap_chain.kicad_pcb "
                    f"{shlex.quote(os.path.join(tmp, 'out.kicad_pcb'))} "
                    f"--nets '*'\n")
        out_json = os.path.join(tmp, 'plan.json')
        r = check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'tests', 'stress', 'manifest_to_plan.py'),
                   plan_man, '-o', out_json], accept=True, env=clean_env())
        steps = json.load(open(evidence(out_json, 'the plan'), encoding='utf-8'))['steps']
        ok("the step is converted", [s['action'] for s in steps] == ['route'],
           str([s['action'] for s in steps]))
        ok("the dropped knob is named", f"{KNOB}=0" in r.stderr
           and 'env knobs' in r.stderr, r.stderr[-300:])
    finally:
        os.environ.clear()
        os.environ.update(saved)
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
