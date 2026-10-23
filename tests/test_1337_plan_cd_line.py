#!/usr/bin/env python3
"""#1337: a plan.sh's `cd <repo>` line sets the cwd its commands run in.

The routing skill says to begin `<board>_plan.sh` with `cd <repo>` (and
route_plan_check enforces it), but parse_manifest read only `# cwd=`
comments and parsed the `cd` -- and the `export` lines -- as ordinary
commands. Every command's cwd stayed None, so manifest_to_plan resolved a
repo-relative `--intent` against the manifest's folder, "rejected" it, and
the pruner dropped everything upstream of it under a summary that said "1
command(s) skipped" (gp14 daplink: 4 plan steps where the chain has 7).

Asserts:
  1. parse_manifest: `cd DIR` is a sticky cwd like `# cwd=` (relative to
     the cwd so far, else the manifest's folder; `cd x && y` stays a
     command); `export` is not a command -- a knob export becomes the env
     prefix of every later command (the command's own prefix wins), and a
     harness export (REDO_*) is dropped;
  2. manifest_to_plan converts a `cd`-led plan into the same steps as the
     `# cwd=` control, from any working directory;
  3. a command that really is rejected takes the steps upstream of it out
     of the plan NAMED and COUNTED, not silently.

    python3 tests/test_1337_plan_cd_line.py
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
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
sys.path.insert(0, TESTS_DIR)

from placement.floorplan import KIND, SCHEMA_VERSION            # noqa: E402
from redo_stress_test import parse_manifest, split_env_prefix   # noqa: E402
from run_utils import check, evidence                           # noqa: E402

M2P = os.path.join(ROOT, 'tests', 'stress', 'manifest_to_plan.py')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def write(path, text):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


def plan_text(repo, intent):
    q = shlex.quote
    return (f"cd {q(repo)}\n"
            "export REDO_RUNDIR=b\n"
            "export REDO_MANIFEST=$REDO_RUNDIR/redo_commands.sh\n"
            "export KICAD_ROUTE_TRACE=1\n"
            "# a plan's comment\n"
            "python3 -X utf8 py_router/route_planes.py b/in.kicad_pcb "
            "b/s1.kicad_pcb --nets GND --plane-layers B.Cu\n"
            "python3 -X utf8 py_placer/place_fanout_clearance.py "
            f"b/s1.kicad_pcb b/s2.kicad_pcb --clearance 0.2 --intent {intent}\n"
            "python3 -X utf8 py_router/route.py b/s2.kicad_pcb b/s3.kicad_pcb "
            "--nets '*'\n")


def convert(man, out_json, allow=()):
    r = check([sys.executable, '-X', 'utf8', M2P, man, '-o', out_json],
              accept=True, cwd=os.path.dirname(os.path.abspath(os.sep)),
              allow=allow)
    steps = json.load(open(evidence(out_json, 'the plan'), encoding='utf-8'))['steps']
    return [s['action'] for s in steps], r.stdout + r.stderr


def main():
    tmp = tempfile.mkdtemp(prefix='t1337_')
    try:
        print("1: parse_manifest")
        sub = os.path.join(tmp, 'sub')
        os.makedirs(sub)
        man = write(os.path.join(tmp, 'm.sh'), (
            "cd sub\n"
            "export KICAD_X=1 REDO_MANIFEST=/elsewhere/redo.sh\n"
            "python3 a.py\n"
            "KICAD_X=2 python3 b.py\n"
            "cd ..\n"
            "cd sub && python3 c.py\n"
            f"# cwd={shlex.quote(tmp)}\n"
            "python3 d.py\n"))
        cmds = parse_manifest(man)
        argvs = [split_env_prefix(a) for _c, a in cmds]
        ok("cd and export lines are not commands",
           [a[1][-1] for a in argvs] == ['a.py', 'b.py', 'c.py', 'd.py'],
           str([a[1] for a in argvs]))
        ok("a relative cd resolves against the manifest's folder",
           cmds[0][0] == sub, str(cmds[0][0]))
        ok("cd .. is relative to the cwd so far", cmds[2][0] == tmp, str(cmds[2][0]))
        ok("`cd x && y` stays one command",
           argvs[2][1][:2] == ['cd', 'sub'], str(argvs[2][1]))
        ok("a knob export reaches later commands, REDO_* does not",
           argvs[0][0] == {'KICAD_X': '1'}, str(argvs[0][0]))
        ok("a command's own prefix wins over the export",
           argvs[1][0] == {'KICAD_X': '2'}, str(argvs[1][0]))
        ok("# cwd= still works after a cd", cmds[3][0] == tmp, str(cmds[3][0]))

        print("2: manifest_to_plan, `cd` vs the `# cwd=` control")
        repo = os.path.join(tmp, 'repo')
        os.makedirs(os.path.join(repo, 'b'))
        write(os.path.join(repo, 'intent.json'), json.dumps(
            {'schema': SCHEMA_VERSION, 'kind': KIND, 'units': 'mm'}))
        plans = os.path.join(tmp, 'plans')
        os.makedirs(plans)
        cd_plan = write(os.path.join(plans, 'p_cd.sh'), plan_text(repo, 'intent.json'))
        cwd_plan = write(os.path.join(plans, 'p_cwd.sh'),
                         f"# cwd={shlex.quote(repo)}\n" + plan_text(repo, 'intent.json'))
        want = ['route_planes', 'optimize_caps', 'route']
        got_cd, log_cd = convert(cd_plan, os.path.join(tmp, 'cd.json'))
        got_cwd, _ = convert(cwd_plan, os.path.join(tmp, 'cwd.json'))
        ok("the `# cwd=` control converts the whole chain", got_cwd == want, str(got_cwd))
        ok("the `cd` plan converts the same chain", got_cd == want, str(got_cd))
        ok("nothing was rejected", 'REJECTED' not in log_cd, log_cd[-300:])
        ok("the exported knob is named, not silently dropped",
           'KICAD_ROUTE_TRACE=1' in log_cd)

        print("3: a real rejection names what it takes with it")
        bad = write(os.path.join(plans, 'p_bad.sh'), plan_text(repo, 'missing.json'))
        # The rejection's own reason quotes the OS error, by design.
        got, log = convert(bad, os.path.join(tmp, 'bad.json'),
                           allow=('No such file or directory',))
        ok("the rejected cap pass and its upstream are not steps", got == ['route'],
           str(got))
        ok("the rejection is reported", 'REJECTED' in log and 'missing.json' in log)
        ok("the upstream plane step is NAMED",
           'upstream of them' in log and 'route_planes.py' in log, log[-400:])
        ok("and COUNTED in the summary", '(2 command(s) skipped' in log,
           next((l for l in log.splitlines() if 'step(s) written' in l), ''))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
