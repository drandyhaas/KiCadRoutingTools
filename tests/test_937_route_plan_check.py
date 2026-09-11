#!/usr/bin/env python3
"""#937: the routing plan checker refuses the right plan, for the right rule.

A checker that refuses everything is worth nothing, and a checker that refuses
nothing is worth less. So this is built as a CONTROL plus one targeted break
per rule:

  * the compliant plan must exit 0 with every rule `ok` -- without that, every
    refusal below is just a checker that says no;
  * each break must exit 4 AND NAME ITS OWN RULE. `assert rc == 4` alone would
    pass on a plan that failed for a different reason, which is how a gate
    ends up testing nothing. CLAUDE.md states this directly: a non-zero exit
    is not evidence, assert the REASON.
  * and a break must not redden OTHER rules, or the naming is a coincidence.

It also pins what this checker reads. `plan_steps_from_manifest` -- the GUI
conversion -- drops `cd`, `cp`, pipes and every `check_*` command, so a
checker built on it could not see six of these rules AT ALL and would report
the broken plans clean. The cases that break those six are the regression
test for that design choice.
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHECKER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing',
                       'scripts', 'route_plan_check.py')

RUN_ALL_TIMEOUT = 300

#: A plan that satisfies every rule. Every break below is this file with ONE
#: line changed, so a reddened rule is attributable to that change.
COMPLIANT = """#!/bin/bash
set -e
cd /repo
# cwd=/repo
python3 -u -X utf8 py_router/bga_fanout.py b.kicad_pcb s1.kicad_pcb \
--component U1 --nets '*' --clearance 0.09 --layers F.Cu B.Cu
# cwd=/repo
python3 -u -X utf8 py_placer/place_fanout_clearance.py s1.kicad_pcb \
s2.kicad_pcb --clearance 0.09
# cwd=/repo
python3 -u -X utf8 py_router/route_planes.py s2.kicad_pcb s3.kicad_pcb \
--nets GND +3V3 --plane-layers In1.Cu In2.Cu
# cwd=/repo
python3 -u -X utf8 py_router/route.py s3.kicad_pcb final.kicad_pcb \
--nets '*' --power-nets GND +3V3 --clearance 0.09
# Verification -- COMMENTS, never executable lines: this file's exit code is
# read as the chain verdict.
#   python3 -X utf8 py_router/check_drc.py final.kicad_pcb
#   python3 -X utf8 py_router/check_connected.py final.kicad_pcb
#   python3 -X utf8 py_tools/check_orphan_stubs.py final.kicad_pcb
"""

#: (rule, what the break is, the edit as (old, new)). `old` must appear in
#: COMPLIANT exactly once -- asserted, because a break that edited nothing
#: would be recorded as "the checker missed it".
BREAKS = [
    ('R01', 'no cd line', ('cd /repo\n', '')),
    ('R02', 'a pipe on the route step',
     ('--clearance 0.09\n# Verification',
      "--clearance 0.09 2>&1 | tee route.log\n# Verification")),
    ('R03', 'a checker as an executable line',
     ('#   python3 -X utf8 py_router/check_drc.py final.kicad_pcb',
      'python3 -X utf8 py_router/check_drc.py final.kicad_pcb')),
    ('R04', 'the chain ends on a re-pour',
     ('# Verification',
      '# cwd=/repo\npython3 -u -X utf8 py_router/route_planes.py '
      'final.kicad_pcb after.kicad_pcb --nets GND --plane-layers In1.Cu\n'
      '# Verification')),
    ('R05', 'connectivity never named',
     ('#   python3 -X utf8 py_router/check_connected.py final.kicad_pcb\n',
      '')),
    ('R06', 'diff gap below clearance',
     ('# Verification',
      '# cwd=/repo\npython3 -u -X utf8 py_router/route_diff.py '
      'final.kicad_pcb d.kicad_pcb --nets /D+ /D- --clearance 0.1 '
      '--diff-pair-gap 0.08\n# cwd=/repo\npython3 -u -X utf8 '
      'py_router/route.py d.kicad_pcb final2.kicad_pcb --nets "*" '
      '--power-nets GND +3V3\n# Verification')),
    ('R07', 'teardrops requested',
     ('--power-nets GND +3V3 --clearance 0.09',
      '--power-nets GND +3V3 --clearance 0.09 --add-teardrops')),
    ('R08', 'smoothing turned off',
     ('--power-nets GND +3V3 --clearance 0.09',
      '--power-nets GND +3V3 --clearance 0.09 --no-smoothing')),
    ('R09', 'iterations capped',
     ('--power-nets GND +3V3 --clearance 0.09',
      '--power-nets GND +3V3 --clearance 0.09 --max-iterations 1000')),
    ('R10', 'max-ripup above the ceiling',
     ('--power-nets GND +3V3 --clearance 0.09',
      '--power-nets GND +3V3 --clearance 0.09 --max-ripup 10')),
    ('R11', 'a bare cp of a board',
     ('# cwd=/repo\npython3 -u -X utf8 py_router/route_planes.py',
      '# cwd=/repo\ncp s2.kicad_pcb copy.kicad_pcb\n'
      '# cwd=/repo\npython3 -u -X utf8 py_router/route_planes.py')),
    ('R12', 'no cap pass after fanout',
     ('# cwd=/repo\npython3 -u -X utf8 py_placer/place_fanout_clearance.py '
      's1.kicad_pcb s2.kicad_pcb --clearance 0.09\n', '')),
    ('R13', 'the first pour carries a via tail',
     ('--nets GND +3V3 --plane-layers In1.Cu In2.Cu',
      '--nets GND +3V3 --plane-layers In1.Cu In2.Cu --add-gnd-vias')),
    ('R14', 'a poured net with no route-step width',
     ('--power-nets GND +3V3', '--power-nets GND')),
    ('R15', 'gnd-via distance under the floor',
     ('--power-nets GND +3V3 --clearance 0.09',
      '--power-nets GND +3V3 --clearance 0.09 --via-size 0.45 '
      '--gnd-via-distance 0.5')),
]


def _run(path, *extra):
    return subprocess.run(
        [sys.executable, '-X', 'utf8', CHECKER, path] + list(extra),
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT, timeout=120)


def _write(tmp, name, text):
    path = os.path.join(tmp, name)
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)
    return path


def t_the_compliant_plan_passes():
    """The control. Without it every refusal below proves nothing."""
    with tempfile.TemporaryDirectory() as tmp:
        r = _run(_write(tmp, 'ok_plan.sh', COMPLIANT))
        assert r.returncode == 0, (
            f'the compliant plan was refused (rc={r.returncode}). A checker '
            f'that cannot pass a good plan only ever says no.\n{r.stdout}')
        assert 'FAIL' not in r.stdout, f'rules reddened:\n{r.stdout}'
        print('  PASS: the compliant plan exits 0 with every rule ok')


def t_each_break_is_refused_by_its_own_rule():
    """Exit 4 AND the rule's own id -- see the module docstring."""
    with tempfile.TemporaryDirectory() as tmp:
        for rid, what, (old, new) in BREAKS:
            assert COMPLIANT.count(old) == 1, (
                f'{rid}: the break anchor matches {COMPLIANT.count(old)} '
                f'times, so this row edits nothing and would be recorded as '
                f'a checker miss')
            r = _run(_write(tmp, f'broken_{rid}.sh',
                            COMPLIANT.replace(old, new, 1)))
            assert r.returncode == 4, (
                f'{rid} ({what}): expected exit 4, got {r.returncode}\n'
                f'{r.stdout}')
            failed = {ln.split()[1] for ln in r.stdout.splitlines()
                      if ln.strip().startswith('FAIL')}
            assert rid in failed, (
                f'{rid} ({what}) was refused, but by {sorted(failed)} -- not '
                f'by its own rule. Asserting only on the exit code would have '
                f'passed this.\n{r.stdout}')
        print(f'  PASS: {len(BREAKS)} break(s), each refused by its own rule')


def t_a_break_does_not_redden_unrelated_rules():
    """Attribution. If one bad line reddens four rules, naming one of them is
    a coincidence rather than a diagnosis."""
    noisy = []
    with tempfile.TemporaryDirectory() as tmp:
        for rid, what, (old, new) in BREAKS:
            r = _run(_write(tmp, f'n_{rid}.sh', COMPLIANT.replace(old, new, 1)))
            failed = {ln.split()[1] for ln in r.stdout.splitlines()
                      if ln.strip().startswith('FAIL')}
            if failed - {rid}:
                noisy.append(f'{rid} ({what}) also reddened '
                             f'{sorted(failed - {rid})}')
    # NO exemptions, and that is measured rather than hoped for: every one of
    # the 15 breaks reddens exactly its own rule. An allowlist was written
    # here first, for a case that turned out not to happen -- and an exempted
    # name is where this repo's guards have failed before, so it is gone.
    assert not noisy, (
        'a single broken line reddened rules it should not have, so naming '
        'one of them is a coincidence rather than a diagnosis:\n  '
        + '\n  '.join(noisy))
    print(f'  PASS: all {len(BREAKS)} breaks are attributable to exactly one '
          f'rule')


def t_the_checker_writes_nothing():
    """Read-only, because the standalone door runs under an analysis-only
    constraint that forbids modifying any file."""
    with tempfile.TemporaryDirectory() as tmp:
        path = _write(tmp, 'ro_plan.sh', COMPLIANT)
        before = {n: os.stat(os.path.join(tmp, n)).st_mtime_ns
                  for n in os.listdir(tmp)}
        _run(path)
        after = {n: os.stat(os.path.join(tmp, n)).st_mtime_ns
                 for n in os.listdir(tmp)}
        assert before == after, (
            f'the checker touched its input directory: '
            f'{set(after) ^ set(before) or "mtime changed"}')
        print('  PASS: no file created, changed or removed')


def t_an_unreadable_plan_is_exit_3_not_a_crash():
    """The dialect: 3 is "the plan could not be read", 1 is a crash. A
    traceback must never be able to read as a verdict."""
    with tempfile.TemporaryDirectory() as tmp:
        r = _run(os.path.join(tmp, 'nope.sh'))
        assert r.returncode == 3, (
            f'a missing plan gave {r.returncode}, not 3\n{r.stderr}')
        assert 'Traceback' not in r.stderr, r.stderr
        print('  PASS: a missing plan is exit 3, with no traceback')


def t_every_rule_and_every_delegation_is_listed():
    """`--list` is the honesty surface: what is checked, and what is not with
    the tool that owns it."""
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', CHECKER, '--list'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT, timeout=60)
    assert r.returncode == 0, r.stderr
    sys.path.insert(0, os.path.dirname(CHECKER))
    import route_plan_check as rpc
    for rid, _what, _cite, _fn in rpc.RULES:
        assert rid in r.stdout, f'{rid} is not in --list'
    for _what, who in rpc.DELEGATED:
        assert who.split(',')[0].strip() in r.stdout, f'{who} not in --list'
    assert len(rpc.DELEGATED) >= 9, (
        f'only {len(rpc.DELEGATED)} delegated checks declared; the skill has '
        f'at least nine rules a recorded plan cannot answer, and dropping one '
        f'from this list is how it stops being anybody\'s job')
    print(f'  PASS: --list names {len(rpc.RULES)} rules and '
          f'{len(rpc.DELEGATED)} delegations')


TESTS = (t_the_compliant_plan_passes,
         t_each_break_is_refused_by_its_own_rule,
         t_a_break_does_not_redden_unrelated_rules,
         t_the_checker_writes_nothing,
         t_an_unreadable_plan_is_exit_3_not_a_crash,
         t_every_rule_and_every_delegation_is_listed)


def _every_case_is_registered():
    defined = {n for n in globals() if n.startswith('t_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(f'--- {fn.__name__}')
        fn()
    print('\nALL PASS')
