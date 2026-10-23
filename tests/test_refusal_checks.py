"""The negative-control guard, tested against the five real false-cleans.

A check that can fail for a reason other than the one it is testing will
eventually report a false clean. That is the same defect class the tools keep
exhibiting -- an instrument that can fail two ways and reports one -- except it
is easier to hit in a test, because a test's failure path is the path nobody
looks at.

Every case below is a REAL mistake made while building the deadline/render
work, replayed against `run_utils.check` to prove the helper separates "the
guard held" from "my control was broken".

    python3 -X utf8 tests/test_refusal_checks.py
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

from run_utils import check, evidence, run_step  # noqa: E402

BAD = []


def want(cond, label):
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        BAD.append(label)


def _broken(fn, expect):
    """fn() must raise AssertionError mentioning `expect`."""
    try:
        fn()
    except AssertionError as e:
        return expect in str(e)
    except Exception:                                          # noqa: BLE001
        return False
    return False


def case_import_error_is_not_a_satisfied_guard():
    """Real: a negative control lived in a temp dir with no sys.path entry, so
    it exited 1 from ModuleNotFoundError and was read as 'the gate refused'."""
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, 'ctl.py')
        open(p, 'w', encoding='utf-8').write('import kicad_parser\n')
        want(_broken(lambda: check([sys.executable, p], refuse='PARTITION'),
                     'BROKEN TEST'),
             'an ImportError is reported as a broken test, not a refusal')


def case_wrong_function_name_is_not_absence():
    """Real: a probe called a function that does not exist and concluded the
    feature was missing."""
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, 'probe.py')
        open(p, 'w', encoding='utf-8').write(
            'import json\n'
            'json.no_such_function()\n')
        want(_broken(lambda: check([sys.executable, p], refuse='did NOT run'),
                     'BROKEN TEST'),
             'an AttributeError is reported as a broken test, not "absent"')


def case_missing_evidence_is_caught_before_the_run():
    """Real: evidence passed as <(echo '{...}'), whose fd is gone by the time a
    native Windows child opens it -- the stage refused for an unrelated reason
    and the test read that refusal as its answer."""
    want(_broken(lambda: evidence('/dev/fd/63', 'the drc json'),
                 'does not exist'),
         'evidence() refuses a path that is not a real file')
    with tempfile.TemporaryDirectory() as tmp:
        e = os.path.join(tmp, 'empty.json')
        open(e, 'w').close()
        want(_broken(lambda: evidence(e, 'the drc json'), 'is empty'),
             'evidence() refuses an empty file')


def case_a_guard_that_does_not_hold_is_caught():
    """The other direction: exit 0 must never read as a refusal."""
    want(_broken(lambda: check([sys.executable, '-c', 'pass'],
                               refuse='anything'),
                 'expected a REFUSAL, got exit 0'),
         'a command that SUCCEEDS is never mistaken for a refusal')


def case_right_failure_wrong_reason_is_caught():
    """A refusal that happens for a different reason than the one asserted."""
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, 'r.py')
        open(p, 'w', encoding='utf-8').write(
            'import sys\n'
            'sys.stderr.write("refused: the board carries copper\\n")\n'
            'sys.exit(3)\n')
        want(_broken(lambda: check([sys.executable, p],
                                   refuse='unlocked_high'),
                     'NOT for the stated reason'),
             'a refusal for the WRONG reason fails the check')
        # ...and the same command passes when the reason matches.
        r = check([sys.executable, p], refuse='carries copper', code=3)
        want(r.returncode == 3,
             'the same refusal passes when the reason and code both match')


def case_it_works_on_the_real_guards():
    """Against a real refusal in a shipped tool, not a synthetic script.

    plan-pcb-placement's make_unplaced refuses a routed input with exit 3 and
    names the strip tool, before writing anything.
    """
    D = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts',
                     'make_unplaced.py')
    src = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'pile.kicad_pcb')
        check([sys.executable, '-X', 'utf8', D, src, out],
              refuse='strip_copper_only.py', code=3)
        want(not os.path.exists(out),
             'make_unplaced refuses a routed input and writes nothing')
        try:
            check([sys.executable, '-X', 'utf8', D, src, out],
                  refuse='some other reason', code=3)
            want(False, 'a real refusal for another reason must not pass')
        except AssertionError:
            want(True, 'a real refusal for the WRONG stated reason fails the check')

def case_run_step_asserts_what_run_did_not():
    """`run_utils.run` printed a failing step and carried on, so the chain
    tests built on it (test_interf_u, test_kit_route) were green whatever
    their steps did. `run_step` must fail each way a step can fail."""
    with tempfile.TemporaryDirectory() as tmp:
        def script(name, body):
            path = os.path.join(tmp, name)
            open(path, 'w', encoding='utf-8').write(body)
            return path
        out = os.path.join(tmp, 'out.kicad_pcb')
        writes = script('writes.py', 'import sys\n'
                        'open(sys.argv[1], "w").write("(kicad_pcb)")\n')
        rc, _ = run_step([writes, out], outputs=[out])
        want(rc == 0, 'a step that exits 0 and writes its output passes')
        want(_broken(lambda: run_step([script('fails.py',
                                              'raise SystemExit(1)\n')]),
                     'exited 1'),
             'a step that exits 1 FAILS -- what run() printed and ignored')
        verdict = script('verdict.py', 'print("FOUND 3"); raise SystemExit(1)\n')
        rc, _ = run_step([verdict], findings_ok=True)
        want(rc == 1, "a checker's exit-1 verdict passes under findings_ok")
        want(_broken(lambda: run_step([script('two.py',
                                              'raise SystemExit(2)\n')],
                                      findings_ok=True), 'exited 2'),
             'findings_ok accepts a verdict, never another failure')
        crash = script('crash.py', 'import sys\n'
                       'try:\n    {}["k"]\n'
                       'except KeyError:\n'
                       '    import traceback; traceback.print_exc()\n')
        want(_broken(lambda: run_step([crash]), 'Traceback'),
             'a traceback that still exits 0 is a FAILED step')
        missing = os.path.join(tmp, 'never.kicad_pcb')
        want(_broken(lambda: run_step([script('noop.py', 'pass\n')],
                                      outputs=[missing]), 'does not exist'),
             'a step that does not write its output FAILS')
        want(_broken(lambda: run_step([script('noop2.py', 'pass\n')],
                                      outputs=[out]), 'predates'),
             'an output left by an EARLIER run is not evidence of this one')


if __name__ == '__main__':
    for fn in (case_import_error_is_not_a_satisfied_guard,
               case_wrong_function_name_is_not_absence,
               case_missing_evidence_is_caught_before_the_run,
               case_a_guard_that_does_not_hold_is_caught,
               case_right_failure_wrong_reason_is_caught,
               case_it_works_on_the_real_guards,
               case_run_step_asserts_what_run_did_not):
        print(fn.__name__)
        try:
            fn()
        except AssertionError as e:
            want(False, f'{fn.__name__} raised: {str(e)[:200]}')
    print('FAIL: %d' % len(BAD) if BAD else 'OK')
    sys.exit(1 if BAD else 0)
