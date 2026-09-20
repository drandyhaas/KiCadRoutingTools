"""`REPORT.md` is inside the audited set now (#963 item 4, second half).

`REPORT.md` is written AFTER `DONE`, and correctly: it has to carry the fence
and provenance verdicts, which do not exist until DONE triggers them. SKILL.md
conceded the consequence in its own words -- "that makes the report the one
artifact the cheat watcher cannot audit". That is the consequence removed, by
keeping the watcher alive past DONE rather than by reordering a run that is not
wrong.

THREE FACTS FROM RUN 29'S OWN ARTIFACTS, none of them in the issue:

  * `DONE` IS NOT ONE-SHOT-SAFE. `run_watch.py`'s `os.path.exists(done_path)`
    fired at 14:22:23 and returned 0. The `DONE` on disk today reads "THIS
    MARKER WAS REWRITTEN. An earlier DONE closed on sha 0b2f0d5e..." and names
    `c22ab32b...` as shipped. So `FENCE VERDICT: CLEAN` and `PROVENANCE
    VERDICT: UNAIDED VIOLATION` in that run's watcher log are verdicts about a
    board superseded 29 minutes later by ledger rows 45 and 46.
  * THE TERMINAL ARTIFACT CONTRADICTS ITSELF AND NOTHING CAUGHT IT.
    `REPORT.md:23` -- the "What ships" table, the FIRST table in the document
    -- names `0b2f0d5e...`, while `:546` concedes that board is superseded and
    `:551` names `c22ab32b...`. A single substring comparison at REPORT time
    finds it.
  * THE AGENT WATCHERS GRADED A MOVING DOCUMENT. `cheats.md:6-9` records
    `REPORT.md` growing 287 -> 393 lines while it was being read, and a journal
    claim falsified 38 seconds after it was written. The final file is 686
    lines: 43% of the terminal artifact was audited by nobody.

WHAT THIS DOES NOT FIX, said here rather than implied: the marker is written by
the run being audited, so a run can write it, be audited, and then extend the
file -- which is exactly what run 29 did to DONE. The audit records the
report's own sha and length so a later reader can tell.
"""
import io
import os
import sys
import tempfile
import threading

RUN_ALL_TIMEOUT = 600

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
import run_utils                                              # noqa: E402
import run_watch as RW                                        # noqa: E402

SKILL = os.path.join(ROOT, '.claude', 'skills',
                     'plan-pcb-placement-and-routing', 'SKILL.md')
RUNBOOK = os.path.join(ROOT, 'tests', 'stress', 'RUNBOOK.md')
WATCHER = os.path.join(ROOT, 'tests', 'stress', 'run_watch.py')


def _wd(**files):
    td = tempfile.mkdtemp()
    for name, text in files.items():
        p = os.path.join(td, name.replace('__', '.'))
        with io.open(p, 'w', encoding='utf-8') as fh:
            fh.write(text)
    return td


def _cheats(td, **kw):
    """Run `watch_cheats` in a thread; return (thread, box) for the exit code."""
    box = {}

    def go():
        box['rc'] = RW.watch_cheats(
            td, None, kw.pop('done', os.path.join(td, 'DONE')),
            kw.pop('poll', 0.02), **kw)
    t = threading.Thread(target=go, daemon=True)
    t.start()
    return t, box


def test_cheats_does_not_exit_at_done_when_a_report_marker_is_expected():
    td = _wd(DONE='done\n', REPORT_DONE='done\n',
             REPORT__md='FENCE VERDICT: CLEAN (exit 0)\n'
                        'PROVENANCE VERDICT: UNAIDED (exit 0)\n')
    t, box = _cheats(td, report_done=os.path.join(td, 'REPORT_DONE'),
                     report_wait=5)
    t.join(timeout=60)
    assert not t.is_alive(), 'the watcher never returned'
    assert box.get('rc') == 0, box
    print("  PASS: it runs the board audits and carries on to the report")


def test_an_old_dir_with_no_marker_still_terminates():
    """Replaying this watcher over a finished run must not hang for 90 minutes.

    The assertion is that the THREAD ended -- a test that only checked the
    printed text would pass while the process sat there forever.
    """
    td = _wd(DONE='done\n')
    t, box = _cheats(td, report_done=os.path.join(td, 'REPORT_DONE'),
                     report_wait=0.2, poll=0.05)
    t.join(timeout=60)
    assert not t.is_alive(), 'the bounded wait did not bound anything'
    assert box.get('rc') == 0, box
    print("  PASS: a missing report marker ends the watch, it does not hang")


def test_report_done_empty_restores_the_exit_at_done_contract():
    td = _wd(DONE='done\n')
    t, box = _cheats(td, report_done='', report_wait=600)
    t.join(timeout=60)
    assert not t.is_alive(), "--report-done '' did not exit at DONE"
    assert box.get('rc') == 0, box
    print("  PASS: the pre-#963 contract is one flag away")


def test_the_report_audit_names_run_29s_own_defect():
    """The check that makes the second marker not theatre.

    Strings from run 29 verbatim: DONE names the shipped board, and the
    report's FIRST digest is the one it superseded.
    """
    td = _wd(DONE='THIS MARKER WAS REWRITTEN. shipped c22ab32b'
                  '00000000000000000000000000000000000000000000000000000000\n',
             REPORT__md='# What ships\n\n| board | 0b2f0d5e'
                        '00000000000000000000000000000000000000000000000000000000 |\n'
                        'FENCE VERDICT: CLEAN (exit 0)\n'
                        'PROVENANCE VERDICT: UNAIDED VIOLATION (exit 4)\n')
    lines = RW.report_audit(td, os.path.join(td, 'REPORT.md'),
                            os.path.join(td, 'DONE'), None)
    assert any('different board' in ln for ln in lines), lines
    assert any(ln.startswith('REPORT audited') for ln in lines), lines
    print("  PASS: a report naming a superseded board is reported")


def test_the_report_audit_wants_both_verdicts_quoted():
    td = _wd(DONE='done\n',
             REPORT__md='the fence and provenance audits both passed\n')
    lines = RW.report_audit(td, os.path.join(td, 'REPORT.md'),
                            os.path.join(td, 'DONE'), None)
    joined = '\n'.join(lines)
    assert 'FENCE' in joined and 'PROVENANCE' in joined, lines
    assert 'exit code' in joined or 'quotes no verdict' in joined, lines
    # ...and a report that DOES quote them, with codes, is not a finding.
    td2 = _wd(DONE='done\n',
              REPORT__md='FENCE VERDICT: CLEAN (exit 0)\n'
                         'PROVENANCE VERDICT: UNAIDED (exit 0)\n')
    ok = RW.report_audit(td2, os.path.join(td2, 'REPORT.md'),
                         os.path.join(td2, 'DONE'), None)
    assert not [ln for ln in ok if not ln.startswith('REPORT audited')], ok
    print("  PASS: a summary is reported; a verbatim quote is not")


def test_a_done_rewritten_between_the_triggers_is_named():
    td = _wd(DONE='done\n',
             REPORT__md='FENCE VERDICT: CLEAN (exit 0)\n'
                        'PROVENANCE VERDICT: UNAIDED (exit 0)\n')
    stale = 'f' * 64
    lines = RW.report_audit(td, os.path.join(td, 'REPORT.md'),
                            os.path.join(td, 'DONE'), stale)
    assert any('REWRITTEN' in ln for ln in lines), lines
    print("  PASS: a DONE that moved between the two audits is named")


def test_logs_label_done_is_not_the_report_marker():
    """`tee_cmd` owns that name family, and a run prompt calls it "the ONLY
    signal that a command finished". Conflating the two would make the watcher
    exit at the first command."""
    td = _wd(DONE='done\n')
    os.makedirs(os.path.join(td, 'logs'), exist_ok=True)
    with io.open(os.path.join(td, 'logs', 'route.done'), 'w',
                 encoding='utf-8') as fh:
        fh.write('done\n')
    t, box = _cheats(td, report_done=os.path.join(td, 'REPORT_DONE'),
                     report_wait=0.2, poll=0.05)
    t.join(timeout=60)
    assert not t.is_alive(), box
    print("  PASS: logs/<label>.done does not satisfy --report-done")


def test_the_docstring_no_longer_claims_there_is_no_time_value():
    """A file that budgets a wait must not say it never compares a time.

    `--report-wait` is a real time value. The old sentence -- "Nothing here
    compares an elapsed time to a threshold and decides something" -- would
    have been a lie the file told about itself.
    """
    src = io.open(WATCHER, encoding='utf-8').read()
    assert 'Neither watcher BUDGETS on time' not in src, (
        'the stale sentence is back')
    assert 'NEITHER WATCHER GRADES ON TIME' in src, (
        'the correction that replaced it is gone')
    assert 'logs/<label>.done' in src, (
        'the two marker families must be told apart where the flag is defined')
    print("  PASS: the docstring says what is true of its own flags")


def test_one_watcher_specification_and_the_run_prompt_defers():
    """AND, not OR.

    `test_431_skill_commands.py:1152` records a pin written as an `or` of two
    phrases that passed with either one deleted. The watcher spec needs both
    halves: the mechanism, and that a run prompt does not restate it.
    """
    skill = io.open(SKILL, encoding='utf-8').read()
    assert 'ONE agent with one section per brief' in skill, skill[:200]
    assert 'only specification of the watcher mechanism' in skill, (
        'nothing says the skill is the single spec, which is the whole of '
        "#963's fourth acceptance line")
    assert 'defers to it' in skill
    assert 'REPORT_DONE' in skill, 'the close-out does not name the marker'
    book = io.open(RUNBOOK, encoding='utf-8').read()
    assert 'REPORT_DONE' in book and '--report-wait' in book, (
        'the RUNBOOK command block does not carry the flags -- and it is in '
        "test_431's SOURCES, so citing them there is checked against the real "
        'argparse for free')
    print("  PASS: one spec, deferred to, and both files name the marker")


if __name__ == '__main__':
    run_utils.evidence(SKILL)
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print("--- " + k)
            v()
    print("ALL PASS")
