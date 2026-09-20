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
    # THE OUTPUT, not just the exit code. Exiting at DONE -- the pre-#963
    # contract, and the defect -- also returns 0 and also ends the thread, so
    # a test asserting only those two passes on the mutant. A mutation row
    # measured exactly that.
    import contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        t, box = _cheats(td, report_done=os.path.join(td, 'REPORT_DONE'),
                         report_wait=5)
        t.join(timeout=60)
    out = buf.getvalue()
    assert not t.is_alive(), 'the watcher never returned'
    assert box.get('rc') == 0, box
    assert 'DONE declared' in out, out[:400]
    assert 'REPORT audited' in out, (
        'it stopped at DONE: the report audit never ran, which is the whole '
        'of this item:\n' + out[:600])
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


#: Run 29's DONE, in the shape that matters: the shipped board on a
#: `sha256 <digest>` line, and the SUPERSEDED one quoted in the marker's own
#: prose. A first cut of this fixture omitted that second sentence -- and with
#: it omitted, the check passed while being INERT on the real file, because
#: collecting every hex token in DONE put the superseded sha in the shipped
#: set and the report then "agreed" with the marker.
_SHIPPED = 'c22ab32b802e62e64ad11b648e97f6aeda98fd5c8ad78c68c81258867998a5cd'
_SUPERSEDED = '0b2f0d5ede91'
_RUN29_DONE = (
    'DONE -- the copper is frozen.\n\n'
    'board            routed.kicad_pcb   sha256 %s\n\n'
    'THIS MARKER WAS REWRITTEN. An earlier DONE closed on sha %s..., and two\n'
    'findings from the tool_usage watcher improved the board afterwards.\n'
    % (_SHIPPED, _SUPERSEDED))


def test_the_report_audit_names_run_29s_own_defect():
    """The check that makes the second marker not theatre.

    Run 29's REPORT.md:23 -- the "What ships" table, the FIRST table in the
    document -- names a board its own DONE marker calls superseded.
    """
    td = _wd(DONE=_RUN29_DONE,
             REPORT__md='# What ships\n\n| board | %s |\n'
                        'later, the shipped board was %s\n'
                        'FENCE VERDICT: CLEAN (exit 0)\n'
                        'PROVENANCE VERDICT: UNAIDED VIOLATION (exit 4)\n'
                        % (_SUPERSEDED, _SHIPPED))
    lines = RW.report_audit(td, os.path.join(td, 'REPORT.md'),
                            os.path.join(td, 'DONE'), None)
    assert any('SUPERSEDED' in ln for ln in lines), lines
    assert any(ln.startswith('REPORT audited') for ln in lines), lines
    print("  PASS: a report opening on a superseded board is reported")


def test_a_digest_the_marker_never_mentions_is_not_an_accusation():
    """The false-firing half, measured on this repo's own run dirs.

    "The first digest in the report" is a git commit on run 26, and
    synthetically a part number or an ISO-basic date. Measured over the
    repo's own run dirs: 23 `REPORT.md` files, 11 carrying any digest, and
    exactly ONE of those opening on the sha its DONE names as shipped -- so
    the naive rule would accuse ten of eleven. Only a digest the MARKER
    itself names as something other than shipped can produce the finding.
    """
    for opener in ('189f7e27',                       # a git commit (run 26)
                   '20260920',                       # an ISO-basic date
                   'deadbeefcafe'):                  # anything else hex
        td = _wd(DONE=_RUN29_DONE,
                 REPORT__md='# Report\n\nRepo HEAD %s\n'
                            'the shipped board is %s\n'
                            'FENCE VERDICT: CLEAN (exit 0)\n'
                            'PROVENANCE VERDICT: UNAIDED (exit 0)\n'
                            % (opener, _SHIPPED))
        lines = [ln for ln in RW.report_audit(
            td, os.path.join(td, 'REPORT.md'), os.path.join(td, 'DONE'), None)
            if not ln.startswith('REPORT audited')]
        assert not lines, (opener, lines)
    print("  PASS: a digest DONE never mentions is not an accusation")


def test_a_marker_that_names_no_board_says_so():
    """`wk/run20/DONE` is zero bytes, and an inert check reads as agreement."""
    td = _wd(DONE='done\n',
             REPORT__md='FENCE VERDICT: CLEAN (exit 0)\n'
                        'PROVENANCE VERDICT: UNAIDED (exit 0)\n')
    lines = RW.report_audit(td, os.path.join(td, 'REPORT.md'),
                            os.path.join(td, 'DONE'), None)
    assert any('names no `sha256' in ln for ln in lines), lines
    print("  PASS: a marker with no digest is disclosed, not passed")


def test_a_report_that_names_no_shipped_digest_at_all_is_reported():
    td = _wd(DONE=_RUN29_DONE,
             REPORT__md='FENCE VERDICT: CLEAN (exit 0)\n'
                        'PROVENANCE VERDICT: UNAIDED (exit 0)\n')
    lines = RW.report_audit(td, os.path.join(td, 'REPORT.md'),
                            os.path.join(td, 'DONE'), None)
    assert any('does not say which board this is' in ln for ln in lines), lines
    print("  PASS: a report that never names the shipped board is reported")


def test_a_rewritten_done_re_runs_the_board_audits():
    """The branch the change calls load-bearing, and which nothing covered.

    Without it run 29's defect survives this entire item: the audits ran at
    14:22 against a board two ledger rows replaced, and the marker that
    triggers the report audit arrives 63 minutes later.
    """
    td = _wd(DONE='first\n',
             REPORT__md='FENCE VERDICT: CLEAN (exit 0)\n'
                        'PROVENANCE VERDICT: UNAIDED (exit 0)\n')
    stale = RW._sha(os.path.join(td, 'DONE'))
    with io.open(os.path.join(td, 'DONE'), 'w', encoding='utf-8') as fh:
        fh.write('rewritten\n')                    # as run 29 did, 29 min later
    with io.open(os.path.join(td, 'REPORT_DONE'), 'w', encoding='utf-8') as fh:
        fh.write('done\n')
    import contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = RW._await_report(td, os.path.join(td, 'DONE'), stale, None, ROOT,
                              os.path.join(td, 'REPORT_DONE'), 5, 0.02)
    out = buf.getvalue()
    assert rc == 0, out
    assert 'DONE was REWRITTEN between the two audits' in out, out[:500]
    # ...and it RE-RAN them, rather than only saying so.
    assert 'FENCE' in out and 'PROVENANCE' in out, out[:500]
    print("  PASS: a DONE that moved re-runs the audits that read the board")


def test_arm_report_writes_the_marker():
    """The production caller, without which the guard never arms.

    Asserted on the SOURCE rather than by running the tool: arm_report needs a
    real work dir with a routed board and a fence verdict, which is minutes of
    setup for a three-line claim. What is checked is that the write exists, is
    beside the report, and cannot take the tool down.
    """
    src = io.open(os.path.join(ROOT, 'tests', 'stress', 'arm_report.py'),
                  encoding='utf-8').read()
    assert "'REPORT_DONE'" in src, (
        'arm_report writes no marker, so the cheat watcher waits for a file '
        'only an agent ever writes')
    # THE WRITE, not just the name. A mutation that kept `'REPORT_DONE'` in
    # the source and wrote `_marker + '.disabled'` instead survived a test
    # that only looked for the name.
    assert "with open(_marker, 'w'" in src, (
        'the marker name is in the source but nothing writes THAT path')
    i = src.index("'REPORT_DONE'")
    tail = src[i:i + 700]
    assert 'os.path.dirname' in src[i - 200:i + 200], (
        'the marker is not placed beside the report it announces')
    assert 'except OSError' in tail, (
        'a marker that cannot be written must not take the report down -- the '
        'report IS written by then')
    print("  PASS: arm_report writes the marker, beside the report, safely")


def test_the_report_audit_wants_both_verdicts_quoted():
    td = _wd(DONE='done\n',
             REPORT__md='the fence and provenance audits both passed\n')
    lines = RW.report_audit(td, os.path.join(td, 'REPORT.md'),
                            os.path.join(td, 'DONE'), None)
    joined = '\n'.join(lines)
    assert 'FENCE' in joined and 'PROVENANCE' in joined, lines
    assert 'exit code' in joined or 'quotes no verdict' in joined, lines
    # ...and a report that DOES quote them, with codes, is not a finding. Its
    # DONE names a board, because a marker with no digest is a disclosure of
    # its own now and would be counted here as a verdict finding.
    td2 = _wd(DONE=_RUN29_DONE,
              REPORT__md='shipped ' + _SHIPPED + '\n'
                         'FENCE VERDICT: CLEAN (exit 0)\n'
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
    # The flags are checked against the REAL argparse here, by running it.
    # `test_431_skill_commands` does not do it for us: its `TOOLS` skips any
    # path under `tests/`, so run_watch.py is not in that gate's population --
    # measured by inserting a bogus flag into RUNBOOK's block and watching
    # test_431 stay green at "1116 flag citations, all real".
    import subprocess as _sp
    _help = _sp.run([sys.executable, '-X', 'utf8', WATCHER, 'cheats', '--help'],
                    capture_output=True, text=True, cwd=ROOT).stdout
    skill = io.open(SKILL, encoding='utf-8').read()
    assert 'ONE agent with one section per brief' in skill, skill[:200]
    assert 'only specification of the watcher mechanism' in skill, (
        'nothing says the skill is the single spec, which is the whole of '
        "#963's fourth acceptance line")
    assert 'defers to it' in skill
    assert 'REPORT_DONE' in skill, 'the close-out does not name the marker'
    book = io.open(RUNBOOK, encoding='utf-8').read()
    assert 'REPORT_DONE' in book and '--report-wait' in book, (
        'the RUNBOOK command block does not carry the flags')
    # AND THE THREE FILES MUST NOT CONTRADICT EACH OTHER. A first cut left
    # SKILL.md:356 saying the watcher "and then exits" at DONE while :362 said
    # it waits -- one file, two mechanisms, which is the defect this whole item
    # is about, rebuilt six lines apart.
    assert 'and then exits' not in skill, (
        'SKILL.md still says the cheat watcher exits at DONE')
    for flag in ('--report-done', '--report-wait'):
        assert flag in _help, f'{flag} is not a real flag: ' + _help[:300]
    print("  PASS: one spec, no contradiction, and the flags are real")


if __name__ == '__main__':
    run_utils.evidence(SKILL)
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print("--- " + k)
            v()
    print("ALL PASS")
