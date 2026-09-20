#!/usr/bin/env python3
"""L5 verifies BEFORE it records, and the paths in its text are the ones its
command reads (#904).

Source order IS the doctrine here. The close-out boundary verification decides
the `--final` row's lens lines, so a text that prints the record command first
can only be obeyed by writing the row twice -- and an append-only ledger with
two close-outs has recorded a disagreement, not a verdict. Measured (run 25):
the final row was written, the verifier returned `VERDICT=FAIL:lens=spec` on a
clause the score already showed as `impedance 1`, and the row had to be
re-recorded; two attempts were refused first.

The second half is the one a reordering alone does not fix, and the one a
hostile reading found: the text used to say "paste each line verbatim" above a
command that reads FILES. Nothing created those files, so the command L5 printed
was refused by L5's own converge. `test_converge.py`'s runs-as-printed pin
cannot catch that -- it writes the files itself before executing, which proves
the command's SHAPE and never that the run produces its inputs. So this file
asserts the text and the command agree on the paths, by parsing both.
"""
import os
import re

import sys

RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, '.claude', 'skills',
                       'plan-pcb-placement-and-routing', 'scripts')
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, ROOT)
import loop_driver as L                                       # noqa: E402


def _terminal_text(name='DONE-EXHAUSTED'):
    """L5's terminal emission, built the way `--dump-all` builds it."""
    import json
    import tempfile
    from board_store import sha256_file
    td = tempfile.mkdtemp()
    bd = os.path.join(td, 'b.kicad_pcb')
    with open(bd, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb)')
    sha = sha256_file(bd)
    rows = ([{'kind': 'placement', 'accepted': True, 'result_sha': sha,
              'score': {'blocking': 0, 'quality': {}}}] * 6
            + [{'kind': 'completion', 'accepted': True, 'result_sha': sha,
                'score': {'blocking': 0, 'quality': {}}}] * 6)
    lp = os.path.join(td, 'l.jsonl')
    with open(lp, 'w', encoding='utf-8') as fh:
        for i, r in enumerate(rows):
            fh.write(json.dumps(dict(r, iteration=i)) + '\n')

    def wrote(nm, doc):
        p = os.path.join(td, nm)
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh)
        return p

    close = {'schema': 1, 'kind': 'board-complete', 'board': bd,
             'score': {'blocking': 0},
             'components': {'orphan_stubs': {'ran': True}},
             'fab_floors': {'ran': True, 'relaxed': []},
             'verdict': 'DONE', 'reason': 'fixture', 'ungraded': []}
    a = L._args(['--board', bd, '--ledger', lp, '--budget', '100',
                 '--routing-close', wrote('c.json', close),
                 '--score', wrote('s.json', {'blocking': 0})])
    out = L.STAGES['L5'](a)
    assert not out.startswith('<error>'), out[:400]
    return out


def test_the_verifier_is_dispatched_before_the_row_it_decides():
    text = _terminal_text()
    marks = {
        'verifier': text.index('<subagent_prompt'),
        'record': text.index('--final --stop-condition'),
        'done': text.index('DONE marker'),
        'report': text.index('Report LAST'),
        # #963: the SECOND marker, and it is last for the same reason the
        # report is -- it is what says the report has stopped moving. Run 29's
        # cheat watcher recorded REPORT.md growing 287 -> 393 lines while it
        # read it, and the finished file is 686.
        'report_done': text.index('REPORT_DONE'),
    }
    order = [k for k, _ in sorted(marks.items(), key=lambda kv: kv[1])]
    assert order == ['verifier', 'record', 'done', 'report',
                     'report_done'], (order, marks)
    # tee_cmd wraps the record, so the wrapper must come first on that line.
    assert text.index('tee_cmd.py') < text.index('py_placer/converge.py record')
    # ...and the old instruction must be gone: it told the executor to paste a
    # line into a command that takes paths.
    assert 'paste each line verbatim' not in text, text[:200]
    print("  PASS: verify -> record -> DONE -> report, with tee_cmd on the "
          "record")


def test_the_text_names_the_files_the_command_reads():
    """The half a reorder does not fix."""
    text = _terminal_text()
    # A SLOT, not the prose that explains it: the paragraph above the command
    # also says "--lens-file", so match the continuation lines of the command.
    cmd = [ln for ln in text.splitlines()
           if ln.strip().startswith('--lens-file')]
    # NOT shlex: on Windows the path is full of backslashes and shlex reads
    # them as escapes ("No closing quotation"). The slot is `--lens-file <path>`
    # with a trailing line-continuation, so take the tail literally.
    read = [ln.split('--lens-file', 1)[1].strip().rstrip('\\').strip()
            for ln in cmd]
    assert len(read) == 3, cmd
    # every path the command reads must appear in the verifier's own block,
    # which is what tells the executor to create it
    block = text[text.index('<subagent_prompt'):text.index('</subagent_prompt>')]
    for p in read:
        assert p in block, (
            f'the close-out reads {p} and nothing in the verifier block tells '
            f'anyone to write it -- the command L5 prints is then refused by '
            f"L5's own converge")
    # ...and the block names a fourth file, for the boundary verification,
    # whose grammar is NOT a lens.
    assert 'verdict_record.txt' in block and 'verdict_record.txt' not in ' '.join(cmd)
    assert 'check=<1-5>' in block, block[-400:]
    print(f"  PASS: the verifier writes the {len(read)} files the close-out "
          f"reads, plus the boundary verdict")


def test_the_paths_follow_the_ledger_and_take_the_cycle_suffix():
    """A fixed `wk/...` literal is wrong twice: outside the work dir on a run
    whose ledger is elsewhere, and overwritten on cycle 2 while cycle 1's row
    still records its sha256."""
    for name in ('verdict_connectivity.txt', 'verdict_drc.txt',
                 'verdict_spec.txt', 'verdict_record.txt'):
        assert name in L._ARTIFACTS, name

    class A:
        ledger = 'somewhere/else/ledger.jsonl'
    _n, paths = L._paths(A())
    assert paths['verdict_spec.txt'] == 'somewhere/else/verdict_spec.txt', paths
    assert L._cyc_name('verdict_spec.txt', 2) == 'verdict_spec_c2.txt'
    print("  PASS: the verdict files live beside the ledger and cycle with it")


def test_the_printed_command_refuses_to_invent_paths():
    try:
        L.final_record_command('l.jsonl', 'b.kicad_pcb', 's.json', 'STUCK', {})
    except ValueError as e:
        assert 'verdict paths' in str(e), e
    else:
        raise AssertionError(
            'final_record_command printed a command with no verdict paths. A '
            'default would point --lens-file outside the run, silently.')
    text = L.final_record_command(
        'l.jsonl', 'b.kicad_pcb', 's.json', 'STUCK',
        {f'verdict_{k}.txt': f'w/verdict_{k}.txt'
         for k in ('connectivity', 'drc', 'spec')})
    assert text.count('--lens-file') == 3 and '--lens ' not in text, text
    assert '--lever' in text, 'the close-out row must not be the one blank row'
    print("  PASS: no verdict paths, no printed command")


def test_the_continue_header_says_which_of_the_two_it_is():
    """"Still improving" and "NOT ANSWERABLE" call for opposite actions.

    The header asserted the first about every half that was not flat, so a half
    that had recorded `--exhausted placement` three times was told, in the
    headline, that it was getting better. converge publishes the per-half `why`
    and has since it was written; nothing here read it.
    """
    import json
    import tempfile
    td = tempfile.mkdtemp()
    lp = os.path.join(td, 'l.jsonl')
    # placement: 5 laps, two of them accepted with no score -> unanswerable.
    # routing: nothing at all -> too-few-laps. Neither is improving.
    rows = ([{'kind': 'placement', 'accepted': True,
              'score': {'blocking': 1, 'quality': {}}}] * 3
            + [{'kind': 'placement', 'accepted': True, 'score': None}] * 2)
    with open(lp, 'w', encoding='utf-8') as fh:
        for i, r in enumerate(rows):
            fh.write(json.dumps(dict(r, iteration=i)) + '\n')
    sp = os.path.join(td, 's.json')
    with open(sp, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'kind': 'board-score', 'blocking': 1,
                   'quality': {}}, fh)
    out = L.STAGES['L5'](L._args(['--board', __file__, '--ledger', lp,
                                  '--score', sp]))
    head = out.splitlines()[1]
    assert 'NOT ANSWERABLE' in head, head
    # The correct header QUOTES the phrase as the thing it is not, so match the
    # assertion form -- a half NAMED as improving -- rather than the words.
    for half in ('placement', 'routing'):
        assert f'{half} is still improving' not in head, (
            'the headline asserted improvement about a half whose plateau is '
            'simply unanswerable: ' + head)
    print("  PASS: the CONTINUE headline reads the per-half `why`")


def test_the_freeze_row_is_recorded_as_systemic():
    """A freeze turns no lap, and as a placement row it retracted the
    declaration before it."""
    import json
    import tempfile
    td = tempfile.mkdtemp()
    # L2 refuses a board that is not in the ledger BY CONTENT, so record one.
    from board_store import sha256_file
    bd = os.path.join(td, 'placed.kicad_pcb')
    with open(bd, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb)')
    lp = os.path.join(td, 'l.jsonl')
    with open(lp, 'w', encoding='utf-8') as fh:
        fh.write(json.dumps({'iteration': 0, 'kind': 'placement',
                             'accepted': True,
                             'result_sha': sha256_file(bd)}) + '\n')
    rp = os.path.join(td, 'p.json')
    with open(rp, 'w', encoding='utf-8') as fh:
        json.dump({'blocking': 0, 'oob_pad_count': 0, 'buildable': True,
                   'verdict': 'buildable (blocking 0)', 'locked_contacts': 0,
                   'pad_conflicts': 0, 'hole_conflicts': 0, 'clearance': 0.2,
                   'new_advisory_pairs': [], 'advisory_pairs': 0}, fh)
    out = L.STAGES['L2'](L._args(
        ['--board', bd, '--ledger', lp, '--placement-report', rp]))
    i = out.find('L2 freeze')
    assert i > 0, out[:300]
    cmd = out[max(0, i - 400):i]
    assert '--kind systemic' in cmd, (
        'the freeze row is prescribed as a placement lap again: it then enters '
        "the placement half's window carrying no score AND retracts the "
        '--exhausted declaration before it.\n' + cmd)
    assert '--kind placement \\' not in cmd, cmd
    print("  PASS: L2 prescribes the freeze as --kind systemic")


def test_the_stage_text_is_on_disk_after_every_invocation():
    """`out_sha` proves two invocations emitted the same thing and nothing
    else. The refusal a run was given had to be recoverable from a tee."""
    import contextlib
    import hashlib
    import io as _io
    import json
    import tempfile
    td = tempfile.mkdtemp()
    lp = os.path.join(td, 'l.jsonl')
    open(lp, 'w').close()
    buf = _io.StringIO()
    for _ in range(2):
        with contextlib.redirect_stdout(buf):
            L.main(['--stage', 'L1', '--board', 'b.kicad_pcb', '--ledger', lp])
    with open(os.path.join(td, 'loop_driver.log'), encoding='utf-8') as fh:
        rows = [json.loads(line) for line in fh]
    assert len(rows) == 2, rows
    assert [r['out_file'] for r in rows] == [
        'logs/loop_driver_L1_1.log', 'logs/loop_driver_L1_2.log'], rows
    for r in rows:
        with open(os.path.join(td, r['out_file']), encoding='utf-8') as fh:
            txt = fh.read()
        assert txt.strip(), 'the archive is EMPTY -- a file that proves nothing'
        assert txt.startswith('<stage_instructions'), txt[:80]
        assert hashlib.sha256(txt.encode('utf-8')).hexdigest()[:16] == \
            r['out_sha'], 'the archived text is not the text the row hashed'
        assert len(txt.splitlines()) == r['out_lines'], r
    print("  PASS: the emitted text is archived, and it is the text hashed")


def _cross(rows, verdict_files=(), accept=(), close_verdict='DONE',
           name='DONE-EXHAUSTED'):
    """Drive `_cross_check` on a fixture ledger; return its refusal or None."""
    import json
    import tempfile
    td = tempfile.mkdtemp()
    lp = os.path.join(td, 'l.jsonl')
    with open(lp, 'w', encoding='utf-8') as fh:
        for i, r in enumerate(rows):
            fh.write(json.dumps(dict(r, iteration=i)) + '\n')
    files = []
    for lens, line in verdict_files:
        p = os.path.join(td, f'verdict_{lens}.txt')
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(line + '\n')
        files.append(p)

    class A:
        ledger = lp
        verifier_verdict = files or None
        accept_unclosed = list(accept) or None
    doc = {'verdict': close_verdict, 'reason': 'fixture'}
    return L._cross_check(A(), name, doc)


def test_a_verifier_file_that_disagrees_with_the_ledger_refuses():
    good = {'kind': 'completion', 'accepted': True, 'final': True,
            'score': {'blocking': 0, 'quality': {}},
            'lenses': ['VERDICT=PASS:lens=spec']}
    assert _cross([good]) is None, 'a fixture with nothing to compare must pass'

    # the run-25 shape: the row says PASS, the verifier's file says FAIL
    out = _cross([good], [('spec', 'VERDICT=FAIL:lens=spec;finding=impedance 1;'
                                   'evidence=score.json')])
    assert out and 'TWO INSTRUMENTS DISAGREE' in out, out
    assert 'verdict_spec.txt' in out, out

    # ...and the mirror: a stale PASS file waving through a recorded FAIL
    bad = dict(good, lenses=['VERDICT=FAIL:lens=spec;finding=x;evidence=y'])
    assert _cross([bad], [('spec', 'VERDICT=PASS:lens=spec')], name='STUCK',
                  close_verdict='INCOMPLETE')

    # a verdict that never reached the ledger at all -- a lost reply
    out = _cross([good], [('drc', 'VERDICT=PASS:lens=drc')])
    assert out and 'never reached' in out, out

    # a boundary verdict is not a lens and says so
    out = _cross([good], [('record', 'VERDICT=FAIL:check=3;finding=x')])
    assert out and 'check=<1-5>' in out, out

    # agreement: the file and the row say the same thing
    assert _cross([good], [('spec', 'VERDICT=PASS:lens=spec')]) is None
    print("  PASS: the file on disk is compared with the row, four ways")


def test_the_two_waiver_tokens_do_not_cover_for_each_other():
    """The entire reason for a second token, and one deleted line from false."""
    false_pass = {'kind': 'completion', 'accepted': True, 'final': True,
                  'score': {'blocking': 32,
                            'blocking_by': {'unrouted': 32, 'broken': 0}},
                  'lenses': ['VERDICT=PASS:lens=connectivity']}
    honest = {'kind': 'completion', 'accepted': True, 'final': True,
              'score': {'blocking': 0, 'quality': {}},
              'lenses': ['VERDICT=PASS:lens=spec']}
    vf = [('spec', 'VERDICT=FAIL:lens=spec;finding=x;evidence=y')]

    assert _cross([false_pass]) is not None, 'the score pair must still fire'
    assert _cross([false_pass], accept=['agreement']) is None
    assert _cross([false_pass], accept=['verifier']) is not None, (
        '`verifier` waived a score-vs-row disagreement it has nothing to do '
        'with -- that is the blanket waiver this token was split off to avoid')

    assert _cross([honest], vf) is not None
    assert _cross([honest], vf, accept=['verifier']) is None
    assert _cross([honest], vf, accept=['agreement']) is not None, (
        '`agreement` waived a --verifier-verdict disagreement; a waiver granted '
        'for a spurious check_complete pair would then silently override the '
        'verifier too')
    assert 'verifier' in L.CLOSE_CHECKS, L.CLOSE_CHECKS
    print("  PASS: agreement and verifier waive their own bucket and no other")


def test_the_waiver_vocabularies_in_the_text_are_the_real_ones():
    """A waivers line copied from a stale comment omits a token. `L2_CHECKS`
    has five members while the comment above it says "the four measurements"."""
    text = _terminal_text()
    for flag in ('--accept-residue', '--accept-unclosed', '--accept-congestion',
                 '--accept-incommensurable', '--waive'):
        assert flag in text, flag
    src = open(os.path.join(SCRIPTS, 'loop_driver.py'), encoding='utf-8').read()
    m = re.search(r'#: The (\w+) measurements L2 reads', src)
    assert m, 'the L2_CHECKS comment no longer states a count'
    words = {'four': 4, 'five': 5, 'three': 3, 'six': 6}
    assert words.get(m.group(1)) == len(L.L2_CHECKS), (
        f'the comment says {m.group(1)} and L2_CHECKS has {len(L.L2_CHECKS)}: '
        f'{L.L2_CHECKS}. A waivers line copied from the comment omits one.')
    print(f"  PASS: five waiver channels named, and L2_CHECKS is "
          f"{len(L.L2_CHECKS)} as its comment says")


# --- #963: the verifier's files, FOUND rather than named --------------------
#
# `--verifier-verdict` was passed on 0 of run 29's 7 L5 calls while 17
# `verdict_*.txt` sat on disk, so the gate that compares a verifier's own file
# against the record could not fire. The paths were never a mystery; they had
# to be typed, and were not.
#
# The trap these cases exist for: `live` -- the per-lens map of `--final` rows
# -- is EMPTY at the first L5 of any close-out, BY CONSTRUCTION, because that
# stage's own text is what tells the operator to write the `--final` row (run
# 29's two final rows are iterations 44 and 46, both after its last L5 call).
# So a discovered file fed through the explicit flag's "no final row mentions
# this lens" arm would print three refusals on every honest close-out.


def _discovered(rows, files, close_verdict='DONE', name='DONE-EXHAUSTED',
                accept=(), cycle_suffix=''):
    """Drive discovery + `_cross_check` with NOTHING named on the command line.

    `files` is {basename: line}, written into the ledger's own directory --
    which is where `_paths` looks, and the point of the whole change.
    """
    import json
    import tempfile
    td = tempfile.mkdtemp()
    lp = os.path.join(td, 'l.jsonl')
    with open(lp, 'w', encoding='utf-8') as fh:
        for i, r in enumerate(rows):
            fh.write(json.dumps(dict(r, iteration=i)) + '\n')
    for base, line in files.items():
        with open(os.path.join(td, base), 'w', encoding='utf-8') as fh:
            fh.write(line + '\n')

    class A:
        ledger = lp
        board = None
        verifier_verdict = None
        accept_unclosed = list(accept) or None
    a = A()
    a._discovered_verdicts, a._absent_verdicts = L._discover_verdicts(a)
    a._td = td          # so a caller can hash the file AS WRITTEN
    doc = {'verdict': close_verdict, 'reason': 'fixture'}
    return a, L._cross_check(a, name, doc)


def _final(lens, verdict='PASS', blocking=0, sources=None):
    row = {'kind': 'completion', 'accepted': True, 'final': True,
           'score': {'blocking': blocking, 'quality': {}},
           'lenses': ['VERDICT=%s:lens=%s' % (verdict, lens)]}
    if sources is not None:
        row['lens_source'] = sources
    return row


def test_discovery_reads_the_cycle_map_not_a_glob():
    """Run 29's own strays: 8 of its 17 verdict files are not routing lenses.

    `legality`, `provenance`, `not-run`, `closeout` are placement-half and
    ad-hoc verdicts, every one a well-formed VERDICT= line that no ROUTING
    close-out should mention. A glob would have turned eight honest files into
    eight refusals -- and `verdict_record.txt`, which IS in `_ARTIFACTS`,
    spells `check=<1-5>` and would fire the "not a lens verdict" arm on every
    single close-out.
    """
    strays = {
        'verdict_legality.txt': 'VERDICT=FAIL:lens=legality;finding=x;evidence=y',
        'verdict_provenance.txt': 'VERDICT=FAIL:lens=provenance;finding=x;evidence=y',
        'verdict_notrun.txt': 'VERDICT=FAIL:lens=not-run;finding=x;evidence=y',
        'verdict_closeout.txt': 'VERDICT=FAIL:lens=closeout;finding=x;evidence=y',
        'verdict_record.txt': 'VERDICT=FAIL:check=3;finding=x;evidence=y',
    }
    a, refusal = _discovered([_final('spec')], strays)
    seen = sorted(os.path.basename(d['path']) for d in a._discovered_verdicts)
    assert seen == [], (
        'discovery picked up files outside the cycle map: %s' % seen)
    assert refusal is None, refusal
    print("  PASS: the strays on run 29's disk are not this cycle's lenses")


def test_discovery_does_not_refuse_a_lens_the_record_has_not_reached():
    """The shape run 29 was in at 13:52, and the likeliest regression.

    Three verdict files on disk, no `--final` row yet -- which is every
    close-out's first L5. Feeding these through the explicit flag's arm would
    print three refusals about a record this very stage is about to ask for.
    """
    files = {'verdict_connectivity.txt': 'VERDICT=PASS:lens=connectivity',
             'verdict_drc.txt': 'VERDICT=PASS:lens=drc',
             'verdict_spec.txt': 'VERDICT=PASS:lens=spec'}
    a, refusal = _discovered([], files)
    assert len(a._discovered_verdicts) == 3, a._discovered_verdicts
    assert refusal is None, (
        'a discovered verdict with no ledger claim yet was refused:\n'
        + str(refusal)[:400])
    print("  PASS: a file the record has not reached yet is not a contradiction")


def test_an_explicit_flag_still_refuses_a_verdict_the_record_never_carries():
    """The row that proves the arm was NARROWED, not deleted.

    Naming a file is the operator's claim that its verdict is already on the
    record; finding one is evidence that the record is not written yet. Those
    are different questions and they must not share a refusal.
    """
    out = _cross([_final('spec')],
                 verdict_files=[('drc', 'VERDICT=PASS:lens=drc')])
    assert out and 'never reached' in out, out
    print("  PASS: an explicitly named verdict still needs a row behind it")


def test_the_honest_fix_and_re_dispatch_path_is_not_refused():
    """The regression a first cut of #963 shipped, and why it is a NOTE now.

    L5's own refusal text says "fix what the close-out names and re-score,
    re-dispatch the lens that disagrees". Walk that: the ledger holds a
    `--final` row carrying the OLD verdict while the re-dispatched file on
    disk carries the new one. Refusing that pair refuses the remedy, and the
    refusal did not even carry the `record --final` command that would clear
    it.

    The premise it rested on -- "live is empty at the first L5" -- is true of
    a run with ONE close-out and false of any run that corrected itself:
    wk/run25/esp_prog/ledger.jsonl has three `final: True` rows.
    """
    files = {'verdict_spec.txt': 'VERDICT=PASS:lens=spec'}
    rows = [_final('spec', verdict='FAIL')]
    a, refusal = _discovered(rows, files, close_verdict='INCOMPLETE',
                             name='STUCK')
    assert refusal is None, (
        'the fix-and-re-dispatch path was refused:\n' + str(refusal)[:400])
    notes = '\n'.join(a._discovered_notes)
    assert 'the record is behind the file' in notes, notes
    print("  PASS: a re-dispatched lens is reported, not refused")


def test_a_file_whose_bytes_changed_after_the_row_quoted_it_is_reported():
    """Freshness by CONTENT, never by mtime.

    `--deadline` was removed from this toolchain because no result may depend
    on timing, and mtime is measurably unreliable on these very artifacts: run
    29's DONE marker reports an mtime 29 minutes after it was first written.
    The row already stores each lens file's sha256, so the honest question is
    whether the bytes are the ones it quoted.
    """
    files = {'verdict_drc.txt': 'VERDICT=PASS:lens=drc'}
    src = [{'path': 'verdict_drc.txt', 'sha256': 'f' * 64, 'line': 1}]
    a, refusal = _discovered([_final('drc', sources=src)], files)
    assert refusal is None, refusal
    notes = '\n'.join(a._discovered_notes)
    assert 'quoted it at sha' in notes and 'it is' in notes, notes
    # ...and the SAME row with the real sha is not a finding. The digest is
    # read off the file AS WRITTEN rather than computed from the string: the
    # helper writes in text mode, so on Windows those bytes carry CRLF and a
    # hand-computed sha is a different file's.
    import hashlib
    with open(os.path.join(a._td, 'verdict_drc.txt'), 'rb') as _fh:
        real = hashlib.sha256(_fh.read()).hexdigest()
    src_ok = [{'path': 'verdict_drc.txt', 'sha256': real, 'line': 1}]
    _a2, ok = _discovered([_final('drc', sources=src_ok)], files)
    assert ok is None, ok
    assert not [n for n in _a2._discovered_notes if 'quoted it at sha' in n], \
        _a2._discovered_notes

    # ...and a row quoting a file of the SAME NAME in ANOTHER DIRECTORY is not
    # this file. Matching on basename alone called it "this same file"; run
    # 24's layout puts the lens files in a per-turn subdirectory, so that is
    # not hypothetical. `lens_source.abspath` exists for exactly this and was
    # unread.
    _a3, other = _discovered(
        [_final('drc', sources=[{'path': 'turn1/verdict_drc.txt',
                                 'abspath': '/elsewhere/turn1/verdict_drc.txt',
                                 'sha256': 'f' * 64, 'line': 1}])], files)
    assert other is None, other
    assert not [n for n in _a3._discovered_notes if 'quoted it at sha' in n], (
        'a file in another directory was called "this same file": '
        + str(_a3._discovered_notes))
    print("  PASS: bytes that moved are named; another directory's are not")


def test_a_discovered_fail_beside_two_finished_instruments_refuses():
    """The one arm discovery gets that needs no --final row.

    Without it, discovery reports and never binds, which is decoration.
    Calibration: this would NOT have fired on run 29, whose stop was condition
    4 and whose close-out was not DONE -- its defect was that nobody read the
    files, not that the files lied.
    """
    files = {'verdict_drc.txt': 'VERDICT=FAIL:lens=drc;finding=short;evidence=x'}
    _a, refusal = _discovered([], files, close_verdict='DONE',
                              name='DONE-EXHAUSTED')
    assert refusal and 'on disk beside them' in refusal, refusal
    # and NOT on a run that is not claiming to be finished
    _a2, ok = _discovered([], files, close_verdict='INCOMPLETE', name='STUCK')
    assert ok is None, ok
    print("  PASS: a FAIL on disk beside DONE + DONE binds; STUCK does not")


def test_the_terminal_text_reports_found_and_absent():
    """Both words, not either.

    `tests/test_431_skill_commands.py:1152` records a pin written as an `or`
    of two phrases, which passed with either one deleted. A report that names
    only what it found reads as a clean bill when three files are missing.

    Asserted on `--dump-all`, which RENDERS the arm, rather than on the source:
    counting an f-string in the file proves the call site exists and says
    nothing about whether a reader ever sees the line.
    """
    import subprocess
    import sys as _sys
    driver = os.path.join(SCRIPTS, 'loop_driver.py')
    r = subprocess.run([_sys.executable, '-X', 'utf8', driver, '--dump-all'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=900)
    assert r.returncode == 0, r.stderr[-400:]
    assert 'VERDICT FILES' in r.stdout, 'the discovery report is not rendered'
    # ON THE REPORT LINE, not anywhere in the dump. Asserting `'found' in
    # r.stdout` passed with the found half DELETED, because "found" appears
    # five more times in the surrounding prose -- the same or-shaped pin this
    # docstring cites test_431 for, rebuilt one line lower.
    line = [ln for ln in r.stdout.splitlines() if 'VERDICT FILES' in ln][0]
    assert 'FOUND' in line, ('the report names no found half: ' + line)
    assert 'ABSENT' in line, ('the report names no absent half: ' + line)
    assert 'cycle' in line, (
        'the cycle number is what makes a misnamed _c<n> file visible; '
        '_paths takes the highest of the ledger index and what is on disk: '
        + line)
    print("  PASS: the report LINE names found, absent and the cycle")


def test_l5_itself_discovers_the_files_with_nothing_named():
    """Through the real stage, because `_cross_check` alone proves too little.

    A mutation battery measured four rows SURVIVING every gate here --
    including `_discover_verdicts` returning nothing at all, and discovery
    writing into `a.verifier_verdict` -- because every test drove
    `_cross_check` with a hand-built namespace. Nothing asserted that l5
    CALLS discovery, which is the claim the whole change rests on.
    """
    import json as _json
    import tempfile as _tf
    td = _tf.mkdtemp()
    board = os.path.join(td, 'b.kicad_pcb')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb)\n')
    sys.path.insert(0, ROOT)
    from board_store import sha256_file
    sha = sha256_file(board)
    led = os.path.join(td, 'l.jsonl')
    row = {'kind': 'completion', 'accepted': True, 'result_sha': sha,
           'score': {'blocking': 0, 'quality': {}}}
    with open(led, 'w', encoding='utf-8') as fh:
        for i, r in enumerate([dict(row, kind='placement')] * 6 + [row] * 6):
            fh.write(_json.dumps(dict(r, iteration=i)) + '\n')
    score = os.path.join(td, 's.json')
    with open(score, 'w', encoding='utf-8') as fh:
        _json.dump({'blocking': 0, 'quality': {}, 'ungraded': [],
                    'board_sha': sha}, fh)
    for lens in ('connectivity', 'drc'):
        with open(os.path.join(td, f'verdict_{lens}.txt'), 'w',
                  encoding='utf-8') as fh:
            fh.write(f'VERDICT=PASS:lens={lens}\n')

    close = os.path.join(td, 'c.json')
    with open(close, 'w', encoding='utf-8') as fh:
        _json.dump({'schema': 1, 'kind': 'board-complete', 'board': board,
                    'score': {'blocking': 0},
                    'components': {'orphan_stubs': {'ran': True}},
                    'fab_floors': {'ran': True, 'relaxed': []},
                    'verdict': 'DONE', 'reason': 'fixture', 'ungraded': []}, fh)
    a = L._args(['--stage', 'L5', '--board', board, '--ledger', led,
                 '--score', score, '--routing-close', close])
    out = L.STAGES['L5'](a)
    assert getattr(a, '_discovered_verdicts', None), (
        'l5 did not call discovery at all -- every other test here drives '
        '_cross_check with a hand-built namespace and cannot see that')
    found = sorted(os.path.basename(d['path'])
                   for d in a._discovered_verdicts)
    assert found == ['verdict_connectivity.txt', 'verdict_drc.txt'], found
    assert a.verifier_verdict is None, (
        'discovery filled --verifier-verdict, so every refusal will name a '
        'flag nobody passed')
    assert 'VERDICT FILES' in out, out[:400]
    assert 'spec' in out.split('VERDICT FILES', 1)[1][:200], (
        'the absent lens is not named: ' + out.split('VERDICT FILES', 1)[1][:200])
    print("  PASS: the real stage discovers, reports, and names the absent one")


def test_the_verifier_waiver_covers_discovered_files_too():
    """One vocabulary per gate, and the discovered files are that gate's."""
    files = {'verdict_drc.txt': 'VERDICT=FAIL:lens=drc;finding=x;evidence=y'}
    _a, refusal = _discovered([], files)
    assert refusal, 'the fixture stopped refusing; this test now proves nothing'
    _a2, waived = _discovered([], files, accept=('verifier',))
    assert waived is None, waived
    _a3, other = _discovered([], files, accept=('agreement',))
    assert other, ('--accept-unclosed agreement cleared the verifier bucket, '
                   'which is the compounding hazard the two tokens were split '
                   'apart for')
    print("  PASS: the verifier waiver covers them and the agreement one does not")


def test_the_report_reaches_the_CONTINUE_branch_too():
    """The branch the measured run actually took, on every call it made.

    The first cut printed the verdict-file report on the terminal branch
    alone, to keep prose off the hot path. Run 29 -- the run this item exists
    for -- took CONTINUE on all seven of its L5 calls and never reached a
    terminal arm, so that report is one its own case would not have seen. A
    round-2 verifier measured it: three discovered files, three notes
    computed, and `VERDICT FILES` nowhere in the text.

    Asserted through the REAL stage, and on the NOTES as well as the header,
    because the notes are what carries a disagreement -- a header with no
    notes is a report that found nothing to say.
    """
    import json as _json
    import tempfile as _tf
    td = _tf.mkdtemp()
    board = os.path.join(td, 'b.kicad_pcb')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb)\n')
    sys.path.insert(0, ROOT)
    from board_store import sha256_file
    sha = sha256_file(board)
    led = os.path.join(td, 'l.jsonl')
    # ONE routing lap, so the unclassified-retry gate does not fire first --
    # this test is about the report, and a refusal would hide it.
    rows = [{'kind': 'placement', 'accepted': True, 'result_sha': sha,
             'score': {'blocking': 2, 'quality': {}}},
            {'kind': 'completion', 'accepted': True, 'result_sha': sha,
             'score': {'blocking': 2, 'quality': {}}},
            {'kind': 'completion', 'accepted': True, 'final': True,
             'result_sha': sha, 'score': {'blocking': 2, 'quality': {}},
             'lenses': ['VERDICT=PASS:lens=drc']}]
    with open(led, 'w', encoding='utf-8') as fh:
        for i, r in enumerate(rows):
            fh.write(_json.dumps(dict(r, iteration=i)) + '\n')
    score = os.path.join(td, 's.json')
    with open(score, 'w', encoding='utf-8') as fh:
        _json.dump({'blocking': 2, 'quality': {}, 'ungraded': [],
                    'board_sha': sha}, fh)
    # The file on disk says FAIL where iteration 2 recorded PASS: a NOTE, not
    # a refusal, because "fix it and re-dispatch" is the honest path.
    with open(os.path.join(td, 'verdict_drc.txt'), 'w', encoding='utf-8') as fh:
        fh.write('VERDICT=FAIL:lens=drc;finding=x;evidence=y\n')
    a = L._args(['--stage', 'L5', '--board', board, '--ledger', led,
                 '--score', score])
    out = L.STAGES['L5'](a)
    assert not out.startswith('<error>'), (
        'the fixture refused, so it proves nothing about the report:\n'
        + out[:500])
    assert 'not done yet' in out, out[:200]
    assert 'VERDICT FILES' in out, (
        'the CONTINUE branch printed no report -- which is every L5 call run '
        '29 made:\n' + out[:800])
    assert 'verdict_drc.txt' in out, out[:800]
    assert 'NOTE' in out and 'the record is behind the file' in out, (
        'the header reached CONTINUE and the notes did not, so a real '
        'disagreement is still invisible on the hot branch:\n' + out[:900])
    print("  PASS: the CONTINUE branch reports what was found and what it says")


def test_a_file_named_for_one_lens_that_speaks_for_another_is_named():
    """`expected` was stored by discovery and read by nobody.

    `verdict_drc.txt` carrying `VERDICT=PASS:lens=spec` means the cycle map
    went looking for `drc` and got a file that answers a different question --
    so `drc` is covered by NOTHING, while a reader counting files sees three
    of three. A verifier mutated the `expected` comparison away and every test
    passed.
    """
    files = {'verdict_drc.txt': 'VERDICT=PASS:lens=spec'}
    a, refusal = _discovered([], files, close_verdict='INCOMPLETE',
                             name='STUCK')
    notes = '\n'.join(a._discovered_notes)
    assert 'names lens spec, not drc' in notes, notes or '(no notes at all)'
    assert 'covered by no file' in notes, notes
    # ...and the honest case stays quiet.
    _a2, _ = _discovered([], {'verdict_drc.txt': 'VERDICT=PASS:lens=drc'},
                         close_verdict='INCOMPLETE', name='STUCK')
    assert not [n for n in _a2._discovered_notes if 'names lens' in n], \
        _a2._discovered_notes
    print("  PASS: a file answering a different lens is named as a gap")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
