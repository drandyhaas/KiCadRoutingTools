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
    }
    order = [k for k, _ in sorted(marks.items(), key=lambda kv: kv[1])]
    assert order == ['verifier', 'record', 'done', 'report'], (order, marks)
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


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
