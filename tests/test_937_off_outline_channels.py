#!/usr/bin/env python3
"""#937: check_assembly carries BOTH off-outline channels, and they agree.

`oob_pad_count` is a part-level AABB inflated by the grading clearance;
`oob_pad_copper_*` is the per-PAD measure against the real outline at margin
0, which is the one CLAUDE.md designates for the top-priority placement defect
("a part whose pad copper lies outside the outline... read it off
render_placement's checklist.a_off_outline.pad_copper").

WHY THIS GATE EXISTS AT ALL. The per-pad measure is now computed in TWO
places: `render_placement` computes it from its own proposed-state model, and
`grade_pad_legality` computes it from the file's own poses. They are genuinely
different call sites over different data paths, so one cannot simply call the
other -- and this repo's recorded failure is exactly that shape ("call the
grader, do not mirror it": a re-implementation disagreed 83 times, worst
0.234mm). Where a call is impossible, the AGREEMENT has to be measured instead
of assumed, and that is what the last case does.

The verdict must not move. `check_assembly`'s five `not_buildable` conjuncts
do not include either channel and this change does not add one -- gating on
the coarse count would flip two human reference boards on a measurement
artifact, which is the finding that produced this split in the first place.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RUN_ALL_TIMEOUT = 1200

#: The discriminating pair, plus a control. `rp2350` is the board where both
#: channels fire; `watchy` is the one where the AABB fires on four
#: edge-mounted switches and NO pad crosses the real outline; `tigard` fires
#: neither. A gate that only looked at agreeing boards would pass with the
#: per-pad channel hard-coded to copy the coarse one.
DISCRIMINATING = ('rp2350_fpga_eensy_prePlane', 'watchy', 'tigard')


def _boards():
    r = subprocess.run(['git', 'ls-files', 'kicad_files/*.kicad_pcb'],
                       cwd=ROOT, capture_output=True, text=True)
    return sorted(p for p in r.stdout.split() if p)


def _assembly(board, tmp):
    out = os.path.join(tmp, os.path.basename(board) + '.assembly.json')
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'check_assembly.py'), board,
         '--json', out],
        cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
        errors='replace', timeout=600)
    assert os.path.isfile(out), (
        f'check_assembly wrote no document for {board} (rc={r.returncode})\n'
        f'{r.stderr[-800:]}')
    with open(out, encoding='utf-8') as fh:
        return json.load(fh), r


def t_both_channels_are_published_and_well_formed():
    with tempfile.TemporaryDirectory() as tmp:
        doc, _r = _assembly('kicad_files/tigard.kicad_pcb', tmp)
    for key in ('oob_pad_count', 'oob_pad_refs', 'oob_pad_basis',
                'oob_pad_copper_count', 'oob_pad_copper_refs',
                'oob_pad_copper_basis'):
        assert key in doc, f'{key} is missing from the assembly document'
    assert isinstance(doc['oob_pad_copper_refs'], list)
    assert doc['oob_pad_copper_count'] == len(doc['oob_pad_copper_refs']), \
        'the per-pad count and its ref list disagree'
    assert 'margin 0' in (doc['oob_pad_copper_basis'] or ''), \
        'the per-pad basis string must say which measure it is'
    print('  PASS: both channels published, counts consistent, basis stated')


def t_the_precise_channel_never_names_a_part_the_coarse_one_misses():
    """An invariant, not a coincidence: the coarse outline is the real one
    INFLATED, so copper outside the real outline is necessarily outside the
    inflated one too. If this ever fails, one of the two is wrong."""
    with tempfile.TemporaryDirectory() as tmp:
        bad = []
        for board in _boards():
            doc, _r = _assembly(board, tmp)
            coarse = {r[0] for r in doc.get('oob_pad_refs') or []}
            exact = {r[0] for r in doc.get('oob_pad_copper_refs') or []}
            if exact - coarse:
                bad.append(f'{board}: per-pad names {sorted(exact - coarse)} '
                           f'which the inflated-outline census does not')
    assert not bad, '\n'.join(bad)
    print('  PASS: the per-pad channel is a subset of the coarse one on every '
          'tracked board')


def t_the_verdict_does_not_move():
    """The whole point of splitting the channels rather than gating the
    coarse one. Pinned as an exact census, so a future change that DID make
    an off-outline channel gate has to come here and say so."""
    with tempfile.TemporaryDirectory() as tmp:
        verdicts = {}
        for board in _boards():
            doc, r = _assembly(board, tmp)
            name = os.path.basename(board)[:-len('.kicad_pcb')]
            verdicts[name] = (doc.get('verdict'), r.returncode)
    nb = sorted(n for n, (v, _c) in verdicts.items() if v != 'buildable '
                '(blocking 0)')
    assert nb == ['rp2350_fpga_eensy_prePlane'], (
        f'the NOT BUILDABLE set moved to {nb}. Neither off-outline channel is '
        f'a not_buildable conjunct, and making one would flip human reference '
        f'boards on a measurement artifact -- if that is intended, it is a '
        f'decision to argue for here.')
    for name, (_v, code) in verdicts.items():
        assert code in (0, 4), f'{name} exited {code}'
    print(f'  PASS: {len(verdicts)} boards, 1 NOT BUILDABLE '
          f'({nb[0]}), unchanged')


def t_the_two_implementations_agree():
    """The mirroring guard -- see the module docstring.

    Compared against `render_placement`'s `checklist.a_off_outline.pad_copper`
    on the discriminating pair: a board where both channels fire, one where
    only the coarse one does, and a control where neither does. Amounts are
    compared, not just membership, because a per-pad measure that named the
    right part with the wrong distance would still be wrong.
    """
    with tempfile.TemporaryDirectory() as tmp:
        for name in DISCRIMINATING:
            board = f'kicad_files/{name}.kicad_pcb'
            doc, _r = _assembly(board, tmp)
            js = os.path.join(tmp, f'{name}.render.json')
            png = os.path.join(tmp, f'{name}.render.png')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_tools', 'render_placement.py'),
                 board, '--json-out', js, '-o', png],
                cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                errors='replace', timeout=900)
            assert os.path.isfile(js), (
                f'render_placement wrote no json for {name} '
                f'(rc={r.returncode})\n{r.stderr[-600:]}')
            with open(js, encoding='utf-8') as fh:
                rendered = json.load(fh)
            theirs = ((rendered.get('checklist') or {})
                      .get('a_off_outline') or {}).get('pad_copper') or []
            mine = doc.get('oob_pad_copper_refs') or []
            norm = sorted((str(a), round(float(b), 3)) for a, b in theirs)
            ours = sorted((str(a), round(float(b), 3)) for a, b in mine)
            assert ours == norm, (
                f'{name}: grade_pad_legality says {ours}, render_placement '
                f'says {norm}. These are two implementations of one measure '
                f'and they have diverged.')
            print(f'    {name}: {ours or "[]"} -- both agree')
    print('  PASS: the two implementations agree on the discriminating set')


TESTS = (t_both_channels_are_published_and_well_formed,
         t_the_precise_channel_never_names_a_part_the_coarse_one_misses,
         t_the_verdict_does_not_move,
         t_the_two_implementations_agree)


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
