#!/usr/bin/env python3
"""#887: `kicad_iso_render`'s machine-independent half -- no kicad-cli needed.

`kicad_iso_render` was the film's iso-panel renderer; the panel is gone
(stage3d is the only film layout) but the module stays as a standalone CLI,
and `stage3d` reuses its `resolve_cli` / `model_dirs`. These tests moved here
from the deleted `test_887_two_panel_frame.py`, which was the killer of
`tests/mutate_887.py`'s 'i' rows on any machine: the 3D-model precheck, the
models note, KIPRJMOD, the board-relative model path, keyed render results
and the unreadable-PNG check -- everything that can be graded with kicad-cli
monkeypatched. `tests/test_887_iso_render.py` is the half that needs a real
kicad-cli, and self-skips without one.

RUN_ALL_FAST_OK: nothing here shells out.
"""
import os
import sys
import tempfile

RUN_ALL_FAST_OK = True

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import kicad_iso_render as kir                             # noqa: E402

BOARD_A = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
TIGARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')

BAD = []


def want(cond, label, extra=''):
    if cond:
        print('  PASS: %s' % label)
    else:
        BAD.append(label)
        print('  FAIL: %s %s' % (label, extra))


def test_the_model_precheck_counts_what_is_on_disk():
    """Machine-independent: no real KiCad tree is read, only temp dirs we made."""
    empty = tempfile.mkdtemp()
    dirs = {v: empty for v in kir._MODEL_DIR_VARS}
    dirs['KIPRJMOD'] = empty
    m = kir.resolve_models(TIGARD, dirs)
    want(m['total'] == 84, 'tigard references 84 models', m['total'])
    want(m['found'] == 0, 'none of which exist in an empty tree', m['found'])
    want(m['other_ext'] == 0, 'and no twins either, yet', m['other_ext'])

    # Now plant the .step twins the real KiCad 10 tree has, and nothing else.
    txt = open(TIGARD, encoding='utf-8', errors='replace').read()
    raws = kir._MODEL_RE.findall(txt)

    def sub(raw):
        # A LAMBDA replacement, never the bare string: `empty` is a Windows temp
        # path, re.sub treats a replacement as a TEMPLATE, and `C:\Users\...`
        # raises "bad escape \U". resolve_models itself already substitutes
        # through a lambda; this test did not, and only the test was wrong.
        return kir._VAR_RE.sub(lambda m: empty, raw).replace('\\', '/')

    # The counter counts REFERENCES, not distinct files, and on a real board
    # those differ a lot: tigard's 84 references point at far fewer files,
    # because every 0402 resistor names the same model. So derive what to
    # expect from the board instead of assuming one file is one reference --
    # a hand-picked "planted 5, expect 5" was wrong by a factor of seven here.
    resolved = [sub(r) for r in raws]
    chosen = sorted(set(resolved))[:3]
    expect_alt = sum(1 for r in resolved if r in chosen)
    for rel in chosen:
        stem = os.path.splitext(rel)[0]
        os.makedirs(os.path.dirname(stem), exist_ok=True)
        open(stem + '.step', 'w').close()
    m2 = kir.resolve_models(TIGARD, dirs)
    want(m2['found'] == 0,
         'a .step twin is not the .wrl the board asked for', m2['found'])
    want(m2['other_ext'] == expect_alt,
         'but it IS counted as present under another extension, once per '
         'REFERENCE -- the measured tigard case, and a different problem from '
         'a missing install', (m2['other_ext'], expect_alt))
    want(expect_alt > len(chosen),
         'and this board really does reuse models, so the two counts differ '
         '(%d references over %d files)' % (expect_alt, len(chosen)))

    # And the real thing: plant the .wrl the board actually names.
    expect_found = sum(1 for r in resolved if r == chosen[0])
    os.makedirs(os.path.dirname(chosen[0]), exist_ok=True)
    open(chosen[0], 'w').close()
    m3 = kir.resolve_models(TIGARD, dirs)
    want(m3['found'] == expect_found,
         'planting the named file resolves every reference to it',
         (m3['found'], expect_found))
    want('BARE BOARD' in kir.models_note(m2),
         'zero resolved models is called a bare board, in the caption',
         kir.models_note(m2))
    want('BARE BOARD' not in kir.models_note({'total': 15, 'found': 10}),
         'a partly-resolved board is NOT called bare -- lvds resolves 10 of 15 '
         'and renders fully populated')


def test_models_note_reports_found_of_total_in_that_order():
    """No test read the numbers, only the BARE BOARD substring, so printing
    them the wrong way round survived."""
    note = kir.models_note({'total': 15, 'found': 10})
    want('10/15' in note, 'found comes first, then total', note)
    want('15/10' not in note, 'and not the other way round', note)
    bare = kir.models_note({'total': 84, 'found': 0, 'other_ext': 78})
    want('0/84' in bare and 'BARE BOARD' in bare, 'a bare board reads 0/84',
         bare)
    # MOSTLY bare: gating the warning on exactly zero left four corpus boards
    # rendering essentially empty with no caption at all (1/160, 5/148, 3/58,
    # 7/75).
    mostly = kir.models_note({'total': 160, 'found': 1, 'other_ext': 149})
    want('MOSTLY BARE' in mostly,
         'and 1 of 160 is called out too, not silently reported as a count',
         mostly)
    want('another extension' in mostly,
         'with the stale-reference reason, which used to stop applying one '
         'model above zero', mostly)
    full = kir.models_note({'total': 15, 'found': 15})
    want('BARE' not in full, 'a fully-resolved board gets no warning', full)


def test_model_dirs_defines_the_projects_own_variable():
    """lvds has two ${KIPRJMOD} refs; the model test plants KIPRJMOD but grades
    tigard, which uses only ${KISYS3DMOD} -- so dropping KIPRJMOD survived."""
    dirs = kir.model_dirs(cli_path=None, board_path=BOARD_A)
    want(dirs.get('KIPRJMOD') == os.path.dirname(os.path.abspath(BOARD_A)),
         'KIPRJMOD is the board\'s own directory', dirs.get('KIPRJMOD'))
    # It is defined WITHOUT a kicad-cli path, because it comes from the board
    # and not from the install -- the versioned 3DMODEL_DIR variables do need
    # the install, and are legitimately unresolved here.
    want('KIPRJMOD' in kir.model_dirs(board_path=BOARD_A),
         'and needs no kicad-cli to be known')
    # Prove it is USED: substitute it and nothing else, then count.
    # DERIVED from the board, not guessed: a hardcoded count was wrong twice
    # here, and a number nobody re-derives is a number that rots.
    raws = kir._MODEL_RE.findall(
        open(BOARD_A, encoding='utf-8', errors='replace').read())
    n_proj = sum(1 for r in raws if '${KIPRJMOD}' in r)
    n_bare = sum(1 for r in raws if '${' not in r)
    want(n_proj > 0, 'lvds really does use ${KIPRJMOD}', n_proj)
    want(n_bare > 0,
         'and it also carries %d BARE relative paths -- the case that used to '
         'be resolved against the caller\'s directory' % n_bare, n_bare)

    only_proj = {'KIPRJMOD': dirs['KIPRJMOD']}
    m = kir.resolve_models(BOARD_A, only_proj)
    want(m['total'] == len(raws), 'every model reference is counted',
         (m['total'], len(raws)))
    want(m['total'] - m['unresolved_var'] == n_proj + n_bare,
         'with KIPRJMOD as the only key, exactly the ${KIPRJMOD} references '
         'plus the bare relative ones get as far as a path -- so dropping that '
         'key would silently make the first group unresolvable',
         (m['total'] - m['unresolved_var'], n_proj + n_bare))
    without = kir.resolve_models(BOARD_A, {})
    want(without['unresolved_var'] == m['total'] - n_bare,
         'and with no keys at all only the bare paths remain resolvable, '
         'because they need no variable', (without['unresolved_var'], n_bare))


def test_a_relative_model_path_is_resolved_against_the_board_not_the_cwd():
    """Measured: leaving a bare relative path relative made os.path.isfile
    answer against os.getcwd(), so whether a reference resolved was decided by
    what happened to sit beside the shell -- and anything found that way is a
    file kicad-cli would never load. This test plants a decoy at the CWD and
    asserts it is NOT counted, which is the defect stated as behaviour rather
    than as a corpus number."""
    d1, d2 = tempfile.mkdtemp(), tempfile.mkdtemp()
    board = os.path.join(d1, 'b.kicad_pcb')
    open(board, 'w', encoding='utf-8').write(
        '(footprint "x" (model "sub/part.step" (offset (xyz 0 0 0))))\n')
    here = os.getcwd()
    try:
        # Plant a decoy at the CWD, where a cwd-relative resolver would find it.
        os.makedirs(os.path.join(d2, 'sub'), exist_ok=True)
        open(os.path.join(d2, 'sub', 'part.step'), 'w').close()
        os.chdir(d2)
        m = kir.resolve_models(board)
        want(m['total'] == 1, 'one model reference', m)
        want(m['found'] == 0,
             'the decoy beside the CWD is NOT counted -- kicad-cli would never '
             'load it', m)
        # Now plant it where KiCad would actually look: beside the board.
        os.makedirs(os.path.join(d1, 'sub'), exist_ok=True)
        open(os.path.join(d1, 'sub', 'part.step'), 'w').close()
        m2 = kir.resolve_models(board)
        want(m2['found'] == 1,
             'while the one beside the BOARD is', m2)
        os.chdir(here)
        want(kir.resolve_models(board)['found'] == 1,
             'and the answer does not change with the caller\'s directory')
    finally:
        os.chdir(here)


def test_render_results_are_keyed_not_in_completion_order():
    """`render_many` promises results KEYED by job, so the film is identical
    at any worker count. A fake `render_iso` (no kicad-cli) that names each
    result after its own job: a result filed under another key is visible."""
    real = kir.render_iso
    seen = []

    def fake(board, png, cli, **kw):
        seen.append(png)
        return png, ''
    kir.render_iso = fake
    try:
        jobs = [(k, BOARD_A, 'shot_%d.png' % k, (-45.0, 0.0, 45.0))
                for k in (7, 3, 11, 5)]
        out = kir.render_many(jobs, 'FAKE', workers=4)
    finally:
        kir.render_iso = real
    want(len(seen) == 4, 'every job ran once', seen)
    want(all(out[k][0] == 'shot_%d.png' % k for k, *_ in jobs),
         'each result is filed under ITS OWN key', out)


def test_an_unreadable_render_is_a_failure_not_a_success():
    """Exit 0 plus a file on disk is not a decodable image: a zero-byte write
    -- a full disk, a killed child -- used to be reported as unqualified
    success. `subprocess.run` is stood in for, so no kicad-cli is needed."""
    import importlib.util
    if importlib.util.find_spec('PIL') is None:
        want(True, 'no Pillow: render_iso cannot verify, and says nothing')
        return
    d = tempfile.mkdtemp()
    png = os.path.join(d, 'empty.png')
    real = kir.subprocess.run

    def fake(argv, **kw):
        open(png, 'wb').close()          # exit 0, file exists, zero bytes

        class R:
            returncode = 0
            stdout = stderr = ''
        return R()

    kir.subprocess.run = fake
    try:
        got, err = kir.render_iso(BOARD_A, png, 'FAKE', 320, 240)
    finally:
        kir.subprocess.run = real
    want(got is None, 'an unreadable PNG is not a success', got)
    want('unreadable' in err and '0 bytes' in err,
         'and the reason names what was wrong, with the size', err)


TESTS_TO_RUN = [
    test_the_model_precheck_counts_what_is_on_disk,
    test_models_note_reports_found_of_total_in_that_order,
    test_model_dirs_defines_the_projects_own_variable,
    test_a_relative_model_path_is_resolved_against_the_board_not_the_cwd,
    test_render_results_are_keyed_not_in_completion_order,
    test_an_unreadable_render_is_a_failure_not_a_success,
]


def main():
    for fn in TESTS_TO_RUN:
        print('--- %s' % fn.__name__)
        try:
            fn()
        except Exception as exc:                            # noqa: BLE001
            import traceback
            BAD.append('%s RAISED %s' % (fn.__name__, exc))
            traceback.print_exc()
    if BAD:
        print('\nFAILED: %d' % len(BAD))
        for b in BAD:
            print('  - %s' % b)
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
