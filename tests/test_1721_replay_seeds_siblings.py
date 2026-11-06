#!/usr/bin/env python3
"""#1721: a replay seeds a relative input board with EVERY sibling.

`redo_stress_test --remap/--workdir` copies an input-only relative board from
the original run dir into the fresh one. It used to carry the board's
`.kicad_pro` and nothing else, so the `.kicad_dru` custom rules (#498) and the
design brief (#1253), which every routing step reads only from its input's
sibling, stayed behind: the whole replayed chain ran without the project's
rules, shipped other copper, and still reported 0 non-zero exits.

Pinned here, on the three staging paths that share the one list
`copy_board.SIBLING_EXTS`:
  - `seed_input_boards` carries every sibling, and names each one it carried;
  - its stem-mismatch fallback (a run dir whose project kept the board's
    original name) carries every sibling under the seeded stem;
  - `mirror_project_sibling` (a replayed `cp`/`mv`) carries every sibling;
  - `partial_replay_from_planes`' staging of a cut-over board does the same.
"""
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _sub in ('tests', os.path.join('tests', 'stress'), 'py_router'):
    sys.path.insert(0, os.path.join(REPO, _sub))

from copy_board import SIBLING_EXTS                         # noqa: E402
from redo_stress_test import (seed_input_boards,            # noqa: E402
                              mirror_project_sibling)

#: One stand-in per sibling, the way a run dir holds them.
BODY = {'.kicad_pro': '{"floor": 0.09}',
        '.kicad_prl': '{}',
        '.kicad_dru': '(version 1)\n(rule bcu (layer "B.Cu")\n'
                      '  (constraint clearance (min 1.0mm)))\n',
        '.design-brief.json': '{"schema": 1}'}


def _write(path, body):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(body)


def _read(path):
    with open(path, encoding='utf-8') as fh:
        return fh.read()


def _stage_run(run, stem):
    _write(os.path.join(run, stem + '.kicad_pcb'), '(kicad_pcb)')
    for ext in SIBLING_EXTS:
        _write(os.path.join(run, stem + ext), BODY[ext])
    manifest = os.path.join(run, 'redo_commands.sh')
    _write(manifest, '# manifest\n')
    return manifest


def test_the_list_names_the_rules():
    # The test is about these two; if the shared list ever loses one, say so
    # here rather than pass on a shorter list.
    assert '.kicad_dru' in SIBLING_EXTS and '.design-brief.json' in SIBLING_EXTS
    assert set(BODY) == set(SIBLING_EXTS), SIBLING_EXTS
    print("  PASS: copy_board.SIBLING_EXTS carries the rules and the brief")


def test_seed_carries_every_sibling():
    with tempfile.TemporaryDirectory() as run, \
            tempfile.TemporaryDirectory() as fresh:
        manifest = _stage_run(run, 'board')
        cmds = [(run, ['py_router/route.py', 'board.kicad_pcb',
                       'routed.kicad_pcb'])]
        seeded = seed_input_boards(manifest, cmds, fresh)
        want = ['board.kicad_pcb'] + ['board' + e for e in SIBLING_EXTS]
        assert sorted(seeded) == sorted(want), seeded
        for ext in SIBLING_EXTS:
            dst = os.path.join(fresh, 'board' + ext)
            assert os.path.isfile(dst) and _read(dst) == BODY[ext], ext
        # The produced board is never seeded, and nor are its siblings.
        assert not os.path.exists(os.path.join(fresh, 'routed.kicad_pcb'))
    print("  PASS: seed_input_boards carries the .kicad_dru and the brief, "
          "and names them")


def test_seed_stem_fallback_carries_every_sibling():
    with tempfile.TemporaryDirectory() as parent, \
            tempfile.TemporaryDirectory() as fresh:
        run = os.path.join(parent, 'someboard')
        os.makedirs(run)
        manifest = _stage_run(run, 'someboard')
        # The worker renamed the board; the project kept the run dir's name.
        os.rename(os.path.join(run, 'someboard.kicad_pcb'),
                  os.path.join(run, 'board0.kicad_pcb'))
        cmds = [(run, ['py_router/route.py', 'board0.kicad_pcb',
                       'routed.kicad_pcb'])]
        seeded = seed_input_boards(manifest, cmds, fresh)
        for ext in SIBLING_EXTS:
            dst = os.path.join(fresh, 'board0' + ext)
            assert os.path.isfile(dst) and _read(dst) == BODY[ext], (ext, seeded)
    print("  PASS: the stem-mismatch fallback carries every sibling under the "
          "seeded stem")


def test_mirror_carries_every_sibling():
    with tempfile.TemporaryDirectory() as d:
        _stage_run(d, 'step')
        mirror_project_sibling(['cp', 'step.kicad_pcb', 'final.kicad_pcb'], d)
        for ext in SIBLING_EXTS:
            dst = os.path.join(d, 'final' + ext)
            assert os.path.isfile(dst) and _read(dst) == BODY[ext], ext
        mirror_project_sibling(['mv', 'final.kicad_pcb', 'moved.kicad_pcb'], d)
        for ext in SIBLING_EXTS:
            assert os.path.isfile(os.path.join(d, 'moved' + ext)), ext
            assert not os.path.exists(os.path.join(d, 'final' + ext)), ext
    print("  PASS: a replayed cp/mv carries every sibling")


def test_partial_replay_staging_uses_the_list():
    # partial_replay_from_planes stages in main(); its sibling copy is the
    # shared copy_siblings, read here from the source rather than run (main
    # replays a whole set).
    src = _read(os.path.join(REPO, 'tests', 'stress',
                             'partial_replay_from_planes.py'))
    assert 'copy_siblings(src, os.path.join(dst_dir, bb))' in src
    assert "'.kicad_pro'" not in src, "a hand-named sibling is back"
    print("  PASS: partial_replay_from_planes stages through copy_siblings")


TESTS = [test_the_list_names_the_rules, test_seed_carries_every_sibling,
         test_seed_stem_fallback_carries_every_sibling,
         test_mirror_carries_every_sibling,
         test_partial_replay_staging_uses_the_list]


def main():
    failed = 0
    for t in TESTS:
        try:
            t()
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL: {t.__name__}: {exc}")
    print(f"\n{len(TESTS) - failed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
