#!/usr/bin/env python3
"""The placement CLIs must carry the .kicad_pro / .kicad_dru siblings (#441).

Why this is a test and not a convention. The sibling `.kicad_pro` holds the DRC
floor the chain routed to. A board written without it makes the NEXT step
resolve its floor from the STOCK netclass and stamp that looser value over
tighter copper, so KiCad then grades correct sub-floor copper as phantom
clearance DRC. CLAUDE.md states the rule; `placement/portfolio.copy_siblings`
implements it; `place_seed` and `place_portfolio` called it and
**`place_optimize` and `place_route_loop` did not**. SKILL.md worked around the
gap by telling the caller to `cp` the files by hand, which is exactly the kind
of instruction that gets dropped from one step of a chain.

The `place_route_loop` case is the one that bites hardest, and it is not the
final output: the loop copies its INPUT to `loop_round0.kicad_pcb` and then
ROUTES that copy. Without the sibling, round 0 -- and therefore every
accept/reject decision that compares against round 0's failure count -- is
measured at the wrong clearance.

Run: python3 -X utf8 tests/test_411_placement_siblings.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ONE list, imported (#711). This tuple used to be a TENTH hand-written copy
# of the sibling set -- narrower than the canonical one -- so this test would
# have stayed green while every placement CLI stranded the new
# `.design-brief.json`. A guard that restates the thing it guards cannot
# detect a change to it; see tests/test_711_sibling_lists.py.
from copy_board import SIBLING_EXTS as SIBLINGS  # noqa: E402


def _stage(tmp, board_name):
    """Copy a fixture board into `tmp` WITH a .kicad_pro and .kicad_dru.

    The `.kicad_dru` is synthesised when the fixture has none: this test is
    about whether the tool carries siblings, not about which siblings the
    fixture happens to ship with, and a test that silently skips the dru
    because no in-repo board has one would assert nothing about it.
    """
    from tests import fixture_boards
    src = fixture_boards.ensure(board_name)
    dst = os.path.join(tmp, board_name)
    shutil.copyfile(src, dst)
    stem_src, stem_dst = os.path.splitext(src)[0], os.path.splitext(dst)[0]
    for ext in SIBLINGS:
        if os.path.exists(stem_src + ext):
            shutil.copyfile(stem_src + ext, stem_dst + ext)
        elif ext == '.kicad_dru':
            with open(stem_dst + ext, 'w', encoding='utf-8') as fh:
                fh.write('(version 1)\n')
        elif ext == '.kicad_pro':
            with open(stem_dst + ext, 'w', encoding='utf-8') as fh:
                json.dump({'board': {'design_settings': {}}}, fh)
        elif ext == '.design-brief.json':
            # #711. Minimal but VALID: a brief that fails to load would make
            # the carry assertion pass on a file the next step refuses.
            with open(stem_dst + ext, 'w', encoding='utf-8') as fh:
                json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                           'product': {'form_factor': 'test-fixture'},
                           'interfaces': []}, fh)
        elif ext == '.kicad_prl':
            with open(stem_dst + ext, 'w', encoding='utf-8') as fh:
                json.dump({'board': {}}, fh)
    return dst


def _assert_siblings(out_board, label):
    stem = os.path.splitext(out_board)[0]
    missing = [e for e in SIBLINGS if not os.path.exists(stem + e)]
    assert not missing, (
        f"{label}: wrote {os.path.basename(out_board)} without {missing}. "
        f"The next step will resolve its DRC floor from the stock netclass "
        f"(#441) and grade correct copper as phantom clearance violations, "
        f"and -- for a stranded .design-brief.json (#711) -- fall back to "
        f"INFERRING every edge it declared from the current pose.")


def test_place_optimize_carries_siblings():
    with tempfile.TemporaryDirectory() as tmp:
        board = _stage(tmp, 'interf_u_unrouted.kicad_pcb')
        out = os.path.join(tmp, 'optimized.kicad_pcb')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_placer', 'place_optimize.py'), board, out,
             '--max-displacement', '0.5', '--max-passes', '1'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=900)
        assert r.returncode == 0, f"place_optimize exited {r.returncode}\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}"
        assert os.path.exists(out), "place_optimize wrote no board"
        _assert_siblings(out, 'place_optimize')


def test_place_optimize_propagates_its_exit_code():
    """--suggest-locks returns 0 from main(); __main__ used to drop it."""
    with tempfile.TemporaryDirectory() as tmp:
        board = _stage(tmp, 'interf_u_unrouted.kicad_pcb')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_placer', 'place_optimize.py'), board, '--suggest-locks'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=600)
        assert r.returncode == 0, f"--suggest-locks exited {r.returncode}"
        # And a board-state refusal must still surface as UNPLACED_EXIT rather
        # than being swallowed into a success.
        from placement.placement_state import UNPLACED_EXIT
        assert UNPLACED_EXIT == 3


def test_route_loop_round0_copy_carries_siblings():
    """The load-bearing one: round 0 ROUTES its copy of the input.

    Asserted WITHOUT running the loop (which would route a whole board): the
    copy is made before any routing, so the check is on the code path, which is
    read here rather than executed. A behavioural version of this test would
    cost minutes per run and assert the same line.
    """
    src = os.path.join(ROOT, 'py_placer', 'place_route_loop.py')
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    m = re.search(r"cur_file\s*=\s*os\.path\.join\(work,\s*'loop_round0\.kicad_pcb'\)"
                  r"(.{0,900}?)\n\s*screened\s*=\s*0", text, re.S)
    assert m, "could not find the round-0 copy block in place_route_loop.py"
    block = m.group(1)
    assert 'shutil.copy(' in block, "round 0 no longer copies the input board"
    assert 'copy_siblings(' in block, (
        "place_route_loop round 0 copies the input board WITHOUT its siblings. "
        "Round 0 routes that copy, so every round -- and every accept/reject "
        "decision measured against round 0 -- runs at the stock netclass floor "
        "instead of the board's own (#441).")
    # And the final output copy.
    tail = text[text.index('shutil.copy(cur_file, args.output_file)'):]
    assert 'copy_siblings(cur_file, args.output_file)' in tail[:400], (
        "the loop's final output copy drops the siblings")


def test_route_loop_emits_a_json_summary():
    """The verdict must be machine-readable, not a text `Final:` line."""
    src = os.path.join(ROOT, 'py_placer', 'place_route_loop.py')
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    assert 'print("JSON_SUMMARY: "' in text, (
        "place_route_loop prints no JSON_SUMMARY on its normal path; the only "
        "structured output of a run would again be the loop_round{N}.json "
        "sidecars")
    for key in ('failures_before', 'failures_after', 'iterations_before',
                'iterations_after', 'vias_before', 'vias_after',
                'rounds_accepted', 'rounds_run', 'max_displacement',
                'max_target_pins'):
        assert f"'{key}'" in text, f"JSON_SUMMARY is missing {key!r}"
    assert 'sys.exit(main() or 0)' in text, (
        "main()'s return value is dropped by __main__, so a refusal exits 0")


TESTS = [
    test_place_optimize_carries_siblings,
    test_place_optimize_propagates_its_exit_code,
    test_route_loop_round0_copy_carries_siblings,
    test_route_loop_emits_a_json_summary,
]


if __name__ == '__main__':
    failed = 0
    for t in TESTS:
        try:
            t()
            print(f"PASS {t.__name__}")
        except Exception as exc:                       # noqa: BLE001
            failed += 1
            print(f"FAIL {t.__name__}: {exc}")
    sys.exit(1 if failed else 0)
