#!/usr/bin/env python3
"""Issue #288 fanout fixes.

1. edge_pair_escape_dir: a diff pair with BOTH pads on the outer ring must
   escape PERPENDICULAR to the pair axis. A corner pad's is_edge_pad direction
   reports the column axis ('right') even when the pair lies along the bottom
   row; escaping along the row collapsed both converge legs onto the row line
   (kuchen U1 T15/T16: DVI_D2_N drawn straight through DVI_D2_P's pad).
2. escape-method 'auto' (new default): when the channel engine drops balls,
   retry with the under-pad grid escape and keep whichever escapes more
   (glasgow U30: channel 90/97 -> auto 97/97).
3. --layer-costs: negative cost excludes a layer from the fanout entirely
   (soon-to-be-plane inner layers); weights fill cheaper layers first.

    python3 tests/test_bga_fanout_escape288.py
"""
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)

from bga_fanout.escape import edge_pair_escape_dir
from bga_fanout.types import BGAGrid

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')

BASE_CMD = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'bga_fanout.py'),
            BOARD, '--component', 'U30',
            '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
            '--nets', '*', '!GND', '!+3V3', '!+1V2',
            '--diff-pairs', '*_P', '*_N',
            '--track-width', '0.13', '--clearance', '0.1',
            '--via-size', '0.45', '--via-drill', '0.2', '--diff-pair-gap', '0.15']


def _grid():
    # 4x4, pitch 1.0, spanning (0,0)-(3,3); bottom row y=3, right col x=3
    return BGAGrid(rows=[0.0, 1.0, 2.0, 3.0], cols=[0.0, 1.0, 2.0, 3.0],
                   pitch_x=1.0, pitch_y=1.0,
                   min_x=0.0, min_y=0.0, max_x=3.0, max_y=3.0,
                   center_x=1.5, center_y=1.5)


def _run_fanout(extra, out):
    res = subprocess.run(BASE_CMD + ['--output', out] + extra,
                         cwd=ROOT, capture_output=True, text=True)
    line = next((l for l in res.stdout.splitlines() if 'JSON_SUMMARY' in l), None)
    return res, (json.loads(line.split('JSON_SUMMARY:', 1)[1]) if line else None)


def run():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            fails.append(name)

    g = _grid()
    # kuchen shape: same BOTTOM-ROW pair, P is the bottom-right CORNER
    # (is_edge_pad says 'right' for it) -> must escape down, not along the row.
    check("corner P + same-row N escapes perpendicular (down)",
          edge_pair_escape_dir(3.0, 3.0, 2.0, 3.0, 'right', 'down', g) == 'down')
    # mirrored: same RIGHT-COLUMN pair, P is the corner reporting 'down'
    check("corner P + same-column N escapes perpendicular (right)",
          edge_pair_escape_dir(3.0, 3.0, 3.0, 2.0, 'down', 'right', g) == 'right')
    # non-corner pair keeps its own (already perpendicular) direction
    check("plain bottom-row pair keeps 'down'",
          edge_pair_escape_dir(1.0, 3.0, 2.0, 3.0, 'down', 'down', g) == 'down')
    # both corners (2x2 grid degenerate): falls back to the nearer edge
    check("both-corner fallback picks a perpendicular direction",
          edge_pair_escape_dir(0.0, 3.0, 1.0, 3.0, 'left', 'right', g) in ('up', 'down'))

    with tempfile.TemporaryDirectory() as td:
        # 'auto' must escape at least as many balls as strict channel, and on
        # this board strictly more (channel deterministically drops balls).
        res_ch, js_ch = _run_fanout(['--escape-method', 'channel'],
                                    os.path.join(td, 'ch.kicad_pcb'))
        res_auto, js_auto = _run_fanout([], os.path.join(td, 'auto.kicad_pcb'))
        check("channel run produced a summary", js_ch is not None)
        check("auto run produced a summary", js_auto is not None)
        if js_ch and js_auto:
            check("channel still drops balls on glasgow (repro intact)",
                  js_ch['failed'] > 0)
            check("auto escapes every ball channel dropped",
                  js_auto['failed'] == 0 and js_auto['escaped'] > js_ch['escaped'])
            check("auto retry message printed",
                  'retrying with the under-pad grid escape' in res_auto.stdout)

        # --layer-costs: forbidding In2.Cu must leave it free of NEW copper.
        out_lc = os.path.join(td, 'lc.kicad_pcb')
        res_lc, js_lc = _run_fanout(['--escape-method', 'channel',
                                     '--layer-costs', '1', '1', '-1', '1'], out_lc)
        check("layer-costs run produced a summary", js_lc is not None)
        if js_lc:
            from kicad_parser import parse_kicad_pcb
            inp = parse_kicad_pcb(BOARD)
            outp = parse_kicad_pcb(out_lc)
            in2_before = sum(1 for s in inp.segments if s.layer == 'In2.Cu')
            in2_after = sum(1 for s in outp.segments if s.layer == 'In2.Cu')
            check("no new copper on the forbidden layer",
                  in2_after == in2_before)
            check("exclusion message printed",
                  'excluding forbidden layer' in res_lc.stdout)

        # validation: wrong count and forbidden top layer must error out
        res_bad, _ = _run_fanout(['--layer-costs', '1', '1'],
                                 os.path.join(td, 'bad.kicad_pcb'))
        check("wrong --layer-costs count fails", res_bad.returncode != 0)
        res_top, _ = _run_fanout(['--layer-costs', '-1', '1', '1', '1'],
                                 os.path.join(td, 'top.kicad_pcb'))
        check("forbidden top layer fails", res_top.returncode != 0)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
