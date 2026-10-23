#!/usr/bin/env python3
"""#1354: render_placement advised a flag it does not have, and pointed its
crop commands at a folder the run never chose.

1. The renderer WARNS where the placement CLIs refuse (gate_or_exit with
   warn_only=True), but format_report only knew `allow_unplaced`, so on a
   pile it printed "Override with --allow-unplaced." -- a flag
   render_placement rejects with exit 2 -- and then rendered anyway, exit
   0. warn_only now gets no override advice (and no advice to run the
   renderer it is), for either condition; a routed board reads "proceeding
   with the copper left as it is".
2. The report's suggested crop renders hard-coded `-o wk/worst{i}.png`,
   `wk/clean.png` and `wk/focus/`, whatever -o was. They now sit beside the
   -o target: `x.png` -> `x_worst1.png`, a directory -> `<dir>/worst1.png`.

The refusing tools are unchanged: format_report without warn_only still
names the override.

    python3 tests/test_1354_render_warn_only_advice.py
"""
import os
import re
import shutil
import sys
import tempfile
from types import SimpleNamespace as NS

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

from placement.placement_state import PlacementState, format_report  # noqa: E402
from render_placement import crop_path_for                           # noqa: E402
from run_utils import check, evidence                                # noqa: E402

PILER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts',
                     'make_unplaced.py')
SRC = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def main():
    print("1: format_report, function level")
    pile = PlacementState(unplaced=True, reasons=['100% outside'],
                          n_footprints=18, distinct_positions=18)
    routed = PlacementState(has_copper=True, segments=10)
    txt = format_report(pile, 'render_placement.py', warn_only=True)
    ok("warn_only on a pile: no override advice",
       'Override with' not in txt, txt[-120:])
    ok("warn_only on a pile: no advice to run the renderer",
       'render_placement.py <board>' not in txt)
    ok("warn_only on a pile: still says it is not placed",
       'does not look PLACED' in txt)
    txt = format_report(routed, 'render_placement.py', warn_only=True)
    ok("warn_only on a routed board: no override advice, proceeding",
       'Override with' not in txt
       and 'proceeding with the copper left as it is' in txt, txt)
    txt = format_report(pile, 'place_optimize.py')
    ok("a refusing tool still names --allow-unplaced",
       'Override with --allow-unplaced' in txt)
    txt = format_report(routed, 'place_optimize.py')
    ok("a refusing tool still names --allow-routed",
       'Override with --allow-routed' in txt)

    print("2: crop_path_for follows -o")
    cp = crop_path_for(NS(output='/a/b/x.png', board='/q/board.kicad_pcb'))
    ok("file target -> stem siblings",
       cp('worst1.png') == '/a/b/x_worst1.png' and cp('focus/') == '/a/b/x_focus/',
       f"{cp('worst1.png')} {cp('focus/')}")
    cp = crop_path_for(NS(output='/a/b/out/', board='/q/board.kicad_pcb'))
    ok("directory target -> inside it",
       cp('clean.png') == os.path.join('/a/b/out/', 'clean.png'), cp('clean.png'))
    cp = crop_path_for(NS(output=None, board='/q/board.kicad_pcb'))
    ok("no -o -> beside the default target",
       cp('clean.png') == '/q/board_placement_clean.png', cp('clean.png'))

    print("3: render_placement on a real pile, end to end")
    tmp = tempfile.mkdtemp(prefix='t1354_')
    try:
        pile_pcb = os.path.join(tmp, 'pile.kicad_pcb')
        check([sys.executable, '-X', 'utf8', PILER, SRC, pile_pcb], accept=True)
        evidence(pile_pcb, 'the pile board')
        out = os.path.join(tmp, 'shots', 'x.png')
        os.makedirs(os.path.dirname(out))
        r = check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'render_placement.py'),
                   pile_pcb, '-o', out], accept=True)
        log = r.stdout + r.stderr
        ok("the board reads as a pile (else this tests nothing)",
           'does not look PLACED' in log)
        ok("no 'Override with --allow-unplaced'",
           'Override with --allow-unplaced' not in log)
        targets = re.findall(r' -o (\S+)', log)
        crops = [t for t in targets if t != out]
        ok("crop commands were suggested", bool(crops), str(targets))
        ok("no crop is under wk/",
           not any(t.startswith('wk/') for t in crops), str(crops))
        stem = os.path.splitext(out)[0] + '_'
        ok("every crop sits beside the -o target",
           all(t.startswith(stem) for t in crops), str(crops))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
