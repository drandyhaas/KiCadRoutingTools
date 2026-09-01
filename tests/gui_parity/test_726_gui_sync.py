#!/usr/bin/env python3
"""#726 GUI gate: the live-board sync must reach BOTH blocks, not one twice.

`gui_utils.sync_footprint_positions_from_board` refreshes a cached `PCBData`
from the live pcbnew board between plan steps (#362: a cap moved by
`optimize_caps` had to stop being routed around where it USED to be). It looked
each live footprint up by `GetReference()`. With two blocks named `TP4` that is
the footprint-level twin of the pad bug the function's own docstring already
describes: both live footprints resolve to the ONE `TP4` entry, the second
overwrites the first's pose, and the cached model has two parts on top of each
other with no error anywhere.

WHY THIS FILE EXISTS AT ALL -- it was written because a mutation battery said
so. `tests/mutate_726.py`'s `gui-sync-matches-by-bare-reference` row reverts
that lookup to `GetReference()`, and on its first run the row SURVIVED: nothing
in the suite covered the function on a board with duplicates.
`tests/gui_parity/test_footprint_position_sync.py` does cover it, and stays
GREEN through the mutation, because it runs on `rp2350_fpga_eensy_prePlane`
(61 blocks, 61 references). A passing gate on a board that cannot express the
defect proves nothing about it, and that is the whole reason this one names a
board that can.

THREE THINGS THIS PINS:

1. **A no-op sync is a true no-op**, on a duplicate-carrying board. Every pose
   and every pad position must come back bit-identical. Under the bare-reference
   lookup, `TP4` acquires `TP4~2`'s pose without anything moving on the board.

2. **A real move reaches the block it was made on.** Move ONE twin on the live
   board, sync, and the cached model must show that twin moved and the other
   one still where it was.

3. **The pads follow the footprint they belong to.** The function updates pad
   positions by ITERATION ORDER within a footprint; if the footprint itself is
   the wrong one, the pads land on the wrong part's coordinates, which is what
   the router then treats as copper.

Needs pcbnew; re-execs into KiCad's python. Lives in `tests/gui_parity/`
because `run_all.py`'s glob only collects `tests/test_*.py`.

    python3 -X utf8 tests/gui_parity/test_726_gui_sync.py
"""
import glob
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), reverse=True),
]

#: A board with two blocks sharing a reference AND pads on both, so a swapped
#: pose is measurable. watchy's TP4/TP5 are real test points 15-18 mm apart.
BOARD = os.path.join(REPO, 'kicad_files', 'watchy.kicad_pcb')
DUP = 'TP4'

TOL = 1e-6

FAILURES = []


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + detail) if detail else ''))
        FAILURES.append(what)


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            argv = [cand, '-X', 'utf8', os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                # os.execv re-splits argv on spaces through the CRT on Windows.
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew found")
    sys.exit(0)


def _snapshot(pcb):
    return {k: (round(f.x, 6), round(f.y, 6), round(f.rotation or 0.0, 6),
                tuple(sorted((round(p.global_x, 6), round(p.global_y, 6))
                             for p in f.pads)))
            for k, f in pcb.footprints.items()}


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    import pcbnew
    for _p in ('', 'py_router', 'py_placer', 'py_tools',
               'kicad_routing_plugin'):
        _d = os.path.join(REPO, _p)
        if _d not in sys.path:
            sys.path.insert(0, _d)
    from kicad_parser import parse_kicad_pcb, mm_to_iu
    from gui_utils import (sync_footprint_positions_from_board,
                           live_footprints_by_key)

    if not os.path.exists(BOARD):
        print('SKIP: %s not found' % BOARD)
        return 0
    print('KiCad build: %s' % pcbnew.GetBuildVersion())

    pcb = parse_kicad_pcb(BOARD)
    twins = sorted(k for k in pcb.footprints if k.startswith(DUP))
    check(len(twins) == 2,
          'the fixture board still carries two %s blocks -- if it stops, this '
          'gate is vacuous, so REPLACE the board rather than deleting the arm'
          % DUP, str(twins))
    if len(twins) != 2:
        return 1
    a, b = twins

    board = pcbnew.LoadBoard(BOARD)
    live = live_footprints_by_key(board)
    check(set(live) >= set(twins),
          'live_footprints_by_key resolves both twins',
          str(sorted(set(twins) - set(live))))
    check(live[a].m_Uuid.AsString() == pcb.footprints[a].uuid
          and live[b].m_Uuid.AsString() == pcb.footprints[b].uuid,
          'and each key names the SAME physical footprint as the parsed model')

    # --- 1. a no-op sync is a true no-op ---
    before = _snapshot(pcb)
    n = sync_footprint_positions_from_board(board, pcb)
    after = _snapshot(pcb)
    check(n > 0, 'the sync ran (it is best-effort and never raises)', str(n))
    drift = {k: (before[k], after[k]) for k in before if before[k] != after[k]}
    check(not drift,
          'a no-op sync moves NOTHING, on a board with duplicate references',
          str(list(drift.items())[:2]))

    # --- 2 & 3. a real move reaches the block it was made on ---
    pcb2 = parse_kicad_pcb(BOARD)
    base = _snapshot(pcb2)
    board2 = pcbnew.LoadBoard(BOARD)
    live2 = live_footprints_by_key(board2)
    fp = live2[b]
    old = fp.GetPosition()
    fp.SetPosition(pcbnew.VECTOR2I(old.x + mm_to_iu(4.0),
                                   old.y + mm_to_iu(3.0)))
    sync_footprint_positions_from_board(board2, pcb2)
    now = _snapshot(pcb2)

    check(abs(now[b][0] - (base[b][0] + 4.0)) < 1e-3
          and abs(now[b][1] - (base[b][1] + 3.0)) < 1e-3,
          '%s picks up the move that was made ON %s' % (b, b),
          '%s -> %s' % (base[b][:2], now[b][:2]))
    check(now[a][:3] == base[a][:3],
          '%s is untouched -- under a bare GetReference() lookup it acquires '
          "%s's pose instead" % (a, b),
          '%s -> %s' % (base[a][:3], now[a][:3]))
    check(now[a][3] == base[a][3],
          "%s's PADS are untouched too (the router's copper obstacles)" % a)
    check(now[b][3] != base[b][3],
          "%s's pads followed its footprint" % b)
    others = [k for k in base if k not in (a, b) and base[k] != now[k]]
    check(not others, 'and no unrelated part moved', str(others[:4]))

    print('\n%d failure(s)' % len(FAILURES))
    for f in FAILURES:
        print('  FAILED: %s' % f)
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
