#!/usr/bin/env python3
"""#829 CLI/GUI parity: both parse paths must agree on who owns the outline.

`build_pcb_data_from_board` is an independent implementation of the text
parser, so a field only the text path fills is INERT under the GUI -- and this
particular field gates whether a footprint may be moved, so an inert one means
the guard silently is not there.

It could not be written the way `test_ref_label_pcbnew_parity.py` was. That gate
sweeps real corpus boards; **0 of the 27 tracked boards carry footprint-embedded
Edge.Cuts at all**, so there is nothing in the repo for it to compare. It uses
the synthetic `tests/fixture_829.py` board instead, which is also the only way
to cover the case that must NOT be locked.

Needs KiCad's pcbnew; self-skips if absent.

    python3 tests/gui_parity/test_829_edge_cuts_owner_parity.py
"""
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    r"C:\Program Files\KiCad\10.0\bin\python.exe",
    r"C:\Program Files\KiCad\9.0\bin\python.exe",
    r"C:\Program Files\KiCad\8.0\bin\python.exe",
]


def _reexec_into_kicad():
    """Re-run under a python that has pcbnew.

    subprocess, not `os.execv`: on Windows the interpreter lives under
    `C:\\Program Files\\...`, and execv re-splits argv[0] on the space, so the
    child tried to open `...\\KRT-829\\Files\\KiCad\\10.0\\bin\\python.exe`.
    """
    for cand in KICAD_PYTHONS:
        if cand != sys.executable and os.path.exists(cand):
            if subprocess.run([cand, '-c', 'import pcbnew'],
                              capture_output=True).returncode == 0:
                sys.exit(subprocess.run(
                    [cand, os.path.abspath(__file__)] + sys.argv[1:]).returncode)
    print("SKIP: no python with pcbnew found")
    sys.exit(0)


def main():
    try:
        import pcbnew
    except ImportError:
        _reexec_into_kicad()
        return 0

    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'tests'))
    import fixture_829 as FIX
    from kicad_parser import parse_kicad_pcb, build_pcb_data_from_board

    fails = []

    def check(label, cond, detail=''):
        print(f"  {'ok ' if cond else 'FAIL'}  {label}"
              + (f"   {detail}" if detail and not cond else ''))
        if not cond:
            fails.append(label)

    for rot in (None, 90, 37.5):
        path = FIX.write(struct_rot=rot)
        text = parse_kicad_pcb(path)
        board = pcbnew.LoadBoard(path)
        live = build_pcb_data_from_board(board)
        tag = f"rot={rot}"
        for ref, (want_owns, want_struct) in FIX.EXPECTED.items():
            t = text.footprints.get(ref)
            g = live.footprints.get(ref)
            if t is None or g is None:
                check(f"{tag} {ref}: present on both paths", False,
                      f"text={t is not None} pcbnew={g is not None}")
                continue
            check(f"{tag} {ref}: owns_edge_cuts agrees",
                  t.owns_edge_cuts == g.owns_edge_cuts,
                  f"text={t.owns_edge_cuts} pcbnew={g.owns_edge_cuts}")
            check(f"{tag} {ref}: owns_board_outline agrees",
                  t.owns_board_outline == g.owns_board_outline,
                  f"text={t.owns_board_outline} pcbnew={g.owns_board_outline}")
            # Agreeing on the WRONG answer is not parity. Pin the value too.
            check(f"{tag} {ref}: owns_board_outline == {want_struct}",
                  t.owns_board_outline == want_struct,
                  f"got {t.owns_board_outline}")

    print()
    if fails:
        print(f"{len(fails)} FAILED")
        return 1
    print("ALL PASS -- both parse paths agree on Edge.Cuts ownership")
    return 0


if __name__ == '__main__':
    sys.exit(main())
