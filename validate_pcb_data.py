#!/usr/bin/env python3
"""Validate parser parity: pcbnew-built PCBData vs text-parsed PCBData.

Headless version of the GUI About-tab "Validate PCB Data" button: load the
board with pcbnew, build PCBData from the live board objects
(build_pcb_data_from_board -- the GUI plugin's path), parse the same file with
the text parser (parse_kicad_pcb -- the CLI path), and diff the two models
with compare_pcb_data. Any difference means one of the two parsers is
mis-modelling the board (pad geometry, zones, arc tracks, bounds, ...) and the
CLI and GUI would route against different worlds.

Needs the pcbnew Python module. When run with a plain python3 it re-execs
itself into KiCad's bundled interpreter automatically (macOS/Linux/Windows
default install paths); pass boards as arguments:

    python3 validate_pcb_data.py board.kicad_pcb [more.kicad_pcb ...]

Exit status: 0 = all boards match, 1 = differences found, 2 = error.
"""
import os
import sys

# Non-fatal pcbnew asserts (PCB_VIA::GetWidth layer arg, wxApp traits) spam
# stderr on some KiCad builds; they do not affect the extracted data.

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",  # Linux distro KiCad installs pcbnew for system python
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
]


def _reexec_with_kicad_python():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable:
            continue
        if os.path.isfile(cand):
            os.execv(cand, [cand, os.path.abspath(__file__)] + sys.argv[1:])
    print("ERROR: pcbnew module not available and no KiCad python found. "
          "Run with KiCad's bundled python3.")
    sys.exit(2)


def main() -> int:
    boards = [a for a in sys.argv[1:] if not a.startswith('-')]
    if not boards:
        print(__doc__)
        return 2

    try:
        import pcbnew
    except ImportError:
        _reexec_with_kicad_python()
        return 2  # unreachable

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from kicad_parser import parse_kicad_pcb, compare_pcb_data, build_pcb_data_from_board

    worst = 0
    for path in boards:
        print(f"=== {path} ===")
        try:
            board = pcbnew.LoadBoard(path)
            board_data = build_pcb_data_from_board(board)
            file_data = parse_kicad_pcb(path)
        except Exception as e:
            print(f"ERROR: {e}")
            worst = max(worst, 2)
            continue
        diffs = compare_pcb_data(board_data, file_data)
        if not diffs:
            print("PASS: pcbnew data matches file parse")
        else:
            print(f"FAIL: {len(diffs)} difference(s):")
            for d in diffs:
                print(f"  - {d}")
            worst = max(worst, 1)
    return worst


if __name__ == '__main__':
    sys.exit(main())
