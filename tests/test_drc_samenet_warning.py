#!/usr/bin/env python3
"""check_drc must treat same-net segment crossings as a WARNING, not a DRC
violation (same-net copper overlap is permitted by KiCad). A different-net
crossing is still a violation (#84).

    python3 tests/test_drc_samenet_warning.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check_drc import run_drc

BOARD = '''(kicad_pcb
\t(version 20221018)
\t(net 0 "")
\t(net 1 "SIG")
\t(net 2 "OTHER")
\t(segment (start 0 0) (end 2 2) (width 0.2) (layer "F.Cu") (net {a}) (uuid "a"))
\t(segment (start 0 2) (end 2 0) (width 0.2) (layer "F.Cu") (net {b}) (uuid "b"))
)'''


def _run(net_a, net_b):
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD.format(a=net_a, b=net_b))
        path = f.name
    try:
        return run_drc(path, clearance=0.1, quiet=True)
    finally:
        os.unlink(path)


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # Two crossing segments on the SAME net -> not a violation (warning only).
    same = _run(1, 1)
    check("same-net crossing is not a violation",
          not any(v['type'] == 'segment-crossing-same-net' for v in same))
    check("same-net crossing board has 0 violations", len(same) == 0)

    # Two crossing segments on DIFFERENT nets -> a real violation.
    diff = _run(1, 2)
    check("different-net crossing is a violation",
          any(v['type'] == 'segment-crossing' for v in diff) and len(diff) >= 1)

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  same-net crossing -> warning (0 violations); different-net")
    print("        crossing -> violation")
    print("\n3/3 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
