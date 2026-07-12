#!/usr/bin/env python3
"""Regression test for issue #344 (icepi_zero C6.1/GPIO26 stacked-via graze).

place_fanout_clearance's via-nudge (#313) rewrites the output text as
"remove the via at its old position (same net) + append it at the new
position". The removal's net guard matched only numeric `(net N)` fields;
KiCad 10 files carry the net NAME (`(net "/GPIO26")`), so every name-based
via parsed as net None, never matched, and removal became a no-op — the
output shipped BOTH the original via (still grazing the cap pad) and the
moved copy stacked next to it. Same file-format class as #264.

The guard must skip only on a POSITIVE net mismatch in whichever form the
file carries, and fall back to positional matching when the net cannot be
compared.

    python3 tests/test_via_move_removal_net_name.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from plane_io import _remove_vias_at_positions

VIA_NAME = '''(kicad_pcb
	(via
		(at 153.600000 101.150000)
		(size 0.25)
		(drill 0.15)
		(layers "F.Cu" "B.Cu")
		(net "/GPIO26")
	)
	(via
		(at 152.800000 100.350000)
		(size 0.25)
		(drill 0.15)
		(layers "F.Cu" "B.Cu")
		(net "/GPIO13")
	)
)
'''

VIA_ID = VIA_NAME.replace('(net "/GPIO26")', '(net 38)').replace(
    '(net "/GPIO13")', '(net 35)')


def main():
    results = []

    # 1. #344 repro: name-based file, ids + names supplied -> removed.
    c, n = _remove_vias_at_positions(
        VIA_NAME, [(153.6, 101.15)], net_ids=[38], net_names=['/GPIO26'])
    results.append(("name-based via removed when name matches",
                    n == 1 and '/GPIO26' not in c and '/GPIO13' in c))

    # 2. Name-based file, WRONG expected name -> left alone (guard holds).
    c, n = _remove_vias_at_positions(
        VIA_NAME, [(153.6, 101.15)], net_ids=[35], net_names=['/GPIO13'])
    results.append(("name-based via kept on positive name mismatch", n == 0))

    # 3. Name-based file, ids only (caller has no name map) -> positional
    #    fallback removes it rather than silently shipping a duplicate.
    c, n = _remove_vias_at_positions(VIA_NAME, [(153.6, 101.15)], net_ids=[38])
    results.append(("name-based via removed via positional fallback", n == 1))

    # 4. Numeric-net file: id guard still enforced both ways.
    c, n = _remove_vias_at_positions(VIA_ID, [(153.6, 101.15)], net_ids=[38])
    results.append(("numeric via removed when id matches", n == 1))
    c, n = _remove_vias_at_positions(VIA_ID, [(153.6, 101.15)], net_ids=[99])
    results.append(("numeric via kept on positive id mismatch", n == 0))

    # 5. No net expectations at all: pure positional (pre-#313 callers).
    c, n = _remove_vias_at_positions(VIA_NAME, [(152.8, 100.35)])
    results.append(("positional-only removal still works", n == 1))

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} via-move removal tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
