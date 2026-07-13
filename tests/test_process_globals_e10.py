#!/usr/bin/env python3
"""#382 E10: low-risk process-global fixes.

1. single_ended_routing._unblock_debug() reads KICAD_UNBLOCK_DEBUG per call
   (was a module-level constant frozen at import).
2. clearance_ledger.fresh_run() context manager resets on entry and preserves
   the recorded minimum after the block.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import clearance_ledger
import single_ended_routing as ser


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return bool(cond)


def main():
    r = []

    # --- _unblock_debug per-call read ---
    prev = os.environ.get('KICAD_UNBLOCK_DEBUG')
    try:
        os.environ.pop('KICAD_UNBLOCK_DEBUG', None)
        r.append(_ok("_unblock_debug False when env unset", ser._unblock_debug() is False))
        os.environ['KICAD_UNBLOCK_DEBUG'] = '1'
        r.append(_ok("_unblock_debug True after env set (no reimport)",
                     ser._unblock_debug() is True))
        os.environ.pop('KICAD_UNBLOCK_DEBUG', None)
        r.append(_ok("_unblock_debug False again after unset", ser._unblock_debug() is False))
    finally:
        if prev is None:
            os.environ.pop('KICAD_UNBLOCK_DEBUG', None)
        else:
            os.environ['KICAD_UNBLOCK_DEBUG'] = prev

    # --- fresh_run() context manager ---
    clearance_ledger.reset()
    clearance_ledger.record(0.13)  # a stale earlier-board value
    with clearance_ledger.fresh_run():
        # entry reset cleared the stale value
        r.append(_ok("fresh_run resets on entry", clearance_ledger.get_min() is None))
        clearance_ledger.record(0.15)
    # value recorded inside survives the block (main() reads it afterwards)
    r.append(_ok("fresh_run preserves recorded min after block",
                 clearance_ledger.get_min() == 0.15))
    # a subsequent fresh_run clears it again
    with clearance_ledger.fresh_run():
        r.append(_ok("next fresh_run clears again", clearance_ledger.get_min() is None))
    clearance_ledger.reset()

    passed = sum(r)
    print(f"\n{passed}/{len(r)} E10 process-global tests passed")
    return 0 if passed == len(r) else 1


if __name__ == "__main__":
    sys.exit(main())
