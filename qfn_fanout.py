#!/usr/bin/env python3
"""
QFN/QFP Fanout CLI wrapper.

This is a thin wrapper that calls the qfn_fanout package.
See qfn_fanout/README.md for documentation.
"""

from qfn_fanout import main

if __name__ == '__main__':
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set
    exit(main())
