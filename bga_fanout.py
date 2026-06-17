#!/usr/bin/env python3
"""
BGA Fanout Strategy - Creates escape routing for BGA packages.

This is a wrapper script that imports from the bga_fanout package.
"""

from bga_fanout import main

if __name__ == '__main__':
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set
    exit(main())
