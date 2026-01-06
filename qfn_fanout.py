#!/usr/bin/env python3
"""
QFN/QFP Fanout CLI wrapper.

This is a thin wrapper that calls the qfn_fanout package.
See qfn_fanout/README.md for documentation.
"""

from qfn_fanout import main

if __name__ == '__main__':
    exit(main())
