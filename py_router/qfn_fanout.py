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
    # #653: the other routing mains get this from cli_banner.install(); the
    # fanout fronts do not install the banner, so print the knob inventory
    # directly rather than leave two mains silently unaudited.
    try:
        import env_knobs as _ek653
        print(_ek653.env_knobs_line(), flush=True)
    except Exception:
        pass
    exit(main())
