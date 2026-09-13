#!/usr/bin/env python3
"""Compatibility shim: this tool moved to ``py_placer/`` in the #522 reorg.

The placement family was split out of ``py_router/`` into ``py_placer/``, but
**297 recorded stress chains across 172 boards still invoke this path**
(``redo_commands.sh`` is a replay manifest -- it records the command line that
was run, verbatim, and those recordings are not ours to rewrite). Without this
file every one of those chains dies mid-replay:

    python3: can't open file '.../py_router/place_fanout_clearance.py'
      -> 0.50s  exit 2
      (non-check command failed)

and the board is abandoned with no final artifact, which a corpus A/B then
reads as 48 boards that "failed to complete" -- a path break wearing the
costume of a routing regression. Measured exactly that way on sets 1-10:
placement produced 100 chain-complete boards against main's 147, entirely from
this one missing path.

Deliberately a forwarder rather than a copy: there is ONE implementation, in
``py_placer/place_fanout_clearance.py``. ``run_name='__main__'`` so its
``if __name__ == '__main__'`` block runs and argv/exit codes pass straight
through, and ``py_placer`` goes on ``sys.path`` first so the real module's own
``import _path`` bootstrap resolves.

Do not delete this without rewriting the recorded corpus -- and note that
rewriting it would make those chains replayable ONLY on branches that carry the
#522 layout, so main and placement could no longer be A/B'd on the same corpus.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'routing'], 'kind': 'actor'}

import os
import runpy
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REAL = os.path.join(_ROOT, 'py_placer', 'place_fanout_clearance.py')

if not os.path.isfile(_REAL):
    sys.exit(f"place_fanout_clearance: real tool missing at {_REAL}")

for _d in (os.path.join(_ROOT, 'py_placer'), os.path.join(_ROOT, 'py_tools')):
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.insert(0, _d)

runpy.run_path(_REAL, run_name='__main__')
