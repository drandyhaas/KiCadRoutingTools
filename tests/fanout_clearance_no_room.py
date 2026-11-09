#!/usr/bin/env python3
"""place_fanout_clearance.py with #1681's make-room rung held off.

The decap comparison tests (#1067, #1122) need a rig where the gated pass
boxes a cap in by its holds and the pass without the gate clears it, so the
UNGATED run is kept. Since #1681 the gated pass moves another part to make
room for that cap, and on their fixture it is no longer worse at any budget.
This runs the CLI unchanged except for that one rung, so those tests keep
testing the comparison and the claims it hands the ungated arm.

    python3 tests/fanout_clearance_no_room.py <place_fanout_clearance args>
"""
import os
import runpy
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import placement.fanout_clearance as FC  # noqa: E402

FC._clear_by_moving_one = lambda *a, **k: None
sys.argv[0] = os.path.join(ROOT, 'py_placer', 'place_fanout_clearance.py')
runpy.run_path(sys.argv[0], run_name='__main__')
