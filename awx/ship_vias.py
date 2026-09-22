#!/usr/bin/env python3
"""ship_vias.py -- a via awx lays in a pad or a paste opening declares
IPC-4761 Type VII (filled + capped), as the route step does at ship time
(#962, `fab_notes.via_protection_stamps`). `check_drc` grades a via under
solder that declares neither as `via-in-paste`, so a fanout board written
without the declaration fails the chain's own DRC gate (the pairs bench at
K36: 11 such vias, the realize step refusing every move; 2026-09-22, the
merge of main). One call after every write that can put a via in a pad:
the plan's fanout, the source realize, the bench's source comb."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))


def stamp(path, context='awx', log=None):
    """Stamp the written board at `path` in place; the record, or None."""
    if not path or not os.path.exists(path):
        return None
    try:
        import fab_notes
        rec = fab_notes.ship_via_protection_file(path, [], context, quiet=True)
    except Exception as e:                                       # noqa: BLE001
        if log:
            log(f'  (via protection stamp skipped: {type(e).__name__}: {e})')
        return None
    if log and rec and rec.get('stamped'):
        log(f'  via protection ({context}): {rec["stamped"]} via(s) in a pad or '
            f'paste opening declared Type VII')
    return rec
