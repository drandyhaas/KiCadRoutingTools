#!/usr/bin/env python3
"""#962 census: does the graphic-copper off-outline rule fire on ORIGINAL boards?

The rule (`check_drc.footprint_graphic_outline_census` + GRAPHIC_WAIVED_STATES)
turns footprint graphic copper past the outline into a `graphic-off-board`
violation unless the owner is board-level art or draws the board outline. The
same census feeds `placement.legality`'s `oob_graphic_copper_*` and
render_placement's `checklist.a_off_outline.graphic_copper`. So an original
board it fires on would fail check_drc, place_pose `legal`, and
render_placement --gate at once.

This measures, per board, using only the production functions:
- every graphic copper owner and its state;
- the worst overrun (signed: negative means clear by that much);
- the copper the parser does not model.

    python3 -X utf8 tests/measure_962_graphic_waiver_census.py [extra_board_dir ...] [--json OUT]

A measurement tool; `run_all` does not collect `measure_*`.
"""
import glob
import json
import os
import sys
from collections import defaultdict

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb  # noqa: E402
from check_drc import footprint_graphic_outline_census, GRAPHIC_WAIVED_STATES  # noqa: E402
from run_utils import corpus_boards  # noqa: E402


def main(argv):
    out_json = None
    if '--json' in argv:
        i = argv.index('--json')
        out_json = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    boards = [(os.path.join(ROOT_DIR, b) if not os.path.isabs(b) else b)
              for b in corpus_boards()]
    for d in argv:
        boards += sorted(glob.glob(os.path.join(os.path.expanduser(d), '*.kicad_pcb')))
    if not boards:
        print('NOT RUN: no boards')
        return 2
    results, fired = [], 0
    for b in boards:
        name = os.path.basename(b)
        try:
            p = parse_kicad_pcb(b)
            c = footprint_graphic_outline_census(p)
        except Exception as e:
            print('%-40s ERROR %s: %s' % (name[:40], type(e).__name__, e))
            results.append({'board': name, 'error': repr(e)})
            continue
        owners = defaultdict(lambda: [None, -1e9])
        for r in c['rows']:
            o = owners[r['owner_ref'] or '<board>']
            o[0] = r['owner_state']
            o[1] = max(o[1], r['overrun_mm'])
        flagged = {k: v for k, v in owners.items()
                   if v[1] > 1e-6 and v[0] not in GRAPHIC_WAIVED_STATES}
        fired += bool(flagged)
        if owners or c['unmeasured']:
            print('%-40s owners=%s flagged=%s unmeasured=%s' % (
                name[:40],
                {k: (v[0], round(v[1], 3)) for k, v in sorted(owners.items())},
                sorted(flagged), [u['owner_ref'] for u in c['unmeasured']]))
        results.append({'board': name,
                        'owners': {k: list(v) for k, v in owners.items()},
                        'flagged': sorted(flagged), 'unmeasured': c['unmeasured']})
    print('\n%d board(s); rule fires on %d original(s)' % (len(results), fired))
    if out_json:
        with open(out_json, 'w', encoding='utf-8') as fh:
            json.dump(results, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
