#!/usr/bin/env python3
"""#962 census: where would a check_drc `via-in-paste` finding fire today?

Before the grade lands, this measures what it would say on real boards, using
the PRODUCTION code it will call:
- `fab_notes.via_paste_sites`, over `paste_apertures.apertures_for_net`;
- `fab_notes.effective_via_protection` / `is_filled_and_capped`, over the
  parsed `via_protection_setup`.

Nothing here re-derives any of that. For every board it counts vias whose
barrel overlaps a paste opening that concerns their own net, split by the
opening's source and by whether the via's effective protection is Type VII.

    python3 -X utf8 tests/measure_962_via_in_paste_census.py [extra_board_dir ...] [--json OUT]

With no directories, it measures the tracked corpus (`run_utils.corpus_boards`).
Every extra directory adds its `*.kicad_pcb` files, e.g. the local stress-set
originals.

A measurement tool, not a test: `run_all` does not collect `measure_*`.
"""
import glob
import json
import os
import sys
from collections import Counter

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb  # noqa: E402
import fab_notes  # noqa: E402
from run_utils import corpus_boards  # noqa: E402


def census(path):
    p = parse_kicad_pcb(path)
    setup = p.board_info.via_protection_setup
    rows = []
    for via, ap, pen in fab_notes.via_paste_sites(p.vias, p):
        eff = fab_notes.effective_via_protection(getattr(via, 'tenting_attrs', {}), setup)
        net = p.nets.get(via.net_id)
        rows.append({
            'x': round(via.x, 4), 'y': round(via.y, 4), 'size': via.size,
            'net': net.name if net else str(via.net_id),
            'aperture': ap.label(), 'source': ap.source,
            'penetration_mm': round(pen, 4),
            'type_vii': fab_notes.is_filled_and_capped(eff),
            'own_spec': dict(getattr(via, 'tenting_attrs', {}) or {}),
        })
    return {'board': os.path.basename(path), 'vias': len(p.vias),
            'setup_capping': setup.get('capping'), 'setup_filling': setup.get('filling'),
            'hits': rows}


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
        print('NOT RUN: no boards (git cannot list the corpus and no dir given)')
        return 2
    results = []
    print('%-44s %5s %5s %5s %6s %6s %6s' % ('board', 'vias', 'hits', 'pad', 'graph', 'p-only', 'unprot'))
    tot = Counter()
    for b in boards:
        try:
            r = census(b)
        except Exception as e:      # a board the parser cannot read is REPORTED
            print('%-44s ERROR %s: %s' % (os.path.basename(b)[:44], type(e).__name__, e))
            results.append({'board': os.path.basename(b), 'error': repr(e)})
            continue
        src = Counter(h['source'] for h in r['hits'])
        unp = sum(1 for h in r['hits'] if not h['type_vii'])
        tot.update({'boards': 1, 'vias': r['vias'], 'hits': len(r['hits']),
                    'unprot': unp, 'boards_firing': 1 if unp else 0, **src})
        print('%-44s %5d %5d %5d %6d %6d %6d' % (
            r['board'][:44], r['vias'], len(r['hits']), src['pad'], src['graphic'],
            src['paste_only_pad'], unp))
        results.append(r)
    print('\nTOTAL boards=%d vias=%d hits=%d (pad %d, graphic %d, paste-only %d) '
          'UNPROTECTED=%d on %d board(s)' % (
              tot['boards'], tot['vias'], tot['hits'], tot['pad'], tot['graphic'],
              tot['paste_only_pad'], tot['unprot'], tot['boards_firing']))
    if out_json:
        with open(out_json, 'w', encoding='utf-8') as fh:
            json.dump(results, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
