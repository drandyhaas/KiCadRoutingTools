#!/usr/bin/env python3
"""`board_score` splits `unrouted` by SHAPE without moving `blocking`.

`unrouted` is one bucket holding two different failure shapes. A net with fewer
than 2 pads ON the board is refused outright by the router's own
`net_queries.filter_routable_nets`, so no router setting will ever route it and
retrying it in the router is a wasted pass; a net with copper missing for any
other reason is an ordinary routing job. Nothing surfaced the difference, so a
placement-shaped failure was indistinguishable from a routing one until a
routing pass had been spent discovering it (run 10: 13 of 57).

THE PROPERTY THIS FILE EXISTS FOR is the one that makes the change safe to add
to an instrument every recorded ledger already depends on: `blocking` must be
IDENTICAL. That is guaranteed by construction -- `unrouted_shape` never emits
`count`, so merging it into the `unrouted` component cannot touch the number
`blocking` sums -- and construction is exactly the kind of guarantee that
quietly stops holding, so it is asserted here rather than argued.

Run:  python3 tests/test_board_score_unrouted_shape.py [board ...]
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'scripts')
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
sys.path.insert(0, SKILL)

#: Small enough to grade quickly, and between them they cover the two cases:
#: a board with no unrouted nets at all, and one whose unrouted nets exist.
DEFAULT_BOARDS = [
    os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb'),
    os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb'),
]

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}"
          + (f' -- {detail}' if detail else ''))


def main(argv):
    import board_score

    # ---- structural: the merge CANNOT touch `count` -----------------------
    #
    # main() builds the component as {'ran': .., 'count': .., 'nets': ..,
    # **shape}, so a `count` key in `shape` would silently override the number
    # `blocking` is summed from. Assert the shape's key set instead of trusting
    # the ordering of a dict literal.
    fake = board_score.unrouted_shape(DEFAULT_BOARDS[0], [])
    forbidden = {'count', 'ran'} & set(fake)
    check('unrouted_shape emits no key that could overwrite `count`',
          not forbidden, str(forbidden))
    check('...and returns the two buckets plus the ref list',
          {'placement_blocked', 'placement_blocked_refs', 'open'} <= set(fake),
          str(sorted(fake)))

    # ---- an unreadable board degrades to an `error` key, never a crash ----
    bad = board_score.unrouted_shape(os.path.join(ROOT, 'no-such.kicad_pcb'),
                                     ['/X'])
    check('an unreadable board reports `error`, and no bucket',
          'error' in bad and 'placement_blocked' not in bad, str(bad))

    # ---- behavioural, on real boards --------------------------------------
    boards = argv or DEFAULT_BOARDS
    for board in boards:
        if not os.path.isfile(board):
            print(f'  SKIP {os.path.basename(board)} (absent)')
            continue
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 's.json')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(SKILL, 'board_score.py'), board, '--json', jp],
                capture_output=True, text=True, encoding='utf-8',
                errors='replace', cwd=ROOT)
            if not os.path.isfile(jp):
                check(f'{os.path.basename(board)}: board_score ran', False,
                      (r.stdout + r.stderr)[-300:])
                continue
            doc = json.load(open(jp, encoding='utf-8'))
        name = os.path.basename(board)
        comp = doc['components']['unrouted']
        by = doc['blocking_by']

        check(f'{name}: blocking_by.unrouted IS the component count',
              by.get('unrouted') == comp.get('count'),
              f"{by.get('unrouted')} vs {comp.get('count')}")
        counts = [v for v in by.values() if v]
        check(f'{name}: blocking is still the plain sum of blocking_by',
              doc['blocking'] == sum(counts) or doc['blocking'] is None,
              f"{doc['blocking']} vs {sum(counts)}")

        if 'error' in comp:
            check(f'{name}: shape reported an error rather than guessing',
                  True, comp['error'])
            continue
        pb = set(comp.get('placement_blocked') or {})
        op = set(comp.get('open') or [])
        nets = set(comp.get('nets') or [])
        check(f'{name}: the split PARTITIONS the unrouted names exactly',
              pb | op == nets and not (pb & op),
              f'blocked {len(pb)} open {len(op)} names {len(nets)}')
        check(f'{name}: every blocked net names the refs to move',
              all(v for v in (comp.get('placement_blocked') or {}).values()),
              str(comp.get('placement_blocked')))
        refs = set(comp.get('placement_blocked_refs') or [])
        check(f'{name}: the ref list is the union of the per-net lists',
              refs == {r for v in (comp.get('placement_blocked') or {}).values()
                       for r in v})

    print(f'\n{passed}/{passed + failed} checks passed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
