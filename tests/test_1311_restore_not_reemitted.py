#!/usr/bin/env python3
"""A #134 last-resort restore does not re-emit the input's own copper (#1311).

route.py's #134 last resort restores a refused net's saved route piece by
piece. For a PRE-EXISTING net that route is its custody registration, whose
pieces are the objects the run was handed -- copper the board being written
already carries. The restore put them on the write-list anyway, which made
each piece its own re-emitted twin, and the #284 stale strip strips an
original whose twin is re-emitted. In a nested in-memory sub-run (the plane
finalize's reconnects) that strip reaches the parent through
plane_write_reconcile.consume_inner_strips, which removes by GEOMETRY from the
parent's write-list, from pcb_data and from the file: the restored escape was
erased everywhere. glasgow gap7_s2a_c (the issue's route.py line):
/IO_Banks/Z9_N "restored 10 segment(s) + 2 via(s)" inside the finalize and
shipped with 0 segments and 0 vias where the input had its U30.H3 escape. At
the top level the same re-emission took /RD off pcb_data while the file kept
it (#1310).

restore_write_entry keeps input objects off the write-list (they stay on the
board, so the stale strip leaves them alone) and still writes the pieces that
are the run's own copper.

  1. restore_write_entry: all input -> no entry; all own -> the result
     itself; mixed -> an `extends` entry carrying the own pieces only;
  2. the nested-run chain, end to end on real helpers (stale strip ->
     consume_inner_strips): the old re-emission erases the restored net from
     the board and the write-list; the new entry leaves it whole;
  3. route.py's #134 last resort writes through restore_write_entry.

    python3 tests/test_1311_restore_not_reemitted.py
"""
import ast
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for p in (ROOT, TESTS_DIR, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import make_seg, make_via, make_pcb, make_net        # noqa: E402
from rip_up_reroute import restore_write_entry                  # noqa: E402
from route import compute_stale_input_segments                  # noqa: E402
from plane_write_reconcile import consume_inner_strips          # noqa: E402

FAILS = []
NET = 137


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


def entries():
    print('1. restore_write_entry')
    a, b = make_seg(0, 0, 1, 0, net_id=NET), make_seg(1, 0, 2, 0, net_id=NET)
    v = make_via(2, 0, net_id=NET)
    own = make_seg(2, 0, 3, 0, net_id=NET)
    inputs = {id(a), id(b), id(v)}
    pr = {'net_id': NET, 'new_segments': [a, b], 'new_vias': [v],
          'is_existing_route': True}
    check('every piece is input copper: no write-list entry',
          restore_write_entry(pr, inputs) is None)
    pr2 = {'net_id': NET, 'new_segments': [own], 'new_vias': []}
    check("every piece is the run's own: the result itself",
          restore_write_entry(pr2, inputs) is pr2)
    pr3 = {'net_id': NET, 'new_segments': [a, own], 'new_vias': [v]}
    e = restore_write_entry(pr3, inputs)
    check('mixed: an entry extending the result, own pieces only',
          e is not None and e is not pr3 and e.get('extends') is pr3
          and e['new_segments'] == [own] and e['new_vias'] == [])


def nested_chain():
    print('2. the nested-run chain: stale strip -> consume_inner_strips')

    def run(write_entry):
        # The nested sub-run was handed these objects (the parent's board).
        a = make_seg(10, 10, 12, 10, net_id=NET)
        b = make_seg(12, 10, 12, 12, net_id=NET)
        pcb = make_pcb(nets={NET: make_net(NET, '/IO_Banks/Z9_N')},
                       segments=[a, b])
        inputs = {id(a), id(b)}
        orig_by_net = {NET: [a, b]}
        # rip, then the #134 last resort restores the same objects
        pcb.segments = []
        pruned = {'net_id': NET, 'new_segments': [a, b], 'new_vias': [],
                  'is_existing_route': True}
        pcb.segments = [a, b]
        results = []
        entry = write_entry(pruned, inputs)
        if entry is not None:
            results.append(entry)
        emitted = [s for r in results for s in r['new_segments']]
        stale = compute_stale_input_segments(
            orig_by_net, {NET}, pcb.segments, emitted,
            final_ids={id(s) for s in pcb.segments})
        # The parent appends the sub-run's results, then consumes its strips.
        parent_wl = [{'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
                      'width': s.width, 'layer': s.layer, 'net_id': s.net_id}
                     for s in emitted]
        fs, fv = [], []
        consume_inner_strips({'segments_to_remove': stale}, parent_wl, [],
                             pcb, fs, fv, 'test')
        return len(pcb.segments), len(parent_wl), len(fs)

    old = run(lambda pruned, inputs: pruned)          # the pre-fix behaviour
    check('control: re-emitting the input objects erases the net',
          old == (0, 0, 2), f'(board, write-list, file strips) = {old}')
    new = run(restore_write_entry)
    check('restore_write_entry: the net stays on the board, nothing stripped',
          new == (2, 0, 0), f'(board, write-list, file strips) = {new}')


def wiring():
    print("3. route.py's #134 last resort writes through restore_write_entry")
    tree = ast.parse(open(os.path.join(ROOT, 'py_router', 'route.py'),
                          encoding='utf-8').read())
    br = [f for f in tree.body if isinstance(f, ast.FunctionDef)
          and f.name == 'batch_route']
    calls = [n for n in ast.walk(br[0]) if isinstance(n, ast.Call)
             and getattr(n.func, 'id', None) == 'restore_write_entry'] if br else []
    appends = [n for n in ast.walk(br[0]) if isinstance(n, ast.Call)
               and isinstance(n.func, ast.Attribute) and n.func.attr == 'append'
               and getattr(n.func.value, 'id', None) == 'results'
               and n.args and getattr(n.args[0], 'id', None) == 'pruned'] if br else []
    check('called once, and `results.append(pruned)` is gone',
          len(calls) == 1 and not appends,
          f'{len(calls)} call(s), {len(appends)} direct append(s)')


def main():
    entries()
    nested_chain()
    wiring()
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
