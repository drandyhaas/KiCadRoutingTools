#!/usr/bin/env python3
"""pair_order's net-id caches live on the state they were built from.

`pair_metrics` reads each part's scoring nets through two caches kept on the
state (`_scoring_net_ids`, `_part_net_set`), so a pass over every pair does not
rebuild `set(state.net_refs)` per call. A cache shared between two states would
answer one board's question with another board's nets.

Run: python3 -X utf8 tests/test_893_pair_order_equivalence.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 600
RUN_ALL_FAST_OK = True

TESTS = []


def _state(board_name):
    from kicad_parser import parse_kicad_pcb
    import pose_score
    path = run_utils.evidence(os.path.join(ROOT, 'kicad_files', board_name),
                              'corpus board')
    pcb = parse_kicad_pcb(path)
    return pose_score.make_state(pcb, path)


def test_caches_are_not_shared_between_states():
    """Two states over different boards must not see each other's caches."""
    from placement import pair_order as PO
    a = _state('esp_prog.kicad_pcb')
    b = _state('tigard.kicad_pcb')
    ids_a = PO._scoring_net_ids(a)
    ids_b = PO._scoring_net_ids(b)
    assert ids_a is not ids_b, 'the scoring-net cache is shared between states'
    assert PO._scoring_net_ids(a) is ids_a, 'the cache is not sticky'
    print('  caches are per-state and sticky')


TESTS.append(test_caches_are_not_shared_between_states)


def main():
    failures = 0
    for fn in TESTS:
        print('%s ...' % fn.__name__)
        try:
            fn()
        except AssertionError as exc:
            failures += 1
            print('FAIL %s: %s' % (fn.__name__, exc))
    if failures:
        print('%d/%d checks FAILED' % (failures, len(TESTS)))
        return 1
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
