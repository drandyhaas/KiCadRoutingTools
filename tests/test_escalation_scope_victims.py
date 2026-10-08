"""Terminal escalation's scope takes in the pre-existing nets the run ripped.

`pcb_data._preexisting_rips` is {net id: name} (rip_up_reroute registers it
that way). The scope used to iterate its keys as NAMES, so an id never
matched a net name and no rip victim ever reached the escalation, although
the comment above it promised exactly that.

Rows:
  - a ripped pre-existing net outside the run's own nets joins the scope, as
    (name, id), after the run's own;
  - one already in the run's nets is not added twice;
  - an id the board no longer has, and a board with no rips, add nothing.

    python3 tests/test_escalation_scope_victims.py
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from route import _escalation_scope  # noqa: E402


def _board(rips):
    nets = {i: types.SimpleNamespace(name=n)
            for i, n in ((1, '/A'), (2, '/B'), (3, '/OLD'))}
    pcb = types.SimpleNamespace(nets=nets)
    if rips is not None:
        pcb._preexisting_rips = rips
    return pcb


def test_victims_join():
    own = [('/A', 1)]
    got = _escalation_scope(own, _board({3: '/OLD', 1: '/A', 9: '/GONE'}))
    assert got == [('/A', 1), ('/OLD', 3)], got
    assert own == [('/A', 1)], "the caller's list was changed"


def test_no_rips():
    assert _escalation_scope([('/B', 2)], _board(None)) == [('/B', 2)]
    assert _escalation_scope([('/B', 2)], _board({})) == [('/B', 2)]


TESTS = [test_victims_join, test_no_rips]


if __name__ == '__main__':
    fails = 0
    for t in TESTS:
        try:
            t()
            print(f"  PASS {t.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"  FAIL {t.__name__}: {e}")
    print('ALL PASS' if not fails else f'{fails} FAILED')
    sys.exit(1 if fails else 0)
