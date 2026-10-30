"""#1540: the anneal prices a zone member's escape from its containment zone
as an AREA -- the mm outside times the courtyard's half-perimeter -- the
way it prices a part off the board.

In plain mm the pull home lost to the mm2 of overlap a member would cost to
get back in, so the anneal left zone members outside their zones and every
seed ended illegal. Keep-out and exclusive-zone terms are already mm2 and
keep their price.

Rows (the anneal's intent term, read on a stub state):
  - a member 2 mm past its zone's tolerance costs 2 x (w + h);
  - a keep-out intrusion of 3 mm2 still costs 3, beside it;
  - a member inside its zone (at or under the tolerance) costs nothing;
  - a part with no claims costs nothing.

    python3 tests/test_1540_zone_escape_priced_as_area.py
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from placement.quench import _IntentTerm  # noqa: E402
from placement.simple_anneal import Annealer  # noqa: E402

W, H = 6.0, 2.0     # the member's courtyard


class _Part:
    x, y, rot = 10.0, 5.0, 0.0

    def rect(self):
        return (self.x - W / 2, self.y - H / 2, self.x + W / 2, self.y + H / 2)

    def grade_rects(self, x, y, rot):
        return (self.rect(), ())


class _State:
    def __init__(self, terms, vals):
        self.parts = {'R1': _Part(), 'R2': _Part()}
        self._spec = {'R1': terms}
        self._vals = {'R1': vals}

    def intent_spec_for(self, ref):
        return self._spec.get(ref, ())

    def intent_terms(self, ref, rects):
        return self._vals[ref]


class _A:
    """Only what `Annealer._intent` reads."""
    def __init__(self, st):
        self.st = st


def _term(rule, threshold):
    return _IntentTerm(rule=rule, name=rule, rect=(0, 0, 1, 1), threshold=threshold,
                       anchor=False, entry=None)


def main():
    fails = []

    def ok(label, got, want):
        good = abs(got - want) < 1e-9
        print(f"  [{'ok' if good else 'FAIL'}] {label}: {got:g}"
              f"{'' if good else f' (want {want:g})'}")
        if not good:
            fails.append(label)

    zone, ko = _term('zone_containment', 0.5), _term('keepout', 0.0)
    a = _A(_State((zone, ko), (2.5, 3.0)))
    ok('2 mm past the zone costs 2 x (w + h), a 3 mm2 keep-out costs 3',
       Annealer._intent(a, 'R1'), 2.0 * (W + H) + 3.0)
    a = _A(_State((zone,), (0.5,)))
    ok('a member at its tolerance costs nothing', Annealer._intent(a, 'R1'), 0.0)
    ok('a part with no claims costs nothing', Annealer._intent(a, 'R2'), 0.0)

    print('ALL PASS' if not fails else f'{len(fails)} FAILED')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
