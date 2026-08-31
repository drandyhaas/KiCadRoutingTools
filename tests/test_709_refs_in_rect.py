"""One rectangle, one answer: `placement.utility.refs_in_rect` (#709).

The census PRINTS a rectangle and `place_seed --reseat-region` LIFTS the parts
in it. If the two resolve the same rectangle differently -- a closed interval
here, a half-open one there, pad centres on one side and footprint origins on
the other -- the census names parts the mover does not touch, and the reseat
target it advertises is a lie. So there is one implementation and this is its
contract.
"""

import os
import sys

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 120

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import parse_kicad_pcb                        # noqa: E402
from placement.utility import refs_in_rect                      # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')

FAILURES = []


def report(name, ok, detail=''):
    print(('  PASS  ' if ok else '  FAIL  ') + name
          + (('  -- ' + detail) if detail else ''))
    if not ok:
        FAILURES.append(name)


class _Pad:
    def __init__(self, x, y, ref):
        self.global_x, self.global_y, self.component_ref = x, y, ref


class _Fp:
    def __init__(self, x, y, pads):
        self.x, self.y, self.pads = x, y, pads


class _Pcb:
    def __init__(self, fps):
        self.footprints = fps


def _synthetic():
    """Four pads, one on each boundary of the window [0,0]-[10,10]."""
    return _Pcb({
        'LO': _Fp(0.0, 0.0, [_Pad(0.0, 0.0, 'LO')]),        # on the low corner
        'HI': _Fp(10.0, 10.0, [_Pad(10.0, 10.0, 'HI')]),    # on the high corner
        'IN': _Fp(5.0, 5.0, [_Pad(5.0, 5.0, 'IN')]),
        'OUT': _Fp(20.0, 20.0, [_Pad(20.0, 20.0, 'OUT')]),
    })


def t_the_interval_is_half_open():
    """A tiling must give every point to exactly ONE window.

    A closed interval hands a part on a shared edge to both neighbours, so two
    adjacent census windows would each claim it and the counts stop summing.
    """
    pcb = _synthetic()
    got = refs_in_rect(pcb, (0.0, 0.0, 10.0, 10.0))
    report('the LOW corner is inside', 'LO' in got, str(got))
    report('the HIGH corner is NOT (half-open)', 'HI' not in got, str(got))
    report('the interior is inside', 'IN' in got, str(got))
    report('a distant part is out', 'OUT' not in got, str(got))

    # The tiling property itself, which is what half-open buys.
    a = refs_in_rect(pcb, (0.0, 0.0, 5.0, 10.0))
    b = refs_in_rect(pcb, (5.0, 0.0, 10.0, 10.0))
    report('two abutting windows never claim the same ref',
           not (set(a) & set(b)), '%s / %s' % (a, b))
    report('  ...and together they claim what the union window claims',
           set(a) | set(b) == set(refs_in_rect(pcb, (0.0, 0.0, 10.0, 10.0))))


def t_the_two_modes_answer_different_questions():
    pcb = _synthetic()
    # A part whose ORIGIN is outside the window but whose PAD reaches into it.
    pcb.footprints['REACH'] = _Fp(50.0, 50.0, [_Pad(5.0, 5.0, 'REACH')])
    by_pad = refs_in_rect(pcb, (0.0, 0.0, 10.0, 10.0))
    by_origin = refs_in_rect(pcb, (0.0, 0.0, 10.0, 10.0), by='origin')
    report("by='pad' sees a part reaching in from outside",
           'REACH' in by_pad, str(by_pad))
    report("by='origin' does not", 'REACH' not in by_origin, str(by_origin))
    report('an unknown mode is refused, not silently treated as one of them',
           _raises(lambda: refs_in_rect(pcb, (0, 0, 1, 1), by='centre')))


def _raises(fn):
    try:
        fn()
    except ValueError:
        return True
    except Exception:                                           # noqa: BLE001
        return False
    return False


def t_it_matches_what_the_census_used_to_do_inline():
    """The census's old inline pad scan, replayed on a real board."""
    pcb = parse_kicad_pcb(BOARD)
    win = (126.0, 102.0, 128.0, 104.0)
    old = sorted({p.component_ref
                  for fp in pcb.footprints.values() for p in fp.pads
                  if win[0] <= p.global_x < win[2]
                  and win[1] <= p.global_y < win[3] and p.component_ref})
    report('same answer as the inline scan it replaced',
           refs_in_rect(pcb, win) == old, str(old))
    report('and the answer is not vacuously empty', bool(old), str(old))


def t_a_degenerate_rect_names_nothing():
    pcb = _synthetic()
    report('a zero-area rect claims nothing',
           refs_in_rect(pcb, (5.0, 5.0, 5.0, 5.0)) == [])
    report('an inverted rect claims nothing',
           refs_in_rect(pcb, (10.0, 10.0, 0.0, 0.0)) == [])


TESTS = [
    ('the interval is half-open', t_the_interval_is_half_open),
    ('pad vs origin', t_the_two_modes_answer_different_questions),
    ('parity with the old inline scan', t_it_matches_what_the_census_used_to_do_inline),
    ('degenerate rects', t_a_degenerate_rect_names_nothing),
]


def _every_case_is_registered():
    """A `t_*` defined and left out of TESTS is a test that never runs.

    That happened here once, to the row that pins the quadrant counts against
    `_region_unit`. The mutation battery is what noticed, which is a long way
    round for something the module can check on itself in three lines.
    """
    g = globals()
    declared = {fn for _l, fn in TESTS}
    missing = sorted(n for n, v in g.items()
                     if n.startswith('t_') and callable(v)
                     and v not in declared)
    if missing:
        print('  FAIL  every t_* case is registered in TESTS  -- ORPHANED: %s'
              % ', '.join(missing))
        FAILURES.append('unregistered cases: %s' % ', '.join(missing))


def main():
    print('refs_in_rect, the one resolver (#709)')
    for label, fn in TESTS:
        print(' ' + label)
        fn()
    _every_case_is_registered()
    if FAILURES:
        print('\nFAILED (%d): %s' % (len(FAILURES), ', '.join(FAILURES)))
        return 1
    print('\nOK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
