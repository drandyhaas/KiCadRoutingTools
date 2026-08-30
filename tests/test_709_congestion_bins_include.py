"""congestion_bins gains `include_bins`, and the A* path must not notice (#709).

The census could not represent an EMPTY window because `bins` was keyed off
`owners`, which only gains a key where some net has a terminal. The fix adds
one default-off kwarg. This file is the guard on the half that is dangerous:
`congestion_bins` also backs `build_congestion2`, whose `bins` dict feeds
`congestion2_rows` -> np.array -> merge_track_proximity_costs. Dict INSERTION
ORDER is therefore the row order of an array the router prices cells with, and
congestion-v2 is OFF by default (KICAD_CONGESTION2_COST 0.0), so a regression
there is silent in the whole default suite. Nothing else would catch it.
"""

import os
import sys

RUN_ALL_FAST_OK = True          # no subprocess, no routing -- seconds
RUN_ALL_TIMEOUT = 180

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from congestion_field import congestion_bins            # noqa: E402
from kicad_parser import parse_kicad_pcb                # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')

FAILURES = []


def report(name, ok, detail=''):
    print(('  PASS  ' if ok else '  FAIL  ') + name
          + (('  -- ' + detail) if detail else ''))
    if not ok:
        FAILURES.append(name)


def _census(**kw):
    pcb = parse_kicad_pcb(BOARD)
    ids = list(pcb.nets)
    layers = len(pcb.board_info.copper_layers or ['F.Cu'])
    return congestion_bins(pcb, ids, layers, 2.0, **kw), layers


def t_default_is_byte_for_byte_the_old_iteration():
    """The A*-path guard: same keys, same ORDER, same values.

    `list(...)` not `set(...)`: a set union would pass a key-set assertion and
    still permute the np.array rows congestion2_rows builds.
    """
    (a, _ta, _ba), _l = _census()
    (b, _tb, _bb), _l = _census(include_bins=None)
    report('include_bins=None leaves the key ORDER identical',
           list(a.items()) == list(b.items()),
           '%d vs %d keys' % (len(a), len(b)))


def t_empty_bins_arrive_with_real_free_area():
    (base, _t, _b), layers = _census()
    # Two keys far outside any part: nothing demands them, nothing is on them.
    extra = [(10 ** 5, 10 ** 5), (10 ** 5 + 1, 10 ** 5)]
    (wide, _t2, _b2), _l = _census(include_bins=extra)

    report('every default key survives, with its value unchanged',
           all(k in wide and wide[k] == v for k, v in base.items()))
    report('the default keys keep their order, extras APPENDED after',
           list(wide)[:len(base)] == list(base))
    report('the extra keys are present', all(k in wide for k in extra))

    full = 2.0 * 2.0 * layers
    frees = [wide[k][0] for k in extra]
    owns = [wide[k][1] for k in extra]
    report('an empty window reports NO owner', all(len(o) == 0 for o in owns))
    report('an empty window reports the FULL free area, not the 5% floor',
           all(abs(f - full) < 1e-9 for f in frees),
           'got %s, want %g' % (frees, full))


def t_an_included_key_that_already_has_demand_is_not_duplicated():
    (base, _t, _b), _l = _census()
    owned = sorted(base)[:3]
    (wide, _t2, _b2), _l = _census(include_bins=owned)
    report('re-including an owned key changes nothing at all',
           list(wide.items()) == list(base.items()))


def t_the_build_congestion2_call_site_passes_nothing():
    """Static: the A* builder must not acquire this kwarg by accident."""
    src = open(os.path.join(ROOT, 'py_router', 'congestion_field.py'),
               encoding='utf-8').read()
    body = src.split('def build_congestion2', 1)[1].split('\ndef ', 1)[0]
    report('build_congestion2 never passes include_bins',
           'include_bins' not in body)


TESTS = [
    ('default iteration is unchanged',
     t_default_is_byte_for_byte_the_old_iteration),
    ('empty bins carry real free area', t_empty_bins_arrive_with_real_free_area),
    ('an owned key is not duplicated',
     t_an_included_key_that_already_has_demand_is_not_duplicated),
    ('the A* builder passes nothing',
     t_the_build_congestion2_call_site_passes_nothing),
]


def main():
    print('congestion_bins include_bins (#709)')
    for label, fn in TESTS:
        print(' ' + label)
        fn()
    if FAILURES:
        print('\nFAILED: ' + ', '.join(FAILURES))
        return 1
    print('\nOK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
