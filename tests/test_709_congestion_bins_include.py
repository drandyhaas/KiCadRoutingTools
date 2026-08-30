"""congestion_bins gains `include_bins`, and the A* path must not notice (#709).

The census could not represent an EMPTY window because `bins` was keyed off
`owners`, which only gains a key where some net has a terminal. The fix adds
one default-off kwarg. This file is the guard on the half that is dangerous:
`congestion_bins` also backs `build_congestion2`, whose `bins` dict feeds
`congestion2_rows` -> np.array -> merge_track_proximity_costs.

ON THE ORDER, PRECISELY -- because the first version of this file overstated
it and a review caught that. The array's consumers are order-INSENSITIVE: the
Rust `set_layer_proximity_batch` is a per-cell max-insert, and the SUM branch
goes through np.unique/bincount, so a permutation does not change what the
router prices cells with. What order really decides is the tie-break in
`check_pockets`' own stable `rows.sort(key=-ratio)`. Preserving it is the
right defensive choice either way, but the stake is that, not the cost map.

And it is asserted against an INDEPENDENT re-derivation rather than against a
second call of the same code. Comparing the code with itself passes any
refactor that permutes both sides equally -- including `sorted(set(owners) |
set(include_bins))`, which is a dressed-up version of exactly the union this
test exists to forbid, and which permuted all 44 esp_prog keys while printing
OK.
"""

import ast
import os
import sys

# Nothing here shells out and nothing routes; it runs in seconds.
# The comment goes ABOVE the marker, never after it: run_all's
# _FAST_OK_MARKER is anchored `...True\s*$`, so a trailing comment
# silently voids the opt-out -- and a trailing comment containing the
# word 'subprocess' also TRIPS the integration proxy that the opt-out
# exists to cancel. Measured: this file classified `integration` and was
# skipped by --fast, which is the one lane it most needed to be in.
RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 180

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from congestion_field import congestion_bins            # noqa: E402
from kicad_parser import parse_kicad_pcb                # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
#: 1701 segments, 383 vias -- the board where "part-free" and "empty" differ.
ROUTED = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')

FAILURES = []


def report(name, ok, detail=''):
    print(('  PASS  ' if ok else '  FAIL  ') + name
          + (('  -- ' + detail) if detail else ''))
    if not ok:
        FAILURES.append(name)


def _census(path=BOARD, ids=None, **kw):
    pcb = parse_kicad_pcb(path)
    if ids is None:
        ids = list(pcb.nets)
    layers = len(pcb.board_info.copper_layers or ['F.Cu'])
    return pcb, congestion_bins(pcb, ids, layers, 2.0, **kw), layers


def _expected_key_order(pcb, ids, bin_mm=2.0):
    """The key sequence, RE-DERIVED from the board rather than from the code.

    `owners` is filled net by net over that net's pads, then its segment
    endpoints, then its vias, with `setdefault` -- so first touch wins.
    Writing the derivation out is the point: a bound standing in for a
    derivation is how a wrong version passes the test written for it.
    """
    out, seen = [], set()
    for nid in set(ids):
        pts = [(p.global_x, p.global_y)
               for p in pcb.pads_by_net.get(nid, [])]
        for s in pcb.segments:
            if s.net_id == nid:
                pts.append((s.start_x, s.start_y))
                pts.append((s.end_x, s.end_y))
        for v in pcb.vias:
            if v.net_id == nid:
                pts.append((v.x, v.y))
        for (x, y) in pts:
            b = (int(x // bin_mm), int(y // bin_mm))
            if b not in seen:
                seen.add(b)
                out.append(b)
    return out


def t_the_census_is_not_empty():
    """Everything below is an `all(...)` or a prefix compare, and both are
    vacuously true over nothing. A census that silently returned {} passed all
    eight checks in the first version of this file."""
    _pcb, (bins, terminals, bin_mm), _l = _census()
    report('esp_prog yields a substantial census',
           len(bins) > 20 and len(terminals) > 10 and bin_mm == 2.0,
           '%d bins, %d nets' % (len(bins), len(terminals)))
    _p2, (rb, _t2, _b2), _l2 = _census(ROUTED)
    report('routed_output yields one too', len(rb) > 50, str(len(rb)))


def t_default_order_matches_an_independent_derivation():
    """The order guard, against the derivation -- not against itself."""
    pcb, (a, _t, _b), _l = _census()
    want = _expected_key_order(pcb, list(pcb.nets))
    report('the default key order IS the owners traversal order',
           list(a) == want,
           '%d keys, first divergence at index %s'
           % (len(a), next((i for i, (g, w) in enumerate(zip(list(a), want))
                            if g != w), 'none')))
    _p2, (b, _t2, _b2), _l2 = _census(include_bins=None)
    report('include_bins=None changes nothing at all',
           list(a.items()) == list(b.items()))
    for spelling in ([], (), set()):
        _p3, (c, _t3, _b3), _l3 = _census(include_bins=spelling)
        report('an empty %s takes the default path too'
               % type(spelling).__name__,
               list(c.items()) == list(a.items()))


def t_empty_bins_arrive_with_real_free_area():
    _pcb, (base, _t, _b), layers = _census()
    # Two keys far outside any part: nothing demands them, nothing is on them.
    extra = [(10 ** 5, 10 ** 5), (10 ** 5 + 1, 10 ** 5)]
    _p2, (wide, _t2, _b2), _l = _census(include_bins=extra)

    report('every default key survives, with its value unchanged',
           len(base) > 0
           and all(k in wide and wide[k] == v for k, v in base.items()))
    report('the default keys keep their order, extras APPENDED after',
           len(base) > 0 and list(wide)[:len(base)] == list(base))
    report('the extra keys are present', all(k in wide for k in extra))

    full = 2.0 * 2.0 * layers
    frees = [wide[k][0] for k in extra]
    owns = [wide[k][1] for k in extra]
    report('an empty window reports NO owner', all(len(o) == 0 for o in owns))
    report('an empty window reports the FULL free area, not the 5% floor',
           all(abs(f - full) < 1e-9 for f in frees),
           'got %s, want %g' % (frees, full))


def t_a_no_demand_window_still_subtracts_copper():
    """The distinction #709 exists to make, at the level that computes it.

    A window with no demand under a NARROW net set can be packed with another
    net's copper. If `free` stopped subtracting copper for newly-included keys
    every one of them would read "empty" -- and the two off-board probes above
    cannot tell the difference, because they carry no copper either.

    The reference is the ALL-NETS census, which reaches the same bin through
    the ordinary `owners` path: same bin, same copper, so the two must agree
    to the bit.
    """
    pcb = parse_kicad_pcb(ROUTED)
    layers = len(pcb.board_info.copper_layers or ['F.Cu'])
    all_ids = list(pcb.nets)
    full_bins, _t, _b = congestion_bins(pcb, all_ids, layers, 2.0)
    one = [nid for nid, n in pcb.nets.items() if n.name == 'GND'] or all_ids[:1]
    narrow, _t2, _b2 = congestion_bins(pcb, one, layers, 2.0)

    hidden = [k for k in full_bins if k not in narrow]
    report('the narrow set really does hide windows the wide one sees',
           len(hidden) > 50, '%d hidden' % len(hidden))
    wide, _t3, _b3 = congestion_bins(pcb, one, layers, 2.0,
                                     include_bins=hidden)
    cap = 2.0 * 2.0 * layers
    withcopper = [k for k in hidden if full_bins[k][0] < cap - 1e-9]
    report('  ...and many of them carry copper', len(withcopper) > 20,
           '%d of %d' % (len(withcopper), len(hidden)))
    bad = [k for k in withcopper if abs(wide[k][0] - full_bins[k][0]) > 1e-9]
    report('a re-included window reports the SAME free area as the wide census',
           not bad, '%d disagree, e.g. %s' % (len(bad), bad[:2]))
    report('  ...which is strictly below the empty-window capacity',
           bool(withcopper)
           and all(wide[k][0] < cap - 1e-9 for k in withcopper))
    report('  ...and it has no owner under the narrow set',
           bool(hidden) and all(len(wide[k][1]) == 0 for k in hidden))


def t_an_included_key_that_already_has_demand_is_not_duplicated():
    _pcb, (base, _t, _b), _l = _census()
    owned = sorted(base)[:3]
    report('the probe keys are real, not an empty slice', len(owned) == 3)
    _p2, (wide, _t2, _b2), _l2 = _census(include_bins=owned)
    report('re-including an owned key changes nothing at all',
           list(wide.items()) == list(base.items()))


def t_the_build_congestion2_call_site_passes_nothing():
    """AST, not a token scan.

    A substring check misses `**{'include' + '_bins': ...}` and a forwarding
    helper, and fires on a comment that merely mentions the name. What has to
    be true is a property of the CALL: `build_congestion2` reaches
    `congestion_bins` with no `include_bins` keyword and no `**` splat.
    """
    src = open(os.path.join(ROOT, 'py_router', 'congestion_field.py'),
               encoding='utf-8').read()
    tree = ast.parse(src)
    fns = [n for n in ast.walk(tree)
           if isinstance(n, ast.FunctionDef) and n.name == 'build_congestion2']
    report('build_congestion2 is still there to check', len(fns) == 1)
    if len(fns) != 1:
        return
    calls = [c for c in ast.walk(fns[0]) if isinstance(c, ast.Call)
             and isinstance(c.func, ast.Name)
             and c.func.id == 'congestion_bins']
    report('  it calls congestion_bins exactly once', len(calls) == 1,
           str(len(calls)))
    for c in calls:
        report('  with no include_bins keyword',
               not any(k.arg == 'include_bins' for k in c.keywords))
        report('  and no ** splat that could smuggle one in',
               not any(k.arg is None for k in c.keywords))
        report('  and at most 4 positionals (the 5th slot is not it)',
               len(c.args) <= 4, str(len(c.args)))
    # ...and it must not reach the kwarg through a helper either.
    helpers = [c for c in ast.walk(fns[0]) if isinstance(c, ast.Call)
               and any(k.arg == 'include_bins' for k in c.keywords)]
    report('  and passes include_bins to nothing else', not helpers)


def t_a_bad_include_bins_shape_is_refused_at_the_boundary():
    """A bare (bx, by) instead of a LIST of them used to insert an int key,
    which then failed far away in the consumer's `for (bx, by), ... in
    bins.items()`. Fail where the mistake is, not three frames later."""

    def raises(arg):
        try:
            _census(include_bins=arg)
        except (TypeError, ValueError):
            return True
        except Exception:                                       # noqa: BLE001
            return False
        return False

    report('a bare (bx, by) tuple is refused, not silently inserted',
           raises((10 ** 5, 10 ** 5)))
    report('a flat list of ints is refused', raises([1, 2, 3]))
    report('a list of real keys is still accepted',
           not raises([(10 ** 5, 10 ** 5)]))
    report('and so is an iterator of them',
           not raises(iter([(10 ** 5, 10 ** 5)])))


TESTS = [
    ('the census is non-empty', t_the_census_is_not_empty),
    ('default order == the derivation',
     t_default_order_matches_an_independent_derivation),
    ('empty bins carry real free area', t_empty_bins_arrive_with_real_free_area),
    ('a no-demand window still subtracts copper',
     t_a_no_demand_window_still_subtracts_copper),
    ('an owned key is not duplicated',
     t_an_included_key_that_already_has_demand_is_not_duplicated),
    ('the A* builder passes nothing (AST)',
     t_the_build_congestion2_call_site_passes_nothing),
    ('a bad include_bins shape is refused',
     t_a_bad_include_bins_shape_is_refused_at_the_boundary),
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
    print('congestion_bins include_bins (#709)')
    for label, fn in TESTS:
        print(' ' + label)
        fn()
    _every_case_is_registered()
    if FAILURES:
        print('\nFAILED: ' + ', '.join(FAILURES))
        return 1
    print('\nOK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
