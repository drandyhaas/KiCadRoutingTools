"""The board-lattice inference (#708): what it may return, and when it declines.

The issue proposes an ARGMAX over a ladder of pitches. That rule is undefined
exactly where it matters, because the ladder contains divisibility chains and
occupancy is monotone along one -- so ties are structural, not accidental.
These cases pin the two rules that replace it (finest-within-slack, and a
minimum sample count), the corpus verdicts they produce, and the three claims
in #708 that measurement contradicts.

Cheap and hermetic: parsing 22 boards, no quench, no routing.
"""

import math
import os
import sys

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 300

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import run_utils                                               # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import board_grid as BG                         # noqa: E402

FAILURES = []


def report(name, ok, detail=''):
    print(('  PASS  ' if ok else '  FAIL  ') + name
          + (('  -- ' + detail) if detail else ''))
    if not ok:
        FAILURES.append(name)


class _FP:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _PCB:
    """The only surface `board_coordinates` touches."""
    def __init__(self, xys):
        self.footprints = {'R%d' % i: _FP(x, y)
                           for i, (x, y) in enumerate(xys)}


def on_lattice(step, n, origin_units=16):
    """n parts whose x and y are exact multiples of `step`.

    Anchored at 0, because that is where the module anchors and where real
    boards put their lattice -- measured, a best-phase search over the corpus
    never beat the origin-anchored fit on any board. A fixture offset by a
    non-multiple (10.0 mm against a 0.635 lattice) is not an on-lattice board
    at all, which is how the first draft of these cases failed.
    """
    return _PCB([(round((origin_units + i * 3) * step, 9),
                  round((origin_units + i * 5) * step, 9))
                 for i in range(n)])


_CORPUS = {}


def corpus():
    if not _CORPUS:
        boards = run_utils.corpus_boards()
        if not boards:
            return None
        for p in boards:
            name = os.path.basename(p).replace('.kicad_pcb', '')
            _CORPUS[name] = BG.infer_board_grid(parse_kicad_pcb(p))
    return _CORPUS


# ------------------------------------------------------------------ cases

def t_occupancy_counts_what_it_says():
    vals = [0.0, 0.3175, 0.635, 0.4]          # 3 of 4 on 0.3175
    report('occupancy is a plain hit rate',
           abs(BG.occupancy(vals, 0.3175) - 0.75) < 1e-12,
           '%.4f' % BG.occupancy(vals, 0.3175))
    report('a degenerate step scores 0 rather than dividing by it',
           BG.occupancy(vals, 0.0) == 0.0 and BG.occupancy([], 0.1) == 0.0)
    # The tolerance is absolute in mm, not relative to the step.
    report('the 1um tolerance is absolute',
           BG.occupancy([0.0009], 1.0) == 1.0
           and BG.occupancy([0.0011], 1.0) == 0.0)


def t_finest_within_slack_not_argmax():
    """A board on 0.635 has occupancy 1.00 at 0.635 AND at 0.3175, because
    0.3175 divides it. The argmax #708 proposes has no answer; this returns
    the finer of the tied pair, deliberately."""
    ev = BG.infer_board_grid(on_lattice(0.635, 20))
    report('a pure 0.635 board reports the tie',
           set(ev['ties']) >= {0.3175, 0.635}, str(ev['ties']))
    report('and resolves to the FINEST of it, not the argmax',
           ev['step'] == 0.3175, str(ev['step']))
    # Why finer is the safe direction: every pose the coarse lattice offers is
    # still offered by the fine one, so nothing is removed. The reverse is not
    # true, and picking coarse is how sonde_u would infer 1.27 off a tie.


def t_only_two_values_are_reachable():
    """Only 0.05 and 0.3175 have no proper divisor in the ladder, and
    occupancy is monotone along a divisibility chain, so no other rung can
    ever win. This is a property of the ladder, not of the corpus."""
    reachable = set()
    for s in BG.GRID_LADDER:
        divisors = [d for d in BG.GRID_LADDER
                    if d < s and abs(s / d - round(s / d)) < 1e-9]
        if not divisors:
            reachable.add(s)
    report('exactly {0.05, 0.3175} have no ladder divisor',
           reachable == {0.05, 0.3175}, str(sorted(reachable)))
    # And the monotonicity the argument rests on, checked on real numbers.
    vals = [1.27 * i + 0.0 for i in range(50)] + [3.1, 7.77, 0.4]
    bad = [(d, s) for s in BG.GRID_LADDER for d in BG.GRID_LADDER
           if d < s and abs(s / d - round(s / d)) < 1e-9
           and BG.occupancy(vals, d) < BG.occupancy(vals, s) - 1e-12]
    report('occupancy is monotone non-increasing along every divisor chain',
           not bad, str(bad[:3]))


def t_a_rate_needs_a_denominator():
    """1-4 part fixtures score 1.00 at six rungs because hand-authored
    coordinates are round by construction. There is no answer to give."""
    ev = BG.infer_board_grid(on_lattice(0.635, 4))
    report('below MIN_PARTS it declines', ev['step'] is None, str(ev['step']))
    report('and names the denominator as the reason',
           'n_parts 4 < 8' in ev['reason'], ev['reason'])
    report('at exactly MIN_PARTS it answers',
           BG.infer_board_grid(on_lattice(0.635, 8))['step'] == 0.3175)


def t_the_floor_does_not_sit_on_a_sample():
    """A threshold resting exactly on a corpus board is decided by `>=` vs `>`.
    interf_u_unrouted scores 0.700000, which is why the floor is 0.67 and not
    the round 0.70 the population gap first suggests."""
    c = corpus()
    if c is None:
        report('floor clearance (SKIPPED: git could not enumerate the corpus)',
               True)
        return
    bests = [(max(ev['profile'].values()), n) for n, ev in c.items()
             if ev['n_parts'] >= BG.MIN_PARTS]
    near = [(b, n) for b, n in bests if abs(b - BG.OCCUPANCY_FLOOR) < 0.02]
    report('no corpus board sits within 0.02 of the floor',
           not near, str(near))
    above = min((b for b, _n in bests if b >= BG.OCCUPANCY_FLOOR), default=None)
    below = max((b for b, _n in bests if b < BG.OCCUPANCY_FLOOR), default=None)
    report('the floor sits inside a real population gap',
           above is not None and below is not None and above - below > 0.05,
           'nearest admitted %.4f, nearest rejected %.4f' % (above, below))


def t_the_corpus_verdicts_are_pinned():
    """The whole table, so a change to any constant has to face every board."""
    expect = {
        # imperial, and the boards #708 is about
        'splitflap_driver': 0.3175, 'flat_hierarchy': 0.3175,
        'sonde_u': 0.3175, 'interf_u_unrouted': 0.3175,
        # metric-fine
        'glasgow_revC': 0.05, 'esp_prog': 0.05,
        'interf_u_unrouted_placed': 0.05, 'lvds_converter_dualclk': 0.05,
        'lvds_converter_dualclk_gnd': 0.05, 'haasoscope_pro_max_test': 0.05,
        'routed_output': 0.05,
        # no lattice: below the floor
        'orangecrab_ext_pll': None, 'tigard': None,
        'rp2350_fpga_eensy_prePlane': None,
        'kit-dev-coldfire-xilinx_5213': None,
        'watchy': None, 'ulx3s': None,
        # no lattice: too few parts
        'cap_chain': None, 'qfn_csi_underpad_diff': None,
        'qfn_diffpair_escape': None, 'qfn_interior_pads': None,
        'qfn_underpad_coupling': None,
    }
    c = corpus()
    if c is None:
        report('corpus verdicts (SKIPPED: git could not enumerate the corpus)',
               True)
        return
    wrong = {n: (c[n]['step'], want) for n, want in expect.items()
             if n in c and c[n]['step'] != want}
    report('every tracked board resolves as recorded', not wrong, str(wrong))
    unlisted = sorted(set(c) - set(expect))
    report('and the table covers the corpus (a new board must be judged)',
           not unlisted, str(unlisted))


def t_the_issue_s_coldfire_claim_is_refuted():
    """#708 cites kit-dev-coldfire-xilinx_5213 as showing "the same signature
    at 1.27mm". It does not, and this is the row that keeps the refutation from
    being re-litigated."""
    c = corpus()
    if c is None:
        report('coldfire (SKIPPED: no corpus)', True)
        return
    ev = c.get('kit-dev-coldfire-xilinx_5213')
    if ev is None:
        report('coldfire is in the corpus', False)
        return
    best = max(ev['profile'].values())
    report('its 1.27 occupancy is far below its own maximum',
           ev['profile'][1.27] < 0.2 and best < 0.3,
           '1.27 -> %.3f, best %.3f' % (ev['profile'][1.27], best))
    report('so it reports NO lattice', ev['step'] is None)
    # The one corpus claim #708 gets right, kept beside the one it does not.
    report('ulx3s and watchy report no lattice, as the issue says',
           c['ulx3s']['step'] is None and c['watchy']['step'] is None)


def t_it_declines_rather_than_inventing():
    report('a board with no footprints declines',
           BG.infer_board_grid(_PCB([]))['reason'] == 'no footprints')
    nonfinite = _PCB([(float('nan'), 1.0)] * 10)
    vals, n_parts = BG.board_coordinates(nonfinite)
    report('non-finite coordinates are dropped from the SAMPLE',
           vals == [1.0] * 10, str(vals[:3]))
    report('while n_parts still counts the footprints that exist',
           n_parts == 10, str(n_parts))
    try:
        BG.infer_board_grid(on_lattice(0.1, 10), ladder=(0.0, 0.1))
        raised = False
    except ValueError:
        raised = True
    report('a non-positive ladder rung raises rather than dividing by zero',
           raised)


def t_resolve_falls_back_and_says_which_branch():
    """There is no flag: the fallback IS the off state, and a caller has to be
    able to tell which branch ran."""
    lat, ev = BG.resolve_snap_lattice(on_lattice(0.635, 20), 0.1)
    report('an inferable board resolves to its lattice',
           lat == 0.3175 and ev['source'] == 'inferred', '%s %s' % (lat, ev['source']))
    lat, ev = BG.resolve_snap_lattice(on_lattice(0.635, 4), 0.1)
    report('a board with no lattice falls back to grid_step',
           lat == 0.1 and ev['source'] == 'grid_step', '%s %s' % (lat, ev['source']))
    report('and the fallback still carries the reason',
           'n_parts' in ev['reason'], ev['reason'])
    lat, ev = BG.resolve_snap_lattice(on_lattice(0.635, 20), 0.1, override=0.5)
    report('an override wins and is labelled as given',
           lat == 0.5 and ev['source'] == 'explicit')
    try:
        BG.resolve_snap_lattice(on_lattice(0.635, 20), 0.1, override=0.0)
        raised = False
    except ValueError:
        raised = True
    report('a non-positive override raises', raised)


def t_describe_names_the_branch_it_took():
    _lat, ev = BG.resolve_snap_lattice(on_lattice(0.635, 20), 0.1)
    line = BG.describe(ev)
    report('the inferred line names the lattice and the evidence',
           '0.3175' in line and 'inferred' in line and '40' in line, line)
    _lat, ev = BG.resolve_snap_lattice(on_lattice(0.635, 4), 0.1)
    line = BG.describe(ev)
    report('the fallback line says it is the raster, and why',
           'raster' in line and 'n_parts' in line, line)


TESTS = [
    ('occupancy', t_occupancy_counts_what_it_says),
    ('finest, not argmax', t_finest_within_slack_not_argmax),
    ('the ladder admits two answers', t_only_two_values_are_reachable),
    ('a rate needs a denominator', t_a_rate_needs_a_denominator),
    ('the floor is off every sample', t_the_floor_does_not_sit_on_a_sample),
    ('corpus verdicts', t_the_corpus_verdicts_are_pinned),
    ("#708's coldfire claim", t_the_issue_s_coldfire_claim_is_refuted),
    ('it declines rather than inventing', t_it_declines_rather_than_inventing),
    ('resolve names its branch', t_resolve_falls_back_and_says_which_branch),
    ('describe', t_describe_names_the_branch_it_took),
]


def _every_case_is_registered():
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
    print('board lattice inference (#708)')
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
