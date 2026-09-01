"""#826: the portfolio's basin-escape lands on the board's own lattice.

`perturb_jitter` offset a part by `r*cos(theta)` at 4 decimal places, so every
portfolio candidate started off-grid and the quench -- which since #708
preserves whatever lattice it is given -- inherited an arbitrary residue.
Measured before the fix, `tests/measure_826_jitter_lattice.py`: ten of the
eleven tracked boards that declare a lattice no longer declared one after the
default jitter, two of them at 0.000 occupancy.

TWO FIXTURE TRAPS, both measured, both of which make an arm silently vacuous
if ignored:

  * `splitflap_driver` -- the default board of every other test_portfolio*.py
    -- perturbs 29 parts of which ZERO have an off-lattice seed. There
    `snap(part.x + dx) - part.x == snap(dx)` for every emitted pose, so an
    ABSOLUTE-snap mutation is invisible. `interf_u_unrouted` perturbs 22 of 22
    with SEVEN off-lattice, and is the fixture the offset arms use. This is the
    identical trap `tests/test_708_seed_relative_snap.py` records at
    `t_group_offsets_probes_the_pose_it_emits` -- "a probe fixture whose parts
    were already on-lattice".

  * the zero-offset branch NEVER fires at the shipped radius -- measured 0
    rejections on all eleven lattice boards at radius 4.0 and at 2.0, first
    firing at 1.0. No population arm at the default can kill a dropped
    zero-guard, so that arm scripts the rng instead.

The radius arms script the rng for a STRONGER version of the same reason: an
outward-snapping draw is 0-6 per board and is ZERO on six of the eleven,
including `esp_prog` and `routed_output`. An arm depending on one being drawn
would be vacuous on most of the corpus, not merely fragile.

MEASURED, `tests/mutate_826.py`, fourth run: 12 rows, 12 killed, 0
survived, 0 broken. The table, from the run:

    the-jitter-snap-is-dropped                               KILLED   11
    the-jitter-snaps-the-absolute-pose                       KILLED   7
    the-offset-snaps-to-the-raster-not-the-lattice           KILLED   12
    a-zero-offset-counts-as-a-perturbation                   KILLED   3
    the-radius-test-goes-back-before-the-snap                KILLED   2
    the-lattice-falls-back-to-the-grid-step-raster           KILLED   6
    the-disclosure-reports-the-inference-not-what-was-used   KILLED   3
    the-radius-guard-is-dropped                              KILLED   4
    the-generator-does-not-pass-the-lattice                  KILLED   8
    the-lattice-is-resolved-per-candidate-from-the-live-state KILLED   9
    the-default-becomes-the-inferred-lattice                 KILLED   1
    the-disclosure-drops-the-board-grid                      KILLED   3

The battery has caught something on every run, which is the point of it. Run 1
killed 10 of 11; the survivor, `the-disclosure-drops-the-board-grid`, was a
hole in THIS file rather than in the engine -- the end-to-end arm compared the
candidate's `board_grid_step` against the BASELINE's, so a mutation nulling the
whole disclosure made both None and `None == None` passed. Two things that
vanish together are not an agreement; both are now asserted against the input
board's own inferred lattice. Run 3 reported two rows BROKEN because a review's
fixes had rewritten the exact lines they anchored on. The twelfth row,
`the-disclosure-reports-the-inference-not-what-was-used`, pins the blocking
defect that review found, and is itself fixture-blind: it dies only under the
`--step 0.25` arm, because at the default step the inferred and resolved
lattices are equal.
"""

import math
import os
import random
import sys

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import routing_defaults as defaults                             # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from list_nets import board_floor_knobs                         # noqa: E402
from placement import board_grid as BG                          # noqa: E402
from placement import portfolio as PF                           # noqa: E402

FAILURES = []

# The imperial fixture: 22 free, 22 perturbed, SEVEN with off-lattice seeds.
IMPERIAL = 'interf_u_unrouted'
# No inferable lattice (best occupancy 0.369 < floor 0.67).
NO_LATTICE = 'rp2350_fpga_eensy_prePlane'
RADIUS = 4.0


def report(name, ok, detail=''):
    print(('  PASS  ' if ok else '  FAIL  ') + name
          + (('  -- ' + detail) if detail else ''))
    if not ok:
        FAILURES.append(name)


def board(name):
    p = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
    assert os.path.isfile(p) and os.path.getsize(p) > 0, p
    return p


def on_grid(v, step, tol=1e-6):
    return abs(v / step - round(v / step)) * step <= tol


def state_for(name):
    """The oracle `portfolio.generate` would build, through the knob
    resolution `place_portfolio.main` performs first."""
    import contextlib
    import io
    path = board(name)
    pcb = parse_kicad_pcb(path)
    free = PF.free_refs(pcb, path, None)
    clearance, edge, _k = board_floor_knobs(path, None, None)
    with contextlib.redirect_stdout(io.StringIO()):
        oracle = PF.make_oracle(pcb, path, free=free, clearance=clearance,
                                board_edge_clearance=edge,
                                grid_step=defaults.GRID_STEP, ignore_ids=set())
    return pcb, path, free, oracle


class Scripted:
    """An rng whose `.random()` replays a fixed pair, forever.

    `perturb_jitter` draws exactly two values per attempt -- `r` then `theta` --
    so a fixed pair puts every attempt of every ref at the same offset, which
    is what lets an arm test one geometric rule without depending on a draw
    happening to land there.
    """
    def __init__(self, u1, u2):
        self.pair = (u1, u2)
        self.i = 0

    def random(self):
        v = self.pair[self.i % 2]
        self.i += 1
        return v


def u_for_r(r, radius):
    """The draw that makes `perturb_jitter` compute this `r`."""
    return (r / radius) ** 2


# ------------------------------------------------------------------ cases

def t_the_resolver_answers_or_declines_with_a_reason():
    pcb, _p, _f, _o = state_for(IMPERIAL)
    step, ev = PF.jitter_lattice(pcb, RADIUS)
    report('an imperial board resolves its own pitch',
           step == 0.3175 and ev['source'] == 'inferred',
           '%s / %s' % (step, ev['source']))
    report('and the evidence carries what it was read off',
           ev['occupancy'] == 0.700 and ev['n_values'] == 50,
           '%s of %s' % (ev['occupancy'], ev['n_values']))

    pcb2, _p2, _f2, _o2 = state_for(NO_LATTICE)
    step2, ev2 = PF.jitter_lattice(pcb2, RADIUS)
    report('a board with no lattice yields NO SNAP, not a raster fallback',
           step2 is None and ev2['resolved'] is None, str(step2))
    report('and says which test it failed',
           'floor' in (ev2['reason'] or ''), ev2['reason'])
    report('the source names the branch, so an inert run is legible',
           ev2['source'] == 'none', ev2['source'])


def t_a_lattice_coarser_than_the_radius_is_refused():
    """Below the lattice the disc holds no destination but the seed itself.
    Measured: --radius 0.3175 perturbs 22 of 22 on this board, --radius 0.3
    perturbs 0 of 22 after burning 440 draws and every candidate goes barren."""
    pcb, _p, _f, _o = state_for(IMPERIAL)
    step, ev = PF.jitter_lattice(pcb, 0.3)
    report('a lattice wider than the radius is refused', step is None, str(step))
    report('and the reason names the radius, not the floor',
           'radius' in (ev['reason'] or ''), ev['reason'])
    step2, _ev2 = PF.jitter_lattice(pcb, 0.3175)
    report('at exactly the lattice it still answers, because it still works',
           step2 == 0.3175, str(step2))


def t_the_resolver_can_only_ever_return_two_values():
    """A property of the ladder, not of the corpus: only 0.05 and 0.3175 have
    no proper divisor in it, so no coarse lattice can starve the disc."""
    seen = set()
    for name in (IMPERIAL, NO_LATTICE, 'splitflap_driver', 'esp_prog',
                 'glasgow_revC'):
        pcb, _p, _f, _o = state_for(name)
        seen.add(PF.jitter_lattice(pcb, RADIUS)[0])
    report('every resolved value is 0.05, 0.3175 or None',
           seen <= {0.05, 0.3175, None}, str(sorted(seen, key=str)))


def t_the_offset_is_snapped_not_the_position():
    """THE arm, and the one with a precondition that must not go quiet.

    On a board where every perturbed part already sits on the lattice,
    `snap(part.x + dx) - part.x == snap(dx)` and an absolute-snap mutation is
    invisible. Measured: splitflap_driver 0 of 29, interf_u_unrouted 7 of 22.
    """
    pcb, _path, free, oracle = state_for(IMPERIAL)
    lat, _ev = PF.jitter_lattice(pcb, RADIUS)
    origin = {r: (pcb.footprints[r].x, pcb.footprints[r].y) for r in free}
    poses = PF.perturb_jitter(oracle, free, random.Random('0:1:jitter'),
                              RADIUS, lattice=lat)
    off = [p for p in poses
           if not (on_grid(origin[p['reference']][0], lat)
                   and on_grid(origin[p['reference']][1], lat))]
    report('the sample contains parts whose OWN seed is off the lattice, so '
           'offset-snap and absolute-snap are distinguishable here',
           len(off) >= 3,
           '%d of %d perturbed (splitflap_driver gives ZERO and this arm '
           'would go blind there)' % (len(off), len(poses)))
    report('every offset is a whole number of %g' % lat,
           all(on_grid(p['new_x'] - origin[p['reference']][0], lat)
               and on_grid(p['new_y'] - origin[p['reference']][1], lat)
               for p in poses), str(len(poses)))
    report('and an off-lattice seed KEEPS its residue -- the pose is not on a '
           'lattice through board origin',
           all(not (on_grid(p['new_x'], lat) and on_grid(p['new_y'], lat))
               for p in off), str([p['reference'] for p in off][:4]))


def t_the_lattice_is_the_boards_not_the_raster():
    """0.1 is not a multiple of 0.3175, so a raster mutation is visible here.
    It would NOT be on a 0.05 board, where every 0.1-multiple is a
    0.05-multiple -- which is why this arm is imperial."""
    pcb, _path, free, oracle = state_for(IMPERIAL)
    lat, _ev = PF.jitter_lattice(pcb, RADIUS)
    origin = {r: (pcb.footprints[r].x, pcb.footprints[r].y) for r in free}
    poses = PF.perturb_jitter(oracle, free, random.Random('0:1:jitter'),
                              RADIUS, lattice=lat)
    raster = [p for p in poses
              if not on_grid(p['new_x'] - origin[p['reference']][0], 0.1)]
    report('offsets are multiples of the BOARD lattice and not of the raster',
           bool(raster),
           '%d of %d offsets are not 0.1-multiples' % (len(raster), len(poses)))


def t_the_radius_stays_an_exact_bound():
    """Scripted, because an outward-snapping draw is 1-6 per board -- likely,
    not guaranteed, and an arm resting on one goes vacuous when a fixture
    moves. snap(3.99, 0.3175) = 4.1275 > 4.0."""
    pcb, _path, free, oracle = state_for(IMPERIAL)
    lat, _ev = PF.jitter_lattice(pcb, RADIUS)
    from placement.utility import snap_to_grid
    r_out = 3.99
    snapped = snap_to_grid(r_out, lat)
    report('the scripted draw really does snap OUT of the disc',
           snapped > RADIUS, 'snap(%.2f, %g) = %.4f > %.1f'
           % (r_out, lat, snapped, RADIUS))
    # Precondition: at least one ref could legally sit at the snapped-out
    # offset, so a mutation that skipped the radius test would EMIT something.
    legal = [r for r in free
             if oracle.parts.get(r) is not None
             and oracle.candidate_valid(r, oracle.parts[r].x + snapped,
                                        oracle.parts[r].y,
                                        oracle.parts[r].rot)]
    report('at least one ref could legally sit at the snapped-out offset, so '
           'this arm tests the radius rule and not the legality gate',
           bool(legal), '%d refs' % len(legal))
    poses = PF.perturb_jitter(oracle, free, Scripted(u_for_r(r_out, RADIUS),
                                                     0.0),
                              RADIUS, attempts=1, lattice=lat)
    report('no pose is emitted from a draw that snapped past the radius',
           poses == [], str(poses[:1]))


def t_a_draw_that_snaps_to_no_movement_is_refused():
    """Also scripted: measured 0 zero-rejections on every lattice board at
    radius 4.0 and 2.0, so no population arm at the shipped radius can kill a
    dropped zero-guard.

    Non-vacuous by construction: the incumbent pose was just asserted legal by
    the guard above the draw, so if the zero-branch is removed the sampler MUST
    emit exactly one pose per ref. Zero versus non-zero is the kill.
    """
    pcb, _path, free, oracle = state_for(IMPERIAL)
    lat, _ev = PF.jitter_lattice(pcb, RADIUS)
    from placement.utility import snap_to_grid
    r_zero = lat / 4.0
    report('the scripted draw really does snap to no movement',
           snap_to_grid(r_zero, lat) == 0.0,
           'snap(%.4f, %g) = %.4f' % (r_zero, lat, snap_to_grid(r_zero, lat)))
    poses = PF.perturb_jitter(oracle, free, Scripted(u_for_r(r_zero, RADIUS),
                                                     0.0),
                              RADIUS, attempts=1, lattice=lat)
    report('a zero offset is not a perturbation and emits nothing',
           poses == [], '%d pose(s)' % len(poses))


def t_no_emitted_pose_sits_on_its_own_seed():
    """The population form of the arm above, at the radius where the branch
    actually fires (measured: 3 zero-rejections on this board at 0.5)."""
    pcb, _path, free, oracle = state_for(IMPERIAL)
    lat, _ev = PF.jitter_lattice(pcb, 0.5)
    origin = {r: (pcb.footprints[r].x, pcb.footprints[r].y) for r in free}
    poses = PF.perturb_jitter(oracle, free, random.Random('0:1:jitter'), 0.5,
                              lattice=lat)
    report('the run produced poses at all', bool(poses), str(len(poses)))
    report('and not one of them is the seed it started from',
           all((p['new_x'], p['new_y']) != origin[p['reference']]
               for p in poses))


def t_it_is_deterministic():
    pcb, _path, free, oracle = state_for(IMPERIAL)
    lat, _ev = PF.jitter_lattice(pcb, RADIUS)
    a = PF.perturb_jitter(oracle, free, random.Random('t'), RADIUS,
                          lattice=lat)
    _pcb2, _p2, free2, oracle2 = state_for(IMPERIAL)
    b = PF.perturb_jitter(oracle2, free2, random.Random('t'), RADIUS,
                          lattice=lat)
    report('a fixed rng seed gives the same poses', a == b,
           '%d vs %d' % (len(a), len(b)))
    _pcb3, _p3, free3, oracle3 = state_for(IMPERIAL)
    c = PF.perturb_jitter(oracle3, free3, random.Random('u'), RADIUS,
                          lattice=lat)
    report('a different one does not', a != c)


def t_the_seed_board_the_quench_parses_still_declares_the_lattice():
    """The end-to-end form: #826 is about what `_quench_to` hands the quench.

    EXACT equality with the input occupancy, not `>= floor` -- a half-fix that
    stays above 0.67 on the wrong coset would survive the weaker assertion.
    """
    import contextlib
    import io
    import shutil
    import tempfile
    work = tempfile.mkdtemp(prefix='pf826_')
    try:
        path = board(IMPERIAL)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            res = PF.generate(path, work, seed=0, n_candidates=2,
                              strategies=('jitter',), radius=RADIUS,
                              quench_kw=dict(max_displacement=3.0, step=1.0,
                                             grid_step=0.1, clearance=0.2,
                                             board_edge_clearance=0.55,
                                             max_passes=3, verbose=False))
        in_ev = BG.infer_board_grid(parse_kicad_pcb(path))
        report('the input declares 0.3175', in_ev['step'] == 0.3175)
        report('the run says which branch the jitter took',
               (res.get('jitter_lattice') or {}).get('source') == 'inferred',
               str((res.get('jitter_lattice') or {}).get('source')))
        cands = res['candidates']
        report('a jitter candidate was produced', bool(cands) and cands[0].board,
               str(cands[0].note if cands else 'none'))
        if not cands or not cands[0].seed_board:
            return
        seed_ev = BG.infer_board_grid(parse_kicad_pcb(cands[0].seed_board))
        report('the PERTURBED SEED BOARD -- what the quench actually parses -- '
               'still declares it', seed_ev['step'] == in_ev['step'],
               str(seed_ev['reason']))
        report('at EXACTLY the input occupancy, not merely above the floor',
               seed_ev['occupancy'] == in_ev['occupancy'],
               '%s vs %s' % (seed_ev['occupancy'], in_ev['occupancy']))
        # Against the INPUT's lattice, not merely against each other. The
        # first draft compared candidate to baseline, and the mutation that
        # nulls the whole disclosure made BOTH None -- so `None == None`
        # passed and the row survived. Two things that vanish together are not
        # an agreement.
        cand_step = cands[0].metrics.get('board_grid_step')
        base_step = res['baseline'].metrics.get('board_grid_step')
        report('the disclosure carries a real lattice, not a null',
               cand_step == in_ev['step'] and base_step == in_ev['step'],
               'candidate %s / baseline %s / input %s'
               % (cand_step, base_step, in_ev['step']))
        report('the candidate reports the occupancy it was read off',
               cands[0].metrics.get('board_grid_reason') == 'inferred',
               str(cands[0].metrics.get('board_grid_reason')))
        # THE FINAL BOARD, not just the seed. The stated purpose is "so the
        # quench that follows can preserve a lattice", and that property
        # belongs to the board the quench WRITES. Checking only the seed board
        # is what let a disclosure defect through review: at `--step 0.25` the
        # seed declares 0.3175 and the final board declares nothing.
        final_ev = BG.infer_board_grid(parse_kicad_pcb(cands[0].board))
        report('the QUENCHED candidate board still declares the lattice',
               final_ev['step'] == in_ev['step'], str(final_ev['reason']))
        report('at the input occupancy',
               final_ev['occupancy'] == in_ev['occupancy'],
               '%s vs %s' % (final_ev['occupancy'], in_ev['occupancy']))


    finally:
        shutil.rmtree(work, ignore_errors=True)


def t_the_disclosure_reports_what_was_used_not_what_was_inferred():
    """`quench` infers a lattice and can then FALL BACK -- when it is coarser
    than `--step` it keeps `step` at the inference and puts what it used in
    `resolved`. A disclosure that reads `step` reports a lattice the quench
    never used, and would say the fix worked on a run where it shipped inert.

    Measured at `--step 0.25` on this imperial board: the candidate the run
    ships has NO lattice and 0.120 occupancy, while `step` still reads 0.3175.
    """
    import contextlib
    import io
    import shutil
    import tempfile
    work = tempfile.mkdtemp(prefix='pf826d_')
    try:
        path = board(IMPERIAL)
        with contextlib.redirect_stdout(io.StringIO()):
            res = PF.generate(path, work, seed=0, n_candidates=2,
                              strategies=('jitter',), radius=RADIUS,
                              quench_kw=dict(max_displacement=3.0, step=0.25,
                                             grid_step=0.1, clearance=0.2,
                                             board_edge_clearance=0.55,
                                             max_passes=3, verbose=False))
        c = res['candidates'][0]
        if not c.board:
            report('the --step 0.25 arm produced a candidate', False, c.note)
            return
        report('the quench fell back, so this arm tests the right thing',
               c.metrics.get('board_grid_inferred') == 0.3175
               and c.metrics.get('board_grid_step') != 0.3175,
               'inferred %s, used %s' % (c.metrics.get('board_grid_inferred'),
                                         c.metrics.get('board_grid_step')))
        # `board_grid_step` is the lattice the quench's OFFSETS were multiples
        # of -- here the 0.1 raster it fell back to. That is NOT the same
        # quantity as what the resulting BOARD declares (None: the shipped
        # candidate's occupancy is 0.120, under the floor), and the first
        # version of this arm compared the two and failed for exactly that
        # reason.
        report('the disclosed lattice is the raster the quench actually used',
               c.metrics.get('board_grid_step') == 0.1,
               'disclosed %s' % c.metrics.get('board_grid_step'))
        # And the point of the arm: reading `step` instead of `resolved` would
        # have reported 0.3175 for a candidate that shipped with no lattice.
        declares = BG.infer_board_grid(parse_kicad_pcb(c.board))['step']
        report('the shipped board declares no lattice, so the INFERRED value '
               'would have claimed a success that did not happen',
               declares is None,
               'board declares %s, inference said %s'
               % (declares, c.metrics.get('board_grid_inferred')))
    finally:
        shutil.rmtree(work, ignore_errors=True)


def t_the_run_level_disclosure_is_honest_in_every_branch():
    """`jitter_lattice`'s dict is written verbatim into portfolio.json, so
    `step` there has to mean what was USED. The first version left it at the
    inference on the radius-guard branch, shipping
    `{"step": 0.3175, "resolved": null}`."""
    pcb, _p, _f, _o = state_for(IMPERIAL)
    step, ev = PF.jitter_lattice(pcb, 0.3)          # radius guard fires
    report('a guarded run reports no lattice in the terse form too',
           step is None and ev['step'] is None and ev['resolved'] is None,
           'step=%s resolved=%s' % (ev['step'], ev['resolved']))
    report('and keeps the inference beside it, so the reason is readable',
           ev['inferred_step'] == 0.3175, str(ev.get('inferred_step')))
    step2, ev2 = PF.jitter_lattice(pcb, RADIUS)     # ordinary branch
    report('an ordinary run agrees with itself',
           step2 == ev2['step'] == ev2['resolved'] == 0.3175,
           '%s / %s / %s' % (step2, ev2['step'], ev2['resolved']))


TESTS = [
    ('the resolver', t_the_resolver_answers_or_declines_with_a_reason),
    ('the radius guard', t_a_lattice_coarser_than_the_radius_is_refused),
    ('the ladder admits two answers',
     t_the_resolver_can_only_ever_return_two_values),
    ('the OFFSET is snapped', t_the_offset_is_snapped_not_the_position),
    ("the board's lattice, not the raster", t_the_lattice_is_the_boards_not_the_raster),
    ('the radius stays exact', t_the_radius_stays_an_exact_bound),
    ('a zero offset is refused', t_a_draw_that_snaps_to_no_movement_is_refused),
    ('no pose sits on its seed', t_no_emitted_pose_sits_on_its_own_seed),
    ('determinism', t_it_is_deterministic),
    ('end to end', t_the_seed_board_the_quench_parses_still_declares_the_lattice),
    ('the disclosure reports what was USED',
     t_the_disclosure_reports_what_was_used_not_what_was_inferred),
    ('the run-level disclosure is honest',
     t_the_run_level_disclosure_is_honest_in_every_branch),
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
    print('portfolio jitter lands on the board lattice (#826)')
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
