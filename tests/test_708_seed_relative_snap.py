"""#708: the quench moves a part by a whole number of the board's grid units.

Before this, `_candidate_positions` snapped the ABSOLUTE candidate to a lattice
through board origin, which threw away the seed's residue: one pass took
splitflap_driver from 0.923 of its footprint coordinates on its own 0.3175mm
lattice to 0.269. The fix snaps the OFFSET instead, to the board's own lattice
where one can be read off it.

The two theorems below are stated so they need no baseline and no golden file:

  RESIDUE     every reported move is seed + an exact multiple of the lattice.
  CONSERVATION the count of on-lattice poses is PRESERVED across a run with
              nudges and group moves, because each maps a part inside its own
              seed coset. Today's count falls 120 -> 35 on splitflap, so this
              is a real assertion and not a restatement of the code.

              NOT with swaps, and the first draft of this file claimed
              otherwise. A swap hands part A the pose of part B, but A's
              candidates are generated from A's OWN seed, so if A's seed is off
              the lattice the next accepted nudge takes it back off and the
              count moves. `t_a_swap_can_move_the_on_lattice_count` pins the
              counterexample (glasgow_revC C3/C9). With swaps on, the invariant
              that IS true is the weaker one asserted here: every pose still
              lies on some part's seed coset.

Both are properties of the mechanism rather than of a board, which is why they
are asserted on an imperial board, a metric-fine one, and one with no
inferable lattice at all.

MEASURED, `tests/mutate_708.py`, third run: 15 rows, 14 killed, 1 survived
(expected), 0 broken. The table, from the run:

    the-tie-break-takes-the-argmax                  SURVIVED  (expected)
    the-tie-break-takes-the-coarsest                KILLED   6
    the-min-parts-gate-is-dropped                   KILLED   7
    the-occupancy-floor-is-dropped                  KILLED   4
    the-floor-returns-to-the-round-0.70             KILLED   2
    the-tolerance-becomes-relative-to-the-step      KILLED   3
    the-sample-drops-one-axis                       KILLED   3
    the-candidate-snap-goes-back-to-absolute        KILLED  10
    the-offset-snaps-to-the-raster-not-the-lattice  KILLED   5
    the-radius-test-goes-back-before-the-snap       KILLED   2
    the-lattice-is-never-resolved                   KILLED   5
    the-group-probe-goes-back-to-the-absolute-pose  KILLED   2
    the-group-offset-snaps-the-absolute-pose        KILLED   3
    the-fanout-snap-goes-back-to-absolute           KILLED   4
    the-reseat-slot-snaps-the-absolute-point        KILLED   3

The first run killed only 9 of 15. Five of its six survivors were holes in
THESE arms rather than in the engine: an assertion at `step=1.0` where the
mutated factor is a no-op, a lattice sweep that never rounded UP past the cap,
a probe fixture whose parts were already on-lattice, and two sites whose owning
suites never look at phase. The sixth, `the-tie-break-takes-the-argmax`, is an
EQUIVALENT MUTANT and is recorded as an expected survivor with its reason. A
seventh row (`the-group-offset-snaps-the-absolute-pose`) was killed in run 1
and only surfaced as a hole in run 2, once the probe fixture moved to a board
whose lattice is the raster. The rows and the reasons are in `mutate_708.py`.
"""

import math
import os
import sys

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 1200

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import parse_kicad_pcb                        # noqa: E402
from placement import board_grid as BG                          # noqa: E402
from placement import fanout_clearance as FC                    # noqa: E402
from placement import quench as Q                               # noqa: E402
from placement import reseat as RS                              # noqa: E402
from placement.utility import snap_to_grid as snap              # noqa: E402

FAILURES = []

# One imperial, one metric-fine, one with no inferable lattice. Small enough
# to quench in seconds; the point is the mechanism, not the board.
BOARDS = (('interf_u_unrouted', 0.3175),
          ('esp_prog', 0.05),
          ('rp2350_fpga_eensy_prePlane', None))

QUENCH_KW = dict(max_displacement=3.0, step=1.0, grid_step=0.1, clearance=0.2,
                 board_edge_clearance=0.55, max_passes=3, verbose=False)


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


_RUNS = {}


def run(name, **over):
    key = (name, tuple(sorted(over.items())))
    if key in _RUNS:
        return _RUNS[key]
    path = board(name)
    pcb = parse_kicad_pcb(path)
    seeds = {r: (f.x, f.y) for r, f in pcb.footprints.items()}
    kw = dict(QUENCH_KW)
    kw.update(over)
    metrics = {}
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        placements = Q.quench(pcb, path, metrics_out=metrics, **kw)
    moved = {p['reference']: (p['new_x'], p['new_y'])
             for p in (placements or [])}
    _RUNS[key] = (seeds, moved, metrics)
    return _RUNS[key]


def lattice_of(name):
    _lat, ev = BG.resolve_snap_lattice(parse_kicad_pcb(board(name)),
                                       QUENCH_KW['grid_step'])
    return _lat, ev


# ------------------------------------------------------------------ cases

def t_the_residue_theorem():
    """Every reported move is the seed plus an exact multiple of the lattice.

    Nudges and group moves only: a swap hands a part ANOTHER part's pose, which
    is on that part's coset, not on this one's. `t_conservation_of_the_lattice`
    states what survives with swaps on, and
    `t_a_swap_can_move_the_on_lattice_count` pins why it is weaker.
    """
    for name, want in BOARDS:
        lat, ev = lattice_of(name)
        if want is not None:
            report('%s resolves the lattice this case is about' % name,
                   ev['step'] == want, str(ev['step']))
        seeds, moved, _m = run(name, allow_swaps=False)
        bad = [(r, round(x - seeds[r][0], 6), round(y - seeds[r][1], 6))
               for r, (x, y) in moved.items()
               if not (on_grid(x - seeds[r][0], lat)
                       and on_grid(y - seeds[r][1], lat))]
        report('%s: every nudge is a whole number of %g mm (%d moves)'
               % (name, lat, len(moved)), not bad, str(bad[:3]))
        report('%s: the run actually moved something' % name, bool(moved),
               '%d moves' % len(moved))


def t_conservation_of_the_lattice():
    """The count of on-lattice poses is preserved -- WITHOUT swaps.

    The first draft of this case asserted equality with swaps ON, reasoning
    that a swap merely permutes poses. That reasoning is wrong, and a review
    produced the counterexample: `_candidate_positions` generates from
    `part.seed_x`, so a part's residue is a property of its SEED, not of where
    it currently sits. A swap hands part A the pose of part B; if A's seed is
    off-lattice, every candidate A is offered afterwards is off-lattice, and
    the next accepted nudge moves the count. On `glasgow_revC` there are five
    same-footprint pairs inside the 3mm swap cap with one member on-lattice and
    one off (C3 <-> C9, 1.985mm apart, is the smallest).

    The equality held on the three boards here only by accident of board
    choice: `interf_u_unrouted` has ZERO same-footprint pairs within the swap
    cap, so its swap phase never fires at all. So the arm is stated where it is
    actually a theorem -- nudges and group moves, which map each part inside
    its own seed coset -- and the swaps-on run gets the weaker statement that
    IS true of it.
    """
    for name, want in BOARDS:
        lat, _ev = lattice_of(name)
        seeds, moved, _m = run(name, allow_swaps=False)
        before = sum(1 for (x, y) in seeds.values()
                     for v in (x, y) if on_grid(v, lat))
        after = 0
        for r, (sx, sy) in seeds.items():
            x, y = moved.get(r, (sx, sy))
            after += sum(1 for v in (x, y) if on_grid(v, lat))
        report('%s: on-%g count preserved with no swaps (%d -> %d over %d '
               'coords)' % (name, lat, before, after, 2 * len(seeds)),
               after == before, 'lost %d' % (before - after))
        if want is not None:
            report('%s: and that count is most of the board, so the '
                   'assertion has something to lose' % name,
                   before >= 0.6 * 2 * len(seeds),
                   '%d/%d' % (before, 2 * len(seeds)))

        # Swaps ON: the true invariant is that every pose still lies on SOME
        # part's seed coset -- no pose is invented off every lattice the board
        # offers. That is what the fix guarantees and what a return to the
        # absolute snap would break.
        _s2, moved2, _m2 = run(name)
        cosets = {round((sx / lat) % 1.0, 6) for sx, _sy in seeds.values()}
        cosets |= {round((sy / lat) % 1.0, 6) for _sx, sy in seeds.values()}
        stray = [(r, x, y) for r, (x, y) in moved2.items()
                 if round((x / lat) % 1.0, 6) not in cosets
                 or round((y / lat) % 1.0, 6) not in cosets]
        report('%s: with swaps, every pose is still on some seed coset of %g'
               % (name, lat), not stray, str(stray[:2]))


def t_a_swap_can_move_the_on_lattice_count():
    """The counterexample, pinned so the weaker statement above is not read as
    a retreat from a theorem that was true.

    glasgow_revC C3 and C9 are the same footprint, 1.985mm apart (inside the
    default swap cap), C3 on the board's 0.05 lattice and C9 off it. Whichever
    one ends up being nudged afterwards is anchored to ITS OWN seed, so the
    count moves. This is a property of the engine, not of the fix -- it was
    equally true before -- and it is the reason conservation is asserted
    without swaps.
    """
    name = 'glasgow_revC'
    lat, ev = lattice_of(name)
    report('%s resolves the 0.05 lattice' % name, ev['step'] == 0.05,
           str(ev['step']))
    pcb = parse_kicad_pcb(board(name))
    fps = pcb.footprints
    a, b = fps.get('C3'), fps.get('C9')
    report('the pair exists and shares a footprint',
           a is not None and b is not None
           and a.footprint_name == b.footprint_name,
           '%s / %s' % (getattr(a, 'footprint_name', None),
                        getattr(b, 'footprint_name', None)))
    if a is None or b is None:
        return
    d = math.hypot(a.x - b.x, a.y - b.y)
    report('and sits inside the default swap cap', d <= 3.0,
           '%.3f mm apart' % d)
    on_a = on_grid(a.x, lat) and on_grid(a.y, lat)
    on_b = on_grid(b.x, lat) and on_grid(b.y, lat)
    report('with exactly one of the two on the lattice', on_a != on_b,
           'C3 on=%s, C9 on=%s' % (on_a, on_b))


def t_the_generator_offers_only_on_lattice_poses():
    """Straight at `_candidate_positions`, so a failure localises."""
    class _P:
        seed_x, seed_y = 131.4450, 138.4300      # 207 x 0.635, 218 x 0.635
    # 0.635 is in this list for the radius arm, not the lattice arm: it is the
    # only rung here where snapping ROUNDS UP past a 3.0mm cap
    # (snap(3.0, 0.635) = 3.175). At 0.1, 0.05 and 0.3175 the snap always
    # rounds down, so a version that tested the radius BEFORE snapping would
    # look identical -- which is exactly how the first draft of this case let
    # the mutation battery's `radius-test-goes-back-before-the-snap` row
    # survive.
    for lat in (0.1, 0.3175, 0.05, 0.635):
        cands = Q._candidate_positions(_P(), 3.0, 1.0, lat)
        off = [(cx - _P.seed_x, cy - _P.seed_y) for cx, cy in cands]
        report('offsets are multiples of %g' % lat,
               all(on_grid(dx, lat) and on_grid(dy, lat) for dx, dy in off))
        report('and every candidate is INSIDE max_disp, not a snap past it '
               '(lattice %g)' % lat,
               all(math.hypot(dx, dy) <= 3.0 + 1e-9 for dx, dy in off),
               'max %.4f' % max(math.hypot(dx, dy) for dx, dy in off))
    # Reach, stated as the bounded claim it is rather than the absolute one
    # the first draft made. At the shipped default the lattice gains
    # candidates; across the max_disp range it sometimes loses a few, because
    # the exact cap rejects an offset that snapped UP past it.
    n_raster = len(Q._candidate_positions(_P(), 10.0, 1.0, 0.1))
    n_lat = len(Q._candidate_positions(_P(), 10.0, 1.0, 0.3175))
    report('at the shipped default a 0.3175 lattice costs no candidates',
           n_lat >= n_raster, 'raster %d, lattice %d' % (n_raster, n_lat))
    worst = 1.0
    for i in range(1, 40):
        md = i * 0.5
        a = len(Q._candidate_positions(_P(), md, 1.0, 0.1))
        b = len(Q._candidate_positions(_P(), md, 1.0, 0.3175))
        worst = min(worst, b / a if a else 1.0)
    report('and never loses more than 20% of them anywhere in max_disp '
           '0.5..19.5', worst >= 0.80, 'worst ratio %.3f' % worst)


def t_a_board_with_no_lattice_keeps_the_raster_granularity():
    """The fallback IS the off state, so it must be the raster exactly."""
    name = 'rp2350_fpga_eensy_prePlane'
    lat, ev = lattice_of(name)
    report('%s reports no lattice' % name, ev['step'] is None, str(ev['step']))
    report('and resolves to the grid_step raster', lat == QUENCH_KW['grid_step'],
           str(lat))

    class _P:
        seed_x, seed_y = 131.4450, 138.4300
    off = [(round(cx - _P.seed_x, 9), round(cy - _P.seed_y, 9))
           for cx, cy in Q._candidate_positions(_P(), 3.0, 1.0, lat)]
    want = sorted((round(ix * 1.0, 9), round(iy * 1.0, 9))
                  for ix in range(-3, 4) for iy in range(-3, 4)
                  if math.hypot(ix, iy) <= 3.0 + 1e-9)
    report('and the offsets are exactly the unsnapped step grid',
           sorted(off) == want, '%d vs %d' % (len(off), len(want)))


def t_group_offsets_probes_the_pose_it_emits():
    """The admissibility probe used to snap the ABSOLUTE p.x + dx while the
    emitted offset was snap(dx); the cap was checked against a pose the block
    never takes. Assert on what is yielded."""
    # A board whose parts sit OFF the lattice, deliberately. On an on-lattice
    # board `snap(p.x + sdx)` and `p.x + sdx` are the same number, so the
    # probe/emit distinction is invisible and the mutation row for it survives
    # -- which is what happened to the first draft, on interf_u.
    name = 'rp2350_fpga_eensy_prePlane'
    path = board(name)
    pcb = parse_kicad_pcb(path)
    lat, _ev = lattice_of(name)
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        st = Q.QuenchState(pcb, path, 0.2, 0.55, 10.0, 0.5, 0.25, 2.0, 2.0,
                           2.0, 0.1, 1.0)
    refs = sorted(st.parts)[:3]
    off_lattice = [r for r in refs if not on_grid(st.parts[r].x, lat)]
    report('the fixture is off-lattice, so probe and emit can disagree',
           bool(off_lattice), '%d of %d' % (len(off_lattice), len(refs)))
    max_disp, step = 3.0, 1.0
    offs = sorted(Q._group_offsets(st, refs, max_disp, step, lat))
    report('the block move offers offsets at all', bool(offs), str(len(offs)))
    # In a FRESH state every part is at its own seed, so the per-member cap
    # reduces to a statement about the offset alone. Assert the whole SET, not
    # just that what came out is valid: a probe measuring the wrong pose drops
    # offsets near the cap, and "everything emitted is legal" cannot see a
    # missing one.
    n = int(max_disp / step)
    want = sorted({(round(snap(ix * step, lat), 9),
                    round(snap(iy * step, lat), 9))
                   for ix in range(-n, n + 1) for iy in range(-n, n + 1)
                   if math.hypot(snap(ix * step, lat),
                                 snap(iy * step, lat)) <= max_disp + 1e-9}
                  - {(0.0, 0.0)})
    got = sorted((round(dx, 9), round(dy, 9)) for dx, dy in offs)
    report('the emitted offsets are EXACTLY the lattice multiples in range',
           got == want, 'got %d, want %d, missing %s'
           % (len(got), len(want), sorted(set(want) - set(got))[:3]))
    bad = [(r, o) for o in offs for r in refs
           if math.hypot(st.parts[r].x + o[0] - st.parts[r].seed_x,
                         st.parts[r].y + o[1] - st.parts[r].seed_y)
           > max_disp + 1e-9]
    report('every EMITTED offset keeps every member inside its own seed cap',
           not bad, str(bad[:2]))
    report('and every offset is a whole number of the lattice',
           all(on_grid(dx, lat) and on_grid(dy, lat) for dx, dy in offs))

    # The board above has NO inferable lattice, so its lattice IS the 0.1
    # raster and "snapped to the lattice" and "snapped to the raster" are the
    # same statement there. A second fixture on a real imperial pitch is what
    # makes the block move's lattice assertion able to fail -- without it the
    # battery's `group-offset-snaps-the-absolute-pose` row survives.
    name2 = 'interf_u_unrouted'
    path2 = board(name2)
    lat2, _ev2 = lattice_of(name2)
    with contextlib.redirect_stdout(io.StringIO()):
        st2 = Q.QuenchState(parse_kicad_pcb(path2), path2, 0.2, 0.55, 10.0,
                            0.5, 0.25, 2.0, 2.0, 2.0, 0.1, 1.0)
    refs2 = sorted(st2.parts)[:3]
    offs2 = list(Q._group_offsets(st2, refs2, max_disp, step, lat2))
    report('a block on an imperial board moves by whole lattice units, not '
           'raster units',
           bool(offs2) and all(on_grid(dx, lat2) and on_grid(dy, lat2)
                               for dx, dy in offs2),
           '%s lattice %g, %d offsets' % (name2, lat2, len(offs2)))
    report('and those offsets are NOT all raster multiples, so the two '
           'statements are distinguishable',
           any(not on_grid(dx, 0.1) for dx, _dy in offs2),
           str([round(d, 4) for d, _ in offs2[:3]]))


def t_the_run_discloses_which_branch_it_took():
    for name, want in BOARDS:
        _s, _mv, metrics = run(name)
        bg = metrics.get('board_grid')
        if not bg:
            report('%s: metrics_out carries board_grid' % name, False)
            continue
        report('%s: metrics name the source' % name,
               bg.get('source') in ('inferred', 'grid_step', 'explicit'),
               str(bg.get('source')))
        report('%s: and the lattice actually used' % name,
               bg.get('resolved') == lattice_of(name)[0],
               str(bg.get('resolved')))
        if want is None:
            report('%s: a declining board still says why' % name,
                   bool(bg.get('reason')), str(bg.get('reason')))


def t_it_is_deterministic():
    """A dict/set iteration in the inference would make the lattice
    seed-dependent, which `test_457_determinism` would only catch as a
    mysterious intermittent."""
    name = 'interf_u_unrouted'
    _RUNS.clear()
    a = run(name)[1]
    _RUNS.clear()
    b = run(name)[1]
    report('two identical runs agree exactly', a == b,
           '%d vs %d moves' % (len(a), len(b)))


def t_the_two_repair_sites_are_seed_relative_but_not_coarsened():
    """`fanout_clearance` and `reseat` take half the fix, on purpose.

    They get the seed-relative half -- their offsets are multiples of the
    RASTER, measured from the seed rather than from board origin -- and NOT the
    board's lattice, because both searches are sub-millimetre clearance repair
    and coarsening them starves the one search that must not be starved
    (measured: 81 candidates -> 29 at 0.3175, -> 9 at 0.635, at fanout's
    step=0.2).

    Asserted here rather than left to the owning suites: neither
    `test_fanout_clearance` nor `test_reseat` looks at phase at all, so the
    mutation rows for these two sites both survived until this case existed.
    """
    grid = 0.1

    class _Cap:
        seed_x, seed_y = 131.4450, 138.4300      # deliberately OFF the raster
    off = [(cx - _Cap.seed_x, cy - _Cap.seed_y)
           for cx, cy in FC._candidate_positions(_Cap(), 1.0, 0.2, grid)]
    report('fanout: candidate offsets are multiples of the raster',
           all(on_grid(dx, grid) and on_grid(dy, grid) for dx, dy in off),
           str([(round(a, 4), round(b, 4)) for a, b in off[:2]]))
    report('fanout: and they are measured from the cap SEED, which is '
           'off-raster, so the residue survives',
           any(not on_grid(_Cap.seed_x + dx, grid) for dx, _dy in off))
    report('fanout: the lattice is NOT applied there', len(off) == 81,
           '%d candidates (81 = the raster set at step=0.2)' % len(off))

    # reseat: every slot must sit on the ANCHOR's phase, not on a lattice
    # through board origin. splitflap's B0 is at 171.57, which is not a
    # multiple of 0.1, so the two differ.
    name = 'splitflap_driver'
    path = board(name)
    pcb = parse_kicad_pcb(path)
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        st = Q.QuenchState(pcb, path, 0.2, 0.55, 10.0, 0.5, 0.25, 2.0, 2.0,
                           2.0, grid, 1.0)
        clusters = RS.clusters_from_tethers(pcb, st, 2.0)
    cl = [c for c in clusters if not on_grid(st.parts[c.anchor].x, grid)]
    report('reseat: the fixture has an off-raster anchor', bool(cl),
           str([c.anchor for c in clusters][:4]))
    if cl:
        c = cl[0]
        ax, ay = st.parts[c.anchor].x, st.parts[c.anchor].y
        with contextlib.redirect_stdout(io.StringIO()):
            slots = RS.slot_pool(st, c, grid_step=grid)
        # The trailing entries are the members' OWN poses, appended unsnapped
        # so the assignment can always return "leave it alone" -- `slot_pool`
        # says so, and a member's pose need not be on any lattice. They are
        # not generated slots and are excluded here rather than silently
        # weakening the assertion to `most`.
        ring = slots[:-len(c.members)]
        identity = slots[-len(c.members):]
        report('reseat: every GENERATED slot is the anchor pose plus a raster '
               'multiple',
               all(on_grid(x - ax, grid) and on_grid(y - ay, grid)
                   for x, y, _r in ring),
               '%s at (%.4f, %.4f), %d ring slots' % (c.anchor, ax, ay,
                                                      len(ring)))
        report('reseat: and NOT on a lattice through board origin',
               any(not on_grid(x, grid) for x, _y, _r in ring))
        report('reseat: the identity poses are the members\' own, untouched',
               identity == [(st.parts[m].x, st.parts[m].y, st.parts[m].rot)
                            for m in c.members], str(identity[:1]))


def t_the_reseat_accepts_on_the_seated_cost_not_the_prediction():
    """The reseat's prediction is blind to a member's UNSCORED nets.

    `reseat_cluster` overrides the members' pads on the SCORED nets, but
    `other_aw` comes from the live board, so a member's other nets are priced
    at the pose it is leaving. On `ulx3s`'s `tether:B0`, C60 carries net 1
    (`GND`, unscored) and net 278 (`PWRBTn`, scored): the prediction said
    1214.28 and the seated board measures 1274.28. That is 60.0 at the
    fixture's `crossing_penalty=30.0` -- two crossings -- and the cluster used
    to be ACCEPTED on the number that did not describe it.

    This was pre-existing; the #708 slot change only moved C60 to the pose that
    exposes it. Kept here rather than in `test_reseat.py` because that suite
    asserts the property (`never accepts a worse cluster cost`) while this
    asserts the MECHANISM, including the dry-run contract, which nothing else
    exercises.
    """
    import test_reseat as TR
    from placement import reseat as R
    pcb, st = TR._state()
    clusters = TR._clusters(pcb, st, limit=3)
    target = [c for c in clusters if c.name == 'tether:B0']
    report('the ulx3s fixture still has tether:B0', bool(target),
           str([c.name for c in clusters]))
    if not target:
        return
    c = target[0]
    before_state = {r: (p.x, p.y, p.rot) for r, p in st.parts.items()}
    res = R.reseat_cluster(st, c, apply=False)
    report('a cluster whose seated cost is worse is REFUSED',
           not res.accepted, 'accepted=%s' % res.accepted)
    report('and the refusal names the mechanism rather than a bare verdict',
           'does not score' in (res.reason or ''), (res.reason or '')[:70])
    report('the reported `after` is the SEATED cost, not the prediction',
           abs(res.after - 1274.2793) < 0.01, '%.4f' % res.after)
    report('apply=False leaves every part exactly where it was',
           all((st.parts[r].x, st.parts[r].y, st.parts[r].rot) == v
               for r, v in before_state.items()))
    # A cluster that IS an improvement must still be accepted -- otherwise this
    # guard would read as "working" while refusing everything.
    accepted = [R.reseat_cluster(st, k, apply=False).accepted
                for k in clusters]
    report('and a genuinely improving cluster is still accepted',
           any(accepted), str(accepted))


TESTS = [
    ('residue', t_the_residue_theorem),
    ('conservation', t_conservation_of_the_lattice),
    ('a swap can move the count', t_a_swap_can_move_the_on_lattice_count),
    ('the generator', t_the_generator_offers_only_on_lattice_poses),
    ('no lattice -> the raster', t_a_board_with_no_lattice_keeps_the_raster_granularity),
    ('group offsets probe what they emit', t_group_offsets_probes_the_pose_it_emits),
    ('reseat accepts on the seated cost',
     t_the_reseat_accepts_on_the_seated_cost_not_the_prediction),
    ('the repair sites take half the fix',
     t_the_two_repair_sites_are_seed_relative_but_not_coarsened),
    ('disclosure', t_the_run_discloses_which_branch_it_took),
    ('determinism', t_it_is_deterministic),
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
    print('seed-relative candidate snap (#708)')
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
