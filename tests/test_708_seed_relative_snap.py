"""#708: the quench moves a part by a whole number of the board's grid units.

Before this, `_candidate_positions` snapped the ABSOLUTE candidate to a lattice
through board origin, which threw away the seed's residue: one pass took
splitflap_driver from 0.923 of its footprint coordinates on its own 0.3175mm
lattice to 0.269. The fix snaps the OFFSET instead, to the board's own lattice
where one can be read off it.

The two theorems below are stated so they need no baseline and no golden file:

  RESIDUE     every reported move is seed + an exact multiple of the lattice.
  CONSERVATION the multiset of poses-modulo-lattice is PRESERVED across a whole
              run, swaps included. A nudge maps on-lattice to on-lattice, and a
              swap permutes poses among same-footprint parts, so the count of
              on-lattice poses cannot change -- not merely "not get worse".
              Today's count falls 120 -> 35 on splitflap, so this is a real
              assertion and not a restatement of the code.

Both are properties of the mechanism rather than of a board, which is why they
are asserted on an imperial board, a metric-fine one, and one with no
inferable lattice at all.
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
    is on that part's lattice offset, not on this one's. Conservation below is
    the statement that covers swaps.
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
    """The count of on-lattice poses is PRESERVED across a whole run.

    Swaps on, which is the case the per-part residue statement cannot make.
    Equality, not `>=`: a nudge maps on-lattice to on-lattice and a swap is a
    permutation, so nothing can add or remove one.
    """
    for name, want in BOARDS:
        lat, _ev = lattice_of(name)
        seeds, moved, _m = run(name)
        before = sum(1 for (x, y) in seeds.values()
                     for v in (x, y) if on_grid(v, lat))
        after = 0
        for r, (sx, sy) in seeds.items():
            x, y = moved.get(r, (sx, sy))
            after += sum(1 for v in (x, y) if on_grid(v, lat))
        report('%s: on-%g count preserved (%d -> %d over %d coords)'
               % (name, lat, before, after, 2 * len(seeds)),
               after == before, 'lost %d' % (before - after))
        if want is not None:
            report('%s: and that count is most of the board, so the '
                   'assertion has something to lose' % name,
                   before >= 0.6 * 2 * len(seeds),
                   '%d/%d' % (before, 2 * len(seeds)))


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
    # The measured claim the fix rests on: the lattice does not cost reach at
    # the step this tool ships.
    n_raster = len(Q._candidate_positions(_P(), 10.0, 1.0, 0.1))
    n_lat = len(Q._candidate_positions(_P(), 10.0, 1.0, 0.3175))
    report('a 0.3175 lattice costs no candidates at step=1.0',
           n_lat >= n_raster, 'raster %d, lattice %d' % (n_raster, n_lat))


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


TESTS = [
    ('residue', t_the_residue_theorem),
    ('conservation', t_conservation_of_the_lattice),
    ('the generator', t_the_generator_offers_only_on_lattice_poses),
    ('no lattice -> the raster', t_a_board_with_no_lattice_keeps_the_raster_granularity),
    ('group offsets probe what they emit', t_group_offsets_probes_the_pose_it_emits),
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
