"""`detect_bga_pitch` returns a PITCH, not the smallest gap on the package.

It used to answer `min()` over the adjacent gaps of the unique x and y pad
coordinates, so ONE anomalous pad pair spoke for the whole footprint. Measured
over corpus sets 1-5 that produced pitches no part has:

    cparti_fpga  U1   256 pads   1.0e-06 mm   (a real 1.0mm array)
    zynq_ad9364  U1   400 pads   2.0e-06 mm   (a real 0.8mm array)
    zynq_ad9364  U2    96 pads   2.0e-06 mm
    watchy       U6    12 pads   1.25e-02 mm  (a real 0.5mm array)

78 of 348 footprints read under 0.05mm. TWO consumers act on the number:

  * `auto_detect_bga_exclusion_zones` / routing_common set
    `edge_tolerance = margin + pitch * 1.1`, which feeds
    `connectivity.is_edge_stub` -- and that compares a pad CENTRE against a box
    drawn at pad EDGES, so a collapsed tolerance can never match. is_edge_stub
    returned False for every pad of those four parts, disabling the outer-row
    test that gates ~10 layer_swap_optimization branches, on the largest arrays
    present. Corpus: 4 such parts before this fix, 0 after.
  * `route_planes._resolve_zone_clearance_impl` takes `pitch - field_via` and
    mins it ACROSS fields, so one bad reading poisons the board -- it returns
    early warning "pour cannot thread the densest BGA lattice even at the fab
    floor" and skips the tightening other fields needed. Its `if not pitch`
    guard misses this: the bad values are ~1e-6, not 0. Recorded corpus:
    quickfeather, whose 10-pad U5 read 0.095mm, produced a NEGATIVE requirement
    (`needs -0.253mm < floor 0.1`).

`diff_pair_routing._field_at` only reports it and says so; it is unaffected.

The median adjacent gap per axis, min of the two axes, is the same number on a
regular array and is not moved by a few odd pads.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def _fn(row):
    """Import it, or fail the row WITH THE REASON -- an ImportError exits the
    same way a caught regression does, and only one of the two is evidence."""
    try:
        from kicad_parser import detect_bga_pitch
        return detect_bga_pitch
    except ImportError as exc:
        check(row, False, f'kicad_parser exports no detect_bga_pitch ({exc})')
        return None


class _Pad:
    def __init__(self, x, y, pad_type='smd'):
        self.global_x, self.global_y, self.pad_type = x, y, pad_type


class _FP:
    def __init__(self, pads):
        self.pads = pads


def _grid(nx, ny, pitch):
    return [_Pad(i * pitch, j * pitch) for i in range(nx) for j in range(ny)]


def _old_min_gap(fp):
    """The behaviour being replaced, so every row below is a change detector
    rather than a restatement of the new code."""
    if not fp.pads or len(fp.pads) < 2:
        return 1.0
    xs = sorted({p.global_x for p in fp.pads})
    ys = sorted({p.global_y for p in fp.pads})
    gaps = ([b - a for a, b in zip(xs, xs[1:])]
            + [b - a for a, b in zip(ys, ys[1:])])
    return min(gaps) if gaps else 1.0


def t_a_regular_array_reads_its_pitch():
    fn = _fn('t_a_regular_array_reads_its_pitch')
    if fn is None:
        return
    for nx, ny, pitch in ((16, 16, 1.0), (20, 20, 0.8), (8, 8, 0.65), (6, 6, 0.4)):
        got = fn(_FP(_grid(nx, ny, pitch)))
        check(f't_grid_{nx}x{ny}_at_{pitch}mm',
              abs(got - pitch) < 1e-9, f'reads {got:.4f}mm')


def t_one_odd_pad_does_not_speak_for_the_package():
    """cparti_fpga's U1 in miniature: a 1.0mm array with one pad 1nm off row."""
    fn = _fn('t_one_odd_pad_does_not_speak_for_the_package')
    if fn is None:
        return
    pads = _grid(16, 16, 1.0) + [_Pad(3.000001, 4.0)]
    got = fn(_FP(pads))
    check('t_one_odd_pad_does_not_speak_for_the_package',
          abs(got - 1.0) < 1e-9, f'reads {got:.6f}mm, not ~1e-6')
    old = _old_min_gap(_FP(pads))
    check('t_the_old_behaviour_really_did_fail_this',
          old < 0.05,
          f'min-gap reads {old:.2e}mm on the same pads -- the negative control, '
          f'so the row above is a change detector')


def t_a_collapsed_pitch_would_kill_is_edge_stub():
    """The CONSEQUENCE, not just the number: at the old pitch no pad of the
    package can satisfy is_edge_stub, because it compares a pad CENTRE against
    a bounding box drawn at pad EDGES."""
    fn = _fn('t_a_collapsed_pitch_would_kill_is_edge_stub')
    if fn is None:
        return
    try:
        from connectivity import is_edge_stub
    except ImportError as exc:
        check('t_a_collapsed_pitch_would_kill_is_edge_stub', False,
              f'could not import is_edge_stub ({exc})')
        return
    pads = _grid(16, 16, 1.0) + [_Pad(3.000001, 4.0)]
    fp = _FP(pads)
    # Pad-EDGE bounding box, as get_footprint_bounds builds it (half a 0.4mm
    # ball beyond the outermost centres).
    half = 0.2
    xs = [q.global_x for q in pads]
    ys = [q.global_y for q in pads]
    box = (min(xs) - half, min(ys) - half, max(xs) + half, max(ys) + half)
    corner = (min(xs), min(ys))
    old_zone = [(*box, _old_min_gap(fp) * 1.1)]
    new_zone = [(*box, fn(fp) * 1.1)]
    check('t_the_old_tolerance_made_every_pad_interior',
          not is_edge_stub(corner[0], corner[1], old_zone),
          'at the min-gap tolerance even the CORNER ball is not an edge stub')
    check('t_the_pitch_tolerance_sees_the_outer_row',
          is_edge_stub(corner[0], corner[1], new_zone),
          'at the real pitch the corner ball is an edge stub again')


def t_undetectable_still_returns_the_documented_default():
    """docs/api-kicad-parser.md states `1.0 if undetectable`; callers rely on a
    sane number (route_planes guards with `if not pitch`), so 0.0 is not an
    option."""
    fn = _fn('t_undetectable_still_returns_the_documented_default')
    if fn is None:
        return
    check('t_one_pad', fn(_FP([_Pad(0, 0)])) == 1.0, 'a single pad -> 1.0')
    check('t_no_pads', fn(_FP([])) == 1.0, 'no pads -> 1.0')
    check('t_all_pads_coincident',
          fn(_FP([_Pad(1.0, 1.0) for _ in range(20)])) == 1.0,
          'twenty pads at one point -> 1.0, never 0.0 (which would read as '
          'falsy AND as an infinitely fine pitch)')


def t_a_coarse_part_is_not_called_fine():
    """A 2.54mm header read 0.006mm under min-gap."""
    fn = _fn('t_a_coarse_part_is_not_called_fine')
    if fn is None:
        return
    header = [_Pad(i * 2.54, j * 2.54) for i in range(10) for j in range(2)]
    header.append(_Pad(2.54 + 0.006, 0.0))   # one plated oddity
    got = fn(_FP(header))
    check('t_a_coarse_part_is_not_called_fine',
          got > 0.8, f'2x10 header on 2.54mm reads {got:.3f}mm '
                     f'(min-gap: {_old_min_gap(_FP(header)):.4f}mm)')


def main():
    t_a_regular_array_reads_its_pitch()
    t_one_odd_pad_does_not_speak_for_the_package()
    t_a_collapsed_pitch_would_kill_is_edge_stub()
    t_undetectable_still_returns_the_documented_default()
    t_a_coarse_part_is_not_called_fine()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
