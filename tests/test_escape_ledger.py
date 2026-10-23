#!/usr/bin/env python3
"""Per-face escape lanes: supply, demand, and who ate the lanes.

The ledger exists to separate *"the router failed"* from *"this was never
routable"*. A face whose demand exceeds its supply is a binding constraint --
net ordering only chooses WHICH nets strand there, never how many -- and whole
runs have been spent on ordering experiments against a face a ledger would have
settled in seconds.

Most of these use a hand-built part whose answer is arithmetic (a 10mm face at
0.45mm lanes supplies 22), so a failure points at the ledger rather than at a
board. The last two run it on real boards, because a signal that only works on
a fixture is not a signal.
"""
import io
import contextlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

from placement import escape                                   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class _Pad:
    def __init__(self, lx, ly, gx, gy, net_id, drill=0.0):
        self.local_x, self.local_y = lx, ly
        self.global_x, self.global_y = gx, gy
        self.net_id, self.drill = net_id, drill


class _Fp:
    def __init__(self, name, pads):
        self.footprint_name, self.pads = name, pads


class _Pcb:
    def __init__(self, fps):
        self.footprints = fps
        self.source_path = None


def _grid_part(name='U1:QFN-64', n=8, pitch=0.5, x0=10.0, y0=10.0,
               net_base=1, interior=False):
    """An n x n perimeter part at `pitch`, optionally with an interior pad."""
    pads = []
    nid = net_base
    for i in range(n):
        for j in range(n):
            edge = i in (0, n - 1) or j in (0, n - 1)
            if not edge and not interior:
                continue
            lx, ly = j * pitch, i * pitch
            pads.append(_Pad(lx, ly, x0 + lx, y0 + ly, nid))
            nid += 1
    return _Fp(name, pads)


def test_supply_is_face_length_over_lane_pitch():
    """The arithmetic, with no neighbours to complicate it."""
    fp = _grid_part(n=8, pitch=0.5)               # 3.5mm across
    pcb = _Pcb({'U1': fp})
    e = escape.part_escape(pcb, 'U1', pitch_mm=0.45)
    north = next(f for f in e.faces if f.face == 'north')
    assert abs(north.span_mm - 3.5) < 1e-9, north.span_mm
    assert north.supply == int(3.5 // 0.45) == 7, north.supply
    assert north.blocked_mm == 0.0 and north.blockers == ()
    print(f"  PASS: 3.5mm face at 0.45mm lanes supplies {north.supply}")


def test_demand_is_the_nets_that_must_leave_through_that_face():
    fp = _grid_part(n=8, pitch=0.5)
    pcb = _Pcb({'U1': fp})
    e = escape.part_escape(pcb, 'U1', pitch_mm=0.45)
    # Every pad on the 8x8 perimeter has its own net; each side has 8 pads,
    # and the corner pads are assigned to exactly one face.
    total = sum(f.demand for f in e.faces)
    assert total == len(fp.pads), (total, len(fp.pads))
    assert e.interior_pads == 0
    print(f"  PASS: {total} perimeter nets split across 4 faces, none lost")


def test_a_deficit_is_reported_when_demand_exceeds_supply():
    """The whole point. A 0.4mm-pitch part cannot pass its own pins at a lane
    pitch wider than its pitch -- that is structural, not an ordering problem.
    """
    fp = _grid_part(n=10, pitch=0.4)
    pcb = _Pcb({'U1': fp})
    e = escape.part_escape(pcb, 'U1', pitch_mm=0.45)
    worst = e.worst
    assert worst is not None and worst.deficit > 0, [f.to_dict()
                                                     for f in e.faces]
    print(f"  PASS: {worst.face} supply {worst.supply} < demand "
          f"{worst.demand}, deficit {worst.deficit}")


def test_interior_pads_belong_to_no_face_and_are_reported_separately():
    """Rolling them into a face's demand would blame the face for what is
    really a fanout problem -- they need a via, not a lane."""
    fp = _grid_part(n=5, pitch=0.5, interior=True)
    pcb = _Pcb({'U1': fp})
    e = escape.part_escape(pcb, 'U1', pitch_mm=0.45)
    assert e.interior_pads == 9, e.interior_pads     # the inner 3x3
    assert sum(f.demand for f in e.faces) == len(fp.pads) - 9
    print(f"  PASS: {e.interior_pads} interior pads counted toward no face")


def test_a_neighbour_eats_supply_and_is_NAMED():
    """`blockers` is the field that turns a signal into a move."""
    fp = _grid_part(n=8, pitch=0.5, x0=10.0, y0=10.0)
    # A part parked just north of U1, overlapping half its span.
    blk = [_Pad(0, 0, 10.0, 9.5, 99), _Pad(0, 0, 11.75, 9.7, 99)]
    pcb = _Pcb({'U1': fp, 'J9': _Fp('CONN', blk)})
    e = escape.part_escape(pcb, 'U1', pitch_mm=0.45)
    north = next(f for f in e.faces if f.face == 'north')
    assert north.blocked_mm > 1.0, north.blocked_mm
    assert 'J9' in north.blockers, north.blockers
    clear = escape.part_escape(_Pcb({'U1': fp}), 'U1', pitch_mm=0.45)
    cn = next(f for f in clear.faces if f.face == 'north')
    assert north.supply < cn.supply, (north.supply, cn.supply)
    print(f"  PASS: J9 ate {north.blocked_mm:.2f}mm, supply "
          f"{cn.supply} -> {north.supply}, and is named")


def test_ignored_nets_do_not_create_demand():
    """A GND pad needs a via to the plane, not a lane. Counting it reports a
    deficit on every power-heavy face of every board."""
    fp = _grid_part(n=6, pitch=0.5)
    pcb = _Pcb({'U1': fp})
    every = {p.net_id for p in fp.pads}
    full = escape.part_escape(pcb, 'U1', pitch_mm=0.45)
    half = escape.part_escape(pcb, 'U1', pitch_mm=0.45,
                              ignore_net_ids=sorted(every)[:10])
    assert sum(f.demand for f in half.faces) < sum(f.demand
                                                  for f in full.faces)
    print("  PASS: ignored nets drop out of demand")


def test_through_hole_parts_are_not_fine_pitch_candidates():
    """A THT pin is reachable on EVERY copper layer -- there is no escape to be
    short of, however many pins it has."""
    pads = [_Pad(i * 0.5, 0, 10 + i * 0.5, 10, i + 1, drill=0.6)
            for i in range(60)]
    pcb = _Pcb({'J1': _Fp('DIP-60', pads)})
    assert escape.fine_pitch_parts(pcb) == []
    # ... and the identical lattice as SMD IS a candidate, so the test is about
    # the drill and not about the geometry failing some other check.
    smd = _Pcb({'J1': _Fp('DIP-60', [_Pad(p.local_x, p.local_y, p.global_x,
                                          p.global_y, p.net_id)
                                     for p in pads])})
    assert escape.fine_pitch_parts(smd) == ['J1']
    print("  PASS: THT excluded; the same lattice as SMD is detected")


def test_pitch_is_read_from_geometry_not_from_the_footprint_name():
    """A house library's part carries no pitch in its name, and a name that
    does carry one can disagree with the copper."""
    fine = _Pcb({'U1': _grid_part('HOUSE_LIB:MYSTERY', n=8, pitch=0.4)})
    assert escape.fine_pitch_parts(fine) == ['U1']
    coarse = _Pcb({'U1': _grid_part('SOME_BGA-256', n=8, pitch=1.27)})
    assert escape.fine_pitch_parts(coarse) == [], \
        "a 1.27mm 'BGA' escapes without help; the NAME must not decide"
    print("  PASS: detection follows the pad lattice, not the name")


def _real(board):
    from kicad_parser import parse_kicad_pcb
    path = os.path.join(ROOT, 'kicad_files', board)
    if not os.path.exists(path):
        return None, None
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path), path


def test_it_runs_on_real_boards_and_finds_the_fine_pitch_parts():
    for board in ('ulx3s.kicad_pcb', 'glasgow_revC.kicad_pcb'):
        pcb, path = _real(board)
        if pcb is None:
            print(f"  SKIP {board}: not in kicad_files/")
            continue
        refs = escape.fine_pitch_parts(pcb)
        led = escape.escape_ledger(pcb, pcb_file=path)
        assert len(led) == len(refs)
        # #1196: asked for, the population is ALSO "fine" at the board's own
        # rules, resolved the way the ledger resolves them -- a superset.
        tw, clr, dr = escape.lane_pitch_parts(pcb, path, return_rules=True)
        vp = escape.via_pitch(pcb, path, clearance=clr, design_rules=dr)
        wide = escape.fine_pitch_parts(pcb, rules=(tw, clr, vp))
        assert set(refs) <= set(wide)
        led_r = escape.escape_ledger(pcb, pcb_file=path,
                                     fine_at_board_rules=True)
        assert sorted(p.ref for p in led_r) == sorted(wide), (
            sorted(p.ref for p in led_r), sorted(wide))
        # Sorted worst-deficit-first, so the first row is the one to read.
        deficits = [p.worst.deficit if p.worst else 0 for p in led]
        assert deficits == sorted(deficits, reverse=True), deficits
        print(f"  PASS: {board} -> {len(refs)} fine-pitch part(s)"
              + (f", worst {led[0].ref} short {deficits[0]} lane(s) on "
                 f"{led[0].worst.face}" if led and led[0].worst else
                 ", none in deficit"))


def test_a_board_with_no_fine_pitch_parts_says_so_rather_than_erroring():
    pcb, _ = _real('splitflap_driver.kicad_pcb')
    if pcb is None:
        print("  SKIP: splitflap_driver not in kicad_files/")
        return
    led = escape.escape_ledger(pcb)
    assert isinstance(led, list), type(led)
    print(f"  PASS: {len(led)} fine-pitch part(s), no error")


if __name__ == '__main__':
    for fn in (test_supply_is_face_length_over_lane_pitch,
               test_demand_is_the_nets_that_must_leave_through_that_face,
               test_a_deficit_is_reported_when_demand_exceeds_supply,
               test_interior_pads_belong_to_no_face_and_are_reported_separately,
               test_a_neighbour_eats_supply_and_is_NAMED,
               test_ignored_nets_do_not_create_demand,
               test_through_hole_parts_are_not_fine_pitch_candidates,
               test_pitch_is_read_from_geometry_not_from_the_footprint_name,
               test_it_runs_on_real_boards_and_finds_the_fine_pitch_parts,
               test_a_board_with_no_fine_pitch_parts_says_so_rather_than_erroring):
        print(fn.__name__)
        fn()
    print("\nALL PASS")
