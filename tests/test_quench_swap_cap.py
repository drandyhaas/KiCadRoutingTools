"""
Tests for the quench swap displacement cap (issue #430).

Same-footprint swaps used to teleport parts arbitrarily far from their seed
positions (only the improvement mattered), stranding components outside the
--max-displacement contract. The fix rejects any swap where either part would
land beyond swap_max_displacement (default: max_displacement) of its OWN
seed, and adds the swap_max_displacement knob to tighten swaps independently.

The synthetic swap board isolates the swap phase: with step=1000 the nudge
grid degenerates to the seed point only (n = int(max_disp/1000) = 0), which
is skipped as the current pose, so ANY movement must come from the swap
block. The final test runs the full optimizer on a real board and checks the
headline #430 invariant: no returned part ends up further than
max_displacement (+ grid snap slop) from where it started.
"""

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

from kicad_parser import parse_kicad_pcb
from placement.quench import quench

INTERF_U = os.path.join(os.path.dirname(__file__), '..', 'kicad_files',
                        'interf_u_unrouted.kicad_pcb')


def _cap_footprint(ref, x, y, net_id, net_name, locked=False):
    """Movable 2-pad 1.6x0.8mm part; pad 1 (at -0.5,0) carries the net."""
    # (locked yes) must sit in the footprint header, BEFORE the first (pad
    # (placement/parser.py extract_locked_refs requirement).
    lock = '\t\t(locked yes)\n' if locked else ''
    return f'''\t(footprint "test:CAP2P"
\t\t(layer "F.Cu")
{lock}\t\t(uuid "fp-{ref}")
\t\t(at {x} {y})
\t\t(property "Reference" "{ref}"
\t\t\t(at 0 0)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at -0.5 0)
\t\t\t(size 0.6 0.8)
\t\t\t(layers "F.Cu")
\t\t\t(net {net_id} "{net_name}")
\t\t\t(uuid "p1-{ref}")
\t\t)
\t\t(pad "2" smd rect
\t\t\t(at 0.5 0)
\t\t\t(size 0.6 0.8)
\t\t\t(layers "F.Cu")
\t\t\t(uuid "p2-{ref}")
\t\t)
\t)
'''


def _anchor_footprint(ref, x, y, net_id, net_name):
    """Locked single-pad net anchor (a connector pin the airwire pulls at)."""
    return f'''\t(footprint "test:PIN"
\t\t(layer "F.Cu")
\t\t(locked yes)
\t\t(uuid "fp-{ref}")
\t\t(at {x} {y})
\t\t(property "Reference" "{ref}"
\t\t\t(at 0 0)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at 0 0)
\t\t\t(size 1 1)
\t\t\t(layers "F.Cu")
\t\t\t(net {net_id} "{net_name}")
\t\t\t(uuid "p1-{ref}")
\t\t)
\t)
'''


def _swap_board(d, lock_c2=False):
    """Two movable same-footprint 2-pad parts C1@(150,100) on net NA and
    C2@(150+d,100) on net NB, plus locked anchors J1 (net NA) at
    (150+d+20,100) and J2 (net NB) at (130,100). C1's net pulls it right,
    C2's pulls it left, so swapping C1<->C2 shortens both airwires by d
    (improving for any d>0). Board outline (100,80)-(200,120) Edge.Cuts.
    Writes the board to a temp file, returns (pcb_data, path)."""
    body = '(kicad_pcb\n\t(version 20241229)\n'
    body += '\t(net 0 "")\n\t(net 1 "NA")\n\t(net 2 "NB")\n'
    body += '''\t(gr_rect
\t\t(start 100 80)
\t\t(end 200 120)
\t\t(stroke
\t\t\t(width 0.1)
\t\t\t(type solid)
\t\t)
\t\t(layer "Edge.Cuts")
\t\t(uuid "edge1")
\t)
'''
    body += _cap_footprint('C1', 150.0, 100.0, 1, 'NA')
    body += _cap_footprint('C2', 150.0 + d, 100.0, 2, 'NB', locked=lock_c2)
    body += _anchor_footprint('J1', 150.0 + d + 20.0, 100.0, 1, 'NA')
    body += _anchor_footprint('J2', 130.0, 100.0, 2, 'NB')
    body += ')\n'
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w') as f:
        f.write(body)
    return parse_kicad_pcb(path), path


# step=1000 makes the nudge phase a guaranteed no-op (only candidate is the
# seed point, skipped as the current pose): any movement comes from swaps.
SWAP_ONLY = dict(step=1000.0, allow_rotations=False, max_passes=2)


def test_swap_beyond_cap_rejected():
    """An IMPROVING swap (gain 2*d = 16mm) is still rejected when the parts
    would land beyond the cap from their own seeds (8mm > 5mm)."""
    pcb, path = _swap_board(8)
    result = quench(pcb, path, max_displacement=5.0, **SWAP_ONLY)
    assert result == [], \
        f"swap beyond max_displacement must be rejected, got {result}"
    os.unlink(path)


def test_swap_within_cap_accepted_and_reported():
    """The same improving swap fires when within the cap (3mm <= 5mm), and
    both parts are reported with exchanged positions."""
    pcb, path = _swap_board(3)
    result = quench(pcb, path, max_displacement=5.0, **SWAP_ONLY)
    by_ref = {r['reference']: r for r in result}
    assert set(by_ref) == {'C1', 'C2'}, f"expected both caps swapped: {result}"
    assert by_ref['C1']['new_x'] == 153.0
    assert by_ref['C1']['new_y'] == 100.0
    assert by_ref['C2']['new_x'] == 150.0
    assert by_ref['C2']['new_y'] == 100.0
    os.unlink(path)


def test_swap_cap_flag_tightens():
    """swap_max_displacement < max_displacement rejects a swap the general
    cap would allow (the exact swap accepted in the previous test)."""
    pcb, path = _swap_board(3)
    result = quench(pcb, path, max_displacement=5.0,
                    swap_max_displacement=2.0, **SWAP_ONLY)
    assert result == [], \
        f"swap beyond swap_max_displacement must be rejected, got {result}"
    os.unlink(path)


def test_locked_parts_never_swap():
    """A locked part is never a swap partner and never appears in the
    result, even when swapping with it would improve the cost."""
    pcb, path = _swap_board(3, lock_c2=True)
    result = quench(pcb, path, max_displacement=5.0, **SWAP_ONLY)
    assert all(r['reference'] != 'C2' for r in result), \
        f"locked C2 must never move: {result}"
    # With its only same-footprint partner locked, C1 has nothing to swap
    # with and the nudge phase is frozen, so nothing moves at all.
    assert result == []
    os.unlink(path)


def test_swap_cap_validation():
    """swap_max_displacement must be within [0, max_displacement]."""
    pcb, path = _swap_board(3)
    with pytest.raises(ValueError):
        quench(pcb, path, max_displacement=5.0, swap_max_displacement=6.0,
               **SWAP_ONLY)
    with pytest.raises(ValueError):
        quench(pcb, path, max_displacement=5.0, swap_max_displacement=-1.0,
               **SWAP_ONLY)
    os.unlink(path)


def test_no_stranding_after_full_run():
    """Headline #430 invariant on a real board: a full quench run (nudges,
    rotations AND swaps) leaves every reported part within max_displacement
    (plus the grid_step snap slop) of its input position."""
    pcb = parse_kicad_pcb(INTERF_U)
    orig = {ref: (fp.x, fp.y) for ref, fp in pcb.footprints.items()}
    max_disp = 3.0
    result = quench(pcb, INTERF_U, max_displacement=max_disp, step=1.0,
                    max_passes=3)
    assert result, "fixture should produce at least one improving move"
    for placement in result:
        ref = placement['reference']
        ox, oy = orig[ref]
        dist = math.hypot(placement['new_x'] - ox, placement['new_y'] - oy)
        # 0.1 = default grid_step: candidate positions snap to the grid, so a
        # radius-capped candidate can end up at most one snap past the cap.
        assert dist <= max_disp + 0.1 + 1e-6, \
            f"{ref} stranded {dist:.3f}mm from seed (cap {max_disp}mm)"


if __name__ == '__main__':
    test_swap_beyond_cap_rejected()
    test_swap_within_cap_accepted_and_reported()
    test_swap_cap_flag_tightens()
    test_locked_parts_never_swap()
    test_swap_cap_validation()
    test_no_stranding_after_full_run()
    print("ALL PASS")
