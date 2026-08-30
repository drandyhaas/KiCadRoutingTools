#!/usr/bin/env python3
"""#628: the CAP-repair pass must take the same own-milled-ring exemption.

PR #638 scoped the swallow probe by ownership at the four sites that "charge at
a non-zero margin" -- `violation_parts`, `candidate_valid`, `legality_metrics`,
`grade_pad_legality` -- and deliberately left the rest, on the argument that a
margin-0 gate is numerically inert because the probe charges `+= self.margin`.

That argument is correct for `lock_advisor`, `placement_state` and `seeder`:
all three call `rect_outside_amount` only, so at margin 0 the ring term
contributes nothing. It does NOT reach `fanout_clearance`, for two independent
reasons:

  1. its gate is built at `margin = max(clearance, board_edge_clearance)`,
     which is not 0; and
  2. it reaches the probe through `rect_blocked`, whose ring term is BOOLEAN
     (`return True`) -- margin never enters it, so even a margin-0 gate would
     not be inert on that path.

The consequence is worse here than in quench, because there is no unfreeze
branch to escape through: `_blocked_geom` is a hard veto, so an owner cap has
EVERY candidate rejected and simply never moves.

Fixture: `C1`, a two-pad front-side cap whose pads sit inside a milled ring --
which is *why* the ring is reclassified (`drop_pad_containing_cutouts` moves a
ring into `board_edge_contours` exactly when it encloses >= 2 pad centres) --
and whose courtyard swallows that ring whole. `C2` owns nothing and is the
anti-over-scoping control: a fix that exempts everyone passes the owner checks
and fails that one.

    python3 tests/test_628_cap_owns_milled_ring.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement.fanout_clearance import _Repair  # noqa: E402

fails = []


def check(name, cond):
    print(("  ok  " if cond else " FAIL ") + name)
    if not cond:
        fails.append(name)


# Degrade rather than crash on a tree without the threading: an AttributeError
# at the first call would abort before the BEHAVIOURAL checks run, and "the
# suite errored" is a much weaker statement than "the owner is still blocked".
def _owned(rep, ref):
    fn = getattr(rep, '_owned_rings', None)
    return fn(ref) if fn is not None else frozenset()


def _blocked(rep, rect, ref=None):
    """_rect_edge_blocked, passing ref only where the signature accepts it."""
    if ref is None:
        return rep._rect_edge_blocked(rect)
    try:
        return rep._rect_edge_blocked(rect, ref=ref)
    except TypeError:
        return rep._rect_edge_blocked(rect)


def _veto(rep, ref, x, y):
    cap = rep.caps[ref]
    return rep._blocked_geom(ref, cap, x, y, cap.seed_rot)


CLEARANCE = 0.25
EDGE = 0.55


def _rect_edge(x1, y1, x2, y2, tag):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return "\n".join(
        f' (gr_line (start {pts[i][0]} {pts[i][1]}) '
        f'(end {pts[(i + 1) % 4][0]} {pts[(i + 1) % 4][1]}) '
        f'(layer "Edge.Cuts") (width 0.05) (uuid "{tag}{i}"))' for i in range(4))


BOARD = f'''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "NA")
 (net 2 "NB")
{_rect_edge(0, 0, 60, 40, 'o')}
{_rect_edge(21, 19.4, 23, 20.6, 'm')}
 (footprint "test:cap" (layer "F.Cu") (at 22 20)
  (property "Reference" "C1" (at 0 -2) (layer "F.SilkS"))
  (fp_rect (start -3 -3) (end 3 3) (layer "F.CrtYd") (uuid "cyC1"))
  (pad "1" smd rect (at -0.6 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "NA") (uuid "pC1a"))
  (pad "2" smd rect (at 0.6 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "NA") (uuid "pC1b"))
 )
 (footprint "test:cap" (layer "F.Cu") (at 32 20)
  (property "Reference" "C2" (at 0 -2) (layer "F.SilkS"))
  (fp_rect (start -3 -3) (end 3 3) (layer "F.CrtYd") (uuid "cyC2"))
  (pad "1" smd rect (at -0.6 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "NB") (uuid "pC2a"))
  (pad "2" smd rect (at 0.6 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "NB") (uuid "pC2b"))
 )
 (footprint "test:BGA-4" (layer "F.Cu") (at 26 31)
  (property "Reference" "U1" (at 0 -3) (layer "F.SilkS"))
  (fp_rect (start -1.5 -1.5) (end 1.5 1.5) (layer "F.CrtYd") (uuid "cyU1"))
  (pad "A1" smd circle (at -0.4 -0.4) (size 0.3 0.3) (layers "F.Cu") (net 1 "NA") (uuid "pU1a"))
  (pad "A2" smd circle (at 0.4 -0.4) (size 0.3 0.3) (layers "F.Cu") (net 2 "NB") (uuid "pU1b"))
  (pad "B1" smd circle (at -0.4 0.4) (size 0.3 0.3) (layers "F.Cu") (net 1 "NA") (uuid "pU1c"))
  (pad "B2" smd circle (at 0.4 0.4) (size 0.3 0.3) (layers "F.Cu") (net 2 "NB") (uuid "pU1d"))
 )
)
'''

# C1's courtyard at its seed (22, 20): 19..25 x 17..23, which contains the whole
# 21..23 x 19.4..20.6 milled ring.
OWNER_RECT = (19.0, 17.0, 25.0, 23.0)


def _write():
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w') as f:
        f.write(BOARD)
    return path


def _repair(pcb, path):
    return _Repair(pcb, path, clearance=CLEARANCE, grid_step=0.05,
                   board_edge_clearance=EDGE, near_margin=12.0,
                   capture_radius=2.0, default_via_size=0.5,
                   cap_prefix='C,R', extra_locked=set())


def main():
    path = _write()
    try:
        pcb = parse_kicad_pcb(path)
        bi = pcb.board_info

        # --- fixture sanity: the ring must really be MILLED, not a cutout.
        # If the reclassification ever stops happening the probe would catch
        # the rect for the wrong reason and this file would pass vacuously.
        ec = [c for c in (getattr(bi, 'board_edge_contours', None) or [])
              if len(c) >= 3]
        cut = [c for c in (bi.board_cutouts or []) if len(c) >= 3]
        check("fixture: exactly one MILLED contour", len(ec) == 1)
        check("fixture: and no genuine cutout", len(cut) == 0)

        rep = _repair(pcb, path)
        check("fixture: the cap repair pass adopted C1 as a mover",
              'C1' in rep.caps)
        check("fixture: the edge gate is ACTIVE (else nothing is tested)",
              rep.edge_gate.active)
        check(f"fixture: gate margin is non-zero ({rep.edge_margin}) -- the "
              f"'margin-0 is inert' argument does not cover this gate",
              rep.edge_margin > 0.0)

        # --- API surface (reported, not fatal -- see _owned/_blocked above)
        check("API: _Repair grows _owned_rings",
              getattr(rep, '_owned_rings', None) is not None)

        # --- ownership
        owned = _owned(rep, 'C1')
        check(f"C1 owns the milled ring its pads sit in ({sorted(owned)})",
              len(owned) == 1)
        check("C2, whose pads are elsewhere, owns nothing",
              not _owned(rep, 'C2'))

        # --- the fix: the exemption is what clears it, not the fixture
        check("without a ref, C1's own relief still blocks it "
              "(pre-#628 behaviour preserved)",
              _blocked(rep, OWNER_RECT) is True)
        check("with ref='C1', its OWN milled ring no longer blocks it",
              _blocked(rep, OWNER_RECT, ref='C1') is False)

        # --- anti-over-scoping: a non-owner over the same ring stays blocked
        check("ANTI-OVER-SCOPING: ref='C2' over C1's ring is STILL blocked",
              _blocked(rep, OWNER_RECT, ref='C2') is True)

        # --- the exemption is per-ring, not a blanket edge pardon
        off = (-1.0, 17.0, 5.0, 23.0)      # hangs off the left board edge
        check("owner is NOT pardoned at the real board edge "
              "(exemption is per-ring, not per-part)",
              _blocked(rep, off, ref='C1') is True)

        # --- and the hard veto that motivated all of this actually lifts
        check("_blocked_geom no longer vetoes C1 at its own seed pose",
              _veto(rep, 'C1', 22.0, 20.0) is False)
    finally:
        os.unlink(path)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
