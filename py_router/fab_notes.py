"""Fabrication notes for geometry this tool creates that the fab must know about.

Right now: via-in-pad (#489 §8). The tool places it from three separate paths
(qfn_fanout `allow_via_in_pad`, `route_planes --same-net-pad-clearance -1`,
bga_fanout/underpad) and said nothing about the requirement it had just created.
Via-in-pad needs IPC-4761 Type VII -- filled, capped and plated -- or solder
wicks out of the joint into the barrel during reflow. A site is counted when
the via BARREL overlaps the pad copper, not merely when its centre is inside
(#695) -- an off-centre tap wicks just the same. The tool cannot enforce
that (it is a fab process, not board geometry), so the least it can do is COUNT
the sites and say so in the run output.

Leaf module: no routing imports, so any front (CLI main, shared engine, GUI tab)
can call it.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

# IPC-4761 Type VII is the protection class via-in-pad requires.
VIA_IN_PAD_FAB_NOTE = (
    "via-in-pad requires IPC-4761 Type VII (filled + capped + plated); "
    "state it on the fab drawing or solder wicks into the barrel")


def _get(obj, name: str, default=None):
    """Read `name` from a dict or an attribute-style object."""
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _pad_holds(pad, x: float, y: float, margin: float = 0.0) -> bool:
    """Is (x, y) within `margin` of the pad's COPPER?

    Exact when check_drc is importable (custom-pad polygons, roundrect
    corners, circle/oval, and rect_rotation), which is the same function --
    and the same function-local-import trick -- check_connected._point_in_pad
    uses. The import is inside the call so this module still IMPORTS as a
    leaf: any front can `import fab_notes` without dragging a checker in at
    module load, and only a point that survives the cheap reject pays for it.

    The reject is the DIAGONAL one, copied from check_connected, not an
    axis-aligned box. A box in board space clips a rotated pad's copper,
    because size_x/size_y stay in the PAD's frame and the tilt rides in
    rect_rotation -- a 1.0x0.2 pad at 45 degrees holds (0.35, 0.35) inside its
    copper, and a box gate rejects it, losing a real Type VII site.

    The box is not a safe credit either, in the other direction (#695):
    inflating one by the barrel radius over-credits by up to a factor of
    sqrt(2) along the diagonal, which is exactly where bga_fanout puts a
    dog-bone via (the half-pitch diagonal). Measured on ulx3s's BGA-381: a box
    named all 379 dog-bone vias as via-in-pad and the exact test named none,
    the nearest copper being 0.37mm from a 0.225mm barrel. So the fallback
    used when check_drc cannot be imported is the pad's circumscribed reach,
    which kills that corner artifact -- still not exact, and deliberately
    erring toward naming a site rather than missing one.
    """
    hx = _get(pad, 'size_x', 0.0) / 2
    hy = _get(pad, 'size_y', 0.0) / 2
    dx = x - _get(pad, 'global_x', 0.0)
    dy = y - _get(pad, 'global_y', 0.0)
    reach = max(hx, hy) + margin
    if dx * dx + dy * dy > reach * reach * 2:   # bbox-DIAGONAL prefilter
        return False
    try:
        from check_drc import point_to_pad_distance
    except ImportError:                          # no checker on this path
        # A point outside the pad's own circumscribed reach cannot be within
        # `margin` of copper that lives inside it. Only an ImportError is
        # caught: a geometry error must surface, not silently downgrade.
        return math.hypot(dx, dy) <= math.hypot(hx, hy) + margin
    return point_to_pad_distance(x, y, pad) <= margin


def via_overlaps_pad(pad, via_x: float, via_y: float, via_size: float,
                     margin: float = 0.0) -> bool:
    """Does a via's BARREL overlap this pad's copper? (#846)

    The public form of ``_pad_holds``, with the barrel radius applied for the
    caller. This is the question "is this via in that pad" actually means, and
    it is asked in two places that used to disagree: here, for the IPC-4761 fab
    note, and in ``qfn_fanout``'s commit loop, which decides whether to clamp
    the via to its pad edge (#202). The commit loop used to ask a 0.001mm
    CENTRE-coincidence question instead, so a via staggered onto its own pad
    was reported by this module as needing Type VII while shipping unclamped.

    ``via_size`` of 0 or None claims the 0.6 default's radius, as every other
    consumer of a size-less via in this repo does. ``margin`` is a FLOOR on the
    credit, not a replacement for it -- see ``via_in_pad_sites``.
    """
    vr = (via_size if via_size else 0.6) / 2.0
    return _pad_holds(pad, via_x, via_y, max(vr - 1e-6, margin))


def via_in_pad_sites(vias, pads_by_net: Dict[int, list],
                     margin: float = 0.0) -> List[Tuple[object, object]]:
    """[(via, pad)] for every via whose BARREL overlaps a SAME-NET pad.

    Overlap, not centre-containment (#695): an off-centre via-in-pad can have
    its centre just outside the pad outline while the barrel still overlaps
    the copper -- and solder wicks through copper continuity, not through the
    via's centre point, so that joint needs Type VII exactly as much. The
    router places such vias on purpose (QFN allow_via_in_pad, plane taps
    clamped to the pad edge, BGA underpad drops). Measured: the centre test
    missed 6 real sites on rp2350_fpga_eensy_prePlane alone, every one a true
    overlap against the exact pad shape.

    The credit must be EXACT, not a box inflated by the radius -- see
    _pad_holds. Over-counting here is not cheap: this note is a fab process
    requirement (filled, capped, plated), so a false site costs money, and
    bga_fanout hands this function its dog-bone vias, which sit on the
    half-pitch diagonal where a box over-credits worst.

    `margin` is a FLOOR on that credit, not a replacement for it: the barrel
    radius applies even when the caller passes 0. That is a change of meaning
    (#695) -- `margin` could once be used to ask for a credit TIGHTER than the
    barrel, and can no longer. No in-repo caller passes it.

    Same-net only: a via inside a FOREIGN pad is a short, not via-in-pad, and is
    the DRC checkers' business. Accepts vias as dicts or objects.
    """
    sites = []
    for via in vias or ():
        net_id = _get(via, 'net_id')
        if net_id is None:
            continue
        vx, vy = _get(via, 'x'), _get(via, 'y')
        if vx is None or vy is None:
            continue
        # A via with no declared size claims the 0.6 default's radius, as
        # every other consumer of a size-less via in this repo does.
        vsz = _get(via, 'size', 0.6)
        for pad in pads_by_net.get(net_id, ()) or ():
            if _get(pad, 'drill', 0.0) or 0.0:
                continue  # a plated TH pad's own barrel is not via-in-pad
            if via_overlaps_pad(pad, vx, vy, vsz, margin):
                sites.append((via, pad))
                break
    return sites


def via_in_pad_summary(vias, pads_by_net: Dict[int, list],
                       margin: float = 0.0) -> Optional[Dict]:
    """Machine-readable via-in-pad record, or None when there is none.

    Sites are barrel-overlap, not centre-in-pad -- see via_in_pad_sites.

    {'count', 'pads' (["U1.A1", ...]), 'note'} -- returned so a caller can put it
    in a run summary instead of only printing it.
    """
    sites = via_in_pad_sites(vias, pads_by_net, margin)
    if not sites:
        return None
    refs = []
    for _via, pad in sites:
        ref = f"{_get(pad, 'component_ref', '?')}.{_get(pad, 'pad_number', '?')}"
        if ref not in refs:
            refs.append(ref)
    return {'count': len(sites), 'pads': refs, 'note': VIA_IN_PAD_FAB_NOTE}


def print_via_in_pad_note(vias, pads_by_net: Dict[int, list],
                          context: str = "", margin: float = 0.0,
                          max_refs: int = 8) -> Optional[Dict]:
    """Print one fab note when this run put vias in pads. Returns the record.

    `margin` floors the barrel-overlap credit; see via_in_pad_sites.
    """
    record = via_in_pad_summary(vias, pads_by_net, margin)
    if not record:
        return None
    where = f" ({context})" if context else ""
    shown = record['pads'][:max_refs]
    more = len(record['pads']) - len(shown)
    print(f"\n  FAB NOTE{where}: {record['count']} via(s) placed in pad(s) "
          f"[{', '.join(shown)}{f', +{more} more' if more > 0 else ''}] -- "
          f"{VIA_IN_PAD_FAB_NOTE}.")
    return record
