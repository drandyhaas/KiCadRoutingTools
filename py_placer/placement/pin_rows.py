"""Is a footprint a through-hole part? (#1196)

`is_through_hole_part`: plated through-hole pins outnumber SMD pads. On a
two-layer board of DIPs, PGAs and headers a through-hole pin is on every
copper layer, so the escape ledger (`escape.fine_pitch_parts`) excludes such
parts by this majority test.
"""
from __future__ import annotations


def _plated(pad) -> bool:
    from kicad_parser import pad_is_plated_through
    return pad_is_plated_through(pad)


def is_through_hole_part(fp) -> bool:
    """Plated through-hole pins outnumber SMD pads -- the part's own pins,
    not a thermal-via field under an exposed pad (#1196). The same majority
    test `escape.fine_pitch_parts` excludes through-hole parts by."""
    from kicad_parser import non_aperture_pads
    pads = non_aperture_pads(fp)
    tht = sum(1 for p in pads if _plated(p))
    smd = sum(1 for p in pads if (getattr(p, 'drill', 0) or 0) <= 0)
    return tht > smd
