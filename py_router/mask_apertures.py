"""Solder-mask opening geometry for pads (#1232).

A pad's mask opening is its copper grown by its resolved solder-mask margin.
`resolve_mask_margin` follows KiCad's `PAD::GetSolderMaskExpansion`, probed on
pcbnew 10.0.7:

- the pad's own `(solder_mask_margin)`, else its footprint's, else the board's
  `(setup (pad_to_mask_clearance))`;
- an explicit 0 is unset up to `kicad_parser.PASTE_ZERO_IS_UNSET_MAX_VERSION`
  (the parsers store None for it), as for the paste margin;
- a pad that lists no copper layer gets 0 (its shape IS the opening);
- a negative margin is clamped at `-min(size) / 2`.

A positive margin grows the copper by the Minkowski sum, so the opening's
corners are ROUNDED at the margin: KiCad plots a 0.3 x 1.3 mm rect pad with a
0.2 mm margin as a `RoundRect` aperture of corner radius 0.2 (watchy J3,
F.Mask Gerber from kicad-cli 10.0.7). The distance to such an opening is the
distance to the copper minus the margin, so custom pads, roundrect corners
and `rect_rotation` hold through `check_drc.point_to_pad_distance`. A negative
margin shrinks the pad (`paste_apertures._inflated_pad`, with its disclosed
superset for a custom pad).

Leaf module: `check_drc` is imported inside the function that needs it.
"""

import re
from dataclasses import dataclass
from typing import List, Optional

MASK_LAYERS = ('F.Mask', 'B.Mask')
MASK_TO_COPPER = {'F.Mask': 'F.Cu', 'B.Mask': 'B.Cu'}


@dataclass
class MaskOpening:
    """One pad's solder-mask opening on one side, in board millimetres.
    `shape_pad` is the pad itself for a margin >= 0, else a shrunk copy."""
    pad: object
    layer: str
    margin: float
    shape_pad: object

    def label(self) -> str:
        return '%s.%s %s' % (getattr(self.pad, 'component_ref', '') or 'board',
                             getattr(self.pad, 'pad_number', ''), self.layer)


def pad_mask_layers(pad) -> List[str]:
    """The mask layers a pad opens, `*.Mask` expanded."""
    out = []
    for ln in (getattr(pad, 'layers', None) or []):
        for one in (MASK_LAYERS if ln == '*.Mask' else (ln,)):
            if one in MASK_LAYERS and one not in out:
                out.append(one)
    return out


def _lists_copper(pad) -> bool:
    # pcbnew grows an NPTH pad that lists `*.Cu` by the margin too (probed), so
    # this reads the layer list, not paste_apertures.pad_has_copper.
    return any(ln.endswith('.Cu') for ln in (getattr(pad, 'layers', None) or []))


def resolve_mask_margin(pad, fp, board_info) -> float:
    """The solder-mask expansion (mm) KiCad applies to `pad`."""
    if not _lists_copper(pad):
        return 0.0
    m = getattr(pad, 'mask_margin', None)
    if m is None and fp is not None:
        m = getattr(fp, 'mask_margin', None)
    if m is None:
        m = getattr(board_info, 'pad_to_mask_clearance', 0.0) if board_info is not None else 0.0
    m = float(m or 0.0)
    if m < 0.0:
        if getattr(pad, 'shape', '') == 'custom':
            sx, sy = getattr(pad, 'anchor_size', None) or (pad.size_x, pad.size_y)
        else:
            sx, sy = pad.size_x, pad.size_y
        m = max(m, -min(sx, sy) / 2.0)
    return m


def pad_mask_openings(pcb_data) -> List[MaskOpening]:
    """Every pad mask opening on the board, F.Mask before B.Mask per pad."""
    from paste_apertures import _inflated_pad
    out = []
    for fp in pcb_data.footprints.values():
        for pad in fp.pads:
            layers = pad_mask_layers(pad)
            if not layers:
                continue
            m = resolve_mask_margin(pad, fp, pcb_data.board_info)
            shape = pad if m >= 0.0 else _inflated_pad(pad, m, m)
            for layer in layers:
                out.append(MaskOpening(pad=pad, layer=layer, margin=m, shape_pad=shape))
    return out


def opening_distance(x: float, y: float, opening: MaskOpening) -> float:
    """Distance (mm) from a point to the opening's edge; 0 inside it."""
    from check_drc import point_to_pad_distance
    d = point_to_pad_distance(x, y, opening.shape_pad)
    return max(0.0, d - opening.margin if opening.margin > 0.0 else d)


def side_enabled(inner: Optional[str], side: str) -> bool:
    """Does a canonical via-protection token's inner text (`(front yes) (back
    no)`, the form fab_notes.effective_via_protection returns) enable `side`
    ('front' or 'back')?"""
    m = re.search(r'\(%s\s+(\w+)\)' % side, inner or '')
    return bool(m) and m.group(1) in ('yes', 'true')
