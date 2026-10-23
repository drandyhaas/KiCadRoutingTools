"""
Data types for QFN/QFP fanout routing.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from kicad_parser import Pad


@dataclass
class QFNLayout:
    """Represents the QFN package layout derived from pad analysis.

    All bbox/center coordinates are in the footprint's LOCAL frame (pad rows are
    axis-aligned there regardless of how the part is rotated on the board). The
    fp_* fields carry the footprint placement so stub geometry computed in the
    local frame can be transformed back to global board coordinates.
    """
    center_x: float
    center_y: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    width: float
    height: float
    pad_pitch: float
    edge_tolerance: float  # How close to edge to be considered "on edge"
    fp_x: float = 0.0          # footprint origin, global (for local->global)
    fp_y: float = 0.0
    fp_rotation: float = 0.0   # footprint rotation, degrees
    center_global_x: float = 0.0   # package center in global board coords
    center_global_y: float = 0.0


@dataclass
class PadInfo:
    """Analyzed information about a pad."""
    pad: Pad
    side: str  # 'top', 'bottom', 'left', 'right', 'center'
    # Unit vector along the pad's long axis, signed away from package center.
    # Since issue #410 'center' (interior/EP) pads carry a real axis too; a pad
    # exactly at the package center keeps the positive long-axis direction.
    escape_direction: Tuple[float, float]
    pad_length: float  # Extent across the escape axis (along edge)
    pad_width: float   # Extent along the escape axis


@dataclass
class FanoutStub:
    """A fanout stub from a QFN pad - two segments: straight then 45 degrees."""
    pad: Pad
    pad_pos: Tuple[float, float]
    corner_pos: Tuple[float, float]  # Where straight meets 45 degrees
    stub_end: Tuple[float, float]    # Final fanned-out endpoint
    side: str
    layer: str = "F.Cu"
    # #1332: the stub reached a SAME-NET pad and ends on it. `joined_pad`
    # names it ("REF.PAD"); `join_end`, when set, is a short closing segment
    # from stub_end into that pad, laid when the deepest contact point is
    # still outside the copper (a corner graze) so the joint is not a sliver.
    joined_pad: Optional[str] = None
    join_end: Optional[Tuple[float, float]] = None

    @property
    def net_id(self) -> int:
        return self.pad.net_id
