"""
Data types for QFN/QFP fanout routing.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

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
    escape_direction: Tuple[float, float]  # Unit vector pointing outward
    pad_length: float  # Length of pad (along edge)
    pad_width: float   # Width of pad (perpendicular to edge)


@dataclass
class FanoutStub:
    """A fanout stub from a QFN pad - two segments: straight then 45 degrees."""
    pad: Pad
    pad_pos: Tuple[float, float]
    corner_pos: Tuple[float, float]  # Where straight meets 45 degrees
    stub_end: Tuple[float, float]    # Final fanned-out endpoint
    side: str
    layer: str = "F.Cu"

    @property
    def net_id(self) -> int:
        return self.pad.net_id
