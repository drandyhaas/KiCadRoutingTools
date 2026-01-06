"""
Data types for QFN/QFP fanout routing.
"""

from dataclasses import dataclass
from typing import Tuple

from kicad_parser import Pad


@dataclass
class QFNLayout:
    """Represents the QFN package layout derived from pad analysis."""
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
