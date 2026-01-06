"""
Data types for BGA fanout routing.
"""

from dataclasses import dataclass, field
from typing import Tuple, Optional, List, TYPE_CHECKING

if TYPE_CHECKING:
    from kicad_parser import Pad


@dataclass
class Channel:
    """A routing channel between ball rows/columns."""
    orientation: str  # 'horizontal' or 'vertical'
    position: float   # Y for horizontal, X for vertical
    index: int


@dataclass
class BGAGrid:
    """Represents the BGA ball grid structure."""
    pitch_x: float
    pitch_y: float
    rows: List[float]  # Sorted Y positions
    cols: List[float]  # Sorted X positions
    center_x: float
    center_y: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass
class DiffPairPads:
    """A differential pair of pads (P and N) - tracks pad objects."""
    base_name: str  # Common name without _P/_N suffix
    p_pad: Optional['Pad'] = None
    n_pad: Optional['Pad'] = None

    @property
    def is_complete(self) -> bool:
        return self.p_pad is not None and self.n_pad is not None


@dataclass
class FanoutRoute:
    """Complete fanout: pad -> 45° stub -> channel -> exit -> jog."""
    pad: 'Pad'
    pad_pos: Tuple[float, float]
    stub_end: Tuple[float, float]  # Where stub meets channel (or exit for edge pads)
    exit_pos: Tuple[float, float]  # Where route exits BGA
    jog_end: Tuple[float, float] = None  # End of 45° jog after exit
    jog_extension: Tuple[float, float] = None  # Extension point for outside track of diff pair
    channel_point: Tuple[float, float] = None  # First channel point for half-edge inner pads (45° entry)
    channel_point2: Tuple[float, float] = None  # Second channel point (after horizontal segment)
    pre_channel_jog: Tuple[float, float] = None  # Jog point before channel (for jogged routes to farther channel)
    channel: Optional[Channel] = None  # None for edge pads with direct escape
    escape_dir: str = ''  # 'left', 'right', 'up', 'down'
    is_edge: bool = False  # True for outer row/column pads
    layer: str = "F.Cu"
    pair_id: Optional[str] = None  # Differential pair base name if part of a pair
    is_p: bool = True  # True for P, False for N in differential pair

    @property
    def net_id(self) -> int:
        return self.pad.net_id


@dataclass
class RoutingConfig:
    """Configuration parameters for BGA fanout routing."""
    track_width: float = 0.1
    clearance: float = 0.1
    diff_pair_gap: float = 0.101
    exit_margin: float = 0.5
    via_size: float = 0.3
    via_drill: float = 0.2
    layers: List[str] = field(default_factory=lambda: ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])

    @property
    def min_spacing(self) -> float:
        """Minimum spacing between tracks (track width + clearance)."""
        return self.track_width + self.clearance

    @property
    def pair_spacing(self) -> float:
        """Center-to-center spacing for differential pairs."""
        return self.track_width + self.diff_pair_gap


@dataclass
class Track:
    """Represents a PCB track segment."""
    start: Tuple[float, float]
    end: Tuple[float, float]
    width: float
    layer: str
    net_id: int
    pair_id: Optional[str] = None
    is_existing: bool = False

    def to_dict(self) -> dict:
        """Convert to dictionary format for compatibility with existing code."""
        d = {
            'start': self.start,
            'end': self.end,
            'width': self.width,
            'layer': self.layer,
            'net_id': self.net_id,
        }
        if self.pair_id is not None:
            d['pair_id'] = self.pair_id
        if self.is_existing:
            d['is_existing'] = self.is_existing
        return d

    @classmethod
    def from_dict(cls, d: dict) -> 'Track':
        """Create Track from dictionary."""
        return cls(
            start=d['start'],
            end=d['end'],
            width=d['width'],
            layer=d['layer'],
            net_id=d['net_id'],
            pair_id=d.get('pair_id'),
            is_existing=d.get('is_existing', False),
        )


def create_track(
    start: Tuple[float, float],
    end: Tuple[float, float],
    width: float,
    layer: str,
    net_id: int,
    pair_id: Optional[str] = None,
    is_existing: bool = False,
) -> dict:
    """
    Factory function to create a track dictionary.

    This provides a single point for track creation, making it easier to
    ensure consistency and eventually migrate to the Track dataclass.
    """
    d = {
        'start': start,
        'end': end,
        'width': width,
        'layer': layer,
        'net_id': net_id,
    }
    if pair_id is not None:
        d['pair_id'] = pair_id
    if is_existing:
        d['is_existing'] = is_existing
    return d
