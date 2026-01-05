"""
Data types for BGA fanout routing.
"""

from dataclasses import dataclass, field
from typing import Tuple, Optional, List


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
