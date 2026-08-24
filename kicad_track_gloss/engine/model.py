"""Small, API-neutral data model used by the selected-track gloss engine."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple


@dataclass(frozen=True)
class Segment:
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    width: float
    layer: int
    net_id: int
    uuid: str = ""
    locked: bool = False
    arc: bool = False
    net_name: str = ""


@dataclass(frozen=True)
class CircleObstacle:
    x: float
    y: float
    radius: float
    net_id: int
    layers: Tuple[int, ...] = ()
    kind: str = "via"
    clearance: float = 0.0


@dataclass(frozen=True)
class PolygonKeepout:
    points: Tuple[Tuple[float, float], ...]
    layers: Tuple[int, ...] = ()


@dataclass(frozen=True)
class PadRegion:
    """Copper area in which a same-net track termination may move."""
    x: float
    y: float
    width: float
    height: float
    orientation_degrees: float
    shape: str
    corner_radius: float
    net_id: int
    layers: Tuple[int, ...] = ()


@dataclass
class BoardModel:
    segments: List[Segment]
    obstacles: List[CircleObstacle] = field(default_factory=list)
    keepouts: List[PolygonKeepout] = field(default_factory=list)
    # Values resolved by KiCad's own effective netclass machinery. Rules that
    # SWIG does not expose remain outside this conservative geometry model.
    net_clearances: Dict[int, float] = field(default_factory=dict)
    minimum_clearance: float = 0.0
    copper_edge_clearance: float = 0.0
    board_bounds: object = None
    pad_regions: List[PadRegion] = field(default_factory=list)


@dataclass(frozen=True)
class AddedSegment:
    start: Tuple[float, float]
    end: Tuple[float, float]
    width: float
    layer: int
    net_id: int


@dataclass
class GlossResult:
    remove_keys: List[str] = field(default_factory=list)
    additions: List[AddedSegment] = field(default_factory=list)
    saved_mm: float = 0.0
    chains_considered: int = 0
    chains_changed: int = 0
    warnings: List[str] = field(default_factory=list)

    @property
    def changed(self) -> bool:
        return bool(self.remove_keys or self.additions)


def geometry_key(segment: Segment) -> str:
    """Return a direction-independent identity when KiCad exposes no KIID."""
    a = (round(segment.start_x, 6), round(segment.start_y, 6))
    b = (round(segment.end_x, 6), round(segment.end_y, 6))
    if b < a:
        a, b = b, a
    return "geom:{:.6f},{:.6f}:{:.6f},{:.6f}:{:.6f}:{}:{}".format(
        a[0], a[1], b[0], b[1], round(segment.width, 6),
        segment.layer, segment.net_id)


def segment_key(segment: Segment) -> str:
    return segment.uuid or geometry_key(segment)
