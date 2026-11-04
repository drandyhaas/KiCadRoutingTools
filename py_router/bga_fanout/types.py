"""
Data types for BGA fanout routing.
"""
from __future__ import annotations

from dataclasses import dataclass
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
    neighbor_connection: bool = False  # True if this connects directly to adjacent same-net pad
    # The ball a neighbour link ends on (its exit_pos). Off the top layer the
    # link reaches that ball through a via there, which manage_vias places.
    neighbor_pad: Optional['Pad'] = None

    @property
    def net_id(self) -> int:
        return self.pad.net_id


_route_uid_counter = 0


def route_uid(route) -> int:
    """Stable per-route identity for track custody (#508 finding 6).

    NOT id(): a replaced/dropped FanoutRoute's id() can be recycled by a later
    allocation, silently aliasing two routes' tracks (the id()-keyed-set
    keepalive footgun). The uid is stamped on the route object on first use
    and monotonically unique for the process lifetime.
    """
    global _route_uid_counter
    uid = getattr(route, '_track_uid', None)
    if uid is None:
        _route_uid_counter += 1
        uid = _route_uid_counter
        route._track_uid = uid
    return uid


def create_track(
    start: Tuple[float, float],
    end: Tuple[float, float],
    width: float,
    layer: str,
    net_id: int,
    pair_id: Optional[str] = None,
    is_existing: bool = False,
    route_uid: Optional[int] = None,
) -> dict:
    """
    Factory function to create a track dictionary.

    This provides a single point for track creation, making it easier to
    ensure consistency and eventually migrate to the Track dataclass.

    route_uid (#508 finding 6): id() of the FanoutRoute this track was derived
    from. _rebuild_track_for_route replaces a route's tracks by THIS key --
    a net_id-scoped removal deleted every same-net ball's escape while only
    one route's segments were re-added (multi-ball power rails shipped
    via-in-pad balls with no track).
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
    if route_uid is not None:
        d['route_uid'] = route_uid
    return d


def top_layer_barred(no_inner_top_layer, net_id) -> bool:
    """Is an inner ball of `net_id` kept off the top layer? `no_inner_top_layer`
    is the --no-inner-top-layer bool (every inner ball), or the set of net ids
    whose OWN class cannot fit the gap between the balls (#1529: the #1518 gap
    test at the pair clearance a ball's class sets)."""
    if isinstance(no_inner_top_layer, (set, frozenset)):
        return net_id in no_inner_top_layer
    return bool(no_inner_top_layer)
