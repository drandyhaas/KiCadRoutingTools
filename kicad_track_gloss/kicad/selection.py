"""Native KiCad connection expansion."""

from __future__ import annotations

from collections import deque

from ..engine.model import segment_key
from .authority import protected_track_keys, protection_warnings
from .types import is_straight_track, is_via


def _touches_anchor(pcbnew, item, anchor):
    points = ((item.GetPosition(),) if is_via(pcbnew, item)
              else (item.GetStart(), item.GetEnd()))
    return any(point.x == anchor.x and point.y == anchor.y for point in points)


def expand_seed_scopes(adapter, board, straight_by_key, seed_keys, warnings):
    """Return each distinct seed-local KiCad connection independently.

    Keeping these scopes is important after a combined native DRC rejection:
    the plugin can retry the same bounded connection a user would obtain by
    selecting one of its segments, rather than treating a complete net as the
    smallest recoverable unit.
    """
    if not seed_keys:
        return ()
    connectivity = board.GetConnectivity()

    protected = protected_track_keys(adapter.pcbnew, board, straight_by_key)
    scopes = set()
    for seed_key in sorted(seed_keys):
        queue = deque([seed_key])
        visited = set()
        expanded = set()
        while queue:
            key = queue.popleft()
            if key in visited:
                continue
            visited.add(key)
            expanded.add(key)
            item, source_segment = straight_by_key[key]
            native_neighbors = list(connectivity.GetConnectedTracks(item))
            native_pads = list(connectivity.GetConnectedPads(item))

            for anchor in (item.GetStart(), item.GetEnd()):
                pads_here = []
                for pad in native_pads:
                    if pad.HitTest(anchor):
                        pads_here.append(pad)
                if pads_here:
                    continue

                touching = [neighbor for neighbor in native_neighbors
                            if _touches_anchor(adapter.pcbnew, neighbor, anchor)]
                if len(touching) != 1:
                    continue
                neighbor = touching[0]
                if not is_straight_track(adapter.pcbnew, neighbor):
                    continue
                neighbor_segment = adapter.segment_from_item(neighbor)
                neighbor_key = segment_key(neighbor_segment)
                if (neighbor_key not in straight_by_key or
                        neighbor_key in protected or
                        neighbor_segment.net_id != source_segment.net_id):
                    continue
                if neighbor_key not in visited:
                    queue.append(neighbor_key)
        if expanded:
            scopes.add(frozenset(expanded))
    return tuple(sorted(scopes, key=lambda scope: tuple(sorted(scope))))


def expand_seed_keys(adapter, board, straight_by_key, seed_keys, warnings):
    """Expand every selected seed to its KiCad connection, up to boundaries."""
    scopes = expand_seed_scopes(
        adapter, board, straight_by_key, seed_keys, warnings)
    return set().union(*scopes) if scopes else set()


def expand_eligible_keys(adapter, board, straight_by_key, seed_keys, warnings=None):
    warnings = warnings if warnings is not None else []
    expanded = expand_seed_keys(
        adapter, board, straight_by_key, set(seed_keys), warnings)
    protected = protected_track_keys(adapter.pcbnew, board, straight_by_key)
    protected_expanded = expanded & set(protected)
    warnings.extend(protection_warnings({
        key: protected[key] for key in protected_expanded}))
    return expanded - protected_expanded, expanded, protected_expanded


def expand_eligible_scopes(
        adapter, board, straight_by_key, seed_keys, warnings=None):
    """Return eligible union plus the distinct protected connection scopes."""
    warnings = warnings if warnings is not None else []
    scopes = expand_seed_scopes(
        adapter, board, straight_by_key, set(seed_keys), warnings)
    expanded = set().union(*scopes) if scopes else set()
    protected = protected_track_keys(adapter.pcbnew, board, straight_by_key)
    protected_expanded = expanded & set(protected)
    eligible = expanded - protected_expanded
    eligible_scopes = tuple(
        frozenset(scope - protected_expanded)
        for scope in scopes if scope - protected_expanded)
    warnings.extend(protection_warnings({
        key: protected[key] for key in protected_expanded}))
    return eligible, expanded, protected_expanded, eligible_scopes
