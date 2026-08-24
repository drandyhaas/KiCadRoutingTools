"""Native KiCad connection expansion and protected-track classification."""

from __future__ import annotations

from collections import deque
import math
import re

from ..engine.model import segment_key


def is_probable_diff_pair(name):
    return bool(re.search(r"(?:[_+.-](?:P|N)|[+-])$", name, re.IGNORECASE))


def meander_keys(segments):
    """Detect tuning geometry in each independent, unbranched track chain.

    Besides obvious direction reversals, KiCad length tuning can leave long
    runs of tiny monotonic jogs.  Passing such a run to the geometric planner
    is both unsafe (it destroys intentional tuning) and disproportionately
    expensive, so classify the whole connected run as protected.
    """
    by_group = {}
    for seg in segments:
        by_group.setdefault((seg.net_id, seg.layer, round(seg.width, 6)), []).append(seg)
    protected = set()
    for group in by_group.values():
        adjacency = {}
        by_key = {segment_key(seg): seg for seg in group}
        for seg in group:
            for point in ((round(seg.start_x, 6), round(seg.start_y, 6)),
                          (round(seg.end_x, 6), round(seg.end_y, 6))):
                adjacency.setdefault(point, []).append(seg)
        remaining = set(by_key)
        while remaining:
            component = set()
            pending = [min(remaining)]
            while pending:
                key = pending.pop()
                if key in component:
                    continue
                component.add(key)
                seg = by_key[key]
                for point in ((round(seg.start_x, 6), round(seg.start_y, 6)),
                              (round(seg.end_x, 6), round(seg.end_y, 6))):
                    pending.extend(segment_key(other) for other in adjacency[point]
                                   if segment_key(other) not in component)
            remaining.difference_update(component)

            lengths = sorted(
                math.hypot(by_key[key].end_x - by_key[key].start_x,
                           by_key[key].end_y - by_key[key].start_y)
                for key in component)
            dense_micro_jogs = (
                len(lengths) >= 32 and
                lengths[len(lengths) // 2] <= 0.05 + 1e-9 and
                sum(value <= 0.1 + 1e-9 for value in lengths) * 10 >=
                len(lengths) * 7
            )
            if dense_micro_jogs:
                protected.update(component)
                continue

            component_adjacency = {
                point: sorted((seg for seg in touching
                               if segment_key(seg) in component), key=segment_key)
                for point, touching in adjacency.items()
            }
            component_adjacency = {point: touching
                                   for point, touching in component_adjacency.items()
                                   if touching}
            starts = sorted(point for point, touching in component_adjacency.items()
                            if len(touching) == 1)
            if len(starts) != 2 or any(len(touching) > 2
                                       for touching in component_adjacency.values()):
                continue

            used, vectors = set(), []
            point = starts[0]
            while True:
                options = [seg for seg in component_adjacency.get(point, ())
                           if segment_key(seg) not in used]
                if not options:
                    break
                seg = options[0]
                used.add(segment_key(seg))
                start = (round(seg.start_x, 6), round(seg.start_y, 6))
                if start == point:
                    point = (round(seg.end_x, 6), round(seg.end_y, 6))
                    vectors.append((seg.end_x - seg.start_x, seg.end_y - seg.start_y))
                else:
                    point = start
                    vectors.append((seg.start_x - seg.end_x, seg.start_y - seg.end_y))
            reversals = sum(
                vectors[i][0] * vectors[i + 2][0] +
                vectors[i][1] * vectors[i + 2][1] < -1e-9
                for i in range(len(vectors) - 2))
            if reversals >= 2:
                protected.update(component)
    return protected


def _touches_anchor(item, anchor):
    points = ((item.GetPosition(),) if str(item.GetClass()) == "PCB_VIA"
              else (item.GetStart(), item.GetEnd()))
    return any(point.x == anchor.x and point.y == anchor.y for point in points)


def expand_seed_keys(adapter, board, straight_by_key, seed_keys, warnings):
    """Expand every selected seed to its KiCad connection, up to boundaries."""
    if not seed_keys:
        return set()
    try:
        connectivity = board.GetConnectivity()
    except Exception:
        warnings.append("KiCad connectivity is unavailable; selection was not expanded.")
        return set(seed_keys)

    expanded = set()
    for seed_key in sorted(seed_keys):
        queue = deque([seed_key])
        visited = set()
        while queue:
            key = queue.popleft()
            if key in visited:
                continue
            visited.add(key)
            expanded.add(key)
            item, source_segment = straight_by_key[key]
            try:
                native_neighbors = list(connectivity.GetConnectedTracks(item))
                native_pads = list(connectivity.GetConnectedPads(item))
            except Exception:
                warnings.append(
                    "KiCad connectivity query failed; expansion stopped at a selected seed.")
                continue

            for anchor in (item.GetStart(), item.GetEnd()):
                pads_here = []
                for pad in native_pads:
                    try:
                        if pad.HitTest(anchor):
                            pads_here.append(pad)
                    except Exception:
                        position = pad.GetPosition()
                        if position.x == anchor.x and position.y == anchor.y:
                            pads_here.append(pad)
                if pads_here:
                    continue

                touching = [neighbor for neighbor in native_neighbors
                            if _touches_anchor(neighbor, anchor)]
                if len(touching) != 1:
                    continue
                neighbor = touching[0]
                if str(neighbor.GetClass()) != "PCB_TRACK":
                    continue
                try:
                    neighbor_segment = adapter.segment_from_item(neighbor)
                except Exception:
                    continue
                neighbor_key = segment_key(neighbor_segment)
                if (neighbor_key not in straight_by_key or neighbor_segment.locked or
                        is_probable_diff_pair(neighbor_segment.net_name) or
                        neighbor_segment.net_id != source_segment.net_id):
                    continue
                if neighbor_key not in visited:
                    queue.append(neighbor_key)
    return expanded


def expand_eligible_keys(adapter, board, straight_by_key, seed_keys, warnings=None):
    warnings = warnings if warnings is not None else []
    expanded = expand_seed_keys(
        adapter, board, straight_by_key, set(seed_keys), warnings)
    meanders = meander_keys([
        segment for _item, segment in straight_by_key.values()
        if segment_key(segment) in expanded
    ])
    eligible = expanded - meanders
    if meanders:
        warnings.append(
            "Probable meander/dense micro-jog length-tuning tracks are protected.")
    return eligible, expanded, meanders
