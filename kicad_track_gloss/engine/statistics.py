"""Plan classification and diagnostic aggregation."""

from __future__ import annotations

from collections import defaultdict
import statistics

from .geometry import length
from .model import Transformation, segment_key


MECHANISM_LABELS = {
    "fixed_endpoints": "Fixed endpoints / geometric routing",
    "track_slide": "Sliding track-intersection termination",
    "pad_slide": "Sliding pad-area termination",
}

GEOMETRY_LABELS = {
    "equal_length_merge": "Equal-length segment merge",
    "dogleg_removal": "Dogleg removal",
    "staircase_shortcut": "Staircase shortcut",
    "corner_chamfer": "Local corner chamfer",
    "corner_relocation": "Corner or endpoint relocation",
    "segment_simplification": "Segment-count simplification",
    "octolinear_shortcut": "Octolinear shortcut",
}

SEARCH_LABELS = {
    "paths_evaluated": "Octolinear paths evaluated",
    "not_improving": "Rejected: insufficient length/complexity gain",
    "board_edge": "Rejected: board edge",
    "foreign_track_clearance": "Rejected: foreign-track clearance",
    "pad_clearance": "Rejected: pad clearance",
    "via_clearance": "Rejected: via clearance",
    "keepout": "Rejected: track keepout",
    "accepted_options": "Safe candidate options",
}


def classify_transformation(segments, path, mechanism, equal_tolerance=0.000001,
                            after_segments=None):
    before_mm = sum(length((segment.start_x, segment.start_y),
                           (segment.end_x, segment.end_y))
                    for segment in segments)
    path_edges = [(a, b) for a, b in zip(path, path[1:])
                  if length(a, b) > equal_tolerance]
    after_mm = sum(length(a, b) for a, b in path_edges)
    before_count = len(segments)
    after_count = len(path_edges) if after_segments is None else after_segments
    gain = before_mm - after_mm
    if abs(gain) <= equal_tolerance and after_count < before_count:
        geometry = "equal_length_merge"
    elif before_count == 2 and after_count == 1:
        geometry = "dogleg_removal"
    elif before_count >= 3 and after_count < before_count:
        geometry = "staircase_shortcut"
    elif before_count == 2 and after_count == 3 and gain > equal_tolerance:
        geometry = "corner_chamfer"
    elif before_count == after_count:
        geometry = "corner_relocation"
    elif after_count < before_count:
        geometry = "segment_simplification"
    else:
        geometry = "octolinear_shortcut"
    sample = segments[0]
    return Transformation(
        mechanism, geometry, sample.net_id, sample.net_name, sample.layer,
        sample.width, before_mm, after_mm, before_count, after_count)


def _category_rows(transformations, attribute, labels):
    grouped = defaultdict(list)
    for transformation in transformations:
        grouped[getattr(transformation, attribute)].append(transformation)
    rows = []
    for key, values in grouped.items():
        net_gain = sum(value.net_gain_mm for value in values)
        rows.append({
            "key": key,
            "label": labels.get(key, key),
            "count": len(values),
            "saved_mm": max(0.0, net_gain),
            "net_gain_mm": net_gain,
            "segments_saved": sum(value.before_segments - value.after_segments
                                  for value in values),
        })
    return sorted(rows, key=lambda row: (-row["net_gain_mm"], row["key"]))


def summarize_plan(model, eligible_keys, plan):
    """Return API-neutral statistics for one chosen immutable edit plan."""
    eligible = set(eligible_keys)
    scoped = [segment for segment in model.segments
              if segment_key(segment) in eligible]
    before_mm = sum(length((segment.start_x, segment.start_y),
                           (segment.end_x, segment.end_y))
                    for segment in scoped)
    removed = set(plan.remove_keys)
    removed_mm = sum(length((segment.start_x, segment.start_y),
                            (segment.end_x, segment.end_y))
                     for segment in scoped if segment_key(segment) in removed)
    added_mm = sum(length(addition.start, addition.end)
                   for addition in plan.additions)
    after_mm = before_mm - removed_mm + added_mm
    saved_mm = max(0.0, before_mm - after_mm)
    gains = [transformation.net_gain_mm for transformation in plan.transformations]
    by_net = defaultdict(
        lambda: {"count": 0, "saved_mm": 0.0, "net_gain_mm": 0.0})
    for transformation in plan.transformations:
        label = transformation.net_name or "net {}".format(transformation.net_id)
        by_net[label]["count"] += 1
        by_net[label]["net_gain_mm"] += transformation.net_gain_mm
        by_net[label]["saved_mm"] = max(
            0.0, by_net[label]["net_gain_mm"])
    top_nets = sorted(
        ({"net": net, **values} for net, values in by_net.items()),
        key=lambda row: (-row["net_gain_mm"], row["net"]))
    fixed_gain = sum(t.net_gain_mm for t in plan.transformations
                     if t.mechanism == "fixed_endpoints")
    terminal_gain = sum(t.net_gain_mm for t in plan.transformations
                        if t.mechanism != "fixed_endpoints")
    return {
        "eligible_segments": len(scoped),
        "before_mm": before_mm,
        "after_mm": max(0.0, after_mm),
        "length_change_mm": after_mm - before_mm,
        "saved_mm": saved_mm,
        "saved_percent": 100.0 * saved_mm / before_mm if before_mm else 0.0,
        "segments_after": len(scoped) - len(plan.remove_keys) + len(plan.additions),
        "segments_saved": len(plan.remove_keys) - len(plan.additions),
        "segment_percent": (100.0 * (len(plan.remove_keys) - len(plan.additions)) /
                            len(scoped) if scoped else 0.0),
        "chains_changed": plan.chains_changed,
        "convergence_passes": plan.convergence_passes,
        "fixed_point": plan.fixed_point,
        "angle_corrections": plan.angle_corrections,
        "transformations": len(plan.transformations),
        "gain_mean": statistics.fmean(gains) if gains else 0.0,
        "gain_median": statistics.median(gains) if gains else 0.0,
        "gain_max": max(gains, default=0.0),
        "fixed_gain": fixed_gain,
        "terminal_gain": terminal_gain,
        "mechanisms": _category_rows(
            plan.transformations, "mechanism", MECHANISM_LABELS),
        "geometries": _category_rows(
            plan.transformations, "geometry", GEOMETRY_LABELS),
        "top_nets": top_nets,
        "search_counts": dict(plan.search_counts),
        "blocking_nets": dict(sorted(
            plan.blocking_nets.items(), key=lambda item: (-item[1], item[0]))),
    }
