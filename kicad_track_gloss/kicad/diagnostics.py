"""Text rendering for the selectable KiCad diagnostic report."""

from __future__ import annotations

import json

from ..engine.statistics import SEARCH_LABELS


def append_search_statistics(report, counts):
    report.extend(["", "Search statistics:"])
    for key in ("paths_evaluated", "not_improving", "board_edge",
                "foreign_track_clearance", "pad_clearance", "via_clearance",
                "keepout", "accepted_options"):
        value = counts.get(key, 0)
        if value or key in ("paths_evaluated", "accepted_options"):
            report.append("  {}: {}".format(SEARCH_LABELS.get(key, key), value))


def append_plan_statistics(report, summary):
    report.extend([
        "",
        "Gloss statistics:",
        "  Eligible copper: {:.6f} -> {:.6f} mm".format(
            summary["before_mm"], summary["after_mm"]),
        "  Copper saved: {:.6f} mm ({:.3f}%)".format(
            summary["saved_mm"], summary["saved_percent"]),
        "  Eligible segments: {} -> {} (net reduction: {}, {:.3f}%)".format(
            summary["eligible_segments"], summary["segments_after"],
            summary["segments_saved"], summary["segment_percent"]),
        "  Changed chains / transformations: {} / {}".format(
            summary["chains_changed"], summary["transformations"]),
        "  Gain per transformation (mean / median / max): "
        "{:.6f} / {:.6f} / {:.6f} mm".format(
            summary["gain_mean"], summary["gain_median"], summary["gain_max"]),
        "  Router-geometric gain (fixed endpoints): {:.6f} mm".format(
            summary["fixed_gain"]),
        "  Terminal-placement gain (track/pad sliding): {:.6f} mm".format(
            summary["terminal_gain"]),
        "",
        "By optimization mechanism:",
    ])
    for row in summary["mechanisms"]:
        report.append("  {}: {} transformation(s), {:.6f} mm, {} segment(s)".format(
            row["label"], row["count"], row["saved_mm"], row["segments_saved"]))
    report.extend(["", "By geometry pattern:"])
    for row in summary["geometries"]:
        report.append("  {}: {} transformation(s), {:.6f} mm, {} segment(s)".format(
            row["label"], row["count"], row["saved_mm"], row["segments_saved"]))
    report.extend(["", "Top improved nets:"])
    for row in summary["top_nets"][:10]:
        report.append("  {}: {:.6f} mm in {} transformation(s)".format(
            row["net"], row["saved_mm"], row["count"]))
    append_search_statistics(report, summary["search_counts"])
    report.extend(["", "Machine-readable JSON:"])
    report.extend(json.dumps(summary, indent=2, sort_keys=True).splitlines())
