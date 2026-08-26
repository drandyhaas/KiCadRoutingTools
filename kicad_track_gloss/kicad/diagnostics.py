"""Text rendering for the selectable KiCad diagnostic report."""

from __future__ import annotations

import json

from ..engine.statistics import SEARCH_LABELS


def split_diagnostic_report(lines):
    """Return concise result, human details, and raw JSON tab contents."""
    lines = list(lines)
    marker = "Machine-readable JSON:"
    try:
        json_index = lines.index(marker)
    except ValueError:
        detail_lines = lines
        json_lines = ["No machine-readable result is available for this run."]
        summary = []
    else:
        detail_lines = lines[:json_index]
        json_lines = lines[json_index + 1:]
        summary = []

    prefixes = (
        "Plugin version:", "KiCad version:", "Eligible net(s)",
        "Selected objects:", "Eligible straight segments:",
        "Optimization coordinates:")
    context = [line for line in detail_lines
               if line.startswith(prefixes)]

    data = None
    if json_lines and json_lines[0] != \
            "No machine-readable result is available for this run.":
        try:
            data = json.loads("\n".join(json_lines))
        except (TypeError, ValueError):
            data = None
    if data is not None:
        saved = data.get("saved_mm", 0.0)
        percent = data.get("saved_percent", 0.0)
        segments_before = data.get("eligible_segments", 0)
        segments_after = data.get("segments_after", segments_before)
        summary.extend([
            "GLOSS APPLIED",
            "",
            "Length saved: {:.6f} mm ({:.3f}%)".format(saved, percent),
            "Copper length: {:.6f} -> {:.6f} mm".format(
                data.get("before_mm", 0.0), data.get("after_mm", 0.0)),
            "Segments: {} -> {} ({} saved)".format(
                segments_before, segments_after,
                data.get("segments_saved", segments_before - segments_after)),
            "Non-octolinear segments corrected: {}".format(
                data.get("angle_corrections", 0)),
        ])
    else:
        result_lines = [line for line in detail_lines
                        if line.startswith(("Result:", "Reason:",
                                            "UNEXPECTED ERROR"))]
        summary.extend(result_lines or ["No modification was applied."])
    if context:
        summary.extend([""] + context)
    return summary, detail_lines, json_lines


def append_search_statistics(report, counts, blocking_nets=None):
    report.extend(["", "Search statistics:"])
    for key in ("paths_evaluated", "not_improving", "board_edge",
                "foreign_track_clearance", "pad_clearance", "via_clearance",
                "keepout", "accepted_options"):
        value = counts.get(key, 0)
        if value or key in ("paths_evaluated", "accepted_options"):
            report.append("  {}: {}".format(SEARCH_LABELS.get(key, key), value))
    if blocking_nets:
        report.extend(["", "Blocking nets:"])
        for name, count in sorted(
                blocking_nets.items(), key=lambda item: (-item[1], item[0])):
            report.append("  {}: {} rejected candidate(s)".format(name, count))


def append_plan_statistics(report, summary):
    report.extend([
        "",
        "Gloss statistics:",
        "  Eligible copper: {:.6f} -> {:.6f} mm".format(
            summary["before_mm"], summary["after_mm"]),
        "  Copper saved: {:.6f} mm ({:.3f}%)".format(
            summary["saved_mm"], summary["saved_percent"]),
        "  Copper length change: {:+.6f} mm".format(
            summary["length_change_mm"]),
        "  Non-octolinear segments corrected: {}".format(
            summary["angle_corrections"]),
        "  Eligible segments: {} -> {} (net reduction: {}, {:.3f}%)".format(
            summary["eligible_segments"], summary["segments_after"],
            summary["segments_saved"], summary["segment_percent"]),
        "  Changed chains / transformations: {} / {}".format(
            summary["chains_changed"], summary["transformations"]),
        "  Convergence passes / fixed point: {} / {}".format(
            summary["convergence_passes"],
            "yes" if summary["fixed_point"] else "no"),
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
            row["label"], row["count"], row["net_gain_mm"],
            row["segments_saved"]))
    report.extend(["", "By geometry pattern:"])
    for row in summary["geometries"]:
        report.append("  {}: {} transformation(s), {:.6f} mm, {} segment(s)".format(
            row["label"], row["count"], row["net_gain_mm"],
            row["segments_saved"]))
    report.extend(["", "Net copper gain:"])
    for row in summary["top_nets"][:10]:
        report.append("  {}: {:.6f} mm in {} transformation(s)".format(
            row["net"], row["net_gain_mm"], row["count"]))
    append_search_statistics(
        report, summary["search_counts"], summary["blocking_nets"])
    timings = summary.get("timings_ms", {})
    if timings:
        report.extend(["", "Performance timings:"])
        labels = {
            "selection_scan": "Selection scan",
            "snapshot": "Board snapshot",
            "terminal_analysis": "Terminal analysis",
            "planning": "Gloss planning",
            "native_drc_gate": "Native DRC gate (wall time)",
            "native_snapshot": "Native board snapshot",
            "native_candidate_snapshot": "Candidate construction",
            "native_before_drc": "Baseline DRC process",
            "native_after_drc": "Candidate DRC process",
            "native_total": "Native validation total",
            "native_cache_lookup": "Native validation cache lookup",
            "apply": "Apply to current board",
            "total": "Total operation",
        }
        for key, value in timings.items():
            report.append("  {}: {:.3f} ms".format(
                labels.get(key, key.replace("_", " ").title()), value))
        report.append("  Baseline DRC cache: {}".format(
            "hit" if summary.get("native_baseline_cached") else "miss"))
        report.append("  Validation mode: {}".format(
            summary.get("validation_mode", "native_parallel")))
    report.extend(["", "Machine-readable JSON:"])
    report.extend(json.dumps(summary, indent=2, sort_keys=True).splitlines())
