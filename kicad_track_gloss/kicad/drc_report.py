"""Pure parsing and comparison of KiCad JSON DRC reports."""

from __future__ import annotations

from collections import Counter
import json
import re


def normalized_text(value):
    value = re.sub(r"\blength\s+[0-9.]+\s+mm\b", "length", str(value))
    return re.sub(r"\s+", " ", value).strip()


def json_report_summary(text):
    document = json.loads(text)
    records = list(document.get("violations", ())) + list(
        document.get("unconnected_items", ()))
    counts = Counter()
    fingerprints = Counter()
    for record in records:
        kind = record.get("type", "unknown")
        counts[kind] += 1
        items = []
        for item in record.get("items", ()):
            position = item.get("pos") or {}
            items.append((
                normalized_text(item.get("description", "")),
                round(float(position.get("x", 0.0)), 3),
                round(float(position.get("y", 0.0)), 3),
            ))
        fingerprints[(kind, normalized_text(
            record.get("description", "")), tuple(sorted(items)))] += 1
    return counts, fingerprints


def json_report_counts(text):
    return json_report_summary(text)[0]


def drc_increases(before, after, before_fingerprints, after_fingerprints):
    """Return actual regressions while tolerating KiCad ratsnest relabelling."""
    new_records = after_fingerprints - before_fingerprints
    increases = Counter()
    for (kind, _description, _items), count in new_records.items():
        if kind != "unconnected_items":
            increases[kind] += count
    unconnected_delta = (
        after.get("unconnected_items", 0) -
        before.get("unconnected_items", 0))
    if unconnected_delta > 0:
        increases["unconnected_items"] = unconnected_delta
    return increases


__all__ = ("drc_increases", "json_report_counts", "json_report_summary")
