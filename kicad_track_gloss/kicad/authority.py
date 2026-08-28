"""KiCad 10 native edit-authority classification.

Geometry never implies user intent here.  A straight track is protected only
when KiCad itself marks the item (or one of its owning objects) as special.
"""

from __future__ import annotations


LOCKED_ITEM = "locked_item"
LOCKED_GROUP = "locked_group"
GENERATED = "generated"
DIFF_PAIR = "diff_pair"


def _parent_authority(pcbnew, item):
    parent = item.GetParentGroup()
    while parent is not None:
        parent_item = parent.AsEdaItem()
        if int(parent_item.Type()) == int(pcbnew.PCB_GENERATOR_T):
            return GENERATED
        if parent_item.IsLocked():
            return LOCKED_GROUP
        parent = parent_item.GetParentGroup()
    return None


def native_authority(pcbnew, board, item):
    """Return the KiCad authority protecting ``item``, or ``None``.

    The implementation intentionally targets KiCad 10 directly.  Missing API
    is an integration error, not permission to infer intent from geometry or
    net names.
    """
    if item.IsLocked():
        return LOCKED_ITEM

    parent_reason = _parent_authority(pcbnew, item)
    if parent_reason is not None:
        return parent_reason

    net = item.GetNet()
    if net is not None and board.DpCoupledNet(net) is not None:
        return DIFF_PAIR

    return None


def protected_track_keys(pcbnew, board, straight_by_key):
    """Return native protection reasons indexed by engine track identity."""
    return {
        key: reason
        for key, (item, _segment) in straight_by_key.items()
        for reason in (native_authority(pcbnew, board, item),)
        if reason is not None
    }


def protection_warnings(reasons):
    counts = {reason: 0 for reason in set(reasons.values())}
    for reason in reasons.values():
        counts[reason] += 1
    labels = {
        LOCKED_ITEM: "KiCad-locked track segments are protected",
        LOCKED_GROUP: "Track segments in KiCad-locked groups are protected",
        GENERATED: "KiCad-generated track segments are protected",
        DIFF_PAIR: "KiCad-recognized differential-pair track segments are protected",
    }
    return ["{} ({}).".format(labels[reason], counts[reason])
            for reason in sorted(counts)]


__all__ = ("native_authority", "protected_track_keys", "protection_warnings")
