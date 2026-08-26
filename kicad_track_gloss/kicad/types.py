"""Semantic KiCad item-type predicates backed by pcbnew classes."""

from __future__ import annotations


def is_via(pcbnew, item):
    try:
        return int(item.Type()) == int(pcbnew.PCB_VIA_T)
    except (AttributeError, TypeError, ValueError):
        pass
    try:
        return isinstance(item, pcbnew.PCB_VIA)
    except (AttributeError, TypeError):
        return False


def is_arc(pcbnew, item):
    try:
        return int(item.Type()) == int(pcbnew.PCB_ARC_T)
    except (AttributeError, TypeError, ValueError):
        pass
    try:
        return isinstance(item, pcbnew.PCB_ARC)
    except (AttributeError, TypeError):
        return False


def is_straight_track(pcbnew, item):
    try:
        return int(item.Type()) == int(pcbnew.PCB_TRACE_T)
    except (AttributeError, TypeError, ValueError):
        pass
    try:
        return (isinstance(item, pcbnew.PCB_TRACK) and
                not is_via(pcbnew, item) and not is_arc(pcbnew, item))
    except (AttributeError, TypeError):
        return False


def is_shape(pcbnew, item):
    try:
        return int(item.Type()) == int(pcbnew.PCB_SHAPE_T)
    except (AttributeError, TypeError, ValueError):
        pass
    try:
        if isinstance(item, pcbnew.PCB_SHAPE):
            return True
    except (AttributeError, TypeError):
        pass
    try:
        return isinstance(item, pcbnew.FP_SHAPE)
    except (AttributeError, TypeError):
        return False
