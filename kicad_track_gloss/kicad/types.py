"""Semantic KiCad item-type predicates backed by pcbnew classes."""

from __future__ import annotations


def is_via(pcbnew, item):
    return int(item.Type()) == int(pcbnew.PCB_VIA_T)


def is_arc(pcbnew, item):
    return int(item.Type()) == int(pcbnew.PCB_ARC_T)


def is_straight_track(pcbnew, item):
    return int(item.Type()) == int(pcbnew.PCB_TRACE_T)
