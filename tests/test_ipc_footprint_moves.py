#!/usr/bin/env python3
"""IPC adapter applies decoupling-cap footprint moves in one commit (issue #130).

The cap-placement engine (placement.fanout_clearance) is pcbnew-free and returns
{reference,new_x,new_y,new_rotation} placements; the IPC plugin applies them via
kicad_ipc_adapter.apply_footprint_moves -- the kipy analogue of the SWIG GUI's
FindFootprintByReference / SetPosition / SetOrientationDegrees. This exercises
that helper with a mock board (real kipy Vector2/Angle), no running KiCad.

    python3 tests/test_ipc_footprint_moves.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import kicad_ipc_adapter as adapter


class _FP:
    def __init__(self, ref):
        self.reference = ref
        self.position = None
        self.orientation = None


class _Board:
    """Minimal kipy-board stand-in: records the commit + the updated items."""
    def __init__(self, fps):
        self._fps = fps
        self.updated = None
        self.pushed = None

    def get_footprints(self):
        return list(self._fps)

    def get_nets(self):
        return []

    def begin_commit(self):
        return "handle"

    def update_items(self, items):
        self.updated = list(items)

    def create_items(self, items):
        pass

    def remove_items(self, items):
        pass

    def push_commit(self, handle, message):
        self.pushed = message

    def drop_commit(self, handle):
        self.pushed = None


def test_moves_applied():
    c1, c2, u9 = _FP("C1"), _FP("C2"), _FP("U9")
    board = _Board([c1, c2, u9])
    placements = [
        {"reference": "C1", "new_x": 10.0, "new_y": 20.0, "new_rotation": 90.0},
        {"reference": "C2", "new_x": -3.5, "new_y": 4.25, "new_rotation": 0.0},
        {"reference": "C9", "new_x": 0.0, "new_y": 0.0, "new_rotation": 0.0},  # absent
    ]
    moved = adapter.apply_footprint_moves(board, placements)
    assert moved == 2, moved
    # positions set in mm (read back via the adapter's Vector2 reader)
    x1, y1 = adapter._vec_xy_mm(c1.position)
    x2, y2 = adapter._vec_xy_mm(c2.position)
    assert abs(x1 - 10.0) < 1e-6 and abs(y1 - 20.0) < 1e-6, (x1, y1)
    assert abs(x2 - (-3.5)) < 1e-6 and abs(y2 - 4.25) < 1e-6, (x2, y2)
    # orientation set in degrees (read back via kipy Angle)
    assert abs(c1.orientation.degrees - 90.0) < 1e-6, c1.orientation.degrees
    # untouched footprint stays put; absent ref ignored
    assert u9.position is None and u9.orientation is None
    # both moved footprints queued in a single commit
    assert board.updated is not None and len(board.updated) == 2
    assert c1 in board.updated and c2 in board.updated
    assert board.pushed and "cap placement" in board.pushed
    print("PASS moves_applied")


def test_empty_placements_noop():
    board = _Board([_FP("C1")])
    assert adapter.apply_footprint_moves(board, []) == 0
    assert board.updated is None and board.pushed is None
    print("PASS empty_placements_noop")


if __name__ == "__main__":
    test_moves_applied()
    test_empty_placements_noop()
    print("\nAll IPC footprint-move tests passed.")
