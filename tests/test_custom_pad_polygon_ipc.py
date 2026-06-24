#!/usr/bin/env python3
"""IPC/kipy parser path carries a custom pad's real copper polygon (issue #188).

The text parser populates Pad.polygons via _custom_pad_global_polygons; the
kipy builder must reach parity by reading the rendered shape from the live board
through board.get_pad_shapes_as_polygons (the IPC analogue of pcbnew's
GetEffectivePolygon). Without a running KiCad we exercise the extraction helpers
with mock kipy objects mirroring the PolygonWithHoles / PolyLine / node shape.

    python3 tests/test_custom_pad_polygon_ipc.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import (
    _kipy_pad_polygon_points, _attach_custom_pad_polygons, Pad,
)


# --- Mock kipy geometry -----------------------------------------------------
class _Vec:
    def __init__(self, x_mm, y_mm):
        self.x_mm = x_mm
        self.y_mm = y_mm


class _Node:
    def __init__(self, x_mm, y_mm, has_point=True):
        self.has_point = has_point
        self.point = _Vec(x_mm, y_mm)


class _PolyLine:
    def __init__(self, pts):
        self.nodes = [_Node(x, y) for x, y in pts]


class _PWH:
    def __init__(self, pts):
        self.outline = _PolyLine(pts)


class _Pad:
    def __init__(self, tag):
        self.tag = tag


class _Board:
    """Fake board: maps (pad, layer) -> PolygonWithHoles via a per-layer table."""
    def __init__(self, table):
        # table: {layer_int: {pad_tag: pts_or_None}}
        self._table = table
        self.calls = []

    def get_pad_shapes_as_polygons(self, pads, layer=3):
        self.calls.append((tuple(p.tag for p in pads), layer))
        out = []
        for p in pads:
            pts = self._table.get(layer, {}).get(p.tag)
            out.append(_PWH(pts) if pts else None)
        return out


def _mk_pad(shape='custom'):
    return Pad(component_ref='U1', pad_number='1', global_x=0, global_y=0,
               local_x=0, local_y=0, size_x=1, size_y=1, shape=shape,
               layers=['F.Cu'], net_id=1, net_name='/N')


def test_extract_points():
    sq = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    out = _kipy_pad_polygon_points(_PWH(sq))
    assert out == [sq], out
    # too few points / None -> None (bbox fallback)
    assert _kipy_pad_polygon_points(_PWH([(0, 0), (1, 1)])) is None
    assert _kipy_pad_polygon_points(None) is None
    print("PASS extract_points")


def test_attach_front_layer():
    from kipy.board_types import BoardLayer
    f = int(BoardLayer.BL_F_Cu)
    comb = [(0, 0), (5, 0), (5, 4), (4, 4), (4, 1), (0, 1)]
    pad = _mk_pad()
    board = _Board({f: {'kA': comb}})
    _attach_custom_pad_polygons(board, [(pad, _Pad('kA'))])
    assert pad.polygons == [comb], pad.polygons
    # resolved on the first (F.Cu) call, so no B.Cu retry
    assert board.calls[0][1] == f
    print("PASS attach_front_layer")


def test_attach_back_layer_retry():
    from kipy.board_types import BoardLayer
    f, b = int(BoardLayer.BL_F_Cu), int(BoardLayer.BL_B_Cu)
    tri = [(0, 0), (3, 0), (0, 3)]
    pad = _mk_pad()
    pad.layers = ['B.Cu']
    board = _Board({b: {'kB': tri}})  # nothing on F.Cu -> must retry B.Cu
    _attach_custom_pad_polygons(board, [(pad, _Pad('kB'))])
    assert pad.polygons == [tri], pad.polygons
    assert [c[1] for c in board.calls] == [f, b], board.calls
    print("PASS attach_back_layer_retry")


def test_attach_noop_on_missing_method():
    class _NoMethod:
        pass
    pad = _mk_pad()
    _attach_custom_pad_polygons(_NoMethod(), [(pad, _Pad('x'))])
    assert pad.polygons is None  # bbox model left in place, no crash
    print("PASS attach_noop_on_missing_method")


def test_attach_noop_on_ipc_error():
    class _Boom:
        def get_pad_shapes_as_polygons(self, pads, layer=3):
            raise RuntimeError("socket")
    pad = _mk_pad()
    _attach_custom_pad_polygons(_Boom(), [(pad, _Pad('x'))])
    assert pad.polygons is None
    print("PASS attach_noop_on_ipc_error")


if __name__ == "__main__":
    test_extract_points()
    test_attach_front_layer()
    test_attach_back_layer_retry()
    test_attach_noop_on_missing_method()
    test_attach_noop_on_ipc_error()
    print("\nAll IPC custom-pad polygon tests passed.")
