"""#382 E7: canonical synthetic-object builders for tests.

~28 test scripts hand-roll their own `_pad()` / `_seg()` / `_via()` / `_pcb()`
helpers, each repeating the full dataclass constructor with slightly different
defaults. These are the ONE shared set: real kicad_parser dataclasses, sensible
defaults, everything overridable by keyword (and passthrough **kw for the long
tail of optional fields like rect_rotation, drill offsets, roundrect_rratio).

A test migrates by importing the builder it needs::

    from synth import make_pad, make_seg, make_via, make_pcb, make_net

Existing tests keep working unchanged; adopt these incrementally.
"""
from __future__ import annotations

import os
import sys

# So `import synth` and the kicad modules resolve when a test file adds tests/
# to sys.path (the existing convention) or runs from the repo root.
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS_DIR)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from kicad_parser import Pad, Segment, Via, Net, PCBData


def make_pad(net_id: int = 1, x: float = 0.0, y: float = 0.0, *,
             ref: str = 'U1', num: str = '1', net_name: str = '',
             size_x: float = 0.5, size_y: float = 0.5, shape: str = 'rect',
             layers=('F.Cu',), local_x: float = 0.0, local_y: float = 0.0,
             drill: float = 0.0, pad_type: str = 'smd', **kw) -> Pad:
    """A Pad with all required fields filled. SMD 0.5x0.5 rect on F.Cu by
    default; pass drill>0 / pad_type='thru_hole' for through-hole."""
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=local_x, local_y=local_y, size_x=size_x, size_y=size_y,
               shape=shape, layers=list(layers), net_id=net_id,
               net_name=net_name, drill=drill, pad_type=pad_type, **kw)


def make_seg(x1: float, y1: float, x2: float, y2: float, *,
             layer: str = 'F.Cu', net_id: int = 1, width: float = 0.2,
             **kw) -> Segment:
    """A track Segment (0.2 mm wide on F.Cu by default)."""
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=width,
                   layer=layer, net_id=net_id, **kw)


def make_via(x: float, y: float, *, net_id: int = 1, size: float = 0.5,
             drill: float = 0.3, layers=('F.Cu', 'B.Cu'), **kw) -> Via:
    """A through Via (0.5/0.3 spanning F.Cu-B.Cu by default)."""
    return Via(x=x, y=y, size=size, drill=drill, layers=list(layers),
               net_id=net_id, **kw)


def make_net(net_id: int, name: str = '', **kw) -> Net:
    """A Net."""
    return Net(net_id=net_id, name=name, **kw)


def make_pcb(*, nets=None, footprints=None, vias=None, segments=None,
             pads_by_net=None, board_info=None, **kw) -> PCBData:
    """A PCBData; every container defaults to empty. Pass what the test needs."""
    return PCBData(board_info=board_info, nets=nets or {},
                   footprints=footprints or {}, vias=list(vias or []),
                   segments=list(segments or []), pads_by_net=pads_by_net or {},
                   **kw)
