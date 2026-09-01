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


# --- board TEXT builders (#726) ----------------------------------------------
#
# The dataclass builders above make a PCBData directly, which is the wrong
# tool for anything about PARSING or WRITING: those act on .kicad_pcb text and
# the whole question is what the text becomes. `tests/test_457_writer_precision`
# grew its own `_board`/`_fp` pair for that; these are the shared version, and
# they differ in the one way #726 needs -- a block's uuid is independent of its
# reference, so two blocks can legitimately claim one reference (or one uuid).

def footprint_text(ref, x=0.0, y=0.0, *, rot=None, uuid=None, name='test:FP',
                   layer='F.Cu', pads=1, pad_net=0, locked=False,
                   ref_property=True, extra=''):
    """One `(footprint ...)` block as KiCad writes it.

    `ref=None` emits NO Reference node at all (the reference-LESS footprint:
    a locked NPTH drill dot, of which thunderscope has 86). `ref=''` emits an
    EMPTY one, which is what esp_prog's three logo blocks carry -- the parser's
    `[^"]+` treats both the same way, and that is deliberate.

    `ref_property=False` emits the KiCad 6/7 `(fp_text reference ...)` form
    instead of the 8+ `(property "Reference" ...)` one. No tracked corpus board
    uses it, so it can only be covered synthetically (#78).
    """
    uid = uuid if uuid is not None else 'uuid-%s' % (ref or 'none')
    at = '(at %s %s%s)' % (x, y, '' if rot is None else ' ' + str(rot))
    if ref is None:
        refnode = ''
    elif ref_property:
        refnode = '\t\t(property "Reference" "%s"\n\t\t\t(at 0 0)\n\t\t)\n' % ref
    else:
        refnode = ('\t\t(fp_text reference "%s"\n\t\t\t(at 0 0)\n'
                   '\t\t\t(layer "F.SilkS")\n\t\t)\n' % ref)
    body = ['\t(footprint "%s"\n\t\t(layer "%s")\n\t\t(uuid "%s")\n\t\t%s\n'
            % (name, layer, uid, at)]
    if locked:
        body.append('\t\t(locked yes)\n')
    body.append(refnode)
    body.append(extra)
    for i in range(pads):
        body.append('\t\t(pad "%d" smd rect\n\t\t\t(at %s 0)\n\t\t\t'
                    '(size 0.6 0.8)\n\t\t\t(layers "F.Cu")\n\t\t\t'
                    '(net %d "N%d")\n\t\t\t(uuid "%s-p%d")\n\t\t)\n'
                    % (i + 1, 0.5 + i, pad_net, pad_net, uid, i + 1))
    body.append('\t)\n')
    return ''.join(body)


def board_text(footprints, *, version=20241229, nets=(0,)):
    """A minimal but REAL .kicad_pcb around `footprints` (a text blob)."""
    head = ['(kicad_pcb\n\t(version %d)\n\t(generator "synth")\n' % version]
    for n in nets:
        head.append('\t(net %d "%s")\n' % (n, '' if n == 0 else 'N%d' % n))
    head.append('\t(gr_rect\n\t\t(start 0 0)\n\t\t(end 200 200)\n'
                '\t\t(layer "Edge.Cuts")\n\t\t(uuid "edge")\n\t)\n')
    return ''.join(head) + footprints + ')\n'


def write_board(text, path=None):
    """`board_text(...)` on disk; returns the path. Caller cleans up."""
    import tempfile
    if path is None:
        fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
        os.close(fd)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path
