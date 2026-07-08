"""
KiCad Routing Tools - GUI Utilities

Shared utilities for the plugin GUI.
"""


class StdoutRedirector:
    """Redirects stdout to a callback function while preserving original output."""

    def __init__(self, callback, original_stdout):
        self.callback = callback
        self.original = original_stdout

    def write(self, text):
        if text:
            # Write to original stdout. Guard against a non-ASCII glyph (arrows,
            # Ω, ...) that a cp1252 Windows console can't encode, so a log line
            # never crashes the run (issue #152, GUI analog of route.py's UTF-8
            # reconfigure).
            if self.original:
                try:
                    self.original.write(text)
                except UnicodeEncodeError:
                    enc = getattr(self.original, 'encoding', None) or 'ascii'
                    self.original.write(text.encode(enc, errors='replace').decode(enc, errors='replace'))
            # Also send to callback (the wx log control handles Unicode fine)
            self.callback(text)

    def flush(self):
        if self.original:
            self.original.flush()


def move_copper_graphics_to_silkscreen_board(board):
    """Move copper graphic shapes (logos / artwork drawn as polys, lines, arcs,
    circles, rects, curves) from F.Cu/B.Cu to the matching silkscreen layer on the
    live pcbnew board.

    Mirrors kicad_writer.move_copper_graphics_to_silkscreen so the GUI matches CLI
    output (issue #146): a net-less copper graphic is not modelled as a router
    obstacle, so plane pours and routed copper run straight over it and short
    against it. Relocating it to silkscreen preserves it visually while taking it
    out of copper.

    Walks both board-level drawings AND footprint graphics - a copper logo is
    frequently a footprint fp_poly (e.g. the orangecrab OSHW logo). Footprint text
    and board text are handled separately by the copper-text mover, so only
    PCB_SHAPE items are touched here. Returns the number of shapes moved.
    """
    import pcbnew

    moved = 0

    def _relocate(item):
        nonlocal moved
        # PCB_SHAPE covers gr_line/poly/arc/circle/rect/curve, and footprint
        # graphics (unified onto PCB_SHAPE in modern KiCad).
        if not isinstance(item, pcbnew.PCB_SHAPE):
            return
        # #337: a NET-TIED copper graphic is functional copper the router models
        # as an immutable obstacle -- LEAVE it (moving it deletes a real
        # connection). Only net-less decoration (a copper logo) is relocated.
        # Mirrors the net guard in kicad_writer.move_copper_graphics_to_silkscreen.
        try:
            if item.GetNetCode() > 0:
                return
        except Exception:
            pass  # older PCB_SHAPE without a net accessor: treat as net-less
        layer = item.GetLayer()
        if layer == pcbnew.F_Cu:
            item.SetLayer(pcbnew.F_SilkS)
            moved += 1
        elif layer == pcbnew.B_Cu:
            item.SetLayer(pcbnew.B_SilkS)
            moved += 1

    for drawing in board.GetDrawings():
        _relocate(drawing)
    for footprint in board.GetFootprints():
        for item in footprint.GraphicalItems():
            _relocate(item)
    return moved
