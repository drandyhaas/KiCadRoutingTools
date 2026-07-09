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


def update_live_drc_floors(board, *, clearance=None, track_width=None,
                           via_size=None, via_drill=None,
                           hole_to_hole=None, edge_clearance=None,
                           log=None):
    """Per-step live DRC floor update -- the GUI twin of the CLI's
    per-step fix_project_for_output (#160). KiCad holds project settings in
    memory, so only the design-settings API affects a DRC run right after a
    step. Two rules, both mirroring the CLI:

    * values only ever RELAX downward (min-merge, like the clearance
      ledger);
    * constraints are clamped to the board's ACTUAL minima (the CLI's
      scan_board_minima step): fine-pitch taps route below the step's
      nominal width/via floor, and a floor of 0.127 still flags a
      legitimate 0.089 tap -- 48 of the 51 'violations' in Andy's bitaxe
      DRC.rpt were this class.

    Best-effort: never raises."""
    try:
        import pcbnew
        bds = board.GetDesignSettings()

        # Actual board minima (copper tracks/vias only).
        min_w = min_via = min_drill = min_ann = None
        for t in board.GetTracks():
            try:
                if t.Type() == pcbnew.PCB_VIA_T:
                    try:
                        w = t.GetWidth()
                    except Exception:
                        w = t.GetFrontWidth()
                    d = t.GetDrillValue()
                    min_via = w if min_via is None else min(min_via, w)
                    min_drill = d if min_drill is None else min(min_drill, d)
                    ann = (w - d) // 2
                    min_ann = ann if min_ann is None else min(min_ann, ann)
                else:
                    w = t.GetWidth()
                    min_w = w if min_w is None else min(min_w, w)
            except Exception:
                continue

        def lower(attr, mm, board_min_iu=None):
            if mm is None and board_min_iu is None:
                return
            iu = pcbnew.FromMM(float(mm)) if mm is not None else None
            if board_min_iu is not None:
                iu = board_min_iu if iu is None else min(iu, board_min_iu)
            if getattr(bds, attr) > iu:
                setattr(bds, attr, iu)
        lower('m_MinClearance', clearance)
        lower('m_TrackMinWidth', track_width, min_w)
        lower('m_ViasMinSize', via_size, min_via)
        lower('m_MinThroughDrill', via_drill, min_drill)
        lower('m_ViasMinAnnularWidth',
              (float(via_size) - float(via_drill)) / 2
              if via_size and via_drill else None, min_ann)
        lower('m_HoleToHoleMin', hole_to_hole)
        # CLI parity: route.py always passes board_edge_clearance (default
        # 0.0), so CLI-fixed projects never flag pre-existing near-edge
        # mounting holes; mirror that unless a step chose a real value.
        lower('m_CopperEdgeClearance',
              edge_clearance if edge_clearance is not None else 0.0)
        try:
            for _name, _nc in board.GetNetClasses().items():
                if _name != 'Default':
                    continue
                def _nc_lower(get, set_, mm, board_min_iu=None):
                    if mm is None and board_min_iu is None:
                        return
                    iu = pcbnew.FromMM(float(mm)) if mm is not None else None
                    if board_min_iu is not None:
                        iu = board_min_iu if iu is None else min(iu, board_min_iu)
                    if get() > iu:
                        set_(iu)
                _nc_lower(_nc.GetClearance, _nc.SetClearance, clearance)
                _nc_lower(_nc.GetTrackWidth, _nc.SetTrackWidth,
                          track_width, min_w)
                _nc_lower(_nc.GetViaDiameter, _nc.SetViaDiameter,
                          via_size, min_via)
                _nc_lower(_nc.GetViaDrill, _nc.SetViaDrill,
                          via_drill, min_drill)
        except Exception:
            pass
        if log:
            log("Live DRC floors relaxed to this step's routed values "
                "(clamped to actual board minima)\n")
    except Exception as e:
        if log:
            log(f"(live DRC floor update skipped: {e})\n")
