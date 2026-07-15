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


def refill_all_zones(board):
    """Re-fill EVERY copper zone on the board so plane pours pull back around
    copper added after they were first filled (#362).

    The plane tab fills only the zones it just created; a signal routed in a
    LATER plan step (e.g. +1V1 after the GND/+3V3 planes exist) then leaves the
    plane fill STALE -- no antipad around the new track/via -- which KiCad DRC
    flags as clearance / shorting violations on the saved board (the CLI board
    is graded with kicad-cli --refill-zones, so it never shows these). Call this
    after any apply that adds copper while filled zones exist. Best-effort.
    Returns the number of zones refilled (0 if none / on error)."""
    try:
        import pcbnew
        zones = list(board.Zones())
        if not zones:
            return 0
        pcbnew.ZONE_FILLER(board).Fill(zones)
        board.BuildConnectivity()
        return len(zones)
    except Exception as e:
        print(f"(zone refill skipped: {e})")
        return 0


def sync_footprint_positions_from_board(board, pcb_data):
    """Refresh footprint and pad POSITIONS in pcb_data from the live pcbnew board
    (#362).

    pcb_data is parsed once when the dialog opens and then reused across plan
    steps. optimize_caps (and manual edits) relocate footprints on the board via
    SetPosition, but the cached pcb_data keeps their load-time positions -- so a
    LATER signal/diff route step routes around where a cap USED to be, and the
    moved cap's pad then shorts the fresh copper (rp2350: +1V1 vias grazing moved
    decoupling caps C18/C22/C23/C24).

    Re-reads each footprint's x/y/rotation and its pads' absolute positions.
    Pad objects are shared with pcb_data.pads_by_net and net.pads, so updating
    them here updates every view the router consults. Best-effort; never raises.
    Returns the number of footprints synced (0 on error).

    Pads are matched to pcb_data by ITERATION ORDER, not pad number: a single
    footprint routinely carries many pads sharing one number (U6 has 11 pads
    numbered "61" for its GND array, plus 4 blank-numbered pads). Matching by
    number collapses them all onto the first pad's position, deleting the copper
    obstacle everywhere else and letting the router short straight through it.
    build_pcb_data_from_board iterates fp.Pads() once with no skips, so order is
    a faithful 1:1 map. The position math below mirrors that function's
    copper-offset fold (#324/#325) so a no-op sync is a true no-op."""
    try:
        import math
        import pcbnew
        n = 0
        for bfp in board.GetFootprints():
            ref = bfp.GetReference()
            pd_fp = pcb_data.footprints.get(ref)
            if pd_fp is None:
                continue
            fpos = bfp.GetPosition()
            pd_fp.x = pcbnew.ToMM(fpos.x)
            pd_fp.y = pcbnew.ToMM(fpos.y)
            try:
                pd_fp.rotation = bfp.GetOrientationDegrees()
            except Exception:
                pass
            bpads = list(bfp.Pads())
            if len(bpads) != len(pd_fp.pads):
                # Order alignment can't be trusted if the counts disagree
                # (should never happen from a live board); skip pads for safety.
                n += 1
                continue
            for pd_pad, bp in zip(pd_fp.pads, bpads):
                ppos = bp.GetPosition()
                gx = pcbnew.ToMM(ppos.x)
                gy = pcbnew.ToMM(ppos.y)
                # Fold the rotated copper offset into the position, exactly as
                # build_pcb_data_from_board does, so global_x/global_y stays the
                # copper center for offset pads.
                try:
                    off = bp.GetOffset()
                    if off.x or off.y:
                        oa = math.radians(-bp.GetOrientationDegrees())
                        ox, oy = pcbnew.ToMM(off.x), pcbnew.ToMM(off.y)
                        gx += ox * math.cos(oa) - oy * math.sin(oa)
                        gy += ox * math.sin(oa) + oy * math.cos(oa)
                        if bp.GetDrillSize().x:
                            pd_pad.hole_x = pcbnew.ToMM(ppos.x)
                            pd_pad.hole_y = pcbnew.ToMM(ppos.y)
                except Exception:
                    pass
                pd_pad.global_x = gx
                pd_pad.global_y = gy
            n += 1
        return n
    except Exception as e:
        print(f"Warning: Error syncing footprint positions from board: {e}")
        return 0


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
        # CLI parity (#338): 0.0 edge clearance means "not enforced this
        # step", NOT "lower the rule to zero" -- writing 0 erased the board's
        # own min_copper_edge_clearance (compute_targets now skips it the
        # same way). A real enforced value is >= the board rule (the engines
        # route to max(flag, board rule)), so only-lower keeps the design
        # value.
        lower('m_CopperEdgeClearance', edge_clearance or None)
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
