"""
KiCad Routing Tools - GUI Utilities

Shared utilities for the plugin GUI.
"""

# Re-entrancy latch for ui_thread_status (see below). Module-level, not
# per-tab: a nested repaint can reach a DIFFERENT tab's helper.
_IN_UI_STATUS = False

# Secondary sink for ui_thread_status messages. The AI-tab plan executor
# registers one while a plan runs: its own status mirror is POLL-driven
# (wx.CallLater), so a step that runs ON the UI thread (fanout, cap
# optimize, every tab's apply phase) blocks the poll and the AI tab froze
# even while the working tab's label was repainting. ui_thread_status
# pushes each message here as well, so the tab the user is actually
# LOOKING at moves too. fn(message); None = no mirror.
_UI_STATUS_MIRROR = None


def set_ui_status_mirror(fn):
    """Register (or clear, with None) the secondary ui_thread_status sink."""
    global _UI_STATUS_MIRROR
    _UI_STATUS_MIRROR = fn


# Timestamp of the last UI-category event-loop pump (see ui_thread_status):
# throttles the YieldFor so a fast message burst stays cheap.
_LAST_UI_YIELD = 0.0


def save_board_via_ui_thread(path, board, timeout_s=None):
    """Plugin-side alias for :func:`ui_thread.save_board_on_ui_thread` (#688).

    The implementation is ENGINE-side because the engine makes worker-thread
    pcbnew calls of its own (the live-fill provider in
    ``kicad_parser.build_pcb_data_from_board``), so the guard cannot live only
    here. See that module for the py-spy evidence and the reasoning.
    """
    from ui_thread import save_board_on_ui_thread, SAVE_BOARD_UI_TIMEOUT_S
    return save_board_on_ui_thread(
        path, board,
        SAVE_BOARD_UI_TIMEOUT_S if timeout_s is None else timeout_s)


def ui_thread_status(status_text, progress_bar, message):
    """Show `message` for work running ON the wx main thread.

    Engine-thread progress reaches the UI through wx.CallAfter, so the main
    loop paints it. Work that runs ON the main thread (the apply phase, the
    fanout, the plane-copper cleanup) BLOCKS that loop, so a bare SetLabel
    would not repaint until the work finished -- the previous phase's label
    stays on screen and reads as a hang.

    Repainting from inside a KiCad ACTION PLUGIN is the delicate part, and the
    reason this is one guarded helper rather than three hand-rolled copies:

      * Only ever touch wx from the main thread. Off-thread callers are
        marshalled with CallAfter instead (never dropped).
      * Refresh + Update ONLY the static text. `wx.Gauge.Pulse()` starts an
        indeterminate ANIMATION -- on macOS that schedules timers and can
        dispatch events re-entrantly, which is exactly what corrupts the
        thread state PYTHON_ACTION_PLUGIN::CallMethod releases on return
        (a PyGILState_Release fatal abort takes KiCad down with it).
      * A latch, because Update() runs the paint path and a nested call from
        it must not recurse.
      * Fully guarded: a status update must never break the work it reports.

    macOS caveat that shaped this function: wxWindow.Update() CANNOT force a
    synchronous repaint on wxOSX/Cocoa -- painting only happens when the run
    loop turns (the wx docs call this out; Andy's report confirms it: labels
    set + Refresh + Update during a blocking fanout never appeared on screen).
    So after invalidating the label(s), this pumps the event loop for the
    UI/paint CATEGORY ONLY, throttled: wx.EventLoopBase.YieldFor(
    wx.EVT_CATEGORY_UI) dispatches paint/geometry events and RE-QUEUES
    everything else -- user input (a stray click, the dialog's close button)
    and timers (the plan executor's poll) are NOT dispatched, so nothing can
    re-enter a handler mid-run. This is deliberately narrower than the plain
    wx.Yield() the tabs use once at run start.

    Escape hatch: KICAD_NO_UI_STATUS_REPAINT=1 sets the label but never forces
    the paint or pumps the loop. The status then lags on blocking phases (the
    pre-fix behaviour), which is strictly cosmetic -- so if a repaint from
    inside the plugin ever destabilises a KiCad build, the label is the thing
    to give up, not the run.
    """
    global _IN_UI_STATUS, _LAST_UI_YIELD
    if _IN_UI_STATUS:
        return
    try:
        import os
        import time
        import wx
        if not wx.IsMainThread():
            wx.CallAfter(ui_thread_status, status_text, progress_bar, message)
            return
        _IN_UI_STATUS = True
        try:
            no_repaint = os.environ.get('KICAD_NO_UI_STATUS_REPAINT', '') in \
                ('1', 'yes', 'true')
            if status_text:
                status_text.SetLabel(message)
                if not no_repaint:
                    status_text.Refresh()
                    status_text.Update()
            if _UI_STATUS_MIRROR is not None:
                # Push to the AI tab's mirror (inside the latch, so a
                # paint-triggered nested call cannot recurse). Guarded
                # separately: a dead mirror widget must not break the
                # working tab's own status.
                try:
                    _UI_STATUS_MIRROR(message)
                except Exception:
                    pass
            if not no_repaint:
                # Actually PAINT the invalidated labels (see macOS caveat in
                # the docstring). Throttled so a fast per-ball burst costs a
                # bounded number of loop turns; a slow phase paints every
                # message. Guarded: a failed pump degrades to the lagging
                # label, never breaks the run.
                now = time.monotonic()
                if now - _LAST_UI_YIELD >= 0.05:
                    _LAST_UI_YIELD = now
                    try:
                        loop = wx.EventLoopBase.GetActive()
                        # IsRunning guard: only pump a loop that is actually
                        # dispatching (KiCad's MainLoop). An activated-but-
                        # never-run loop (synthetic harnesses) asserts inside
                        # YieldFor's pending-event sweep.
                        if loop is not None and loop.IsRunning():
                            loop.YieldFor(wx.EVT_CATEGORY_UI)
                    except Exception:
                        pass
        finally:
            _IN_UI_STATUS = False
    except Exception:
        _IN_UI_STATUS = False


from contextlib import contextmanager


@contextmanager
def redirect_prints_to_log(append_log):
    """Route print() output into the GUI log for the duration of a block,
    while preserving the original stdout (StdoutRedirector tees, not swallows).

    One shared helper because the audit found the coverage was accidental:
    the route/diff/planes WORKERS redirected, but the fanout tab never did,
    and every tab's APPLY phase runs after its worker restored stdout -- so
    engine narration (fanout escapes, the oracle reconnect, zone refills,
    plane cleanup) reached the terminal but never the log tab. Guarded:
    append_log=None (or a failure) degrades to plain stdout, never breaks
    the work.
    """
    import sys
    if not append_log:
        yield
        return
    original = sys.stdout
    try:
        sys.stdout = StdoutRedirector(append_log, original)
    except Exception:
        yield
        return
    try:
        yield
    finally:
        sys.stdout = original


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


def apply_via_protection(pcb_via, tenting_attrs):
    """Put a Via's parsed protection spec back onto a pcbnew PCB_VIA (#489 §8).

    GUI counterpart of the writer's tenting round-trip: without it, a via the GUI
    RE-PLACES (rip-up/reroute, sub-grid nudge, tap relocation) comes back with the
    board default and silently loses its own tenting/plugging/filling -- the loss
    the CLI side now avoids. A via with NO spec is deliberately left alone:
    pcbnew's *_MODE_FROM_BOARD already means "inherit the board setting".

    Best-effort; does nothing on a KiCad without these accessors. Returns True
    when something was applied.
    """
    if not tenting_attrs:
        return False
    try:
        import pcbnew
    except ImportError:
        return False
    import re

    def _sides(inner):
        """'(front no) (back yes)' -> {'front': False, 'back': True}"""
        return {m.group(1): (m.group(2) == 'yes')
                for m in re.finditer(r'\((front|back)\s+(yes|no)\)', inner or '')}

    applied = False
    for token, front_set, back_set, yes_name, no_name in (
            ('tenting', 'SetFrontTentingMode', 'SetBackTentingMode',
             'TENTING_MODE_TENTED', 'TENTING_MODE_NOT_TENTED'),
            ('covering', 'SetFrontCoveringMode', 'SetBackCoveringMode',
             'COVERING_MODE_COVERED', 'COVERING_MODE_NOT_COVERED'),
            ('plugging', 'SetFrontPluggingMode', 'SetBackPluggingMode',
             'PLUGGING_MODE_PLUGGED', 'PLUGGING_MODE_NOT_PLUGGED')):
        if token not in tenting_attrs:
            continue
        yes_const = getattr(pcbnew, yes_name, None)
        no_const = getattr(pcbnew, no_name, None)
        if yes_const is None or no_const is None:
            continue
        sides = _sides(tenting_attrs[token])
        for side, setter_name in (('front', front_set), ('back', back_set)):
            if side not in sides:
                continue
            setter = getattr(pcb_via, setter_name, None)
            if setter is None:
                continue
            try:
                setter(yes_const if sides[side] else no_const)
                applied = True
            except Exception:
                pass

    for token, setter_name, yes_name, no_name in (
            ('capping', 'SetCappingMode', 'CAPPING_MODE_CAPPED', 'CAPPING_MODE_NOT_CAPPED'),
            ('filling', 'SetFillingMode', 'FILLING_MODE_FILLED', 'FILLING_MODE_NOT_FILLED')):
        if token not in tenting_attrs:
            continue
        yes_const = getattr(pcbnew, yes_name, None)
        no_const = getattr(pcbnew, no_name, None)
        setter = getattr(pcb_via, setter_name, None)
        if yes_const is None or no_const is None or setter is None:
            continue
        value = (tenting_attrs[token] or '').strip().lower()
        if value not in ('yes', 'no'):
            continue
        try:
            setter(yes_const if value == 'yes' else no_const)
            applied = True
        except Exception:
            pass

    return applied


def apply_castellated_landing_retract(board, board_edge_clearance):
    """CLI/GUI parity twin of pcb_modification.retract_castellated_landings
    (run-6 fix 1.7): compute the shared castellated-landing retract delta on
    PCBData built from the LIVE board, then move the matching live track
    endpoints in place. Returns the number of endpoints moved. Best-effort --
    a failure never blocks an already-applied routing result."""
    if not board_edge_clearance or board_edge_clearance <= 0:
        return 0
    try:
        import pcbnew
        from kicad_parser import build_pcb_data_from_board, mm_to_iu
        from pcb_modification import compute_castellated_landing_retract
        pcb = build_pcb_data_from_board(board)
        delta = compute_castellated_landing_retract(pcb, board_edge_clearance)
        if delta.is_empty:
            return 0
        # Match live tracks by rounded endpoints + net, the same key style as
        # the planes tab's cleanup applier.
        moves = {}
        for s, end, nx, ny in delta.moves:
            k = (round(s.start_x, 3), round(s.start_y, 3),
                 round(s.end_x, 3), round(s.end_y, 3), s.net_id)
            moves[k] = (end, nx, ny)
        moved = 0
        for item in board.GetTracks():
            if item.Type() != pcbnew.PCB_TRACE_T:
                continue
            k = (round(pcbnew.ToMM(item.GetStart().x), 3),
                 round(pcbnew.ToMM(item.GetStart().y), 3),
                 round(pcbnew.ToMM(item.GetEnd().x), 3),
                 round(pcbnew.ToMM(item.GetEnd().y), 3),
                 item.GetNetCode())
            mv = moves.get(k)
            if mv is None:
                continue
            end, nx, ny = mv
            # mm_to_iu, never pcbnew.FromMM: FromMM truncates (1 nm low on
            # ~2% of 4-decimal coordinates), so the GUI would store a
            # different integer than the CLI's text write of the same
            # retracted endpoint -- the #493 divergence class this module's
            # own docstring promises cannot happen.
            pt = pcbnew.VECTOR2I(mm_to_iu(nx), mm_to_iu(ny))
            if end == 'start':
                item.SetStart(pt)
            else:
                item.SetEnd(pt)
            moved += 1
        if moved:
            board.SetModified()
            print(f"  Castellated landings: {moved} track end(s) retracted "
                  f"inside the edge-clearance zone (pad_prop_castellated)")
        return moved
    except Exception as e:
        print(f"  (skipped castellated-landing retract: {e})")
        return 0


def apply_teardrops_to_board(board):
    """Enable teardrops on every pad and via of the live board (#489 §9).

    GUI counterpart of the writers' `add_teardrops_to_pads` /
    `add_teardrops_to_vias`: the CLI applies teardrops to the output FILE, while
    the GUI applies copper straight into pcbnew, so without this the "Add
    teardrops" checkbox did nothing on the tabs that apply in memory (fanout, and
    the planes tabs, whose config key was read but never supplied).

    Matches the writers' scope -- all pads and all vias, not just the ones this
    run added -- so the two fronts produce the same board. Best-effort; returns
    (pads_changed, vias_changed), (0, 0) on a KiCad without the accessors.
    """
    try:
        import pcbnew
    except ImportError:
        return (0, 0)

    pads = vias = 0
    try:
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                if not pad.GetTeardropsEnabled():
                    pad.SetTeardropsEnabled(True)
                    pads += 1
        for track in board.GetTracks():
            if track.GetClass() != 'PCB_VIA':
                continue
            if not track.GetTeardropsEnabled():
                track.SetTeardropsEnabled(True)
                vias += 1
    except AttributeError as e:
        print(f"(teardrops skipped: this KiCad has no teardrop API: {e})")
        return (0, 0)
    if pads or vias:
        print(f"Teardrops enabled on {pads} pad(s) and {vias} via(s)")
    return (pads, vias)


def refill_all_zones(board):
    """Re-fill EVERY copper zone, then rebuild connectivity -- the ONLY safe
    way to register freshly added copper while filled zones exist (#362).

    Two distinct corruptions this prevents:
    - STALE FILLS (#362): a signal routed after planes exist leaves the plane
      fill with no antipad around the new track/via, which KiCad DRC flags as
      clearance / shorting violations on the saved board (the CLI board is
      graded with kicad-cli --refill-zones, so it never shows these).
    - NET FLIPS (mez_rx): ``board.BuildConnectivity()`` over a stale fill
      REASSIGNS a new via's netcode to the zone's net (the fill has no
      knockout, so pcbnew's net propagation sees them touching -- measured:
      a fresh RGMII_TX_CTL via under the BGA came back GND/V1P8/V3P3, 42 of
      131 fanout vias misnetted). Once flipped, the via is same-net with the
      zone, so a LATER refill keeps no knockout and the wrong net STICKS.
      ``pcbnew.LoadBoard`` propagates the same way, so a saved board with
      stale fills is corrupted for every pcbnew consumer that opens it.

    Therefore: call THIS (never a bare ``BuildConnectivity``) as the first
    connectivity rebuild after adding copper. ZONE_FILLER computes knockouts
    from the still-correct netcodes, and only then is connectivity rebuilt.
    Best-effort; builds connectivity even when the board has no zones.
    Returns the number of zones refilled (0 if none / on error)."""
    try:
        import pcbnew
        zones = list(board.Zones())
        if zones:
            pcbnew.ZONE_FILLER(board).Fill(zones)
        board.BuildConnectivity()
        return len(zones)
    except Exception as e:
        print(f"(zone refill skipped: {e})")
        try:
            board.BuildConnectivity()
        except Exception:
            pass
        return 0


def live_footprints_by_key(board):
    """{PCBData key: live pcbnew footprint} for a live board (#726).

    `board.FindFootprintByReference(ref)` returns the FIRST footprint with that
    reference, and `PCBData` keys duplicated references `TP4`, `TP4~2`, ... So
    a bare lookup silently returns the WRONG twin for one of them and `None`
    for the other -- and `None` is a `continue` at every call site, which is a
    part the GUI quietly declines to apply.

    Built with the same `disambiguate_references` over the same
    `board.GetFootprints()` order that `build_pcb_data_from_board` uses, so the
    keys are the ones the engine was handed. Returns `{}` if pcbnew is absent
    or the board cannot be walked; callers fall back to their old lookup, which
    is exactly today's behaviour.
    """
    try:
        from kicad_parser import disambiguate_references
        live = list(board.GetFootprints())
        raw = []
        for f in live:
            r = f.GetReference()
            if not r:
                try:
                    r = "#" + f.m_Uuid.AsString()
                except Exception:
                    r = "?"
            raw.append(r)
        return dict(zip(disambiguate_references(raw), live))
    except Exception:                                            # noqa: BLE001
        return {}


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
        from kicad_parser import disambiguate_references
        n = 0
        # Names are resolved for the WHOLE board and zipped on, exactly as
        # `build_pcb_data_from_board` does (#726). A bare `GetReference()`
        # lookup was the footprint-level twin of the pad bug this function's
        # docstring already describes: on a board with two `TP4` blocks both
        # of them looked up the ONE `TP4` entry, so the second overwrote the
        # first's pose and the parts moved onto each other in the cached model
        # -- the same class of silent corruption, one level up.
        _live = list(board.GetFootprints())
        _raw = []
        for _f in _live:
            _r = _f.GetReference()
            if not _r:
                try:
                    _r = "#" + _f.m_Uuid.AsString()
                except Exception:
                    _r = "?"
            _raw.append(_r)
        _keys = disambiguate_references(_raw)
        for bfp, ref in zip(_live, _keys):
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


def board_minima_from_live(board):
    """The GUI twin of fix_kicad_drc_settings.scan_board_minima(), read from the
    LIVE pcbnew board instead of re-parsing the board file.

    Same five keys, same meaning: the smallest track width / via diameter / via
    drill / via annular ring / through-hole diameter actually present, so KiCad's
    min-size rules can be floored at or below the board's own copper.

    Why not just call scan_board_minima: it runs parse_kicad_pcb over the whole
    file, allocating thousands of GC-tracked objects. The GUI calls it from
    _write_drc_floors, which runs inside a wx TIMER dispatch, and that allocation
    burst triggers an automatic gen0 collection mid-dispatch which walks a dict
    holding a stale pointer and segfaults (visit_decref -> dict_traverse ->
    collect, faulting on a read-only page). Measured at 3-7 crashes in 10 runs.
    The dangling reference is in a C extension, not here; the tractable fix is to
    stop doing a full board re-parse in that context when the live board -- which
    the GUI already holds -- has every number we need.

    Best-effort: returns {} on error, which makes compute_targets() fall back to
    its param-only behaviour exactly as an unparseable board would.
    """
    try:
        import pcbnew
        widths, via_sizes, via_drills, annular, holes = [], [], [], [], []
        for t in board.GetTracks():
            try:
                if t.Type() == pcbnew.PCB_VIA_T:
                    # GetFrontWidth() first: a bare PCB_VIA::GetWidth() trips a
                    # wxASSERT for every via on KiCad 10 (see
                    # update_live_drc_floors).
                    try:
                        w = pcbnew.ToMM(t.GetFrontWidth())
                    except Exception:
                        w = pcbnew.ToMM(t.GetWidth())
                    d = pcbnew.ToMM(t.GetDrillValue())
                    if w:
                        via_sizes.append(w)
                    if d:
                        via_drills.append(d)
                        holes.append(d)
                    if w and d and w > d:
                        annular.append((w - d) / 2.0)
                else:
                    w = pcbnew.ToMM(t.GetWidth())
                    if w:
                        widths.append(w)
            except Exception:
                continue
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                try:
                    ds = pad.GetDrillSize()
                    d = pcbnew.ToMM(max(ds.x, ds.y))
                    if d:
                        holes.append(d)
                except Exception:
                    continue
        out = {}
        if widths:
            out['min_track_width'] = min(widths)
        if via_sizes:
            out['min_via_diameter'] = min(via_sizes)
        if via_drills:
            out['min_via_drill'] = min(via_drills)
        if annular:
            out['min_via_annular_width'] = min(annular)
        if holes:
            out['min_through_hole_diameter'] = min(holes)
        return out
    except Exception as e:
        print(f"(live board minima scan skipped: {e})")
        return {}


def default_netclass(board):
    """The board's Default NETCLASS, or None.

    `board.GetNetClasses()` returns an EMPTY map on KiCad 10 -- the Default
    class moved to `GetDesignSettings().m_NetSettings.GetDefaultNetclass()`.
    Every `for name, nc in board.GetNetClasses().items(): if name == 'Default'`
    loop therefore iterated ZERO times, and because they all sit inside
    `except Exception: pass`, the entire netclass half of the GUI's DRC-floor
    writeback was a silent no-op. That is why a GUI plan run left the Default
    class at stock values (eth_tap: 0.125 / via 0.5-0.25) while the CLI chain
    tightened it per step (0.0889 / via 0.25-0.15) -- and every later step
    resolves its geometry from that class, so the two fronts routed with
    different parameters.

    Tries the KiCad 10 accessor first, then the legacy map."""
    try:
        ns = board.GetDesignSettings().m_NetSettings
        nc = ns.GetDefaultNetclass()
        if nc is not None:
            return nc
    except Exception:
        pass
    try:
        for _name, _nc in board.GetNetClasses().items():
            if _name == 'Default':
                return _nc
    except Exception:
        pass
    return None


def update_live_drc_floors(board, *, clearance=None, track_width=None,
                           via_size=None, via_drill=None,
                           hole_to_hole=None, edge_clearance=None,
                           nondefault_clamp_mm=None, log=None):
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

    ``nondefault_clamp_mm`` (#782) additionally clamps every NON-Default net
    class's clearance down to that value. THE PRESENCE OF A VALUE IS THE SWITCH,
    exactly as `--clearance` is on the CLI: None means the run honoured the
    board's classes and they are preserved; a number means the run was given a
    ceiling, priced every class at min(class, ceiling), and the classes must be
    lowered to match or KiCad grades correct copper against a class the run never
    used (#439/#768).

    Pass the CEILING, not the effective clearance. They differ whenever the
    board's Default sits below the ceiling -- flat_hierarchy is Default 0.2 /
    Wide 0.4, so an operator ceiling of 0.3 prices Wide at 0.3 while the
    effective clearance is 0.2. Clamping to 0.2 would ship a class BELOW the
    value the pass was priced at, which is #768's own shape in the safe
    direction, and it is still #768's shape.

    RETURNS the list of non-Default clamp change strings (empty when nothing was
    clamped, which includes every call that passes no ceiling). Returned rather
    than logged from in here on purpose: the caller decides how to disclose a
    change to the board's declared spec, and routing that through this
    function's `log` would have meant handing it a logger the fanout tab
    deliberately does not pass -- which would have added the generic
    "floors relaxed" line to every fanout run as a side effect of a clamp fix.

    Best-effort: never raises."""
    # #521: manual (non-plan) runs consume the engine-noted protection
    # candidates here -- the per-step floor update is the GUI's step boundary.
    # Written to the sibling .kicad_pro JSON like the CLI writeback; KiCad may
    # overwrite it if the user saves the project from KiCad afterwards (same
    # caveat as the plan executor's best-effort persistence).
    try:
        from protected_nets import (consume_protection_candidates,
                                    consume_impedance_specs,
                                    persist_protected_nets,
                                    persist_impedance_specs, pro_path_for_board)
        _bf = board.GetFileName() if board is not None else ""
        if _bf:
            _pro = pro_path_for_board(_bf)
            persist_protected_nets(_pro, consume_protection_candidates())
            persist_impedance_specs(_pro, consume_impedance_specs())
    except Exception:
        pass
    _clamped = []
    try:
        import pcbnew
        # mm_to_iu, NOT pcbnew.FromMM: FromMM TRUNCATES (#493). Measured on this
        # KiCad: FromMM(1.001) = 1000999 where the correct value is 1001000 --
        # 24 of 2010 swept floor values are 1 nm low, all >= 1mm. Today's floors
        # are sub-millimetre so nothing on the current corpus is affected, but a
        # board declaring e.g. a 1.27mm (50 mil) edge clearance would have its
        # DRC floor stamped 1 nm below what was routed, which is exactly the
        # phantom-violation class #493 fixed elsewhere.
        from kicad_parser import mm_to_iu
        bds = board.GetDesignSettings()

        # Actual board minima (copper tracks/vias only).
        min_w = min_via = min_drill = min_ann = None
        for t in board.GetTracks():
            try:
                if t.Type() == pcbnew.PCB_VIA_T:
                    # GetFrontWidth() FIRST. On KiCad 10 a bare
                    # PCB_VIA::GetWidth() trips a wxASSERT ("GetWidth called
                    # without a layer argument") for EVERY via on the board. It
                    # does not raise, so the old order hit the assert path
                    # hundreds of times per call and then fell through to the
                    # same answer -- pure noise in every headless log.
                    # NOT the segfault: with this reversed the asserts are gone
                    # (verified 0 in the log) and a 1-step replay still crashed
                    # 4 runs in 6. Tidy-up only; see the open crash note.
                    try:
                        w = t.GetFrontWidth()
                    except Exception:
                        w = t.GetWidth()
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
            iu = mm_to_iu(float(mm)) if mm is not None else None
            if board_min_iu is not None:
                iu = board_min_iu if iu is None else min(iu, board_min_iu)
            if getattr(bds, attr) > iu:
                setattr(bds, attr, iu)
        # #498 parity with fix_project_for_output: cap the board's ABSOLUTE
        # min-clearance floor at the smallest .kicad_dru layer rule -- the
        # floor outranks custom rules, so leaving it above a relaxing rule
        # re-manufactures phantom violations on that rule's layer.
        _clr_floor = clearance
        try:
            from kicad_dru import min_rule_clearance
            _dru_min = min_rule_clearance(board.GetFileName() or "")
            if _dru_min is not None and _clr_floor is not None \
                    and _dru_min < float(_clr_floor):
                _clr_floor = _dru_min
        except Exception:
            pass
        lower('m_MinClearance', _clr_floor)
        lower('m_TrackMinWidth', track_width, min_w)
        lower('m_ViasMinSize', via_size, min_via)
        lower('m_MinThroughDrill', via_drill, min_drill)
        lower('m_ViasMinAnnularWidth',
              (float(via_size) - float(via_drill)) / 2
              if via_size and via_drill else None, min_ann)
        lower('m_HoleToHoleMin', hole_to_hole)
        # CLI parity (#338/#441): 0.0 edge clearance means "not enforced this
        # step", NOT "lower the rule to zero". Unlike every other floor here,
        # copper-to-edge is PINNED UP to the fab minimum (0.20 mm): a board
        # declaring a sub-fab (or 0) edge rule would otherwise grade clean while
        # copper runs to the milled edge. Raise to max(routed edge, fab floor);
        # a board rule already above the fab floor (e.g. 0.5) is preserved.
        from fix_kicad_drc_settings import fab_edge_floor
        _edge_pin = max(edge_clearance or 0.0, fab_edge_floor())
        if _edge_pin > 0:
            _pin_iu = mm_to_iu(float(_edge_pin))
            if getattr(bds, 'm_CopperEdgeClearance', 0) < _pin_iu:
                bds.m_CopperEdgeClearance = _pin_iu
        try:
            _nc = default_netclass(board)
            if _nc is not None:
                def _nc_lower(get, set_, mm, board_min_iu=None):
                    if mm is None and board_min_iu is None:
                        return
                    iu = mm_to_iu(float(mm)) if mm is not None else None
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
        # #782: the NON-Default classes. Delegated, never re-implemented -- this
        # is the same clamp `apply_targets_to_board` applies for the signal /
        # differential / planes tabs, and a second copy here is the bug class
        # #736/#747/#775 each fixed once in the placement engine.
        if nondefault_clamp_mm is not None:
            try:
                from fix_kicad_drc_settings import (
                    clamp_nondefault_netclasses_on_board)
                _clamped = clamp_nondefault_netclasses_on_board(
                    board, {'min_clearance': float(nondefault_clamp_mm)})
            except Exception as _nde:                          # noqa: BLE001
                _clamped = [f'(non-Default net-class clamp skipped: {_nde})']
        if log:
            log("Live DRC floors relaxed to this step's routed values "
                "(clamped to actual board minima)\n")
        return _clamped
    except Exception as e:
        if log:
            log(f"(live DRC floor update skipped: {e})\n")
    return _clamped


def run_kicad_oracle_on_live_board(board, net_names, *, clearance,
                                   track_width, via_size, via_drill,
                                   grid_step, track_via_clearance=None,
                                   hole_to_hole_clearance=None,
                                   layer_clearances=None,
                                   layers=None, layer_costs=None,
                                   power_net_widths=None,
                                   progress_callback=None):
    """Staged-save kicad-oracle recheck against the LIVE pcbnew board.

    The CLI plane fronts (and route.py's plane finalize, #562) finish with
    oracle_reconnect on their WRITTEN file; kicad-cli's exact fill needs a
    real file, so the GUI counterpart temp-saves the live board, routes the
    exact links KiCad reports missing, and applies the returned copper (and
    stranded-fragment deletions, #508 finding 15) back onto the live board.

    Shared core for the planes tab's post-apply recheck and the signal tab's
    plane-finalize oracle leg -- one implementation, both fronts. Callers
    refill zones AFTER this returns (the routed links change the fill).
    Returns the oracle result dict, or None when skipped (no kicad-cli).
    Skips quietly on any error: the recheck is an earner, never a blocker.
    """
    try:
        import os
        import tempfile
        import pcbnew
        from kicad_parser import mm_to_iu
        from kicad_oracle import oracle_reconnect, find_kicad_cli
        import routing_defaults as defaults
        if find_kicad_cli() is None:
            return None
        nets = [n for n in (net_names or []) if n]
        if not nets:
            return None
        from routing_config import GridRouteConfig
        # #338: the temp save has no sibling .kicad_pro, so oracle_reconnect
        # cannot resolve the board's copper-to-edge rule itself -- read it
        # from the live board's design settings (nm -> mm).
        try:
            _edge_mm = (board.GetDesignSettings().m_CopperEdgeClearance
                        or 0) / 1e6
        except Exception:
            _edge_mm = 0.0
        # #658 (audit finding): this config used to be BARE -- no layers, no
        # layer_costs, no power_net_widths -- so the weld router here ran at
        # UNIFORM layer economics while the CLI finalize priced them, and the
        # #658 forbidden-layer guards inside oracle_reconnect were inert.
        # Costs are soft, so a weld that MUST cross a plane layer still can.
        # Omitted values fall back to the dataclass defaults, so an older
        # caller that passes none behaves exactly as before.
        _cfg_kw = {}
        if layers:
            _cfg_kw['layers'] = list(layers)
        if layer_costs:
            _cfg_kw['layer_costs'] = list(layer_costs)
        if power_net_widths:
            _cfg_kw['power_net_widths'] = dict(power_net_widths)
        ocfg = GridRouteConfig(
            clearance=clearance, track_width=track_width,
            via_size=via_size, via_drill=via_drill, grid_step=grid_step,
            board_edge_clearance=_edge_mm, **_cfg_kw)
        # #498: the temp save has no sibling .kicad_dru, so the oracle's own
        # auto-read finds nothing -- install the map explicitly (caller's
        # resolved map when given, else read the LIVE board's project file).
        try:
            from kicad_dru import install_layer_clearances
            install_layer_clearances(
                ocfg, layer_clearances,
                (board.GetFileName() or None) if layer_clearances is None
                else None, None)
        except Exception:
            pass
        with tempfile.NamedTemporaryFile(suffix='.kicad_pcb',
                                         delete=False) as f:
            tmp = f.name
        # aSkipSettings: the implicit settings save aborts KiCad on worker
        # threads for pre-KiCad-10 projects (uncaught C++ type_error in the
        # .kicad_pro merge; see _stage_live_board in swig_gui.py). But the
        # sibling .kicad_pro it used to write carried the session's LIVE
        # rules to the oracle's exact-fill refill -- project_from below only
        # stages the on-disk project, which is missing every clamp
        # update_live_drc_floors applied in memory (#627). Re-author the
        # sibling from the live board instead, crash-free.
        pcbnew.SaveBoard(tmp, board, aSkipSettings=True)
        try:
            from kicad_parser import stage_live_project_rules
            stage_live_project_rules(tmp, board)
        except Exception:
            pass
        # #490: stage the REAL project's netclasses for the refill, or the
        # exact-fill link source runs at stock rules.
        try:
            _proj_from = board.GetFileName() or None
        except Exception:
            _proj_from = None
        orc = oracle_reconnect(
            tmp, nets, ocfg,
            project_from=_proj_from,
            track_via_clearance=(track_via_clearance
                                 if track_via_clearance is not None
                                 else defaults.PLANE_TRACK_VIA_CLEARANCE),
            hole_to_hole_clearance=(hole_to_hole_clearance
                                    if hole_to_hole_clearance is not None
                                    else defaults.HOLE_TO_HOLE_CLEARANCE),
            progress_callback=progress_callback)
        from copy_board import SIBLING_EXTS as _sib_exts
        for _p in (tmp,) + tuple(os.path.splitext(tmp)[0] + _e
                                 for _e in _sib_exts):
            try:
                os.unlink(_p)
            except OSError:
                pass
        # #713 item 3: this front never inspected `available`, so an oracle
        # that COULD NOT RUN was byte-for-byte indistinguishable here from one
        # that ran and found everything already connected -- both are an empty
        # results dict and both were silent. Say which, on the GUI's own log,
        # since the CLI front has said so since #508.
        if not orc.get('available'):
            print(f"KiCad-oracle: did NOT run -- "
                  f"{orc.get('why', 'no reason recorded')}. The zone-aware "
                  f"completion check behind this apply is UNBACKED; a clean "
                  f"result here is 'not checked', not 'checked and clean'.")
        # #508 finding 15: the oracle's stranded-fragment deletions are
        # stripped from its temp file, but the LIVE board still holds that
        # copper -- delete it here, BEFORE the adds, so a same-position
        # emission is never collateral.
        _orc_rm = ((orc.get('removed_segments') or [])
                   + (orc.get('removed_vias') or []))
        if _orc_rm:
            _rm_keys = set()
            for s in _orc_rm:
                if hasattr(s, 'start_x'):
                    _rm_keys.add((round(s.start_x, 3), round(s.start_y, 3),
                                  round(s.end_x, 3), round(s.end_y, 3),
                                  s.net_id))
                else:
                    _rm_keys.add((round(s.x, 3), round(s.y, 3), s.net_id))
            _n_orc_rm = 0
            for item in list(board.GetTracks()):
                if item.Type() == pcbnew.PCB_VIA_T:
                    k = (round(pcbnew.ToMM(item.GetPosition().x), 3),
                         round(pcbnew.ToMM(item.GetPosition().y), 3),
                         item.GetNetCode())
                else:
                    k = (round(pcbnew.ToMM(item.GetStart().x), 3),
                         round(pcbnew.ToMM(item.GetStart().y), 3),
                         round(pcbnew.ToMM(item.GetEnd().x), 3),
                         round(pcbnew.ToMM(item.GetEnd().y), 3),
                         item.GetNetCode())
                    if k not in _rm_keys:
                        k = (k[2], k[3], k[0], k[1], k[4])
                if k in _rm_keys:
                    board.RemoveNative(item)
                    _n_orc_rm += 1
            if _n_orc_rm:
                print(f"KiCad-oracle (GUI): deleted {_n_orc_rm} stranded "
                      f"fragment piece(s) from the live board")
        for s in orc.get('new_segments') or []:
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(pcbnew.VECTOR2I(
                mm_to_iu(s.start_x), mm_to_iu(s.start_y)))
            track.SetEnd(pcbnew.VECTOR2I(
                mm_to_iu(s.end_x), mm_to_iu(s.end_y)))
            track.SetWidth(mm_to_iu(s.width))
            track.SetLayer(board.GetLayerID(s.layer))
            track.SetNetCode(s.net_id)
            board.Add(track)
        for v in orc.get('new_vias') or []:
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pcbnew.VECTOR2I(mm_to_iu(v.x), mm_to_iu(v.y)))
            via.SetDrill(mm_to_iu(v.drill))
            via.SetWidth(mm_to_iu(v.size))
            via.SetNetCode(v.net_id)
            lys = v.layers or ['F.Cu', 'B.Cu']
            via.SetLayerPair(board.GetLayerID(lys[0]),
                             board.GetLayerID(lys[-1]))
            board.Add(via)
        if orc.get('links_routed'):
            print(f"KiCad-oracle (GUI): routed {orc['links_routed']} "
                  f"missing link(s), applied to the live board")
        return orc
    except Exception as e:
        print(f"KiCad-oracle (GUI) skipped: {e}")
        return None
