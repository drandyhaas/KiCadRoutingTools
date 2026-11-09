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


#: Set True on a plan step's owner (the routing dialog, or the tab that runs
#: the step) when its operation PRODUCED ITS RESULT -- applied to the live
#: board, an empty result included. It is the GUI's "the step's output board
#: exists": a CLI step that refuses, fails or is interrupted writes no output,
#: so the next step of the chain fails on the missing file and the chain stops
#: there. The plan executor clears it before each step and stops the plan when
#: the step ends without it, so a refusal path nobody marks still stops.
PLAN_STEP_PRODUCED = '_plan_step_produced'


def mark_plan_step_produced(owner):
    """Record that `owner`'s operation produced its result (see above)."""
    setattr(owner, PLAN_STEP_PRODUCED, True)


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
    return save_board_via_ui_thread_ex(path, board, timeout_s)[0]


def save_board_via_ui_thread_ex(path, board, timeout_s=None):
    """Plugin-side alias for :func:`ui_thread.save_board_on_ui_thread_ex`
    (#828): ``(ok, SaveStatus)``, so a caller can tell the 120 s expiry
    (this machine, this moment -- retryable) from a ``SaveBoard`` exception
    (this board -- it will throw again) instead of one bare ``False``."""
    from ui_thread import save_board_on_ui_thread_ex, SAVE_BOARD_UI_TIMEOUT_S
    return save_board_on_ui_thread_ex(
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


def layer_costs_for_layers(costs, layers, selected_layers, copper_layers):
    """The Basic tab's Layer Costs values as one cost per entry of `layers`.

    The control is read the way it is written: one value per SELECTED routing
    layer, else one per board copper layer (its default, its tooltip's order
    and an AI plan's order). Either way the values are placed on `layers` BY
    NAME -- the engines price them positionally, and a plane step routes on
    F.Cu + its pour layers + B.Cu, not on the selected layers. Returns [] when
    the control is empty. A count that matches neither list is returned as
    typed: the engine reads it positionally against `layers`, and refuses it
    when it is not one value per layer (routing_config.layer_costs_count_error).
    Raises ValueError, naming the layers, when the values are for the selected
    layers and `layers` holds one that is not selected (found reviewing #1269:
    the Planes tab handed create_plane the selected layers' costs against its
    own layer list, so a 4-layer board's default costs priced the wrong
    layers and left the unrequested inner layer routable)."""
    costs = list(costs or [])
    if not costs:
        return []
    by_name = None
    for names in (selected_layers, copper_layers):
        if names and len(costs) == len(names):
            by_name = dict(zip(names, costs))
            break
    if by_name is None:
        return costs
    missing = [l for l in layers if l not in by_name]
    if missing:
        raise ValueError(
            f"Layer Costs gives {len(costs)} value(s), one per selected layer "
            f"({' '.join(selected_layers)}), but this step also routes on "
            f"{' '.join(missing)}: give one value per copper layer "
            f"({' '.join(copper_layers)}) instead")
    return [by_name[l] for l in layers]


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


#: What was said before a tab's log capture began, for the next capture to carry.
_log_notes = []


def note_for_the_log(text):
    """Say `text` in the log of the run it concerns. A run's parameters are read
    before its tab starts capturing stdout, so a plain print there reaches the
    terminal and never the log a GUI user reads: held until the next capture
    starts, or printed now inside one."""
    import sys
    if isinstance(sys.stdout, StdoutRedirector):
        print(text)
    else:
        _log_notes.append(text)


def apply_drc_settings_fix(cfg, *, diff_pair=False, fix_kwargs=None,
                           hole_clearance=None):
    """Loosen the open project's DRC floors to the routed values (issue #160).

    The IPC counterpart of the CLI route auto-fix. Shared by every routing tab
    so the config-key mapping and the user-facing reload note live in one place.
    Reads the floors from `cfg` (the tab's routing config dict) and delegates to
    the IPC adapter, which edits the sibling `.kicad_pro` on disk. Because kipy
    cannot push design settings into KiCad's in-memory project, the user must
    RELOAD the project for the change to take effect -- we print that clearly.

    No-op (returns None) when cfg disables it (`fix_drc_settings` False) or the
    board has no project file on disk yet. Best-effort and fully guarded -- a
    failure here must never block applying the routes.

    `diff_pair=True` also lowers the Default net class's differential-pair
    geometry to the routed gap/width (the diff tab's track_width is the per-pair
    trace width).

    `fix_kwargs` replaces the floors read off `cfg` with keywords decided
    elsewhere -- the fanout tab's, from fix_kicad_drc_settings.
    fanout_fix_kwargs, the ONE decision bga_fanout's / qfn_fanout's mains and
    both GUI fronts read. The gate stays `cfg`'s, so every tab's writeback
    still goes through this one choke point (#693). The Planes tab's come
    from fix_kicad_drc_settings.plane_fix_kwargs (#1319), as route_planes'
    main writes them.

    `hole_clearance` (cfg-built floors only) is the copper-to-hole floor the
    step HELD (`held_hole_clearance_live`, #1369/#1493); a `fix_kwargs`
    caller puts it in its own dict.

    The step's tool records are persisted whatever the gate says
    (`persist_live_tool_records`, #1415): they are not DRC floors.
    """
    _snpc_rec = (cfg or {}).get('same_net_pad_clearance')
    if not (cfg and cfg.get('fix_drc_settings', True)):
        persist_live_tool_records(None, _snpc_rec)
        return None
    try:
        from kicad_ipc_adapter import write_drc_settings_to_project, get_board
        board = get_board()
        if board is None:
            return None
        import clearance_ledger
        _clr = cfg.get('clearance')
        kwargs = dict(fix_kwargs) if fix_kwargs is not None else dict(
            # Graded at the smallest clearance any step actually routed at,
            # as route.py / route_diff.py write it and the SWIG tabs do.
            clearance=clearance_ledger.effective(_clr) if _clr else _clr,
            hole_to_hole=cfg.get('hole_to_hole_clearance'),
            edge_clearance=cfg.get('board_edge_clearance'),
            track_width=cfg.get('track_width'),
            via_diameter=cfg.get('via_size'),
            via_drill=cfg.get('via_drill'),
            keep_thermal=cfg.get('keep_thermal', False),
            # #439/#768 writeback half. The Min-Clearance override IS the
            # GUI's "--clearance was GIVEN": ticked -> each net routed at
            # min(its class, the ceiling), so the classes must be clamped down
            # to match or KiCad grades correct copper against the wider stock
            # class. Unticked -> every net routed at its OWN class and the
            # classes are PRESERVED.
            #
            # Gated rather than left to fix_project_for_output's own True
            # default, which clamped on both branches: the tabs read
            # `clamp_netclasses` when they PRICE (routing_dialog's net_clearances
            # cap, fanout's netclass_ceiling), so the writeback must read the
            # same key or the two halves decide differently -- #782 in the
            # other direction. Every tab's config carries it (signal, diff,
            # planes and fanout all set it off self.clearance_check); the
            # False default matches "the operator never ticked the override".
            clamp_nondefault_netclasses=bool(cfg.get('clamp_netclasses', False)),
            # #1492: the raw ceiling the clamped classes land at, None when
            # the box is unticked
            class_ceiling=cfg.get('clearance_ceiling'),
            hole_clearance=hole_clearance,
        )
        if diff_pair and fix_kwargs is None:
            kwargs['diff_pair_gap'] = cfg.get('diff_pair_gap')
            kwargs['diff_pair_width'] = cfg.get('track_width')
        proj = write_drc_settings_to_project(board, **kwargs)
        if proj:
            print(f"DRC settings: loosened Board Setup floors in {proj}")
            print("  NOTE: reload the project (close & reopen, or File - "
                  "Revert) so KiCad picks up the new DRC rules -- the IPC API "
                  "cannot change them in the live editor.")
        # The tool records, after the floors, as the CLI writes them (#521,
        # #581, #678, #1379, #1503), and whatever the write did (#1415).
        persist_live_tool_records(None, _snpc_rec)
        return proj
    except Exception as e:
        print(f"(skipped DRC-settings write-back: {e})")
        persist_live_tool_records(None, _snpc_rec)
        return None


def persist_live_tool_records(board=None, same_net_pad_clearance=None):
    """Record the step's tool records -- protected nets, impedance
    declarations, the P/N skews matched (#1503), the power widths asked for
    (#1379), pour-served balls (#678) and an active same-net pad via
    clearance (#581) -- the IPC twin of main's helper of the same name, and
    INDEPENDENT of the DRC-floor writeback: "Fix DRC settings after routing"
    unchecked keeps the floors where they are, it must not void the step's
    records (#1415).

    Into the board's records through protected_nets (tool_records): the
    project AND the board file. kipy cannot write the live board's
    properties, so `board` is unused (main's tabs pass theirs) and the path
    comes from the adapter; KiCad shows the board-file copy only after a
    reload, the known IPC gap. Each consume empties its queue, so calling
    this twice records nothing twice. Best-effort: never raises."""
    try:
        from protected_nets import (consume_protection_candidates,
                                    consume_impedance_specs,
                                    consume_pour_served_pads,
                                    consume_intra_pair_tolerances,
                                    consume_power_widths,
                                    persist_protected_nets,
                                    persist_impedance_specs,
                                    persist_pour_served_pads,
                                    persist_intra_pair_tolerances,
                                    persist_power_widths,
                                    persist_same_net_pad_clearance,
                                    pro_path_for_board)
        from kicad_ipc_adapter import get_board_full_path
        _bf = get_board_full_path()
        if not _bf:
            return
        _pro = pro_path_for_board(_bf)
        persist_protected_nets(_pro, consume_protection_candidates())
        persist_impedance_specs(_pro, consume_impedance_specs())
        persist_intra_pair_tolerances(_pro, consume_intra_pair_tolerances())
        persist_power_widths(_pro, consume_power_widths())
        persist_pour_served_pads(_pro, consume_pour_served_pads())
        if same_net_pad_clearance is not None and same_net_pad_clearance > 0:
            persist_same_net_pad_clearance(_pro, same_net_pad_clearance)
    except Exception:
        pass


def held_hole_clearance_live(pcb_data, board, results_data, clearance):
    """The copper-to-hole floor to write for this step (#1369): the board's
    declared hole clearance the engine held
    (``obstacle_map.declared_hole_clearance`` on the PCBData it routed),
    lowered only to what the copper this step drew needs
    (``fix_kicad_drc_settings.step_copper_to_hole``) -- the GUI twin of
    route_diff.py's ``step_hole_clearance``, shared by the differential and
    route tabs (#1493). None (write the copper clearance, as before) when
    the board declares none. The IPC twin of main's helper: the board path
    comes from the adapter, so `board` is unused."""
    try:
        from obstacle_map import declared_hole_clearance
        from fix_kicad_drc_settings import (step_copper_to_hole,
                                            held_hole_clearance)
        from kicad_ipc_adapter import get_board_full_path
        held, _src = declared_hole_clearance(pcb_data,
                                             get_board_full_path() or None)
        if not held or clearance is None or held <= clearance:
            return None
        segs, vias = [], []
        for result in results_data.get('results', []):
            segs.extend(result.get('new_segments', []))
            vias.extend(result.get('new_vias', []))
        segs.extend(results_data.get('all_swap_segments', []))
        vias.extend(results_data.get('all_swap_vias', []))
        # The board the copper was drawn on: the routed PCBData without the
        # copper this step removed.
        from types import SimpleNamespace

        def _sk(s):
            a = (round(s.start_x, 4), round(s.start_y, 4))
            b = (round(s.end_x, 4), round(s.end_y, 4))
            return (min(a, b), max(a, b), s.layer, s.net_id)

        def _vk(v):
            return (round(v.x, 4), round(v.y, 4), v.net_id)
        _gone_s = {_sk(s) for s in results_data.get('segments_to_remove') or ()}
        _gone_v = {_vk(v) for v in results_data.get('vias_to_remove') or ()}
        pcb = SimpleNamespace(
            segments=[s for s in pcb_data.segments if _sk(s) not in _gone_s],
            vias=[v for v in pcb_data.vias if _vk(v) not in _gone_v],
            footprints=pcb_data.footprints,
            board_info=pcb_data.board_info)
        step_min = step_copper_to_hole(pcb, segs, vias, held, clearance)
        v = held_hole_clearance(held, clearance, step_min)
        # #1493: a floor written under the one held is ledgered, as the CLI's
        # step_hole_clearance ledgers it
        if v is not None and v < float(held) - 1e-6:
            from fab_tiers import note_narrowing
            note_narrowing(None, 'hole_clearance', float(held), v,
                           "copper-to-hole floor written under the board's own: "
                           "this step's copper sits closer to a foreign hole")
        return v
    except Exception as e:                                  # noqa: BLE001
        print(f"(copper-to-hole floor: {e}; following the copper clearance)")
        return None


def held_hole_clearance_dicts(pcb_data, board, tracks, vias, clearance):
    """:func:`held_hole_clearance_live` for a tab whose copper is the
    engine's track and via DICTS (the fanout tab, #1568; the Planes tab,
    #1577) rather than routing results."""
    from kicad_parser import Segment, Via
    segs = [Segment(start_x=t['start'][0], start_y=t['start'][1],
                    end_x=t['end'][0], end_y=t['end'][1], width=t['width'],
                    layer=t['layer'], net_id=t['net_id']) for t in tracks]
    new_vias = [Via(x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                    layers=list(v.get('layers') or ['F.Cu', 'B.Cu']),
                    net_id=v['net_id']) for v in vias]
    return held_hole_clearance_live(
        pcb_data, board,
        {'results': [{'new_segments': segs, 'new_vias': new_vias}]}, clearance)


class StdoutRedirector:
    """Redirects stdout to a callback function while preserving original output."""

    def __init__(self, callback, original_stdout):
        self.callback = callback
        self.original = original_stdout
        while _log_notes:
            self.write(_log_notes.pop(0) + '\n')

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
