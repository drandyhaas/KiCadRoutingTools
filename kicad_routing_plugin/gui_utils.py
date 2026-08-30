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


def apply_drc_settings_fix(cfg, *, diff_pair=False):
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
    """
    if not (cfg and cfg.get('fix_drc_settings', True)):
        return None
    try:
        from kicad_ipc_adapter import write_drc_settings_to_project, get_board
        board = get_board()
        if board is None:
            return None
        kwargs = dict(
            clearance=cfg.get('clearance'),
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
        )
        if diff_pair:
            kwargs['diff_pair_gap'] = cfg.get('diff_pair_gap')
            kwargs['diff_pair_width'] = cfg.get('track_width')
        proj = write_drc_settings_to_project(board, **kwargs)
        if proj:
            print(f"DRC settings: loosened Board Setup floors in {proj}")
            print("  NOTE: reload the project (close & reopen, or File - "
                  "Revert) so KiCad picks up the new DRC rules -- the IPC API "
                  "cannot change them in the live editor.")
        return proj
    except Exception as e:
        print(f"(skipped DRC-settings write-back: {e})")
        return None


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
