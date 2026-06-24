"""
KiCad Routing Tools - GUI Utilities

Shared utilities for the plugin GUI.
"""


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
