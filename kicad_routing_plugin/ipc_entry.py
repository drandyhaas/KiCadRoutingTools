"""KiCad Routing Tools - IPC plugin entry point.

KiCad 10+ spawns this script as a separate process when the user clicks the
plugin's action button. The script connects back to KiCad over the IPC
socket (env vars KICAD_API_SOCKET + KICAD_API_TOKEN), reads the open board,
shows the wxPython RoutingDialog, and applies the user's routing results
back via a single commit.

Diagnostics
-----------
KiCad swallows stdout/stderr from spawned plugin processes. Everything
this entry point does is also appended to ``~/.kicad_routing_tools.log``
so a stuck/crashed launch can be inspected after the fact.

To run from the command line during development:

    1. Launch KiCad 10 and open a board.
    2. Make sure the IPC API is enabled in Preferences → Plugins.
    3. Export the IPC env vars from the same shell KiCad is running in,
       or have KiCad spawn this script directly.
    4. python kicad_routing_plugin/ipc_entry.py
"""

from __future__ import annotations

import os
import sys
import traceback
from datetime import datetime


# wxPython 4.2 raises an assertion when a sizer combines wxEXPAND with
# alignment flags (wxALIGN_*). The dialog was written against an older,
# laxer wx and trips this in many places. Suppress the new check rather
# than auditing 2500 lines of layout code as part of the IPC migration.
# The env var must be set BEFORE wx is imported. We also call
# wx.SizerFlags.DisableConsistencyChecks() later as a belt-and-suspenders.
os.environ.setdefault("WXSUPPRESS_SIZER_FLAGS_CHECK", "1")


# Make sure the plugin's parent directory is on sys.path so `kicad_parser`,
# `kicad_ipc_adapter`, `routing_defaults`, etc. import cleanly when KiCad
# spawns this script with the cwd set somewhere else.
_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.dirname(_PLUGIN_DIR)
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)
if _PLUGIN_DIR not in sys.path:
    sys.path.insert(0, _PLUGIN_DIR)


_LOG_PATH = os.path.expanduser("~/.kicad_routing_tools.log")
_SETTINGS_PATH = os.path.expanduser("~/.kicad_routing_tools_settings.json")


def _load_settings() -> dict:
    """Read previously-persisted dialog settings, or {} on first launch."""
    import json
    try:
        with open(_SETTINGS_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except Exception as e:
        _log(f"settings load failed ({e}); starting with defaults")
        return {}


def _save_settings(settings: dict) -> None:
    """Persist dialog settings. Tuples become lists (JSON has no tuples)."""
    import json

    def _coerce(obj):
        if isinstance(obj, tuple):
            return [_coerce(x) for x in obj]
        if isinstance(obj, list):
            return [_coerce(x) for x in obj]
        if isinstance(obj, dict):
            return {k: _coerce(v) for k, v in obj.items()}
        if isinstance(obj, set):
            return [_coerce(x) for x in obj]
        return obj

    try:
        with open(_SETTINGS_PATH, "w", encoding="utf-8") as f:
            json.dump(_coerce(settings), f, indent=2, default=str)
    except Exception as e:
        _log(f"settings save failed: {e}")


def _log(msg: str) -> None:
    """Append a line to the diagnostic log. Best-effort; never throws."""
    try:
        with open(_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(f"[{datetime.now().isoformat(timespec='seconds')}] {msg}\n")
    except Exception:
        pass


def _activate_macos_app() -> None:
    """Force the spawned process to become the frontmost macOS app.

    Without this, the wx dialog can open behind KiCad and look like nothing
    happened. Best-effort; silently no-op if PyObjC is unavailable.
    """
    if sys.platform != "darwin":
        return
    try:
        from AppKit import NSApplication, NSApplicationActivationPolicyRegular
        app = NSApplication.sharedApplication()
        app.setActivationPolicy_(NSApplicationActivationPolicyRegular)
        app.activateIgnoringOtherApps_(True)
        _log("macOS: activation policy set to Regular, activated")
    except Exception as e:
        _log(f"macOS activation skipped (PyObjC not available?): {e}")


def main() -> int:
    _log("=" * 60)
    _log(f"ipc_entry start, pid={os.getpid()}, cwd={os.getcwd()}")
    _log(f"sys.executable={sys.executable}")
    _log(f"sys.path[0:3]={sys.path[0:3]}")
    _log(f"KICAD_API_SOCKET={os.environ.get('KICAD_API_SOCKET', '<unset>')}")
    _log(f"KICAD_API_TOKEN present: {bool(os.environ.get('KICAD_API_TOKEN'))}")

    # wx is required by every dialog; fail fast if it's not in the venv.
    try:
        import wx
        _log(f"wx imported, version={wx.version()}")
    except ImportError as e:
        _log(f"FATAL: wxPython not installed: {e}")
        return 2

    # Disable wxPython 4.2's sizer-flags consistency assertion — see the
    # WXSUPPRESS_SIZER_FLAGS_CHECK comment at the top of this file.
    try:
        wx.SizerFlags.DisableConsistencyChecks()
        _log("wx.SizerFlags consistency checks disabled")
    except AttributeError:
        # Older wxPython without this API — the env var is enough.
        pass

    # Build the wx.App before any wx.MessageBox / Dialog call.
    app = wx.App(False)
    _activate_macos_app()

    # Connect to KiCad over IPC and grab the open board.
    try:
        from kicad_ipc_adapter import connect, get_board
        _log("connecting to KiCad over IPC...")
        kicad = connect()
        _log("connected; fetching board...")
        board = get_board(kicad)
        _log(f"board={board!r}")
    except Exception as e:
        tb = traceback.format_exc()
        _log(f"IPC connect failed:\n{tb}")
        wx.MessageBox(
            f"Could not connect to KiCad over IPC:\n\n{e}\n\n"
            f"See {_LOG_PATH} for details.",
            "KiCadRoutingTools — IPC error",
            wx.OK | wx.ICON_ERROR,
        )
        return 3

    if board is None:
        _log("No board open")
        wx.MessageBox(
            "No board is currently open.\nPlease open a PCB file first.",
            "No Board",
            wx.OK | wx.ICON_WARNING,
        )
        return 4

    # Read the board into PCBData (the project's internal representation).
    try:
        _log("building PCBData...")
        from kicad_parser import build_pcb_data_from_board
        pcb_data = build_pcb_data_from_board(board)
        nets_with_pads = sum(1 for n in pcb_data.nets.values() if len(n.pads) >= 2)
        total_pads = sum(len(fp.pads) for fp in pcb_data.footprints.values())
        _log(f"PCBData built: {len(pcb_data.footprints)} footprints, "
             f"{total_pads} pads, "
             f"{len(pcb_data.nets)} nets ({nets_with_pads} with >=2 pads), "
             f"{len(pcb_data.segments)} segments")
    except Exception as e:
        tb = traceback.format_exc()
        _log(f"build_pcb_data_from_board failed:\n{tb}")
        wx.MessageBox(
            f"Failed to read board data:\n\n{e}\n\nSee {_LOG_PATH} for details.",
            "Board Read Error",
            wx.OK | wx.ICON_ERROR,
        )
        return 5

    # Resolve the board's absolute filesystem path. kipy.Board.name is just
    # the basename; the full path comes from kicad.get_open_documents()
    # (best-effort — see kicad_ipc_adapter.get_board_full_path). We need an
    # absolute path for the schematic-dir picker, _validate_pcb_data, etc.
    board_filename = ""
    try:
        from kicad_ipc_adapter import get_board_full_path
        full = get_board_full_path(kicad)
        if full:
            board_filename = full
    except Exception as e:
        _log(f"get_board_full_path failed: {e}")
    if not board_filename:
        for attr in ("filename", "file_name", "path", "name"):
            v = getattr(board, attr, None)
            if isinstance(v, str) and v:
                board_filename = v
                break
    _log(f"board_filename={board_filename!r}")

    # Construct + show the routing dialog. RoutingDialog.__init__ is large
    # (loads nets, builds fanout/differential/planes panels, etc.) and any
    # exception there used to take the whole process down silently.
    saved_settings = _load_settings()
    _log(f"loaded {len(saved_settings)} saved-settings keys "
         f"from {_SETTINGS_PATH}")
    try:
        _log("constructing RoutingDialog...")
        from kicad_routing_plugin.swig_gui import RoutingDialog
        dlg = RoutingDialog(None, pcb_data, board_filename,
                             saved_settings=saved_settings)
        dlg._ipc_board = board
        dlg._ipc_kicad = kicad
        _log("RoutingDialog constructed; calling Raise() + ShowModal()")
    except Exception as e:
        tb = traceback.format_exc()
        _log(f"RoutingDialog __init__ failed:\n{tb}")
        wx.MessageBox(
            f"Failed to build the routing dialog:\n\n{e}\n\n"
            f"Full traceback in {_LOG_PATH}",
            "Dialog Error",
            wx.OK | wx.ICON_ERROR,
        )
        return 6

    try:
        # Give the dialog focus on macOS; ShowModal blocks until close.
        app.SetTopWindow(dlg)
        try:
            dlg.Raise()
        except Exception:
            pass
        dlg.ShowModal()
        _log("ShowModal returned cleanly")
    except Exception as e:
        tb = traceback.format_exc()
        _log(f"ShowModal failed:\n{tb}")
        wx.MessageBox(
            f"Routing dialog crashed:\n\n{e}\n\nSee {_LOG_PATH} for details.",
            "Dialog Crash",
            wx.OK | wx.ICON_ERROR,
        )
        return 7
    finally:
        # Snapshot settings (window size/pos + all dialog state) to disk
        # before destruction so the next launch can restore them.
        try:
            _save_settings(dlg.get_settings())
            _log(f"settings saved to {_SETTINGS_PATH}")
        except Exception as e:
            _log(f"could not capture settings on exit: {e}")
        try:
            dlg.Destroy()
        except Exception:
            pass
        _log("ipc_entry exiting")

    return 0


if __name__ == "__main__":
    sys.exit(main())
