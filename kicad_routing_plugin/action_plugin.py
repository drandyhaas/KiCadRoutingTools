"""
KiCad Routing Tools - SWIG Action Plugin

This provides a SWIG-based ActionPlugin that integrates with KiCad's
PCB Editor menu system.
"""

import os
import sys
import pcbnew
import wx

# Add parent directory to path for importing routing modules
PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
# #522 layout: the engine modules live in py_router/ under the repo root.
# The exists() guard keeps a FLAT installed layout (PCM zip) working too.
_ENGINE_DIR = os.path.join(ROOT_DIR, 'py_router')
if os.path.isdir(_ENGINE_DIR) and _ENGINE_DIR not in sys.path:
    sys.path.insert(0, _ENGINE_DIR)
# py_placer/ holds the placement package (placement.groups / .fanout_clearance
# are imported from here) and py_tools/ the instruments. Same exists() guard so
# a FLAT installed layout (PCM zip) keeps working.
for _sib in ('py_placer', 'py_tools'):
    _d = os.path.join(ROOT_DIR, _sib)
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.append(_d)

# #795: KiCad's hand-written __iter__ for GetTracks()/GetDrawings() calls the
# py2 `SwigPyIterator.next()`, dropped by current SWIG -- restore the alias at
# the GUI entry, before _get_selected_net_names() or the dialog walk a board.
try:
    from swig_compat import patch_swig_iterators as _patch_swig_iterators
    _patch_swig_iterators()
except ImportError:
    pass         # kicad_parser makes the same call, and it is what parses here


def _get_selected_net_names(board):
    """Return the set of net names that the user has selected in the PCB editor.

    Looks at the current selection (tracks, vias, pads, and zones) and collects
    the net of each selected item. Selecting any item that belongs to a net is
    treated as selecting that net for routing. Returns an empty set if nothing
    relevant is selected, so the dialog falls back to its normal behaviour.
    """
    net_names = set()
    try:
        selected_items = []

        # Tracks and vias
        for track in board.GetTracks():
            if track.IsSelected():
                selected_items.append(track)

        # Pads (including those on whole-footprint selections)
        for footprint in board.GetFootprints():
            fp_selected = footprint.IsSelected()
            for pad in footprint.Pads():
                if fp_selected or pad.IsSelected():
                    selected_items.append(pad)

        # Copper zones / planes
        for i in range(board.GetAreaCount()):
            zone = board.GetArea(i)
            if zone.IsSelected():
                selected_items.append(zone)

        for item in selected_items:
            try:
                name = item.GetNetname()
            except Exception:
                name = None
            # Ignore the unconnected/no-net pseudo-net (net code 0)
            if name and item.GetNetCode() > 0:
                net_names.add(name)
    except Exception:
        # Selection introspection is best-effort; never block the dialog.
        pass

    return net_names


class KiCadRoutingToolsPlugin(pcbnew.ActionPlugin):
    """Action Plugin for KiCad Routing Tools."""

    # Class-level storage for dialog settings (persists across invocations)
    _saved_settings = None
    _saved_board_filename = None

    def defaults(self):
        self.name = "KiCadRoutingTools"
        self.category = "Routing"
        self.description = "KiCadRoutingTools, by DrAndyHaas"
        self.show_toolbar_button = True
        icon_path = os.path.join(PLUGIN_DIR, "icon_24.png")
        if os.path.exists(icon_path):
            self.icon_file_name = icon_path

        # Pre-import modules so they're cached for faster startup on button click
        try:
            import kicad_parser
            import routing_defaults
        except Exception:
            pass

    def Run(self):
        """Called when the plugin is invoked from the menu or toolbar."""
        try:
            self._run_plugin()
        except Exception as e:
            wx.MessageBox(
                f"Error running KiCad Routing Tools:\n\n{e}",
                "Routing Error",
                wx.OK | wx.ICON_ERROR
            )

    def _run_plugin(self):
        """Main plugin logic."""
        # Ensure scipy/shapely are available (KiCad doesn't bundle them, and
        # PCM-installed plugins can't pip-install during install).
        from .deps_check import ensure_dependencies
        parent_for_deps = wx.GetTopLevelWindows()[0] if wx.GetTopLevelWindows() else None
        if not ensure_dependencies(parent_for_deps):
            return

        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox(
                "No board is currently open.\nPlease open a PCB file first.",
                "No Board",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Get the board filename (used for settings persistence and validation)
        board_filename = board.GetFileName() or ""

        # Import our modules
        from kicad_parser import build_pcb_data_from_board
        from .swig_gui import RoutingDialog

        # Build PCBData directly from pcbnew's in-memory board (fast, no file I/O)
        try:
            pcb_data = build_pcb_data_from_board(board)
        except Exception as e:
            wx.MessageBox(
                f"Failed to read board data:\n\n{e}",
                "Board Read Error",
                wx.OK | wx.ICON_ERROR
            )
            return

        # Collect the nets the user has selected in the PCB editor so the
        # dialog can pre-check them for routing (GitHub issue #6).
        preselected_nets = _get_selected_net_names(board)

        # Show the configuration dialog (modal)
        parent = wx.GetTopLevelWindows()[0] if wx.GetTopLevelWindows() else None

        # Check if we have saved settings for this board
        saved_settings = None
        if (KiCadRoutingToolsPlugin._saved_board_filename == board_filename
                and KiCadRoutingToolsPlugin._saved_settings is not None):
            saved_settings = KiCadRoutingToolsPlugin._saved_settings

        dlg = RoutingDialog(parent, pcb_data, board_filename,
                            saved_settings=saved_settings,
                            preselected_nets=preselected_nets)

        dlg.ShowModal()

        # Save settings before destroying the dialog
        KiCadRoutingToolsPlugin._saved_settings = dlg.get_settings()
        KiCadRoutingToolsPlugin._saved_board_filename = board_filename

        dlg.Destroy()

        # Refresh the board view
        pcbnew.Refresh()
