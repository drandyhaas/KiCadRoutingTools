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


class KiCadRoutingToolsPlugin(pcbnew.ActionPlugin):
    """Action Plugin for KiCad Routing Tools."""

    # Class-level dialog reference (persists across Run calls)
    _dialog = None
    _dialog_board_filename = None

    def defaults(self):
        self.name = "KiCadRoutingTools"
        self.category = "Routing"
        self.description = "KiCadRoutingTools, by DrAndyHaas"
        self.show_toolbar_button = True
        icon_path = os.path.join(PLUGIN_DIR, "icon_512.png")
        if os.path.exists(icon_path):
            self.icon_file_name = icon_path

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
        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox(
                "No board is currently open.\nPlease open a PCB file first.",
                "No Board",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Get the board filename
        board_filename = board.GetFileName()
        if not board_filename:
            wx.MessageBox(
                "Please save the board before routing.",
                "Board Not Saved",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Check if we have an existing dialog for this board
        cls = KiCadRoutingToolsPlugin
        if cls._dialog is not None:
            # Check if dialog is still valid and for the same board
            try:
                if cls._dialog_board_filename == board_filename:
                    # Re-show the existing dialog (preserves all settings)
                    cls._dialog.Show()
                    cls._dialog.Raise()
                    # Refresh board state in case user made changes in KiCad
                    cls._dialog.refresh_from_board()
                    return
                else:
                    # Different board - destroy old dialog
                    cls._dialog.Destroy()
                    cls._dialog = None
            except RuntimeError:
                # Dialog was destroyed externally
                cls._dialog = None

        # Import our modules
        from kicad_parser import parse_kicad_pcb
        from .swig_gui import RoutingDialog

        # Parse the board file to get our PCBData structure
        try:
            pcb_data = parse_kicad_pcb(board_filename)
        except Exception as e:
            wx.MessageBox(
                f"Failed to parse board file:\n\n{e}",
                "Parse Error",
                wx.OK | wx.ICON_ERROR
            )
            return

        # Create and show the dialog (non-modal so user can interact with KiCad)
        parent = wx.GetTopLevelWindows()[0] if wx.GetTopLevelWindows() else None
        cls._dialog = RoutingDialog(parent, pcb_data, board_filename)
        cls._dialog_board_filename = board_filename

        # Bind close event to hide instead of destroy
        cls._dialog.Bind(wx.EVT_CLOSE, self._on_dialog_close)

        cls._dialog.Show()

    def _on_dialog_close(self, event):
        """Hide the dialog instead of destroying it."""
        cls = KiCadRoutingToolsPlugin
        if cls._dialog:
            cls._dialog.Hide()
        # Don't call event.Skip() - we don't want the default close behavior
