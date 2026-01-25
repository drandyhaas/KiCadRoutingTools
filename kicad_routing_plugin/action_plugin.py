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

    def defaults(self):
        self.name = "Auto-routing tools"
        self.category = "Routing"
        self.description = "Advanced auto-routing tools, by DrAndyHaas"
        self.show_toolbar_button = True
        icon_path = os.path.join(PLUGIN_DIR, "icon.png")
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

        # Show the configuration dialog
        parent = wx.GetTopLevelWindows()[0] if wx.GetTopLevelWindows() else None
        dlg = RoutingDialog(parent, pcb_data, board_filename)

        result = dlg.ShowModal()
        dlg.Destroy()

        if result == wx.ID_OK:
            # Refresh the board view
            pcbnew.Refresh()
