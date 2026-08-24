"""KiCad 10 SWIG ActionPlugin entry point for selected-track gloss."""

from __future__ import annotations

import os

import pcbnew
import wx

from .board_adapter import BoardAdapter
from .dialog import GlossDialog, GlossSettings
from .drc_validation import validate_on_copy
from .gloss_engine import smooth_selected_chains


PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))


class KiCadTrackGlossPlugin(pcbnew.ActionPlugin):
    _saved_settings = GlossSettings()

    def defaults(self):
        self.name = "KiCad Track Gloss"
        self.category = "Routing"
        self.description = ("Shorten and simplify selected PCB track chains "
                            "while preserving connectivity and design rules")
        self.show_toolbar_button = True
        self.icon_file_name = os.path.join(PLUGIN_DIR, "icon_24.png")
        dark = os.path.join(PLUGIN_DIR, "icon_24_dark.png")
        if os.path.exists(dark):
            self.dark_icon_file_name = dark

    def Run(self):
        try:
            self._run()
        except Exception as exc:
            wx.MessageBox(str(exc), "KiCad Track Gloss", wx.OK | wx.ICON_ERROR)

    def _run(self):
        board = pcbnew.GetBoard()
        if board is None:
            raise RuntimeError("Open a PCB in the PCB Editor first.")
        adapter = BoardAdapter(pcbnew)
        snapshot = adapter.snapshot(board)
        if len(snapshot.eligible_keys) < 2:
            raise ValueError("Select a connected chain containing at least two eligible straight segments.")
        parent = wx.GetTopLevelWindows()[0] if wx.GetTopLevelWindows() else None
        dialog = GlossDialog(parent, self.__class__._saved_settings)
        try:
            if dialog.ShowModal() != wx.ID_OK:
                return
            settings = dialog.settings()
            self.__class__._saved_settings = settings
        finally:
            dialog.Destroy()

        result = smooth_selected_chains(
            snapshot.model, snapshot.eligible_keys,
            min_gain=settings.min_gain,
            allow_equal_length_simpler=settings.allow_simplify)
        if not result.changed:
            detail = "\n".join(snapshot.warnings)
            message = "No safe improvement was found; the board was not changed."
            if detail:
                message += "\n\n" + detail
            wx.MessageBox(message, "KiCad Track Gloss", wx.OK | wx.ICON_INFORMATION)
            return

        summary = ("Candidate result:\n"
                   "• segments removed: {}\n"
                   "• segments added: {}\n"
                   "• length saved: {:.3f} mm\n"
                   "• chains changed: {}".format(
                       len(result.remove_keys), len(result.additions),
                       result.saved_mm, result.chains_changed))
        if snapshot.warnings:
            summary += "\n\nProtected items:\n" + "\n".join(snapshot.warnings)
        if settings.preview:
            if wx.MessageBox(summary + "\n\nApply this gloss?", "KiCad Track Gloss",
                             wx.YES_NO | wx.NO_DEFAULT | wx.ICON_QUESTION) != wx.YES:
                return
        if settings.run_drc:
            validate_on_copy(pcbnew, board, result)

        # Complete validation precedes mutation. RemoveNative is mandatory in
        # SWIG; BoardAdapter restores every removed track on any exception.
        adapter.apply(board, result, rollback_on_error=True)
        pcbnew.Refresh()
        wx.MessageBox(summary + "\n\nApplied successfully.", "KiCad Track Gloss",
                      wx.OK | wx.ICON_INFORMATION)

