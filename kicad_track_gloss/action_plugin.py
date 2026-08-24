"""KiCad 10 SWIG ActionPlugin entry point for selection-seeded track gloss."""

from __future__ import annotations

import logging
import os
import traceback

import pcbnew
import wx

from .engine import (find_pad_terminal_targets, find_track_terminal_vertices,
                     generate_candidate_plans, summarize_plan)
from .kicad import BoardAdapter
from .kicad.diagnostics import (append_plan_statistics,
                                append_search_statistics)
from .version import __version__


PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
LOG = logging.getLogger("KiCadTrackGloss")

# One-click policy: no dialog, preview, temporary board, DRC subprocess, or
# success/no-op popup. KiCad's native rules still constrain candidate search.
MIN_GAIN_MM = 0.01
ALLOW_EQUAL_LENGTH_SIMPLIFICATION = True


def _show_report(title, lines):
    """Show a selectable, resizable diagnostic report inside KiCad."""
    dialog = wx.Dialog(None, title=title, size=(760, 520),
                       style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
    layout = wx.BoxSizer(wx.VERTICAL)
    report = wx.TextCtrl(dialog, value="\n".join(lines),
                         style=wx.TE_MULTILINE | wx.TE_READONLY | wx.HSCROLL)
    try:
        report.SetFont(wx.Font(wx.FontInfo(9).Family(wx.FONTFAMILY_TELETYPE)))
    except Exception:
        pass
    layout.Add(report, 1, wx.EXPAND | wx.ALL, 10)
    buttons = dialog.CreateButtonSizer(wx.OK)
    if buttons is not None:
        layout.Add(buttons, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)
    dialog.SetSizer(layout)
    dialog.CentreOnParent()
    try:
        dialog.ShowModal()
    finally:
        dialog.Destroy()


def _warning_bell():
    """Use the same native warning bell KiCad invokes for invalid actions."""
    try:
        wx.Bell()
    except Exception:
        LOG.exception("Could not play the KiCad warning bell")


def _selection_counts(board):
    counts = {"segments": 0, "arcs": 0, "vias": 0, "other": 0}
    for item in board.GetTracks():
        try:
            if not item.IsSelected():
                continue
            kind = str(item.GetClass())
        except Exception:
            continue
        if kind == "PCB_TRACK":
            counts["segments"] += 1
        elif kind == "PCB_ARC":
            counts["arcs"] += 1
        elif kind == "PCB_VIA":
            counts["vias"] += 1
        else:
            counts["other"] += 1
    return counts


class KiCadTrackGlossPlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "KiCad Track Gloss"
        self.category = "Routing"
        self.description = ("Gloss one or more selected track segments, "
                            "connections, or complete nets")
        self.show_toolbar_button = True
        self.icon_file_name = os.path.join(PLUGIN_DIR, "icon_24.png")
        dark = os.path.join(PLUGIN_DIR, "icon_24_dark.png")
        if os.path.exists(dark):
            self.dark_icon_file_name = dark

    def Run(self):
        try:
            changed = self._run([])
        except Exception:
            _warning_bell()
            LOG.exception("Track gloss failed; the board was left unchanged")
            try:
                _show_report("KiCad Track Gloss — Error", [
                    "UNEXPECTED ERROR",
                    "Plugin version: " + __version__,
                    "The operation was aborted; in-memory rollback was requested.",
                    "",
                    traceback.format_exc(),
                ])
            except Exception:
                LOG.exception("Could not display the Track Gloss error report")
        else:
            if not changed:
                _warning_bell()

    def _run(self, report, diagnostic=False):
        report.append("Plugin version: " + __version__)
        board = pcbnew.GetBoard()
        if board is None:
            report.append("Result: no active PCB board.")
            return False
        try:
            report.append("KiCad version: " + str(pcbnew.Version()))
        except Exception:
            pass
        counts = _selection_counts(board)
        report.append(
            "Selected objects: {segments} straight segment(s), {arcs} arc(s), "
            "{vias} via(s), {other} other.".format(**counts))
        adapter = BoardAdapter(pcbnew)
        try:
            snapshot = adapter.snapshot(board)
        except ValueError as error:
            report.append("Result: selection rejected.")
            report.append("Reason: " + str(error))
            return False
        report.append("Eligible straight segments: " + str(len(snapshot.eligible_keys)))
        report.append("Automatic connection expansion: {} seed(s) + {} segment(s).".format(
            snapshot.selection_seed_count, snapshot.auto_expanded_count))
        report.append("Protected tuned segments: " +
                      str(snapshot.tuned_protected_count))
        for warning in snapshot.warnings:
            report.append("Protection: " + warning)
        track_terminals = find_track_terminal_vertices(
            snapshot.model, snapshot.eligible_keys)
        pad_terminals = find_pad_terminal_targets(
            snapshot.model, snapshot.eligible_keys)
        report.append("Sliding track-intersection terminations: " +
                      str(len(track_terminals)))
        report.append("Sliding pad-area terminations: " +
                      str(len(pad_terminals)))
        if (len(snapshot.eligible_keys) < 2 and not track_terminals and
                not pad_terminals):
            report.append("Result: no modification.")
            report.append(
                "Reason: automatic connection expansion did not find a second eligible "
                "straight segment or a sliding track/pad termination.")
            return False

        plans = generate_candidate_plans(
            snapshot.model, snapshot.eligible_keys,
            min_gain=MIN_GAIN_MM,
            allow_equal_length_simpler=ALLOW_EQUAL_LENGTH_SIMPLIFICATION,
            clearance=snapshot.minimum_clearance,
            collect_statistics=diagnostic)
        report.append("Candidate plans evaluated: " + str(len(plans)))
        report.append("Connected chains considered: " +
                      str(max((plan.chains_considered for plan in plans), default=0)))
        searched_plan = plans[0] if plans else None
        plans = [plan for plan in plans if plan.changed]
        if not plans:
            if diagnostic and searched_plan is not None:
                append_search_statistics(report, searched_plan.search_counts)
            report.append("Result: no safe improvement found.")
            report.append(
                "Possible reasons: disconnected selection, fixed junction, locked/tuned "
                "track, insufficient length gain, clearance, pad, via, keepout, or board edge.")
            return False
        best = plans[0]
        report.append("Chosen plan: remove {} segment(s), add {} segment(s).".format(
            len(best.remove_keys), len(best.additions)))
        report.append("Copper length saved: {:.3f} mm.".format(best.saved_mm))
        if diagnostic:
            append_plan_statistics(report, summarize_plan(
                snapshot.model, snapshot.eligible_keys, best))
        adapter.apply(board, best, rollback_on_error=True)
        try:
            board.SetModified()
        except Exception:
            pass
        pcbnew.Refresh()
        report.append("Result: modification applied to the current board.")
        report.append("Use KiCad Undo to revert it.")
        return True


class KiCadTrackGlossDiagnosticPlugin(KiCadTrackGlossPlugin):
    def defaults(self):
        self.name = "KiCad Track Gloss — Diagnostic"
        self.category = "Routing"
        self.description = ("Run Track Gloss and display a detailed diagnostic "
                            "report, including no-op reasons")
        self.show_toolbar_button = False
        self.icon_file_name = os.path.join(PLUGIN_DIR, "icon_24.png")
        dark = os.path.join(PLUGIN_DIR, "icon_24_dark.png")
        if os.path.exists(dark):
            self.dark_icon_file_name = dark

    def Run(self):
        report = ["KiCad Track Gloss diagnostic", ""]
        try:
            changed = self._run(report, diagnostic=True)
        except Exception:
            _warning_bell()
            LOG.exception("Track gloss diagnostic run failed")
            report.extend([
                "",
                "UNEXPECTED ERROR",
                "The operation was aborted; in-memory rollback was requested.",
                "",
                traceback.format_exc(),
            ])
        else:
            if not changed:
                _warning_bell()
        try:
            _show_report("KiCad Track Gloss — Diagnostic", report)
        except Exception:
            LOG.exception("Could not display the Track Gloss diagnostic report")
