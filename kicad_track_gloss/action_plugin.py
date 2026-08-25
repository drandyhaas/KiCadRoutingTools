"""KiCad 10 SWIG ActionPlugin entry point for selection-seeded track gloss."""

from __future__ import annotations

import logging
import os
import traceback

import pcbnew
import wx

from .engine import (find_pad_terminal_targets, find_track_terminal_vertices,
                     generate_converged_plan, summarize_plan)
from .engine.model import segment_key
from .kicad import BoardAdapter
from .kicad.diagnostics import (append_plan_statistics,
                                 append_search_statistics,
                                 split_diagnostic_report)
from .version import __version__


PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
LOG = logging.getLogger("KiCadTrackGloss")

# One-click policy: no dialog, preview, temporary board, DRC subprocess, or
# success/no-op popup. KiCad's native rules still constrain candidate search.
MIN_GAIN_MM = 0.01
ALLOW_EQUAL_LENGTH_SIMPLIFICATION = True


class NoTrackSelection(ValueError):
    """Normal user condition handled by a focused selection warning."""


def _show_selection_warning():
    dialog = wx.MessageDialog(
        None,
        "Select at least one straight track segment before running Track Gloss.",
        "KiCad Track Gloss",
        wx.OK | wx.ICON_WARNING)
    try:
        dialog.ShowModal()
    finally:
        dialog.Destroy()


def _show_report(title, lines):
    """Show a selectable, resizable diagnostic report inside KiCad."""
    text = "\n".join(lines)
    dialog = wx.Dialog(None, title=title, size=(760, 520),
                       style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
    layout = wx.BoxSizer(wx.VERTICAL)
    report = wx.TextCtrl(dialog, value=text,
                         style=wx.TE_MULTILINE | wx.TE_READONLY | wx.HSCROLL)
    try:
        report.SetFont(wx.Font(wx.FontInfo(9).Family(wx.FONTFAMILY_TELETYPE)))
    except Exception:
        pass
    layout.Add(report, 1, wx.EXPAND | wx.ALL, 10)
    buttons = wx.BoxSizer(wx.HORIZONTAL)
    copy_button = wx.Button(dialog, label="Copier")
    ok_button = wx.Button(dialog, wx.ID_OK)
    try:
        ok_button.SetDefault()
    except Exception:
        pass

    def copy_report(_event):
        if not wx.TheClipboard.Open():
            _warning_bell()
            return
        try:
            wx.TheClipboard.SetData(wx.TextDataObject(text))
            try:
                wx.TheClipboard.Flush()
            except Exception:
                pass
        finally:
            wx.TheClipboard.Close()

    copy_button.Bind(wx.EVT_BUTTON, copy_report)
    buttons.Add(copy_button, 0)
    buttons.AddStretchSpacer(1)
    buttons.Add(ok_button, 0)
    layout.Add(buttons, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)
    dialog.SetSizer(layout)
    dialog.CentreOnParent()
    try:
        dialog.ShowModal()
    finally:
        dialog.Destroy()


def _show_diagnostic_report(title, lines):
    """Show result, human-readable details, and JSON in separate tabs."""
    summary, details, json_lines = split_diagnostic_report(lines)
    tabs = (
        ("Résultat", "\n".join(summary)),
        ("Détails", "\n".join(details)),
        ("JSON", "\n".join(json_lines)),
    )
    all_text = "\n\n".join(
        "=== {} ===\n{}".format(label, text) for label, text in tabs)
    dialog = wx.Dialog(None, title=title, size=(820, 600),
                       style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
    layout = wx.BoxSizer(wx.VERTICAL)
    notebook = wx.Notebook(dialog)
    controls = []
    for label, text in tabs:
        panel = wx.Panel(notebook)
        panel_layout = wx.BoxSizer(wx.VERTICAL)
        control = wx.TextCtrl(
            panel, value=text,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.HSCROLL)
        try:
            control.SetFont(wx.Font(
                wx.FontInfo(9).Family(wx.FONTFAMILY_TELETYPE)))
        except Exception:
            pass
        panel_layout.Add(control, 1, wx.EXPAND | wx.ALL, 8)
        panel.SetSizer(panel_layout)
        notebook.AddPage(panel, label)
        controls.append(control)
    layout.Add(notebook, 1, wx.EXPAND | wx.ALL, 10)

    buttons = wx.BoxSizer(wx.HORIZONTAL)
    copy_tab_button = wx.Button(dialog, label="Copier l'onglet")
    copy_all_button = wx.Button(dialog, label="Copier tout")
    ok_button = wx.Button(dialog, wx.ID_OK)
    try:
        ok_button.SetDefault()
    except Exception:
        pass

    def copy_text(text):
        if not wx.TheClipboard.Open():
            _warning_bell()
            return
        try:
            wx.TheClipboard.SetData(wx.TextDataObject(text))
            try:
                wx.TheClipboard.Flush()
            except Exception:
                pass
        finally:
            wx.TheClipboard.Close()

    def copy_tab(_event):
        selection = notebook.GetSelection()
        if 0 <= selection < len(controls):
            copy_text(controls[selection].GetValue())

    copy_tab_button.Bind(wx.EVT_BUTTON, copy_tab)
    copy_all_button.Bind(wx.EVT_BUTTON, lambda _event: copy_text(all_text))
    buttons.Add(copy_tab_button, 0)
    buttons.Add(copy_all_button, 0, wx.LEFT, 6)
    buttons.AddStretchSpacer(1)
    buttons.Add(ok_button, 0)
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

    other_collections = []
    for alternatives in (("GetFootprints",), ("GetDrawings",),
                         ("Zones", "GetZones")):
        for accessor in alternatives:
            try:
                other_collections.append(getattr(board, accessor)())
                break
            except Exception:
                continue
    try:
        other_collections.extend(footprint.Pads()
                                 for footprint in board.GetFootprints())
    except Exception:
        pass
    for values in other_collections:
        for item in values:
            try:
                if item.IsSelected():
                    counts["other"] += 1
            except Exception:
                continue
    return counts


def _eligible_net_names(model, eligible_keys):
    eligible = set(eligible_keys)
    labels = {segment.net_name or "net {}".format(segment.net_id)
              for segment in model.segments
              if segment_key(segment) in eligible}
    return sorted(labels)


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
        except NoTrackSelection:
            try:
                _show_selection_warning()
            except Exception:
                LOG.exception("Could not display the track-selection warning")
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
        report.append(
            "Optimization coordinates: exact copper geometry; active KiCad grid not used.")
        counts = _selection_counts(board)
        report.append(
            "Selected objects: {segments} straight segment(s), {arcs} arc(s), "
            "{vias} via(s), {other} other.".format(**counts))
        if counts["segments"] == 0 and not diagnostic:
            raise NoTrackSelection(
                "Select at least one straight track segment before running Track Gloss.")
        adapter = BoardAdapter(pcbnew)
        try:
            snapshot = adapter.snapshot(board)
        except ValueError as error:
            report.append("Result: selection rejected.")
            report.append("Reason: " + str(error))
            return False
        report.append("Eligible straight segments: " + str(len(snapshot.eligible_keys)))
        net_names = _eligible_net_names(snapshot.model, snapshot.eligible_keys)
        report.append("Eligible net(s) ({}): {}".format(
            len(net_names), ", ".join(net_names) if net_names else "none"))
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

        best = generate_converged_plan(
            snapshot.model, snapshot.eligible_keys,
            min_gain=MIN_GAIN_MM,
            allow_equal_length_simpler=ALLOW_EQUAL_LENGTH_SIMPLIFICATION,
            clearance=snapshot.minimum_clearance,
            collect_statistics=diagnostic,
            parallel=True)
        report.append("Convergence passes: " + str(best.convergence_passes))
        report.append("Fixed point reached: " +
                      ("yes" if best.fixed_point else "no"))
        report.append("Connected chains considered: " +
                      str(best.chains_considered))
        if not best.changed:
            if diagnostic:
                append_search_statistics(
                    report, best.search_counts, best.blocking_nets)
            report.append("Result: no safe improvement found.")
            report.append(
                "Possible reasons: disconnected selection, fixed junction, locked/tuned "
                "track, insufficient length gain, clearance, pad, via, keepout, or board edge.")
            return False
        report.append("Chosen plan: remove {} segment(s), add {} segment(s).".format(
            len(best.remove_keys), len(best.additions)))
        report.append("Copper length saved: {:.3f} mm.".format(best.saved_mm))
        report.append("Non-octolinear segments corrected: {}.".format(
            best.angle_corrections))
        if diagnostic:
            append_plan_statistics(report, summarize_plan(
                snapshot.model, snapshot.eligible_keys, best))
        adapter.apply(board, best, rollback_on_error=True)
        try:
            board.SetModified()
        except Exception:
            pass
        pcbnew.Refresh()
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
            _show_diagnostic_report("KiCad Track Gloss — Diagnostic", report)
        except Exception:
            LOG.exception("Could not display the Track Gloss diagnostic report")
