"""wx dialogs for Track Gloss errors and diagnostic reports."""

from __future__ import annotations

import logging

import wx

from .diagnostics import split_diagnostic_report


LOG = logging.getLogger("KiCadTrackGloss")


def warning_bell():
    """Use the same native warning bell KiCad invokes for invalid actions."""
    try:
        wx.Bell()
    except Exception:
        LOG.exception("Could not play the KiCad warning bell")


def _copy_text(text):
    if not wx.TheClipboard.Open():
        warning_bell()
        return
    try:
        wx.TheClipboard.SetData(wx.TextDataObject(text))
        try:
            wx.TheClipboard.Flush()
        except Exception:
            pass
    finally:
        wx.TheClipboard.Close()


def show_report(title, lines):
    """Show a selectable, resizable report inside KiCad."""
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
    copy_button = wx.Button(dialog, label="Copy")
    ok_button = wx.Button(dialog, wx.ID_OK)
    try:
        ok_button.SetDefault()
    except Exception:
        pass
    copy_button.Bind(wx.EVT_BUTTON, lambda _event: _copy_text(text))
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


def show_diagnostic_report(title, lines):
    """Show result, human-readable details, and JSON in separate tabs."""
    summary, details, json_lines = split_diagnostic_report(lines)
    tabs = (("Result", "\n".join(summary)),
            ("Details", "\n".join(details)),
            ("JSON", "\n".join(json_lines)))
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
    copy_tab_button = wx.Button(dialog, label="Copy tab")
    copy_all_button = wx.Button(dialog, label="Copy all")
    ok_button = wx.Button(dialog, wx.ID_OK)
    try:
        ok_button.SetDefault()
    except Exception:
        pass

    def copy_tab(_event):
        selection = notebook.GetSelection()
        if 0 <= selection < len(controls):
            _copy_text(controls[selection].GetValue())

    copy_tab_button.Bind(wx.EVT_BUTTON, copy_tab)
    copy_all_button.Bind(wx.EVT_BUTTON, lambda _event: _copy_text(all_text))
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


__all__ = ("show_diagnostic_report", "show_report", "warning_bell")
