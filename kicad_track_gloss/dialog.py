"""Small modal settings dialog; imported only inside KiCad."""

from __future__ import annotations

from dataclasses import dataclass

import wx


@dataclass
class GlossSettings:
    allow_simplify: bool = False
    min_gain: float = 0.01
    run_drc: bool = True
    preview: bool = True


class GlossDialog(wx.Dialog):
    def __init__(self, parent, saved=None):
        super().__init__(parent, title="KiCad Track Gloss")
        saved = saved or GlossSettings()
        panel = wx.Panel(self)
        mode_label = wx.StaticText(panel, label="Mode")
        self.mode = wx.Choice(panel, choices=["Shorten only", "Shorten or simplify"])
        self.mode.SetSelection(1 if saved.allow_simplify else 0)
        gain_label = wx.StaticText(panel, label="Minimum gain (mm)")
        self.gain = wx.SpinCtrlDouble(panel, min=0.0, max=10.0, inc=0.01,
                                     initial=saved.min_gain)
        self.gain.SetDigits(3)
        self.drc = wx.CheckBox(panel, label="Validate with official KiCad DRC")
        self.drc.SetValue(saved.run_drc)
        self.preview = wx.CheckBox(panel, label="Preview summary before applying")
        self.preview.SetValue(saved.preview)
        grid = wx.FlexGridSizer(2, 2, 8, 12)
        grid.Add(mode_label, 0, wx.ALIGN_CENTER_VERTICAL)
        grid.Add(self.mode, 1, wx.EXPAND)
        grid.Add(gain_label, 0, wx.ALIGN_CENTER_VERTICAL)
        grid.Add(self.gain, 1, wx.EXPAND)
        grid.AddGrowableCol(1)
        buttons = self.CreateButtonSizer(wx.OK | wx.CANCEL)
        outer = wx.BoxSizer(wx.VERTICAL)
        outer.Add(grid, 0, wx.EXPAND | wx.ALL, 12)
        outer.Add(self.drc, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 12)
        outer.Add(self.preview, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 12)
        outer.Add(buttons, 0, wx.EXPAND | wx.ALL, 8)
        panel.SetSizer(outer)
        frame = wx.BoxSizer(wx.VERTICAL)
        frame.Add(panel, 1, wx.EXPAND)
        self.SetSizerAndFit(frame)

    def settings(self):
        return GlossSettings(self.mode.GetSelection() == 1, float(self.gain.GetValue()),
                             bool(self.drc.GetValue()), bool(self.preview.GetValue()))

