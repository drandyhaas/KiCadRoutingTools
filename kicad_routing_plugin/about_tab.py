"""
KiCad Routing Tools - About Tab

Displays version info, author, and links.
"""

import os
import subprocess
from datetime import datetime
import wx
import wx.adv


# Directory paths
PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)


class AboutTab(wx.Panel):
    """About tab panel with version info, author, and links."""

    def __init__(self, parent, on_reset_settings=None):
        super().__init__(parent)
        self.on_reset_settings = on_reset_settings
        self._create_ui()

    def _create_ui(self):
        """Create the About tab UI."""
        about_sizer = wx.BoxSizer(wx.VERTICAL)

        # Add some top padding
        about_sizer.AddSpacer(20)

        # Icon with text for about tab
        icon_path = os.path.join(PLUGIN_DIR, "icon_512_text.png")

        if os.path.exists(icon_path):
            try:
                img = wx.Image(icon_path, wx.BITMAP_TYPE_PNG)
                # Scale down for display
                img = img.Scale(256, 256, wx.IMAGE_QUALITY_HIGH)
                bitmap = wx.StaticBitmap(self, bitmap=wx.Bitmap(img))
                about_sizer.Add(bitmap, 0, wx.ALIGN_CENTER | wx.ALL, 10)
            except Exception:
                pass  # Skip icon if loading fails

        # Project name
        title = wx.StaticText(self, label="KiCadRoutingTools")
        title_font = title.GetFont()
        title_font.SetPointSize(18)
        title_font.SetWeight(wx.FONTWEIGHT_BOLD)
        title.SetFont(title_font)
        about_sizer.Add(title, 0, wx.ALIGN_CENTER | wx.ALL, 5)

        # Subtitle
        subtitle = wx.StaticText(self, label="Rust-accelerated A* autorouter for KiCad")
        about_sizer.Add(subtitle, 0, wx.ALIGN_CENTER | wx.BOTTOM, 15)

        # Get git version info
        version_str, commit_date = self._get_git_info()

        # Info grid
        info_panel = wx.Panel(self)
        info_sizer = wx.FlexGridSizer(cols=2, hgap=15, vgap=8)

        def add_info_row(label, value):
            lbl = wx.StaticText(info_panel, label=label)
            lbl.SetFont(lbl.GetFont().Bold())
            info_sizer.Add(lbl, 0, wx.ALIGN_RIGHT)
            val = wx.StaticText(info_panel, label=value)
            info_sizer.Add(val, 0, wx.ALIGN_LEFT)

        add_info_row("Version:", version_str)
        add_info_row("Release Date:", commit_date)
        add_info_row("Author:", "DrAndyHaas")

        info_panel.SetSizer(info_sizer)
        about_sizer.Add(info_panel, 0, wx.ALIGN_CENTER | wx.ALL, 10)

        # GitHub link
        github_url = "https://github.com/drandyhaas/KiCadRoutingTools"
        github_link = wx.adv.HyperlinkCtrl(
            self,
            label="GitHub Repository",
            url=github_url
        )
        about_sizer.Add(github_link, 0, wx.ALIGN_CENTER | wx.ALL, 10)

        # License/copyright
        copyright_text = wx.StaticText(
            self,
            label="Open source - see repository for license details"
        )
        copyright_text.SetForegroundColour(wx.Colour(128, 128, 128))
        about_sizer.Add(copyright_text, 0, wx.ALIGN_CENTER | wx.TOP, 20)

        # Reset settings button
        about_sizer.AddStretchSpacer()
        reset_btn = wx.Button(self, label="Reset All Settings to Defaults")
        reset_btn.SetToolTip("Clear log, selections, and reset all parameters to defaults")
        reset_btn.Bind(wx.EVT_BUTTON, self._on_reset_settings)
        about_sizer.Add(reset_btn, 0, wx.ALIGN_CENTER | wx.ALL, 20)

        self.SetSizer(about_sizer)

    def _on_reset_settings(self, event):
        """Handle reset settings button click."""
        result = wx.MessageBox(
            "This will reset all settings to defaults, clear the log, "
            "and uncheck all net selections.\n\nContinue?",
            "Reset Settings",
            wx.YES_NO | wx.ICON_WARNING
        )
        if result == wx.YES and self.on_reset_settings:
            self.on_reset_settings()

    def _get_git_info(self):
        """Get version and date from git."""
        version_str = "Unknown"
        commit_date = "Unknown"

        try:
            # Get version tag/describe
            result = subprocess.run(
                ["git", "describe", "--tags", "--always"],
                cwd=ROOT_DIR,
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                version_str = result.stdout.strip()

            # Get commit date
            result = subprocess.run(
                ["git", "log", "-1", "--format=%ci"],
                cwd=ROOT_DIR,
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0:
                date_str = result.stdout.strip()
                # Parse and reformat the date
                try:
                    dt = datetime.strptime(date_str[:19], "%Y-%m-%d %H:%M:%S")
                    commit_date = dt.strftime("%B %d, %Y")
                except Exception:
                    commit_date = date_str[:10]  # Just the date part
        except Exception:
            pass  # Keep defaults if git fails

        return version_str, commit_date
