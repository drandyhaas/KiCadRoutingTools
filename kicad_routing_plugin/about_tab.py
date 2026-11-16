"""
KiCad Routing Tools - About Tab

Displays version info, author, and links.
"""

import math
import os
import time
import wx
import wx.adv


# Directory paths
PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)

# Where the Donate button sends people. Ko-fi is primary because a donor can
# pay there by card WITHOUT creating an account (PayPal.Me requires the sender
# to have a PayPal account); GitHub Sponsors and PayPal stay as secondary links.
# The same URLs appear in README.md, .github/FUNDING.yml (GitHub's Sponsor
# button) and metadata.json's resources -- change them together.
DONATE_URL = "https://ko-fi.com/drandyhaas"
SPONSORS_URL = "https://github.com/sponsors/drandyhaas"
PAYPAL_URL = "https://www.paypal.me/DrAndyHaas"

DONATE_TOOLTIP = (
    "Support KiCadRoutingTools.\n\n"
    "Donations go to what it costs to build this: about $500/month of\n"
    "cloud computing and AI-assisted development. The compute\n"
    "regression-tests the router against a corpus of real\n"
    "open-source boards.\n\n"
    + DONATE_URL
)

# The heart from docs/donate_button.svg, in that path's own 24-unit box.
_HEART = [
    ('M', 12, 21.35), ('L', 10.55, 20.03),
    ('C', 5.4, 15.36, 2, 12.28, 2, 8.5),
    ('C', 2, 5.42, 4.42, 3, 7.5, 3),
    ('C', 9.24, 3, 10.91, 3.81, 12, 5.09),
    ('C', 13.09, 3.81, 14.76, 3, 16.5, 3),
    ('C', 19.58, 3, 22, 5.42, 22, 8.5),
    ('C', 22, 12.28, 18.6, 15.36, 13.45, 20.04),
]


def _pill_polygon(w, top, h, steps=24):
    """A w x h pill (fully rounded ends) as a convex polygon."""
    r = h / 2.0
    cy = top + r
    pts = []
    for i in range(steps + 1):          # right cap, top to bottom
        a = -math.pi / 2 + math.pi * i / steps
        pts.append((w - r + r * math.cos(a), cy + r * math.sin(a)))
    for i in range(steps + 1):          # left cap, bottom to top
        a = math.pi / 2 + math.pi * i / steps
        pts.append((r + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _clip_halfplane(poly, nx, ny, c, keep_above):
    """Clip a convex polygon to n.p >= c (keep_above) or n.p <= c."""
    sign = 1.0 if keep_above else -1.0
    out = []
    for i, p in enumerate(poly):
        q = poly[(i + 1) % len(poly)]
        dp = sign * (nx * p[0] + ny * p[1] - c)
        dq = sign * (nx * q[0] + ny * q[1] - c)
        if dp >= 0:
            out.append(p)
        if (dp >= 0) != (dq >= 0):
            s = dp / (dp - dq)
            out.append((p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1])))
    return out


class ShimmerButton(wx.Control):
    """A blue pill with a highlight that sweeps across it every few seconds.

    Drawn to match the README's docs/donate_button.svg: same colours, heart
    and cycle. Custom-drawn because no native button animates, and because a
    native macOS button ignores the height it is given (its bezel is drawn at
    a fixed height and centred, so a tall one still LOOKS small). Sends
    wx.EVT_BUTTON on a click, or on Space/Enter when it has focus.

    Every colour, pen and brush is built explicitly, never wx.WHITE,
    wx.TRANSPARENT_PEN or another stock object: wxPython initialises those
    only when PYTHON creates the wx.App, and inside KiCad the app is KiCad's,
    so there they are invalid (the label drew black and the heart not at all).
    """

    PERIOD_S = 3.6      # one sweep plus the pause after it
    SWEEP = 0.38        # fraction of the period the highlight is moving
    FRAME_MS = 30
    FACE = ((27, 138, 214), (0, 112, 186))          # gradient, top to bottom
    FACE_HOVER = ((52, 156, 226), (12, 126, 200))
    LIP = (0, 79, 133)                              # the darker edge below
    LEAN_DEG = 22       # the band leans like the SVG's skewX(-22deg)

    def __init__(self, parent, label, size):
        super().__init__()
        # macOS composites every window, so the corners outside the pill can
        # simply be left unpainted. Elsewhere the control paints its own
        # background (see _backdrop). The style must be set before Create().
        self._transparent = wx.Platform == '__WXMAC__'
        self.SetBackgroundStyle(wx.BG_STYLE_TRANSPARENT if self._transparent
                                else wx.BG_STYLE_PAINT)
        self.Create(parent, style=wx.BORDER_NONE)
        self._label = label
        self._hover = False
        self._pressed = False
        self._was_moving = False
        self._t0 = time.monotonic()
        self.SetInitialSize(self.FromDIP(wx.Size(*size)))
        self.SetCursor(wx.Cursor(wx.CURSOR_HAND))

        self.Bind(wx.EVT_PAINT, self._on_paint)
        self.Bind(wx.EVT_ENTER_WINDOW, lambda e: self._set_state(hover=True))
        self.Bind(wx.EVT_LEAVE_WINDOW,
                  lambda e: self._set_state(hover=False, pressed=False))
        self.Bind(wx.EVT_LEFT_DOWN, lambda e: self._set_state(pressed=True))
        self.Bind(wx.EVT_LEFT_UP, self._on_left_up)
        self.Bind(wx.EVT_CHAR, self._on_char)
        self.Bind(wx.EVT_SET_FOCUS, lambda e: self.Refresh(False))
        self.Bind(wx.EVT_KILL_FOCUS, lambda e: self.Refresh(False))
        self.Bind(wx.EVT_WINDOW_DESTROY, self._on_destroy)
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self._on_tick, self._timer)
        self._timer.Start(self.FRAME_MS)

    def AcceptsFocusFromKeyboard(self):
        return self.IsEnabled()

    def _set_state(self, hover=None, pressed=None):
        if hover is not None:
            self._hover = hover
        if pressed is not None:
            self._pressed = pressed
        self.Refresh(False)

    def _on_left_up(self, event):
        if self._pressed:
            self._set_state(pressed=False)
            self._click()

    def _on_char(self, event):
        if event.GetKeyCode() in (wx.WXK_SPACE, wx.WXK_RETURN, wx.WXK_NUMPAD_ENTER):
            self._click()
        else:
            event.Skip()

    def _click(self):
        evt = wx.CommandEvent(wx.wxEVT_BUTTON, self.GetId())
        evt.SetEventObject(self)
        self.GetEventHandler().ProcessEvent(evt)

    def _on_destroy(self, event):
        if event.GetEventObject() is self:
            self._timer.Stop()
        event.Skip()

    def _phase(self):
        return ((time.monotonic() - self._t0) % self.PERIOD_S) / self.PERIOD_S

    def _on_tick(self, event):
        # Repaint only while the band is moving (plus one frame to clear it),
        # and never while the About tab is not the one on screen.
        if not self.IsShownOnScreen():
            return
        moving = self._phase() < self.SWEEP
        if moving or self._was_moving:
            self.Refresh(False)
        self._was_moving = moving

    def _backdrop(self):
        """The colour behind the button: Windows paints a notebook page in its
        theme's colour, which is not the page panel's own background colour."""
        w = self.GetParent()
        while w is not None:
            if isinstance(w, wx.Notebook):
                c = w.GetThemeBackgroundColour()
                if c.IsOk():
                    return c
                break
            w = w.GetParent()
        return self.GetParent().GetBackgroundColour()

    def _on_paint(self, event):
        if self._transparent:
            dc = wx.PaintDC(self)
        else:
            dc = wx.AutoBufferedPaintDC(self)
            dc.SetBackground(wx.Brush(self._backdrop()))
            dc.Clear()
        gc = wx.GraphicsContext.Create(dc)
        if gc:
            self._render(gc, *self.GetClientSize())

    def _render(self, gc, w, h):
        lip = self.FromDIP(4)
        face_h = h - lip
        top = lip if self._pressed else 0   # pressed: the face sinks onto its lip
        r = face_h / 2.0

        gc.SetPen(wx.Pen(wx.Colour(0, 0, 0), 1, wx.PENSTYLE_TRANSPARENT))
        gc.SetBrush(wx.Brush(wx.Colour(*self.LIP)))
        gc.DrawRoundedRectangle(0, lip, w, face_h, r)
        hi, lo = self.FACE_HOVER if self._hover else self.FACE
        gc.SetBrush(gc.CreateLinearGradientBrush(
            0, top, 0, top + face_h, wx.Colour(*hi), wx.Colour(*lo)))
        gc.DrawRoundedRectangle(0, top, w, face_h, r)
        self._draw_shine(gc, w, top, face_h)

        rim = 150 if self.HasFocus() else 64
        gc.SetBrush(wx.Brush(wx.Colour(0, 0, 0), wx.BRUSHSTYLE_TRANSPARENT))
        gc.SetPen(gc.CreatePen(wx.GraphicsPenInfo(wx.Colour(255, 255, 255, rim), 1.5)))
        gc.DrawRoundedRectangle(1, top + 1, w - 2, face_h - 2, r - 1)
        self._draw_label(gc, w, top + face_h / 2.0)

    def _draw_shine(self, gc, w, top, face_h):
        t = self._phase() / self.SWEEP
        if t >= 1:
            return
        t = t * t * (3 - 2 * t)             # ease in and out
        k = math.tan(math.radians(self.LEAN_DEG))
        nx, ny = 1 / math.hypot(1, k), k / math.hypot(1, k)   # band normal
        half = 0.14 * w
        pill = _pill_polygon(w, top, face_h)
        proj = [nx * x + ny * y for x, y in pill]
        lo, hi = min(proj) - half, max(proj) + half
        c = lo + t * (hi - lo)
        # The band is drawn as its intersection with the pill: wx cannot clip
        # to a path, and a gradient brush past its end points repeats on some
        # backends (GDI+) rather than holding its end colour.
        band = _clip_halfplane(pill, nx, ny, c - half, True)
        band = _clip_halfplane(band, nx, ny, c + half, False)
        if len(band) < 3:
            return
        clear = wx.Colour(255, 255, 255, 0)
        stops = wx.GraphicsGradientStops(clear, clear)
        stops.Add(wx.Colour(255, 255, 255, 128), 0.5)
        a, b = c - half, c + half
        gc.SetBrush(gc.CreateLinearGradientBrush(nx * a, ny * a, nx * b, ny * b, stops))
        path = gc.CreatePath()
        path.MoveToPoint(*band[0])
        for p in band[1:]:
            path.AddLineToPoint(*p)
        path.CloseSubpath()
        gc.FillPath(path)

    def _draw_label(self, gc, w, cy):
        gc.SetFont(self.GetFont(), wx.Colour(255, 255, 255))
        tw, th, descent, _ = gc.GetFullTextExtent(self._label)
        ascent = th - descent
        size = ascent                       # heart width, as in the SVG
        gap = 0.45 * size
        x = (w - size - gap - tw) / 2.0
        s = size / 20.0                     # the heart spans x 2..22, y 3..21.35
        path = gc.CreatePath()
        pt = lambda px, py: (x + (px - 2) * s, cy + (py - 12.175) * s)
        for op, *xy in _HEART:
            pts = [pt(xy[i], xy[i + 1]) for i in range(0, len(xy), 2)]
            if op == 'M':
                path.MoveToPoint(*pts[0])
            elif op == 'L':
                path.AddLineToPoint(*pts[0])
            else:
                path.AddCurveToPoint(*pts[0], *pts[1], *pts[2])
        path.CloseSubpath()
        gc.SetPen(wx.Pen(wx.Colour(0, 0, 0), 1, wx.PENSTYLE_TRANSPARENT))
        gc.SetBrush(wx.Brush(wx.Colour(255, 255, 255)))
        gc.FillPath(path)
        # Centre the capitals, not the text box: "Donate" has no descender.
        gc.DrawText(self._label, x + size + gap, cy - 0.62 * ascent)


def donate_button(parent, size, point_boost):
    """The Donate button: the shimmering pill, its tooltip, and a click that
    opens DONATE_URL. SIZE is in DIPs; POINT_BOOST enlarges the bold label.
    The About tab shows it large; the Autorouter tab's column a smaller one."""
    btn = ShimmerButton(parent, label="Donate", size=size)
    font = btn.GetFont()
    font.SetPointSize(font.GetPointSize() + point_boost)
    font.SetWeight(wx.FONTWEIGHT_BOLD)
    btn.SetFont(font)
    btn.SetToolTip(DONATE_TOOLTIP)
    btn.Bind(wx.EVT_BUTTON, lambda e: wx.LaunchDefaultBrowser(DONATE_URL))
    return btn


class AboutTab(wx.Panel):
    """About tab panel with version info, author, and links."""

    def __init__(self, parent, on_reset_settings=None, on_transparency_changed=None,
                 initial_transparency=240, on_validate_pcb_data=None):
        super().__init__(parent)
        self.on_reset_settings = on_reset_settings
        self.on_transparency_changed = on_transparency_changed
        self.on_validate_pcb_data = on_validate_pcb_data
        self._initial_transparency = initial_transparency
        self._needs_layout_refresh = True
        self._create_ui()
        # Bind to paint event to refresh layout on first draw (fixes Linux rendering)
        self.Bind(wx.EVT_PAINT, self._on_first_paint)

    def _on_first_paint(self, event):
        """Refresh layout on first paint to fix Linux rendering."""
        event.Skip()  # Allow normal painting
        if self._needs_layout_refresh:
            self._needs_layout_refresh = False
            # Unbind to avoid overhead on subsequent paints
            self.Unbind(wx.EVT_PAINT)
            # Schedule layout after this paint completes
            wx.CallAfter(self._refresh_layout)

    def _refresh_layout(self):
        """Refresh the layout to fix initial rendering on Linux."""
        self.InvalidateBestSize()
        self.GetParent().Layout()
        self.Layout()
        self.Refresh()

    def _create_ui(self):
        """Create the About tab UI."""
        about_sizer = wx.BoxSizer(wx.VERTICAL)

        # Header: the icon on the left, name / subtitle / versions on its
        # right. Stacked vertically they overflowed the 800x800 dialog.
        about_sizer.AddSpacer(10)
        header = wx.BoxSizer(wx.HORIZONTAL)

        icon_path = os.path.join(PLUGIN_DIR, "icon_512_text.png")
        if os.path.exists(icon_path):
            try:
                img = wx.Image(icon_path, wx.BITMAP_TYPE_PNG)
                # Scale down for display (use NORMAL quality - HIGH is very slow)
                img = img.Scale(256, 256, wx.IMAGE_QUALITY_NORMAL)
                bitmap = wx.StaticBitmap(self, bitmap=wx.Bitmap(img))
                header.Add(bitmap, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 20)
            except Exception:
                pass  # Skip icon if loading fails

        text_sizer = wx.BoxSizer(wx.VERTICAL)

        # Project name
        title = wx.StaticText(self, label="KiCadRoutingTools")
        title_font = title.GetFont()
        title_font.SetPointSize(18)
        title_font.SetWeight(wx.FONTWEIGHT_BOLD)
        title.SetFont(title_font)
        text_sizer.Add(title, 0, wx.BOTTOM, 4)

        # Subtitle
        subtitle = wx.StaticText(self, label="Rust-accelerated A* autorouter for KiCad")
        text_sizer.Add(subtitle, 0, wx.BOTTOM, 15)

        # Version info
        plugin_version, router_version = self._get_versions()

        info_panel = wx.Panel(self)
        info_sizer = wx.FlexGridSizer(cols=2, hgap=15, vgap=8)

        def add_info_row(label, value):
            lbl = wx.StaticText(info_panel, label=label)
            lbl.SetFont(lbl.GetFont().Bold())
            info_sizer.Add(lbl, 0, wx.ALIGN_RIGHT)
            val = wx.StaticText(info_panel, label=value)
            info_sizer.Add(val, 0, wx.ALIGN_LEFT)

        add_info_row("Plugin:", plugin_version)
        add_info_row("Router:", router_version)
        add_info_row("Author:", "DrAndyHaas")

        info_panel.SetSizer(info_sizer)
        text_sizer.Add(info_panel, 0)

        header.Add(text_sizer, 0, wx.ALIGN_CENTER_VERTICAL)
        about_sizer.Add(header, 0, wx.ALIGN_CENTER | wx.ALL, 10)

        # GitHub link
        github_url = "https://github.com/drandyhaas/KiCadRoutingTools"
        github_link = wx.adv.HyperlinkCtrl(
            self,
            label="GitHub Repository",
            url=github_url
        )
        about_sizer.Add(github_link, 0, wx.ALIGN_CENTER | wx.ALL, 10)

        # Donate button: the README's shimmering button, drawn in wx.
        donate_btn = donate_button(self, size=(260, 60), point_boost=8)
        about_sizer.Add(donate_btn, 0, wx.ALIGN_CENTER | wx.TOP, 10)

        donate_hint = wx.StaticText(
            self,
            label=("Free and MIT-licensed. Donations cover the ~$500/month of\n"
                   "cloud compute and AI development behind the tool."),
            style=wx.ALIGN_CENTRE_HORIZONTAL
        )
        donate_hint.SetForegroundColour(wx.Colour(128, 128, 128))
        about_sizer.Add(donate_hint, 0, wx.ALIGN_CENTER | wx.TOP, 8)

        # Secondary routes: GitHub Sponsors for anyone who already sponsors
        # projects there, PayPal for anyone who would rather use it directly.
        sponsors_link = wx.adv.HyperlinkCtrl(
            self, label="or sponsor on GitHub", url=SPONSORS_URL
        )
        about_sizer.Add(sponsors_link, 0, wx.ALIGN_CENTER | wx.TOP, 4)
        paypal_link = wx.adv.HyperlinkCtrl(
            self, label="or send directly via PayPal", url=PAYPAL_URL
        )
        about_sizer.Add(paypal_link, 0, wx.ALIGN_CENTER | wx.TOP, 2)

        # License/copyright
        copyright_text = wx.StaticText(
            self,
            label="Open source - see repository for license details"
        )
        copyright_text.SetForegroundColour(wx.Colour(128, 128, 128))
        about_sizer.Add(copyright_text, 0, wx.ALIGN_CENTER | wx.TOP, 20)

        # Transparency slider
        transparency_sizer = wx.BoxSizer(wx.HORIZONTAL)
        transparency_label = wx.StaticText(self, label="Window Transparency:")
        transparency_sizer.Add(transparency_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 20)
        self.transparency_slider = wx.Slider(
            self, value=self._initial_transparency, minValue=155, maxValue=255,
            style=wx.SL_HORIZONTAL
        )
        self.transparency_slider.SetMinSize((-1, 40))  # Ensure enough height for thumb
        self.transparency_slider.SetToolTip("Adjust window transparency")
        self.transparency_slider.Bind(wx.EVT_SLIDER, self._on_transparency_slider)
        transparency_sizer.Add(self.transparency_slider, 1, wx.EXPAND)
        about_sizer.Add(transparency_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.TOP, 30)

        # Validate PCB Data button
        validate_btn = wx.Button(self, label="Validate PCB Data")
        validate_btn.SetToolTip(
            "Compare pcbnew-extracted data against file parse to verify correctness.\n"
            "Results are shown in the Log tab."
        )
        validate_btn.Bind(wx.EVT_BUTTON, self._on_validate_pcb_data)
        about_sizer.Add(validate_btn, 0, wx.ALIGN_CENTER | wx.TOP, 20)

        # Reset settings button
        about_sizer.AddStretchSpacer()
        reset_btn = wx.Button(self, label="Reset All Settings to Defaults")
        reset_btn.SetToolTip("Clear log, selections, and reset all parameters to defaults")
        reset_btn.Bind(wx.EVT_BUTTON, self._on_reset_settings)
        about_sizer.Add(reset_btn, 0, wx.ALIGN_CENTER | wx.ALL, 20)

        self.SetSizer(about_sizer)

    def _on_validate_pcb_data(self, event):
        """Handle validate PCB data button click."""
        if self.on_validate_pcb_data:
            self.on_validate_pcb_data()

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

    def _on_transparency_slider(self, event):
        """Handle transparency slider change."""
        value = self.transparency_slider.GetValue()
        if self.on_transparency_changed:
            self.on_transparency_changed(value)

    def _get_versions(self):
        """Get plugin version from VERSION file and router version from rust module."""
        # Plugin version from VERSION file
        plugin_version = "Unknown"
        version_file = os.path.join(ROOT_DIR, "VERSION")
        try:
            with open(version_file, 'r') as f:
                plugin_version = f.read().strip()
        except Exception:
            pass

        # Router version from rust module
        router_version = "Unknown"
        try:
            import sys
            sys.path.insert(0, os.path.join(ROOT_DIR, 'rust_router'))
            if ROOT_DIR not in sys.path:
                sys.path.insert(0, ROOT_DIR)
            import rust_alloc  # noqa: F401  # issue #419: MIMALLOC_PURGE_DELAY before grid_router
            import grid_router
            router_version = grid_router.__version__
        except Exception:
            pass

        return plugin_version, router_version
