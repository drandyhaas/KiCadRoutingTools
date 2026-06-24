"""
KiCad Routing Tools - Planes GUI Components

Provides wx-based panels for power/ground plane creation and repair.
"""

import os
import sys
import wx

# Add parent directory to path
PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import routing_defaults as defaults
from .fanout_gui import NetSelectionPanel
from .gui_utils import StdoutRedirector


def parse_plane_mappings_result(value, copper_layers, net_names):
    """Parse a /recommend-plane-mappings RESULT line (issue #53).

    Format: assignment groups separated by ';', nets within a group joined
    by '|', one copper layer per group after the final ':', e.g.
    'GND:In1.Cu;VCC|+3V3:In2.Cu'.

    Returns (assignments, notes): assignments as the panel's
    (nets_list, layers_list) tuples (empty if nothing valid), notes listing
    what was rejected. Unknown nets and non-copper layers are dropped.
    """
    assignments, notes = [], []
    copper = set(copper_layers)
    known = set(net_names)
    for group in str(value).split(";"):
        group = group.strip()
        if not group:
            continue
        nets_part, sep, layer = group.rpartition(":")
        layer = layer.strip()
        if not sep or not nets_part:
            notes.append(f"plane mappings: unparseable group {group!r}")
            continue
        if layer not in copper:
            notes.append(f"plane mappings: {layer!r} is not a copper layer, group dropped")
            continue
        nets = [n.strip() for n in nets_part.split("|") if n.strip()]
        unknown = [n for n in nets if n not in known]
        nets = [n for n in nets if n in known]
        if unknown:
            notes.append(f"plane mappings: unknown nets {unknown} dropped")
        if nets:
            assignments.append((nets, [layer]))
        else:
            notes.append(f"plane mappings: group {group!r} had no known nets")
    return assignments, notes


class PlaneAssignmentPanel(wx.Panel):
    """Panel for managing net-to-layer assignments for plane creation.

    Each assignment maps a group of nets (joined with "|") to one or more target layers.
    """

    def __init__(self, parent, pcb_data, get_selected_nets_callback, on_ask_claude=None):
        """
        Create a plane assignment panel.

        Args:
            parent: Parent window
            pcb_data: PCBData object with board info
            get_selected_nets_callback: Function that returns currently selected nets
            on_ask_claude: Callback for the "Ask Claude" button that recommends
                net -> layer mappings (issue #53); button hidden if None.
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.get_selected_nets = get_selected_nets_callback
        self.on_ask_claude = on_ask_claude
        self.assignments = []  # List of (nets_list, layers_list) tuples

        self._create_ui()

    def _create_ui(self):
        """Create the panel UI."""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Assignment list - capped at ~3 visible rows; scrolls internally when overflowing.
        self.assignment_list = wx.ListBox(self, style=wx.LB_EXTENDED)
        self.assignment_list.SetToolTip("Net → Layer assignments. Select and click Remove to delete.")
        row_h = self.assignment_list.GetCharHeight() + 2
        list_h = row_h * 3 + 8  # 3 rows + a little frame padding
        self.assignment_list.SetMinSize((-1, list_h))
        self.assignment_list.SetMaxSize((-1, list_h))
        sizer.Add(self.assignment_list, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Layer selection with checkboxes
        layer_label = wx.StaticText(self, label="Target Layers:")
        sizer.Add(layer_label, 0, wx.BOTTOM, 2)

        # Create checkbox wrap sizer for layers (wraps to next line if needed)
        copper_layers = self._get_copper_layers()
        layer_sizer = wx.WrapSizer(wx.HORIZONTAL)
        self.layer_checks = {}
        for layer in copper_layers:
            cb = wx.CheckBox(self, label=layer)
            cb.SetToolTip(f"Include {layer} in this assignment")
            self.layer_checks[layer] = cb
            layer_sizer.Add(cb, 0, wx.RIGHT | wx.BOTTOM, 10)

        sizer.Add(layer_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Buttons row
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.add_btn = wx.Button(self, label="Add Assignment")
        self.add_btn.SetToolTip("Add selected nets to checked layers")
        self.add_btn.Bind(wx.EVT_BUTTON, self._on_add)
        btn_sizer.Add(self.add_btn, 1, wx.RIGHT, 5)

        self.remove_btn = wx.Button(self, label="Remove")
        self.remove_btn.SetToolTip("Remove selected assignments")
        self.remove_btn.Bind(wx.EVT_BUTTON, self._on_remove)
        btn_sizer.Add(self.remove_btn, 0)

        if self.on_ask_claude is not None:
            self.ask_claude_btn = wx.Button(self, label="Ask Claude", style=wx.BU_EXACTFIT)
            self.ask_claude_btn.SetToolTip(
                "Run the /recommend-plane-mappings skill: recommends which nets "
                "deserve planes and on which layers (GND adjacency, GND/VCC pairing), "
                "then fills this assignment list. Takes a few minutes.")
            self.ask_claude_btn.Bind(wx.EVT_BUTTON, lambda event: self.on_ask_claude())
            btn_sizer.Add(self.ask_claude_btn, 0, wx.LEFT, 5)

        sizer.Add(btn_sizer, 0, wx.EXPAND)

        self.SetSizer(sizer)

    def _get_copper_layers(self):
        """Get list of copper layer names from PCB data."""
        # Try to get from board_info if available
        if hasattr(self.pcb_data, 'board_info') and self.pcb_data.board_info:
            if hasattr(self.pcb_data.board_info, 'copper_layers'):
                return self.pcb_data.board_info.copper_layers

        # Fallback to common layer names
        return ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'B.Cu']

    def _get_selected_layers(self):
        """Get list of currently checked layer names."""
        return [layer for layer, cb in self.layer_checks.items() if cb.GetValue()]

    def _on_add(self, event):
        """Add selected nets as a new assignment."""
        selected_nets = list(self.get_selected_nets())
        if not selected_nets:
            wx.MessageBox(
                "Please select nets from the left panel first.",
                "No Nets Selected",
                wx.OK | wx.ICON_WARNING
            )
            return

        selected_layers = self._get_selected_layers()
        if not selected_layers:
            wx.MessageBox(
                "Please check at least one target layer.",
                "No Layers Selected",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Add assignment
        self.assignments.append((selected_nets, selected_layers))
        self._refresh_list()

        # Clear layer checkboxes for next assignment
        for cb in self.layer_checks.values():
            cb.SetValue(False)

    def _on_remove(self, event):
        """Remove selected assignments."""
        selections = list(self.assignment_list.GetSelections())
        if not selections:
            return

        # Remove in reverse order to maintain indices
        for idx in sorted(selections, reverse=True):
            if idx < len(self.assignments):
                del self.assignments[idx]

        self._refresh_list()

    def _refresh_list(self):
        """Refresh the assignment list display."""
        self.assignment_list.Clear()
        for nets, layers in self.assignments:
            # Truncate display if too long
            nets_str = ", ".join(nets)
            if len(nets_str) > 25:
                nets_str = nets_str[:22] + "..."
            layers_str = ", ".join(layers)
            if len(layers_str) > 20:
                layers_str = layers_str[:17] + "..."
            self.assignment_list.Append(f"{nets_str} → {layers_str}")

    def get_assignments(self):
        """Get all assignments as list of (nets_list, layer) tuples."""
        return list(self.assignments)

    def set_assignments(self, assignments):
        """Set assignments from a list of (nets_list, layer) tuples."""
        self.assignments = list(assignments)
        self._refresh_list()

    def clear_assignments(self):
        """Clear all assignments."""
        self.assignments = []
        self._refresh_list()


class CreatePlanesOptionsPanel(wx.Panel):
    """Options panel for creating copper planes (route_planes.py)."""

    def __init__(self, parent, on_ask_claude=None):
        """Create the options panel.

        Args:
            on_ask_claude: Callback for the "Ask Claude" button next to the
                GND via distance field (issue #39); button hidden if None.
        """
        super().__init__(parent)
        self.on_ask_claude = on_ask_claude
        self._create_ui()

    def _create_ui(self):
        """Create the panel UI."""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Zone parameters
        zone_box = wx.StaticBox(self, label="Zone Parameters")
        zone_sizer = wx.StaticBoxSizer(zone_box, wx.VERTICAL)

        grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        grid.AddGrowableCol(1)

        # Zone clearance
        grid.Add(wx.StaticText(self, label="Zone Clearance (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['plane_zone_clearance']
        self.zone_clearance = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                 initial=defaults.PLANE_ZONE_CLEARANCE, inc=r['inc'])
        self.zone_clearance.SetDigits(r['digits'])
        self.zone_clearance.SetToolTip("Clearance from zone fill to other copper")
        grid.Add(self.zone_clearance, 0, wx.EXPAND)

        # Max search radius
        grid.Add(wx.StaticText(self, label="Max Search Radius (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['plane_max_search_radius']
        self.max_search_radius = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                    initial=defaults.PLANE_MAX_SEARCH_RADIUS, inc=r['inc'])
        self.max_search_radius.SetDigits(r['digits'])
        self.max_search_radius.SetToolTip("Maximum radius to search for valid via placement")
        grid.Add(self.max_search_radius, 0, wx.EXPAND)

        # Same-net pad clearance (default = main clearance; checkbox below overrides to via-in-pad)
        grid.Add(wx.StaticText(self, label="Same-net Pad Clearance (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['same_net_pad_clearance']
        self.same_net_pad_clearance = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                        initial=defaults.CLEARANCE, inc=r['inc'])
        self.same_net_pad_clearance.SetDigits(r['digits'])
        self.same_net_pad_clearance.SetToolTip(
            "Edge-to-edge clearance between stitching vias and same-net pads. "
            "Disabled if 'Allow via-in-pad' is checked.")
        grid.Add(self.same_net_pad_clearance, 0, wx.EXPAND)

        zone_sizer.Add(grid, 0, wx.EXPAND | wx.ALL, 5)

        # Via-in-pad override: when checked, vias may be placed inside same-net pads
        # (and Same-net Pad Clearance is disabled / passed as -1).
        self.via_in_pad_check = wx.CheckBox(self, label="Allow via-in-pad (override clearance)")
        self.via_in_pad_check.SetToolTip(
            "When checked, stitching vias may be placed on top of same-net pads, "
            "ignoring 'Same-net Pad Clearance'.")
        self.via_in_pad_check.SetValue(False)
        self.via_in_pad_check.Bind(wx.EVT_CHECKBOX, self._on_via_in_pad_toggle)
        zone_sizer.Add(self.via_in_pad_check, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        sizer.Add(zone_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Rip-up options
        ripup_box = wx.StaticBox(self, label="Blocker Handling")
        ripup_sizer = wx.StaticBoxSizer(ripup_box, wx.VERTICAL)

        self.rip_blocker_check = wx.CheckBox(self, label="Rip up blocking nets")
        self.rip_blocker_check.SetToolTip(
            "Remove nets that block via placement (uses Max Rip-up from Basic tab). "
            "Ripped nets are left unrouted - run the routing tab afterward to reconnect "
            "them (it handles rip-up/restore safely).")
        ripup_sizer.Add(self.rip_blocker_check, 0, wx.ALL, 5)

        sizer.Add(ripup_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # GND Return Vias section
        gnd_box = wx.StaticBox(self, label="GND Return Vias")
        gnd_sizer = wx.StaticBoxSizer(gnd_box, wx.VERTICAL)

        self.add_gnd_vias_check = wx.CheckBox(self, label="Add GND vias near signal vias")
        self.add_gnd_vias_check.SetToolTip("Add GND vias near signal vias for return current path")
        gnd_sizer.Add(self.add_gnd_vias_check, 0, wx.ALL, 5)

        # GND via parameters
        gnd_grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        gnd_grid.AddGrowableCol(1)

        gnd_grid.Add(wx.StaticText(self, label="Max Distance (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['gnd_via_distance']
        self.gnd_via_distance = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                   initial=defaults.GND_VIA_DISTANCE, inc=r['inc'])
        self.gnd_via_distance.SetDigits(r['digits'])
        self.gnd_via_distance.SetToolTip("Maximum distance from signal via to place GND via")
        if self.on_ask_claude is not None:
            dist_sizer = wx.BoxSizer(wx.HORIZONTAL)
            dist_sizer.Add(self.gnd_via_distance, 1, wx.EXPAND | wx.RIGHT, 5)
            self.ask_claude_btn = wx.Button(self, label="Ask Claude", style=wx.BU_EXACTFIT)
            self.ask_claude_btn.SetToolTip(
                "Run the /find-high-speed-nets skill: looks up component datasheets to "
                "classify nets by speed and recommends this distance for GND return "
                "vias. Takes a few minutes (web lookups).")
            self.ask_claude_btn.Bind(wx.EVT_BUTTON, lambda event: self.on_ask_claude())
            dist_sizer.Add(self.ask_claude_btn, 0)
            gnd_grid.Add(dist_sizer, 0, wx.EXPAND)
        else:
            gnd_grid.Add(self.gnd_via_distance, 0, wx.EXPAND)

        gnd_grid.Add(wx.StaticText(self, label="GND Net Name:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.gnd_via_net = wx.TextCtrl(self, value=defaults.GND_VIA_NET)
        self.gnd_via_net.SetToolTip("Net name for GND vias (e.g., GND)")
        gnd_grid.Add(self.gnd_via_net, 0, wx.EXPAND)

        gnd_sizer.Add(gnd_grid, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        sizer.Add(gnd_sizer, 0, wx.EXPAND)

        self.SetSizer(sizer)

    def _on_via_in_pad_toggle(self, event):
        """Enable/disable the same-net pad clearance spin ctrl based on the via-in-pad checkbox."""
        self.same_net_pad_clearance.Enable(not self.via_in_pad_check.GetValue())

    def get_config(self):
        """Get the configuration values."""
        if self.via_in_pad_check.GetValue():
            same_net_clr = -1.0  # via-in-pad allowed
        else:
            same_net_clr = self.same_net_pad_clearance.GetValue()
        return {
            'zone_clearance': self.zone_clearance.GetValue(),
            'max_search_radius': self.max_search_radius.GetValue(),
            'rip_blocker_nets': self.rip_blocker_check.GetValue(),            'add_gnd_vias': self.add_gnd_vias_check.GetValue(),
            'gnd_via_distance': self.gnd_via_distance.GetValue(),
            'gnd_via_net': self.gnd_via_net.GetValue(),
            'same_net_pad_clearance': same_net_clr,
        }


class RepairPlanesOptionsPanel(wx.Panel):
    """Options panel for repairing disconnected planes (route_disconnected_planes.py)."""

    def __init__(self, parent, get_track_width=None):
        """Create the options panel.

        Args:
            parent: Parent window
            get_track_width: Callback to get track width from Basic tab for validation
        """
        super().__init__(parent)
        self._get_track_width = get_track_width
        self._create_ui()

    def _create_ui(self):
        """Create the panel UI."""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Track parameters
        track_box = wx.StaticBox(self, label="Track Parameters")
        track_sizer = wx.StaticBoxSizer(track_box, wx.VERTICAL)

        grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        grid.AddGrowableCol(1)

        # Max track width
        grid.Add(wx.StaticText(self, label="Max Track Width (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['repair_max_track_width']
        self.max_track_width = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                  initial=defaults.REPAIR_MAX_TRACK_WIDTH, inc=r['inc'])
        self.max_track_width.SetDigits(r['digits'])
        self.max_track_width.SetToolTip("Maximum track width for region connections (must be >= Track Width)")
        self.max_track_width.Bind(wx.EVT_SPINCTRLDOUBLE, self._on_max_track_width_changed)
        grid.Add(self.max_track_width, 0, wx.EXPAND)

        # Analysis grid step
        grid.Add(wx.StaticText(self, label="Analysis Grid (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['repair_analysis_grid_step']
        self.analysis_grid = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                initial=defaults.REPAIR_ANALYSIS_GRID_STEP, inc=r['inc'])
        self.analysis_grid.SetDigits(r['digits'])
        self.analysis_grid.SetToolTip("Grid step for connectivity analysis (coarser = faster)")
        grid.Add(self.analysis_grid, 0, wx.EXPAND)

        track_sizer.Add(grid, 0, wx.EXPAND | wx.ALL, 5)
        sizer.Add(track_sizer, 0, wx.EXPAND)

        # Repair pad connections (on by default; matches the CLI --repair-pads).
        self.repair_pads = wx.CheckBox(self, label="Repair pad connections")
        self.repair_pads.SetValue(True)
        self.repair_pads.SetToolTip(
            "Also tap pads that aren't connected to their plane (pad-level repair). "
            "Uncheck to only connect disconnected zone regions (CLI --no-repair-pads).")
        sizer.Add(self.repair_pads, 0, wx.LEFT | wx.TOP, 5)

        # Rip blocking nets to connect a pad that can't reach its plane (e.g. a
        # tiny connector GND pin) by tracing to an adjacent same-net pad. Mirrors
        # the Create tab (CLI --rip-blocker-nets). Ripped nets are left unrouted;
        # the routing tab reconnects them afterward.
        self.rip_blocker_check = wx.CheckBox(self, label="Rip up blocking nets")
        self.rip_blocker_check.SetToolTip(
            "When a pad can't connect to its plane, trace it to a nearby same-net pad, "
            "ripping the signal net(s) blocking it (uses Max Rip-up from the Basic tab). "
            "Ripped nets are left unrouted - run the routing tab afterward to reconnect them.")
        sizer.Add(self.rip_blocker_check, 0, wx.LEFT | wx.TOP, 5)

        # Info text
        info = wx.StaticText(self, label="Leave nets/layers empty to auto-detect existing zones.")
        info.SetForegroundColour(wx.Colour(100, 100, 100))
        sizer.Add(info, 0, wx.ALL, 5)

        self.SetSizer(sizer)

    def get_config(self):
        """Get the configuration values."""
        return {
            'max_track_width': self.max_track_width.GetValue(),
            'analysis_grid_step': self.analysis_grid.GetValue(),
            'repair_pads': self.repair_pads.GetValue(),
            'rip_blocker_nets': self.rip_blocker_check.GetValue(),        }

    def _on_max_track_width_changed(self, event):
        """Validate max track width >= track width from Basic tab."""
        # Guard against re-entrancy: see _on_drc_param_changed in routing_dialog.py (#30).
        if getattr(self, '_validating', False):
            return

        if not self._get_track_width:
            event.Skip()
            return

        track_width = self._get_track_width()
        max_width = self.max_track_width.GetValue()

        if max_width < track_width:
            self._validating = True
            try:
                self.max_track_width.SetValue(track_width)
            finally:
                self._validating = False
            wx.CallAfter(
                wx.MessageBox,
                f"Max Track Width cannot be less than Track Width ({track_width:.3f} mm)",
                "Invalid Value",
                wx.OK | wx.ICON_WARNING
            )
        else:
            event.Skip()


class PlanesTab(wx.Panel):
    """Tab for copper plane creation and repair."""

    def __init__(self, parent, pcb_data, board_filename,
                 get_shared_params=None, on_planes_complete=None,
                 get_connectivity_check=None, append_log=None,
                 sync_pcb_data_callback=None, get_claude_params=None):
        """
        Create the planes tab.

        Args:
            parent: Parent notebook
            pcb_data: PCBData object
            board_filename: Path to the PCB file
            get_shared_params: Callback to get shared parameters from Basic tab
            on_planes_complete: Callback when planes operation finishes
            get_connectivity_check: Callback that returns connectivity check function
            append_log: Callback to append text to log
            sync_pcb_data_callback: Callback to sync pcb_data from board
            get_claude_params: Callback returning the Claude tab's
                {'model', 'effort'} selections for headless runs
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self.get_shared_params = get_shared_params
        self.get_claude_params = get_claude_params
        self.on_planes_complete = on_planes_complete
        self.get_connectivity_check = get_connectivity_check
        self.append_log = append_log
        self.sync_pcb_data_callback = sync_pcb_data_callback
        self._routing_thread = None
        self._cancel_requested = False

        self._create_ui()

        # Set up connectivity check
        if self.get_connectivity_check:
            self.net_panel.set_check_function(self.get_connectivity_check())

    def _create_ui(self):
        """Create the tab UI."""
        # GridSizer for an even 50/50 split — see routing_dialog._create_config_tab.
        main_sizer = wx.GridSizer(rows=1, cols=2, hgap=0, vgap=0)

        # Left side: Net selection
        net_box = wx.StaticBox(self, label="Net Selection")
        net_box_sizer = wx.StaticBoxSizer(net_box, wx.VERTICAL)

        self.net_panel = NetSelectionPanel(
            self, self.pcb_data,
            instructions="Select nets for plane creation (e.g., GND, VCC)...",
            show_hide_checkbox=True,
            show_component_dropdown=True,
            show_hide_differential=False,
        )
        net_box_sizer.Add(self.net_panel, 1, wx.EXPAND)

        main_sizer.Add(net_box_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Right side: Mode, layers, options, buttons
        right_sizer = wx.BoxSizer(wx.VERTICAL)

        # Mode selection
        mode_box = wx.StaticBox(self, label="Mode")
        mode_sizer = wx.StaticBoxSizer(mode_box, wx.VERTICAL)

        self.mode_selector = wx.RadioBox(
            self, choices=["Create Planes", "Repair Disconnected"],
            style=wx.RA_HORIZONTAL
        )
        self.mode_selector.SetToolTip("Create: Add zones with via stitching\nRepair: Connect disconnected regions")
        self.mode_selector.Bind(wx.EVT_RADIOBOX, self._on_mode_changed)
        mode_sizer.Add(self.mode_selector, 0, wx.EXPAND | wx.ALL, 5)

        right_sizer.Add(mode_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Plane assignments (shown in Create mode)
        self.assign_box = wx.StaticBox(self, label="Net → Layer Assignments")
        self.assign_sizer = wx.StaticBoxSizer(self.assign_box, wx.VERTICAL)

        self.assignment_panel = PlaneAssignmentPanel(
            self, self.pcb_data,
            get_selected_nets_callback=lambda: self.net_panel.get_selected_nets(),
            on_ask_claude=self._on_ask_claude_plane_mappings
        )
        self.assign_sizer.Add(self.assignment_panel, 1, wx.EXPAND | wx.ALL, 5)

        right_sizer.Add(self.assign_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Scrollable container for the mode-specific options panels.
        # Spans the available height between Net→Layer Assignments and Status
        # so when the active options panel doesn't fit, a vertical scrollbar
        # appears here instead of squashing or pushing other sections offscreen.
        self.options_scroll = wx.ScrolledWindow(self, style=wx.VSCROLL | wx.TAB_TRAVERSAL)
        options_scroll_sizer = wx.BoxSizer(wx.VERTICAL)

        # Create options panel
        self.create_options = CreatePlanesOptionsPanel(
            self.options_scroll, on_ask_claude=self._on_ask_claude_gnd_via)
        options_scroll_sizer.Add(self.create_options, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Repair options panel (initially hidden)
        def get_track_width():
            if self.get_shared_params:
                return self.get_shared_params().get('track_width', defaults.TRACK_WIDTH)
            return defaults.TRACK_WIDTH
        self.repair_options = RepairPlanesOptionsPanel(
            self.options_scroll, get_track_width=get_track_width)
        options_scroll_sizer.Add(self.repair_options, 0, wx.EXPAND | wx.BOTTOM, 5)
        self.repair_options.Hide()

        self.options_scroll.SetSizer(options_scroll_sizer)
        self.options_scroll.SetScrollRate(0, 10)
        self.options_scroll.FitInside()
        right_sizer.Add(self.options_scroll, 1, wx.EXPAND | wx.BOTTOM, 5)

        # Status
        status_box = wx.StaticBox(self, label="Status")
        status_sizer = wx.StaticBoxSizer(status_box, wx.VERTICAL)

        self.status_text = wx.StaticText(self, label="Ready")
        status_sizer.Add(self.status_text, 0, wx.ALL, 5)

        self.progress_bar = wx.Gauge(self, range=100)
        status_sizer.Add(self.progress_bar, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        right_sizer.Add(status_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.action_btn = wx.Button(self, label="Create Planes")
        self.action_btn.SetToolTip("Execute plane creation or repair operation")
        self.action_btn.Bind(wx.EVT_BUTTON, self._on_action)
        btn_sizer.Add(self.action_btn, 1, wx.RIGHT, 5)

        self.cancel_btn = wx.Button(self, label="Close")
        self.cancel_btn.SetToolTip("Close dialog (or cancel operation if in progress)")
        self.cancel_btn.Bind(wx.EVT_BUTTON, self._on_cancel_or_close)
        btn_sizer.Add(self.cancel_btn, 1)

        right_sizer.Add(btn_sizer, 0, wx.EXPAND)

        main_sizer.Add(right_sizer, 0, wx.EXPAND | wx.ALL, 5)

        self.SetSizer(main_sizer)

    def _on_mode_changed(self, event):
        """Handle mode selection change."""
        mode = self.mode_selector.GetSelection()
        if mode == 0:  # Create Planes
            self.create_options.Show()
            self.repair_options.Hide()
            self.action_btn.SetLabel("Create Planes")
        else:  # Repair Disconnected
            self.create_options.Hide()
            self.repair_options.Show()
            self.action_btn.SetLabel("Repair")
        # Re-layout the scroll container so it recomputes the virtual size,
        # then the outer tab so the scrollbar appears/disappears as needed.
        self.options_scroll.Layout()
        self.options_scroll.FitInside()
        self.Layout()

    def _on_ask_claude_gnd_via(self):
        """Run /find-high-speed-nets headless and fill the GND via distance
        field from its recommendation (issue #39)."""
        from .claude_gui import find_claude, ClaudeSkillDialog, board_path_for_analysis

        claude_path = find_claude()
        if claude_path is None:
            wx.MessageBox(
                "Claude Code CLI not found. Install it (https://claude.com/claude-code) "
                "and make sure `claude` is on your PATH.",
                "Claude", wx.OK | wx.ICON_WARNING)
            return
        board = board_path_for_analysis(self.board_filename)
        if board is None:
            return

        prompt = (
            f"/find-high-speed-nets {os.path.abspath(board)} — analysis only, do not "
            "modify any files. After the report, end your reply with exactly one "
            "line of the form RESULT=<recommended --gnd-via-distance in mm> "
            "(a bare number), e.g. RESULT=2.5"
        )
        # Obey the model/effort selected on the Claude tab
        model = effort = None
        if self.get_claude_params:
            claude_params = self.get_claude_params()
            model = claude_params.get('model')
            effort = claude_params.get('effort')
        dlg = ClaudeSkillDialog(
            self, "Claude: recommend GND via distance", prompt,
            claude_path=claude_path, model=model, effort=effort,
            intro=f"Running /find-high-speed-nets on {os.path.basename(board)} ...\n"
                  "(datasheet lookups; typically a few minutes)")
        dlg.ShowModal()
        value = dlg.result_value
        dlg.Destroy()
        if value is not None:
            self._apply_gnd_via_recommendation(value)

    def _on_ask_claude_plane_mappings(self):
        """Run /recommend-plane-mappings headless and fill the assignment
        list from its recommendation (issue #53)."""
        from .claude_gui import find_claude, ClaudeSkillDialog, board_path_for_analysis

        claude_path = find_claude()
        if claude_path is None:
            wx.MessageBox(
                "Claude Code CLI not found. Install it (https://claude.com/claude-code) "
                "and make sure `claude` is on your PATH.",
                "Claude", wx.OK | wx.ICON_WARNING)
            return
        board = board_path_for_analysis(self.board_filename)
        if board is None:
            return

        prompt = (
            f"/recommend-plane-mappings {os.path.abspath(board)} — analysis only, "
            "do not modify any files. After the report, end your reply with exactly "
            "one line of the form RESULT=<net>:<layer>;<net>|<net>:<layer> "
            "(groups separated by ';', nets sharing a layer joined by '|', exact "
            "net names, one copper layer per group), e.g. RESULT=GND:In1.Cu;VCC:In2.Cu"
        )
        model = effort = None
        if self.get_claude_params:
            claude_params = self.get_claude_params()
            model = claude_params.get('model')
            effort = claude_params.get('effort')
        dlg = ClaudeSkillDialog(
            self, "Claude: recommend plane mappings", prompt,
            claude_path=claude_path, model=model, effort=effort,
            intro=f"Running /recommend-plane-mappings on {os.path.basename(board)} ...\n"
                  "(board analysis; typically a few minutes)")
        dlg.ShowModal()
        value = dlg.result_value
        dlg.Destroy()
        if value is not None:
            self._apply_plane_mappings_recommendation(value)

    def _apply_plane_mappings_recommendation(self, value):
        """Validate Claude's RESULT value and fill the assignment list."""
        net_names = {net.name for net in self.pcb_data.nets.values() if net.name}
        copper = self.assignment_panel._get_copper_layers()
        assignments, notes = parse_plane_mappings_result(value, copper, net_names)
        for note in notes:
            if self.append_log:
                self.append_log(f"Claude: {note}\n")
        if not assignments:
            if self.append_log:
                self.append_log(f"Claude: no usable plane mappings in {value!r}\n")
            return

        existing = self.assignment_panel.get_assignments()
        if existing:
            choice = wx.MessageBox(
                "The assignment list already has entries.\n\n"
                "Yes = replace them with Claude's recommendation\n"
                "No = merge (keep existing, add new ones)",
                "Plane mappings", wx.YES_NO | wx.CANCEL | wx.ICON_QUESTION)
            if choice == wx.CANCEL:
                return
            if choice == wx.NO:
                seen = {(tuple(nets), tuple(layers)) for nets, layers in existing}
                assignments = existing + [
                    (nets, layers) for nets, layers in assignments
                    if (tuple(nets), tuple(layers)) not in seen]
        self.assignment_panel.set_assignments(assignments)
        if self.append_log:
            shown = ", ".join(f"{'|'.join(nets)} -> {'/'.join(layers)}"
                              for nets, layers in assignments)
            self.append_log(f"Claude recommended plane mappings: {shown}\n")

    def _apply_gnd_via_recommendation(self, value):
        """Validate Claude's RESULT value and apply it to the GUI controls."""
        try:
            distance = float(value)
        except ValueError:
            if self.append_log:
                self.append_log(f"Claude: unusable GND via distance {value!r}\n")
            return
        r = defaults.PARAM_RANGES['gnd_via_distance']
        clamped = max(r['min'], min(r['max'], distance))
        self.create_options.gnd_via_distance.SetValue(clamped)
        self.create_options.add_gnd_vias_check.SetValue(True)
        if self.append_log:
            note = "" if clamped == distance else f" (clamped from {distance})"
            self.append_log(f"Claude recommended GND via distance: {clamped} mm{note}; "
                            "enabled 'Add GND vias near signal vias'\n")

    def _on_cancel_or_close(self, event):
        """Handle cancel/close button."""
        if self._routing_thread and self._routing_thread.is_alive():
            self._cancel_requested = True
            self.status_text.SetLabel("Cancelling...")
        else:
            self.GetTopLevelParent().EndModal(wx.ID_CANCEL)

    def _on_action(self, event):
        """Handle action button click."""
        mode = self.mode_selector.GetSelection()

        # Validation - both modes require assignments
        assignments = self.assignment_panel.get_assignments()
        if not assignments:
            mode_name = "plane creation" if mode == 0 else "repair"
            wx.MessageBox(
                f"Please add at least one net → layer assignment for {mode_name}.\n\n"
                "Select nets on the left, check target layers, then click 'Add Assignment'.",
                "No Assignments",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Get shared parameters
        shared_params = {}
        if self.get_shared_params:
            shared_params = self.get_shared_params()

        # Get mode-specific config
        if mode == 0:
            config = self.create_options.get_config()
        else:
            config = self.repair_options.get_config()

        # Both modes use assignments
        config['assignments'] = assignments

        config.update(shared_params)
        config['mode'] = 'create' if mode == 0 else 'repair'

        # Disable UI
        self.action_btn.Disable()
        self.cancel_btn.SetLabel("Cancel")
        self._cancel_requested = False

        action_name = "Creating planes" if mode == 0 else "Repairing planes"
        self.status_text.SetLabel(f"{action_name}...")
        self.progress_bar.Pulse()
        wx.Yield()

        # Run in thread
        import threading
        self._routing_thread = threading.Thread(
            target=self._run_planes_operation,
            args=(config,),
            daemon=True
        )
        self._routing_thread.start()

        # Poll for completion
        self._poll_operation()

    def _run_planes_operation(self, config):
        """Run plane operation in background thread."""
        import sys

        original_stdout = sys.stdout
        if self.append_log:
            sys.stdout = StdoutRedirector(self.append_log, original_stdout)

        try:
            if config['mode'] == 'create':
                self._run_create_planes(config)
            else:
                self._run_repair_planes(config)

        except Exception as e:
            import traceback
            traceback.print_exc()
            self._operation_result = {'error': str(e)}

        finally:
            sys.stdout = original_stdout

    def _run_create_planes(self, config):
        """Run plane creation."""
        # The pour doesn't delete ripped tracks via pcbnew; clear any stale set
        # from a prior repair run so the apply step doesn't remove copper.
        self._ripped_net_ids = []
        # Remember the routed floors so _apply_results_to_board can make the live
        # board's DRC constraints consistent with them (issue #160).
        self._plane_drc_config = dict(config)
        from route_planes import create_plane
        from add_gnd_vias import add_gnd_vias_to_existing_board
        from routing_config import GridRouteConfig, GridCoord
        from obstacle_map import build_base_obstacle_map

        # Get assignments: each is (nets_list, layers_list)
        assignments = config['assignments']

        # Get all copper layers for routing
        all_layers = self._get_all_copper_layers()

        total_vias = 0
        total_traces = 0
        total_pads = 0

        # Expand assignments: each net goes on each layer in the assignment
        # e.g., nets=['+3.3V'] layers=['F.Cu', 'In2.Cu'] becomes:
        #   expanded_nets=['+3.3V', '+3.3V'], expanded_layers=['F.Cu', 'In2.Cu']
        # Also build layer_nets dict for multi-net layer handling (Voronoi boundaries)
        expanded_nets = []
        expanded_layers = []
        layer_nets = {}  # layer -> list of nets on that layer
        for nets_list, layers_list in assignments:
            for layer in layers_list:
                if layer not in layer_nets:
                    layer_nets[layer] = []
                for net in nets_list:
                    expanded_nets.append(net)
                    expanded_layers.append(layer)
                    if net not in layer_nets[layer]:
                        layer_nets[layer].append(net)

        print(f"\nCreating planes for {len(expanded_nets)} net/layer pairs:")
        for net, layer in zip(expanded_nets, expanded_layers):
            print(f"  {net} -> {layer}")
        # Show multi-net layers
        for layer, nets in layer_nets.items():
            if len(nets) > 1:
                print(f"  Multi-net layer {layer}: {', '.join(nets)}")

        if not expanded_nets:
            print("No net/layer assignments to process")
            return

        failed_pads = 0
        try:
            (vias, traces, pads_needing, new_vias, new_segments, new_zones,
             failed_pads) = create_plane(
                input_file=self.board_filename,
                output_file="",
                net_names=expanded_nets,
                plane_layers=expanded_layers,
                via_size=config.get('via_size', defaults.VIA_SIZE),
                via_drill=config.get('via_drill', defaults.VIA_DRILL),
                track_width=config.get('track_width', defaults.TRACK_WIDTH),
                clearance=config.get('clearance', defaults.CLEARANCE),
                zone_clearance=config.get('zone_clearance', defaults.PLANE_ZONE_CLEARANCE),
                min_thickness=defaults.PLANE_MIN_THICKNESS,
                grid_step=config.get('grid_step', defaults.GRID_STEP),
                max_search_radius=config.get('max_search_radius', defaults.PLANE_MAX_SEARCH_RADIUS),
                max_via_reuse_radius=defaults.PLANE_MAX_VIA_REUSE_RADIUS,
                hole_to_hole_clearance=config.get('hole_to_hole_clearance', defaults.HOLE_TO_HOLE_CLEARANCE),
                board_edge_clearance=config.get('board_edge_clearance', defaults.BOARD_EDGE_CLEARANCE),
                all_layers=all_layers,
                dry_run=True,  # Don't write to file, apply via pcbnew
                rip_blocker_nets=config.get('rip_blocker_nets', False),
                max_rip_nets=config.get('max_ripup', defaults.MAX_RIPUP),                # Re-route a ripped wide power net at its proper width.
                power_nets=config.get('power_nets'),
                power_nets_widths=config.get('power_nets_widths'),
                # Match signal routing's No-BGA-Zones intent when rerouting
                # ripped nets on a BGA board (issue #88). The route tab's
                # "ALL" means disable every BGA exclusion zone.
                no_bga_zone=(config.get('no_bga_zones_text', '').upper() == 'ALL'),
                pcb_data=self.pcb_data,
                return_results=True,
                layer_nets=layer_nets,
                same_net_pad_clearance=config.get('same_net_pad_clearance', defaults.SAME_NET_PAD_CLEARANCE),
                skip_existing_zones=True,
            )

            total_vias = vias
            total_traces = traces
            total_pads = pads_needing
            self._new_vias = new_vias
            self._new_segments = new_segments
            self._new_zones = new_zones

            # Add GND return vias if enabled
            if config.get('add_gnd_vias', False):
                try:
                    gnd_via_distance = config.get('gnd_via_distance', defaults.GND_VIA_DISTANCE)
                    gnd_via_net = config.get('gnd_via_net', defaults.GND_VIA_NET)

                    # Create config for GND via placement
                    gnd_config = GridRouteConfig(
                        via_size=config.get('via_size', defaults.VIA_SIZE),
                        via_drill=config.get('via_drill', defaults.VIA_DRILL),
                        track_width=config.get('track_width', defaults.TRACK_WIDTH),
                        clearance=config.get('clearance', defaults.CLEARANCE),
                        grid_step=config.get('grid_step', defaults.GRID_STEP),
                        layers=all_layers,
                        # Thread the fab hole-to-hole minimum so GND-via placement
                        # enforces real drill spacing (issue #125), not the default.
                        hole_to_hole_clearance=config.get(
                            'hole_to_hole_clearance', defaults.HOLE_TO_HOLE_CLEARANCE)
                    )
                    coord = GridCoord(gnd_config.grid_step)

                    # Build obstacle map from PCB data (excluding no nets since we want all obstacles)
                    obstacles = build_base_obstacle_map(self.pcb_data, gnd_config, [])

                    # Add GND vias near existing signal vias
                    gnd_vias = add_gnd_vias_to_existing_board(
                        self.pcb_data,
                        gnd_via_net,
                        gnd_via_distance,
                        gnd_config,
                        obstacles,
                        coord
                    )

                    # Add to new vias list
                    for gv in gnd_vias:
                        self._new_vias.append({
                            'x': gv.x,
                            'y': gv.y,
                            'size': gv.size,
                            'drill': gv.drill,
                            'net_id': gv.net_id,
                            'layers': gv.layers if hasattr(gv, 'layers') else ['F.Cu', 'B.Cu']
                        })
                    total_vias += len(gnd_vias)

                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    print(f"Error adding GND vias: {e}")

        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"Error creating planes: {e}")

        self._operation_result = {
            'mode': 'create',
            'total_vias': total_vias,
            'total_traces': total_traces,
            'total_pads': total_pads,
            'failed_pads': failed_pads,
            'cancelled': self._cancel_requested,
            'affected_nets': sorted(set(expanded_nets)),
            'config': config,
        }

    def _run_repair_planes(self, config):
        """Run disconnected plane repair."""
        from route_disconnected_planes import route_planes as repair_planes

        # Remember the routed floors so _apply_results_to_board can make the live
        # board's DRC constraints consistent with them (issue #160), mirroring
        # route_disconnected_planes.py's auto-fix.
        self._plane_drc_config = dict(config)

        # Flatten assignments into parallel net_names and plane_layers lists
        # For each (nets_list, layers_list) assignment, create an entry for
        # each net on each layer
        assignments = config['assignments']
        net_names = []
        plane_layers = []
        for nets_list, layers_list in assignments:
            for layer in layers_list:
                for net in nets_list:
                    net_names.append(net)
                    plane_layers.append(layer)

        all_layers = self._get_all_copper_layers()

        print(f"Repairing zones: {list(zip(net_names, plane_layers))}")

        try:
            routes_added, regions_connected, new_vias, new_segments, ripped_net_ids = repair_planes(
                input_file=self.board_filename,
                output_file="",
                net_names=net_names,
                plane_layers=plane_layers,
                track_width=config.get('track_width', defaults.TRACK_WIDTH),
                max_track_width=config.get('max_track_width', defaults.REPAIR_MAX_TRACK_WIDTH),
                min_track_width=config.get('track_width', defaults.TRACK_WIDTH),
                clearance=config.get('clearance', defaults.CLEARANCE),
                via_size=config.get('via_size', defaults.VIA_SIZE),
                via_drill=config.get('via_drill', defaults.VIA_DRILL),
                grid_step=config.get('grid_step', defaults.GRID_STEP),
                analysis_grid_step=config.get('analysis_grid_step', defaults.REPAIR_ANALYSIS_GRID_STEP),
                hole_to_hole_clearance=config.get('hole_to_hole_clearance', defaults.HOLE_TO_HOLE_CLEARANCE),
                board_edge_clearance=config.get('board_edge_clearance', defaults.BOARD_EDGE_CLEARANCE),
                max_iterations=config.get('max_iterations', defaults.MAX_ITERATIONS),
                routing_layers=all_layers,
                repair_pads=config.get('repair_pads', True),
                rip_blocker_nets=config.get('rip_blocker_nets', False),
                max_rip_nets=config.get('max_ripup', defaults.MAX_RIPUP),                power_nets=config.get('power_nets'),
                power_nets_widths=config.get('power_nets_widths'),
                # The route tab's "ALL" no-BGA-zones intent, mirrored when
                # re-routing ripped nets on a BGA board (issue #88).
                no_bga_zone=(config.get('no_bga_zones_text', '').upper() == 'ALL'),
                dry_run=True,  # Don't write to file, apply via pcbnew
                pcb_data=self.pcb_data,
                return_results=True,
            )

            self._new_vias = new_vias
            self._new_segments = new_segments
            # Tracks of these nets were ripped to clear blocked pad repairs and
            # re-routed (their new copper is in new_segments/new_vias); the apply
            # step deletes the old tracks before adding the new ones.
            self._ripped_net_ids = ripped_net_ids
            self._operation_result = {
                'mode': 'repair',
                'routes_added': routes_added,
                'regions_connected': regions_connected,
                'cancelled': self._cancel_requested,
                'affected_nets': sorted(set(net_names)),
            }

        except Exception as e:
            import traceback
            traceback.print_exc()
            self._operation_result = {'error': str(e)}

    def _get_all_copper_layers(self):
        """Get all copper layers from PCB data."""
        if hasattr(self.pcb_data, 'board_info') and self.pcb_data.board_info:
            if hasattr(self.pcb_data.board_info, 'copper_layers'):
                return self.pcb_data.board_info.copper_layers
        return ['F.Cu', 'B.Cu']

    def _poll_operation(self):
        """Poll for operation completion."""
        if self._routing_thread and self._routing_thread.is_alive():
            if self._cancel_requested:
                self.status_text.SetLabel("Cancelling...")
            wx.CallLater(100, self._poll_operation)
        else:
            self._on_operation_complete()

    def _on_operation_complete(self):
        """Handle operation completion."""
        self.action_btn.Enable()
        self.cancel_btn.SetLabel("Close")
        self.progress_bar.SetValue(0)

        if self._cancel_requested:
            self._cancel_requested = False
            self.status_text.SetLabel("Cancelled")
            return

        result = getattr(self, '_operation_result', None)
        if not result:
            self.status_text.SetLabel("Operation finished (no result)")
            return

        if 'error' in result:
            self.status_text.SetLabel(f"Error: {result['error']}")
            wx.MessageBox(
                f"Plane operation failed:\n\n{result['error']}",
                "Operation Error",
                wx.OK | wx.ICON_ERROR
            )
            return

        # Apply results to board
        self._apply_results_to_board()

        # Show completion message
        if result['mode'] == 'create':
            msg = f"Plane creation complete!\n\n"
            msg += f"Vias placed: {result.get('total_vias', 0)}\n"
            msg += f"Traces added: {result.get('total_traces', 0)}\n"
            failed_pads = result.get('failed_pads', 0)
            if failed_pads:
                msg += f"Failed pads (no via placed): {failed_pads}\n"
            self.status_text.SetLabel(
                f"Created: {result.get('total_vias', 0)} vias, "
                f"{result.get('total_traces', 0)} traces, "
                f"{failed_pads} failed")
        else:
            msg = f"Plane repair complete!\n\n"
            msg += f"Routes added: {result.get('routes_added', 0)}\n"
            msg += f"Regions connected: {result.get('regions_connected', 0)}\n"
            self.status_text.SetLabel(f"Repaired: {result.get('routes_added', 0)} routes")

        # If any pads failed to get a via, append heuristic suggestions.
        if result['mode'] == 'create' and result.get('failed_pads', 0) > 0:
            try:
                from routing_diagnostics import (
                    suggest_plane_adjustments, format_suggestions_for_dialog)
                suggestions = suggest_plane_adjustments(
                    failed_pads=result.get('failed_pads', 0),
                    total_pads=result.get('total_pads', 0),
                    config=result.get('config', {}))
                block = format_suggestions_for_dialog(suggestions)
                if block:
                    msg += "\n" + block + "\n"
            except Exception as e:
                print(f"Warning: failed to build plane suggestions: {e}")

        msg += "\nUse Edit -> Undo to revert changes."
        wx.MessageBox(msg, "Operation Complete", wx.OK | wx.ICON_INFORMATION)

        # Callback - hand the parent the nets that were just touched so it
        # can invalidate their connectivity cache entries.
        if self.on_planes_complete:
            affected = result.get('affected_nets') or []
            try:
                self.on_planes_complete(affected_nets=affected)
            except TypeError:
                # Older callbacks (no kwarg) - fall back to no-arg call.
                self.on_planes_complete()

        # Refresh net list
        self.net_panel.refresh()

    def _apply_results_to_board(self):
        """Apply planes results (zones + stitch vias + tracks) via the IPC adapter."""
        from kicad_ipc_adapter import (apply_planes_results, get_board,
                                        move_copper_graphics_to_silkscreen)

        new_vias = list(getattr(self, "_new_vias", []) or [])
        new_segments = list(getattr(self, "_new_segments", []) or [])
        new_zones = list(getattr(self, "_new_zones", []) or [])

        # Clear the buffers so the user can re-run without stale state.
        self._new_vias = []
        self._new_segments = []
        self._new_zones = []

        if not (new_vias or new_segments or new_zones):
            return

        board = get_board()
        if board is None:
            wx.MessageBox("Board is no longer open", "Error",
                          wx.OK | wx.ICON_ERROR)
            return

        # Relocate net-less copper logos/graphics to silkscreen (issue #146),
        # matching the CLI plane writer: a copper logo is not a router obstacle,
        # so the pour/repair copper added here would short against it. Done via
        # the IPC adapter (kipy), not the SWIG pcbnew path. The ripped-net old
        # copper is removed inside apply_planes_results below (ripped_net_ids=).
        gfx_moved = move_copper_graphics_to_silkscreen(board)
        if gfx_moved:
            print(f"Moved {gfx_moved} copper graphic(s)/logo(s) to silkscreen")

        try:
            counts = apply_planes_results(
                board,
                pcb_data=self.pcb_data,
                new_vias=new_vias,
                new_segments=new_segments,
                new_zones=new_zones,
                # Nets ripped to clear a blocked pad repair: delete their old
                # (blocking) tracks/vias before adding the re-routed copper in
                # the same commit, or the repair trace would short them (#112).
                ripped_net_ids=getattr(self, '_ripped_net_ids', None),
                message="KiCadRoutingTools: planes",
            )
        except Exception as e:
            wx.MessageBox(
                f"Failed to apply plane operations:\n\n{e}",
                "Apply error", wx.OK | wx.ICON_ERROR,
            )
            return

        if counts["zones_skipped"]:
            for net_name, layer in counts.get("_skipped_keys", []):
                print(f"Skipped duplicate zone for '{net_name}' on {layer}")

        print(f"Added to board: {counts['zones']} zones, "
              f"{counts['vias']} vias, {counts['tracks']} tracks "
              f"({counts['zones_skipped']} zones skipped as duplicates)")

        # KiCad fills zones server-side on commit; no separate ZoneFiller
        # needed under IPC. If a fill is missing, the user can press B in
        # the PCB Editor to trigger a full board re-fill.

        # Make the open project's DRC floors consistent with the plane routing
        # values (issue #160), mirroring route_planes.py's auto-fix. Edits
        # .kicad_pro on disk (kipy can't write live design settings); the user
        # must reload the project. Best-effort.
        if counts['zones'] or counts['vias'] or counts['tracks']:
            from gui_utils import apply_drc_settings_fix
            apply_drc_settings_fix(getattr(self, "_plane_drc_config", None))

        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()

    def get_assignments(self):
        """Get list of net→layer assignments."""
        return self.assignment_panel.get_assignments()
