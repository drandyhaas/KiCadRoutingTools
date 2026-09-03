"""
KiCad Routing Tools - Planes GUI Components

Provides wx-based panels for power/ground plane (pour) creation.
Plane REPAIR is a default part of every route.py run since #562 -- the
route step's in-run plane finalize owns taps, region joins, and the
kicad-oracle completion verify; there is no repair mode here anymore.
"""

import os
import sys
import wx

# Add parent directory to path
PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
# #522 layout: the engine modules live in py_router/ under the repo root.
# The exists() guard keeps a FLAT installed layout (PCM zip) working too.
_ENGINE_DIR = os.path.join(ROOT_DIR, 'py_router')
if os.path.isdir(_ENGINE_DIR) and _ENGINE_DIR not in sys.path:
    sys.path.insert(0, _ENGINE_DIR)
# py_placer/ holds the placement package (placement.groups / .fanout_clearance
# are imported from here) and py_tools/ the instruments. Same exists() guard so
# a FLAT installed layout (PCM zip) keeps working.
for _sib in ('py_placer', 'py_tools'):
    _d = os.path.join(ROOT_DIR, _sib)
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.append(_d)

import routing_defaults as defaults
from kicad_parser import mm_to_iu
from .fanout_gui import NetSelectionPanel
from .gui_utils import StdoutRedirector, board_minima_from_live


def _live_board_edge_clearance():
    """The LIVE board's declared copper-to-edge rule in mm, else 0.0.

    The GUI counterpart of the CLI's
    ``board_constraint(input_file, 'min_copper_edge_clearance')``: both
    fronts must inset a pour by whatever the board declares, or they build
    different pour outlines and every downstream consumer of the exact fill
    (notably the plane-fragility cost field) diverges with them.

    0.0 means "the board declares nothing", which the caller reads as
    fall-back-to-PLANE_EDGE_CLEARANCE -- the same meaning board_constraint's
    None carries on the CLI side.
    """
    try:
        import pcbnew
        board = pcbnew.GetBoard()
        if board is None:
            return 0.0
        return (board.GetDesignSettings().m_CopperEdgeClearance or 0) / 1e6
    except Exception:
        return 0.0


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

    def __init__(self, parent, pcb_data, get_selected_nets_callback, on_ask_ai=None):
        """
        Create a plane assignment panel.

        Args:
            parent: Parent window
            pcb_data: PCBData object with board info
            get_selected_nets_callback: Function that returns currently selected nets
            on_ask_ai: Callback for the "Ask AI" button that recommends
                net -> layer mappings (issue #53); button hidden if None.
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.get_selected_nets = get_selected_nets_callback
        self.on_ask_ai = on_ask_ai
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

        # Checkbox grid for layers, 5 per row. A GridSizer (not WrapSizer) on
        # purpose: WrapSizer's reported min height depends on the width of the
        # last layout pass, and a pass at a narrow width can make it claim one
        # row PER LAYER - on a 10-layer board that inflated the assignments
        # box and crushed the options area below (user-reported mis-spacing).
        copper_layers = self._get_copper_layers()
        layer_sizer = wx.GridSizer(cols=5, hgap=10, vgap=4)
        self.layer_checks = {}
        for layer in copper_layers:
            cb = wx.CheckBox(self, label=layer)
            cb.SetToolTip(f"Include {layer} in this assignment")
            self.layer_checks[layer] = cb
            layer_sizer.Add(cb, 0)

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

        if self.on_ask_ai is not None:
            self.ask_claude_btn = wx.Button(self, label="Ask AI", style=wx.BU_EXACTFIT)
            self.ask_claude_btn.SetToolTip(
                "Run the recommend-plane-mappings skill: recommends which nets "
                "deserve planes and on which layers (GND adjacency, GND/VCC pairing), "
                "then fills this assignment list. Takes a few minutes.")
            self.ask_claude_btn.Bind(wx.EVT_BUTTON, lambda event: self.on_ask_ai())
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

    def __init__(self, parent, on_ask_ai=None):
        """Create the options panel.

        Args:
            on_ask_ai: Callback for the "Ask AI" button next to the
                GND via distance field (issue #39); button hidden if None.
        """
        super().__init__(parent)
        self.on_ask_ai = on_ask_ai
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
        # Checkbox + spin row, SAME convention as the basic tab's geometry
        # floors (track width etc.): UNCHECKED = automatic (pour follows the
        # routed Min Clearance, auto-stepping toward the fab floor when a
        # dense BGA lattice can't be threaded -- the ottercast sealed-field
        # fix); CHECKING the box overrides with the typed value.
        r = defaults.PARAM_RANGES['plane_zone_clearance']
        _zrow = wx.BoxSizer(wx.HORIZONTAL)
        self.zone_clearance_check = wx.CheckBox(self, label="")
        self.zone_clearance_check.SetValue(False)
        self.zone_clearance_check.SetToolTip(
            "Override the zone (pour) clearance (unchecked = follow Min "
            "Clearance, auto-reduced to thread dense BGA fields).")
        self.zone_clearance = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                 initial=defaults.PLANE_ZONE_CLEARANCE, inc=r['inc'])
        self.zone_clearance.SetDigits(r['digits'])
        self.zone_clearance.SetToolTip("Clearance from zone fill to other copper")
        self.zone_clearance.Enable(False)
        self.zone_clearance_check.Bind(
            wx.EVT_CHECKBOX,
            lambda evt: self.zone_clearance.Enable(evt.IsChecked()))
        _zrow.Add(self.zone_clearance_check, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        _zrow.Add(self.zone_clearance, 1, wx.EXPAND)
        grid.Add(_zrow, 0, wx.EXPAND)

        zone_sizer.Add(grid, 0, wx.EXPAND | wx.ALL, 5)
        # #581: the via-in-pad policy controls (Allow via-in-pad checkbox +
        # Same-net Pad Clearance spin) moved to the TOP of the Basic tab's
        # Options box -- one policy shared by every step; this tab reads it
        # through get_shared_params.

        # #487: thermal-relief pad connections (the zone writer always
        # supported them; every front hardcoded solid). Control named after
        # the engine param so AI plans can set it.
        self.thermal_relief = wx.CheckBox(self, label="Thermal relief pad connections")
        self.thermal_relief.SetValue(False)
        self.thermal_relief.SetToolTip(
            "Connect pads to the pour with thermal-relief spokes instead of solid "
            "copper (easier hand soldering/rework; solid = lowest impedance)")
        zone_sizer.Add(self.thermal_relief, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.thermal_vias = wx.CheckBox(self, label="Thermal via arrays under exposed pads")
        self.thermal_vias.SetValue(defaults.THERMAL_VIAS)
        self.thermal_vias.SetToolTip(
            "Give exposed/thermal pads (>= 2mm both axes) a lattice of vias into "
            "the plane instead of a single shared via (#487)")
        zone_sizer.Add(self.thermal_vias, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        sizer.Add(zone_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # (No Blocker Handling box: the pour step does no routing (#562
        # pours-first -- unconditional, the tap machinery is deleted, there
        # is no kill switch), so there are no tap corridors to rip.
        # route_planes.py has no --rip-blocker-nets anymore either; only the
        # standalone repair_planes.py utility keeps it, for boards routed
        # outside the chain.)

        # Area via stitching (#485). Controls named after the engine params
        # (stitch_vias / stitch_pitch) so AI plans can set them.
        stitch_box = wx.StaticBox(self, label="Area Via Stitching")
        stitch_sizer = wx.StaticBoxSizer(stitch_box, wx.VERTICAL)

        self.stitch_vias = wx.CheckBox(self, label="Stitch plane layers with a via lattice")
        self.stitch_vias.SetValue(False)
        self.stitch_vias.SetToolTip(
            "Bond each plane net's pours across layers with a periodic via "
            "lattice (EMI/SI practice). Applies to the selected plane nets "
            "that own 2+ plane layers; every site is checked against the "
            "predicted zone fill and the same clearance/hole-to-hole/edge "
            "checks as any routed via.")
        stitch_sizer.Add(self.stitch_vias, 0, wx.ALL, 5)

        self.stitch_edge_fence = wx.CheckBox(
            self, label="Board-edge via fence (EMI guard ring)")
        self.stitch_edge_fence.SetValue(False)
        self.stitch_edge_fence.SetToolTip(
            "Add a row of stitching vias tracking the board outline(s). Same "
            "net rule and site checks as the lattice; works with or without "
            "it.")
        stitch_sizer.Add(self.stitch_edge_fence, 0,
                         wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        stitch_grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        stitch_grid.AddGrowableCol(1)
        stitch_grid.Add(wx.StaticText(self, label="Lattice Pitch (mm):"), 0,
                        wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['stitch_pitch']
        self.stitch_pitch = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                              initial=defaults.STITCH_PITCH,
                                              inc=r['inc'])
        self.stitch_pitch.SetDigits(r['digits'])
        self.stitch_pitch.SetToolTip("Spacing between stitching vias")
        stitch_grid.Add(self.stitch_pitch, 0, wx.EXPAND)

        stitch_grid.Add(wx.StaticText(self, label="Max Frequency (MHz, 0=off):"),
                        0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['stitch_max_freq']
        self.stitch_max_freq = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                 initial=0.0, inc=r['inc'])
        self.stitch_max_freq.SetDigits(r['digits'])
        self.stitch_max_freq.SetToolTip(
            "Maximum frequency of interest: derives the pitch as lambda/20 "
            "from the stackup's dielectric (overrides Lattice Pitch). 0 = "
            "use Lattice Pitch as-is.")
        stitch_grid.Add(self.stitch_max_freq, 0, wx.EXPAND)

        stitch_grid.Add(wx.StaticText(self, label="Fence Pitch (mm, 0=lattice):"),
                        0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['stitch_fence_pitch']
        self.stitch_fence_pitch = wx.SpinCtrlDouble(self, min=r['min'],
                                                    max=r['max'],
                                                    initial=0.0, inc=r['inc'])
        self.stitch_fence_pitch.SetDigits(r['digits'])
        self.stitch_fence_pitch.SetToolTip(
            "Via spacing along the edge fence. 0 = follow the lattice pitch.")
        stitch_grid.Add(self.stitch_fence_pitch, 0, wx.EXPAND)

        stitch_grid.Add(wx.StaticText(self, label="Fence Inset (mm, 0=auto):"),
                        0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['stitch_inset']
        self.stitch_inset = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                              initial=0.0, inc=r['inc'])
        self.stitch_inset.SetDigits(r['digits'])
        self.stitch_inset.SetToolTip(
            "Distance from the board edge to the fence via centers. 0 = "
            "auto (edge clearance + the pour's fill margin).")
        stitch_grid.Add(self.stitch_inset, 0, wx.EXPAND)
        stitch_sizer.Add(stitch_grid, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        sizer.Add(stitch_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

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
        if self.on_ask_ai is not None:
            dist_sizer = wx.BoxSizer(wx.HORIZONTAL)
            dist_sizer.Add(self.gnd_via_distance, 1, wx.EXPAND | wx.RIGHT, 5)
            self.ask_claude_btn = wx.Button(self, label="Ask AI", style=wx.BU_EXACTFIT)
            self.ask_claude_btn.SetToolTip(
                "Run the find-high-speed-nets skill: looks up component datasheets to "
                "classify nets by speed and recommends this distance for GND return "
                "vias. Takes a few minutes (web lookups).")
            self.ask_claude_btn.Bind(wx.EVT_BUTTON, lambda event: self.on_ask_ai())
            dist_sizer.Add(self.ask_claude_btn, 0)
            gnd_grid.Add(dist_sizer, 0, wx.EXPAND)
        else:
            gnd_grid.Add(self.gnd_via_distance, 0, wx.EXPAND)

        gnd_grid.Add(wx.StaticText(self, label="GND Net Name:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.gnd_via_net = wx.TextCtrl(self, value=defaults.GND_VIA_NET)
        self.gnd_via_net.SetToolTip(
            "Pin GND return vias to this net. Leave EMPTY for auto: each signal "
            "returns to its own ground domain (plain GND on a board with one "
            "ground; AGND/DGND matched per signal on a split-ground board). "
            "Matches the CLI --gnd-via-net default.")
        gnd_grid.Add(self.gnd_via_net, 0, wx.EXPAND)

        gnd_sizer.Add(gnd_grid, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)
        sizer.Add(gnd_sizer, 0, wx.EXPAND)

        self.SetSizer(sizer)

    def get_config(self):
        """Get the configuration values."""
        return {
            'zone_clearance': (self.zone_clearance.GetValue()
                               if self.zone_clearance_check.GetValue() else None),
            'add_gnd_vias': self.add_gnd_vias_check.GetValue(),
            'gnd_via_distance': self.gnd_via_distance.GetValue(),
            'gnd_via_net': self.gnd_via_net.GetValue(),
            'thermal_relief': self.thermal_relief.GetValue(),
            'thermal_vias': self.thermal_vias.GetValue(),
            'stitch_vias': self.stitch_vias.GetValue(),
            'stitch_pitch': self.stitch_pitch.GetValue(),
            # 0 = auto/off in the controls -> None for the engine (CLI parity)
            'stitch_edge_fence': self.stitch_edge_fence.GetValue(),
            'stitch_fence_pitch': self.stitch_fence_pitch.GetValue() or None,
            'stitch_inset': self.stitch_inset.GetValue() or None,
            'stitch_max_freq': self.stitch_max_freq.GetValue() or None,
        }


# (RepairPlanesOptionsPanel is GONE, #562: plane repair is a default part
# of every route.py run -- the in-run plane finalize runs the repair engine
# (pad taps + region joins), the plane-copper cleanup, and the kicad-oracle
# exact-fill verify at the ROUTE step's parameters. The standalone
# repair_planes.py CLI remains for boards routed outside the
# chain; it needs no GUI surface.)


class PlanesTab(wx.Panel):
    """Tab for copper plane (pour) creation -- repair lives in the route step (#562)."""

    def __init__(self, parent, pcb_data, board_filename,
                 get_shared_params=None, on_planes_complete=None,
                 get_connectivity_check=None, append_log=None,
                 sync_pcb_data_callback=None, get_ai_params=None):
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
            get_ai_params: Callback returning the AI tab's
                {'model', 'effort'} selections for headless runs
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self.get_shared_params = get_shared_params
        self.get_ai_params = get_ai_params
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
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)

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

        main_sizer.Add(net_box_sizer, 1, wx.EXPAND | wx.ALL, 5)

        # Right side: Mode, layers, options, buttons
        right_sizer = wx.BoxSizer(wx.VERTICAL)

        # (No Mode selector, #562: repair is part of routing now; this tab
        # creates pours + via features only.)

        # Plane assignments
        self.assign_box = wx.StaticBox(self, label="Net → Layer Assignments")
        self.assign_sizer = wx.StaticBoxSizer(self.assign_box, wx.VERTICAL)

        self.assignment_panel = PlaneAssignmentPanel(
            self, self.pcb_data,
            get_selected_nets_callback=lambda: self.net_panel.get_selected_nets(),
            on_ask_ai=self._on_ask_ai_plane_mappings
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
            self.options_scroll, on_ask_ai=self._on_ask_ai_gnd_via)
        options_scroll_sizer.Add(self.create_options, 0, wx.EXPAND | wx.BOTTOM, 5)

        self.options_scroll.SetSizer(options_scroll_sizer)
        self.options_scroll.SetScrollRate(0, 10)
        self.options_scroll.FitInside()
        # Floor: never let the surrounding boxes crush the options area below
        # ~6 parameter rows (it scrolls internally beyond that).
        self.options_scroll.SetMinSize((-1, 240))
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
        self.action_btn.SetToolTip("Create the plane pours + via features")
        self.action_btn.Bind(wx.EVT_BUTTON, self._on_action)
        btn_sizer.Add(self.action_btn, 1, wx.RIGHT, 5)

        self.cancel_btn = wx.Button(self, label="Close")
        self.cancel_btn.SetToolTip("Close dialog (or cancel operation if in progress)")
        self.cancel_btn.Bind(wx.EVT_BUTTON, self._on_cancel_or_close)
        btn_sizer.Add(self.cancel_btn, 1)

        right_sizer.Add(btn_sizer, 0, wx.EXPAND)

        main_sizer.Add(right_sizer, 1, wx.EXPAND | wx.ALL, 5)

        self.SetSizer(main_sizer)

    def _on_ask_ai_gnd_via(self):
        """Run find-high-speed-nets headless and fill the GND via distance
        field from its recommendation (issue #39)."""
        from .ai_gui import run_skill_dialog, board_path_for_analysis
        from .ai_backend import ANALYSIS_CONSTRAINT

        board = board_path_for_analysis(self.board_filename)
        if board is None:
            return
        value = run_skill_dialog(
            self, "AI: recommend GND via distance",
            "find-high-speed-nets", os.path.abspath(board),
            ANALYSIS_CONSTRAINT + " After the report, end "
            "your reply with exactly one line of the form "
            "RESULT=<recommended --gnd-via-distance in mm> "
            "(a bare number), e.g. RESULT=2.5",
            intro=f"Running find-high-speed-nets on {os.path.basename(board)} ...\n"
                  "(datasheet lookups; typically a few minutes)",
            # Obey the backend/model/effort selected on the AI tab
            ai_params=self.get_ai_params() if self.get_ai_params else None)
        if value is not None:
            self._apply_gnd_via_recommendation(value)

    def _on_ask_ai_plane_mappings(self):
        """Run recommend-plane-mappings headless and fill the assignment
        list from its recommendation (issue #53)."""
        from .ai_gui import run_skill_dialog, board_path_for_analysis
        from .ai_backend import ANALYSIS_CONSTRAINT

        board = board_path_for_analysis(self.board_filename)
        if board is None:
            return
        value = run_skill_dialog(
            self, "AI: recommend plane mappings",
            "recommend-plane-mappings", os.path.abspath(board),
            ANALYSIS_CONSTRAINT + " After the report, end "
            "your reply with exactly one line of the form "
            "RESULT=<net>:<layer>;<net>|<net>:<layer> "
            "(groups separated by ';', nets sharing a layer joined by '|', exact "
            "net names, one copper layer per group), e.g. RESULT=GND:In1.Cu;VCC:In2.Cu",
            intro=f"Running recommend-plane-mappings on {os.path.basename(board)} ...\n"
                  "(board analysis; typically a few minutes)",
            ai_params=self.get_ai_params() if self.get_ai_params else None)
        if value is not None:
            self._apply_plane_mappings_recommendation(value)

    def _apply_plane_mappings_recommendation(self, value):
        """Validate the AI's RESULT value and fill the assignment list."""
        net_names = {net.name for net in self.pcb_data.nets.values() if net.name}
        copper = self.assignment_panel._get_copper_layers()
        assignments, notes = parse_plane_mappings_result(value, copper, net_names)
        for note in notes:
            if self.append_log:
                self.append_log(f"AI: {note}\n")
        if not assignments:
            if self.append_log:
                self.append_log(f"AI: no usable plane mappings in {value!r}\n")
            return

        existing = self.assignment_panel.get_assignments()
        if existing:
            choice = wx.MessageBox(
                "The assignment list already has entries.\n\n"
                "Yes = replace them with the AI's recommendation\n"
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
            self.append_log(f"AI recommended plane mappings: {shown}\n")

    def _apply_gnd_via_recommendation(self, value):
        """Validate the AI's RESULT value and apply it to the GUI controls."""
        try:
            distance = float(value)
        except ValueError:
            if self.append_log:
                self.append_log(f"AI: unusable GND via distance {value!r}\n")
            return
        r = defaults.PARAM_RANGES['gnd_via_distance']
        clamped = max(r['min'], min(r['max'], distance))
        self.create_options.gnd_via_distance.SetValue(clamped)
        self.create_options.add_gnd_vias_check.SetValue(True)
        if self.append_log:
            note = "" if clamped == distance else f" (clamped from {distance})"
            self.append_log(f"AI recommended GND via distance: {clamped} mm{note}; "
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
        # Validation - assignments required
        assignments = self.assignment_panel.get_assignments()
        if not assignments:
            wx.MessageBox(
                "Please add at least one net \u2192 layer assignment for plane creation.\n\n"
                "Select nets on the left, check target layers, then click 'Add Assignment'.",
                "No Assignments",
                wx.OK | wx.ICON_WARNING
            )
            return

        config = self._build_mode_config()
        config['assignments'] = assignments

        # Disable UI
        self.action_btn.Disable()
        self.cancel_btn.SetLabel("Cancel")
        self._cancel_requested = False

        self.status_text.SetLabel("Creating planes...")
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

    def _build_mode_config(self):
        """Assemble the engine config for a create run.

        The ONE assembly point (the kwarg-parity gate drives this method
        directly, so what it returns IS what the engine call reads -- no
        hand-mirrored dict to drift). Repair mode is GONE (#562): plane
        repair is a default part of every route.py run's plane finalize;
        this tab creates pours + via features only.
        """
        config = self.create_options.get_config()
        config.update(self.get_shared_params() if self.get_shared_params else {})
        return config

    def _run_planes_operation(self, config):
        """Run plane operation in background thread."""
        import sys

        from fab_tiers import set_fab_tier_from_config
        set_fab_tier_from_config(config)
        original_stdout = sys.stdout
        if self.append_log:
            sys.stdout = StdoutRedirector(self.append_log, original_stdout)

        try:
            self._run_create_planes(config)

        except Exception as e:
            import traceback
            traceback.print_exc()
            self._operation_result = {'error': str(e)}

        finally:
            sys.stdout = original_stdout

    def _make_progress_callback(self):
        """Engine progress hook (issue #364): marshals engine-thread milestone
        updates onto the UI thread so the status bar / gauge track the plane
        create/repair phases instead of freezing on "Creating planes...".
        The plan executor mirrors status_text/progress_bar into the AI
        tab via _status_source, so it lights up there too."""
        import time as _time

        def on_progress(current, total, label=""):
            wx.CallAfter(self._update_progress, current, total, label)
            # Brief sleep releases the GIL so the main thread can paint the
            # CallAfter update (same pattern as the route tab's callback).
            _time.sleep(0.01)
        return on_progress

    def _update_progress(self, current, total, label):
        """Update progress bar and status text (must run on the UI thread)."""
        if total > 0:
            self.progress_bar.SetRange(100)
            self.progress_bar.SetValue(min(100, int(100 * current / total)))
            self.status_text.SetLabel(f"{label} ({current}/{total})")
        else:
            self.progress_bar.Pulse()  # Indeterminate phase
            self.status_text.SetLabel(label)

    def _apply_status(self, message):
        """Status update for work running ON the UI thread (the apply path).

        _update_progress is marshalled from the engine thread via CallAfter, so
        the main loop paints it. Anything running ON the main thread blocks that
        loop, so a bare SetLabel would not repaint until the work finished --
        leaving the previous phase's label on screen and reading as a hang.
        Guarded: a status update must never break the apply. See
        gui_utils.ui_thread_status for why the repaint is deliberately narrow
        (no Gauge.Pulse) inside an action plugin.
        """
        from .gui_utils import ui_thread_status
        ui_thread_status(getattr(self, 'status_text', None),
                         getattr(self, 'progress_bar', None), message)

    def _run_create_planes(self, config):
        """Run plane creation."""
        # Cleared here; repopulated from create_plane's returned ripped set
        # below (rip_blocker_nets rips signal nets whose original copper must
        # be deleted from the live board before the new/restored copper is
        # applied -- CLI strip-and-replace parity, b2557cd).
        self._ripped_net_ids = []
        # Remember the routed floors so _apply_results_to_board can make the live
        # board's DRC constraints consistent with them (issue #160).
        self._plane_drc_config = dict(config)
        # Start a fresh clearance ledger so a prior operation's fine-pitch
        # clearance doesn't leak into this board's DRC floor.
        import clearance_ledger
        clearance_ledger.reset()
        from route_planes import create_plane
        from add_gnd_vias import add_gnd_vias_to_existing_board
        from routing_config import GridRouteConfig, GridCoord
        from obstacle_map import build_base_obstacle_map

        # Get assignments: each is (nets_list, layers_list)
        assignments = config['assignments']

        # Routing layers for the plane-connection traces. CLI/GUI PARITY:
        # route_planes.py defaults all_layers to ['F.Cu'] + plane_layers +
        # ['B.Cu'] (outer layers + the pour layers) when --layers is omitted, NOT
        # every copper layer. Passing all 6 copper layers (the old GUI behavior)
        # hands the router 2 extra inner layers to thread connection traces
        # through -> different, worse-balanced plane routing than the CLI on the
        # same board. Mirror the CLI default: outer layers + the assignment's
        # plane layers, deduped in order. (#362 plane-parity)
        _plane_layers_in_order = []
        for _nets_list, _layers_list in config['assignments']:
            for _L in _layers_list:
                if _L not in _plane_layers_in_order:
                    _plane_layers_in_order.append(_L)
        _seen = set()
        all_layers = [l for l in (['F.Cu'] + _plane_layers_in_order + ['B.Cu'])
                      if not (l in _seen or _seen.add(l))]

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

        # Remember which nets this plane op touched, so the post-apply plane
        # copper cleanup (CLI parity) scopes to them (see _apply_results_to_board).
        self._plane_net_names = list(dict.fromkeys(expanded_nets))

        # #439: build net_clearances from the live board so plane taps/vias and
        # ripped-net reroutes honor KiCad's cross-class max(A,B) -- mirrors
        # route_planes.py and the route tab (swig_gui). Passing an explicit map
        # STOPS the engine's internal always-cap auto-read, so honoring classes
        # (Min Clearance override unchecked) actually reaches the router.
        # Checking Min Clearance (== the CLI passing --clearance) caps each class
        # at min(class, clearance); unchecked routes each class in full. The
        # clamp/ceiling params are also threaded so create_plane's OWN internal
        # auto-read (the ripped-net reroute path) honors/caps the same way.
        _plane_clearance = config.get('clearance', defaults.CLEARANCE)
        _plane_clamp = config.get('clamp_netclasses', False)
        _plane_ceiling = _plane_clearance if _plane_clamp else None
        _plane_net_clearances = {}
        plane_error = None      # set if plane creation raises; surfaced to the dialog
        try:
            from .fanout_gui import _get_net_classes_from_board
            from .swig_gui import _get_netclass_parameters
            all_net_to_class, all_class_names = _get_net_classes_from_board()
            class_clearance_cache = {}
            for cname in all_class_names:
                params = _get_netclass_parameters(cname)
                if params:
                    class_clearance_cache[cname] = params.get('clearance', _plane_clearance)
                else:
                    class_clearance_cache[cname] = _plane_clearance
            for net in self.pcb_data.nets.values():
                cname = all_net_to_class.get(net.name, 'Default')
                _plane_net_clearances[net.net_id] = class_clearance_cache.get(
                    cname, _plane_clearance)
            if _plane_clamp:
                _plane_net_clearances = {nid: min(clr, _plane_clearance)
                                         for nid, clr in _plane_net_clearances.items()}
        except Exception as e:
            print(f"Warning: Could not get net class clearances: {e}")
            _plane_net_clearances = None

        failed_pads = 0
        try:
            (vias, traces, pads_needing, new_vias, new_segments, new_zones,
             failed_pads, ripped_net_ids, reconnect_swap_data,
             reconnect_strips) = create_plane(
                input_file=self.board_filename,
                output_file="",
                net_names=expanded_nets,
                plane_layers=expanded_layers,
                via_size=config.get('via_size', defaults.VIA_SIZE),
                via_drill=config.get('via_drill', defaults.VIA_DRILL),
                track_width=config.get('track_width', defaults.TRACK_WIDTH),
                clearance=config.get('clearance', defaults.CLEARANCE),
                zone_clearance=config.get('zone_clearance'),
                min_thickness=config.get('min_thickness', defaults.PLANE_MIN_THICKNESS),
                thermal_relief=config.get('thermal_relief', False),
                thermal_vias=config.get('thermal_vias', defaults.THERMAL_VIAS),
                stitch_vias=config.get('stitch_vias', False),
                stitch_pitch=config.get('stitch_pitch', defaults.STITCH_PITCH),
                stitch_edge_fence=config.get('stitch_edge_fence', False),
                stitch_fence_pitch=config.get('stitch_fence_pitch'),
                stitch_inset=config.get('stitch_inset'),
                stitch_max_freq=config.get('stitch_max_freq'),
                grid_step=config.get('grid_step', defaults.GRID_STEP),
                hole_to_hole_clearance=config.get('hole_to_hole_clearance', defaults.HOLE_TO_HOLE_CLEARANCE),
                # #424: route_planes.py passes --ripup-blocker-select to
                # create_plane; the shared Basic-tab dropdown reaches the plane
                # engine here (supplied by the planes get_shared_params).
                ripup_blocker_select=config.get('ripup_blocker_select',
                                                defaults.RIPUP_BLOCKER_SELECT),
                # #381 D6: per-layer cost bias from the shared Basic-tab control
                # (route_planes.py passes --layer-costs); None -> engine default.
                layer_costs=config.get('layer_costs') or None,
                # #381 D6: plane knobs the CLI threads but the GUI dropped -- now
                # config-driven, defaulting to the same value route_planes.py's
                # argparse uses so current GUI behavior is unchanged unless a
                # plan/control sets them.
                plane_proximity_radius=config.get('plane_proximity_radius', 3.0),
                plane_proximity_cost=config.get('plane_proximity_cost', 2.0),
                plane_track_via_clearance=config.get('plane_track_via_clearance',
                                                     defaults.PLANE_TRACK_VIA_CLEARANCE),
                voronoi_seed_interval=config.get('voronoi_seed_interval', 2.0),
                plane_max_iterations=config.get('plane_max_iterations', defaults.MAX_ITERATIONS),
                debug_lines=config.get('debug_lines', False),
                add_teardrops=config.get('add_teardrops', False),
                verbose=config.get('verbose', False),
                # Resolution ORDER must match route_planes.py main(): an
                # explicit value wins, else the BOARD's own declared
                # min_copper_edge_clearance, else PLANE_EDGE_CLEARANCE (0.5).
                #
                # The board lookup is the parity-critical middle term. The CLI
                # reads it via board_constraint(input_file,
                # 'min_copper_edge_clearance'); this front reads the same rule
                # off the LIVE board. Skipping it made the GUI inset every pour
                # at 0.5 while the CLI used whatever the board declared, and a
                # different pour OUTLINE is not a cosmetic difference: it
                # changes the exact fill, which feeds the plane-fragility cost
                # field, which re-prices the A* -- measured as 12/8 divergent
                # GND segments and a relocated via on splitflap_driver.
                # (It went unnoticed because that fixture declares no rule and
                # KiCad's default happens to BE 0.5.)
                #
                # PLANE_EDGE_CLEARANCE (0.5), NOT the generic BOARD_EDGE_CLEARANCE
                # (0.0), is the final fallback: using 0.0 let GUI plane pours run
                # to the edge -- a parity gap and a fab concern (#362).
                # `or` (not a plain default): the shared Basic-tab
                # board_edge_clearance is a ROUTING value that defaults to 0.0
                # and gets merged into the plane config, overriding a plain
                # default. (#362)
                board_edge_clearance=(config.get('board_edge_clearance')
                                      or _live_board_edge_clearance()
                                      or defaults.PLANE_EDGE_CLEARANCE),
                all_layers=all_layers,
                dry_run=True,  # Don't write to file, apply via pcbnew
                # Re-route a ripped wide power net at its proper width.
                power_nets=config.get('power_nets'),
                power_nets_widths=config.get('power_nets_widths'),
                # Match signal routing's No-BGA-Zones intent when rerouting
                # ripped nets on a BGA board (issue #88). The route tab's
                # "ALL" means disable every BGA exclusion zone.
                no_bga_zone=(config.get('no_bga_zones_text', '').upper() == 'ALL'),
                pcb_data=self.pcb_data,
                return_results=True,
                layer_nets=layer_nets,
                # #581: from the Basic tab via get_shared_params (explicit;
                # -1 = via-in-pad allowed).
                same_net_pad_clearance=config.get('same_net_pad_clearance', -1.0),
                # #381 D6: config-driven (was hardcoded True). Interactive
                # default stays True -- an interactive re-create on a live board
                # should skip re-emitting an existing zone. A plan can override
                # to False (the CLI default) to match a route_planes.py replay.
                skip_existing_zones=config.get('skip_existing_zones', True),
                net_clearances=_plane_net_clearances,
                clamp_netclasses=_plane_clamp,
                clearance_ceiling=_plane_ceiling,
                progress_callback=self._make_progress_callback(),
                cancel_check=lambda: self._cancel_requested,
            )

            total_vias = vias
            total_traces = traces
            total_pads = pads_needing
            self._new_vias = new_vias
            self._new_segments = new_segments
            self._new_zones = new_zones
            self._ripped_net_ids = ripped_net_ids
            self._reconnect_swap_data = reconnect_swap_data or {}
            # #508 finding 1: input copper the in-memory reconnect's cleanup
            # withdrew from pcb_data -- the applier deletes these individually
            # (the whole-net delete only covers ripped nets).
            self._strip_segments = reconnect_strips or []

            # Add GND return vias if enabled
            if config.get('add_gnd_vias', False):
                try:
                    gnd_via_distance = config.get('gnd_via_distance', defaults.GND_VIA_DISTANCE)
                    gnd_via_net = config.get('gnd_via_net', defaults.GND_VIA_NET)

                    def _gnd_via_edge_clearance():
                        """The board's copper-to-edge rule, mm. 0.0 when it cannot
                        be read -- add_gnd_vias' own bounding-box backstop then
                        still keeps the via inside the outline."""
                        try:
                            import pcbnew as _pcbnew
                            return (_pcbnew.GetBoard().GetDesignSettings()
                                    .m_CopperEdgeClearance or 0) / 1e6
                        except Exception:
                            return 0.0

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
                            'hole_to_hole_clearance', defaults.HOLE_TO_HOLE_CLEARANCE),
                        # CLI parity with route_planes main: copper-to-EDGE, so
                        # build_base_obstacle_map populates the static off-board
                        # keep-out (#422) and a return via cannot land outside the
                        # outline. Left unset this carried the 0.0 default and a
                        # GND via was placed 1.40mm beyond the board edge.
                        # Read from the live board's design settings (nm -> mm),
                        # NOT cfg 'board_edge_clearance': that is the plane-zone
                        # inset, a pour aesthetic, not an enforced routing floor
                        # (same reasoning as _run_kicad_oracle_after_apply).
                        board_edge_clearance=_gnd_via_edge_clearance(),
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
            # SURFACE it. This used to only print, and the result dict below was
            # then built with zeros and no 'error' key -- so a hard failure inside
            # create_plane was indistinguishable from a legitimate no-op, and the
            # traceback went to stdout, which is invisible unless KiCad's scripting
            # console happens to be open. The dialog reported "0 vias, 0 traces"
            # (now "No plane was created") and the user had nothing to act on.
            plane_error = f"{type(e).__name__}: {e}"

        # How many requested pours the ENGINE kept as-is because that net already
        # has a zone on that layer (skip_existing_zones). Without this the dialog
        # cannot distinguish "already there, nothing to do" from a real failure --
        # both arrive as zero new zones.
        try:
            _have = {(z.net_name, z.layer)
                     for z in (getattr(self.pcb_data, 'zones', None) or [])}
            zones_kept = sum(1 for _n, _l in zip(expanded_nets, expanded_layers)
                             if (_n, _l) in _have)
        except Exception:
            zones_kept = 0

        self._operation_result = {
            'mode': 'create',
            'zones_kept': zones_kept,
            'total_vias': total_vias,
            'total_traces': total_traces,
            'total_pads': total_pads,
            'failed_pads': failed_pads,
            'cancelled': self._cancel_requested,
            'affected_nets': sorted(set(expanded_nets)),
            'config': config,
        }
        if plane_error:
            self._operation_result['error'] = plane_error


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
        # Button re-enabled AFTER apply (finally): it is the plan executor's
        # busy signal, and error/completion MessageBoxes pump events -- an
        # early enable let the executor start the next step mid-apply.
        try:
            self._on_operation_complete_body()
        finally:
            self.action_btn.Enable()
            self.cancel_btn.SetLabel("Close")

    def _on_operation_complete_body(self):
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

        # Show completion message. Lead with ZONES: the pour is what this tab
        # produces. Vias/traces are legacy tap counters that are structurally 0
        # now the plane step places no taps (#492/#562), so only mention them
        # when non-zero -- otherwise every successful pour announced "0 and 0".
        zones_added, zones_skipped = getattr(self, '_last_zone_counts', (0, 0))
        nets = result.get('affected_nets') or []
        failed_pads = result.get('failed_pads', 0)

        if zones_added:
            msg = "Plane creation complete!\n\n"
            msg += f"Planes poured: {zones_added} zone(s)"
            msg += f" on {len(nets)} net(s)\n" if nets else "\n"
        else:
            msg = "No plane was created.\n\n"
        kept = zones_skipped + result.get('zones_kept', 0)
        if kept:
            # The usual reason a pour looks like it did nothing. Covers both the
            # engine keeping an existing zone and the apply step declining to
            # duplicate one already on the live board.
            msg += (f"Kept existing: {kept} zone(s) -- that net already has a "
                    f"zone on that layer, so nothing needed pouring.\n"
                    f"Uncheck 'Skip existing zones' to replace it instead.\n")
        for label, key in (("Vias placed", 'total_vias'), ("Traces added", 'total_traces')):
            if result.get(key, 0):
                msg += f"{label}: {result[key]}\n"
        if failed_pads:
            msg += f"Failed pads (no via placed): {failed_pads}\n"

        status = f"Created: {zones_added} zone(s)"
        if kept:
            status += f", {kept} kept (already present)"
        if failed_pads:
            status += f", {failed_pads} failed"
        self.status_text.SetLabel(status)

        # If any pads failed to get a via, append heuristic suggestions.
        if result.get('failed_pads', 0) > 0:
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
        # Routing movie (#506): snapshot the board this step just produced,
        # BEFORE the completion popup blocks on the user. No-op unless the
        # Advanced tab's "Make routing movie" box is ticked.
        from .movie_recorder import record_movie_step
        record_movie_step(self, 'planes')

        if getattr(getattr(self, 'GetTopLevelParent', lambda: self)(), '_suppress_completion_popups', False):
            print(msg)  # unattended plan run: no per-step OK dialog
        else:
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
        """Apply operation results to the pcbnew board.

        Delegates under a log tee: the worker's stdout redirect is restored
        before this main-thread handler runs, so without it the zone-add/
        refill/cleanup narration reached the terminal but never the log tab.
        """
        from .gui_utils import redirect_prints_to_log
        with redirect_prints_to_log(self.append_log):
            return self._apply_results_to_board_body()

    def _apply_results_to_board_body(self):
        import pcbnew

        # Reset before the early return below, so the completion message can
        # never report a PREVIOUS run's zone tally.
        self._last_zone_counts = (0, 0)
        board = pcbnew.GetBoard()
        if board is None:
            return

        self._apply_status("Applying plane copper to the board...")

        # Relocate net-less copper logos/graphics to silkscreen (issue #146),
        # matching the CLI plane writer (plane_io.py): a copper logo is not a
        # router obstacle, so the pour/repair copper added here would short
        # against it. The CLI does this unconditionally on plane output.
        from .gui_utils import move_copper_graphics_to_silkscreen_board
        gfx_moved = move_copper_graphics_to_silkscreen_board(board)
        if gfx_moved:
            print(f"Moved {gfx_moved} copper graphic(s)/logo(s) to silkscreen")

        # Delete the tracks/vias of nets that were ripped to clear a blocked pad
        # repair - their re-routed copper is in _new_segments/_new_vias below, so
        # the old (blocking) copper must go first or it would short the repair.
        # Individually-stripped input copper from the in-memory cleanup
        # (dead-end trims on plane nets that were NOT ripped): positional
        # match, same tolerance style as the route tab's removal loop.
        strips = getattr(self, '_strip_segments', None) or []
        if strips:
            import pcbnew as _p
            _keys = set()
            for s in strips:
                if hasattr(s, 'start_x'):
                    _keys.add((round(s.start_x, 3), round(s.start_y, 3),
                               round(s.end_x, 3), round(s.end_y, 3),
                               s.net_id))
                else:  # via
                    _keys.add((round(s.x, 3), round(s.y, 3), s.net_id))
            _removed = 0
            for item in list(board.GetTracks()):
                if item.Type() == _p.PCB_VIA_T:
                    k = (round(_p.ToMM(item.GetPosition().x), 3),
                         round(_p.ToMM(item.GetPosition().y), 3),
                         item.GetNetCode())
                else:
                    k = (round(_p.ToMM(item.GetStart().x), 3),
                         round(_p.ToMM(item.GetStart().y), 3),
                         round(_p.ToMM(item.GetEnd().x), 3),
                         round(_p.ToMM(item.GetEnd().y), 3),
                         item.GetNetCode())
                    if k not in _keys:
                        k = (k[2], k[3], k[0], k[1], k[4])  # reversed ends
                if k in _keys:
                    board.RemoveNative(item)
                    _removed += 1
            if _removed:
                print(f"Removed {_removed} cleanup-stripped segment(s)/via(s)")
        # One-shot: a later run on this dialog must not re-apply stale strips
        # (#508 finding 19 -- _new_vias/_new_segments were already cleared
        # after apply, _strip_segments never was).
        self._strip_segments = []

        ripped = set(getattr(self, '_ripped_net_ids', None) or [])
        if ripped:
            removed = 0
            for item in list(board.GetTracks()):  # tracks + vias
                if item.GetNetCode() in ripped:
                    board.RemoveNative(item)
                    removed += 1
            if removed:
                print(f"Removed {removed} old track/via(s) of {len(ripped)} ripped net(s)")

        # #484 H3: target/pad swaps and segment layer mods the in-memory
        # ripped-net reconnect performed can touch NON-ripped nets' copper
        # (a target swap exchanges stubs between two nets); the whole-net
        # delete above does not cover those. Apply them like the route tab
        # does, BEFORE adding new copper (the routes assume the swaps).
        _swaps = getattr(self, '_reconnect_swap_data', None) or {}
        if any(_swaps.get(k) for k in ('single_ended_target_swap_info',
                                       'all_segment_modifications',
                                       'pad_swaps')):
            try:
                from .board_swaps import apply_swaps_to_board
                apply_swaps_to_board(board, _swaps)
                print("Applied reconnect swap/modification channel(s) "
                      "to the live board (#484)")
            except Exception as _se:
                print(f"WARNING: reconnect swaps could not be applied: {_se}")
        # One-shot for the same reason as _strip_segments above: the repair
        # path never assigns this, so a create-then-repair sequence would
        # re-apply the create run's swaps (#508 finding 19's shape).
        self._reconnect_swap_data = {}

        # Get layer name to ID mapping
        name_to_id = {}
        for i in range(pcbnew.PCB_LAYER_ID_COUNT):
            name = board.GetLayerName(i)
            if name:
                name_to_id[name] = i

        def get_layer_id(layer_name):
            return name_to_id.get(layer_name, pcbnew.F_Cu)

        # Add vias from create_plane results
        vias_added = 0
        if hasattr(self, '_new_vias') and self._new_vias:
            for via_data in self._new_vias:
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(pcbnew.VECTOR2I(
                    mm_to_iu(via_data['x']),
                    mm_to_iu(via_data['y'])
                ))
                via.SetDrill(mm_to_iu(via_data['drill']))
                via.SetWidth(mm_to_iu(via_data['size']))
                via.SetNetCode(via_data['net_id'])
                # Set via layers
                layers = via_data.get('layers', ['F.Cu', 'B.Cu'])
                if len(layers) >= 2:
                    via.SetLayerPair(get_layer_id(layers[0]), get_layer_id(layers[-1]))
                board.Add(via)
                vias_added += 1
            self._new_vias = []

        # Add segments from create_plane results
        tracks_added = 0
        if hasattr(self, '_new_segments') and self._new_segments:
            for seg_data in self._new_segments:
                start = seg_data['start']
                end = seg_data['end']
                track = pcbnew.PCB_TRACK(board)
                track.SetStart(pcbnew.VECTOR2I(
                    mm_to_iu(start[0]),
                    mm_to_iu(start[1])
                ))
                track.SetEnd(pcbnew.VECTOR2I(
                    mm_to_iu(end[0]),
                    mm_to_iu(end[1])
                ))
                track.SetWidth(mm_to_iu(seg_data['width']))
                track.SetLayer(get_layer_id(seg_data['layer']))
                track.SetNetCode(seg_data['net_id'])
                board.Add(track)
                tracks_added += 1
            self._new_segments = []

        # Add zones from create_plane results
        zones_added = 0
        zones_skipped = 0
        new_zone_objs = []  # Track newly added zones so we can fill them below.
        if hasattr(self, '_new_zones') and self._new_zones:
            # Snapshot which (net_name, layer) pairs already have a zone on
            # the live board so we never add a duplicate.
            existing_zone_keys = set()
            try:
                for existing_zone in board.Zones():
                    try:
                        existing_net = existing_zone.GetNet().GetNetname()
                    except Exception:
                        existing_net = ''
                    try:
                        existing_layer = board.GetLayerName(existing_zone.GetLayer())
                    except Exception:
                        existing_layer = ''
                    existing_zone_keys.add((existing_net, existing_layer))
            except Exception:
                pass

            # Names of existing rule areas so re-runs never duplicate the
            # NPTH-slot keepouts (#448).
            existing_zone_names = set()
            try:
                for existing_zone in board.Zones():
                    try:
                        existing_zone_names.add(existing_zone.GetZoneName())
                    except Exception:
                        pass
            except Exception:
                pass

            for zone_data in self._new_zones:
                if zone_data.get('keepout'):
                    # NPTH-slot copper_pour keepout rule area (#448) -- mirrors
                    # the CLI's generate_keepout_zone_sexpr output.
                    kname = zone_data.get('name', '')
                    if kname and kname in existing_zone_names:
                        zones_skipped += 1
                        continue
                    try:
                        zone = pcbnew.ZONE(board)
                        zone.SetIsRuleArea(True)
                        zone.SetDoNotAllowCopperPour(True)
                        zone.SetDoNotAllowTracks(False)
                        zone.SetDoNotAllowVias(False)
                        try:
                            zone.SetDoNotAllowPads(False)
                            zone.SetDoNotAllowFootprints(False)
                        except AttributeError:
                            pass  # older pcbnew API names
                        if kname:
                            zone.SetZoneName(kname)
                        lset = pcbnew.LSET()
                        for lname in zone_data.get('layers', []):
                            lid = get_layer_id(lname)
                            try:
                                lset.AddLayer(lid)
                            except AttributeError:
                                lset.addLayer(lid)  # older binding name
                        zone.SetLayerSet(lset)
                        outline = zone.Outline()
                        outline.NewOutline()
                        for x, y in zone_data['polygon_points']:
                            outline.Append(mm_to_iu(x), mm_to_iu(y))
                        try:
                            zone.HatchBorder()
                        except Exception:
                            pass
                        board.Add(zone)
                        zones_added += 1
                    except Exception as e:
                        print(f"Failed to add NPTH-slot keepout {kname}: {e}")
                    continue
                key = (zone_data.get('net_name', ''), zone_data.get('layer', ''))
                if key in existing_zone_keys:
                    print(f"Skipping new zone for '{key[0]}' on {key[1]} "
                          f"(zone already exists on board)")
                    zones_skipped += 1
                    continue
                zone = pcbnew.ZONE(board)
                # Prefer looking up the net by name on the live board - net_id
                # values can drift between the parser and pcbnew (different
                # ordering for KiCad 10, or stale disk state vs. live board).
                # Fall back to SetNetCode if the name isn't found.
                net_assigned = False
                net_name = zone_data.get('net_name')
                if net_name:
                    try:
                        net_item = board.FindNet(net_name)
                    except Exception:
                        net_item = None
                    if net_item:
                        zone.SetNet(net_item)
                        net_assigned = True
                if not net_assigned:
                    zone.SetNetCode(zone_data['net_id'])
                zone.SetLayer(get_layer_id(zone_data['layer']))

                # Set zone outline from polygon points
                outline = zone.Outline()
                outline.NewOutline()
                for x, y in zone_data['polygon_points']:
                    outline.Append(mm_to_iu(x), mm_to_iu(y))

                # Fill priority: overlapping zones MUST differ, or KiCad has no
                # defined winner for the contested area and tie-breaks on KIID
                # -- the fill then moves between runs (see
                # kicad_writer.zone_overlap_priorities). The CLI writes
                # (priority N) into the s-expr; this is the GUI's equivalent.
                _zprio = int(zone_data.get('priority', 0) or 0)
                if _zprio:
                    # SetAssignedPriority is the KiCad 7+ name; older builds
                    # expose SetPriority.
                    _setter = (getattr(zone, 'SetAssignedPriority', None)
                               or getattr(zone, 'SetPriority', None))
                    if _setter:
                        _setter(_zprio)

                # Set zone properties
                zone.SetLocalClearance(mm_to_iu(zone_data.get('clearance', 0.2)))
                zone.SetMinThickness(mm_to_iu(zone_data.get('min_thickness', 0.1)))
                zone.SetPadConnection(
                    pcbnew.ZONE_CONNECTION_THERMAL
                    if zone_data.get('thermal_relief')
                    else pcbnew.ZONE_CONNECTION_FULL)
                # PARITY (review finding 4): the CLI writer stamps every zone
                # with (thermal_gap 0.2)(thermal_bridge_width 0.2)
                # (kicad_writer.generate_zone_sexpr defaults). A fresh
                # pcbnew.ZONE carries KiCad's 0.5/0.5 defaults, so with
                # thermal relief on, GUI pours got 0.5mm spokes/gaps vs the
                # CLI's 0.2 -- different fill geometry and spoke ampacity.
                # Keep these two numbers in lockstep with kicad_writer.
                try:
                    zone.SetThermalReliefGap(mm_to_iu(0.2))
                    zone.SetThermalReliefSpokeWidth(mm_to_iu(0.2))
                except Exception:
                    pass    # ancient pcbnew without the setters
                # Hatch the outline so the zone is visible immediately;
                # the actual copper fill is computed below via ZONE_FILLER.
                try:
                    zone.HatchBorder()
                except Exception:
                    pass

                board.Add(zone)
                new_zone_objs.append(zone)
                zones_added += 1
            self._new_zones = []

        if vias_added > 0 or tracks_added > 0 or zones_added > 0:
            print(f"Added to board: {zones_added} zones, {vias_added} vias, {tracks_added} tracks")

        # Hand the zone tally to the completion message. The pour itself is the
        # product here -- vias/traces are structurally 0 since the plane step
        # stopped placing taps (#492/#562), so a dialog reporting only those two
        # reads as "nothing happened" on a perfectly successful pour, and hides
        # the one number that explains a no-op: zones SKIPPED because the net
        # already has a zone on that layer (skip_existing_zones, default on).
        self._last_zone_counts = (zones_added, zones_skipped)

        # NO bare BuildConnectivity here: the taps/stitch vias just added sit
        # under the EXISTING (stale) fills, and building connectivity before
        # the refill flips their netcodes to the zones' nets (see
        # refill_all_zones's docstring). The refill below rebuilds
        # connectivity itself, after the fills have correct knockouts.

        # Teardrops, if the shared "Add teardrops" checkbox is on (#489 §9). Both
        # plane modes apply copper into pcbnew, so the writers' file-side pass
        # never runs here -- and config['add_teardrops'] was read by the create
        # path but never supplied, so the checkbox did nothing on this tab.
        _tdcfg = getattr(self, "_plane_drc_config", None) or {}
        if _tdcfg.get('add_teardrops'):
            from .gui_utils import apply_teardrops_to_board
            apply_teardrops_to_board(board)

        # Fill any newly-added zones so they appear as solid copper without
        # the user needing to run "Fill All Zones" (B) manually.
        # Re-fill EVERY zone (not just newly-created ones): a repair step adds
        # tap/reconnect copper into EXISTING plane zones, which must pull back
        # around it, and later signal steps add copper these planes must clear
        # too (#362). Filling only new_zone_objs left the existing planes stale.
        from .gui_utils import refill_all_zones
        self._apply_status("Refilling zones (KiCad exact fill)...")
        _rf = refill_all_zones(board)
        if _rf:
            print(f"Filled/refilled {_rf} zone(s)")
        elif new_zone_objs:
            print(f"Warning: could not auto-fill zones. Press B in pcbnew to fill manually.")

        # Plane-copper cleanup -- CLI/GUI PARITY. The CLI plane mains run
        # clean_plane_copper on their OUTPUT FILE after every plane step
        # (dead-end trim, stub-gap snap, graze prune); the GUI must do the same
        # or it ships plane copper the CLI strips (rp2350: ~18 redundant GND
        # segments at create alone). Runs unconditionally now: the delta is
        # computed from build_pcb_data_from_board(board) -- the SAME source as
        # the live board and validated at parser parity -- so the removal-match
        # is reliable (the earlier KICAD_GUI_PLANE_CLEANUP gate guarded against a
        # coord mismatch that no longer applies). _run_plane_copper_cleanup is
        # best-effort (try/except) and BuildConnectivity-gated, so a bad match
        # can never break an already-applied plane result.
        self._run_plane_copper_cleanup(board, get_layer_id)

        # Castellated landings (run-6 fix 1.7, GUI twin of the plane mains'
        # retract_castellated_landings): plane taps/joins that landed inside a
        # castellated pad's edge-clearance zone are pulled to its inner reach.
        from .gui_utils import apply_castellated_landing_retract
        try:
            _live_edge = (board.GetDesignSettings().m_CopperEdgeClearance
                          or 0) / 1e6
        except Exception:
            _live_edge = 0.0
        _cfg_edge = (getattr(self, '_plane_drc_config', {}) or {}).get(
            'board_edge_clearance') or 0.0
        apply_castellated_landing_retract(board, max(_cfg_edge, _live_edge))

        # Make the live board's DRC constraints consistent with the plane routing
        # floors (issue #160), mirroring route_planes.py's auto-fix. Best-effort.
        cfg = getattr(self, "_plane_drc_config", None)
        if cfg and cfg.get('fix_drc_settings', True):
            try:
                from fix_kicad_drc_settings import (compute_targets, severity_plan,
                                                    apply_targets_to_board, fab_edge_floor)
                # Grade at the smallest clearance any step actually routed at
                # (e.g. fine-pitch taps below nominal), mirroring the CLI.
                import clearance_ledger
                eff_clearance = clearance_ledger.effective(cfg.get('clearance')) \
                    if cfg.get('clearance') else cfg.get('clearance')
                targets = compute_targets(
                    clearance=eff_clearance,
                    hole_to_hole=cfg.get('hole_to_hole_clearance'),
                    edge_clearance=cfg.get('board_edge_clearance'),
                    track_width=cfg.get('track_width'),
                    via_diameter=cfg.get('via_size'),
                    via_drill=cfg.get('via_drill'),
                    fab_edge=fab_edge_floor(),
                    # #530: caps min_clearance at the smallest pad override
                    minima=board_minima_from_live(board))
                # #856: severities only on explicit request; {} = untouched.
                _sev = severity_plan() if cfg.get('relax_drc_severities') else {}
                if apply_targets_to_board(
                        board, targets, _sev,
                        clamp_nondefault_netclasses=cfg.get('clamp_netclasses', False)):
                    board.SetModified()
                    print("DRC settings: loosened Board Setup floors to the plane routing values")
            except Exception as e:
                print(f"(skipped DRC-settings write-back: {e})")

        self._apply_status("Refreshing the board view...")
        pcbnew.Refresh()

        # Sync pcb_data
        self._apply_status("Syncing board data...")
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()

        # #693: gated on the shared "Fix DRC settings after routing"
        # checkbox, like the netclass/severity writeback above -- it used to
        # run unconditionally, so an unchecked box still moved the board's
        # Board Setup floors. The CLI gates its twin on
        # --no-fix-drc-settings.
        _cfg = getattr(self, '_plane_drc_config', {}) or {}
        if _cfg.get('fix_drc_settings', True):
            from .gui_utils import update_live_drc_floors
            update_live_drc_floors(
                board,
                clearance=_cfg.get('clearance'),
                track_width=_cfg.get('track_width'),
                via_size=_cfg.get('via_size'),
                via_drill=_cfg.get('via_drill'),
                hole_to_hole=_cfg.get('hole_to_hole_clearance'))

        # (No oracle recheck here, #562: after plane CREATION the remaining
        # gaps are deliberate -- the route step's in-run plane finalize and
        # its post-apply staged-save oracle (swig_gui, shared core
        # gui_utils.run_kicad_oracle_on_live_board) own plane completion.
        # Stitching gaps at pour time is premature copper.)

        # LAST -- final refill so the pour reflects every via/track above.
        from .gui_utils import refill_all_zones
        refill_all_zones(board)

    def _run_plane_copper_cleanup(self, board, get_layer_id):
        """CLI/GUI parity: apply the shared plane-copper cleanup delta
        (pcb_modification.compute_plane_copper_cleanup -- the SAME core the CLI
        clean_plane_copper file front runs) to the LIVE board. Best-effort; a
        failure here never blocks the plane result already applied."""
        names = getattr(self, '_plane_net_names', None)
        if not names:
            return
        try:
            import pcbnew
            import routing_defaults as defaults
            from kicad_parser import build_pcb_data_from_board
            from pcb_modification import compute_plane_copper_cleanup
            cfg = getattr(self, '_plane_drc_config', {}) or {}
            clearance = cfg.get('clearance') or defaults.CLEARANCE
            grid_step = cfg.get('grid_step', defaults.GRID_STEP)
            self._apply_status("Plane cleanup: reading the live board...")
            pcb = build_pcb_data_from_board(board)
            # Runs on the UI thread, so it reports through _apply_status (which
            # forces the repaint) rather than the engine-thread callback. This
            # is the shared 13-pass pipeline -- it named none of its passes.
            delta = compute_plane_copper_cleanup(
                pcb, names, clearance, grid_step,
                progress_callback=(lambda c, t, m:
                                   self._apply_status(f"{m} ({c}/{t})" if t else m)))
            if delta.is_empty:
                return

            # Remove stripped segments/vias: match live board items by rounded
            # endpoint(s)+net, same key style as the ripped-copper strip loop.
            rm_keys = set()
            for s in delta.segments_to_remove:
                rm_keys.add((round(s.start_x, 3), round(s.start_y, 3),
                             round(s.end_x, 3), round(s.end_y, 3), s.net_id))
            via_keys = set()
            for v in delta.vias_to_remove:
                via_keys.add((round(v.x, 3), round(v.y, 3), v.net_id))
            removed = 0
            for item in list(board.GetTracks()):
                if item.Type() == pcbnew.PCB_VIA_T:
                    k = (round(pcbnew.ToMM(item.GetPosition().x), 3),
                         round(pcbnew.ToMM(item.GetPosition().y), 3),
                         item.GetNetCode())
                    if k in via_keys:
                        board.RemoveNative(item)
                        removed += 1
                    continue
                a = (round(pcbnew.ToMM(item.GetStart().x), 3),
                     round(pcbnew.ToMM(item.GetStart().y), 3),
                     round(pcbnew.ToMM(item.GetEnd().x), 3),
                     round(pcbnew.ToMM(item.GetEnd().y), 3),
                     item.GetNetCode())
                b = (a[2], a[3], a[0], a[1], a[4])  # endpoints reversed
                if a in rm_keys or b in rm_keys:
                    board.RemoveNative(item)
                    removed += 1

            # Add connector/replacement segments (snap connectors, micro-shift
            # replacements, soft-joint bridges).
            added = 0
            for s in delta.connectors:
                track = pcbnew.PCB_TRACK(board)
                track.SetStart(pcbnew.VECTOR2I(mm_to_iu(s.start_x),
                                               mm_to_iu(s.start_y)))
                track.SetEnd(pcbnew.VECTOR2I(mm_to_iu(s.end_x),
                                             mm_to_iu(s.end_y)))
                track.SetWidth(mm_to_iu(s.width))
                track.SetLayer(get_layer_id(s.layer))
                track.SetNetCode(s.net_id)
                board.Add(track)
                added += 1

            if removed or added or delta.snapped:
                # refill-first (rebuilds connectivity itself): a bare
                # BuildConnectivity over stale fills flips new copper's nets.
                from .gui_utils import refill_all_zones
                refill_all_zones(board)
                print(f"Plane cleanup: closed {delta.snapped} stub gap(s), "
                      f"trimmed {len(delta.segments_to_remove)} dead-end "
                      f"segment(s), added {added} connector(s)")
        except Exception as e:
            print(f"Warning: plane copper cleanup skipped ({e})")

    def get_assignments(self):
        """Get list of net→layer assignments."""
        return self.assignment_panel.get_assignments()
