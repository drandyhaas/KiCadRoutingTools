"""
KiCad Routing Tools - wxPython GUI for SWIG plugin

Provides a wx-based dialog for routing configuration.
"""

import os
import re
import sys
import time
import wx
import threading

# Add parent directory to path
PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

import routing_defaults as defaults
from kicad_parser import POSITION_DECIMALS
from .fanout_gui import NetSelectionPanel
from .gui_utils import StdoutRedirector
from .settings_persistence import get_dialog_settings, restore_dialog_settings


def _build_layer_mappings():
    """Build layer name <-> ID mappings using pcbnew.

    Returns:
        tuple: (name_to_id dict, id_to_name dict)
    """
    import pcbnew
    name_to_id = {'F.Cu': pcbnew.F_Cu, 'B.Cu': pcbnew.B_Cu}
    id_to_name = {pcbnew.F_Cu: 'F.Cu', pcbnew.B_Cu: 'B.Cu'}
    for i in range(1, 31):
        layer_id = getattr(pcbnew, f'In{i}_Cu', None)
        if layer_id is not None:
            name_to_id[f'In{i}.Cu'] = layer_id
            id_to_name[layer_id] = f'In{i}.Cu'
    return name_to_id, id_to_name


def _get_netclass_parameters(class_name):
    """Get routing parameters for a specific net class from pcbnew.

    Args:
        class_name: Name of the net class (e.g., 'Default', 'Wide')

    Returns:
        dict with keys: track_width, clearance, via_size, via_drill,
        diff_pair_width, diff_pair_gap (all in mm)
        Returns None if net class not found or error occurs.
    """
    try:
        import pcbnew
        board = pcbnew.GetBoard()
        if board is None:
            return None

        ds = board.GetDesignSettings()
        net_settings = ds.m_NetSettings

        # Get the net class by name
        netclass = net_settings.GetNetClassByName(class_name)
        if not netclass:
            # Try getting default
            netclass = net_settings.GetDefaultNetclass()
        if not netclass:
            return None

        # KiCad stores values in nanometers, convert to mm
        nm_to_mm = 1e-6

        result = {
            'track_width': netclass.GetTrackWidth() * nm_to_mm,
            'clearance': netclass.GetClearance() * nm_to_mm,
            'via_size': netclass.GetViaDiameter() * nm_to_mm,
            'via_drill': netclass.GetViaDrill() * nm_to_mm,
        }

        # Add differential pair parameters if available
        if hasattr(netclass, 'GetDiffPairWidth'):
            result['diff_pair_width'] = netclass.GetDiffPairWidth() * nm_to_mm
        if hasattr(netclass, 'GetDiffPairGap'):
            result['diff_pair_gap'] = netclass.GetDiffPairGap() * nm_to_mm

        return result
    except Exception:
        return None


def _get_board_minimum_constraints():
    """Get board-level minimum constraints from pcbnew.

    These are the hard floors from Board Setup → Design Rules → Constraints.

    Returns:
        dict with min_track_width, min_clearance, min_via_size, min_via_drill,
        min_hole_to_hole, min_copper_edge_clearance (in mm)
        or None if board not available
    """
    try:
        import pcbnew
        board = pcbnew.GetBoard()
        if board is None:
            return None

        ds = board.GetDesignSettings()
        nm_to_mm = 1e-6

        result = {
            'min_track_width': ds.m_TrackMinWidth * nm_to_mm,
            'min_clearance': ds.m_MinClearance * nm_to_mm,
            'min_via_size': ds.m_ViasMinSize * nm_to_mm,
            'min_via_drill': ds.m_MinThroughDrill * nm_to_mm,
        }

        # Try to get hole-to-hole clearance (may not be available in all versions)
        if hasattr(ds, 'm_HoleToHoleMin'):
            result['min_hole_to_hole'] = ds.m_HoleToHoleMin * nm_to_mm

        # Try to get copper-to-edge clearance
        if hasattr(ds, 'm_CopperEdgeClearance'):
            result['min_copper_edge_clearance'] = ds.m_CopperEdgeClearance * nm_to_mm

        return result
    except Exception:
        return None


class RoutingDialog(wx.Dialog):
    """Main dialog for configuring and running the router."""

    def __init__(self, parent, pcb_data, board_filename, saved_settings=None):
        super().__init__(
            parent,
            title="KiCad Routing Tools",
            size=(800, 800),
            style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER
        )

        # Get saved transparency or use default
        self._initial_transparency = 240
        if saved_settings and 'window_transparency' in saved_settings:
            self._initial_transparency = saved_settings['window_transparency']

        # Set window transparency (0=fully transparent, 255=opaque)
        self.SetTransparent(self._initial_transparency)

        # Configure tooltip timing (in milliseconds)
        wx.ToolTip.SetDelay(250)       # Delay before showing
        wx.ToolTip.SetAutoPop(10000)   # How long tooltip stays visible
        wx.ToolTip.SetReshow(50)       # Delay when moving between controls

        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self._cancel_requested = False
        self._routing_thread = None
        self._connectivity_cache = {}  # Cache: net_id -> is_connected
        self._saved_settings = saved_settings  # Settings to restore after init
        self._last_notebook = None  # Track netclass notebook for cleanup
        self._initial_load = True  # Skip board sync on first refresh (data is already current)

        self._create_ui()
        self._load_nets_immediate()  # Load net names only (fast)
        self.Centre()

        # Defer sync and connectivity check until after dialog is shown
        wx.CallAfter(self._deferred_init)

    def _sync_pcb_data_from_board(self):
        """Sync pcb_data.segments and pcb_data.vias from pcbnew's in-memory board.

        This is necessary because pcb_data is parsed from the file on disk,
        but tracks added during previous routing sessions only exist in pcbnew's
        memory until the file is saved.
        """
        # Clear connectivity cache since board state is changing
        self._connectivity_cache = {}

        try:
            import pcbnew
            from kicad_parser import Segment, Via

            board = pcbnew.GetBoard()
            if board is None:
                return
        except Exception as e:
            print(f"Warning: Could not sync from board: {e}")
            return

        try:
            # Get layer mappings
            _, id_to_name = _build_layer_mappings()

            def get_layer_name(layer_id):
                return id_to_name.get(layer_id, 'F.Cu')

            # Collect all segments from board
            new_segments = []
            for track in board.GetTracks():
                if track.GetClass() == "PCB_TRACK":
                    seg = Segment(
                        start_x=pcbnew.ToMM(track.GetStart().x),
                        start_y=pcbnew.ToMM(track.GetStart().y),
                        end_x=pcbnew.ToMM(track.GetEnd().x),
                        end_y=pcbnew.ToMM(track.GetEnd().y),
                        width=pcbnew.ToMM(track.GetWidth()),
                        layer=get_layer_name(track.GetLayer()),
                        net_id=track.GetNetCode(),
                    )
                    new_segments.append(seg)

            # Collect all vias from board
            new_vias = []
            for track in board.GetTracks():
                if track.GetClass() == "PCB_VIA":
                    via = track
                    top_layer = via.TopLayer()
                    bot_layer = via.BottomLayer()
                    v = Via(
                        x=pcbnew.ToMM(via.GetPosition().x),
                        y=pcbnew.ToMM(via.GetPosition().y),
                        size=pcbnew.ToMM(via.GetWidth()),
                        drill=pcbnew.ToMM(via.GetDrill()),
                        layers=[get_layer_name(top_layer), get_layer_name(bot_layer)],
                        net_id=via.GetNetCode(),
                    )
                    new_vias.append(v)

            # Replace pcb_data segments and vias with what's in pcbnew
            self.pcb_data.segments = new_segments
            self.pcb_data.vias = new_vias
        except Exception as e:
            print(f"Warning: Error syncing tracks from board: {e}")

    def _create_ui(self):
        """Create the dialog UI with tabs for Configure, Advanced, and Log."""
        main_panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Create notebook for tabs
        self.notebook = wx.Notebook(main_panel)

        # Tab 1: Basic parameters
        config_panel = self._create_config_tab()
        self.notebook.AddPage(config_panel, "Basic")

        # Tab 2: Advanced (swappable nets + advanced parameters + options)
        advanced_panel = self._create_advanced_tab()
        self.notebook.AddPage(advanced_panel, "Advanced")

        # Tab 3: Differential
        differential_panel = self._create_differential_tab()
        self.notebook.AddPage(differential_panel, "Differential")

        # Tab 4: Fanout
        self.fanout_tab = self._create_fanout_tab()
        self.notebook.AddPage(self.fanout_tab, "Fanout")

        # Tab 5: Planes
        self.planes_tab = self._create_planes_tab()
        self.notebook.AddPage(self.planes_tab, "Planes")

        # Tab 6: Log
        log_panel = self._create_log_tab()
        self.notebook.AddPage(log_panel, "Log")

        # Tab 7: About
        self.about_tab = self._create_about_tab()
        self.notebook.AddPage(self.about_tab, "About")

        # Add notebook to main sizer
        main_sizer.Add(self.notebook, 1, wx.EXPAND | wx.ALL, 5)

        # Bind tab change to validate settings when switching tabs
        self.notebook.Bind(wx.EVT_NOTEBOOK_PAGE_CHANGED, self._on_main_tab_changed)

        # Status bar at bottom
        self.status_bar = wx.StaticText(main_panel, label="")
        main_sizer.Add(self.status_bar, 0, wx.EXPAND | wx.ALL, 5)

        main_panel.SetSizer(main_sizer)

    def _create_about_tab(self):
        """Create the About tab."""
        from .about_tab import AboutTab
        return AboutTab(
            self.notebook,
            on_reset_settings=self._reset_all_settings,
            on_transparency_changed=self._on_transparency_changed,
            initial_transparency=self._initial_transparency,
            on_validate_pcb_data=self._validate_pcb_data
        )

    def _validate_pcb_data(self):
        """Compare pcbnew-extracted PCBData against file-parsed PCBData."""
        from kicad_parser import parse_kicad_pcb, compare_pcb_data

        if not self.board_filename:
            self._append_log("Validation: Board has no filename (not saved yet). "
                             "Save the board first to enable file-based validation.\n")
            return

        self._append_log("=== PCB Data Validation ===\n")
        self._append_log(f"Comparing pcbnew data vs file parse of: {self.board_filename}\n")

        try:
            file_data = parse_kicad_pcb(self.board_filename)
        except Exception as e:
            self._append_log(f"Error parsing file: {e}\n")
            return

        diffs = compare_pcb_data(self.pcb_data, file_data)

        if not diffs:
            self._append_log("PASS: No differences found! pcbnew data matches file parse.\n")
        else:
            self._append_log(f"Found {len(diffs)} difference(s):\n")
            for diff in diffs:
                self._append_log(f"  - {diff}\n")

        self._append_log("=== Validation Complete ===\n\n")

        # Switch to log tab to show results
        for i in range(self.notebook.GetPageCount()):
            if self.notebook.GetPageText(i) == "Log":
                self.notebook.SetSelection(i)
                break

    def _on_transparency_changed(self, value):
        """Handle transparency slider change from About tab."""
        self.SetTransparent(value)

    def _create_config_tab(self):
        """Create the Basic tab with basic routing parameters and options."""
        panel = wx.Panel(self.notebook)
        config_sizer = wx.BoxSizer(wx.VERTICAL)
        h_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # Left side: Net selection (1:1 ratio with right side)
        net_box = wx.StaticBox(panel, label="Net Selection")
        net_sizer = wx.StaticBoxSizer(net_box, wx.VERTICAL)

        self.net_panel = NetSelectionPanel(
            panel, self.pcb_data,
            instructions="Select nets to route...",
            hide_label="Hide connected",
            hide_tooltip="Hide nets that are already fully connected",
            show_hide_checkbox=True,
            show_component_filter=True,
            show_component_dropdown=True,
            min_pads_for_dropdown=3,
            show_hide_differential=True,
            hide_differential_default=False
        )
        self.net_panel.set_selection_changed_callback(self._update_status_bar)
        self.net_panel.set_tabbed_view_changed_callback(self._on_tabbed_view_changed)
        net_sizer.Add(self.net_panel, 1, wx.EXPAND)

        h_sizer.Add(net_sizer, 1, wx.EXPAND | wx.ALL, 5)

        # Right side: Basic parameters, layers, options, progress, buttons (2:1:2 ratio)
        right_sizer = wx.BoxSizer(wx.VERTICAL)
        right_sizer.Add(self._create_parameters_panel(panel), 2, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self._create_layers_panel(panel), 1, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self._create_basic_options_panel(panel), 2, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self._create_progress_panel(panel), 0, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self._create_buttons_panel(panel), 0, wx.EXPAND)
        h_sizer.Add(right_sizer, 1, wx.EXPAND | wx.ALL, 5)

        config_sizer.Add(h_sizer, 1, wx.EXPAND | wx.ALL, 5)
        panel.SetSizer(config_sizer)
        return panel

    def _create_parameters_panel(self, panel):
        """Create the parameters panel with basic settings only."""
        param_box = wx.StaticBox(panel, label="Parameters")
        param_box_sizer = wx.StaticBoxSizer(param_box, wx.VERTICAL)

        # Use net class definitions checkbox
        self.use_netclass_check = wx.CheckBox(panel, label="Use net class definitions")
        self.use_netclass_check.SetValue(False)
        self.use_netclass_check.SetToolTip("Use track width, clearance, via size from selected net class")
        self.use_netclass_check.Bind(wx.EVT_CHECKBOX, self._on_use_netclass_changed)
        param_box_sizer.Add(self.use_netclass_check, 0, wx.ALL, 5)

        # Obey design rule constraints checkbox
        self.obey_drc_check = wx.CheckBox(panel, label="Obey design rule constraints")
        self.obey_drc_check.SetValue(True)
        self.obey_drc_check.SetToolTip(
            "Enforce KiCad's board-level minimum constraints from Board Setup → Design Rules"
        )
        self.obey_drc_check.Bind(wx.EVT_CHECKBOX, self._on_obey_drc_changed)
        param_box_sizer.Add(self.obey_drc_check, 0, wx.ALL, 5)

        param_scroll = wx.ScrolledWindow(panel, style=wx.VSCROLL)
        param_scroll.SetScrollRate(0, 10)
        param_inner = wx.BoxSizer(wx.VERTICAL)

        # Basic parameters grid
        param_grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        param_grid.AddGrowableCol(1)
        self._add_basic_parameters(param_scroll, param_grid)
        param_inner.Add(param_grid, 0, wx.EXPAND | wx.ALL, 5)

        param_scroll.SetSizer(param_inner)
        param_box_sizer.Add(param_scroll, 1, wx.EXPAND)
        return param_box_sizer

    def _add_basic_parameters(self, parent, grid):
        """Add basic parameter controls to grid."""
        # Map control names to DRC minimum keys
        self._drc_min_keys = {
            'track_width': 'min_track_width',
            'clearance': 'min_clearance',
            'via_size': 'min_via_size',
            'via_drill': 'min_via_drill',
            'hole_to_hole_clearance': 'min_hole_to_hole',
            'board_edge_clearance': 'min_copper_edge_clearance',
        }
        params = [
            ('track_width', 'Track Width (mm):', defaults.TRACK_WIDTH, "Width of routed traces"),
            ('clearance', 'Clearance (mm):', defaults.CLEARANCE, "Minimum spacing between traces and other copper"),
            ('via_size', 'Via Size (mm):', defaults.VIA_SIZE, "Outer diameter of vias"),
            ('via_drill', 'Via Drill (mm):', defaults.VIA_DRILL, "Drill hole diameter for vias"),
            ('hole_to_hole_clearance', 'Hole Clearance (mm):', defaults.HOLE_TO_HOLE_CLEARANCE, "Minimum spacing between via/pad drill holes"),
        ]
        for name, label, default, tooltip in params:
            r = defaults.PARAM_RANGES[name]
            grid.Add(wx.StaticText(parent, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            ctrl = wx.SpinCtrlDouble(parent, min=r['min'], max=r['max'], initial=default, inc=r['inc'])
            ctrl.SetDigits(r['digits'])
            ctrl.SetToolTip(tooltip)
            ctrl.Bind(wx.EVT_SPINCTRLDOUBLE, lambda evt, n=name: self._on_drc_param_changed(evt, n))
            setattr(self, name, ctrl)
            grid.Add(ctrl, 0, wx.EXPAND)

        # Edge clearance (checkbox + value)
        grid.Add(wx.StaticText(parent, label="Edge Clearance (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        edge_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.edge_clearance_check = wx.CheckBox(parent, label="")
        self.edge_clearance_check.SetValue(False)
        self.edge_clearance_check.SetToolTip("Enable custom edge clearance (unchecked = use track clearance)")
        self.edge_clearance_check.Bind(wx.EVT_CHECKBOX, self._on_edge_clearance_check)
        r = defaults.PARAM_RANGES['board_edge_clearance']
        self.board_edge_clearance = wx.SpinCtrlDouble(parent, min=r['min'], max=r['max'], initial=defaults.CLEARANCE, inc=r['inc'])
        self.board_edge_clearance.SetDigits(r['digits'])
        self.board_edge_clearance.Bind(wx.EVT_SPINCTRLDOUBLE, lambda evt: self._on_drc_param_changed(evt, 'board_edge_clearance'))
        self.board_edge_clearance.SetToolTip("When disabled, tracks use the Clearance value for board edge spacing")
        self.board_edge_clearance.Enable(False)
        edge_sizer.Add(self.edge_clearance_check, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        edge_sizer.Add(self.board_edge_clearance, 1, wx.EXPAND)
        grid.Add(edge_sizer, 0, wx.EXPAND)

        # Grid step
        r = defaults.PARAM_RANGES['grid_step']
        grid.Add(wx.StaticText(parent, label="Grid Step (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.grid_step = wx.SpinCtrlDouble(parent, min=r['min'], max=r['max'], initial=defaults.GRID_STEP, inc=r['inc'])
        self.grid_step.SetDigits(r['digits'])
        self.grid_step.SetToolTip("Routing grid resolution (smaller = finer routing, slower)")
        grid.Add(self.grid_step, 0, wx.EXPAND)

        # Via cost (integer)
        r = defaults.PARAM_RANGES['via_cost']
        grid.Add(wx.StaticText(parent, label="Via Cost:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.via_cost = wx.SpinCtrl(parent, min=r['min'], max=r['max'], initial=defaults.VIA_COST)
        self.via_cost.SetToolTip("Cost penalty for adding vias (higher = fewer layer changes)")
        grid.Add(self.via_cost, 0, wx.EXPAND)

        # Max rip-up count
        r = defaults.PARAM_RANGES['max_ripup']
        grid.Add(wx.StaticText(parent, label="Max Rip-up:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.max_ripup = wx.SpinCtrl(parent, min=r['min'], max=r['max'], initial=defaults.MAX_RIPUP)
        self.max_ripup.SetToolTip("Maximum number of nets to rip up and reroute when blocked")
        grid.Add(self.max_ripup, 0, wx.EXPAND)

    def _on_obey_drc_changed(self, event):
        """Handle checkbox toggle - apply board minimums if enabled."""
        if self.obey_drc_check.GetValue():
            self._apply_board_minimums_to_controls()

    def _on_drc_param_changed(self, event, ctrl_name):
        """Validate parameter change against DRC minimums."""
        if not (hasattr(self, 'obey_drc_check') and self.obey_drc_check.GetValue()):
            event.Skip()
            return

        minimums = _get_board_minimum_constraints()
        if minimums is None:
            event.Skip()
            return

        min_key = self._drc_min_keys.get(ctrl_name)
        if not min_key or min_key not in minimums:
            event.Skip()
            return

        ctrl = getattr(self, ctrl_name, None)
        if ctrl is None:
            event.Skip()
            return

        minimum = minimums[min_key]
        current = ctrl.GetValue()

        if current < minimum:
            # Show warning and revert to minimum
            label = ctrl_name.replace('_', ' ').title()
            wx.MessageBox(
                f"{label} cannot be less than {minimum:.3f} mm\n"
                f"(Board minimum from Design Rules)",
                "Design Rule Constraint",
                wx.OK | wx.ICON_WARNING
            )
            ctrl.SetValue(minimum)
        else:
            event.Skip()

    def _apply_board_minimums_to_controls(self):
        """Apply board-level minimum constraints to control values.

        Called when dialog opens, before values are displayed to user.
        Silently adjusts values to meet board minimums.
        """
        if not (hasattr(self, 'obey_drc_check') and self.obey_drc_check.GetValue()):
            return

        minimums = _get_board_minimum_constraints()
        if minimums is None:
            return

        # Map control names to minimum keys
        checks = [
            ('track_width', 'min_track_width'),
            ('clearance', 'min_clearance'),
            ('via_size', 'min_via_size'),
            ('via_drill', 'min_via_drill'),
            ('hole_to_hole_clearance', 'min_hole_to_hole'),
            ('board_edge_clearance', 'min_copper_edge_clearance'),
        ]

        for ctrl_name, min_key in checks:
            ctrl = getattr(self, ctrl_name, None)
            if ctrl and minimums.get(min_key):
                current = ctrl.GetValue()
                minimum = minimums[min_key]
                if current < minimum:
                    ctrl.SetValue(minimum)

    def _add_advanced_parameters(self, parent, grid):
        """Add advanced parameter controls to grid."""
        # Impedance routing (checkbox + value)
        grid.Add(wx.StaticText(parent, label="Impedance:"), 0, wx.ALIGN_CENTER_VERTICAL)
        impedance_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.impedance_check = wx.CheckBox(parent, label="")
        self.impedance_check.SetValue(False)
        self.impedance_check.SetToolTip("Use impedance-based track width (overrides Track Width)")
        r = defaults.PARAM_RANGES['impedance']
        self.impedance_value = wx.SpinCtrl(parent, min=r['min'], max=r['max'], initial=defaults.IMPEDANCE_DEFAULT)
        self.impedance_value.SetToolTip("Target impedance in ohms")
        impedance_sizer.Add(self.impedance_check, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        impedance_sizer.Add(self.impedance_value, 1, wx.EXPAND)
        impedance_sizer.Add(wx.StaticText(parent, label="\u03A9"), 0, wx.ALIGN_CENTER_VERTICAL | wx.LEFT, 3)
        grid.Add(impedance_sizer, 0, wx.EXPAND)

        # Integer parameters
        int_params = [
            ('max_iterations', 'Max Iterations:', defaults.MAX_ITERATIONS, "Maximum A* iterations per net before giving up"),
            ('max_probe_iterations', 'Probe Iterations:', defaults.MAX_PROBE_ITERATIONS, "Iterations for quick probe routing attempts"),
            ('turn_cost', 'Turn Cost:', defaults.TURN_COST, "Penalty for 90-degree turns (encourages straighter routes)"),
            ('direction_preference_cost', 'Dir. Pref. Cost:', defaults.DIRECTION_PREFERENCE_COST, "Penalty for routing against layer's preferred direction"),
        ]
        for name, label, default, tooltip in int_params:
            r = defaults.PARAM_RANGES[name]
            grid.Add(wx.StaticText(parent, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            ctrl = wx.SpinCtrl(parent, min=r['min'], max=r['max'], initial=default)
            ctrl.SetToolTip(tooltip)
            setattr(self, name, ctrl)
            grid.Add(ctrl, 0, wx.EXPAND)

        # Heuristic weight
        r = defaults.PARAM_RANGES['heuristic_weight']
        grid.Add(wx.StaticText(parent, label="Heuristic Weight:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.heuristic_weight = wx.SpinCtrlDouble(parent, min=r['min'], max=r['max'], initial=defaults.HEURISTIC_WEIGHT, inc=r['inc'])
        self.heuristic_weight.SetDigits(r['digits'])
        self.heuristic_weight.SetToolTip("A* heuristic weight (higher = faster but less optimal routes)")
        grid.Add(self.heuristic_weight, 0, wx.EXPAND)

        # Proximity heuristic factor
        r = defaults.PARAM_RANGES['proximity_heuristic_factor']
        grid.Add(wx.StaticText(parent, label="Prox. Heuristic Factor:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.proximity_heuristic_factor = wx.SpinCtrlDouble(parent, min=r['min'], max=r['max'], initial=defaults.PROXIMITY_HEURISTIC_FACTOR, inc=r['inc'])
        self.proximity_heuristic_factor.SetDigits(r['digits'])
        self.proximity_heuristic_factor.SetToolTip("Factor for proximity-aware A* heuristic (0 = disabled)")
        grid.Add(self.proximity_heuristic_factor, 0, wx.EXPAND)

        # Float parameters
        float_params = [
            ('bga_proximity_radius', 'BGA Proximity (mm):', defaults.BGA_PROXIMITY_RADIUS, "Radius around BGA pads to apply extra cost"),
            ('bga_proximity_cost', 'BGA Prox. Cost:', defaults.BGA_PROXIMITY_COST, "Cost multiplier for routing near BGA pads"),
            ('stub_proximity_radius', 'Stub Proximity (mm):', defaults.STUB_PROXIMITY_RADIUS, "Radius around stubs to apply extra cost"),
            ('stub_proximity_cost', 'Stub Prox. Cost:', defaults.STUB_PROXIMITY_COST, "Cost for routing near stubs of other nets"),
            ('via_proximity_cost', 'Via Prox. Multiplier:', defaults.VIA_PROXIMITY_COST, "Cost multiplier for placing vias near other vias"),
            ('track_proximity_distance', 'Track Prox. (mm):', defaults.TRACK_PROXIMITY_DISTANCE, "Distance to detect parallel tracks for bunching avoidance"),
            ('track_proximity_cost', 'Track Prox. Cost:', defaults.TRACK_PROXIMITY_COST, "Cost for routing parallel to existing tracks"),
            ('vertical_attraction_radius', 'Vert. Attract (mm):', defaults.VERTICAL_ATTRACTION_RADIUS, "Radius for attracting route toward target vertically"),
            ('vertical_attraction_cost', 'Vert. Attract Cost:', defaults.VERTICAL_ATTRACTION_COST, "Bonus for moving toward target's vertical position"),
            ('ripped_route_avoidance_radius', 'Rip Avoid (mm):', defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS, "Radius to avoid area where previous route failed"),
            ('ripped_route_avoidance_cost', 'Rip Avoid Cost:', defaults.RIPPED_ROUTE_AVOIDANCE_COST, "Cost for routing through previously ripped area"),
            ('routing_clearance_margin', 'Clearance Margin:', defaults.ROUTING_CLEARANCE_MARGIN, "Extra clearance margin multiplier for safety"),
        ]
        for name, label, default, tooltip in float_params:
            r = defaults.PARAM_RANGES[name]
            grid.Add(wx.StaticText(parent, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            ctrl = wx.SpinCtrlDouble(parent, min=r['min'], max=r['max'], initial=default, inc=r['inc'])
            ctrl.SetDigits(r['digits'])
            ctrl.SetToolTip(tooltip)
            setattr(self, name, ctrl)
            grid.Add(ctrl, 0, wx.EXPAND)

        # Ordering strategy
        grid.Add(wx.StaticText(parent, label="Ordering Strategy:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.ordering_strategy = wx.Choice(parent, choices=["mps", "inside_out", "original"])
        self.ordering_strategy.SetSelection(0)
        self.ordering_strategy.SetToolTip("Net ordering strategy: mps (minimum planar subset), inside_out, or original order")
        grid.Add(self.ordering_strategy, 0, wx.EXPAND)

        # Direction dropdown
        grid.Add(wx.StaticText(parent, label="Routing Direction:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.direction_choice = wx.Choice(parent, choices=["Auto", "Forward", "Backward"])
        self.direction_choice.SetSelection(0)
        self.direction_choice.SetToolTip("Direction search order for each net route")
        grid.Add(self.direction_choice, 0, wx.EXPAND)

    def _create_layers_panel(self, panel):
        """Create the layers selection panel."""
        layer_box = wx.StaticBox(panel, label="Layers")
        layer_box_sizer = wx.StaticBoxSizer(layer_box, wx.VERTICAL)
        layer_scroll = wx.ScrolledWindow(panel, style=wx.VSCROLL)
        layer_scroll.SetScrollRate(0, 10)
        layer_inner = wx.WrapSizer(wx.HORIZONTAL)

        self.layer_checks = {}
        for layer in self.pcb_data.board_info.copper_layers:
            cb = wx.CheckBox(layer_scroll, label=layer)
            # Default to all copper layers defined in the PCB
            cb.SetValue(True)
            cb.SetToolTip(f"Include {layer} for routing")
            self.layer_checks[layer] = cb
            layer_inner.Add(cb, 0, wx.ALL, 3)

        layer_scroll.SetSizer(layer_inner)
        layer_box_sizer.Add(layer_scroll, 1, wx.EXPAND)
        return layer_box_sizer

    def _create_basic_options_panel(self, panel):
        """Create the basic options panel for the Basic tab."""
        options_box = wx.StaticBox(panel, label="Options")
        options_box_sizer = wx.StaticBoxSizer(options_box, wx.VERTICAL)
        options_scroll = wx.ScrolledWindow(panel, style=wx.VSCROLL)
        options_scroll.SetScrollRate(0, 10)
        options_inner = wx.BoxSizer(wx.VERTICAL)

        # Stub layer swaps
        self.enable_layer_switch = wx.CheckBox(options_scroll, label="Stub layer swaps")
        self.enable_layer_switch.SetValue(True)
        self.enable_layer_switch.SetToolTip("Enable stub layer switching optimization")
        options_inner.Add(self.enable_layer_switch, 0, wx.ALL, 3)

        # Move copper text
        self.move_text_check = wx.CheckBox(options_scroll, label="Move copper text to silkscreen")
        self.move_text_check.SetValue(True)
        self.move_text_check.SetToolTip("Move gr_text from copper layers to silkscreen to prevent routing interference")
        options_inner.Add(self.move_text_check, 0, wx.ALL, 3)

        # Add teardrops
        self.add_teardrops_check = wx.CheckBox(options_scroll, label="Add teardrops")
        self.add_teardrops_check.SetValue(False)
        self.add_teardrops_check.SetToolTip("Add teardrop settings to all pads in output file")
        options_inner.Add(self.add_teardrops_check, 0, wx.ALL, 3)

        # Power nets
        power_sizer = wx.BoxSizer(wx.HORIZONTAL)
        power_sizer.Add(wx.StaticText(options_scroll, label="Power Nets:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.power_nets_ctrl = wx.TextCtrl(options_scroll)
        self.power_nets_ctrl.SetToolTip("Glob patterns for power nets (e.g., *GND* *VCC*)")
        power_sizer.Add(self.power_nets_ctrl, 1, wx.EXPAND)
        options_inner.Add(power_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Power net widths
        widths_sizer = wx.BoxSizer(wx.HORIZONTAL)
        widths_sizer.Add(wx.StaticText(options_scroll, label="Power Widths:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.power_widths_ctrl = wx.TextCtrl(options_scroll)
        self.power_widths_ctrl.SetToolTip("Track widths in mm for each power-net pattern (space-separated)")
        widths_sizer.Add(self.power_widths_ctrl, 1, wx.EXPAND)
        options_inner.Add(widths_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # No BGA zones
        bga_sizer = wx.BoxSizer(wx.HORIZONTAL)
        bga_sizer.Add(wx.StaticText(options_scroll, label="No BGA Zones:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.no_bga_zones_ctrl = wx.TextCtrl(options_scroll, value="ALL")
        self.no_bga_zones_ctrl.SetToolTip("Disable BGA exclusion zones: component refs (e.g., U1 U3), ALL, or leave empty to exclude none")
        bga_sizer.Add(self.no_bga_zones_ctrl, 1, wx.EXPAND)
        options_inner.Add(bga_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Layer costs
        layer_sizer = wx.BoxSizer(wx.HORIZONTAL)
        layer_sizer.Add(wx.StaticText(options_scroll, label="Layer Costs:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.layer_costs_ctrl = wx.TextCtrl(options_scroll)
        # Generate default layer costs: 1.0 for F.Cu, 3.0 for others
        default_costs = []
        for layer in self.pcb_data.board_info.copper_layers:
            default_costs.append("1.0" if layer == "F.Cu" else "3.0")
        self.layer_costs_ctrl.SetValue(" ".join(default_costs))
        self.layer_costs_ctrl.SetToolTip("Per-layer cost multipliers (order: " + " ".join(self.pcb_data.board_info.copper_layers) + ")")
        layer_sizer.Add(self.layer_costs_ctrl, 1, wx.EXPAND)
        options_inner.Add(layer_sizer, 0, wx.EXPAND | wx.ALL, 3)

        options_scroll.SetSizer(options_inner)
        options_box_sizer.Add(options_scroll, 1, wx.EXPAND)
        return options_box_sizer

    def _create_options_panel(self, panel):
        """Create the advanced options panel (MPS, crossing, length matching, debug)."""
        options_box = wx.StaticBox(panel, label="Options")
        options_box_sizer = wx.StaticBoxSizer(options_box, wx.VERTICAL)
        options_scroll = wx.ScrolledWindow(panel, style=wx.VSCROLL)
        options_scroll.SetScrollRate(0, 10)
        options_inner = wx.BoxSizer(wx.VERTICAL)

        # MPS options
        mps_label = wx.StaticText(options_scroll, label="MPS Options:")
        mps_label.SetFont(mps_label.GetFont().Bold())
        options_inner.Add(mps_label, 0, wx.LEFT | wx.TOP, 3)

        self.mps_reverse_rounds = wx.CheckBox(options_scroll, label="Reverse MPS rounds")
        self.mps_reverse_rounds.SetToolTip("Route most-conflicting groups first instead of least-conflicting")
        options_inner.Add(self.mps_reverse_rounds, 0, wx.ALL, 3)

        self.mps_layer_swap = wx.CheckBox(options_scroll, label="MPS layer swap")
        self.mps_layer_swap.SetToolTip("Enable MPS-aware layer swaps to reduce crossing conflicts")
        options_inner.Add(self.mps_layer_swap, 0, wx.ALL, 3)

        self.mps_segment_intersection = wx.CheckBox(options_scroll, label="MPS segment intersection")
        self.mps_segment_intersection.SetToolTip("Force MPS to use segment intersection for crossing detection")
        options_inner.Add(self.mps_segment_intersection, 0, wx.ALL, 3)

        options_inner.AddSpacer(10)

        # Bus routing options
        bus_label = wx.StaticText(options_scroll, label="Bus Routing:")
        bus_label.SetFont(bus_label.GetFont().Bold())
        options_inner.Add(bus_label, 0, wx.LEFT | wx.TOP, 3)

        self.bus_enabled = wx.CheckBox(options_scroll, label="Enable bus routing")
        self.bus_enabled.SetToolTip("Auto-detect and route parallel groups of nets together")
        options_inner.Add(self.bus_enabled, 0, wx.ALL, 3)

        bus_detect_sizer = wx.BoxSizer(wx.HORIZONTAL)
        bus_detect_sizer.Add(wx.StaticText(options_scroll, label="Detection radius:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        r = defaults.PARAM_RANGES['bus_detection_radius']
        self.bus_detection_radius = wx.SpinCtrlDouble(options_scroll, min=r['min'], max=r['max'],
                                                       initial=defaults.BUS_DETECTION_RADIUS, inc=r['inc'])
        self.bus_detection_radius.SetDigits(r['digits'])
        self.bus_detection_radius.SetToolTip("Max endpoint distance to form bus group (mm)")
        bus_detect_sizer.Add(self.bus_detection_radius, 1, wx.EXPAND)
        options_inner.Add(bus_detect_sizer, 0, wx.EXPAND | wx.ALL, 3)

        bus_attract_sizer = wx.BoxSizer(wx.HORIZONTAL)
        bus_attract_sizer.Add(wx.StaticText(options_scroll, label="Attraction radius:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        r = defaults.PARAM_RANGES['bus_attraction_radius']
        self.bus_attraction_radius = wx.SpinCtrlDouble(options_scroll, min=r['min'], max=r['max'],
                                                        initial=defaults.BUS_ATTRACTION_RADIUS, inc=r['inc'])
        self.bus_attraction_radius.SetDigits(r['digits'])
        self.bus_attraction_radius.SetToolTip("Attraction radius from neighbor track (mm)")
        bus_attract_sizer.Add(self.bus_attraction_radius, 1, wx.EXPAND)
        options_inner.Add(bus_attract_sizer, 0, wx.EXPAND | wx.ALL, 3)

        bus_bonus_sizer = wx.BoxSizer(wx.HORIZONTAL)
        bus_bonus_sizer.Add(wx.StaticText(options_scroll, label="Attraction bonus:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        r = defaults.PARAM_RANGES['bus_attraction_bonus']
        self.bus_attraction_bonus = wx.SpinCtrl(options_scroll, min=r['min'], max=r['max'],
                                                 initial=defaults.BUS_ATTRACTION_BONUS)
        self.bus_attraction_bonus.SetToolTip("Cost bonus for staying parallel to neighbor track")
        bus_bonus_sizer.Add(self.bus_attraction_bonus, 1, wx.EXPAND)
        options_inner.Add(bus_bonus_sizer, 0, wx.EXPAND | wx.ALL, 3)

        bus_min_sizer = wx.BoxSizer(wx.HORIZONTAL)
        bus_min_sizer.Add(wx.StaticText(options_scroll, label="Min nets:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        r = defaults.PARAM_RANGES['bus_min_nets']
        self.bus_min_nets = wx.SpinCtrl(options_scroll, min=r['min'], max=r['max'],
                                         initial=defaults.BUS_MIN_NETS)
        self.bus_min_nets.SetToolTip("Minimum number of nets to form a bus group")
        bus_min_sizer.Add(self.bus_min_nets, 1, wx.EXPAND)
        options_inner.Add(bus_min_sizer, 0, wx.EXPAND | wx.ALL, 3)

        options_inner.AddSpacer(10)

        # Crossing/swap options
        self.no_crossing_layer_check = wx.CheckBox(options_scroll, label="Ignore crossing layers")
        self.no_crossing_layer_check.SetToolTip("Count crossings regardless of layer overlap")
        options_inner.Add(self.no_crossing_layer_check, 0, wx.ALL, 3)

        self.can_swap_to_top = wx.CheckBox(options_scroll, label="Allow swap to top layer")
        self.can_swap_to_top.SetToolTip("Allow swapping stubs to F.Cu (top layer)")
        options_inner.Add(self.can_swap_to_top, 0, wx.ALL, 3)

        # Crossing penalty
        crossing_sizer = wx.BoxSizer(wx.HORIZONTAL)
        crossing_sizer.Add(wx.StaticText(options_scroll, label="Crossing Penalty:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        r = defaults.PARAM_RANGES['crossing_penalty']
        self.crossing_penalty = wx.SpinCtrlDouble(options_scroll, min=r['min'], max=r['max'],
                                                   initial=defaults.CROSSING_PENALTY, inc=r['inc'])
        self.crossing_penalty.SetDigits(r['digits'])
        self.crossing_penalty.SetToolTip("Penalty for crossing assignments in target swap optimization")
        crossing_sizer.Add(self.crossing_penalty, 1, wx.EXPAND)
        options_inner.Add(crossing_sizer, 0, wx.EXPAND | wx.ALL, 3)

        options_inner.AddSpacer(10)

        # Length matching
        length_label = wx.StaticText(options_scroll, label="Length Matching:")
        length_label.SetFont(length_label.GetFont().Bold())
        options_inner.Add(length_label, 0, wx.LEFT | wx.TOP, 3)

        group_sizer = wx.BoxSizer(wx.HORIZONTAL)
        group_sizer.Add(wx.StaticText(options_scroll, label="Groups:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.length_match_groups_ctrl = wx.TextCtrl(options_scroll)
        self.length_match_groups_ctrl.SetToolTip("Net patterns to length-match (comma-separated groups, e.g., 'DATA*,ADDR*')")
        group_sizer.Add(self.length_match_groups_ctrl, 1, wx.EXPAND)
        options_inner.Add(group_sizer, 0, wx.EXPAND | wx.ALL, 3)

        length_params_sizer = wx.BoxSizer(wx.HORIZONTAL)
        length_params_sizer.Add(wx.StaticText(options_scroll, label="Tolerance:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        r = defaults.PARAM_RANGES['length_match_tolerance']
        self.length_match_tolerance = wx.SpinCtrlDouble(options_scroll, min=r['min'], max=r['max'],
                                                        initial=defaults.LENGTH_MATCH_TOLERANCE, inc=r['inc'])
        self.length_match_tolerance.SetDigits(r['digits'])
        self.length_match_tolerance.SetToolTip("Acceptable length difference in mm for matched nets")
        length_params_sizer.Add(self.length_match_tolerance, 0, wx.RIGHT, 10)
        length_params_sizer.Add(wx.StaticText(options_scroll, label="Amplitude:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        r = defaults.PARAM_RANGES['meander_amplitude']
        self.meander_amplitude = wx.SpinCtrlDouble(options_scroll, min=r['min'], max=r['max'],
                                                   initial=defaults.MEANDER_AMPLITUDE, inc=r['inc'])
        self.meander_amplitude.SetDigits(r['digits'])
        self.meander_amplitude.SetToolTip("Height of meander waves for length matching")
        length_params_sizer.Add(self.meander_amplitude, 0)
        options_inner.Add(length_params_sizer, 0, wx.EXPAND | wx.ALL, 3)

        # Time matching option
        time_match_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.time_matching_check = wx.CheckBox(options_scroll, label="Time matching")
        self.time_matching_check.SetValue(False)
        self.time_matching_check.SetToolTip("Match propagation time instead of length (accounts for layer dielectric)")
        time_match_sizer.Add(self.time_matching_check, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 10)
        time_match_sizer.Add(wx.StaticText(options_scroll, label="Time tol (ps):"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        r = defaults.PARAM_RANGES['time_match_tolerance']
        self.time_match_tolerance = wx.SpinCtrlDouble(options_scroll, min=r['min'], max=r['max'],
                                                      initial=defaults.TIME_MATCH_TOLERANCE, inc=r['inc'])
        self.time_match_tolerance.SetDigits(r['digits'])
        self.time_match_tolerance.SetToolTip("Acceptable time variance in picoseconds")
        time_match_sizer.Add(self.time_match_tolerance, 0)
        options_inner.Add(time_match_sizer, 0, wx.EXPAND | wx.ALL, 3)

        options_inner.AddSpacer(10)

        # Debug options
        debug_label = wx.StaticText(options_scroll, label="Debug:")
        debug_label.SetFont(debug_label.GetFont().Bold())
        options_inner.Add(debug_label, 0, wx.LEFT | wx.TOP, 3)

        self.debug_lines_check = wx.CheckBox(options_scroll, label="Add debug visualization lines")
        self.debug_lines_check.SetValue(False)
        self.debug_lines_check.SetToolTip("Add routing paths to User layers for debugging")
        options_inner.Add(self.debug_lines_check, 0, wx.ALL, 3)

        self.verbose_check = wx.CheckBox(options_scroll, label="Verbose output")
        self.verbose_check.SetToolTip("Print detailed diagnostic output")
        options_inner.Add(self.verbose_check, 0, wx.ALL, 3)

        self.skip_routing_check = wx.CheckBox(options_scroll, label="Skip routing (swaps only)")
        self.skip_routing_check.SetToolTip("Skip actual routing, only do swaps and write debug info")
        options_inner.Add(self.skip_routing_check, 0, wx.ALL, 3)

        self.debug_memory_check = wx.CheckBox(options_scroll, label="Debug memory")
        self.debug_memory_check.SetToolTip("Print memory usage statistics at key points")
        options_inner.Add(self.debug_memory_check, 0, wx.ALL, 3)

        self.stats_check = wx.CheckBox(options_scroll, label="Show A* statistics")
        self.stats_check.SetToolTip("Show A* search statistics (iterations, expansions, etc.)")
        options_inner.Add(self.stats_check, 0, wx.ALL, 3)

        options_scroll.SetSizer(options_inner)
        options_box_sizer.Add(options_scroll, 1, wx.EXPAND)
        return options_box_sizer

    def _create_progress_panel(self, panel):
        """Create the progress panel."""
        progress_box = wx.StaticBox(panel, label="Progress")
        progress_sizer = wx.StaticBoxSizer(progress_box, wx.VERTICAL)

        self.progress_bar = wx.Gauge(panel, range=100)
        progress_sizer.Add(self.progress_bar, 0, wx.EXPAND | wx.ALL, 5)

        self.status_text = wx.StaticText(panel, label="Ready")
        progress_sizer.Add(self.status_text, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        return progress_sizer

    def _create_buttons_panel(self, panel):
        """Create the buttons panel."""
        button_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.route_btn = wx.Button(panel, label="Route")
        self.route_btn.SetToolTip("Start routing selected nets")
        self.route_btn.Bind(wx.EVT_BUTTON, self._on_route)
        button_sizer.Add(self.route_btn, 1, wx.RIGHT, 5)

        self.cancel_btn = wx.Button(panel, label="Close")
        self.cancel_btn.SetToolTip("Close dialog (or cancel routing if in progress)")
        self.cancel_btn.Bind(wx.EVT_BUTTON, self._on_cancel_or_close)
        button_sizer.Add(self.cancel_btn, 1)

        return button_sizer

    def _on_cancel_or_close(self, event):
        """Handle cancel/close button - cancel if routing, otherwise close dialog."""
        if self._routing_thread and self._routing_thread.is_alive():
            # Routing is running - cancel it
            self._cancel_requested = True
            self.status_text.SetLabel("Cancelling...")
            # Also notify the differential tab if it has a routing operation running
            if hasattr(self, 'differential_tab'):
                self.differential_tab.request_cancel()
        else:
            # Not routing - close the modal dialog
            self.EndModal(wx.ID_CANCEL)

    def _create_log_tab(self):
        """Create the Log tab."""
        log_panel = wx.Panel(self.notebook)
        log_sizer = wx.BoxSizer(wx.VERTICAL)

        # Log output text control
        self.log_text = wx.TextCtrl(
            log_panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_RICH2 | wx.HSCROLL | wx.VSCROLL | wx.ALWAYS_SHOW_SB
        )
        font = wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.log_text.SetFont(font)
        log_sizer.Add(self.log_text, 1, wx.EXPAND | wx.ALL, 5)

        # Clear log button
        clear_log_btn = wx.Button(log_panel, label="Clear Log")
        clear_log_btn.SetToolTip("Clear all log output")
        clear_log_btn.Bind(wx.EVT_BUTTON, self._on_clear_log)
        log_sizer.Add(clear_log_btn, 0, wx.ALIGN_RIGHT | wx.RIGHT | wx.BOTTOM, 5)

        log_panel.SetSizer(log_sizer)
        return log_panel

    def _create_fanout_tab(self):
        """Create the Fanout tab for BGA/QFN fanout operations."""
        from .fanout_gui import FanoutTab

        def get_shared_params():
            return {
                'track_width': self.track_width.GetValue(),
                'clearance': self.clearance.GetValue(),
                'via_size': self.via_size.GetValue(),
                'via_drill': self.via_drill.GetValue(),
                'layers': self._get_selected_layers(),
                'diff_pair_gap': self.differential_tab.diff_pair_gap.GetValue(),
            }

        return FanoutTab(
            self.notebook,
            self.pcb_data,
            self.board_filename,
            get_shared_params=get_shared_params,
            on_fanout_complete=self._on_tab_operation_complete,
            get_connectivity_check=self._get_connectivity_check_fn
        )

    def _create_planes_tab(self):
        """Create the Planes tab for copper plane creation and repair."""
        from .planes_gui import PlanesTab

        def get_shared_params():
            edge_clearance = self.board_edge_clearance.GetValue() if self.edge_clearance_check.GetValue() else self.clearance.GetValue()
            return {
                'track_width': self.track_width.GetValue(),
                'clearance': self.clearance.GetValue(),
                'via_size': self.via_size.GetValue(),
                'via_drill': self.via_drill.GetValue(),
                'grid_step': self.grid_step.GetValue(),
                'hole_to_hole_clearance': self.hole_to_hole_clearance.GetValue(),
                'max_iterations': int(self.max_iterations.GetValue()),
                'max_ripup': int(self.max_ripup.GetValue()),
                'board_edge_clearance': edge_clearance,
            }

        return PlanesTab(
            self.notebook,
            self.pcb_data,
            self.board_filename,
            get_shared_params=get_shared_params,
            on_planes_complete=self._on_tab_operation_complete,
            get_connectivity_check=self._get_connectivity_check_fn,
            append_log=self._append_log,
            sync_pcb_data_callback=self._sync_pcb_data_from_board
        )

    def _create_differential_tab(self):
        """Create the Differential tab for differential pair routing."""
        from .differential_gui import DifferentialTab

        def get_shared_params():
            return {
                'track_width': self.track_width.GetValue(),
                'clearance': self.clearance.GetValue(),
                'via_size': self.via_size.GetValue(),
                'via_drill': self.via_drill.GetValue(),
            }

        def get_routing_config():
            """Get full routing configuration from the main dialog."""
            return {
                'layers': self._get_selected_layers(),
                'track_width': self.track_width.GetValue(),
                'clearance': self.clearance.GetValue(),
                'via_size': self.via_size.GetValue(),
                'via_drill': self.via_drill.GetValue(),
                'grid_step': self.grid_step.GetValue(),
                'via_cost': self.via_cost.GetValue(),
                'max_iterations': self.max_iterations.GetValue(),
                'max_probe_iterations': self.max_probe_iterations.GetValue(),
                'heuristic_weight': self.heuristic_weight.GetValue(),
                'proximity_heuristic_factor': self.proximity_heuristic_factor.GetValue(),
                'turn_cost': self.turn_cost.GetValue(),
                'direction_preference_cost': self.direction_preference_cost.GetValue(),
                'max_ripup': self.max_ripup.GetValue(),
                'ordering_strategy': self.ordering_strategy.GetString(self.ordering_strategy.GetSelection()),
                'debug_lines': self.debug_lines_check.GetValue(),
                'verbose': self.verbose_check.GetValue(),
                'enable_layer_switch': self.enable_layer_switch.GetValue(),
            }

        def sync_pcb_data():
            """Sync pcb_data from board after routing and clear connectivity cache."""
            self._sync_pcb_data_from_board()
            self._connectivity_cache = {}

        self.differential_tab = DifferentialTab(
            self.notebook,
            self.pcb_data,
            self.board_filename,
            get_shared_params=get_shared_params,
            get_connectivity_check=self._get_connectivity_check_fn,
            get_routing_config=get_routing_config,
            append_log=self._append_log,
            sync_pcb_data_callback=sync_pcb_data
        )
        return self.differential_tab

    def _create_advanced_tab(self):
        """Create the Advanced tab with swappable nets on left, parameters+options on right."""
        panel = wx.Panel(self.notebook)
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # Left side: Swappable nets selection (1:1 ratio with right side)
        left_sizer = self._create_swappable_nets_panel(panel)
        main_sizer.Add(left_sizer, 1, wx.EXPAND | wx.ALL, 5)

        # Right side: Advanced parameters and options
        right_sizer = wx.BoxSizer(wx.VERTICAL)
        right_sizer.Add(self._create_advanced_parameters_panel(panel), 1, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self._create_options_panel(panel), 1, wx.EXPAND)
        main_sizer.Add(right_sizer, 1, wx.EXPAND | wx.ALL, 5)

        panel.SetSizer(main_sizer)

        return panel

    def _create_swappable_nets_panel(self, panel):
        """Create the swappable nets selection panel (left side of Advanced tab)."""
        swap_box = wx.StaticBox(panel, label="Swappable Nets")
        swap_sizer = wx.StaticBoxSizer(swap_box, wx.VERTICAL)

        # Use shared NetSelectionPanel
        self.swappable_net_panel = NetSelectionPanel(
            panel, self.pcb_data,
            instructions="Select nets that can swap targets ...",
            hide_label="Hide connected",
            hide_tooltip="Hide nets that are already fully connected",
            show_hide_checkbox=True,
            show_component_filter=True,
            show_component_dropdown=True,
            min_pads_for_dropdown=3
        )
        swap_sizer.Add(self.swappable_net_panel, 1, wx.EXPAND | wx.ALL, 5)

        # Schematic update options
        self.update_schematic_check = wx.CheckBox(panel, label="Update schematic with swaps")
        self.update_schematic_check.SetToolTip("Update .kicad_sch files with pin swap changes")
        self.update_schematic_check.Bind(wx.EVT_CHECKBOX, self._on_update_schematic_changed)
        swap_sizer.Add(self.update_schematic_check, 0, wx.ALL, 5)

        dir_sizer = wx.BoxSizer(wx.HORIZONTAL)
        dir_sizer.Add(wx.StaticText(panel, label="Schematic dir.:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.schematic_dir_ctrl = wx.TextCtrl(panel)
        self.schematic_dir_ctrl.SetValue(os.path.dirname(self.board_filename))
        self.schematic_dir_ctrl.SetToolTip("Directory containing .kicad_sch files")
        self.schematic_dir_ctrl.Enable(False)
        dir_sizer.Add(self.schematic_dir_ctrl, 1, wx.EXPAND | wx.RIGHT, 5)
        self.browse_schematic_btn = wx.Button(panel, label="...")
        self.browse_schematic_btn.SetToolTip("Browse for schematic directory")
        self.browse_schematic_btn.Bind(wx.EVT_BUTTON, self._on_browse_schematic_dir)
        self.browse_schematic_btn.Enable(False)
        dir_sizer.Add(self.browse_schematic_btn, 0)
        swap_sizer.Add(dir_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        return swap_sizer

    def _create_advanced_parameters_panel(self, panel):
        """Create the advanced parameters panel (right side of Advanced tab)."""
        param_box = wx.StaticBox(panel, label="Parameters")
        param_box_sizer = wx.StaticBoxSizer(param_box, wx.VERTICAL)
        param_scroll = wx.ScrolledWindow(panel, style=wx.VSCROLL)
        param_scroll.SetScrollRate(0, 10)
        param_inner = wx.BoxSizer(wx.VERTICAL)

        # Advanced parameters grid
        param_grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        param_grid.AddGrowableCol(1)
        self._add_advanced_parameters(param_scroll, param_grid)
        param_inner.Add(param_grid, 0, wx.EXPAND | wx.ALL, 5)

        param_scroll.SetSizer(param_inner)
        param_box_sizer.Add(param_scroll, 1, wx.EXPAND)
        return param_box_sizer


    def _on_edge_clearance_check(self, event):
        """Handle edge clearance checkbox change."""
        self.board_edge_clearance.Enable(self.edge_clearance_check.GetValue())

    def _on_main_tab_changed(self, event):
        """Handle main notebook tab change - validate settings when switching tabs."""
        event.Skip()  # Allow normal tab switching

        # Check if switching to Planes tab (index 4)
        if event.GetSelection() == 4:
            # Validate max_track_width >= track_width
            track_width = self.track_width.GetValue()
            max_width = self.planes_tab.repair_options.max_track_width.GetValue()
            if max_width < track_width:
                self.planes_tab.repair_options.max_track_width.SetValue(track_width)

    def _on_update_schematic_changed(self, event):
        """Handle update schematic checkbox change."""
        enabled = self.update_schematic_check.GetValue()
        self.schematic_dir_ctrl.Enable(enabled)
        self.browse_schematic_btn.Enable(enabled)

    def _on_browse_schematic_dir(self, event):
        """Browse for schematic directory."""
        default_path = self.schematic_dir_ctrl.GetValue() or os.path.dirname(self.board_filename)
        dlg = wx.DirDialog(self, "Select Schematic Directory",
                           defaultPath=default_path,
                           style=wx.DD_DEFAULT_STYLE | wx.DD_DIR_MUST_EXIST)
        if dlg.ShowModal() == wx.ID_OK:
            self.schematic_dir_ctrl.SetValue(dlg.GetPath())
        dlg.Destroy()

    def _get_swappable_nets(self):
        """Get list of selected swappable net names."""
        return self.swappable_net_panel.get_selected_nets()

    def _parse_length_match_groups(self):
        """Parse length match groups from the text control."""
        text = self.length_match_groups_ctrl.GetValue().strip()
        if not text:
            return None
        # Split by comma to get separate groups, each group has space-separated patterns
        groups = []
        for group_text in text.split(','):
            patterns = group_text.strip().split()
            if patterns:
                groups.append(patterns)
        return groups if groups else None

    def _load_nets_immediate(self):
        """Load net names from PCB data (fast, no connectivity check)."""
        # The NetSelectionPanel loads nets automatically, but we need to store
        # the list for connectivity checking
        self.all_nets = []

        for net_id, net in self.pcb_data.nets.items():
            name = net.name
            if not name:
                continue
            # Skip unconnected nets
            if name.lower().startswith('unconnected-'):
                continue
            # Skip nets with fewer than 2 pads
            if len(net.pads) < 2:
                continue
            self.all_nets.append((name, net_id))

        # Sort by name
        self.all_nets.sort(key=lambda x: x[0].lower())

        # Update the net panel's all_nets list to match our filtered list
        self.net_panel.all_nets = self.all_nets
        self.net_panel.refresh()
        self.status_text.SetLabel("Loading...")

    def _deferred_init(self):
        """Run after dialog is shown: sync board and check connectivity."""
        # Set up connectivity check function on the net panel
        # This function returns True if a net should be hidden (i.e., is connected)
        def is_connected(net_id):
            if net_id in self._connectivity_cache:
                return self._connectivity_cache[net_id]
            is_conn = self._is_net_connected(net_id)
            self._connectivity_cache[net_id] = is_conn
            return is_conn

        self.net_panel.set_check_function(is_connected)
        self.swappable_net_panel.set_check_function(is_connected)
        self.differential_tab.pair_panel.set_check_function(is_connected)

        # Restore saved settings if available, otherwise use defaults
        if self._saved_settings:
            restore_dialog_settings(self, self._saved_settings)
        else:
            # Enable hide checkbox by default on Basic tab only
            if self.net_panel.hide_check:
                self.net_panel.hide_check.SetValue(True)

        # Apply board-level minimum constraints if checkbox is enabled
        self._apply_board_minimums_to_controls()

        # Do initial refresh
        self.refresh_from_board()

    def refresh_from_board(self):
        """Refresh pcb_data from the current board state.

        Call this when re-showing the dialog after the user has made
        changes in KiCad.
        """
        # Save current selections from all net panels BEFORE any refresh
        # Use _checked_nets directly to preserve restored settings (don't sync from visible items)
        saved_selections = {
            'net_panel': set(self.net_panel._checked_nets),
            'swappable_net_panel': set(self.swappable_net_panel._checked_nets),
            'fanout_tab': set(self.fanout_tab.net_panel._checked_nets),
            'planes_tab': set(self.planes_tab.net_panel._checked_nets),
        }
        # DiffPairSelectionPanel uses _checked_pairs, not _checked_nets
        saved_diff_pairs = set(self.differential_tab.pair_panel._checked_pairs)

        # Sync pcb_data with pcbnew's in-memory board state
        # Skip on initial load since pcb_data was built directly from pcbnew
        if self._initial_load:
            self._initial_load = False
        else:
            self.status_text.SetLabel("Syncing with board...")
            wx.Yield()
            self._sync_pcb_data_from_board()

        # Run connectivity check with progress
        self._check_connectivity_with_progress()

        # Restore selections to each panel before refreshing
        self.net_panel._checked_nets = saved_selections['net_panel']
        self.swappable_net_panel._checked_nets = saved_selections['swappable_net_panel']
        self.fanout_tab.net_panel._checked_nets = saved_selections['fanout_tab']
        self.planes_tab.net_panel._checked_nets = saved_selections['planes_tab']
        self.differential_tab.pair_panel._checked_pairs = saved_diff_pairs

        # Refresh all net panels (skip syncing from visible to preserve restored selections)
        self.net_panel.refresh(sync_from_visible=False)
        self.swappable_net_panel.refresh(sync_from_visible=False)
        self.differential_tab.pair_panel.refresh(sync_from_visible=False)
        self.fanout_tab.net_panel.refresh(sync_from_visible=False)
        self.planes_tab.net_panel.refresh(sync_from_visible=False)

        # Update status bar and progress text
        self._update_status_bar()

    def _is_net_connected(self, net_id):
        """Check if a net is already fully connected using check_connected logic."""
        try:
            from check_connected import check_net_connectivity

            net_segments = [s for s in self.pcb_data.segments if s.net_id == net_id]
            net_vias = [v for v in self.pcb_data.vias if v.net_id == net_id]
            net_pads = self.pcb_data.pads_by_net.get(net_id, [])
            net_zones = [z for z in self.pcb_data.zones if z.net_id == net_id]

            # Need at least 2 pads to be a routable net
            if len(net_pads) < 2:
                return True  # Nothing to route

            # No segments means not connected (unless connected via zones)
            if len(net_segments) == 0 and len(net_zones) == 0:
                return False

            result = check_net_connectivity(
                net_id, net_segments, net_vias, net_pads, net_zones,
                tolerance=0.02
            )

            return result['connected']
        except Exception as e:
            import traceback
            traceback.print_exc()
            return False

    def _check_connectivity_with_progress(self):
        """Check connectivity for all nets with progress display."""
        # Check which nets need connectivity check (not in cache)
        uncached_nets = [(name, net_id) for name, net_id in self.all_nets
                         if net_id not in self._connectivity_cache]

        if uncached_nets:
            self.progress_bar.SetRange(len(uncached_nets))
            self.progress_bar.SetValue(0)

            for i, (name, net_id) in enumerate(uncached_nets):
                is_connected = self._is_net_connected(net_id)
                self._connectivity_cache[net_id] = is_connected

                self.progress_bar.SetValue(i + 1)
                self.status_text.SetLabel(f"Checking connectivity... {i + 1}/{len(uncached_nets)}")
                wx.Yield()

        # Note: Don't refresh here - caller (refresh_from_board) will do it
        # after restoring selections
        self.progress_bar.SetValue(0)

    def _update_net_list(self):
        """Update the net list - delegates to panel and updates status."""
        self.net_panel.refresh()
        self.swappable_net_panel.refresh()
        self.differential_tab.pair_panel.refresh()
        self.fanout_tab.net_panel.refresh()
        self.planes_tab.net_panel.refresh()

        # Update status bar and progress text
        self._update_status_bar()

    def _update_status_bar(self):
        """Update the status bar with net counts."""
        total_nets = len(self.all_nets)
        connected_count = sum(1 for v in self._connectivity_cache.values() if v)
        remaining = total_nets - connected_count
        selected_count = len(self.net_panel.get_selected_nets())

        # Update bottom status bar
        self.status_bar.SetLabel(
            f"Total: {total_nets}  |  Connected: {connected_count}  |  "
            f"To route: {remaining}  |  Selected: {selected_count}"
        )

        # Update progress text (simplified)
        self.status_text.SetLabel(f"Ready - {selected_count} nets selected to route")

    def _on_clear_log(self, event):
        """Clear the log text control."""
        self.log_text.Clear()

    def _get_connectivity_check_fn(self):
        """Return a function to check if a net is connected.

        Used as callback by Fanout, Planes, and Differential tabs.
        """
        def is_connected(net_id):
            if net_id in self._connectivity_cache:
                return self._connectivity_cache[net_id]
            is_conn = self._is_net_connected(net_id)
            self._connectivity_cache[net_id] = is_conn
            return is_conn
        return is_connected

    def _on_tab_operation_complete(self):
        """Handle completion of tab operations (fanout, planes, etc.).

        Syncs board data, refreshes connectivity cache, and updates net lists.
        """
        self._sync_pcb_data_from_board()
        self._check_connectivity_with_progress()
        self._update_net_list()

    def _reset_all_settings(self):
        """Reset all settings to defaults, clear log, and uncheck all selections."""
        # Clear log
        self.log_text.Clear()

        # Clear all net selections
        self.net_panel._checked_nets = set()
        self.swappable_net_panel._checked_nets = set()
        self.fanout_tab.net_panel._checked_nets = set()
        self.planes_tab.net_panel._checked_nets = set()
        self.differential_tab.pair_panel._checked_pairs = set()

        # Reset basic parameters to defaults
        self.track_width.SetValue(defaults.TRACK_WIDTH)
        self.clearance.SetValue(defaults.CLEARANCE)
        self.via_size.SetValue(defaults.VIA_SIZE)
        self.via_drill.SetValue(defaults.VIA_DRILL)
        self.grid_step.SetValue(defaults.GRID_STEP)
        self.via_cost.SetValue(defaults.VIA_COST)
        self.max_ripup.SetValue(defaults.MAX_RIPUP)

        # Reset layer selections (select all copper layers by default)
        for layer, cb in self.layer_checks.items():
            cb.SetValue(True)

        # Reset basic options
        self.enable_layer_switch.SetValue(True)
        self.move_text_check.SetValue(True)
        self.add_teardrops_check.SetValue(True)
        self.power_nets_ctrl.SetValue("")
        self.power_widths_ctrl.SetValue("")
        self.no_bga_zones_ctrl.SetValue("ALL")
        self.layer_costs_ctrl.SetValue("")

        # Reset advanced parameters
        self.impedance_check.SetValue(False)
        self.impedance_value.SetValue(50.0)
        self.max_iterations.SetValue(defaults.MAX_ITERATIONS)
        self.max_probe_iterations.SetValue(defaults.MAX_PROBE_ITERATIONS)
        self.heuristic_weight.SetValue(defaults.HEURISTIC_WEIGHT)
        self.proximity_heuristic_factor.SetValue(defaults.PROXIMITY_HEURISTIC_FACTOR)
        self.turn_cost.SetValue(defaults.TURN_COST)
        self.direction_preference_cost.SetValue(defaults.DIRECTION_PREFERENCE_COST)
        self.ordering_strategy.SetSelection(0)
        self.bga_proximity_radius.SetValue(defaults.BGA_PROXIMITY_RADIUS)
        self.bga_proximity_cost.SetValue(defaults.BGA_PROXIMITY_COST)
        self.stub_proximity_radius.SetValue(defaults.STUB_PROXIMITY_RADIUS)
        self.stub_proximity_cost.SetValue(defaults.STUB_PROXIMITY_COST)
        self.via_proximity_cost.SetValue(defaults.VIA_PROXIMITY_COST)
        self.track_proximity_distance.SetValue(defaults.TRACK_PROXIMITY_DISTANCE)
        self.track_proximity_cost.SetValue(defaults.TRACK_PROXIMITY_COST)
        self.vertical_attraction_radius.SetValue(defaults.VERTICAL_ATTRACTION_RADIUS)
        self.vertical_attraction_cost.SetValue(defaults.VERTICAL_ATTRACTION_COST)
        self.ripped_route_avoidance_radius.SetValue(defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS)
        self.ripped_route_avoidance_cost.SetValue(defaults.RIPPED_ROUTE_AVOIDANCE_COST)
        self.routing_clearance_margin.SetValue(defaults.ROUTING_CLEARANCE_MARGIN)
        self.hole_to_hole_clearance.SetValue(defaults.HOLE_TO_HOLE_CLEARANCE)
        self.edge_clearance_check.SetValue(False)
        self.board_edge_clearance.SetValue(defaults.BOARD_EDGE_CLEARANCE)
        self.board_edge_clearance.Enable(False)
        self.direction_choice.SetSelection(0)

        # Reset advanced options
        self.mps_reverse_rounds.SetValue(True)
        self.mps_layer_swap.SetValue(True)
        self.mps_segment_intersection.SetValue(True)
        self.bus_enabled.SetValue(False)
        self.bus_detection_radius.SetValue(defaults.BUS_DETECTION_RADIUS)
        self.bus_attraction_radius.SetValue(defaults.BUS_ATTRACTION_RADIUS)
        self.bus_attraction_bonus.SetValue(defaults.BUS_ATTRACTION_BONUS)
        self.bus_min_nets.SetValue(defaults.BUS_MIN_NETS)
        self.no_crossing_layer_check.SetValue(False)
        self.can_swap_to_top.SetValue(True)
        self.crossing_penalty.SetValue(defaults.CROSSING_PENALTY)
        self.length_match_groups_ctrl.SetValue("")
        self.length_match_tolerance.SetValue(defaults.LENGTH_MATCH_TOLERANCE)
        self.meander_amplitude.SetValue(defaults.MEANDER_AMPLITUDE)
        self.time_matching_check.SetValue(defaults.TIME_MATCHING)
        self.time_match_tolerance.SetValue(defaults.TIME_MATCH_TOLERANCE)
        self.debug_lines_check.SetValue(False)
        self.verbose_check.SetValue(False)
        self.skip_routing_check.SetValue(False)
        self.debug_memory_check.SetValue(False)
        self.stats_check.SetValue(False)

        # Reset hide checkboxes
        if self.net_panel.hide_check:
            self.net_panel.hide_check.SetValue(True)
        if self.net_panel.hide_diff_check:
            self.net_panel.hide_diff_check.SetValue(False)
        if self.swappable_net_panel.hide_check:
            self.swappable_net_panel.hide_check.SetValue(False)
        if self.differential_tab.pair_panel.hide_check:
            self.differential_tab.pair_panel.hide_check.SetValue(False)
        if self.fanout_tab.net_panel.hide_check:
            self.fanout_tab.net_panel.hide_check.SetValue(False)
        if self.planes_tab.net_panel.hide_check:
            self.planes_tab.net_panel.hide_check.SetValue(False)

        # Reset filters
        self.net_panel.filter_ctrl.SetValue("")
        self.swappable_net_panel.filter_ctrl.SetValue("")
        self.differential_tab.pair_panel.filter_ctrl.SetValue("")
        self.fanout_tab.net_panel.filter_ctrl.SetValue("")
        self.planes_tab.net_panel.filter_ctrl.SetValue("")

        # Reset component dropdowns to "All"
        if self.net_panel.component_dropdown:
            self.net_panel.component_dropdown.SetSelection(0)
            self.net_panel._component_filter_value = ""
        if self.swappable_net_panel.component_dropdown:
            self.swappable_net_panel.component_dropdown.SetSelection(0)
            self.swappable_net_panel._component_filter_value = ""
        if self.differential_tab.pair_panel.component_dropdown:
            self.differential_tab.pair_panel.component_dropdown.SetSelection(0)
            self.differential_tab.pair_panel._component_filter_value = ""
        if self.fanout_tab.net_panel.component_dropdown:
            self.fanout_tab.net_panel.component_dropdown.SetSelection(0)
            self.fanout_tab.net_panel._component_filter_value = ""
        if self.planes_tab.net_panel.component_dropdown:
            self.planes_tab.net_panel.component_dropdown.SetSelection(0)
            self.planes_tab.net_panel._component_filter_value = ""

        # Reset differential tab
        self.differential_tab.use_netclass_check.SetValue(False)
        self.differential_tab.diff_pair_width.SetValue(defaults.DIFF_PAIR_WIDTH)
        self.differential_tab.diff_pair_width.Enable(True)
        self.differential_tab.diff_pair_gap.SetValue(defaults.DIFF_PAIR_GAP)
        self.differential_tab.diff_pair_gap.Enable(True)
        self.differential_tab.min_turning_radius.SetValue(defaults.DIFF_PAIR_MIN_TURNING_RADIUS)
        self.differential_tab.max_setback_angle.SetValue(defaults.DIFF_PAIR_MAX_SETBACK_ANGLE)
        self.differential_tab.max_turn_angle.SetValue(defaults.DIFF_PAIR_MAX_TURN_ANGLE)
        self.differential_tab.chamfer_extra.SetValue(defaults.DIFF_PAIR_CHAMFER_EXTRA)
        self.differential_tab.fix_polarity_check.SetValue(False)
        self.differential_tab.gnd_via_check.SetValue(False)
        self.differential_tab.intra_match_check.SetValue(False)

        # Reset fanout tab
        self.fanout_tab.fanout_type.SetSelection(0)
        self.fanout_tab._on_type_changed(None)

        # Reset planes tab
        self.planes_tab.mode_selector.SetSelection(0)
        self.planes_tab._on_mode_changed(None)
        self.planes_tab.assignment_panel.clear_assignments()

        # Reset transparency to default
        self.SetTransparent(240)
        self.about_tab.transparency_slider.SetValue(240)

        # Refresh all net panels
        self._update_net_list()

        # Update status
        self.status_text.SetLabel("Settings reset to defaults")

    def _append_log(self, text):
        """Append text to the log (thread-safe via CallAfter)."""
        wx.CallAfter(self._do_append_log, text)

    def _do_append_log(self, text):
        """Actually append text to log (must be called on main thread).

        Parses ANSI escape codes and applies corresponding colors.
        """
        # ANSI color code mapping
        ansi_colors = {
            '\033[91m': wx.Colour(220, 50, 50),    # RED
            '\033[92m': wx.Colour(50, 180, 50),    # GREEN
            '\033[93m': wx.Colour(200, 180, 50),   # YELLOW
            '\033[0m': None,                        # RESET
        }

        # Pattern to match ANSI escape codes
        ansi_pattern = re.compile(r'\033\[\d+m')

        # Split text by ANSI codes while keeping track of positions
        parts = ansi_pattern.split(text)
        codes = ansi_pattern.findall(text)

        current_color = None
        for i, part in enumerate(parts):
            if part:
                start_pos = self.log_text.GetLastPosition()
                self.log_text.AppendText(part)
                if current_color:
                    end_pos = self.log_text.GetLastPosition()
                    attr = wx.TextAttr(current_color)
                    self.log_text.SetStyle(start_pos, end_pos, attr)
            # Update color for next part
            if i < len(codes):
                current_color = ansi_colors.get(codes[i])

    def _get_selected_nets(self):
        """Get list of selected net names, including those checked but currently filtered out."""
        return self.net_panel.get_selected_nets()

    def _get_selected_layers(self):
        """Get list of selected layers."""
        return [layer for layer, cb in self.layer_checks.items() if cb.GetValue()]

    def _validate_routing_inputs(self):
        """Validate routing inputs before starting.

        Returns:
            tuple: (selected_nets, selected_layers) if valid, (None, None) if invalid
        """
        selected_nets = self._get_selected_nets()
        if not selected_nets:
            wx.MessageBox(
                "Please select at least one net to route.",
                "No Nets Selected",
                wx.OK | wx.ICON_WARNING
            )
            return None, None

        selected_layers = self._get_selected_layers()
        if not selected_layers:
            wx.MessageBox(
                "Please select at least one layer.",
                "No Layers Selected",
                wx.OK | wx.ICON_WARNING
            )
            return None, None

        return selected_nets, selected_layers

    def _build_routing_config(self, selected_nets, selected_layers):
        """Build the routing configuration dictionary from UI controls.

        Args:
            selected_nets: List of selected net names
            selected_layers: List of selected layer names

        Returns:
            dict: Configuration for the router
        """
        config = {
            'nets': selected_nets,
            'layers': selected_layers,
            # Basic parameters
            'track_width': self.track_width.GetValue(),
            'clearance': self.clearance.GetValue(),
            'via_size': self.via_size.GetValue(),
            'via_drill': self.via_drill.GetValue(),
            'grid_step': self.grid_step.GetValue(),
            'via_cost': self.via_cost.GetValue(),
            'move_copper_text': self.move_text_check.GetValue(),
            'debug_lines': self.debug_lines_check.GetValue(),
            # Impedance routing
            'impedance': self.impedance_value.GetValue() if self.impedance_check.GetValue() else None,
            # Advanced parameters
            'max_iterations': self.max_iterations.GetValue(),
            'max_probe_iterations': self.max_probe_iterations.GetValue(),
            'heuristic_weight': self.heuristic_weight.GetValue(),
            'proximity_heuristic_factor': self.proximity_heuristic_factor.GetValue(),
            'turn_cost': self.turn_cost.GetValue(),
            'direction_preference_cost': self.direction_preference_cost.GetValue(),
            'max_ripup': self.max_ripup.GetValue(),
            'ordering_strategy': self.ordering_strategy.GetString(self.ordering_strategy.GetSelection()),
            'stub_proximity_radius': self.stub_proximity_radius.GetValue(),
            'stub_proximity_cost': self.stub_proximity_cost.GetValue(),
            'via_proximity_cost': self.via_proximity_cost.GetValue(),
            'track_proximity_distance': self.track_proximity_distance.GetValue(),
            'track_proximity_cost': self.track_proximity_cost.GetValue(),
            'bga_proximity_radius': self.bga_proximity_radius.GetValue(),
            'bga_proximity_cost': self.bga_proximity_cost.GetValue(),
            'vertical_attraction_radius': self.vertical_attraction_radius.GetValue(),
            'vertical_attraction_cost': self.vertical_attraction_cost.GetValue(),
            'ripped_route_avoidance_radius': self.ripped_route_avoidance_radius.GetValue(),
            'ripped_route_avoidance_cost': self.ripped_route_avoidance_cost.GetValue(),
            'crossing_penalty': self.crossing_penalty.GetValue(),
            'routing_clearance_margin': self.routing_clearance_margin.GetValue(),
            'hole_to_hole_clearance': self.hole_to_hole_clearance.GetValue(),
            'board_edge_clearance': self.board_edge_clearance.GetValue() if self.edge_clearance_check.GetValue() else 0.0,
            'enable_layer_switch': self.enable_layer_switch.GetValue(),
            # Direction
            'direction': ['forward', 'backward'][self.direction_choice.GetSelection() - 1] if self.direction_choice.GetSelection() > 0 else None,
            # Options
            'add_teardrops': self.add_teardrops_check.GetValue(),
            'verbose': self.verbose_check.GetValue(),
            'skip_routing': self.skip_routing_check.GetValue(),
            'debug_memory': self.debug_memory_check.GetValue(),
            'stats': self.stats_check.GetValue(),
            # MPS options
            'mps_reverse_rounds': self.mps_reverse_rounds.GetValue(),
            'mps_layer_swap': self.mps_layer_swap.GetValue(),
            'mps_segment_intersection': self.mps_segment_intersection.GetValue(),
            # Bus routing options
            'bus_enabled': self.bus_enabled.GetValue(),
            'bus_detection_radius': self.bus_detection_radius.GetValue(),
            'bus_attraction_radius': self.bus_attraction_radius.GetValue(),
            'bus_attraction_bonus': self.bus_attraction_bonus.GetValue(),
            'bus_min_nets': self.bus_min_nets.GetValue(),
            # Crossing/swap options
            'no_crossing_layer_check': self.no_crossing_layer_check.GetValue(),
            'can_swap_to_top_layer': self.can_swap_to_top.GetValue(),
            # Swappable nets
            'swappable_nets': self._get_swappable_nets() or None,
            'schematic_dir': self.schematic_dir_ctrl.GetValue() if self.update_schematic_check.GetValue() else None,
            # Length matching
            'length_match_groups': self._parse_length_match_groups(),
            'length_match_tolerance': self.length_match_tolerance.GetValue(),
            'meander_amplitude': self.meander_amplitude.GetValue(),
            # Time matching
            'time_matching': self.time_matching_check.GetValue(),
            'time_match_tolerance': self.time_match_tolerance.GetValue(),
        }

        # Parse power nets and widths
        power_nets_text = self.power_nets_ctrl.GetValue().strip()
        power_widths_text = self.power_widths_ctrl.GetValue().strip()
        if power_nets_text:
            config['power_nets'] = power_nets_text.split()
            if power_widths_text:
                try:
                    config['power_nets_widths'] = [float(w) for w in power_widths_text.split()]
                except ValueError:
                    config['power_nets_widths'] = []
            else:
                config['power_nets_widths'] = []
        else:
            config['power_nets'] = []
            config['power_nets_widths'] = []

        # Parse no BGA zones
        no_bga_text = self.no_bga_zones_ctrl.GetValue().strip()
        if no_bga_text.upper() == 'ALL':
            config['no_bga_zones'] = []  # Empty list means disable all
        elif no_bga_text:
            config['no_bga_zones'] = no_bga_text.split()
        else:
            config['no_bga_zones'] = None  # None means use BGA zones

        # Parse layer costs
        layer_costs_text = self.layer_costs_ctrl.GetValue().strip()
        if layer_costs_text:
            try:
                config['layer_costs'] = [float(c) for c in layer_costs_text.split()]
            except ValueError:
                config['layer_costs'] = []
        else:
            config['layer_costs'] = []

        # If using net class definitions, build per-class parameter mapping
        if self.use_netclass_check.GetValue():
            nets_by_class = self._group_nets_by_class(selected_nets)
            class_params = {}
            for class_name in nets_by_class.keys():
                params = self._get_netclass_params(class_name)
                if params:
                    class_params[class_name] = params
                else:
                    # Fallback to current control values
                    class_params[class_name] = {
                        'track_width': config['track_width'],
                        'clearance': config['clearance'],
                        'via_size': config['via_size'],
                        'via_drill': config['via_drill'],
                    }
            config['use_netclass_params'] = True
            config['nets_by_class'] = nets_by_class
            config['class_params'] = class_params
        else:
            config['use_netclass_params'] = False

        return config

    def _on_use_netclass_changed(self, event):
        """Handle the 'Use net class definitions' checkbox toggle."""
        use_netclass = self.use_netclass_check.GetValue()

        # List of controls that are overridden by net class
        netclass_controls = [self.track_width, self.clearance, self.via_size, self.via_drill]

        if use_netclass:
            # Get the selected net class name
            class_name = self._get_selected_netclass_name()
            params = self._get_netclass_params(class_name)

            if params:
                # Populate the controls with net class values
                self.track_width.SetValue(params['track_width'])
                self.clearance.SetValue(params['clearance'])
                self.via_size.SetValue(params['via_size'])
                self.via_drill.SetValue(params['via_drill'])

            # Disable the controls
            for ctrl in netclass_controls:
                ctrl.Disable()

            # Connect to net panel's notebook tab changes if in tabbed mode
            if hasattr(self.net_panel, '_netclass_notebook') and self.net_panel._netclass_notebook:
                self.net_panel._netclass_notebook.Bind(wx.EVT_NOTEBOOK_PAGE_CHANGED, self._on_netclass_tab_changed)
        else:
            # Enable the controls
            for ctrl in netclass_controls:
                ctrl.Enable()

            # Unbind tab change handler
            if hasattr(self.net_panel, '_netclass_notebook') and self.net_panel._netclass_notebook:
                self.net_panel._netclass_notebook.Unbind(wx.EVT_NOTEBOOK_PAGE_CHANGED)

    def _on_tabbed_view_changed(self, notebook):
        """Called when net panel's tabbed view is created or destroyed.

        Args:
            notebook: The wx.Notebook if tabbed view was created, None if destroyed
        """
        if notebook and self.use_netclass_check.GetValue():
            # Tabbed view was created and we're using net class definitions
            # Bind the tab change event and update parameters for current tab
            notebook.Bind(wx.EVT_NOTEBOOK_PAGE_CHANGED, self._on_netclass_tab_changed)
            class_name = self._get_selected_netclass_name()
            params = self._get_netclass_params(class_name)
            if params:
                self.track_width.SetValue(params['track_width'])
                self.clearance.SetValue(params['clearance'])
                self.via_size.SetValue(params['via_size'])
                self.via_drill.SetValue(params['via_drill'])
        elif notebook is None and hasattr(self, '_last_notebook') and self._last_notebook:
            # Tabbed view was destroyed - unbind from old notebook
            try:
                self._last_notebook.Unbind(wx.EVT_NOTEBOOK_PAGE_CHANGED)
            except Exception:
                pass  # Notebook may already be destroyed

        # Keep track of the notebook for cleanup
        self._last_notebook = notebook

    def _on_netclass_tab_changed(self, event):
        """Handle net class tab change to update parameters."""
        event.Skip()  # Allow normal tab switching

        if not self.use_netclass_check.GetValue():
            return

        class_name = self._get_selected_netclass_name()
        params = self._get_netclass_params(class_name)

        if params:
            self.track_width.SetValue(params['track_width'])
            self.clearance.SetValue(params['clearance'])
            self.via_size.SetValue(params['via_size'])
            self.via_drill.SetValue(params['via_drill'])

    def _get_selected_netclass_name(self):
        """Get the currently selected net class name from the net panel."""
        if (hasattr(self.net_panel, '_separate_by_netclass') and
            self.net_panel._separate_by_netclass and
            self.net_panel._netclass_notebook):
            # Get the selected tab's class name
            current_tab = self.net_panel._netclass_notebook.GetSelection()
            if current_tab >= 0 and current_tab < len(self.net_panel._netclass_names):
                return self.net_panel._netclass_names[current_tab]
        return 'Default'

    def _get_netclass_params(self, class_name):
        """Get parameters for a net class."""
        return _get_netclass_parameters(class_name)

    def _group_nets_by_class(self, net_names):
        """Group net names by their net class.

        Args:
            net_names: List of net names

        Returns:
            dict: {class_name: [net_names]} mapping
        """
        # Use the net_to_class mapping from the net panel if available
        if (hasattr(self.net_panel, '_net_to_class') and
            self.net_panel._net_to_class):
            net_to_class = self.net_panel._net_to_class
        else:
            # Build mapping from pcbnew
            from .fanout_gui import _get_net_classes_from_board
            net_to_class, _ = _get_net_classes_from_board()

        # Group nets by class
        groups = {}
        for net_name in net_names:
            class_name = net_to_class.get(net_name, 'Default')
            if class_name not in groups:
                groups[class_name] = []
            groups[class_name].append(net_name)

        return groups

    def _on_route(self, event):
        """Handle route button click."""
        selected_nets, selected_layers = self._validate_routing_inputs()
        if selected_nets is None:
            return

        # Disable UI during routing
        self.route_btn.Disable()
        self.cancel_btn.SetLabel("Cancel")
        self._cancel_requested = False
        self._routing_start_time = time.time()

        # Build configuration
        config = self._build_routing_config(selected_nets, selected_layers)

        # Run routing in a thread
        self._routing_thread = threading.Thread(
            target=self._run_routing,
            args=(config,),
            daemon=True
        )
        self._routing_thread.start()

        # Poll for completion
        self._poll_routing()

    def _run_routing(self, config):
        """Run the routing in a background thread."""
        # Set up stdout redirection to capture routing output
        original_stdout = sys.stdout
        sys.stdout = StdoutRedirector(self._append_log, original_stdout)

        try:
            try:
                # Capture stdout during import so startup_checks messages
                # are preserved for error reporting
                import io
                captured = io.StringIO()
                old_stdout = sys.stdout
                sys.stdout = captured
                try:
                    from route import batch_route
                finally:
                    sys.stdout = old_stdout
                    captured_output = captured.getvalue()
                if captured_output:
                    self._append_log(captured_output)
            except SystemExit as e:
                captured_output = captured.getvalue() if 'captured' in dir() else ''
                # Check which dependencies are missing
                missing = []
                try:
                    import numpy
                except ImportError:
                    missing.append('numpy')
                try:
                    import scipy
                except ImportError:
                    missing.append('scipy')
                try:
                    from shapely.geometry import Polygon
                except ImportError:
                    missing.append('shapely')

                if missing:
                    msg = f"Missing Python dependencies: {', '.join(missing)}\n\n"
                    msg += "Install them using KiCad's Python interpreter:\n"
                    msg += f"  {sys.executable} -m pip install " + " ".join(missing)
                    raise RuntimeError(msg)

                # Check if Rust router is the problem
                try:
                    rust_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'rust_router')
                    if rust_dir not in sys.path:
                        sys.path.insert(0, rust_dir)
                    import grid_router
                except ImportError:
                    msg = "Rust router module not found.\n\n"
                    msg += "Build it by running:\n"
                    msg += "  python build_router.py\n\n"
                    msg += "from the KiCadRoutingTools directory."
                    raise RuntimeError(msg)

                # Include any captured output in the error message
                if captured_output.strip():
                    raise RuntimeError(captured_output.strip())
                raise RuntimeError(f"Startup check failed: {e}")

            def check_cancel():
                return self._cancel_requested

            def on_progress(current, total, net_name=""):
                wx.CallAfter(self._update_progress, current, total, net_name)
                # Brief sleep releases GIL, allowing main thread to process CallAfter events
                time.sleep(0.01)

            # Build net_clearances for ALL nets on the PCB
            # This ensures obstacles from nets with larger clearance (e.g., Wide class pads)
            # are properly expanded even when routing nets from a different class
            net_clearances = {}
            net_name_to_id = {net.name: net.net_id for net in self.pcb_data.nets.values()}
            try:
                from .fanout_gui import _get_net_classes_from_board
                all_net_to_class, all_class_names = _get_net_classes_from_board()
                # Cache class clearances
                class_clearance_cache = {}
                for cname in all_class_names:
                    params = self._get_netclass_params(cname)
                    if params:
                        class_clearance_cache[cname] = params.get('clearance', config['clearance'])
                    else:
                        class_clearance_cache[cname] = config['clearance']
                # Build net_clearances for ALL nets
                for net_name, net_id in net_name_to_id.items():
                    cname = all_net_to_class.get(net_name, 'Default')
                    net_clearances[net_id] = class_clearance_cache.get(cname, config['clearance'])
            except Exception as e:
                print(f"Warning: Could not get net class clearances: {e}")
                # Fall back to using config clearance for all nets

            def run_batch(net_names, track_width, clearance, via_size, via_drill):
                """Run batch_route with given parameters."""
                return batch_route(
                    input_file=self.board_filename,
                    output_file="",  # Not used when return_results=True
                    net_names=net_names,
                    layers=config['layers'],
                    track_width=track_width,
                    clearance=clearance,
                    via_size=via_size,
                    via_drill=via_drill,
                    grid_step=config['grid_step'],
                    via_cost=config['via_cost'],
                    impedance=config.get('impedance'),
                    max_iterations=config['max_iterations'],
                    max_probe_iterations=config.get('max_probe_iterations', 5000),
                    heuristic_weight=config['heuristic_weight'],
                    proximity_heuristic_factor=config.get('proximity_heuristic_factor', 0.02),
                    turn_cost=config['turn_cost'],
                    direction_preference_cost=config.get('direction_preference_cost', 50),
                    max_rip_up_count=config['max_ripup'],
                    ordering_strategy=config['ordering_strategy'],
                    direction_order=config.get('direction'),
                    stub_proximity_radius=config['stub_proximity_radius'],
                    stub_proximity_cost=config['stub_proximity_cost'],
                    via_proximity_cost=config['via_proximity_cost'],
                    track_proximity_distance=config['track_proximity_distance'],
                    track_proximity_cost=config['track_proximity_cost'],
                    bga_proximity_radius=config.get('bga_proximity_radius', 7.0),
                    bga_proximity_cost=config.get('bga_proximity_cost', 0.2),
                    vertical_attraction_radius=config.get('vertical_attraction_radius', 1.0),
                    vertical_attraction_cost=config.get('vertical_attraction_cost', 0.0),
                    ripped_route_avoidance_radius=config.get('ripped_route_avoidance_radius', 1.0),
                    ripped_route_avoidance_cost=config.get('ripped_route_avoidance_cost', 0.1),
                    crossing_penalty=config.get('crossing_penalty', 1000.0),
                    routing_clearance_margin=config['routing_clearance_margin'],
                    hole_to_hole_clearance=config['hole_to_hole_clearance'],
                    board_edge_clearance=config['board_edge_clearance'],
                    enable_layer_switch=config['enable_layer_switch'],
                    crossing_layer_check=not config.get('no_crossing_layer_check', False),
                    can_swap_to_top_layer=config.get('can_swap_to_top_layer', False),
                    swappable_net_patterns=config.get('swappable_nets'),
                    schematic_dir=config.get('schematic_dir'),
                    mps_reverse_rounds=config.get('mps_reverse_rounds', False),
                    mps_layer_swap=config.get('mps_layer_swap', False),
                    mps_segment_intersection=config.get('mps_segment_intersection', False),
                    bus_enabled=config.get('bus_enabled', False),
                    bus_detection_radius=config.get('bus_detection_radius', 5.0),
                    bus_attraction_radius=config.get('bus_attraction_radius', 5.0),
                    bus_attraction_bonus=config.get('bus_attraction_bonus', 5000),
                    bus_min_nets=config.get('bus_min_nets', 2),
                    power_nets=config.get('power_nets', []),
                    power_nets_widths=config.get('power_nets_widths', []),
                    disable_bga_zones=config.get('no_bga_zones'),
                    layer_costs=config.get('layer_costs', []),
                    length_match_groups=config.get('length_match_groups'),
                    length_match_tolerance=config.get('length_match_tolerance', 0.1),
                    meander_amplitude=config.get('meander_amplitude', 1.0),
                    time_matching=config.get('time_matching', False),
                    time_match_tolerance=config.get('time_match_tolerance', 1.0),
                    add_teardrops=config.get('add_teardrops', False),
                    verbose=config.get('verbose', False),
                    skip_routing=config.get('skip_routing', False),
                    debug_memory=config.get('debug_memory', False),
                    collect_stats=config.get('stats', False),
                    debug_lines=config['debug_lines'],
                    cancel_check=check_cancel,
                    progress_callback=on_progress,
                    return_results=True,
                    pcb_data=self.pcb_data,
                    net_clearances=net_clearances,
                )

            # Check if using per-netclass parameters
            if config.get('use_netclass_params') and config.get('nets_by_class'):
                # Route each net class group with its own parameters
                total_successful = 0
                total_failed = 0
                all_results = {'results': [], 'all_swap_vias': [], 'exclusion_zone_lines': [], 'boundary_debug_labels': []}

                class_names = list(config['nets_by_class'].keys())
                total_classes = len(class_names)

                for class_idx, class_name in enumerate(class_names):
                    if self._cancel_requested:
                        break

                    class_nets = config['nets_by_class'][class_name]
                    params = config['class_params'].get(class_name, {})

                    # Update status to show which class is being routed
                    track_w = params.get('track_width', config['track_width'])

                    print(f"\nRouting {len(class_nets)} nets from class '{class_name}' "
                          f"(track={track_w:.3f}mm, "
                          f"clearance={params.get('clearance', config['clearance']):.3f}mm)")

                    # Create class-aware progress callback
                    def make_class_progress(cname, cidx, total_cls):
                        def class_on_progress(current, total, net_name=""):
                            status = f"[{cname}] {net_name}" if net_name else f"[{cname}]"
                            wx.CallAfter(self._update_progress, current, total, status)
                            time.sleep(0.01)
                        return class_on_progress

                    class_progress = make_class_progress(class_name, class_idx, total_classes)

                    successful, failed, batch_time, results_data = batch_route(
                        input_file=self.board_filename,
                        output_file="",
                        net_names=class_nets,
                        layers=config['layers'],
                        track_width=params.get('track_width', config['track_width']),
                        clearance=params.get('clearance', config['clearance']),
                        via_size=params.get('via_size', config['via_size']),
                        via_drill=params.get('via_drill', config['via_drill']),
                        grid_step=config['grid_step'],
                        via_cost=config['via_cost'],
                        impedance=config.get('impedance'),
                        max_iterations=config['max_iterations'],
                        max_probe_iterations=config.get('max_probe_iterations', 5000),
                        heuristic_weight=config['heuristic_weight'],
                        proximity_heuristic_factor=config.get('proximity_heuristic_factor', 0.02),
                        turn_cost=config['turn_cost'],
                        direction_preference_cost=config.get('direction_preference_cost', 50),
                        max_rip_up_count=config['max_ripup'],
                        ordering_strategy=config['ordering_strategy'],
                        direction_order=config.get('direction'),
                        stub_proximity_radius=config['stub_proximity_radius'],
                        stub_proximity_cost=config['stub_proximity_cost'],
                        via_proximity_cost=config['via_proximity_cost'],
                        track_proximity_distance=config['track_proximity_distance'],
                        track_proximity_cost=config['track_proximity_cost'],
                        bga_proximity_radius=config.get('bga_proximity_radius', 7.0),
                        bga_proximity_cost=config.get('bga_proximity_cost', 0.2),
                        vertical_attraction_radius=config.get('vertical_attraction_radius', 1.0),
                        vertical_attraction_cost=config.get('vertical_attraction_cost', 0.0),
                        ripped_route_avoidance_radius=config.get('ripped_route_avoidance_radius', 1.0),
                        ripped_route_avoidance_cost=config.get('ripped_route_avoidance_cost', 0.1),
                        crossing_penalty=config.get('crossing_penalty', 1000.0),
                        routing_clearance_margin=config['routing_clearance_margin'],
                        hole_to_hole_clearance=config['hole_to_hole_clearance'],
                        board_edge_clearance=config['board_edge_clearance'],
                        enable_layer_switch=config['enable_layer_switch'],
                        crossing_layer_check=not config.get('no_crossing_layer_check', False),
                        can_swap_to_top_layer=config.get('can_swap_to_top_layer', False),
                        swappable_net_patterns=config.get('swappable_nets'),
                        schematic_dir=config.get('schematic_dir'),
                        mps_reverse_rounds=config.get('mps_reverse_rounds', False),
                        mps_layer_swap=config.get('mps_layer_swap', False),
                        mps_segment_intersection=config.get('mps_segment_intersection', False),
                        bus_enabled=config.get('bus_enabled', False),
                        bus_detection_radius=config.get('bus_detection_radius', 5.0),
                        bus_attraction_radius=config.get('bus_attraction_radius', 5.0),
                        bus_attraction_bonus=config.get('bus_attraction_bonus', 5000),
                        bus_min_nets=config.get('bus_min_nets', 2),
                        power_nets=config.get('power_nets', []),
                        power_nets_widths=config.get('power_nets_widths', []),
                        disable_bga_zones=config.get('no_bga_zones'),
                        layer_costs=config.get('layer_costs', []),
                        length_match_groups=config.get('length_match_groups'),
                        length_match_tolerance=config.get('length_match_tolerance', 0.1),
                        meander_amplitude=config.get('meander_amplitude', 1.0),
                        time_matching=config.get('time_matching', False),
                        time_match_tolerance=config.get('time_match_tolerance', 1.0),
                        add_teardrops=config.get('add_teardrops', False),
                        verbose=config.get('verbose', False),
                        skip_routing=config.get('skip_routing', False),
                        debug_memory=config.get('debug_memory', False),
                        collect_stats=config.get('stats', False),
                        debug_lines=config['debug_lines'],
                        cancel_check=check_cancel,
                        progress_callback=class_progress,
                        return_results=True,
                        pcb_data=self.pcb_data,
                        net_clearances=net_clearances,
                    )

                    total_successful += successful
                    total_failed += failed
                    if results_data:
                        all_results['results'].extend(results_data.get('results', []))
                        all_results['all_swap_vias'].extend(results_data.get('all_swap_vias', []))
                        all_results['exclusion_zone_lines'].extend(results_data.get('exclusion_zone_lines', []))
                        all_results['boundary_debug_labels'].extend(results_data.get('boundary_debug_labels', []))

                successful = total_successful
                failed = total_failed
                results_data = all_results
            else:
                # Standard routing with single set of parameters
                successful, failed, total_time, results_data = run_batch(
                    config['nets'],
                    config['track_width'],
                    config['clearance'],
                    config['via_size'],
                    config['via_drill'],
                )

            if self._cancel_requested:
                wx.CallAfter(self._routing_cancelled)
            else:
                # Calculate wall time from button press
                wall_time = time.time() - self._routing_start_time
                # Apply results to pcbnew on main thread
                wx.CallAfter(self._apply_results_to_board, results_data, successful, failed, wall_time, config)

        except Exception as e:
            wx.CallAfter(self._routing_error, str(e))
        finally:
            # Restore original stdout
            sys.stdout = original_stdout

    def _poll_routing(self):
        """Poll for routing thread completion."""
        if self._routing_thread and self._routing_thread.is_alive():
            wx.CallLater(100, self._poll_routing)
        else:
            self.route_btn.Enable()
            self.cancel_btn.SetLabel("Close")

    def _update_progress(self, current, total, step_name):
        """Update progress bar and status."""
        if total > 0:
            percent = int(100 * current / total)
            self.progress_bar.SetValue(percent)
            self.progress_bar.SetRange(100)
            self.status_text.SetLabel(f"{step_name} ({current}/{total})")
        else:
            # Setup phase - no count, just show the step name
            self.progress_bar.Pulse()  # Indeterminate progress
            self.status_text.SetLabel(step_name)

    def _apply_results_to_board(self, results_data, successful, failed, total_time, config):
        """Apply routing results directly to the open pcbnew board."""
        import pcbnew

        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox("Board is no longer open", "Error", wx.OK | wx.ICON_ERROR)
            return

        # Move copper text to silkscreen if enabled
        text_moved = 0
        if config.get('move_copper_text', True):
            text_moved = self._move_copper_text_to_silkscreen(board)

        # Track counts for reporting
        tracks_added = 0
        vias_added = 0
        debug_lines_added = 0

        # Get layer mappings
        name_to_id, _ = _build_layer_mappings()

        def get_layer_id(layer_name):
            """Convert layer name to pcbnew layer ID."""
            return name_to_id.get(layer_name, pcbnew.F_Cu)

        # Add segments from routing results
        for result in results_data.get('results', []):
            for seg in result.get('new_segments', []):
                track = pcbnew.PCB_TRACK(board)
                # Convert mm to internal units (nanometers)
                # Round to POSITION_DECIMALS to avoid floating-point precision issues
                track.SetStart(pcbnew.VECTOR2I(
                    pcbnew.FromMM(round(seg.start_x, POSITION_DECIMALS)),
                    pcbnew.FromMM(round(seg.start_y, POSITION_DECIMALS))
                ))
                track.SetEnd(pcbnew.VECTOR2I(
                    pcbnew.FromMM(round(seg.end_x, POSITION_DECIMALS)),
                    pcbnew.FromMM(round(seg.end_y, POSITION_DECIMALS))
                ))
                track.SetWidth(pcbnew.FromMM(round(seg.width, POSITION_DECIMALS)))
                track.SetLayer(get_layer_id(seg.layer))
                track.SetNetCode(seg.net_id)
                board.Add(track)
                tracks_added += 1

            for via in result.get('new_vias', []):
                self._add_via_to_board(board, via, get_layer_id)
                vias_added += 1

        # Add vias from layer swapping
        for via in results_data.get('all_swap_vias', []):
            self._add_via_to_board(board, via, get_layer_id)
            vias_added += 1

        # Add debug visualization lines if enabled
        if config.get('debug_lines', False):
            debug_lines_added = self._add_debug_lines(board, results_data)

        # Build connectivity to register new items properly
        board.BuildConnectivity()

        # Refresh the view
        pcbnew.Refresh()

        # Sync pcb_data from pcbnew board to ensure subsequent routing and
        # connectivity checks see the new tracks
        self._sync_pcb_data_from_board()

        # Update UI and show completion message
        self.progress_bar.SetValue(100)
        self.status_text.SetLabel(f"Complete: {successful} routed, {failed} failed")

        msg = f"Routing complete!\n\n"
        msg += f"Successfully routed: {successful} nets\n"
        msg += f"Failed: {failed}\n"
        msg += f"Time: {total_time:.1f}s\n\n"
        msg += f"Added to board:\n"
        msg += f"  {tracks_added} segments\n"
        msg += f"  {vias_added} vias\n"
        if text_moved > 0:
            msg += f"  {text_moved} text items moved to silkscreen\n"
        if debug_lines_added > 0:
            msg += f"  {debug_lines_added} debug lines\n"
        msg += "\nUse Edit -> Undo to revert changes."

        wx.MessageBox(msg, "Routing Complete", wx.OK | wx.ICON_INFORMATION)

        # Clear the selected nets since they've been routed
        self.net_panel._checked_nets.clear()
        # Also uncheck all visible checkboxes
        for i in range(self.net_panel.net_list.GetCount()):
            self.net_panel.net_list.Check(i, False)
        # Uncheck in tabbed view if active
        if self.net_panel._tabbed_net_lists:
            for check_list in self.net_panel._tabbed_net_lists.values():
                for i in range(check_list.GetCount()):
                    check_list.Check(i, False)

        # Check connectivity after dialog is dismissed
        self._check_connectivity_with_progress()

        # Refresh net list to hide newly connected nets (don't sync from visible since we just cleared)
        self.net_panel.refresh(sync_from_visible=False)
        self._update_status_bar()

    def _add_via_to_board(self, board, via, get_layer_id):
        """Add a via to the pcbnew board."""
        import pcbnew
        pcb_via = pcbnew.PCB_VIA(board)
        # Round to POSITION_DECIMALS to avoid floating-point precision issues
        pcb_via.SetPosition(pcbnew.VECTOR2I(
            pcbnew.FromMM(round(via.x, POSITION_DECIMALS)),
            pcbnew.FromMM(round(via.y, POSITION_DECIMALS))
        ))
        pcb_via.SetWidth(pcbnew.FromMM(round(via.size, POSITION_DECIMALS)))
        pcb_via.SetDrill(pcbnew.FromMM(round(via.drill, POSITION_DECIMALS)))
        pcb_via.SetNetCode(via.net_id)
        if hasattr(via, 'layers') and len(via.layers) >= 2:
            top_layer = get_layer_id(via.layers[0])
            bot_layer = get_layer_id(via.layers[1])
            pcb_via.SetLayerPair(top_layer, bot_layer)
        board.Add(pcb_via)

    def _move_copper_text_to_silkscreen(self, board):
        """Move gr_text items from copper layers to silkscreen."""
        import pcbnew

        count = 0
        for drawing in board.GetDrawings():
            if drawing.GetClass() == "PCB_TEXT":
                layer = drawing.GetLayer()
                # Check if on F.Cu or B.Cu
                if layer == pcbnew.F_Cu:
                    drawing.SetLayer(pcbnew.F_SilkS)
                    count += 1
                elif layer == pcbnew.B_Cu:
                    drawing.SetLayer(pcbnew.B_SilkS)
                    count += 1
        return count

    def _add_debug_lines(self, board, results_data):
        """Add debug visualization lines to User layers."""
        import pcbnew

        count = 0

        # User layer mapping
        user_layers = {
            'User.3': pcbnew.User_3,   # Connector lines
            'User.4': pcbnew.User_4,   # Stub direction arrows
            'User.5': pcbnew.User_5,   # Exclusion zones
            'User.8': pcbnew.User_8,   # Simplified path
            'User.9': pcbnew.User_9,   # Raw A* path
        }

        def add_line(start, end, layer_name, width_mm=0.05):
            nonlocal count
            layer_id = user_layers.get(layer_name, pcbnew.User_9)
            shape = pcbnew.PCB_SHAPE(board)
            shape.SetShape(pcbnew.SHAPE_T_SEGMENT)
            shape.SetStart(pcbnew.VECTOR2I(
                pcbnew.FromMM(start[0]),
                pcbnew.FromMM(start[1])
            ))
            shape.SetEnd(pcbnew.VECTOR2I(
                pcbnew.FromMM(end[0]),
                pcbnew.FromMM(end[1])
            ))
            shape.SetWidth(pcbnew.FromMM(width_mm))
            shape.SetLayer(layer_id)
            board.Add(shape)
            count += 1

        for result in results_data.get('results', []):
            # Raw A* path on User.9
            raw_path = result.get('raw_astar_path', [])
            if len(raw_path) >= 2:
                for i in range(len(raw_path) - 1):
                    x1, y1 = raw_path[i][0], raw_path[i][1]
                    x2, y2 = raw_path[i + 1][0], raw_path[i + 1][1]
                    if abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                        add_line((x1, y1), (x2, y2), 'User.9')

            # Simplified path on User.8
            simplified_path = result.get('simplified_path', [])
            if len(simplified_path) >= 2:
                for i in range(len(simplified_path) - 1):
                    x1, y1 = simplified_path[i][0], simplified_path[i][1]
                    x2, y2 = simplified_path[i + 1][0], simplified_path[i + 1][1]
                    if abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                        add_line((x1, y1), (x2, y2), 'User.8')

            # Connector segments on User.3
            for start, end in result.get('debug_connector_lines', []):
                add_line(start, end, 'User.3')

            # Stub direction arrows on User.4
            for start, end in result.get('debug_stub_arrows', []):
                add_line(start, end, 'User.4')

        # Exclusion zone lines on User.5
        for start, end in results_data.get('exclusion_zone_lines', []):
            add_line(start, end, 'User.5')

        return count

    def _routing_cancelled(self):
        """Handle routing cancellation."""
        self.route_btn.Enable()
        self.cancel_btn.SetLabel("Close")
        self.progress_bar.SetValue(0)
        self.status_text.SetLabel("Cancelled")

    def _routing_error(self, error_msg):
        """Handle routing error."""
        self.route_btn.Enable()
        self.cancel_btn.SetLabel("Close")
        self.progress_bar.SetValue(0)
        self.status_text.SetLabel("Error")
        wx.MessageBox(
            f"Routing error:\n\n{error_msg}",
            "Routing Error",
            wx.OK | wx.ICON_ERROR
        )

    def get_settings(self):
        """Get all current dialog settings for persistence."""
        return get_dialog_settings(self)
