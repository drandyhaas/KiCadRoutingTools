"""
KiCad Routing Tools - Fanout GUI Components

Provides wx-based panels for BGA and QFN fanout configuration.
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


class NetSelectionPanel(wx.Panel):
    """Reusable net selection panel with filtering."""

    def __init__(self, parent, pcb_data,
                 hide_label="Hide connected",
                 hide_tooltip="Hide nets that are already processed",
                 show_hide_checkbox=True,
                 show_component_filter=True):
        """
        Create a net selection panel.

        Args:
            parent: Parent window
            pcb_data: PCBData object with nets and pads
            hide_label: Label for the hide checkbox
            hide_tooltip: Tooltip for the hide checkbox
            show_hide_checkbox: Whether to show the hide checkbox
            show_component_filter: Whether to show the component filter
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.all_nets = []  # List of (name, net_id) tuples
        self._checked_nets = set()
        self._check_fn = None  # Optional connectivity check function
        self._show_hide_checkbox = show_hide_checkbox
        self._component_filter_value = ""  # For programmatic component filtering

        self._create_ui(hide_label, hide_tooltip, show_hide_checkbox, show_component_filter)
        self._load_nets()

    def _create_ui(self, hide_label, hide_tooltip, show_hide_checkbox, show_component_filter):
        """Create the panel UI."""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Hide checkbox (optional)
        if show_hide_checkbox:
            self.hide_check = wx.CheckBox(self, label=hide_label)
            self.hide_check.SetValue(False)
            self.hide_check.SetToolTip(hide_tooltip)
            self.hide_check.Bind(wx.EVT_CHECKBOX, self._on_filter_changed)
            sizer.Add(self.hide_check, 0, wx.LEFT | wx.RIGHT | wx.TOP, 5)
        else:
            self.hide_check = None

        # Filter by name
        filter_sizer = wx.BoxSizer(wx.HORIZONTAL)
        filter_sizer.Add(wx.StaticText(self, label="Filter:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.filter_ctrl = wx.TextCtrl(self)
        self.filter_ctrl.Bind(wx.EVT_TEXT, self._on_filter_changed)
        filter_sizer.Add(self.filter_ctrl, 1, wx.EXPAND)
        sizer.Add(filter_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Filter by component (optional)
        if show_component_filter:
            comp_filter_sizer = wx.BoxSizer(wx.HORIZONTAL)
            comp_filter_sizer.Add(wx.StaticText(self, label="Component:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
            self.component_filter_ctrl = wx.TextCtrl(self)
            self.component_filter_ctrl.SetToolTip("Filter by component reference (e.g., U1)")
            self.component_filter_ctrl.Bind(wx.EVT_TEXT, self._on_filter_changed)
            comp_filter_sizer.Add(self.component_filter_ctrl, 1, wx.EXPAND)
            sizer.Add(comp_filter_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 5)
        else:
            self.component_filter_ctrl = None

        # Net list
        self.net_list = wx.CheckListBox(self, size=(200, -1), style=wx.LB_EXTENDED)
        self.net_list.Bind(wx.EVT_KEY_DOWN, self._on_net_list_key)
        sizer.Add(self.net_list, 1, wx.EXPAND | wx.ALL, 5)

        # Select/Unselect buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.select_btn = wx.Button(self, label="Select")
        self.select_btn.Bind(wx.EVT_BUTTON, self._on_select)
        self.unselect_btn = wx.Button(self, label="Unselect")
        self.unselect_btn.Bind(wx.EVT_BUTTON, self._on_unselect)
        btn_sizer.Add(self.select_btn, 1, wx.RIGHT, 5)
        btn_sizer.Add(self.unselect_btn, 1)
        sizer.Add(btn_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.SetSizer(sizer)

    def _load_nets(self):
        """Load net names from pcb_data."""
        self.all_nets = []
        for net_id, net in self.pcb_data.nets.items():
            if net.name and net_id > 0:  # Skip unconnected (net 0)
                self.all_nets.append((net.name, net_id))
        self.all_nets.sort(key=lambda x: x[0].lower())
        self._update_net_list()

    def set_check_function(self, fn):
        """Set the function used to check if a net should be hidden.

        Args:
            fn: Function(net_id) -> bool, returns True if net should be hidden
        """
        self._check_fn = fn

    def set_component_filter(self, component_ref):
        """Set the component filter value.

        This can be used to filter by component even when the component filter
        text control is not shown.
        """
        self._component_filter_value = component_ref
        if self.component_filter_ctrl:
            self.component_filter_ctrl.SetValue(component_ref)
        self._update_net_list()

    def _update_net_list(self):
        """Update the net list based on filters."""
        filter_text = self.filter_ctrl.GetValue().lower()

        # Component filter - use text control if shown, otherwise use programmatic value
        if self.component_filter_ctrl:
            component_filter = self.component_filter_ctrl.GetValue().strip()
        else:
            component_filter = self._component_filter_value

        hide_checked = False
        if self.hide_check:
            hide_checked = self.hide_check.GetValue()

        # Save checked state before filtering
        for i in range(self.net_list.GetCount()):
            name = self.net_list.GetString(i)
            if self.net_list.IsChecked(i):
                self._checked_nets.add(name)
            else:
                self._checked_nets.discard(name)

        # Build set of nets connected to the filtered component
        component_nets = set()
        if component_filter:
            for net_id, pads in self.pcb_data.pads_by_net.items():
                for pad in pads:
                    if pad.component_ref and component_filter.lower() in pad.component_ref.lower():
                        net_info = self.pcb_data.nets.get(net_id)
                        if net_info and net_info.name:
                            component_nets.add(net_info.name)
                        break

        # Filter by text and component
        filtered_nets = []
        for name, net_id in self.all_nets:
            if filter_text and filter_text not in name.lower():
                continue
            if component_filter and name not in component_nets:
                continue
            filtered_nets.append((name, net_id))

        # Populate list
        self.net_list.Clear()
        for name, net_id in filtered_nets:
            # Check if should be hidden
            if hide_checked and self._check_fn:
                if self._check_fn(net_id):
                    continue
            idx = self.net_list.Append(name)
            # Restore checked state
            if name in self._checked_nets:
                self.net_list.Check(idx, True)

        # Highlight all items by default
        for i in range(self.net_list.GetCount()):
            self.net_list.SetSelection(i)

    def _on_filter_changed(self, event):
        """Handle filter text change."""
        self._update_net_list()

    def _on_net_list_key(self, event):
        """Handle keyboard events in net list."""
        # Ctrl+A selects all items
        if event.GetKeyCode() == ord('A') and event.ControlDown():
            for i in range(self.net_list.GetCount()):
                self.net_list.SetSelection(i)
        else:
            event.Skip()

    def _get_selected_indices(self):
        """Get indices of selected (highlighted) items in the list."""
        return list(self.net_list.GetSelections())

    def _on_select(self, event):
        """Check the highlighted nets."""
        for i in self._get_selected_indices():
            self.net_list.Check(i, True)

    def _on_unselect(self, event):
        """Uncheck the highlighted nets."""
        for i in self._get_selected_indices():
            self.net_list.Check(i, False)

    def get_selected_nets(self):
        """Get list of selected net names."""
        # Update _checked_nets with current visible state
        for i in range(self.net_list.GetCount()):
            name = self.net_list.GetString(i)
            if self.net_list.IsChecked(i):
                self._checked_nets.add(name)
            else:
                self._checked_nets.discard(name)

        # Return all checked nets
        return list(self._checked_nets)

    def refresh(self):
        """Refresh the net list."""
        self._update_net_list()


class BGAOptionsPanel(wx.Panel):
    """BGA fanout options panel (parameters not in Basic tab)."""

    def __init__(self, parent, copper_layers):
        """
        Create BGA options panel.

        Args:
            parent: Parent window
            copper_layers: List of copper layer names available
        """
        super().__init__(parent)
        self.copper_layers = copper_layers
        self._create_ui()

    def _create_ui(self):
        """Create the panel UI."""
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Parameters section (only BGA-specific params, shared ones come from Basic tab)
        param_box = wx.StaticBox(self, label="BGA Parameters")
        param_sizer = wx.StaticBoxSizer(param_box, wx.VERTICAL)

        grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        grid.AddGrowableCol(1)

        # Exit margin
        r = defaults.PARAM_RANGES['exit_margin']
        grid.Add(wx.StaticText(self, label="Exit Margin (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.exit_margin = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                              initial=defaults.BGA_EXIT_MARGIN, inc=r['inc'])
        self.exit_margin.SetDigits(r['digits'])
        grid.Add(self.exit_margin, 0, wx.EXPAND)

        # Diff pair gap
        r = defaults.PARAM_RANGES['diff_pair_gap']
        grid.Add(wx.StaticText(self, label="Diff Pair Gap (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.diff_pair_gap = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                initial=defaults.BGA_DIFF_PAIR_GAP, inc=r['inc'])
        self.diff_pair_gap.SetDigits(r['digits'])
        grid.Add(self.diff_pair_gap, 0, wx.EXPAND)

        # Diff pair patterns
        grid.Add(wx.StaticText(self, label="Diff Pair Patterns:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.diff_pair_patterns = wx.TextCtrl(self)
        self.diff_pair_patterns.SetToolTip("Glob patterns for diff pairs (e.g., *lvds*), space-separated")
        grid.Add(self.diff_pair_patterns, 0, wx.EXPAND)

        param_sizer.Add(grid, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(param_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Escape direction section
        escape_box = wx.StaticBox(self, label="Escape Direction")
        escape_sizer = wx.StaticBoxSizer(escape_box, wx.VERTICAL)

        self.escape_direction = wx.RadioBox(
            self, label="", choices=["Horizontal", "Vertical"],
            majorDimension=2, style=wx.RA_SPECIFY_COLS
        )
        escape_sizer.Add(self.escape_direction, 0, wx.EXPAND | wx.ALL, 5)

        self.force_escape = wx.CheckBox(self, label="Force escape direction")
        self.force_escape.SetToolTip("Only use primary escape direction, don't fall back")
        escape_sizer.Add(self.force_escape, 0, wx.LEFT | wx.BOTTOM, 5)

        self.rebalance_escape = wx.CheckBox(self, label="Rebalance escape directions")
        self.rebalance_escape.SetToolTip("Rebalance for more even distribution")
        escape_sizer.Add(self.rebalance_escape, 0, wx.LEFT | wx.BOTTOM, 5)

        main_sizer.Add(escape_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Layers section
        layers_box = wx.StaticBox(self, label="Layers")
        layers_sizer = wx.StaticBoxSizer(layers_box, wx.VERTICAL)

        self.layer_checks = {}
        layers_grid = wx.GridSizer(cols=2, hgap=5, vgap=2)
        for layer in self.copper_layers:
            cb = wx.CheckBox(self, label=layer)
            # Default: F.Cu and B.Cu checked
            cb.SetValue(layer in ['F.Cu', 'B.Cu'])
            self.layer_checks[layer] = cb
            layers_grid.Add(cb, 0)
        layers_sizer.Add(layers_grid, 0, wx.EXPAND | wx.ALL, 5)

        main_sizer.Add(layers_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Options section
        options_box = wx.StaticBox(self, label="Options")
        options_sizer = wx.StaticBoxSizer(options_box, wx.VERTICAL)

        self.check_previous = wx.CheckBox(self, label="Skip pads with existing fanout")
        self.check_previous.SetToolTip("Skip pads that already have fanout tracks")
        options_sizer.Add(self.check_previous, 0, wx.ALL, 5)

        self.no_inner_top = wx.CheckBox(self, label="No inner pads on top layer")
        self.no_inner_top.SetToolTip("Prevent inner pads from using F.Cu")
        options_sizer.Add(self.no_inner_top, 0, wx.LEFT | wx.BOTTOM, 5)

        main_sizer.Add(options_sizer, 0, wx.EXPAND)

        self.SetSizer(main_sizer)

    def get_config(self):
        """Get the configuration values (BGA-specific only, shared params come from Basic tab)."""
        return {
            'exit_margin': self.exit_margin.GetValue(),
            'diff_pair_gap': self.diff_pair_gap.GetValue(),
            'diff_pair_patterns': self.diff_pair_patterns.GetValue().split(),
            'primary_escape': 'horizontal' if self.escape_direction.GetSelection() == 0 else 'vertical',
            'force_escape_direction': self.force_escape.GetValue(),
            'rebalance_escape': self.rebalance_escape.GetValue(),
            'layers': [layer for layer, cb in self.layer_checks.items() if cb.GetValue()],
            'check_for_previous': self.check_previous.GetValue(),
            'no_inner_top_layer': self.no_inner_top.GetValue(),
        }


class QFNOptionsPanel(wx.Panel):
    """QFN fanout options panel (parameters not in Basic tab)."""

    def __init__(self, parent, copper_layers):
        """
        Create QFN options panel.

        Args:
            parent: Parent window
            copper_layers: List of copper layer names available
        """
        super().__init__(parent)
        self.copper_layers = copper_layers
        self._create_ui()

    def _create_ui(self):
        """Create the panel UI."""
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Parameters section (QFN-specific only, shared params come from Basic tab)
        param_box = wx.StaticBox(self, label="QFN Parameters")
        param_sizer = wx.StaticBoxSizer(param_box, wx.VERTICAL)

        grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        grid.AddGrowableCol(1)

        # Layer selection (single choice - QFN uses single layer unlike BGA multi-layer)
        grid.Add(wx.StaticText(self, label="Layer:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.layer_choice = wx.Choice(self, choices=self.copper_layers)
        if 'F.Cu' in self.copper_layers:
            self.layer_choice.SetSelection(self.copper_layers.index('F.Cu'))
        elif self.copper_layers:
            self.layer_choice.SetSelection(0)
        grid.Add(self.layer_choice, 0, wx.EXPAND)

        param_sizer.Add(grid, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(param_sizer, 0, wx.EXPAND)

        self.SetSizer(main_sizer)

    def get_config(self):
        """Get the configuration values (QFN-specific only, shared params come from Basic tab)."""
        layer_idx = self.layer_choice.GetSelection()
        layer = self.copper_layers[layer_idx] if layer_idx >= 0 else 'F.Cu'
        return {
            'layer': layer,
        }


class FanoutTab(wx.Panel):
    """Complete fanout tab combining component/net selection with options."""

    def __init__(self, parent, pcb_data, board_filename,
                 get_shared_params=None, on_fanout_complete=None):
        """
        Create the fanout tab.

        Args:
            parent: Parent notebook
            pcb_data: PCBData object
            board_filename: Path to the PCB file
            get_shared_params: Callback to get shared parameters from Basic tab
                               Returns dict with track_width, clearance, via_size, via_drill
            on_fanout_complete: Callback after fanout completes
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self.get_shared_params = get_shared_params
        self.on_fanout_complete = on_fanout_complete

        # Get copper layers from board
        self.copper_layers = pcb_data.board_info.copper_layers if pcb_data.board_info else ['F.Cu', 'B.Cu']

        # Find components that could be fanned out
        self._find_fanout_components()

        self._create_ui()

    def _find_fanout_components(self):
        """Find BGA and QFN/QFP components in the board."""
        from kicad_parser import find_components_by_type

        self.bga_components = []
        self.qfn_components = []

        # Find BGA components (grid-like pad patterns)
        bga_refs = find_components_by_type(self.pcb_data, 'BGA')
        for ref in bga_refs:
            if ref in self.pcb_data.footprints:
                self.bga_components.append(ref)

        # Find QFN/QFP components (peripheral pad patterns)
        for pattern in ['QFN', 'QFP', 'LQFP', 'TQFP', 'SOIC', 'SSOP', 'TSSOP']:
            refs = find_components_by_type(self.pcb_data, pattern)
            for ref in refs:
                if ref in self.pcb_data.footprints and ref not in self.qfn_components:
                    self.qfn_components.append(ref)

        # Sort for consistent display
        self.bga_components.sort()
        self.qfn_components.sort()

    def _create_ui(self):
        """Create the tab UI."""
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # Left side: Component and Net selection
        left_sizer = wx.BoxSizer(wx.VERTICAL)

        # Component selection
        comp_box = wx.StaticBox(self, label="Component")
        comp_sizer = wx.StaticBoxSizer(comp_box, wx.VERTICAL)

        # Component dropdown
        comp_h_sizer = wx.BoxSizer(wx.HORIZONTAL)
        comp_h_sizer.Add(wx.StaticText(self, label="Reference:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.component_choice = wx.Choice(self, choices=[])
        self.component_choice.Bind(wx.EVT_CHOICE, self._on_component_changed)
        comp_h_sizer.Add(self.component_choice, 1, wx.EXPAND)
        comp_sizer.Add(comp_h_sizer, 0, wx.EXPAND | wx.ALL, 5)

        left_sizer.Add(comp_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Net selection panel
        net_box = wx.StaticBox(self, label="Net Selection")
        net_box_sizer = wx.StaticBoxSizer(net_box, wx.VERTICAL)

        self.net_panel = NetSelectionPanel(
            self, self.pcb_data,
            hide_label="Hide fanned out",
            hide_tooltip="Hide nets that already have fanout tracks",
            show_hide_checkbox=False,  # Fanout doesn't need connectivity check
            show_component_filter=False  # Component is selected above
        )
        net_box_sizer.Add(self.net_panel, 1, wx.EXPAND)

        left_sizer.Add(net_box_sizer, 1, wx.EXPAND)

        main_sizer.Add(left_sizer, 1, wx.EXPAND | wx.ALL, 5)

        # Right side: Fanout type and options
        right_sizer = wx.BoxSizer(wx.VERTICAL)

        # Fanout type selector
        type_box = wx.StaticBox(self, label="Fanout Type")
        type_sizer = wx.StaticBoxSizer(type_box, wx.VERTICAL)

        self.fanout_type = wx.RadioBox(
            self, label="", choices=["BGA", "QFN/QFP"],
            majorDimension=2, style=wx.RA_SPECIFY_COLS
        )
        self.fanout_type.Bind(wx.EVT_RADIOBOX, self._on_type_changed)
        type_sizer.Add(self.fanout_type, 0, wx.EXPAND | wx.ALL, 5)

        right_sizer.Add(type_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Options panels (stacked, show/hide based on type)
        self.bga_options = BGAOptionsPanel(self, self.copper_layers)
        self.qfn_options = QFNOptionsPanel(self, self.copper_layers)

        right_sizer.Add(self.bga_options, 1, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self.qfn_options, 1, wx.EXPAND | wx.BOTTOM, 5)

        # Progress section
        progress_box = wx.StaticBox(self, label="Progress")
        progress_sizer = wx.StaticBoxSizer(progress_box, wx.VERTICAL)

        self.status_text = wx.StaticText(self, label="Ready")
        progress_sizer.Add(self.status_text, 0, wx.EXPAND | wx.ALL, 5)

        self.progress_bar = wx.Gauge(self, range=100)
        progress_sizer.Add(self.progress_bar, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        right_sizer.Add(progress_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.fanout_btn = wx.Button(self, label="Fanout")
        self.fanout_btn.Bind(wx.EVT_BUTTON, self._on_fanout)
        btn_sizer.Add(self.fanout_btn, 1, wx.RIGHT, 5)

        right_sizer.Add(btn_sizer, 0, wx.EXPAND)

        main_sizer.Add(right_sizer, 1, wx.EXPAND | wx.ALL, 5)

        self.SetSizer(main_sizer)

        # Initial state
        self._update_component_list()
        self._on_type_changed(None)

    def _update_component_list(self):
        """Update component dropdown based on selected fanout type."""
        is_bga = self.fanout_type.GetSelection() == 0
        components = self.bga_components if is_bga else self.qfn_components

        self.component_choice.Clear()
        for ref in components:
            self.component_choice.Append(ref)

        if components:
            self.component_choice.SetSelection(0)
            self._on_component_changed(None)

    def _on_type_changed(self, event):
        """Handle fanout type change."""
        is_bga = self.fanout_type.GetSelection() == 0

        # Show/hide appropriate options panel
        self.bga_options.Show(is_bga)
        self.qfn_options.Show(not is_bga)

        # Update component list
        self._update_component_list()

        # Refresh layout
        self.Layout()

    def _on_component_changed(self, event):
        """Handle component selection change."""
        component_ref = self.component_choice.GetStringSelection()
        if not component_ref:
            return

        # Filter net list to only show nets connected to this component
        self.net_panel.set_component_filter(component_ref)

    def _on_fanout(self, event):
        """Handle fanout button click."""
        component_ref = self.component_choice.GetStringSelection()
        if not component_ref:
            wx.MessageBox(
                "Please select a component.",
                "No Component Selected",
                wx.OK | wx.ICON_WARNING
            )
            return

        selected_nets = self.net_panel.get_selected_nets()
        if not selected_nets:
            wx.MessageBox(
                "Please select at least one net to fanout.",
                "No Nets Selected",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Get component footprint
        footprint = self.pcb_data.footprints.get(component_ref)
        if not footprint:
            wx.MessageBox(
                f"Component {component_ref} not found.",
                "Error",
                wx.OK | wx.ICON_ERROR
            )
            return

        is_bga = self.fanout_type.GetSelection() == 0

        if is_bga:
            config = self.bga_options.get_config()
            if not config['layers']:
                wx.MessageBox(
                    "Please select at least one layer for BGA fanout.",
                    "No Layers Selected",
                    wx.OK | wx.ICON_WARNING
                )
                return
            self._run_bga_fanout(footprint, selected_nets, config)
        else:
            config = self.qfn_options.get_config()
            self._run_qfn_fanout(footprint, selected_nets, config)

    def _run_bga_fanout(self, footprint, net_patterns, config):
        """Run BGA fanout."""
        self.fanout_btn.Disable()
        self.status_text.SetLabel("Running BGA fanout...")
        self.progress_bar.Pulse()
        wx.Yield()

        # Get shared parameters from Basic tab
        shared = self.get_shared_params() if self.get_shared_params else {}
        track_width = shared.get('track_width', defaults.BGA_TRACK_WIDTH)
        clearance = shared.get('clearance', defaults.BGA_CLEARANCE)
        via_size = shared.get('via_size', defaults.BGA_VIA_SIZE)
        via_drill = shared.get('via_drill', defaults.BGA_VIA_DRILL)

        try:
            from bga_fanout import generate_bga_fanout

            tracks, vias_to_add, vias_to_remove = generate_bga_fanout(
                footprint,
                self.pcb_data,
                net_filter=net_patterns,
                diff_pair_patterns=config['diff_pair_patterns'] or None,
                layers=config['layers'],
                track_width=track_width,
                clearance=clearance,
                diff_pair_gap=config['diff_pair_gap'],
                exit_margin=config['exit_margin'],
                primary_escape=config['primary_escape'],
                force_escape_direction=config['force_escape_direction'],
                rebalance_escape=config['rebalance_escape'],
                via_size=via_size,
                via_drill=via_drill,
                check_for_previous=config['check_for_previous'],
                no_inner_top_layer=config['no_inner_top_layer'],
            )

            self._apply_fanout_results(tracks, vias_to_add)

        except Exception as e:
            import traceback
            traceback.print_exc()
            wx.MessageBox(
                f"BGA fanout failed:\n\n{e}",
                "Fanout Error",
                wx.OK | wx.ICON_ERROR
            )
        finally:
            self.fanout_btn.Enable()
            self.progress_bar.SetValue(0)

    def _run_qfn_fanout(self, footprint, net_patterns, config):
        """Run QFN fanout."""
        self.fanout_btn.Disable()
        self.status_text.SetLabel("Running QFN fanout...")
        self.progress_bar.Pulse()
        wx.Yield()

        # Get shared parameters from Basic tab
        shared = self.get_shared_params() if self.get_shared_params else {}
        track_width = shared.get('track_width', defaults.QFN_TRACK_WIDTH)
        clearance = shared.get('clearance', defaults.QFN_CLEARANCE)

        try:
            from qfn_fanout import generate_qfn_fanout

            tracks, vias = generate_qfn_fanout(
                footprint,
                self.pcb_data,
                net_filter=net_patterns,
                layer=config['layer'],
                track_width=track_width,
                clearance=clearance,
            )

            self._apply_fanout_results(tracks, vias)

        except Exception as e:
            import traceback
            traceback.print_exc()
            wx.MessageBox(
                f"QFN fanout failed:\n\n{e}",
                "Fanout Error",
                wx.OK | wx.ICON_ERROR
            )
        finally:
            self.fanout_btn.Enable()
            self.progress_bar.SetValue(0)

    def _apply_fanout_results(self, tracks, vias):
        """Apply fanout results to the pcbnew board."""
        import pcbnew
        from .swig_gui import _build_layer_mappings

        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox("Board is no longer open", "Error", wx.OK | wx.ICON_ERROR)
            return

        # Get layer mappings
        name_to_id, _ = _build_layer_mappings()

        def get_layer_id(layer_name):
            return name_to_id.get(layer_name, pcbnew.F_Cu)

        tracks_added = 0
        vias_added = 0

        # Add tracks
        for track_dict in tracks:
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(pcbnew.VECTOR2I(
                pcbnew.FromMM(track_dict['start'][0]),
                pcbnew.FromMM(track_dict['start'][1])
            ))
            track.SetEnd(pcbnew.VECTOR2I(
                pcbnew.FromMM(track_dict['end'][0]),
                pcbnew.FromMM(track_dict['end'][1])
            ))
            track.SetWidth(pcbnew.FromMM(track_dict['width']))
            track.SetLayer(get_layer_id(track_dict['layer']))
            track.SetNetCode(track_dict['net'])
            board.Add(track)
            tracks_added += 1

        # Add vias
        for via_dict in vias:
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pcbnew.VECTOR2I(
                pcbnew.FromMM(via_dict['x']),
                pcbnew.FromMM(via_dict['y'])
            ))
            via.SetWidth(pcbnew.FromMM(via_dict['size']))
            via.SetDrill(pcbnew.FromMM(via_dict['drill']))
            via.SetNetCode(via_dict['net'])
            if 'layers' in via_dict and len(via_dict['layers']) >= 2:
                via.SetLayerPair(
                    get_layer_id(via_dict['layers'][0]),
                    get_layer_id(via_dict['layers'][1])
                )
            board.Add(via)
            vias_added += 1

        # Refresh the view
        pcbnew.Refresh()

        # Update status
        self.status_text.SetLabel(f"Complete: {tracks_added} tracks, {vias_added} vias added")
        self.progress_bar.SetValue(100)

        # Show completion message
        msg = f"Fanout complete!\n\n"
        msg += f"Added to board:\n"
        msg += f"  {tracks_added} tracks\n"
        msg += f"  {vias_added} vias\n"
        msg += "\nUse Edit -> Undo to revert changes."

        wx.MessageBox(msg, "Fanout Complete", wx.OK | wx.ICON_INFORMATION)

        # Callback
        if self.on_fanout_complete:
            self.on_fanout_complete()
