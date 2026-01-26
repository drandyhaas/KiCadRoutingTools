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
                 instructions=None,
                 hide_label="Hide connected",
                 hide_tooltip="Hide nets that are already processed",
                 show_hide_checkbox=True,
                 show_component_filter=True,
                 show_component_dropdown=False,
                 min_pads_for_dropdown=3,
                 show_hide_differential=False,
                 hide_differential_default=True,
                 auto_hide_differential=False):
        """
        Create a net selection panel.

        Args:
            parent: Parent window
            pcb_data: PCBData object with nets and pads
            instructions: Optional instruction text to show at the top
            hide_label: Label for the hide checkbox
            hide_tooltip: Tooltip for the hide checkbox
            show_hide_checkbox: Whether to show the hide checkbox
            show_component_filter: Whether to show the component filter text box
            show_component_dropdown: Whether to show the component dropdown
            min_pads_for_dropdown: Minimum pads for a component to appear in dropdown
            show_hide_differential: Whether to show the hide differential checkbox
            hide_differential_default: Default value for hide differential checkbox
            auto_hide_differential: Auto-hide differential nets when not in differential mode
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.all_nets = []  # List of (name, net_id) tuples
        self._checked_nets = set()
        self._check_fn = None  # Optional connectivity check function
        self._show_hide_checkbox = show_hide_checkbox
        self._show_hide_differential = show_hide_differential
        self._hide_differential_default = hide_differential_default
        self._auto_hide_differential = auto_hide_differential
        self._component_filter_value = ""  # For programmatic component filtering
        self._min_pads_for_dropdown = min_pads_for_dropdown
        self._differential_mode = False  # When True, show diff pairs as "name_P/N"
        self._diff_pairs = {}  # base_name -> (p_net_id, n_net_id) when in diff mode

        self._create_ui(instructions, hide_label, hide_tooltip, show_hide_checkbox,
                       show_component_filter, show_component_dropdown)
        self._load_nets()

    def _create_ui(self, instructions, hide_label, hide_tooltip, show_hide_checkbox,
                   show_component_filter, show_component_dropdown):
        """Create the panel UI."""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Instructions (optional)
        if instructions:
            instr_text = wx.StaticText(self, label=instructions)
            instr_text.Wrap(350)
            sizer.Add(instr_text, 0, wx.ALL, 5)

        # Hide checkbox (optional)
        if show_hide_checkbox:
            self.hide_check = wx.CheckBox(self, label=hide_label)
            self.hide_check.SetValue(False)
            self.hide_check.SetToolTip(hide_tooltip)
            self.hide_check.Bind(wx.EVT_CHECKBOX, self._on_filter_changed)
            sizer.Add(self.hide_check, 0, wx.LEFT | wx.RIGHT | wx.TOP, 5)
        else:
            self.hide_check = None

        # Hide differential checkbox (optional)
        if self._show_hide_differential:
            self.hide_diff_check = wx.CheckBox(self, label="Hide differential")
            self.hide_diff_check.SetValue(self._hide_differential_default)
            self.hide_diff_check.SetToolTip("Hide differential pair nets (_P/_N, +/-)")
            self.hide_diff_check.Bind(wx.EVT_CHECKBOX, self._on_filter_changed)
            sizer.Add(self.hide_diff_check, 0, wx.LEFT | wx.RIGHT | wx.TOP, 5)
        else:
            self.hide_diff_check = None

        # Component dropdown (optional) - shows components with many pads
        if show_component_dropdown:
            comp_dropdown_sizer = wx.BoxSizer(wx.HORIZONTAL)
            comp_dropdown_sizer.Add(wx.StaticText(self, label="Component:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
            self.component_dropdown = wx.Choice(self)
            self.component_dropdown.SetToolTip("Select component to highlight its nets")
            self.component_dropdown.Bind(wx.EVT_CHOICE, self._on_component_dropdown_changed)
            comp_dropdown_sizer.Add(self.component_dropdown, 1, wx.EXPAND)
            sizer.Add(comp_dropdown_sizer, 0, wx.EXPAND | wx.ALL, 5)
            self._populate_component_dropdown()
        else:
            self.component_dropdown = None

        # Filter by name
        filter_sizer = wx.BoxSizer(wx.HORIZONTAL)
        filter_sizer.Add(wx.StaticText(self, label="Filter:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.filter_ctrl = wx.TextCtrl(self)
        self.filter_ctrl.Bind(wx.EVT_TEXT, self._on_filter_changed)
        filter_sizer.Add(self.filter_ctrl, 1, wx.EXPAND)
        sizer.Add(filter_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Filter by component text box (optional)
        if show_component_filter:
            comp_filter_sizer = wx.BoxSizer(wx.HORIZONTAL)
            comp_filter_sizer.Add(wx.StaticText(self, label="Comp Filter:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
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

    def _populate_component_dropdown(self):
        """Populate component dropdown with components having enough pads."""
        if not self.component_dropdown:
            return

        # Count pads per component
        component_pad_counts = {}
        for footprint in self.pcb_data.footprints.values():
            ref = footprint.reference
            pad_count = len(footprint.pads)
            if pad_count >= self._min_pads_for_dropdown:
                component_pad_counts[ref] = pad_count

        # Sort by reference
        sorted_components = sorted(component_pad_counts.keys())

        # Add to dropdown with pad count
        self.component_dropdown.Clear()
        self.component_dropdown.Append("(none)")  # First option to clear filter
        for ref in sorted_components:
            count = component_pad_counts[ref]
            self.component_dropdown.Append(f"{ref} ({count} pads)")

        self.component_dropdown.SetSelection(0)

    def _on_component_dropdown_changed(self, event):
        """Handle component dropdown selection change."""
        if not self.component_dropdown:
            return

        selection = self.component_dropdown.GetSelection()
        if selection <= 0:  # "(none)" selected
            self._component_filter_value = ""
        else:
            # Extract component reference (remove pad count)
            text = self.component_dropdown.GetString(selection)
            ref = text.split(' (')[0]
            self._component_filter_value = ref

        self._update_net_list()

    def _load_nets(self):
        """Load net names from pcb_data."""
        if self._differential_mode:
            self._load_diff_pairs()
            self._update_net_list()
            return

        self.all_nets = []
        self._diff_pairs = {}
        for net_id, net in self.pcb_data.nets.items():
            if not net.name or net_id <= 0:
                continue
            # Skip unconnected nets
            if net.name.lower().startswith('unconnected-'):
                continue
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

        # Component filter - combine text control and dropdown/programmatic value
        component_filter = self._component_filter_value
        if self.component_filter_ctrl:
            text_filter = self.component_filter_ctrl.GetValue().strip()
            if text_filter:
                component_filter = text_filter

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
        component_net_ids = set()
        if component_filter:
            for net_id, pads in self.pcb_data.pads_by_net.items():
                for pad in pads:
                    if pad.component_ref and component_filter.lower() in pad.component_ref.lower():
                        net_info = self.pcb_data.nets.get(net_id)
                        if net_info and net_info.name:
                            component_nets.add(net_info.name)
                            component_net_ids.add(net_id)
                        break

        # Filter by text and component
        filtered_nets = []
        for name, net_id in self.all_nets:
            if filter_text and filter_text not in name.lower():
                continue
            if component_filter:
                # In differential mode, check if either P or N net belongs to the component
                if self._differential_mode and name in self._diff_pairs:
                    p_net_id, n_net_id = self._diff_pairs[name]
                    if p_net_id not in component_net_ids and n_net_id not in component_net_ids:
                        continue
                elif name not in component_nets:
                    continue
            filtered_nets.append((name, net_id))

        # Check if hiding differential nets
        hide_diff = False
        if self.hide_diff_check:
            hide_diff = self.hide_diff_check.GetValue()
        # Auto-hide differential nets when not in differential mode (for fanout tab)
        if self._auto_hide_differential and not self._differential_mode:
            hide_diff = True

        # Populate list
        self.net_list.Clear()
        for name, net_id in filtered_nets:
            # Check if should be hidden (connected)
            if hide_checked and self._check_fn:
                if self._check_fn(net_id):
                    continue
            # Check if should be hidden (differential)
            if hide_diff and self._is_differential_net(name):
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

    def _is_differential_net(self, name):
        """Check if a net name looks like a differential pair net."""
        from net_queries import extract_diff_pair_base
        return extract_diff_pair_base(name) is not None

    def set_differential_mode(self, enabled):
        """Switch between single-ended and differential pair display mode."""
        if self._differential_mode == enabled:
            return
        self._differential_mode = enabled
        self._checked_nets.clear()
        self._load_nets()
        self._update_net_list()

    def _load_diff_pairs(self):
        """Load nets as differential pairs."""
        from net_queries import find_differential_pairs

        # Find all differential pairs
        diff_pairs = find_differential_pairs(self.pcb_data, ['*'])

        self.all_nets = []
        self._diff_pairs = {}
        for base_name, pair in diff_pairs.items():
            display_name = f"{base_name}_P/N"
            # Use p_net_id as the "net_id" for filtering purposes
            self.all_nets.append((display_name, pair.p_net_id))
            self._diff_pairs[display_name] = (pair.p_net_id, pair.n_net_id)

        # Sort by name
        self.all_nets.sort(key=lambda x: x[0].lower())

    def get_selected_diff_pairs(self):
        """Get list of (p_net_id, n_net_id) for selected differential pairs."""
        if not self._differential_mode:
            return []

        # Update _checked_nets with current visible state
        for i in range(self.net_list.GetCount()):
            name = self.net_list.GetString(i)
            if self.net_list.IsChecked(i):
                self._checked_nets.add(name)
            else:
                self._checked_nets.discard(name)

        # Return pair info for checked pairs
        result = []
        for name in self._checked_nets:
            if name in self._diff_pairs:
                result.append(self._diff_pairs[name])
        return result

    def get_selected_component(self):
        """Get the selected component reference from the dropdown, or None if none selected."""
        if not self.component_dropdown:
            return None
        selection = self.component_dropdown.GetSelection()
        if selection <= 0:  # "(none)" or nothing selected
            return None
        # Extract component reference (remove pad count)
        text = self.component_dropdown.GetString(selection)
        return text.split(' (')[0]


class BGAOptionsPanel(wx.Panel):
    """BGA fanout options panel (parameters not in Basic tab)."""

    def __init__(self, parent, on_differential_changed=None):
        """
        Create BGA options panel.

        Args:
            parent: Parent window
            on_differential_changed: Callback(bool) when differential checkbox changes
        """
        super().__init__(parent)
        self._on_differential_changed_callback = on_differential_changed
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

        param_sizer.Add(grid, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(param_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Routing mode section
        mode_box = wx.StaticBox(self, label="Routing Mode")
        mode_sizer = wx.StaticBoxSizer(mode_box, wx.VERTICAL)

        self.differential_check = wx.CheckBox(self, label="Differential pairs")
        self.differential_check.SetValue(False)
        self.differential_check.SetToolTip("Route as differential pairs (auto-detects _P/_N nets)")
        self.differential_check.Bind(wx.EVT_CHECKBOX, self._on_differential_changed)
        mode_sizer.Add(self.differential_check, 0, wx.ALL, 5)

        # Diff pair gap (only shown when differential is checked)
        self.diff_gap_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.diff_gap_label = wx.StaticText(self, label="Diff Pair Gap (mm):")
        self.diff_gap_sizer.Add(self.diff_gap_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        r = defaults.PARAM_RANGES['diff_pair_gap']
        self.diff_pair_gap = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                initial=defaults.BGA_DIFF_PAIR_GAP, inc=r['inc'])
        self.diff_pair_gap.SetDigits(r['digits'])
        self.diff_gap_sizer.Add(self.diff_pair_gap, 1, wx.EXPAND)
        mode_sizer.Add(self.diff_gap_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        # Initially hide diff pair gap
        self.diff_gap_label.Hide()
        self.diff_pair_gap.Hide()

        main_sizer.Add(mode_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

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

    def _on_differential_changed(self, event):
        """Handle differential checkbox change."""
        is_diff = self.differential_check.GetValue()
        self.diff_gap_label.Show(is_diff)
        self.diff_pair_gap.Show(is_diff)
        self.Layout()
        self.GetParent().Layout()
        # Notify callback
        if self._on_differential_changed_callback:
            self._on_differential_changed_callback(is_diff)

    def get_config(self):
        """Get the configuration values (BGA-specific only, shared params come from Basic tab)."""
        is_differential = self.differential_check.GetValue()
        return {
            'exit_margin': self.exit_margin.GetValue(),
            'differential': is_differential,
            'diff_pair_gap': self.diff_pair_gap.GetValue() if is_differential else 0,
            'diff_pair_patterns': ['*'] if is_differential else [],  # Auto-detect all diff pairs when enabled
            'primary_escape': 'horizontal' if self.escape_direction.GetSelection() == 0 else 'vertical',
            'force_escape_direction': self.force_escape.GetValue(),
            'rebalance_escape': self.rebalance_escape.GetValue(),
            'check_for_previous': self.check_previous.GetValue(),
            'no_inner_top_layer': self.no_inner_top.GetValue(),
        }


class QFNOptionsPanel(wx.Panel):
    """QFN fanout options panel (parameters not in Basic tab)."""

    def __init__(self, parent):
        """
        Create QFN options panel.

        Args:
            parent: Parent window
        """
        super().__init__(parent)
        self._create_ui()

    def _create_ui(self):
        """Create the panel UI."""
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Info text - QFN uses component's layer automatically
        info_text = wx.StaticText(self, label="QFN fanout routes on the component's layer.\nTrack width comes from Basic tab.")
        info_text.Wrap(250)
        main_sizer.Add(info_text, 0, wx.ALL, 10)

        # Parameters section
        param_box = wx.StaticBox(self, label="QFN Parameters")
        param_sizer = wx.StaticBoxSizer(param_box, wx.VERTICAL)

        grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        grid.AddGrowableCol(1)

        # Extension parameter
        r = defaults.PARAM_RANGES['qfn_extension']
        grid.Add(wx.StaticText(self, label="Extension (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.extension = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                            initial=defaults.QFN_EXTENSION, inc=r['inc'])
        self.extension.SetDigits(r['digits'])
        self.extension.SetToolTip("Extension past pad edge before bend")
        grid.Add(self.extension, 0, wx.EXPAND)

        param_sizer.Add(grid, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(param_sizer, 0, wx.EXPAND)

        self.SetSizer(main_sizer)

    def get_config(self):
        """Get the configuration values (QFN-specific only, shared params come from Basic tab)."""
        return {
            'extension': self.extension.GetValue(),
        }


class FanoutTab(wx.Panel):
    """Complete fanout tab combining component/net selection with options."""

    def __init__(self, parent, pcb_data, board_filename,
                 get_shared_params=None, on_fanout_complete=None,
                 get_connectivity_check=None):
        """
        Create the fanout tab.

        Args:
            parent: Parent notebook
            pcb_data: PCBData object
            board_filename: Path to the PCB file
            get_shared_params: Callback to get shared parameters from Basic tab
                               Returns dict with track_width, clearance, via_size, via_drill
            on_fanout_complete: Callback after fanout completes
            get_connectivity_check: Callback that returns a connectivity check function
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self.get_shared_params = get_shared_params
        self.on_fanout_complete = on_fanout_complete
        self.get_connectivity_check = get_connectivity_check

        self._create_ui()

        # Set up connectivity check after UI creation
        if self.get_connectivity_check:
            self.net_panel.set_check_function(self.get_connectivity_check())

    def _create_ui(self):
        """Create the tab UI."""
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # Left side: Net selection (same as other tabs)
        net_box = wx.StaticBox(self, label="Net Selection")
        net_box_sizer = wx.StaticBoxSizer(net_box, wx.VERTICAL)

        self.net_panel = NetSelectionPanel(
            self, self.pcb_data,
            instructions="Select nets to fanout...",
            hide_label="Hide connected",
            hide_tooltip="Hide nets that are already fully connected",
            show_hide_checkbox=True,
            show_component_filter=True,
            show_component_dropdown=True,
            min_pads_for_dropdown=3,
            auto_hide_differential=True
        )
        net_box_sizer.Add(self.net_panel, 1, wx.EXPAND)

        main_sizer.Add(net_box_sizer, 1, wx.EXPAND | wx.ALL, 5)

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
        self.bga_options = BGAOptionsPanel(self, on_differential_changed=self._on_bga_differential_changed)
        self.qfn_options = QFNOptionsPanel(self)

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
        self._on_type_changed(None)

    def _on_type_changed(self, event):
        """Handle fanout type change."""
        is_bga = self.fanout_type.GetSelection() == 0

        # Show/hide appropriate options panel
        self.bga_options.Show(is_bga)
        self.qfn_options.Show(not is_bga)

        # When switching to QFN, ensure we're in single-ended mode
        if not is_bga:
            self.net_panel.set_differential_mode(False)

        # Refresh layout
        self.Layout()

    def _on_bga_differential_changed(self, is_differential):
        """Handle BGA differential checkbox change - switch net panel mode."""
        self.net_panel.set_differential_mode(is_differential)

    def _on_fanout(self, event):
        """Handle fanout button click."""
        component_ref = self.net_panel.get_selected_component()
        if not component_ref:
            wx.MessageBox(
                "Please select a component.",
                "No Component Selected",
                wx.OK | wx.ICON_WARNING
            )
            return

        is_bga = self.fanout_type.GetSelection() == 0

        # In differential mode, get the actual net names for selected pairs
        if is_bga and self.net_panel._differential_mode:
            selected_pairs = self.net_panel.get_selected_diff_pairs()
            if not selected_pairs:
                wx.MessageBox(
                    "Please select at least one differential pair to fanout.",
                    "No Pairs Selected",
                    wx.OK | wx.ICON_WARNING
                )
                return
            # Convert net IDs to net names for the filter
            selected_nets = []
            for p_net_id, n_net_id in selected_pairs:
                p_net = self.pcb_data.nets.get(p_net_id)
                n_net = self.pcb_data.nets.get(n_net_id)
                if p_net and p_net.name:
                    selected_nets.append(p_net.name)
                if n_net and n_net.name:
                    selected_nets.append(n_net.name)
        else:
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

        if is_bga:
            config = self.bga_options.get_config()
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

        # Get shared parameters from Basic tab (includes layers)
        shared = self.get_shared_params() if self.get_shared_params else {}
        track_width = shared.get('track_width', defaults.BGA_TRACK_WIDTH)
        clearance = shared.get('clearance', defaults.BGA_CLEARANCE)
        via_size = shared.get('via_size', defaults.BGA_VIA_SIZE)
        via_drill = shared.get('via_drill', defaults.BGA_VIA_DRILL)
        layers = shared.get('layers', defaults.DEFAULT_LAYERS)

        if not layers:
            wx.MessageBox(
                "Please select at least one layer on the Basic tab.",
                "No Layers Selected",
                wx.OK | wx.ICON_WARNING
            )
            self.fanout_btn.Enable()
            self.progress_bar.SetValue(0)
            return

        try:
            from bga_fanout import generate_bga_fanout

            tracks, vias_to_add, vias_to_remove = generate_bga_fanout(
                footprint,
                self.pcb_data,
                net_filter=net_patterns,
                diff_pair_patterns=config['diff_pair_patterns'] or None,
                layers=layers,
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

        # Get extension from config (QFN-specific parameter)
        extension = config.get('extension', defaults.QFN_EXTENSION)

        try:
            from qfn_fanout import generate_qfn_fanout

            # Use the component's layer (F.Cu for top, B.Cu for bottom)
            component_layer = footprint.layer if hasattr(footprint, 'layer') else 'F.Cu'

            tracks, vias = generate_qfn_fanout(
                footprint,
                self.pcb_data,
                net_filter=net_patterns,
                layer=component_layer,
                track_width=track_width,
                extension=extension,
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
            track.SetNetCode(track_dict['net_id'])
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
            via.SetNetCode(via_dict['net_id'])
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
