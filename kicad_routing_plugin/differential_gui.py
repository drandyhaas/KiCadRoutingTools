"""
KiCad Routing Tools - Differential Pair GUI Components

Provides wx-based panels for differential pair routing configuration.
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
from .gui_utils import StdoutRedirector


class DiffPairSelectionPanel(wx.Panel):
    """Panel for selecting differential pairs to route."""

    def __init__(self, parent, pcb_data, instructions=None,
                 show_hide_checkbox=True, show_component_dropdown=True):
        """
        Create a differential pair selection panel.

        Args:
            parent: Parent window
            pcb_data: PCBData object with nets
            instructions: Optional instruction text
            show_hide_checkbox: Whether to show the hide checkbox
            show_component_dropdown: Whether to show the component dropdown
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.all_pairs = []  # List of (display_name, base_name, p_net_id, n_net_id)
        self._checked_pairs = set()
        self._check_fn = None
        self._suspend_check = False  # Temporarily disable check_fn during restore
        self._component_filter_value = ""

        self._create_ui(instructions, show_hide_checkbox, show_component_dropdown)
        self._load_diff_pairs()

    def _create_ui(self, instructions, show_hide_checkbox, show_component_dropdown):
        """Create the panel UI."""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Instructions
        if instructions:
            instr_text = wx.StaticText(self, label=instructions)
            instr_text.Wrap(350)
            sizer.Add(instr_text, 0, wx.ALL, 5)

        # Hide checkbox
        if show_hide_checkbox:
            self.hide_check = wx.CheckBox(self, label="Hide connected")
            self.hide_check.SetValue(False)
            self.hide_check.SetToolTip("Hide differential pairs that are already fully connected")
            self.hide_check.Bind(wx.EVT_CHECKBOX, self._on_filter_changed)
            sizer.Add(self.hide_check, 0, wx.LEFT | wx.RIGHT | wx.TOP, 5)
        else:
            self.hide_check = None

        # Component dropdown
        if show_component_dropdown:
            comp_sizer = wx.BoxSizer(wx.HORIZONTAL)
            comp_sizer.Add(wx.StaticText(self, label="Component:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
            self.component_dropdown = wx.Choice(self)
            self.component_dropdown.SetToolTip("Select component to highlight its differential pairs")
            self.component_dropdown.Bind(wx.EVT_CHOICE, self._on_component_changed)
            comp_sizer.Add(self.component_dropdown, 1, wx.EXPAND)
            sizer.Add(comp_sizer, 0, wx.EXPAND | wx.ALL, 5)
        else:
            self.component_dropdown = None

        # Filter
        filter_sizer = wx.BoxSizer(wx.HORIZONTAL)
        filter_sizer.Add(wx.StaticText(self, label="Filter:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.filter_ctrl = wx.TextCtrl(self)
        self.filter_ctrl.Bind(wx.EVT_TEXT, self._on_filter_changed)
        filter_sizer.Add(self.filter_ctrl, 1, wx.EXPAND)
        sizer.Add(filter_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Pair list
        self.pair_list = wx.CheckListBox(self, size=(200, -1), style=wx.LB_EXTENDED)
        self.pair_list.Bind(wx.EVT_KEY_DOWN, self._on_list_key)
        sizer.Add(self.pair_list, 1, wx.EXPAND | wx.ALL, 5)

        # Select/Unselect buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        select_btn = wx.Button(self, label="Select")
        select_btn.Bind(wx.EVT_BUTTON, self._on_select)
        unselect_btn = wx.Button(self, label="Unselect")
        unselect_btn.Bind(wx.EVT_BUTTON, self._on_unselect)
        btn_sizer.Add(select_btn, 1, wx.RIGHT, 5)
        btn_sizer.Add(unselect_btn, 1)
        sizer.Add(btn_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.SetSizer(sizer)

    def _load_diff_pairs(self):
        """Load all differential pairs from PCB data."""
        from net_queries import find_differential_pairs

        # Find all differential pairs
        diff_pairs = find_differential_pairs(self.pcb_data, ['*'])

        self.all_pairs = []
        for base_name, pair in diff_pairs.items():
            display_name = f"{base_name}_P/N"
            self.all_pairs.append((display_name, base_name, pair.p_net_id, pair.n_net_id))

        # Sort by name
        self.all_pairs.sort(key=lambda x: x[0].lower())

        # Populate component dropdown
        self._populate_component_dropdown()

        self._update_pair_list()

    def _populate_component_dropdown(self):
        """Populate component dropdown with components that have differential pairs."""
        if not self.component_dropdown:
            return

        # Find components connected to diff pairs
        components = set()
        for _, _, p_net_id, n_net_id in self.all_pairs:
            for net_id in [p_net_id, n_net_id]:
                pads = self.pcb_data.pads_by_net.get(net_id, [])
                for pad in pads:
                    if pad.component_ref:
                        components.add(pad.component_ref)

        sorted_components = sorted(components)

        self.component_dropdown.Clear()
        self.component_dropdown.Append("(none)")
        for ref in sorted_components:
            self.component_dropdown.Append(ref)
        self.component_dropdown.SetSelection(0)

    def _on_component_changed(self, event):
        """Handle component dropdown change."""
        if not self.component_dropdown:
            return
        selection = self.component_dropdown.GetSelection()
        if selection <= 0:
            self._component_filter_value = ""
        else:
            self._component_filter_value = self.component_dropdown.GetString(selection)
        self._update_pair_list()

    def _on_filter_changed(self, event):
        """Handle filter change."""
        self._update_pair_list()

    def _update_pair_list(self, sync_from_visible=True):
        """Update the pair list based on filters.

        Args:
            sync_from_visible: If True, sync _checked_pairs from visible items first.
                              Set to False when restoring settings to avoid clearing them.
        """
        filter_text = self.filter_ctrl.GetValue().lower()
        hide_connected = self.hide_check and self.hide_check.GetValue()

        # Save checked state (only if syncing from visible)
        if sync_from_visible:
            for i in range(self.pair_list.GetCount()):
                name = self.pair_list.GetString(i)
                if self.pair_list.IsChecked(i):
                    self._checked_pairs.add(name)
                else:
                    self._checked_pairs.discard(name)

        # Build set of pairs connected to filtered component
        component_pairs = set()
        if self._component_filter_value:
            for display_name, _, p_net_id, n_net_id in self.all_pairs:
                for net_id in [p_net_id, n_net_id]:
                    pads = self.pcb_data.pads_by_net.get(net_id, [])
                    for pad in pads:
                        if pad.component_ref and self._component_filter_value.lower() in pad.component_ref.lower():
                            component_pairs.add(display_name)
                            break

        # Filter pairs
        filtered_pairs = []
        for display_name, base_name, p_net_id, n_net_id in self.all_pairs:
            if filter_text and filter_text not in display_name.lower():
                continue
            if self._component_filter_value and display_name not in component_pairs:
                continue
            filtered_pairs.append((display_name, base_name, p_net_id, n_net_id))

        # Populate list
        self.pair_list.Clear()
        for display_name, base_name, p_net_id, n_net_id in filtered_pairs:
            # Check if should be hidden (both P and N connected)
            if hide_connected and self._check_fn and not self._suspend_check:
                if self._check_fn(p_net_id) and self._check_fn(n_net_id):
                    continue
            idx = self.pair_list.Append(display_name)
            if display_name in self._checked_pairs:
                self.pair_list.Check(idx, True)

        # Highlight all
        for i in range(self.pair_list.GetCount()):
            self.pair_list.SetSelection(i)

    def _on_list_key(self, event):
        """Handle keyboard events."""
        if event.GetKeyCode() == ord('A') and event.ControlDown():
            for i in range(self.pair_list.GetCount()):
                self.pair_list.SetSelection(i)
        else:
            event.Skip()

    def _on_select(self, event):
        """Check highlighted pairs."""
        for i in self.pair_list.GetSelections():
            self.pair_list.Check(i, True)
            self._checked_pairs.add(self.pair_list.GetString(i))

    def _on_unselect(self, event):
        """Uncheck highlighted pairs."""
        for i in self.pair_list.GetSelections():
            self.pair_list.Check(i, False)
            self._checked_pairs.discard(self.pair_list.GetString(i))

    def set_check_function(self, fn):
        """Set connectivity check function."""
        self._check_fn = fn

    def suspend_check(self):
        """Temporarily disable connectivity checking during settings restore."""
        self._suspend_check = True

    def resume_check(self):
        """Re-enable connectivity checking after settings restore."""
        self._suspend_check = False

    def refresh(self, sync_from_visible=True):
        """Refresh the pair list.

        Args:
            sync_from_visible: If True, sync _checked_pairs from visible items first.
                              Set to False when restoring settings to avoid clearing them.
        """
        self._update_pair_list(sync_from_visible=sync_from_visible)

    def get_selected_pairs(self):
        """Get list of selected differential pair base names."""
        # Update checked state from visible items
        for i in range(self.pair_list.GetCount()):
            name = self.pair_list.GetString(i)
            if self.pair_list.IsChecked(i):
                self._checked_pairs.add(name)
            else:
                self._checked_pairs.discard(name)

        # Return base names (strip _P/N suffix)
        selected = []
        for display_name in self._checked_pairs:
            base_name = display_name.replace('_P/N', '')
            selected.append(base_name)
        return selected

    def get_selected_pair_net_ids(self):
        """Get list of (base_name, p_net_id, n_net_id) for selected pairs."""
        selected_names = set(self._checked_pairs)

        # Update from visible items
        for i in range(self.pair_list.GetCount()):
            name = self.pair_list.GetString(i)
            if self.pair_list.IsChecked(i):
                selected_names.add(name)
            else:
                selected_names.discard(name)

        result = []
        for display_name, base_name, p_net_id, n_net_id in self.all_pairs:
            if display_name in selected_names:
                result.append((base_name, p_net_id, n_net_id))
        return result


class DifferentialTab(wx.Panel):
    """Tab for differential pair routing configuration."""

    def __init__(self, parent, pcb_data, board_filename,
                 get_shared_params=None, get_connectivity_check=None,
                 get_routing_config=None, append_log=None,
                 sync_pcb_data_callback=None):
        """
        Create the differential pair routing tab.

        Args:
            parent: Parent notebook
            pcb_data: PCBData object
            board_filename: Path to the PCB file
            get_shared_params: Callback to get shared parameters from Basic tab
            get_connectivity_check: Callback that returns connectivity check function
            get_routing_config: Callback to get full routing config from main dialog
            append_log: Callback to append text to log
            sync_pcb_data_callback: Callback to sync pcb_data from board after routing
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self.get_shared_params = get_shared_params
        self.get_connectivity_check = get_connectivity_check
        self.get_routing_config = get_routing_config
        self.append_log = append_log
        self.sync_pcb_data_callback = sync_pcb_data_callback
        self._routing_thread = None
        self._cancel_requested = False

        self._create_ui()

        # Set up connectivity check
        if self.get_connectivity_check:
            self.pair_panel.set_check_function(self.get_connectivity_check())

    def _create_ui(self):
        """Create the tab UI."""
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # Left side: Diff pair selection
        pair_box = wx.StaticBox(self, label="Differential Pair Selection")
        pair_box_sizer = wx.StaticBoxSizer(pair_box, wx.VERTICAL)

        self.pair_panel = DiffPairSelectionPanel(
            self, self.pcb_data,
            instructions="Select differential pairs to route...",
            show_hide_checkbox=True,
            show_component_dropdown=True
        )
        pair_box_sizer.Add(self.pair_panel, 1, wx.EXPAND)

        main_sizer.Add(pair_box_sizer, 1, wx.EXPAND | wx.ALL, 5)

        # Right side: Parameters
        right_sizer = wx.BoxSizer(wx.VERTICAL)

        # Diff pair parameters
        param_box = wx.StaticBox(self, label="Differential Pair Parameters")
        param_sizer = wx.StaticBoxSizer(param_box, wx.VERTICAL)

        # Use net class definitions checkbox
        self.use_netclass_check = wx.CheckBox(self, label="Use net class definitions")
        self.use_netclass_check.SetValue(False)
        self.use_netclass_check.SetToolTip("Use DP width and gap from selected net class")
        self.use_netclass_check.Bind(wx.EVT_CHECKBOX, self._on_use_netclass_changed)
        param_sizer.Add(self.use_netclass_check, 0, wx.ALL, 5)

        param_grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        param_grid.AddGrowableCol(1)

        # Diff pair width
        param_grid.Add(wx.StaticText(self, label="Pair Width (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['diff_pair_width']
        self.diff_pair_width = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                  initial=defaults.DIFF_PAIR_WIDTH, inc=r['inc'])
        self.diff_pair_width.SetDigits(r['digits'])
        self.diff_pair_width.SetToolTip("Track width for differential pair traces")
        param_grid.Add(self.diff_pair_width, 0, wx.EXPAND)

        # Diff pair gap
        param_grid.Add(wx.StaticText(self, label="Pair Gap (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['diff_pair_gap']
        self.diff_pair_gap = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                initial=defaults.DIFF_PAIR_GAP, inc=r['inc'])
        self.diff_pair_gap.SetDigits(r['digits'])
        self.diff_pair_gap.SetToolTip("Gap between P and N traces")
        param_grid.Add(self.diff_pair_gap, 0, wx.EXPAND)

        # Min turning radius
        param_grid.Add(wx.StaticText(self, label="Min Turn Radius (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['diff_pair_min_turning_radius']
        self.min_turning_radius = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                     initial=defaults.DIFF_PAIR_MIN_TURNING_RADIUS, inc=r['inc'])
        self.min_turning_radius.SetDigits(r['digits'])
        self.min_turning_radius.SetToolTip("Minimum turning radius for curved routing")
        param_grid.Add(self.min_turning_radius, 0, wx.EXPAND)

        # Max setback angle
        param_grid.Add(wx.StaticText(self, label="Max Setback Angle:"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['diff_pair_max_setback_angle']
        self.max_setback_angle = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                    initial=defaults.DIFF_PAIR_MAX_SETBACK_ANGLE, inc=r['inc'])
        self.max_setback_angle.SetDigits(int(r['digits']))
        self.max_setback_angle.SetToolTip("Maximum angle for setback position search")
        param_grid.Add(self.max_setback_angle, 0, wx.EXPAND)

        # Max turn angle
        param_grid.Add(wx.StaticText(self, label="Max Turn Angle:"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['diff_pair_max_turn_angle']
        self.max_turn_angle = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                 initial=defaults.DIFF_PAIR_MAX_TURN_ANGLE, inc=r['inc'])
        self.max_turn_angle.SetDigits(int(r['digits']))
        self.max_turn_angle.SetToolTip("Max cumulative turn angle before reset (prevents U-turns)")
        param_grid.Add(self.max_turn_angle, 0, wx.EXPAND)

        # Chamfer extra
        param_grid.Add(wx.StaticText(self, label="Meander Chamfer:"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['diff_pair_chamfer_extra']
        self.chamfer_extra = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                initial=defaults.DIFF_PAIR_CHAMFER_EXTRA, inc=r['inc'])
        self.chamfer_extra.SetDigits(r['digits'])
        self.chamfer_extra.SetToolTip("Chamfer multiplier for meanders (>1 avoids P/N crossings)")
        param_grid.Add(self.chamfer_extra, 0, wx.EXPAND)

        param_sizer.Add(param_grid, 0, wx.EXPAND | wx.ALL, 5)

        right_sizer.Add(param_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Options
        options_box = wx.StaticBox(self, label="Options")
        options_sizer = wx.StaticBoxSizer(options_box, wx.VERTICAL)

        self.fix_polarity_check = wx.CheckBox(self, label="Fix polarity swaps")
        self.fix_polarity_check.SetValue(True)
        self.fix_polarity_check.SetToolTip("Swap target pad net assignments if polarity swap is needed")
        options_sizer.Add(self.fix_polarity_check, 0, wx.ALL, 5)

        self.gnd_via_check = wx.CheckBox(self, label="Add GND vias")
        self.gnd_via_check.SetValue(True)
        self.gnd_via_check.SetToolTip("Add GND vias near differential pair signal vias")
        options_sizer.Add(self.gnd_via_check, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.intra_match_check = wx.CheckBox(self, label="Intra-pair length matching")
        self.intra_match_check.SetValue(False)
        self.intra_match_check.SetToolTip("Add meanders to shorter track of each pair for P/N matching")
        options_sizer.Add(self.intra_match_check, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        right_sizer.Add(options_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Status
        status_box = wx.StaticBox(self, label="Status")
        status_sizer = wx.StaticBoxSizer(status_box, wx.VERTICAL)

        self.status_text = wx.StaticText(self, label="Ready")
        status_sizer.Add(self.status_text, 0, wx.ALL, 5)

        self.progress_bar = wx.Gauge(self, range=100)
        status_sizer.Add(self.progress_bar, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        right_sizer.Add(status_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Route and Cancel buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.route_btn = wx.Button(self, label="Route")
        self.route_btn.Bind(wx.EVT_BUTTON, self._on_route)
        btn_sizer.Add(self.route_btn, 1, wx.RIGHT, 5)

        self.cancel_btn = wx.Button(self, label="Close")
        self.cancel_btn.Bind(wx.EVT_BUTTON, self._on_cancel_or_close)
        btn_sizer.Add(self.cancel_btn, 1)

        right_sizer.Add(btn_sizer, 0, wx.EXPAND)

        # Add spacer to push content up
        right_sizer.AddStretchSpacer(1)

        main_sizer.Add(right_sizer, 1, wx.EXPAND | wx.ALL, 5)

        self.SetSizer(main_sizer)

    def _on_cancel_or_close(self, event):
        """Handle cancel/close button - cancel if routing, otherwise close dialog."""
        if self._routing_thread and self._routing_thread.is_alive():
            # Routing is running - cancel it
            self.request_cancel()
        else:
            # Not routing - close the parent modal dialog
            self.GetTopLevelParent().EndModal(wx.ID_CANCEL)

    def request_cancel(self):
        """Request cancellation of the current routing operation."""
        self._cancel_requested = True
        self.status_text.SetLabel("Cancelling...")

    def _on_route(self, event):
        """Handle route button click."""
        selected_pair_info = self.get_selected_pair_net_ids()
        if not selected_pair_info:
            wx.MessageBox(
                "Please select at least one differential pair to route.",
                "No Pairs Selected",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Get routing config from main dialog
        if not self.get_routing_config:
            wx.MessageBox(
                "Routing configuration not available.",
                "Error",
                wx.OK | wx.ICON_ERROR
            )
            return

        routing_config = self.get_routing_config()
        if not routing_config.get('layers'):
            wx.MessageBox(
                "Please select at least one layer on the Basic tab.",
                "No Layers Selected",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Get differential pair specific config
        diff_config = self.get_config()

        # Merge configs
        config = {**routing_config, **diff_config}

        # Build actual net names from selected pair net IDs
        net_names = []
        for base_name, p_net_id, n_net_id in selected_pair_info:
            # Look up actual net names from pcb_data
            if p_net_id in self.pcb_data.nets:
                net_names.append(self.pcb_data.nets[p_net_id].name)
            if n_net_id in self.pcb_data.nets:
                net_names.append(self.pcb_data.nets[n_net_id].name)

        # Disable UI during routing
        self.route_btn.Disable()
        self.cancel_btn.SetLabel("Cancel")
        self._cancel_requested = False

        self.status_text.SetLabel(f"Routing {len(selected_pair_info)} differential pair(s)...")
        self.progress_bar.Pulse()
        wx.Yield()

        # Run routing in a thread
        import threading
        self._routing_thread = threading.Thread(
            target=self._run_diff_routing,
            args=(config, net_names),
            daemon=True
        )
        self._routing_thread.start()

        # Poll for completion
        self._poll_routing()

    def _update_progress(self, current, total, step_name):
        """Update progress bar and status."""
        if total > 0:
            percent = int(100 * current / total)
            self.progress_bar.SetValue(percent)
            self.progress_bar.SetRange(100)
            self.status_text.SetLabel(f"{step_name} ({current}/{total})")
        else:
            # Setup phase - no count, just show the step name
            self.progress_bar.Pulse()
            self.status_text.SetLabel(step_name)

    def _run_diff_routing(self, config, net_names):
        """Run differential pair routing in a background thread."""
        import sys
        import time

        original_stdout = sys.stdout
        if self.append_log:
            sys.stdout = StdoutRedirector(self.append_log, original_stdout)

        try:
            from route_diff import batch_route_diff_pairs

            # Run differential pair routing with return_results=True
            def check_cancel():
                return self._cancel_requested

            def on_progress(current, total, pair_name=""):
                wx.CallAfter(self._update_progress, current, total, pair_name)
                # Brief sleep releases GIL, allowing main thread to process CallAfter events
                time.sleep(0.01)

            successful, failed, total_time, results_data = batch_route_diff_pairs(
                input_file=self.board_filename,
                output_file="",  # Not used when return_results=True
                net_names=net_names,
                layers=config.get('layers', defaults.DEFAULT_LAYERS),
                track_width=config.get('diff_pair_width', defaults.DIFF_PAIR_WIDTH),
                clearance=config.get('clearance', 0.1),
                via_size=config.get('via_size', 0.3),
                via_drill=config.get('via_drill', 0.2),
                grid_step=config.get('grid_step', 0.1),
                via_cost=config.get('via_cost', 50),
                max_iterations=config.get('max_iterations', 200000),
                diff_pair_gap=config.get('diff_pair_gap', 0.101),
                min_turning_radius=config.get('min_turning_radius', 0.2),
                max_setback_angle=config.get('max_setback_angle', 45.0),
                max_turn_angle=config.get('max_turn_angle', 180.0),
                diff_chamfer_extra=config.get('diff_chamfer_extra', 1.5),
                fix_polarity=config.get('fix_polarity', True),
                gnd_via_enabled=config.get('gnd_via_enabled', True),
                diff_pair_intra_match=config.get('diff_pair_intra_match', False),
                enable_layer_switch=config.get('enable_layer_switch', True),
                debug_lines=config.get('debug_lines', False),
                verbose=config.get('verbose', False),
                return_results=True,
                pcb_data=self.pcb_data,
                cancel_check=check_cancel,
                progress_callback=on_progress,
            )

            self._routing_result = {
                'successful': successful,
                'failed': failed,
                'total_time': total_time,
                'results_data': results_data,
            }

        except Exception as e:
            import traceback
            traceback.print_exc()
            self._routing_result = {
                'error': str(e),
            }

        finally:
            sys.stdout = original_stdout

    def _poll_routing(self):
        """Poll for routing completion."""
        if self._routing_thread and self._routing_thread.is_alive():
            # Check if cancel was requested
            if self._cancel_requested:
                self.status_text.SetLabel("Cancelling... (waiting for current operation)")
            # Still running, poll again
            wx.CallLater(100, self._poll_routing)
        else:
            # Routing complete
            self._on_routing_complete()

    def _on_routing_complete(self):
        """Handle routing completion."""
        self.route_btn.Enable()
        self.cancel_btn.SetLabel("Close")
        self.progress_bar.SetValue(0)

        # Check if cancelled
        if self._cancel_requested:
            self._cancel_requested = False
            self.status_text.SetLabel("Cancelled")
            return

        result = getattr(self, '_routing_result', None)
        if not result:
            self.status_text.SetLabel("Routing finished (no result)")
            return

        if 'error' in result:
            self.status_text.SetLabel(f"Routing failed: {result['error']}")
            wx.MessageBox(
                f"Differential pair routing failed:\n\n{result['error']}",
                "Routing Error",
                wx.OK | wx.ICON_ERROR
            )
            return

        successful = result.get('successful', 0)
        failed = result.get('failed', 0)
        total_time = result.get('total_time', 0)
        results_data = result.get('results_data', {})

        # Apply results to the board
        tracks_added, vias_added = self._apply_results_to_board(results_data)

        self.status_text.SetLabel(f"Complete: {successful} routed, {failed} failed in {total_time:.1f}s")

        msg = f"Differential pair routing complete!\n\n"
        msg += f"Routed: {successful}\n"
        msg += f"Failed: {failed}\n"
        msg += f"Time: {total_time:.1f}s\n\n"
        msg += f"Added to board:\n"
        msg += f"  {tracks_added} segments\n"
        msg += f"  {vias_added} vias\n"
        msg += "\nUse Edit -> Undo to revert changes."

        wx.MessageBox(msg, "Routing Complete", wx.OK | wx.ICON_INFORMATION)

        # Refresh the pair list to show updated connectivity
        self.pair_panel.refresh()

    def _apply_results_to_board(self, results_data):
        """Apply routing results directly to the open pcbnew board."""
        import pcbnew
        from .swig_gui import _build_layer_mappings

        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox("Board is no longer open", "Error", wx.OK | wx.ICON_ERROR)
            return 0, 0

        tracks_added = 0
        vias_added = 0

        # Get layer mappings
        name_to_id, _ = _build_layer_mappings()

        def get_layer_id(layer_name):
            return name_to_id.get(layer_name, pcbnew.F_Cu)

        # Add segments from routing results
        for result in results_data.get('results', []):
            for seg in result.get('new_segments', []):
                track = pcbnew.PCB_TRACK(board)
                track.SetStart(pcbnew.VECTOR2I(
                    pcbnew.FromMM(seg.start_x),
                    pcbnew.FromMM(seg.start_y)
                ))
                track.SetEnd(pcbnew.VECTOR2I(
                    pcbnew.FromMM(seg.end_x),
                    pcbnew.FromMM(seg.end_y)
                ))
                track.SetWidth(pcbnew.FromMM(seg.width))
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

        # Build connectivity to register new items properly
        board.BuildConnectivity()

        # Refresh the view
        pcbnew.Refresh()

        # Sync pcb_data from pcbnew board
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()

        return tracks_added, vias_added

    def _add_via_to_board(self, board, via, get_layer_id):
        """Add a via to the pcbnew board."""
        import pcbnew
        pcb_via = pcbnew.PCB_VIA(board)
        pcb_via.SetPosition(pcbnew.VECTOR2I(
            pcbnew.FromMM(via.x),
            pcbnew.FromMM(via.y)
        ))
        pcb_via.SetWidth(pcbnew.FromMM(via.size))
        pcb_via.SetDrill(pcbnew.FromMM(via.drill))
        pcb_via.SetNetCode(via.net_id)
        if hasattr(via, 'layers') and len(via.layers) >= 2:
            top_layer = get_layer_id(via.layers[0])
            bot_layer = get_layer_id(via.layers[1])
            pcb_via.SetLayerPair(top_layer, bot_layer)
        board.Add(pcb_via)

    def get_config(self):
        """Get the differential pair configuration."""
        return {
            'diff_pair_width': self.diff_pair_width.GetValue(),
            'diff_pair_gap': self.diff_pair_gap.GetValue(),
            'min_turning_radius': self.min_turning_radius.GetValue(),
            'max_setback_angle': self.max_setback_angle.GetValue(),
            'max_turn_angle': self.max_turn_angle.GetValue(),
            'diff_chamfer_extra': self.chamfer_extra.GetValue(),
            'fix_polarity': self.fix_polarity_check.GetValue(),
            'gnd_via_enabled': self.gnd_via_check.GetValue(),
            'diff_pair_intra_match': self.intra_match_check.GetValue(),
        }

    def _on_use_netclass_changed(self, event):
        """Handle the 'Use net class definitions' checkbox toggle."""
        use_netclass = self.use_netclass_check.GetValue()

        if use_netclass:
            # Try to get net class from first selected pair, fall back to Default
            class_name = self._get_selected_pair_netclass() or 'Default'
            from .swig_gui import _get_netclass_parameters
            params = _get_netclass_parameters(class_name)
            if params:
                # Update diff pair width and gap from net class
                if 'diff_pair_width' in params and params['diff_pair_width'] > 0:
                    self.diff_pair_width.SetValue(params['diff_pair_width'])
                if 'diff_pair_gap' in params and params['diff_pair_gap'] > 0:
                    self.diff_pair_gap.SetValue(params['diff_pair_gap'])
            # Disable manual editing
            self.diff_pair_width.Enable(False)
            self.diff_pair_gap.Enable(False)
        else:
            # Re-enable manual editing
            self.diff_pair_width.Enable(True)
            self.diff_pair_gap.Enable(True)

    def _get_selected_pair_netclass(self):
        """Get the net class name for the first selected pair, or None."""
        try:
            import pcbnew
            board = pcbnew.GetBoard()
            if board is None:
                return None

            # Get first selected pair's P net
            selected = self.get_selected_pair_net_ids()
            if not selected:
                return None

            _, p_net_id, _ = selected[0]
            net = board.FindNet(p_net_id)
            if net:
                return net.GetNetClassName()
            return None
        except Exception:
            return None

    def get_selected_pairs(self):
        """Get list of selected differential pair base names."""
        return self.pair_panel.get_selected_pairs()

    def get_selected_pair_net_ids(self):
        """Get list of (base_name, p_net_id, n_net_id) for selected pairs."""
        return self.pair_panel.get_selected_pair_net_ids()
