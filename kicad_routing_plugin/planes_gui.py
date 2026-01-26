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


class PlaneAssignmentPanel(wx.Panel):
    """Panel for managing net-to-layer assignments for plane creation.

    Each assignment maps a group of nets (joined with "|") to one or more target layers.
    """

    def __init__(self, parent, pcb_data, get_selected_nets_callback):
        """
        Create a plane assignment panel.

        Args:
            parent: Parent window
            pcb_data: PCBData object with board info
            get_selected_nets_callback: Function that returns currently selected nets
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.get_selected_nets = get_selected_nets_callback
        self.assignments = []  # List of (nets_list, layers_list) tuples

        self._create_ui()

    def _create_ui(self):
        """Create the panel UI."""
        sizer = wx.BoxSizer(wx.VERTICAL)

        # Assignment list
        self.assignment_list = wx.ListBox(self, style=wx.LB_EXTENDED)
        self.assignment_list.SetToolTip("Net → Layer assignments. Select and click Remove to delete.")
        sizer.Add(self.assignment_list, 1, wx.EXPAND | wx.BOTTOM, 5)

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

    def __init__(self, parent):
        """Create the options panel."""
        super().__init__(parent)
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

        # Edge clearance
        grid.Add(wx.StaticText(self, label="Edge Clearance (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['plane_edge_clearance']
        self.edge_clearance = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                 initial=defaults.PLANE_EDGE_CLEARANCE, inc=r['inc'])
        self.edge_clearance.SetDigits(r['digits'])
        self.edge_clearance.SetToolTip("Clearance from zone to board edge")
        grid.Add(self.edge_clearance, 0, wx.EXPAND)

        # Max search radius
        grid.Add(wx.StaticText(self, label="Max Search Radius (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['plane_max_search_radius']
        self.max_search_radius = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                    initial=defaults.PLANE_MAX_SEARCH_RADIUS, inc=r['inc'])
        self.max_search_radius.SetDigits(r['digits'])
        self.max_search_radius.SetToolTip("Maximum radius to search for valid via placement")
        grid.Add(self.max_search_radius, 0, wx.EXPAND)

        zone_sizer.Add(grid, 0, wx.EXPAND | wx.ALL, 5)
        sizer.Add(zone_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Rip-up options
        ripup_box = wx.StaticBox(self, label="Blocker Handling")
        ripup_sizer = wx.StaticBoxSizer(ripup_box, wx.VERTICAL)

        self.rip_blocker_check = wx.CheckBox(self, label="Rip up blocking nets")
        self.rip_blocker_check.SetToolTip("Remove nets that block via placement, then retry (uses Max Rip-up from Basic tab)")
        ripup_sizer.Add(self.rip_blocker_check, 0, wx.ALL, 5)

        self.reroute_ripped_check = wx.CheckBox(self, label="Auto-reroute ripped nets")
        self.reroute_ripped_check.SetToolTip("Automatically re-route ripped nets after plane creation")
        ripup_sizer.Add(self.reroute_ripped_check, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        sizer.Add(ripup_sizer, 0, wx.EXPAND)

        self.SetSizer(sizer)

    def get_config(self):
        """Get the configuration values."""
        return {
            'zone_clearance': self.zone_clearance.GetValue(),
            'edge_clearance': self.edge_clearance.GetValue(),
            'max_search_radius': self.max_search_radius.GetValue(),
            'rip_blocker_nets': self.rip_blocker_check.GetValue(),
            'reroute_ripped_nets': self.reroute_ripped_check.GetValue(),
        }


class RepairPlanesOptionsPanel(wx.Panel):
    """Options panel for repairing disconnected planes (route_disconnected_planes.py)."""

    def __init__(self, parent):
        """Create the options panel."""
        super().__init__(parent)
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
        self.max_track_width.SetToolTip("Maximum track width for region connections")
        grid.Add(self.max_track_width, 0, wx.EXPAND)

        # Min track width
        grid.Add(wx.StaticText(self, label="Min Track Width (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['repair_min_track_width']
        self.min_track_width = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                  initial=defaults.REPAIR_MIN_TRACK_WIDTH, inc=r['inc'])
        self.min_track_width.SetDigits(r['digits'])
        self.min_track_width.SetToolTip("Minimum track width for region connections")
        grid.Add(self.min_track_width, 0, wx.EXPAND)

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

        # Info text
        info = wx.StaticText(self, label="Leave nets/layers empty to auto-detect existing zones.")
        info.SetForegroundColour(wx.Colour(100, 100, 100))
        sizer.Add(info, 0, wx.ALL, 5)

        self.SetSizer(sizer)

    def get_config(self):
        """Get the configuration values."""
        return {
            'max_track_width': self.max_track_width.GetValue(),
            'min_track_width': self.min_track_width.GetValue(),
            'analysis_grid_step': self.analysis_grid.GetValue(),
        }


class PlanesTab(wx.Panel):
    """Tab for copper plane creation and repair."""

    def __init__(self, parent, pcb_data, board_filename,
                 get_shared_params=None, on_planes_complete=None,
                 get_connectivity_check=None, append_log=None,
                 sync_pcb_data_callback=None):
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
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self.get_shared_params = get_shared_params
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
            get_selected_nets_callback=lambda: self.net_panel.get_selected_nets()
        )
        self.assign_sizer.Add(self.assignment_panel, 1, wx.EXPAND | wx.ALL, 5)

        right_sizer.Add(self.assign_sizer, 1, wx.EXPAND | wx.BOTTOM, 5)

        # Create options panel
        self.create_options = CreatePlanesOptionsPanel(self)
        right_sizer.Add(self.create_options, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Repair options panel (initially hidden)
        self.repair_options = RepairPlanesOptionsPanel(self)
        right_sizer.Add(self.repair_options, 0, wx.EXPAND | wx.BOTTOM, 5)
        self.repair_options.Hide()

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
        self.action_btn.Bind(wx.EVT_BUTTON, self._on_action)
        btn_sizer.Add(self.action_btn, 1, wx.RIGHT, 5)

        self.cancel_btn = wx.Button(self, label="Close")
        self.cancel_btn.Bind(wx.EVT_BUTTON, self._on_cancel_or_close)
        btn_sizer.Add(self.cancel_btn, 1)

        right_sizer.Add(btn_sizer, 0, wx.EXPAND)

        # Add spacer
        right_sizer.AddStretchSpacer(1)

        main_sizer.Add(right_sizer, 1, wx.EXPAND | wx.ALL, 5)

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
        self.Layout()

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

        # Redirect stdout to log
        class LogRedirector:
            def __init__(self, callback, original):
                self.callback = callback
                self.original = original

            def write(self, text):
                if text and self.original:
                    self.original.write(text)
                if text and self.callback:
                    self.callback(text)

            def flush(self):
                if self.original:
                    self.original.flush()

        original_stdout = sys.stdout
        if self.append_log:
            sys.stdout = LogRedirector(self.append_log, original_stdout)

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
        from route_planes import create_plane

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
        expanded_nets = []
        expanded_layers = []
        for nets_list, layers_list in assignments:
            for layer in layers_list:
                for net in nets_list:
                    expanded_nets.append(net)
                    expanded_layers.append(layer)

        print(f"\nCreating planes for {len(expanded_nets)} net/layer pairs:")
        for net, layer in zip(expanded_nets, expanded_layers):
            print(f"  {net} -> {layer}")

        if not expanded_nets:
            print("No net/layer assignments to process")
            return

        try:
            vias, traces, pads_needing, new_vias, new_segments = create_plane(
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
                board_edge_clearance=config.get('edge_clearance', defaults.PLANE_EDGE_CLEARANCE),
                all_layers=all_layers,
                dry_run=True,  # Don't write to file, apply via pcbnew
                rip_blocker_nets=config.get('rip_blocker_nets', False),
                max_rip_nets=config.get('max_ripup', defaults.MAX_RIPUP),
                reroute_ripped_nets=config.get('reroute_ripped_nets', False),
                pcb_data=self.pcb_data,
                return_results=True,
            )

            total_vias = vias
            total_traces = traces
            total_pads = pads_needing
            self._new_vias = new_vias
            self._new_segments = new_segments

        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"Error creating planes: {e}")

        self._operation_result = {
            'mode': 'create',
            'total_vias': total_vias,
            'total_traces': total_traces,
            'total_pads': total_pads,
            'cancelled': self._cancel_requested,
        }

    def _run_repair_planes(self, config):
        """Run disconnected plane repair."""
        from route_disconnected_planes import route_planes as repair_planes

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
            routes_added, regions_connected, new_vias, new_segments = repair_planes(
                input_file=self.board_filename,
                output_file="",
                net_names=net_names,
                plane_layers=plane_layers,
                track_width=config.get('track_width', defaults.TRACK_WIDTH),
                max_track_width=config.get('max_track_width', defaults.REPAIR_MAX_TRACK_WIDTH),
                min_track_width=config.get('min_track_width', defaults.REPAIR_MIN_TRACK_WIDTH),
                clearance=config.get('clearance', defaults.CLEARANCE),
                via_size=config.get('via_size', defaults.VIA_SIZE),
                via_drill=config.get('via_drill', defaults.VIA_DRILL),
                grid_step=config.get('grid_step', defaults.GRID_STEP),
                analysis_grid_step=config.get('analysis_grid_step', defaults.REPAIR_ANALYSIS_GRID_STEP),
                hole_to_hole_clearance=config.get('hole_to_hole_clearance', defaults.HOLE_TO_HOLE_CLEARANCE),
                max_iterations=config.get('max_iterations', defaults.MAX_ITERATIONS),
                routing_layers=all_layers,
                dry_run=True,  # Don't write to file, apply via pcbnew
                pcb_data=self.pcb_data,
                return_results=True,
            )

            self._new_vias = new_vias
            self._new_segments = new_segments
            self._operation_result = {
                'mode': 'repair',
                'routes_added': routes_added,
                'regions_connected': regions_connected,
                'cancelled': self._cancel_requested,
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
            self.status_text.SetLabel(f"Created: {result.get('total_vias', 0)} vias, {result.get('total_traces', 0)} traces")
        else:
            msg = f"Plane repair complete!\n\n"
            msg += f"Routes added: {result.get('routes_added', 0)}\n"
            msg += f"Regions connected: {result.get('regions_connected', 0)}\n"
            self.status_text.SetLabel(f"Repaired: {result.get('routes_added', 0)} routes")

        msg += "\nUse Edit -> Undo to revert changes."
        wx.MessageBox(msg, "Operation Complete", wx.OK | wx.ICON_INFORMATION)

        # Callback
        if self.on_planes_complete:
            self.on_planes_complete()

        # Refresh net list
        self.net_panel.refresh()

    def _apply_results_to_board(self):
        """Apply operation results to the pcbnew board."""
        import pcbnew

        board = pcbnew.GetBoard()
        if board is None:
            return

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
                    pcbnew.FromMM(via_data['x']),
                    pcbnew.FromMM(via_data['y'])
                ))
                via.SetDrill(pcbnew.FromMM(via_data['drill']))
                via.SetWidth(pcbnew.FromMM(via_data['size']))
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
                track = pcbnew.PCB_TRACK(board)
                start = seg_data['start']
                end = seg_data['end']
                track.SetStart(pcbnew.VECTOR2I(
                    pcbnew.FromMM(start[0]),
                    pcbnew.FromMM(start[1])
                ))
                track.SetEnd(pcbnew.VECTOR2I(
                    pcbnew.FromMM(end[0]),
                    pcbnew.FromMM(end[1])
                ))
                track.SetWidth(pcbnew.FromMM(seg_data['width']))
                track.SetLayer(get_layer_id(seg_data['layer']))
                track.SetNetCode(seg_data['net_id'])
                board.Add(track)
                tracks_added += 1
            self._new_segments = []

        if vias_added > 0 or tracks_added > 0:
            print(f"Added to board: {vias_added} vias, {tracks_added} tracks")

        # Build connectivity and refresh
        board.BuildConnectivity()
        pcbnew.Refresh()

        # Sync pcb_data
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()

    def get_assignments(self):
        """Get list of net→layer assignments."""
        return self.assignment_panel.get_assignments()
