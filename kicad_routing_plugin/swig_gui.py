"""
KiCad Routing Tools - wxPython GUI for SWIG plugin

Provides a wx-based dialog for routing configuration.
"""

import os
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


class StdoutRedirector:
    """Redirects stdout to a callback function while preserving original output."""

    def __init__(self, callback, original_stdout):
        self.callback = callback
        self.original = original_stdout

    def write(self, text):
        if text:
            # Write to original stdout
            if self.original:
                self.original.write(text)
            # Also send to callback
            self.callback(text)

    def flush(self):
        if self.original:
            self.original.flush()


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


class RoutingDialog(wx.Dialog):
    """Main dialog for configuring and running the router."""

    def __init__(self, parent, pcb_data, board_filename):
        super().__init__(
            parent,
            title="KiCad Routing Tools",
            size=(800, 800),
            style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER
        )

        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self._cancel_requested = False
        self._routing_thread = None
        self._connectivity_cache = {}  # Cache: net_id -> is_connected

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
        """Create the dialog UI with tabs for Configure and Log."""
        main_panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Create notebook for tabs
        self.notebook = wx.Notebook(main_panel)

        # Tab 1: Configure
        config_panel = self._create_config_tab()
        self.notebook.AddPage(config_panel, "Configure")

        # Tab 2: Log
        log_panel = self._create_log_tab()
        self.notebook.AddPage(log_panel, "Log")

        # Add notebook to main sizer
        main_sizer.Add(self.notebook, 1, wx.EXPAND | wx.ALL, 5)
        main_panel.SetSizer(main_sizer)

    def _create_config_tab(self):
        """Create the Configure tab with all routing options."""
        panel = wx.Panel(self.notebook)
        config_sizer = wx.BoxSizer(wx.VERTICAL)
        h_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # Left side: Net selection
        net_sizer = self._create_net_selection_panel(panel)
        h_sizer.Add(net_sizer, 3, wx.EXPAND | wx.ALL, 5)

        # Right side: Parameters, layers, options, progress, buttons
        right_sizer = wx.BoxSizer(wx.VERTICAL)
        right_sizer.Add(self._create_parameters_panel(panel), 2, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self._create_layers_panel(panel), 1, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self._create_options_panel(panel), 1, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self._create_progress_panel(panel), 0, wx.EXPAND | wx.BOTTOM, 5)
        right_sizer.Add(self._create_buttons_panel(panel), 0, wx.EXPAND)
        h_sizer.Add(right_sizer, 2, wx.EXPAND | wx.ALL, 5)

        config_sizer.Add(h_sizer, 1, wx.EXPAND | wx.ALL, 5)
        panel.SetSizer(config_sizer)
        return panel

    def _create_net_selection_panel(self, panel):
        """Create the net selection panel (left side)."""
        net_box = wx.StaticBox(panel, label="Net Selection")
        net_sizer = wx.StaticBoxSizer(net_box, wx.VERTICAL)

        # Hide connected checkbox
        self.hide_connected_check = wx.CheckBox(panel, label="Hide connected")
        self.hide_connected_check.SetValue(True)
        self.hide_connected_check.SetToolTip("Hide nets that are already fully connected")
        self.hide_connected_check.Bind(wx.EVT_CHECKBOX, self._on_filter_changed)
        net_sizer.Add(self.hide_connected_check, 0, wx.LEFT | wx.RIGHT | wx.TOP, 5)

        # Filter
        filter_sizer = wx.BoxSizer(wx.HORIZONTAL)
        filter_sizer.Add(wx.StaticText(panel, label="Filter:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.filter_ctrl = wx.TextCtrl(panel)
        self.filter_ctrl.Bind(wx.EVT_TEXT, self._on_filter_changed)
        filter_sizer.Add(self.filter_ctrl, 1, wx.EXPAND)
        net_sizer.Add(filter_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Net list
        self.net_list = wx.CheckListBox(panel, size=(200, -1), style=wx.LB_EXTENDED)
        self.net_list.Bind(wx.EVT_KEY_DOWN, self._on_net_list_key)
        net_sizer.Add(self.net_list, 1, wx.EXPAND | wx.ALL, 5)

        # Select/Unselect buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.select_btn = wx.Button(panel, label="Select")
        self.select_btn.Bind(wx.EVT_BUTTON, self._on_select)
        self.unselect_btn = wx.Button(panel, label="Unselect")
        self.unselect_btn.Bind(wx.EVT_BUTTON, self._on_unselect)
        btn_sizer.Add(self.select_btn, 1, wx.RIGHT, 5)
        btn_sizer.Add(self.unselect_btn, 1)
        net_sizer.Add(btn_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        return net_sizer

    def _create_parameters_panel(self, panel):
        """Create the parameters panel with basic and advanced settings."""
        param_box = wx.StaticBox(panel, label="Parameters")
        param_box_sizer = wx.StaticBoxSizer(param_box, wx.VERTICAL)
        param_scroll = wx.ScrolledWindow(panel, style=wx.VSCROLL)
        param_scroll.SetScrollRate(0, 10)
        param_inner = wx.BoxSizer(wx.VERTICAL)

        # Basic parameters grid
        param_grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        param_grid.AddGrowableCol(1)
        self._add_basic_parameters(param_scroll, param_grid)
        param_inner.Add(param_grid, 0, wx.EXPAND | wx.ALL, 5)

        # Advanced section header
        adv_label = wx.StaticText(param_scroll, label="Advanced")
        adv_label.SetFont(adv_label.GetFont().Bold())
        param_inner.Add(adv_label, 0, wx.LEFT | wx.TOP, 5)
        param_inner.Add(wx.StaticLine(param_scroll), 0, wx.EXPAND | wx.ALL, 5)

        # Advanced parameters grid
        adv_grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        adv_grid.AddGrowableCol(1)
        self._add_advanced_parameters(param_scroll, adv_grid)
        param_inner.Add(adv_grid, 0, wx.EXPAND | wx.ALL, 5)

        param_scroll.SetSizer(param_inner)
        param_box_sizer.Add(param_scroll, 1, wx.EXPAND)
        return param_box_sizer

    def _add_basic_parameters(self, parent, grid):
        """Add basic parameter controls to grid."""
        params = [
            ('track_width', 'Track Width (mm):', defaults.TRACK_WIDTH),
            ('clearance', 'Clearance (mm):', defaults.CLEARANCE),
            ('via_size', 'Via Size (mm):', defaults.VIA_SIZE),
            ('via_drill', 'Via Drill (mm):', defaults.VIA_DRILL),
            ('grid_step', 'Grid Step (mm):', defaults.GRID_STEP),
        ]
        for name, label, default in params:
            r = defaults.PARAM_RANGES[name]
            grid.Add(wx.StaticText(parent, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            ctrl = wx.SpinCtrlDouble(parent, min=r['min'], max=r['max'], initial=default, inc=r['inc'])
            ctrl.SetDigits(r['digits'])
            setattr(self, name, ctrl)
            grid.Add(ctrl, 0, wx.EXPAND)

        # Via cost (integer)
        r = defaults.PARAM_RANGES['via_cost']
        grid.Add(wx.StaticText(parent, label="Via Cost:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.via_cost = wx.SpinCtrl(parent, min=r['min'], max=r['max'], initial=defaults.VIA_COST)
        grid.Add(self.via_cost, 0, wx.EXPAND)

    def _add_advanced_parameters(self, parent, grid):
        """Add advanced parameter controls to grid."""
        # Integer parameters
        int_params = [
            ('max_iterations', 'Max Iterations:', defaults.MAX_ITERATIONS),
            ('turn_cost', 'Turn Cost:', defaults.TURN_COST),
            ('max_ripup', 'Max Rip-up Count:', defaults.MAX_RIPUP),
        ]
        for name, label, default in int_params:
            r = defaults.PARAM_RANGES[name]
            grid.Add(wx.StaticText(parent, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            ctrl = wx.SpinCtrl(parent, min=r['min'], max=r['max'], initial=default)
            setattr(self, name, ctrl)
            grid.Add(ctrl, 0, wx.EXPAND)

        # Heuristic weight
        r = defaults.PARAM_RANGES['heuristic_weight']
        grid.Add(wx.StaticText(parent, label="Heuristic Weight:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.heuristic_weight = wx.SpinCtrlDouble(parent, min=r['min'], max=r['max'], initial=defaults.HEURISTIC_WEIGHT, inc=r['inc'])
        self.heuristic_weight.SetDigits(r['digits'])
        grid.Add(self.heuristic_weight, 0, wx.EXPAND)

        # Ordering strategy
        grid.Add(wx.StaticText(parent, label="Ordering Strategy:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.ordering_strategy = wx.Choice(parent, choices=["mps", "inside_out", "original"])
        self.ordering_strategy.SetSelection(0)
        grid.Add(self.ordering_strategy, 0, wx.EXPAND)

        # Float parameters
        float_params = [
            ('stub_proximity_radius', 'Stub Proximity (mm):', defaults.STUB_PROXIMITY_RADIUS),
            ('stub_proximity_cost', 'Stub Prox. Cost:', defaults.STUB_PROXIMITY_COST),
            ('via_proximity_cost', 'Via Prox. Cost:', defaults.VIA_PROXIMITY_COST),
            ('track_proximity_distance', 'Track Prox. (mm):', defaults.TRACK_PROXIMITY_DISTANCE),
            ('track_proximity_cost', 'Track Prox. Cost:', defaults.TRACK_PROXIMITY_COST),
            ('routing_clearance_margin', 'Clearance Margin:', defaults.ROUTING_CLEARANCE_MARGIN),
            ('hole_to_hole_clearance', 'Hole Clearance (mm):', defaults.HOLE_TO_HOLE_CLEARANCE),
            ('board_edge_clearance', 'Edge Clearance (mm):', defaults.BOARD_EDGE_CLEARANCE),
        ]
        for name, label, default in float_params:
            r = defaults.PARAM_RANGES[name]
            grid.Add(wx.StaticText(parent, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            ctrl = wx.SpinCtrlDouble(parent, min=r['min'], max=r['max'], initial=default, inc=r['inc'])
            ctrl.SetDigits(r['digits'])
            setattr(self, name, ctrl)
            grid.Add(ctrl, 0, wx.EXPAND)

        # Layer switching checkbox
        grid.Add(wx.StaticText(parent, label="Layer Switching:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.enable_layer_switch = wx.CheckBox(parent)
        self.enable_layer_switch.SetValue(True)
        grid.Add(self.enable_layer_switch, 0, wx.EXPAND)

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
            cb.SetValue(True)
            self.layer_checks[layer] = cb
            layer_inner.Add(cb, 0, wx.ALL, 3)

        layer_scroll.SetSizer(layer_inner)
        layer_box_sizer.Add(layer_scroll, 1, wx.EXPAND)
        return layer_box_sizer

    def _create_options_panel(self, panel):
        """Create the options panel."""
        options_box = wx.StaticBox(panel, label="Options")
        options_box_sizer = wx.StaticBoxSizer(options_box, wx.VERTICAL)
        options_scroll = wx.ScrolledWindow(panel, style=wx.VSCROLL)
        options_scroll.SetScrollRate(0, 10)
        options_inner = wx.BoxSizer(wx.VERTICAL)

        self.move_text_check = wx.CheckBox(options_scroll, label="Move copper text to silkscreen")
        self.move_text_check.SetValue(True)
        self.move_text_check.SetToolTip("Move gr_text from copper layers to silkscreen to prevent routing interference")
        options_inner.Add(self.move_text_check, 0, wx.ALL, 3)

        self.debug_lines_check = wx.CheckBox(options_scroll, label="Add debug visualization lines")
        self.debug_lines_check.SetValue(False)
        self.debug_lines_check.SetToolTip("Add routing paths to User layers for debugging")
        options_inner.Add(self.debug_lines_check, 0, wx.ALL, 3)

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
        self.route_btn.Bind(wx.EVT_BUTTON, self._on_route)
        button_sizer.Add(self.route_btn, 1, wx.RIGHT, 5)

        self.cancel_btn = wx.Button(panel, wx.ID_CANCEL, label="Close")
        button_sizer.Add(self.cancel_btn, 1)

        return button_sizer

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
        clear_log_btn.Bind(wx.EVT_BUTTON, self._on_clear_log)
        log_sizer.Add(clear_log_btn, 0, wx.ALIGN_RIGHT | wx.RIGHT | wx.BOTTOM, 5)

        log_panel.SetSizer(log_sizer)
        return log_panel

    def _load_nets_immediate(self):
        """Load net names from PCB data (fast, no connectivity check)."""
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

        # Show all nets initially (before connectivity check)
        self.net_list.Clear()
        for name, net_id in self.all_nets:
            self.net_list.Append(name)
        self.status_text.SetLabel("Loading...")

    def _deferred_init(self):
        """Run after dialog is shown: sync board and check connectivity."""
        # Sync pcb_data with pcbnew's in-memory board state
        self.status_text.SetLabel("Syncing with board...")
        wx.Yield()
        self._sync_pcb_data_from_board()

        # Now update net list with connectivity check
        self._update_net_list()

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

    def _update_net_list(self):
        """Update the net list based on filter and hide_connected setting."""
        filter_text = self.filter_ctrl.GetValue().lower()
        hide_connected = self.hide_connected_check.GetValue()

        # Filter by text first (fast)
        filtered_nets = [(name, net_id) for name, net_id in self.all_nets
                         if filter_text in name.lower()]

        # Start with all filtered nets in the list
        self.net_list.Clear()
        for name, net_id in filtered_nets:
            self.net_list.Append(name)

        # Check which nets need connectivity check (not in cache)
        uncached_nets = [(name, net_id) for name, net_id in filtered_nets
                         if net_id not in self._connectivity_cache]

        # If we have uncached nets, run connectivity check with progress
        if uncached_nets:
            self.progress_bar.SetRange(len(uncached_nets))
            self.progress_bar.SetValue(0)

            for i, (name, net_id) in enumerate(uncached_nets):
                is_connected = self._is_net_connected(net_id)
                self._connectivity_cache[net_id] = is_connected

                # If hiding connected and this net is connected, remove it from list
                if hide_connected and is_connected:
                    idx = self.net_list.FindString(name)
                    if idx != wx.NOT_FOUND:
                        self.net_list.Delete(idx)

                self.progress_bar.SetValue(i + 1)
                remaining = self.net_list.GetCount()
                self.status_text.SetLabel(f"Checking connectivity... {i + 1}/{len(uncached_nets)} ({remaining} to route)")
                wx.Yield()

        # If all nets were cached, still need to filter if hide_connected
        if not uncached_nets and hide_connected:
            # Remove connected nets using cache (fast, no progress needed)
            for name, net_id in filtered_nets:
                if self._connectivity_cache.get(net_id, False):
                    idx = self.net_list.FindString(name)
                    if idx != wx.NOT_FOUND:
                        self.net_list.Delete(idx)

        # Count connected for status
        connected_count = sum(1 for name, net_id in filtered_nets
                              if self._connectivity_cache.get(net_id, False))
        remaining = self.net_list.GetCount()
        if hide_connected:
            self.status_text.SetLabel(f"Ready - {remaining} nets to route ({connected_count} connected)")
        else:
            self.status_text.SetLabel(f"Ready - {remaining} nets")
        self.progress_bar.SetValue(0)

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

    def _on_clear_log(self, event):
        """Clear the log text control."""
        self.log_text.Clear()

    def _append_log(self, text):
        """Append text to the log (thread-safe via CallAfter)."""
        wx.CallAfter(self._do_append_log, text)

    def _do_append_log(self, text):
        """Actually append text to log (must be called on main thread)."""
        self.log_text.AppendText(text)

    def _get_selected_nets(self):
        """Get list of selected net names."""
        selected = []
        for i in range(self.net_list.GetCount()):
            if self.net_list.IsChecked(i):
                selected.append(self.net_list.GetString(i))
        return selected

    def _get_selected_layers(self):
        """Get list of selected layers."""
        return [layer for layer, cb in self.layer_checks.items() if cb.GetValue()]

    def _on_route(self, event):
        """Handle route button click."""
        selected_nets = self._get_selected_nets()
        if not selected_nets:
            wx.MessageBox(
                "Please select at least one net to route.",
                "No Nets Selected",
                wx.OK | wx.ICON_WARNING
            )
            return

        selected_layers = self._get_selected_layers()
        if not selected_layers:
            wx.MessageBox(
                "Please select at least one layer.",
                "No Layers Selected",
                wx.OK | wx.ICON_WARNING
            )
            return

        # Disable UI during routing
        self.route_btn.Disable()
        self._cancel_requested = False
        self._routing_start_time = time.time()  # Track wall time from button press

        # Get parameters
        config = {
            'nets': selected_nets,
            'layers': selected_layers,
            'track_width': self.track_width.GetValue(),
            'clearance': self.clearance.GetValue(),
            'via_size': self.via_size.GetValue(),
            'via_drill': self.via_drill.GetValue(),
            'grid_step': self.grid_step.GetValue(),
            'via_cost': self.via_cost.GetValue(),
            'move_copper_text': self.move_text_check.GetValue(),
            'debug_lines': self.debug_lines_check.GetValue(),
            # Advanced parameters
            'max_iterations': self.max_iterations.GetValue(),
            'heuristic_weight': self.heuristic_weight.GetValue(),
            'turn_cost': self.turn_cost.GetValue(),
            'max_ripup': self.max_ripup.GetValue(),
            'ordering_strategy': self.ordering_strategy.GetString(self.ordering_strategy.GetSelection()),
            'stub_proximity_radius': self.stub_proximity_radius.GetValue(),
            'stub_proximity_cost': self.stub_proximity_cost.GetValue(),
            'via_proximity_cost': self.via_proximity_cost.GetValue(),
            'track_proximity_distance': self.track_proximity_distance.GetValue(),
            'track_proximity_cost': self.track_proximity_cost.GetValue(),
            'routing_clearance_margin': self.routing_clearance_margin.GetValue(),
            'hole_to_hole_clearance': self.hole_to_hole_clearance.GetValue(),
            'board_edge_clearance': self.board_edge_clearance.GetValue(),
            'enable_layer_switch': self.enable_layer_switch.GetValue(),
        }

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
                from route import batch_route
            except SystemExit as e:
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
                else:
                    raise RuntimeError(f"Startup check failed: {e}")

            def check_cancel():
                return self._cancel_requested

            def on_progress(current, total, net_name=""):
                wx.CallAfter(self._update_progress, current, total, net_name)

            # Run the router with return_results=True to get track data
            successful, failed, total_time, results_data = batch_route(
                input_file=self.board_filename,
                output_file="",  # Not used when return_results=True
                net_names=config['nets'],
                layers=config['layers'],
                track_width=config['track_width'],
                clearance=config['clearance'],
                via_size=config['via_size'],
                via_drill=config['via_drill'],
                grid_step=config['grid_step'],
                via_cost=config['via_cost'],
                max_iterations=config['max_iterations'],
                heuristic_weight=config['heuristic_weight'],
                turn_cost=config['turn_cost'],
                max_rip_up_count=config['max_ripup'],
                ordering_strategy=config['ordering_strategy'],
                stub_proximity_radius=config['stub_proximity_radius'],
                stub_proximity_cost=config['stub_proximity_cost'],
                via_proximity_cost=config['via_proximity_cost'],
                track_proximity_distance=config['track_proximity_distance'],
                track_proximity_cost=config['track_proximity_cost'],
                routing_clearance_margin=config['routing_clearance_margin'],
                hole_to_hole_clearance=config['hole_to_hole_clearance'],
                board_edge_clearance=config['board_edge_clearance'],
                enable_layer_switch=config['enable_layer_switch'],
                debug_lines=config['debug_lines'],
                cancel_check=check_cancel,
                progress_callback=on_progress,
                return_results=True,
                pcb_data=self.pcb_data,
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

        # Add debug visualization lines if enabled
        if config.get('debug_lines', False):
            debug_lines_added = self._add_debug_lines(board, results_data)

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

        # Refresh net list to hide newly connected nets
        self._update_net_list()

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
        self.status_text.SetLabel("Cancelled")

    def _routing_error(self, error_msg):
        """Handle routing error."""
        self.status_text.SetLabel("Error")
        wx.MessageBox(
            f"Routing error:\n\n{error_msg}",
            "Routing Error",
            wx.OK | wx.ICON_ERROR
        )
