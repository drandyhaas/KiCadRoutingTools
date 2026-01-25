"""
KiCad Routing Tools - wxPython GUI for SWIG plugin

Provides a wx-based dialog for routing configuration.
"""

import os
import sys
import wx
import threading

# Add parent directory to path
PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)


class RoutingDialog(wx.Dialog):
    """Main dialog for configuring and running the router."""

    def __init__(self, parent, pcb_data, board_filename):
        super().__init__(
            parent,
            title="KiCad Routing Tools",
            size=(600, 700),
            style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER
        )

        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self._cancel_requested = False
        self._routing_thread = None

        self._create_ui()
        self._load_nets()
        self.Centre()

    def _create_ui(self):
        """Create the dialog UI."""
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Net selection
        net_box = wx.StaticBox(panel, label="Net Selection")
        net_sizer = wx.StaticBoxSizer(net_box, wx.VERTICAL)

        # Filter
        filter_sizer = wx.BoxSizer(wx.HORIZONTAL)
        filter_sizer.Add(wx.StaticText(panel, label="Filter:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.filter_ctrl = wx.TextCtrl(panel)
        self.filter_ctrl.Bind(wx.EVT_TEXT, self._on_filter_changed)
        filter_sizer.Add(self.filter_ctrl, 1, wx.EXPAND)
        net_sizer.Add(filter_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Net list
        self.net_list = wx.CheckListBox(panel, size=(-1, 150))
        net_sizer.Add(self.net_list, 1, wx.EXPAND | wx.ALL, 5)

        # Select all/none buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.select_all_btn = wx.Button(panel, label="Select All")
        self.select_all_btn.Bind(wx.EVT_BUTTON, self._on_select_all)
        self.select_none_btn = wx.Button(panel, label="Select None")
        self.select_none_btn.Bind(wx.EVT_BUTTON, self._on_select_none)
        btn_sizer.Add(self.select_all_btn, 0, wx.RIGHT, 5)
        btn_sizer.Add(self.select_none_btn, 0)
        net_sizer.Add(btn_sizer, 0, wx.ALL, 5)

        main_sizer.Add(net_sizer, 1, wx.EXPAND | wx.ALL, 10)

        # Routing parameters
        param_box = wx.StaticBox(panel, label="Routing Parameters")
        param_sizer = wx.StaticBoxSizer(param_box, wx.VERTICAL)
        param_grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        param_grid.AddGrowableCol(1)

        # Track width
        param_grid.Add(wx.StaticText(panel, label="Track Width (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.track_width = wx.SpinCtrlDouble(panel, min=0.05, max=5.0, initial=0.3, inc=0.05)
        self.track_width.SetDigits(2)
        param_grid.Add(self.track_width, 0, wx.EXPAND)

        # Clearance
        param_grid.Add(wx.StaticText(panel, label="Clearance (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.clearance = wx.SpinCtrlDouble(panel, min=0.05, max=5.0, initial=0.25, inc=0.05)
        self.clearance.SetDigits(2)
        param_grid.Add(self.clearance, 0, wx.EXPAND)

        # Via size
        param_grid.Add(wx.StaticText(panel, label="Via Size (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.via_size = wx.SpinCtrlDouble(panel, min=0.2, max=2.0, initial=0.5, inc=0.05)
        self.via_size.SetDigits(2)
        param_grid.Add(self.via_size, 0, wx.EXPAND)

        # Via drill
        param_grid.Add(wx.StaticText(panel, label="Via Drill (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.via_drill = wx.SpinCtrlDouble(panel, min=0.1, max=1.5, initial=0.3, inc=0.05)
        self.via_drill.SetDigits(2)
        param_grid.Add(self.via_drill, 0, wx.EXPAND)

        # Grid step
        param_grid.Add(wx.StaticText(panel, label="Grid Step (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.grid_step = wx.SpinCtrlDouble(panel, min=0.01, max=1.0, initial=0.05, inc=0.01)
        self.grid_step.SetDigits(3)
        param_grid.Add(self.grid_step, 0, wx.EXPAND)

        # Via cost
        param_grid.Add(wx.StaticText(panel, label="Via Cost:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.via_cost = wx.SpinCtrl(panel, min=1, max=1000, initial=80)
        param_grid.Add(self.via_cost, 0, wx.EXPAND)

        param_sizer.Add(param_grid, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(param_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 10)

        # Layer selection
        layer_box = wx.StaticBox(panel, label="Layers")
        layer_sizer = wx.StaticBoxSizer(layer_box, wx.HORIZONTAL)

        self.layer_checks = {}
        for layer in self.pcb_data.board_info.copper_layers:
            cb = wx.CheckBox(panel, label=layer)
            cb.SetValue(True)
            self.layer_checks[layer] = cb
            layer_sizer.Add(cb, 0, wx.ALL, 5)

        main_sizer.Add(layer_sizer, 0, wx.EXPAND | wx.ALL, 10)

        # Options
        options_box = wx.StaticBox(panel, label="Options")
        options_sizer = wx.StaticBoxSizer(options_box, wx.VERTICAL)

        self.move_text_check = wx.CheckBox(panel, label="Move copper text to silkscreen")
        self.move_text_check.SetValue(True)
        self.move_text_check.SetToolTip("Move gr_text from copper layers to silkscreen to prevent routing interference")
        options_sizer.Add(self.move_text_check, 0, wx.ALL, 5)

        self.debug_lines_check = wx.CheckBox(panel, label="Add debug visualization lines")
        self.debug_lines_check.SetValue(False)
        self.debug_lines_check.SetToolTip("Add routing paths to User layers for debugging")
        options_sizer.Add(self.debug_lines_check, 0, wx.ALL, 5)

        main_sizer.Add(options_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 10)

        # Progress
        progress_box = wx.StaticBox(panel, label="Progress")
        progress_sizer = wx.StaticBoxSizer(progress_box, wx.VERTICAL)

        self.progress_bar = wx.Gauge(panel, range=100)
        progress_sizer.Add(self.progress_bar, 0, wx.EXPAND | wx.ALL, 5)

        self.status_text = wx.StaticText(panel, label="Ready")
        progress_sizer.Add(self.status_text, 0, wx.EXPAND | wx.ALL, 5)

        main_sizer.Add(progress_sizer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT, 10)

        # Buttons
        button_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.route_btn = wx.Button(panel, label="Route")
        self.route_btn.Bind(wx.EVT_BUTTON, self._on_route)
        button_sizer.Add(self.route_btn, 0, wx.RIGHT, 10)

        self.cancel_btn = wx.Button(panel, wx.ID_CANCEL, label="Close")
        button_sizer.Add(self.cancel_btn, 0)

        main_sizer.Add(button_sizer, 0, wx.ALIGN_CENTER | wx.ALL, 10)

        panel.SetSizer(main_sizer)

    def _load_nets(self):
        """Load nets from PCB data."""
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

        # Populate list
        self._update_net_list()

    def _update_net_list(self):
        """Update the net list based on filter."""
        filter_text = self.filter_ctrl.GetValue().lower()

        self.net_list.Clear()
        for name, net_id in self.all_nets:
            if filter_text in name.lower():
                self.net_list.Append(name)

    def _on_filter_changed(self, event):
        """Handle filter text change."""
        self._update_net_list()

    def _on_select_all(self, event):
        """Select all visible nets."""
        for i in range(self.net_list.GetCount()):
            self.net_list.Check(i, True)

    def _on_select_none(self, event):
        """Deselect all nets."""
        for i in range(self.net_list.GetCount()):
            self.net_list.Check(i, False)

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
                    msg += "Install them in KiCad's Python with:\n"
                    msg += "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/pip3 install "
                    msg += " ".join(missing)
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
                max_iterations=200000,
                heuristic_weight=1.9,
                debug_lines=config['debug_lines'],
                cancel_check=check_cancel,
                progress_callback=on_progress,
                return_results=True,
            )

            if self._cancel_requested:
                wx.CallAfter(self._routing_cancelled)
            else:
                # Apply results to pcbnew on main thread
                wx.CallAfter(self._apply_results_to_board, results_data, successful, failed, total_time, config)

        except Exception as e:
            wx.CallAfter(self._routing_error, str(e))

    def _poll_routing(self):
        """Poll for routing thread completion."""
        if self._routing_thread and self._routing_thread.is_alive():
            wx.CallLater(100, self._poll_routing)
        else:
            self.route_btn.Enable()

    def _update_progress(self, current, total, net_name):
        """Update progress bar and status."""
        if total > 0:
            percent = int(100 * current / total)
            self.progress_bar.SetValue(percent)
        self.status_text.SetLabel(f"Routing: {net_name} ({current}/{total})")

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

        def get_layer_id(layer_name):
            """Convert layer name to pcbnew layer ID."""
            # Standard copper layer mapping (KiCad supports up to 32 copper layers)
            layer_ids = {
                'F.Cu': pcbnew.F_Cu,
                'B.Cu': pcbnew.B_Cu,
            }
            # Add inner copper layers In1.Cu through In30.Cu
            for i in range(1, 31):
                layer_ids[f'In{i}.Cu'] = getattr(pcbnew, f'In{i}_Cu', None)

            layer_id = layer_ids.get(layer_name)
            if layer_id is not None:
                return layer_id
            return pcbnew.F_Cu

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
                pcb_via = pcbnew.PCB_VIA(board)
                pcb_via.SetPosition(pcbnew.VECTOR2I(
                    pcbnew.FromMM(via.x),
                    pcbnew.FromMM(via.y)
                ))
                pcb_via.SetWidth(pcbnew.FromMM(via.size))
                pcb_via.SetDrill(pcbnew.FromMM(via.drill))
                pcb_via.SetNetCode(via.net_id)
                # Set via layers
                if hasattr(via, 'layers') and len(via.layers) >= 2:
                    top_layer = get_layer_id(via.layers[0])
                    bot_layer = get_layer_id(via.layers[1])
                    pcb_via.SetLayerPair(top_layer, bot_layer)
                board.Add(pcb_via)
                vias_added += 1

        # Add vias from layer swapping
        for via in results_data.get('all_swap_vias', []):
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
            vias_added += 1

        # Add debug visualization lines if enabled
        if config.get('debug_lines', False):
            debug_lines_added = self._add_debug_lines(board, results_data)

        # Refresh the view
        pcbnew.Refresh()

        # Update UI and show completion message
        self.progress_bar.SetValue(100)
        self.status_text.SetLabel(f"Complete: {successful} routed, {failed} failed")

        msg = f"Routing complete!\n\n"
        msg += f"Successfully routed: {successful} nets\n"
        msg += f"Failed: {failed}\n"
        msg += f"Time: {total_time:.1f}s\n\n"
        msg += f"Added to board:\n"
        msg += f"  {tracks_added} tracks\n"
        msg += f"  {vias_added} vias\n"
        if text_moved > 0:
            msg += f"  {text_moved} text items moved to silkscreen\n"
        if debug_lines_added > 0:
            msg += f"  {debug_lines_added} debug lines\n"
        msg += "\nUse Edit -> Undo to revert changes."

        wx.MessageBox(msg, "Routing Complete", wx.OK | wx.ICON_INFORMATION)

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
