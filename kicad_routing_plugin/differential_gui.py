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


def parse_diff_pairs_result(value):
    """Parse an /identify-diff-pairs RESULT line (issue #40).

    Format: sections separated by ';', each '<kind>:<comma-separated items>'.
      confirm: pair base names verified as differential by pin function
      reject:  base names that name-matching paired but are NOT differential
      custom:  P|N net-name pairs found by pin function that don't follow
               P/N naming conventions (not yet representable in the GUI)
    e.g. 'confirm:/CLK,/DATA;reject:/FOO;custom:/TXP|/TXN'

    Returns (dict with confirm/reject/custom lists, notes).
    """
    parsed = {"confirm": [], "reject": [], "custom": []}
    notes = []
    for section in str(value).split(";"):
        section = section.strip()
        if not section:
            continue
        kind, sep, items = section.partition(":")
        kind = kind.strip().lower()
        if not sep or kind not in parsed:
            notes.append(f"diff pairs: unparseable section {section!r}")
            continue
        for item in items.split(","):
            item = item.strip()
            if not item:
                continue
            if kind == "custom":
                p, sep2, n = item.partition("|")
                if sep2 and p.strip() and n.strip():
                    parsed["custom"].append((p.strip(), n.strip()))
                else:
                    notes.append(f"diff pairs: unparseable custom pair {item!r}")
            else:
                parsed[kind].append(item)
    return parsed, notes


class DiffPairSelectionPanel(wx.Panel):
    """Panel for selecting differential pairs to route."""

    def __init__(self, parent, pcb_data, instructions=None,
                 show_hide_checkbox=True, show_component_dropdown=True,
                 on_ask_claude=None):
        """
        Create a differential pair selection panel.

        Args:
            parent: Parent window
            pcb_data: PCBData object with nets
            instructions: Optional instruction text
            show_hide_checkbox: Whether to show the hide checkbox
            show_component_dropdown: Whether to show the component dropdown
            on_ask_claude: Callback for the "Ask Claude" button that verifies
                pairs by pin function (issue #40); button hidden if None.
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.on_ask_claude = on_ask_claude
        self.all_pairs = []  # List of (display_name, base_name, p_net_id, n_net_id)
        self._checked_pairs = set()
        self._check_fn = None
        self._suspend_check = False  # Temporarily disable check_fn during restore
        self._component_filter_value = ""
        self._hide_short = False      # hide electrically-short (deferred) pairs
        self._short_check_fn = None   # fn(p_net_id, n_net_id) -> bool

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
        self.filter_ctrl.SetToolTip("Filter differential pairs by name (case-insensitive)")
        self.filter_ctrl.Bind(wx.EVT_TEXT, self._on_filter_changed)
        filter_sizer.Add(self.filter_ctrl, 1, wx.EXPAND)
        sizer.Add(filter_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Pair list
        self.pair_list = wx.CheckListBox(self, size=(200, -1), style=wx.LB_EXTENDED)
        self.pair_list.SetToolTip("Check differential pairs to route (Ctrl+A to select all highlighted)")
        self.pair_list.Bind(wx.EVT_KEY_DOWN, self._on_list_key)
        sizer.Add(self.pair_list, 1, wx.EXPAND | wx.ALL, 5)

        # Select/Unselect buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        select_btn = wx.Button(self, label="Select")
        select_btn.SetToolTip("Check all highlighted pairs")
        select_btn.Bind(wx.EVT_BUTTON, self._on_select)
        unselect_btn = wx.Button(self, label="Unselect")
        unselect_btn.SetToolTip("Uncheck all highlighted pairs")
        unselect_btn.Bind(wx.EVT_BUTTON, self._on_unselect)
        btn_sizer.Add(select_btn, 1, wx.RIGHT, 5)
        btn_sizer.Add(unselect_btn, 1)
        if self.on_ask_claude is not None:
            ask_btn = wx.Button(self, label="Ask Claude", style=wx.BU_EXACTFIT)
            ask_btn.SetToolTip(
                "Run the /identify-diff-pairs skill: verifies pairs by pin "
                "function via datasheets, then checks confirmed pairs and "
                "unchecks name-matching false positives. Takes a few minutes.")
            ask_btn.Bind(wx.EVT_BUTTON, lambda event: self.on_ask_claude())
            btn_sizer.Add(ask_btn, 0, wx.LEFT, 5)
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
            # Check if should be hidden (electrically short -> deferred to
            # single-ended, so coupling it does nothing)
            if self._hide_short and self._short_check_fn:
                if self._short_check_fn(p_net_id, n_net_id):
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

    def set_short_check_function(self, fn):
        """Set the short-pair test: fn(p_net_id, n_net_id) -> bool."""
        self._short_check_fn = fn

    def set_hide_short(self, enabled):
        """Enable/disable hiding electrically-short (deferred) pairs, then refresh."""
        self._hide_short = bool(enabled)
        self._update_pair_list()

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

    def set_selected_pairs_by_net(self, net_names):
        """Pre-check differential pairs whose P or N net is in net_names.

        Replaces the current checked state with the pairs that have either
        their positive or negative net named in net_names, then refreshes.

        Args:
            net_names: Iterable of net name strings.
        """
        names = set(net_names)
        checked = set()
        for display_name, base_name, p_net_id, n_net_id in self.all_pairs:
            p_net = self.pcb_data.nets.get(p_net_id)
            n_net = self.pcb_data.nets.get(n_net_id)
            p_name = p_net.name if p_net else None
            n_name = n_net.name if n_net else None
            if p_name in names or n_name in names:
                checked.add(display_name)
        self._checked_pairs = checked
        self.refresh(sync_from_visible=False)

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
                 sync_pcb_data_callback=None, get_claude_params=None):
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
            get_claude_params: Callback returning the Claude tab's
                {'model', 'effort'} selections for headless runs
        """
        super().__init__(parent)
        self.pcb_data = pcb_data
        self.board_filename = board_filename
        self.get_shared_params = get_shared_params
        self.get_connectivity_check = get_connectivity_check
        self.get_routing_config = get_routing_config
        self.append_log = append_log
        self.sync_pcb_data_callback = sync_pcb_data_callback
        self.get_claude_params = get_claude_params
        self._routing_thread = None
        self._cancel_requested = False
        # Called when "Hide short routes" or a param affecting it changes, so the
        # main dialog can refresh the Basic tab's single-ended net list too.
        self.on_hide_short_changed = None
        self._short_cache = {}  # (params_key, p_net_id, n_net_id) -> bool

        self._create_ui()

        # Set up connectivity check
        if self.get_connectivity_check:
            self.pair_panel.set_check_function(self.get_connectivity_check())
        # Wire short-pair hiding (default on)
        self.pair_panel.set_short_check_function(self._is_short_pair)
        self.pair_panel.set_hide_short(self.hide_short_check.GetValue())
        # Pair Width / Gap / Centerline Setback change the short threshold, so a
        # change must re-evaluate which pairs are short.
        for ctrl in (self.diff_pair_width, self.diff_pair_gap, self.centerline_setback):
            ctrl.Bind(wx.EVT_SPINCTRLDOUBLE, self._on_short_param_changed)
            ctrl.Bind(wx.EVT_TEXT, self._on_short_param_changed)

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
            show_component_dropdown=True,
            on_ask_claude=self._on_ask_claude_diff_pairs
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

        # Differential impedance (optional). When > 0 the per-layer trace width
        # is computed from the board stackup using Pair Gap as spacing and
        # overrides Pair Width -- the GUI counterpart of route_diff.py --impedance.
        param_grid.Add(wx.StaticText(self, label="Diff Impedance (Ω):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.diff_impedance = wx.SpinCtrlDouble(self, min=0.0, max=200.0,
                                                initial=0.0, inc=5.0)
        self.diff_impedance.SetDigits(0)
        self.diff_impedance.SetToolTip(
            "Target differential impedance in ohms (e.g. 90, 100). When > 0, the "
            "per-layer trace width is derived from the board stackup using Pair "
            "Gap as spacing, overriding Pair Width. 0 = off (use Pair Width).")
        param_grid.Add(self.diff_impedance, 0, wx.EXPAND)

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

        # Centerline setback
        param_grid.Add(wx.StaticText(self, label="Centerline Setback (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        r = defaults.PARAM_RANGES['diff_pair_centerline_setback']
        self.centerline_setback = wx.SpinCtrlDouble(self, min=r['min'], max=r['max'],
                                                     initial=defaults.DIFF_PAIR_CENTERLINE_SETBACK, inc=r['inc'])
        self.centerline_setback.SetDigits(r['digits'])
        self.centerline_setback.SetToolTip("Distance in front of stubs to start centerline route (0 = auto: 2x P-N spacing)")
        param_grid.Add(self.centerline_setback, 0, wx.EXPAND)

        param_sizer.Add(param_grid, 0, wx.EXPAND | wx.ALL, 5)

        right_sizer.Add(param_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Options
        options_box = wx.StaticBox(self, label="Options")
        options_sizer = wx.StaticBoxSizer(options_box, wx.VERTICAL)

        # Per-pair polarity-swap allowlist (#279): glob patterns naming the
        # pairs allowed to resolve a P/N mismatch by swapping pad nets.
        # '*' = all pairs (the historical behavior), empty = no swaps ever.
        polarity_row = wx.BoxSizer(wx.HORIZONTAL)
        polarity_row.Add(wx.StaticText(self, label="Polarity-swap allowed nets:"),
                         0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        # #381 D3: default EMPTY (deny all swaps), matching route_diff.py's
        # --polarity-swap-nets deny-by-default (#279). The historical '*'
        # (allow all) silently widened polarity swaps vs the CLI.
        self.polarity_swap_nets_text = wx.TextCtrl(self, value="", size=(180, -1))
        self.polarity_swap_nets_text.SetToolTip(
            "Glob patterns (comma/space-separated) naming the diff pairs ALLOWED to "
            "resolve a P/N polarity mismatch by swapping pad net assignments (board "
            "only - schematic must be updated to match). A swap is only harmless when "
            "an endpoint can compensate (FPGA generic I/O, polarity-tolerant SerDes) - "
            "never allow USB/MIPI/TMDS pairs. '*' allows every pair; EMPTY disables "
            "swaps entirely (a mismatched pair is routed with the connectors out the "
            "opposite side, or skipped if that's not possible).")
        polarity_row.Add(self.polarity_swap_nets_text, 1, wx.ALIGN_CENTER_VERTICAL)
        options_sizer.Add(polarity_row, 0, wx.EXPAND | wx.ALL, 5)

        self.gnd_via_check = wx.CheckBox(self, label="Add GND vias")
        self.gnd_via_check.SetValue(True)
        self.gnd_via_check.SetToolTip("Add GND vias near differential pair signal vias")
        options_sizer.Add(self.gnd_via_check, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.intra_match_check = wx.CheckBox(self, label="Intra-pair length matching")
        self.intra_match_check.SetValue(False)
        self.intra_match_check.SetToolTip("Add meanders to shorter track of each pair for P/N matching")
        options_sizer.Add(self.intra_match_check, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.ac_couple_check = wx.CheckBox(self, label="AC-coupled end-to-end matching")
        self.ac_couple_check.SetValue(False)
        self.ac_couple_check.SetToolTip("Length-match diff pairs split by series DC-blocking caps "
                                        "end-to-end: detect the cap chain and match the whole P path "
                                        "vs N path across the caps, not each side alone (#196)")
        options_sizer.Add(self.ac_couple_check, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.hide_short_check = wx.CheckBox(self, label="Hide short routes")
        self.hide_short_check.SetValue(True)
        self.hide_short_check.SetToolTip(
            "Hide electrically-short pairs that the router defers to single-ended "
            "routing (coupling a sub-few-mm run gains nothing). They drop out of "
            "the differential pair list here, and stay visible on the Basic tab "
            "(so they get routed single-ended) even when 'Hide differential' is on.")
        self.hide_short_check.Bind(wx.EVT_CHECKBOX, self._on_hide_short_changed)
        options_sizer.Add(self.hide_short_check, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

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
        self.route_btn = wx.Button(self, label="Route Pairs")
        self.route_btn.SetToolTip("Start routing selected differential pairs")
        self.route_btn.Bind(wx.EVT_BUTTON, self._on_route)
        btn_sizer.Add(self.route_btn, 1, wx.RIGHT, 5)

        self.cancel_btn = wx.Button(self, label="Close")
        self.cancel_btn.SetToolTip("Close dialog (or cancel routing if in progress)")
        self.cancel_btn.Bind(wx.EVT_BUTTON, self._on_cancel_or_close)
        btn_sizer.Add(self.cancel_btn, 1)

        right_sizer.Add(btn_sizer, 0, wx.EXPAND)

        # Add spacer to push content up
        right_sizer.AddStretchSpacer(1)

        main_sizer.Add(right_sizer, 1, wx.EXPAND | wx.ALL, 5)

        self.SetSizer(main_sizer)

    def _short_params(self):
        """Current (track_width, gap, centerline_setback) driving the short test."""
        return (self.diff_pair_width.GetValue(),
                self.diff_pair_gap.GetValue(),
                self.centerline_setback.GetValue())

    def _is_short_pair(self, p_net_id, n_net_id):
        """Whether this pair would be deferred to single-ended as electrically
        short, given the current Pair Width / Gap / Centerline Setback. Cached
        per (params, pair)."""
        from diff_pair_multipoint import diff_pair_is_short
        width, gap, setback = self._short_params()
        key = ((round(width, 6), round(gap, 6), round(setback, 6)), p_net_id, n_net_id)
        if key not in self._short_cache:
            try:
                self._short_cache[key] = diff_pair_is_short(
                    self.pcb_data, p_net_id, n_net_id, width, gap, setback)
            except Exception:
                self._short_cache[key] = False
        return self._short_cache[key]

    def is_hide_short_enabled(self):
        """True when short (deferred) pairs should be hidden from diff lists and
        kept on the single-ended list."""
        return self.hide_short_check.GetValue()

    def get_short_pair_net_ids(self):
        """Net ids (P and N) of every pair that is currently electrically short.
        Used by the Basic tab to keep these single-ended nets visible even when
        'Hide differential' is on."""
        ids = set()
        for _display, _base, p_net_id, n_net_id in self.pair_panel.all_pairs:
            if self._is_short_pair(p_net_id, n_net_id):
                ids.add(p_net_id)
                ids.add(n_net_id)
        return ids

    def _on_hide_short_changed(self, event):
        """Toggle hiding short pairs; refresh this list and the Basic tab list."""
        self.pair_panel.set_hide_short(self.hide_short_check.GetValue())
        if self.on_hide_short_changed:
            self.on_hide_short_changed()

    def _on_short_param_changed(self, event):
        """A param affecting the short threshold changed - drop the cache and
        refresh both lists if short hiding is active."""
        event.Skip()
        self._short_cache.clear()
        if self.hide_short_check.GetValue():
            self.pair_panel.refresh()
            if self.on_hide_short_changed:
                self.on_hide_short_changed()

    def _on_ask_claude_diff_pairs(self):
        """Run /identify-diff-pairs headless and update the pair selection
        from its findings (issue #40)."""
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
            f"/identify-diff-pairs {os.path.abspath(board)} — analysis only, do not "
            "modify any files. After the report, end your reply with exactly one "
            "line of the form RESULT=confirm:<pair base names verified as "
            "differential by pin function, comma-separated, P/N suffix stripped>"
            ";reject:<base names that name-matching paired but are NOT differential>"
            ";custom:<P|N net-name pairs found by pin function that do not follow "
            "P/N naming conventions>. Omit empty sections; use exact net names, "
            "e.g. RESULT=confirm:/CLK,/DATA;custom:/TXP|/TXN"
        )
        model = effort = None
        if self.get_claude_params:
            claude_params = self.get_claude_params()
            model = claude_params.get('model')
            effort = claude_params.get('effort')
        dlg = ClaudeSkillDialog(
            self, "Claude: identify differential pairs", prompt,
            claude_path=claude_path, model=model, effort=effort,
            intro=f"Running /identify-diff-pairs on {os.path.basename(board)} ...\n"
                  "(datasheet lookups; typically a few minutes)")
        dlg.ShowModal()
        value = dlg.result_value
        dlg.Destroy()
        if value is not None:
            self._apply_diff_pairs_recommendation(value)

    def _apply_diff_pairs_recommendation(self, value):
        """Check confirmed pairs, uncheck rejected ones, report the rest."""
        parsed, notes = parse_diff_pairs_result(value)
        for note in notes:
            if self.append_log:
                self.append_log(f"Claude: {note}\n")

        display_by_base = {base: display for display, base, _p, _n
                           in self.pair_panel.all_pairs}
        confirmed = [b for b in parsed["confirm"] if b in display_by_base]
        rejected = [b for b in parsed["reject"] if b in display_by_base]
        unknown = [b for b in parsed["confirm"] + parsed["reject"]
                   if b not in display_by_base]

        for base in confirmed:
            self.pair_panel._checked_pairs.add(display_by_base[base])
        for base in rejected:
            self.pair_panel._checked_pairs.discard(display_by_base[base])
        if confirmed or rejected:
            self.pair_panel._update_pair_list(sync_from_visible=False)

        if self.append_log:
            if confirmed:
                self.append_log(f"Claude confirmed diff pairs: {', '.join(confirmed)}\n")
            if rejected:
                self.append_log("Claude flagged as NOT differential (unchecked): "
                                f"{', '.join(rejected)}\n")
            if unknown:
                self.append_log(f"Claude named pairs not in the list (ignored): "
                                f"{', '.join(unknown)}\n")
            if parsed["custom"]:
                pairs_str = ", ".join(f"{p}/{n}" for p, n in parsed["custom"])
                self.append_log(
                    f"Claude found pairs with unconventional names: {pairs_str} - "
                    "the GUI and router pair nets by P/N naming, so these can't be "
                    "routed as pairs yet (rename the nets, or see the explicit-pair "
                    "support issue)\n")
            if not any([confirmed, rejected, unknown, parsed["custom"]]):
                self.append_log(f"Claude: no usable diff-pair findings in {value!r}\n")

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
        # Refresh shared pcb_data from the live board BEFORE routing so the diff
        # router sees moved footprints/caps from a preceding optimize_caps step,
        # not their load-time positions (#362; same fix as the signal route path).
        if self.sync_pcb_data_callback:
            try:
                self.sync_pcb_data_callback()
            except Exception as e:
                print(f"(pre-route pcb_data sync skipped: {e})")
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

        # Remember the routed floors so _apply_results_to_board can make the live
        # board's DRC constraints consistent with them (issue #160).
        self._diff_drc_config = dict(config)

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

        # Fresh clearance ledger so a prior op's fine-pitch clearance doesn't
        # leak into this board's DRC floor.
        import clearance_ledger
        clearance_ledger.reset()
        from fab_tiers import set_fab_tier_from_config
        set_fab_tier_from_config(config)

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
                layer_costs=config.get('layer_costs') or None,  # per-layer bias (#193); None -> all 1.0
                track_width=config.get('diff_pair_width', defaults.DIFF_PAIR_WIDTH),
                impedance=config.get('impedance'),  # when set, overrides per-layer width
                clearance=config.get('clearance', 0.1),
                via_size=config.get('via_size', 0.3),
                via_drill=config.get('via_drill', 0.2),
                hole_to_hole_clearance=config.get('hole_to_hole_clearance',
                                                  defaults.HOLE_TO_HOLE_CLEARANCE),
                board_edge_clearance=config.get('board_edge_clearance',
                                                defaults.BOARD_EDGE_CLEARANCE),
                grid_step=config.get('grid_step', 0.1),
                via_cost=config.get('via_cost', 50),
                max_iterations=config.get('max_iterations', 200000),
                proximity_heuristic_factor=config.get('proximity_heuristic_factor', 0.02),
                keepout_enabled=config.get('keepout_enabled', False),
                keepout_layer=config.get('keepout_layer', defaults.KEEPOUT_LAYER),
                diff_pair_gap=config.get('diff_pair_gap', 0.101),
                min_turning_radius=config.get('min_turning_radius', 0.2),
                max_setback_angle=config.get('max_setback_angle', 45.0),
                max_turn_angle=config.get('max_turn_angle', 180.0),
                diff_chamfer_extra=config.get('diff_chamfer_extra', 1.5),
                diff_pair_centerline_setback=config.get('diff_pair_centerline_setback'),
                polarity_swap_nets=config.get('polarity_swap_nets'),
                gnd_via_enabled=config.get('gnd_via_enabled', True),
                diff_pair_intra_match=config.get('diff_pair_intra_match', False),
                ac_couple_match=config.get('ac_couple_match', False),
                enable_layer_switch=config.get('enable_layer_switch', True),
                debug_lines=config.get('debug_lines', False),
                verbose=config.get('verbose', False),
                # --- #381 D1: thread the FULL merged config through, mirroring the
                # single-ended route tab (swig_gui.py). Previously ~30 engine params
                # the tab collected were silently dropped, so a GUI/plan-replayed
                # diff run ordered pairs and cost-weighted routes DIFFERENTLY from
                # the equivalent route_diff.py command. config.get(..., <CLI default>)
                # so absent keys fall to the same value route_diff.py's argparse uses.
                ordering_strategy=config.get('ordering_strategy', 'mps'),
                direction_order=config.get('direction'),
                disable_bga_zones=config.get('no_bga_zones'),
                max_probe_iterations=config.get('max_probe_iterations', 5000),
                heuristic_weight=config.get('heuristic_weight', 1.9),
                turn_cost=config.get('turn_cost', 1000),
                direction_preference_cost=config.get(
                    'direction_preference_cost', defaults.DIRECTION_PREFERENCE_COST),
                max_rip_up_count=config.get('max_ripup', defaults.MAX_RIPUP),
                bus_enabled=config.get('bus_enabled', False),
                bus_detection_radius=config.get('bus_detection_radius',
                                                defaults.BUS_DETECTION_RADIUS),
                bus_attraction_radius=config.get('bus_attraction_radius',
                                                 defaults.BUS_ATTRACTION_RADIUS),
                bus_attraction_bonus=config.get('bus_attraction_bonus',
                                                defaults.BUS_ATTRACTION_BONUS),
                bus_min_nets=config.get('bus_min_nets', defaults.BUS_MIN_NETS),
                stub_proximity_radius=config.get('stub_proximity_radius', 2.0),
                stub_proximity_cost=config.get('stub_proximity_cost', 0.2),
                via_proximity_cost=config.get('via_proximity_cost', 10.0),
                bga_proximity_radius=config.get('bga_proximity_radius', 7.0),
                bga_proximity_cost=config.get('bga_proximity_cost', 0.2),
                track_proximity_distance=config.get('track_proximity_distance', 2.0),
                track_proximity_cost=config.get('track_proximity_cost',
                                                defaults.TRACK_PROXIMITY_COST),
                crossing_layer_check=not config.get('no_crossing_layer_check', False),
                can_swap_to_top_layer=config.get('can_swap_to_top_layer', False),
                swappable_net_patterns=config.get('swappable_nets'),
                crossing_penalty=config.get('crossing_penalty', defaults.CROSSING_PENALTY),
                routing_clearance_margin=config.get('routing_clearance_margin',
                                                    defaults.ROUTING_CLEARANCE_MARGIN),
                vertical_attraction_radius=config.get('vertical_attraction_radius',
                                                      defaults.VERTICAL_ATTRACTION_RADIUS),
                vertical_attraction_cost=config.get('vertical_attraction_cost',
                                                    defaults.VERTICAL_ATTRACTION_COST),
                ripped_route_avoidance_radius=config.get('ripped_route_avoidance_radius',
                                                         defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS),
                ripped_route_avoidance_cost=config.get('ripped_route_avoidance_cost',
                                                       defaults.RIPPED_ROUTE_AVOIDANCE_COST),
                length_match_groups=config.get('length_match_groups'),
                length_match_tolerance=config.get('length_match_tolerance',
                                                  defaults.LENGTH_MATCH_TOLERANCE),
                meander_amplitude=config.get('meander_amplitude', defaults.MEANDER_AMPLITUDE),
                time_matching=config.get('time_matching', False),
                time_match_tolerance=config.get('time_match_tolerance',
                                                defaults.TIME_MATCH_TOLERANCE),
                mps_reverse_rounds=config.get('mps_reverse_rounds', False),
                mps_layer_swap=config.get('mps_layer_swap', False),
                keep_input_copper=config.get('keep_input_copper', False),
                mps_segment_intersection=config.get('mps_segment_intersection', False),
                schematic_dir=config.get('schematic_dir'),
                add_teardrops=config.get('add_teardrops', False),
                skip_routing=config.get('skip_routing', False),
                debug_memory=config.get('debug_memory', False),
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
                'config': config,
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
        # NOTE: the button is re-enabled in _finish_complete's finally, AFTER
        # the results are applied -- it is the plan executor's busy signal,
        # and the completion MessageBox pumps events, so enabling it up
        # front let the executor start the NEXT step mid-apply (Andy's
        # 'tracks don't all appear, rerun fixes it').
        try:
            self._on_routing_complete_body()
        finally:
            self.route_btn.Enable()
            self.cancel_btn.SetLabel("Close")

    def _on_routing_complete_body(self):
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
        tracks_added, vias_added, tracks_removed = self._apply_results_to_board(results_data)

        self.status_text.SetLabel(f"Complete: {successful} routed, {failed} failed in {total_time:.1f}s")

        single_ended_pairs = results_data.get('single_ended_diff_pairs', [])
        skipped_bad_fanout = results_data.get('skipped_bad_fanout', [])

        msg = f"Differential pair routing complete!\n\n"
        msg += f"Routed: {successful}\n"
        msg += f"Failed: {failed}\n"
        if single_ended_pairs:
            msg += (f"Deferred to single-ended: {len(single_ended_pairs)} "
                    f"(electrically short - route these single-ended next)\n")
        if skipped_bad_fanout:
            msg += (f"Skipped: {len(skipped_bad_fanout)} "
                    f"(fanout stubs self-overlap - fix the fanout first)\n")
        msg += f"Time: {total_time:.1f}s\n\n"
        msg += f"Added to board:\n"
        msg += f"  {tracks_added} segments\n"
        msg += f"  {vias_added} vias\n"
        if tracks_removed > 0:
            msg += f"  {tracks_removed} stale segment(s) removed\n"
        if failed > 0:
            try:
                from routing_diagnostics import (
                    suggest_diff_pair_adjustments, format_suggestions_for_dialog)
                suggestions = suggest_diff_pair_adjustments(
                    failed=failed, total=successful + failed,
                    config=result.get('config', {}))
                block = format_suggestions_for_dialog(suggestions)
                if block:
                    msg += "\n" + block + "\n"
            except Exception as e:
                print(f"Warning: failed to build diff pair suggestions: {e}")
        msg += "\nUse Edit -> Undo to revert changes."

        if getattr(getattr(self, 'GetTopLevelParent', lambda: self)(), '_suppress_completion_popups', False):
            print(msg)  # unattended plan run: no per-step OK dialog
        else:
            wx.MessageBox(msg, "Routing Complete", wx.OK | wx.ICON_INFORMATION)

        # Refresh the pair list to show updated connectivity
        self.pair_panel.refresh()

    def _apply_results_to_board(self, results_data):
        """Apply routing results directly to the open pcbnew board."""
        import pcbnew
        from kicad_parser import POSITION_DECIMALS
        from .swig_gui import _build_layer_mappings
        from .board_swaps import apply_swaps_to_board

        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox("Board is no longer open", "Error", wx.OK | wx.ICON_ERROR)
            return 0, 0, 0

        # Apply pad/stub net swaps (polarity fixes, target swaps) and stub layer
        # modifications BEFORE adding new tracks - the routes were created
        # assuming these swaps, so skipping them leaves shorts at swapped pads
        apply_swaps_to_board(board, results_data)

        tracks_added = 0
        vias_added = 0
        tracks_removed = 0

        # Get layer mappings
        name_to_id, _ = _build_layer_mappings()

        def get_layer_id(layer_name):
            return name_to_id.get(layer_name, pcbnew.F_Cu)

        # Remove original-board copper the cleanup pipeline flagged (dead-end
        # sweep / cycle prune / graze prune), mirroring the CLI writer's strip
        # and swig_gui.py's single-ended apply path -- otherwise stale copper
        # from a prior routing pass stays on the live board.
        segs_to_remove = results_data.get('segments_to_remove') or []
        if segs_to_remove:
            remove_keys = set()
            for s in segs_to_remove:
                a = (round(s.start_x, POSITION_DECIMALS), round(s.start_y, POSITION_DECIMALS))
                b = (round(s.end_x, POSITION_DECIMALS), round(s.end_y, POSITION_DECIMALS))
                remove_keys.add((frozenset((a, b)), s.layer, s.net_id))
            for track in list(board.GetTracks()):
                if track.Type() != pcbnew.PCB_TRACE_T:
                    continue  # skip vias / arcs
                a = (round(pcbnew.ToMM(track.GetStart().x), POSITION_DECIMALS),
                     round(pcbnew.ToMM(track.GetStart().y), POSITION_DECIMALS))
                b = (round(pcbnew.ToMM(track.GetEnd().x), POSITION_DECIMALS),
                     round(pcbnew.ToMM(track.GetEnd().y), POSITION_DECIMALS))
                key = (frozenset((a, b)), board.GetLayerName(track.GetLayer()),
                       track.GetNetCode())
                if key in remove_keys:
                    board.Remove(track)
                    tracks_removed += 1

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

        # Make the live board's DRC constraints consistent with what we just
        # routed to (issue #160), the GUI counterpart of the CLI route_diff's
        # fix_kicad_drc_settings auto-fix: loosen the Board Setup floors + Default
        # net class + non-routing severities to the routed values via the pcbnew
        # API, so the user's manual DRC only flags genuine problems. Best-effort
        # and guarded -- never block applying the routes. The user's next save
        # persists it (mark the board modified).
        cfg = getattr(self, "_diff_drc_config", None)
        if tracks_added > 0 and cfg and cfg.get('fix_drc_settings', True):
            try:
                from fix_kicad_drc_settings import (compute_targets, severity_plan,
                                                    apply_targets_to_board)
                # Grade at the smallest clearance any step actually routed at, like the CLI.
                import clearance_ledger
                eff_clearance = clearance_ledger.effective(cfg.get('clearance')) \
                    if cfg.get('clearance') else cfg.get('clearance')
                targets = compute_targets(
                    clearance=eff_clearance,
                    hole_to_hole=cfg.get('hole_to_hole_clearance'),
                    edge_clearance=cfg.get('board_edge_clearance'),
                    track_width=cfg.get('track_width'),
                    via_diameter=cfg.get('via_size'),
                    via_drill=cfg.get('via_drill'))
                if apply_targets_to_board(
                        board, targets, severity_plan(keep_thermal=cfg.get('keep_thermal', False)),
                        diff_pair_gap=cfg.get('diff_pair_gap'),
                        diff_pair_width=cfg.get('track_width'),
                        clamp_nondefault_netclasses=cfg.get('clamp_netclasses', False)):
                    board.SetModified()
                    print("DRC settings: loosened Board Setup floors to the diff-pair routing values (save to persist)")
            except Exception as e:
                print(f"(skipped DRC-settings write-back: {e})")

        # Refresh the view
        pcbnew.Refresh()

        # Sync pcb_data from pcbnew board
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()

        return tracks_added, vias_added, tracks_removed

        from .gui_utils import update_live_drc_floors
        _cfg = getattr(self, '_diff_drc_config', {}) or {}
        import pcbnew as _p
        update_live_drc_floors(
            _p.GetBoard(),
            clearance=_cfg.get('clearance'),
            track_width=_cfg.get('diff_pair_width') or _cfg.get('track_width'),
            via_size=_cfg.get('via_size'),
            via_drill=_cfg.get('via_drill'),
            hole_to_hole=_cfg.get('hole_to_hole_clearance'))

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

    def get_polarity_swap_nets(self):
        """Parse the polarity-swap allowlist field into a pattern list (#279).

        Comma/space-separated globs; empty field -> None (no swaps ever),
        matching route_diff.py's --polarity-swap-nets semantics.
        """
        text = self.polarity_swap_nets_text.GetValue()
        patterns = [t for t in text.replace(',', ' ').split() if t]
        return patterns or None

    def get_config(self):
        """Get the differential pair configuration."""
        setback = self.centerline_setback.GetValue()
        return {
            'diff_pair_width': self.diff_pair_width.GetValue(),
            'diff_pair_gap': self.diff_pair_gap.GetValue(),
            'impedance': self.diff_impedance.GetValue() or None,  # 0 = off
            'min_turning_radius': self.min_turning_radius.GetValue(),
            'max_setback_angle': self.max_setback_angle.GetValue(),
            'max_turn_angle': self.max_turn_angle.GetValue(),
            'diff_chamfer_extra': self.chamfer_extra.GetValue(),
            'diff_pair_centerline_setback': setback if setback > 0 else None,  # 0 = auto
            'polarity_swap_nets': self.get_polarity_swap_nets(),
            'gnd_via_enabled': self.gnd_via_check.GetValue(),
            'diff_pair_intra_match': self.intra_match_check.GetValue(),
            'ac_couple_match': self.ac_couple_check.GetValue(),
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
