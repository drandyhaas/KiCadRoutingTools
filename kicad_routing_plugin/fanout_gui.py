"""
KiCad Routing Tools - Fanout GUI Components

Provides wx-based panels for BGA and QFN fanout configuration.
"""

import os
import sys
import threading
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

#: #742: `bga_fanout.constants.DEFAULT_VIA_SIZE`, the value
#: `place_fanout_clearance.py --default-via-size` defaults to and
#: `repair_fanout_clearance`'s signature repeats. NOT
#: `routing_defaults.VIA_SIZE` -- a fallback SIZE for an unreadable via is a
#: different quantity from the via geometry this tab places.
#:
#: COPIED, not imported, and measured: `from bga_fanout.constants import ...`
#: costs ~1 s and 62 new top-level modules at dialog load -- numpy, the Rust
#: `grid_router`, `obstacle_map`, `kicad_writer` -- because that package's
#: `__init__` is the whole fanout engine. swig_gui and planes_gui import this
#: module at import time, so every routing dialog would pay it for one float.
#: This file's own idiom is the lazy `import bga_fanout` further down.
#: `tests/test_772_ai_plan_cap_params.py` pins the two spellings equal, which
#: is where a drift detector belongs -- in a test, not in the plugin's
#: start-up path.
CAP_DEFAULT_VIA_SIZE = 0.3


def _get_net_classes_from_board():
    """Get net class mapping from pcbnew.

    Returns:
        tuple: (net_to_class dict mapping net_name -> class_name,
                list of class names sorted with 'Default' first)
    """
    try:
        import pcbnew
        board = pcbnew.GetBoard()
        if board is None:
            return {}, ['Default']

        net_to_class = {}
        netclass_names = set()

        # Get net settings which has the net class info
        ds = board.GetDesignSettings()
        net_settings = ds.m_NetSettings

        # Get all defined net classes
        net_classes = net_settings.GetNetclasses()
        for class_name in net_classes.keys():
            netclass_names.add(str(class_name))

        # Always include Default
        netclass_names.add('Default')

        # Get net class for each net using GetEffectiveNetClass
        net_info = board.GetNetInfo()
        nets_by_name = net_info.NetsByName()

        for net_name_wx, net in nets_by_name.items():
            net_name = str(net_name_wx)
            if not net_name or net_name.lower().startswith('unconnected-'):
                continue

            try:
                # GetEffectiveNetClass returns the actual NETCLASS object
                netclass = net_settings.GetEffectiveNetClass(net_name)
                if netclass:
                    class_name_raw = str(netclass.GetName())
                    # Handle composite class names like 'Wide,Default'
                    # Use the first non-Default class, or 'Default' if only Default
                    if ',' in class_name_raw:
                        parts = [p.strip() for p in class_name_raw.split(',')]
                        non_default = [p for p in parts if p != 'Default']
                        class_name = non_default[0] if non_default else 'Default'
                    else:
                        class_name = class_name_raw
                else:
                    class_name = 'Default'
                net_to_class[net_name] = class_name
            except Exception:
                net_to_class[net_name] = 'Default'

        # Sort with 'Default' first
        sorted_classes = ['Default'] if 'Default' in netclass_names else []
        sorted_classes.extend(sorted(c for c in netclass_names if c != 'Default'))

        return net_to_class, sorted_classes
    except Exception:
        return {}, ['Default']


def cap_optimization_summary(result):
    """The one-line summary for a repair_fanout_clearance result (#130/#746).

    A module-level function rather than a method body so it can be driven with
    a plain dict: it needs no board, no dialog and no wx, and before #746 the
    only way to reach it was a live pcbnew board, so nothing tested it and the
    wording below went wrong unnoticed.

    Reads every key with `.get` -- the engine's two early returns carry neither
    'via_moves'/'new_segments' nor 'via_resolved'/'regrazed'.

    #746: `resolved` is graded at the END of the pass, so it spans BOTH
    mechanisms -- a cap the descent walked clear and a cap only the via-nudge
    could free. `via_resolved` says which, so this line does too instead of
    leaving the operator to infer it from the engine's stdout. `regrazed`
    names the caps that were clean before the nudge and are grazing after it
    -- the pass broke them, whether or not it had fixed them first. Silence
    there is the normal case.
    """
    moved = len(result.get('placements') or [])
    nudged = len(result.get('via_moves') or [])
    unresolved = result.get('unresolved') or []
    via_resolved = result.get('via_resolved') or []
    regrazed = result.get('regrazed') or []
    summary = f"Decoupling caps optimized: {moved} moved"
    if nudged:
        summary += f"; {nudged} via(s) nudged with reconnect (#313)"
    if via_resolved:
        summary += f"; {len(via_resolved)} cap(s) freed by that nudge"
    if unresolved:
        # The verdict has ALWAYS been via + track + pad (the engine's
        # graze_penalty is via #130 + track #278 + pad #275). This line said
        # "could not clear a foreign via", naming one of the three: wrong
        # before #736 and more visibly wrong after it -- not because the
        # track channel was new (it has been in graze_penalty since the module
        # landed) but because #736 made the pass's OWN connector tracks
        # reachable in this verdict for the first time. Worded to match the
        # engine's own seed disclosure.
        summary += (f"; {len(unresolved)} still grazing foreign copper "
                    f"(via/track/pad) "
                    f"(manual: {', '.join(sorted(unresolved))})")
    if regrazed:
        summary += (f"; {len(regrazed)} re-grazed by this pass's own "
                    f"connector copper: {', '.join(sorted(regrazed))}")
    return summary


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
        self._suspend_check = False  # Temporarily disable check_fn during restore
        self._show_hide_checkbox = show_hide_checkbox
        self._show_hide_differential = show_hide_differential
        self._hide_differential_default = hide_differential_default
        self._auto_hide_differential = auto_hide_differential
        self._component_filter_value = ""  # For programmatic component filtering
        self._min_pads_for_dropdown = min_pads_for_dropdown
        self._differential_mode = False  # When True, show diff pairs as "name_P/N"
        self._diff_pairs = {}  # base_name -> (p_net_id, n_net_id) when in diff mode
        self._on_selection_changed = None  # Callback when selection changes
        self._on_tabbed_view_changed = None  # Callback when tabbed view is created/destroyed
        # Optional: keep electrically-short diff-pair nets visible even when
        # "Hide differential" is on, so they can be routed single-ended. Set by
        # the main dialog to wire the Differential tab's "Hide short routes".
        self._short_nets_provider = None      # () -> set of net_ids
        self._hide_short_enabled_fn = None    # () -> bool
        self._short_nets = set()              # net_ids kept visible under hide_diff

        # Net class separation
        self._separate_by_netclass = False
        self._net_to_class = {}  # net_name -> netclass_name
        self._netclass_names = ['Default']
        self._tabbed_net_lists = {}  # netclass_name -> wx.CheckListBox
        self._netclass_notebook = None

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

        # Separate by net class checkbox
        self.separate_netclass_check = wx.CheckBox(self, label="Separate by net class")
        self.separate_netclass_check.SetValue(False)
        self.separate_netclass_check.SetToolTip("Organize nets by KiCad net class in tabs")
        self.separate_netclass_check.Bind(wx.EVT_CHECKBOX, self._on_separate_netclass_changed)
        sizer.Add(self.separate_netclass_check, 0, wx.LEFT | wx.RIGHT | wx.TOP, 5)

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
        self.filter_ctrl.SetToolTip("Filter nets by name (case-insensitive)")
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

        # Net list (wrapped in container for view swapping)
        self._list_container_sizer = wx.BoxSizer(wx.VERTICAL)
        self.net_list = wx.CheckListBox(self, size=(200, -1), style=wx.LB_EXTENDED)
        self.net_list.SetToolTip("Check nets to include in operation (Ctrl+A to select all highlighted)")
        self.net_list.Bind(wx.EVT_KEY_DOWN, self._on_net_list_key)
        self.net_list.Bind(wx.EVT_CHECKLISTBOX, self._on_checklist_toggled)
        self._list_container_sizer.Add(self.net_list, 1, wx.EXPAND)
        sizer.Add(self._list_container_sizer, 1, wx.EXPAND | wx.ALL, 5)

        # Select/Unselect buttons
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.select_btn = wx.Button(self, label="Select")
        self.select_btn.SetToolTip("Check all highlighted nets")
        self.select_btn.Bind(wx.EVT_BUTTON, self._on_select)
        self.unselect_btn = wx.Button(self, label="Unselect")
        self.unselect_btn.SetToolTip("Uncheck all highlighted nets")
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

    def set_selection_changed_callback(self, fn):
        """Set callback to be called when selection changes.

        Args:
            fn: Function() called when nets are selected/unselected
        """
        self._on_selection_changed = fn

    def _notify_selection_changed(self):
        """Notify that selection has changed."""
        if self._on_selection_changed:
            self._on_selection_changed()

    def set_tabbed_view_changed_callback(self, fn):
        """Set callback to be called when tabbed view is created/destroyed.

        Args:
            fn: Function(notebook_or_none) called with the notebook when created,
                or None when destroyed
        """
        self._on_tabbed_view_changed = fn

    def _notify_tabbed_view_changed(self):
        """Notify that tabbed view has changed."""
        if self._on_tabbed_view_changed:
            self._on_tabbed_view_changed(self._netclass_notebook)

    def suspend_check(self):
        """Temporarily disable connectivity checking during settings restore."""
        self._suspend_check = True

    def resume_check(self):
        """Re-enable connectivity checking after settings restore."""
        self._suspend_check = False

    def set_component_filter(self, component_ref):
        """Set the component filter value.

        This can be used to filter by component even when the component filter
        text control is not shown.
        """
        self._component_filter_value = component_ref
        if self.component_filter_ctrl:
            self.component_filter_ctrl.SetValue(component_ref)
        self._update_net_list()

    def _update_net_list(self, sync_from_visible=True):
        """Update the net list based on filters.

        Args:
            sync_from_visible: If True, sync _checked_nets from visible items first.
                              Set to False when restoring settings to avoid clearing them.
        """
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

        # Save checked state before filtering (only if syncing from visible)
        if sync_from_visible:
            self._sync_checked_state_from_view()

        # Build set of nets connected to the filtered component. Shared with the
        # CLI via net_queries (#537) so the same reference cannot select
        # different nets here than it does in route.py. 'substring' keeps this
        # box's long-standing behaviour -- a bare "U1" still narrows to U1, U10,
        # U100 as you type -- while a token carrying * ? or [ is now honoured as
        # a glob instead of being searched for literally.
        component_nets = set()
        component_net_ids = set()
        if component_filter:
            from net_queries import nets_for_components
            _sel = nets_for_components(self.pcb_data, [component_filter],
                                       match='substring')
            component_nets = set(_sel.net_names)
            component_net_ids = set(_sel.net_ids)

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

        # Electrically-short pairs are deferred to single-ended routing, so when
        # "Hide short routes" is on they stay visible here even under hide_diff.
        self._short_nets = set()
        if (hide_diff and not self._differential_mode
                and self._hide_short_enabled_fn and self._short_nets_provider
                and self._hide_short_enabled_fn()):
            try:
                self._short_nets = self._short_nets_provider() or set()
            except Exception:
                self._short_nets = set()

        # Populate either single list or tabbed lists
        if self._separate_by_netclass and self._tabbed_net_lists:
            self._populate_tabbed_lists(filtered_nets, hide_checked, hide_diff)
        else:
            self._populate_single_list(filtered_nets, hide_checked, hide_diff)

    def set_short_net_filter(self, short_nets_provider, hide_short_enabled_fn):
        """Wire the Differential tab's 'Hide short routes': short (deferred) pair
        nets are kept visible under 'Hide differential' so they route single-ended.
        short_nets_provider() -> set of net_ids; hide_short_enabled_fn() -> bool."""
        self._short_nets_provider = short_nets_provider
        self._hide_short_enabled_fn = hide_short_enabled_fn

    def _populate_single_list(self, filtered_nets, hide_checked, hide_diff):
        """Populate the single CheckListBox."""
        self.net_list.Clear()
        for name, net_id in filtered_nets:
            # Check if should be hidden (connected)
            if hide_checked and self._check_fn and not self._suspend_check:
                if self._check_fn(net_id):
                    continue
            # Check if should be hidden (differential) - but keep short pairs,
            # which are routed single-ended
            if hide_diff and self._is_differential_net(name):
                if net_id not in self._short_nets:
                    continue
            idx = self.net_list.Append(name)
            # Restore checked state
            if name in self._checked_nets:
                self.net_list.Check(idx, True)

        # Highlight all items by default
        for i in range(self.net_list.GetCount()):
            self.net_list.SetSelection(i)

    def _populate_tabbed_lists(self, filtered_nets, hide_checked, hide_diff):
        """Populate the tabbed CheckListBoxes by net class."""
        # Group nets by class
        nets_by_class = {c: [] for c in self._netclass_names}

        for name, net_id in filtered_nets:
            # Check if should be hidden (connected)
            if hide_checked and self._check_fn and not self._suspend_check:
                if self._check_fn(net_id):
                    continue
            # Check if should be hidden (differential) - but keep short pairs,
            # which are routed single-ended
            if hide_diff and self._is_differential_net(name):
                if net_id not in self._short_nets:
                    continue
            class_name = self._net_to_class.get(name, 'Default')
            if class_name in nets_by_class:
                nets_by_class[class_name].append((name, net_id))

        # Populate each tab
        for class_name, check_list in self._tabbed_net_lists.items():
            check_list.Clear()
            for name, net_id in nets_by_class.get(class_name, []):
                idx = check_list.Append(name)
                if name in self._checked_nets:
                    check_list.Check(idx, True)

            # Highlight all items
            for i in range(check_list.GetCount()):
                check_list.SetSelection(i)

    def _on_filter_changed(self, event):
        """Handle filter text change."""
        self._update_net_list()

    def _on_checklist_toggled(self, event):
        """Handle checkbox toggle in the net list."""
        self._notify_selection_changed()
        event.Skip()

    def _on_net_list_key(self, event):
        """Handle keyboard events in net list."""
        # Ctrl+A selects all items
        if event.GetKeyCode() == ord('A') and event.ControlDown():
            for i in range(self.net_list.GetCount()):
                self.net_list.SetSelection(i)
        else:
            event.Skip()

    def _get_selected_indices(self):
        """Get indices of selected (highlighted) items in the active list."""
        check_list = self._get_active_check_list()
        if check_list:
            return list(check_list.GetSelections())
        return []

    def _on_select(self, event):
        """Check the highlighted nets."""
        check_list = self._get_active_check_list()
        if check_list:
            for i in self._get_selected_indices():
                check_list.Check(i, True)
                # Also update _checked_nets
                name = check_list.GetString(i)
                self._checked_nets.add(name)
            self._notify_selection_changed()

    def _on_unselect(self, event):
        """Uncheck the highlighted nets."""
        check_list = self._get_active_check_list()
        if check_list:
            for i in self._get_selected_indices():
                check_list.Check(i, False)
                # Also update _checked_nets
                name = check_list.GetString(i)
                self._checked_nets.discard(name)
            self._notify_selection_changed()

    def get_selected_nets(self):
        """Get selected net names, in a DETERMINISTIC (sorted) order.

        `_checked_nets` is a set, and `list(set_of_str)` follows Python's
        per-process randomized string hashing -- so this returned a different
        ORDER on every launch. The router routes nets in the order it is given,
        so that made the whole GUI non-deterministic: three runs of the same
        plan on the same board produced 3425, 3433 and 3431 segments (measured
        on eth_tap step 11; with PYTHONHASHSEED=0 pinned, two runs were
        bit-identical). It also made the GUI disagree with the CLI, which
        passes `expand_net_patterns`' SORTED list -- the same 295 nets in a
        different order.

        Sorted matches the CLI exactly, so both fronts route in one order.
        """
        # Sync from the current view
        self._sync_checked_state_from_view()
        # Return all checked nets
        return sorted(self._checked_nets)

    def set_selected_nets(self, net_names):
        """Pre-check the given net names (only those present in this panel).

        Replaces the current checked state with the intersection of net_names
        and the nets known to this panel, then refreshes the view.

        Args:
            net_names: Iterable of net name strings to check.
        """
        available = {name for name, _ in self.all_nets}
        self._checked_nets = {name for name in net_names if name in available}
        self.refresh(sync_from_visible=False)
        self._notify_selection_changed()

    def refresh(self, sync_from_visible=True):
        """Refresh the net list.

        Args:
            sync_from_visible: If True, sync _checked_nets from visible items first.
                              Set to False when restoring settings to avoid clearing them.
        """
        self._update_net_list(sync_from_visible=sync_from_visible)

    def _is_differential_net(self, name):
        """Check if a net name looks like a differential pair net."""
        from net_queries import extract_diff_pair_base
        return extract_diff_pair_base(name) is not None

    def _on_separate_netclass_changed(self, event):
        """Handle the separate by net class checkbox toggle."""
        enable_tabs = self.separate_netclass_check.GetValue()

        # Sync checked state from current view before switching
        self._sync_checked_state_from_view()

        if enable_tabs:
            if not self._create_tabbed_view():
                return  # Failed to create tabs, checkbox already unchecked

            # Hide single list, show notebook
            self.net_list.Hide()
            self._list_container_sizer.Clear()
            self._list_container_sizer.Add(self._netclass_notebook, 1, wx.EXPAND)
            self._netclass_notebook.Show()
            self._separate_by_netclass = True
            self._notify_tabbed_view_changed()
        else:
            # Hide notebook, show single list
            self._destroy_tabbed_view()
            self._list_container_sizer.Clear()
            self._list_container_sizer.Add(self.net_list, 1, wx.EXPAND)
            self.net_list.Show()
            self._separate_by_netclass = False
            self._notify_tabbed_view_changed()

        self.Layout()
        # Don't sync again - we already synced before switching views
        self._update_net_list(sync_from_visible=False)

    def _create_tabbed_view(self):
        """Create the tabbed notebook view for net class separation.

        Returns:
            bool: True if tabs were created, False if only Default class exists
        """
        # Fetch net classes from pcbnew
        self._net_to_class, self._netclass_names = _get_net_classes_from_board()

        # If we only have Default or couldn't get classes, don't switch
        if len(self._netclass_names) <= 1:
            wx.MessageBox(
                "No custom net classes found. All nets are in 'Default' class.",
                "Net Classes",
                wx.OK | wx.ICON_INFORMATION
            )
            self.separate_netclass_check.SetValue(False)
            return False

        # Create notebook
        self._netclass_notebook = wx.Notebook(self)
        self._tabbed_net_lists = {}

        for class_name in self._netclass_names:
            panel = wx.Panel(self._netclass_notebook)
            panel_sizer = wx.BoxSizer(wx.VERTICAL)

            check_list = wx.CheckListBox(panel, size=(200, -1), style=wx.LB_EXTENDED)
            check_list.Bind(wx.EVT_KEY_DOWN, self._on_net_list_key)
            check_list.Bind(wx.EVT_CHECKLISTBOX, self._on_checklist_toggled)
            panel_sizer.Add(check_list, 1, wx.EXPAND)

            panel.SetSizer(panel_sizer)
            self._netclass_notebook.AddPage(panel, class_name)
            self._tabbed_net_lists[class_name] = check_list

        return True

    def _destroy_tabbed_view(self):
        """Destroy the tabbed notebook and return to single list view."""
        if self._netclass_notebook:
            self._netclass_notebook.Destroy()
            self._netclass_notebook = None
            self._tabbed_net_lists = {}

    def _sync_checked_state_from_view(self):
        """Sync _checked_nets from the currently visible view."""
        if self._separate_by_netclass and self._tabbed_net_lists:
            for class_name, check_list in self._tabbed_net_lists.items():
                for i in range(check_list.GetCount()):
                    name = check_list.GetString(i)
                    if check_list.IsChecked(i):
                        self._checked_nets.add(name)
                    else:
                        self._checked_nets.discard(name)
        else:
            for i in range(self.net_list.GetCount()):
                name = self.net_list.GetString(i)
                if self.net_list.IsChecked(i):
                    self._checked_nets.add(name)
                else:
                    self._checked_nets.discard(name)

    def _get_active_check_list(self):
        """Get the currently active CheckListBox (either single list or current tab)."""
        if self._separate_by_netclass and self._netclass_notebook:
            current_tab = self._netclass_notebook.GetSelection()
            if current_tab >= 0 and current_tab < len(self._netclass_names):
                class_name = self._netclass_names[current_tab]
                return self._tabbed_net_lists.get(class_name)
        return self.net_list

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

        # Sync from the current view
        self._sync_checked_state_from_view()

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


class BGAOptionsPanel(wx.ScrolledWindow):
    """BGA fanout options panel (parameters not in Basic tab).

    A vertical ScrolledWindow: the panel has more sections (escape, options,
    cap placement) than fit a short dialog, and without scrolling the lower
    controls were simply unreachable.
    """

    # wx.Choice index -> engine escape_method value (order matches the dropdown)
    ESCAPE_METHODS = ('auto', 'channel', 'underpad', 'dogbone')

    #: The "Cap Placement (advanced)" knobs: control attribute -> the value
    #: the control is CREATED with -- which is ALSO
    #: place_fanout_clearance.py's argparse default for the same flag and
    #: repair_fanout_clearance's signature default for the same kwarg
    #: (--capture-radius / --near-margin / --step / --max-displacement /
    #: --max-displacement-cap / --displacement-growth /
    #: --board-edge-clearance / --max-passes / --cap-prefix / --no-rotate).
    #:
    #: ONE table, three readers (#772):
    #:   swig_gui.reset_params_to_defaults      CLAUDE.md's "add it to
    #:       reset_params_to_defaults ... or the param leaks between
    #:       steps". EIGHT of these ten had never been in it -- only
    #:       optimize_caps, cap_allow_rotation and cap_max_passes were.
    #:   swig_gui.reset_cap_params_to_defaults  the SCOPED reset the plan
    #:       executor runs before a cap step that names any of them (the
    #:       per-step reset is skipped for optimize_caps by design).
    #:   ai_plan._next_step                     reads the NAMES, to decide
    #:       whether the step named a cap knob at all.
    #:
    #: Hand-written next to the _cap_spin calls rather than derived from
    #: them, so tests/gui_parity/test_772_cap_params_reach_engine.py can
    #: assert on a FRESHLY CONSTRUCTED panel that every name exists and
    #: every default matches -- and, separately, that each equals the
    #: engine signature default. Drift is caught by the gate, not hoped
    #: away.
    CAP_PARAM_DEFAULTS = (
        ('cap_capture_radius', 2.0),
        ('cap_near_margin', 1.0),
        ('cap_step', 0.2),
        ('cap_max_displacement', 2.0),
        ('cap_max_displacement_cap', 3.0),
        ('cap_displacement_growth', 1.5),
        # 0.0 == UNSET, not a margin of zero: get_config maps it to None,
        # and the engine's resolve_cap_edge_clearance applies the same
        # non-positive-is-unset rule to an EXPLICIT CLI value, so both
        # fronts land on the same resolved margin.
        ('cap_board_edge_clearance', 0.0),
        ('cap_max_passes', 30),
        ('cap_prefix', 'C,R,FB'),
        # #742. bga_fanout.constants.DEFAULT_VIA_SIZE, which is what
        # place_fanout_clearance.py --default-via-size defaults to. NOT the
        # Basic tab's via_size (0.5 out of the box) -- see the control.
        ('cap_default_via_size', CAP_DEFAULT_VIA_SIZE),
        ('cap_allow_rotation', True),
    )

    def __init__(self, parent, on_differential_changed=None):
        """
        Create BGA options panel.

        Args:
            parent: Parent window
            on_differential_changed: Callback(bool) when differential checkbox changes
        """
        super().__init__(parent, style=wx.VSCROLL)
        self.SetScrollRate(0, 10)
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
        self.exit_margin.SetToolTip("Distance from BGA edge to route escape vias")
        grid.Add(self.exit_margin, 0, wx.EXPAND)

        param_sizer.Add(grid, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(param_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Routing mode section
        mode_box = wx.StaticBox(self, label="Routing Mode")
        mode_sizer = wx.StaticBoxSizer(mode_box, wx.VERTICAL)

        self.differential_check = wx.CheckBox(self, label="Differential pairs")
        self.differential_check.SetValue(False)
        self.differential_check.SetToolTip("Route as differential pairs (uses Pair Gap from Differential tab)")
        self.differential_check.Bind(wx.EVT_CHECKBOX, self._on_differential_changed)
        mode_sizer.Add(self.differential_check, 0, wx.ALL, 5)

        main_sizer.Add(mode_sizer, 0, wx.EXPAND | wx.BOTTOM, 5)

        # Escape direction section
        escape_box = wx.StaticBox(self, label="Escape Direction")
        escape_sizer = wx.StaticBoxSizer(escape_box, wx.VERTICAL)

        self.escape_direction = wx.RadioBox(
            self, label="", choices=["Horizontal", "Vertical"],
            majorDimension=2, style=wx.RA_SPECIFY_COLS
        )
        self.escape_direction.SetToolTip("Primary direction for escape routes from BGA pads")
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

        esc_method_row = wx.BoxSizer(wx.HORIZONTAL)
        esc_method_row.Add(wx.StaticText(self, label="Escape method:"), 0,
                           wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        self.escape_method_choice = wx.Choice(
            self, choices=["Auto (channel, under-pad retry)",
                           "Channel",
                           "Under-pad (dense BGA)",
                           "Dog-bone (gap vias)"])
        self.escape_method_choice.SetSelection(0)
        self.escape_method_choice.SetToolTip(
            "Auto (default): run the channel router and, if it drops any ball, "
            "retry with the under-pad grid escape and keep whichever escapes "
            "more (#288). Channel: 45-stub + channel router only. Under-pad: "
            "each signal vias in its pad and routes under the pad field on "
            "inner layers - escapes fully-populated arrays the channel router "
            "can't (#122); use a small via/track for dense pitch (e.g. via "
            "0.35, track 0.12). Dog-bone: under-pad with each escape via in "
            "the diagonal inter-ball gap instead of in the pad (#128) - the "
            "standard hand-layout escape; keeps ball-grid positions free of "
            "barrels on inner layers and avoids via-vs-neighbour-ball grazes "
            "on small balls.")
        esc_method_row.Add(self.escape_method_choice, 1)
        options_sizer.Add(esc_method_row, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.plane_drop = wx.CheckBox(self, label="Drop plane-net balls to vias")
        self.plane_drop.SetValue(True)
        self.plane_drop.SetToolTip(
            "After the signal escape, give every plane-net ball (a net excluded "
            "from the fanout with >= 6 balls, or one that already owns a zone) "
            "a via now: a dog-bone via in a free inter-ball gap, else "
            "via-in-pad. The plane poured later picks the via up at fill, "
            "instead of the plane step pushing a tap through the finished ball "
            "field (#360/#424). On by default; matches the CLI --plane-drop.")
        options_sizer.Add(self.plane_drop, 0, wx.LEFT | wx.BOTTOM, 5)

        # Future-pour declaration (review parity finding 5: --plane-net-layers
        # was CLI-only, so a manifest-converted plan silently dropped it).
        # Named after the engine param so the plan executor's generic loop
        # fills it; empty = None, same as the CLI omitting the flag.
        pnl_row = wx.BoxSizer(wx.HORIZONTAL)
        pnl_row.Add(wx.StaticText(self, label="Plane net layers:"), 0,
                    wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        self.plane_net_layers_ctrl = wx.TextCtrl(self, value="")
        self.plane_net_layers_ctrl.SetToolTip(
            "Declare the FUTURE plane plan for the plane-drop decision: for "
            "each plane net, the layer(s) it will be poured on, "
            "space-separated NET:LAYER[,LAYER...] specs, e.g. "
            "'GND:In1.Cu,In4.Cu P3.3V:In2.Cu'. Empty = none declared "
            "(matches the CLI --plane-net-layers).")
        pnl_row.Add(self.plane_net_layers_ctrl, 1)
        options_sizer.Add(pnl_row, 0,
                          wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 5)

        self.optimize_caps = wx.CheckBox(self, label="Optimize decoupling cap placement")
        self.optimize_caps.SetValue(False)
        self.optimize_caps.SetToolTip(
            "After fanout, nudge decoupling caps near the BGA so their pads "
            "clear every foreign-net fanout via (and any foreign track on the "
            "cap's side), and pull each pad toward the nearest same-net ball so "
            "a power/GND via dropped there later also lands on the cap (#130). "
            "Caps move as little as possible, never overlap each other, and a "
            "cap that can't clear is reported. Uses the Basic-tab clearance/via "
            "settings. Off by default.")
        options_sizer.Add(self.optimize_caps, 0, wx.LEFT | wx.BOTTOM, 5)

        main_sizer.Add(options_sizer, 0, wx.EXPAND)

        # Decoupling-cap placement (advanced) -- knobs for the "Optimize
        # decoupling cap placement" repair (place_fanout_clearance.py / #130).
        # Only take effect when that checkbox is on; clearance/grid/via come
        # from the Basic tab.
        cap_box = wx.StaticBox(self, label="Cap Placement (advanced)")
        cap_sizer = wx.StaticBoxSizer(cap_box, wx.VERTICAL)
        cap_grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=4)
        cap_grid.AddGrowableCol(1)

        def _cap_spin(label, initial, lo, hi, inc, digits, tip):
            cap_grid.Add(wx.StaticText(self, label=label), 0, wx.ALIGN_CENTER_VERTICAL)
            ctrl = wx.SpinCtrlDouble(self, min=lo, max=hi, initial=initial, inc=inc)
            ctrl.SetDigits(digits)
            ctrl.SetToolTip(tip)
            cap_grid.Add(ctrl, 0, wx.EXPAND)
            return ctrl

        self.cap_capture_radius = _cap_spin(
            "Capture radius (mm):", 2.0, 0.0, 20.0, 0.5, 2,
            "How far from a BGA edge a cap is considered for nudging (--capture-radius)")
        self.cap_near_margin = _cap_spin(
            "Near margin (mm):", 1.0, 0.0, 10.0, 0.25, 2,
            "Extra slack pulling a cap pad toward its nearest same-net ball (--near-margin)")
        self.cap_step = _cap_spin(
            "Search step (mm):", 0.2, 0.05, 2.0, 0.05, 2,
            "Displacement search step (--step)")
        self.cap_max_displacement = _cap_spin(
            "Max displacement (mm):", 2.0, 0.0, 20.0, 0.5, 2,
            "Initial cap of how far a cap may move (--max-displacement)")
        self.cap_max_displacement_cap = _cap_spin(
            "Max displacement cap (mm):", 3.0, 0.0, 40.0, 0.5, 2,
            "Hard ceiling the per-pass growth can reach (--max-displacement-cap)")
        self.cap_displacement_growth = _cap_spin(
            "Displacement growth:", 1.5, 1.0, 4.0, 0.1, 2,
            "Per-pass multiplier on the displacement budget (--displacement-growth)")
        # #742: its OWN key, deliberately not the Basic tab's `via_size`. That
        # value is three quantities at once on this tab -- the diameter of the
        # vias fanout PLACES, this fallback, and the via floor
        # update_live_drc_floors writes back -- and a plan param named
        # `via_size` would additionally tick the Basic-tab override
        # (ai_plan._GEOMETRY_OVERRIDE_CHECKS), leaking a floor into every later
        # step. Same reasoning as --board-edge-clearance in #772.
        self.cap_default_via_size = _cap_spin(
            "Default via size (mm):", CAP_DEFAULT_VIA_SIZE, 0.05, 2.0, 0.05, 2,
            "Fallback via outer diameter for vias whose size can't be read; "
            "it sets the keep-out radius the cap nudge treats them at "
            "(--default-via-size)")
        # #733 follow-up: the cap repair's OWN board-edge margin. It lives HERE,
        # with the other cap knobs, and NOT on the Basic tab's shared "Min Edge
        # Clearance" control -- that one is the SIGNAL copper-to-edge keep-out,
        # a different quantity that happens to share the CLI flag SPELLING
        # (route.py --board-edge-clearance vs place_fanout_clearance.py
        # --board-edge-clearance, two independent tools). Driving both from one
        # control meant ticking the shared override for signal routing at a
        # normal 0.20-0.25 silently dropped the cap margin from 0.55 to that
        # value, which is the direction #733 exists to close. The CLI can set
        # the two independently; so can this panel.
        self.cap_board_edge_clearance = _cap_spin(
            "Board edge margin (mm):", 0.0, 0.0, 10.0, 0.05, 2,
            "Hard clearance from the board edge for MOVED CAPS "
            "(--board-edge-clearance of place_fanout_clearance.py). "
            "0 = let the engine resolve it: the board's own "
            "min_copper_edge_clearance when it asks for MORE than 0.55mm, "
            "else 0.55mm. This is NOT the Basic tab's Min Edge Clearance, "
            "which is the signal copper-to-edge keep-out.")

        cap_grid.Add(wx.StaticText(self, label="Max passes:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.cap_max_passes = wx.SpinCtrl(self, min=1, max=200, initial=30)
        self.cap_max_passes.SetToolTip("Maximum refinement passes (--max-passes)")
        cap_grid.Add(self.cap_max_passes, 0, wx.EXPAND)

        cap_grid.Add(wx.StaticText(self, label="Movable prefix(es):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.cap_prefix = wx.TextCtrl(self, value="C,R,FB")
        self.cap_prefix.SetToolTip("Comma-separated reference prefix(es) for movable "
                                   "passives near a BGA (--cap-prefix; default "
                                   "C,R,FB = caps, resistors and ferrite beads; "
                                   "RN-style arrays auto-excluded "
                                   "by the 2-copper-pad test)")
        cap_grid.Add(self.cap_prefix, 0, wx.EXPAND)

        cap_sizer.Add(cap_grid, 0, wx.EXPAND | wx.ALL, 5)

        self.cap_allow_rotation = wx.CheckBox(self, label="Allow cap rotation")
        self.cap_allow_rotation.SetValue(True)
        self.cap_allow_rotation.SetToolTip("Allow 90-degree cap rotation to fit (off = --no-rotate)")
        cap_sizer.Add(self.cap_allow_rotation, 0, wx.LEFT | wx.BOTTOM, 5)

        main_sizer.Add(cap_sizer, 0, wx.EXPAND | wx.TOP, 5)

        self.SetSizer(main_sizer)

    def get_escape_method(self) -> str:
        """The engine escape_method value for the current dropdown selection."""
        sel = self.escape_method_choice.GetSelection()
        return self.ESCAPE_METHODS[sel] if 0 <= sel < len(self.ESCAPE_METHODS) else 'auto'

    def set_escape_method(self, value):
        """Set the dropdown from an engine value ('auto'/'channel'/'underpad').

        Unknown values are ignored (dropdown keeps its current selection).
        """
        v = str(value).strip().lower()
        if v in self.ESCAPE_METHODS:
            self.escape_method_choice.SetSelection(self.ESCAPE_METHODS.index(v))

    def _on_differential_changed(self, event):
        """Handle differential checkbox change."""
        is_diff = self.differential_check.GetValue()
        # Notify callback
        if self._on_differential_changed_callback:
            self._on_differential_changed_callback(is_diff)

    def get_config(self):
        """Get the configuration values (BGA-specific only, shared params come from Basic tab)."""
        is_differential = self.differential_check.GetValue()
        return {
            'exit_margin': self.exit_margin.GetValue(),
            'differential': is_differential,
            'diff_pair_patterns': ['*'] if is_differential else [],  # Auto-detect all diff pairs when enabled
            'primary_escape': 'horizontal' if self.escape_direction.GetSelection() == 0 else 'vertical',
            'force_escape_direction': self.force_escape.GetValue(),
            'rebalance_escape': self.rebalance_escape.GetValue(),
            'check_for_previous': self.check_previous.GetValue(),
            'no_inner_top_layer': self.no_inner_top.GetValue(),
            # Dropdown: auto (default, channel + under-pad retry, #288) /
            # channel / underpad - same choices and default as the CLI.
            'escape_method': self.get_escape_method(),
            # #424 plane-ball drops; checkbox bool -> engine 'auto'/'off'.
            'plane_drop': self.plane_drop.GetValue(),
            # Future-pour declaration: raw NET:LAYER[,LAYER...] spec strings
            # (space separated), parsed at the call site like the CLI main.
            'plane_net_layers': self.plane_net_layers_ctrl.GetValue().split()
                                or None,
            'optimize_caps': self.optimize_caps.GetValue(),
            # Decoupling-cap placement (advanced) knobs (#130)
            'cap_capture_radius': self.cap_capture_radius.GetValue(),
            'cap_near_margin': self.cap_near_margin.GetValue(),
            'cap_step': self.cap_step.GetValue(),
            'cap_max_displacement': self.cap_max_displacement.GetValue(),
            'cap_max_displacement_cap': self.cap_max_displacement_cap.GetValue(),
            'cap_displacement_growth': self.cap_displacement_growth.GetValue(),
            # 0 in the spin control is UNSET, not a margin of zero -- None lets
            # the shared engine resolve it, exactly as an omitted CLI flag does.
            'cap_board_edge_clearance': (
                self.cap_board_edge_clearance.GetValue()
                if self.cap_board_edge_clearance.GetValue() > 1e-9 else None),
            'cap_max_passes': self.cap_max_passes.GetValue(),
            'cap_default_via_size': self.cap_default_via_size.GetValue(),
            'cap_prefix': self.cap_prefix.GetValue().strip() or 'C,R,FB',
            'cap_allow_rotation': self.cap_allow_rotation.GetValue(),
        }


class QFNOptionsPanel(wx.ScrolledWindow):
    """QFN fanout options panel (parameters not in Basic tab)."""

    def __init__(self, parent):
        """
        Create QFN options panel.

        Args:
            parent: Parent window
        """
        super().__init__(parent, style=wx.VSCROLL)
        self.SetScrollRate(0, 10)
        self._create_ui()

    def _create_ui(self):
        """Create the panel UI."""
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Info text - QFN uses component's layer automatically
        info_text = wx.StaticText(self, label="QFN fanout routes on the component's layer.")
        info_text.Wrap(350)
        main_sizer.Add(info_text, 0, wx.ALL, 10)

        # Parameters section
        param_box = wx.StaticBox(self, label="QFN Parameters")
        param_sizer = wx.StaticBoxSizer(param_box, wx.VERTICAL)

        grid = wx.FlexGridSizer(cols=2, hgap=10, vgap=5)
        grid.AddGrowableCol(1)

        # #381 D7: QFN-specific track width & clearance, defaulting to the CLI's
        # QFN-tuned 0.1/0.1 (qfn_fanout.py --width/--clearance) instead of
        # inheriting the Basic-tab 0.3/0.25. Fine-pitch QFN/QFP fanouts fail at
        # 0.3 where they succeed at 0.1. Own controls so QFN doesn't inherit the
        # BGA/route width and a plan can still set them (ai_plan routes a QFN
        # fanout step's width/clearance here).
        rw = defaults.PARAM_RANGES['track_width']
        grid.Add(wx.StaticText(self, label="Track width (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.qfn_track_width = wx.SpinCtrlDouble(self, min=rw['min'], max=rw['max'],
                                                 initial=defaults.QFN_TRACK_WIDTH, inc=rw['inc'])
        self.qfn_track_width.SetDigits(rw['digits'])
        self.qfn_track_width.SetToolTip("QFN fanout track width (CLI qfn_fanout --width, default 0.1)")
        grid.Add(self.qfn_track_width, 0, wx.EXPAND)

        rc = defaults.PARAM_RANGES['clearance']
        grid.Add(wx.StaticText(self, label="Clearance (mm):"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.qfn_clearance = wx.SpinCtrlDouble(self, min=rc['min'], max=rc['max'],
                                               initial=defaults.QFN_CLEARANCE, inc=rc['inc'])
        self.qfn_clearance.SetDigits(rc['digits'])
        self.qfn_clearance.SetToolTip("QFN fanout clearance (CLI qfn_fanout --clearance, default 0.1)")
        grid.Add(self.qfn_clearance, 0, wx.EXPAND)

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

        self.underpad_escape = wx.CheckBox(self, label="Under-pad escape (via-drop)")
        self.underpad_escape.SetToolTip(
            "Drop a through-via just past each pad and escape on an inner/back "
            "layer instead of the surface 45-degree fan (#164). For crowded "
            "fine-pitch edges where the surface fan has no room (e.g. a diff pair "
            "boxed in by a neighbour pair and a foreign track). Via size/drill use "
            "the Basic-tab via settings.")
        main_sizer.Add(self.underpad_escape, 0, wx.ALL, 8)

        self.allow_via_in_pad = wx.CheckBox(self, label="Allow via-in-pad (stagger inward)")
        self.allow_via_in_pad.SetToolTip(
            "Under-pad escape only: let the escape via overlap its OWN pad "
            "(via-in-pad), so a leg boxed in on the outward side (a neighbour "
            "pad/track a pitch away) staggers inward toward the chip instead of "
            "being dropped (#161). The via still must clear other-net pads, vias "
            "and tracks.")
        main_sizer.Add(self.allow_via_in_pad, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        self.SetSizer(main_sizer)

    def get_config(self):
        """Get the configuration values (QFN-specific only, shared params come from Basic tab)."""
        return {
            # #381 D7: QFN-tuned width/clearance (own controls, default 0.1/0.1)
            # override the Basic-tab shared values in _run_qfn_fanout.
            'track_width': self.qfn_track_width.GetValue(),
            'clearance': self.qfn_clearance.GetValue(),
            'extension': self.extension.GetValue(),
            'escape_method': 'underpad' if self.underpad_escape.GetValue() else 'stub',
            'allow_via_in_pad': self.allow_via_in_pad.GetValue(),
        }


class FanoutTab(wx.Panel):
    """Complete fanout tab combining component/net selection with options."""

    def __init__(self, parent, pcb_data, board_filename,
                 get_shared_params=None, on_fanout_complete=None,
                 get_connectivity_check=None, sync_pcb_data_callback=None,
                 append_log=None):
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
        # Keeps the dialog's in-memory pcb_data in step with the board after a
        # fanout applies copper (see _apply_fanout_results).
        self.sync_pcb_data_callback = sync_pcb_data_callback
        # Dialog log sink: the fanout engines narrate through print(), and
        # without this tee that narration reached the terminal but never the
        # log tab (the route/diff/planes workers already redirected).
        self.append_log = append_log

        # #621 cancel state. `_cancel_requested` is the flag the plan executor
        # sets through PlanExecutor.stop() (ai_plan._action_owner already maps
        # both "fanout" and "optimize_caps" to this tab), and the one the
        # Cancel button sets directly. `_running` gates the button's dual role
        # and keeps the event pump from re-entering a run.
        self._cancel_requested = False
        self._running = False
        self._fanout_thread = None
        self._operation_result = None
        self._operation_error = None

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
        self.fanout_type.SetToolTip("BGA: Ball Grid Array with via escape\nQFN/QFP: Side pads with outward extension")
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
        self.fanout_btn.SetToolTip("Generate fanout traces and vias for selected component")
        self.fanout_btn.Bind(wx.EVT_BUTTON, self._on_fanout)
        btn_sizer.Add(self.fanout_btn, 1, wx.RIGHT, 5)

        self.close_btn = wx.Button(self, label="Close")
        self.close_btn.SetToolTip("Close dialog (or cancel the fanout if one "
                                  "is running)")
        self.close_btn.Bind(wx.EVT_BUTTON, self._on_cancel_or_close)
        btn_sizer.Add(self.close_btn, 1)

        right_sizer.Add(btn_sizer, 0, wx.EXPAND)

        main_sizer.Add(right_sizer, 1, wx.EXPAND | wx.ALL, 5)

        self.SetSizer(main_sizer)

        # Initial state
        self._on_type_changed(None)

    def _on_close(self, event):
        """Close the parent dialog (matches the other tabs' Close button)."""
        self.GetTopLevelParent().EndModal(wx.ID_CANCEL)

    def _on_cancel_or_close(self, event):
        """Cancel a running fanout, else close -- the planes tab's idiom.

        #621: the fanout engines now take the same cooperative `cancel_check`
        as batch_route / create_plane, and the escape runs on a worker thread,
        so this button has both something to set and a UI thread free to
        deliver the click.
        """
        if self._running:
            self._cancel_requested = True
            self.status_text.SetLabel("Cancelling...")
        else:
            self.GetTopLevelParent().EndModal(wx.ID_CANCEL)

    def _begin_run(self, label):
        """Enter the running state: disable Fanout, arm Cancel, clear the flag.

        `fanout_btn` being disabled is ALSO the plan executor's busy signal
        (ai_plan.py's `_poll_until_idle` watches `fanout_btn.IsEnabled()`), so
        it must go down before the worker starts and only come back up in
        `_end_run`, after the results are applied. Re-enabling it any earlier
        lets the executor start the next step mid-apply -- the hazard the
        planes tab documents at the same place.
        """
        self._running = True
        self._cancel_requested = False
        self._operation_result = None
        self._operation_error = None
        self.fanout_btn.Disable()
        self.close_btn.SetLabel("Cancel")
        self.status_text.SetLabel(label)
        self.progress_bar.Pulse()
        wx.Yield()

    def _end_run(self):
        """Leave the running state, whatever the outcome."""
        self._running = False
        self._fanout_thread = None
        self.fanout_btn.Enable()
        self.close_btn.SetLabel("Close")
        self.progress_bar.SetValue(0)

    def _fanout_worker(self, kind, footprint, kwargs):
        """Run the escape engine OFF the UI thread (#621).

        Everything wx-shaped is resolved by the caller before this starts: the
        worker touches only the engine, `self.pcb_data` (read-only for the
        duration) and the result slots. It deliberately does NOT apply anything
        to the board -- pcbnew mutation happens in `_on_operation_complete`, on
        the UI thread.
        """
        from .gui_utils import redirect_prints_to_log
        # The tee lives INSIDE the worker: `_run_*_fanout` returns as soon as
        # the thread starts, so a redirect installed on the UI thread would be
        # restored while the engine was still printing (and sys.stdout is
        # process-global). Same placement as the planes tab's worker.
        # `append_log` marshals with wx.CallAfter, so this is thread-safe.
        try:
            with redirect_prints_to_log(self.append_log):
                if kind == 'bga':
                    import bga_fanout
                    tracks, vias, vias_to_remove, failed = \
                        bga_fanout.generate_bga_fanout(
                            footprint, self.pcb_data, **kwargs)
                    skipped = list(bga_fanout.LAST_CANCEL_SKIPPED)
                else:
                    import qfn_fanout
                    tracks, vias, failed = qfn_fanout.generate_qfn_fanout(
                        footprint, self.pcb_data, **kwargs)
                    vias_to_remove = None
                    skipped = list(qfn_fanout.LAST_CANCEL_SKIPPED)
                self._operation_result = {
                    'tracks': tracks, 'vias': vias, 'failed': failed,
                    'vias_to_remove': vias_to_remove, 'skipped': skipped}
        except Exception as exc:                                # noqa: BLE001
            import traceback
            traceback.print_exc()
            self._operation_error = exc

    def _poll_operation(self, apply_kw, kind):
        """Poll for worker completion, mirroring the planes tab's loop."""
        if self._fanout_thread is not None and self._fanout_thread.is_alive():
            if self._cancel_requested:
                self.status_text.SetLabel("Cancelling...")
            wx.CallLater(100, self._poll_operation, apply_kw, kind)
        else:
            self._on_operation_complete(apply_kw, kind)

    def _on_operation_complete(self, apply_kw, kind):
        """Apply (or report) the worker's result on the UI thread."""
        try:
            if self._operation_error is not None:
                wx.MessageBox(
                    f"{kind.upper()} fanout failed:\n\n{self._operation_error}",
                    "Fanout Error", wx.OK | wx.ICON_ERROR)
                return
            res = self._operation_result or {}
            # #621: a cancelled run discards, like the planes tab. The engine
            # hands back a coherent partial, but half a fanout applied to the
            # live board is not what pressing Cancel asks for.
            if self._cancel_requested:
                self._report_cancelled(
                    res.get('skipped') or [],
                    kind='ball' if kind == 'bga' else 'pad net')
                return
            # The APPLY phase narrates too -- gui_utils.redirect_prints_to_log
            # exists because every tab's apply ran after its worker restored
            # stdout, so its output reached the terminal but never the log tab.
            from .gui_utils import redirect_prints_to_log
            with redirect_prints_to_log(self.append_log):
                self._apply_fanout_results(
                    res.get('tracks') or [], res.get('vias') or [],
                    failed_nets=res.get('failed'),
                    vias_to_remove=res.get('vias_to_remove'),
                    **apply_kw)
        finally:
            self._end_run()

    def _make_cancel_check(self):
        """The engines' zero-arg cooperative cancel predicate (#621).

        The fanout runs SYNCHRONOUSLY on the UI thread (unlike the planes tab,
        The engine runs on a WORKER thread (`_fanout_worker`), so this is a
        plain flag read -- the UI thread stays free to deliver the Cancel
        click, exactly as on the planes tab. It reads a bool written by another
        thread, which needs no lock: a torn read is impossible for a bool, and
        the worst case is noticing the cancel one loop head later.
        """
        return lambda: self._cancel_requested

    def _report_cancelled(self, skipped, kind='ball'):
        """Tell the user what a cancelled run did and did NOT measure (#621).

        The untried nets are deliberately NOT presented as escape failures: an
        unfinished search measured nothing about them, and reading them as
        failures is what sends someone into a pointless tighter-clearance
        retry. Nothing is applied to the board -- same policy as the planes
        tab, whose cancelled runs also discard.
        """
        self.status_text.SetLabel("Cancelled")
        names = sorted(skipped or ())
        msg = ["Fanout cancelled. Nothing was applied to the board.", ""]
        if names:
            shown = ', '.join(names[:20]) + (' ...' if len(names) > 20 else '')
            msg.append(f"{len(names)} {kind}(s) were never attempted:")
            msg.append("")
            msg.append(shown)
            msg.append("")
            msg.append("These are NOT escape failures -- the search never ran "
                       "on them, so they say nothing about clearance. Re-run "
                       "without cancelling before changing any setting on "
                       "this evidence.")
        else:
            msg.append("The cancel landed before any result was concluded.")
        wx.MessageBox("\n".join(msg), "Fanout Cancelled",
                      wx.OK | wx.ICON_INFORMATION)

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

    def _fanout_status(self, message):
        """Status update for a fanout phase.

        Engine-thread progress reaches the UI through gui_utils.ui_thread_status,
        which marshals off-thread callers with wx.CallAfter and repaints
        narrowly (no Gauge.Pulse) -- see its docstring for why that matters
        inside a KiCad action plugin. Guarded: reporting must never break the
        fanout.
        """
        from .gui_utils import ui_thread_status
        ui_thread_status(getattr(self, 'status_text', None),
                         getattr(self, 'progress_bar', None), message)

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
        self._begin_run("Running BGA fanout...")

        # Get shared parameters from Basic tab (includes layers)
        shared = self.get_shared_params() if self.get_shared_params else {}
        from fab_tiers import set_fab_tier_from_config
        set_fab_tier_from_config(shared)
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
            self._end_run()
            return

        # Everything wx-shaped is read HERE, on the UI thread; the worker
        # gets a plain kwargs dict (#621).
        engine_kw = dict(
            cancel_check=self._make_cancel_check(),
            net_filter=net_patterns,
            diff_pair_patterns=config['diff_pair_patterns'] or None,
            layers=layers,
            track_width=track_width,
            clearance=clearance,
            # BGA_DIFF_PAIR_GAP, and NOT shared['diff_pair_gap'] (#493).
            # Two bugs in one line: the fallback named the signal-routing
            # constant (DIFF_PAIR_GAP 0.101) instead of the fanout one
            # (BGA_DIFF_PAIR_GAP 0.1) -- every neighbouring param here
            # correctly uses its BGA_* default -- and the shared lookup
            # leaked the DIFFERENTIAL tab's _effective_diff_pair_gap() into
            # fanout, which resolves to the board's Default net-class gap
            # when its override box is unchecked. On eth_tap that handed the
            # escape router 0.125 where the CLI's bga_fanout uses 0.1, and
            # the ball field escaped down different channels (BOOT0 at
            # x=123.275 vs 122.625, FPGA_I on F.Cu vs In1.Cu) -- which then
            # cascaded through the whole chain. Same leak class as the
            # no_bga_zone/max_iterations bleed from the route tab into the
            # plane step. bga_fanout.py's --diff-pair-gap likewise defaults
            # to BGA_DIFF_PAIR_GAP and does not consult the net class, so
            # this is the value the recorded chains were routed at.
            diff_pair_gap=defaults.BGA_DIFF_PAIR_GAP,
            exit_margin=config['exit_margin'],
            primary_escape=config['primary_escape'],
            force_escape_direction=config['force_escape_direction'],
            rebalance_escape=config['rebalance_escape'],
            via_size=via_size,
            via_drill=via_drill,
            check_for_previous=config['check_for_previous'],
            no_inner_top_layer=config['no_inner_top_layer'],
            escape_method=config.get('escape_method', 'auto'),
            # #424 plane-ball drops -- checkbox bool -> engine token, same
            # default (on/'auto') as the CLI's --plane-drop.
            plane_drop=('auto' if config.get('plane_drop', True) else 'off'),
            # Same NET:LAYER[,...] spec parse as bga_fanout's main()
            # (review parity finding 5: this kwarg was CLI-only).
            plane_net_layers=(
                {spec.split(':', 1)[0]: spec.split(':', 1)[1].split(',')
                 for spec in config['plane_net_layers']
                 if ':' in spec}
                if config.get('plane_net_layers') else None),
            grid_step=shared.get('grid_step', defaults.GRID_STEP),
            # Shared Basic-tab per-layer costs (issue #288), same values the
            # route/diff tabs use; None when the control is empty/invalid.
            layer_costs=shared.get('layer_costs') or None,
            # Per-ball progress into the status line. Safe from the worker:
            # ui_thread_status marshals off-thread callers with CallAfter.
            # Only the counted x/N lines reach the status feed -- the
            # uncounted phase chatter (gridding, staging, ...) is log-only.
            progress_callback=(lambda c, t, m:
                               self._fanout_status(f"{m} ({c}/{t})")
                               if t else None),
        )
        apply_kw = dict(
            fanout_config={
                'track_width': track_width, 'clearance': clearance,
                'via_size': via_size, 'via_drill': via_drill,
                'exit_margin': config.get('exit_margin'),
                'grid_step': shared.get('grid_step', defaults.GRID_STEP),
                # Advanced cap-placement knobs (#130) so the inline checkbox
                # path honours them too, not just defaults.
                **{k: v for k, v in config.items() if k.startswith('cap_')},
                # #780: ...and the #768 netclass CEILING, which is NOT a
                # cap_* key and so is not swept up by the line above.
                # _optimize_decoupling_caps reads `clearance_ceiling` off
                # THIS dict, and the standalone path
                # (run_cap_optimization) has always supplied it from
                # `shared` -- this one did not, so the INLINE cap pass ran
                # #768's OMITTED branch whatever the operator typed and
                # ticked. Measured on the real headless dialog before this
                # existed: Min Clearance override CHECKED at 0.2 ->
                # get_shared_params carried clearance_ceiling=0.2 and the
                # engine still received netclass_ceiling=None.
                #
                # `clamp_netclasses` rides along for parity with the
                # standalone dict, which has carried it since #768.
                # NOTHING ON THIS TAB READS IT -- grepped: the signal,
                # differential and planes tabs each consume their own copy
                # as `clamp_nondefault_netclasses`, and this tab has no
                # such writeback (#782). It is carried rather than dropped
                # because it is precisely the argument that writeback will
                # need, and because the two values coming from different
                # places is how they came apart here in the first place --
                # but it is inert today, and an earlier draft of this
                # comment implied otherwise.
                'clamp_netclasses': shared.get('clamp_netclasses', False),
                'clearance_ceiling': shared.get('clearance_ceiling'),
                # Shared "Add teardrops" checkbox (#489 section 9).
                'add_teardrops': shared.get('add_teardrops', False),
                # #693: shared "Fix DRC settings after routing" checkbox --
                # the apply path gates its live-floor writeback on this.
                'fix_drc_settings': shared.get('fix_drc_settings', True),
            },
            optimize_caps=config.get('optimize_caps', False),
        )

        self._fanout_thread = threading.Thread(
            target=self._fanout_worker, args=('bga', footprint, engine_kw),
            daemon=True)
        self._fanout_thread.start()
        self._poll_operation(apply_kw, 'bga')

    def _run_qfn_fanout(self, footprint, net_patterns, config):
        """Run QFN fanout."""
        self._begin_run("Running QFN fanout...")

        # Get shared parameters from Basic tab
        shared = self.get_shared_params() if self.get_shared_params else {}
        from fab_tiers import set_fab_tier_from_config
        set_fab_tier_from_config(shared)
        # #381 D7: QFN width/clearance come from the QFN panel's own controls
        # (default 0.1/0.1 = qfn_fanout.py's CLI defaults), NOT the Basic-tab
        # 0.3/0.25 that BGA/route use. `config` is the QFN options config.
        track_width = config.get('track_width', defaults.QFN_TRACK_WIDTH)
        clearance = config.get('clearance', defaults.QFN_CLEARANCE)

        # Get extension from config (QFN-specific parameter)
        extension = config.get('extension', defaults.QFN_EXTENSION)
        # Under-pad (via-drop) escape uses the Basic-tab via settings (#164)
        escape_method = config.get('escape_method', 'stub')
        allow_via_in_pad = config.get('allow_via_in_pad', False)
        via_size = shared.get('via_size', defaults.BGA_VIA_SIZE)
        via_drill = shared.get('via_drill', defaults.BGA_VIA_DRILL)

        # Use the component's layer (F.Cu for top, B.Cu for bottom)
        component_layer = footprint.layer if hasattr(footprint, 'layer') else 'F.Cu'

        engine_kw = dict(
            cancel_check=self._make_cancel_check(),
            net_filter=net_patterns,
            layer=component_layer,
            track_width=track_width,
            extension=extension,
            clearance=clearance,
            grid_step=shared.get('grid_step', defaults.GRID_STEP),
            escape_method=escape_method,
            via_size=via_size,
            via_drill=via_drill,
            allow_via_in_pad=allow_via_in_pad,
            board_edge_clearance=shared.get('board_edge_clearance', 0.0),
            # See the BGA path: safe from the worker via ui_thread_status.
            # Only the counted x/N lines reach the status feed -- the
            # uncounted phase chatter (gridding, staging, ...) is log-only.
            progress_callback=(lambda c, t, m:
                               self._fanout_status(f"{m} ({c}/{t})")
                               if t else None),
        )
        apply_kw = dict(
            fanout_config={
                'track_width': track_width,
                'extension': extension,
                # Shared "Add teardrops" checkbox (#489 section 9).
                'add_teardrops': shared.get('add_teardrops', False),
                # #693: shared "Fix DRC settings after routing" checkbox --
                # the apply path gates its live-floor writeback on this.
                'fix_drc_settings': shared.get('fix_drc_settings', True),
            },
            fanout_kind='qfn',
        )

        self._fanout_thread = threading.Thread(
            target=self._fanout_worker, args=('qfn', footprint, engine_kw),
            daemon=True)
        self._fanout_thread.start()
        self._poll_operation(apply_kw, 'qfn')

    def _apply_fanout_results(self, tracks, vias, failed_nets=None,
                              fanout_config=None, fanout_kind='bga',
                              optimize_caps=False, vias_to_remove=None):
        """Apply fanout results to the pcbnew board.

        Args:
            tracks: list of track dicts to add
            vias: list of via dicts to add
            failed_nets: optional list of net names that couldn't be fanned
                out (BGA) or whose stub endpoints landed too close to
                another net's (QFN) - used to build a suggestion block in
                the completion dialog.
            fanout_config: optional dict of the parameters used so
                suggestions can reference the user's actual values.
            fanout_kind: 'bga' or 'qfn' - selects which suggestion helper
                to use when displaying parameter advice.
            vias_to_remove: optional list of via dicts (x, y, net_id) that
                manage_vias decided are superseded (a route moved to the
                top layer no longer needs its via). The CLI writer strips
                them from the file; the GUI must delete them from the live
                board too (#508 finding 17 -- they were silently discarded).
        """
        import pcbnew
        from .swig_gui import _build_layer_mappings

        board = pcbnew.GetBoard()
        if board is None:
            wx.MessageBox("Board is no longer open", "Error", wx.OK | wx.ICON_ERROR)
            return

        self._fanout_status("Applying fanout copper to the board...")

        # Get layer mappings
        name_to_id, _ = _build_layer_mappings()

        def get_layer_id(layer_name):
            return name_to_id.get(layer_name, pcbnew.F_Cu)

        tracks_added = 0
        vias_added = 0

        # Remove superseded existing vias BEFORE the adds (#508 finding 17).
        if vias_to_remove:
            _rm_keys = {(round(v['x'], 3), round(v['y'], 3), v.get('net_id'))
                        for v in vias_to_remove}
            _n_rm = 0
            for item in list(board.GetTracks()):
                if item.Type() != pcbnew.PCB_VIA_T:
                    continue
                k = (round(pcbnew.ToMM(item.GetPosition().x), 3),
                     round(pcbnew.ToMM(item.GetPosition().y), 3),
                     item.GetNetCode())
                if k in _rm_keys:
                    board.RemoveNative(item)
                    _n_rm += 1
            if _n_rm:
                print(f"Removed {_n_rm} superseded fanout via(s) "
                      f"(top-layer routes, #508)")

        # Add tracks
        for track_dict in tracks:
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(pcbnew.VECTOR2I(
                mm_to_iu(track_dict['start'][0]),
                mm_to_iu(track_dict['start'][1])
            ))
            track.SetEnd(pcbnew.VECTOR2I(
                mm_to_iu(track_dict['end'][0]),
                mm_to_iu(track_dict['end'][1])
            ))
            track.SetWidth(mm_to_iu(track_dict['width']))
            track.SetLayer(get_layer_id(track_dict['layer']))
            track.SetNetCode(track_dict['net_id'])
            board.Add(track)
            tracks_added += 1

        # Add vias
        for via_dict in vias:
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pcbnew.VECTOR2I(
                mm_to_iu(via_dict['x']),
                mm_to_iu(via_dict['y'])
            ))
            via.SetWidth(mm_to_iu(via_dict['size']))
            via.SetDrill(mm_to_iu(via_dict['drill']))
            via.SetNetCode(via_dict['net_id'])
            if 'layers' in via_dict and len(via_dict['layers']) >= 2:
                via.SetLayerPair(
                    get_layer_id(via_dict['layers'][0]),
                    get_layer_id(via_dict['layers'][1])
                )
            # Keep a re-placed via's own tenting/plugging/filling (#489 §8).
            from .gui_utils import apply_via_protection
            apply_via_protection(via, via_dict.get('tenting_attrs'))
            board.Add(via)
            vias_added += 1

        # Refill zones, THEN rebuild connectivity (refill_all_zones does both,
        # in that order). A bare BuildConnectivity here flipped fanout via
        # netcodes to the stale pours' nets (mez_rx: 42 of 131 vias came back
        # V3P3/V1P8/GND) -- see refill_all_zones's docstring.
        self._fanout_status("Refilling zones and rebuilding connectivity...")
        from .gui_utils import refill_all_zones
        refill_all_zones(board)

        # Teardrops, if the shared "Add teardrops" checkbox is on (#489 §9). The
        # CLI writer applies them to the output file; the GUI applies copper into
        # pcbnew, so it has to set them on the board itself.
        if (fanout_config or {}).get('add_teardrops'):
            from .gui_utils import apply_teardrops_to_board
            apply_teardrops_to_board(board)

        # Optionally tidy decoupling caps around the fresh fanout vias (#130)
        cap_summary = None
        if optimize_caps and fanout_kind == 'bga':
            cap_summary = self._optimize_decoupling_caps(
                board, pcbnew, fanout_config or {})
            refill_all_zones(board)   # never bare BuildConnectivity: net flips

        # Sync the dialog's in-memory pcb_data from the board.
        #
        # The fanout tab was the ONLY tab that never did this: it added tracks
        # and vias to the live board but left pcb_data untouched, so every later
        # step routed against a board model missing all the fanout copper. The
        # CLI never sees this because it is file-to-file -- each step re-parses
        # the previous step's output and therefore sees everything.
        #
        # Measured on eth_tap: each fanout step's BOARD matched the CLI
        # bit-exactly, yet by step 11 the GUI laid 3514 segments / 366 vias
        # against the CLI's 3428 / 334 -- more copper, because the router saw
        # fewer obstacles. A 2-step chain (route_diff -> route, pcb_data built
        # fresh) was bit-identical, which is what localized it to the carry
        # rather than to the router.
        self._fanout_status("Syncing board data...")
        if self.sync_pcb_data_callback:
            self.sync_pcb_data_callback()

        # Per-step live DRC floors (GUI twin of bga_fanout's per-step
        # fix_project_for_output). The fanout tab had NO counterpart at all, so
        # a plan whose first steps are fanouts left the Default class at stock
        # values while the CLI had already tightened it: on eth_tap the CLI was
        # at clearance 0.09 / via 0.25-0.15 after step 1, the GUI still at
        # 0.125 / 0.5-0.25 after step 9. Later steps resolve their geometry
        # from that class, so the fronts diverge from there.
        _fcfg = fanout_config or {}
        # #693: gated on the shared "Fix DRC settings after routing" checkbox.
        # This tab is the one whose shared params did not even CARRY the flag,
        # so the gate and the flag were added together -- see the
        # get_shared_params() that feeds FanoutTab in swig_gui.
        if _fcfg.get('fix_drc_settings', True):
            try:
                from .gui_utils import update_live_drc_floors
                _nd_changes = update_live_drc_floors(
                    board,
                    clearance=_fcfg.get('clearance'),
                    track_width=_fcfg.get('track_width'),
                    via_size=_fcfg.get('via_size'),
                    via_drill=_fcfg.get('via_drill'),
                    hole_to_hole=_fcfg.get('hole_to_hole_clearance'),
                    edge_clearance=_fcfg.get('board_edge_clearance'),
                    # #782: the writeback half of #768's GIVEN branch. This tab
                    # priced every class at min(class, ceiling) and then lowered
                    # NONE of them, so a Wide-class pair priced at 0.2 was graded
                    # by KiCad at the still-0.4 class -- violations on copper the
                    # pass considered legal. The CEILING is the value to clamp to
                    # (see the helper's docstring for why not `clearance`), and
                    # it is None exactly when the Min-Clearance override is
                    # unticked, which is #768's OMITTED branch: classes preserved.
                    #
                    # Gated on `clearance_ceiling` ALONE, not on the
                    # `clamp_netclasses` bool beside it in this dict. They are the
                    # same switch read twice (swig_gui sets both off
                    # self.clearance_check), and two values from two places coming
                    # apart is exactly how #780 happened. `_optimize_decoupling_caps`
                    # already gates its pricing on this one value; the writeback
                    # must gate on the same one or the halves can disagree again.
                    nondefault_clamp_mm=_fcfg.get('clearance_ceiling'))
                # Disclosed, because it CHANGES THE BOARD'S DECLARED SPEC and
                # an operator reading the log must see that. Printed only when
                # something actually moved: no ceiling -> empty list -> silence,
                # so an ordinary fanout gains no new output from this fix.
                for _line in (_nd_changes or []):
                    print(f"  {_line}")
            except Exception as _e:
                print(f"(live DRC floor update skipped: {_e})")

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
        if cap_summary:
            msg += "\n" + cap_summary + "\n"
        if failed_nets:
            if fanout_kind == 'qfn':
                msg += f"\nNets whose stubs are too close to neighbours ({len(failed_nets)}):\n"
            else:
                msg += f"\nFailed nets ({len(failed_nets)}):\n"
            for name in failed_nets[:8]:
                msg += f"  - {name}\n"
            if len(failed_nets) > 8:
                msg += f"  ... and {len(failed_nets) - 8} more (see Log tab)\n"
            try:
                from routing_diagnostics import (
                    suggest_bga_fanout_adjustments,
                    suggest_qfn_fanout_adjustments,
                    format_suggestions_for_dialog)
                suggest_fn = (suggest_qfn_fanout_adjustments
                              if fanout_kind == 'qfn'
                              else suggest_bga_fanout_adjustments)
                # Estimate "total" - we don't know the input count here, just
                # use failed + tracks_added as a rough denominator.
                rough_total = max(len(failed_nets) + tracks_added, len(failed_nets))
                suggestions = suggest_fn(
                    failed=len(failed_nets), total=rough_total,
                    config=fanout_config or {})
                block = format_suggestions_for_dialog(suggestions)
                if block:
                    msg += "\n" + block + "\n"
            except Exception as e:
                print(f"Warning: failed to build fanout suggestions: {e}")
        msg += "\nUse Edit -> Undo to revert changes."

        # Routing movie (#506): snapshot the board this step just produced,
        # BEFORE the completion popup blocks on the user. No-op unless the
        # Advanced tab's "Make routing movie" box is ticked.
        from .movie_recorder import record_movie_step
        record_movie_step(self, 'fanout')

        if getattr(getattr(self, 'GetTopLevelParent', lambda: self)(), '_suppress_completion_popups', False):
            print(msg)  # unattended plan run: no per-step OK dialog
        else:
            wx.MessageBox(msg, "Fanout Complete", wx.OK | wx.ICON_INFORMATION)

        # Callback
        if self.on_fanout_complete:
            self.on_fanout_complete()

    def _optimize_decoupling_caps(self, board, pcbnew, fanout_config):
        """Run the fanout-clearance cap repair on the live board (#130).

        Rebuilds PCBData from the just-fanned board (so the new vias are
        present), runs the shared engine, and applies the resulting footprint
        moves directly to pcbnew. Courtyards/locked refs come from the saved
        file (position-independent, unchanged by fanout). Returns a one-line
        summary for the completion dialog, or None on no-op/error.
        """
        try:
            from kicad_parser import build_pcb_data_from_board
            from placement.fanout_clearance import repair_fanout_clearance
            from .swig_gui import _build_layer_mappings

            self.status_text.SetLabel("Optimizing decoupling cap placement...")
            wx.Yield()

            pcb_data = build_pcb_data_from_board(board)
            result = repair_fanout_clearance(
                pcb_data,
                pcb_file=self.board_filename,
                clearance=fanout_config.get('clearance', defaults.BGA_CLEARANCE),
                # #768: the --clearance ceiling. The CLI switches it on the
                # PRESENCE of the flag; a dialog has no "absent", so the switch
                # is the control that already MEANS "I am overriding the board's
                # clearance": the Basic tab's Min Clearance override, exported
                # as `clamp_netclasses` (swig_gui.py, `self.clearance_check`)
                # and consumed as `clamp_nondefault_netclasses` by every other
                # step. ai_plan.py:1279-1282 spells the same equivalence.
                #
                # It is NOT `fix_drc_settings`, which an earlier cut of this
                # change used, on the premise that a checked box means the
                # classes get clamped. Measured, that premise is false:
                # `update_live_drc_floors` writes `m_MinClearance` and the
                # DEFAULT class only, carries no `clamp_nondefault_netclasses`,
                # and this tab never calls `apply_targets_to_board`. Gated
                # there, the GUI priced every pair at the ceiling and clamped no
                # class at all -- pricing on the GIVEN branch and writing back
                # on the OMITTED one, which is #768 pointing the other way.
                #
                # AND HALF OF THAT SURVIVES THE CORRECT GATE (#782), stated
                # here because the paragraph above reads as though choosing
                # the right switch fixed it. It fixed WHICH runs are priced
                # at the ceiling; it did not add the writeback. With the
                # override ticked this tab still prices non-Default classes
                # at min(class, ceiling) and lowers none of them --
                # update_live_drc_floors writes the DEFAULT class only, and
                # this tab never calls fix_project_for_output the way the
                # signal, differential and planes tabs do. A plan run is
                # covered by ai_plan's end-of-run writeback; both
                # INTERACTIVE paths are not. On a single-class board -- most
                # boards -- there is nothing to clamp and no difference.
                #
                # Default False, not True: an absent key means the operator
                # never ticked the override, and the safe reading of that is
                # "honour the board", which is what an omitted CLI flag means.
                netclass_ceiling=fanout_config.get('clearance_ceiling'),
                grid_step=fanout_config.get('grid_step', defaults.GRID_STEP),
                # #733: the plugin used to pass NOTHING here, so it silently took
                # the signature default whatever the board or the operator said,
                # while the cap mover insets by max(clearance, this). None = the
                # engine resolves it, which is what an omitted CLI flag does too.
                board_edge_clearance=fanout_config.get('cap_board_edge_clearance'),
                # #742: the CLI's --default-via-size, on its own key. This used
                # to read `via_size`, which on this tab is the diameter of the
                # vias fanout PLACES and the via floor written back to the
                # project -- a different quantity that happened to reach the
                # same engine parameter, so a recorded run replayed here with a
                # different keep-out radius and therefore different copper.
                default_via_size=fanout_config.get('cap_default_via_size',
                                                   CAP_DEFAULT_VIA_SIZE),
                # Advanced cap-placement knobs from the BGA fanout tab (#130)
                capture_radius=fanout_config.get('cap_capture_radius', 2.0),
                near_margin=fanout_config.get('cap_near_margin', 1.0),
                step=fanout_config.get('cap_step', 0.2),
                max_displacement=fanout_config.get('cap_max_displacement', 2.0),
                max_displacement_cap=fanout_config.get('cap_max_displacement_cap', 3.0),
                displacement_growth=fanout_config.get('cap_displacement_growth', 1.5),
                max_passes=int(fanout_config.get('cap_max_passes', 30)),
                # 'C,R,FB' -- the CLI's default, the engine signature's, and
                # this tab's control value. It read 'C,R' (#742): unreachable,
                # since both call paths populate the key, but a leaner config
                # would have silently stopped moving ferrite beads.
                cap_prefix=fanout_config.get('cap_prefix', 'C,R,FB'),
                allow_rotations=fanout_config.get('cap_allow_rotation', True),
                # Runs ON the UI thread; _fanout_status forces the repaint so
                # the label moves per cap visit instead of freezing (#130).
                # x/N lines only (see the fanout call sites).
                progress_callback=(lambda c, t, m:
                                   self._fanout_status(f"{m} ({c}/{t})")
                                   if t else None),
            )

            # #829: skip a cap that draws the board's own outline. The engine's
            # own cap gate already excludes it, so this should never trigger --
            # but this loop applies poses to the live board directly, without
            # `write_placed_output`, so it is the CLI's raise-on-refusal
            # backstop that is missing on this front and this is where it goes.
            # It PRINTS, because the summary below is built from the engine's
            # `result['placements']` rather than from what was applied, so a
            # silent skip would be reported as a move.
            _outline_skipped = []
            for p in result.get('placements', []):
                fp = board.FindFootprintByReference(p['reference'])
                if fp is None:
                    continue
                _pd = (pcb_data.footprints.get(p['reference'])
                       if pcb_data is not None else None)
                if getattr(_pd, 'owns_board_outline', False):
                    _outline_skipped.append(p['reference'])
                    continue
                fp.SetOrientationDegrees(p['new_rotation'])
                fp.SetPosition(pcbnew.VECTOR2I(
                    mm_to_iu(p['new_x']), mm_to_iu(p['new_y'])))
            if _outline_skipped:
                print(f"  NOT MOVED (#829): {', '.join(_outline_skipped)} -- "
                      f"draws the board outline; moving it would resize the "
                      f"board. The summary below counts the engine's proposal, "
                      f"not what was applied.")

            # Via-nudge with reconnect (#313): the shared engine also moves a
            # boxed-in cap's offending fanout via off the pad and adds connector
            # segment(s) back to the stub start. The CLI applies these via
            # write_placed_output; on the live board we must mirror it (else the
            # via stays put and the graze the summary claims to have fixed
            # persists). GUI parity for the via-nudge block in
            # placement/writer.py (`# Via-nudge rewrites (#313)` to the
            # splice) -- named by its marker comment rather than by line
            # numbers, which this file has now got wrong twice.
            name_to_id, _ = _build_layer_mappings()

            def _layer_id(layer_name):
                return name_to_id.get(layer_name, pcbnew.F_Cu)

            via_moves = result.get('via_moves', []) or []
            for old_x, old_y, vd in via_moves:
                # remove the via at its OLD position, matching NET too (parity
                # with placement/writer._remove_vias_at_positions net_ids, #313):
                # a position-only match could delete a DIFFERENT net's via sitting
                # within 1um of the moved via's old spot.
                # This is the via NUDGE: the old via is deleted and an identical
                # one re-added a fraction of a mm away. Carry its protection spec
                # across or the nudge silently re-tents it (#489 §8).
                #
                # #741: the ENGINE now populates this key, so on this path it
                # is always present -- and legitimately {} for a via that
                # inherits the board's `(setup (tenting ...))`, which
                # apply_via_protection correctly leaves alone.
                #
                # Note the guard below is `if not moved_attrs`: TRUTHINESS,
                # not presence, so it fires for that inheriting via too. It
                # cannot mis-stamp -- apply_via_protection returns early on an
                # empty spec either way -- but it is NOT the regression
                # detector for either half of #741: it re-derives its answer
                # from the same track via the same call that built pcb_data,
                # so it would MASK an engine revert.
                # tests/test_741_via_nudge_tenting.py asserts on the engine
                # dict for exactly that reason.
                #
                # And on KiCad 10.0.0 the re-read is inert for EVERY via, not
                # just an inheriting one: pcbnew's SWIG wrapper does not export
                # TENTING_MODE_TENTED and friends (measured -- the setters
                # exist, the constants do not, and the getters hand back an
                # opaque SwigPyObject), so _pcbnew_via_protection_attrs raises
                # internally and returns {}. That is pre-existing #489
                # behaviour and NOT this fix's doing, but it means the GUI half
                # of the round trip does not currently carry a spec at all.
                # #751.
                moved_attrs = vd.get('tenting_attrs')
                for track in list(board.GetTracks()):
                    if track.GetClass() != 'PCB_VIA':
                        continue
                    if track.GetNetCode() != vd['net_id']:
                        continue
                    pos = track.GetPosition()
                    if (abs(pcbnew.ToMM(pos.x) - old_x) < 1e-3 and
                            abs(pcbnew.ToMM(pos.y) - old_y) < 1e-3):
                        if not moved_attrs:
                            try:
                                # #751: the resolver, not the raw live-object
                                # reader. On a pcbnew whose SWIG wrapper omits
                                # the protection enums the latter answers {}
                                # for EVERY via, so this re-read was inert on
                                # the shipping KiCad 10 rather than only on an
                                # inheriting via.
                                from kicad_parser import (
                                    pcbnew_via_protection_attrs,
                                    via_protection_attrs_from_board_file)
                                moved_attrs = pcbnew_via_protection_attrs(
                                    track,
                                    via_protection_attrs_from_board_file(board))
                            except Exception:
                                moved_attrs = None
                        board.RemoveNative(track)
                        break
                nv = pcbnew.PCB_VIA(board)
                nv.SetPosition(pcbnew.VECTOR2I(
                    mm_to_iu(vd['x']), mm_to_iu(vd['y'])))
                nv.SetWidth(mm_to_iu(vd['size']))
                nv.SetDrill(mm_to_iu(vd['drill']))
                nv.SetNetCode(vd['net_id'])
                if len(vd.get('layers', [])) >= 2:
                    nv.SetLayerPair(_layer_id(vd['layers'][0]),
                                    _layer_id(vd['layers'][1]))
                from .gui_utils import apply_via_protection
                apply_via_protection(nv, moved_attrs)
                board.Add(nv)

            for nsd in (result.get('new_segments', []) or []):
                nt = pcbnew.PCB_TRACK(board)
                nt.SetStart(pcbnew.VECTOR2I(
                    mm_to_iu(nsd['start'][0]), mm_to_iu(nsd['start'][1])))
                nt.SetEnd(pcbnew.VECTOR2I(
                    mm_to_iu(nsd['end'][0]), mm_to_iu(nsd['end'][1])))
                nt.SetWidth(mm_to_iu(nsd['width']))
                nt.SetLayer(_layer_id(nsd['layer']))
                nt.SetNetCode(nsd['net_id'])
                board.Add(nt)

            if via_moves:
                # Re-placed vias sit under the (now stale) pours; a bare
                # BuildConnectivity would flip their netcodes to the zones'.
                from .gui_utils import refill_all_zones
                refill_all_zones(board)

            return cap_optimization_summary(result)
        except Exception as e:
            import traceback
            traceback.print_exc()
            return f"Cap optimization skipped (error: {e})"

    def run_cap_optimization(self, log=None):
        """Standalone decoupling-cap optimization on the live board (#130).

        Entry point for the AI-plan executor's "optimize_caps" step (and
        any caller that wants the repair without running a fanout). Reads the
        clearance/grid/via from the Basic tab and the advanced cap knobs from
        the BGA options panel, runs the shared engine, and applies the moves.
        Synchronous, no modal dialog. Returns a one-line summary or None.
        """
        import pcbnew
        board = pcbnew.GetBoard()
        if board is None:
            return None
        shared = self.get_shared_params() if self.get_shared_params else {}
        cfg = dict(self.bga_options.get_config())  # advanced cap_* knobs
        cfg.update({
            'clearance': shared.get('clearance', defaults.BGA_CLEARANCE),
            'grid_step': shared.get('grid_step', defaults.GRID_STEP),
            # `via_size` used to be here, and it was here for exactly one
            # reason: _optimize_decoupling_caps read it as the engine's
            # `default_via_size`. That was #742's bug -- the Basic tab's via
            # GEOMETRY standing in for the unreadable-via FALLBACK -- and the
            # cap pass now takes `cap_default_via_size` from get_config()
            # above. Nothing on this path reads `via_size` any more, so a row
            # that looks load-bearing under the comment below would be dead.
            # #768: this path builds its config from a HANDFUL of shared keys,
            # so anything the engine call reads off `fanout_config` and that is
            # not listed here silently takes its `.get` default. That is how the
            # first cut of the ceiling gate came to be INERT on the standalone
            # and plan-executor path while looking correct on the inline one --
            # the same shape as the #693 finding the parity ledger records.
            'clamp_netclasses': shared.get('clamp_netclasses', False),
            'clearance_ceiling': shared.get('clearance_ceiling'),
            'fix_drc_settings': shared.get('fix_drc_settings', True),
        })
        from .gui_utils import redirect_prints_to_log, refill_all_zones
        with redirect_prints_to_log(self.append_log):
            summary = self._optimize_decoupling_caps(board, pcbnew, cfg)
            # #782: the STANDALONE button is the second interactive path into
            # the cap pass, and it wrote no DRC settings at all. It prices at the
            # ceiling exactly like the inline path (both read `clearance_ceiling`
            # off their cfg), so it owes the same class writeback -- otherwise
            # which button the operator pressed decides whether the board ships
            # a class the run honoured.
            #
            # The NON-Default clamp only, deliberately, and not the whole
            # `update_live_drc_floors`: this button places parts and draws
            # connectors, it lays no escape copper, and the inline path's floor
            # update exists because the FANOUT wrote tracks and vias. Widening
            # this to the Default class and the size minima is a real change to
            # what the button does and belongs to whoever wants it, not to #782.
            # The Default class is untouched here, so a ceiling BELOW it stays a
            # pricing decision rather than silently retightening the board.
            if cfg.get('clearance_ceiling') is not None:
                try:
                    from fix_kicad_drc_settings import (
                        clamp_nondefault_netclasses_on_board)
                    _nd = clamp_nondefault_netclasses_on_board(
                        board,
                        {'min_clearance': float(cfg['clearance_ceiling'])})
                    if _nd:
                        # The only SetModified on this tab, and it earns the
                        # asymmetry: a net-class edit is a design-SETTINGS
                        # change, and this path can move nothing else at all
                        # (zero caps is a legitimate outcome), so without it a
                        # run whose only effect was the clamp would let the
                        # operator close without being offered the save. The
                        # inline path needs no equivalent -- it has just added
                        # fanout tracks and vias, which mark the board itself.
                        if hasattr(board, 'SetModified'):
                            board.SetModified()
                        print("Non-Default net classes clamped to the "
                              f"{float(cfg['clearance_ceiling']):g}mm ceiling: "
                              + ", ".join(_nd))
                except Exception as _nde:                      # noqa: BLE001
                    print(f"(non-Default net-class clamp skipped: {_nde})")
            refill_all_zones(board)   # never bare BuildConnectivity: net flips
        pcbnew.Refresh()
        if summary:
            self.status_text.SetLabel(summary)
            if log:
                log(summary)
        return summary
