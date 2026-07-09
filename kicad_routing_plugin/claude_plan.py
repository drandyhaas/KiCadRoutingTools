"""
KiCad Routing Tools - Claude routing-plan integration (issue #40)

Turns /plan-pcb-routing output into GUI state and executes it:
- parse_plan_result(): validate the machine-readable JSON plan
- apply_step_params() / apply_step_selection(): fill tab controls so the
  user reviews each step in the tab they already know
- PlanExecutor: run checked steps sequentially through the tabs' own
  in-process machinery (Claude only plans; the GUI routes the live board)
"""

import json
import fnmatch

import wx

KNOWN_ACTIONS = ("fanout", "optimize_caps", "route_diff", "route",
                 "route_planes", "repair_planes")

# Appended to the /plan-pcb-routing prompt so the plan lands as parseable JSON.
PLAN_RESULT_SCHEMA = (
    'RESULT=<compact single-line JSON> with this exact schema: '
    '{"steps": [ '
    '{"action": "fanout", "component": "<ref e.g. U1>", "kind": "bga"|"qfn", '
    '"nets": ["<glob>", ...], '
    '"params": {"escape_method": "auto"|"underpad"|"channel", "exit_margin": <mm>, '
    '"extension": <mm>}} | '
    '{"action": "route_diff", "pairs": ["<pair base name, the net name with its '
    'P/N suffix stripped, e.g. /lvds_rx0>", ...], '
    '"params": {"diff_pair_width": <mm>, "diff_pair_gap": <mm>, '
    '"impedance": <target differential ohms, optional - when set, per-layer '
    'trace width is derived from the stackup and overrides diff_pair_width>, '
    '"layer_costs": [<per-copper-layer cost multiplier, in board layer order>, ...]}} | '
    '{"action": "route", "nets": ["<glob>", ...], '
    '"params": {"track_width": <mm>, "clearance": <mm>, "via_size": <mm>, '
    '"via_drill": <mm>, "power_nets": ["<glob>", ...], '
    '"power_nets_widths": [<mm>, ...], '
    '"layer_costs": [<per-copper-layer cost multiplier, in board layer order>, ...]}} | '
    '{"action": "route_planes", "assignments": [{"nets": ["<exact net name>", ...], '
    '"layer": "<copper layer e.g. In1.Cu>"}], '
    '"params": {"add_gnd_vias": true|false, "gnd_via_distance": <mm>, '
    '"gnd_via_net": "<net name>", "rip_blocker_nets": true|false}} | '
    '{"action": "repair_planes", '
    '"assignments": [{"nets": ["<exact net name>", ...], "layer": "<copper layer>"}], '
    '"params": {"via_size": <mm>, "via_drill": <mm>, "max_track_width": <mm>, '
    '"grid_step": <mm>, "analysis_grid_step": <mm>, "repair_pads": true|false, '
    '"rip_blocker_nets": true|false}} '
    ']} '
    'In any step, params MAY additionally include ANY option shown on that '
    'tab or the shared options panel, keyed by its snake_case field name '
    '(e.g. max_iterations, max_ripup, grid_step, board_edge_clearance, '
    'hole_to_hole_clearance, via_cost, heuristic_weight, turn_cost, '
    'ordering_strategy) - unknown names are ignored with a note. '
    'List steps in execution order: fanout first, then route_diff, then route, '
    'then route_planes, then repair_planes - signals route before planes because '
    'plane stitching vias can adapt around tracks, but a via placed early can '
    'block a diff pair. repair_planes connects disconnected plane regions and taps '
    'plane pads the pour missed. Give it the SAME "assignments" as the route_planes '
    'step (same nets and layers). Include exactly one repair_planes step whenever '
    'there is a route_planes step. Set its rip_blocker_nets true so a plane pad '
    'blocked by a signal trace (e.g. a connector GND pin) is connected by ripping '
    'the blocker out of the way; the ripped blocker is then left UNROUTED. So '
    'whenever rip_blocker_nets is set, add ONE more route step AFTER repair_planes '
    '(nets "*" minus the plane net names) to reconnect the ripped blockers - route '
    'handles rip-up/restore safely against the live obstacle map and reuses the '
    'route step\'s power_nets/power_nets_widths. '
    'The route step\'s "nets" globs support '
    '"!" exclusions and MUST exclude any net that a route_planes step will handle, '
    'e.g. ["*", "!GND", "!VCC"]. '
    'Use only these actions; omit any parameter you have no recommendation for; '
    'all params are optional.'
)


def parse_plan_result(value):
    """Parse the RESULT= JSON plan.

    Returns (steps, errors): steps is a list of validated step dicts (None if
    the value is unusable), errors lists what was rejected or dropped.
    """
    errors = []
    try:
        data = json.loads(value)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        return None, [f"plan is not valid JSON: {e}"]
    if not isinstance(data, dict) or not isinstance(data.get("steps"), list):
        return None, ['plan JSON has no "steps" list']
    steps = []
    for i, step in enumerate(data["steps"]):
        if not isinstance(step, dict):
            errors.append(f"step {i + 1}: not an object, dropped")
            continue
        action = step.get("action")
        if action not in KNOWN_ACTIONS:
            errors.append(f"step {i + 1}: unknown action {action!r}, dropped")
            continue
        if action == "fanout" and not step.get("component"):
            errors.append(f"step {i + 1}: fanout without component, dropped")
            continue
        if action == "route_planes" and not step.get("assignments"):
            errors.append(f"step {i + 1}: route_planes without assignments, dropped")
            continue
        steps.append(step)
    if not steps:
        return None, errors + ["no usable steps in plan"]
    _insert_cap_optimization(steps)
    return steps, errors


def _insert_cap_optimization(steps):
    """Insert one decoupling-cap optimization step right after the last BGA
    fanout (issue #130), unless the plan already has one. The cap engine is
    board-global, so running it once after every BGA's vias are placed clears
    all cap/fanout-via collisions; placing it after the LAST bga fanout (the
    plan lists fanouts first) is exactly that timing. QFN fanouts don't need it.
    """
    if any(s["action"] == "optimize_caps" for s in steps):
        return
    last_bga = None
    for i, s in enumerate(steps):
        if s["action"] == "fanout" and (s.get("kind") or "bga").lower() == "bga":
            last_bga = i
    if last_bga is not None:
        steps.insert(last_bga + 1, {"action": "optimize_caps", "cap_prefix": "C,R,FB"})


def step_label(index, step):
    """Human-readable one-line label for the step list."""
    action = step["action"]
    if action == "fanout":
        kind = step.get("kind", "bga").upper()
        return f"{index}. Fanout {step.get('component')} ({kind})"
    if action == "optimize_caps":
        return f"{index}. Optimize decoupling cap placement"
    if action == "route_diff":
        pairs = step.get("pairs", [])
        shown = ", ".join(pairs[:3]) + (", ..." if len(pairs) > 3 else "")
        return f"{index}. Route diff pairs: {shown or '(all)'}"
    if action == "route":
        nets = step.get("nets", ["*"])
        shown = " ".join(nets[:4]) + (" ..." if len(nets) > 4 else "")
        power = (step.get("params") or {}).get("power_nets")
        suffix = f" (power: {' '.join(power)})" if power else ""
        return f"{index}. Route nets: {shown}{suffix}"
    if action == "route_planes":
        parts = []
        for a in step.get("assignments", []):
            if isinstance(a, dict):
                parts.append(f"{'|'.join(a.get('nets', []))}->{a.get('layer')}")
        return f"{index}. Planes: {', '.join(parts)}"
    if action == "repair_planes":
        return f"{index}. Repair planes (connect regions + tap pads)"
    return f"{index}. {action}"


# --------------------------------------------------------------- application

def apply_step_params(step, dialog):
    """Fill parameter controls from the step (plan time). Returns notes."""
    notes = []
    action = step["action"]
    params = step.get("params") or {}
    # ANY GUI parameter (Andy): a plan step's params may name any control
    # on the step's tab or the shared options panels; resolve by attribute
    # name and coerce by control type. Composite fields with special
    # formatting (power_nets pairs, diff geometry, assignments) are handled
    # by the action-specific blocks below, which run AFTER and win.
    _GENERIC_SKIP = {
        "route": {"track_width", "clearance", "via_size", "via_drill",
                  "power_nets", "power_nets_widths", "layer_costs"},
        "route_diff": {"diff_pair_width", "diff_pair_gap", "impedance",
                       "layer_costs"},
        "route_planes": {"add_gnd_vias", "gnd_via_distance", "gnd_via_net"},
        "repair_planes": set(),
        "fanout": set(),
    }

    def _owners():
        d = dialog
        if action == "route_diff":
            t = getattr(d, "differential_tab", None)
            return [t, d] if t is not None else [d]
        if action == "fanout":
            t = getattr(d, "fanout_tab", None)
            return [t, d] if t is not None else [d]
        if action in ("route_planes", "repair_planes"):
            t = getattr(d, "planes_tab", None)
            subs = []
            if t is not None:
                for sub in ("create_options", "repair_options"):
                    s = getattr(t, sub, None)
                    if s is not None:
                        subs.append(s)
                subs.append(t)
            subs.append(d)
            return subs
        return [d]

    def _set_control(owner, name, value):
        ctrl = getattr(owner, name, None)
        if ctrl is None:
            return False
        import wx
        try:
            if isinstance(ctrl, wx.CheckBox):
                ctrl.SetValue(bool(value))
            elif isinstance(ctrl, (wx.SpinCtrl,)):
                ctrl.SetValue(int(float(value)))
            elif isinstance(ctrl, wx.SpinCtrlDouble):
                ctrl.SetValue(float(value))
            elif isinstance(ctrl, wx.Choice):
                idx = ctrl.FindString(str(value))
                if idx == wx.NOT_FOUND:
                    return False
                ctrl.SetSelection(idx)
            elif hasattr(ctrl, "SetValue"):
                if isinstance(value, (list, tuple)):
                    ctrl.SetValue(" ".join(str(v) for v in value))
                elif isinstance(value, bool):
                    ctrl.SetValue(value)
                else:
                    ctrl.SetValue(str(value))
            else:
                return False
            return True
        except Exception:
            return False

    _skip = _GENERIC_SKIP.get(action, set())
    for name, value in list(params.items()):
        if name in _skip:
            continue
        placed = False
        for owner in _owners():
            if owner is not None and _set_control(owner, name, value):
                notes.append(f"set {name}={value}")
                placed = True
                break
        if not placed and name not in _skip:
            notes.append(f"no control for {name}, ignored")

    if action == "route":
        for name in ("track_width", "clearance", "via_size", "via_drill"):
            if name in params:
                try:
                    getattr(dialog, name).SetValue(float(params[name]))
                except (TypeError, ValueError):
                    notes.append(f"ignored non-numeric {name}={params[name]!r}")
        power = params.get("power_nets")
        widths = params.get("power_nets_widths")
        if power:
            if widths and len(widths) == len(power):
                dialog.power_nets_ctrl.SetValue(" ".join(str(p) for p in power))
                dialog.power_widths_ctrl.SetValue(" ".join(f"{float(w):g}" for w in widths))
            else:
                notes.append("power_nets/widths count mismatch, fields not filled")
        costs = params.get("layer_costs")
        if costs:
            try:
                dialog.layer_costs_ctrl.SetValue(" ".join(f"{float(c):g}" for c in costs))
            except (TypeError, ValueError):
                notes.append(f"ignored non-numeric layer_costs={costs!r}")
    elif action == "route_diff":
        tab = dialog.differential_tab
        if "diff_pair_width" in params or "diff_pair_gap" in params:
            # Explicit values from the plan override netclass-derived ones
            tab.use_netclass_check.SetValue(False)
            tab.diff_pair_width.Enable(True)
            tab.diff_pair_gap.Enable(True)
        for name in ("diff_pair_width", "diff_pair_gap"):
            if name in params:
                try:
                    getattr(tab, name).SetValue(float(params[name]))
                except (TypeError, ValueError):
                    notes.append(f"ignored non-numeric {name}={params[name]!r}")
        # Impedance-controlled diff routing: per-layer width is derived from the
        # stackup, so this overrides diff_pair_width (the diff tab control above).
        if "impedance" in params:
            try:
                tab.diff_impedance.SetValue(float(params["impedance"]))
            except (TypeError, ValueError):
                notes.append(f"ignored non-numeric impedance={params['impedance']!r}")
        # layer_costs lives on the shared Basic-tab control; the Differential tab
        # reads it via get_routing_config, so set it here too (issue #193).
        costs = params.get("layer_costs")
        if costs:
            try:
                dialog.layer_costs_ctrl.SetValue(" ".join(f"{float(c):g}" for c in costs))
            except (TypeError, ValueError):
                notes.append(f"ignored non-numeric layer_costs={costs!r}")
    elif action == "route_planes":
        opts = dialog.planes_tab.create_options
        if "add_gnd_vias" in params:
            opts.add_gnd_vias_check.SetValue(bool(params["add_gnd_vias"]))
        if "gnd_via_distance" in params:
            try:
                opts.gnd_via_distance.SetValue(float(params["gnd_via_distance"]))
            except (TypeError, ValueError):
                notes.append(f"ignored non-numeric gnd_via_distance={params['gnd_via_distance']!r}")
        if "gnd_via_net" in params:
            opts.gnd_via_net.SetValue(str(params["gnd_via_net"]))
        if "rip_blocker_nets" in params:
            opts.rip_blocker_check.SetValue(bool(params["rip_blocker_nets"]))
    elif action == "repair_planes":
        # via size/drill and the routing grid step come from the shared Basic-tab
        # controls (same as route); the repair-specific knobs live on the Repair
        # options panel.
        for name in ("via_size", "via_drill", "grid_step"):
            if name in params:
                try:
                    getattr(dialog, name).SetValue(float(params[name]))
                except (TypeError, ValueError):
                    notes.append(f"ignored non-numeric {name}={params[name]!r}")
        opts = dialog.planes_tab.repair_options
        for name in ("max_track_width", "analysis_grid_step"):
            if name in params:
                ctrl = opts.max_track_width if name == "max_track_width" else opts.analysis_grid
                try:
                    ctrl.SetValue(float(params[name]))
                except (TypeError, ValueError):
                    notes.append(f"ignored non-numeric {name}={params[name]!r}")
        if "repair_pads" in params:
            opts.repair_pads.SetValue(bool(params["repair_pads"]))
        if "rip_blocker_nets" in params:
            opts.rip_blocker_check.SetValue(bool(params["rip_blocker_nets"]))
    elif action == "fanout":
        kind = (step.get("kind") or "bga").lower()
        if kind == "bga":
            opts = dialog.fanout_tab.bga_options
            if "escape_method" in params:
                opts.set_escape_method(params["escape_method"])
            if "exit_margin" in params:
                try:
                    opts.exit_margin.SetValue(float(params["exit_margin"]))
                except (TypeError, ValueError):
                    notes.append(f"ignored non-numeric exit_margin={params['exit_margin']!r}")
        else:
            opts = dialog.fanout_tab.qfn_options
            if "extension" in params:
                try:
                    opts.extension.SetValue(float(params["extension"]))
                except (TypeError, ValueError):
                    notes.append(f"ignored non-numeric extension={params['extension']!r}")
    return notes


def apply_step_selection(step, dialog):
    """Apply the step's net/pair/component/assignment selection (also re-run
    right before executing the step, since consecutive steps of the same
    action share one tab's selection state). Returns notes."""
    notes = []
    action = step["action"]
    if action == "route":
        globs = step.get("nets") or ["*"]
        names = _match_net_names(dialog.pcb_data, globs)
        if not names:
            notes.append(f"route: no nets match {globs}")
        dialog.net_panel.set_selected_nets(names)
    elif action == "route_diff":
        tab = dialog.differential_tab
        wanted = step.get("pairs") or ["*"]
        matched_display = set()
        for display_name, base_name, _p, _n in tab.pair_panel.all_pairs:
            if any(fnmatch.fnmatch(base_name, w) or base_name == w for w in wanted):
                matched_display.add(display_name)
        if not matched_display:
            notes.append(f"route_diff: no pairs match {wanted}")
        tab.pair_panel._checked_pairs = matched_display
        tab.pair_panel._update_pair_list(sync_from_visible=False)
    elif action == "fanout":
        tab = dialog.fanout_tab
        ref = step.get("component")
        kind = (step.get("kind") or "bga").lower()
        tab.fanout_type.SetSelection(0 if kind == "bga" else 1)
        tab._on_type_changed(None)
        if not _select_component(tab.net_panel, ref):
            notes.append(f"fanout: component {ref} not in dropdown")
        globs = step.get("nets") or ["*"]
        names = _component_net_names(dialog.pcb_data, ref, globs)
        if not names:
            notes.append(f"fanout: no nets match {globs} on {ref}")
        tab.net_panel.set_selected_nets(names)
    elif action == "route_planes":
        tab = dialog.planes_tab
        tab.mode_selector.SetSelection(0)  # Create Planes
        tab._on_mode_changed(None)
        assignments = _plane_assignments_from_step(step, dialog, notes, "route_planes")
        if not assignments:
            notes.append("route_planes: no valid assignments")
        tab.assignment_panel.set_assignments(assignments)
    elif action == "repair_planes":
        # Repair runs on the same planes tab in Repair mode. The GUI repair
        # requires net->layer assignments (unlike the CLI, which auto-detects the
        # poured zones). Use the step's own assignments if given; otherwise keep
        # the ones the preceding route_planes step left in the panel.
        tab = dialog.planes_tab
        tab.mode_selector.SetSelection(1)  # Repair Disconnected
        tab._on_mode_changed(None)
        if step.get("assignments"):
            assignments = _plane_assignments_from_step(step, dialog, notes, "repair_planes")
            if assignments:
                tab.assignment_panel.set_assignments(assignments)
        if not tab.assignment_panel.get_assignments():
            notes.append("repair_planes: no assignments (add a route_planes step first, "
                         "or include assignments on the repair step)")
    return notes


def _plane_assignments_from_step(step, dialog, notes, action_name):
    """Build (nets_list, layers_list) assignment tuples from a plane step's
    "assignments", validating nets and copper layers against the board."""
    copper = set(dialog.pcb_data.board_info.copper_layers)
    net_names = {net.name for net in dialog.pcb_data.nets.values() if net.name}
    assignments = []
    for a in step.get("assignments", []):
        if not isinstance(a, dict):
            continue
        # Accept a single "layer" or a "layers" list from the plan; the
        # assignment panel stores (nets_list, layers_list) tuples.
        layers = a.get("layers") if isinstance(a.get("layers"), list) else [a.get("layer")]
        valid_layers = [l for l in layers if l in copper]
        nets = [n for n in a.get("nets", []) if n in net_names]
        unknown = [n for n in a.get("nets", []) if n not in net_names]
        if unknown:
            notes.append(f"{action_name}: unknown nets {unknown} dropped")
        if not valid_layers:
            notes.append(f"{action_name}: no valid copper layers in {layers}, assignment dropped")
            continue
        if nets:
            assignments.append((nets, valid_layers))
    return assignments


def _match_net_names(pcb_data, globs):
    """Match net names against include globs, minus "!" exclusion globs
    (CLI semantics: the plan's route step excludes plane nets as "!GND").

    Uses the shared split_net_patterns helper so a literal active-low net name
    like "!RESET" stays selectable rather than being read as an exclusion
    (issue #177)."""
    from net_queries import split_net_patterns
    known_names = {net.name for net in pcb_data.nets.values() if net.name}
    includes, excludes = split_net_patterns(globs, known_names)
    if not includes:
        includes = ["*"]
    names = []
    for net in pcb_data.nets.values():
        if not net.net_id or not net.name:
            continue
        if any(fnmatch.fnmatch(net.name, g) for g in includes) and \
                not any(fnmatch.fnmatch(net.name, g) for g in excludes):
            names.append(net.name)
    return names


def _component_net_names(pcb_data, ref, globs):
    footprint = pcb_data.footprints.get(ref)
    if footprint is None:
        return []
    names = set()
    for pad in footprint.pads:
        if pad.net_id and pad.net_name and any(fnmatch.fnmatch(pad.net_name, g) for g in globs):
            names.add(pad.net_name)
    return sorted(names)


def _select_component(net_panel, ref):
    dropdown = net_panel.component_dropdown
    if not dropdown or not ref:
        return False
    for i in range(dropdown.GetCount()):
        if dropdown.GetString(i).split(' (')[0] == ref:
            dropdown.SetSelection(i)
            net_panel._on_component_dropdown_changed(None)
            return True
    return False


# ----------------------------------------------------------------- executor

class PlanExecutor:
    """Runs checked plan steps sequentially through the tabs' own machinery.

    Each step: re-apply its selection, switch to its tab, invoke the tab's
    own action handler, then poll the tab's action button (every tab disables
    it while working and re-enables on completion, including error paths)
    until the operation finishes. Stops on the first failed step.

    Callbacks (all on the main thread):
      on_status(step_index, status)  status in 'running' | 'done' | 'failed'
      on_finished(completed_count, aborted_reason_or_None)
    """

    POLL_MS = 400
    # Polls to wait for an operation to visibly start before concluding the
    # handler declined to run (e.g. a validation popup) or finished instantly.
    START_GRACE_POLLS = 5

    def __init__(self, dialog, steps, indices, on_status, on_finished,
                 log=None, on_progress=None):
        self.dialog = dialog
        self.steps = steps
        self.indices = list(indices)
        self.on_status = on_status
        self.on_finished = on_finished
        self.on_progress = on_progress
        self.log = log or (lambda message: None)
        self._queue = []
        self._completed = 0
        self._stop_requested = False
        self._step_started = None

    def start(self):
        # The plan sequences its own route_planes steps, so the route step's
        # "create planes first?" offer (which jumps to the Planes tab and
        # aborts routing) must not fire during an automated run.
        self.dialog._suppress_plane_offer = True
        self._queue = list(self.indices)
        self._next_step()

    def stop(self):
        """Stop before the next step starts (the running one finishes)."""
        self._stop_requested = True

    # -- per-action wiring ---------------------------------------------------

    def _status_source(self, action):
        """The (status_text, progress_bar) pair of the tab actually doing
        the work, so the Claude tab's status bar can MIRROR it live -- a
        route_diff step shows exactly what the differential tab shows."""
        d = self.dialog
        owner = {
            "route": d,
            "route_diff": getattr(d, "differential_tab", None),
            "fanout": getattr(d, "fanout_tab", None),
            "optimize_caps": getattr(d, "fanout_tab", None),
            "route_planes": getattr(d, "planes_tab", None),
            "repair_planes": getattr(d, "planes_tab", None),
        }.get(action)
        if owner is None:
            return None, None
        return (getattr(owner, "status_text", None),
                getattr(owner, "progress_bar", None))

    def _action_parts(self, action):
        """(invoke callable, busy predicate) for an action. The handlers run
        fine without their tab being the visible page, so execution stays on
        the Claude tab."""
        d = self.dialog
        return {
            "route": (lambda: d._on_route(None),
                      lambda: not d.route_btn.IsEnabled()),
            "route_diff": (lambda: d.differential_tab._on_route(None),
                           lambda: not d.differential_tab.route_btn.IsEnabled()),
            "fanout": (lambda: d.fanout_tab._on_fanout(None),
                       lambda: not d.fanout_tab.fanout_btn.IsEnabled()),
            # Synchronous (no worker thread); the start-grace period in
            # _poll_until_idle covers its instant completion.
            "optimize_caps": (lambda: d.fanout_tab.run_cap_optimization(log=self.log),
                              lambda: False),
            "route_planes": (lambda: d.planes_tab._on_action(None),
                             lambda: not d.planes_tab.action_btn.IsEnabled()),
            # Same planes tab + action button; apply_step_selection has already
            # switched it to Repair mode, so _on_action runs the repair.
            "repair_planes": (lambda: d.planes_tab._on_action(None),
                              lambda: not d.planes_tab.action_btn.IsEnabled()),
        }[action]

    # -- sequencing ----------------------------------------------------------

    def _finish(self, aborted_reason):
        self.dialog._suppress_plane_offer = False
        self._write_drc_floors()
        self.on_finished(self._completed, aborted_reason)

    def _write_drc_floors(self):
        """CLI parity (gap #2): every CLI step records its routed floors in
        the sibling .kicad_pro (fix_project_for_output); a GUI plan run used
        to leave the project file untouched, so a manual DRC graded at stock
        defaults. Best-effort; never blocks plan completion."""
        try:
            import os
            import clearance_ledger
            board_file = getattr(self.dialog, 'board_filename', None)
            # Gather every floor the CLI records (route.py passes clearance,
            # hole_to_hole, edge_clearance, track_width, via size/drill;
            # route_diff adds the pair geometry) from the plan's steps --
            # smallest value wins where steps disagree, like the ledger.
            floors = {}

            def _take(key, val, smallest=True):
                try:
                    v = float(val)
                except (TypeError, ValueError):
                    return
                if key not in floors or (smallest and v < floors[key]):
                    floors[key] = v
            for step in self.steps:
                p = step.get('params') or {}
                for k in ('clearance', 'track_width', 'via_size', 'via_drill',
                          'hole_to_hole_clearance', 'board_edge_clearance',
                          'diff_pair_width', 'diff_pair_gap'):
                    if p.get(k) is not None:
                        _take(k, p[k])
            clearance = floors.get('clearance')
            track_width = floors.get('track_width')
            if clearance is None:
                return
            eff = clearance_ledger.effective(clearance)
            # LIVE settings first: KiCad holds project settings in memory,
            # so editing the .kicad_pro on disk is invisible to a DRC run
            # right after the plan (and liable to be clobbered when KiCad
            # saves the project). The design-settings API updates the open
            # session immediately and persists on the user's next save.
            try:
                import pcbnew
                board = pcbnew.GetBoard()
                if board is not None:
                    bds = board.GetDesignSettings()
                    bds.m_MinClearance = pcbnew.FromMM(eff)
                    if track_width:
                        bds.m_TrackMinWidth = pcbnew.FromMM(track_width)
                    if floors.get('via_size'):
                        bds.m_ViasMinSize = pcbnew.FromMM(floors['via_size'])
                    if floors.get('via_drill'):
                        bds.m_MinThroughDrill = pcbnew.FromMM(
                            floors['via_drill'])
                    if floors.get('hole_to_hole_clearance'):
                        bds.m_HoleToHoleMin = pcbnew.FromMM(
                            floors['hole_to_hole_clearance'])
                    if floors.get('board_edge_clearance'):
                        bds.m_CopperEdgeClearance = pcbnew.FromMM(
                            floors['board_edge_clearance'])
                    try:
                        for _name, _nc in board.GetNetClasses().items():
                            if _name == 'Default':
                                _nc.SetClearance(pcbnew.FromMM(eff))
                                if track_width:
                                    _nc.SetTrackWidth(
                                        pcbnew.FromMM(track_width))
                                if floors.get('via_size'):
                                    _nc.SetViaDiameter(
                                        pcbnew.FromMM(floors['via_size']))
                                if floors.get('via_drill'):
                                    _nc.SetViaDrill(
                                        pcbnew.FromMM(floors['via_drill']))
                                if floors.get('diff_pair_width'):
                                    _nc.SetDiffPairWidth(pcbnew.FromMM(
                                        floors['diff_pair_width']))
                                if floors.get('diff_pair_gap'):
                                    _nc.SetDiffPairGap(pcbnew.FromMM(
                                        floors['diff_pair_gap']))
                    except Exception:
                        pass
                    self.log(f"Claude plan: live DRC settings updated "
                             f"(min clearance {eff:.4g}mm, "
                             f"{len(floors)} floor(s))")
            except Exception as e:
                self.log(f"Claude plan: live DRC settings skipped: {e}")
            # Best-effort persistence for a later close/reopen; note KiCad
            # may overwrite this if it saves its in-memory project state.
            if board_file and os.path.isfile(board_file):
                from fix_kicad_drc_settings import fix_project_for_output
                fix_project_for_output(
                    board_file, input_pcb=board_file,
                    clearance=eff,
                    track_width=track_width,
                    via_diameter=floors.get('via_size'),
                    via_drill=floors.get('via_drill'),
                    hole_to_hole=floors.get('hole_to_hole_clearance'),
                    edge_clearance=floors.get('board_edge_clearance'),
                    diff_pair_width=floors.get('diff_pair_width'),
                    diff_pair_gap=floors.get('diff_pair_gap'))
                self.log(f"Claude plan: recorded DRC floors in the project "
                         f"file (clearance {eff:.4g}; live session already "
                         f"updated via the API)")
        except Exception as e:
            self.log(f"Claude plan: DRC floor write skipped: {e}")

    def _next_step(self):
        if self._stop_requested:
            self._finish("stopped by user")
            return
        if not self._queue:
            self._finish(None)
            return
        index = self._queue.pop(0)
        step = self.steps[index]
        self.on_status(index, "running")
        self.log(f"Claude plan: step {index + 1} ({step['action']}) starting")
        try:
            # Re-apply BOTH this step's parameters and its selection right before
            # running it: consecutive steps of the same action share one tab's
            # controls, so plan-time fill leaves only the last such step's
            # track_width/clearance/via/diff-pair geometry in place. Without this
            # re-apply, e.g. a fine-pitch route step and a general route step
            # would both run at whichever was applied last.
            notes = apply_step_params(step, self.dialog)
            notes += apply_step_selection(step, self.dialog)
            for note in notes:
                self.log(f"Claude plan: {note}")
            invoke, busy = self._action_parts(step["action"])
            import time as _time
            self._step_started = _time.time()
            invoke()
        except Exception as e:
            self.on_status(index, "failed")
            self.log(f"Claude plan: step {index + 1} failed: {e}")
            self._finish(f"step {index + 1} raised: {e}")
            return
        self._poll_until_idle(index, busy, polls=0, seen_busy=False)

    def _poll_until_idle(self, index, busy, polls, seen_busy):
        try:
            is_busy = busy()
        except RuntimeError:
            # A control died (dialog closing) - abort quietly
            self._finish("dialog closed")
            return
        if self.on_progress is not None:
            # Mirror the working tab's own status bar (label + gauge) into
            # the Claude tab, with per-step elapsed time.
            try:
                import time as _time
                st, pb = self._status_source(self.steps[index]["action"])
                label = st.GetLabel() if st is not None else ""
                val = pb.GetValue() if pb is not None else 0
                rng = pb.GetRange() if pb is not None else 100
                elapsed = _time.time() - (self._step_started or _time.time())
                self.on_progress(index, self.steps[index], label, val, rng,
                                 elapsed, is_busy)
            except Exception:
                pass
        if is_busy:
            wx.CallLater(self.POLL_MS, self._poll_until_idle, index, busy, polls + 1, True)
            return
        if not seen_busy and polls < self.START_GRACE_POLLS:
            # Not busy yet: either finished instantly or hasn't started.
            # Give it a short grace period before declaring completion.
            wx.CallLater(self.POLL_MS, self._poll_until_idle, index, busy, polls + 1, False)
            return
        self._completed += 1
        self.on_status(index, "done")
        self.log(f"Claude plan: step {index + 1} finished")
        self._next_step()
