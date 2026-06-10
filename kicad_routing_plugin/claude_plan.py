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

KNOWN_ACTIONS = ("fanout", "route_diff", "route", "route_planes")

# Appended to the /plan-pcb-routing prompt so the plan lands as parseable JSON.
PLAN_RESULT_SCHEMA = (
    'RESULT=<compact single-line JSON> with this exact schema: '
    '{"steps": [ '
    '{"action": "fanout", "component": "<ref e.g. U1>", "kind": "bga"|"qfn", '
    '"nets": ["<glob>", ...]} | '
    '{"action": "route_diff", "pairs": ["<pair base name, the net name with its '
    'P/N suffix stripped, e.g. /lvds_rx0>", ...], '
    '"params": {"diff_pair_width": <mm>, "diff_pair_gap": <mm>}} | '
    '{"action": "route", "nets": ["<glob>", ...], '
    '"params": {"track_width": <mm>, "clearance": <mm>, "via_size": <mm>, '
    '"via_drill": <mm>, "power_nets": ["<glob>", ...], '
    '"power_nets_widths": [<mm>, ...]}} | '
    '{"action": "route_planes", "assignments": [{"nets": ["<exact net name>", ...], '
    '"layer": "<copper layer e.g. In1.Cu>"}], '
    '"params": {"add_gnd_vias": true|false, "gnd_via_distance": <mm>}} '
    ']} '
    'List steps in execution order: fanout first, then route_diff, then route, '
    'then route_planes - signals route before planes because plane stitching '
    'vias can adapt around tracks, but a via placed early can block a diff '
    'pair. The route step\'s "nets" globs support "!" exclusions and MUST '
    'exclude any net that a route_planes step will handle, e.g. '
    '["*", "!GND", "!VCC"]. '
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
    return steps, errors


def step_label(index, step):
    """Human-readable one-line label for the step list."""
    action = step["action"]
    if action == "fanout":
        kind = step.get("kind", "bga").upper()
        return f"{index}. Fanout {step.get('component')} ({kind})"
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
    return f"{index}. {action}"


# --------------------------------------------------------------- application

def apply_step_params(step, dialog):
    """Fill parameter controls from the step (plan time). Returns notes."""
    notes = []
    action = step["action"]
    params = step.get("params") or {}
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
    elif action == "route_planes":
        opts = dialog.planes_tab.create_options
        if "add_gnd_vias" in params:
            opts.add_gnd_vias_check.SetValue(bool(params["add_gnd_vias"]))
        if "gnd_via_distance" in params:
            try:
                opts.gnd_via_distance.SetValue(float(params["gnd_via_distance"]))
            except (TypeError, ValueError):
                notes.append(f"ignored non-numeric gnd_via_distance={params['gnd_via_distance']!r}")
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
                notes.append(f"route_planes: unknown nets {unknown} dropped")
            if not valid_layers:
                notes.append(f"route_planes: no valid copper layers in {layers}, assignment dropped")
                continue
            if nets:
                assignments.append((nets, valid_layers))
        if not assignments:
            notes.append("route_planes: no valid assignments")
        tab.assignment_panel.set_assignments(assignments)
    return notes


def _match_net_names(pcb_data, globs):
    """Match net names against include globs, minus "!" exclusion globs
    (CLI semantics: the plan's route step excludes plane nets as "!GND")."""
    includes = [g for g in globs if not g.startswith("!")] or ["*"]
    excludes = [g[1:] for g in globs if g.startswith("!")]
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

    def __init__(self, dialog, steps, indices, on_status, on_finished, log=None):
        self.dialog = dialog
        self.steps = steps
        self.indices = list(indices)
        self.on_status = on_status
        self.on_finished = on_finished
        self.log = log or (lambda message: None)
        self._queue = []
        self._completed = 0
        self._stop_requested = False

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
            "route_planes": (lambda: d.planes_tab._on_action(None),
                             lambda: not d.planes_tab.action_btn.IsEnabled()),
        }[action]

    # -- sequencing ----------------------------------------------------------

    def _finish(self, aborted_reason):
        self.dialog._suppress_plane_offer = False
        self.on_finished(self._completed, aborted_reason)

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
            notes = apply_step_selection(step, self.dialog)
            for note in notes:
                self.log(f"Claude plan: {note}")
            invoke, busy = self._action_parts(step["action"])
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
