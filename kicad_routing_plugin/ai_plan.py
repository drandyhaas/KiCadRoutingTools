"""
KiCad Routing Tools - AI routing-plan integration (issue #40)

Turns /plan-pcb-routing output into GUI state and executes it:
- parse_plan_result(): validate the machine-readable JSON plan
- apply_step_params() / apply_step_selection(): fill tab controls so the
  user reviews each step in the tab they already know
- PlanExecutor: run checked steps sequentially through the tabs' own
  in-process machinery (the AI only plans; the GUI routes the live board)
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
    '"gnd_via_net": "<net name>", '
    '"stitch_vias": true|false, "stitch_pitch": <mm, default 20>, '
    '"stitch_max_freq": <MHz, derives pitch as lambda/20 from the stackup>, '
    '"stitch_edge_fence": true|false, "stitch_fence_pitch": <mm>, '
    '"stitch_inset": <mm>}} | '
    ']} '
    'In any step, params MAY additionally include ANY option shown on that '
    'tab or the shared options panel, keyed by its snake_case field name '
    '(e.g. max_iterations, max_ripup, ripup_abandon_metric, grid_step, board_edge_clearance, '
    'hole_to_hole_clearance, via_cost, heuristic_weight, turn_cost, '
    'ordering_strategy) - unknown names are ignored with a note. '
    'List the steps in execution order - the executor runs the array in that '
    'order. Take the routing order and step composition from the skill you '
    'just ran; do not re-derive them here. '
    'Use only these actions; omit any parameter you have no recommendation for; '
    'all params are optional.'
)

# NOTE: this string is the GUI's MACHINE CONTRACT only -- the RESULT= line, the
# action/param schema, and the tab-control passthrough rule. It must NOT restate
# routing doctrine. It used to, and the copy drifted: it said "fanout first ...
# then route_planes" while calling itself "the #562 pours-first chain", and
# because it is appended AFTER the skill's own text it WON. The plans the GUI
# produced therefore fanned out before pouring -- the opposite of what the skill
# says in four places ("Pour the planes FIRST - before fanout, before any
# routing") -- so the GUI and the CLI generated different chains from the same
# skill. On an 8-layer board (mez_rx) that cost every GND ball its pour-direct
# skip (#424 prints "N pour-covered (no via needed)"; measured 104 of 127 balls
# on a 285-ball BGA), left the plane-drop vias with no pour to land on, and
# handed the pour a fanout-shredded board to flood around.
#
# Anything about ORDER, step composition, or which nets go where belongs in
# .claude/skills/plan-pcb-routing/SKILL.md, which both fronts read. Only add
# text here when the GUI executor genuinely cannot act without it.


def _join_nets(values):
    """Space-join net names for a whitespace-separated GUI field, quoting any
    that contain spaces. KiCad net names may ('/Management Interface/VDDA');
    unquoted, one such name splits into two on read and every consumer that
    pairs names with widths/groups mismatches (#493)."""
    import shlex
    return " ".join(shlex.quote(str(v)) for v in values)


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
    _append_final_plane_verify(steps)
    return steps, errors


def _insert_cap_optimization(steps):
    """Insert one decoupling-cap optimization step right after the last BGA
    fanout (issue #130), unless the plan already has one. The cap engine is
    board-global, so running it once after every BGA's vias are placed clears
    all cap/fanout-via collisions; placing it after the LAST bga fanout is
    exactly that timing (found by scan, so it does not care where the fanouts
    sit relative to the pour). QFN fanouts don't need it.

    Once-after-all is the skill's DEFAULT cadence (Step 1c), not a choice
    between two: per-BGA runs compound cap displacement (each run re-seeds at
    the already-moved position) and change what later fanouts route around
    (cap pads are escape obstacles). A plan that already carries its own
    optimize_caps step(s) is left exactly as the skill produced it.
    """
    if any(s["action"] == "optimize_caps" for s in steps):
        return
    last_bga = None
    for i, s in enumerate(steps):
        if s["action"] == "fanout" and (s.get("kind") or "bga").lower() == "bga":
            last_bga = i
    if last_bga is not None:
        # NO params, deliberately: this step INHERITS the preceding fanout's
        # live control state, which is what the plan executor's reset exception
        # exists to preserve.
        #
        # #772: it used to carry a TOP-LEVEL "cap_prefix": "C,R,FB" -- outside
        # `params`, which apply_step_params never reads -- so it was inert while
        # reading, in the plan JSON and in review, as though the prefix were
        # being set. Nothing anywhere reads a top-level step key of that name
        # (checked). Moving it INTO params would be WORSE than dropping it:
        # since #772 a cap step that names a cap knob gets the SCOPED cap reset,
        # so the auto-inserted step would stop inheriting the operator's
        # interactive cap tweaks -- the one behaviour it is documented to have.
        # The panel's own default is 'C,R,FB' regardless, so nothing changes.
        steps.insert(last_bga + 1, {"action": "optimize_caps"})


def _append_final_plane_verify(steps):
    """Late-pinch guard (#479 wave finding): copper-modifying steps that run
    AFTER the last plane step can sever plane fill. Since #562 every route
    step ENDS with the in-run plane finalize (taps + region joins + cleanup +
    kicad-oracle verify), so the guard is simply: make sure a route step runs
    last. When copper actions follow the last plane step and none of them is
    a route step, append ONE final route step with nets ["*"] -- on an intact
    board it skips every connected net and the finalize verifies the planes
    (fast no-op); when a late step did pinch a pour, the finalize heals it."""
    # `repair_planes` is NOT a plane step any more -- the executor skips it
    # as a no-op (#562). Counting it as one re-opened the very hole this
    # guard exists to close: a LEGACY plan ending in `route -> repair_planes`
    # set last_plane to that final index, so tail was empty, the guard
    # returned early, and the trailing step did nothing -- leaving the plan
    # with NO plane verify at all, where pre-#562 that step WAS the verify.
    plane_actions = {"route_planes"}
    inert_actions = {"repair_planes"}     # legacy, skipped at run time
    last_plane = None
    for i, s in enumerate(steps):
        if s["action"] in plane_actions:
            last_plane = i
    if last_plane is None:
        return
    tail = [s for s in steps[last_plane + 1:]
            if s["action"] not in inert_actions]
    # Since #562 a pour ALONE connects nothing: the plane step places no
    # taps, so plane pads are welded by the route step's pour-launch and
    # completed by its finalize. So the rule is not merely "a route must run
    # after a late copper step" -- it is "a route MUST run after the last
    # pour", full stop. A legacy plan of pour -> repair_planes (where the
    # repair used to be both the weld and the verify) otherwise pours and
    # then does nothing at all.
    #
    # AND the trailing route's net scope must COVER the pour nets (review
    # finding F1): the finalize filters its zone-net scope by the route's
    # nets, so a plan ending in a SCOPED route (`nets: ["SDRAM_*"]`) after a
    # GND pour welds nothing on GND and ships every GND pad disconnected
    # while the run reports success for its own scope. Trusting any trailing
    # route step was this guard's blind spot -- check the coverage.
    if tail and tail[-1]["action"] == "route":
        _pour_nets = set()
        for s in steps:
            if s["action"] in plane_actions:
                for a in (s.get("assignments") or []):
                    if isinstance(a, dict):
                        _pour_nets.update(a.get("nets") or [])
        _route_nets = tail[-1].get("nets", ["*"])
        try:
            from net_queries import matches_net_filter as _mnf
            _covered = all(_mnf(n, _route_nets) for n in _pour_nets)
        except Exception:
            _covered = "*" in _route_nets
        if _covered:
            return  # its finalize welds and verifies the planes
        # Fall through: append the verify step (nets ["*"] covers the pours).
    # Reuse the last route step's params so the verify routes at the chain's
    # own geometry (clearance/track/via/power widths).
    src_params = {}
    for s in reversed(steps):
        if s["action"] == "route":
            src_params = dict(s.get("params") or {})
            break
    steps.append({"action": "route", "nets": ["*"], "params": src_params})
    return True


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
        return (f"{index}. Repair planes (obsolete step, skipped -- repair "
                f"runs inside every route step since #562)")
    return f"{index}. {action}"


# --------------------------------------------------------------- application

# --- #381 D5: param -> GUI-control resolution tables (module level so the no-wx
# parity gate can extract them by AST without importing wx). A plan param whose
# GUI home is not a same-named control on the step's tab must appear here, or it
# falls to "no control, ignored" and the reset default silently applies.
#
# _PARAM_CONTROL_ALIASES: param name -> the differently-named control attribute
# that _set_control() targets (resolved on the step's owners in order).
_PARAM_CONTROL_ALIASES = {
    # (rip_blocker_nets / repair_pads / analysis_grid_step had controls on
    # the plane panels that #562 deleted -- the pour step does no routing and
    # the repair step is gone. Their per-action blocks note them instead.)
    # #381 D3: route_diff's polarity-swap allowlist.
    'polarity_swap_nets': 'polarity_swap_nets_text',
    # #381 D5: route.py options that previously fell to "no control, ignored"
    # (so a plan-replayed #284-class rip-existing / ordering / keepout chain
    # silently routed at the reset defaults instead of the recorded values).
    'rip_existing_nets': 'rip_existing_nets_ctrl',
    'ordering': 'ordering_strategy',
    'direction': 'direction_choice',
    'time_matching': 'time_matching_check',
    'keepout': 'keepout_check',
    'guide_corridor': 'guide_corridor_check',
    # #486: coplanar-waveguide declaration (the gap itself is a same-named
    # SpinCtrlDouble and needs no alias).
    'coplanar_nets': 'coplanar_nets_ctrl',
    # #489 section 9: one shared checkbox drives teardrops on every step.
    'add_teardrops': 'add_teardrops_check',
    # Review parity finding 5: bga_fanout's future-pour declaration
    # (NET:LAYER[,...] specs). List param -> space-joined into the text ctrl.
    'plane_net_layers': 'plane_net_layers_ctrl',
    # #237: plans converted BEFORE manifest_to_plan mapped --fab-overrides
    # carry it under the fallthrough name `fab_overrides`; the control is
    # fab_overrides_path. New conversions emit fab_overrides_path directly.
    'fab_overrides': 'fab_overrides_path',
}
# _PARAM_SPECIAL: params handled by _apply_special() (composite / inverted /
# panel-backed controls that a plain SetValue can't fill).
_PARAM_SPECIAL = {'layers', 'no_bga_zone', 'no_bga_zones', 'power_nets',
                  'power_nets_widths', 'escape_method', 'no_gnd_vias',
                  # #381 D5:
                  'impedance', 'length_match_groups', 'swappable_nets',
                  # #486:
                  'coplanar_nets',
                  # review parity finding 5:
                  'plane_net_layers'}

# #439: geometry-floor param -> its Basic-tab override checkbox attribute. A plan
# step that names one of these is the GUI equivalent of the CLI passing that flag,
# so setting the spinctrl must ALSO check the override box and enable the control
# (otherwise _effective_<name>() ignores the typed value and uses the board's own).
# The edge control's checkbox is named edge_clearance_check, not *_check.
_GEOMETRY_OVERRIDE_CHECKS = {
    'track_width': 'track_width_check',
    'clearance': 'clearance_check',
    'via_size': 'via_size_check',
    'via_drill': 'via_drill_check',
    'hole_to_hole_clearance': 'hole_to_hole_clearance_check',
    'board_edge_clearance': 'edge_clearance_check',
    # #435: diff-tab geometry overrides (differential panel). A plan step setting
    # diff_pair_width/gap == the CLI passing --track-width/--diff-pair-gap, so the
    # override box must be checked -- otherwise _effective_* ignores the typed value
    # and each pair uses its OWN netclass diff geometry (the omitted-flag default).
    'diff_pair_width': 'diff_pair_width_check',
    'diff_pair_gap': 'diff_pair_gap_check',
}


def _enable_geometry_override(dialog, name):
    """Check the override box + enable the spinctrl for a geometry floor set from
    a plan step (no-op if `name` is not a geometry floor or the control is absent)."""
    chk_attr = _GEOMETRY_OVERRIDE_CHECKS.get(name)
    if not chk_attr:
        return
    chk = getattr(dialog, chk_attr, None)
    if chk is not None:
        chk.SetValue(True)
    ctrl = getattr(dialog, name, None)
    if ctrl is not None and hasattr(ctrl, 'Enable'):
        ctrl.Enable(True)


# #772: action -> (tab attribute, sub-panel attributes searched BEFORE the tab).
# The generic loop resolves a param by walking these owners IN ORDER and taking
# the first one carrying a same-named control, so this table decides which
# controls a plan step can reach AT ALL.
#
# `optimize_caps` had no entry and fell through to [dialog], while every one of
# the eleven "Cap Placement (advanced)" controls lives on fanout_tab.bga_options.
# Measured on the real headless dialog before this landed: TEN of the eleven
# params a converted manifest carries were logged "no control, ignored" and the
# engine ran at its signature defaults (capture_radius 2.0 for a plan's 5.0,
# max_passes 30 for 7, cap_prefix 'C,R,FB' for 'C', allow_rotations True for
# False). Only `clearance` arrived -- correctly, see _GENERIC_SKIP below.
#
# A MODULE-LEVEL TABLE rather than the if/elif chain it replaces, for two
# reasons. ONE: the wx-free parity gate can AST-extract it, exactly as it does
# _PARAM_CONTROL_ALIASES, and assert OWNER-SCOPED reachability. That check did
# not exist, which is why #772 shipped -- check_param_resolution only asks "does
# a control with this name exist ANYWHERE across the four GUI files", and every
# cap_* control has always existed, throughout the entire period not one of them
# was reachable. TWO: swig_gui.reset_params_to_defaults already descends into
# these same sub-panels, and its own comment names this bug class. Apply and
# reset must agree about what a step can touch; two hand-written lists in two
# files did not, and that disagreement IS #772.
_ACTION_OWNERS = {
    'route_diff': ('differential_tab', ()),
    # #772: the fanout action reaches its option PANELS too, which is what
    # swig_gui.reset_params_to_defaults has always done (its `_fctl` holder
    # search, whose comment names this exact bug class). Apply and reset now
    # agree about what a fanout step can touch; before, the per-action block
    # reached bga_options by hand for three params and the generic loop
    # could reach neither panel.
    #
    # Measured on the real headless dialog before widening: the live
    # control-name sets of RoutingDialog (96), FanoutTab (1),
    # BGAOptionsPanel (20) and QFNOptionsPanel (5) are pairwise DISJOINT
    # except `progress_bar`, a wx.Gauge on both the dialog and the tab that
    # no plan param is named after and that the fanout action could already
    # reach. So the panels shadow nothing.
    'fanout': ('fanout_tab', ('bga_options', 'qfn_options')),
    'optimize_caps': ('fanout_tab', ('bga_options',)),
    'route_planes': ('planes_tab', ('create_options',)),
    'repair_planes': ('planes_tab', ('create_options',)),
    # `route` is deliberately absent: its controls are all on the dialog.
}


def apply_step_params(step, dialog):
    """Fill parameter controls from the step (plan time). Returns notes."""
    notes = []
    action = step["action"]
    params = step.get("params") or {}
    if action == "repair_planes":
        # Obsolete step (#562): the executor skips it, so applying its params
        # is not merely pointless -- the generic loop below calls
        # _enable_geometry_override, TICKING the Basic-tab override
        # checkboxes (via size/drill, ...) for a step that then does nothing.
        # Those overrides persist into the NEXT step and silently change its
        # geometry. Apply nothing.
        return ["repair_planes params ignored (#562: the step is a skipped "
                "no-op; applying them would leak geometry overrides into "
                "the next step)"]
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
        # rip_blocker_nets: the control is GONE (#562, the pour step does no
        # routing, and the repair step is obsolete); the per-action block
        # prints an explanatory note instead of letting the generic loop emit
        # a bare "no control, ignored".
        "route_planes": {"add_gnd_vias", "gnd_via_distance", "gnd_via_net",
                         "rip_blocker_nets"},
        "repair_planes": {"rip_blocker_nets"},
        # #381 D7: QFN width/clearance are set by the fanout action block onto
        # the QFN panel's own controls; skip them in the generic loop (which has
        # no same-named control on the fanout owners) to avoid a spurious
        # "no control, ignored" note.
        "fanout": {"qfn_track_width", "qfn_clearance"},
        # #772: on a CAP step, `board_edge_clearance` is
        # place_fanout_clearance.py's flag, whose GUI home is the BGA
        # panel's cap_board_edge_clearance -- NOT the Basic tab's
        # same-named SIGNAL copper-to-edge keep-out, a different quantity
        # that merely shares the flag SPELLING across two independent
        # tools (the #733 follow-up split them apart on purpose). Left to
        # the generic loop it landed on the signal control AND ticked
        # edge_clearance_check -- measured: `edge_clearance_check = True,
        # board_edge_clearance = 0.85` after the step, which then leaks
        # into the NEXT step's routing -- while the cap engine still
        # received None. The optimize_caps block below re-homes it.
        #
        # NOT `clearance`, which is CORRECT through the generic loop:
        # setting the Basic tab's Min Clearance and ticking its override
        # is exactly the GUI's spelling of "--clearance was GIVEN"
        # (#768). Measured on the real dialog, a cap step's
        # `clearance: 0.1` arrives as both clearance=0.1 AND
        # netclass_ceiling=0.1. Skipping it would break that branch.
        #
        # #742 adds `via_size` for the same shape of reason. On a cap step the
        # CLI flag is `--default-via-size`, whose GUI home is now the panel's
        # cap_default_via_size -- NOT the Basic tab's via GEOMETRY, which sets
        # the diameter of the vias fanout PLACES. Left to the generic loop a
        # plan naming `via_size` on an optimize_caps step lands on that
        # control, ticks via_size_check through _GEOMETRY_OVERRIDE_CHECKS, and
        # is harvested by _write_drc_floors into the project. Before #742 that
        # at least reached the cap engine (run_cap_optimization forwarded it);
        # now it reaches nothing, so it would be a pure leak. Same fix as
        # --board-edge-clearance got, on the same reasoning.
        "optimize_caps": {"board_edge_clearance", "via_size"},
    }

    def _owners():
        # Owner search order for this action (see _ACTION_OWNERS). An
        # action with no entry -- `route` -- resolves on the dialog only.
        #
        # Behaviour-identical to the if/elif chain this replaced for the
        # four actions that had one: route_diff -> [differential_tab, d]
        # or [d]; fanout -> [fanout_tab, d] or [d]; route_planes and
        # repair_planes -> [create_options?, planes_tab, d] or [d];
        # anything unlisted -> [d]. Only optimize_caps changes.
        d = dialog
        tab_attr, subs = _ACTION_OWNERS.get(action, (None, ()))
        if not tab_attr:
            return [d]
        t = getattr(d, tab_attr, None)
        if t is None:
            return [d]
        out = [p for p in (getattr(t, name, None) for name in subs)
               if p is not None]
        out.append(t)
        out.append(d)
        return out

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

    # Param -> GUI-control resolution (module-level tables so the no-wx parity
    # gate can introspect them; see _PARAM_CONTROL_ALIASES / _PARAM_SPECIAL).
    _SPECIAL = _PARAM_SPECIAL
    _ALIASES = _PARAM_CONTROL_ALIASES

    def _apply_special(name, value):
        if name == 'layers' and isinstance(value, (list, tuple)):
            checks = getattr(dialog, 'layer_checks', None)
            if not checks:
                return False
            wanted = {str(v) for v in value}
            for layer, cb in checks.items():
                cb.SetValue(layer in wanted)
            return True
        if name in ('no_bga_zone', 'no_bga_zones'):
            # route.py spells it --no-bga-zones (plural); accept both the
            # singular and the plural param name so plans emitted by an older
            # converter (unknown-flag -> 'no_bga_zones') or by the live LLM
            # still reach the control instead of being "ignored".
            ctl = getattr(dialog, 'no_bga_zones_ctrl', None)
            if ctl is None:
                return False
            ctl.SetValue('ALL' if value else '')
            return True
        if name == 'power_nets' and isinstance(value, (list, tuple)):
            ctl = getattr(dialog, 'power_nets_ctrl', None)
            if ctl is None:
                return False
            ctl.SetValue(_join_nets(value))
            return True
        if name == 'coplanar_nets':
            # #486: list of glob patterns -> the space-separated text control.
            ctl = getattr(dialog, 'coplanar_nets_ctrl', None)
            if ctl is None:
                return False
            ctl.SetValue(_join_nets(value) if isinstance(value, (list, tuple))
                         else str(value or ''))
            return True
        if name == 'plane_net_layers':
            # Review parity finding 5: NET:LAYER[,...] spec list -> the
            # space-separated fanout text control (specs contain no spaces).
            ctl = getattr(dialog, 'plane_net_layers_ctrl', None)
            if ctl is None:
                return False
            ctl.SetValue(' '.join(value) if isinstance(value, (list, tuple))
                         else str(value or ''))
            return True
        if name == 'power_nets_widths' and isinstance(value, (list, tuple)):
            ctl = getattr(dialog, 'power_widths_ctrl', None)
            if ctl is None:
                return False
            ctl.SetValue(' '.join(str(v) for v in value))
            return True
        if name == 'no_gnd_vias':
            # route_diff's GND-return-via toggle: the CLI flag is the NEGATIVE
            # --no-gnd-vias, while the differential tab's checkbox is the
            # POSITIVE "Add GND vias" (gnd_via_check, default on) -- invert so a
            # recorded --no-gnd-vias actually unchecks it. (route_planes'
            # add_gnd_vias is a separate control, handled in its own block.)
            chk = getattr(getattr(dialog, 'differential_tab', None),
                          'gnd_via_check', None)
            if chk is None:
                return False
            chk.SetValue(not bool(value))
            return True
        if name == 'no_thermal_vias':
            # route_planes' NEGATIVE flag (--no-thermal-vias, default-on
            # BooleanOptionalAction) vs the POSITIVE planes-tab checkbox --
            # invert, like no_gnd_vias above.
            chk = getattr(getattr(getattr(dialog, 'planes_tab', None),
                                  'create_options', None), 'thermal_vias', None)
            if chk is None:
                return False
            chk.SetValue(not bool(value))
            return True
        if name == 'escape_method':
            # Fanout escape dropdown lives on the BGA options panel and shows
            # DISPLAY strings ("Auto (channel, under-pad retry)"), while the
            # plan/CLI value is the engine token ('auto'/'channel'/'underpad').
            # Map value -> index via the panel's ESCAPE_METHODS tuple.
            opts = getattr(getattr(dialog, 'fanout_tab', None), 'bga_options', None)
            choice = getattr(opts, 'escape_method_choice', None)
            if choice is None:
                return False
            methods = getattr(type(opts), 'ESCAPE_METHODS', ('auto', 'channel', 'underpad'))
            v = str(value).lower()
            if v in methods:
                choice.SetSelection(methods.index(v))
                return True
            return False
        if name == 'plane_drop':
            # #424: the CLI value is a token ('auto'/'off'); the BGA panel
            # control is a checkbox. A generic SetValue('off') would be truthy,
            # so coerce explicitly.
            opts = getattr(getattr(dialog, 'fanout_tab', None), 'bga_options', None)
            chk = getattr(opts, 'plane_drop', None)
            if chk is None:
                return False
            chk.SetValue(str(value).strip().lower() not in ('off', '0', 'false', 'no'))
            return True
        if name == 'impedance':
            # #381 D5: route.py's --impedance drives a checkbox+value pair on the
            # Basic tab (impedance_check enables impedance-based width). A plain
            # SetValue on a same-named control doesn't exist, so a route step's
            # impedance previously fell to "no control, ignored". (route_diff's
            # impedance is a separate diff_impedance control, handled in the
            # route_diff action block and skipped from this generic loop.)
            chk = getattr(dialog, 'impedance_check', None)
            val = getattr(dialog, 'impedance_value', None)
            if chk is None or val is None:
                return False
            try:
                val.SetValue(float(value))
                chk.SetValue(True)
                return True
            except (TypeError, ValueError):
                return False
        if name == 'length_match_groups':
            # #381 D5: route.py's --length-match-group lives in a single text
            # field parsed by _parse_length_match_groups (groups comma-separated,
            # patterns within a group space-separated). Accept a plain string, a
            # flat list (one group), or a list-of-groups and format accordingly.
            ctl = getattr(dialog, 'length_match_groups_ctrl', None)
            if ctl is None:
                return False
            if isinstance(value, str):
                text = value
            elif value and isinstance(value[0], (list, tuple)):
                text = ', '.join(_join_nets(g) for g in value)
            else:
                text = _join_nets(value or [])
            ctl.SetValue(text)
            return True
        if name == 'swappable_nets':
            # #381 D5: route.py's --swappable-nets. The GUI expresses swappable
            # nets as a checkbox panel keyed by REAL net names (not globs), so a
            # plan carrying explicit net names selects them; glob patterns won't
            # match panel entries (a known GUI limitation -- the panel has no
            # pattern field). Best-effort: check the named nets.
            panel = getattr(dialog, 'swappable_net_panel', None)
            if panel is None:
                return False
            try:
                panel._checked_nets = {str(v) for v in (value or [])}
                if hasattr(panel, 'refresh'):
                    panel.refresh(sync_from_visible=False)
                return True
            except Exception:
                return False
        return False

    _skip = _GENERIC_SKIP.get(action, set())
    for name, value in list(params.items()):
        if name in _skip:
            continue
        if name in _SPECIAL:
            if _apply_special(name, value):
                notes.append(f"set {name}={value}")
            else:
                notes.append(f"no control for {name}, ignored")
            continue
        lookup = _ALIASES.get(name, name)
        placed = False
        for owner in _owners():
            if owner is not None and _set_control(owner, lookup, value):
                notes.append(f"set {name}={value}")
                # #439: a plan value for a geometry floor enables its override.
                _enable_geometry_override(dialog, lookup)
                placed = True
                break
        if not placed:
            notes.append(f"no control for {name}, ignored")

    if action == "route":
        for name in ("track_width", "clearance", "via_size", "via_drill"):
            if name in params:
                try:
                    getattr(dialog, name).SetValue(float(params[name]))
                    # #439: an explicit plan value == the CLI passing the flag; check
                    # the override box + enable so _effective_<name>() uses it.
                    _enable_geometry_override(dialog, name)
                    notes.append(f"set {name}={params[name]}")
                except (TypeError, ValueError):
                    notes.append(f"ignored non-numeric {name}={params[name]!r}")
        power = params.get("power_nets")
        widths = params.get("power_nets_widths")
        if power:
            if widths and len(widths) == len(power):
                # Quote names containing spaces: the control is whitespace
                # separated and KiCad net names may contain spaces
                # ('/Management Interface/VDDA'). Unquoted, one such net split
                # into two on read, the power-net/width counts disagreed, and
                # identify_power_nets raised inside the routing worker -- the
                # step then reported FINISHED having routed nothing (#493).
                dialog.power_nets_ctrl.SetValue(_join_nets(power))
                dialog.power_widths_ctrl.SetValue(" ".join(f"{float(w):g}" for w in widths))
                notes.append(f"set power_nets={list(power)} widths={list(widths)}")
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
        # An explicit diff_pair_width/gap in the plan overrides the board
        # net-class value: check that param's override box and enable its spinctrl
        # so _effective_diff_pair_width/gap return the plan value (an omitted param
        # leaves the box unchecked -> board Default net-class value is used).
        for name in ("diff_pair_width", "diff_pair_gap"):
            if name in params:
                getattr(tab, name + "_check").SetValue(True)
                getattr(tab, name).Enable(True)
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
        if "zone_clearance" in params and params["zone_clearance"] is not None:
            _pop = getattr(dialog.planes_tab, "create_options", None)
            if _pop is not None and hasattr(_pop, "zone_clearance_check"):
                # explicit plan value checks the override box (basic-tab
                # convention: checked = use the typed value)
                _pop.zone_clearance_check.SetValue(True)
                if hasattr(_pop, "zone_clearance"):
                    _pop.zone_clearance.Enable(True)
        opts = dialog.planes_tab.create_options
        # A plan step is a COMPLETE spec of feature toggles: absent means
        # OFF. Leaving the persisted/panel state in place let a previously
        # enabled 'Add GND vias' leak into a loaded stress-manifest plan
        # that never asked for stitching (Andy's bitaxe DRC2 grazes).
        opts.add_gnd_vias_check.SetValue(bool(params.get("add_gnd_vias")))
        if hasattr(opts, "thermal_relief"):
            opts.thermal_relief.SetValue(bool(params.get("thermal_relief")))
        if hasattr(opts, "thermal_vias"):
            # Default-ON param: absent means the DEFAULT (True), unlike the
            # absent-means-off feature toggles above. A recorded
            # --no-thermal-vias arrives as no_thermal_vias (see the alias).
            import routing_defaults as _rd
            opts.thermal_vias.SetValue(bool(params.get(
                "thermal_vias", not params.get("no_thermal_vias",
                                               not _rd.THERMAL_VIAS))))
        if not params.get("add_gnd_vias"):
            notes.append("add_gnd_vias off (not in plan step)")
        if "gnd_via_distance" in params:
            try:
                opts.gnd_via_distance.SetValue(float(params["gnd_via_distance"]))
            except (TypeError, ValueError):
                notes.append(f"ignored non-numeric gnd_via_distance={params['gnd_via_distance']!r}")
        if "gnd_via_net" in params:
            opts.gnd_via_net.SetValue(str(params["gnd_via_net"]))
        if "rip_blocker_nets" in params:
            notes.append("rip_blocker_nets ignored (#562: the pour step does "
                         "no routing, so there are no tap corridors to rip)")
    elif action == "repair_planes":
        # Obsolete since #562: plane repair runs inside every route step's
        # finalize. Legacy plans keep loading; the step is skipped at run
        # time (see _action_trigger) and its params are ignored here.
        notes.append("repair_planes step is obsolete (#562) -- skipped; "
                     "plane repair runs inside every route step's finalize")
    elif action == "fanout":
        kind = (step.get("kind") or "bga").lower()
        if kind == "bga":
            opts = dialog.fanout_tab.bga_options
            if "escape_method" in params:
                opts.set_escape_method(params["escape_method"])
            if "plane_drop" in params:
                # CLI token 'auto'/'off' (or a plan bool) -> checkbox (#424)
                opts.plane_drop.SetValue(
                    str(params["plane_drop"]).strip().lower()
                    not in ('off', '0', 'false', 'no'))
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
            # #381 D7: QFN width/clearance live on the QFN panel's own controls
            # (default 0.1/0.1), not the Basic tab. manifest_to_plan maps
            # qfn_fanout --width/--clearance to qfn_track_width/qfn_clearance.
            for _pname, _ctl in (("qfn_track_width", "qfn_track_width"),
                                 ("qfn_clearance", "qfn_clearance")):
                if _pname in params:
                    try:
                        getattr(opts, _ctl).SetValue(float(params[_pname]))
                    except (TypeError, ValueError):
                        notes.append(f"ignored non-numeric {_pname}={params[_pname]!r}")
    elif action == "optimize_caps":
        # Every cap_* param resolves through the generic loop now that
        # bga_options is one of this action's owners (#772). Only the
        # LEGACY spelling needs a block: plans converted before #772 carry
        # place_fanout_clearance's --board-edge-clearance as
        # `board_edge_clearance`, on the (false) claim that
        # _GEOMETRY_OVERRIDE_CHECKS would carry it to the engine. Re-home
        # it onto the cap knob, and do NOT touch edge_clearance_check --
        # ticking that is what leaked a PLACEMENT margin into the next
        # step's routing keep-out.
        _opts = getattr(getattr(dialog, "fanout_tab", None),
                        "bga_options", None)
        _ctl = getattr(_opts, "cap_board_edge_clearance", None)
        if "board_edge_clearance" in params and _ctl is not None:
            try:
                _ctl.SetValue(float(params["board_edge_clearance"]))
                notes.append(
                    f"board_edge_clearance={params['board_edge_clearance']}"
                    f" -> cap_board_edge_clearance (#772: on a cap step"
                    f" this is the PLACEMENT margin, not the Basic tab's"
                    f" signal copper-to-edge keep-out)")
            except (TypeError, ValueError):
                notes.append("ignored non-numeric board_edge_clearance="
                             f"{params['board_edge_clearance']!r}")
    return notes


def _plan_plane_nets(steps, dialog):
    """Plane nets a step running BEFORE the plane steps must not route/fan
    out as signals: the union of every route_planes/repair_planes step's
    exact assignment net names in the PLAN (a whole-chain declaration, so it
    covers fanout/route steps that run before the planes exist -- the
    ottercast stub-clutter class). Steps AFTER the planes may include these
    nets freely: the engine's fill-aware selection skips a plane-connected
    net untouched and track-patches only genuinely disconnected pads (#479)."""
    nets = set()
    for s in steps or []:
        if s.get("action") in ("route_planes", "repair_planes"):
            for a in s.get("assignments") or []:
                nets.update(n for n in a.get("nets") or [] if n)
            nets.update(n for n in s.get("nets") or [] if n)
    return nets


def _precedes_first_plane_step(step, all_steps):
    """True when `step` runs before the plan's first route_planes/repair_planes
    step (or when its position is unknown). Identity comparison: the runner
    passes the same step dicts it iterates."""
    if not all_steps:
        return True
    first = next((i for i, s in enumerate(all_steps)
                  if s.get("action") in ("route_planes", "repair_planes")), None)
    if first is None:
        return True
    idx = next((i for i, s in enumerate(all_steps) if s is step), None)
    return idx is None or idx < first


def _drop_plane_nets(names, globs, plane_nets, notes, label):
    """Drop the plan's declared plane nets from a WILDCARD-matched selection;
    a glob naming one verbatim keeps it (explicit override).

    This is a SAFETY NET for plans that never spelled the exclusion (an LLM-
    authored plan, or the ottercast stub-clutter class). A plan converted from a
    recorded chain normally carries `!GND` itself, and since #493 that exclusion
    actually bites in both fronts -- so on a faithful replay this finds nothing
    left to drop and the GUI and CLI select the same nets.
    """
    if not plane_nets:
        return names
    from net_queries import net_pattern_matches
    # "Names it verbatim" is sheet-path aware too (#493): a plan that says 'GND'
    # is explicitly asking for the board's '/GND'. Wildcards never count as
    # explicit -- dropping wildcard-selected plane nets is the whole point.
    literal = {n for n in plane_nets
               for g in (globs or [])
               if '*' not in g and '?' not in g and net_pattern_matches(n, g)}
    excluded = sorted(n for n in names if n in plane_nets and n not in literal)
    if excluded:
        notes.append(f"{label}: plan plane nets excluded {', '.join(excluded)}")
        return [n for n in names if n not in set(excluded)]
    return names


def apply_step_selection(step, dialog, all_steps=None):
    """Apply the step's net/pair/component/assignment selection (also re-run
    right before executing the step, since consecutive steps of the same
    action share one tab's selection state). Returns notes."""
    notes = []
    action = step["action"]
    plane_nets = _plan_plane_nets(all_steps, dialog)
    # The step's RAW globs, for batch_route's net_name_patterns (#521 override
    # semantics). Cleared for EVERY step first: the names below are expanded
    # before selection, so a stale glob list from an earlier route step would
    # silently grant protection-override to nets this step never named.
    dialog._plan_net_globs = None
    if action == "route":
        globs = step.get("nets") or ["*"]
        dialog._plan_net_globs = list(globs)
        names = _match_net_names(dialog.pcb_data, globs)
        # Drop wildcard-selected plane nets only from route steps that run
        # BEFORE the first plane step (routing a whole rail as tracks there
        # fights the later pour). A route step AFTER the planes keeps them:
        # the engine skips plane-connected nets and track-patches only pads
        # the plane steps left disconnected (#479).
        if _precedes_first_plane_step(step, all_steps):
            names = _drop_plane_nets(names, globs, plane_nets, notes, "route")
        # #459: a recorded `--group BLOCK` scopes the step to one placement
        # block. Without this the block is lost and `globs` falls back to ["*"],
        # so the GUI routes the WHOLE BOARD where the CLI routed one block.
        if step.get("group"):
            names = _group_net_names(dialog.pcb_data, step, names, notes)
        if not names:
            notes.append(f"route: no nets match {globs}")
        dialog.net_panel.set_selected_nets(names)
    elif action == "route_diff":
        tab = dialog.differential_tab
        wanted = step.get("pairs") or ["*"]
        # A plan's "pairs" may be base names ("/USB/D") OR the individual P/N
        # net names the recorded route_diff --nets carried ("/USB/D+",
        # "/USB/D-"). manifest_to_plan forwards --nets verbatim, so match a pair
        # when the base name OR either half's full net name matches -- otherwise
        # a name-per-half plan selected NO pairs and the signal step later
        # routed the pair single-ended/uncoupled (set11 USB D+/D-).
        nets_by_id = {i: n.name for i, n in dialog.pcb_data.nets.items()}
        matched_display = set()
        for display_name, base_name, p_id, n_id in tab.pair_panel.all_pairs:
            cands = [base_name, nets_by_id.get(p_id, ""), nets_by_id.get(n_id, "")]
            if any(fnmatch.fnmatch(c, w) or c == w
                   for w in wanted for c in cands if c):
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
        names = _drop_plane_nets(names, globs, plane_nets, notes, "fanout")
        if not names:
            notes.append(f"fanout: no nets match {globs} on {ref}")
        tab.net_panel.set_selected_nets(names)
    elif action == "route_planes":
        tab = dialog.planes_tab
        assignments = _plane_assignments_from_step(step, dialog, notes, "route_planes")
        if not assignments:
            notes.append("route_planes: no valid assignments")
        tab.assignment_panel.set_assignments(assignments)
    elif action == "repair_planes":
        pass  # obsolete step (#562), skipped at run time -- nothing to set up
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
    from net_queries import split_net_patterns, net_pattern_matches
    known_names = {net.name for net in pcb_data.nets.values() if net.name}
    includes, excludes = split_net_patterns(globs, known_names)
    if not includes:
        includes = ["*"]
    names = []
    for net in pcb_data.nets.values():
        if not net.net_id or not net.name:
            continue
        # CLI parity: expand_net_patterns drops KiCad no-connect nets
        # ('unconnected-*') from every selection; the plan executor must not
        # hand the net panel nets the CLI would never route.
        if net.name.lower().startswith('unconnected-'):
            continue
        # Sheet-path aware, like the CLI's expand_net_patterns (#493): an
        # unqualified '!GND' must exclude the board's '/GND'.
        if any(net_pattern_matches(net.name, g) for g in includes) and \
                not any(net_pattern_matches(net.name, g) for g in excludes):
            names.append(net.name)
    return names


def _group_net_names(pcb_data, step, names, notes):
    """Narrow an already-glob-matched net list to one placement block (#459).

    Mirrors route.py's composition rule exactly: the block's nets INTERSECTED
    with whatever the step's patterns selected. The CLI's default scope is
    'touching' and its default source set is 'auto', so the same defaults apply
    here -- a plan that omitted them means the CLI used them too.

    On any failure this returns the unnarrowed list and NOTES it rather than
    raising: a plan step that silently widened to the whole board is exactly the
    divergence this function exists to prevent, so it must be visible.
    """
    block = step.get("group")
    try:
        import sys as _sys, os as _os
        _sys.path.insert(0, _os.path.dirname(_os.path.dirname(
            _os.path.abspath(__file__))))
        from group_routing import block_net_names, block_refs
        from placement.groups import parse_sources
        refs = block_refs(pcb_data, block,
                          parse_sources(step.get("group_by") or "auto"))
        scope = step.get("group_scope") or "touching"
        in_block = set(block_net_names(pcb_data, refs, scope))
    except Exception as e:
        notes.append(f"route: could not resolve --group {block!r} ({e}); "
                     f"this step is NOT scoped to the block")
        return names
    narrowed = [n for n in names if n in in_block]
    if not narrowed:
        notes.append(f"route: --group {block!r} ({scope}) selected no nets")
    return narrowed


def _component_net_names(pcb_data, ref, globs):
    """The component's nets that the step's globs select.

    #493 item 2: this used to test `any(glob matches)` across the WHOLE glob
    list, so a step's "!" exclusions were ignored outright -- '*' always matched
    and pulled every net back in. A fanout step recorded as
    `--nets '*' '!GND' '!3V3'` therefore fanned out GND/3V3 in the GUI, and only
    looked correct on boards where _drop_plane_nets happened to remove the same
    nets afterwards; any non-plane exclusion was silently a no-op. Use the shared
    include/exclude semantics (matches_net_filter) so a replayed plan selects the
    same nets the recorded CLI command did.
    """
    from net_queries import matches_net_filter, nets_for_components
    # #537: resolve the reference through the shared helper so a replayed plan
    # selects the same nets the recorded CLI command did. 'glob' keeps this
    # path's exact-reference matching (a plan names one real footprint).
    sel = nets_for_components(pcb_data, [ref], match='glob')
    return sorted(n for n in sel.net_names if matches_net_filter(n, globs))


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
      on_status(step_index, status)
          status in 'running' | 'done' | 'failed' | 'stopped'
          ('stopped' = Stop cancelled the step mid-run; it did not complete)
      on_finished(completed_count, aborted_reason_or_None)
    """

    POLL_MS = 400
    # Polls to wait for an operation to visibly start before concluding the
    # handler declined to run (e.g. a validation popup) or finished instantly.
    START_GRACE_POLLS = 5

    def __init__(self, dialog, steps, indices, on_status, on_finished,
                 log=None, on_progress=None, quiet=False):
        self.dialog = dialog
        self.steps = steps
        self.indices = list(indices)
        self.on_status = on_status
        self.on_finished = on_finished
        self.on_progress = on_progress
        self.log = log or (lambda message: None)
        # quiet: suppress each step's "Routing/Operation/Fanout Complete" OK
        # popup so the whole plan runs unattended ("Run All Selected Steps").
        # The per-step summary still prints; step status/log are unchanged.
        self.quiet = quiet
        self._queue = []
        self._completed = 0
        self._stop_requested = False
        self._step_started = None
        self._current_action = None  # action of the step running right now

    def start(self):
        # The plan sequences its own route_planes steps, so the route step's
        # "create planes first?" offer (which jumps to the Planes tab and
        # aborts routing) must not fire during an automated run.
        self.dialog._suppress_plane_offer = True
        self.dialog._suppress_completion_popups = self.quiet
        self._queue = list(self.indices)
        self._next_step()

    def stop(self):
        """Stop before the next step starts AND cancel the step running right
        now: the owning tab's _cancel_requested flag feeds the engines'
        cancel_check (plane create/repair, batch_route, route_diff, and since
        #621 both fanout engines), so the running operation aborts at its next
        safe boundary instead of being waited out (#364 follow-up).

        Every tab this executor drives now carries the flag."""
        self._stop_requested = True
        owner = self._action_owner(self._current_action) \
            if self._current_action else None
        if owner is not None and hasattr(owner, '_cancel_requested'):
            owner._cancel_requested = True

    # -- per-action wiring ---------------------------------------------------

    def _action_owner(self, action):
        """The tab (or dialog) that runs `action`'s operation."""
        d = self.dialog
        return {
            "route": d,
            "route_diff": getattr(d, "differential_tab", None),
            "fanout": getattr(d, "fanout_tab", None),
            "optimize_caps": getattr(d, "fanout_tab", None),
            "route_planes": getattr(d, "planes_tab", None),
            "repair_planes": getattr(d, "planes_tab", None),
        }.get(action)

    def _status_source(self, action):
        """The (status_text, progress_bar) pair of the tab actually doing
        the work, so the AI tab's status bar can MIRROR it live -- a
        route_diff step shows exactly what the differential tab shows."""
        owner = self._action_owner(action)
        if owner is None:
            return None, None
        return (getattr(owner, "status_text", None),
                getattr(owner, "progress_bar", None))

    def _action_parts(self, action):
        """(invoke callable, busy predicate) for an action. The handlers run
        fine without their tab being the visible page, so execution stays on
        the AI tab."""
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
            # Obsolete (#562): plane repair runs inside every route step's
            # finalize. Legacy plans' repair steps complete instantly as
            # no-ops (synchronous like optimize_caps; the start-grace period
            # in _poll_until_idle covers it).
            "repair_planes": (lambda: self.log(
                                  "repair_planes step skipped (#562: repair "
                                  "runs inside every route step's finalize)"),
                              lambda: False),
        }[action]

    # -- sequencing ----------------------------------------------------------

    def _join_worker_threads(self, timeout=60.0):
        """Wait for every tab's routing worker thread to be fully dead.

        _poll_until_idle decides a step is done by polling a CONTROL's state
        (plus the tab's _apply_pending latch); it never touches the thread. So
        in principle _finish could run while a worker was still tearing down,
        and everything after it would race that. This makes the guarantee
        explicit instead of assumed. Bounded and best-effort: a stuck worker
        must never hang the GUI, so a timeout is logged and execution
        continues.

        HONEST SCOPE: this does NOT fix the headless segfault, and in practice
        it is usually a no-op -- the thread has already exited by the time the
        control reports idle. Measured: with the join in place a 1-step replay
        still crashed on run 1. Kept because polling a button to infer that a
        thread is finished is a real (if currently latent) hazard, not because
        it fixed anything. See the crash note in git history for what IS known.
        """
        import threading
        me = threading.current_thread()
        owners = [self.dialog]
        for attr in ('differential_tab', 'planes_tab', 'fanout_tab',
                     'placement_tab'):
            owner = getattr(self.dialog, attr, None)
            if owner is not None:
                owners.append(owner)
        for owner in owners:
            t = getattr(owner, '_routing_thread', None)
            if t is None or t is me:
                continue
            try:
                if not t.is_alive():
                    continue
                t.join(timeout)
                if t.is_alive():
                    self.log(f"AI plan: worker thread on "
                             f"{type(owner).__name__} still alive after "
                             f"{timeout}s; continuing without it")
            except Exception as e:
                self.log(f"AI plan: worker-thread join skipped ({e})")

    def _finish(self, aborted_reason):
        self._current_action = None
        # Unhook the ui_thread_status push-mirror: after the plan ends,
        # tab-local status must stay tab-local.
        try:
            from .gui_utils import set_ui_status_mirror
            set_ui_status_mirror(None)
        except Exception:
            pass
        self.dialog._suppress_plane_offer = False
        self.dialog._suppress_completion_popups = False
        # Before any heavy Python work below (see _join_worker_threads).
        self._join_worker_threads()
        self._write_drc_floors()
        # Prep the GUI for the NEXT step to run, so its params are shown (and
        # editable) after this batch finishes (Andy's requested behavior: run the
        # steps before planes, and the plane step's params -- e.g.
        # max_iterations=200000, not the leaked route 1000000 -- are then in the
        # GUI). "Next" = the step after the last one just run, in plan order;
        # after the FINAL plan step it loops back to step 1. Reset first so
        # unspecified params show CLI-default-equivalent values. Best-effort;
        # skipped on abort (leave the last state for inspection).
        if aborted_reason is None and self.steps and self.indices:
            try:
                nxt = self.indices[-1] + 1
                if nxt >= len(self.steps):
                    nxt = 0  # ran the last plan step -> loop back to step 1
                if hasattr(self.dialog, 'reset_params_to_defaults'):
                    self.dialog.reset_params_to_defaults()
                apply_step_params(self.steps[nxt], self.dialog)
                apply_step_selection(self.steps[nxt], self.dialog,
                                     all_steps=self.steps)
                self.log(f"AI plan: GUI prepped for step {nxt + 1} "
                         f"({self.steps[nxt]['action']})")
            except Exception as e:
                self.log(f"AI plan: end-of-run prep skipped: {e}")
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
            _FLOOR_KEYS = ('clearance', 'track_width', 'via_size',
                           'via_drill', 'hole_to_hole_clearance',
                           'board_edge_clearance', 'diff_pair_width',
                           'diff_pair_gap')
            for step in self.steps:
                p = step.get('params') or {}
                _keys = _FLOOR_KEYS
                if step.get('action') == 'optimize_caps':
                    # #772: place_fanout_clearance's --board-edge-clearance
                    # is a PLACEMENT margin, not a routing-enforced floor.
                    # Its own writeback says exactly that and passes no
                    # edge_clearance ("must not tighten the rule",
                    # py_placer/place_fanout_clearance.py). Harvesting it
                    # here wrote a cap margin into the project as the
                    # routing copper-to-edge RULE -- the same wrong-quantity
                    # confusion #772 fixes at the control, one layer down.
                    # Converted plans now spell it
                    # cap_board_edge_clearance, which is not a floor key at
                    # all; this covers plans converted before that.
                    # `clearance` is deliberately STILL harvested from a cap
                    # step -- that IS what the CLI writes (#768/#769).
                    _keys = tuple(k for k in _FLOOR_KEYS
                                  if k != 'board_edge_clearance')
                for k in _keys:
                    if p.get(k) is not None:
                        _take(k, p[k])
            clearance = floors.get('clearance')
            track_width = floors.get('track_width')
            if clearance is None:
                return
            eff = clearance_ledger.effective(clearance)
            # kipy exposes NO setter for live design settings (unlike the SWIG
            # plugin's board.GetDesignSettings()), so the IPC plan records the
            # routed floors in the sibling .kicad_pro on disk -- exactly what the
            # CLI does (fix_project_for_output). KiCad holds the project in
            # memory, so the user must RELOAD the project for a manual DRC to
            # pick these up; the write only ever loosens floors and never
            # touches the .kicad_pcb, so it is never harmful.
            # #693: honor the shared "Fix DRC settings after routing" checkbox.
            # On the SWIG branch that gate sits on update_live_drc_floors, which
            # has no kipy equivalent -- so if the gate were not moved HERE, the
            # checkbox would be entirely inert in the IPC plan path: this
            # .kicad_pro write is the plugin's ONLY DRC-floor writer. Read the
            # live control (this path owns the real dialog); default True if it
            # is somehow absent, matching the unchecked-means-unchanged contract
            # everywhere else.
            _fixdrc693 = True
            try:
                _fixdrc693 = bool(self.dialog.fix_drc_check.GetValue())
            except Exception:
                pass
            if board_file and os.path.isfile(board_file):
                # #439: clamp non-Default classes in the written .kicad_pro only when
                # this plan routed with a --clearance ceiling (the Min-Clearance
                # override the executor checks when a step sets clearance), matching
                # the interactive route tab -- not unconditionally (the function default).
                _cc = getattr(self.dialog, 'clearance_check', None)
                _clamp = bool(_cc.GetValue()) if _cc is not None else False
                # No `minima=` here (the SWIG GUI passes
                # gui_utils.board_minima_from_live): that argument exists only to
                # keep fix_project_for_output from re-parsing the board file
                # inside a wx TIMER dispatch, where the allocation burst
                # segfaults pcbnew mid-collection. The IPC plugin holds no live
                # pcbnew board at all, so there is nothing to read the minima
                # from -- and no segfault to dodge, since the parse happens in
                # our own process. minima=None (the default) makes
                # fix_project_for_output scan the file itself, which is the same
                # five values by the same code the CLI uses.
                if _fixdrc693:
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
                        diff_pair_gap=floors.get('diff_pair_gap'),
                        clamp_nondefault_netclasses=_clamp)
                # #521: persist the plan's protection-worthy nets (matched
                # groups, routed diff pairs -- noted engine-side during the
                # steps) so later steps/chains refuse to rip them.
                try:
                    from protected_nets import (consume_protection_candidates,
                                                consume_impedance_specs,
                                                persist_protected_nets,
                                                persist_impedance_specs,
                                                pro_path_for_board)
                    _pro = pro_path_for_board(board_file)
                    persist_protected_nets(_pro, consume_protection_candidates())
                    persist_impedance_specs(_pro, consume_impedance_specs())
                except Exception as _pe:
                    self.log(f"AI plan: protected-nets record skipped: {_pe}")
                if _fixdrc693:
                    self.log(f"AI plan: recorded DRC floors in the project "
                             f"file (min clearance {eff:.4g}mm); reload the "
                             f"project in KiCad for a manual DRC to use them")
                else:
                    self.log("AI plan: DRC floors NOT written ('Fix DRC "
                             "settings after routing' is unchecked)")
        except Exception as e:
            self.log(f"AI plan: DRC floor write skipped: {e}")

    def _next_step(self):
        if self._stop_requested:
            self._finish("stopped by user")
            return
        if not self._queue:
            self._finish(None)
            return
        index = self._queue.pop(0)
        step = self.steps[index]
        self._current_action = step['action']
        self.on_status(index, "running")
        self.log(f"AI plan: step {index + 1} ({step['action']}) starting")
        try:
            # Reset every routing PARAMETER to its default BEFORE applying this
            # step's params, so a SHARED control an earlier step set doesn't leak
            # into a later step that doesn't re-specify it -- e.g. a route step's
            # max_iterations=1000000 or no_bga_zones=ALL persisting into the
            # plane step (the CLI runs each command from its own defaults). The
            # reset touches PARAMETERS only, not selections/log; apply_step_params
            # + apply_step_selection below then restore exactly THIS step's state.
            # (reset_params_to_defaults' own docstring says it is called here.)
            # EXCEPTION: optimize_caps is the tail of the BGA fanout, not a
            # standalone op -- it clears decoupling caps from THE FANOUT'S vias,
            # at the fanout's clearance/via-size (the CLI runs it as
            # place_fanout_clearance right after bga_fanout with the same
            # --clearance). It carries no params of its own, so it must INHERIT
            # the preceding fanout step's options; resetting to defaults first
            # runs it at the wrong clearance and it stops moving the caps. So
            # skip the per-step reset for optimize_caps and let it keep the
            # fanout step's live control state.
            # #768: inheriting the VALUE is right, inheriting the SEMANTIC
            # BIT is not. Since #768 the PRESENCE of --clearance decides whether
            # the cap step caps the net classes and clamps the project, so a
            # step that carries no `clearance` param must run the OMITTED
            # branch -- and with the Min-Clearance override left ticked by the
            # preceding fanout step, it would run the GIVEN one. Unticking it
            # also lands the right flat value: `_effective_clearance()` then
            # returns the board's own Default class, which is exactly what
            # `resolve_pair_clearance(pcb_file, None)` gives the CLI.
            #
            # A step that DOES carry `clearance` is unaffected: apply_step_params
            # ticks the override for it two lines below. That case exists now
            # because manifest_to_plan carries the flag into the step (#768);
            # before it did not, which is why this exception was blanket.
            #
            # #780: and a FANOUT step that switches the inline cap pass on
            # runs the same pass, so it needs the same rule. Until #780 it
            # did not matter -- the inline path dropped the ceiling on the
            # floor, so the semantic bit could not reach the engine that
            # way. Now it can, and `reset_params_to_defaults` does NOT
            # reset `clearance_check` (it resets edge_ and zone_), so a
            # fanout step following one that set `clearance` inherits a
            # ticked override. `manifest_to_plan` never emits that shape --
            # it converts the cap step separately -- but a Claude-authored
            # plan may, and this is the executor's rule, not the
            # converter's.
            _p = step.get("params") or {}
            _runs_caps = (step["action"] == "optimize_caps"
                          or (step["action"] == "fanout"
                              and _p.get("optimize_caps")))
            if _runs_caps and not _p.get("clearance"):
                _cc = getattr(self.dialog, 'clearance_check', None)
                if _cc is not None and _cc.GetValue():
                    _cc.SetValue(False)
                    self.log("AI plan: %s has no --clearance; "
                             "cleared the Min-Clearance override so the cap "
                             "pass runs at the board's own class, as the CLI "
                             "does" % step["action"])
            # #772: the per-step reset below is skipped for optimize_caps, so a
            # cap knob an earlier cap step set carries into the next one --
            # there is no fanout step in between to reset it. A BLANKET reset
            # here would undo the inheritance that exception exists for, so it
            # is scoped two ways: only the CAP PANEL's controls, and only when
            # the step actually NAMES one of them.
            #
            # The rule is CLI parity, and it is exact rather than approximate:
            # the panel's creation defaults ARE place_fanout_clearance.py's
            # argparse defaults, value for value (2.0 / 1.0 / 0.2 / 2.0 / 3.0 /
            # 1.5 / 30 / 'C,R,FB' / 0.3 / rotate on / edge unset -- checked
            # against repair_fanout_clearance's signature, 11 of 11). So "reset
            # the cap panel, then apply this step's params" IS "run the CLI with
            # exactly the flags this step carries", which is what a replayed
            # manifest is supposed to mean. A recorded `--near-margin 1.5` gives
            # the other ten flags their argparse defaults; inheriting ten
            # leftovers instead is not that run.
            #
            # A step with NO params is the auto-inserted one
            # (_insert_cap_optimization), which is left alone. WHAT IT THEN
            # INHERITS IS NARROWER THAN THIS COMMENT USED TO CLAIM, and the
            # same review measured that too: since the cap knobs joined
            # reset_params_to_defaults, a PRECEDING fanout step's own
            # per-step reset already returns them to the CLI defaults. So an
            # operator's interactive cap tweak survives into a bare cap step
            # only when nothing precedes it -- which in a real plan is
            # rarely the case. That is the right answer for a REPLAY (the
            # recorded run had no operator) and a real change to the
            # interactive path, so it is disclosed rather than implied.
            #
            # SHARED knobs (clearance / grid_step / via_size) are NOT touched:
            # they come from the Basic tab, and the #768 inheritance rationale
            # above is about those.
            # THE DISCRIMINATOR IS "DID THE PLAN SPECIFY THIS STEP", not
            # "did it name a cap knob". An adversarial review measured the
            # difference and it is a real leak, not a nicety: a manifest
            # step converted from `place_fanout_clearance.py --clearance
            # 0.1 --grid-step 0.05` carries params but names no CAP knob,
            # so the name-based test skipped the reset and step B ran at
            # step A's near_margin / cap_prefix / max_passes instead of the
            # CLI defaults. The --grid-step row this branch adds makes that
            # shape MORE reachable, not less.
            #
            # `params` is the exact signal, and it is exact because of the
            # commit two along: _insert_cap_optimization emits
            # {"action": "optimize_caps"} with NO params key at all, so
            # "has params" distinguishes a plan-authored step from the
            # auto-inserted one with no proxy in between.
            if step["action"] == "optimize_caps":
                _given = sorted(step.get("params") or {})
                if _given and hasattr(self.dialog,
                                      'reset_cap_params_to_defaults'):
                    try:
                        self.dialog.reset_cap_params_to_defaults()
                        self.log("AI plan: optimize_caps specifies "
                                 + ", ".join(_given)
                                 + " -- cap knobs reset to the CLI defaults "
                                   "first, so the ones it omits are defaults "
                                   "rather than the previous step's values")
                    except Exception as _e:
                        self.log(f"AI plan: cap-panel reset skipped: {_e}")
            if (step["action"] != "optimize_caps"
                    and hasattr(self.dialog, 'reset_params_to_defaults')):
                try:
                    self.dialog.reset_params_to_defaults()
                except Exception as _e:
                    self.log(f"AI plan: per-step reset skipped: {_e}")
            # Re-apply BOTH this step's parameters and its selection right before
            # running it: consecutive steps of the same action share one tab's
            # controls, so plan-time fill leaves only the last such step's
            # track_width/clearance/via/diff-pair geometry in place. Without this
            # re-apply, e.g. a fine-pitch route step and a general route step
            # would both run at whichever was applied last.
            notes = apply_step_params(step, self.dialog)
            notes += apply_step_selection(step, self.dialog, all_steps=self.steps)
            for note in notes:
                self.log(f"AI plan: {note}")
            invoke, busy = self._action_parts(step["action"])
            import time as _time
            self._step_started = _time.time()
            # Push-mirror for UI-thread steps (fanout, cap optimize, the
            # apply phases): this tab's poll (_poll_until_idle, wx.CallLater)
            # cannot fire while a step BLOCKS the main loop, so without this
            # the AI tab froze exactly when the working tab was busiest.
            # ui_thread_status forwards every forced-repaint message here.
            from .gui_utils import set_ui_status_mirror

            def _mirror(msg, _idx=index, _step=step):
                if self.on_progress is not None:
                    _el = _time.time() - (self._step_started or _time.time())
                    self.on_progress(_idx, _step, msg, 0, 0, _el, True,
                                     force_repaint=True)
            set_ui_status_mirror(_mirror)
            invoke()
        except Exception as e:
            self.on_status(index, "failed")
            self.log(f"AI plan: step {index + 1} failed: {e}")
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
            # the AI tab, with per-step elapsed time.
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
        if self._stop_requested:
            _owner = self._action_owner(self.steps[index]["action"])
            if _owner is not None and hasattr(_owner, '_cancel_requested'):
                # Stop was pressed while this step ran and its tab is
                # cancellable: the engine aborted at its next safe boundary
                # and the tab discarded the partial results, so the step did
                # NOT complete -- mark it stopped (and leave it checked for a
                # re-run), never "[ok]". A non-cancellable tab (fanout) ran
                # its step to completion, so it falls through to "done".
                self.on_status(index, "stopped")
                self.log(f"AI plan: step {index + 1} stopped (cancelled)")
                self._next_step()
                return
        self._completed += 1
        self.on_status(index, "done")
        self.log(f"AI plan: step {index + 1} finished")
        self._next_step()
