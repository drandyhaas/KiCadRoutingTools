#!/usr/bin/env python3
"""Convert a stress-test redo_commands.sh manifest into a GUI plan JSON.

The AI tab's Load button accepts the output, so a recorded stress chain
can be replayed through the GUI plan executor without any LLM run:

    python3 tests/stress/manifest_to_plan.py runs_set1/<board>/redo_commands.sh plan.json
    python3 tests/stress/manifest_to_plan.py runs_set1/<board>/redo_commands.sh -o plan.json

Maps each routing command to a plan step (action + nets/pairs/assignments +
params). Since the GUI accepts ANY snake_case option name in params, every
recognized --flag value is carried over (--max-iterations, --max-ripup,
--grid-step, ...). Check/grade commands and unknown tools are skipped with a
note; every board-WRITING tool the registry declares is instead mapped,
refused by name (REFUSED_TOOLS, loud on stderr) or a named no-op (NOOP_TOOLS). Only the file-dependency chain to the FINAL board is emitted (retries
that were superseded are dropped), mirroring redo_stress_test's pruning.
"""
import json
import os
import re
import shlex
import sys

TOOL_ACTIONS = {
    'route.py': 'route',
    'route_diff.py': 'route_diff',
    'route_planes.py': 'route_planes',
    # BOTH names: recorded manifests carry the historical spelling forever;
    # the module (CLI remains a standalone utility) is repair_planes.py now.
    'route_disconnected_planes.py': 'repair_planes',
    'repair_planes.py': 'repair_planes',
    'bga_fanout.py': 'fanout',
    'qfn_fanout.py': 'fanout',
}

# Board-mutating tools with NO plan representation (#431). They are RECORDED in
# the manifest (#132) so the file-dependency chain stays intact -- a
# refused-but-PRESENT command keeps compute_prune_keep linking
# board -> board_placed, whereas a missing one leaves the next step's input
# produced by nothing and the pruner drops legitimate upstream steps.
#
# Refused rather than mapped, because there is nothing to map TO: the plan
# format has no placement step. (The GUI's Placement sub-tab drives the
# placement SKILLS headless - it is not a plan action, so a plan step's
# max_displacement/crossing_penalty/halo_* would still resolve to nonexistent
# dialog attributes and silently run at hardcoded defaults while the plan JSON
# claims otherwise.) And refused rather than DROPPED, because the unknown-tool path
# only bumps a `skipped` counter -- a number, not a name -- so the converted
# plan looks complete when it is not.
REFUSED_TOOLS = {
    'place_simple.py': (
        'places the board by its groups and one annealed objective; the plan '
        'format has no placement step. Run it BEFORE the plan and start the '
        'plan from the placed board'),
    'judge_seeds.py': (
        "fast-routes and scores place_simple's judged seeds (the placement "
        "skill's judging step); the plan format has no placement step. Run "
        'it BEFORE the plan and start the plan from the kept seed\'s board'),
    'place_movie.py': (
        'films place_simple placing the board (and, with --route, the '
        "placement skill's flow, routing included); the plan format has no "
        'placement step. Run it BEFORE the plan and start the plan from the '
        'placed board'),
    # movie3d/: the 3D placement movie, place_simple placing the board on camera.
    'make_movie.py': (
        'films place_simple placing the board in 3D (and, with --route, the '
        "placement skill's flow, routing included); the plan format has no "
        'placement step. Run it BEFORE the plan and start the plan from the '
        'placed board'),
    'record.py': (
        "records place_simple's run for the 3D placement movie and writes its "
        'placed board; the plan format has no placement step. Run it BEFORE '
        'the plan and start the plan from the placed board'),
    # place_optimize, place_route_loop, place_seed, place_groups,
    # place_candidates, place_reconstruct, place_portfolio, rank_rotations and
    # place_intent_sides are retired, but recorded manifests still name them,
    # and a step this table does not know is dropped silently, not refused.
    'place_optimize.py': (
        'moves footprints; the plan format has no placement step. Run it BEFORE '
        'the plan and start the plan from the placed board'),
    'place_route_loop.py': (
        'routes and moves footprints in a loop; the plan format has no placement '
        'step. Run it on the CLI'),
    'place_seed.py': (
        'seeds/repairs a placement from an intent; the plan format has no '
        'placement step. Run it BEFORE the plan and start the plan from the '
        'seeded board'),
    'place_groups.py': (
        'seeds a placement one functional group at a time into the regions '
        'of an intent (place_seed per group); the plan format has no placement '
        'step. Run it BEFORE the plan and start the plan from its output'),
    'place_candidates.py': (
        'places a grouped board several ways at once (place_seed and '
        'place_groups over seeds) and keeps the best; the plan format has no '
        'placement step, and keeping a candidate is a decision. Run it BEFORE '
        'the plan and start the plan from its output'),
    'place_reconstruct.py': (
        'reconstructs a damaged placement; the plan format has no placement '
        'step. Run it BEFORE the plan and start the plan from its output'),
    'place_portfolio.py': (
        'generates a SLATE of placements to choose between; the plan format has '
        'no placement step, and picking one is a decision, not a replayable '
        'step. Run it on the CLI and start the plan from the adopted board'),
    'rank_rotations.py': (
        'seeds the board once per candidate angle of one part and writes the '
        'ranking (#1113); the plan format has no placement step, and choosing '
        'an angle is a decision. Run it on the CLI and start the plan from '
        'the board seeded at the chosen angle'),
    'place_pose.py': (
        'applies a pose the MODEL chose (#892), stamps or strips a KiCad lock, '
        'or copies one sheet instance\'s arrangement onto its siblings '
        '(`replicate`, #1211); the plan format has no placement step, and a '
        'pose is a decision rather than a replayable parameter set. Run it on '
        'the CLI and start the plan from its output'),
    'place_intent_sides.py': (
        'mirrors parts onto the faces a floorplan intent declares (#1280); '
        'the plan format has no placement step. Run it BEFORE the plan and '
        'start the plan from its output'),
    'render_placement.py': (
        'renders a PNG; it changes no board and has nothing to replay'),
    'beautify_labels.py': (
        'tidies reference-designator silkscreen; no plan step yet -- run it '
        'before/after the plan'),
    'add_rule_area.py': (
        'writes a copper keep-out rule area that every later route step obeys '
        '(#1200); the plan format has no step for it. Run it on the CLI and '
        'start the plan from its output'),
    'autoplan.py': (
        'builds its OWN chain of plane, fanout, decap, diff-pair and route '
        'steps from the board and runs it; the plan format has no step that '
        're-derives a chain. Run it on the CLI and start the plan from its '
        'output (the steps it ran are in <output>.autoplan/plan.json)'),
    # Every board-writing KRT_TOOL (krt_registry `kind` actor or conditional)
    # is mapped, refused here, or in NOOP_TOOLS below: the parity gate reads
    # the registry and fails on one that is none of the three (#937).
    'make_unplaced.py': (
        'piles every pad-bearing part off the outline to build a from-scratch '
        'placement input; the plan format has no placement step. Run it on '
        'the CLI and start the plan from its output'),
    'compare_seeds.py': (
        'seeds the board once per seed and ranks them; the plan format has no '
        'placement step, and adopting a seed is a decision. Run it on the CLI '
        'and start the plan from the adopted board'),
    'converge.py': (
        'the placement convergence driver (ledger, ranked poses, stepping '
        'back); its moves are decisions and the plan format has no placement '
        'step. Run it on the CLI and start the plan from its output'),
    'fix_kicad_drc_settings.py': (
        'rewrites the project\'s DRC floors and severities with values of its '
        'own; a GUI plan writes only the floors its steps routed to (per step '
        'and at the end), so the explicit ones would be lost. Run it on the '
        'CLI over the board the plan leaves'),
    'pack_river.py': (
        'its standalone CLI (off-grid bus-lane compaction after routing, #658) '
        'was removed; there is nothing to run. In-run packing is '
        'KICAD_PACK_INLINE=1 on the route step'),
    'run_plan.py': (
        'runs a whole GUI plan on the CLI (#507); a plan step cannot hold a '
        'plan. Load that plan in the AI tab instead'),
    # The awx bus router and its benches. route_bus is the one that sits in a
    # board's chain (the bus step); the rest build or replay research benches.
    'route_bus.py': (
        'routes an array pair\'s bus with the awx bus router (the bus step); '
        'the plan format has no bus step. Run it on the CLI and start the plan '
        'from its output'),
    'route_lanes.py': (
        'routes chosen lanes of an awx bus plan one at a time; the plan format '
        'has no bus step. Run it on the CLI'),
    'joint_escape.py': (
        'plans and lays one awx escape for every signal ball of an array; the '
        'plan\'s fanout step is bga_fanout / qfn_fanout, not this. Run it on '
        'the CLI and start the plan from its output'),
    'dogbone_arrays.py': (
        'dog-bones every netted ball of an awx bus\'s two arrays, the base of '
        'route_bus --escape; the plan\'s fanout step is bga_fanout / '
        'qfn_fanout, not this. Run it on the CLI and start the plan from its '
        'output'),
    'escape_plan.py': (
        'plans and lays an awx escape fanout on a dog-boned array pair, a '
        'stage of route_bus --escape; the plan format has no bus step. Run it '
        'on the CLI and start the plan from its output'),
    'braid.py': (
        'emits the awx braid for a fanned-out bus; the plan format has no bus '
        'step. Run it on the CLI and start the plan from its output'),
    'pack_board.py': (
        'packs a braided awx board again; the plan format has no bus step. Run '
        'it on the CLI and start the plan from its output'),
    'baseline_freerouting.py': (
        'routes an awx rung with Freerouting, another router; no plan step '
        'runs it. Run it on the CLI'),
    'make_bench.py': (
        'builds an awx bench article from a board, not a step of the board\'s '
        'own chain. Run it on the CLI'),
    'human_ends_bench.py': (
        'builds an awx bench whose ends are the human\'s, not a step of the '
        'board\'s own chain. Run it on the CLI'),
    'baseline_bench.py': (
        'routes an awx bench with two other routers and the designer\'s, for '
        'comparison, not a step of the board\'s own chain. Run it on the CLI'),
    'synth_bus.py': (
        'generates awx bus articles with known answers (#622), not a step of '
        'a board\'s chain. Run it on the CLI'),
    'synth_handoff.py': (
        'generates awx trunk-to-ring handoff cases (#622), not a step of a '
        'board\'s chain. Run it on the CLI'),
    'synth_layers.py': (
        'generates awx many-layer whole-route cases (#622), not a step of a '
        'board\'s chain. Run it on the CLI'),
    'whole_route.py': (
        'runs the awx whole route on a bench rung end to end; the plan format '
        'has no bus step. Run it on the CLI'),
    'whole_ladder.py': (
        'runs the awx whole route over a ladder of benches; a research driver, '
        'not a board-chain step. Run it on the CLI'),
    'resolve_round.py': (
        'replays one awx round\'s first solve on that round\'s own files; a '
        'research driver, not a board-chain step. Run it on the CLI'),
    'whole_compare.py': (
        'renders an awx rung\'s routed board beside the human\'s; a research '
        'tool, not a board-chain step. Run it on the CLI'),
    'fanout_logdiff.py': (
        'compares two awx fanout logs; a research tool, not a board-chain '
        'step. Run it on the CLI'),
}

# Board-writing tools whose recorded command has NOTHING to replay on a live
# board -- distinct from a refusal, whose absence makes the replay diverge.
# They stay in the manifest, so compute_prune_keep's file chain still links
# their input to their output, and convert to no plan step BY NAME rather than
# through the unknown-tool path's bare `skipped` count.
NOOP_TOOLS = {
    'copy_board.py': (
        'copies a board with its sibling project files (#441); a plan runs on '
        'the live board, which the next step reads as it is, with the open '
        'project, so there is nothing to copy'),
    'fill_for_delivery.py': (
        'writes a FILLED copy of the board for delivery (#910); KiCad fills '
        'the live board itself (every GUI step refills its zones), as for '
        'route.py\'s --write-fill'),
    'plane_score.py': (
        'scores plane fragility on a scratch copy; nothing the chain reads is '
        'written'),
}

# The decap-placement step: no TOOL_ACTIONS entry because it converts through
# cap_optimization_step into a standalone `optimize_caps` step (see there).
CAP_TOOL = 'place_fanout_clearance.py'

# CLI flag -> plan params key (numbers parsed; lists collected).
FLAG_PARAMS = {
    '--track-width': 'track_width',
    '--clearance': 'clearance',
    '--via-size': 'via_size',
    '--via-drill': 'via_drill',
    '--grid-step': 'grid_step',
    '--max-iterations': 'max_iterations',
    '--max-ripup': 'max_ripup',
    '--ripup-abandon-metric': 'ripup_abandon_metric',
    '--ripup-blocker-select': 'ripup_blocker_select',
    # route.py spells the strategy flag --ordering; the GUI control (and the
    # plan executor's param name) is ordering_strategy. Without this mapping
    # the generic fallthrough carried it as 'ordering', which matches no
    # dialog control, so a replayed plan silently routed in default order.
    '--ordering': 'ordering_strategy',
    # #1146: the keep-away free radius and cost (Advanced-tab spin controls of
    # the same names).
    '--keep-away-free': 'keep_away_free',
    '--keep-away-cost': 'keep_away_cost',
    # route_planes' --zone-clearance is type=float (a value, NOT a toggle); it
    # must consume its argument here. It briefly lived in BOOL_FLAGS, which
    # dropped the value and set zone_clearance=True -> the plan executor's
    # generic loop stamped float(True)=1.0 onto the plane zone-clearance
    # SpinCtrlDouble (range max 2.0, so unclamped), replaying a board routed at
    # e.g. 0.12 mm pour clearance with a 1.0 mm clearance instead.
    '--zone-clearance': 'zone_clearance',
    '--hole-to-hole-clearance': 'hole_to_hole_clearance',
    '--board-edge-clearance': 'board_edge_clearance',
    '--via-cost': 'via_cost',
    '--heuristic-weight': 'heuristic_weight',
    '--turn-cost': 'turn_cost',
    '--diff-pair-gap': 'diff_pair_gap',
    '--impedance': 'impedance',
    '--coplanar-gap': 'coplanar_gap',
    '--gnd-via-distance': 'gnd_via_distance',
    # #485: route_planes area via stitching -- value flags.
    '--stitch-pitch': 'stitch_pitch',
    '--stitch-fence-pitch': 'stitch_fence_pitch',
    '--stitch-inset': 'stitch_inset',
    '--stitch-max-freq': 'stitch_max_freq',
    # route_planes' zone width and shared-layer (Voronoi split) knobs: the
    # planes tab's controls of the create_plane param names.
    '--min-thickness': 'min_thickness',
    '--plane-max-iterations': 'plane_max_iterations',
    '--voronoi-seed-interval': 'voronoi_seed_interval',
    '--exit-margin': 'exit_margin',
    '--extension': 'extension',
    # #860 follow-up: bga_fanout/qfn_fanout's escape selector. It was in
    # NO table here and survived only on the unknown-flag fallthrough,
    # which happens to spell it 'escape_method' -- the name ai_plan's
    # _PARAM_SPECIAL wants, so it worked BY LUCK and unasserted, exactly
    # like --fab-tier above. Registered so the parity gate binds it.
    '--escape-method': 'escape_method',
    # bga_fanout's primary escape direction ('horizontal' / 'vertical'): the
    # BGA panel's escape_direction RadioBox, which the plan executor sets by
    # item label. The fallthrough's `primary_escape` matched no control.
    '--primary-escape': 'escape_direction',
    # #581: same-net pad via clearance -- valid on planes, route, route_diff,
    # bga/qfn fanout and repair steps; the GUI control lives on the Basic tab.
    '--same-net-pad-clearance': 'same_net_pad_clearance',
    '--max-track-width': 'max_track_width',
    '--min-track-width': 'min_track_width',
    '--analysis-grid-step': 'analysis_grid_step',
    # #237's shared fab-capability flags (fab_tiers.add_fab_tier_args, on ten
    # CLIs) were in NONE of this module's tables. The unknown-flag fallthrough
    # still carried both into the plan, with opposite outcomes: `fab_tier`
    # happened to MATCH its dialog control's name, so it worked -- by luck,
    # unasserted -- while `fab_overrides` matched no control and no alias, so
    # ai_plan silently ignored it and a replayed plan routed with the bare
    # tier (and with escalation re-enabled, which supplying the file turns
    # off). Same class as `--ordering` above: the fallthrough name is not the
    # control name. Both are string-valued (`_num` falls through to the raw
    # string); both controls are already in reset_params_to_defaults.
    '--fab-tier': 'fab_tier',
    '--fab-overrides': 'fab_overrides_path',
    # #857: the escalation policy (the GUI Choice of the same name).
    '--escalation': 'escalation',
    # #530: the explicit class ceiling (Min Clearance + the ceiling box).
    '--clearance-ceiling': 'clearance_ceiling',
    # route.py's guide-corridor (#7) and keepout (#27) layers and spacing. The
    # fallthrough spelled them after the flag, which matches no control, so a
    # replay followed the default User.1 / User.2 layers whatever the run had
    # drawn on. Their controls are the text fields beside the enabling
    # checkboxes (both reset per step, so a value cannot outlive its step).
    '--guide-corridor-layer': 'guide_corridor_layer_ctrl',
    '--guide-corridor-spacing': 'guide_corridor_spacing_ctrl',
    '--keepout-layer': 'keepout_layer_ctrl',
    # route.py / route_diff.py's explicit per-net clearance FILE: the
    # Advanced tab's Net Clearances File field (a relative path is resolved
    # against the recorded cwd, see PATH_PARAMS).
    '--net-clearances': 'net_clearances_path',
    # route_diff.py's meander chamfer multiplier: the diff tab's control.
    '--diff-chamfer-extra': 'chamfer_extra',
    # route_diff.py's centerline setback: the diff tab's control of that name
    # (0 = auto there, which the tab hands the engine as None -- the CLI's
    # omitted-flag default). The fallthrough spelled it after the flag, which
    # no control matched, so a replay ran at the auto setback.
    '--diff-pair-centerline-setback': 'centerline_setback',
    # (#856's --relax-drc-severities is a store_true switch and lives in
    # BOOL_FLAGS. It used to sit here, where the value branch consumed the
    # NEXT token as its value: `--relax-drc-severities --clearance 0.1`
    # converted to relax_drc_severities='--clearance', lost the clearance,
    # and with no --nets turned `0.1` into the step's net list.)
}
LIST_FLAGS = {
    '--layers': 'layers',
    '--power-nets': 'power_nets',
    '--power-nets-widths': 'power_nets_widths',
    '--layer-costs': 'layer_costs',
    # #381 D3: route_diff's polarity-swap allowlist (nargs='+' globs). Carried
    # explicitly (not via the generic unknown-flag fallthrough) so a scoped
    # allowlist survives as a list param that ai_plan's alias routes to the
    # diff tab's polarity_swap_nets_text field.
    '--polarity-swap-nets': 'polarity_swap_nets',
    # #486: route.py's coplanar-waveguide net allowlist (nargs='+' globs).
    # LIST, not FLAG_PARAMS -- as a scalar flag only the FIRST pattern survived.
    '--coplanar-nets': 'coplanar_nets',
    # #284: the rip-existing allowlist (nargs='+' globs), routed to a TextCtrl
    # via ai_plan's alias table. #521's `--protect-nets` was REMOVED from every
    # tool in 53a5a16e and is deliberately absent here: it had no LIST_FLAGS
    # entry, so the conversion loop would have raised KeyError on any manifest
    # carrying it. Protection is now recorded in the .kicad_pro by the step that
    # routes a matched group or diff pair, not passed as a flag.
    '--rip-existing-nets': 'rip_existing_nets',
    # bga_fanout's future-pour declaration (NET:LAYER[,LAYER...] specs,
    # nargs='+'). Review parity finding 5: a recorded manifest carrying the
    # flag used to convert to a plan that silently dropped it. The GUI/plan
    # side accepts the same raw spec strings (fanout_gui parses them like
    # the CLI main does).
    '--plane-net-layers': 'plane_net_layers',
    # #1146: route.py's keep-away rules (AGG:VICTIM:GAP strings). The plan
    # executor space-joins the list into the Advanced tab's one-line
    # `keep_away` text control, whose parser splits on whitespace again.
    '--keep-away': 'keep_away',
    '--nets': None,  # handled per action
    '--pairs': None,
    '--plane-layers': None,
}
# LIST_FLAGS whose argparse action is 'extend': every occurrence ADDS to the
# list, where a plain nargs='+' flag repeated keeps only its last value.
EXTEND_LIST_FLAGS = {'--keep-away'}
BOOL_FLAGS = {
    '--rip-blocker-nets': 'rip_blocker_nets',
    '--smoothing': 'smoothing',      # #536 octolinear smoothing (default ON)
    # (--no-smoothing is in INVERTED_BOOL_FLAGS below: it used to be here as
    # `no_smoothing`, a name no control or alias matched.)
    '--add-gnd-vias': 'add_gnd_vias',
    # #485: route_planes area via stitching toggles (planes-tab checkboxes
    # stitch_vias / stitch_edge_fence, applied by the plan executor's
    # generic loop).
    '--stitch-vias': 'stitch_vias',
    '--stitch-edge-fence': 'stitch_edge_fence',
    # route_planes: keep a net's existing pour on the layer (the planes tab's
    # "Skip existing zones" box; reset unticked, so a step WITHOUT the flag
    # replaces the pour, as the CLI does).
    '--skip-existing-zones': 'skip_existing_zones',
    '--no-gnd-vias': 'no_gnd_vias',
    # route.py spells it --no-bga-zones (plural, nargs='*'); bga_fanout uses the
    # singular. Both map to the GUI's no_bga_zone special (bare = exclude ALL).
    '--no-bga-zone': 'no_bga_zone',
    '--no-bga-zones': 'no_bga_zone',
    # #489 section 9: now on every step that writes pad/via copper (route,
    # route_diff, route_planes, route_disconnected_planes, bga/qfn fanout).
    '--add-teardrops': 'add_teardrops',
    # #910: route.py's opt-in delivery fill. Registered not because the GUI
    # has a control for it (it does not -- the GUI mutates a live board KiCad
    # fills itself), but because an UNREGISTERED bare flag falls through to
    # the unknown-flag branch, whose value loop does not stop at a
    # `.kicad_pcb` -- so `route.py in.kicad_pcb --write-fill out.kicad_pcb`
    # would swallow the OUTPUT path out of the step's `_files`, which pruning
    # reads. Mapping it to a param keeps the file list intact.
    '--write-fill': 'write_fill',
    # #487: route_planes' default-on thermal-via arrays; the NEGATIVE flag
    # must survive conversion or a replay re-enables what the run disabled
    # (ai_plan's no_thermal_vias alias unchecks the planes checkbox).
    '--no-thermal-vias': 'no_thermal_vias',
    # The POSITIVE spellings must survive too: ai_plan already consumes
    # both params (thermal_relief checkbox; thermal_vias with the
    # default-ON absent-means-default rule), but a converted manifest that
    # DROPPED the flag would replay a --thermal-relief run without relief
    # spokes (found by the ef4c19a..4db8c18 parity audit).
    '--thermal-relief': 'thermal_relief',
    '--thermal-vias': 'thermal_vias',
    # #515 / PR #533: rip+re-route the selected nets from scratch. Same-named
    # basic-tab checkbox; applied by the plan executor's generic loop.
    '--force-reroute': 'force_reroute',
    # #860 follow-up: qfn_fanout's under-pad via-in-pad opt-in. #846 made
    # this flag consequential (a via that OVERLAPS its pad is now
    # classified and clamped as one), and it reached the GUI through
    # nothing: unregistered here, so a recorded manifest replayed the
    # step WITHOUT it. The param name matches the QFNOptionsPanel
    # checkbox, so the executor's generic loop places it once it arrives.
    '--allow-via-in-pad': 'allow_via_in_pad',
    # route.py's bus mode. Its GUI home is the Advanced-options checkbox
    # `bus_enabled`, but the unknown-flag fallthrough spelled it `bus`, which
    # matches no control and no alias: ai_plan logged "no control for bus,
    # ignored" and a replayed step routed with bus mode OFF, while the
    # --bus-detection-radius / --bus-min-nets / --ordering bus it travels
    # with all landed. Same class as --fab-overrides above: the fallthrough
    # name is not the control name, so emit the control's own name.
    '--bus': 'bus_enabled',
    # Same shape, found by the route.py flag enumeration in
    # tests/gui_parity/test_manifest_plan_parity.py: each flag's GUI control
    # exists, is reset per step, and has a different name.
    '--can-swap-to-top-layer': 'can_swap_to_top',
    '--skip-routing': 'skip_routing_check',
    # #856: opt-in severity relaxation (ai_plan aliases it onto its checkbox).
    # A switch, so it must not consume the next token -- see FLAG_PARAMS.
    '--relax-drc-severities': 'relax_drc_severities',
    # Found by the flag enumeration over route_diff.py and bga_fanout.py:
    # each has a control, reset per step, under another name.
    '--diff-pair-intra-match': 'intra_match_check',     # route_diff
    '--ac-couple-match': 'ac_couple_check',              # route_diff
    '--check-for-previous': 'check_previous',            # bga_fanout
    '--no-inner-top-layer': 'no_inner_top',              # bga_fanout
    '--force-escape-direction': 'force_escape',          # bga_fanout
}
# route.py `--no-X` flags whose GUI home is a POSITIVE checkbox (default on).
# The converter emits the control's own name with False, the CAP_BOOL_FLAGS
# `--no-rotate` shape, so the plan executor's generic loop unticks it. An alias
# could not do this: an alias renames, it does not invert. `--no-smoothing`
# used to convert to `no_smoothing: True`, which no control matched, so a
# replay routed WITH the #536 smoothing the flag exists to turn off.
INVERTED_BOOL_FLAGS = {
    '--no-smoothing': 'smoothing',
    '--no-stub-layer-swap': 'enable_layer_switch',
    '--no-power-tap-neckdown': 'power_tap_neckdown_check',
    # The shared "Fix DRC settings after routing" box (#160/#693): a step
    # carrying the flag skips the DRC-floor writeback, a step without it
    # writes back -- the CLI's per-step semantics, since reset_params_to_
    # defaults re-ticks the box before every step.
    '--no-fix-drc-settings': 'fix_drc_check',
}
# Repeatable nargs='+' flags (argparse action='append'): each OCCURRENCE is one
# group, so the plan param is a list of lists. route.py's --length-match-group
# fell through to `length_match_group`, which ai_plan ignored -- a replay lost
# every recorded length-match group -- and a repeated flag kept only its LAST
# occurrence. `length_match_groups` is the name ai_plan's special handler
# formats into the GUI's comma-separated group field.
GROUP_LIST_FLAGS = {
    '--length-match-group': 'length_match_groups',
}

# Flags whose values are file paths / bookkeeping -- consumed, never params.
# --output still feeds the chain-pruning file list. (--net-clearances left
# this set: it names a per-net clearance FILE that replaces the net classes'
# map, so dropping it replayed the step at different clearances. It is
# FLAG_PARAMS' net_clearances_path now, made absolute like a cap --intent.)
#
# --deadline is a HARNESS budget, not a routing parameter: it changes nothing
# about the copper, only how long the process is allowed to spend making it, and
# the recorded number belongs to whatever external timeout the recording harness
# had. The GUI has no equivalent and needs none -- it has a live Cancel button
# driving the same `cancel_check`. Without this entry it falls through to the
# unknown-flag branch and lands in the plan as a `deadline` param that no dialog
# control matches, which the executor then drops silently; consuming it here
# says so on purpose instead.
IGNORE_FLAGS = {'--output', '--summary-json', '--schematic-dir', '--report',
                '--deadline',
                # #856: deprecated no-op (routing steps no longer touch DRC
                # severities; the control it drove is gone).
                '--keep-thermal',
                # #857: a HARNESS exit-code flag (non-zero when anything was
                # delivered below its requested size); changes no copper, and
                # the GUI's results panel is its equivalent.
                '--strict-sizes'}

# Per-tool flag renames: bga_fanout calls the trace width --width (routed to the
# Basic-tab track_width, which BGA fanout reads). qfn_fanout also uses --width
# but its GUI home is the QFN panel's own control (see TOOL_FLAG_PARAMS, #381 D7).
TOOL_FLAG_ALIASES = {
    'bga_fanout.py': {'--width': '--track-width'},
    # Both fanout CLIs now accept BOTH spellings (they used to disagree, which
    # cost a recorded run and a replay a wasted step each). Normalize qfn's
    # --track-width to --width HERE, before TOOL_FLAG_PARAMS is consulted --
    # otherwise it falls through to the global FLAG_PARAMS['--track-width'] and
    # lands on the Basic-tab track_width instead of the QFN panel's own
    # control, which is precisely the #381 D7 bug the override below fixes.
    'qfn_fanout.py': {'--track-width': '--width'},
}

# Per-tool flag -> plan-param overrides (win over the global FLAG_PARAMS).
# #381 D4: route_diff.py's trace width is --track-width, but its GUI home is the
# diff tab's diff_pair_width control (NOT the Basic-tab track_width the global
# FLAG_PARAMS would target). Without this override a recorded route_diff
# --track-width 0.2 set the Basic-tab width and left the diff tab at its default,
# so a plan-replayed diff routed at the wrong width.
# #381 D7: qfn_fanout.py's --width/--clearance map to the QFN panel's own
# controls (default 0.1/0.1), not the Basic-tab track_width/clearance that
# BGA/route use, so a plan-replayed QFN fanout keeps its fine-pitch width.
TOOL_FLAG_PARAMS = {
    'route_diff.py': {'--track-width': 'diff_pair_width'},
    # qfn_fanout's --layer: the QFN panel's escape-layer choice (its item 0,
    # the mounted layer, is the omitted flag), selected by layer name.
    # --clearance -> qfn_clearance, which the executor applies by TICKING the
    # panel's Clearance override (#1328). An omitted --clearance emits no
    # param, the executor's reset leaves the box unticked, and the panel then
    # resolves the board's Default class exactly as the CLI does; a
    # --clearance-ceiling maps to the global clearance_ceiling row, which the
    # panel's resolver caps at.
    'qfn_fanout.py': {'--width': 'qfn_track_width', '--clearance': 'qfn_clearance',
                      '--layer': 'qfn_layer'},
    # bga_fanout's coupled-pair gap is the BGA panel's OWN control, never the
    # diff tab's diff_pair_gap that the global row targets (#493: that one
    # resolves to the net-class gap).
    'bga_fanout.py': {'--diff-pair-gap': 'bga_diff_pair_gap'},
}

# Per-tool list flags (nargs='*' / '+') whose GUI home is one text field.
# bga_fanout's --diff-pairs names the pairs to couple; the BGA panel's Coupled
# pairs field takes the patterns space-joined. A bare flag is [] -- no pairs,
# as in the CLI, where an empty list is falsy -- so it is not an
# optional-list flag with a True for bare.
TOOL_LIST_FLAGS = {
    'bga_fanout.py': {'--diff-pairs': 'diff_pair_patterns_ctrl'},
}

# Per-tool flags taking an OPTIONAL list (argparse nargs='*'). route.py and
# route_diff.py spell --no-bga-zones that way: bare disables EVERY BGA
# exclusion zone, `--no-bga-zones U1 U3` only those components' zones. It used
# to sit in BOOL_FLAGS alone, which set "disable ALL" and dropped the refs --
# as positional net globs when the step had no --nets (23 recorded commands
# name refs, all with --nets, so the refs were silently discarded). The param
# is True for bare and the ref list otherwise; ai_plan writes either into the
# route tab's no_bga_zones_ctrl, which the GUI parses to the same [] / refs
# list the CLI hands batch_route(disable_bga_zones=...). route_planes /
# repair_planes take the flag as a plain switch, so BOOL_FLAGS serves them.
TOOL_OPTIONAL_LIST_FLAGS = {
    'route.py': {'--no-bga-zones': 'no_bga_zone',
                 '--no-bga-zone': 'no_bga_zone'},
    'route_diff.py': {'--no-bga-zones': 'no_bga_zone'},
}


def _num(v):
    try:
        f = float(v)
        return int(f) if f == int(f) else f
    except ValueError:
        return v


# Plan params naming a FILE the CLI opened relative to ITS cwd. The GUI would
# resolve a relative path against KiCad's own cwd, so parse_command makes it
# absolute against the command's recorded cwd -- what #1067 does for a cap
# step's --intent. (--fab-overrides was carried raw: a relative override file
# then went unread, and the replay ran at the bare tier with escalation on.)
PATH_PARAMS = {'net_clearances_path', 'fab_overrides_path'}


def parse_command(argv, cwd=None):
    tool = None
    for a in argv:
        base = os.path.basename(a)
        if base in TOOL_ACTIONS:
            tool = base
            break
        if base in REFUSED_TOOLS:
            # Named and reported, not silently tallied into `skipped`.
            return {'_refused': f"{base}: {REFUSED_TOOLS[base]}"}
        if base in NOOP_TOOLS:
            return {'_noop': f"{base}: {NOOP_TOOLS[base]}"}
    if tool is None:
        return None
    # A `--help` invocation (the agent inspecting a tool during the run) is not a
    # routing step -- skip it so the plan doesn't carry no-op `help: true` steps.
    if '--help' in argv or '-h' in argv:
        return None
    # #459: --preview writes no board and --undo REMOVES copper. The plan format
    # has no way to say either, and the unknown-flag fallthrough turns both into
    # an ordinary route step with nets ['*'] -- so a replayed --undo would ROUTE
    # the whole board where the CLI unrouted a block, the exact inverse. There is
    # no faithful conversion, so refuse loudly rather than emit a wrong one; the
    # caller prints these so a dropped step is never silent.
    for _flag, _why in (('--preview', 'writes no board; nothing to replay'),
                        ('--list-groups', 'prints a listing and exits; '
                                          'converting it would ROUTE the board'),
                        ('--undo', 'removes copper; the plan format cannot '
                                   'express an unroute, and converting it '
                                   'would ROUTE those nets instead')):
        if _flag in argv:
            return {'_refused': f"{tool} {_flag}: {_why}"}
    action = TOOL_ACTIONS[tool]
    step = {'action': action, 'params': {}}
    step['_files'] = []  # positional .kicad_pcb args (input/output), for pruning
    lists = {}
    # Normalize `--flag=value` to `--flag value` (recorded manifests mix both
    # forms; core64_logic's `--nets=-BATT` otherwise fell into the unknown-flag
    # branch as a mangled param, left --nets empty, and cascaded into EMPTY
    # plane assignments even though --plane-layers parsed fine).
    argv = [t for a in argv
            for t in (a.split('=', 1) if a.startswith('--') and '=' in a
                      else (a,))]
    i = argv.index([a for a in argv if os.path.basename(a) == tool][0]) + 1
    positional = []
    aliases = TOOL_FLAG_ALIASES.get(tool, {})
    tool_params = TOOL_FLAG_PARAMS.get(tool, {})
    tool_optional_lists = TOOL_OPTIONAL_LIST_FLAGS.get(tool, {})
    tool_lists = TOOL_LIST_FLAGS.get(tool, {})
    while i < len(argv):
        a = aliases.get(argv[i], argv[i])
        if a in tool_optional_lists or a in tool_lists:
            # Stop at a positional board file, like --component below.
            vals = []
            i += 1
            while (i < len(argv) and not argv[i].startswith('--')
                   and not argv[i].endswith('.kicad_pcb')):
                vals.append(str(argv[i]))
                i += 1
            if a in tool_lists:
                step['params'][tool_lists[a]] = vals
            else:
                step['params'][tool_optional_lists[a]] = vals or True
        elif a in IGNORE_FLAGS:
            i += 1
            while i < len(argv) and not argv[i].startswith('--'):
                if argv[i].endswith('.kicad_pcb'):
                    step['_files'].append(argv[i])
                i += 1
        elif a in BOOL_FLAGS:
            step['params'][BOOL_FLAGS[a]] = True
            i += 1
        elif a in INVERTED_BOOL_FLAGS:
            step['params'][INVERTED_BOOL_FLAGS[a]] = False
            i += 1
        elif a in GROUP_LIST_FLAGS:
            # Stop at a positional board file too, like --component below: the
            # patterns must not swallow the output path out of step['_files'].
            vals = []
            i += 1
            while (i < len(argv) and not argv[i].startswith('--')
                   and not argv[i].endswith('.kicad_pcb')):
                vals.append(str(argv[i]))
                i += 1
            if vals:
                step['params'].setdefault(GROUP_LIST_FLAGS[a], []).append(vals)
        elif a in tool_params or a in FLAG_PARAMS:
            key = tool_params.get(a) or FLAG_PARAMS[a]
            val = _num(argv[i + 1])
            if (key in PATH_PARAMS and cwd and isinstance(val, str)
                    and not os.path.isabs(val)):
                val = os.path.normpath(os.path.join(cwd, val))
            step['params'][key] = val
            i += 2
        elif a in LIST_FLAGS:
            vals = []
            i += 1
            while i < len(argv) and not argv[i].startswith('--'):
                vals.append(_num(argv[i]))
                i += 1
            if a in EXTEND_LIST_FLAGS:
                lists.setdefault(a, []).extend(vals)
            else:
                lists[a] = vals
        elif a in ('--group', '--group-scope', '--group-by'):
            # #459 placement blocks: these must land as TOP-LEVEL step keys,
            # not in params. ai_plan's route action reads step["group"] /
            # ["group_by"] / ["group_scope"] directly (_group_net_names); the
            # generic unknown-flag path below would file them under params,
            # where nothing reads them -- and per that function's own comment
            # the step then silently widens to the WHOLE BOARD where the CLI
            # routed one block. Same shape as --component just below.
            i += 1
            if i < len(argv) and not argv[i].startswith('--'):
                step[a.lstrip('-').replace('-', '_')] = argv[i]
                i += 1
        elif a == '--component':
            # #537: --component now takes one or more references. One converts
            # to the `component` key the plan executor already understands;
            # several are kept under `components` so the step is not silently
            # narrowed to the first footprint (the GUI list control that
            # consumes them is still to come -- see the issue's Tier 2).
            # Stop at a positional board file: bga_fanout/qfn_fanout still take
            # a SINGLE --component, and `--component U9 out.kicad_pcb` must not
            # swallow the output path out of step['_files'] (which pruning uses).
            vals = []
            i += 1
            while (i < len(argv) and not argv[i].startswith('--')
                   and not argv[i].endswith('.kicad_pcb')):
                vals.append(argv[i])
                i += 1
            if len(vals) == 1:
                step['component'] = vals[0]
            elif vals:
                step['components'] = vals
        elif a.startswith('--'):
            # unknown flag: skip it and any non-flag values (still carried
            # generically when it maps to a control name)
            key = a.lstrip('-').replace('-', '_')
            vals = []
            i += 1
            while i < len(argv) and not argv[i].startswith('--'):
                vals.append(_num(argv[i]))
                i += 1
            if len(vals) == 1:
                step['params'][key] = vals[0]
            elif vals:
                step['params'][key] = vals
            else:
                step['params'][key] = True
        else:
            positional.append(a)
            if a.endswith('.kicad_pcb'):
                step['_files'].append(a)
            i += 1

    nets = lists.get('--nets', [])
    if action in ('route',):
        # route.py also takes net names POSITIONALLY, after the input and output
        # boards -- --nets is optional:
        #     route.py in.kicad_pcb out.kicad_pcb '/Mgmt/LED0' '/Mgmt/VSMPS' ...
        # Same defect as the route_diff branch below: those positionals were
        # collected and dropped, so the step converted to nets: ['*'] -- the
        # exact OPPOSITE of what was recorded, routing every net on the board
        # instead of the handful the CLI retried. eth_tap steps 12 and 16 are
        # both positional retries of specific failed nets.
        # Empty still legitimately means "all nets" (what the CLI does with no
        # net args), so the ['*'] fallback stays for genuinely net-less steps.
        # NOT for a --component step: route.py defaults to '*' only when no
        # component is named either, and a component-only step routes that
        # component's nets LESS power/ground, which a '*' pattern would turn
        # into the intersection that keeps them. ai_plan reads the empty list
        # as "no patterns", the CLI's own reading.
        net_globs = [p for p in positional if not p.endswith('.kicad_pcb')]
        has_refs = 'component' in step or 'components' in step
        step['nets'] = ([str(n) for n in (nets or net_globs)]
                        or ([] if has_refs else ['*']))
    elif action == 'route_diff':
        # route_diff.py takes its pair patterns POSITIONALLY, after the input and
        # output boards -- there is no --pairs flag on the real CLI:
        #     route_diff.py in.kicad_pcb out.kicad_pcb '*ETH0_A_*' '*ETH0_B_*' ...
        # Those were collected into `positional` and then dropped on the floor,
        # so every recorded diff step converted to `pairs: []`. The GUI reads an
        # empty pairs list as "route every auto-detected pair", so a replayed
        # plan routed pairs the CLI never touched (eth_tap: +22 segments on
        # VCP_P/N and C2_P/N, two pairs absent from the recorded chain, which
        # then cascaded through every later step). 204 of the corpus's 206
        # recorded diff steps were affected.
        # Empty still legitimately means "all pairs" -- that is what the CLI does
        # when given no patterns -- so the ambiguity resolves itself once the
        # patterns actually survive.
        pair_globs = [p for p in positional if not p.endswith('.kicad_pcb')]
        step['pairs'] = [str(n) for n in
                         (lists.get('--pairs') or nets or pair_globs)]
    elif action in ('route_planes', 'repair_planes'):
        layers = [str(l) for l in lists.get('--plane-layers', [])]
        net_names = [str(n) for n in nets]
        if net_names and layers:
            step['assignments'] = [
                {'nets': [n], 'layer': layers[min(k, len(layers) - 1)]}
                for k, n in enumerate(net_names)]
        elif net_names and action == 'route_planes':
            # No --plane-layers (route_planes auto-detects zones): emit a
            # layer-less assignment; the GUI plane tab resolves the layer.
            step['assignments'] = [{'nets': net_names, 'layer': ''}]
        # repair_planes with no --plane-layers: emit NO assignments, so the
        # post-pass below inherits the preceding route_planes step's REAL
        # layers. route_disconnected_planes auto-detects zones on the CLI, but
        # the GUI repair needs explicit copper layers -- an empty-layer
        # assignment blocks the inherit fallback ("no valid copper layers in
        # ['']") and the whole repair step is silently dropped.
    elif action == 'fanout':
        step['kind'] = 'bga' if tool == 'bga_fanout.py' else 'qfn'
        step['nets'] = [str(n) for n in nets] or ['*']
    # --plane-net-layers was collected above and then never copied out: the
    # LIST_FLAGS row's own comment says a converted plan used to drop it, and
    # it still did (26 kept bga_fanout steps on 18 corpus boards).
    for k in ('--power-nets', '--power-nets-widths', '--layer-costs',
              '--layers', '--polarity-swap-nets', '--coplanar-nets',
              '--rip-existing-nets', '--plane-net-layers', '--keep-away'):
        if k in lists:
            step['params'][LIST_FLAGS[k]] = lists[k]
    return step


# place_fanout_clearance.py has no standalone CLI-tool action, but in the GUI it
# IS a plan step in its own right: the live Claude-plan format (ai_plan.py's
# KNOWN_ACTIONS + _insert_cap_optimization) represents "Optimize decoupling cap
# placement" (#130) as a SEPARATE `optimize_caps` step placed right after the last
# BGA fanout, run via fanout_tab.run_cap_optimization() -- NOT as a param on the
# fanout step. So a recorded place_fanout_clearance emits the same standalone step
# (in its manifest position, i.e. after the fanout it followed), for parity with a
# live-generated plan. Flag -> fanout-tab cap-placement control name:
CAP_FLAG_PARAMS = {
    # #768. NOT a cap_* name, for the same reason --board-edge-clearance below
    # is not: its GUI home is the SHARED Basic-tab clearance control, which is
    # what fanout_gui's cap step reads. Absent from this table, a recorded
    # `place_fanout_clearance.py ... --clearance 0.1` silently lost its
    # clearance on conversion, and the GUI re-derived a different one -- on the
    # one parameter whose two branches decide whether the step clamps the
    # project at all.
    '--clearance': 'clearance',
    '--capture-radius': 'cap_capture_radius',
    '--near-margin': 'cap_near_margin',
    '--step': 'cap_step',
    '--max-displacement': 'cap_max_displacement',
    '--max-displacement-cap': 'cap_max_displacement_cap',
    '--displacement-growth': 'cap_displacement_growth',
    '--max-passes': 'cap_max_passes',
    '--cap-prefix': 'cap_prefix',
    # #772. This row used to say 'board_edge_clearance', with a comment
    # claiming ai_plan's _GEOMETRY_OVERRIDE_CHECKS 'already maps by this exact
    # name ... so a plan carrying it ticks the override box and the value
    # reaches the engine'. That became FALSE the moment the #733 FOLLOW-UP
    # gave the cap margin its own control: the Basic tab's Min Edge Clearance
    # is the SIGNAL copper-to-edge keep-out, a different quantity that only
    # shares a flag SPELLING across two independent tools, and
    # get_shared_params stopped emitting cap_board_edge_clearance in the same
    # change. Measured on the real headless dialog: `--board-edge-clearance
    # 0.85` set the SIGNAL control, TICKED its override (which then leaked
    # into the next step's routing), and the cap engine received None.
    #
    # The right name is the CONTROL's name -- the rule the nine rows above
    # already follow. An explicit 0 needs no special case: the spin maps 0 to
    # None and resolve_cap_edge_clearance applies the same
    # non-positive-is-unset rule to an explicit CLI value, so both fronts land
    # on the same resolved margin. Only an EXPLICIT flag needs carrying; an
    # omitted one resolves identically on both fronts.
    '--board-edge-clearance': 'cap_board_edge_clearance',
    # #772: the cap pass snaps its candidate positions to this and reads it
    # off the Basic tab via get_shared_params. Without a row, a recorded
    # --grid-step was dropped and the step ran at whatever the previous one
    # left behind.
    '--grid-step': 'grid_step',
    # #742. It used to be listed below as deliberately unmapped, on the
    # reasoning that its "only same-named GUI home is the Basic tab's
    # via_size" -- true, and the reason mapping it THERE would have been a bug
    # (it would tick the via-size override and feed
    # PlanExecutor._write_drc_floors). The answer was a distinct control, not
    # no control: since #732 this value decides which vias the nudge relocates
    # and the tolerance that draws a connector segment back to a stub, so a
    # replay that dropped it produced different COPPER.
    '--default-via-size': 'cap_default_via_size',
    # #1067: the floorplan intent whose decap limits the cap pass holds. A
    # path, carried as written; the GUI resolves a relative one against the
    # board's folder.
    '--intent': 'cap_intent_path',
}
# Deliberately NOT mapped, and why:
#   --lock              nargs='+' extra locked refs; the GUI has no control.
#   --verbose           console verbosity, not a routing parameter.
CAP_BOOL_FLAGS = {'--no-rotate': ('cap_allow_rotation', False),  # inverted sense
                  '--beneath-only': ('cap_beneath_only', True)}


def cap_optimization_step(argv, cwd=None):
    """A place_fanout_clearance.py invocation -> a standalone `optimize_caps` plan
    step (matching ai_plan.py's live format), carrying the non-default cap_*
    knobs so a loaded plan optimizes caps the way the recorded run did."""
    params = {}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in CAP_FLAG_PARAMS and i + 1 < len(argv):
            params[CAP_FLAG_PARAMS[a]] = _num(argv[i + 1]); i += 2
            # #1067: the CLI read a relative intent from ITS cwd; the GUI
            # would read it from the board's folder. Make it absolute here.
            if (CAP_FLAG_PARAMS[a] == 'cap_intent_path' and cwd
                    and not os.path.isabs(argv[i - 1])):
                params['cap_intent_path'] = os.path.normpath(
                    os.path.join(cwd, argv[i - 1]))
        elif a in CAP_BOOL_FLAGS:
            key, val = CAP_BOOL_FLAGS[a]; params[key] = val; i += 1
        else:
            i += 1
    step = {'action': 'optimize_caps'}
    if params:
        step['params'] = params
    return step


def prune_manifest(cmds, manifest=None):
    """(keep, rejected) for a parsed manifest: the indices of the commands a
    plan is built from, and {index: why} for every converted-tool command its
    tool would refuse before routing (cli_argv_check): at argparse, or in a
    post-parse check its main makes on the arguments or on a file they name,
    resolved against the command's recorded cwd (else the manifest's folder).

    A rejected command ran nothing and wrote no board, so it is pruned as if it
    were not in the manifest: a board it names binds to the command that really
    wrote it last, and the rejected command is never kept. The plain pruner
    cannot know this -- the manifest records no exit status -- and kept such a
    command whenever it was the last to NAME a board a later step read; this
    module then converted it into a plan step that ran, and succeeded, in the
    GUI (hexberry_fpga's `qfn_fanout.py ... --dry-run` is the recorded case:
    qfn_fanout has no --dry-run, so the CLI exited 2 with no output).
    """
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from redo_stress_test import compute_prune_keep, is_check_cmd
    from cli_argv_check import rejection
    rejected = {}
    for i, (_cwd, argv) in enumerate(cmds):
        if is_check_cmd(argv) or '--help' in argv or '-h' in argv:
            continue
        why = rejection(argv, cwd=_cwd or (os.path.dirname(os.path.abspath(
            manifest)) if manifest else None))
        if why:
            rejected[i] = why
    keep, _info = compute_prune_keep(
        [(c, ['true']) if i in rejected else (c, a)
         for i, (c, a) in enumerate(cmds)])
    return keep - set(rejected), rejected


def plan_steps_from_manifest(manifest, keep_files=False):
    """(steps, skipped_count) for a recorded manifest -- the whole conversion.

    THE single implementation: main() (and make_plan.py) dump these steps as the
    plan JSON, and tests/gui_parity/replay_plan_vs_run.py calls it with
    ``keep_files=True`` to pair each step with the CLI chain board it produced.
    It used to be a loop in main() with a hand-kept copy in the parity harness.

    ``keep_files``: keep each step's private ``_files`` (its CLI input/output
    boards). The plan JSON must NOT carry them, so main() leaves this False.
    """
    # Prune with redo_stress_test's canonical file-dependency logic so the plan
    # is EXACTLY the simplified/deduplicated chain a replay runs -- the same
    # "N of M commands" set, with superseded retries and dead-end branches
    # dropped. Pruning on the FULL command set (before GUI-mapping) is what makes
    # this correct: a kept command the GUI can't represent as a step
    # (place_fanout_clearance.py, board_image.py) must still hold the file chain
    # together. The old inline pruner walked only GUI-recognized steps, so a
    # skipped intermediate (e.g. place_fanout_clearance) BROKE the chain and
    # silently dropped legitimate upstream steps (e.g. the whole bga_fanout).
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'py_router'))  # #522/py_placer layout
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'py_placer'))  # #522/py_placer layout
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'py_tools'))  # #522/py_placer layout
    from redo_stress_test import parse_manifest, is_check_cmd, split_env_prefix

    cmds = parse_manifest(manifest)
    keep, rejected = prune_manifest(cmds, manifest)
    # #1337: a rejected command writes no board, so the chain breaks at it
    # and everything that fed ONLY it falls out of the prune too. Those are
    # named and counted with it: daplink's one rejected cap pass took the
    # plane, fanout and copy steps with it while the summary said "1
    # command(s) skipped".
    lost = []
    if rejected:
        from redo_stress_test import compute_prune_keep
        plain, _ = compute_prune_keep(cmds)
        lost = sorted(i for i in plain - keep - set(rejected)
                      if not is_check_cmd(cmds[i][1]))

    steps = []
    skipped = len(rejected) + len(lost)
    refused = []
    noops = []
    env_dropped = []
    for i, (_cwd, argv) in enumerate(cmds):
        if i not in keep or is_check_cmd(argv):
            continue  # pruned out, or a check/grade command (no GUI step)
        # #1302: a recorded `NAME=value` prefix is the step's environment. A
        # plan step has nowhere to carry it (#670), so the GUI runs the step
        # at the defaults; converted without it, and named below.
        _env, argv = split_env_prefix(argv)
        if any(os.path.basename(a) == CAP_TOOL for a in argv):
            # standalone optimize_caps step, matching the live GUI plan (see above)
            step = cap_optimization_step(argv, cwd=_cwd)
            step['_files'] = [a for a in argv if a.endswith('.kicad_pcb')]
            steps.append(step)
            if _env:
                env_dropped.append((i, _env))
            continue
        # A relative file the command named was opened from its recorded cwd
        # (else the manifest's own folder, where the harness ran it).
        step = parse_command(argv, cwd=_cwd or os.path.dirname(
            os.path.abspath(manifest)))
        if step is None:
            # kept-but-not-GUI-representable (place_fanout_clearance, board_image,
            # --help, ...): pruning already accounted for it -- just emit no step.
            skipped += 1
            continue
        if '_refused' in step:
            # A command with no faithful plan representation (#459 --undo /
            # --preview). Emitting nothing is the safe half; the caller MUST
            # report these, because an unroute silently missing from a replay
            # leaves copper the recorded chain had removed.
            refused.append(step['_refused'])
            skipped += 1
            continue
        if '_noop' in step:
            # Nothing to replay on a live board (NOOP_TOOLS); named, so the
            # skip is a declared one rather than an unknown tool's.
            noops.append(step['_noop'].split(':', 1)[0])
            skipped += 1
            continue
        if step['action'] == 'repair_planes' and 'assignments' not in step:
            # The repair CLI auto-detects zones; the GUI repair needs
            # explicit assignments -- inherit the last plane step's.
            for prev in reversed(steps):
                if prev['action'] == 'route_planes' and prev.get('assignments'):
                    step['assignments'] = [dict(a) for a in prev['assignments']]
                    break
        steps.append(step)
        if _env:
            env_dropped.append((i, _env))

    if not keep_files:
        for s in steps:
            s.pop('_files', None)
    if rejected:
        # Loud and counted, like the refusals below: the plan does what the
        # CLI chain did for these, which is nothing.
        print(f"WARNING: {len(rejected)} recorded command(s) were REJECTED by "
              f"their tool's own argument parser -- the CLI ran nothing for "
              f"them -- and are not plan steps:", file=sys.stderr)
        for i, why in sorted(rejected.items()):
            print(f"  - command {i + 1}: {why}", file=sys.stderr)
        if lost:
            print(f"  ...and {len(lost)} command(s) upstream of them are not "
                  f"plan steps either: the boards they wrote reach the final "
                  f"board only through a rejected command:", file=sys.stderr)
            for i in lost:
                tool = next((os.path.basename(a) for a in cmds[i][1]
                             if a.endswith('.py')), cmds[i][1][0])
                print(f"  - command {i + 1}: {tool}", file=sys.stderr)
    if refused:
        # Loud, on stderr, every time -- never a silent drop.
        print(f"WARNING: {len(refused)} command(s) have NO faithful plan "
              f"representation and were NOT converted:", file=sys.stderr)
        for r in refused:
            print(f"  - {r}", file=sys.stderr)
        print("  The replayed plan will DIVERGE from the recorded chain at "
              "these steps. Run them on the CLI.", file=sys.stderr)
    if env_dropped:
        # #1302: loud, like the refusals -- the plan step runs, but not as
        # the recorded one did.
        # One line per distinct knob set: a plan.sh `export` puts the same
        # set on every step after it (#1337).
        print(f"WARNING: {len(env_dropped)} plan step(s) ran with env knobs "
              f"a plan step cannot carry (#670); the GUI runs them at the "
              f"defaults, so the replayed plan may route a different board:",
              file=sys.stderr)
        groups = {}
        for i, env in env_dropped:
            groups.setdefault(tuple(env.items()), []).append(i + 1)
        for env, nums in groups.items():
            print(f"  - {' '.join(f'{k}={v}' for k, v in env)}: command(s) "
                  f"{', '.join(map(str, nums))}", file=sys.stderr)
        print("  Export them in the KiCad session's environment, or replay "
              "the manifest on the CLI.", file=sys.stderr)
    if noops:
        counts = {n: noops.count(n) for n in sorted(set(noops))}
        print("NOTE: no plan step for " + ", ".join(
            f"{n}" + (f" x{c}" if c > 1 else "") for n, c in counts.items())
              + " -- nothing to replay on a live board (NOOP_TOOLS)",
              file=sys.stderr)
    return steps, skipped


def main():
    # Output is POSITIONAL, but `-o/--output` is accepted too: the -o form is the
    # reflex for a tool like this, and the old raw-argv parse silently took the
    # literal string '-o' AS the output path -- writing a plan to a file named
    # `-o` in the cwd while the real target kept its stale contents. That looks
    # exactly like "the converter is broken" until you check the file's mtime.
    argv, positional, out = sys.argv[1:], [], None
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ('-h', '--help'):
            print(__doc__)
            return 0
        if a in ('-o', '--output'):
            if i + 1 >= len(argv):
                print("error: -o/--output needs a path")
                return 2
            out, i = argv[i + 1], i + 2
            continue
        positional.append(a)
        i += 1
    manifest = positional[0] if positional else None
    if out is None and len(positional) > 1:
        out = positional[1]
    if not manifest or not out:
        print(__doc__)
        return 2
    if out.startswith('-'):
        # Belt and braces: never write to something that looks like a flag.
        print(f"error: refusing to write the plan to {out!r} -- that looks like a "
              f"flag, not a path. Use: manifest_to_plan.py <manifest> <plan.json>")
        return 2
    steps, skipped = plan_steps_from_manifest(manifest)
    with open(out, 'w', encoding='utf-8') as f:
        json.dump({'steps': steps}, f, indent=2)
    print(f"{len(steps)} step(s) written to {out} "
          f"({skipped} command(s) skipped: kept but not GUI steps, "
          f"rejected by their tool's parser, or upstream of a rejected one)")
    print("Load it in the AI tab (Load... next to 'Parsed result') and "
          "press 'Run Selected Steps'.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
