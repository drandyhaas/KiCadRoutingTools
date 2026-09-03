"""
KiCad Routing Tools - Dialog Settings Persistence

Handles saving and restoring dialog settings between invocations.
"""


def get_dialog_settings(dialog):
    """Get all current dialog settings for persistence.

    Args:
        dialog: RoutingDialog instance

    Returns:
        dict: All settings that should be persisted
    """
    settings = {
        # Tab selection
        'active_tab': dialog.notebook.GetSelection(),

        # Net selections (use net names for persistence across reloads)
        'net_panel_checked': list(dialog.net_panel.get_selected_nets()),
        'swappable_net_panel_checked': list(dialog.swappable_net_panel.get_selected_nets()),
        'fanout_net_panel_checked': list(dialog.fanout_tab.net_panel.get_selected_nets()),
        'diff_pairs_checked': list(dialog.differential_tab.pair_panel._checked_pairs),

        # Basic tab parameters
        'track_width': dialog.track_width.GetValue(),
        'clearance': dialog.clearance.GetValue(),
        'via_size': dialog.via_size.GetValue(),
        'via_drill': dialog.via_drill.GetValue(),
        'grid_step': dialog.grid_step.GetValue(),
        'via_cost': dialog.via_cost.GetValue(),
        'max_ripup': dialog.max_ripup.GetValue(),
        'ripup_abandon_metric': dialog.ripup_abandon_metric.GetString(
            dialog.ripup_abandon_metric.GetSelection()),
        'ripup_blocker_select': dialog.ripup_blocker_select.GetString(
            dialog.ripup_blocker_select.GetSelection()),
        'obey_design_rules': dialog.obey_drc_check.GetValue(),

        # Layer selections
        'layers': [layer for layer, cb in dialog.layer_checks.items() if cb.GetValue()],

        # Basic options
        # #581: via-in-pad policy (moved from the planes tab to the Basic tab)
        'allow_via_in_pad': dialog.via_in_pad_check.GetValue(),
        'same_net_pad_clearance': dialog.same_net_pad_clearance.GetValue(),
        'enable_layer_switch': dialog.enable_layer_switch.GetValue(),
        'move_text_check': dialog.move_text_check.GetValue(),
        'add_teardrops_check': dialog.add_teardrops_check.GetValue(),
        'fix_drc_settings': dialog.fix_drc_check.GetValue(),
        'relax_drc_severities': dialog.relax_drc_severities_check.GetValue(),
        'power_nets': dialog.power_nets_ctrl.GetValue(),
        'power_widths': dialog.power_widths_ctrl.GetValue(),
        'no_bga_zones': dialog.no_bga_zones_ctrl.GetValue(),
        'rip_existing_nets': dialog.rip_existing_nets_ctrl.GetValue(),
        'layer_costs': dialog.layer_costs_ctrl.GetValue(),

        # Advanced parameters
        'impedance_check': dialog.impedance_check.GetValue(),
        'impedance_value': dialog.impedance_value.GetValue(),
        'coplanar_gap': dialog.coplanar_gap.GetValue(),
        'coplanar_nets': dialog.coplanar_nets_ctrl.GetValue(),
        'max_iterations': dialog.max_iterations.GetValue(),
        'max_probe_iterations': dialog.max_probe_iterations.GetValue(),
        'heuristic_weight': dialog.heuristic_weight.GetValue(),
        'turn_cost': dialog.turn_cost.GetValue(),
        'direction_preference_cost': dialog.direction_preference_cost.GetValue(),
        'ordering_strategy': dialog.ordering_strategy.GetSelection(),
        'fab_tier': dialog.fab_tier.GetSelection(),
        'fab_overrides_path': dialog.fab_overrides_path.GetValue(),
        'fab_overrides_recent': list(dialog.fab_overrides_path.GetStrings()),
        'bga_proximity_radius': dialog.bga_proximity_radius.GetValue(),
        'bga_proximity_cost': dialog.bga_proximity_cost.GetValue(),
        'stub_proximity_radius': dialog.stub_proximity_radius.GetValue(),
        'stub_proximity_cost': dialog.stub_proximity_cost.GetValue(),
        'neckdown_length': dialog.neckdown_length.GetValue(),
        'neckdown_taper_length': dialog.neckdown_taper_length.GetValue(),
        'power_tap_neckdown': dialog.power_tap_neckdown_check.GetValue(),
        'via_proximity_cost': dialog.via_proximity_cost.GetValue(),
        'track_proximity_distance': dialog.track_proximity_distance.GetValue(),
        'track_proximity_cost': dialog.track_proximity_cost.GetValue(),
        'vertical_attraction_radius': dialog.vertical_attraction_radius.GetValue(),
        'vertical_attraction_cost': dialog.vertical_attraction_cost.GetValue(),
        'ripped_route_avoidance_radius': dialog.ripped_route_avoidance_radius.GetValue(),
        'ripped_route_avoidance_cost': dialog.ripped_route_avoidance_cost.GetValue(),
        'routing_clearance_margin': dialog.routing_clearance_margin.GetValue(),
        'hole_to_hole_clearance': dialog.hole_to_hole_clearance.GetValue(),
        'edge_clearance_check': dialog.edge_clearance_check.GetValue(),
        'board_edge_clearance': dialog.board_edge_clearance.GetValue(),
        # #439 geometry-floor override checkboxes (checked = override, unchecked =
        # use the board's own value). Restored to also enable/disable the spinctrl.
        'track_width_override': dialog.track_width_check.GetValue(),
        'clearance_override': dialog.clearance_check.GetValue(),
        'via_size_override': dialog.via_size_check.GetValue(),
        'via_drill_override': dialog.via_drill_check.GetValue(),
        'hole_to_hole_clearance_override': dialog.hole_to_hole_clearance_check.GetValue(),
        'board_edge_clearance_override': dialog.edge_clearance_check.GetValue(),
        'direction_choice': dialog.direction_choice.GetSelection(),

        # Advanced options
        'mps_reverse_rounds': dialog.mps_reverse_rounds.GetValue(),
        'mps_layer_swap': dialog.mps_layer_swap.GetValue(),
        'keep_input_copper': dialog.keep_input_copper.GetValue(),
        'smoothing': dialog.smoothing.GetValue(),
        'force_reroute': dialog.force_reroute.GetValue(),
        'mps_segment_intersection': dialog.mps_segment_intersection.GetValue(),
        'no_crossing_layer_check': dialog.no_crossing_layer_check.GetValue(),
        'can_swap_to_top': dialog.can_swap_to_top.GetValue(),
        'crossing_penalty': dialog.crossing_penalty.GetValue(),

        # Bus routing options
        'bus_enabled': dialog.bus_enabled.GetValue(),
        'bus_detection_radius': dialog.bus_detection_radius.GetValue(),
        'bus_attraction_radius': dialog.bus_attraction_radius.GetValue(),
        'bus_attraction_bonus': dialog.bus_attraction_bonus.GetValue(),
        'bus_min_nets': dialog.bus_min_nets.GetValue(),

        # Guide corridor (issue #7)
        'guide_corridor_check': dialog.guide_corridor_check.GetValue(),
        'guide_corridor_layer': dialog.guide_corridor_layer_ctrl.GetValue(),
        'guide_corridor_spacing': dialog.guide_corridor_spacing_ctrl.GetValue(),

        # Keepout zone (issue #27)
        'keepout_check': dialog.keepout_check.GetValue(),
        'keepout_layer': dialog.keepout_layer_ctrl.GetValue(),

        # Clear User-layer graphics after a successful route
        'clear_guide_layer_check': dialog.clear_guide_layer_check.GetValue(),
        'clear_keepout_layer_check': dialog.clear_keepout_layer_check.GetValue(),

        'length_match_groups': dialog.length_match_groups_ctrl.GetValue(),
        'length_match_tolerance': dialog.length_match_tolerance.GetValue(),
        'meander_amplitude': dialog.meander_amplitude.GetValue(),
        'meander_spacing': dialog.meander_spacing.GetValue(),
        'time_matching_check': dialog.time_matching_check.GetValue(),
        'time_match_tolerance': dialog.time_match_tolerance.GetValue(),
        'debug_lines_check': dialog.debug_lines_check.GetValue(),
        'verbose_check': dialog.verbose_check.GetValue(),
        'skip_routing_check': dialog.skip_routing_check.GetValue(),
        'debug_memory_check': dialog.debug_memory_check.GetValue(),
        'stats_check': dialog.stats_check.GetValue(),
        'make_movie_check': dialog.make_movie_check.GetValue(),

        # Swappable nets options
        'update_schematic_check': dialog.update_schematic_check.GetValue(),
        'schematic_dir': dialog.schematic_dir_ctrl.GetValue(),

        # Hide checkboxes
        'net_panel_hide': dialog.net_panel.hide_check.GetValue() if dialog.net_panel.hide_check else False,
        'net_panel_hide_diff': dialog.net_panel.hide_diff_check.GetValue() if dialog.net_panel.hide_diff_check else False,
        'net_panel_separate_netclass': dialog.net_panel.separate_netclass_check.GetValue(),
        'swappable_hide': dialog.swappable_net_panel.hide_check.GetValue() if dialog.swappable_net_panel.hide_check else False,
        'diff_panel_hide': dialog.differential_tab.pair_panel.hide_check.GetValue() if dialog.differential_tab.pair_panel.hide_check else False,
        'fanout_hide': dialog.fanout_tab.net_panel.hide_check.GetValue() if dialog.fanout_tab.net_panel.hide_check else False,

        # Filters
        'net_panel_filter': dialog.net_panel.filter_ctrl.GetValue(),
        'swappable_filter': dialog.swappable_net_panel.filter_ctrl.GetValue(),
        'diff_panel_filter': dialog.differential_tab.pair_panel.filter_ctrl.GetValue(),
        'fanout_filter': dialog.fanout_tab.net_panel.filter_ctrl.GetValue(),

        # Component dropdowns
        'net_panel_component': dialog.net_panel.component_dropdown.GetSelection() if dialog.net_panel.component_dropdown else 0,
        'swappable_component': dialog.swappable_net_panel.component_dropdown.GetSelection() if dialog.swappable_net_panel.component_dropdown else 0,
        'diff_panel_component': dialog.differential_tab.pair_panel.component_dropdown.GetSelection() if dialog.differential_tab.pair_panel.component_dropdown else 0,
        'fanout_component': dialog.fanout_tab.net_panel.component_dropdown.GetSelection() if dialog.fanout_tab.net_panel.component_dropdown else 0,

        # Differential tab parameters
        'diff_pair_width_override': dialog.differential_tab.diff_pair_width_check.GetValue(),
        'diff_pair_gap_override': dialog.differential_tab.diff_pair_gap_check.GetValue(),
        'diff_pair_width': dialog.differential_tab.diff_pair_width.GetValue(),
        'diff_pair_gap': dialog.differential_tab.diff_pair_gap.GetValue(),
        'diff_impedance': dialog.differential_tab.diff_impedance.GetValue(),
        'min_turning_radius': dialog.differential_tab.min_turning_radius.GetValue(),
        'max_setback_angle': dialog.differential_tab.max_setback_angle.GetValue(),
        'max_turn_angle': dialog.differential_tab.max_turn_angle.GetValue(),
        'chamfer_extra': dialog.differential_tab.chamfer_extra.GetValue(),
        'centerline_setback': dialog.differential_tab.centerline_setback.GetValue(),
        'polarity_swap_nets_text': dialog.differential_tab.polarity_swap_nets_text.GetValue(),
        'gnd_via_check': dialog.differential_tab.gnd_via_check.GetValue(),
        'intra_match_check': dialog.differential_tab.intra_match_check.GetValue(),
        'ac_couple_check': dialog.differential_tab.ac_couple_check.GetValue(),
        'diff_hide_short': dialog.differential_tab.hide_short_check.GetValue(),

        # Fanout tab settings
        'fanout_type': dialog.fanout_tab.fanout_type.GetSelection(),
        'fanout_bga_exit_margin': dialog.fanout_tab.bga_options.exit_margin.GetValue(),
        'fanout_bga_differential': dialog.fanout_tab.bga_options.differential_check.GetValue(),
        'fanout_bga_escape_direction': dialog.fanout_tab.bga_options.escape_direction.GetSelection(),
        'fanout_bga_force_escape': dialog.fanout_tab.bga_options.force_escape.GetValue(),
        'fanout_bga_rebalance': dialog.fanout_tab.bga_options.rebalance_escape.GetValue(),
        'fanout_bga_check_previous': dialog.fanout_tab.bga_options.check_previous.GetValue(),
        'fanout_bga_no_inner_top': dialog.fanout_tab.bga_options.no_inner_top.GetValue(),
        'fanout_bga_escape_method': dialog.fanout_tab.bga_options.get_escape_method(),
        'fanout_bga_plane_drop': dialog.fanout_tab.bga_options.plane_drop.GetValue(),
        'fanout_bga_plane_net_layers': dialog.fanout_tab.bga_options.plane_net_layers_ctrl.GetValue(),
        'fanout_bga_optimize_caps': dialog.fanout_tab.bga_options.optimize_caps.GetValue(),
        'fanout_bga_cap_capture_radius': dialog.fanout_tab.bga_options.cap_capture_radius.GetValue(),
        'fanout_bga_cap_near_margin': dialog.fanout_tab.bga_options.cap_near_margin.GetValue(),
        'fanout_bga_cap_step': dialog.fanout_tab.bga_options.cap_step.GetValue(),
        'fanout_bga_cap_max_displacement': dialog.fanout_tab.bga_options.cap_max_displacement.GetValue(),
        'fanout_bga_cap_max_displacement_cap': dialog.fanout_tab.bga_options.cap_max_displacement_cap.GetValue(),
        'fanout_bga_cap_displacement_growth': dialog.fanout_tab.bga_options.cap_displacement_growth.GetValue(),
        'fanout_bga_cap_board_edge_clearance': dialog.fanout_tab.bga_options.cap_board_edge_clearance.GetValue(),
        'fanout_bga_cap_max_passes': dialog.fanout_tab.bga_options.cap_max_passes.GetValue(),
        'fanout_bga_cap_prefix': dialog.fanout_tab.bga_options.cap_prefix.GetValue(),
        'fanout_bga_cap_default_via_size': dialog.fanout_tab.bga_options.cap_default_via_size.GetValue(),
        'fanout_bga_cap_allow_rotation': dialog.fanout_tab.bga_options.cap_allow_rotation.GetValue(),
        'fanout_qfn_extension': dialog.fanout_tab.qfn_options.extension.GetValue(),
        # #381 D7: QFN-specific track width / clearance (default 0.1/0.1).
        'fanout_qfn_track_width': dialog.fanout_tab.qfn_options.qfn_track_width.GetValue(),
        'fanout_qfn_clearance': dialog.fanout_tab.qfn_options.qfn_clearance.GetValue(),
        'fanout_qfn_underpad': dialog.fanout_tab.qfn_options.underpad_escape.GetValue(),
        'fanout_qfn_allow_via_in_pad': dialog.fanout_tab.qfn_options.allow_via_in_pad.GetValue(),

        # Planes tab settings
        'planes_net_panel_checked': list(dialog.planes_tab.net_panel.get_selected_nets()),
        'planes_assignments': dialog.planes_tab.assignment_panel.get_assignments(),
        'planes_hide': dialog.planes_tab.net_panel.hide_check.GetValue() if dialog.planes_tab.net_panel.hide_check else False,
        'planes_filter': dialog.planes_tab.net_panel.filter_ctrl.GetValue(),
        'planes_component': dialog.planes_tab.net_panel.component_dropdown.GetSelection() if dialog.planes_tab.net_panel.component_dropdown else 0,
        # Create mode options
        'planes_zone_clearance': dialog.planes_tab.create_options.zone_clearance.GetValue(),
        'planes_thermal_relief': dialog.planes_tab.create_options.thermal_relief.GetValue(),
        'planes_thermal_vias': dialog.planes_tab.create_options.thermal_vias.GetValue(),
        'planes_add_gnd_vias': dialog.planes_tab.create_options.add_gnd_vias_check.GetValue(),
        'planes_gnd_via_distance': dialog.planes_tab.create_options.gnd_via_distance.GetValue(),
        'planes_stitch_vias': dialog.planes_tab.create_options.stitch_vias.GetValue(),
        'planes_stitch_pitch': dialog.planes_tab.create_options.stitch_pitch.GetValue(),
        'planes_stitch_edge_fence': dialog.planes_tab.create_options.stitch_edge_fence.GetValue(),
        'planes_stitch_fence_pitch': dialog.planes_tab.create_options.stitch_fence_pitch.GetValue(),
        'planes_stitch_inset': dialog.planes_tab.create_options.stitch_inset.GetValue(),
        'planes_stitch_max_freq': dialog.planes_tab.create_options.stitch_max_freq.GetValue(),
        'planes_gnd_via_net': dialog.planes_tab.create_options.gnd_via_net.GetValue(),
        # Repair mode options

        # AI tab settings (issue #40; backend selection #503). Model/effort
        # entries are stored per backend so switching backends doesn't lose
        # e.g. an opencode provider/model string; 'claude_model'/'claude_effort'
        # keep their pre-#503 meaning (the Claude Code backend's entries).
        'ai_backend': dialog.ai_tab.get_backend_value(),
        'claude_model': dialog.ai_tab.get_model_value_for('claude'),
        'claude_effort': dialog.ai_tab.get_effort_value_for('claude'),
        'opencode_model': dialog.ai_tab.get_model_value_for('opencode'),
        'opencode_effort': dialog.ai_tab.get_effort_value_for('opencode'),
        'ai_plan': dialog.ai_tab.get_plan_state(),

        # Placement sub-tab of the AI notebook (issue #481). The labels
        # options travel as ONE dict so the panel can grow fields without new
        # persistence keys; the transcript is deliberately NOT persisted (an
        # hours-long run's stream can be tens of MB).
        'ai_active_subtab': dialog.ai_notebook.GetSelection() if hasattr(dialog, 'ai_notebook') else 0,
        'placement_backend': dialog.placement_tab.get_backend_value(),
        'placement_model': dialog.placement_tab.get_model_value(),
        'placement_effort': dialog.placement_tab.get_effort_value(),
        'placement_extra_instructions': dialog.placement_tab.extra_instructions.GetValue(),
        'placement_last_workdir': getattr(dialog.placement_tab, 'last_workdir', '') or '',
        'placement_labels_options': dialog.placement_tab.labels_options.get_config(),

        # Log content
        'log_content': dialog.log_text.GetValue(),

        # Window transparency
        'window_transparency': dialog.about_tab.transparency_slider.GetValue() if hasattr(dialog, 'about_tab') else 240,
    }
    return settings


def restore_dialog_settings(dialog, settings):
    """Restore dialog settings from saved state.

    Args:
        dialog: RoutingDialog instance
        settings: dict of saved settings
    """
    if not settings:
        return

    # Suspend connectivity checks during restore to avoid expensive recalculations
    dialog.net_panel.suspend_check()
    dialog.swappable_net_panel.suspend_check()
    dialog.differential_tab.pair_panel.suspend_check()
    dialog.fanout_tab.net_panel.suspend_check()
    dialog.planes_tab.net_panel.suspend_check()

    # Restore tab selection
    if 'active_tab' in settings:
        dialog.notebook.SetSelection(settings['active_tab'])

    # Note: Net selections are restored at the END of this function
    # because setting filters/checkboxes below triggers events that clear selections

    # Restore basic parameters
    if 'track_width' in settings:
        dialog.track_width.SetValue(settings['track_width'])
    if 'clearance' in settings:
        dialog.clearance.SetValue(settings['clearance'])
    # #581: via-in-pad policy (Basic tab; absent in legacy dicts -> defaults
    # keep via-in-pad allowed). Restore the checkbox LAST so the spin's
    # enabled-state matches it.
    if 'same_net_pad_clearance' in settings:
        dialog.same_net_pad_clearance.SetValue(settings['same_net_pad_clearance'])
    if 'allow_via_in_pad' in settings:
        dialog.via_in_pad_check.SetValue(settings['allow_via_in_pad'])
        dialog.same_net_pad_clearance.Enable(not settings['allow_via_in_pad'])
    if 'via_size' in settings:
        dialog.via_size.SetValue(settings['via_size'])
    if 'via_drill' in settings:
        dialog.via_drill.SetValue(settings['via_drill'])
    if 'grid_step' in settings:
        dialog.grid_step.SetValue(settings['grid_step'])
    if 'via_cost' in settings:
        dialog.via_cost.SetValue(settings['via_cost'])
    if 'max_ripup' in settings:
        dialog.max_ripup.SetValue(settings['max_ripup'])
    if 'ripup_abandon_metric' in settings:
        dialog.ripup_abandon_metric.SetStringSelection(settings['ripup_abandon_metric'])
    if 'ripup_blocker_select' in settings:
        dialog.ripup_blocker_select.SetStringSelection(settings['ripup_blocker_select'])
    if 'obey_design_rules' in settings:
        dialog.obey_drc_check.SetValue(settings['obey_design_rules'])

    # Restore layer selections
    if 'layers' in settings:
        selected_layers = set(settings['layers'])
        for layer, cb in dialog.layer_checks.items():
            cb.SetValue(layer in selected_layers)

    # Restore basic options
    if 'enable_layer_switch' in settings:
        dialog.enable_layer_switch.SetValue(settings['enable_layer_switch'])
    if 'move_text_check' in settings:
        dialog.move_text_check.SetValue(settings['move_text_check'])
    if 'add_teardrops_check' in settings:
        dialog.add_teardrops_check.SetValue(settings['add_teardrops_check'])
    if 'fix_drc_settings' in settings:
        dialog.fix_drc_check.SetValue(settings['fix_drc_settings'])
    # 'keep_thermal' (legacy key, <= v0.21.5) is intentionally not restored:
    # the control it drove is gone (#856 -- routing steps no longer touch DRC
    # severities), replaced by the opt-in below.
    if 'relax_drc_severities' in settings:
        dialog.relax_drc_severities_check.SetValue(settings['relax_drc_severities'])
    if 'power_nets' in settings:
        dialog.power_nets_ctrl.SetValue(settings['power_nets'])
    if 'power_widths' in settings:
        dialog.power_widths_ctrl.SetValue(settings['power_widths'])
    if 'no_bga_zones' in settings:
        dialog.no_bga_zones_ctrl.SetValue(settings['no_bga_zones'])
    if 'rip_existing_nets' in settings:
        dialog.rip_existing_nets_ctrl.SetValue(settings['rip_existing_nets'])
    if 'layer_costs' in settings:
        dialog.layer_costs_ctrl.SetValue(settings['layer_costs'])

    # Restore advanced parameters
    if 'impedance_check' in settings:
        dialog.impedance_check.SetValue(settings['impedance_check'])
    if 'impedance_value' in settings:
        dialog.impedance_value.SetValue(settings['impedance_value'])
    if 'coplanar_gap' in settings:
        dialog.coplanar_gap.SetValue(settings['coplanar_gap'])
    if 'coplanar_nets' in settings:
        dialog.coplanar_nets_ctrl.SetValue(settings['coplanar_nets'])
    if 'max_iterations' in settings:
        dialog.max_iterations.SetValue(settings['max_iterations'])
    if 'max_probe_iterations' in settings:
        dialog.max_probe_iterations.SetValue(settings['max_probe_iterations'])
    if 'heuristic_weight' in settings:
        dialog.heuristic_weight.SetValue(settings['heuristic_weight'])
    if 'turn_cost' in settings:
        dialog.turn_cost.SetValue(settings['turn_cost'])
    if 'direction_preference_cost' in settings:
        dialog.direction_preference_cost.SetValue(settings['direction_preference_cost'])
    if 'ordering_strategy' in settings:
        dialog.ordering_strategy.SetSelection(settings['ordering_strategy'])
    if 'fab_overrides_recent' in settings:
        dialog.fab_overrides_path.Set(list(settings['fab_overrides_recent']))
    if 'fab_overrides_path' in settings:
        dialog.fab_overrides_path.SetValue(settings['fab_overrides_path'])
    if 'fab_tier' in settings:
        dialog.fab_tier.SetSelection(settings['fab_tier'])
    if 'bga_proximity_radius' in settings:
        dialog.bga_proximity_radius.SetValue(settings['bga_proximity_radius'])
    if 'bga_proximity_cost' in settings:
        dialog.bga_proximity_cost.SetValue(settings['bga_proximity_cost'])
    if 'stub_proximity_radius' in settings:
        dialog.stub_proximity_radius.SetValue(settings['stub_proximity_radius'])
    if 'stub_proximity_cost' in settings:
        dialog.stub_proximity_cost.SetValue(settings['stub_proximity_cost'])
    if 'neckdown_length' in settings:
        dialog.neckdown_length.SetValue(settings['neckdown_length'])
    if 'neckdown_taper_length' in settings:
        dialog.neckdown_taper_length.SetValue(settings['neckdown_taper_length'])
    if 'power_tap_neckdown' in settings:
        dialog.power_tap_neckdown_check.SetValue(settings['power_tap_neckdown'])
    if 'via_proximity_cost' in settings:
        dialog.via_proximity_cost.SetValue(settings['via_proximity_cost'])
    if 'track_proximity_distance' in settings:
        dialog.track_proximity_distance.SetValue(settings['track_proximity_distance'])
    if 'track_proximity_cost' in settings:
        dialog.track_proximity_cost.SetValue(settings['track_proximity_cost'])
    if 'vertical_attraction_radius' in settings:
        dialog.vertical_attraction_radius.SetValue(settings['vertical_attraction_radius'])
    if 'vertical_attraction_cost' in settings:
        dialog.vertical_attraction_cost.SetValue(settings['vertical_attraction_cost'])
    if 'ripped_route_avoidance_radius' in settings:
        dialog.ripped_route_avoidance_radius.SetValue(settings['ripped_route_avoidance_radius'])
    if 'ripped_route_avoidance_cost' in settings:
        dialog.ripped_route_avoidance_cost.SetValue(settings['ripped_route_avoidance_cost'])
    if 'routing_clearance_margin' in settings:
        dialog.routing_clearance_margin.SetValue(settings['routing_clearance_margin'])
    if 'hole_to_hole_clearance' in settings:
        dialog.hole_to_hole_clearance.SetValue(settings['hole_to_hole_clearance'])
    if 'edge_clearance_check' in settings:
        dialog.edge_clearance_check.SetValue(settings['edge_clearance_check'])
        dialog.board_edge_clearance.Enable(settings['edge_clearance_check'])
    if 'board_edge_clearance' in settings:
        dialog.board_edge_clearance.SetValue(settings['board_edge_clearance'])
    # #439 geometry-floor override checkboxes: restore checked state AND the
    # matching spinctrl enable so the row round-trips like the edge control.
    if 'track_width_override' in settings:
        dialog.track_width_check.SetValue(settings['track_width_override'])
        dialog.track_width.Enable(settings['track_width_override'])
    if 'clearance_override' in settings:
        dialog.clearance_check.SetValue(settings['clearance_override'])
        dialog.clearance.Enable(settings['clearance_override'])
    if 'via_size_override' in settings:
        dialog.via_size_check.SetValue(settings['via_size_override'])
        dialog.via_size.Enable(settings['via_size_override'])
    if 'via_drill_override' in settings:
        dialog.via_drill_check.SetValue(settings['via_drill_override'])
        dialog.via_drill.Enable(settings['via_drill_override'])
    if 'hole_to_hole_clearance_override' in settings:
        dialog.hole_to_hole_clearance_check.SetValue(settings['hole_to_hole_clearance_override'])
        dialog.hole_to_hole_clearance.Enable(settings['hole_to_hole_clearance_override'])
    if 'board_edge_clearance_override' in settings:
        dialog.edge_clearance_check.SetValue(settings['board_edge_clearance_override'])
        dialog.board_edge_clearance.Enable(settings['board_edge_clearance_override'])
    if 'direction_choice' in settings:
        dialog.direction_choice.SetSelection(settings['direction_choice'])

    # Restore advanced options
    if 'mps_reverse_rounds' in settings:
        dialog.mps_reverse_rounds.SetValue(settings['mps_reverse_rounds'])
    if 'mps_layer_swap' in settings:
        dialog.mps_layer_swap.SetValue(settings['mps_layer_swap'])
    if 'keep_input_copper' in settings:
        dialog.keep_input_copper.SetValue(settings['keep_input_copper'])
    if 'smoothing' in settings:
        dialog.smoothing.SetValue(settings['smoothing'])
    if 'force_reroute' in settings:
        dialog.force_reroute.SetValue(settings['force_reroute'])
    if 'mps_segment_intersection' in settings:
        dialog.mps_segment_intersection.SetValue(settings['mps_segment_intersection'])
    if 'no_crossing_layer_check' in settings:
        dialog.no_crossing_layer_check.SetValue(settings['no_crossing_layer_check'])
    if 'can_swap_to_top' in settings:
        dialog.can_swap_to_top.SetValue(settings['can_swap_to_top'])
    if 'crossing_penalty' in settings:
        dialog.crossing_penalty.SetValue(settings['crossing_penalty'])

    # Restore bus routing options
    if 'bus_enabled' in settings:
        dialog.bus_enabled.SetValue(settings['bus_enabled'])
    if 'bus_detection_radius' in settings:
        dialog.bus_detection_radius.SetValue(settings['bus_detection_radius'])
    if 'bus_attraction_radius' in settings:
        dialog.bus_attraction_radius.SetValue(settings['bus_attraction_radius'])
    if 'bus_attraction_bonus' in settings:
        dialog.bus_attraction_bonus.SetValue(settings['bus_attraction_bonus'])
    if 'bus_min_nets' in settings:
        dialog.bus_min_nets.SetValue(settings['bus_min_nets'])

    # Restore guide corridor (issue #7)
    if 'guide_corridor_check' in settings:
        dialog.guide_corridor_check.SetValue(settings['guide_corridor_check'])
    if 'guide_corridor_layer' in settings:
        dialog.guide_corridor_layer_ctrl.SetValue(settings['guide_corridor_layer'])
    if 'guide_corridor_spacing' in settings:
        dialog.guide_corridor_spacing_ctrl.SetValue(str(settings['guide_corridor_spacing']))
    if 'keepout_check' in settings:
        dialog.keepout_check.SetValue(settings['keepout_check'])
    if 'keepout_layer' in settings:
        dialog.keepout_layer_ctrl.SetValue(settings['keepout_layer'])
    if 'clear_guide_layer_check' in settings:
        dialog.clear_guide_layer_check.SetValue(settings['clear_guide_layer_check'])
    if 'clear_keepout_layer_check' in settings:
        dialog.clear_keepout_layer_check.SetValue(settings['clear_keepout_layer_check'])

    if 'length_match_groups' in settings:
        dialog.length_match_groups_ctrl.SetValue(settings['length_match_groups'])
    if 'length_match_tolerance' in settings:
        dialog.length_match_tolerance.SetValue(settings['length_match_tolerance'])
    if 'meander_amplitude' in settings:
        dialog.meander_amplitude.SetValue(settings['meander_amplitude'])
    if 'meander_spacing' in settings:
        dialog.meander_spacing.SetValue(settings['meander_spacing'])
    if 'time_matching_check' in settings:
        dialog.time_matching_check.SetValue(settings['time_matching_check'])
    if 'time_match_tolerance' in settings:
        dialog.time_match_tolerance.SetValue(settings['time_match_tolerance'])
    if 'debug_lines_check' in settings:
        dialog.debug_lines_check.SetValue(settings['debug_lines_check'])
    if 'verbose_check' in settings:
        dialog.verbose_check.SetValue(settings['verbose_check'])
    if 'skip_routing_check' in settings:
        dialog.skip_routing_check.SetValue(settings['skip_routing_check'])
    if 'debug_memory_check' in settings:
        dialog.debug_memory_check.SetValue(settings['debug_memory_check'])
    if 'stats_check' in settings:
        dialog.stats_check.SetValue(settings['stats_check'])
    if 'make_movie_check' in settings:
        # Restoring the box does NOT arm the recorder: its baseline is taken
        # when the user ticks it (or when a plan run starts), so a restored
        # tick still records from the board as it stands at that moment.
        dialog.make_movie_check.SetValue(settings['make_movie_check'])

    # Restore swappable nets options
    if 'update_schematic_check' in settings:
        dialog.update_schematic_check.SetValue(settings['update_schematic_check'])
        dialog.schematic_dir_ctrl.Enable(settings['update_schematic_check'])
        dialog.browse_schematic_btn.Enable(settings['update_schematic_check'])
    if 'schematic_dir' in settings:
        dialog.schematic_dir_ctrl.SetValue(settings['schematic_dir'])

    # Restore hide checkboxes
    if 'net_panel_hide' in settings and dialog.net_panel.hide_check:
        dialog.net_panel.hide_check.SetValue(settings['net_panel_hide'])
    if 'net_panel_hide_diff' in settings and dialog.net_panel.hide_diff_check:
        dialog.net_panel.hide_diff_check.SetValue(settings['net_panel_hide_diff'])
    # Restore net class separation checkbox and trigger its handler
    if 'net_panel_separate_netclass' in settings:
        dialog.net_panel.separate_netclass_check.SetValue(settings['net_panel_separate_netclass'])
        if settings['net_panel_separate_netclass']:
            dialog.net_panel._on_separate_netclass_changed(None)
    if 'swappable_hide' in settings and dialog.swappable_net_panel.hide_check:
        dialog.swappable_net_panel.hide_check.SetValue(settings['swappable_hide'])
    if 'diff_panel_hide' in settings and dialog.differential_tab.pair_panel.hide_check:
        dialog.differential_tab.pair_panel.hide_check.SetValue(settings['diff_panel_hide'])
    if 'fanout_hide' in settings and dialog.fanout_tab.net_panel.hide_check:
        dialog.fanout_tab.net_panel.hide_check.SetValue(settings['fanout_hide'])

    # Restore filters
    if 'net_panel_filter' in settings:
        dialog.net_panel.filter_ctrl.SetValue(settings['net_panel_filter'])
    if 'swappable_filter' in settings:
        dialog.swappable_net_panel.filter_ctrl.SetValue(settings['swappable_filter'])
    if 'diff_panel_filter' in settings:
        dialog.differential_tab.pair_panel.filter_ctrl.SetValue(settings['diff_panel_filter'])
    if 'fanout_filter' in settings:
        dialog.fanout_tab.net_panel.filter_ctrl.SetValue(settings['fanout_filter'])

    # Restore component dropdowns and their filter values
    if 'net_panel_component' in settings and dialog.net_panel.component_dropdown:
        idx = settings['net_panel_component']
        if idx < dialog.net_panel.component_dropdown.GetCount():
            dialog.net_panel.component_dropdown.SetSelection(idx)
            if idx > 0:
                text = dialog.net_panel.component_dropdown.GetString(idx)
                dialog.net_panel._component_filter_value = text.split(' (')[0]
    if 'swappable_component' in settings and dialog.swappable_net_panel.component_dropdown:
        idx = settings['swappable_component']
        if idx < dialog.swappable_net_panel.component_dropdown.GetCount():
            dialog.swappable_net_panel.component_dropdown.SetSelection(idx)
            if idx > 0:
                text = dialog.swappable_net_panel.component_dropdown.GetString(idx)
                dialog.swappable_net_panel._component_filter_value = text.split(' (')[0]
    if 'diff_panel_component' in settings and dialog.differential_tab.pair_panel.component_dropdown:
        idx = settings['diff_panel_component']
        if idx < dialog.differential_tab.pair_panel.component_dropdown.GetCount():
            dialog.differential_tab.pair_panel.component_dropdown.SetSelection(idx)
            if idx > 0:
                text = dialog.differential_tab.pair_panel.component_dropdown.GetString(idx)
                dialog.differential_tab.pair_panel._component_filter_value = text
    if 'fanout_component' in settings and dialog.fanout_tab.net_panel.component_dropdown:
        idx = settings['fanout_component']
        if idx < dialog.fanout_tab.net_panel.component_dropdown.GetCount():
            dialog.fanout_tab.net_panel.component_dropdown.SetSelection(idx)
            if idx > 0:
                text = dialog.fanout_tab.net_panel.component_dropdown.GetString(idx)
                dialog.fanout_tab.net_panel._component_filter_value = text.split(' (')[0]

    # Restore differential tab parameters
    if 'diff_pair_width' in settings:
        dialog.differential_tab.diff_pair_width.SetValue(settings['diff_pair_width'])
    if 'diff_pair_gap' in settings:
        dialog.differential_tab.diff_pair_gap.SetValue(settings['diff_pair_gap'])
    if 'diff_impedance' in settings:
        dialog.differential_tab.diff_impedance.SetValue(settings['diff_impedance'])
    # Restore the diff-pair width/gap override checkboxes and sync spinctrl enable
    if 'diff_pair_width_override' in settings:
        dialog.differential_tab.diff_pair_width_check.SetValue(settings['diff_pair_width_override'])
        dialog.differential_tab.diff_pair_width.Enable(settings['diff_pair_width_override'])
    if 'diff_pair_gap_override' in settings:
        dialog.differential_tab.diff_pair_gap_check.SetValue(settings['diff_pair_gap_override'])
        dialog.differential_tab.diff_pair_gap.Enable(settings['diff_pair_gap_override'])
    if 'min_turning_radius' in settings:
        dialog.differential_tab.min_turning_radius.SetValue(settings['min_turning_radius'])
    if 'max_setback_angle' in settings:
        dialog.differential_tab.max_setback_angle.SetValue(settings['max_setback_angle'])
    if 'max_turn_angle' in settings:
        dialog.differential_tab.max_turn_angle.SetValue(settings['max_turn_angle'])
    if 'chamfer_extra' in settings:
        dialog.differential_tab.chamfer_extra.SetValue(settings['chamfer_extra'])
    if 'centerline_setback' in settings:
        dialog.differential_tab.centerline_setback.SetValue(settings['centerline_setback'])
    if 'polarity_swap_nets_text' in settings:
        dialog.differential_tab.polarity_swap_nets_text.SetValue(
            settings['polarity_swap_nets_text'])
    elif 'fix_polarity_check' in settings:
        # Migrate the pre-#279 boolean: True -> allow all pairs, False -> none.
        dialog.differential_tab.polarity_swap_nets_text.SetValue(
            '*' if settings['fix_polarity_check'] else '')
    if 'gnd_via_check' in settings:
        dialog.differential_tab.gnd_via_check.SetValue(settings['gnd_via_check'])
    if 'intra_match_check' in settings:
        dialog.differential_tab.intra_match_check.SetValue(settings['intra_match_check'])
    if 'ac_couple_check' in settings:
        dialog.differential_tab.ac_couple_check.SetValue(settings['ac_couple_check'])
    if 'diff_hide_short' in settings:
        dialog.differential_tab.hide_short_check.SetValue(settings['diff_hide_short'])
        dialog.differential_tab.pair_panel.set_hide_short(settings['diff_hide_short'])

    # Restore fanout tab settings
    if 'fanout_type' in settings:
        dialog.fanout_tab.fanout_type.SetSelection(settings['fanout_type'])
        # Trigger type change to show/hide appropriate options
        dialog.fanout_tab._on_type_changed(None)
    if 'fanout_bga_exit_margin' in settings:
        dialog.fanout_tab.bga_options.exit_margin.SetValue(settings['fanout_bga_exit_margin'])
    if 'fanout_bga_differential' in settings:
        dialog.fanout_tab.bga_options.differential_check.SetValue(settings['fanout_bga_differential'])
    if 'fanout_bga_escape_direction' in settings:
        dialog.fanout_tab.bga_options.escape_direction.SetSelection(settings['fanout_bga_escape_direction'])
    if 'fanout_bga_force_escape' in settings:
        dialog.fanout_tab.bga_options.force_escape.SetValue(settings['fanout_bga_force_escape'])
    if 'fanout_bga_rebalance' in settings:
        dialog.fanout_tab.bga_options.rebalance_escape.SetValue(settings['fanout_bga_rebalance'])
    if 'fanout_bga_check_previous' in settings:
        dialog.fanout_tab.bga_options.check_previous.SetValue(settings['fanout_bga_check_previous'])
    if 'fanout_bga_no_inner_top' in settings:
        dialog.fanout_tab.bga_options.no_inner_top.SetValue(settings['fanout_bga_no_inner_top'])
    if 'fanout_bga_escape_method' in settings:
        dialog.fanout_tab.bga_options.set_escape_method(settings['fanout_bga_escape_method'])
    if 'fanout_bga_plane_drop' in settings:
        dialog.fanout_tab.bga_options.plane_drop.SetValue(bool(settings['fanout_bga_plane_drop']))
    if 'fanout_bga_plane_net_layers' in settings:
        dialog.fanout_tab.bga_options.plane_net_layers_ctrl.SetValue(
            str(settings['fanout_bga_plane_net_layers']))
    elif 'fanout_bga_underpad' in settings:
        # Migrate the pre-dropdown checkbox (#288): checked meant under-pad,
        # unchecked meant the default engine (now 'auto').
        dialog.fanout_tab.bga_options.set_escape_method(
            'underpad' if settings['fanout_bga_underpad'] else 'auto')
    if 'fanout_bga_optimize_caps' in settings:
        dialog.fanout_tab.bga_options.optimize_caps.SetValue(settings['fanout_bga_optimize_caps'])
    if 'fanout_bga_cap_capture_radius' in settings:
        dialog.fanout_tab.bga_options.cap_capture_radius.SetValue(settings['fanout_bga_cap_capture_radius'])
    if 'fanout_bga_cap_board_edge_clearance' in settings:
        dialog.fanout_tab.bga_options.cap_board_edge_clearance.SetValue(
            settings['fanout_bga_cap_board_edge_clearance'])
    if 'fanout_bga_cap_near_margin' in settings:
        dialog.fanout_tab.bga_options.cap_near_margin.SetValue(settings['fanout_bga_cap_near_margin'])
    if 'fanout_bga_cap_step' in settings:
        dialog.fanout_tab.bga_options.cap_step.SetValue(settings['fanout_bga_cap_step'])
    if 'fanout_bga_cap_max_displacement' in settings:
        dialog.fanout_tab.bga_options.cap_max_displacement.SetValue(settings['fanout_bga_cap_max_displacement'])
    if 'fanout_bga_cap_max_displacement_cap' in settings:
        dialog.fanout_tab.bga_options.cap_max_displacement_cap.SetValue(settings['fanout_bga_cap_max_displacement_cap'])
    if 'fanout_bga_cap_displacement_growth' in settings:
        dialog.fanout_tab.bga_options.cap_displacement_growth.SetValue(settings['fanout_bga_cap_displacement_growth'])
    if 'fanout_bga_cap_max_passes' in settings:
        dialog.fanout_tab.bga_options.cap_max_passes.SetValue(settings['fanout_bga_cap_max_passes'])
    if 'fanout_bga_cap_prefix' in settings:
        dialog.fanout_tab.bga_options.cap_prefix.SetValue(settings['fanout_bga_cap_prefix'])
    if 'fanout_bga_cap_default_via_size' in settings:
        dialog.fanout_tab.bga_options.cap_default_via_size.SetValue(
            settings['fanout_bga_cap_default_via_size'])
    if 'fanout_bga_cap_allow_rotation' in settings:
        dialog.fanout_tab.bga_options.cap_allow_rotation.SetValue(settings['fanout_bga_cap_allow_rotation'])
    if 'fanout_qfn_extension' in settings:
        dialog.fanout_tab.qfn_options.extension.SetValue(settings['fanout_qfn_extension'])
    if 'fanout_qfn_track_width' in settings:
        dialog.fanout_tab.qfn_options.qfn_track_width.SetValue(settings['fanout_qfn_track_width'])
    if 'fanout_qfn_clearance' in settings:
        dialog.fanout_tab.qfn_options.qfn_clearance.SetValue(settings['fanout_qfn_clearance'])
    if 'fanout_qfn_underpad' in settings:
        dialog.fanout_tab.qfn_options.underpad_escape.SetValue(settings['fanout_qfn_underpad'])
    if 'fanout_qfn_allow_via_in_pad' in settings:
        dialog.fanout_tab.qfn_options.allow_via_in_pad.SetValue(settings['fanout_qfn_allow_via_in_pad'])

    # Restore planes tab settings
    # (No mode restore since #562: the tab is pour-creation only, so a
    # legacy 'planes_mode' key is simply ignored.)
    if 'planes_assignments' in settings:
        dialog.planes_tab.assignment_panel.set_assignments(settings['planes_assignments'])
    if 'planes_hide' in settings and dialog.planes_tab.net_panel.hide_check:
        dialog.planes_tab.net_panel.hide_check.SetValue(settings['planes_hide'])
    if 'planes_filter' in settings:
        dialog.planes_tab.net_panel.filter_ctrl.SetValue(settings['planes_filter'])
    if 'planes_component' in settings and dialog.planes_tab.net_panel.component_dropdown:
        idx = settings['planes_component']
        if idx < dialog.planes_tab.net_panel.component_dropdown.GetCount():
            dialog.planes_tab.net_panel.component_dropdown.SetSelection(idx)
            if idx > 0:
                text = dialog.planes_tab.net_panel.component_dropdown.GetString(idx)
                dialog.planes_tab.net_panel._component_filter_value = text.split(' (')[0]
    # Create mode options
    if 'planes_zone_clearance' in settings:
        dialog.planes_tab.create_options.zone_clearance.SetValue(settings['planes_zone_clearance'])
    if 'planes_thermal_relief' in settings:
        dialog.planes_tab.create_options.thermal_relief.SetValue(settings['planes_thermal_relief'])
    if 'planes_thermal_vias' in settings:
        dialog.planes_tab.create_options.thermal_vias.SetValue(settings['planes_thermal_vias'])
    if 'planes_add_gnd_vias' in settings:
        dialog.planes_tab.create_options.add_gnd_vias_check.SetValue(settings['planes_add_gnd_vias'])
    if 'planes_gnd_via_distance' in settings:
        dialog.planes_tab.create_options.gnd_via_distance.SetValue(settings['planes_gnd_via_distance'])
    if 'planes_stitch_vias' in settings:
        dialog.planes_tab.create_options.stitch_vias.SetValue(settings['planes_stitch_vias'])
    if 'planes_stitch_pitch' in settings:
        dialog.planes_tab.create_options.stitch_pitch.SetValue(settings['planes_stitch_pitch'])
    if 'planes_stitch_edge_fence' in settings:
        dialog.planes_tab.create_options.stitch_edge_fence.SetValue(settings['planes_stitch_edge_fence'])
    if 'planes_stitch_fence_pitch' in settings:
        dialog.planes_tab.create_options.stitch_fence_pitch.SetValue(settings['planes_stitch_fence_pitch'])
    if 'planes_stitch_inset' in settings:
        dialog.planes_tab.create_options.stitch_inset.SetValue(settings['planes_stitch_inset'])
    if 'planes_stitch_max_freq' in settings:
        dialog.planes_tab.create_options.stitch_max_freq.SetValue(settings['planes_stitch_max_freq'])
    if 'planes_gnd_via_net' in settings:
        dialog.planes_tab.create_options.gnd_via_net.SetValue(settings['planes_gnd_via_net'])
    # Repair mode options

    # Restore AI tab backend/model/effort (issue #40; #503). Per-backend
    # model/effort entries first, then the backend selection LAST so its
    # refresh loads the right entries into the combos; an unknown saved
    # backend id reverts to the default (Claude Code).
    if 'claude_model' in settings:
        dialog.ai_tab.set_model_value(settings['claude_model'], backend_id='claude')
    if 'claude_effort' in settings:
        dialog.ai_tab.set_effort_value(settings['claude_effort'], backend_id='claude')
    if 'opencode_model' in settings:
        dialog.ai_tab.set_model_value(settings['opencode_model'], backend_id='opencode')
    if 'opencode_effort' in settings:
        dialog.ai_tab.set_effort_value(settings['opencode_effort'], backend_id='opencode')
    if 'ai_backend' in settings:
        dialog.ai_tab.set_backend_value(settings['ai_backend'])
    # 'claude_plan' is the pre-rename key for the same plan state (settings
    # files saved by older versions) - honor it when 'ai_plan' is absent.
    plan_state = settings.get('ai_plan', settings.get('claude_plan'))
    if plan_state is not None:
        dialog.ai_tab.restore_plan_state(plan_state)

    # Placement sub-tab (issue #481). Restoring the last workdir re-surfaces
    # a previous session's movie/report/preview buttons when the artifacts
    # still exist on disk.
    if 'ai_active_subtab' in settings and hasattr(dialog, 'ai_notebook'):
        idx = settings['ai_active_subtab']
        if isinstance(idx, int) and 0 <= idx < dialog.ai_notebook.GetPageCount():
            dialog.ai_notebook.SetSelection(idx)
    if 'placement_backend' in settings:
        # Unsupported/unknown saved ids revert to Claude Code inside the tab.
        dialog.placement_tab.set_backend_value(settings['placement_backend'])
    if 'placement_model' in settings:
        dialog.placement_tab.set_model_value(settings['placement_model'])
    if 'placement_effort' in settings:
        dialog.placement_tab.set_effort_value(settings['placement_effort'])
    if 'placement_extra_instructions' in settings:
        dialog.placement_tab.extra_instructions.SetValue(
            settings['placement_extra_instructions'])
    if 'placement_labels_options' in settings and isinstance(
            settings['placement_labels_options'], dict):
        dialog.placement_tab.labels_options.set_config(
            settings['placement_labels_options'])
    if 'placement_last_workdir' in settings and settings['placement_last_workdir']:
        dialog.placement_tab.restore_last_workdir(
            settings['placement_last_workdir'])

    # Restore net selections LAST - after all filters/checkboxes are set
    # This prevents the selections from being cleared by filter change events
    if 'net_panel_checked' in settings:
        dialog.net_panel._checked_nets = set(settings['net_panel_checked'])
    if 'swappable_net_panel_checked' in settings:
        dialog.swappable_net_panel._checked_nets = set(settings['swappable_net_panel_checked'])
    if 'fanout_net_panel_checked' in settings:
        dialog.fanout_tab.net_panel._checked_nets = set(settings['fanout_net_panel_checked'])
    if 'planes_net_panel_checked' in settings:
        dialog.planes_tab.net_panel._checked_nets = set(settings['planes_net_panel_checked'])
    if 'diff_pairs_checked' in settings:
        dialog.differential_tab.pair_panel._checked_pairs = set(settings['diff_pairs_checked'])

    # Restore log content
    if 'log_content' in settings and settings['log_content']:
        dialog.log_text.SetValue(settings['log_content'])

    # Restore window transparency
    if 'window_transparency' in settings:
        transparency = settings['window_transparency']
        dialog.SetTransparent(transparency)
        if hasattr(dialog, 'about_tab'):
            dialog.about_tab.transparency_slider.SetValue(transparency)

    # Resume connectivity checks (actual check happens in refresh_from_board)
    dialog.net_panel.resume_check()
    dialog.swappable_net_panel.resume_check()
    dialog.differential_tab.pair_panel.resume_check()
    dialog.fanout_tab.net_panel.resume_check()
    dialog.planes_tab.net_panel.resume_check()
