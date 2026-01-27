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

        # Layer selections
        'layers': [layer for layer, cb in dialog.layer_checks.items() if cb.GetValue()],

        # Basic options
        'enable_layer_switch': dialog.enable_layer_switch.GetValue(),
        'move_text_check': dialog.move_text_check.GetValue(),
        'add_teardrops_check': dialog.add_teardrops_check.GetValue(),
        'power_nets': dialog.power_nets_ctrl.GetValue(),
        'power_widths': dialog.power_widths_ctrl.GetValue(),
        'no_bga_zones': dialog.no_bga_zones_ctrl.GetValue(),
        'layer_costs': dialog.layer_costs_ctrl.GetValue(),

        # Advanced parameters
        'impedance_check': dialog.impedance_check.GetValue(),
        'impedance_value': dialog.impedance_value.GetValue(),
        'max_iterations': dialog.max_iterations.GetValue(),
        'max_probe_iterations': dialog.max_probe_iterations.GetValue(),
        'heuristic_weight': dialog.heuristic_weight.GetValue(),
        'turn_cost': dialog.turn_cost.GetValue(),
        'ordering_strategy': dialog.ordering_strategy.GetSelection(),
        'bga_proximity_radius': dialog.bga_proximity_radius.GetValue(),
        'bga_proximity_cost': dialog.bga_proximity_cost.GetValue(),
        'stub_proximity_radius': dialog.stub_proximity_radius.GetValue(),
        'stub_proximity_cost': dialog.stub_proximity_cost.GetValue(),
        'via_proximity_cost': dialog.via_proximity_cost.GetValue(),
        'track_proximity_distance': dialog.track_proximity_distance.GetValue(),
        'track_proximity_cost': dialog.track_proximity_cost.GetValue(),
        'vertical_attraction_radius': dialog.vertical_attraction_radius.GetValue(),
        'vertical_attraction_cost': dialog.vertical_attraction_cost.GetValue(),
        'routing_clearance_margin': dialog.routing_clearance_margin.GetValue(),
        'hole_to_hole_clearance': dialog.hole_to_hole_clearance.GetValue(),
        'edge_clearance_check': dialog.edge_clearance_check.GetValue(),
        'board_edge_clearance': dialog.board_edge_clearance.GetValue(),
        'direction_choice': dialog.direction_choice.GetSelection(),

        # Advanced options
        'mps_reverse_rounds': dialog.mps_reverse_rounds.GetValue(),
        'mps_layer_swap': dialog.mps_layer_swap.GetValue(),
        'mps_segment_intersection': dialog.mps_segment_intersection.GetValue(),
        'no_crossing_layer_check': dialog.no_crossing_layer_check.GetValue(),
        'can_swap_to_top': dialog.can_swap_to_top.GetValue(),
        'crossing_penalty': dialog.crossing_penalty.GetValue(),
        'length_match_groups': dialog.length_match_groups_ctrl.GetValue(),
        'length_match_tolerance': dialog.length_match_tolerance.GetValue(),
        'meander_amplitude': dialog.meander_amplitude.GetValue(),
        'time_matching_check': dialog.time_matching_check.GetValue(),
        'time_match_tolerance': dialog.time_match_tolerance.GetValue(),
        'debug_lines_check': dialog.debug_lines_check.GetValue(),
        'verbose_check': dialog.verbose_check.GetValue(),
        'skip_routing_check': dialog.skip_routing_check.GetValue(),
        'debug_memory_check': dialog.debug_memory_check.GetValue(),

        # Swappable nets options
        'update_schematic_check': dialog.update_schematic_check.GetValue(),
        'schematic_dir': dialog.schematic_dir_ctrl.GetValue(),

        # Hide checkboxes
        'net_panel_hide': dialog.net_panel.hide_check.GetValue() if dialog.net_panel.hide_check else False,
        'net_panel_hide_diff': dialog.net_panel.hide_diff_check.GetValue() if dialog.net_panel.hide_diff_check else False,
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
        'diff_pair_gap': dialog.differential_tab.diff_pair_gap.GetValue(),
        'min_turning_radius': dialog.differential_tab.min_turning_radius.GetValue(),
        'max_setback_angle': dialog.differential_tab.max_setback_angle.GetValue(),
        'max_turn_angle': dialog.differential_tab.max_turn_angle.GetValue(),
        'chamfer_extra': dialog.differential_tab.chamfer_extra.GetValue(),
        'fix_polarity_check': dialog.differential_tab.fix_polarity_check.GetValue(),
        'gnd_via_check': dialog.differential_tab.gnd_via_check.GetValue(),
        'intra_match_check': dialog.differential_tab.intra_match_check.GetValue(),

        # Fanout tab settings
        'fanout_type': dialog.fanout_tab.fanout_type.GetSelection(),
        'fanout_bga_exit_margin': dialog.fanout_tab.bga_options.exit_margin.GetValue(),
        'fanout_bga_differential': dialog.fanout_tab.bga_options.differential_check.GetValue(),
        'fanout_bga_diff_gap': dialog.fanout_tab.bga_options.diff_pair_gap.GetValue(),
        'fanout_bga_escape_direction': dialog.fanout_tab.bga_options.escape_direction.GetSelection(),
        'fanout_bga_force_escape': dialog.fanout_tab.bga_options.force_escape.GetValue(),
        'fanout_bga_rebalance': dialog.fanout_tab.bga_options.rebalance_escape.GetValue(),
        'fanout_bga_check_previous': dialog.fanout_tab.bga_options.check_previous.GetValue(),
        'fanout_bga_no_inner_top': dialog.fanout_tab.bga_options.no_inner_top.GetValue(),
        'fanout_qfn_extension': dialog.fanout_tab.qfn_options.extension.GetValue(),

        # Planes tab settings
        'planes_net_panel_checked': list(dialog.planes_tab.net_panel.get_selected_nets()),
        'planes_mode': dialog.planes_tab.mode_selector.GetSelection(),
        'planes_assignments': dialog.planes_tab.assignment_panel.get_assignments(),
        'planes_hide': dialog.planes_tab.net_panel.hide_check.GetValue() if dialog.planes_tab.net_panel.hide_check else False,
        'planes_filter': dialog.planes_tab.net_panel.filter_ctrl.GetValue(),
        'planes_component': dialog.planes_tab.net_panel.component_dropdown.GetSelection() if dialog.planes_tab.net_panel.component_dropdown else 0,
        # Create mode options
        'planes_zone_clearance': dialog.planes_tab.create_options.zone_clearance.GetValue(),
        'planes_edge_clearance': dialog.planes_tab.create_options.edge_clearance.GetValue(),
        'planes_max_search_radius': dialog.planes_tab.create_options.max_search_radius.GetValue(),
        'planes_rip_blocker_check': dialog.planes_tab.create_options.rip_blocker_check.GetValue(),
        'planes_reroute_ripped_check': dialog.planes_tab.create_options.reroute_ripped_check.GetValue(),
        # Repair mode options
        'planes_repair_max_track_width': dialog.planes_tab.repair_options.max_track_width.GetValue(),
        'planes_repair_min_track_width': dialog.planes_tab.repair_options.min_track_width.GetValue(),
        'planes_repair_analysis_grid': dialog.planes_tab.repair_options.analysis_grid.GetValue(),

        # Log content
        'log_content': dialog.log_text.GetValue(),
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
    if 'power_nets' in settings:
        dialog.power_nets_ctrl.SetValue(settings['power_nets'])
    if 'power_widths' in settings:
        dialog.power_widths_ctrl.SetValue(settings['power_widths'])
    if 'no_bga_zones' in settings:
        dialog.no_bga_zones_ctrl.SetValue(settings['no_bga_zones'])
    if 'layer_costs' in settings:
        dialog.layer_costs_ctrl.SetValue(settings['layer_costs'])

    # Restore advanced parameters
    if 'impedance_check' in settings:
        dialog.impedance_check.SetValue(settings['impedance_check'])
    if 'impedance_value' in settings:
        dialog.impedance_value.SetValue(settings['impedance_value'])
    if 'max_iterations' in settings:
        dialog.max_iterations.SetValue(settings['max_iterations'])
    if 'max_probe_iterations' in settings:
        dialog.max_probe_iterations.SetValue(settings['max_probe_iterations'])
    if 'heuristic_weight' in settings:
        dialog.heuristic_weight.SetValue(settings['heuristic_weight'])
    if 'turn_cost' in settings:
        dialog.turn_cost.SetValue(settings['turn_cost'])
    if 'ordering_strategy' in settings:
        dialog.ordering_strategy.SetSelection(settings['ordering_strategy'])
    if 'bga_proximity_radius' in settings:
        dialog.bga_proximity_radius.SetValue(settings['bga_proximity_radius'])
    if 'bga_proximity_cost' in settings:
        dialog.bga_proximity_cost.SetValue(settings['bga_proximity_cost'])
    if 'stub_proximity_radius' in settings:
        dialog.stub_proximity_radius.SetValue(settings['stub_proximity_radius'])
    if 'stub_proximity_cost' in settings:
        dialog.stub_proximity_cost.SetValue(settings['stub_proximity_cost'])
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
    if 'routing_clearance_margin' in settings:
        dialog.routing_clearance_margin.SetValue(settings['routing_clearance_margin'])
    if 'hole_to_hole_clearance' in settings:
        dialog.hole_to_hole_clearance.SetValue(settings['hole_to_hole_clearance'])
    if 'edge_clearance_check' in settings:
        dialog.edge_clearance_check.SetValue(settings['edge_clearance_check'])
        dialog.board_edge_clearance.Enable(settings['edge_clearance_check'])
    if 'board_edge_clearance' in settings:
        dialog.board_edge_clearance.SetValue(settings['board_edge_clearance'])
    if 'direction_choice' in settings:
        dialog.direction_choice.SetSelection(settings['direction_choice'])

    # Restore advanced options
    if 'mps_reverse_rounds' in settings:
        dialog.mps_reverse_rounds.SetValue(settings['mps_reverse_rounds'])
    if 'mps_layer_swap' in settings:
        dialog.mps_layer_swap.SetValue(settings['mps_layer_swap'])
    if 'mps_segment_intersection' in settings:
        dialog.mps_segment_intersection.SetValue(settings['mps_segment_intersection'])
    if 'no_crossing_layer_check' in settings:
        dialog.no_crossing_layer_check.SetValue(settings['no_crossing_layer_check'])
    if 'can_swap_to_top' in settings:
        dialog.can_swap_to_top.SetValue(settings['can_swap_to_top'])
    if 'crossing_penalty' in settings:
        dialog.crossing_penalty.SetValue(settings['crossing_penalty'])
    if 'length_match_groups' in settings:
        dialog.length_match_groups_ctrl.SetValue(settings['length_match_groups'])
    if 'length_match_tolerance' in settings:
        dialog.length_match_tolerance.SetValue(settings['length_match_tolerance'])
    if 'meander_amplitude' in settings:
        dialog.meander_amplitude.SetValue(settings['meander_amplitude'])
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
    if 'diff_pair_gap' in settings:
        dialog.differential_tab.diff_pair_gap.SetValue(settings['diff_pair_gap'])
    if 'min_turning_radius' in settings:
        dialog.differential_tab.min_turning_radius.SetValue(settings['min_turning_radius'])
    if 'max_setback_angle' in settings:
        dialog.differential_tab.max_setback_angle.SetValue(settings['max_setback_angle'])
    if 'max_turn_angle' in settings:
        dialog.differential_tab.max_turn_angle.SetValue(settings['max_turn_angle'])
    if 'chamfer_extra' in settings:
        dialog.differential_tab.chamfer_extra.SetValue(settings['chamfer_extra'])
    if 'fix_polarity_check' in settings:
        dialog.differential_tab.fix_polarity_check.SetValue(settings['fix_polarity_check'])
    if 'gnd_via_check' in settings:
        dialog.differential_tab.gnd_via_check.SetValue(settings['gnd_via_check'])
    if 'intra_match_check' in settings:
        dialog.differential_tab.intra_match_check.SetValue(settings['intra_match_check'])

    # Restore fanout tab settings
    if 'fanout_type' in settings:
        dialog.fanout_tab.fanout_type.SetSelection(settings['fanout_type'])
        # Trigger type change to show/hide appropriate options
        dialog.fanout_tab._on_type_changed(None)
    if 'fanout_bga_exit_margin' in settings:
        dialog.fanout_tab.bga_options.exit_margin.SetValue(settings['fanout_bga_exit_margin'])
    if 'fanout_bga_differential' in settings:
        dialog.fanout_tab.bga_options.differential_check.SetValue(settings['fanout_bga_differential'])
        # Trigger differential change to show/hide gap control
        dialog.fanout_tab.bga_options._on_differential_changed(None)
    if 'fanout_bga_diff_gap' in settings:
        dialog.fanout_tab.bga_options.diff_pair_gap.SetValue(settings['fanout_bga_diff_gap'])
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
    if 'fanout_qfn_extension' in settings:
        dialog.fanout_tab.qfn_options.extension.SetValue(settings['fanout_qfn_extension'])

    # Restore planes tab settings
    if 'planes_mode' in settings:
        dialog.planes_tab.mode_selector.SetSelection(settings['planes_mode'])
        # Trigger mode change to show/hide appropriate options
        dialog.planes_tab._on_mode_changed(None)
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
    if 'planes_edge_clearance' in settings:
        dialog.planes_tab.create_options.edge_clearance.SetValue(settings['planes_edge_clearance'])
    if 'planes_max_search_radius' in settings:
        dialog.planes_tab.create_options.max_search_radius.SetValue(settings['planes_max_search_radius'])
    if 'planes_rip_blocker_check' in settings:
        dialog.planes_tab.create_options.rip_blocker_check.SetValue(settings['planes_rip_blocker_check'])
    if 'planes_reroute_ripped_check' in settings:
        dialog.planes_tab.create_options.reroute_ripped_check.SetValue(settings['planes_reroute_ripped_check'])
    # Repair mode options
    if 'planes_repair_max_track_width' in settings:
        dialog.planes_tab.repair_options.max_track_width.SetValue(settings['planes_repair_max_track_width'])
    if 'planes_repair_min_track_width' in settings:
        dialog.planes_tab.repair_options.min_track_width.SetValue(settings['planes_repair_min_track_width'])
    if 'planes_repair_analysis_grid' in settings:
        dialog.planes_tab.repair_options.analysis_grid.SetValue(settings['planes_repair_analysis_grid'])

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

    # Resume connectivity checks (actual check happens in refresh_from_board)
    dialog.net_panel.resume_check()
    dialog.swappable_net_panel.resume_check()
    dialog.differential_tab.pair_panel.resume_check()
    dialog.fanout_tab.net_panel.resume_check()
    dialog.planes_tab.net_panel.resume_check()
