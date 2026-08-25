import math
import random
import sys
import tempfile
import types
import zipfile
import json
from pathlib import Path

from kicad_track_gloss.engine import (find_track_terminal_vertices,
                                      generate_candidate_plans,
                                      generate_converged_plan,
                                      smooth_selected_chains, summarize_plan)
from kicad_track_gloss.engine.model import (AddedSegment, BoardModel,
                                            CircleObstacle, GlossResult,
                                            PadRegion, Segment, segment_key)
from kicad_track_gloss.kicad.selection import meander_keys as _meander_keys
from kicad_track_gloss.engine.pads import segment_hits_pad
from kicad_track_gloss.kicad.rules import via_track_hole_clearance
from kicad_track_gloss.engine.planner import _apply_to_model


def test_custom_via_track_hole_clearance_uses_matching_active_rule(tmp_path):
    board_path = tmp_path / "board.kicad_pcb"
    board_path.write_text("(kicad_pcb)", encoding="utf-8")
    board_path.with_suffix(".kicad_dru").write_text(
        """
# (rule "disabled" (constraint hole_clearance (min 9mm))
#   (condition "A.Type == 'via' && B.Type == 'track'"))
(rule "via-track"
  (constraint hole_clearance (min 0.254mm))
  (condition "A.Type == 'via' && B.Type == 'track'"))
(rule "pth-track"
  (constraint hole_clearance (min 0.33mm))
  (condition "A.Type != 'Via' && B.Type == 'track'"))
""", encoding="utf-8")

    board = types.SimpleNamespace(GetFileName=lambda: str(board_path))
    assert via_track_hole_clearance(None, board) == 0.254


def staircase(count=20, pitch=0.2, net=1):
    result = []
    x = y = 10.0
    for i in range(count):
        if i % 2:
            nxt = (x, y + pitch)
        else:
            nxt = (x + pitch, y)
        result.append(Segment(x, y, nxt[0], nxt[1], 0.15, 0, net, f"s{i}"))
        x, y = nxt
    return result


def total_length(segments):
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segments)


def test_selected_staircase_shortens():
    segs = staircase()
    model = BoardModel(segs)
    result = smooth_selected_chains(model, {segment_key(s) for s in segs})
    assert result.changed
    assert result.saved_mm > 0.5
    assert len(result.additions) < len(result.remove_keys)
    assert result.transformations
    assert all(item.mechanism == "fixed_endpoints"
               for item in result.transformations)
    assert result.search_counts["paths_evaluated"] > 0
    summary = summarize_plan(model, {segment_key(s) for s in segs}, result)
    assert sum(row["segments_saved"] for row in summary["mechanisms"]) == \
        summary["segments_saved"]


def test_converged_plan_reaches_a_reported_fixed_point():
    segs = staircase()
    result = generate_converged_plan(
        BoardModel(segs), {segment_key(segment) for segment in segs},
        min_gain=0.01, allow_equal_length_simpler=True, clearance=0.0)
    assert result.changed
    assert result.fixed_point
    assert result.convergence_passes >= 1


def test_convergence_observer_reports_monotone_states_and_fixed_point():
    segs = staircase()
    states = []
    result = generate_converged_plan(
        BoardModel(segs), {segment_key(segment) for segment in segs},
        min_gain=0.01, allow_equal_length_simpler=True, clearance=0.0,
        pass_observer=states.append)
    assert result.fixed_point
    assert states[0]["event"] == "initial"
    assert states[0]["pass_gain_mm"] == 0.0
    assert states[-1]["event"] == "fixed_point"
    changed = [state for state in states if state["event"] == "changed"]
    assert len(changed) == result.convergence_passes
    assert all(state["geometry_signature"] for state in states)


def test_nested_convergence_uses_unique_synthetic_segment_keys():
    existing = Segment(
        0.0, 0.0, 1.0, 0.0, 0.2, 0, 1,
        "__track_gloss__pass-0-0", net_name="VCC")
    plan = GlossResult(additions=[
        AddedSegment((1.0, 0.0), (2.0, 0.0), 0.2, 0, 1)])

    updated, eligible = _apply_to_model(
        BoardModel([existing]), {segment_key(existing)}, plan, 0)
    keys = [segment_key(segment) for segment in updated.segments]

    assert len(keys) == len(set(keys)) == 2
    assert "__track_gloss__pass-0-0" in keys
    assert "__track_gloss__pass-0-0-1" in keys
    assert set(keys) == eligible


def test_unselected_half_is_never_removed():
    segs = staircase()
    selected = {segment_key(s) for s in segs[:10]}
    result = smooth_selected_chains(BoardModel(segs), selected)
    assert set(result.remove_keys) <= selected
    assert not ({segment_key(s) for s in segs[10:]} & set(result.remove_keys))


def test_locked_track_splits_selection():
    segs = staircase()
    locked = segs[10]
    segs[10] = Segment(locked.start_x, locked.start_y, locked.end_x, locked.end_y,
                       locked.width, locked.layer, locked.net_id, locked.uuid, True)
    result = smooth_selected_chains(BoardModel(segs), {segment_key(s) for s in segs})
    assert "s10" not in result.remove_keys


def test_foreign_via_blocks_shortcut():
    segs = staircase(count=8, pitch=1.0)
    obstacle = CircleObstacle(11.5, 11.5, 0.3, 2, (0,), "via")
    result = smooth_selected_chains(BoardModel(segs, [obstacle]),
                                    {segment_key(s) for s in segs}, clearance=0.1)
    for new in result.additions:
        # No accepted candidate may pass through the obstacle clearance disk.
        from kicad_track_gloss.engine.geometry import point_segment_distance
        assert point_segment_distance((obstacle.x, obstacle.y), new.start, new.end) >= 0.475 - 1e-6


def test_roundrect_clearance_uses_real_shape_not_bounding_circle():
    pad = PadRegion(183.642, 109.474, 3.2, 1.6, 0.0,
                    "roundrect", 0.8, 68, (0,))
    path = ((182.3, 108.3), (187.3, 108.3))

    # The real lower edge is 0.374 mm away. The former 1.788854 mm
    # circumscribed radius incorrectly occupied this entire corridor.
    assert not segment_hits_pad(pad, *path, margin=0.373)
    assert segment_hits_pad(pad, *path, margin=0.375)


def test_shortened_existing_copper_is_not_rejected_by_coarse_pad_circle():
    # Real-board VCC regression: the conservative pad circle overlaps an
    # already routed horizontal segment.  Shortening that same segment is not
    # new copper and must not be rejected as a new clearance violation.
    segments = [
        Segment(0, 0, 10, 0, 0.25, 0, 1, "horizontal"),
        Segment(10, 0, 10, 0.2, 0.25, 0, 1, "short"),
    ]
    pad = CircleObstacle(5, 1.4, 1.9, 0, (0,), "pad")
    result = smooth_selected_chains(
        BoardModel(segments, [pad], minimum_clearance=0.25),
        {"horizontal", "short"}, min_gain=0.01, clearance=0.25,
        span_strategy="global")
    assert result.changed
    assert set(result.remove_keys) == {"horizontal", "short"}
    assert result.saved_mm > 0.1
    assert [(addition.start, addition.end) for addition in result.additions] == [
        ((0, 0), (9.8, 0)), ((9.8, 0), (10, 0.2))]


def test_equal_length_simplification_is_mode_gated():
    segs = [Segment(0, 0, 1, 0, 0.2, 0, 1, "a"),
            Segment(1, 0, 2, 0, 0.2, 0, 1, "b")]
    selected = {"a", "b"}
    shorten = smooth_selected_chains(BoardModel(segs), selected,
                                      allow_equal_length_simpler=False)
    simplify = smooth_selected_chains(BoardModel(segs), selected,
                                       allow_equal_length_simpler=True)
    assert not shorten.changed
    assert simplify.changed and len(simplify.additions) == 1


def test_no_selection_is_noop():
    assert not smooth_selected_chains(BoardModel(staircase()), set()).changed


def test_meander_direction_reversal_is_protected():
    points = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 2), (1, 2)]
    segs = [Segment(a[0], a[1], b[0], b[1], 0.2, 0, 1, f"m{i}")
            for i, (a, b) in enumerate(zip(points, points[1:]))]
    assert _meander_keys(segs) == {segment_key(s) for s in segs}
    assert not _meander_keys(staircase(8))


def test_meander_protection_does_not_spill_into_disconnected_route():
    meander_points = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 2), (1, 2)]
    meander = [Segment(a[0], a[1], b[0], b[1], 0.2, 0, 1, f"m{i}")
               for i, (a, b) in enumerate(zip(meander_points, meander_points[1:]))]
    ordinary = [Segment(10, 0, 11, 0, 0.2, 0, 1, "ordinary-a"),
                Segment(11, 0, 12, 1, 0.2, 0, 1, "ordinary-b")]
    assert _meander_keys(meander + ordinary) == {segment_key(s) for s in meander}


def test_single_direction_reversal_is_not_misclassified_as_meander():
    # Regression from dispenser_labels.kicad_pcb / Net-(U2-VCC): an ordinary
    # route has one A/B/-A reversal, but no repeated length-tuning serpent.
    points = [(213.6, 116.7), (213.6, 118.95), (213.05, 119.5),
              (213.0, 119.55), (211.575, 119.55), (211.575, 118.425),
              (210.275, 117.125), (207.35, 117.125)]
    segments = [Segment(a[0], a[1], b[0], b[1], 0.127, 0, 1, f"real{i}")
                for i, (a, b) in enumerate(zip(points, points[1:]))]
    assert not _meander_keys(segments)


def test_dense_micro_jog_tuning_is_protected_without_reversals():
    # Regression for dispenser_labels /cpu/~{csn}: 111 connected segments,
    # mostly 0.035 mm long, previously triggered ~9,700 clearance checks and
    # made KiCad look frozen although no candidate was ultimately accepted.
    segments = staircase(40, pitch=0.025)
    assert _meander_keys(segments) == {segment_key(s) for s in segments}


def test_long_ordinary_route_is_not_dense_micro_jog_tuning():
    segments = staircase(40, pitch=0.2)
    assert not _meander_keys(segments)


def _plan_signature(plan):
    return (tuple(sorted(plan.remove_keys)),
            tuple(sorted((a.start, a.end, a.width, a.layer, a.net_id)
                         for a in plan.additions)),
            round(plan.saved_mm, 9))


def test_batch_result_is_independent_of_input_order():
    original = staircase(24)
    eligible = {segment_key(s) for s in original}
    expected = [_plan_signature(p) for p in generate_candidate_plans(
        BoardModel(list(original)), eligible)]
    for seed in range(10):
        shuffled = list(original)
        random.Random(seed).shuffle(shuffled)
        actual = [_plan_signature(p) for p in generate_candidate_plans(
            BoardModel(shuffled), eligible)]
        assert actual == expected


def test_kicad_netclass_clearance_is_honored():
    segs = staircase(count=8, pitch=1.0)
    foreign = Segment(11.5, 10.8, 11.5, 12.2, 0.15, 0, 2, "foreign")
    model = BoardModel(segs + [foreign], net_clearances={1: 0.8, 2: 0.8},
                       minimum_clearance=0.2)
    result = smooth_selected_chains(model, {segment_key(s) for s in segs},
                                    clearance=0.0, span_strategy="global")
    from kicad_track_gloss.engine.geometry import segment_distance
    for new in result.additions:
        assert segment_distance(new.start, new.end,
                                (foreign.start_x, foreign.start_y),
                                (foreign.end_x, foreign.end_y)) >= 0.95 - 1e-6


def test_batch_pool_contains_combined_and_isolated_fallbacks():
    first = staircase(12, net=1)
    second = [Segment(s.start_x, s.start_y + 20, s.end_x, s.end_y + 20,
                      s.width, s.layer, 2, "b" + s.uuid)
              for s in staircase(12, net=2)]
    all_segments = first + second
    plans = generate_candidate_plans(
        BoardModel(all_segments), {segment_key(s) for s in all_segments})
    first_keys = {segment_key(s) for s in first}
    second_keys = {segment_key(s) for s in second}
    removed_sets = [set(plan.remove_keys) for plan in plans if plan.changed]
    assert any(keys & first_keys and keys & second_keys for keys in removed_sets)
    assert any(keys <= first_keys for keys in removed_sets)
    assert any(keys <= second_keys for keys in removed_sets)


def test_parallel_and_sequential_planning_are_identical():
    first = staircase(40, net=1)
    second = [Segment(s.start_x, s.start_y + 20, s.end_x, s.end_y + 20,
                      s.width, s.layer, 2, "p" + s.uuid)
              for s in staircase(40, net=2)]
    segments = first + second
    eligible = {segment_key(segment) for segment in segments}
    sequential = generate_candidate_plans(
        BoardModel(segments), eligible, parallel=False)
    parallel = generate_candidate_plans(
        BoardModel(segments), eligible, parallel=True)

    assert [_plan_signature(plan) for plan in parallel] == [
        _plan_signature(plan) for plan in sequential]


def test_mixed_width_fallback_combinations_keep_connectivity():
    """Individually safe width plans can be unsafe after they are merged."""
    from kicad_track_gloss.engine.validation import validate_result

    segments = [
        Segment(-6, -4, 0, 0, 0.1, 0, 1, "a"),
        Segment(-4, 6, 0, 0, 0.2, 0, 1, "b"),
    ]
    pads = [
        PadRegion(-6, -4, 0.6, 0.6, 0, "circle", 0.15, 1, (0,)),
        PadRegion(-4, 6, 0.6, 0.6, 0, "circle", 0.15, 1, (0,)),
    ]
    model = BoardModel(segments, pad_regions=pads)
    eligible = {"a", "b"}
    plans = generate_candidate_plans(
        model, eligible, min_gain=0.01, clearance=0.0,
        max_refinement_passes=0)

    assert plans
    for plan in plans:
        validate_result(model, eligible, plan, check_connectivity=True)
        removed_mm = sum(total_length([segment]) for segment in segments
                         if segment.uuid in plan.remove_keys)
        added_mm = sum(math.dist(addition.start, addition.end)
                       for addition in plan.additions)
        assert abs(plan.saved_mm - max(0.0, removed_mm - added_mm)) < 1e-9


def test_failed_worker_is_killed_and_reaped():
    import subprocess
    from kicad_track_gloss.engine.parallel import _stop_processes

    class StuckProcess:
        def __init__(self):
            self.killed = False
            self.waited_after_kill = False

        def poll(self):
            return 1 if self.killed else None

        def terminate(self):
            pass

        def wait(self, timeout):
            if not self.killed:
                raise subprocess.TimeoutExpired("worker", timeout)
            self.waited_after_kill = True
            return 1

        def kill(self):
            self.killed = True

    process = StuckProcess()
    _stop_processes([process])
    assert process.killed
    assert process.waited_after_kill


def test_spatial_blocker_queries_match_exhaustive_search():
    from kicad_track_gloss.engine.context import PlannerContext
    from kicad_track_gloss.engine.model import PolygonKeepout
    from kicad_track_gloss.engine.planner import _path_blocker

    rng = random.Random(20260825)
    for trial in range(100):
        segments = []
        for index in range(24):
            x, y = rng.uniform(-20, 20), rng.uniform(-20, 20)
            segments.append(Segment(
                x, y, x + rng.uniform(-5, 5), y + rng.uniform(-5, 5),
                rng.choice((0.1, 0.2, 0.5)), rng.choice((0, 2)),
                rng.choice((2, 3)), "foreign-{}-{}".format(trial, index)))
        obstacles = [CircleObstacle(
            rng.uniform(-20, 20), rng.uniform(-20, 20),
            rng.uniform(0.1, 1.5), rng.choice((2, 3)), (0, 2), "via",
            rng.uniform(0, 0.4)) for _ in range(12)]
        pads = [PadRegion(
            rng.uniform(-20, 20), rng.uniform(-20, 20),
            rng.uniform(0.2, 3), rng.uniform(0.2, 3), rng.uniform(0, 180),
            rng.choice(("circle", "rect", "oval", "roundrect")), 0.1,
            rng.choice((2, 3)), (0, 2), rng.uniform(0, 0.4))
                for _ in range(12)]
        keepouts = []
        for _ in range(8):
            x, y = rng.uniform(-20, 20), rng.uniform(-20, 20)
            width, height = rng.uniform(0.2, 3), rng.uniform(0.2, 3)
            keepouts.append(PolygonKeepout(
                ((x, y), (x + width, y), (x + width, y + height),
                 (x, y + height)), (rng.choice((0, 2)),)))
        model = BoardModel(
            segments=segments, obstacles=obstacles, keepouts=keepouts,
            net_clearances={1: 0.1, 2: 0.25, 3: 0.4},
            minimum_clearance=0.1, pad_regions=pads)
        moving = Segment(0, 0, 1, 1, rng.choice((0.1, 0.25, 0.6)),
                         rng.choice((0, 2)), 1, "moving")
        path = [(rng.uniform(-20, 20), rng.uniform(-20, 20)),
                (rng.uniform(-20, 20), rng.uniform(-20, 20))]

        class ExhaustiveContext:
            segment_by_key = {segment_key(item): item for item in model.segments}

            def nearby_segments(self, *_args):
                return model.segments

            def nearby_obstacles(self, *_args):
                return model.obstacles

            def nearby_pads(self, *_args):
                return model.pad_regions

            def nearby_keepouts(self, *_args):
                return model.keepouts

        expected = _path_blocker(
            model, path, moving, set(), 0.1, ExhaustiveContext())
        actual = _path_blocker(
            model, path, moving, set(), 0.1, PlannerContext(model))
        assert actual == expected


def test_mixed_width_chain_is_glossed_without_merging_width_values():
    segments = [
        Segment(0, 0, 0, 3, 0.0889, 0, 1, "thin"),
        Segment(0, 3, 3, 0, 0.09, 0, 1, "wide"),
    ]
    result = smooth_selected_chains(
        BoardModel(segments), {segment.uuid for segment in segments},
        min_gain=0.01, span_strategy="global", clearance=0.0)

    assert result.changed
    assert set(result.remove_keys) == {"thin", "wide"}
    assert {round(addition.width, 6) for addition in result.additions} == {
        0.0889, 0.09}
    assert all(abs(addition.start[1]) < 1e-9 and
               abs(addition.end[1]) < 1e-9
               for addition in result.additions)
    assert result.saved_mm > 4.2


def test_non_octolinear_copper_is_normalized_even_when_length_increases():
    segments = [
        Segment(0, 0, 0.5, 1.0, 0.2, 0, 1, "a"),
        Segment(0.5, 1.0, 1.0, 2.0, 0.2, 0, 1, "b"),
    ]
    before = sum(((s.end_x - s.start_x) ** 2 +
                  (s.end_y - s.start_y) ** 2) ** 0.5 for s in segments)
    result = smooth_selected_chains(
        BoardModel(segments), {segment.uuid for segment in segments},
        min_gain=0.01, span_strategy="global", clearance=0.0)
    after = sum(((a.end[0] - a.start[0]) ** 2 +
                 (a.end[1] - a.start[1]) ** 2) ** 0.5
                for a in result.additions)

    assert result.changed
    assert result.angle_corrections == 2
    assert result.saved_mm == 0.0
    assert after > before
    assert all(
        abs(a.end[0] - a.start[0]) < 1e-9 or
        abs(a.end[1] - a.start[1]) < 1e-9 or
        abs(abs(a.end[0] - a.start[0]) -
            abs(a.end[1] - a.start[1])) < 1e-9
        for a in result.additions)


def test_rp2350_uart_mixed_width_pad_route_is_jointly_glossed():
    # Extracted from rp2350_fpga_eensy_prePlane.kicad_pcb,
    # /RP2354A/RP.UART0_TX. The 0.0889/0.09 transition must move; neither
    # width may be rounded into the other.
    selected = [
        Segment(146.05, 110.6275, 146.05, 111.1275,
                0.0889, 0, 50, "uart-a"),
        Segment(146.05, 111.1275, 144.75, 112.45,
                0.0889, 0, 50, "uart-b"),
        Segment(144.75, 112.45, 141.1, 108.8,
                0.09, 0, 50, "uart-c"),
    ]
    pads = [
        PadRegion(146.05, 110.6275, 0.2, 0.8, 0.0,
                  "roundrect", 0.05, 50, (0,)),
        PadRegion(140.88, 108.815, 1.6, 1.6, 270.0,
                  "circle", 0.4, 50, (0,)),
    ]
    result = generate_candidate_plans(
        BoardModel(selected, pad_regions=pads),
        {segment.uuid for segment in selected}, min_gain=0.01,
        allow_equal_length_simpler=True, clearance=0.0)[0]

    assert result.saved_mm > 2.5
    assert result.angle_corrections == 1
    assert set(result.remove_keys) == {segment.uuid for segment in selected}
    assert {round(addition.width, 6) for addition in result.additions} == {
        0.0889, 0.09}


def test_rp2350_vreg_lx_arbitrary_angle_becomes_octolinear():
    # Extracted from the same board, Net-(U6-VREG_LX). The original -54.98
    # degree segment must not survive merely because it is shorter.
    selected = [
        Segment(150.85, 103.7275, 150.85, 103.2275,
                0.0889, 0, 74, "vreg-a"),
        Segment(150.85, 103.2275, 151.5, 102.3,
                0.0889, 0, 74, "vreg-b"),
        Segment(151.5, 102.3, 152.0, 101.8,
                0.09, 0, 74, "vreg-c"),
        Segment(152.0, 101.8, 152.0, 100.4,
                0.09, 0, 74, "vreg-d"),
    ]
    continuation = Segment(
        152.0, 100.4, 151.975, 100.415, 0.09, 0, 74, "immutable")
    pads = [
        PadRegion(150.85, 103.7275, 0.2, 0.8, 0.0,
                  "roundrect", 0.05, 74, (0,)),
        PadRegion(151.975, 100.415, 0.8, 1.8, 0.0,
                  "roundrect", 0.2, 74, (0,)),
    ]
    result = generate_candidate_plans(
        BoardModel(selected + [continuation], pad_regions=pads),
        {segment.uuid for segment in selected}, min_gain=0.01,
        allow_equal_length_simpler=True, clearance=0.0)[0]

    assert result.saved_mm > 1.3
    assert result.angle_corrections == 1
    assert all(
        abs(a.end[0] - a.start[0]) < 1e-6 or
        abs(a.end[1] - a.start[1]) < 1e-6 or
        abs(abs(a.end[0] - a.start[0]) -
            abs(a.end[1] - a.start[1])) < 1e-6
        for a in result.additions)


def test_action_plugin_is_silent_and_has_no_file_roundtrip():
    from pathlib import Path
    source = Path("kicad_track_gloss/action_plugin.py").read_text(encoding="utf-8")
    for forbidden in ("MessageBox", "GlossDialog", "SaveBoard", "LoadBoard",
                      "kicad-cli", "choose_best_with_kicad"):
        assert forbidden not in source
    assert "KiCadTrackGlossDiagnosticPlugin" in source
    assert "show_toolbar_button = False" in source
    assert "wx.Bell()" in source
    assert "wx.MessageDialog" in source
    assert "Select at least one straight track segment" in source
    assert "Plugin version: " in source
    assert 'label="Copier"' in source
    assert "Use KiCad Undo to revert it." not in source
    assert "Result: modification applied to the current board." not in source


def test_normal_action_bells_once_only_on_noop():
    import importlib

    calls = []
    fake_pcbnew = types.ModuleType("pcbnew")
    fake_pcbnew.ActionPlugin = object
    fake_wx = types.ModuleType("wx")
    fake_wx.Bell = lambda: calls.append("bell")
    previous_pcbnew = sys.modules.get("pcbnew")
    previous_wx = sys.modules.get("wx")
    sys.modules["pcbnew"] = fake_pcbnew
    sys.modules["wx"] = fake_wx
    sys.modules.pop("kicad_track_gloss.action_plugin", None)
    try:
        module = importlib.import_module("kicad_track_gloss.action_plugin")
        named = Segment(0, 0, 1, 0, 0.2, 0, 17, "named",
                        net_name="Net-(U2A-DATA_8)")
        unnamed = Segment(0, 1, 1, 1, 0.2, 0, 18, "unnamed")
        assert module._eligible_net_names(
            BoardModel([named, unnamed]), {"named", "unnamed"}) == [
                "Net-(U2A-DATA_8)", "net 18"]

        class SelectedItem:
            def __init__(self, kind="OTHER", selected=True):
                self.kind = kind
                self.selected = selected

            def GetClass(self):
                return self.kind

            def IsSelected(self):
                return self.selected

        footprint = SelectedItem()
        footprint.Pads = lambda: [SelectedItem()]

        class SelectedBoard:
            def GetTracks(self):
                return [SelectedItem("PCB_TRACK"), SelectedItem("PCB_ARC"),
                        SelectedItem("PCB_VIA")]

            def GetFootprints(self):
                return [footprint]

            def GetDrawings(self):
                return [SelectedItem()]

            def Zones(self):
                return [SelectedItem()]

        assert module._selection_counts(SelectedBoard()) == {
            "segments": 1, "arcs": 1, "vias": 1, "other": 4}
        plugin = module.KiCadTrackGlossPlugin()
        plugin._run = lambda _report: False
        plugin.Run()
        assert calls == ["bell"]
        plugin._run = lambda _report: True
        plugin.Run()
        assert calls == ["bell"]
        warnings = []
        module._show_selection_warning = lambda: warnings.append("warning")

        def no_selection(_report):
            raise module.NoTrackSelection()

        plugin._run = no_selection
        plugin.Run()
        assert warnings == ["warning"]
        assert calls == ["bell"]
    finally:
        sys.modules.pop("kicad_track_gloss.action_plugin", None)
        if previous_pcbnew is None:
            sys.modules.pop("pcbnew", None)
        else:
            sys.modules["pcbnew"] = previous_pcbnew
        if previous_wx is None:
            sys.modules.pop("wx", None)
        else:
            sys.modules["wx"] = previous_wx


def test_pcm_archive_uses_flat_entrypoint_with_internal_packages():
    from kicad_track_gloss.package_pcm import build

    with tempfile.TemporaryDirectory() as directory:
        archive_path = build(directory, "v-test-alpha")
        with zipfile.ZipFile(archive_path) as archive:
            names = set(archive.namelist())
            packaged_metadata = json.loads(
                archive.read("metadata.json").decode("utf-8"))
        official_path = (Path(directory) / "kicad-official" / "packages" /
                         "com.github.fca1.kicadtrackgloss" / "metadata.json")
        official_metadata = json.loads(
            official_path.read_text(encoding="utf-8"))
        archive_size = archive_path.stat().st_size

    assert "plugins/__init__.py" in names
    assert "plugins/action_plugin.py" in names
    assert "plugins/version.py" in names
    assert "plugins/engine/planner.py" in names
    assert "plugins/engine/context.py" in names
    assert "plugins/engine/parallel.py" in names
    assert "plugins/engine/pads.py" in names
    assert "plugins/engine/statistics.py" in names
    assert "plugins/engine/terminals.py" in names
    assert "plugins/kicad/adapter.py" in names
    assert "plugins/kicad/diagnostics.py" in names
    assert "plugins/kicad/reader.py" in names
    assert not any(name.startswith("plugins/kicad_track_gloss/") for name in names)
    assert "metadata.json" in names
    assert "resources/icon.png" in names
    assert packaged_metadata["$schema"].endswith("/v2")
    assert "download_url" not in packaged_metadata["versions"][0]
    official_version = official_metadata["versions"][0]
    assert official_version["status"] == "testing"
    assert official_version["download_url"].endswith(
        "/v-test-alpha/" + archive_path.name)
    assert len(official_version["download_sha256"]) == 64
    assert official_version["download_size"] == archive_size
    assert official_metadata["maintainer"]["name"] == "Frantz"


def test_plugin_version_matches_metadata():
    from pathlib import Path
    from kicad_track_gloss.version import __version__

    metadata = json.loads(
        Path("kicad_track_gloss/metadata.json").read_text(encoding="utf-8"))
    assert metadata["versions"][0]["version"] == __version__


class _NativePoint:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _NativeUuid:
    def __init__(self, value):
        self.value = value

    def AsString(self):
        return self.value


class _NativeTrack:
    def __init__(self, uuid, start, end, net, clearance=0.127):
        self.uuid = uuid
        self.start = _NativePoint(*start)
        self.end = _NativePoint(*end)
        self.net = net
        self.clearance = clearance

    def GetClass(self): return "PCB_TRACK"
    def GetStart(self): return self.start
    def GetEnd(self): return self.end
    def GetWidth(self): return 0.2
    def GetLayer(self): return 0
    def GetNetCode(self): return self.net
    def GetNetname(self): return "NET" + str(self.net)
    def GetUuid(self): return _NativeUuid(self.uuid)
    def GetOwnClearance(self, _layer): return self.clearance
    def IsLocked(self): return False


class _NativeConnectivity:
    def __init__(self, tracks):
        self.tracks = tracks

    def GetConnectedTracks(self, source):
        anchors = {(source.start.x, source.start.y), (source.end.x, source.end.y)}
        return [track for track in self.tracks if track is not source and
                track.net == source.net and anchors &
                {(track.start.x, track.start.y), (track.end.x, track.end.y)}]

    def GetConnectedPads(self, _source):
        return []


class _NativeBoard:
    def __init__(self, tracks):
        self.connectivity = _NativeConnectivity(tracks)

    def GetConnectivity(self):
        return self.connectivity


class _NativePcbnew:
    @staticmethod
    def ToMM(value): return value


def _native_records(adapter, tracks):
    records = {}
    for track in tracks:
        segment = adapter._segment_from_item(track)
        records[segment_key(segment)] = (track, segment)
    return records


def test_native_connection_expansion_batches_multiple_nets():
    tracks = [
        _NativeTrack("a1", (0, 0), (1, 0), 1),
        _NativeTrack("a2", (1, 0), (2, 0), 1),
        _NativeTrack("a3", (2, 0), (3, 0), 1),
        _NativeTrack("b1", (0, 10), (1, 10), 2),
        _NativeTrack("b2", (1, 10), (2, 10), 2),
    ]
    from kicad_track_gloss.kicad import BoardAdapter
    adapter = BoardAdapter(_NativePcbnew())
    records = _native_records(adapter, tracks)
    seeds = {"a1", "b1"}
    expanded = adapter._expand_seed_keys(_NativeBoard(tracks), records, seeds, [])
    assert expanded == {"a1", "a2", "a3", "b1", "b2"}


def test_native_segment_uses_kicad_resolved_track_clearance():
    from kicad_track_gloss.kicad import BoardAdapter
    adapter = BoardAdapter(_NativePcbnew())
    segment = adapter._segment_from_item(
        _NativeTrack("rule", (0, 0), (1, 0), 1, clearance=0.127))

    assert segment.clearance == 0.127


def test_adapter_rounds_millimetres_to_exact_integer_nanometres():
    class Pcbnew:
        @staticmethod
        def FromMM(_value):
            return 130_199_999  # Demonstrate the SWIG conversion truncation.

        @staticmethod
        def VECTOR2I(x, y):
            return x, y

    from kicad_track_gloss.kicad import BoardAdapter
    adapter = BoardAdapter(Pcbnew())

    assert adapter.from_mm(130.2) == 130_200_000
    assert adapter.vector((130.2, 0.25)) == (130_200_000, 250_000)


def test_via_obstacle_uses_native_non_contiguous_copper_layer_set():
    class LayerSet:
        @staticmethod
        def Seq():
            return [0, 2, 4, 6, 5]

    class Via:
        @staticmethod
        def GetLayerSet():
            return LayerSet()

    class Board:
        @staticmethod
        def GetLayerName(layer):
            return {0: "F.Cu", 2: "B.Cu", 4: "In1.Cu",
                    6: "In2.Cu", 5: "F.Silkscreen"}[int(layer)]

    from kicad_track_gloss.kicad.reader import _via_copper_layers

    assert _via_copper_layers(Board(), Via()) == (0, 2, 4, 6)


def test_native_connection_expansion_stops_at_junction():
    tracks = [
        _NativeTrack("seed", (0, 0), (1, 0), 1),
        _NativeTrack("straight", (1, 0), (2, 0), 1),
        _NativeTrack("branch", (1, 0), (1, 1), 1),
    ]
    from kicad_track_gloss.kicad import BoardAdapter
    adapter = BoardAdapter(_NativePcbnew())
    records = _native_records(adapter, tracks)
    expanded = adapter._expand_seed_keys(_NativeBoard(tracks), records, {"seed"}, [])
    assert expanded == {"seed"}


def test_mid_track_t_junction_is_a_sliding_gloss_termination():
    points = [(0, 0), (1, 0), (1, 1), (2, 1),
              (3, 1), (3, 0), (4, 0)]
    selected = [Segment(a[0], a[1], b[0], b[1], 0.2, 0, 1, f"t{i}")
                for i, (a, b) in enumerate(zip(points, points[1:]))]
    # This immutable through-track is deliberately not split at (2, 1), which
    # mirrors the KiCad geometry in the reported board screenshot.
    through = Segment(2, -1, 2, 3, 0.2, 0, 1, "through")
    model = BoardModel(selected + [through])
    eligible = {segment_key(segment) for segment in selected}

    assert find_track_terminal_vertices(model, eligible) == {(1, 0, (2, 1))}
    result = smooth_selected_chains(
        model, eligible, min_gain=0.01, span_strategy="global", clearance=0.0)

    assert result.changed
    assert any(abs(point[0] - 2) < 1e-9
               for addition in result.additions
               for point in (addition.start, addition.end))
    assert "through" not in result.remove_keys


def test_connection_between_two_through_tracks_glosses_between_terminations():
    points = [(0, 0), (1, 0), (1, 1), (2, 1), (2, 2), (3, 2)]
    selected = [Segment(a[0], a[1], b[0], b[1], 0.2, 0, 1, f"c{i}")
                for i, (a, b) in enumerate(zip(points, points[1:]))]
    left = Segment(0, -1, 0, 1, 0.2, 0, 1, "left-through")
    right = Segment(3, 1, 3, 3, 0.2, 0, 1, "right-through")
    model = BoardModel(selected + [left, right])
    eligible = {segment_key(segment) for segment in selected}

    terminals = find_track_terminal_vertices(model, eligible)
    assert terminals == {(1, 0, (0, 0)), (1, 0, (3, 2))}
    result = smooth_selected_chains(
        model, eligible, min_gain=0.01, span_strategy="global", clearance=0.0)

    assert result.changed
    surviving = [segment for segment in selected
                 if segment_key(segment) not in result.remove_keys]
    final_segments = [((segment.start_x, segment.start_y),
                       (segment.end_x, segment.end_y)) for segment in surviving]
    final_segments.extend((addition.start, addition.end)
                          for addition in result.additions)
    assert any(abs(point[0]) < 1e-9 for segment in final_segments for point in segment)
    assert any(abs(point[0] - 3) < 1e-9 for segment in final_segments for point in segment)
    assert not ({"left-through", "right-through"} & set(result.remove_keys))


def test_single_branch_endpoint_slides_to_shortest_track_contact():
    selected = [
        Segment(0, 0, 2, 0, 0.2, 0, 1, "s0"),
        Segment(2, 0, 3, 1, 0.2, 0, 1, "s1"),
        Segment(3, 1, 4, 1, 0.2, 0, 1, "s2"),
    ]
    through = Segment(4, -2, 4, 3, 0.2, 0, 1, "through")
    model = BoardModel(selected + [through])
    eligible = {segment_key(segment) for segment in selected}
    result = smooth_selected_chains(
        model, eligible, min_gain=0.01, span_strategy="global", clearance=0.0)

    assert result.changed
    assert set(result.remove_keys) == eligible
    assert len(result.additions) == 1
    assert {result.additions[0].start, result.additions[0].end} == {(0, 0), (4, 0)}
    assert result.saved_mm > 0.4


def test_one_segment_can_shorten_by_sliding_its_t_contact():
    branch = Segment(0, 0, 4, 1, 0.2, 0, 1, "branch")
    through = Segment(4, -2, 4, 3, 0.2, 0, 1, "through")
    model = BoardModel([branch, through])
    result = smooth_selected_chains(
        model, {"branch"}, min_gain=0.01, span_strategy="global", clearance=0.0)

    assert result.remove_keys == ["branch"]
    assert len(result.additions) == 1
    assert {result.additions[0].start, result.additions[0].end} == {(0, 0), (4, 0)}
    assert result.saved_mm > 0.1


def test_one_segment_can_shorten_between_two_pad_copper_areas():
    segment = Segment(211.7, 94.3, 209.2, 96.8, 0.127, 0, 47, "bst")
    pads = [
        PadRegion(212.06, 94.6605, 1.075, 0.95, 90.0,
                  "roundrect", 0.2375, 47, (0,)),
        PadRegion(209.1545, 96.793, 1.325, 0.6, 0.0,
                  "roundrect", 0.15, 47, (0,)),
    ]
    model = BoardModel([segment], pad_regions=pads)
    result = smooth_selected_chains(
        model, {"bst"}, min_gain=0.01, span_strategy="global", clearance=0.0)

    assert result.changed
    assert result.remove_keys == ["bst"]
    assert result.saved_mm > 0.59
    assert len(result.additions) <= 2
    from kicad_track_gloss.engine.pads import pad_contains
    endpoints = {point for addition in result.additions
                 for point in (addition.start, addition.end)}
    assert any(pad_contains(pads[0], point, 1e-6) for point in endpoints)
    assert any(pad_contains(pads[1], point, 1e-6) for point in endpoints)
    assert [item.mechanism for item in result.transformations] == ["pad_slide"]
    assert [item.geometry for item in result.transformations] == [
        "corner_relocation"]
    summary = summarize_plan(model, {"bst"}, result)
    assert summary["saved_mm"] == result.saved_mm
    assert summary["fixed_gain"] == 0.0
    assert summary["terminal_gain"] == result.saved_mm
    assert summary["mechanisms"][0]["key"] == "pad_slide"


def test_diagnostic_collection_does_not_change_the_edit_plan():
    segments = staircase(12)
    model = BoardModel(segments)
    eligible = {segment_key(segment) for segment in segments}
    diagnostic = smooth_selected_chains(
        model, eligible, span_strategy="global", collect_statistics=True)
    normal = smooth_selected_chains(
        model, eligible, span_strategy="global", collect_statistics=False)

    assert _plan_signature(normal) == _plan_signature(diagnostic)
    assert diagnostic.transformations and diagnostic.search_counts
    assert normal.transformations == []
    assert normal.search_counts == {}


def test_transformation_statistics_use_real_addition_count_and_signed_gain():
    from kicad_track_gloss.engine.statistics import classify_transformation

    segments = [Segment(0, 0, 1, 0, 0.1, 0, 1, "one")]
    transformation = classify_transformation(
        segments, [(0, 0), (1, 1)], "fixed_endpoints",
        after_segments=2)

    assert transformation.after_segments == 2
    assert transformation.net_gain_mm < 0
    assert transformation.saved_mm == 0


def test_diagnostic_report_contains_human_and_machine_readable_statistics():
    from kicad_track_gloss.kicad.diagnostics import (append_plan_statistics,
                                                      split_diagnostic_report)

    segments = staircase(12)
    model = BoardModel(segments)
    eligible = {segment_key(segment) for segment in segments}
    plan = smooth_selected_chains(model, eligible, span_strategy="global")
    report = []
    append_plan_statistics(report, summarize_plan(model, eligible, plan))
    text = "\n".join(report)

    assert "Gloss statistics:" in text
    assert "By optimization mechanism:" in text
    assert "By geometry pattern:" in text
    assert "Search statistics:" in text
    assert "Non-octolinear segments corrected:" in text
    assert "Machine-readable JSON:" in text

    summary, details, json_lines = split_diagnostic_report([
        "KiCad Track Gloss diagnostic", "Plugin version: 0.3.20",
        "KiCad version: 10.0.5", "Eligible net(s) (1): TEST",
        "Optimization coordinates: exact copper geometry; active KiCad grid not used.",
    ] + report)
    assert "Length saved: {:.6f} mm".format(plan.saved_mm) in "\n".join(summary)
    assert "Machine-readable JSON:" not in details
    assert json.loads("\n".join(json_lines))["saved_mm"] == plan.saved_mm
    assert any("active KiCad grid not used" in line for line in summary)


def test_diagnostic_reports_the_foreign_net_blocking_a_shortcut():
    from kicad_track_gloss.kicad.diagnostics import append_search_statistics

    selected = [
        Segment(211.4, 95.0, 211.7, 94.7, 0.177693, 0, 8, "a",
                net_name="Net-(U2A-DATA_8)"),
        Segment(211.7, 94.7, 214.3, 94.7, 0.177693, 0, 8, "b",
                net_name="Net-(U2A-DATA_8)"),
        Segment(214.3, 94.7, 214.6, 95.0, 0.177693, 0, 8, "c",
                net_name="Net-(U2A-DATA_8)"),
        Segment(214.6, 95.0, 214.7, 95.0, 0.177693, 0, 8, "d",
                net_name="Net-(U2A-DATA_8)"),
    ]
    blocker = Segment(
        212.7, 95.2, 213.5, 95.2, 0.177693, 0, 11, "blocker",
        net_name="Net-(U2A-DATA_11)")
    result = smooth_selected_chains(
        BoardModel(selected + [blocker],
                   net_clearances={8: 0.2, 11: 0.2}),
        {segment.uuid for segment in selected}, min_gain=0.01,
        allow_equal_length_simpler=False, clearance=0.2)

    assert not result.changed
    assert result.search_counts["foreign_track_clearance"] > 0
    assert result.blocking_nets == {
        "Net-(U2A-DATA_11)":
        result.search_counts["foreign_track_clearance"]}
    report = []
    append_search_statistics(report, result.search_counts, result.blocking_nets)
    text = "\n".join(report)
    assert "Blocking nets:" in text
    assert "Net-(U2A-DATA_11)" in text


def test_refinement_prunes_a_wider_generated_tail_after_same_net_t():
    selected = [
        Segment(219.2, 97.5, 218.8, 97.5, 0.177693, 0, 1,
                "wide-tail", net_name="N"),
        Segment(218.8, 97.925, 219.2, 97.5, 0.1, 0, 1,
                "narrow-a", net_name="N"),
        Segment(218.8, 97.5, 218.2, 96.9, 0.177693, 0, 1,
                "wide-kept", net_name="N"),
        Segment(218.8, 98.4625, 218.8, 97.925, 0.1, 0, 1,
                "narrow-b", net_name="N"),
    ]
    immutable_continuation = Segment(
        218.2, 96.9, 214.9, 100.2, 0.111346, 0, 1,
        "immutable", net_name="N")
    pad = PadRegion(
        218.8, 98.4625, 0.875, 0.2, 0.0,
        "roundrect", 0.05, 1, (0,))
    result = generate_candidate_plans(
        BoardModel(selected + [immutable_continuation], pad_regions=[pad]),
        {segment.uuid for segment in selected}, min_gain=0.01,
        allow_equal_length_simpler=True, clearance=0.0)[0]

    assert result.changed
    assert "wide-tail" in result.remove_keys
    # The mixed-width optimizer may now replace wide-kept as part of a shorter
    # route, but its useful continuation must remain connected rather than
    # being mistaken for the removable free tail.
    assert ("wide-kept" not in result.remove_keys or any(
        abs((addition.start[0] + addition.start[1]) - 315.1) < 1e-6 or
        abs((addition.end[0] + addition.end[1]) - 315.1) < 1e-6
        for addition in result.additions))
    assert not any(
        point == (219.006061, 97.706061)
        for addition in result.additions
        for point in (addition.start, addition.end))
    assert any(
        segment_hits_pad(pad, point, point, margin=0.0)
        for addition in result.additions
        for point in (addition.start, addition.end))
    assert {round(addition.width, 6) for addition in result.additions} == {
        0.1, 0.177693}


def test_batch_rejects_colliding_new_copper_on_different_nets():
    from kicad_track_gloss.engine.validation import validate_result

    model = BoardModel([], minimum_clearance=0.2,
                       net_clearances={1: 0.2, 2: 0.25})
    result = GlossResult(additions=[
        AddedSegment((0, 0), (2, 2), 0.2, 0, 1),
        AddedSegment((0, 2), (2, 0), 0.2, 0, 2),
    ])
    try:
        validate_result(model, set(), result)
    except ValueError as error:
        assert "inter-net clearance" in str(error)
    else:
        raise AssertionError("Crossing additions on different nets were accepted")


def test_real_board_two_sliding_t_terminations_choose_nearest_contacts():
    # Regression extracted from dispenser_labels.kicad_pcb, GND connection
    # 14370d81... + 82ef7f9e... and its two immutable target tracks.
    selected = [
        Segment(158.5, 98.7, 159.1, 98.1, 0.127, 0, 1, "branch-a"),
        Segment(159.1, 98.1, 159.1, 96.4, 0.127, 0, 1, "branch-b"),
    ]
    targets = [
        Segment(158.5, 101.6, 158.5, 96.0, 0.127, 0, 1, "target-a"),
        Segment(159.1, 96.4, 158.4, 95.7, 0.25, 0, 1, "target-b"),
    ]
    result = smooth_selected_chains(
        BoardModel(selected + targets), {"branch-a", "branch-b"},
        min_gain=0.01, span_strategy="global", clearance=0.0)

    assert result.changed
    assert set(result.remove_keys) == {"branch-a", "branch-b"}
    assert len(result.additions) == 1
    assert {result.additions[0].start, result.additions[0].end} == {
        (158.5, 96.0), (158.6, 95.9)}
    assert abs(result.saved_mm - 2.40710678118655) < 1e-9


def test_preexisting_same_net_copper_allows_shorter_horizontal_t_contact():
    # Regression from dispenser_labels VCC. The final 0.041 mm of the shortest
    # horizontal candidate is inside an immutable 0.25 mm VCC target track.
    # Rechecking that already-existing copper against U3-VBUS must not invent a
    # new violation and force the longer 45-degree alternative.
    selected = [
        Segment(159.275, 96.325, 159.275, 98.34, 0.127, 0, 7,
                "vcc-selected-a", net_name="VCC"),
        Segment(159.275, 98.34, 159.587, 98.652, 0.127, 0, 7,
                "vcc-selected-b", net_name="VCC"),
    ]
    same_net_targets = [
        Segment(159.275, 96.325, 159.3, 96.325, 0.127, 0, 7,
                "vcc-start-horizontal", net_name="VCC"),
        Segment(159.275, 96.325, 158.4, 97.2, 0.127, 0, 7,
                "vcc-start-diagonal", net_name="VCC"),
        Segment(159.587, 95.413, 159.587, 98.652, 0.25, 0, 7,
                "vcc-wide-target", net_name="VCC"),
        Segment(159.3, 98.939, 159.587, 98.652, 0.25, 0, 7,
                "vcc-upper-target", net_name="VCC"),
    ]
    foreign = [
        Segment(159.9025, 98.341696, 159.9025, 96.4475, 0.127,
                0, 41, "vbus-vertical", net_name="Net-(U3-VBUS)"),
        Segment(159.9025, 96.4475, 160.65, 95.7, 0.127,
                0, 41, "vbus-diagonal", net_name="Net-(U3-VBUS)"),
    ]
    model = BoardModel(
        selected + same_net_targets + foreign,
        net_clearances={7: 0.25, 41: 0.12}, minimum_clearance=0.12)
    result = generate_candidate_plans(
        model, {segment.uuid for segment in selected}, min_gain=0.01,
        clearance=0.12, collect_statistics=True)[0]

    assert result.changed
    assert len(result.additions) == 1
    assert result.additions[0].start == (159.3, 96.325)
    assert result.additions[0].end == (159.587, 96.325)
    assert abs(result.saved_mm - 2.169234631460415) < 1e-9
    assert result.blocking_nets == {}


def test_same_net_cover_does_not_hide_truly_new_clearance_violation():
    from kicad_track_gloss.engine.context import PlannerContext
    from kicad_track_gloss.engine.planner import _path_blocker

    moving = Segment(159.3, 96.325, 159.7, 96.325, 0.127,
                     0, 7, "moving", net_name="VCC")
    cover = Segment(159.587, 95.413, 159.587, 98.652, 0.25,
                    0, 7, "cover", net_name="VCC")
    foreign = Segment(159.9025, 98.341696, 159.9025, 96.4475, 0.127,
                      0, 41, "foreign", net_name="Net-(U3-VBUS)")
    model = BoardModel(
        [cover, foreign], net_clearances={7: 0.25, 41: 0.12},
        minimum_clearance=0.12)

    blocker = _path_blocker(
        model, ((159.3, 96.325), (159.7, 96.325)), moving, set(), 0.12,
        PlannerContext(model), {"cover", "foreign"})
    assert blocker == ("foreign_track_clearance", 41)


def test_kicad_resolved_track_rule_overrides_larger_netclass_fallback():
    """Regression for dispenser_labels U3-VBUS after a clean KiCad DRC.

    Its netclass clearance is 0.25 mm, but the custom outer-track rule resolved
    by KiCad is 0.127 mm. A safe route must use that evaluated item value.
    """
    from kicad_track_gloss.engine.context import PlannerContext
    from kicad_track_gloss.engine.planner import _path_blocker

    moving = Segment(0, 0, 0, 10, 0.127, 0, 41, "moving",
                     net_name="Net-(U3-VBUS)", clearance=0.127)
    foreign = Segment(0.3, 0, 0.3, 10, 0.127, 0, 7, "foreign",
                      net_name="VCC", clearance=0.127)
    model = BoardModel(
        [foreign], net_clearances={7: 0.25, 41: 0.25},
        minimum_clearance=0.12)

    assert _path_blocker(
        model, ((0, 0), (0, 10)), moving, set(), 0.12,
        PlannerContext(model)) is None

    fallback_moving = Segment(
        0, 0, 0, 10, 0.127, 0, 41, "fallback-moving")
    assert _path_blocker(
        model, ((0, 0), (0, 10)), fallback_moving, set(), 0.12,
        PlannerContext(model)) == ("foreign_track_clearance", 7)
