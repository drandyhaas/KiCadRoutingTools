import math
import random
import sys
import tempfile
import time
import types
import zipfile
import json
from pathlib import Path

import pytest

from kicad_track_gloss.engine import (combine_plans, find_track_terminal_vertices,
                                      generate_candidate_plans,
                                      generate_converged_plan,
                                      generate_single_connection_alternatives,
                                      rank_candidate_plans,
                                      smooth_selected_chains, summarize_plan)
from kicad_track_gloss.engine.model import (AddedSegment, BoardModel,
                                            CircleObstacle, GlossResult,
                                            PadRegion, Segment, segment_key)
from kicad_track_gloss.engine.pads import segment_hits_pad
from kicad_track_gloss.kicad.native_salvage import (
    maximize_safe_native_candidates, maximize_safe_native_connections)
from kicad_track_gloss.engine.planner import (
    PlanningDeadlineExceeded, _apply_to_model,
    _group_dependency_signature)


def test_expired_planning_deadline_stops_before_candidate_search():
    model = BoardModel(staircase(8))
    eligible = {segment_key(segment) for segment in model.segments}
    with pytest.raises(PlanningDeadlineExceeded):
        smooth_selected_chains(
            model, eligible, deadline=time.monotonic() - 1.0)


def test_disjoint_local_connection_plans_compose_monotonically():
    segments = [
        Segment(0, 0, 1, 1, 0.2, 0, 1, "a1", net_name="A"),
        Segment(1, 1, 2, 0, 0.2, 0, 1, "a2", net_name="A"),
        Segment(0, 10, 1, 11, 0.2, 0, 2, "b1", net_name="B"),
        Segment(1, 11, 2, 10, 0.2, 0, 2, "b2", net_name="B"),
    ]
    model = BoardModel(segments)
    first = GlossResult(
        remove_keys=["a1", "a2"],
        additions=[AddedSegment((0, 0), (2, 0), 0.2, 0, 1)],
        saved_mm=2 * math.sqrt(2) - 2, convergence_passes=1,
        fixed_point=True)
    second = GlossResult(
        remove_keys=["b1", "b2"],
        additions=[AddedSegment((0, 10), (2, 10), 0.2, 0, 2)],
        saved_mm=2 * math.sqrt(2) - 2, convergence_passes=2,
        fixed_point=True)

    combined = combine_plans(
        model, {segment.uuid for segment in segments}, [first, second])

    assert set(combined.remove_keys) == {"a1", "a2", "b1", "b2"}
    assert abs(combined.saved_mm - 2 * (2 * math.sqrt(2) - 2)) < 1e-9
    assert combined.convergence_passes == 2
    assert combined.fixed_point

    weaker_global = GlossResult(
        remove_keys=["a1", "a2"],
        additions=[AddedSegment((0, 0), (2, 0), 0.2, 0, 1)],
        saved_mm=first.saved_mm)
    assert rank_candidate_plans([weaker_global, combined])[0] is combined


def _native_result(allowed, mode="native_parallel"):
    return types.SimpleNamespace(
        allowed=allowed, error="", validation_mode=mode,
        increases={}, timings_ms={})


def _candidate(label, saved_mm):
    return GlossResult(
        remove_keys=[label],
        additions=[AddedSegment((0, 0), (1, 0), 0.2, 0, 1)],
        saved_mm=saved_mm, convergence_passes=1, fixed_point=True)


def test_native_candidate_search_never_drops_safe_third_candidate():
    global_plan = _candidate("global", 154.0)
    connection_plan = _candidate("connection", 152.5)
    conservative = _candidate("conservative", 148.5)
    ladder_calls = []
    followups = []

    class Adapter:
        def validate_plan_ladder(self, _board, plans, **_kwargs):
            ladder_calls.append(tuple(plan.remove_keys[0] for plan in plans))
            return [_native_result(False), _native_result(False),
                    _native_result(True)]

        def validate_plan(self, _board, plan, **_kwargs):
            followups.append(plan.remove_keys[0])
            return _native_result(False)

    decision = maximize_safe_native_candidates(
        Adapter(), object(), BoardModel([]), set(),
        [global_plan, connection_plan, conservative],
        conservative_plan=conservative, connection_plans=[],
        force_native=False, skip_native=False,
        operation_deadline=time.monotonic() + 5.0,
        wait_callback=None)

    assert ladder_calls == [("global", "conservative", "connection")]
    assert followups == []
    assert decision.plan is connection_plan
    assert decision.native.allowed
    assert decision.fallback_used


def test_native_candidate_search_does_not_salvage_an_identical_incumbent(
        monkeypatch):
    primary = _candidate("primary", 10.0)

    class Adapter:
        def validate_plan(self, _board, _plan, **_kwargs):
            return _native_result(True)

    def unexpected_salvage(*_args, **_kwargs):
        raise AssertionError("identical approved plan must not be salvaged")

    monkeypatch.setattr(
        "kicad_track_gloss.kicad.native_salvage."
        "maximize_safe_native_connections", unexpected_salvage)
    decision = maximize_safe_native_candidates(
        Adapter(), object(), BoardModel([]), {"primary"}, [primary],
        conservative_plan=None, connection_plans=[primary],
        force_native=False, skip_native=False,
        operation_deadline=time.monotonic() + 5.0,
        wait_callback=None)

    assert decision.plan is primary
    assert not decision.salvage_used


def test_native_connection_salvage_runs_three_candidates_in_one_drc_wave():
    segments = []
    plans = []
    eligible = set()
    for net_id, y in enumerate((0.0, 10.0, 20.0), start=1):
        first = Segment(0, y, 1, y + 1, 0.2, 0, net_id,
                        "{}a".format(net_id))
        second = Segment(1, y + 1, 2, y, 0.2, 0, net_id,
                         "{}b".format(net_id))
        segments.extend((first, second))
        eligible.update((first.uuid, second.uuid))
        plans.append(GlossResult(
            remove_keys=[first.uuid, second.uuid],
            additions=[AddedSegment((0, y), (2, y), 0.2, 0, net_id)],
            saved_mm=2 * math.sqrt(2) - 2,
            convergence_passes=1, fixed_point=True))
    waves = []

    class Adapter:
        def validate_plan_ladder(self, _board, candidates, **_kwargs):
            waves.append(len(candidates))
            if len(waves) == 1:
                return [_native_result(False), _native_result(False),
                        _native_result(True)]
            return [_native_result(True) for _candidate in candidates]

        def validate_plan(self, _board, _candidate, **_kwargs):
            waves.append(1)
            return _native_result(True)

    result, native, _attempts, _deadline, retained, total = \
        maximize_safe_native_connections(
            Adapter(), object(), BoardModel(segments), eligible, plans,
            force_native=False, skip_native=False,
            operation_deadline=time.monotonic() + 5.0,
            wait_callback=None)

    assert waves[0] == 3
    assert result is not None and native.allowed
    assert retained == total == 3


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


def test_large_convergence_preserves_the_exact_conservative_opening_plan():
    route = staircase(8, pitch=1.0)
    # The locked records make this a large scope without adding candidate
    # work. Real selection expansion excludes them; here they exercise only
    # the refinement-limit contract used by the cached DRC ladder.
    locked = [Segment(
        1000.0 + index, 0.0, 1000.5 + index, 0.0,
        0.2, 0, index + 2, "locked{}".format(index), locked=True)
        for index in range(121)]
    model = BoardModel(route + locked)
    eligible = {segment_key(segment) for segment in model.segments}
    ladder = []

    generate_converged_plan(
        model, eligible, min_gain=0.01, clearance=0.0,
        max_passes=3, parallel=False, conservative_ladder=ladder)
    expected = generate_converged_plan(
        model, eligible, min_gain=0.01, clearance=0.0,
        max_passes=1, return_partial_on_limit=True,
        max_refinement_passes=0, _allow_junction_scopes=False,
        parallel=False)

    assert len(ladder) == 1
    assert _plan_signature(ladder[0]) == _plan_signature(expected)
    assert ladder[0].convergence_passes == 1
    assert not ladder[0].fixed_point


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


def test_long_collinear_chain_has_no_artificial_split_boundary():
    segments = [Segment(
        float(index), 0.0, float(index + 1), 0.0,
        0.2, 0, 1, "long{}".format(index))
        for index in range(105)]
    eligible = {segment_key(segment) for segment in segments}

    result = smooth_selected_chains(
        BoardModel(segments), eligible,
        allow_equal_length_simpler=True, span_strategy="farthest")

    assert set(result.remove_keys) == eligible
    assert len(result.additions) == 1
    assert {result.additions[0].start, result.additions[0].end} == {
        (0.0, 0.0), (105.0, 0.0)}


def test_no_selection_is_noop():
    assert not smooth_selected_chains(BoardModel(staircase()), set()).changed


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
    from kicad_track_gloss.engine.geometry import (point_segment_distance,
                                                   segment_distance)
    for new in result.additions:
        distance = segment_distance(
            new.start, new.end, (foreign.start_x, foreign.start_y),
            (foreign.end_x, foreign.end_y))
        retained_original_copper = any(
            point_segment_distance(
                new.start, (old.start_x, old.start_y),
                (old.end_x, old.end_y)) <= 1e-6 and
            point_segment_distance(
                new.end, (old.start_x, old.start_y),
                (old.end_x, old.end_y)) <= 1e-6
            for old in segs)
        assert distance >= 0.95 - 1e-6 or retained_original_copper


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


def test_parallel_group_cache_recomputes_only_changed_influence_zone(
        monkeypatch):
    from kicad_track_gloss.engine import parallel as parallel_module

    first = [Segment(index, 0, index + 0.5, 0, 0.2, 0, 1,
                     "a{}".format(index)) for index in range(32)]
    second = [Segment(index, 100, index + 0.5, 100, 0.2, 0, 2,
                      "b{}".format(index)) for index in range(32)]
    calls = []

    def fake_parallel(_model, tasks, _kwargs, **_options):
        calls.append(tuple(key for key, _eligible in tasks))
        return ([(key, GlossResult(), "") for key, _eligible in tasks], False)

    monkeypatch.setattr(
        parallel_module, "run_parallel_group_plans", fake_parallel)
    cache = {}
    eligible = {segment.uuid for segment in first + second}

    generate_candidate_plans(
        BoardModel(first + second), eligible, parallel=True,
        converge_groups=True, _group_plan_cache=cache)
    generate_candidate_plans(
        BoardModel(first + second), eligible, parallel=True,
        converge_groups=True, _group_plan_cache=cache)

    moved_second = list(second)
    moved_second[0] = Segment(
        0, 101, 0.5, 101, 0.2, 0, 2, "b0")
    generate_candidate_plans(
        BoardModel(first + moved_second), eligible, parallel=True,
        converge_groups=True, _group_plan_cache=cache)

    assert len(calls[0]) == 2
    assert len(calls) == 2  # The identical middle pass is entirely cached.
    assert calls[1] == (("primary", 2, 0),)


def test_group_dependency_signature_ignores_distant_foreign_copper():
    from kicad_track_gloss.engine.context import PlannerContext

    selected = Segment(0, 0, 10, 0, 0.2, 0, 1, "selected")
    distant = Segment(100, 100, 110, 100, 0.2, 0, 2, "distant")
    first_model = BoardModel([selected, distant], minimum_clearance=0.2)
    moved_model = BoardModel([
        selected,
        Segment(100, 101, 110, 101, 0.2, 0, 2, "distant"),
    ], minimum_clearance=0.2)

    assert _group_dependency_signature(
        first_model, {"selected"}, PlannerContext(first_model)) == \
        _group_dependency_signature(
            moved_model, {"selected"}, PlannerContext(moved_model))

    near_model = BoardModel([
        selected,
        Segment(4, 0.3, 6, 0.3, 0.2, 0, 2, "near"),
    ], minimum_clearance=0.2)
    moved_near_model = BoardModel([
        selected,
        Segment(4, 0.35, 6, 0.35, 0.2, 0, 2, "near"),
    ], minimum_clearance=0.2)
    assert _group_dependency_signature(
        near_model, {"selected"}, PlannerContext(near_model)) != \
        _group_dependency_signature(
            moved_near_model, {"selected"}, PlannerContext(moved_near_model))


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
    report_source = Path(
        "kicad_track_gloss/kicad/report_dialog.py").read_text(encoding="utf-8")
    for forbidden in ("MessageBox", "GlossDialog", "SaveBoard", "LoadBoard",
                      "kicad-cli", "choose_best_with_kicad"):
        assert forbidden not in source
    assert "KiCadTrackGlossDiagnosticPlugin" in source
    assert "show_toolbar_button = False" in source
    assert "wx.Bell()" in report_source
    assert "Select at least one straight track segment" in source
    settings_source = Path(
        "kicad_track_gloss/kicad/settings_dialog.py").read_text(
            encoding="utf-8")
    assert "wx.MessageDialog" in settings_source
    assert 'label="Close"' in settings_source
    assert 'label="Cancel"' in settings_source
    assert "SetToolTip" in settings_source
    assert "current KiCad session" in settings_source
    assert "Plugin version: " in source
    assert 'label="Copy"' in report_source
    for french_label in ("Copier", "Résultat", "Détails"):
        assert french_label not in source
    assert "BUSY_CURSOR_DELAY_SECONDS = 3.0" in source
    assert "wx.ProgressDialog" not in source
    assert "BeginBusyCursor" in source
    assert "EndBusyCursor" in source
    assert "Use KiCad Undo to revert it." not in source
    assert "Result: modification applied to the current board." not in source


def test_normal_action_bells_once_only_on_noop():
    import importlib

    calls = []
    fake_pcbnew = types.ModuleType("pcbnew")
    fake_pcbnew.ActionPlugin = object
    class FakeTrack:
        pass
    class FakeArc(FakeTrack):
        pass
    class FakeVia(FakeTrack):
        pass
    fake_pcbnew.PCB_TRACK = FakeTrack
    fake_pcbnew.PCB_ARC = FakeArc
    fake_pcbnew.PCB_VIA = FakeVia
    fake_pcbnew.PCB_TRACE_T = 1
    fake_pcbnew.PCB_ARC_T = 2
    fake_pcbnew.PCB_VIA_T = 3
    fake_wx = types.ModuleType("wx")
    fake_wx.Bell = lambda: calls.append("bell")
    fake_wx.BeginBusyCursor = lambda: calls.append("busy-begin")
    fake_wx.EndBusyCursor = lambda: calls.append("busy-end")
    fake_wx.YieldIfNeeded = lambda: None
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

        class SelectedTrack(SelectedItem, FakeTrack):
            def Type(self):
                return fake_pcbnew.PCB_TRACE_T

        class SelectedArc(SelectedItem, FakeArc):
            def Type(self):
                return fake_pcbnew.PCB_ARC_T

        class SelectedVia(SelectedItem, FakeVia):
            def Type(self):
                return fake_pcbnew.PCB_VIA_T

        class SelectedBoard:
            def GetTracks(self):
                return [SelectedTrack("PCB_TRACK"), SelectedArc("PCB_ARC"),
                        SelectedVia("PCB_VIA")]

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
        settings = []
        module._show_session_settings = lambda: settings.append("settings")

        def no_selection(_report):
            raise module.NoTrackSelection()

        plugin._run = no_selection
        plugin.Run()
        assert settings == ["settings"]
        assert calls == ["bell"]

        assert module._run_api_neutral(
            lambda: "fast", lambda: calls.append("unexpected-poll")) == "fast"
        assert calls == ["bell"]

        def slow_result():
            time.sleep(0.08)
            return "slow"

        wait_callback, close_cursor = module._busy_cursor_controller(
            time.monotonic(), delay_seconds=0.001)
        assert module._run_api_neutral(
            slow_result, wait_callback) == "slow"
        # A following stage reuses the same active cursor; it must not flicker.
        wait_callback()
        assert calls[-1] == "busy-begin"
        assert calls.count("busy-begin") == 1
        assert calls.count("busy-end") == 0
        close_cursor()
        assert calls[-2:] == ["busy-begin", "busy-end"]

        def slow_failure():
            time.sleep(0.08)
            raise RuntimeError("planning failed")

        wait_callback, close_cursor = module._busy_cursor_controller(
            time.monotonic(), delay_seconds=0.001)
        try:
            with pytest.raises(RuntimeError, match="planning failed"):
                module._run_api_neutral(slow_failure, wait_callback)
        finally:
            close_cursor()
        assert calls[-2:] == ["busy-begin", "busy-end"]

        before = list(calls)
        wait_callback, close_cursor = module._busy_cursor_controller(
            time.monotonic(), delay_seconds=10.0)
        wait_callback()
        close_cursor()
        assert calls == before

        wait_callback, close_cursor = module._busy_cursor_controller(
            time.monotonic() - 4.0, delay_seconds=3.0)
        wait_callback()
        close_cursor()
        assert calls[-2:] == ["busy-begin", "busy-end"]

        local_plans = []
        local_keys = set()
        local_segments = []
        for net_id in range(1, 5):
            y = float(net_id * 20)
            first = Segment(0, y, 1, y + 1, 0.2, 0, net_id,
                            "local-{}-a".format(net_id))
            second = Segment(1, y + 1, 2, y, 0.2, 0, net_id,
                             "local-{}-b".format(net_id))
            local_segments.extend((first, second))
            local_keys.update((first.uuid, second.uuid))
            local_plans.append(GlossResult(
                remove_keys=[first.uuid, second.uuid],
                additions=[AddedSegment((0, y), (2, y), 0.2, 0, net_id)],
                saved_mm=2 * math.sqrt(2) - 2,
                convergence_passes=1, fixed_point=True))
        local_model = BoardModel(local_segments)

        class ConnectionAdapter:
            @staticmethod
            def _result(candidate):
                nets = {item.net_id for item in candidate.additions}
                allowed = not bool(nets & {1, 2})
                return types.SimpleNamespace(
                    allowed=allowed, error="", timings_ms={},
                    validation_mode="native_parallel")

            def validate_plan(self, _board, candidate, **_kwargs):
                return self._result(candidate)

            def validate_plan_ladder(self, _board, candidates, **_kwargs):
                return [self._result(candidate) for candidate in candidates]

        (connection_plan, connection_native, connection_attempts,
         connection_deadline, retained, total) = \
            maximize_safe_native_connections(
                ConnectionAdapter(), object(), local_model, local_keys,
                local_plans, force_native=False, skip_native=False,
                operation_deadline=time.monotonic() + 5.0,
                wait_callback=lambda: None)
        assert connection_native.allowed
        assert {item.net_id for item in connection_plan.additions} == {3, 4}
        assert connection_attempts >= 4
        assert (retained, total) == (2, 4)
        assert not connection_deadline

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
    assert "plugins/configuration.py" in names
    assert "plugins/internal_config.json" in names
    assert "plugins/engine/planner.py" in names
    assert "plugins/engine/context.py" in names
    assert "plugins/engine/parallel.py" in names
    assert "plugins/engine/pads.py" in names
    assert "plugins/engine/statistics.py" in names
    assert "plugins/engine/terminals.py" in names
    assert "plugins/kicad/adapter.py" in names
    assert "plugins/kicad/diagnostics.py" in names
    assert "plugins/kicad/reader.py" in names
    assert "plugins/kicad/types.py" in names
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
    assert official_metadata["author"]["name"] == "ChatGPT/Codex (OpenAI)"
    assert "co-author" not in packaged_metadata["description_full"].lower()
    assert "inspired by and reusing part of DrAndyHaas's code" in \
        packaged_metadata["description_full"]


def test_plugin_version_matches_metadata():
    from pathlib import Path
    from kicad_track_gloss.version import __version__

    metadata = json.loads(
        Path("kicad_track_gloss/metadata.json").read_text(encoding="utf-8"))
    assert metadata["versions"][0]["version"] == __version__


def test_repository_documentation_is_split_by_public_contract():
    root = Path(__file__).resolve().parents[3]
    required = {
        "plugin-usage.md",
        "cli.md",
        "configuration.md",
        "output-contracts.md",
        "safety-and-drc.md",
        "architecture.md",
    }
    docs = root / "docs"
    assert required <= {path.name for path in docs.glob("*.md")}
    readme = (root / "README.md").read_text(encoding="utf-8")
    for name in required:
        assert "docs/" + name in readme

    output_contract = (docs / "output-contracts.md").read_text(
        encoding="utf-8")
    assert "SCORE_JSON=" in output_contract
    assert "GLOSS_SCORE_JSON=" in output_contract
    assert "SCORE=<float>" in output_contract
    assert "stderr" in output_contract


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
        self.m_Uuid = _NativeUuid(uuid)
        self.start = _NativePoint(*start)
        self.end = _NativePoint(*end)
        self.net = net
        self.clearance = clearance

    def GetClass(self): return "PCB_TRACK"
    def Type(self): return _NativePcbnew.PCB_TRACE_T
    def GetStart(self): return self.start
    def GetEnd(self): return self.end
    def GetWidth(self): return 0.2
    def GetLayer(self): return 0
    def GetNetCode(self): return self.net
    def GetNet(self): return self
    def GetNetname(self): return "NET" + str(self.net)
    def GetOwnClearance(self, _layer): return self.clearance
    def IsLocked(self): return False
    def GetParentGroup(self): return None


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

    def DpCoupledNet(self, _net):
        return None


class _NativePcbnew:
    PCB_IU_PER_MM = 1_000_000
    PCB_TRACE_T = 1
    PCB_ARC_T = 2
    PCB_VIA_T = 3
    PCB_TRACK = _NativeTrack
    PCB_ARC = type("NativeArc", (), {})
    PCB_VIA = type("NativeVia", (), {})

    @staticmethod
    def ToMM(value): return value


def _native_records(adapter, tracks):
    records = {}
    for track in tracks:
        segment = adapter.segment_from_item(track)
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
    scopes = adapter._expand_seed_scopes(
        _NativeBoard(tracks), records, seeds, [])
    assert scopes == (
        frozenset({"a1", "a2", "a3"}),
        frozenset({"b1", "b2"}),
    )


def test_native_segment_uses_kicad_resolved_track_clearance():
    from kicad_track_gloss.kicad import BoardAdapter
    adapter = BoardAdapter(_NativePcbnew())
    segment = adapter.segment_from_item(
        _NativeTrack("rule", (0, 0), (1, 0), 1, clearance=0.127))

    assert segment.clearance == 0.127


def test_adapter_rounds_millimetres_to_exact_integer_nanometres():
    class Pcbnew:
        PCB_IU_PER_MM = 1_000_000

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


def test_live_apply_reads_back_exact_requested_copper():
    class Point:
        def __init__(self, x=0, y=0):
            self.x, self.y = x, y

    class Track:
        sequence = 0

        def __init__(self, _board, uuid=None):
            Track.sequence += 1
            self.m_Uuid = _NativeUuid(uuid or "new{}".format(Track.sequence))
            self.start, self.end = Point(), Point()
            self.width, self.layer, self.net = 0, 0, 0

        def Type(self): return Pcbnew.PCB_TRACE_T
        def GetStart(self): return self.start
        def GetEnd(self): return self.end
        def GetWidth(self): return self.width
        def GetLayer(self): return self.layer
        def GetNetCode(self): return self.net
        def GetNetname(self): return "NET{}".format(self.net)
        def GetOwnClearance(self, _layer): return 0
        def IsLocked(self): return False
        def SetStart(self, value): self.start = value
        def SetEnd(self, value): self.end = value
        def SetWidth(self, value): self.width = value
        def SetLayer(self, value): self.layer = value
        def SetNetCode(self, value): self.net = value

    class Pcbnew:
        PCB_IU_PER_MM = 1_000_000
        PCB_TRACE_T = 1
        PCB_ARC_T = 2
        PCB_VIA_T = 3
        PCB_TRACK = Track
        PCB_ARC = type("Arc", (), {})
        PCB_VIA = type("Via", (), {})
        VECTOR2I = Point

        @staticmethod
        def ToMM(value): return value / 1_000_000

    class Board:
        def __init__(self, tracks): self.tracks = list(tracks)
        def GetTracks(self): return list(self.tracks)
        def Add(self, item): self.tracks.append(item)
        def RemoveNative(self, item): self.tracks.remove(item)

    original = Track(None, "old")
    original.SetStart(Point(0, 0))
    original.SetEnd(Point(1_000_000, 1_000_000))
    original.SetWidth(200_000)
    original.SetNetCode(1)
    board = Board([original])
    from kicad_track_gloss.kicad import BoardAdapter
    adapter = BoardAdapter(Pcbnew())
    plan = GlossResult(
        remove_keys=["old"],
        additions=[AddedSegment((0, 0), (1, 0), 0.2, 0, 1)])

    adapter.apply(board, plan)

    assert len(board.tracks) == 1
    assert board.tracks[0].GetStart().x == 0
    assert board.tracks[0].GetEnd().x == 1_000_000
    assert board.tracks[0].GetEnd().y == 0


def test_live_apply_readback_rejects_a_silent_native_remove_failure(
        monkeypatch):
    from kicad_track_gloss.kicad import writer

    monkeypatch.setattr(writer, "_track_map",
                        lambda _adapter, _board: {"old": object()})

    with pytest.raises(RuntimeError, match="removed tracks still present"):
        writer._verify_applied_plan(
            object(), object(), GlossResult(remove_keys=["old"]), [])


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
        def GetEnabledLayers():
            return LayerSet()

    class Pcbnew:
        @staticmethod
        def IsCopperLayer(layer):
            return int(layer) in (0, 2, 4, 6)

    class Adapter:
        pcbnew = Pcbnew()

    from kicad_track_gloss.kicad.reader import _via_copper_layers

    assert _via_copper_layers(Adapter(), Board(), Via()) == (0, 2, 4, 6)


def test_native_type_ids_classify_generic_swig_wrappers():
    from kicad_track_gloss.kicad.types import is_arc, is_straight_track, is_via

    class Pcbnew:
        PCB_TRACE_T = 13
        PCB_VIA_T = 14
        PCB_ARC_T = 15

    class GenericTrackWrapper:
        def __init__(self, native_type):
            self.native_type = native_type

        def Type(self):
            return self.native_type

    assert is_straight_track(Pcbnew, GenericTrackWrapper(13))
    assert is_via(Pcbnew, GenericTrackWrapper(14))
    assert is_arc(Pcbnew, GenericTrackWrapper(15))


def test_unknown_layer_ids_fail_closed_without_kicad_semantics():
    from kicad_track_gloss.kicad.rules import copper_layers

    class LayerSet:
        @staticmethod
        def Seq():
            return [0, 2, 4, 6]

    class Board:
        @staticmethod
        def GetEnabledLayers():
            return LayerSet()

    class Pcbnew:
        @staticmethod
        def IsCopperLayer(_layer):
            raise RuntimeError("unavailable")

    class Adapter:
        pcbnew = Pcbnew()

    with pytest.raises(RuntimeError, match="unavailable"):
        copper_layers(Adapter(), Board(), LayerSet())


def test_via_layers_use_kicad_10_layer_set_directly():
    from kicad_track_gloss.kicad.reader import _via_copper_layers

    class LayerSet:
        @staticmethod
        def Seq():
            return [0, 3, 5, 31]

    class Board:
        @staticmethod
        def GetEnabledLayers():
            return LayerSet()

    class Via:
        @staticmethod
        def GetLayerSet():
            return LayerSet()

    class Pcbnew:
        @staticmethod
        def IsCopperLayer(layer):
            return layer in (0, 3, 5, 31)

    class Adapter:
        pcbnew = Pcbnew()

    assert _via_copper_layers(Adapter(), Board(), Via()) == (0, 3, 5, 31)


def test_pad_shapes_use_pcbnew_enum_values():
    from kicad_track_gloss.kicad.reader import _pad_shape

    class Pcbnew:
        PAD_SHAPE_CIRCLE = 41
        PAD_SHAPE_RECT = 42
        PAD_SHAPE_OVAL = 43
        PAD_SHAPE_ROUNDRECT = 44

    class Adapter:
        pcbnew = Pcbnew()

    class Pad:
        def __init__(self, shape):
            self.shape = shape

        def GetShape(self):
            return self.shape

    assert _pad_shape(Adapter(), Pad(41)) == "circle"
    assert _pad_shape(Adapter(), Pad(42)) == "rect"
    assert _pad_shape(Adapter(), Pad(43)) == "oval"
    assert _pad_shape(Adapter(), Pad(44)) == "roundrect"
    assert _pad_shape(Adapter(), Pad(99)) is None


def test_exact_board_outline_rejects_concave_shortcut_and_hole():
    from kicad_track_gloss.engine.context import PlannerContext
    from kicad_track_gloss.engine.geometry import segment_inside_board
    from kicad_track_gloss.engine.model import BoardOutline

    outline = BoardOutline(
        (((0, 0), (10, 0), (10, 10), (6, 10), (6, 4),
          (4, 4), (4, 10), (0, 10)),),
        (((1, 1), (3, 1), (3, 3), (1, 3)),))
    assert segment_inside_board((7, 2), (9, 8), outline, 0.1)
    assert not segment_inside_board((3, 8), (7, 8), outline, 0.1)
    assert not segment_inside_board((0.5, 2), (3.5, 2), outline, 0.1)
    context = PlannerContext(BoardModel([], board_outline=outline))
    randomizer = random.Random(20260826)
    cases = [((7, 2), (9, 8)), ((3, 8), (7, 8)),
             ((0.5, 2), (3.5, 2))]
    cases.extend(
        (((randomizer.uniform(-1, 11), randomizer.uniform(-1, 11)),
          (randomizer.uniform(-1, 11), randomizer.uniform(-1, 11))))
        for _ in range(200))
    for start, end in cases:
        assert context.segment_inside_board(start, end, 0.1) == \
            segment_inside_board(start, end, outline, 0.1)


def test_native_drc_report_counts_rule_categories():
    from kicad_track_gloss.kicad.native_validation import (
        _drc_increases, _json_report_summary)

    counts, _fingerprints = _json_report_summary(json.dumps({
        "violations": [{"type": "clearance"}, {"type": "clearance"}],
        "unconnected_items": [{"type": "unconnected_items"}],
    }))
    assert counts == {"clearance": 2, "unconnected_items": 1}
    before = json.dumps({
        "violations": [],
        "unconnected_items": [{
            "type": "unconnected_items", "description": "Missing A",
            "items": [{"description": "Pad A", "pos": {"x": 1, "y": 2}}],
        }],
    })
    after = json.dumps({
        "violations": [],
        "unconnected_items": [{
            "type": "unconnected_items", "description": "Missing B",
            "items": [{"description": "Pad B", "pos": {"x": 3, "y": 4}}],
        }],
    })
    before_counts, before_fingerprints = _json_report_summary(before)
    after_counts, after_fingerprints = _json_report_summary(after)
    assert _drc_increases(
        before_counts, after_counts,
        before_fingerprints, after_fingerprints) == {}
    added = json.dumps({
        "violations": [],
        "unconnected_items": json.loads(after)["unconnected_items"] + [{
            "type": "unconnected_items", "description": "Missing C",
            "items": [{"description": "Pad C", "pos": {"x": 5, "y": 6}}],
        }],
    })
    added_counts, added_fingerprints = _json_report_summary(added)
    assert _drc_increases(
        before_counts, added_counts,
        before_fingerprints, added_fingerprints) == {
            "unconnected_items": 1}


def test_native_drc_fingerprint_preserves_lengths_and_exact_positions():
    from kicad_track_gloss.kicad.native_validation import (
        _drc_increases, _json_report_summary)

    def report(length, x):
        return json.dumps({"violations": [{
            "type": "length_out_of_range",
            "description": "length {} mm".format(length),
            "items": [{"description": "track", "pos": {"x": x, "y": 2}}],
        }]})

    before_counts, before = _json_report_summary(report("10.000", 1.0001))
    changed_length_counts, changed_length = _json_report_summary(
        report("10.001", 1.0001))
    changed_position_counts, changed_position = _json_report_summary(
        report("10.000", 1.0002))
    assert _drc_increases(
        before_counts, changed_length_counts, before, changed_length) == {
        "length_out_of_range": 1}
    assert _drc_increases(
        before_counts, changed_position_counts, before, changed_position) == {
        "length_out_of_range": 1}


def test_native_helpers_are_hidden_on_windows():
    import os
    import subprocess
    from kicad_track_gloss.kicad.native_validation import (
        _hidden_process_kwargs)

    kwargs = _hidden_process_kwargs()
    if os.name == "nt":
        assert kwargs["creationflags"] == subprocess.CREATE_NO_WINDOW
    else:
        assert kwargs == {}


def test_native_drc_requests_all_track_errors(monkeypatch, tmp_path):
    """Avoid nondeterministic per-track finding suppression in KiCad DRC."""
    from kicad_track_gloss.kicad import native_validation

    captured = {}

    class Process:
        returncode = 0
        stdout = ""
        stderr = ""

    def fake_run(command, **kwargs):
        captured["command"] = command
        report = Path(command[command.index("--output") + 1])
        report.write_text(
            '{"violations":[],"unconnected_items":[]}', encoding="utf-8")
        return Process()

    monkeypatch.setattr(native_validation, "_kicad_cli",
                        lambda _adapter: Path("kicad-cli"))
    monkeypatch.setattr(native_validation.subprocess, "run", fake_run)

    native_validation._run_drc(
        object(), tmp_path / "board.kicad_pcb", tmp_path / "report.json")

    assert "--all-track-errors" in captured["command"]


def test_native_candidate_helper_supports_direct_script_entry_point():
    """Reproduce the exact subprocess import mode used for candidate boards."""
    import subprocess

    script = Path(
        "kicad_track_gloss/kicad/native_validation.py").resolve()
    process = subprocess.run(
        [sys.executable, str(script)], capture_output=True, text=True)
    assert process.returncode != 0
    assert "expected --apply-plan BASELINE CANDIDATE PLAN" in process.stderr
    assert "attempted relative import" not in process.stderr


def test_connectivity_partition_reuses_unchanged_net_signature():
    from kicad_track_gloss.engine.validation import (
        _connectivity_partition, _connectivity_signature)

    _connectivity_partition.cache_clear()
    segments = [
        Segment(0, 0, 1, 0, 0.2, 0, 1, "selected"),
        Segment(1, 0, 2, 0, 0.2, 0, 1, "immutable"),
    ]
    pads = [PadRegion(0, 0, 0.5, 0.5, 0, "circle", 0, 1, (0,))]
    first = _connectivity_signature(
        segments, [], pads, {"immutable"}, 1, 0.000001)
    second = _connectivity_signature(
        segments, [], pads, {"immutable"}, 1, 0.000001)

    assert first == second
    assert _connectivity_partition.cache_info().hits == 1


def test_native_fast_path_only_accepts_existing_copper_without_zones():
    from kicad_track_gloss.kicad import BoardAdapter
    from kicad_track_gloss.kicad.native_validation import (
        _is_strict_removal_only_plan)

    track = _NativeTrack("original", (0, 0), (2, 0), 1)

    class Board:
        @staticmethod
        def GetTracks(): return [track]

        @staticmethod
        def Zones(): return []

    adapter = BoardAdapter(_NativePcbnew())
    subset = GlossResult(
        remove_keys=["original"],
        additions=[AddedSegment((0, 0), (1, 0), 0.2, 0, 1)])
    diagonal = GlossResult(
        remove_keys=["original"],
        additions=[AddedSegment((0, 0), (1, 1), 0.2, 0, 1)])

    assert _is_strict_removal_only_plan(adapter, Board(), subset)
    assert not _is_strict_removal_only_plan(adapter, Board(), diagonal)

    class ZonedBoard(Board):
        @staticmethod
        def Zones(): return [object()]

    assert not _is_strict_removal_only_plan(adapter, ZonedBoard(), subset)


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


def test_internal_corner_is_partially_chamfered_before_a_blocker():
    # Left-hand VCC corner from magic_keys.  Replacing both complete segments
    # crosses A5, but cutting back their safe tails produces the useful local
    # 45-degree chamfer made manually in KiCad.
    selected = [
        Segment(106.426, 54.61, 106.426, 44.45,
                0.2, 0, 53, "vertical", net_name="VCC", clearance=0.2),
        Segment(106.426, 44.45, 142.748, 44.45,
                0.2, 0, 53, "horizontal", net_name="VCC", clearance=0.2),
    ]
    blockers = [
        Segment(111.506, 54.61, 111.506, 45.212,
                0.2, 0, 2, "a5-vertical", net_name="Net-(A1-A5)",
                clearance=0.2),
        Segment(111.506, 45.212, 141.986, 45.212,
                0.2, 0, 2, "a5-horizontal", net_name="Net-(A1-A5)",
                clearance=0.2),
    ]
    model = BoardModel(selected + blockers, minimum_clearance=0.2)
    result = smooth_selected_chains(
        model, {"vertical", "horizontal"}, min_gain=0.2,
        span_strategy="global", clearance=0.2)

    assert result.changed
    assert set(result.remove_keys) == {"vertical", "horizontal"}
    assert result.saved_mm > 3.0
    assert any(
        addition.start[0] == pytest.approx(106.426) and
        44.45 < addition.start[1] < 54.61 and
        addition.end[0] > 106.426 and
        addition.end[1] == pytest.approx(44.45)
        for addition in result.additions)
    assert [item.geometry for item in result.transformations] == [
        "corner_chamfer"]


def test_internal_segment_translates_to_last_safe_position_before_obstacle():
    # The globally shortest A->D diagonal crosses the foreign obstacle.  A
    # pure gloss can still slide the internal vertical, retaining the two
    # outer supporting lines and stopping at the clearance boundary.
    selected = [
        Segment(0, 0, 2, 2, 0.2, 0, 1, "a"),
        Segment(2, 2, 2, 6, 0.2, 0, 1, "middle"),
        Segment(2, 6, 6, 6, 0.2, 0, 1, "c"),
    ]
    obstacle = CircleObstacle(5, 5, 0.4, 2, (0,), "via", 0.1)
    model = BoardModel(
        selected, obstacles=[obstacle], minimum_clearance=0.1)

    result = smooth_selected_chains(
        model, {"a", "middle", "c"}, min_gain=0.01,
        span_strategy="global", clearance=0.1, solution_rank=1)

    assert result.changed
    assert result.saved_mm > 0.1
    verticals = [item for item in result.additions
                 if abs(item.start[0] - item.end[0]) < 1e-8]
    assert verticals
    assert 2.0 < verticals[0].start[0] < 5.0


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


def test_larger_t_selection_preserves_smaller_branch_gloss():
    # Extracted from tensorrail_mini GND at (13.7517, 16.0). Selecting the
    # horizontal pad branch used to remove it as a sliding target and reduce
    # the useful saving from 1.260431 mm to an invisible 0.5 mm inside a pad.
    main = Segment(13.7517, 16.0, 10.8392, 18.9125,
                   0.2, 0, 1, "main", net_name="GND")
    tail = Segment(10.8392, 18.9125, 9.8, 18.9125,
                   0.2, 0, 1, "tail", net_name="GND")
    pad_branch = Segment(12.95, 16.0, 13.7517, 16.0,
                         0.2, 0, 1, "pad-branch", net_name="GND")
    immutable_branch = Segment(13.7517, 16.0, 14.2517, 16.5,
                               0.2, 0, 1, "immutable", net_name="GND")
    pad = PadRegion(12.95, 16.0, 1.0, 1.3, 0.0, "rect", 0.0,
                    1, (0,))
    model = BoardModel(
        [main, tail, pad_branch, immutable_branch], pad_regions=[pad])

    smaller = generate_converged_plan(
        model, {"main", "tail"}, max_passes=4,
        return_partial_on_limit=True, group_max_passes=2,
        min_gain=0.01, clearance=0.0)
    larger = generate_converged_plan(
        model, {"main", "tail", "pad-branch"}, max_passes=4,
        return_partial_on_limit=True, group_max_passes=2,
        min_gain=0.01, clearance=0.0)

    assert smaller.saved_mm > 1.0
    assert larger.saved_mm >= smaller.saved_mm - 1e-9
    assert {"main", "tail"}.issubset(larger.remove_keys)


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


def test_single_connection_portfolio_converges_three_terminal_policies():
    selected = [
        Segment(0, 0, 2, 0, 0.2, 0, 1, "s0"),
        Segment(2, 0, 3, 1, 0.2, 0, 1, "s1"),
        Segment(3, 1, 4, 1, 0.2, 0, 1, "s2"),
    ]
    through = Segment(4, -2, 4, 3, 0.2, 0, 1, "through")
    pad = PadRegion(0, 0, 1, 1, 0, "rect", 0, 1, (0,))
    model = BoardModel(selected + [through], pad_regions=[pad])
    eligible = {"s0", "s1", "s2"}
    primary = generate_converged_plan(
        model, eligible, max_passes=None, return_partial_on_limit=True,
        group_max_passes=2, min_gain=0.01, clearance=0.0)

    candidates = generate_single_connection_alternatives(
        model, eligible, primary, min_gain=0.01,
        allow_equal_length_simpler=True, clearance=0.0,
        group_max_passes=2, collect_statistics=True,
        planning_deadline=None, cancellation_grace_seconds=1.0)

    assert len(candidates) == 3
    assert all(candidate.fixed_point for candidate in candidates)
    assert len({tuple((item.start, item.end)
                      for item in candidate.additions)
                for candidate in candidates}) == 3


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

    assert result.changed
    assert result.saved_mm > 0.1
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
