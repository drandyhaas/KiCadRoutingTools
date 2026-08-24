import math
import random
import sys
import tempfile
import types
import zipfile
import json

from kicad_track_gloss.engine import (find_track_terminal_vertices,
                                      generate_candidate_plans,
                                      smooth_selected_chains)
from kicad_track_gloss.engine.model import (AddedSegment, BoardModel,
                                            CircleObstacle, GlossResult,
                                            Segment, segment_key)
from kicad_track_gloss.kicad.selection import meander_keys as _meander_keys


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


def test_action_plugin_is_silent_and_has_no_file_roundtrip():
    from pathlib import Path
    source = Path("kicad_track_gloss/action_plugin.py").read_text(encoding="utf-8")
    for forbidden in ("MessageBox", "GlossDialog", "SaveBoard", "LoadBoard",
                      "kicad-cli", "choose_best_with_kicad"):
        assert forbidden not in source
    assert "KiCadTrackGlossDiagnosticPlugin" in source
    assert "show_toolbar_button = False" in source
    assert "wx.Bell()" in source
    assert "Plugin version: " in source


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
        plugin = module.KiCadTrackGlossPlugin()
        plugin._run = lambda _report: False
        plugin.Run()
        assert calls == ["bell"]
        plugin._run = lambda _report: True
        plugin.Run()
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
        archive_path = build(directory)
        with zipfile.ZipFile(archive_path) as archive:
            names = set(archive.namelist())

    assert "plugins/__init__.py" in names
    assert "plugins/action_plugin.py" in names
    assert "plugins/version.py" in names
    assert "plugins/engine/planner.py" in names
    assert "plugins/engine/terminals.py" in names
    assert "plugins/kicad/adapter.py" in names
    assert "plugins/kicad/reader.py" in names
    assert not any(name.startswith("plugins/kicad_track_gloss/") for name in names)
    assert "metadata.json" in names
    assert "resources/icon.png" in names


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
    def __init__(self, uuid, start, end, net):
        self.uuid = uuid
        self.start = _NativePoint(*start)
        self.end = _NativePoint(*end)
        self.net = net

    def GetClass(self): return "PCB_TRACK"
    def GetStart(self): return self.start
    def GetEnd(self): return self.end
    def GetWidth(self): return 0.2
    def GetLayer(self): return 0
    def GetNetCode(self): return self.net
    def GetNetname(self): return "NET" + str(self.net)
    def GetUuid(self): return _NativeUuid(self.uuid)
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
