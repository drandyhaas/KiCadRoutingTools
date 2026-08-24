import math
import random

from kicad_track_gloss.gloss_engine import generate_candidate_plans, smooth_selected_chains
from kicad_track_gloss.model import BoardModel, CircleObstacle, Segment, segment_key
from kicad_track_gloss.board_adapter import _meander_keys


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
        from kicad_track_gloss.geometry import point_segment_distance
        assert point_segment_distance((obstacle.x, obstacle.y), new.start, new.end) >= 0.475 - 1e-6


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
    from kicad_track_gloss.geometry import segment_distance
    for new in result.additions:
        assert segment_distance(new.start, new.end,
                                (foreign.start_x, foreign.start_y),
                                (foreign.end_x, foreign.end_y)) >= 0.95 - 1e-6


def test_kicad_drc_oracle_tries_next_best_candidate():
    import pathlib
    import kicad_track_gloss.drc_validation as drc
    from kicad_track_gloss.model import GlossResult

    first = GlossResult(saved_mm=2.0)
    second = GlossResult(saved_mm=1.0)

    class FakePcbnew:
        @staticmethod
        def SaveBoard(path, board):
            pathlib.Path(path).write_text("board", encoding="ascii")

        @staticmethod
        def LoadBoard(path):
            return object()

    class FakeBoard:
        def GetFileName(self):
            return ""

    class FakeAdapter:
        def __init__(self, module):
            pass

        def apply(self, board, plan, rollback_on_error=False):
            return None

    calls = []

    def fake_run(cli, board_path, report_path):
        calls.append(str(board_path))
        if "baseline" in str(board_path):
            return [("existing",)]
        if "candidate-0" in str(board_path):
            return [("existing",), ("new",)]
        return [("existing",)]

    saved = drc.find_kicad_cli, drc.BoardAdapter, drc._run
    try:
        drc.find_kicad_cli = lambda: "kicad-cli"
        drc.BoardAdapter = FakeAdapter
        drc._run = fake_run
        chosen, comparison, attempt = drc.choose_best_with_kicad(
            FakePcbnew, FakeBoard(), [first, second])
        assert chosen is second and attempt == 2
        assert comparison.new_violations == [] and len(calls) == 3
    finally:
        drc.find_kicad_cli, drc.BoardAdapter, drc._run = saved


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
