import math

from kicad_track_gloss.gloss_engine import smooth_selected_chains
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
