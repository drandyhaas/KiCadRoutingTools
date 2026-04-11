"""
Smoke test — obstacle_map.

Verifies the module imports cleanly. The Rust grid_router extension
may or may not be built locally; obstacle_map handles that case by
setting GridObstacleMap to None, so import must succeed regardless.
"""
import obstacle_map


def test_import_obstacle_map():
    """Module imports without raising."""
    # obstacle_map exposes GridObstacleMap (None if Rust router not built)
    assert hasattr(obstacle_map, "GridObstacleMap")


def test_obstacle_map_graceful_rust_fallback():
    """
    The module must not crash when the Rust router extension is absent.

    On a fresh fork checkout without `python build_router.py`, grid_router
    is not installed. obstacle_map handles this via try/except and sets
    GridObstacleMap = None. This test documents that contract.
    """
    # Either the Rust extension is built (non-None) or gracefully None.
    gom = obstacle_map.GridObstacleMap
    assert gom is None or callable(gom) or isinstance(gom, type)
