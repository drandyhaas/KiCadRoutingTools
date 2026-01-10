"""Memory debugging utilities for tracking routing memory usage."""

import sys
from typing import Dict, Any, Optional

# Try to import memory tracking libraries (graceful fallback)
_HAS_RESOURCE = False
_HAS_PSUTIL = False

try:
    import resource
    _HAS_RESOURCE = True
except ImportError:
    pass

try:
    import psutil
    _HAS_PSUTIL = True
except ImportError:
    pass


def get_process_memory_mb() -> float:
    """Get current process memory usage in MB.

    Tries multiple methods in order of reliability:
    1. psutil (most accurate, cross-platform)
    2. resource (Unix only)
    """
    if _HAS_PSUTIL:
        process = psutil.Process()
        return process.memory_info().rss / (1024 * 1024)

    if _HAS_RESOURCE:
        # maxrss is in KB on Linux, bytes on macOS
        maxrss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        if sys.platform == 'darwin':
            return maxrss / (1024 * 1024)  # bytes to MB
        return maxrss / 1024  # KB to MB

    return 0.0  # Unable to measure


def format_memory_stats(label: str, memory_mb: float, delta_mb: Optional[float] = None) -> str:
    """Format memory statistics for display."""
    if delta_mb is not None and abs(delta_mb) > 0.1:
        sign = "+" if delta_mb > 0 else ""
        return f"[MEMORY] {label}: {memory_mb:.1f} MB ({sign}{delta_mb:.1f} MB)"
    return f"[MEMORY] {label}: {memory_mb:.1f} MB"


def estimate_net_obstacles_cache_mb(cache: Dict) -> float:
    """Estimate memory size of net_obstacles_cache (Dict[int, NetObstacleData])."""
    if not cache:
        return 0.0

    total = sys.getsizeof(cache)
    for net_id, data in cache.items():
        total += sys.getsizeof(net_id)
        total += sys.getsizeof(data)
        if hasattr(data, 'blocked_cells'):
            # numpy arrays: use nbytes for actual memory usage
            if hasattr(data.blocked_cells, 'nbytes'):
                total += data.blocked_cells.nbytes
            else:
                total += sys.getsizeof(data.blocked_cells)
                total += len(data.blocked_cells) * 24  # tuple of 3 ints (legacy)
        if hasattr(data, 'blocked_vias'):
            if hasattr(data.blocked_vias, 'nbytes'):
                total += data.blocked_vias.nbytes
            else:
                total += sys.getsizeof(data.blocked_vias)
                total += len(data.blocked_vias) * 16  # tuple of 2 ints (legacy)
    return total / (1024 * 1024)


def estimate_track_proximity_cache_mb(cache: Dict) -> float:
    """Estimate memory size of track_proximity_cache."""
    if not cache:
        return 0.0

    total = sys.getsizeof(cache)
    for net_id, layer_dict in cache.items():
        total += sys.getsizeof(net_id) + sys.getsizeof(layer_dict)
        if isinstance(layer_dict, dict):
            for layer_idx, cell_dict in layer_dict.items():
                total += sys.getsizeof(layer_idx) + sys.getsizeof(cell_dict)
                if isinstance(cell_dict, dict):
                    total += len(cell_dict) * 32  # (tuple of 2 ints) -> int
    return total / (1024 * 1024)


def estimate_routed_paths_mb(paths: Dict) -> float:
    """Estimate memory size of routed_net_paths (Dict[int, List[Tuple]])."""
    if not paths:
        return 0.0

    total = sys.getsizeof(paths)
    for net_id, path in paths.items():
        total += sys.getsizeof(net_id) + sys.getsizeof(path)
        if path:
            total += len(path) * 24  # tuple of 3 ints per coordinate
    return total / (1024 * 1024)


def format_obstacle_map_stats(obstacles) -> str:
    """Format Rust GridObstacleMap statistics."""
    if obstacles is None or not hasattr(obstacles, 'get_stats'):
        return "[MEMORY] Obstacle map: N/A (no get_stats method)"

    stats = obstacles.get_stats()
    blocked_cells, blocked_vias, stub_prox, layer_prox, cross_layer, source_target = stats

    # Estimate memory: ~40 bytes per HashMap entry (key + value + overhead)
    bytes_per_entry = 40
    estimated_mb = (blocked_cells + blocked_vias + stub_prox + layer_prox + cross_layer + source_target) * bytes_per_entry / (1024 * 1024)

    return (f"[MEMORY] Rust obstacle map: ~{estimated_mb:.1f} MB estimated\n"
            f"         blocked_cells: {blocked_cells:,}, blocked_vias: {blocked_vias:,}\n"
            f"         stub_proximity: {stub_prox:,}, layer_proximity: {layer_prox:,}\n"
            f"         cross_layer: {cross_layer:,}, source_target: {source_target:,}")
