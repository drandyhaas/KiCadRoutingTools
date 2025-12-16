"""
Visualization callback interface for the PCB router.

This module provides a callback interface that the batch router uses
to communicate with the visualizer without depending on pygame.
"""

from typing import List, Tuple, Set, Optional, Callable, Protocol
from dataclasses import dataclass


@dataclass
class VisualizationData:
    """Data passed to visualization callbacks for rendering obstacles."""
    blocked_cells: List[Set[Tuple[int, int]]]  # Per-layer blocked cells
    blocked_vias: Set[Tuple[int, int]]
    bga_zones_grid: List[Tuple[int, int, int, int]]
    bounds: Tuple[float, float, float, float]  # min_x, min_y, max_x, max_y in mm


class VisualizationCallback(Protocol):
    """Protocol for visualization callbacks."""

    def on_routing_start(self, total_nets: int, layers: List[str], grid_step: float) -> None:
        """Called when batch routing starts."""
        ...

    def on_net_start(self, net_name: str, net_num: int, net_id: int,
                     sources: List[Tuple[int, int, int]],
                     targets: List[Tuple[int, int, int]],
                     obstacles, vis_data: VisualizationData) -> None:
        """Called when a net starts routing.

        Args:
            net_name: Name of the net
            net_num: Current net number (1-indexed)
            net_id: Net ID
            sources: Source grid coordinates (gx, gy, layer)
            targets: Target grid coordinates (gx, gy, layer)
            obstacles: GridObstacleMap object
            vis_data: Visualization data (blocked cells, BGA zones, bounds)
        """
        ...

    def on_route_step(self, snapshot) -> bool:
        """Called during routing with search snapshot.

        Args:
            snapshot: SearchSnapshot from VisualRouter

        Returns:
            True to continue, False to abort
        """
        ...

    def on_net_complete(self, net_name: str, success: bool, path: Optional[List[Tuple[int, int, int]]],
                        iterations: int, direction: str) -> bool:
        """Called when a net finishes routing.

        Args:
            net_name: Name of the net
            success: Whether routing succeeded
            path: The routed path (if success)
            iterations: Total iterations used
            direction: Direction that succeeded ("forward" or "backward")

        Returns:
            True to continue to next net, False to stop
        """
        ...

    def on_routing_complete(self, successful: int, failed: int, total_iterations: int) -> None:
        """Called when all routing is complete."""
        ...

    def should_pause(self) -> bool:
        """Check if routing should pause (e.g., user pressed space)."""
        ...

    def get_iterations_per_step(self) -> int:
        """Get number of A* iterations to run between visualization updates."""
        ...

    def is_running(self) -> bool:
        """Check if visualization is still running (not quit by user)."""
        ...


class NullVisualizationCallback:
    """No-op callback for when visualization is disabled."""

    def on_routing_start(self, total_nets: int, layers: List[str], grid_step: float) -> None:
        pass

    def on_net_start(self, net_name: str, net_num: int, net_id: int,
                     sources: List[Tuple[int, int, int]],
                     targets: List[Tuple[int, int, int]],
                     obstacles, vis_data: VisualizationData) -> None:
        pass

    def on_route_step(self, snapshot) -> bool:
        return True

    def on_net_complete(self, net_name: str, success: bool, path: Optional[List[Tuple[int, int, int]]],
                        iterations: int, direction: str) -> bool:
        return True

    def on_routing_complete(self, successful: int, failed: int, total_iterations: int) -> None:
        pass

    def should_pause(self) -> bool:
        return False

    def get_iterations_per_step(self) -> int:
        return 100000  # Run full routing without interruption

    def is_running(self) -> bool:
        return True
