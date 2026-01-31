"""
PyGame implementation of the visualization callback.

This wraps the existing RoutingVisualizer to implement the callback interface.
"""

import time
from typing import List, Tuple, Set, Optional

from .callback import VisualizationCallback, VisualizationData
from .visualizer import RoutingVisualizer
from .config import VisualizerConfig


class PyGameVisualizationCallback(VisualizationCallback):
    """PyGame-based visualization callback implementation."""

    def __init__(self, config: VisualizerConfig = None, auto_advance: bool = False,
                 display_time: float = 0.0):
        """Initialize the PyGame visualizer.

        Args:
            config: Visualizer configuration
            auto_advance: If True, automatically advance to next net
            display_time: Seconds to display completed route before advancing (with auto_advance)
        """
        self.config = config or VisualizerConfig()
        self.visualizer = RoutingVisualizer(self.config)
        self.auto_advance = auto_advance
        self.display_time = display_time

        self._total_nets = 0
        self._grid_step = 0.1
        self._layers: List[str] = []
        self._current_obstacles = None
        self._display_wait_start: Optional[float] = None
        self._waiting_for_next = False

        # In auto-advance mode, use maximum speed
        if auto_advance:
            self.visualizer.iterations_per_frame = self.config.max_speed

    def on_routing_start(self, total_nets: int, layers: List[str], grid_step: float) -> None:
        """Called when batch routing starts."""
        self._total_nets = total_nets
        self._layers = layers
        self._grid_step = grid_step
        self.config.layers = layers
        print(f"\nStarting visualization with {total_nets} nets...")
        print("Controls: Space=pause, N=next net, R=restart, +/-=speed, Z=zoom to net, Q=quit")

    def on_net_start(self, net_name: str, net_num: int, net_id: int,
                     sources: List[Tuple[int, int, int]],
                     targets: List[Tuple[int, int, int]],
                     obstacles, vis_data: VisualizationData) -> None:
        """Called when a net starts routing."""
        self._current_obstacles = obstacles
        self._waiting_for_next = False
        self._display_wait_start = None

        # Update visualizer context
        self.visualizer.set_routing_context(
            obstacles,
            sources,
            targets,
            grid_step=self._grid_step,
            bounds=vis_data.bounds,
            bga_zones=vis_data.bga_zones_grid,
            blocked_cells=vis_data.blocked_cells,
            blocked_vias=vis_data.blocked_vias,
        )
        self.visualizer.set_current_net(net_name, net_num, self._total_nets)
        self.visualizer.status_message = ""
        self.visualizer.snapshot = None

    def on_route_step(self, snapshot) -> bool:
        """Called during routing with search snapshot."""
        self.visualizer.update_snapshot(snapshot)

        # Render and handle events
        self.visualizer.render()
        self.visualizer.tick()

        if not self.visualizer.handle_events():
            return False

        # Check for quit
        if not self.visualizer.running:
            return False

        return True

    def on_net_complete(self, net_name: str, success: bool, path: Optional[List[Tuple[int, int, int]]],
                        iterations: int, direction: str) -> bool:
        """Called when a net finishes routing."""
        if success and path:
            self.visualizer.status_message = f"Path found ({direction}) - N=next"
            self.visualizer.add_completed_route(path)
        else:
            self.visualizer.status_message = f"Failed ({direction}) - N=next"

        # Wait for user to press N (or auto-advance after display_time)
        self._waiting_for_next = True
        self._display_wait_start = time.time()

        while self._waiting_for_next and self.visualizer.running:
            # Handle events
            if not self.visualizer.handle_events():
                return False

            # Check for N key
            if self.visualizer.next_net_requested:
                self.visualizer.next_net_requested = False
                self._waiting_for_next = False
                break

            # Check auto-advance
            if self.auto_advance:
                elapsed = time.time() - self._display_wait_start
                if elapsed >= self.display_time:
                    self._waiting_for_next = False
                    break
                else:
                    remaining = self.display_time - elapsed
                    self.visualizer.status_message = f"Displaying route... ({remaining:.1f}s)"

            # Render while waiting
            self.visualizer.render()
            self.visualizer.tick()

        return self.visualizer.running

    def on_routing_complete(self, successful: int, failed: int, total_iterations: int) -> None:
        """Called when all routing is complete."""
        print(f"\n{'='*60}")
        print(f"All nets processed: {successful} successful, {failed} failed")
        print(f"Total iterations: {total_iterations}")

        if self.auto_advance:
            # Auto quit when done in auto mode
            self.visualizer.running = False
        else:
            print("Press Q to quit or close window")
            # Keep running until user quits
            while self.visualizer.running:
                if not self.visualizer.handle_events():
                    break
                self.visualizer.render()
                self.visualizer.tick()

        self.visualizer.quit()

    def should_pause(self) -> bool:
        """Check if routing should pause."""
        return self.visualizer.paused and not self.visualizer.step_mode

    def get_iterations_per_step(self) -> int:
        """Get number of iterations per visualization update."""
        if self.visualizer.step_mode:
            self.visualizer.step_mode = False
            return 1
        return self.visualizer.iterations_per_frame

    def is_running(self) -> bool:
        """Check if visualization is still running."""
        return self.visualizer.running


def create_pygame_callback(layers: List[str] = None, auto_advance: bool = False,
                           display_time: float = 0.0) -> PyGameVisualizationCallback:
    """Factory function to create a PyGame visualization callback.

    Args:
        layers: List of layer names for coloring
        auto_advance: If True, automatically advance to next net
        display_time: Seconds to display completed route before advancing

    Returns:
        Configured PyGameVisualizationCallback
    """
    config = VisualizerConfig()
    if layers:
        config.layers = layers
    return PyGameVisualizationCallback(config, auto_advance=auto_advance, display_time=display_time)
