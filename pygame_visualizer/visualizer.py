"""
PyGame-based real-time visualizer for PCB routing.

Renders the A* search progression in real-time using the Rust router with:
- Obstacles (tracks, pads, vias)
- BGA exclusion zones
- Open set (cells in priority queue)
- Closed set (explored cells)
- Current path being explored
- Final path when found
"""

import sys
from typing import List, Tuple, Optional, Set

try:
    import pygame
    from pygame import Surface, Rect
except ImportError:
    print("PyGame not installed. Install with: pip install pygame")
    sys.exit(1)

from .config import VisualizerConfig, LayerColors, SearchColors


class Camera:
    """2D camera for pan and zoom."""

    def __init__(self, screen_width: int, screen_height: int):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.offset_x = 0.0  # World offset in pixels
        self.offset_y = 0.0
        self.zoom = 1.0
        self.min_zoom = 0.1
        self.max_zoom = 10.0

        # Drag state
        self.dragging = False
        self.drag_start = (0, 0)
        self.drag_offset_start = (0.0, 0.0)

    def world_to_screen(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates."""
        sx = int((wx + self.offset_x) * self.zoom)
        sy = int((wy + self.offset_y) * self.zoom)
        return (sx, sy)

    def screen_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        """Convert screen coordinates to world coordinates."""
        wx = sx / self.zoom - self.offset_x
        wy = sy / self.zoom - self.offset_y
        return (wx, wy)

    def zoom_at(self, screen_x: int, screen_y: int, factor: float):
        """Zoom centered on screen position."""
        wx, wy = self.screen_to_world(screen_x, screen_y)
        new_zoom = max(self.min_zoom, min(self.max_zoom, self.zoom * factor))
        if new_zoom == self.zoom:
            return
        self.zoom = new_zoom
        self.offset_x = screen_x / self.zoom - wx
        self.offset_y = screen_y / self.zoom - wy

    def start_drag(self, screen_x: int, screen_y: int):
        """Start panning."""
        self.dragging = True
        self.drag_start = (screen_x, screen_y)
        self.drag_offset_start = (self.offset_x, self.offset_y)

    def drag(self, screen_x: int, screen_y: int):
        """Continue panning."""
        if self.dragging:
            dx = (screen_x - self.drag_start[0]) / self.zoom
            dy = (screen_y - self.drag_start[1]) / self.zoom
            self.offset_x = self.drag_offset_start[0] + dx
            self.offset_y = self.drag_offset_start[1] + dy

    def end_drag(self):
        """End panning."""
        self.dragging = False

    def fit_to_bounds(self, min_x: float, min_y: float, max_x: float, max_y: float, padding: float = 50):
        """Fit view to world bounds."""
        world_width = max_x - min_x
        world_height = max_y - min_y

        if world_width <= 0 or world_height <= 0:
            return

        zoom_x = (self.screen_width - 2 * padding) / world_width
        zoom_y = (self.screen_height - 2 * padding) / world_height
        self.zoom = min(zoom_x, zoom_y)
        self.zoom = max(self.min_zoom, min(self.max_zoom, self.zoom))

        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        self.offset_x = self.screen_width / (2 * self.zoom) - center_x
        self.offset_y = self.screen_height / (2 * self.zoom) - center_y


class RoutingVisualizer:
    """Main visualizer class using the Rust router."""

    def __init__(self, config: VisualizerConfig = None):
        """Initialize visualizer."""
        self.config = config or VisualizerConfig()

        pygame.init()
        pygame.display.set_caption(self.config.title)

        self.screen = pygame.display.set_mode(
            (self.config.window_width, self.config.window_height),
            pygame.RESIZABLE
        )
        self.clock = pygame.time.Clock()
        self.camera = Camera(self.config.window_width, self.config.window_height)

        # Fonts
        self.font = pygame.font.SysFont('consolas', 14)
        self.font_large = pygame.font.SysFont('consolas', 18)

        # State
        self.running = True
        self.paused = False
        self.step_mode = False
        self.restart_requested = False      # R = restart current net
        self.restart_all_requested = False  # Ctrl+R = restart all nets
        self.next_net_requested = False     # N = advance to next net
        self.backwards_requested = False    # B = try backwards direction
        self.iterations_per_frame = self.config.iterations_per_frame

        # Current net info for display
        self.current_net_name = ""
        self.current_net_num = 0
        self.total_nets = 0
        self.status_message = ""  # Additional status message (e.g., "Forward failed, press N for reverse")

        # Routing data (from Rust)
        self.rust_obstacles = None  # GridObstacleMap from Rust
        self.sources: List[Tuple[int, int, int]] = []
        self.targets: List[Tuple[int, int, int]] = []
        self.grid_step: float = 0.1
        self.bounds: Tuple[float, float, float, float] = (0, 0, 100, 100)

        # BGA zones for display
        self.bga_zones: List[Tuple[int, int, int, int]] = []

        # Current search snapshot (from Rust VisualRouter)
        self.snapshot = None

        # Pre-rendered surfaces
        self._obstacle_surface: Optional[Surface] = None
        self._obstacle_dirty = True
        self._obstacle_offset = (0, 0)
        self._obstacle_cell_size = self.config.cell_size  # Cell size used for obstacle surface

        # Blocked cells cache for rendering (from Python obstacle building)
        self._blocked_cells_cache: List[Set[Tuple[int, int]]] = []
        self._blocked_vias_cache: Set[Tuple[int, int]] = set()

        # Completed routes from previous nets
        # Each entry is a dict with 'path' (list of (gx, gy, layer)) and 'vias' (list of (gx, gy))
        self.completed_routes: List[dict] = []

    def set_routing_context(
        self,
        rust_obstacles,  # GridObstacleMap from Rust module
        sources: List[Tuple[int, int, int]],
        targets: List[Tuple[int, int, int]],
        grid_step: float = 0.1,
        bounds: Tuple[float, float, float, float] = None,
        bga_zones: List[Tuple[int, int, int, int]] = None,
        blocked_cells: List[Set[Tuple[int, int]]] = None,
        blocked_vias: Set[Tuple[int, int]] = None,
    ):
        """Set the routing context."""
        self.rust_obstacles = rust_obstacles
        self.sources = list(sources)
        self.targets = list(targets)
        self.grid_step = grid_step
        self.bga_zones = bga_zones or []
        self._blocked_cells_cache = blocked_cells or []
        self._blocked_vias_cache = blocked_vias or set()
        self._obstacle_dirty = True

        if bounds:
            self.bounds = bounds
            min_gx = int(bounds[0] / grid_step)
            min_gy = int(bounds[1] / grid_step)
            max_gx = int(bounds[2] / grid_step)
            max_gy = int(bounds[3] / grid_step)
            cell_size = self.config.cell_size
            self.camera.fit_to_bounds(
                min_gx * cell_size, min_gy * cell_size,
                max_gx * cell_size, max_gy * cell_size
            )

    def update_snapshot(self, snapshot):
        """Update the current search snapshot from Rust VisualRouter."""
        self.snapshot = snapshot

    def set_current_net(self, net_name: str, net_num: int, total_nets: int):
        """Set the current net being routed for status display."""
        self.current_net_name = net_name
        self.current_net_num = net_num
        self.total_nets = total_nets

    def zoom_to_current_net(self):
        """Zoom to fit the current net's sources and targets."""
        if not self.sources and not self.targets:
            return
        all_points = list(self.sources) + list(self.targets)
        if not all_points:
            return
        min_gx = min(p[0] for p in all_points) - 20  # Add padding
        max_gx = max(p[0] for p in all_points) + 20
        min_gy = min(p[1] for p in all_points) - 20
        max_gy = max(p[1] for p in all_points) + 20
        cell_size = self.config.cell_size
        self.camera.fit_to_bounds(
            min_gx * cell_size, min_gy * cell_size,
            max_gx * cell_size, max_gy * cell_size
        )

    def add_completed_route(self, path: List[Tuple[int, int, int]]):
        """
        Add a completed route to be drawn persistently.

        Args:
            path: List of (gx, gy, layer) tuples representing the routed path
        """
        if not path:
            return

        # Extract via positions (where layer changes)
        vias = []
        for i in range(len(path) - 1):
            gx1, gy1, layer1 = path[i]
            gx2, gy2, layer2 = path[i + 1]
            if layer1 != layer2:
                vias.append((gx1, gy1))

        self.completed_routes.append({
            'path': list(path),
            'vias': vias,
        })

    def clear_completed_routes(self):
        """Clear all completed routes (for restart all)."""
        self.completed_routes = []

    def remove_last_completed_route(self):
        """Remove the most recently added completed route (for restart current net)."""
        if self.completed_routes:
            self.completed_routes.pop()

    def grid_to_screen(self, gx: int, gy: int) -> Tuple[int, int]:
        """Convert grid coordinates to screen coordinates."""
        cell_size = self.config.cell_size
        wx = gx * cell_size
        wy = gy * cell_size
        return self.camera.world_to_screen(wx, wy)

    def _render_route_with_layer_colors(self, path: List[Tuple[int, int, int]],
                                         line_width: int, via_radius: int,
                                         alpha_factor: float = 1.0):
        """
        Render a route path with each segment colored by its layer.

        Args:
            path: List of (gx, gy, layer) tuples
            line_width: Width of the route lines
            via_radius: Radius of via circles
            alpha_factor: Color brightness factor (1.0 = full, <1.0 = dimmer for old routes)
        """
        if not path or len(path) < 2:
            return

        cell_size = self.config.cell_size
        half_cell = int(cell_size * self.camera.zoom // 2)

        # Draw segments between consecutive points
        for i in range(len(path) - 1):
            gx1, gy1, layer1 = path[i]
            gx2, gy2, layer2 = path[i + 1]

            sx1, sy1 = self.grid_to_screen(gx1, gy1)
            sx2, sy2 = self.grid_to_screen(gx2, gy2)
            sx1 += half_cell
            sy1 += half_cell
            sx2 += half_cell
            sy2 += half_cell

            if layer1 == layer2:
                # Same layer - draw segment with layer color
                color = LayerColors.get_layer_color_by_index(layer1)
                # Apply alpha factor for dimming old routes
                color = tuple(int(c * alpha_factor) for c in color)
                pygame.draw.line(self.screen, color, (sx1, sy1), (sx2, sy2), line_width)
            else:
                # Layer change - draw via
                via_color = (255, 255, 255) if alpha_factor >= 1.0 else (180, 180, 180)
                pygame.draw.circle(self.screen, via_color, (sx1, sy1), via_radius)

    def _get_completed_route_cells(self) -> Set[Tuple[int, int]]:
        """Get set of all grid cells covered by completed routes."""
        cells = set()
        for route_info in self.completed_routes:
            path = route_info['path']
            for gx, gy, layer in path:
                cells.add((gx, gy))
        return cells

    def _render_completed_routes(self):
        """Render all previously completed routes with bright layer colors."""
        # Draw previous routes with normal line width and bright colors
        old_line_width = max(2, int(3 * self.camera.zoom))
        old_via_radius = max(2, int(3 * self.camera.zoom))

        for route_info in self.completed_routes:
            path = route_info['path']
            # Use full brightness (alpha_factor=1.0) for previous routes
            self._render_route_with_layer_colors(path, old_line_width, old_via_radius, alpha_factor=1.0)

    def _render_obstacles_to_surface(self, skip_cells: Set[Tuple[int, int]] = None) -> Optional[Surface]:
        """Pre-render obstacles to a surface for efficiency.

        Args:
            skip_cells: Set of (gx, gy) cells to skip drawing (e.g., where routes exist)
        """
        min_gx = int(self.bounds[0] / self.grid_step) - 10
        min_gy = int(self.bounds[1] / self.grid_step) - 10
        max_gx = int(self.bounds[2] / self.grid_step) + 10
        max_gy = int(self.bounds[3] / self.grid_step) + 10

        grid_width = max_gx - min_gx + 1
        grid_height = max_gy - min_gy + 1
        cell_size = self.config.cell_size

        # Calculate required surface size and adjust if needed
        width = grid_width * cell_size
        height = grid_height * cell_size

        # Soft limit: reduce cell_size to fit within reasonable memory usage
        # Hard limit: pygame surfaces have maximum texture size (typically 16384)
        soft_limit = 8192  # Warn and reduce detail above this
        hard_limit = 16384  # Pygame/GPU texture limit

        max_dim = max(width, height)
        if max_dim > hard_limit:
            # Calculate minimum cell_size to fit within hard limit
            min_cell_size = max(1, hard_limit // max(grid_width, grid_height))
            if min_cell_size < 1:
                print(f"[VIS WARNING] Board too large to render obstacles "
                      f"({grid_width}x{grid_height} grid cells). Skipping obstacle layer.")
                return None
            cell_size = min_cell_size
            width = grid_width * cell_size
            height = grid_height * cell_size
            if not hasattr(self, '_size_warning_shown'):
                print(f"[VIS WARNING] Large board - reduced obstacle detail "
                      f"(cell_size {self.config.cell_size} -> {cell_size})")
                self._size_warning_shown = True
        elif max_dim > soft_limit:
            # Reduce cell_size to stay under soft limit for better performance
            scale = soft_limit / max_dim
            cell_size = max(1, int(cell_size * scale))
            width = grid_width * cell_size
            height = grid_height * cell_size

        if skip_cells is None:
            skip_cells = set()

        surface = Surface((width, height))
        surface.fill(SearchColors.BACKGROUND)

        # Render blocked cells from cache as gray outlines (skip cells with routes)
        layer_idx = self.config.current_layer
        num_layers = len(self._blocked_cells_cache)
        blocked_cell_color = (80, 80, 80)  # Gray for blocked cells

        if layer_idx >= 0 and layer_idx < num_layers:
            for gx, gy in self._blocked_cells_cache[layer_idx]:
                if (gx, gy) in skip_cells:
                    continue  # Skip cells with completed routes
                if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                    rect = Rect(
                        (gx - min_gx) * cell_size,
                        (gy - min_gy) * cell_size,
                        cell_size, cell_size
                    )
                    pygame.draw.rect(surface, blocked_cell_color, rect, 1)  # Outline only
        else:
            for layer in range(num_layers):
                for gx, gy in self._blocked_cells_cache[layer]:
                    if (gx, gy) in skip_cells:
                        continue  # Skip cells with completed routes
                    if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                        rect = Rect(
                            (gx - min_gx) * cell_size,
                            (gy - min_gy) * cell_size,
                            cell_size, cell_size
                        )
                        pygame.draw.rect(surface, blocked_cell_color, rect, 1)  # Outline only

        # Render blocked vias as small X marks (gray), skip cells with routes
        via_blocked_color = (70, 70, 70)  # Slightly darker gray for via X marks
        for gx, gy in self._blocked_vias_cache:
            if (gx, gy) in skip_cells:
                continue  # Skip cells with completed routes
            if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                x = (gx - min_gx) * cell_size
                y = (gy - min_gy) * cell_size
                # Draw X (two diagonal lines)
                pygame.draw.line(surface, via_blocked_color,
                               (x + 1, y + 1), (x + cell_size - 2, y + cell_size - 2), 1)
                pygame.draw.line(surface, via_blocked_color,
                               (x + cell_size - 2, y + 1), (x + 1, y + cell_size - 2), 1)

        # Render BGA zones as lighter gray outline (on top of blocked cells)
        bga_zone_color = (120, 120, 120)  # Lighter gray for BGA zones
        for zone in self.bga_zones:
            zmin_gx, zmin_gy, zmax_gx, zmax_gy = zone[:4]
            rect = Rect(
                (zmin_gx - min_gx) * cell_size,
                (zmin_gy - min_gy) * cell_size,
                (zmax_gx - zmin_gx + 1) * cell_size,
                (zmax_gy - zmin_gy + 1) * cell_size
            )
            pygame.draw.rect(surface, bga_zone_color, rect, 2)  # Outline only, width=2

        self._obstacle_offset = (min_gx, min_gy)
        self._obstacle_cell_size = cell_size  # Store the cell size used for this surface
        return surface

    def render(self):
        """Render the current state."""
        self.screen.fill(SearchColors.BACKGROUND)
        cell_size = self.config.cell_size

        # Render pre-cached obstacles (skip cells where completed routes exist)
        if self._obstacle_dirty or self.completed_routes:
            # Always re-render when we have completed routes to skip their cells
            skip_cells = self._get_completed_route_cells()
            self._obstacle_surface = self._render_obstacles_to_surface(skip_cells)
            if not self.completed_routes:
                self._obstacle_dirty = False

        if self._obstacle_surface and self.config.show_obstacles:
            min_gx, min_gy = self._obstacle_offset
            dest_x, dest_y = self.camera.world_to_screen(min_gx * cell_size, min_gy * cell_size)
            # Scale factor: convert from obstacle surface cell_size to display cell_size, then apply zoom
            # This handles large boards where obstacle surface uses reduced cell_size
            scale_factor = (cell_size / self._obstacle_cell_size) * self.camera.zoom
            scaled_size = (
                int(self._obstacle_surface.get_width() * scale_factor),
                int(self._obstacle_surface.get_height() * scale_factor)
            )
            if scaled_size[0] > 0 and scaled_size[1] > 0:
                scaled = pygame.transform.scale(self._obstacle_surface, scaled_size)
                self.screen.blit(scaled, (dest_x, dest_y))

        # Render completed routes from previous nets (thinner, dimmer)
        self._render_completed_routes()

        # Render search state from Rust snapshot
        if self.snapshot:
            snapshot = self.snapshot
            layer_filter = self.config.current_layer

            # Closed set
            if self.config.show_closed_set and snapshot.closed_cells:
                for gx, gy, layer in snapshot.closed_cells:
                    if layer_filter >= 0 and layer != layer_filter:
                        continue
                    sx, sy = self.grid_to_screen(gx, gy)
                    size = int(cell_size * self.camera.zoom)
                    if 0 <= sx < self.config.window_width + size and 0 <= sy < self.config.window_height + size:
                        color = LayerColors.get_layer_color_by_index(layer)
                        color = tuple(c // 3 for c in color)
                        rect = Rect(sx, sy, max(1, size), max(1, size))
                        pygame.draw.rect(self.screen, color, rect)

            # Open set
            if self.config.show_open_set and snapshot.open_cells:
                for gx, gy, layer in snapshot.open_cells:
                    if layer_filter >= 0 and layer != layer_filter:
                        continue
                    sx, sy = self.grid_to_screen(gx, gy)
                    size = int(cell_size * self.camera.zoom)
                    if 0 <= sx < self.config.window_width + size and 0 <= sy < self.config.window_height + size:
                        color = LayerColors.get_layer_color_by_index(layer)
                        color = tuple(min(255, c + 80) for c in color)
                        rect = Rect(sx, sy, max(1, size), max(1, size))
                        pygame.draw.rect(self.screen, color, rect)

            # Final path - draw with bold, layer-colored segments (wider than previous routes)
            if snapshot.path and len(snapshot.path) > 1:
                # Use wider lines and larger vias for current route
                current_line_width = max(4, int(6 * self.camera.zoom))
                current_via_radius = max(5, int(6 * self.camera.zoom))
                self._render_route_with_layer_colors(
                    snapshot.path, current_line_width, current_via_radius, alpha_factor=1.0
                )

            # Current node
            if snapshot.current:
                gx, gy, layer = snapshot.current
                sx, sy = self.grid_to_screen(gx, gy)
                size = int(cell_size * self.camera.zoom)
                rect = Rect(sx, sy, max(2, size), max(2, size))
                pygame.draw.rect(self.screen, SearchColors.CURRENT, rect)

        # Render sources and targets
        for gx, gy, _ in self.sources:
            sx, sy = self.grid_to_screen(gx, gy)
            size = max(4, int(cell_size * self.camera.zoom))
            pygame.draw.circle(self.screen, SearchColors.SOURCE,
                             (sx + size // 2, sy + size // 2), size // 2 + 2)

        for gx, gy, _ in self.targets:
            sx, sy = self.grid_to_screen(gx, gy)
            size = max(4, int(cell_size * self.camera.zoom))
            pygame.draw.circle(self.screen, SearchColors.TARGET,
                             (sx + size // 2, sy + size // 2), size // 2 + 2)

        self._render_ui()
        pygame.display.flip()

    def _render_ui(self):
        """Render UI overlay."""
        y_offset = 10

        if self.paused:
            status = "PAUSED (Space to resume, S to step)"
            color = (255, 200, 100)
        else:
            status = f"Running ({self.iterations_per_frame} iter/frame)"
            color = (100, 255, 100)

        text = self.font.render(status, True, color)
        self.screen.blit(text, (10, y_offset))
        y_offset += 20

        # Show current net being routed
        if self.current_net_name:
            net_status = f"Net [{self.current_net_num}/{self.total_nets}]: {self.current_net_name}"
            text = self.font.render(net_status, True, (150, 200, 255))
            self.screen.blit(text, (10, y_offset))
            y_offset += 20

        # Show additional status message (e.g., forward failed)
        if self.status_message:
            text = self.font.render(self.status_message, True, (255, 150, 100))
            self.screen.blit(text, (10, y_offset))
            y_offset += 20

        if self.snapshot and self.config.show_stats:
            snapshot = self.snapshot
            info_lines = [
                f"Iteration: {snapshot.iteration}",
                f"Open: {snapshot.open_count}",
                f"Closed: {snapshot.closed_count}",
            ]
            if snapshot.found:
                path_len = len(snapshot.path) if snapshot.path else 0
                info_lines.append(f"PATH FOUND! Length: {path_len}")
                info_lines.append("Press N for next net")

            for line in info_lines:
                text = self.font.render(line, True, (200, 200, 200))
                self.screen.blit(text, (10, y_offset))
                y_offset += 18

        # Layer legend (bottom-left)
        if self.config.show_layer_legend:
            y_offset = self.config.window_height - 100
            text = self.font.render("Layers (1-4, 0=all):", True, (200, 200, 200))
            self.screen.blit(text, (10, y_offset))
            y_offset += 18

            for i, layer_name in enumerate(self.config.layers):
                color = LayerColors.get_layer_color(layer_name)
                indicator = " *" if self.config.current_layer == i else ""
                if self.config.current_layer < 0:
                    indicator = ""
                text = self.font.render(f"  {i+1}: {layer_name}{indicator}", True, color)
                self.screen.blit(text, (10, y_offset))
                y_offset += 16

        # Speed indicator (top-right)
        speed_text = f"+/- Speed: {self.iterations_per_frame}x"
        text = self.font.render(speed_text, True, (150, 150, 150))
        self.screen.blit(text, (self.config.window_width - 160, 10))

        # Legend (right side)
        if self.config.show_legend:
            self._render_legend()

    def _render_legend(self):
        """Render the legend explaining what each visual element represents."""
        x_offset = self.config.window_width - 220
        y_offset = 40
        line_height = 18
        swatch_size = 12

        # Background box for legend
        legend_height = 340  # Fits all sections
        legend_rect = Rect(x_offset - 10, y_offset - 5, 220, legend_height)
        pygame.draw.rect(self.screen, (40, 40, 45), legend_rect)
        pygame.draw.rect(self.screen, (80, 80, 85), legend_rect, 1)

        # Title
        text = self.font.render("Legend", True, (220, 220, 220))
        self.screen.blit(text, (x_offset, y_offset))
        y_offset += line_height + 5

        # Search state section
        text = self.font.render("Search State:", True, (180, 180, 180))
        self.screen.blit(text, (x_offset, y_offset))
        y_offset += line_height

        # Get current layer color for accurate legend
        layer_idx = self.config.current_layer if self.config.current_layer >= 0 else 0
        layer_color = LayerColors.get_layer_color_by_index(layer_idx)
        open_color = tuple(min(255, c + 80) for c in layer_color)
        closed_color = tuple(c // 3 for c in layer_color)

        # Source - circle
        center = (x_offset + 5 + swatch_size // 2, y_offset + 2 + swatch_size // 2)
        pygame.draw.circle(self.screen, SearchColors.SOURCE, center, swatch_size // 2)
        text = self.font.render("Source (start)", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        # Target - circle
        center = (x_offset + 5 + swatch_size // 2, y_offset + 2 + swatch_size // 2)
        pygame.draw.circle(self.screen, SearchColors.TARGET, center, swatch_size // 2)
        text = self.font.render("Target (goal)", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        # Current node - filled rect
        swatch_rect = Rect(x_offset + 5, y_offset + 2, swatch_size, swatch_size)
        pygame.draw.rect(self.screen, SearchColors.CURRENT, swatch_rect)
        text = self.font.render("Current node", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        # Open set - filled rect
        swatch_rect = Rect(x_offset + 5, y_offset + 2, swatch_size, swatch_size)
        pygame.draw.rect(self.screen, open_color, swatch_rect)
        text = self.font.render("Open set (frontier)", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        # Closed set - filled rect
        swatch_rect = Rect(x_offset + 5, y_offset + 2, swatch_size, swatch_size)
        pygame.draw.rect(self.screen, closed_color, swatch_rect)
        text = self.font.render("Closed set (explored)", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        y_offset += 5

        # Routes section
        text = self.font.render("Routes:", True, (180, 180, 180))
        self.screen.blit(text, (x_offset, y_offset))
        y_offset += line_height

        # Current route - thick line
        y_center = y_offset + 2 + swatch_size // 2
        pygame.draw.line(self.screen, layer_color,
                        (x_offset + 5, y_center), (x_offset + 5 + swatch_size, y_center), 4)
        text = self.font.render("Current route", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        # Previous route - thin line
        y_center = y_offset + 2 + swatch_size // 2
        dimmed_color = tuple(int(c * 0.6) for c in layer_color)
        pygame.draw.line(self.screen, dimmed_color,
                        (x_offset + 5, y_center), (x_offset + 5 + swatch_size, y_center), 2)
        text = self.font.render("Previous routes", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        # Via - circle
        center = (x_offset + 5 + swatch_size // 2, y_offset + 2 + swatch_size // 2)
        pygame.draw.circle(self.screen, (255, 255, 255), center, swatch_size // 2)
        text = self.font.render("Via", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        y_offset += 5

        # Obstacles section
        text = self.font.render("Obstacles:", True, (180, 180, 180))
        self.screen.blit(text, (x_offset, y_offset))
        y_offset += line_height

        # BGA exclusion zone - unfilled square
        swatch_rect = Rect(x_offset + 5, y_offset + 2, swatch_size, swatch_size)
        pygame.draw.rect(self.screen, (120, 120, 120), swatch_rect, 1)
        text = self.font.render("BGA exclusion zone", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        # Blocked cells - unfilled square
        swatch_rect = Rect(x_offset + 5, y_offset + 2, swatch_size, swatch_size)
        pygame.draw.rect(self.screen, (80, 80, 80), swatch_rect, 1)
        text = self.font.render("Blocked cells", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        # Blocked via positions - X mark
        x1, y1 = x_offset + 5, y_offset + 2
        x2, y2 = x1 + swatch_size, y1 + swatch_size
        pygame.draw.line(self.screen, (100, 100, 100), (x1, y1), (x2, y2), 2)
        pygame.draw.line(self.screen, (100, 100, 100), (x1, y2), (x2, y1), 2)
        text = self.font.render("Blocked via positions", True, (200, 200, 200))
        self.screen.blit(text, (x_offset + swatch_size + 12, y_offset))
        y_offset += line_height

        # Controls legend (bottom-right)
        self._render_controls_legend()

    def _render_controls_legend(self):
        """Render keyboard/mouse controls in bottom-right."""
        line_height = 16
        x_offset = self.config.window_width - 220

        controls = [
            ("Space", "Pause/Resume"),
            ("S", "Step (paused)"),
            ("N", "Next net"),
            ("R", "Restart net"),
            ("Ctrl+R", "Restart all"),
            ("+/-", "Speed 2x"),
            ("Z", "Zoom to net"),
            ("1-4", "Layer filter"),
            ("0", "All layers"),
            ("O/C", "Open/Closed set"),
            ("H", "Toggle legend"),
            ("Scroll", "Zoom"),
            ("Drag", "Pan"),
            ("Q/Esc", "Quit"),
        ]

        box_height = len(controls) * line_height + 25
        y_start = self.config.window_height - box_height - 10

        # Background box
        legend_rect = Rect(x_offset - 10, y_start, 220, box_height)
        pygame.draw.rect(self.screen, (40, 40, 45), legend_rect)
        pygame.draw.rect(self.screen, (80, 80, 85), legend_rect, 1)

        y_offset = y_start + 5

        # Title
        text = self.font.render("Controls", True, (220, 220, 220))
        self.screen.blit(text, (x_offset, y_offset))
        y_offset += line_height + 3

        for key, action in controls:
            key_text = self.font.render(f"{key}:", True, (150, 200, 150))
            action_text = self.font.render(action, True, (180, 180, 180))
            self.screen.blit(key_text, (x_offset + 5, y_offset))
            self.screen.blit(action_text, (x_offset + 70, y_offset))
            y_offset += line_height

    def handle_events(self) -> bool:
        """Handle pygame events. Returns False if should quit."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    return False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_s:
                    if self.paused:
                        self.step_mode = True
                elif event.key == pygame.K_r:
                    mods = pygame.key.get_mods()
                    if mods & pygame.KMOD_CTRL:
                        # Ctrl+R = restart all nets
                        self.restart_all_requested = True
                    else:
                        # R = restart current net
                        self.restart_requested = True
                    self.snapshot = None
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    # 2x speed increase
                    self.iterations_per_frame = min(
                        self.config.max_speed,
                        self.iterations_per_frame * 2
                    )
                elif event.key == pygame.K_MINUS:
                    # 2x speed decrease (halve)
                    self.iterations_per_frame = max(
                        self.config.min_speed,
                        self.iterations_per_frame // 2
                    )
                elif event.key == pygame.K_g:
                    self.config.show_grid_lines = not self.config.show_grid_lines
                    self._obstacle_dirty = True
                elif event.key == pygame.K_o:
                    self.config.show_open_set = not self.config.show_open_set
                elif event.key == pygame.K_c:
                    self.config.show_closed_set = not self.config.show_closed_set
                elif event.key == pygame.K_p:
                    self.config.show_proximity = not self.config.show_proximity
                elif event.key == pygame.K_l:
                    self.config.show_layer_legend = not self.config.show_layer_legend
                elif event.key == pygame.K_h:
                    self.config.show_legend = not self.config.show_legend
                elif event.key == pygame.K_n:
                    self.next_net_requested = True
                elif event.key == pygame.K_b:
                    self.backwards_requested = True
                elif event.key == pygame.K_0:
                    self.config.current_layer = -1
                    self._obstacle_dirty = True
                elif pygame.K_1 <= event.key <= pygame.K_9:
                    layer = event.key - pygame.K_1
                    if layer < len(self.config.layers):
                        self.config.current_layer = layer
                        self._obstacle_dirty = True
                elif event.key == pygame.K_z:
                    self.zoom_to_current_net()

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.camera.start_drag(*event.pos)
                elif event.button == 4:
                    self.camera.zoom_at(*event.pos, 1.2)
                elif event.button == 5:
                    self.camera.zoom_at(*event.pos, 1 / 1.2)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.camera.end_drag()

            elif event.type == pygame.MOUSEMOTION:
                if self.camera.dragging:
                    self.camera.drag(*event.pos)

            elif event.type == pygame.VIDEORESIZE:
                self.config.window_width = event.w
                self.config.window_height = event.h
                self.camera.screen_width = event.w
                self.camera.screen_height = event.h

        return True

    def should_step(self) -> bool:
        """Check if we should advance the search."""
        if not self.paused:
            return True
        if self.step_mode:
            self.step_mode = False
            return True
        return False

    def tick(self):
        """Advance frame timing."""
        self.clock.tick(self.config.target_fps)

    def quit(self):
        """Clean up pygame."""
        pygame.quit()
