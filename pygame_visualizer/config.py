"""
Configuration and color schemes for the routing visualizer.
"""

from dataclasses import dataclass, field
from typing import Dict, Tuple, List


# RGB color tuples
Color = Tuple[int, int, int]


class LayerColors:
    """Color scheme for PCB layers."""

    # Layer colors (matches KiCad conventions)
    F_CU = (200, 50, 50)      # Red - Front copper
    IN1_CU = (50, 180, 50)    # Green - Inner 1
    IN2_CU = (50, 50, 200)    # Blue - Inner 2
    B_CU = (180, 50, 180)     # Magenta - Back copper

    # Additional inner layers if needed
    IN3_CU = (200, 150, 50)   # Orange
    IN4_CU = (50, 180, 180)   # Cyan

    @classmethod
    def get_layer_color(cls, layer_name: str) -> Color:
        """Get color for a layer by name."""
        layer_map = {
            'F.Cu': cls.F_CU,
            'In1.Cu': cls.IN1_CU,
            'In2.Cu': cls.IN2_CU,
            'In3.Cu': cls.IN3_CU,
            'In4.Cu': cls.IN4_CU,
            'B.Cu': cls.B_CU,
        }
        return layer_map.get(layer_name, (128, 128, 128))

    @classmethod
    def get_layer_color_by_index(cls, idx: int, num_layers: int = 4) -> Color:
        """Get color for a layer by index."""
        colors = [cls.F_CU, cls.IN1_CU, cls.IN2_CU, cls.B_CU, cls.IN3_CU, cls.IN4_CU]
        if idx < len(colors):
            return colors[idx]
        return (128, 128, 128)


class SearchColors:
    """Color scheme for A* search visualization."""

    # Background and grid
    BACKGROUND = (30, 30, 35)
    GRID_LINE = (50, 50, 55)

    # Obstacles
    OBSTACLE = (60, 60, 65)
    PAD = (100, 100, 110)
    VIA_BLOCKED = (80, 60, 80)
    BGA_ZONE = (80, 40, 40)

    # Search state
    OPEN_SET = (100, 150, 200)      # Light blue - cells in priority queue
    CLOSED_SET = (70, 70, 80)       # Dark gray - already explored
    CURRENT = (255, 255, 100)       # Yellow - current node being expanded

    # Path
    PATH_FOUND = (50, 255, 50)      # Bright green - final path
    PATH_WORKING = (255, 200, 50)   # Orange - backtrack preview

    # Endpoints
    SOURCE = (50, 255, 150)         # Cyan-green - source points
    TARGET = (255, 100, 100)        # Light red - target points
    STUB = (255, 150, 50)           # Orange - unrouted stub endpoints

    # Stub proximity field
    PROXIMITY_LOW = (60, 50, 50)
    PROXIMITY_HIGH = (120, 60, 60)


@dataclass
class VisualizerConfig:
    """Configuration for the routing visualizer."""

    # Window settings
    window_width: int = 1400
    window_height: int = 900
    title: str = "PCB Routing Visualizer"

    # Grid display
    cell_size: int = 8           # Pixels per grid cell
    show_grid_lines: bool = False  # Grid lines (slow for large grids)

    # View settings
    padding_mm: float = 2.0      # Padding around the routing area in mm
    auto_fit: bool = True        # Auto-fit view to routing area

    # Animation settings
    update_interval: int = 1     # Update display every N iterations
    target_fps: int = 60         # Target frame rate
    iterations_per_frame: int = 8  # A* iterations between display updates (power of 2 for 2x scaling)

    # Speed control (2x scale: 1, 2, 4, 8, 16, 32, ...)
    min_speed: int = 1           # Minimum iterations per frame
    max_speed: int = 65536       # Maximum iterations per frame (2^16)

    # Display options
    show_open_set: bool = True   # Show cells in open set (priority queue)
    show_closed_set: bool = True # Show already-explored cells
    show_proximity: bool = True  # Show stub proximity cost field
    show_obstacles: bool = True  # Show blocked cells
    show_layer_legend: bool = True
    show_stats: bool = True      # Show iteration count, etc.
    show_legend: bool = True     # Show color legend

    # Layer display
    layers: List[str] = field(default_factory=lambda: ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])
    current_layer: int = 0       # Which layer to display (or -1 for all)

    # Colors
    layer_colors: type = LayerColors
    search_colors: type = SearchColors


# Keyboard controls help text
CONTROLS_HELP = """
Keyboard Controls:
  Space     - Pause/Resume
  S         - Single step (when paused)
  N         - Next net (after route completes)
  B         - Force backwards direction (restart & try backwards)
  R         - Restart current net
  Ctrl+R    - Restart all nets
  +/-       - Double/Halve speed (1x, 2x, 4x...)
  1-4       - Show layer 1-4 only
  0         - Show all layers
  G         - Toggle grid lines
  O         - Toggle open set display
  C         - Toggle closed set display
  P         - Toggle proximity field
  L         - Toggle layer legend
  H         - Toggle color legend
  Q/Esc     - Quit

Mouse:
  Scroll    - Zoom in/out
  Drag      - Pan view
"""
