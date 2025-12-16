"""
PyGame-based real-time visualizer for PCB routing algorithm.

This visualizer shows the A* search progression in real-time as it explores
the routing grid, using the same Rust router as the batch router.

Usage:
    python batch_grid_router.py input.kicad_pcb output.kicad_pcb "Net-*" --visualize
"""

from .visualizer import RoutingVisualizer
from .config import VisualizerConfig, LayerColors
from .callback import VisualizationCallback, NullVisualizationCallback, VisualizationData
from .pygame_callback import PyGameVisualizationCallback, create_pygame_callback

__all__ = [
    'RoutingVisualizer', 'VisualizerConfig', 'LayerColors',
    'VisualizationCallback', 'NullVisualizationCallback', 'VisualizationData',
    'PyGameVisualizationCallback', 'create_pygame_callback',
]
__version__ = '2.0.0'
