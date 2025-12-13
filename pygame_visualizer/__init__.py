"""
PyGame-based real-time visualizer for PCB routing algorithm.

This visualizer shows the A* search progression in real-time as it explores
the routing grid, using the same Rust router as the batch router.
"""

from .visualizer import RoutingVisualizer
from .config import VisualizerConfig, LayerColors

__all__ = ['RoutingVisualizer', 'VisualizerConfig', 'LayerColors']
__version__ = '1.0.0'
