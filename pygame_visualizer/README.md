# PyGame PCB Routing Visualizer

Real-time visualization of the A* routing algorithm for PCB autorouting.

## Features

- **Real-time A* visualization** - Watch the Rust router explore the grid
- **Identical results to batch router** - Same Rust A* engine, same obstacle handling
- **Layer-based coloring** - Each copper layer has a distinct color
- **Persistent route display** - Completed routes remain visible
- **Interactive controls** - Pause, step, zoom, pan, adjust speed

## Installation

```bash
# pygame-ce (Community Edition) has better Python version support
pip install pygame-ce

# Or standard pygame
pip install pygame
```

The Rust router must also be built:

```bash
python build_router.py
```

## Usage

Add `--visualize` to any batch router command:

```bash
# Single net
python route.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_0)" --visualize

# Multiple nets with wildcards
python route.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --visualize

# Auto-advance through nets
python route.py input.kicad_pcb output.kicad_pcb "Net-(U2A-*)" --visualize --auto
```

## Command-Line Options

### Visualization Options

| Option | Default | Description |
|--------|---------|-------------|
| `--visualize`, `-V` | (disabled) | Enable real-time visualization |
| `--auto` | (disabled) | Auto-advance to next net |
| `--display-time` | `0.0` | Seconds to display completed route (with --auto) |

All standard batch router options are also supported (--ordering, --via-cost, etc.).

## Keyboard Controls

| Key | Action |
|-----|--------|
| Space | Pause/Resume |
| S | Single step (when paused) |
| N | Next net (after route completes) |
| R | Restart current net |
| Ctrl+R | Restart all nets |
| +/- | Double/Halve speed (1x to 65536x) |
| 1-4 | Show layer 1-4 only |
| 0 | Show all layers |
| G | Toggle grid lines |
| O | Toggle open set display |
| C | Toggle closed set display |
| H | Toggle legend |
| L | Toggle layer legend |
| Q/Esc | Quit |

## Mouse Controls

| Action | Effect |
|--------|--------|
| Scroll | Zoom in/out |
| Drag | Pan view |

## Color Legend

### Layers
- **Red** - F.Cu (Front copper)
- **Green** - In1.Cu (Inner layer 1)
- **Blue** - In2.Cu (Inner layer 2)
- **Magenta** - B.Cu (Back copper)

### Search State
- **Bright colors** - Open set (cells in priority queue)
- **Dark colors** - Closed set (already explored)
- **Yellow** - Current node being expanded
- **Cyan-green circles** - Source points
- **Red circles** - Target points

### Routes
- **Wide layer-colored lines** - Current net's route
- **Thin layer-colored lines** - Previously completed routes
- **White circles** - Vias

## Architecture

The visualizer is now integrated with the main batch router:

```
route.py --visualize
    │
    ├── routing logic (single_ended_routing.py, obstacle_map.py, etc.)
    │
    └── PyGameVisualizationCallback
            │
            └── RoutingVisualizer (pygame rendering)
```

### Module Structure

```
pygame_visualizer/
├── __init__.py          # Package exports
├── config.py            # Configuration and color schemes
├── visualizer.py        # PyGame rendering engine
├── callback.py          # Visualization callback interface
└── pygame_callback.py   # PyGame callback implementation
```

### Key Components

- **VisualizationCallback** - Protocol for visualization hooks
- **PyGameVisualizationCallback** - PyGame implementation of callback
- **RoutingVisualizer** - PyGame rendering engine

## Integration

To integrate visualization with custom code:

```python
from pygame_visualizer import create_pygame_callback
from route import batch_route

# Create callback
vis_callback = create_pygame_callback(
    layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
    auto_advance=False,
    display_time=0.0
)

# Run with visualization
batch_route(
    input_file, output_file, net_names,
    vis_callback=vis_callback
)
```

## Performance Notes

- The Rust router runs at full native speed between visualization updates
- Snapshots are created only when needed (configurable iterations per frame)
- Cell lists are limited to 50,000 entries to prevent memory issues
- Pre-rendered obstacle surface is cached for efficient rendering
- Results match batch router exactly
