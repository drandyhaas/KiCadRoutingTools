# PyGame PCB Routing Visualizer

Real-time visualization of the A* routing algorithm for PCB autorouting.

## Features

- **Real-time A* visualization** - Watch the Rust router explore the grid
- **Identical results to batch router** - Same Rust A* engine, same obstacle handling, same iteration counts
- **Layer-based coloring** - Each copper layer has a distinct color
- **Persistent route display** - Completed routes remain visible when routing subsequent nets
- **Interactive controls** - Pause, step, zoom, pan, adjust speed
- **Search state display** - See open set (frontier), closed set (explored), final path
- **Pad-only routing** - Routes nets with only pads (no stubs) directly between pads

## Installation

```bash
# pygame-ce (Community Edition) has better Python version support
pip install pygame-ce

# Or standard pygame (may not work on Python 3.13+)
pip install pygame
```

The Rust router must also be built (same as for the batch router):

```bash
cd rust_router && cargo build --release
cp target/release/grid_router.dll grid_router.pyd  # Windows
```

## Usage

From the autorouter directory:

```bash
python pygame_visualizer/run_visualizer.py input.kicad_pcb "Net-Name"
```

Or as a module:

```bash
python -m pygame_visualizer.run_visualizer input.kicad_pcb "Net-Name"
```

### Wildcard Patterns

Use `*` and `?` wildcards to match multiple nets. All matching nets will be routed sequentially:

```bash
# Route and visualize all 32 DATA nets
python pygame_visualizer/run_visualizer.py fanout_starting_point.kicad_pcb "Net-(U2A-DATA_*)"
```

Each successfully routed net is added as an obstacle for subsequent routes, just like the batch router.

### Auto-Advance Mode

Use `--auto` to automatically progress through all nets without waiting for the N key:

```bash
python pygame_visualizer/run_visualizer.py --auto fanout_starting_point.kicad_pcb "Net-(U2A-DATA_*)"
```

This is useful for benchmarking or batch visualization. The visualizer will quit automatically when all nets are processed.

### Disable BGA Exclusion Zones

Use `--no-bga-zones` to disable BGA exclusion zone blocking. This is needed for nets that connect pads inside BGA packages (like LVDS signals):

```bash
python pygame_visualizer/run_visualizer.py --no-bga-zones fanout_starting_point.kicad_pcb "*lvds*"
```

### Pad-Only Routing

The visualizer can route nets that have only pads (no stub segments). These are nets where no fanout stubs have been created yet - the router will connect the pads directly.

### Example

```bash
python pygame_visualizer/run_visualizer.py fanout_starting_point.kicad_pcb "Net-(U2A-DATA_0)"
```

## Command-Line Options

The visualizer supports the same command-line arguments as `batch_grid_router.py`:

```bash
python pygame_visualizer/run_visualizer.py input.kicad_pcb "Net-(U2A-*)" [OPTIONS]
```

### Visualizer-Specific Options

| Option | Default | Description |
|--------|---------|-------------|
| `--auto` | (disabled) | Automatically advance to next net (no waiting for N key) |
| `--display-time` | `0.0` | Seconds to display completed route before advancing (with --auto) |

### Ordering and Strategy Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ordering`, `-o` | `mps` | Net ordering: `mps`, `inside_out`, or `original` |
| `--no-bga-zones` | (disabled) | Disable BGA exclusion zone detection |
| `--layers`, `-l` | `F.Cu In1.Cu In2.Cu B.Cu` | Routing layers to use |

### Track and Via Geometry

| Option | Default | Description |
|--------|---------|-------------|
| `--track-width` | `0.1` | Track width in mm |
| `--clearance` | `0.1` | Clearance between tracks in mm |
| `--via-size` | `0.3` | Via outer diameter in mm |
| `--via-drill` | `0.2` | Via drill size in mm |

### Router Algorithm Parameters

| Option | Default | Description |
|--------|---------|-------------|
| `--grid-step` | `0.1` | Grid resolution in mm |
| `--via-cost` | `25` | Penalty for placing a via (in grid steps) |
| `--max-iterations` | `100000` | Max A* iterations before giving up |
| `--heuristic-weight` | `1.5` | A* heuristic weight (higher = faster but less optimal) |

### Stub Proximity Penalty

| Option | Default | Description |
|--------|---------|-------------|
| `--stub-proximity-radius` | `1.5` | Radius around stubs to penalize routing (mm) |
| `--stub-proximity-cost` | `2.0` | Cost penalty near stubs (mm equivalent) |

### Example with Options

```bash
# Route with auto-advance (MPS ordering is default)
python pygame_visualizer/run_visualizer.py --auto input.kicad_pcb "Net-(U2A-*)"

# Auto-advance with 2 second display of each completed route
python pygame_visualizer/run_visualizer.py --auto --display-time 2.0 input.kicad_pcb "Net-(U2A-*)"

# Route with custom via cost (fewer layer changes)
python pygame_visualizer/run_visualizer.py --via-cost 100 input.kicad_pcb "Net-(U2A-DATA_*)"

# Route LVDS nets through BGA areas
python pygame_visualizer/run_visualizer.py --no-bga-zones input.kicad_pcb "*lvds*"
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| Space | Pause/Resume |
| S | Single step (when paused) |
| N | Next net (after route completes), or try backwards if forward failed |
| B | Force backwards direction (restart track & route backwards) |
| R | Restart current net |
| Ctrl+R | Restart all nets from beginning |
| +/- | Double/Halve speed (1x to 65536x) |
| 1-4 | Show layer 1-4 only |
| 0 | Show all layers |
| G | Toggle grid lines |
| O | Toggle open set display |
| C | Toggle closed set display |
| P | Toggle proximity field |
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
- **Normal layer-colored lines** - Previously completed routes
- **White circles** - Vias (larger for current route)

### Obstacles
- **Light gray outline** - BGA exclusion zone
- **Gray outline** - Blocked cells (tracks, pads)
- **Gray X marks** - Blocked via positions

Note: Blocked cell/via indicators are hidden where completed routes exist for better visibility.

## Architecture

```
pygame_visualizer/
├── __init__.py          # Package exports
├── config.py            # Configuration and color schemes
├── visualizer.py        # PyGame rendering engine
├── run_visualizer.py    # Main entry point
└── README.md
```

### Key Classes

- **VisualRouter** (Rust) - A* router with `step()` method that returns search snapshots
- **SearchSnapshot** (Rust) - Contains iteration count, open/closed sets, path
- **RoutingVisualizer** - PyGame-based renderer with camera controls
- **Camera** - Pan/zoom handling

## How It Works

The visualizer uses the same Rust router as `batch_grid_router.py`, but with a different interface:

1. **VisualRouter** - A new Rust class that exposes incremental search:
   - `init(sources, targets, max_iterations)` - Initialize the search
   - `step(obstacles, num_iterations)` - Run N iterations, return snapshot
   - `is_done()` - Check if search is complete
   - `get_path()` - Get the final path

2. **SearchSnapshot** - Returned by `step()`, contains:
   - `iteration` - Current iteration count
   - `open_count`, `closed_count` - Set sizes
   - `open_cells`, `closed_cells` - Actual cell coordinates for rendering
   - `found` - Whether path was found
   - `path` - The final path (if found)

This approach has **zero impact on the normal batch router** - `GridRouter.route_multi()` is unchanged and runs at full speed.

## Performance Notes

- The Rust router runs at full native speed between visualization updates
- Snapshots are created only when needed (configurable iterations per frame)
- Cell lists are limited to 50,000 entries to prevent memory issues
- Pre-rendered obstacle surface is cached for efficient rendering
- **Incremental obstacle caching** - Base obstacle map is built once at startup (~0.3s), then cloned and updated per-net (~0.15-0.6s vs ~1.2s without caching)
- **Results match batch router exactly** - Verified on 47 U2A nets: 47/47 success, 1,073,811 total iterations

Both routers use identical obstacle handling:
- **Routed nets**: segments, vias, and pads added as obstacles
- **Unrouted nets**: segments (stubs), vias (if any), and pads added as obstacles
- **Stub proximity costs**: applied to all remaining unrouted nets

## Integration

To integrate with your own code:

```python
from grid_router import GridObstacleMap, VisualRouter
from pygame_visualizer import RoutingVisualizer, VisualizerConfig

# Build obstacle map (same as batch router)
obstacles = GridObstacleMap(num_layers=4)
# ... add blocked cells, vias, etc.

# Set up visualizer
config = VisualizerConfig(layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])
visualizer = RoutingVisualizer(config)
visualizer.set_routing_context(obstacles, sources, targets, ...)

# Initialize Rust router
router = VisualRouter(via_cost=500000, h_weight=1.5)
router.init(sources, targets, max_iterations=100000)

# Run with visualization
while not router.is_done():
    if visualizer.should_step():
        snapshot = router.step(obstacles, visualizer.iterations_per_frame)
        visualizer.update_snapshot(snapshot)
    visualizer.render()
    visualizer.tick()
    if not visualizer.handle_events():
        break
```
