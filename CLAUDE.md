# Project Notes for Claude

## Building the Rust Router

Use `build_router.py` to build the Rust router:

```bash
python build_router.py
```

This builds the Rust module, copies the library to the correct location, and verifies the version. Do not run `cargo build` directly.

**Important:** When making changes to the Rust router, bump the version in `rust_router/Cargo.toml` and update the version history in `rust_router/README.md`.

## KiCad Parser Usage

The project uses `kicad_parser` module to parse KiCad PCB files:

```python
from kicad_parser import parse_kicad_pcb, Pad, Footprint, PCBData

pcb = parse_kicad_pcb('path/to/file.kicad_pcb')
```

### PCBData Structure

- `pcb.footprints` - Dict[str, Footprint] keyed by reference (e.g., 'U9', 'R1')
- `pcb.nets` - Dict[int, Net] keyed by net_id
- `pcb.segments` - List of track segments
- `pcb.vias` - List of vias

### Footprint Attributes

- `footprint.reference` - Component reference (e.g., 'U9')
- `footprint.footprint_name` - Footprint library name (e.g., 'interf_u:PGA120')
- `footprint.pads` - List[Pad] of pads
- `footprint.x`, `footprint.y` - Footprint position
- `footprint.rotation` - Rotation in degrees
- `footprint.layer` - Layer (e.g., 'F.Cu')

### Pad Attributes

- `pad.pad_number` - Pad identifier (e.g., 'H2', '1')
- `pad.net_id` - Net ID (int)
- `pad.net_name` - Net name (e.g., '/PC-A7')
- `pad.global_x`, `pad.global_y` - Absolute position
- `pad.local_x`, `pad.local_y` - Position relative to footprint
- `pad.size_x`, `pad.size_y` - Pad dimensions
- `pad.shape` - 'circle', 'oval', 'rect', etc.
- `pad.layers` - List of layer names
- `pad.drill` - Drill diameter (0 for SMD, >0 for through-hole)
- `pad.component_ref` - Parent component reference
- `pad.pinfunction`, `pad.pintype` - Pin metadata

### Through-Hole vs SMD Pads

- Through-hole pads (`pad.drill > 0`) block tracks on ALL layers
- SMD pads (`pad.drill == 0`) only block their specific layer
- Even unconnected through-hole pads (net_id=0) physically block tracks

### Net Attributes

- `net.net_id` - Net ID (int)
- `net.name` - Net name string
- `net.pads` - List[Pad] of connected pads

### Segment (Track) Attributes

- `segment.start_x`, `segment.start_y` - Start point
- `segment.end_x`, `segment.end_y` - End point
- `segment.width` - Track width
- `segment.layer` - Layer name
- `segment.net_id` - Net ID

### Via Attributes

- `via.x`, `via.y` - Position
- `via.size` - Via outer diameter
- `via.drill` - Drill diameter
- `via.layers` - Layer span
- `via.net_id` - Net ID
