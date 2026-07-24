# Board rendering & routing animation

A fast, dependency-light way to **look at a routed board** and to **watch the
router work** — every track and via being laid down, ripped up, and restored,
in order, from bare input to finished board (issue #482).

Unlike the KiCad-based renderer it replaces, none of this needs KiCad,
`kicad-cli`, an SVG step, or a headless browser. It rasterizes the parsed
geometry directly with [Pillow](https://python-pillow.org/), so a still is
~0.2 s and a full movie renders in about a second.

Three pieces:

| Tool | Role |
|------|------|
| `route_render.py` | **Renderer / viewer** — draw a board (segments, vias, pads, zones, outline) to a PNG straight from geometry. |
| `route_trace.py` | **Trace recorder** — with `KICAD_ROUTE_TRACE=1`, record every segment/via as it is committed, ripped, and restored. |
| `animate_route.py` | **Animator** — replay a trace (or a whole run) into an `.mp4` / `.gif` movie. |

---

## Quick start

```bash
# 1. Static image of any board (no trace needed)
python3 route_render.py board.kicad_pcb -o board.png

# 2. Record a trace while routing (default-OFF flag)
KICAD_ROUTE_TRACE=1 python3 route.py in.kicad_pcb out.kicad_pcb
#    -> writes out_routetrace.json next to the board

# 3. Movie of that routing step
python3 animate_route.py out_routetrace.json --board in.kicad_pcb -o routing.mp4

# 4. Movie of a WHOLE stress run (fanout -> diff -> planes -> signal -> repair)
python3 animate_route.py --run-dir runs_set1/myboard -o routing.mp4
```

In a movie, **new copper flashes white**, **reroutes / restores flash green**,
and **rips flash red** on the frame before they vanish. Copper layers are drawn
with per-layer transparency so overlaps blend at crossings.

---

## `route_render.py` — the renderer / viewer

```bash
python3 route_render.py BOARD.kicad_pcb [-o OUT.png] [--size 1600]
    [--supersample 2] [--layer-alpha 150] [--no-pads] [--no-zones]
    [--layers F.Cu,B.Cu]
```

- `--size` — longest image dimension in px. `--supersample` anti-aliases (1 = fastest, 2 = crisp stills).
- `--layer-alpha` — per-layer copper opacity 1–255; `<255` blends overlapping layers at crossings, `255` = opaque.
- `--layers` — render only these copper layers (per-layer views).

Library API (also the substrate for the animator):

```python
from kicad_parser import parse_kicad_pcb
from route_render import BoardRenderer

r = BoardRenderer(parse_kicad_pcb("board.kicad_pcb"))
r.render().save("board.png")                       # whole board
r.frame(segments=subset, vias=[]).save("f.png")    # an arbitrary subset (a frame)
```

`BoardRenderer` computes the world→pixel transform and the static substrate
(outline, zones, pads) **once**; each `frame(...)` composites a chosen subset of
copper on top, with optional highlight colors — which is exactly what the
animator drives.

---

## `route_trace.py` — the trace (`KICAD_ROUTE_TRACE=1`)

Setting `KICAD_ROUTE_TRACE=1` makes each routing front-end drop a sibling
`<output>_routetrace.json` — a time-ordered log of the copper it added, ripped,
and restored. It is **default-off**, read-only over the router (never changes
the routed result), and cheap.

**Granularity by step** — how finely each step is animated:

| Step | Front-end | Granularity |
|------|-----------|-------------|
| Signal routing | `route.py` | **Individual** — every commit / rip / restore, in true order |
| Differential pairs | `route_diff.py` | **Individual** — same choke points |
| Plane creation | `route_planes.py` | Per-**plane** taps + the pour **fills in** on the frame it is created |
| Plane repair | `route_disconnected_planes.py` | **Individual** per-join and per-rip, in order |
| BGA fanout | `bga_fanout.py` | Coarse — no trace; shown as the step's board delta |

Signal and diff-pair copper flows through the two `pcb_modification.py` choke
points (`add_route_to_pcb_data` / `remove_route_from_pcb_data`), so it is traced
per segment/via automatically. The plane front-ends add copper outside those
choke points, so they snapshot-diff `pcb_data` (repair) or emit from their tap
dicts (creation); a nested reconnect `batch_route` never double-records.

Not shown per-region: **voronoi fill sub-regions**. The router chain's boards do
not store computed zone fills, so a plane pours in as one shape at its creation
step, not region by region.

---

## `animate_route.py` — the animator

Two modes:

```bash
# Single trace over one board
python3 animate_route.py TRACE.json --board BOARD.kicad_pcb -o out.mp4

# Whole run: every stepN_*.kicad_pcb board's copper delta in chain order,
# with fine rip/restore animation spliced in for steps that recorded a trace
python3 animate_route.py --run-dir RUNDIR -o out.mp4
```

Key options: `--size`, `--fps`, `--layer-alpha`, `--rip-hold` (frames to hold a
rip red), `--end-hold` (seconds on the final frame), `--chunks` (reveal batches
for an untraced step), `--png-dir` (also dump raw PNG frames for external
encoding).

The whole-run mode relies on step outputs being named `stepN_<what>.kicad_pcb`;
it orders steps by the leading `stepN` and uses the last as the final board.

### Output format: `.mp4` vs `.gif`

The **output extension picks the format**:

- **`.mp4`** (H.264) — ~10–50× smaller than GIF, full color, plays natively in
  browsers / phones / Slack / social. Best for sharing and posting online.
  Needs `imageio` + `imageio-ffmpeg`:
  ```bash
  pip install imageio imageio-ffmpeg
  ```
  (bundles a static ffmpeg binary — no system install). If unavailable, the
  animator falls back to writing a sibling `.gif`.
- **`.gif`** (default) — native Pillow, **no dependency**, autoplays inline in
  chat / Markdown / GitHub issue bodies. Larger and 256-color.

---

## Stress-run integration (`tests/stress/render_run.py`)

Every stress run (live `run_board.sh` and no-LLM `redo_stress_test.py`) renders,
per board:

- `<final-board>.png` — combined snapshot, and `<final-board>_<layer>.png` per copper layer
- `<run-dir>/routing.mp4` — the whole-run movie (H.264 when `imageio-ffmpeg` is
  installed, else `routing.gif`). Chain boards are ordered by `stepN` when
  present, otherwise by write-time (for semantically-named chains).

`KICAD_ROUTE_TRACE=1` is exported by default in stress runs (set
`KICAD_ROUTE_TRACE=0` to skip; the movie then falls back to a coarse per-step
reveal). See the [stress-test runbook](../tests/stress/RUNBOOK.md#run-artifacts-final-snapshot--routing-movie-482).
