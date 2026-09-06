# Board rendering & routing animation

A fast, dependency-light way to **look at a routed board** and to **watch the
router work** — every track and via being laid down, ripped up, and restored,
in order, from bare input to finished board (issue #482).

Unlike the KiCad-based renderer it replaces, none of this needs KiCad,
`kicad-cli`, an SVG step, or a headless browser. It rasterizes the parsed
geometry directly with [Pillow](https://python-pillow.org/), so a still is
~0.2 s and a full movie renders in about a second.

That is still true of everything below **by default**, and it is the reason this
subsystem exists. One opt-in feature does need `kicad-cli` — the 3D isometric
panel (#887) — and it is off unless asked for, costs ~2-3 s per render when it
is, and degrades to the ordinary single-panel movie, at full speed and with a
stated reason, when the binary is absent. See
[the 3D isometric panel](#the-3d-isometric-panel-and-the-run-clock-887).

Three pieces:

| Tool | Role |
|------|------|
| `make_movie.py` | **Front door** — make the movie from whatever you have: a run directory, a list of step boards, or one board. |
| `route_render.py` | **Renderer / viewer** — draw a board (segments, vias, pads, zones, outline) to a PNG straight from geometry. |
| `route_trace.py` | **Trace recorder** — with `KICAD_ROUTE_TRACE=1`, record every segment/via as it is committed, ripped, and restored. |
| `animate_route.py` | **Animator** — the engine `make_movie.py` drives; also replays a single trace over a board. |

---

## Quick start

```bash
# 1. Static image of any board (no trace needed)
python3 py_router/route_render.py board.kicad_pcb -o board.png

# 2. Record a trace while routing (default-OFF flag)
KICAD_ROUTE_TRACE=1 python3 py_router/route.py in.kicad_pcb out.kicad_pcb
#    -> writes out_routetrace.json next to the board

# 3. Movie of that routing step
python3 py_router/animate_route.py out_routetrace.json --board in.kicad_pcb -o routing.mp4

# 4. Movie of a WHOLE run (fanout -> diff -> planes -> signal -> repair)
python3 py_router/make_movie.py runs_set1/myboard            # -> runs_set1/myboard/routing.mp4
```

---

## `make_movie.py` — making the movie (issue #506)

The stress harness renders one movie per run; this makes the same movie on
demand, from any set of boards you already have.

```bash
python3 py_router/make_movie.py RUNDIR                       # whole chain in a directory
python3 py_router/make_movie.py step1.kicad_pcb step2.kicad_pcb step3.kicad_pcb
python3 py_router/make_movie.py board.kicad_pcb              # one board draws its own copper
python3 py_router/make_movie.py RUNDIR -o out.mp4 --size 1400 --fps 8 --png
```

- **A directory** → its chain, ordered by `stepN` in the filename when present,
  otherwise by write time (the order the chain produced them).
- **Several boards** → animated in the order given (`step*.kicad_pcb` from a shell
  glob works; each board's *new* copper is the delta from the previous one).
- **One board** → its copper reveals in `--chunks` batches.

Any board with a sibling `<board>_routetrace.json` animates per segment/via
instead of as a coarse delta, so `KICAD_ROUTE_TRACE=1` on the routing steps buys
a much finer movie for free.

Options: `-o/--output` (the extension picks `.mp4` vs `.gif`), `--size`, `--fps`,
`--supersample`, `--layer-alpha`, `--rip-hold`, `--chunks`, `--end-hold`,
`--png-dir` (dump raw frames), `--png` (also write a full-resolution still of the
final board), `--quiet`.

**In the GUI:** tick **Make routing movie** in the Advanced options tab's *Debug*
section (default off). While it is on the plugin records what it routes and
writes the movie next to the board as `<board>_routing.mp4` (then `_routing_2`,
`_routing_3`, … — earlier movies are never overwritten), printing the path in
**green** in the Log tab:

- **Any single routing step** — Route, Route Diff Pairs, Create Planes, Repair
  Planes, Fanout — gets its own movie the moment it completes, animating just
  that step's new copper.
- **A plan run** from the AI tab (*Run Selected Steps* / *Run All Selected
  Steps*) gets **one** movie covering all of its steps together.

The baseline is the board as it stood when you ticked the box (and the end state
of each movie afterwards), so successive movies never re-animate copper an
earlier one already showed. Rendering happens off the UI thread. `run_plan.py --movie`
(see [plans from the command line](claude-skills.md#plans-from-the-command-line))
does the same for a headless run.

Library use (what the GUI calls):

```python
from make_movie import make_movie
path = make_movie(['runs_set1/myboard'], out='routing.mp4')   # or a board list
```

In a movie, **new copper flashes white**, **reroutes / restores flash green**,
and **rips flash red** on the frame before they vanish. Copper layers are drawn
with per-layer transparency so overlaps blend at crossings.

---

## `route_render.py` — the renderer / viewer

```bash
python3 py_router/route_render.py BOARD.kicad_pcb [-o OUT.png] [--size 1600]
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
| Plane repair | `repair_planes.py` | **Individual** per-join and per-rip, in order |
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
python3 py_router/animate_route.py TRACE.json --board BOARD.kicad_pcb -o out.mp4

# Whole run: every stepN_*.kicad_pcb board's copper delta in chain order,
# with fine rip/restore animation spliced in for steps that recorded a trace
python3 py_router/animate_route.py --run-dir RUNDIR -o out.mp4
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

## Placement movies: the camera (#431)

`make_movie.py` animates a `place_route_loop` work dir as well as a routing
chain. It detects one by the `loop_round{N}.json` sidecars the loop writes --
never by a `loop_round*.kicad_pcb` glob, because `--work-dir` defaults to the
output board's directory (which may hold unrelated boards) and mtime order would
animate a REJECTED round as though it had been kept. Only ACCEPTED rounds enter
the chain; the sidecar's `parent` is what makes the set a chain at all, since
round N follows the last accepted board rather than N-1.

```bash
python3 py_router/make_movie.py WORKDIR --camera auto -o placement.mp4
python3 py_placer/place_route_loop.py board.kicad_pcb out.kicad_pcb --route-args '...' --movie
KICAD_MOVIE_CAMERA=auto python3 py_router/make_movie.py WORKDIR     # env knob, same effect
```

The camera is **off by default**, everywhere. Every GUI movie is a routing
movie, so turning it on there would be regression risk for no gain; the env knob
exists so one variable covers the GUI recorder, `run_plan.py --movie` and the
stress renderer at once.

What it does: an establishing overview, a zoom to the parts a round actually
moved, a pan when the next round works elsewhere, and the moves play only after
the camera arrives. That last part is structural, not a timing guess --

> a frame either MOVES THE CAMERA or CHANGES THE BOARD, never both

-- so every transition happens over a frozen board, and a settle beat separates
arrival from action. Long moves dolly out through a waypoint and back in, since
a flat pan across a dense board at 6 fps is an unreadable smear. Nearly-identical
consecutive rounds produce NO camera move at all (hysteresis): they nudge the
same parts, and without it the camera vibrates while saying nothing.

Two implementation notes worth knowing:

* Part motion needs no new drawing code. `build_boards` already builds its
  renderer with `dynamic_zones=True`, which draws pads per frame from
  `renderer.pcb` -- so re-pointing that attribute animates the footprints. With
  `dynamic_zones=False` pads are baked into the static base and the same
  re-pointing does nothing.
* Views are letterboxed in WORLD space so every frame is the same size.
  `_write_mp4` fails on mixed sizes and the failure is SILENT: it is caught, and
  the whole movie degrades to a GIF.

Rotation snaps rather than tweening (quench rotations are 90-degree multiples and
rare); `--camera-budget SECONDS` caps the runtime, scaling camera shots first and
only touching the moves as a last resort.

---

## The 3D isometric panel, and the run clock (#887)

Two optional additions to the frame. **Both are off by default, and the fast
path above is unchanged when they are** — which matters here more than it
usually would, because this subsystem exists precisely because `kicad-cli` was
taken out of it, and one of these puts it back on an opt-in path.

### `--panels xray+iso` — a 3D view under the board

```bash
python3 py_router/make_movie.py WORKDIR --panels xray+iso -o routing.mp4
KICAD_MOVIE_PANELS=xray+iso python3 py_router/make_movie.py WORKDIR   # env knob, same effect
```

The X-ray board view keeps the full frame width and a `kicad-cli pcb render` is
stacked underneath it. Like the camera, it has no GUI control of its own: one
variable covers the GUI recorder, `run_plan.py --movie` and the stress renderer
at once.

**What it does NOT show is routing progress.** Copper sits under soldermask, so
the 3D view barely changes as tracks are laid. What it shows is the parts moving
across placement rounds, and the board turning: shot *k* of *K* is rendered at
`yaw0 + sweep·k/(K−1)`, one slow turn across the whole film. That sweep is what
makes the panel animated, and it is free — a different `--rotate` costs exactly
the same render.

Measured on KiCad 10.0.0, and each number shapes the design:

| | |
|---|---|
| one render, `--quality basic` | **~2–7 s**, machine- and load-dependent. The board matters less than the machine: within one quiet pass the spread across tigard, lvds, ulx3s (225 models) and glasgow_revC (224) is under 2x, with glasgow consistently slowest |
| `--quality high` | **4.8–6.9 s** at 640×480 with no `--floor`, against 3.3 s for `basic` on the same job. (#887's own table reports 12.6 s, but at 900×700 `--floor`, which is a different question) |
| 8 renders, serial vs 6 workers | **~2.4x**, e.g. 24.0 s vs 10.2 s |
| a 900×700 request returns | **872×672** |
| a 640×480 request returns | **616×448** — the same for two very different boards, and across an 8-step yaw sweep |

So: **one render per chain STEP, never per frame** (`--iso-max-renders`, default
24, caps it — a COUNT rather than a number of seconds, so the same chain
composes the same movie on a fast machine and a slow one), and **the returned
size is never trusted**. Every panel is letterboxed into a box the composer
chose, and the render is asked for at 1.15× that box so the fit downscales
rather than blurs.

**Component bodies depend on the board, and their absence is silent.**
`kicad_files/tigard.kicad_pcb` renders as a *bare board* — pads, mask,
silkscreen, no parts. Its 84 `(model …)` references are 81 `${KISYS3DMOD}` +
3 `${KIPRJMOD}`, and 82 of them name a `.wrl`, while KiCad 10 ships `.step`;
`-D KISYS3DMOD=…` does not fix it. `lvds_converter_dualclk` renders fully populated. `kicad-cli` says
nothing either way, so the panel counts what is actually on disk and captions
`3D models N/M`, adding `BARE BOARD` and the reason at zero — never an empty
green rectangle that reads as a bug.

Without `kicad-cli` you get the single-panel movie at full speed and a line
saying so, naming `$KICAD_CLI`. That is a whole-movie decision taken **once**,
before compositing, and it is made on a probe render rather than on the binary
merely existing — because after the first composed frame the height is fixed and
cannot change. A single failed render later keeps its box with the reason drawn
inside it, for the same reason: mixed frame sizes make `_write_mp4` degrade the
whole movie to GIF, silently.

### The run clock

A run wrapped in `tests/stress/tee_cmd.py` leaves a `cmd_timing.jsonl`, and when
the movie finds one beside the chain it draws a run-clock overlay bottom-left,
opposite the caption:

```
RUN CLOCK  +0:51:23 of 1:17:39
stage  R3-route
basis  cmd_timing.jsonl - 153 wrapped commands, mapped by mtime
at  2026-08-20T10:03:32Z
```

The basis is the **run** clock, and that is measured rather than stylistic: in
run 24 the wrapped commands account for 253.3 s of a 4658.7 s run — 5.4% — so a
clock driven by tool time would sit near zero for over an hour.

**It counts up, and there is deliberately no countdown.** One was implemented
and it was exact — the movie is built after the run, so `t1 − instant` is the
subtraction of two recorded facts, not a forecast. It came out anyway, because
exact is not the same as legible: a countdown *reads* as "time left in this
video" and *means* "time that remained in the run", and a 25-frame GIF that ends
in four seconds while showing `remaining 0:15:57` invites exactly that
misreading. `+0:51:23 of 1:17:39` says the same thing with no way to misread it,
and anyone who wants the other number can subtract. Removing it also deleted the
machinery it needed to be safe — a coverage predicate over whether the ledger
spanned the film, its shortfall message, and an exact-or-absent branch in both
the overlay and the metadata.

A frame is mapped to an instant by its board's **mtime** falling inside a
command's `[t_start, t_end]`: `tee_cmd` stamps `time.time()` and a file's mtime
is the same clock, and it runs commands serially, so at most one row can contain
one. On run 24 that resolves all 17 chain boards. argv matching is the fallback
(mtime does not survive copying a work dir), and it is only a fallback because
on that same run it picks the wrong command for **9 of the 17** chain boards,
five of them by more than a minute — a dry run that never wrote the board, a
checker that only read it, a step that exited 1, and six more.

**The `at` line is UTC, and that is the point.** The ledger's own `iso_start` is
*local time with no offset* (`tee_cmd` uses `time.localtime`), so it means
different things on different machines; `t_start` is epoch seconds, so UTC is a
total function of a recorded fact. `--png-dir` frames carry the same instant as
PNG text metadata:

| key | |
|---|---|
| `krt:utc` | absolute UTC instant of this frame — the one to read |
| `krt:run_started_utc` | when the run began, so elapsed is checkable from the PNG alone |
| `krt:elapsed_s`, `krt:elapsed_hms` | position in the run |
| `krt:step`, `krt:stage`, `krt:step_wall_s` | which beat, which command, and what that command cost |
| `krt:run_total_s`, `krt:tool_s`, `krt:outside_s` | the run's totals |
| `krt:clock_basis` | how this frame was placed: `mtime`, `argv`, `pre-run`, … |

A frame is therefore self-describing: it needs no ledger and no knowledge of the
machine that produced it to be placed in time, which is the whole reason the
block exists. There is no `krt:eta`, no `krt:progress` and no `krt:remaining_s`
— a prediction, a percentage and a countdown all invite being read as forecasts.

The same reader is a standalone report — the end-of-run timing audit that used
to be a hand-run watch subagent:

```bash
python3 -X utf8 py_router/cmd_timing.py WORKDIR          # step table, subtotals, totals
python3 -X utf8 py_router/cmd_timing.py WORKDIR --json
```
