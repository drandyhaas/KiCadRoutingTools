# BGA Fanout Generator

Creates escape routing for BGA (Ball Grid Array) packages in KiCad PCB files.

## Features

- **Generic BGA support** - Works with any BGA package pitch and size
- **Any placement angle** - The grid/channel/escape logic is global-axis-based, so a
  BGA placed at a non-orthogonal angle (anything other than 0/90/180/270°) is routed
  by rigidly rotating the whole board into the footprint's own frame, running the
  pipeline there, and mapping the resulting tracks/vias back (issue #137). The transform
  is an isometry, so the result is identical to solving the un-rotated board; orthogonal
  placements skip it entirely and are unaffected. Foreign-pad clearance is exact, including
  for rotated rectangular neighbours (both the routing obstacle map and the explicit
  pad-collision test honour each pad's rotation).
- **Either face** - A BGA on the back fans out as the mirror of the same BGA on the
  front: the whole board is turned over in memory (every part to the other face, y
  mirrored about the board's centre line, every layer swapped, pad locals re-derived
  under the parser's convention), the pipeline runs on the part now on F -- the
  rotation transform above still applies after it -- and the copper is mirrored back
  (`bga_fanout/flip_frame.py`). Measured before this on a board and its mirror, 18 of
  51 escapes differed (gaps along a face assigned in the other order, one net on the
  other layer with an extra via); after it, none. A part on the front skips it entirely.
- **Differential pair routing** - P/N pairs routed together on same layer
- **Collision-free routing** - Automatic layer assignment to avoid conflicts
- **Multi-layer support** - Distributes routes evenly across available layers
- **Adjacent same-net optimization** - Connects neighboring pads on same net directly
- **Existing fanout detection** - With `--check-for-previous`, skips pads that already have fanouts
- **Spare pins skipped** - A net with one pad on the whole board (an NC ball) gets no escape; it would only burn a channel (`skipped_nc` in the summary)
- **Via management** - Adds vias only for SMD pads on non-top layers (through-hole pads connect all layers)
- **Under-pad escape for dense arrays** - An alternate engine (`underpad`, tried automatically by the default `auto`) that escapes fully-populated BGAs the channel router can't (see [Escape methods](#escape-methods))
- **Plane balls dropped to vias, or served by the pour** - see [Plane-ball drops](#plane-ball-drops-424)
- **Board rules honoured** - the board's per-layer `.kicad_dru` clearance rules (#498: the
  escape clearance is floored at the largest rule on the escape layers), the project's
  same-net-pad record (#581), and the fab floors and escalation policy every routing step
  shares (`--fab-tier` / `--escalation`, below). Net classes are not read: the escape runs
  at `--clearance` throughout
- **Staggered packages refused** - a staggered no-lead array (AQFN and friends) is not a
  ball grid: its rows read at half the real pitch. bga_fanout stops in a second and prints
  the `qfn_fanout.py ... --escape-method underpad --allow-via-in-pad` command that handles
  it (#500; `KICAD_ALLOW_STAGGERED_BGA=1` runs anyway)

## Usage

```bash
# Basic fanout
python3 py_router/bga_fanout.py kicad_files/input.kicad_pcb --component U3 --output kicad_files/output.kicad_pcb

# With net filter (include pattern)
python3 py_router/bga_fanout.py kicad_files/input.kicad_pcb --component U3 --output kicad_files/output.kicad_pcb --nets "*DATA*"

# With exclusion pattern (all nets except GND and VCC)
python3 py_router/bga_fanout.py kicad_files/input.kicad_pcb --component U3 --output kicad_files/output.kicad_pcb \
    --nets "*" "!GND" "!VCC"

# Differential pairs
python3 py_router/bga_fanout.py kicad_files/input.kicad_pcb --component IC1 --output kicad_files/output.kicad_pcb \
    --nets "*lvds*" --diff-pairs "*lvds*" --primary-escape vertical

# Specify layers
python3 py_router/bga_fanout.py kicad_files/input.kicad_pcb --component U3 --output kicad_files/output.kicad_pcb \
    --layers F.Cu In1.Cu In2.Cu In3.Cu B.Cu
```

## Net Pattern Syntax

The `--nets` option supports fnmatch-style wildcards (`*`, `?`) and exclusion patterns:

| Pattern | Description |
|---------|-------------|
| `*DATA*` | Nets containing "DATA" |
| `/*` | Nets starting with "/" (hierarchical) |
| `!GND` | Exclude net named "GND" |
| `!*VCC*` | Exclude nets containing "VCC" |
| `"*" "!GND" "!VCC"` | All nets except GND and VCC |

**Note:** Nets starting with "unconnected-" (KiCad pins not connected in schematic) are automatically excluded.

## Options

| Option | Description | Default |
|--------|-------------|---------|
| `--component`, `-c` | Component reference | Auto-detect BGA (the first one found) |
| `--output`, `-o` | Output PCB file | kicad_files/fanout_test.kicad_pcb |
| `--layers`, `-l` | Routing layers | F.Cu B.Cu |
| `--track-width`, `--width`, `-w` | Track width (mm) | 0.3 |
| `--clearance` | Track clearance (mm) | 0.25 |
| `--via-size` | Via outer diameter (mm) | 0.5 |
| `--via-drill` | Via drill size (mm) | 0.3 |
| `--nets`, `-n` | Net patterns to include | All nets |
| `--diff-pairs`, `-d` | Differential pair patterns | None |
| `--diff-pair-gap` | Gap between P/N traces (mm) | 0.1 |
| `--exit-margin` | Distance past BGA boundary (mm) | 0.5 |
| `--primary-escape`, `-p` | Primary escape direction | horizontal |
| `--force-escape-direction` | Only use primary direction | False |
| `--rebalance-escape` | Rebalance escape directions | False |
| `--check-for-previous` | Skip existing fanouts | False |
| `--no-inner-top-layer` | Prevent inner pads from using F.Cu | False |
| `--escape-method` | `auto`, `channel`, `underpad` (dense arrays) or `dogbone` (gap vias); see [Escape methods](#escape-methods) | auto |
| `--plane-drop` | `auto` = drop a via for every excluded plane-net ball after the signal escape (#424); `off` disables. `KICAD_FANOUT_PLANE_DROP=0/1` overrides either | auto |
| `--plane-net-layers` | The future plane plan, `NET:LAYER[,LAYER...]` per plane net (e.g. `GND:In1.Cu,In4.Cu P3.3V:In2.Cu`): a ball the declared pour will reach on its own layer gets no drop via (see [Plane-ball drops](#plane-ball-drops-424)) | none |
| `--same-net-pad-clearance` | `> 0` forbids via-in-pad: under-pad escapes and plane drops run dog-bone, a ball with no legal off-pad site fails visibly, and the value is recorded in the sibling `.kicad_pro` for later steps (#581). `-1` allows via-in-pad | the project's record, else allowed |
| `--grid-step` | Routing grid step (mm); escape stub ends are snapped to it (#149). Match the `--grid-step` you give `route.py` | 0.1 |
| `--layer-costs` | One cost per `--layers` entry, route.py semantics (#288): negative = no escape copper on that layer (e.g. a future plane layer), otherwise a weight in [1, 1000]; the channel engine fills cheaper layers first | none |
| `--add-teardrops` | Add teardrop settings to all pads and vias in the output | False |

The fab floor and escalation flags every routing step shares (`--fab-tier`,
`--fab-overrides`, `--escalation`, `--clearance-ceiling`, `--strict-sizes`) and
the post-run DRC-settings flags (`--no-fix-drc-settings`,
`--relax-drc-severities`, `--keep-thermal`, `--enable-used-layers`) are
described in [Configuration](../../docs/configuration.md#fab-tier-options). A
fan-out whose via cannot fit the pitch at the standard floor may escalate to the
advanced one under the default `--fab-tier auto`; it is warned and counted in
`JSON_SUMMARY.design_rules`.

### Sizing the escape via to the pitch (issue #158)

The channel engine runs one escape track down the **half-pitch** between adjacent
via columns, so the via, track, and clearance must fit it or **every** escape
grazes the neighbouring column's via by a few µm (a sub-clearance DRC violation
the fanout would otherwise report as a clean `failed: 0`). The budget, per array:

```
via_size  ≤  pitch − track_width − 2·clearance        (one escape track per channel)
via_size  ≥  via_drill + 2·min_annular_ring           (fab floor)
```

Example: at 0.8 mm pitch with `--track-width 0.127 --clearance 0.1`, the escape
via must be ≤ 0.473 mm — **Ø0.5 grazes (every escape ~13 µm short), Ø0.45 is
clean** (pair with `--via-drill 0.25` to keep a 0.1 mm annular ring). When handed
infeasible params, `bga_fanout` prints a `WARNING: escape via ... busts the
half-pitch budget` with the recommended maximum. The `plan-pcb-routing` skill
computes this automatically.

## Plane-ball drops (#424)

With any escape method, after the signal escape completes every SMD ball on a
**plane net** — a net excluded from `--nets` with ≥ 6 balls on the part, or an
excluded net that already owns a copper zone — is dropped to a via immediately:
a dog-bone via in a free inter-ball gap (the #128 site allocator, exact-checked),
falling back to a pad-clamped via-in-pad tap. The ball is not routed anywhere;
the plane poured later picks the via up at fill. This claims the tap while the
under-package space is still open, instead of leaving the plane step to push a
via through the finished ball field (the #360 failure class). Existing same-net
connections are respected, so re-runs are idempotent; per-net counts are
reported in `JSON_SUMMARY.plane_drop`. Disable with `--plane-drop off`;
`KICAD_FANOUT_PLANE_DROP=0/1` overrides the flag either way (the
recorded-manifest A/B switch). With `--same-net-pad-clearance > 0` there is no
via-in-pad fallback: a ball with no free gap is left for the route step.

**Pour-direct.** A ball needs no drop via at all when a pour of its own net
already reaches it on the ball's own copper layer (the main fill component, not
an islet): the fill connects it, as human boards do for rail balls under an
outer-layer flood. A ball the fill just misses gets a short surface track into
the fill instead of a via (#652). `--plane-net-layers` extends this to pours
that do not exist yet: each declared (net, layer) is modelled as a board-sized
zone filled against the copper present now, and a ball that predicted fill
reaches skips its via. The per-net report counts these as `pour`.

**A pour-served ball is a promise (#678).** Each one is recorded, keyed
`"REF.PAD"`, in the output's `kicad_routing_tools.pour_served_pads` record (the
sibling `.kicad_pro`, merged across a chain's fanout steps). The route step's
in-run plane finalize audits every promise after routing; see
[route-plane.md](../../docs/route-plane.md).

## Escape methods

`--escape-method` selects the fanout engine:

- **`auto`** (default) - runs `channel` first and, if it drops any ball,
  retries with `underpad` and keeps whichever escapes more (#288).

- **`channel`** - the 45°-stub + channel router. Each ball jogs to a
  routing channel between the ball rows and runs out to the boundary. Full
  differential-pair support. Best for sparse/medium BGAs and signal-grouped escapes.

- **`underpad`** - a grid escape for **dense, fully-populated arrays** (issue
  #122). The channel router confines every layer to the gaps *between* ball
  rows, so on a fully-populated array a few channels over-subscribe and the
  deepest balls can't escape (e.g. ulx3s 22×22 dropped ~23 balls). The under-pad
  method instead drops a **via in each signal ball's pad** and routes it
  *straight under the pad field* on an inner layer (SMD pads only block their own
  layer), jogging into a between-ball channel only to dodge a via. Near-edge
  balls escape via-less on the BGA's own layer, keeping the rim clear so the
  deeper balls can run underneath. Balls are routed deepest-first (inside-out).

- **`dogbone`** - the under-pad engine with the escape via in the **diagonal
  inter-ball gap** instead of in the pad (issue #128): ball → short 45° stub →
  staggered gap via → inner-layer run starting at the via. This is the standard
  hand-layout escape pattern (the human ulx3s board is 100% dog-bone): the
  ball-grid positions stay free of barrels on the inner layers, opening the
  routing streets a via-in-pad grid closes, and the via ring never bulges into
  a small neighbouring ball (the #267 graze class -- daisho U2 went 31 → 1
  pre-cap-opt violations, at identical 113/113 escapes). Every gap site is
  validated against exact geometry (drill hole-to-hole, foreign rings, locked
  pads, the stub itself); a ball with no free gap falls back to via-in-pad, so
  dog-bone never escapes fewer balls than `underpad`. Coupled diff-pair escapes
  use each ball's gap via the same way (partner legs are checked against the
  partner's via explicitly).

  Notes / when to use it:
  - Use it when `channel` reports dropped balls (`failed > 0`) on a dense array.
  - **Power/plane nets are skipped as escapes** - but every skipped plane ball
    still gets a **plane-drop via** (below), so excluding them with `--nets` is
    all the setup a later plane pour needs.
  - **Differential pairs are escaped coupled** when `--diff-pairs` is given
    (issue #182): each pair's P and N exit on the **same layer**, converged to
    the diff spacing and extended past the boundary, so `route_diff` picks them
    up. Edge pairs escape **via-free on the top layer** (broadside pairs straight
    off the boundary; stacked pairs *END-ON*, the trail ball bulging through the
    pad gap), which keeps the rim via-free so the **deeper stacked pairs escape
    END-ON on an inner layer** (via-in-pad, same layer for P/N) running underneath
    them. A pair that can't fit a coupled corridor falls back to single-ended
    (still connected, just not coupled). On `glasgow_revC` (BGA U30, 0.8 mm
    pitch) all 13 FPGA Z-pairs escape coupled and route_diff couples them.
  - Use a **small via and track** for the pitch - the escape needs one clean
    track between adjacent via-in-pads, roughly `via ≤ pitch − 2·track − 2·clearance`.
    For 0.8 mm pitch, `--via-size 0.35 --track-width 0.12 --clearance 0.1` works well.
  - Works for top- **and** bottom-side (B.Cu) BGAs - the via-less edge escape
    runs on the BGA's own layer.
  - **Movable passives' pads are soft keep-outs** (#278): escapes prefer to
    detour around foreign C/R/FB pads in the region (a wide cap between two
    escape legs can't be moved anywhere by the cap-placement step) and only
    graze one when boxed in - `place_fanout_clearance.py` then moves the cap.
    Keep-out disks are stamped against the pad/via's true coordinate, so a
    clear grid cell genuinely guarantees the clearance (no more few-µm
    VIA-SEGMENT quantization grazes).
  - **Existing tracks are obstacles whichever way they were drawn** (#1315):
    a board segment counts when its extent, grown by its keep-out, reaches the
    escape window, so a foreign track running INTO the ball field from outside
    blocks the escapes it crosses (on a layer this fanout routes) and the via
    sites it passes (on any copper layer, since a barrel spans the stack). The
    test used to be the segment's start point, which made such a track
    invisible or not depending on which end KiCad wrote first.

```bash
# Dense BGA that the channel router can't fully escape (diff pairs escaped coupled)
python3 py_router/bga_fanout.py board.kicad_pcb -c U1 -o out.kicad_pcb \
    --layers F.Cu In1.Cu In2.Cu B.Cu --diff-pairs "*" \
    --escape-method underpad --via-size 0.35 --track-width 0.12 --clearance 0.1
```

## Planned moves: following a caller's plan (full-move hints)

`generate_bga_fanout(..., escape_dir_hints=...)` takes, per pad (keyed by
board-frame pad position), either a bare face (`'left'`, `'right'`, `'up'`,
`'down'`) or a FULL planned move:

```python
{'face': 'down', 'exit': (x, y), 'layer': 'B.Cu',
 'kind': 'surface' | 'via_in_pad' | 'dogbone', 'site': (x, y) or None}
```

`exit` is a board point on the array's boundary line; only its coordinate
along the face is used (the gap). A rotated part's hints are transformed
into the routing frame with its pads (`rotate_frame.forward_transform`).
The channel engine reads the face of either form. The UNDER-PAD engine
(`escape_method='underpad'`; `'auto'` lets the channel engine take the
face first) follows a full move in its plan-follow phase, which lays the
planned balls to the most of their moves that can be achieved OVERALL:

* planned balls leave the generic phases; a `via_in_pad` ball's centre
  and a `dogbone` ball's asked site are reserved before any track is laid
  (the site validated exactly as the engine's own dog-bone chooser does:
  an inter-ball gap, clear of every registered copper and reservation,
  with a clear pad-to-site stub), the engine's own gap chosen when the
  asked one is not legal;
* every ball is routed to its EXACT move deepest-first: the A* takes a
  goal cell, the boundary cell at the asked gap, and may leave the window
  nowhere else; a `surface` ball on the top layer with no via, a
  `via_in_pad` ball diving at its pad to the asked layer, a `dogbone` ball
  running from its reserved site on the asked layer;
* a ball whose exact move is blocked NEGOTIATES: its blockers among the
  same call's committed escapes are found by routing the move on a
  snapshot of the occupancy taken before any commit, ripped, the ball
  laid exactly, the blockers re-laid, and the new state kept only if the
  score (escaped balls, then exact, then face+layer, then face, then
  fewer vias) rises -- at most 3 blockers per ball, 16 tries per call;
* what is still short degrades along the least damaging dimension: the
  nearest free gaps first (up to 6 pitches along the face, both ways),
  then the other layer/kind, then any face;
* the outcome is printed per ball (`Plan-follow: ... exact N, face+layer
  N, ...`, one line per ball that lost a dimension, the negotiation
  ledger) and returned in `pcb_data._fanout_plan_report` as
  `{net: {'asked', 'face', 'gap', 'layer', 'kind', 'got', 'level'}}`.

Balls with a bare-face hint, or none, run the unchanged phases. Two
engine facts a planner must respect, both measured: a via cannot sit in
a diagonal gap of a 0.65 mm pitch array unless it is small enough (0.45
fails by 5 um at 0.1 clearance; 0.25/0.15 fits), and a partial re-fan of
an already-fanned array must use the under-pad engine -- the channel
engine assigns channels without treating the unmoved nets' existing
stubs as occupants and lays new escapes over them.

## After fanout: optimize decoupling-cap placement (issue #130)

A fanout drops vias near the ball field. Where a foreign-net via lands under a
decoupling cap placed at a ball, the via copper overlaps the cap pad — a real
`PAD-VIA` DRC violation at the clearance floor. The fix is *placement*, so run
[`place_fanout_clearance.py`](../../py_placer/placement/README.md) on the **fanned** board
to nudge those caps clear and pull each pad toward its nearest same-net ball
(so a power/GND via dropped there later shares the via):

```bash
python3 py_placer/place_fanout_clearance.py out.kicad_pcb capclean.kicad_pcb --clearance 0.1
```

Use the same `--clearance` as the fanout. It only moves 2-pad caps near a BGA,
never overlaps caps, and is a no-op when nothing collides. The pass is
board-global (it reads every via and every BGA), so run it **once, after all
fanouts**, before signal routing: per-BGA runs compound the displacement. (GUI:
the "Optimize decoupling cap placement" checkbox on the BGA fanout tab does this
automatically.)

## Module Structure

```
bga_fanout/
├── __init__.py          # Main fanout logic and public API
├── types.py             # Data types: Channel, BGAGrid, DiffPairPads, FanoutRoute, create_track
├── escape.py            # Escape channel finding and assignment
├── reroute.py           # Collision resolution and rerouting
├── layer_balance.py     # Layer rebalancing for even distribution
├── layer_assignment.py  # Layer assignment for collision avoidance
├── underpad.py          # Under-pad grid escape engine (dense arrays, #122), plane drops
├── rotate_frame.py      # Non-orthogonal parts: route in the footprint's own frame (#137)
├── flip_frame.py        # Back-side parts: route the board turned over, mirror back
├── tracks.py            # Track generation and collision detection
├── geometry.py          # 45° stub and jog calculations
├── collision.py         # Low-level collision detection utilities
├── grid.py              # BGA grid analysis
├── diff_pair.py         # Differential pair detection
└── constants.py         # Configuration constants
```

### Key Components

- **`generate_bga_fanout()`** - Main function that generates fanout tracks
- **`generate_tracks_from_routes()`** - Converts FanoutRoute objects to track dictionaries
- **`find_differential_pairs()`** - Finds P/N pairs in a footprint
- **`assign_layers_smart()`** - Assigns routes to layers avoiding collisions
- **`rebalance_layers()`** - Rebalances routes across layers for even distribution
- **`detect_collisions()`** - Detects track spacing violations
- **`resolve_collisions()`** - Attempts to fix collisions via rerouting
- **`BGAGrid`** - Analyzes BGA pad grid geometry
- **`FanoutRoute`** - Represents a single pad's fanout route
- **`Channel`** - Routing channel between pad rows/columns

## Output

The fanned board is written to `--output` (passed through unchanged when there
is nothing to fan, so a chain always has a next input), and the run ends with a
`JSON_SUMMARY:` line: `requested` / `escaped` / `failed` / `unescaped_nets`
(balls actually attempted, so skipped plane balls and already-fanned nets are
not counted), `skipped_nc`, `clearance`, `track_width`, `layers`, `drc_grazes`
(a `check_drc` pass over the output at `--clearance`: `pad_via`,
`via_segment`, `pad_segment`, `segment_segment`, `total`; #130),
`min_clearance_used`, `deferred_fanout_nets` (nets deliberately left for the
route step; empty by default), `plane_drop` (per-net
`gap`/`in_pad`/`existing`/`pour`/`failed` counts and the promised
`pour_served_pads`), `env_knobs`, and `design_rules` when a feature was
delivered below its requested size. Unless `--no-fix-drc-settings`, the output's
`.kicad_pro` DRC floors are lowered to what the fanout used (#160), so the next
step and `check_drc` grade at it.

## Algorithm

1. **Grid Analysis** - Detect BGA pitch, grid size, and boundary
2. **Channel Calculation** - Identify routing channels between pad rows/columns
3. **Escape Direction Assignment** - Determine which edge each pad exits toward
4. **Adjacent Pad Connection** - Connect neighboring pads on same net directly
5. **Layer Assignment** - Distribute routes across layers to avoid collisions
6. **Track Generation** - Create 45° stubs, channel segments, and exit jogs
7. **Collision Resolution** - Reassign layers or reroute to resolve conflicts
8. **Layer Rebalancing** - Even out route distribution across layers
9. **Via Management** - Add vias for SMD pads on non-top layers
