# Tests

This directory holds the repo's tests: about 770 standalone `test_*.py`
scripts (unit tests and CLI/board integration chains), the runner that drives
them (`run_all.py`), the GUI/CLI parity gates (`gui_parity/`, see
[its README](gui_parity/README.md)), checked-in fixtures (`fixtures/`), the
mutation batteries (`mutate_*.py`) and the real-world stress harness
(`stress/`).

## Running Tests

Every test is a plain script. Run one from the project root or from the tests
directory:

```bash
# From project root
python3 tests/test_fanout_and_route.py --all

# From tests directory
cd tests
python3 test_fanout_and_route.py --all
```

## Running the WHOLE suite

```bash
python3 tests/run_all.py                     # everything, 4 at a time, locally
python3 tests/run_all.py --fast              # unit only (skip CLI/board tests)
python3 tests/run_all.py 908                 # only files whose name contains "908"
python3 tests/run_all.py --list              # what would run, and its classification
python3 tests/run_all.py --shard 3/50        # one of 50 duration-balanced slices (0-based)
```

`--timeout S` is the per-test budget (default 600 s; a test may declare more,
see "Writing a test" below), `-j N` the parallelism (default 4, `1` = serial),
`--shard-by stride` splits by name instead of by measured time, and
`--durations-out FILE` writes each test's wall seconds. A test is
*integration* (skipped by `--fast`) when its source imports `run_utils` or uses
`subprocess`. Each test runs with `TMPDIR`/`TEMP`/`TMP` pointed at a scratch
dir of its own, removed when it ends; run a test directly to keep its temp
output.

The summary keeps four buckets apart: passed, failed, **timed out** (a
machine-speed fact, re-run it alone before calling it a failure, though it
still makes the exit non-zero) and **self-skipped** (exit 77 with a `SKIP:`
line: the test asserted nothing, and it is never counted as a pass).

### ...on Modal, fanned out

The suite is about 770 files and, last measured, roughly 40 minutes of one
laptop. `--shard I/N` splits it into N disjoint slices, and the Modal driver
runs them in parallel containers, so a full run takes about as long as its
slowest shard:

```bash
modal run tests/stress/modal_suite/run_all_modal.py               # 50 shards
modal run tests/stress/modal_suite/run_all_modal.py --shards 25
modal run tests/stress/modal_suite/run_all_modal.py --fast        # unit lane only
modal run tests/stress/modal_suite/run_all_modal.py --filters "908 910"  # those families
```

Its other options: `--timeout` (per test, default 600), `--jobs` (per
container, default 2), `--out-dir DIR` (every shard's log; default a temp dir,
printed), and `--write-durations` (below), which `--force-durations` lets
write from a red run.

Three things to know before you trust its output:

- **`run_all.py --shard` does the splitting**, so discovery and classification
  have ONE source of truth -- the driver never globs `test_*.py` itself. A
  local run and a 50-way fan-out therefore cover the same set. The slices are
  packed longest-first onto the least-loaded shard from the measured wall
  seconds in `tests/run_all_durations.json` (a new test is priced at the
  median of its kind; with no table it falls back to a strided split by
  name). `--write-durations` refreshes the table from a GREEN run; commit it
  when the suite's cost shape has moved. Each shard's banner says whether it
  was balanced, and on how many measured tests. No balancing beats the
  slowest single test, so a test of independent rows may declare
  `RUN_ALL_PARTS = N` and take `--part I/N`: it then runs as N units
  (`test_x.py[3/8]`), each sharded on its own.
- **The verdict is each shard's own exit code**, never the parsed counts. A
  container that OOMs prints no summary line at all, and a driver that decided
  on parsed counts would read that silence as zero failures. A shard that
  never reported fails the run and is named.
- **THIS image -- the SUITE image -- has no KiCad**, so every test needing
  pcbnew/wx self-skips (exit 77). Those are reported in their own bucket and
  are *not* passes -- and the wx/pcbnew parity gates in `tests/gui_parity/`
  are not collected by `run_all` at all, so they still need a local
  KiCad-python session.

  **Do not read that as "the cloud has no KiCad".** It is true of
  `modal_suite/run_all_modal.py` (plain `debian_slim`, no switch) and of
  nothing else. The STRESS app has carried KiCad since 2026-08-23:
  `cloud_replay_sets.py` passes `--with-kicad` **by default**
  (`--no-kicad` opts out), which sets `KICAD_SWEEP_WITH_KICAD=1` and builds
  `modal_sweep/modal_app.py` on `kicad/kicad:10.0.6`, proving `import pcbnew`
  and `kicad-cli version` during the image build so a missing binding kills
  the BUILD rather than quietly deadening the oracle legs. Note the two
  defaults differ: `modal_app.py` read on its own defaults the env var OFF,
  while the CLI you actually launch defaults it ON, and a KiCad wave labels
  itself `-kc` (`-kc-<tag>` when `KICAD_SWEEP_KICAD_IMAGE` picks another
  image).

  The suite image's self-skips have three different reasons, so read each
  `SKIP:` line rather than assuming:
  - **No KiCad.** Seven would be recovered by giving the suite image KiCad:
    `test_910_fill_for_delivery`, the two `test_1279_*_pcbnew` tests and
    `test_1450_chamfered_pad_pcbnew` (each wants KiCad's bundled python), and three that run a pcbnew/wx half under
    KiCad's python and self-skip without one (found reviewing #1272: such a
    half used to print a skip and pass, so it ran nowhere):
    `test_505_min_connection_clamp`, `test_505_gui_half_runs` and
    `test_engine_kwarg_parity_launch`.
  - **No `ortools`** (a local Python only). The `test_622_*` tests that plan
    with CP-SAT skip without it. `requirements.txt` lists it -- the bus step
    needs it (#1355) -- so the suite image installs it and runs them.

The image is a clean checkout of HEAD (reproducible, and you can keep editing
while it runs); `KICAD_SWEEP_DIRTY=1` ships the working tree instead and stamps
the provenance `+dirty`.

**The image has to be a FAITHFUL checkout, and that is harder than it looks.**
The first full cloud run reported 15 failures that all pass locally. None was a
code defect; every one was the image differing from a real working copy. They
are listed here because each is a trap that will come back:

| what was missing | what it broke |
|---|---|
| a git **index** | `run_utils.corpus_boards()` asks `git ls-files`, and `git archive` ships no `.git`, so it returned `[]` and 9 corpus-walking tests graded an EMPTY corpus instead of skipping |
| `Pillow` | `startup_checks.check_render_dependencies` raised; 3 render tests died |
| `pytest` | 2 tests imported it for fixtures (a test-only dep, correctly absent from `requirements.txt`). No test needs it now, so the image no longer installs it |
| the right **Python version** | 2 tests behave differently on 3.12 vs 3.13+ |

Three of those deserve spelling out:

- **Runtime deps come from `requirements.txt`**, never a hand-copied list. The
  sweep and route images pin numpy/scipy/shapely because that is the whole
  dependency set of the *routing* path; the suite is not that narrow.
- **The index is rebuilt from `git ls-files`, with `--force`** -- NOT
  `git add -A`. `add` honours `.gitignore`, and a file added before a matching
  rule stays tracked forever, so `-A` silently drops
  `kicad_files/interf_u_unrouted.kicad_pcb` and leaves 21 of 22 boards. The
  build asserts the count against the one measured on the host, so a short
  corpus fails the build instead of quietly shrinking what the tests grade.
- **The container's Python matches the interpreter you launched with.** This
  repo has already had a Python upgrade change *routing results* (`math.fsum`),
  so grading on a different one is not grading your code. Override with
  `KICAD_SUITE_PYTHON=3.12` if Modal has no image for yours.

The moral, which cost a full red run to learn: **when a cloud suite fails tests
that pass locally, suspect the image before the code.**

## Writing a test

A test is a standalone script ending `if __name__ == "__main__":
sys.exit(main())`: exit 0 passes, anything else fails. `run_all.py` runs it as
`python3 -X utf8 tests/test_x.py` from the repo root; there is no pytest.

- **A test that cannot run self-skips**: it prints a line starting `SKIP:`
  (with the colon) on stdout and exits 77. An exit 77 without that line is
  reported as a FAIL, because 77 is also an ordinary crash code.
- **Declarations `run_all` reads from the source** (never by importing):
  `RUN_ALL_TIMEOUT = <seconds>` gives the test its own budget above
  `--timeout`; `RUN_ALL_FAST_OK = True` keeps a test whose shelling-out is
  cheap (a `git ls-files`) in `--fast`; `RUN_ALL_PARTS = N` runs a test of
  independent rows as N units, each invoked with `--part I/N` (0-based) and
  sharded on its own.
- **`run_utils` helpers** (`from run_utils import ...`):
  - `run_step(argv, outputs=[...], findings_ok=False)` runs one CLI chain step
    and ASSERTS it worked: exit 0 (or 1 for a checker with `findings_ok`), no
    traceback in its output, and every path in `outputs` written by this step.
    The older `run(cmd)` asserts nothing.
  - `check(argv, refuse='<the reason>', code=N)` asserts a command refused
    FOR THE STATED REASON (`accept=True` asserts success instead). A command
    that died of something else -- a traceback, an `ImportError`, an argparse
    error, a missing file -- raises as a BROKEN TEST, never as a guard that
    held.
  - `evidence(path)` refuses an input that is not a real, non-empty file
    before a check reads it (a `<(echo ...)` process substitution is not a
    file on Windows).
  - `corpus_boards()` is the boards git tracks under `kicad_files/`; it is
    `[]` when git cannot answer, and the caller must then SKIP.
- **Mutation batteries** (`mutate_*.py`) call `preflight(__file__)` from
  `mutation_anchors` right after their row table, so a stale anchor refuses
  (exit 2) in one second instead of reporting BROKEN after the witnesses ran;
  `--verify-anchors` checks the anchors and stops. `python3
  tests/mutation_anchors.py` censuses every battery (`--battery NAME`,
  `--verbose`, `--repo-wide`). Prefer a single-line anchor for a `.md` target:
  a multi-line one can match differently on a CRLF checkout.

## Test Scripts

A selection: the end-to-end chain scripts and a few focused tests worth knowing
by name. Every other `test_*.py` says what it pins in its own docstring.

The chain scripts built on `run_utils.run` -- `test_fanout_and_route.py`,
`test_flat_hierarchy.py`, `test_sonde_u.py` -- print each step and the
checkers' verdicts but do not fail on a bad step, so read their output.
`test_kit_route.py` and `test_interf_u.py` assert every step (`run_step`).

### test_fanout_and_route.py - Full BGA Board Test

The most comprehensive chain, exercising BGA/QFN fanout, single-ended routing, differential pair routing, DDR routing with length matching, and power plane creation on `kicad_files/haasoscope_pro_max_test.kicad_pcb` (10 copper layers; signals route on up to 5, the planes go on In4.Cu/In5.Cu). Its outputs are written into `kicad_files/`.

```bash
# Run all stages (fanout, routing, checks)
python3 tests/test_fanout_and_route.py --all

# Quick mode (reduced scope for faster testing)
python3 tests/test_fanout_and_route.py --all --quick

# Unbuffered output (useful for real-time logging)
python3 tests/test_fanout_and_route.py --all -u

# Run only specific stages
python3 tests/test_fanout_and_route.py --ftdi --lvds    # Only FTDI and LVDS routing
python3 tests/test_fanout_and_route.py --ram --planes   # Only RAM routing and planes

# Run only DRC/connectivity checks (no routing)
python3 tests/test_fanout_and_route.py --onlychecks --ftdi --lvds --ram
```

**Options:**
| Option | Description |
|--------|-------------|
| `--all` | Run all stages: fanout, ftdi, lvds, ram, planes, and checks |
| `--quick` | Run quick test with reduced routing (fewer nets per stage) |
| `--fanout` | Run fanout tests (QFN and BGA fanout) |
| `--ftdi` | Run FTDI single-ended routing tests |
| `--lvds` | Run LVDS differential pair routing tests |
| `--ram` | Run DDR RAM routing tests |
| `--planes` | Run power/ground plane routing tests |
| `--checks` | Run DRC and connectivity checks after routing (the connectivity checks are skipped in --quick mode) |
| `--onlychecks` | Run only checks, skip all routing stages (requires stage flags like --ftdi to specify which checks) |
| `-u, --unbuffered` | Run python commands with unbuffered output |

**Pipeline stages:**
1. QFN fanout for U2 (ADC interface)
2. BGA fanout for FTDI (U3), ADC (IC1), FPGA (U3), and DDR (U1)
3. Single-ended routing (FTDI data lanes with swappable targets)
4. Differential pair routing (the 56 LVDS pairs at 100 Ω in two batches, with swappable targets)
5. DDR routing (DQS/CK diff pairs + DQ data lanes with auto byte-lane grouping)
6. GND and power planes on In4.Cu/In5.Cu
7. DRC and connectivity verification per stage

### test_kit_route.py - 4-Layer Pad-to-Pad Routing Test

Routes directly between pads without fanout on the kit-dev-coldfire-xilinx board. Demonstrates routing without BGA fanout, power plane creation, and disconnected plane repair. Outputs go to a fresh temp dir, never into `kicad_files/` (#426).

```bash
# Default: route only the /AN* nets (~30 s), and the three checkers must be clean
python3 tests/test_kit_route.py

# The full signal set (~10 min); the board does not route completely (#426),
# so the checkers' verdicts are printed, not asserted
python3 tests/test_kit_route.py --full

# Keep the generated boards for inspection
python3 tests/test_kit_route.py --workdir /path/to/dir

# Skip routing, only redo the planes on that earlier run's kit-out.kicad_pcb
python3 tests/test_kit_route.py --planes-only --workdir /path/to/dir
```

**Pipeline stages:**
1. Route the signals with power net width overrides
2. Create +3.3V and GND planes on all 4 layers
3. Connect disconnected plane regions (`repair_planes.py`)
4. DRC, connectivity and orphan-stub verification

### test_flat_hierarchy.py - 2-Layer Board with GND Plane

Simple 2-layer board test demonstrating GND plane creation and power net width assignments.

```bash
python3 tests/test_flat_hierarchy.py

# With unbuffered output
python3 tests/test_flat_hierarchy.py -u
```

**Pipeline stages:**
1. Create GND plane on B.Cu using `route_planes.py`
2. Route all signals with power net width overrides (GND/VCC at 1.0mm, VPP at 0.7mm)
3. DRC and connectivity verification

The test uses power net configurations generated by `/analyze-power-nets`.

### test_interf_u.py - 2-Layer Non-Rectangular Board

Tests routing on a 2-layer board with a non-rectangular board outline (has a bottom protrusion). Every step is asserted; outputs go to a fresh temp dir.

```bash
python3 tests/test_interf_u.py

# Keep the generated boards for inspection
python3 tests/test_interf_u.py --workdir /path/to/dir
```

**Pipeline stages:**
1. Create VCC and GND planes on both F.Cu and B.Cu
2. Fan out U9 (PGA120 package) nets
3. Route all signals with polygon-based board edge blocking
4. Connect disconnected plane regions
5. DRC and connectivity verification

### test_sonde_u.py - Route + GND Plane with Return Vias

Routes `kicad_files/sonde_u.kicad_pcb`, which has no `.kicad_pro`, so the router's own defaults apply (0.3 mm track, 0.25 mm clearance), then pours a GND plane with GND return vias. Its outputs are written into `kicad_files/`.

```bash
python3 tests/test_sonde_u.py

# Skip checks
python3 tests/test_sonde_u.py --no-checks

# With unbuffered output
python3 tests/test_sonde_u.py -u
```

**Pipeline stages:**
1. Route all nets at the defaults (`sonde_u_routed.kicad_pcb`)
2. GND plane on B.Cu with `route_planes.py --add-gnd-vias` (its default output, `sonde_u_routed_routed.kicad_pcb`)
3. DRC (at 0.2 mm) and connectivity on `sonde_u_routed.kicad_pcb`, the board from step 1

### test_guide_corridor.py - Guide-Corridor (Waypoint) Routing Test

Pass/fail tests for the guide-corridor feature (issue #7), where nets are routed
to follow a polyline drawn on a User layer. Unlike the other scripts, this one is
self-contained: it builds temporary boards from `kicad_files/flat_hierarchy.kicad_pcb`
by inserting `User.1` graphic lines, routes with/without `--guide-corridor`, and
asserts each scenario. It prints a `PASS`/`FAIL` line per scenario plus log detail,
and exits non-zero if any scenario fails.

```bash
python3 tests/test_guide_corridor.py        # needs the built Rust router (build_router.py)
python3 tests/test_guide_corridor.py -v      # verbose routing output
```

**Scenarios:**
| ID | Checks |
|----|--------|
| S0 | Regression — with the flag OFF, a guide on the board is ignored; the net routes its normal short path (guards against behavior change when the feature isn't requested) |
| S1 | Follow — a single net follows an arch drawn over it (track reaches the drawn peak), connects, no clearance errors |
| S2 | Blocked waypoint — a vertex placed on another net's through-hole pad is snapped to the nearest free cell; the net connects with no clip of that pad |
| S3 | Shared corridor — two nets follow the same arch, both connect, and occupy no shared copper cell (pack alongside, non-overlapping) |
| S4 | Never blocks — a pathological guide (unreachable off-board vertex) must not make a net fail; it routes and connects regardless (the unfollowable waypoint is dropped) |
| S5 | Real board — `kicad_files/lvds_converter_dualclk.kicad_pcb` has a 6-vertex `User.1` guide spanning the 3-pad `/CLK` net (the board from the issue #7 bug report). `/CLK` routes, connects, is DRC-clean, adds no vias the direct route didn't need, and passes near all guide vertices, distributed across both MST segments |
| S6 | Spacing subdivides — with `--guide-corridor-spacing 1.0`, `build_corridor_waypoints` subdivides the LVDS guide's long segments into denser, evenly-spaced waypoints (more points, no consecutive gap larger than the spacing) |
| S7 | Real board, spacing>0 — routing `/CLK` with `--guide-corridor-spacing 1.0` still connects, is DRC-clean, adds no extra vias, and follows the guide (exercises the subdivision path end-to-end) |

Pass/fail uses real clearance violations (to OTHER nets) plus connectivity / follow /
non-overlap. **Same-net self-crossings** are reported as a quality note, not a
failure: waypoint routing concatenates A* legs, so a multi-point tap or a
blocked-waypoint detour can cross an earlier leg of the same net. KiCad permits
same-net copper to overlap, so this is electrically harmless (the net stays
connected) — a route-quality nicety to improve later.

### test_keepout.py - Keepout-Zone Routing Test

Pass/fail tests for the keepout feature (issue #27), where routed tracks are kept
**out** of a closed polygon drawn on a User layer. Like `test_guide_corridor.py`,
it is self-contained: it builds temporary boards from
`kicad_files/flat_hierarchy.kicad_pcb` by inserting a `User.2` `gr_poly`, routes
with/without `--keepout`, and asserts each scenario. It prints a `PASS`/`FAIL` line
per scenario plus log detail, and exits non-zero if any scenario fails.

```bash
python3 tests/test_keepout.py        # needs the built Rust router (build_router.py)
python3 tests/test_keepout.py -v      # verbose routing output
```

**Scenarios:**
| ID | Checks |
|----|--------|
| K0 | Regression — with the flag OFF, a keepout polygon is ignored; the net cuts straight through the zone region (guards against behavior change when the feature isn't requested) |
| K1 | Hard avoid — a polygon straddling a net's straight path forces a detour; the net still connects, is DRC-clean, and no routed cell lies inside the polygon |
| K2 | Multi-net — two nets routed with the keepout enabled both connect and neither occupies a cell inside the zone |
| K3 | Real board — `kicad_files/lvds_converter_dualclk.kicad_pcb` with a `User.2` polygon across the `/CLK` net's path: `/CLK` routes, connects, is DRC-clean, and avoids the zone interior |

Unlike the guide corridor (best-effort), a keepout is a **hard** block: a zone that
walls off the only path to a pad will make that net fail — by design. Zones are
meant for open board area, not over pads.

### test_rule_area_keepout.py - KiCad Keep-out Rule-Area Test

Pass/fail tests for the *native* KiCad keepout (`(zone ... (keepout (tracks
not_allowed) (vias not_allowed) ...))`), as opposed to the user-drawn User-layer
keepout above. It has no enable flag — it is honored automatically whenever the
board contains a keep-out rule area, blocking only the disallowed item (tracks
and/or vias) on the keepout's listed layers. Self-contained: it inserts a keepout
zone into `kicad_files/flat_hierarchy.kicad_pcb` and reuses the `test_keepout.py`
harness.

```bash
python3 tests/test_rule_area_keepout.py
python3 tests/test_rule_area_keepout.py -v       # verbose routing output
```

**Scenarios:**
| ID | Checks |
|----|--------|
| R1 | Avoid — a rule area straddling a net's path forces a detour; the net connects, is DRC-clean, and no routed cell lies inside the polygon |
| R2 | Multi-net — two nets routed with rule areas present both connect and neither occupies a cell inside a rule area |
| R3 | No-op gating — a keepout that *allows* both tracks and vias is a no-op; the route cuts straight through (guards the tracks/vias-allowed gating, including the vias-not-allowed parse fix) |

### test_diff_pair_route.py - Differential-Pair Routing Correctness Test

A small, fast focused guard for the differential-pair router (`route_diff.py`),
separate from the big `test_fanout_and_route.py` pipeline. It routes individual
`/fpga_adc/lvds_rx1_NN` pairs on the post-fanout board
`kicad_files/routed_output.kicad_pcb` (each a 2-pad point-to-point on F.Cu) and
asserts each is **fully connected** and **DRC-clean**.

```bash
python3 tests/test_diff_pair_route.py
python3 tests/test_diff_pair_route.py -v
```

Each scenario routes one pair with `route_diff.py`, then checks `check_connected.py`
reports `ALL NETS FULLY CONNECTED` and `check_drc.py --clearance 0.1` reports no
violations. **DRC is scoped to each pair's own nets** (`--nets "*lvds_rx1_NN*"`):
the board carries pre-existing violations among other already-routed nets, so an
unscoped check would fail regardless — the same scoping `test_fanout_and_route.py`
uses for its LVDS stage. Three confirmed-clean pairs are checked
(`lvds_rx1_10/11/12`); ~9s total.

### test_multipoint_diff_route.py - Bare-Pad / Multi-Point Diff Pair Test

Routes the two diff pairs of `kicad_files/lvds_converter_dualclk.kicad_pcb`, a
simple 2-layer LVDS converter whose pairs terminate on bare pads (no fanout
stubs) at multiple terminals: `/DATA+`/`/DATA-` spans 3 pad-pair terminals
(connector, IC, termination resistor) and `/CLK+`/`/CLK-` spans 4. Asserts the
multi-point chain plan (legs passing through shared terminals), full
connectivity of **every** terminal, no pad-net swaps, and pair-scoped DRC
cleanliness, using the board's netclass geometry (0.2/0.25/0.2 mm).

```bash
python3 tests/test_multipoint_diff_route.py
python3 tests/test_multipoint_diff_route.py -v
```

### test_diff_connector_graze.py - Connector/Partner-Pad Graze Reject (issue #165)

Unit test for the diff-pair final-geometry guard `_connector_grazes_foreign_copper`
(foreign pads **and** vias).
The connector/setback segments are clearance-checked during routing against an
obstacle map that excludes **both** halves of the pair, so a connector can graze
the **partner** net's pad (USB-C, fine-pitch) unseen. The guard re-validates the
final geometry against foreign pads using the **same** geometry and tolerance as
`check_drc`, so it rejects a real graze (the tigard 0.164 mm `/USB_DP`↔`J1.A7`
short) without false-rejecting tight-but-legal packing that sits exactly at
clearance. Pure-geometry, no Rust router or board file needed.

```bash
python3 tests/test_diff_connector_graze.py
```

### test_diff_terminal_pairing.py - Diff-Pair Terminal Pairing (issue #165)

Unit test for `get_diff_pair_terminals`. The old greedy-by-distance matcher is
only locally optimal: a closest-first pick can strand leftover pads into a far
pairing. On a USB-C connector whose redundant DP/DN pads interleave (J1 y-order
B7,A6,A7,B6), greedy took `{A6,A7}`(0.5mm) first and was forced into
`{B6,B7}`(1.5mm) — the wide terminal drives a long diagonal connector that grazes
the partner pad. Minimum-total-cost matching yields `{A6,B7}+{B6,A7}` (0.5+0.5).
The test asserts the optimal pairing on both the scipy and the no-scipy
brute-force code paths (KiCad's bundled Python has no scipy).

```bash
python3 tests/test_diff_terminal_pairing.py
```

### test_diff_pad_end_launch.py - Connector Pad-Edge Launch (issue #165)

Unit test for `_pad_edge_launch`. Diff-pair connectors used to launch from the
pad **center**; on a long (1.45mm USB-C) pad that makes the connector cut back
across the pad field and graze a neighbour. `_pad_edge_launch` shifts the launch
to the pad **edge facing the route**. The test asserts it shifts toward the route
edge, stays inside the pad copper (so the track still attaches), never overshoots
a near route start, leaves non-pad (stub) launch points untouched, and handles
diagonal (`rect_rotation`) pads.

```bash
python3 tests/test_diff_pad_end_launch.py
```

### test_diff_relocation_fans.py - Relocation Fallback (issue #165)

Unit test for the multipoint terminal-relocation fallback. When a pair can't
launch on its pads' congested layer, the router relocates terminals to an open
inner layer by dropping a through-via per pad. Two guards: `_attach_fans` must
put the fan VIAS on a real leg result (the output is written from per-leg
`results`; segment-sync doesn't carry vias, so a via left only on the bookkeeping
`merged` dict is dropped and the relocated route floats with no via to its pad —
the tigard `/USB_D` "DRC-clean but disconnected" failure). `_fans_fit` rejects a
relocation whose per-pad vias can't fit (USB-C 0.5mm pitch + 0.5mm via collide),
so the pair falls through to single-ended instead of emitting via-via violations.

```bash
python3 tests/test_diff_relocation_fans.py
```

### test_diff_multipoint_escape_dir.py - Multipoint Escape Direction (issue #165)

Unit test for `get_pair_end_direction`. The multipoint diff-pair path passes the
PAD position to `get_stub_direction` (which expects the stub free end), so a
fanned-out pin's escape stub gave a REVERSED direction pointing into the chip
body — backing the coupled centerline setback into the pad field so the pair
couldn't route (the tigard `/USB_D` coupled-route failure). The fix orients the
stub-derived escape away from the component's pad centroid. The test asserts the
escape points outward whether called with the pad position or the stub free end.

```bash
python3 tests/test_diff_multipoint_escape_dir.py
```

### test_tigard_usb_diff.py - Real-World USB Diff Pair (issue #165)

End-to-end test on the **Tigard** board (`kicad_files/tigard.kicad_pcb`, a
QFN-64 FT2232H + USB-C connector) — the board that drove the #165 work. The USB
2.0 pair `/USB_DP`+`/USB_DN` is a fine-pitch connector fan-out (J1 has redundant
0.5mm-pitch DP/DN pads; U3.7/U3.8 sit on the congested QFN edge). Asserts that
`route_diff` **coupled-routes** the pair (not defers/floats/grazes it) DRC-clean,
then a single-ended `route.py` follow-up connects the redundant J1 row, leaving
**all USB pads fully connected with no DRC errors**. Runs the full pipeline under
three fan-out setups: **bare** (no fanout), **underpad-scoped** (`qfn_fanout` USB
pair only, `--escape-method underpad`), and **stub-non-scoped** (`qfn_fanout` all
U3 nets, default surface-stub escape).

```bash
python3 tests/test_tigard_usb_diff.py
python3 tests/test_tigard_usb_diff.py -v
```

### test_net_format_roundtrip.py - KiCad 9/10 net-format preservation

Unit test that the writers keep the output's net-token format consistent with the
**input** board: a KiCad-9 numeric board gets `(net <id>)` refs and a KiCad-10
board gets name-only `(net "name")` refs — even when the caller passes a
`net_id_to_name` map (the fanout tools always do). Guards against the regression
where fanout injected KiCad-10 name nets into a KiCad-9 board, producing a hybrid
file KiCad 9 reads as net-less. Covers `board_uses_name_nets` detection (v9 /
v10 / hybrid) and `add_tracks_and_vias_to_pcb` / `add_tracks_to_pcb` output.

```bash
python3 tests/test_net_format_roundtrip.py
```

### stress/ - Real-World Board Stress Test

Stress-tests the router against open-source boards downloaded from GitHub
(keyboards up to 6-layer SoC carriers), measuring routing completion rates
and DRC violations on each. Boards are **not** checked into the repo — the
harness downloads, format-normalizes (KiCad `pcbnew` round-trip), and strips
them (tracks/vias/pours removed) into an external corpus directory
(`$STRESS_DIR`, default `~/Documents/kicad_stress_test`). Each board is then
routed end-to-end following the plan-pcb-routing skill workflow (fanout →
diff pairs → signals → planes → repair → verification) and produces a
per-board results JSON.

```bash
cd tests/stress
python3 fetch_boards.py        # download corpus (needs gh auth)
# normalize + strip with KiCad's bundled python — see stress/README.md
python3 validate_boards.py     # corpus sanity check
```

Once the corpus exists, drive the whole run with the queue manager and watch
it with the status script:

```bash
bash run_queue.sh [max_concurrency=8] [model]   # keeps headless workers in flight
bash stress_status.sh                           # DONE/RUNNING/TODO + admit/hold
```

Above a floor of `QUEUE_CONC_MIN` workers (default 2), the queue admits a new
board only while the 1-minute load average is under `QUEUE_LOAD_MAX` (default:
the core count minus 2), and at most one a minute; `max_concurrency` is a
ceiling, not a target. The model defaults to the backend's own (`claude`: the
latest Sonnet).

See `tests/stress/README.md` for the full pipeline, the per-board run
procedure (`RUNBOOK.md`), the ~12 GB-per-step memory watchdog (`run_limited.sh`),
the **deterministic no-LLM replay** of a recorded run (`redo_stress_test.py` +
the command manifest the tools record — fast, reproducible, and the way
to A/B an engine change), and the list of kicad_parser issues the corpus
preparation currently works around.

The whole suite can also be driven by Claude Code with the `/stress-test-router`
skill (see `docs/claude-skills.md`): it prepares the corpus if missing, runs the
boards through the disk-driven queue manager (admitted by load, under the per-step memory cap),
aggregates the results, and drafts GitHub issues for new findings (filed only
after user approval).

### run_doc_examples.py - Python API Documentation Examples

Extracts every ` ```python ` block from `docs/python-api.md` and the
`docs/api-*.md` reference pages and runs each complete example against the
boards in `kicad_files/`, so the API documentation can't silently drift from
the code. Signature listings and intentionally incomplete fragments are
skipped automatically; ~30s total. It also fails when a public
`GridRouteConfig` field is missing from `docs/api-routing-config.md`'s tables,
when `docs/configuration.md` quotes a stale `GridRouteConfig` default, or when
an executed example opens a `kicad_files/` board git does not track.

```bash
python3 tests/run_doc_examples.py
```

### Post-route cleanup unit tests

Fast, board-free regression tests for the output-cleanup pass (issues #133 / #84).
No KiCad needed; each runs in well under a second.

```bash
python3 tests/test_phantom_copper.py     # drop write-list copper not on the board (#133)
python3 tests/test_dead_end_prune.py     # dead-end spur pruning + connectivity-gated removal
python3 tests/test_snap_stub_gaps.py     # close small same-net gaps (extend stub when clear)
python3 tests/test_remove_segments.py    # output segment removal (incl. v10 paren net names)
python3 tests/test_orphan_tjunction.py   # orphan detection by real copper extent (no false T-junctions)
python3 tests/test_restore_collision.py  # collision-safe rip-up restore: refuse stale copper that would short (#134)
python3 tests/test_drill_hole_keepout.py # hole-to-hole via keepout by mm distance, not a floored disk (#70/#125)
python3 tests/test_pad_offset_keepout.py # pad keepout measured from the real center, not the quantized cell (#70)
python3 tests/test_arc_track_parse.py    # (arc) track elements linearized into connected segments (hand-routed-board ingest)
```

## Verifying With KiCad's Own DRC (manual, not part of the tests)

The tests use the project's `check_drc.py` so they run without KiCad installed.
For a second opinion with KiCad's full rule engine (netclass clearances, board
min track width, solder mask, zone clearances), run `kicad-cli` on a routed
output manually:

```bash
# kicad-cli is on PATH on Linux/Windows; on macOS use the bundled binary:
#   /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli

kicad-cli pcb drc --severity-error --refill-zones --format json -o drc.json routed.kicad_pcb

# Errors AND warnings (catches dangling track ends):
kicad-cli pcb drc --refill-zones --format json -o drc.json routed.kicad_pcb

# Non-zero exit code on violations (for scripting):
kicad-cli pcb drc --refill-zones --exit-code-violations routed.kicad_pcb
```

Gotchas:

- **Always pass `--refill-zones`** when the board has copper pours: without it,
  stale zone fills produce bogus zone-clearance violations against the new
  tracks (the pour hasn't been re-poured around them yet).
- KiCad checks the **board's own design rules**, so routing with geometry that
  doesn't match the netclass (e.g. 0.101 mm tracks on a board with a 0.2 mm
  minimum width, or a pair gap below the netclass clearance) is reported as
  violations even though the router kept the clearances it was given. Route
  with the board's netclass values, or add matching custom rules in KiCad.
- To inspect only specific nets, filter the JSON, e.g.
  `python3 -c "import json; [print(v['type'], v['description']) for v in json.load(open('drc.json'))['violations'] if 'CLK' in json.dumps(v)]"`

## Performance Benchmarks

Results from `test_fanout_and_route.py` (full mode), recorded in January 2026
when this README was first written. The router has changed a great deal since;
read them as an order of magnitude, not a current benchmark.

| Stage | Nets | Time | Iterations |
|-------|------|------|------------|
| FTDI single-ended | 47/47 | 8.6s | 319K |
| LVDS diff pairs (batch 1) | 28/28 | 29.7s | 10.2M |
| LVDS diff pairs (batch 2) | 28/28 | 28.0s | 12.0M |
| DDR diff pairs | 5/5 | 0.3s | 25K |
| DDR single-ended | 51/51 | 6.2s | 565K |

Rust acceleration provides ~10x speedup vs pure Python.
