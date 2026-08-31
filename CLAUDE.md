# Project Notes for Claude

## Running Python

Invoke Python as `python3` (bare `python` does not exist on macOS and many
Linux distros). On Windows, if `python3` is missing, fall back to `py -3`
or `python` — don't retry blindly. Add `-X utf8` when a script prints
special characters (Ω etc.) to avoid Windows encoding errors.

**On Windows/Git Bash, `export MSYS2_ARG_CONV_EXCL='*'` before any command
carrying NET NAMES.** MSYS2 rewrites any argument starting with `/` into a
Windows path, and **every KiCad net name is `/`-prefixed**: `/+1V1` reaches the
tool as `C:/Program Files/Git/+1V1`. Nothing warns, because a tool cannot tell a
mangled net name from a net that does not exist. Measured: 61 nets passed to
`--ignore-nets`, 4 survived (the four not starting with `/`), and the resulting
render reported 1315 crossings where the truth was 357. It hits `--nets`,
`--ignore-nets`, `--power-nets`, `--rip-existing-nets` — every net-name argument.
**The variable is not free**: it also disables conversion for legitimate paths,
so `~/Documents/...` then arrives as an unusable `/c/Users/...`. Pass
Windows-style paths (`C:/Users/...`) in the same command.

## Building the Rust Router

Use `build_router.py` to build the Rust router:

```bash
python build_router.py
```

This builds the Rust module, copies the library to the correct location, and verifies the version. Do not run `cargo build` directly.

**Prefer Python-only solutions; avoid changing the Rust router (`rust_router/`)
unless clearly necessary and agreed.** A Rust change forces a crate version bump,
a `build_router.py --from-source` rebuild, and re-distributing prebuilt per-platform
binaries via GitHub Releases — heavy overhead. When a feature seems to need a Rust
change, surface that cost early and check for a Python-only approach first.

**Important:** When making changes to the Rust router, bump the version in `rust_router/Cargo.toml` and update the version history in `rust_router/README.md`. The release triple is `rust_router/Cargo.toml` + `/VERSION` + `metadata.json` — keep them aligned. **The crate is 0.20.1 and the 0.20.1 binaries ARE now published, in the v0.20.2 release** (2026-08-09; verified — the published `grid_router-macos-arm64.so` reports `__version__ == 0.20.1`). v0.20.2 is a python-only release, so `Cargo.toml` correctly stayed at 0.20.1 while `/VERSION` went to 0.20.2, and the release built the crate as it stands: a plain `python3 build_router.py` now **downloads and keeps** the prebuilt instead of paying a wasted download and rebuilding from source. (The older hazard, for reference: the v0.20.1 release carried 0.20.0-built assets. `build_router.py` handles that case on its own — it skips the download when Cargo.toml is ahead of the release tag, and when the tag matches but the asset inside is stale it detects the version mismatch after install and rebuilds from source. Both checks verify in a fresh subprocess: an in-process re-import of a compiled extension reports the previously-loaded library. `--from-source` just skips the lookup. 0.20.1 removes the `block_vias` parameter from `add_stub_proximity_costs_batch`, so the 0.20.0 binary is API-incompatible with current Python besides the version gate.) **Note this is why `/VERSION` can lead `Cargo.toml`** — a python-only release still republishes the current crate binaries, which is the cheapest way to unstick a stale-asset release.

## Testing & Verification

Validate routed boards against the *real* spec, with the right checker — most
"mystery" bugs here turn out to be grading mistakes, not routing bugs:

- **Connectivity is orthogonal to DRC.** A DRC-clean board can be fully
  disconnected (isolated copper has no clearance conflicts). Always run
  `check_connected.py` in addition to `check_drc.py` before calling a route clean.
- **Grade DRC at the clearance the board was actually routed to** — the route
  step's `--clearance` (recorded in `redo_commands.sh`), or the board's
  `.kicad_pro`/netclass — never a guessed/round value. Grading stricter than the
  route used manufactures phantom sub-clearance grazes. `check_drc.py
  --clearance-margin 0.1` filters ~grid-quantization noise (~8 µm artifacts).
- **Routing is deterministic, but outputs carry per-run random UUIDs.** Never hash
  or whole-file-`diff` `.kicad_pcb` outputs to judge determinism — compare
  `check_drc` / `check_connected` counts (stable run-to-run) instead.
- **`route.py` reads/writes a sibling `<output>.kicad_pro` DRC floor.** Re-running
  to the same output path reads it back and silently changes the routing (looks
  like non-determinism; it isn't). For clean A/B comparisons, route to a FRESH
  output path each run (or `rm` the `.kicad_pro` first).
- **Never `cp` a board without its `.kicad_pro` (#441).** The sibling `.kicad_pro`
  carries the DRC floor (the Default-netclass clearance/track/via the chain routed
  to). A bare `cp a.kicad_pcb b.kicad_pcb` strands it: the next route step reads no
  project, resolves its floor from the STOCK (looser) netclass, and stamps that over
  tighter copper — so KiCad grades correct sub-floor copper as phantom clearance DRC
  (icepi_zero: a dropped 0.09 floor became 0.10 → 160 phantom grazes). Use
  `python3 py_router/copy_board.py src.kicad_pcb dst.kicad_pcb` (copies `.kicad_pcb` + every
  sibling, self-records into the redo manifest), or copy the `.kicad_pro` too. The
  route scripts WARN when an input board has no sibling `.kicad_pro`.
- **Routers can report false success.** A router's own "routed" tally may come from
  a local/heuristic proxy while pads stay disconnected; re-verify with the
  authoritative, zone/fill-aware `check_net_connectivity` before trusting it.
- **A test's own failure path is the path nobody looks at.** A check that dies
  before it checks anything reports the same non-zero exit as a satisfied guard,
  so **a non-zero exit is not evidence — assert the REASON.** `tests/run_utils.py`
  has `check(argv, refuse='<the reason>', code=N)`, which reports an
  `ImportError`/traceback/argparse accident as a **BROKEN TEST** rather than as a
  guard that held; use it instead of `assert r.returncode == 2`. Likewise
  **verify the input before trusting the output**: `run_utils.evidence(path)`
  refuses a path that is not a real non-empty file, because a check whose input
  is missing tests nothing — and process substitution (`<(echo ...)`) is not a
  file on Windows. Measured: a negative control copied to a temp dir died on
  `ModuleNotFoundError` and was read as "the gate refused".
- **Read the failure buckets by their real definitions.** `failed_single` = "no
  result at all"; `open_single` = a KEPT result whose pads are still disconnected
  (non-multipoint only — a multipoint shortfall is already the pad deficit). A
  verdict is `len(failed_single) + len(open_single) + pad-deficit`, which is what
  `place_route_loop` counts. Reading only `failed_single` + the deficit is how a
  board shipping open copper reports `failures=0`: a NON-multipoint open net
  contributes to neither term. `terminal_restores` names rip victims restored at
  terminal failure with their outcome (`full` is the only success; `full_open`
  and `stub` ship broken). `stacked_copper` discloses same-net duplicate copper
  KiCad's DRC will never flag — the writer drops exact via re-emissions before
  writing, and whatever still stacks (e.g. two same-net barrels at one point with
  DIFFERENT drill/size, which is a fab question, not a bookkeeping one) is named
  in the summary rather than shipped silently.
- **Net classes are RESPECTED (PR392), and `--clearance` is a pure CEILING over ALL
  of them (#439).** The router honors KiCad's pairwise `max(classA, classB)` between
  nets of different classes — including copper routed earlier in the SAME call (in-run)
  — pricing each foreign obstacle at `config.obstacle_clearance(net_id)` (see
  `docs/api-routing-config.md`). `route.py` / `route_diff.py` / the fanout and plane
  scripts **always auto-read** every net's class clearance from the sibling `.kicad_pro`
  (override with `--net-clearances <json>`; all-Default boards are inert). **The
  PRESENCE of `--clearance` is the clamp switch, and there is nothing special about the
  Default class:**
  - **`--clearance` GIVEN** → it is a ceiling on *every* class (Default included): each
    net routes and grades at `min(its class, --clearance)` (the base/Default-net
    clearance is `min(Default class, --clearance)`; non-Default classes are capped in
    the map). A class tighter than `--clearance` survives; a looser one is capped. The
    output `.kicad_pro` writeback clamps every class DOWN to the routed floor so KiCad
    grades exactly what was routed.
  - **`--clearance` OMITTED** → no ceiling: each net routes at its OWN net-class
    clearance (base = the board's Default class, else `routing_defaults.CLEARANCE`
    0.25), and the writeback PRESERVES the classes. This is how you honor a genuine
    impedance board's class spec — just don't pass `--clearance`.
  - **`place_fanout_clearance.py` obeys the same two branches (#768/#769)**, and
    it is the only PLACEMENT step that does, because it is the only one that
    lays copper (the #313 via nudge) and therefore the only one that writes a
    DRC floor back. The ceiling caps the NETCLASS tier only -- a `.kicad_dru`
    rule and a pad `local_clearance` outrank it, since the writeback clamps
    neither. The project is written in EVERY exit path including the zero-move
    one, so a run that legitimately moves nothing still ships the spec it was
    graded against. `grade_pad_legality` and `quench` keep the uncapped
    `max(base, netclass)` semantics: they write no project, so the class they
    price at is one KiCad will still enforce.
  - `--hole-to-hole-clearance` / `--board-edge-clearance` work the same way: omitted →
    the board's own `min_hole_to_hole` / `min_copper_edge_clearance` constraint (via
    `list_nets.board_constraint`), else the fixed default.
- **A board may DECLARE what it is for (#711), in a sibling
  `<board>.design-brief.json`.** Placement otherwise infers everything from the
  board: `emit_intent` is "a starter intent READ OFF the board", and every
  connector's edge is guessed from its current pose by `_nearest_edge`, which is
  the only source of an edge in the toolchain. The brief is the channel for the
  facts a board file cannot contain -- which connectors are user-facing, which
  edge each belongs on and **where along it**, what the enclosure forbids. It is
  auto-discovered by `check_floorplan.py` and `board_brief.py` the way
  `kicad_dru` discovers a `.kicad_dru`, carried by `copy_board.SIBLING_EXTS`
  (which every other sibling-copy site now imports), and **compiled** into the
  existing intent by `check_floorplan --emit-intent` rather than being a second
  constraint system -- so it adds no intent key, and the placement CLIs receive
  it through the `--intent` they already take rather than through a flag of
  their own. (Unlike `.kicad_dru`, which EVERY routing step reads, the brief is
  read by those two tools and reaches the rest as a compiled intent.)
  `--brief PATH` overrides, `--no-brief` is the OFF arm,
  `--require-brief` refuses a grade with nothing declared behind it. On the
  `--intent` path it reports DRIFT instead of merging, because the graded
  document must be the file the caller pointed at. "I do not know" is a first-
  class value: `"unknown"` (the author looked) is reported apart from an absent
  key (nobody looked), and neither is ever guessed. `envelope`, `outline` and
  `height` are refused BY NAME -- the outline is not ours to change, and nothing
  in the placement stack measures z, so a declared height limit would grade
  nothing at all. See `docs/design-brief.md`.
- **Protected nets (#521): matched groups and routed diff pairs are recorded in
  the sibling `.kicad_pro`** (`kicad_routing_tools.protected_nets`, written next
  to the DRC-floor writeback, carried down chains by the project copy) and later
  steps will NOT rip them: `--rip-existing-nets` globs skip them (printed
  exclusion) and plane-repair `--rip-blocker-nets` never picks them as blockers.
  The override is naming the net EXACTLY (no glob) in `--nets` or
  `--rip-existing-nets` — there is deliberately no CLI flag or GUI control.
  Rationale: a retry step once ripped a whole DDR match group and rerouted it
  unmatched (allwinner_h3_ddr3, 40/41 nets unmatched, one net stranded).
  **KiCad-LOCKED copper** (`segment.locked`/`via.locked`, both parse paths) makes
  its net never-rippable with NO override. **Impedance declarations** persist
  the same way (`net_impedance` key: ohms/differential/pair_gap/coplanar_gap):
  impedance nets stay rippable, but a later step touching them without
  `--impedance` recomputes the same widths from the stackup and applies them
  per-net (config `net_layer_widths`; route_diff reapplies call-level, one
  spec only).
- **Per-layer clearance comes from the board's `.kicad_dru` (#498) and OUTRANKS
  `--clearance`.** KiCad stores layer-scoped clearance in custom rules
  (`(rule x (layer inner) (constraint clearance (min 0.15mm)))`); netclasses can't
  express it. **Every routing step** auto-reads the sibling `.kicad_dru`
  (`kicad_dru.install_layer_clearances`; engines without an `input_file`
  discover it via `PCBData.source_path`): signal, diff pairs, the pour step,
  the route step's in-run plane finalize (taps, region joins, reconnects),
  BGA/QFN fanout (QFN swaps
  its single escape layer exactly; BGA floors its scalar at the largest rule
  on its escape layers — conservative, tighten-only), the oracle sub-routes
  (config clones carry the map), and nested reconciliation sub-runs (the
  parent forwards its resolved map — the output's dru sibling doesn't exist
  yet mid-run). **Replacement** semantics — a rule
  value replaces the net/class pair clearance on its layer, tightening or
  relaxing, exactly like KiCad's own precedence (custom rules outrank classes;
  only the fab floor pins them up). There is deliberately **no CLI flag and no
  GUI control** — the rules file is the single source of truth; `check_drc` and
  the staged kicad-cli grade read the same file, `copy_board`/
  `fix_project_for_output` carry it as a sibling, and the DRC writeback caps
  `min_clearance` at the smallest rule so a relaxing rule isn't floored away.
  Grade a ruled board with plain `check_drc.py` (it auto-reads); a hand-rolled
  checker that ignores the dru will manufacture phantom flags on relaxed layers
  and miss real ones on tightened layers.
  **Why clamp on a ceiling:** stock net classes are largely *aspirational* — corpus and
  real boards route below them, and even the human-routed references violate their own
  class (zynq: 499 clearance violations at its 0.2 class, routed ~0.1), so keeping the
  stock class in the output manufactures phantom sub-class DRC on copper routed
  correctly at the fab floor. Helpers: `list_nets.board_default_netclass_clearance` /
  `board_constraint`; the GUI mirrors this with per-floor **override checkboxes** (Min
  Clearance / Min Hole-to-Hole / Min Edge Clearance — unchecked = use the board's own
  minimum, checked = clamp to the entered value). The old `--clamp-netclasses` **and**
  `--no-clamp-netclasses` flags are **removed** (the `--clearance` ceiling replaces
  both; `--net-clearances <json>` gives explicit per-net control). Grade multi-class
  boards at the netclasses that survived (`kicad_drc_compare._staged_copy`).

## What a placement run is FOR (read before grading one)

**The objective is a board that ROUTES: parts arranged so they work together,
and zero `unrouted` and zero `broken` nets at the end. It is NOT restoring
parts to the poses they had before.**

This is stated here because the perturbed-corpus rig (#411) grades on
`recovery` and `home /N`, which measure **distance to the original pose** —
and those are the wrong headline for this goal. A placement that is
electrically excellent but arranged differently scores ~0 recovery. Measured,
run 10: `recovery` **−0.0014** and `home` **0/30** on a run that took
copper-free DRC 9 → 0, assembly blocking 4 → 0, and `check_assembly` from NOT
BUILDABLE to **buildable**. The headline said failure about a board that had
become strictly more buildable.

So, when grading a placement:

- **Lead with the routed outcome** — `board_score`'s `blocking`
  (`unrouted + broken` first, then the rest), with `quality` (vias,
  copper_mm, segments) as the tie-break once `blocking` is 0.
- **Keep `recovery` as a DIAGNOSTIC, never the score.** It is how you catch a
  run that wandered — `collateral_pad_rms` rising (run 10: 0.000 → 3.670 mm)
  means parts nothing had damaged were moved, which is a real defect. But a
  negative recovery on a board that got more buildable is not a failure of
  the run.
- **The human original is a BENCHMARK TO APPROACH, not a pose to match**
  (its vias / copper_mm / segments), because a human layout is one solution,
  not the only one.

**A part whose pad copper lies outside the outline is the top-priority
placement defect**, ahead of every clearance graze: its nets cannot be routed
at all, so it converts one-for-one into `unrouted` and `broken`. Measured, run
10: 11 such parts produced ALL 13 unrouted nets and most of the 37 broken ones.
Read it off `render_placement --json-out`'s
`checklist.a_off_outline.pad_copper` — a whole-board pass/fail verdict is the
wrong channel for it.

**Scope a placement search to the refs the gate names.** When a gate names
specific parts, free exactly those and lock everything else. A global sweep
orders its violators by its own priority — usually worst-off-board first — and
may never reach the ones actually blocking you. Measured: freeing 2 parts and
locking the other 105 cleared both blocking pairs in **63 seconds**, where
whole-board sweeps ran 10+ minutes without touching them.

**Repair searches start from the part's CURRENT pose**, which carries no
information once a part is tens of millimetres from where it belongs. Expect
cost to grow sharply with the displacement cap, and prefer re-seating such a
part over nudging it.

## Stress testing & A/B replay

Every recorded stress run leaves a `redo_commands.sh` manifest that replays the
full chain with **no LLM**. To regression-test or A/B an engine change across the
board corpus, use `tests/stress/ab_replay_grade.py` (whole-set replay + DRC/
connectivity grading) or `tests/stress/redo_diff_stage.py` (diff-pair stages only).
For CORPUS-SCALE work (~$1/arm on rented cores, keeps the routed boards):
`tests/stress/cloud_replay_sets.py` A/Bs a change over whole sets, and
`tests/stress/corpus_bisect.sh` scores one engine commit for bisecting a
regression. See `tests/stress/RUNBOOK.md` ("Replaying & A/B (no LLM)" and
"Corpus-scale A/B and bisect on the cloud") for the recipes and the rules that
make them trustworthy -- notably: the baseline is the RECORDED RUNS re-graded
(not an archived wave), grade both sides on the same terms, compare only boards
that replayed an IDENTICAL chain, and **a two-board result is not a default
change** (per-board spread is +-2..3 nets).

**A new PLACEMENT objective term goes through `tests/test_placement_ab.py`
before it ships on.** It runs the same board twice (flag off, flag on), writes
both, and grades both with an *independent* check — `floorplan.grade(...,
with_health=True)` re-derives its corridors from the FINAL poses, so a term that
only improves the model it is computed from shows up as "improved nothing". Add
a row to `ROWS`, do not add a file. Five rules the table encodes and that are
easy to get wrong:

- **Judge on ≥3 DISTINCT boards, paired and directional** (improve on ≥ N−1,
  regress on none), never a per-board absolute. Neutral boards are printed, not
  dropped. This is **enforced in `gate()`**, not just stated here (#694): the
  old rule counted trial *rows*, so one improving row on one board passed, and
  `--row` made that the convenient path. The refusal applies to rows **on
  trial**, and every row in the table today is pinned — so a real run does not
  reach it and `--self-test` does. A term whose per-board direction is a coin
  flip passes the rule 1 run in 2^N (1 in 8 at N=3), which the run prints when
  a term is on trial.
- **Keep the row that disagrees.** A term that helps on one board of three is
  not a term, and deleting the dissenting row is how that becomes folklore.
- **A rejected term keeps its rows**, marked `rejected` with its measured
  `expect`, so it stays a change detector instead of a permanent red mark that
  someone eventually deletes along with the finding.
- **Numbers live in `tests/placement_ab_baseline.json`, never in a `why`
  string.** Every run re-measures and compares it per key and per arm, reporting
  a reversed direction (`INVERTED`) apart from a moved value (`DRIFT`), a
  baseline row `ROWS` no longer declares (`ORPHAN`), and a baseline that is not
  shaped like one (`MALFORMED`). A `why` records the MECHANISM only. This exists
  because `corridor-ulx3s` sat rejected on a recorded claim whose signal had
  reversed while the gate printed PASS (#694) — and the reason is worth getting
  right, because the obvious reading is wrong: **the gate never compared the
  signal.** `_verdict` collapses the signal, the guards and intent errors into
  ONE mark, and only that mark is checked against `expect`, so the reversal was
  MASKED by a different criterion turning the mark `regress` for its own
  reasons. An aggregate verdict cannot say which of its inputs moved.
  **A placement-engine change that moves these numbers re-records the baseline
  (`--write-baseline`) in the same commit**, after reading the table; a partial
  run refuses to write one, and a missing baseline FAILs rather than passing.
  `--baseline ""` is the deliberate way to run without the comparison, and
  `--self-test` runs the gate and comparator logic in milliseconds at the top of
  every invocation.
- **A mark resting on intent errors names the rules that moved**
  (`intent errors A -> B (zone_containment X -> Y)`). An unattributed error
  count is what let #694's inverted row keep reading as an intact finding.

Two traps measured the hard way: the first run of that harness reported the
corridor term inert because it had been pointed at a **merged** net glob whose
`cover` was 0.46 — a phantom corridor (declare sub-buses separately; `SDRAM_A*`
scores 0.81, `SDRAM_*` 0.46). And grade intent errors **paired**, not against
zero: both arms quench the board, so both walk parts out of the emitted intent's
zones for reasons the flag did not cause.

Every metric in that harness is still a **proxy**, so `tests/test_placement_probe.py`
(opt-in, slower) actually routes. It scopes the route **causally, not by which
parts moved** — the nets `net_affinity` flagged plus the declared corridor nets,
fixed from the OFF board and identical on both. Scoping by moved parts is
circular: a term that moves nothing would score a perfect null.

## Keep CLI and GUI routing in sync

There are two front-ends to the same routing engine, and a fix to one is
**not** automatically a fix to the other:

- **CLI scripts** — `route.py`, `route_diff.py`, `route_planes.py`,
  `repair_planes.py` (renamed from `route_disconnected_planes.py`; the CLI
  remains a standalone utility only — the chain step is absorbed into
  route.py's finalize, #562), `bga_fanout.py`, etc. Their `main()`
  parses args and calls the shared engine functions (`batch_route`,
  `batch_route_diff_pairs`, `create_plane`, `generate_bga_fanout`, ...).
- **GUI plugin** — `kicad_routing_plugin/` (`swig_gui.py`,
  `differential_gui.py`, `planes_gui.py`, `fanout_gui.py`) calls those
  **same shared functions directly** (with `return_results=True`,
  `dry_run=True`/`output_file=""`), building `PCBData` from the live
  pcbnew board via `build_pcb_data_from_board` rather than parsing a file.

Because both call the shared engine, fixes **inside** those functions are
picked up by both for free. The gaps appear at the edges:

- **A fix in a CLI `main()` only** (argparse, defaults, output writing,
  post-processing) is invisible to the GUI — the GUI re-implements that
  layer. Put shared logic in the engine function, not in `main()`.
- **A new engine parameter / flag** must be threaded through *both* the
  argparse layer *and* every GUI call site (plus its config dict, the
  options panel, and `settings_persistence.py`). A new `batch_route`
  kwarg that only `route.py` passes silently does nothing in the GUI.
  It must also stay **Claude-settable end to end**: (1) the GUI plan
  executor (`ai_plan.py`) applies any snake_case param whose name
  matches a dialog control — so name the control after the param and add
  it to `reset_params_to_defaults` (the plan executor resets through
  that, or the param leaks between steps); (2) add the `--flag` →
  param-name mapping to `tests/stress/manifest_to_plan.py` `FLAG_PARAMS`
  so recorded manifests convert to `*_plan.json` with the param intact.
  Verify with: convert a manifest carrying the flag and check the plan
  JSON step params include it. (There is **no longer a third step**: the
  GUI-parity gates used to hand-mirror a config map in
  `test_gui_engine_parity.py`, which had to be kept in sync by hand and
  silently drifted. Both gates now drive the REAL dialog through a plan,
  so a new param needs no mirroring — if it reaches the dialog control it
  reaches the engine.)
- **A changed default** must match in both places — the GUI sets its own
  values from UI controls and does not inherit argparse defaults.
- **Parser/obstacle/writer fixes** in shared low-level modules are used by
  both, *except* file-text-parsing fixes in `parse_kicad_pcb`: the GUI
  builds `PCBData` from pcbnew instead, so `build_pcb_data_from_board`
  must be kept at parity with the text parser separately.

- **A post-pass added to a CLI `main()`** (running *after* the shared engine
  call — cleanup, oracle recheck, DRC-floor writeback) is invisible to the
  GUI unless separately replicated (the set11 plane-shorts bug:
  `repair_planes.main()` (then `route_disconnected_planes`) ran
  `clean_plane_copper`, the planes tab didn't). Prefer putting the pass INSIDE the shared engine function; when it
  must operate on the written file, refactor a **board-level core** and call
  it from both fronts (as `compute_plane_copper_cleanup` now backs both
  `clean_plane_copper` and `planes_gui._run_plane_copper_cleanup`).

**Rule of thumb:** whenever you change routing behavior via the CLI, check
whether the corresponding GUI call site (and its options panel) needs the
same change — and vice versa. When adding a flag, grep the
`kicad_routing_plugin/` call sites for the function you changed and wire it
through there too.

**Parity gates (run these when touching CLI/GUI routing):**
- `tests/gui_parity/test_manifest_plan_parity.py` — no wx; asserts every CLI
  `--flag` survives `manifest_to_plan` into the GUI plan step (plan→params).
- `tests/gui_parity/test_cli_postpass_coverage.py` — no wx; asserts every CLI
  `main()` post-engine pass has a GUI counterpart, and blocks a new CLI-only
  post-pass (Class-2 drift). Register new passes there.
- `tests/gui_parity/test_settings_roundtrip.py` — needs KiCad python; builds
  the REAL headless dialog and round-trips the settings dict: save (the CLOSE
  path), restore (the reopen path), restore from a LEGACY dict whose keys a
  newer version dropped, and re-save key parity. **Run it whenever you add,
  rename, or REMOVE a dialog control** — deleting one without updating
  `settings_persistence` crashes on close and loses the user's settings
  (that shipped once; see 31f359c). Seconds to run, no routing.
- `tests/gui_parity/test_fanout_rotated_gui.py` — needs KiCad python; the only
  gate that runs a **fanout** step. Drives the REAL FanoutTab on a rotated
  QFN (haasoscope U2, QFN-76 @ 90°) and compares against the CLI's text-parsed
  engine call, replaying the tab's OWN captured kwargs so the two fronts cannot
  differ by a parameter. Asserts side classification parity (the check that
  catches a local-frame bug), emitted-copper parity, and outward escapes.
  Seconds to run. Its wx-free half is `tests/test_rotated_footprint_frame.py`
  (round-trip invariants + change detectors, ~1 s, runs under `run_all.py`).
  **Fanout is the only routing consumer of `pad.local_x/local_y`**, and those
  are the one PCBData field the GUI COMPUTES rather than reads — see the
  `_global_to_local` entry in `.gui-parity-checked` for the bug that motivated
  these.
- `tests/gui_parity/test_gui_engine_parity.py` — needs KiCad python; runs the
  plan through the GUI engine path and grades against the CLI chain
  (`KICAD_DUMP_BATCH_KWARGS` diffs the full batch_route param set, ~105 keys).

  **How to run it (it is NOT a special-session thing — just run it):**

  ```bash
  python3 tests/gui_parity/test_gui_engine_parity.py            # default board
  python3 tests/gui_parity/test_gui_engine_parity.py [board.kicad_pcb] [--workdir DIR]
  ```

  It **re-execs into KiCad's bundled python automatically** (it probes
  `/Applications/KiCad/KiCad.app/.../bin/python3`, then `/usr/bin/python3`, then
  the Windows path), so plain `python3` is the correct way to launch it — you do
  NOT need to find the interpreter yourself. The dialog is constructed HEADLESS
  (`parent=None`, never shown, `WXSUPPRESS_SIZER_FLAGS_CHECK=1`), so no window
  appears and no display setup is needed. It routes the small in-repo board
  `kicad_files/splitflap_driver.kicad_pcb` (~9 s of chain), so it costs a couple
  of minutes, not an afternoon.

  Verify the prerequisites in one line before concluding you can't run it:

  ```bash
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 \
      -c "import pcbnew, wx; print(pcbnew.GetBuildVersion(), wx.version())"
  ```

  **Do not skip this gate on the assumption that "this session has no wx/pcbnew"**
  — six consecutive `.gui-parity-checked` markers carried it over as "pending a
  KiCad-python session" while the prerequisites were installed and working the
  whole time. CHECK with the line above; only record it as not-run if that
  actually fails.

  **macOS: if a wx gate hangs at ~0 CPU, it is NOT wx, load, or a deadlock — it
  is a modal alert you cannot see.** After a wx process is killed (a `pkill`, a
  timeout, a crash), macOS decides the app "quit unexpectedly" and the NEXT
  headless launch stops inside `NSApplication` bootstrap showing the
  *restore-windows* alert: `-[NSPersistentUIRestorer
  promptToIgnorePersistentState]` → `-[NSAlert runModal]`. Headless, nobody can
  click it, so it waits forever — the process sits in state `SN` accruing ~0.3 s
  of CPU over many minutes, which reads exactly like a hang. Diagnose it with:

  ```bash
  sample <pid> 3 -mayDie | grep -E "NSAlert|PersistentUI"
  ```

  The fix is a one-time user default (a sandboxed `HOME` does **not** work —
  `cfprefsd` serves the pref per-user regardless of `HOME`):

  ```bash
  defaults write -g ApplePersistenceIgnoreState -bool YES
  ```

  This cost an entire session's worth of "wx is blocked, gate not run" marker
  entries. With it set, `test_gui_engine_parity.py` completes in ~90 s.

  Two caveats when reading its output:
  - It **REPORTS divergence; it is not pass/fail.** Known-deliberate divergences
    exist (the CLI mains' kicad-oracle recheck, `clean_plane_copper`, end-of-run
    reconciliation, `.kicad_pro` floor carryover, the plan-parameter whitelist),
    so read the printed diff rather than trusting an exit code.
  - Its plan is a **signal + plane** chain, so it does NOT exercise diff-pair or
    impedance/coplanar parameters. A gap on those (e.g. the `coplanar_gap` one
    fixed in `c99ffb4`) can pass this gate untouched — cover such params with a
    diff-pair board and `check_impedance.py`.

  Siblings worth running the same way: `test_gui_livechain_rp2350.py`
  (reshaped to the #562 chain — pour → ONE route step carrying the plane
  nets in its `--nets`, whose in-run finalize is the weld/repair/oracle;
  both legs stage-aligned again and PASS. Two teachings baked into it: the
  plane nets must ride in the route step's net list or the finalize excludes
  the pours BY PLAN, and it stages its project-less fixture with a
  pcbnew-authored `.kicad_pro` — a project-less board makes the two fronts
  legitimately diverge), and
  `replay_plan_vs_run.py` — the latter is the *general* harness (a real headless
  `RoutingDialog` + real `PlanExecutor`, nothing mocked; it caught #493's
  one-ULP netclass clearance bug on its first run). See
  `tests/gui_parity/README.md`.

**Tracking the last-audited commit:** `.gui-parity-checked` (repo root,
git-committed) holds the SHA of the last commit a full CLI/GUI parity audit
covered, plus the date and outcome. To bring it up to date: `git log
--oneline <that-sha>..HEAD` to see what's new, `git diff <that-sha>..HEAD --
<CLI scripts> kicad_routing_plugin/` to see the engine-side vs GUI-side
diffs, then check every new engine parameter/flag/results-data key against
the GUI call sites (per the rule of thumb above). When the audit finds and
fixes a gap, commit the fix first, note it in the file, then update the file
to current `HEAD` and commit that too — so the recorded SHA always reflects
"parity confirmed as of here," not "parity assumed."

## KiCad Parser Usage

Full user-facing API docs (parser, writer, modification, config, net
analysis, impedance) live in `docs/python-api.md` and the `docs/api-*.md`
pages — keep them in sync when changing these modules. The doc examples are
verified by `tests/run_doc_examples.py`. Quick reference:

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
- `pcb.board_info` - BoardInfo (layers, bounds, stackup)

### BoardInfo / Stackup Attributes

- `pcb.board_info.copper_layers` - List[str] of copper layer names (e.g., ['F.Cu', 'B.Cu'])
- `pcb.board_info.layers` - Dict[int, str] layer_id -> layer_name
- `pcb.board_info.board_bounds` - (min_x, min_y, max_x, max_y) or None
- `pcb.board_info.stackup` - List[StackupLayer], ordered top to bottom
  (NOT `pcb.stackup`). Empty list if the board has no stackup section.
- StackupLayer fields: `name`, `layer_type` ('copper', 'core', 'prepreg', ...),
  `thickness` (mm), `epsilon_r`, `loss_tangent`, `material`

### Footprint Attributes

- `footprint.reference` - Component reference (e.g., 'U9')
- `footprint.footprint_name` - Footprint library name (e.g., 'interf_u:PGA120')
- `footprint.pads` - List[Pad] of pads
- `footprint.x`, `footprint.y` - Footprint position
- `footprint.rotation` - Rotation in degrees
- `footprint.layer` - Layer (e.g., 'F.Cu')
- `footprint.net_tie_groups` - List[List[str]] of pad-number groups the
  footprint deliberately shorts (`(net_tie_pad_groups "1, 2")`, Kelvin shunts /
  net-ties). KiCad's clearance exemption between the grouped pads is LOCAL:
  the tied net's copper may contact the partner pad only where the contact
  lies on its own pad. Consumers: `PCBData.net_tie_exempt_pad_ids(net_id)`,
  the obstacle builders (own-pad-sliver lift), and check_drc's waiver.
- `footprint.ref_label` - Optional[RefLabel]: the Reference silkscreen text's
  geometry (#481): `at_x/at_y` (footprint-LOCAL mm), `rotation` (the stored
  angle, which is ABSOLUTE board angle — probed on KiCad 10, `% 360`
  normalized), `size_h/size_w`, `thickness`, `layer`, `justify` (raw tokens;
  `mirror` is meaningful on B-side), `hidden`, `is_property_node` (False =
  KiCad 6/7 `fp_text` form). Both parse paths fill it identically
  (`tests/gui_parity/test_ref_label_pcbnew_parity.py` pins that). Consumers:
  `placement/labels.py` (beautify_labels engine), `write_label_output`.

### Pad Attributes

- `pad.pad_number` - Pad identifier (e.g., 'H2', '1')
- `pad.net_id` - Net ID (int)
- `pad.net_name` - Net name (e.g., '/PC-A7')
- `pad.global_x`, `pad.global_y` - Absolute position
- `pad.local_x`, `pad.local_y` - Position relative to footprint
- `pad.size_x`, `pad.size_y` - Pad dimensions in board space (resolved from the
  pad's absolute angle; swapped for ~90° pads so they're axis-aligned)
- `pad.rect_rotation` - Residual rect tilt (deg, in (-90,90]); 0 for axis-aligned
  pads, non-zero only for pads on non-orthogonal angles. Obstacle/DRC geometry
  rotates the pad rectangle by this. Run `check_pads.py` before fanout to catch
  mis-modelled (overlapping) pad geometry.
- `pad.shape` - 'circle', 'oval', 'rect', etc.
- `pad.layers` - List of layer names
- `pad.drill` - Drill diameter (0 for SMD, >0 for through-hole)
- `pad.hole_x`, `pad.hole_y` - Drill/hole position when the pad copper is
  OFFSET from it (`(drill (offset x y))`, castellated-module paddles);
  `None` = hole at `global_x/global_y`. `global_x/global_y` is always the
  COPPER center (clearance/DRC/obstacle consumers use it directly); drill
  geometry must use `pad_drill_capsule`/`pad_drill_circles` or hole_x/y.
- `pad.pad_type` - 'smd', 'thru_hole', 'np_thru_hole', 'connect'. NPTH pads have
  NO copper even when `layers` lists `*.Cu` (size = mask opening only): skip them
  in copper-clearance logic, only their drill hole matters. For "does this pad's
  barrel tie copper layers together", use `pad_is_plated_through(pad)` — never
  bare `pad.drill > 0` (a net-tied NPTH mounting hole is not a connection, #328)
- `pad.component_ref` - Parent component reference
- `pad.pinfunction`, `pad.pintype` - Pin metadata
- `pad.castellated` - KiCad `(property pad_prop_castellated)`: a deliberate
  half-hole pad ON the outline (both parse paths). The routing mains' retract
  post-pass (`pcb_modification.retract_castellated_landings`) pulls track ends
  landing in its edge-clearance zone back to the pad's inner reach.
- `pad.local_clearance` - RESOLVED per-pad clearance override in mm (#326): the
  pad's own `(clearance ...)`, else the footprint-level override (recorded raw
  in `footprint.clearance`), else 0 (= global/netclass clearance applies).
  KiCad enforces max(the two items' clearances) per pair; the obstacle stamps
  and check_drc honor it the same way. Clearance consumers should read this
  field, never re-derive footprint inheritance.
  **The PLACEMENT side honors it too, since #697** — `placement.legality`'s
  `PadClearanceModel` resolves each pad pair at check_drc's own value
  (`max(clearance, netclass a, netclass b)` → `.kicad_dru` layer rules over the
  SHARED copper layers, which REPLACE → `max(…, lc_a, lc_b)`), and CALLS
  check_drc's `pad_copper_layers` / `pads_shared_layer_clearance` rather than
  mirroring them. It is strictly inert (`model.active` False, every consumer on
  its original flat-scalar path) when the board declares no netclass, no dru
  rule and no pad override. Before #697 the census priced every pair at one
  flat scalar and read `local_clearance` nowhere in `py_placer/`, so a board
  failing DRC on a 1.016mm fiducial keep-clear reported **0 conflict pairs** to
  fix. Two consequences worth knowing: a pair graded above the board-wide
  clearance is disclosed in `grade_pad_legality`'s `required` key (print it via
  `legality.format_required_clause`, never a hand-copied string), and
  `placement/fanout_clearance.py` is a SEPARATE flat-scalar channel that still
  has this bug.

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
- `segment.locked` - KiCad `(locked yes)`: user-pinned copper; its net is never
  rip-eligible (#521), no override

### Via Attributes

- `via.x`, `via.y` - Position
- `via.size` - Via outer diameter
- `via.drill` - Drill diameter
- `via.layers` - Layer span
- `via.net_id` - Net ID
- `via.locked` - KiCad `(locked yes)` (see `segment.locked`)
- `via.tenting_attrs` - Protection spec `{token: raw inner s-expr}` for
  `tenting`/`covering`/`plugging`/`capping`/`filling` (#489 §8); `{}` = the board
  specified nothing. Read by BOTH parse paths in the same normalized form. Pass it
  back via `generate_via_sexpr(..., tenting_attrs=...)` for any via that already
  existed — a RE-PLACED via (rip-up, sub-grid nudge, tap relocation) otherwise
  loses its spec and is re-stamped with front+back tenting, which is wrong for
  via-in-pad (needs IPC-4761 Type VII filled+capped+plated). Vias the tool ADDS
  emit **no protection token at all**, so they inherit the board's own
  `(setup ...)` policy — what pcbnew does for a via the GUI adds and KiCad for
  one the user places. Probed against pcbnew 10.0.0: a via at
  `*_MODE_FROM_BOARD` serialises with NO token and a token appears **only** for
  an explicit override, so anything stamped turns an inheriting via into an
  override. The old rules — a hardcoded front+back tenting, then
  `prevailing_via_protection(pcb.vias)` — are both retired: measured over 886
  corpus boards a prevailing spec NEVER disagreed with the board's own setup, so
  it only wrote a redundant token, and the tool then read its OWN stamps back as
  "the board's convention" next run. The hardcoded default was worse than
  redundant: three boards (nanovoltmeter_marge, hexberry_fpga, pedal_404) declare
  `(tenting (front no) (back no))` board-wide and had every added via stamped
  tented — a fab error, hidden because KiCad's FACTORY policy is tented so the
  two agree on an ordinary board. `prevailing_via_protection` still exists and is
  still correct; it is just not a default any more. When RE-PLACING a via, also pass
  `inherit_when_unspecified=True` (#741). `None` **and `{}`** otherwise both mean
  "the caller has no opinion", which on KiCad 10 output stamps front+back
  tenting (on a numeric-net board they emit nothing) — and `{}` is exactly what
  `Via.tenting_attrs` holds for a via that carries no spec, so handing it back
  verbatim is the bug. With the flag an empty spec emits nothing, so the via
  keeps inheriting the board's `(setup ...)` — what it had, and what the GUI
  side (`gui_utils.apply_via_protection`, early-return on an empty spec) has
  always done. Spell it `tenting_attrs=v.tenting_attrs,
  inherit_when_unspecified=True` — a keyword rather than a sentinel VALUE,
  because the repo's own idiom for carrying a spec is `dict(...)`, which would
  turn any dict-shaped sentinel back into a plain `{}` and silently restore the
  bug. **Every emit site must also keep the board's net DIALECT**, via the ONE
  resolver `kicad_writer.via_net_name(net_id, net_id_to_name)` (#749 D):
  `net_id_to_name` has no key 0 on ANY board, so a plain `.get` sends every
  no-net via down the numeric dialect. `docs/api-kicad-writer.md` has the table
  of which site passes what, and
  `tests/test_749_via_protection_emit_sites.py` walks the AST to catch a new
  site that forgets. **#748** is the parser half of the same story: its
  numeric-net via pattern had no gap for the protection tokens, so a numeric
  ref emitted next to a spec was a via `extract_vias` could not read back at
  ALL -- an invisible barrel, not just a lost spec. Both dialects now read the
  whole family in any position, and each via is matched inside its own
  paren-balanced block, so no pattern can run out of one via into the next (on
  a MIXED-dialect board -- which this repo's own fanout step produces -- that
  used to invent a barrel and swallow a real one). GUI side:
  `gui_utils.apply_via_protection(pcb_via, attrs)` writes, and
  `kicad_parser.pcbnew_via_protection_attrs(via, text_specs)` READS -- not the
  private `_pcbnew_via_protection_attrs`, which answers `{}` for every via on
  the shipping KiCad 10.0.0 because its SWIG wrapper omits the
  `TENTING_MODE_*` family (#751); the resolver falls back to the board file. `fab_notes.print_via_in_pad_note`
  emits the IPC-4761 note from the shared engines when a run puts vias in pads.
