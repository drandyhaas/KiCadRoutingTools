---
name: pcb-free-agent
description: Places and/or routes a KiCad board to a finished, verified result. The agent picks its own steps from the repo's CLIs, then one independent verifier grades the board; it hands back the board, a film and REPORT.md. Modes are full (place + route), place, and route (a placed board, no part moved). Use it to place, arrange, re-place or optimise components, fix a placement, handle an unplaced board or parts piled off the outline, place and route end to end or from scratch, cut vias, or run the GUI's Place / Place + Route. It never changes the outline. For a routing PLAN only, use plan-pcb-routing; for sign-off QA, use review-routed-board.
---

# PCB free agent

You get a board, a goal and a toolbox, and you decide the steps. This skill
replaces the staged placement and combined drivers. It keeps the evidence,
because the verifier and the grader measure the board, but it does not
prescribe the process.

Invocation: `/pcb-free-agent <mode> <board.kicad_pcb> [intent.json]`, where
mode is `full`, `place` or `route`. With no mode, use `full` for an unplaced
board and `route` for a placed one. The positive test for which one is
`python3 -X utf8 py_tools/board_brief.py <board> --json wk/<run>/brief.json`:
read `pile` and `has_copper` on its `JSON_SUMMARY` line (the file carries
them under `state`); exit codes are not the test. Read `pile`, not
`unplaced`: a staging ring of parts around the outline is a pile but reads
`unplaced: false` (#1109). The same line's `poured_nets` names the nets a
copper pour already serves (`pours` in the file: layers, filled or not,
coverage), which `has_copper` does not count; route.py's plane finalize
serves those nets from the fill.

**Measured basis.** Two runs used this contract before it became a skill:
- **An 18-part 2-layer board, from a pile:** DONE in 12 min, 6 vias. The
  staged loop took 3h54m to 39 vias.
- **A 264-part 4-layer board with a BGA-121, all parts piled and unlocked:**
  blocking 15 in 7h56m. The staged loop took ~32 h to reach 35 on an easier
  input.

## 1. The goal, per mode

| mode | input | DONE means | better, once DONE |
|---|---|---|---|
| `full` | an unplaced or placed board | `board_score` blocking 0, zero unrouted, zero broken, `check_complete` DONE | fewer vias, then less copper_mm, then fewer segments |
| `place` | an unplaced or badly placed board | `check_assembly` buildable, `check_floorplan --intent` 0 errors, no pad, graphic or keep-out copper off the outline, mechanical facts met | fewer floorplan warnings, then lower hpwl, then fewer airwire crossings (`render_placement --json-out`, or `py_placer/placement_score.py` on a copper-free board) |
| `route` | a placed board | the same as `full`, **with no part moved** | the same as `full` |

- **"Better" is lexicographic, never a weighted sum.** A lower via count never
  buys back an open net.
- **DONE is measured with every spec the board declares.** Pass
  `--net-min-widths`, `--impedance-nets`, `--length-groups` (a FILE) and
  `--min-track-width` / `--min-via-diameter` / `--min-via-drill` to
  `board_score` and `check_complete`. An `ungraded` component is UNEXAMINED,
  not passed: without spec flags, `undersized` grades at the fab floor, and
  141 of 141 vias once read clean against a 0.6 mm spec.
- **Matching the original layout does not count.** A human layout is a
  benchmark to approach, not a pose to match.

## 2. Before the first command

- **Declared facts win.** If the board has a sibling
  `<board>.design-brief.json` or `mechanical.json`, compile the intent from it:
  `python3 -X utf8 py_tools/check_floorplan.py <board> --emit-intent <intent.json>`.
  If the user states a mechanical fact (a connector's edge, a mounting-hole
  position), it is a requirement, and the verifier checks it.
- **Never emit the intent from a damaged board.** It records the damage as
  the spec: one such intent failed the correct board and passed a 142 mm²
  pile-up. Emit it from the brief, or edit it down.
- **When a placed reference of the design exists, arm the decap rule from
  it.** Add `--decaps-from <reference.kicad_pcb>` to the `--emit-intent`
  call (a human layout, or an earlier placement), and say in your report
  that you did. A pile has no decap distances to read, so without it the
  rule stays unarmed: run 36 left StickHub's hub decaps 2.1-9.8 mm from
  their pins, where the human board keeps them within 2.2 mm. A cap the
  reference keeps within 5 mm of its chip and your board leaves beyond it is
  then a `decap_ungraded` ERROR, named by ref (#1142). Nothing repairs it
  automatically -- `--repair-decaps` covers `decap_distance` and
  `decap_pin_distance` only -- so move the cap with `place_pose`.
- **On a pile, read the emitter's decap line before you seed.** It says how
  many caps the per-supply-pin stage can claim, and which only once their
  owner ICs are seated. When `place_seed` then reports the decap stage
  claimed 0 because no owner IC was seated before it, the first remedy is
  to seat the owners first (a `fixed_poses` entry or a zoned block). The
  other is an EXPERIMENT, not a fix: re-seed with
  `--decap-claim-after-ics` and keep it only if it grades better on
  routed outcome. It claims the decaps once the centroid stage has seated
  their ICs, and the seed places every IC exactly as without it -- only
  the polish that follows can move one (#1105). The line's stage-2.5
  count assumes those early seats succeed. The corpus A/B rejected it as
  a default: it marked `regress` on all 7 of its boards. On run 38's
  StickHub pile (seed 0, run 38's own arguments) it claimed 16 caps and
  cut the seed's grade errors 16 to 7.
- **Give `--intent` to every placement tool.** It is a per-move gate only in
  tools that receive it. It stops a part LEAVING its zone; it never moves one
  back in.
- **Nothing is sacred except a KiCad lock.** The tools never move a locked part
  (`(locked yes)`), and there is no override. For a from-scratch experiment
  with everything off the board, build the input with
  `python3 -X utf8 .claude/skills/pcb-free-agent/scripts/make_unplaced.py <src> <dst>`.
  It piles every pad-bearing part off the outline and removes each moved
  part's lock. It refuses a routed input: strip that first with
  `python3 -X utf8 tests/stress/strip_copper_only.py <src> <dst>`, which keeps
  the outline bit-identical.
- **Work in a run directory**, `wk/<run>/`. Copy every board with
  `python3 -X utf8 py_router/copy_board.py <src> <dst>`, never a bare `cp`,
  because the `.kicad_pro` beside a board carries its DRC floor.
- **Git Bash:** `export MSYS2_ARG_CONV_EXCL='*'` before any command that
  carries a net name. Quote `--nets '*'`.

## 3. The toolbox

Everything in `py_placer/`, `py_router/`, `py_tools/` and `docs/` is yours.
Read `--help` before assuming a flag does not exist. Two runs declared
"no lever left" while the lever sat unread in `--help`.

| job | tools |
|---|---|
| score (the authority on `blocking`) | `py_tools/board_score.py <board> --intent <i> --baseline <input> --json <out>`; `check_complete.py <board> --intent <i> --baseline <input>` (fails closed; `--baseline` arms check_assembly's courtyard gate, and without it that gate is listed `ungraded`) |
| read the board | `py_tools/board_brief.py <board> --json <out>`, `py_tools/board_context.py --md` (per-part sheet: pin order, `CROSSED` pairs) |
| place from scratch | lock the fixed parts with `py_placer/place_pose.py` first, then `py_placer/place_seed.py` (about 5–15 min on a 250-part board; rank seeds with `py_placer/compare_seeds.py`) |
| improve a placement | `py_placer/place_optimize.py --max-displacement 3` (the quench, for ROUGH placements), `py_placer/place_reconstruct.py` (structural damage), `place_seed --repair` (local violations) / `--reseat` (parts far off), `py_placer/place_portfolio.py --intent --lock --full-probe` (on a SEEDED board), `py_placer/converge.py poses --ref X` (rank one part's poses), `py_placer/place_fanout_clearance.py` |
| placement vs routing, in a loop | `py_placer/place_route_loop.py` (when routing failed on congestion) |
| check a placement | `py_tools/check_assembly.py` (read `buildable`), `py_tools/check_floorplan.py --intent` (`--plan-only` before seeding; `--health` for escape lanes), `py_tools/render_placement.py --json-out` (then LOOK at the PNG; `--before <prev> --pair` diffs findings by name), `py_router/check_drc.py --clearance-margin 0` on a copper-free board |
| will it fit, can it escape | `py_tools/check_pockets.py`, `py_tools/check_channels.py --baseline <input> --gate`, `py_tools/check_capacity.py`, `py_tools/check_reachability.py --pad REF.PAD` |
| route | `py_router/route_planes.py` (pour first), `py_router/bga_fanout.py`, `py_router/qfn_fanout.py`, `py_router/route.py`, `py_router/route_diff.py`; `py_router/check_pads.py` before fanout. Not `py_router/repair_planes.py` (§4) |
| check a routed board | `py_router/check_connected.py`, `py_router/check_drc.py --baseline <input>`, `py_router/check_weird.py`, `py_tools/kicad_unconnected.py --items` (zone-aware oracle) |
| other skills | `plan-pcb-routing` (the routing recipe; its "Retrying a failed net" section), `diagnose-routing-failures`, `review-routed-board` (diff pairs, length, return vias) |

## 4. Traps measured in real runs (read these)

**Placement**
- **Don't polish a placement that measures clean.** If copper-free
  `check_drc` and `check_assembly` are clean, route it as it stands: the quench
  made a careful hand placement worse, causing 2 new routing failures.
- **Place and lock the connectors and mechanically fixed parts first**
  (`place_pose set … --rot`, then `lock`, or the plan's `fixed_poses`). Then
  zone the rest and seed. The seeder puts undeclared parts at their
  connectivity centroid in the first rotation that fits.
- **On a pile, rank the biggest IC's rotation before you keep a seed.** A
  pile part keeps its input rotation, which is a generator default, and
  `converge.py poses` cannot rank an IC's rotation once its decaps are packed
  against its pins (its `dropped_by` says what vetoed each move). Rank it at
  seed level, passing the same `--seed-args` you will seed with:
  `python3 -X utf8 py_placer/rank_rotations.py <pile> --intent <intent.json> --out-dir wk/<run>/rot --probe --write-intent wk/<run>/intent_rot.json`
  then seed from the written intent. `--jobs N` runs the control and the
  angles' seeds N at a time with identical results; on a board where one
  seed takes minutes, set it to your free cores. The seed holds that angle through its
  polish and re-seat: a part the re-seat cannot put back at it is named on a
  `NOT repaired` line (`reseat_declined`) and the seed exits 4, and
  `place_portfolio --intent` turns it only within its declaration (#1121).
  Without `--ref` it ranks the unlocked,
  undeclared part with the most connected pads. It costs one `place_seed`
  per angle plus one full-board probe per `--probe-top` angle (default 2).
  Run 39 found StickHub's U1 at 270 instead of the pile's 0 by hand: seed
  crossings 222 to 182, first-route blocking 31-37 to 16-19.
- **Rotation and pin order.** A `CROSSED` pin-order pair in `board_context`
  costs a via per net at every rotation. After rotating an IC, re-seat its
  caps: one rotation left a decap at 9.57 mm while crossings and hpwl both
  improved.
- **`place_pose` refuses a pad stack, but "legal" is still not
  "buildable".** Two parts' pad copper overlapping, any net, is
  `check_assembly`'s `pad_intersection`; `place_pose` measures it with
  the same function (#1064), so `legal` and `no_worse` see it and
  `--near` looks for a pose off it. A snap re-grades at most
  `--snap-tries` ranked poses, then as many nearer lattice ones, so it
  can refuse with a no-worse pose still in reach: when place_pose's snap
  census shows `candidates_tried` short of its `ranked` + `lattice`
  counts, raise `--snap-tries` / `--radius` before reading it as "stuck".
  Courtyards, bodies and coincident origins are
  still `check_assembly`'s alone: after every pose change, run it and
  read `buildable`, not `blocking`.
- **`render_placement`'s pad-clearance list is the grader's** (#1065): each
  pair is confirmed with `check_drc`'s exact pad check at the pose it
  draws, so it agrees with `grade_pad_legality`, and so does the caption's
  `pad-conflicts`; its `courtyard overlap` is the checklist's census
  (#1126). Two render numbers are still bounding-box counts: the
  pad-stack list (`b_body_overlap_pairs`; read stacks from
  `check_assembly`) and the JSON `metrics`, the optimizer's own currency.
- **Keep-out bands.** A pad in a `(keepout (tracks not_allowed))` band cannot
  be routed even on an empty board (#1031). Treat
  `checklist.a_off_outline.keepout_copper` like off-outline pad copper.
- **A module antenna needs its copper keep-out WRITTEN, after placement.** A
  design brief's `keepouts[]` grades placement only; no routing step reads it.
  Once the module is placed, write the area onto the board in the module's own
  frame, so a re-run after a move follows the part:
  `python3 -X utf8 py_router/add_rule_area.py <in> <out> --name ANT_KEEPOUT --ref U1 --rect X0 Y0 X1 Y1`
  (#1200). The router and KiCad then both keep copper out.
- **A killed `place_*` job leaves no board.** Bound it by SCOPE instead: free
  only the refs the gate names, and lock the rest. Freeing 2 parts cleared
  both blocking pairs in 63 s, where whole-board sweeps ran over 10 min. For
  parts tens of mm off, use `--reseat`: `--repair` ran 5 min and attempted
  none of 11.
- **Read `unseated_refs` after every `place_seed`.** A part in that
  list is NOT placed -- left where it came in, or staged below the board
  when its input pose stacked on a seated part
  (`unseated_disposition`) -- so seat it (`--repair`, or
  `place_pose.py`) before any route. The exit-4 line names these parts; run
  36 routed a board with C20 still in the pile and the router took GND off
  the board to reach it.
- **What the decap tools report:**
  - `place_seed --repair` counts a violator `repaired` only once its
    finding is gone; read `unresolved_refs` / `unresolved_by_rule` in its
    `JSON_SUMMARY` for the rest (#1066). Add `--repair-decaps` to seat
    charged caps at their IC's pin (opt-in; `decap_rung` says what it did).
    Pass `--baseline <input>` too: only then does the repair charge the
    courtyard pairs check_assembly gates (#1182), and on a library that
    draws bodies but no courtyards add `--body-model`, or the seat search
    spaces pad boxes and such a pair stays unresolved;
  - `place_fanout_clearance` holds both decap limits when you pass it
    `--intent` (#1067): no cap move takes a decap claim past its limit and
    further than before, unless no clear pose keeps it -- then the cap
    clears the foreign copper anyway and the claim it broke is named under
    `Decap limit broken`. When it broke a claim or left a cap grazing, it
    also runs the pass without the gate and keeps whichever ends with fewer
    unresolved grazes, then fewer decap claims made worse (`Decap: ...`
    says which).
    It prints the decap grade before and after. It also holds the
    intent's declared rotations in both passes (#1122); without
    `--intent` it can turn a cap whose angle you declared.
    Without `--intent` it can move a cap past `decap_pin_distance`
    silently;
  - `place_seed --reseat`'s intent basis counts only the rules it prints
    (`intent[...]`, `accept_basis.intent_rules`) -- decap and proximity
    included since #1068.

  Re-run `check_floorplan --intent` after each of these tools.
- **Re-placing strands routed copper.** Strip the copper and re-route; do not
  lean on `--allow-routed`.

**Routing**
- **Classify a routing failure before retrying it.**
  - Failed net's `blockers` empty, and `boxed_in[].geometry` above the
    board/fab floor: parameters (grid, ripup budget, width).
  - Geometry already at the floor, failures clustered at one part's escape
    face, or `check_reachability --pad` says CAGED: placement. Re-place,
    strip, re-route. A finer grid alone never widens a gap: grid
    0.05 → 0.0125 cost 40 min and left the same three nets unrouted.
- **`route.py`'s cleanup removes what `check_weird` calls removable** (#1063).
  Both grade by one predicate, so a plain `route.py` output carries no
  `removable-segment` or `redundant-cycle` finding on the nets that run
  cleaned. One that remains is copper the run did not own: a net outside its
  `--nets`, or input copper kept by `--keep-input-copper`.
- **`route.py`'s failure tally covers only the nets that step owns.** It
  re-grades the board it wrote over the nets its passes worked on or
  disturbed (#1069: the `JSON_REGRADE` line, merged into `JSON_SUMMARY_MIN`
  and `--json-out`), so reconciliation laps no longer hide broken nets. It
  does not grade the rest of the board, and it uses the router's fill model
  rather than KiCad's refill. Count the board's open nets with
  `check_connected` or `board_score`.
- **`route.py` finishes the planes; a repair step does not (#562, #1112).**
  Pour first (`route_planes.py`), then route with the plane nets inside
  `--nets` (`'*'` covers them). Pour-launch welds their pads, and the in-run
  plane finalize taps and joins what the fill cannot reach, at that step's
  own track and via sizes. A scoped round that names a few nets still lists
  the plane nets: its copper can cut a pour, the finalize repairs only plane
  nets in scope, and the improvement gate then reverts the round on the cut
  plane net (`JSON_IMPROVEMENT_GATE` says
  `rejected_on_excluded_plane_nets_alone`, #1114). A pour alone connects nothing, so if you pour
  after routing, end the chain on another `route.py --nets '*'` with the same
  size flags; the finalize runs even when that step has nothing else to
  route. Do not end on `repair_planes.py`: it cannot know the sizes you
  routed at, so it falls back to the board's net-class via and track.
- **Widths are requests.** After each route, read
  `power_widths.<net>.under_mm`: one run asked for 0.3 mm on +3V3 and shipped
  34 % of it at 0.127 mm. Grade power widths with `board_score --net-min-widths`.
- **Check DRC after any env change**, such as the routing skill's tuned knobs
  (`KICAD_GLOBAL_PLAN_RIVER` and the others).
- **Around a 0.8 mm BGA, the router can run out of lanes.** Two changes that
  measured wins on a 264-part board:
  - finer tracks and vias (`--track-width 0.0762 --clearance 0.0889 --via-size 0.25 --via-drill 0.15`);
  - a cheaper ground-plane layer cost (`--layer-costs`).

  Both go below the board's authored floors, so disclose that (§6). On later
  steps of a chain, pass `--clearance-ceiling`, not `--clearance`.
- **Hand-written copper:** stage each join with `py_tools/check_join.py` before
  committing it, and stamp it `(locked yes)`. Pad-edge arithmetic once made 42
  shorts, and the plane repair rips unlocked hand joins. A lock freezes the
  WHOLE net: one locked segment takes it out of every later rip and
  `--force-reroute`, with no override. Lock a join once its net is done, and
  unlock it in the board before re-routing that net.

## 5. Rules

1. **You choose the steps.** Do not re-create a staged driver. The stop rules
   and the verifier are the only process.
2. **Never change the board outline** (Edge.Cuts), and never move a part that
   draws it. The outline is the enclosure's, not the layout's.
3. **Stop rules:**
   - **First reach DONE**; nothing else before it.
   - **Watch long jobs.** Never leave a background job unwatched for more than
     20 minutes, and kill a search whose best has not improved in the last
     third of its run.
   - **After DONE, optimise in bounded rounds.** Stop when two consecutive
     rounds each cut vias by less than 5 % (in `place` mode: hpwl by less
     than 2 %).
   - **If DONE will not come:** after three consecutive DIFFERENT approaches
     that each fail to lower `unrouted` + `broken` (then the rest of
     `blocking`), measured, not argued, stop and ship the best board with
     every remaining blocker itemised with its measurement. When comparing:
     - compare `blocking` only between scores whose `ungraded` and
       `nets_analyzed` match;
     - a fanout RAISES `blocking` by construction, moving nets from unrouted
       to broken.
   - **Hard cap:** 10 hours of wall clock, unless the user sets another.
4. **An "impossible" claim needs its measurement.** "I tried A–F" is not a
   measurement: 9 of 14 such claims were later refuted. Quote
   `check_reachability` (PASSABLE is a router problem, CAGED is geometry) or
   the log block, not only its first refusal line.
5. **One verifier, at most 3 calls.** When you believe you have your best
   board, spawn ONE fresh subagent (not a fork) with
   `references/verifier.md`, filled in for your mode, board, sha256, baseline,
   intent and spec flags. On FAIL, fix what it names and continue. Spawn no
   other subagents unless the user asks.
6. **Record milestones for the film:** the first legal placement, each kept
   placement, the first routed board, the first DONE, each improvement, the
   final board, and tried-and-worse boards with `--rejected`.
   ```bash
   python3 -X utf8 py_tools/board_score.py <board> --intent <i> --baseline <input> --json <board>.score.json --quiet
   python3 -X utf8 py_placer/converge.py record --ledger wk/<run>/ledger.jsonl \
       --board <board> --kind placement --parent <the board it was made from> \
       --lever "<what you did, one line>" --score-file <board>.score.json
   ```
   - Use `--kind completion` for a routed board. The film's benchmark band
     is drawn from these rows.
   - `record` refuses (exit 2) a score whose `blocking` is not a non-negative
     number, such as a per-term dict; record board_score's own JSON.
   - **Close the ledger** with one `record --final --stop-condition <1|2|3|4>`
     row, passing the verifier's per-lens files as `--lens-file`.

## 6. Hand-back

1. **The board**, with its `.kicad_pro`, and its sha256.
2. **The film**:
   ```bash
   python3 -X utf8 py_tools/make_film.py --from-ledger wk/<run>/ledger.jsonl \
       --theme light --aspect 4:3 -o wk/<run>/<run>_film.mp4
   ```
   Look at a few frames before you call it done. Its last line says which
   board box the film got: `make_film: WARNING board box: 2D X-ray` means
   the 3D board did not render (in a fresh worktree, run `npm ci` in
   `py_router/stage3d` and film again).
3. **`wk/<run>/REPORT.md`**, containing:
   - **the result first:** a table of the mode's DONE conditions with measured
     values, the `blocking_by` breakdown, and vias / copper_mm / segments for
     routed modes;
   - **which stop rule fired**, and every remaining blocker BY NAME with its
     measurement;
   - **floors:** where the run went below the board's authored or fab floors.
     Quote route.py's `Design rules [...]` line and every floor
     `check_complete --authored-from <input>` lists;
   - **a timeline** from your first command: first legal placement, first
     fully routed board, first DONE, final board, verifier call(s);
   - **where the time went:** working vs waiting on long jobs;
   - **what worked and what didn't**, each with its measurement;
   - **tool gaps**, including any tool that said "fine" about something that
     was not; check the open issues first (`gh issue list --search`);
   - **each verifier verdict.**
4. **Independent grade and timing**, for the report's own table, never
   replacing the verifier:
   ```bash
   python3 -X utf8 .claude/skills/pcb-free-agent/scripts/grade.py <board> --baseline <input> --intent <i> --mode <mode>
   python3 -X utf8 .claude/skills/pcb-free-agent/scripts/measure.py --root .
   ```
   Add `--spec <name>=<value>` to `grade.py` for each spec flag the board
   declares (e.g. `--spec net-min-widths=widths.json`).
