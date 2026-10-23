# Placement A/B: route placed boards by their recorded chain

A placement is judged by routing it (CLAUDE.md, "What a placement run is
FOR"): place each arm over seeds, route every placed board the way the stress
corpus routed the original, and compare the routed results paired per board
and seed, with blocking first, then `joins_needed` (the joins the open nets
still need: a missing rail is not fewer vias), then vias, copper_mm and
segments.

- **`route_placed.py PLACED.kicad_pcb BOARD SET OUT_DIR`** routes one placed
  board on this machine. It replays the corpus board's recorded chain
  (`runs_set<SET>/<BOARD>/redo_commands.sh`, under `--corpus`, default
  `$KICAD_STRESS_CORPUS`; building the corpus is in `tests/stress/RUNBOOK.md`)
  on the placed board and scores the final board with `board_score`.
  `--quick` adds route.py's fast judge to the chain's whole-board route
  steps. `--quick 2`, the seed judge, also passes it to every
  `route_diff.py` step and scores with `py_tools/quick_score.py` (the
  connectivity of the nets without a zone, and board_score's quality counts),
  as the placement skill does.
- **`place_modal.py OUT BOARDS SEEDS`** places a whole arm on Modal: every
  board and seed in its own container, each `place_simple.py --seed N` of the
  tree being measured (`--src`), into `OUT/<key>/s<seed>.kicad_pcb` -- where
  `route_modal.py` looks for them. BOARDS is a comma list of
  `key=BOARD.kicad_pcb:GROUPS.json:INTENT.json`; use `route_modal.py`'s keys.
  Arguments after `--` go to every `place_simple.py`. Place every arm of an
  A/B there (Linux and the KiCad image's Python), never some on a Mac.
- **`route_modal.py PLACED_DIR BOARDS SEEDS --out OUT`** routes a whole arm on
  Modal with `route_placed.py --quick` (`--quick 2` for the seed judge,
  `--full` for the whole chain, the final route of a chosen placement), one
  container per board and seed. BOARDS are keys of its table (`ch32`,
  `noahfc`, `fusb`, `tig`, `bitaxe`, `z80`), or `key=<corpus board>:<set>`
  for any other corpus board. Every route runs at once, so an arm takes about
  as long as its slowest route.

```bash
# place each arm with the tree it measures (--src), from the same inputs
python3 tests/stress/placement_ab/place_modal.py arms/base \
    tig=fixed.kicad_pcb:groups.json:intent.json 0-7 --src ../base_tree
python3 tests/stress/placement_ab/place_modal.py arms/new \
    tig=fixed.kicad_pcb:groups.json:intent.json 0-7
# route both arms with one tree (this checkout): arms/<arm>/tig/s<seed>.kicad_pcb
python3 tests/stress/placement_ab/route_modal.py arms/base tig 0-7 --out routed
python3 tests/stress/placement_ab/route_modal.py arms/new  tig 0-7 --out routed
# routed/base_tig_s0/score.json, ... routed/new_tig_s7/score.json
```

Rules that keep the comparison honest:

- **Compare Modal routes only with Modal routes.** Modal runs Linux, the
  KiCad image's Python and its KiCad (`kicad/kicad:10.0.0`; set
  `KRT_PAB_KICAD_IMAGE` for another), so route the baseline arm there too.
- **The tree that routes is `--src`**: this checkout by default, *as it
  stands*. Uncommitted and untracked-but-not-ignored files are shipped;
  ignored ones (work dirs, built libraries) are not. Pass a worktree to route
  with its router. The deployed app is named after a hash of everything it
  ships, so runs over the same tree share one app and a changed tree gets its
  own.
- The app is deployed when it is missing and **stopped automatically** once
  nothing spawned on it is pending (`~/.cache/krt_placement_ab/apps.json`
  records who waits on which app).
- **Rerunning a command resumes it.** Finished routes are skipped, and calls
  still running are collected rather than respawned. A call outlives the
  client, so a dropped link loses nothing.
- `route_modal.py` makes no placements: place first (`place_modal.py`), then
  route. Each placement's `.kicad_pro` and `.kicad_prl` ride along.
