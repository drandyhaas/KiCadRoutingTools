# run32 fixtures: glasgow_revC placed and routed from an off-board pile

Boards from run 32 (discussion #118, report in `docs/runs/run32/README.md`). Both are
expensive to regenerate, so they are committed: `placed_v2` is a seed plus three placement
passes (~15 min), and `routed_c3` is the end of a 32-hour place-and-route loop. The run's
input pile itself is not here, because `kicad_files/glasgow_revC.kicad_pcb` plus run31's
`make_unplaced.py` recipe rebuilds it.

Each board ships with its sibling `.kicad_pro` and `.kicad_prl` (CLAUDE.md #441). The
project carries the DRC floor the board was routed to.

| file | what it is | used by |
|---|---|---|
| `placed_v2.kicad_pcb` | placement cycle 2: `check_assembly` says buildable, yet 17 signal pads sit in the board's F/B rule-area keepout band | repro for #1031 |
| `routed_c3.kicad_pcb` | the shipped board: 0 unrouted, 19 broken joins over 17 nets (KiCad oracle), DRC 0 at the routed 0.1 mm, 2 TRACK-HOLE at J5 when graded at the declared 0.25 mm copper-to-hole | repro for #1033 (power-net widths), #1038 (copper-to-hole) |
| `glasgow.intent.json` | the compiled floorplan intent the run was graded against | `check_floorplan`, `place_portfolio` (#1037) |
| `glasgow.design-brief.json` | the design brief it was compiled from | the placement skill's P1 |
| `keepout_census.py` | read-only census of pads whose copper reaches a `tracks_allowed False` rule area (margin: clearance + half track) | #1031 |

```bash
python3 -X utf8 tests/fixtures/run32/keepout_census.py tests/fixtures/run32/placed_v2.kicad_pcb
#   ... SMD signal (non-GND, netted): 17        (inside or within margin of the band)
python3 -X utf8 py_tools/check_assembly.py tests/fixtures/run32/placed_v2.kicad_pcb --clearance 0.2
#   VERDICT: buildable (blocking 0)             <- the gap #1031 describes

python3 -X utf8 tests/fixtures/run32/keepout_census.py tests/fixtures/run32/routed_c3.kicad_pcb
#   ... SMD signal (non-GND, netted): 5         (all NEAR the band, 0.0-0.25 mm; none inside)
python3 -X utf8 py_router/check_drc.py tests/fixtures/run32/routed_c3.kicad_pcb --clearance 0.1 --hole-clearance 0.25
#   TRACK-HOLE violations (2)                   <- #1038
```

The other repro boards named in #1032–#1036 (`K3A_it1_g`, `K3C_diff`, `K3C_route`,
`placed_v3`) are in the `repro/run32/` folder of the `run32-assets` branch on
`edgehero/KiCadRoutingTools`. They were left out here to keep the tree small.
