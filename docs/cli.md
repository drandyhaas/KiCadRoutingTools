# Track Gloss command-line interface

`tools/score_track_gloss.py` applies the same fixed-point optimization engine
as the plugin to a KiCad board in a headless process. It can score a route
without writing it, create a separate glossed board, or act as an external
judge for KiCadRoutingTools `place_route_loop`.

The CLI must run with a Python interpreter able to import the matching KiCad
`pcbnew` module.

## Direct use

```powershell
D:\kicad\bin\python.exe tools\score_track_gloss.py design.kicad_pro
D:\kicad\bin\python.exe tools\score_track_gloss.py candidate.kicad_pcb --project design.kicad_pro
D:\kicad\bin\python.exe tools\score_track_gloss.py candidate.kicad_pcb --project design.kicad_pro --output glossed.kicad_pcb
```

A positional `.kicad_pro` selects its same-stem `.kicad_pcb`. A positional
`.kicad_pcb` automatically uses a same-stem project when present; `--project`
can instead supply the rules for a differently named candidate.

Without `--output`, the input is loaded and optimized in memory only. With
`--output`, a new `.kicad_pcb` is written. The CLI never overwrites its input
and refuses an existing destination unless `--force` is present.

## Scope

The default scope is `ALL`: every admissible straight track is used as a seed.
Narrower experiments can repeat `--scope`:

```powershell
# Exact net name
D:\kicad\bin\python.exe tools\score_track_gloss.py board.kicad_pcb --scope "net:VCC"

# Exact KiCad segment UUID
D:\kicad\bin\python.exe tools\score_track_gloss.py board.kicad_pcb --scope "segment:01234567-89ab-cdef-0123-456789abcdef"
```

A scope manifest uses the following form:

```json
{
  "scopes": ["net:VCC", "net:GND"]
}
```

Pass it with `--scope-file scope.json`. `ALL` cannot be combined with narrower
scopes. Net matching is exact and case-sensitive so an automation cannot
silently grade the wrong subset.

## Convergence and performance

- `--max-passes N` sets the hard changed-pass guard. The packaged CLI default
  is 16.
- `--time-budget SECONDS` bounds total planning and DRC time. The packaged CLI
  default is unlimited because it is intended for offline automation.
- `--no-parallel` disables independent net/layer worker processes.
- `--trace-passes` sends one `GLOSS_PASS_JSON=` record per convergence state to
  stderr, including the terminal fixed-point, limit, or timeout state.

Plugin and CLI call the same planner and convergence orchestration. They have
different default time budgets: the plugin favors interactive responsiveness;
the CLI favors a complete fixed point. Selecting all admissible tracks in the
plugin and using CLI scope `ALL` therefore describe the same optimization,
provided neither run is stopped by its interactive budget.

## JSON file output

`--json-out result.json` writes the same canonical payload emitted on stdout as
`SCORE_JSON=`. This follows the KiCadRoutingTools convention for tools that
write a machine-readable result to a path. The PCB destination remains
`--output`; the two options are independent.

## place_route_loop

KiCadRoutingTools appends three positional paths to an `--accept-cmd`:

```text
PLACED.kicad_pcb ROUTED.kicad_pcb ROUTE.json
```

Use Track Gloss as follows:

```powershell
python py_placer\place_route_loop.py ... --accept-cmd "D:\kicad\bin\python.exe tools\score_track_gloss.py --place-route-loop"
```

The routed board is scored; the placed board and route JSON paths are retained
in the structured payload. `--output` is forbidden in this mode because an
acceptance judge must not mutate the candidate supplied by the loop.

The final stdout line is always:

```text
SCORE=<float>
```

The score is the virtual post-gloss total straight-track copper length in
millimetres. Lower is better. It is a quality metric, not a substitute for DRC,
connectivity, impedance, length matching, or functional requirements.

## Exit status

- `0`: the board was successfully evaluated, including a valid no-op;
- `1`: invalid input, unavailable KiCad runtime, planning/validation failure,
  or inability to write a requested output;
- argparse may use its standard non-zero usage status for malformed options.

Whether copper changed, whether a fixed point was reached, and whether native
DRC accepted a candidate are fields in the JSON payload rather than separate
process exit codes. See [Output contracts](output-contracts.md).
