# KiCad Track Gloss

This branch contains the standalone **KiCad Track Gloss** ActionPlugin for
KiCad 10. Select one or more straight PCB track segments and run the plugin: it
expands each selected connection, searches for a shorter or simpler safe
0/45/90-degree route, applies the result directly to the current board, and
then exits. KiCad Undo remains the normal way to reject a result.

The standalone adaptation was created by ChatGPT/Codex (OpenAI), inspired by
and reusing part of DrAndyHaas's code, and is maintained by Frantz. The
original copyright, MIT license, algorithms, and code provenance are retained.

The separate diagnostic action reports affected nets, length and segment
gains, optimization mechanisms, rejected candidates, and machine-readable
JSON. Normal operation remains silent unless no modification is possible.
Launching the normal or diagnostic action without any straight segment
selected opens a session-settings dialog. **Close** applies the edited values
until KiCad exits; **Cancel** leaves them unchanged. Each setting has a tooltip
explaining its effect. An eligible no-op still uses only KiCad's warning bell.

Safety takes precedence over latency. The gloss safety system uses KiCad's
native DRC on private before/after board snapshots. Starting KiCad's DRC
processes can therefore dominate response time and add seconds even when only
one connection is selected. This validation is intentional; neither source
board nor zone fills are modified by the check.

## Repository layout

- `kicad_track_gloss/`: complete plugin source, PCM metadata, icons, licence,
  documentation, and package builder.
- `tests/track_gloss/unit/`: API-neutral engine and packaging tests.
- `tests/track_gloss/patterns/`: frozen KiCad board/project/rule fixture.
- `tests/track_gloss/run_patterns.py`: KiCad-Python integration, determinism,
  and full-scope regression runner.
- `tools/score_track_gloss.py`: read-only whole-board score CLI compatible
  with `place_route_loop`.
- `tools/diagnose_track_gloss_board.py`: read-only headless board diagnosis.

The optimization engine is deliberately independent from `pcbnew`. Candidate
search remains in `engine/planner.py`; exact local copper/clearance checks are
isolated in `engine/candidate_geometry.py`; reusable spatial indexes (including
Edge.Cuts indexing) live in `engine/context.py`; and the plugin/CLI candidate
ladder is shared by `engine/workflow.py`. KiCad-specific board conversion,
native DRC, report parsing, dialogs, and writes remain under `kicad/`.

The implementation does not import or require the autorouter modules from the
main KiCadRoutingTools branch. The generated PCM archive is self-contained.

## Scoped CLI score

The repository also provides a read-only CLI for automated routing loops. It
uses the same fixed-point function as the plugin and prints a final
`SCORE=<float>` line compatible with `place_route_loop --accept-cmd`. The score
is the virtual post-gloss total straight-track copper length in millimetres,
so a lower value is better. The default scope is `ALL`; narrower experiments
use repeatable `--scope net:<name>`, `--scope segment:<uuid>`, or
`--scope-file manifest.json` arguments.

```powershell
D:\kicad\bin\python.exe tools\score_track_gloss.py design.kicad_pro
D:\kicad\bin\python.exe tools\score_track_gloss.py --project design.kicad_pro candidate.kicad_pcb
D:\kicad\bin\python.exe tools\score_track_gloss.py --scope net:VCC --project design.kicad_pro candidate.kicad_pcb
D:\kicad\bin\python.exe tools\score_track_gloss.py --project design.kicad_pro --output glossed.kicad_pcb candidate.kicad_pcb
D:\kicad\bin\python.exe tools\score_track_gloss.py --max-passes 32 --trace-passes --project design.kicad_pro candidate.kicad_pcb
D:\kicad\bin\python.exe tools\score_track_gloss.py --time-budget 900 --project design.kicad_pro candidate.kicad_pcb
```

For `place_route_loop`, add `--place-route-loop` to the acceptance command; the
loop-provided routed PCB is graded and the placed PCB plus `route.json` are
recorded in `GLOSS_SCORE_JSON`. The input board is never modified or saved.
Writing a converged board requires the explicit `--output` option; the CLI
refuses to overwrite its input.
`--max-passes N` controls the hard convergence guard (default 16), while
`--trace-passes` emits machine-readable `GLOSS_PASS_JSON` records to stderr
for convergence analysis without changing the final stdout score contract.
The CLI uses the complete fixed-point search intended for offline scoring and
has no time limit by default. Automated callers can add `--time-budget N` to
set an explicit total planning-and-DRC budget in seconds; this is independent
from `--max-passes`. The interactive plugin has a separate responsiveness
guard: 5 seconds for planning, 10 seconds for the complete operation, four
global reconciliation passes, and two local passes per independent batch. If
an interactive guard is reached, only an already validated partial improvement
may be applied; the CLI instead reports that its requested fixed point was not
reached.
This is a quality score only and must be combined with separate DRC,
connectivity, and specification checks. Full CLI details and an acceptance
command example are in
[`kicad_track_gloss/README.md`](kicad_track_gloss/README.md#headless-score-cli-and-place_route_loop).

Engine policy values are centralized in
`kicad_track_gloss/internal_config.json`. This packaged JSON records validated
defaults for minimum saving, interactive and CLI time and convergence limits,
and one-track DRC. The no-selection settings dialog can override the
interactive values in memory for the current KiCad session; it never rewrites
the packaged JSON.

## Validation

Run the API-neutral suite with Python 3.12 and `pytest`:

```powershell
py -3.12 -m pytest tests/track_gloss/unit -q
```

Run the frozen real-board regression with KiCad's Python interpreter:

```powershell
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py --all-orders --full-sweep
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py --segment-subdivisions
```

Build the Plugin and Content Manager archive:

```powershell
py -3.12 kicad_track_gloss\package_pcm.py
```

The archive is written to `dist/KiCadTrackGloss-<version>.zip`.

See [`kicad_track_gloss/README.md`](kicad_track_gloss/README.md) for the engine
contracts, KiCad integration details, release structure, and maintenance rules.

## Attribution

This plugin is directly derived from the work and source code of
[DrAndyHaas/KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools).
It contains reused and adapted smoothing code, algorithms, and implementation
patterns by DrAndyHaas. The standalone plugin integration and subsequent
changes were produced with ChatGPT/Codex. Full notices are in
[`kicad_track_gloss/NOTICE`](kicad_track_gloss/NOTICE).

MIT licensed. See [`LICENSE`](LICENSE).
