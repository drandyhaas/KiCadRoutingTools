# KiCad Track Gloss

KiCad Track Gloss is a standalone KiCad 10 ActionPlugin that shortens and
simplifies selected PCB tracks. Select one or more straight segments and run
the plugin: it expands their connections, searches for a shorter or simpler
safe 0/45/90-degree route, validates the composed result, and applies it to the
current board as one KiCad Undo operation.

Successful glosses are silent. The plugin does not save the board, create
before/after files, show a preview, or request confirmation. If no safe change
is possible, KiCad plays its standard warning sound once.

## Documentation

- [Using the plugin](docs/plugin-usage.md)
- [Command-line interface](docs/cli.md)
- [Configuration reference](docs/configuration.md)
- [Machine-readable output contracts](docs/output-contracts.md)
- [Safety and native KiCad DRC](docs/safety-and-drc.md)
- [Architecture](docs/architecture.md)
- [Regression-test runbook](tests/track_gloss/README.md)

The diagnostic action reports affected nets, saved copper, segment reduction,
optimization mechanisms, rejected candidates, native DRC status, timings, and
machine-readable JSON. Run either action with no selected straight segment to
edit the current KiCad session settings.

Safety takes precedence over latency. Depending on session policy, the final
plan is checked by KiCad's native DRC on private before/after board snapshots.
Starting the helper processes, refilling zones, and running DRC can therefore
add seconds even for a single connection. The current board is not saved or
modified by validation.

Performance policy is scope-aware: one-connection glosses prioritize minimum
latency; larger selections replay those exact local scopes and use their
configured maximum time budget to retain the best safe composition found so
far. A rejected connection does not cancel safe improvements elsewhere, even
when both connections belong to the same net.

## Repository layout

- `kicad_track_gloss/`: complete ActionPlugin and PCM package source;
- `kicad_track_gloss/engine/`: API-neutral optimization engine;
- `kicad_track_gloss/kicad/`: `pcbnew`, UI, rules, DRC, and board-write adapter;
- `docs/`: user, CLI, configuration, output, safety, and architecture guides;
- `tools/score_track_gloss.py`: headless score/gloss CLI compatible with
  `place_route_loop`;
- `tools/diagnose_track_gloss_board.py`: read-only board diagnosis utility;
- `tests/track_gloss/unit/`: fast API-neutral and packaging tests;
- `tests/track_gloss/patterns/`: frozen KiCad board/project/rule fixtures;
- `tests/track_gloss/run_patterns.py`: KiCad-Python integration runner.

## Quick CLI example

The CLI defaults to all admissible straight tracks and never writes its input:

```powershell
D:\kicad\bin\python.exe tools\score_track_gloss.py design.kicad_pro
D:\kicad\bin\python.exe tools\score_track_gloss.py design.kicad_pro --output glossed.kicad_pcb
```

It emits a structured score and ends with the acceptance contract used by
KiCadRoutingTools:

```text
SCORE=<post-gloss straight-track copper length in mm>
```

Lower is better. This is a routing-quality metric, not a substitute for DRC,
connectivity, or functional checks. See the [CLI guide](docs/cli.md).

## Validation and packaging

Run the fast API-neutral suite:

```powershell
py -3.12 -m pytest tests/track_gloss/unit -q
```

Run real-board integration with KiCad's Python:

```powershell
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py
```

The full connection sweep is opt-in:

```powershell
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py --full-sweep
```

Build the Plugin and Content Manager archive:

```powershell
py -3.12 kicad_track_gloss\package_pcm.py
```

Artifacts are written under `dist/`.

## Provenance

This plugin is derived from the work and source code of
[DrAndyHaas/KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools)
and is **inspired by and reusing part of DrAndyHaas's code**. Reused and
adapted smoothing algorithms, implementation patterns, original copyright, and
MIT licensing are retained.

The standalone plugin integration and subsequent modifications were produced
with ChatGPT/Codex (OpenAI) at the project owner's direction. The project is
maintained by Frantz. See [NOTICE](kicad_track_gloss/NOTICE) and
[LICENSE](LICENSE).
