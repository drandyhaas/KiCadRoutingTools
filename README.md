# KiCad Track Gloss

This branch contains the standalone **KiCad Track Gloss** ActionPlugin for
KiCad 10. Select one or more straight PCB track segments and run the plugin: it
expands each selected connection, searches for a shorter or simpler safe
0/45/90-degree route, applies the result directly to the current board, and
then exits. KiCad Undo remains the normal way to reject a result.

The separate diagnostic action reports affected nets, length and segment
gains, optimization mechanisms, rejected candidates, and machine-readable
JSON. Normal operation remains silent unless no modification is possible.
Launching the normal action without any straight segment selected displays a
single focused selection warning; an eligible no-op still uses only KiCad's
warning bell.

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
- `codex/`: isolated experimental Codex agent, prompt, scope/result schemas,
  examples, and executable-ready launcher.

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
```

For `place_route_loop`, add `--place-route-loop` to the acceptance command; the
loop-provided routed PCB is graded and the placed PCB plus `route.json` are
recorded in `GLOSS_SCORE_JSON`. The input board is never modified or saved.
Writing a converged board requires the explicit `--output` option; the CLI
refuses to overwrite its input.
This is a quality score only and must be combined with separate DRC,
connectivity, and specification checks. Full CLI details and an acceptance
command example are in
[`kicad_track_gloss/README.md`](kicad_track_gloss/README.md#headless-score-cli-and-place_route_loop).

## Validation

Run the API-neutral suite with Python 3.12 and `pytest`:

```powershell
py -3.12 -m pytest tests/track_gloss/unit -q
```

Run the frozen real-board regression with KiCad's Python interpreter:

```powershell
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py --all-orders --full-sweep
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
