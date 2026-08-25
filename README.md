# KiCad Track Gloss

This branch contains the standalone **KiCad Track Gloss** ActionPlugin for
KiCad 10. Select one or more straight PCB track segments and run the plugin: it
expands each selected connection, searches for a shorter or simpler safe
0/45/90-degree route, applies the result directly to the current board, and
then exits. KiCad Undo remains the normal way to reject a result.

The separate diagnostic action reports affected nets, length and segment
gains, optimization mechanisms, rejected candidates, and machine-readable
JSON. Normal operation remains silent unless no modification is possible.

## Repository layout

- `kicad_track_gloss/`: complete plugin source, PCM metadata, icons, licence,
  documentation, and package builder.
- `tests/track_gloss/unit/`: API-neutral engine and packaging tests.
- `tests/track_gloss/patterns/`: frozen KiCad board/project/rule fixture.
- `tests/track_gloss/run_patterns.py`: KiCad-Python integration, determinism,
  and full-scope regression runner.
- `tools/diagnose_track_gloss_board.py`: read-only headless board diagnosis.

The implementation does not import or require the autorouter modules from the
main KiCadRoutingTools branch. The generated PCM archive is self-contained.

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
