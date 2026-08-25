# Track Gloss validation

This directory owns all tests and real-board patterns for the standalone
KiCad Track Gloss plugin.

```text
track_gloss/
├── unit/                 API-neutral and mocked-KiCad pytest tests
├── patterns/             frozen KiCad board/project/rule inputs
│   └── dispenser_labels/
└── run_patterns.py       integration replay under KiCad's Python
```

Run the fast tests from the repository root:

```text
py -3.12 -m pytest tests/track_gloss/unit -q
```

Run real-board validation without saving any PCB:

```text
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py
```

The default real-board run evaluates the all-selected board to its shared
fixed point once. The costly order-independence replay is deliberately
suspended from routine validation. Run its seven input orders only when
specifically investigating determinism. Equality checks order independence,
not mathematical optimality:

```text
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py --all-orders
```

The exhaustive generation of every connection scope and the corresponding
fresh-board applications are also suspended by default. Enable that separate
deep check only when changing the planner or KiCad writer:

```text
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py --full-sweep
```

Both optional checks can be combined when a complete deep validation is
actually required.

Artificial collinear subdivision invariance is another deliberately optional
alpha/deep check. It splits every eligible real-board segment into halves and
then thirds wherever the artificial cut is not a pad, via, or track-junction
anchor. It converges all representations independently, removes artificial
degree-two breakpoints, and requires the complete final copper geometry and
saved length to be identical:

```text
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py --segment-subdivisions
```

This check is disabled during routine release validation, just like the seven
input-order replay, because it performs two additional all-board fixed-point
searches.

Smoke-test the all-track read-only score CLI with the frozen board, project,
and design rules:

```text
D:\kicad\bin\python.exe tools\score_track_gloss.py --project tests\track_gloss\patterns\dispenser_labels\dispenser_labels.kicad_pro tests\track_gloss\patterns\dispenser_labels\dispenser_labels.kicad_pcb
```

The final stdout line must be `SCORE=1045.370136582`. The preceding
`GLOSS_SCORE_JSON=` document must report 706 selected seeds, 590 eligible
tracks, 116 protected tuned tracks, two changed convergence passes,
64.073413144 mm of actual copper saved, and 32 segments saved. Saving with `--output`
and rescoring that output must report zero changed passes and the same score.
The fixture must remain byte-for-byte unchanged.

Boards tested manually should not be added automatically. When a board becomes
a useful non-regression case, copy its `.kicad_pcb` and, when available, its
matching `.kicad_pro` and `.kicad_dru` into a named directory under `patterns/`.
Record SHA-256 fingerprints, keep Git line-ending conversion disabled for the
fixture, load it only in memory, and add explicit expected results to the
runner. Never overwrite a pattern during validation.
