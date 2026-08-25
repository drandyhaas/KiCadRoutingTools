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
py -3.12 -m pytest tests/track_gloss/unit tests/test_smooth_route.py -q
```

Run real-board validation without saving any PCB:

```text
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py
```

The default real-board run evaluates the all-selected board once. The costly
order-independence replay is deliberately suspended from routine validation.
Run its seven input orders only when specifically investigating determinism:

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

Boards tested manually should not be added automatically. When a board becomes
a useful non-regression case, copy its `.kicad_pcb` and, when available, its
matching `.kicad_pro` and `.kicad_dru` into a named directory under `patterns/`.
Record SHA-256 fingerprints, keep Git line-ending conversion disabled for the
fixture, load it only in memory, and add explicit expected results to the
runner. Never overwrite a pattern during validation.
