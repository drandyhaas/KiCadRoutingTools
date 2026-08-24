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

Boards tested manually should not be added automatically. When a board becomes
a useful non-regression case, copy its `.kicad_pcb` and, when available, its
matching `.kicad_pro` and `.kicad_dru` into a named directory under `patterns/`.
Record SHA-256 fingerprints, keep Git line-ending conversion disabled for the
fixture, load it only in memory, and add explicit expected results to the
runner. Never overwrite a pattern during validation.
