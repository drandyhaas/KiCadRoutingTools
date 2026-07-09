# GUI/CLI engine-parity harness

Measures whether the claude tab's "run selected steps" (GUI engine calls on
the live pcbnew board) produces the same final board as the recorded stress
chain's CLI steps run file-to-file.

- `test_gui_engine_parity.py` — headless: CLI leg via subprocesses, GUI leg
  inside KiCad's bundled python (auto re-exec) driving the real tab apply
  methods on shimmed dialog instances. Prints a parity report; known
  deliberate divergences are listed in the module docstring.

Workdir: `tests/gui_parity/work/` (gitignored).

## Comparison bars

1. **Grade parity** (the harness VERDICT): fully-connected, DRC-clean, and
   kicad-cli-refilled unconnected counts equal. This is the acceptance bar
   the stress tests themselves use.
2. **Copper-set identity** (the `COPPER-SET COMPARISON` block): segments and
   vias as canonical UUID-independent sets. Byte equality is meaningless by
   design (.kicad_pcb outputs carry per-run random UUIDs -- see project
   notes: never whole-file-diff routing outputs).

Current measurement on splitflap: grade PARITY, copper sets DIFFER
(~300/1200 segments common, same via count at different positions). The
router is deterministic, so the fork is input-representation differences:
the GUI leg routes PCBData built from pcbnew (float coordinates from nm
conversion, container orderings) while the CLI parses the file text --
A* tie-breaks diverge on the first sub-micron difference, and the .kicad_pro
floor carryover differs per step. Both outputs are equally valid; making
them bit-identical would require a canonical PCBData representation shared
by both fronts (open follow-up).
