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

## Command/input identity findings (2026-07-08)

- Harness bug found by asking 'were the commands the same?': the GUI leg
  hardcoded ordering_strategy='inside_out' while the real GUI default (and
  CLI default) is 'mps'. Fixed; the two legs now run the same effective
  parameters.
- With commands matched: 381/~1200 segments common. With the
  GUI_PARITY_INPUT=parser diagnostic (GUI leg text-parses a temp-saved
  board, isolating the builder-vs-parser representation): 444/~1200.
- Conclusion: the dominant copper fork is NOT parameters and NOT coordinate
  representation -- it is item ORDERING (pcbnew re-save normalizes element
  order; MPS ordering, spatial-index insertion and A* tie-breaks all follow
  it) plus the CLI chain's per-step .kicad_pro floor carryover. Grade
  parity is unaffected. Bit-identical copper would require engine-level
  order canonicalization (sort nets/pads/segments deterministically before
  routing) -- a follow-up decision, not a bug.

## Parameter-identity verification (KICAD_DUMP_BATCH_KWARGS)

route.py's batch_route dumps its FULL parameter set (76 keys) and returns
when KICAD_DUMP_BATCH_KWARGS=<file> is set -- diffing a CLI invocation
against the GUI leg's call verifies the two fronts hand the engine the
same parameters. Current state: only `return_results` (definitional) and
`layer_costs` [] vs None (proven equivalent via get_layer_costs) differ.

The probe found two phantom forks in this harness itself (both the exact
CLAUDE.md 'defaults must match in both places' class): ordering_strategy
'inside_out' vs the real mps default, and track_proximity_cost 0.2 vs the
real 0.0. Copper-overlap ladder on splitflap as each fork was removed:

    300/1200 segments common   (initial)
    381/1200                   (+ ordering matched)
    914/1200, 135/165 vias     (+ track_proximity_cost matched = all params)
    1168/1190, 164/165 vias    (+ GUI_PARITY_INPUT=parser: same text-parsed
                                representation -- 98.2% identical copper)

Residual ~2%: pcbnew re-save normalization (element order/precision of the
temp file vs the original bytes), per-step .kicad_pro floor carryover, and
in-memory vs file-based post-pass sequencing. The production GUI path
(builder representation) sits at ~77% copper identity with full grade
parity; closing it to ~98% is the order-canonicalization follow-up.
