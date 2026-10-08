# Verifier brief (fill in the angle-bracket fields)

You are the independent verifier for a board another agent says is finished.
Trust nothing it told you. Measure it yourself, from the repo root.

- **Mode:** `<full | place | route>`
- **Board:** `<BOARD PATH>`
- **Claimed sha256:** `<SHA256>`
- **Input board (baseline):** `<INPUT BOARD>`
- **Intent:** `<INTENT JSON, or "none">`
- **Spec flags the board declares:** `<e.g. "--net-min-widths widths.json --impedance-nets USB_D*", or "none">`
- **Mechanical facts to check:** `<e.g. "MK1-4 centred 4.0 mm from each corner; J1 on the west edge", or "none">`
- **Write your verdict to:** `<RUN DIR>/verdict_<N>.txt`

## Steps

1. **Identity.** Recompute the board's sha256. If it differs, write
   `VERDICT=FAIL` with reason `sha mismatch` and stop.
2. **Rules.** The board must have a sibling `.kicad_pro`. Grade at the rules it
   declares, never at a guessed value.
3. **Every mode** (read the whole output, not the first line):
   - `python3 -X utf8 py_tools/check_assembly.py <BOARD> --intent <INTENT>`: must be buildable. It names
     pad copper past the outline and any part on a PCB-edge plug's mating region (`mating_keepout_refs`);
     name each one in your verdict. A part still in the staging pile is pad copper off the outline, so it is
     in `oob_pad_copper_gating_refs`; any such part on the board you grade is a FAIL. (`place_seed` names its
     unseated parts in its `JSON_SUMMARY` line, `unseated_refs`, and its exit-4 message, not in the ledger.)
   - `python3 -X utf8 py_tools/check_floorplan.py <BOARD> --intent <INTENT> --allow-routed`: 0 errors;
   - `python3 -X utf8 py_tools/render_placement.py <BOARD> --json-out <RUN DIR>/verify_<N>_render.json`:
     `checklist.a_off_outline` must list no gating pad copper (`pad_copper_gating`), graphic or keep-out (`keepout_copper`) copper; LOOK at the PNG;
   - the mechanical facts above, measured from the board.
4. **`full` and `route` modes, also:**
   - `python3 -X utf8 py_tools/board_score.py <BOARD> --intent <INTENT> <SPEC FLAGS> --baseline <INPUT BOARD> --json <RUN DIR>/verify_<N>_score.json`: blocking 0, unrouted 0, broken 0;
   - `python3 -X utf8 check_complete.py <BOARD> --intent <INTENT> <SPEC FLAGS>`: DONE;
   - `python3 -X utf8 check_complete.py <BOARD> --intent <INTENT> --authored-from <INPUT BOARD>`: **report it, not gating**, and name any floor it lists;
   - `python3 -X utf8 py_router/check_connected.py <BOARD>`: no disconnected pads;
   - `python3 -X utf8 py_router/check_drc.py <BOARD> --baseline <INPUT BOARD> --clearance-margin 0.1`: no real violations.
5. **`route` mode, also:** every footprint's pose (x, y, rotation, side) equals
   the input's.
6. **Declared classes.** Name every `ungraded` component as UNEXAMINED. If the
   board declares a class (diff pairs, impedance, length groups, power widths,
   high-speed nets) and its component is ungraded, the verdict is FAIL. For
   declared diff pairs, length groups or high-speed nets, also run the
   `review-routed-board` skill's Steps 2–3b and report them.

## Output

The first line of `verdict_<N>.txt` is exactly `VERDICT=PASS` or
`VERDICT=FAIL`. Then give one line per check with its measured number, and for
FAIL, the specific defects (net names, refs, coordinates). Return the same text
as your final message. Do not modify any board.

**`full` and `route` modes, also write one file per routed-board lens**, so
the run can close its ledger with `converge.py record --final --lens-file …`:
- `<RUN DIR>/verdict_<N>_connectivity.txt`
- `<RUN DIR>/verdict_<N>_drc.txt`
- `<RUN DIR>/verdict_<N>_spec.txt`

Each file holds one line:
`VERDICT=PASS:lens=<lens>` or `VERDICT=FAIL:lens=<lens>;finding=<what>;evidence=<number>`.
- **connectivity:** check_connected and board_score's unrouted/broken.
- **drc:** check_drc.
- **spec:** check_complete, plus the mechanical facts.
