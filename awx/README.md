# awx -- the K28 bus chain (#622)

The minimal tool set that reproduces the current best practice for
routing a fanned-out bus between two BGAs (the `fb_t2q_fresh` bench:
an FPGA `U1` and a DDR3 `DU1`, 28 nets of the coherent K-ladder):

    bash chain_k.sh TAG 28          # -> tmp/TAG_k28.kicad_pcb, graded

Reference result on the bench (2026-09-06): 28/28 connected, 0 DRC at
the routed 0.1 mm floor, 38 vias, 1440 segments, ~55 s end to end.
The chain is deterministic: `cmp_copper.py A.kicad_pcb B.kicad_pcb`
reports IDENTICAL copper between two runs (UUIDs differ, copper does
not), which is the regression test for any change here.

## The chain

1. `coherent_nets.py K` -- the first K routable nets of the coherent
   ladder (`k_ladder_coherent.txt`: whole rivers, tightest first; a
   prefix never splits a river).
2. `fanout_from_plan.py OUT.kicad_pcb K --board=BASE` -- the PLAN
   and the destination fanout. `plan_ends.py` picks one escape per
   ball at the destination from menus of legal moves
   (`escape_moves.py`), judged by the crossing floor of the lane order
   the braid will see (`select_moves.py`, `taut_clean.py`,
   `detect_buses.py` for the taut-path buses), with the source
   refinement run for its effect on the launch points. The chosen
   directions go to the production engine (`py_router/bga_fanout`,
   `escape_dir_hints`) which lays the copper; the copper that actually
   leaves each ball is measured against the plan (`obeyed`). A fanout
   that is not DRC-clean and complete stops the chain.
3. `braid.py --board FO.kicad_pcb --dest DU1 --nets ... --out STEM` --
   corridors from the geometry (`corridor.py`), a spine per corridor,
   order and layers from the two-page schedule (`schedule.py`), every
   lane routed by the real router inside its band (`connect.py`,
   `topo_strings.py`). Refused lanes get a last call, a rip assist, and
   the economy re-lay; what is still refused is reported and left open.
4. `grade_k.py BOARD NETS` -- connectivity scoped to the run's nets,
   whole-board DRC at the routed floor, the via census
   (`via_census.py`).

No environment variables, no options beyond BASE / DEST (the inputs).
Everything the branch had tried and not adopted -- rivers, packing,
negotiation, tail rescue, page sidecars, line and slot hints, the
order model, the surgical and split B passes, plan dumps, debug
switches -- is gone from this tree; it lives in the `bus622-take4`
history.

## What this adds to `py_router` (and nothing else)

`generate_bga_fanout(..., escape_dir_hints=None)`: a per-pad planned
escape side, keyed by board-frame pad position. Re-keyed into the
footprint frame for a rotated part, taken first by the channel engine
(`preferred_dir`) and by the under-pad engine (a side-constrained A*
tried before the unconstrained one; the dog-bone gap site on the
planned side). With no hints every path is unchanged.
