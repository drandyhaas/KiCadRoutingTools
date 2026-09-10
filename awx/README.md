# awx -- the K-bus chain (#622): one plan, a fanout that follows it, a braid

(We have no idea what `awx` stands for. The name predates every note
that mentions it.)

The tool set for routing a fanned-out bus between two BGAs, where the
PLAN decides both ends of every net, the FANOUT lays exactly the plan's
moves (and tells the plan what it could not), the BRAID routes the lanes
in a corridor of pages, and a refused lane is negotiated rather than
left open:

    bash chain_k.sh TAG 15 28 41           # -> tmp/TAG_k<K>.kicad_pcb, graded
    python3 make_bench.py BOARD SRC DST OUT # another array pair, any board
    bash pose_gate.sh BOARD SRC DST 15 28   # the same pair in every pose

## Where it stands (2026-09-08)

The bench (`fb_t2q_fresh`: an H3 BGA `U1` to a DDR3 `DU1`, the coherent
K-ladder), one fanout per K, byte-deterministic, 0 DRC at the routed 0.1
mm floor everywhere, and since 2026-09-08 the SAME result for the board
moved anywhere on the sheet (the translation section below):

| K  | open | vias | in-band | chain (memo warm) | human vias | old strings, same engine | 09-07 |
|----|------|------|---------|-------------------|------------|--------------------------|-------|
| 15 | 0 | 16  | 15 / 15 | 10 s  | 22 | 14, 15 s | 14, 15 s |
| 28 | 0 | 36  | 28 / 28 | 36 s  | 46 | 36, 37 s | 38, 28 s |
| 35 | 0 | 61  | 35 / 35 | 89 s  | 58 | 54, 81 s | 54, 76 s |
| 41 | 0 | 112 | 32 / 41 | 174 s | 70 | 86, 200 s | 82, 155 s |
| 51 | **1 (SBA1)** | 129 | 33 / 48 | 355 s | 85 | 1 open (SA4), 130, 429 s | 0 open, 141, 339 s |

Measured 2026-09-08 evening with the batched relaxation as the default
(tags `tf4`, `td2`; cold: 13 / 37 / 261 s and K35 142 s, K51 562 s);
K41's row re-measured late that evening (tag `ps`) after the braid took
the board's edge clearance (the fence at the bottom edge moved its
draw, 106 -> 112) and the soft stamp lost its row sort (-48 s on the
same draw; the section on the K41 profile below).
K41 and K51 completed for the first time on 09-07 (the blocker-directed
rip at the last call); since then two general rules re-decided the
draws: the pose work's exact-tie stamps in the fanout and the main
router's pad keep-outs ("old strings, same engine": K28 -2, K41 +4,
K51's SA4 open), and then the convergent relaxation's strings (this
column: K15 +2, K35 +7, K41 +20, K51 SBA1 open instead of SA4, 74 s
faster). Both draws are the chain's knife edge, not the rules' -- the
same rules grade the moved board identically -- and the vias are TODO
2's business. "In-band" is the lanes the braid routed inside their
planned bands; the rest were re-laid at the last call.

![K41 on the bench: one corridor of 41 lanes, both pages, the rides round the destination](img/k41_corridor.png)

*K41: the corridor from `U1` (left) to `DU1` (right) -- front lanes red,
back lanes blue, every lane in its band, the far-face exits riding round
the destination's east face.*

A second array pair (the corpus's `zynq_ad9364` made two-layer, `U1` ->
`U2`, 44 nets; `make_bench.py`): K11 0 open 24 vias, K20 0 open 32,
K28 0 open 55, 0 DRC -- complete, but 17 of 28 in-band where the bench
has 28 of 28. The pose gate (`pose_gate.sh`: both arrays on the back,
either one, the board rotated) completes every pose; the rotations are
exact isometries of the plan and the braid's rules, the residual being
the router's octilinear lattice; and the board TURNED OVER
(`mirror_board.py`) exposed the chain's own front call-outs: the layer
the taut paths relax against (fixed to the layer the teeth are born
on), the destination selector, which is handed and now runs every
pair in the pair's own frame (`PairFrame`: the mirror gets the mirror
of the plan, the bench is untouched by construction), and the braid's
planner and the braid, which take the same frame at their own boundary
(`braid.setup`: a -1 pair's board turned over in memory, the plan
mirrored in, the copper mirrored back). The mirror now grades 16 and 38
against the front's 16 and 38, the same plan to the letter (the pose
gate section below).

## The chain

1. `coherent_nets.py K` -- the first K routable nets of the coherent
   ladder (`k_ladder_coherent.txt`: whole rivers, tightest first; a
   prefix never splits a river).
2. `fanout_from_plan.py OUT.kicad_pcb K --board=BASE` -- ONE consistent
   loop over the plan and both fanouts:
   * `plan_state` reads everything off the board AS IT IS: the menus of
     legal escape moves at both ends (`escape_moves.py`; the source menu
     prices its moves against the other nets' real stubs), the launch
     points (the source teeth on the board), each tooth's layer and
     vias, the taut-path buses.
   * The destination is chosen against those teeth (`select_moves.py`),
     the source refined on paper against that destination
     (`plan_ends.refine_source`), and the refinement is REALIZED:
     `source_realize.py` strips those nets' source copper and re-fans
     them with the production engine in the plan's full moves, restores
     any ball the engine refuses, DRC-gates the board, and audits every
     tooth -- original vs asked vs achieved, per dimension (face, exit
     gap along the face, layer, kind) and as an ORDER along each face.
     The next round chooses the destination against the teeth that
     copper produced; the best round's board and choice are kept.
   * FEEDBACK: a move the engine did not lay exactly leaves that net's
     menu (`banned`, keyed by `source_realize.move_sig`) and the round
     re-plans; the destination is its own select -> fan out -> audit ->
     ban -> re-select loop that ends at "every berth laid as planned"
     or, when it never converges (K41), ships its LAST pass with that
     pass's own sidecar. The engine is the authority on what is possible.
   * What the plan is JUDGED on (`plan_ends.judged_cost`): the vias its
     own model implies -- per net a dive if the corridor cannot keep it
     on its tooth layer, a via where the delivered layer is not the berth
     escape's, the berth escape's vias (`select_moves.true_vias`) -- plus
     the SOURCE escape's vias, plus the ride round BOTH arrays at
     `VIA_MM` per via (`around_box`, hit-tested against a box shrunk by a
     hair so a leg along a face is not a hit). Keepers are judged within
     the corridors the braid will form: `planned_buses` calls the braid's
     own `corridor.cluster_corridors` on taut paths from each tooth to
     its planned exit. `explain_plan` prints the model per net (launch
     and exit order, keepers, predicted vias, crossing pairs) so it can
     be held against `via_census.py`.
   * The plan's lane model: two moves cannot share a gap on one layer
     over overlapping stretches, a dog-bone via is a THROUGH obstacle in
     any other lane, two teeth cannot share an exit point whatever their
     layers, and a dog-bone site must be an inter-ball gap (not the
     boundary line).
3. `braid.py --board FO.kicad_pcb --dest DU1 --nets ... --out STEM` --
   corridors from the geometry (`corridor.py`: nets whose stubs one
   spine can reach), a straight spine per corridor, launch and target
   orders from the lanes' offsets, the two-page schedule
   (`schedule.py`: pages BY TOOTH LAYER -- page F seeded by the largest
   crossing-free set among front-born nets, page B among back-born, the
   rest filling whichever page they do not cross, own layer first; what
   fits neither swims), and every lane routed by the real router inside
   its band (`connect.py`, `topo_strings.py`), the lanes not yet routed
   stamped as virtual copper. Up to six attempts widen the launch pitch
   and route refused lanes earlier, the best attempt is kept; refused
   lanes then get a wider last call, and a lane still refused there a
   BLOCKER-DIRECTED RIP (the router's blocked frontier attributed to
   this run's lanes, a min-cut probe naming the cut set, victims re-laid
   or negotiated one level down); lanes with three or more vias get an
   economy re-lay kept only when strictly cheaper. What is still refused
   is reported and left open. The output is smoothed (the repo's
   octolinear pass) and written with an Eco overlay of the planned lanes.
5. `make_bench.py`, `rotate_board.py`, `mirror_board.py`, `pose_gate.sh`
   -- an article from any board (a two-layer version, the source fanned
   out the chain's way, the floor stamped, the ladder beside it), its
   rotations and its mirror, and the gate that runs the chain in every
   pose (below).
4. `grade_k.py BOARD NETS` -- connectivity scoped to the run's nets,
   whole-board DRC at the routed floor, the via census
   (`via_census.py`).

No environment variables, no options beyond BASE / DEST / LADDER (the
inputs: the board, the destination reference, the ladder beside it).

## The ingredients, in pictures

![K28 on the bench](img/k28_corridor.png)

*K28: two pages. A front lane and a back lane cross for free; a page
lane keeps its layer through the schedule region and pays a via only
where its tooth or berth is on the other layer. 38 vias for 28 nets.*

![The east face at K41](img/k41_east_face.png)

*Far-face exits: a berth on the destination's far face is a side exit
of the main corridor whose leg lies beyond the array and whose jog runs
back along the stub's own line -- not a corridor of its own through the
ball field.*

![SBA2 after the rip at K41](img/k41_rip_sba2.png)

*The blocker-directed rip: SBA2 (the highlighted lane) was refused at the
last call, boxed by lanes routed before it. Its blocked frontier named
the lanes on it, the min-cut probe found the cheapest crossing set,
SCKE1 was ripped, SBA2 routed, SCKE1 refused and negotiated in turn by
ripping SA8 -- every lane routed, K41 complete.*

![K51 on the bench](img/k51_corridor.png)

*K51: 48 routable nets, complete at 141 vias (human 85). Twenty-four rips
landed a lane; the vias are the work now.*

![The human's K51 on the original board](img/k51_human.png)

*The same 48 nets as the human routed them (`allwinner_h3_ddr3`, the
original board): 85 vias, most nets on one layer between their two
escape vias, the address rides nested round the destination, the data
lanes meandered to length. The benchmark to approach, not a pose to
match.*

![The second array pair at K28](img/zynq_k28.png)

*The zynq article (`make_bench.py --two-layer`): the corridor runs north
from the Zynq to the DDR3, berths on three faces. Complete at 55 vias;
17 of 28 in-band, the gap a second board shows.*

![The bench turned over](img/gate_mirror_article.png)

*The mirror article: the fanned bench flipped through its plane -- every
part on the other face, every stub on the other layer, y mirrored,
self-verified -- so the chain's own front call-outs show without the
fanout engine's.*

![The bench rotated 30 degrees, K15](img/gate_r30_k15.png)

*A 30-degree rotation: complete, but 33 vias and 6 of 15 in-band -- the
plan's faces are compass directions and the router's lattice is
octilinear, so a non-orthogonal pose is outside both models today.*

## Benches and gates

    python3 make_bench.py BOARD SRC DST OUT.kicad_pcb [--two-layer]
                          [--src-side F|B] [--dst-side F|B] [--rotate DEG]
    python3 rotate_board.py IN OUT DEG      # the whole board, self-verified
    python3 mirror_board.py IN OUT          # the board turned over, self-verified
    python3 bend_bench.py OUT.kicad_pcb CX CY [--rot DEG] [--bottom Y]
    python3 channel_bench.py OUT.kicad_pcb DCX DCY HX HY [--right X] [--bottom Y]
                             [--rows N] [--pad MM] [--drill MM]
    [POSES="R90 R30"] [GATE=name] [LADDER=file] bash pose_gate.sh BOARD SRC DST K...

`make_bench.py` prepares an article the way the bench was prepared:
inner layers out (`--two-layer`), the pair's two-pad nets, SRC fanned
out with the chain's own destination engine call, the project stamped
with the chain's floor, DRC-gated, the ladder beside it from the plan's
river detection; `--src-side` / `--dst-side` mirror an array to the
other face (the caps under it following), `--rotate` turns the finished
article. `pose_gate.sh` runs the chain over FF / BF / FB / BB / R90 /
R180 / R270 (or any `R<deg>`) with one ladder beside every pose and
prints open / DRC / vias / in-band / seconds per pose and K.

## TODO for future sessions

THE one list (2026-09-08 evening; the session memory points here). In
the order worth taking them, each with what is known.

1. **Vias, and the K51 open.** The ladder with every rule of 2026-09-08
   is 16 / 36 / 61 / 112 vias and K51 one open (SBA1 on this draw),
   against the human's 22 / 46 / 58 / 70 / 85. **2026-09-09 (fourth
   session): the SIDES the plan hands the braid are the lever -- the
   human's DU1 sides pinned (`PLAN_FORCE_DST`), nothing else changed,
   K41 112 -> 91 (0 open, 0 DRC); the section "The human's K41, and the
   sides the plan hands the braid" has the census, the wave model and
   the three steps that follow (side chooser, source basin move, single
   dive for the DQ class).** **K51 diagnosed 2026-09-09 (fifth session; the
   section "K51: why the passes never converge, and what the human does
   there"): the human's win is a JOINT order at both ends (its source
   order with our berths is worse than our own), eight of its eleven north
   riders climb 5-10 mm under U1 on B to exit at a chosen row -- the move
   we lack -- and the chooser's flat swimmer price bought seats with
   whole-bundle diagonals (SA10 open). Build the source exit-row move
   first, then price swimmers by their crossings.** **Sixth session
   (2026-09-10, the section "The re-berth loop, the boxed end, and the
   climb"): the re-berth loop is built (`reberth.py`, `REBERTH=1`) and
   finds the boxed end is the TOOTH at K51 (pocket 2.6 mm against 24);
   its pad-to-pad backstop closes one of two opens per K51 board at a
   price; the climb move exists in the menu (`SRC_CLIMB=k`) and the plan
   picks climbs, but one at a time, and the braid grades it worse (4
   open); the chooser on the first round only gives K41 88 vias in 2:20.
   What is left is the JOINT source move: a group generator for climbs,
   the co-move (source menu against a bare array), and pruning for time.**
   **Seventh session (2026-09-10, the section "The braid's verdict
   re-plans the ends"): the route itself is the judge -- `replan.py`
   reads the braid's verdict, re-plans the nets it paid for at either end
   (climbs with half-pitch exits, co-moved berths), probes each move with
   the real router on the routed board, audits what the fanout laid, and
   keeps the probes' board: K51 0 open, 0 DRC, 107 vias, the best complete
   K51 so far. Left: a judge for the hard swimmers (SA11/SA4 refuse in
   every local group) and the round time.** The two moves below are still known:
   - **A re-berth AND a re-fan move for trapped stubs.** The rip stops
     at "walled by static copper -- a fanout matter". Take4's negotiator
     answered that by re-berthing (`negotiate_stubs`, `relay_net.py
     --ref`): rip the berth and fan the ball out again in another move.
     Do the same for the TEETH -- a re-fan of the source escape for a
     trapped stub, at either end, judged by the chain. Concrete cases:
     the FB pose (destination array on the back) at K28 leaves SA0 open
     after a depth-2 rip, walled by static copper at its berth; K51's
     SA4 or SBA1, whichever the draw leaves.
   - **Collapse the short dives** (take4's `collapse_dives.py`, 388
     lines, never ported): on a routed board, a short dive is two
     same-net vias joined by a brief single-layer bridge -- the A*'s
     zigzag escapes, a lane that surfaces for 0.85 mm and dives again.
     Each pair is tried serially, accept-and-build: rip the two vias and
     the bridge, ask the real router band-free for a path between the
     cut ends, keep it only if it adds no via (two saved per accept),
     verify by re-walking the net's endpoint degrees, grade as ever. The
     cheapest via reducer on the table; `ledger_cal.py` (floor against
     slack per lane) says where the slack is.
2. **The corpus A/B for the production changes, then the PR to main.**
   Three engine changes of 2026-09-07/08 are in shared code and owe the
   A/B before main: the pad keep-out's sub-cell offset quantised
   (`routing_utils.pad_blocked_cells_array`, every routing step), the
   back-side BGA fanned as the front's mirror (`bga_fanout/flip_frame`),
   and the rotate frame's fixes (exact quarter turns, pad sizes swapped,
   foreign angles). Plus a B-side case in the GUI fanout parity gate
   (today only a rotated F-side QFN). Sets 1-5 on Modal, both arms at
   one commit, per the RUNBOOK.
3. **Time: K41 at 174 s warm against the two-minute edict**, profiled
   stage by stage (the section "Where the K41 time goes"). In order of
   value per effort:
   - the fanout's A* is 50 of its 82 s and 40 of those are searches
     that FAIL by flooding their window (284 of 820; a failure is ten
     times a success). The reach record that skips hopeless gaps exists
     inside the level-1 walk only: hand level 0's reach to levels 1 and
     2 in `underpad._follow_plan.attempt` -- about 15-20 s, contained;
   - one base obstacle map per WINDOW in `connect` instead of per
     attempt, a `clone_fresh` per attempt (byte-identical: that is how
     the router's own per-net maps are made); the key must cover the
     window's copper (lanes land and rips re-lay between attempts) and
     the virtual copper appended before the build; only same-window
     attempts share (the ladder's first two rungs, margin 2.0) -- about
     5-10 s, moderate;
   - the octilinear smoother's clearance sweep (`_seg_foreign_seg_dist`,
     41,000 calls sampling every 0.02 mm against the windowed foreign
     segments; ~25 s real; production `pcb_modification`): an exact
     segment-to-segment distance or a spatial hash;
   - the last-call rip's econ re-lay: cap its widening for a lane the
     rip just placed (K41's braid spends ~30 s there; K51 355 s).
   The A* core itself moves only with a Rust port. Cold adds the taut
   strings (~35 s real at K41), which stay on the sharded memo.
4. **Poses off the axes.** The flow frame makes the four quarter turns
   one run; a non-orthogonal pose (R30: K15 33 vias 6/15 in band, K28
   15 DRC; R45 fails the plan stage at K28) breaks the plan's compass
   faces and needs a trigonometric turn of the file, which is not a
   lattice symmetry -- the engine's own `rotate_frame` does it for the
   fanout; the chain-level version is this item. Also a pair whose two
   arrays sit at different angles (only one can be axis-aligned).
5. **Better spines -- BUILT, measured, and the DEFAULT since 0b826fbf
   (2026-09-08 late; the section "Better spines: the medial line,
   relaxed, in grid legs"; `corridor.build_spine(relax=True)`, no
   switch).** The relaxed medial line is back on the batched relaxer,
   in octilinear legs; the bench is byte-identical (a clear chord is
   never relaxed) and on an article whose channel holds a part no
   track can pass, K28 goes from 3 open / 12 DRC to 0 / 0 (reproduced
   2026-09-08 night: chanD K15 0 open 26 vias 692 segs, K28 0 open 0
   DRC 54 vias 1777 segs). What is NOT finished: the in-band count on
   that article trails the chord's (K15 9 of 15 against 10, K28 13 of
   28; the last call routes the rest), and the ribbon model is one
   scalar inflation where the ribbon is asymmetric (per-side, per-s
   extents, the section's last paragraph). Neither has an arm yet.
6. **The second bench's in-band gap.** zynq_ad9364 (`tmp/bench2`, not in
   git; `make_bench.py` rebuilds it) K28: 0 open, 55 vias, but 17 of 28
   in band -- 11 lanes at the last call (A4 A6 DQ9 DQ8 DQ14 RAS CKE DQ3
   DQ1 A5 DQ10). The leg rules were tuned on one bench. `wall_probe.py`
   takes `DEST=U2` now; probe those eleven; run K34 / 40 / 44.
   **2026-09-09: the article no longer runs through `chain_k.sh` -- the
   flow frame's quarter turn (FK=1 about (109.3, -103.2)) fails its own
   verification (`TURN FAILED: pad C139.1 extents; pad C139.2 extents;
   segment`), so every stage is skipped. Fix `flow_frame.py turn` for a
   board with rotated / off-grid parts before the zynq numbers can be
   re-measured (the berth chooser was not measured on it for this).**
7. **Fanout non-convergence.** K41's destination passes never converge
   (8 passes, the last ships) and SA9 is never planned -- its menu is
   banned away. A net whose menu is exhausted gets its achieved berth
   back as a menu entry ("freeze what worked"); measure K41 / K51.
8. **The packing -- BUILT 2026-09-09 (`pack.py`, opt-in `BRAID_PACK=1`;
   `pack_board.py` packs a braided board again in seconds; the section
   "The pack: every lane a taut string against its neighbour").** Every
   lane, vias included, relaxed as one taut string against the board as
   it stands and re-emitted octilinear where a build clears; from the
   roomy side of the corridor inward. 0 open 0 DRC at K28/35/41 with the
   vias unchanged; K28 1574 -> 875 segments and the rivers read as a hand
   would draw them, K41 2258 -> 1843; K35 1435 -> 1475 (its base was
   already clean and spread). Pack 4 / 6 / 10 s. Not a default. The
   follow-ups, in the order worth taking them:
   - **Its purpose is unmeasured.** The rationale was that packed rivers
     leave room for the swimmers and their vias; every grade so far is
     at write time on the pk0 boards, where the pack is grade-neutral by
     construction. Run `BRAID_PACK=1` through `chain_k.sh` on the ladder
     (K15..K51) and the zynq bench, and try a pack BEFORE the last call
     so the refused lanes search a river's free width in one piece (a
     mode removed untested; the runner is the emitter for it now).
   - **Default or not.** K35's base is already clean and its packed
     board carries more segments (1475 vs 1435) at the lowest off-grid
     length; whether to pack a board whose lanes are not staircases is a
     verdict for the renders (`img/pack_k*_{before,after}.png`).
   - **The fan from the source's south row** (6-degree lanes between a
     pad row and a river): a taut string is a straight line at whatever
     angle its ends dictate, and there is no elbow room in any order.
     Needs an octilinear-metric tension -- a staircase costing what its
     chord costs -- and the cheap first step is the straightening's taut
     path (`_straighten`) with the octilinear length as its edge cost;
     the emitter already elbows a chord where clear.
   - **Coupled corners** (SRST and SA9 turning into adjacent gaps of the
     passives, each hugging the other's any-angle corner): no order
     breaks the cycle. Related: the full second pass
     (`BRAID_PACK_PASS2=1`) packs tighter (median 0.47 -> 0.32 mm) and
     emits worse because tight packing round vias makes arcs; if the
     chamfer emission improves at exact pitch, re-measure it.
   - **Untested paths.** The sidecar and `pack_board.py` handle a
     mirrored or turned board (`ctx.M`) in code; no pose-gate run has
     exercised them. A lane the pack rejects and leaves as router copper
     has not been checked through the runner's stub trim.
   - **Residue.** A 1-degree approach piece at SA4's landing; `HUG_FLAT`
     0.05 and `PULL_MIN` 3 tuned on three boards; the off-grid proxy
     (`tmp/offgrid.py`) moves 5-10 mm when one long chord flips, so it
     is read with a render. The via count is TODO 1's business (SA12's
     rip re-lay with three vias is the dive collapse's case); the pack
     adds 10 s to K41's braid against TODO 3.
9. **A better routing order.** Lanes are laid sequentially -- pages in
   target order, then swimmers largest displacement first, refused
   lanes boosted next attempt -- and every refusal the rip repairs is a
   sequential loss. Take4's order model (`plan_order.BraidOrder`, the
   braid's own rules as the plan's cost) and its rip assist are the
   references; candidates are most-constrained-first, the min-cut
   probe's crossing counts as the order, and the negotiator's history.
10. **Memory -- DONE 2026-09-08 night (the section "Memory: the chain
    under a gigabyte").** Every stage under 1 GB with the copper
    identical: chanD K28 fanout 1161 -> 303 MB, braid 1746 -> 771; the
    bench K41 fanout 442, braid 511. Two of the evening's four
    suspects were real (the taut memo parsed from JSON at 3.6x its disk
    size; `_OBS_MEMO` never evicting the models of boards the plan loop
    had left behind) and the two largest were not on the list (the band
    cells' window-sized intermediates per attempt; the min-cut probe's
    disc-per-point soft stamp). The band then moved into the map's
    static bitmap (Python-only; the Rust allocator's peak 329 -> 90 MB).
    Then the production smoother's sweep in row chunks of 512 KB
    (its freed matrices had been kept by macOS malloc as 230 MB of empty
    large regions): K28 braid high-water 797 -> 372 MB, route.py
    copper-identical. Left: `blocking_analysis._NET_CELLS_MEMO` (~85 MB)
    and the fanout stage's resident memo shards (K41 ~140 MB).
11. **The exact taut solver**, if that line is picked up again
    (`tmp/uncommitted_0906_archive/taut_exact.py`, its section above):
    the union walk done in the batched array, and the homotopy class
    chosen the way the flow chooses it rather than by the nearer side.

Closed on 2026-09-08 and recorded in their sections above, not here:
the braid's own handedness (the mirror grades as the front), the
chain's translation invariance, the flow frame for the quarter turns,
the pose gate's own verdict, the taut memo sharded and the batched
relaxation as the default, the braid's edge clearance from the board,
the soft stamp's sort.

## Still on take4, worth a port

Beyond the TODOs above, these on the `bus622-take4` branch still earn a
port, grouped by what they would answer today. (The branch's full
inventory -- 136 files by purpose, the mechanisms cut from the modules
kept here -- lived in this README's history section until 2026-09-07;
`git show bus622-take4:awx/README.md` has it.)

**Where the extra vias are** (K41 82 against 70, K51 141 against 85).

- `ledger_cal.py`: per net, the DP floor -- the vias its real crossings
  force -- against its slack, the realization waste. That split is the
  first question to ask of the 141: slack is cheap to recover, floor
  means the plan's crossing set.
- `harvest_k.py` and `surgical.py`: re-braid each slack net alone, in
  place, against everyone else's frozen copper, keep strictly better.
  The primitives are exactly the "one net swapped or stripped and
  braided alone" that the re-berth TODO also needs.
- `census_vs_human.py` and `human_at_k.py`: per net, our vias, copper
  and via positions (source end, destination end, mid-field) against
  the human's on the same K set. The human's vias are at the ends; the
  census names which of ours are not.

**Why the plan makes so many swimmers** (22 of 47 at K51).

- `cut_ledger.py`: the Maley cut-capacity check on the plan before any
  lane is routed, so "not enough room" is measured, not inferred from
  refusals.
- `group_pages.py` and `plan_nest.py`: joint ride assignment over a
  crossing group, and homotopy nesting. Their measured lesson at K35
  was that the gap to the human was the crossing set, not the pages.
- `channel_shift.py`: the human's one-track-per-channel pattern at an
  array face, which set the K35 record at 66. It is a concrete re-fan
  move for the teeth TODO.

**Output hygiene.**

- `nudge_grazes.py`: the write-time micro-nudge for the ~35 um
  quantization grazes, the class the 30-degree pose showed 15 of.
- `prune_debris.py`: dead tails and twin arms left by mid-stub joints,
  which count as copper against the human.

**Poses off the axes.**

- `flow_frame.py`: rotate the pair into its own frame before planning,
  the way `rotate_frame.py` already does for a rotated BGA, then rotate
  back. That is the answer to the compass-direction faces the 30 and 45
  degree rows hit.

**Workflow.**

- `drive_k.py` composes "complete first, then cheapen"; `retry_chain.py`
  is chain-level directed iteration with the refused nets forced
  through the back-side arms. Completion is the rip's job now, but the
  composition is the shape the via work will take.
- `band_dump.py`: the per-lane world picture, for the second bench's 17
  of 28 in-band.

`improve_k.py` (the diagnose-and-move loop) is the heaviest and leaned
on the pages sidecar that take5 cut, so it is last.

## What this adds to `py_router` (and nothing else)

`generate_bga_fanout(..., escape_dir_hints=...)`: a per-pad planned
escape keyed by board-frame pad position -- a bare FACE (`'down'`), or a
FULL MOVE (`{'face', 'exit', 'layer', 'kind', 'site'}`: the exit point on
the boundary line, the layer the run leaves on, `surface` /
`via_in_pad` / `dogbone`, the dog-bone via point). Re-keyed and
transformed into the footprint frame for a rotated part
(`rotate_frame.forward_transform`), threaded through the escape-priority
passes and the auto-retry ladder; the channel engine reads the face of
either. The UNDER-PAD engine follows a full move in its plan-follow
phase (`underpad._follow_plan`): planned balls leave the generic phases,
their via sites are reserved first (an asked dog-bone site validated
exactly as the engine's own), every ball is routed to its EXACT move
deepest-first (the A* takes a goal cell: the boundary cell at the asked
gap, the only way out), a ball whose exact move is blocked negotiates --
its blockers among the same call's escapes are found by routing it on a
pre-commit occupancy snapshot, ripped, the ball laid, the blockers
re-laid, the state kept only if the count of balls landed as asked rises
-- and what is still short degrades along the least damaging dimension:
the nearest free gaps first (+-6 pitches), then the other layer/kind,
then any face. Every ball's outcome is reported per dimension and
returned in `pcb_data._fanout_plan_report`. With face-only hints every
path is unchanged (copper identical to the previous chain); with no hints
nothing changes at all.

## One planner

The braid's own planning stage is the planner. `braid.setup(plan=)` takes
the ends, their layers and their escape directions from a plan instead
of reading them off copper, and `braid.plan_braid(board, names, dest,
plan)` runs the plan-only stage on them: corridors as the braid forms
them, spines, offsets, launch and target orders, the schedule's pages
and swimmers, side-exit legs. The fanout loop judges EVERY round and
every destination re-plan with it (`fanout_from_plan.judge_by_braid`:
per net the vias the pages imply -- `plan_ends.vias_from_pages`: tooth
vias, tooth/page mismatch, the arrival through a side-exit leg, the
berth's vias, a swimmer's dive and surface -- plus the ride round both
arrays); the fast proxy (`plan_ends.judged_cost`) serves only the source
refinement's inner loop. The shipped plan is written beside the fanout
board as `<board>.plan.json` with the ACHIEVED stub ends, layers and
faces, and the braid reads it (`setup` finds it; a sidecar that names
only some of the run's nets is the plan for those, the rest read off
the board) and builds its corridors from the identical inputs. At
every K the braid's orders and pages are the planner's.

Results (bench, same engine, previous chain -> this one): K4 4 -> 4 vias
(predicted 4, per net identical), K8 8 -> 6 (predicted 6), K15 22 -> 14
(predicted 12), K28 38 -> 42 (predicted 32); all complete and DRC-clean,
every berth laid as planned. The prediction missed every page lane the
braid had to route with an under-pass; see the via model below.

### The plan's via model

What a page lane costs is the number of LAYER CHANGES along its whole
profile (`braid.Corridor.layer_profile`): the tooth's layer, then every
stretch the schedule requires in s order -- its page over the schedule
region, and in the tail the OTHER layer wherever a same-layer exit leg
crosses it -- then its exit leg's layer, then the berth's. Adjacent
equal layers merge; the changes plus the tooth's and the berth's own
vias are the lane's prediction (`plan_ends.vias_from_pages(changes=)`).
The old count saw only the corridor's interior (`xa < s1`), so every
dive under an exit leg in the tail was free on paper: K28 predicted 32
for 42 laid, and each miss was a lane forced to the other layer after
s1. Two things follow from the profile:

- Exit legs choose their layer ALONG s (`lay_lanes`): a lane is crossed
  only by legs earlier than its own, so with the legs decided in
  ascending s every stretch the earlier legs imposed -- on the leg's
  own lane and on the lanes it crosses -- is known when it chooses. A
  crossed lane already on the other layer pays nothing more; a leg on
  the layer its lane is already on needs no corner via. Judged by pages
  alone the old rule sent K28's SA9 F -> B -> F -> B (three changes) for
  the one the router found. In-band the braid now routes 22 of 28 K28
  lanes at the first attempt where it routed 8.
- A later corridor's lane that crosses an earlier corridor's planned
  lanes on a layer they may use pays one dive, two vias
  (`braid.cross_corridor_vias`). Unpriced, the honest judge preferred a
  K15 plan that made SA9 a corridor of its own (predicted 0, realized
  2, the board 18 for 14).

Measured (HEAD f8b04714 -> this, same bench, warm taut memo, chain time
unchanged): K15 14 -> 14 vias with the prediction exact per net; K28 42
-> 40 (predicted 36); K35 56 -> 54 (predicted 59); K41 12 open -> 9
open, 72 -> 92 vias (three more nets routed; 18 of 37 lanes swim, the
two-page schedule is past its capacity there, and the plan is the same
in both arms). The residual at K28 is the braid's in-band execution:
the lanes it refuses are re-laid at last call, where some pick up a
dive the plan never asked for. Six execution changes were measured on
K15/K28 the same day (a layer-blind self-stamp in `cross_reserve`, a
symmetric launch pitch with a longer fan-in, re-running the best
attempt, the dodge tube kept out of the fan-in, wider virtual copper,
exit-corner via reservations set in along the leg) and none was a net
gain: each moved vias by two to four between lanes. They are not here.

### The walls, named cell by cell (2026-09-06 evening)

Every in-band refusal at K28 was traced with `tmp/wall_probe.py` (the
router intercepted at the lane's first call; the pocket flood-filled
from the tooth and its wall attributed by clearance zone to an owner --
a virtual lane by net, a reserve piece, real copper, a via, a hop, a
pad, the band; a channel profile along the planned polyline; the
farthest s reached in the whole window). Six lanes were refused at
HEAD and each had a name: a B-page lane's 1.5 mm reserve stamp on F
past its own B requirement (SDQ9 over SDQ10, SDQ11 over SDQ8), the
swimmer SA4's reserved diagonal over SDQ7, SDQ14's dive via beside
SDQM0's tooth, and the pads of C5 -- a front-side 0402 inside the
corridor, through which three lanes were planned straight while the
page rule closed the other layer exactly there. What was built from
that, each keyed on geometry read off the board:

- `cross_reserve` no longer stamps the corridor being routed (its own
  lanes are stamped by `virtual_of`, which follows the layer rules), and
  a reservation is clipped round every other net's free end.
- STATIC ISLANDS: every pad of a part that is not one of the arrays,
  projected on the spine and inflated by a track's clearance, merged
  when less than 0.1 mm apart, from s0 to the farthest stub
  (`static_islands`). `deflect_islands` bends the lanes on the island's
  layer round it -- the side by the smaller corner deflection, nearest
  lane at the edge, the rest outward at their own gap, other islands on
  that side stepped over -- into the (s, o) polylines the bands, the
  virtual copper and the windows read; in the tail the exit comb bends
  outward, a lane with no room to return before its leg stays bent to
  the leg and the leg starts there, and a leg over an island on its own
  layer is re-placed off it (`place_and_decide` run twice). The via
  model does not change: a lane bent on its own layer changes no layer.
- The birth and landing via at the slot (a via costs the same anywhere
  on a stretch, and the forward search leaves the tooth layer only when
  forced -- so a 0.45 mm stretch put the via at its far end, where the
  neighbours had converged).
- The slot pitch scaled by the secant of the lane's angle to the spine
  (`pair_floor`): clearance is perpendicular to a lane, a slot pitch is
  measured across the spine, and at 45 degrees 0.35 mm is 0.25 mm of
  room; a slot where a lane changes layer gets a via's room. The launch
  relax is symmetric and the fan-in grows with the largest shift, so no
  fan-in is steeper than 45 degrees (checked on paper first,
  `tmp/pitch_check.py`).
- A swimmer's reserved hop keeps a 2-D distance from every lane's
  polyline, not from its offset at one s.

- EARLY DIVE: a lane whose tail crosses an island on the layer it is on
  and which owes a change to the other layer anyway (its berth is there,
  or its page already is) takes that change BEFORE the island -- no via
  the plan did not already count, and no bend. Seeded into the leg
  placement so the leg's layer follows. At K35 the plan had looped SRST,
  SA0 and SA15 3 mm round a six-part passive cluster on F; they now go
  under it on B in-band. K41: 98 -> 84 vias at the same 8 open; K35 58
  -> 62, three other swimmers landing at 4.

The remaining walls are the swimmers (ten at K35, eighteen at K41, 2..4
vias each, and every K41 open is one): the two-page ribbon's capacity.

Speed: a taut path depends only on its two ends and the static copper it
relaxes against, and the loop asked for the same ones at every judgment
(210 relaxations for 15 nets), so `detect_buses.taut_paths` memoises on
the ends and `Obstacles.signature()`, persisted in `tmp/taut_memo.json`
across processes (the braid reuses the fanout stage's paths);
`braid.build_obstacles` memoises per board file. K15: 67 s -> 40 s cold,
19 s with a warm memo, copper identical. The obstacle model still counts
the run's nets' VIAS while excluding their segments (inconsistent, and it
changes the memo key on every realized board); excluding them changes
taut paths and needs an A/B.

### The best attempt is kept, and stale attempts end the loop (2026-09-06/07)

The attempt loop's feedback -- a wider launch pitch, refused lanes
boosted to the front -- is a heuristic for the refused lanes and a
change of world for every other lane, and the LAST attempt used to
ship. At K35 attempt 0 routed 27/32 in 33 vias with every swimmer at
2 (the router's own world flooded at each swimmer's first call says 2
is the minimum there); attempt 3, the one that shipped, routed 26/32
in 40 with five swimmers weaving for 4 each. Each attempt is a full
re-route from the base copper, so the one with the most lanes routed
(fewest vias on a tie) is restored -- copper, bookkeeping and plan
geometry -- before the last call. K35 62 -> 57 vias at 0 open, K41 84
-> 80 at the same 8 open, K15/K28 unchanged.

The loop used to stop only when the refused SET repeated at the maxed
pitch; a set alternating between two lanes (K35 with the far-face
exits: SA4 / SCKE1) never repeats and ran all six attempts for a best
that was attempt 0. An attempt at the maxed pitch that does not beat
the best attempt's routed count is stale; two in a row end the loop.
Copper identical (the best attempt is kept either way); K35 braid 85
-> 43 s.

### Far-face exits (2026-09-07)

A net whose berth sits on the destination array's FAR face -- past the
last ball along the spine, escaping away from the bundle -- used to be
split into a corridor of its own: the split rule asked whether its lane
could run from the stub back along the spine to the spine's end, and
that run goes through the ball field. The corridor it then got was
spined straight from its teeth to its berths, through the main bundle,
so every lane it planned was fiction: at K35 SA9/SA13/SA8 were refused
in-band on every attempt and re-laid at last call round the south and
east of everything (2 vias each, the human's homotopy and count), while
the judge priced them 4 each; at K41 that corridor held three of the
eight open nets.

The copper that ships is an ordinary side exit of the MAIN corridor
whose leg lies beyond the array: the bundle's outermost lane on that
side runs past the far face, a leg turns in along it, and the jog runs
back into the stub tip. So:

- `corridor.cluster_corridors` admits a stub past the spine's end when
  a short run FORWARD from it (away from the array, a pitch to a block's
  width) and a leg from there out across the array's side are pad-clear
  (`wrap_clear`); the old run-back test is tried first.
- `braid.Corridor.classify` marks a side exit whose stub lies beyond
  the last ball as `far_exit`, with `s_leg_min` = last ball + its radius
  + clearance + half a track: its exit leg is placed at or past that
  (`place_and_decide`, the floor enforced through `_leg_s`'s avoid), the
  target order already gives it the outermost slot (largest exit s), and
  the spine's forward extension and the static-island window reach
  `WRAP_REACH` past it, because `Spine.project` clamps s at the spine's
  end and an island at the array's corner (K35: C10 on F) is what the
  leg must clear.

Measured (bench fb_t2q_fresh, 0 DRC, warm taut memo): K15 14v and K28
38v unchanged, copper identical; K35 ONE corridor of 35, SA9/SA13/SA8
in-band at attempt 0, 57 -> 55 vias, plan 59 -> 52 predicted (SA13 and
SA7 0 vias); **K41 8 open -> 1 open (SA4), 80 -> 86 vias**, one corridor
of 41. Chain times K35 67 s, K41 151 s (fanout 68 + braid 82). The
first run after any plan change pays the taut memo cold (K41: 342
recomputations, ~5 minutes) -- a one-time cost, not the mechanism.
`chain_k.sh` now stamps the fanout and braid stage boundaries. (The
memo, `tmp/taut_memo.json`, is content-keyed and has never been wrong,
but it has grown to 158 MB and every process loads all of it, 1.6 s
each and two processes per K; a mirror or a new pose pays it cold --
K15 61 s against 21 s warm. Pruning it, or keying a file per board, is
a time item of its own.)

### The fanout engine's axes: translation first, then rotation (2026-09-08)

The pose gate had shown the engine's fanout depends on the angle the
array was dropped at: a FRESH fanout of the same source array on the
board rotated by 0 / 90 / 180 / 270 degrees gave 502 / 540 / 425 / 430
tracks and the chain 38 / 42 / 54 / 38 vias at K28. Routing every
rotated part in its own frame (`rotate_frame`, extended from the
non-orthogonal case to every angle, exact quarter turns about a lattice
point, the whole board carried: holes, zones, keep-outs, bounds, foreign
parts' angles by MINUS the turn, pad sizes swapped on odd quarter turns,
`frame_rotation` stamped for the braid's obstacle memo) made the half
turn exact and the quarter turns agree with each other -- and left them
different from the unrotated article. The reason was not rotation: the
engine was not TRANSLATION-invariant. The same board shifted 1 mm in x
fanned out to 447 tracks instead of 502.

**Translation invariance, found by bisecting the engine's own log and
its per-ball searches on the native and the shifted board
(`tmp/frame/src_*.py`, `trace_ball.py`, `gnd_trace*.py`).** Every cause
was a decision made by the last bit of a coordinate where exact
arithmetic has a tie, and every fix is the same: round the key so equal
stays equal, and let a deterministic order decide.

- `underpad.depth`, the ball routing order: balls on one ring are
  equally deep; the noise ordered them, and a different order routed one
  ball (SDQ4) into a corner, whose rip-swap rescue re-assigned five nets.
  Rounded to a nanometre; the stable sort keeps the footprint's own pad
  order, which moves and turns with the part.
- `escape.py`: the four edge distances of a ball (a corner ball ties two),
  the nearest channel to a pad that sits exactly midway between two, the
  nearest far end of a net, and the target-side comparison |dx| >= |dy| (a
  far pad on the array's own diagonal ties). `__init__._surface_gap_escape`'s
  four-exit order. All rounded.
- `reroute._seg_hits_pad`: a sample point exactly on a pad edge. With no
  margin a centreline on the edge is copper on copper (a hit); with a
  clearance margin exactly-at-clearance is clear, as KiCad grades it.
- `plane_fill_model.nearest_component_point`: the pour-direct tap of a
  corner plane ball is equally near two fill cells; the argmin is rounded.
- `underpad._Occ.cell`: the occupancy lattice is a NODE lattice (cell
  `(ix, iy)` is the point `x0 + ix*res`), and balls sit EXACTLY on nodes
  whenever the window margin is a whole number of cells (0.8 / 1.0 / 0.5
  mm pitches: 1.0 mm = 40 cells of pitch/32); the truncation let the last
  bit choose between the node and the one below. A point within 1e-9 of a
  node now belongs to THAT node (the upper cell). The LOWER cell was
  shipped first -- translation-invariant too, and the bench draw the user
  accepted (14 / 38 / 98) -- until `tests/test_bga_fanout_dogbone.py`
  showed what it does: every on-node ball moves one cell down, so a
  via-in-pad at the ball centre has its stub start 25 um away (ulx3s B12,
  the one orphan of 8 checks). The upper cell is the right one; what had
  made it look wrong was the next two items.
- `underpad._Occ._disk_spans` / `_capsule_spans`, the rasterisers behind
  every stamp: the centre in cell units (`(x - x0)/res`) carries the
  coordinate's noise, and a boundary cell at an exact integer radius
  (`i*i + j*j == (r/res)**2`: the 3-4-5 cells of a 0.125 mm disc, the
  edge of a capsule on a node row) was decided by it -- 244 and 288 cells
  of the initial stamp on the orangecrab article, and one net's jog took
  the other side of its corridor. Quantised to a billionth of a cell;
  everything downstream is arithmetic on those numbers and is then
  bit-identical in every frame.

`tests/test_fanout_translation.py` pins it wx-free on a real 96-ball
BGA (orangecrab U4): under three lattice translations the engine's
INITIAL OCCUPANCY STAMP is byte-identical (the check that exposed both
rasteriser ties) and the routing is the same (vias exact, every segment
endpoint within one occupancy cell of the other run's copper; with the
upper cell the moved runs have the same track count as in place), with a
non-vacuity check that the board has ring ties the shift re-orders in
raw arithmetic. The origin article (`U1` on `h3_FF_base`) is the same
447 tracks and 8 vias under eleven shifts. `translate_board.py` makes a
moved article for the pose gate (the third isometry, beside rotation and
the mirror).

**The bench's cost, and the decision.** The ordering fixes alone leave
the bench at 14 / 38 / 82. The cell rule cannot: every deterministic
convention re-decides the exact-edge stamps at the destination array,
and K41's draw moves with them (K15 14 and K28 38 hold under all). The
tuned 82 was the noise's draw, and that K41 sits on a knife edge is the
chain's (the rip and plan-adherence items). Decided by the user
2026-09-08: whichever rule is right ships and K41 is to be won back in
the chain. The lower cell went in first (14 / 38 / 98); the section
below is what it took to make the CHAIN invariant, and the bench with
the final rules is **14 / 36 / 86**.

**Rotation, measured with that in** (`tmp/frame/rot10.out`, a fresh
fanout on the rotated article, then the chain; K15 / K28 vias):

| pose | frame off (default) | frame on (`KICAD_FANOUT_FRAME_QUARTER=1`) |
|---|---|---|
| R0 | 14 / 42 | 12 / 40 (417 / 2107 segments) |
| R90 | 16 / 50 | 12 / 36 + SA4 open (705 / 1578) |
| R180 | 14 / 54 | 12 / 40, identical to R0 to the segment |
| R270 | 16 / 42 | 12 / 36 + SA4 open, identical to R90 |

With the frame on all four poses fan out to the same 447 tracks and 8
vias: the FANOUT is exactly rotation-covariant now. The half turn is an
exact symmetry of the whole chain; the quarter turns agree with each
other and differ from R0 by the PLAN's and the BRAID's own axes (the
braid's octilinear search leans on one axis, the plan's faces are
compass directions), which is the next frame to build (TODO: the pair's
flow frame, take4's idea). The quarter-turn frame stays opt-in until
then: making it the default re-fans the bench's destination array (at
90 degrees) and moves that draw again.

### Where the K41 time goes: the chain profiled warm and cold (2026-09-08 evening)

`tmp/prof_k41.sh`: the two stages under cProfile, memo warm and with
the memo set aside (the profiler inflates Python-heavy code, so the
numbers rank, they do not add up to the wall time; the wall time alone
was fanout 85 s + braid 160 s warm before the fix below).

The fanout stage (warm 119 s profiled): two thirds is the under-pad
engine's A* in pure Python -- 825 searches, 57 s of their own time,
through the plan-follow's attempt ladder (exact, face and layer, face,
any), where a search that fails explores its whole window. The rest:
the braid's own planner as the judge (18 calls, 12 s), the per-net
obstacle models (4,281 derivations, 7.6 s plus the cell packing), the
selector (5.8 s), the source realisation (5.5 s), the taut strings
warm (6.7 s). Cold adds the strings: 90 s profiled for 33 batched
calls (25,000 rounds), which is what the memo is for.

The braid stage (warm 172 s profiled): `connect` 118 s over 499 lanes,
of which the Rust search itself is 24 s and the rest is what the Python
side does around it -- `_stamp_soft` 43 s, of which 41 s was ONE
`np.unique(axis=0)` sorting the soft copper's cell rows before the
batch call (the map keeps the MAX cost per cell, so a duplicate row is
the same map: the sort is gone, -48 s on the same draw, copper
identical); a fresh base obstacle map per lane attempt (499 builds, 25
s); the band cells (10 s). Then the last-call rip (73 s, inside those
connects) and the octilinear smoother (46 s: 41,000 clearance checks,
each sampling the span every 0.02 mm against the windowed foreign
segments -- `_seg_foreign_seg_dist`, 36 s of its own time).

What is left, in order of what it would buy on the warm K41 (174 s at
the current draw, fanout 86 + braid 87): the fanout's Python A* (~60 s
real; fewer failed searches, or the search itself in Rust -- a heavy
change); the smoother's clearance sweep (~25 s real; an exact
segment-to-segment distance instead of the 0.02 mm sampling, or a
spatial hash -- production code, `pcb_modification`); one base map per
lane instead of per attempt (~15 s); and the last-call rip's re-lay
(TODO 8). Cold, the strings (~35 s real) stay on the memo.

### The flow frame: every pose of a pair is one run (2026-09-08 evening)

The routing lattice's symmetries are the eight poses of a square: four
quarter turns, each with or without a mirror. The mirror half was the
chirality frame (the selector's `PairFrame`, the braid's turn-over in
`setup`). The quarter-turn half is `flow_frame.py`, and it works at the
FILE level: `chain_k.sh` asks it for the quarter turn that points the
run's source-to-destination centroid vector along +x (`quarter`), turns
the base board by exactly that (`turn`: (dx, dy) -> (-dy, dx) about a
point on the 0.1 mm lattice, no trigonometry, so every routing grid is
its own image and a turned file turned back is the file to the bit;
footprints by their placement, the stored angle going DOWN by the turn
and the pads' absolute angles with it; siblings copied; self-verified),
runs every stage on that file unchanged, and turns the braid's board
back. A pair dropped at any of the four angles is then the identical
computation, because every stage sees one board: there is nothing to
hunt stage by stage, and the last-bit ties fixed for translation hold
in every frame. The bench itself is at k = 0 and runs on its own file,
untouched; a near-diagonal pair sits on the boundary between two
frames, either of which is a legitimate, translation-invariant run.

Why it is the whole argument: the four poses' frame boards are the SAME
board up to a translation (the same vector (+19.638, +0.635) from `U1`
to `DU1`, the same part angles, the same 447 tracks and 8 vias of source
copper), and the chain is exactly translation-invariant since the
section above. Measured on the origin article's four poses (the source
fanned with the engine's quarter-turn frame on, `tmp/frame/ffgate.out`):

| pose | turn | K15 | K28 |
|---|---|---|---|
| R0 | 0 | 14 vias, 403 segments | 41 vias, 1239 segments |
| R90 | 3 | 14, 403 | 41, 1239 |
| R180 | 2 | 14, 403 | 41, 1239 |
| R270 | 1 | 14, 403 | 41, 1239 |

The four K28 frame boards compared to the segment after the shift:
1465 segments each, 45 vias each, every net the same length, every
endpoint shared but two a micron apart (the six-decimal file rounding
at a boundary). Before this (`rot11.out`, the same articles): R90 and
R270 agreed with each other and differed from R0 -- K15 14 / 701
against 14 / 442, K28 36 with SA4 open against 40 -- by the plan's
compass faces and the braid's octilinear search leaning on one axis; a
half turn maps x to -x and y to -y, which no stage notices, and a
quarter turn swaps them.

What remains outside: a pair whose arrays sit at different angles
(only one can be axis-aligned in the frame), and non-orthogonal poses
(R30: the compass faces themselves), which need a trigonometric turn of
the file that is not an exact lattice symmetry -- the engine's own
`rotate_frame` does that for the fanout already; the chain-level version
is TODO 5's remaining half. The engine's `KICAD_FANOUT_FRAME_QUARTER`
stays as it is: inside the flow frame the destination array is at the
bench's own angle, so the chain does not need it, and the source
fanout that precedes the chain still does.

### The chain after the fanout: three more ties, and the pad offset in the main router (2026-09-08)

With the fanout engine exactly translation-invariant, the chain on the
bench moved by (10.3, -7.7) mm (`translate_board.py`, `tmp/gate/fbT`)
still graded K41 109 against 98 in place (K15 and K28 identical). Found
by diffing the two chains' logs stage by stage with the numbers
stripped, and fixed in order; each fix was measured by re-running both
chains and diffing again.

1. **The rip-swap rescue's victim order** (`bga_fanout.__init__.
   _underpad_rip_rescue`). In destination pass 2 the plan-follow stage
   was identical on both boards to the ball, and then the rescue of SA3
   evicted SA2 in place and SA15 moved: the two neighbours are EQUALLY far
   from the ball (0.425 mm, the symmetric pair on a regular pitch), the
   log printed the last bit as 0.42 and 0.43, and whichever came first
   opened a different corridor. Both rescues succeed. Rule now: the
   distance to a nanometre, and victims at the SAME distance are all
   tried and the RESULT decides (fewest vias, least copper, then name) --
   a general rule, not a coin. The corridor band's widest-gap choice got
   the same nanometre key. Chain identical in both frames after this,
   grading 109 in both: the tie-by-name draw. (`_surface_gap_escape`'s
   `KICAD_FANOUT_RESCUE_DEBUG=1` prints every corridor it tries.)
2. **The occupancy rasterisers** (the bullet above): found when the
   upper cell, adopted for the dogbone orphan, failed the translation test
   on the orangecrab article -- 244 boundary cells of the initial stamp
   differed, and the fix for the disc showed the capsule's 288. With the
   upper cell and both rasterisers quantised the chain grades 14 / 36 in
   both frames at K15 / K28, and at K41 86 in place against 84 moved:
   the FANOUT stage identical to the log line, the braid's first lane
   differing by ONE A* iteration (92960 against 92959).
3. **The main router's pad keep-out** (`routing_utils.
   pad_blocked_cells_array`, shared by every routing step). Dumping the
   obstacle map the braid hands the router for that first lane on both
   boards (`is_blocked` / `is_via_blocked` / the two cost maps over the
   lane's region, `tmp/frame/braid_obs_*.py`) and comparing them shifted:
   blocked, layer costs and stub costs identical, THREE via-keep-out cells
   different -- each exactly at the keep-out boundary of a roundrect
   passive's pad (R3.1, R3.2, C12.2 at 0.5663 mm: a 3-4-5 cell of the
   corner arc). The rasteriser is exact in the pad's own frame, but its
   sub-cell offset `pad.global_x - gx*grid_step` is computed from the
   absolute coordinate and carries its last bit into `dist_sq <
   margin_sq`, which has no tie epsilon. The offset is quantised to a
   nanometre at the function's entry (and in `iter_pad_blocked_cells`,
   its bit-identical twin); `_capsule_mask` already resolves its ties
   through `GRID_TIE_EPS`, and the via stamps' `off_cells` is a hypot
   (never negative), so those were robust. After this the first lane's
   map is byte-identical, and the whole chain is: **K15 14 / 673, K28
   36 / 1596, K41 86 / 2489 in both frames, the K41 braid logs identical
   line for line.** This one is production code: it joins the engine
   changes owed a corpus A/B before main (TODO 7).

What the exercise says about the chain: every stage that orders by a
raw distance or rasterises from an absolute coordinate has a tie on a
regular pitch, and a translation is the cheapest instrument for finding
them -- the two chains differ only where a last bit decided something.
The same instrument on the braid's own keys (`braid.py` has thirty
keyed sorts on raw projections and distances) is available whenever a
pose gate shows the braid leaning.

### The relaxation, vectorised and convergent: `taut_fast` (2026-09-08; the DEFAULT since that evening, `TAUT_FAST=0` for the old one)

The user asked for the current algorithm sped up, its oscillations
removed and its rounds vectorised, and `taut_fast.relax_many` is that:
the same model (curve shortening on a densified polyline against discs
and capsules, the ends frozen, a shortcut every 25 rounds), with

- contact as a CONSTRAINT: a pushed point lands on the boundary plus a
  micron, not 0.01 mm past it, so smooth-then-project is a projected
  gradient step and the round trip that kept every contact string at its
  limit cycle for 400 rounds is gone;
- a capsule the string crosses transversally (more than 30 degrees off
  its direction) is transparent for the crossing -- a dive, which the
  cleanliness check already tolerates -- while one it runs along still
  pushes, so the mean spine stays off foreign tracks;
- every string of a `taut_paths` call in ONE array (a Jacobi sweep,
  diffusion number 1/4; a trust region of 0.05 mm per round so no step
  can jump a thin capsule; per-point candidate lists of the 16 nearest
  obstacles within 0.9 mm from the shared base model, the string's own
  net masked), coarse spacing (0.48 mm) then fine (0.12);
- convergence judged where it can be seen: the shortcut-and-densify
  polyline stopping between blocks (Hausdorff below 0.02 mm), and a
  string that touches nothing leaves after one block;
- a Douglas-Peucker pass before each shortcut -- a relaxed string is
  straight between contacts and the chord along a straight run needs no
  clearance test (the shortcut had become 1.8 of 2.2 s: 64,000 tests);
- the output projected once more after its final densify, so chord
  points between two contact points are not left inside the disc.

Measured on the front article (`tmp/frame/fast_batch.py`): K28's 28
strings 8.9 -> 1.9 s (x4.6), K41's 41 strings 7.9 -> 3.0 s (x2.7,
its strings still spend their round budget: the tangential creep of
contact points is diffusion-limited too). The strings are cleaner: the
old relaxation's wedged oscillation left SA7's and SWE's strings 0.19
mm INSIDE pads U1.Y20 and Y21, the new goes round them; the disc-side
"disagreements" between old and new are those, and ties at touching
pads. The bench through the chain (`TAUT_FAST=1`, `tmp/frame/tf1.out`):
K15 16 / K28 38 / K41 94 vias against 14 / 38 / 98 -- equivalent, a
different draw of the same plan loop. Chain time, memo warm: 10 / 35 /
161 s against 17 / 27 / 182 s; cold (every string computed): 22 / 87 /
342 s. The memo tags its entries `#fast`, so the two algorithms'
strings never mix inside a run.

**Default since 2026-09-08 (user decision).** The old relaxation is a
known-wedged algorithm whose K41 strings we liked by accident; this
one is the same model with the defects removed, faster and convergent,
and the bench draw it changes (K41 86 -> 106 on the final engine, K15
14 -> 16, K28 36 = 36) is the chain's knife edge, by the same
reasoning that let the fanout's cell rule change the bench. The status
table at the top is measured with it. `TAUT_FAST=0` keeps the old
per-string relaxation reachable for comparison; the two never mix in a
run (memo tag `#fast`).

What it does not reach: the 100x that would retire the memo. A cold
K41 still computes its 342 strings (~35 s batched against ~160 s), so
the sharded memo stays. The rounds are the limit now, not the
arithmetic: sliding a contact point along its disc is as diffusion-
limited as bending the string was, and only an implicit smoothing step
(with a per-round crossing check, since a large step can carry a run of
points across a 0.23 mm capsule) or the vertex solver (contacts as the
state, tangent geometry, a few iterations) goes further. (The translation
remark that stood here is superseded: the chain is exactly translation-
invariant since the section above, whichever strings it uses.)

### The exact solver: contacts, unions, and what the bench said (2026-09-08, `TAUT_FAST=2`)

Whether the rounds could go altogether was measured first
(`tmp/frame/contact_stab.py`): the batched relaxation's TOPOLOGY is not
settled early -- at 25 rounds 18 of 28 strings (22 of 41) wrap the same
bodies as at 400, at 200 rounds 3 of 28 still differ. The late rounds
are contacts sliding off one body onto the next tangent. So a solver
that tightens a seed's class exactly cannot replace the rounds; it has
to choose its contacts itself. `taut_exact.solve` does: the string is a
list of CONTACTS (a wall and the side the path keeps it on) with tangent
chords between them; from the bare chord, every blocked chord gets its
deepest blocker as a contact (recursively, so a chord along a row of
pads gets every pad in one sweep), each contact is re-placed from its
neighbours' nodes on whichever side is the shorter way round, contacts
whose chord clears them lift off, and the sweeps end when nothing
moves. Bodies are convex polygons a micron outside their boundary
(discs and capsules on one code path); a transversal capsule is
transparent, as before. Overlapping walls are walked as ONE: the entry
and exit tangents are taken over the union, and the walk follows each
body's polygon in turn, switching at the intersections and growing the
union when it meets a wall that is not a member yet -- the convex hull
was tried first and is wrong wherever a pinned end sits in the escape
comb (a rubber band pinned in a concavity lies in it). Bodies that a
contact's chord enters again merge into it. A string whose contacts do
not settle (the comb's overlapping pads and stubs can cycle between one
union and its parts: 4 of 41, 5 of 28) gets `POLISH` rounds of the
batched relaxation FROM the exact string (`taut_fast.relax_many(start=)`).
Exact on eight synthetic cases (tangent lengths analytic, all chords
clear: disc, capsule, slalom, overlapping discs, overlapping parallel
tracks, a via with its transversal track, a row of six, a pinned end in
a corner). `TRACE` lists the contacts per sweep.

On the origin article: K41 41 strings 1.2 s against the batched 1.9 s
(x1.6) and the old relaxation's ~8 s; cleanliness 36 clean / 5
violating against 31 / 10 (K28: 19 / 2 / 7 against 17 / 1 / 10). The
strings are in other homotopy classes than the relaxation's for about
half the nets (shorter for 8, longer for 23 at K41; the class is the
greedy nearer-side choice, not the flow's). **The bench, through the
chain, said no**: cold 10 / 106 / 440 s at 16 / 38 / 100 vias, warm 11 /
47 / 213 s at the same, against the base strings' 15 / 37 / 200 s at
14 / 36 / 86 (the batched relaxation's own strings: 16 / 36 / 106, warm
10 / 36 / 245). Cold it is SLOWER than the relaxation in the chain: the
plan loop computes each net's string against each round's board, and
per-string Python with a few unsettled strings costs more than one
batched array of all of them. Every set of strings is a different draw
of the same plan loop; none of the three is better than the base on
this bench, and the exact one is not faster where it counts. Not kept in the
tree: `taut_exact.py` is archived in `tmp/uncommitted_0906_archive/`
beside the abandoned planners (user decision, 2026-09-08), this section
being its record. What would change
the picture: the walk vectorised or the contacts solved in the batched
array (the per-string cost), and the class chosen as the flow chooses
it rather than by the nearer side (the draw) -- or, more simply, the
memo, which already makes a warm chain indifferent to which solver
computed its strings.

### The taut memo, sharded; and why the relaxation itself is not fast (2026-09-08)

`detect_buses.taut_paths` memoises each taut string on `ends@signature`
(the obstacle model's content hash) and persists the memo across the
chain's processes. It had grown into ONE file of 158 MB and 31,000
entries, loaded in full by every process (1.6 s, two processes per K)
and rewritten in full whenever a run added an entry -- a cold K41
dumped it twenty times. It is now `tmp/taut_memo/<xx>.json`, one shard
per two-hex-digit prefix of the signature, loaded on first touch, only
dirty shards written, each merged with the shard on disk first (a
parallel chain's additions survive), entries untouched for 14 days
dropped at write time; the old file is migrated into shards once and
renamed `.migrated`. Nothing about the answers changes.

Whether the memo could go altogether -- the user's question -- was
measured on the front article's 28 K28 strings (`tmp/frame/taut_*.py`):
13.3 s cold, 0.47 s mean, 1.0-1.5 s for the long ones, 83 % of it in
`point_violation` (59 million `hypot` calls: 400 iterations x ~200
points x the pushes). Every string that touches copper runs ALL 400
iterations: the exit test (total movement below 1e-4 mm per point)
never triggers, because a point where the string crosses a foreign
track's capsule -- a legitimate dive on a two-layer ribbon, the
assert-only "violating" strings -- is pushed out and smoothed back
every iteration, an oscillation of 0.01-0.2 mm that the periodic
shortcut-and-densify rebuilds each time. The movement plateaus by
iteration 50-100 and then creeps (SRAS: 0.34 at 25, 0.30 at 100,
0.27 at 200, 0.25 at 400). What was tried against the 400-iteration
result: stop when the movement stagnates (5 % over 50 iterations):
2.5x fewer iterations, paths drift up to 0.17 mm; 2 %: 2.3x, 0.15 mm;
count only un-wedged movement: nothing (their neighbours keep moving);
freeze the wedged points: nothing, and one string drifted 0.57 mm (the
rebuild re-forms them). A numpy red-black sweep is worth about 2x on
top and changes the update scheme; numba is not installed and mypyc
was measured and rejected for this repo. The relaxation's floor is the
contact oscillation, and a 0.17 mm drift is a quarter of the lane
pitch -- a different plan, not a faster one. The real answer is an
exact shortest-homotopic-path solver with explicit contacts (~100x, and
it converges), which is a rewrite, not a fix. Decision: keep the memo,
sharded; a new pose pays its strings once (K15 mirror: 61 s cold, 21 s
warm).

### Better spines: the medial line, relaxed, in grid legs (2026-09-08 late)

TODO 5. The spine was the straight chord between the two end zones,
with two corners when the flows bent; take4's relaxation of the members'
mean taut path had been pruned as never reached at K28. It is back, in
four pieces, all in `corridor.py` / `taut_fast.py` / `braid.py`:

- **When.** A straight, CLEAR chord is never relaxed -- the bench's
  corridors all are, so the ladder is byte-identical (K15 16 / 341,
  K28 36 / 1574, K35 61 / 1435, K41 112 / 2258; K51 1 open 129 vias
  on both arms). The middle is relaxed when the flows bend by more than
  30 degrees, or when a corridor already laid stands in the chord, or
  when a big part (10 pads or more) that the lanes cannot THREAD does
  (`braid.unthreadable`: some pair of its pads closer, edge to edge,
  than a track plus two clearances). A part the lanes can pass between
  is transparent to the spine: a 2.54 mm header with 1.7 mm pads leaves
  0.84 mm between pins, and the chord with the island logic threads 27
  of 28 K28 lanes through it at 46 vias where a spine bent round it
  shipped 57 vias and 4021 segments; with 2.3 mm pads (0.24 mm gaps)
  the chord ships 3 open and 12 DRC and the bent spine 0 / 0.
- **From what.** The members' mean taut path between the two end zones
  (`resample`, `mean_path`), from the teeth's centroid to the stubs'
  centroid along their own flows: the bundle's medial line, in the
  homotopy class the taut paths chose. (The straight branch's axis
  through the MIDPOINT of the two centroids is kept for the chord; for a
  relaxed spine it left the teeth 2 mm off the axis and the top lanes in
  the part.)
- **Against what.** `RampedObstacles`: the big parts' pads, inflated by
  the bundle's half-width, and every corridor already laid as ONE tube
  (its spine at a lane pitch, inflated by its half-width plus this
  one's), each obstacle's inflation ramped by the distance to the nearer
  end -- nothing within the amount, all of it from twice the amount, so
  a fat neighbour never reaches a thin corridor's end zone. The
  half-width is the larger of the nominal (a lane pitch per member) and
  the ends' actual spread across their flows: a face of 15 teeth at the
  ball pitch is 5.9 mm half-wide where the pitch says 2.8. Per-LANE
  tubes (take4's) made a later spine snake between an earlier
  corridor's lanes (bench K15 SA9: seven vertices, corners to 43
  degrees); whole tubes leave both chords clear.
- **How.** `taut_fast.relax_spine`: the strings' own rounds (Jacobi
  under the trust region, projection onto the boundary, a shortcut per
  block, coarse then fine, done when the resampled polyline stops
  moving), with the ramp as a per-point, per-obstacle inflation; a tube
  crossed transversally is transparent, one run along pushes. A string
  several chords long returns the chord. 30 ms for a 20 mm chord.
- **In what shape.** `octilinearise` + `clear_legs`: the relaxed arc
  simplified at 1 mm, every off-grid leg replaced by the two grid legs
  that span it in the order whose corner clears the model, then every
  leg pushed out of the ramped model along its own normal by the depth
  it violates and re-cut against its neighbours' lines (a grid leg is a
  chord of the arc and cuts inside it: the 45-degree leg into the bottom
  of a dip lay 1.8 mm nearer the part than the string had settled). The
  arc itself was measured first: its bands sit at shallow angles to the
  router's grid and every lane became a staircase -- channel article
  K15, 2683 segments for 15 lanes against 590, 2501 of them under 0.3
  mm. A bundle turns a part in legs at 0, 45 and 90 degrees, as a
  human's does.
- `Spine.project`'s outer-wedge branch and `Spine.lane_xy`'s corner
  rendering are back from take4 (a lane piece across a bend was drawn
  as its chord), with one change: a corner is MITRED on both sides. The
  arc take4 drew on the outer side is what curved tracks do; on an
  octilinear router the band of an arc is a staircase, and every lane
  took one round every corner (`chanD` K28: 2492 segments, 2087 with
  mitres; K15 839 -> 735). A cell in the outer wedge projects at the
  larger of its offsets from the two legs' lines, the mitred offset
  polyline that passes through it.

**Where it engages, and what it measures.** The bench has no bent bundle,
and neither does the L-shaped article (`bend_bench.py`: the DDR moved
south-east): the plan puts every berth on the face that faces the
source and both arms are identical at K15 and K28. The corpus has no
two-layer BGA-to-BGA pair whose bus bends (`muzy_zynq2`'s TSOP
destination has no berth menu; the chain is BGA-to-BGA). So the case is
the TODO's own: a part standing in the channel (`channel_bench.py`: the
DDR 20 mm further east, a synthetic 2x5 through-hole header in the
chord; the spine sees parts of 10 pads or more). The two articles, from
the bench, byte-reproducible:

    python3 channel_bench.py tmp/chanH.kicad_pcb 160.0 64.56 136.0 60.0 --right 178
    python3 channel_bench.py tmp/chanD.kicad_pcb 160.0 64.56 136.0 60.0 --right 178 --bottom 90 --pad 2.3 --drill 1.2
    BASE=tmp/chanD.kicad_pcb DEST=DU1 bash chain_k.sh TAG 15 28

and the L-shaped one, `python3 bend_bench.py tmp/bendL.kicad_pcb 138.0 84.0`.

| article | K | straight chord | relaxed, octilinear |
|---|---|---|---|
| header at 2.54 mm pitch, threadable (`chanH`) | 15 | 0 open, 18 vias, 590 segs, 12/15 in band | identical (transparent to the spine; bent: 18 vias, 672 segs) |
| same | 28 | 0 open, 46 vias, 1166 segs, 27/28 in band | identical (bent: 57 vias, 4021 segs) |
| header with 2.3 mm pads, no track passes (`chanD`) | 15 | 0 open, 24 vias, 899 segs | 0 open, 26 vias, 692 segs (9/15 in band; radial ramp: 20 vias, 735 segs, 7/15) |
| same | 28 | **3 open, 12 DRC**, 52 vias, 1867 segs | **0 open, 0 DRC, 54 vias, 1777 segs** (13/28 in band; radial ramp: 64 vias, 2087 segs, 19/28) |

The straight chord threads 27 of 28 lanes between a 2.54 mm header's
pins at K28 and is the better frame there, which is why such a part is
now transparent to the spine; where no track can pass, the chord's
frame runs through the part, the island logic repairs the lanes one by
one, and at K28 three ship open with twelve DRC, while the relaxed spine
takes the whole ribbon under the part in 45-degree legs and ships
clean. The in-band count on `chanD` K15 is 7 of 15 for the relaxed
spine against 10 (the last call routes the rest; the vias still favour
the relaxed frame) -- the leg repair above was the answer to that and
is measured below.

The leg repair changed nothing there: the grid legs already cleared the
RAMPED model. The 3.55 mm the island logic saw is the ramp itself --
nothing is inflated within one half-width of an end, and with the
teeth's spread the half-width is 5.9 mm, so a part 6 to 8 mm from the
launch centroid stands in a 12 mm end zone where the parts are barely
inflated; the 7 mm dip was for the header's FAR pads. The obvious
correction -- the parts inflated by the ribbon's half-extent
interpolated from the teeth's spread to the stubs', with no ramp -- was
built and measured and LOST: full-width inflation from the launch makes
the string wander, the grid legs became a sawtooth of eleven (corners
46, 45, -90, 45, -90, 45, -45, 45), and both articles shipped 3 open
at K28 (`chanD` 69 vias, 3 to 5 of 28 lanes in band).

**The ramp, along the flow (2026-09-08, last).** What was wrong with
the radial ramp was not the ramp but what it measured: the distance to
the end CENTROID, so a part 7 mm ahead of the teeth and one 7 mm beside
them were the same, and the dead zone grew with the ribbon's width. The
parts now ramp ALONG THE FLOW, anchored at the end zone the spine
already has (the launch leg: the teeth's spread along the flow plus
half a millimetre): nothing inside it -- a part beside the teeth stays
the lanes' business and the string's frozen end is never inside an
inflated obstacle, which is what wrecked the unramped variant -- then
one millimetre of inflation per millimetre of run past it, up to the
half-width, because a ribbon of 45-degree legs cannot shift sideways
faster than that; a part the string can reach is inflated exactly as
much as the lanes can honour. The tubes of earlier corridors keep the
radial ramp (a corridor's exits may sit among another's stubs in any
direction). Measured on `chanD`: K28 64 -> 54 vias and 2087 -> 1777
segments at 0 open; K15 20 -> 26 vias (two more lanes in band, 735 ->
692 segments) -- the K15 draw moves by that much between any two
variants here, the K28 gain does not. A sub-pitch leg left by the
quantisation (a 0.55 mm step between the flat and the climb) is merged
into its neighbours (`merge_short_legs`), which is where the K15 draw
moved from 24 to 26. The ribbon's asymmetry (the joiners' side inflated
as wide as the head-on side) is the remaining slack in the dip.

Two things the work said about the frame itself. The ribbon is
ASYMMETRIC: joiners come in on one side, and it narrows toward the
target where the exit slots pack at a lane pitch; one scalar inflation
is the ribbon's widest side everywhere, so the dip is deeper than the
lanes need (chanH K15: 7 mm). The right model is per-side, per-s extents
(the teeth's about the launch axis lerped to the stubs' about the
arrival axis), which the relaxer can carry as easily as the scalar.
And a chord tolerance for the grid legs is a real parameter: at 0.25 mm
an arc of radius 4 mm kept eight legs and became a sawtooth (5 of 15 in
band); at 1 mm it is one or two chords.

![chanD K15, the straight chord](img/spine_chanD_k15_chord.png)

*K15 on the unthreadable-header article with the straight chord: the
frame runs through the header and the lanes are squeezed under it one
by one (24 vias).*

![chanD K15, the relaxed spine in grid legs](img/spine_chanD_k15_relaxed.png)

*The same with the relaxed spine (the final rules: the along-flow
ramp, grid legs, mitred corners): the ribbon rounds the part in
45-degree legs, lanes a pitch apart through the bend (26 vias, 692
segments against the chord's 24 and 899). Rendered by
`tmp/render_eco.py`, whose defaults are now a faint plan overlay and
bright copper for every track.*

### Memory: the chain under a gigabyte (2026-09-08 night)

TODO 10. The chain reached 2 GB of real memory at K28, and two chains
side by side had the system killing background tasks (the machine has
8 GB). Measured stage by stage and attributed to the line, it was four
things, none of them the router's search; all four are fixed with the
copper IDENTICAL -- every segment and via of every board below equal
as a set (`copper_same.py A B`; a file diff cannot say, the UUIDs
differ run to run) -- and every stage of the chain is under a gigabyte.

**How it was measured.** `mem_chain.sh TAG K...` is `chain_k.sh` under
`mem_watch.py`: every second, for each process of the chain, ps's RSS
and top's MEM and CMPRS, and `mem_report.py` prints the peak per
process. Read the MEM column: it is the physical footprint, compressed
pages included, and a peak read as RSS under memory pressure is LOW --
macOS compresses pages out of RSS, so the 1956 MB of the evening's
first measurement (RSS, sampled beside another chain) was itself an
undercount. Attribution came from two instruments: `MEM_TRACE=1`, which
stamps every braid log line with the seconds since start and the
process's peak RSS so far (a jump names the phase), and stamps each
step of `connect()` -- window, base map, band cells, band stamped, soft
stamped, routed -- with the same; and `tmp/memtrace_run.py`, the stage
run in-process under tracemalloc with a sampling thread that keeps the
top allocation sites at every new high. The tracer sees the Python side
only, and on a run that holds 16 million small objects it costs a
gigabyte of its own, so its absolute numbers are not the process's; its
site lists are what mattered.

**What the memory was** (the `chanD` article of the spines section,
K28; the footprint column of `mem_report.py`):

- The fanout stage, 1161 MB. The taut memo's shards, parsed from JSON
  into nested lists of two-element lists: 150 shards touched, 177 MB on
  disk, resident at 3.6 times the disk size (measured: 4.9 MB of shards
  became 17 MB), 550 MB in 15.9 million objects and still growing at
  the last sample. And the braid's obstacle memo (`_OBS_MEMO`), one
  model per net per layer per BOARD FILE, where every realized round of
  the plan loop writes a new board (`src1`, `src2`, ...) and nothing was
  ever evicted: 1580 models, 220 MB.
- The braid stage, 1746 MB. Per lane attempt, `_band_cells`: every
  window cell outside the lane's band, as rows for the router -- 3.1
  million of a 1.77-million-cell window's 3.5 million -- built by
  projecting every cell onto the spine and evaluating the band with a
  dozen window-sized float64 intermediates alive at once: the first
  attempt took the process from 148 to 564 MB, and the ladder's wider
  windows (2.1 million cells, 4.2 million cells outside) more. In the
  rip phase, the min-cut probe's soft stamp: a disc of cells at EVERY
  Bresenham point of every priced lane -- 197 cells at half-width 8 on
  the 0.025 mm grid -- 12 million int64 rows for one probe, 660 MB at
  the peak moment (the arrays were the tracer's top two sites, 331 and
  330 MB). The memo again, 27 shards, 116 MB. And the Rust map itself:
  a 1.77-million-cell window with 3.1 million blocked cells is ~300 MB
  of hash tables per attempt, and mimalloc keeps ~200 MB of a dropped
  map for the next one (measured in isolation: the same with
  `MIMALLOC_PURGE_DELAY` 0 and 10, and the in-process setting from
  `rust_alloc` reads back as 10 in mimalloc's own option dump on this
  machine), a one-time floor rather than growth.

**The four changes.**

1. `connect._walk_capsule_cells`: the union of the discs along a walk as
   ONE span per column. Exact: a Bresenham walk visits every column
   between its ends and consecutive centres differ by at most one cell
   per coordinate, so in any column the discs' intervals overlap or
   touch and their union is contiguous -- [min over the taps of (the
   walk's lowest y in the tapped column - h(ex)), max of (highest +
   h(ex))], h(ex) = isqrt(hw^2 - ex^2). Checked against the disc union
   on 2184 random walks at seven radii: identical, no duplicate cell;
   11.5x fewer rows for a 2400-cell lane at half-width 8; a millisecond
   per lane. Rows int32 from the start. The map keeps the MAX cost per
   cell, so the deduplicated stamp is the same map.
2. `band()` in `braid.band_of`, and `_band_cells`, in STRIPS of 64
   columns: the projection into two window-sized arrays a strip at a
   time (`Spine.project` is per point), the sample edges `lo1`/`hi1`
   computed once on the whole window's sample set exactly as before,
   every per-cell formula -- an interpolation onto those samples, a
   comparison, a searchsorted -- per strip, and the rows built int32 per
   strip in the order one nonzero over the whole mask gave them. The
   first attempt's band cells: +72 MB instead of +416.
3. `detect_buses`: the memo's resident form is a flat double array per
   entry (`_compact` on read, `_expand` on write); the JSON on disk is
   byte-identical (three real shards written back equal to the byte),
   and a shard now costs about half its disk size resident instead of
   3.6 times. `memo_stats()`, and a line at exit -- `taut memo: N
   shard(s) resident, M MB on disk` -- so the count is in every log.
4. `braid._OBS_MEMO` keeps the models of the two most recent boards
   only (`_obs_remember`): a model is a pure function of its key, so an
   evicted one asked for again is rebuilt.

**Measured** (footprint peak per stage, MB; grades and copper identical
in every row):

| article | K | fanout before | fanout after | braid before | braid after |
|---|---|---|---|---|---|
| chanD | 15 | 532 | 201 | 1621 | 569 |
| chanD | 28 | 1161 | 303 | 1746 | 771 |
| bench | 15 | -- | 181 | -- | 229 |
| bench | 28 | -- | 316 | -- | 503 |
| bench | 41 | -- | 442 | -- | 511 |

The braid alone on chanD K28, same machine: 158-163 s before, 147 s
after (the 41-second row sort of 2026-09-08 was the same stamp; its
rows are now a twentieth).

**The band into the static bitmap (the fifth change, the same night).**
What the table above still carried on the Rust side was the band
itself: four million cells outside the lane's corridor stamped through
`add_blocked_cells_batch` into the map's ref-counted per-layer hash
tables, ~300 MB a window (measured in isolation: 3.1 million cells,
+305 MB), and the allocator keeping ~200 MB of it after the map is
dropped. The map has had a static keep-out BITMAP since #422
(`add_static_blocked_cells_batch`; `is_blocked` ORs it, the search's
step and via moves both test through `segment_blocked` -> `is_blocked`,
the frontier sink records a statically blocked cell exactly as a
refcounted one, and the base map is already stamped through it by the
proxy in `build_base_obstacle_map`) -- the band went through the hash
path only because that was the call `connect` made. It now goes to the
bitmap, a strip at a time (`_band_cell_strips`; the full array is never
built), and an older binary without the API takes the hash path.
Measured on the K28 braid alone: mimalloc's committed peak 329.5 -> 89.6
MB (`MIMALLOC_SHOW_STATS=1`), the process high-water 1021 -> 797 MB, the
first attempt's band +29 MB where +72 (and +416 before the strips),
130 s where 147; copper identical as a set, and the chain / bench rows:
chanD K15 fanout 201, braid 569 -> 315; chanD K28 303 / 771 -> 775 (the
smoother's, see below); bench K15 181 / 229 -> 179, K28 320 / 503 -> 327,
K41 442 / 511 -> 584 (the bench K41 rows were sampled beside a traced
braid running on the same machine; its K41 braid alone is the number to
re-read).

With that, the braid's peak is no longer routing at all: the process
sits under 375 MB for the whole lay and rip (104 of 126 s) and climbs
to 670-825 MB in the 22 seconds of the production octilinear smoother
(`smooth_octolinear_chains`, #536) that runs after every lane is laid,
and STAYS there (rss 299 MB before it, 451-800 after). Stamped around
that one call (`MEM_TRACE=1`: ps, `vmmap --summary`, and a tracemalloc
window): Python holds 103 MB at the peak inside the smoother and 13 MB
after it -- nothing is kept -- while `MALLOC_LARGE (empty)` goes from
27 to 230 MB resident in 37 regions. Those are the clearance sweep's
matrices (`_seg_foreign_seg_dist`: a sample every 0.02 mm along a span
against every windowed foreign segment, float64, eight per call),
freed, and kept by macOS's malloc as dirty empty regions because every
call's matrix is a different size and none reuses another's. Neither
tracer could see it: tracemalloc counts live Python objects, mimalloc
is the Rust side (peak commit 90 MB).

**The sweep in row chunks (the sixth change, production code).** A probe
of 300 sweep-shaped calls at random sizes left 636 MB resident; the
same calls with every matrix capped at 65536 elements (512 KB of
float64) left 39 MB and ran faster (1.9 s against 2.1 s; a 64 KB cap
was slower, 3.1 s). So `_seg_foreign_seg_dist` and `_seg_foreign_pad_dist`
in `py_router/single_ended_routing.py` now run in row chunks of that
cap (`_SWEEP_CHUNK`): every element is computed from its own sample and
its own foreign item and the result is the min, so the chunked sweep is
the one-matrix sweep to the bit, and a call under the cap takes exactly
the path it always did. Measured on the K28 braid: the smoother now
RELEASES memory (rss 315 MB before it, 216 after; Python's peak inside
it 14 MB where 103), the process high-water 797 -> 372 MB, the footprint
peak 342 MB; copper identical. Standard routing, which shares the
helper: `route.py` on `kicad_files/splitflap_driver` copper-identical
(1423 segments, 168 vias), and on `kicad_files/flat_hierarchy` (487 segments, 39 vias; the committed
sweep run against the chunked one in one serialized script), and the four tests that cover the
helpers and the smoother pass (`test_pad_shape_distance`,
`test_617_pcb_modification_hole_clearance`, `test_760_hole_local_clearance`,
`test_smooth_route`). Chain and bench with everything: chanD K15 fanout 202 / braid 365, K28 303 / 372
(the fanout stage is the memo and the plan loop, untouched by this);
bench K15 181 / 163, K28 317 / 226, K41 450 / 523 -- against the
evening's starting point of 532 / 1621, 1161 / 1746 on chanD. Copper
identical on all five boards; chanD K28 chain 2 min 20 s, bench K41
chain 2 min 24 s, the machine otherwise idle.

Left: `blocking_analysis._NET_CELLS_MEMO`, production, ~85 MB by the end
of the rip phase; and the taut memo in the fanout stage (K41: 244
shards, ~140 MB resident at the compact ratio).

### The human's K41, and the sides the plan hands the braid (2026-09-09, fourth session)

*Status (2026-09-10 evening): the code of this section was NOT committed -- reverted when the seventh session's loop landed (only what the 107-via board used was kept); it is in `awx/tmp/handoff_0910b/all_uncommitted_0910b.patch` (with `reberth.py`, `wave.py` and the full `fanout_from_plan.py` / `select_moves.py` beside it).*


**The human's 70 vias, measured** (`tmp/net_table.py`, `tmp/swim_anatomy.py`,
`tmp/corridor_census.py` on `boards_set24/allwinner_h3_ddr3`, the same 41
nets): 38 of 41 nets carry exactly 2 vias, one under U1 or just past its
tooth and one beside the DU1 ball; the corridor itself changes layer 9
times. The structure is two pages with NO weaving: page B is the nine
SE-corner address nets (SA8 SA11 SA15 SA12 SBA1 SA1 SA0 SA2 SA4) plus the
three outermost south riders, page F everything else. The nine dive AT
their balls, climb under U1's own field on B (column gaps, then a 45-degree
run) so they reach the corridor already in the nested order the north ride
needs, cross the whole F bundle for free, ride DU1's north flank nested and
descend into the field each in its own column gap -- the back layer under
BOTH fields is free 2-D room, which is why the human's ends cost nothing.
The five DQ nets that must cross their F neighbours dive ONCE, and the
dive is timed: SDQ0 first (its B run goes back west of the others' dives),
the down-movers after -- two phases along s, B riders then DQ divers.
Ours on the same nets: 112 = source 17 / corridor 35 / destination 60
(human 11 / 9 / 50), 21 swimmers at 3-5 corridor vias each.

**The wave schedule** (`wave.py`, opt-in `BRAID_WAVE=1`, `WAVE_BEAM`,
`WAVE_SLOPE`): every lane moves once, launch offset to target offset, on
one layer, in ordered waves; a move is legal when every lane it passes
over is on the other layer at that moment; a lane that cannot move that
way weaves (the old swimmer, transparent to the movers, priced by its
alternations + `WEAVE_EXTRA`). A beam over move sequences, with the
two-page schedule as a second candidate so the result is never dearer
than the braid's own. On the human's copper orders it reproduces the
human exactly: 9 corridor vias, 3 waves, 0 weavers (wave 0 the B riders
and the F holders, wave 1 SDQ0, wave 2 the DQ down-divers). On the
BRAID'S orders it is no help: the base K41 permutation has a two-chain
cover of 23 (18 lanes must swim by construction; 332 inversions against
the human's 74) and the two-page candidate wins everywhere (K28 36 = base,
K35 61 = base, byte-identical -- the first schedule of a plan pass is the
two-page one so the pitch relaxation is the base's). Where the beam wins
(K28 first call, 20 v / 4 waves; the forced K41 board, 53 v / 23 weavers)
the EXECUTION loses: waves squeezed into the 4.5 mm region at 0.32 of
their slope need, the forced K41 board 2 open 116 v where the two-page
braid on the same berths routes 86. The wave model is a faithful judge of
copper-like orders and a poor one of the braid's abstract orders, which
over-count crossings the copper resolves in 2-D (join legs, tails, the
basins). Not a default, kept as the judge it is.

**The sides are the lever, measured.** `PLAN_FORCE_DST=file.json`
({net: {direction, layer}}) restricts a net's destination menu to that
class (`fanout_from_plan._force`; a pinned net the greedy cannot place gets
its whole menu back and the selection reruns, `select_dst`), and
`tmp/human_sides.py` reads the human's class per net off its copper. With
the human's DU1 sides and NOTHING else changed (same braid, flag off):

| arm | open | DRC | vias | source / corridor / destination | note |
|---|---|---|---|---|---|
| base | 0 | 0 | 112 | 17 / 35 / 60 | 21 swimmers |
| hf (sides, 5 nets unplaced by the menu's channel model, routed to the bare balls) | 0 | 0 | **86** | 16 / 19 / 51 | 17 swimmers |
| hf2 (sides, fallback: 41 placed) | 0 | 0 | **91** | 12 / 30 / 49 | 15 swimmers, chain 101 s |
| hf3 (hf2 + the human's U1 sides) | 0 | 0 | 91 | = hf2 | round 0 kept: the source force never reached the board |
| human | | | 70 | 11 / 9 / 50 | |

The plan's own via model already prefers the human's sides -- predicted
95 (base) vs 75 (hf2) -- with the ride term a wash (~92 via-equivalents
both); what is missing is a SEARCH over sides (`select` is greedy on its
own move cost and the plan loop only judges what it produced). The
residual over the human on hf2: the DQ B-divers +13 (SDQ9 6, SDQ14 5,
SDQ15 4, SDQ11 4: they weave where the human dives once, timed), the B
riders +12 (SBA1 6, SA4 6, SA8 4, SA11 4: they cross EACH OTHER because
their launch order is the row order, not the nested one the source basin
gives the human), SCKE1 +2.

What follows, in order (TODO 1):
1. **A side chooser.** Search over (direction, layer) classes per net,
   judged by the plan model's predicted vias (`vias_from_pages` through
   the braid's planner; the ride as a tie-break, not a term), on the warm
   planner (`WarmPlan` in `tmp/handoff_0909c/all_uncommitted_0909.patch`,
   134 lines: classify / offsets / Schedule per candidate on spines built
   once). The reverted `refine_dest` is the generator; its objective
   (swimmer count) was the wrong one.
2. **The source basin move.** A dogbone escape whose B run under the
   field goes to a CHOSEN exit row through free column / row gaps, so a
   set of riders can be handed to the corridor in nested order -- the
   human's SA8 (ball W18) exits at the top of the face. `escape_moves`
   enumerates it, `select_moves._lane_span` must price a two-gap path.
3. **The single dive for the DQ class:** a swimmer whose crossings sort
   (B-page lanes before F-page lanes along its line) dives once, at the
   boundary; the free weave only for the rest.

### The berth chooser: side and layer together, judged by the braid's plan model (2026-09-09, fourth session, later)

*Status (2026-09-10 evening): the code of this section was NOT committed -- reverted when the seventh session's loop landed (only what the 107-via board used was kept); it is in `awx/tmp/handoff_0910b/all_uncommitted_0910b.patch` (with `reberth.py`, `wave.py` and the full `fanout_from_plan.py` / `select_moves.py` beside it).*


`select_moves.select` fixes a side per taut-path bus by cut capacity and
copper (at K41: the 16-net bus refused left and right, "9 slots for 16
nets", and sent down whole), prices a dog-bone's via at three surface
moves, aligns layers last, and the plan loop only JUDGES what it
produced. The human decides layer and side per net by what the lane must
cross, and its DU1 sides pinned on our plan routed K41 at 91 (above).
`fanout_from_plan.refine_sides` (opt-in `PLAN_SIDES=1`; `SIDES_ROUNDS`,
`SIDES_SWIM`, `SIDES_CONFIRM`, `SIDES_GROW`, `SIDES_EVERY_ROUND`) is the
search that reaches such assignments, after `select` at the first source
round and at every destination pass:

- **The judge is the braid's own plan model, warm.** `braid.WarmPlan`
  (setup and spines once; per candidate the corridor's classify, offsets
  and Schedule on the SAME code paths run() takes) gives pages and each
  flank block's leg layer in 5 ms -- the screen -- and with `full=True`
  lays the lanes (`lay_lanes`: exit legs placed, their economics decided
  along s) for every page lane's layer changes, which is `plan_braid`'s
  own number to the via (77 = 77, 74 = 74 on the two K41 assignments) in
  0.1 s -- what a move is accepted on. Price = `plan_ends.vias_from_pages`
  with those changes, a swimmer at what it PAYS (`SIDES_SWIM` 3.5: K41
  base 78 vias over 21 swimmers, the human's sides 53 over 15 -- the
  model's 2 is its dive and surface), the ride at `VIA_MM` per via, an
  unplaced net as a swimmer plus its two vias. The pages-only price could
  not tell select's assignment from the human's (158 vs 157); with the
  changes it tracks the routed boards.
- **Three generators, because a single berth moved from the greedy's
  start looks worse until a whole block has turned** (the valley the
  swimmer-count refinement of the previous session sat in): one net to
  another (side, layer) class -- its cheapest conflict-free move, with an
  exchange when the lane is held by exactly one other net -- one full
  confirm per net; a whole flank block to the other layer at once; a
  GROUP grown in launch order from a seed moved to a class, its launch
  neighbours pulled to the same class while the screened cost holds, the
  best prefix confirmed. Capacity is the menu's channel model throughout.
- **Freeze what worked** (TODO 7): a berth the fanout engine laid exactly
  as asked keeps that move through the destination passes that follow
  (`plan_state(pin=)`), so a pass re-chooses only the misses. Without it
  every pass re-planned all 41 from scratch and banned 10-16 more (the
  first run: 55 bans, 3 open, 1 DRC, 7 min); with it K41 converged at
  pass 4, "every berth laid as planned". It rides with `PLAN_SIDES` (the
  flag-off K51 chain with it graded 5 open where the record says 1; its
  own A/B is owed).
- **An approach test per candidate berth** (`approachable`): a straight
  run of 0.9 mm outward from the exit point on the move's layer, clear of
  the static copper (the caps under the array, which the menu's channel
  model never sees) and of every other chosen berth's via site.
- **Realized as planned, verified** (`tmp/berth_check.py`, independent of
  the chain's audit): on the refined K41 board every one of the 41 stub
  ends is within 0.05 mm of the sidecar's, on its layer, on its face, and
  the face counts (down 18 / left 5 / up 12 / right 6) are the pass line's;
  the base's shipped board had 2 of 41 off by more than a millimetre (its
  last pass shipped 4 misses). The sidecar records the ACHIEVED berth, so
  the asked-vs-laid proof is the audit (41/41 exact) plus those counts.
- **What the chooser cannot see: a pocket with one entrance.** K15 with
  the chooser leaves SA9 OPEN (12 vias, base 16 clean): SA9's berth is
  the base's own (down/F at the SE corner), but SA7 moved from the north
  face to the adjacent down/F berth, and its B ride's corner via plus its
  stub took the one approach the pocket has (balls above, the cap bank
  east); the approach test passes it (0.8 mm apart) and the base routed
  SA9 at the last call with SA7 elsewhere. The braid's frontier says
  "walled by static copper". The fix is the braid-level RE-BERTH loop
  (TODO 1's first bullet): a refused lane feeds back to the plan, that
  net takes another class, the fanout re-lays, the braid re-runs.

Measured on the bench, one rung at a time, flag off (`lb`) and on (`ls`),
chain wall time in brackets (the table is filled from `tmp/ladder_*.out`):

| K | flag off | chooser, first source round | chooser, every round (2 rounds) | chooser, every round (3 rounds) | human |
|---|---|---|---|---|---|
| 15 | 16 [0:17] | 12, **1 open** [0:20] | 12 [0:22] | 16 [1:10] | 22 |
| 28 | 36 [0:31] | 40 [1:15] | 40 [1:27] | 34 [2:24] | 46 |
| 35 | 61 [1:00] | 56 [2:19] | 56 [2:22] | 60 [5:12] | 58 |
| 41 | 112 [2:20] | 108 [4:16] | 91 [5:33] | 94 [5:14] | 70 |
| 51 | 129, **1 open** [3:10] | 103, **2 open** [5:46] | 152, **1 open** [10:08] | 132, **1 open**, 3 DRC [10:52] | 85 |

Read the table with the edicts: the chooser is clearly better at K41 in
every configuration (112 -> 91 / 94 / 108; 91 is the pinned-human-sides
experiment's number, reached with no human input) and at K35 (61 -> 56 /
60), a coin flip at K15 and K28 (two rounds: 12 and 40; three rounds: 16
and 34), and WORSE at K51 in every configuration (the knife edge: two
rounds 152 v with 1 open, three rounds 132 v with 1 open and 3 DRC, first
round only 103 v with 2 open; the base 129 v with 1 open). Every rung
costs 1-3 minutes more than the base (the refinement is ~13 s a call at
K41). NOT a default: a mixed ladder, over the time budget, and the K51
regression says the search's berths are harder for the braid at scale
in a way the plan model does not price.

The zynq article could not be measured: its flow-frame turn fails (TODO 6).

### K51: why the passes never converge, and what the human does there (2026-09-09, fifth session)

Measured on the chooser session's ladder boards -- `tmp/lb_k51` (flag off:
129 vias, SBA1 open, 3:10) and `tmp/le_k51` (chooser every round: 152
vias, SA10 open, 10:08) -- against the human's `allwinner_h3_ddr3` on the
same 47 DU1 nets (81 vias; the 48th net, SZQ, has no DU1 ball: its other
pad is R6 on the back, 0.09 mm from the U1 ball, one via). Tools:
`tmp/corridor_census.py`, `tmp/net_table.py`, `tmp/swim_anatomy.py`,
`tmp/entry_faces.py`, `tmp/human_sides.py`, `tmp/judge_vs_actual.py`,
renders by `tmp/render_eco.py` (whole pair, the north riders at U1, DU1).

**The non-convergence, mechanically.** The source loop runs its nine
rounds with 1-3 nets unplaced every round (the channel model has no
conflict-free move for them) and keeps round 2 on the judged floor. The
destination loop then misses 12 / 7 / 7 / 3 / 3 / 2 / 4 / 3 berths over
eight passes, 38 bans; the chooser re-chooses 10 berths at pass 0, 3 at
pass 1, then 0 / 0 / 0 / 0 / 2 / 0, so passes 2-7 are in effect the base's
own ban loop (shipped boards of 396 / 324 / 352 / 394 / 381 / 384 tracks:
no progress). SA12 is banned four times, SZQ three (a "berth" on a resistor
the BGA engine can never lay -- pure waste), SBA0 and SWE three. Unplaced
at the end: SA9 (R3, the SE corner, the chronic one since K41) and SDQ5
(H7), fanned by the engine's own choice. The braid then gets orders whose
two-page schedule seats 17 F + 8 B and leaves 22 swimmers (base 19 F / 9
B / 19), spends 64 rip re-lays (base 29), and SA10 stays open after three
rescues and six rips.

**Where the vias go, and what the judge saw.** `judge_vs_actual`: the
chooser's plan model predicted 97 vias for its choice and the board
routed 152 (swimmers 59 -> 89, page-F lanes 14 -> 34); the base's model
said 110 and routed 129. `swim_anatomy`: le's 21 swimmers carry 84 vias
over 341 crossings (SCKE0 31 crossings, SA4 28, SBA1 23, SRST 23, SDQ0
22), lb's 18 carry 58 over 280. The judge prices every swimmer at the
flat `SIDES_SWIM` 3.5, so it bought page seats with swimmers that cross
the whole bundle. SA10 is the type case: its tooth is at (127.0, 69.5),
the SOUTH end of U1's east face (the bench's own escape from ball P19 at
y 65.9 runs 3.5 mm south inside the field before leaving), and the
chooser gave it `surface/up/F` on DU1's NORTH face -- the human's own
class for SA10 -- an 8.5 mm diagonal across 46 lanes that nothing can
lay. Source round 4 had moved that tooth to its own row
(`dogbone/right/B` at 65.56) and was discarded for round 2 on a judged
floor that cannot see the difference.

**The permutation, decomposed** (`corridor_census`: y-order at the first
and last corridor stations, the two-page cover of the permutation):

| source order -> destination order | inversions | LIS | two-page cover | swim |
|---|---|---|---|---|
| human -> human | 89 | 30 | 41 of 47 | 6 |
| lb -> lb | 113 | 23 | 36 of 46 | 10 |
| le -> le | 152 | 22 | 34 of 46 | 12 |
| human source, le berths | 304 | 17 | 29 | 17 |
| le teeth, human berths | 291 | 18 | 28 | 18 |
| human source, lb berths | 331 | 16 | 25 | 21 |
| lb teeth, human berths | 363 | 18 | 26 | 20 |
| `hd51`: human DU1 sides PINNED on our teeth (`PLAN_FORCE_DST=tmp/force_h51_dst.json`, chooser off) | 101 | 25 | 35 of 47 | 12 |

The human's orders are matched to EACH OTHER: its source order with our
berths, or our teeth with its berths, is worse than either of our boards.
The pinned experiment is the direct test of the destination half alone:
with the human's DU1 classes forced (36 of 47 nets land on the human's
side AND layer; the source loop kept the bench's own teeth) the chain
grades **0 open, 0 DRC, 135 vias in 3:32** -- complete where both the
base (129, SBA1 open) and the chooser (152, SA10 open) ship an open net,
but on 20 swimmers (base 19, chooser 22) carrying 83 vias, and the plan
model again under-prices its swimmers (predicted 52, routed 83). The
human's sides buy completion at K51, not the human's corridor: the
inversions fall from 113 to 101 against the human's 89, and the source
order still differs from the human's by 175 pair swaps (le 250, lb 324).
So the chooser, which searches the destination against OUR teeth, is
right to refuse the human's sides at K51 (it agrees with the human on
direction for 29 of 45 nets, on layer for 20, on both for 11), and no
destination-only search can reach the human's corridor. The lever is
joint, and its source half is a move we do not have.

**What the human does.** 40 of the 47 nets carry exactly 2 vias, 7 carry
none; the corridor changes layer 11 times over 11 nets (ours 28-40 over
20). Three blocks, each nested at both ends and each on a constant layer:

- **North, 11 nets, all on B from the U1 ball to the DU1 ball** (SA8 SA14
  SA11 SA15 SA10 SA12 SBA1 SA1 SA0 SA2 SA4). Eight of them dogbone AT the
  ball and climb north-east under U1's own field on B, between the balls
  at 45 degrees (SA8 10.2 mm, the rest 5-6 mm), and leave the east face
  at y 58.9-62.1 in the nested order the north flank needs; SA0 SA2 SA4
  leave at their own rows (y 67-68.5) and climb on B in the corridor's
  first 2.7 mm, done by x 130.3 -- BEFORE the first DQ dive (x 129.4 to
  132.5): the wave. They ride DU1's north flank at y 56.8-60.0 (ten
  tracks in 3.2 mm), descend on B through the north band's column gaps
  into the 3.2 mm middle street (rows 7 and 3 both border it) and surface
  beside the ball.
- **Middle, the 18 DQ nets on F, straight at DU1's west half**: 7 with no
  via at all, 8 diving ONCE at x 129.4-132.5 and surfacing at the ball,
  entering left 9 (5 F / 4 B), up 3, down 6.
- **South, 18 nets**: 13 on F entering the south face in column gaps, 5 on
  B (SCS1 SCS0 SCAS SA7 SRST) diving at U1 and surfacing south or east of
  DU1 (SA7 and SRST enter R2 and T2 from the east face).
- DU1 classes: down/B 13, up/B 8, right/F 7, left/F 5, up/F 5, down/F 5,
  left/B 4 -- 25 B / 22 F. Ours: le 10 B / 35 F (with our orders the B
  page holds 8 nets, so a B berth costs a change and the chooser fled
  B), lb 24 / 23.
- U1 exits: right/F 23, right/B 13 (the eleven riders and two more),
  down/F 11 -- and NO along-row B escape: everything that is not riding
  leaves on F, so the back layer under U1's east half is free room for
  the climb (47.7 mm of the human's K51 copper under U1 is on B, over 14
  nets, all of it climbs and dives). Our source fanout puts along-row B
  escapes under U1 running EAST (`dogbone/right/B`, `via_in_pad/right/B`):
  8 nets on the bench itself, 14 after the plan's source rounds -- in the
  way of any climb.

**The moves we do not have, in the order the evidence ranks them:**

1. **The source exit-ROW choice** (TODO 1 step 2): a dogbone or
   via-in-pad whose back-layer run under the field goes to a CHOSEN point
   on the face. `escape_moves` enumerates kind x direction x layer, but a
   move's exit point is its ball's own row or column; the human's eight
   climbers exit 5-10 mm from their row. It comes with a joint source
   LAYER assignment (the non-riders stay on F under the field, or the
   climb has no room), which is what `refine_source` would have to
   choose; today it moves teeth one at a time within the same semantics
   and every source round keeps every face's rank order.
2. **A judge that prices a swimmer by what it crosses** and refuses one
   whose launch-to-target gap the corridor cannot carry, instead of the
   flat 3.5: the chooser under-priced its own K51 choice by 55 vias and
   let SA10's diagonal through; the base's model was off by 19.
3. **The timed single dive** (wave 0 the riders, then the DQ divers):
   `wave.py` exists and is inert on our orders; with copper-like orders
   from 1 it is the schedule the human's corridor has.
4. **A re-fan of a refused lane's SOURCE tooth** at the plan (SA10), not
   another rip at the braid.
5. Hygiene: SZQ (no DU1 ball) out of the destination loop; stop the ban
   loop when the chooser makes no move and the miss set repeats.

### The destination loop converges: class bans, adopted gaps, frozen berths, incremental passes (2026-09-09, fifth session)

*Status (2026-09-10 evening): the code of this section was NOT committed -- reverted when the seventh session's loop landed (only what the 107-via board used was kept); it is in `awx/tmp/handoff_0910b/all_uncommitted_0910b.patch` (with `reberth.py`, `wave.py` and the full `fanout_from_plan.py` / `select_moves.py` beside it).*


Graded on the FANOUT STAGE's own terms before any braid ran (`tmp/fo_ladder.sh`
runs `fanout_from_plan.py` alone; `tmp/fo_sum.py` reads any fanout log):
passes, misses per pass, whether "every berth laid as planned" was reached,
exact berths at the shipped pass, unplaced nets, DRC, the plan model's
predicted vias and swimmers, wall time.

**What the misses were** (K51, every pass, `tmp/conflict_probe.py` reproduces
pass 0's choice to the ask): the base's 62 misses over eight passes were
33 "other face" (the engine walked six gaps either side on both layers
and kinds and left by another face: the plan had asked a north-band ball
to run through the street and the whole south band, or a 7 mm back-layer
row run past three via-in-pads), 13 layer/kind, 12 gap slides, 4
refused (3 of them SZQ, whose far pad is a resistor on the back, 0.09 mm
from its U1 ball -- no BGA berth exists). A crossing-aware conflict
model flags 10 of the 11 misses but also 17 of the 33 exact berths, and
the human's classes pinned lay 46 of 47 exact at pass 0: no geometric
proxy separates them, the engine is the only authority. The loop's job
is to USE its verdicts well, and it did not: one move signature banned
per miss (a class of a dozen gaps banned one gap a pass), nothing frozen
unless `PLAN_SIDES`, and every pass re-fanning the whole array from bare.

**The loop now (`DST_LOOP=class`, the default; `DST_LOOP=old` is the loop
above, byte-identical to the recorded control):**

- **Ban by class.** An "other face" verdict bans every move of that net on
  the asked face; a layer miss bans the asked (face, layer); a kind miss
  the asked (face, layer, kind).
- **Adopt a slid gap** (`GAP_ADOPT` 1.0 mm): a berth laid on the asked face,
  layer and kind but along the face from the asked gap becomes the plan's
  (the menu move of that class nearest the laid stub end is pinned).
- **Freeze what worked, every arm**: a berth laid exactly keeps its move;
  a net the plan left unplaced keeps what the engine laid when the menu
  names it. Frozen nets are `trusted` in `select_moves.select`: placed
  without the model's conflict test, whose false positives (a via-in-pad's
  back run past a neighbour's via, two legs the A* weaves apart) would
  otherwise leave one of a frozen pair unplaced every pass.
- **Incremental passes.** Pass 0 fans the array from bare; every later pass
  keeps the previous pass's board, strips the destination copper of the
  re-chosen nets only (a 2 mm window round the array; the source stubs
  stay) and re-fans those against everything else's copper. Measured
  before this, with the three rules above alone, K41 missed 7/6/7/4/3/1/1/2
  and K51 9/4/3/1/2/1/1/1 -- nearly every late miss a berth laid EXACTLY
  the pass before, displaced when the whole array was re-fanned and a
  neighbour's new ask was laid ahead of it (the engine claims deepest
  first), and then banned for it.
- **Off-array nets** (SZQ) are not planned at the destination; the braid
  reads them off the board (a partial sidecar is the plan for the nets it
  names).

| K | old loop (control) | class bans + freeze | + incremental passes | + chooser (`PLAN_SIDES=1 SIDES_EVERY_ROUND=1`) |
|---|---|---|---|---|
| 28 | 2 passes, converged, 28/28; 3 swimmers, 38 model vias; 13 s | same | same | 3, converged, 27/27 (1 unplaced); 5 / 35; 49 s |
| 35 | 8 passes, **never**, 34/35; 13 / 65; 38 s | 3, converged, 35/35; 12 / 65; 20 s | 4, converged, 35/35; 13 / 66; 21 s | 3, converged, 34/34 (1 unplaced); 7 / 51; 73 s |
| 41 | 8, **never**, 37/41, 1 unplaced; 21 / 95; 58 s | 8, never, 38/40, 2 unplaced; 21 / 85; 65 s | 4, **converged**, 40/40, 1 unplaced; 13 / 79; 39 s | 5, converged, 37/37, 4 unplaced; 12 / 67; 191 s |
| 51 | 8, **never**, 42/47, 3 unplaced; 19 / 105; 76 s | 8, never, 43/44, 2 unplaced; 18 / 86; 158 s | 4, **converged**, 45/45, 2 unplaced; 18 / 80; 37 s | (below) |

Every rung now converges, in three or four passes, faster than the
control (the incremental passes re-fan 9, 5, 2 nets instead of 47), with
every asked berth exact and DRC clean; the plan model's own swimmers at
K41 fall from 21 to 13 and its vias 95 -> 79 (K51 105 -> 80) with no
chooser. What is left at the plan stage: the one or two UNPLACED nets
per rung (a net whose classes were banned away and whose remaining menu
conflicts with frozen neighbours in the model -- laid by the engine's own
choice and kept, absent from the sidecar) and the chooser's cost in time
(its every-round refinement, ~13 s a call) and in unplaced nets (4 at
K41: its exchanges leave more nets without a model-clean move).

| K | chooser + incremental passes, K51 (`dis`) |
|---|---|
| 51 | 5 passes, converged, 45/45, 2 unplaced; 17 swimmers / 91 model vias; 318 s |

**The braid on the converged boards** (`tmp/braid_fo.sh`, braid.py + grade
on the fanout board as it stands; the control is the recorded `lb` chain):

| K | control | converged plan (`di`) | converged plan + chooser (`dis`) |
|---|---|---|---|
| 35 | 0 open, 61 vias, 13 swimmers | 0 open, **66**, 13 swimmers, 0 rips, 50 s | -- |
| 41 | 0 open, 112, 21 swimmers | 0 open, **107**, 14 swimmers, 24 rips, 121 s | 0 open, **97**, 14 swimmers, 18 rips, 107 s |
| 51 | **1 open** (SBA1), 129, 19 swimmers | **2 open** (SA11 SA14), 108, 19 swimmers, 15 rips, 95 s | **3 open** (SDQ11 SDQ3 SDQ9), 139, 17 swimmers, 37 rips, 204 s |

Mixed, as the braid grades it (K41 better by 5 and 15 vias, K35 worse by
5, K51 loses completion): a plan converged to berths the ENGINE lays is
not yet a plan converged to berths the BRAID reaches. Every net left open
is a swimmer refused at the last call after three rescues and a
blocker-directed rip -- its berth exact and frozen, its lane walled -- and
the class bans drove two of them to far berths the model priced cheaply
(SDQ3, a south-band ball, asked left, refused, then given a 7.6 mm
back-layer run east to the far face because the whole west face was
banned; SA11, a north-band ball, given the south face through the street
and the south band). And **the plan model cannot see which swimmers the
braid will refuse**: counting swimmers by their launch-to-target index gap
(>= 10 places of 45) gives the control 13, `di` 10, `dis` 14 -- and the
pinned-human-sides arm, the one board that completes, 16 (gap sum 345,
the largest). The refusals are walls of copper, not order.

So the next mechanism is the braid-level RE-BERTH loop (TODO 1's first
bullet), now that a destination pass costs seconds: a lane refused at the
last call bans its berth class at the plan, an incremental pass re-fans
that net alone against the routed board's copper, and the net is routed
alone against the frozen copper (take4's `negotiate_stubs`); and, for the
plan's own choice, the source exit-row move, which is what keeps the
human's swimmers short in the first place. A cheaper first probe: the
chooser with `SIDES_EVERY_ROUND=0` on the incremental loop (its every-round
refinement is 150-250 s of the K41/K51 fanout stage).


### The re-berth loop, the boxed end, and the climb (2026-09-10, sixth session)

*Status (2026-09-10 evening): the code of this section was NOT committed -- reverted when the seventh session's loop landed (only what the 107-via board used was kept: the CLIMB move in the source menu, `SRC_CLIMB`, stayed; the re-berth loop and the source chooser did not); it is in `awx/tmp/handoff_0910b/all_uncommitted_0910b.patch` (with `reberth.py`, `wave.py` and the full `fanout_from_plan.py` / `select_moves.py` beside it).*


Three moves were on the table at the end of the fifth session, in the
order the evidence ranked them: a braid-level RE-BERTH loop (a refused lane
bans its berth class, an incremental pass re-fans that net alone, the net
is routed alone against the frozen copper), the SOURCE EXIT-ROW move (the
human's climb under U1 on the back layer), and a cheap probe first (the
chooser with `SIDES_EVERY_ROUND=0` on the incremental loop). All three
were run; the measurements below are what each turned out to be.

**The cheap probe** (`PLAN_SIDES=1 SIDES_EVERY_ROUND=0 ./chain_k.sh dis0`,
the chooser on the first source round only, on the converging destination
loop): **K41 0 open, 0 DRC, 88 vias in 2:20** (fanout 73 s, braid 67 s) --
the best K41 the chain has produced (flag off 112, incremental loop alone
107, chooser every round 97 at 5-7 min) and inside the time budget. K51:
2 open (SDQ12 SDQ3), 0 DRC, 120 vias, 6:36 (the fanout stage 3:39: the
first-round refinement over 47 nets). A clear K41 win; no K51 completion. The whole ladder, both arms timed
back to back on a quiet machine (`tmp/ladder_0910.log`):

| K | flag off (the converging loop, `ctl0`) | probe (`dis0`) |
|---|---|---|
| 28 | 0 open, 36 v, 1:02 | 0 open, 36 v, 1:40 |
| 35 | 0 open, 66 v, 2:00 | **1 open** (SDQ10), 54 v, 2:34 |
| 41 | 0 open, 107 v, 4:23 | 0 open, **88** v, 2:20 |
| 51 | 2 open, 108 v | 2 open, 120 v, 6:36 |

One rung up, one rung down (a net lost at K35): not a default (edict 3);
`PLAN_SIDES=1` stays opt-in.

**The re-berth loop** (`reberth.py TAG K`, a driver that runs AFTER the
braid on the routed board, `REBERTH=1` in `chain_k.sh`): per refused net,
the berth it has is measured off the board and its (face, layer) class
banned; a new berth is chosen for that net alone -- its menu enumerated
against the routed board's copper, every other net pinned to the berth
it was laid to (`_menu_match` on the fanout board), the selector's costs
deciding -- re-fanned by `fanout_once`'s incremental pass against the
routed copper, and routed alone by a one-net `braid.py` run (every other
lane static; ~3 s; its refusal record says why). Two things it found
before landing anything:

- **The plan state must be read off a SOURCE VIEW of the fanout board**
  (its destination copper stripped, `reberth.source_view`): `plan_state`
  reads the ends with no `dest_ref`, and on a board fanned out at both
  ends that attributes the berth tip as the source end of half the nets
  (28 of 47 berths "not in the menu" until this was fixed).
- **On a routed board the berth menu is EMPTY.** With every neighbour's
  berth and lane as copper, no straight escape of SA11's or SA14's is
  clear on any face or layer, so the loop asks the ENGINE for a face
  instead (`fanout_once(face_asks=)`: a bare-face hint, the engine's own
  search choosing gap, layer and kind). It lays berths; the braid then
  refuses every one of them, because:

**The boxed end is the TOOTH.** The refusal record now says where a
walled lane is walled (`braid.Corridor._note_wall`, from `connect`'s
forward and backward frontiers kept apart, `report['blocked_fwd'/'_bwd']`):
each end's pocket reach in mm, and a verdict -- the end whose pocket is
under a third of the other's is the boxed one (`WALL_POCKET_RATIO`), two
pockets under `WALL_POCKET_MM` 2.5 is boxed at both ends, two comparable
pockets a wall across the corridor. On the recorded K51 (`di`): SA11's
pocket round the tooth reaches 2.6 mm (384 cells) and round the berth 24 mm
(10,000 cells, the cap); SA14 5.2 / 28 mm. Both are boxed at U1's east
face, where the fifth session's diagnosis put the human's climb -- and a
destination re-berth cannot answer that. So the loop's other move is
**pad to pad**: the net stripped to its two balls and routed by the
production router (`route.py --nets`) against the board as it stands, the
engine that re-fans an end AND routes in one search, up a ladder of
(the chain's 0.127 width, no rips) -> (0.10, the 2-layer advanced tier,
disclosed) -> (0.127, 3 rips) -> (0.10, 3 rips), the first rung whose
whole-run grade is no worse (the net closed, no other net opened, no DRC)
shipping. Measured:

| K51 board | braid | after `reberth.py` | what landed |
|---|---|---|---|
| `di` (incremental loop) | 2 open (SA11 SA14), 108 v | **1 open**, 112 v, 39 s | SA11 pad to pad at 0.10 mm, no rip: 5 vias, 49 segments -- round U1's WEST side and over the top of the whole bundle to DU1's north-east corner (~50 mm; `tmp/di_rb_k51_SA11.png`). SA14: no path on any rung. |
| `dis0` (probe) | 2 open (SDQ12 SDQ3), 120 v | **1 open**, 132 v, 51 s | SDQ12 pad to pad at 0.127 with 3 rips: a short corridor swimmer with one dive (4 vias), three neighbours re-laid at +8 vias (`tmp/dis0_rb_k51_SDQ12.png`). SDQ3: no path on any rung. |

Completion first, so on the chain's own scale both boards improve; the
copper is the copper of a last resort (SA11's detour is nothing a
designer ships), and the second open on each board has NO path with the
others frozen, even with three rips. Opt-in (`REBERTH=1`), not a default.

**The climb** (`escape_moves.enumerate_moves(climb=k)`, `SRC_CLIMB=k` on
the source menu; `Move.climb`; the selector's conflict test over every
gap stretch a move takes, `_lane_spans`; the mirrored frame carries it;
`SRC_CLIMB=0` byte-identical, verified: the flag-off K51 fanout stage
reproduces the recorded one to the pass). Measured on the human's board
first: SA11's tooth is a dog-bone via beside the ball, a short 45-degree
leg on B into a COLUMN GAP, 2.7 mm straight north along it (x 125.78) and
out of the east face at y 61.7; SA12 4.0 mm at 126.86, SA15 3.1 at 126.07,
SBA1 4.2 at 127.14 -- the back layer under a BGA has no pads, only via
barrels. The move: a dog-bone or via-in-pad whose run first travels up to
k pitches along a gap and leaves the face at a chosen row (left/right) or
column (up/down); a via-in-pad steps half a pitch into the gap first. On
the K51 bench it adds 1062 climbs to 199 plain moves (SA11 17, SBA1 31,
SA15 58; SA10 and SDQ3 none: boxed). The plan with `SRC_CLIMB=8`
converged (44/44 exact, 3 unplaced), model swimmers 18 -> 14 but model
vias 80 -> 91, and took 209 s against 30 s (`plan_ends._refine_source`
judges every candidate move by the full plan cost). It chose nine climbs
-- SA15 9 pitches north on B, SDQ8/SDQ15 (DQ nets), SA10, SBA1 3 pitches
SOUTH, SBA2 out the WEST face -- not the human's pattern (its eleven
north riders all climb north on B into one nested order), and the braid
graded **4 open, 97 vias, 16 swimmers** against the control's 2 open /
108 / 19: worse. The source refinement moves one net at a time, and a
single climb pays only when the whole group climbs together -- the
chooser session's finding at the destination ("a single berth moved from
the greedy's start looks worse until a block turns"), now at the source.

**The upper bound, both ends human** (`tmp/human_src_realize.py`): the
human's U1 teeth read off its board (face, layer, kind and the row) and
laid on the bench by our own `source_realize` with the climb menu
(`--climb=16`): 29 nets matched a class, the engine laid all 29, 22 at
the human's row (SBA1, SA12, SA10's rows were unreachable in the menu --
neighbours' current escapes block those climbs, the co-move problem --
and the engine's negotiation slid seven). The chain from that bench, our
destination plan (`hs`) and the human's DU1 sides pinned too (`hh`):

| arm (K51) | open | vias | time | note |
|---|---|---|---|---|
| `di`, the recorded incremental loop (control) | 2 (SA11 SA14) | 108 | 2:12 | |
| `hd51`, human DU1 sides pinned on our teeth (fifth session) | 0 | 135 | 3:32 | the one complete board |
| `hs`: human-like teeth (22/47 exact) + our destination plan | 1 (SDQM0) | 136 | 4:52 | the source loop moved the teeth again (kept round 5) |
| `hh`: human-like teeth + human DU1 sides pinned | **7** | 125 | 3:38 | the sides that complete on OUR teeth strand seven on the approximation |

The human's source order is not reachable with our engine's menu on this
bench -- 18 nets had no move of the human's class at all, 7 more could not
reach its row -- so this is not the human's board and not an upper bound
of the climb; it says that a PARTIAL copy of the human's teeth is worse
than either whole, which is what the fifth session's permutation
decomposition predicted (human source + our berths: 304 inversions
against 113 / 89 for the two wholes).

**The source chooser, built and measured** (`refine_climbs`, `PLAN_CLIMB=1`
with `SRC_CLIMB=k`; `WarmPlan` gained tooth overrides, the symmetric of its
berth overrides): each tooth's face, layer and row chosen together, judged
by the braid's plan model, with a per-net generator over each class's rows,
the group grown in launch order, and the co-move as an exchange with the one
neighbour whose copper alone closes a class. Two menu readings, both at K51,
three source rounds:

| arm | menu | plan on paper | engine laid as asked | fanout stage | braid |
|---|---|---|---|---|---|
| control (`di`) | -- | 18 swimmers | -- | 30 s | 2 open, 108 v, 19 swimmers |
| `cl` | bare array (every run net's copper excluded) | swimmers 19 -> 7 -> 5 -> 3, 22 teeth a round | 13-16 of 22 (asks for runs across the field to the far faces) | 393 s | 2 open, **172** v, 19 swimmers, 45 rips |
| `cl2` | honest (legs clear of every other net's copper; exchange with a sole blocker) | 19 -> 12 -> 15 -> 12, 12 / 7 / 6 teeth | 9/12, 6/7, 5/6 | 483 s | **4 open**, 134 v, 18 swimmers, 73 rips |

The paper swimmers fall every round and the braid routes the same 18-19.
So the judge was calibrated (`tmp/judge_calib.py`): over the 16 K51 boards
with both a fanout and a routed board, the plan model's prediction against
the routed vias (+3 per open net), swimmers priced flat (`SWIM_VIAS`) and
priced by the layer changes their crossings with the page lanes force
(`Corridor.swim_profile`, `SWIM_CROSS=1`, opt-in, byte-identical off):

| | Spearman vs routed | mean abs error |
|---|---|---|
| flat swimmers | 0.20 | 44 vias |
| crossing-priced swimmers | 0.36 | 40 vias |

A rank agreement of 0.2-0.4 on a spread of 97-172 routed vias: the plan
model does not rank K51 plans, with either swimmer price, and every chooser
judged by it (the destination chooser of the fifth session, both source
choosers here) came out worse at the braid for that reason. The
crossing-priced swimmer over-counts twice over (a straight line's forced
changes, 6.5 per swimmer, against the 3.2 the router pays), and neither
price models completion at all, which is where the arms differ most.

**Where this leaves the list.** The destination loop is converged, the
braid's refusals are located, the climb is in the menu and a source
chooser exists -- and the JUDGE is the wall: at K51 the plan model ranks
plans at 0.2-0.4 rank agreement with the braid, so no search over it can
be trusted there, and the fast judge that could (the router itself on the
swimmers, against the page lanes' real copper rather than their lines --
the 0909 swimmer planner's lesson) is the next thing to build before any
chooser is run at K51 again. Two things that ARE settled: the probe
config's K41 (88 vias in 2:20), and the re-berth loop as the backstop for
whatever a plan leaves open. Tools: `reberth.py`, `tmp/human_src_realize.py`,
`tmp/fo_ladder.sh`, `tmp/braid_fo.sh`, `tmp/render_eco.py`.

### The braid's verdict re-plans the ends: `replan.py` (2026-09-10, seventh session)

The sixth session ended at the judge: the plan model ranks K51 plans at
0.2-0.4 rank agreement with the braid, so no chooser judged by it can win
there. This session takes the other road (Andy's): **use the route itself
as the judge and the previous route as the price** -- run the real braid,
read what it did to each net, re-plan the nets it paid for, realize them,
route again, and iterate; and verify at every step that the plan, and
then the new plan, are what the fanout LAID on each side, rather than
judging on completion and via counts alone. `replan.py TAG K` is that
loop, on a fanout board F (both ends laid, its plan sidecar beside it)
and its routed board R:

1. **The verdict** off R: each net's class in the braid's own schedule
   (page F / page B / swimmer), its real vias (the board's, the ends'
   taken off), refused and the boxed end, its ranks, and the braid's
   OWN CENSUS of what walled it -- the lanes on its blocked frontier and
   the min-cut probe's crossing set, which the braid already prints per
   rip. The plan model's prediction for the plan F carries is recomputed
   and `real - predicted` per net becomes `plan_ends.RESIDUAL`, the
   learned price every judge below runs with (empty by default: byte-
   identical everywhere else). Measured on the recorded K51 (`di`): the
   plan's pages match the braid's (18 swimmers on paper, 19 routed); the
   error is all in the swimmers' price (+19 vias over 19 swimmers, +4 on
   each page), and SA1 and SA3 are not in the sidecar at all.
2. **The bad nets**: the refused, then every net whose lane costs three
   vias or more (`--worst` of them a round), and the **gatekeepers** --
   good nets the census names in the way of two or more bad ones
   (`--gate-min`), carried across rounds. They are not frozen: their
   lanes are re-laid in the bad nets' probes and they are re-planned
   themselves in a second phase.
3. **Candidates** per bad net at each end the verdict allows: the
   destination menu on the bare array with the berths in its way named as
   a CO-MOVE set (re-fanned with it, the engine negotiating; at most
   `MAX_COMOVE`), the source menu with climbs (`SRC_CLIMB` 14; the honest
   menu, legs clear of every other net's copper), ranked by the judged
   cost with everyone else as the board carries them.
4. **The probe**: the real router on R. The net is stripped to its tooth,
   the lanes in the way of the new end (geometrically, and by the census)
   are stripped with their teeth and berths kept, the asked end is laid
   by the production engine and AUDITED (face, gap, layer, kind, the
   climb's row), and the group is braided together against the frozen
   rest, graded whole (`grade_k`). A candidate stands only when its own
   net routed and the board grades better (opens, then vias, no DRC); a
   refusal -- its own or a re-laid neighbour's -- is UNJUDGED, not a
   verdict, because the full braid rips what the local one cannot.
5. **Apply**: the standing moves (pairwise non-conflicting) realized on F
   -- teeth by `source_realize`, berths by `fanout_once`'s incremental
   pass, both audited, with the co-moved berths asked for exactly where
   the probe's engine LAID them -- the unmoved ends checked unchanged off
   the board, the sidecar rewritten from the board it sits beside. In the
   default `--mode=incremental` the probes' board, whose changed ends are
   checked against the fanout board's (same face and layer, gap within
   0.5 mm), IS the round's routed board; `--mode=rebraid` runs the full
   braid on the new F instead and keeps it only when better AND every
   move was laid in its asked class.

**Results, K51 (the recorded `di` chain: 2 open, 108 vias).** Every
board below grades 0 DRC with `grade_k` (check_connected + check_drc at
the routed floor), and every move in it was audited as laid in its asked
class with the other ends unchanged:

| run | mode | what moved | open | vias | time |
|---|---|---|---|---|---|
| `rp2` round 2 | rebraid | SA6's tooth up one row on B (climb 1), SA12's tooth out U1's SOUTH face on B | **0** | 119 | 548 s for 4 rounds (two full braids) |
| `rp2` round 3 | rebraid | + SA8's tooth out the south face on F (probe: 119 -> 115) | 0 | 135 | the full braid re-laid everything: a knife edge, rejected |
| `rp3` round 1 | incremental, census | SA8's south-face tooth | 0 | 115 | 228 s (18 probes) |
| `rp3` round 2 | incremental, census | SA12's berth to DU1's EAST face on F, SA1 and SA14 co-moved | 0 | 111 | (the process was killed for memory after its apply; the pair is `tmp/rp3b_k51`) |
| `rp5` round 1 | + joint pairs, substitutes | SCS0's berth as the engine's substitute (via-in-pad on B, DU1's south face), SCS1 co-moved | 0 | 109 | 394 s / 2 rounds |
| `rp6b` round 1 | + engine screen, synthesized asks | SBA2's tooth as the engine's own south-face surface escape | 0 | **107** | 515 s / 2 rounds |

The chain's previous complete K51 boards were 121 (the plan search of
09-04) and 135 (the human's DU1 sides pinned); 107 is the best complete
K51 the chain has produced, from general moves the real router chose --
the address nets born at the south end of U1's east face leave by the
south face and ride the corridor's bottom, which is the human's
"climb" turned the other way. Renders: `tmp/rp2_r2_{all,u1,du1}.png`
(the 119 board; SA12's south exit and SA6's dogbone one row up are where
the loop said).

**Three more things the runs asked for (Andy's three notes, in order):**
move MORE bad nets a round and do not freeze the good ones in their way --
every net whose lane costs three vias or more is a bad net, the braid's own
blocker census names the good nets in the way of two or more of them
(the GATEKEEPERS: SCS1 blocked SA11, SCS0 and SA4; SBA1 and SBA0 two each),
their lanes are re-laid in the bad nets' probes and they are re-planned
themselves in a second phase, and the census is written beside every
board the loop keeps (`<board>.census.json`) so a continuation run
inherits it; consider a TOOTH AND A BERTH together -- joint pairs are
ranked by the judged cost with both applied and probed beside the single
ends (`--joint`, on by default); and make the plan's moves REALISTIC --
the engine itself screens every candidate in a dry run on the parsed
fanout board (`engine_lays`: the end's copper taken off in memory, the
production fanout asked for the move, its laid end measured; 0.03-0.65 s
against 10-20 s a probe), a move it lays in another class goes forward as
that SUBSTITUTE (menu-matched, or synthesized from the laid end when the
menu cannot name it: SBA2's south-face surface escape six gaps west of any
menu move), and the asked class is banned. Measured on the 109-via board
the screen reproduces every probe verdict: SBA2's five moves all refused,
SCS0's south berth substituted by the via-in-pad the probe had kept, every
SA11 and SA14 climb exact. `rp5` (joint pairs, substitutes): 111 -> **109**
(SCS0's substitute berth, 394 s for two rounds); `rp6b` (the screen,
synthesized substitutes, run under `nohup` after the harness's low-memory
guard had stopped two runs): 109 -> **107** (SBA2's tooth as the engine's
own surface escape on the south face, six gaps west of any menu move;
515 s for two rounds). `tmp/rp6b_rp_k51{,_fo}.kicad_pcb` is the pair to
continue from; renders `tmp/rp6b_{all,u1}.png`.

**Where the time goes (profiled on the 107 board, one probe of a 4-net
group, `tmp/rp_prof.py`):** the Board build 0.9 s, the rankings 0.1 s,
the engine screen 0.6 s, the strip 0.05 s, the re-fan with its DRC 0.6 s,
the grade 0.8 s -- and the LOCAL BRAID 8.6 s (21 s with the machine
loaded), all of it in `connect`: FAILING searches, repeated. The attempt
ladder re-ran three identical attempts (the same 226k-, 351k- and
410k-iteration refusals for SA12 and SA11, nothing changing between
them), then the last call searched 0.9-1.6 M iterations for SA11, which
does not route. `BRAID_ATTEMPTS` and `BRAID_BUDGET_X` (opt-in, defaults =
the braid as it was) let the probe run one attempt: 8.6 -> 6.7 s with the
SAME board; the halved budget (3.8 s) loses SA11, so the probe keeps the
full budget. A round is 18-21 such probes (`--worst=6`, three per net
with the joint pair, plus the gatekeepers): 250-265 s. What is left to
cut: the census groups (a bad net re-laid with 8-12 census lanes takes
20-27 s; `--census` is 4 now), and running different nets' probes in
PARALLEL on the round's base board (two workers on this 8 GB machine),
which the sequential incremental board does not allow as written.

**What the faithfulness checks found (the reason this session was
worth its time):**

- **The plan's honest source menu names 30 of the 47 teeth the fanout
  laid** (two F tracks share one 0.65 mm gap; the menu offers one lane
  per gap). `_menu_match` is None for those; their class is read off the
  board (`measure_tooth`, which now tolerates a bare or fully routed net
  and can read the SOURCE end of a doubly-fanned board, `which='src'`).
- **The engine lays every climb the menu offers, exactly** -- SA14 at
  1..8 rows in both gaps, SA12's 101 moves -- on the fanout board's source
  view. The 0910 session's "climb infeasible even alone" was the ROUTED
  board: another net's lane already leaves the face at the climb's row.
  The selector's strict lane test (a dog-bone via in the column gap)
  refused every climb the engine threads past; it is not applied at the
  source here.
- **The east face is full at every gap midline**: with two teeth per pitch
  and the braid's rule that two teeth cannot share an exit point (any
  layer), a climb's only free exits were the four rows north of SA0.
  `escape_moves` climbs now step by HALF a pitch on the run layer -- a
  BGA's back has no pads, so a B run may leave along a row LINE as well as
  a gap midline (the human's nested riders leave at half-pitch spacing);
  measured laid exactly (SA14 at 61.98, SA11 at 63.93).
- **The destination menu is drawn on a bare array**, so it offers a
  dog-bone site the other berths' vias block (SA6: 'exact move infeasible
  even alone', the original re-laid) and surface runs seven rows deep the
  plan-follow never lays (SA12). The berths in a move's way are its
  co-move set; surface berths deeper than three rows are dropped.
- **A surface leg across another surface leg on the same layer** (SA6's
  south-face column run across SA9/SA13/SA7's east-face rows) is in the
  selector's non-strict menu and not layable: `legs_cross` filters it.
- **A slot exchange the engine negotiates on the routed board it refuses
  on the fanout board**: SA6 asked for SA8's east-face slot; on R the
  plan-follow slid SA8 0.4 mm and laid both, on F its deepest-first order
  laid SA8 exact first and its rule (a negotiation is kept only when the
  exact count rises) dropped SA6 to `via_in_pad/up/B`. So the apply step
  asks for the co-moved berths exactly where the probe's engine laid them
  (`comove_got`, menu-matched), and two standing moves may not conflict
  (SA11 and SA6 once both stood on one slot: neither laid as asked).
- **A co-moved berth grazes a lane routed against the old one** (SA4 <->
  SA1): the DRC pair names the lane, which is re-laid with the group.
- **A probe's "better" must be the net's own**: SA11's berth once "stood"
  at 107 vias with SA11 still refused, the saving being SA4 and SA8
  re-laid cheaper.
- **The local judge cannot judge the hard swimmers**: SA11 and SA4 refuse
  in every group they are re-laid in (they route only under the full
  braid's rip-assist over every lane), so every move that needs them
  re-laid -- SCS1's, SBA1's, SA11's own climb, which routes SA11 at 5-6
  vias from 10 -- is unjudged. And the full braid is a knife edge: SA8's
  local win (119 -> 115) re-braided to 135.

Traps: zsh does not split `$v` (a `set -- $v` loop hands `--view` no
argument) and `echo =====` looks a command up; a running `replan.py`
keeps the code it loaded (its braid and fanout SUBPROCESSES pick up an
edit); the machine has 8 GB and the loop was killed for memory once (its
peak RSS is now printed per round). Tools: `replan.py`, `tmp/rp_check.py`
(verdict + model-vs-real table + candidates, no probes), `tmp/rp_probe2.py`
(one probe), `tmp/rp_climb_bisect.py` (every climb of a net realized).

### The pack: every lane a taut string against its neighbour (2026-09-09)

`pack.py`, TODO 8, opt-in with `BRAID_PACK=1` and run by `write_out`
after the production smoother. The idea, and the whole of it: after a
corridor's lanes are laid, each lane's copper -- every run on either
layer and every via joining them -- is ONE polyline from tooth to
landing, and it is relaxed as a taut string between its two ends
(which never move) against the board AS IT STANDS: curve shortening
against every obstacle on its layer (pads, foreign copper, the other
lanes, vias, the board edge -- capsules inflated by the clearance and
half a track, so contact IS the clearance; a via point against both
layers at the via's own radius), plus a FOLLOW force that snaps each
track point into the tube of the settled copper on its layer wherever
the string runs alongside it, on the packing side, in plain sight. A
via moves with its lane: nothing pulls it directly, the string does.
The relaxed string is emitted run by run as octilinear copper where a
build clears, validated piece by piece at the true radii, and a lane
whose copper fails keeps the router's -- legal by construction beside
the packed ones, because every lane packed against the others as they
stood.

![K28 before the pack: the braid's copper, every river a fan of staircases](img/pack_k28_before.png)

*K28 as the braid leaves it (`BRAID_PACK` off): 1574 segments; the
bottom river is a fan of router staircases, the diagonal group from
the source's east teeth a spread of them.*

![K28 after the pack: horizontal lanes with 45-degree jogs, the diagonals packed](img/pack_k28_after.png)

*K28 packed (`pack_board.py`, 4 s): 875 segments, every lane, 0 open,
0 DRC, the vias where they were. The bottom river is horizontal lanes
with 45-degree jogs; the rides at the top are straight.*

![K41 before the pack](img/pack_k41_before.png)

*K41 as the braid leaves it: 2258 segments.*

![K41 after the pack](img/pack_k41_after.png)

*K41 packed (10 s): 1843 segments, 41 of 41 lanes, 0 open, 0 DRC. The
far-face rides round the top and east are grid legs, the source's
wraps nested chamfers; what is left off the grid is the 6-degree fan
from the source's south row into the bottom river and two coupled
corners at the passives (the section's end).*

The order across the corridor is the plan's target slots, **from the
outer lane with more room on its outer side, inward**: the first lane
hugs whatever settled copper stands beside it and otherwise goes
taut; every next lane hugs the one packed just before it that has
copper on its layer (a page lane on the back has nothing to hug in a
front lane's copper -- the predecessor and the hug side are per
layer), on the roomy side, and its elbows bulge into the room that
lane vacated. A bundle of router staircases across a corridor becomes
nested elbows only in this order, each lane moving before the lane it
would cross. Measured on K28 with the same emitter: from the wall
inward 23/53 runs octilinear and 1429 segments (it hugs the ragged
static copper and never has the room); the centre outward 28/52 and
1226 (the centre stays a staircase every hug copies); the roomy side
inward 29/51 and 949.

The emitter reads the string's own structure. The settled copper on
the layer is chained and simplified at 0.03 mm into CHORDS (a router
staircase of 0.05 mm pieces is one chord; a packed lane's grid legs
are their own), and every maximal group of points hugging one chord
at one distance becomes ONE line in that chord's direction -- snapped
to the grid only when the snap moves the line's far end by under 10
um (a 2-degree snap moves the end of a 6 mm hug by 0.2 mm, into the
neighbour) -- at the distance the string settled at; every free group
becomes the grid legs of its chords (a chord within 2 degrees of a
grid direction is one line; any other the two legs that span it, the
corner that clears). The lines are met at their intersections
(parallel neighbours merged under 60 um of offset, else joined by a
45-degree jog; a leg that reverses or falls under 50 um drops its
line and the rest are met again). A leg that still cuts the model is
REPAIRED alone -- replaced by the string's own chords between its
ends, at the 12 um the true-radius validation allows -- so a run is
any-angle only where a leg had no room: an arc round a via between
two hug lines. The whole run falls back to the string's chords only
when the build itself fails (two lines running opposite ways).

What lost on the way (all measured on K28; the session memory has the
list): the string steering the router inside a tube (the router
refused the exact pitch); a per-piece octilinear shove of the
router's copper (nothing moves in a staircase of a hundred pieces);
strings per run with the vias frozen; a relaxation coarser than the
capsules; the centre re-laid first in a world without the other
lanes (they lay across it); a chord-by-chord octilinearisation of the
string (a sawtooth); a fan pre-pass that re-laid every run across the
corridor as lead + 45-degree leg + run before the pack (inert once
the order was right: 467 against 475 lane pieces); and packing modes
before the last call.

The numbers, 2026-09-09 (`tmp/pk_braid.sh TAG 1 K`, the braid alone
on the pk0 fanout boards; base = the same braid with the pack off):

| K  | lanes packed (taut, no follow) | runs octilinear | segments base -> packed | lane mm | pack s | braid s |
|----|-------------------------------|-----------------|-------------------------|---------|--------|---------|
| 28 | 28 / 28 (7) | 31 / 54 | 1574 -> 898 | 518 -> 528 | 3.9 | 17 -> 21 |
| 35 | 35 / 35 (0) | 33 / 80 | 1435 -> 1641 | 727 -> 720 | 7.2 | 21 -> 28 |
| 41 | 41 / 41 (3) | 48 / 125 | 2258 -> 2012 | 949 -> 944 | 9.9 | 84 -> 92 |

0 open, 0 DRC and the via count unchanged at every K (36 / 61 / 112);
every lane packs. K35 and K41 come out SHORTER than the router's
copper now. (Tag `tp`, 2026-09-09 late; the rows before the "five
oddities" round below were K28 977, K35 1782, K41 2028 with K41's SA9
and K35's SDQM0 unpacked.)

**The five oddities (2026-09-09 late), each run to its mechanism with a
probe that relaxes one lane alone against the board
(`tmp/lane_probe.py`, `tmp/replay_probe.py` on a `BRAID_PACK_DUMP`
string, `BRAID_PACK_TRACE` on named points):**

- *A hat over a via cluster* (SDQ6): a legitimate wrap over SDQ15's via
  and tracks, 45 / 0 / 45 degrees. Not a defect; gone anyway once the
  lanes round it moved.
- *A spike into a pad gap* (SA9): the follow's pulls toward a track a
  millimetre above were correctly refused by plain sight, but the two
  points at the spike's base were still MARKED hugging, and hug points
  anchor the straightening, so the spike's flanks were a stretch whose
  only shortcut cut the pads. A refused pull is not a hug.
- *An any-angle corner on the outer ride* (SA0) and *near-grid chords
  at 10 and 102 degrees* (SA13, SCKE0): the repair patches an unclear
  leg with the string's chords at 12 um and then re-checked every leg
  at the 5 um octilinear tolerance, so the patch failed its own check
  and the whole run fell to coarse any-angle chords. Judged at its own
  tolerance now. The residue -- corners the router laid at exactly 0.100
  mm, 13 um inside the packer's inflated model and boxed in -- is
  handled by judging a leg against the string's own depth there (a leg
  may cut as deep as the string does) and by tightening the patch to
  the string where it is that deep.
- *A free apex* (SCS0) and the spike above: the relaxation had not
  converged. The sum of the movement over a lane's 500 points hid one
  vertex collapsing at 0.1 mm a round. Convergence now also needs every
  point's move ACROSS the string, net over two rounds, under 10 um
  (along-string drift is the smoothing evening the spacing, harmless).
  That alone sent thirteen K41 lanes to the round cap: points
  ping-ponging by the full step between a snap toward the tube and a
  push out of an obstacle. A point whose move reverses between rounds
  has its pull gain halved. One lane at the cap now, K41 pack 9.9 s.
- The patch also used to step back to the string point nearest a leg's
  end when it lay behind it, leaving 40 um spurs that check_drc flags
  as same-net soft joints (K35 shipped one). Only points that project
  inside the leg are used, and every emitted run is despiked.

**And the straightening was re-done (the SCKE0 ride, 2026-09-09
late).** With the above, SCKE0's back-layer ride ran 3 mm up the
board's edge and back down a 26-degree diagonal, 45.5 mm where 42 had
been possible. Its starting string had that detour from the router;
nothing hugged anything up there; the shortcut across was blocked by
SA6's ride and a pad; and the old straightening -- the farthest clear
chord found by HALVING the index range -- landed on the index midpoint
of the stretch, the detour's own apex, accepted the chord to it, then
the chord from it, and re-laid the same two chords every 25 rounds. The
straightening is now the TAUT PATH over the stretch's own vertices:
its Douglas-Peucker corners plus a vertex every 0.5 mm along a long
straight piece (from the foot of the east leg every chord grazed SA6's
end by a micron; from 0.3 mm up it was free), every pair tested for a
clear chord in one sparse call, the string's own arc between
consecutive vertices always an edge (a chord across a contact arc never
clears), and the shortest path through that graph re-laid straight
where it took a chord. One straightening takes SCKE0 from 46.0 to 39.4
mm; the ride is a 45-degree leg and a horizontal.

**The six oddities (2026-09-09, later; tag `cs`).** A spike on SA0's
top ride, a hook on SA15, a W on SDQ7 and a Z on SCKE1 were all lanes
packed EARLY (SA13 sixth, SA0 tenth of 41) against neighbours still at
their router positions: each went taut round copper that then moved,
and nothing revisits an early lane. Re-relaxed alone against the final
board every one of them vanished. A SECOND PASS over all lanes takes
them out and packs tighter (median 0.47 -> 0.32 mm) but measured worse
on the emission (K41 1948 -> 2272 segments, 45 -> 49 long any-angle
pieces) at twice the time -- every wrap it tightens is one more arc --
so it is opt-in (`BRAID_PACK_PASS2=1`) until the emission earns it.
The arc round SA0's east corner and SA13's step were the wrap problem
itself, so the emitter now draws WRAPS AS CHAMFERS: a free stretch
whose points sit on one disc's inflated circle (or a capsule end's),
turning 30 degrees or more, becomes the tangent lines at the grid
directions whose tangent point lies 15 degrees or more inside the arc
(a tangent nearer the arc's end ran a millimetre before the exit chord
met it -- SODT1's V), and the line meeting gives the octagon's corners,
8 % of the radius outside the circle. That room exists because the
lane packed before this one hugged the same chamfer; where it does not
(a neighbour still an arc), the corner lands inside the model and is
SNAPPED to the nearest string point before the repair -- the repair
keeps a leg's ends, and a leg that starts inside an obstacle stays
unclear however it is patched (fallbacks 22 -> 2 at K41 from that
alone). And the hug SIDE is now read off the predecessor's copper first
and the plan's lines only when the copper says nothing: SRST's plan
line lay on one side of SA0's, its ride on the other, so it never
hugged and shipped an 8 mm chord 4 degrees off; SA9 hugged that.

| K  | lanes | runs octilinear | segments base -> packed | lane mm | off-grid > 1 mm, before -> after | pack s |
|----|-------|-----------------|-------------------------|---------|----------------------------------|--------|
| 28 | 28 / 28 | 24 / 54 | 1574 -> 1036 | 518 -> 536 | 12 -> 9 | 3.9 |
| 35 | 34 / 35 | 34 / 78 | 1435 -> 1510 | 727 -> 721 | 24 -> 6 | 7.0 |
| 41 | 41 / 41 | 66 / 125 | 2258 -> 1991 | 949 -> 948 | 28 -> 21 | 9.8 |

("off-grid > 1 mm": routed-net pieces over 1 mm more than 3 degrees
from a grid direction, the proxy for what the eye calls colinearity.)
**Packing on its own: `pack_board.py` (2026-09-09, later).** The braid
now writes `<out>.pack.json` beside its board at write time -- each
lane's copper as the pack receives it, every corridor's members and
target order, its planned centrelines, each lane's tooth and stub end,
and the destination stub chain tip-first -- and `pack_board.py BOARD`
packs a braided board again from that in seconds (K41 10 s where the
braid took 92 + 10), with the stub trim re-run on the result.
`tmp/pk_pack.sh TAG K..` packs the pk0 boards, grades and renders. The
`PK_*` environment overrides on the constants and rule switches exist
for bisecting a change that way; that is how the flat-hug tolerance
below was found. Identity: the runner's pack summary matches a braid
mode-1 run to within one lane (the braid's re-anchor also sees stub
vertices its trim has already removed; now excluded).

**Five little things (2026-09-09, later still; tag `fin2`).** A 0.05 mm
bump on SRST's top ride and a 0.1 mm dip on SA9's: the FOLLOW pulled a
single point up through the gap between two pads toward a ride 0.9 mm
above, because plain sight was true for that one point and false for
its neighbours. A pull now applies only in a run of PULL_MIN (3)
consecutive pulled points; spreading each refusal to its neighbours
instead cost K35 its hugs (off-grid 17 -> 74 mm). A hug of one chord
varying by under HUG_FLAT (0.05 mm) is one line at its largest
distance; at 0.15 it moved long hugs off their neighbour for one short
plateau (K35 12 -> 48 mm). SA4's 0.8 mm hairpin above its via: the lane
was routed to the stub's far tip, and a lane arriving from the other
side ran up alongside the stub to reach it; the landing may now be any
stub vertex nearer the approach when the chord to it clears, and the
trim drops the bypassed stub. SA11's jog and SRST's corner: the fold
re-pack (a lane whose relaxed string still folds is packed again
against the final board; the full second pass stays opt-in, it packs
tighter and emits worse: K28 982 -> 1120 segments) and a vertex
THINNING of every emitted run (a vertex within 20 um of its
neighbours' chord goes when the chord clears and no grid leg is lost).

| K  | lanes | segments base -> packed | off-grid length mm (pieces > 0.5 mm, > 3 deg): commit 8389558f -> now | pack s |
|----|-------|-------------------------|-----------------------------------------------------------------------|--------|
| 28 | 28 / 28 | 1574 -> 875  | 27 -> 36 | 4.4 |
| 35 | 35 / 35 | 1435 -> 1475 | 17 -> 7  | 5.5 |
| 41 | 41 / 41 | 2258 -> 1843 | 72 -> 40 | 10.5 |

0 open, 0 DRC, vias unchanged. The proxy is noisy per board (one long
chord flipping is 5-10 mm), so it is read with the render. What is
left: the coupled corner where SRST and SA9 turn into the passives'
row gaps (two 0.2-0.6 mm chords at 63 and 157 degrees, each hugging
the other's corner), a 1-degree approach piece at SA4's landing, and
the fan from the source's south row into the bottom river, where a
6-degree lane between a pad row and a river has no room for an elbow
in any order -- a taut string is a straight line at whatever angle its
ends dictate, and only an octilinear-metric tension would make the
staircase cost the same as the chord.

**A fold is alongside nothing, and a via feels its two stretches
(2026-09-09, the circled via).** K41's SA12 had a via wedged between
R4's two pads under the resistor body, with a hairpin on F: north 0.15
mm from the via, then back south past it. The slot was open straight
below the via (the map of legal via centres in `tmp/sa12_probe.py`), the
string had every reason to slide it down, and 150 rounds moved it 40 um.
Not a wedge: the FOLLOW held it. A long F track of another net runs one
pitch north of the hairpin's tip, and at a hairpin the tangent (the
chord between a point's two neighbours) is degenerate, so the tip passed
the alongside test by accident and the follow snapped it onto that
track's tube every round, against the tension. A point whose two
neighbours are closer than `FOLD` (1.2 steps) is now alongside nothing,
in the relaxation and in the emitter's hug. With that, the hairpin
collapsed in one round and the via crept down the slot -- at 17 um a
round, because a via's smoothing pull came from neighbours 0.08 mm away
on a nearly straight string, and the convergence test then stopped the
relaxation at round 30 with the via still in the slot. So a via's pull
now comes from `VIA_SPAN` (5) points away on either side -- the joint
feels the angle between its two stretches -- and a round that moves a
via more than `VIA_EXIT` (2 um) is not a converged one. SA12's via
travels 1.41 mm to the end of its B ride, the lane is 1.3 mm shorter,
and K41 packs every lane. A wedged point also SLIDES now: the move less
its component into the nearest wall, kept when the projection converges
from there (`relax_lane`), which is the projected-gradient step for a
point pressed against a wall; it was not what held SA12, but it is
right. The price is K35 (1620 -> 1782 segments): more of its vias move,
and each move re-emits the arcs round it.

**A lane the follow would lengthen goes taut instead.** The first
version rejected 3-5 lanes per K as "longer" (a tenth plus 0.3 mm over
the router's length) and kept their staircases, which every later hug
then copied. The follow window was not the cause -- at 6, 3 and 2 mm
the same number of lanes were rejected -- the ORDER is: packing toward
the roomy side pulls an inner lane out onto the outer lane's longer
arc, with 85 % of its points hugging. So such a lane is relaxed again
with no follow at all: taut against the board, re-emitted clean, and
what packs after it hugs a line. Every lane is now packed at K28 and
K35. The wall-inward order was re-measured with this emitter for the
same reason (it shortens: K41 -1.6 % against +1.7 %), and lost on the
renders: every lane copies the ragged static copper it packs against
and the river reads as a wave (K28 1484 segments against 952).

**Speed (2026-09-09, K28 9.6 -> 4.7 s, K41 19.9 -> 10.0 s, copper
identical).** The profile (`BRAID_PACK_PROFILE=file` round the pack
in `write_out`) put 8 of 9.6 s in the relaxation, and 7 of those in
two dense point-against-every-obstacle matrices per round: the
plain-sight test of each pull (4.6 s) and the projection (2.6 s). Both
now run on the (point, obstacle) pairs whose boxes overlap only
(`_box_pairs`; a pull segment meets a handful of the lane's hundreds
of capsules), discs and capsules merged into one array per lane with
its boxes computed once (`Caps`), and the emitter's chord test goes
the same way. What is left is numpy call overhead: 25,000 pair tests
at 80 us, and the nearest-segment query per round (0.9 s at K28),
which needs the true nearest and stays dense.

What the renders say (`tmp/pk_render.sh`): K28's bottom river, a fan
of staircases before, is horizontal lanes with 45-degree jogs after,
and the diagonal group from the source's east teeth is a clean set of
parallels; K41 likewise. K35 is the row to read before this becomes a
default: its base was already clean -- long 45-degree lanes, loosely
spread -- and packing them into rivers round the swimmers' vias makes
every lane copy the wrap of the lane before it, so the river reads as
nested S-bends and the piece count goes UP (786 -> 1086 lane pieces).
The wrap itself is the open problem: a free stretch round a via emits
as an arc (repair chords) because the octilinear chamfer of a circle
stands 8 % of the radius outside it, into a neighbour at exact pitch.
Also open: the taut lanes (6 at K28) sit in the river unpacked -- a
hand router would either pack them and accept the length or leave the
gap where it is, and the packer now does the latter.

### The sidecar describes the board it sits beside (2026-09-07)

Two ways the chain's braid ran on a different world than the one the
fanout's judge had chosen, both at K41 and both silent:

- The fanout's selector can leave a net UNPLACED (K41: SA9, its menu
  banned away over eight destination passes) and fan it out unplanned,
  so the sidecar named 40 of 41 nets -- and `braid.setup` discarded the
  whole plan ("does not name every net of this run -- ignored", line 1
  of every K41 log) and read every end, layer and direction off copper,
  while the judge that chose that fanout had applied the plan to the
  other 40. A partial sidecar is now the plan for the nets it names
  (`plan from X: 40 of 41 nets; SA9 read off the board`), which is what
  `setup(plan=)` always did for a plan passed in.
- `fanout_from_plan.braid_plan_of` wrote the ASKED berth (`m.layer`,
  `DIRS[m.direction]`) while the board carries the LAID one; at K41 the
  destination passes never converge and 22 of 41 berth layers (and 5
  faces) disagreed with the copper. A braid trusting that sidecar
  routed 15 lanes to a stub end on a layer with no copper there,
  reported them routed, and shipped them open (measured: 19 of 41).
  The sidecar now carries the laid layer and face from the `achieved`
  audit record (`source_realize.measure_tooth`) whenever the fanout
  has laid the berth. Checked against the copper at K41: no berth
  point off copper, no layer wrong.

Neither fix touches the judge's inputs: the K15/K28/K35/K41 fanouts and
their sidecar ends are byte-identical to before. K15/K28/K35 sidecars
were complete already, so those grades are unchanged; at K41 the braid
now follows the plan (braid stage 82 -> 59 s) and grades the same opens
as the copper-read fallback did on the same tree (2 open on the split-
rule tree, 82 -> 88 vias). A probe that passes the plan dict explicitly
already applied a partial plan, so until this the K41 probes and the
chain disagreed (spine 0.07 mm off, different legs): a probe is
trustworthy only when line 1 of the chain's log says the plan was read.

### Exit legs and static islands: the split leg (2026-09-07)

`_leg_s` moved a leg off a static island by a lane pitch along the stub
row. At K35 that started a cascade: SA12's B leg to an F stub was
islanded only in its last 0.1 mm (C6..C9's inflated box under DU1's
bottom ball row), moved a pitch onto SA1's stub end, and every leg of
that row then jogged a pitch onto the next stub -- stub ends 0.4 apart
leave no legal foreign foot -- so SA1/SA5/SA6 were refused at the stub
on every attempt and re-laid at last call (SA1 through DU1's central gap
with three vias). The router had laid SA12's leg straight down from its
own stub all along (F to y 70.75, a via, B under the lanes); the damage
was the planned leg's VIRTUAL stamp on the neighbours.

A leg whose layer is islanded only at its stub end, with the stub on the
other layer, now keeps its s and takes the via it owes anyway just past
the island (a via's room), the last stretch on the stub's layer --
provided that stretch is island-free and crosses no lane
(`leg_split_at`, `Corridor.leg_split`). The virtual stamp is split there
(`virtual_of`), the via reserved (`virtual_vias_of`), a mark drawn. The
via count the plan implies is unchanged.

Measured (bench fb_t2q_fresh, 0 DRC, chain, one fanout): K15 14v and
K28 38v identical; **K35 55 -> 50 vias**, 0 open, SA1/SA6 in-band (30 ->
32 of 35); K41 2 open 89v -> 2 open 88v, 29 -> 30 of 41 in-band.

### Exit legs and static islands: flip or hop, and the jog rule (2026-09-07)

The same pitch move, at K41, pushed SCKE1, SCKE0 and SA15 off the F
passive cluster to one s, 3.3 mm from their stubs, and their jogs then
ran along the stub row on F over each other's stub ends: SCKE0 refused at
its stub every attempt. Two rules replace the move, both keyed on the
island's place along the leg (the island helpers are built once, above
`place_and_decide`):

- **Flip or hop, priced** (`move_cost` in `place_and_decide`): an
  islanded layer that cannot split costs, in the leg's layer economics,
  the cheaper of the veto (`ISLAND_VETO`) and the move along the row that
  clears the island, a pitch of jog worth a via -- so SDQ2 (K35) hops
  0.3 mm off C12 on its own layer for less than a via while SA15 (K41)
  takes B for one where every F move jogs over a stub. A leg still
  islanded on the layer it chose is moved as before.
- **A jog may not run over a free end** (`_leg_s.jogged`): the jog from a
  moved leg back to its end runs on that end's own layer (`virtual_of`),
  so a candidate whose jog passes over another member's free end ON THAT
  LAYER is out; with none left the leg stays. Layer-aware because a K41
  join jog on F, refused the pitch over a B tooth, took the other side
  onto SBA0's F tooth instead. Join legs go through the same `_leg_s`.

Measured (chain, one fanout): K15/K28/K35 identical to the split leg
alone (14 / 38 / 50); **K41 2 open 88v -> 2 open 77v, 30 -> 33 of 41
in-band**, SCKE1/SCKE0/SA15/SBA1/SDQ6 in-band; the fanout's judge runs
these rules too and its K41 choice did not move (sidecars byte-identical).

### The east face, and a leg's room (2026-09-07)

Two walls left at K41 after the rules above, both at a free end:

- **The east face.** SA9, SA13 and SA7 berth on DU1's east face, escaping
  east -- along the spine -- 0.25 mm apart in o with their ends just
  inside the last ball column, so they are not `far_exit` by the
  past-the-last-ball test; a leg in o at each stub's own s runs down the
  face over the neighbouring stubs (SA9's search reached s 29.82 with its
  stub enclosed by SA13's leg stamp and SA7's copper). Refused in-band in
  every arm, they cost 2-8 vias each at last call (SA6 8) -- the largest
  via pool at K41. A side exit whose berth escape direction lies within
  45 degrees of the spine's is now a far-face exit (`classify`,
  `stub_dir . spine dir > 0.7`): its leg goes beyond the array and the
  jog runs back along the stub's own line, the spine's frame extended
  for it (`build_spine`); far exits are placed last, innermost lane first,
  so their legs cross nothing; the jog's o-tolerance is the stamp's own
  reach (`LEG_O`). SA9/SA13/SA6 2 vias each, SA7 0, all in-band.
- **A leg's room.** With every candidate clashing, `_leg_s` took the
  first least-clashing one however close: SA8's join leg 0.007 mm from
  SA5's tooth, SBA2's on SDQ6's -- a stamp on both layers over the tooth,
  the net refused at its first cell every attempt. A candidate within the
  legal minimum (`TRACK + CLEAR`) of a foreign end or a placed leg is no
  candidate (`too_close`); ties among the least-clashing break by the
  most room; with none legal the leg stays.

Measured (chain, one fanout; each rule also graded alone: east face
K41 2 open 86v 38/41, room rule K41 2 open 83v 34/41 with SA4 routed for
the first time): K15 14v / K28 38v identical; K35 0 open 54v, **34 of 35
in-band** (only SODT0 refused); **K41 1 open (SBA2) 78v, 40 of 41
in-band**, 130 s (braid 60 s). Against the committed baseline before this
day's leg work: K35 55 -> 54, K41 2 open 89v 29/41 -> 1 open 78v 40/41
(human 70v). Every constant is a design constant or a direction
quadrant; nothing reads this board's names or coordinates -- but all of
it is measured on ONE bench (fb_t2q_fresh), so a second array pair is the
next confirmation before any of it is treated as a default elsewhere.

### Refusals at last call: the blocker-directed rip (2026-09-07)

A lane still refused at last call, when every other lane is real copper,
is boxed by lanes routed before it -- the sequential loss, an earlier
lane having taken the one channel a later one needs -- and no wider
window answers that. K41's SBA2 (a swimmer, tooth and stub both on B)
was refused only at the kept attempt: SCKE1's B run had crossed its
approach a millimetre before the stub, and at last call SA1 and SA2
closed the B corridor at s 10 while SA9 and SBA0 walled F at the tooth.

`connect(report=)` now hands a refusal's blocked FRONTIER back -- the
cells the A* tried to expand into and found blocked, the window it
searched, the config -- and `Corridor.rip_for` attributes it to the
lanes of this run with the production router's own blocking analysis
(`blocking_analysis.analyze_frontier_blocking`, the one route.py's rip
ladder uses). The frontier ranks lanes by EXPOSURE (the perimeter of the
reachable pocket), not by whether ripping them opens anything, so a
MIN-CUT PROBE measures it: one more search with every lane of this run
PRICED instead of blocked (`connect(soft=)`, the take4 mechanism, ported:
the clearance footprint of each lane stamped as a per-cell cost through
`set_layer_proximity_batch`) finds the path that crosses the fewest of
them, and the lanes that path conflicts with, in path order, are the cut
set -- jointly sufficient by construction. The rip ladder is that set's
prefixes, then the most exposed lanes singly: each trial rips its
victims, routes the refused lane against the rest through its ladder,
re-lays each victim against the new lane through ITS ladder (band first,
so a page lane stays on its page when it can), and a victim that cannot
be re-laid NEGOTIATES one level down with the placed lane protected (the
PathFinder move, in the braid's own vocabulary). The state is kept only
when the refused lane and every victim route; otherwise every piece of
copper goes back exactly. Static copper is never a victim: a lane whose
probe finds no path even with every lane priced is walled by stubs, pads
or other nets, says so and stays open -- a fanout matter.

Measured on K41's SBA2 (chain, one fanout, byte-identical fanout
boards; K15 14v / K28 38v / K35 0 open 54v identical, no last-call
refusal to rip):

- exposure order alone: SA8 (766 of 20000 frontier cells) ripped, SBA2
  routed at 6 vias -- a ride south round the bundle and up the
  destination's west face -- SA8 re-laid at 4 then 0 in the economy
  re-lay: **1 open 78v -> 0 open 84v**, the rip 1.4 s;
- the min-cut probe: SBA2's straight B lane, 0 vias, crosses SCKE1, SA1,
  SA2, SDQ6, SA3, SCKE0; ripping SCKE1 routes SBA2 at 4 but SCKE1 is lost,
  the pairs likewise lose SA1 -- so the cut set alone is not enough;
- the cut set with one level of negotiation: SCKE1 ripped, SBA2 at 4
  vias, SCKE1 refused and negotiated in turn by ripping SA8 (SCKE1 2
  vias, SA8 2 then 0): **0 open 82v, 0 DRC**, the rip 21 s. The first
  complete K41 on this chain (human 70v).

Time (the chain alone on the machine): K15 15 s, K28 28 s, K35 76 s,
K41 155 s (was 130; fanout 69 of it) -- most of the braid's extra time
is not the rip (18 s on its log lines) but the economy re-lay, which now
has heavier lanes to try at three widening windows each.

K51 on the same chain: the plan named 44 of 48 routable nets (SA9, SDQ7,
SA2, SZQ read off the board), the 47-net corridor routed 31 in-band and
16 at the last call, six of them by the rip (SA4, SCKE1, SA11, SBA1,
SDQ3, SA14; up to three victims, two levels), and the braid then crashed
on the singleton corridor SZQ: `corridor.build_spine` had no initial
polyline for a corridor whose launch and arrival flows bend by more than
30 degrees, because the mean-path relaxation that used to fill it was
pruned from this chain as never reached at K28. The middle is now the
chord between the two end zones in both branches (identical for a
straight corridor). Re-run: **K51 (48 routable nets) 0 open, 0 DRC, 141
vias, 339 s** (fanout 4.5 min, braid 3.2 min; 24 rips landed a lane) --
the first complete K51 on this chain, against the human's 85 vias: the
vias are where the work is now, not completion.

### The destination passes never converge, and the last one ships (2026-09-07)

At K41 the destination loop runs all eight passes (misses 10, 10, 4, 4,
2, 2, 7, 6) because a miss is not a property of the banned move alone: a
berth laid as asked in one pass fails in the next when its neighbours
change. Two things were wrong with what shipped from that, one measured
harmless and one fixed:

- The sidecar was written from the RE-PLAN after the last pass -- a
  choice no board was ever laid to -- with the last pass's `achieved`
  patched over it. At K41 the two choices named the same nets, so no
  copper differed; the sidecar now comes from the last pass fanned out
  and audited (`laid_pass`), which is the only thing it can honestly
  describe.
- "Ship the best pass by audit" was built and measured WORSE: pass 5
  (38/40 berths exact, 40/40 layers) graded 1 open 98v 30/41 in-band in
  216 s against pass 7's (34/40 exact) 1 open 78v 40/41 in 130 s. Every
  pass board was then braided (`tmp/passes_k41.sh`, committed braid):
  passes 0..7 graded 6 / 3 / 4 / 5 / 3 / 1 / 1 / 1 open at 86 / 108 / 92
  / 74 / 81 / 98 / 102 / 78 vias (pass 6 with 6 DRC). The last pass is
  the best on this board, and neither the audit's exact count nor the
  judge's cost of the choice (pass 6 the lowest at 152.56, pass 7
  156.88, pass 0 160.93 -> 6 open) predicts the braid's grade. So the
  last pass ships, as before -- with the ban set that makes it the
  engine's most feasible choice -- and the fanout's convergence stays a
  fanout-stage problem: SA9's menu is banned away by pass 3 and it is
  fanned out unplanned every time (`plan from X: 40 of 41 nets; SA9 read
  off the board`).

### A second array pair: `make_bench.py`, and the ladder beside its board (2026-09-07)

Every rule above is measured on one bench. `make_bench.py BOARD SRC DST
OUT [--two-layer]` now prepares an article from any board the way the
first one was prepared: the inner copper layers and the zones on them
removed (`--two-layer`: the braid is a two-page router, and the corpus
holds no 2-layer BGA-to-BGA DDR pair), the pair's two-pad nets found,
SRC fanned out for them with the chain's own destination engine call
(`fanout_from_plan.fanout_once`: the production engine, F/B, the
braid's track / clearance / via, foreign parts immovable, no plane
drop; a refused net is left out of the ladder), the project stamped
with the chain's floor (`fix_project_for_output`; a stock 0.2 mm class
graded a clean 0.1 mm fanout as 959 phantom violations), the article
DRC-gated, and the ladder written beside it from the plan's own river
detection (`plan_state` -> detect_buses on taut paths; whole rivers,
largest first, singletons last). `coherent_nets.py` reads `<board
stem>.ladder.txt` beside a bench board when there is one, else the
bench's `k_ladder_coherent.txt`; `chain_k.sh` passes `BASE` through
(`--board=`), `fanout_from_plan.main` passes its base, and the parser's
board warnings go to stderr (they were on the stdout a chain captures as
the net list, and the braid received them as net names).

The second article: the corpus's `zynq_ad9364` (Zynq CLG400, 0.8 mm,
U1 -> DDR3 BGA-96, U2; 46 two-pad nets, 44 with a tooth -- A14 and ODT
refused), six rivers of 11/9/8/6/6/4 nets:

    python3 make_bench.py .../zynq_ad9364.kicad_pcb U1 U2 tmp/bench2/zynq.kicad_pcb --two-layer
    BASE=tmp/bench2/zynq.kicad_pcb DEST=U2 bash chain_k.sh z 11 20 28

Its first hour: K11 0 open 24v 30 s, K20 0 open 32v 51 s, K28 0 open
55v 0 DRC 136 s -- complete, but **17 of 28 in-band** where the first
bench has 28 of 28: the in-band execution is what the leg rules learned
on one board, and that gap is the next thing to probe there. It also
found the braid routing to the config's default hole-to-hole (0.2 mm)
on a board whose project declares 0.25, one drill-to-drill graze at
K28: `setup` now reads the board's `min_hole_to_hole` and tightens to it
(tighten-only, so the first bench at 0.127 routes as before).

### The pose gate: rotations and faces (2026-09-07)

Take4's rotation gate is back (`rotate_board.py`, a whole-board rotation
that walks the s-expression by depth -- a footprint's nested coordinates
are local and ride along -- and self-verifies every pad, segment and
via against the transform), and widened to the FACES: `make_bench.py
--src-side B` / `--dst-side B` put an array on the other face through
the placement writer's mirror (the #714 path), and every part its pads
then collide with (the decoupling caps under a BGA sit on the far face)
goes over with it until the article is pad-clean; `--rotate DEG`
rotates the finished article. A fanned bench can be the input: the
pair's copper is stripped first, so the source is fanned out in its
final pose. `pose_gate.sh BOARD SRC DST K...` builds FF (the control),
BF, FB, BB, R90, R180, R270 into `tmp/gate/`, puts ONE ladder beside
every pose (`LADDER=`, else FF's own) so the K prefixes name the same
nets everywhere, runs the chain on each, and prints the table.

A rotation is an isometry: a grade that changes there is a stage leaning
on the board's axes. A side switch is a different article, so its grade
may differ -- but the chain must complete it, and nothing may assume a
tooth is on F. First reading, the bench (`fb_t2q_fresh`, its own copper)
rotated 90 degrees: K15 0 open 12 vias against 14, K28 0 open 36
against 38, 0 DRC both. The PLAN is invariant -- the same predicted
vias (16, 38) and the same judged cost (40.99, 95.86) in both frames --
and the difference is one lane each (SA9 at K15 2 -> 0, SA4 at K28 4 ->
2), a swimmer the grid router laid cheaper in the rotated frame: the
A* lattice is the stage that leans on the axes, not the braid's rules.

The gate on the origin board (`allwinner_h3_ddr3` unrouted, the human's
passive poses, a fresh source fanout per article, the bench's ladder;
`LADDER=k_ladder_coherent.txt bash pose_gate.sh tmp/gate/h3.kicad_pcb
U1 DU1 15 28`), open / DRC / vias / in-band:

| pose | K15 | K28 |
|------|-----|-----|
| FF (control) | 0 / 0 / 16 / 13 of 15 | 0 / 0 / 38 / 22 of 28 |
| BF, source on the back | 0 / 0 / 21 / 14 | 0 / 0 / 46 / 23 |
| FB, destination on the back | 0 / 0 / 21 / 11 | **1 open** / 0 / 47 / 21 |
| BB, both on the back | 0 / 0 / 24 / 13 | 0 / 0 / 44 / 25 |
| R90, the FF article rotated | 0 / 0 / 14 / 13 | 0 / 0 / 38 / 22 |
| R180 | 0 / 0 / 16 / 13 | 0 / 0 / 38 / 22 |
| R270 | 0 / 0 / 14 / 13 | 0 / 0 / 38 / 22 |

**The same gate on 2026-09-08 evening**, with every rule of that day
(the translation ties, the batched strings as the default, the flow
frame) and the source fanned in the engine's own frame
(`KICAD_FANOUT_FRAME_QUARTER=1`; `tmp/gate/h3_final_table.txt`, and
`h3_tgate_table.txt` for T), open / DRC / vias / segments / in-band:

| pose | K15 | K28 |
|------|-----|-----|
| FF (control) | 0 / 0 / 14 / 417 / 15 of 15 | 0 / 0 / 43 / 1252 / 22 of 28 |
| T, moved by (10.3, -7.7) mm | 0 / 0 / 14 / 417 / 15 | 0 / 0 / 43 / 1252 / 22 |
| MM, turned over | 0 / 0 / 14 / 417 / 15 | 0 / 0 / 43 / 1252 / 22 |
| R90 | 0 / 0 / 14 / 417 / 15 | 0 / 0 / 43 / 1252 / 22 |
| R180 | 0 / 0 / 14 / 417 / 15 | 0 / 0 / 43 / 1252 / 22 |
| R270 | 0 / 0 / 14 / 417 / 15 | 0 / 0 / 43 / 1252 / 22 |
| BF, source on the back | 0 / 0 / 17 / 312 / 14 | 0 / **3 edge** / 42 / 1265 / 27 |
| FB, destination on the back | 0 / 0 / 29 / 926 / 7 | 0 / 0 / 50 / 1085 / 23 |
| BB, both on the back | 0 / 0 / 26 / 470 / 14 | 0 / 0 / 52 / 1160 / 27 |

Every isometry is now the control to the via and the segment -- the
mirror included, which had differed by two segments at K15 before the
day's tie fixes -- and `pose_gate.sh` says so itself: it builds the T
pose (`translate_board.py`), prints the segments, and ends with an
ISOMETRY VERDICT per pose and K against FF on (open, vias, segments),
exit status 1 on a failure. The side switches are other articles and
grade as such. BF's three DRC at K28 were a finding of their own: the
braid's router ran with `board_edge_clearance` at its default 0, which
falls back to the 0.1 mm track clearance, while the project grades the
edge at 0.2 -- a lane along an edge never happens on the bench, and
SCAS's did on that article. The braid now takes the board's own
`min_copper_edge_clearance`, tighten-only, beside the hole-to-hole
floor it already took.

Three readings. Every back-side article completes (the one open, SA0
with the destination on the back at K28, is a rip whose victim SDQ14
lost its own victim one level down -- the re-berth TODO below), so
nothing in the chain assumes a tooth on F; a back-born tooth reaching
a front berth or a dog-bone costs about a via per lane, which is what
the +5..+9 vias are. The rotations are exact at K28 in all four frames
and exact at 180 degrees at K15, and two vias cheaper at 90 and 270:
the octilinear lattice's relation to the lanes is what a quarter turn
changes and a half turn keeps, so the residual is the router's grid,
not a rule. And a FRESH fanout on a rotated board is not invariant at
all (the first run of the gate, `tmp/gate/h3_gate1_*`: K28 38 / 42 /
54 / 38 vias across the four frames, R180 at 28 of 28 in-band) --
`bga_fanout`'s escape order leans on the axes, which is why
`make_bench.py` rotates after the fanout and why the fanout's own
sensitivity is a finding for the engine, not for this chain.

**The board turned over.** Both arrays on the back should grade like
both on the front -- a reflection through the board's plane is an
isometry too -- and the gate's BB pose (24 / 44 vias) is nowhere near
FF (16 / 38). The fresh back-side fanout was one reason (611 tracks and
6 vias for the same balls the front fans out in 502 and 8, four teeth
on the far side) -- FIXED in the engine: `bga_fanout/flip_frame.py`
turns the board over in memory for a part on the back, runs the
pipeline on the part now on F and mirrors the copper back, the way
`rotate_frame.py` handles an angle, so a chip on the back now fans out
as the exact mirror of the same chip on the front (0 of 51 escapes
differ on the origin board and its mirror; `tests/test_fanout_flip_frame.py`
pins it with a change detector). Before that, `mirror_board.py` had
turned the FANNED front article over instead -- every part to the other face through the placement
writer, y mirrored, every layer swapped, self-verified -- and the chain
on that mirror measures its own front call-outs alone: K15 16 vias (=
FF), **K28 50 against 38**. Every literal `F.Cu` in the chain was then
read (`grep`): the schedule seeds its pages symmetrically by tooth layer
and `divers` only feeds a log line; the braid breaks two exact ties to
F (an exit block's shared leg layer on an even split, a leg's layer at
equal cost) and filters back-side required stretches near s1 only; and
the taut paths, the spine and the plan's pad-clear test relaxed against
FRONT copper by name. That last one is fixed -- the layer the majority of
teeth are born on (`braid.bundle_layer_of`), identical on the bench by
construction -- and moved the mirror to 48. What remained was the PLAN:
the front chose 17 berths on its down face where the mirror should
choose 17 on its up face and chose 9 up, 12 down, with the predicted
vias 55 against 37; the mirrored geometry ties every cost exactly, so
the selector's tie-breaks -- the menu's gap order, the face iteration
order, a first-index LIS, a quarter-turn axis, a crossing test that
counts a shared endpoint on one side only -- decided, and none is
mirror-invariant.

**The selector's frame** (`select_moves.PairFrame`, 2026-09-08). Two
answers were measured. Making every tie canonical (invariant keys, an
oriented axis, a symmetric crossing test) did make the selector
symmetric, but the crossing count had been tuned into the plan: the
physical count lost the bench at every K and every weight (K28 38 ->
44..52, K41 82 -> 114), and the tie changes alone turned K41 into a
different draw (100 vias; five single reverts all 100..112). So the
handed selector stays exactly as it is, and runs every pair in the
pair's own canonical frame instead -- the move `flip_frame` makes for
the fanout engine, done at the selector's boundary: `pair_chirality`
reads the sign of the run's BALLS' moment about the line between the
two array centres (+1395 on the front article, -1395 on its mirror,
positive on the bench at every K and on the zynq; the balls, not the
teeth, because the plan's rounds move the teeth and at K15 their sign
flipped at round 1 and mirrored the bench against itself), and a -1
pair has its menus, launches, box and pads mirrored in, the chosen
moves mapped back by identity. Three things the mirror alone did not
give, each found by comparing the two worlds stage by stage: the
menus re-sorted into the generator's own order (`menu_order`, read off
a move's geometry and verified equal to the generated order on every
net of the bench, the origin board and the zynq -- the selector breaks
ties by list order), the layer NAMES swapped (the refinement sorts
slots by name), and a mirror line on the 0.0005 mm lattice. The judge's
`plan_pages` and the source refinement take the same frame. Read off
the pair alone, so a board with three arrays gives every pair its own
frame; a +1 pair never enters the wrapper, so the bench is unchanged
by construction: K15 14 / K28 38 / K41 82, same segment counts.
Measured on the origin board and its mirror: the selector alone
chooses identically through every stage (0 of 15, 0 of 28 differ);
the chain then graded K15 16 = 16, K28 38 against 40, the residual
being the braid's own planner, which judges the plan loop's rounds and
still broke two ties toward F.

**The braid's frame** (`braid.setup`, 2026-09-08). The same move at
the braid's boundary, for the planner (`plan_braid`, the judge) and the
braid alike: `setup` reads the pair's chirality off the same balls and
boxes as the fanout (`pair_chirality_of`), checks it against the one
the plan was made in (`plan['chi']`, written by `braid_plan_of`), and
for a -1 pair turns the board over in memory with the engine's own
`flip_frame.to_front_frame`, mirrors the plan into that frame
(`mirror_plan`: ends, layers, escape directions), and runs everything
unchanged; `plan_braid` swaps the pages and leg layers back and
`write_out` mirrors the copper, the Eco overlay and the refusal report
back. A +1 pair never enters it, so the bench is unchanged to the
segment (K15 14 / 673, K28 38 / 1530, K41 82 / 2051). Two things the
frame alone did not give, each found by comparing the two worlds stage
by stage until the planner agreed on every net:

- the braid's obstacle memo (`_OBS_MEMO`) is keyed on the board FILE,
  and the turned board is, as a file, the same board: it was handed the
  real board's model, on which the mirror's front layer holds 22 discs
  where the turned board's front holds 473, and every taut path was a
  straight chord where the front's bent. The turned copy now carries
  `frame_axis` and the key includes it. (The taut memo is content-keyed
  and was never wrong, only cold: the mirror's first run at K15 took
  61 s to the front's 21, and 21 s warm.)
- the engine's mirror line was the bounds' centre, off the lattice;
  the router's grids are anchored at the origin, so a mirror about it
  mapped the grid onto a grid shifted by a fraction of a cell, and the
  turned board fanned out with 0.2 mm jogs the front did not have (50
  of 266 segments at K15). `mirror_axis` now snaps the line so that
  twice it is a multiple of 0.1 mm, and the bounds are mirrored with
  everything else (the flip-frame test asserts it).

Measured with that: the mirror's plan is the front's to the letter at
every round, its fanout has the same escapes and vias, and the chain
grades **K15 16 = 16 (466 against 464 segments), K28 38 = 38 (1687 =
1687)**. The segments that still differ are the fanout engine's
exact-edge cells: the plan's exit points sit at pitch fractions that
land exactly on the occupancy grid's cell edges, and a plain truncation
lets the last bit of floating-point noise choose the cell. An epsilon
before the truncation made the copper identical in both frames and was
measured and REJECTED: it moves every exact-edge decision on every
board, the bench included (K28 38 -> 36, K41 82 -> 85 and 120 s
slower), because those decisions had been made by the same noise when
the bench was tuned. The bar for the mirror is the same grade, not the
same copper (user decision, 2026-09-08).

**Non-orthogonal rotations.** 30 degrees: K15 complete at 33 vias, 6 of
15 in-band; K28 0 open but 15 DRC, 95 vias, 12 of 28 in-band, 661 s.
The plan's faces are compass directions (`DIRS`), the audit measures a
gap ALONG a face by x or y, and the router's lattice is octilinear, so a
pose off the axes is outside both models today; that it completes at
all is the braid's ladder. 45 degrees is the octilinear-friendly angle:
K15 complete at 20 vias, 11 of 15 in-band; at K28 the plan stage itself
fails (`endpoints`: a realized tooth's free stub end not found on the
rotated copper), so the pose is refused before the braid.
