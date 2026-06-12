# Placement Optimization for Routability

Research notes and design proposal for improving component placement with the
goal of improving autorouter success. Written June 2026, based on a survey of
~40 years of placement literature plus the current commercial and academic
landscape. All sources are linked in the [references](#references-and-further-reading).

**Short version:** from-scratch autoplacement has failed for a well-understood
reason that has nothing to do with optimization power, while perturbative
refinement of a human (or AI) seed placement is precisely the formulation that
works — and most of the machinery it needs already exists in this project
(`placement/engine.py`, `rust_placer`, and the router itself).

## Why from-scratch autoplacement fails

The killer isn't the algorithm — it's **constraint capture**. The constraints
that govern placement live in mechanical CAD, datasheets, and the engineer's
head, not in the board file:

- enclosure fit, button/connector positions, mounting holes (MCAD)
- thermal spreading and rework access
- EMI zoning: noisy switchers vs sensitive analog
- datasheet intent: switching-regulator hot-loop area must be minimized as the
  *primary* objective for those 4–5 parts (TI SNVA021, ADI AN-1119 both say
  "copy the manufacturer's layout") — a wirelength-minimizing placer has no
  concept of "minimize this loop's enclosed area"
- assembly direction, pick-and-place orientation conventions, test-point access
- plain designer intent (even schematic ordering of parallel parts silently
  communicates intended board order)

Altium's own product manager conceded that routing constraints are easy to
model but placement constraints "can be driven by mechanical considerations…
ergonomic issues… heat dissipation" and that modeling them "introduces
significant overheads" — and Altium
[deleted their autoplacer at v18](https://www.altium.com/documentation/altium-designer/autoplacer-cmd-runautoplacerrunautoplacer-ad?version=17.1)
(2017) after decades of "produces garbage" feedback. The constraint-entry
death spiral: by the time you've configured the rooms/keepouts/rules the tool
needs to do well, you've already placed the board by hand.

The structural differences from ASIC placement (which *is* solved) compound
this:

- **~50–1,000 wildly heterogeneous parts** (0402 next to a BGA) vs millions of
  uniform standard cells in rows; the smoothed density fields that modern
  analytical placers depend on are only meaningful at large N
- **arbitrary rotations and two board sides** — standard-cell placers optimize
  (x, y) only; orientation is a second-class patch even in research placers
- **2–8 routing layers vs 10+ metal layers**, so placement mistakes can't be
  routed around — which specific pin pairs cross matters more than aggregate
  congestion
- **no amortization**: each board's constraint set is unique, unlike a cell
  library reused across millions of instances

The flagship academic result, **Cypress** (Cornell/NVIDIA, ISPD 2025 best
paper), states this directly: stock VLSI analytical placers like RePlAce
"fail to find a routable placement for multilayer PCB designs with components
of diverse sizes."

## What the classical toolbox offers

Four families dominate the literature; their PCB fit differs sharply:

| Family | Canonical work | PCB fit |
|---|---|---|
| Simulated annealing | TimberWolf (1985), VPR (1997), SA-PCB (2019) | **Excellent.** Derivative-free, so rotations, side flips, polygon overlap, and arbitrary constraints are just move-legality checks. Abandoned in VLSI for *scalability*, never quality — irrelevant at N ≈ 100. |
| Force-directed | Quinn & Breuer 1979 (written *for* PCBs), Kraftwerk | Good for fast initial seeds; collapses clusters into overlap without density machinery that assumes large N. |
| Min-cut partitioning | Fiduccia–Mattheyses 1982, Capo | Useful as a clustering prior (functional blocks → regions; top/bottom side assignment = one cut); not a placer for continuous coordinates. |
| Analytical/gradient | GORDIAN → ePlace → DREAMPlace → Cypress | Documented to fail on PCBs without Cypress-level surgery (per-side density maps, orientation-aware wirelength, GPU, Bayesian hyperparameter search). Wrong tool at this scale. |

TimberWolf's move set — single-part **displace** within a window that shrinks
with temperature, **pairwise exchange**, **orientation change** — is still the
canon 40 years later, and is exactly the "nudge, rotate, swap" set proposed
below. VPR's adaptive schedule is the standard tuning recipe: initial
temperature from the cost standard deviation over random moves, cooling rate
chosen to hold move-acceptance near **0.44** (Lam's result), moves per
temperature ∝ N^(4/3), exit when T < 0.005·cost/N_nets. VPR also ships the
exact UX precedent: load an existing placement file and run
`--place_quench_only` — a zero-temperature quench for when "the initial
placement is already good."

## Why perturbative refinement is the validated niche

This is the strongest finding across all research threads:

- **Detailed placement is an entire ASIC sub-discipline doing exactly this.**
  After global placement and legalization, placers run local-move refinement
  on a legal placement. The canonical move set (FastDP, ICCAD 2005):
  - *global swap*: compute a cell's **optimal region** — the median of its
    nets' bounding boxes (Goto 1981), an O(pins) computation that answers
    "where would this part ideally sit?" — then swap it with a cell or gap
    near that region if Δcost < 0
  - *local reordering*: exhaustively permute k ≈ 3 consecutive cells in a
    sliding window
  - *single-row clustering / Abacus-style DP*: with ordering fixed, slide
    cells within a row to optimal offsets by dynamic programming — directly
    applicable to a line of passives along an IC edge
  - *mirroring/flipping*: orientation-only pass; in macro placement, flipping
    alone is worth an average **7.9% HPWL** (DAS-MP 2025)

  Typical total recovery from local refinement: **5–10% wirelength**, with
  each move family worth roughly 1–8%.
- **Refine-from-seed empirically beats from-scratch** at PCB-like scale
  (tens–hundreds of rotatable blocks):
  - MaskRegulate (NeurIPS 2024) recast RL placement as *refining* a finished
    placement one macro per step, with reward = 0.7·wirelength +
    0.3·regularity; vs from-scratch MaskPlace it gained 17.1% routed
    wirelength and **73%/39% horizontal/vertical congestion-overflow
    reduction**.
  - WireMask-BBO (NeurIPS 2023) fine-tunes any existing placement by
    black-box optimization — up to 50% HPWL improvement from a seed.
  - The Kahng/Cheng re-evaluations of Google's Nature 2021 RL chip placement
    found that properly tuned parallel SA with moves {move, swap, shift,
    mirror, shuffle} **beat the RL placer at lower runtime**, and that the RL
    results depended heavily on the quality of the (commercial) *seed*
    placement — backhanded but strong evidence for "good seed + local
    optimization."
- **A human seed sidesteps constraint capture entirely.** Bound each part's
  displacement to a small radius of its original position and keep clusters
  intact, and the optimizer inherits every unmodeled constraint for free —
  the human already satisfied them. This is structurally why the *accepted*
  placement automations (Cadence Place Replicate, decap fanout under BGAs)
  all operate inside human-specified intent. Cypress itself scopes to
  "noncritical components" with critical parts fixed (and supports partial
  locks: position fixed, rotation free).
- **Reviewability fits the trust culture.** Practitioners accept co-creation
  tools and reject full automation ("never trust the autorouter"). Output
  that is visibly "your placement, nudged" can be reviewed in minutes.

## What to optimize

The existing scorer in `placement/engine.py` / `rust_placer` — **airwire
length + crossing penalty** — is already the right objective family.

**Crossings are the key PCB-specific signal.** Cypress's central technical
argument: with only 2–6 layers and free-angle routing, *which specific pin
pairs cross* matters more than aggregate bin congestion. They show two
placements with identical RUDY congestion scores where one has a routing
conflict and one doesn't. Their cost: decompose each net into source–sink pin
pairs (star model); for each same-layer segment pair compute the intersection
parameters (t, u) in closed form; smooth the binary "crosses" indicator with a
bell function so it's differentiable: `NC = Σ B(t−0.5)·B(u−0.5)`. Our discrete
crossing counter is the non-differentiable version of the same quantity —
fine, since SA doesn't need gradients. The theory behind crossing counts is
topological routing (SURF, DAC 1991): zero airwire crossings on a layer means
a planar ratsnest, i.e. routable on that layer in the topological sense; every
crossing costs a via pair or a detour.

For reference, RUDY (the standard VLSI congestion proxy) smears each net's
expected wire area uniformly over its bounding box —
`d_n = (HPWL_n · pitch) / (w_n · h_n)` accumulated onto a grid. It's O(1) to
update incrementally and worth keeping as a *tiebreaker*, but not as the
primary objective, for the reason above.

Two additions the literature argues for:

1. **A spreading/whitespace term.** An open-source SA placer benchmarked
   against Quilter found the gap wasn't wirelength — Quilter won
   route-completion (99.4% vs 87–90%) by *spreading parts apart for fanout
   room*. Pure wirelength minimization packs too tight and strangles the
   router. The author's conclusion: "a good placement is one the router can
   finish, which is not something you can compute directly from the
   placement." A pad-density map or per-component halo (AutoDMP's macro-halo
   trick) fixes this. Pin density also captures the real bottleneck at
   BGA/connector escapes.
2. **Trial-routing as the arbiter.** The fidelity hierarchy used by everything
   serious — TimberWolf stage 3 (1985) accepted placement moves only if they
   reduced global-route track counts; Cypress *evaluates* with FreeRouting
   (routability = routed pin pairs / total pin pairs); Quilter scores complete
   candidates by route completion + DRC. The pattern: cheap proxies per-move
   (microseconds), real routing per-candidate (seconds–minutes) to rank
   finalists. This project's structural advantage: the literature validates
   with FreeRouting; we have a faster router in-house, so "fraction of pin
   pairs routed" becomes a measurable, optimizable number.

### Metric cheat-sheet

| Metric | Cost per move | Routability signal at PCB scale | Verdict |
|---|---|---|---|
| HPWL / airwire length | O(1) incremental | Necessary, far from sufficient (exact only for 2–3 pin nets; congestion-blind) | Always include |
| Airwire pin-pair crossings | O(moved part's segments) | Best single PCB proxy (Cypress, NS-Place); grounded in planarity theory | Primary second objective |
| Pad/pin density map | ~free | Captures BGA/connector escape limits and fanout room | Cheap secondary term |
| RUDY congestion map | O(1) incremental rect update | Misses pin-pair conflicts on few-layer boards (Cypress Fig. 3) | Tiebreaker only |
| Steiner trees (FLUTE) | ~10× HPWL | Marginal over HPWL — PCB nets are mostly 2-pin | Skip |
| ML routability predictor | ms inference | Beats proxies (MIT thesis: NN trained on 75k *routed* placements) but needs per-board-family training data | Only if labels are cheap |
| Actual routing | seconds–minutes per candidate | Ground truth | Rank finalists, not per-move |

## Proposed shape of a `place_optimize.py`

Seeded SA — or a plain greedy quench first (VPR's `--place_quench_only`
precedent), which is simpler and may capture most of the value:

- **Moves** (the TimberWolf/Kahng-Cheng canon, adapted to PCB):
  - *nudge*: translate within a window that shrinks as temperature drops;
    optionally bias toward the part's optimal region (median of connected
    nets' bounding boxes — O(pins) to compute)
  - *rotate*: 0/90/180/270
  - *swap*: same-footprint pairs (footprint-identical R/C swaps are free
    wins; mixed-size swaps are usually illegal anyway, so restrict by
    footprint compatibility)
  - *side flip* (optional; mirrored courtyard): treated as a first-class move
    in recent PCB literature
  - *rigid-group moves*: an IC plus its decoupling caps moves/rotates as one
    super-component
- **Constraints by construction, not penalty** (the discipline from analog-IC
  placement and Synopsys ICC relative-placement groups: encode constraints
  into the move generator so every perturbation is feasible, rather than
  penalizing violations):
  - respect `locked` flags (already parsed by `placement/parser.py`)
  - `--max-displacement` radius from the seed position, so output reads as
    "your placement, nudged" and inherited human constraints survive
  - courtyard non-overlap via the existing rect machinery
  - decaps tethered to their IC (group membership or hard radius)
  - rotation disabled per-class where assembly conventions matter
- **Cost**: Δ(crossings)·penalty + Δ(airwire length) + density/halo term —
  all incrementally updatable (only airwires touching the moved part's nets
  change). Mostly already in `rust_placer`; extending the scorer to evaluate
  a *perturbation* rather than a candidate position is a modest change, and
  it's a separate crate from the router.
- **Schedule**: VPR-style adaptive — T_init from cost σ over random moves,
  hold acceptance near 0.44, moves/temperature ∝ N^(4/3) (trivial at N ≈
  100), exit at T < 0.005·cost/N_nets. Or skip all of it and quench.
- **Validation loop**: after optimization, run `route.py` on before/after and
  report completion % and failed-net count. The Cypress benchmark suite (10
  open boards, 41–476 components, with KiCad converters) gives ready-made
  test cases beyond our own boards.

### Role of AI

Two distinct roles, kept separate:

- *Seed generation*: the existing `place_components_initially` (greedy
  constructive: descending pin count, connectivity-weighted centroid target,
  4 rotations, crossings+length scoring) is plausible for simple boards; keep
  `place.py` as a seed generator and fallback, not the headline feature.
- *Constraint extraction* — the more interesting angle: the project already
  has skills doing per-component datasheet lookup (`analyze-power-nets`,
  `identify-diff-pairs`). The same pattern could emit a placement-constraint
  file: which parts are decaps and whose IC they tether to, which clusters
  are switcher hot-loops that must move as rigid groups or not at all, which
  connectors should be locked. That directly attacks the constraint-capture
  problem that killed every previous autoplacer, and feeds the optimizer
  exactly the lock/tether/group inputs it needs. (Quilter's "circuit
  comprehension" — auto-detect bypass caps, diff pairs, power nets, then have
  the user verify — is the commercial version of this idea.)

## What to avoid

- **RL**: the startups (Quilter, DeepPCB, Flux) bet on it, but the best
  peer-reviewed result (Cypress) is non-RL; Quilter takes hours per board
  with hard size limits (<1,000 pins, ~100 components, 2.6 h for a
  176-component board as independently tested in 2024); and the Google
  chip-RL saga showed well-tuned SA matches or beats it given equal compute.
  We also have no training data.
- **Analytical/gradient placement** (DREAMPlace-style): needs Cypress-level
  engineering plus GPU plus multi-objective Bayesian hyperparameter search to
  work on PCBs — itself an admission that gradient methods are
  tuning-fragile at this scale. Wrong tool for N ≈ 100.
- **From-scratch placement of complex boards**: see constraint capture above.
- One framing note from the practitioner forums: the word "autoplacer" is
  culturally radioactive — "placement optimizer" that visibly preserves the
  user's layout is the framing people accept.

## First experiment

Cheap and decisive: take a hand-placed board, run a greedy quench with just
nudge+rotate+swap moves against the existing crossings+length scorer, and
measure route completion before/after with `route.py`. That one number tells
us whether the whole direction is worth building out. If the proxy improves
but route completion doesn't, add the spreading term before anything else —
that's the documented failure mode of wirelength-driven placement.

## Experiment results (June 2026)

Implemented as `place_optimize.py` + `placement/quench.py`: greedy quench
(nudge within `--max-displacement` of the seed, 90° rotations with correct
pad-angle rewriting, same-footprint swaps), cost = `length_weight`·airwire
length + `crossing_penalty`·crossings + pin-count-scaled halo + soft edge
margin. Test board: `interf_u` (25 parts, PGA120 + bus connectors, 2 layers),
pipeline `route_planes` → `bga_fanout U9` → `route.py` with the
`tests/test_interf_u.py` arguments. Router iterations ≈ effort; vias and
completion are the quality metrics.

| placement | single-ended | multipoint pads | vias | router iterations |
|---|---|---|---|---|
| hand (KiCad demo) | 108/108 | 80/80 | 136 | 2.3 M |
| hand + quench, default weights | 106/108 (2 fail) | 80/80 | 150 | 3.1 M |
| hand + quench, strong halo | 100% of attempted | — | 157 | 6.7 M |
| hand + quench, crossing-focused¹ | 108/108 | 80/80 | 135 | 2.6 M |
| `place.py` from-scratch seed | 106/108 (2 fail) | 65/77 (12 fail) | 202 | 1 141 M |
| from-scratch seed + quench¹ | 108/108 | 71/80 (9 fail) | 240 | 364 M |

¹ `--length-weight 0.3 --crossing-penalty 30 --halo-weight 10 --halo-coef 0.5 --edge-halo 3`

**Conclusions:**

1. **The hand placement is dramatically better than from-scratch constructive
   placement** — 500× less router effort, no failures. The constraint-capture
   story is real even on a 25-part board: the human's bus-flow arrangement
   (BUS1 → buffers → PGA → RAM) is what makes it routable, and its
   "suboptimal" wirelength is buying that structure.
2. **Quenching an already-good seed is neutral at best.** Proxy improvements
   (crossings −13%, length −12%) did not translate: default weights *caused*
   2 failures, strong-halo variants tripled router effort. The
   crossing-focused parameter set merely matched the hand placement. On a
   board with no completion headroom there is nothing for the proxy to win,
   and chasing it perturbs structure the proxies can't see.
3. **Quenching a mediocre seed genuinely helps**: from-scratch + quench fixed
   both single-ended failures, cut multipoint failures 12 → 9, and reduced
   router effort 3× (1 141 M → 364 M iterations). Refinement works exactly
   where the literature says it does — when there is headroom.
4. **Proxy–routability correlation is weak**, confirming the MIT-thesis
   finding: the variant with the *best* crossing reduction (length weight 0)
   failed 2 nets. Any production version of this tool should rank candidate
   placements by an actual trial route (our router does this board in ~1–2 s
   of routing time), not by the proxy alone.

**Practical upshot:** ship the quench as a *repair* tool for rough/generated
placements (`place.py` output, imported or auto-generated layouts), not as a
polish pass on careful hand placements. The next-step experiments are (a)
router-in-the-loop candidate ranking, since single routes are cheap, and
(b) a proxy that models *escape/fanout room* around high-pin-count parts
explicitly rather than via the generic halo.

### Second board: kit-dev-coldfire-xilinx_5213 (160 parts, 4 layers)

Setup: `tests/test_kit_route.py` signal-stage arguments; connectors/headers
locked via `--lock`; plane-routed nets excluded from airwire scoring via
`--ignore-nets GND +3.3V` (both options added for this experiment). All
quench runs use the crossing-focused weights (`--length-weight 0.3
--crossing-penalty 30`).

| variant | single-ended | multipoint | vias | router iterations | route time |
|---|---|---|---|---|---|
| hand (KiCad demo) | 204/207 (3 fail) | 227/227 | 346 | 122 M | 45 s |
| quench, 3 mm cap, modest halo | 205/207 (2 fail) | 235/236 (1 fail) | 346 | **24.6 M** | 13 s |
| quench, 5 mm cap, modest halo | 204/207 (3 fail) | 223/227 (4 fail) | 331 | 22.4 M | 10 s |
| quench, 10 mm cap, strong halo | **192/207 (15 fail)** | 222/226 (4 fail) | 334 | 235 M | 97 s |

**Dose–response is the story.** At a 3 mm displacement cap the quench matches
the hand placement's completion while cutting router effort **5×** (122 M →
24.6 M iterations) — on this denser board the crossing reduction (−11%)
translates into real router savings, unlike on interf_u. At 5 mm it's
marginally worse on completion but keeps the effort win. At 10 mm with strong
halos, 149 of 150 movable parts moved and the placement collapsed: 12 of the
15 new failures were `/xilinx/XIL_D*` — the quench had destroyed the data-bus
corridor between the Xilinx and the MCU, exactly the macro structure the
crossing/length/halo proxies cannot see. (Failed-net identities vary between
runs near the noise floor; the iteration counts are the robust signal.)

Refined conclusions:

- `--max-displacement` is the dominant safety knob: small caps keep the
  human's macro structure (bus corridors, cluster geometry) intact by
  construction, which is the entire value of seeding from a human placement.
  ~3 mm was the sweet spot on both boards tested.
- The big halos backfire on dense boards: with `--halo-coef 0.5` the
  144-pin Xilinx demands a 6.5 mm halo that a dense board cannot satisfy, so
  the halo gradient dominates everything and scatters the layout. Modest
  halos (`--halo-coef 0.15`) only fire where parts are genuinely cramped.
- Lock connectors (`--lock`) and exclude plane-routed power nets
  (`--ignore-nets`) — both matter for honest objectives on real boards.

## References and further reading

### PCB-specific placement research

- [Cypress: VLSI-Inspired PCB Placement with GPU Acceleration](https://www.csl.cornell.edu/~zhiruz/pdfs/cypress-ispd2025.pdf) (Zhang et al., ISPD 2025 best paper) — net-crossing objective, per-side density maps, lock-critical-parts scoping, open benchmarks with KiCad converters ([code](https://github.com/NVlabs/Cypress), [ACM](https://dl.acm.org/doi/10.1145/3698364.3705346))
- [SA-PCB](https://github.com/The-OpenROAD-Project/SA-PCB) (UCSD/OpenROAD, ~2019) — open-source SA PCB placer: polygon-exact overlap via Boost geometry, 90°/45°/free rotation, TimberWolf cooling
- [NS-Place: Net Separation-Oriented PCB Placement via Margin Maximization](https://arxiv.org/pdf/2210.14259) (2022) — SVM-like net-hull separation; −25% routed WL, −50% vias, −79% DRVs vs wirelength-minimal placement
- [Quinn & Breuer, "A force directed component placement procedure for printed circuit boards"](https://ieeexplore.ieee.org/document/1084652/) (IEEE TCAS 1979) — the founding force-directed placement paper, written for PCBs
- [Abboud, Grötschel & Koch, "Mathematical methods for physical layout of printed circuit boards"](https://link.springer.com/article/10.1007/s00291-007-0080-9) (OR Spectrum 2008) — survey of exact/heuristic PCB layout models
- [Reade, MIT M.Eng thesis](https://dspace.mit.edu/handle/1721.1/129238) (2020) — NN routability predictor trained on 75k placements labeled by actually routing them; "wirelength and crossings only correlate roughly with routability"
- [Crocker, MIT M.Eng thesis: Physically Constrained PCB Placement with Deep RL](https://dspace.mit.edu/handle/1721.1/139247) (2021)
- [RL_PCB](https://github.com/LukeVassallo/RL_PCB) (Vassallo, DATE 2024; [thesis](https://www.lukevassallo.com/wp-content/uploads/2023/09/automated_pcb_component_placement_using_rl_msc_thesis_v2_1_lv.pdf)) — RL that learns *local placement moves*; closest published work to a learned perturbative refiner
- [DAC 2024 LBR: Modern Automatic PCB Placement with Complex Constraints](https://dl.acm.org/doi/10.1145/3649329.3663495) and an [analytical fine-tuning follow-up](https://www.sciencedirect.com/science/article/abs/pii/S016792602500224X) (2025) — SA-based pad-alignment fine-tuning that improves local routing space
- [Sutherland & Oestreicher, "How Big Should a Printed Circuit Board Be?"](https://ieeexplore.ieee.org/document/1672352/) (IEEE Trans. Computers 1973) — classic cut-line wiring-capacity bound

### Classical placement (ASIC/FPGA) — the algorithm toolbox

- [TimberWolf](https://janders.eecg.utoronto.ca/1387_2015/readings/timberwolf.pdf) (Sechen & Sangiovanni-Vincentelli, JSSC 1985) — origin of the displace/swap/rotate SA move set, shrinking displacement window, and routing-in-the-loop refinement stage
- [VPR](https://www.eecg.toronto.edu/~vaughn/papers/fpl97.pdf) (Betz & Rose, FPL 1997; [docs](https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/)) — adaptive annealing schedule (target acceptance ≈ 0.44); `--place_quench_only` is the exact "polish an existing placement" UX
- [Lam & Delosme cooling schedule](https://www.researchgate.net/publication/221433579_An_Efficient_Simple_Cooling_Schedule_for_Simulated_Annealing) (1988) — source of the 0.44 acceptance-rate target
- [FastDP](https://home.engineering.iastate.edu/~cnchu/pubs/c30.pdf) (Pan, Viswanathan & Chu, ICCAD 2005) — canonical detailed-placement move set: global swap, vertical swap, local reordering, single-segment clustering
- [Abacus legalization](https://www.semanticscholar.org/paper/Abacus:-fast-legalization-of-standard-cell-circuits-Spindler-Schlichtmann/b7c0656875460a88616342fa9ab55da9496bd22f) (ISPD 2008) — minimal-displacement legalization via per-row dynamic programming
- [Mongrel optimal interleaving](https://users.ece.utexas.edu/~dpan/EE382V_PDA/papers/iccad00_mongrel.pdf) (ICCAD 2000) — DP window reordering
- [ABCDPlace](https://yibolin.com/publications/papers/ABCDPLACE_TCAD2020_Lin.pdf) (TCAD 2020) — batched/parallel detailed placement (independent-set matching, global swap, reordering)
- [GORDIAN](https://janders.eecg.utoronto.ca/1387/readings/gordian.pdf) (ICCAD 1988) — quadratic/analytical placement ancestor
- [ePlace](https://cseweb.ucsd.edu/~jlu/papers/eplace-todaes14/paper.pdf) (TODAES 2015) and [DREAMPlace](https://research.nvidia.com/sites/default/files/pubs/2019-06_DREAMPlace:-Deep-Learning/54_1_Lin_DREAMPLACE.pdf) (DAC 2019) — modern electrostatic/GPU analytical placement (what Cypress builds on; overkill at PCB scale)
- [DAS-MP dataflow-aware flipping](https://arxiv.org/html/2505.16445) (2025) — orientation moves alone worth avg 7.9% HPWL in macro placement
- [Markov, Hu & Kim, "Progress and Challenges in VLSI Placement Research"](https://users.soe.ucsc.edu/~pang/200/f18/papers/2018/ProCha.pdf) (Proc. IEEE 2015) — the field survey; documents that SA was abandoned for scalability, not quality
- [Synopsys ICC relative-placement groups](http://s3-us-west-2.amazonaws.com/valpont/uploads/20151120031409/icc_study_notes.pdf) and [analog symmetry-island B*-trees](https://dl.acm.org/doi/abs/10.1109/TCAD.2009.2017433) — constraints-by-construction precedents (super-cells, feasibility guaranteed by the representation)

### Routability metrics

- [RUDY](https://past.date-conference.com/proceedings-archive/2007/DATE07/PDFFILES/08.7_1.PDF) (Spindler & Johannes, DATE 2007) — standard VLSI congestion proxy, `d_n = (HPWL_n·pitch)/(w_n·h_n)` per net bounding box; see Cypress §4.2.1 for why it underperforms on few-layer PCBs
- [RISA net weighting](https://link.springer.com/chapter/10.1007/0-387-48550-3_2) (Cheng, ICCAD 1994) — pin-count-aware HPWL weighting for congestion
- [FLUTE](https://home.engineering.iastate.edu/~cnchu/pubs/j29.pdf) (Chu & Wong, TCAD 2008) — fast Steiner-tree wirelength (marginal gain over HPWL for mostly-2-pin PCB nets)
- [SimPLR](https://web.eecs.umich.edu/~imarkov/pubs/conf/iccad11-simplr.pdf) (ICCAD 2011) — global-router-in-the-placement-loop
- [RouteNet](https://zhiyaoxie.com/files/18_RouteNet.pdf) (ICCAD 2018) — ML DRC-hotspot prediction from placement features
- [SURF rubber-band topological routing](https://dl.acm.org/doi/pdf/10.1145/127601.127622) (DAC 1991) — the theoretical grounding for crossing counts: zero crossings = planar = single-layer routable

### Refinement-from-seed evidence

- [MaskRegulate](https://arxiv.org/html/2412.07167) (NeurIPS 2024) — "RL as macro regulator, not placer": refining a finished placement beats from-scratch; 73% congestion-overflow reduction
- [WireMask-BBO](https://openreview.net/forum?id=hoyL1Ypjoo) (NeurIPS 2023; [code](https://github.com/lamda-bbo/WireMask-BBO)) — black-box fine-tuning of any existing placement, up to 50% HPWL improvement
- [Kahng/Cheng re-evaluation of Google RL placement](https://arxiv.org/pdf/2302.11014) (ISPD 2023), [Markov, "The False Dawn"](https://arxiv.org/html/2306.09633v10), and [CACM coverage](https://cacm.acm.org/research/reevaluating-googles-reinforcement-learning-for-ic-macro-placement/) — well-tuned SA matches/beats RL; RL gains traced largely to seed quality. Original: [Mirhoseini et al., Nature 2021](https://www.nature.com/articles/s41586-021-03544-w); rebuttal: [That Chip Has Sailed](https://arxiv.org/abs/2411.10053)

### Commercial landscape and practitioner context

- [Quilter](https://www.quilter.ai/) ([technology](https://www.quilter.ai/product/technology)) — RL placement+routing with router/DRC/physics scoring in the loop; hours per board, hard size limits as of 2024
- [DeepPCB / InstaDeep](https://deeppcb.ai/) ([placement launch, 2025](https://deeppcb.ai/2025/04/15/ai-powered-pcb-placement-by-deeppcb/)) — RL self-play placement and routing
- [Cadence Allegro X AI](https://www.cadence.com/en_US/home/company/newsroom/press-releases/pr/2023/cadence-introduces-allegro-x-ai-accelerating-pcb-design-with.html) (2023) — MCTS-based generative placement
- [Altium Autoplacer docs (last version, 17.1)](https://www.altium.com/documentation/altium-designer/autoplacer-cmd-runautoplacerrunautoplacer-ad?version=17.1) — deprecated at v18
- [tinycomputers.io: What a commercial PCB placer does that my open-source one can't](https://tinycomputers.io/posts/what-a-commercial-pcb-placer-does-that-my-open-source-one-cant.html) — open-source SA vs Quilter; spreading for fanout room beats wirelength minimization
- [edaboard: why autoplacement doesn't work in Altium](https://www.edaboard.com/threads/why-autoplacement-dont-work-in-altium-designer.354093/) — practitioner consensus on constraint capture
- [Engineer Live: The future of PCB design automation](https://www.engineerlive.com/content/future-pcb-design-automation) — Altium PM on why placement constraints can't be modeled in ECAD
- [TI SNVA021](https://www.ti.com/lit/pdf/snva021) and [ADI AN-1119](https://www.analog.com/en/resources/app-notes/an-1119.html) — switching-regulator layout intent that lives in datasheets, not netlists
- [HN: tscircuit autorouter discussion](https://news.ycombinator.com/item?id=43499992) and [JITX discussion](https://news.ycombinator.com/item?id=39771983) — autorouter/autoplacer trust culture
- [Cypress benchmark suite](https://github.com/NVlabs/Cypress) — the only open PCB placement benchmark set (10 boards, 41–476 components)
