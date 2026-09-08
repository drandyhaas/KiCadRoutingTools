# The seven boundary criteria, worked on a real board

The subject is a **2-layer, 21-part USB debug adapter**: a micro-USB receptacle,
a USB-to-UART bridge in an SSOP-20, a 3-pad SOT-89 regulator, a crystal with two
load caps, an auto-reset transistor pair, two headers and a handful of 0402s.
It is a tracked fixture in this repository, and the numbers below come from the
instruments named beside them — every one is re-derived by a test, so a figure
that stops being true fails the suite rather than ageing quietly in prose.

The board it is taken from PASSED every gate the toolchain had. That is the
point of the example: the criteria exist because a review can follow the
blind-first mandate exactly and still hand on a layout a competent engineer
would send back.

---

## 1. Pair and bus length — **A FINDING**

```
board_context.py <board> --json   ->   pin_order.rows[].span_mm
```

| pair | scope | span |
|---|---|---|
| bridge ↔ receptacle | the differential pair alone | **8.10 mm** |
| bridge ↔ receptacle | the whole interface (3 nets) | **12.70 mm** |
| header ↔ regulator | interface (3 nets) | 13.77 mm |
| bulk cap ↔ regulator | interface (2 nets) | 2.56 mm |

The two parts are 7.2 × 5.3 mm and 9 × 7 mm. Placed edge to edge their pads
could face each other across roughly 2–3 mm, so the pair is running about **3×
the shortest length these bodies allow** — well past the ratio worth explaining.
On a 2-layer board that length is also where the return path has to come from.

**Verdict: explain or move.** The two parts are on opposite sides of the board
with a header between them.

## 2. Pin-order agreement — **A FINDING**

```
board_context.py <board> --json   ->   pin_order.rows[].verdict
```

The differential pair reads **CROSSED**, one inversion: the receptacle's D- pad
is north of its D+, and the bridge's D+ is north of its D-. Parity is invariant
under rotation — only a mirror or a hop flips it — so **no pose of either part
fixes this**. It costs a via per net or back-side copper.

The whole interface reads CROSSED with three inversions, and that is why the
pair-scoped row exists: asked as one question, the interface answer would have
blended the pair's parity into a bus statistic and hidden the fact that decided
this board's only open clause. It was found after routing.

**Say which nets.** Here: the two pair nets, and nothing else.

## 3. Cluster distance — **A FINDING, and the one no rule could reach**

The regulator has **three pads**. The decap tether election requires four, so it
can never be a tether target at any radius, and its own bulk caps were graded
against a USB socket and the bridge IC instead — 2.03 mm and 1.83 mm to the
wrong partners. Nothing in the toolchain measured the thing the spec actually
said.

That is what `proximity` clauses are for. Declared, the same board reports the
crystal's far leg at **3.14 mm** from the pin it loads against a 2.00 mm limit,
and the bulk caps at 1.12 mm and 0.29 mm from the regulator — measured pad edge
to pad edge, against the parts the spec names.

The input-cap-on-the-input-side half is **not measured by anything**. Say it in
words or it is not said.

## 4. Facing — reads clean

```
board_context.py <board> --json   ->   parts[].pads_by_face, parts[].partners
```

The bridge's west row carries the receptacle's nets and faces it; the crystal's
pads face the pins they load. Nothing to report — which is what a criterion that
passes looks like, and it still gets written down.

## 5. Seams — **A FINDING**

```
render_placement.py <board> --review-sheet <PATH> --json-out <PATH>.json --quiet
                                  ->   checklist.b_body_seam
```

**-0.133 mm, bridge ↔ crystal**, sourced `silk` / `fab`. Negative is an overlap:
the SSOP body end intrudes into the crystal can. It is below any hand-assembly
threshold because it is not a gap at all.

Two things this number teaches. It was found on this board only by an
adversarial reviewer who wrote their own geometry — no instrument in the chain
produced a seam until one was built, because the overlap report gave a depth
only for pairs that ALREADY overlap, so a board one micron from collision
reported nothing. And the sources matter: a seam between two silk markings is a
much weaker claim than one between two drawn outlines. Quote the rung.

## 6. Density and balance — reads clean, with a caveat

```
check_pockets.py <board> --bin 5 --json <PATH>
```

- emptiest contiguous region **24.0 mm²**, 8.7 % of the board in cold windows
- centroid offset **3.0 %** of span (the part-count control reads 6.9 %)

A 3 % centroid offset on a board this size is not a finding. **The weight is
COURTYARD AREA, not pad area** — the tool says so in its own output, and on one
tracked board the two readings differ by 5× — so quote it as the courtyard-area
centroid, not as "the centroid".

## 7. The human question

> Would a competent engineer accept this layout without changes? If not, the
> first thing they would move is ___.

For this board: **no**. The first thing to move is the bridge IC, to put its
pair pads facing the receptacle — which addresses criteria 1 and 2 together and
is the only change that can fix the crossing without a via.

---

## What the example is for

FOUR of the seven fire on a board that passed every automated gate -- pair
length, pin order, cluster distance and the seam -- and they are the ones a
human notices in the first five seconds. That gap is the reason the criteria
are written down rather than left to judgement: the judgement was there, and it
had nothing to measure.

Two of the numbers here are also a warning about prose. The seam was recorded
in a run journal as 0.183 mm and the pair as 9.3 mm; re-measured with the
shipped instruments they are -0.133 mm and 8.10 mm. Neither journal figure was
wrong when it was written — they were measured differently, by hand, on a
different lap — but neither could be reproduced from what was written down.
Every number above names the instrument and is pinned by a test for exactly
that reason.
