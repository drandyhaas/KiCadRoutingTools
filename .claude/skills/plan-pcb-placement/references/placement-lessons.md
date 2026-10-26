# Placement lessons: what human boards do, what a routed board shows, and what to move

Read this when you place a board or review a placement: before you write the groups,
and when you look at a placed or routed candidate.

The lessons come from two sources:

- **Humans:** a survey of the 437 human-placed, human-routed boards in stress sets 1-30
  (230 two-layer, 207 with four or more layers). Each rule gives what those boards do,
  usually with the share a random arrangement would give, so you can tell a habit from
  chance.
- **Measured on a route:** a placement change that a quick route or a full route graded
  on one board. These are the cases where moving parts changed the routing.

**Notation.** Numbers are `median [p25 p75]`, with the number of instances and of
boards. Distances use these terms:

- **pad gap:** pad edge to pad edge.
- **courtyard gap:** courtyard edge to courtyard edge; 0 means touching.
- **centre:** part centre to the pin or part named.

"2-layer" and "4-layer" split boards by copper layer count; "4-layer" includes 6 and 8.

Use the numbers as targets and checks, not as hard limits. The board's own design rules
and the router decide what is legal. Section "What humans do not do consistently" lists
habits that look like rules but are not; do not enforce those.

**Contents:** What human placements optimise · Spacing and density · Around a fine-pitch chip · Parts that serve a chip's pins · Decoupling · Bulk caps and filters · Switching regulators · Linear regulators · Power entry · Chips, buses and connectors · Rows, banks and alignment · Repeated channels · Functional groups · The back face · Connectors and the board edge · Interface protection and helpers · Clocks · Signals between chips · User interface · Modules and mechanical parts · What humans do not do consistently.

When a review finds a new pattern that a move fixed, add an entry in the same shape.

**How to find the problems on a routed board:** `python3 -X utf8 py_tools/route_review.py
BOARD` lists the open nets, the costliest nets, the congested areas with their parts and
the turned two-pad parts in one pass. One at a time:
- **Open nets:** for each one, `python3 -X utf8 py_placer/converge.py where BOARD --nets
  NET ...` gives the gap and the copper walling it in.
- **Nets with many vias, and long detours:** compare each net's routed length with the
  span of its pads. A net routed at 1.5-2x its span with several vias is usually a part
  placed away from the pins it serves.
- **Two-pin parts whose airwires cross:** turning the part 180 degrees shortens both.
  Use the `place_pose ... set` pose route_review prints, which swaps the two pads in
  place; `rotate` turns about the footprint origin (#1322).
- `python3 -X utf8 py_tools/render_placement.py BOARD -o OUT.png --ratsnest-nets NET ...`
  draws just the failing nets' airwires.

**Move one cluster at a time, and route again.** Moves in several places at once
cannot be attributed: on bitaxe_ultra, seven moves together closed two nets and opened
six others.

## What human placements optimise

- **Each net's wire is its minimum spanning tree over its pads, measured
  Manhattan, not each connection on its own.**
  - *Humans:* among every legal position within ±15 mm, a part's human position is
    likeliest under the rectilinear MST of its nets, then the Euclidean MST, then
    half-perimeter; a star (each pad to the net's centre) and a clique (every pad pair)
    fit worse (12,781 parts, 419 boards). The difference is in multi-point nets: for a
    part on nets of 6 or more parts, the MST optimum lies a median 3.2 mm from the human
    position, the star's 6.3 mm, and an optimum aimed at the centroid of every partner
    10 mm. 74-80% of legal 1-4 mm moves, turns and swaps make the human MST longer.
  - *Apply:* place a part where its pads join each net's tree cheapest: next to the
    nearest pad of each net, not in the middle of all its partners.
  - *Measured on a route:* in the grouped placer `place_simple` replaced, aiming parts
    this way, pricing the links between groups as each net's tree, and the spacing
    below routed six candidates on each of 10 corpus boards (by each board's recorded
    chain) with blocking 4.2 → 3.2, 6% fewer vias and 5% less copper; vias were lower
    on 9 of the 10 boards. `place_simple`'s wire term is this tree.
- **Humans do not pay much wire to avoid a straight-airwire crossing.**
  - *Humans:* fitted on 165,771 legal moves, one crossing is worth about 0.14 mm of
    Manhattan MST (0.07 mm on 4-layer boards, 0.65 mm on 2-layer ones). Still, human
    boards have 1.07 crossings per airwire against 2.09 with every part turned at random
    and 6.0 with positions shuffled: the crossings they avoid come from turning parts
    and ordering pins, not from moving them apart.
  - *Not:* the straight-airwire "layers needed" (the colouring of the crossing graph) is
    4 on 2-layer boards and 6 on 4-layer ones: it is not the routing layer count.
- **Align parts exactly with a neighbour.**
  - *Humans:* 68-72% of movable parts share an exact x or y with a part within 25 mm,
    against 7-16% after a ±1 mm jitter, and 31% of airwires lie within 5° of an axis
    (11% for random angles). Of every preference measured, this is the strongest one
    wire length does not explain.
  - *Apply:* snap a part onto a neighbour's x or y when it costs no wire.
- **Congestion grows with the layer count, not past it.**
  - *Humans:* the 95th percentile of tracks needed per 1 mm cell, divided by the copper
    layers, is 0.09 [0.06 0.14] on 2-, 4- and 6-layer boards alike: designers add layers
    in proportion to how much wiring crosses each area.

## Spacing and density

- **Pack neighbours courtyard to courtyard. The working gap is a few tenths of a
  millimetre.**
  - *Humans:* the nearest same-face neighbour's courtyard gap is 0.17 mm [0.05 0.50] for
    passives (24,670 parts, 392 boards), 0.27 mm for ICs, 0.13 mm for crystals and 0.69
    mm for connectors. Between two ICs it is 0.87 mm.
  - *Apply:* target 0.15-0.3 mm between passives and between a passive and an IC, 0.5-1
    mm next to connectors, about 1 mm between two ICs. Touching (0) is fine.
- **Set the gaps from the board's density, not its layer count.**
  - *Humans:* passives sit 0.45 mm apart on sparse boards (under 0.94 parts/cm²), 0.28
    mm on middle ones and 0.09 mm on dense ones (over 2.54 parts/cm²). At a fixed part
    size, 2-layer and 4-layer boards differ by only 0.02-0.1 mm.
  - *Apply:* compute the board's density first, then use that band's gaps.
- **Keep a passive's pads about 0.45 mm (0402) or 0.75 mm (0603 and larger) from any
  foreign pad.**
  - *Humans:* nearest foreign pad gap: 0201 0.40, 0402 0.45, 0603 0.73, 0805 0.86 mm.
    The 10th percentile is 0.30 mm for 0402 and 0.46-0.50 mm for larger sizes. A
    board's tightest passive is 0.38 mm from a foreign pad.
  - *Apply:* treat the 10th percentile for the size as a warning and the median as the
    target.
- **Courtyards may touch. An overlap is a deliberate tuck-in, not a packing habit.**
  - *Humans:* 6.5% of parts overlap another courtyard by more than 0.1 mm. These are
    decaps in a QFP's corner notch, small passives against a U.FL or a connector's empty
    courtyard area, and headers butted end to end.
  - *Apply:* grade legality on pads and bodies. Keep any courtyard-overlap penalty soft.
- **Fill about half the front face.**
  - *Humans:* courtyards cover 48% of the front face on both 2-layer and 4-layer boards.
    Small boards (under 10 cm²) reach 65%. The largest empty disc has a radius of about
    0.13-0.16 √(board area).
  - *Apply:* a seed with crowded areas next to an empty disc wider than 0.2 √(area) is
    unevenly spread. Even it out before routing.
  - *Not:* free space is not spread evenly part by part. Humans pack groups tight and
    leave pools: 27-31% of 5 mm blocks are empty, against 11-22% in a random legal
    arrangement.
- **Give no part extra room for its pin count. A gap widens only for the tracks that
  must pass through it.**
  - *Humans:* the free band around a part is about 1.0-1.3 mm at every pin count (2 pins
    0.98 mm, 65-128 pins 1.33 mm), and fine-pitch chips get less, not more, because
    their decaps fill it. 78-87% of the gaps between neighbours carry no airwire. Where
    tracks pass, the copper gap grows by about 0.33 mm per track on the parts' layer:
    at least 0.41 + 0.33·n mm on 4-layer boards and 0.73 + 0.33·n mm on 2-layer ones
    (5th percentile, n tracks).
  - *Apply:* leave room where the wiring needs a channel, not around every part. An IC
    keeps a little more than a passive: its neighbours' copper rarely comes within 0.5
    mm (4-layer) or 0.8 mm (2-layer).
  - *Measured on a route:* the polish of the grouped placer `place_simple` replaced
    asked 0.5 + 0.15·√pins mm of courtyard room per part, about 1.4 mm between two
    passives, which forbade 53-81% of human spacings. Cut to 0.02·√pins, its six
    candidates on each of 10 corpus boards (routed by each board's recorded chain)
    routed with blocking 4.8 → 3.2, 2% fewer vias and 1% less copper.

## Around a fine-pitch chip

- **Keep a ring around a fine-pitch chip for its own parts. Unrelated parts stay
  farther out.**
  - *Humans:* the nearest foreign pad sits 0.59 mm from a 0.5 mm-pitch QFN (1.2 pitches),
    0.74 mm from a QFP, 1.8 mm from a 0.8 mm BGA (2.3 pitches) and 1.0 mm from a SOIC.
    On 4-layer boards the QFN ring is 0.56 mm, on 2-layer boards 0.82 mm. Unrelated parts
    stay 1.7 mm (QFN) or 3.1 mm (BGA) away. 83-94% of the parts within 3 mm share a net
    with the chip.
  - *Apply:* reserve a band of about 1.5 mm around a QFN or QFP and 3 mm around a BGA for
    the chip's own decaps, terminations and pull-ups. Keep everything else outside it.
    Courtyards may touch the chip's courtyard.
- **Budget about 7 of the chip's own parts within 3 mm, spread over its sides.**
  - *Humans:* parts within 3 mm on the chip's face: QFN 7, QFP 10, BGA 6, SOIC 4; 1-3 per
    side. 4-layer boards carry more than 2-layer boards.
  - *Apply:* when one side carries 6 or more while another is empty, move parts to the
    side where their pins are.
- **Leave one side of a BGA open, preferably the side that faces its bus partner.**
  - *Humans:* 75% of BGAs have a side with no part within 3 mm. The side facing the main
    bus partner holds 0 parts within 3 mm, against 1.7 on the other sides, and it is
    the emptier side in 60% of BGAs. For QFNs and QFPs the effect is weak.
  - *Apply:* fill a BGA's other sides first. For QFNs and QFPs treat this as a
    tie-breaker.
- **A QFP's corner notches are decap sites.**
  - *Humans:* 19% of QFPs have a part in a corner square, beyond the ends of the pin rows
    and inside the QFP's courtyard, and another 9% have one near the corner. QFNs (2%)
    and BGAs (1%) almost never do this: their corners are too small.
  - *Apply:* offer a QFP's four corner squares to its 0402 and 0603 decaps, and do not
    reject them for overlapping its courtyard.
- **Under a BGA, on 4-layer boards, put the decaps on the other face, among the power
  balls of the inner half.** Keep the outer escape rows clear.
  - *Humans:* 68% of BGA decaps on 4-layer boards are on the other face; 69% at 0.8 mm
    pitch. 49% sit in the inner half of the ball field, where a uniform spread would put
    25%. The six balls nearest such a cap are supply or ground 67% of the time, against
    34% for the whole BGA. A BGA carries about 9 such caps per 100 balls, mostly 0402
    and 0201. Leftover bulk caps form a row just outside the field.
  - *Seen:* butterstick's ECP5 (0.8 mm pitch, back face): about 30 0402 caps fill the
    central power region, the outer 4-5 signal rows are clear, and a row of caps runs
    along one side outside the field.
  - *Not:* at 0.5 mm pitch or less, 70% stay on the BGA's face, in the ring.
- **Under a BGA, land the cap's pads on the via sites.**
  - *Humans:* both pads sit on a via site in 23% of under-BGA caps, against about 3% at
    random. At 0.8 mm pitch an 0402's pads span the gap between two interstitial via
    sites. Vias in or touching the pads: about half of these caps, against 6% of all
    decaps.
  - *Apply:* at 0.8 mm pitch or more, snap the cap so its pads land on the half-pitch via
    sites; use 0402 at 0.8 mm and 0201 at 0.5-0.65 mm. Count those vias as shared with
    the fanout.
- **A fine-pitch chip's caps on the other face go under the chip's body or well clear of
  its escape field, never over the pin rows.**
  - *Humans:* back-face decaps of QFNs sit under the pad field, about 0.8 mm inside the
    pin row; SOIC 1.0 mm inside, QFP 3.7 mm inside.
  - *Looks like, too far:* each supply net (chip pin, caps, a test point) spans 15-25 mm
    and needs 3-6 vias.
  - *Looks like, on the pin rows:* the chip's own signals start failing, because the caps
    take the escape tracks' room and the via sites.
  - *Measured on a route:* bitaxe_ultra, the ASIC U14 (a QFN) on the back with its
    stacked-domain caps on the front. With the caps 8-14 mm west of the pins, the domain
    nets needed 3-6 vias each. Caps directly over the pin rows opened two more nets: a
    domain net and the ASIC's RO signal. Caps 2-3 mm off the pin rows left no more nets
    open than before, with fewer vias (156 against 162 on a quick route).

## Parts that serve a chip's pins

- **Put a serving part on the side of the chip where its pin is.** Standing next to the
  chip's outline is not enough; a chip or module has pins on several edges.
  - *Humans:* 82% of decaps, pull-ups, pull-downs, terminations and filter caps sit on
    their pin's side of the chip, against 26% at random (8,658 parts, 324 boards). By
    package: QFN 74%, QFP 70%, BGA 68%, SOIC 53%.
  - *Looks like:* open or long nets between a header or test point and one edge of a
    module, with the part on the module's opposite side.
  - *Measured on a route:* bitaxe_ultra. The ESP32 module U12 has its IO0, P_TX and P_RX
    pins on its north edge. The programming header J5 and its test points had been
    placed against its south side, 20-35 mm from those pins. Moved beside the pins, all
    three nets routed.
- **Turn the pad that connects to the chip toward its pin.** The other pad (ground, the
  rail, the far net) points away.
  - *Humans:* 88-93% for every role, against 50% at random (6,217 parts, 314 boards).
    Within 3 mm of the pin it is 88%; at 6-10 mm, 72%.
  - *Apply:* after seating a two-pin part, try both 180° angles and keep the one whose
    chip-side pad is nearer the pin.
- **Budget each serving part's distance by its role.** Centre to pin, with the 25th
  percentile as the target:
  - decap 4.0 mm (25th percentile 2.4); for 0402 decaps 2.9 mm, for 1206 8.3 mm;
  - AC-coupling cap 3.7 mm;
  - pull-down or bias resistor 4.8 mm;
  - filter cap to ground 5.0 mm;
  - pull-up 5.5 mm;
  - series resistor 6.2 mm.

  4-layer boards are about 1 mm tighter. The long tails are parts that serve the far
  end of a net, such as a pull-up at the other end of a bus.
- **A connector's test points and helper parts move with it.** A test point left behind
  is a long net of its own.
  - *Measured on a route:* bitaxe_ultra. Moving J5 north and leaving P_RX's test point
    TP22 south opened P_RX; moving TP22 to the pin closed it.

## Decoupling

- **Put each decap on its own supply pin, not just beside the chip.** The chip pin
  nearest the cap should be a pin on the cap's rail.
  - *Humans:* pad gap from a decap to the pin on its rail:

    | Package | 4-layer | 2-layer |
    |---|---|---|
    | QFN | 1.36 mm [0.78 2.43] | 1.93 mm |
    | QFP | 1.49 mm | 1.87 mm |
    | SOIC | 1.38 mm | 1.86 mm |
    | SOT | 0.98 mm | 1.02 mm |
    | DIP | 2.29 mm | 2.29 mm |
    | BGA, same face | 3.04 mm | n/a |

    The chip pad nearest the cap is on the cap's own rail for 49% of decaps, against 10%
    at random (4,103 decaps, 319 boards).
  - *Apply:* score a decap against its own rail's pin, never against the chip outline.
- **The smaller the cap, the closer to the pin.** Order a pin's caps by value: 100 nF
  or less nearest, then 1-4.7 µF, then 10 µF or more.
  - *Humans:* pad gaps of 1.56 / 1.86 / 3.22 mm by value, and 1.32 / 1.77 / 2.80 mm for
    0402 / 0603 / 0805 and larger. On rails with two or more values, the smallest is the
    nearest in 72-74% of cases, against 49-56% at random.
  - *Not:* it is a tendency, not a law. Do not reject a layout for order alone.
- **Give each supply pin one cap.** A second cap on the same pin is a small plus a bulk
  cap. A BGA shares caps between neighbouring balls.
  - *Humans:* caps per rail equal the rail's supply pins (ratio 1.00, 2,623 rails); 76%
    of served pins have one cap and 19% have two. BGAs, and rails with 9 or more pins,
    have about 0.7-0.8 caps per pin.
  - *Apply:* place the caps the schematic has. Do not invent caps.
- **Put the decap's supply pad toward the pin and its ground pad on the far end.**
  - *Humans:* the supply pad is the nearer one in 89% of decaps; 95% when the gap is
    under 1.5 mm. Decaps that stand radial to the chip edge have the ground pad outward in
    95%. Resistors do the same (84%), so this is a habit for every passive.
- **Give each decap's ground pad its own via, just beyond the pad's outer end.**
  - *Humans:* 4-layer boards put a ground via 0.25 mm [0.06 0.70] from the decap's ground
    pad, against 1.71 mm from a signal resistor's pad. 2-layer boards: 0.69 against 3.25
    mm. The via sits beyond the pad's outer end in 50% (25% at random) or beside the pad
    in 47%, and almost never back toward the supply pad (4%). One in five such vias is
    shared with another decap.
  - *Apply:* reserve a via site 0.2-0.3 mm beyond or beside the ground pad. Never put it
    between the pads or toward the pin.
- **On 4-layer boards with a plane rail, the decap's supply pad gets a via too.** On a
  rail that is a trace, the supply pad runs to the pin on the surface.
  - *Humans:* 0.21 mm from the supply pad to a rail via when the rail has a zone, 0.67 mm
    when it does not. 2-layer boards: 2.4 mm, so a surface trace.
- **On 2-layer boards keep decaps on the chip's face. On 4-layer boards, putting them on
  the back, behind the pin, is a per-board style.**
  - *Humans:* decaps on the other face, 2-layer: 12-24% by package. 4-layer: about 25% for
    QFN, SOIC and SOT, 68% for BGA. 42 of 191 4-layer boards put most of their decaps on
    the back; most boards put none there.
  - *Apply:* use the back for decaps only on 4-layer boards, and first when the front
    ring is full.
- **Oscillator decaps sit tighter than IC decaps: within about 1 mm of VDD, with a ground
  via at about 0.2 mm.**
  - *Humans:* 0.96-0.98 mm pad gap; 95% of oscillators have one. An addressable LED
    (WS2812 and similar) has its cap at its VDD pin as well (6 boards only).
- **When a board has several identical chips, solve one chip's decap layout and copy it
  in each chip's own frame.**
  - *Humans:* 36% of identical-chip pairs repeat 80% or more of their decap positions
    within 0.3 mm, in the chip's frame. Unrelated chips repeat none. The same holds for
    regulator stages (25% exact within 0.5 mm, 43% within 2 mm).
  - *Seen:* icepi_sbc_h3 lays out six SY8088/SY8089 bucks as near-copies.
  - *Not:* two thirds of pairs are adapted to their surroundings. Seed from the copy,
    then let legality and routing adjust.

## Bulk caps and filters

- **Put input-rail bulk caps at the regulator; split output-rail bulk caps between the
  regulator and the load.** Small caps belong at the loads. Bulk caps rarely sit at the
  power connector.
  - *Humans:* bulk caps (10 µF or more) on a regulator's input rail sit nearest the
    regulator in 71%, at about 3 mm, and nearest the connector in only 13%. On an output
    rail they split: 51% at the regulator or inductor, 39% at the load. Caps of 100 nF or
    less on an output rail sit at the load in 90%.
  - *Apply:* anchor input bulk caps to the regulator's VIN pin at 1-3 mm. Put the first
    output bulk cap at the regulator output (a buck's inductor output pad), and any
    others at the largest load on that rail.
- **A supply ferrite bead goes at the one chip it feeds, about 3 mm from that chip's
  pin, with a cap right after the bead and a decap at the pin.**
  - *Humans:* 152 supply beads on 62 boards; 93% feed a single chip. The bead's output
    pad sits 3.3 mm from the consumer's pin and is nearer the consumer than the regulator
    in 68%. The first downstream cap sits 0.64 mm after the bead, and a decap 1.07 mm
    from the pin. 45% also have a cap before the bead, forming a C-L-C filter.
  - *Apply:* treat bead, cap and consumer pin as one chain.

## Switching regulators

- **A switching regulator's power stage is one tight cluster.** Place the inductor first,
  then the input cap, then the output caps, then the feedback divider and boot cap.
  - *Humans:* every member of the stage sits within about 0.3 mm (courtyard gap) of its
    neighbour. icepi_sbc_h3's U6 (a SOT-23-5 buck): input cap against the chip's west
    side, inductor directly north, output caps beside the inductor's output pad, the
    feedback divider and its caps in a tight row on the south side.
- **Butt the inductor against the SW pin, with its switch-node pad facing the pin.**
  - *Humans:* SW pin to inductor pad: 1.30 mm on 4-layer boards (159 stages, 91
    boards) and 2.56 mm on 2-layer boards. Courtyard gap 0.35 mm. On the chip's face
    in 98%. The switch-node pad is the nearer pad in 86%, against 50% at random.
  - *Apply:* courtyard gap 0.2-0.5 mm, switch-node pad gap at most 1.5 mm (4-layer) or
    2.5 mm (2-layer).
- **Put the input cap across VIN and ground: about 0.8 mm from VIN, with its ground pad
  toward the chip's ground, so the loop is about 3 mm.**
  - *Humans:* VIN to the nearest input cap 0.84 mm on 4-layer boards (2.29 mm on 2-layer).
    The loop (cap to VIN, plus cap ground to chip ground) measures 2.9 mm on 4-layer
    boards and 4.9 mm on 2-layer ones.
  - *Apply:* score the input cap by that loop. Target 3 mm or less on 4-layer boards, 5
    mm or less on 2-layer ones.
- **Put the input cap and the inductor on different sides of the chip, about 90° apart
  around it.**
  - *Humans:* the angle input cap - chip - inductor is 97° [68 143]. Only 22% share a side.
  - *Apply:* let the pinout pick the sides: the inductor on the SW pin's side, the input
    cap on the VIN/ground side.
- **A buck's output caps sit against the inductor's output pad, about 0.8 mm away,
  usually two of them.** They do not sit at the chip.
  - *Humans:* 0.79 mm on 4-layer boards, 1.25 mm on 2-layer ones; 2 caps within 5 mm.
  - *Not:* a boost reverses this: the inductor's far pad is VIN, so the input cap goes
    there.
- **The feedback divider sits at the FB pin, about 0.9 mm away.** It is usually on the
  side away from the inductor, but only because the FB pin is.
  - *Humans:* 0.91 mm on 4-layer boards; on the far side from the inductor in 68%. Its
    distance to switch-node copper is the same as other nearby passives'.
  - *Apply:* anchor it to the FB pin. Do not add a separate keep-away from the switch
    node.
- **The boot cap bridges BOOT and SW at the pins.** It sits about 1 mm from BOOT and
  within 2.5 mm of SW, and may share the switch-node copper with the inductor.
- **An asynchronous buck's catch diode sits within about 2 mm of the SW pin, beside the
  inductor's switch-node pad.** Small sample (16 boards).
- **In a discrete buck, the two FETs face each other with their switch-node pads about
  1.4 mm apart, courtyards touching, and the inductor right against them.**
  - *Humans:* three boards with SMD power FETs have their switch-node pads 1.30-1.40 mm
    apart and the inductor 0-1.7 mm away.
  - *Looks like:* the switch node (a wide net) open or detouring, and the FETs and
    inductor at their default angles with the switch-node pads facing away from each
    other.
  - *Measured on a route:* bitaxe_ultra. The placement had Q1 to Q2 at 12.0 mm, Q2 to L1
    at 12.7 mm and Q1 to L1 at 24.5 mm, all at 0 degrees, and Q2's switch-node tab faced
    away from L1. The human layout has 6.0, 8.5 and 14.4 mm, with the FETs at 90 and -90
    degrees. With the cluster compacted to the human spacing, a quick route closed
    /Power/SW and the enable line beside it.
- **Do not leave extra room around power parts for wide copper.** Make power nets short
  by clustering the stage instead.
  - *Humans:* passives on wide nets sit exactly as close to their neighbours as signal
    passives (paired difference 0.00 mm over 194 boards). The switch node is only 3.9 mm
    long and twice the signal width.

## Linear regulators

- **Put each LDO cap against its own pin, about 0.9 mm away, on the regulator's face.**
  When IN and OUT are on different sides of the package, the caps flank the regulator;
  when they share a side, the caps sit side by side.
  - *Humans:* 0.87 mm from IN and from OUT on 4-layer boards (321 regulators, 200
    boards); the input cap drifts to 1.7 mm on 2-layer boards. Each cap is nearer its own
    pin in 87-91%. Both are on the regulator's face in 96%. With IN and OUT apart, the
    caps flank it in 59%; with them on one side, they sit side by side in 60%.
- **Do not pull a regulator onto its load.** Place it by its input, between the power
  entry and its loads, and let the decaps carry locality.
  - *Humans:* a single-load LDO's output pin is 12 mm from its load, and a single-load
    buck's inductor 20 mm, while decaps sit 1-2 mm from their pins.

## Power entry

- **From the power connector inward: the fuse and TVS first, then the reverse-polarity
  part, the bulk cap, any ferrite, and the regulator.**
  - *Humans:* the TVS sits about 4-8 mm from the connector's power pin (pad gap), the
    fuse about 6-12 mm, and both are among the 5% of parts nearest the connector. The
    fuse is nearer the connector than the reverse diode in 14 of 16 chains, and nearer
    than the bulk cap in 24 of 35. Regulators sit 20-25 mm in.
  - *Not:* most boards have no entry protection. A fuse or TVS appears on only 5% of
    power connectors (12% of USB and barrel inputs), and 67 of 115 paths from a power
    connector to a regulator have no series part at all. Place what the schematic has;
    do not reserve room for absent parts.

## Chips, buses and connectors

- **Put chips that share a bus next to each other.** A chip's nearest IC is usually its
  partner.
  - *Humans:* pairs sharing 3 or more signal nets sit at 0.67× the centre distance of
    unrelated pairs, and closer on 80% of 186 boards. A chip's nearest IC shares a net
    with it in 50% of cases, against 21% of all IC pairs. The courtyard gap between
    talking chips is 7-15 mm, which leaves room for the bus and its terminations.
  - *Apply:* seed bus partners next to each other, 3-10 mm apart. A chip whose nearest IC
    is unrelated while its partner is far away is a defect.
- **Turn a four-sided chip (QFN, QFP, BGA) so the pins it shares with a partner face
  that partner.** Dual-row chips (SOIC, TSSOP) do not need this.
  - *Humans:* the angle between the shared pins' centroid direction and the direction to
    the partner is 31° [11 82] for four-sided chips (n=164, 96 boards), against about
    90° at random. It is 62% within 45°, against 9%. The angle is 22° for QFPs, 32° for
    QFNs and 38° for BGAs, but 88° for SOICs. The effect fades beyond a 25 mm gap.
  - *Apply:* for each four-sided chip with a bus partner, choose the 90° rotation that
    points its shared-pin centroid at the partner.
- **Turn a chip's bus side toward its connector, and the connector's pins toward the
  chip.** The connector does not have to be centred on those pins.
  - *Humans:* the side faces the connector at 50° against an 85° baseline (893 pairs,
    246 boards). The two pin groups are offset by 0.46 of their span along the face, so
    a connector slides along its edge freely.
- **The USB chip is the IC nearest the USB connector, about 9 mm behind it.**
  - *Humans:* rank 0.10 among the board's ICs by distance (the nearest in 46%, the
    nearest third in 74%; 125 connectors). The courtyard gap is 9.0 mm [4.0 18.5],
    against 30.6 mm for the median other IC.
  - *Not:* Compute Module carriers and USB muxes carry D+/D- across the board.
- **A debug or programming header (SWD, JTAG, UART) goes at the edge, at the point
  nearest the MCU it programs.**
  - *Humans:* courtyard gap 6 mm (JTAG) to 9 mm (SWD, UART), 51-68% within 3 mm of the
    edge. It is usually the nearest connector to that MCU.
- **An op-amp is the IC nearest the connector its signal enters by.** Its gain and
  filter parts pack around it.
  - *Humans:* rank 0.10 among ICs by distance to that connector (79% in the nearest
    third, 43 boards). Its R/C network sits 4.1 mm (pad gap) from its pins, against 9.9
    mm for an MCU's R/C parts. Feedback parts sit 2.5 mm from the pins, on the same face
    in 99%.

## Rows, banks and alignment

- **Line up same-footprint passives as a comb: parallel, side by side, centres on one
  line, at a fixed pitch.**
  - *Humans:* 1,524 rows on 253 boards; about a quarter of all passives sit in one.
    The comb pitch is 1.09 mm for 0402, 1.75 for 0603 and 2.5 for 0805 and 1206, about
    the part width plus 0.5 mm. 75-81% of rows are on an exact line (residual under
    0.01 mm). 92-100% share one rotation modulo 180°.
  - *Apply:* seat several same-size passives serving one area as a comb at these
    pitches, and snap their centres onto one line.
  - *Not:* only 40-53% of rows hold a single value. A row is a geometric habit, not a
    "same part" habit, so do not sort passives into rows by value.
- **Give a passive the same axis as its nearest same-size neighbour.**
  - *Humans:* same axis in 88-91% of nearest-neighbour pairs, against 55% from the
    board's own axis mix (15,439 pairs, 348 boards). Near neighbours are often aligned
    exactly as well (62-77% within 0.01 mm).
  - *Not:* across the whole board, only 62% of passives are on one axis. The alignment
    is local, so do not force one axis board-wide.
  - *Board-wide habits:* 75% of boards place every part at a multiple of 90°, and about
    half of all parts are aligned in a row or column with a neighbour within 4 mm.
- **When side-by-side passives share a net, put that pad at the same end, so the pads
  form one strip.**
  - *Humans:* 94% for a shared ground or supply pad, 89% for a shared signal pad, against
    50% at random (329 boards).
  - *Apply:* in a row of decaps, point every ground pad the same way. In a row of
    pull-ups, line up the rail pads. One straight track or pour then joins them.
- **A bank serving a bus (terminations, pull-ups, filters) follows the pin order, at
  about twice the pin pitch, parallel to the chip edge that carries those pins.**
  - *Humans:* 58% of rows serve 3 or more pins of one chip or connector. Their order
    follows the pins in 76-82% of rows, against 21% at random. The row pitch is about
    2.1× (resistors) to 2.6× (caps) the pin pitch. The nearest member sits 5.6 mm from
    its pin, beyond the chip's first ring.
- **Turn a part so the pads carrying its connections face the parts they connect to.**
  A seed keeps a part's input angle when it fits, and a pile's angles are arbitrary.
  - *Looks like:* a net that routes around its own part's body to reach the far pad, or
    two airwires crossing at a two-pin part.
  - *Measured on a route:* bitaxe_ultra's Q2 had its switch-node tab facing away from
    the inductor (see Switching regulators).

## Repeated channels

- **Lay out the first channel, then place the others as copies of it, translated along
  one row or column, about twice the channel's radius apart.** A channel is a repeated
  block: N motor drivers, N LED drivers, N input stages.
  - *Humans:* 513 pairs of matching channels on 109 boards. 38% are exact translations
    (median satellite residual under 0.25 mm), 49% within 1 mm. Nearly all exact copies
    (96%) are offset along one axis only, with the satellites at the same rotations.
    Copies sit 17.5 mm [10.3 22.6] apart, which is 2.0× the channel's median satellite
    offset. Groups of 3 or more copies are on one line in 42% of cases.
  - *Seen:* lumenpnp_mobo lays four motor-driver channels (driver, inductor column, cap
    column, connector) on a 2×2 grid, and stacks four FET channels in one column.
  - *Apply:* detect channels as parts of one footprint and value whose neighbours match
    by footprint, value and pin. Seed the copies as translations of the first.
  - *Not:* 39% of channel pairs are rearranged by more than 1 mm per part, because
    humans adapt copies to the board's shape. Treat a copy as a strong seed, not a
    constraint. 180° copies are rare (4%) and exact mirrors never occur.

## Functional groups

- **Place by function: write the groups.** A group is the parts that work together
  (power entry and its conditioning, the core and its clock, an interface and its
  protection, the user interface); `place_simple` keeps each group in one piece,
  whatever shape the free space gives it, and arranges the pieces by one objective.
  - *Apply:* five to eight groups suit a small board; more than about twelve leave
    each group too few parts to be a piece. Place more than one seed: seeds differ
    more than most changes to the groups.
- **Keep each functional group compact and on one face.**
  - *Humans:* on 94 boards whose hierarchical sheets label the parts, a part's nearest
    labelled neighbour is in its own sheet 93% of the time, against 37% with the labels
    shuffled. This holds on every one of the 94 boards. Groups have 12 [7 20] members
    and a radius of about 0.14 √(board area): 6.6 mm for 4-10 parts, 11.4 mm for 10-30,
    18.3 mm for more. They stay on one face (median 100% of members).
  - *Apply:* a group with a `side` is put on that face before placing; leave `side`
    out to keep each part on the face it is on.
- **Parts drawn together on the schematic are placed together.**
  - *Humans:* on 239 boards whose schematics were fetched from their repositories,
    how close two parts are drawn predicts whether they are among the board's 10%
    closest pairs with a median AUC of 0.76, against 0.60 for how strongly the
    netlist connects them and 0.66 for sheet plus netlist, on 88% of boards. It
    still predicts with the netlist distance held fixed (0.79 for parts sharing a
    net, 0.73 for parts sharing none). A drawn block (`board_context`, a median 3.6
    parts) sits 0.20 as wide as random parts of its size, against 0.45 for as many
    netlist clusters, on 91% of boards. For 4-8 groups, sheet plus netlist is as
    tight as the drawing on hierarchical boards.
  - *Routed:* on 4 corpus boards placed from a pile (12 seeds each, every seed
    fast-routed), group membership written with the schematic routed better than
    membership written without it on all 4, with the groups' `near`/`apart`/`edge`
    hints removed from both: more clean seeds or a lower median of blocking nets
    (esp-rust-board 7 vs 4 clean of 12, nudac a median of 1 vs 3).
  - *Apply:* take the groups from the sheets and keep each drawn block in one
    group (SKILL.md §3.3).
- **A designer's layout note ("C11 close to pin45") is ignored unless it becomes
  a `proximity` row.**
  - *Routed:* on 8 corpus boards placed from a pile with the same groups, 12 seeds
    each and every seed fast-routed, the noted pairs met their limit (2.0 mm at a
    pin, 5.0 mm part to part) in 58 of 180 seeds without the rows and 164 of 180
    with them, with no measurable routing cost. Without a row the placer put
    usb_fm_radio's C6 a median 11 mm from its pin. Notes go stale: on usb_fm_radio
    two of four pin notes name a cap a renumbering moved to another rail.
  - *Apply:* SKILL.md §3.3. The human designers kept these pairs closer than 2 mm
    in about half the cases, so a limit the user gives outranks the default.
- **On a 4-layer board, leave about 1 mm between neighbouring groups.** Inside a group,
  parts are about 0.2 mm apart. 2-layer boards show no such gutter.
  - *Humans:* boundary parts sit 1.02 mm from the nearest part of another group and
    0.17 mm from their own group (4-layer boards; n=1,744, 74 boards). On 2-layer boards
    the two are 0.91 and 0.89 mm.
- **Put the power conditioners at the power entry, not at the edge.** Do not give the
  power group an `edge` hint.
  - *Humans:* regulators, switchers and chargers are no closer to the outline than an
    average part; only 12% are within 3 mm of it. On the median board only 16% of the
    parts are nearer the entry connector than the conditioner is, and 66% of
    conditioners sit nearer the entry than the main IC does. The input cap sits about
    2.0 mm away, the output cap 2.2 mm and the inductor 2.6 mm.
- **Put the main IC central.**
  - *Humans:* 57% of main ICs (the part with the most pads) sit in the central region,
    against 28% of all parts.
- **Put ESD and TVS parts at their connector.**
  - *Humans:* about 2.5 mm from it; 96% sit nearer their connector than the main IC.
    Section "Interface protection and helpers" has the per-interface numbers.
- **LEDs and buttons lean toward the edge; connectors sit on it.** Sections "User
  interface" and "Connectors and the board edge" have the numbers.
- **Decaps sit about 2.5 mm from the supply pin they serve:** 3.6 mm on 2-layer boards
  and 2.2 mm on 4-layer boards. On 4-layer boards about a third sit on the opposite face.
  Section "Decoupling" has the detail.
- **A group's parts on the other face stand by the group's connector.**
  - *Measured:* pulled only toward the rest of its group, bitaxe_ultra's USB group put
    its CC resistors 35 mm from their connector J8 (locked on the back), and its fan
    controller 80 mm from the fan header.
- **The groups' numbers come from `tests/stress/placement_survey.py`.** To reproduce them
  (building the corpus is in `tests/stress/RUNBOOK.md`):

```bash
python3 tests/stress/placement_survey.py run --sets 1-30 --jobs 6 --out rows.jsonl
python3 tests/stress/placement_survey.py report rows.jsonl
```

## The back face

- **Use the back face for passives on dense 4-layer boards, under the chip they serve.**
  Keep ICs, LEDs and connectors on the front unless the board already puts them on the
  back.
  - *Humans:* 61% of 4-layer boards and 35% of 2-layer boards carry SMD parts on the
    back, rising to 66% of the densest third. On those boards, about a third of the parts
    sit on the back, and passives are the largest share.
    Above a back-face passive sits nothing (51%), a fine-pitch IC (19%), another IC (9%)
    or a connector (7%).
  - *Not:* a few boards put everything on the back (3% of 2-layer boards). Read the
    input's own side convention before moving parts across faces.
  - Keyboards put their diodes and LEDs on the back, under the switches.

## Connectors and the board edge

- **A USB receptacle goes on the edge with its mouth facing out, its long axis parallel to
  the edge, and its shell about 1 mm past the outline.**
  - *Humans:* 227 connectors on 180 boards. 98% of right-angle receptacles face the
    nearest edge, and the long axis is parallel to the edge in 93-95%. The courtyard
    front overhangs the outline by 0.7-1.4 mm, depending on the footprint family.
  - *Apply:* seat it by its signal pads, with their centroid 6.8 mm from the edge for
    USB-C, 4.4 mm for micro-B and 12.9 mm for USB-A.
  - *Not:* where it sits along the edge is free: 42% are centred, 14% near a corner.
- **Turn every right-angle connector so its mating face points at the nearest edge.**
  - *Humans:* 92% of 502 right-angle connectors within 3 mm of the edge (USB-C 99%,
    right-angle headers 99%, barrel jacks 85%, RJ45 83%).
  - A barrel jack enters end-on, with its plug axis perpendicular to the edge, and sits
    near a corner (its position along the edge is 0.18 of the way from a corner).
- **The edge setback depends on the connector type.** The setback is courtyard to
  outline; a negative setback is an overhang.

  | Type | Setback (mm) | Within 3 mm of the edge |
  |---|---|---|
  | USB-C | −0.85 | 94% |
  | micro-B | −0.68 | 97% |
  | RJ45 | −1.82 | 100% |
  | Barrel jack | −0.80 | 97% |
  | HDMI | −1.26 | 95% |
  | microSD | 0.61 | 82% |
  | Terminal block | 0.84 | 82% |
  | JST-SH | 0.30 | 86% |
  | JST-PH, JST-XH | 0.5-0.6 | 68% |
  | Box (IDC) header | 1.95 | 70% |
  | Right-angle header | pins hang out | 98% |
  | Vertical header, 7-20 pins | 1.39 | 62% |
  | Vertical header, 1-6 pins | 4-5 | 41% |
  | All parts | 10.6 | 18% |

  Short vertical headers have no edge rule.
- **Neighbouring connectors on one edge sit about 3 mm apart. Identical ones butt
  together; SMA connectors need about 6 mm.**
  - *Humans:* courtyard gap 2.96 mm [0.76 8.82] (1,000 pairs, 273 boards). Terminal
    blocks sit 1.0 mm apart, JST-PH 0.7 mm, SMA 6.3 mm (wrench room).
  - *Apply:* lay identical connectors as one row at a fixed pitch with about 1 mm gaps.
- **Leave mating room around pin headers and box headers: about 1 mm of courtyard
  clearance.** USB and SD receptacles are packed like any other part.
  - *Humans:* 2.54 mm headers have 1.06 mm to their nearest neighbour (971, 211 boards),
    JST and terminal blocks about 0.55 mm, USB 0.20 mm and SD 0.13 mm. The figure for all
    parts is 0.20 mm.
- **Each connector type keeps its own helpers within about 2 mm of the pins they
  serve, on the connector's side.**
  - *Humans:* USB has CC pull-downs (60% of connectors, 0.5 mm courtyard gap), ESD (46%,
    0.8 mm) and series R (27%, 1.8 mm). An SD card has pull-ups (30%, 0.6 mm) and series
    R (36%). HDMI has series R (71%, 1.8 mm). A Qwiic/JST-SH connector has ESD, series R
    and pull-ups at 0.3-1.7 mm. Pin headers mostly have none.
- **Keep passive and IC pads at least 1.5 mm from the outline; 2 mm is typical.** LEDs
  and buttons may come to 0.6 mm.
  - *Humans:* passive pads are 2.0-2.2 mm from the outline at the 5th percentile. The
    tightest passive on a board is 1.55 mm in (median of 401 boards), ICs 1.6 mm, SOT
    ICs 1.2 mm.

## Interface protection and helpers

- **USB ESD goes right behind the connector's pin row, between the connector and the
  chip, ahead of any series resistor.**
  - *Humans:* 2.0 mm pad gap from the D pins (104 parts, 76 boards; 4-layer 1.8, 2-layer
    3.1). Along the line from the connector pin to the chip pin it sits at 0.23 of the
    way, and between the two in 93%. It is nearer the connector than the series R in 82%.
    ESD and TVS parts on every kind of connector: 2.0 mm, 82% between connector and IC.
  - *Not:* the 7% behind the connector are flow-through arrays under the receptacle.
- **Put each 5.1 kΩ CC pull-down beside its own CC pin.** The two are not a tight pair.
  - *Humans:* 2.6 mm pad gap from the CC pin (205 resistors, 95 boards; 4-layer 1.9,
    2-layer 3.7), 84% on the connector's side. Only 27% of the two sit within 1 mm of each
    other, because CC1 and CC2 leave the pin row at different points.
- **Place the two USB D+/D- series resistors as one touching pair: side by side, one
  rotation, centres aligned.**
  - *Humans:* courtyard gap 0.14 mm, the same rotation in 99%, aligned in 92% (88 pairs,
    69 boards). On an RP2040 or RP2350 the pair sits at the chip (82%, about 3 mm pad
    gap). For other chips, chip end and connector end are equally common.

## Clocks

- **Put a crystal beside the oscillator pins on the chip's face, its pad axis parallel
  to the XIN/XOUT pin pair, with the legs uncrossed.**
  - *Humans:* 186 crystals on 143 boards. The far pad sits this far (pad gap) from its
    pin: 3.4 mm for 2016 and 2520 packages, 5.0 mm for 3225, about 6 mm for 5032 and
    HC49, about 3 mm at 32 kHz. On the chip's face in 94-99%. Pad axis parallel to the
    pin pair in 69%. The straight pad-to-pin lines cross in only 6%.
  - *Apply:* reach the distance target, then try both 180° flips and keep the one whose
    legs do not cross.
  - *Not:* crystals are not kept away from the edge (11% within 3 mm, like other parts).
    The strip between crystal and chip may hold the chip's decaps (38% have a part
    there).
  - *Limits:* measured pad edge to the pad on the same net, the way an intent's
    `proximity[]` claim grades it, the farther crystal leg is 5.4 mm from its pin
    (75th percentile 8.1 mm; 258 crystals) and the farther load cap 0.8 mm from the
    crystal (75th percentile 2.2 mm; the 174 with a load cap). `placement/crystals.py`
    writes claims at those 75th percentiles; a leg through a series resistor is
    claimed link by link.
  - *Measured on a route:* turning the IC for its crystal alone cost a 21-part board 13 routed
    vias; turn it for the crystal only among the rotations its other connections
    accept.
  - *Apply:* the crystal and its load caps belong to their IC's group: `place_simple`
    folds them into it and claims them at its oscillator pins. Held to a separate
    clock group beside the
    chip, a 25-part board's load caps went round to the far side of the IC, 7 mm from
    the crystal, and the oscillator nets routed 13-40 mm with 2-5 vias each where the
    human routes 6 mm.
- **Put one load cap at each crystal pad, about 0.7 mm away, as a mirrored pair
  flanking the crystal.**
  - *Humans:* 0.66 mm pad gap (327 caps, 129 boards), each on its own pad's half of the
    crystal in 91%. The two distances are equal to 0.03 mm, and the pair shares a rotation
    in 80%.
  - *Not:* whether the caps sit toward the chip, level with the crystal or away from it
    varies freely (37 / 21 / 42%).
- **An oscillator's decap goes within 1 mm of its VDD pin.** That is tighter than an IC
  decap. Its output goes about 5 mm from the clock input.
  - *Humans:* 0.98 mm pad gap (110 decaps, 79 boards); 95% of oscillators have one.

## Signals between chips

- **Put a series resistor (10-150 Ω) within about 2 mm of one end of its link, never
  midway. Put it at the driver when the pin types say which end drives.**
  - *Humans:* 2.25 mm from the nearer end (308 chip-to-chip resistors, 56 boards). Where
    pin types name the driver, 90% sit nearer it, but that rests on only 17 boards.
    Ethernet PHY RX lines carry the resistor at the PHY, their driver, in 94%. Between a
    chip and a connector there is no preferred end.
  - *Apply:* treat an output or tri-state pin, or a known family such as a PHY driving
    RXD, RX_CLK and RX_DV, as the driver. Do not infer direction from RX/TX names: a PHY's
    RXD is an output.
  - Series terminations stand radial to the chip edge (78% of those within 4 mm of their
    pin, against 50% at random), so the signal runs straight through.
- **Put the I2C pull-ups as a touching pair at one end of the bus, usually a peripheral
  end, not at the controller.**
  - *Humans:* pair courtyard gap 0.33 mm, the same rotation in 91% (174 pairs, 123
    boards). They sit 2.6 mm from the nearest bus pin. That pin belongs to a peripheral in
    65% of cases and to the controller in 29%, against 37% by chance.
- **Put a reset RC beside the reset button when the board has one.** Without a button it
  sits about 5 mm from the MCU's reset pin; humans do not keep it tight.
  - *Humans:* with a button on the net, the R is nearer the button in 69% (1.9 mm away)
    and the C in 67% (3.1 mm away). Without one, the R is 5.5 mm and the C 6.1 mm from
    the pin.

## User interface

- **An indicator LED and its resistor touch, with the LED pad on the resistor's net
  facing it and the resistor on the inboard side.**
  - *Humans:* courtyard gap 0.38 mm (894 pairs, 242 boards). The pad faces the resistor
    in 74%, and the resistor is inboard of the LED in 70%. Inline (50%), side by side
    (30%) and L-shaped (20%) arrangements are all common.
  - *Apply:* move the LED and its resistor as one unit.
- **Three or more status LEDs form one straight row at a uniform pitch of about 3 mm,
  near and parallel to an edge, with the resistors in a parallel row about 2.5 mm
  behind.**
  - *Humans:* 86 rows on 65 boards. Pitch 2.97 mm (common pitches 2.0, 1.5, 3.0 and 5.0
    mm), uniform in 74%, one rotation in 90%, within 6 mm of the edge in 66%. The
    resistors are collinear in 81%.
  - *Not:* status LEDs are not placed near the MCU.
  - *Not:* a row of like value is not a bank. ft2232h_jtag's row of 1 kΩ parts is a
    reset pull-up and four LED resistors.
- **A side-actuated button goes on the outline; a top-actuated one leans toward the
  edge.** Their pull-ups and debounce caps are loose, 6-7 mm away.
  - *Humans:* side-actuated buttons: 86% within 3 mm of the edge, 39% overhanging (69
    parts). Top-actuated buttons: 44% within 3 mm (302 parts), against 19% of all parts.

## Modules and mechanical parts

- **Turn a PCB-antenna module (ESP32 WROOM or MINI, nRF MDBT, ESP-12) so its antenna
  end faces the nearest edge, with the antenna flush or overhanging.** Keep parts and
  vias out from under the antenna.
  - *Humans:* faces the nearest edge in 91%, flush or overhanging in 72% (33 modules, a
    small sample). Some boards notch the outline under the antenna instead. A third of
    boards do put parts or vias under it, which falls short of the module guidelines, so
    do not copy that.
- **Put mounting holes in the corners, inset about 1.25 hole diameters from both edges,
  forming a rectangle.**
  - *Humans:* inset 2.8 mm for M2, 3.1 mm for M2.5, 4.0 mm for M3, 5.1 mm for M4
    (701 corner holes, 195 boards). The x and y insets are equal in 68%. Four-hole boards
    form a rectangle in 86%. Foreign pads stay 1.5 mm (M2) to 4 mm (M3) from the hole
    edge. 48% of boards have no holes; those are the smaller boards.
  - *Only four:* all four holes sit within 8 mm of a corner on 86 of the 125 boards with
    exactly four. With one to three holes, or five or more, no hole count reaches 30%,
    so do not send those to the corners by default. A hole's position is the
    enclosure's: lock it where the brief or the user puts it.
- **Put a test point within about 2 mm of a pad it probes, all test points on one face,
  at least 2.5 mm apart (centre to centre), and not at the edge.**
  - *Humans:* 1.95 mm pad gap (2,446 test points, 181 boards). All on one face on 80% of
    boards. The nearest test point is 2.7 mm away, and the per-board minimum is 2.5 mm.
    Only 11% sit within 3 mm of the edge.
- **Add fiducials only when the brief asks.** Only 12% of boards have them. When
  present, there are three per populated face, near the corners, about 3 mm from the
  edge, spread over 84% of the board diagonal.

## What humans do not do consistently

These look like rules but the boards say otherwise. Do not enforce them, and do not
score a placement down for breaking them.

- **One rotation for all ICs, or pin 1 pointing one way.** The most common IC rotation
  covers about half a face's ICs; all ICs share one rotation on only 12% of faces. Rotate
  a chip for its bus instead.
- **One axis for decaps** (radial or parallel to the chip edge). 47-48% are radial, and
  boards split into styles: 48 boards have 20% or fewer radial, 34 have 80% or more.
  Score the pad order and the gap, not the axis.
- **One axis for all passives on the board.** Alignment is local (neighbours agree 88-91%);
  board-wide, only 62% share an axis.
- **A coarse placement grid.** Offsets between nearby parts are multiples of 0.05 mm in
  42% of cases (14% by chance), but multiples of 1.27 mm in only 2%. Snap only to make
  alignments exact.
- **Rows of one value.** Only 40-53% of rows hold one value, and decaps are no more
  aligned than resistors. Rows are tidiness; never trade a gap or a pad order for one.
- **Exact copies of repeated channels.** 38% are exact; the rest are adapted. Exact
  mirrors never occur.
- **A gutter between groups on 2-layer boards** (there is one on 4-layer boards).
- **The facing and side rules on dual-row chips** (SOIC, TSSOP): the side rule holds
  for 53% and the facing angle is no better than chance.
- **A connector centred on its chip's pins.** The pin groups are offset by about half
  their span; a connector slides along its edge.
- **Courtyards never overlapping.** 6.5% of parts tuck into another's courtyard.
- **Load caps toward or away from the chip.** 37 / 21 / 42% toward, level, away.
- **A preferred end for a series resistor between a chip and a connector** (51% at the
  chip), or for USB series resistors on chips other than the RP2040 (50%).
- **I2C pull-ups at the controller.** 29%, below the 37% chance rate.
- **Short vertical headers at the edge.** Only 41% are within 3 mm.
- **One LED-resistor shape.** Inline 50%, side by side 30%, L-shaped 20%.
- **Status LEDs near the MCU.** They sit farther from it than an average part does.
- **Reset and boot RC tight to the pin** (without a button), or button pull-ups and
  debounce caps at the button. 28-41% within 3 mm; 6-7 mm is normal.
- **Where a USB connector sits along its edge.** Centred 42%, near a corner 14%.
- **Crystals away from the edge, and an empty strip between crystal and chip.**
- **Regulators at their loads.** A 10-20 mm output rail is normal.
- **Extra keep-away between the feedback divider and the switch node**, or extra room
  around power parts for wide copper.
- **Entry protection, fiducials, mounting holes.** 5% of power inputs have a fuse or TVS,
  88% of boards have no fiducials, 48% have no mounting holes. Place what the board has.
- **One exception, where humans fall short of the guideline:** a third of on-board
  antennas have parts or vias under them. Keep the antenna area clear anyway.
