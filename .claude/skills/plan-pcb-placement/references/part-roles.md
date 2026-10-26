# Part roles: what gets misread, and how to tell

Read this before you decide what a part IS, and above all before you sort parts into
functional groups. A part's reference prefix, footprint name and pin names are hints,
not facts. Each entry gives what the part looks like, what it often is instead, how to
tell the two apart, and which group it belongs in. Where a real board showed the
problem, the board is named; the boards are in the stress corpus.

When a run misreads a part, add an entry in the same shape: what it looked like, what
it was, how to tell, and the board.

**The order of evidence, strongest first:**
1. the part's datasheet, looked up by its value or part number;
2. the nets on its pins, meaning what the part is wired TO;
3. pin functions, unless they just repeat the pad number;
4. the footprint name and the reference prefix.

## Power parts

- **Pins named IN and OUT do not make a regulator.** Amplifiers, LNAs, buffers and
  level shifters use the same names. On hackrf_one, an MGA-81563 RF amplifier has
  exactly GND / IN / OUT.
  - *Tell:* a regulator's OUT net is a supply rail that feeds other parts' supply
    pins. An amplifier's OUT net is a signal, often named `Net-(C62-Pad2)`.
  - *Group:* the signal chain.
- **Regulator direction when the pins carry no function.** The higher-voltage net is
  the input: `+5V` → `+3V3`, and `+3V3` → `+1V8` (ice5lp4k_hat, an MCP1824). For a
  negative regulator (79xx, LT1964) the more negative net is the input. Do not take
  the net with more parts as the output: a 3V3 rail feeding a 1V8 LDO has more parts
  than the 1V8 rail.
- **EN tied to VIN renames the input net.** The input of hifi_dsp_board's TPS7A05 is
  named `Net-(U3-EN)`. Read the pin, not the net name.
- **Power-path parts that are not regulators still belong to the power group:**
  - fuses and polyfuses;
  - ferrite beads on the input;
  - reverse-polarity diodes and P-FETs;
  - load switches, eFuses and ideal-diode controllers;
  - battery protectors (DW01 plus a dual FET);
  - power muxes;
  - chargers (TP4056, MCP73831, BQ24xxx);
  - fuel gauges (MAX17048).

  A P-FET sitting between a power connector and the regulator is reverse-polarity
  protection or a load switch, not a signal transistor.
- **Some regulators belong to their LOAD, not to the power group:**
  - DDR termination (LP2996, VTT) goes with the DRAM;
  - a voltage reference (REF30xx, a TL431 used as a reference) goes with the ADC or
    comparator it feeds;
  - a point-of-load LDO whose output feeds one chip or one analog section goes with
    that chip.

  *Tell:* count the parts on its output net. One consumer means it belongs with that
  consumer.
- **Part-number substrings lie:**
  - DMP3013 (a P-MOSFET, on berkeley_obsidian) contains "MP3013", a buck family.
  - MAX17048 is a fuel gauge, while many MAX17xxx parts are bucks.

  Match part numbers from their start, and check the datasheet when a family prefix
  is shared.
- **Package names do not decide a role.** SOT-89, SOT-223 and TO-252 carry regulators
  AND transistors (an ME6210 and a BCX56 share SOT-89). TO-92 carries a 78L05 and a
  BC547. SOT-23 carries LDOs, transistors and ESD diodes. `D_SMA` is the diode
  package "SMA", not an SMA RF connector (tx_band_splitter D201, an LL4002).

## Clock parts

- **Crystal load caps are clock parts, not decaps.** A 2-pad cap from an oscillator
  net to GND serves the crystal, though by its nets alone it looks like the chip's
  decap (ch32v003_usb's C4, 20 pF on PA2, beside U3). The toolchain's decap rule
  does not count it: an oscillator net is no supply rail.
  - *Group:* with the crystal, the caps symmetric at the crystal's pads, and the
    crystal on the IC's oscillator face. `place_simple` writes that as `proximity[]`
    claims (`py_placer/placement/crystals.py`): the crystal at its IC's oscillator
    pins (link by link through a series resistor on a leg), each cap at the
    crystal, all in the IC's group.
- **A 4-pin crystal is not a 4-pin oscillator.** A 2016 or 3225 crystal has two GND
  pins. An oscillator has a supply pin, and the cap on that pin is the oscillator's
  decap.
- **Jumpers and 0 Ω on oscillator pins are bridges.** On ch32v003_usb, JP1/JP2 route
  PA1/PA2 to the header so the pins can be GPIO instead.
  - *Group:* clock, at the crystal pins, facing the header pin they reach. Make the
    crystal the clock group's `anchor`: `place_simple` moves the crystal into its
    IC's group, and the rest of the clock group follows only when the crystal is
    its anchor.

## Caps and resistors whose role is their nets

- **An RC reset cap is not a decap**, though 100 nF from NRST to ground reads like
  one (ch32v003_usb's C6 on PD7/NRST).
  - *Group:* the reset circuit, at the NRST pin or with the reset button.
- **Bulk caps (10 µF and up, electrolytic or tantalum)** belong at the regulator or
  the power entry, not at an IC's supply pin.
- **Pull-ups and pull-downs go at the end they define:**
  - USB-C CC 5.1 kΩ pull-downs: at the connector;
  - boot and strap resistors: at the MCU pin;
  - I2C pull-ups: on the bus, usually near the controller;
  - a USB D- 1.5 kΩ pull-up switched by a GPIO (software USB, ch32v003_usb R1): with
    the MCU, at that pin.
- **Series resistors (22-33 Ω on USB, clocks, SPI) are bridges between two groups.**
  Source termination goes at the driver. For USB series resistors, follow the MCU's
  datasheet (usually at the MCU). Assign each to one group and record which group it
  borders.
- **An LED and its series resistor are one unit.** Humans keep them about 1.8 mm apart
  (median). Indicator LEDs usually go in a row along an edge.
- **A regulator's feedback divider** goes at its FB pin, in the power group, away from
  the inductor and the switch node.
- **A ferrite bead between rails (VDD → VDDA)** goes with the analog consumer.

## Connectors, mechanical parts, user interface

- **The pin headers on a DIP-style module board are mechanical.** Two full-length 1×N
  rows (ch32v003_usb J2/J3) are its legs. Place and lock them first. Which end is pin
  1 is the module's pinout, so take it from the brief or the user.
- **A header can also be a power entry.** ch32v003_usb's J2 carries VBUS on pin 1
  beside the USB-C receptacle. Put the power group by the connector's POWER pins and
  the signal group by its signal pins.
- **Test points** (TP*, or a "TestPoint_01x02" referenced J4 on ch32v003_usb, which
  has no courtyard) go with the net they probe. They need no group of their own.
- **Buttons.** The part classifier calls any footprint named switch, button or tact
  `edge_actuator`, by name alone, and that class never calls an interior pose
  wrong. A top-actuated
  tact switch (Omron B3U-1000P) can sit anywhere; a side-actuated one (B3U-3000P)
  belongs at the edge. Check the datasheet before you give a button an edge.
- **Mounting holes, fiducials and logos** are in no functional group. Holes and
  fiducials are mechanical: declare their poses in the intent's
  `fixed_poses[]` (from `mechanical.json` or the brief), which seats and holds
  them with no lock -- and leave them unlocked, since a KiCad lock voids the
  marker waiver on their courtyard pairs (SKILL.md §3.1). Place and lock
  first only a hole or fiducial no declaration names: an unlocked part with a
  copper pad must be in a group or seated by `fixed_poses[]`, or
  `check_groups --intent` reports an ERROR. A pad-less logo is never placed.
- **A wireless module** (an ESP32 module, a Pico) is the core. Its antenna end needs
  its keep-out at the edge.

## Tool output that misleads grouping

- **Rails dominate shared-net counts.** On a small board, GND and the main rail touch
  almost every part (ch32v003_usb: GND reaches 16 parts and +3V3 8). Judge which
  group a part belongs to by its SIGNAL nets, and ignore ground and supply rails.
- **Sheet groups can be empty of meaning.** Some boards carry one sheet path per
  symbol (ch32v003_usb: 25 paths for 25 parts), so grouping by sheet says nothing
  there. Check the count before trusting it. `board_context`'s `## Schematic`
  names the sheets and how many parts each holds, and says how many parts
  matched their symbols: a low count means the schematic and the board disagree.
- **A pin function equal to the pad number** ("1", "2") carries no information.
- **The decap rule reads nets, not roles.** `check_groups`, `place_simple` and the
  grade count as a decap any 2-net cap from ground to a rail some chip (not a
  connector) has a supply pin on, or across two of one chip's supply rails. A load
  or RC cap on a signal pin is not one; a bulk, input or filter cap on such a rail
  is, and `place_simple` pulls it to the supply pins of its group's chips on that
  rail. Put each cap in its real group and say why in the group's reason: a decap
  whose chips are all in other groups is left out of the decap term, and
  `check_groups` WARNs it.

## Where humans put each group

The priors for where each group sits on the board (power at its entry, the main IC
central, ESD at its connector, LEDs and buttons toward the edge), with the numbers
and the commands that reproduce them, are in `placement-lessons.md`, section
"Functional groups". That page also has the spacing, orientation and arrangement
inside each group.
