# KiCad Track Gloss

KiCad Track Gloss is an independent KiCad 10 ActionPlugin that shortens or
simplifies PCB track connections seeded by the straight segments explicitly
selected by the user. Each seed is automatically expanded to pads, vias, arcs,
locked tracks, or junctions; it does not blindly select the entire net.

The smoothing algorithm is derived from KiCadRoutingTools by DrAndyHaas. See
`NOTICE` and `LICENSE` for attribution and license terms.

## Install for testing

Build the PCM archive from the repository root:

```text
python kicad_track_gloss/package_pcm.py
```

The archive is written to `dist/KiCadTrackGloss-0.3.6.zip`. It can be tested
through a custom KiCad PCM repository, or the `kicad_track_gloss` folder can be
copied directly into KiCad's scripting plugins directory during development.

The PCM archive intentionally places `__init__.py` and the other Python modules
directly in its `plugins/` directory. KiCad does not load PCM Python plugins
that add another package-directory level below `plugins/`.

## Use

1. Open a board in PCB Editor.
2. Select one or more straight track segments as connection seeds.
3. Run **Tools → External Plugins → KiCad Track Gloss**.
4. The best deterministic gloss is applied directly to the current board.

Each selected segment is automatically expanded through KiCad's native
connectivity to the rest of its connection, stopping at pads, vias, arcs,
locked tracks, and junctions. Multiple seeds — including seeds on different
nets — are expanded independently, deduplicated, and optimized as one
deterministic batch. The visible KiCad selection does not need to be expanded
manually first.

A connection endpoint or intermediate vertex that lands on the middle of an
immutable same-net track is treated as a sliding termination. The gloss chain
is split at that T intersection, but the contact may move along the traversing
segment. Deterministic candidates include the nearest projection and direct
0/45/90-degree contacts; the traversing track remains unchanged.

Probable length-tuning meanders are protected only when the direction-reversal
pattern repeats. A single ordinary A/B/-A routing turn is not classified as a
meander and therefore does not suppress the whole connection.

There is no dialog, preview, temporary board, subprocess, success message, or
no-op message. If no safe improvement exists, the action simply returns. Use
KiCad **Undo** if the visual result is not desired.

For troubleshooting, run **Tools → External Plugins → KiCad Track Gloss —
Diagnostic**. It performs the same operation and then opens a selectable log
showing the detected selection, protected objects, number of candidate plans,
length saving, and the reason for a no-op. The normal action remains silent;
only an unexpected internal error opens the diagnostic report automatically.

The optimizer reads KiCad's board minimum clearance, copper-to-edge setting,
effective aggregate netclasses, and pad-local clearance through `pcbnew`.
It generates deterministic global and greedy alternatives and immediately
applies the highest-saving internally valid alternative.

KiCad's internal C++ `PNS::OPTIMIZER` is not exposed by the public SWIG or IPC
plugin APIs in KiCad 10, so this package does not pretend to call it. The
engine/adapter boundary is intentionally kept narrow so a future official PNS
IPC endpoint can replace the Python candidate generator without changing
selection scoping, attribution, or the one-click workflow.

## Safety boundaries

The first release treats the following as immutable: copper outside the
automatically expanded connections, locked tracks, arcs, vias, probable
differential-pair nets, pads, layer/width changes, junctions, and keepouts.
All immutable copper remains present in the geometry and connectivity model.

The plugin uses the same `Add()` / `RemoveNative()` ActionPlugin pattern as
KiCad's official Undo/Redo example. It also performs in-memory rollback if an
exception occurs while applying the plan.
