# KiCad Track Gloss

KiCad Track Gloss is an independent KiCad 10 ActionPlugin that shortens or
simplifies only the straight PCB track segments explicitly selected by the
user. It never expands a selection to an entire net.

The smoothing algorithm is derived from KiCadRoutingTools by DrAndyHaas. See
`NOTICE` and `LICENSE` for attribution and license terms.

## Install for testing

Build the PCM archive from the repository root:

```text
python kicad_track_gloss/package_pcm.py
```

The archive is written to `dist/KiCadTrackGloss-0.3.2.zip`. It can be tested
through a custom KiCad PCM repository, or the `kicad_track_gloss` folder can be
copied directly into KiCad's scripting plugins directory during development.

The PCM archive intentionally places `__init__.py` and the other Python modules
directly in its `plugins/` directory. KiCad does not load PCM Python plugins
that add another package-directory level below `plugins/`.

## Use

1. Open a board in PCB Editor.
2. Select at least two connected straight track segments.
3. Run **Tools → External Plugins → KiCad Track Gloss**.
4. The best deterministic gloss is applied directly to the current board.

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

The first release treats the following as immutable: unselected tracks, locked
tracks, arcs, vias, probable differential-pair nets, pads, layer/width changes,
junctions, and keepouts. Non-selected copper remains present in the geometry
and connectivity model. A single straight segment normally cannot be shortened
because both endpoints are fixed.

The plugin uses the same `Add()` / `RemoveNative()` ActionPlugin pattern as
KiCad's official Undo/Redo example. It also performs in-memory rollback if an
exception occurs while applying the plan.
