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

The archive is written to `dist/KiCadTrackGloss-0.2.0.zip`. It can be tested
through a custom KiCad PCM repository, or the `kicad_track_gloss` folder can be
copied directly into KiCad's scripting plugins directory during development.

## Use

1. Open a board in PCB Editor.
2. Select at least two connected straight track segments.
3. Run **Tools → External Plugins → KiCad Track Gloss**.
4. Choose **Shorten only** or **Shorten or simplify**.
5. Review the preview and apply.

With DRC validation enabled, the plugin saves temporary baseline and candidate
copies, refills zones with `kicad-cli`, and rejects the operation if the
candidate adds any official DRC violation. Existing violations are tolerated.

The optimizer reads KiCad's board minimum clearance, copper-to-edge setting,
effective aggregate netclasses, and pad-local clearance through `pcbnew`.
It generates deterministic global and greedy alternatives, then asks KiCad's
official DRC oracle to accept the highest-saving valid alternative. A single
baseline DRC is shared by all alternatives.

KiCad's internal C++ `PNS::OPTIMIZER` is not exposed by the public SWIG or IPC
plugin APIs in KiCad 10, so this package does not pretend to call it. The
engine/adapter boundary is intentionally kept narrow so a future official PNS
IPC endpoint can replace the Python candidate generator without changing
selection scoping, preview, attribution, or DRC comparison.

## Safety boundaries

The first release treats the following as immutable: unselected tracks, locked
tracks, arcs, vias, probable differential-pair nets, pads, layer/width changes,
junctions, and keepouts. Non-selected copper remains present in the geometry
and connectivity model. A single straight segment normally cannot be shortened
because both endpoints are fixed.

The SWIG API does not expose a reliable cross-version one-entry Undo transaction.
This release therefore validates fully before mutation and performs in-memory
rollback on any application exception. The architecture isolates `BoardAdapter`
so a future IPC adapter can use `begin_commit()` / `push_commit()` for one-step
Undo/Redo without rewriting the engine.
