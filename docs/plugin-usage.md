# Using the KiCad Track Gloss plugin

KiCad Track Gloss shortens and simplifies existing PCB tracks directly in
KiCad PCB Editor. It is designed for a one-click workflow: select copper, run
the action, inspect the result, and use KiCad **Undo** if you prefer the
original route.

## Normal gloss

1. Select at least one straight track segment. Seeds may belong to different
   connections or nets.
2. Run **KiCad Track Gloss** from the toolbar or from
   **Tools > External Plugins**.
3. The plugin expands every selected seed to its eligible connection, plans a
   deterministic batch of 0/45/90-degree improvements, validates the composed
   result, and applies it as one Undo operation.

Successful runs are silent. The plugin does not save the board, create
before/after copies, display a preview, or ask for confirmation. When no safe
change is available, KiCad plays its standard warning sound once.

The selection may also contain footprints, text, drawings, vias, arcs, or
other objects. These objects are counted for diagnostics but are not treated
as modifiable straight-track seeds. Selected vias and protected copper remain
unchanged.

## Automatic connection expansion

A selected straight segment is a seed, not necessarily the complete route.
The plugin follows its same-net connection so that users do not need to run
KiCad's **Select/Expand Connection** command first. Multiple disconnected
seeds and multiple nets are expanded and processed in one deterministic batch.

Expansion stops or protects geometry at relevant pads, vias, arcs, locked
tracks, junctions, probable differential pairs, and probable length-tuning
meanders. Selection remains an authorization boundary: copper outside the
eligible expanded scope is immutable.

## Session settings

Run either Track Gloss action with no straight track selected to open the
session settings dialog. The dialog title includes the plugin version.

- **Use native KiCad DRC for a one-track selection** enables the strongest
  single-connection safety gate. Disabling it makes ordinary one-track glosses
  substantially more responsive, but removes that native before/after check.
- **Minimum saved length** rejects changes that save less than the configured
  amount. Its default is 0.1 mm and it is edited in 0.1 mm increments.
- **Total time budget**, **planning time budget**, and **cancellation grace**
  bound interactive work and prevent the editor from appearing indefinitely
  blocked.

Hover over a field to see its effect. **Close** applies edited values in memory
until KiCad exits. **Cancel** leaves the current session values unchanged. The
packaged JSON defaults are never rewritten.

## Diagnostic action

**KiCad Track Gloss - Diagnostic** runs the same optimizer and validation as
the normal action, then opens a report with three tabs:

- **Result**: outcome, saved length, copper before/after, segment reduction,
  plugin version, and KiCad version;
- **Details**: expanded scope, protected geometry, convergence, mechanisms,
  rejected candidates, blocking nets, native DRC status, and timings;
- **JSON**: the machine-readable result without the human report.

**Copy tab** copies the visible tab. **Copy all** copies the complete report.
The former success footer about applying the board and using Undo is omitted;
the saved-length result is the prominent outcome.

## Responsiveness and the busy cursor

Calculations completing in less than three seconds do not change the cursor.
If planning is still active after three seconds, the editor displays its busy
cursor until planning completes or stops. Only the API-neutral planner may run
off the main thread. All `pcbnew` reads and live-board changes remain on KiCad's
main thread.

KiCad's native DRC runs in separate hidden processes on private board copies.
Process startup, zone refill, and full-board DRC can take seconds even when the
geometric planning itself takes only milliseconds. See
[Safety and native DRC](safety-and-drc.md).

## Protected and unsupported operations

Track Gloss does not move footprints, pads, vias, arcs, zones, tuned meanders,
differential pairs, or locked tracks. It does not invoke KiCad's interactive
router or its **Cleanup Tracks and Vias** dialog. KiCad 10 does not expose its
internal C++ PNS optimizer through the public Python API, so candidate planning
is performed by the plugin's independent engine using constraints obtained
from `pcbnew`.
