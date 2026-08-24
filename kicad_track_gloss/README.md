# KiCad Track Gloss

KiCad Track Gloss is an independent KiCad 10 ActionPlugin for shortening and
simplifying existing PCB tracks. It operates directly on straight track
segments selected in PCB Editor and produces deterministic 0/45/90-degree
copper while preserving connectivity, geometry constraints, and protected
board objects.

The intended workflow is a single click followed, if necessary, by KiCad
**Undo**. The plugin does not save the board, create before/after files, show a
preview, or ask the user to confirm a result.

## Code provenance — important

**This plugin is directly derived from the work and source code of
DrAndyHaas.** It is not an independently invented smoothing engine. It contains
reused and adapted code, algorithms, and implementation patterns from
[DrAndyHaas/KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools),
including track-chain recognition, octolinear 0/45/90-degree smoothing,
shortcut evaluation, improvement gating, geometry checks, and connectivity
preservation. The original MIT copyright and license are preserved in
`LICENSE`.

**The standalone-plugin modifications in this branch were created with
ChatGPT/Codex (OpenAI), at the project owner's direction.** This includes the
selection-seeded KiCad 10 ActionPlugin integration, automatic connection
expansion, deterministic batch planning, sliding T terminations, additional
safety validation, diagnostics, warning-bell behavior, regression fixtures,
tests, documentation, branding, and PCM packaging. Those modifications build
on and do not replace the attribution owed to DrAndyHaas for the underlying
work and reused code.

The active project repository is the fca1 fork:
<https://github.com/fca1/KiCadRoutingTools>.

## User-visible contract

1. The user selects one or more straight track segments. Seeds may belong to
   different connections or nets.
2. **KiCad Track Gloss** expands every seed through KiCad's native connectivity
   up to a pad, via, arc, locked segment, or junction.
3. The optimizer evaluates the expanded connections as one deterministic batch
   and applies the highest-saving safe candidate to the current board.
4. A successful operation returns silently. When no copper is changed for any
   reason, KiCad's standard warning bell is played exactly once.
5. **KiCad Track Gloss — Diagnostic** runs the same operation and displays a
   selectable report containing selection counts, protections, candidates,
   length saving, and no-op or error details.

Unexpected internal errors display an error report; failures during plan
application request an in-memory rollback. Normal no-op conditions never open
a message window.

## Optimization model

Selected seeds are expanded independently, deduplicated, and converted into an
API-neutral `BoardModel`. Candidate chains are grouped by net, copper layer,
and width. The engine explores deterministic octolinear shortcuts and uses
weighted interval scheduling across each chain. Multi-net candidates are
ranked by saved length, then by segment reduction and a stable geometry
signature. Isolated group plans remain available when a combined candidate is
not valid.

An eligible chain that terminates in the middle of an immutable same-net track
has a sliding T termination. The contact may move along that traversing track
to the nearest useful 0/45/90-degree location; the traversing track itself is
not changed. This models the useful effect of manually breaking the last
segment and finalizing it again in KiCad.

An eligible chain ending in same-net pad copper has a sliding pad termination.
The engine models circle, rectangle, oval, and rounded-rectangle pad areas,
including size, rotation, copper layers, and rounded corners. It evaluates a
bounded deterministic set of 0/45/90-degree contacts on the pad boundary. Pad
copper remains immutable; only the connected track endpoint moves within it.

Repeated direction reversals and long runs dominated by dense micro-jogs are
classified as probable length-tuning geometry. Detection is performed per
independent unbranched component so tuning cannot protect an unrelated route
on the same net. A single ordinary `A/B/-A` turn is not sufficient to classify
a route as tuned.

The engine is deterministic with respect to selection order, track order, and
net order. Determinism is regression-tested with original, reversed,
ascending-net, descending-net, and shuffled board inputs.

## KiCad integration and constraints

The KiCad-facing adapter and ActionPlugin use `pcbnew` for:

- native connectivity expansion;
- board minimum clearance and copper-to-edge clearance;
- effective netclass and pad-local clearances;
- pads, vias, tracks, layers, rule-area keepouts, and board bounds;
- live-board `Add()` and `RemoveNative()` operations;
- refresh and modified-state notification.

KiCad 10 does not expose the internal C++ `PNS::OPTIMIZER` through the public
SWIG or IPC plugin APIs. The candidate generator is therefore implemented in
Python and consumes the constraints that `pcbnew` exposes. The adapter/engine
boundary is deliberately narrow so an official PNS API could replace the
candidate generator without changing selection handling or the user workflow.

The following remain immutable or protected:

- copper outside the expanded eligible scope;
- pads, vias, arcs, and locked tracks;
- probable differential-pair nets and probable tuned meanders;
- net identity, layer, and track width;
- traversing tracks used by sliding T terminations;
- board edges, foreign-net copper, pads, vias, and track keepouts.

Before live mutation, validation rejects unknown or duplicate removals,
unselected removals, missing replacement copper, net/layer/width changes,
inter-net clearance violations, and degraded connectivity between immutable
terminals. Application verifies track identities again and restores removed
copper if an exception occurs.

Pad clearance uses conservative enclosing circles when KiCad does not expose a
more precise shape through the adapter. A candidate portion that is strictly a
retained subsegment of removed copper is not treated as new copper against that
coarse approximation; every genuinely new portion remains fully checked.
Identity replacements are retained as their original native KiCad items.
Rectangular pad obstacles use the circumscribed bounding-box radius so their
corners cannot be mistaken for free space. Same-net pad connectivity is
validated against the modeled pad shape rather than that coarse obstacle.

## Code map

- `__init__.py`: registers the normal and diagnostic ActionPlugins in KiCad.
- `action_plugin.py`: one-click orchestration, reporting, warning bell, and UI
  contract.
- `version.py`: single source of truth for the plugin version displayed by the
  diagnostic and used by the PCM builder.
- `kicad/adapter.py`: small public `BoardAdapter` facade.
- `kicad/reader.py`: live-board and selection conversion to `BoardModel`.
- `kicad/selection.py`: native connection expansion, differential-pair and
  meander protection.
- `kicad/rules.py`: board bounds, effective netclasses, and track keepouts.
- `kicad/writer.py`: live-board plan application and rollback.
- `engine/planner.py`: API-neutral chain discovery, octolinear candidate
  generation, global scheduling, and batch fallbacks.
- `engine/terminals.py`: detection and movement candidates for sliding T
  terminations.
- `engine/validation.py`: immutable pre-apply safety and connectivity checks.
- `engine/geometry.py`: dependency-free geometry kernels.
- `engine/model.py`: immutable geometry records and edit plans.
- `package_pcm.py`: builds the standalone KiCad PCM archive.
- `metadata.json`: PCM identity, compatibility, and release version.
- `../tools/diagnose_track_gloss_board.py`: headless inspection and sweep tool
  for real KiCad boards; `--apply-in-memory` never saves the board.
- `../tests/track_gloss/unit/`: engine, expansion, T-junction, packaging, and
  user-contract unit tests.
- `../tests/track_gloss/run_patterns.py`: KiCad-Python integration replay.
- `../tests/track_gloss/patterns/`: frozen real-board regression inputs.

## Reference regression data

The complete KiCad board, project, and design rules are stored byte-for-byte in
`tests/track_gloss/patterns/dispenser_labels/`. Its README contains the
SHA-256 fingerprints. Tests must load this fixture in memory and must never save
over it.

Current required integration results are:

- **All 706 straight tracks selected in one batch:** 49.680420 mm saved and 18
  net segments removed (116 removed, 98 added). 116 probable tuned segments
  are protected and 590 segments are eligible. Seven input orders must produce
  the exact same complete plan.
- **Connections evaluated independently:** 334 unique scopes, 138 improving
  plans, 88.878551 mm total isolated potential, and 138 successful applications
  to freshly loaded in-memory boards.
- **Dense micro-jog regression (`/cpu/~{csn}`):** selecting UUID
  `58ebb541-fac6-4d02-8a68-65aca50766b5` expands to 111 tuned segments; all
  111 must be protected and no geometric candidate may be planned.
- **Short VCC regression:** selecting UUID
  `cc798608-5e9b-4c2a-9856-dde85f9d85f0` must save 0.959686 mm while retaining
  safe existing copper and moving eligible contacts inside pad areas.
- **Pad-area regression (`Net-(U1-BST)`):** selecting UUID
  `54640123-2d45-4136-984c-783155178230` must replace its 3.535534 mm segment
  with one 2.938736 mm diagonal between the two rounded-rectangle pad areas,
  saving 0.596798 mm.

The isolated total is not the expected result of the simultaneous all-track
batch. T-junction mobility and clearance interactions differ when surrounding
tracks are eligible at the same time. Treat both figures as separate
non-regression contracts; neither is a proof of a mathematical global maximum.

The all-track plan is also applied to a temporary board copy and checked with
KiCad 10.0.5's exhaustive native DRC. The frozen fixture reports 170 existing
violations and one existing unconnected item both before and after gloss, with
identical violation-category counts.

## Running validation

From the repository root, run the API-neutral tests with a standard Python that
has `pytest`:

```text
py -3.12 -m pytest tests/track_gloss/unit tests/test_smooth_route.py -q
```

Run the real-board replay with KiCad's bundled Python so `pcbnew` is available:

```text
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py
```

The integration replay takes several minutes because it recomputes the full
all-track plan under seven input orders and applies every accepted pattern to a
fresh in-memory board.

## Building and installing

The release value in `version.py` and `metadata.json` must always match;
`package_pcm.py` imports it from `version.py`.
Build from the repository root:

```text
python kicad_track_gloss/package_pcm.py
```

The generated archive is `dist/KiCadTrackGloss-<version>.zip`. Its required PCM
layout is flat:

```text
metadata.json
resources/icon.png
plugins/__init__.py
plugins/action_plugin.py
plugins/engine/...
plugins/kicad/...
```

Do not add a `plugins/kicad_track_gloss/` directory level: KiCad will not load
the ActionPlugin from that PCM layout. For direct development, the complete
`kicad_track_gloss` directory may instead be copied into KiCad's scripting
plugins directory.

## Maintenance rules

- Preserve the silent-success, one-bell-on-no-op interaction contract.
- Never add automatic board saving, temporary PCB round-trips, or confirmation
  dialogs to the normal action.
- Keep selection expansion and meander filtering centralized in
  `kicad/selection.py`; expose it through `BoardAdapter` so the plugin,
  diagnostic tool, and regression replay cannot diverge.
- Keep the engine independent of `pcbnew`; KiCad-specific work belongs in the
  adapter or ActionPlugin layer.
- Preserve deterministic sorting and signatures whenever adding candidates.
- Update regression expectations only after inspecting and justifying the
  geometric change on the frozen board.
- Run both validation commands before building a PCM archive.

## Attribution and license

This package contains code reused and adapted from DrAndyHaas's
KiCadRoutingTools, plus standalone-plugin modifications generated with
ChatGPT/Codex at the project owner's direction. This provenance must remain
visible in redistributed source and packages. `NOTICE` gives the complete
statement and `LICENSE` preserves the original MIT terms.
