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
   three-tab report. **Résultat** prominently states the saved length in mm and
   the before/after segment count, **Détails** contains selection, protection,
   transformation, rejection, and blocking-net information, and **JSON** keeps
   the machine-readable payload separate. **Copier l'onglet** copies only the
   visible tab; **Copier tout** copies the complete report for troubleshooting.

Unexpected internal errors display an error report; failures during plan
application request an in-memory rollback. Normal no-op conditions never open
a message window.

## Optimization model

Selected seeds are expanded independently, deduplicated, and converted into an
API-neutral `BoardModel`. Candidate chains are grouped by net and copper layer;
exact width values and their order remain attached to the replacement copper,
but a width-transition point may move during optimization. The engine explores
deterministic octolinear shortcuts and uses
weighted interval scheduling across each chain. Multi-net candidates are
ranked by saved length, then by segment reduction and a stable geometry
signature. Isolated group plans remain available when a combined candidate is
not valid.

A shared planner context precomputes track identities, maximum rule envelopes,
pad rotations, and conservative spatial indexes. Candidate clearance and
connectivity checks query only nearby geometry before running the same exact
kernels. Whole-board validations are retained for the chosen combined plan;
redundant leave-one-out connectivity replays are avoided after their component
plans have already passed the complete checks.

Selections of at least 64 eligible segments may distribute independent
net/layer and exact-width fallback searches over as many as four local Python
worker processes. Results are serialized without `pcbnew`, sorted
deterministically, and required to match sequential planning. If KiCad has no
safe sibling Python interpreter or a worker fails, planning automatically
continues sequentially. Small selections never start workers, avoiding process
startup latency during ordinary one-connection glosses.

Every changed path is octolinear. Removing a non-0/45/90-degree segment takes
priority over length reduction, so the engine may accept a small length increase
when that is required to normalize imported or manually drawn arbitrary-angle
copper. Among safe octolinear solutions it then maximizes saved length.

Optimization uses exact existing copper geometry and intersection coordinates,
not PCB Editor's active drawing grid. This preserves connections to tracks and
pads that may themselves be off-grid. The diagnostic states this explicitly;
grid snapping must not be added as a hard constraint without checking that a
snapped terminal remains on its target copper.

For bounded selections, the best combined plan is replayed in the API-neutral
model for additional convergence passes. Newly created same-net centreline
intersections become T contacts. A generated free tail is removed when it
terminates on wider through-copper, or when a narrower track terminates exactly
at its T contact; copper-area checks protect nearby useful continuations. The
result is optimized again. Every composed result is revalidated against the
original selection. Very large scopes skip this optional pass to keep runtime
bounded.

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
- KiCad's evaluated per-track clearance for the current layer (including
  matching custom `.kicad_dru` rules), with netclass fallback, plus pad-local
  clearances;
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

Clearance search distinguishes genuinely new copper from a candidate portion
already fully contained inside a wider immutable track of the same net. A
pre-existing nearby foreign-net condition is not counted as a new violation
when the replacement adds no copper to that clearance envelope; any part that
extends beyond the same-net cover remains subject to the full effective rule.

Pad clearance uses the actual rotated circle, rectangle, oval, or rounded-
rectangle copper shape. Paste/mask-only apertures are ignored. Unsupported and
custom pads retain a conservative enclosing circle built from KiCad's effective
bounding box, including all custom primitives. A candidate portion that is
strictly retained original copper is not treated as new copper against a
fallback approximation; every genuinely new portion remains checked. Identity
replacements are retained as their original native KiCad items.

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
- `kicad/rules.py`: board bounds, netclass fallbacks, and track keepouts.
- `kicad/writer.py`: live-board plan application and rollback.
- `kicad/diagnostics.py`: human-readable tables and machine-readable JSON for
  diagnostic runs.
- `engine/planner.py`: API-neutral chain discovery, octolinear candidate
  generation, global scheduling, batch fallbacks, and bounded convergence.
- `engine/context.py`: reusable spatial indexes and rule envelopes.
- `engine/parallel.py`: deterministic local worker orchestration and sequential
  fallback.
- `engine/terminals.py`: detection and movement candidates for sliding T
  terminations.
- `engine/pads.py`: pad containment and bounded copper-contact geometry.
- `engine/statistics.py`: transformation classification and aggregate metrics.
- `engine/validation.py`: immutable pre-apply safety and connectivity checks.
- `engine/geometry.py`: dependency-free geometry kernels.
- `engine/model.py`: immutable geometry records and edit plans.
- `package_pcm.py`: builds the standalone KiCad PCM archive.
- `metadata.json`: PCM identity, compatibility, and release version.
- `../tools/score_track_gloss.py`: read-only whole-board score CLI compatible
  with `place_route_loop --accept-cmd`.
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

- **All 706 straight tracks selected in one batch:** 62.419635 mm saved, 18 net
  segments removed (211 removed, 193 added), and 39 arbitrary-angle segments
  normalized. 116 probable tuned segments
  are protected and 590 segments are eligible. Seven input orders must produce
  the exact same complete plan.
- **Connections evaluated independently (optional deep sweep):** every unique
  connection scope is generated and every changed plan is applied to a freshly
  loaded in-memory board. This costly inventory is suspended during routine
  release builds and is enabled explicitly with `--full-sweep`.
- **Dense micro-jog regression (`/cpu/~{csn}`):** selecting UUID
  `58ebb541-fac6-4d02-8a68-65aca50766b5` expands to 111 tuned segments; all
  111 must be protected and no geometric candidate may be planned.
- **Short VCC regression:** selecting UUID
  `cc798608-5e9b-4c2a-9856-dde85f9d85f0` must save 1.003620 mm while retaining
  safe existing copper and moving eligible contacts inside pad areas.
- **Reported clearance/convergence regressions:** the `/cpu/SW_PULL` paste-pad
  case saves 1.453743 mm; the GND horizontal at `Y=108.2` moves to `Y=108.3`
  and saves 0.714108 mm; the mixed-width GND batch replaces its diagonal chain
  and trailing stub with a horizontal T contact, saving 2.856996 mm.
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
py -3.12 -m pytest tests/track_gloss/unit -q
```

Run the real-board replay with KiCad's bundled Python so `pcbnew` is available:

```text
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py
```

The routine integration replay evaluates the all-selected board once. Two
costly deep checks are retained but suspended by default:

```text
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py --all-orders
D:\kicad\bin\python.exe tests\track_gloss\run_patterns.py --full-sweep
```

The first compares all seven deterministic input orders. The second generates
all connection scopes and applies every changed plan to fresh in-memory boards.
Combine the flags only when a complete deep validation is needed.

## Headless score CLI and `place_route_loop`

`tools/score_track_gloss.py` grades a complete route without changing or
saving the input board. It treats every unlocked, non-differential straight
track as selected in one simultaneous batch; connection expansion and tuned
track protection are the same as in the ActionPlugin. Safe all-track passes
are applied to the in-memory board until a geometric fixed point is reached.
The process aborts on a repeated geometry or after 16 changed passes instead
of returning a falsely converged score. It prints detailed compact JSON
followed by the protocol line required by `place_route_loop`:

```text
GLOSS_SCORE_JSON={...}
SCORE=1046.286691506
```

The score is the total straight-track copper length, in millimetres, after the
best safe virtual all-track gloss. Lower is better. The JSON also reports the
original copper length, total saved length and percentage, segment gain,
convergence-pass count, eligible count, and protected tuned-track count. Using the virtual post-gloss
length prevents a needlessly untidy but easily glossable route from being
rewarded merely because it contains more removable copper.

Run it with KiCad's Python interpreter so `pcbnew` is available. A project path
may be passed directly; the matching `.kicad_pcb` is then inferred:

```powershell
D:\kicad\bin\python.exe tools\score_track_gloss.py design.kicad_pro
D:\kicad\bin\python.exe tools\score_track_gloss.py --project design.kicad_pro candidate.kicad_pcb
D:\kicad\bin\python.exe tools\score_track_gloss.py --project design.kicad_pro --output glossed.kicad_pcb candidate.kicad_pcb
```

The second form is intended for generated boards whose filename differs from
the original project. The CLI makes a temporary same-stem copy of the board,
`.kicad_pro`, and sibling `.kicad_dru`, allowing KiCad to evaluate the original
project rules without touching any source file.

`--output` is the only mode that writes copper. It saves the already converged
in-memory result to a different `.kicad_pcb` and refuses to overwrite either
the input or an existing output unless `--force` is explicit. Re-running the
CLI on that output must report zero changed passes, zero saved length, and the
same score: A0 → A1 is therefore a fixed point and A1 → A2 leaves the route
geometry unchanged. Byte-for-byte identity is not promised because KiCad may
reorder records or regenerate identifiers when serializing a board.

For `place_route_loop`, use the scorer as the acceptance command and add
`--place-route-loop`. The loop appends its placed PCB, routed PCB, and
`route.json`; the scorer deliberately grades the second (routed) PCB:

```powershell
python py_placer\place_route_loop.py <its normal arguments> `
  --accept-cmd 'D:/kicad/bin/python.exe tools/score_track_gloss.py --place-route-loop --project design.kicad_pro'
```

`place_route_loop` accepts a candidate only when its numeric score is strictly
lower. This score is a routing-quality comparator, not a validity certificate:
it does not replace DRC, connectivity, placement, assembly, or project-specific
specification gates. Keep those independent checks in the route loop.
`--output` is forbidden in this mode so the acceptance judge cannot mutate a
candidate owned by the loop.

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
- Keep diagnostic collection optional. The normal one-click action must not
  pay the classification and aggregation cost, and both modes must generate
  the exact same edit plan.
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
