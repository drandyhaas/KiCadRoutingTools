# Architecture

KiCad Track Gloss separates pure optimization from KiCad integration so the
same algorithm serves the ActionPlugin, diagnostic action, CLI, and tests.

## Main flow

```text
selected KiCad objects
        |
        v
selection expansion and BoardModel snapshot
        |
        v
API-neutral fixed-point candidate planning
        |
        v
composed plan and internal validation
        |
        v
private KiCad native DRC gate
        |
        v
single live-board Undo transaction
```

The normal and diagnostic actions differ only in reporting. CLI scope `ALL`
uses the same engine and convergence function as a complete plugin selection;
only interactive/offline budgets differ.

Two performance contracts are intentionally distinct. A one-net gloss follows
the lowest-latency path available while retaining its configured safety gate.
A multi-net gloss is an anytime optimization: it uses the configured maximum
budget to improve quality, keeps the best native-DRC-approved subset reached
so far, and returns that subset when time expires. Fast no-op completion is not
a valid substitute for already discovered safe work.

## Engine packages

- `engine/model.py`: immutable board records and edit plans.
- `engine/geometry.py`: dependency-free geometry kernels.
- `engine/context.py`: reusable spatial indexes, identities, rule envelopes,
  pad rotations, and Edge.Cuts queries.
- `engine/candidate_geometry.py`: exact local identity, clearance, pad,
  keepout, mask, and edge validation.
- `engine/local_operators.py`: corridor-preserving corner chamfers and
  interior-segment translations, independent of convergence orchestration.
- `engine/terminals.py`: sliding same-net T termination analysis.
- `engine/pads.py`: pad copper containment and bounded contact candidates.
- `engine/planner.py`: chain discovery, octolinear generation, scheduling,
  alternative composition, and fixed-point convergence.
- `engine/workflow.py`: shared plugin/CLI candidate ladder and conservative
  fallback policy.
- `engine/parallel.py`: deterministic worker orchestration with sequential
  fallback.
- `engine/validation.py`: immutable pre-apply invariants and connectivity.
- `engine/statistics.py`: transformation classification and aggregate metrics.

The engine imports neither `pcbnew` nor wx. Worker payloads consist only of
serializable model records and configuration values.

## KiCad adapter

- `kicad/reader.py`: converts the board and selection to the pure model.
- `kicad/selection.py`: native connectivity expansion.
- `kicad/authority.py`: strict KiCad 10 locked/group/generator/differential-
  pair edit authority, without geometric intent heuristics.
- `kicad/rules.py`: semantic layers, rule values, keepouts, board boundaries,
  pads, vias, and masks.
- `kicad/writer.py`: identity-checked live application and rollback.
- `kicad/native_validation.py`: temporary snapshots, refill, hidden helpers,
  DRC orchestration, caching, and timeout handling.
- `kicad/drc_report.py`: pure KiCad JSON report normalization and comparison.
- `kicad/diagnostics.py`: shared human and machine metrics.
- `kicad/settings_dialog.py` and `kicad/report_dialog.py`: session settings and
  diagnostic UI.
- `kicad/adapter.py`: narrow public facade used by actions and CLI.

`action_plugin.py` owns the one-click lifecycle and delayed busy cursor but
does not contain optimization geometry. `configuration.py` validates packaged
defaults and maintains process-local session overrides. `version.py` is the
single source of version truth for UI and packaging.

## Determinism and batching

Selected seeds are expanded independently, deduplicated, and grouped by net,
layer, and compatible geometry. Candidate ranking uses saved length, segment
reduction, and a stable geometry signature. Independent parallel results are
sorted before composition. Selection order, net order, file object order, and
worker completion order must not affect the result.

For small local scopes, bounded branch alternatives keep a larger selection
from suppressing a better valid sub-scope transformation. Larger scopes use
the global scheduling algorithm to bound runtime. The planner follows newly
opened simplifications to a fixed point and composes changed passes against the
original model, so the live board receives one atomic edit plan.

Every selected seed expansion is also retained as a distinct local-connection
scope. Multi-connection planning rebuilds these scopes through the exact same
path used by a one-segment selection, composes every mutually compatible local
optimum, and ranks that composition alongside the global converged plan. A
larger selection therefore cannot silently replace a better local result with
a lower-quality plan.

After a native DRC rejection, up to three connection-local candidates are
probed in one parallel native wave until a safe base exists. Later DRC waves
validate the complete remaining batch and complementary partial extensions.
Every accepted extension
immediately becomes the retained result, so expiration returns useful work
without treating a complete net as the smallest recoverable unit.

## KiCad API boundary

KiCad 10 does not expose the internal C++ `PNS::OPTIMIZER` through public SWIG
or IPC plugin APIs. Track Gloss therefore obtains board objects, evaluated rule
data, connectivity, semantic layers, and mutations from `pcbnew`, while its
candidate generator remains Python and API-neutral. The narrow boundary makes
it possible to substitute an official optimizer API later without rewriting
selection, reporting, CLI, or packaging.

## Provenance

The standalone plugin is inspired by and reuses part of DrAndyHaas's
KiCadRoutingTools code, algorithms, and implementation patterns. The original
MIT notices are retained. Standalone integration and subsequent modifications
were produced with ChatGPT/Codex at the project owner's direction, and the
project is maintained by Frantz. See `kicad_track_gloss/NOTICE` for the formal
notice.
