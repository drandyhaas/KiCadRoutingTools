# Configuration reference

Track Gloss has no persistent user preferences. Validated defaults are stored
in `kicad_track_gloss/internal_config.json`; the no-selection dialog creates an
in-memory session override and never writes the file.

## Packaged policy

| JSON path | Default | Unit/range | Scope | Effect |
|---|---:|---|---|---|
| `gloss.minimum_saved_length_mm` | `0.1` | mm, non-negative | Plugin and CLI | Rejects a length-only change below this saving. Angle normalization and permitted equal-length simplification retain their own explicit rules. |
| `convergence.interactive_max_passes` | `4` | integer >= 1 | Plugin, internal | Maximum global reconciliation passes for an interactive run. |
| `convergence.interactive_group_max_passes` | `2` | integer >= 1 | Plugin, internal | Maximum local passes per independent group. |
| `convergence.cli_max_passes` | `16` | integer >= 1 | CLI | Default changed-pass convergence guard; overridden by `--max-passes`. |
| `timing.interactive_total_time_budget_seconds` | `20.0` | seconds > 0 | Plugin, session-editable | Bounds the complete interactive operation, including native validation. Long planning displays a cancellable progress dialog after three seconds. |
| `timing.interactive_planning_time_budget_seconds` | `10.0` | seconds > 0 | Plugin, session-editable | Bounds candidate planning before native DRC. It cannot exceed the total budget. |
| `timing.interactive_cancellation_grace_seconds` | `1.0` | seconds >= 0 | Plugin and worker cancellation | Time allowed for cooperative planner/worker shutdown. |
| `timing.cli_total_time_budget_seconds` | `null` | seconds > 0 or `null` | CLI | `null` means unlimited offline evaluation; overridden by `--time-budget`. |
| `safety.kicad_drc_for_single_track` | `true` | boolean | Plugin, session-editable | Enables native before/after KiCad DRC for a one-connection selection. Disabling it improves latency but removes this gate. |

The JSON has a required integer `schema_version`. Unknown keys, missing keys,
invalid booleans, non-finite numbers, or incoherent time budgets must fail
loudly rather than silently changing safety policy.

## Settings dialog

The session dialog intentionally exposes only controls that are meaningful
during interactive work:

1. native KiCad DRC for a one-track selection;
2. minimum saved length;
3. total interactive time budget;
4. planning time budget;
5. cancellation grace.

Pass limits remain internal because they are implementation guards, not routing
styles. There is no conservative/aggressive mode and no grid preference. The
optimizer always uses exact copper coordinates and the same objective.

## CLI overrides

`--max-passes` and `--time-budget` change the current CLI invocation only.
They do not mutate `internal_config.json` or affect a running KiCad session.
`--no-parallel` changes execution strategy but must not change the selected
plan. Scope options authorize which existing tracks may seed changes; they are
not optimization quality parameters.

## Safety implications

Increasing budgets or pass limits permits a broader search but does not relax
clearance, connectivity, keepout, pad, via, layer, width, or Edge.Cuts rules.
The single-track DRC switch is different: disabling it deliberately removes
the native validation layer for that interactive case, leaving the internal
geometric and connectivity validation active.
