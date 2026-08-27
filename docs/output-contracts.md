# Machine-readable output contracts

Track Gloss keeps human diagnostics separate from automation protocols. This
document defines the stable CLI records and relates them to conventions used
by KiCadRoutingTools.

## Standard output

A successful CLI evaluation emits these records in order:

```text
SCORE_JSON={...}
GLOSS_SCORE_JSON={...}
SCORE=<float>
```

`SCORE_JSON=` is the canonical score payload and matches the naming used by
KiCadRoutingTools score instruments. `GLOSS_SCORE_JSON=` is a compatibility
alias containing the exact same compact JSON document. It may be removed only
after known consumers have migrated. `SCORE=` remains the final line and the
mandatory `place_route_loop --accept-cmd` contract.

No narration or progress output should be added to stdout. A caller should
match a complete line by prefix instead of assuming that the JSON line is at a
fixed numeric position while the compatibility alias exists.

## Standard error

Errors and optional convergence trace records use stderr:

```text
GLOSS_PASS_JSON={...}
track-gloss-score: ErrorType: explanation
```

KiCad or wx runtime warnings should also be directed to stderr wherever the
runtime permits. They must never appear between the `SCORE=` prefix and its
number.

## JSON file

`--json-out PATH` writes the canonical payload as an indented JSON document.
It contains no prefix. Its semantic content is identical to `SCORE_JSON=`;
formatting and key order are not part of the schema contract.

## Score payload

The payload currently uses:

- `schema`: integer schema revision;
- `kind`: `track-gloss-score`;
- `plugin_version`;
- `board`, `project`, and optional loop-provided paths;
- `score` and `score_meaning`;
- `scopes`, selected seeds, expanded and eligible tracks;
- before/after length and segment metrics;
- `changed`, `fixed_point`, `convergence_passes`, `max_passes`, time budget,
  and `minimum_saved_length_mm`;
- native DRC status, category counts, errors, timings, cache state, and
  validation mode;
- output board path when one was requested;
- conservative candidate-ladder fallback state.

New optional fields may be added without changing `schema`. Removing or
renaming a field, changing its type, or changing score semantics requires a
schema revision and a migration note.

## Result versus process success

A safe no-op is a successful evaluation and exits `0`; it reports
`changed: false`. Reaching a configured limit must not be reported as a fixed
point. Invalid arguments, unreadable inputs, missing KiCad runtime, validation
infrastructure failure, and write failure exit non-zero and do not emit a
misleading `SCORE=` result.

The plugin diagnostic JSON uses the same metric vocabulary where applicable,
but its report dialog is not a CLI protocol. Human labels may evolve without
changing the score schema.
