# KiCad Track Gloss

KiCad Track Gloss is a KiCad 10 ActionPlugin for shortening and simplifying
existing PCB tracks. Select one or more straight segments and run the action.
The plugin automatically expands the selected connections, searches for safe
0/45/90-degree improvements, validates the composed plan, and applies it to the
current board as one Undo operation.

Successful operations are silent and never save the board. Use KiCad Undo to
reject a result. A valid no-op plays KiCad's standard warning sound once.

## Actions

- **KiCad Track Gloss** performs the normal one-click operation.
- **KiCad Track Gloss - Diagnostic** performs the same operation and displays
  Result, Details, and JSON tabs with copy actions.

Run either action without a selected straight segment to edit in-memory session
settings. The first setting controls native KiCad DRC for a one-track
selection: disabling it is substantially faster, but removes that native
before/after safety check. The packaged default is enabled.

Native validation uses private temporary board copies, refills zones, and
compares KiCad DRC reports before and after the candidate. This can add seconds
even when geometric planning is fast. It neither saves nor modifies the live
board.

## Documentation

The complete documentation is maintained in the repository:

- [Plugin usage](../docs/plugin-usage.md)
- [CLI](../docs/cli.md)
- [Configuration](../docs/configuration.md)
- [Output contracts](../docs/output-contracts.md)
- [Safety and DRC](../docs/safety-and-drc.md)
- [Architecture](../docs/architecture.md)

Online source and documentation:
<https://github.com/fca1/KiCadRoutingTools/tree/codex/kicad-track-gloss>

## Provenance

This standalone plugin is inspired by and reusing part of DrAndyHaas's code
from [KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools).
Original MIT notices for reused and adapted material are retained. Standalone
integration and subsequent modifications were produced with ChatGPT/Codex
(OpenAI) at the project owner's direction. The project is maintained by Frantz.
See `NOTICE` and `LICENSE`.
