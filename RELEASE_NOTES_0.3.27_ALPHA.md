# KiCad Track Gloss 0.3.27 alpha

This alpha adds delayed visual feedback for long interactive calculations.
Gloss operations that finish in less than three seconds remain visually
silent. If planning is still running after three seconds, KiCad displays its
busy cursor until the calculation finishes or fails.

The API-neutral planning engine is the only work moved to a background thread.
Reading and modifying the current board remain on KiCad's main thread, and the
cursor is restored even when planning raises an error.

The routing engine and optimization results are unchanged from 0.3.26.

This version is marked `testing`. Back up important work and run KiCad DRC
after experimenting with non-trivial selections. The plugin never saves the
board automatically.

## Provenance

The project is based primarily on work and source code by DrAndyHaas from
KiCadRoutingTools. Frantz is co-author and maintainer of this standalone
adaptation. Standalone integration and modifications were developed with
ChatGPT/Codex (OpenAI) at the project owner's direction. The complete MIT
license and provenance notice are included in the package.
