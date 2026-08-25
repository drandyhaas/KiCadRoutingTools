# KiCad Track Gloss 0.3.26 alpha

This is the first community alpha of the standalone KiCad Track Gloss plugin
for KiCad 10.x. Select one or more straight PCB track segments and run the
plugin to expand their connections and apply a shorter or simpler safe
0/45/90-degree route directly to the current board. Use KiCad Undo if the
result is not desired.

Highlights:

- silent single-click normal action and a separate detailed diagnostic action;
- simultaneous multi-track and multi-net batches;
- movable track-intersection and pad-area terminations;
- exact-width, octolinear, clearance-aware optimization;
- shared deterministic engine for the plugin and headless score CLI;
- bounded interactive execution, convergent offline CLI, and optional process
  parallelism for large independent batches;
- optional deep validation for input-order and collinear-subdivision
  invariance;
- PCM v2 package metadata and official-repository submission artifacts.

This version is marked `testing`. Back up important work and run KiCad DRC
after experimenting with non-trivial selections. The plugin never saves the
board automatically.

## Provenance

The project is based primarily on work and source code by DrAndyHaas from
KiCadRoutingTools. Frantz is co-author and maintainer of this standalone
adaptation. Standalone integration and modifications were developed with
ChatGPT/Codex (OpenAI) at the project owner's direction. The complete MIT
license and provenance notice are included in the package.
