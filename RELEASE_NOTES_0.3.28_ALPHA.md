# KiCad Track Gloss 0.3.28 alpha

This alpha hardens Track Gloss against native KiCad DRC regressions:

- renamed internal copper layers are detected semantically, so through-vias
  and through-hole pads remain obstacles and connectivity terminals;
- complete Edge.Cuts polygons, holes, and curved boundaries replace the older
  bounding-box-only candidate check;
- explicit solder-mask graphics are protected from newly routed copper;
- one final KiCad-native DRC comparison refills zones on private temporary
  snapshots and rejects a composed plan if any new semantic violation or
  unconnected-item relationship appears;
- headless KiCad font warnings are redirected to stderr instead of opening a
  blocking message window.

The DRC gate fails closed: if its temporary validation cannot run, the current
board remains unchanged.

This version is marked `testing`. Back up important work and run KiCad DRC
after experimenting with non-trivial selections. The plugin never saves the
current board automatically.

## Provenance

The project is based primarily on work and source code by DrAndyHaas from
KiCadRoutingTools. Frantz is co-author and maintainer of this standalone
adaptation. Standalone integration and modifications were developed with
ChatGPT/Codex (OpenAI) at the project owner's direction. The complete MIT
license and provenance notice are included in the package.
