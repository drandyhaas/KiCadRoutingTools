"""The stage3d film's 3D board (#1081).

`timeline` turns the per-frame stage record `animate_route.build_boards
(stage_out=)` collects into a JSON timeline; `scene` builds the static scene
(outline, parts, pads, copper with its lifetimes) from the boards; `render3d`
drives a pinned three.js page in headless Chromium (Node + playwright-core)
to draw each frame. Everything here is a pure function of the film's own
record, so the 3D board shows exactly the events the 2D X-ray shows.
"""
