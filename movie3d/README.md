# movie3d: a 3D placement movie (prototype)

A 3D take on `py_placer/place_movie.py`. It tells the same placement story
(the input with its piles, the fixed parts flying to their places, the
scatter, the anneal, the best placement) on a 3D board, rendered to an .mp4.
With `--route`, it also films place_movie's flow: one seed's placement and
fast route, then every judged seed's board side by side, the best one picked,
and its routes, with copper appearing and ripped copper going. It is a
prototype, to judge whether a 3D movie is worth more work.

**The story is shared with the 2D movie.** `py_placer/placement_story.py`
holds it: place_simple's hooks, the piles, the schedule and the captions.
`place_movie.py` and this directory are its two renderers. Everything else
lives in this directory. It imports the repo's Python modules read-only
(`place_simple` and its `placement` package, `placement_story`,
`place_movie`'s flow helpers, `route_movie`'s copper helpers, `kicad_parser`,
`render_theme`, `render_placement.group_palette`). Its Node libraries are
installed in `movie3d/node_modules` only.

## Making a movie

**Make one with the `make-3d-movie` skill** (`/make-3d-movie <board>`,
`.claude/skills/make-3d-movie/SKILL.md`). It is the one way to make this
movie: it prepares the board the way `plan-pcb-placement` does (the
connectors, switches and mounting holes locked, every other part piled off
the board, the intent and the groups), finds the 3D models the installed
library lacks, and runs `make_movie.py`.

`make_movie.py` takes the board and an output board first; every other
argument but its own goes to `place_simple.py`, so its output board is
place_simple's placement for that seed. The movie is written beside it as
`_3d.mp4`, or to `--movie`, with four stills: `_start.png`, `_anneal.png`
(mid-anneal), `_bottom.png` (the board turned over) and `_final.png`. The
JSON and the frames go to the output's name + `_movie3d/` (or `--work`).

With `--route BOARD:SET` (a stress-corpus board whose recorded chain routes
this one), it runs place_movie's own `run_flow`, in `--route-work` (default:
the output's name + `_work`, place_movie's default, so a 2D flow run's work
is reused):

1. the round's seeds are placed (`--scattered` 6, `--clustered` 6, and the
   opt-in `--anchored`, 0);
2. the `--judge` 6 cheapest legal seeds are fast-routed;
3. the best of them is routed in full.

The movie films a seed the judge did NOT pick, so the pick is something the
viewer sees happen: the judged seed the objective rates cheapest, or the next
cheapest when that one is the judge's best. It records that seed's placement
and plays its fast route. Then the camera pulls back to every judged seed's
fast-routed board side by side and frames the best one. It flies in to the
best seed, clears its fast route, and plays its full route on the 3D board.
The output board is the best seed. The stills add `_placed.png`,
`_fast_route.png` and `_seeds.png` (the seed grid with the best one framed).
Options for the flow: `--jobs`, `--route-jobs`, `--fresh` and
`--route-frames` (past this many router events in a chain, several share a
frame; 240).

Own options: `--movie`, `--fps` (12), `--width`/`--height` (1000x560),
`--frames` (anneal frames, tweens included, 120), `--tween` (frames per anneal
step, 3; 1 gives the 2D movie's cuts), `--speedup` (how much faster the anneal
plays at its end, 4), `--no-models` (boxes only), `--kicad-cli`, `--tint` (how
far each model's colours go toward its group's colour, 0.4), `--work`,
`--browser`, `--gpu` (auto), `--crf` (20).

**Drawing on the GPU.** On macOS with Google Chrome (or Chromium) in
/Applications, `--gpu auto` (the default) draws the frames on the GPU through
Metal; `--gpu off` keeps the software renderer. On a 306-model board the
render took 70 s on an M3's GPU against about 24 minutes in software, and the
two give the same frames to within a fraction of a grey level.

**What it needs.** Its Node libraries, `playwright-core` 1.52.0 and `three`
0.186.1, are pinned by `package-lock.json` and installed in
`movie3d/node_modules` only (about 31 MB). playwright-core 1.52.0 drives
Chromium build 1169: Playwright's per-user cache holds it once any project on
Playwright 1.52 has fetched it (`~/Library/Caches/ms-playwright` on macOS),
or `--browser` (or `$MOVIE3D_CHROMIUM`) names a Chrome or Chromium
executable. Also `ffmpeg` on PATH and Node 18 or newer; Python needs nothing
beyond the repo's `requirements.txt`.

**Part models** need KiCad's `kicad-cli` (KiCad 10, with its 3D model
library), found through `--kicad-cli`, then `$KICAD_CLI`, then the repo's
`kicad_oracle` resolver, then `/Applications/KiCad/KiCad.app`, then PATH.
Without it, every part is drawn as a box. **A model the library lacks** is
found on the web by the skill -- and so is one for a part whose footprint
names no model at all -- and recorded with `find_models.py` in
`movie3d/cache/found/` (gitignored): its download is kept only when it is a
STEP or VRML model (https only, at most 40 MB, a zip read in memory).
make_movie runs no agent: it draws whatever has been recorded, names the
models still missing, and keeps the box for a found model that does not sit
on its footprint.

## What it shows

- **The board**: its outline extruded, standing above a table. It is drawn at
  most 0.8 mm thick (`DRAWN_THICKNESS`): at its real 1.6 mm the slab looked
  like a block next to 0402 parts. It is see-through (55% opaque,
  `BOARD_ALPHA`), so the back's parts, their airwires and the back's copper
  show through it, dimmed.
- **Each part**:
  - its pads, with dark drills;
  - its courtyard, as a plate lightly tinted in its group's colour;
  - its body: its **KiCad 3D model**, with its colours tinted 40% of the way
    toward its group's colour (`--tint`). The models come from `models.py`: kicad-cli
    exports the placed board's parts as one GLB, and the page re-poses each
    part's node every frame.
  - A part without a model falls back to a **box**, and the console says
    which parts did. Connectors and switches get a box the size of the
    courtyard, inset 0.25 mm; all other parts get the pad field, inset. The
    height is a guess from the footprint name or the reference prefix
    (`NAME_HEIGHTS` and `PREFIX_HEIGHTS` in `record.py`).
  - Fixed parts are grey and see-through, so they never hide the parts that
    move. A COVER -- a part other parts may stand under (#1443): a module, a
    shield can, a socket whose card the intent declares -- is see-through
    too, in its group's colour, so the parts under it stay in view; a
    declared card, which has no model of its own, is a see-through slab from
    its declared underside, moving with its connector. Unassembled parts with no model (test pads, solder jumpers) get no
    body. A back-side part hangs below the board. Parts off the board lie on
    the table.
- **Airwires**: the nets the objective prices (ground is left out), as one
  spanning tree per net, recomputed every frame.
- **Text**: place_movie's caption at the top left, and the group key along the
  bottom.

The steps, in order:

1. The input: the board, with the piles on the table beside it.
2. The fixed parts arc to their places, biggest first.
3. The scatter: the pile takes off part by part and arcs to the seeded places,
   while the camera closes in on the board.
4. Edge seats, when the intent declares any.
5. The anneal, sampled on place_movie's schedule and playing faster as it goes.
   The frame budget (`--frames`) is shared: `--frames / --tween` snapshots,
   each played as a move of `--tween` frames. A part that moved since the last
   snapshot slides there with a small hop, turning the short way round, so
   moves read as motion rather than cuts.
6. The best placement: the parts go back to the best placement the anneal saw.
7. Placed, held for 3 s.
8. Turned over: the board rises off the table, turns over about the screen's
   horizontal, holds 2 s with its bottom up, turns on round to its top and
   settles. The camera closes in on the board face up and face down, and
   steps back while it turns.

Over the whole movie the camera orbits slowly, 20 degrees either side of a
base azimuth, at 46 degrees of elevation. Each frame is fitted to the board,
or to the board and the piles.

With `--route`, the placed seed's routes follow, as place_movie's 2D flow
plays them:

1. The fast route plays: one router event per frame. New copper is lit white
   (cyan when the router restored it). A ripped track turns magenta, then
   goes; the 2D movie uses red, but F.Cu is red here. Whatever else a step
   changed (a pour, the fanout, a step with no trace) then lands net by net.
2. The camera pulls back to every judged seed's board side by side, each at
   its placement with its fast route and labelled with its score (blocking |
   vias | mm). The grid lies along the camera's screen axes, in the
   columns x rows that fill the frame best (`record.Recorder._lay_grid`).
3. The best seed is framed in gold, blinking as the 2D movie's box does, and
   its label says BEST.
4. While the camera is pulled back, the live board becomes the best seed's,
   unseen, because its copy in the grid looks the same. The camera flies in
   to it, and the other seeds' boards go.
5. Its fast route is cleared, net by net.
6. Its full route plays the same way.
7. The board turns over, as at the end of a placement movie.

Copper is drawn per layer in the theme's layer colours, vias as barrels,
pours as tinted flats clipped to the board.

The base azimuth keeps the action in view. For every 10 degrees,
`record.Recorder._hidden` measures how much the tall bodies (0.9 mm and
taller) hide of the other parts, from the anneal on. A part counts as hidden
when its centre lies in the ground a body hides behind itself: up to its
height / tan(elevation) behind it. Parts that are moving count double, and
the see-through fixed parts hide at half. The least-hidden azimuth wins, and a
view across the board's long side wins a near tie.

## How it works

- `py_placer/placement_story.py` (shared with `place_movie.py`) is the story.
  Its hooks (`build`, `scatter`, `anneal`, and
  `Annealer.start_temperature`/`trial`/`overlap`) only read the state and pass
  every call through, so the placement is place_simple's own. It decides what
  is shot when, and calls a renderer's primitives (`setup`, `shot`, `view_of`,
  `set_view`). The 2D renderer is place_movie's `Film`; its output is
  unchanged by the move: the same frames, hash for hash, on a test board's
  seed 0, both with and without `--route`.
- `record.py`'s `Recorder` is the 3D renderer. Each shot becomes a timeline
  row, and the 3D beats are overrides: arcs, the table, the staggered scatter,
  the tweened anneal, the glide back to the best placement. It runs
  `place_simple.main` in-process and writes two files:
  - `scene.json`: every part in its own frame (pads, courtyard, body, side,
    colour), plus the nets and the key;
  - `timeline.json`: one row per frame, with every part's
    `[x, y, rot, height]`, the caption, the parts that moved, the camera's box
    and its azimuth.
- `models.py` exports the parts' models with
  `kicad-cli pcb export glb --no-board-body --subst-models`. It first stages
  the board (`py_router/model_export.py`, which the placer's part heights
  share): each model path is made absolute, a `.wrl` the library no longer
  ships is pointed at its `.step` twin, a still-missing model is found by
  its file name in the library, and failing that it takes the one the
  make-3d-movie skill found on the web (`find_models.py`, one per
  footprint). The GLB is cached in `movie3d/cache/<key>/` (gitignored), keyed by the staged board and kicad-cli's version, so a
  re-render or another seed does not export again. kicad-cli writes no
  metallic factor, and glTF's default of 1 makes every model a black mirror,
  so the page sets each model material's metalness to 0.15.
- `route3d.py` plays a route_placed.py chain as route_movie's `play_chain`
  does, with its own helpers. Each track, via and pour gets the tick it was
  born at and the tick it died at (stage3d's copper lifetimes). A frame names
  one tick, and the page's shader shows, lights or reddens each item for it.
- `render.mjs` uses Node and playwright-core to open `page/index.html` in
  headless Chromium, on SwiftShader (CPU WebGL, so the pixels do not depend on
  the GPU), or with `--gpu` an installed Chrome on Metal. The page is served through `page.route` from a fake host, so
  nothing leaves the machine. For each frame it calls `m3dFrame(i)` and
  screenshots the result to `frames/f00000.png`, and so on.
- `page/movie3d.mjs` is the three.js scene. It is a pure function of
  (scene, timeline, frame index).
- `make_movie.py` runs the three steps above, then ffmpeg.

**Reused from the removed `py_router/stage3d/` (#1081, still on `main`):**
- the page design: one pure function per frame, an importmap, served through
  `page.route`;
- the skeleton of `render.mjs`: SwiftShader, CDP screenshots, JSON-lines
  progress;
- the board extrusion and the frame mapping (KiCad (x, y) to three (x, z), and
  a KiCad rotation to `rotation.y`);
- the body and not-assembled rules;
- the three.js version (r186).

Also reused: stage3d's kicad-cli GLB model export and its re-pose,
`F(now) · F(export)⁻¹`, and its copper-lifetime idea and track quads. Its
Blender path was not carried over, and the turn over is this page's own.

## Limits

- The seed grid shows each judged seed's fast route as it ended, not played.
  Only the filmed seed's and the best seed's routes play.
- A part whose model is missing from the installed library, and not found
  on the web, is a box with a guessed height. The occlusion measure uses the guessed heights even for parts
  drawn from models. A custom pad is drawn as its bounding box. No silkscreen
  is drawn, and in a flow a pour is drawn by its zone outline, not its fill.
- Models make rendering slower: 110 ms per frame on a 25-part board, against
  72 ms with boxes.
- A part the intent puts on the other face, or at a declared pose, is drawn
  there from the first frame. The 2D movie shows it moving there.
- Early in the anneal one step covers about a hundred moves, so a tween is a
  straight slide from one snapshot to the next, not the path the part took.
- Airwires are 1 px lines, because WebGL draws lines 1 px wide.
- The occlusion measure uses the guessed box heights and part centres only.
- Tested on two boards: a 25-part one (with `--route` too) and an 89-part one.
