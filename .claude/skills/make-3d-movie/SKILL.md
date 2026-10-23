---
name: make-3d-movie
description: Makes the 3D movie of a board being placed, and with a recorded routing chain routed (movie3d/make_movie.py). It prepares the board the way plan-pcb-placement does (fixed parts locked, the rest piled, intent, groups), finds on the web the 3D models of parts the installed KiCad library lacks so they are drawn as parts rather than boxes, then makes the movie. This is the one way to make the 3D movie. Use it to make or remake a 3D placement or flow movie, or to fill in a board's missing 3D models.
---

# Make a 3D movie

Invocation: `/make-3d-movie <board.kicad_pcb> [--route BOARD:SET] [--seed N]
[groups.json] [intent.json]`.
- `--route` films the whole flow: the seeds placed, the judged ones
  fast-routed, the best one routed in full. BOARD:SET names the
  stress-corpus board whose recorded chain routes it, as its name and
  set number.
- `--seed` picks the seed of a placement-only movie (default 0).
- A given groups or intent file is used in place of writing one.

`movie3d/README.md` says what the movie shows and how it is made.

## Step 1: Setup

From the repo root, check:
- `movie3d/node_modules` exists. If not, run `cd movie3d && npm ci`.
- `node` and `ffmpeg` are on PATH, and KiCad 10's `kicad-cli` is installed.
- If the render later fails for want of a browser, run
  `npx playwright-core install chromium-headless-shell` from `movie3d/`, or
  pass `--browser` a Chrome or Chromium executable.

Work in `wk/<run>/`, named after the board. Copy every board with
`python3 -X utf8 py_router/copy_board.py <src> <dst>`, never a bare `cp`.

## Step 2: Prepare the board as plan-pcb-placement does

The movie places the board with `place_simple`, as `/plan-pcb-placement`
does, so prepare it the same way. Read that skill
(`.claude/skills/plan-pcb-placement/SKILL.md`) and do these sections of it:

- **§2, reading the board:** `board_brief`, `board_context`, and its two
  references (`part-roles.md`, `placement-lessons.md`). Its questions to the
  user about EMC limits are for a real placement; a movie can skip them.
- **§3.1, place and lock the fixed parts:**
  - the connectors at the edge, the switches and buttons the enclosure
    reaches, the module headers, the mounting holes and fiducials;
  - take their poses from the input when it is a believable layout.

  This is what keeps the mounting holes in their corners and the connectors
  and switches on their edges in the movie. A part left unlocked is
  re-placed like any other.
- **Then pile every other part off the board**, so the movie starts from the
  piles and nothing starts where the input had it:

  ```bash
  python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/make_unplaced.py \
      wk/<run>/locked.kicad_pcb wk/<run>/fixed.kicad_pcb --keep-locked
  ```
- **§3.2 to §3.5:** the intent (on the pile, as §3.2 says), the groups, the
  connector check, and the datasheet layouts.

You end with `wk/<run>/fixed.kicad_pcb`, `wk/<run>/intent.json` and
`wk/<run>/groups.json`.

## Step 3: Find the missing 3D models

A part whose model the installed library lacks is drawn as a box in its
group's colour. List those models:

```bash
python3 movie3d/find_models.py missing wk/<run>/fixed.kicad_pcb
```

It prints JSON:
- `board`: what identifies the project (file name, title, company,
  comments, footprint library nicknames).
- `library_root`: where KiCad's 3D library is installed.
- `missing`: one entry per model. A part whose footprint names no model
  at all is listed too, keyed `fp-` and its footprint's name: find a model
  of that part the same way. Each entry has:
  - its `key`;
  - the path(s) the footprint names (`referenced_as`);
  - the parts that use it (reference, value, footprint, description, pad
    count, face);
  - `library_near`: installed files whose names are close to it;
  - `library_folders`: the installed folders a comparable model is likely in.
- `not_found_before`: models recorded as not found earlier, left out.
  `--retry` puts them back.

For each model, take the first of these that exists. Search the web with
WebSearch and WebFetch, and the installed library with `ls`:

1. **The exact file the footprint names.** The same file name counts; a
   `.step` and a `.wrl` of one name are equally good.
   - A `${KIPRJMOD}/...` path is the board's own project. Find its repository
     from `board`: the title, a company or copyright comment, and a footprint
     library nickname that is not KiCad's own. The model is in the
     repository's `*.3dshapes` folder.
   - A `${KISYS3DMOD}/...` or `${KICADn_3DMODEL_DIR}/...` path is KiCad's
     library, at https://gitlab.com/kicad/libraries/kicad-packages3D. Check
     master and its release tags (`9.0.0`, `8.0.0`, `7.0.0`, `6.0.0`: tags,
     not `9.0`-style branch names, which do not exist there) for models since
     removed or renamed. A raw file is
     `https://gitlab.com/kicad/libraries/kicad-packages3D/-/raw/<tag>/<Lib>.3dshapes/<file>`.
   - When a `library_near` file is the same model under a new name (KiCad
     renamed `SW_CuK_JS202011CQN...` to `SW_CK_JS202011CQN...`), use it.
2. **The manufacturer's model of the exact part**, found from the value, the
   part number or the description. Only one that downloads without a login or
   a payment. Give it a try or two: many vendor sites refuse a fetch.
3. **A comparable model in the installed KiCad library.** List the
   `library_folders` under `library_root` (`ls`), or browse the others
   there. Take the model of the same kind of part with the same pin count,
   pitch and body size: the nearest slide switch for a slide switch, the same
   header in another colour, a receptacle of the same type. File names say
   most of it. Record it with `library ... --match equivalent`.

Give each model a few tries. Record each answer:

```bash
python3 movie3d/find_models.py add KEY URL --match exact --source "<repository or site>"
python3 movie3d/find_models.py library KEY PATH --match equivalent --note "<why it fits>"
python3 movie3d/find_models.py none KEY --note "<what you tried>"
```

- **`add`** downloads the file and checks it:
  - https only, at most 40 MB;
  - a zip is read in memory and only its model taken;
  - the file is kept only if it really is a STEP or VRML model.

  A refusal prints the reason. "An HTML page" means you gave a page, not the
  file: give its raw-file URL. If one candidate is refused, try the next.
- **`library`** records an installed file.
- **`none`** keeps the model out of later lists.

A footprint that names two models needs only one: once one is recorded, the
other is no longer listed. Run `missing` again to see what is left.

The models live in `movie3d/cache/found/` (gitignored). Never commit them:
they carry their sources' licences.

## Step 4: Make the movie

Placement only:

```bash
python3 -X utf8 movie3d/make_movie.py wk/<run>/fixed.kicad_pcb wk/<run>/placed.kicad_pcb \
    --groups wk/<run>/groups.json --intent wk/<run>/intent.json --seed 0
```

With `--route BOARD:SET`, add it to that command. Every argument other than
make_movie's own is passed to `place_simple`. The movie goes to
`wk/<run>/placed_3d.mp4`, with stills beside it. A small board takes a
minute or two; `--route` takes longer.

## Step 5: Check it

- **The `record:` line** says how many parts are drawn from models, which
  fell back to a box, and which models were found on the web.
- **The `make_movie: rendering` line** names any found model that does not
  sit on its footprint. Those are kept as boxes, because the model is wrong
  for the footprint (rotated, offset, or another part). Replace it (`add`
  another URL, or `none`) and make the movie again.
- **The stills** beside the movie:
  - `_start.png`: every part lies in the piles beside the board. Its
    caption counts the fixed parts, and they fly to their places first.
  - `_final.png`: the mounting holes, connectors and switches are where
    Step 2 locked them, and the key lists them as "fixed".
  - `_bottom.png`: the board turned over.

## Step 6: Report

Report:
- the movie's path and length;
- what was locked;
- each model found: its key, where it came from, and whether it is exact or
  equivalent;
- the parts still drawn as boxes, and why.
