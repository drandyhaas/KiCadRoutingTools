#!/usr/bin/env python3
"""Stage a board for an UNAIDED placement run: netlist in, nothing else.

    python3 -X utf8 tests/stress/stage_unaided.py SRC.kicad_pcb WORKDIR TRUTHDIR

`--kind pile` was the closest thing to this and it is not close enough. Its
own docstring says "this kind exists to produce the place-from-scratch task"
(`placement/perturb.py:368-382`) and it hands over:

  * EVERY free part's original rotation (`:389-391` writes
    `'new_rotation': state.parts[r].rot`). Measured on splitflap_driver:
    65 of 65 preserved, 32 of them non-zero.
  * every pad-less and every `(locked yes)` ref at its true x, y AND rotation,
    because `portfolio.free_refs` skips those and `_all_at_current` then
    writes them where they were.

And `fence_audit` cannot see any of it: `pose_match` counts a ref only when
position AND rotation are inside tolerance, so on a pile the positions are all
wrong, `match_frac` is 0.0000, and the verdict is CLEAN. Run 19's fence exited
CLEAN, 0 on exactly this.

So this stager does three things the pile kind does not:

  1. **Rotation 0 for every non-exempt part.** An angle is a design decision
     as much as a position; handing it over is handing over the answer.
  2. **Exemptions are DECLARED, per ref, with a reason**, in a
     `mechanical.json` the run may read. Some inheritance is legitimate -- a
     mounting hole's position is a mechanical fact, not a placement choice --
     but the difference between a benchmark input and a leak is whether it is
     written down. Silent inheritance is what `--kind pile` does.
  3. **The source is recorded by HASH, not by path.** `stage_blind.py:204`
     writes `'source': src` into the truth dir, naming a file whose original
     placement is one `Read` away. The mapping lives in the truth dir instead.

Truth goes to a SIBLING directory, never a child of the work dir -- the fence
audit recurses, and `perturb()`'s own default writes the control beside the
damaged board, which is how a run gets fenced on paper and open in fact.

Exit codes: 0 staged, 2 usage/IO, 3 the board cannot be staged (no outline).
"""
import argparse
import hashlib
import json
import os
import shutil
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (ROOT, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

SCHEMA = 1


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def classify_exempt(pcb_data, pcb_file=None):
    """{ref: (x, y, rot)} plus {ref: reason} for the parts a real new board
    would genuinely know the position of.

    A PROJECTION of `placement.part_class.mechanical_parts`, not a second
    taxonomy. It used to be the implementation, which meant the only thing in
    the tree that could name a board's mechanical parts lived in a stress
    harness -- so `board_brief` could not answer "where do the fixed parts
    go?" and an unaided author had to read `mechanical.json`, an artifact no
    product path emits. Same classifier, two readers.
    """
    from placement.part_class import mechanical_parts
    found = mechanical_parts(pcb_data)
    refs = {r: tuple(v['at']) for r, v in found.items()}
    reasons = {r: v['reason'] for r, v in found.items()}
    return refs, reasons


#: Project keys that are the AUTHOR'S, not the board's spec. Every one is
#: empty on all 13 in-repo projects and read by nothing in this tree, so
#: dropping them is lossless for every consumer -- but a hand-placed source
#: fills them with board coordinates (`drc_exclusions` carries the offending
#: items' positions; the viewports are named views like "MCU corner"), and
#: `last_paths` carries the author's directories.
_LEAKY_PROJECT_KEYS = (
    ('board', 'design_settings', 'drc_exclusions'),
    ('board', 'viewports'),
    ('board', '3dviewports'),
    ('board', 'layer_presets'),
    ('pcbnew', 'last_paths'),
)


#: What replaces a string that names the source. Not deletion: a run reading
#: the staged project should see that something was withheld, rather than a
#: project that looks like it never carried provenance at all.
_REDACTED = '[redacted: named the source board]'


def _redact_source_stem(node, stem, path=(), hits=None):
    """Replace every string ANYWHERE in the project that names `stem`.

    The key list above is a list of carriers known to leak, and this module's
    own thesis is that the next carrier will have a different name. It does:
    measured on tigard, the leak was
    `kicad_routing_tools.floor_provenance._note` ("tigard is a KiCad 5
    design...") and `.source`
    (`https://raw.githubusercontent.com/tigard-tools/tigard/.../tigard.kicad_pcb`)
    -- an annotation this repo writes itself, in a node no key list contained,
    naming the upstream board and giving a URL that serves its original
    placement. `fence_audit` caught it; the stager did not.

    So the backstop is content-shaped rather than key-shaped: whatever key it
    lives under, a string that spells the source's stem is a path back to the
    original poses. Numbers are untouched, which is what keeps the #441 floor
    intact -- every value the floor is graded at is numeric.
    """
    if hits is None:
        hits = []
    if isinstance(node, dict):
        for k, v in list(node.items()):
            if isinstance(v, str) and stem in v.lower():
                node[k] = _REDACTED
                hits.append('.'.join(path + (str(k),)))
            else:
                _redact_source_stem(v, stem, path + (str(k),), hits)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            if isinstance(v, str) and stem in v.lower():
                node[i] = _REDACTED
                hits.append('.'.join(path + (str(i),)))
            else:
                _redact_source_stem(v, stem, path + (str(i),), hits)
    return hits


def sanitize_staged_project(out_board, source_stem=None):
    """Strip the source's IDENTITY from the carried project; declare the rest.

    `source_stem` is the SOURCE board's stem (e.g. `tigard`). Pass it: the
    key list is a list of known carriers, and the stem sweep is what catches
    the ones nobody has met yet. Omitting it keeps the old key-only behaviour.

    The project must travel -- without it the staged board grades at the
    stock netclass (#441), and on flat_hierarchy that is 0.2 -> 0.25 with the
    source flipping from `board netclass` to `fixed default`. But KiCad
    stores `meta.filename` inside it, so a verbatim copy puts the SOURCE'S
    NAME in the work dir: measured, the staged project read
    `flat_hierarchy.kicad_pro`.

    That defeats this module's own rule -- "the source is recorded by HASH,
    not by path", because a path is one Read away from the original
    placement -- and `fence_audit` could not see it: its `SCANNED_EXT` was
    ('.kicad_pcb', '.json'), so it scanned one file and returned CLEAN.

    Returns the declaration recorded in `mechanical.json`: what travelled,
    and the floors the run will actually be graded at.
    """
    stem = os.path.splitext(out_board)[0]
    # `.kicad_prl` does not enter the work dir. NOTHING in this repo reads it
    # (grep: only copy-lists and a temp cleanup) -- it is KiCad's per-board
    # view state, active layer and selection filter -- and it carries the
    # source's name the same way `meta.filename` does. Measured: after
    # sanitising the project, `board.kicad_prl` was the one remaining file in
    # the work dir containing 'flat_hierarchy'. Carrying a file no consumer
    # reads, at the cost of a second identity leak, is a bad trade. It still
    # travels to the TRUTH dir, which is outside the fence.
    prl = stem + '.kicad_prl'
    dropped_prl = os.path.isfile(prl)
    if dropped_prl:
        os.remove(prl)
    pro = stem + '.kicad_pro'
    out = {'carried': os.path.isfile(pro), 'sanitized_keys': [],
           'dropped_prl': dropped_prl}
    if out['carried']:
        with open(pro, encoding='utf-8') as f:
            doc = json.load(f)
        meta = doc.get('meta')
        if isinstance(meta, dict) and meta.get('filename'):
            meta['filename'] = os.path.basename(pro)
            out['sanitized_keys'].append('meta.filename')
        for path in _LEAKY_PROJECT_KEYS:
            node = doc
            for k in path[:-1]:
                node = node.get(k) if isinstance(node, dict) else None
                if node is None:
                    break
            if isinstance(node, dict) and node.get(path[-1]):
                node[path[-1]] = [] if path[-1] != 'last_paths' else {}
                out['sanitized_keys'].append('.'.join(path))
        if source_stem:
            hits = _redact_source_stem(doc, source_stem.lower())
            out['redacted_source_strings'] = hits
            out['sanitized_keys'].extend(hits)
        with open(pro, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=2)
        out['sha256'] = sha256_file(pro)
    # The numbers the run is graded at, whether or not a project travelled.
    try:
        from list_nets import board_floor_knobs
        out['floors'] = board_floor_knobs(out_board)[2]
    except Exception as e:                                   # noqa: BLE001
        out['floors'] = f'unavailable: {type(e).__name__}: {e}'
    for ext in ('.kicad_dru', '.kicad_prl'):
        out[ext.lstrip('.')] = os.path.isfile(stem + ext)
    return out


def read_mechanical(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def stage(src, out_board, truth_dir=None, mechanical_out=None):
    """Write the unaided board; return the mechanical declaration.

    Everything free lands on ONE coordinate at ROTATION 0. The single
    coordinate is what `placement_state.assess_placement` needs to call the
    board unplaced (`duplicate_fraction >= 0.5` AND `distinct_positions <=
    max(3, 0.1n)`), which is what makes the placement tools accept it as a
    from-scratch task rather than refusing it as a damaged placement.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    from placement.portfolio import copy_siblings

    pcb = parse_kicad_pcb(src)
    bi = pcb.board_info
    if bi.board_bounds is None:
        raise ValueError("the board has no Edge.Cuts outline, so there is "
                         "nothing to stage a placement against")
    x0, y0, x1, y1 = bi.board_bounds
    cx, cy = round((x0 + x1) / 2.0, 6), round((y0 + y1) / 2.0, 6)

    exempt, reasons = classify_exempt(pcb, src)
    placements = []
    for ref, fp in sorted(pcb.footprints.items()):
        if ref in exempt:
            ex, ey, erot = exempt[ref]
            placements.append({'reference': ref, 'new_x': ex, 'new_y': ey,
                               'new_rotation': erot})
        else:
            placements.append({'reference': ref, 'new_x': cx, 'new_y': cy,
                               'new_rotation': 0.0})
    write_placed_output(src, out_board, placements)
    copy_siblings(src, out_board)
    project = sanitize_staged_project(
        out_board, os.path.splitext(os.path.basename(src))[0])

    doc = {'schema': SCHEMA, 'kind': 'mechanical-declaration',
           'refs': exempt, 'reasons': reasons,
           # The project is an input the run reads and it sets every
           # clearance the placement is graded at -- and it was written down
           # NOWHERE, while this file's own note claims the declaration is
           # what makes an input legitimate. A project-less source is now a
           # declared `carried: false` rather than silence.
           'project': project,
           'note': 'the ONLY poses carried over from the source. Everything '
                   'else is at the board centre at rotation 0. A run may read '
                   'this; it is an input, not a leak, BECAUSE it is written '
                   'down. `project` names the floors the run will be graded '
                   'at, for the same reason.'}
    mech = mechanical_out or os.path.join(os.path.dirname(out_board),
                                          'mechanical.json')
    with open(mech, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1, sort_keys=True)

    if truth_dir:
        os.makedirs(truth_dir, exist_ok=True)
        control = os.path.join(truth_dir, 'control.kicad_pcb')
        shutil.copyfile(src, control)
        copy_siblings(src, control)
        with open(os.path.join(truth_dir, 'stage.json'), 'w',
                  encoding='utf-8') as f:
            # The source by HASH. stage_blind records `'source': <path>`,
            # which names a file whose original placement the run can simply
            # open; the mapping belongs on this side of the fence.
            json.dump({'schema': SCHEMA, 'kind': 'unaided-stage',
                       'source_sha256': sha256_file(src),
                       'source_basename': os.path.basename(src),
                       'staged_sha256': sha256_file(out_board),
                       'exempt_refs': sorted(exempt),
                       'piled_at': [cx, cy],
                       'note': 'source recorded by hash, not by path'},
                      f, indent=1, sort_keys=True)
    return doc


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Stage a board for an unaided placement run.")
    p.add_argument("src")
    p.add_argument("workdir")
    p.add_argument("truthdir", nargs='?')
    a = p.parse_args(argv)

    # WORKDIR is a directory, not the output board. Passing `wk/board.kicad_pcb`
    # -- the obvious guess, and what the output is actually called -- made a
    # DIRECTORY of that name containing `board.kicad_pcb/board.kicad_pcb`, and
    # the next command's "cannot read" pointed at the argument rather than the
    # mistake.
    if a.workdir.lower().endswith('.kicad_pcb'):
        p.error(f"WORKDIR is a directory, not a board: {a.workdir!r} names the "
                f"output file. Pass the directory; the staged board is always "
                f"written to WORKDIR/board.kicad_pcb.")

    if a.truthdir and os.path.abspath(a.truthdir).startswith(
            os.path.abspath(a.workdir) + os.sep):
        p.error("TRUTHDIR must be a SIBLING of WORKDIR, never a child -- the "
                "fence audit recurses into the work dir, so truth inside it "
                "is inside the fence")
    os.makedirs(a.workdir, exist_ok=True)
    out = os.path.join(a.workdir, 'board.kicad_pcb')
    try:
        doc = stage(a.src, out, truth_dir=a.truthdir)
    except ValueError as e:
        print(f"stage_unaided: {e}", file=sys.stderr)
        return 3
    except OSError as e:
        print(f"stage_unaided: {e}", file=sys.stderr)
        return 2

    # Disclosure discipline, from stage_blind: say the SHAPE of the board and
    # nothing about what was staged away.
    print(f"Staged {out}")
    print(f"  {len(doc['refs'])} ref(s) declared mechanical and carried at "
          f"their source pose; everything else at the board centre, "
          f"rotation 0")
    for ref in sorted(doc['refs'])[:8]:
        print(f"    {ref}: {doc['reasons'][ref]}")
    if a.truthdir:
        print(f"  truth in {a.truthdir} (a sibling; nothing in the work dir "
              f"names the source)")
    print("JSON_SUMMARY: " + json.dumps(
        {'board': out, 'exempt': len(doc['refs']),
         'mechanical': os.path.join(a.workdir, 'mechanical.json')},
        sort_keys=True))
    return 0


if __name__ == "__main__":
    # In LEVER_REGISTRY, so it must DECLARE -- an entry that writes
    # poses without declaring makes an armed regime refuse the engine
    # itself, which is the failure the registry exists to prevent.
    from placement.provenance import declare_lever
    with declare_lever('stage_unaided.py', sys.argv):
        sys.exit(main())
