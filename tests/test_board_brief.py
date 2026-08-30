#!/usr/bin/env python3
"""The board brief assembles; it does not compute.

That is the whole claim, and it is the one worth testing: a brief that
re-derived any number would become a SECOND source of truth for it, and the
first time the two disagreed nobody would know which one the graders use. So
each section here is compared against the emitter that owns it, called
directly, and required to be equal -- not merely plausible.

The other half is honesty about absence. A section that cannot run has to
appear in `skipped` WITH A REASON and be absent from the body; a brief that
silently omitted a measurement would read as "nothing to report", which is the
failure `check_channels` names when it exits 3 rather than answering "clean"
for a gate that examined nothing.
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (REPO,):
    if p not in sys.path:
        sys.path.insert(0, p)
        sys.path.insert(0, os.path.join(p, 'py_router'))
        sys.path.insert(0, os.path.join(p, 'py_tools'))
        sys.path.insert(0, os.path.join(p, 'py_placer'))

BOARD = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


if not os.path.isfile(BOARD):
    print("SKIP: fixture missing")
    sys.exit(0)

from kicad_parser import parse_kicad_pcb
from board_brief import build_brief, format_text

pcb = parse_kicad_pcb(BOARD)
brief = build_brief(pcb, BOARD, requirements="USB on the north edge",
                    fit=[(15.5, 15.5), (400.0, 400.0)])

# --------------------------------------------------------------------------
# shape
# --------------------------------------------------------------------------
check("the brief carries every section",
      {'board', 'state', 'parts', 'structure', 'measurements', 'fit',
       'sources', 'skipped'} <= set(brief), str(sorted(brief)))
check("nothing was skipped on a healthy board", not brief['skipped'],
      str(brief['skipped']))
check("every section names the function that produced it",
      all(k in brief['sources'] for k in
          ('board', 'state', 'parts', 'structure', 'measurements', 'fit')),
      str(sorted(brief['sources'])))
check("the brief is JSON-serialisable",
      isinstance(json.dumps(brief, default=str), str))
check("format_text produces something to read",
      len(format_text(brief).splitlines()) > 4)

# --------------------------------------------------------------------------
# ASSEMBLES, DOES NOT COMPUTE: each number equals its emitter's
# --------------------------------------------------------------------------
check("board.footprints == the parsed count",
      brief['board']['footprints'] == len(pcb.footprints),
      f"{brief['board']['footprints']} vs {len(pcb.footprints)}")
check("board.bounds == board_info.board_bounds",
      brief['board']['bounds'] == list(pcb.board_info.board_bounds))

from placement.placement_state import assess_placement
st = assess_placement(pcb, BOARD)
check("state.unplaced == assess_placement's",
      brief['state']['unplaced'] == st.unplaced)
check("state.stacked_suspect_refs == assess_placement's",
      brief['state']['stacked_suspect_refs'] == list(st.stacked_suspect_refs))

from placement.groups import derive_groups
for src in ('decap', 'sheet'):
    want = derive_groups(pcb, (src,))
    got = (brief['structure']['groups'] or {}).get(src)
    if want:
        check(f"structure.groups[{src}] == derive_groups'",
              got == {k: sorted(v) for k, v in sorted(want.items())},
              f"{len(got or {})} vs {len(want)}")
    else:
        check(f"structure.groups has no empty {src} bucket", got is None,
              f"emitter returned nothing but the brief carries {got!r}")

from placement.escape import escape_ledger
led = escape_ledger(pcb, pcb_file=BOARD, clearance=brief['clearance'])
esc = brief['measurements'].get('escape') or {}
check("measurements.escape.parts == escape_ledger's length",
      esc.get('parts') == len(led), f"{esc.get('parts')} vs {len(led)}")
# `PartEscape` has no `worst_deficit` ATTRIBUTE -- only a to_dict() key -- so
# a getattr with a 0 default silently reports every board as deficit-free.
# Compare against the emitter's own dict, which is where the name is real.
want_parts = sum(1 for p in led if p.to_dict()['worst_deficit'] > 0)
check("measurements.escape.deficit_parts == the emitter's own worst_deficit",
      esc.get('deficit_parts') == want_parts,
      f"{esc.get('deficit_parts')} vs {want_parts}")

from placement.lock_advisor import advise_locks, to_json as lock_json
adv = lock_json(advise_locks(pcb, pcb_file=BOARD,
                             conflict_clearance=brief['clearance']))
check("measurements.locks.unlocked_high == the advisor's own tally",
      (brief['measurements']['locks']['tally'].get('unlocked_high')
       == adv.get('unlocked_high')),
      f"{brief['measurements']['locks']['tally'].get('unlocked_high')} vs "
      f"{adv.get('unlocked_high')}")
check("measurements.locks.lock_argv is the advisor's paste-ready form",
      brief['measurements']['locks']['lock_argv'] == adv.get('lock_argv'))

# --------------------------------------------------------------------------
# the parts table
# --------------------------------------------------------------------------
parts = brief['parts']
check("every footprint appears in the parts table",
      set(parts) == set(pcb.footprints), str(len(parts)))
check("a part's extent says whether it came from a courtyard or a pad bbox",
      all(p['extent_source'] in ('courtyard', 'pad_bbox', 'none')
          for p in parts.values()),
      str({p['extent_source'] for p in parts.values()}))
check("at least one part fell back to its pad bbox, and says so",
      any(p['extent_source'] == 'pad_bbox' for p in parts.values()),
      "this board's H6/H7 have no courtyard, so the distinction must show")

# --------------------------------------------------------------------------
# fit: opt-in, and honest when the answer is nowhere
# --------------------------------------------------------------------------
fits = {tuple(r['extent_mm']): r for r in brief['fit']}
check("a part that fits reports where", fits[(15.5, 15.5)]['fits']
      and fits[(15.5, 15.5)]['legal_cells'] > 0, str(fits[(15.5, 15.5)]))
check("a part larger than the board reports NOWHERE, not an empty success",
      fits[(400.0, 400.0)]['fits'] is False
      and fits[(400.0, 400.0)]['legal_cells'] == 0,
      str(fits[(400.0, 400.0)]))
check("fit names itself as computed rather than assembled",
      'COMPUTED' in brief['sources']['fit'], brief['sources']['fit'])
check("fit is absent unless asked for",
      'fit' not in build_brief(pcb, BOARD))

# --------------------------------------------------------------------------
# requirements are carried VERBATIM -- they are the constraints that live
# nowhere in the board file, and paraphrasing them would be this tool
# inventing mechanical geometry
# --------------------------------------------------------------------------
check("requirements are carried verbatim",
      brief['requirements'] == "USB on the north edge", brief['requirements'])
check("no requirements key when none were given",
      'requirements' not in build_brief(pcb, BOARD))

# --------------------------------------------------------------------------
# absence is disclosed, not silent
# --------------------------------------------------------------------------
import board_brief as bb
real = bb.escape_ledger if hasattr(bb, 'escape_ledger') else None


def _boom(*a, **kw):
    raise RuntimeError("deliberate")


import placement.escape as esc_mod
_saved = esc_mod.escape_ledger
esc_mod.escape_ledger = _boom
try:
    broken = build_brief(pcb, BOARD)
finally:
    esc_mod.escape_ledger = _saved
check("a section that raises is reported in `skipped` with its reason",
      any('escape' in k for k in broken['skipped'])
      and 'deliberate' in ' '.join(broken['skipped'].values()),
      str(broken['skipped']))
check("and is absent from the body rather than present-and-empty",
      'escape' not in (broken['measurements'] or {}),
      str(sorted(broken['measurements'] or {})))

# The advisor's tally is a TALLY: counts, not the whole findings list. A
# `startswith('findings')` filter also matches `findings` itself, which
# silently embedded every per-part reason and evidence blob under it.
tally = brief['measurements']['locks']['tally']
check("locks.tally carries counts only",
      all(isinstance(v, int) for v in tally.values()),
      str({k: type(v).__name__ for k, v in tally.items()}))
check("locks.tally does not embed the findings list",
      'findings' not in tally, str(sorted(tally)))

# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
with tempfile.TemporaryDirectory() as wd:
    out = os.path.join(wd, 'brief.json')
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join('py_tools',
                                                    'board_brief.py'),
         BOARD, '--json', out, '--fit', '15.5x15.5'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=REPO, timeout=900,
        env=dict(os.environ, PYTHONIOENCODING='utf-8'))
    check("the CLI exits 0 on a healthy board", r.returncode == 0,
          r.stderr[-400:])
    check("the CLI writes the JSON it says it wrote", os.path.isfile(out))
    check("the CLI prints a JSON_SUMMARY",
          any(l.startswith('JSON_SUMMARY:') for l in r.stdout.splitlines()))
    # The floors path takes no --clearance here, which is the call shape that
    # crashed while board_floor_knobs' TRIPLE return was read as a mapping.
    check("the CLI resolves floors without an explicit --clearance",
          'Traceback' not in r.stderr, r.stderr[-400:])

# --------------------------------------------------------------------------
# extent_mm is BOARD-FRAME and includes the pads' own size
#
# Two defects that shipped together, both silent:
#   * the pad fallback measured max(global_x) - min(global_x), i.e. pad
#     CENTRES, so a two-pad part had ZERO width along its pad axis;
#   * both geometry sources return the footprint-LOCAL frame and nothing
#     rotated them, so every 90/270 part was transposed.
# The brief's own `fit` section IS board-frame, so `extent_mm` could not be
# fed to --fit WxH for half a board -- and the understated area fed
# grow_board's utilisation.
# --------------------------------------------------------------------------
from placement.legality import rotate_local_bounds
from placement.parser import extract_courtyard_bboxes
from placement.utility import compute_footprint_bbox_local

for _bd in (BOARD, os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')):
    if not os.path.isfile(_bd):
        continue
    _pcb = parse_kicad_pcb(_bd)
    _brief = build_brief(_pcb, _bd)
    _cy = extract_courtyard_bboxes(_bd) or {}
    bad, rotated, twopad = [], 0, 0
    for _ref, _row in _brief['parts'].items():
        _fp = _pcb.footprints.get(_ref)
        if _fp is None or _row['extent_source'] == 'none':
            continue
        _box = _cy.get(_ref) or compute_footprint_bbox_local(_fp)
        _g = rotate_local_bounds(*_box, _fp.rotation or 0.0)
        want = [round(_g[2] - _g[0], 3), round(_g[3] - _g[1], 3)]
        if want != _row['extent_mm']:
            bad.append((_ref, _row['extent_mm'], want))
        if (_fp.rotation or 0.0) % 180 not in (0.0,):
            rotated += 1
        if len(_fp.pads or ()) == 2 and _row['extent_source'] == 'pad_bbox':
            twopad += 1
            if min(_row['extent_mm']) <= 0.0:
                bad.append((_ref, _row['extent_mm'], 'zero-width two-pad part'))
    name = os.path.basename(_bd)
    check(f"{name}: every extent matches the canonical board-frame bbox",
          not bad, f"{len(bad)} mismatch(es): {bad[:3]}")
    check(f"{name}: the fixture has rotated parts, so the frame check bites",
          rotated > 0, f"{rotated} part(s) at a non-0/180 rotation")

_ep = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
if os.path.isfile(_ep):
    _b = build_brief(parse_kicad_pcb(_ep), _ep)
    c1 = _b['parts'].get('C1', {}).get('extent_mm')
    check("esp_prog C1 (an 0603) has real width on both axes",
          c1 and min(c1) > 0.5, f"{c1} -- it used to read [0.0, 1.778]")

# --------------------------------------------------------------------------
# COORDINATES. The brief emitted `rotation` and no position, from the same
# footprint, in the same dict literal -- while six of the thirteen plan ops
# take a literal board coordinate and `place_fixed` REQUIRES one.
# --------------------------------------------------------------------------
from placement.legality import graded_parts_from_file

for _bd in (BOARD, os.path.join(REPO, 'kicad_files', 'watchy.kicad_pcb')):
    if not os.path.isfile(_bd):
        continue
    name = os.path.basename(_bd)
    _pcb = parse_kicad_pcb(_bd)
    _brief = build_brief(_pcb, _bd)

    bad_at = [(r, row.get('at'), [round(_pcb.footprints[r].x, 6),
                                 round(_pcb.footprints[r].y, 6)])
              for r, row in _brief['parts'].items()
              if r in _pcb.footprints
              and row.get('at') != [round(_pcb.footprints[r].x, 6),
                                    round(_pcb.footprints[r].y, 6)]]
    check(f"{name}: every part row carries its own pose",
          _brief['parts'] and not bad_at,
          f"{len(bad_at)} wrong: {bad_at[:3]}")

    # The absolute rect must be the one the LEGALITY GRADER uses, or the
    # brief is publishing a second geometry an author would place against
    # and a grader would then contradict.
    #
    # `extent_source: 'none'` is excluded and then checked separately: the
    # two DO disagree there, deliberately. `compute_footprint_bbox_local`
    # substitutes a 1x1mm placeholder for a part with no pads, so the grader
    # has something to grade; the brief says it has no geometry instead.
    # Asserting them equal would force the brief to publish the placeholder
    # as if it were a measurement.
    _graded = {p.ref: p.rect for p in graded_parts_from_file(_pcb, _bd)}
    _cmp = {r: v for r, v in _graded.items()
            if r in _brief['parts']
            and _brief['parts'][r]['extent_source'] != 'none'}
    bad_rect = [(r, _brief['parts'][r].get('rect'), list(_cmp[r]))
                for r in _cmp
                if (_brief['parts'][r].get('rect') is None
                    or max(abs(a - b) for a, b in
                           zip(_brief['parts'][r]['rect'], _cmp[r])) > 1e-6)]
    check(f"{name}: parts[*].rect == graded_parts_from_file's rect",
          _cmp and not bad_rect,
          f"{len(bad_rect)} of {len(_cmp)} graded differ: {bad_rect[:2]}")
    _nogeom = [r for r, row in _brief['parts'].items()
               if row['extent_source'] == 'none']
    check(f"{name}: a null rect means the part really has no pads",
          all(not (_pcb.footprints[r].pads or ()) for r in _nogeom)
          and all(_brief['parts'][r]['rect'] is None for r in _nogeom),
          str([(r, len(_pcb.footprints[r].pads or ())) for r in _nogeom]))

    # Rings, not counts. `place_keepout` takes a rect and nothing in the
    # brief could supply one.
    _bi = _pcb.board_info
    check(f"{name}: cutout rings round-trip to the parser's own",
          _brief['board']['cutout_rings'] ==
          [[[round(float(x), 6), round(float(y), 6)] for x, y in ring]
           for ring in (getattr(_bi, 'board_cutouts', None) or ()) if ring],
          f"{len(_brief['board']['cutout_rings'])} ring(s) vs "
          f"{len(getattr(_bi, 'board_cutouts', None) or ())} parsed")
    check(f"{name}: the ring count still matches the published rings",
          _brief['board']['cutouts'] == len(_brief['board']['cutout_rings'])
          and _brief['board']['outlines'] ==
          len(_brief['board']['outline_rings']))

_w = os.path.join(REPO, 'kicad_files', 'watchy.kicad_pcb')
if os.path.isfile(_w):
    _b = build_brief(parse_kicad_pcb(_w), _w)
    # watchy is the corpus's only board with cutouts: two strap slots worth
    # 118.97mm2, which a bbox area reports as copper that is not there.
    check("watchy: copper area subtracts the cutouts",
          _b['board'].get('copper_area_mm2') == 1434.09,
          f"copper {_b['board'].get('copper_area_mm2')}, "
          f"bbox {_b['board'].get('area_mm2')} (must not be equal)")
    check("watchy: the bbox area is UNCHANGED, so no reader silently moves",
          _b['board'].get('area_mm2') == 1553.06,
          str(_b['board'].get('area_mm2')))
    check("watchy: the cutouts are the difference",
          abs((_b['board']['outline_area_mm2']
               - _b['board']['cutout_area_mm2'])
              - _b['board']['copper_area_mm2']) < 0.01)

# --------------------------------------------------------------------------
# MECHANICAL. The one channel for "which parts' positions are fixed" that
# survives a pile -- `measurements.locks.high` is geometric and does not.
# --------------------------------------------------------------------------
from placement.part_class import mechanical_parts

for _bd in (BOARD, _w, os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb')):
    if not os.path.isfile(_bd):
        continue
    name = os.path.basename(_bd)
    _pcb = parse_kicad_pcb(_bd)
    _brief = build_brief(_pcb, _bd)
    _mech = (_brief.get('mechanical') or {}).get('refs') or {}
    check(f"{name}: mechanical names the same refs as its own emitter",
          set(_mech) == set(mechanical_parts(_pcb)),
          f"{sorted(_mech)} vs {sorted(mechanical_parts(_pcb))}")
    check(f"{name}: the fixture HAS mechanical parts, so the check bites",
          len(_mech) > 0, f"{len(_mech)} found")
    # Two grounds, two evidence shapes. A CLASSIFIED ref must carry the
    # classifier's confidence and what it saw; a ref admitted only because
    # the file locks it has no class evidence to give -- the lock IS the
    # evidence -- so requiring one of it would have forced a fabricated
    # value (tigard's G*** is exactly this case).
    bad_ev = [(r, v.get('class'), v.get('confidence'), v.get('evidence'),
               v.get('locked_in_source'))
              for r, v in _mech.items()
              if not v.get('reason')
              or (v.get('class') and not (v.get('confidence')
                                          and v.get('evidence')))
              or (not v.get('class') and not v.get('locked_in_source'))]
    check(f"{name}: every mechanical ref carries evidence for its own ground",
          not bad_ev, str(bad_ev[:3]))
    # The `at` is the field `place_fixed` consumes. Nothing validated it.
    bad_mat = [(r, v['at'], [round(_pcb.footprints[r].x, 6),
                             round(_pcb.footprints[r].y, 6),
                             round(_pcb.footprints[r].rotation or 0.0, 6)])
               for r, v in _mech.items()
               if v['at'] != [round(_pcb.footprints[r].x, 6),
                              round(_pcb.footprints[r].y, 6),
                              round(_pcb.footprints[r].rotation or 0.0, 6)]]
    check(f"{name}: mechanical `at` is the part's real pose, x/y AND rotation",
          not bad_mat, str(bad_mat[:3]))
    # On a PLACED board nothing shares a coordinate, so every `at` is
    # assertable. That is the value this must report, and the piled arm
    # below is the one that must report the opposite.
    check(f"{name}: on a placed board every mechanical pose stands alone",
          all(not v['pose_shared_with'] for v in _mech.values()),
          str({r: v['pose_shared_with'] for r, v in _mech.items()
               if v['pose_shared_with']}))

# --------------------------------------------------------------------------
# THE PILE. Graded on three boards, because this is not a property of one:
# swept placed -> piled, ulx3s reports 19 -> 109 escape deficit lanes and
# locks.high 10 -> 0, kit-dev-coldfire 0 -> 82 lanes on a board with no
# escape problem at all, splitflap_driver locks.high 16 -> 2. Same schema,
# no marking anywhere.
#
# The pile is built IN MEMORY -- moving every footprint to the board centre
# and shifting its pads -- so this needs no new board file and cannot drift
# from the boards the rest of the suite uses.
# --------------------------------------------------------------------------
from placement.placement_state import assess_placement
from board_brief import POSITION_DEPENDENT, _null_path


def piled(path):
    """The board as `stage_unaided.stage()` leaves it, in memory.

    ROTATION 0 is not a detail -- `stage_unaided.py:118` writes it for every
    part it moves, and `extent_mm`, `rect` and the whole per-face escape
    table follow rotation. An earlier version of this helper translated the
    parts and KEPT their angles, so nothing rotation-derived could move and
    the arm below could not see any of it: an injection that rotated every
    face's demand by one face passed all 102 checks.
    """
    p = parse_kicad_pcb(path)
    b = p.board_info.board_bounds
    cx, cy = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
    for _r, _f in p.footprints.items():
        dx, dy = cx - _f.x, cy - _f.y
        _f.x, _f.y = cx, cy
        _f.rotation = 0.0
        for _p in _f.pads:
            _p.global_x += dx
            _p.global_y += dy
    return p


def dig(doc, path):
    """Follow one POSITION_DEPENDENT path to its value(s)."""
    node, parts = doc, path.split('.')
    for i, key in enumerate(parts):
        listy = key.endswith('[]')
        if listy:
            key = key[:-2]
        if not isinstance(node, dict) or key not in node:
            return []
        node = node[key]
        if listy:
            return [v for it in (node or ())
                    for v in dig(it, '.'.join(parts[i + 1:]))]
    return [node]


_demand_boards = []
for _bd in (BOARD, _w, os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb')):
    if not os.path.isfile(_bd):
        continue
    name = os.path.basename(_bd)
    _pcb = piled(_bd)
    check(f"{name}: the in-memory pile really is unplaced",
          assess_placement(_pcb, _bd).unplaced,
          "if this is False the whole arm below proves nothing")
    _pb = build_brief(_pcb, _bd, fit=[(5.0, 5.0)])
    _pd = _pb.get('position_dependent') or {}

    check(f"{name}: the brief discloses that it nulled things",
          bool(_pd.get('invalid')) and bool(_pd.get('because')),
          str(_pd)[:120])
    # EVERY path it names is actually null ...
    still = [p for p in _pd.get('invalid', ())
             if any(v is not None for v in dig(_pb, p))]
    check(f"{name}: every path named in `invalid` is really null",
          not still, str(still))
    # ... and nothing it did NOT name got nulled behind the reader's back.
    # Scanned over the WHOLE measurements/structure subtree, not just over
    # POSITION_DEPENDENT: ranging over the table made the "no others" half
    # definitionally true, and an injection that nulled `structure.top_nets`
    # without declaring it passed every check.
    def null_leaves(node, path=''):
        if isinstance(node, dict):
            for k, v in node.items():
                yield from null_leaves(v, f'{path}.{k}' if path else k)
        elif isinstance(node, list):
            for it in node:
                yield from null_leaves(it, path + '[]')
        elif node is None:
            yield path

    declared = set(_pd.get('invalid', ()))
    unnamed = sorted({p for sub in ('measurements', 'structure')
                      for p in null_leaves(_pb.get(sub), sub)
                      if p not in declared})
    check(f"{name}: `invalid` names every nulled path and no others",
          not unnamed,
          f"{unnamed} -- null with nothing in position_dependent explaining it")

    # The valid half SURVIVES. A caveat that nulled the netlist facts too
    # would be a refusal wearing a caveat's clothes.
    esc = (_pb['measurements'].get('escape') or {})
    check(f"{name}: the per-part escape numbers survive the pile",
          esc.get('parts') is not None
          and all(f.get('demand') is not None and f.get('span_mm') is not None
                  for p in (esc.get('worst') or []) for f in p['faces']),
          str({k: v for k, v in esc.items() if not isinstance(v, list)}))
    # ...but the per-FACE split follows rotation and a staged board carries
    # rotation 0, so it must be NAMED. It is kept, not nulled, because a
    # deficit is read off it -- and an injection that rotated every face's
    # demand by one face passed the check above, which is why this one is
    # here: the value is not asserted, its DISCLOSURE is.
    _named = {d['key'] for d in (_pd.get('also_pose_derived') or ())}
    check(f"{name}: the per-face escape table is named as pose-derived",
          any('faces[]' in k and 'demand' in k for k in _named), str(_named))
    check(f"{name}: so is extent_mm, which swaps W/H with rotation",
          any('extent_mm' in k for k in _named), str(_named))
    check(f"{name}: and the render fold-in, which draws the pile",
          any(k.startswith('renders') for k in _named), str(_named))
    check(f"{name}: worst[] is named as pile-SELECTED, not merely reordered",
          any('membership' in k for k in _named), str(_named))
    # The invariant the per-face split is NOT: a part's total demand.
    _tot = {p['ref']: sum(f['demand'] for f in p['faces'])
            for p in (esc.get('worst') or [])}
    _placed_esc = (build_brief(parse_kicad_pcb(_bd), _bd)['measurements']
                   .get('escape') or {})
    _ptot = {p['ref']: sum(f['demand'] for f in p['faces'])
             for p in (_placed_esc.get('worst') or [])}
    _shared = set(_tot) & set(_ptot)
    if _shared:
        _demand_boards.append(name)
        check(f"{name}: total demand per part IS rotation-invariant",
              all(_tot[r] == _ptot[r] for r in _shared),
              str({r: (_tot[r], _ptot[r]) for r in _shared
                   if _tot[r] != _ptot[r]}))
    tal = (_pb['measurements'].get('locks') or {}).get('tally') or {}
    check(f"{name}: locked_in_file is a FILE fact and survives",
          tal.get('locked_in_file') is not None, str(tal))
    check(f"{name}: the netlist structure survives",
          _pb['structure'].get('power_nets') is not None
          and _pb['structure'].get('diff_pairs') is not None)

    # fit probes the OUTLINE and never another part, so it is the section
    # that still answers "where could this go".
    _placed = build_brief(parse_kicad_pcb(_bd), _bd, fit=[(5.0, 5.0)])
    check(f"{name}: `fit` is identical placed and piled",
          _pb['fit'] == _placed['fit'],
          f"{_pb['fit']} vs {_placed['fit']}")

    # And the section that answers "which parts are fixed" must survive too,
    # while reporting honestly that the poses are now pile poses.
    _pm = (_pb.get('mechanical') or {}).get('refs') or {}
    _plm = (_placed.get('mechanical') or {}).get('refs') or {}
    check(f"{name}: mechanical still names its refs on the pile",
          _plm and set(_pm) == set(_plm),
          f"{sorted(_pm)} vs {sorted(_plm)}")
    check(f"{name}: and says every pose is now shared, so none is assertable",
          all(v['pose_shared_with'] for v in _pm.values()),
          str({r: v['pose_shared_with'] for r, v in _pm.items()
               if not v['pose_shared_with']}))

    check(f"{name}: format_text survives a nulled brief",
          len(format_text(_pb).splitlines()) > 4)
    # RELATIVE, not a fixed line window: a 12-line prefix happens to work on
    # these three boards and would stop biting on a shorter one.
    _lines = format_text(_pb).splitlines()
    _warn = next((i for i, ln in enumerate(_lines) if 'NULL' in ln), None)
    _num = next((i for i, ln in enumerate(_lines)
                 if ln.lstrip().startswith(('escape:', 'locks:', 'groups['))),
                None)
    check(f"{name}: and warns BEFORE the numbers, not after",
          _warn is not None and _num is not None and _warn < _num,
          f"caveat at line {_warn}, first number at line {_num}")

check("at least one board actually exercised the demand invariant",
      bool(_demand_boards),
      f"{_demand_boards} -- a board with no fine-pitch parts makes that "
      f"check vacuous, so it is skipped there and must bite somewhere")

# --------------------------------------------------------------------------
# THE PRODUCT PATH. The in-memory pile above is a model; this is the thing
# the toolchain actually produces, and the two disagree in the way that
# matters. `stage_unaided` exempts the mechanical parts, which keeps their
# real coordinates -- so `distinct_positions` is 8, not 1, on
# splitflap_driver, `assess_placement` returns partially_unplaced rather than
# unplaced, and a disclosure gated on `state.unplaced` alone DID NOT FIRE on
# 58-of-65 parts stacked at one point. Gate on the pile fraction here.
# --------------------------------------------------------------------------
import shutil
import tempfile

sys.path.insert(0, os.path.join(REPO, 'tests', 'stress'))
try:
    from stage_unaided import stage as _stage
except Exception as _e:                                      # noqa: BLE001
    _stage = None
    check("stage_unaided is importable (the product path arm needs it)",
          False, str(_e))

if _stage is not None:
    for _bd in (BOARD, _w):
        if not os.path.isfile(_bd):
            continue
        name = os.path.basename(_bd)
        _tmp = tempfile.mkdtemp(prefix='brief_staged_')
        try:
            _out = os.path.join(_tmp, 'board.kicad_pcb')
            _stage(_bd, _out, os.path.join(_tmp, 'truth'))
            _spcb = parse_kicad_pcb(_out)
            _sb = build_brief(_spcb, _out)
            _st = _sb['state']
            _spd = _sb.get('position_dependent') or {}
            check(f"{name} STAGED: the disclosure fires on the real stager's "
                  f"output", bool(_spd.get('invalid')),
                  f"unplaced={_st['unplaced']} "
                  f"partially={_st['partially_unplaced']} "
                  f"dup={_st['duplicate_fraction']} "
                  f"distinct={_st['signals'].get('distinct_positions')} -- "
                  f"gating on state.unplaced alone misses this board")
            check(f"{name} STAGED: and says which gate fired",
                  bool(_spd.get('gate')), str(_spd.get('gate')))
            check(f"{name} STAGED: the pile really is a pile "
                  f"(else the check above is free)",
                  (_st['duplicate_fraction'] or 0) >= 0.5,
                  str(_st['duplicate_fraction']))
            # The mechanical refs KEEP their real pose through staging --
            # that is what makes `place_fixed` writable from the brief.
            _sm = (_sb.get('mechanical') or {}).get('refs') or {}
            _om = (build_brief(parse_kicad_pcb(_bd), _bd)
                   .get('mechanical') or {}).get('refs') or {}
            check(f"{name} STAGED: every mechanical pose survives staging",
                  _sm and all(_sm[r]['at'] == _om[r]['at'] for r in _sm),
                  str({r: (_sm[r]['at'], _om[r]['at']) for r in _sm
                       if _sm[r]['at'] != _om[r]['at']}))
            check(f"{name} STAGED: and each is alone, so `at` is assertable",
                  all(not v['pose_shared_with'] for v in _sm.values()),
                  str({r: v['pose_shared_with'] for r, v in _sm.items()
                       if v['pose_shared_with']}))
            # The rotation-derived keys the note now names: prove they really
            # do move, so `also_pose_derived` is a live disclosure.
            _ob = build_brief(parse_kicad_pcb(_bd), _bd)
            swapped = [r for r, row in _sb['parts'].items()
                       if r in _ob['parts'] and r not in _sm
                       and row['extent_mm'] == list(
                           reversed(_ob['parts'][r]['extent_mm']))
                       and row['extent_mm'] != _ob['parts'][r]['extent_mm']]
            check(f"{name} STAGED: extent_mm really does swap on staged parts",
                  len(swapped) > 0,
                  f"{len(swapped)} part(s) transposed -- if this is 0 the "
                  f"`also_pose_derived` entry for extent_mm is not earning "
                  f"its place")
        finally:
            shutil.rmtree(_tmp, ignore_errors=True)

# A PLACED board must be untouched by all of this.
_pl = build_brief(parse_kicad_pcb(BOARD), BOARD)
check("a placed board gets no position_dependent block at all",
      'position_dependent' not in _pl, str(_pl.get('position_dependent')))
check("and its numbers are not nulled",
      (_pl['measurements'].get('escape') or {}).get('deficit_lanes')
      is not None)

# DERIVED FROM THE BRIEF, not from a hand-written list. Both of these used
# to enumerate section names, so they asserted that a dict literal contained
# the keys of the same dict literal and could never notice a new section
# that shipped unsourced -- which is exactly what `sources` exists to stop.
_META = {'schema', 'kind', 'clearance', 'board_edge_clearance', 'sources',
         'skipped', 'requirements', 'position_dependent'}
_pl_full = build_brief(parse_kicad_pcb(BOARD), BOARD, fit=[(5.0, 5.0)],
                       requirements='x')
_sections = {k for k, v in _pl_full.items()
             if k not in _META and isinstance(v, (dict, list))}
check("every section the brief emits names the function that produced it",
      _sections and _sections <= set(_pl_full['sources']),
      f"unsourced: {sorted(_sections - set(_pl_full['sources']))}")
check("and the fixture really has the sections worth checking",
      {'board', 'state', 'parts', 'mechanical', 'structure', 'measurements',
       'fit'} <= _sections, str(sorted(_sections)))
check("and each source says what it REQUIRES of the input",
      all('[requires:' in _pl_full['sources'][k] for k in _sections),
      str([k for k in _sections if '[requires:' not in _pl_full['sources'][k]]))

print(f"\n{passed} passed, {failed} failed")
sys.exit(1 if failed else 0)
