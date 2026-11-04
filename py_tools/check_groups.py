#!/usr/bin/env python3
"""Check a groups file against its board, before `place_simple.py` places it.

The groups -- which parts belong together functionally -- are an agent's
judgement (see `.claude/skills/plan-pcb-placement/SKILL.md`). This instrument
refuses a file that cannot be placed and measures the judgement so it can be
checked before any part moves:

  ERRORS (exit 4)  a movable part in no group, a part in two groups, a ref the
                   board does not have, a group with no part on the board.
                   A part is fixed, not movable, when the board locks it or
                   the intent (`--intent`) seats it at a declared pose
                   (`fixed_poses[]`), as place_simple holds it.
  WARNINGS         a part with more SIGNAL links to another group than to its
                   own (rails left out: `placement.grouping.rail_net_ids`); a
                   decap whose IC is in another group and no IC of its own
                   group is on its rail (`place_simple` then leaves it out of
                   the decap term: right for a load or RC cap, wrong for a
                   real decap).
  REPORTED         bridges (2-pad parts linking two groups), the inter-group
                   signal-link matrix, and the rails.
  TEMPLATES        the template candidates the groups name (`templates`: a
                   switching stage or an RF front end whose datasheet layout
                   the skill's §3.5 copies), each with its state against the
                   intent (templated, skipped and why, or no row yet); a WARN
                   for a template member in another group than its chip's
                   (place_simple moves it there, as it moves a claimed part);
                   and as HINTs the chips the board's pins and nets suggest
                   that no group names or records as `none`
                   (`placement.grouping.template_hints`): the agent weighs
                   them; they never fail the check.
  CLAIMS           the parts place_simple seats by a proximity claim -- the
                   intent's `proximity[]` rows, and each passive crystal with
                   its load caps at its IC's oscillator pins
                   (`placement/crystals.py`) -- and the group each follows
                   its partner into (`moved`), its group's other parts with
                   it when it is the anchor. A group all of whose parts
                   leave is DISSOLVED (`dissolved`): a WARN names the group
                   that absorbs it and its own `near` / `apart` / `edge`
                   hints, which are dropped, and the group table lists the
                   groups as they will be placed. A claimed cap is no decap
                   conflict (`--no-crystals`: checked as if no crystal claim
                   were written).

    python3 py_tools/check_groups.py board.kicad_pcb --groups groups.json --intent intent.json
    python3 py_tools/check_groups.py board.kicad_pcb --groups groups.json --json out.json

Exit codes: 0 placeable (warnings allowed), 2 unreadable input, 4 errors.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import sys

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement.grouping import (GroupsError, analyze, claim_rows,  # noqa: E402
                                fold_groups, fold_text, follower_roots, load_groups,
                                template_candidates, template_hints)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('board', help='the board the groups are for')
    ap.add_argument('--groups', required=True, help='the groups JSON file')
    ap.add_argument('--json', dest='json_out', help='write the full report here')
    ap.add_argument('--intent', help="the intent place_simple will read (check_floorplan "
                    "--emit-intent): the parts it seats at a declared pose are "
                    "fixed, and its proximity claims are followed, as place_simple does")
    ap.add_argument('--no-crystals', action='store_true',
                    help="check as if no crystal claim were written")
    a = ap.parse_args(argv)
    try:
        doc = load_groups(a.groups)
    except GroupsError as ex:
        print(f"check_groups: {ex}", file=sys.stderr)
        return 2
    try:
        pcb = parse_kicad_pcb(a.board)
    except Exception as ex:  # an unreadable board is an input error, not a finding
        print(f"check_groups: cannot read {a.board}: {ex}", file=sys.stderr)
        return 2
    intent_doc, seated, tmpls = {}, [], []
    if a.intent:
        from placement import floorplan
        try:
            with open(a.intent, encoding='utf-8') as f:
                intent_doc = json.load(f)
            intent = floorplan.load_intent(a.intent)
        except (OSError, ValueError) as ex:
            print(f"check_groups: cannot read {a.intent}: {ex}", file=sys.stderr)
            return 2
        seated = sorted({str(f['ref']) for f in intent.fixed_poses
                         if str(f['ref']) in (pcb.footprints or {})})
        tmpls = list(intent.templates) + intent.cover_templates()
    rows, crystal_notes = claim_rows(pcb, intent_doc, crystals=not a.no_crystals)
    followers = sorted(follower_roots(rows))
    for line in crystal_notes:
        print(f"  {line} -- seated by its claim, beside the IC")
    if seated:
        print(f"  at a declared pose (fixed): {', '.join(seated)}")
    # a claimed part, and a template's member, follow their partner into its
    # group, as place_simple folds them (#1467: a group they empty is dropped)
    fold = fold_groups(doc, rows, tmpls)
    moved, gone = fold['moved'], {d['group']: d for d in fold['dissolved']}
    moved_line, dissolved_lines = fold_text(fold)
    if moved_line:
        print(f"  {moved_line}")
    rep = analyze(doc, pcb, followers=followers, extra_fixed=seated)
    rep['warnings'] += dissolved_lines
    rep['moved'] = {r: list(ft) for r, ft in sorted(moved.items())}
    rep['dissolved'] = fold['dissolved']

    # the groups as place_simple places them: a part that follows a claim is
    # listed in its partner's group, and a dissolved group says where it went
    held = {k: {r for g in rep['groups'] for r in g[k]} for k in ('movable', 'fixed')}
    for g in rep['groups']:
        role = g.get('role') or '-'
        d = gone.get(g['name'])
        if d:
            print(f"  {g['name']:<14} {role:<11} dissolved into {d['into']}: " + '; '.join(
                f"{', '.join(rs)} -> {t}" for t, rs in sorted(d['parts'].items())))
            continue
        come = sorted(r for r, (_f, t) in moved.items() if t == g['name'])
        now = {k: [r for r in g[k] if r not in moved]
               + [f"{r} (from {moved[r][0]})" for r in come if r in held[k]]
               for k in ('movable', 'fixed')}
        fixed = f"  fixed: {', '.join(now['fixed'])}" if now['fixed'] else ''
        print(f"  {g['name']:<14} {role:<11} "
              f"{len(now['movable'])} movable: {', '.join(now['movable'])}{fixed}")
    if rep['links']:
        print("  signal links between groups (rails excluded):")
        for row in rep['links']:
            print(f"    {row['a']} <-> {row['b']}: {row['signal_links']:.3g}")
    print(f"  rails: {', '.join(rep['rails']) or 'none'}")
    if rep['bridges']:
        print("  bridges: " + ', '.join(f"{b['ref']} ({b['group']} | {'/'.join(b['borders'])})"
                                        for b in rep['bridges']))
    cands = template_candidates(doc)
    rowed = {t['anchor'] for t in tmpls}
    for t in cands:
        state = (f"skipped: {t['skip']}" if t.get('skip') else
                 'templated' if t['chip'] in rowed else
                 'no templates[] row in the intent yet' if a.intent else '')
        print(f"  template candidate {t['chip']} ({t['kind']}, group {t['group']}): {t['why']}"
              + (f" -- {state}" if state else ''))
    hints = template_hints(doc, pcb)
    for h in hints:
        print(f"  HINT  {h}")
    gof = {r: g['name'] for g in doc['groups'] for r in g['refs']}
    for t in tmpls:
        off = [m['ref'] for m in t['members'] if gof.get(m['ref']) not in (None, gof.get(t['anchor']))]
        if off:
            rep['warnings'].append(
                f"template {t['anchor']}: {', '.join(f'{r} ({gof[r]})' for r in off)} "
                f"not in {t['anchor']}'s group {gof.get(t['anchor'])!r}; place_simple moves "
                f"them into it")
    for w in rep['warnings']:
        print(f"  WARN  {w}")
    for e in rep['errors']:
        print(f"  ERROR {e}")
    if a.json_out:
        with open(a.json_out, 'w', encoding='utf-8') as f:
            json.dump(rep, f, indent=1, sort_keys=True)
    summary = {'errors': len(rep['errors']), 'warnings': len(rep['warnings']),
               'groups': len(rep['groups']), 'bridges': len(rep['bridges']),
               'pulled': [p['ref'] for p in rep['pulled']],
               'decap_cross_group': [d['cap'] for d in rep['decap_cross_group']],
               'crystals': crystal_notes, 'followers': followers,
               'moved': rep['moved'], 'dissolved': sorted(gone),
               'seated': seated,
               'template_candidates': [t['chip'] for t in cands], 'template_hints': len(hints)}
    print('JSON_SUMMARY: ' + json.dumps(summary, sort_keys=True))
    return 4 if rep['errors'] else 0


if __name__ == '__main__':
    import cli_banner
    cli_banner.install()   # CMD/EXIT self-echo (run-3 B1)
    sys.exit(main())
