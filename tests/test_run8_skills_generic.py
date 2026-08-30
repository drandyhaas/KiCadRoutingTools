#!/usr/bin/env python3
"""A skill states rules, not board names.

The skills are meant to work on ANY KiCad board. Every rule in them has to be
a test on the board in front of the reader -- a detection predicate and a
threshold with its units -- never "on <that board>, <that part> did <thing>".
Measured evidence is what makes these rules trustworthy and it stays, but
anonymised: "a 92-part 2-layer board", not the board's name.

This is a banlist, and a banlist is only as good as its list. It carries the
names of every board in the in-repo corpus (which is where a specific example
would most plausibly come from), the work-dir paths that only exist during an
experiment, and the reference numbers of the experiment runs when used as
authority.

Run: python3 -X utf8 tests/test_run8_skills_generic.py
"""
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKILLS = os.path.join(ROOT, '.claude', 'skills')
OWNED = ('plan-pcb-placement', 'plan-pcb-routing',
         'plan-pcb-placement-and-routing')

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def corpus_names():
    """Board names a specific example would most plausibly be drawn from."""
    out = set()
    for f in glob.glob(os.path.join(ROOT, 'kicad_files', '*.kicad_pcb')):
        stem = os.path.basename(f)[:-len('.kicad_pcb')]
        # Only distinctive stems: 'routed_output' or 'cap_chain' are ordinary
        # English and would fire on prose.
        if len(stem) >= 6 and not stem.replace('_', '').isalpha():
            out.add(stem)
        elif stem in ('tigard', 'watchy', 'ulx3s', 'orangecrab_ext_pll',
                      'glasgow_revC', 'splitflap_driver', 'esp_prog',
                      'flat_hierarchy', 'sonde_u'):
            out.add(stem)
    return out


def skill_files():
    for name in OWNED:
        d = os.path.join(SKILLS, name)
        if not os.path.isdir(d):
            continue
        for root, _dirs, files in os.walk(d):
            for f in sorted(files):
                if f.endswith(('.md', '.py')):
                    yield os.path.join(root, f)


def main():
    files = list(skill_files())
    check('the three skills exist', len(files) >= 3,
          str([os.path.relpath(f, SKILLS) for f in files]))

    banned = corpus_names()
    print(f'  ({len(banned)} corpus board name(s) on the banlist)')

    hits = []
    for path in files:
        text = open(path, encoding='utf-8').read()
        rel = os.path.relpath(path, ROOT)
        for name in sorted(banned):
            for m in re.finditer(re.escape(name), text):
                line = text.count('\n', 0, m.start()) + 1
                hits.append(f'{rel}:{line}: board name {name!r}')
        # Work-dir paths exist only inside an experiment.
        for m in re.finditer(r'\bwk/run\d+\b', text):
            line = text.count('\n', 0, m.start()) + 1
            hits.append(f'{rel}:{line}: experiment work dir {m.group(0)!r}')

    check('no skill names a specific board or work dir', not hits,
          '\n        '.join(hits[:12]))

    print('the skills stay separable')
    place = os.path.join(SKILLS, 'plan-pcb-placement', 'SKILL.md')
    route = os.path.join(SKILLS, 'plan-pcb-routing', 'SKILL.md')
    both = os.path.join(SKILLS, 'plan-pcb-placement-and-routing', 'SKILL.md')
    for p in (place, route, both):
        check(f'{os.path.basename(os.path.dirname(p))} has front matter',
              open(p, encoding='utf-8').read().startswith('---\n'))

    ptext = open(place, encoding='utf-8').read()
    rtext = open(route, encoding='utf-8').read()
    btext = open(both, encoding='utf-8').read()

    check('the placement skill owns the placement gate',
          '## Step 0: Placement gate' in ptext)
    # ca6bb455 deliberately reshaped both of these, so the old expectations
    # are stale rather than violated:
    #   * the routing skill is now main's file VERBATIM (it had grown from
    #     2293 to 5479 lines with placement-domain sections). It therefore does
    #     NOT point at /plan-pcb-placement -- the COMBINED skill owns the
    #     cross-references, which is what we check instead.
    #   * the combined skill absorbed Step 9's 892-line convergence loop, the
    #     hub of the place->route cycle, so "thin" no longer describes it. The
    #     property that still matters -- it must not RESTATE either half -- is
    #     checked separately just below and is left untouched.
    check('the combined skill points at both halves',
          '/plan-pcb-placement' in btext and '/plan-pcb-routing' in btext)
    check('the routing skill kept its routing half',
          'Step 1: Load and Analyze PCB Structure' in rtext)
    check('the combined skill does not restate either half',
          'Step 0a-0' not in btext and 'Load and Analyze' not in btext)

    print('the load-bearing rules survived the cut')
    for label, needle, where in [
            ('outline prohibition', 'OUTLINE IS NOT YOURS TO CHANGE',
             ptext + rtext),
            ('the placement gate is a measurement, not a default',
             'measure first, then decide', ptext + rtext),
            ('a clean placement is left alone, as a CONSEQUENCE of measuring',
             'WORSE by a polish pass', ptext),
            ('locks are never the tool\'s to move', 'locked yes', ptext),
            ('crossings is not a gate', 'crossings', ptext),
            ('placement invalidates downstream boards', 'invalidates',
             ptext + rtext + btext)]:
        check(f'{label} is still stated', needle in where)

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
