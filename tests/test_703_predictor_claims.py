#!/usr/bin/env python3
"""Every correlation numeral in this repo says which variable it was measured against (#703).

THE DEFECT THIS STANDS AGAINST

`r(crossings) = +0.780` is a real measurement. It is measured against
*distance-to-the-correct-placement*, over 29 candidates, on ONE board. The
corridor law's `r = +0.41 .. +0.90` is measured against *the gap a human left*.
Neither is measured against routed `blocking`, which CLAUDE.md's "What a
placement run is FOR" names as the only right headline -- and by the time #703
was filed those numbers had been copied to a dozen sites, one of them a verifier
prompt an LLM reads verbatim, with the dependent variable dropped somewhere
along the way.

A one-time sweep fixes the sites that exist today and nothing else. This is the
standing gate: a NEW site that quotes one of these numerals without naming what
it was measured against fails here, on the day it is written.

WHY BY CHARACTER DISTANCE AND NOT BY LINE

The scoping clause is usually on the next line -- these files are wrapped at 79
columns and the numeral rarely shares a line with the phrase that qualifies it.
A line-based grep therefore reports a correctly-scoped site as a violation and
teaches its reader to ignore the output. The window is characters in the file,
which is what "next to" actually means in a wrapped document.

BOTH DIRECTIONS, so it cannot rot

`REGISTERED` names every file that legitimately carries one of these numerals.
A numeral in an unregistered file fails (a new site). A registered file that no
longer carries one is reported as stale (the sweep moved on and nobody updated
the map). That is `test_718_static_test_hygiene.py`'s doctrine, for the same
reason: a one-directional map decays into a list of things that used to matter.

    python3 -X utf8 tests/test_703_predictor_claims.py
"""
import io
import os
import re
import sys

RUN_ALL_TIMEOUT = 120
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: The numerals that ARE correlation coefficients in this repo. Anchored so a
#: number that merely looks like one cannot match: `plan-pcb-routing/SKILL.md`
#: carries "+0.78 pts completion", which is points of routing completion from a
#: rip-up depth A/B and has nothing to do with a correlation. It is the reason
#: this pattern requires an `r`-ish prefix rather than matching bare digits, and
#: `t_no_false_positive_on_completion_points` pins that exclusion.
#:
#: THE EXPLICIT SIGN IS THE DISCRIMINATOR for the bare `r = ` form. This repo is
#: full of `r=0.3` and `r = 0.15` -- they are RADII in clearance fixtures, 39
#: files of them, and a pattern that matched those would demand a
#: dependent-variable clause on a millimetre. Every real correlation here is
#: written with its sign (`+0.780`, `+0.72`, `+0.41`), because the sign is the
#: interesting half of a correlation and nobody writes a signed radius. The
#: `r(name) = ` form needs no sign: the parenthesised variable is already
#: unambiguous.
NUMERAL = re.compile(
    r'r\s*\(\s*\w+\s*\)\s*=\s*[+-]?0\.\d+'      # r(crossings) = +0.780
    r'|\br\s*=\s*[+-]0\.\d+'                    # r = +0.78 / r=+0.72
    r'|\brho\s*=\s*[+-]0\.\d+')                 # rho = +0.339

#: What a properly-scoped citation must say within WINDOW characters.
SCOPES = ('distance-to-truth', 'distance-to-the-correct-placement',
          "human's gap", 'human left', 'THE GAP A HUMAN LEFT',
          'placement-predictors.md', 'DISTANCE, not routed',
          'not routed `blocking`', 'not against routed', 'never with routed',
          'never against routed', 'NOT against routed', 'not with routed',
          # `rho=+0.339 [LOO +0.053..+0.632, K=6]` is `rank_stats.fmt_rho`'s
          # output, and the bracket IS the scope: the leave-one-out span and K
          # travel with the number by construction. Demanding a prose clause
          # beside a token that already carries its own is how a gate teaches
          # people to work around it.
          'LOO')
WINDOW = 420

#: Files that legitimately carry a correlation numeral. Both directions: an
#: unregistered file carrying one FAILS, and a registered file that no longer
#: carries one is reported as stale.
REGISTERED = {
    '.claude/skills/plan-pcb-placement/SKILL.md',
    '.claude/skills/plan-pcb-placement/scripts/placement_driver.py',
    '.claude/skills/plan-pcb-placement-and-routing/references/evidence-map.md',
    '.claude/skills/plan-pcb-placement-and-routing/references/verifier-prompts.md',
    '.claude/skills/plan-pcb-placement-and-routing/scripts/loop_driver.py',
    'py_placer/placement/reconstruct.py',
    'py_placer/placement/routability.py',
    # #553's mover ranking, which quotes the legality rows in order to say
    # exactly what they do and do not license: they were measured as
    # BOARD-level scalars ranking a BOARD-level outcome, and this module uses
    # them as PER-CANDIDATE counts within one board.
    'py_placer/placement/diagnosis.py',
    'tests/test_run8_gate_conjuncts.py',
    'docs/placement-optimization.md',
    # The measurement apparatus itself, which quotes the numbers in order to
    # say what is wrong with how they were quoted.
    'py_placer/placement/portfolio.py',
    'docs/placement-predictors.md',
    'tests/stress/rank_stats.py',
    'tests/stress/predictor_study.py',
    'tests/test_703_rank_stats.py',
    'tests/test_703_predictor_claims.py',
}

SEARCH_DIRS = ('.claude/skills', 'docs', 'py_placer', 'py_router', 'py_tools',
               'tests', 'kicad_routing_plugin')
SEARCH_EXT = ('.md', '.py')
SKIP_PARTS = ('.claude/worktrees', '__pycache__', os.path.join('wk', ''))

FAILURES = []


def check(cond, what):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}')
        FAILURES.append(what)


def walk():
    for d in SEARCH_DIRS:
        base = os.path.join(ROOT, d)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [x for x in dirnames
                           if x not in ('__pycache__', 'worktrees', 'target')]
            for fn in filenames:
                if not fn.endswith(SEARCH_EXT):
                    continue
                p = os.path.join(dirpath, fn)
                rel = os.path.relpath(p, ROOT).replace('\\', '/')
                if any(s.replace('\\', '/') in rel for s in SKIP_PARTS if s):
                    continue
                yield rel, p


def hits_in(path):
    try:
        s = io.open(path, encoding='utf-8', errors='replace').read()
    except OSError:
        return []
    s = s.replace('\r\n', '\n')
    out = []
    for m in NUMERAL.finditer(s):
        lo = max(0, m.start() - WINDOW)
        hi = min(len(s), m.end() + WINDOW)
        # CASE-INSENSITIVE. These sites shout when they mean it -- one of them
        # writes "against DISTANCE-TO-TRUTH" in capitals -- and a gate that
        # reads a correctly-scoped citation as a violation because the author
        # emphasised it is a gate people learn to route around.
        ctx = s[lo:hi].lower()
        scoped = any(k.lower() in ctx for k in SCOPES)
        line = s.count('\n', 0, m.start()) + 1
        out.append({'text': m.group(0), 'line': line, 'scoped': scoped})
    return out


def t_every_numeral_is_scoped():
    unscoped = []
    carrying = set()
    for rel, path in walk():
        hs = hits_in(path)
        if hs:
            carrying.add(rel)
        for h in hs:
            if not h['scoped']:
                unscoped.append(f'{rel}:{h["line"]}  {h["text"]}')
    check(not unscoped,
          'every correlation numeral names its dependent variable within '
          f'{WINDOW} chars'
          + ('' if not unscoped else
             '\n         UNSCOPED:\n         ' + '\n         '.join(unscoped)))
    return carrying


def t_registration_holds_both_ways(carrying):
    new = sorted(carrying - REGISTERED)
    check(not new,
          'no UNREGISTERED file carries a correlation numeral'
          + ('' if not new else
             f'\n         NEW SITES (add to REGISTERED and scope them): {new}'))
    stale = sorted(REGISTERED - carrying)
    check(not stale,
          'no REGISTERED file has stopped carrying one'
          + ('' if not stale else
             f'\n         STALE (the sweep moved on; drop them): {stale}'))


def t_no_false_positive_on_completion_points():
    """The samples below quote r-values against distance-to-truth on purpose.

    They are pattern fixtures, not citations, and the sentence above is what
    keeps this function's own block scoped -- the gate holds itself to its rule
    rather than exempting the file that defines it.

    `+0.78 pts completion` is a routing-completion delta from a rip-up depth
    A/B, not a correlation. If the pattern matched it, this gate would demand a
    dependent-variable clause on a number that has none -- and the resulting
    noise is how a standing gate gets switched off."""
    for sample in ('`--max-ripup 5` beat 10 (+0.78 pts completion, 13 fewer',
                   'improved by 0.78 points', 'version 0.780 of the tool',
                   # RADII, of which this repo has 39 files' worth. An
                   # unsigned `r=` is a millimetre, not a coefficient.
                   'r=0.3', 'r = 0.15', 'clearance r=0.25 keepout',
                   'pad_shape_distance(r=0.75)'):
        check(not NUMERAL.search(sample),
              f'not a correlation, not matched: {sample[:46]!r}')
    # Every numeral below is a real citation from this repo, and every one of
    # them is measured against distance-to-truth or against the human's gap --
    # never against routed `blocking`. The clause is HERE, beside the samples,
    # rather than up in the docstring, because the docstring is ~1100 chars away
    # and this gate's own WINDOW is 420. It failed itself on exactly that, which
    # is the right outcome: a gate that exempts the file defining it is not a
    # gate. See `docs/placement-predictors.md`.
    for sample in ('r(crossings) = +0.780', 'r = +0.78', 'r=+0.72',
                   'r = +0.41'):
        check(bool(NUMERAL.search(sample)),
              f'IS a correlation, matched: {sample!r}')


def t_the_canonical_site_is_corrected():
    """The one site every other one cites. If it regresses, the rest follow."""
    p = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                     'SKILL.md')
    s = io.open(p, encoding='utf-8').read().replace('\r\n', '\n')
    check('r(crossings) = +0.780' in s,
          'the measurement is KEPT -- annotate, never delete')
    i = s.index('r(crossings) = +0.780')
    ctx = s[i:i + WINDOW]
    check('NOT against\n   routed `blocking`' in ctx or
          'NOT against routed `blocking`' in ctx.replace('\n   ', ' '),
          'and it says NOT against routed blocking, in as many words')
    check('placement-predictors.md' in ctx,
          'and points at the file that records what HAS been measured')


def main():
    print('t_no_false_positive_on_completion_points:')
    t_no_false_positive_on_completion_points()
    print('t_the_canonical_site_is_corrected:')
    t_the_canonical_site_is_corrected()
    print('t_every_numeral_is_scoped:')
    carrying = t_every_numeral_is_scoped()
    print('t_registration_holds_both_ways:')
    t_registration_holds_both_ways(carrying)
    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_703_predictor_claims: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
