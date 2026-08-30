"""`place_seed --reseat-region`: naming a reseat scope by GEOMETRY (#459/#709).

#459's 2026-08-22 comment names what was missing: "`reseat_scope` already
lifts a named set ... what is missing is a way to name a REGION rather than a
ref list, plus a gate that can accept an on-board part." The gate half landed
as #698. This is the region half, and the two things that make it correct are
both easy to get wrong:

1. THE RECTANGLE MUST MEAN ONE THING. `check_pockets` prints a rectangle and
   this lifts the parts in it. Both go through `placement.utility.refs_in_rect`
   -- not two lookalike pad scans that agree until they do not.

2. THE SCOPE SOURCE MUST STAY `'explicit'`. `seeder.reseat_accept` switches on
   `scope_source != 'explicit'` by STRING EQUALITY (seeder.py:2834). A
   'region:...' source would drop every region re-seat into the
   `auto:oob-strict` policy, which a legal on-board part can never satisfy --
   the pass would refuse everything and read as broken rather than as wrong.
   So the region is resolved to refs at the CLI layer and handed to the
   ordinary `refs=` argument, and the provenance is reported in a SEPARATE
   key.

And the claim this makes must not be bigger than the thing: a re-seat lands
each part at its own net centroid, so pointing it at an empty region does not
move anything INTO that region. That is intent-zone work, and the help text
and the census both say so.

# Comment above the marker, never after it -- a trailing one voids run_all's
# `...True\s*$` anchor.
"""

import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 600

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from run_utils import check, evidence, tool                     # noqa: E402
import test_698_reseat_acceptance as F698                       # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from placement.utility import refs_in_rect                      # noqa: E402

SEED = tool('place_seed.py')
POCKETS = tool('check_pockets.py')

FAILURES = []


def report(name, ok, detail=''):
    # A PASS prints its label only; a FAIL prints the evidence. Dumping a
    # whole stdout beside every green row buries the one red one.
    d = ' '.join(str(detail).split())[:300]
    print(('  PASS  ' + name) if ok
          else ('  FAIL  ' + name + (('  -- ' + d) if d else '')))
    if not ok:
        FAILURES.append(name)


def _argv(*extra):
    return [sys.executable, '-X', 'utf8', SEED] + list(extra)


def _fixture(wd):
    """test_698's plain board: CON2/U1/U2 clustered, R8/R9 off to the side."""
    b = F698.plain_board(wd)
    _p, it = F698.plain_intent(wd)
    return evidence(b, 'the region fixture board'), _p


def t_a_region_names_the_same_parts_the_census_does():
    """Property 1, on a REAL board and through both fronts.

    check_pockets resolves a window's refs and place_seed resolves a region's
    refs. Same rectangle, same call, so the answers cannot drift.
    """
    board = evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'))
    pcb = parse_kicad_pcb(board)
    with tempfile.TemporaryDirectory() as td:
        jp = os.path.join(td, 'p.json')
        r = check([sys.executable, '-X', 'utf8', POCKETS, board,
                   '--json', jp, '--top', '4'], accept=True)
        doc = json.load(open(jp, encoding='utf-8'))
    hot = [w for w in doc['windows'] if w.get('refs')]
    report('the census names refs on at least one window', bool(hot),
           '%d windows' % len(doc['windows']))
    if not hot:
        return
    disagree = [w['window'] for w in hot
                if refs_in_rect(pcb, tuple(w['window'])) != w['refs']]
    report('every census window resolves to the same refs the mover would lift',
           not disagree, str(disagree[:2]))
    rt = doc.get('reseat_target')
    report('the census prints a reseat_region argument', bool(rt), str(rt))
    if rt:
        report('  ...and it is a well-formed rectangle',
               rt['reseat_region'][2] > rt['reseat_region'][0]
               and rt['reseat_region'][3] > rt['reseat_region'][1],
               str(rt['reseat_region']))
    report('the census run itself is clean', r.returncode == 0)


def t_the_policy_stays_explicit():
    """Property 2: the acceptance basis must be the EXPLICIT one.

    If this ever reads `auto:oob-strict`, a region re-seat of a legal on-board
    part can never be accepted and the flag is decorative.
    """
    with tempfile.TemporaryDirectory() as wd:
        b, it = _fixture(wd)
        out = os.path.join(wd, 'out.kicad_pcb')
        # A rectangle around the CON2/U1/U2 cluster.
        r = check(_argv(b, out, '--intent', it, '--dry-run',
                        '--reseat-region', '0', '0', '20', '14'), accept=True)
        txt = r.stdout
        report('the run is clean', r.returncode == 0, txt[-300:])
        report('the region line names its parts', 'region [0,0]-[20,14]' in txt,
               txt[:400])
        report('the scope is reported EXPLICIT, not auto',
               'Reseat (explicit)' in txt,
               next((ln for ln in txt.splitlines()
                     if ln.startswith('Reseat')), '<no Reseat line>'))
        report('the selector is disclosed beside it, not inside scope_source',
               'scope_selector: region:0,0,20,14' in txt,
               next((ln for ln in txt.splitlines()
                     if 'scope_selector' in ln), '<none>'))
        line = next((ln for ln in txt.splitlines()
                     if ln.startswith('JSON_SUMMARY: ')), None)
        report('a JSON_SUMMARY line is emitted', line is not None)
        if line:
            doc = json.loads(line[len('JSON_SUMMARY: '):])
            report('the summary carries scope_source == explicit',
                   doc.get('scope_source') == 'explicit',
                   str(doc.get('scope_source')))
            report('  ...and scope_selector as a SEPARATE key',
                   (doc.get('scope_selector') or '').startswith('region:'),
                   str(doc.get('scope_selector')))
            ab = doc.get('accept_basis') or {}
            report('  ...and the accept basis is an explicit policy',
                   str(ab.get('policy', '')).startswith('explicit'),
                   str(ab.get('policy')))


def t_an_empty_region_is_a_result_not_a_failure():
    """It must NOT fall through to `refs=None`, which is the AUTO scope under
    a different acceptance rule entirely."""
    with tempfile.TemporaryDirectory() as wd:
        b, it = _fixture(wd)
        out = os.path.join(wd, 'out.kicad_pcb')
        r = check(_argv(b, out, '--intent', it, '--dry-run',
                        '--reseat-region', '900', '900', '910', '910'),
                  accept=True)
        report('an empty region exits 0', r.returncode == 0, r.stdout[-300:])
        report('  ...and says the region is empty, in those words',
               'contain no part' in r.stdout, r.stdout[-300:])
        report('  ...and does NOT silently become the auto scope',
               'auto:damage_witnesses' not in r.stdout
               and 'Reseat (auto' not in r.stdout, r.stdout[-300:])


def t_a_bare_reseat_beside_a_region_is_refused():
    """#698 gave the two scopes different acceptance rules; merging them
    silently is how that distinction disappears."""
    with tempfile.TemporaryDirectory() as wd:
        b, it = _fixture(wd)
        out = os.path.join(wd, 'out.kicad_pcb')
        check(_argv(b, out, '--intent', it, '--reseat',
                    '--reseat-region', '0', '0', '20', '14'),
              code=2, refuse='different rules',
              allow=('error: argument',))
        report('a bare --reseat beside a region is refused for that reason',
               True)


def t_an_inverted_rectangle_is_refused():
    with tempfile.TemporaryDirectory() as wd:
        b, it = _fixture(wd)
        out = os.path.join(wd, 'out.kicad_pcb')
        check(_argv(b, out, '--intent', it,
                    '--reseat-region', '20', '14', '0', '0'),
              code=2, refuse='X1>X0', allow=('error: argument',))
        check(_argv(b, out, '--intent', it,
                    '--reseat-region', '0', '0', '20', '0'),
              code=2, refuse='Y1>Y0', allow=('error: argument',))
        report('an inverted / degenerate rectangle is refused by its reason',
               True)


def t_a_region_alone_implies_an_explicit_reseat():
    """Typing --reseat too must not be required; a region IS a scope."""
    with tempfile.TemporaryDirectory() as wd:
        b, it = _fixture(wd)
        out = os.path.join(wd, 'out.kicad_pcb')
        r = check(_argv(b, out, '--intent', it, '--dry-run',
                        '--reseat-region', '0', '0', '20', '14'), accept=True)
        report('--reseat-region alone runs a re-seat',
               'Reseat (explicit)' in r.stdout, r.stdout[-300:])
        report('  ...and --dry-run is accepted without --reseat being typed',
               r.returncode == 0)


def t_the_union_with_named_refs():
    with tempfile.TemporaryDirectory() as wd:
        b, it = _fixture(wd)
        out = os.path.join(wd, 'out.kicad_pcb')
        r = check(_argv(b, out, '--intent', it, '--dry-run',
                        '--reseat', 'R8',
                        '--reseat-region', '0', '0', '20', '14'), accept=True)
        report('named refs and a region union into one explicit scope',
               'Reseat (explicit)' in r.stdout, r.stdout[-300:])
        line = next((ln for ln in r.stdout.splitlines()
                     if 'scope_selector' in ln), '')
        report('  ...and the split is disclosed (n from the region, m named)',
               'from the region' in line and '1 named' in line, line)


TESTS = [
    ('one rectangle, one answer', t_a_region_names_the_same_parts_the_census_does),
    ('the policy stays explicit', t_the_policy_stays_explicit),
    ('an empty region is a result', t_an_empty_region_is_a_result_not_a_failure),
    ('a bare --reseat beside a region is refused',
     t_a_bare_reseat_beside_a_region_is_refused),
    ('an inverted rectangle is refused', t_an_inverted_rectangle_is_refused),
    ('a region alone implies --reseat', t_a_region_alone_implies_an_explicit_reseat),
    ('union with named refs', t_the_union_with_named_refs),
]


def _every_case_is_registered():
    g = globals()
    declared = {fn for _l, fn in TESTS}
    missing = sorted(n for n, v in g.items()
                     if n.startswith('t_') and callable(v)
                     and v not in declared)
    if missing:
        print('  FAIL  every t_* case is registered in TESTS  -- ORPHANED: %s'
              % ', '.join(missing))
        FAILURES.append('unregistered cases: %s' % ', '.join(missing))


def main():
    print('place_seed --reseat-region (#459 region half / #709 handoff)')
    for label, fn in TESTS:
        print(' ' + label)
        fn()
    _every_case_is_registered()
    if FAILURES:
        print('\nFAILED (%d): %s' % (len(FAILURES), ', '.join(FAILURES)))
        return 1
    print('\nOK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
