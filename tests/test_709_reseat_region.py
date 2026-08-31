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
    report('the census names a landing site', bool(rt), str(rt))
    if rt:
        report('  ...as a well-formed rectangle',
               rt['zone'][2] > rt['zone'][0] and rt['zone'][3] > rt['zone'][1],
               str(rt['zone']))
        # ...and deliberately NOT as a --reseat-region argument. A cold band
        # holds no part by construction, so that command resolves to an empty
        # scope on every board -- which is what the census used to advertise.
        report('  ...and NOT as a --reseat-region argument',
               'reseat_region' not in rt, str(sorted(rt)))
        report('  ...which the mover agrees with: the zone lifts nothing',
               refs_in_rect(pcb, tuple(rt['zone'])) == [],
               str(refs_in_rect(pcb, tuple(rt['zone']))))
    report('the census run itself is clean', r.returncode == 0)


def t_the_mover_resolves_the_rectangle_THE_SAME_WAY():
    """The property the whole design rests on, tested where it can fail.

    A rectangle that contains no part on its boundary cannot tell a half-open
    resolver from a closed one -- and that is exactly what a mutation replacing
    `refs_in_rect` with an inline closed-interval scan exploited: it survived
    the whole battery, because the fixture rectangle had nothing on its edge.

    So this builds the rectangle FROM the board: its far corner is placed
    exactly on a pad centre, so the two rules genuinely disagree, and then it
    asserts the CLI's own printed answer against `refs_in_rect`.
    """
    with tempfile.TemporaryDirectory() as wd:
        b, it = _fixture(wd)
        pcb = parse_kicad_pcb(b)
        # A pad to sit on the far edge, and one strictly inside, so the
        # rectangle is neither empty nor everything.
        pads = sorted(((p.global_x, p.global_y, p.component_ref)
                       for fp in pcb.footprints.values() for p in fp.pads
                       if p.component_ref),
                      key=lambda t: (t[0], t[1]))
        report('the fixture has pads to build an edge case from', len(pads) > 4,
               str(len(pads)))
        if len(pads) <= 4:
            return
        edge = pads[len(pads) // 2]
        rect = (-1.0, -1.0, edge[0], edge[1])

        half_open = refs_in_rect(pcb, rect)
        closed = sorted({ref for x, y, ref in pads
                         if rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]})
        report('the probe rectangle really does separate the two rules',
               half_open != closed,
               'half-open %s vs closed %s' % (half_open, closed))
        if half_open == closed:
            return

        out = os.path.join(wd, 'out.kicad_pcb')
        r = check(_argv(b, out, '--intent', it, '--dry-run',
                        '--reseat-region', repr(rect[0]), repr(rect[1]),
                        repr(rect[2]), repr(rect[3])), accept=True)
        line = next((ln for ln in r.stdout.splitlines()
                     if ln.strip().startswith('region [')), '')
        report('the CLI printed its region line', bool(line), r.stdout[-300:])
        named = sorted(x.strip() for x in
                       line.split('part(s)')[1].split(',')) if 'part(s)' in line \
            else []
        named = [x for x in named if x and x != '...']
        report('the mover lifts the HALF-OPEN set, not the closed one',
               named == half_open,
               'CLI %s / half-open %s / closed %s' % (named, half_open, closed))


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
               'scope_selector: region:0.0,0.0,20.0,14.0' in txt,
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
    """A no-op still writes a board, still summarises, and still repairs.

    The first version of this case ran `--dry-run` and asserted rc 0 plus two
    stdout substrings -- which is a test of the MESSAGE, not of the contract.
    It passed identically while an early `return 0` skipped the output board,
    the JSON summary and the entire `--repair` pass, so
    `--repair --reseat-region <empty>` exited 0 having produced nothing. The
    invariant place_seed states three lines from the skipped write is the one
    that was broken: "'nothing needed doing' must not look like 'the tool
    produced nothing'". So this runs for real, on both arms.
    """
    for label, extra in (('alone', []), ('with --repair', ['--repair'])):
        with tempfile.TemporaryDirectory() as wd:
            b, it = _fixture(wd)
            out = os.path.join(wd, 'out.kicad_pcb')
            r = check(_argv(b, out, '--intent', it,
                            '--reseat-region', '900', '900', '910', '910',
                            *extra), accept=True)
            report('%s: an empty region exits 0' % label,
                   r.returncode == 0, r.stdout[-300:])
            report('  %s: the OUTPUT BOARD is still written' % label,
                   os.path.isfile(out) and os.path.getsize(out) > 0,
                   'exists=%s' % os.path.isfile(out))
            line = next((ln for ln in r.stdout.splitlines()
                         if ln.startswith('JSON_SUMMARY: ')), None)
            report('  %s: the JSON summary is still emitted' % label,
                   line is not None, r.stdout[-300:])
            if line:
                doc = json.loads(line[len('JSON_SUMMARY: '):])
                report('  %s: the scope stayed EXPLICIT and empty' % label,
                       doc.get('scope_source') == 'explicit'
                       and doc.get('scope') == [],
                       '%s / %s' % (doc.get('scope_source'), doc.get('scope')))
                report('  %s: and it is reported ACCEPTED, not refused' % label,
                       doc.get('accepted') is not False,
                       str(doc.get('accepted')))
            report('  %s: says the region is empty, in those words' % label,
                   'contain no part' in r.stdout, r.stdout[-300:])
            report('  %s: does NOT silently become the auto scope' % label,
                   'auto:damage_witnesses' not in r.stdout
                   and 'Reseat (auto' not in r.stdout, r.stdout[-300:])
            if extra:
                report('  %s: the repair pass still RAN' % label,
                       'Repair:' in r.stdout, r.stdout[-400:])


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
    """The union must be a union of the RESOLVED SCOPE, not of a printed line.

    The first version asserted on the disclosure string, which is computed from
    `args.reseat` and never from `reseat['scope']` -- so replacing
    `_refs.extend(_region_refs)` with `_refs = list(_region_refs)` silently
    dropped the named ref and the test stayed green while printing "1 named".
    """
    with tempfile.TemporaryDirectory() as wd:
        b, it = _fixture(wd)
        out = os.path.join(wd, 'out.kicad_pcb')
        pcb = parse_kicad_pcb(b)
        region = (0.0, 0.0, 20.0, 14.0)
        from_region = refs_in_rect(pcb, region)
        report('the region and the named ref are DISJOINT, or this proves '
               'nothing', 'R8' not in from_region, str(from_region))
        r = check(_argv(b, out, '--intent', it, '--dry-run',
                        '--reseat', 'R8',
                        '--reseat-region', '0', '0', '20', '14'), accept=True)
        report('named refs and a region union into one explicit scope',
               'Reseat (explicit)' in r.stdout, r.stdout[-300:])
        line = next((ln for ln in r.stdout.splitlines()
                     if ln.startswith('JSON_SUMMARY: ')), None)
        report('a JSON summary is emitted', line is not None)
        if not line:
            return
        scope = set(json.loads(line[len('JSON_SUMMARY: '):]).get('scope') or [])
        report('  the NAMED ref is in the resolved scope', 'R8' in scope,
               str(sorted(scope)))
        report('  every region ref is in it too',
               set(from_region) <= scope, str(sorted(scope)))
        report('  and the scope is exactly their union',
               scope == set(from_region) | {'R8'},
               '%s vs %s' % (sorted(scope), sorted(set(from_region) | {'R8'})))
        sel = next((ln for ln in r.stdout.splitlines()
                    if 'scope_selector' in ln), '')
        report('  the split is disclosed against the SCOPE, and adds up',
               ('%d of %d in scope came from the region, 1 from --reseat'
                % (len(from_region), len(scope))) in sel, sel)


TESTS = [
    ('one rectangle, one answer', t_a_region_names_the_same_parts_the_census_does),
    ('the mover resolves it the same way, ON the boundary',
     t_the_mover_resolves_the_rectangle_THE_SAME_WAY),
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
