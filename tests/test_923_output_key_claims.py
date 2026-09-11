#!/usr/bin/env python3
"""#923: a claim about a tool's OUTPUT, resolved against the real output.

`tests/test_431_skill_commands.py` holds every `--flag` the skills cite to the
tool's real argparse. It has never been able to see the other half of what a
skill tells a reader to do: *read the `X` field*. That is a whole class, and it
rots the same way -- `hot[].ratio` shipped in a mandatory criterion while the
emitted key was `windows[].ratio` (`hot` is a local inside `check_pockets`),
and two lines told a reader to read `broken.poured_nets_meaning` from a
document that writes it at `components.broken.poured_nets_meaning`.

`tests/test_895_boundary_criteria.py` closed that hole for the seven boundary
criteria -- eight paths, one hand-written resolver lambda each. This file is
the same idea without the hand list: run the five instruments the skills quote
ONCE on a tracked fixture, and resolve every key claim the skills make about
them against the document they actually wrote.

WHAT IT ENROLS, and why it is not "every dotted word in the skills". A key
claim needs an INSTRUMENT to be a claim at all -- `metrics.halo` is true of
`render_placement` and false of `check_pockets` -- so a citation is enrolled
only where the text says whose output it is:

  1. `references/evidence-map.md`, whose sections are headed with the command
     that produces the document and whose first column is the key. AUTHORITATIVE:
     the heading names the tool, so the row is resolved against THAT tool.
     Sections headed by a tool this file does not run are counted and skipped.
  2. `references/boundary-criteria.md`'s `instrument ... -> path` lines, where
     the instrument is the nearest one named at or above the arrow. Also
     authoritative.
  3. Prose: a backticked path within `NEAR` characters of an instrument's name,
     spelled with or without `.py`. This is the channel that would have caught
     `hot[].ratio`, which sat one line under `check_pockets.py`. Proximity is a
     GUESS, so a prose claim only has to resolve in one of the five, and a
     claim that resolves in a different one than it sits beside is printed
     rather than failed.

WHAT IT STILL CANNOT SEE, said here rather than implied away:

  * a key claim in free prose more than `NEAR` characters from any instrument
    name, or inside a fenced JSON example (the `"handler": "repair_planes"`
    blob is not backticked and is not enrolled);
  * a claim about a tool this file does not run -- `route.py`'s `JSON_SUMMARY`,
    `place_optimize`'s lock advice, the `place_route_loop` sidecars;
  * a key that exists but MEANS something else;
  * a key whose only emitted spelling carries an upper-case segment
    (`arrangement.sides.F.offset_mm`, `checklist.b_body_sources.C1`): segments
    are matched lower-case, which is a FALSE-POSITIVE filter (`F.Cu`,
    `board_store.Ledger`, `Default.clearance` are all real strings in the
    skills and none of them is an output key), and its cost is that family.
    The `sides[<layer>]` spelling the docs actually use resolves.

The run prints what it dropped, per channel, because a scanner that quietly
stops matching is the failure this whole file exists to make visible.

    python3 -X utf8 tests/test_923_output_key_claims.py
"""
import functools
import json
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import run_utils                                             # noqa: E402

RUN_ALL_TIMEOUT = 900

#: The placed, copper-free, zone-free corpus board every one of these tools can
#: read. Zone-free matters: `board_score` -> `check_connected` reaches for
#: `kicad-cli` only when the parser sees zones, so this fixture keeps a doc
#: gate off KiCad entirely.
FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                       'esp_prog_placed.kicad_pcb')
#: The declared clauses, so `check_floorplan` writes a real `brief_coverage`
#: block rather than an empty one.
BRIEF = os.path.join(ROOT, 'tests', 'fixtures', '902',
                     'esp_prog_proximity.design-brief.json')

SKILLS = os.path.join(ROOT, '.claude', 'skills')
EVIDENCE_MAP = os.path.join(SKILLS, 'plan-pcb-placement-and-routing',
                            'references', 'evidence-map.md')
BOUNDARY = os.path.join(SKILLS, 'plan-pcb-placement-and-routing',
                        'references', 'boundary-criteria.md')

#: How close a backticked path has to sit to an instrument's name in prose for
#: this file to read it as a claim about that instrument's output. 400 is the
#: largest value at which this corpus stays clean: at 800 three other tools'
#: keys (`impedance.nets_analyzed`, `net_widths.patterns_matching_no_routed_net`,
#: `route_summary.merge_summaries`) come into range of an instrument name and
#: are reported as defects they are not.
NEAR = 400

#: A cited path this gate cannot resolve and that is not a defect, with the
#: reason AND a file that must carry the name. Not a waiver list: the entry is
#: CHECKED, so a rename still fails, and an entry nothing cites any more is
#: reported stale -- the shape `test_803_cited_paths_are_tracked.UNTRACKED_OK`
#: uses, held in both directions.
UNRESOLVED_OK = {
    'score.failed_nets': (
        "a converge LEDGER ROW's nested score, not an instrument document",
        'py_placer/converge.py', 'failed_nets'),
}

FAILURES = []


def check(name, cond, detail=''):
    if cond:
        print(f'  PASS: {name}')
    else:
        FAILURES.append(f'{name} -- {detail}')
        print(f'  FAIL: {name} -- {detail}')


def _text(path):
    with open(path, encoding='utf-8') as fh:
        return fh.read()


def _run(argv, expect=(0,)):
    """Run a tool, and REFUSE to build an artifact out of a failed run.

    `check_floorplan` and `board_score` exit 4 when they find something, which
    is what a graded board looks like -- so the expected set is per call rather
    than "zero". Anything else is reported with the child's stderr instead of
    surfacing later as a FileNotFoundError on a temp path.
    """
    env = run_utils.tool_env()
    env.update(KRT_NO_BANNER='1', KICAD_NO_GRADE_RECONCILE='1')
    p = subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, env=env, timeout=300)
    check(f'{os.path.basename(argv[0])} exited {p.returncode}',
          p.returncode in expect,
          f'expected {expect}; stderr tail:\n        '
          + (p.stderr or '')[-400:].replace('\n', '\n        '))
    return p


def _summary(text):
    """The `JSON_SUMMARY:` line, which is a DIFFERENT document from --json."""
    for line in text.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            return json.loads(line.split('JSON_SUMMARY:', 1)[1])
    return None


def _declare_health(intent_path):
    """Give the emitted intent the declarations the health signals need.

    Four `health_*` keys exist only when something was declared for them:
    `block_displacement_max_mm` and `blocks_displaced` need blocks (and the
    second needs a LIMIT -- `routability.py` writes it only
    `if limit is not None`, which is why it read as absent on every board for
    months), and `bus_foreign_crossings` needs a corridor. Declaring them here
    is the difference between resolving those rows and waiving them.
    """
    with open(intent_path, encoding='utf-8') as fh:
        doc = json.load(fh)
    if not doc.get('blocks'):
        doc['blocks'] = [{'name': 'usb', 'refs': ['U1', 'USB1']},
                         {'name': 'power', 'refs': ['U2', 'C1', 'C3']}]
    health = dict(doc.get('health') or {})
    health.setdefault('block_displacement_mm', 15.0)
    health.setdefault('bus_corridors',
                      [{'name': 'usb', 'nets': ['/D_*'], 'width_mm': 4.0}])
    doc['health'] = health
    with open(intent_path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)


def build_artifacts(tmp):
    """{instrument: [(artifact label, document)]}, from real runs."""
    art = {}

    r = _run([os.path.join('py_tools', 'board_context.py'), FIXTURE, '--json'])
    art['board_context.py'] = [('--json', json.loads(r.stdout))]

    pk = os.path.join(tmp, 'pockets.json')
    r = _run([os.path.join('py_tools', 'check_pockets.py'), FIXTURE,
              '--bin', '5', '--json', pk])
    run_utils.evidence(pk, 'the check_pockets document')
    art['check_pockets.py'] = [('--json', json.load(open(pk, encoding='utf-8'))),
                               ('JSON_SUMMARY', _summary(r.stdout))]

    cf = os.path.join('py_tools', 'check_floorplan.py')
    intent = os.path.join(tmp, 'intent.json')
    graded = os.path.join(tmp, 'graded.json')
    _run([cf, FIXTURE, '--brief', BRIEF, '--emit-intent', intent], expect=(0, 4))
    run_utils.evidence(intent, 'the emitted intent')
    _declare_health(intent)
    # --health because section E's own heading carries it, and the `health_*`
    # keys exist ONLY when it is passed.
    r = _run([cf, FIXTURE, '--brief', BRIEF, '--intent', intent,
              '--health', '--json', graded], expect=(0, 4))
    run_utils.evidence(graded, 'the graded intent document')
    art['check_floorplan.py'] = [
        ('--json', json.load(open(graded, encoding='utf-8'))),
        ('JSON_SUMMARY', _summary(r.stdout))]

    rj = os.path.join(tmp, 'render.json')
    _run([os.path.join('py_tools', 'render_placement.py'), FIXTURE,
          '--json-out', rj, '-o', os.path.join(tmp, 'render.png')],
         expect=(0, 1, 2, 3, 4))
    run_utils.evidence(rj, 'the render document')
    art['render_placement.py'] = [('--json-out',
                                   json.load(open(rj, encoding='utf-8')))]

    score = os.path.join('.claude', 'skills', 'plan-pcb-placement-and-routing',
                         'scripts', 'board_score.py')
    parent = os.path.join(tmp, 'parent.json')
    bs = os.path.join(tmp, 'score.json')
    # --intent and --placement-terms so the floorplan and placement components
    # GRADE rather than reporting UNGRADED, and --parent-score because the
    # `placement.vs_parent` block exists only in a comparison. A component that
    # did not run writes none of its keys, and a key absent for that reason is
    # not a misspelt key -- so the run is set up to grade what the skills quote.
    # ...and a width spec, because `components.net_widths` reports UNGRADED
    # without one and its keys are then absent for a reason that is not a
    # misspelling. On a copper-free board every pattern lands in
    # `patterns_matching_no_routed_net`, which is the key the skills quote.
    widths = os.path.join(tmp, 'widths.json')
    with open(widths, 'w', encoding='utf-8') as fh:
        json.dump({'/VBUS': 0.8}, fh)
    common = [FIXTURE, '--intent', intent, '--placement-terms',
              '--net-min-widths', widths, '-q']
    _run([score] + common + ['--json', parent], expect=(0, 4))
    _run([score] + common + ['--json', bs, '--parent-score', parent],
         expect=(0, 4))
    run_utils.evidence(bs, 'the board score')
    art['board_score.py'] = [('--json', json.load(open(bs, encoding='utf-8')))]
    return art


def skill_files():
    out = []
    for base, _dirs, names in os.walk(SKILLS):
        for n in sorted(names):
            if n.endswith('.md'):
                out.append(os.path.join(base, n))
    return sorted(out)


@functools.lru_cache(maxsize=None)
def _repo_modules():
    """{module basename: path} over every tracked .py."""
    out = {}
    for path in run_utils.corpus_boards('*.py'):
        out.setdefault(os.path.basename(path)[:-3], path)
    return out


def _module_symbol(raw):
    """Is `a.b` a MODULE and something defined in it, rather than a key?

    `net_queries.filter_routable_nets` and `route_summary.merge_summaries` are
    both spelled like key paths and sit next to an instrument's name, and both
    are functions. Answered mechanically -- the module is tracked and the name
    is defined in it -- rather than by a list of exceptions, because the list
    is what stops describing the thing it covers.
    """
    head, _dot, rest = raw.partition('.')
    path = _repo_modules().get(head)
    if not path or not rest or not os.path.isfile(path):
        return False
    name = rest.split('.')[0].split('[')[0]
    if not name:
        return False
    src = _text(path)
    return bool(re.search(r'^\s*(?:def|class)\s+%s\b' % re.escape(name),
                          src, re.M)
                or re.search(r'^%s\s*[:=]' % re.escape(name), src, re.M))


def _backticked(text):
    """(raw, offset) for every `...` span."""
    out, i = [], 0
    while True:
        a = text.find('`', i)
        if a < 0:
            return out
        b = text.find('`', a + 1)
        if b < 0:
            return out
        out.append((text[a + 1:b], a))
        i = b + 1


def _names(known):
    """Every spelling an instrument is anchored by: `board_score.py` and the
    bare `board_score`, which the skills use just as often."""
    out = {}
    for tool in known:
        out[tool] = tool
        out[tool[:-3]] = tool
    return out


def cites_from_evidence_map(known):
    """(tools, path, segments, where) per row, plus what was skipped/dropped.

    The section heading carries the command that produced the document and the
    first column is the key -- that is the design of the page, and it is what
    makes attribution AUTHORITATIVE here rather than a guess.
    """
    rows, skipped, dropped = [], [], []
    who, prefix = (), None
    for lineno, line in enumerate(_text(EVIDENCE_MAP).splitlines(), 1):
        if line.startswith('#'):
            hit = tuple(sorted({t for n, t in _names(known).items()
                                if n in line}))
            who, prefix = hit, None
            if line.startswith('##') and not hit:
                skipped.append(line.strip()[:70])
            continue
        if not who or not line.startswith('|'):
            continue
        cell = line.split('|')[1]
        for raw, _off in _backticked(cell):
            token = raw.strip()
            if token.startswith('.') and prefix:
                # A CONTINUATION of the row above: `outline.cutouts` /
                # `.edge_contours` names two keys, and reading only the first
                # drops half the row.
                token = prefix + token
            segs = run_utils.parse_json_path(token)
            if segs:
                rows.append((who, token, segs, f'evidence-map.md:{lineno}'))
                if '.' in token:
                    prefix = token.rsplit('.', 1)[0]
            else:
                dropped.append((f'evidence-map.md:{lineno}', raw))
    return rows, skipped, dropped


def cites_from_boundary(known):
    """`instrument <board> --json   ->   a.b[].c`, the instrument being the
    nearest one named at or above the arrow -- the command wraps, so the arrow
    is often on a line whose left side is empty."""
    rows, dropped = [], []
    if not os.path.isfile(BOUNDARY):
        return rows, dropped
    names = _names(known)
    who = ()
    for lineno, line in enumerate(_text(BOUNDARY).splitlines(), 1):
        hit = tuple(sorted({t for n, t in names.items() if n in line}))
        if hit:
            who = hit
        if '->' not in line or not who:
            continue
        for part in line.split('->', 1)[1].split(','):
            token = part.strip().strip('`')
            segs = run_utils.parse_json_path(token)
            if segs:
                rows.append((who, token, segs,
                             f'boundary-criteria.md:{lineno}'))
            elif token:
                dropped.append((f'boundary-criteria.md:{lineno}', token))
    return rows, dropped


def cites_from_prose(known):
    """A backticked path within NEAR characters of an instrument's name."""
    rows = []
    names = _names(known)
    for path in skill_files():
        rel = os.path.relpath(path, ROOT).replace('\\', '/')
        if path in (EVIDENCE_MAP, BOUNDARY):
            continue
        text = _text(path)
        where = {}
        for name, tool in names.items():
            start = 0
            while True:
                i = text.find(name, start)
                if i < 0:
                    break
                where.setdefault(tool, []).append(i)
                start = i + 1
        if not where:
            continue
        for raw, off in _backticked(text):
            segs = run_utils.parse_json_path(raw)
            if not segs or len(segs) < 2:
                continue                # a bare word is not a claim about a doc
            if _module_symbol(raw):
                continue                # `module.function`, not `doc.key`
            near = sorted((min(abs(off - i) for i in idx), tool)
                          for tool, idx in where.items())
            if near and near[0][0] <= NEAR:
                lineno = text.count('\n', 0, off) + 1
                rows.append(((near[0][1],), raw, segs, f'{rel}:{lineno}'))
    return rows


def test_the_skills_key_what_the_instruments_emit():
    """Every attributable key claim, resolved against the real document."""
    with tempfile.TemporaryDirectory() as tmp:
        art = build_artifacts(tmp)

    for name, docs in sorted(art.items()):
        for label, doc in docs:
            check(f'{name} wrote its {label} document',
                  isinstance(doc, dict) and bool(doc),
                  f'{type(doc).__name__}')

    known = tuple(sorted(art))
    em_rows, skipped, em_dropped = cites_from_evidence_map(known)
    bd_rows, bd_dropped = cites_from_boundary(known)
    pr_rows = cites_from_prose(known)
    print(f'  enrolled: {len(em_rows)} evidence-map, {len(bd_rows)} '
          f'boundary-criteria, {len(pr_rows)} prose; '
          f'{len(skipped)} section(s) skipped (tool not run here); '
          f'{len(em_dropped) + len(bd_dropped)} token(s) not shaped like a key')
    for where, raw in em_dropped + bd_dropped:
        print(f'  drop: {where}: `{raw}`')

    # A gate that reports zero claims passes for the wrong reason, and the
    # extractor breaking is far likelier than the skills losing every claim.
    check('the evidence-map channel found rows', len(em_rows) >= 60,
          f'{len(em_rows)}')
    # ...and a floor alone is not the whole guard, because attribution is by
    # TOOL NAME ON THE HEADING LINE: rename one heading so it drops the tool and
    # every row beneath it silently stops being checked. Measured when #936
    # split section E in two -- the split is safe only because BOTH new headings
    # kept the literal `check_floorplan.py` -- dropping it takes the channel
    # 73 -> 58 (E2) or 73 -> 59 (E1), i.e. 15 or 14 rows. At TODAY's population
    # the raised floor above catches that too; this arm is what still holds when
    # the population grows past 60 by other means, which is exactly when a lost
    # section stops showing up in the total.
    #
    # `skipped` is the count of `##` headings naming no tool this file runs. It
    # is legitimate for a page to describe a tool this gate does not run, so the
    # rule is NO GROWTH against the measured population, not zero.
    check('no evidence-map section stopped being attributed',
          len(skipped) <= 8, f'{len(skipped)} skipped: {skipped}')
    check('the prose channel found rows', len(pr_rows) >= 5, f'{len(pr_rows)}')
    check('the boundary-criteria channel found rows', len(bd_rows) >= 4,
          f'{len(bd_rows)}')

    def resolves_in(tool, segs):
        return any(run_utils.resolve_json_path(doc, segs)
                   for _label, doc in art[tool] if isinstance(doc, dict))

    # ...and per INSTRUMENT, because two of the five have no evidence-map
    # section at all and hang entirely on prose: dropping `.py` from one tool
    # name in one paragraph took `check_pockets` out of the gate completely and
    # still cleared the channel floor.
    covered = {t: 0 for t in known}
    for who, _raw, _segs, _where in em_rows + bd_rows + pr_rows:
        for tool in who:
            covered[tool] = covered.get(tool, 0) + 1
    for tool, n in sorted(covered.items()):
        check(f'{tool} has claims to check', n >= 1, f'{n}')

    bad, elsewhere, used_waiver = [], [], set()
    for who, raw, segs, where in em_rows + bd_rows:
        # AUTHORITATIVE: the document names the producing command, so a key
        # that belongs to a different instrument is a defect here, not a note.
        if not any(resolves_in(t, segs) for t in who):
            bad.append(('/'.join(who), raw, where))
    for who, raw, segs, where in pr_rows:
        if any(resolves_in(t, segs) for t in who):
            continue
        # Proximity is a guess -- `checklist.b_body_seam` is a real
        # render_placement key that sits nearer check_pockets in one paragraph
        # -- so a prose claim only has to be SOMEBODY's key.
        others = sorted(o for o in art if resolves_in(o, segs))
        if others:
            elsewhere.append((who[0], raw, others[0], where))
        elif raw in UNRESOLVED_OK:
            used_waiver.add(raw)
            reason, where_from, literal = UNRESOLVED_OK[raw]
            check(f'`{raw}` is {reason}',
                  literal in _text(os.path.join(ROOT, where_from)),
                  f'{where_from} does not carry {literal!r} -- renamed?')
        else:
            bad.append(('/'.join(who), raw, where))
    stale = sorted(set(UNRESOLVED_OK) - used_waiver)
    check('no declared exception has gone stale', not stale,
          f'nothing cites {stale} any more -- delete the entry')
    check('every enrolled key claim resolves in the output it names',
          not bad,
          '\n        ' + '\n        '.join(
              f'{w}: `{r}` is in no document {o} writes'
              for o, r, w in sorted(set(bad))) if bad else '')
    for who, raw, other, where in sorted(set(elsewhere)):
        print(f'  note: {where}: `{raw}` reads as {other}\'s key, attributed '
              f'to {who} by proximity')


def test_the_extractor_catches_the_claim_that_started_this():
    """The positive control: a lint that cannot fail is decoration.

    `hot[].ratio` is the real defect #923 names, and `route.py` is the shape a
    naive `word.word` scan mistakes for a key. Both are asserted here, against
    the same functions the run above uses, so a scanner that silently stops
    matching fails HERE rather than passing everything.
    """
    doc = {'windows': [{'ratio': 0.5}], 'cold_regions': [{'area_mm2': 1.0}],
           'checklist': {'b_body_seam': {'mm': -0.13}},
           'arrangement': {'sides': {'F.Cu': {'offset_mm': 1.0}}},
           'state_unplaced': False, 'state_spread_ratio': 1.0}
    for good in ('windows[0].ratio', 'windows[].ratio',
                 'cold_regions[0].area_mm2', 'checklist.b_body_seam.mm',
                 'arrangement.sides[<layer>].offset_mm', 'state_*'):
        segs = run_utils.parse_json_path(good)
        check(f'`{good}` parses and resolves',
              bool(segs) and run_utils.resolve_json_path(doc, segs), str(segs))
    segs = run_utils.parse_json_path('hot[].ratio')
    check('`hot[].ratio` parses as a key claim', bool(segs), str(segs))
    check('...and does NOT resolve -- it was never an emitted key',
          not run_utils.resolve_json_path(doc, segs))
    # A `[]` that also descended a dict made this resolve against
    # `checklist.b_body_seam.mm`, so a path with a SEGMENT MISSING read as
    # correct. `[]` is a list step; `[<name>]` is the dict one.
    check('`checklist[].mm` does not resolve -- a list step is not a dict step',
          not run_utils.resolve_json_path(
              doc, run_utils.parse_json_path('checklist[].mm')))
    for not_a_key in ('route.py', 'board_store.Ledger', 'F.Cu', 'conn.txt',
                      'wk/render.json'):
        check(f'`{not_a_key}` is not read as a key claim',
              run_utils.parse_json_path(not_a_key) is None)

    # And the prose channel must actually enrol a claim sitting beside an
    # instrument's name -- in BOTH spellings, because the skills use the bare
    # tool name as often as the `.py` one and anchoring on `.py` alone took
    # three real citations out of the scan.
    for anchor in ('check_pockets.py', 'check_pockets'):
        text = f'Read `hot[].ratio` from {anchor}, the densest window first.'
        hits = [raw for raw, off in _backticked(text)
                if run_utils.parse_json_path(raw)]
        check(f'a claim beside `{anchor}` is enrolled', 'hot[].ratio' in hits,
              str(hits))
    check('the bare name is an anchor', 'check_pockets' in _names(
        ('check_pockets.py',)))


TESTS = [test_the_extractor_catches_the_claim_that_started_this,
         test_the_skills_key_what_the_instruments_emit]


def main():
    for t in TESTS:
        print(f'--- {t.__name__}')
        try:
            t()
        except Exception as exc:                             # noqa: BLE001
            check(f'{t.__name__} ran to completion', False,
                  f'{type(exc).__name__}: {exc}')
    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #923 output-key claims, "
          f"{len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f'  - {f}')
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
