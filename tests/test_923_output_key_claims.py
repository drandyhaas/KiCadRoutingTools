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
the same idea without the hand list: run the instruments the skills quote
ONCE on a tracked fixture, and resolve every key claim the skills make about
them against the document they actually wrote.

WHAT IT ENROLS, and why it is not "every dotted word in the skills". A key
claim needs an INSTRUMENT to be a claim at all -- `metrics.halo` is true of
`render_placement` and false of `check_pockets` -- so a citation is enrolled
only where the text says whose output it is: a backticked path within `NEAR`
characters of an instrument's name, spelled with or without `.py`. This is
the channel that would have caught `hot[].ratio`, which sat one line under
`check_pockets.py`. Proximity is a GUESS, so a claim only has to resolve in
one of the instruments run, and one that resolves in a different one than it
sits beside is printed rather than failed.

Two AUTHORITATIVE channels -- the staged placement skill's
`references/evidence-map.md` (a section heading names the tool, the first
column the key) and `references/boundary-criteria.md`'s `instrument -> path`
lines -- were retired with that skill, and with them the four instruments
only they quoted (board_context, check_pockets, check_floorplan's graded
document, place_seed). What is run today is render_placement and board_score.

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

#: How close a backticked path has to sit to an instrument's name in prose for
#: this file to read it as a claim about that instrument's output. 400 is the
#: largest value at which this corpus stays clean: at 800 three other tools'
#: keys (`impedance.nets_analyzed`, `net_widths.patterns_matching_no_routed_net`,
#: `route_summary.merge_summaries`) come into range of an instrument name and
#: are reported as defects they are not.
NEAR = 400

#: Measured 2 after the staged placement skills were retired (the free-agent
#: verifier's `checklist.a_off_outline` and the routing skill's
#: `components.unrouted.placement_blocked`). Held at the measured value: with
#: a population this small, losing one claim IS the scanner breaking.
PROSE_FLOOR = 2

#: A cited path this gate cannot resolve and that is not a defect, with the
#: reason AND a file that must carry the name. Not a waiver list: the entry is
#: CHECKED, so a rename still fails, and an entry nothing cites any more is
#: reported stale -- the shape `test_803_cited_paths_are_tracked.UNTRACKED_OK`
#: uses, held in both directions.
#: (The staged skills' pages that cited `score.failed_nets`, a converge ledger
#: row's nested score, were retired; this one came with pcb-free-agent. Its
#: sibling, `components.broken.nets[].handler`, went when #1112 made every
#: break route.py's and the skill stopped citing the key.)
UNRESOLVED_OK = {
    'power_widths.<net>.under_mm': (
        "route.py's JSON_SUMMARY key, not a board_score document",
        'py_router/route.py', "summary['power_widths']"),
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

    # Four instruments used to be run here as well -- board_context,
    # check_pockets, check_floorplan's graded document and four place_seed
    # shapes -- because the staged placement skill's evidence map and
    # boundary criteria quoted their keys. Those pages were retired with that
    # skill, and nothing left in the skills quotes a key of theirs (measured:
    # 0 prose claims each), so running them would grade nothing.
    # check_floorplan still EMITS the intent board_score grades against.
    cf = os.path.join('py_tools', 'check_floorplan.py')
    intent = os.path.join(tmp, 'intent.json')
    _run([cf, FIXTURE, '--brief', BRIEF, '--emit-intent', intent], expect=(0, 4))
    run_utils.evidence(intent, 'the emitted intent')
    _declare_health(intent)

    rj = os.path.join(tmp, 'render.json')
    _run([os.path.join('py_tools', 'render_placement.py'), FIXTURE,
          '--json-out', rj, '-o', os.path.join(tmp, 'render.png')],
         expect=(0, 1, 2, 3, 4))
    run_utils.evidence(rj, 'the render document')
    art['render_placement.py'] = [('--json-out',
                                   json.load(open(rj, encoding='utf-8')))]

    score = os.path.join('py_tools', 'board_score.py')
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


def cites_from_prose(known):
    """A backticked path within NEAR characters of an instrument's name."""
    rows = []
    names = _names(known)
    for path in skill_files():
        rel = os.path.relpath(path, ROOT).replace('\\', '/')
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
    pr_rows = cites_from_prose(known)
    print(f'  enrolled: {len(pr_rows)} prose')
    # The two AUTHORITATIVE channels (the evidence map and the boundary
    # criteria pages) left with the staged placement skills they belonged
    # to; prose over every skill page is what remains.
    #
    # A gate that reports zero claims passes for the wrong reason, and the
    # extractor breaking is far likelier than the skills losing every claim.
    check('the prose channel found rows', len(pr_rows) >= PROSE_FLOOR,
          f'{len(pr_rows)}')

    def resolves_in(tool, segs):
        return any(run_utils.resolve_json_path(doc, segs)
                   for _label, doc in art[tool] if isinstance(doc, dict))

    # ...and per INSTRUMENT, because an instrument hangs entirely on prose:
    # dropping `.py` from one tool name in one paragraph took `check_pockets`
    # out of the gate completely and still cleared the channel floor.
    covered = {t: 0 for t in known}
    for who, _raw, _segs, _where in pr_rows:
        for tool in who:
            covered[tool] = covered.get(tool, 0) + 1
    for tool, n in sorted(covered.items()):
        check(f'{tool} has claims to check', n >= 1, f'{n}')

    bad, elsewhere, used_waiver = [], [], set()
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
