#!/usr/bin/env python3
"""#923: a claim about a tool's OUTPUT, resolved against the real output.

`tests/test_431_skill_commands.py` holds every `--flag` the skills cite to the
tool's real argparse. It has never been able to see the other half of what a
skill tells a reader to do: *read the `X` field*. That is a whole class, and it
rots the same way -- `hot[].ratio` shipped in a mandatory criterion while the
emitted key was `windows[].ratio` (`hot` is a local inside `check_pockets`);
the `aim:` line survived three edits after #709 deleted the feature; the
`"handler"` example names a tool the producer does not emit.

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
     that produces the document and whose first column is the key. Sections
     headed by a tool this file does not run are counted and skipped.
  2. `references/boundary-criteria.md`'s `instrument ... -> path` arrow lines.
  3. Prose: a backticked path within `NEAR` characters of an instrument's name.
     This is the channel that would have caught `hot[].ratio`, which sat one
     line under `check_pockets.py`.

WHAT IT STILL CANNOT SEE, said here rather than implied away: a key claim in
free prose that names no instrument anywhere near it, a claim about a tool this
file does not run (`route.py`'s `JSON_SUMMARY`, `place_route_loop`'s sidecars),
and a key that exists but means something else. The first two are enrolment
gaps and the third is not a gate's kind of question.

    python3 -X utf8 tests/test_923_output_key_claims.py
"""
import json
import os
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
#: this file to read it as a claim about that instrument's output.
NEAR = 400

#: Keys this fixture cannot produce, each with its reason AND the file that
#: emits it. Not a waiver list: the entry is CHECKED -- the key must appear as
#: a literal at the emit site named here -- so a renamed or misspelt key still
#: fails, and an entry nothing cites any more is reported stale. That is the
#: shape `test_803_cited_paths_are_tracked.UNTRACKED_OK` uses, held in both
#: directions, because a list that only ever grows stops describing what it
#: covers.
#:
#: These three are written `if key in r.health`, i.e. only when that health
#: signal ran, and the signals that produce them need a declared block or bus
#: corridor in the intent. The fixture's emitted intent declares neither.
UNRESOLVED_OK = {
    'health_block_displacement_max_mm': (
        'health signal did not run: the fixture declares no blocks',
        'py_placer/placement/floorplan.py'),
    'health_blocks_displaced': (
        'health signal did not run: the fixture declares no blocks',
        'py_placer/placement/floorplan.py'),
    'health_bus_foreign_crossings': (
        'health signal did not run: the fixture declares no bus corridors',
        'py_placer/placement/floorplan.py'),
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


def _run(argv, tmp):
    env = dict(os.environ, KRT_NO_BANNER='1', KICAD_NO_GRADE_RECONCILE='1')
    env.update(run_utils.tool_env())
    return subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, env=env, timeout=600)


def _summary(text):
    """The `JSON_SUMMARY:` line, which is a DIFFERENT document from --json."""
    for line in text.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            return json.loads(line.split('JSON_SUMMARY:', 1)[1])
    return None


def build_artifacts(tmp):
    """{instrument: [(artifact label, document)]}, from real runs.

    A non-zero exit is not a failure here: `check_floorplan` and `board_score`
    exit 4 when they find something, which is what a graded board looks like.
    The document is the evidence, so it is checked instead.
    """
    art = {}

    r = _run([os.path.join('py_tools', 'board_context.py'), FIXTURE, '--json'],
             tmp)
    art['board_context.py'] = [('--json', json.loads(r.stdout))]

    pk = os.path.join(tmp, 'pockets.json')
    r = _run([os.path.join('py_tools', 'check_pockets.py'), FIXTURE,
              '--bin', '5', '--json', pk], tmp)
    art['check_pockets.py'] = [('--json', json.load(open(pk, encoding='utf-8'))),
                               ('JSON_SUMMARY', _summary(r.stdout))]

    cf = os.path.join('py_tools', 'check_floorplan.py')
    intent = os.path.join(tmp, 'intent.json')
    graded = os.path.join(tmp, 'graded.json')
    _run([cf, FIXTURE, '--brief', BRIEF, '--emit-intent', intent], tmp)
    # --health because section E's own heading carries it, and the `health_*`
    # keys exist ONLY when it is passed: a key absent because the run was not
    # asked for it is not a misspelt key.
    r = _run([cf, FIXTURE, '--brief', BRIEF, '--intent', intent,
              '--health', '--json', graded], tmp)
    art['check_floorplan.py'] = [
        ('--json', json.load(open(graded, encoding='utf-8'))),
        ('JSON_SUMMARY', _summary(r.stdout))]

    rj = os.path.join(tmp, 'render.json')
    _run([os.path.join('py_tools', 'render_placement.py'), FIXTURE,
          '--json-out', rj, '-o', os.path.join(tmp, 'render.png')], tmp)
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
    common = [FIXTURE, '--intent', intent, '--placement-terms', '-q']
    _run([score] + common + ['--json', parent], tmp)
    _run([score] + common + ['--json', bs, '--parent-score', parent], tmp)
    art['board_score.py'] = [('--json', json.load(open(bs, encoding='utf-8')))]
    return art


def skill_files():
    out = []
    for base, _dirs, names in os.walk(SKILLS):
        for n in sorted(names):
            if n.endswith('.md'):
                out.append(os.path.join(base, n))
    return sorted(out)


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


def cites_from_evidence_map(known):
    """(instrument, path, where) for every row under a section it can attribute.

    The section heading carries the command that produced the document, and the
    first column is the key -- that is the whole design of the page, and it is
    what makes attribution free here.
    """
    rows, skipped = [], []
    who = None
    for lineno, line in enumerate(_text(EVIDENCE_MAP).splitlines(), 1):
        if line.startswith('#'):
            hit = [k for k in known if k in line]
            who = hit[0] if len(hit) == 1 else None
            if line.startswith('##') and not hit:
                skipped.append(line.strip()[:70])
            continue
        if not who or not line.startswith('|'):
            continue
        cell = line.split('|')[1]
        for raw, _off in _backticked(cell):
            segs = run_utils.parse_json_path(raw)
            if segs:
                rows.append((who, raw, segs,
                             f'evidence-map.md:{lineno}'))
    return rows, skipped


def cites_from_boundary(known):
    """`instrument <board> --json   ->   a.b[].c` lines, both sides matter."""
    rows = []
    if not os.path.isfile(BOUNDARY):
        return rows
    who = None
    for lineno, line in enumerate(_text(BOUNDARY).splitlines(), 1):
        if '->' not in line:
            continue
        left, right = line.split('->', 1)
        hit = [k for k in known if k in left]
        if hit:
            who = hit[0]
        elif left.strip():
            continue                    # a prose arrow, not an instrument line
        if not who:
            continue
        for part in right.split(','):
            segs = run_utils.parse_json_path(part.strip().strip('`'))
            if segs:
                rows.append((who, part.strip().strip('`'), segs,
                             f'boundary-criteria.md:{lineno}'))
    return rows


def cites_from_prose(known):
    """A backticked path within NEAR characters of an instrument's name."""
    rows = []
    for path in skill_files():
        rel = os.path.relpath(path, ROOT).replace('\\', '/')
        if path in (EVIDENCE_MAP, BOUNDARY):
            continue
        text = _text(path)
        where = {}
        for name in known:
            start = 0
            while True:
                i = text.find(name, start)
                if i < 0:
                    break
                where.setdefault(name, []).append(i)
                start = i + 1
        if not where:
            continue
        for raw, off in _backticked(text):
            segs = run_utils.parse_json_path(raw)
            if not segs or len(segs) < 2:
                continue                # a bare word is not a claim about a doc
            near = [(min(abs(off - i) for i in idx), name)
                    for name, idx in where.items()]
            near.sort()
            if near and near[0][0] <= NEAR:
                lineno = text.count('\n', 0, off) + 1
                rows.append((near[0][1], raw, segs, f'{rel}:{lineno}'))
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
    em_rows, skipped = cites_from_evidence_map(known)
    bd_rows = cites_from_boundary(known)
    pr_rows = cites_from_prose(known)
    print(f'  enrolled: {len(em_rows)} evidence-map, {len(bd_rows)} '
          f'boundary-criteria, {len(pr_rows)} prose; '
          f'{len(skipped)} section(s) skipped (tool not run here)')

    # A gate that reports zero claims passes for the wrong reason, and the
    # extractor breaking is far likelier than the skills losing every claim.
    check('the evidence-map channel found rows', len(em_rows) >= 20,
          f'{len(em_rows)}')
    check('the prose channel found rows', len(pr_rows) >= 5, f'{len(pr_rows)}')
    check('the boundary-criteria channel found rows', len(bd_rows) >= 4,
          f'{len(bd_rows)}')

    def resolves_in(who, segs):
        return any(run_utils.resolve_json_path(doc, segs)
                   for _label, doc in art[who] if isinstance(doc, dict))

    bad, used_waiver, elsewhere = [], set(), []
    for who, raw, segs, where in em_rows + bd_rows + pr_rows:
        if resolves_in(who, segs):
            continue
        # ENROLMENT names an instrument; the ASSERTION is over all five. A
        # prose citation is attributed by proximity, and proximity is a guess:
        # `checklist.b_body_seam` is a real render_placement key that sits
        # nearer check_pockets' name in one paragraph. Failing on the guess
        # would report the document's own correct claim as a defect, so the
        # mismatch is PRINTED and the refusal is kept for a key that is in
        # nobody's output -- which is the class #923 names.
        others = sorted(o for o in art if resolves_in(o, segs))
        if others:
            elsewhere.append((who, raw, others[0], where))
            continue
        if raw in UNRESOLVED_OK:
            used_waiver.add(raw)
            reason, emitter = UNRESOLVED_OK[raw]
            src = _text(os.path.join(ROOT, emitter))
            check(f'`{raw}` is emitted by {emitter} ({reason})',
                  f"'{segs[-1]}'" in src or f'"{segs[-1]}"' in src,
                  'the declared emit site does not name it -- renamed?')
            continue
        bad.append((who, raw, where))
    check('every enrolled key claim resolves in the output of some instrument',
          not bad,
          '\n        ' + '\n        '.join(
              f'{w}: `{r}` is in no document {o} or its four siblings write'
              for o, r, w in sorted(set(bad))) if bad else '')
    for who, raw, other, where in sorted(set(elsewhere)):
        print(f'  note: {where}: `{raw}` reads as {other}\'s key, attributed '
              f'to {who} by proximity')

    stale = sorted(set(UNRESOLVED_OK) - used_waiver)
    check('no declared exception has gone stale', not stale,
          f'nothing cites {stale} any more -- delete the entry')


def test_the_extractor_catches_the_claim_that_started_this():
    """The positive control: a lint that cannot fail is decoration.

    `hot[].ratio` is the real defect #923 names, and `route.py` is the shape a
    naive `word.word` scan mistakes for a key. Both are asserted here, against
    the same functions the run above uses, so a scanner that silently stops
    matching fails HERE rather than passing everything.
    """
    doc = {'windows': [{'ratio': 0.5}], 'cold_regions': [{'area_mm2': 1.0}]}
    for good in ('windows[0].ratio', 'windows[].ratio',
                 'cold_regions[0].area_mm2'):
        segs = run_utils.parse_json_path(good)
        check(f'`{good}` parses and resolves',
              bool(segs) and run_utils.resolve_json_path(doc, segs), str(segs))
    segs = run_utils.parse_json_path('hot[].ratio')
    check('`hot[].ratio` parses as a key claim', bool(segs), str(segs))
    check('...and does NOT resolve -- it was never an emitted key',
          not run_utils.resolve_json_path(doc, segs))
    for not_a_key in ('route.py', 'board_store.Ledger', 'F.Cu', 'conn.txt',
                      'wk/render.json', 'metrics.oob_*'):
        check(f'`{not_a_key}` is not read as a key claim',
              run_utils.parse_json_path(not_a_key) is None)

    # And the prose channel must actually enrol a claim sitting beside an
    # instrument's name, or the control above proves only that the resolver
    # works on a dict nobody reads.
    text = ('Read `hot[].ratio` from check_pockets.py, the densest window '
            'first.')
    hits = [raw for raw, off in _backticked(text)
            if run_utils.parse_json_path(raw)]
    check('the prose channel would enrol a claim beside an instrument name',
          'hot[].ratio' in hits, str(hits))


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
