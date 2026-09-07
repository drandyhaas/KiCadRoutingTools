"""#891: the component-context sheet says the things it exists to say.

The headline row is the one run 25 needed and did not have: whether a
differential pair's polarity order CROSSES between the two parts carrying it.
Two boards are asserted deliberately, because they answer differently and the
difference is the point --

    kicad_files/esp_prog.kicad_pcb        UNDETERMINED
    tests/fixtures/run25/..._lap5         CROSSED

On the tracked board U1 sits at rotation 0, so its /D_P and /D_N pads share a y
while the channel to USB1 runs along x: both project to one point on the
channel axis and the order is not a fact about the board. On run 25's lap 5, U1
is at 270 and the two orders genuinely cross. An earlier draft of this tool
reported AGREES for the first case, which is a wrong answer dressed as a
measurement -- so the UNDETERMINED row is asserted as hard as the CROSSED one.
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [os.path.join(ROOT, 'py_router'),
                os.path.join(ROOT, 'py_placer'),
                os.path.join(ROOT, 'py_tools')]
sys.path.insert(0, HERE)

RUN_ALL_TIMEOUT = 1200

FAILURES = []
TOOL = os.path.join(ROOT, 'py_tools', 'board_context.py')
TRACKED = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
LAP5 = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                    'esp_prog_lap5.kicad_pcb')


def check(name, cond, detail=''):
    if cond:
        print(f'  PASS: {name}')
    else:
        FAILURES.append(f'{name}{(" -- " + detail) if detail else ""}')
        print(f'  FAIL: {name} -- {detail}')


def _run(board, *extra):
    import run_utils
    argv = [sys.executable, '-X', 'utf8', TOOL, board, *extra]
    return subprocess.run(argv, capture_output=True, text=True,
                          env=run_utils.tool_env(), timeout=900)


def _doc(board, *extra):
    r = _run(board, '--json', *extra)
    if r.returncode != 0:
        raise AssertionError(f'board_context failed on {board}: '
                             f'{r.stderr[-800:]}')
    return json.loads(r.stdout), r


def test_json_is_parseable_from_char_zero():
    """The trap this tool was born into.

    `cli_banner` is the famous one, but it is not the only writer to stdout:
    `parser.warn_missing_courtyards` prints from inside the quench, and
    esp_prog has 18 courtyard-less parts. An arm that only checked "does the
    document contain the right keys" would have passed on a build whose stdout
    could not be parsed at all.
    """
    r = _run(TRACKED, '--json')
    check('--json exits 0', r.returncode == 0, r.stderr[-300:])
    try:
        doc = json.loads(r.stdout)
        check('--json stdout parses from char 0', True)
    except Exception as exc:                                 # noqa: BLE001
        check('--json stdout parses from char 0', False,
              f'{type(exc).__name__}: {exc}; first 80 chars '
              f'{r.stdout[:80]!r}')
        return
    check('the courtyard warning went to stderr, not into the document',
          'WARNING' in r.stderr and 'WARNING' not in r.stdout)
    check('the document names its schema and board',
          doc.get('schema') and doc.get('board') == 'esp_prog.kicad_pcb',
          str(doc.get('board')))


def test_the_usb_pair_row():
    """THE row, on both boards, with the reason each reads as it does."""
    tracked, _ = _doc(TRACKED)
    lap5, _ = _doc(LAP5)

    def pair_row(doc):
        rows = (doc.get('pin_order') or {}).get('rows') or []
        for r in rows:
            if {r['a'], r['b']} == {'U1', 'USB1'} \
                    and str(r.get('scope', '')).startswith('pair'):
                return r
        return None

    a, b = pair_row(tracked), pair_row(lap5)
    check('the tracked board has a pair-scoped U1<->USB1 row', a is not None,
          'no row; the pair scope is the whole point of this sheet')
    check('run 25 lap 5 has a pair-scoped U1<->USB1 row', b is not None)
    if a:
        check('tracked: UNDETERMINED (U1 at rot 0 puts /D_P and /D_N at one '
              'y, so both project to one point on the channel axis)',
              a['verdict'].startswith('UNDETERMINED') and a['ties'] > 0,
              f"verdict {a['verdict']!r} ties {a.get('ties')}")
    if b:
        check('lap 5: CROSSED (U1 at rot 270; the parity that decided run '
              "25's only open clause)",
              b['verdict'] == 'CROSSED' and b['inversions'] == 1
              and b['ties'] == 0, f'row {b}')


def test_the_pair_scope_attributes_what_the_interface_blends():
    """The argument for `only_nets`, stated as it actually measures.

    An earlier draft of this test claimed the interface row says AGREES on
    lap 5 while the pair row says CROSSED. That is FALSE and the test caught
    it: measured, both scopes reach the same VERDICT on both boards --

        tracked   pair nets=2 inv=0 ties=1   interface nets=3 inv=0 ties=2
        lap 5     pair nets=2 inv=1 ties=0   interface nets=3 inv=3 ties=0

    What the pair scope actually buys is ATTRIBUTION. `inversions` is a lower
    bound on forced crossings, and the interface number blends three nets into
    one figure: 3 on lap 5. The pair scope says exactly 1 of those is the
    declared /D_P//D_N pair, which is the part a mirror or a via hop would fix
    and rotation would not. A reader given only "3" cannot tell whether the
    pair is implicated at all.

    (The verdicts agree here only because `ties` was added at the same time.
    Before it, the tracked board's interface row read AGREES on a question
    with no answer -- which is the failure mode the original claim described,
    correctly, about the old code.)
    """
    doc, _ = _doc(LAP5)
    rows = (doc.get('pin_order') or {}).get('rows') or []
    iface = [r for r in rows if {r['a'], r['b']} == {'U1', 'USB1'}
             and r.get('scope') == 'interface']
    pair = [r for r in rows if {r['a'], r['b']} == {'U1', 'USB1'}
            and str(r.get('scope', '')).startswith('pair')]
    check('lap 5 carries both scopes for U1<->USB1',
          len(iface) == 1 and len(pair) == 1,
          f'{len(iface)} interface, {len(pair)} pair')
    if iface and pair:
        check('the interface row sees more nets than the pair row',
              iface[0]['nets'] > pair[0]['nets'],
              f"{iface[0]['nets']} vs {pair[0]['nets']}")
        check('the interface BLENDS more crossings than the pair is '
              'responsible for (3 vs 1 on lap 5) -- attribution is what the '
              'pair scope buys',
              iface[0]['inversions'] > pair[0]['inversions'] > 0,
              f"interface {iface[0]['inversions']}, "
              f"pair {pair[0]['inversions']}")


def test_the_mating_face_column():
    """#891 names USB1 explicitly, and the measurement disagrees with half of
    the claim -- which is worth an arm rather than a quiet omission.

    The issue's acceptance reads "USB1's mating face reads W with overhang >
    0". The face is W. The overhang is ZERO on both esp_prog boards: USB1's
    body starts at x 114.000 and the outline's west edge IS 114.000, so the
    receptacle is flush, not overhanging. Asserted as flush, because a test
    that quietly relaxed to `>= 0` would let a real overhang regression
    through, and one that asserted `> 0` would be asserting something the
    board does not say.

    The number also depends on WHICH rect is measured, which is why this
    column reads the body model: `connector_edge_facts` measures
    `model.rect`, the quench's pad-box ladder, and USB1's pad box sits 0.49mm
    INSIDE the west edge while its drawn body reaches it.
    """
    for board, name in ((TRACKED, 'tracked'), (LAP5, 'lap 5')):
        doc, _ = _doc(board)
        row = next((p['mating'] for p in doc['parts']
                    if p['ref'] == 'USB1' and p.get('mating')), None)
        check(f'{name}: USB1 carries a mating row', row is not None)
        if row:
            check(f'{name}: USB1 mates through the W edge',
                  row['edge'] == 'W', str(row))
            check(f'{name}: flush, not overhanging (body x0 == outline x0)',
                  row['overhang_mm'] == 0.0 and row['dist_mm'] == 0.0,
                  str(row))
            check(f'{name}: and the row says which geometry it measured',
                  row.get('basis') in ('courtyard', 'fab', 'silk',
                                       'pad_bbox'), str(row))


def test_every_derived_fact_names_its_source():
    """#711's rule, applied here: no claim without a channel it came from."""
    doc, _ = _doc(TRACKED)
    missing = [p['ref'] for p in doc['parts']
               if not (p.get('role') or {}).get('source')]
    check('every part carries a role source', missing == [],
          f'{len(missing)}: {missing[:5]}')
    check('every part carries a body source',
          all(p.get('body_source') for p in doc['parts']))
    check('the sheet declares where each column comes from',
          len(doc.get('sources') or {}) >= 6, str(sorted(doc.get('sources',
                                                                {}))))
    # `unknown` must be REACHABLE, or the honesty rule is decorative: esp_prog
    # carries three OLIMEX logo blocks with no reference convention at all.
    unknown = [p['ref'] for p in doc['parts']
               if p['role']['role'] == 'unknown']
    check('"unknown" is a value this sheet actually produces', unknown != [],
          'no part reported unknown; the rule is untested if nothing hits it')


def test_the_body_column_is_the_896_model():
    """Not a fourth body ladder: the sources the sheet reports must be the
    model's own, per part."""
    from kicad_parser import parse_kicad_pcb
    from placement.body import board_bodies
    doc, _ = _doc(TRACKED)
    bodies = board_bodies(parse_kicad_pcb(TRACKED), TRACKED)
    wrong = [(p['ref'], p['body_source'],
              getattr(bodies.get(p['ref']), 'source', None))
             for p in doc['parts']
             if p['ref'] in bodies
             and p['body_source'] != bodies[p['ref']].source]
    check('every body source on the sheet is placement.body\'s own',
          wrong == [], f'{len(wrong)}: {wrong[:5]}')
    srcs = {p['body_source'] for p in doc['parts']}
    check('esp_prog exercises more than one rung', len(srcs) >= 3, str(srcs))


def test_panels_are_written_and_referenced():
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'panels')
        doc, _ = _doc(TRACKED, '--panels', out)
        check('panels were written', (doc.get('panels_written') or 0) > 0,
              f"written={doc.get('panels_written')} "
              f"error={doc.get('panels_error')}")
        refd = [p for p in doc['parts'] if p.get('panel')]
        check('the parts reference them', len(refd) > 0)
        check('every referenced panel exists on disk',
              all(os.path.isfile(p['panel']) for p in refd),
              str([p['panel'] for p in refd
                   if not os.path.isfile(p['panel'])][:3]))


def test_md_is_a_sheet_a_reader_can_use():
    r = _run(TRACKED, '--md')
    check('--md exits 0', r.returncode == 0, r.stderr[-300:])
    text = r.stdout
    for want in ('# Component context', '## Pin-order agreement', '## Parts',
                 '## Sources', 'pair /D_P//D_N'):
        check(f'the sheet carries {want!r}', want in text)


TESTS = [test_json_is_parseable_from_char_zero,
         test_the_usb_pair_row,
         test_the_pair_scope_attributes_what_the_interface_blends,
         test_the_mating_face_column,
         test_every_derived_fact_names_its_source,
         test_the_body_column_is_the_896_model,
         test_panels_are_written_and_referenced,
         test_md_is_a_sheet_a_reader_can_use]


def main():
    for t in TESTS:
        print(f'--- {t.__name__}')
        try:
            t()
        except Exception as exc:                             # noqa: BLE001
            check(f'{t.__name__} ran to completion', False,
                  f'{type(exc).__name__}: {exc}')
    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #891 board context, "
          f"{len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f'  - {f}')
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
