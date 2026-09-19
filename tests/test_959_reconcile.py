#!/usr/bin/env python3
"""#959 / #1001: reconcile every ref two channels declare; read mechanical.json.

Run 29's `mechanical.json` declared USB1 at the west edge and its design brief
declared USB1 east, and `contradictions` reported `[]`. The #959 comment's
`mechanical_probe` measured it directly: an absent and a deliberately
contradictory `mechanical.json` produced byte-identical intents. The same run
moved a fiducial 17.8 mm off its declared mechanical pose and no gate objected.

Traps written against:

  * a single reading is not a comparison -- every contradiction arm asserts
    BOTH values and their authorities, and the control arm (the file absent)
    asserts the rows are gone;
  * self-labels are not authority -- a zone plan that agrees with the brief
    carries a declaration, one that disagrees is a guess, and the guess must
    read as drift (declared wins) rather than as a contradiction the model
    could clear by writing its own disposition;
  * a fixture that makes the check vacuous -- the anchor arms move a REAL part
    by the distance run 29 moved it, and a turned part is checked apart from a
    moved one.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _sub in ('', 'py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(REPO, _sub) if _sub else REPO
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402
from placement import reconcile as R                        # noqa: E402

RUN_ALL_TIMEOUT = 2400

ESP = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
BRIEF_711 = os.path.join(REPO, 'tests', 'fixtures', '711',
                         'esp_prog.design-brief.json')
FIX = os.path.join(REPO, 'tests', 'fixtures', '959')
PILE = os.path.join(FIX, 'run29_pile.kicad_pcb')
MECH_29 = os.path.join(FIX, 'run29_mechanical.json')
DRIVER = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement',
                      'scripts', 'placement_driver.py')
CHECK = None

#: The #959 comment's `mechanical_probe.py` control, verbatim: USB1 on the
#: WEST edge (the brief says east), C1 fixed at (0, 0) -- off the board -- and
#: a clearance floor of 0.4.
PROBE = {'interfaces': [{'ref': 'USB1', 'edge': 'west'}],
         'fixed': [{'ref': 'C1', 'x': 0, 'y': 0,
                    'reason': 'controlled deliberately incompatible '
                              'declaration'}],
         'project': {'floors': {'clearance': 0.4}}}


def _check():
    return run_utils.tool('check_floorplan.py')


def _stage(tmp, board=ESP, brief=BRIEF_711, mech=None, name='board'):
    """A board in its own directory with a sibling brief (and mechanical)."""
    d = os.path.join(tmp, name)
    os.makedirs(d, exist_ok=True)
    b = os.path.join(d, 'board.kicad_pcb')
    shutil.copy(board, b)
    pro = board[:-len('.kicad_pcb')] + '.kicad_pro'
    if os.path.isfile(pro):
        shutil.copy(pro, b[:-len('.kicad_pcb')] + '.kicad_pro')
    if brief:
        shutil.copy(brief, os.path.join(d, 'board.design-brief.json'))
    if mech is not None:
        p = os.path.join(d, 'mechanical.json')
        if isinstance(mech, str):
            shutil.copy(mech, p)
        else:
            with open(p, 'w', encoding='utf-8') as fh:
                json.dump(mech, fh)
    return b


def _emit(board, out, *extra):
    run_utils.check([sys.executable, '-X', 'utf8', _check(), board,
                     '--emit-intent', out, '--declare-classes',
                     '--allow-unplaced'] + list(extra), accept=True)
    with open(out, encoding='utf-8') as fh:
        return json.load(fh)


def test_both_shapes_load_and_a_stranger_is_refused():
    m = R.load_mechanical(MECH_29)
    assert m['shape'] == 'stage_unaided', m['shape']
    assert sorted(m['poses']) == ['Ref*', 'Ref*~2', 'USB1'], m['poses']
    assert m['poses']['USB1'] == {'x': 117.5, 'y': 100.0, 'rot': 180.0,
                                  'reason': m['poses']['USB1']['reason']}
    assert m['floors']['knobs']['clearance'] == {
        'value': 0.25, 'source': 'fixed default'}, m['floors']
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, 'm.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(PROBE, fh)
        d = R.load_mechanical(p)
        assert d['shape'] == 'declaration' and d['edges'] == {'USB1': 'west'}
        assert d['poses']['C1']['rot'] is None, d['poses']
        assert d['floors']['knobs']['clearance']['value'] == 0.4
        for bad in ({'nonsense': 1}, {'refs': {'U1': [1, 2]}},
                    {'interfaces': [{'ref': 'J1', 'edge': 'up'}]}):
            with open(p, 'w', encoding='utf-8') as fh:
                json.dump(bad, fh)
            try:
                R.load_mechanical(p)
            except R.MechanicalError:
                pass
            else:
                raise AssertionError(f'accepted {bad!r}')
        # ...and the CLI refuses it at exit 2, not as "no mechanical facts".
        b = _stage(tmp, mech={'nonsense': 1}, name='bad')
        run_utils.check([sys.executable, '-X', 'utf8', _check(), b,
                         '--emit-intent', os.path.join(tmp, 'x.json')],
                        refuse='not a mechanical declaration', code=2)
    print("  PASS: both shapes load; three strangers refused; CLI exit 2")


def test_the_comments_probe_is_no_longer_byte_identical():
    """`mechanical_probe.py`: absent vs contradictory. Before #959 the two
    emitted intents were byte-identical with `contradictions: []`."""
    with tempfile.TemporaryDirectory() as tmp:
        a = _emit(_stage(tmp, name='absent'), os.path.join(tmp, 'a.json'))
        c = _emit(_stage(tmp, mech=PROBE, name='contra'),
                  os.path.join(tmp, 'c.json'))
        rows = {r['id']: r for r in c['context']['reconciliation']}
        usb = rows['USB1:edge']
        assert usb['kind'] == 'contradiction', usb
        assert usb['values']['brief']['value'] == 'east'
        assert usb['values']['mechanical']['value'] == 'west'
        assert usb['values']['mechanical']['authority'] == 'recorded_fact'
        assert 'unverified' in usb['values']['mechanical']['source']
        c1 = rows['C1:on_board']
        assert c1['kind'] == 'contradiction' and c1['winner'] == 'outline'
        cl = rows['floors:clearance']
        assert cl['kind'] == 'report', cl
        assert cl['values']['mechanical']['value'] == 0.4
        assert cl['values']['mechanical']['authority'] == 'assumption'
        assert cl['values']['graded']['value'] == 0.25
        a_ids = {r['id'] for r in a['context']['reconciliation']
                 if r['kind'] == 'contradiction'}
        assert not a_ids, a_ids
        assert any('USB1:edge' in s
                   for s in c['context']['brief']['contradictions']), c[
                       'context']['brief']['contradictions']
        assert a != c
    print("  PASS: the probe's two intents now differ; USB1 edge, C1 "
          "off-board and the clearance floor are all named")


def test_no_mechanical_is_the_off_arm():
    with tempfile.TemporaryDirectory() as tmp:
        a = _emit(_stage(tmp, name='none'), os.path.join(tmp, 'a.json'))
        b = _emit(_stage(tmp, mech=PROBE, name='off'),
                  os.path.join(tmp, 'b.json'), '--no-mechanical')
        # The two boards live in different directories, so every PATH differs
        # and nothing else may. Paths are normalised, not dropped: a row that
        # differs in anything but its source path fails.
        def _norm(doc, d):
            return json.loads(json.dumps(doc).replace(
                json.dumps(d)[1:-1], '<dir>'))
        da = os.path.dirname(os.path.join(tmp, 'none', 'x')).replace(
            '\\', '/')
        db_ = os.path.dirname(os.path.join(tmp, 'off', 'x')).replace(
            '\\', '/')
        na = _norm(json.loads(json.dumps(a).replace('\\\\', '/')), da)
        nb = _norm(json.loads(json.dumps(b).replace('\\\\', '/')), db_)
        assert 'mechanical' not in nb['context'], nb['context'].get(
            'mechanical')
        assert na == nb, 'the OFF arm read the file anyway'
    print("  PASS: --no-mechanical emits what an absent file emits")


def test_run29_anchors_and_the_lost_usb1():
    """Run 29's own pile, brief and mechanical.json: USB1's west pose loses to
    the brief's east edge, so it is NOT anchored; the fiducials are, under
    escaped patterns that do not capture each other."""
    with tempfile.TemporaryDirectory() as tmp:
        b = _stage(tmp, board=PILE, mech=MECH_29)
        doc = _emit(b, os.path.join(tmp, 'i.json'))
        mech = doc['context']['mechanical']
        assert mech['anchored'] == ['Ref*', 'Ref*~2'], mech
        assert 'USB1' in mech['skipped'], mech
        pcb = parse_kicad_pcb(b)
        m = R.load_mechanical(os.path.join(os.path.dirname(b),
                                           'mechanical.json'))
        anchors = {x['name']: x for x in R.anchor_blocks(
            pcb, b, m, lost=['USB1'])[0]}
        assert anchors['mech:Ref*']['refs'] == ['Ref[*]']
        assert anchors['mech:Ref*~2']['refs'] == ['Ref[*]~2']
        # At its declared pose, an anchored part grades clean.
        res = fp.grade(fp.intent_from_dict(doc, ''), pcb, b, mechanical=m,
                       mechanical_skip=['USB1'])
        bad = [v for v in res.violations
               if (v.block or '').startswith('mech:')
               or v.rule == 'mechanical_drift']
        assert not bad, bad
    print("  PASS: USB1 not anchored (lost to the brief); Ref[*] resolves "
          "to Ref* alone; anchored parts at their pose grade clean")


def _move(board, ref, dx=0.0, dy=0.0, drot=0.0, out=None):
    """Move one footprint by editing its `(at ...)` -- a test-only pose edit
    of the one block named, never a regex over the whole file."""
    from kicad_parser import iter_footprint_blocks
    text = open(board, encoding='utf-8').read()
    for start, end, _fp_text, _raw, key in iter_footprint_blocks(text):
        if key != ref:
            continue
        block = text[start:end]
        i = block.index('(at ')
        j = block.index(')', i)
        parts = block[i + 4:j].split()
        x, y = float(parts[0]) + dx, float(parts[1]) + dy
        rot = (float(parts[2]) if len(parts) > 2 else 0.0) + drot
        block = block[:i] + f'(at {x:g} {y:g} {rot:g}' + block[j:]
        text = text[:start] + block + text[end:]
        break
    else:
        raise AssertionError(f'{ref} not found')
    with open(out or board, 'w', encoding='utf-8') as fh:
        fh.write(text)


def test_a_moved_or_turned_mechanical_ref_is_graded():
    """Run 29 moved `Ref*` 17.8 mm off (141.2, 95.9) and nothing objected.
    Moved: the anchor's zone_containment ERROR plus mechanical_drift. Turned
    in place: mechanical_drift alone, naming the rotation."""
    with tempfile.TemporaryDirectory() as tmp:
        b = _stage(tmp, board=PILE, mech=MECH_29)
        doc = _emit(b, os.path.join(tmp, 'i.json'))
        it = fp.intent_from_dict(doc, '')
        m = R.load_mechanical(os.path.join(os.path.dirname(b),
                                           'mechanical.json'))
        _move(b, 'Ref*', dx=-17.55, dy=-2.7)
        pcb = parse_kicad_pcb(b)
        res = fp.grade(it, pcb, b, mechanical=m, mechanical_skip=['USB1'])
        zc = [v for v in res.violations if v.rule == 'zone_containment'
              and v.block == 'mech:Ref*']
        md = [v for v in res.violations if v.rule == 'mechanical_drift'
              and v.ref == 'Ref*']
        assert zc and zc[0].severity == 'error', res.violations
        assert md and md[0].severity == 'warn', res.violations
        assert abs(md[0].measured['distance_mm'] - 17.756) < 0.01, md[0]
        _move(b, 'Ref*', dx=17.55, dy=2.7, drot=90)
        pcb = parse_kicad_pcb(b)
        res = fp.grade(it, pcb, b, mechanical=m, mechanical_skip=['USB1'])
        md = [v for v in res.violations if v.rule == 'mechanical_drift'
              and v.ref == 'Ref*']
        assert md and md[0].measured['rotation_off_deg'] == 90.0, md
        assert md[0].measured['distance_mm'] < 0.01, md[0]
        assert md[0].severity == 'error', md[0]
        # A plan cannot DEMOTE the turn: the pose is a recorded fact (PR
        # fact-check: `severity: {mechanical_drift: warn}` read it `warn`).
        # It may promote the move-only WARN.
        demoted = fp.intent_from_dict(dict(doc, severity={
            'mechanical_drift': 'warn'}), '')
        res = fp.grade(demoted, pcb, b, mechanical=m,
                       mechanical_skip=['USB1'])
        md = [v for v in res.violations if v.rule == 'mechanical_drift'
              and v.ref == 'Ref*']
        assert md and md[0].severity == 'error', md
        # The skipped ref (lost to the brief) is never graded against the
        # losing value.
        assert not [v for v in res.violations
                    if v.rule == 'mechanical_drift' and v.ref == 'USB1']
    print("  PASS: moved -> anchor ERROR + drift WARN (17.756 mm); turned "
          "-> drift names 90 deg; the lost ref is never graded")


def test_provenance_verified_then_mismatch():
    with tempfile.TemporaryDirectory() as tmp:
        wd = os.path.join(tmp, 'wd')
        run_utils.check([sys.executable, '-X', 'utf8',
                         os.path.join(REPO, 'tests', 'stress',
                                      'stage_unaided.py'), ESP, wd],
                        accept=True)
        b = os.path.join(wd, 'board.kicad_pcb')
        mp = os.path.join(wd, 'mechanical.json')
        run_utils.evidence(mp, 'the staged mechanical.json')
        assert R.mechanical_provenance(R.load_mechanical(mp), b)[0] == \
            'verified'
        with open(mp, 'a', encoding='utf-8') as fh:
            fh.write(' ')
        st = R.mechanical_provenance(R.load_mechanical(mp), b)
        assert st[0] == 'mismatch', st
        rows = R.reconcile(parse_kicad_pcb(b), b,
                           mechanical=R.load_mechanical(mp))
        pose = [r for r in rows if r['field'] == 'pose']
        assert pose and all(r['values']['mechanical']['authority']
                            == 'hypothesis' for r in pose), pose
    print("  PASS: the staged file is verified; one byte later it is the "
          "run's own writing (hypothesis)")


def test_a_hypothesis_is_drift_not_a_contradiction():
    """A zone plan that says USB1 west while the brief says east is the run's
    guess: DRIFT, the brief wins, and no disposition can make it a decision.
    One that agrees with the brief carries the declaration."""
    pcb = parse_kicad_pcb(ESP)
    from placement import design_brief as db
    frag, _ = db.compile_brief(db.load_brief(BRIEF_711),
                               board_refs=sorted(pcb.footprints))
    west = {'edge_connectors': [{'ref': 'USB1', 'edge': 'west',
                                 'source': 'brief'}]}
    rows = {r['id']: r for r in R.reconcile(pcb, ESP, brief_fragment=frag,
                                            intent_doc=west)}
    r = rows['USB1:edge']
    assert r['kind'] == 'drift' and r['winner'] == 'brief', r
    assert r['values']['intent']['authority'] == 'hypothesis', r
    east = {'edge_connectors': [{'ref': 'USB1', 'edge': 'east'}]}
    rows = {r['id']: r for r in R.reconcile(pcb, ESP, brief_fragment=frag,
                                            intent_doc=east)}
    assert rows['USB1:edge']['values']['intent']['authority'] == 'declared'
    print("  PASS: a self-labelled `source: brief` guess is drift; the "
          "brief's own value carried by the plan is declared")


def test_an_overhanging_mechanical_part_raises_no_envelope_error():
    """33 of 97 mechanical refs overhang the outline by design (tigard H1
    among them). An anchor is where the part IS, so the envelope says nothing
    about it."""
    with tempfile.TemporaryDirectory() as tmp:
        wd = os.path.join(tmp, 'tig')
        tig = os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb')
        run_utils.check([sys.executable, '-X', 'utf8',
                         os.path.join(REPO, 'tests', 'stress',
                                      'stage_unaided.py'), tig, wd],
                        accept=True)
        b = os.path.join(wd, 'board.kicad_pcb')
        doc = _emit(b, os.path.join(tmp, 'i.json'))
        assert 'H1' in doc['context']['mechanical']['anchored'], doc[
            'context']['mechanical']
        m = R.load_mechanical(os.path.join(wd, 'mechanical.json'))
        res = fp.grade(fp.intent_from_dict(doc, ''), parse_kicad_pcb(b), b,
                       mechanical=m)
        bad = [v for v in res.violations
               if (v.block or '').startswith('mech:')
               or v.rule == 'intent_zone_outside_envelope']
        assert not bad, bad
    print("  PASS: tigard's overhanging anchors raise no envelope error")


def test_p1_refuses_a_contradiction_until_dispositioned():
    sys.path.insert(0, os.path.dirname(DRIVER))
    import importlib
    drv = importlib.import_module('placement_driver')
    with tempfile.TemporaryDirectory() as tmp:
        d = os.path.join(tmp, 'b')
        os.makedirs(d)
        board = drv._tiny_board(os.path.join(d, 'board.kicad_pcb'),
                                ('U1', 'U2'), locked=('U2',))
        with open(os.path.join(d, 'board.design-brief.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'board': 'board.kicad_pcb',
                       'interfaces': [{'ref': 'U2', 'edge': 'east',
                                       'user_facing': True}]}, fh)
        with open(os.path.join(d, 'mechanical.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump({'interfaces': [{'ref': 'U2', 'edge': 'west'}]}, fh)
        blocks = [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                   'note': 'both parts, one zone'}]

        def plan(name, **extra):
            p = os.path.join(tmp, name)
            doc = drv._zone_plan_doc(
                blocks, edge_connectors=[{'ref': 'U2', 'edge': 'east',
                                          'class': 'edge_receptacle'}],
                **extra)
            with open(p, 'w', encoding='utf-8') as fh:
                json.dump(doc, fh)
            return p
        argv = [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
                '--board', board]
        r = run_utils.check(argv + ['--zone-plan', plan('a.json')],
                            refuse='contradiction(s) between DECLARED',
                            code=4)
        assert "brief 'east' [declared" in r.stdout, r.stdout
        assert "mechanical 'west' [recorded_fact" in r.stdout, r.stdout
        # The row names its winner, and acknowledging it accepts that.
        assert '-> brief wins' in r.stdout, r.stdout
        assert 'ACCEPTS that winner' in r.stdout, r.stdout
        disp = {'rules': {'envelope': 'fixture', 'legality': 'fixture'},
                'contradictions': {
                    'U2:edge': 'the brief holds: the enclosure moved'}}
        r = run_utils.check(argv + ['--zone-plan',
                                    plan('b.json', dispositions=disp)],
                            accept=True)
        assert '<stage_instructions' in r.stdout
        r = run_utils.check(argv + ['--zone-plan', plan('c.json'),
                                    '--no-mechanical'], accept=True)
    print("  PASS: P1 refuses the contradiction naming both sources, passes "
          "once dispositioned, and --no-mechanical is the OFF arm")


def test_p1_refuses_an_unlocked_mechanical_ref():
    sys.path.insert(0, os.path.dirname(DRIVER))
    import importlib
    drv = importlib.import_module('placement_driver')
    with tempfile.TemporaryDirectory() as tmp:
        board = drv._tiny_board(os.path.join(tmp, 'board.kicad_pcb'),
                                ('U1', 'U2'))
        with open(os.path.join(tmp, 'mechanical.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump({'fixed': [{'ref': 'U1', 'x': 2.0, 'y': 2.0, 'rot': 0,
                                  'reason': 'the datum'}]}, fh)
        p = os.path.join(tmp, 'p.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(drv._zone_plan_doc(
                [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                  'note': 'both'}]), fh)
        argv = [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
                '--board', board, '--zone-plan', p]
        r = run_utils.check(argv, refuse='U1 is not locked', code=4)
        assert 'not held at their declared pose' in r.stdout, r.stdout
        assert "lock 'U1'" in r.stdout
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('place_pose.py'), board, board,
                         'lock', 'U1'], accept=True)
        run_utils.check(argv, accept=True)
    print("  PASS: P1 refuses an unlocked mechanical ref, passes once locked")




def test_the_loader_refuses_every_stranger():
    """Round 1 of the Phase-3 verifier: `reasons` as a list crashed the CLI
    at exit 1, and a `refs`-shape file with a bad `interfaces` row loaded
    with the row silently dropped."""
    base = {'refs': {'U1': [1, 2, 0]}}
    strangers = [
        dict(base, reasons=['not', 'a', 'map']),
        dict(base, interfaces=[{'ref': 'J1', 'edge': 'up'}]),
        dict(base, bogus=1),
        {'fixed': []},
        {'interfaces': [], 'fixed': []},
        {'fixed': [{'ref': 'U1', 'x': 1, 'y': 2}],
         'refs': {'U1': [1, 2, 0]}},
        {'interfaces': [{'ref': 'J1', 'edge': 'east'},
                        {'ref': 'J1', 'edge': 'west'}]},
        {'kind': 'something-else', 'refs': {'U1': [1, 2, 0]}},
        {'fixed': [{'ref': 'U1', 'x': 1, 'y': 2}], 'reasons': {}},
    ]
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, 'm.json')
        for bad in strangers:
            with open(p, 'w', encoding='utf-8') as fh:
                json.dump(bad, fh)
            try:
                R.load_mechanical(p)
            except R.MechanicalError:
                pass
            else:
                raise AssertionError(f'accepted {bad!r}')
        # A refs-shape file MAY carry interfaces; they are read, not dropped.
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(dict(base, interfaces=[{'ref': 'J1', 'edge': 'east'}]),
                      fh)
        assert R.load_mechanical(p)['edges'] == {'J1': 'east'}
        # The CLI refuses the list-shaped `reasons` at exit 2, no traceback.
        b = _stage(tmp, mech=strangers[0], name='bad')
        r = run_utils.check([sys.executable, '-X', 'utf8', _check(), b,
                             '--emit-intent', os.path.join(tmp, 'x.json')],
                            refuse='`reasons` must map ref -> text', code=2)
        assert 'Traceback' not in r.stdout + r.stderr
    print(f"  PASS: {len(strangers)} strangers refused; a refs-shape file's "
          "interfaces are read; the CLI exits 2 without a traceback")


def test_a_plan_cannot_carry_or_claim_an_anchor():
    """B3: `context.basis: mechanical` on a plan block exempted it from the
    envelope and overlap checks. Anchors now come from the file at grade
    time, and a plan may neither declare one nor borrow the exemption."""
    raw = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
           'envelope': {'rect': [0, 0, 20, 10]},
           'blocks': [{'name': 'mech:U1', 'refs': ['U1'],
                       'zone': [0, 0, 2, 2]}]}
    try:
        fp.intent_from_dict(raw, '')
    except fp.IntentError as exc:
        assert 'reserved' in str(exc), exc
    else:
        raise AssertionError('a plan declared a mech: anchor')
    raw['blocks'] = [{'name': 'xtal', 'refs': ['Y1'],
                      'zone': [100, 80, 200, 200],
                      'context': {'basis': 'mechanical'}}]
    env = [v for v in fp.validate_intent(fp.intent_from_dict(raw, ''))
           if v.rule == 'intent_zone_outside_envelope']
    assert env, 'a self-labelled block escaped the envelope check'
    with tempfile.TemporaryDirectory() as tmp:
        doc = _emit(_stage(tmp, board=PILE, mech=MECH_29),
                    os.path.join(tmp, 'i.json'))
        assert not [b for b in doc.get('blocks') or []
                    if b['name'].startswith('mech:')], doc['blocks']
        assert doc['context']['mechanical']['anchored'] == ['Ref*',
                                                            'Ref*~2']
    print("  PASS: `mech:` is refused in a plan, a basis label exempts "
          "nothing, and emit writes no anchor into the plan")


def _grade_run29(b, plan_doc, mech):
    it = fp.intent_from_dict(plan_doc, '')
    return fp.grade(it, parse_kicad_pcb(b), b, mechanical=mech,
                    mechanical_skip=['USB1'])


def _mech_findings(res, ref):
    return ([v for v in res.violations if v.rule == 'zone_containment'
             and v.block == f'mech:{ref}'],
            [v for v in res.violations if v.rule == 'mechanical_drift'
             and v.ref == ref])


def test_the_grade_anchors_from_the_file_whatever_the_plan_says():
    """B1: run 29's own plan carries no anchor, so its moved fiducial graded
    a WARN. The grade now compiles the anchor itself. Boundaries: 0.1 mm off
    is an anchor ERROR (tolerance 0.05); 0.03 mm is not, but IS drift (0.01);
    1 degree is drift; the lost USB1 is never graded, however far it moves."""
    with open(os.path.join(FIX, 'zone_plan_r1.json'), encoding='utf-8') as fh:
        r1 = json.load(fh)
    with tempfile.TemporaryDirectory() as tmp:
        b = _stage(tmp, board=PILE, mech=MECH_29)
        m = R.load_mechanical(MECH_29)
        zc, md = _mech_findings(_grade_run29(b, r1, m), 'Ref*')
        assert not zc and not md, (zc, md)
        _move(b, 'Ref*', dx=-17.55, dy=-2.7)
        zc, md = _mech_findings(_grade_run29(b, r1, m), 'Ref*')
        assert zc and zc[0].severity == 'error', zc
        assert md and abs(md[0].measured['distance_mm'] - 17.756) < 0.01
        # A plan cannot demote it.
        demoted = dict(r1, severity={'zone_containment': 'warn'})
        zc, _ = _mech_findings(_grade_run29(b, demoted, m), 'Ref*')
        assert zc and zc[0].severity == 'error', zc
        _move(b, 'Ref*', dx=17.55 + 0.1, dy=2.7)
        zc, md = _mech_findings(_grade_run29(b, r1, m), 'Ref*')
        assert zc, 'a 0.1 mm move escaped a 0.05 mm anchor'
        _move(b, 'Ref*', dx=-0.07)
        zc, md = _mech_findings(_grade_run29(b, r1, m), 'Ref*')
        assert not zc, zc
        assert md and 0.025 < md[0].measured['distance_mm'] < 0.035, md
        _move(b, 'Ref*', dx=-0.03, drot=1)
        zc, md = _mech_findings(_grade_run29(b, r1, m), 'Ref*')
        assert md and md[0].measured['rotation_off_deg'] == 1.0, md
        _move(b, 'USB1', dx=5.0)
        res = _grade_run29(b, r1, m)
        zc, md = _mech_findings(res, 'USB1')
        assert not zc and not md, (zc, md)
    print("  PASS: anchored from the file with r1 (no anchors): 17.8 mm and "
          "0.1 mm ERROR, 0.03 mm drift only, 1 deg drift, lost USB1 ungraded")


def test_an_off_lattice_or_unrotated_declaration_grades_clean():
    """The anchor is the exact rotated rect unioned with the grader's own,
    so a part at 30 degrees sits in its anchor; and a declaration with no
    `rot` pins no rotation (the verifier's CON2 turned in place got a 7.62 mm
    ERROR)."""
    pcb0 = parse_kicad_pcb(ESP)
    u1 = pcb0.footprints['U1']
    with tempfile.TemporaryDirectory() as tmp:
        b = _stage(tmp, brief=None)
        _move(b, 'U1', drot=30)
        rot = ((u1.rotation or 0.0) + 30) % 360
        mp = os.path.join(tmp, 'm.json')
        with open(mp, 'w', encoding='utf-8') as fh:
            json.dump({'fixed': [{'ref': 'U1', 'x': u1.x, 'y': u1.y,
                                  'rot': rot},
                                 {'ref': 'CON2', 'x': pcb0.footprints[
                                     'CON2'].x, 'y': pcb0.footprints[
                                     'CON2'].y}]}, fh)
        m = R.load_mechanical(mp)
        _move(b, 'CON2', drot=90)
        res = fp.grade(fp.intent_from_dict(_raw_intent(), ''),
                       parse_kicad_pcb(b), b, mechanical=m)
        for ref in ('U1', 'CON2'):
            zc, md = _mech_findings(res, ref)
            assert not zc and not md, (ref, zc, md)
    print("  PASS: U1 at 30 deg and a rot-less CON2 turned 90 deg both grade "
          "clean in their anchors")


def _raw_intent(**extra):
    d = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'}
    d.update(extra)
    return d


def _stage_regime(tmp, name='wd'):
    wd = os.path.join(tmp, name)
    run_utils.check([sys.executable, '-X', 'utf8',
                     os.path.join(REPO, 'tests', 'stress', 'stage_unaided.py'),
                     ESP, wd], accept=True)
    return wd, os.path.join(wd, 'board.kicad_pcb'), os.path.join(
        wd, 'mechanical.json')


def test_the_regime_owns_its_mechanical_file():
    """SF1: under a regime that recorded a mechanical.json, the run cannot
    make it disappear -- the flag, another path, a rewrite and a deletion
    are all exit 2 -- and a lap board copied elsewhere still reads it."""
    with tempfile.TemporaryDirectory() as tmp:
        wd, b, mp = _stage_regime(tmp)
        out = os.path.join(tmp, 'x.json')
        emit = [sys.executable, '-X', 'utf8', _check(), b, '--emit-intent',
                out, '--allow-unplaced']
        run_utils.check(emit + ['--no-mechanical'],
                        refuse='a recorded input cannot be switched off',
                        code=2)
        other = os.path.join(tmp, 'other.json')
        with open(other, 'w', encoding='utf-8') as fh:
            json.dump({'interfaces': [{'ref': 'USB1', 'edge': 'east'}]}, fh)
        run_utils.check(emit + ['--mechanical', other],
                        refuse='another file is not it', code=2)
        lap = os.path.join(wd, 'laps', 'lap1')
        os.makedirs(lap)
        run_utils.check([sys.executable, '-X', 'utf8',
                         os.path.join(REPO, 'py_router', 'copy_board.py'), b,
                         os.path.join(lap, 'board.kicad_pcb')], accept=True)
        r = run_utils.check([sys.executable, '-X', 'utf8', _check(),
                             os.path.join(lap, 'board.kicad_pcb'),
                             '--emit-intent', out, '--allow-unplaced'],
                            accept=True)
        assert os.path.abspath(mp) in r.stdout, r.stdout[:1500]
        with open(mp, 'a', encoding='utf-8') as fh:
            fh.write(' ')
        run_utils.check(emit, refuse='changed after staging', code=2)
        os.remove(mp)
        run_utils.check(emit, refuse='is gone -- restore it', code=2)
    print("  PASS: under a regime --no-mechanical, another file, a rewrite "
          "and a deletion are exit 2; a copied lap board keeps the file")


def test_a_run_written_lock_is_not_a_recorded_fact():
    """B2: the staged board is edited in place, so its CURRENT locks said
    nothing about what existed before the run. Now: `staged_lock_poses` in
    the manifest, and a board pose is a recorded fact only while the part is
    locked WHERE it was locked at staging."""
    with tempfile.TemporaryDirectory() as tmp:
        wd, b, mp = _stage_regime(tmp)
        from placement import provenance as PV
        man_p = os.path.join(wd, PV.REGIME_NAME)
        with open(man_p, encoding='utf-8') as fh:
            man = json.load(fh)
        assert isinstance(man.get('staged_lock_poses'), dict), sorted(man)
        m = R.load_mechanical(mp)
        x, y, rot = (m['poses']['Ref*'][k] for k in ('x', 'y', 'rot'))

        def auth():
            rows = {r['id']: r for r in R.reconcile(parse_kicad_pcb(b), b,
                                                    mechanical=m)}
            row = rows['Ref*:pose']
            return row['values']['board']['authority'], row['kind']
        pose = [sys.executable, '-X', 'utf8', run_utils.tool('place_pose.py'),
                b, b]
        # The run's own lock, even AT the declared pose, is its writing.
        run_utils.check(pose + ['lock', 'Ref*'], accept=True)
        assert auth() == ('hypothesis', 'agree'), auth()
        # A lock the manifest recorded at staging, still there: a fact.
        man['staged_lock_poses'] = {'Ref*': [x, y, rot]}
        with open(man_p, 'w', encoding='utf-8') as fh:
            json.dump(man, fh)
        assert auth() == ('recorded_fact', 'agree'), auth()
        # ...moved by the run and locked again: the run's writing -- drift
        # the declaration wins, never a contradiction to disposition away.
        run_utils.check(pose + ['unlock', 'Ref*', 'set', 'Ref*', '115.6',
                                '92.2'], accept=True)
        run_utils.check(pose + ['lock', 'Ref*'], accept=True)
        assert auth() == ('hypothesis', 'drift'), auth()
    print("  PASS: a run lock is a hypothesis; a staged lock in place is a "
          "recorded fact; a staged lock moved is drift, not a contradiction")


def test_p1_refuses_the_run29_move_the_plan_never_anchored():
    """B1 end to end, the verifier's own scenario: run 29's pile, brief,
    mechanical.json and r1 plan (with its other debts answered); `Ref*`
    moved 25.9 mm and locked. P1 passed. It must refuse, print the command
    that puts it back, and never demand the LOST USB1 be locked west."""
    with open(os.path.join(FIX, 'zone_plan_r1.json'), encoding='utf-8') as fh:
        plan = json.load(fh)
    plan['dispositions'] = {
        'refs': {k: 'a logo; test' for k in (
            '#00000000-0000-0000-0000-00005a3b5201',
            '#00000000-0000-0000-0000-00005d8c51dd',
            '#00000000-0000-0000-0000-00005e7dd057')},
        'contradictions': {'USB1:edge': 'the brief holds'},
        'rules': {'decap_distance': 'test'},
        'withheld': {'overlap_area': 'test'}}
    with tempfile.TemporaryDirectory() as tmp:
        b = _stage(tmp, board=PILE, mech=MECH_29)
        pp = os.path.join(tmp, 'plan.json')
        with open(pp, 'w', encoding='utf-8') as fh:
            json.dump(plan, fh)
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('place_pose.py'), b, b, 'set', 'Ref*',
                         '115.6', '92.2', 'lock', 'Ref*', 'lock', 'Ref*~2'],
                        accept=True)
        # USB1 LOST its edge to the brief, so its mechanical pose is not a
        # target: moved off it (parked clear of Ref*'s declared pose), it must not be
        # demanded back (a lost ref is skipped by the drift check too).
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('place_pose.py'), b, b, 'set', 'USB1',
                         '125', '104', '--rot', '180', '--force'],
                        accept=True)
        argv = [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
                '--board', b, '--zone-plan', pp,
                '--waive', 'seed-connectors:the probe hands them over']
        r = run_utils.check(argv, refuse='not held at their declared pose',
                            code=4)
        assert "Ref* is 25.866mm from its declared" in r.stdout, r.stdout
        assert "unlock 'Ref*' set 'Ref*' 141.2 95.9 --rot 0.0\n" in \
            r.stdout, r.stdout
        assert "'USB1'" not in r.stdout.split('not held')[1], r.stdout
        # The printed remedy, run as printed: two calls.
        pose = [sys.executable, '-X', 'utf8',
                run_utils.tool('place_pose.py'), b, b]
        run_utils.check(pose + ['unlock', 'Ref*', 'set', 'Ref*', '141.2',
                                '95.9', '--rot', '0.0'], accept=True)
        run_utils.check(pose + ['lock', 'Ref*'], accept=True)
        r = subprocess.run(argv, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', cwd=REPO,
                           timeout=900)
        assert 'not held at their declared pose' not in r.stdout, r.stdout
    print("  PASS: run 29's moved-and-locked Ref* is refused at P1 with the "
          "command that restores it; USB1 (lost) is not demanded")


def test_p1_drift_command_carries_unlock_and_allow_routed():
    sys.path.insert(0, os.path.dirname(DRIVER))
    import importlib
    drv = importlib.import_module('placement_driver')
    with tempfile.TemporaryDirectory() as tmp:
        board = drv._tiny_board(os.path.join(tmp, 'board.kicad_pcb'),
                                ('U1', 'U2'))
        with open(os.path.join(tmp, 'mechanical.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump({'fixed': [{'ref': 'U1', 'x': 2.0, 'y': 2.0, 'rot': 0,
                                  'reason': 'the datum'}]}, fh)
        p = os.path.join(tmp, 'p.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(drv._zone_plan_doc(
                [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                  'note': 'both'}]), fh)
        argv = [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
                '--board', board, '--zone-plan', p]
        pose = [sys.executable, '-X', 'utf8', run_utils.tool('place_pose.py'),
                board, board]
        run_utils.check(pose + ['set', 'U1', '3', '2', 'lock', 'U1'],
                        accept=True)
        r = run_utils.check(argv, refuse='U1 is 1.000mm from its declared',
                            code=4)
        assert "unlock 'U1' set 'U1' 2.0 2.0 --rot 0.0\n" in r.stdout, \
            r.stdout
        assert r.stdout.count(f"{board} lock 'U1'") == 1, r.stdout
        # Run exactly what it printed, and P1 has nothing left to say
        # about U1.
        cmds = [ln.strip() for ln in r.stdout.splitlines()
                if ln.strip().startswith('python3 -X utf8 py_placer/'
                                         'place_pose.py')]
        assert len(cmds) == 2, cmds
        import shlex
        for c in cmds:
            # The board path (backslashes on Windows) is taken out before
            # the POSIX split, which would read them as escapes.
            rest = c.split('place_pose.py ', 1)[1].replace(board, '', 2)
            run_utils.check(pose + shlex.split(rest), accept=True)
        r2 = subprocess.run(argv, capture_output=True, text=True,
                            encoding='utf-8', errors='replace', cwd=REPO,
                            timeout=900)
        assert 'not held at their declared pose' not in r2.stdout, r2.stdout
        run_utils.check(pose + ['unlock', 'U1', 'set', 'U1', '3', '2'],
                        accept=True)
        run_utils.check(pose + ['lock', 'U1'], accept=True)
        assert '--allow-routed' not in r.stdout
        text = open(board, encoding='utf-8').read().rstrip()
        assert text.endswith(')')
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write(text[:-1] + '  (segment (start 1 8) (end 4 8) (width '
                     '0.2) (layer "F.Cu") (net 1) (uuid "s1"))\n)\n')
        r = run_utils.check(argv, refuse='U1 is 1.000mm from its declared',
                            code=4)
        assert '--allow-routed unlock' in r.stdout, r.stdout
        assert '--allow-routed lock' in r.stdout, r.stdout
    print("  PASS: P1's drift refusal prints unlock, and --allow-routed on "
          "a board that carries copper")


def test_stale_contradiction_answers_agree_between_grader_and_p1():
    """SF5: check_floorplan reported `stale_dispositions: []` for a
    contradiction id P1 refused as stale."""
    sys.path.insert(0, os.path.dirname(DRIVER))
    import importlib
    drv = importlib.import_module('placement_driver')
    with tempfile.TemporaryDirectory() as tmp:
        board = drv._tiny_board(os.path.join(tmp, 'board.kicad_pcb'),
                                ('U1', 'U2'))
        p = os.path.join(tmp, 'p.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(drv._zone_plan_doc(
                [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                  'note': 'both'}], dispositions={
                    'rules': {'envelope': 'fixture', 'legality': 'fixture'},
                    'contradictions': {'BOGUS:edge': 'x'}}), fh)
        r = subprocess.run([sys.executable, '-X', 'utf8', _check(), board,
                            '--intent', p, '--plan-only'],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=REPO, timeout=900)
        line = [x for x in r.stdout.splitlines()
                if x.startswith('JSON_SUMMARY:')][-1]
        s = json.loads(line.split('JSON_SUMMARY: ', 1)[1])
        assert s['stale_dispositions'] == [
            'dispositions.contradictions.BOGUS:edge: no such contradiction '
            'on this board and these inputs'], s['stale_dispositions']
        run_utils.check([sys.executable, '-X', 'utf8', DRIVER, '--stage',
                         'P1', '--board', board, '--zone-plan', p],
                        refuse='dispositions.contradictions.BOGUS:edge '
                               'answers no contradiction', code=4)
        # ...and the GRADE path, not only --plan-only.
        r = subprocess.run([sys.executable, '-X', 'utf8', _check(), board,
                            '--intent', p],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=REPO, timeout=900)
        line = [x for x in r.stdout.splitlines()
                if x.startswith('JSON_SUMMARY:')][-1]
        s = json.loads(line.split('JSON_SUMMARY: ', 1)[1])
        assert any('contradictions.BOGUS:edge' in x
                   for x in s['stale_dispositions']), s['stale_dispositions']
    print("  PASS: a stale contradiction answer is named by the grader and "
          "refused by P1 alike")


def test_rows_read_edges_the_way_the_grader_does():
    """SF2: reading every part's edge off its drawn body disagreed with the
    grader, which reads the courtyard for a part that is not an edge
    receptacle. rp2350's SW1 (declared south, top_mount) graded PASS and
    reconciled as a contradiction. And N3: a ref whose mechanical value lost
    one row does not win its others."""
    rp = os.path.join(REPO, 'kicad_files',
                      'rp2350_fpga_eensy_prePlane.kicad_pcb')
    pcb = parse_kicad_pcb(rp)
    sw = pcb.footprints['SW1']
    from placement import design_brief as db
    with tempfile.TemporaryDirectory() as tmp:
        bp = os.path.join(tmp, 'b.design-brief.json')
        with open(bp, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'interfaces': [{'ref': 'SW1', 'edge': 'south',
                                       'user_facing': False,
                                       'mount_mode': 'top_mount'}]}, fh)
        frag, _ = db.compile_brief(db.load_brief(bp),
                                   board_refs=sorted(pcb.footprints))
    mech = {'path': rp, 'sha256': 'x', 'shape': 'stage_unaided',
            'poses': {'SW1': {'x': sw.x, 'y': sw.y,
                              'rot': (sw.rotation or 0.0) % 360,
                              'reason': 'test'}},
            'edges': {}, 'floors': {'knobs': {}, 'unavailable': None}}
    rows = {r['id']: r for r in R.reconcile(pcb, rp, brief_fragment=frag,
                                            mechanical=mech)}
    assert rows['SW1:edge']['kind'] != 'contradiction', rows['SW1:edge']
    with tempfile.TemporaryDirectory() as tmp:
        c = _emit(_stage(tmp, mech=PROBE, name='contra'),
                  os.path.join(tmp, 'c.json'))
        rows = {r['id']: r for r in c['context']['reconciliation']}
        assert rows['C1:on_board']['winner'] == 'outline'
        assert rows['C1:pose']['winner'] != 'mechanical', rows['C1:pose']
    print("  PASS: SW1 reconciles as the grader reads it; C1's pose row no "
          "longer names the losing mechanical value its winner")


def test_floors_unavailable_is_reported():
    pcb = parse_kicad_pcb(ESP)
    mech = {'path': MECH_29, 'sha256': 'x', 'shape': 'stage_unaided',
            'poses': {}, 'edges': {},
            'floors': {'knobs': {},
                       'unavailable': 'unavailable: no project file'}}
    rows = {r['id']: r for r in R.reconcile(
        pcb, ESP, mechanical=mech,
        floors_used={'clearance': {'value': 0.25,
                                   'source': 'fixed default'}})}
    row = rows['floors:clearance']
    assert row['kind'] == 'report', row
    assert row['values']['mechanical']['value'] is None
    assert 'no project file' in row['values']['mechanical']['source']
    assert row['values']['graded']['value'] == 0.25
    print("  PASS: an 'unavailable' floor is a report naming why")


def test_p1_refuses_plan_drift_and_honours_a_named_waiver():
    sys.path.insert(0, os.path.dirname(DRIVER))
    import importlib
    drv = importlib.import_module('placement_driver')
    with tempfile.TemporaryDirectory() as tmp:
        d = os.path.join(tmp, 'b')
        os.makedirs(d)
        board = drv._tiny_board(os.path.join(d, 'board.kicad_pcb'),
                                ('U1', 'U2'), locked=('U2',))
        with open(os.path.join(d, 'board.design-brief.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'board': 'board.kicad_pcb',
                       'interfaces': [{'ref': 'U2', 'edge': 'east',
                                       'user_facing': True}]}, fh)
        p = os.path.join(tmp, 'p.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(drv._zone_plan_doc(
                [{'name': 'all', 'refs': ['U*'], 'zone': [0, 0, 10, 10],
                  'note': 'both'}],
                edge_connectors=[{'ref': 'U2', 'edge': 'west',
                                  'class': 'edge_receptacle'}]), fh)
        argv = [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
                '--board', board, '--zone-plan', p]
        r = run_utils.check(argv, refuse='drops or contradicts 1 clause(s)',
                            code=4)
        cid = r.stdout.split('of the design brief:\n  - ', 1)[1].split(
            ': ', 1)[0]
        assert 'U2' in cid, cid
        run_utils.check(argv + ['--waive', f'brief-clause:{cid}:'],
                        refuse='needs a REASON', code=4)
        run_utils.check(argv + ['--waive',
                                f'brief-clause:{cid}:the enclosure moved'],
                        accept=True)
        # With a second dropped clause, waiving the first leaves the second
        # refused, by name.
        with open(os.path.join(d, 'board.design-brief.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'board': 'board.kicad_pcb',
                       'interfaces': [{'ref': 'U2', 'edge': 'east',
                                       'user_facing': True}],
                       'proximity': [{'ref': 'U1', 'near': 'U2',
                                      'max_mm': 5.0, 'requirement': 'R',
                                      'why': 'test'}]}, fh)
        r = run_utils.check(argv + ['--waive',
                                    f'brief-clause:{cid}:the enclosure moved'],
                            refuse='drops or contradicts 1 clause(s)',
                            code=4)
        assert 'proximity[' in r.stdout and cid not in r.stdout.split(
            'of the design brief:')[1].split('The brief is')[0], r.stdout
    print(f"  PASS: P1 refuses a plan that drifts from the brief ({cid}) and "
          "passes it once waived by name with a reason")




def test_round2_the_stagers_empty_declaration_is_a_declaration():
    """Round-2 BLOCKING 1: `stage_unaided` writes `refs: {}` for a board with
    no mechanical parts (9 of 22 corpus boards), the loader refused it, and
    the regime refused every remedy -- every unaided run on those boards
    dead-ended. A hand-written empty file is still refused."""
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, 'm.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'mechanical-declaration',
                       'refs': {}, 'reasons': {}, 'note': 'x'}, fh)
        m = R.load_mechanical(p)
        assert m['poses'] == {} and m['edges'] == {}, m
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump({'refs': {}}, fh)
        try:
            R.load_mechanical(p)
        except R.MechanicalError:
            pass
        else:
            raise AssertionError('a kind-less empty refs map loaded')
        wd = os.path.join(tmp, 'wd')
        run_utils.check([sys.executable, '-X', 'utf8',
                         os.path.join(REPO, 'tests', 'stress',
                                      'stage_unaided.py'),
                         os.path.join(REPO, 'kicad_files',
                                      'cap_chain.kicad_pcb'), wd],
                        accept=True)
        run_utils.check([sys.executable, '-X', 'utf8', _check(),
                         os.path.join(wd, 'board.kicad_pcb'),
                         '--emit-intent', os.path.join(tmp, 'i.json'),
                         '--allow-unplaced'], accept=True)
    print("  PASS: the stager's empty declaration loads, and a staged board "
          "with no mechanical parts emits; a kind-less empty file is refused")


def test_round2_a_brief_written_in_the_run_cannot_outrank_the_record():
    """Round-2 BLOCKING 2: under an unaided regime the placement skill has the
    RUN write the brief, so one brief row declaring `Ref*` on another edge
    beat the recorded mechanical pose and unanchored it. The brief is the
    run's reading there -- a hypothesis -- unless the manifest recorded it."""
    from placement import provenance as PV
    with tempfile.TemporaryDirectory() as tmp:
        wd, b, mp = _stage_regime(tmp)
        bp = os.path.join(wd, 'board.design-brief.json')
        with open(bp, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'board': 'board.kicad_pcb',
                       'interfaces': [{'ref': 'Ref*', 'edge': 'north'}]}, fh)
        from placement import design_brief as db
        frag, _ = db.compile_brief(db.load_brief(bp),
                                   board_refs=sorted(parse_kicad_pcb(
                                       b).footprints))
        m = R.load_mechanical(mp)

        def row():
            rows = {r['id']: r for r in R.reconcile(
                parse_kicad_pcb(b), b, brief_fragment=frag, brief_source=bp,
                mechanical=m)}
            return rows.get('Ref*:edge'), R.lost_mechanical_refs(
                list(rows.values()))
        r, lost = row()
        assert r['values']['brief']['authority'] == 'hypothesis', r
        assert r['kind'] == 'drift' and r['winner'] == 'mechanical', r
        assert 'Ref*' not in lost, lost
        # A manifest that recorded this brief at staging makes it a
        # declaration again -- and then a contradiction to acknowledge.
        man_p = os.path.join(wd, PV.REGIME_NAME)
        man = json.load(open(man_p, encoding='utf-8'))
        man['brief_sha256'] = R._sha256(bp)
        json.dump(man, open(man_p, 'w', encoding='utf-8'))
        r, lost = row()
        assert r['values']['brief']['authority'] == 'declared', r
        assert r['kind'] == 'contradiction', r
    print("  PASS: a brief the run wrote is a hypothesis the record outranks; "
          "one the regime recorded is a declaration")


def test_p1_refuses_a_run_written_value_the_record_outranks():
    """Pre-push review BLOCKING: under an unaided regime the brief is the
    run's reading, so run 29's own case -- brief USB1 east, mechanical.json
    west -- is drift the recorded value wins. P1 passed it: it demanded the
    mechanical refs locked (west) while the brief-clause check demanded the
    plan carry EAST, and the grade then failed on a locked part. P1 must
    refuse the losing values and name both; no disposition answers it."""
    with tempfile.TemporaryDirectory() as tmp:
        wd, b, _mp = _stage_regime(tmp)
        bp = os.path.join(wd, 'board.design-brief.json')
        with open(bp, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'board': 'board.kicad_pcb',
                       'interfaces': [{'ref': 'USB1', 'edge': 'east',
                                       'user_facing': True}]}, fh)
        bounds = parse_kicad_pcb(b).board_info.board_bounds
        plan = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                'blocks': [{'name': 'all', 'refs': ['*'],
                            'zone': [round(v, 3) for v in bounds],
                            'note': 'everything, one zone'}],
                'edge_connectors': [{'ref': 'USB1', 'edge': 'east'}],
                'dispositions': {'refs': {k: 'a logo; test' for k in (
                    '#00000000-0000-0000-0000-00005a3b5201',
                    '#00000000-0000-0000-0000-00005d8c51dd',
                    '#00000000-0000-0000-0000-00005e7dd057')}}}
        pp = os.path.join(tmp, 'plan.json')
        with open(pp, 'w', encoding='utf-8') as fh:
            json.dump(plan, fh)
        r = run_utils.check(
            [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
             '--board', b, '--zone-plan', pp,
             '--waive', 'seed-connectors:the probe hands them over'],
            refuse='disagree with a RECORDED fact', code=4)
        line = [x for x in r.stdout.splitlines() if 'USB1:edge' in x]
        assert line and "mechanical 'west'" in line[0] and (
            "brief 'east' [hypothesis" in line[0]) and (
            "intent 'east' [hypothesis" in line[0]), r.stdout[-2000:]
        # A disposition does not answer it: acknowledging it changes nothing.
        plan['dispositions']['contradictions'] = {'USB1:edge': 'accepted'}
        with open(pp, 'w', encoding='utf-8') as fh:
            json.dump(plan, fh)
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', DRIVER, '--stage', 'P1',
             '--board', b, '--zone-plan', pp,
             '--waive', 'seed-connectors:the probe hands them over'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=REPO, timeout=900)
        assert r.returncode == 4 and ('disagree with a RECORDED fact' in
                                      r.stdout or 'answers contradictions'
                                      in r.stdout), r.stdout[-1500:]
    print("  PASS: a run-written brief and plan that the recorded edge "
          "outranks are refused at P1, both values named")


def test_a_board_with_no_outline_still_exits_3_with_a_brief():
    """Pre-push review: reconciliation ran before the grade's outline check,
    and on a board with no outline a brief that declares an edge turned the
    base's exit 3 into a traceback (exit 1)."""
    with tempfile.TemporaryDirectory() as tmp:
        text = open(ESP, encoding='utf-8').read()
        # Every top-level `(gr_...` block on Edge.Cuts, removed by a
        # paren-balanced scan (the blocks span several lines).
        out, i = [], 0
        while True:
            j = text.find('(gr_', i)
            if j < 0:
                out.append(text[i:])
                break
            depth, k = 0, j
            while True:
                depth += {'(': 1, ')': -1}.get(text[k], 0)
                k += 1
                if depth == 0:
                    break
            out.append(text[i:j])
            if '"Edge.Cuts"' not in text[j:k]:
                out.append(text[j:k])
            i = k
        b = os.path.join(tmp, 'noedge.kicad_pcb')
        with open(b, 'w', encoding='utf-8') as fh:
            fh.write(''.join(out))
        from placement.floorplan import outline_state
        npcb = parse_kicad_pcb(b)
        assert 'USB1' in npcb.footprints
        assert not outline_state(npcb, b)['trustworthy']
        with open(os.path.join(tmp, 'noedge.design-brief.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                       'board': 'noedge.kicad_pcb',
                       'interfaces': [{'ref': 'USB1', 'edge': 'east'}]}, fh)
        intent = os.path.join(tmp, 'i.json')
        with open(intent, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent',
                       'units': 'mm'}, fh)
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            run_utils.tool('check_floorplan.py'), b,
                            '--intent', intent],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=REPO, timeout=600)
        assert 'Traceback' not in r.stderr + r.stdout, r.stderr[-1500:]
        assert r.returncode == 3, (r.returncode, r.stdout[-800:],
                                   r.stderr[-800:])
    print("  PASS: no outline + a brief -> exit 3, no traceback")


def test_round2_a_moved_run_dir_keeps_its_declaration():
    """Round-2 SHOULD-FIX: the regime bound an absolute path, so an archived
    or relocated run dir exited 2 with a sha-matching file beside it."""
    import shutil as _sh
    with tempfile.TemporaryDirectory() as tmp:
        wd, b, mp = _stage_regime(tmp)
        wd2 = os.path.join(tmp, 'moved')
        _sh.move(wd, wd2)
        b2 = os.path.join(wd2, 'board.kicad_pcb')
        r = run_utils.check([sys.executable, '-X', 'utf8', _check(), b2,
                             '--emit-intent', os.path.join(tmp, 'x.json'),
                             '--allow-unplaced'], accept=True)
        assert os.path.join(wd2, 'mechanical.json') in r.stdout, r.stdout[
            :1200]
        # ...and a relocated file with OTHER bytes is still refused.
        with open(os.path.join(wd2, 'mechanical.json'), 'a',
                  encoding='utf-8') as fh:
            fh.write(' ')
        run_utils.check([sys.executable, '-X', 'utf8', _check(), b2,
                         '--emit-intent', os.path.join(tmp, 'y.json'),
                         '--allow-unplaced'],
                        refuse='is gone -- restore it', code=2)
    print("  PASS: a moved run dir reads its sha-matching declaration; other "
          "bytes are refused")


def test_round2_turns_and_padless_drift_are_errors_and_the_anchor_is_tight():
    """Round-2 SHOULD-FIX: a symmetric body sits inside its anchor turned 180
    (68 of 97 corpus refs), so a turn must be an ERROR on its own; a pad-less
    ref has no anchor, so its drift must be too. And an anchor with no
    declared `rot` is built at the part's current rotation -- the union over
    rotations admitted a 16 mm move."""
    pcb0 = parse_kicad_pcb(ESP)
    con2 = pcb0.footprints['CON2']
    logo = sorted(k for k, f in pcb0.footprints.items() if not f.pads)[0]
    lf = pcb0.footprints[logo]
    with tempfile.TemporaryDirectory() as tmp:
        b = _stage(tmp, brief=None)
        mp = os.path.join(tmp, 'm.json')
        with open(mp, 'w', encoding='utf-8') as fh:
            json.dump({'fixed': [
                {'ref': 'CON2', 'x': con2.x, 'y': con2.y},
                {'ref': 'Ref*', 'x': pcb0.footprints['Ref*'].x,
                 'y': pcb0.footprints['Ref*'].y,
                 'rot': (pcb0.footprints['Ref*'].rotation or 0) % 360},
                {'ref': logo, 'x': lf.x + 3.0, 'y': lf.y}]}, fh)
        m = R.load_mechanical(mp)
        _move(b, 'CON2', dy=6.0)
        _move(b, 'Ref*', drot=180)
        res = fp.grade(fp.intent_from_dict(_raw_intent(), ''),
                       parse_kicad_pcb(b), b, mechanical=m)
        zc, _ = _mech_findings(res, 'CON2')
        assert zc and zc[0].severity == 'error', res.violations
        _zc, md = _mech_findings(res, 'Ref*')
        assert md and md[0].severity == 'error', md
        _zc, md = _mech_findings(res, logo)
        assert md and md[0].severity == 'error', md
    print("  PASS: CON2 (no rot) moved 6 mm is an anchor ERROR; a 180-degree "
          "turn and a pad-less drift are drift ERRORs")


def test_round2_old_manifests_and_ledger_statuses():
    """Mutation survivors: the sha-verified fallback for a manifest recorded
    before `staged_lock_poses`, the reconciliation ledger statuses, floors
    REPORT rows kept out of the ledger's carried facts, and the grade path's
    stale contradiction answers."""
    from placement import provenance as PV
    with tempfile.TemporaryDirectory() as tmp:
        wd, b, mp = _stage_regime(tmp)
        man_p = os.path.join(wd, PV.REGIME_NAME)
        pose = [sys.executable, '-X', 'utf8', run_utils.tool('place_pose.py'),
                b, b]
        run_utils.check(pose + ['lock', 'Ref*'], accept=True)
        man = json.load(open(man_p, encoding='utf-8'))
        man.pop('staged_lock_poses', None)
        json.dump(man, open(man_p, 'w', encoding='utf-8'))
        # The staged board changed (the lock), so nothing vouches for it.
        assert R.staged_lock_poses(man) is None
        # Were the sha still the recorded one, its locks WOULD be pre-run.
        man['staged_sha256'] = R._sha256(b)
        assert 'Ref*' in (R.staged_lock_poses(man) or {}), man
    it = fp.intent_from_dict(_raw_intent(dispositions={'contradictions': {
        'X:edge': 'the brief holds'}}), '')
    rows = [{'id': 'X:edge', 'ref': 'X', 'field': 'edge',
             'kind': 'contradiction', 'winner': 'brief',
             'values': {'brief': {'value': 'east', 'authority': 'declared'},
                        'mechanical': {'value': 'west',
                                       'authority': 'recorded_fact'}},
             'why': ''},
            {'id': 'Y:edge', 'ref': 'Y', 'field': 'edge',
             'kind': 'contradiction', 'winner': 'brief',
             'values': {'brief': {'value': 'east', 'authority': 'declared'},
                        'mechanical': {'value': 'west',
                                       'authority': 'recorded_fact'}},
             'why': ''},
            {'id': 'Z:edge', 'ref': 'Z', 'field': 'edge', 'kind': 'drift',
             'winner': 'brief',
             'values': {'brief': {'value': 'east', 'authority': 'declared'},
                        'board': {'value': 'west', 'authority': 'inferred'}},
             'why': ''},
            {'id': 'floors:clearance', 'ref': None, 'field': 'floors.x',
             'kind': 'report', 'winner': 'graded',
             'values': {'graded': {'value': 0.25,
                                   'authority': 'assumption'}},
             'why': ''}]
    led = {x['id']: x for x in fp.declaration_ledger(it, [],
                                                     reconciliation=rows)}
    assert led['reconcile:X:edge']['status'] == 'dispositioned', led
    assert led['reconcile:Y:edge']['status'] == 'graded_fail', led
    assert led['reconcile:Z:edge']['status'] == 'pending', led
    assert 'reconcile:floors:clearance' not in led, sorted(led)
    print("  PASS: an old manifest's locks count only while its sha holds; "
          "the ledger reads acknowledged, open and board-only rows apart, "
          "and a floor assumption is not a carried fact")


TESTS = [
    test_both_shapes_load_and_a_stranger_is_refused,
    test_round2_the_stagers_empty_declaration_is_a_declaration,
    test_round2_a_brief_written_in_the_run_cannot_outrank_the_record,
    test_p1_refuses_a_run_written_value_the_record_outranks,
    test_a_board_with_no_outline_still_exits_3_with_a_brief,
    test_round2_a_moved_run_dir_keeps_its_declaration,
    test_round2_turns_and_padless_drift_are_errors_and_the_anchor_is_tight,
    test_round2_old_manifests_and_ledger_statuses,
    test_the_loader_refuses_every_stranger,
    test_a_plan_cannot_carry_or_claim_an_anchor,
    test_the_grade_anchors_from_the_file_whatever_the_plan_says,
    test_an_off_lattice_or_unrotated_declaration_grades_clean,
    test_the_regime_owns_its_mechanical_file,
    test_a_run_written_lock_is_not_a_recorded_fact,
    test_p1_refuses_the_run29_move_the_plan_never_anchored,
    test_p1_drift_command_carries_unlock_and_allow_routed,
    test_stale_contradiction_answers_agree_between_grader_and_p1,
    test_rows_read_edges_the_way_the_grader_does,
    test_floors_unavailable_is_reported,
    test_p1_refuses_plan_drift_and_honours_a_named_waiver,
    test_the_comments_probe_is_no_longer_byte_identical,
    test_no_mechanical_is_the_off_arm,
    test_run29_anchors_and_the_lost_usb1,
    test_a_moved_or_turned_mechanical_ref_is_graded,
    test_provenance_verified_then_mismatch,
    test_a_hypothesis_is_drift_not_a_contradiction,
    test_an_overhanging_mechanical_part_raises_no_envelope_error,
    test_p1_refuses_a_contradiction_until_dispositioned,
    test_p1_refuses_an_unlocked_mechanical_ref,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
