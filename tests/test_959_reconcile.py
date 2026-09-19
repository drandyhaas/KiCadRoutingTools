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

RUN_ALL_TIMEOUT = 1200

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
        anchors = {x['name']: x for x in doc['blocks']
                   if (x.get('context') or {}).get('basis') == 'mechanical'}
        assert anchors['mech:Ref*']['refs'] == ['Ref[*]']
        it = fp.intent_from_dict(doc, '')
        pcb = parse_kicad_pcb(b)
        blocks, _ = fp.resolve_blocks(it, pcb, ())
        assert blocks['mech:Ref*'] == ['Ref*'], blocks['mech:Ref*']
        assert blocks['mech:Ref*~2'] == ['Ref*~2'], blocks['mech:Ref*~2']
        # At its declared pose, an anchored part grades clean.
        res = fp.grade(it, pcb, b)
        bad = [v for v in res.violations
               if v.rule in ('zone_containment', 'intent_zone_overlap',
                             'intent_zone_outside_envelope')
               and (v.block or '').startswith('mech:')]
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
        it = fp.intent_from_dict(doc, '')
        env = [v for v in fp.validate_intent(it)
               if v.rule == 'intent_zone_outside_envelope'
               and (v.block or '').startswith('mech:')]
        assert not env, env
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
                blocks, edge_connectors=[{'ref': 'U2', 'edge': 'east'}],
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
        r = run_utils.check(argv, refuse='are not locked in the board: U1',
                            code=4)
        assert "lock 'U1'" in r.stdout
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('place_pose.py'), board, board,
                         'lock', 'U1'], accept=True)
        run_utils.check(argv, accept=True)
    print("  PASS: P1 refuses an unlocked mechanical ref, passes once locked")


TESTS = [
    test_both_shapes_load_and_a_stranger_is_refused,
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
