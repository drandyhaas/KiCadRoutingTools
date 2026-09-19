#!/usr/bin/env python3
"""#959 / #1000: connector declarations compile to clauses a rule grades.

Run 29's brief declared `mount_mode`, `cable_entry` and `user_top_side`, and
the #959 comment measured that changing any of them changed NO measurement:
all three were carried into `context` and graded by nothing. This compiles
them -- by the evidence the Phase-0 control measured on five as-built boards,
not by the issue's literal mapping:

  * `edge_mount` -> the drawn body within 0.75 mm of its edge (tigard J7, an
    edge-mount header on a shipping board, sits 0.60 mm in);
  * `through_edge` -> the body reaches the edge (past it, or within 0.75);
  * `top_mount` / `bottom_mount` -> NOT held to the receptacle seat (8
    vertical headers on the as-built boards failed it);
  * `perpendicular_*` / `user_facing` with `product.user_top_side` -> the
    face, graded as `edge_connector_side` at a fixed WARN that steers no
    search;
  * a cable keep-out ONLY from a declared `cable_envelope_mm`, and only for a
    FILE-locked part -- there is no default: none passed the controls.

The control that keeps this honest: the five as-built briefs
(`tests/fixtures/959/asbuilt/`) describe their boards AS THEY SHIP, so the
compiled clauses must add no ERROR to any of them.
"""
import json
import os
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
from placement import design_brief as db                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402

RUN_ALL_TIMEOUT = 2400

ASBUILT = os.path.join(REPO, 'tests', 'fixtures', '959', 'asbuilt')
ESP = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
BRIEF_711 = os.path.join(REPO, 'tests', 'fixtures', '711',
                         'esp_prog.design-brief.json')


def _brief(raw_or_path, **edits):
    if isinstance(raw_or_path, str):
        with open(raw_or_path, encoding='utf-8') as fh:
            raw = json.load(fh)
    else:
        raw = dict(raw_or_path)
    raw = json.loads(json.dumps(raw))
    raw.update(edits)
    return db.brief_from_dict(raw, '')


def _rows(frag_rep):
    return {r['id']: r for r in frag_rep[1]['consequences']}


def _grade(board, frag, rep, pcb=None):
    pcb = pcb or parse_kicad_pcb(board)
    doc = fp.emit_intent(pcb, board, declare_classes=True)
    doc = db.merge_into_intent(doc, frag, rep)
    return fp.grade(fp.intent_from_dict(doc, ''), pcb, board), doc


def test_each_declaration_compiles_or_says_why_not():
    """Acceptance 4: every connector declaration either compiles to a clause
    with a grader and a basis, or is reported unmeasured / withheld with the
    missing dimension named."""
    pcb = parse_kicad_pcb(ESP)
    rows = _rows(db.compile_with_consequences(_brief(BRIEF_711), pcb, ESP))
    usb = rows['interfaces[USB1].mount_mode']
    assert usb['status'] == 'compiled' and usb['basis'] == 'derived_default'
    assert usb['compiled_to'] == 'edge_connectors[USB1].max_setback_mm'
    assert usb['value'] == db.EDGE_MOUNT_SETBACK_MM == 0.75, usb
    con2 = rows['interfaces[CON2].mount_mode']
    assert con2['status'] == 'compiled' and con2['value'][
        'max_setback_mm'] == 0.75, con2
    side = rows['interfaces[CON2].cable_entry']
    assert side['grader'] == 'edge_connector_side' and side['value'] == 'F'
    assert rows['interfaces[USB1].user_facing']['value'] == 'F', rows
    for ref in ('USB1', 'CON2'):
        env = rows[f'interfaces[{ref}].cable_envelope_mm']
        assert env['status'] == 'unmeasured', env
        assert 'no default' in env['why'] and 'z-height' in env['why'], env
    print(f"  PASS: {len(rows)} consequence rows on fixture 711; edge_mount "
          f"and through_edge compile, the face compiles, the envelope is "
          f"unmeasured and says why")


def test_the_as_built_briefs_gain_no_error():
    """The control: five boards graded against briefs that describe them as
    they ship. Compiling the connector declarations may add WARNs; it must
    add no ERROR. (Before the evidence revision, the literal mapping's 0.5 mm
    seat failed tigard J7, and `user_facing` failed 8 vertical headers.)"""
    checked = 0
    for name in ('esp_prog', 'glasgow_revC', 'splitflap_driver', 'tigard',
                 'ulx3s'):
        board = os.path.join(REPO, 'kicad_files', f'{name}.kicad_pcb')
        bpath = os.path.join(ASBUILT, f'{name}.design-brief.json')
        run_utils.evidence(bpath, 'an as-built brief')
        pcb = parse_kicad_pcb(board)
        brief = db.load_brief(bpath)
        plain = db.compile_brief(brief, board_refs=sorted(pcb.footprints))
        full = db.compile_with_consequences(brief, pcb, board)
        before, _ = _grade(board, *plain, pcb=pcb)
        after, doc = _grade(board, *full, pcb=pcb)
        new = sorted({(v.rule, v.ref) for v in after.errors}
                     - {(v.rule, v.ref) for v in before.errors})
        assert not new, (name, new)
        assert any('max_setback_mm' in c or 'side' in c
                   for c in doc.get('edge_connectors') or ()), name
        checked += 1
    print(f"  PASS: {checked} as-built boards gain no ERROR from the compiled "
          f"connector clauses")


def test_a_vertical_mount_is_not_held_to_the_seat():
    """esp_prog's CON2 is a vertical header 0.69 mm in from its edge. Declared
    `user_facing` it was an edge receptacle held to the 0.5 mm seat, and
    failed on the shipping board; `top_mount` says what it is."""
    pcb = parse_kicad_pcb(ESP)
    rawb = {'schema': 1, 'kind': 'design-brief', 'units': 'mm',
            'board': 'esp_prog.kicad_pcb',
            'product': {'user_top_side': 'F'},
            'interfaces': [{'ref': 'CON2', 'edge': 'east',
                            'user_facing': True}]}
    seat = []
    for mm in (None, 'top_mount'):
        iface = dict(rawb['interfaces'][0])
        if mm:
            iface['mount_mode'] = mm
        frag, rep = db.compile_with_consequences(
            _brief(rawb, interfaces=[iface]), pcb, ESP)
        res, _doc = _grade(ESP, frag, rep, pcb=pcb)
        seat.append([v for v in res.errors if v.rule == 'edge_connector'
                     and v.ref == 'CON2' and 'seated' in v.message])
    assert seat[0] and not seat[1], seat
    print("  PASS: user_facing alone fails CON2's seat; top_mount does not")


def test_the_face_is_advisory_and_steers_nothing():
    """`edge_connector_side` is a fixed WARN: an intent cannot raise it, and
    seeding the same pile with and without `side` gives the same poses."""
    raw = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
           'edge_connectors': [{'ref': 'USB1', 'edge': 'east',
                                'side': 'B'}]}
    pcb = parse_kicad_pcb(ESP)
    res = fp.grade(fp.intent_from_dict(raw, ''), pcb, ESP)
    sv = [v for v in res.violations if v.rule == 'edge_connector_side']
    assert sv and sv[0].severity == 'warn', res.violations
    for sev in ('error',):
        try:
            fp.intent_from_dict(dict(raw, severity={
                'edge_connector_side': sev}), '')
        except fp.IntentError as exc:
            assert 'advisory by design' in str(exc), exc
        else:
            raise AssertionError('edge_connector_side raised to error')
    fp.intent_from_dict(dict(raw, severity={'edge_connector_side': 'warn'}),
                        '')
    pile = os.path.join(REPO, 'tests', 'fixtures', '959',
                        'run29_pile.kicad_pcb')
    with tempfile.TemporaryDirectory() as tmp:
        outs = []
        for tag, side in (('a', None), ('b', 'B')):
            ec = {'ref': 'USB1', 'edge': 'east'}
            if side:
                ec['side'] = side
            ip = os.path.join(tmp, f'{tag}.json')
            with open(ip, 'w', encoding='utf-8') as fh:
                json.dump({'schema': 1, 'kind': 'floorplan-intent',
                           'units': 'mm', 'edge_connectors': [ec]}, fh)
            out = os.path.join(tmp, f'{tag}.kicad_pcb')
            subprocess.run([sys.executable, '-X', 'utf8',
                            run_utils.tool('place_seed.py'), pile, out,
                            '--intent', ip, '--seed', '3'],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=REPO, timeout=900)
            run_utils.evidence(out, f'the {tag} seed')
            p_ = parse_kicad_pcb(out)
            outs.append({k: (round(f.x, 4), round(f.y, 4),
                             round(f.rotation or 0.0, 3))
                         for k, f in p_.footprints.items()})
        assert outs[0] == outs[1], [k for k in outs[0]
                                    if outs[0][k] != outs[1].get(k)]
    print("  PASS: the face is a WARN nothing can raise, and a `side` changes "
          "no seeded pose")


def _locked_copy(tmp, ref):
    from kicad_parser import iter_footprint_blocks
    text = open(ESP, encoding='utf-8').read()
    for start, end, _t, _r, key in iter_footprint_blocks(text):
        if key == ref:
            block = text[start:end]
            i = block.index(')', block.index('(at '))
            block = block[:i + 1] + ' (locked yes)' + block[i + 1:]
            text = text[:start] + block + text[end:]
            break
    out = os.path.join(tmp, 'esp_locked.kicad_pcb')
    with open(out, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return out


def test_a_cable_keepout_needs_a_declared_envelope_and_a_lock():
    rawb = {'schema': 1, 'kind': 'design-brief', 'units': 'mm',
            'board': 'esp_prog.kicad_pcb',
            'product': {'user_top_side': 'F'},
            'interfaces': [{'ref': 'CON2', 'edge': 'south',
                            'mount_mode': 'top_mount',
                            'cable_entry': 'perpendicular_top',
                            'cable_envelope_mm': {'clear': 1.5}}]}
    pcb = parse_kicad_pcb(ESP)
    frag, rep = db.compile_with_consequences(_brief(rawb), pcb, ESP)
    env = _rows((frag, rep))['interfaces[CON2].cable_envelope_mm']
    assert env['status'] == 'withheld' and 'lock CON2' in env['why'], env
    assert not frag.get('keepouts'), frag.get('keepouts')
    with tempfile.TemporaryDirectory() as tmp:
        lb = _locked_copy(tmp, 'CON2')
        lpcb = parse_kicad_pcb(lb)
        frag, rep = db.compile_with_consequences(_brief(rawb), lpcb, lb)
        env = _rows((frag, rep))['interfaces[CON2].cable_envelope_mm']
        assert env['status'] == 'compiled' and env['basis'] == 'declared'
        k = [x for x in frag['keepouts'] if x['name'] == 'cable:CON2'][0]
        assert k['sides'] == ['F'] and k['allow'] == ['CON2'], k
        # A stranger sitting in the envelope is a keepout finding.
        res, _doc = _grade(lb, frag, rep, pcb=lpcb)
        hits = {v.ref for v in res.violations if v.rule == 'keepout'
                and v.block == 'cable:CON2' or (v.rule == 'keepout'
                                                and 'cable:CON2'
                                                in v.message)}
        assert 'CON2' not in hits, hits
    for env_v, want in (('unknown', 'declared "unknown"'),
                        (None, 'no default is used')):
        iface = dict(rawb['interfaces'][0])
        if env_v is None:
            iface.pop('cable_envelope_mm')
        else:
            iface['cable_envelope_mm'] = env_v
        rows = _rows(db.compile_with_consequences(
            _brief(rawb, interfaces=[iface]), pcb, ESP))
        row = rows['interfaces[CON2].cable_envelope_mm']
        assert row['status'] == 'unmeasured' and want in row['why'], row
    for bad in ({'clear': 0}, {'depth': -1}, {'reach': 2}):
        iface = dict(rawb['interfaces'][0], cable_envelope_mm=bad)
        try:
            _brief(rawb, interfaces=[iface])
        except Exception as exc:                            # noqa: BLE001
            assert 'cable_envelope_mm' in str(exc), exc
        else:
            raise AssertionError(f'accepted {bad!r}')
    print("  PASS: unlocked -> withheld; locked -> a declared keep-out on the "
          "top face; unknown / absent -> unmeasured; bad envelopes refused")


def test_changing_a_carried_field_now_drifts():
    """The #959 comment's `carried_changed` control: an intent emitted with
    fixture 711, then graded against a brief whose `user_top_side`,
    `mount_mode` or `cable_entry` changed. Before this, nothing drifted."""
    pcb = parse_kicad_pcb(ESP)
    base = db.compile_with_consequences(_brief(BRIEF_711), pcb, ESP)
    doc = db.merge_into_intent(fp.emit_intent(pcb, ESP, declare_classes=True),
                               *base)
    with open(BRIEF_711, encoding='utf-8') as fh:
        raw = json.load(fh)

    def drifted(mutate):
        r2 = json.loads(json.dumps(raw))
        mutate(r2)
        frag, _rep = db.compile_with_consequences(
            db.brief_from_dict(r2, ''), pcb, ESP)
        return set(db.drifted_clause_ids(doc, frag))
    assert not drifted(lambda r: None), drifted(lambda r: None)
    top = drifted(lambda r: r['product'].__setitem__('user_top_side', 'B'))
    assert {'interfaces[USB1].user_facing',
            'interfaces[CON2].cable_entry'} <= top, top

    def mm(r):
        r['interfaces'][0]['mount_mode'] = 'top_mount'
    assert 'interfaces[USB1].mount_mode' in drifted(mm), drifted(mm)

    def ce(r):
        r['interfaces'][1]['cable_entry'] = 'perpendicular_bottom'
    assert 'interfaces[CON2].cable_entry' in drifted(ce), drifted(ce)
    print("  PASS: user_top_side, mount_mode and cable_entry each drift on "
          "their own clause id")


def test_the_grade_path_derives_what_the_emit_path_did():
    """The consequences are compiled BEFORE the emit/grade branch, so an
    intent emitted with a brief grades against that brief with no drift."""
    with tempfile.TemporaryDirectory() as tmp:
        b = os.path.join(tmp, 'board.kicad_pcb')
        import shutil
        shutil.copy(ESP, b)
        shutil.copy(BRIEF_711, os.path.join(tmp, 'board.design-brief.json'))
        out = os.path.join(tmp, 'i.json')
        run_utils.check([sys.executable, '-X', 'utf8',
                         run_utils.tool('check_floorplan.py'), b,
                         '--emit-intent', out, '--declare-classes',
                         '--no-mechanical'], accept=True)
        doc = json.load(open(out, encoding='utf-8'))
        usb = [c for c in doc['edge_connectors'] if c['ref'] == 'USB1'][0]
        assert usb['max_setback_mm'] == 0.75 and usb['side'] == 'F', usb
        assert doc['min_reader'] >= 6, doc.get('min_reader')
        js = os.path.join(tmp, 'g.json')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            run_utils.tool('check_floorplan.py'), b,
                            '--intent', out, '--json', js,
                            '--no-mechanical'],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=REPO, timeout=900)
        line = [x for x in r.stdout.splitlines()
                if x.startswith('JSON_SUMMARY:')][-1]
        s = json.loads(line.split('JSON_SUMMARY: ', 1)[1])
        assert s.get('brief_drift', 0) == 0, s
        assert 'derived:interfaces[USB1].mount_mode' in s[
            'derived_default_clauses'], s['derived_default_clauses']
        ledger = json.load(open(js, encoding='utf-8'))['declaration_ledger']
        kinds = {x['kind'] for x in ledger}
        assert 'derived_clause' in kinds, kinds
        assert any(x['status'] == 'unmeasured' for x in ledger), ledger
    print("  PASS: emit and grade derive the same clauses (no drift); the "
          "ledger lists derived_default clauses and the unmeasured envelope")


TESTS = [
    test_each_declaration_compiles_or_says_why_not,
    test_the_as_built_briefs_gain_no_error,
    test_a_vertical_mount_is_not_held_to_the_seat,
    test_the_face_is_advisory_and_steers_nothing,
    test_a_cable_keepout_needs_a_declared_envelope_and_a_lock,
    test_changing_a_carried_field_now_drifts,
    test_the_grade_path_derives_what_the_emit_path_did,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
