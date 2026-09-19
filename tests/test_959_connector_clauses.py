#!/usr/bin/env python3
"""#959 / #1000: connector declarations compile to clauses a rule grades.

Run 29's brief declared `mount_mode`, `cable_entry` and `user_top_side`, and
the #959 comment measured that changing any of them changed NO measurement:
all three were carried into `context` and graded by nothing. This compiles
them -- by the evidence the Phase-0 control measured on five as-built boards,
not by the issue's literal mapping:

  * `edge_mount` -> the drawn body within 0.75 mm of its edge (two shipping
    edge-mount bodies sit 0.60 and 0.614 mm in);
  * `through_edge` -> the same setback, on the courtyard unless the entry is
    a user-facing receptacle;
  * `top_mount` / `bottom_mount` -> NOT held to the receptacle seat (8
    vertical headers on the as-built boards failed it);
  * `perpendicular_*` with `product.user_top_side` -> the face the cable
    plugs into, graded as `edge_connector_side` at a fixed WARN that steers
    no search (`user_facing` names no face);
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


def _ledger(board, frag, rep, pcb=None):
    """Grade, then build coverage and the ledger the way check_floorplan
    does. `(coverage, {ledger id: row}, doc)`."""
    res, doc = _grade(board, frag, rep, pcb=pcb)
    cov = db.clause_coverage(rep, doc, rules_run=res.rules_run,
                             abstained=res.budget_abstained,
                             drifted_ids=db.drifted_clause_ids(doc, frag))
    led = fp.declaration_ledger(res.intent, res.roster, result=res,
                                coverage=cov, brief_source='b.json',
                                consequences=rep.get('consequences'))
    return cov, {r['id']: r for r in led}, doc


def test_each_declaration_compiles_or_says_why_not():
    """Acceptance 4: every connector declaration either compiles to a clause
    with a grader and a basis, or is reported unmeasured / withheld with the
    missing dimension named."""
    pcb = parse_kicad_pcb(ESP)
    frag, rep = db.compile_with_consequences(_brief(BRIEF_711), pcb, ESP)
    rows = _rows((frag, rep))
    usb = rows['interfaces[USB1].mount_mode']
    assert usb['status'] == 'compiled' and usb['basis'] == 'derived_default'
    assert usb['compiled_to'] == 'edge_connectors[USB1].max_setback_mm'
    assert usb['value'] == db.EDGE_MOUNT_SETBACK_MM == 0.75, usb
    con2 = rows['interfaces[CON2].mount_mode']
    assert con2['status'] == 'compiled' and con2['value'] == 0.75, con2
    side = rows['interfaces[CON2].cable_entry']
    assert side['grader'] == 'edge_connector_side' and side['value'] == 'F'
    # `user_facing` compiles no face (it put B-side connectors on F).
    assert 'interfaces[USB1].user_facing' not in rows, rows
    assert 'side' not in [c for c in frag['edge_connectors']
                          if c['ref'] == 'USB1'][0]
    # in_plane with a declared edge restates that edge: carried, not a
    # second clause; without one it is unmeasured.
    assert rows['interfaces[USB1].cable_entry']['status'] == 'carried'
    for ref in ('USB1', 'CON2'):
        env = rows[f'interfaces[{ref}].cable_envelope_mm']
        assert env['status'] == 'unmeasured', env
        assert 'no default' in env['why'] and 'z-height' in env['why'], env
    # What compiled leaves `not_graded`; the viewing face is used by CON2's
    # perpendicular cable, so it leaves too. What did not compile stays.
    ng = set(rep['not_graded'])
    assert not {'interfaces[USB1].mount_mode', 'interfaces[CON2].mount_mode',
                'interfaces[CON2].cable_entry',
                'product.user_top_side'} & ng, ng
    assert 'interfaces[USB1].cable_entry' in ng, ng
    with open(BRIEF_711, encoding='utf-8') as fh:
        raw = json.load(fh)
    raw['interfaces'][0]['edge'] = 'unknown'
    raw['interfaces'][0].pop('along_edge', None)
    raw['interfaces'][0].pop('along_edge_tolerance_mm', None)
    r2 = _rows(db.compile_with_consequences(db.brief_from_dict(raw, ''),
                                            pcb, ESP))
    assert r2['interfaces[USB1].cable_entry']['status'] == 'unmeasured', r2
    print(f"  PASS: {len(rows)} consequence rows on fixture 711; edge_mount "
          f"and through_edge compile, a perpendicular cable compiles the "
          f"face and user_facing does not, in_plane is carried, the envelope "
          f"is unmeasured and says why")


def test_through_edge_keeps_the_emitted_overhang_cap():
    """Phase-5 verifier B1: through_edge wrote `overhang_mm {min: 0}`, the
    merge replaced the emitted `{min 0, max 2.0}` wholesale, and a part with
    no `max` loses its off-outline exemption -- a through-edge connector
    hanging correctly past the edge graded as an off-board part."""
    pile = os.path.join(REPO, 'tests', 'fixtures', '959',
                        'run29_pile.kicad_pcb')
    ppcb = parse_kicad_pcb(pile)
    rawb = {'schema': 1, 'kind': 'design-brief', 'units': 'mm',
            'board': 'esp_prog.kicad_pcb', 'product': {'user_top_side': 'F'},
            'interfaces': [{'ref': 'USB1', 'edge': 'west',
                            'user_facing': True,
                            'mount_mode': 'through_edge'}]}
    frag, rep = db.compile_with_consequences(_brief(rawb), ppcb, pile)
    emitted = fp.emit_intent(ppcb, pile, declare_classes=True)
    em = [c for c in emitted['edge_connectors'] if c['ref'] == 'USB1'][0]
    assert (em.get('overhang_mm') or {}).get('max') is not None, em
    merged = db.merge_into_intent(emitted, frag, rep)
    mu = [c for c in merged['edge_connectors'] if c['ref'] == 'USB1'][0]
    assert mu['overhang_mm'] == em['overhang_mm'], (em, mu)
    assert mu['max_setback_mm'] == 0.75, mu
    fu = [c for c in frag['edge_connectors'] if c['ref'] == 'USB1'][0]
    assert 'overhang_mm' not in fu, fu
    print(f"  PASS: through_edge compiles the setback and leaves the emitted "
          f"overhang {em['overhang_mm']} -- and its exemption -- intact")


def test_the_as_built_briefs_gain_no_error():
    """The control: five boards graded against briefs that describe them as
    they ship. Compiling the connector declarations may add WARNs; it must
    add no ERROR. (Before the evidence revision, `user_facing` held 8
    vertical headers to the receptacle seat and failed them. tigard J7's body
    sits 0.60 mm in, past the literal mapping's 0.5 mm -- but its courtyard
    reaches the edge, so the seat does not bind on it as built.)"""
    checked = 0
    worst = (0.0, '')
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
        # ABSOLUTE, not only a delta (verifier S7): every part declared
        # edge- or through-mounted must SEAT on its shipping board...
        mounted = {i['ref'] for i in brief.interfaces
                   if i.get('mount_mode') in ('edge_mount', 'through_edge')}
        unseated = sorted({v.ref for v in after.errors
                           if v.rule == 'edge_connector' and v.ref in mounted
                           and 'seated' in v.message})
        assert not unseated, (name, unseated)
        # ...and the default must ADMIT every such body the grade measures.
        # The seat binds only where the courtyard sits a margin inside the
        # board, and on the as-built boards it does not bind on either body
        # that lies past 0.5 mm (tigard J7 0.60, a courtyard reaching the
        # edge) -- so "seats" alone could not see the setback shrink
        # (Phase-7 fact-check). The body setback is measured either way.
        for e in after.edge_seating or ():
            if e.get('ref') in mounted and e.get('body_setback_mm') is not None:
                worst = max(worst, (e['body_setback_mm'], f"{name} {e['ref']}"))
        checked += 1
    assert worst[0] <= db.EDGE_MOUNT_SETBACK_MM, worst
    assert worst[0] > 0.5, (
        f"the widest as-built body setback is {worst}: the 0.75 default has "
        f"no as-built body past 0.5 mm left to admit -- re-derive it")
    print(f"  PASS: {checked} as-built boards gain no ERROR from the compiled "
          f"connector clauses, every edge/through-mounted part seats, and "
          f"the {db.EDGE_MOUNT_SETBACK_MM} mm default admits the widest "
          f"measured body ({worst[1]}, {worst[0]} mm)")


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
    for mm in (None, 'top_mount', 'bottom_mount'):
        iface = dict(rawb['interfaces'][0])
        if mm:
            iface['mount_mode'] = mm
        frag, rep = db.compile_with_consequences(
            _brief(rawb, interfaces=[iface]), pcb, ESP)
        res, _doc = _grade(ESP, frag, rep, pcb=pcb)
        seat.append([v for v in res.errors if v.rule == 'edge_connector'
                     and v.ref == 'CON2' and 'seated' in v.message])
        if mm:
            # An exemption is REPORTED, as carried: it grades nothing of its
            # own, so it is neither a pass nor a fail.
            row = _rows((frag, rep))['interfaces[CON2].mount_mode']
            assert row['status'] == 'carried' and \
                'does not apply' in row['why'], row
            cov, led, _d = _ledger(ESP, frag, rep, pcb=pcb)
            st = {c['id']: c['state'] for c in cov['clauses']}
            assert st['interfaces[CON2].mount_mode'] == 'carried', st
            # What grades `user_facing` is the seat it is exempt from.
            assert st['interfaces[CON2].user_facing'] == 'carried', st
            assert led['derived:interfaces[CON2].mount_mode'][
                'status'] == 'carried', led
            # And no face: user_facing names none (verifier S3).
            assert 'side' not in [c for c in frag['edge_connectors']
                                  if c['ref'] == 'CON2'][0]
    assert seat[0] and not seat[1] and not seat[2], seat
    print("  PASS: user_facing alone fails CON2's seat; top_mount and "
          "bottom_mount do not, are reported carried, and compile no face")


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
    frag_u, rep_u = frag, rep
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
                and (v.measured or {}).get('keepout') == 'cable:CON2'}
        assert 'CON2' not in hits, hits
        # The ledger judges the clause by the keep-out's NAME -- the finding
        # names the intruder, never CON2 (verifier S1). A clear wide enough
        # to take in a neighbour fails it; a tiny one passes.
        for clear, want in ((6.0, 'graded_fail'), (0.05, 'graded_pass')):
            iface = dict(rawb['interfaces'][0],
                         cable_envelope_mm={'clear': clear})
            f2, r2 = db.compile_with_consequences(
                _brief(rawb, interfaces=[iface]), lpcb, lb)
            cov, led, _d = _ledger(lb, f2, r2, pcb=lpcb)
            got = (led['derived:interfaces[CON2].cable_envelope_mm'][
                'status'],
                led['interfaces[CON2].cable_envelope_mm']['status'])
            assert got == (want, want), (clear, got)
        # A keep-out the brief declares under the same name WINS: the
        # envelope is graded through it, never replaces it (verifier S4).
        mine = {'name': 'cable:CON2', 'rect': [100.0, 100.0, 101.0, 101.0],
                'sides': ['F', 'B']}
        f3, r3 = db.compile_with_consequences(
            _brief(rawb, keepouts=[mine]), lpcb, lb)
        k = [x for x in f3['keepouts'] if x['name'] == 'cable:CON2']
        assert len(k) == 1 and k[0]['rect'] == mine['rect'], k
        # CARRIED: the declared keep-out is its own clause, and the
        # envelope's dimension grades nothing (round-2 verifier).
        row = _rows((f3, r3))['interfaces[CON2].cable_envelope_mm']
        assert row['status'] == 'carried' and 'declares' in row['why'], row
        # ...unlocked too: a declared keep-out needs no lock to win.
        f3u, r3u = db.compile_with_consequences(
            _brief(rawb, keepouts=[mine]), pcb, ESP)
        assert _rows((f3u, r3u))['interfaces[CON2].cable_envelope_mm'][
            'status'] == 'carried'
        assert [x['rect'] for x in f3u['keepouts']
                if x['name'] == 'cable:CON2'] == [mine['rect']], f3u
        # A derived keep-out the brief stops deriving drifts, and so does
        # one whose envelope changed shape (verifier S5, M15).
        base_doc = db.merge_into_intent(
            fp.emit_intent(lpcb, lb, declare_classes=True), frag, rep)
        for env2 in (None, 'unknown', {'clear': 2.5}):
            iface = dict(rawb['interfaces'][0])
            if env2 is None:
                iface.pop('cable_envelope_mm')
            else:
                iface['cable_envelope_mm'] = env2
            f4, r4 = db.compile_with_consequences(
                _brief(rawb, interfaces=[iface]), lpcb, lb)
            ids = db.drifted_clause_ids(base_doc, f4)
            assert 'interfaces[CON2].cable_envelope_mm' in ids, (env2, ids)
            # ...and the drift REACHES coverage, which P1 and P-close read:
            # removed or "unknown", the clause has no declared row of its
            # own, and without one no gate refused (round-2 verifier).
            cov = db.clause_coverage(r4, base_doc, rules_run=('keepout',),
                                     drifted_ids=ids)
            row = [c for c in cov['clauses']
                   if c['id'] == 'interfaces[CON2].cable_envelope_mm']
            assert row and row[0]['drifted'] and not cov['complete'], (
                env2, row, cov['drifted'])
    # Unlocked, the envelope is WITHHELD: an abstention, so coverage is not
    # complete and the ledger agrees on both rows (verifier S6).
    cov, led, _d = _ledger(ESP, frag_u, rep_u, pcb=pcb)
    st = {c['id']: c['state'] for c in cov['clauses']}
    assert st['interfaces[CON2].cable_envelope_mm'] == 'abstained', st
    assert not cov['complete'], cov
    assert led['interfaces[CON2].cable_envelope_mm']['status'] == \
        led['derived:interfaces[CON2].cable_envelope_mm']['status'] == \
        'abstained', led
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
    # The viewing face reaches the one part whose cable is perpendicular.
    top = drifted(lambda r: r['product'].__setitem__('user_top_side', 'B'))
    assert top == {'interfaces[CON2].cable_entry'}, top

    # A compiled key the brief no longer produces (edge_mount -> nothing):
    # USB1's setback is stale. Neither side is a vertical mount, so this is
    # the stale-key check alone (M13).
    def mm_gone(r):
        r['interfaces'][0].pop('mount_mode')
    assert 'interfaces[USB1].mount_mode' in drifted(mm_gone), drifted(mm_gone)

    def ce(r):
        r['interfaces'][1]['cable_entry'] = 'perpendicular_bottom'
    assert 'interfaces[CON2].cable_entry' in drifted(ce), drifted(ce)

    # A vertical mount compiles NO key, so only the mount comparison can see
    # it change (M14): an intent built with CON2 top_mount, graded against a
    # brief that no longer says so.
    r_v = json.loads(json.dumps(raw))
    r_v['interfaces'][1]['mount_mode'] = 'top_mount'
    base_v = db.compile_with_consequences(db.brief_from_dict(r_v, ''),
                                          pcb, ESP)
    doc_v = db.merge_into_intent(
        fp.emit_intent(pcb, ESP, declare_classes=True), *base_v)
    r_n = json.loads(json.dumps(r_v))
    r_n['interfaces'][1].pop('mount_mode')
    frag_n, _ = db.compile_with_consequences(db.brief_from_dict(r_n, ''),
                                             pcb, ESP)
    ids = set(db.drifted_clause_ids(doc_v, frag_n))
    assert ids == {'interfaces[CON2].mount_mode'}, ids
    print("  PASS: user_top_side, a dropped mount_mode, a vertical mount and "
          "cable_entry each drift on their own clause id")


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
        con2 = [c for c in doc['edge_connectors'] if c['ref'] == 'CON2'][0]
        assert usb['max_setback_mm'] == 0.75 and con2['side'] == 'F', (
            usb, con2)
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


def test_the_ledger_reads_the_face_and_the_viewing_side_honestly():
    """A fired side WARN is not a pass (verifier S1); the viewing face is
    GRADED once a perpendicular cable turns it into a side, and leaves the
    carried list (S8); an unrelated edge error does not fail a clause whose
    grader is the side finding."""
    pcb = parse_kicad_pcb(ESP)
    rawb = {'schema': 1, 'kind': 'design-brief', 'units': 'mm',
            'board': 'esp_prog.kicad_pcb', 'product': {'user_top_side': 'F'},
            'interfaces': [{'ref': 'CON2', 'edge': 'west',
                            'cable_entry': 'perpendicular_bottom'}]}
    frag, rep = db.compile_with_consequences(_brief(rawb), pcb, ESP)
    cov, led, _doc = _ledger(ESP, frag, rep, pcb=pcb)
    # CON2 is on F; the bottom cable puts it on B -> the WARN fires. CON2 is
    # also off its declared west edge -> an edge_connector ERROR on CON2,
    # which must not decide the side clause's verdict.
    assert led['derived:interfaces[CON2].cable_entry']['status'] == \
        'graded_warn', led['derived:interfaces[CON2].cable_entry']
    assert led['interfaces[CON2].cable_entry']['status'] == 'graded_warn'
    st = {c['id']: c['state'] for c in cov['clauses']}
    assert st['product.user_top_side'] == 'graded', st
    assert led['product.user_top_side']['status'] == 'graded_warn', led[
        'product.user_top_side']
    assert 'product.user_top_side' not in fp.ledger_summary(
        list(led.values()))['carried_facts']
    rawb['interfaces'][0]['cable_entry'] = 'perpendicular_top'
    frag, rep = db.compile_with_consequences(_brief(rawb), pcb, ESP)
    cov, led, _doc = _ledger(ESP, frag, rep, pcb=pcb)
    assert led['derived:interfaces[CON2].cable_entry']['status'] == \
        'graded_pass', led['derived:interfaces[CON2].cable_entry']
    # A default dimension is this code's number, not the author's.
    rawb['interfaces'][0]['mount_mode'] = 'edge_mount'
    frag, rep = db.compile_with_consequences(_brief(rawb), pcb, ESP)
    _cov, led, _doc = _ledger(ESP, frag, rep, pcb=pcb)
    assert led['derived:interfaces[CON2].mount_mode']['authority'] == \
        'assumption', led['derived:interfaces[CON2].mount_mode']
    print("  PASS: a fired side WARN reads graded_warn on both rows, the "
          "viewing face is graded and not carried, a default is an "
          "assumption")


def test_an_in_plane_band_sits_on_the_declared_edge_or_not_at_all():
    """The band runs in from the DECLARED edge (M21), and only when the
    locked part reaches it: fixture 711 declares USB1 east while it ships
    flush west, where an east band would flag only bystanders (N8)."""
    with tempfile.TemporaryDirectory() as tmp:
        lb = _locked_copy(tmp, 'USB1')
        lpcb = parse_kicad_pcb(lb)
        bounds = lpcb.board_info.board_bounds
        rawb = {'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                'board': 'esp_prog.kicad_pcb',
                'interfaces': [{'ref': 'USB1', 'edge': 'west',
                                'cable_entry': 'in_plane',
                                'cable_envelope_mm': {'depth': 2.0}}]}
        frag, rep = db.compile_with_consequences(_brief(rawb), lpcb, lb)
        k = [x for x in frag['keepouts'] if x['name'] == 'cable:USB1'][0]
        assert abs(k['rect'][0] - bounds[0]) < 1e-3 and abs(
            k['rect'][2] - (bounds[0] + 2.0)) < 1e-3, (k, bounds)
        rawb['interfaces'][0]['edge'] = 'east'
        frag, rep = db.compile_with_consequences(_brief(rawb), lpcb, lb)
        row = _rows((frag, rep))['interfaces[USB1].cable_envelope_mm']
        assert row['status'] == 'withheld' and 'declared east edge' in row[
            'why'], row
        assert not any(x['name'] == 'cable:USB1'
                       for x in frag.get('keepouts') or ()), frag
        # The same for north / south: CON2 ships on the north edge.
        lc = _locked_copy(tmp, 'CON2')
        cpcb = parse_kicad_pcb(lc)
        cb = cpcb.board_info.board_bounds
        rawc = {'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                'board': 'esp_prog.kicad_pcb',
                'interfaces': [{'ref': 'CON2', 'edge': 'north',
                                'cable_entry': 'in_plane',
                                'cable_envelope_mm': {'depth': 2.5}}]}
        frag, rep = db.compile_with_consequences(_brief(rawc), cpcb, lc)
        k = [x for x in frag['keepouts'] if x['name'] == 'cable:CON2'][0]
        assert abs(k['rect'][1] - cb[1]) < 1e-3 and abs(
            k['rect'][3] - (cb[1] + 2.5)) < 1e-3, (k, cb)
        rawc['interfaces'][0]['edge'] = 'south'
        frag, rep = db.compile_with_consequences(_brief(rawc), cpcb, lc)
        assert _rows((frag, rep))['interfaces[CON2].cable_envelope_mm'][
            'status'] == 'withheld'
    print("  PASS: the in-plane band runs in from the declared edge (west, "
          "north), and is withheld when the part does not reach it (east, "
          "south)")


def test_a_missing_part_fails_its_clauses_and_a_stale_overhang_drifts():
    """A connector not on the board fails every clause about it, the side
    clause included -- the side finding only WARNs, so it read a missing part
    as a pass. And an intent written before through_edge stopped deriving an
    overhang floor carries `{min: 0}` with no `max`, which drifts (round-2
    verifier)."""
    pcb = parse_kicad_pcb(ESP)
    rawb = {'schema': 1, 'kind': 'design-brief', 'units': 'mm',
            'board': 'esp_prog.kicad_pcb', 'product': {'user_top_side': 'F'},
            'interfaces': [{'ref': 'CON2', 'edge': 'north',
                            'cable_entry': 'perpendicular_bottom'}]}
    frag, rep = db.compile_with_consequences(_brief(rawb), pcb, ESP)
    doc = db.merge_into_intent(fp.emit_intent(pcb, ESP, declare_classes=True),
                               frag, rep)
    from kicad_parser import iter_footprint_blocks
    text = open(ESP, encoding='utf-8').read()
    with tempfile.TemporaryDirectory() as tmp:
        for start, end, _t, _r, key in iter_footprint_blocks(text):
            if key == 'CON2':
                text = text[:start] + text[end:]
                break
        gone = os.path.join(tmp, 'no_con2.kicad_pcb')
        with open(gone, 'w', encoding='utf-8') as fh:
            fh.write(text)
        gpcb = parse_kicad_pcb(gone)
        assert 'CON2' not in gpcb.footprints
        res = fp.grade(fp.intent_from_dict(doc, ''), gpcb, gone)
        cov = db.clause_coverage(rep, doc, rules_run=res.rules_run)
        led = {r['id']: r for r in fp.declaration_ledger(
            res.intent, res.roster, result=res, coverage=cov,
            consequences=rep['consequences'])}
    assert led['derived:interfaces[CON2].cable_entry']['status'] == \
        'graded_fail', led['derived:interfaces[CON2].cable_entry']
    assert led['interfaces[CON2].cable_entry']['status'] == 'graded_fail'
    # The stale overhang floor.
    stale = json.loads(json.dumps(doc))
    for c in stale['edge_connectors']:
        if c['ref'] == 'CON2':
            c['overhang_mm'] = {'min': 0.0}
            c.setdefault('context', {}).setdefault(
                'compiled_from', {})['overhang_mm'] = 'mount_mode'
    rawb['interfaces'][0]['mount_mode'] = 'through_edge'
    f2, _r2 = db.compile_with_consequences(_brief(rawb), pcb, ESP)
    ids = db.drifted_clause_ids(stale, f2)
    assert 'interfaces[CON2].mount_mode' in ids, ids
    print("  PASS: a missing connector fails its side clause; a stale "
          "derived overhang floor drifts")


def test_an_envelope_needs_a_cable():
    """M33: an envelope on an interface that declares no cable is refused."""
    for ce in (None, 'none', 'unknown'):
        iface = {'ref': 'USB1', 'edge': 'east',
                 'cable_envelope_mm': {'depth': 2.0}}
        if ce is not None:
            iface['cable_entry'] = ce
        try:
            _brief({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                    'board': 'x', 'interfaces': [iface]})
        except Exception as exc:                            # noqa: BLE001
            assert 'no cable_entry' in str(exc), exc
        else:
            raise AssertionError(f'accepted an envelope with cable {ce!r}')
    print("  PASS: an envelope with no cable to apply to is refused")


def test_coverage_is_graded_only_when_the_intent_carries_the_clause():
    """M16/M17: a compiled consequence is `graded` when the intent carries
    what it compiled to, and `uncovered` when a hand edit dropped it."""
    pcb = parse_kicad_pcb(ESP)
    frag, rep = db.compile_with_consequences(_brief(BRIEF_711), pcb, ESP)
    cov, _led, doc = _ledger(ESP, frag, rep, pcb=pcb)
    st = {c['id']: c['state'] for c in cov['clauses']}
    assert st['interfaces[USB1].mount_mode'] == 'graded', st
    doc2 = json.loads(json.dumps(doc))
    for c in doc2['edge_connectors']:
        if c['ref'] == 'USB1':
            c.pop('max_setback_mm')
    cov2 = db.clause_coverage(rep, doc2, rules_run=('edge_connector',))
    st2 = {c['id']: c['state'] for c in cov2['clauses']}
    assert st2['interfaces[USB1].mount_mode'] == 'uncovered', st2
    print("  PASS: graded when carried, uncovered when the intent drops it")


TESTS = [
    test_each_declaration_compiles_or_says_why_not,
    test_through_edge_keeps_the_emitted_overhang_cap,
    test_the_ledger_reads_the_face_and_the_viewing_side_honestly,
    test_an_in_plane_band_sits_on_the_declared_edge_or_not_at_all,
    test_a_missing_part_fails_its_clauses_and_a_stale_overhang_drifts,
    test_an_envelope_needs_a_cable,
    test_coverage_is_graded_only_when_the_intent_carries_the_clause,
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
