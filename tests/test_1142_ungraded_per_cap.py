#!/usr/bin/env python3
"""#1142: `decap_ungraded` is promoted PER CAP under a --decaps-from intent.

`decap_distance` grades only the caps within the 5 mm tether search radius of
their chip; a cap beyond it is `decap_ungraded`, a WARN. #1102 promoted the
rule to ERROR board-wide, and only when the reference kept EVERY rail cap
inside the radius, which none of the seven #1105 references does -- so a
decoupler the reference keeps 1 mm from its IC and a seed strands 15 mm away
read as a WARN.

Now `emit_intent(decaps_from=REF)` lists the caps REF keeps within the radius
(same ref, same footprint on the emitted board) in `decaps.within_radius_refs`
with the radius they were read at (`decaps.within_radius_mm`), and
`rule_decap_ungraded` grades a LISTED cap left beyond the radius at ERROR and
every other one at WARN. An explicit `severity.decap_ungraded` still wins in
both directions.

Cases:
* esp_prog with C2 (held) moved 20 mm away: C2 is an ERROR, C1 (the cap the
  reference itself keeps beyond) is a WARN; an unlisted cap is a WARN;
* an explicit warn turns the held cap down; an old-shape (#1102) intent with
  `severity.decap_ungraded: error` and no list grades every beyond cap ERROR;
* `_gating` follows the same precedence;
* loader refusals, each for its stated reason; a reader-7 build refuses the
  emitted intent;
* the emitter: esp_prog's list is C2, C3, C4 (C1 excluded), there is no
  top-level severity, `min_reader` is 8, the basis names the reference, and
  a cap whose footprint differs from the reference's is not held;
* the CLI fixed point: esp_prog graded on its own intent exits 0;
* the issue's splitflap pile: the seed's stranded held caps are ERRORs, and
  C3 (beyond on the reference) stays a WARN.

    python3 -X utf8 tests/test_1142_ungraded_per_cap.py [case ...]
"""
import contextlib
import copy
import io
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 1200

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FLOORPLAN = os.path.join(ROOT, 'py_tools', 'check_floorplan.py')
_TMP = tempfile.TemporaryDirectory(prefix='t1142_')


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _parse(path):
    from kicad_parser import parse_kicad_pcb
    return _quiet(parse_kicad_pcb, path)


_DOC = {}


def _esp_doc():
    """esp_prog's own --decaps-from intent, emitted once."""
    if 'esp' not in _DOC:
        from placement import floorplan as fp
        _DOC['esp'] = _quiet(fp.emit_intent, _parse(ESP), ESP,
                             decaps_from=ESP)
    return copy.deepcopy(_DOC['esp'])


def _moved_esp(ref='C2', dx=20.0):
    """esp_prog with `ref` moved `dx` mm: a held cap stranded."""
    from placement.writer import write_placed_output
    pcb = _parse(ESP)
    fp = pcb.footprints[ref]
    out = os.path.join(tempfile.mkdtemp(dir=_TMP.name), 'moved.kicad_pcb')
    _quiet(write_placed_output, ESP, out,
           [{'reference': ref, 'new_x': fp.x + dx, 'new_y': fp.y,
             'new_rotation': fp.rotation or 0.0}])
    return out


def _grade_doc(doc, board):
    from placement import floorplan as fp
    intent = fp.intent_from_dict(doc)
    return _quiet(fp.grade, intent, _parse(board), board)


def _sev(g, ref):
    got = [v.severity for v in list(g.errors) + list(g.warnings)
           if v.rule == 'decap_ungraded' and v.ref == ref]
    assert len(got) == 1, (ref, got)
    return got[0]


def test_esp_prog_emits_the_held_list_not_a_severity():
    doc = _esp_doc()
    assert doc['decaps']['within_radius_refs'] == ['C2', 'C3', 'C4'], (
        doc['decaps'])
    assert doc['decaps']['within_radius_mm'] == 5.0, doc['decaps']
    assert 'decap_ungraded' not in (doc.get('severity') or {}), doc.get(
        'severity')
    assert doc['min_reader'] == 8, doc.get('min_reader')
    basis = doc['context']['basis']
    assert basis['decaps.within_radius_refs'] == 'reference:esp_prog.kicad_pcb'
    cen = doc['context']['decap_census']
    assert cen['reference_beyond_radius_refs'] == ['C1'], cen
    print(f"  PASS: held {doc['decaps']['within_radius_refs']}, C1 (beyond "
          f"on the reference) not held, min_reader 8, no severity key")


def test_a_held_cap_stranded_is_an_error_and_a_reference_bulk_cap_a_warn():
    board = _moved_esp('C2')
    g = _grade_doc(_esp_doc(), board)
    assert _sev(g, 'C2') == 'error'
    assert _sev(g, 'C1') == 'warn'
    du = [v for v in g.errors if v.rule == 'decap_ungraded']
    assert [v.ref for v in du] == ['C2'], [v.ref for v in du]
    assert du[0].measured['held_by_reference'] is True, du[0].measured
    assert 'stranded decoupler' in du[0].message, du[0].message
    print("  PASS: stranded C2 ERROR (held), C1 WARN (beyond on the "
          "reference)")


def test_an_unlisted_cap_is_a_warn():
    doc = _esp_doc()
    doc['decaps']['within_radius_refs'] = ['C3', 'C4']
    g = _grade_doc(doc, _moved_esp('C2'))
    assert _sev(g, 'C2') == 'warn'
    print("  PASS: C2 not on the list -> WARN")


def test_an_explicit_severity_wins_both_ways():
    from placement import floorplan as fp
    board = _moved_esp('C2')
    doc = _esp_doc()
    doc['severity'] = {'decap_ungraded': 'warn'}
    g = _grade_doc(doc, board)
    assert _sev(g, 'C2') == 'warn' and _sev(g, 'C1') == 'warn'
    assert not fp._gating('decap_ungraded', fp.intent_from_dict(doc), None)
    # The #1102 shape: board-wide error, no list -- graded as it always was.
    old = _esp_doc()
    old['decaps'].pop('within_radius_refs')
    old['decaps'].pop('within_radius_mm')
    old['severity'] = {'decap_ungraded': 'error'}
    g = _grade_doc(old, board)
    assert _sev(g, 'C2') == 'error' and _sev(g, 'C1') == 'error'
    assert fp._gating('decap_ungraded', fp.intent_from_dict(old), None)
    print("  PASS: explicit warn turns the held cap down; the #1102 shape "
          "grades every beyond cap ERROR")


def test_gating_follows_the_list():
    from placement import floorplan as fp
    doc = _esp_doc()
    assert fp._gating('decap_ungraded', fp.intent_from_dict(doc), None)
    doc['decaps'].pop('within_radius_refs')
    doc['decaps'].pop('within_radius_mm')
    assert not fp._gating('decap_ungraded', fp.intent_from_dict(doc), None)
    print("  PASS: a non-empty list makes the rule gating; none does not")


def test_the_loader_refuses_a_list_it_cannot_apply():
    from placement import floorplan as fp

    def refuses(mut, why):
        doc = _esp_doc()
        mut(doc['decaps'])
        try:
            fp.intent_from_dict(doc)
        except fp.IntentError as exc:
            assert why in str(exc), (why, str(exc))
            return
        raise AssertionError(f'loaded: {doc["decaps"]}')
    refuses(lambda d: d.pop('within_radius_mm'), 'come together')
    refuses(lambda d: d.pop('within_radius_refs'), 'come together')
    refuses(lambda d: d.pop('max_distance_mm'), 'would grade nothing')
    refuses(lambda d: d.__setitem__('search_radius_mm', 3.0),
            'cannot be graded at another')
    refuses(lambda d: d.__setitem__('within_radius_refs', 'C2'),
            'bare string')
    refuses(lambda d: d.__setitem__('within_radius_mm', 0), 'positive')
    # null would read as () and disarm the rule silently.
    refuses(lambda d: d.__setitem__('within_radius_refs', None), 'is null')
    # A search radius that matches is accepted.
    doc = _esp_doc()
    doc['decaps']['search_radius_mm'] = 5.0
    fp.intent_from_dict(doc)
    # A reader-7 build refuses the emitted intent by its min_reader.
    saved = fp.READER_VERSION
    try:
        fp.READER_VERSION = 7
        try:
            fp.intent_from_dict(_esp_doc())
        except fp.IntentError as exc:
            assert 'this build is reader 7' in str(exc), str(exc)
        else:
            raise AssertionError('a reader-7 build loaded min_reader 8')
    finally:
        fp.READER_VERSION = saved
    print("  PASS: seven refusals, a matching radius loads, reader 7 refuses")


def test_a_footprint_mismatch_is_not_held():
    from placement import floorplan as fp
    pcb = _parse(ESP)
    pcb.footprints['C3'].footprint_name = 'other_lib:NOT_THE_SAME'
    doc = _quiet(fp.emit_intent, pcb, ESP, decaps_from=ESP)
    assert doc['decaps']['within_radius_refs'] == ['C2', 'C4'], doc['decaps']
    print("  PASS: C3 under another footprint is not held")


def test_the_cli_fixed_point_exits_zero():
    td = tempfile.mkdtemp(dir=_TMP.name)
    ip = os.path.join(td, 'i.json')
    r = subprocess.run([sys.executable, '-X', 'utf8', FLOORPLAN, ESP,
                        '--emit-intent', ip, '--decaps-from', ESP],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    assert r.returncode == 0, r.stdout[-800:] + r.stderr[-800:]
    assert 'decap_ungraded promoted to error -- per cap' in r.stdout, (
        r.stdout[-1500:])
    g = subprocess.run([sys.executable, '-X', 'utf8', FLOORPLAN, ESP,
                        '--intent', ip],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    assert g.returncode == 0, g.stdout[-1500:]
    print("  PASS: esp_prog graded on its own --decaps-from intent exits 0")


def test_repair_does_not_push_a_held_cap_further():
    """`place_seed --repair` seats a violator at its nearest LEGAL pose, not
    toward its IC, so charging a stranded held cap nudged it further out and
    shipped the worse pose (final review: esp_prog C2 at (118, 95), 5.64 ->
    5.69 mm). The repair charges no one for `decap_ungraded`: C2 is not a
    violator and does not move, and its ERROR stays in the grade."""
    from placement import floorplan as fp
    from placement import seeder
    from placement.writer import write_placed_output
    out = os.path.join(tempfile.mkdtemp(dir=_TMP.name), 'c2.kicad_pcb')
    pcb0 = _parse(ESP)
    c2 = pcb0.footprints['C2']
    _quiet(write_placed_output, ESP, out,
           [{'reference': 'C2', 'new_x': 118.0, 'new_y': 95.0,
             'new_rotation': c2.rotation or 0.0}])
    pcb = _parse(out)
    doc = _quiet(fp.emit_intent, pcb, out, decaps_from=ESP)
    intent = fp.intent_from_dict(doc)
    g = _quiet(fp.grade, intent, pcb, out)
    held = [v for v in g.errors if v.rule == 'decap_ungraded']
    assert [v.ref for v in held] == ['C2'], [v.ref for v in held]
    res = _quiet(seeder.repair_placement, _parse(out), out, intent,
                 clearance=0.2)
    assert 'C2' not in (res.get('violators') or []), res.get('violators')
    moved = {m.get('reference') for m in (res.get('moves') or [])
             if isinstance(m, dict)}
    assert 'C2' not in moved, res.get('moves')
    # ...and it is SAID, so a --dry-run (no final grade) still names it.
    assert any(n.startswith('C2: not charged') for n in res.get('notes') or ()), (
        res.get('notes'))
    print(f"  PASS: C2 (held, stranded) is no violator and does not move; "
          f"its decap_ungraded ERROR stays, and the notes name it")


def test_the_issue_pile_splitflap():
    """The issue's repro, on its first board: splitflap's reference keeps C3
    beyond the radius; the seed strands seven caps the reference keeps
    within it. Those are ERRORs now, and C3 stays a WARN."""
    import test_placement_ab as AB
    from placement import floorplan as fp
    from placement import groups
    board = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    d = tempfile.mkdtemp(dir=_TMP.name)
    pile, intent, doc, seed_refs = _quiet(AB._pile_inputs, board, d)
    held = set(doc['decaps']['within_radius_refs'])
    near, beyond, _o = groups.decap_populations(_parse(board))
    assert held == {c for caps in near.values() for c, _d in caps}, held
    assert 'C3' not in held, held
    out = os.path.join(d, 'seed', 'splitflap_driver.kicad_pcb')
    os.makedirs(os.path.dirname(out))
    _quiet(AB._run_seed, pile, out, intent, {'seed_refs': seed_refs},
           ignore_nets=['GND'])
    g = _quiet(fp.grade, intent, _parse(out), out)
    du = [v for v in list(g.errors) + list(g.warnings)
          if v.rule == 'decap_ungraded']
    stranded = sorted(v.ref for v in du if v.ref in held)
    assert stranded, 'the seed stranded no held cap: the case tests nothing'
    assert all(v.severity == 'error' for v in du if v.ref in held), du
    assert all(v.severity == 'warn' for v in du if v.ref not in held), du
    assert any(v.ref == 'C3' and v.severity == 'warn' for v in du), (
        [(v.ref, v.severity) for v in du])
    print(f"  PASS: stranded held caps {stranded} are ERROR; C3 stays WARN")


TESTS = [v for k, v in sorted(globals().items())
         if k.startswith('test_') and callable(v)]


if __name__ == '__main__':
    want = sys.argv[1:]
    n = 0
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(t.__name__)
        t()
        n += 1
    print(f"ALL PASS ({n} case(s))")
