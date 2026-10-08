#!/usr/bin/env python3
"""#1113: `rank_rotations.py`, the seed-level ranker for one part's rotation.
The fast half: its decisions, without paying for a seed per angle.

* `rotation_key`: a hard fail ranks last; fewer unseated parts beats fewer
  crossings; a probed angle outranks an unprobed one, then fewer probe
  failures; then crossings, hpwl, grade errors; a tie keeps the input angle.
  A gated seed (place_seed exit 4) is NOT a tier.
* `classify_row`: a part written at another angle than declared is
  `rotation_not_applied`, an unseated one `ref_unseated`.
* `candidate_rotations` / `same_angle`: the input angle first, quarter turns,
  45-degree turns only on request, modulo 360.
* `eligible_refs` on the committed run-29 pile: U1 by default; the
  connectors are excluded by class; a declared rotation, a lock, a fixed pose
  exclude a part outright.
* `with_rotation`: the derived intent loads, carries exactly one rotation
  block for the part, at reader >= 5, and names the part literally.
* refusals, each for its stated reason (`run_utils.check`).

    python3 tests/test_1113_rank_rotations.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb    # noqa: E402
from placement import floorplan as fp       # noqa: E402
import rank_rotations as rr                 # noqa: E402
import run_utils                            # noqa: E402

RUN_ALL_TIMEOUT = 900

PILE = os.path.join(TESTS_DIR, 'fixtures', '959', 'run29_pile.kicad_pcb')
ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
TOOL = run_utils.tool('rank_rotations.py')
SOURCES = ('kicad', 'sheet')


def _agg(rot, idx, crossings, *, hpwl=100.0, unseated=0, hard=None,
         probed=False, failures=None, errors=0):
    def _s(v):
        return {'median': v, 'min': v, 'max': v, 'by_seed': {0: v}}
    return {'rotation': rot, 'ladder_index': idx, 'hard_fail': hard,
            'unseated_max': unseated, 'probed': probed,
            'probe_failures': failures, 'crossings': _s(crossings),
            'hpwl': _s(hpwl), 'grade_errors': _s(errors)}


def _rank(aggs):
    return [a['rotation'] for a in sorted(aggs, key=rr.rotation_key)]


def test_rotation_key_orders_as_documented():
    assert _rank([_agg(0, 0, 100), _agg(90, 1, 90, hard='ref_unseated')]) \
        == [0, 90]
    assert _rank([_agg(0, 0, 50, unseated=1), _agg(90, 1, 99)]) == [90, 0]
    # a probe verdict outranks crossings, fewer failures first
    assert _rank([_agg(0, 0, 10), _agg(90, 1, 99, probed=True, failures=5)]) \
        == [90, 0]
    assert _rank([_agg(0, 0, 10, probed=True, failures=6),
                  _agg(90, 1, 99, probed=True, failures=5)]) == [90, 0]
    assert _rank([_agg(0, 0, 100, hpwl=9), _agg(90, 1, 100, hpwl=8)]) \
        == [90, 0]
    # crossings outrank hpwl when they disagree, hpwl outranks grade errors
    assert _rank([_agg(0, 0, 100, hpwl=1), _agg(90, 1, 99, hpwl=500)]) \
        == [90, 0]
    assert _rank([_agg(0, 0, 100, hpwl=8, errors=0),
                  _agg(90, 1, 100, hpwl=7, errors=5)]) == [90, 0]
    assert _rank([_agg(0, 0, 100, errors=2), _agg(90, 1, 100, errors=1)]) \
        == [90, 0]
    # a full tie keeps the input angle (ladder index 0)
    assert _rank([_agg(90, 1, 100), _agg(0, 0, 100)]) == [0, 90]
    # #1117: a declined re-seat ranks after a held one, whatever its
    # crossings, and before a hard fail
    a0 = dict(_agg(0, 0, 10), declined_seeds=1)
    assert _rank([a0, _agg(90, 1, 500)]) == [90, 0]
    assert _rank([a0, _agg(90, 1, 1, hard='ref_unseated')]) == [0, 90]
    # missing metrics rank after measured ones
    assert _rank([_agg(0, 0, None), _agg(90, 1, 500)]) == [90, 0]
    print("  PASS: hard fail < unseated < probe < crossings < hpwl < errors "
          "< ladder")


def test_classify_row():
    base = {'rotation': 270.0, 'place_seed_rc': 4, 'unseated_refs': [],
            'rotation_unseated': {}, 'crossings': 10}
    assert rr.classify_row(dict(base), 'U1', -90.0)['hard_fail'] is None
    assert rr.classify_row(dict(base), 'U1', 0.0)['hard_fail'] == \
        'rotation_not_applied'
    r = dict(base, unseated_refs=['U1'])
    assert rr.classify_row(r, 'U1', 270.0)['hard_fail'] == 'ref_unseated'
    r = dict(base, rotation_unseated={'U1': 'no pose'})
    assert rr.classify_row(r, 'U1', 270.0)['hard_fail'] == 'ref_unseated'
    # #1117: the angle held but the re-seat could not put the part back -- a
    # TIER (rotation_key), not a hard fail. Another part's decline is not
    # this arm's.
    r = rr.classify_row(dict(base, reseat_declined={'U1': {'rotation': 270.0}}),
                        'U1', 270.0)
    assert r['hard_fail'] is None and r['reseat_declined_ref'] is True, r
    r = rr.classify_row(dict(base, reseat_declined={'C3': {'rotation': None}}),
                        'U1', 270.0)
    assert r['reseat_declined_ref'] is False, r
    assert rr.classify_row(dict(base, place_seed_rc=1), 'U1', 270.0)[
        'hard_fail'] == 'place_seed_failed'
    assert rr.classify_row(dict(base, crossings=None), 'U1', 270.0)[
        'hard_fail'] == 'no_metrics'
    print("  PASS: written angle, unseated, rc and metrics each hard-fail")


def test_candidate_rotations():
    assert rr.candidate_rotations(0.0) == [0.0, 90.0, 180.0, 270.0]
    assert rr.candidate_rotations(-135.0) == [225.0, 315.0, 45.0, 135.0]
    assert rr.candidate_rotations(0.0, diagonal=True)[4:] == \
        [45.0, 135.0, 225.0, 315.0]
    assert rr.candidate_rotations(0.0, explicit=[270, 0]) == [270.0, 0.0]
    assert rr.same_angle(-90, 270) and not rr.same_angle(0, 90)
    print("  PASS: input first, quarter turns, diagonals on request")


def _pile_intent(td, mutate=None):
    doc = fp.emit_intent(parse_kicad_pcb(PILE), PILE, decaps_from=ESP)
    if mutate:
        mutate(doc)
    path = os.path.join(td, 'intent.json')
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=1)
    return doc, path


def _eligible(doc, pcb=None, **kw):
    pcb = pcb or parse_kicad_pcb(PILE)
    intent = fp.intent_from_dict(doc, PILE)
    blocks, _ = fp.resolve_blocks(intent, pcb, SOURCES)
    return rr.eligible_refs(pcb, intent, blocks, PILE, **kw)


def test_eligibility_on_the_run29_pile():
    run_utils.evidence(PILE, 'the run-29 pile')
    with tempfile.TemporaryDirectory() as td:
        doc, _p = _pile_intent(td)
        cands, ex = _eligible(doc)
        assert cands and cands[0][0] == 'U1', cands
        _c6, ex6 = _eligible(doc, min_pads=6)
        assert {'CON1', 'CON2'} <= {r for r, w in ex6['soft'].items()
                                    if w.startswith('class:')}, ex6
        # a declared rotation excludes it outright
        doc2 = json.loads(json.dumps(doc))
        doc2['blocks'] = list(doc2.get('blocks') or []) + [
            {'name': 'u1_rot', 'refs': ['U1'], 'rotation': 90}]
        doc2['min_reader'] = max(int(doc2.get('min_reader') or 0), 5)
        c2, ex2 = _eligible(doc2)
        assert ex2['hard'].get('U1') == 'declared_rotation', ex2
        assert 'U1' not in dict(c2), c2
        # a file lock excludes it outright
        pcb = parse_kicad_pcb(PILE)
        pcb.footprints['U1'].locked = True
        c3, ex3 = _eligible(doc, pcb=pcb)
        assert ex3['hard'].get('U1') == 'locked', ex3
    print(f"  PASS: U1 by default ({cands[0][1]} pads); CON1/CON2 by class; "
          f"declared and locked excluded")


def test_the_derived_intent():
    with tempfile.TemporaryDirectory() as td:
        doc, _p = _pile_intent(td)
        arm = rr.with_rotation(doc, 'U1', 270.0)
        intent = fp.intent_from_dict(arm, PILE)
        rot = [b for b in arm['blocks'] if b.get('rotation') is not None]
        assert len(rot) == 1 and rot[0]['refs'] == ['U1'], rot
        assert arm['min_reader'] >= 5, arm['min_reader']
        pcb = parse_kicad_pcb(PILE)
        blocks, _ = fp.resolve_blocks(intent, pcb, SOURCES)
        assert fp.rotations_for_ref(intent, blocks)['U1'][0] == 270.0
        assert doc.get('blocks') != arm['blocks']      # the input is a copy
        assert rr.rotation_block('Ref*', 0)['refs'] == ['Ref[*]']
        taken = {'rotation:U1'}
        assert rr.rotation_block('U1', 0, taken)['name'] == 'rotation:U1~2'
    print("  PASS: one literal rotation block at reader >= 5; the input "
          "document is untouched")


def test_refusals_say_why():
    with tempfile.TemporaryDirectory() as td:
        _doc, ipath = _pile_intent(td)
        base = [sys.executable, '-X', 'utf8', TOOL, PILE, '--intent', ipath,
                '--out-dir', os.path.join(td, 'o')]
        run_utils.check(base + ['--ref', 'U9'], code=2,
                        refuse='U9 names nothing on this board')
        run_utils.check(base + ['--seed-args=--seed 3'], code=2,
                        refuse='--seed-args may not carry --seed')
        # an abbreviation argparse would expand is the flag it expands to
        run_utils.check(base + ['--seed-args=--se 3'], code=2,
                        refuse='--se (= --seed)')
        run_utils.check(base + ['--seed-args=--no-pol'], code=2,
                        refuse='--no-pol (= --no-polish)')
        run_utils.check(base + ['--seed-args=--re'], code=2,
                        refuse='--re (= <ambiguous>)')
        # output paths are refused before any seed runs
        run_utils.check(base + ['--write-best', os.path.join(td, 'x.pcb')],
                        code=2, refuse='--write-best must name a .kicad_pcb')
        run_utils.check(base + ['--write-intent',
                                os.path.join(td, 'nodir', 'i.json')],
                        code=2, refuse='--write-intent: no such directory')
        run_utils.check(base + ['--rotations', '0', '360'], code=2,
                        refuse='--rotations has duplicates')
        _d2, ip2 = _pile_intent(td, lambda d: d.update(
            blocks=list(d.get('blocks') or []) + [
                {'name': 'u1_rot', 'refs': ['U1'], 'rotation': 90}],
            min_reader=max(int(d.get('min_reader') or 0), 5)))
        run_utils.check(base[:6] + [ip2] + base[7:] + ['--ref', 'U1'],
                        code=4, refuse='already has a declared rotation')
        run_utils.check(base + ['--min-pads', '999'], code=4,
                        refuse='no part is eligible to rank')
        # a placed board: place_seed itself refuses (exit 3), and says how
        esp_doc = fp.emit_intent(parse_kicad_pcb(ESP), ESP)
        eip = os.path.join(td, 'esp.json')
        with open(eip, 'w', encoding='utf-8') as fh:
            json.dump(esp_doc, fh)
        run_utils.check([sys.executable, '-X', 'utf8', TOOL, ESP, '--intent',
                         eip, '--out-dir', os.path.join(td, 'e'), '--ref',
                         'U1'], code=3, refuse="--seed-args='--force'")
    print("  PASS: unknown ref, forbidden seed arg, duplicate angle (2); "
          "declared, nothing eligible (4); placed board (3)")


TESTS = [
    test_rotation_key_orders_as_documented,
    test_classify_row,
    test_candidate_rotations,
    test_eligibility_on_the_run29_pile,
    test_the_derived_intent,
    test_refusals_say_why,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        # A filter that names no case passes nothing: a mutation battery
        # witness spelled wrong would otherwise read every row as SURVIVED.
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
