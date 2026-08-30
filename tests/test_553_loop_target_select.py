"""#553: `--target-select`, and the proof that its default changed nothing.

Two claims, and the first is the one that matters most:

  1. `--target-select pins` -- the default, and the behaviour every recorded
     manifest replays -- passes the quench EXACTLY the kwargs it passed before,
     and writes a round sidecar with exactly the keys it wrote before. Asserted
     by running the loop twice (flag absent, flag given) and comparing the
     recorded kwargs, plus a source pin on the pins branch itself.
  2. `--target-select diagnosis` actually reaches the diagnosis branch. The
     trap here is a flag that looks wired and silently reduces to `pins`: on a
     board where the two selectors agree, every assertion about "the diagnosis
     was used" passes without the branch ever running. So this file asserts
     `pins != diagnosis` FIRST, on a board built so they cannot coincide, and
     only then asserts which one the quench received.

The ranking itself is not tested here -- `test_553_diagnosis_rank.py` owns it,
and `diagnose_round` is replaced with a recorder so this file tests WIRING.
That separation is deliberate: a wiring test that also computes the ranking
cannot tell a wiring bug from a ranking bug.

Nothing is routed. The router, the quench and the writer are recorders, so
this stays in the `run_all.py --fast` lane.

    python3 -X utf8 tests/test_553_loop_target_select.py
"""

import json
import os
import shutil
import sys
import tempfile
from io import StringIO

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))   # placement split

import place_route_loop as prl  # noqa: E402
from placement import diagnosis as D  # noqa: E402

# What `nets_to_refs` returns on this board: every pad owner of net NA,
# INCLUDING the KiCad-locked J1 -- the pin filter does not read `locked`,
# quench does. Pinned as a name so the diagnosis budget and the overlap
# triple below cannot silently drift apart from it.
PINS_SET = {'C1', 'C2', 'R1', 'J1'}

FAILURES = []


def check(cond, what):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}')
        FAILURES.append(what)


def _pad(ref, x, uuid):
    return (f'\t(footprint "test:CAP2P"\n\t\t(layer "F.Cu")\n'
            f'\t\t(uuid "fp-{uuid}")\n\t\t(at {x} 100)\n'
            f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0)\n\t\t)\n'
            f'\t\t(pad "1" smd rect\n\t\t\t(at -0.5 0)\n\t\t\t(size 0.8 0.8)\n'
            f'\t\t\t(layers "F.Cu")\n\t\t\t(net 1 "NA")\n'
            f'\t\t\t(uuid "p1-{uuid}")\n\t\t)\n\t)\n')


def _board():
    """THREE movable parts on one net, plus a locked anchor.

    Three, not one: with a single movable part the pin selector and any other
    selector return the same set, and every "the diagnosis was used" assertion
    passes on a branch that never ran.
    """
    body = '(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n\t(net 1 "NA")\n'
    body += ('\t(gr_rect\n\t\t(start 100 80)\n\t\t(end 200 120)\n'
             '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type solid)\n\t\t)\n'
             '\t\t(layer "Edge.Cuts")\n\t\t(uuid "edge1")\n\t)\n')
    body += _pad('C1', 140, 'C1') + _pad('C2', 150, 'C2') + _pad('R1', 160, 'R1')
    body += ('\t(footprint "test:PIN"\n\t\t(layer "F.Cu")\n\t\t(locked yes)\n'
             '\t\t(uuid "fp-J1")\n\t\t(at 170 100)\n'
             '\t\t(property "Reference" "J1"\n\t\t\t(at 0 0)\n\t\t)\n'
             '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1 1)\n'
             '\t\t\t(layers "F.Cu")\n\t\t\t(net 1 "NA")\n'
             '\t\t\t(uuid "p1-J1")\n\t\t)\n\t)\n)\n')
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w') as f:
        f.write(body)
    return path, tempfile.mkdtemp()


def _diagnosis(selected, degenerate=False):
    """A canned Diagnosis: this file tests wiring, not ranking."""
    if degenerate:
        return D.Diagnosis(skipped={s: 'canned: nothing to rank'
                                    for s in D.SIGNAL_ORDER})
    members = tuple(sorted(selected))
    return D.Diagnosis(
        candidates=[D.Candidate(key=members[0], kind='part', members=members,
                                rows=(D.SignalRow('legality_pairs', 2.0, 1,
                                                  'pairs'),),
                                selected_by=('legality_pairs',))],
        selected=sorted(selected), selected_keys=[members[0]],
        skipped={'block_displacement': 'canned', 'blocker_cells': 'canned'})


def _run(extra_args, rounds=1, diag=None, seen=None):
    """main() with the router, quench, writer and diagnosis all recorded.

    Returns (quench kwargs per round, captured stdout).
    """
    calls = []

    def fake_quench(pcb_data, **kw):
        calls.append(dict(kw))
        return [{'reference': 'C1', 'new_x': 141.0, 'new_y': 100.0,
                 'new_rotation': 0.0}]

    def fake_run_route(pcb_file, routed_file, route_args, log_file, **kw):
        if seen is not None:
            seen.setdefault('keep_blocker_cells', []).append(
                kw.get('keep_blocker_cells'))
        return {'failures': 2, 'failed_nets': ['NA'], 'blockers': [],
                'iterations': 1000, 'vias': 0,
                'blocker_report': [{'net': 'NA', 'blocked_by': [
                    {'net': 'NA', 'blocked_count': 3}]}]}

    def fake_diagnose_round(pcb_data, pcb_file, blocks, metrics, **kw):
        if seen is not None:
            seen.setdefault('diagnose', []).append(
                {'blocks': dict(blocks), 'kw': dict(kw),
                 'blocker_report': metrics.get('blocker_report')})
        return diag

    board, work = _board()
    saved = (prl.quench, prl.run_route, prl.write_placed_output,
             prl.diagnose_round, sys.argv, sys.stdout)
    prl.quench = fake_quench
    prl.run_route = fake_run_route
    prl.write_placed_output = lambda src, dst, pl: shutil.copy(src, dst)
    if diag is not None:
        prl.diagnose_round = fake_diagnose_round
    sys.argv = ['place_route_loop.py', board,
                os.path.join(work, 'out.kicad_pcb'),
                '--route-args', '--nets "*"', '--rounds', str(rounds),
                '--max-displacement', '3.0', '--work-dir', work] + extra_args
    buf = StringIO()
    sys.stdout = buf
    sidecars = {}
    try:
        prl.main()
    finally:
        sys.stdout = saved[5]
        for n in range(rounds + 1):
            p = os.path.join(work, f'loop_round{n}.json')
            if os.path.exists(p):
                with open(p, encoding='utf-8') as f:
                    sidecars[n] = json.load(f)
        (prl.quench, prl.run_route, prl.write_placed_output,
         prl.diagnose_round, sys.argv, sys.stdout) = saved
        os.unlink(board)
        shutil.rmtree(work, ignore_errors=True)
    if seen is not None:
        seen['sidecars'] = sidecars
        seen['stdout'] = buf.getvalue()
    return calls, buf.getvalue()


def _summary(text):
    for line in text.splitlines():
        if line.startswith('JSON_SUMMARY: '):
            return json.loads(line[len('JSON_SUMMARY: '):])
    return {}


# --------------------------------------------------------------------------
# 1. The default did not move
# --------------------------------------------------------------------------

def t_the_default_passes_the_quench_identical_kwargs():
    absent, _ = _run([])
    explicit, _ = _run(['--target-select', 'pins'])

    def _cmp(calls):
        # `pcb_file` is a per-run temp path and is the ONLY key allowed to
        # differ; comparing it would make this test about tempfile, not about
        # the selector.
        return [{k: v for k, v in c.items() if k != 'pcb_file'} for c in calls]

    check(_cmp(absent) == _cmp(explicit),
          'flag absent and --target-select pins record identical quench kwargs')
    check(len(absent) == 1 and absent[0]['move_refs'] == PINS_SET,
          f"and the move set is still every pad owner of the failed net "
          f"({absent[0]['move_refs'] if absent else None})")
    check(absent[0]['groups'] == {},
          'with no --group-by there are still no groups')


def t_the_default_sidecar_keeps_its_key_set():
    seen = {}
    _run([], seen=seen)
    doc = seen['sidecars'][1]
    check(sorted(doc) == ['accepted', 'board', 'groups', 'metrics', 'moved',
                          'parent', 'round', 'routed', 'schema', 'screened',
                          'targets'],
          f'a pins round writes the eleven keys it always wrote ({sorted(doc)})')
    check('diagnosis' not in doc,
          'and no None placeholder for the key it did not use')


def t_the_default_verdict_gains_exactly_one_key():
    _, out = _run([])
    s = _summary(out)
    check(s.get('target_select') == 'pins',
          'the verdict echoes the selector, like group_by before it')
    extra = [k for k in s if k.startswith('target_select_')]
    check(extra == [],
          f'and carries no diagnosis bookkeeping in pins mode ({extra})')


def t_the_pins_branch_is_still_the_pins_branch():
    """A source pin: a refactor that moved the default path fails HERE, with a
    name, rather than as a mysterious behaviour difference two phases later."""
    import inspect
    src = inspect.getsource(prl.main)
    for frag in ('pins_targets = nets_to_refs(pcb_data,',
                 "best['failed_nets'] + best['blockers'],",
                 'args.max_target_pins, args.lock)'):
        check(frag in src, f'the pin filter still reads {frag!r}')


def t_the_router_is_only_asked_for_cells_when_they_are_wanted():
    seen = {}
    _run([], seen=seen)
    check(set(seen['keep_blocker_cells']) == {False},
          'pins mode never asks the router summary for its cell counts')
    seen2 = {}
    _run(['--target-select', 'diagnosis', '--group-by', 'decap'],
         diag=_diagnosis({'C2'}), seen=seen2)
    check(set(seen2['keep_blocker_cells']) == {True},
          'diagnosis mode asks for them on every route, round 0 included')


# --------------------------------------------------------------------------
# 2. The diagnosis branch is actually reached
# --------------------------------------------------------------------------

def t_the_two_selectors_disagree_on_this_board():
    """Asserted BEFORE anything about which was used. If they agreed, every
    assertion below would pass on a branch that never ran."""
    pins, _ = _run([])
    seen = {}
    diag, _ = _run(['--target-select', 'diagnosis', '--group-by', 'decap'],
                   diag=_diagnosis({'C2'}), seen=seen)
    check(pins[0]['move_refs'] == PINS_SET,
          f'pins offers every pad owner, locked ones included -- quench does '
          f'that filtering, not nets_to_refs ({pins[0]["move_refs"]})')
    check(diag[0]['move_refs'] == {'C2'}, 'the diagnosis offers one')
    check(pins[0]['move_refs'] != diag[0]['move_refs'],
          'the two selectors return DIFFERENT sets -- this file is not vacuous')


def t_the_diagnosis_receives_the_routers_evidence():
    seen = {}
    _run(['--target-select', 'diagnosis', '--group-by', 'decap'],
         diag=_diagnosis({'C2'}), seen=seen)
    got = seen['diagnose'][0]
    check(got['blocker_report'] == [{'net': 'NA', 'blocked_by': [
              {'net': 'NA', 'blocked_count': 3}]}],
          'the cell counts reach the ranking, not just the net names')
    check(got['kw'].get('budget') == len(PINS_SET),
          f"the budget is what pins would have spent ({got['kw'].get('budget')})")
    check(got['kw'].get('top_k') == D.TOP_K,
          'and the report size comes from --diagnosis-top-k')


def t_an_operator_lock_is_never_overridden_by_a_diagnosis():
    seen = {}
    calls, _ = _run(['--target-select', 'diagnosis', '--group-by', 'decap',
                     '--lock', 'C2'],
                    diag=_diagnosis({'C2', 'R1'}), seen=seen)
    check(calls[0]['move_refs'] == {'R1'},
          f"a diagnosed part matching --lock is dropped "
          f"({calls[0]['move_refs']})")


def t_a_degenerate_diagnosis_falls_back_loudly():
    seen = {}
    calls, out = _run(['--target-select', 'diagnosis', '--group-by', 'decap'],
                      diag=_diagnosis(set(), degenerate=True), seen=seen)
    check(calls[0]['move_refs'] == PINS_SET,
          'the round still runs, on the pin set')
    s = _summary(out)
    check(s.get('target_select_rounds_fallback') == 1
          and s.get('target_select_rounds_diagnosis') == 0,
          f'and the verdict COUNTS the fallback '
          f'({s.get("target_select_rounds_diagnosis")}/'
          f'{s.get("target_select_rounds_fallback")})')
    reasons = s.get('target_select_fallback_reasons') or []
    check(reasons and all(sig in reasons[0] for sig in D.SIGNAL_ORDER),
          f'naming every signal that could not be defined ({reasons})')
    check('FALLING BACK' in out, 'and says so on stdout, not only in the JSON')


def t_the_verdict_cannot_be_read_without_the_no_efficacy_sentence():
    _, out = _run(['--target-select', 'diagnosis', '--group-by', 'decap'],
                  diag=_diagnosis({'C2'}))
    s = _summary(out)
    check(s.get('target_select_efficacy') == D.NO_EFFICACY_CLAIM,
          'the verdict carries the sentence saying nothing measured this')
    check(s.get('target_select_rounds_diagnosis') == 1,
          'and counts the round the diagnosis actually steered')
    ov = s.get('target_select_overlap') or []
    check(ov and ov[0] == {'round': 1, 'pins': len(PINS_SET),
                           'diagnosis': 1, 'overlap': 1},
          f'the pins/diagnosis overlap is recorded per round ({ov}) -- if the '
          f'flag ever degenerates into pins, these three numbers are equal')


def t_the_round_sidecar_carries_the_diagnosis():
    seen = {}
    _run(['--target-select', 'diagnosis', '--group-by', 'decap'],
         diag=_diagnosis({'C2'}), seen=seen)
    doc = seen['sidecars'][1]
    check('diagnosis' in doc, 'the round records what the ranking said')
    check(doc['diagnosis']['efficacy'] == D.NO_EFFICACY_CLAIM,
          'including the disclaimer, so a recorded round cannot lose it')
    check(doc['diagnosis']['selected'] == ['C2'],
          'and the selection it produced')


def t_the_block_census_is_printed_before_round_zero():
    seen = {}
    _run(['--target-select', 'diagnosis', '--group-by', 'decap'],
         diag=_diagnosis({'C2'}), seen=seen)
    out = seen['stdout']
    check('Group sources on this board:' in out, 'the census is printed')
    for src in ('kicad', 'sheet', 'netprefix', 'decap'):
        check(src in out, f'naming {src}')
    check('nothing on this board' in out,
          'and saying which sources derive nothing, rather than leaving the '
          'operator to infer it from a selection that never changes')
    check(out.index('Group sources') < out.index('Round 0'),
          'BEFORE round 0 routes the whole board')
    check('WARNING' in out and 'cannot run' in out,
          'a source that derives no block warns that the displacement signal '
          'is lost -- it does not silently rank on two signals')


TESTS = [
    t_the_default_passes_the_quench_identical_kwargs,
    t_the_default_sidecar_keeps_its_key_set,
    t_the_default_verdict_gains_exactly_one_key,
    t_the_pins_branch_is_still_the_pins_branch,
    t_the_router_is_only_asked_for_cells_when_they_are_wanted,
    t_the_two_selectors_disagree_on_this_board,
    t_the_diagnosis_receives_the_routers_evidence,
    t_an_operator_lock_is_never_overridden_by_a_diagnosis,
    t_a_degenerate_diagnosis_falls_back_loudly,
    t_the_verdict_cannot_be_read_without_the_no_efficacy_sentence,
    t_the_round_sidecar_carries_the_diagnosis,
    t_the_block_census_is_printed_before_round_zero,
]


def main():
    for fn in TESTS:
        print(f'{fn.__name__}:')
        try:
            fn()
        except Exception as e:                       # noqa: BLE001
            check(False, f'{fn.__name__} raised {type(e).__name__}: {e}')
    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_553_loop_target_select: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
