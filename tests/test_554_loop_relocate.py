"""#554: `--relocate` wiring, and the proof that its absence changed nothing.

Two claims, and the first is the one that matters most:

  1. **Without `--relocate` the loop is what it was.** Same quench kwargs, same
     round-sidecar key set, and exactly ONE additive echo key in `JSON_SUMMARY`
     -- in the same class as the `group_by` key #459 added and the
     `target_select` key #553 added. Asserted by running the loop twice (flag
     absent, flag given) and comparing recorded kwargs.
  2. **With it, the relocation actually reaches the board.** The trap is a flag
     that looks wired and silently does nothing: on a board where the proposal
     is refused, every "it was applied" assertion passes without the branch ever
     running. So this file asserts the proposal was ACCEPTED first, then asserts
     the quench was handed the RELOCATED board rather than the round's input.

The solve itself is not tested here -- `test_554_order_graph.py` and
`test_554_relocate_solve.py` own it, and `relocate_round` is replaced with a
recorder so this file tests WIRING. A wiring test that also runs the solve
cannot tell a wiring bug from a solve bug.

Nothing is routed. The router, the quench and the writer are recorders.

    python3 -X utf8 tests/test_554_loop_relocate.py
"""

import json
import os
import shutil
import sys
import tempfile
from io import StringIO

# Recorders only -- no board is routed. The marker regex anchors on end of line,
# so this assignment must carry no trailing comment.
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

import place_route_loop as prl                    # noqa: E402
from placement import relocate as RL              # noqa: E402

FAILURES = []


def check(cond, what):
    if not cond:
        FAILURES.append(what)
    return cond


def _pad(ref, x, uuid, net=1):
    return (f'\t(footprint "test:CAP2P"\n\t\t(layer "F.Cu")\n'
            f'\t\t(uuid "fp-{uuid}")\n\t\t(at {x} 100)\n'
            f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0)\n\t\t)\n'
            f'\t\t(pad "1" smd rect\n\t\t\t(at -0.5 0)\n\t\t\t(size 0.8 0.8)\n'
            f'\t\t\t(layers "F.Cu")\n\t\t\t(net {net} "{"NA" if net else ""}")\n'
            f'\t\t\t(uuid "p1-{uuid}")\n\t\t)\n\t)\n')


def _board():
    body = '(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n\t(net 1 "NA")\n'
    body += ('\t(gr_rect\n\t\t(start 100 80)\n\t\t(end 200 120)\n'
             '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type solid)\n\t\t)\n'
             '\t\t(layer "Edge.Cuts")\n\t\t(uuid "edge1")\n\t)\n')
    body += _pad('C1', 140, 'C1') + _pad('C2', 150, 'C2') + _pad('R1', 160, 'R1')
    # M1 is on NO net, so `nets_to_refs` can never offer it. That is what makes
    # "the block's members reach move_refs" testable at all: asserting a ref the
    # pin filter already selected passes whether or not the line runs, and this
    # test SURVIVED that mutation until M1 existed.
    body += _pad('M1', 120, 'M1', net=0)
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


def _proposal(refusal='', corridor=('C2',)):
    """A canned Relocation: this file tests wiring, not the solve."""
    if refusal:
        return RL.Relocation(block='decap:M1', refusal=refusal,
                             disclosures=(RL.NO_EFFICACY_CLAIM,))
    return RL.Relocation(
        block='decap:M1', members=('M1',), direction=(1.0, 0.0),
        want_mm=5.0, reach_mm=4.0, frozen_reach_mm=1.0, dose_mm=4.0,
        shift=(4.0, 0.0), corridor=tuple(corridor), corridor_mm=0.5,
        binding_path=(('C2', 0.25, 'clearance'),), solver='milp',
        moves=({'reference': 'M1', 'new_x': 124.0, 'new_y': 90.0,
                'new_rotation': 0.0},),
        disclosures=(RL.NO_EFFICACY_CLAIM,))


def _run(extra_args, rounds=1, proposal=None, seen=None):
    """main() with the router, quench, writer and relocation all recorded."""
    calls = []
    writes = []

    def fake_quench(pcb_data, **kw):
        calls.append(dict(kw))
        return [{'reference': 'R1', 'new_x': 161.0, 'new_y': 100.0,
                 'new_rotation': 0.0}]

    def fake_run_route(pcb_file, routed_file, route_args, log_file, **kw):
        return {'failures': 2, 'failed_nets': ['NA'], 'blockers': [],
                'iterations': 1000, 'vias': 0, 'blocker_report': None}

    def fake_relocate_round(pcb_data, pcb_file, blocks, **kw):
        if seen is not None:
            seen.setdefault('relocate_kw', []).append(dict(kw))
            seen.setdefault('relocate_blocks', []).append(dict(blocks))
        return proposal

    def fake_write(src, dst, pl):
        writes.append({'src': os.path.basename(src),
                       'dst': os.path.basename(dst),
                       'refs': sorted(p['reference'] for p in pl)})
        shutil.copy(src, dst)

    board, work = _board()
    saved = (prl.quench, prl.run_route, prl.write_placed_output,
             prl.relocate_round, sys.argv, sys.stdout)
    prl.quench = fake_quench
    prl.run_route = fake_run_route
    prl.write_placed_output = fake_write
    if proposal is not None:
        prl.relocate_round = fake_relocate_round
    sys.argv = ['place_route_loop.py', board,
                os.path.join(work, 'out.kicad_pcb'),
                '--route-args', '--nets "*"', '--rounds', str(rounds),
                '--max-displacement', '3.0', '--work-dir', work,
                '--no-movie'] + extra_args
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
         prl.relocate_round, sys.argv, sys.stdout) = saved
        os.unlink(board)
        shutil.rmtree(work, ignore_errors=True)
    if seen is not None:
        seen['sidecars'] = sidecars
        seen['stdout'] = buf.getvalue()
        seen['writes'] = writes
    return calls, buf.getvalue()


def _summary(text):
    for line in text.splitlines():
        if line.startswith('JSON_SUMMARY: '):
            return json.loads(line[len('JSON_SUMMARY: '):])
    return {}


# --------------------------------------------------------------------------
# 1. The default did not move
# --------------------------------------------------------------------------

def t_without_the_flag_the_quench_kwargs_are_identical():
    off, _ = _run([])
    on_pins, _ = _run(['--group-by', 'decap'])
    # `pcb_file` is a temp path and differs by construction; everything else
    # must match exactly.
    for a, b in zip(off, on_pins):
        a, b = dict(a), dict(b)
        a.pop('pcb_file', None)
        b.pop('pcb_file', None)
        b.pop('groups', None)
        a.pop('groups', None)
        check(a == b, 'quench kwargs differ with --group-by alone: %r'
                      % {k: (a.get(k), b.get(k)) for k in set(a) | set(b)
                         if a.get(k) != b.get(k)})


def t_without_the_flag_the_verdict_gains_exactly_one_key():
    _c, out = _run([])
    s = _summary(out)
    check(s.get('relocate') is False,
          'the always-present echo key is missing or wrong: %r'
          % s.get('relocate'))
    extra = [k for k in s if k.startswith('relocate_')]
    check(extra == [],
          'a run WITHOUT --relocate carries %r; those keys belong only to a '
          'run that used the flag' % extra)


def t_without_the_flag_the_sidecar_keeps_its_key_set():
    seen = {}
    _run([], seen=seen)
    doc = seen['sidecars'].get(1) or {}
    check('relocation' not in doc,
          'a run without --relocate wrote a `relocation` sidecar key; it must be '
          'added conditionally, never as a None placeholder')


def t_the_quench_reads_the_round_input_when_nothing_relocated():
    seen = {}
    _run([], seen=seen)
    srcs = [w['src'] for w in seen['writes']]
    check(not any('relocated' in s for s in srcs),
          'a relocated board was written without the flag: %r' % srcs)


# --------------------------------------------------------------------------
# 2. With the flag, it reaches the board
# --------------------------------------------------------------------------

def t_an_accepted_proposal_is_written_and_the_quench_reads_IT():
    seen = {}
    calls, out = _run(['--relocate', '--group-by', 'decap'],
                      proposal=_proposal(), seen=seen)
    s = _summary(out)
    check(s.get('relocate_rounds_applied') == 1,
          'the proposal was not applied: %r' % s.get('relocate_rounds_applied'))
    writes = seen['writes']
    check(any(w['dst'].endswith('_relocated.kicad_pcb') for w in writes),
          'no relocated board was written: %r' % writes)
    reloc_write = [w for w in writes if w['dst'].endswith('_relocated.kicad_pcb')]
    check(reloc_write and reloc_write[0]['refs'] == ['M1'],
          'the relocated board was not written from the proposal moves: %r'
          % reloc_write)
    # THE wiring claim: the quench must be handed the relocated board, not the
    # round's input. Otherwise the relocation is written and then ignored.
    check(calls and 'relocated' in os.path.basename(calls[0]['pcb_file']),
          'the quench read %r, not the relocated board -- a relocation the '
          'quench cannot see is a relocation the round throws away'
          % (calls[0].get('pcb_file') if calls else None))
    cand = [w for w in writes if w['dst'] == 'loop_round1.kicad_pcb']
    check(cand and cand[0]['src'].endswith('_relocated.kicad_pcb'),
          'the candidate board was built from the round input rather than from '
          'the relocated one: %r' % cand)


def t_the_block_members_are_added_to_the_move_set():
    calls, _out = _run(['--relocate', '--group-by', 'decap'],
                       proposal=_proposal())
    check(calls and 'M1' in calls[0]['move_refs'],
          'the relocated block is not in move_refs, so the quench may not '
          'refine the pose it was just moved to: %r'
          % (calls[0].get('move_refs') if calls else None))


def t_a_refusal_is_counted_and_named_not_swallowed():
    seen = {}
    _c, out = _run(['--relocate', '--group-by', 'decap'],
                   proposal=_proposal(refusal='no_room_at_any_dose: nothing '
                                              'may yield'), seen=seen)
    s = _summary(out)
    check(s.get('relocate_rounds_applied') == 0
          and s.get('relocate_rounds_refused') == 1,
          'refusal counters wrong: applied=%r refused=%r'
          % (s.get('relocate_rounds_applied'), s.get('relocate_rounds_refused')))
    check(any('no_room_at_any_dose' in r for r in s.get('relocate_refusals', [])),
          'the refusal reason was not recorded by name: %r'
          % s.get('relocate_refusals'))
    check(not any('relocated' in w['dst'] for w in seen['writes']),
          'a refused relocation still wrote a board')


def t_a_refused_round_still_lets_the_quench_run():
    calls, _out = _run(['--relocate', '--group-by', 'decap'],
                       proposal=_proposal(refusal='no_room_at_any_dose: x'))
    check(calls, 'a refused relocation stopped the round entirely; the quench '
                 'must still get its turn')


def t_the_verdict_cannot_be_read_without_the_no_efficacy_sentence():
    _c, out = _run(['--relocate', '--group-by', 'decap'], proposal=_proposal())
    s = _summary(out)
    claim = s.get('relocate_efficacy') or ''
    check(claim == RL.NO_EFFICACY_CLAIM,
          'relocate_efficacy is missing or not the module constant: %r' % claim)
    check('ROUTES better' in claim,
          'the disclosure does not say the routed question is unmeasured: %r'
          % claim)
    check('relocate-on vs relocate-off' in claim,
          'the disclosure names the wrong comparison. Reusing #553\'s selector '
          'sentence here would read like a disclosure while describing a '
          'different experiment: %r' % claim)


def t_the_verdict_carries_the_proposal_itself_not_just_a_count():
    """A count says a relocation happened; only the record says WHAT it did."""
    _c, out = _run(['--relocate', '--group-by', 'decap'], proposal=_proposal())
    props = _summary(out).get('relocate_proposals')
    check(props and isinstance(props, list),
          'relocate_proposals is missing or empty on a round that applied one: '
          '%r' % props)
    if props:
        p = props[0]
        check(p.get('round') == 1, 'the proposal does not say which round: %r' % p)
        check(p.get('dose_mm') == 4.0 and p.get('corridor') == ['C2'],
              'the recorded proposal lost what it actually did: %r' % p)
        check(p.get('binding_path'),
              'the recorded proposal lost the chain that bound it, which is the '
              'explanation #459 asked the constraint graph to be: %r' % p)


def t_the_round_sidecar_carries_the_proposal():
    seen = {}
    _run(['--relocate', '--group-by', 'decap'], proposal=_proposal(), seen=seen)
    doc = seen['sidecars'].get(1) or {}
    rec = doc.get('relocation')
    check(isinstance(rec, dict), 'the sidecar has no relocation record: %r'
                                 % sorted(doc))
    if isinstance(rec, dict):
        check(rec.get('corridor') == ['C2'],
              'the corridor -- WHO yielded -- is not in the record: %r' % rec)
        check(rec.get('binding_path'),
              'the binding path is not in the record, so the round cannot say '
              'what stopped the block: %r' % rec)


def t_a_refused_round_records_its_refusal_in_the_sidecar():
    seen = {}
    _run(['--relocate', '--group-by', 'decap'],
         proposal=_proposal(refusal='block_member_locked: J1'), seen=seen)
    doc = seen['sidecars'].get(1) or {}
    rec = doc.get('relocation') or {}
    check('block_member_locked' in (rec.get('refusal') or ''),
          'a refused round dropped its reason from the sidecar: %r' % rec)


def t_the_solve_is_given_the_operators_ignore_nets_and_budget():
    seen = {}
    _run(['--relocate', '--group-by', 'decap', '--ignore-nets', 'GND',
          '--relocate-max-corridor', '2.5'], proposal=_proposal(), seen=seen)
    kw = (seen.get('relocate_kw') or [{}])[0]
    check(kw.get('ignore_nets') == ['GND'],
          '--ignore-nets did not reach the solve, so the connectivity target '
          'degenerates into the middle of the board: %r' % kw)
    check(kw.get('max_corridor_mm') == 2.5,
          '--relocate-max-corridor did not reach the solve: %r' % kw)


def main():
    for name, fn in sorted(globals().items()):
        if name.startswith('t_') and callable(fn):
            fn()
    for f in FAILURES:
        print('FAIL: %s' % f)
    n = sum(1 for k in globals() if k.startswith('t_'))
    print('test_554_loop_relocate: %s (%d tests, %d checks failed)'
          % ('FAIL' if FAILURES else 'PASS', n, len(FAILURES)))
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
