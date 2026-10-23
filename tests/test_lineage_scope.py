#!/usr/bin/env python3
"""A routable net that no step of the final lineage scopes is REPORTED.

    python3 tests/test_lineage_scope.py

One recorded board shipped 34 nets open on every replay after #734, with
every step reporting success: its recorded chain routed GND alone in one step
and relied on that step's end-of-run reconciliation enrolling every pad-deficit
net on the board, which #734 stopped. Nothing in the grade said the CHAIN
never asked for those nets, so they were booked against the engine.
`tests/stress/lineage_scope.py` reads which nets each lineage step scopes and
`ab_replay_grade` carries the report beside every grade. A tiny synthetic
manifest + board pins what it must see:

 1. the escaped-only shape: a net whose only lineage step is a fanout escape is
    `unscoped`, named with the step that escaped it -- and a `'*'` route
    step that the pruner drops (its output feeds nothing) does not count;
 2. the scoping rules are the tools' own: route.py's '!' exclusion and
    globs, route_diff.py's '*' scoping PAIRS only, route_planes' pour
    reported as `pour_only` (since #562 a pour connects nothing), a net the
    input already carries copper on kept apart;
 3. what check_connected does not grade is not reported (an auto-named
    'unconnected-*' net, a one-pad net);
 4. the negative control: a chain ending on a `'*'` route step leaves
    nothing unscoped, so the report is not a constant;
 5. the CLI reports and exits 0 -- a report, not a gate;
 6. ab_replay_grade carries it: `lineage_scope_row` puts the names in the
    grade row, and `--compare` names the board;
 7. without swapping sys.stdout under the wave's worker threads (#383): the
    parser and the net expansion print, so the row is computed in a child.
    The control row shows the watcher DOES see the in-process call swap it.
"""
import contextlib
import io
import json
import os
import shlex
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('tests', 'tests/stress', 'py_router'):
    _q = os.path.join(ROOT, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

from run_utils import check as run_check, evidence, tool  # noqa: E402

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    if ok:
        passed += 1
        print(f"  OK   {name}" + (f" -- {detail}" if detail else ""))
    else:
        failed += 1
        print(f"  FAIL {name}" + (f" -- {detail}" if detail else ""))


NETS = ['GND', 'SIG_A', 'SIG_B', 'SIG_C', 'SIG_D', '/CLK_P', '/CLK_N',
        '/SHEET/VCC']


def _pad(num, x, net_id, net):
    return (f'  (pad "{num}" smd rect (at {x} 0) (size 1 1) (layers "F.Cu") '
            f'(net {net_id} "{net}"))\n')


def _fp(ref, at, pads):
    return (f' (footprint "t:P" (layer "F.Cu") (at {at[0]} {at[1]})\n'
            f'  (property "Reference" "{ref}" (at 0 -2) (layer "F.SilkS"))\n'
            f'{"".join(pads)} )\n')


def write_board(path):
    nets = NETS + ['unconnected-(J1-Pad1)', 'LONELY']
    decl = ''.join(f' (net {i + 1} "{n}")\n' for i, n in enumerate(nets))
    u1 = [_pad(i + 1, 2 * i, i + 1, n) for i, n in enumerate(NETS)]
    u2 = [_pad(i + 1, 2 * i, i + 1, n) for i, n in enumerate(NETS)]
    nc = len(NETS) + 1
    j1 = [_pad(1, 0, nc, nets[nc - 1]), _pad(2, 2, nc, nets[nc - 1])]
    r1 = [_pad(1, 0, nc + 1, 'LONELY')]
    txt = ('(kicad_pcb\n (version 20221018)\n (generator "test_lineage_scope")\n'
           ' (general (thickness 1.6))\n'
           ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
           ' (net 0 "")\n' + decl
           + ' (gr_rect (start 0 0) (end 60 40) (layer "Edge.Cuts") (width 0.1))\n'
           + _fp('U1', (10, 10), u1) + _fp('U2', (10, 30), u2)
           + _fp('J1', (40, 20), j1) + _fp('R1', (50, 20), r1)
           # SIG_C (net 4) arrives with a track already on the input board
           + ' (segment (start 16 10) (end 16 30) (width 0.2) (layer "F.Cu") '
             '(net 4) (uuid "c0"))\n)\n')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(txt)


def write_manifest(path, run_dir, steps):
    lines = ['#!/bin/bash', 'set -e']
    for s in steps:
        lines += [f'# cwd={shlex.quote(run_dir)}', s]
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')


def py(name, args):
    # run_utils.tool raises when the CLI is not where the reorg put it, so a
    # moved tool fails here rather than reading as an unmodelled step.
    return f'python3 -u -X utf8 {shlex.quote(tool(name))} {args}'


def chain(final_step):
    return [
        py('qfn_fanout.py', 'board.kicad_pcb -c U1 -o step1.kicad_pcb'),
        py('route_diff.py', "step1.kicad_pcb step2.kicad_pcb '*'"),
        py('route_planes.py', 'step2.kicad_pcb step3.kicad_pcb --nets GND '
                              "'/SHEET/VCC' --plane-layers B.Cu B.Cu"),
        py('route.py', 'step3.kicad_pcb step4.kicad_pcb GND SIG_A'),
        # A dead end: nothing reads step5_dead, so the pruner drops it and its
        # '*' must not count -- the recorded chain's '*' attempts were this.
        py('route.py', "step4.kicad_pcb step5_dead.kicad_pcb '*'"),
        final_step,
    ]


def names(rows):
    return sorted(r['net'] for r in rows)


def main():
    import lineage_scope as LS
    import ab_replay_grade as AB

    tmp = tempfile.mkdtemp(prefix='lineage_scope_')
    board = os.path.join(tmp, 'board.kicad_pcb')
    write_board(board)
    evidence(board, 'synthetic input board')

    man = os.path.join(tmp, 'redo_commands.sh')
    write_manifest(man, tmp, chain(py(
        'route.py', "step4.kicad_pcb final.kicad_pcb 'SIG_*' '!SIG_B' '!SIG_C'")))
    evidence(man, 'synthetic manifest')
    rep = LS.lineage_scope(man)

    print("1. the escaped-only shape: a pruned '*' does not count")
    check("the lineage is the pruned chain (command 5, step5_dead, dropped)",
          [s['index'] for s in rep['lineage']] == [1, 2, 3, 4, 6],
          f"lineage={[(s['index'], s['tool'], s['kind']) for s in rep['lineage']]}")
    un = {r['net']: r for r in rep['unscoped']}
    check("SIG_B is unscoped", 'SIG_B' in un, f"unscoped={names(rep['unscoped'])}")
    check("...named with the fanout that escaped it",
          un.get('SIG_B', {}).get('escaped_by') == ['qfn_fanout.py U1'],
          f"{un.get('SIG_B')}")
    check("...and its pad count", un.get('SIG_B', {}).get('pads') == 2)

    print("2. the tools' own scoping rules")
    check("route.py '!' excludes and globs: SIG_D is scoped, SIG_B is not",
          'SIG_D' not in un and 'SIG_B' in un)
    check("route_diff '*' scopes the PAIR, not every net",
          '/CLK_P' not in un and '/CLK_N' not in un and 'SIG_B' in un)
    check("a pour alone is pour_only (#562)",
          names(rep['pour_only']) == ['/SHEET/VCC'], f"{rep['pour_only']}")
    check("GND, poured AND routed, is scoped",
          'GND' not in un and 'GND' not in names(rep['pour_only']))
    check("input copper is kept apart",
          names(rep['unscoped_input_copper']) == ['SIG_C'],
          f"{rep['unscoped_input_copper']}")
    check("exactly SIG_B is unscoped", names(rep['unscoped']) == ['SIG_B'],
          f"{names(rep['unscoped'])}")
    check("no step went unread", rep['unmodelled'] == [], f"{rep['unmodelled']}")

    print("3. what check_connected does not grade is not reported")
    every = names(rep['unscoped'] + rep['unscoped_input_copper'] + rep['pour_only'])
    check("no 'unconnected-*' net, no one-pad net",
          not any(n.startswith('unconnected-') or n == 'LONELY' for n in every),
          f"{every}")
    check("routable = the 8 multi-pad named nets", rep['routable_nets'] == 8,
          f"routable_nets={rep['routable_nets']}")

    print("4. negative control: a chain ending on '*' leaves nothing unscoped")
    man2 = os.path.join(tmp, 'redo_all.sh')
    write_manifest(man2, tmp, chain(py(
        'route.py', "step4.kicad_pcb final.kicad_pcb '*'")))
    rep2 = LS.lineage_scope(evidence(man2, 'control manifest'))
    check("nothing unscoped, nothing pour-only",
          not rep2['unscoped'] and not rep2['pour_only']
          and not rep2['unscoped_input_copper'],
          f"{LS.compact(rep2)}")

    print("5. the CLI reports and exits 0")
    out_json = os.path.join(tmp, 'scope.json')
    r = run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'tests', 'stress', 'lineage_scope.py'),
                   man, man2, '--json', out_json], accept=True)
    check("names SIG_B with its escape", 'unscoped: SIG_B (2 pads)  escaped by '
          'qfn_fanout.py U1' in r.stdout, r.stdout[-400:])
    check("tallies 1 of 2 manifests",
          '2 manifest(s): 1 with nets their final lineage does not scope' in r.stdout)
    reps = json.load(open(evidence(out_json, 'CLI json')))
    check("json carries both reports", len(reps) == 2
          and names(reps[0]['unscoped']) == ['SIG_B'] and not reps[1]['unscoped'])

    print("6. ab_replay_grade carries it")
    row = AB.lineage_scope_row(man, board)
    check("the grade row names the nets",
          row.get('unscoped') == ['SIG_B'] and row.get('pour_only') == ['/SHEET/VCC'],
          f"{row}")
    man3 = os.path.join(tmp, 'redo_missing.sh')
    write_manifest(man3, tmp, [py('route.py', "gone.kicad_pcb out.kicad_pcb '*'")])
    gone = AB.lineage_scope_row(evidence(man3, 'missing-board manifest'))
    check("a missing input board is reported, not raised",
          'input board not found' in gone.get('error', ''), f"{gone}")
    check("an unreadable manifest is reported, not raised",
          'error' in AB.lineage_scope_row(os.path.join(tmp, 'nope.sh')))
    check("the log line names them",
          'SIG_B' in AB.lineage_scope_note(row)
          and AB.lineage_scope_note({'unscoped': []}) == '')
    old_s, new_s = os.path.join(tmp, 'old.json'), os.path.join(tmp, 'new.json')
    base = {"board": "synth", "chain_complete": True, "drc": 0, "conn": 0,
            "nets_incomplete": 1, "completion_pct": 87.5}
    for p, rows in ((old_s, [dict(base)]), (new_s, [dict(base, lineage_scope=row)])):
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(rows, fh)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        AB.compare(evidence(old_s), evidence(new_s))
    check("--compare names the board, old->new",
          'lineage scope: 1 board(s)' in buf.getvalue()
          and 'synth 0->1' in buf.getvalue(), buf.getvalue()[-300:])

    print("7. the grade row never swaps the process-global stdout (#383)")
    # Every in-process stdout redirect is counted: redirect_stdout is looked
    # up on the contextlib module at call time, by any caller.
    real_redirect, entered = contextlib.redirect_stdout, []

    class Counting(real_redirect):
        def __enter__(self):
            entered.append(1)
            return super().__enter__()
    contextlib.redirect_stdout = Counting
    try:
        AB.lineage_scope_row(man, board)
        in_row = len(entered)
        LS.lineage_scope(man, board=board)
        in_control = len(entered) - in_row
    finally:
        contextlib.redirect_stdout = real_redirect
    check("lineage_scope_row redirects nothing in this process", in_row == 0,
          f"{in_row} redirect(s)")
    check("control: the in-process report DOES redirect (the counter can see)",
          in_control > 0, f"{in_control} redirect(s)")

    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
