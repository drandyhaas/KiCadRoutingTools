#!/usr/bin/env python3
"""#865: route.py routes toward a `.kicad_dru` `via_count (max N)` rule.

KiCad's too_many_vias counts every via of a net against the last `via_count`
rule binding it. check_drc grades that (#1707); the route step never looked
at the rule and gave a net the vias its cheapest path took.

A net with a max is still routed whatever it takes. A route that would put
it over its max is searched again at 4x and 16x the via cost and the result
with the fewest vias is kept; a net that still ends over its max is named in
`JSON_SUMMARY.design_rules.via_count.over`.

The fixtures (tests/fixtures/865) are synthetic: SIG has two F.Cu pads either
side of a locked F.Cu wall of another net; under the wall, on B.Cu, the way
is 10 mm and two vias, and the rule is `via_count (max 1)` for SIG.
  * wall_open: the wall stops 10 mm short of the board's edge, so SIG can
    also go round it on F.Cu, about 44 mm and no via.
  * wall_closed: the wall runs to the edge; only the two-via route exists.

Measured with kicad-cli 10.0.8: the unfixed step routed wall_open with two
vias and KiCad reported `too_many_vias`; the route kept here has none and
KiCad is clean.

Checks:
  1. the budgets read off the rules: a positive max for the net the rule
     names, none for the other net, none for `max 0` (a via ban, honoured
     elsewhere) and none without a rule;
  2. wall_open: routed with no via, check_drc clean, the run names the rule
     and says it kept the route with fewer vias; nothing is listed as over;
  3. wall_open with the rule file emptied: routed with two vias, and
     check_drc fails that board on the rule once it is back;
  4. wall_closed: still routed, with two vias, and SIG is listed as over its
     max in the summary and on the console;
  5. the map's reference counts balance (KICAD_OBSTACLE_AUDIT).
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '865')
FAILS = []
NO_RULES = '(version 1)\n'


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru=None):
    os.makedirs(td, exist_ok=True)
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        shutil.copy(evidence(os.path.join(FIX, name + ext), 'the fixture'),
                    os.path.join(td, name + ext))
    if dru is not None:
        with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return os.path.join(td, name + '.kicad_pcb')


def route(td, board, tag, env=None):
    out = os.path.join(td, tag + '.kicad_pcb')
    summary = os.path.join(td, tag + '.json')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out,
                    '--nets', 'SIG', '--escalation', 'off', '--json-out', summary],
              accept=True, timeout=900, env=env)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
    sig = next(n for n in pcb.nets.values() if n.name == 'SIG').net_id
    return (out, doc, sum(1 for v in pcb.vias if v.net_id == sig),
            (r.stdout or '') + (r.stderr or ''))


def drc(board, flagged=False):
    argv = PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                 evidence(board, 'the routed board'), '--clearance-margin-mm', '0.0005']
    r = (check(argv, refuse="few vias", code=1) if flagged else check(argv, accept=True))
    return r.stdout or ''


def read_budgets(td):
    print("1. the budgets read off the rules")
    import contextlib
    import io
    from design_rules import install_design_rules
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    for tag, dru, want in (
            ('max 1', None, {'SIG': 1}),
            ('max 0', '(version 1)\n(rule "few vias"\n  (condition "A.NetName == \'SIG\'")\n'
                      '  (constraint via_count (max 0)))\n', {}),
            ('no rule', NO_RULES, {})):
        board = stage(os.path.join(td, 'b_' + tag.replace(' ', '_')), 'wall_open', dru=dru)
        pcb = parse_kicad_pcb(board)
        cfg = GridRouteConfig(layers=list(pcb.board_info.copper_layers))
        with contextlib.redirect_stdout(io.StringIO()):
            install_design_rules(cfg, board, pcb)
        got = {pcb.nets[n].name: mx for n, (mx, _s) in (cfg.via_budgets or {}).items()}
        ok(got == want, f"{tag}: {got}")


def main():
    with tempfile.TemporaryDirectory() as td:
        read_budgets(td)

        print("2. a way round with no via")
        sub = os.path.join(td, 'open')
        out, doc, vias, log = route(sub, stage(sub, 'wall_open'), 'out')
        ok(doc.get('routed_single') == ['SIG'] and vias == 0,
           f"SIG is routed with {vias} via(s)")
        ok('NO DRC VIOLATIONS FOUND' in drc(out), "check_drc is clean")
        ok("rule 'few vias': via_count max 1" in log and 'kept the route with 0' in log,
           "the run names the rule and says which route it kept")
        vc = (doc.get('design_rules') or {}).get('via_count') or {}
        ok(vc.get('nets') == 1 and vc.get('over') == [],
           f"the summary counts the budgeted net and lists none over: {vc}")

        print("3. the same board with the rule file emptied")
        sub = os.path.join(td, 'open_norule')
        out, doc, vias, _log = route(sub, stage(sub, 'wall_open', dru=NO_RULES), 'out')
        ok(doc.get('routed_single') == ['SIG'] and vias == 2,
           f"SIG takes the short way with {vias} via(s)")
        ok('via_count' not in (doc.get('design_rules') or {}), "no budget is in the summary")
        shutil.copy(os.path.join(FIX, 'wall_open.kicad_dru'), os.path.join(sub, 'out.kicad_dru'))
        ok('VIA-COUNT' in drc(out, flagged=True).upper(),
           "and check_drc fails that board on the rule once it is back")

        print("4. no way inside the budget")
        sub = os.path.join(td, 'closed')
        out, doc, vias, log = route(sub, stage(sub, 'wall_closed'), 'out')
        ok(doc.get('routed_single') == ['SIG'] and vias == 2,
           f"SIG is still routed, with {vias} via(s)")
        over = ((doc.get('design_rules') or {}).get('via_count') or {}).get('over') or []
        ok(len(over) == 1 and over[0].get('net') == 'SIG' and over[0].get('vias') == 2
           and over[0].get('max') == 1, f"and is listed as over its max: {over}")
        ok('SIG has 2 via(s), over its max 1' in log, "on the console too")

        print("5. the map's reference counts balance")
        sub = os.path.join(td, 'audit')
        env = dict(os.environ, KICAD_OBSTACLE_AUDIT='1')
        _o, _d, _v, log = route(sub, stage(sub, 'wall_open'), 'audit', env=env)
        ok('[OBSTACLE AUDIT]' in log and 'BALANCED' in log,
           "the working map returns exactly to the base map")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
