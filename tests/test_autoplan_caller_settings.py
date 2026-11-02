#!/usr/bin/env python3
"""autoplan's caller settings: what each one changes, and that with none of
them autoplan runs the chain it ran before they existed.

1. Defaults: with no caller setting, the command of every step run_plan and
   route_passes build for one fixed analysis -- planes, a BGA's and a QFN's
   fanout ladders, a mezzanine, the decap step, a diff pair with a net
   rerouted, three route passes, and three under --finish -- is the one
   autoplan built at 77ffd6e9 (GOLDEN, recorded from that commit), and
   `analyze` returns the keys it returned there.
2. --power-nets with --power-nets-widths, read back through route.py's own
   selector (`identify_power_nets`) on the board: every route and reroute
   step is given the caller's patterns as typed, ahead of autoplan's rails, so
   each net they select routes at their width (a rail included), the other
   rails at RAIL_WIDTH and no other net; every other step's command is
   unchanged, and the analysis states each net's width as route.py reads it
   (`GND` selects no `/GND`; `/LED_*` every LED net). On a routed
   splitflap_driver the named net's copper is drawn at its width. A malformed
   width and a pattern count that is not the width count are refused for
   their reason, before any step runs.
3. --keep-pours: the planes are the board's own pours (read off the file
   here, not through the parser), where autoplan would pour others; the
   full chain runs no planes step and is otherwise the default chain. On a
   routed sonde_u every pour ships as the input drew it (its zone text up to
   the fill: uuid, clearance, pad connection, outline).
4. --json-out: each route pass, and no other step, runs with route.py's own
   --json-out; the document carries that file's object for the pass that
   shipped, verbatim, the step that shipped and the chain plan.json has.
   Every other outcome says so in `status`, with a null summary: a pass
   whose summary is missing, a board from a step that is no pass (the
   diff-pair reroute, the input under --finish), no board at all. Real runs:
   splitflap_driver routed (with --power-nets /CLOCK_IN at 0.4, which route.py's
   own power_widths record reads back at 0.4), --finish on that routed board
   (the input ships, no pass), and a refused run, which leaves no stale file
   behind.
5. --fab-tier, --fab-overrides, --escalation: autoplan's floors are the
   tier's (JLC's standard via 0.45/0.20 under standard, never the advanced
   0.25/0.15), the override file's values, or the board's own rules under
   board (a project written here). Each flag the caller types is recorded
   as typed, a default value included, and a relative file as its absolute
   path; a missing file is refused (exit 2) before any step, and the stale
   --json-out document is gone. Chain.run and Chain.check_drc pass them to
   exactly the tools whose own parser lists all three (read off each tool's
   --help), and to none without a caller setting. On a routed sonde_u, given
   the file by a relative path, the route step's own parser read it (its
   unknown-key warning, in that step's log), route.py's own summary reports
   the tier and policy, and every track is at least the file's width.
"""
import json
import os
import re
import subprocess
import sys
import tempfile
import traceback
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import autoplan as AP                                        # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from net_queries import identify_power_nets                  # noqa: E402
from run_utils import check, evidence                        # noqa: E402

RUN_ALL_TIMEOUT = 900
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
SHEETED = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
POURED = os.path.join(ROOT, 'kicad_files', 'kit-dev-coldfire-xilinx_5213.kicad_pcb')
SONDE = os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb')
SCRIPT = os.path.join(ROOT, 'py_router', 'autoplan.py')
FAILS = []


def expect(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILS.append(what)


class Recorder:
    """Chain.run recorded, the rest scripted: each fanout rung drops an
    escape (the whole ladder runs), the diff step leaves D_P to the route
    step and D_N in a new DRC violation (rerouted), and the route passes
    grade 5, 3 and 1 open nets (all three run)."""

    def __init__(self, input_grade=None):
        self.cur = 'IN'
        self.runs = []
        self.steps = []
        self.grades = [G(5), G(3), G(1)]
        self.input_grade = input_grade
        self.last_iterations = None

    def run(self, kind, args, tag):
        self.runs.append((kind, tag, list(args)))
        self.steps.append({'tag': tag, 'ok': True})
        self.last_iterations = 50
        s = {}
        if kind in ('bga', 'qfn'):
            s = {'escaped': 1, 'failed': 1}
        if kind == 'diff':
            s = {'single_ended_followup_nets': ['D_P']}
        return f'OUT{len(self.runs)}', s, {}

    def accept(self, out):
        if out:
            self.cur = out

    def grade(self, board):
        return self.input_grade if board == 'IN' else self.grades.pop(0)

    def drc_nets(self, board):
        diff = [i for i, r in enumerate(self.runs) if r[0] == 'diff']
        return {'D_N'} if diff and board == f'OUT{diff[0] + 1}' else set()

    def check_drc(self, board):
        return {'violations': 0, 'items': []}


def G(n):
    return {'open': n, 'drc': 0, 'open_nets': []}


#: One analysis exercising every step autoplan builds.
A = dict(layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], clearance=0.09,
         track=0.1, via=0.45, drill=0.25, fine_via=0.25, fine_drill=0.15,
         planes=[('GND', 'In1.Cu'), ('+3V3', 'In2.Cu')], plane_nets=['GND', '+3V3'],
         power_nets=['GND', '+3V3', 'VBUS'], layer_costs=[1.0, -1.0, -1.0, 1.0],
         grid=0.05, congested=True, n_nets=200, pair_nets=['D_P', 'D_N'],
         globs={'GND': 'GN[D/]', '+3V3': '+3V[3/]', 'VBUS': 'VBU[S/]',
                'D_P': 'D_[P/]', 'D_N': 'D_[N/]'},
         parts=[('U1', 'bga', 0.8), ('U2', 'qfn', 0.4), ('U3', 'mezz', 0.4)])


def chain_runs(a, finish=False, **kw):
    """[(chain, kind, tag, args)] of every step run_plan builds for A."""
    ch = Recorder(input_grade=G(7) if finish else None)
    orig = AP.Chain
    AP.Chain = lambda *args, **k: ch
    try:
        AP.run_plan('IN', 'OUT', a, 'WORK', finish=finish, effort_factor=4.0, **kw)
    finally:
        AP.Chain = orig
    return [('finish' if finish else 'full', k, t, args) for k, t, args in ch.runs]


#: Every step's command for A with no caller setting, as autoplan 77ffd6e9
#: built it (run_plan + route_passes through Recorder).
GOLDEN = [
    ('full', 'planes', 'planes',
     '{IN} {OUT} --nets GND +3V3 --plane-layers In1.Cu In2.Cu --clearance 0.09'),
    ('full', 'bga', 'fanout_U1',
     '{IN} --output {OUT} --component U1 --nets * !GN[D/] !+3V[3/] --layers F.Cu '
     'In1.Cu In2.Cu B.Cu --layer-costs 1.0 -1.0 -1.0 1.0 --track-width 0.1 '
     '--grid-step 0.05 --clearance 0.09 --clearance-ceiling 0.09 --via-size 0.45 '
     '--via-drill 0.25'),
    ('full', 'bga', 'fanout_U1_r1',
     '{IN} --output {OUT} --component U1 --nets * !GN[D/] !+3V[3/] --layers F.Cu '
     'In1.Cu In2.Cu B.Cu --layer-costs 1.0 -1.0 -1.0 1.0 --track-width 0.1 '
     '--grid-step 0.05 --clearance 0.09 --clearance-ceiling 0.09 --via-size 0.25 '
     '--via-drill 0.15'),
    ('full', 'bga', 'fanout_U1_r2',
     '{IN} --output {OUT} --component U1 --nets * !GN[D/] !+3V[3/] --layers F.Cu '
     'In1.Cu In2.Cu B.Cu --layer-costs 1.0 -1.0 -1.0 1.0 --track-width 0.1 '
     '--grid-step 0.05 --clearance 0.09 --clearance-ceiling 0.09 --via-size 0.25 '
     '--via-drill 0.15 --escape-method dogbone'),
    ('full', 'qfn', 'fanout_U2',
     '{IN} --output {OUT} --component U2 --nets * !GN[D/] !+3V[3/] --width 0.1 '
     '--grid-step 0.05 --clearance 0.09 --clearance-ceiling 0.09'),
    ('full', 'qfn', 'fanout_U2_r1',
     '{IN} --output {OUT} --component U2 --nets * !GN[D/] !+3V[3/] --width 0.1 '
     '--grid-step 0.05 --clearance 0.09 --clearance-ceiling 0.09 --escape-method '
     'underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad'),
    ('full', 'qfn', 'fanout_U3',
     '{IN} --output {OUT} --component U3 --nets * !GN[D/] !+3V[3/] --width 0.1 '
     '--grid-step 0.05 --clearance 0.09 --clearance-ceiling 0.09 --escape-method '
     'underpad --via-size 0.25 --via-drill 0.15 --allow-via-in-pad'),
    ('full', 'decap', 'decap',
     '{IN} {OUT} --clearance 0.09'),
    ('full', 'diff', 'diffpairs',
     '{IN} {OUT} --nets D_[P/] D_[N/] --track-width 0.1 --diff-pair-gap 0.1 '
     '--clearance-ceiling 0.09 --via-size 0.45 --via-drill 0.25 --layers F.Cu '
     'In1.Cu In2.Cu B.Cu --layer-costs 1.0 -1.0 -1.0 1.0 --grid-step 0.05'),
    ('full', 'route', 'reroute_pairs',
     '{IN} {OUT} --nets D_[N/] --force-reroute --no-bga-zones --clearance-ceiling '
     '0.09 --track-width 0.1 --via-size 0.45 --via-drill 0.25 --grid-step 0.05 '
     '--power-nets GN[D/] +3V[3/] VBU[S/] --power-nets-widths 0.3 0.3 0.3 '
     '--max-ripup 5 --rip-existing-nets D_[P/] --layers F.Cu In1.Cu In2.Cu B.Cu '
     '--layer-costs 1.0 -1.0 -1.0 1.0'),
    ('full', 'route', 'route',
     '{IN} {OUT} --nets * --no-bga-zones --clearance-ceiling 0.09 --track-width '
     '0.1 --via-size 0.45 --via-drill 0.25 --grid-step 0.05 --power-nets GN[D/] '
     '+3V[3/] VBU[S/] --power-nets-widths 0.3 0.3 0.3 --max-ripup 5 '
     '--rip-existing-nets D_[P/] --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs '
     '1.0 -1.0 -1.0 1.0'),
    ('full', 'route', 'route_p2',
     '{IN} {OUT} --nets * --no-bga-zones --clearance-ceiling 0.09 --track-width '
     '0.1 --via-size 0.45 --via-drill 0.25 --grid-step 0.05 --power-nets GN[D/] '
     '+3V[3/] VBU[S/] --power-nets-widths 0.3 0.3 0.3 --max-ripup 5 '
     '--rip-existing-nets D_[P/] --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs '
     '1.0 3.0 3.0 1.0'),
    ('full', 'route', 'route_p3',
     '{IN} {OUT} --nets * --no-bga-zones --clearance-ceiling 0.09 --track-width '
     '0.1 --via-size 0.45 --via-drill 0.25 --grid-step 0.05 --power-nets GN[D/] '
     '+3V[3/] VBU[S/] --power-nets-widths 0.3 0.3 0.3 --max-ripup 5 '
     '--rip-existing-nets D_[P/] --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs '
     '1.0 3.0 3.0 1.0'),
    ('finish', 'route', 'route',
     '{IN} {OUT} --nets * --no-bga-zones --clearance-ceiling 0.09 --track-width '
     '0.1 --via-size 0.45 --via-drill 0.25 --grid-step 0.05 --power-nets GN[D/] '
     '+3V[3/] VBU[S/] --power-nets-widths 0.3 0.3 0.3 --max-ripup 5 --layers F.Cu '
     'In1.Cu In2.Cu B.Cu --layer-costs 1.0 -1.0 -1.0 1.0'),
    ('finish', 'route', 'route_p2',
     '{IN} {OUT} --nets * --no-bga-zones --clearance-ceiling 0.09 --track-width '
     '0.1 --via-size 0.45 --via-drill 0.25 --grid-step 0.05 --power-nets GN[D/] '
     '+3V[3/] VBU[S/] --power-nets-widths 0.3 0.3 0.3 --max-ripup 5 --layers F.Cu '
     'In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0'),
    ('finish', 'route', 'route_p3',
     '{IN} {OUT} --nets * --no-bga-zones --clearance-ceiling 0.09 --track-width '
     '0.1 --via-size 0.45 --via-drill 0.25 --grid-step 0.05 --power-nets GN[D/] '
     '+3V[3/] VBU[S/] --power-nets-widths 0.3 0.3 0.3 --max-ripup 5 --layers F.Cu '
     'In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0'),
]

#: The keys analyze returned at 77ffd6e9.
ANALYSIS_KEYS = ['clearance', 'congested', 'drill', 'fine_drill', 'fine_via', 'globs',
                 'grid', 'layer_costs', 'layers', 'n_nets', 'pair_nets', 'parts',
                 'plane_nets', 'planes', 'power_nets', 'track', 'via']


def test_defaults():
    print("no caller setting: the chain autoplan ran before the settings existed")
    n = len(FAILS)
    got = [(c, k, t, ' '.join(args)) for c, k, t, args in chain_runs(dict(A))
           + chain_runs(dict(A), finish=True)]
    expect(len(got) == len(GOLDEN), f"{len(GOLDEN)} steps ({len(got)})")
    for want, have in zip(GOLDEN, got):
        expect(have == want, f"{want[0]} {want[2]}: the command is unchanged"
               + ('' if have == want else f"\n         got  {have[3]}\n         want {want[3]}"))
    for board in (BOARD, SHEETED):
        for finish in (False, True):
            keys = sorted(AP.analyze(board, finish=finish))
            expect(keys == ANALYSIS_KEYS, f"{os.path.basename(board)} finish={finish}: "
                                          f"analyze adds no key ({keys})")
    assert len(FAILS) == n, FAILS[n:]


def _stub_board(names):
    nets = {i + 1: SimpleNamespace(name=nm) for i, nm in enumerate(names)}
    return SimpleNamespace(nets=nets)


def _values(args, flag):
    """The values after FLAG in ARGS, up to the next option."""
    if flag not in args:
        return []
    out = []
    for x in args[args.index(flag) + 1:]:
        if x.startswith('--'):
            break
        out.append(x)
    return out


def routed_widths(pcb, args):
    """{net name: width} route.py's own selector reads off a route step's
    --power-nets and --power-nets-widths, on PCB."""
    pats = _values(args, '--power-nets')
    widths = [float(w) for w in _values(args, '--power-nets-widths')]
    if len(pats) != len(widths):
        return None
    ids = identify_power_nets(pcb, pats, widths)
    return {pcb.nets[i].name: w for i, w in ids.items()}


def test_power_nets_steps():
    print("--power-nets: every route and reroute step, read by route.py's selector")
    n = len(FAILS)
    names = ['GND', '+3V3', 'VBUS', 'VBUS2', '/P/VBUS', 'SIG', 'SIGN', '/S/SIG', 'D_P', 'D_N']
    stub = _stub_board(names)
    a = dict(A, caller_power_nets=[['VBUS', 0.6], ['SIG*', 0.5]])
    runs = chain_runs(a) + chain_runs(a, finish=True)
    route = [(c, t, args) for c, k, t, args in runs if k == 'route']
    expect({t for _, t, _ in route} == {'reroute_pairs', 'route', 'route_p2', 'route_p3'},
           f"the reroute and three passes in each chain ({sorted({t for _, t, _ in route})})")
    want = {'GND': AP.RAIL_WIDTH, '+3V3': AP.RAIL_WIDTH, 'VBUS': 0.6, 'SIG': 0.5, 'SIGN': 0.5}
    for c, t, args in route:
        got = routed_widths(stub, args)
        expect(got == want, f"{c} {t}: VBUS over its rail width, SIG* at 0.5, the other "
                            f"rails at {AP.RAIL_WIDTH}, nothing else ({got})")
        pats = _values(args, '--power-nets')
        expect(pats[:2] == ['VBUS', 'SIG*'],
               f"{c} {t}: the caller's patterns as typed, ahead of the rails ({pats})")
    others = [(c, k, t, ' '.join(args)) for c, k, t, args in runs if k != 'route']
    expect(others == [g for g in GOLDEN if g[1] != 'route'],
           "every step that is not a route step is unchanged")

    print("--power-nets on splitflap_driver: the analysis states route.py's reading")
    pcb = parse_kicad_pcb(BOARD)
    b = AP.analyze(BOARD, caller_power=[('+12V', 0.6), ('/CLOCK_IN', 0.4)])
    expect(b['net_widths'] == {'+12V': 0.6, '/CLOCK_IN': 0.4}
           and b['caller_power_nets'] == [['+12V', 0.6], ['/CLOCK_IN', 0.4]],
           f"the analysis carries the caller's patterns and widths ({b.get('net_widths')})")
    ch = Recorder()
    AP.route_passes(ch, b, b['layers'], [str(x) for x in b['layer_costs']], 4.0)
    want = {n: AP.RAIL_WIDTH for n in b['power_nets']}
    want.update({'+12V': 0.6, '/CLOCK_IN': 0.4})
    for kind, tag, args in ch.runs:
        got = routed_widths(pcb, args)
        expect(got == want, f"{tag}: +12V at 0.6 over the rail width, /CLOCK_IN at 0.4, "
                            f"{sorted(set(b['power_nets']) - {'+12V'})} at the rail width ({got})")
    led = AP.analyze(BOARD, caller_power=[('/LED_*', 0.25)])['net_widths']
    expect(led == {f'/LED_{x}': 0.25 for x in 'ABCDEF'},
           f"a wildcard gives each net it selects its width ({led})")
    assert len(FAILS) == n, FAILS[n:]


def test_power_nets_refused():
    print("--power-nets refusals, each for its reason, and route.py's reading of a name")
    n = len(FAILS)
    for bad in ('wide', '0', '-0.4', 'nan', 'inf'):
        check([sys.executable, '-X', 'utf8', SCRIPT, BOARD, os.devnull + '.kicad_pcb',
               '--dry-run', '--power-nets', '/CLOCK_IN', '--power-nets-widths', bad],
              refuse='must be a finite width > 0', code=2, allow=('error: argument',))
        expect(True, f"{bad!r} refused as malformed")
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'routed.kicad_pcb')
        for flags, why in (
                (('--power-nets', '/CLOCK_IN', '+12V', '--power-nets-widths', '0.4'),
                 '--power-nets gives 2 pattern(s) and --power-nets-widths 1 width(s)'),
                (('--power-nets', '/CLOCK_IN'),
                 '--power-nets gives 1 pattern(s) and --power-nets-widths 0 width(s)'),
                (('--power-nets-widths', '0.4'),
                 '--power-nets gives 0 pattern(s) and --power-nets-widths 1 width(s)')):
            check([sys.executable, '-X', 'utf8', SCRIPT, BOARD, out, *flags],
                  refuse=f'AUTOPLAN: refused: {why}', code=2)
            expect(not os.path.exists(out + '.autoplan') and not os.path.exists(out),
                   f"{why}: refused before any step runs (no work directory, no board)")
    for pattern, want in (('GND', '{}'), ('/GND', '{"/GND": 0.4}')):
        r = check([sys.executable, '-X', 'utf8', SCRIPT, SHEETED, os.devnull + '.kicad_pcb',
                   '--dry-run', '--power-nets', pattern, '--power-nets-widths', '0.4'],
                  accept=True)
        expect(f'"net_widths": {want}' in r.stdout,
               f"{pattern!r} on a board whose net is '/GND': net_widths {want}, as "
               f"route.py's fnmatch reads it")
    assert len(FAILS) == n, FAILS[n:]


def _autoplan(*argv, cwd=None):
    return subprocess.run([sys.executable, '-X', 'utf8', SCRIPT, *argv],
                          capture_output=True, text=True, timeout=1800, cwd=cwd)


def test_run():
    print("a real run: splitflap_driver, --power-nets /CLOCK_IN at 0.4, --json-out")
    n = len(FAILS)
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'routed.kicad_pcb')
        js = os.path.join(tmp, 'autoplan.json')
        r = _autoplan(BOARD, out, '--power-nets', '/CLOCK_IN', '--power-nets-widths', '0.4',
                      '--json-out', js)
        expect(r.returncode == 0, f"autoplan exits 0 (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            raise AssertionError('autoplan failed')
        evidence(out, 'routed board')
        pcb = parse_kicad_pcb(out)
        nid = next(i for i, x in pcb.nets.items() if x.name == '/CLOCK_IN')
        segs = [s for s in pcb.segments if s.net_id == nid]
        length = lambda ss: sum(((s.end_x - s.start_x) ** 2 + (s.end_y - s.start_y) ** 2) ** 0.5
                                for s in ss)
        wide = length([s for s in segs if abs(s.width - 0.4) < 1e-6])
        expect(segs and wide >= 0.9 * length(segs),
               f"/CLOCK_IN copper is drawn at 0.4 ({wide:.1f} of {length(segs):.1f} mm)")
        expect(all(s.width <= 0.4 + 1e-6 for s in segs), "and none of it wider")

        doc = json.load(open(evidence(js, '--json-out document')))
        plan = json.load(open(out + '.autoplan/plan.json'))
        shipped = [s for s in plan['steps']
                   if s.get('log') == os.path.basename(plan['final'])[:-len('.kicad_pcb')] + '.log']
        expect(len(shipped) == 1 and doc['status'] == 'route_pass'
               and doc['shipped_step'] == shipped[0]['tag']
               and doc['shipped_board'] == os.path.basename(plan['final']),
               f"a route pass shipped, and the document names it "
               f"({doc['status']}, {doc['shipped_step']}, {doc['shipped_board']})")
        argv = shipped[0]['argv'] if shipped else []
        mine = argv[argv.index('--json-out') + 1] if '--json-out' in argv else None
        expect(mine and doc['route_summary'] == json.load(open(evidence(mine, 'route.py summary'))),
               "its summary is the object route.py's own --json-out wrote for that pass")
        pw = ((doc['route_summary'] or {}).get('power_widths') or {}).get('/CLOCK_IN') or {}
        expect(pw.get('requested_mm') == 0.4,
               f"route.py's own record: /CLOCK_IN requested at 0.4 ({pw.get('requested_mm')})")
        expect(all(doc[k] == plan[k] for k in ('analysis', 'steps', 'final', 'grade')),
               "the chain and grade are plan.json's")
        expect([s['tag'] for s in plan['steps'] if '--json-out' in s['argv']]
               == [s['tag'] for s in plan['steps']
                   if s['tool'] == 'route' and s['tag'] != 'reroute_pairs'],
               "every route pass ran with --json-out")

        print("--finish on the routed board: the input ships, and the document says no pass did")
        fin = os.path.join(tmp, 'finished.kicad_pcb')
        js2 = os.path.join(tmp, 'finished.json')
        r = _autoplan(out, fin, '--finish', '--json-out', js2)
        expect(r.returncode == 0, f"--finish exits 0 (rc={r.returncode})")
        doc = json.load(open(evidence(js2, '--json-out document')))
        expect(doc['status'] == 'no_route_pass' and doc['route_summary'] is None
               and doc['shipped_step'] == 'input' and 'input' in doc.get('why', ''),
               f"status no_route_pass, the input named, no summary ({doc['status']}, "
               f"{doc['shipped_step']}, {doc.get('why')})")

        print("a refused run leaves no stale document")
        open(js, 'w').write('{"status": "route_pass"}')
        check([sys.executable, '-X', 'utf8', SCRIPT, BOARD, out, '--json-out', js,
               '--power-nets', 'NOPE'], refuse='give one width per pattern', code=2)
        expect(not os.path.exists(js), "the earlier document is gone")
    assert len(FAILS) == n, FAILS[n:]


class Steps:
    """A finished chain's steps, as Chain records them."""

    def __init__(self, steps):
        self.steps = steps


def test_shipped_record():
    print("--json-out: each outcome, said in its status")
    n = len(FAILS)
    with tempfile.TemporaryDirectory() as tmp:
        def step(i, tool, tag, summary=None):
            stem = os.path.join(tmp, f'step{i:02d}_{tag}')
            if summary is not None:
                json.dump(summary, open(stem + '.summary.json', 'w'))
            return {'tool': tool, 'tag': tag, 'ok': True, 'log': os.path.basename(stem) + '.log'}
        board = lambda i, tag: os.path.join(tmp, f'step{i:02d}_{tag}.kicad_pcb')
        summary = {'successful': 12, 'failed': 1, 'power_widths': {'GND': {'requested_mm': 0.3}}}
        ch = Steps([step(1, 'planes', 'planes'), step(2, 'route', 'reroute_pairs'),
                    step(3, 'route', 'route', {'successful': 9}), step(4, 'route', 'route_p2', summary),
                    step(5, 'route', 'route_p3')])
        d = AP.shipped_record(ch, board(4, 'route_p2'))
        expect(d['status'] == 'route_pass' and d['route_summary'] == summary
               and d['shipped_step'] == 'route_p2' and 'why' not in d,
               f"the shipped pass's own summary, verbatim ({d})")
        d = AP.shipped_record(ch, board(5, 'route_p3'))
        expect(d['status'] == 'route_pass_without_summary' and d['route_summary'] is None
               and 'route_p3' in d['why'], f"a shipped pass with no summary says so ({d})")
        d = AP.shipped_record(ch, board(2, 'reroute_pairs'))
        expect(d['status'] == 'no_route_pass' and d['route_summary'] is None
               and 'from reroute_pairs' in d['why'],
               f"the reroute is a route step but no pass ({d})")
        d = AP.shipped_record(ch, board(1, 'planes'))
        expect(d['status'] == 'no_route_pass' and d['shipped_step'] == 'planes',
               f"the last step that succeeded, named ({d})")
        d = AP.shipped_record(ch, os.path.join(tmp, 'input.kicad_pcb'))
        expect(d['status'] == 'no_route_pass' and d['shipped_step'] == 'input'
               and d['route_summary'] is None, f"the input under --finish ({d})")
        d = AP.shipped_record(ch, None)
        expect(d['status'] == 'no_board' and d['shipped_board'] is None
               and d['route_summary'] is None, f"no board at all ({d})")

    print("--json-out: route pass commands")
    runs = chain_runs(dict(A), route_json=True) + chain_runs(dict(A), finish=True, route_json=True)
    for c, k, t, args in runs:
        flagged = '--json-out' in args
        want = k == 'route' and t != 'reroute_pairs'
        expect(flagged == want and (not flagged or args[args.index('--json-out') + 1] == '{JSON}'),
               f"{c} {t}: {'--json-out {JSON}' if want else 'no --json-out'}")
    plain = [(c, k, t, ' '.join(a for i, a in enumerate(args)
                                if '--json-out' not in args[max(i - 1, 0):i + 1]))
             for c, k, t, args in runs]
    expect(plain == GOLDEN, "and every command is otherwise the default one")
    assert len(FAILS) == n, FAILS[n:]


def zone_blocks(path):
    """Each board-level (zone ...) block of PATH up to its fill, whitespace
    folded: what the board says about a pour, not what a fill made of it."""
    text = open(path, encoding='utf-8').read()
    out = []
    depth, quoted, start = 0, False, None
    for i, c in enumerate(text):
        if quoted:
            quoted = not (c == '"' and text[i - 1] != '\\')
        elif c == '"':
            quoted = True
        elif c == '(':
            depth += 1
            if depth == 2 and text.startswith('(zone', i) and not text[i + 5].isalnum():
                start = i
        elif c == ')':
            depth -= 1
            if depth == 1 and start is not None:
                block = text[start:i + 1]
                cut = block.find('(filled_polygon')
                out.append(' '.join((block if cut < 0 else block[:cut]).split()))
                start = None
    return out


def pours(path):
    """{(net, layer)} of PATH's copper pours, read off the zone blocks: a zone
    with a net name, not a keep-out, one layer per entry."""
    out = set()
    for b in zone_blocks(path):
        net = re.search(r'\(net_name "([^"]*)"\)', b) or re.search(r'\(net "([^"]*)"\)', b)
        if not net or not net.group(1) or '(keepout' in b:
            continue
        lay = re.search(r'\(layers? ((?:"[^"]*" ?)+)\)', b)
        out |= {(net.group(1), l) for l in re.findall(r'"([^"]*)"', lay.group(1))}
    return out


def test_keep_pours_plan():
    print("--keep-pours: the board's pours are the planes, and nothing is poured")
    n = len(FAILS)
    full = AP.analyze(POURED)
    kept = AP.analyze(POURED, keep_pours=True)
    want = pours(POURED)
    expect(set(full['planes']) != want,
           f"the fixture: autoplan would pour {full['planes']}, not the board's {sorted(want)}")
    expect(set(kept['planes']) == want, f"the planes are the board's pours ({kept['planes']})")
    expect(kept['planes'] == AP.analyze(POURED, finish=True)['planes'],
           "the same planes --finish reads")
    inner = {l for _, l in want} - {kept['layers'][0], kept['layers'][-1]}
    expect([c < 0 for c in kept['layer_costs']] == [l in inner for l in kept['layers']],
           f"signals kept off the inner layers the pours cover ({kept['layer_costs']})")
    a = dict(A, keep_pours=True)
    runs = chain_runs(a)
    expect('planes' not in [k for _, k, _, _ in runs], "no planes step runs")
    expect([(c, k, t, ' '.join(args)) for c, k, t, args in runs]
           == [g for g in GOLDEN if g[0] == 'full' and g[1] != 'planes'],
           "every other step is the default chain's")
    assert len(FAILS) == n, FAILS[n:]


def test_keep_pours_run():
    print("--keep-pours on a real run: sonde_u's GND pour ships as drawn")
    n = len(FAILS)
    before = zone_blocks(SONDE)
    expect(len(before) == 1 and 'clearance 0.508' in before[0],
           "the fixture: one pour, at 0.508 clearance")
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'routed.kicad_pcb')
        r = _autoplan(SONDE, out, '--keep-pours')
        expect(r.returncode == 0, f"autoplan exits 0 (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            raise AssertionError('autoplan failed')
        evidence(out, 'routed board')
        plan = json.load(open(out + '.autoplan/plan.json'))
        expect([s['tag'] for s in plan['steps']][:1] == ['route'],
               f"the chain starts at the route step ({[s['tag'] for s in plan['steps']]})")
        expect(zone_blocks(out) == before,
               "the shipped pour is the input's: uuid, clearance, pad connection, outline")
    assert len(FAILS) == n, FAILS[n:]


FAB_FLAGS = ('--fab-tier', '--fab-overrides', '--escalation')


def _analysis(board, *argv, cwd=None):
    r = _autoplan(board, os.devnull + '.kicad_pcb', '--dry-run', *argv, cwd=cwd)
    m = re.search(r'^AUTOPLAN_ANALYSIS: (\{.*\})$', r.stdout, re.M)
    return json.loads(m.group(1)) if r.returncode == 0 and m else None


def _floors(a):
    return a and {k: a[k] for k in ('clearance', 'track', 'via', 'drill',
                                    'fine_via', 'fine_drill')}


def test_fab_floors():
    print("--fab-tier / --fab-overrides / --escalation: autoplan's own floors")
    n = len(FAILS)
    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'b.kicad_pcb')
        with open(POURED) as src, open(board, 'w') as dst:
            dst.write(src.read())
        json.dump({'board': {'design_settings': {'rules': {
            'min_clearance': 0.2, 'min_track_width': 0.25}}}},
            open(os.path.join(tmp, 'b.kicad_pro'), 'w'))
        floor = os.path.join(tmp, 'floor.txt')
        open(floor, 'w').write('clearance = 0.2\ntrack_width = 0.2\n'
                               'via_diameter = 0.6\nvia_drill = 0.3\n')

        a = _analysis(board)
        expect(a and (a['fine_via'], a['fine_drill']) == (0.25, 0.15)
               and a['clearance'] < 0.2 and 'fab_args' not in a,
               f"the fixture: by default the fine via is the advanced 0.25/0.15 and "
               f"the clearance under the board's 0.2 ({_floors(a)})")
        a = _analysis(board, '--fab-tier', 'standard')
        expect(a and min(a['via'], a['fine_via']) >= 0.45
               and min(a['drill'], a['fine_drill']) >= 0.2,
               f"--fab-tier standard: no via under JLC's standard 0.45/0.20 ({_floors(a)})")
        a = _analysis(board, '--fab-overrides', floor)
        expect(a and (a['clearance'], a['track'], a['fine_via'], a['fine_drill'])
               == (0.2, 0.2, 0.6, 0.3) and a['via'] >= 0.6 and a['drill'] >= 0.3,
               f"--fab-overrides: the file's clearance, track and via ({_floors(a)})")
        a = _analysis('b.kicad_pcb', '--fab-overrides', 'floor.txt', cwd=tmp)
        expect(a and a.get('fab_args') == ['--fab-overrides', os.path.realpath(floor)],
               f"a relative file is recorded as its absolute path ({a and a.get('fab_args')})")
        a = _analysis(board, '--fab-tier', 'auto', '--escalation', 'fab')
        expect(a and a.get('fab_args') == ['--fab-tier', 'auto', '--escalation', 'fab'],
               f"a flag given at its default value is recorded ({a and a.get('fab_args')})")
        a = _analysis(board, '--escalation', 'board')
        expect(a and (a['clearance'], a['track']) == (0.2, 0.25),
               f"--escalation board: the board's own min_clearance and "
               f"min_track_width ({_floors(a)})")
        out = os.path.join(tmp, 'o.kicad_pcb')
        js = os.path.join(tmp, 'o.json')
        open(js, 'w').write('{"status": "route_pass"}')
        check([sys.executable, '-X', 'utf8', SCRIPT, board, out, '--json-out', js,
               '--fab-overrides', os.path.join(tmp, 'missing.txt')],
              refuse='missing.txt is no file', code=2)
        expect(not os.path.exists(out + '.autoplan') and not os.path.exists(js),
               "a missing file: no step ran, and the earlier document is gone")
    assert len(FAILS) == n, FAILS[n:]


def _takes_fab_flags(path):
    r = subprocess.run([sys.executable, '-X', 'utf8', path, '--help'],
                       capture_output=True, text=True, timeout=300)
    return r.returncode == 0 and all(f in r.stdout for f in FAB_FLAGS)


def test_fab_steps():
    print("the fab flags reach the steps whose own parser takes them, and only those")
    n = len(FAILS)
    takes = {k: _takes_fab_flags(p) for k, p in AP.TOOLS.items()}
    expect(sum(takes.values()) >= 4 and not all(takes.values()),
           f"the fixture: some step tools take the flags and some do not ({takes})")
    flags = ['--fab-tier', 'standard', '--escalation', 'off', '--fab-overrides', '/F']
    with tempfile.TemporaryDirectory() as tmp:
        stub = os.path.join(tmp, 'stub.py')
        open(stub, 'w').write('raise SystemExit(0)\n')
        orig = dict(AP.TOOLS)
        AP.TOOLS.update({k: stub for k in AP.TOOLS})
        try:
            for fab in (flags, []):
                ch = AP.Chain(os.path.join(tmp, 'in.kicad_pcb'), tmp, fab_args=fab)
                for kind in orig:
                    ch.run(kind, ['{IN}', '--x'], kind)
                for kind, step in zip(orig, ch.steps):
                    want = [ch.cur, '--x'] + (fab if takes[kind] else [])
                    expect(step['argv'][0] == stub and step['argv'][1:] == want,
                           f"{kind} ({'takes' if takes[kind] else 'does not take'} them), "
                           f"{'with' if fab else 'without'} a caller setting: "
                           f"{' '.join(step['argv'][2:]) or 'nothing'} added")
        finally:
            AP.TOOLS.clear()
            AP.TOOLS.update(orig)

        drc = os.path.join(AP.HERE, 'check_drc.py')
        takes_drc = _takes_fab_flags(drc)
        expect(takes_drc, "the fixture: check_drc's own parser takes them")
        log = os.path.join(tmp, 'drc_argv.json')
        open(os.path.join(tmp, 'check_drc.py'), 'w').write(
            'import json, sys\n'
            f'json.dump(sys.argv[1:], open({log!r}, "w"))\n'
            'json.dump({"violations": 0, "items": []}, '
            'open(sys.argv[sys.argv.index("--json") + 1], "w"))\n')
        here = AP.HERE
        AP.HERE = tmp
        try:
            for fab in (flags, []):
                ch = AP.Chain(os.path.join(tmp, 'in.kicad_pcb'), tmp, fab_args=fab)
                d = ch.check_drc(os.path.join(tmp, 'in.kicad_pcb'))
                argv = json.load(open(log))
                expect(d is not None and (argv[-len(fab):] == fab if fab
                                          else not set(FAB_FLAGS) & set(argv)),
                       f"the grading check_drc, {'with' if fab else 'without'} a caller "
                       f"setting: {' '.join(argv[argv.index('--json') + 2:]) or 'nothing'} added")
        finally:
            AP.HERE = here
    assert len(FAILS) == n, FAILS[n:]


def test_fab_run():
    print("a real run: sonde_u under --fab-tier standard --escalation off --fab-overrides")
    n = len(FAILS)
    with tempfile.TemporaryDirectory() as tmp:
        floor = os.path.join(tmp, 'floor.txt')
        open(floor, 'w').write('clearance = 0.2\ntrack_width = 0.2\nnot_a_floor_key = 1\n')
        out = os.path.join(tmp, 'routed.kicad_pcb')
        js = os.path.join(tmp, 'autoplan.json')
        r = _autoplan(SONDE, out, '--keep-pours', '--json-out', js, '--fab-tier', 'standard',
                      '--escalation', 'off', '--fab-overrides', 'floor.txt', cwd=tmp)
        expect(r.returncode == 0, f"autoplan exits 0 (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            raise AssertionError('autoplan failed')
        doc = json.load(open(evidence(js, '--json-out document')))
        route = [s for s in doc['steps'] if s['tool'] == 'route']
        warned = [s['tag'] for s in route if "not_a_floor_key" in
                  open(os.path.join(out + '.autoplan', s['log'])).read()]
        expect(route and warned == [s['tag'] for s in route],
               f"each route step's own parser read the file ({warned})")
        rules = (doc['route_summary'] or {}).get('design_rules') or {}
        expect((rules.get('fab_tier'), rules.get('escalation_policy')) == ('standard', 'off'),
               f"route.py's own summary: tier standard, policy off "
               f"({rules.get('fab_tier')}, {rules.get('escalation_policy')})")
        widths = sorted({s.width for s in parse_kicad_pcb(out).segments})
        expect(widths and widths[0] >= 0.2 - 1e-6,
               f"every track is at least the file's 0.2 ({widths[:4]})")
    assert len(FAILS) == n, FAILS[n:]


def main():
    for t in (test_defaults, test_power_nets_steps, test_power_nets_refused,
              test_keep_pours_plan, test_keep_pours_run, test_shipped_record, test_run,
              test_fab_floors, test_fab_steps, test_fab_run):
        before = len(FAILS)
        try:
            t()
        except AssertionError as exc:
            if len(FAILS) == before:     # a check() refusal, not an expect
                print(f"  FAIL {t.__name__}: {exc}")
                FAILS.append(f"{t.__name__}: {exc}")
        except Exception:                                        # noqa: BLE001
            traceback.print_exc()
            FAILS.append(f"{t.__name__} raised")
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS: autoplan caller settings")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
