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
"""
import os
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


def chain_runs(a, finish=False):
    """[(chain, kind, tag, args)] of every step run_plan builds for A."""
    ch = Recorder(input_grade=G(7) if finish else None)
    orig = AP.Chain
    AP.Chain = lambda *args, **kw: ch
    try:
        AP.run_plan('IN', 'OUT', a, 'WORK', finish=finish, effort_factor=4.0)
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


def _autoplan(*argv):
    return subprocess.run([sys.executable, '-X', 'utf8', SCRIPT, *argv],
                          capture_output=True, text=True, timeout=1800)


def test_power_nets_run():
    print("--power-nets on a real run: splitflap_driver, /CLOCK_IN at 0.4")
    n = len(FAILS)
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'routed.kicad_pcb')
        r = _autoplan(BOARD, out, '--power-nets', '/CLOCK_IN', '--power-nets-widths', '0.4')
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
    assert len(FAILS) == n, FAILS[n:]


def main():
    for t in (test_defaults, test_power_nets_steps, test_power_nets_refused,
              test_power_nets_run):
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
