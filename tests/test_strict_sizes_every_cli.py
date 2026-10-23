"""--strict-sizes and the `Design rules [...]` line on every routing CLI that
registers the flag (found reviewing #1282).

`fab_tiers.add_fab_tier_args` gives route_planes, repair_planes, route_diff,
bga_fanout and qfn_fanout `--strict-sizes`, but only route.py ever acted on
it: the others printed no `Design rules` line and exited 0 however much they
had narrowed, so a harness relying on exit 3 read a narrowed run as exact.
They now end through the shared `fab_tiers.finish_design_rules`.

Per CLI, a WITNESS run that narrows and a CONTROL run that does not:

  * bga_fanout / qfn_fanout: real escalations -- glasgow U30's and U1's
    under-pad via-in-pad clamp steps the via down to the advanced floor.
    Controls: the same U30 command under --escalation off (test_1217's), and
    U1's default stub escape, which narrows nothing.
  * route_planes / repair_planes: neither narrows anything on a board small
    enough for this suite, so the witness is INJECTED: a sitecustomize
    records one ledger row right after the CLI sets its policy
    (`set_policy_from_args`, which resets the ledger). What is tested is the
    main's handling of a non-empty ledger -- the part that was missing.
    Controls: the same commands without the injection.
  * route_diff: a real witness -- lvds_converter_dualclk's /DATA pair, where
    the #318 partner neck takes two /DATA+ attach segments 0.3 -> 0.2799
    against a ~10 um graze. That neck used to be in no ledger, so this
    command was the CONTROL and exited 0 while shipping them. The control is
    now cap_chain's two pairs, which narrow nothing, and the injected witness
    runs on that command.

Witness, with --strict-sizes: exit 3, saying why (FAILS on bb5943a0: exit 0).
Witness, without it: exit 0, and the line is printed.
Control, with --strict-sizes: exit 0, and neither line appears.
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from run_utils import check, evidence, tool, tool_env  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
LINE = 'Design rules ['
STRICT = '--strict-sizes: Design rules ['

_INJECT = '''
import fab_tiers as _ft
_real = _ft.set_policy_from_args


def _with_witness(*a, **kw):
    out = _real(*a, **kw)
    _ft.note_narrowing(None, 'track_width', 0.3, 0.2, 'injected witness')
    return out


_ft.set_policy_from_args = _with_witness
'''


def _env(td, inject):
    env = tool_env()
    if inject:
        inj = os.path.join(td, 'inj')
        os.makedirs(inj, exist_ok=True)
        with open(os.path.join(inj, 'sitecustomize.py'), 'w',
                  encoding='utf-8') as f:
            f.write(_INJECT)
        env['PYTHONPATH'] = inj + os.pathsep + env['PYTHONPATH']
    return env


def _copy(name, td):
    dst = os.path.join(td, name)
    subprocess.run(PY + [tool('copy_board.py'),
                         os.path.join(ROOT, 'kicad_files', name), dst],
                   check=True, capture_output=True, cwd=ROOT)
    return evidence(dst, f'the staged {name}')


def _witness(argv, env, what, unflagged=True):
    r = check(argv + ['--strict-sizes'], code=3, refuse=STRICT, env=env)
    out = r.stdout + r.stderr
    assert out.count(LINE) >= 2, f"{what}: the line and the --strict-sizes " \
                                 f"line should both be printed\n{out[-800:]}"
    if not unflagged:
        print(f"  ok  {what}: witness exits 3 under --strict-sizes")
        return
    r = check(argv, accept=True, env=env)
    assert LINE in r.stdout, f"{what}: without --strict-sizes the Design " \
                             f"rules line must still be printed\n" \
                             f"{(r.stdout + r.stderr)[-800:]}"
    print(f"  ok  {what}: witness exits 3 under --strict-sizes, 0 and "
          f"disclosed without it")


def _control(argv, env, what):
    r = check(argv + ['--strict-sizes'], accept=True, env=env)
    out = r.stdout + r.stderr
    assert LINE not in out and '--strict-sizes:' not in out, \
        f"{what}: a run that narrowed nothing printed a Design rules line" \
        f"\n{out[-800:]}"
    print(f"  ok  {what}: control exits 0 and prints no Design rules line")


def t_fanouts(td):
    g = _copy('glasgow_revC.kicad_pcb', td)
    bga = PY + [tool('bga_fanout.py'), g, '-o', os.path.join(td, 'bga.kicad_pcb'),
                '--component', 'U30', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu',
                'B.Cu', '--track-width', '0.25', '--clearance', '0.25',
                '--via-size', '0.6', '--via-drill', '0.3', '--nets',
                '/IO_Banks/DA*', '--escape-method', 'underpad']
    # (the unflagged arm is the injected rows' job: these runs cost ~25 s)
    _witness(bga, tool_env(), 'bga_fanout (via-in-pad clamp to advanced)',
             unflagged=False)
    _control(bga + ['--escalation', 'off'], tool_env(),
             'bga_fanout --escalation off')
    qfn = PY + [tool('qfn_fanout.py'), g, '--output',
                os.path.join(td, 'qfn.kicad_pcb'), '-c', 'U1']
    _witness(qfn + ['--escape-method', 'underpad', '--allow-via-in-pad'],
             tool_env(), 'qfn_fanout (via-in-pad clamp to advanced)',
             unflagged=False)
    _control(qfn, tool_env(), 'qfn_fanout stub escape')


def t_planes_and_diff(td):
    sf = _copy('splitflap_driver.kicad_pcb', td)
    poured = os.path.join(td, 'poured.kicad_pcb')
    rp = PY + [tool('route_planes.py'), sf, poured, '--nets', 'GND',
               '--plane-layers', 'B.Cu']
    _control(rp, tool_env(), 'route_planes')
    evidence(poured, 'the poured board')
    _witness(PY + [tool('route_planes.py'), sf,
                   os.path.join(td, 'poured_w.kicad_pcb'), '--nets', 'GND',
                   '--plane-layers', 'B.Cu'],
             _env(td, True), 'route_planes (injected ledger row)')
    rep = PY + [tool('repair_planes.py'), poured,
                os.path.join(td, 'repaired.kicad_pcb')]
    _control(rep, tool_env(), 'repair_planes')
    _witness(rep, _env(td, True), 'repair_planes (injected ledger row)')
    lv = _copy('lvds_converter_dualclk.kicad_pcb', td)
    _witness(PY + [tool('route_diff.py'), lv, '--output',
                   os.path.join(td, 'diff_lv.kicad_pcb'), '--nets', '/DATA*'],
             tool_env(), 'route_diff (#318 partner neck on /DATA+)')
    cc = _copy('cap_chain.kicad_pcb', td)
    diff = PY + [tool('route_diff.py'), cc, '--output',
                 os.path.join(td, 'diff.kicad_pcb'), '--nets', 'DP*']
    _control(diff, tool_env(), 'route_diff')
    _witness(diff, _env(td, True), 'route_diff (injected ledger row)')


def main():
    cases = [t_fanouts, t_planes_and_diff]
    want = set(sys.argv[1:])
    for fn in cases:
        if want and fn.__name__ not in want:
            continue
        td = tempfile.mkdtemp(prefix='tstrict_')
        try:
            print(fn.__name__)
            fn(td)
        finally:
            shutil.rmtree(td, ignore_errors=True)
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
