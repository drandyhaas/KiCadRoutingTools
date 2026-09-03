#!/usr/bin/env python3
"""#530 replay knob: KICAD_CLEARANCE_LEGACY_CEILING=1 makes --clearance read
the pre-#530 way (a ceiling capping every class), so a corpus manifest
recorded under that reading replays like-for-like. It exists for
cloud_replay_sets --env arms, never for a real run.

Harness board: net P1 in class 'power' (clearance 0.3, pattern 'P*'), net A
in Default. route.py --clearance 0.15:

  knob unset -> the run says the Default class routes at 0.15 and the power
                class is "honored in full" at 0.3
  knob = 1   -> the run says every class is capped at 0.15
                (the same lines --clearance-ceiling 0.15 prints)
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))
import constraint_agreement as ca  # noqa: E402


def _route(board, out, env_extra):
    env = dict(os.environ)
    env.pop('KICAD_CLEARANCE_LEGACY_CEILING', None)
    env.update(env_extra)
    return subprocess.run([sys.executable, '-X', 'utf8',
                           os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                           '--nets', 'A', 'P1', '--clearance', '0.15'],
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, env=env)


def main():
    fails = []
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'k.kicad_pcb')
        fps = [{'ref': 'U1', 'x': 10, 'y': 10, 'net_id': 1, 'net_name': 'A', 'layer': 'F.Cu'},
               {'ref': 'U2', 'x': 20, 'y': 10, 'net_id': 1, 'net_name': 'A', 'layer': 'F.Cu'},
               {'ref': 'U3', 'x': 10, 'y': 20, 'net_id': 3, 'net_name': 'P1', 'layer': 'F.Cu'},
               {'ref': 'U4', 'x': 20, 'y': 20, 'net_id': 3, 'net_name': 'P1', 'layer': 'F.Cu'}]
        saved = dict(ca.NETS)
        ca.NETS[3] = 'P1'
        try:
            ca.write_board(board, footprints=fps,
                           classes=[{'name': 'power', 'clearance': 0.3, 'track_width': 0.3,
                                     'via_diameter': 0.6, 'via_drill': 0.3, 'priority': 0}],
                           patterns=[('P*', 'power')])
        finally:
            ca.NETS.clear()
            ca.NETS.update(saved)

        r = _route(board, os.path.join(td, 'o1.kicad_pcb'), {})
        if r.returncode != 0:
            fails.append(f"plain run exited {r.returncode}:\n{r.stdout[-1500:]}\n{r.stderr[-500:]}")
        if 'the Default net class routes at it this run' not in r.stdout:
            fails.append("plain run did not announce the Default-class reading of --clearance")
        if 'honored in full' not in r.stdout or '[0.3]' not in r.stdout:
            fails.append("plain run did not honour the power class at 0.3")
        if 'capped at --clearance-ceiling' in r.stdout:
            fails.append("plain run capped the classes without the knob")

        r2 = _route(board, os.path.join(td, 'o2.kicad_pcb'),
                    {'KICAD_CLEARANCE_LEGACY_CEILING': '1'})
        if r2.returncode != 0:
            fails.append(f"knob run exited {r2.returncode}:\n{r2.stdout[-1500:]}\n{r2.stderr[-500:]}")
        if '--clearance-ceiling 0.15' not in r2.stdout:
            fails.append("knob run did not turn --clearance into the ceiling")
        if 'capped at --clearance-ceiling 0.15' not in r2.stdout:
            fails.append("knob run did not cap the power class at 0.15")
        if 'KICAD_CLEARANCE_LEGACY_CEILING' not in r2.stdout:
            fails.append("knob run did not disclose the knob in its ENV KNOBS line")

        # The pre-#530 reading also lowered the RUN clearance to min(Default
        # class, ceiling): a project whose Default class an earlier step lowered
        # to 0.1 routed at 0.1 under --clearance 0.2 (watchy, rp2040_dev). Under
        # the knob the run must do the same; without it, --clearance IS the
        # Default class for the run (decision 2) and the run stays at 0.2.
        low = os.path.join(td, 'low.kicad_pcb')
        saved_dc = dict(ca.DEFAULT_CLASS)
        ca.DEFAULT_CLASS['clearance'] = 0.1
        ca.NETS[3] = 'P1'
        try:
            ca.write_board(low, footprints=fps,
                           classes=[{'name': 'power', 'clearance': 0.3, 'track_width': 0.3,
                                     'via_diameter': 0.6, 'via_drill': 0.3, 'priority': 0}],
                           patterns=[('P*', 'power')])
        finally:
            ca.DEFAULT_CLASS.clear()
            ca.DEFAULT_CLASS.update(saved_dc)
            ca.NETS.clear()
            ca.NETS.update(saved)

        def _run_low(out, env_extra):
            env = dict(os.environ)
            env.pop('KICAD_CLEARANCE_LEGACY_CEILING', None)
            env.update(env_extra)
            return subprocess.run([sys.executable, '-X', 'utf8',
                                   os.path.join(ROOT, 'py_router', 'route.py'), low, out,
                                   '--nets', 'A', 'P1', '--clearance', '0.2'],
                                  capture_output=True, text=True, encoding='utf-8',
                                  errors='replace', cwd=ROOT, env=env)

        r3 = _run_low(os.path.join(td, 'o3.kicad_pcb'), {'KICAD_CLEARANCE_LEGACY_CEILING': '1'})
        if '"min_clearance_used": 0.1' not in r3.stdout:
            fails.append("knob run on a Default-0.1 project did not route at min(Default, ceiling) = 0.1")
        r4 = _run_low(os.path.join(td, 'o4.kicad_pcb'), {})
        if '"min_clearance_used": 0.2' not in r4.stdout:
            fails.append("plain run on a Default-0.1 project did not route at the explicit 0.2")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: KICAD_CLEARANCE_LEGACY_CEILING=1 reads --clearance as the pre-#530 "
          "every-class ceiling and says so; unset, --clearance is the Default class only")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
