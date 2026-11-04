"""#1495: a split plane layer's quoted `|` group is its nets, for every reader.

route_planes takes `--nets GND '+3V3|+5V' --plane-layers In1.Cu In2.Cu`
(the documented split-layer form). route_plan_check refused it -- R02 read
the quoted `|` as a shell pipe, R14 compared `'+3V3|+5V'` with the
`--power-nets` list -- and manifest_to_plan handed the GUI a net called
`+3V3|+5V`, which the plane tab dropped. Every reader now splits the group
through `plane_groups.split_plane_group`.

    python3 tests/test_1495_split_plane_group.py
"""
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'tests', 'stress'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

CHECK = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-routing', 'scripts',
                     'route_plan_check.py')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def plan(td, name, pour, power):
    out = os.path.join(td, 'final.kicad_pcb')
    text = (f"#!/bin/bash\nset -e\ncd {REPO}\n"
            f"python3 -u -X utf8 py_router/route_planes.py kicad_files/tigard.kicad_pcb "
            f"{td}/s1.kicad_pcb {pour}\n"
            f"python3 -u -X utf8 py_router/route.py {td}/s1.kicad_pcb {out} --nets '*' {power}\n"
            f"# Verification -- comments:\n"
            f"#   python3 -X utf8 py_router/check_drc.py {out}\n"
            f"#   python3 -X utf8 py_router/check_connected.py {out}\n"
            f"#   python3 -X utf8 py_tools/check_orphan_stubs.py {out}\n")
    p = os.path.join(td, name)
    with open(p, 'w') as fh:
        fh.write(text)
    return p


def run_check(p):
    return subprocess.run([sys.executable, '-B', '-X', 'utf8', CHECK, p, '--board',
                           os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb')],
                          capture_output=True, text=True, cwd=REPO)


def main():
    from plane_groups import split_plane_group, plane_net_names
    check('the helper splits a group', split_plane_group(' +3V3|+5V ') == ['+3V3', '+5V'])
    check('...and passes a plain net through', plane_net_names(['GND', 'A|B']) == ['GND', 'A', 'B'])
    td = tempfile.mkdtemp(prefix='krt1495_')
    try:
        split = "--nets GND '+3V3|+5V' --plane-layers In1.Cu In2.Cu"
        widths = '--power-nets GND +3V3 +5V --power-nets-widths 0.4 0.4 0.4'
        r = run_check(plan(td, 'split.sh', split, widths))
        check('route_plan_check passes the quoted split group (no R02, no R14)',
              r.returncode == 0 and 'OK:' in r.stdout, r.stdout[-600:])
        r = run_check(plan(td, 'missing.sh', split,
                           '--power-nets GND +3V3 --power-nets-widths 0.4 0.4'))
        check("R14 still names a group member with no width (+5V)",
              r.returncode == 4 and "Poured but no route-step width: ['+5V']" in r.stdout,
              r.stdout[-600:])
        p = plan(td, 'pipe.sh', split, widths)
        with open(p) as fh:
            text = fh.read().replace("--power-nets-widths 0.4 0.4 0.4",
                                     "--power-nets-widths 0.4 0.4 0.4 2>&1 | tee x.log")
        with open(p, 'w') as fh:
            fh.write(text)
        r = run_check(p)
        check('a real, unquoted pipe is still R02', 'FAIL  R02' in r.stdout, r.stdout[-600:])
        from manifest_to_plan import plan_steps_from_manifest
        steps, _ = plan_steps_from_manifest(plan(td, 'conv.sh', split, widths))
        got = [s.get('assignments') for s in steps if s.get('action') == 'route_planes']
        check('manifest_to_plan: one In2.Cu assignment of +3V3 and +5V',
              got == [[{'nets': ['GND'], 'layer': 'In1.Cu'},
                       {'nets': ['+3V3', '+5V'], 'layer': 'In2.Cu'}]], f'{got}')
        from render_power import chain_nets
        _power, planes = chain_nets(os.path.join(td, 'conv.sh'))
        check('render_power reads the group as its nets', planes == {'GND', '+3V3', '+5V'},
              f'{planes}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
