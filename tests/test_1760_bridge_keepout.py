#!/usr/bin/env python3
"""Cleanup bridges keep out of keep-outs (#1760).

route.py's search keeps out of a `tracks not_allowed` rule area, but the
copper close_soft_joints and snap_stub_gaps lay after it was gated on other
nets' copper only, so the #470 via->pad graze bridge ran a track to a pad
CENTRE inside the area the route had landed beside. Every such connector now
asks one shared predicate, `check_drc.copper_keepout_gate` (a forbidding board
rule area through `keepout_refusal`, a net-filtered keep-out through
`net_keepouts.audit_items`; copper grown by half its width, no margin), and a
refused bridge is skipped (the joint it would firm is already electrical).

  1. the gate: None on a board with no forbidding area; refuses a segment
     inside the area and one whose COPPER (not centreline) reaches in, passes
     one whose copper stops short (no clearance margin) and one on a layer the
     area does not cover; a net-filtered keep-out refuses its bound net only;
  2. snap_stub_gaps' `_connector_clear` refuses a connector inside the area
     and passes the same connector with the area elsewhere;
  3. end to end, the issue's esp_prog repro: with the same-net via and NO
     area the bridge fires (non-vacuity); with the area it does not, check_drc
     finds no keep-out copper, the net is connected, and the run's summary
     carries `rule_area_keepouts: []`.

    python3 tests/test_1760_bridge_keepout.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb, Segment                # noqa: E402
from check_drc import copper_keepout_gate                        # noqa: E402
from pcb_modification import _connector_clear                    # noqa: E402
import net_keepouts                                              # noqa: E402
import run_utils                                                 # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
NET = 'Net-(C2-Pad1)'
X, Y = 127.0, 100.0          # open board space (test_1442_layer_switch_keepout)
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def py(*argv):
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8', *map(str, argv)], cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError(f'BROKEN TEST: {argv[0]} crashed:\n{out[-1500:]}')
    return r.returncode, out


def with_area(src, out, rect, layers=('F.Cu',), forbid=('tracks', 'vias')):
    rc, o = py('py_router/add_rule_area.py', src, out, '--name', 'KO',
               '--layers', *layers, '--rect', *rect, '--forbid', *forbid)
    if rc != 0:
        raise AssertionError(f'add_rule_area failed: {o[-400:]}')
    return run_utils.evidence(out, 'board with a rule area')


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def seg(x1, y1, x2, y2, w=0.2, layer='F.Cu', net=1):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=layer, net_id=net)


def gate_checks(tmp):
    print('1. copper_keepout_gate')
    plain = parse(BOARD)
    ok('no forbidding area -> None', copper_keepout_gate(plain) is None)
    here = parse(with_area(BOARD, os.path.join(tmp, 'here.kicad_pcb'),
                           (X - 1, Y - 1, X + 1, Y + 1)))
    g = copper_keepout_gate(here)
    ok('area -> a gate', g is not None)
    ok('segment inside refused', bool(g([seg(X - 2, Y, X + 2, Y)])))
    # centreline 0.05 outside the edge, half width 0.1: copper reaches 0.05 in
    ok('copper reaching in refused', bool(g([seg(X - 2, Y + 1.05, X + 2, Y + 1.05)])))
    # copper edge 0.05 short of the area: free (no clearance margin)
    ok('copper short of the edge passes', g([seg(X - 2, Y + 1.15, X + 2, Y + 1.15)]) == '')
    ok('other layer passes', g([seg(X - 2, Y, X + 2, Y, layer='B.Cu')]) == '')
    ok('far copper passes', g([seg(X + 5, Y, X + 6, Y)]) == '')
    nid = next(i for i, n in plain.nets.items() if n.name == NET)
    other = next(i for i, n in plain.nets.items() if i and n.name and n.name != NET)
    net_keepouts.attach(plain, [{
        'name': 'NK', 'nets': frozenset({NET}), 'layers': ['F.Cu'],
        'polygon': [(X - 1, Y - 1), (X + 1, Y - 1), (X + 1, Y + 1), (X - 1, Y + 1)],
        'holes': [], 'tracks_allowed': False, 'vias_allowed': False}])
    g2 = copper_keepout_gate(plain)
    ok('net-filtered: bound net refused', bool(g2 and g2([seg(X - 2, Y, X + 2, Y, net=nid)])))
    ok('net-filtered: other net passes', g2 is not None
       and g2([seg(X - 2, Y, X + 2, Y, net=other)]) == '')
    return here, nid


def connector_checks(tmp, here, nid):
    print("2. snap_stub_gaps' _connector_clear")
    away = parse(with_area(BOARD, os.path.join(tmp, 'away.kicad_pcb'),
                           (X + 9, Y - 1, X + 11, Y + 1)))
    ok('connector clear with the area elsewhere',
       _connector_clear(X - 0.3, Y, X + 0.3, Y, 0.2, 'F.Cu', nid, away, 0.1))
    ok('connector inside the area refused',
       not _connector_clear(X - 0.3, Y, X + 0.3, Y, 0.2, 'F.Cu', nid, here, 0.1))


def add_via(src, dst):
    from kicad_writer import generate_via_sexpr
    t = open(src, encoding='utf-8', newline='').read()
    v = generate_via_sexpr(129.925, 104.485, 0.5, 0.3, ['F.Cu', 'B.Cu'], 0, net_name=NET)
    i = t.rstrip().rfind(')')
    open(dst, 'w', encoding='utf-8', newline='').write(t[:i].rstrip('\n') + '\n' + v + '\n' + t[i:])
    return run_utils.evidence(dst, 'board with the via')


def end_to_end(tmp):
    print('3. end to end: the via->pad graze bridge (esp_prog U1.10)')
    a = os.path.join(tmp, 'a.kicad_pcb')
    rc, o = py('py_router/copy_board.py', BOARD, a)
    run_utils.evidence(a, 'copied board')
    # the issue's area: its edge crosses U1.10, centre 0.034 mm inside
    k = with_area(a, os.path.join(tmp, 'k.kicad_pcb'), (129.45, 101.0, 130.4, 103.654),
                  layers=('F.Cu', 'B.Cu'))
    runs = {'av': add_via(a, os.path.join(tmp, 'av.kicad_pcb')),
            'kv': add_via(k, os.path.join(tmp, 'kv.kicad_pcb'))}
    logs = {}
    for name, src in runs.items():
        out = os.path.join(tmp, f'r_{name}.kicad_pcb')
        rc, logs[name] = py('py_router/route.py', src, out, '--nets', NET,
                            '--track-width', '0.1', '--clearance', '0.1')
        ok(f'{name}: route.py exit 0', rc == 0, f'exit {rc}')
        run_utils.evidence(out, f'routed board {name}')
    ok('control (via, no area): the bridge fires',
       'Bridged 1 same-net soft joint' in logs['av'])
    ok('with the area: no bridge laid', 'Bridged' not in logs['kv'])
    out = os.path.join(tmp, 'r_kv.kicad_pcb')
    rc, o = py('py_router/check_drc.py', out)
    ok('check_drc: no keep-out copper', 'KEEPOUT' not in o and 'keep-out rule area' not in o,
       '' if 'KEEPOUT' not in o else o[o.find('KEEPOUT'):][:300])
    rc, o = py('py_router/check_connected.py', out, '--nets', NET)
    ok('check_connected: the net is connected', rc == 0, o[-300:] if rc else '')
    m = re.search(r'^JSON_SUMMARY: (.*)$', logs['kv'], re.M)
    s = json.loads(m.group(1)) if m else {}
    ok("summary: rule_area_keepouts == []", s.get('rule_area_keepouts') == [],
       repr(s.get('rule_area_keepouts', '<absent>')))


def main():
    tmp = tempfile.mkdtemp(prefix='t1760_')
    try:
        here, nid = gate_checks(tmp)
        connector_checks(tmp, here, nid)
        end_to_end(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
