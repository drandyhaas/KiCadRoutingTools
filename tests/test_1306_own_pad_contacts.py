#!/usr/bin/env python3
"""#1306: two of ONE footprint's pads touching on different nets are graded
by check_drc, check_assembly and place_pose -- KiCad's shorting_items.

steppenprobe U1 came out of the placement writer with its pads at their old
angle (#1305), 164 different-net pad pairs overlapping. KiCad reported 176
shorting_items; check_drc said NO DRC VIOLATIONS (it skipped a footprint's
own pads), check_assembly `buildable` and place_pose `pad_conflicts 0`,
because every pad channel there is between two parts.

The fixture is that defect made by hand: esp_prog's U1 (SSOP-20, pads 0.325 x
1.27 mm at 0.65 mm pitch) with every pad turned 90 degrees in place.

Checks:
  1. The census (`check_drc.footprint_own_pad_census`): nothing on the
     healthy board; on the turned one only U1 rows, with real-net shorts and
     no-net contacts told apart.
  2. KiCad's exemptions (`own_pad_pair_exemption`): one net, two no-net
     pads, one pad number unless it carries two real nets, a net-tie group.
     A pair closer than clearance without touching is a row, not a contact.
  3. check_drc: the turned board fails with U1-internal `pad-pad` rows; with
     --baseline the healthy board they stay violations, with --baseline the
     turned board itself they are accepted `inherited-own-pad-contact`.
  4. check_assembly: NOT BUILDABLE for this reason, exit 4; --baseline the
     turned board itself is buildable; the healthy board is buildable.
  5. place_pose: `own_pad_short_count` reads the shorts and `legal` is
     false; the healthy board reads 0.

    python3 tests/test_1306_own_pad_contacts.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from copy_board import copy_board                                   # noqa: E402
from kicad_parser import parse_kicad_pcb, Pad, Footprint, Net, PCBData  # noqa: E402
from check_drc import (footprint_own_pad_census,                    # noqa: E402
                       own_pad_pair_exemption)
from run_utils import check as run_check, evidence                  # noqa: E402

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
DRC = os.path.join(ROOT, 'py_router', 'check_drc.py')
POSE = os.path.join(ROOT, 'py_placer', 'place_pose.py')
ASSEMBLY = os.path.join(ROOT, 'py_tools', 'check_assembly.py')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def turn_u1_pads(src, dst):
    """esp_prog's U1 with every pad turned 90 degrees about its own centre,
    the footprint left where it is: what #1305's writer did to U1."""
    with open(src, encoding='utf-8', newline='') as fh:
        txt = fh.read()
    i = txt.index('(property "Reference" "U1"')
    start = txt.rindex('(footprint ', 0, i)
    end = txt.index('\n\t(footprint ', i)
    block, n = re.subn(r'(\(pad "\d+" smd rect\s+\(at -?[\d.]+ -?[\d.]+)\)',
                       r'\1 90)', txt[start:end])
    assert n == 20, n
    with open(dst, 'w', encoding='utf-8', newline='') as fh:
        fh.write(txt[:start] + block + txt[end:])


def pad(num, net, name, x, size=(0.4, 0.4)):
    return Pad(pad_number=num, net_id=net, net_name=name, global_x=x,
               global_y=0.0, local_x=x, local_y=0.0, size_x=size[0],
               size_y=size[1], shape='rect', layers=['F.Cu'], drill=0,
               pad_type='smd', component_ref='P1')


def synthetic(pads, ties=()):
    nets = {p.net_id: Net(p.net_id, p.net_name) for p in pads if p.net_id}
    fp = Footprint(reference='P1', footprint_name='L:P', x=0, y=0, rotation=0,
                   layer='F.Cu', pads=list(pads),
                   net_tie_groups=[list(g) for g in ties])
    by_net = {}
    for p in pads:
        by_net.setdefault(p.net_id, []).append(p)
    return PCBData(board_info=None, nets=nets, footprints={'P1': fp},
                   vias=[], segments=[], pads_by_net=by_net)


def drc_json(board, work, name, *extra):
    out = os.path.join(work, name + '.json')
    r = run_check([sys.executable, '-X', 'utf8', DRC, board, '--json', out,
                   '-q'] + list(extra), code=1)
    del r
    with open(evidence(out, 'check_drc --json'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return [v for v in doc['items'] if v.get('own_footprint')]


def summary(r):
    m = re.search(r'JSON_SUMMARY: (\{.*\})', r.stdout or '')
    return json.loads(m.group(1)) if m else {}


def main():
    work = tempfile.mkdtemp(prefix='krt1306_')
    try:
        esp = os.path.join(work, 'esp.kicad_pcb')
        quiet(copy_board, ESP, esp)
        turned = os.path.join(work, 'turned.kicad_pcb')
        turn_u1_pads(esp, turned)
        evidence(turned, 'U1 with its pads turned')

        print("1. the census")
        healthy = footprint_own_pad_census(quiet(parse_kicad_pcb, esp),
                                           contacts_only=True)['rows']
        rows = footprint_own_pad_census(quiet(parse_kicad_pcb, turned),
                                        contacts_only=True)['rows']
        shorts = [r for r in rows if r['short']]
        check('the healthy board has no own-pad contact', healthy == [],
              str(healthy[:3]))
        check('the turned board has contacts, all inside U1',
              rows and {r['ref'] for r in rows} == {'U1'}, str(len(rows)))
        check('real-net contacts are shorts, no-net ones are not',
              shorts and all(r['net_id_a'] and r['net_id_b'] for r in shorts)
              and all(r['no_net'] and not r['short']
                      for r in rows if r not in shorts),
              f"{len(shorts)} shorts of {len(rows)}")

        print("2. KiCad's exemptions")
        a, b = pad('1', 1, 'A', 0.0), pad('2', 2, 'B', 0.3)
        check('two nets touching are graded', own_pad_pair_exemption(a, b) == '')
        check('one net is exempt', own_pad_pair_exemption(
            a, pad('2', 1, 'A', 0.3)) == 'same net')
        check('two no-net pads are not graded', own_pad_pair_exemption(
            pad('1', 0, '', 0), pad('2', 0, '', 0.3)) == 'no net')
        check('one number with a no-net copy is one pad',
              own_pad_pair_exemption(pad('5', 1, 'A', 0),
                                     pad('5', 0, '', 0.3)) == 'same pad number')
        check("one number on two `unconnected-(...)` nets is one pad",
              own_pad_pair_exemption(
                  pad('5', 3, 'unconnected-(P1-Pad5)', 0),
                  pad('5', 4, 'unconnected-(P1-Pad5)_1', 0.3))
              == 'same pad number')
        check('one number on two REAL nets is graded',
              own_pad_pair_exemption(pad('5', 1, 'A', 0),
                                     pad('5', 2, 'B', 0.3)) == '')
        check('a declared net-tie group is exempt',
              own_pad_pair_exemption(a, b, [{'1', '2'}]) == 'net tie')
        board = synthetic([pad('1', 1, 'A', 0.0), pad('2', 2, 'B', 0.3),
                           pad('3', 3, 'C', 0.9)])
        rows2 = footprint_own_pad_census(board, 0.2, ['F.Cu'])['rows']
        by = {(r['pad_a'], r['pad_b']): r for r in rows2}
        check('1/2 overlap: a contact', ('1', '2') in by
              and by[('1', '2')]['contact'] and by[('1', '2')]['short'],
              str(by.get(('1', '2'))))
        check('2/3 0.2 mm apart at clearance 0.2: inside the 5% margin, no row',
              ('2', '3') not in by, str(by.get(('2', '3'))))
        rows3 = footprint_own_pad_census(board, 0.3, ['F.Cu'])['rows']
        by3 = {(r['pad_a'], r['pad_b']): r for r in rows3}
        check('2/3 at clearance 0.3: a row, not a contact',
              ('2', '3') in by3 and not by3[('2', '3')]['contact'],
              str(by3.get(('2', '3'))))
        tied = synthetic([pad('1', 1, 'A', 0.0), pad('2', 2, 'B', 0.3)],
                         ties=[('1', '2')])
        check('the same contact inside a tie group makes no row',
              footprint_own_pad_census(tied, 0.2, ['F.Cu'])['rows'] == [])

        print("3. check_drc")
        own = drc_json(turned, work, 'drc_turned')
        check('U1-internal pad-pad violations are reported',
              own and all(v['type'] == 'pad-pad' and v['pad_ref'].startswith('U1.')
                          and v['pad_ref2'].startswith('U1.')
                          and not v.get('accepted') for v in own),
              f"{len(own)} rows")
        check('one per contact the census found', len(own) == len(rows),
              f"{len(own)} vs {len(rows)}")
        own_b = drc_json(turned, work, 'drc_base_esp', '--baseline', esp)
        check('--baseline the healthy board: still violations (U1 changed)',
              len(own_b) == len(rows)
              and not any(v.get('accepted') for v in own_b), str(len(own_b)))
        out = os.path.join(work, 'drc_self.json')
        # Exit 0: the board's only violations were U1's, all inherited now.
        run_check([sys.executable, '-X', 'utf8', DRC, turned, '--json', out,
                   '-q', '--baseline', turned], accept=True)
        with open(evidence(out, 'check_drc --json'), encoding='utf-8') as fh:
            self_items = [v for v in json.load(fh)['items']
                          if v.get('own_footprint')]
        check('--baseline the turned board itself: every one inherited',
              len(self_items) == len(rows) and all(
                  v.get('accepted') == 'inherited-own-pad-contact'
                  for v in self_items), str(len(self_items)))

        print("4. check_assembly")
        r = run_check([sys.executable, '-X', 'utf8', ASSEMBLY, turned],
                      refuse='OWN PADS TOUCHING', code=4)
        check('NOT BUILDABLE on the turned board, naming U1',
              'NOT BUILDABLE' in r.stdout and 'U1.' in r.stdout)
        run_check([sys.executable, '-X', 'utf8', ASSEMBLY, turned,
                   '--baseline', turned], accept=True)
        check('--baseline the turned board itself: buildable', True)
        run_check([sys.executable, '-X', 'utf8', ASSEMBLY, esp], accept=True)
        check('the healthy board stays buildable', True)

        print("5. place_pose")
        r = run_check([sys.executable, '-X', 'utf8', POSE, turned,
                       os.path.join(work, 'pp.kicad_pcb'), '--dry-run',
                       '--allow-routed', 'lock', 'U1'], accept=True)
        s = summary(r)
        check('own_pad_short_count reads the shorts, before and after',
              s.get('own_pad_short_count_before') == len(shorts)
              == s.get('own_pad_short_count_after'),
              f"{s.get('own_pad_short_count_before')} / {len(shorts)}")
        check('...and the board is not legal', s.get('legal') is False)
        r = run_check([sys.executable, '-X', 'utf8', POSE, esp,
                       os.path.join(work, 'pp2.kicad_pcb'), '--dry-run',
                       '--allow-routed', 'lock', 'U1'], accept=True)
        check('the healthy board reads 0',
              summary(r).get('own_pad_short_count_after') == 0)
    finally:
        shutil.rmtree(work, ignore_errors=True)

    print(f"\n{'FAILED' if failures else 'PASSED'}: {len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
