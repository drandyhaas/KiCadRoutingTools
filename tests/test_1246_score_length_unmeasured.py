#!/usr/bin/env python3
"""#1246: `board_score --length-groups` grades a net only once its pads are joined by copper, in BOTH modes.

`score_length` had two modes, and only `pin_pair` treated "not routed yet" as
unmeasured. The default total-copper mode read whatever copper a net had:

- no copper read 0.0 mm, so two unrouted nets passed at spread 0.0;
- fanout stubs read their stub length, so a stub-only group FAILED and added
  1 to `blocking` on top of `broken`.

The verdict depended on the mode, not on the board. Both modes now go through
`net_queries.net_match_length`, and a member whose pads are not joined by track
copper makes its group unmeasured. When every group is unmeasured, `length`
reads `ran: false` and lands in `ungraded`.

Nothing tested `score_length` before this file. Covered here:

1. The issue's boards, both modes:
   - esp_prog `/D_P`, `/D_N` (no copper at all);
   - orangecrab_ext_pll `RAM_D8`..`RAM_D15` (fanout stubs only, 3.0138 mm
     "spread" and `length=1` before the fix).
   A witness asserts each fixture still is what it claims (zero copper, stub
   copper), so a board edit cannot turn this into a vacuous pass.
2. A routed positive control (rp2350 USB pair): still measured, same numbers.
3. A synthetic board for the group rules: a real out-of-tolerance spread
   still FAILS; a zero-copper, a stub-only and a partially routed group are
   each unmeasured; a 3-pad net joined on only two pads is unmeasured in total
   mode and measured in pin_pair mode (pin_pair needs only its pair).
4. The CLI end to end on esp_prog: `length` is listed in UNGRADED and
   `blocking_by.length` is null.

Run:
    python3 tests/test_1246_score_length_unmeasured.py
"""
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

import board_score  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from net_queries import (net_copper_length, net_match_length,  # noqa: E402
                         net_pads_joined)
from run_utils import evidence  # noqa: E402

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
OC = os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
RP = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
RAM_HI = ['RAM_D%d' % i for i in range(8, 16)]
USB_RP = ['/RP2354A/USB_D+', '/RP2354A/USB_D-']

FAILURES = []


def check(cond, what, detail=''):
    print('  %s  %s%s' % ('ok  ' if cond else 'FAIL', what,
                          '' if cond or not detail else '  -- ' + str(detail)))
    if not cond:
        FAILURES.append(what)


def write_groups(tmp, name, groups):
    path = os.path.join(tmp, name)
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(groups, fh)
    return evidence(path, 'length-groups file')


def group(nets, tol, pin_pair=False):
    g = {'nets': nets, 'tolerance_mm': tol}
    if pin_pair:
        g['mode'] = 'pin_pair'
    return g


def ids_by_name(pcb):
    return {n.name: i for i, n in pcb.nets.items()}


# --------------------------------------------------------------------------
# A synthetic board with KNOWN answers. Single-pad parts so every pad is its
# own footprint; tracks end at pad centres.
#   /A  routed, 10.0 mm        /B  routed, 10.4 mm (a 0.2 mm jog each end)
#   /C  no copper              /D  a 2 mm stub off its first pad only
#   /E  three pads, a track joins the first two only
# The pads are 0.1 mm: KiCad's length DRC cuts a run where it leaves the pad its
# track ends at (every release; modelled since #1432), so a 0.2 mm jog inside a
# 1 mm pad would not count as 0.2 mm.
SYN_NETS = ['/A', '/B', '/C', '/D', '/E']


def _fp(ref, x, y, net_id, net):
    return (' (footprint "L:P" (layer "F.Cu") (at %s %s)\n'
            '  (property "Reference" "%s" (at 0 -2 0) (layer "F.SilkS") '
            '(effects (font (size 1 1) (thickness 0.15))))\n'
            '  (pad "1" smd rect (at 0 0) (size 0.1 0.1) (layers "F.Cu") '
            '(net %d "%s")))\n' % (x, y, ref, net_id, net))


def _seg(x1, y1, x2, y2, net_id):
    return (' (segment (start %s %s) (end %s %s) (width 0.2) (layer "F.Cu") '
            '(net %d))\n' % (x1, y1, x2, y2, net_id))


def synthetic_board(tmp):
    nets = ''.join(' (net %d "%s")\n' % (i + 1, n) for i, n in enumerate(SYN_NETS))
    body = ''
    ref = 0
    for i, (net, y) in enumerate(zip(SYN_NETS, (10, 20, 30, 40, 50))):
        xs = (10, 20, 30) if net == '/E' else (10, 20)
        for x in xs:
            ref += 1
            body += _fp('P%d' % ref, x, y, i + 1, net)
    body += _seg(10, 10, 20, 10, 1)                       # /A 10.0
    body += _seg(10, 20, 10, 20.2, 2)                     # /B 0.2
    body += _seg(10, 20.2, 20, 20.2, 2)                   #    10.0
    body += _seg(20, 20.2, 20, 20, 2)                     #    0.2
    body += _seg(10, 40, 12, 40, 4)                       # /D stub 2.0
    body += _seg(10, 50, 20, 50, 5)                       # /E first two pads
    path = os.path.join(tmp, 'syn1246.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator "pcbnew") '
                 '(general (thickness 1.6))\n (paper "A4")\n'
                 ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 ' (setup)\n (net 0 "")\n' + nets + body + ')\n')
    return evidence(path, 'synthetic board')


def test_issue_boards(tmp):
    print('\n1. the issue\'s boards, both modes')
    esp = parse_kicad_pcb(ESP)
    eid = ids_by_name(esp)
    check(all(net_copper_length(esp, eid[n]) == 0.0 for n in ('/D_P', '/D_N')),
          'witness: esp_prog /D_P and /D_N carry NO copper')
    oc = parse_kicad_pcb(OC)
    oid = ids_by_name(oc)
    stub = {n: net_copper_length(oc, oid[n]) for n in RAM_HI}
    check(all(v > 0.5 for v in stub.values()),
          'witness: orangecrab RAM_D8..RAM_D15 each carry stub copper', stub)
    check(not any(net_pads_joined(oc, oid[n]) for n in RAM_HI),
          'witness: no RAM_D8..RAM_D15 net has its pads joined')

    for pin_pair in (False, True):
        mode = 'pin_pair' if pin_pair else 'total'
        r = board_score.score_length(ESP, write_groups(
            tmp, 'esp_%s.json' % mode, {'USB': group(['/D_P', '/D_N'], 0.15, pin_pair)}))
        g = r.get('groups', {}).get('USB', {})
        check(r['ran'] is False and r['count'] is None,
              'esp_prog %s: length is NOT graded (ran false, count null)' % mode, r)
        check('pass' not in g and 'spread_mm' not in g
              and sorted(g.get('unjoined_nets') or []) == ['/D_N', '/D_P'],
              'esp_prog %s: the group reads unmeasured, naming both nets' % mode, g)

        r = board_score.score_length(OC, write_groups(
            tmp, 'oc_%s.json' % mode, {'BYTE1': group(RAM_HI, 0.5, pin_pair)}))
        g = r.get('groups', {}).get('BYTE1', {})
        check(r['ran'] is False and r['count'] is None,
              'orangecrab %s: a stub-only group is NOT graded (was count 1, '
              'spread 3.0138, in total mode)' % mode, r)
        check(sorted(g.get('unjoined_nets') or []) == sorted(RAM_HI)
              and 'spread_mm' not in g,
              'orangecrab %s: every stub-only member is named unjoined' % mode, g)


def test_routed_control(tmp):
    print('\n2. a routed group is still measured (positive control)')
    pcb = parse_kicad_pcb(RP)
    rid = ids_by_name(pcb)
    check(all(net_pads_joined(pcb, rid[n]) for n in USB_RP),
          'rp2350 USB_D+/USB_D- pads are joined by track copper')
    want = {False: 0.0366, True: 0.0873}
    for pin_pair in (False, True):
        mode = 'pin_pair' if pin_pair else 'total'
        r = board_score.score_length(RP, write_groups(
            tmp, 'rp_%s.json' % mode, {'USB': group(USB_RP, 0.1, pin_pair)}))
        g = r.get('groups', {}).get('USB', {})
        check(r['ran'] is True and r['count'] == 0 and r['unmeasured_groups'] == 0
              and g.get('pass') is True
              and abs(g.get('spread_mm', -1) - want[pin_pair]) < 1e-4,
              'rp2350 %s: measured, spread %.4f, pass' % (mode, want[pin_pair]), r)
    check(abs(net_match_length(pcb, rid[USB_RP[0]]) - 2.52938) < 1e-4
          and abs(net_match_length(pcb, rid[USB_RP[1]]) - 2.49275) < 1e-4,
          'rp2350: net_match_length is total copper once joined (2.52938 / 2.49275)')


def test_group_rules(tmp):
    print('\n3. the group rules, on a synthetic board with known answers')
    syn = synthetic_board(tmp)
    pcb = parse_kicad_pcb(syn)
    sid = ids_by_name(pcb)
    check(abs(net_match_length(pcb, sid['/A']) - 10.0) < 1e-6
          and abs(net_match_length(pcb, sid['/B']) - 10.4) < 1e-6,
          'net_match_length: /A 10.0, /B 10.4')
    check(net_match_length(pcb, sid['/C']) is None
          and net_match_length(pcb, sid['/D']) is None,
          'net_match_length: no copper and a stub are None, never 0.0 or 2.0',
          (net_match_length(pcb, sid['/C']), net_match_length(pcb, sid['/D'])))
    check(abs(net_copper_length(pcb, sid['/D']) - 2.0) < 1e-6,
          'witness: /D does carry 2.0 mm of stub copper')
    check(net_match_length(pcb, sid['/E']) is None
          and abs(net_match_length(pcb, sid['/E'], mode='pin_pair') - 10.0) < 1e-6,
          '/E (3 pads, 2 joined): total None, pin_pair 10.0')

    r = board_score.score_length(syn, write_groups(tmp, 'syn_mixed.json', {
        'AB': group(['/A', '/B'], 0.5),
        'AB_tight': group(['/A', '/B'], 0.1),
        'AC': group(['/A', '/C'], 0.5),
        'AD': group(['/A', '/D'], 0.5),
        'ABC': group(['/A', '/B', '/C'], 0.5),
        'AE': group(['/A', '/E'], 0.5),
        'AE_pp': group(['/A', '/E'], 0.5, pin_pair=True),
    }))
    g = r.get('groups', {})
    check(r['ran'] is True and r['count'] == 1 and r['unmeasured_groups'] == 4,
          'mixed file: graded, ONE failure (AB_tight), four unmeasured', r)
    check(g['AB'].get('pass') is True and abs(g['AB']['spread_mm'] - 0.4) < 1e-6,
          'AB: spread 0.4 passes at 0.5', g['AB'])
    check(g['AB_tight'].get('pass') is False,
          'AB_tight: a REAL out-of-tolerance spread still fails', g['AB_tight'])
    for name, unj in (('AC', ['/C']), ('AD', ['/D']), ('ABC', ['/C']), ('AE', ['/E'])):
        check(g[name].get('unjoined_nets') == unj and 'pass' not in g[name],
              '%s: unmeasured, unjoined %s' % (name, unj), g[name])
    check(g['AE_pp'].get('pass') is True and g['AE_pp'].get('spread_mm') == 0.0,
          'AE_pp: pin_pair measures /E on its first pad pair', g['AE_pp'])

    r = board_score.score_length(syn, write_groups(tmp, 'syn_none.json', {
        'AC': group(['/A', '/C'], 0.5), 'AD': group(['/A', '/D'], 0.5)}))
    check(r['ran'] is False and r['count'] is None,
          'every group unmeasured: length is ran false, count null', r)

    r = board_score.score_length(syn, write_groups(tmp, 'syn_missing.json', {
        'AZ': group(['/A', '/NOPE'], 0.5)}))
    check(r['ran'] is True and r['count'] == 1,
          'a group naming a net that does not exist still counts (a spec finding)', r)


def test_cli(tmp):
    print('\n4. the CLI end to end on esp_prog')
    groups = write_groups(tmp, 'esp_cli.json', {'USB': group(['/D_P', '/D_N'], 0.15)})
    out = os.path.join(tmp, 'esp_score.json')
    p = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_tools', 'board_score.py'), ESP,
                        '--length-groups', groups, '--json', out],
                       capture_output=True, text=True, cwd=ROOT, timeout=600)
    check('Traceback' not in p.stderr, 'board_score ran without a traceback',
          p.stderr[-800:])
    try:
        d = json.load(open(evidence(out, 'board_score --json output'), encoding='utf-8'))
    except AssertionError as e:
        check(False, 'board_score wrote its JSON', e)
        return
    check('length' in d.get('ungraded', []), "'length' is listed in ungraded",
          d.get('ungraded'))
    check(d.get('blocking_by', {}).get('length', 'absent') is None,
          'blocking_by.length is null (never 0, which reads as a pass)',
          d.get('blocking_by'))
    line = [ln for ln in p.stdout.splitlines() if ln.startswith('UNGRADED')]
    check(bool(line) and 'length' in line[0],
          'the printed UNGRADED line names length', line)


def main():
    for b in (ESP, OC, RP):
        evidence(b, 'tracked board')
    tmp = tempfile.mkdtemp(prefix='t1246_')
    test_issue_boards(tmp)
    test_routed_control(tmp)
    test_group_rules(tmp)
    test_cli(tmp)
    print('\n%d failure(s)' % len(FAILURES))
    for f in FAILURES:
        print('  FAILED: %s' % f)
    if not FAILURES:
        print('PASS  #1246 score_length grades only joined nets, in both modes')
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
