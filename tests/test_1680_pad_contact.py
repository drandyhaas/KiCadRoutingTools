#!/usr/bin/env python3
"""#1680: a via or track lying on another net's pad is gone after a route step
that was asked to route its net, and the run says so when it is not.

A clearance step names a part it could not move off fanout copper on its
SHORTED line, and said the later route steps repair it. They had no way to: the
copper is the input's and a net that reads connected never routes, so the short
shipped under a clean exit.

The rig: a two-layer board where net /B runs U1.1 -> via -> U1.2, the via on the
back-side pad of C1, which is net /A.

1. The measure: `pad_contacts` names exactly what check_drc marks `short`, a
   track on a pad as well as a via, and exempts a net tie's partner pad.
2. The rip takes the via and the stubs that led to it, stops at the first node,
   and leaves alone copper of a net out of scope, locked copper and a protected
   net.
3. `route.py --nets /B`: the via is gone, /B connects, check_drc has no short,
   exit 0, and the run names what it took (`pad_contact_rips`). Both fronts: the
   engine's return_results path lists the via and its tracks for removal.
4. `route.py --nets /A`: /B is out of scope, the short stays, and the end of the
   run names it on the console, in JSON_SUMMARY_MIN and in --json-out.
5. check_complete names it among its reasons.
6. #1681's half: copper inside a pad's clearance but not on it stays where it
   is and is named `pad_grazes` at the end of the run, not `pad_contacts`.

Run:
    python3 tests/test_1680_pad_contact.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from pad_contact import pad_contacts, rip_pad_contacts  # noqa: E402

FAILS = []
VIA_AT = (20, 15)


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _pad(num, x, net, layer, name):
    return ('\t\t(pad "%s" smd rect (at %s 0) (size 1 1) (layers "%s") '
            '(net %d "%s") (uuid "p%s-%s"))\n'
            % (num, x, layer, net, name, name.strip('/'), num))


def _fp(ref, x, y, layer, pads, extra=''):
    return ('\t(footprint "t:%s" (layer "%s") (uuid "u%s") (at %s %s)\n'
            '\t\t(property "Reference" "%s" (at 0 0) (layer "F.SilkS"))\n%s%s\t)\n'
            % (ref, layer, ref, x, y, ref, extra, ''.join(pads)))


def _seg(x1, y1, x2, y2, layer, net, tag, width=0.25, locked=False):
    return ('\t(segment (start %s %s) (end %s %s) (width %s) (layer "%s") '
            '%s(net "%s") (uuid "%s"))\n'
            % (x1, y1, x2, y2, width, layer, '(locked yes) ' if locked else '',
               net, tag))


def _board(work, name, via_locked=False, copper=None, extra_fp='',
           project=None):
    """/B: U1.1 (F.Cu) -> via on C1.1 (B.Cu, net /A) -> U2.1 (F.Cu)."""
    vx, vy = VIA_AT
    nets = '\t(net 0 "")\n\t(net 1 "/A")\n\t(net 2 "/B")\n'
    fps = (_fp('U1', 10, 15, 'F.Cu', [_pad('1', 0, 2, 'F.Cu', '/B')])
           + _fp('U2', 30, 15, 'F.Cu', [_pad('1', 0, 2, 'F.Cu', '/B')])
           + _fp('C1', vx, vy, 'B.Cu', [_pad('1', 0, 1, 'B.Cu', '/A')])
           + _fp('J1', 20, 28, 'B.Cu', [_pad('1', 0, 1, 'B.Cu', '/A')])
           + extra_fp)
    if copper is None:
        copper = (_seg(10, 15, vx, vy, 'F.Cu', '/B', 's1')
                  + _seg(vx, vy, 30, 15, 'F.Cu', '/B', 's2')
                  + '\t(via (at %s %s) (size 0.6) (drill 0.3) '
                    '(layers "F.Cu" "B.Cu") %s(net "/B") (uuid "v1"))\n'
                  % (vx, vy, '(locked yes) ' if via_locked else ''))
    text = ('(kicad_pcb (version 20260206) (generator "pcbnew")\n'
            ' (general (thickness 1.6)) (paper "A4")\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            ' (setup)\n' + nets
            + ' (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type solid)) '
              '(fill none) (layer "Edge.Cuts"))\n'
            + fps + copper + ')\n')
    p = os.path.join(work, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(text)
    with open(p[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(project or {'board': {'design_settings': {'rules': {}}}}, fh)
    return p


def _py(*argv):
    return [sys.executable, '-X', 'utf8', *argv]


def _drc(path, work, tag):
    """check_drc's items. Its exit is the verdict either way, so it is the JSON
    that is asserted, through `evidence`."""
    j = os.path.join(work, tag + '.json')
    subprocess.run(_py(os.path.join('py_router', 'check_drc.py'), path,
                       '--clearance', '0.1', '--json', j, '--quiet'),
                   cwd=ROOT, capture_output=True)
    with open(evidence(j), encoding='utf-8') as fh:
        return json.load(fh)['items']


def test_measure(work):
    print('The measure agrees with check_drc:')
    p = _board(work, 'via')
    rows = pad_contacts(parse_kicad_pcb(p))
    check('one row, the via on C1.1',
          [(r['pad'], r['net'], r['kind']) for r in rows] == [('C1.1', '/B', 'via')],
          str(rows))
    shorts = [i for i in _drc(p, work, 'via') if i.get('short')]
    check('check_drc marks that one pad-via `short`',
          [(i['type'], i.get('pad_ref')) for i in shorts] == [('pad-via', 'C1.1')],
          str(shorts))
    t = _board(work, 'track', copper=(
        _seg(10, 15, 19, 15, 'B.Cu', '/B', 'a') + _seg(19, 15, 30, 15, 'B.Cu', '/B', 'b')))
    rows = pad_contacts(parse_kicad_pcb(t))
    check('a track across the pad is a row too',
          [(r['pad'], r['kind']) for r in rows] == [('C1.1', 'track')], str(rows))
    clear = _board(work, 'clear', copper=_seg(10, 15, 30, 15, 'F.Cu', '/B', 'a'))
    check('copper on another layer is no contact',
          pad_contacts(parse_kicad_pcb(clear)) == [])
    tie = _fp('NT1', 20, 15, 'F.Cu', [_pad('1', 0, 1, 'F.Cu', '/A'),
                                      _pad('2', 0.6, 2, 'F.Cu', '/B')],
              extra='\t\t(net_tie_pad_groups "1, 2")\n')
    n = _board(work, 'tie', extra_fp=tie, copper=_seg(10, 15, 20.6, 15, 'F.Cu', '/B', 'a'))
    pcb = parse_kicad_pcb(n)
    check('a net tie partner pad is exempt',
          not [r for r in pad_contacts(pcb) if r['pad'].startswith('NT1')],
          str(pad_contacts(pcb)))
    tab = _fp('SH1', 20, 15, 'B.Cu', ['\t\t(pad "1" smd rect (at 8 0) (size 1 1) '
                                      '(layers "B.Cu") (uuid "tab"))\n'])
    n = _board(work, 'tab', extra_fp=tab, copper=_seg(28, 15, 28, 10, 'B.Cu', '/B', 'a'))
    check('a pad with no net shorts no two nets',
          pad_contacts(parse_kicad_pcb(n)) == [], str(pad_contacts(parse_kicad_pcb(n))))
    g = _board(work, 'graze', copper=_seg(10, 15.7, 30, 15.7, 'B.Cu', '/B', 'a'))
    pcb = parse_kicad_pcb(g)
    rows = pad_contacts(pcb, clearance=0.2)
    check('with a clearance a near miss is a row, not a short',
          [(r['pad'], r['short']) for r in rows] == [('C1.1', False)]
          and pad_contacts(pcb) == [], str(rows))


def test_rip(work):
    print('The rip:')
    pcb = parse_kicad_pcb(_board(work, 'rip'))
    got = rip_pad_contacts(pcb, {2}, None)
    check('takes the via and both stubs',
          len(got) == 1 and got[0]['removed_vias'] == 1 and got[0]['removed_tracks'] == 2
          and not pcb.vias and not pcb.segments, str(got))
    pcb = parse_kicad_pcb(_board(work, 'rip2'))
    check('a net out of scope is left alone',
          rip_pad_contacts(pcb, {1}, None) == [] and len(pcb.vias) == 1)
    pcb = parse_kicad_pcb(_board(work, 'rip3', via_locked=True))
    check('a locked via is left alone',
          rip_pad_contacts(pcb, {2}, None) == [] and len(pcb.vias) == 1)
    prot = _board(work, 'rip4', project={'board': {'design_settings': {'rules': {}}},
                                         'kicad_routing_tools': {
                                             'protected_nets': {'/B': 'matched'}}})
    pcb = parse_kicad_pcb(prot)
    check('a protected net is left alone',
          rip_pad_contacts(pcb, {2}, prot) == [] and len(pcb.vias) == 1)
    pcb = parse_kicad_pcb(prot)
    check('...unless the step names it exactly (#521)',
          len(rip_pad_contacts(pcb, {2}, prot, overrides=['/B'])) == 1)
    # A branch: the run stops at the junction, the far arm and trunk survive.
    vx, vy = VIA_AT
    branch = _board(work, 'rip5', copper=(
        _seg(vx, vy, 20, 10, 'F.Cu', '/B', 'stub1')
        + _seg(20, 10, 15, 10, 'F.Cu', '/B', 'stub2')       # junction at (15,10)
        + _seg(15, 10, 10, 10, 'F.Cu', '/B', 'arm1')
        + _seg(15, 10, 15, 5, 'F.Cu', '/B', 'arm2')
        + '\t(via (at %s %s) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
          '(net "/B") (uuid "v1"))\n' % (vx, vy)))
    pcb = parse_kicad_pcb(branch)
    got = rip_pad_contacts(pcb, {2}, None)
    check('the stub ends at the first junction',
          len(got) == 1 and got[0]['removed_tracks'] == 2 and len(pcb.segments) == 2,
          f"{got} left {len(pcb.segments)}")


def _route(work, inp, out, nets):
    j = out[:-len('.kicad_pcb')] + '.summary.json'
    return run_check(_py(os.path.join('py_router', 'route.py'), inp, out, '--nets',
                         *nets, '--clearance', '0.1', '--track-width', '0.25',
                         '--json-out', j), accept=True, cwd=ROOT), j


def test_route_in_scope(work):
    print('route.py --nets /B repairs it:')
    p = _board(work, 'in')
    out = os.path.join(work, 'in_out.kicad_pcb')
    r, j = _route(work, p, out, ['/B'])
    pcb = parse_kicad_pcb(evidence(out))
    check('the via is off the pad', pad_contacts(pcb) == [] and not any(
        abs(v.x - VIA_AT[0]) < 0.3 and abs(v.y - VIA_AT[1]) < 0.3 for v in pcb.vias),
        str(pcb.vias))
    check('check_drc: no short', not [i for i in _drc(out, work, 'in_out') if i.get('short')])
    conn = run_check(_py(os.path.join('py_router', 'check_connected.py'), out,
                         '--nets', '/B'), accept=True, cwd=ROOT)
    check('/B connects', 'ALL NETS FULLY CONNECTED' in conn.stdout, conn.stdout[-300:])
    with open(evidence(j), encoding='utf-8') as fh:
        doc = json.load(fh)
    check('the run names what it took',
          [x['pad'] for x in doc.get('pad_contact_rips', [])] == ['C1.1'], str(doc.get('pad_contact_rips')))
    check('nothing is left to report', 'pad_contacts' not in doc)
    check('the console says so', 'Pad contacts: 1 via(s)/track(s)' in r.stdout)


def test_route_gui_front(work):
    print('The engine path the GUI takes lists the copper for removal:')
    from route import batch_route
    p = _board(work, 'gui')
    pcb = parse_kicad_pcb(p)
    with contextlib.redirect_stdout(io.StringIO()):
        res = batch_route(p, '', ['/B'], clearance=0.1, track_width=0.25,
                          pcb_data=pcb, return_results=True)
    rd = res[3]
    check('the via is on the removal list',
          [(round(v.x, 2), round(v.y, 2)) for v in rd.get('vias_to_remove', [])] == [VIA_AT],
          str(rd.get('vias_to_remove')))
    check('its tracks are too', len(rd.get('segments_to_remove', [])) == 2,
          str(rd.get('segments_to_remove')))
    check('nothing is left on the write model', rd.get('pad_contacts') == [],
          str(rd.get('pad_contacts')))


def test_route_out_of_scope(work):
    print('route.py --nets /A leaves it and says so:')
    p = _board(work, 'out')
    out = os.path.join(work, 'out_out.kicad_pcb')
    r, j = _route(work, p, out, ['/A'])
    check('the short is still on the board',
          [x['pad'] for x in pad_contacts(parse_kicad_pcb(evidence(out)))] == ['C1.1'])
    check('the console names it',
          'SHORTED on the board this run ships' in r.stdout and 'C1.1 (/A) on a /B via' in r.stdout,
          r.stdout[-600:])
    mins = [json.loads(l[len('JSON_SUMMARY_MIN: '):]) for l in r.stdout.splitlines()
            if l.startswith('JSON_SUMMARY_MIN: ')]
    check('JSON_SUMMARY_MIN names the pad',
          len(mins) == 1 and mins[0].get('pad_contacts') == ['C1.1'], str(mins))
    with open(evidence(j), encoding='utf-8') as fh:
        doc = json.load(fh)
    check('--json-out carries the row',
          [x['pad'] for x in doc.get('pad_contacts', [])] == ['C1.1'], str(doc.get('pad_contacts')))
    # Only the verdict's reason text is under test: --skip-slow keeps it quick.
    jc = os.path.join(work, 'cc.json')
    run_check(_py('check_complete.py', out, '--skip-slow', '--json', jc),
              code=4, refuse="on another net's pad", cwd=ROOT)
    with open(evidence(jc), encoding='utf-8') as fh:
        cc = json.load(fh)
    check('check_complete names it among its reasons',
          'C1.1' in cc.get('reason', '') and 'on another net' in cc.get('reason', ''),
          cc.get('reason', ''))


def test_route_graze(work):
    print('A graze stays and is named apart from a short:')
    # The /B track runs 0.05 mm off C1.1 on B.Cu: inside 0.1, not touching.
    p = _board(work, 'grz', copper=(
        _seg(10, 15.675, 30, 15.675, 'B.Cu', '/B', 'a')
        + '\t(via (at 10 15.675) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
          '(net "/B") (uuid "v1"))\n'
        + '\t(via (at 30 15.675) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
          '(net "/B") (uuid "v2"))\n'
        + _seg(10, 15, 10, 15.675, 'F.Cu', '/B', 'b')
        + _seg(30, 15, 30, 15.675, 'F.Cu', '/B', 'c')))
    out = os.path.join(work, 'grz_out.kicad_pcb')
    r, j = _route(work, p, out, ['/B'])
    mins = [json.loads(l[len('JSON_SUMMARY_MIN: '):]) for l in r.stdout.splitlines()
            if l.startswith('JSON_SUMMARY_MIN: ')]
    check('JSON_SUMMARY_MIN names the graze',
          len(mins) == 1 and mins[0].get('pad_grazes') == ['C1.1']
          and 'pad_contacts' not in mins[0], str(mins))
    check('the console says NOT CLEAR', 'NOT CLEAR on the board this run ships' in r.stdout,
          r.stdout[-500:])
    with open(evidence(j), encoding='utf-8') as fh:
        doc = json.load(fh)
    check('--json-out carries it',
          [x['pad'] for x in doc.get('pad_grazes', [])] == ['C1.1'], str(doc.get('pad_grazes')))


def main():
    work = tempfile.mkdtemp(prefix='t1680_')
    try:
        test_measure(work)
        test_rip(work)
        test_route_in_scope(work)
        test_route_gui_front(work)
        test_route_out_of_scope(work)
        test_route_graze(work)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print(f"{'FAILED: ' + ', '.join(FAILS) if FAILS else 'OK'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
