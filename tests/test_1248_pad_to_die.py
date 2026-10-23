#!/usr/bin/env python3
"""#1248: a pad's `(die_length ...)` / `(die_delay ...)` is parsed, matched to and graded, as KiCad counts it.

KiCad adds a pad's pad-to-die length to the net's length:
- in its tuners, for the tuned path's start and end pads;
- in its length/skew DRC, for every pad of the net.

KiCad 10 adds `die_delay` (ps) in the delay domain. Before #1248 no parse path
read either token. So `--length-match-group` meandered to a target that left it
out, `--time-matching` left out the delay, and `board_score --length-groups`
graded a spread KiCad does not see.

The reference is KiCad 10.0.0's own DRC on the issue's synthetic board
(rp2350_fpga_eensy_prePlane with `(die_length 1.5)` on U6 pad 52):

    /RP2354A/USB_D+  4.0294 mm   (2.5294 without the token)
    /RP2354A/USB_D-  2.4927 mm
    skew             1.5366 mm   (skew_out_of_range at 0.1 mm)

`tests/gui_parity/test_1248_die_length_parity.py` regenerates those numbers with
kicad-cli and holds the pcbnew parse path to the text path. This file runs
without KiCad:

1. The text parser reads both tokens, and only from their own pad.
2. `net_match_length` / `score_length` give KiCad's lengths and skew on the
   reference board. The control (no token) gives KiCad's 2.5294 and a pass.
   The pin_pair mode adds the measured pair's term only.
3. `member_pad_to_die` picks KiCad's pads for each member kind:
   - a 2-pad net;
   - a member measured from board copper;
   - a multipoint main edge;
   - a pair (the larger leg);
   - the delay.
   3b. A pair is KiCad's pair tuner EXACTLY: each leg its own copper (stub
   included) plus its own end pads' term, the pair the larger leg, under
   either copper convention; a multipoint pair counts only its matched leg's
   terminals (by the net each is on now); time the same way.
4. End to end through the real CLIs on a synthetic board:
   - `route.py --length-match-group` meanders the die-free net up to the
     die-inclusive target;
   - `--time-matching` does the same with `die_delay`;
   - `route_diff --diff-pair-intra-match` matches P + its die to N;
   - `route_diff --length-match-group` on two pairs without intra-match, one
     with unequal legs and a die on its N leg, matches the pairs as KiCad's
     pair tuner measures them (the gui_parity gate has KiCad's DRC referee
     the same board).
   Each is graded on the KiCad measure (within tolerance), with a witness that
   the copper itself differs by the declared term, so a run that did nothing
   cannot pass.
5. `board_score --length-groups` discloses the term (`pad_to_die_mm`).

Run:
    python3 tests/test_1248_pad_to_die.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import uuid
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

import board_score  # noqa: E402
from kicad_parser import Pad, Segment, parse_kicad_pcb  # noqa: E402
from length_matching import member_pad_to_die  # noqa: E402
from net_queries import (net_match_length, net_match_pad_to_die,  # noqa: E402
                         net_pad_to_die_delay, pin_pair_path_length)
from run_utils import evidence  # noqa: E402

RP = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
USB = ['/RP2354A/USB_D+', '/RP2354A/USB_D-']
#: KiCad 10.0.0 DRC on the issue's board, 4 decimals as it prints them.
KICAD_DIE = {'/RP2354A/USB_D+': 4.0294, '/RP2354A/USB_D-': 2.4927}
KICAD_NODIE = {'/RP2354A/USB_D+': 2.5294, '/RP2354A/USB_D-': 2.4927}
KICAD_SKEW = 1.5366
#: KiCad prints 4 decimals, so its value is known to +-0.00005.
KICAD_PRINT = 0.00005 + 1e-9

FAILURES = []


def check(cond, what, detail=''):
    print('  %s  %s%s' % ('ok  ' if cond else 'FAIL', what,
                          '' if cond or not detail else '  -- ' + str(detail)))
    if not cond:
        FAILURES.append(what)


def die_board(tmp, extra=''):
    """The issue's board: rp2350 with (die_length 1.5) on U6 pad 52."""
    with open(evidence(RP, 'tracked board'), encoding='utf-8', newline='') as fh:
        text = fh.read()
    anchor = '(pinfunction "USB_DP_52")'
    assert text.count(anchor) == 1, 'U6 pad 52 anchor moved'
    text = text.replace(anchor, anchor + ' (die_length 1.5)' + extra)
    path = os.path.join(tmp, 'die.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    return evidence(path, 'die board')


def ids(pcb):
    return {n.name: i for i, n in pcb.nets.items()}


def test_parse(tmp):
    print('\n1. the text parser reads both tokens, from their own pad only')
    pcb = parse_kicad_pcb(die_board(tmp, ' (die_delay 0.012345)'))
    hits = [(p.component_ref, p.pad_number, p.die_length, p.die_delay)
            for fp in pcb.footprints.values() for p in fp.pads
            if p.die_length or p.die_delay]
    check(hits == [('U6', '52', 1.5, 0.012345)],
          'U6.52 reads die_length 1.5 mm and die_delay 0.012345 ps; no other '
          'pad reads either', hits)
    plain = parse_kicad_pcb(RP)
    check(not any(p.die_length or p.die_delay
                  for fp in plain.footprints.values() for p in fp.pads),
          'the untouched board reads 0.0 everywhere')


def test_kicad_reference(tmp):
    print('\n2. KiCad\'s lengths and skew on the reference board')
    die = die_board(tmp)
    pcb = parse_kicad_pcb(die)
    nid = ids(pcb)
    got = {n: net_match_length(pcb, nid[n]) for n in USB}
    check(all(abs(got[n] - KICAD_DIE[n]) <= KICAD_PRINT for n in USB),
          'net_match_length = KiCad DRC 4.0294 / 2.4927 mm', got)
    cu = {n: net_match_length(pcb, nid[n], include_pad_to_die=False) for n in USB}
    check(all(abs(cu[n] - KICAD_NODIE[n]) <= KICAD_PRINT for n in USB),
          'include_pad_to_die=False = KiCad without the token, 2.5294 / 2.4927', cu)

    groups = os.path.join(tmp, 'usb.json')
    with open(groups, 'w', encoding='utf-8') as fh:
        json.dump({'USB': {'nets': USB, 'tolerance_mm': 0.1}}, fh)
    r = board_score.score_length(die, evidence(groups, 'groups file'))
    g = r.get('groups', {}).get('USB', {})
    check(r.get('count') == 1 and g.get('pass') is False
          and abs(g.get('spread_mm', 0) - KICAD_SKEW) <= 2 * KICAD_PRINT,
          'score_length: spread 1.5366 = KiCad skew, and it FAILS at 0.1 mm '
          '(was spread 0.0366, pass)', r)
    check(g.get('pad_to_die_mm') == {'/RP2354A/USB_D+': 1.5},
          'the group discloses the term it graded (pad_to_die_mm)', g)
    r0 = board_score.score_length(RP, groups)
    g0 = r0.get('groups', {}).get('USB', {})
    check(r0.get('count') == 0 and abs(g0.get('spread_mm', 9) - 0.0366) < 1e-4
          and 'pad_to_die_mm' not in g0,
          'control, no token: spread 0.0366, pass, nothing disclosed', r0)

    dp = nid[USB[0]]
    pads = pcb.pads_by_net[dp]
    path = pin_pair_path_length(pcb, dp, pads[0], pads[1])
    check(abs(net_match_length(pcb, dp, mode='pin_pair') - path - 1.5) < 1e-9
          and net_match_pad_to_die(pcb, dp, 'pin_pair') == 1.5,
          'pin_pair mode adds the measured pad pair\'s 1.5 mm')


def _pad(net_id, ref, die=0.0, delay=0.0):
    return Pad(component_ref=ref, pad_number='1', global_x=0, global_y=0,
               local_x=0, local_y=0, size_x=1, size_y=1, shape='rect',
               layers=['F.Cu'], net_id=net_id, net_name='N%d' % net_id,
               pad_type='smd', die_length=die, die_delay=delay)


def test_member_rule():
    print('\n3. member_pad_to_die picks KiCad\'s pads per member kind')
    a1, a2 = _pad(1, 'U1', 1.0, 7.0), _pad(1, 'U2', 0.25)
    m1, m2, m3 = _pad(2, 'U1', 2.0), _pad(2, 'U2', 0.5), _pad(2, 'U3', 4.0)
    p1, p2 = _pad(3, 'U1', 0.75), _pad(3, 'U2')
    n1, n2 = _pad(4, 'U1', 0.5), _pad(4, 'U2', 0.5)
    pcb = SimpleNamespace(
        pads_by_net={1: [a1, a2], 2: [m1, m2, m3], 3: [p1, p2], 4: [n1, n2]},
        nets={i: SimpleNamespace(name='N%d' % i) for i in (1, 2, 3, 4)})
    check(member_pad_to_die({}, 'N1', pcb) == 1.25,
          '2-pad net: both pads (KiCad\'s tuner and DRC agree)')
    check(member_pad_to_die({'from_board': True}, 'N2', pcb) == 6.5,
          'member measured from board copper (total copper): every pad, KiCad\'s DRC')
    info = [(0, 0, 0, 0, 0, m1), (0, 0, 0, 0, 0, m2), (0, 0, 0, 0, 0, m3)]
    check(member_pad_to_die({'is_multipoint': True, 'multipoint_pad_info': info,
                             'routed_pad_indices': {0, 2}}, 'N2', pcb) == 6.0,
          'multipoint net routed this run: the two pads its main edge joins')
    check(member_pad_to_die({'is_diff_pair': True, 'p_net_id': 3, 'n_net_id': 4},
                            'N3', pcb) == 1.0,
          'pair: the larger leg (max(P 0.75, N 1.0)), as KiCad\'s DP tuner')
    check(member_pad_to_die({}, 'N1', pcb, attr='die_delay') == 7.0
          and net_pad_to_die_delay(pcb, 1) == 7.0,
          'the delay sums die_delay (ps), never a converted die_length')
    check(member_pad_to_die({}, 'N1', None) == 0.0,
          'no board: no term')


def _seg(net_id, length, layer='F.Cu'):
    return Segment(start_x=0.0, start_y=float(net_id), end_x=float(length),
                   end_y=float(net_id), width=0.2, layer=layer, net_id=net_id)


def test_pair_rule(tmp):
    print('\n3b. a pair is KiCad\'s DP tuner exactly: per leg, the larger leg')
    # Legs whose COPPER differs (no intra-match): P 10.0 mm, N 11.5 mm, and
    # dies P 1.5, N 0.5. KiCad (DP_MEANDER_PLACER::origPathLength):
    # max(10 + 1.5, 11.5 + 0.5) = 12.0. The old rule, copper + max(1.5, 0.5),
    # read 13.0 on a max-of-legs pair.
    p1, p2 = _pad(3, 'U1', 1.5), _pad(3, 'U2')
    n1, n2 = _pad(4, 'U1', 0.5), _pad(4, 'U2')
    pcb = SimpleNamespace(pads_by_net={3: [p1, p2], 4: [n1, n2]},
                          nets={3: SimpleNamespace(name='DP_P'),
                                4: SimpleNamespace(name='DP_N')})
    legs = [_seg(3, 10.0), _seg(4, 11.5)]
    pair = {'is_diff_pair': True, 'p_net_id': 3, 'n_net_id': 4,
            'new_segments': legs, 'new_vias': [],
            'p_stub_length': 0.0, 'n_stub_length': 0.0}
    intra = dict(pair, route_length=11.5)       # max(P, N): intra-match on
    avg = dict(pair, route_length=10.75)        # the centreline average
    got = (11.5 + member_pad_to_die(intra, 'DP_P', pcb),
           10.75 + member_pad_to_die(avg, 'DP_P', pcb))
    check(all(abs(v - 12.0) < 1e-9 for v in got),
          'unequal legs, different dies: the pair length is max(10+1.5, '
          '11.5+0.5) = 12.0 under either copper convention', got)
    check(abs(member_pad_to_die(dict(intra, p_stub_length=1.0), 'DP_P', pcb)
              - (12.5 - 11.5)) < 1e-9,
          'each leg carries its OWN stub: P 10 + 1.0 stub + 1.5 = 12.5')
    bare = SimpleNamespace(pads_by_net={3: [p2], 4: [n2]}, nets=pcb.nets)
    check(member_pad_to_die(avg, 'DP_P', bare) == 0.0,
          'a pair declaring no term keeps route_length exactly (the '
          'centreline average included), so token-free copper cannot move')

    # Multipoint pair: a chain of two coupled legs, T1 -> T2 (8 mm) and
    # T2 -> T3 (12 mm). The matched span is the longer leg, and KiCad's tuner
    # adds only ITS terminals' pads: P {p_b, p_c} 1.0, N {n_b, n_c} 0.25. T1's
    # 5.0 mm, on a pad that leg never reaches, used to be counted.
    pa, pb, pc = _pad(5, 'U1', 5.0), _pad(5, 'U2'), _pad(5, 'U3', 1.0)
    na, nb, nc = _pad(6, 'U1'), _pad(6, 'U2', 0.25), _pad(6, 'U3')
    mp = SimpleNamespace(pads_by_net={5: [pa, pb, pc], 6: [na, nb, nc]},
                         nets={5: SimpleNamespace(name='M_P'),
                               6: SimpleNamespace(name='M_N')})
    leg1 = {'new_segments': [_seg(5, 8.0), _seg(6, 8.0)], 'new_vias': [],
            'p_net_id': 5, 'n_net_id': 6, '_leg_terms': ((pa, na), (pb, nb))}
    leg2 = {'new_segments': [_seg(5, 12.0), _seg(6, 12.0)], 'new_vias': [],
            'p_net_id': 5, 'n_net_id': 6, '_leg_terms': ((pb, nb), (pc, nc))}
    chain = {'is_diff_pair': True, 'is_multipoint': True, 'p_net_id': 5,
             'n_net_id': 6, 'leg_results': [leg1, leg2], 'route_length': 12.0,
             'new_segments': leg1['new_segments'] + leg2['new_segments']}
    check(abs(member_pad_to_die(chain, 'M_P', mp) - 1.0) < 1e-9,
          'multipoint pair: only the matched span\'s terminals count '
          '(max(12+1.0, 12+0.25) - 12 = 1.0, not every pad\'s 6.0)',
          member_pad_to_die(chain, 'M_P', mp))
    # A polarity swap moves a terminal pad to the other net: the pad counts
    # for the net it is ON now.
    pc.net_id, nc.net_id = 6, 5
    check(abs(member_pad_to_die(chain, 'M_P', mp) - 1.25) < 1e-9,
          'a swapped terminal pad counts for the net it is on now '
          '(N 0.25 + 1.0 = 1.25)', member_pad_to_die(chain, 'M_P', mp))
    pc.net_id, nc.net_id = 5, 6

    # Time: the same rule on each leg's delay and die_delay.
    real = parse_kicad_pcb(_synthetic(os.path.join(tmp, 'v.kicad_pcb'),
                                      [('V1', 10, 10, 'X', '')]))
    tp = SimpleNamespace(pads_by_net={3: [_pad(3, 'U1', delay=30.0), p2],
                                      4: [_pad(4, 'U1', delay=5.0), n2]},
                         nets=pcb.nets, board_info=real.board_info)
    from impedance import calculate_route_propagation_time_ps as tps
    t_p, t_n = tps([legs[0]], [], tp), tps([legs[1]], [], tp)
    want = max(t_p + 30.0, t_n + 5.0) - max(t_p, t_n)
    check(abs(member_pad_to_die(pair, 'DP_P', tp, attr='die_delay') - want) < 1e-6,
          'time: max(P time + 30 ps, N time + 5 ps) over the larger leg\'s '
          'time (%.2f ps, not max(30, 5))' % want)


# ---------------------------------------------------------------- end to end

def _synthetic(path, pads):
    """A KiCad-10 board of single-pad footprints (see test_meander_demo_boards
    for why it carries no top-level net table)."""
    parts = ['(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
             '\t(generator_version "10.0")\n'
             '\t(general (thickness 1.6) (legacy_teardrops no))\n\t(paper "A4")\n'
             '\t(layers (0 "F.Cu" signal) (2 "B.Cu" signal) (1 "F.Mask" user) '
             '(3 "B.Mask" user) (5 "F.SilkS" user "F.Silkscreen") '
             '(7 "B.SilkS" user "B.Silkscreen") (25 "Edge.Cuts" user) '
             '(35 "F.Fab" user) (33 "B.Fab" user))\n'
             '\t(setup (stackup (layer "F.Cu" (type "copper") (thickness 0.035)) '
             '(layer "dielectric 1" (type "core") (thickness 0.15) (material "FR4") '
             '(epsilon_r 4.5) (loss_tangent 0.02)) '
             '(layer "B.Cu" (type "copper") (thickness 0.035))) '
             '(pad_to_mask_clearance 0))']
    for ref, x, y, net, extra in pads:
        parts.append(
            '\t(footprint "DemoPad" (layer "F.Cu") (uuid "%s") (at %s %s)\n'
            '\t\t(property "Reference" "%s" (at 0 -1.5 0) (layer "F.SilkS") '
            '(uuid "%s") (effects (font (size 1 1) (thickness 0.15))))\n'
            '\t\t(attr smd)\n'
            '\t\t(pad "1" smd circle (at 0 0) (size 0.8 0.8) (layers "F.Cu" '
            '"F.Mask") (net "%s") %s (uuid "%s")))'
            % (uuid.uuid4(), x, y, ref, uuid.uuid4(), net, extra, uuid.uuid4()))
    for x1, y1, x2, y2 in ((5, 5, 65, 5), (65, 5, 65, 40), (65, 40, 5, 40), (5, 40, 5, 5)):
        parts.append('\t(gr_line (start %s %s) (end %s %s) (stroke (width 0.1) '
                     '(type default)) (layer "Edge.Cuts") (uuid "%s"))'
                     % (x1, y1, x2, y2, uuid.uuid4()))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(parts) + '\n)\n')
    return evidence(path, 'synthetic board')


def _run(argv, what):
    p = subprocess.run([sys.executable, '-X', 'utf8'] + argv, cwd=ROOT,
                       capture_output=True, text=True, timeout=600)
    ok = p.returncode == 0 and 'Traceback' not in p.stderr
    check(ok, '%s ran (exit 0, no traceback)' % what,
          'exit %s: %s' % (p.returncode, (p.stderr or p.stdout)[-800:]))
    return p.stdout if ok else None


def _lengths(board, nets, delay=False):
    from impedance import calculate_route_propagation_time_ps
    pcb = parse_kicad_pcb(evidence(board, 'routed board'))
    nid = ids(pcb)
    out = {}
    for n in nets:
        if delay:
            segs = [s for s in pcb.segments if s.net_id == nid[n]]
            vias = [v for v in pcb.vias if v.net_id == nid[n]]
            cu = calculate_route_propagation_time_ps(segs, vias, pcb)
            out[n] = (cu + net_pad_to_die_delay(pcb, nid[n]), cu)
        else:
            out[n] = (net_match_length(pcb, nid[n]),
                      net_match_length(pcb, nid[n], include_pad_to_die=False))
    return out


SE_PADS = [('P1', 10, 12, 'SE1', '%s'), ('P2', 50, 12, 'SE1', ''),
           ('P3', 10, 18, 'SE2', ''), ('P4', 50, 18, 'SE2', '')]
COMMON = ['--track-width', '0.2', '--clearance', '0.15']


def test_route_group(tmp):
    print('\n4a. route.py --length-match-group meanders to the die-inclusive target')
    src = _synthetic(os.path.join(tmp, 'se.kicad_pcb'),
                     [(r, x, y, n, e % '(die_length 3.0)' if '%s' in e else e)
                      for r, x, y, n, e in SE_PADS])
    out = os.path.join(tmp, 'se_out.kicad_pcb')
    log = _run(['py_router/route.py', src, out, '--nets', 'SE1', 'SE2'] + COMMON
               + ['--length-match-group', 'SE*', '--length-match-tolerance', '0.1'],
               'route.py --length-match-group')
    if log is None:
        return
    check('Pad-to-die included (#1248): SE1 +3.000mm' in log,
          'the run says which term it added')
    L = _lengths(out, ['SE1', 'SE2'])
    spread = abs(L['SE1'][0] - L['SE2'][0])
    # 0.1 tolerance plus the meander's own quantization (bumps scale to the
    # target but stop at the closest result, #520).
    check(spread <= 0.15, 'KiCad-measured spread %.3f mm is within tolerance' % spread, L)
    check(abs((L['SE2'][1] - L['SE1'][1]) - 3.0) <= 0.15,
          'witness: SE2\'s COPPER is ~3.0 mm longer than SE1\'s (the die term '
          'moved real copper)', L)


def test_route_time(tmp):
    print('\n4b. route.py --time-matching adds die_delay')
    src = _synthetic(os.path.join(tmp, 'tm.kicad_pcb'),
                     [(r, x, y, n, e % '(die_delay 20)' if '%s' in e else e)
                      for r, x, y, n, e in SE_PADS])
    out = os.path.join(tmp, 'tm_out.kicad_pcb')
    log = _run(['py_router/route.py', src, out, '--nets', 'SE1', 'SE2'] + COMMON
               + ['--length-match-group', 'SE*', '--time-matching',
                  '--time-match-tolerance', '1.0'],
               'route.py --time-matching')
    if log is None:
        return
    check('Pad-to-die delay included (#1248): SE1 +20.000ps' in log,
          'the run says which delay it added')
    T = _lengths(out, ['SE1', 'SE2'], delay=True)
    skew = abs(T['SE1'][0] - T['SE2'][0])
    check(skew <= 1.5, 'delay skew %.2f ps is within tolerance' % skew, T)
    check(abs((T['SE2'][1] - T['SE1'][1]) - 20.0) <= 1.5,
          'witness: SE2\'s copper delay is ~20 ps longer than SE1\'s', T)


def test_route_diff_intra(tmp):
    print('\n4c. route_diff --diff-pair-intra-match matches P + its die to N')
    src = _synthetic(os.path.join(tmp, 'dp.kicad_pcb'), [
        ('P5', 10, 30, 'DP_P', '(die_length 1.5)'), ('P6', 10, 32, 'DP_N', ''),
        ('P7', 50, 30, 'DP_P', ''), ('P8', 50, 32, 'DP_N', '')])
    out = os.path.join(tmp, 'dp_out.kicad_pcb')
    log = _run(['py_router/route_diff.py', src, out, 'DP_P', 'DP_N'] + COMMON
               + ['--diff-pair-gap', '0.15', '--diff-pair-intra-match'],
               'route_diff.py --diff-pair-intra-match')
    if log is None:
        return
    L = _lengths(out, ['DP_P', 'DP_N'])
    skew = abs(L['DP_P'][0] - L['DP_N'][0])
    check(skew <= 0.15, 'KiCad-measured P/N skew %.3f mm is within tolerance' % skew, L)
    check(abs((L['DP_N'][1] - L['DP_P'][1]) - 1.5) <= 0.15,
          'witness: N\'s COPPER is ~1.5 mm longer than P\'s', L)


#: Two pairs in one group, routed without intra-match. DA makes a U-turn, so
#: its P leg's copper is ~1.2 mm longer than N's, and N declares 2.0 mm of
#: pad-to-die; DB runs straight with none, and is the longer pair.
#: tests/gui_parity/test_1248_die_length_parity.py routes the same board and
#: has kicad-cli's DRC referee it.
PAIR_GROUP_PADS = [
    ('A1', 10, 12, 'DA_P', ''), ('A2', 10, 13.2, 'DA_N', '(die_length 2.0)'),
    ('A3', 10, 30, 'DA_P', ''), ('A4', 10, 28.8, 'DA_N', ''),
    ('B1', 23, 34, 'DB_P', ''), ('B2', 23, 35.2, 'DB_N', ''),
    ('B3', 50, 34, 'DB_P', ''), ('B4', 50, 35.2, 'DB_N', '')]
PAIR_GROUP_ARGV = ['DA_P', 'DA_N', 'DB_P', 'DB_N', '--track-width', '0.2',
                   '--clearance', '0.15', '--diff-pair-gap', '0.15',
                   '--length-match-group', 'D*', '--length-match-tolerance', '0.1']


def test_route_pair_group(tmp):
    print('\n4d. route_diff --length-match-group, no intra-match: a pair is its '
          'larger leg, die included')
    src = _synthetic(os.path.join(tmp, 'pg.kicad_pcb'), PAIR_GROUP_PADS)
    out = os.path.join(tmp, 'pg_out.kicad_pcb')
    log = _run(['py_router/route_diff.py', src, out] + PAIR_GROUP_ARGV,
               'route_diff.py --length-match-group (two pairs)')
    if log is None:
        return
    check('(pair, larger leg of' in log, 'the run names the pair\'s legs')
    L = _lengths(out, ['DA_P', 'DA_N', 'DB_P', 'DB_N'])
    check(L['DA_P'][1] - L['DA_N'][1] > 0.8,
          'witness: DA\'s legs are NOT matched (P copper %.3f vs N %.3f)'
          % (L['DA_P'][1], L['DA_N'][1]), L)
    a = max(L['DA_P'][0], L['DA_N'][0])
    b = max(L['DB_P'][0], L['DB_N'][0])
    check(abs(a - b) <= 0.15,
          'KiCad-measured pairs match: DA %.3f vs DB %.3f (each its larger '
          'leg, die included)' % (a, b), L)
    old = (L['DA_P'][1] + L['DA_N'][1]) / 2 + 2.0
    check(abs(old - a) >= 0.3,
          'witness: the old rule (centreline + the larger die, %.3f) is %.3f mm '
          'off KiCad\'s DA, outside the tolerance' % (old, abs(old - a)))


def test_board_score_cli(tmp):
    print('\n5. board_score --length-groups end to end')
    die = die_board(tmp)
    groups = os.path.join(tmp, 'usb_cli.json')
    with open(groups, 'w', encoding='utf-8') as fh:
        json.dump({'USB': {'nets': USB, 'tolerance_mm': 0.1}}, fh)
    out = os.path.join(tmp, 'score.json')
    # board_score exits 4 whenever blocking > 0 (this board has DRC findings
    # of its own), so the exit code says nothing here: read the document.
    p = subprocess.run([sys.executable, '-X', 'utf8', 'py_tools/board_score.py', die,
                        '--length-groups', groups, '--json', out],
                       cwd=ROOT, capture_output=True, text=True, timeout=600)
    check('Traceback' not in p.stderr, 'board_score ran without a traceback',
          p.stderr[-800:])
    try:
        d = json.load(open(evidence(out, 'board_score --json'), encoding='utf-8'))
    except AssertionError as e:
        check(False, 'board_score wrote its JSON', e)
        return
    g = d['components']['length']['groups']['USB']
    check(d['blocking_by'].get('length') == 1 and g.get('pass') is False,
          'blocking_by.length is 1: the skew KiCad reports is a blocker here too', g)
    audit = d.get('net_name_audit') or {}
    check(audit.get('ran') is True and audit.get('unknown_count') == 0
          and 'components.length.groups[].pad_to_die_mm' in audit.get('fields_checked', []),
          'the published pad_to_die_mm net names are audited and all exist', audit)


def main():
    tmp = tempfile.mkdtemp(prefix='t1248_')
    try:
        test_parse(tmp)
        test_kicad_reference(tmp)
        test_member_rule()
        test_pair_rule(tmp)
        test_route_group(tmp)
        test_route_time(tmp)
        test_route_diff_intra(tmp)
        test_route_pair_group(tmp)
        test_board_score_cli(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('\n%d failure(s)' % len(FAILURES))
    for f in FAILURES:
        print('  FAILED: %s' % f)
    if not FAILURES:
        print('PASS  #1248 pad-to-die is parsed, matched to and graded as KiCad counts it')
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
