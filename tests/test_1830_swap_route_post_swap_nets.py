"""#1830: a polarity pad-swap route is judged on the board the swap leaves.

A pad-swap route lays P onto the pad that is N's until the swap and N onto
P's; `apply_polarity_swap` exchanges the two pads' nets afterwards. The checks
that judge the route's copper against foreign copper ran before that, on the
board as it stood: the #165 graze check read P landing on the pad it will own
as P grazing the partner's pad, so every swap whose end is a bare pad failed
(allowed, or probed under #1802's denied-swap rule), and the terminal neck
read the same pad as foreign copper and narrowed both terminals to the fab
floor. They now read `_pending_swap`'s view: the two target pads, and the stub
copper the swap relabels, under their post-swap nets.

The fixture is an in-repo board with two connector pads of one pair exchanged,
so the pair's two ends are genuinely opposite-handed.

    1. the view: P ending on the pad it will own is clean on the view (and
       grazes on the board as it stands, the defect); P over the pad N
       will own still grazes, named under N; an own-pair via on the stub copper
       the swap relabels is read under its post-swap net; the board itself is
       untouched.
    2. swaps allowed: the swap route wins -- the pair is polarity-swapped,
       coupled end to end (no hybrid escape), no terminal narrowed, DRC clean
       at the routed clearance, connected.
    3. swaps denied: the probe routes, so the pair is listed
       polarity-swap-denied (#1802's rule), and its copper is the hybrid's.

    python3 -X utf8 tests/test_1830_swap_route_post_swap_nets.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-X', 'utf8', '-B']
BOARD = 'glasgow_revC.kicad_pcb'
REF, PAD_A, PAD_B = 'J5', '27', '29'
P_NAME, N_NAME = '/IO_Banks/Z3_P', '/IO_Banks/Z3_N'
PAIR = '/IO_Banks/Z3'
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def _block_end(s, i):
    depth = 0
    for j in range(i, len(s)):
        depth += {'(': 1, ')': -1}.get(s[j], 0)
        if depth == 0 and s[j] == ')':
            return j + 1
    raise AssertionError('BROKEN TEST: unbalanced block')


def exchange_pad_nets(path, ref, a, b):
    """Exchange the (net ...) of pads `a` and `b` of footprint `ref`."""
    with open(path, encoding='utf-8', newline='') as fh:
        t = fh.read()
    m = re.search(r'\(property "Reference" "%s"' % re.escape(ref), t)
    if not m:
        raise AssertionError(f'BROKEN TEST: no footprint {ref} in the fixture')
    fs = t.rfind('(footprint ', 0, m.start())
    fe = _block_end(t, fs)
    fp = t[fs:fe]
    spans = []
    for pad in (a, b):
        pm = re.search(r'\(pad "%s" ' % re.escape(pad), fp)
        nm = re.search(r'\(net [^()]*\)', fp[pm.start():_block_end(fp, pm.start())])
        spans.append((pm.start() + nm.start(), pm.start() + nm.end(), nm.group(0)))
    (sa, ea, na), (sb, eb, nb) = sorted(spans)
    if na == nb:
        raise AssertionError('BROKEN TEST: the two pads already share a net')
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(t[:fs] + fp[:sa] + nb + fp[ea:sb] + na + fp[eb:] + t[fe:])


def fixture(td):
    dst = os.path.join(td, 'swapped.kicad_pcb')
    run_utils.check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'),
                          os.path.join(REPO, 'kicad_files', BOARD), dst], accept=True)
    run_utils.evidence(dst)
    exchange_pad_nets(dst, REF, PAD_A, PAD_B)
    return dst


def the_view(board):
    print('1. the view a swap route is judged on')
    from kicad_parser import parse_kicad_pcb, Segment, Via
    from routing_config import GridRouteConfig
    from diff_pair_routing import _connector_grazes_foreign_copper, _pending_swap
    pcb = parse_kicad_pcb(board)
    nid = {n.name: i for i, n in pcb.nets.items()}
    p_id, n_id = nid[P_NAME], nid[N_NAME]
    pads = {p.pad_number: p for p in pcb.footprints[REF].pads}
    # after the exchange, the pad the swap gives P is N's, and the reverse
    p_will_own = next(p for p in pads.values() if p.net_id == n_id)
    n_will_own = next(p for p in pads.values() if p.net_id == p_id)
    # an own-pair via on N's stub at that end: the swap relabels it P's
    sx, sy = p_will_own.global_x, p_will_own.global_y + 3.0
    stub = Segment(start_x=p_will_own.global_x, start_y=p_will_own.global_y,
                   end_x=sx, end_y=sy, width=0.2, layer='F.Cu', net_id=n_id)
    via = Via(x=sx, y=sy, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=n_id)
    pcb.segments.append(stub)
    pcb.vias.append(via)
    swap_info = {'p_pos': (n_will_own.global_x, n_will_own.global_y),
                 'n_pos': (p_will_own.global_x, p_will_own.global_y),
                 'p_net_id': p_id, 'n_net_id': n_id}
    segs_before, vias_before = list(pcb.segments), list(pcb.vias)
    pend = _pending_swap(pcb, swap_info)
    view = pend[2]
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=0.2, track_width=0.2)

    def p_seg(x0, y0, x1, y1):
        return Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1, width=0.2,
                       layer='F.Cu', net_id=p_id)

    # P's connector ending inside the pad the swap gives it, from below
    land = p_seg(p_will_own.global_x, p_will_own.global_y + 2.0,
                 p_will_own.global_x, p_will_own.global_y + 1.0)
    on_board = _connector_grazes_foreign_copper([land], pcb, p_id, n_id, cfg)
    check('on the board as it stands, P landing on the pad it will own reads as a '
          'graze of the partner (the defect)',
          bool(on_board) and on_board[0] == 'pad' and on_board[1] is p_will_own,
          f'{on_board}')
    on_view = _connector_grazes_foreign_copper([land], view, p_id, n_id, cfg)
    check('on the view it is clean', on_view is None, f'{on_view}')
    # P over the pad N will own (along its long axis, clear of its neighbours)
    across = p_seg(n_will_own.global_x, n_will_own.global_y - 0.5,
                   n_will_own.global_x, n_will_own.global_y + 0.5)
    g = _connector_grazes_foreign_copper([across], view, p_id, n_id, cfg)
    check('P over the pad N will own still grazes, named under N',
          bool(g) and g[0] == 'pad' and g[1].pad_number == n_will_own.pad_number
          and g[1].net_id == n_id and g[1].net_name == N_NAME, f'{g}')
    # P onto the relabelled stub via
    to_via = p_seg(sx + 1.0, sy, sx, sy)
    g = _connector_grazes_foreign_copper([to_via], pcb, p_id, n_id, cfg)
    check('on the board as it stands, P onto N\'s stub via grazes it',
          bool(g) and g[0] == 'via', f'{g}')
    g = _connector_grazes_foreign_copper([to_via], view, p_id, n_id, cfg)
    check('on the view that via is P\'s: clean', g is None, f'{g}')
    check('the board is untouched: pad nets, segment and via lists',
          p_will_own.net_id == n_id and n_will_own.net_id == p_id
          and p_will_own in pcb.pads_by_net[n_id] and n_will_own in pcb.pads_by_net[p_id]
          and pcb.segments == segs_before and pcb.vias == vias_before
          and stub.net_id == n_id and via.net_id == n_id)


def _route(td, board, tag, extra):
    out, js = os.path.join(td, tag + '.kicad_pcb'), os.path.join(td, tag + '.json')
    r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'route_diff.py'), board, out,
                              '--nets', '/IO_Banks/Z3_*', '--no-bga-zones',
                              '--json-out', js] + extra, accept=True)
    run_utils.evidence(js)
    with open(js, encoding='utf-8') as fh:
        doc = json.load(fh)
    rep = next((p for p in doc.get('pair_reports', []) if p.get('pair') == PAIR), {})
    return r, out, doc, rep


def allowed(td, board):
    print('2. swaps allowed: the swap route wins')
    r, out, doc, rep = _route(td, board, 'allow', ['--polarity-swap-nets', '*'])
    if 'Polarity' not in r.stdout:
        raise AssertionError('BROKEN TEST: the fixture no longer asks for a polarity '
                             'swap, so this tests nothing\n' + r.stdout[-2000:])
    check('no connector graze on the partner', 'Connector grazes' not in r.stdout,
          next((ln for ln in r.stdout.splitlines() if 'Connector grazes' in ln), ''))
    check('the pair is polarity-swapped', doc.get('polarity_swapped_pairs') == [PAIR],
          f"{doc.get('polarity_swapped_pairs')}")
    check('coupled end to end, not the hybrid escape',
          rep.get('outcome') == 'coupled' and 'escape' not in rep,
          f"{rep.get('outcome')}/{rep.get('escape')}")
    skew = (rep.get('copper') or {}).get('skew_mm')
    check('skew under 0.1 mm', skew is not None and skew < 0.1, f'{skew}')
    narrowed = [n for n in (doc.get('design_rules') or {}).get('narrowed', [])
                if n.get('net_name') in (P_NAME, N_NAME)]
    check('no terminal of the pair narrowed', not narrowed, f'{narrowed}')
    d = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                              '--nets', '/IO_Banks/Z3_*'], accept=True)
    check('DRC clean on the pair at the routed clearance',
          'NO DRC VIOLATIONS FOUND' in d.stdout, d.stdout[-800:])
    c = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'check_connected.py'), out,
                              '--nets', '/IO_Banks/Z3_*'], accept=True)
    check('fully connected', 'ALL NETS FULLY CONNECTED' in c.stdout, c.stdout[-800:])


def denied(td, board):
    print('3. swaps denied: the probe routes, so the pair is polarity-swap-denied')
    r, _out, doc, rep = _route(td, board, 'deny', [])
    row = next((ln.strip() for ln in r.stdout.splitlines()
                if ln.strip().startswith('pad swap:')), '')
    if not row:
        raise AssertionError('BROKEN TEST: no ladder verdict, so this tests nothing\n'
                             + r.stdout[-2000:])
    check('the pad swap row: not allowed, and the probe would route',
          row == 'pad swap: not allowed for this pair (--polarity-swap-nets); '
                 'probed, the swap would route', row)
    check('listed in polarity_swap_denied_pairs',
          doc.get('polarity_swap_denied_pairs') == [PAIR],
          f"{doc.get('polarity_swap_denied_pairs')}")
    check('nothing swapped: it ships through the hybrid escape',
          doc.get('polarity_swapped_pairs') == [] and rep.get('escape') == 'hybrid',
          f"{doc.get('polarity_swapped_pairs')}/{rep.get('escape')}")


def main():
    td = tempfile.mkdtemp(prefix='t1830_')
    try:
        board = fixture(td)
        the_view(board)
        allowed(td, board)
        denied(td, board)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'\nFAILED {len(failures)}: ' + '; '.join(failures))
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
