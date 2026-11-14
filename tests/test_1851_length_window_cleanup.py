#!/usr/bin/env python3
"""#1851: a net a `.kicad_dru` length rule holds to a window is read-only to
the graze passes, and a late pass that moves it out of its window says so.

A skew-graded net had both protections (#1759, #1798). A net held to a
length window had neither: the post-route cleanup's length-changing passes
could re-bend its input copper, and when an end-of-run pass moved it out of
its window the step printed nothing and reported nothing. check_drc and
KiCad still graded the shipped board.

The chain is tests/test_1798_late_collapse_skew.py's: an underpad QFN
fanout, then route_diff. A redundant detour is added beside one segment of
CSI_D0_N (that test's `add_detour`), the pair record is taken off the board
and a length rule binds CSI_D0_N with a window 0.1 mm either side of its
length WITH the detour. The all-nets `route.py --nets '*'` step's end-of-run
collapse removes the detour, which shortens the net by more than 0.1 mm.

Rows:
  1. the input grades the net inside its window;
  2. the net is in the graze passes' read-only set, with the skew-graded
     nets, and the comparison (`length_moved_out_of_window`) reports a net
     moved from inside to outside, one already out and now further, and
     neither a net that stayed inside nor one that moved back in;
  3. CLI: the step names the net in a WARNING and in --json-out's
     `length_moved_by_cleanup`, with the lengths either side;
  4. the row is true: check_drc grades the shipped board's net out of range;
  5. the GUI leg (batch_route in memory) carries the same row;
  6. the ROUTED path (the run routes a net, so the post-route cleanup runs):
     the USB pair of the #1759 chain, its record taken off and a length rule
     holding /D_P to a window 0.1 mm either side of its matched length, with
     one foreign via inside the bump's clearance. The re-bend keeps the held
     net's input copper and the net ships inside its window; the same board
     with the rule on another net (so /D_P is held by nothing) re-bends the
     bump and leaves the window, which is what the hold prevents;
  7. the hold YIELDS to copper that grazes a foreign net: the CSI chain with
     CSI_CLK_N's tracks taken off (so it is routed), where the detour on
     CSI_D0_N grazes its partner. The graze prune removes it, the step says
     the hold yielded, the board ships with no clearance violation, and the
     report names the net the removal moved out of its window.

    python3 tests/test_1851_length_window_cleanup.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence, tool                             # noqa: E402
import test_1798_late_collapse_skew as chain                     # noqa: E402
import test_1759_cleanup_skew_disclosed as usb                   # noqa: E402

FAILS = []
NET = 'CSI_D0_N'


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def strip_net_tracks(src, dst, net):
    """Copy `src` (with its siblings) and take every track of `net` off the
    copy, so the next step routes it. Returns the number taken off."""
    chain.py(tool('copy_board.py'), src, dst)
    with open(evidence(dst, 'board copy'), encoding='utf-8') as fh:
        t = fh.read()
    out, i, n = [], 0, 0
    while True:
        j = t.find('\t(segment', i)
        if j < 0:
            out.append(t[i:])
            break
        depth, k = 0, t.index('(', j)
        while True:
            depth += {'(': 1, ')': -1}.get(t[k], 0)
            if depth == 0:
                break
            k += 1
        block = t[j:k + 1]
        out.append(t[i:j])
        if f'(net "{net}")' in block:
            n += 1
        else:
            out.append(block)
        i = k + 1
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(''.join(out))
    return n


def drc(path):
    """check_drc at the clearance the CSI chain routes to: (the violations
    other than the length rule's that name CSI_D0_N, the length rule's
    violations)."""
    js = path[:-len('.kicad_pcb')] + '.drc.json'
    chain.graded(tool('check_drc.py'), evidence(path, 'board'), '-c', '0.1',
                 '--json', js)
    with open(evidence(js, 'check_drc --json'), encoding='utf-8') as fh:
        items = json.load(fh)['items']
    mine = [i for i in items if NET in (i.get('net1'), i.get('net2'))]
    return ([i for i in mine if i['type'] != 'length-out-of-range'],
            [i for i in mine if i['type'] == 'length-out-of-range'])


def routed_hold(tmp):
    """Row 6: the hold on the routed path keeps a length-window net's input
    copper from the re-bend."""
    a, b, p, q, c, d = (os.path.join(tmp, f'u{t}.kicad_pcb') for t in 'abpqcd')
    chain.py(tool('copy_board.py'), os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), a)
    r = chain.py(tool('route_diff.py'), evidence(a, 'esp_prog copy'), b,
                 '--nets', '/D_P', '/D_N', *usb.G, '--diff-pair-gap', '0.25',
                 '--no-gnd-vias', '--diff-pair-intra-match',
                 '--length-match-tolerance', '0.127')
    if 'bumps added' not in r.stdout:
        print("  [BROKEN TEST] route_diff added no bump: the meander the re-bend "
              "would straighten is not there")
        return 1
    with open(evidence(b, 'matched board'), encoding='utf-8') as fh:
        t = fh.read()
    i = t.rstrip().rfind(')')
    with open(b, 'w', encoding='utf-8') as fh:
        fh.write(t[:i] + usb.VIA + t[i:])
    held = chain.length_rule_board(b, p, '/D_P', '/D_P')
    chain.length_rule_board(b, q, '/D_N', '/D_P')   # the control: /D_P unheld
    from kicad_parser import parse_kicad_pcb
    from cleanup_pipeline import _skew_bound_net_ids, _length_window_net_ids
    pp, pq = parse_kicad_pcb(p), parse_kicad_pcb(q)
    ids = {n.name: k for k, n in pp.nets.items()}
    if _skew_bound_net_ids(pp) or _length_window_net_ids(pp) != {ids['/D_P']}:
        print("  [BROKEN TEST] the held board does not hold /D_P by its length "
              "rule alone")
        return 1
    if ids['/D_P'] in _length_window_net_ids(pq):
        print("  [BROKEN TEST] the control board holds /D_P")
        return 1
    r = chain.py(tool('route.py'), p, c, '--nets', '*', *usb.G)
    log = r.stdout + r.stderr
    if not re.search(r'SUCCESS: \d+ segments', log):
        print("  [BROKEN TEST] the step routed nothing: the post-route cleanup "
              "this row grades did not run\n" + log[-800:])
        return 1
    rq = chain.py(tool('route.py'), q, d, '--nets', '*', *usb.G)
    from net_queries import net_match_length

    def dp_len(path):
        g = parse_kicad_pcb(evidence(path, 'routed board'))
        return net_match_length(g, next(k for k, n in g.nets.items()
                                         if n.name == '/D_P'), 'total')
    lc, ld = dp_len(c), dp_len(d)
    print(f"  /D_P {held:.4f} mm in; held {lc:.4f} mm, unheld control {ld:.4f} mm")
    check("control: unheld, the re-bend straightens the bump out of the window",
          'Graze nudge: re-bent' in rq.stdout and held - ld > 0.1,
          f"{held:.4f} -> {ld:.4f}")
    check("the graze passes name the held net",
          re.search(r"Graze passes: 1 skew- or length-graded net\(s\) keep their "
                    r"input copper \(#1759, #1851\): /D_P", log) is not None)
    check("held, the net keeps its length (the re-bend left its input copper)",
          abs(lc - held) < 1e-3, f"{held:.4f} -> {lc:.4f}")
    out = chain.graded(tool('check_drc.py'), evidence(c, 'routed board'))
    check("check_drc grades it inside its window",
          '1 net(s) graded, 0 out of range' in out,
          (re.search(r'Length rules[^\n]*', out) or [''])[0][:160])
    return 0


def routed_yield(tmp):
    """Row 7: the hold yields to input copper that grazes a foreign net."""
    e, le, ler, out = (os.path.join(tmp, f'{t}.kicad_pcb')
                       for t in ('ye', 'yle', 'yler', 'yout'))
    b = os.path.join(tmp, 'b.kicad_pcb')
    chain.add_detour(evidence(b, 'route_diff board'), e)
    chain.length_rule_board(evidence(e, 'detour board'), le, NET, NET)
    if not strip_net_tracks(le, ler, 'CSI_CLK_N'):
        print("  [BROKEN TEST] CSI_CLK_N has no tracks to take off")
        return 1
    clr_in, _ = drc(ler)
    if not any(i['type'] == 'segment-segment' for i in clr_in):
        print("  [BROKEN TEST] the detour grazes nothing on the input: no "
              "graze for the hold to yield to")
        return 1
    js = os.path.join(tmp, 'yout.json')
    r = chain.py(tool('route.py'), ler, out, '--nets', '*', '--json-out', js)
    log = r.stdout + r.stderr
    if 'Graze passes:' not in log or not re.search(r'SUCCESS: \d+ segments', log):
        print("  [BROKEN TEST] the routed path or its held set did not run\n"
              + log[-800:])
        return 1
    check("the step says the hold yielded on the net",
          re.search(r"Graze prune: the hold yields on 1 graded net\(s\), whose "
                    r"input copper grazed a foreign net \(#1851\): " + NET, log)
          is not None)
    clr_out, length_out = drc(out)
    print(f"  violations naming {NET} besides its length: input {len(clr_in)}, "
          f"shipped {len(clr_out)}")
    check("the shipped board has no clearance violation on the net, and its "
          "length is graded out of its window",
          not clr_out and len(length_out) == 1, str(clr_out)[:300])
    with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
        doc = json.load(fh)
    check("the net the removal moved out of its window is named",
          [x['net'] for x in doc.get('length_moved_by_cleanup') or []] == [NET]
          and f"moved {NET} out of its length rule" in log,
          str(doc.get('length_moved_by_cleanup')))
    return 0


def main():
    tmp = tempfile.mkdtemp(prefix='t1851_')
    try:
        a, f, b, e, le, lec = (os.path.join(tmp, f'{t}.kicad_pcb')
                               for t in ('a', 'f', 'b', 'e', 'le', 'lec'))
        chain.py(tool('copy_board.py'), chain.BOARD, a)
        chain.py(tool('qfn_fanout.py'), evidence(a, 'board copy'), '-o', f,
                 '--component', 'U1', '--nets', 'CSI_*', '--escape-method', 'underpad',
                 '--layer', 'In1.Cu', '--width', '0.127', '--via-size', '0.5',
                 '--via-drill', '0.2', '--allow-via-in-pad')
        chain.py(tool('route_diff.py'), evidence(f, 'fanout board'), b,
                 '--nets', 'CSI_D0*', 'CSI_CLK*', '--layers', 'F.Cu', 'In1.Cu',
                 '--track-width', '0.127', '--diff-pair-gap', '0.2', '--via-size', '0.6',
                 '--diff-pair-intra-match')
        chain.add_detour(evidence(b, 'route_diff board'), e)
        with_detour = chain.length_rule_board(evidence(e, 'detour board'), le, NET, NET)

        print("1. the input")
        out = chain.graded(tool('check_drc.py'), evidence(le, 'length-rule board'),
                           '-c', '0.1')
        if '1 net(s) graded, 0 out of range' not in out:
            print("  [BROKEN TEST] the length rule does not grade its net inside "
                  "the window on the input:\n" + out[-800:])
            return 1
        print(f"  {NET}: {with_detour:.4f} mm, window +-0.1 mm")

        print("2. the read-only set and the comparison")
        from kicad_parser import parse_kicad_pcb
        from cleanup_pipeline import (_length_window_net_ids, _skew_bound_net_ids,
                                      _centre_stub_net_ids)
        from check_drc import length_moved_out_of_window as moved
        pl = parse_kicad_pcb(le)
        nid = next(k for k, n in pl.nets.items() if n.name == NET)
        check("no skew rule grades the board's nets", not _skew_bound_net_ids(pl))
        check("the net is a length-window net, and keeps its centre stubs too",
              _length_window_net_ids(pl) == {nid} and nid in _centre_stub_net_ids(pl))

        def row(length, lo=10.0, hi=11.0):
            ex = max(lo - length, length - hi, 0.0)
            return {'net': 'N', 'rule': 'r', 'unit': 'mm', 'length': length,
                    'min': lo, 'max': hi, 'excess': ex}
        got = moved({'N': row(10.5)}, {'N': row(9.8)})
        check("inside -> outside is a row, was_within",
              len(got) == 1 and got[0]['was_within'] and got[0]['length_after'] == 9.8)
        got = moved({'N': row(9.9)}, {'N': row(9.5)})
        check("outside -> further out is a row, not was_within",
              len(got) == 1 and not got[0]['was_within'])
        check("inside -> inside, outside -> inside and an unmeasured net are not",
              moved({'N': row(10.5)}, {'N': row(10.2)}) == []
              and moved({'N': row(9.5)}, {'N': row(10.2)}) == []
              and moved({'N': row(10.5)}, {'N': None}) == [])

        print("3. the all-nets step (CLI)")
        js = os.path.join(tmp, 'lec.json')
        r = chain.py(tool('route.py'), le, lec, '--nets', '*', '--json-out', js)
        log = r.stdout + r.stderr
        if 'Strict collapse (#1063, end of run)' not in log:
            print("  [BROKEN TEST] the end-of-run collapse removed nothing: the "
                  "detour this row needs removed is not\n" + log[-800:])
            return 1
        with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
            doc = json.load(fh)
        rows = [x for x in doc.get('length_moved_by_cleanup') or [] if x['net'] == NET]
        check("--json-out names the net", len(rows) == 1 and rows[0]['was_within']
              and abs(rows[0]['length_before'] - with_detour) < 1e-3
              and rows[0]['length_after'] < rows[0]['min'],
              str(doc.get('length_moved_by_cleanup')))
        check("the WARNING names it", re.search(
            r"WARNING: the end-of-run cleanup moved " + NET
            + r" out of its length rule 'stub len'", log) is not None,
            ' | '.join(l.strip() for l in log.splitlines() if 'length rule' in l)[-300:])

        print("4. the row is true")
        out = chain.graded(tool('check_drc.py'), evidence(lec, 'shipped board'), '-c', '0.1')
        check("check_drc grades the shipped net out of range",
              '1 net(s) graded, 1 out of range' in out,
              (re.search(r'Length rules[^\n]*', out) or [''])[0][:160])

        print("5. the GUI leg")
        from route import batch_route
        pcb = parse_kicad_pcb(le)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            _ok, _fail, _t, rd = batch_route(
                le, '', [n.name for n in pcb.nets.values() if n.name],
                pcb_data=pcb, return_results=True)
        rows = [x for x in rd.get('length_moved_by_cleanup') or [] if x['net'] == NET]
        check("results_data names the net", len(rows) == 1,
              str(rd.get('length_moved_by_cleanup')) + buf.getvalue()[-300:])

        print("6. the routed path: the hold keeps the input copper")
        if routed_hold(tmp):
            return 1

        print("7. the hold yields to copper that grazes a foreign net")
        if routed_yield(tmp):
            return 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}): " + '; '.join(FAILS))
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
