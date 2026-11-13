#!/usr/bin/env python3
"""#1798: a later route.py step leaves a matched pair's recorded skew alone,
or names the group its end-of-run passes moved.

The chain: an underpad QFN fanout, then route_diff matches and records two
pairs. One member's track enters a via off its centre, so the soft-joint
pass cut it back to the via's rim and laid the rim-to-centre stub (#1235)
that KiCad's length DRC measures the run into the via by. The next step,
`route.py --nets '*'`, had nothing to route, and its end-of-run strict
collapse (#1063) removed that stub as redundant: the member measured a via
radius shorter, the pair failed its recorded skew, and nothing was printed.

Rows:
  1. the route_diff board grades both pairs within their recorded tolerance,
     and carries the stub (a track from a via's rim to its centre);
  2. check_weird does not call the stub removable (the collapse and the
     checker share StrictRemovalModel, so both keep it) -- on a net a skew
     rule grades, as this one is; on any other net the model removes it;
  3. after `route.py --nets '*'` the member's KiCad length is unchanged and
     check_drc grades 0 nets out of range;
  4. the disclosure, CLI: a redundant detour on a net a `.kicad_dru` skew
     rule grades is removed by the end-of-run collapse, and the group it
     moves out of range is in the WARNING and in --json-out's
     `skew_moved_by_cleanup`;
  5. ...and the GUI leg (batch_route in memory) carries the same row;
  6. control: `--keep-input-copper` prints neither late-pass line;
  7. the same stub with the pair's record taken off the board and a
     `.kicad_dru` length rule binding its net: check_weird and the all-nets
     step keep it and check_drc grades the net inside its window. With the
     rule on another net the stub is removable, as on any ungraded net.

    python3 tests/test_1798_late_collapse_skew.py
"""
import contextlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []
BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_csi_underpad_diff.kicad_pcb')


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def py(*argv):
    return run_check([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                     accept=True)


def graded(*argv):
    """A grader whose exit code carries findings: run it, refuse a crash."""
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError('BROKEN TEST: the grader crashed\n' + out[-1500:])
    return out


def net_len_mm(path, name):
    from kicad_parser import parse_kicad_pcb
    from net_queries import kicad_net_length_nm
    p = parse_kicad_pcb(path)
    i = next(k for k, n in p.nets.items() if n.name == name)
    return kicad_net_length_nm(p, [s for s in p.segments if s.net_id == i],
                               [v for v in p.vias if v.net_id == i],
                               p.nets[i].pads) / 1e6


def centre_stubs(path):
    """(net, segment) for every track running from a same-net via's rim to its
    centre: one end within 1 nm of the centre, the other within 1 um of the
    rim."""
    from kicad_parser import parse_kicad_pcb
    p = parse_kicad_pcb(path)
    out = []
    for s in p.segments:
        for v in p.vias:
            if v.net_id != s.net_id:
                continue
            d = sorted(math.hypot(x - v.x, y - v.y)
                       for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y)))
            if d[0] < 1e-6 and abs(d[1] - v.size / 2) < 1e-3:
                out.append((p.nets[s.net_id].name, s))
    return out


def length_rule_board(src, dst, rule_net, measured_net):
    """Copy `src`, take the recorded pair tolerances off the copy's project
    (so no skew rule grades its nets) and write a .kicad_dru length rule that
    binds `rule_net`, with a window 0.1 mm either side of `measured_net`'s
    length as check_drc measures it. Returns that length."""
    from kicad_parser import parse_kicad_pcb
    from net_queries import net_match_length
    py(tool('copy_board.py'), src, dst)
    pro = dst[:-len('.kicad_pcb')] + '.kicad_pro'
    with open(evidence(pro, 'project copy'), encoding='utf-8') as fh:
        doc = json.load(fh)
    doc['kicad_routing_tools'].pop('intra_pair_tolerance')
    with open(pro, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=2)
    p = parse_kicad_pcb(dst)
    nid = next(k for k, n in p.nets.items() if n.name == measured_net)
    length = net_match_length(p, nid, 'total')
    with open(dst[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n(rule "stub len" (condition "A.NetName == \'%s\'")'
                 ' (constraint length (min %.4fmm) (max %.4fmm)))\n'
                 % (rule_net, length - 0.1, length + 0.1))
    return length


def add_detour(src, dst):
    """Copy `src`, add a redundant two-segment detour beside the 1 mm-ish
    segment of the pair-D0 N member, and a .kicad_dru skew rule over the two
    N members that the board meets only while the detour is there. Returns
    the rule's max."""
    from kicad_parser import parse_kicad_pcb
    from net_queries import net_match_length
    py(tool('copy_board.py'), src, dst)
    p = parse_kicad_pcb(dst)
    nid = {n.name: i for i, n in p.nets.items()}
    segs = [s for s in p.segments if s.net_id == nid['CSI_D0_N']]
    s = min(segs, key=lambda s: abs(math.hypot(s.end_x - s.start_x,
                                               s.end_y - s.start_y) - 1.0))
    L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
    cx = (s.start_x + s.end_x) / 2 - (s.end_y - s.start_y) / L * 0.3
    cy = (s.start_y + s.end_y) / 2 + (s.end_x - s.start_x) / L * 0.3

    def seg(x1, y1, x2, y2):
        return (f'\t(segment\n\t\t(start {x1:.6f} {y1:.6f})\n\t\t(end {x2:.6f} '
                f'{y2:.6f})\n\t\t(width {s.width})\n\t\t(layer "{s.layer}")\n'
                f'\t\t(net "CSI_D0_N")\n\t)\n')
    with open(dst, encoding='utf-8') as fh:
        t = fh.read()
    i = t.rstrip().rfind(')')
    t = t[:i] + seg(s.start_x, s.start_y, cx, cy) + seg(cx, cy, s.end_x, s.end_y) + t[i:]
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(t)
    q = parse_kicad_pcb(dst)
    a = net_match_length(q, nid['CSI_CLK_N'], 'total')
    b = net_match_length(q, nid['CSI_D0_N'], 'total')
    b0 = net_match_length(p, nid['CSI_D0_N'], 'total')
    mx = round(((a - b) + (a - b0)) / 2, 3)
    with open(dst[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n(rule "n skew" (constraint skew (max %smm))\n'
                 '  (condition "A.NetName == \'CSI_CLK_N\' || '
                 'A.NetName == \'CSI_D0_N\'"))\n' % mx)
    return mx


def main():
    tmp = tempfile.mkdtemp(prefix='t1798_')
    try:
        a, f, b, c, c2, e, ec = (os.path.join(tmp, f'{t}.kicad_pcb')
                                 for t in ('a', 'f', 'b', 'c', 'c2', 'e', 'ec'))
        py(tool('copy_board.py'), BOARD, a)
        py(tool('qfn_fanout.py'), evidence(a, 'board copy'), '-o', f,
           '--component', 'U1', '--nets', 'CSI_*', '--escape-method', 'underpad',
           '--layer', 'In1.Cu', '--width', '0.127', '--via-size', '0.5',
           '--via-drill', '0.2', '--allow-via-in-pad')
        py(tool('route_diff.py'), evidence(f, 'fanout board'), b,
           '--nets', 'CSI_D0*', 'CSI_CLK*', '--layers', 'F.Cu', 'In1.Cu',
           '--track-width', '0.127', '--diff-pair-gap', '0.2', '--via-size', '0.6',
           '--diff-pair-intra-match')

        print("1. the matched board")
        out = graded(tool('check_drc.py'), evidence(b, 'route_diff board'), '-c', '0.1')
        if '2 group(s)/pair(s) graded, 0 net(s) out of range' not in out:
            print("  [BROKEN TEST] the pairs are not graded within their recorded "
                  "tolerance on the route_diff board:\n" + out[-800:])
            return 1
        stubs = centre_stubs(b)
        if not stubs:
            print("  [BROKEN TEST] no rim-to-centre via stub on the route_diff "
                  "board: nothing here exercises #1798")
            return 1
        name, stub = stubs[0]
        print(f"  stub: {name} {stub.layer} ({stub.start_x},{stub.start_y})->"
              f"({stub.end_x},{stub.end_y})")

        print("2. check_weird keeps the stub")
        w = graded(tool('check_weird.py'), b)
        tag = f"({stub.start_x:.3f}, {stub.start_y:.3f})-({stub.end_x:.3f}, {stub.end_y:.3f})"
        check("the stub is not listed removable",
              not re.search(r'removable-segment.*' + re.escape(tag), w)
              and tag not in w, tag)
        # The rule is scoped to the nets a skew rule grades: on any other net
        # the stub is copper no grade reads, and keeping it everywhere moved
        # later steps' routing (a corpus A/B, 13 boards worse / 5 better).
        from kicad_parser import parse_kicad_pcb
        from cleanup_pipeline import _skew_bound_net_ids
        from pcb_modification import StrictRemovalModel, strict_removable_segments
        pb = parse_kicad_pcb(b)
        nid = next(k for k, n in pb.nets.items() if n.name == name)
        check("the board's recorded tolerance makes the stub's net skew-graded",
              nid in _skew_bound_net_ids(pb))
        segs = [s for s in pb.segments if s.net_id == nid]
        at = next(i for i, s in enumerate(segs)
                  if {(round(s.start_x, 6), round(s.start_y, 6)),
                      (round(s.end_x, 6), round(s.end_y, 6))}
                  == {(round(stub.start_x, 6), round(stub.start_y, 6)),
                      (round(stub.end_x, 6), round(stub.end_y, 6))})
        verdict = {}
        for keep in (True, False):
            m = StrictRemovalModel(nid, segs, [v for v in pb.vias if v.net_id == nid],
                                   pb.pads_by_net.get(nid, []),
                                   pb.board_info.copper_layers,
                                   keep_centre_stubs=keep)
            verdict[keep] = m.valid and at in strict_removable_segments(m)
        check("the model keeps the stub on a skew-graded net and removes it on "
              "any other", verdict == {True: False, False: True}, str(verdict))

        print("3. the all-nets route.py step")
        before = net_len_mm(b, name)
        r = py(tool('route.py'), b, c, '--nets', '*')
        after = net_len_mm(evidence(c, 'routed board'), name)
        check(f"{name}'s KiCad length is unchanged", abs(after - before) < 1e-6,
              f"{before:.4f} -> {after:.4f} mm")
        out = graded(tool('check_drc.py'), c, '-c', '0.1')
        check("check_drc grades the pairs within their tolerance",
              '2 group(s)/pair(s) graded, 0 net(s) out of range' in out,
              (re.search(r'skew: [^;\n]*', out) or [''])[0])

        print("4. a late pass that moves a group says so (CLI)")
        mx = add_detour(b, e)
        js = os.path.join(tmp, 'ec.json')
        r = py(tool('route.py'), evidence(e, 'detour board'), ec, '--nets', '*',
               '--json-out', js)
        log = r.stdout + r.stderr
        if 'Strict collapse (#1063, end of run)' not in log:
            print("  [BROKEN TEST] the end-of-run collapse removed nothing: the "
                  "detour this row needs removed is not\n" + log[-800:])
            return 1
        with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
            doc = json.load(fh)
        rows = [x for x in doc.get('skew_moved_by_cleanup') or []
                if set(x['nets']) == {'CSI_CLK_N', 'CSI_D0_N'}]
        check("--json-out names the group", len(rows) == 1 and rows[0]['was_within']
              and rows[0]['max_mm'] == mx and rows[0]['skew_after_mm'] > mx,
              str(doc.get('skew_moved_by_cleanup')))
        check("the WARNING names it", re.search(
            r"WARNING: the end-of-run cleanup \(#1798\) moved CSI_CLK_N / CSI_D0_N "
            r"out of its skew rule 'n skew'", log) is not None)

        print("5. ...and the GUI leg")
        from kicad_parser import parse_kicad_pcb
        from route import batch_route
        pcb = parse_kicad_pcb(e)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            _ok, _fail, _t, rd = batch_route(
                e, '', [n.name for n in pcb.nets.values() if n.name],
                pcb_data=pcb, return_results=True)
        rows = [x for x in rd.get('skew_moved_by_cleanup') or []
                if set(x['nets']) == {'CSI_CLK_N', 'CSI_D0_N'}]
        check("results_data names the group", len(rows) == 1,
              str(rd.get('skew_moved_by_cleanup')) + buf.getvalue()[-400:])

        print("6. control: --keep-input-copper")
        r = py(tool('route.py'), b, c2, '--nets', '*', '--keep-input-copper')
        log = r.stdout + r.stderr
        check("neither late-pass line is printed",
              'Strict collapse' not in log and 'Dangling vias' not in log)

        print("7. a net a .kicad_dru length rule binds keeps the stub")
        lb, lc, ob, oc = (os.path.join(tmp, f'{t}.kicad_pcb')
                          for t in ('lb', 'lc', 'ob', 'oc'))
        other = next(n for n in ('CSI_D0_P', 'CSI_CLK_P') if n != name)
        want = length_rule_board(b, lb, name, name)
        length_rule_board(b, ob, other, other)
        for path in (lb, ob):
            if _skew_bound_net_ids(parse_kicad_pcb(path)):
                print("  [BROKEN TEST] a skew rule still grades a net of "
                      f"{os.path.basename(path)}: the record was not removed")
                return 1
        out = graded(tool('check_drc.py'), evidence(lb, 'length-rule board'),
                     '-c', '0.1')
        if '1 net(s) graded, 0 out of range' not in out:
            print("  [BROKEN TEST] the length rule does not grade its net "
                  "inside the window on the input:\n" + out[-800:])
            return 1
        w = graded(tool('check_weird.py'), lb)
        check("check_weird does not list the stub removable", tag not in w, tag)
        r = py(tool('route.py'), lb, lc, '--nets', '*')
        after = net_len_mm(evidence(lc, 'routed board'), name)
        check(f"{name}'s KiCad length is unchanged by the all-nets step",
              abs(after - before) < 1e-6, f"{before:.4f} -> {after:.4f} mm")
        out = graded(tool('check_drc.py'), lc, '-c', '0.1')
        check("check_drc grades the net inside its length window",
              '1 net(s) graded, 0 out of range' in out,
              (re.search(r'Length rules[^\n]*', out) or [''])[0]
              + f'  (window {want - 0.1:.4f}..{want + 0.1:.4f})')
        # The rule on another net: this stub is on a net no grade reads.
        w = graded(tool('check_weird.py'), evidence(ob, 'control board'))
        check(f"with the rule on {other}, check_weird lists the stub removable",
              tag in w, tag)
        r = py(tool('route.py'), ob, oc, '--nets', '*')
        after = net_len_mm(evidence(oc, 'control routed board'), name)
        check("...and the all-nets step's collapse removes it",
              'Strict collapse (#1063, end of run)' in r.stdout + r.stderr
              and after < before - 0.1, f"{before:.4f} -> {after:.4f} mm")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
