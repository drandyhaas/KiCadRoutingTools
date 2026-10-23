#!/usr/bin/env python3
"""The verified-pour re-audit: a pour KiCad verified complete must not ship
cut by the final reconciliation.

route.py's in-run plane finalize audits every zone net with the KiCad oracle
and then runs the final reconciliation, which lays and rips copper WITH RIP
AUTHORITY. Nothing looked again, and pours KiCad had just called complete
shipped cut on five boards (the 24f811a1 wave, replayed with a snapshot
after every pass): rescued keyboard rows and a rerouted key row cut GND's
pour; a rescued RXD0 fenced a capacitor pad off on a 2.4 mm2 B.Cu island; a
/S11 rip-reroute split the B.Cu GND fill where only KiCad's refill saw it (a
0.09 mm neck between two vias, under the zone's 0.1 mm minimum width, which
the raster model passes); an SD data line's reconciliation route cut the
In1.Cu GND plane.

Checks:
  B. verified_pour_compare grades the pours by KiCad on both boards: a
     raster-joined island KiCad splits is lost, a raster-split pour KiCad
     calls whole is not, a pour the oracle left open at the audit is graded
     open on both sides, one KiCad graded on one side only is left out
  C. _verified_pour_reaudit on the CLI file, KiCad oracle stubbed: a pour
     the weld cannot mend REFUSES a reconciliation that connected one net
     (the file is the verified board again), KEEPS one that connected two
     (and hands the links to the re-grade), keeps and says so when the
     reconciliation moved a part, and leaves a whole board alone
  D. the same on the GUI change-set: the refusal puts back results_data's
     lists and result dicts, pcb_data's copper, and drops the weld it merged
  E. batch_route takes the snapshot before the final reconciliation and runs
     the re-audit after it, before the #984 bridge and the end-of-run
     collapse, and hands the re-grade the links of a pour it kept cut

    python3 tests/test_pour_reaudit.py
"""
import ast
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence                                   # noqa: E402
from kicad_parser import parse_kicad_pcb, Segment               # noqa: E402
from kicad_writer import generate_segment_sexpr                 # noqa: E402
from improvement_gate import (verified_pour_compare,           # noqa: E402
                              copper_signature)
import kicad_oracle                                              # noqa: E402
import route                                                     # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files',
                     'lvds_converter_dualclk_gnd.kicad_pcb')
ZONE = '/GND'
failures = []


def check(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}"
          + (f" -- {detail}" if detail and not cond else ''))
    if not cond:
        failures.append(label)


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


# ------------------------------------------------------- B. KiCad grades pours
def t_compare():
    print('B. verified_pour_compare grades the pours by KiCad, not the raster')
    nm = {1: 'GND', 2: '/S11', 3: '/S13', 4: 'VCC'}.get
    # The /S11 case: the raster joins GND's islands; KiCad reports one link.
    before = {1: (True, 0), 2: (False, 1), 3: (True, 0)}
    after = {1: (True, 0), 2: (True, 0), 3: (False, 1)}
    cmp, v = verified_pour_compare(before, after, nm, {1: (True, 0)},
                                {1: (False, 1)}, {1})
    check('a pour KiCad splits is lost even where the raster joins it',
          cmp['lost'] == ['/S13', 'GND'] and v == 'reject', str(cmp))
    # The raster splits a pour KiCad calls whole: not a loss.
    cmp, v = verified_pour_compare({1: (False, 2), 2: (False, 1)},
                                {1: (False, 3), 2: (True, 0)}, nm,
                                {1: (True, 0)}, {1: (True, 0)}, {1})
    check('a pour KiCad calls whole is not lost (raster disagreement ignored)',
          cmp['lost'] == [] and cmp['gained'] == ['/S11'] and v == 'accept',
          str(cmp))
    # A pour KiCad left open at the audit (custody) that the raster joined
    # on the verified board: KiCad's word on both sides.
    cmp, v = verified_pour_compare({4: (True, 0), 2: (False, 1)},
                                {4: (False, 2), 2: (True, 0)}, nm,
                                {4: (False, 1)}, {4: (False, 2)}, {4})
    check('a custody pour open on both boards by KiCad is not "lost"',
          cmp['lost'] == [] and cmp['worsened'] == [('VCC', 1, 2)], str(cmp))
    cmp, v = verified_pour_compare({4: (False, 1), 2: (False, 1)},
                                {4: (True, 0), 2: (True, 0)}, nm,
                                {4: (False, 1)}, {}, {4})
    check('a pour KiCad graded on one side only is left out',
          cmp['nets_compared'] == 1 and cmp['gained'] == ['/S11'], str(cmp))


# ----------------------------------------------------------- board fixtures
def pads_of(pcb, name):
    nid = next(i for i, n in pcb.nets.items() if n.name == name)
    return nid, pcb.pads_by_net[nid]


def with_links(path, nets):
    """Append a straight F.Cu track joining the first two pads of each net
    in `nets` -- the copper a reconciliation that connected them leaves."""
    pcb = quiet(parse_kicad_pcb, path)
    sx = []
    for name in nets:
        nid, pads = pads_of(pcb, name)
        a, b = pads[0], pads[1]
        sx.append(generate_segment_sexpr((a.global_x, a.global_y),
                                         (b.global_x, b.global_y), 0.2,
                                         'F.Cu', nid, name))
    with open(path, encoding='utf-8') as f:
        c = f.read()
    lp = c.rfind(')')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(c[:lp] + '\n'.join(sx) + '\n' + c[lp:])


def move_c2(path):
    with open(path, encoding='utf-8') as f:
        c = f.read()
    assert c.count('(at 114.75 63)') == 1
    with open(path, 'w', encoding='utf-8') as f:
        f.write(c.replace('(at 114.75 63)', '(at 115.75 63)'))


class Oracle:
    """Stands in for kicad_oracle.oracle_reconnect: records its calls and
    answers with `links` still missing on the zone net."""

    def __init__(self, links):
        self.links, self.calls = links, []

    def __call__(self, board, nets, config, **kw):
        self.calls.append((board, list(nets), kw))
        rl = [[ZONE, [100.0, 70.0, 'B.Cu', 'zone'],
               [101.0, 70.0, 'B.Cu', 'zone']]] * self.links
        return {'available': True, 'reason': 'ok', 'rounds': 1,
                'links_routed': 0, 'links_failed': self.links,
                'remaining': self.links, 'remaining_links': rl}


def run_finish(snap, pcb, out, rd, oracle, **over):
    real = kicad_oracle.oracle_reconnect
    kicad_oracle.oracle_reconnect = oracle
    nets, links = set(), []
    kw = dict(stage=lambda: (None, []), drop=lambda t: None,
              merge=lambda o, r: None, write_model=None,
              oracle_nets=nets, open_links=links)
    kw.update(over)
    try:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rec = route._verified_pour_reaudit(
                snap, [ZONE], {}, pcb, out, rd, (None, object(), {}, {}),
                type('C', (), {'hole_to_hole_clearance': 0.25})(), out, None,
                **kw)
    finally:
        kicad_oracle.oracle_reconnect = real
    return rec, buf.getvalue(), links


def sig(path):
    p = quiet(parse_kicad_pcb, path)
    return copper_signature(p.segments, p.vias, lambda i: p.nets[i].name)


# --------------------------------------------------------------- C. CLI file
def t_cli():
    print('C. the CLI re-audit refuses, keeps or leaves the written board')
    evidence(BOARD, 'fixture board')
    d = tempfile.mkdtemp(prefix='pour_reaudit_')
    try:
        out = os.path.join(d, 'out.kicad_pcb')

        def case(nets, links, moved=False):
            shutil.copy(BOARD, out)
            pcb = quiet(parse_kicad_pcb, out)
            snap = route._verified_pour_snapshot(pcb, out, None)
            with_links(out, nets)               # the reconciliation's copper
            if moved:
                move_c2(out)
            after = sig(out)
            oracle = Oracle(links)
            rec, log, kept = run_finish(snap, pcb, out, None, oracle)
            os.unlink(snap['file'])
            return rec, log, kept, after, [c[:2] for c in oracle.calls]

        rec, log, kept, after, calls = case(['/OUT_A'], 1)
        check('one re-audit, of the written file, over the verified pours',
              calls == [(out, [ZONE])], str(calls))
        check('one net connected for a pour the weld cannot mend: REFUSED',
              rec.get('verdict') == 'refused' and 'REFUSED' in log, str(rec))
        check('... and the written board is the verified one again',
              sig(out) == sig(BOARD) and sig(out) != after)
        check('... and no link is handed to the re-grade (the pour is whole)',
              kept == [])

        rec, log, kept, after, _ = case(['/OUT_A', '/OUT_B'], 1)
        check('two nets connected for one pour: KEPT, and said',
              rec.get('verdict') == 'kept' and 'KEPT' in log
              and rec.get('lost') == [ZONE], str(rec))
        check('... the reconciliation\'s copper ships',
              sig(out) == after)
        check('... and the cut pour\'s link goes to the final re-grade',
              len(kept) == 1 and kept[0][0] == ZONE, str(kept))

        rec, log, kept, after, _ = case(['/OUT_A'], 1, moved=True)
        check('a reconciliation that moved a part is kept, and says why',
              rec.get('verdict') == 'kept' and 'moved a part' in log
              and sig(out) == after, str(rec))

        rec, log, kept, after, _ = case(['/OUT_A'], 0)
        check('a board whose pours the re-audit finds whole is left alone',
              rec.get('verdict') == 'whole' and sig(out) == after, str(rec))
    finally:
        shutil.rmtree(d, ignore_errors=True)


# --------------------------------------------------------- D. GUI change-set
def t_gui():
    print('D. the GUI re-audit restores the change-set and pcb_data')
    pcb = quiet(parse_kicad_pcb, BOARD)
    base_segs = list(pcb.segments)
    nid, pads = pads_of(pcb, '/OUT_A')
    first = {'net_name': '/X', 'new_segments': [], 'new_vias': []}
    rd = {'results': [first], 'segments_to_remove': [], 'vias_to_remove': [],
          'all_swap_vias': [], 'all_swap_segments': [], 'pad_swaps': []}
    results_alias, swap_alias = rd['results'], rd['all_swap_vias']
    snap = route._verified_pour_snapshot(pcb, '', rd)
    # the reconciliation: a new result, a rebound list, a removal, pcb copper
    s = Segment(start_x=pads[0].global_x, start_y=pads[0].global_y,
                end_x=pads[1].global_x, end_y=pads[1].global_y, width=0.2,
                layer='F.Cu', net_id=nid)
    rd['results'].append({'net_name': '/OUT_A', 'new_segments': [s],
                          'new_vias': []})
    first['new_segments'] = [s]
    rd['segments_to_remove'].append(base_segs[0])
    pcb.segments = pcb.segments + [s]
    staged = []

    def stage():
        fd, p = tempfile.mkstemp(suffix='.kicad_pcb')
        os.close(fd)
        shutil.copy(BOARD, p)
        staged.append(p)
        return p, [p]

    def drop(tmps):
        for p in tmps:
            os.unlink(p)

    weld = Segment(start_x=1, start_y=1, end_x=2, end_y=1, width=0.2,
                   layer='B.Cu', net_id=nid)

    def merge(orc, r):
        r['results'].append({'net_name': '(pour re-audit)',
                             'new_segments': [weld], 'new_vias': []})

    def write_model(r):
        drop_ids = {id(x) for x in r.get('segments_to_remove') or []}
        segs, vias = {}, {}
        for x in base_segs:
            if id(x) not in drop_ids:
                segs.setdefault(x.net_id, []).append(x)
        for x in pcb.vias:
            vias.setdefault(x.net_id, []).append(x)
        for res in r.get('results') or []:
            for x in res.get('new_segments') or []:
                segs.setdefault(x.net_id, []).append(x)
        return segs, vias

    oracle = Oracle(1)
    rec, log, kept = run_finish(snap, pcb, '', rd, oracle, stage=stage,
                                drop=drop, merge=merge,
                                write_model=write_model)
    check('the GUI re-audit ran on a staged board', bool(staged)
          and oracle.calls and oracle.calls[0][0] == staged[0]
          and not os.path.exists(staged[0]))
    check('one net connected for a pour the weld cannot mend: REFUSED',
          rec.get('verdict') == 'refused', str(rec))
    check('results_data is the snapshot again (in place)',
          rd['results'] is results_alias and len(rd['results']) == 1
          and first['new_segments'] == [] and rd['segments_to_remove'] == []
          and rd['all_swap_vias'] is swap_alias)
    check('pcb_data\'s copper is the snapshot again',
          [id(x) for x in pcb.segments] == [id(x) for x in base_segs])


# ------------------------------------------------------------- E. the wiring
def t_wiring():
    print('E. batch_route snapshots before the reconciliation, re-audits '
          'after')
    with open(os.path.join(ROOT, 'py_router', 'route.py'),
              encoding='utf-8') as f:
        src = f.read()
    fn = next(n for n in ast.parse(src).body
              if isinstance(n, ast.FunctionDef) and n.name == 'batch_route')
    body = ast.get_source_segment(src, fn)
    pos = {k: body.find(k) for k in (
        '_verified_pour_snapshot(', "Final reconciliation: retrying",
        '_vp_ran = True', 'def _gui_write_model', '_verified_pour_reaudit(',
        '_late_soft_joint_bridge984(', '_late_strict_collapse1063(',
        '_final_regrade(')}
    check('every anchor is in batch_route', all(v >= 0 for v in pos.values()),
          str(pos))
    check('the snapshot precedes the reconciliation, which marks that it ran',
          pos['_verified_pour_snapshot(']
          < pos["Final reconciliation: retrying"]
          < pos['_vp_ran = True'])
    check('the re-audit follows the reconciliation and _gui_write_model, and '
          'precedes the #984 bridge and the strict collapse',
          pos['_vp_ran = True'] < pos['def _gui_write_model']
          < pos['_verified_pour_reaudit('] < pos['_late_soft_joint_bridge984(']
          < pos['_late_strict_collapse1063('])
    check('the final re-grade is handed the links of a pour kept cut',
          '+ list(_vp_open_links)' in body[pos['_final_regrade(']:])
    check('... and, after a refusal, every custody link again',
          '_vp_open_links.extend(_custody_links9'
          in body[pos['_verified_pour_reaudit(']:pos['_final_regrade(']])


def main():
    t_compare()
    t_cli()
    t_gui()
    t_wiring()
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nVerified-pour re-audit: all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
