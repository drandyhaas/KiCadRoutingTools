#!/usr/bin/env python3
"""#1342: qfn_fanout skips a net with one pad board-wide, as bga_fanout does.

A net whose only pad is on the package (a spare pin the schematic still names)
has nothing to connect to, so its escape is copper that can only block
routing. bga_fanout has skipped such nets since #122 and reported them as
`skipped_nc`; qfn_fanout filtered only net 0 and `unconnected-*` names, so on
a CM5 module (gp14 M28) 33 of 70 new vias were on single-pad nets
(/CM5/DPHY1_P.*, HDMI1, USB3, SD...), and on its second connector 6 of 7
"failed" nets were such pins. The skip is now in `generate_qfn_fanout` (the
engine both fronts call) with the same predicate (`single_pad_net_ids`), and
the CLI's JSON_SUMMARY carries `skipped_nc`.

Checks:
  1. ulx3s U8 (a real board; its Net-(U8-Pad12) and Net-(U8-Pad25) are
     named single-pad nets): the CLI stub fan reports skipped_nc 2, says so,
     lays no copper on either net, does not count them as failed, and still
     escapes the board's other nets.
  2. The engine (the GUI's call) on the qfn_diffpair_escape fixture, whose
     DP1_P/DP1_N exist only on U1: nothing is laid and nothing is reported
     failed. The same run on a copy where each net has a partner pad
     escapes both -- so the skip is exactly the one-pad condition.

    python3 tests/test_1342_qfn_skips_single_pad_nets.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from qfn_fanout import generate_qfn_fanout                     # noqa: E402
from bga_fanout import single_pad_net_ids                      # noqa: E402
from run_utils import check as run_check, evidence             # noqa: E402
from synth import with_partner_pads                            # noqa: E402

ULX3S = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
NC_NETS = {'Net-(U8-Pad12)', 'Net-(U8-Pad25)'}
FIXTURE = os.path.join(ROOT, 'kicad_files', 'qfn_diffpair_escape.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def engine(path):
    pcb = parse(path)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        tracks, vias, failed = generate_qfn_fanout(
            pcb.footprints['U1'], pcb, net_filter=['DP1*'], layer='F.Cu',
            track_width=0.1, clearance=0.15, grid_step=0.05,
            escape_method='underpad', via_size=0.45, via_drill=0.2,
            allow_via_in_pad=True)
    names = {pcb.nets[i].name for i in
             {t['net_id'] for t in tracks} | {v['net_id'] for v in vias}}
    return names, failed, out.getvalue()


def main():
    tmp = tempfile.mkdtemp(prefix='t1342_')
    try:
        # 1. A real board, through the CLI.
        src = parse(evidence(ULX3S, 'ulx3s board'))
        fp = src.footprints['U8']
        nc = {src.nets[i].name for i in single_pad_net_ids(fp, src)}
        check('precondition: U8 carries the two named single-pad nets',
              NC_NETS <= nc and not any(n.lower().startswith('unconnected-')
                                        for n in NC_NETS), str(sorted(nc)))
        out = os.path.join(tmp, 'u8.kicad_pcb')
        r = run_check([sys.executable, '-X', 'utf8', 'py_router/qfn_fanout.py',
                       ULX3S, '--component', 'U8', '--nets', '*',
                       '--width', '0.15', '--clearance', '0.15', '-o', out],
                      accept=True)
        evidence(out, 'qfn_fanout output')
        line = next((l for l in r.stdout.splitlines() if 'JSON_SUMMARY' in l), '')
        js = json.loads(line.split('JSON_SUMMARY:', 1)[1]) if line else {}
        check('the summary reports skipped_nc', js.get('skipped_nc') == len(nc),
              f"skipped_nc={js.get('skipped_nc')} vs {len(nc)}")
        check('the run says it skipped them',
              f'Skipping {len(nc)} single-pad/NC net(s)' in r.stdout)
        check('neither is counted failed',
              not NC_NETS & set(js.get('unescaped_nets') or []),
              str(js.get('unescaped_nets')))
        check('the board\'s other nets still escape',
              (js.get('escaped') or 0) > 0, f"escaped={js.get('escaped')}")
        after = parse(out)
        laid = {after.nets[s.net_id].name for s in after.segments
                if s.net_id in after.nets} | \
               {after.nets[v.net_id].name for v in after.vias
                if v.net_id in after.nets}
        before = {src.nets[s.net_id].name for s in src.segments
                  if s.net_id in src.nets}
        check('no copper is laid on a single-pad net',
              not (NC_NETS & laid) - before, str(sorted(NC_NETS & laid)))

        # 2. The engine, on the fixture as it is and with partners.
        names, failed, log = engine(evidence(FIXTURE, 'qfn fixture'))
        check('fixture as tracked: DP1 nets are single-pad, so nothing is laid',
              not names and not failed, f'laid={sorted(names)} failed={failed}')
        check('fixture as tracked: the engine prints the skip',
              'single-pad/NC net(s)' in log)
        partnered = os.path.join(tmp, 'qfn_diffpair_escape.kicad_pcb')
        added = with_partner_pads(FIXTURE, partnered, (2.0, 2.0))
        names, failed, _log = engine(partnered)
        check('with a partner pad each, both DP1 nets escape',
              {'DP1_P', 'DP1_N'} <= set(added)
              and names == {'DP1_P', 'DP1_N'} and not failed,
              f'laid={sorted(names)} failed={failed}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
