#!/usr/bin/env python3
"""check_weird and route.py's cleanup agree on what is removable (#1063).

check_complete blocks DONE on any check_weird finding, and check_weird counted
in-pad / in-via wiggles as removable-segment while collapse_strict_redundant
kept them "by choice" (#217). So no plain route.py output could reach DONE:
esp_prog shipped 29 removable segments, every one with both ends buried in
same-net pad or via copper, plus a redundant loop on /D_N (close_soft_joints'
via->pad bridge laid a direct F.Cu path beside a B.Cu detour through two vias).

The contract, pinned here:

  1. a wiggle buried at both ends (in a pad, in a via) is removed by the
     cleanup, the net stays connected, and check_weird then reports nothing;
  2. a redundant loop through two vias loses its VIA branch (the vias go too,
     and none is left dangling) -- and when the vias are not the run's own, the
     pass removes the other branch instead of stranding them;
  3. the invariant, on real routed copper: after the delete-only passes,
     check_weird calls nothing removable and no finding category grew;
  4. a plain route.py of esp_prog reaches check_complete's weird_copper clean,
     and it is the END-OF-RUN collapse that gets it there: route.py's in-run
     cleanup no longer collapses (it steered the plane finalize's rip/reroute,
     cparti_fpga 6 open nets -> 15), and the one pass after the finalize and
     the reconciliation removes copper;
  5. that end-of-run pass on the GUI front: a removed wiggle this run laid
     leaves its result, a removed INPUT wiggle joins segments_to_remove, and
     --keep-input-copper keeps the input one.

    python3 tests/test_1063_weird_inpad_wiggle_contract.py
"""
import collections
import json
import os
import subprocess
import sys
import tempfile

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import make_pad, make_seg, make_via, make_pcb, make_net
from kicad_parser import parse_kicad_pcb
from check_connected import check_net_connectivity
from check_weird import check_weird, blocking_findings
from pcb_modification import collapse_strict_redundant, prune_redundant_cycles
from run_utils import tool, tool_env, evidence

FAILS = []
NET = 7


def check(name, cond, detail=""):
    if not cond:
        FAILS.append(name)
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))


def _board(segs, vias, pads):
    by_net = {}
    for p in pads:
        by_net.setdefault(p.net_id, []).append(p)
    return make_pcb(nets={NET: make_net(NET, '/N7')}, segments=segs, vias=vias,
                    pads_by_net=by_net)


def _cats(pcb, **kw):
    # what the exit code (and so check_complete) counts: #1235's advisory
    # off-centre-via-end is reported apart and blocks nothing
    f = blocking_findings(check_weird(pcb, **kw)[0])
    return collections.Counter(x['category'] for x in f), f


def _connected(pcb):
    r = check_net_connectivity(NET, pcb.segments, pcb.vias,
                               pcb.pads_by_net[NET], [])
    return r['connected'] and not r['disconnected_pads']


def wiggles():
    print("1. buried-at-both-ends wiggles")
    # (a) in a pad: a 2x2 pad with a 1 mm wiggle inside it, beside the trunk.
    pads = [make_pad(NET, 0, 0, ref='U1', size_x=2.0, size_y=2.0),
            make_pad(NET, 10, 0, ref='U2', size_x=0.6, size_y=0.6)]
    trunk = make_seg(0, 0, 10, 0, net_id=NET)
    wiggle = make_seg(-0.5, 0.3, 0.5, 0.3, net_id=NET)
    pcb = _board([trunk, wiggle], [], pads)
    c, f = _cats(pcb)
    flagged = [x for x in f if x['category'] == 'removable-segment']
    check("check_weird flags the in-pad wiggle, and only it",
          len(flagged) == 1 and abs(flagged[0]['y'] - 0.3) < 1e-9, str(dict(c)))
    n, _ = collapse_strict_redundant([], pcb, None)
    check("the cleanup removes it and keeps the trunk",
          n == 1 and wiggle not in pcb.segments and trunk in pcb.segments)
    check("the net stays connected", _connected(pcb))
    c, _ = _cats(pcb, tolerance=0)
    check("check_weird then reports nothing", not c, str(dict(c)))

    # (b) in a via: a real layer change (F.Cu pad -> via -> B.Cu pad) with a
    # short F.Cu tail from the via centre to a point inside its barrel.
    pads = [make_pad(NET, 0, 0, ref='U1', size_x=0.6, size_y=0.6),
            make_pad(NET, 10, 0, ref='U2', size_x=0.6, size_y=0.6,
                     layers=('B.Cu',))]
    via = make_via(5, 0, net_id=NET, size=0.6)
    top = make_seg(0, 0, 5, 0, net_id=NET)
    bot = make_seg(5, 0, 10, 0, net_id=NET, layer='B.Cu')
    tail = make_seg(5, 0, 5.15, 0.15, net_id=NET)
    pcb = _board([top, bot, tail], [via], pads)
    c, _ = _cats(pcb)
    check("check_weird flags the in-via tail",
          c.get('removable-segment') == 1, str(dict(c)))
    n, _ = collapse_strict_redundant([], pcb, None)
    check("the cleanup removes it; the via still joins both layers",
          n == 1 and tail not in pcb.segments and via in pcb.vias)
    check("the net stays connected", _connected(pcb))
    c, _ = _cats(pcb, tolerance=0)
    check("check_weird then reports nothing", not c, str(dict(c)))


def _via_loop():
    """U1 (0,0) and U2 (10,0), F.Cu. Between two vias at x=3 and x=7 the net
    runs twice: a 4 mm B.Cu straight and a 10 mm F.Cu detour around it. The
    via branch is the SHORTER one, so 'longest first' alone would keep it."""
    pads = [make_pad(NET, 0, 0, ref='U1', size_x=0.6, size_y=0.6),
            make_pad(NET, 10, 0, ref='U2', size_x=0.6, size_y=0.6)]
    v1, v2 = make_via(3, 0, net_id=NET), make_via(7, 0, net_id=NET)
    lead_in = make_seg(0, 0, 3, 0, net_id=NET)
    lead_out = make_seg(7, 0, 10, 0, net_id=NET)
    around = [make_seg(3, 0, 3, -3, net_id=NET), make_seg(3, -3, 7, -3, net_id=NET),
              make_seg(7, -3, 7, 0, net_id=NET)]
    under = make_seg(3, 0, 7, 0, net_id=NET, layer='B.Cu')
    pcb = _board([lead_in, lead_out, under] + around, [v1, v2], pads)
    return pcb, (v1, v2), under, around


def via_loop():
    print("2. a redundant loop through two vias")
    pcb, vias, under, around = _via_loop()
    # The run's own copper: the vias (and everything) are in the write-list.
    results = [{'new_segments': list(pcb.segments), 'new_vias': list(pcb.vias)}]
    st = {}
    n, strip = collapse_strict_redundant(results, pcb, None, stats=st)
    check("the via branch goes: the B.Cu run and both vias",
          under not in pcb.segments and not pcb.vias and st.get('vias') == 2,
          str(st))
    check("the direct F.Cu branch stays",
          all(s in pcb.segments for s in around))
    check("the vias leave the write-list too",
          results[0]['new_vias'] == [] and under not in results[0]['new_segments']
          and strip == [])
    check("the net stays connected", _connected(pcb))
    c, _ = _cats(pcb, tolerance=0)
    check("check_weird then reports nothing (no dangling via)", not c, str(dict(c)))

    # Control: the same board read from a file -- every via is INPUT copper,
    # which this pass may not drop. It must not strand them: it takes the
    # other branch and the vias keep joining both layers.
    pcb, vias, under, around = _via_loop()
    n, strip = collapse_strict_redundant([], pcb, None)
    check("with input vias the F.Cu detour goes instead",
          under in pcb.segments and all(s not in pcb.segments for s in around)
          and all(v in pcb.vias for v in vias), f"n={n}")
    check("the net stays connected", _connected(pcb))
    c, _ = _cats(pcb, tolerance=0)
    check("check_weird then reports nothing (no dangling via)", not c, str(dict(c)))


def _net_state(pcb):
    out = {}
    for nid, pads in pcb.pads_by_net.items():
        if nid == 0 or len(pads) < 2:
            continue
        r = check_net_connectivity(
            nid, [s for s in pcb.segments if s.net_id == nid],
            [v for v in pcb.vias if v.net_id == nid], pads,
            [z for z in (pcb.zones or []) if z.net_id == nid])
        out[nid] = (r.get('num_components') or 1,
                    len(r.get('disconnected_pads') or []))
    return out


def invariant():
    print("3. invariant on routed copper: the delete-only passes leave nothing "
          "check_weird calls removable")
    for name in ('rp2350_fpga_eensy_prePlane', 'routed_output'):
        path = evidence(os.path.join(ROOT, 'kicad_files', f'{name}.kicad_pcb'),
                        'routed board')
        pcb = parse_kicad_pcb(path)
        c0, _ = _cats(pcb, tolerance=0)
        before = _net_state(pcb)
        check(f"{name}: the board carries removable copper to test with "
              f"(guard not vacuous)", c0.get('removable-segment', 0) > 0,
              str(dict(c0)))
        prune_redundant_cycles([], pcb, None, clearance=0.1)
        collapse_strict_redundant([], pcb, None)
        c1, _ = _cats(pcb, tolerance=0)
        check(f"{name}: nothing removable or redundant is left",
              not c1.get('removable-segment') and not c1.get('redundant-cycle'),
              str(dict(c1)))
        grew = {k: (c0.get(k, 0), v) for k, v in c1.items() if v > c0.get(k, 0)}
        check(f"{name}: no finding category grew", not grew, str(grew))
        after = _net_state(pcb)
        worse = [n for n in before if after.get(n, (99, 99)) > before[n]]
        check(f"{name}: no net's connectivity got worse", not worse, str(worse))


def routed_esp_prog():
    print("4. plain route.py of esp_prog reaches weird_copper clean")
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
        inp = os.path.join(tmp, 'e.kicad_pcb')
        out = os.path.join(tmp, 'e_r.kicad_pcb')
        env = tool_env()
        py = [sys.executable, '-X', 'utf8']
        subprocess.run(py + [tool('copy_board.py'), src, inp], cwd=ROOT, env=env,
                       capture_output=True, text=True, check=True)
        r = subprocess.run(py + [tool('route.py'), inp, '--output', out], cwd=ROOT,
                           env=env, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', timeout=900)
        check("route.py exits 0", r.returncode == 0, (r.stdout + r.stderr)[-400:]
              if r.returncode else '')
        if r.returncode:
            return
        import re
        m = re.search(r'Strict collapse \(#1063, end of run\): removed (\d+)', r.stdout)
        check("the end-of-run strict collapse removed copper (guard not vacuous)",
              m is not None and int(m.group(1)) > 0,
              m.group(0) if m else 'no end-of-run collapse line')
        check("no in-run strict collapse ran (route.py collapses at the end)",
              not re.search(r'^\s*(Final s|S)trict collapse: removed', r.stdout, re.M))
        evidence(out, 'routed esp_prog')
        pcb = parse_kicad_pcb(out)
        c, f = _cats(pcb)
        check("check_weird finds nothing", not c,
              '; '.join(f"{x['category']} {x['net']} {x['detail']}" for x in f[:5]))
        nets = _net_state(pcb)
        check("every net is connected",
              all(v == (1, 0) for v in nets.values()), str(nets))
        jp = os.path.join(tmp, 'cc.json')
        r = subprocess.run(py + [tool('check_complete.py'), out, '--json', jp],
                           cwd=ROOT, env=env, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', timeout=1800)
        doc = json.load(open(evidence(jp, 'check_complete JSON'), encoding='utf-8'))
        wc = (doc.get('components') or {}).get('weird_copper') or {}
        check("check_complete ran check_weird and graded it clean",
              wc.get('ran') is True and wc.get('clean') is True, str(wc))


def end_of_run_gui():
    print("5. the end-of-run collapse on the GUI front (write model)")
    from improvement_gate import copper_signature
    from route import _late_strict_collapse1063

    def case(keep_input):
        pads = [make_pad(NET, 0, 0, ref='U1', size_x=2.0, size_y=2.0),
                make_pad(NET, 10, 0, ref='U2', size_x=0.6, size_y=0.6)]
        trunk = make_seg(0, 0, 10, 0, net_id=NET)
        in_wiggle = make_seg(-0.5, -0.3, 0.5, -0.3, net_id=NET)
        run_wiggle = make_seg(-0.5, 0.3, 0.5, 0.3, net_id=NET)
        pcb = _board([trunk, in_wiggle, run_wiggle], [], pads)
        name = {NET: '/N7'}.get
        sig = copper_signature([trunk, in_wiggle], [], name)
        rd = {'results': [{'new_segments': [run_wiggle], 'new_vias': []}]}

        def write_model(r):
            drop = {id(s) for s in r.get('segments_to_remove') or []}
            segs = [s for s in (trunk, in_wiggle) if id(s) not in drop]
            segs += [s for x in r['results'] for s in x['new_segments']]
            return {NET: segs}, {}

        n, v = _late_strict_collapse1063(pcb, None, True, rd, write_model, sig,
                                         {'/N7'}, keep_input)
        return n, rd, trunk, in_wiggle, run_wiggle, write_model

    n, rd, trunk, in_wiggle, run_wiggle, wm = case(False)
    check("both buried wiggles removed", n == 2, f"removed {n}")
    check("this run's wiggle left its result's new_segments",
          run_wiggle not in rd['results'][0]['new_segments'])
    check("the input wiggle joined segments_to_remove",
          in_wiggle in (rd.get('segments_to_remove') or []))
    check("the trunk ships", wm(rd)[0][NET] == [trunk])
    n, rd, trunk, in_wiggle, run_wiggle, wm = case(True)
    check("--keep-input-copper: only this run's wiggle goes",
          n == 1 and in_wiggle not in (rd.get('segments_to_remove') or [])
          and run_wiggle not in rd['results'][0]['new_segments'], f"removed {n}")


def main():
    wiggles()
    via_loop()
    invariant()
    routed_esp_prog()
    end_of_run_gui()
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
