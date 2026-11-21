#!/usr/bin/env python3
"""py_tools/signal_review.py: signal quality after a route.

On routed_output (a routed BGA board with 61 LVDS pairs, a few seconds):
  1. the review runs end to end, from the CLI too, and writes every key;
  2. a via put on a signal net's own SMD pad is listed, by that pad;
  3. a coupled pair with its N copper moved 2 mm off its P is uncoupled;
  4. a pair renamed with its polarity in the middle of the name leaves the
     pairs and is listed as a name that does not pair, with a rename that
     route_diff pairs again;
  5. `--nets` selects both halves of a pair by its base name.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for d in ('py_tools', 'py_router', 'py_placer'):
    sys.path.insert(0, os.path.join(ROOT, d))
from kicad_parser import Via, parse_kicad_pcb  # noqa: E402
from net_queries import extract_diff_pair_base  # noqa: E402
import signal_review  # noqa: E402

ROUTED = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
results = []


def ok(name, cond, detail=''):
    results.append(cond)
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f" -- {detail}" if not cond and detail else ''))


def _best_pair(r):
    return max(r['diff_pairs'], key=lambda p: p['coupled_fraction'] or 0)


def test_runs_and_writes_its_keys():
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'signal.json')
        p = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_tools', 'signal_review.py'), ROUTED,
                            '--json', out], capture_output=True, text=True)
        ok("the CLI exits 0", p.returncode == 0, p.stderr[-400:])
        ok("the CLI prints its summary", 'signal_review:' in p.stdout, p.stdout[-400:])
        r = json.load(open(out, encoding='utf-8')) if os.path.isfile(out) else {}
    for k in ('nets', 'vias_in_pads', 'diff_pairs', 'near_miss_pairs', 'counts', 'thresholds'):
        ok(f"key {k}", k in r)
    ok("the board's pairs are measured", r.get('counts', {}).get('diff_pairs', 0) > 0,
       str(r.get('counts')))


def test_a_via_in_a_pad_is_listed():
    pcb = parse_kicad_pcb(ROUTED)
    r0 = signal_review.review(ROUTED, pcb=pcb)
    listed = {(v['net'], p) for v in r0['vias_in_pads'] for p in v['pads']}
    pad = next(p for pads in pcb.pads_by_net.values() for p in pads
               if p.net_id and not p.drill and len(pcb.pads_by_net[p.net_id]) >= 2
               and (p.net_name, f"{p.component_ref}.{p.pad_number}") not in listed)
    pcb.vias.append(Via(x=pad.global_x, y=pad.global_y, size=0.3, drill=0.15,
                        layers=['F.Cu', 'B.Cu'], net_id=pad.net_id))
    r = signal_review.review(ROUTED, pcb=pcb)
    label = f"{pad.component_ref}.{pad.pad_number}"
    ok(f"a via on {label} is listed by its pad",
       any(v['net'] == pad.net_name and label in v['pads'] for v in r['vias_in_pads']),
       str(r['vias_in_pads'][:3]))
    ok("the board count grows by one",
       r['counts']['vias_in_pads'] == r0['counts']['vias_in_pads'] + 1, str(r['counts']))


def test_a_moved_n_is_uncoupled():
    pcb = parse_kicad_pcb(ROUTED)
    best = _best_pair(signal_review.review(ROUTED, pcb=pcb))
    ok(f"{best['pair']} is coupled as routed", not best['flags'], str(best))
    n_id = next(nid for nid, n in pcb.nets.items() if n.name == best['n_net'])
    for s in pcb.segments:
        if s.net_id == n_id:
            s.start_x += 2.0
            s.end_x += 2.0
    row = next(p for p in signal_review.review(ROUTED, pcb=pcb)['diff_pairs']
               if p['pair'] == best['pair'])
    ok("with N 2 mm off P it is uncoupled",
       row['coupled_mm'] == 0 and row['flags'] == ['uncoupled'], str(row))


def test_a_mid_name_polarity_does_not_pair():
    pcb = parse_kicad_pcb(ROUTED)
    best = _best_pair(signal_review.review(ROUTED, pcb=pcb))
    base = best['p_net'][:-2]
    head, leaf = base.rsplit('/', 1)
    renamed = {best['p_net']: f"{head}/P_{leaf}", best['n_net']: f"{head}/N_{leaf}"}
    for n in pcb.nets.values():
        n.name = renamed.get(n.name, n.name)
    r = signal_review.review(ROUTED, pcb=pcb)
    ok("the renamed pair is not measured",
       all(p['p_net'] not in renamed.values() for p in r['diff_pairs']))
    miss = [m for m in r['near_miss_pairs'] if m['p'] == renamed[best['p_net']]]
    ok("it is listed as a name that does not pair", bool(miss) and miss[0]['n'] == renamed[best['n_net']],
       str(r['near_miss_pairs']))
    if miss:
        new = miss[0]['rename']
        a, b = (extract_diff_pair_base(new.get(nm, nm)) for nm in (miss[0]['p'], miss[0]['n']))
        ok("its rename pairs for route_diff", a and b and a[0] == b[0] and a[1] and not b[1],
           str(new))


def test_nets_selects_a_pair_by_its_base():
    r0 = signal_review.review(ROUTED)
    best = _best_pair(r0)
    r = signal_review.review(ROUTED, nets=[best['pair']])
    ok("the base name selects the pair", [p['pair'] for p in r['diff_pairs']] == [best['pair']],
       str([p['pair'] for p in r['diff_pairs']]))
    r = signal_review.review(ROUTED, nets=[best['p_net'], best['n_net']])
    ok("its two nets are reviewed",
       sorted(n['net'] for n in r['nets']) == sorted([best['p_net'], best['n_net']]),
       str([n['net'] for n in r['nets']]))


def main():
    print("signal_review: signal quality after a route")
    for fn in (test_runs_and_writes_its_keys, test_a_via_in_a_pad_is_listed,
               test_a_moved_n_is_uncoupled, test_a_mid_name_polarity_does_not_pair,
               test_nets_selects_a_pair_by_its_base):
        print(f"--- {fn.__name__}")
        fn()
    print(f"\n{sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
