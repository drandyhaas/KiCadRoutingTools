#!/usr/bin/env python3
"""#622 SOURCE-side round-trip: teeth agree with the braid's ride.

The bench's source stubs are input copper nobody re-picks, so a B
dogbone whose lane rides F pays the dogbone via plus an immediate
return (the U1-bottom pattern; collapse_dives proved pure-F escapes
exist at those balls). This driver closes the loop at the SOURCE:

  1. braid PLAN-ONLY on the fanout board -> the braid's own pages
     and each net's tooth layer;
  2. every net whose tooth layer differs from its PAGE is relayed IN
     PLACE onto the page layer (relay_net --keep-pos; a MISS -- the
     engine cannot put that ball on that layer -- keeps the original
     tooth, honestly);
  3. re-dump the plan on the relayed board (the stale-frame
     protocol) and PIN the new pages as the sidecar, so the final
     braid agrees with the frame the teeth were re-laid for.

History says source perturbations flip nets open at high K -- but
those experiments moved teeth to different GAPS; this flips only the
LAYER at the same position, the smallest possible source move.

usage: src_roundtrip.py FO_BOARD K   (mutates FO_BOARD in place)
"""
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

FO, K = sys.argv[1], sys.argv[2]
NETS = subprocess.run([sys.executable, 'coherent_nets.py', K],
                      capture_output=True, text=True).stdout.strip()
ENV = dict(os.environ, TWO_PAGE='1')
STEM = os.path.splitext(FO)[0]


def dump_plan(tag):
    p = f'{STEM}_srt_{tag}.json'
    subprocess.run(
        [sys.executable, 'braid.py', '--board', FO, '--dest', 'DU1',
         '--nets', NETS, '--out', os.devnull, '--plan-json', p],
        env=ENV, capture_output=True, text=True)
    return json.load(open(p))


plan = dump_plan('r0')
mis = sorted(n for n, d in plan.items()
             if d.get('page') and d.get('tooth_layer')
             and d['tooth_layer'] != d['page'])
print(f'SRC-RT: {len(mis)} tooth/page mismatch(es): {",".join(mis)}'
      if mis else 'SRC-RT: no tooth/page mismatches')
relaid = []
for n in mis:
    want = plan[n]['page']
    out = f'{STEM}_srt_{n}.kicad_pcb'
    r = subprocess.run(
        [sys.executable, 'relay_net.py', FO, n, '--out', out,
         '--keep-pos', '--layer', want],
        capture_output=True, text=True)
    ok = os.path.exists(out) and 'OBEYED' in r.stdout \
        and 'MISSED' not in r.stdout
    if not ok:
        print(f'SRC-RT: {n} -> {want[0]} relay refused '
              '(ball cannot escape on that layer); tooth kept')
        if os.path.exists(out):
            os.remove(out)
        continue
    shutil.copy(out, FO)
    pro = os.path.splitext(out)[0] + '.kicad_pro'
    if os.path.exists(pro):
        shutil.copy(pro, STEM + '.kicad_pro')
    os.remove(out)
    relaid.append(n)
    print(f'SRC-RT: {n} tooth -> {want}')
if relaid:
    plan2 = dump_plan('r1')
    pages = {n: d.get('page') for n, d in plan2.items()}
    with open(STEM + '.pages.json', 'w') as f:
        json.dump(pages, f, indent=1)
    left = sorted(n for n, d in plan2.items()
                  if d.get('page') and d.get('tooth_layer')
                  and d['tooth_layer'] != d['page'])
    print(f'SRC-RT: {len(relaid)} tooth(s) re-laid, pages PINNED; '
          f'{len(left)} mismatch(es) remain'
          + (f' ({",".join(left)})' if left else ''))
