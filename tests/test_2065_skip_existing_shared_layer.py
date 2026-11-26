#!/usr/bin/env python3
"""#2065: --skip-existing-zones on a layer two nets share keeps the pours.

`--nets "A|B" "A|B" --plane-layers F.Cu B.Cu --skip-existing-zones` said
"zone ... already exists - keeping it" for every net/layer pair, then split
both shared layers again anyway: the multi-net split never read the per-net
keep decision, so a second set of pours was written above the kept one (4
pours became 9), the stitch lattice was laid on that new split, and KiCad's
refill of the output reported open items and isolated copper the input did
not have. The single-net-layer path always honoured the keep.

  1. pour two nets on both outer layers, route, then run the planes step
     again with --skip-existing-zones --stitch-vias (the chain order the
     stitch pass asks for): the output carries exactly the input's pours
     (net, layer, priority), the log says the layers were not split again
     (the reason, not only the count), and every stitch via the run adds
     sits on the KEPT fill of its net on both layers;
  2. only some nets kept: with one net's pour removed from one shared layer,
     the keep run pours that net alone there -- the kept net gets no second
     pour, every kept pour is untouched, the kept net is the split's
     background and the new pour fills above it -- and the layer whose nets
     are all kept is not split;
  3. control: the same pour run WITHOUT --skip-existing-zones still replaces
     the pours (4 pours, one per net and layer);
  4. (kicad-cli present) KiCad's refill + DRC reads the keep-and-stitch
     output as it reads the input: 0 unconnected, 0 isolated copper. Without
     kicad-cli the file exits 77 after 1-3.

    python3 tests/test_2065_skip_existing_shared_layer.py
"""
import collections
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb          # noqa: E402
from kicad_oracle import find_kicad_cli           # noqa: E402
from plane_fill_model import ZoneFillModel        # noqa: E402
import run_utils                                  # noqa: E402

FIXTURE = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')
NETS = ['--nets', '/GND|/VCC', '/GND|/VCC', '--plane-layers', 'F.Cu', 'B.Cu']
SPLIT_LINE = 'Computing zone boundaries for multi-net layer'
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def step(script, *args):
    r = run_utils.check([sys.executable, '-X', 'utf8', run_utils.tool(script), *args],
                        accept=True)
    return r.stdout + r.stderr


def pours(path):
    """The board's net pours as a sorted (net, layer, priority) list."""
    pcb = parse_kicad_pcb(run_utils.evidence(path, 'board'))
    return sorted((z.net_name, z.layer, int(z.priority or 0))
                  for z in pcb.zones if z.net_name)


def drop_pour(path, net, layer):
    """Delete the top-level pour of `net` on `layer` from the board file."""
    text = open(path, encoding='utf-8', newline='').read()
    out, i, dropped = [], 0, 0
    for m in re.finditer(r'\n\t\(zone\b', text):
        if m.start() < i:
            continue
        start = m.start() + 1                   # the tab before "(zone"
        depth, j = 0, m.start() + 2             # its "("
        while True:
            depth += {'(': 1, ')': -1}.get(text[j], 0)
            if depth == 0:
                break
            j += 1
        block = text[start:j + 1]
        if (f'(net "{net}")' in block or f'(net_name "{net}")' in block) \
                and f'(layer "{layer}")' in block:
            out.append(text[i:start])
            i = j + 1
            dropped += 1
    out.append(text[i:])
    open(path, 'w', encoding='utf-8', newline='').write(''.join(out))
    return dropped


def vias_off_kept_fill(kept_path, out_path):
    """The vias out_path adds to kept_path that do not sit on kept_path's fill
    of their own net on BOTH outer layers (the stitch pass's own fill model)."""
    kept, out = parse_kicad_pcb(kept_path), parse_kicad_pcb(out_path)
    models = collections.defaultdict(list)
    for z in kept.zones:
        if z.net_id and getattr(z, 'polygon', None):
            m = ZoneFillModel(kept, z)
            if m.ok:
                models[(z.net_id, z.layer)].append(m)
    old = {(round(v.x, 3), round(v.y, 3)) for v in kept.vias}
    new = [v for v in out.vias if (round(v.x, 3), round(v.y, 3)) not in old]
    off = [v for v in new
           if not all(any(m.query_component(v.x, v.y) is not None
                          for m in models.get((v.net_id, lyr), ()))
                      for lyr in ('F.Cu', 'B.Cu'))]
    return new, off


def kicad_drc(cli, path):
    rpt = path + '.drc.json'
    subprocess.run([cli, 'pcb', 'drc', '--refill-zones', '--format', 'json',
                    '-o', rpt, path], capture_output=True, text=True)
    d = json.load(open(run_utils.evidence(rpt, 'kicad-cli DRC report')))
    kinds = collections.Counter(v['type'] for v in d['violations'])
    return len(d['unconnected_items']), kinds['isolated_copper']


def main():
    tmp = tempfile.mkdtemp(prefix='t2065_')
    skipped = None
    try:
        src = os.path.join(tmp, 'in.kicad_pcb')
        step('copy_board.py', FIXTURE, src)
        p1 = os.path.join(tmp, 'p1.kicad_pcb')
        step('route_planes.py', src, p1, *NETS)
        base = pours(p1)
        check('non-vacuity: the pour step puts both nets on both layers',
              sorted((n, l) for n, l, _p in base) == sorted(
                  (n, l) for n in ('/GND', '/VCC') for l in ('F.Cu', 'B.Cu')), str(base))
        r1 = os.path.join(tmp, 'r1.kicad_pcb')
        step('route.py', p1, r1)
        kept = pours(r1)

        # 1. every pair kept: no second split, stitch on the kept pours
        p2 = os.path.join(tmp, 'p2.kicad_pcb')
        log = step('route_planes.py', r1, p2, *NETS, '--skip-existing-zones',
                   '--stitch-vias', '--stitch-pitch', '1.5')
        check('the keep run says it keeps all four pairs',
              log.count('already exists on') == 4)
        check('the shared layers are not split again (no multi-net split ran)',
              SPLIT_LINE not in log)
        check('the log names both layers as kept, not split again',
              all(f'Multi-net layer {l}: every net' in log for l in ('F.Cu', 'B.Cu')))
        got = pours(p2)
        check("the output carries exactly the input's pours (net, layer, priority)",
              got == kept, f'{len(kept)} in, {len(got)} out: {got}')
        new, off = vias_off_kept_fill(r1, p2)
        check('non-vacuity: the stitch pass laid vias', len(new) > 0, log[-600:])
        check('every stitch via sits on the kept fill of its net on both layers',
              not off, f'{len(off)} of {len(new)} off it: '
              + ', '.join(f'({v.x}, {v.y})' for v in off[:5]))

        # 2. only some nets kept on a shared layer
        r1v = os.path.join(tmp, 'r1v.kicad_pcb')
        step('copy_board.py', r1, r1v)
        check('non-vacuity: one pour removed from the partial-keep input',
              drop_pour(r1v, '/VCC', 'F.Cu') == 1)
        part_in = pours(r1v)
        p3 = os.path.join(tmp, 'p3.kicad_pcb')
        log3 = step('route_planes.py', r1v, p3, *NETS, '--skip-existing-zones')
        part_out = pours(p3)
        added = list((collections.Counter(part_out) - collections.Counter(part_in)).elements())
        check('every kept pour is still there, unchanged',
              not (collections.Counter(part_in) - collections.Counter(part_out)),
              f'in {part_in}, out {part_out}')
        check('only the net with no pour on the layer gets one there',
              added and all((n, l) == ('/VCC', 'F.Cu') for n, l, _p in added), str(added))
        kept_gnd = max(p for n, l, p in part_in if (n, l) == ('/GND', 'F.Cu'))
        check("the new pour fills above the kept net's pour on the layer",
              added and min(p for _n, _l, p in added) > kept_gnd, str(added))
        check('the kept net is the split background',
              "'/GND' = background" in log3,
              '; '.join(re.findall(r"Spine split: '[^']*' = background", log3)) or 'no split line')
        check('the layer whose nets are all kept is not split',
              'Multi-net layer B.Cu: every net' in log3 and log3.count(SPLIT_LINE) == 1)

        # 3. control: without the flag the pours are replaced, not stacked
        p1b = os.path.join(tmp, 'p1b.kicad_pcb')
        step('route_planes.py', p1, p1b, *NETS)
        rep = pours(p1b)
        check('control: without --skip-existing-zones the pours are replaced '
              '(one per net and layer)',
              len(rep) == 4 and len({(n, l) for n, l, _p in rep}) == 4, str(rep))

        # 4. KiCad's own refill
        cli = find_kicad_cli(warn=False)
        if not cli:
            skipped = 'kicad-cli not present: the KiCad refill + DRC leg did not run'
        else:
            u_in, iso_in = kicad_drc(cli, r1)
            check('non-vacuity: KiCad reads the routed input clean',
                  (u_in, iso_in) == (0, 0), f'unconnected {u_in}, isolated {iso_in}')
            u, iso = kicad_drc(cli, p2)
            check("KiCad's refill of the keep-and-stitch output: 0 unconnected, "
                  "0 isolated copper", (u, iso) == (0, 0),
                  f'unconnected {u}, isolated_copper {iso}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print('FAILED: ' + ', '.join(failures))
        return 1
    if skipped:
        print(f'SKIP: {skipped}')
        return 77
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
