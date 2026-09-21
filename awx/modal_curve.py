#!/usr/bin/env python3
"""modal_curve.py -- solve_curve.py on Modal: the pages-first instances the
chain wrote (PLAN_PAGES_DUMP) re-solved under a long deterministic budget,
one container per instance, so the curve is measured while the local
machine runs the DET ladder undisturbed (CP-SAT drifts under load).

    modal run awx/modal_curve.py --ks 28,35,41,51 --det 640 --dump tmp/s9/dump

Reads <dump>/k<K>/*_solve1.pb (+ .json) and ships them with solve_curve.py's
SOURCE (uncommitted files are not in the image), writes
<dump>/k<K>/curve_modal.log. A cloud stop may differ from the local one
(modal_solve.py: a byte-identical instance was bistable across containers),
so the chain's own DET-40 objective is the cross-check, not this run.
"""
from __future__ import annotations

import glob
import os
import sys

import modal

for _d in (os.path.dirname(os.path.abspath(__file__)), "/opt/krt/awx"):
    if os.path.isfile(os.path.join(_d, "modal_k.py")):
        sys.path.insert(0, _d)
        break
from modal_k import image          # the same pinned image the sweeps use

app = modal.App("bus622-curve", image=image)


@app.function(cpu=(4, 4), memory=(1024, 8192), timeout=3600, max_containers=8)
def curve(arg: dict) -> str:
    import subprocess
    import tempfile
    d = tempfile.mkdtemp()
    stem = os.path.join(d, 'inst_solve1')
    with open(stem + '.pb', 'wb') as f:
        f.write(arg['pb'])
    with open(stem + '.json', 'w', encoding='utf-8') as f:
        f.write(arg['meta'])
    with open(os.path.join(d, 'solve_curve.py'), 'w', encoding='utf-8') as f:
        f.write(arg['src'])
    cmd = [sys.executable, os.path.join(d, 'solve_curve.py'), stem + '.pb', '--det', str(arg['det']),
           '--workers', str(arg['workers'])] + list(arg.get('extra', []))
    r = subprocess.run(cmd, capture_output=True, text=True)
    import platform
    return (f'# K{arg["k"]} on {platform.node()} python {platform.python_version()}, cmd {" ".join(cmd[2:])}\n'
            + r.stdout + ('\nSTDERR:\n' + r.stderr if r.returncode else ''))


@app.local_entrypoint()
def main(ks: str = "28,35,41,51", det: float = 640.0, workers: int = 4, dump: str = "tmp/s9/dump",
         extra: str = "", tag: str = "curve_modal"):
    here = os.path.dirname(os.path.abspath(__file__))
    src = open(os.path.join(here, 'solve_curve.py'), encoding='utf-8').read()
    jobs = []
    for k in ks.split(','):
        k = k.strip()
        hits = sorted(glob.glob(os.path.join(here, dump, f'k{k}', '*_solve1.pb')))
        if not hits:
            print(f'K{k}: no *_solve1.pb under {dump}/k{k} -- skipped')
            continue
        pb = hits[0]
        meta = open(os.path.splitext(pb)[0] + '.json', encoding='utf-8').read()
        jobs.append({'k': k, 'pb': open(pb, 'rb').read(), 'meta': meta, 'src': src, 'det': det,
                     'workers': workers, 'extra': extra.split() if extra else [], 'dir': os.path.dirname(pb)})
    for job, out in zip(jobs, curve.map(jobs)):
        path = os.path.join(job['dir'], f'{tag}.log')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(out)
        tail = [l for l in out.splitlines() if l.startswith('FINAL') or l.startswith('  at det')]
        print(f'K{job["k"]}: wrote {path}; ' + ' | '.join(tail))
