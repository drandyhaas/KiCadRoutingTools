#!/usr/bin/env python3
"""Place a placement A/B arm on Modal: every board and seed in its own
container at once, each one `place_simple.py --seed N` of the tree being
measured. The boards land where route_modal.py looks for them.

    place_modal.py OUT BOARDS SEEDS [--prefix P] [--src TREE] [-- PLACE_SIMPLE ARGS...]

  OUT       the arm's directory: OUT/<key>/s<seed>.kicad_pcb (+ the board's
            .kicad_pro / .kicad_prl / .kicad_dru / .design-brief.json
            siblings place_simple writes), s<seed>.json (its report) and
            s<seed>.log (its console). An existing s<seed>.json is kept, so
            rerunning a command resumes it.
  BOARDS    comma list of key=BOARD.kicad_pcb:GROUPS.json:INTENT.json, paths
            from the current directory; the board's siblings ride along.
  SEEDS     0-15, or 0,1,5.
  --src     the tree whose place_simple runs (default this one, AS IT
            STANDS, exactly as route_modal.py ships it -- see there).
  after --   more place_simple arguments, the same for every seed.

The app is route_modal.py's: one image, named after a hash of everything it
ships, deployed when missing and stopped when nothing is pending on it. A
seed's placement is the same placement `place_simple.py --seed N` gives on
any machine with the same Python stack -- but Modal is Linux and the KiCad
image's Python, so place every arm of an A/B here, not some on a Mac.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from route_modal import (BOARDS, Registry, THIS_REPO, deploy, deployed, retry,  # noqa: E402
                         seeds_of, shipped)

SIBLINGS = ('.kicad_pro', '.kicad_prl', '.kicad_dru', '.design-brief.json')


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    extra = argv[argv.index('--') + 1:] if '--' in argv else []
    argv = argv[:argv.index('--')] if '--' in argv else argv
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out')
    ap.add_argument('boards')
    ap.add_argument('seeds')
    ap.add_argument('--prefix', default='place')
    ap.add_argument('--src', default=THIS_REPO)
    ap.add_argument('--corpus', default=os.environ.get('KICAD_STRESS_CORPUS',
                                                       os.path.expanduser('~/Documents/kicad_stress_test')))
    a = ap.parse_args(argv)
    import modal
    out, src = os.path.abspath(a.out), os.path.abspath(a.src)
    specs = {}
    for spec in a.boards.split(','):
        key, _, paths = spec.partition('=')
        parts = paths.split(':')
        if len(parts) != 3 or not all(os.path.isfile(p) for p in parts):
            sys.exit(f'{spec!r}: want key=BOARD.kicad_pcb:GROUPS.json:INTENT.json, all existing files')
        specs[key] = parts
    jobs = []
    for key, (pcb, groups, intent) in specs.items():
        os.makedirs(os.path.join(out, key), exist_ok=True)
        files = {'board.kicad_pcb': open(pcb, 'rb').read(), 'groups.json': open(groups, 'rb').read(),
                 'intent.json': open(intent, 'rb').read()}
        stem = pcb[:-len('.kicad_pcb')]
        for ext in SIBLINGS:
            if os.path.isfile(stem + ext):
                files['board' + ext] = open(stem + ext, 'rb').read()
        for s in seeds_of(a.seeds):
            if os.path.exists(os.path.join(out, key, f's{s}.json')):
                print(f'{a.prefix} {key} s{s}: have', flush=True)
                continue
            args = ['{in}/board.kicad_pcb', f'{{out}}/s{s}.kicad_pcb', '--groups', '{in}/groups.json',
                    '--intent', '{in}/intent.json', '--seed', str(s), '--json', f'{{out}}/s{s}.json', *extra]
            # The call's name carries a digest of its inputs: a pending call
            # is collected rather than respawned by NAME, so two arms that
            # share a key (the same board, different groups) and run at once
            # would otherwise collect each other's placements.
            h = hashlib.sha256()
            for k in sorted(files):
                h.update(k.encode() + b'\0' + files[k])
            h.update('\0'.join(args).encode())
            name = f'{a.prefix}_{key}_s{s}_{h.hexdigest()[:10]}'
            jobs.append((key, s, name, files, args))
    if not jobs:
        return 0
    tree, files_c, app = shipped(src, a.corpus, list(BOARDS.values()))
    mine = {}
    with Registry() as reg:
        pend = reg.d.setdefault(app, {})
        if not deployed(app):
            pend.clear()
            deploy(app, src, a.corpus, tree, files_c)
        fn = modal.Function.from_name(app, 'place')
        for key, s, name, files, args in jobs:
            old = next((c for c, n in pend.items() if n == name), None)
            if old:
                mine[old] = (key, s, name)
                continue
            call = retry(lambda: fn.spawn(name, files, args), f'spawn {name}')
            pend[call.object_id] = name
            mine[call.object_id] = (key, s, name)
    print(f'{len(mine)} placement(s) on {app}', flush=True)
    left = dict(mine)
    while left:
        for cid, (key, s, name) in list(left.items()):
            try:
                res = modal.FunctionCall.from_id(cid).get(timeout=0)
            except (modal.exception.FunctionTimeoutError, modal.exception.OutputExpiredError,
                    modal.exception.NotFoundError) as e:
                res = {'rc': -1, 'log': f'CALL LOST ({type(e).__name__}: {str(e)[:160]})'}
            except (TimeoutError, modal.exception.TimeoutError):
                continue
            except (ConnectionError, OSError, modal.exception.ConnectionError):
                continue
            except Exception as e:  # noqa: BLE001 -- the function itself raised
                res = {'rc': -1, 'log': f'FAILED ({type(e).__name__}: {str(e)[:200]})'}
            d = os.path.join(out, key)
            for k, v in res.items():
                if k.startswith('out/'):
                    with open(os.path.join(d, k[4:]), 'wb') as fh:
                        fh.write(v)
            with open(os.path.join(d, f's{s}.log'), 'w') as fh:
                fh.write(res.get('log', ''))
            rep = os.path.join(d, f's{s}.json')
            if res.get('rc') in (0, 4) and os.path.exists(rep):
                # exit 4: placed and written, with parts no legal pose was found for
                j = json.load(open(rep))
                print(f"{a.prefix} {key} s{s}: {res['seconds']:.0f} s cost {j['final']['cost']:.1f}"
                      + (f" ILLEGAL {j['illegal']}" if j.get('illegal') else ''), flush=True)
            else:
                print(f"{a.prefix} {key} s{s}: FAILED rc {res.get('rc')} (see {d}/s{s}.log)", flush=True)
            del left[cid]
            with Registry() as reg:
                reg.d.get(app, {}).pop(cid, None)
        if left:
            time.sleep(10)
    with Registry() as reg:
        if not reg.d.get(app):
            subprocess.run(['modal', 'app', 'stop', app, '-y'], capture_output=True)
            reg.d.pop(app, None)
            print(f'stopped {app} (nothing pending on it)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
