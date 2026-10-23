#!/usr/bin/env python3
"""Route a placement A/B arm on Modal: every placed board and seed in its own
container at once, each by route_placed.py (the board's recorded chain), with
route.py's --quick (level 1, the fast judge) unless --quick 2 or --full.

    route_modal.py PLACED_DIR BOARDS SEEDS --out OUT [--prefix P] [--src TREE]
                   [--corpus DIR] [--quick LEVEL | --full]

  PLACED_DIR  the arm's placements: PLACED_DIR/<board>/s<seed>.kicad_pcb, with
              its .kicad_pro / .kicad_prl / .kicad_dru / .design-brief.json
              siblings when there are any.
  BOARDS      comma list of keys of BOARDS below (ch32,noahfc,...), or
              key=<corpus board>:<set> for any other corpus board.
  SEEDS       0-15, or 0,1,5.
  --out OUT   results land in OUT/<P>_<board>_s<seed>/: score.json,
              chain.log, final.kicad_pcb + .kicad_pro (the graded board),
              result.json (provenance, seconds); failures in OUT/_failed/.
  --prefix P  default: PLACED_DIR's name.
  --src TREE  the tree the chain runs (router and all) -- default this one, AS
              IT STANDS: its tracked and untracked-but-not-ignored files are
              shipped, so an uncommitted change is routed; ignored files
              (work dirs, built libraries) never are.
  --corpus    the stress corpus (default $KICAD_STRESS_CORPUS, else
              ~/Documents/kicad_stress_test).
  --quick 2   the seed judge (route_placed.py --quick 2): about half the
              time of level 1, and it picks seeds as well.
  --full      route the whole recorded chain, no --quick: the final
              route of a chosen placement, minutes rather than seconds. Give
              it its own --prefix (or --out) so it never reads as a fast route.

One line per route as it lands, the shape route_placed.py prints. The app is
named after a hash of everything it ships, so two runs over the same tree share
one deployed app and a changed tree gets its own; it is deployed when missing
and STOPPED once nothing spawned on it is pending. Rerunning the same command
resumes: finished routes are skipped and pending calls collected, never
respawned (a call outlives this client, so a dropped link loses nothing).

Modal routes run on Linux, KiCad's image python and its KiCad (modal_app.py):
compare them only with Modal routes -- route the baseline arm here too.
Placements are not made here: place_modal.py places an arm on the same app.
"""
import argparse
try:                    # POSIX only: Windows runs without the registry lock (one client at a time)
    import fcntl
    _HAS_FCNTL = True
except ImportError:
    _HAS_FCNTL = False
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
THIS_REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))   # tests/stress/placement_ab/
CACHE = os.path.join(os.path.expanduser('~'), '.cache', 'krt_placement_ab')
REG = os.path.join(CACHE, 'apps.json')

#: the placement A/B boards: key -> (corpus board, set)
BOARDS = {
    'ch32': ('ch32v003_usb', '6'),
    'noahfc': ('noahfc', '10'),
    'fusb': ('fusb302_pd_dev', '9'),
    'tig': ('tigard', '3'),
    'bitaxe': ('bitaxe_ultra', '1'),
    'z80': ('z80_devboard_rp2040', '12'),
}


def seeds_of(spec):
    out = []
    for part in spec.split(','):
        lo, _, hi = part.partition('-')
        out += range(int(lo), int(hi or lo) + 1)
    return out


def retry(fn, what, tries=8):
    """`fn()` through a flaky link."""
    for k in range(tries):
        try:
            return fn()
        except Exception as e:  # noqa: BLE001 -- the last try raises
            if k == tries - 1:
                raise
            print(f'  ({what}: {type(e).__name__}: {str(e)[:120]}; retrying)', flush=True)
            time.sleep(min(60, 5 * 2 ** k))


class Registry:
    """apps.json under an flock: {app: {call id: route name}} -- what every
    client still waits for, per app, so the last one out stops the app."""
    def __enter__(self):
        os.makedirs(CACHE, exist_ok=True)
        self.fd = os.open(REG + '.lock', os.O_RDWR | os.O_CREAT, 0o644)
        if _HAS_FCNTL:
            fcntl.flock(self.fd, fcntl.LOCK_EX)
        try:
            self.d = json.load(open(REG))
        except (OSError, ValueError):
            self.d = {}
        return self

    def __exit__(self, *exc):
        with open(REG + '.tmp', 'w') as fh:
            json.dump(self.d, fh, indent=1)
        os.replace(REG + '.tmp', REG)
        if _HAS_FCNTL:
            fcntl.flock(self.fd, fcntl.LOCK_UN)
        os.close(self.fd)


def shipped(src, corpus, boards):
    """(tree files, corpus files, app name) -- the app is named after their content."""
    ls = subprocess.run(['git', '-C', src, 'ls-files', '-co', '--exclude-standard', '-z'],
                        capture_output=True, check=True).stdout.decode().split('\0')
    tree = sorted(f for f in ls if f and os.path.isfile(os.path.join(src, f)) and not f.startswith('awx/'))
    files = []
    for board, setn in sorted(set(boards)):
        rd = f'runs_set{setn}/{board}'
        files += [f'{rd}/{x}' for x in sorted(os.listdir(os.path.join(corpus, rd)))
                  if os.path.isfile(os.path.join(corpus, rd, x))]
        ud = f'boards_unrouted_set{setn}'
        files += [f'{ud}/{x}' for x in sorted(os.listdir(os.path.join(corpus, ud))) if x.startswith(board + '.')]
    h = hashlib.sha1()
    for root, names in ((src, tree), (corpus, files)):
        for f in names:
            h.update(f.encode() + b'\0' + open(os.path.join(root, f), 'rb').read())
    for f in ('route_placed.py', 'modal_app.py'):
        h.update(open(os.path.join(HERE, f), 'rb').read())
    h.update(os.environ.get('KRT_PAB_KICAD_IMAGE', '').encode())
    return tree, files, f'krt-pab-{h.hexdigest()[:10]}'


def deploy(app, src, corpus, tree, files):
    """Stage what the image ships, deploy, and drop the stage (Modal has its copy)."""
    sd = os.path.join(CACHE, 'stage', app)
    shutil.rmtree(sd, ignore_errors=True)
    for root, names, sub in ((src, tree, 'tree'), (corpus, files, 'corpus')):
        for f in names:
            dst = os.path.join(sd, sub, f)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy2(os.path.join(root, f), dst)
    shutil.copy2(os.path.join(HERE, 'route_placed.py'), os.path.join(sd, 'route_placed.py'))
    sha = subprocess.run(['git', '-C', src, 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', '-C', src, 'status', '--porcelain'], capture_output=True, text=True).stdout.strip()
    prov = f"{src}@{sha}{'+dirty' if dirty else ''} {app}"
    log = os.path.join(CACHE, f'deploy_{app}.log')
    print(f'deploying {app} ({prov}); log {log}', flush=True)
    env = {**os.environ, 'KRT_PAB_STAGE': sd, 'KRT_PAB_APP': app, 'KRT_PAB_PROV': prov}
    with open(log, 'w') as fh:
        r = subprocess.run(['modal', 'deploy', os.path.join(HERE, 'modal_app.py')], env=env, cwd=HERE,
                           stdout=fh, stderr=subprocess.STDOUT)
    shutil.rmtree(sd, ignore_errors=True)
    if r.returncode:
        sys.exit(f'deploy of {app} failed (rc {r.returncode}); see {log}')


def deployed(app):
    import modal

    def look():
        try:
            modal.Function.from_name(app, 'route').hydrate()
            return True
        except modal.exception.NotFoundError:
            return False
    return retry(look, f'look up {app}')


def save(out, name, res):
    ok = res.get('rc') == 0 and 'score.json' in res
    d = os.path.join(out, name) if ok else os.path.join(out, '_failed', name)
    os.makedirs(d, exist_ok=True)
    for key, fn in (('score.json', 'score.json'), ('chain.log', 'chain.log'),
                    ('final.kicad_pcb', 'final.kicad_pcb'), ('final.kicad_pro', 'final.kicad_pro')):
        if key in res:
            open(os.path.join(d, fn), 'wb').write(res[key])
    with open(os.path.join(d, 'result.json'), 'w') as fh:
        json.dump({k: v for k, v in res.items() if not isinstance(v, bytes)}, fh, indent=1)
    return ok, d


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('placed_dir')
    ap.add_argument('boards')
    ap.add_argument('seeds')
    ap.add_argument('--out', required=True)
    ap.add_argument('--prefix')
    ap.add_argument('--src', default=THIS_REPO)
    ap.add_argument('--quick', type=int, default=1, choices=(1, 2), metavar='LEVEL')
    ap.add_argument('--full', action='store_true')
    ap.add_argument('--corpus', default=os.environ.get('KICAD_STRESS_CORPUS',
                                                       os.path.expanduser('~/Documents/kicad_stress_test')))
    a = ap.parse_args(argv)
    import modal
    placed_dir, out, src = (os.path.abspath(x) for x in (a.placed_dir, a.out, a.src))
    prefix = a.prefix or os.path.basename(placed_dir.rstrip('/'))
    boards = {}
    for spec in a.boards.split(','):
        key, _, where = spec.partition('=')
        if where:
            board, _, setn = where.partition(':')
            boards[key] = (board, setn)
        elif key in BOARDS:
            boards[key] = BOARDS[key]
        else:
            sys.exit(f'unknown board {key!r}: one of {", ".join(BOARDS)}, or key=<corpus board>:<set>')
    jobs = []
    for key, (board, setn) in boards.items():
        for s in seeds_of(a.seeds):
            name = f'{prefix}_{key}_s{s}'
            if os.path.exists(os.path.join(out, name, 'score.json')):
                print(f'{prefix} {key} s{s}: have', flush=True)
            elif not os.path.exists(os.path.join(placed_dir, key, f's{s}.kicad_pcb')):
                print(f'{prefix} {key} s{s}: NOT PLACED', flush=True)
            else:
                jobs.append((key, s, board, setn, name))
    if not jobs:
        return 0
    # every A/B board is shipped, so arms over different board subsets share an app
    tree, files, app = shipped(src, a.corpus, list(BOARDS.values()) + list(boards.values()))
    mine = {}
    with Registry() as reg:
        pend = reg.d.setdefault(app, {})
        if not deployed(app):
            pend.clear()                    # a stopped app's calls are gone with it
            deploy(app, src, a.corpus, tree, files)
        for key, s, board, setn, name in jobs:
            old = next((c for c, n in pend.items() if n == name), None)
            if old:
                mine[old] = (key, s, name)
                continue
            files_in = {}
            # every sibling copy_board carries: the .kicad_dru's rules and the
            # brief's keep_away reach the route step (#498, #1253)
            for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_prl', '.kicad_dru', '.design-brief.json'):
                p = os.path.join(placed_dir, key, f's{s}{ext}')
                if os.path.exists(p):
                    files_in['placed' + ext] = open(p, 'rb').read()
            fn = modal.Function.from_name(app, 'route')
            call = retry(lambda: fn.spawn(name, board, setn, files_in, 0 if a.full else a.quick), f'spawn {name}')
            pend[call.object_id] = name
            mine[call.object_id] = (key, s, name)
    print(f'{len(mine)} route(s) on {app}', flush=True)
    left = dict(mine)
    while left:
        for cid, (key, s, name) in list(left.items()):
            try:
                res = modal.FunctionCall.from_id(cid).get(timeout=0)
            # both subclass modal's TimeoutError, which below means "not done yet"
            except (modal.exception.FunctionTimeoutError, modal.exception.OutputExpiredError,
                    modal.exception.NotFoundError) as e:
                res = {'name': name, 'rc': -1, 'line': f'CALL LOST ({type(e).__name__}: {str(e)[:160]})'}
            except (TimeoutError, modal.exception.TimeoutError):
                continue
            except (ConnectionError, OSError, modal.exception.ConnectionError):
                continue                    # the link; the call is still running
            except Exception as e:  # noqa: BLE001 -- the function itself raised
                res = {'name': name, 'rc': -1, 'line': f'FAILED ({type(e).__name__}: {str(e)[:200]})'}
            ok, d = save(out, name, res)
            line = res.get('line', '?').split(': ', 1)[-1]
            print(f"{prefix} {key} s{s}: {line}  [{res.get('seconds', 0):.0f} s]" if ok else
                  f"{prefix} {key} s{s}: FAILED rc {res.get('rc')}: {line} (see {d})", flush=True)
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
