"""The Modal half of route_modal.py: route_placed.py (with --quick LEVEL unless
the client asks for the full chain), one placed board per container. Deployed by route_modal.py, which stages what it ships
and names the app after a hash of it -- not meant to be deployed by hand.

The image is KiCad's own (default kicad/kicad:10.0.0; KRT_PAB_KICAD_IMAGE
overrides), because the quick pass regrades pours on KiCad's refill through a
pcbnew subprocess, and it must run on KiCad's system python for pcbnew to
import. Linux, that python and that KiCad all differ from a desktop: compare
Modal routes only with Modal routes.

Staged layout (KRT_PAB_STAGE): tree/ (the shipped source tree), corpus/ (the
boards' recorded runs and inputs), route_placed.py (the CLIENT's copy, so a
tree older than this tool still routes).
"""
import os
import sys

import modal

REPO = '/opt/krt'
CORPUS = '/corpus'
STAGE = os.environ.get('KRT_PAB_STAGE', '')
APP = os.environ.get('KRT_PAB_APP', 'krt-pab')
PROV = os.environ.get('KRT_PAB_PROV', 'unknown')
KICAD = os.environ.get('KRT_PAB_KICAD_IMAGE', 'kicad/kicad:10.0.0')
TREE = os.path.join(STAGE, 'tree')

image = (
    modal.Image.from_registry(KICAD, setup_dockerfile_commands=['USER root'])
    # python-is-python3: Modal's builder runs `python -m pip`; PEP 668 needs
    # --break-system-packages on the distro python (the sweep app's recipe)
    .apt_install('curl', 'procps', 'build-essential', 'python3-pip', 'python-is-python3')
    .pip_install('numpy==2.5.2', 'scipy==1.18.0', 'shapely==2.1.2', extra_options='--break-system-packages')
    .run_commands('curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal',
                  'python3 -c "import pcbnew, sys; print(pcbnew.GetBuildVersion(), sys.version.split()[0])"')
    # the crate before the rest of the tree, so a Python-only change reuses the Rust build
    .add_local_dir(os.path.join(TREE, 'rust_router'), f'{REPO}/rust_router', copy=True,
                   ignore=['target/**', '*.so', '*.pyd'])
    .add_local_file(os.path.join(TREE, 'build_router.py'), f'{REPO}/build_router.py', copy=True)
    .add_local_file(os.path.join(TREE, 'VERSION'), f'{REPO}/VERSION', copy=True)
    .add_local_file(os.path.join(TREE, 'metadata.json'), f'{REPO}/metadata.json', copy=True)
    .run_commands(f'cd {REPO} && . $HOME/.cargo/env && python3 build_router.py --from-source',
                  f'rm -rf {REPO}/rust_router/target')
    .add_local_dir(TREE, REPO, copy=True, ignore=['rust_router/**', 'build_router.py', 'VERSION', 'metadata.json'])
    .add_local_dir(os.path.join(STAGE, 'corpus'), CORPUS, copy=True)
    .add_local_file(os.path.join(STAGE, 'route_placed.py'), '/opt/route_placed.py', copy=True)
    .run_commands(f"cd {REPO} && python3 -c \"import sys; sys.path.insert(0, 'rust_router'); "
                  f"import grid_router; print('grid_router', grid_router.__version__)\"")
    .env({'KRT_PAB_PROV': PROV, 'PYTHONUNBUFFERED': '1'})
)

app = modal.App(APP, image=image)


def _route(name, board, setn, files, quick):
    import subprocess
    import time
    d = f'/tmp/{name}'
    os.makedirs(f'{d}/placed', exist_ok=True)
    for fn, data in files.items():
        with open(f'{d}/placed/{fn}', 'wb') as fh:
            fh.write(data)
    t0 = time.time()
    r = subprocess.run([sys.executable, '-X', 'utf8', '/opt/route_placed.py', f'{d}/placed/placed.kicad_pcb',
                        board, setn, f'{d}/{name}', *(['--quick', str(int(quick))] if quick else []),
                        '--corpus', CORPUS, '--repo', REPO],
                       capture_output=True, text=True, cwd=REPO)
    res = {'name': name, 'rc': r.returncode, 'seconds': round(time.time() - t0, 1), 'prov': PROV,
           'line': (r.stdout.strip().splitlines() or ['?'])[-1], 'stderr': r.stderr[-3000:]}
    out = f'{d}/{name}'
    for fn in ('score.json', 'chain.log'):
        if os.path.exists(f'{out}/{fn}'):
            res[fn] = open(f'{out}/{fn}', 'rb').read()
    if not os.path.exists(f'{out}/manifest/redo_commands.sh'):
        return res
    sys.path.insert(0, '/opt')
    from route_placed import final_board
    lines = open(f'{out}/manifest/redo_commands.sh').read().split('\n')
    base = os.path.splitext(os.path.join(out, 'out', os.path.basename(final_board(lines))))[0]
    for ext in ('.kicad_pcb', '.kicad_pro'):
        if os.path.exists(base + ext):
            res['final' + ext] = open(base + ext, 'rb').read()
    return res


# no memory request: Modal sizes the container to what the route uses
@app.function(cpu=4.0, timeout=3 * 3600, retries=0)
def route(name: str, board: str, setn: str, files: dict, quick: int = 1) -> dict:
    return _route(name, board, setn, files, quick)


def _place(name, files, args):
    """place_simple.py with `args`, whose {in} and {out} name the call's input
    files (`files`, written to in/) and its output directory; returns every
    output file's bytes under 'out/<name>'."""
    import subprocess
    import time
    d = f'/tmp/{name}'
    for sub in ('in', 'out'):
        os.makedirs(f'{d}/{sub}', exist_ok=True)
    for fn, data in files.items():
        with open(f'{d}/in/{fn}', 'wb') as fh:
            fh.write(data)
    argv = [x.replace('{in}', f'{d}/in').replace('{out}', f'{d}/out') for x in args]
    t0 = time.time()
    r = subprocess.run([sys.executable, '-X', 'utf8', f'{REPO}/py_placer/place_simple.py', *argv],
                       capture_output=True, text=True, cwd=REPO)
    res = {'name': name, 'rc': r.returncode, 'seconds': round(time.time() - t0, 1), 'prov': PROV,
           'log': (r.stdout + r.stderr)[-20000:]}
    for fn in os.listdir(f'{d}/out'):
        res['out/' + fn] = open(f'{d}/out/{fn}', 'rb').read()
    return res


# one seed per call: place_simple runs a seed in one process
@app.function(cpu=1.0, timeout=3 * 3600, retries=0)
def place(name: str, files: dict, args: list) -> dict:
    return _place(name, files, args)
