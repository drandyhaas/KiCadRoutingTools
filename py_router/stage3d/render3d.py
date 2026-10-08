#!/usr/bin/env python3
"""Drive the stage3d page in headless Chromium and collect its frames (#1081).

Three optional tools, each resolved as `(path, why)` and never raised, like
`kicad_iso_render.resolve_cli`: a callable caller turns a None into a stated
2D fallback, never into a traceback --

  * **Node** (`$KICAD_STAGE3D_NODE`, else `node` on PATH);
  * **playwright-core**, pinned by `package.json` / `package-lock.json` beside
    this file: `npm ci` in `py_router/stage3d` installs it;
  * **a Chromium** (`$KICAD_STAGE3D_CHROMIUM`, else Playwright's own browser
    cache -- the build playwright-core 1.58.2 was cut against first, then any
    other -- else an installed Chrome/Chromium).

**Only SwiftShader is accepted.** The page reports its WebGL renderer, and a
render that did not run on the SwiftShader CPU rasteriser is refused: on a
GPU the pixels depend on the machine's driver, and a film that differs between
machines cannot be pinned by a test. Measured on the spike: SwiftShader draws
byte-identical frames across seven launches and three Chromium builds.
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import sys
from typing import Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
#: A console program started from kicad.exe (the GUI recorder) opens a
#: console window on Windows unless told not to -- ai_gui does the same.
NO_WINDOW = ({'creationflags': subprocess.CREATE_NO_WINDOW}
             if sys.platform.startswith('win') else {})
#: The Chromium revision playwright-core 1.58.2 drives natively.
PREFERRED_REVISION = '1208'
#: Node older than this has no stable ES modules + fetch.
NODE_MIN_MAJOR = 18
#: Seconds per state on top of a fixed start-up allowance. Measured ~0.1 s
#: per 1344x756 state on SwiftShader; the allowance is a HANG guard only.
STATE_BUDGET_S = 3.0
STARTUP_S = 120.0


def resolve_node(explicit=None) -> Tuple[Optional[str], str]:
    cand = explicit or os.environ.get('KICAD_STAGE3D_NODE') or shutil.which(
        'node')
    if not cand:
        return None, 'no Node.js on PATH (set $KICAD_STAGE3D_NODE)'
    try:
        r = subprocess.run([cand, '--version'], capture_output=True,
                           text=True, timeout=30, **NO_WINDOW)
        ver = (r.stdout or '').strip().lstrip('v')
        major = int(ver.split('.')[0])
    except Exception as exc:                                   # noqa: BLE001
        return None, 'could not run %s --version (%s)' % (cand, exc)
    if major < NODE_MIN_MAJOR:
        return None, 'Node %s is older than %d' % (ver, NODE_MIN_MAJOR)
    return cand, 'node %s' % ver


def resolve_playwright() -> Tuple[Optional[str], str]:
    pkg = os.path.join(HERE, 'node_modules', 'playwright-core',
                       'package.json')
    if not os.path.isfile(pkg):
        return None, ('playwright-core is not installed: run `npm ci` in %s'
                      % HERE)
    try:
        with open(pkg, encoding='utf-8') as f:
            ver = json.load(f).get('version', '?')
    except (OSError, ValueError):
        ver = '?'
    return os.path.dirname(pkg), 'playwright-core %s' % ver


def _playwright_cache_dirs() -> List[str]:
    env = os.environ.get('PLAYWRIGHT_BROWSERS_PATH')
    out = [env] if env and env != '0' else []
    home = os.path.expanduser('~')
    if sys.platform.startswith('win'):
        out.append(os.path.join(os.environ.get('LOCALAPPDATA', home),
                                'ms-playwright'))
    elif sys.platform == 'darwin':
        out.append(os.path.join(home, 'Library', 'Caches', 'ms-playwright'))
    else:
        out.append(os.path.join(home, '.cache', 'ms-playwright'))
    return [d for d in out if d and os.path.isdir(d)]


_SHELL_EXE = ('chrome-headless-shell-win64/chrome-headless-shell.exe',
              'chrome-headless-shell-linux64/chrome-headless-shell',
              'chrome-headless-shell-mac-arm64/chrome-headless-shell',
              'chrome-headless-shell-mac-x64/chrome-headless-shell',
              'chrome-linux/headless_shell')
_FULL_EXE = ('chrome-win64/chrome.exe', 'chrome-win/chrome.exe',
             'chrome-linux64/chrome', 'chrome-linux/chrome',
             'chrome-mac-arm64/Google Chrome for Testing.app/Contents/MacOS/'
             'Google Chrome for Testing',
             'chrome-mac/Chromium.app/Contents/MacOS/Chromium')
_SYSTEM = ('C:/Program Files/Google/Chrome/Application/chrome.exe',
           'C:/Program Files (x86)/Google/Chrome/Application/chrome.exe',
           '/usr/bin/chromium', '/usr/bin/chromium-browser',
           '/usr/bin/google-chrome', '/usr/bin/google-chrome-stable',
           '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome')


def _rev(path) -> int:
    try:
        return int(os.path.basename(path).rsplit('-', 1)[1])
    except (IndexError, ValueError):
        return -1


def resolve_browser(explicit=None) -> Tuple[Optional[str], str]:
    cand = explicit or os.environ.get('KICAD_STAGE3D_CHROMIUM')
    if cand:
        return (cand, 'browser %s' % cand) if os.path.isfile(cand) else (
            None, '$KICAD_STAGE3D_CHROMIUM %r is not a file' % cand)
    for root in _playwright_cache_dirs():
        for kind, exes in (('chromium_headless_shell', _SHELL_EXE),
                           ('chromium', _FULL_EXE)):
            dirs = sorted(glob.glob(os.path.join(root, kind + '-*')),
                          key=lambda p: (_rev(p) != int(PREFERRED_REVISION),
                                         -_rev(p)))
            for d in dirs:
                for exe in exes:
                    p = os.path.join(d, *exe.split('/'))
                    if os.path.isfile(p):
                        return p, 'playwright %s-%d' % (kind, _rev(d))
    for p in _SYSTEM:
        if os.path.isfile(p):
            return p, 'system browser %s' % p
    for name in ('chromium', 'chromium-browser', 'google-chrome', 'chrome'):
        p = shutil.which(name)
        if p:
            return p, 'system browser %s' % p
    return None, ('no Chromium: set $KICAD_STAGE3D_CHROMIUM, or `npx '
                  'playwright-core install chromium-headless-shell` in %s'
                  % HERE)


#: Where a Blender is looked for when neither `$KICAD_STAGE3D_BLENDER` nor
#: `blender` on PATH names one (#1089).
_BLENDER_GLOBS = (
    'C:/Program Files/Blender Foundation/Blender */blender.exe',
    os.path.join(os.environ.get('LOCALAPPDATA', ''), 'krt-blender',
                 'blender.exe'),
    '/Applications/Blender.app/Contents/MacOS/Blender',
    '/usr/bin/blender', '/snap/bin/blender')


def resolve_blender(explicit=None) -> Tuple[Optional[str], str]:
    """The Blender for the hi-fi backend (#1089): `$KICAD_STAGE3D_BLENDER`,
    else `blender` on PATH, else a standard install. Returns (path, why)."""
    cand = explicit or os.environ.get('KICAD_STAGE3D_BLENDER')
    if cand:
        return (cand, 'blender %s' % cand) if os.path.isfile(cand) else (
            None, '$KICAD_STAGE3D_BLENDER %r is not a file' % cand)
    p = shutil.which('blender')
    if p:
        return p, 'blender %s' % p
    for pat in _BLENDER_GLOBS:
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1], 'blender %s' % hits[-1]
    return None, ('no Blender: set $KICAD_STAGE3D_BLENDER, or install '
                  'Blender (4.2 LTS or newer)')


def available(backend='three') -> Tuple[bool, str]:
    """`(ok, why)` -- can this machine render the 3D board at all, with
    `backend` ('three' or 'blender')?"""
    if backend == 'blender':
        p, why = resolve_blender()
        return (bool(p), 'ok' if p else why)
    # the no-cost checks first: `npm ci` not run means no process is
    # spawned at all (resolve_node runs `node --version`)
    for fn in (resolve_playwright, resolve_browser, resolve_node):
        p, why = fn()
        if not p:
            return False, why
    return True, 'ok'


def colors_for(theme_name, layers) -> Dict[str, list]:
    """The page's colours from the film's theme: copper by the same
    `layer_palette` the X-ray uses, so a layer is one colour in both."""
    import render_theme
    th = render_theme.theme(theme_name, strict=False)
    pal = render_theme.layer_palette(list(layers), th)

    def c(role, fb):
        try:
            return list(th.rgb(role))
        except Exception:                                      # noqa: BLE001
            return list(fb)
    # the board is WHITE soldermask in the light theme and green in the
    # dark one. Near the light ground a white slab alone read as blank, so
    # its outline is drawn on both faces in the theme's `board_edge`.
    return {'ground': c('ground', (14, 16, 18)),
            'board': [22, 58, 40] if th.name == 'dark' else [250, 250, 247],
            'edge': c('board_edge', (78, 86, 72)),
            'pad': c('pad', (200, 170, 80)), 'via': c('via', (180, 180, 180)),
            'body': [48, 50, 54] if th.name == 'dark' else [70, 72, 76],
            'hilite': c('hilite', (255, 255, 255)),
            'hole': c('pad_hole', (20, 20, 22)),
            'layers': [list(pal.get(n, (200, 120, 60))) for n in layers]}


def render(scene, timeline, *, width, height, out_dir, theme=None,
           glb=None, node=None, browser=None, timeout=None,
           quiet=True, probe=None,
           backend='three') -> Tuple[Optional[List[str]], dict, str]:
    """Render every timeline STATE to `out_dir`. Returns `(pngs, info,
    why)`: `pngs[i]` is state i's frame, or `(None, info, why)` when any part
    of it failed -- and a failure is never a partial list.

    `backend` 'blender' (#1089) renders the SAME scene and timeline with
    Cycles on the CPU (`blender_scene.py`, fixed seed and samples): slower,
    physically lit, and like SwiftShader independent of the machine's GPU."""
    if backend == 'blender':
        return _render_blender(scene, timeline, width=width, height=height,
                               out_dir=out_dir, theme=theme, glb=glb,
                               timeout=timeout)
    node, nwhy = resolve_node(node)
    if not node:
        return None, {}, nwhy
    pw, pwhy = resolve_playwright()
    if not pw:
        return None, {}, pwhy
    browser, bwhy = resolve_browser(browser)
    if not browser:
        return None, {}, bwhy
    os.makedirs(out_dir, exist_ok=True)
    data = os.path.join(out_dir, 'data')
    os.makedirs(data, exist_ok=True)
    sp, tp = os.path.join(data, 'scene.json'), os.path.join(data,
                                                            'timeline.json')
    with open(sp, 'w', encoding='utf-8') as f:
        json.dump(scene, f)
    with open(tp, 'w', encoding='utf-8') as f:
        json.dump(timeline, f)
    frames_dir = os.path.join(out_dir, 'frames')
    theme = theme or _default_theme_name()
    job = {'browser': browser, 'width': int(width), 'height': int(height),
           'outDir': frames_dir, 'scene': sp, 'timeline': tp,
           'glb': (glb or {}).get('path') if glb else None,
           'colors': colors_for(theme, timeline['layers']),
           'probe': probe}
    jp = os.path.join(out_dir, 'job.json')
    with open(jp, 'w', encoding='utf-8') as f:
        json.dump(job, f)
    try:
        n = len(timeline['states'])
        timeline['layers']
    except (KeyError, TypeError) as exc:
        return None, {}, 'the timeline is malformed (%s)' % exc
    if n == 0:
        return None, {}, 'the timeline has no states to render'
    budget = timeout or (STARTUP_S + STATE_BUDGET_S * n)
    try:
        r = subprocess.run([node, os.path.join(HERE, 'render.mjs'), jp],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=budget, cwd=HERE,
                           **NO_WINDOW)
    except subprocess.TimeoutExpired:
        return None, {}, 'the 3D render took over %.0f s (%d states)' % (
            budget, n)
    except OSError as exc:
        return None, {}, 'could not run node (%s)' % exc
    info, done, err = {}, None, None
    for line in (r.stdout or '').splitlines():
        try:
            msg = json.loads(line)
        except ValueError:
            continue
        if not isinstance(msg, dict):
            continue
        if msg.get('type') == 'info':
            info = msg
        elif msg.get('type') == 'probe':
            info['probe'] = msg
        elif msg.get('type') == 'done':
            done = msg
        elif msg.get('type') == 'error':
            err = msg.get('why')
    info['tools'] = '%s; %s; %s' % (nwhy, pwhy, bwhy)
    if err or r.returncode != 0 or done is None:
        blob = err or (r.stderr or '').strip().splitlines()[:1]
        return None, info, 'the 3D render failed: %s' % (
            blob if isinstance(blob, str) else (blob[0] if blob else
                                                'exit %d' % r.returncode))
    rend = str(info.get('renderer') or '')
    if 'swiftshader' not in rend.lower():
        return None, info, ('the 3D render ran on %r, not SwiftShader -- '
                            'refused: its pixels would depend on this '
                            'machine\'s GPU' % rend)
    pngs = [os.path.join(frames_dir, 's%06d.png' % i) for i in range(n)]
    missing = [p for p in pngs if not os.path.isfile(p)]
    if missing:
        return None, info, '%d of %d state frames missing' % (len(missing), n)
    info['ms_per_state'] = done.get('ms_per_state')
    if info.get('glbError'):
        info['glb_note'] = 'part models failed (%s); boxes drawn' % (
            info['glbError'])
    return pngs, info, 'rendered %d states at %.0f ms each (SwiftShader, %s)' % (
        n, done.get('ms_per_state') or 0, bwhy)



def _default_theme_name():
    try:
        import render_theme
        return render_theme.default_theme().name
    except Exception:                                          # noqa: BLE001
        return 'light'


#: Seconds per state for Cycles on the CPU -- a HANG guard, not a budget
#: (measured ~1.3 s per 672x490 state at 8 samples).
BLENDER_STATE_BUDGET_S = 60.0


def _render_blender(scene, timeline, *, width, height, out_dir, theme,
                    glb, timeout):
    """The Blender backend: the three.js job's own files, rendered by
    `blender -b -P blender_scene.py`. Same all-or-nothing contract."""
    blender, why = resolve_blender()
    if not blender:
        return None, {}, why
    try:
        n = len(timeline['states'])
        timeline['layers']
    except (KeyError, TypeError) as exc:
        return None, {}, 'the timeline is malformed (%s)' % exc
    if n == 0:
        return None, {}, 'the timeline has no states to render'
    data = os.path.join(out_dir, 'data')
    os.makedirs(data, exist_ok=True)
    sp, tp = (os.path.join(data, 'scene.json'),
              os.path.join(data, 'timeline.json'))
    with open(sp, 'w', encoding='utf-8') as f:
        json.dump(scene, f)
    with open(tp, 'w', encoding='utf-8') as f:
        json.dump(timeline, f)
    frames_dir = os.path.join(out_dir, 'frames')
    job = {'width': int(width), 'height': int(height), 'outDir': frames_dir,
           'scene': sp, 'timeline': tp,
           'glb': (glb or {}).get('path') if glb else None,
           'colors': colors_for(theme or _default_theme_name(),
                                timeline['layers'])}
    jp = os.path.join(out_dir, 'job.json')
    with open(jp, 'w', encoding='utf-8') as f:
        json.dump(job, f)
    budget = timeout or (STARTUP_S + BLENDER_STATE_BUDGET_S * n)
    try:
        r = subprocess.run([blender, '-b', '--factory-startup', '-P',
                            os.path.join(HERE, 'blender_scene.py'), '--',
                            jp], capture_output=True, text=True,
                           encoding='utf-8', errors='replace',
                           timeout=budget, **NO_WINDOW)
    except subprocess.TimeoutExpired:
        return None, {}, 'the Blender render took over %.0f s (%d states)' % (
            budget, n)
    except OSError as exc:
        return None, {}, 'could not run Blender (%s)' % exc
    info, done, err = {}, None, None
    for line in (r.stdout or '').splitlines():
        if not line.startswith('{'):
            continue
        try:
            msg = json.loads(line)
        except ValueError:
            continue
        if not isinstance(msg, dict):
            continue
        if msg.get('type') == 'info':
            info = msg
        elif msg.get('type') == 'done':
            done = msg
        elif msg.get('type') == 'error':
            err = msg.get('why')
    info['tools'] = why
    if err or done is None:
        return None, info, 'the Blender render failed: %s' % (
            err or 'exit %d' % r.returncode)
    pngs = [os.path.join(frames_dir, 's%06d.png' % i) for i in range(n)]
    missing = [p for p in pngs if not os.path.isfile(p)]
    if missing:
        return None, info, '%d of %d state frames missing' % (len(missing), n)
    # Re-encode without metadata: Cycles writes its render statistics
    # (times, date) into every PNG, so identical pictures were different
    # FILES -- the pixels were already identical run to run.
    from PIL import Image
    for p in pngs:
        with Image.open(p) as im:
            px = im.convert('RGB')
        px.save(p, format='PNG')
    return pngs, info, 'rendered %d states at %.0f ms each (%s)' % (
        n, done.get('ms_per_state') or 0, info.get('renderer'))
