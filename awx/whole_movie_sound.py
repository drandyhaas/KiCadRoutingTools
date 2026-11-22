#!/usr/bin/env python3
"""whole_movie_sound.py -- the sound track of whole_movie.py's film (its --sound): narration, effects and music.

THE NARRATION is each scene's own lines (Scene.says: [(t, words[, t_end])], written beside the scene's captions in
whole_movie.build_scenes, with the run's own numbers), spoken offline (narrate) -- by default by Kokoro-82M, a neural
voice (DEFAULT_VOICE, kokoro:am_puck; --voice kokoro:NAME for another of its voices): installed on first use into a
venv of its own in the user's cache ($XDG_CACHE_HOME or ~/.cache, krt_tts; never in the repo or the user's Python, by
uv where there is one), its weights fetched there once from huggingface.co, the words then spoken with the network off
(kokoro_python, kokoro_ready, kokoro_speak). `python3 whole_movie_sound.py --fetch [kokoro:NAME]` has it ready ahead
of time, for a machine that will film offline. Where it cannot be had the film stops, saying how to fetch it; a macOS
`say` voice (--voice Samantha; `say -v ?` lists them) needs nothing installed. Every line is brought to one loudness
(VOICE_RMS), whichever voice speaks it. A line is spoken from
its moment t of the scene, LEAD after it (its caption fades in first). The film's timeline is then FITTED to the
narration (fit): where a line needs more time than the scene gives it before its next line (or before its t_end, where
what it speaks of leaves the screen), the scene is slowed over that stretch; where its last line needs more than the
scene has left, the scene's last frame is held. A line ends at least GAP before the next line, its t_end and the next
scene, so no line runs into any of them (report checks it): a scene is as long on film as its narration needs, and
never shorter than its own length.

THE EFFECTS are the scenes' own cues (Scene.cues: [(t, effect[, x[, t_end]])]), each a thing a scene draws at its
moment t -- a plan the solve found, a bound it raised, a round of the 2-D geometry, a via changing cell, a lane laid --
put on the fitted timeline exactly as the frames are (moment); and one more at every scene change: a bell where a new
stage lights up on the film's timeline (higher, the later the stage), a breath of air between two scenes of one stage.
x is the effect's colour (the pitch of a ping or a pluck, the size of a round), t_end the end of one that lasts (a
sweep, a glide). Where one kind comes faster than the ear can part them (a search's hundred plans, a diagram's
crossings), the later of two closer than its kind's THIN is left out. Every sound is made here, by numpy and scipy,
from sines, noise and a plucked string; nothing is read from anywhere but the film and its voice.

THE MUSIC is a slow pad of chords in D, a chord to each scene (a long scene's split into parts of about BAR seconds),
over a bass, with a few soft bell notes of the chord, in a made reverb; it rises under the title, resolves on the coda
and dies away at the end.

THE SHORT's sound (whole_movie --short: short_score) is music and effects alone: a four to the floor in D at the
short's tempo (beat_music: kick, clap, hats, a bass in eighths, a plucked arpeggio in sixteenths, a pad, all pumped by
the kick, a crash on the cut into the last clip), and its clips' own cues on the beat with a whoosh into every cut.

Needs numpy and scipy, and ffmpeg for the mix. The three are written as WAVs (score) and mixed by ffmpeg (mux): the music ducked under the voice by a sidechain
compressor, the three summed, a limiter on top, AAC beside the film's own video stream (copied, not encoded again).
"""
KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # #937: a research tool (awx), catalogued, shown at no door

import collections
import json
import math
import os
import shutil
import subprocess
import sys
import time
import wave

import numpy as np

DEFAULT_VOICE = 'kokoro:am_puck'   # the narration's voice where --voice names none (the neural voice, had on first use)
SR = 48000                 # samples a second, every stem
LEAD, GAP = 0.35, 0.45     # a line starts LEAD after its moment, and ends GAP before the next line or scene at least
RATE = 185                 # `say`'s words a minute
VOICE_RMS = 0.098          # every line's loudness, whichever voice speaks it: -20.2 dBFS RMS over the line
BAR = 9.0                  # the music's longest chord
FX_GAIN = 1.7              # the effects over their own levels (theirs a few dB under the music's loudest)
THIN = {'tick': 0.07, 'ping': 0.1, 'round': 0.09, 'blip': 0.14, 'pluck': 0.09, 'thump': 0.35, 'shimmer': 0.3}


# =============================================================================== the narration
def _read(path):
    """a 16-bit WAV's samples, mono, -1..1"""
    with wave.open(path) as w:
        x = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768.0
        if w.getnchannels() > 1:
            x = x.reshape(-1, w.getnchannels()).mean(axis=1)
        if w.getframerate() != SR:
            from scipy.signal import resample_poly
            g = math.gcd(SR, w.getframerate())
            x = resample_poly(x, SR // g, w.getframerate() // g).astype(np.float32)
    return x


def narrate(scenes, work, voice=DEFAULT_VOICE, rate=RATE):
    """every scene's lines spoken into WAVs in `work` -- by macOS `say` (a voice `say -v ?` lists), or by Kokoro-82M
    (a voice 'kokoro:NAME', NAME one its VOICES.md lists, e.g. kokoro:am_michael: kokoro_speak) -- each kept in
    Scene.voiced as (t, words, path, i0, i1, t_end): its moment, its words, its voiced samples i0..i1 (the quiet
    before and after trimmed), and the moment it must be done by (None: the next line's, or the scene's end). A voice
    that cannot be had stops the film: there is no other voice to fall back on"""
    jobs = []
    for i, sc in enumerate(scenes):
        sc.voiced = []
        for k, (t, words, *until) in enumerate(sorted(sc.says, key=lambda x: x[0])):
            jobs.append((sc, t, words, until[0] if until else None, os.path.join(work, f'say{i:02d}_{k}.wav')))
    if voice.startswith('kokoro:'):
        kokoro_speak(voice.split(':', 1)[1], [(words, p) for _sc, _t, words, _u, p in jobs], work)
    else:
        say = shutil.which('say')
        if say is None:
            raise SystemExit('whole_movie: --sound speaks its narration with macOS `say`, and there is none here (or '
                             'ask for a Kokoro voice: --voice kokoro:am_puck)')
        names = subprocess.run([say, '-v', '?'], capture_output=True, text=True).stdout.splitlines()
        if not any(ln.startswith(voice + ' ') for ln in names):
            raise SystemExit(f'whole_movie: `say` has no voice {voice!r} (`say -v ?` lists them)')
        for _sc, _t, words, _u, p in jobs:
            subprocess.run([say, '-v', voice, '-r', str(rate), '-o', p, '--file-format=WAVE',
                            f'--data-format=LEI16@{SR}', words], check=True)
    for sc, t, words, until, p in jobs:
        on = np.nonzero(np.abs(_read(p)) > 0.004)[0]
        sc.voiced.append((t, words, p, int(on[0]) if len(on) else 0, int(on[-1]) + 1 if len(on) else 0, until))


# ------------------------------------------------------------------------------- Kokoro-82M, had on first use
KOKORO_REPO = 'hexgrad/Kokoro-82M'                 # (Apache-2.0: the model, and its voices)
KOKORO_FILES = ('config.json', 'kokoro-v1_0.pth')
KOKORO_PINS = ['kokoro==0.9.4', 'en_core_web_sm @ https://github.com/explosion/spacy-models/releases/download/'
               'en_core_web_sm-3.8.0/en_core_web_sm-3.8.0-py3-none-any.whl']
KOKORO_PY = '3.12'                                 # (kokoro runs on a Python from 3.10 to 3.12)
SAY_HINT = (' -- or film with a macOS `say` voice instead, which needs nothing installed: --voice Samantha (`say -v ?` '
            'lists them)')


def tts_home():
    """the narration engine's own cache, outside the repo and outside the user's Python -- $XDG_CACHE_HOME (else
    ~/.cache)/krt_tts, as placement/heights.py keeps its measurements -- holding its venv and the model's weights"""
    return os.path.join(os.environ.get('XDG_CACHE_HOME') or os.path.expanduser('~/.cache'), 'krt_tts')


def kokoro_python():
    """the Python of the cache's own venv (tts_home()/kokoro-venv) with Kokoro in it, made on first use -- by uv where
    there is one (it brings a Python 3.12 of its own where the machine has none), else by a python3.10 to 3.12 on the
    PATH and its pip. Nothing is installed anywhere else, and nothing for a film without a Kokoro voice. Stops, naming
    the commands to run, where it cannot be made"""
    venv = os.path.join(tts_home(), 'kokoro-venv')
    py = os.path.join(venv, 'Scripts', 'python.exe') if os.name == 'nt' else os.path.join(venv, 'bin', 'python')
    probe = [py, '-I', '-c', 'import kokoro, en_core_web_sm']
    if os.path.isfile(py) and subprocess.run(probe, capture_output=True).returncode == 0:
        return py
    uv = shutil.which('uv')
    base = next((shutil.which(f'python3.{m}') for m in (12, 11, 10) if shutil.which(f'python3.{m}')), None)
    hint = f'uv venv --clear --python {KOKORO_PY} {venv} && uv pip install --python {py} ' \
        + ' '.join(f'"{p}"' for p in KOKORO_PINS)
    if uv:
        steps = [[uv, 'venv', '--clear', '--python', KOKORO_PY, venv], [uv, 'pip', 'install', '--python', py] + KOKORO_PINS]
    elif base:
        steps = [[base, '-m', 'venv', '--clear', venv], [py, '-m', 'pip', 'install'] + KOKORO_PINS]
    else:
        raise SystemExit('whole_movie: a kokoro voice needs Kokoro-82M, installed once into a venv of its own, and '
                         f'there is neither uv nor a Python 3.10 to 3.12 here to make it with -- install uv, then: {hint}'
                         + SAY_HINT)
    print(f'whole_movie: the voice is Kokoro-82M\'s, not yet installed for this tool: installing kokoro (with torch and '
          f'spaCy, about 1 GB) once, for the narration only, into {venv}', flush=True)
    for cmd in steps:
        print('  ' + ' '.join(cmd), flush=True)
        if subprocess.run(cmd).returncode != 0:
            raise SystemExit(f'whole_movie: could not install Kokoro-82M into {venv} (offline?) -- when online, run: '
                             f'{hint}' + SAY_HINT)
    if subprocess.run(probe, capture_output=True).returncode != 0:
        raise SystemExit(f'whole_movie: Kokoro-82M was installed into {venv} but does not import -- run: {hint}'
                         + SAY_HINT)
    return py


def kokoro_ready(name):
    """(the cache venv's Python, its environment) with Kokoro-82M installed and the model's weights and the voice
    `name` fetched, each where the cache has not got it, into tts_home()/hf (a process of its own, given no narration).
    The one step that uses the network, on first use only -- or ahead of time, for a machine that will film offline:
    python3 whole_movie_sound.py --fetch [kokoro:NAME]"""
    py = kokoro_python()
    me = os.path.abspath(__file__)
    env = dict(os.environ, HF_HOME=os.path.join(tts_home(), 'hf'), HF_HUB_DISABLE_TELEMETRY='1')
    if subprocess.run([py, '-I', me, '--kokoro-fetch', name], env=env).returncode != 0:
        raise SystemExit(f'whole_movie: could not fetch Kokoro-82M\'s weights and voice {name!r} into {env["HF_HOME"]} '
                         f'(offline? or no such voice: VOICES.md at huggingface.co/{KOKORO_REPO} lists them) -- when '
                         f'online, run: python3 {me} --fetch kokoro:{name}' + SAY_HINT)
    return py, env


def kokoro_speak(name, jobs, work):
    """the lines [(words, path)] spoken by Kokoro-82M's voice `name` into WAVs (24 kHz), in the cache venv (made, and
    the weights fetched, first where they are not there yet: kokoro_ready); every line spoken in one process with the
    hub offline (HF_HUB_OFFLINE), so the words never leave the machine, torch on KRT_TTS_THREADS threads (default 2)"""
    py, env = kokoro_ready(name)
    me = os.path.abspath(__file__)
    jf = os.path.join(work, 'kokoro_lines.json')
    with open(jf, 'w') as f:
        json.dump(jobs, f)
    t0 = time.time()
    if subprocess.run([py, '-I', me, '--kokoro-speak', name, jf], env=dict(env, HF_HUB_OFFLINE='1')).returncode != 0:
        raise SystemExit(f'whole_movie: Kokoro-82M could not speak the narration in voice {name!r}')
    print(f'whole_movie: Kokoro-82M ({name}) spoke {len(jobs)} lines in {time.time() - t0:.0f} s', flush=True)


def _kokoro_fetch(name):
    """(in the cache venv) the model's files and the voice, each fetched where the cache has not got it"""
    from huggingface_hub import hf_hub_download, try_to_load_from_cache
    miss = [f for f in KOKORO_FILES + (f'voices/{name}.pt',) if not isinstance(try_to_load_from_cache(KOKORO_REPO, f), str)]
    if miss:
        print(f'whole_movie: fetching {", ".join(miss)} of huggingface.co/{KOKORO_REPO} into {os.environ["HF_HOME"]}',
              flush=True)
    for f in miss:
        hf_hub_download(KOKORO_REPO, f)


def _kokoro_speak(name, jobs_path):
    """(in the cache venv, offline) every line of jobs_path [(words, path)] spoken by the voice `name`"""
    import logging
    import warnings
    warnings.simplefilter('ignore')
    logging.getLogger('torch.distributed.elastic.multiprocessing.redirects').setLevel(logging.ERROR)
    import espeakng_loader
    # (espeak-ng, which reads the words misaki's dictionaries have not got, keeps its data path in 160 bytes, and a
    # venv deep in a cache can have a longer one: it is given relative, from beside it)
    os.chdir(os.path.dirname(espeakng_loader.get_data_path()))
    os.environ['ESPEAK_DATA_PATH'] = '.'
    import torch
    torch.set_num_threads(int(os.environ.get('KRT_TTS_THREADS', '2')))
    from kokoro import KPipeline
    pipe = KPipeline(lang_code=name[0], repo_id=KOKORO_REPO)       # (a voice's first letter is its language)
    with open(jobs_path) as f:
        jobs = json.load(f)
    for words, path in jobs:
        a = np.concatenate([np.asarray(r.audio, np.float32).reshape(-1) for r in pipe(words, voice=name)])
        with wave.open(path, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(24000)
            w.writeframes((np.clip(a, -1, 1) * 32767).astype('<i2').tobytes())


def fit(scenes, lead=LEAD, gap=GAP):
    """each narrated scene's timeline fitted to its lines (Scene.knots: film seconds into the scene -> its moment t):
    the stretch from a line's moment to the next line's -- or to its own t_end, the moment it must be done by (what it
    speaks of leaves the screen there) -- is as long on film as the scene gives it, or as the line needs (lead + the
    line + gap) where that is longer: the scene slowed over it. After a last line with no t_end, the scene's rest, its
    last frame held as long as the line needs beyond it. Scene.said: (film seconds into the scene, the line) each line
    starts"""
    for sc in scenes:
        sc.said = []
        if not getattr(sc, 'voiced', None):
            continue
        ts = [v[0] for v in sc.voiced] + [1.0]
        s = ts[0] * sc.secs
        knots = [(0.0, 0.0)] + ([(s, ts[0])] if s > 0 else [])

        def knot(s_, t_):
            if s_ > knots[-1][0] + 1e-9:
                knots.append((s_, t_))
        for k, v in enumerate(sc.voiced):
            until = min(ts[k + 1], v[5]) if v[5] is not None else ts[k + 1]
            have = (until - ts[k]) * sc.secs
            need = lead + (v[4] - v[3]) / SR + gap
            sc.said.append((s + lead, v))
            if until < 1.0:
                s += max(have, need)
                knot(s, until)
                s += (ts[k + 1] - until) * sc.secs        # (the rest up to the next line, as it is)
                knot(s, ts[k + 1])
            else:
                s += have
                knot(s, 1.0)
                if need > have:
                    s += need - have
                    knot(s, 1.0)                  # (the last frame, held)
        sc.knots = knots


def moment(sc, t):
    """film seconds into scene `sc` of its moment t (where its last frame is held, the first of them)"""
    if not sc.knots:
        return t * sc.secs
    for (s0, t0), (s1, t1) in zip(sc.knots, sc.knots[1:]):
        if t1 > t0 and t <= t1:
            return s0 + (max(t, t0) - t0) / (t1 - t0) * (s1 - s0)
    return sc.knots[-1][0]


def report(scenes):
    """the sound's timeline, printed: each scene's start and length on film (its own where its narration slowed it),
    each line's start and length and its words, and the scene's effects by kind -- and every line checked to end GAP
    before the next line, before the scene's end, and before its own t_end"""
    print('whole_movie: the sound -- each scene\'s start, each line\'s start and length, the scene\'s effects')
    acc = 0.0
    for sc in scenes:
        slow = f'   (its own {sc.secs:.1f} s, fitted to its narration)' if sc.span > sc.secs + 0.05 else ''
        print(f'  {acc:6.1f} s  {sc.key:9s} {sc.span:5.1f} s{slow}')
        ends = [s + (v[4] - v[3]) / SR for s, v in sc.said]
        for j, (s, v) in enumerate(sc.said):
            print(f'      line {acc + s:6.1f} s + {(v[4] - v[3]) / SR:4.1f} s: {v[1]}')
            nxt = min([sc.said[j + 1][0] if j + 1 < len(sc.said) else sc.span]
                      + ([moment(sc, v[5])] if v[5] is not None else []))
            assert ends[j] + GAP <= nxt + 1e-6, (sc.key, j, ends[j], nxt)
        cnt = collections.Counter(c[1] for c in sc.cues)
        if cnt:
            print('      effects: ' + ', '.join(f'{n} {k}' for k, n in cnt.most_common()))
        acc += sc.span


def timeline(scenes):
    """every effect on the film's timeline, (film seconds, effect, x, seconds it lasts): the scenes' cues, thinned
    (THIN), and a scene change at each scene's start"""
    ev, acc, prev = [], 0.0, None
    stages = list(dict.fromkeys(sc.stage for sc in scenes if sc.stage))
    for i, sc in enumerate(scenes):
        if i:
            lit = bool(sc.stage) and sc.stage != prev
            ev.append((acc, 'stage' if lit else 'turn', stages.index(sc.stage) / max(1, len(stages) - 1) if lit else 0.5,
                       0.0))
        prev = sc.stage
        for c in sc.cues:
            t0 = acc + moment(sc, c[0])
            ev.append((t0, c[1], c[2] if len(c) > 2 else 0.5, acc + moment(sc, c[3]) - t0 if len(c) > 3 else 0.0))
        acc += sc.span
    ev.sort(key=lambda e: e[0])
    out, last = [], {}
    for e in ev:
        if e[0] - last.get(e[1], -1e9) >= THIN.get(e[1], 0.0):
            out.append(e)
            last[e[1]] = e[0]
    return out


# =============================================================================== the effects
PENTA = (0, 2, 4, 7, 9)                 # D major's pentatonic, from D


def hz(m):
    return 440.0 * 2.0 ** ((m - 69) / 12.0)


def penta(x, base, n):
    """the pentatonic note x of the way (0 to 1) up n notes from MIDI `base` (a D), in Hz"""
    k = int(round(min(1.0, max(0.0, x)) * (n - 1)))
    return hz(base + 12 * (k // 5) + PENTA[k % 5])


def _tt(secs):
    return np.arange(max(1, int(secs * SR))) / SR


def _env(t, attack, tau):
    return np.minimum(1.0, t / max(attack, 1e-4)) * np.exp(-t / tau)


def _bell(f, t, tau):
    return (np.sin(2 * np.pi * f * t) + 0.35 * np.sin(2 * np.pi * 2.76 * f * t) * np.exp(-t / (0.4 * tau))
            + 0.12 * np.sin(2 * np.pi * 5.4 * f * t) * np.exp(-t / (0.2 * tau))) * _env(t, 0.003, tau)


def _band(x, f0, f1, q=1.4, block=256):
    """x through a band-pass whose centre moves from f0 to f1 (log), block by block"""
    from scipy.signal import lfilter
    out, zi = np.empty_like(x), np.zeros(2)
    for i in range(0, len(x), block):
        fc = f0 * (f1 / f0) ** (i / max(1, len(x) - 1))
        w = 2 * np.pi * min(fc, 0.45 * SR) / SR
        al = np.sin(w) / (2 * q)
        b = np.array([al, 0.0, -al]) / (1 + al)
        a = np.array([1.0, -2 * np.cos(w) / (1 + al), (1 - al) / (1 + al)])
        out[i:i + block], zi = lfilter(b, a, x[i:i + block], zi=zi)
    return out


def _norm(y, peak):
    m = float(np.abs(y).max()) if len(y) else 0.0
    return y * (peak / m) if m > 0 else y


def _pluck(f, secs, rng, rho=0.996):
    """a plucked string (Karplus-Strong), period by period"""
    P = max(2, int(round(SR / f)))
    n = int(secs * SR)
    y = np.zeros(n + 2 * P)
    y[:P] = rng.uniform(-1, 1, P)
    y[:P] = np.convolve(y[:P], np.ones(3) / 3, mode='same')
    prev = np.concatenate([[0.0], y[:P - 1]])
    for k in range(1, n // P + 2):
        a, b = (k - 1) * P, k * P
        y[b:b + P] = rho * 0.5 * (y[a:b] + prev)
        prev = y[b - 1:b + P - 1]
    return y[:n] * _env(_tt(n / SR), 0.002, secs * 0.45)


def effect(kind, x, dur, rng):
    """one effect, (samples, pan -1..1)"""
    if kind == 'turn':                   # a scene of the same stage: a breath of air
        t = _tt(0.9)
        y = _band(rng.standard_normal(len(t)), 500, 1800, q=0.8) * _env(t, 0.3, 0.35)
        return _norm(y, 0.035), 0.0
    if kind == 'stage':                  # a stage lights up on the timeline: its bell, over a breath of air
        t = _tt(2.2)
        y = _norm(_bell(penta(x, 74, 9), t, 0.7), 0.07)
        a, _p = effect('turn', 0, 0, rng)
        y[:len(a)] += a
        return y, 0.0
    if kind == 'swell':                  # a panel or a diagram comes in
        t = _tt(1.8)
        f = penta(x, 62, 8)
        y = (np.sin(2 * np.pi * f * t) + 0.6 * np.sin(2 * np.pi * 1.5 * f * t)) * np.minimum(1, t / 0.45) * np.exp(-t / 0.6)
        return _norm(y, 0.04), 0.0
    if kind == 'shimmer':                # things lit: a few high, quiet pings
        t = _tt(0.9)
        y = np.zeros(len(t))
        for j in range(6):
            o = int(rng.uniform(0, 0.35) * SR)
            f = penta(rng.uniform(x * 0.5, 0.5 + x * 0.5), 86, 10)
            y[o:] += _bell(f, t[:len(t) - o], 0.18) * rng.uniform(0.5, 1.0)
        return _norm(y, 0.03), float(rng.uniform(-0.4, 0.4))
    if kind == 'tick':                   # a mark drawn
        t = _tt(0.06)
        y = np.sin(2 * np.pi * penta(x, 86, 10) * t) * _env(t, 0.001, 0.012)
        return _norm(y, 0.03), float(rng.uniform(-0.5, 0.5))
    if kind == 'ping':                   # a plan found: higher, the better it is
        t = _tt(0.5)
        f = penta(x, 74, 11)
        y = (np.sin(2 * np.pi * f * t) + 0.15 * np.sin(2 * np.pi * 3 * f * t)) * _env(t, 0.002, 0.12)
        return _norm(y, 0.035), 0.6 * (x - 0.5)
    if kind == 'bell':                   # a plan the board and the braid move to
        t = _tt(1.8)
        return _norm(_bell(penta(x, 74, 11), t, 0.5), 0.06), 0.6 * (x - 0.5)
    if kind == 'thump':                  # a bound raised
        t = _tt(0.3)
        f = 95 * np.exp(-t / 0.15) + 55
        y = np.sin(2 * np.pi * np.cumsum(f) / SR) * _env(t, 0.003, 0.08)
        return _norm(y, 0.06), 0.0
    if kind == 'chord':                  # proved, passed: the chord of D; kept unproved: an open fifth
        t = _tt(2.6)
        y = np.zeros(len(t))
        notes = (74, 78, 81, 86) if x >= 0.5 else (74, 81)
        for j, m in enumerate(notes):
            o = int(0.07 * j * SR)
            y[o:] += _bell(hz(m), t[:len(t) - o], 0.8)
        return _norm(y, 0.07 if x >= 0.5 else 0.05), 0.0
    if kind == 'mark':                   # something lit for a closer look
        t = _tt(1.0)
        y = _bell(hz(81), t, 0.3)
        o = int(0.12 * SR)
        y[o:] += _bell(hz(86), t[:len(t) - o], 0.3)
        return _norm(y, 0.04), 0.0
    if kind == 'blip':                   # a via changes cell
        t = _tt(0.22)
        f0 = 520 + 300 * x
        f = f0 * (1 + 0.6 * np.minimum(1, t / 0.07))
        y = np.sin(2 * np.pi * np.cumsum(f) / SR) * _env(t, 0.002, 0.06)
        return _norm(y, 0.04), float(rng.uniform(-0.3, 0.3))
    if kind == 'boing':                  # a point bulges into the next cell
        t = _tt(0.6)
        f = 330 * (1 + 0.22 * np.sin(2 * np.pi * 7 * t) * np.exp(-t / 0.18))
        y = np.sin(2 * np.pi * np.cumsum(f) / SR) * _env(t, 0.004, 0.18)
        return _norm(y, 0.045), 0.0
    if kind == 'zip':                    # a run pulled taut: a quick rise, and the string it becomes
        t = _tt(0.22)
        y = np.zeros(int(1.2 * SR))
        y[:len(t)] = _band(rng.standard_normal(len(t)), 500, 5000, q=2.0) * np.minimum(1, t / 0.18)
        y = _norm(y, 1.0)
        p = _pluck(hz(74), 1.0, rng)
        y[len(t) - 200:len(t) - 200 + len(p)] += _norm(p, 0.8)[:len(y) - len(t) + 200]
        return _norm(y, 0.045), 0.0
    if kind in ('slide', 'glide', 'sweep'):     # a thing moving over a while: a via slid, a layout blended, a scan
        d = max(dur, 0.35)
        t = _tt(d + 0.3)
        e = np.minimum(1, t / (0.3 * d)) * np.clip((d + 0.3 - t) / (0.3 + 0.3 * d), 0, 1)
        if kind == 'slide':
            f = hz(69) * 2 ** (7 / 12 * np.minimum(1, t / d))
            y = np.sin(2 * np.pi * np.cumsum(f) / SR) * (1 + 0.1 * np.sin(2 * np.pi * 5 * t)) * e
            return _norm(y, 0.035), 0.0
        lo, hi = (300 + 400 * x, 1400 + 1200 * x) if kind == 'glide' else (800 + 600 * x, 4000 + 2000 * x)
        y = _band(rng.standard_normal(len(t)), lo, hi, q=3.0 if kind == 'sweep' else 1.2) * e
        return _norm(y, 0.022 if kind == 'glide' else 0.02), float(rng.uniform(-0.3, 0.3))
    if kind == 'round':                  # a round of the 2-D geometry's program: louder, the more it holds
        t = _tt(0.12)
        f = 760 + 120 * x
        y = (np.sin(2 * np.pi * f * t) + 0.5 * np.sin(2 * np.pi * 2.45 * f * t)) * _env(t, 0.001, 0.025)
        return _norm(y, 0.012 + 0.03 * x), 0.0
    if kind == 'pluck':                  # a lane laid: rising along the order they are laid in, left to right
        return _norm(_pluck(penta(x, 62, 15), 1.1, rng), 0.05), 0.8 * (x - 0.5)
    if kind == 'snap':                   # onto the grid
        t = _tt(0.08)
        y = rng.standard_normal(len(t)) * _env(t, 0.0005, 0.004) + 0.6 * np.sin(2 * np.pi * 2350 * t) * _env(t, 0.001, 0.02)
        return _norm(y, 0.06 + 0.02 * x), 0.0
    if kind == 'whoosh':                 # from one board to the other
        t = _tt(1.1)
        y = _band(rng.standard_normal(len(t)), 300, 2500 if x < 0.5 else 900, q=0.9) * np.minimum(1, t / 0.45) \
            * np.exp(-np.maximum(0, t - 0.45) / 0.2)
        return _norm(y, 0.04), 0.0
    if kind == 'alert':                  # something short, paid, open
        t = _tt(1.2)
        y = (np.sin(2 * np.pi * hz(50) * t) + 0.8 * np.sin(2 * np.pi * hz(51) * t)) * np.minimum(1, t / 0.08) * np.exp(-t / 0.35)
        return _norm(y, 0.05), 0.0
    raise ValueError(f'whole_movie_sound: no effect {kind!r}')


def _reverb(x, secs, wet, seed=7):
    """x (n, 2) in a made room: each side convolved with its own decaying noise (a stereo impulse response)"""
    from scipy.signal import oaconvolve
    rng = np.random.default_rng(seed)
    t = _tt(secs)
    ir = [_band(rng.standard_normal(len(t)), 2500, 600, q=0.5) * np.exp(-t * 6.9 / secs) for _ in range(2)]
    ir = [h / np.sqrt((h ** 2).sum()) for h in ir]
    w = np.stack([oaconvolve(x[:, c], ir[c])[:len(x)] for c in range(2)], axis=1)
    return (1 - wet) * x + wet * w * (np.abs(x).max() / max(1e-9, np.abs(w).max()))


# =============================================================================== the music
CHORDS = [(38, (54, 57, 61, 64)),       # D maj9 (F# A C# E over D)
          (35, (54, 57, 61, 62)),       # B min9
          (43, (54, 57, 59, 62)),       # G maj9
          (45, (52, 57, 59, 62)),       # A sus4 add9
          (38, (54, 57, 61, 66)),       # D maj7
          (42, (52, 57, 61, 64)),       # F# min7
          (43, (55, 59, 62, 66)),       # G maj7
          (45, (52, 57, 61, 64))]       # A
HOME = (38, (50, 57, 62, 66, 69))       # D, the coda's


def music(scenes, T):
    """the music under the film, (n, 2) at SR: a chord to each scene, a long scene's in parts of about BAR seconds,
    the coda's the home chord; pad and bass in a room, a few bell notes of the chord, in from the start and away at
    the end"""
    from scipy.signal import resample_poly
    sr = SR // 3                                       # (made at a third of the rate: nothing in it is high)
    n = int(round(T * sr))
    out = np.zeros((n, 2))
    rng = np.random.default_rng(11)
    parts, acc, j = [], 0.0, 0
    for sc in scenes:
        k = max(1, int(math.ceil(sc.span / BAR - 0.25)))
        for q in range(k):
            parts.append((acc + q * sc.span / k, acc + (q + 1) * sc.span / k,
                          HOME if sc is scenes[-1] and len(scenes) > 1 else CHORDS[j % len(CHORDS)]))
            j += 1
        acc += sc.span
    for p, (t0, t1, (bass, pad)) in enumerate(parts):
        last = p == len(parts) - 1
        rel = 4.0 if last else 2.6
        a0 = int(t0 * sr)
        m = min(n, int((t1 + rel) * sr)) - a0
        if m <= 0:
            continue
        t = np.arange(m) / sr
        att = 2.6 if p == 0 else 1.6
        env = np.minimum(1, t / att) * np.clip(1 - (t - (t1 - t0)) / rel, 0, 1) ** 1.5
        for v, note in enumerate(pad):
            f = hz(note)
            y = np.zeros(m)
            for h, amp in ((1, 1.0), (2, 0.38), (3, 0.16), (4, 0.07)):
                for c in (-1, 1):
                    y += amp * np.sin(2 * np.pi * f * h * (1 + c * (2 + h) / 1731.0) * t + rng.uniform(0, 2 * np.pi))
            y *= env * (1 + 0.12 * np.sin(2 * np.pi * (0.09 + 0.02 * v) * (t + t0) + v))
            pan = -0.6 + 1.2 * v / max(1, len(pad) - 1)
            out[a0:a0 + m, 0] += y * (1 - pan) * 0.5 * 0.11
            out[a0:a0 + m, 1] += y * (1 + pan) * 0.5 * 0.11
        fb = hz(bass)
        yb = (np.sin(2 * np.pi * fb * t) + 0.25 * np.sin(2 * np.pi * 2 * fb * t)) * env * 0.3
        out[a0:a0 + m] += yb[:, None] * 0.5
        # (a bell note or two of the chord, an octave or two up, somewhere in the part)
        for _b in range(int(rng.integers(1, 3))):
            o = rng.uniform(0.15, 0.8) * (t1 - t0)
            bt = np.arange(min(m - int(o * sr), int(2.5 * sr))) / sr
            if len(bt) < 10:
                continue
            fbl = hz(pad[int(rng.integers(len(pad)))] + 12 * int(rng.integers(1, 3)))
            yb2 = np.sin(2 * np.pi * fbl * bt) * _env(bt, 0.004, 0.7) * 0.05
            s0 = a0 + int(o * sr)
            pan = rng.uniform(-0.7, 0.7)
            out[s0:s0 + len(bt), 0] += yb2 * (1 - pan) * 0.5
            out[s0:s0 + len(bt), 1] += yb2 * (1 + pan) * 0.5
    up = np.stack([resample_poly(out[:, c], 3, 1) for c in range(2)], axis=1)
    N = int(round(T * SR))
    up = np.pad(up, ((0, max(0, N - len(up))), (0, 0)))[:N]
    up = _reverb(up, 2.8, 0.45, seed=3)
    tt = np.arange(N) / SR
    up *= (np.minimum(1, tt / 2.5) * np.clip((T - tt) / 3.5, 0, 1))[:, None]
    return up * (0.045 / max(1e-9, float(np.sqrt((up ** 2).mean()))))


# ------------------------------------------------------------------------------- the short's music (whole_movie --short)
BEAT_PROG = [(38, (62, 66, 69, 74)),    # D
             (35, (59, 62, 66, 71)),    # B minor
             (43, (62, 67, 71, 74)),    # G
             (45, (61, 64, 69, 73))]    # A


def _kick(rng):
    t = _tt(0.32)
    f = 48 + 120 * np.exp(-t / 0.04)
    return np.sin(2 * np.pi * np.cumsum(f) / SR) * np.exp(-t / 0.22) * np.minimum(1, t / 0.002)


def _clap(rng):
    t = _tt(0.25)
    n = _band(rng.standard_normal(len(t)), 1800, 1100, q=0.9)
    env = sum(np.exp(-np.maximum(0, t - o) / 0.012) * (t >= o) for o in (0.0, 0.011, 0.022)) * 0.4 + \
        np.exp(-np.maximum(0, t - 0.03) / 0.09) * (t >= 0.03)
    return _norm(n * env, 1.0)


def _hat(rng, open_=False):
    t = _tt(0.18 if open_ else 0.05)
    n = rng.standard_normal(len(t))
    n = n - np.convolve(n, np.ones(4) / 4, mode='same')            # (the highs only)
    return _norm(n * np.exp(-t / (0.06 if open_ else 0.014)), 1.0)


def _crash(rng):
    t = _tt(1.6)
    n = rng.standard_normal(len(t))
    n = n - np.convolve(n, np.ones(3) / 3, mode='same')
    return _norm(n * np.exp(-t / 0.45) * np.minimum(1, t / 0.003), 1.0)


def beat_music(T, bpm, hits=()):
    """the short's music, (n, 2) at SR: a driving four to the floor in D at `bpm` -- a kick on every beat, a clap on two
    and four, hats on the eighths, a bass on the chord's root in eighths, a plucked arpeggio of the chord in
    sixteenths, a soft pad -- a chord a bar (D, B minor, G, A), all but the kick pumped by it; a crash on each of
    `hits` (seconds: the cuts the film makes the most of), a rise of noise into each, and on the last beat"""
    rng = np.random.default_rng(23)
    N = int(round(T * SR))
    beat = 60.0 / bpm
    nb = int(math.ceil(T / beat)) + 1
    drums, tonal = np.zeros((N, 2)), np.zeros((N, 2))

    def put(buf, y, t0, gain, pan=0.0):
        a = int(round(t0 * SR))
        if a >= N:
            return
        m = min(len(y), N - a)
        buf[a:a + m, 0] += y[:m] * gain * (1 - pan)
        buf[a:a + m, 1] += y[:m] * gain * (1 + pan)
    kick, clap = _kick(rng), _clap(rng)
    for b in range(nb):
        t0 = b * beat
        put(drums, kick, t0, 0.9)
        if b % 2 == 1:
            put(drums, clap, t0, 0.28)
        for e in (0, 1):
            put(drums, _hat(rng, open_=(e == 1 and b % 4 == 3)), t0 + e * beat / 2, 0.07 if e == 0 else 0.11,
                pan=0.25)
        bass, pad = BEAT_PROG[(b // 4) % len(BEAT_PROG)]
        for e in (0, 1):                                            # the bass, its root in eighths, an octave on the off
            t = _tt(beat / 2)
            f = hz(bass + 12 * e)
            y = sum(np.sin(2 * np.pi * h * f * t) / h for h in range(1, 7)) * np.exp(-t / 0.16) * np.minimum(1, t / 0.004)
            put(tonal, y, t0 + e * beat / 2, 0.16)
        for e in range(4):                                          # the arpeggio, in sixteenths
            note = pad[(b * 4 + e) % len(pad)] + (12 if (b // 2) % 2 else 0)
            put(tonal, _pluck(hz(note), 0.5, rng, rho=0.993), t0 + e * beat / 4, 0.05, pan=(-0.5 if e % 2 else 0.5))
        if b % 4 == 0:                                              # the pad, a bar long
            t = _tt(4 * beat + 0.3)
            y = sum(np.sin(2 * np.pi * hz(m_) * t + v_) for v_, m_ in enumerate(pad))
            put(tonal, y * np.minimum(1, t / 0.4) * np.clip((4 * beat + 0.3 - t) / 0.3, 0, 1), t0, 0.022)
    # (every part but the kick pumped by it: down at each beat, back by its middle)
    ph = (np.arange(N) / SR) % beat
    tonal *= (1 - 0.55 * np.exp(-ph / 0.09))[:, None]
    for h in sorted(set(round(x, 4) for x in hits) | {round((nb - 2) * beat, 4)}):
        if 0 <= h < T:
            put(drums, _crash(rng), h, 0.16)
            t = _tt(min(2 * beat, h))
            if len(t) > 10:                                         # (a rise into it)
                y = _band(rng.standard_normal(len(t)), 600, 7000, q=1.5) * (t / t[-1]) ** 2
                put(drums, _norm(y, 1.0), h - len(t) / SR, 0.05)
    out = drums + tonal
    tt = np.arange(N) / SR
    out *= np.clip((T - tt) / 0.25, 0, 1)[:, None]                  # (out with the film's last frame)
    return out * (0.12 / max(1e-9, float(np.sqrt((out ** 2).mean()))))


def short_score(scenes, work, bpm):
    """the short's two stems on its timeline (and a silent voice, for mux): the beat music, its crashes on the cut into
    the last clip; the clips' own effects (their cues, thinned), and a whoosh into each cut"""
    T = sum(sc.span for sc in scenes)
    N = int(round(T * SR))
    ev, acc, cuts = [], 0.0, []
    for i, sc in enumerate(scenes):
        if i:
            cuts.append(acc)
            ev.append((max(0.0, acc - 0.4), 'whoosh', 0.3, 0.0))
        for c in sc.cues:
            t0 = acc + c[0] * sc.span
            ev.append((t0, c[1], c[2] if len(c) > 2 else 0.5, (c[3] - c[0]) * sc.span if len(c) > 3 else 0.0))
        acc += sc.span
    ev.sort(key=lambda e: e[0])
    out, last = [], {}
    for e in ev:
        if e[0] - last.get(e[1], -1e9) >= THIN.get(e[1], 0.0):
            out.append(e)
            last[e[1]] = e[0]
    fx = np.zeros((N, 2))
    for j, (t0, kind, x, dur) in enumerate(out):
        y, pan = effect(kind, x, dur, np.random.default_rng(2000 + j))
        a = int(round(t0 * SR))
        m = min(len(y), N - a)
        if m > 0:
            fx[a:a + m, 0] += y[:m] * (1 - pan)
            fx[a:a + m, 1] += y[:m] * (1 + pan)
    fx = _reverb(fx, 1.2, 0.2, seed=5) * FX_GAIN * 2.2
    mus = beat_music(T, bpm, hits=cuts[-1:])
    stems = {'music': os.path.join(work, 'music.wav'), 'effects': os.path.join(work, 'effects.wav'),
             'voice': os.path.join(work, 'voice.wav')}
    _write(stems['music'], mus)
    _write(stems['effects'], fx)
    _write(stems['voice'], np.zeros((N, 2)))
    print(f'whole_movie: the short\'s sound -- {len(out)} effects ('
          + ', '.join(f'{n} {k}' for k, n in collections.Counter(e[1] for e in out).most_common())
          + f'), music at {bpm} beats a minute, {T:.1f} s')
    return stems


# =============================================================================== the mix
def _write(path, x):
    """(n, 2) floats -1..1 as a 16-bit stereo WAV"""
    y = (np.clip(x, -1, 1) * 32767).astype('<i2')
    with wave.open(path, 'wb') as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(y.tobytes())


def score(scenes, work):
    """the three stems on the fitted timeline, as WAVs in `work`: the voice (each line at its start), the effects
    (timeline), the music; {stem: path}"""
    T = sum(sc.span for sc in scenes)
    N = int(round(T * SR))
    voice = np.zeros(N)
    acc = 0.0
    for sc in scenes:
        for s, (_t, _w, path, i0, i1, _until) in getattr(sc, 'said', []):
            x = _read(path)[i0:i1].astype(float)
            if len(x):
                x *= VOICE_RMS / max(1e-9, float(np.sqrt((x ** 2).mean())))
                x *= min(1.0, 0.95 / max(1e-9, float(np.abs(x).max())))
            r = min(len(x) // 2, int(0.006 * SR))
            if r:
                x[:r] *= np.linspace(0, 1, r)
                x[-r:] *= np.linspace(1, 0, r)
            a = int(round((acc + s) * SR))
            voice[a:a + len(x)] += x[:max(0, N - a)]
        acc += sc.span
    fx = np.zeros((N, 2))
    ev = timeline(scenes)
    for j, (t0, kind, x, dur) in enumerate(ev):
        y, pan = effect(kind, x, dur, np.random.default_rng(1000 + j))
        a = int(round(t0 * SR))
        m = min(len(y), N - a)
        if m > 0:
            fx[a:a + m, 0] += y[:m] * (1 - pan)
            fx[a:a + m, 1] += y[:m] * (1 + pan)
    fx = _reverb(fx, 1.6, 0.25, seed=5) * FX_GAIN
    stems = {'music': os.path.join(work, 'music.wav'), 'effects': os.path.join(work, 'effects.wav'),
             'voice': os.path.join(work, 'voice.wav')}
    mus = music(scenes, T)
    _write(stems['music'], mus)
    _write(stems['effects'], fx)
    _write(stems['voice'], np.stack([voice, voice], axis=1))
    # (each stem's level, as written: its loudness over the stretches it sounds in, and its peak, dBFS)
    db = lambda v: 20 * math.log10(max(v, 1e-9))
    for k_, x_ in (('music', mus), ('effects', fx), ('voice', voice)):
        x_ = np.asarray(x_).reshape(len(x_), -1).mean(axis=1)
        on_ = np.abs(x_) > 1e-4
        print(f'whole_movie: the sound -- {k_}: {db(float(np.sqrt((x_[on_] ** 2).mean())) if on_.any() else 0.0):.1f} '
              f'dBFS RMS where it sounds ({100 * on_.mean():.0f} % of the film), peak {db(float(np.abs(x_).max())):.1f} '
              f'dBFS')
    print(f'whole_movie: the sound -- {sum(len(getattr(sc, "said", [])) for sc in scenes)} lines, {len(ev)} effects ('
          + ', '.join(f'{n} {k}' for k, n in collections.Counter(e[1] for e in ev).most_common()) + f'), {T:.0f} s')
    return stems


def mux(ffmpeg, video, stems, out):
    """the stems mixed by ffmpeg onto the silent `video`, into `out`: the music ducked under the voice (a sidechain
    compressor keyed by the voice), the three summed as they are, a limiter (its lookahead compensated: the sound keeps
    the film's time to the sample); the video stream copied"""
    fc = ('[3:a]asplit=2[key][vo];'
          '[1:a][key]sidechaincompress=threshold=0.024:ratio=2.5:attack=60:release=900:knee=3[duck];'
          '[duck][2:a][vo]amix=inputs=3:normalize=0:duration=longest,alimiter=limit=0.9:level=disabled:latency=1[mix]')
    cmd = [ffmpeg, '-y', '-loglevel', 'error', '-i', video, '-i', stems['music'], '-i', stems['effects'], '-i',
           stems['voice'], '-filter_complex', fc, '-map', '0:v', '-map', '[mix]', '-c:v', 'copy', '-c:a', 'aac', '-b:a',
           '192k', '-movflags', '+faststart', out]
    if subprocess.run(cmd).returncode != 0:
        raise SystemExit('whole_movie: ffmpeg could not mix the sound onto the film')


if __name__ == '__main__':
    # (the Kokoro engine's two steps, run by kokoro_speak with the cache venv's Python; nothing else is run from here)
    if any(a_ in ('--help', '-h') for a_ in sys.argv[1:]):
        print('usage: whole_movie_sound.py --fetch [kokoro:NAME]   -- the neural voice of whole_movie.py --sound had '
              'ahead of time (the sound itself is made by whole_movie.py --sound)')
    elif sys.argv[1:2] == ['--fetch'] and len(sys.argv) <= 3:
        # (the neural voice had ahead of time: installed into the cache and its weights fetched, nothing spoken)
        v_ = sys.argv[2] if len(sys.argv) == 3 else DEFAULT_VOICE
        if not v_.startswith('kokoro:'):
            raise SystemExit(f'whole_movie_sound: --fetch takes a kokoro:NAME voice (a `say` voice needs no fetch), '
                             f'not {v_!r}')
        kokoro_ready(v_.split(':', 1)[1])
        print(f'whole_movie_sound: {v_} is ready in {tts_home()}')
    elif sys.argv[1:2] == ['--kokoro-fetch'] and len(sys.argv) == 3:
        _kokoro_fetch(sys.argv[2])
    elif sys.argv[1:2] == ['--kokoro-speak'] and len(sys.argv) == 4:
        _kokoro_speak(sys.argv[2], sys.argv[3])
    else:
        raise SystemExit('whole_movie_sound.py is the sound of whole_movie.py --sound: run that (or, to have the '
                         'neural voice ahead of time: whole_movie_sound.py --fetch [kokoro:NAME])')
