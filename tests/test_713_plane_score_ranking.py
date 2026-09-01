#!/usr/bin/env python3
"""#713 item 1: a wall clock no longer decides which placement candidate wins.

`--plane-score-budget` was a 300 s budget over all candidates. On overrun the
scoring loop broke and `plane_islands` / `plane_neck` were stripped from the
rank key of EVERY candidate -- so the same board on a slower machine ranked on
a different key set, could promote a different winner, and (because the kept
set feeds the probe) could probe-route a different set of boards.

The flag is gone. What remains is split by whether the cause is the same for
every candidate:

    uniform      strip and continue -- the run ranks exactly as an un-flagged
                 one would, which is fair because nothing was scored
    non-uniform  REFUSE (exit 3) -- a strip here would mean the winner
                 depended on which candidates happened to score

The load-bearing fact behind all of it: `rank_key` reads the plane terms as
`m.get('plane_islands', 0)`, and 0 is the OPTIMUM of both terms. There is no
neutral sentinel, so partial coverage cannot be ranked at all.
"""
import io
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


print("--- 0 is the OPTIMUM, so partial coverage is unrankable")
from placement import portfolio as pf  # noqa: E402


def _cand(idx, **metrics):
    c = pf.Candidate(index=idx, strategy='s', seed_board='', board='b',
                     poses={}, metrics=dict(metrics), gates={'passed': True})
    return c


_scored = _cand(1, crossings=100, hpwl=10.0, plane_islands=4, plane_neck=9.0)
_unscored = _cand(2, crossings=100, hpwl=10.0)
check("an UNSCORED candidate outranks a scored one on the plane term",
      pf.rank_key(_unscored, 0).plane_islands
      < pf.rank_key(_scored, 0).plane_islands,
      "0 is the best island count -- this is why the strip is all-or-nothing")

print("\n--- the budget flag is gone")
p = subprocess.run([sys.executable, '-X', 'utf8',
                    os.path.join(ROOT, 'py_placer', 'place_portfolio.py'),
                    BOARD, '--out-dir', 'x', '--plane-score-budget', '60'],
                   capture_output=True, text=True, encoding='utf-8',
                   errors='replace', cwd=ROOT)
check("--plane-score-budget is an argparse error", p.returncode == 2,
      f"rc={p.returncode}")
check("and argparse names it", 'unrecognized arguments: --plane-score-budget'
      in (p.stdout + p.stderr), (p.stdout + p.stderr)[-200:])
h = subprocess.run([sys.executable, '-X', 'utf8',
                    os.path.join(ROOT, 'py_placer', 'place_portfolio.py'),
                    '--help'], capture_output=True, text=True,
                   encoding='utf-8', errors='replace', cwd=ROOT)
check("--help advertises no budget for plane scoring",
      'plane-score-budget' not in h.stdout)
check("no seconds-valued budget survives on this main at all",
      'budget' not in h.stdout.lower()
      or 'seconds' not in h.stdout.lower().split('budget')[1][:200],
      "a COUNT is fine; a clock is not")

print("\n--- the two arms, driven through the real scoring loop")
import plane_score as psc     # noqa: E402


_DRIVER = '''\
import sys
sys.path[:0] = [r"{root}/py_placer", r"{root}/py_router", r"{root}/py_tools"]
import plane_score as psc
{stub}
psc.plane_fragility_score_ex = _stub
import place_portfolio
sys.argv = ["place_portfolio.py"] + {argv!r}
sys.exit(place_portfolio.main())
'''


def _run_portfolio(stub_src, plane=('GND',), candidates='3'):
    """One real portfolio run, in ITS OWN PROCESS, with the scorer stubbed.

    A separate process per arm on purpose: several modules in this repo
    announce once per PROCESS, so running two arms in one interpreter makes
    the second arm's output differ for a reason the arm did not cause.

    `plane=None` omits --plane-score entirely -- the OFF arm. It is a separate
    parameter rather than an `extra` list because an earlier draft passed the
    flag through `extra` and then forgot to supply it, so every arm silently
    ran with plane scoring OFF and the OFF-arm assertion passed for the wrong
    reason.
    """
    out = tempfile.mkdtemp(prefix='t713_')
    argv = [BOARD, '--out-dir', out, '--candidates', candidates,
            '--keep', '2', '--route-top', '0', '--no-render',
            '--seed', '0']
    if plane:
        argv += ['--plane-score'] + list(plane)
    src = _DRIVER.format(root=ROOT.replace('\\', '/'), stub=stub_src,
                         argv=argv)
    fd, path = tempfile.mkstemp(suffix='.py')
    os.close(fd)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(src)
    try:
        r = subprocess.run([sys.executable, '-X', 'utf8', path],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=ROOT, timeout=1800)
    finally:
        os.unlink(path)
    return r.returncode, r.stdout, r.stderr, out


def _summary(text):
    for line in text.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            return json.loads(line[len('JSON_SUMMARY:'):].strip())
    return None


# --- non-uniform: a TIMEOUT on the SECOND contender ------------------------
_TIMEOUT_ON_SECOND = """
_calls = [0]
def _stub(board, specs, **kw):
    _calls[0] += 1
    if _calls[0] >= 2:
        return None, psc.PlaneScoreStatus('timeout', 'limit 300s', 301.0)
    return ({'islands': 2, 'neck_sum': 1.0, 'cells': 1, 'per_net': {}},
            psc.PlaneScoreStatus('ok'))
"""

rc, out, err, _d = _run_portfolio(_TIMEOUT_ON_SECOND)
check("a per-candidate timeout REFUSES rather than ranking", rc == 3,
      f"rc={rc}")
check("it says which candidate and how far it got",
      'REFUSING to rank' in err and 'of' in err, err.strip()[-300:])
check("it names the timeout, not a generic unavailability",
      'TIMED OUT' in err, err.strip()[-300:])
check("it tells the reader the two ways out",
      'fix the cause' in err and 'drop the flag' in err, err.strip()[-300:])
s = _summary(out)
check("the refusal is on the wire", s is not None and
      s.get('status') == 'plane_score_refused', str(s))
check("and discloses complete: false, the repo's existing partial marker",
      s is not None and s.get('complete') is False, str(s))
check("the wire records the reason and the coverage",
      s is not None and s['plane_score']['reason'] == 'timeout'
      and s['plane_score']['scored'] == 1, str(s and s.get('plane_score')))

# --- uniform: no pcbnew at all --------------------------------------------
_NO_PCBNEW = """
def _stub(b, sp, **kw):
    return None, psc.PlaneScoreStatus('no_kicad_python')
"""

rc, out, err, _d = _run_portfolio(_NO_PCBNEW)
check("a UNIFORM cause does not refuse -- nothing was scored, so the strip "
      "is fair", rc in (0, 4), f"rc={rc}")
check("it says so and says what it ranked on instead",
      'plane scoring unavailable' in out and 'without the plane terms' in out,
      out.strip()[-400:])
s = _summary(out)
check("the wire distinguishes unavailable from refused",
      s is not None and s['plane_score']['status'] == 'unavailable',
      str(s and s.get('plane_score')))
check("a uniform run is NOT marked incomplete", s is not None
      and s.get('complete') is not False, str(s))

# --- the happy path -------------------------------------------------------
_ALL_OK = """
def _stub(b, sp, **kw):
    return ({'islands': 1, 'neck_sum': 2.0, 'cells': 1, 'per_net': {}},
            psc.PlaneScoreStatus('ok'))
"""

rc, out, err, _d = _run_portfolio(_ALL_OK)
check("a fully scored run succeeds", rc in (0, 4), f"rc={rc}")
s = _summary(out)
check("the wire says ok and how many were scored",
      s is not None and s['plane_score']['status'] == 'ok'
      and s['plane_score']['scored'] == s['plane_score']['of'],
      str(s and s.get('plane_score')))
check("scored == of, i.e. every contender was covered",
      s is not None and s['plane_score']['scored'] > 0, str(s))

# --- flag off -------------------------------------------------------------
_NEVER = """
def _stub(b, sp, **kw):
    raise AssertionError('the scorer must not run without --plane-score')
"""

_rc, _o, _e, _d = _run_portfolio(_NEVER, plane=None, candidates='2')
s = _summary(_o)
check("with no --plane-score the wire says null, not a fake ok",
      s is not None and s.get('plane_score') is None, str(s))

print("\n--- scoring is scoped to VIABLE candidates, as the help text says")
_COUNTING = """
_seen = []
def _stub(board, specs, **kw):
    _seen.append(board)
    print('SCORED_CALL ' + str(len(_seen)))
    return ({'islands': 1, 'neck_sum': 1.0, 'cells': 1, 'per_net': {}},
            psc.PlaneScoreStatus('ok'))
"""

rc, out, err, _d = _run_portfolio(_COUNTING)
s = _summary(out)
_n_calls = out.count('SCORED_CALL ')
check("one refill per contender, no more",
      _n_calls == s['plane_score']['of'],
      f"{_n_calls} refills for {s['plane_score']['of']} contenders")
check("contenders never exceed --candidates (the COUNT that bounds the work)",
      s['plane_score']['of'] <= 3,
      f"of={s['plane_score']['of']} with --candidates 3")

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
