#!/usr/bin/env python3
"""Four small things run 20 tripped over, none of which is about a board.

  8b  `check_reachability --json` is not machine-readable. cli_banner prints
      `CMD:` to stdout before the dump, so `json.load(stdout)` fails at char 0.
  8c  The #562 scoped-`--nets` plane-finalize exclusion was log-only, so a later
      grade could see disconnected plane pads and not tell "declined BY PLAN"
      from "failed".
  8d  `loop_driver` could not tell when an INNER half invoked it -- the run-14
      shape, caught in run 20 only because a watcher was reading the log.
  8e  `converge record --kind` had no word for a classification lap, so the L3
      decision had to be filed as `systemic`.
"""
import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools'),
           os.path.join(REPO, 'py_placer'), os.path.join(REPO, 'tests', 'stress')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

passed = failed = 0


def check(label, cond, detail=''):
    global passed, failed
    if cond:
        passed += 1
        print(f'  OK   {label}')
    else:
        failed += 1
        print(f'  FAIL {label} -- {detail}')


_D = tempfile.mkdtemp()

print('--- 8b: a machine-readable channel that is actually machine-readable ---')

_BOARD = os.path.join(_D, 'r.kicad_pcb')
with open(_BOARD, 'w', encoding='utf-8') as fh:
    fh.write('''(kicad_pcb (version 20260206) (generator test)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))
 (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
 (footprint "t:r" (layer "F.Cu") (at 5 10)
  (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 0.6) (layers "F.Cu") (net 1 "SIG")))
 (footprint "t:r" (layer "F.Cu") (at 15 10)
  (property "Reference" "U2" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 0.6) (layers "F.Cu") (net 1 "SIG")))
 (net 0 "") (net 1 "SIG")
)''')
_CR = os.path.join(REPO, 'py_tools', 'check_reachability.py')
_OUT = os.path.join(_D, 'reach.json')
p = subprocess.run([sys.executable, '-X', 'utf8', _CR, _BOARD,
                    '--pad', 'U1.1', '--track', '0.15',
                    '--json-out', _OUT],
                   capture_output=True, text=True, timeout=300)
check('--json-out writes a file', os.path.isfile(_OUT),
      (p.stdout or '') + (p.stderr or ''))
if os.path.isfile(_OUT):
    try:
        _doc = json.load(open(_OUT, encoding='utf-8'))
        _ok = isinstance(_doc, dict)
    except Exception as exc:                                # noqa: BLE001
        _doc, _ok = None, False
        print(f'       ({exc})')
    check('and json.load succeeds on it', _ok, str(_doc)[:200])

# The defect itself, kept as a regression: stdout is NOT a JSON document, and
# the help must not pretend otherwise.
p2 = subprocess.run([sys.executable, '-X', 'utf8', _CR, _BOARD,
                     '--pad', 'U1.1', '--track', '0.15', '--json'],
                    capture_output=True, text=True, timeout=300)
_stdout_is_json = True
try:
    json.loads(p2.stdout)
except Exception:                                           # noqa: BLE001
    _stdout_is_json = False
check('--help says so when --json stdout is not loadable',
      _stdout_is_json or '--json-out' in (subprocess.run(
          [sys.executable, _CR, '--help'], capture_output=True,
          text=True, timeout=120).stdout or ''),
      'if the banner ever moves off stdout this check passes the other way, '
      'which is the point -- it pins the CONTRACT, not the defect')

print('--- 8e: a classification lap has a word for itself ---')

import converge as CV                                            # noqa: E402

_par = CV.build_parser() if hasattr(CV, 'build_parser') else None
p3 = subprocess.run([sys.executable, os.path.join(REPO, 'py_placer',
                                                  'converge.py'),
                     'record', '--help'], capture_output=True, text=True,
                    timeout=120)
check('`classification` is an accepted --kind',
      'classification' in (p3.stdout or ''), (p3.stdout or '')[:300])
check('and it belongs to NEITHER half, like systemic',
      'classification' not in CV._HALF,
      f'{CV._HALF} -- a lap that changes no board must not be able to make a '
      f'half look like it is still improving')

_LED = os.path.join(_D, 'ledger.jsonl')
_CVP = os.path.join(REPO, 'py_placer', 'converge.py')
p4 = subprocess.run([sys.executable, _CVP, 'record', '--ledger', _LED,
                     '--board', _BOARD, '--kind', 'classification',
                     '--lever', 'L3: placement-shaped, U4.54 caged'],
                    capture_output=True, text=True, timeout=300)
check('recording one works end to end', p4.returncode == 0,
      (p4.stdout or '') + (p4.stderr or ''))
if os.path.isfile(_LED):
    rows = [json.loads(x) for x in open(_LED, encoding='utf-8') if x.strip()]
    check('and it lands in the ledger with its own kind',
          any(r.get('kind') == 'classification' for r in rows),
          str(rows)[:300])

print('--- 8e, the other half: the watcher treats it as a sequence break ---')

import run_watch as RW                                           # noqa: E402

check('`classification` is not a routing kind to the BLOCKING-UP rule',
      'classification' not in RW.ROUTING_KINDS,
      f'{sorted(RW.ROUTING_KINDS)} -- a classification lap between two routing '
      f'laps must RESET the comparison, not join it: the boards either side of '
      f'a re-entry decision are not the same experiment')

print('--- 8d: an inner half driving the outer loop is NAMED ---')

_LD = os.path.join(REPO, '.claude', 'skills',
                   'plan-pcb-placement-and-routing', 'scripts',
                   'loop_driver.py')
sys.path.insert(0, os.path.dirname(_LD))
import loop_driver as LD                                         # noqa: E402
import io                                                        # noqa: E402
import contextlib                                                # noqa: E402

_log = os.path.join(_D, 'loop_driver.log')
with open(_log, 'w', encoding='utf-8') as fh:
    fh.write(json.dumps({'t': 1000.0, 'stage': 'L2', 'board': '/a/frozen.kicad_pcb',
                         'cwd': '/a'}) + '\n')
_err = io.StringIO()
with contextlib.redirect_stderr(_err):
    LD._note_inner_half(_log, {'t': 1005.0, 'stage': 'L2',
                               'board': '/b/placed.kicad_pcb', 'cwd': '/b'})
_msg = _err.getvalue()
check('a same-stage call on a DIFFERENT board seconds later is noted',
      'already invoked' in _msg and 'frozen.kicad_pcb' in _msg
      and 'placed.kicad_pcb' in _msg, repr(_msg)[:400])
check('and the note says it is not a refusal',
      'Not refused' in _msg, repr(_msg)[:400])

_err = io.StringIO()
with contextlib.redirect_stderr(_err):
    LD._note_inner_half(_log, {'t': 1005.0, 'stage': 'L2',
                               'board': '/a/frozen.kicad_pcb', 'cwd': '/a'})
check('a plain re-run of the SAME thing is silent',
      _err.getvalue() == '', repr(_err.getvalue())[:300])

_err = io.StringIO()
with contextlib.redirect_stderr(_err):
    LD._note_inner_half(_log, {'t': 1000.0 + LD._INNER_HALF_WINDOW_S + 60,
                               'stage': 'L2', 'board': '/b/placed.kicad_pcb',
                               'cwd': '/b'})
check('and a call well outside the window is silent -- that is just working',
      _err.getvalue() == '', repr(_err.getvalue())[:300])

_err = io.StringIO()
with contextlib.redirect_stderr(_err):
    LD._note_inner_half(_log, {'t': 1005.0, 'stage': 'L4',
                               'board': '/b/placed.kicad_pcb', 'cwd': '/b'})
check('a DIFFERENT stage is not the shape this looks for',
      _err.getvalue() == '', repr(_err.getvalue())[:300])

print('--- 8c: the finalize exclusion reaches the summary, not just the log ---')

_rt = open(os.path.join(REPO, 'py_router', 'route.py'),
           encoding='utf-8').read()
check("the excluded set is recorded under summary['finalize_excluded_nets']",
      "summary['finalize_excluded_nets'] = _excluded9" in _rt,
      'a later grade can see disconnected plane pads but cannot tell '
      '"declined BY PLAN" from "failed" without this')
check('and the log line is a WARNING, since the printed JSON_SUMMARY '
      'predates the finalize',
      'WARNING: Plane finalize: zone net(s)' in _rt,
      'the summary dict is in _SUMMARY_SINK by reference so --json-out gets '
      'the key, but the already-printed line cannot -- so the log form has to '
      'carry its own severity')

print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
