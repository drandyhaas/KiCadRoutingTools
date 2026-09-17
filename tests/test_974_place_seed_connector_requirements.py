#!/usr/bin/env python3
"""#974: place_seed reports its declared connector requirements in
JSON_SUMMARY -- abstain and report, never withhold, never move the exit code.

The CLI half of `floorplan.connector_requirements` (whose contract is pinned
in-process by `test_974_connector_requirements.py`). Every summary place_seed
prints carries `connector_requirements`: the fresh seed, `--repair`,
`--reseat`, and the dry runs of both, where it is exactly
`{complete: false, reason: 'dry-run'}`.

Checks are named by what they hold:

* `rc:` / `board:` -- the exit code and the written board. These pass on the
  commit BEFORE the key existed (measured: every `rc:` and `board:` check
  green, every `key:` check red), so they pin the codes the change must not
  move. The codes are that measurement, not a policy restated.
* `key:` -- the report itself.
* `ab:` -- the exit code does not depend on the report. Each arm runs twice in
  its own process, once as shipped and once with the report's body forced to
  raise; the rc, the board bytes and the summary minus the key must agree.

Issue acceptance, by arm: (1) a body-less connector writes the board at its
old rc and is listed unmeasured -- `pinned`, `unlocked`, `shorts`; (2) a band
failure writes the board with rc 4 and names it -- `reseat_other`; (3) the
run27-pinned fixture reports its error pinned with rc 0 -- `pinned`; (4) the
board exists at every rc a non-dry run exits with.

Run:
    python3 tests/test_974_place_seed_connector_requirements.py
"""
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

import test_run27_seed_gate_pinned as run27        # noqa: E402
import test_run27_seed_gate_shorts as shorts       # noqa: E402

RUN_ALL_FAST_OK = True

SEED = os.path.join(ROOT, 'py_placer', 'place_seed.py')
KEY = 'connector_requirements'
DRY = {'complete': False, 'reason': 'dry-run'}

#: A PLACED board for the --repair / --reseat arms. J1's drawn Fab body
#: reaches 1.5 mm past the west edge against a declared maximum of 0.5 mm;
#: its pads are on the board, so the band is the finding.
PLACED = '''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "/A") (net 2 "/B") (net 3 "/C")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
 (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "test:USB" (layer "F.Cu") (uuid "fp-j1") (at -0.5 10)
   (property "Reference" "J1" (at 0 0 0))
   (fp_rect (start -1 -1.5) (end 1 1.5) (layer "F.Fab") (uuid "j1fab"))
   (pad "1" smd rect (at 0.9 -1) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A") (uuid "j1p1"))
   (pad "2" smd rect (at 0.9 1) (size 0.6 0.6) (layers "F.Cu") (net 2 "/B") (uuid "j1p2"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-u1") (at 15 10)
   (property "Reference" "U1" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A") (uuid "u1p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 3 "/C") (uuid "u1p2"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-u2") (at 20 6)
   (property "Reference" "U2" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "/B") (uuid "u2p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 3 "/C") (uuid "u2p2"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-r1") (at 10 15)
   (property "Reference" "R1" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A") (uuid "r1p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "/B") (uuid "r1p2"))
 )
)
'''

PLACED_INTENT = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                 'edge_connectors': [{'ref': 'J1', 'edge': 'west',
                                      'overhang_mm': {'min': 0.0,
                                                      'max': 0.5}}]}

#: The run27 pile with J1's drawn body flush on the west edge: every declared
#: requirement is measurable, so the report is complete.
FLUSH = run27.BOARD.replace(
    '(at 3 10)%s', '(at 1 10)%s').replace(
    '(property "Reference" "J1" (at 0 0 0))',
    '(property "Reference" "J1" (at 0 0 0))\n'
    '   (fp_rect (start -1 -1.5) (end 1 1.5) (layer "F.Fab") (uuid "j1fab"))')

#: In-process runner for the `ab:` arms: `place_seed.main()` with the report's
#: body optionally forced to raise. One process per run, so nothing a module
#: prints or caches once per process can differ between the two arms.
RUNNER = '''
import sys
sys.path[:0] = [{py_placer!r}, {py_router!r}]
from placement import floorplan
if sys.argv[1] == 'boom':
    def _boom(*a, **k):
        raise RuntimeError('forced by the #974 A/B')
    floorplan._connector_requirements = _boom
sys.argv = [{seed!r}] + sys.argv[2:]
import place_seed
sys.exit(place_seed.main())
'''.format(py_placer=os.path.join(ROOT, 'py_placer'),
           py_router=os.path.join(ROOT, 'py_router'), seed=SEED)


def _env():
    return dict(os.environ, PYTHONHASHSEED='0', KRT_NO_BANNER='1')


def _summary(stdout):
    found = None
    for ln in (stdout or '').splitlines():
        if ln.startswith('JSON_SUMMARY: '):
            found = json.loads(ln[len('JSON_SUMMARY: '):])
    return found


def _write(tmp, name, board, intent):
    bpath = os.path.join(tmp, f'{name}.kicad_pcb')
    with open(bpath, 'w', encoding='utf-8') as fh:
        fh.write(board)
    ipath = os.path.join(tmp, f'{name}.json')
    with open(ipath, 'w', encoding='utf-8') as fh:
        json.dump(intent, fh)
    return bpath, ipath


def _cli(bpath, ipath, out, extra):
    if os.path.exists(out):
        os.remove(out)
    r = subprocess.run([sys.executable, '-X', 'utf8', SEED, bpath, out,
                        '--intent', ipath, '--seed', '0'] + list(extra),
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, env=_env())
    return r.returncode, _summary(r.stdout), (r.stdout or '') + (r.stderr or '')


def _runner(mode, bpath, ipath, out, extra):
    if os.path.exists(out):
        os.remove(out)
    r = subprocess.run([sys.executable, '-X', 'utf8', '-c', RUNNER, mode,
                        bpath, out, '--intent', ipath, '--seed', '0']
                       + list(extra),
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, env=_env())
    data = None
    if os.path.exists(out):
        with open(out, 'rb') as fh:
            data = fh.read()
    return r.returncode, _summary(r.stdout), data, r.stderr or ''


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    def entries(rep, requirement):
        return [u['ref'] for u in (rep or {}).get('unmeasured') or ()
                if u.get('requirement') == requirement]

    def graded_invariants(arm, rc, s):
        """What must hold on every summary of a graded run."""
        rep = (s or {}).get(KEY)
        check(f'key: {arm}: the summary carries the report',
              isinstance(rep, dict) and 'reason' not in rep, f'{rep}')
        if not isinstance(rep, dict):
            return {}
        own, pinned = rep.get('errors_own'), rep.get('errors_pinned')
        check(f'key: {arm}: rc 0 never sits next to an own connector error',
              rc != 0 or own == [], f'rc {rc} own {own}')
        check(f'key: {arm}: its errors are within the counts the gate used',
              isinstance(own, list) and isinstance(pinned, list)
              and len(own) <= s.get('grade_errors', -1)
              and len(pinned) <= s.get('grade_errors_pinned', -1),
              f'{s.get("grade_errors")}/{s.get("grade_errors_pinned")} {rep}')
        check(f'key: {arm}: every reported error is a connector error',
              all(e.get('rule') == 'edge_connector'
                  for e in (own or []) + (pinned or [])), f'{rep}')
        check(f'key: {arm}: `complete` is its definition',
              rep.get('complete') == (not rep.get('unmeasured')
                                      and not rep.get('bands_dropped')),
              f'{rep}')
        return rep

    with tempfile.TemporaryDirectory(prefix='t974_cli_') as tmp:
        def out_of(name):
            return os.path.join(tmp, f'{name}_out.kicad_pcb')

        # --- fresh seed: run27's pinned receptacle (acceptance 1 and 3) ---
        b, i = _write(tmp, 'pinned', run27.BOARD % ' (locked yes)',
                      run27._intent([8, 5, 22, 15], must_lock=['J1']))
        rc, s, text = _cli(b, i, out_of('pinned'), [])
        check('rc: pinned: a pinned connector error does not fail the seed',
              rc == 0, f'rc {rc}\n{text[-900:]}')
        check('board: pinned: written', os.path.exists(out_of('pinned')))
        rep = graded_invariants('pinned', rc, s)
        check('key: pinned: the error is reported PINNED, and none as own',
              rep.get('errors_own') == []
              and [e.get('ref') for e in rep.get('errors_pinned') or ()]
              and all(e.get('ref') == 'J1'
                      for e in rep.get('errors_pinned') or ()), f'{rep}')
        check('key: pinned: the body-less receptacle is listed unmeasured',
              entries(rep, 'overhang_body') == ['J1']
              and rep.get('complete') is False, f'{rep}')

        # --- fresh seed: the same receptacle, not locked -------------------
        b, i = _write(tmp, 'unlocked', run27.BOARD % '',
                      run27._intent([8, 5, 22, 15]))
        rc, s, text = _cli(b, i, out_of('unlocked'), [])
        check('rc: unlocked: exit code as before the report', rc == 0,
              f'rc {rc}\n{text[-900:]}')
        check('board: unlocked: written', os.path.exists(out_of('unlocked')))
        rep = graded_invariants('unlocked', rc, s)
        check('key: unlocked: the body-less receptacle is listed unmeasured',
              entries(rep, 'overhang_body') == ['J1'], f'{rep}')
        check('key: unlocked: nothing is reported pinned',
              rep.get('errors_pinned') == [], f'{rep}')

        # --- fresh seed: run27's own-error arm (zone too small) ------------
        b, i = _write(tmp, 'small_zone', run27.BOARD % ' (locked yes)',
                      run27._intent([14.5, 9.5, 15.5, 10.5],
                                    must_lock=['J1']))
        rc, s, text = _cli(b, i, out_of('small_zone'), [])
        check('rc: small_zone: an own error still fails the seed', rc == 4,
              f'rc {rc}\n{text[-900:]}')
        check('board: small_zone: written at rc 4',
              os.path.exists(out_of('small_zone')))
        graded_invariants('small_zone', rc, s)

        # --- fresh seed: run27-shorts, the #971 review's counterexample ----
        bpath = os.path.join(tmp, 'shorts.kicad_pcb')
        with open(bpath, 'w', encoding='utf-8') as fh:
            fh.write(shorts.BOARD % ('15 18.4', '26', ''))
        _b, i = _write(tmp, 'shorts_intent', '', shorts.INTENT)
        rc, s, text = _cli(bpath, i, out_of('shorts'),
                           ['--board-edge-clearance', '0.2'])
        check('rc: shorts: the seed that shorts its own pads fails', rc == 4,
              f'rc {rc}\n{text[-900:]}')
        check('board: shorts: written at rc 4',
              os.path.exists(out_of('shorts')))
        rep = graded_invariants('shorts', rc, s)
        check('key: shorts: the body-less header is listed unmeasured',
              entries(rep, 'overhang_body') == ['J1'], f'{rep}')

        # --- fresh seed: a drawn body flush with its edge ------------------
        b, i = _write(tmp, 'flush', FLUSH % ' (locked yes)',
                      run27._intent([8, 5, 22, 15], must_lock=['J1']))
        rc, s, text = _cli(b, i, out_of('flush'), [])
        check('rc: flush: a seated, measured receptacle passes', rc == 0,
              f'rc {rc}\n{text[-900:]}')
        rep = graded_invariants('flush', rc, s)
        ev = rep.get('overhang_evidence') or [{}]
        check('key: flush: every declared requirement measured, on the body',
              rep.get('complete') is True and rep.get('unmeasured') == []
              and str(ev[0].get('overhang_basis')).startswith('body:'),
              f'{rep}')

        # --- --reseat / --repair on a placed board -------------------------
        b, i = _write(tmp, 'placed', PLACED, PLACED_INTENT)
        rc, s, text = _cli(b, i, out_of('reseat_other'), ['--reseat', 'U1'])
        check('rc: reseat_other: a band failure fails the run', rc == 4,
              f'rc {rc}\n{text[-900:]}')
        check('board: reseat_other: written at rc 4',
              os.path.exists(out_of('reseat_other')))
        rep = graded_invariants('reseat_other', rc, s)
        band = [e for e in rep.get('errors_own') or ()
                if e.get('ref') == 'J1'
                and 'overhang_mm' in (e.get('measured') or {})]
        check('key: reseat_other: the band failure is NAMED as own, with its '
              'number and limit',
              band and band[0]['measured']['overhang_mm']
              > band[0]['expected']['max'], f'{rep}')

        rc, s, text = _cli(b, i, out_of('reseat_auto'), ['--reseat'])
        check('rc: reseat_auto: exit code as before the report', rc == 4,
              f'rc {rc}\n{text[-900:]}')
        check('board: reseat_auto: written at rc 4',
              os.path.exists(out_of('reseat_auto')))
        graded_invariants('reseat_auto', rc, s)

        rc, s, text = _cli(b, i, out_of('repair'), ['--repair'])
        check('rc: repair: exit code as before the report', rc == 0,
              f'rc {rc}\n{text[-900:]}')
        check('board: repair: written', os.path.exists(out_of('repair')))
        rep = graded_invariants('repair', rc, s)
        check('key: repair: no band was dropped', rep.get('bands_dropped') == [],
              f'{rep}')

        rc, s, text = _cli(b, i, out_of('reseat_j1'), ['--reseat', 'J1'])
        check('rc: reseat_j1: exit code as before the report', rc == 0,
              f'rc {rc}\n{text[-900:]}')
        check('board: reseat_j1: written', os.path.exists(out_of('reseat_j1')))
        rep = graded_invariants('reseat_j1', rc, s)
        check('key: reseat_j1: the dropped band is listed, not graded, and '
              'the report is incomplete',
              [(d.get('ref'), d.get('band_max_mm'), d.get('graded'))
               for d in rep.get('bands_dropped') or ()] == [('J1', 0.5, False)]
              and rep.get('declared_refs') == []
              and rep.get('complete') is False, f'{rep}')
        check('key: reseat_j1: it matches the summary\'s own edge_bands_dropped',
              sorted(d['ref'] for d in rep.get('bands_dropped') or ())
              == sorted((s or {}).get('edge_bands_dropped') or {}), f'{s}')

        for arm, extra in (('reseat_j1_dry', ['--reseat', 'J1', '--dry-run']),
                           ('repair_dry', ['--repair', '--dry-run'])):
            rc, s, text = _cli(b, i, out_of(arm), extra)
            check(f'rc: {arm}: exit code as before the report', rc == 0,
                  f'rc {rc}\n{text[-900:]}')
            check(f'board: {arm}: a dry run writes nothing',
                  s is not None and not os.path.exists(out_of(arm)))
            check(f'key: {arm}: the report is exactly the ungraded form',
                  (s or {}).get(KEY) == DRY, f'{(s or {}).get(KEY)}')

        # --- ab: the exit code does not depend on the report --------------
        for arm, (bpath, ipath, extra) in (
                ('pinned', (os.path.join(tmp, 'pinned.kicad_pcb'),
                            os.path.join(tmp, 'pinned.json'), [])),
                ('reseat_other', (b, i, ['--reseat', 'U1'])),
                ('reseat_j1', (b, i, ['--reseat', 'J1']))):
            got = {}
            for mode in ('asis', 'boom'):
                # One output path for both: the summary names it.
                got[mode] = _runner(mode, bpath, ipath, out_of(f'ab_{arm}'),
                                    extra)
            (rc0, s0, d0, e0), (rc1, s1, d1, e1) = got['asis'], got['boom']
            check(f'ab: {arm}: the same exit code with the report broken',
                  rc0 == rc1 and s0 is not None and s1 is not None,
                  f'{rc0} vs {rc1}\n{e0[-600:]}\n{e1[-600:]}')
            check(f'ab: {arm}: the same board bytes', d0 is not None and d0 == d1)
            check(f'ab: {arm}: the same summary, the report aside',
                  s0 is not None and s1 is not None
                  and {k: v for k, v in s0.items() if k != KEY}
                  == {k: v for k, v in s1.items() if k != KEY})
            check(f'ab: {arm}: the broken report says it failed',
                  str(((s1 or {}).get(KEY) or {}).get('reason', '')).startswith(
                      'connector_requirements failed: RuntimeError'),
                  f'{(s1 or {}).get(KEY)}')

    if fails:
        print(f"FAIL: {len(fails)} check(s)")
        return 1
    print("PASS: every place_seed summary reports its declared connector "
          "requirements, and no exit code moved")
    return 0


if __name__ == '__main__':
    sys.exit(main())
