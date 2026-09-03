#!/usr/bin/env python3
"""Issue #846: KICAD_QFN_ONPAD_REACH, and the measurement that kept `full`.

`--allow-via-in-pad` enables a ladder of signed offsets along the escape axis.
It was called `_onpad` and documented as "on-pad (via-in-pad) offsets ...
0 == via centred on the pad", but its increment is the INTER-NET stagger -- the
centre-to-centre a via needs from a DIFFERENT net's via at this pitch -- which
on a fine-pitch part exceeds the pad. On routed_output's QFN-76 (pitch 0.40,
via 0.45, clearance 0.1) that is 0.4275 mm against a pad whose escape-axis
extent is 0.875 mm -- so rung 1 lands 0.0100 mm inside the pad EDGE, rung 2 is
already 0.8550 mm out (most of a pad-length past the far edge), and rung 8
reaches +-3.4199 mm. Measured: `pad` keeps 3 of the 17 rungs here, `barrel`
keeps 1.

The issue poses a fork -- the ladder is right and the NAME is wrong, or the name
is right and the LADDER is wrong -- and asks for a corpus A/B before anything
changes, because those long offsets are load-bearing for escape counts.

THE A/B, AS RUN (tests/sweep_846_onpad_ladder.py, upstream/main e239e067 plus
this branch's classification fix; one process per arm, --escape-method underpad
--allow-via-in-pad, grid 0.05):

    board / component      arm      escaped  failed  via_in_pad  max_stub
    tigard U3              full          32       0          48    0.3000
                           pad           32       0          48    0.3000
                           barrel        32       0          28    0.7500
    qfn_underpad_coupling  full           8       0           8    0.3375
                           pad            8       0           8    0.3375
                           barrel         6       2           4    0.7625
    qfn_diffpair_escape    full           2       0           2    0.3875
                           pad            2       0           2    0.3875
                           barrel         2       0           1    3.4625
    qfn_interior_pads      full           5       0           6    0.2625
                           pad            5       0           6    0.2625
                           barrel         4       1           4    0.7625
    routed_output U2       full          15      22           0    3.0125
                           pad           10      27           0    2.9125
                           barrel        10      27           0    2.9125

    pad:    0 improved, 1 regressed, 4 unchanged  -- NOT ADOPTABLE
    barrel: 0 improved, 3 regressed, 2 unchanged  -- NOT ADOPTABLE

    (Doctrine: improve on >= N-1 boards, regress on none. `drc_grazes.total`
    was unchanged in all 15 runs.)

SO THE FORK RESOLVES TO "THE LADDER IS RIGHT AND THE NAME WAS WRONG", and the
default stays `full`. The sharpest single row is routed_output U2: **not one of
its selected offsets overlaps a pad**, and confining the ladder to the pad still
costs it 5 escapes. The "on-pad" ladder earns its keep doing OFF-pad work --
which is precisely why the name had to change and the behaviour did not.

The knob stays so the measurement is re-runnable, and so a later board that
disagrees can be found rather than argued about.

Run: python3 tests/test_846_onpad_ladder_reach.py
"""
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

CHECKS = []


def check(name, ok, detail=''):
    CHECKS.append((name, bool(ok)))
    print(('  PASS: ' if ok else '  FAIL: ') + name
          + (' -- ' + detail if detail else ''))


PROBE = (
    'import os, sys, json, math\n'
    'R = sys.argv[1]\n'
    'sys.path[:0] = [R, os.path.join(R, "py_router"), '
    'os.path.join(R, "py_tools")]\n'
    'import env_knobs\n'
    'from qfn_fanout import axis_offset_ladder\n'
    'pad_width, via, clearance, pitch = (float(a) for a in sys.argv[2:6])\n'
    'need = via + clearance\n'
    'stagger = (math.sqrt(max(0.0, need * need - pitch * pitch)) + 0.05\n'
    '           if need > pitch else 0.0)\n'
    'step = max(stagger, 0.05, 0.05)\n'
    'axis = axis_offset_ladder(pad_width, via, step)\n'
    'print(json.dumps({"knob": env_knobs.QFN_ONPAD_REACH, "n": len(axis),\n'
    '                  "max": max(abs(d) for d in axis), "step": step}))\n'
)


def offsets_for(arm, pad_width=0.875, via=0.45, clearance=0.1, pitch=0.40):
    """The ladder the ENGINE builds, under `arm`, in a CHILD process.

    It CALLS `qfn_fanout.axis_offset_ladder` -- the engine's only source for
    these offsets -- rather than restating its arithmetic. The first draft of
    this file rebuilt the ladder from the same formula, and every knob row of
    tests/mutate_846.py SURVIVED: a test that mirrors the code it grades cannot
    detect that code changing. Only `step` is computed here, and it is a
    property of the fixture geometry, not of the thing under test.

    A child process because env_knobs reads every knob ONCE at import, so two
    arms in one process would both see whichever value was set first.
    """
    env = dict(os.environ)
    env['KICAD_QFN_ONPAD_REACH'] = arm
    p = subprocess.run([sys.executable, '-X', 'utf8', '-c', PROBE, ROOT,
                        str(pad_width), str(via), str(clearance), str(pitch)],
                       capture_output=True, text=True, env=env, cwd=ROOT)
    if p.returncode != 0:
        raise AssertionError('probe failed: ' + (p.stderr or p.stdout)[-300:])
    import json as _j
    return _j.loads(p.stdout.strip().splitlines()[-1])


ENGINE_PROBE = (
    'import os, sys, json, math, io, contextlib\n'
    'R = sys.argv[1]\n'
    'sys.path[:0] = [R, os.path.join(R, "py_router"), '
    'os.path.join(R, "py_tools")]\n'
    'from kicad_parser import parse_kicad_pcb\n'
    'from qfn_fanout import generate_qfn_fanout\n'
    'pcb = parse_kicad_pcb(os.path.join(R, "kicad_files", sys.argv[2]))\n'
    'fp = pcb.footprints[sys.argv[3]]\n'
    'buf = io.StringIO()\n'
    'with contextlib.redirect_stdout(buf):\n'
    '    tracks, vias, dropped = generate_qfn_fanout(\n'
    '        fp, pcb, net_filter=None, layer="F.Cu", track_width=0.1,\n'
    '        clearance=0.1, grid_step=0.05, escape_method="underpad",\n'
    '        via_size=0.45, via_drill=0.25, allow_via_in_pad=True)\n'
    'own = {}\n'
    'for p in fp.pads:\n'
    '    if p.net_id:\n'
    '        own.setdefault(p.net_id, []).append(p)\n'
    'worst = 0.0\n'
    'for v in vias:\n'
    '    c = own.get(v["net_id"])\n'
    '    if not c:\n'
    '        continue\n'
    '    pad = min(c, key=lambda q: math.hypot(q.global_x - v["x"],\n'
    '                                          q.global_y - v["y"]))\n'
    '    worst = max(worst, math.hypot(v["x"] - pad.global_x,\n'
    '                                  v["y"] - pad.global_y))\n'
    'print(json.dumps({"vias": len(vias), "dropped": len(dropped),\n'
    '                  "max_offset": round(worst, 4)}))\n'
)


def engine_for(arm, board='tigard', ref='U3'):
    """A REAL escape under `arm`: what the knob does to EMITTED copper."""
    env = dict(os.environ)
    env['KICAD_QFN_ONPAD_REACH'] = arm
    p = subprocess.run([sys.executable, '-X', 'utf8', '-c', ENGINE_PROBE, ROOT,
                        board + '.kicad_pcb', ref],
                       capture_output=True, text=True, env=env, cwd=ROOT)
    if p.returncode != 0:
        raise AssertionError('engine probe failed: '
                             + (p.stderr or p.stdout)[-400:])
    import json as _j
    return _j.loads(p.stdout.strip().splitlines()[-1])


def main():
    # The rig's own premise: on this geometry the step EXCEEDS half the pad, so
    # confining the ladder to the pad must actually remove rungs. Without this,
    # every check below could pass on a part where the knob does nothing.
    full = offsets_for('full')
    pad = offsets_for('pad')
    barrel = offsets_for('barrel')
    # The rig's own premise, stated as what the arms must DO rather than as an
    # inequality about the step: an earlier draft asserted `step > pad_width/2`,
    # which is FALSE on this geometry (0.4275 vs 0.4375 -- rung 1 sits 0.01 mm
    # INSIDE the pad edge) and would have failed a rig that works perfectly.
    check('the arms actually shorten the ladder, so these checks are not '
          'vacuous',
          full['n'] > pad['n'] > barrel['n'],
          f"full={full['n']} pad={pad['n']} barrel={barrel['n']} "
          f"step={full['step']:.4f}")

    check("'full' is 17 rungs reaching 8*step", full['n'] == 17
          and abs(full['max'] - 8 * full['step']) < 1e-9,
          f"n={full['n']} max={full['max']:.4f}")

    check("'pad' keeps only offsets whose via CENTRE stays on the pad",
          pad['max'] <= 0.875 / 2.0 + 1e-9 and pad['n'] < full['n'],
          f"n={pad['n']} max={pad['max']:.4f}")

    check("'barrel' keeps only offsets whose whole BARREL stays on the pad",
          barrel['max'] <= 0.875 / 2.0 - 0.45 / 2.0 + 1e-9,
          f"n={barrel['n']} max={barrel['max']:.4f}")
    check("'barrel' collapses this geometry to the centred rung alone",
          barrel['n'] == 1 and barrel['max'] == 0.0,
          'the two arms are not interchangeable, which is why both are named')

    # A typo must not silently shorten the ladder. This is the property
    # KICAD_QFN_UNDERPAD_ERASED_GATE states for itself, and the reason the knob
    # is a string rather than a bool.
    typo = offsets_for('PADD')
    check('an unrecognised value behaves as `full`',
          typo['n'] == full['n'] and abs(typo['max'] - full['max']) < 1e-9,
          f"n={typo['n']}")

    # Case-insensitive and whitespace-tolerant, per env_knobs' own parse.
    loud = offsets_for('  PAD ')
    check('the knob is case- and whitespace-insensitive',
          loud['n'] == pad['n'], f"n={loud['n']} vs pad {pad['n']}")

    # And end to end, on real copper: the ladder is what the escape actually
    # uses, so the knob must move the EMITTED vias, not just a list. tigard U3
    # is 0.5mm-pitch with 0.80 x 0.25 leads.
    e_full = engine_for('full')
    e_barrel = engine_for('barrel')
    check('the knob reaches emitted copper, not just the offset list',
          e_barrel['max_offset'] != e_full['max_offset'],
          f"full max_offset={e_full['max_offset']} vias={e_full['vias']} "
          f"vs barrel max_offset={e_barrel['max_offset']} "
          f"vias={e_barrel['vias']}")
    # MEASURED, and the opposite of the intuition that shortening a ladder
    # shortens stubs: on tigard U3 confining the axis ladder pushes the worst
    # offset from 0.3000 to 0.7500, because the escape then falls through to
    # the OUTWARD ladder, whose first rung is already past the pad edge
    # (base = pad_width/2 + via/2 + clearance = 0.725). Confining the "on-pad"
    # ladder does not produce more on-pad vias -- it produces further-out ones.
    check('confining the axis ladder LENGTHENS stubs (the fallback is the '
          'outward ladder)',
          e_barrel['max_offset'] > e_full['max_offset'],
          f"full={e_full['max_offset']} barrel={e_barrel['max_offset']}")
    check('`full` is the shipped default, and it is the arm the A/B kept',
          engine_for('full') == engine_for('nonsense-value'),
          'an unrecognised value no longer behaves as `full` END TO END')

    bad = [n for n, ok in CHECKS if not ok]
    print(f"\n{'FAIL' if bad else 'PASS'}  #846 ladder reach: "
          f"{len(CHECKS) - len(bad)}/{len(CHECKS)} checks")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
