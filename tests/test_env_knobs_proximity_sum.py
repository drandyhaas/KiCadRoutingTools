#!/usr/bin/env python3
"""KICAD_PROXIMITY_SUM selects a proximity composition mode by every documented spelling, and a value no mode
reads is said, not silently taken as MAX (py_router/env_knobs.py).

    python3 tests/test_env_knobs_proximity_sum.py

`sum` is the spelling docs/api-routing-config.md gives; the parse used to accept only 1/true/on for it, so
KICAD_PROXIMITY_SUM=sum measured the default (MAX) while the run's ENV KNOBS line listed the variable as set.
"""
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import env_knobs  # noqa: E402

fails = []


def mode(value):
    """(PROXIMITY_SUM_MODE, PROXIMITY_SUM, stderr) under KICAD_PROXIMITY_SUM=value (None: unset)"""
    saved = os.environ.get('KICAD_PROXIMITY_SUM')
    try:
        if value is None:
            os.environ.pop('KICAD_PROXIMITY_SUM', None)
        else:
            os.environ['KICAD_PROXIMITY_SUM'] = value
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            env_knobs.refresh()
        return env_knobs.PROXIMITY_SUM_MODE, env_knobs.PROXIMITY_SUM, err.getvalue()
    finally:
        if saved is None:
            os.environ.pop('KICAD_PROXIMITY_SUM', None)
        else:
            os.environ['KICAD_PROXIMITY_SUM'] = saved
        env_knobs.refresh()


def check(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}" + (f': {detail}' if detail and not cond else ''))
    if not cond:
        fails.append(label)


for v in ('sum', 'SUM', ' Sum ', '1', 'true', 'on'):
    m, on, err = mode(v)
    check(f'{v!r} selects sum', (m, on) == ('sum', True) and not err, f'{m!r} {on} {err!r}')
for v in ('zoned', 'softcap'):
    m, on, err = mode(v)
    check(f'{v!r} selects {v}', (m, on) == (v, True) and not err, f'{m!r} {on} {err!r}')
for v in (None, '', '0', 'off', 'false'):
    m, on, err = mode(v)
    check(f'{v!r} is max, said nothing', (m, on) == ('', False) and not err, f'{m!r} {on} {err!r}')
m, on, err = mode('bogus')
check("'bogus' is max, and is said", (m, on) == ('', False) and 'KICAD_PROXIMITY_SUM' in err and 'bogus' in err,
      f'{m!r} {on} {err!r}')
m, on, err = mode('bogus')
check("...once only", not err, repr(err))

if fails:
    print(f'FAIL: {len(fails)} check(s)')
    sys.exit(1)
print('PASS: KICAD_PROXIMITY_SUM reads every documented spelling, and says a value no mode reads')
