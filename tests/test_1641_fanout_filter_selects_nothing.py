#!/usr/bin/env python3
"""#1641: a fanout --nets filter that selects no pad says so, and a net's own
name selects it.

`qfn_fanout.py --nets X` used to end with `"requested": 0, "failed": 0`,
exit 0, whenever the filter selected no pad -- an interior pad the stub
escape skips, a net whose name holds `[n]` (fnmatch reads `[3]` as a class,
so 'D[3]' did not match itself), or a net that does not exist -- which reads
the same as a part with nothing to do. bga_fanout shared the matcher and the
ledger.

Checks:
  1. The matcher: a pattern equal to a net's name (or its last sheet
     component) matches it; with the board's names such a pattern is
     literal ('D[3]' does not select 'D3'); literal_net_patterns rewrites a
     filter so fnmatch reads it the same way; expand_net_patterns too.
  2. qfn_fanout.py on qfn_interior_pads: --nets GND (three interior pads,
     stub mode) lists GND in `unmatched` as `interior`, naming the pads;
     --nets 'D[3]' (a perimeter net renamed) fans it out; --nets NO_SUCH_NET
     is refused with exit 2 and writes no board; a list with one real and
     one missing net runs and lists both misses.
  3. The engine both fronts call: LAST_FILTER_MISSES carries the interior
     miss in stub mode and is empty under underpad, which escapes the pads.
  4. bga_fanout.py on a BGA: a missing net is refused with exit 2, and a
     ball net renamed with a bus index fans out by its name.

    python3 tests/test_1641_fanout_filter_selects_nothing.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence             # noqa: E402

QFN = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
BGA = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
BGA_NET = '/IO_Banks/U5'                 # one ball of U30
BGA_ARGS = ['--component', 'U30', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu',
            'B.Cu', '--via-size', '0.25', '--via-drill', '0.15']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def summary(out):
    m = re.search(r'^JSON_SUMMARY: (.*)$', out, re.M)
    return json.loads(m.group(1)) if m else None


def renamed(src, dst, old, new):
    """SRC with net OLD renamed NEW, its siblings carried."""
    with open(src, encoding='utf-8') as f:
        text = f.read()
    assert f'"{old}"' in text, old
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(text.replace(f'"{old}"', f'"{new}"'))
    pro = src[:-len('.kicad_pcb')] + '.kicad_pro'
    if os.path.isfile(pro):
        shutil.copyfile(pro, dst[:-len('.kicad_pcb')] + '.kicad_pro')
    return evidence(dst)


def matcher():
    print("1. the matcher")
    from net_queries import (net_pattern_matches, matches_net_filter,
                             literal_net_patterns, expand_net_patterns)
    from kicad_parser import parse_kicad_pcb
    known = frozenset({'D[3]', 'D3', '/bus/A[1]', 'GND'})
    check("'D[3]' matches itself", net_pattern_matches('D[3]', 'D[3]'))
    check("'A[1]' matches '/bus/A[1]' by its last component",
          net_pattern_matches('/bus/A[1]', 'A[1]'))
    check("with the board's names 'D[3]' does not select 'D3'",
          not net_pattern_matches('D3', 'D[3]', known))
    check("a class pattern naming no net still globs",
          net_pattern_matches('D3', 'D[0-9]', known))
    lit = literal_net_patterns(['D[3]', '!A[1]', 'D[0-9]', '*'], known)
    check("literal_net_patterns escapes only names of board nets",
          lit == ['D[[]3]', '!A[[]1]', 'D[0-9]', '*'], repr(lit))
    check("the rewritten filter selects 'D[3]' and not 'D3'",
          matches_net_filter('D[3]', lit[:1]) and not matches_net_filter('D3', lit[:1]))
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(QFN)
        got = expand_net_patterns(pcb, ['GND'])
    check("expand_net_patterns still resolves a plain name", got == ['GND'], repr(got))
    from types import SimpleNamespace as N
    both = N(nets={1: N(name='D3'), 2: N(name='D[3]')}, pads_by_net={}, footprints={})
    got = [expand_net_patterns(both, p, quiet=True)
           for p in (['D[3]'], ['*', '!D[3]'], ['D[0-9]'])]
    check("expand_net_patterns with both 'D3' and 'D[3]': the name is literal, "
          "a class still globs", got == [['D[3]'], ['D3'], ['D3']], repr(got))


def qfn_cli(tmp):
    print("2. qfn_fanout.py")
    b = os.path.join(tmp, 'b.kicad_pcb')
    shutil.copyfile(QFN, b)
    br = renamed(QFN, os.path.join(tmp, 'br.kicad_pcb'), 'FOREIGN', 'D[3]')
    base = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'qfn_fanout.py')]

    r = run_check(base + [b, '--component', 'U1', '--output',
                          os.path.join(tmp, 'a.kicad_pcb'), '--nets', 'GND'], accept=True)
    s = summary(r.stdout)
    miss = (s or {}).get('unmatched') or []
    check("--nets GND: GND listed as interior, with its three pads",
          len(miss) == 1 and miss[0]['reason'] == 'interior'
          and miss[0]['pads'] == ['U1.33', 'U1.36', 'U1.37'], repr(miss))
    check("--nets GND: the console names underpad",
          '--escape-method underpad reaches them' in r.stdout)

    r = run_check(base + [br, '--component', 'U1', '--output',
                          os.path.join(tmp, 'c.kicad_pcb'), '--nets', 'D[3]'], accept=True)
    s = summary(r.stdout) or {}
    check("--nets 'D[3]' fans out the net of that name",
          s.get('requested') == 1 and s.get('escaped') == 1 and s.get('unmatched') == [],
          f"requested={s.get('requested')} escaped={s.get('escaped')}")

    out = os.path.join(tmp, 'd.kicad_pcb')
    run_check(base + [b, '--component', 'U1', '--output', out, '--nets', 'NO_SUCH_NET'],
              refuse='names a net on this board', code=2)
    check("--nets NO_SUCH_NET: refused, no board written", not os.path.exists(out))

    r = run_check(base + [b, '--component', 'U1', '--output',
                          os.path.join(tmp, 'g.kicad_pcb'), '--nets', 'GND', 'NO_SUCH_NET'],
                  accept=True)
    reasons = [(m['pattern'], m['reason']) for m in (summary(r.stdout) or {}).get('unmatched', [])]
    check("one real, one missing net: runs, both misses listed",
          reasons == [('GND', 'interior'), ('NO_SUCH_NET', 'no_such_net')], repr(reasons))


def engine():
    print("3. the engine (the fanout tab's call)")
    import qfn_fanout
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(QFN)
    kw = dict(net_filter=['GND'], layer='F.Cu', track_width=0.1, clearance=0.1,
              grid_step=0.05, via_size=0.45, via_drill=0.25, allow_via_in_pad=True)
    with contextlib.redirect_stdout(io.StringIO()):
        qfn_fanout.generate_qfn_fanout(pcb.footprints['U1'], pcb, escape_method='stub', **kw)
    stub = [m['reason'] for m in qfn_fanout.LAST_FILTER_MISSES]
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(QFN)
        qfn_fanout.generate_qfn_fanout(pcb.footprints['U1'], pcb, escape_method='underpad', **kw)
    under = list(qfn_fanout.LAST_FILTER_MISSES)
    check("stub: LAST_FILTER_MISSES = [interior]", stub == ['interior'], repr(stub))
    check("underpad: no miss", under == [], repr(under))


def bga_cli(tmp):
    print("4. bga_fanout.py")
    base = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'bga_fanout.py')]
    b = os.path.join(tmp, 'g.kicad_pcb')
    shutil.copyfile(BGA, b)
    out = os.path.join(tmp, 'g_missing.kicad_pcb')
    run_check(base + [b] + BGA_ARGS + ['--output', out, '--nets', 'NO_SUCH_NET'],
              refuse='names a net on this board', code=2)
    check("missing net: refused, no board written", not os.path.exists(out))
    br = renamed(BGA, os.path.join(tmp, 'gr.kicad_pcb'), BGA_NET, '/IO_Banks/U[5]')
    r = run_check(base + [br] + BGA_ARGS + ['--output', os.path.join(tmp, 'gr_out.kicad_pcb'),
                                            '--nets', '/IO_Banks/U[5]'], accept=True)
    s = summary(r.stdout) or {}
    check("a ball net named with a bus index fans out by its name",
          s.get('requested') == 1 and s.get('escaped') == 1 and s.get('unmatched') == [],
          f"requested={s.get('requested')} escaped={s.get('escaped')}")


def main():
    tmp = tempfile.mkdtemp(prefix='t1641_')
    try:
        matcher()
        qfn_cli(tmp)
        engine()
        bga_cli(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
