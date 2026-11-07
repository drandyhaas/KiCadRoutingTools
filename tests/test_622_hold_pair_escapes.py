#!/usr/bin/env python3
"""A ball of a pair the comb refuses holds the escape the board came with only WITH its partner's -- the two together
or neither (awx/route_bus.py: hold_board_escapes).

  python3 tests/test_622_hold_pair_escapes.py

On a shared field a whole-array bus's comb fans its balls again round the other buses' escapes, and a ball it refuses
holds the escape the field gave it, where every piece of it stands clear of the copper there. Held alone, one leg of a
memory bus's strobe pair kept its escape on an inner layer while the comb fanned the other again on the surface, onto
the held leg's tip: the pair's two tips stood one above the other, and no round found it an end connector. On a
generated four-copper bench of four pairs (synth_bus, its source fanned by make_bench) -- the fanned board as the board
the escapes came with, the raw board as the copper standing:

1. LIVENESS: each leg of SYP0 has an escape at SU1 on the fanned board; else the checks test nothing (BROKEN TEST);
2. a refused P leg holds with its partner: both held;
3. a refused leg whose partner is not among the bus's nets holds alone, as a single does;
4. a partner whose escape does not stand clear (another net's track over it): neither leg held -- though the other
   leg's own escape stands clear, held alone where its partner is no bus net -- while another pair refused with it
   holds both of its legs.

About a second.
"""
import contextlib
import io
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
sys.path.insert(0, AWX)
sys.path.insert(1, os.path.join(ROOT, 'py_router'))
BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


with tempfile.TemporaryDirectory() as td:
    raw, bench = os.path.join(td, 'raw.kicad_pcb'), os.path.join(td, 'bench.kicad_pcb')
    env = dict(os.environ, ROUTE_LAYERS='F.Cu,B.Cu,In1.Cu,In2.Cu')
    synth = ['synth_bus.py', raw, '--k', '8', '--gap', '2.5', '--pattern', 'sorted', '--pairs', '4', '--copper', '4']
    for cmd in (synth, ['make_bench.py', raw, 'SU1', 'SD1', bench]):
        r = subprocess.run([sys.executable] + cmd, cwd=AWX, capture_output=True, text=True, timeout=600, env=env)
        if r.returncode != 0:
            print(f'BROKEN TEST: {cmd[0]} failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
            sys.exit(2)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        import route_bus as rb
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(bench)
    names = sorted(n.name for n in pcb.nets.values() if n.name and n.name.split('/')[-1].startswith('SYP'))
    txt = open(bench, encoding='utf-8').read()
    legs = {nm: rb.escape_blocks(txt, pcb, 'SU1', [nm]) for nm in ('SYP0_P', 'SYP0_N')}

    # 1. liveness
    if not all(legs.values()) or len(names) != 8:
        print(f'BROKEN TEST: the bench\'s pairs or SYP0\'s escapes at SU1 are not there ({names}; '
              f'{ {k: len(v) for k, v in legs.items()} })')
        sys.exit(2)
    check(True, f'liveness: SYP0\'s legs escape at SU1 ({ {k: len(v) for k, v in legs.items()} } pieces)')

    def hold(base, failed, names_):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return sorted(rb.hold_board_escapes(bench, base, os.path.join(td, 'held.kicad_pcb'), pcb, 'SU1', failed,
                                                names_))

    # 2. a refused leg holds with its partner
    got = hold(raw, ['SYP0_P'], names)
    check(got == ['SYP0_N', 'SYP0_P'], f'a refused P leg holds with its partner ({got})')
    # 3. a leg whose partner is not a bus net holds alone
    got = hold(raw, ['SYP0_P'], [n for n in names if n != 'SYP0_N'])
    check(got == ['SYP0_P'], f'a refused leg whose partner is not among the bus\'s nets holds alone ({got})')
    # 4. the partner's escape not clear: another net's track laid over its first piece
    seg = next(b for b in legs['SYP0_N'] if b.lstrip().startswith('(segment'))
    other = next(n for n in pcb.nets.values() if n.name == 'SYP3_P')
    if re.search(r'\(net "', seg):
        seg = re.sub(r'\(net "[^"]*"\)', '(net "SYP3_P")', seg)
    else:
        seg = re.sub(r'\(net \d+\)', f'(net {other.net_id})', seg)
    seg = re.sub(r'\(uuid "[^"]*"\)', '', seg)
    rtxt = open(raw, encoding='utf-8').read()
    end = rtxt.rstrip().rfind(')')
    blocked = os.path.join(td, 'blocked.kicad_pcb')
    with open(blocked, 'w', encoding='utf-8') as f:
        f.write(rtxt[:end].rstrip() + '\n  ' + seg + '\n' + rtxt[end:])
    alone = hold(blocked, ['SYP0_P'], [n for n in names if n != 'SYP0_N'])
    got = hold(blocked, ['SYP0_P', 'SYP1_P'], names)
    check(alone == ['SYP0_P'] and got == ['SYP1_N', 'SYP1_P'],
          f'a partner whose escape is not clear: neither leg held, though the P leg\'s own escape stands clear '
          f'({alone} alone), the other pair refused with it held whole ({got})')

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)
