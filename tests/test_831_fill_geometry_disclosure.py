#!/usr/bin/env python3
"""#831: which copper the plane-fragility field was priced on is DISCLOSED,
and a fallback that a faster machine would not have taken says so.

`plane_fragility` prices the exact KiCad fill when pcbnew finishes within
`EXACT_FILL_TIMEOUT`, else the drawn zone outlines -- geometry the module
docstring calls "near-useless" for a full-board pour (measured below: on
sonde_u the outlines yield a fraction of the exact fill's cells). Until
this change the choice was visible only as a console line, so nothing that
grades a run could tell a board priced on outlines because it HAS no fill
from one priced on outlines because pcbnew was slow HERE.

Every arm asserts the RECORD (`source`, `refill_status`, `machine_dependent`),
never merely that some notice was printed. The negative control is the
`no_kicad_python` arm: a fallback that is a fact about the install must NOT
be marked machine-dependent, or the flag means nothing.

Arms are driven by monkeypatching the specific failure rather than by a slow
board (a timeout arm that waits 300 s would be a machine-speed test of a
change whose point is disclosing machine speed).
"""
import ast
import contextlib
import io
import os
import subprocess
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

import kicad_exact_fill as kef                      # noqa: E402
import plane_fragility as pf                        # noqa: E402
from kicad_exact_fill import RefillStatus           # noqa: E402
from kicad_parser import parse_kicad_pcb            # noqa: E402

#: A board WITH zones (splitflap_driver has none; see test_713_refill_status).
ZONED = os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}"
          f"{(' -- ' + str(detail)) if detail else ''}")


class _Reset:
    def __enter__(self):
        self._py = list(kef._KICAD_PYTHON_MEMO)
        self._failed = set(kef._REFILL_FAILED)
        self._memo = dict(kef._REFILL_MEMO)
        self._run = subprocess.run
        self._refill = kef.refill_islands_ex
        kef._KICAD_PYTHON_MEMO.clear()
        kef._REFILL_FAILED.clear()
        kef._REFILL_MEMO.clear()
        return self

    def __exit__(self, *a):
        kef._KICAD_PYTHON_MEMO[:] = self._py
        kef._REFILL_FAILED.clear()
        kef._REFILL_FAILED.update(self._failed)
        kef._REFILL_MEMO.clear()
        kef._REFILL_MEMO.update(self._memo)
        subprocess.run = self._run
        kef.refill_islands_ex = self._refill
        return False


def _cfg(pcb):
    return SimpleNamespace(grid_step=0.5,
                           layers=list(pcb.board_info.copper_layers or
                                       ['F.Cu', 'B.Cu']),
                           cell_cost=lambda mm: int(mm * 1000))


def _run(setup, provider=None):
    """(n_cells, geometry, printed) under a forced refill outcome."""
    with _Reset():
        setup()
        pcb = parse_kicad_pcb(ZONED)
        pcb.exact_fill_provider = provider
        pcb.source_path = ZONED
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            cells, geo = pf.compute_plane_fragility_cells_ex(pcb, _cfg(pcb))
        return len(cells), geo, buf.getvalue()


def _timeout():
    kef._KICAD_PYTHON_MEMO.append(sys.executable)

    def _boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd='refill', timeout=300)
    subprocess.run = _boom


print("--- the record's vocabulary")
check("every source the record can name is declared",
      set(pf.GEOMETRY_SOURCES) == {'live_board', 'live_board_in_process',
                                   'kicad_refill', 'zone_outlines', 'none'},
      pf.GEOMETRY_SOURCES)
try:
    pf._geometry('outline')
    check("an undeclared source is refused", False)
except AssertionError:
    check("an undeclared source is refused", True)

print("\n--- a TIMEOUT fallback is disclosed as machine-dependent")
n_to, geo, out = _run(_timeout)
check("geometry source is the zone outlines", geo['source'] == 'zone_outlines',
      geo)
check("the refill status is carried, not re-derived",
      geo['refill_status'] == 'timeout', geo)
check("and it is marked MACHINE-dependent", geo['machine_dependent'] is True,
      geo)
check("the console says so, naming machine speed and the issue",
      'machine speed' in out and '#831' in out and 'OUTLINES' in out,
      out.strip()[-200:])
check("the outlines still produced a field (the fallback is real, not empty)",
      n_to > 0, n_to)

print("\n--- NEGATIVE CONTROL: a board/install fact is NOT machine-dependent")
n_np, geo, out = _run(lambda: kef._KICAD_PYTHON_MEMO.append(None))
check("no-pcbnew fallback is the zone outlines too",
      geo['source'] == 'zone_outlines' and geo['refill_status'] == 'no_kicad_python',
      geo)
check("but NOT machine-dependent", geo['machine_dependent'] is False, geo)
check("and the machine-speed WARNING does not fire for it",
      'machine speed' not in out, out.strip()[-200:])
check("the two fallbacks rasterize the same outline field",
      n_np == n_to, f"{n_np} vs {n_to}")


def _refill_failed():
    kef._KICAD_PYTHON_MEMO.append(sys.executable)

    class _R:
        returncode = 1
        stdout = ''
        stderr = 'boom'
    subprocess.run = lambda *a, **k: _R()


n_rf, geo, out = _run(_refill_failed)
check("a refill FAILURE is a board fact: outlines, not machine-dependent",
      geo['source'] == 'zone_outlines' and geo['refill_status'] == 'refill_failed'
      and geo['machine_dependent'] is False, geo)


def _empty_fill():
    kef._KICAD_PYTHON_MEMO.append(sys.executable)
    kef.refill_islands_ex = lambda *a, **k: (
        {}, RefillStatus('ok', 'the refill ran and produced no islands', 1.0))


n_ef, geo, out = _run(_empty_fill)
check("an EMPTY fill falls back to outlines with status ok, not machine-dependent",
      geo['source'] == 'zone_outlines' and geo['refill_status'] == 'ok'
      and geo['machine_dependent'] is False and 'no islands' in geo['why'],
      geo)

print("\n--- a successful refill is reported as the refill")


def _stub_fill():
    kef._KICAD_PYTHON_MEMO.append(sys.executable)
    kef.refill_islands_ex = lambda *a, **k: (
        {('GND', 'B.Cu'): [[(10.0, 10.0), (60.0, 10.0), (60.0, 40.0),
                             (10.0, 40.0)]]},
        RefillStatus('ok', '', 1.0))


n_sf, geo, out = _run(_stub_fill)
check("source is kicad_refill with the island count",
      geo['source'] == 'kicad_refill' and geo['islands'] == 1
      and geo['machine_dependent'] is False, geo)

print("\n--- the GUI provider's account is read, not guessed")


def _provider_factory(account):
    def _p():
        return {('GND', 'B.Cu'): [[(10.0, 10.0), (60.0, 10.0), (60.0, 40.0),
                                    (10.0, 40.0)]]}
    if account is not None:
        _p.last_status = account
    return _p


acct = {'source': 'live_board_in_process', 'refill_status': 'timeout',
        'why': 'the KiCad refill TIMED OUT after 300s', 'machine_dependent': True}
n_p, geo, out = _run(lambda: None, provider=_provider_factory(acct))
check("an in-process fallback the provider reports is not called the live board",
      geo['source'] == 'live_board_in_process', geo)
check("its machine-dependence is carried through",
      geo['machine_dependent'] is True and geo['refill_status'] == 'timeout', geo)
n_p2, geo, out = _run(lambda: None, provider=_provider_factory(None))
check("a provider with no account is the live board, with no claim attached",
      geo['source'] == 'live_board' and geo['machine_dependent'] is False
      and geo['refill_status'] is None, geo)

print("\n--- register_plane_fragility publishes the record on config")
with _Reset():
    _timeout()
    pcb = parse_kicad_pcb(ZONED)
    pcb.exact_fill_provider = None
    pcb.source_path = ZONED
    cfg = _cfg(pcb)
    cfg.clearance = 0.2
    cache = {}
    os.environ['KICAD_PLANE_FRAGILITY_DYNAMIC'] = '0'
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            pf.register_plane_fragility(pcb, cfg, cache)
    finally:
        del os.environ['KICAD_PLANE_FRAGILITY_DYNAMIC']
    rec = getattr(cfg, '_plane_fragility_geometry', None)
    check("config._plane_fragility_geometry is set",
          isinstance(rec, dict) and rec['source'] == 'zone_outlines'
          and rec['machine_dependent'] is True, rec)

with _Reset():
    pcb = parse_kicad_pcb(ZONED)
    pcb.zones = []
    cfg = _cfg(pcb)
    with contextlib.redirect_stdout(io.StringIO()):
        pf.register_plane_fragility(pcb, cfg, {})
    rec = getattr(cfg, '_plane_fragility_geometry', None)
    check("set even when there is nothing to price (source 'none', why given)",
          isinstance(rec, dict) and rec['source'] == 'none' and rec['why'],
          rec)

print("\n--- the record reaches batch_route's JSON summary (AST, not grep)")
with open(os.path.join(ROOT, 'py_router', 'route.py'), encoding='utf-8') as f:
    _tree = ast.parse(f.read())


def _func(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


_br = _func(_tree, 'batch_route')
_hit = False
for node in ast.walk(_br):
    if (isinstance(node, ast.Assign) and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Subscript)
            and isinstance(node.targets[0].value, ast.Name)
            and node.targets[0].value.id == 'summary'
            and isinstance(node.targets[0].slice, ast.Constant)
            and node.targets[0].slice.value == 'plane_fragility'):
        _src = ast.unparse(node.value)
        _hit = '_pfg' in _src or '_plane_fragility_geometry' in _src
check("batch_route assigns summary['plane_fragility'] from the config record",
      _hit)

print("\n--- the GUI provider leaves an account on every path (AST)")
with open(os.path.join(ROOT, 'py_router', 'kicad_parser.py'),
          encoding='utf-8') as f:
    _kp = ast.parse(f.read())
_lf = _func(_kp, '_live_fill')
_try = next((n for n in ast.walk(_lf) if isinstance(n, ast.Try)), None)
_in_finally = any(
    isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Attribute)
    and n.targets[0].attr == 'last_status'
    for st in (_try.finalbody if _try else []) for n in ast.walk(st))
check("_live_fill publishes last_status in its finally (covers every return)",
      _in_finally)
_returns = [n for n in ast.walk(_lf) if isinstance(n, ast.Return)]
check("_live_fill has the return paths the account must cover (>= 4)",
      len(_returns) >= 4, len(_returns))

print("\n--- the fallback's fidelity, measured (the docstring's claim)")
# Optional: needs pcbnew. Reported as not-run rather than passed when absent.
with _Reset():
    if kef.find_kicad_python():
        pcb = parse_kicad_pcb(ZONED)
        pcb.exact_fill_provider = None
        pcb.source_path = ZONED
        with contextlib.redirect_stdout(io.StringIO()):
            cells, geo = pf.compute_plane_fragility_cells_ex(pcb, _cfg(pcb))
        if geo['source'] == 'kicad_refill':
            check("the exact fill prices MORE cells than the outline fallback",
                  len(cells) > n_to, f"exact {len(cells)} vs outlines {n_to}")
        else:
            print(f"  (not run: real refill gave {geo})")
    else:
        print("  (not run: no pcbnew on this machine)")

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
