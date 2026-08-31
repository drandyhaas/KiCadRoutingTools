#!/usr/bin/env python3
"""#713 item 4: the exact fill says WHICH thing went wrong.

Five distinct causes used to return one bare `None`, so no caller could tell
"this machine has no pcbnew" from "this board did not finish in 300 s". Exactly
one of the five, `timeout`, is a fact about the MACHINE rather than the board --
and `plane_fragility` reported that one to the user as "the KiCad refill
failed", which sends the reader to the wrong repair and is the item.

Every arm asserts the REASON, never merely that the result was None: a check
that only asserts `is None` passes for all five causes at once and would have
stayed green through the whole defect.

Arms are driven by monkeypatching the SPECIFIC failure rather than by a slow
board, because a timeout arm that waits 300 s is a machine-speed test of a fix
whose entire purpose is to remove machine speed from the answer.
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))

import kicad_exact_fill as kef          # noqa: E402
from kicad_exact_fill import RefillStatus, refill_islands_ex  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
#: plane_fragility returns before touching the refill when a board has no
#: zones (`not pcb_data.zones` at the top of compute_plane_fragility_cells),
#: and splitflap_driver has none -- pointing the fallback arms at it made
#: them assert about output the code never produced. Measured: 0 zones.
ZONED = os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


class _Reset:
    """Every arm starts from a clean module state, or arm N inherits arm N-1's
    memo and measures the wrong thing."""

    def __enter__(self):
        self._py = list(kef._KICAD_PYTHON_MEMO)
        self._failed = set(kef._REFILL_FAILED)
        self._memo = dict(kef._REFILL_MEMO)
        self._run = subprocess.run
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
        return False


print("--- the closed vocabulary")
check("REFILL_REASONS is closed and names every arm below",
      set(kef.REFILL_REASONS) == {'ok', 'no_kicad_python', 'memoised_failure',
                                  'refill_failed', 'timeout', 'error'},
      str(kef.REFILL_REASONS))
check("only `ok` reports ok", [r for r in kef.REFILL_REASONS
                               if RefillStatus(r).ok] == ['ok'])
check("only `timeout` reports is_timeout",
      [r for r in kef.REFILL_REASONS if RefillStatus(r).is_timeout]
      == ['timeout'])
check("every reason has a why() that is not the bare token",
      all(RefillStatus(r).why() not in ('', r) for r in kef.REFILL_REASONS),
      str({r: RefillStatus(r).why() for r in kef.REFILL_REASONS}))

print("\n--- arm: no_kicad_python")
with _Reset():
    kef._KICAD_PYTHON_MEMO.append(None)
    isl, st = refill_islands_ex(BOARD)
    check("islands is None", isl is None)
    check("reason is no_kicad_python", st.reason == 'no_kicad_python', st.reason)
    check("why() names the env var the reader must set",
          'KICAD_PYTHON' in st.why(), st.why())
    check("not reported as a timeout", not st.is_timeout)

print("\n--- arm: memoised_failure")
with _Reset():
    kef._KICAD_PYTHON_MEMO.append('/nonexistent/python')
    key = kef._refill_memo_key(BOARD)
    check("the board is keyable at all", key is not None)
    kef._REFILL_FAILED.add(key)
    isl, st = refill_islands_ex(BOARD)
    check("islands is None", isl is None)
    check("reason is memoised_failure", st.reason == 'memoised_failure',
          st.reason)
    check("why() says it is a REMEMBERED no, not a fresh one",
          'already failed' in st.why(), st.why())

print("\n--- arm: refill_failed (subprocess ran, produced no REFILL_OK)")
with _Reset():
    kef._KICAD_PYTHON_MEMO.append(sys.executable)

    class _R:
        returncode = 3
        stdout = ''
        stderr = 'ZONE_FILLER exploded'

    subprocess.run = lambda *a, **k: _R()
    isl, st = refill_islands_ex(BOARD)
    check("islands is None", isl is None)
    check("reason is refill_failed", st.reason == 'refill_failed', st.reason)
    check("detail carries the child's rc and stderr",
          'rc=3' in st.detail and 'exploded' in st.detail, st.detail)
    check("a genuine failure IS memoised (it is a fact about these bytes)",
          kef._refill_memo_key(BOARD) in kef._REFILL_FAILED)

print("\n--- arm: timeout")
with _Reset():
    kef._KICAD_PYTHON_MEMO.append(sys.executable)

    def _boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd='refill', timeout=300)

    subprocess.run = _boom
    isl, st = refill_islands_ex(BOARD, timeout=300)
    check("islands is None", isl is None)
    check("reason is timeout", st.reason == 'timeout', st.reason)
    check("is_timeout is the ONLY arm flagged machine-dependent", st.is_timeout)
    check("elapsed_s is recorded", isinstance(st.elapsed_s, float))
    check("detail names the limit that was hit", '300' in st.detail, st.detail)
    # THE regression guard for the misattribution this item is about.
    check("why() does NOT claim the refill failed",
          'refill failed' not in st.why().lower(), st.why())
    check("why() says TIMED OUT", 'TIMED OUT' in st.why(), st.why())
    # The documented policy, which predates #713 and must survive it.
    check("a timeout is NOT memoised as a failure",
          kef._refill_memo_key(BOARD) not in kef._REFILL_FAILED,
          "a later step may legitimately have more headroom")

print("\n--- arm: error (the refill could not be attempted)")
with _Reset():
    kef._KICAD_PYTHON_MEMO.append(sys.executable)
    _copy = kef.shutil.copyfile

    def _nope(*a, **k):
        raise OSError(28, 'No space left on device')

    kef.shutil.copyfile = _nope
    try:
        isl, st = refill_islands_ex(BOARD)
    finally:
        kef.shutil.copyfile = _copy
    check("islands is None", isl is None)
    check("reason is error", st.reason == 'error', st.reason)
    check("detail names the exception type", 'OSError' in st.detail, st.detail)
    check("not reported as a timeout", not st.is_timeout)

print("\n--- the wrapper keeps its contract")
with _Reset():
    kef._KICAD_PYTHON_MEMO.append(None)
    check("refill_islands returns the islands alone, not the pair",
          kef.refill_islands(BOARD) is None)
import inspect  # noqa: E402
_sig = inspect.signature(kef.refill_islands)
check("refill_islands' parameter names are unchanged for its 6 call sites",
      list(_sig.parameters) == ['board_file', 'timeout', 'verbose',
                                'project_from'],
      str(list(_sig.parameters)))

print("\n--- plane_fragility reports the TRUE cause (the item's headline)")
import plane_fragility as pf  # noqa: E402


def _fragility_notice(reason_setup):
    """Run the fallback path with a forced cause; return what it printed."""
    import io
    import contextlib
    from kicad_parser import parse_kicad_pcb
    from types import SimpleNamespace
    with _Reset():
        reason_setup()
        pcb = parse_kicad_pcb(ZONED)
        pcb.exact_fill_provider = None
        pcb.source_path = ZONED
        cfg = SimpleNamespace(grid_step=0.5,
                              layers=list(pcb.board_info.copper_layers or
                                          ['F.Cu', 'B.Cu']),
                              cell_cost=lambda mm: int(mm * 1000))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            pf.compute_plane_fragility_cells(pcb, cfg)
        return buf.getvalue()


def _force_timeout():
    kef._KICAD_PYTHON_MEMO.append(sys.executable)

    def _boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd='refill', timeout=300)
    subprocess.run = _boom


out = _fragility_notice(_force_timeout)
check("a TIMEOUT is not reported as 'the KiCad refill failed'",
      'the KiCad refill failed' not in out,
      out.strip()[:200])
check("a TIMEOUT says so, with the elapsed time",
      'TIMED OUT' in out, out.strip()[:200])
check("it still says which geometry it fell back to",
      'using zone outlines' in out, out.strip()[:200])

def _force_empty_fill():
    """The SIXTH outcome: the refill ran and KiCad poured nothing. Several
    callers test truthiness rather than `is None`, so this used to be the one
    path that printed nothing at all before substituting outlines."""
    kef._KICAD_PYTHON_MEMO.append(sys.executable)
    pf_real = kef.refill_islands_ex
    kef.refill_islands_ex = lambda *a, **k: (
        {}, RefillStatus('ok', 'the refill ran and produced no islands', 1.0))
    _force_empty_fill.restore = lambda: setattr(
        kef, 'refill_islands_ex', pf_real)


out = _fragility_notice(_force_empty_fill)
_force_empty_fill.restore()
check("an EMPTY fill is announced, not silently swapped for outlines",
      'produced no islands' in out, out.strip()[:200] or '(printed nothing)')
check("an empty fill is not reported as unavailable -- the refill DID run",
      'exact fill unavailable' not in out,
      out.strip()[:200] or '(printed nothing)')

out = _fragility_notice(lambda: kef._KICAD_PYTHON_MEMO.append(None))
check("a missing interpreter still names the env var (the #647 contract)",
      'no python with pcbnew found' in out and 'KICAD_PYTHON' in out,
      out.strip()[:200])
check("no NoneType traceback leaks through either arm (#647)",
      'NoneType' not in out, out.strip()[:200])

print("\n--- the GUI provider warns on the QUIET path too")
# `_live_fill` is a closure over a live pcbnew board, so it cannot be called
# here without one. Pin the STRUCTURE instead, via the AST rather than a text
# grep: a grep is satisfied by a comment that happens to quote the call, which
# has produced a green test over deleted code in this repo before.
import ast  # noqa: E402

_kp = os.path.join(ROOT, 'py_router', 'kicad_parser.py')
with open(_kp, encoding='utf-8') as _f:
    _tree = ast.parse(_f.read())


def _find_func(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


_lf = _find_func(_tree, '_live_fill')
check("_live_fill still exists to be checked", _lf is not None)


def _warn_calls(node, inside_except):
    """Calls to the fallback warner, split by whether an `except` guards them."""
    out = []
    for child in ast.iter_child_nodes(node):
        if isinstance(child, ast.Try):
            for h in child.handlers:
                out += _warn_calls(h, True)
            for sub in (child.body, child.orelse, child.finalbody):
                for s in sub:
                    out += _warn_calls(s, inside_except)
            continue
        if (isinstance(child, ast.Call)
                and isinstance(child.func, ast.Name)
                and child.func.id == '_warn_live_fill_fallback'):
            out.append(inside_except)
        out += _warn_calls(child, inside_except)
    return out


_calls = _warn_calls(_lf, False) if _lf else []
check("the fallback warning is reachable WITHOUT an exception",
      False in _calls,
      f"guarded-by-except flags: {_calls}")
check("it is still reachable FROM an exception too",
      True in _calls, f"guarded-by-except flags: {_calls}")

print("\n--- plane_score forwards the reason, and classifies it")
import plane_score as psc  # noqa: E402

check("uniform is True exactly for the causes every candidate shares",
      [r for r in ('ok', 'no_bounds', 'no_named_net', 'no_kicad_python',
                   'memoised_failure', 'refill_failed', 'timeout', 'error')
       if psc.PlaneScoreStatus(r).uniform]
      == ['no_bounds', 'no_named_net', 'no_kicad_python'])
check("a timeout is NOT uniform -- it can strike one candidate and spare "
      "another", not psc.PlaneScoreStatus('timeout').uniform)
check("plane_score forwards the refill's own wording rather than restating it",
      psc.PlaneScoreStatus('timeout', 'limit 300s', 12.0).why()
      == RefillStatus('timeout', 'limit 300s', 12.0).why())
with _Reset():
    kef._KICAD_PYTHON_MEMO.append(None)
    _sc, _st = psc.plane_fragility_score_ex(BOARD, [('GND', 'B.Cu')])
    check("score is None and the reason survives the layer boundary",
          _sc is None and _st.reason == 'no_kicad_python', _st.reason)
check("plane_fragility_score returns the score alone",
      not isinstance(psc.plane_fragility_score(BOARD, [('__nonexistent__',
                                                        'B.Cu')]), tuple))

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
