#!/usr/bin/env python3
"""#713 item 3: the oracle's timeout memo is call-scoped, and a leg that did
not run says so.

The issue reads this site backwards, and the correction is the point of the
change. It says one slow DRC "disables the KiCad oracle for the rest of the
run" via a module-global memo. The memo's only READ sat behind
`env_knobs.LEGACY_ORACLE` -- `KICAD_LEGACY_ORACLE`, default off, set by
nothing in this repo -- so it was written and never consulted. The live defect
was the inverse: ORACLE_DRC_TIMEOUT was re-paid on every round of every leg.

A module-global fixes the re-pay by making the answer depend on RUN HISTORY,
which is the class of defect #713 is about, and pcbnew is a long-lived process
so in the GUI one slow board would poison every board opened after it.
Call-scoped state fixes the re-pay AND keeps the answer a function of the
board.

The gate has to be structural in places: driving a real kicad-cli timeout would
mean finding a board slow enough on this machine, which is the machine-speed
dependence the change exists to remove.
"""
import ast
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import kicad_oracle as ko  # noqa: E402

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


print("--- no module-global carries a verdict between boards")
check("_ORACLE_TIMED_OUT is gone", not hasattr(ko, '_ORACLE_TIMED_OUT'))
_src = open(os.path.join(ROOT, 'py_router', 'kicad_oracle.py'),
            encoding='utf-8').read()
_tree = ast.parse(_src)
# Module-level assignments to a mutable container are how run history leaks in.
_globals = {}
_CONTAINERISH = (ast.Call, ast.Dict, ast.List, ast.Set, ast.DictComp,
                 ast.ListComp, ast.SetComp, ast.Name)
for node in _tree.body:
    # AnnAssign as well as Assign. `_UNCONNECTED_MEMO: Dict[...] = {}` is an
    # AnnAssign and was not collected at all by the first version -- so a new
    # `_ORACLE_TIMED_OUT_2: set = set()`, which is the exact style
    # kicad_exact_fill._REFILL_FAILED already uses, would have passed this
    # gate. Comprehensions and a bare alias (`_D = _B`) were missed too.
    if isinstance(node, ast.Assign) and isinstance(node.value, _CONTAINERISH):
        for t in node.targets:
            if isinstance(t, ast.Name):
                _globals[t.id] = ast.dump(node.value)[:40]
    elif (isinstance(node, ast.AnnAssign) and node.value is not None
            and isinstance(node.value, _CONTAINERISH)
            and isinstance(node.target, ast.Name)):
        _globals[node.target.id] = ast.dump(node.value)[:40]
# Declared, with the reason each is allowed to persist across boards.
_ALLOWED = {
    'KICAD_CLI_CANDIDATES': 'a PATH probe, not a verdict about any board',
    '_IGNORE_SEVERITIES': 'a constant list of DRC severities',
    '_WARNED_NO_CLI': 'a once-per-process shout; changes no output',
    '_UNCONNECTED_MEMO': 'keyed on the board BYTES, so it cannot answer for a '
                         'different board',
    '_NET_RE': 'a compiled regex', '_LAYER_RE': 'a compiled regex',
}
_unregistered = sorted(set(_globals) - set(_ALLOWED))
check("every module-global container is registered with a reason",
      not _unregistered,
      f"unregistered: {_unregistered} -- a new one may carry run history")

print("\n--- the memo is a caller-owned set")
import inspect  # noqa: E402
_sig = inspect.signature(ko.kicad_unconnected)
check("kicad_unconnected takes a caller-owned `timed_out`",
      'timed_out' in _sig.parameters)
check("and defaults it to None, so a direct caller memoises nothing",
      _sig.parameters['timed_out'].default is None)

_orc_src = ast.get_source_segment(
    _src, next(n for n in ast.walk(_tree)
               if isinstance(n, ast.FunctionDef) and n.name == 'oracle_reconnect'))
check("oracle_reconnect creates exactly one set for the whole call",
      _orc_src.count('_timed_out = set()') == 1)
check("and every kicad_unconnected it makes shares it",
      _orc_src.count('kicad_unconnected(board_file, kicad_cli')
      == _orc_src.count('timed_out=_timed_out'),
      f"{_orc_src.count('kicad_unconnected(board_file, kicad_cli')} calls, "
      f"{_orc_src.count('timed_out=_timed_out')} threaded")

print("\n--- behaviour: a timeout is remembered WITHIN a call, forgotten after")
_spawns = {'n': 0}
_real_run = subprocess.run


def _always_timeout(*a, **k):
    _spawns['n'] += 1
    raise subprocess.TimeoutExpired(cmd='kicad-cli', timeout=240)


BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
ko.subprocess.run = _always_timeout
try:
    shared = set()
    ko.kicad_unconnected(BOARD, 'kicad-cli', timed_out=shared)
    n_after_first = _spawns['n']
    ko.kicad_unconnected(BOARD, 'kicad-cli', timed_out=shared)
    ko.kicad_unconnected(BOARD, 'kicad-cli', timed_out=shared)
    check("the first call spawns", n_after_first == 1, f"{n_after_first}")
    check("later calls SHARING the set do not re-pay the timeout",
          _spawns['n'] == 1, f"{_spawns['n']} spawns for 3 calls")
    check("the set records the board", len(shared) == 1, str(len(shared)))

    # A FRESH set is a fresh call: it must decide for itself.
    _spawns['n'] = 0
    ko.kicad_unconnected(BOARD, 'kicad-cli', timed_out=set())
    check("a NEW call re-decides -- no verdict crosses the call boundary",
          _spawns['n'] == 1, f"{_spawns['n']}")

    # No set at all: every call decides for itself, as every direct caller does.
    _spawns['n'] = 0
    ko.kicad_unconnected(BOARD, 'kicad-cli')
    ko.kicad_unconnected(BOARD, 'kicad-cli')
    check("with no set, nothing is memoised", _spawns['n'] == 2,
          f"{_spawns['n']}")
finally:
    ko.subprocess.run = _real_run

print("\n--- a verdict that did not happen says why")
_avail_returns = [n for n in ast.walk(_tree)
                  if isinstance(n, ast.Return) and isinstance(n.value, ast.Dict)
                  and any(isinstance(k, ast.Constant) and k.value == 'available'
                          for k in n.value.keys)]
check("every oracle_reconnect return carries a reason",
      _avail_returns and all(
          any(isinstance(k, ast.Constant) and k.value == 'reason'
              for k in r.value.keys) for r in _avail_returns),
      f"{len(_avail_returns)} available-returns")
check("...and a human `why`",
      all(any(isinstance(k, ast.Constant) and k.value == 'why'
              for k in r.value.keys) for r in _avail_returns))

print("\n--- the consumers stopped guessing the cause")
_rp = open(os.path.join(ROOT, 'py_router', 'repair_planes.py'),
           encoding='utf-8').read()
check("repair_planes no longer blames 'kicad-cli not found' for every cause",
      "'NOTE: kicad-cli not found -- the oracle reconnect pass '" not in _rp)
check("it prints the oracle's own reason instead", "_orc.get('why'" in _rp)

_gu = open(os.path.join(ROOT, 'kicad_routing_plugin', 'gui_utils.py'),
           encoding='utf-8').read()
check("the GUI front now inspects `available` at all",
      "orc.get('available')" in _gu,
      "an oracle that could not run was indistinguishable from a clean one")
check("and says the clean result is unbacked", 'UNBACKED' in _gu)

_rt = open(os.path.join(ROOT, 'py_router', 'route.py'), encoding='utf-8').read()
check("the plane-finalize leg reaches JSON_SUMMARY at all",
      "summary['oracle_reconnect']" in _rt,
      "it had no summary key, so available:False was invisible to consumers")

print("\n--- the constant is untouched: a hang detector, not a budget")
check("ORACLE_DRC_TIMEOUT is still 240", ko.ORACLE_DRC_TIMEOUT == 240,
      str(ko.ORACLE_DRC_TIMEOUT))

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
