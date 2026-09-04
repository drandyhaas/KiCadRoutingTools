#!/usr/bin/env python3
"""#713: every wall-clock site is registered, and none of them decides an output.

#621 deleted the wall-clock budget facility repo-wide on the rule that no
result may depend on timing -- and left nothing to stop it growing back. Five
sites survived, in four files, because each one LOOKS like reporting: its wrong
answer is silent.

This is the change detector that was missing. It censuses every clock site in
the engine directories and requires each FILE to be registered with a category
and the reason it may exist. An unregistered file fails the gate; a file
registered `reporting` whose clock reaches a comparison fails; a registry entry
naming a file with no clock left fails, so the list cannot rot into folklore.

Cheap by construction -- AST and regex, no subprocess, no board, ~4 s over
~250 files -- so it belongs in the fast loop where a new site is caught immediately
rather than 900 s later. Modelled on tests/test_process_order_determinism.py,
which guards the sibling class (results that depend on process history), and on
tests/test_718_static_test_hygiene.py for the shape.

Docstrings and comments are stripped before matching. Prose quoting a clock has
satisfied a source-grep gate in this repo before, and this file's own first run
flagged `clearance_ledger.py` for the word "monotonic" in a sentence.

THE CATEGORIES. The distinction that matters is not "is there a clock" but
"what does the clock decide":

  reporting      accumulates or prints elapsed time. Must NEVER reach a
                 comparison -- that is the shape all five #713 survivors had.
  throttle       a clock decides only how OFTEN a message is emitted. It
                 compares, legitimately; what it cannot change is any output.
  record         a timestamp written into a ledger row, as data.
  time_claim     a claim ABOUT time (an out-of-order ledger warning). The one
                 thing a clock is entitled to decide.
  hang_detector  a timeout on an EXTERNAL child or a thread join. Legitimate:
                 without it a hung child hangs the run forever. Its expiry
                 must yield a NAMED, non-passing verdict -- never a bare None
                 a caller reads as "unavailable", which is #713 items 3 and 4.
  harness        a driver that shells our own tools under a bound. Not an
                 engine decision; the bound belongs to the harness, which is
                 #621's stated position ("that is now the killing harness's
                 problem").
"""
import ast
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIRS = ('py_router', 'py_placer', 'py_tools', 'kicad_routing_plugin')

REGISTRY = {
    # --- hang detectors on external children -------------------------------
    'py_router/kicad_oracle.py': (
        'hang_detector',
        'ORACLE_DRC_TIMEOUT on the kicad-cli DRC child. Its expiry is a NAMED '
        'verdict -- the leg reports available/reason/why -- and the memo of it '
        'is CALL-SCOPED, so no answer crosses a call boundary (#713 item 3).'),
    'py_router/kicad_exact_fill.py': (
        'hang_detector',
        'EXACT_FILL_TIMEOUT on the pcbnew ZONE_FILLER child, plus a 120 s '
        'probe of a candidate interpreter. The expiry returns '
        'RefillStatus(timeout), distinct from the four other causes that used '
        'to share one bare None (#713 item 4).'),
    'py_tools/kicad_unconnected.py': (
        'hang_detector',
        'kicad-cli DRC child. Expiry returns an explicit error string and the '
        'CLI exits 3 -- a refusal, never a clean verdict.'),
    'kicad_routing_plugin/deps_check.py': (
        'hang_detector',
        'pip/import probes of an external toolchain, plus a worker join.'),
    'kicad_routing_plugin/ai_gui.py': (
        'hang_detector', 'a stderr reader thread join.'),
    'kicad_routing_plugin/ai_plan.py': (
        'hang_detector', 'worker thread joins on plan cancellation.'),

    # --- harness drivers ----------------------------------------------------
    'py_router/headless_plan.py': (
        'harness',
        'executes a GUI plan step by step through the real plugin for the '
        'parity harnesses. The bound is the harness\'s, not an engine '
        'decision; it also prints per-step elapsed.'),
    'py_router/run_plan.py': (
        'harness', 'the CLI front for headless_plan; forwards its bound.'),

    # --- throttles: a clock decides only how often something PRINTS ---------
    'py_router/obstacle_map.py': (
        'throttle',
        'a ~5 s log throttle and a 0.25 s progress-callback throttle, so a '
        'minutes-long build is visibly progressing. Its own docstring: '
        '"Reporting only: the map contents are untouched."'),
    'kicad_routing_plugin/gui_utils.py': (
        'throttle',
        'a 0.05 s UI repaint throttle so a fast per-ball burst costs a '
        'bounded number of event-loop turns.'),

    # --- records: a timestamp stored as data --------------------------------
    'py_placer/board_store.py': ('record', 'ledger row timestamp'),
    'py_placer/placement/provenance.py': ('record', 'provenance row timestamp'),
    'py_router/redo_record.py': (
        'record', 'wall seconds recorded into the redo manifest row'),

    # --- a claim about time itself ------------------------------------------
    'py_placer/converge.py': (
        'time_claim',
        'the ledger BACK-FILL warning compares two recorded timestamps to say '
        'the entries are out of order. It changes no output; it is a claim '
        'ABOUT time, which is the one thing a clock may decide.'),

    # --- reporting only: elapsed accumulated or printed ---------------------
    'py_router/route.py': ('reporting', 'phase timing prints'),
    'py_router/route_diff.py': ('reporting', 'phase timing prints'),
    'py_router/global_plan.py': ('reporting', 'plan timing print'),
    'py_router/leg_rip.py': ('reporting', 'per-rip ms print'),
    'py_router/net_rescue.py': ('reporting', "per-net timing and summary['time']"),
    'py_router/phase3_routing.py': ('reporting', 'stats.total_time'),
    'py_router/plane_fragility.py': ('reporting', 'refresh_s accumulator'),
    'py_router/plane_obstacle_builder.py': ('reporting', 'per-stage timing prints'),
    'py_router/diff_pair_loop.py': ('reporting', 'loop timing'),
    'py_router/reroute_loop.py': ('reporting', 'loop timing'),
    'py_router/single_ended_loop.py': ('reporting', 'loop timing'),
    'py_router/single_ended_routing.py': ('reporting', 'tap-phase timing'),
    'kicad_routing_plugin/placement_gui.py': (
        'throttle',
        'elapsed in the status line, plus a QUIET_AFTER_S comparison that '
        'decides whether to append "(last agent event Nm ago)" to that '
        'string. A display decision, not an output one -- and it is here as a '
        'throttle rather than as reporting because this gate flagged the '
        'comparison and the honest answer was to reclassify it, not to widen '
        'the reporting rule.'),
    'kicad_routing_plugin/swig_gui.py': (
        'reporting', 'wall_time reported after a routing run'),
    'py_router/plane_region_connector.py': (
        'reporting',
        '_total_route_time, via `import time as _time`. Invisible to the '
        'first discovery regex, whose \\b before `time.time` is killed by the '
        'leading underscore.'),
    'py_router/history_congestion.py': (
        'reporting',
        'record_s / rows_s, via `from time import perf_counter as _perf`. '
        'Invisible to the first discovery regex, and its ACCUMULATORS are '
        '`+=`, which the comparison visitor also could not see.'),

    # --- the hang detector the first census could not see -------------------
    'py_router/ui_thread.py': (
        'hang_detector',
        'SAVE_BOARD_UI_TIMEOUT_S = 120 on `done.wait(timeout_s)`, marshalling '
        'a SaveBoard onto the wx main thread (#688). The timeout is passed '
        'POSITIONALLY, so `timeout=` never matched it. Its expiry used to '
        'return a bare False a caller could not tell from a wx save '
        'exception -- the shape this category exists to forbid; #828 added '
        'save_board_on_ui_thread_ex, whose SaveStatus names `timeout` '
        '(this machine, this moment) apart from `save_failed` (this board), '
        'and both callers now branch on it. tests/test_828_ui_thread_save_'
        'status.py pins the three arms wx-free.'),

    # --- filesystem timestamps, not our elapsed time ------------------------
    'py_router/animate_route.py': (
        'file_mtime', 'orders movie frames by st_mtime'),
    'py_router/route_planes.py': (
        'file_mtime',
        'compares the output file st_mtime before/after to detect that a '
        'post-pass rewrote it'),
    'kicad_routing_plugin/placement_run.py': (
        'file_mtime',
        'strftime run-directory name, and picks the newest artifact by mtime'),

    # --- an event-loop yield ------------------------------------------------
    'kicad_routing_plugin/differential_gui.py': (
        'throttle', 'time.sleep(0.01) yielding to the wx event loop'),
    'kicad_routing_plugin/planes_gui.py': (
        'throttle', '_time.sleep(0.01) yielding to the wx event loop'),

    # --- a render cap, not a routing decision -------------------------------
    'py_router/make_movie.py': (
        'harness',
        '--camera-budget caps a RENDER, which produces no board'),

    # --- no clock at all ----------------------------------------------------
    'py_router/routing_common.py': (
        'unused_import',
        'a bare `import time` with no call anywhere; every other "time" here '
        'is time_matching, a propagation delay, which is physics not a clock'),
}

#: Categories whose clock may legitimately reach a comparison.
MAY_COMPARE = {'throttle', 'time_claim', 'hang_detector', 'harness', 'record',
               'file_mtime', 'unused_import'}

#: A clock CALL, as opposed to a mere import or a constant. Used to hold the
#: `unused_import` category honest: a file registered that way must contain no
#: call at all, or the registration is a lie the moment someone uses it.
_CLOCK_CALL = re.compile(
    r'(?:\w*\.)?(?:time|monotonic|perf_counter|process_time)\s*\(')

#: DISCOVERY. Deliberately looser than it looks like it needs to be, because
#: the first version missed three real files and an adversarial review found
#: them, not this gate:
#:
#:   `_time.time()`      -- `\btime\.time` does not match: the leading
#:                          underscore kills the word boundary
#:                          (plane_region_connector.py)
#:   `_perf()` after `from time import perf_counter as _perf`
#:                       -- the call site carries none of the real names
#:                          (history_congestion.py)
#:   `done.wait(timeout_s)` -- a timeout passed POSITIONALLY, with a 120 s
#:                          constant beside it (ui_thread.py, a live hang
#:                          detector whose expiry returns a bare False)
#:
#: The alias fix that shipped first went into the AST visitor, which only ever
#: runs on files DISCOVERY already found -- so it could not have caught the
#: third of those. Discovery is the layer that has to be generous; the
#: registry is where precision belongs.
_CLOCK = re.compile(
    # a clock call, however the module or function was aliased
    r'(?:\w*\.)?(?:time|monotonic|perf_counter|process_time)\s*\(|'
    # any import OF a clock, which is what an alias must go through
    r'from\s+time\s+import|'
    r'\bimport\s+time\b|'
    # a timeout, by keyword or as a named constant, and the exception
    r'\btimeout\w*\s*=|'
    r'\b[A-Z_]*TIMEOUT[A-Z_]*\b|'
    r'\bTimeoutExpired\b|'
    # the other stdlib clocks, so a future site cannot arrive unseen
    r'\bdatetime\.now\s*\(|\bsignal\.alarm\s*\(|\bthreading\.Timer\b|'
    r'\basyncio\.wait_for\b|\bsettimeout\s*\(')

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


def _strip_prose(src):
    """Source with comments and every docstring removed.

    Both matter. A comment quoting a clock and a docstring SENTENCE containing
    the word "monotonic" both produced false positives on this file's first
    run, and a gate that can be satisfied by prose is not a gate.
    """
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return '\n'.join(l.split('#')[0] for l in src.splitlines())
    doc_lines = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.ClassDef)):
            body = getattr(node, 'body', None)
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                d = body[0]
                doc_lines.update(range(d.lineno, (d.end_lineno or d.lineno) + 1))
    out = []
    for i, line in enumerate(src.splitlines(), 1):
        out.append('' if i in doc_lines else line.split('#')[0])
    return '\n'.join(out)


def _py_files():
    for d in DIRS:
        for base, dirs, files in os.walk(os.path.join(ROOT, d)):
            dirs[:] = [x for x in dirs if x != '__pycache__']
            for f in sorted(files):
                if f.endswith('.py'):
                    p = os.path.join(base, f)
                    yield os.path.relpath(p, ROOT).replace(os.sep, '/'), p


# Every file read and stripped ONCE. The first version re-walked the tree for
# each removed-flag check and cost 14.6 s, while its own docstring claimed
# "well under a second" -- a false claim in the gate that exists to stop false
# claims.
SOURCES = {}
for _rel, _path in _py_files():
    with open(_path, encoding='utf-8') as _f:
        _raw = _f.read()
    SOURCES[_rel] = (_raw, _strip_prose(_raw))

found = {rel: raw for rel, (raw, code) in SOURCES.items()
         if _CLOCK.search(code)}

print(f"--- {len(found)} file(s) carry a clock; every one must be registered")
_unregistered = sorted(set(found) - set(REGISTRY))
check("no unregistered wall-clock site in the engine directories",
      not _unregistered,
      "register it with a category and a reason, or remove it: "
      + ', '.join(_unregistered))
_stale = sorted(set(REGISTRY) - set(found))
check("no REGISTRY entry names a file whose clock is gone",
      not _stale, "stale: " + ', '.join(_stale))
check("every registration carries a reason, not just a category",
      all(len(v) == 2 and v[1].strip() for v in REGISTRY.values()))
check("every category is one of the declared ones",
      all(v[0] in (MAY_COMPARE | {'reporting'}) for v in REGISTRY.values()),
      str(sorted({v[0] for v in REGISTRY.values()})))

print("\n--- a reporting-only clock never reaches a decision")


class _Decides(ast.NodeVisitor):
    """Does a clock READING flow into a comparison?

    Narrow on purpose: it flags a Compare whose operand is a clock call, or a
    name assigned from one, or an arithmetic expression over either. That is
    the shape all five #713 survivors had (`spent > budget`, `now - t0 > x`).
    """

    #: The clock functions, by their real names.
    _FUNCS = ('time', 'monotonic', 'perf_counter')

    def __init__(self):
        self.clock_names = set()
        #: Local names bound to a clock function by an import alias.
        self.aliases = set()
        #: Module aliases: `import time as t` -> t.monotonic() is a clock.
        self.modules = {'time'}
        self.hits = []

    def visit_ImportFrom(self, node):
        if node.module == 'time':
            for a in node.names:
                if a.name in self._FUNCS:
                    self.aliases.add(a.asname or a.name)
        self.generic_visit(node)

    def visit_Import(self, node):
        for a in node.names:
            if a.name == 'time':
                self.modules.add(a.asname or a.name)
        self.generic_visit(node)

    def _is_clock(self, node):
        if isinstance(node, ast.Call):
            f = node.func
            if isinstance(f, ast.Attribute):
                # `time.monotonic()`, and `t.monotonic()` after an alias.
                base = f.value.id if isinstance(f.value, ast.Name) else ''
                return f.attr in self._FUNCS and (base in self.modules
                                                  or base == '')
            if isinstance(f, ast.Name):
                # A bare call: the real name, or an import alias for it.
                # `from time import monotonic as _mono` evaded the first
                # version of this check entirely, and an aliased import is
                # exactly how an absence guard has been dodged here before.
                return f.id in self._FUNCS or f.id in self.aliases
            return False
        if isinstance(node, ast.Name):
            return node.id in self.clock_names
        if isinstance(node, ast.BinOp):
            return self._is_clock(node.left) or self._is_clock(node.right)
        if isinstance(node, ast.Subscript):
            return self._is_clock(node.value)
        return False

    def _bind(self, target):
        if isinstance(target, ast.Name):
            self.clock_names.add(target.id)
        elif isinstance(target, ast.Subscript) and isinstance(target.value,
                                                              ast.Name):
            self.clock_names.add(target.value.id)
        elif isinstance(target, ast.Attribute):
            self.clock_names.add(target.attr)

    def visit_Assign(self, node):
        if self._is_clock(node.value):
            for t in node.targets:
                self._bind(t)
        self.generic_visit(node)

    def visit_AnnAssign(self, node):
        if node.value is not None and self._is_clock(node.value):
            self._bind(node.target)
        self.generic_visit(node)

    def visit_AugAssign(self, node):
        """`spent += time.time() - t0` -- the ACCUMULATOR shape.

        This is the literal definition of the `reporting` category, and the
        first version of this visitor could not see it: only `visit_Assign`
        bound clock names, so `spent` was never a clock and `if spent >
        budget` was invisible. `history_congestion.record_s` and
        `plane_fragility.refresh_s` are both live `+=` accumulators.
        """
        if self._is_clock(node.value):
            self._bind(node.target)
        self.generic_visit(node)

    def visit_Compare(self, node):
        if self._is_clock(node.left) or any(self._is_clock(c)
                                            for c in node.comparators):
            self.hits.append(node.lineno)
        self.generic_visit(node)


_checked = 0
for rel, src in sorted(found.items()):
    cat = REGISTRY.get(rel, ('', ''))[0]
    if cat != 'reporting':
        continue
    _checked += 1
    v = _Decides()
    v.visit(ast.parse(src))
    check(f"{rel}: no clock reaches a comparison", not v.hits,
          f"lines {v.hits} -- if this is a throttle, register it as one")
check("the reporting check actually ran on the reporting files",
      _checked >= 10, f"{_checked} file(s) checked")

for rel, (cat, _why) in sorted(REGISTRY.items()):
    if cat == 'unused_import':
        check(f"{rel}: registered as an unused import, and really has no call",
              not _CLOCK_CALL.search(SOURCES[rel][1]),
              "the registration stops being true the moment someone uses it")

print("\n--- the deleted budgets stay deleted")
_GONE = {
    '--deadline': 'the #621 facility',
    '--plane-score-budget': '#713 item 1: it picked the winning candidate',
    '--route-timeout': '#713 item 2: it erased a probe verdict',
}
# `add_argument\s*\(` then the flag anywhere in the call, not the two literal
# spellings the first version matched: `add_argument(` followed by a newline is
# the prevailing style at 5 sites in these directories, and
# `add_argument( "--flag"` with a space evaded it too. The repo already ships
# this idiom in tests/test_doc_flag_liveness.py.
_ADD_ARG_CALL = re.compile(r'add_argument\s*\(')


def _flag_is_registered(code, flag):
    """Does any add_argument(...) call in `code` name `flag`?"""
    for m in _ADD_ARG_CALL.finditer(code):
        depth, i = 0, m.end() - 1
        while i < len(code):
            if code[i] == '(':
                depth += 1
            elif code[i] == ')':
                depth -= 1
                if depth == 0:
                    break
            i += 1
        if flag in code[m.end():i]:
            return True
    return False


for flag, why in _GONE.items():
    live = [rel for rel, (_raw, code) in SOURCES.items()
            if _flag_is_registered(code, flag)]
    check(f"{flag} is not re-added ({why})", not live, ', '.join(live))

check("no wall-clock budget env knob",
      'KRT_DEADLINE' not in open(os.path.join(ROOT, 'py_router',
                                              'env_knobs.py'),
                                 encoding='utf-8').read())

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
