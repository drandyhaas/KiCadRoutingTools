"""
KiCad Routing Tools - the Bus tab's model of a bus step, wx-free.

The Bus tab (bus_gui.py) runs awx/route_bus.py as a child process and follows
its events file (awx/bus_events.py: one JSON object a line, written by every
process of the step). This module is the part of the tab that needs no wx, so
it is tested on its own:

  build_argv / find_argv   the step's command line, the CLI's own: a GUI run
                           and a terminal run of the same arguments are the
                           same run (and the same redo-manifest line)
  BusList                  the buses the tab will name: --find's, in its
                           order, or the user's own list (--bus each, in order)
  RunModel                 the events read into rows -- a group, its buses,
                           their stages -- each with a state, a time and a
                           result, and the fractions the tab's gauges show
  describe                 one event as a line of text (the status line)

Nothing here estimates: a gauge is a count of things that ended out of things
that will run (a group's stages, a group's buses), and a solve shows its
elapsed time, never a percentage.
"""

import ast
import json
import os

PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(PLUGIN_DIR)
AWX_DIR = os.path.join(ROOT_DIR, 'awx')
ROUTE_BUS_SCRIPT = os.path.join(AWX_DIR, 'route_bus.py')
EVENTS_NAME = 'events.jsonl'

# A group's stages, in order, for the stage gauge: the buses' ends models
# captured, the group's ends model, the buses routed on its field, the merge of
# their boards and its grade. A group round after the first runs the last four
# again.
GROUP_STAGES = ('capture', 'ends', 'route', 'merge', 'grade')
ROUND_STAGES = GROUP_STAGES[1:]

_STAGE_NAMES = {
    'capture': "ends model captured",
    'ends': "the group's ends model",
    'route': "buses routed on the field",
    'merge': "merged",
    'grade': "graded on the board",
    'fanout': "fanout",
    'solve': "solve",
    'coupled': "coupled solve",
    'loop': "loop",
    'lay': "lay",
}


def _size_args():
    """route_bus.SIZE_ARGS, read from its source: the sizes it takes, in the
    order its flags are passed. Read, not imported -- the step's modules are
    the child's, and are not loaded into KiCad's process."""
    try:
        with open(ROUTE_BUS_SCRIPT, encoding='utf-8') as fh:
            tree = ast.parse(fh.read())
        for node in tree.body:
            if (isinstance(node, ast.Assign) and len(node.targets) == 1
                    and isinstance(node.targets[0], ast.Name)
                    and node.targets[0].id == 'SIZE_ARGS'):
                return tuple(ast.literal_eval(node.value))
    except (OSError, SyntaxError, ValueError):
        pass
    return ('clearance', 'clearance_ceiling', 'track_width', 'fanout_track_width',
            'via_size', 'via_drill', 'hole_to_hole_clearance', 'board_edge_clearance')


SIZE_ARGS = _size_args()


def _flag(name):
    return '--' + name.replace('_', '-')


def _num(v):
    """A size as the command line spells it: the shortest exact repr."""
    return repr(float(v)) if not float(v).is_integer() else str(float(v))


def parse_bus_spec(spec):
    """'SRC:DEST' -> (src, dest), DEST one part or several ('U4+U3'); raises
    ValueError for anything route_bus.parse_bus would refuse."""
    parts = str(spec).strip().split(':')
    if len(parts) != 2 or not parts[0] or not parts[1] or any(
            not r for r in parts[1].split('+')):
        raise ValueError(f"bus {spec!r}: SRC:DEST, DEST one part or several "
                         f"joined by '+' (U4+U3)")
    return parts[0], parts[1]


def size_flags(sizes):
    """The sizes given, as route_bus flags in SIZE_ARGS order; a size that is
    None is omitted, which the step resolves as route.py does."""
    out = []
    for name in SIZE_ARGS:
        v = (sizes or {}).get(name)
        if v is not None:
            out += [_flag(name), _num(v)]
    return out


def build_argv(python, board, out, buses=None, options=None, sizes=None,
               script=ROUTE_BUS_SCRIPT):
    """The bus step's command line, as a terminal would type it.

    BUSES: the --bus specs in routing order, or None for the buses the step
    finds itself (no --bus). OPTIONS: rounds, feedback_rounds, k, inproc,
    round_edits -- each passed only where it differs from the CLI's default,
    so the defaults give the bare command. SIZES: SIZE_ARGS name -> mm, or
    None for one not given (fanout_track_width among them)."""
    import routing_defaults as defaults
    o = options or {}
    argv = [python, '-u', '-X', 'utf8', script, board, out]
    for spec in buses or ():
        parse_bus_spec(spec)
        argv += ['--bus', spec]
    rounds = int(o.get('rounds', defaults.BUS_ROUNDS))
    if rounds != defaults.BUS_ROUNDS:
        argv += ['--rounds', str(rounds)]
    fb = int(o.get('feedback_rounds', defaults.BUS_FEEDBACK_ROUNDS))
    if fb != defaults.BUS_FEEDBACK_ROUNDS:
        argv += ['--feedback-rounds', str(fb)]
    if int(o.get('k') or 0) > 0:
        argv += ['--k', str(int(o['k']))]
    if o.get('inproc'):
        argv.append('--inproc')
    if o.get('round_edits'):
        argv += ['--round-edits', str(o['round_edits'])]
    return argv + size_flags(sizes)


def find_argv(python, board, json_out, sizes=None, script=ROUTE_BUS_SCRIPT):
    """route_bus --find at SIZES, its listing written to JSON_OUT."""
    sizes = {k: v for k, v in (sizes or {}).items() if k != 'fanout_track_width'}
    return ([python, '-u', '-X', 'utf8', script, board, '--find',
             '--json-out', json_out] + size_flags(sizes))


#: The child-python probe, run in a child, never in KiCad's process
#: (deps_check.CLI_ONLY_PACKAGES): the step's source parses under it, the
#: step's own dependency gate passes (OR-Tools, the Rust router), and -- where
#: pcbnew is importable, as in KiCad's own python -- pcbnew loads beside
#: OR-Tools: the step's modules import pcbnew wherever they can, and the two
#: in one process can crash it (measured with KiCad 10's bundled python on
#: macOS, either order). Each problem is a PROBLEM: line; the last line is OK
#: or NOT OK, absent when the pcbnew load crashed the probe (PCBNEW_CHECK
#: printed before it).
PROBE = r"""
import importlib.util, sys
awx, pyr, script = sys.argv[1], sys.argv[2], sys.argv[3]
print('python %d.%d.%d' % sys.version_info[:3], flush=True)
has_pcbnew = importlib.util.find_spec('pcbnew') is not None
if has_pcbnew:
    print('PCBNEW_IMPORTABLE', flush=True)
bad = 0
try:
    compile(open(script, encoding='utf-8').read(), script, 'exec')
except SyntaxError as e:
    bad += 1
    print('PROBLEM: %s does not parse under this Python %d.%d (%s, line %s)'
          % (script, sys.version_info[0], sys.version_info[1], e.msg, e.lineno), flush=True)
sys.path[:0] = [awx, pyr]
try:
    from startup_checks import check_bus_dependencies
    check_bus_dependencies()
except Exception as e:
    bad += 1
    print('PROBLEM: ' + str(e).replace('\n', ' | '), flush=True)
else:
    if has_pcbnew:
        print('PCBNEW_CHECK', flush=True)
        import pcbnew  # noqa: F401
print('NOT OK' if bad else 'OK', flush=True)
sys.exit(3 if bad else 0)
"""

#: What a probe that died loading pcbnew beside OR-Tools means.
PCBNEW_CLASH = ("OR-Tools and pcbnew cannot be loaded in one process under this "
                "Python (the probe crashed loading them together), and the step's "
                "modules load pcbnew wherever it is importable: choose a Python "
                "without KiCad's pcbnew for the step")


def probe_verdict(returncode, output):
    """{ok, version, problems, pcbnew, installable} from the probe's exit and
    output: `pcbnew` whether that python can import KiCad's pcbnew, and
    `installable` whether installing OR-Tools there is the one thing it lacks
    -- never for a python with pcbnew, beside which OR-Tools crashed the
    probe (PCBNEW_CLASH)."""
    lines = [ln.strip() for ln in (output or '').splitlines() if ln.strip()]
    ver = next((ln for ln in lines if ln.startswith('python ')), '')
    problems = [ln[len('PROBLEM: '):] for ln in lines if ln.startswith('PROBLEM: ')]
    if 'PCBNEW_CHECK' in lines and lines[-1] not in ('OK', 'NOT OK'):
        problems.append(PCBNEW_CLASH + f" (exit {returncode})")
    ok = returncode == 0 and lines[-1:] == ['OK'] and not problems
    if not ok and not problems:
        problems.append(lines[-1] if lines else f"no output (exit {returncode})")
    pcb = 'PCBNEW_IMPORTABLE' in lines
    installable = (not ok and not pcb and len(problems) == 1
                   and 'ortools' in problems[0].lower())
    return {'ok': ok, 'version': ver, 'problems': problems, 'pcbnew': pcb,
            'installable': installable}


_EVENTS = []


def bus_events():
    """awx/bus_events.py -- the events file's writer, whose `read` and
    `default_path` the tab uses -- imported with awx/ on sys.path for the
    import alone: that folder's other module names are not to shadow
    anything in KiCad's process."""
    if not _EVENTS:
        import sys
        sys.path.insert(0, AWX_DIR)
        try:
            import bus_events as be
        finally:
            sys.path.remove(AWX_DIR)
        _EVENTS.append(be)
    return _EVENTS[0]


def probe_argv(python):
    return [python, '-c', PROBE, AWX_DIR, os.path.join(ROOT_DIR, 'py_router'),
            ROUTE_BUS_SCRIPT]


def read_find(path):
    """--find --json-out's listing: {buses, left, step, arrays}, or None."""
    try:
        with open(path, encoding='utf-8') as fh:
            got = json.load(fh)
    except (OSError, ValueError):
        return None
    return got if isinstance(got, dict) else None


# ---------------------------------------------------------------- the bus list

class BusList:
    """The buses the tab names, in routing order: rows of
    {spec, checked, nets, why, found}. While the rows are --find's, all
    checked and in its order, the run names none (the step finds the same
    buses itself, at the same sizes); once the user unchecks, reorders, adds
    or edits one, the run names each checked row with --bus, in order."""

    def __init__(self):
        self.rows = []
        self.found = None           # --find's specs, in order, or None
        self.left = []              # --find's buses not taken
        self.arrays = []            # the board's ball arrays (--find)

    def set_found(self, listing, keep_rows=False):
        """--find's listing. KEEP_ROWS: a list the user or a plan named
        stays as it is, its rows given --find's nets and why where it has
        them."""
        self.found = [f"{b['src']}:{b['dest']}" for b in listing.get('buses') or []]
        self.left = list(listing.get('left') or [])
        self.arrays = list(listing.get('arrays') or [])
        rows = [dict(spec=f"{b['src']}:{b['dest']}", checked=True,
                     nets=b.get('nets'), why=b.get('why') or '', found=True)
                for b in listing.get('buses') or []]
        if keep_rows and self.rows:
            by = {r['spec']: r for r in rows}
            for r in self.rows:
                if r['spec'] in by:
                    r.update(nets=by[r['spec']]['nets'], why=by[r['spec']]['why'], found=True)
            return
        self.rows = rows

    def set_named(self, specs):
        """The rows a plan step names, in its order, every one checked."""
        known = {r['spec']: r for r in self.rows}
        self.rows = []
        for spec in specs:
            parse_bus_spec(spec)
            r = dict(known.get(spec) or dict(spec=spec, nets=None, why='', found=False))
            r['checked'] = True
            self.rows.append(r)

    def use_found(self):
        """Back to the step's own buses: --find's rows, all checked."""
        if self.found is not None:
            by = {r['spec']: r for r in self.rows}
            self.rows = [dict(by.get(s) or dict(spec=s, nets=None, why='', found=True),
                              checked=True) for s in self.found]
        else:
            self.rows = []

    def checked(self):
        return [r['spec'] for r in self.rows if r['checked']]

    def names_buses(self):
        """Does the run name its buses (--bus), rather than take --find's?"""
        return self.found is None and bool(self.rows) or (
            self.found is not None and self.checked() != self.found)

    def argv_buses(self):
        """The --bus specs for the run, or None for the step's own."""
        return self.checked() if self.names_buses() else None

    def add(self, spec, at=None):
        parse_bus_spec(spec)
        row = dict(spec=spec, checked=True, nets=None, why='added by hand', found=False)
        self.rows.insert(len(self.rows) if at is None else at, row)

    def replace(self, i, spec):
        parse_bus_spec(spec)
        self.rows[i] = dict(self.rows[i], spec=spec, why='edited by hand', found=False,
                            nets=None)

    def remove(self, i):
        del self.rows[i]

    def move(self, i, d):
        j = i + d
        if 0 <= i < len(self.rows) and 0 <= j < len(self.rows):
            self.rows[i], self.rows[j] = self.rows[j], self.rows[i]
            return j
        return i


# --------------------------------------------------------------- the run model

def _fmt_secs(s):
    if s is None:
        return ''
    s = int(round(s))
    return f"{s // 60}:{s % 60:02d}"


def describe(ev):
    """One event as a line of text."""
    st, kind = ev.get('stage'), ev.get('kind')
    g = lambda k, d='?': ev.get(k, d) if ev.get(k) is not None else d   # noqa: E731
    if st == 'step':
        if kind == 'start':
            return ('the step starts: ' + (', '.join(ev['buses']) if ev.get('buses')
                                          else 'the buses it finds'))
        if kind == 'plan':
            return (f"buses found: {', '.join(b['src'] + ':' + b['dest'] for b in ev.get('buses') or []) or 'none'}"
                    + (f"; {len(ev.get('left') or [])} not taken" if ev.get('left') else ''))
        if kind == 'end':
            return {0: 'the step ended: every bus handed on routed',
                    1: 'the step ended: a bus left as it came (or none found)',
                    2: 'the step ended: refused'}.get(ev.get('status'), f"the step ended ({ev.get('status')})")
    if st == 'group':
        if kind == 'start':
            return f"group {ev.get('group')}: {', '.join(ev.get('buses') or [])}"
        if kind == 'round':
            return (f"group round {g('round')}: {g('open')} open, DRC {g('drc')}, "
                    f"{g('walled')} walled ({g('secs')} s)")
        if kind == 'end':
            return (f"group {ev.get('group')} handed on: {g('open')} open, DRC {g('drc')}, "
                    f"{g('walled')} walled")
    if st == 'capture':
        if kind == 'start':
            return f"capturing each bus's ends model: {', '.join(ev.get('buses') or [])}"
        if kind == 'end':
            return (f"ends model captured after {g('secs')} s" if ev.get('ok')
                    else f"ends model NOT captured (exit {g('exit')}) -- {ev.get('log')}")
    if st == 'ends':
        if kind == 'start':
            return f"the group's ends model, round {g('round')}"
        if kind == 'solve':
            return (f"model check {g('check')}: [{g('status')}] in {g('secs')} s, the rest {g('rest')}"
                    + (f" (bound {ev['bound']})" if ev.get('bound') is not None else '')
                    + f"; {g('changing_layer')} lanes changing layer, {g('walled')} walled")
        if kind == 'check':
            return f"model check {g('check')}: {', '.join(ev.get('refused') or [])} refused -- solved again"
        if kind == 'field':
            return (f"the field as laid: DRC {g('drc')} (the bare board {g('drc_bare')})"
                    + (f", {ev['lessons']} lesson(s)" if ev.get('lessons') else '')
                    + (' -- breaks the board\'s rules' if ev.get('bad') else ''))
        if kind == 'no_plan':
            return f"the model has no plan for {', '.join(ev.get('buses') or [])}"
        if kind == 'done':
            return (f"the ends model done in {g('secs')} s"
                    + (f"; refused: {'; '.join(ev['refused'])}" if ev.get('refused') else ''))
        if kind == 'end':
            return (f"the ends model ended in {g('secs')} s" if not ev.get('exit')
                    else f"the ends model FAILED (exit {ev.get('exit')}) -- {ev.get('log')}")
    if st == 'route' and kind == 'start':
        return (f"routing {', '.join(ev.get('buses') or [])} on the field"
                + (', negotiated' if ev.get('negotiated') else ''))
    if st == 'merge' and kind == 'end':
        return f"merged: {g('conflicts')} conflict(s) between buses"
    if st == 'grade' and kind == 'end':
        return f"graded on the board: {g('open')} open, DRC {g('drc')}, {g('walled')} walled"
    if st == 'coupled':
        if kind == 'search':
            if ev.get('status') == 'NONE':
                return f"coupled solve round {g('round')}: no coupling broken -- each bus keeps its own plan"
            return (f"coupled solve round {g('round')}: [{g('status')}] {g('secs')} s"
                    + (f", {ev['cost']} vias-worth (bound {ev.get('bound')}), {ev.get('broken')} of "
                       f"{ev.get('couplings')} couplings broken" if ev.get('cost') is not None else ''))
        if kind == 'start':
            return f"waiting for round {g('round')}'s coupled solve"
        if kind == 'end':
            return (f"coupled plan taken ({g('secs')} s)" if ev.get('joint') and not ev.get('own')
                    else f"its own plan kept ({g('secs')} s)")
    if st == 'bus':
        if kind == 'start':
            return f"{g('nets')} nets, K={g('k')}"
        if kind == 'graded':
            return (f"on the board: vias {g('vias')}, copper {g('copper_mm')} mm, "
                    f"{'connected' if ev.get('connected') else 'NOT connected'}, DRC {g('drc')}, "
                    f"{g('walled')} walled")
        if kind == 'end':
            if ev.get('phase') == 'capture':
                return 'capture run ended'
            return (f"handed on routed: {g('routed')} nets ({g('secs')} s)" if ev.get('exit') == 0
                    else f"left as it came: {ev.get('why') or 'not routed'}")
        if kind == 'exit':
            return f"its process ended, exit {g('exit')} after {g('secs')} s"
    if st == 'round' and kind == 'start':
        return f"round {g('round')}" + (' (feedback)' if ev.get('feedback') else '')
    if st in ('fanout', 'loop'):
        if kind == 'start':
            return f"{_STAGE_NAMES[st]} r{g('round')}" + (f" again on {ev['again']}" if ev.get('again') else '')
        return f"{_STAGE_NAMES[st]} r{g('round')}: exit {g('exit')} at {g('secs')} s"
    if st == 'solve':
        if kind == 'start':
            return f"solve r{g('round')}"
        return (f"solve: [{g('status')}] {g('secs')} s, {g('crossings')} crossings"
                + (f", obj {ev['obj']} bound {ev.get('bound')}" if ev.get('obj') is not None else '')
                + ('' if ev.get('plan', True) else ' -- no plan'))
    if st == 'lay':
        if kind == 'start':
            return f"lay r{g('round')}"
        if not ev.get('laid', True):
            return f"lay r{g('round')}: nothing laid"
        return (f"laid: lanes {g('lanes')}, vias {g('vias')}, copper {g('copper_mm')} mm, "
                f"open {g('open')}, DRC {g('drc')}")
    return f"{st} {kind}"


class _Row(dict):
    """A display row: key, level (0 group, 1 bus, 2 stage), label, state,
    t0, t1, result, log, details (list of text), parent (row key)."""


class RunModel:
    """The events of one run, read into rows (see module docstring)."""

    def __init__(self):
        self.rows = {}              # key -> _Row
        self.order = []             # group keys in order
        self.children = {}          # key -> [child keys]
        self.t_first = None
        self.t_last = None
        self.latest = None
        self.step = 'pending'       # pending | running | done
        self.status = None          # the step's status (step end)
        self.out = None
        self.plan = None            # {'buses', 'left'} when the step found them
        self.planned = {}           # group -> stages it will run
        self.events = 0

    # ---- rows
    def _row(self, key, level, label, parent=None):
        r = self.rows.get(key)
        if r is None:
            r = _Row(key=key, level=level, label=label, state='pending', t0=None,
                     t1=None, result='', log=None, details=[], parent=parent)
            self.rows[key] = r
            if parent is None:
                self.order.append(key)
                self.planned.setdefault(key, len(GROUP_STAGES))
            else:
                self.children.setdefault(parent, []).append(key)
        return r

    def _group(self, src):
        return self._row(('g', src), 0, f"{src} group", None)

    def _bus(self, spec):
        src = spec.split(':', 1)[0]
        g = self._group(src)
        return self._row(('b', spec), 1, spec.replace(':', ' -> '), g['key'])

    def _stage_rows(self, owner):
        return [self.rows[k] for k in self.children.get(owner['key'], ())
                if self.rows[k]['level'] == 2]

    def _open(self, owner, stage, ev, label=None):
        n = len(self.children.get(owner['key'], ()))
        if label is None:
            label = (_STAGE_NAMES.get(stage, stage)
                     + (f" r{ev['round']}" if ev.get('round') else '')
                     + (f" (again, on {ev['again']})" if ev.get('again') else ''))
        r = self._row((owner['key'], stage, n), 2, label, owner['key'])
        r.update(stage=stage, state='running', t0=ev.get('t'), log=ev.get('log'))
        return r

    def _close(self, owner, stage, ev, state='done', result=None, create=True):
        """The latest open row of STAGE under OWNER closed; a closed one made
        when none is open (CREATE)."""
        for r in reversed(self._stage_rows(owner)):
            if r.get('stage') == stage and r['state'] == 'running':
                break
        else:
            if not create:
                return None
            r = self._open(owner, stage, ev)
            r['t0'] = None
        r['state'] = state
        r['t1'] = ev.get('t')
        r['result'] = describe(ev) if result is None else result
        if ev.get('log'):
            r['log'] = ev['log']
        if ev.get('secs') is not None and r['t0'] is None:
            r['secs'] = ev['secs']
        return r

    def _cut_off(self, owner, t, why):
        for r in self._stage_rows(owner):
            if r['state'] == 'running':
                r['state'] = 'ended'
                r['t1'] = t
                r['result'] = r['result'] or why

    # ---- events
    def feed_all(self, events):
        for ev in events:
            self.feed(ev)

    def feed(self, ev):
        self.events += 1
        t = ev.get('t')
        if self.t_first is None:
            self.t_first = t
        self.t_last = t
        self.latest = ev
        st, kind = ev.get('stage'), ev.get('kind')
        grp, bus = ev.get('group'), ev.get('bus')
        if st == 'step':
            if kind == 'start':
                self.step = 'running'
                for spec in ev.get('buses') or ():
                    self._bus(spec)
            elif kind == 'plan':
                self.plan = {'buses': ev.get('buses') or [], 'left': ev.get('left') or []}
                for b in self.plan['buses']:
                    r = self._bus(f"{b['src']}:{b['dest']}")
                    r['nets'] = b.get('nets')
                    r['result'] = r['result'] or f"{b.get('nets')} nets -- {b.get('why')}"
            elif kind == 'end':
                self.step = 'done'
                self.status = ev.get('status')
                self.out = ev.get('out')
                for key in self.order:
                    self._finish_group(self.rows[key], t, 'the step ended')
            return
        if bus:
            owner = self._bus(bus)
        elif grp:
            owner = self._group(grp)
        else:
            return
        if st == 'group':
            g = self._group(grp)
            if kind == 'start':
                g.update(state='running', t0=t, log=ev.get('work'))
                for spec in ev.get('buses') or ():
                    self._bus(spec)
            elif kind == 'round':
                g['details'].append(describe(ev))
            elif kind == 'end':
                g['result'] = describe(ev)
                self._finish_group(g, t, 'the group ended', ok=ev.get('exit') in (0, 1))
            return
        if ev.get('phase') == 'capture':
            # (a bus's capture run: its stages are its ends model's, shown
            # under its capture row; the run is stopped once the model stands)
            cap = next((r for r in reversed(self._stage_rows(owner))
                        if r.get('stage') == 'capture'), None)
            if cap is not None and st not in ('bus',):
                cap['details'].append(describe(ev))
            return
        if bus is None:
            self._group_event(owner, st, kind, ev)
        else:
            self._bus_event(owner, st, kind, ev)

    def _group_event(self, g, st, kind, ev):
        t = ev.get('t')
        if st == 'capture' and kind == 'start':
            self._open(g, 'capture', ev, label=_STAGE_NAMES['capture'])
            for spec in ev.get('buses') or ():
                b = self._bus(spec)
                b.update(state='running', t0=b['t0'] or t)
                self._open(b, 'capture', ev, label=_STAGE_NAMES['capture'])
        elif st == 'ends':
            if kind == 'start':
                if (ev.get('round') or 1) > 1:
                    self.planned[g['key']] = self.planned.get(g['key'], len(GROUP_STAGES)) + len(ROUND_STAGES)
                self._open(g, 'ends', ev, label=f"{_STAGE_NAMES['ends']}, round {ev.get('round', 1)}")
            elif kind == 'end':
                self._close(g, 'ends', ev, state='done' if not ev.get('exit') else 'failed')
            else:
                r = next((r for r in reversed(self._stage_rows(g))
                          if r.get('stage') == 'ends' and r['state'] == 'running'), None)
                if r is not None:
                    r['details'].append(describe(ev))
                    if kind in ('solve', 'field', 'check', 'no_plan'):
                        r['result'] = describe(ev)
        elif st == 'route' and kind == 'start':
            r = self._open(g, 'route', ev, label=_STAGE_NAMES['route']
                           + (', negotiated' if ev.get('negotiated') else ''))
            r['buses'] = list(ev.get('buses') or [])
            r['exited'] = []
            r['result'] = f"0 of {len(r['buses'])} buses finished"
            for spec in r['buses']:
                b = self._bus(spec)
                b.update(state='running')
        elif st == 'coupled' and kind == 'search':
            r = next((r for r in reversed(self._stage_rows(g))
                      if r.get('stage') == 'route'), None)
            (r or g)['details'].append(describe(ev))
        elif st in ('merge', 'grade') and kind == 'end':
            self._close(g, 'route', ev, result=None, create=False)
            self._close(g, st, ev)
        else:
            g['details'].append(describe(ev))

    def _bus_event(self, b, st, kind, ev):
        t = ev.get('t')
        g = self.rows[b['parent']]
        if st == 'capture' and kind == 'end':
            self._close(b, 'capture', ev, state='done' if ev.get('ok') else 'failed')
            caps = [self._stage_rows(self.rows[k]) for k in self.children.get(g['key'], ())
                    if self.rows[k]['level'] == 1]
            if all(any(r.get('stage') == 'capture' and r['state'] != 'running' for r in rows)
                   for rows in caps if rows):
                self._close(g, 'capture', ev, result=f"{len(caps)} bus(es) captured", create=False)
            return
        if st == 'bus':
            if kind == 'start':
                b.update(state='running', nets=ev.get('nets'), work=ev.get('work'))
                b['details'].append(describe(ev))
            elif kind == 'graded':
                b['grade'] = {k: ev.get(k) for k in ('vias', 'copper_mm', 'connected', 'drc', 'walled')}
                b['details'].append(describe(ev))
            elif kind == 'end':
                b['exit'] = ev.get('exit')
                b['why'] = ev.get('why')
                b['details'].append(describe(ev))
            elif kind == 'exit':
                self._cut_off(b, t, 'its process ended')
                b['t1'] = t
                code = b.get('exit', ev.get('exit'))
                b['state'] = ('done' if code == 0 else 'not routed' if code == 1
                              else 'failed')
                if ev.get('exit') not in (0, None) and 'exit' not in b:
                    b['state'] = 'failed'
                b['log'] = ev.get('log')
                b['result'] = self.bus_result(b)
                r = next((r for r in reversed(self._stage_rows(g))
                          if r.get('stage') == 'route' and r['state'] == 'running'), None)
                if r is not None:
                    r['exited'].append(b['key'][1])
                    r['result'] = f"{len(r['exited'])} of {len(r['buses'])} buses finished"
                    if len(r['exited']) >= len(r['buses']):
                        self._close(g, 'route', ev, result=r['result'])
            return
        if st == 'round':
            b['round'] = ev.get('round')
            return
        if kind == 'start':
            self._open(b, st, ev)
        elif kind == 'end':
            if st == 'solve' and not any(r.get('stage') == 'solve' and r['state'] == 'running'
                                         for r in self._stage_rows(b)):
                # (a re-solve inside the loop: the loop's detail)
                r = next((r for r in reversed(self._stage_rows(b)) if r['state'] == 'running'), None)
                if r is not None:
                    r['details'].append('re-solve: ' + describe(ev))
                    return
            r = self._close(b, st, ev, state='done' if not ev.get('exit') else 'done')
            if st == 'lay' and ev.get('laid', True):
                b['laid'] = {k: ev.get(k) for k in ('lanes', 'vias', 'copper_mm', 'open', 'drc')}
            if st == 'solve' and ev.get('plan') is False:
                r['state'] = 'failed'

    def _finish_group(self, g, t, why, ok=True):
        if g['state'] in ('pending', 'running'):
            g['state'] = 'done' if ok else 'failed'
        g['t1'] = g['t1'] or t
        self._cut_off(g, t, why)
        for k in self.children.get(g['key'], ()):
            b = self.rows[k]
            if b['level'] == 1:
                self._cut_off(b, t, why)
                if b['state'] == 'running':
                    b['state'] = 'ended'
        # a group's stages that never ran will not: the gauge counts what ran
        self.planned[g['key']] = len([r for r in self._stage_rows(g)])

    def close(self, t, why):
        """The run's process ended (or was stopped): every row still running
        is ENDED at T, with WHY where it has no result -- no event will end
        it now."""
        for r in self.rows.values():
            if r['state'] in ('running',):
                r['state'] = 'ended'
                r['t1'] = r['t1'] or t
                r['result'] = r['result'] or why
        if self.step == 'running':
            self.step = 'ended'

    # ---- views
    def bus_result(self, b):
        parts = []
        lay = b.get('laid') or {}
        if lay.get('lanes'):
            parts.append(f"lanes {lay['lanes']}")
        gr = b.get('grade') or {}
        if gr:
            parts.append(f"vias {gr.get('vias')}, copper {gr.get('copper_mm')} mm, "
                         f"open {lay.get('open', '?')}, DRC {gr.get('drc')}, walled {gr.get('walled')}")
        elif lay:
            parts.append(f"vias {lay.get('vias')}, copper {lay.get('copper_mm')} mm, "
                         f"open {lay.get('open')}, DRC {lay.get('drc')}")
        if b.get('exit') == 1:
            parts.append('left as it came' + (f": {b['why']}" if b.get('why') else ''))
        return '; '.join(parts) or b.get('result', '')

    def flat(self):
        """Every row, in display order: each group, its stages and its buses,
        each bus followed by its stages."""
        out = []

        def walk(key):
            out.append(self.rows[key])
            kids = self.children.get(key, ())
            for k in kids:
                if self.rows[k]['level'] == 2:
                    out.append(self.rows[k])
            for k in kids:
                if self.rows[k]['level'] == 1:
                    walk(k)
        for key in self.order:
            walk(key)
        return out

    def elapsed(self, row, now):
        if row.get('t0') is None:
            return row.get('secs')
        return (row.get('t1') or now) - row['t0']

    def fractions(self):
        """{'stages': (ended, will run), 'buses': (finished, routing)}: the
        groups' stages ended of those that will run, and the routing stage's
        buses finished of its buses (the latest routing stage)."""
        done = sum(1 for k in self.order for r in self._stage_rows(self.rows[k])
                   if r['state'] not in ('running', 'pending'))
        total = sum(self.planned.get(k, len(GROUP_STAGES)) for k in self.order)
        route = None
        for k in self.order:
            for r in self._stage_rows(self.rows[k]):
                if r.get('stage') == 'route':
                    route = r
        buses = (len(route['exited']), len(route['buses'])) if route else (0, 0)
        return {'stages': (done, max(total, done)), 'buses': buses}

    def status_line(self, now):
        if self.latest is None:
            return ''
        ev = self.latest
        who = ev.get('bus') or ev.get('group') or 'step'
        at = _fmt_secs((ev.get('t') or now) - (self.t_first or now))
        return f"[{at}] {who}: {describe(ev)}"

    def summary(self):
        """{bus spec: result text} for the buses that ended."""
        return {r['key'][1]: r['result'] for r in self.rows.values()
                if r['level'] == 1 and r['state'] not in ('pending', 'running')}
