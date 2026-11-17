"""bus_events.py -- the bus step's PROGRESS EVENTS: what route_bus.py is doing, as it does it, for a reader that follows
the step while it runs (the GUI's Bus tab, a person with `tail -f`).

One JSON object a line, appended to the file the BUS_EVENTS setting names. The step sets it (run: <OUT stem>.bus/
events.jsonl, started empty) unless the caller already did, and every stage's process inherits it with the rest of the
step's settings, so the capture processes, the group's model and each bus's own process all write to the one file.
With no BUS_EVENTS nothing is written: a stage run by itself, a bench, a harness.

    {"v": 1, "t": <epoch s>, "pid": <process>, "stage": "solve", "kind": "end", "group": "U1", "bus": "U1:U2",
     "phase": "capture", ...payload}

  stage, kind  what happened: a stage's "start" and "end", or another kind of its own (stage "ends", kind "solve": one
               solve of the group's model). The step's stages and their payloads: docs/bus-routing.md, "Progress
               events"
  group        the source part: each source's buses are one group (route_bus.route_buses)
  bus          "SRC:DEST" as --bus names it (DEST may be U4+U3), or null for the group's own events
  phase        "capture" in a bus's ends-model capture (route_bus under ENDS_ONLY), whose process is stopped as soon as
               its capture stands; absent otherwise

A long CP-SAT solve says how far it has come (solver_progress: kind "progress", the best plan's cost and the bound the
search has proved, from the solver's own progress lines -- the gap between them closes as the search proves its plan).

A process routing one bus says so once (for_bus: the BUS_EVENTS_BUS setting, beside its other settings), and its
events -- and its stages' -- carry that bus unless they name another. An appended line is ONE os.write on a descriptor
opened O_APPEND, so the lines of the step's several processes interleave whole. A write that fails is dropped: an
event never fails a run, and nothing here is read back by the routing.
"""
import contextlib
import json
import math
import os
import re
import time

import awx_settings

V = 1
ENV = 'BUS_EVENTS'              # the events file
BUS = 'BUS_EVENTS_BUS'          # the bus this process routes, SRC:DEST
NAME = 'events.jsonl'
_FROM_SETTINGS = object()       # (emit's group / bus not given: the process's own, from BUS)


def work_dir(out):
    """the work directory of a step writing `out`: <OUT stem>.bus/"""
    out = os.path.abspath(out)
    if out.endswith('.kicad_pcb'):
        out = out[:-len('.kicad_pcb')]
    return out + '.bus'


def default_path(out):
    """the events file of a step writing `out`: <OUT stem>.bus/events.jsonl"""
    return os.path.join(work_dir(out), NAME)


@contextlib.contextmanager
def run(work):
    """The step's events file for the block: the one the caller named (BUS_EVENTS given: appended to), else
    WORK/events.jsonl, started empty and given as BUS_EVENTS to everything inside -- the step's own events and every
    stage process it starts. Yields the path"""
    given = awx_settings.get(ENV)
    if given:
        yield given
        return
    path = os.path.join(os.path.abspath(work), NAME)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    try:
        if os.path.exists(path):
            os.remove(path)
    except OSError:
        pass
    with awx_settings.given({**awx_settings.environ(), ENV: path}):
        yield path


def for_bus(src, dest):
    """the setting that says a process routes the bus SRC:DEST: its events, and its stages', carry it"""
    return {BUS: f'{src}:{dest}'}


_GROUP = [None]            # (for_group: the group this process works for)


def for_group(src):
    """This process works for the group of buses off SRC (the group's ends model): its events that name no bus and no
    group carry it"""
    _GROUP[0] = src


def emit(stage, kind, group=_FROM_SETTINGS, bus=_FROM_SETTINGS, **payload):
    """one event, appended to BUS_EVENTS (nothing when it is unset). `bus` not given: the process's own (BUS); given
    (None included), as given. `group` not given: the bus's source, else the process's own bus's, else the group the
    process works for (for_group)"""
    path = awx_settings.get(ENV)
    if not path:
        return
    own = awx_settings.get(BUS) or None
    if bus is _FROM_SETTINGS:
        bus = own
    if group is _FROM_SETTINGS:
        group = (bus or own).split(':', 1)[0] if (bus or own) else _GROUP[0]
    rec = {'v': V, 't': round(time.time(), 3), 'pid': os.getpid(), 'stage': stage, 'kind': kind,
           'group': group, 'bus': bus}
    if bus and awx_settings.get('ENDS_ONLY'):
        rec['phase'] = 'capture'
    rec.update(payload)
    try:
        line = (json.dumps(rec, default=str, separators=(',', ':')) + '\n').encode('utf-8')
        fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT | getattr(os, 'O_BINARY', 0), 0o644)
        try:
            os.write(fd, line)
        finally:
            os.close(fd)
    except (OSError, TypeError, ValueError):
        pass


def read(path, offset=0):
    """(events, offset): the whole lines of `path` from byte `offset` on, each parsed (a line that is not a JSON object
    skipped), and the offset after the last whole line -- a line still being written is left for the next read"""
    try:
        with open(path, 'rb') as f:
            f.seek(offset)
            data = f.read()
    except OSError:
        return [], offset
    end = data.rfind(b'\n')
    if end < 0:
        return [], offset
    out = []
    for ln in data[:end].split(b'\n'):
        try:
            ev = json.loads(ln.decode('utf-8', errors='replace'))
        except ValueError:
            continue
        if isinstance(ev, dict):
            out.append(ev)
    return out, offset + end + 1


_PROGRESS = re.compile(r'^#(?:Bound|\d+)\s.*?best:(\S+)\s+next:\[([^,\]]+)')


def solver_progress(stage, label, scale=1.0, every=5.0, then=None, bus=_FROM_SETTINGS):
    """A CP-SAT log callback's reader (the solver's parameters log_search_progress on, log_to_stdout off): each of the
    search's progress lines -- '#Bound' and '#n', 'best:B next:[L,U]' -- becomes a "progress" event of STAGE: LABEL,
    the best plan's cost and the proven bound, both over SCALE (a via's worth), and the seconds since the reader was
    made. A first plan, and a better one, at once (better ones a fifth of a second apart at most), else at most one
    every EVERY seconds. THEN, a callable, is given
    every line after (the solve's own reader); BUS as emit takes it. Nothing here touches the search: it reads the lines the solver writes.
    Returns the callback."""
    st = {'t0': time.time(), 'last': -math.inf, 'best': None}

    def feed(line):
        if then is not None:
            then(line)
        m = _PROGRESS.match(line)
        if not m or not awx_settings.get(ENV):
            return
        try:
            best, lo = float(m.group(1)), float(m.group(2))
        except ValueError:
            return
        best = best if math.isfinite(best) else None
        now = time.time()
        first = best is not None and st['best'] is None
        better = best is not None and st['best'] is not None and best < st['best']
        if not first and now - st['last'] < (0.2 if better else every):
            return
        st['last'], st['best'] = now, best
        emit(stage, 'progress', bus=bus, label=label, best=None if best is None else round(best / scale, 1),
             bound=round(lo / scale, 1) if math.isfinite(lo) else None, secs=round(now - st['t0']))
    return feed


def watch_solve(solver, stage, label, scale=1.0, bus=_FROM_SETTINGS):
    """SOLVER (a cp_model.CpSolver with no log callback of its own) reports its search's progress as STAGE's
    "progress" events (solver_progress); nothing without BUS_EVENTS. A solver with a reader of its own is given
    solver_progress(..., then=its reader) instead"""
    if awx_settings.get(ENV):
        solver.parameters.log_search_progress = True
        solver.parameters.log_to_stdout = False
        solver.log_callback = solver_progress(stage, label, scale, bus=bus)
    return solver
