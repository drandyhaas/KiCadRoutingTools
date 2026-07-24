"""Route trace (KICAD_ROUTE_TRACE=1): a time-ordered record of every track
segment and via as it is committed, ripped up, and restored during a routing
run -- the data an animation of the routing process needs (issue #482).

Design: **direct recording at the two copper choke points.** Every commit in
the engine funnels through ``add_route_to_pcb_data`` and every rip through
``remove_route_from_pcb_data`` (both in ``pcb_modification.py``); restores
re-enter through the add choke point. Those functions hold the actual
``Segment``/``Via`` objects with full geometry, so each call appends one
event with exactly what was added or removed, in true call order. This is
why PARTIAL multipoint work falls out for free: taps are committed
incrementally (one add event per tap slice) and a piece-level #134 restore
re-adds only its subset -- each is its own event, no diffing or guessing
which edges changed.

Ordering is a per-record monotonic counter (``seq``); the engine's
``route_index`` is only per-net, too coarse for a per-copper timeline. Router
copper has no stable ``uuid`` during the run (minted at write time), so we
serialize geometry, not identity -- which is all the animator needs.

The recorder is read-only over engine state and, like net_story, is
default-OFF (no cost on normal runs). ``batch_route`` attaches it as
``pcb_data._route_trace`` and writes ``<output_basename>_routetrace.json``.

Consumed by ``animate_route.py``.
"""
from __future__ import annotations

import json
import os
from typing import Any, Dict, List, Optional, Tuple


def route_trace_enabled() -> bool:
    return os.environ.get('KICAD_ROUTE_TRACE', '') in ('1', 'true', 'on')


def attach_trace(pcb_data) -> None:
    """If KICAD_ROUTE_TRACE=1, attach a RouteTrace to ``pcb_data`` so the copper
    choke points record into it. Idempotent; no-op when disabled. Call once,
    right after parsing the board, in each routing front-end."""
    if route_trace_enabled() and getattr(pcb_data, '_route_trace', None) is None:
        try:
            pcb_data._route_trace = RouteTrace(list(pcb_data.board_info.copper_layers))
        except Exception:
            pass


def dump_trace(pcb_data, output_file: str) -> None:
    """Dump ``pcb_data``'s attached RouteTrace next to ``output_file``. No-op if
    none attached or no output path (e.g. GUI dry-run / an internal reconnect
    batch_route called with output_file=""). Best-effort."""
    rt = getattr(pcb_data, '_route_trace', None)
    if rt is not None and output_file:
        try:
            rt.dump(output_file, pcb_data)
        except Exception as e:
            print(f"  route trace dump failed: {e}")


def start_plane_trace(pcb_data, output_file: str):
    """A LOCAL RouteTrace for the plane front-ends (route_planes /
    route_disconnected_planes), which add copper OUTSIDE the choke points and
    call batch_route internally. It is deliberately NOT attached to
    ``pcb_data._route_trace`` -- so a nested batch_route neither records into it
    nor sees it -- and is driven purely by :meth:`RouteTrace.capture`. Returns
    the trace (baseline captured) or None when disabled / no output path."""
    if not (route_trace_enabled() and output_file):
        return None
    try:
        t = RouteTrace(list(pcb_data.board_info.copper_layers))
        t.capture(pcb_data, 'preexisting')   # baseline: the input board's copper
        return t
    except Exception:
        return None


class RouteTrace:
    """Accumulates add/remove events recorded directly at the copper choke
    points. One instance per run, attached as ``pcb_data._route_trace``. The
    choke points call :meth:`record_add` / :meth:`record_remove`; the engine
    calls :meth:`dump` at end of run."""

    def __init__(self, layers: List[str]):
        self.layers = list(layers)
        self._li = {name: i for i, name in enumerate(self.layers)}
        self.events: List[Dict[str, Any]] = []
        self._seq = 0
        # snapshot-diff state (for capture(); front-ends without a choke point)
        self._snap_s: Dict[Tuple, None] = {}
        self._snap_v: Dict[Tuple, None] = {}

    # -- serialization (compact rows; layer names -> indices) -----------
    def _ser_seg(self, s) -> List:
        return [round(s.start_x, 4), round(s.start_y, 4),
                round(s.end_x, 4), round(s.end_y, 4),
                round(s.width, 4), self._li.get(s.layer, 0)]

    def _ser_via(self, v) -> List:
        ls = getattr(v, 'layers', None) or []
        a = self._li.get(ls[0], 0) if ls else 0
        b = self._li.get(ls[-1], len(self.layers) - 1) if ls else len(self.layers) - 1
        return [round(v.x, 4), round(v.y, 4), round(v.size, 4),
                round(v.drill, 4), a, b]

    def _event(self, event: str, key: str, seg_rows: List, via_rows: List,
               net_id: Optional[int], net_name: str, by: Optional[str]) -> None:
        if not seg_rows and not via_rows:
            return
        self._seq += 1
        ev: Dict[str, Any] = {'seq': self._seq, 'event': event}
        if net_id is not None:
            try:
                ev['net'] = int(net_id)
            except (TypeError, ValueError):
                pass
        if net_name:
            ev['net_name'] = net_name
        if by:
            ev['by'] = by
        if seg_rows:
            ev[key + '_s'] = seg_rows
        if via_rows:
            ev[key + '_v'] = via_rows
        self.events.append(ev)

    # -- recording (called from the pcb_modification choke points) ------
    def record_add(self, segments, vias, net_id: Optional[int] = None,
                   net_name: str = '', event: str = 'route') -> None:
        self._event(event, 'add',
                    [self._ser_seg(s) for s in (segments or [])],
                    [self._ser_via(v) for v in (vias or [])],
                    net_id, net_name, None)

    def record_remove(self, segments, vias, net_id: Optional[int] = None,
                      net_name: str = '', event: str = 'rip',
                      by: Optional[str] = None) -> None:
        self._event(event, 'del',
                    [self._ser_seg(s) for s in (segments or [])],
                    [self._ser_via(v) for v in (vias or [])],
                    net_id, net_name, by)

    # -- recording from copper DICTS (route_planes builds tap copper as dicts,
    #    {'start','end','width','layer','net_id'} / {'x','y','size','drill',
    #    'layers','net_id'}, never touching pcb_data.segments) --------------
    def _seg_row_from_dict(self, d) -> List:
        s, e = d['start'], d['end']
        return [round(s[0], 4), round(s[1], 4), round(e[0], 4), round(e[1], 4),
                round(d.get('width', 0.1), 4), self._li.get(d.get('layer'), 0)]

    def _via_row_from_dict(self, d) -> List:
        ls = d.get('layers') or []
        a = self._li.get(ls[0], 0) if ls else 0
        b = self._li.get(ls[-1], len(self.layers) - 1) if ls else len(self.layers) - 1
        return [round(d['x'], 4), round(d['y'], 4), round(d.get('size', 0), 4),
                round(d.get('drill', 0), 4), a, b]

    def record_dicts(self, seg_dicts, via_dicts, event: str = 'plane-tap',
                     net_id: Optional[int] = None, net_name: str = '') -> None:
        rows_s, rows_v = [], []
        for d in (seg_dicts or []):
            try:
                rows_s.append(self._seg_row_from_dict(d))
            except Exception:
                pass
        for d in (via_dicts or []):
            try:
                rows_v.append(self._via_row_from_dict(d))
            except Exception:
                pass
        self._event(event, 'add', rows_s, rows_v, net_id, net_name, None)

    # -- snapshot-diff (front-ends without a choke point) ---------------
    def capture(self, pcb_data, event: str = 'plane', net_id: Optional[int] = None,
                net_name: str = '', by: Optional[str] = None) -> None:
        """Snapshot ``pcb_data`` copper, diff against the previous capture, and
        emit the delta (segments/vias added and removed since) as one event.

        For the plane front-ends (route_planes / route_disconnected_planes),
        whose taps, joins and blocker rips mutate ``pcb_data.segments``/``vias``
        directly rather than through ``add_route_to_pcb_data``. Call once right
        after parsing (a 'preexisting' baseline of the input copper) and then at
        each per-net / per-region boundary; diffing yields exactly what that
        step added or ripped, including partial restores."""
        cur_s: Dict[Tuple, None] = {}
        for s in getattr(pcb_data, 'segments', None) or []:
            if s.layer in self._li:
                cur_s[tuple(self._ser_seg(s))] = None
        cur_v: Dict[Tuple, None] = {tuple(self._ser_via(v)): None
                                    for v in (getattr(pcb_data, 'vias', None) or [])}
        add_s = [list(k) for k in cur_s if k not in self._snap_s]
        del_s = [list(k) for k in self._snap_s if k not in cur_s]
        add_v = [list(k) for k in cur_v if k not in self._snap_v]
        del_v = [list(k) for k in self._snap_v if k not in cur_v]
        self._snap_s, self._snap_v = cur_s, cur_v
        if not (add_s or del_s or add_v or del_v):
            return
        self._seq += 1
        ev: Dict[str, Any] = {'seq': self._seq, 'event': event}
        if net_id is not None:
            try:
                ev['net'] = int(net_id)
            except (TypeError, ValueError):
                pass
        if net_name:
            ev['net_name'] = net_name
        if by:
            ev['by'] = by
        if add_s:
            ev['add_s'] = add_s
        if del_s:
            ev['del_s'] = del_s
        if add_v:
            ev['add_v'] = add_v
        if del_v:
            ev['del_v'] = del_v
        self.events.append(ev)

    # -- finalize --------------------------------------------------------
    def finalize(self, pcb_data) -> None:
        """Reconcile the trace's cumulative copper with the FINAL board so the
        animation ends exactly on the routed result. Not every removal goes
        through the rip choke point: the end-of-run passes (drop_phantom_copper,
        stale-input strip) drop superseded copper directly. Emit one closing
        'reconcile' delete for trace-live copper absent from the board, and one
        'final' add for board copper the trace never recorded (e.g. pre-existing
        input tracks registered as a pseudo-commit)."""
        live_s: Dict[Tuple, List] = {}
        live_v: Dict[Tuple, List] = {}
        for ev in self.events:
            for r in ev.get('add_s', ()):
                live_s[tuple(r)] = r
            for r in ev.get('del_s', ()):
                live_s.pop(tuple(r), None)
            for r in ev.get('add_v', ()):
                live_v[tuple(r)] = r
            for r in ev.get('del_v', ()):
                live_v.pop(tuple(r), None)
        final_s = {tuple(self._ser_seg(s)) for s in getattr(pcb_data, 'segments', [])}
        final_v = {tuple(self._ser_via(v)) for v in getattr(pcb_data, 'vias', [])}
        extra_s = [list(k) for k in live_s if k not in final_s]
        extra_v = [list(k) for k in live_v if k not in final_v]
        miss_s = [list(k) for k in final_s if k not in live_s]
        miss_v = [list(k) for k in final_v if k not in live_v]
        if extra_s or extra_v:
            self._event('reconcile', 'del', extra_s, extra_v, None, '', None)
        if miss_s or miss_v:
            self._event('final', 'add', miss_s, miss_v, None, '', None)

    # -- output ----------------------------------------------------------
    def dump(self, output_file: str, pcb_data, finalize: bool = True) -> str:
        # finalize=False for dict-sourced traces (route_planes): the tap copper
        # lives in all_new_* dicts, not pcb_data, so a pcb_data diff would wrongly
        # reconcile it away. The whole-run animator trues up to the step board.
        if finalize:
            self.finalize(pcb_data)
        data = {
            'layers': self.layers,
            'board_bounds': list(pcb_data.board_info.board_bounds)
            if pcb_data.board_info.board_bounds else None,
            'events': self.events,
        }
        base, _ = os.path.splitext(output_file or 'routing')
        path = f"{base}_routetrace.json"
        with open(path, 'w') as f:
            json.dump(data, f)
        n_add = sum(len(e.get('add_s', ())) for e in self.events)
        n_del = sum(len(e.get('del_s', ())) for e in self.events)
        print(f"  Route trace written to {path} "
              f"({len(self.events)} events, +{n_add}/-{n_del} segs, KICAD_ROUTE_TRACE)")
        return path


# ---------------------------------------------------------------------------
# Consumer side (used by animate_route.py)
# ---------------------------------------------------------------------------
class _Seg:
    """Lightweight segment adapter with the fields BoardRenderer reads."""
    __slots__ = ('start_x', 'start_y', 'end_x', 'end_y', 'width', 'layer', 'net_id')

    def __init__(self, row, layers):
        (self.start_x, self.start_y, self.end_x, self.end_y,
         self.width, li) = row
        self.layer = layers[int(li)] if 0 <= int(li) < len(layers) else 'F.Cu'
        self.net_id = 0


class _Via:
    __slots__ = ('x', 'y', 'size', 'drill', 'layers', 'net_id')

    def __init__(self, row, layers):
        self.x, self.y, self.size, self.drill, a, b = row
        self.layers = [layers[int(a)] if 0 <= int(a) < len(layers) else 'F.Cu',
                       layers[int(b)] if 0 <= int(b) < len(layers) else 'B.Cu']
        self.net_id = 0


def load_trace(path: str) -> Dict[str, Any]:
    with open(path) as f:
        return json.load(f)


def seg_key_row(row) -> Tuple:
    return (round(row[0], 4), round(row[1], 4), round(row[2], 4),
            round(row[3], 4), round(row[4], 4), int(row[5]))


def via_key_row(row) -> Tuple:
    return (round(row[0], 4), round(row[1], 4), round(row[2], 4),
            round(row[3], 4), int(row[4]), int(row[5]))
