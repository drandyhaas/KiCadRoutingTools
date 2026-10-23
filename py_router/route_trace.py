"""Route trace (KICAD_ROUTE_TRACE=1): every track and via as the router lays,
rips and restores it, in order -- what `route_movie.py` animates (#482).

Every commit in the engine goes through `add_route_to_pcb_data` and every rip
through `remove_route_from_pcb_data` (pcb_modification.py); a restore re-enters
through the add, labelled 'restore'. Each call records one event here.
`batch_route` and `batch_route_diff_pairs` attach the recorder after parsing
and write `<output>_routetrace.json` at the end. A call that writes no board
(the GUI, an internal reconnect pass) records nothing.

Off unless the variable is set. Recording never changes copper, and a failure
here never stops a route. Copper that reaches the board some other way (the
plane finalize, the end-of-run clean-ups) is not in the trace: the movie
squares each step with the board that step wrote.

File: {'layers': [...], 'board_bounds': [...], 'events': [{'seq', 'event'
('route' | 'restore' | 'rip'), 'net', 'net_name', 'add_s' | 'del_s': [[x1, y1,
x2, y2, width, layer index]], 'add_v' | 'del_v': [[x, y, size, drill, first
layer index, last layer index]]}]}.
"""
from __future__ import annotations

import json
import os


def enabled() -> bool:
    return os.environ.get('KICAD_ROUTE_TRACE', '') in ('1', 'true', 'on')


def attach(pcb_data, output_file) -> None:
    """Start recording on `pcb_data` when the variable is set and the run
    writes a board."""
    if enabled() and output_file and getattr(pcb_data, '_route_trace', None) is None:
        try:
            pcb_data._route_trace = {'layers': list(pcb_data.board_info.copper_layers),
                                     'events': []}
        except Exception:
            pass


def seg_row(s, li):
    return [round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
            round(s.end_y, 4), round(s.width, 4), li.get(s.layer, 0)]


def via_row(v, li, last):
    ls = getattr(v, 'layers', None) or []
    return [round(v.x, 4), round(v.y, 4), round(v.size, 4), round(v.drill, 4),
            li.get(ls[0], 0) if ls else 0, li.get(ls[-1], last) if ls else last]


def record(pcb_data, segments, vias, result, event) -> None:
    """One event: `segments`/`vias` added ('route', 'restore') or removed
    ('rip')."""
    tr = getattr(pcb_data, '_route_trace', None)
    if tr is None or not (segments or vias):
        return
    try:
        li = {n: i for i, n in enumerate(tr['layers'])}
        key = 'del' if event == 'rip' else 'add'
        ev = {'seq': len(tr['events']) + 1, 'event': event}
        ids = [result.get('net_id')] if isinstance(result, dict) and result.get('net_id') else []
        for s in segments or ():
            if s.net_id not in ids and len(ids) < 2:
                ids.append(s.net_id)
        if ids:
            ev['net'] = int(ids[0])
            ev['net_name'] = ' / '.join(pcb_data.nets[n].name for n in ids if n in pcb_data.nets)
        if segments:
            ev[key + '_s'] = [seg_row(s, li) for s in segments]
        if vias:
            ev[key + '_v'] = [via_row(v, li, len(li) - 1) for v in vias]
        tr['events'].append(ev)
    except Exception:
        pass


def dump(pcb_data, output_file) -> None:
    """Write `<output>_routetrace.json` when the run recorded an event. A
    nested call writing the same board (the final reconciliation) writes
    first; the outer run's trace replaces it."""
    tr = getattr(pcb_data, '_route_trace', None)
    if tr is None or not output_file or not tr['events']:
        return
    try:
        bounds = pcb_data.board_info.board_bounds
        path = os.path.splitext(output_file)[0] + '_routetrace.json'
        with open(path, 'w') as f:
            json.dump({'layers': tr['layers'], 'board_bounds': list(bounds) if bounds else None,
                       'events': tr['events']}, f)
        print(f"  Route trace written to {path} ({len(tr['events'])} events)")
    except Exception as e:
        print(f"  route trace not written: {e}")
