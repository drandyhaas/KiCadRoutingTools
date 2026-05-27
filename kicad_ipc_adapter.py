"""KiCad IPC adapter — centralises all `kipy` (kicad-python) interaction.

This is the only module that imports kipy. Every other file in the plugin
talks to KiCad through the helpers here so the public surface stays small
and any kicad-python API drift only needs to be patched in one place.

Coordinate model
----------------
kipy uses `Vector2` with `from_xy_mm()` / `.x_mm` / `.y_mm` helpers. Internal
units are nanometres but we never expose them; the rest of the codebase deals
in millimetres throughout (see kicad_parser.PCBData).

Layer model
-----------
kipy exposes layers as the `BoardLayer` enum (`BL_F_Cu`, `BL_B_Cu`,
`BL_In1_Cu` … `BL_In30_Cu`, `BL_F_SilkS`, `BL_B_SilkS`, `BL_Edge_Cuts`,
`BL_User_1` … `BL_User_9`). The rest of the codebase uses KiCad's string
layer names ("F.Cu", "In1.Cu", "F.SilkS", ...) so we keep two dicts that
translate either direction.

Nets
----
kipy `Track` / `Via` carry a `.net` attribute that is a `Net` *object*, not a
numeric code. The router speaks in net_id ints, so `NetMap` looks up nets by
id (with name fallback) and caches the mapping.

Commit model
------------
Multiple item creations are batched into a single commit so KiCad treats the
whole route as one undo step:

    with begin_commit(board) as commit:
        commit.add_tracks([...])
        commit.add_vias([...])
        commit.add_shapes([...])
    # commit pushed on context exit; KiCad refreshes automatically
"""

from __future__ import annotations

import threading
from contextlib import contextmanager
from typing import Iterable, Optional


# --- kipy imports -----------------------------------------------------------
# kipy is only installed inside KiCad's per-plugin venv, so we import on
# first use and give a clear error if someone tries to use this module
# outside KiCad. Lazily cached in `_kipy` so subsequent calls are free.

_kipy = None  # set by _ensure_kipy() on first call


def _ensure_kipy():
    """Import kipy once, cache the modules we use, and return the cache."""
    global _kipy
    if _kipy is not None:
        return _kipy
    try:
        import kipy  # noqa: F401  (exposed as _kipy.kipy)
        from kipy import board_types as bt
        from kipy import geometry as geom
    except ImportError as e:
        raise ImportError(
            "kicad-python (kipy) is not installed. This plugin runs as a "
            "KiCad 10 IPC plugin and requires the `kicad-python` package. "
            "If you're running the plugin from KiCad's PCM, KiCad should "
            "provision a venv automatically. If you're running standalone, "
            "`pip install kicad-python>=0.7.0`."
        ) from e

    class _Cache:
        pass
    c = _Cache()
    c.kipy = kipy
    c.BoardLayer = bt.BoardLayer
    c.Track = bt.Track
    c.Via = bt.Via
    c.BoardShape = bt.BoardShape
    c.Vector2 = geom.Vector2
    # Zone-related types are imported lazily on first zone construction
    # so plugins that never touch the planes tab don't trigger the import.
    c._zone_types = None
    _kipy = c
    return _kipy


def _zone_types():
    """Lazy import of zone-related types (Zone, ZoneType, PolyLine, ...).

    Kept separate so callers that only route tracks/vias don't pay the
    import cost or break if a kicad-python release moves zone classes around.
    """
    c = _ensure_kipy()
    if c._zone_types is None:
        from kipy.board_types import Zone, ZoneType
        from kipy.geometry import PolyLine, PolyLineNode, PolygonWithHoles
        c._zone_types = (Zone, ZoneType, PolyLine, PolyLineNode, PolygonWithHoles)
    return c._zone_types


# --- Connection -------------------------------------------------------------
# A single IPC connection is reused across the plugin's lifetime — kipy uses
# a synchronous, single-threaded protobuf socket, and we never spawn multiple
# `KiCad()` clients per process.

_KICAD_SINGLETON = None
# Serialises IPC calls across threads. kipy uses an NNG REQ/REP socket and
# we have no published guarantee that two threads can hit the same client
# concurrently without corrupting the socket. RoutingDialog launches a
# routing worker thread that calls back into IPC (net-class lookup, etc.),
# so wrap every IPC entry point in `with ipc_lock():`.
_IPC_LOCK = threading.RLock()


@contextmanager
def ipc_lock():
    """Context manager that serialises kipy calls across threads."""
    _IPC_LOCK.acquire()
    try:
        yield
    finally:
        _IPC_LOCK.release()


def connect() -> "kipy.KiCad":
    """Open (or reuse) the IPC connection to the running KiCad instance.

    Reads `KICAD_API_SOCKET` and `KICAD_API_TOKEN` from the environment;
    KiCad sets both when it spawns an IPC plugin.
    """
    global _KICAD_SINGLETON
    if _KICAD_SINGLETON is None:
        _KICAD_SINGLETON = _ensure_kipy().kipy.KiCad()
    return _KICAD_SINGLETON


def get_board(kicad: Optional["kipy.KiCad"] = None):
    """Return the currently-open PCB, or None if no board is open."""
    if kicad is None:
        kicad = connect()
    try:
        with ipc_lock():
            return kicad.get_board()
    except Exception:
        return None


def ping(kicad: Optional["kipy.KiCad"] = None, timeout: float = 1.0) -> bool:
    """Lightweight liveness check: returns True if KiCad is still reachable.

    The ping runs on a daemon thread with a hard timeout so a half-closed
    NNG socket (KiCad mid-shutdown) can't deadlock the dialog. If kipy's
    `get_version()` doesn't return within `timeout` seconds we treat it
    as "KiCad gone" — leaking the daemon thread is fine; it dies with
    the process.

    Without this, pinging KiCad while it was tearing down its API server
    caused both processes to wedge: kipy waited for a response that never
    arrived, and KiCad waited for our socket to close cleanly.
    """
    if kicad is None:
        kicad = connect()

    # If someone else holds the lock (a commit is in flight, or a previous
    # ping is still stuck on a half-closed socket), don't queue up — just
    # report "alive" and try again next tick. Wedging here would create a
    # growing pile of daemon threads on every heartbeat.
    if not _IPC_LOCK.acquire(blocking=False):
        return True
    try:
        result: list = [None]

        def _do_ping():
            try:
                # kipy.KiCad ships its own purpose-built ping() — cheaper
                # than get_version() and the call kipy itself recommends
                # for connectivity checks.
                if hasattr(kicad, "ping"):
                    kicad.ping()
                else:
                    kicad.get_version()
                result[0] = True
            except Exception:
                result[0] = False

        t = threading.Thread(target=_do_ping, daemon=True)
        t.start()
        t.join(timeout)
        if result[0] is None:
            # The kipy call is still sitting on the socket. Treat as gone;
            # the daemon thread will exit with the process.
            return False
        return bool(result[0])
    finally:
        try:
            _IPC_LOCK.release()
        except RuntimeError:
            # Lock wasn't held (defensive — shouldn't happen).
            pass


def close_connection(timeout: float = 1.0) -> None:
    """Close the IPC connection, releasing KiCad's end of the socket.

    kicad.close() can itself block when KiCad is mid-shutdown (it tries
    to handshake with a peer that's already tearing down). Run it on a
    daemon thread with a hard timeout so the plugin exit path is never
    held up. Safe to call multiple times.
    """
    global _KICAD_SINGLETON
    if _KICAD_SINGLETON is None:
        return
    target = _KICAD_SINGLETON
    _KICAD_SINGLETON = None

    def _do_close():
        try:
            if hasattr(target, "close"):
                target.close()
        except Exception:
            pass

    t = threading.Thread(target=_do_close, daemon=True)
    t.start()
    t.join(timeout)
    # If t is still alive, we let it die with the process; the OS will
    # drop the socket when this process exits.


def get_board_full_path(kicad: Optional["kipy.KiCad"] = None) -> Optional[str]:
    """Best-effort absolute filesystem path to the open PCB.

    kipy.Board.name only returns the basename. The directory lives on the
    DocumentSpecifier returned by `get_open_documents(DOCTYPE_PCB)` —
    specifically `doc.project.path` on kicad-python 0.7. Returns None if
    the joined path doesn't exist on disk.
    """
    import os
    if kicad is None:
        kicad = connect()
    try:
        from kipy.proto.common.types import DocumentType  # type: ignore
    except Exception:
        return None
    try:
        with ipc_lock():
            docs = list(kicad.get_open_documents(DocumentType.DOCTYPE_PCB))  # type: ignore[attr-defined]
    except Exception:
        return None

    for doc in docs or []:
        basename = getattr(doc, "board_filename", "") or ""
        proj = getattr(doc, "project", None)
        proj_path = getattr(proj, "path", "") if proj is not None else ""
        if not (basename and proj_path):
            continue
        # Try project.path/basename first; fall back to its dirname (some
        # releases put the .kicad_pro file path in project.path instead of
        # the project directory).
        for candidate in (os.path.join(proj_path, basename),
                          os.path.join(os.path.dirname(proj_path), basename)):
            if os.path.isfile(candidate):
                return candidate
    return None


# --- Coordinate helpers -----------------------------------------------------

def vec_mm(x_mm: float, y_mm: float):
    """Construct a kipy Vector2 from millimetre coordinates."""
    return _ensure_kipy().Vector2.from_xy_mm(x_mm, y_mm)


def _mm_to_nm(v: float) -> int:
    """Convert millimetres to the integer nanometres kipy expects on the wire."""
    return int(round(v * 1_000_000))


# --- Layer mappings ---------------------------------------------------------

def _build_layer_maps():
    """Build dicts that translate between KiCad string layer names and the
    `BoardLayer` enum kipy exposes. Returns (name_to_bl, bl_to_name).
    """
    BoardLayer = _ensure_kipy().BoardLayer
    name_to_bl: dict[str, "BoardLayer"] = {
        "F.Cu": BoardLayer.BL_F_Cu,
        "B.Cu": BoardLayer.BL_B_Cu,
        "F.SilkS": BoardLayer.BL_F_SilkS,
        "B.SilkS": BoardLayer.BL_B_SilkS,
        "F.Mask": BoardLayer.BL_F_Mask,
        "B.Mask": BoardLayer.BL_B_Mask,
        "F.Paste": BoardLayer.BL_F_Paste,
        "B.Paste": BoardLayer.BL_B_Paste,
        "Edge.Cuts": BoardLayer.BL_Edge_Cuts,
    }
    for i in range(1, 31):
        attr = f"BL_In{i}_Cu"
        bl = getattr(BoardLayer, attr, None)
        if bl is not None:
            name_to_bl[f"In{i}.Cu"] = bl
    for i in range(1, 10):
        attr = f"BL_User_{i}"
        bl = getattr(BoardLayer, attr, None)
        if bl is not None:
            name_to_bl[f"User.{i}"] = bl

    bl_to_name = {bl: name for name, bl in name_to_bl.items()}
    return name_to_bl, bl_to_name


_LAYER_MAPS_CACHE = None


def layer_maps():
    """Return cached (name_to_bl, bl_to_name) dicts."""
    global _LAYER_MAPS_CACHE
    if _LAYER_MAPS_CACHE is None:
        _LAYER_MAPS_CACHE = _build_layer_maps()
    return _LAYER_MAPS_CACHE


def layer_id_for(name: str):
    """Translate a string layer name → kipy BoardLayer enum.

    Falls back to BL_F_Cu (with a warning print) for unknown names so the
    write path doesn't crash mid-route on a typo.
    """
    name_to_bl, _ = layer_maps()
    bl = name_to_bl.get(name)
    if bl is None:
        print(f"Warning: unknown layer name '{name}', defaulting to F.Cu")
        return name_to_bl["F.Cu"]
    return bl


def layer_name_for(bl) -> str:
    """Translate a kipy BoardLayer enum → string layer name."""
    _, bl_to_name = layer_maps()
    return bl_to_name.get(bl, "F.Cu")


# --- Net lookup -------------------------------------------------------------

class NetMap:
    """Name → kipy Net lookup for one board read.

    PCBData uses synthetic integer net_ids that don't correspond to
    kipy.Net.code (which is deprecated in K10 and unreliable). The only
    canonical identifier across both sides is the net name, so callers
    must pass `net_name` and we do all resolution by name.
    """

    def __init__(self, board):
        nets = list(board.get_nets())
        self._by_name: dict[str, object] = {
            (getattr(n, "name", "") or ""): n for n in nets
        }

    def resolve(self, net_name: Optional[str] = None, **_ignored):
        """Return a kipy Net object for the given net name, or None."""
        if not net_name:
            return None
        return self._by_name.get(net_name)


# --- Commit context ---------------------------------------------------------

class _Commit:
    """Builder that batches new items into a single push_commit call."""

    def __init__(self, board, net_map: NetMap):
        self._board = board
        self._net_map = net_map
        self._items: list = []
        self._handle = board.begin_commit()

    @property
    def board(self):
        return self._board

    @property
    def net_map(self) -> NetMap:
        return self._net_map

    def add(self, item) -> None:
        self._items.append(item)

    def extend(self, items: Iterable) -> None:
        self._items.extend(items)

    def push(self, message: str) -> None:
        if self._items:
            self._board.create_items(self._items)
        self._board.push_commit(self._handle, message)

    def drop(self) -> None:
        self._board.drop_commit(self._handle)


@contextmanager
def begin_commit(board, message: str = "KiCadRoutingTools"):
    """Context manager: yields a `_Commit` that aggregates new items.

    On clean exit, pushes everything as a single undo step. On exception,
    drops the commit so the board state is unchanged. The whole commit is
    serialised against other IPC calls (see `ipc_lock`).
    """
    with ipc_lock():
        net_map = NetMap(board)
        commit = _Commit(board, net_map)
        try:
            yield commit
        except Exception:
            commit.drop()
            raise
        else:
            commit.push(message)


# --- Track / Via / Shape construction --------------------------------------

def make_track(net_map: NetMap, start_x_mm: float, start_y_mm: float,
               end_x_mm: float, end_y_mm: float, width_mm: float,
               layer_name: str, net_name: Optional[str] = None):
    """Build a kipy Track ready to be added to a commit.

    Net is assigned by name (the only stable identifier between PCBData's
    synthetic net_ids and kipy's Net objects).
    """
    t = _ensure_kipy().Track()
    t.start = vec_mm(start_x_mm, start_y_mm)
    t.end = vec_mm(end_x_mm, end_y_mm)
    t.width = _mm_to_nm(width_mm)
    t.layer = layer_id_for(layer_name)
    net = net_map.resolve(net_name=net_name)
    if net is not None:
        t.net = net
    elif net_name:
        print(f"Warning: net '{net_name}' not found on board; track will be netless")
    return t


def make_via(net_map: NetMap, x_mm: float, y_mm: float, size_mm: float,
             drill_mm: float, top_layer: str = "F.Cu",
             bottom_layer: str = "B.Cu", net_name: Optional[str] = None):
    """Build a kipy Via ready to be added to a commit.

    Note: kipy doesn't expose a simple `SetLayerPair`; through-hole vias
    work out-of-the-box (default padstack spans F.Cu→B.Cu). Blind/buried
    vias may need padstack customisation — see TODO in plan's open
    questions.
    """
    v = _ensure_kipy().Via()
    v.position = vec_mm(x_mm, y_mm)
    # kipy 0.7 has set_diameter/set_drill_diameter setters; older releases
    # exposed the same fields as attributes. Use the setter when present.
    if hasattr(v, "set_diameter"):
        v.set_diameter(_mm_to_nm(size_mm))
    else:
        v.diameter = _mm_to_nm(size_mm)
    if hasattr(v, "set_drill_diameter"):
        v.set_drill_diameter(_mm_to_nm(drill_mm))
    else:
        v.drill_diameter = _mm_to_nm(drill_mm)
    net = net_map.resolve(net_name=net_name)
    if net is not None:
        v.net = net
    elif net_name:
        print(f"Warning: net '{net_name}' not found on board; via will be netless")
    # Blind/buried vias: kipy stores layer span on padstack.drill, not on
    # the Via wrapper itself. Through-hole is the default (F.Cu→B.Cu) so
    # only write when the caller asked for something else.
    if top_layer != "F.Cu" or bottom_layer != "B.Cu":
        try:
            drill = v.padstack.drill
            drill.start_layer = layer_id_for(top_layer)
            drill.end_layer = layer_id_for(bottom_layer)
        except (AttributeError, Exception) as e:
            print(f"Warning: non-through-hole via ({top_layer}→{bottom_layer}) "
                  f"not supported on this kipy version ({e}); "
                  f"falling back to through via")
    return v


def make_debug_line(start_xy: tuple[float, float], end_xy: tuple[float, float],
                    layer_name: str, width_mm: float = 0.05):
    """Build a kipy BoardShape segment for debug visualisation on a User layer."""
    s = _ensure_kipy().BoardShape()
    # BoardShape.shape_type / .stroke_width vary between kipy releases.
    # Probe defensively; missing attrs are silently ignored.
    try:
        from kipy.board_types import BoardShapeType
        s.shape_type = BoardShapeType.SEGMENT
    except (ImportError, AttributeError):
        pass
    s.start = vec_mm(*start_xy)
    s.end = vec_mm(*end_xy)
    s.layer = layer_id_for(layer_name)
    if hasattr(s, "stroke_width"):
        s.stroke_width = _mm_to_nm(width_mm)
    else:
        s.width = _mm_to_nm(width_mm)
    return s


# --- Zone construction -----------------------------------------------------

def make_zone(net_map: NetMap, polygon_mm, layer_name: str,
              net_name: Optional[str] = None,
              clearance_mm: float = 0.2,
              min_thickness_mm: float = 0.1):
    """Build a kipy Zone (copper fill) ready to be added to a commit.

    polygon_mm: iterable of (x_mm, y_mm) outline points. Must form a
        closed loop (last point auto-connects to first).
    """
    Zone, ZoneType, PolyLine, PolyLineNode, PolygonWithHoles = _zone_types()

    polyline = PolyLine()
    # PolyLineNode.from_xy_mm exists on kipy 0.7+; older releases only
    # have from_xy which takes nanometres. Probe both.
    if hasattr(PolyLineNode, "from_xy_mm"):
        for x_mm, y_mm in polygon_mm:
            polyline.append(PolyLineNode.from_xy_mm(x_mm, y_mm))
    else:
        for x_mm, y_mm in polygon_mm:
            polyline.append(PolyLineNode.from_xy(_mm_to_nm(x_mm), _mm_to_nm(y_mm)))

    poly_with_holes = PolygonWithHoles()
    poly_with_holes.outline = polyline

    zone = Zone()
    zone.type = ZoneType.ZT_COPPER
    zone.outline = poly_with_holes
    zone.layers = [layer_id_for(layer_name)]
    zone.clearance = _mm_to_nm(clearance_mm)
    zone.min_thickness = _mm_to_nm(min_thickness_mm)

    if net_name:
        net = net_map.resolve(net_name=net_name)
        if net is not None:
            zone.net = net
        else:
            print(f"Warning: net '{net_name}' not found; zone will be netless")
    return zone


def existing_zone_keys(board) -> set[tuple[str, str]]:
    """Return the {(net_name, layer_name)} pairs of zones already on the board.

    Used by the planes tab to skip duplicate zone creation when running the
    plane builder more than once.
    """
    keys: set[tuple[str, str]] = set()
    try:
        with ipc_lock():
            zones = list(board.get_zones())
    except AttributeError:
        return keys
    except Exception:
        return keys
    for z in zones:
        net_name = ""
        net = getattr(z, "net", None)
        if net is not None:
            net_name = getattr(net, "name", "") or ""
        layers = getattr(z, "layers", None) or []
        for bl in layers:
            keys.add((net_name, layer_name_for(bl)))
    return keys


# --- Apply routing results --------------------------------------------------

def apply_routing_results(board, results_data: dict, *,
                          pcb_data=None,
                          add_debug_lines: bool = False,
                          message: str = "KiCadRoutingTools: route") -> dict:
    """Apply a routing-result payload to the live board in one commit.

    `pcb_data` is required to translate the synthetic net_ids that segments
    and vias carry into the canonical net names that kipy uses to assign
    `track.net` / `via.net`. Without it every track ends up netless.

    Returns a dict with counts: {"tracks": N, "vias": N, "debug_lines": N}.
    """
    counts = {"tracks": 0, "vias": 0, "debug_lines": 0}

    def net_name_for(net_id):
        if pcb_data is None or net_id is None:
            return None
        net = pcb_data.nets.get(net_id)
        return net.name if net is not None else None

    with begin_commit(board, message) as commit:
        # Segments + vias from each net's result
        for result in results_data.get("results", []):
            for seg in result.get("new_segments", []):
                commit.add(make_track(
                    commit.net_map,
                    seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                    seg.width, seg.layer,
                    net_name=net_name_for(seg.net_id),
                ))
                counts["tracks"] += 1
            for via in result.get("new_vias", []):
                commit.add(_via_from_obj(commit.net_map, via, net_name_for))
                counts["vias"] += 1
        # Vias added by inter-layer swap optimisation
        for via in results_data.get("all_swap_vias", []):
            commit.add(_via_from_obj(commit.net_map, via, net_name_for))
            counts["vias"] += 1
        # Optional debug lines on User layers
        if add_debug_lines:
            counts["debug_lines"] = _add_debug_lines(commit, results_data)
    return counts


def apply_planes_results(board, *, pcb_data,
                         new_vias=None, new_segments=None, new_zones=None,
                         message: str = "KiCadRoutingTools: planes") -> dict:
    """Apply planes-tab results (zones + stitch vias + tracks) in one commit.

    Each input is a list of dicts (see planes_gui callers for the exact
    keys). Returns counts and the list of skipped zones (those whose
    net+layer already had a zone on the board).
    """
    counts = {"tracks": 0, "vias": 0, "zones": 0, "zones_skipped": 0}
    skipped: list[tuple[str, str]] = []

    def name_for(net_id):
        if pcb_data is None or net_id is None:
            return None
        net = pcb_data.nets.get(net_id)
        return net.name if net is not None else None

    existing = existing_zone_keys(board)

    with begin_commit(board, message) as commit:
        for vd in (new_vias or []):
            layers = vd.get('layers') or ['F.Cu', 'B.Cu']
            top = layers[0]
            bot = layers[-1] if len(layers) >= 2 else 'B.Cu'
            commit.add(make_via(
                commit.net_map,
                vd['x'], vd['y'], vd['size'], vd['drill'],
                top_layer=top, bottom_layer=bot,
                net_name=name_for(vd.get('net_id')),
            ))
            counts["vias"] += 1

        for sd in (new_segments or []):
            start = sd['start']
            end = sd['end']
            commit.add(make_track(
                commit.net_map,
                start[0], start[1], end[0], end[1],
                sd['width'], sd['layer'],
                net_name=name_for(sd.get('net_id')),
            ))
            counts["tracks"] += 1

        for zd in (new_zones or []):
            net_name = zd.get('net_name') or name_for(zd.get('net_id'))
            layer = zd.get('layer', 'F.Cu')
            key = (net_name or "", layer)
            if key in existing:
                skipped.append(key)
                counts["zones_skipped"] += 1
                continue
            try:
                commit.add(make_zone(
                    commit.net_map,
                    polygon_mm=zd['polygon_points'],
                    layer_name=layer,
                    net_name=net_name,
                    clearance_mm=zd.get('clearance', 0.2),
                    min_thickness_mm=zd.get('min_thickness', 0.1),
                ))
                counts["zones"] += 1
            except Exception as e:
                print(f"Warning: skipped zone for '{net_name}' on {layer}: {e}")

    counts["_skipped_keys"] = skipped
    return counts


def _via_from_obj(net_map: NetMap, via, net_name_for) -> object:
    """Build a kipy Via from a kicad_parser.Via dataclass instance."""
    layers = getattr(via, "layers", None) or ["F.Cu", "B.Cu"]
    top = layers[0] if len(layers) >= 1 else "F.Cu"
    bot = layers[1] if len(layers) >= 2 else "B.Cu"
    return make_via(
        net_map, via.x, via.y, via.size, via.drill,
        top_layer=top, bottom_layer=bot,
        net_name=net_name_for(via.net_id),
    )


def _add_debug_lines(commit: _Commit, results_data: dict) -> int:
    count = 0
    for result in results_data.get("results", []):
        raw_path = result.get("raw_astar_path", [])
        for i in range(len(raw_path) - 1):
            x1, y1 = raw_path[i][0], raw_path[i][1]
            x2, y2 = raw_path[i + 1][0], raw_path[i + 1][1]
            if abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                commit.add(make_debug_line((x1, y1), (x2, y2), "User.9"))
                count += 1
        simplified = result.get("simplified_path", [])
        for i in range(len(simplified) - 1):
            x1, y1 = simplified[i][0], simplified[i][1]
            x2, y2 = simplified[i + 1][0], simplified[i + 1][1]
            if abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                commit.add(make_debug_line((x1, y1), (x2, y2), "User.8"))
                count += 1
        for start, end in result.get("debug_connector_lines", []):
            commit.add(make_debug_line(start, end, "User.3"))
            count += 1
        for start, end in result.get("debug_stub_arrows", []):
            commit.add(make_debug_line(start, end, "User.4"))
            count += 1
    for start, end in results_data.get("exclusion_zone_lines", []):
        commit.add(make_debug_line(start, end, "User.5"))
        count += 1
    return count


# --- Text move helper -------------------------------------------------------

def move_copper_text_to_silkscreen(board) -> int:
    """Move PCB text from F.Cu/B.Cu onto the matching silkscreen layer.

    Returns the count of moved items. Wrapped in its own commit so it
    appears as a separate undo step from the route.
    """
    BoardLayer = _ensure_kipy().BoardLayer
    moved = []
    try:
        items = board.get_shapes()
    except AttributeError:
        return 0
    f_cu = BoardLayer.BL_F_Cu
    b_cu = BoardLayer.BL_B_Cu
    f_silk = BoardLayer.BL_F_SilkS
    b_silk = BoardLayer.BL_B_SilkS
    for item in items:
        # We only want PCB_TEXT-equivalents; the kipy shape API distinguishes
        # text via `kind`/class attributes that vary across kipy versions, so
        # check defensively.
        cls = type(item).__name__
        if "Text" not in cls:
            continue
        layer = getattr(item, "layer", None)
        if layer == f_cu:
            item.layer = f_silk
            moved.append(item)
        elif layer == b_cu:
            item.layer = b_silk
            moved.append(item)
    if not moved:
        return 0
    handle = board.begin_commit()
    try:
        board.update_items(moved)
        board.push_commit(handle, "Move copper text to silkscreen")
    except Exception:
        board.drop_commit(handle)
        raise
    return len(moved)
