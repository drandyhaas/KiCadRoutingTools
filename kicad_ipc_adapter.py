"""KiCad IPC adapter — centralises all `kipy` (kicad-python) interaction.

Every place that used to import `pcbnew` now calls into this module instead,
so the SWIG → IPC churn doesn't leak across the codebase. Keep the surface
small and stable.

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

from contextlib import contextmanager
from typing import Iterable, Optional


# --- Lazy imports -----------------------------------------------------------
# kipy is only installed inside KiCad's per-plugin venv, so guard the import
# to give a clear error if someone tries to use this module outside KiCad.

def _import_kipy():
    try:
        import kipy
        from kipy.board_types import BoardLayer, Track, Via, BoardShape
        from kipy.geometry import Vector2
        return kipy, BoardLayer, Track, Via, BoardShape, Vector2
    except ImportError as e:
        raise ImportError(
            "kicad-python (kipy) is not installed. This plugin runs as a "
            "KiCad 10 IPC plugin and requires the `kicad-python` package. "
            "If you're running the plugin from KiCad's PCM, KiCad should "
            "provision a venv automatically. If you're running standalone, "
            "`pip install kicad-python>=0.7.0`."
        ) from e


# --- Connection -------------------------------------------------------------
# A single IPC connection is reused across the plugin's lifetime — kipy uses
# a synchronous, single-threaded protobuf socket, and we never spawn multiple
# `KiCad()` clients per process.

_KICAD_SINGLETON = None


def connect() -> "kipy.KiCad":
    """Open (or reuse) the IPC connection to the running KiCad instance.

    Reads `KICAD_API_SOCKET` and `KICAD_API_TOKEN` from the environment;
    KiCad sets both when it spawns an IPC plugin.
    """
    global _KICAD_SINGLETON
    if _KICAD_SINGLETON is None:
        kipy, *_ = _import_kipy()
        _KICAD_SINGLETON = kipy.KiCad()
    return _KICAD_SINGLETON


def get_board(kicad: Optional["kipy.KiCad"] = None):
    """Return the currently-open PCB, or None if no board is open."""
    if kicad is None:
        kicad = connect()
    try:
        return kicad.get_board()
    except Exception:
        return None


# --- Coordinate helpers -----------------------------------------------------

def vec_mm(x_mm: float, y_mm: float):
    """Construct a kipy Vector2 from millimetre coordinates."""
    _, _, _, _, _, Vector2 = _import_kipy()
    return Vector2.from_xy_mm(x_mm, y_mm)


def to_mm_xy(vec) -> tuple[float, float]:
    """Convert a kipy Vector2 to a (x_mm, y_mm) tuple."""
    return (vec.x_mm, vec.y_mm)


# --- Layer mappings ---------------------------------------------------------

def _build_layer_maps():
    """Build dicts that translate between KiCad string layer names and the
    `BoardLayer` enum kipy exposes. Returns (name_to_bl, bl_to_name).
    """
    _, BoardLayer, *_ = _import_kipy()

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

    def by_name(self, name: str):
        return self._by_name.get(name)

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
    drops the commit so the board state is unchanged.
    """
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
    _, _, Track, _, _, _ = _import_kipy()
    t = Track()
    t.start = vec_mm(start_x_mm, start_y_mm)
    t.end = vec_mm(end_x_mm, end_y_mm)
    t.width = int(round(width_mm * 1_000_000))  # mm → nm
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
    _, _, _, Via, _, _ = _import_kipy()
    v = Via()
    v.position = vec_mm(x_mm, y_mm)
    # kipy Via exposes diameter + drill in nm via setter helpers if present.
    if hasattr(v, "set_diameter"):
        v.set_diameter(int(round(size_mm * 1_000_000)))
    else:
        v.diameter = int(round(size_mm * 1_000_000))
    if hasattr(v, "set_drill_diameter"):
        v.set_drill_diameter(int(round(drill_mm * 1_000_000)))
    else:
        v.drill_diameter = int(round(drill_mm * 1_000_000))
    net = net_map.resolve(net_name=net_name)
    if net is not None:
        v.net = net
    elif net_name:
        print(f"Warning: net '{net_name}' not found on board; via will be netless")
    # Blind/buried support is TODO — see plan open questions.
    if top_layer != "F.Cu" or bottom_layer != "B.Cu":
        try:
            v.start_layer = layer_id_for(top_layer)
            v.end_layer = layer_id_for(bottom_layer)
        except AttributeError:
            print(f"Warning: non-through-hole via ({top_layer}→{bottom_layer}) "
                  f"not supported yet via kipy; falling back to through via")
    return v


def make_debug_line(start_xy: tuple[float, float], end_xy: tuple[float, float],
                    layer_name: str, width_mm: float = 0.05):
    """Build a kipy BoardShape segment for debug visualisation on a User layer."""
    _, _, _, _, BoardShape, _ = _import_kipy()
    s = BoardShape()
    # The BoardShape API varies by kipy version; the common surface is to set
    # type to segment and provide start/end points. Try the high-level path
    # first; fall back to attribute assignment if not present.
    try:
        from kipy.board_types import BoardShapeType
        s.shape_type = BoardShapeType.SEGMENT
    except (ImportError, AttributeError):
        pass
    s.start = vec_mm(*start_xy)
    s.end = vec_mm(*end_xy)
    s.layer = layer_id_for(layer_name)
    if hasattr(s, "stroke_width"):
        s.stroke_width = int(round(width_mm * 1_000_000))
    else:
        s.width = int(round(width_mm * 1_000_000))
    return s


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
    _, BoardLayer, *_ = _import_kipy()
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
