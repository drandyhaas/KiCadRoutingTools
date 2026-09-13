#!/usr/bin/env python3
"""In-process stand-in for a kipy Board, backed by a .kicad_pcb file.

WHY THIS EXISTS
---------------
The GUI-parity harnesses drive the REAL RoutingDialog headlessly and grade the
board it produces against the CLI chain. Under SWIG that was easy: `pcbnew`
lives in-process, so `headless_plan` could do

    board = pcbnew.LoadBoard(path)
    pcbnew.GetBoard = lambda: board      # the plugin's "live board"

and the whole plugin operated on a file-loaded board with no KiCad running.

The IPC plugin has no in-process board at all -- `kicad_ipc_adapter` reaches a
RUNNING KiCad over a kipy socket. Taken literally that would make every parity
gate require a GUI KiCad with the right document open, which is neither
reproducible nor CI-able, and would mutate whatever the user has open.

This class restores the old property. It satisfies the slice of the kipy Board
API that `kicad_ipc_adapter` actually calls, so the adapter -- and therefore the
dialog, the plan executor and the engines above it -- run UNMODIFIED, while the
"live board" is really a file on disk. Every commit is flushed back to that
file, so after each plan step there is a genuine .kicad_pcb to grade, exactly
like the CLI's file-to-file chain.

WHAT IT DOES AND DOES NOT PROVE
-------------------------------
It exercises everything ABOVE the socket: dialog config assembly, the plan
executor, engine kwargs, and the adapter's own add/remove/update bookkeeping.
It does NOT exercise real kipy wire behaviour -- for that, run against a live
KiCad. The read path (kipy -> PCBData) is deliberately served by re-parsing the
file with parse_kicad_pcb rather than being re-implemented here; that path has
its own parity tooling (validate_pcb_data.py) and re-implementing it would mean
testing this file's guess at kipy semantics instead of the plugin.

Items handed out by get_tracks()/get_vias() are REAL kipy objects (built through
the adapter's own make_track/make_via), so the adapter's attribute reads,
position maths and identity-based removal all behave as they do in production --
which is how the bare `.x_mm` bug (kipy 0.7.1 Vector2 has only integer-nm .x/.y)
was caught.

Needs kipy importable. KiCad's bundled python does not ship it; install with
    <kicad_python> -m pip install --user 'kicad-python>=0.7.0'
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
# The repo root AND py_placer: `_flush_footprint_moves` imports
# `placement.writer`, which lives under py_placer/ since the #522 reorg. The
# path belongs HERE, in the module that has the dependency, rather than in each
# gate that happens to drive a footprint move -- `test_footprint_position_sync`
# set up four paths and not this one, so every run of it died on
# `ModuleNotFoundError: No module named 'placement.writer'` inside push_commit,
# which exits exactly like a failed assertion.
for _p in (_REPO, os.path.join(_REPO, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import parse_kicad_pcb, KICAD_10_MIN_VERSION  # noqa: E402


# id(kipy Zone) -> the arguments make_zone was called with. Module level so a
# board created before install() still resolves zones built after it.
_ZONE_SRC = {}


def _net_obj(name):
    """A real kipy Net carrying `name`.

    kipy items are protobuf-backed: assigning a duck-typed stand-in to
    `track.net` raises `AttributeError: 'X' object has no attribute 'proto'`.
    Net() with no proto and a name set afterwards is constructible offline.
    """
    from kipy.board_types import Net
    n = Net()
    n.name = name or ""
    return n


class _FakeNetMap:
    """Name -> kipy Net, mirroring kicad_ipc_adapter.NetMap.

    Unlike the real one this MINTS nets on demand: the router routinely emits
    copper for a net that carries no copper yet, and a None here would silently
    drop it (the adapter passes net=None straight through).
    """

    def __init__(self, names=()):
        self._by_name = {n: _net_obj(n) for n in names if n}

    def resolve(self, net_name=None, **_ignored):
        if not net_name:
            return None
        if net_name not in self._by_name:
            self._by_name[net_name] = _net_obj(net_name)
        return self._by_name[net_name]


class _FakeFootprint:
    """Minimal movable footprint: reference + position + orientation."""

    def __init__(self, reference, x_mm, y_mm, rotation_deg):
        from kipy.geometry import Vector2
        self.reference = reference
        self.position = Vector2.from_xy_mm(x_mm, y_mm)
        self.orientation = _Deg(rotation_deg or 0.0)


class _FakeZone:
    """Zone stand-in shaped for existing_zone_keys().

    That helper reads `.net.name` and `.layers` -- PLURAL, a list of BoardLayer
    enums it puts through layer_name_for(). Exposing a single `.layer` string
    made every re-run miss the dedupe and stack a second pour on the same
    net+layer.
    """

    def __init__(self, net_name, layer_name):
        from kicad_ipc_adapter import layer_maps
        self.net = _NamedNet(net_name)
        self.layer = layer_name
        name_to_bl, _ = layer_maps()
        bl = name_to_bl.get(layer_name)
        self.layers = [bl] if bl is not None else []


class _NamedNet:
    def __init__(self, name):
        self.name = name or ""


class _Deg:
    """Angle stand-in exposing `.degrees`, the field the adapter reads back.

    apply_footprint_moves ASSIGNS a real kipy Angle; this is only the seed value
    so an un-moved footprint round-trips at its current rotation.
    """

    def __init__(self, degrees):
        self.degrees = degrees


class FakeIpcBoard:
    """The kipy Board slice `kicad_ipc_adapter` uses, backed by `path`.

    The file at `path` is mutated in place on every push, so it is always the
    board as the plugin currently sees it.
    """

    def __init__(self, path):
        self.path = os.path.abspath(path)
        self._commit_seq = 0
        self._reseed()

    # -- state -------------------------------------------------------------

    def _reseed(self):
        """Rebuild the kipy-object view from the file on disk.

        Called after every push so a later step's get_tracks() sees the copper
        the previous step wrote -- the property that made the SWIG harness able
        to run multi-step plans.
        """
        from kicad_ipc_adapter import make_track, make_via
        self.pcb_data = parse_kicad_pcb(self.path)
        names = {n.name for n in self.pcb_data.nets.values() if n.name}
        self.net_map = _FakeNetMap(names)
        self._name_of = {nid: n.name for nid, n in self.pcb_data.nets.items()}

        # id(kipy item) -> the parsed dataclass it came from, so a removal
        # (which the adapter expresses as "remove THIS item") can be turned
        # back into something the text writer can strip.
        self._src = {}
        self._tracks, self._vias = [], []
        for s in self.pcb_data.segments:
            t = make_track(self.net_map, s.start_x, s.start_y, s.end_x, s.end_y,
                           s.width, s.layer, net_name=self._name_of.get(s.net_id))
            self._src[id(t)] = s
            self._tracks.append(t)
        for v in self.pcb_data.vias:
            layers = v.layers or ['F.Cu', 'B.Cu']
            k = make_via(self.net_map, v.x, v.y, v.size, v.drill,
                         top_layer=layers[0], bottom_layer=layers[-1],
                         net_name=self._name_of.get(v.net_id))
            self._src[id(k)] = v
            self._vias.append(k)

        self._footprints = [
            _FakeFootprint(ref, fp.x, fp.y, fp.rotation)
            for ref, fp in self.pcb_data.footprints.items()
        ]
        self._zones = [
            _FakeZone(self._name_of.get(z.net_id, ''), z.layer)
            for z in self.pcb_data.zones
        ]

        # Pending, flushed on push_commit.
        self._added, self._removed, self._updated = [], [], []
        self._zone_sexprs = []

    # -- reads -------------------------------------------------------------

    def get_tracks(self):
        return list(self._tracks)

    def get_vias(self):
        return list(self._vias)

    def get_nets(self):
        return [self.net_map.resolve(n) for n in sorted(
            {n.name for n in self.pcb_data.nets.values() if n.name})]

    def get_footprints(self):
        """Movable footprint stand-ins (reference / position / orientation).

        Only what apply_footprint_moves touches: it reads the reference through
        kicad_parser._fp_reference (which probes `.reference` first), then SETS
        `.position` / `.orientation` and commit.update()s the object. These are
        plain Python objects on purpose -- nothing is assigned INTO a kipy
        protobuf here, so no `.proto` is required.
        """
        return list(self._footprints)

    # The adapter reads these but nothing it does headlessly depends on their
    # contents; PCBData (from the file) is the real source for geometry.

    def get_pads(self):
        return []

    def get_zones(self):
        """Zone stand-ins carrying net + layer.

        existing_zone_keys() reads exactly those two fields to skip duplicate
        zone creation when the plane builder runs twice. Returning [] made every
        re-run look like a fresh board.
        """
        return list(self._zones)

    def get_shapes(self):
        return []

    def get_drawings(self):
        return []

    def get_selection(self):
        return []

    def get_enabled_layers(self):
        from kipy.board_types import BoardLayer
        out = []
        for name in self.pcb_data.board_info.copper_layers:
            bl = getattr(BoardLayer, 'BL_' + name.replace('.', '_'), None)
            if bl is not None:
                out.append(bl)
        return out

    # -- commit ------------------------------------------------------------

    def begin_commit(self):
        self._commit_seq += 1
        self._added, self._removed, self._updated = [], [], []
        return self._commit_seq

    def create_items(self, items):
        self._added.extend(items)

    def remove_items(self, items):
        self._removed.extend(items)

    def update_items(self, items):
        self._updated.extend(items)

    def _flush_footprint_moves(self):
        """Write updated footprint positions back to the file.

        apply_footprint_moves (optimize_caps, #130) mutates footprint objects and
        commit.update()s them. Without this the moves would live only in memory,
        and the next step's PCBData -- re-parsed from the file -- would show the
        caps back at their ORIGINAL positions: the #362 stale-position bug, but
        manufactured by the harness rather than by the plugin.
        """
        from placement.writer import write_placed_output
        placements = []
        for fp in self._updated:
            ref = getattr(fp, 'reference', None)
            pos = getattr(fp, 'position', None)
            if not ref or pos is None:
                continue
            from kicad_ipc_adapter import _vec_xy_mm
            x, y = _vec_xy_mm(pos)
            rot = 0.0
            ang = getattr(fp, 'orientation', None)
            if ang is not None:
                rot = getattr(ang, 'degrees', None)
                rot = rot if rot is not None else 0.0
            placements.append({'reference': ref, 'new_x': x, 'new_y': y,
                               'new_rotation': rot})
        if placements:
            write_placed_output(self.path, self.path, placements)
        return len(placements)

    def drop_commit(self, _handle):
        self._added, self._removed, self._updated = [], [], []

    def push_commit(self, _handle, _message=""):
        """Flush the batch to the .kicad_pcb, then re-read it."""
        from plane_io import write_plane_output
        from kicad_writer import generate_zone_sexpr
        from kicad_ipc_adapter import _vec_xy_mm, layer_name_for

        def _name(item):
            net = getattr(item, 'net', None)
            return (getattr(net, 'name', '') or '') if net is not None else ''

        def _nid(name):
            for nid, nm in self._name_of.items():
                if nm == name:
                    return nid
            return 0

        segs, vias = [], []
        for it in self._added:
            if hasattr(it, 'start') and hasattr(it, 'end'):
                sx, sy = _vec_xy_mm(it.start)
                ex, ey = _vec_xy_mm(it.end)
                segs.append({'start': (sx, sy), 'end': (ex, ey),
                             'width': getattr(it, 'width', 0) / 1e6,
                             'layer': layer_name_for(getattr(it, 'layer', None)),
                             'net_id': _nid(_name(it))})
            elif hasattr(it, 'position'):
                x, y = _vec_xy_mm(it.position)
                vias.append({'x': x, 'y': y,
                             'size': getattr(it, 'diameter', 0) / 1e6,
                             'drill': getattr(it, 'drill_diameter', 0) / 1e6,
                             'layers': ['F.Cu', 'B.Cu'],
                             'net_id': _nid(_name(it))})

        # Removals: map the kipy items back to the parsed objects the text
        # writer understands. Items the router ADDED in this same commit and
        # then removed never reached the file, so they just drop out.
        rm_segs = [self._src[id(it)] for it in self._removed
                   if id(it) in self._src and hasattr(self._src[id(it)], 'start_x')]
        rm_vias = [self._src[id(it)] for it in self._removed
                   if id(it) in self._src and hasattr(self._src[id(it)], 'x')]

        # Zones. A kipy Zone carries none of the fields the text writer needs in
        # a shape we could read back reliably, so install() wraps make_zone and
        # records the SOURCE arguments -- lossless, and it cannot drift from what
        # the plugin actually asked for. Dropping these was worth 257 phantom
        # "gui-only" segments: with no pour on the board, the next step's plane
        # REPAIR reconnected the regions with tracks instead.
        for it in self._added:
            zd = _ZONE_SRC.get(id(it))
            if zd is None:
                continue
            self._zone_sexprs.append(generate_zone_sexpr(
                net_id=_nid(zd['net_name']),
                net_name=zd['net_name'],
                layer=zd['layer'],
                polygon_points=zd['polygon_mm'],
                clearance=zd['clearance_mm'],
                min_thickness=zd['min_thickness_mm'],
                direct_connect=True,
                use_net_name=self.pcb_data.kicad_version >= KICAD_10_MIN_VERSION,
            ))

        zone_sexpr = '\n'.join(self._zone_sexprs) if self._zone_sexprs else None
        ok = write_plane_output(
            self.path, self.path, zone_sexpr, vias, segs,
            net_id_to_name=self._name_of,
            removed_segments=rm_segs or None)
        if not ok:
            raise RuntimeError("FakeIpcBoard: write_plane_output failed")

        if rm_vias:
            self._strip_vias(rm_vias)
        if self._updated:
            self._flush_footprint_moves()

        self._zone_sexprs = []
        self._reseed()

    def _strip_vias(self, vias):
        """Delete the file's `(via ...)` blocks for vias the adapter removed.

        `net_id_to_name` is NOT optional here, though the signature allows it.
        `remove_vias_from_content` matches on (position, NET TOKEN), and it
        canonicalises the token through this map: without it every target key
        carries the numeric net id while a KiCad 10 board's via blocks spell
        `(net "NAME")`, so NOTHING matches and the vias stay in the file. The
        sibling call in `push_commit` -- `write_plane_output(...,
        removed_segments=...)` -- has always passed it, which is why this
        harness removed segments correctly and vias not at all.

        That asymmetry made the harness manufacture a product bug. Measured on
        rp2350_fpga_eensy_prePlane: the GUI leg shipped 8 vias the CLI leg does
        not -- including two barrels stacked in one hole -- and 9 DRC
        violations, and `test_gui_livechain_rp2350` reported it as a live-chain
        divergence (#362's own subject) for as long as the gate has been able
        to run. The real IPC plugin never reaches this code: it removes items
        over the socket.

        Unmatched targets are REPORTED rather than dropped, because a silent
        zero-match is precisely what hid this.
        """
        from kicad_writer import remove_vias_from_content
        with open(self.path, 'r', encoding='utf-8') as f:
            content = f.read()
        leftovers = []
        content, _n = remove_vias_from_content(
            content, vias, self._name_of, unmatched_out=leftovers)
        if leftovers:
            print(f"  FakeIpcBoard: {len(leftovers)} via(s) the adapter "
                  f"removed could not be matched in the file and were NOT "
                  f"stripped")
        with open(self.path, 'w', encoding='utf-8') as f:
            f.write(content)

    # -- misc --------------------------------------------------------------

    def add_zone_sexpr(self, sexpr):
        """Queue a zone s-expression for the next push (planes tab)."""
        self._zone_sexprs.append(sexpr)

    def save_as(self, path, overwrite=True, include_project=False):
        import shutil
        if os.path.abspath(path) != self.path:
            shutil.copy(self.path, path)
        return True


def install(board_path):
    """Point `kicad_ipc_adapter` (and the PCBData builder) at a FakeIpcBoard.

    Returns the board. This is the IPC analogue of the SWIG harness's
    `pcbnew.GetBoard = lambda: board`: the plugin keeps calling
    `get_board()` / `build_pcb_data_from_board()` exactly as in production.
    """
    import kicad_ipc_adapter as adapter
    import kicad_parser

    board = FakeIpcBoard(board_path)

    # Record what each kipy Zone was BUILT from. Reading a protobuf-backed Zone
    # back (outline nodes, layer enum, net, clearance) is fiddly and version
    # dependent; capturing the call arguments is exact and cannot drift.
    _real_make_zone = adapter.make_zone

    def _make_zone(net_map, polygon_mm, layer_name, net_name=None,
                   clearance_mm=0.2, min_thickness_mm=0.1, **kw):
        z = _real_make_zone(net_map, polygon_mm, layer_name, net_name=net_name,
                            clearance_mm=clearance_mm,
                            min_thickness_mm=min_thickness_mm, **kw)
        _ZONE_SRC[id(z)] = {
            'polygon_mm': list(polygon_mm), 'layer': layer_name,
            'net_name': net_name or '', 'clearance_mm': clearance_mm,
            'min_thickness_mm': min_thickness_mm,
        }
        return z

    adapter.make_zone = _make_zone

    adapter.get_board = lambda *a, **k: board
    adapter.connect = lambda *a, **k: None
    adapter.get_board_full_path = lambda *a, **k: board.path
    adapter.save_board_snapshot = lambda path, *a, **k: board.save_as(path)
    adapter.ping = lambda *a, **k: True

    # Every plugin call site does a function-local
    # `from kicad_parser import build_pcb_data_from_board`, so rebinding the
    # module attribute reaches all of them. Serving it from the file keeps the
    # harness honest: the board the engines see is the board on disk.
    _real_build = kicad_parser.build_pcb_data_from_board

    def _build(b, *a, **k):
        if isinstance(b, FakeIpcBoard):
            return build_pcb_data_like_ipc(b.path)
        return _real_build(b, *a, **k)

    kicad_parser.build_pcb_data_from_board = _build
    return board


def build_pcb_data_like_ipc(path):
    """parse_kicad_pcb + the extras the REAL kipy builder adds.

    The kipy builder does not only translate board objects: net-class settings
    are not exposed over IPC at all, so it reads them out of the sibling
    .kicad_pro and lands them in `netclass_params` / `net_to_class`. The text
    parser does not do that, so serving PCBData with a bare parse_kicad_pcb
    silently hands the dialog an EMPTY netclass map -- and the dialog's
    `_effective_*` floors then fall back to their control defaults (0.3 track
    width instead of the board's 0.127).

    That would make the harness invent divergences that do not exist in the real
    GUI, which is precisely the failure mode the previous shim harness had. So
    mirror the builder here.
    """
    pcb = parse_kicad_pcb(path)
    try:
        from kicad_parser import (read_netclasses_from_kicad_pro,
                                  _resolve_net_to_class)
        classes_by_name, patterns, explicit = read_netclasses_from_kicad_pro(path)
        pcb.netclass_params = classes_by_name
        pcb.net_to_class = {
            n.name: _resolve_net_to_class(n.name, patterns, explicit)
            for n in pcb.nets.values() if n.name
        }
    except Exception as e:
        print(f"fake_ipc_board: netclass read failed: {e}")
    return pcb
