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

from kicad_parser import (parse_kicad_pcb, KICAD_10_MIN_VERSION,  # noqa: E402
                          kipy_via_protection_live, kipy_via_type,
                          kipy_via_layers)


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
    """Minimal movable footprint: reference + position + orientation.

    Also, for the result_apply / Labels appliers (all optional, so a caller
    that passes only the first four arguments gets what it always got):
    `layer` (the side, a BoardLayer, which kipy's footprint carries and a
    flip sets), `locked` (a PROPERTY with a setter, as kicad-python 0.7.1's
    FootprintInstance has -- `set_item_locked` writes only through a class
    property, so a plain attribute would swallow the lock), and
    `reference_field`, a REAL kipy Field (write-through protobuf, as kipy's
    `FootprintInstance.reference_field` hands out) seeded from the file's
    Reference label: world position, the stored angle, size and thickness.
    `push_commit` writes back whichever of side, lock and label changed.
    """

    def __init__(self, reference, x_mm, y_mm, rotation_deg, layer_name=None,
                 locked=False, ref_label=None):
        from kipy.geometry import Vector2
        self.reference = reference
        self.position = Vector2.from_xy_mm(x_mm, y_mm)
        self.orientation = _Deg(rotation_deg or 0.0)
        from kipy.board_types import BoardLayer
        self._back = str(layer_name or 'F.Cu').startswith('B')
        self.layer = BoardLayer.BL_B_Cu if self._back else BoardLayer.BL_F_Cu
        self._locked = bool(locked)
        self._seed_locked = self._locked
        try:
            self.reference_field = _reference_field(x_mm, y_mm, rotation_deg,
                                                    ref_label)
            self._seed_label = self.reference_field.proto.SerializeToString()
        except Exception as e:                                  # noqa: BLE001
            # An older kicad-python shapes Field differently: no label to
            # write, rather than no fake board at all.
            print(f"  FakeIpcBoard: {reference}'s Reference field not "
                  f"modelled on this kipy ({e})")
            self.reference_field = None
            self._seed_label = None

    @property
    def locked(self):
        return self._locked

    @locked.setter
    def locked(self, value):
        self._locked = bool(value)

    def label_changed(self):
        """Did the applier write the Reference field (position, angle, size,
        thickness, alignment)?"""
        if self.reference_field is None:
            return False
        return self.reference_field.proto.SerializeToString() != self._seed_label


def _reference_field(fp_x, fp_y, fp_rot, label):
    """A kipy Field seeded the way KiCad reports a Reference over IPC: the
    text's WORLD position (the file stores it in the footprint frame;
    `local_to_global` is the parser's own transform), its stored angle, its
    size (x = width, y = height) and stroke. No label: an empty field."""
    from kipy.board_types import Field
    from kipy.geometry import Vector2
    f = Field()
    if label is None:
        return f
    from kicad_parser import local_to_global, mm_to_iu
    wx_, wy_ = local_to_global(fp_x, fp_y, fp_rot or 0.0, label.at_x, label.at_y)
    t = f.text
    t.position = Vector2.from_xy(mm_to_iu(wx_), mm_to_iu(wy_))
    a = t.attributes
    a.angle = float(label.rotation or 0.0)
    a.size = Vector2.from_xy(mm_to_iu(label.size_w), mm_to_iu(label.size_h))
    a.stroke_width = mm_to_iu(label.thickness)
    return f


class _FakeZone:
    """Zone stand-in shaped for the adapter's zone reads (_live_pours).

    It reads `.net.name`, `.layers` -- PLURAL, a list of BoardLayer enums it
    puts through layer_name_for() -- `.type` and `.name`. ONE stand-in per
    physical zone with its whole layer set, as kipy reports it: the text
    parser hands a multi-layer pour back as one Zone per layer, and one
    single-layer stand-in each would make a multi-layer pour look replaceable
    (route_planes never deletes one to replace a layer). `.type` is
    ZT_TEARDROP for a teardrop (#1164), so it is no pour. `uuid` is how a
    removal reaches the file.
    """

    def __init__(self, net_name, layer_names, uuid='', teardrop=False,
                 name='', parsed=None):
        from kicad_ipc_adapter import layer_maps
        from kipy.proto.board.board_types_pb2 import ZoneType
        if isinstance(layer_names, str):
            layer_names = [layer_names]
        self.net = _NamedNet(net_name)
        self.layer = layer_names[0] if layer_names else ''
        name_to_bl, _ = layer_maps()
        self.layers = [name_to_bl[n] for n in layer_names if n in name_to_bl]
        self.type = ZoneType.ZT_TEARDROP if teardrop else ZoneType.ZT_COPPER
        self.name = name
        self.uuid = uuid
        # The zone's geometry and fill settings as KiCad reports them over
        # IPC, so a reader of the live pours (`kicad_parser.
        # _kipy_zone_entries`, which result_apply reads them with) sees the
        # pour the file holds. Built from the PARSED zone here, never by the
        # plugin's own zone builder, so the two sides of a comparison are
        # not one function. Without it every live pour read as no pour.
        self._kz = None
        if parsed is not None:
            try:
                self._kz = _kipy_zone_from_parsed(parsed, layer_names, net_name,
                                                  self.type, teardrop)
            except Exception as e:                              # noqa: BLE001
                print(f"  FakeIpcBoard: zone {uuid or '?'} not readable as a "
                      f"kipy zone ({e}); readers see no pour there")

    # kipy Zone reads, served from `_kz` (absent -> AttributeError, as before)

    def _k(self):
        if self._kz is None:
            raise AttributeError('zone built without its parsed geometry')
        return self._kz

    @property
    def proto(self):
        return self._k().proto

    @property
    def outline(self):
        return self._k().outline

    @property
    def priority(self):
        return self._k().priority

    @property
    def clearance(self):
        return self._k().clearance

    @property
    def min_thickness(self):
        return self._k().min_thickness

    @property
    def island_mode(self):
        return self._k().island_mode

    @property
    def min_island_area(self):
        return self._k().min_island_area

    @property
    def teardrop(self):
        return self._k().teardrop

    def is_rule_area(self):
        return self._k().is_rule_area()


#: Zone.pad_connection -> kipy ZoneConnectionStyle, and the parser's island
#: mode -> IslandRemovalMode (KiCad's file values: 0 always, 1 never, 2 area).
_FAKE_CONNECT = {'none': 'ZCS_NONE', 'thermal': 'ZCS_THERMAL',
                 'solid': 'ZCS_FULL', 'thru_hole_only': 'ZCS_PTH_THERMAL'}
_FAKE_ISLAND = {0: 'IRM_ALWAYS', 1: 'IRM_NEVER', 2: 'IRM_AREA'}


def _kipy_zone_from_parsed(z, layer_names, net_name, zone_type, teardrop):
    """A real kipy Zone carrying parsed zone Z's outline, holes, layers, net,
    priority, clearance, minimum thickness, pad connection and spokes, and
    island removal -- what KiCad hands back for the zone over IPC."""
    from kipy.board_types import Zone
    from kipy.geometry import PolyLine, PolyLineNode, PolygonWithHoles
    from kipy.proto.board.board_types_pb2 import (ZoneConnectionStyle,
                                                   IslandRemovalMode,
                                                   TeardropType)
    from kicad_ipc_adapter import layer_id_for
    from kicad_parser import mm_to_iu

    def polyline(points):
        pl = PolyLine()
        for x, y in points or ():
            pl.append(PolyLineNode.from_xy(mm_to_iu(x), mm_to_iu(y)))
        return pl
    kz = Zone()
    kz.type = zone_type
    pwh = PolygonWithHoles()
    pwh.outline = polyline(z.polygon)
    for h in getattr(z, 'holes', None) or ():
        pwh.proto.holes.append(polyline(h).proto)
    kz.proto.outline.polygons.append(pwh.proto)
    kz.layers = [layer_id_for(n) for n in layer_names]
    kz.priority = int(getattr(z, 'priority', 0) or 0)
    cs = kz.proto.copper_settings
    if getattr(z, 'clearance', None) is not None:
        cs.clearance.value_nm = mm_to_iu(z.clearance)
    if getattr(z, 'min_thickness', None) is not None:
        cs.min_thickness.value_nm = mm_to_iu(z.min_thickness)
    pc = getattr(z, 'pad_connection', None)
    if pc in _FAKE_CONNECT:
        cs.connection.zone_connection = ZoneConnectionStyle.Value(_FAKE_CONNECT[pc])
    if getattr(z, 'thermal_gap', None) is not None:
        cs.connection.thermal_spokes.gap.value_nm = mm_to_iu(z.thermal_gap)
    if getattr(z, 'thermal_bridge_width', None) is not None:
        cs.connection.thermal_spokes.width.value_nm = mm_to_iu(z.thermal_bridge_width)
    cs.island_mode = IslandRemovalMode.Value(
        _FAKE_ISLAND.get(int(getattr(z, 'island_removal_mode', 0) or 0), 'IRM_ALWAYS'))
    # The file carries `(island_area_min ...)` for an area-mode pour alone;
    # any other pour reports KiCad's default, which kipy's Zone() holds
    # (10 mm^2), as a live KiCad does.
    if getattr(z, 'island_area_min', None):
        cs.min_island_area = int(round(float(z.island_area_min) * 1e12))
    if teardrop:
        cs.teardrop.type = TeardropType.Value('TDT_VIA_PAD')
    if net_name:
        kz.net = _net_obj(net_name)
    return kz


def _zone_sexpr_from_kipy(zone, net_id, use_net_name):
    """The `(zone ...)` block KiCad would save for a copper Zone the plugin
    CREATED over IPC (result_apply.make_pour builds it directly, so
    install()'s make_zone recording never sees it). Read back through the
    plugin's own kipy zone reader (`_kipy_zone_entries`, the IPC builder's),
    and written with every field that reader returns: net, every copper
    layer, outline and holes (each further `(polygon ...)` is a hole, as
    KiCad loads it), priority, pad connection and clearance, minimum
    thickness, spokes and island removal. Unfilled: KiCad fills it when the
    plugin asks (`refill_zones`). None when it is not a copper pour."""
    import uuid as _uuid
    from kicad_parser import _kipy_zone_entries
    from kicad_writer import _escape_net_name
    entries = _kipy_zone_entries(
        zone, lambda n: (net_id, (getattr(n, 'name', '') or '') if n is not None else ''))
    if not entries:
        return None
    z = entries[0]
    layers = [e.layer for e in entries]
    name = _escape_net_name(z.net_name or '')
    net = (f'(net "{name}")' if use_net_name
           else f'(net {net_id})\n\t\t(net_name "{name}")')
    lay = (f'(layer "{layers[0]}")' if len(layers) == 1
           else '(layers ' + ' '.join(f'"{n}"' for n in layers) + ')')
    conn = {'solid': ' yes', 'none': ' no', 'thru_hole_only': ' thru_hole_only'}.get(
        z.pad_connection, '')
    clr = z.clearance if z.clearance is not None else 0.2
    fill = []
    if z.thermal_gap is not None:
        fill.append(f'(thermal_gap {z.thermal_gap:g})')
    if z.thermal_bridge_width is not None:
        fill.append(f'(thermal_bridge_width {z.thermal_bridge_width:g})')
    fill.append(f'(island_removal_mode {int(z.island_removal_mode or 0)})')
    if z.island_area_min:
        fill.append(f'(island_area_min {z.island_area_min:g})')

    def poly(points):
        return ('\t\t(polygon\n\t\t\t(pts\n\t\t\t\t'
                + ' '.join(f'(xy {x:.6f} {y:.6f})' for x, y in points)
                + '\n\t\t\t)\n\t\t)')
    body = [f'\t\t{net}', f'\t\t{lay}', f'\t\t(uuid "{_uuid.uuid4()}")',
            '\t\t(hatch edge 0.5)']
    if z.priority:
        body.append(f'\t\t(priority {int(z.priority)})')
    body.append(f'\t\t(connect_pads{conn}\n\t\t\t(clearance {clr:g})\n\t\t)')
    if z.min_thickness is not None:
        body.append(f'\t\t(min_thickness {z.min_thickness:g})')
    if not use_net_name:
        body.append('\t\t(filled_areas_thickness no)')
    body.append('\t\t(fill ' + ' '.join(fill) + ')')
    body.append(poly(z.polygon))
    body.extend(poly(h) for h in (getattr(z, 'holes', None) or ()))
    return '\t(zone\n' + '\n'.join(body) + '\n\t)'


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
            # The file's own protection spec rides on the kipy via, as KiCad
            # reports it over IPC -- so the plugin's live read sees it.
            k = make_via(self.net_map, v.x, v.y, v.size, v.drill,
                         top_layer=layers[0], bottom_layer=layers[-1],
                         net_name=self._name_of.get(v.net_id),
                         tenting_attrs=v.tenting_attrs)
            self._src[id(k)] = v
            self._vias.append(k)

        self._footprints = [
            _FakeFootprint(ref, fp.x, fp.y, fp.rotation,
                           layer_name=getattr(fp, 'layer', None),
                           locked=bool(getattr(fp, 'locked', False)),
                           ref_label=getattr(fp, 'ref_label', None))
            for ref, fp in self.pcb_data.footprints.items()
        ]
        # One stand-in per physical zone (see _FakeZone): the parser's
        # per-layer Zones regrouped by uuid. A footprint's own zones are not
        # board zones (kipy's get_zones does not list them either).
        by_zone = {}
        for z in self.pcb_data.zones:
            if getattr(z, 'in_footprint', False):
                continue
            key = getattr(z, 'uuid', '') or ('#', id(z))
            ent = by_zone.setdefault(key, [z, []])
            if z.layer not in ent[1]:
                ent[1].append(z.layer)
        self._zones = [
            _FakeZone(self._name_of.get(z.net_id, ''), layers,
                      uuid=getattr(z, 'uuid', ''),
                      teardrop=bool(getattr(z, 'teardrop', False)),
                      parsed=z)
            for z, layers in by_zone.values()
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
        locks, unlocks, labelled = [], [], []
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
            p = {'reference': ref, 'new_x': x, 'new_y': y, 'new_rotation': rot}
            # Side, lock and label: only what the applier CHANGED, so a plain
            # pose update writes exactly the placement it always wrote.
            side_changed = lock_changed = label_changed = False
            if isinstance(fp, _FakeFootprint):
                from kipy.board_types import BoardLayer
                back = fp.layer == BoardLayer.BL_B_Cu
                if back != fp._back:
                    p['new_side'] = 'B' if back else 'F'
                    side_changed = True
                if fp.locked != fp._seed_locked:
                    (locks if fp.locked else unlocks).append(ref)
                    lock_changed = True
                if fp.label_changed():
                    labelled.append((fp, x, y, rot))
                    label_changed = True
            parsed = self.pcb_data.footprints.get(ref)
            pose_same = (parsed is not None and not side_changed
                         and abs(parsed.x - x) < 1e-6 and abs(parsed.y - y) < 1e-6
                         and abs(((parsed.rotation or 0) - rot + 180) % 360 - 180) < 1e-6)
            if not (pose_same and (lock_changed or label_changed)):
                placements.append(p)
        if placements:
            write_placed_output(self.path, self.path, placements)
        if locks or unlocks:
            from placement.seeder import stamp_locked, stamp_unlocked
            if locks:
                stamp_locked(self.path, locks)
            if unlocks:
                stamp_unlocked(self.path, unlocks)
        if labelled:
            self._flush_labels(labelled)
        return len(placements)

    def _flush_labels(self, labelled):
        """Write the Reference fields an applier changed into the file.

        KiCad takes the field's position in BOARD coordinates and stores it in
        the footprint's frame; this inverts it with the parser's own
        transform (`_global_to_local`, against the footprint's pose after this
        commit) and writes it with the CLI's label writer, which writes a
        centred label of one size (height). What it does NOT model: KiCad's
        own handling of the angle, alignment and a B-side mirror over IPC --
        only a live KiCad proves those."""
        from types import SimpleNamespace
        from kicad_parser import _global_to_local
        from placement.writer import write_label_output
        from kicad_ipc_adapter import _vec_xy_mm
        results = []
        for fp, x, y, rot in labelled:
            t = fp.reference_field.text
            a = t.attributes
            wx_, wy_ = _vec_xy_mm(t.position)
            lx, ly = _global_to_local(x, y, rot, wx_, wy_)
            results.append(SimpleNamespace(
                reference=fp.reference, changed=True,
                at_x=round(lx, 6), at_y=round(ly, 6),
                file_rotation=float(a.angle or 0.0),
                size=a.size.y / 1e6, thickness=a.stroke_width / 1e6))
        write_label_output(self.path, self.path, results)

    def refill_zones(self, *_a, **_k):
        """kipy's `Board.refill_zones`: KiCad's own zone filler over every
        zone, as pressing B does. Here pcbnew's ZONE_FILLER on the file, saved
        through `tool_records.save_board` (#1906), so a pour the plugin
        created is filled as KiCad fills it after the commit. Raises without
        pcbnew -- a fake that cannot fill must not pass for one that did."""
        import pcbnew
        import tool_records
        self.refill_calls = getattr(self, 'refill_calls', 0) + 1
        b = pcbnew.LoadBoard(self.path)
        zones = list(b.Zones())
        if zones:
            pcbnew.ZONE_FILLER(b).Fill(zones)
        tool_records.save_board(pcbnew.SaveBoard, self.path, b, aSkipSettings=True)
        self._reseed()

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
                # The type and span the plugin wrote (#1268): a blind or micro
                # via the plugin re-created used to land in the file THROUGH.
                vtype = kipy_via_type(it)
                vias.append({'x': x, 'y': y,
                             'size': getattr(it, 'diameter', 0) / 1e6,
                             'drill': getattr(it, 'drill_diameter', 0) / 1e6,
                             'layers': list(kipy_via_layers(it, vtype)),
                             'via_type': vtype,
                             'net_id': _nid(_name(it)),
                             # What the plugin wrote onto the padstack, in
                             # the form KiCad saves it (probed on 10.0.0).
                             'tenting_attrs': kipy_via_protection_live(it) or {}})

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
        _k10 = self.pcb_data.kicad_version >= KICAD_10_MIN_VERSION
        for it in self._added:
            zd = _ZONE_SRC.get(id(it))
            if zd is None:
                # A zone built without make_zone (result_apply.make_pour):
                # written from the kipy object itself. It used to be dropped
                # here in silence, so an Apply's pours never reached the file.
                if type(it).__name__ == 'Zone':
                    net = getattr(it, 'net', None)
                    sx = _zone_sexpr_from_kipy(
                        it, _nid(getattr(net, 'name', '') if net is not None else ''),
                        _k10)
                    if sx is None:
                        print("  FakeIpcBoard: a created zone that is no copper "
                              "pour was NOT written")
                    else:
                        self._zone_sexprs.append(sx)
                continue
            if zd.get('keepout'):
                from kicad_writer import generate_keepout_zone_sexpr
                self._zone_sexprs.append(generate_keepout_zone_sexpr(
                    zd['layers'], zd['polygon_mm'], zd['name'],
                    use_net_name=_k10))
                continue
            self._zone_sexprs.append(generate_zone_sexpr(
                net_id=_nid(zd['net_name']),
                net_name=zd['net_name'],
                layer=zd['layer'],
                polygon_points=zd['polygon_mm'],
                clearance=zd['clearance_mm'],
                min_thickness=zd['min_thickness_mm'],
                direct_connect=not zd.get('thermal_relief', False),
                priority=int(zd.get('priority', 0) or 0),
                use_net_name=_k10,
            ))
        # Zones the plugin REMOVED (a replaced pour, a78cc7d2), by uuid.
        rm_zone_uuids = {getattr(it, 'uuid', '') for it in self._removed
                         if isinstance(it, _FakeZone)} - {''}

        zone_sexpr = '\n'.join(self._zone_sexprs) if self._zone_sexprs else None
        ok = write_plane_output(
            self.path, self.path, zone_sexpr, vias, segs,
            net_id_to_name=self._name_of,
            removed_segments=rm_segs or None)
        if not ok:
            raise RuntimeError("FakeIpcBoard: write_plane_output failed")

        if rm_vias:
            self._strip_vias(rm_vias)
        if rm_zone_uuids:
            self._strip_zones(rm_zone_uuids)
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

    def _strip_zones(self, uuids):
        """Delete the file's top-level `(zone ...)` blocks with these uuids.
        Unmatched uuids are REPORTED, as _strip_vias reports its misses."""
        import re
        with open(self.path, 'r', encoding='utf-8') as f:
            content = f.read()
        out, i, found = [], 0, set()
        for m in re.finditer(r'\(zone[\s\n]', content):
            if m.start() < i:
                continue
            depth, j = 0, m.start()
            while j < len(content):
                c = content[j]
                if c == '(':
                    depth += 1
                elif c == ')':
                    depth -= 1
                    if depth == 0:
                        break
                j += 1
            block = content[m.start():j + 1]
            u = re.search(r'\(uuid\s+"?([0-9a-fA-F-]+)"?\)', block)
            if u and u.group(1) in uuids:
                found.add(u.group(1))
                out.append(content[i:m.start()].rstrip(' \t'))
                i = j + 1
                if content[i:i + 1] == '\n':
                    i += 1
        out.append(content[i:])
        missing = set(uuids) - found
        if missing:
            print(f"  FakeIpcBoard: {len(missing)} removed zone(s) could not be "
                  f"matched in the file and were NOT stripped")
        with open(self.path, 'w', encoding='utf-8') as f:
            f.write(''.join(out))

    # -- misc --------------------------------------------------------------

    def add_zone_sexpr(self, sexpr):
        """Queue a zone s-expression for the next push (planes tab)."""
        self._zone_sexprs.append(sexpr)

    def save_as(self, path, overwrite=True, include_project=True):
        """kipy's `Board.save_as` (SaveCopyOfDocument), on the file.

        `include_project` defaults True, as kipy's does, and then copies the
        board's `.kicad_pro` beside the copy -- what KiCad 10.0.0 writes
        (probed: `.kicad_pcb` + `.kicad_pro`, not the `.kicad_prl`). The
        `.kicad_dru` goes too when there is one: the graders read it, and the
        CLI leg carries it.

        This used to ignore the flag and copy the board alone, so every GUI
        step snapshot was graded with NO project rule at all while the CLI leg
        was graded against the `.kicad_pro` it wrote. On rp2350 that read as
        "GUI create clean, CLI create 10 TRACK-HOLE" for copper the two fronts
        shared -- the #441 trap, in the harness.
        """
        import shutil
        dst = os.path.abspath(path)
        if dst != self.path:
            shutil.copy(self.path, dst)
            if include_project:
                src_stem = os.path.splitext(self.path)[0]
                dst_stem = os.path.splitext(dst)[0]
                for ext in ('.kicad_pro', '.kicad_dru'):
                    if os.path.isfile(src_stem + ext):
                        shutil.copy(src_stem + ext, dst_stem + ext)
        return True


def install(board_path):
    """Point `kicad_ipc_adapter` (and the PCBData builder) at a FakeIpcBoard.

    Returns the board. This is the IPC analogue of the SWIG harness's
    `pcbnew.GetBoard = lambda: board`: the plugin keeps calling
    `get_board()` / `build_pcb_data_from_board()` exactly as in production.

    With `UNSAVED_ENV` set to 1 the board is an UNSAVED document, as in a
    real KiCad: see `_unsaved_live_copy`.
    """
    import kicad_ipc_adapter as adapter
    import kicad_parser

    unsaved = os.environ.get(UNSAVED_ENV) == '1'
    board = FakeIpcBoard(_unsaved_live_copy(board_path) if unsaved else board_path)

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
            'priority': kw.get('priority', 0),
            'thermal_relief': kw.get('thermal_relief', False),
        }
        return z

    adapter.make_zone = _make_zone

    _real_make_keepout = adapter.make_keepout_zone

    def _make_keepout_zone(polygon_mm, layer_names, name=''):
        z = _real_make_keepout(polygon_mm, layer_names, name=name)
        _ZONE_SRC[id(z)] = {'keepout': True, 'polygon_mm': list(polygon_mm),
                            'layers': list(layer_names), 'name': name}
        return z

    adapter.make_keepout_zone = _make_keepout_zone

    adapter.get_board = lambda *a, **k: board
    adapter.connect = lambda *a, **k: None
    adapter.get_board_full_path = lambda *a, **k: board.path
    # include_project=False, as the real save_board_snapshot passes it: the
    # oracle's temp copy must carry NO project (it stages the real one through
    # project_from, #490).
    adapter.save_board_snapshot = lambda path, *a, **k: board.save_as(
        path, include_project=False)
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

    if unsaved:
        # The document is `board_path`: the adapter's path (every writeback
        # and the plan's end-of-plan write read the board FILE there, which
        # holds the copper from before the run) and PCBData.source_path (the
        # real builder's `_ipc_board_path`). The copper lives in board.path.
        doc = os.path.abspath(board_path)
        board.document_path = doc
        adapter.get_board_full_path = lambda *a, **k: doc

        def _build_unsaved(b, *a, **k):
            pcb = _build(b, *a, **k)
            if isinstance(b, FakeIpcBoard):
                pcb.source_path = doc
            return pcb

        kicad_parser.build_pcb_data_from_board = _build_unsaved
        import atexit
        atexit.register(_save_unsaved_document, board)
    return board


#: Opt-in: `install` makes the board an UNSAVED document.
UNSAVED_ENV = 'KRT_FAKE_IPC_UNSAVED'


def _unsaved_live_copy(board_path):
    """The file a FakeIpcBoard edits when the document is unsaved.

    A real KiCad holds the routed copper in memory and leaves the board FILE
    as it was opened until the user saves, so whatever the plugin reads off
    the file (`fix_project_for_output`'s scan behind each writeback, the
    plan's end-of-plan write) sees the copper from BEFORE the run. The
    default harness flushes every commit to the document itself, so such a
    read sees the step's copper and a writeback that only works on a saved
    file passes here. Opt-in (`UNSAVED_ENV`): the commits go to a copy in a
    `.fake_ipc_live` directory beside the board, whose project siblings are
    symlinks to the document's, so a rule the plugin writes beside the
    document is the one the engines read; the document is overwritten with
    the copy at exit (`_save_unsaved_document`, the user's save)."""
    import shutil
    from copy_board import SIBLING_EXTS
    doc = os.path.abspath(board_path)
    live_dir = os.path.join(os.path.dirname(doc), '.fake_ipc_live')
    os.makedirs(live_dir, exist_ok=True)
    live = os.path.join(live_dir, os.path.basename(doc))
    shutil.copy(doc, live)
    doc_stem, live_stem = os.path.splitext(doc)[0], os.path.splitext(live)[0]
    for ext in SIBLING_EXTS:
        if os.path.lexists(live_stem + ext):
            os.remove(live_stem + ext)
        os.symlink(doc_stem + ext, live_stem + ext)
    print(f"FakeIpcBoard: unsaved document {os.path.basename(doc)}; the "
          f"commits go to {live}")
    return live


def _save_unsaved_document(board):
    """The user's File > Save at the end of an unsaved run: the board file
    only (KiCad's own project save is not modelled)."""
    import shutil
    doc = getattr(board, 'document_path', None)
    if doc and os.path.isfile(board.path):
        shutil.copy(board.path, doc)
        print(f"FakeIpcBoard: saved the unsaved document to {doc}")


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
