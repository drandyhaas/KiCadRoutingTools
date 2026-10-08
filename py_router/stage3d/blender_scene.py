"""The stage3d board in Blender (#1089) -- run INSIDE Blender, never imported.

    blender -b --factory-startup -P blender_scene.py -- job.json

`job.json` is the three.js backend's own job (`render3d._job`): the SAME
`scene.json` and `timeline.json`, so the two backends cannot disagree about
what happens on which frame -- only about how it looks. Every distinct
timeline state is rendered to `<outDir>/s%06d.png`, one JSON line per event
on stdout (`{"type": "info"|"progress"|"done"|"error"}`), exactly like
`render.mjs`, so `render3d` reads both the same way.

Cycles on the CPU with a FIXED seed, fixed samples, no denoiser and no stamp:
the same inputs on the same machine render the same bytes. A GPU device is
never used -- as with SwiftShader in the three.js backend, a render that
depends on the machine's graphics driver cannot be pinned by a test.

World frame: KiCad's board (x, y-down) maps to Blender (X, -Y) with Z UP out
of the board's top face, so a KiCad rotation `rot` is `Rz(rot)`, and a back-
side part is `Rz(rot) Rx(180)` -- the same frames kicad-cli's GLB carries
(glTF's y-up rotated into Blender's z-up by the importer).
"""
import json
import math
import os
import sys

# bpy, bmesh and mathutils exist only INSIDE Blender; they are imported by
# `_blender()` at run time, so no module-scope import pretends this file is
# a dependency of the repo (tests/test_887_small_items checks requirements).
bmesh = bpy = Matrix = Vector = None


def _blender():
    global bmesh, bpy, Matrix, Vector
    import bmesh as _bm
    import bpy as _bpy
    from mathutils import Matrix as _M, Vector as _V
    bmesh, bpy, Matrix, Vector = _bm, _bpy, _M, _V

SAMPLES = int(os.environ.get('KICAD_STAGE3D_BLENDER_SAMPLES', '24'))


def out(o):
    sys.stdout.write(json.dumps(o) + '\n')
    sys.stdout.flush()


def srgb(c):
    """A theme's sRGB byte triple as Blender's LINEAR colour."""
    def lin(v):
        v = v / 255.0
        return v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4
    return (lin(c[0]), lin(c[1]), lin(c[2]), 1.0)


def mat(name, rgba, metal=0.0, rough=0.6, alpha=1.0, emit=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes['Principled BSDF']
    b.inputs['Base Color'].default_value = rgba
    b.inputs['Metallic'].default_value = metal
    b.inputs['Roughness'].default_value = rough
    b.inputs['Alpha'].default_value = alpha
    if emit:
        b.inputs['Emission Color'].default_value = rgba
        b.inputs['Emission Strength'].default_value = emit
    return m


def wp(x, y, z=0.0):
    """A KiCad board point (x, y-down) at height z, in Blender's world."""
    return Vector((x, -y, z))


def mesh_obj(name, verts, faces, material, parent=None):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    ob = bpy.data.objects.new(name, me)
    ob.data.materials.append(material)
    bpy.context.collection.objects.link(ob)
    if parent is not None:
        ob.parent = parent
    return ob


def prism(name, poly, z0, z1, material, parent=None):
    """An extruded polygon (KiCad board coordinates), z0..z1."""
    bm = bmesh.new()
    bot = [bm.verts.new(wp(x, y, z0)) for x, y in poly]
    face = bm.faces.new(bot)
    ext = bmesh.ops.extrude_face_region(bm, geom=[face])
    top = [v for v in ext['geom'] if isinstance(v, bmesh.types.BMVert)]
    bmesh.ops.translate(bm, verts=top, vec=Vector((0, 0, z1 - z0)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    ob.data.materials.append(material)
    bpy.context.collection.objects.link(ob)
    if parent is not None:
        ob.parent = parent
    return ob


def layer_z(li, n, d):
    if li == 0:
        return d + 0.035
    if li == n - 1:
        return -0.035
    return d * (1 - li / float(n - 1))


def seg_quads(rows, z, verts, faces):
    for r in rows:
        sx, sy, ex, ey, w = r[0], r[1], r[2], r[3], r[4]
        dx, dy = ex - sx, ey - sy
        L = math.hypot(dx, dy) or 1e-6
        dx, dy = dx / L, dy / L
        hw = max(w, 0.05) / 2
        nx, ny = -dy * hw, dx * hw
        ax, ay, bx, by = sx - dx * hw, sy - dy * hw, ex + dx * hw, ey + dy * hw
        k = len(verts)
        verts += [wp(ax + nx, ay + ny, z), wp(ax - nx, ay - ny, z),
                  wp(bx - nx, by - ny, z), wp(bx + nx, by + ny, z)]
        faces.append((k, k + 1, k + 2, k + 3))


def main():
    _blender()
    job = json.load(open(sys.argv[sys.argv.index('--') + 1], encoding='utf-8'))
    scene = json.load(open(job['scene'], encoding='utf-8'))
    tl = json.load(open(job['timeline'], encoding='utf-8'))
    col = job['colors']
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.engine = 'CYCLES'
    sc.cycles.device = 'CPU'
    sc.cycles.samples = SAMPLES
    sc.cycles.seed = 0
    sc.cycles.use_denoising = False
    sc.cycles.use_adaptive_sampling = False
    sc.render.use_stamp = False
    # no METADATA either: Blender writes the render date and times into the
    # PNG, so two identical pictures were different files
    for _k in dir(sc.render):
        if _k.startswith('use_stamp') and isinstance(
                getattr(sc.render, _k, None), bool):
            try:
                setattr(sc.render, _k, False)
            except (AttributeError, TypeError):
                pass
    sc.render.resolution_x, sc.render.resolution_y = job['width'], job['height']
    sc.render.resolution_percentage = 100
    sc.render.image_settings.file_format = 'PNG'
    sc.render.image_settings.color_mode = 'RGB'
    sc.render.film_transparent = False
    sc.view_settings.view_transform = 'Standard'
    world = bpy.data.worlds.new('w')
    world.use_nodes = True
    world.node_tree.nodes['Background'].inputs['Color'].default_value = \
        srgb(col['ground'])
    world.node_tree.nodes['Background'].inputs['Strength'].default_value = 1.0
    sc.world = world

    d = scene['thickness']
    n = len(tl['layers'])
    x0, y0, x1, y1 = scene['bounds']
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    # the flip pivot: the board turns about its own long screen axis,
    # through its centre -- Blender's world Y is the board's y
    pivot = bpy.data.objects.new('pivot', None)
    bpy.context.collection.objects.link(pivot)
    pivot.location = wp(cx, cy, d / 2)
    root = bpy.data.objects.new('root', None)
    bpy.context.collection.objects.link(root)
    root.parent = pivot
    root.location = -wp(cx, cy, d / 2)

    board_m = mat('board', srgb(col['board']), rough=0.8)
    prism('board', scene['outline'], 0.0, d, board_m, root)
    pad_m = mat('pad', srgb(col['pad']), metal=0.8, rough=0.35)
    body_m = mat('body', srgb(col['body']), rough=0.7)
    hole_m = mat('hole', srgb(col.get('hole', [20, 20, 22])), rough=0.9)
    hl_m = mat('hilite', srgb(col['hilite']), emit=0.6)
    layer_m = [mat('L%d' % i, srgb(c), metal=0.6, rough=0.4)
               for i, c in enumerate(col['layers'])]
    via_m = mat('via', srgb(col['via']), metal=0.7, rough=0.4)

    # parts: an empty per part, its pads and body as children
    parts = {}
    for ref in sorted(scene['parts']):
        P = scene['parts'][ref]
        e = bpy.data.objects.new('part_' + ref, None)
        bpy.context.collection.objects.link(e)
        e.parent = root
        faces_ = {}
        for f in ('F', 'B'):
            g = bpy.data.objects.new('face_%s_%s' % (f, ref), None)
            bpy.context.collection.objects.link(g)
            g.parent = e
            faces_[f] = g
        for p in P['pads']:
            lx, ly, sx, sy, ang, shape, face = p[:7]
            polys = p[9] if len(p) > 9 else None
            for f in (('F', 'B') if face == 'T' else (face,)):
                z0, z1 = (0.0, 0.05) if f == 'F' else (-0.05, 0.0)
                if polys:
                    for poly in polys:
                        prism('pad', poly, z0, z1, pad_m, faces_[f])
                else:
                    a = math.radians(ang)
                    cs, sn = math.cos(a), math.sin(a)
                    corners = []
                    for ux, uy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2),
                                   (sx / 2, sy / 2), (-sx / 2, sy / 2)):
                        # the pad rect in the part frame (the renderer's _rot)
                        corners.append((lx + ux * cs - uy * sn,
                                        ly + ux * sn + uy * cs))
                    prism('pad', corners, z0, z1, pad_m, faces_[f])
                drill = p[7] if len(p) > 7 else 0
                if drill and drill > 0:
                    # the drill, at its OWN centre (#1090)
                    hx, hy = (p[8] if len(p) > 8 and p[8] else (lx, ly))
                    ring = [(hx + drill / 2 * math.cos(2 * math.pi * k / 16),
                             hy + drill / 2 * math.sin(2 * math.pi * k / 16))
                            for k in range(16)]
                    hz = (z1, z1 + 0.01) if f == 'F' else (z0 - 0.01, z0)
                    prism('hole', ring, hz[0], hz[1], hole_m, faces_[f])
        body = None
        if P.get('body'):
            bx0, by0, bx1, by1, h = P['body']
            body = prism('body', [(bx0, by0), (bx1, by0), (bx1, by1),
                                  (bx0, by1)], 0.06, 0.06 + h, body_m, e)
        parts[ref] = {'e': e, 'faces': faces_, 'body': body, 'glb': []}

    # the part models, re-posed by F(now) * F(final)^-1 like the page
    def F(x, y, rot, back, face_z):
        m = Matrix.Translation(wp(x, y, face_z)) @ Matrix.Rotation(
            math.radians(rot), 4, 'Z')
        if back:
            m = m @ Matrix.Rotation(math.pi, 4, 'X')
        return m
    if job.get('glb') and scene.get('glb'):
        before = set(bpy.data.objects)
        try:
            bpy.ops.import_scene.gltf(filepath=job['glb'])
            new = [o for o in bpy.data.objects if o not in before]
            want = set(scene['glb']['matched'])
            mm = Matrix.Scale(1000.0, 4)
            byname = {}
            for o in new:
                base = o.name.split('.')[0]
                if base in want and not (o.parent and o.parent.name.split(
                        '.')[0] in want):
                    byname.setdefault(base, []).append(o)
            for base, objs in byname.items():
                keys = [base] + [k for k in ('%s~%d' % (base, i)
                                             for i in range(2, 50))
                                 if k in parts]
                for i, o in enumerate(objs):
                    key = keys[i] if len(objs) == len(keys) else base
                    pose = scene['glb']['poses'].get(key)
                    if not pose or key not in parts:
                        continue
                    world_m = mm @ o.matrix_world
                    rest = F(pose[0], pose[1], pose[2], pose[3] == 'B',
                             0.0 if pose[3] == 'B' else d).inverted() @ world_m
                    # under the board's root, so the model turns over
                    # with the board; root's frame IS the world at rest
                    o.parent = root
                    o.matrix_parent_inverse.identity()
                    parts[key]['glb'].append((o, rest))
        except Exception as exc:                               # noqa: BLE001
            out({'type': 'warn', 'why': 'GLB import failed: %s' % exc})

    # pours, shown from their reveal
    pours = []
    for z in scene.get('pours') or []:
        zz = layer_z(z['layer'], n, d) + (0.01 if z['layer'] == 0 else -0.01)
        ob = prism('pour', z['poly'], zz - 0.005, zz,
                   layer_m[z['layer']] if z['layer'] < len(layer_m)
                   else pad_m, root)
        ob['net'] = z['net']
        pours.append(ob)

    # camera: a 3/4 view from the board's near edge, fitted to every pose
    cam_d = bpy.data.cameras.new('cam')
    cam_d.lens_unit = 'FOV'
    cam_d.angle = math.radians(30 * job['width'] / float(job['height']))
    cam = bpy.data.objects.new('cam', cam_d)
    bpy.context.collection.objects.link(cam)
    sc.camera = cam
    ex0, ey0, ex1, ey1 = x0, y0, x1, y1
    for ep in tl['epochs']:
        for ref, pose in ep.items():
            ex0, ex1 = min(ex0, pose[0] - 2), max(ex1, pose[0] + 2)
            ey0, ey1 = min(ey0, pose[1] - 2), max(ey1, pose[1] + 2)
    r = 0.5 * math.hypot(ex1 - ex0, ey1 - ey0) + 2
    el = math.radians(52)
    dist = r / math.sin(math.radians(15)) * 0.9
    tgt = wp((ex0 + ex1) / 2, (ey0 + ey1) / 2, d / 2)
    cam.location = tgt + Vector((0, -dist * math.cos(el), dist * math.sin(el)))
    cam.rotation_euler = (math.pi / 2 - el, 0, 0)
    sun = bpy.data.lights.new('sun', 'SUN')
    sun.energy = 3.0
    so = bpy.data.objects.new('sun', sun)
    bpy.context.collection.objects.link(so)
    so.rotation_euler = (math.radians(35), math.radians(15), 0)

    segs, vias = tl['segs'], tl['vias']
    copper = []
    os.makedirs(job['outDir'], exist_ok=True)
    out({'type': 'info', 'renderer': 'Blender %s Cycles CPU, %d samples'
         % (bpy.app.version_string, SAMPLES),
         'three': None, 'states': len(tl['states']),
         'glbParts': sum(len(p['glb']) for p in parts.values()),
         'browser': None, 'errors': []})
    import time
    t0 = time.time()
    for si, st in enumerate(tl['states']):
        for ob in copper:
            bpy.data.objects.remove(ob, do_unlink=True)
        copper = []
        hide = set(st['hide'])
        per = {}
        for i, s in enumerate(segs):
            if s[6] < st['ns'] and (s[7] < 0 or s[7] >= st['ns']) \
                    and i not in hide:
                per.setdefault(s[5], []).append(s)
        for li, rows in per.items():
            verts, faces = [], []
            seg_quads(rows, layer_z(li, n, d), verts, faces)
            copper.append(mesh_obj('cu%d' % li, verts, faces,
                                   layer_m[li] if li < len(layer_m)
                                   else pad_m, root))
        vv, vf = [], []
        for v in vias:
            if v[6] < st['nv'] and (v[7] < 0 or v[7] >= st['nv']):
                rr = max(v[2], 0.1) / 2
                k = len(vv)
                for j in range(8):
                    a = 2 * math.pi * j / 8
                    vv.append(wp(v[0] + rr * math.cos(a),
                                 v[1] + rr * math.sin(a), d + 0.045))
                vf.append(tuple(range(k, k + 8)))
        if vv:
            copper.append(mesh_obj('vias', vv, vf, via_m, root))
        if st['hl_s']:
            verts, faces = [], []
            seg_quads(st['hl_s'], d + 0.08, verts, faces)
            copper.append(mesh_obj('hl', verts, faces, hl_m, root))
        on = set(st.get('zones') or [])
        for ob in pours:
            ob.hide_render = ob['net'] not in on
        ep = tl['epochs'][max(0, min(st['epoch'], len(tl['epochs']) - 1))]
        for ref, part in parts.items():
            e = ep.get(ref)
            if not e:
                part['e'].hide_render = True
                continue
            part['e'].hide_render = False       # present again after a gap
            m = st['moving'].get(ref)
            x, y, rot = (m[0], m[1], m[2]) if m else (e[0], e[1], e[2])
            back = str(e[3]).startswith('B')
            lift = 1.2 if m else 0.0
            part['e'].location = wp(x, y, -lift if back else d + lift)
            part['e'].rotation_euler = (0, 0, math.radians(rot))
            part['faces']['F'].location = (0, 0, -d if back else 0)
            part['faces']['B'].location = (0, 0, 0 if back else -d)
            if part['body'] is not None:
                part['body'].scale = (1, 1, -1 if back else 1)
                part['body'].hide_render = bool(part['glb'])
            for o, rest in part['glb']:
                o.matrix_basis = F(x, y, rot, back,
                                   0.0 if back else d) @ rest
        pivot.rotation_euler = (0, st['angle'], 0)
        fit_camera(sc, cam, pivot, el)
        sc.render.filepath = os.path.join(job['outDir'], 's%06d.png' % si)
        bpy.ops.render.render(write_still=True)
        if si % 10 == 9:
            out({'type': 'progress', 'done': si + 1,
                 'of': len(tl['states'])})
    out({'type': 'done', 'states': len(tl['states']),
         'ms_per_state': 1000.0 * (time.time() - t0)
         / max(1, len(tl['states']))})


def fit_camera(sc, cam, pivot, el):
    """Fit the camera to what THIS state renders, as the three.js page does
    (`fitCamera`): the film-wide fit had to cover the pile beside the board
    and a mid-flip board, and drew esp_prog at about a third of its box
    (run 35). Pure in the state, so a render stays deterministic."""
    from bpy_extras.object_utils import world_to_camera_view
    bpy.context.view_layer.update()

    def shown(o):
        while o is not None:
            if o.hide_render:
                return False
            if o is pivot:
                return True
            o = o.parent
        return False
    # a pour is built from the zone OUTLINE, which may run far past the board
    # (KiCad clips the fill), so it never sets the fit
    pts = [o.matrix_world @ Vector(c) for o in bpy.data.objects
           if o.type == 'MESH' and 'net' not in o and shown(o)
           for c in o.bound_box]
    if not pts:
        return
    lo = Vector([min(p[i] for p in pts) for i in range(3)])
    hi = Vector([max(p[i] for p in pts) for i in range(3)])
    tgt = (lo + hi) / 2
    corners = [Vector((x, y, z)) for x in (lo.x, hi.x) for y in (lo.y, hi.y)
               for z in (lo.z, hi.z)]
    dist = (hi - lo).length / math.tan(math.radians(15))
    for _k in range(5):
        cam.location = tgt + Vector((0, -dist * math.cos(el),
                                     dist * math.sin(el)))
        bpy.context.view_layer.update()
        ext = max(max(abs(v.x - 0.5), abs(v.y - 0.5)) * 2
                  for v in (world_to_camera_view(sc, cam, c)
                            for c in corners))
        dist *= ext / 0.92                  # 8 % margin
    cam.location = tgt + Vector((0, -dist * math.cos(el),
                                 dist * math.sin(el)))


try:
    main()
except Exception as exc:                                       # noqa: BLE001
    import traceback
    out({'type': 'error', 'why': '%s: %s' % (
        type(exc).__name__, str(exc).splitlines()[0][:200] if str(exc)
        else traceback.format_exc().splitlines()[-1])})
    sys.exit(2)
