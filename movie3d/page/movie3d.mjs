// The 3D placement movie's page (prototype). A pure function of (scene,
// timeline, frame index): `m3dFrame(i)` poses every part, draws the airwires,
// places the camera and renders once. Nothing reads a clock.
//
// Frames (stage3d's, #1081): KiCad's board (x, y-down) maps to three (x, z)
// with y UP out of the board's top face, so a KiCad rotation `rot` is
// `rotation.y = rad(rot)` (checked against placement.legality.pads_at_pose).
// The board's bottom face is y = 0, its top face y = d; the table it stands
// over is y = scene.table. Each part's geometry is in its own frame (its pose
// (0, 0, 0)), and a frame's pose row is [x, y, rot, h]: h is the height of the
// part's mounting face (the board's face, the table, or mid-hop).
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

const S = {};
const rad = (d) => d * Math.PI / 180;
// theme colours are sRGB bytes: say so, or three treats them as linear
const rgb = (c) => new THREE.Color().setRGB(c[0] / 255, c[1] / 255, c[2] / 255, THREE.SRGBColorSpace);
const css = (c) => `rgb(${c[0]},${c[1]},${c[2]})`;
const PAD_T = 0.04;

// The board is see-through (scene `board_alpha`, or a frame's `alpha`), and
// it dims exactly what lies behind it: the opaque scene is drawn first, then
// whatever is see-through behind the board, then the board, then whatever
// is see-through in front of it. Each see-through thing that writes no depth
// (a courtyard plate, a fixed part's ghost, a pour) is in S.sided with its
// draw order and the face it lies on (+1 top, -1 bottom, 0 inside); every
// frame it goes before the board or after it by which face the camera sees
// (orderBySide), which turns over when the board does.
const BOARD_ORDER = 6, NEAR = 10;
// a fixed part's ghost, its depth only: drawn just before the ghost itself
// (loadModels), and see-through so it is drawn in the board's pass, not before it
const GHOST_DEPTH = new THREE.MeshBasicMaterial({ colorWrite: false, depthWrite: true, transparent: true });
function sided(o, order, side) {
  o.renderOrder = order + (side > 0 ? NEAR : 0);     // seen from above (a seed grid's clones keep it)
  S.sided.push([o, order, side]);
}

// ---------------------------------------------------------------- board
function buildBoard(sc) {
  const d = sc.thickness, C = sc.colors;
  const shape = new THREE.Shape(sc.outline.map((p) => new THREE.Vector2(p[0], p[1])));
  for (const ring of sc.cutouts) shape.holes.push(new THREE.Path(ring.map((p) => new THREE.Vector2(p[0], p[1]))));
  const geo = new THREE.ExtrudeGeometry(shape, { depth: d, bevelEnabled: false, curveSegments: 24 });
  geo.rotateX(Math.PI / 2);           // shape (x, y) -> three (x, z); depth -> -y
  geo.translate(0, d, 0);             // y in [0, d]
  // see-through, so the back's parts, airwires and copper show through it
  const board = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: rgb(C.board), roughness: 0.85,
                                                                     metalness: 0.0, transparent: true }));
  board.renderOrder = BOARD_ORDER;
  S.boardMat = board.material;
  S.boardMesh = board;
  board.receiveShadow = true;
  board.castShadow = true;
  const lm = new THREE.LineBasicMaterial({ color: rgb(C.edge), transparent: true, opacity: 0.55 });
  for (const ring of [sc.outline, ...sc.cutouts]) {
    const pts = ring.map((p) => new THREE.Vector3(p[0], d + 0.01, p[1]));
    board.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(pts), lm));
  }
  return board;
}

function buildTable(sc) {
  const [x0, y0, x1, y1] = sc.bounds;
  const span = 40 * Math.max(x1 - x0, y1 - y0, 20);     // past the frame, even round the seed grid
  const g = new THREE.PlaneGeometry(span, span);
  g.rotateX(-Math.PI / 2);
  const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: rgb(sc.colors.table), roughness: 1.0 }));
  m.position.set((x0 + x1) / 2, sc.table, (y0 + y1) / 2);
  m.receiveShadow = true;
  return m;
}

// ---------------------------------------------------------------- parts
const _geo = {};
function padGeo(sx, sy, shape) {
  const k = [sx, sy, shape].join('/');
  if (_geo[k]) return _geo[k];
  let g;
  if (shape === 'circle') {
    g = new THREE.CylinderGeometry(Math.max(sx, sy) / 2, Math.max(sx, sy) / 2, PAD_T, 24);
  } else if (shape === 'oval' || shape === 'roundrect') {
    // a stadium (oval) or a rectangle with rounded corners, extruded flat
    const r = shape === 'oval' ? Math.min(sx, sy) / 2 : 0.25 * Math.min(sx, sy);
    const w = sx / 2, h = sy / 2, s = new THREE.Shape();
    s.moveTo(-w + r, -h); s.lineTo(w - r, -h); s.absarc(w - r, -h + r, r, -Math.PI / 2, 0);
    s.lineTo(w, h - r); s.absarc(w - r, h - r, r, 0, Math.PI / 2);
    s.lineTo(-w + r, h); s.absarc(-w + r, h - r, r, Math.PI / 2, Math.PI);
    s.lineTo(-w, -h + r); s.absarc(-w + r, -h + r, r, Math.PI, 1.5 * Math.PI);
    g = new THREE.ExtrudeGeometry(s, { depth: PAD_T, bevelEnabled: false, curveSegments: 8 });
    g.rotateX(Math.PI / 2);
    g.translate(0, PAD_T / 2, 0);
  } else {
    g = new THREE.BoxGeometry(Math.max(sx, 0.05), PAD_T, Math.max(sy, 0.05));
  }
  return (_geo[k] = g);
}

// a custom pad's copper, its polygons in the part's frame, extruded flat
function polyGeo(rings) {
  const shapes = rings.map((ring) => {
    const s = new THREE.Shape();
    ring.forEach(([x, y], i) => (i ? s.lineTo(x, y) : s.moveTo(x, y)));
    return s;
  });
  const g = new THREE.ExtrudeGeometry(shapes, { depth: PAD_T, bevelEnabled: false });
  g.rotateX(Math.PI / 2);
  g.translate(0, PAD_T / 2, 0);
  return g;
}

function rectLoop(r, y, mat) {
  const [x0, y0, x1, y1] = r;
  const pts = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map((p) => new THREE.Vector3(p[0], y, p[1]));
  return new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(pts), mat);
}

function buildParts(sc, root) {
  const d = sc.thickness, C = sc.colors;
  // a part's model sits on its pads with its leads and exposed pad microns
  // from the pad copper drawn here, too close for the depth buffer: seen
  // from below through the board they fought, speckling under a QFN's exposed
  // pad as the camera moved. The copper wins the tie, as on a board, where
  // it lies between the board and the part.
  const tie = { polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -4 };
  const padMat = new THREE.MeshStandardMaterial({ color: rgb(C.pad), roughness: 0.3, metalness: 0.75, ...tie });
  const holeMat = new THREE.MeshBasicMaterial({ color: rgb(C.hole), ...tie });
  const soft = sc.soft ?? 0.4;     // how far a box, plate or outline's colour goes toward neutral
  const grey = rgb([120, 120, 126]), board = rgb(C.board);
  S.parts = [];
  for (const P of sc.parts) {
    const grp = new THREE.Group();
    const col = rgb(P.color);
    const up = P.side === 'F' ? 1 : -1;          // the part's own face looks this way
    const near = [], far = [];
    for (const p of P.pads) {
      const [lx, ly, sx, sy, tilt, shape, face, drill, hx, hy, polys] = p;
      // world faces this pad has copper on: top (+1) and/or bottom (-1)
      const dirs = face === 'T' ? [1, -1] : face === 'F' ? [1] : face === 'B' ? [-1] : [];
      for (const dir of dirs) {
        // local height of that face: the part's own face is 0
        const y0 = dir === up ? 0 : -up * d;
        const m = new THREE.Mesh(polys ? polyGeo(polys) : padGeo(sx, sy, shape), padMat);
        m.position.set(polys ? 0 : lx, y0 + dir * (PAD_T / 2 + 0.003), polys ? 0 : ly);
        m.rotation.y = rad(tilt);
        m.castShadow = false;
        m.receiveShadow = true;
        grp.add(m);
        (dir === up ? near : far).push(m);
      }
      if (drill > 0) {
        // the drill at its own centre, on the part's own face (and the far
        // face for a plated hole)
        for (const dir of (face === 'N' ? [up] : dirs)) {
          const y0 = dir === up ? 0 : -up * d;
          const h = new THREE.Mesh(new THREE.CircleGeometry(drill / 2, 20), holeMat);
          h.rotation.x = dir > 0 ? -Math.PI / 2 : Math.PI / 2;
          h.position.set(hx ?? lx, y0 + dir * (PAD_T + 0.008), hy ?? ly);
          grp.add(h);
          (dir === up ? near : far).push(h);
        }
      }
    }
    // the courtyard: a tinted plate and its outline, on the part's own face
    const [cx0, cy0, cx1, cy1] = P.court;
    const plate = new THREE.PlaneGeometry(Math.max(cx1 - cx0, 0.05), Math.max(cy1 - cy0, 0.05));
    plate.rotateX(up > 0 ? -Math.PI / 2 : Math.PI / 2);
    const pm = new THREE.Mesh(plate, new THREE.MeshBasicMaterial({
      color: col, transparent: true, opacity: P.fixed ? 0.1 : (sc.plate ?? 0.2), depthWrite: false }));
    pm.position.set((cx0 + cx1) / 2, up * 0.006, (cy0 + cy1) / 2);
    sided(pm, 1, up);
    grp.add(pm);
    const line = rectLoop(P.court, up * 0.012,
                          new THREE.LineBasicMaterial({ color: col.clone().lerp(board, soft) }));
    grp.add(line);
    let boxed = null;
    if (P.body) {
      const [x0, y0, x1, y1, h] = P.body;
      const bg = new THREE.BoxGeometry(Math.max(x1 - x0, 0.1), h, Math.max(y1 - y0, 0.1));
      // a fixed part's body is ghosted: it never moves, and must not hide
      // the parts that do; so is a cover's (#1443), the parts under it shown
      const ghost = P.fixed || P.cover;
      const body = new THREE.Mesh(bg, new THREE.MeshStandardMaterial({
        color: P.fixed ? rgb([150, 150, 156]) : col.clone().lerp(grey, soft), roughness: 0.55, metalness: 0.05,
        transparent: ghost, opacity: ghost ? 0.32 : 1.0, depthWrite: !ghost }));
      if (ghost) sided(body, 3, up);
      body.position.set((x0 + x1) / 2, up * (h / 2 + PAD_T * 0.5), (y0 + y1) / 2);
      body.castShadow = true;
      body.receiveShadow = true;
      grp.add(body);
      boxed = body;
    }
    // a declared cover's body (#1443: a card in its socket, which has no
    // model of its own): a see-through slab from its underside up, moving
    // with its connector
    if (P.card) {
      const [x0, y0, x1, y1, under] = P.card;
      const t = 0.8;
      const cg = new THREE.BoxGeometry(Math.max(x1 - x0, 0.1), t, Math.max(y1 - y0, 0.1));
      const slab = new THREE.Mesh(cg, new THREE.MeshStandardMaterial({
        color: col.clone().lerp(rgb([200, 210, 255]), 0.5), roughness: 0.6, metalness: 0.0,
        transparent: true, opacity: 0.18, depthWrite: false }));
      slab.position.set((x0 + x1) / 2, up * (under + t / 2 + PAD_T * 0.5), (y0 + y1) / 2);
      sided(slab, 4, up);
      grp.add(slab);
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(cg),
                                           new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: 0.7 }));
      edges.position.copy(slab.position);
      grp.add(edges);
    }
    root.add(grp);
    S.parts.push({ P, grp, near, far, up, boxed, col });
  }
}

function poseParts(fr) {
  const d = S.sc.thickness;
  S.parts.forEach((part, i) => {
    // a fifth value turns the part over about its own x axis: a part drawn
    // by the face it ends on, shown on the face it was on then (record.py)
    const [x, y, rot, h, turn] = fr.p[i];
    part.grp.position.set(x, h, y);
    part.grp.rotation.set(rad(turn || 0), rad(rot), 0, 'YXZ');
    // the far face's copper shows only while the part stands on the board
    const onBoard = Math.abs(h - (part.up > 0 ? d : 0)) < 1e-3;
    for (const m of part.far) m.visible = onBoard;
  });
}

// ---------------------------------------------------------------- models
// The parts' KiCad models (models.py): kicad-cli's GLB holds each part as a
// node named by its reference, at its export pose. Each node is re-parented
// into its part's group as `[Rx(180) for a back part] F(export)^-1 world`,
// so the group's pose carries it (stage3d's re-pose, #1081). Its colours are
// tinted toward the group's (a fixed part's model is ghosted instead), and
// the part's box gives way to it.
// A part's model as one mesh per material, in the model node's own frame.
// A part model is dozens to hundreds of small meshes (each pin, each face of
// a body), and every one is a draw call in the main pass, the shadow pass and
// a ghost's depth pass: on a 306-model board the frame's JavaScript, not the
// drawing, was the slow half. Merged, the same triangles and materials are a
// handful of calls. A mesh whose attributes do not match its material's
// others, or that carries several materials, stays as it was.
function mergedNode(o) {
  const inv = o.matrixWorld.clone().invert();
  const out = new THREE.Group();
  out.name = o.name;
  const byMat = new Map();
  o.traverse((c) => {
    if (!c.isMesh) return;
    const rel = inv.clone().multiply(c.matrixWorld);
    if (Array.isArray(c.material)) {
      const m = new THREE.Mesh(c.geometry, c.material);
      m.applyMatrix4(rel);
      out.add(m);
      return;
    }
    const g = c.geometry.index ? c.geometry.toNonIndexed() : c.geometry.clone();
    g.applyMatrix4(rel);
    for (const name of Object.keys(g.attributes)) if (name !== 'position' && name !== 'normal' && name !== 'color') g.deleteAttribute(name);
    g.morphAttributes = {};
    const key = c.material.uuid + '|' + Object.keys(g.attributes).sort().join(',');
    if (!byMat.has(key)) byMat.set(key, { mat: c.material, geos: [] });
    byMat.get(key).geos.push(g);
  });
  for (const { mat, geos } of byMat.values()) {
    const g = geos.length === 1 ? geos[0] : mergeGeometries(geos, false);
    if (g) {
      out.add(new THREE.Mesh(g, mat));
    } else {
      for (const one of geos) out.add(new THREE.Mesh(one, mat));
    }
  }
  return out;
}

async function loadModels(url, sc) {
  const buf = await (await fetch(url)).arrayBuffer();
  const gltf = await new GLTFLoader().parseAsync(buf, '');
  gltf.scene.updateMatrixWorld(true);
  const want = new Set(sc.glb.matched.map((r) => r.split('~')[0]));
  const picked = [];
  gltf.scene.traverse((o) => {
    if (!want.has(o.name)) return;
    for (let a = o.parent; a; a = a.parent) if (want.has(a.name)) return;   // top node only
    picked.push(o);
  });
  const index = {};
  S.parts.forEach((p, i) => { index[p.P.ref] = i; });
  // a duplicate reference: kicad-cli names every node by the BARE reference,
  // the parser keys the later blocks REF~2...; the k-th node goes to the
  // k-th block when the counts agree
  const byName = {};
  for (const o of picked) (byName[o.name] = byName[o.name] || []).push(o);
  const mm = new THREE.Matrix4().makeScale(1000, 1000, 1000);        // glTF metres -> mm
  const flip = new THREE.Matrix4().makeRotationX(Math.PI);
  const found = new Set(sc.glb.found || []), misfit = [];
  let n = 0;
  for (const name of Object.keys(byName)) {
    const nodes = byName[name];
    const keys = [name];
    for (let k = 2; index[name + '~' + k] !== undefined; k++) keys.push(name + '~' + k);
    nodes.forEach((o, j) => {
      const key = nodes.length === keys.length ? keys[j] : name;
      const i = index[key], pose = sc.export_poses[key];
      if (i === undefined || !pose) return;
      const part = S.parts[i];
      const world = new THREE.Matrix4().multiplyMatrices(mm, o.matrixWorld);
      const F = new THREE.Matrix4().makeRotationY(rad(pose[2]));
      if (pose[3] === 'B') F.multiply(flip);
      F.setPosition(pose[0], world.elements[13], pose[1]);   // on the face the export put it on
      const rest = F.clone().invert().multiply(world);
      const clone = mergedNode(o);
      clone.matrixAutoUpdate = false;
      clone.matrix.multiplyMatrices(pose[3] === 'B' ? flip : new THREE.Matrix4(), rest);
      // a ghost's own stencil value (1-254): see the depth pass below
      // a fixed part and a cover (#1443) are ghosted: see-through, so the
      // parts that move, and the parts under a cover, stay in view
      const ghost = part.P.fixed || part.P.cover;
      const gid = ghost ? (S.ghostId = (S.ghostId || 0) % 254 + 1) : 0;
      let k = 0;
      clone.traverse((c) => {
        if (!c.isMesh) return;
        const mats = (Array.isArray(c.material) ? c.material : [c.material]).map((m) => {
          const q = m.clone();
          // kicad-cli writes no metallicFactor, so glTF's default of 1 makes
          // every model a mirror -- near black with no environment to reflect
          if ('metalness' in q) { q.metalness = 0.15; q.roughness = 0.6; }
          if (!part.P.fixed && q.color) q.color.lerp(part.col, sc.tint ?? 0.5);
          if (ghost) {
            q.transparent = true; q.opacity = 0.32; q.depthWrite = false;
            // its front faces pass the depth pass's test within a hair, and
            // the first to land on a pixel marks it, so no other face of
            // this ghost blends there again
            Object.assign(q, { polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -4,
                               stencilWrite: true, stencilRef: gid, stencilFunc: THREE.NotEqualStencilFunc,
                               stencilZPass: THREE.ReplaceStencilOp });
          }
          return q;
        });
        c.material = Array.isArray(c.material) ? mats : mats[0];
        c.castShadow = true;
        c.receiveShadow = !ghost;
        // a fixed order, so the same face lands first every frame
        if (ghost) sided(c, 3 + 1e-6 * k++, part.up);
      });
      part.grp.add(clone);
      // a model Claude found on the web (find_models.py) must sit on its
      // footprint: its centre over the courtyard and its size near it, else
      // the part keeps its box
      if (found.has(part.P.ref) && !sits(clone, part)) {
        part.grp.remove(clone);
        misfit.push(part.P.ref);
        return;
      }
      if (ghost) {
        // a ghost is dozens of overlapping see-through meshes, which three
        // sorts by their origins every frame: as the camera orbits, two at
        // nearly one depth swap places and the blend flashes (a USB-C ghost's
        // pin rows lit and dimmed frame to frame).
        // Its depth drawn first, unseen, lets only its front surface blend:
        // one shell, the same every frame. A model's near-coincident faces
        // (pins against the housing, plating over the body) sit within
        // rounding of that depth, so without the tolerance, the stencil and
        // the fixed order above, which of them passed -- and how many times a
        // pixel blended -- changed with the view: a fine-pitch connector's
        // shell shimmered in stair steps as the board turned.
        const front = clone.clone(true);
        front.traverse((c) => {
          if (!c.isMesh) return;
          c.material = GHOST_DEPTH;
          c.castShadow = false;
          c.receiveShadow = false;
          sided(c, 2.5, part.up);
        });
        part.grp.add(front);
      }
      if (part.boxed) part.grp.remove(part.boxed);
      n += 1;
    });
  }
  return { n, misfit };
}

function sits(model, part) {
  part.grp.updateMatrixWorld(true);       // the group stands at the origin until the first frame
  const b = new THREE.Box3().setFromObject(model);
  if (b.isEmpty()) return false;
  const [x0, y0, x1, y1] = part.P.court;
  const cx = (b.min.x + b.max.x) / 2, cz = (b.min.z + b.max.z) / 2;
  const span = Math.max(b.max.x - b.min.x, b.max.z - b.min.z), court = Math.max(x1 - x0, y1 - y0);
  return cx > x0 - 1 && cx < x1 + 1 && cz > y0 - 1 && cz < y1 + 1 && span < 4 * court + 2;
}

// ---------------------------------------------------------------- copper
// The flow movie's copper (route3d.py): every track, via and pour with the
// tick it was born at and the tick it died at (-1: never). A frame shows an
// item from its birth to its death: lit (white, or cyan when restored) on
// the tick it is born, red on the tick it dies, gone after. Tracks are one
// flat quad each (square-capped), per layer, the tick test in the shader
// (stage3d's copper lifetimes, #1081).
const COPPER_VS = `
attribute float born; attribute float died; attribute float restored;
uniform float uT; varying float vKeep; varying float vState;
void main() {
  vKeep = (born <= uT && (died < 0.0 || died >= uT)) ? 1.0 : 0.0;
  vState = (died == uT) ? 2.0 : ((born == uT) ? (restored > 0.5 ? 3.0 : 1.0) : 0.0);
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;
const COPPER_FS = `
uniform vec3 uColor; uniform vec3 uNew; uniform vec3 uRip; uniform vec3 uRestore; uniform float uAlpha;
varying float vKeep; varying float vState;
void main() {
  if (vKeep < 0.5) discard;
  vec3 c = vState > 2.5 ? uRestore : (vState > 1.5 ? uRip : (vState > 0.5 ? uNew : uColor));
  gl_FragColor = vec4(c, uAlpha);
}`;
// a ShaderMaterial writes its colour as given: sRGB bytes go straight out
const srgb = (c) => new THREE.Vector3(c[0] / 255, c[1] / 255, c[2] / 255);

function layerY(li, n, d) {
  if (li === 0) return d + 0.02;                  // F.Cu, on the top face
  if (li === n - 1) return -0.02;                 // B.Cu, on the bottom face
  return d * (1 - li / (n - 1));                  // an inner layer, inside
}

// one flat quad per track (square-capped), both faces, with each item's
// birth, death and restored flag per vertex
function trackGeometry(items, y) {
  const pos = [], born = [], died = [], rest = [], tri = [];
  let v = 0;
  for (const [sx, sy, ex, ey, w, , b, dd, r] of items) {
    let dx = ex - sx, dy = ey - sy;
    const L = Math.hypot(dx, dy) || 1e-6;
    dx /= L; dy /= L;
    const hw = Math.max(w, 0.05) / 2, nx = -dy * hw, ny = dx * hw;
    const ax = sx - dx * hw, ay = sy - dy * hw, bx = ex + dx * hw, by = ey + dy * hw;
    pos.push(ax + nx, y, ay + ny, ax - nx, y, ay - ny, bx - nx, y, by - ny, bx + nx, y, by + ny);
    tri.push(v, v + 2, v + 1, v, v + 3, v + 2, v, v + 1, v + 2, v, v + 2, v + 3);
    for (let k = 0; k < 4; k++) { born.push(b); died.push(dd); rest.push(r); }
    v += 4;
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('born', new THREE.Float32BufferAttribute(born, 1));
  g.setAttribute('died', new THREE.Float32BufferAttribute(died, 1));
  g.setAttribute('restored', new THREE.Float32BufferAttribute(rest, 1));
  g.setIndex(tri);
  return g;
}

function pourMesh(z, C, d, n) {
  const shape = new THREE.Shape(z.poly.map((q) => new THREE.Vector2(q[0], q[1])));
  for (const ring of z.holes || []) shape.holes.push(new THREE.Path(ring.map((q) => new THREE.Vector2(q[0], q[1]))));
  const g = new THREE.ShapeGeometry(shape);
  g.rotateX(Math.PI / 2);
  g.translate(0, z.layer === 0 ? d + 0.01 : (z.layer === n - 1 ? -0.01 : layerY(z.layer, n, d)), 0);
  const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
    color: rgb(C.board).lerp(rgb((C.pour_layers || C.layers)[z.layer] || C.pad), 0.5), transparent: true, opacity: 0.5,
    side: THREE.DoubleSide, depthWrite: false }));
  sided(m, 1, z.layer === 0 ? 1 : (z.layer === n - 1 ? -1 : 0));
  return m;
}

function buildCopper(sc, root) {
  const cu = sc.copper, C = sc.colors, n = cu.layers.length, d = sc.thickness;
  S.copper = [];
  cu.layers.forEach((_name, li) => {
    const items = cu.segs.filter((q) => q[5] === li);
    if (!items.length) return;
    const g = trackGeometry(items, layerY(li, n, d));
    const m = new THREE.Mesh(g, new THREE.ShaderMaterial({
      vertexShader: COPPER_VS, fragmentShader: COPPER_FS,
      uniforms: { uT: { value: -1 }, uColor: { value: srgb(C.layers[li] || C.pad) }, uNew: { value: srgb(C.new) },
                  uRip: { value: srgb(C.rip) }, uRestore: { value: srgb(C.restored) }, uAlpha: { value: 1.0 } } }));
    m.renderOrder = 2;
    m.frustumCulled = false;
    root.add(m);
    S.copper.push(m);
  });
  // vias: one instanced barrel each, scaled away when not alive
  const cyl = new THREE.CylinderGeometry(0.5, 0.5, d + 0.08, 16);
  cyl.translate(0, d / 2, 0);
  S.vias = new THREE.InstancedMesh(cyl, new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.4, metalness: 0.2 }),
                                   Math.max(1, cu.vias.length));
  S.vias.count = cu.vias.length;
  S.vias.frustumCulled = false;
  root.add(S.vias);
  // pours: a flat tinted shape each, on its layer
  S.pours = [];
  for (const z of cu.pours) {
    const m = pourMesh(z, C, d, n);
    m.userData.life = [z.born, z.died];
    root.add(m);
    S.pours.push(m);
  }
}

function setCopper(fr) {
  if (!S.copper) return;
  const T = fr.tick ?? -1;
  for (const m of S.copper) m.material.uniforms.uT.value = T;
  const C = S.sc.colors, M = new THREE.Matrix4(), Z = new THREE.Matrix4().makeScale(0, 0, 0);
  const col = { via: rgb(C.via), new: rgb(C.new), rip: rgb(C.rip), restored: rgb(C.restored) };
  S.sc.copper.vias.forEach((q, i) => {
    const [x, y, size, , , , b, dd, r] = q;
    if (b <= T && (dd < 0 || dd >= T)) {
      const k = Math.max(size, 0.1);
      M.makeScale(k, 1, k).setPosition(x, 0, y);
      S.vias.setMatrixAt(i, M);
      S.vias.setColorAt(i, dd === T ? col.rip : (b === T ? (r ? col.restored : col.new) : col.via));
    } else {
      S.vias.setMatrixAt(i, Z);
    }
  });
  S.vias.instanceMatrix.needsUpdate = true;
  if (S.vias.instanceColor) S.vias.instanceColor.needsUpdate = true;
  for (const m of S.pours) {
    const [b, dd] = m.userData.life;
    m.visible = b <= T && (dd < 0 || dd > T);
  }
}

// ---------------------------------------------------------------- the seed grid
// A flow movie's zoom out (route3d.py): every judged seed's board side by
// side, at that seed's placement and with its fast route's copper, each
// labelled with its score, the best one framed. The live board stands in
// one cell (a frame's `ro`), whose copy is hidden; the cells lie along the
// camera's own screen axes (record.py lays them out).
function staticCopper(cu, C, d, n) {
  const g = new THREE.Group();
  for (let li = 0; li < n; li++) {
    const items = cu.segs.filter((q) => q[5] === li);
    if (!items.length) continue;
    g.add(new THREE.Mesh(trackGeometry(items, layerY(li, n, d)),
                         new THREE.MeshBasicMaterial({ color: rgb(C.layers[li] || C.pad), side: THREE.DoubleSide })));
  }
  if (cu.vias.length) {
    const cyl = new THREE.CylinderGeometry(0.5, 0.5, d + 0.08, 16);
    cyl.translate(0, d / 2, 0);
    const vm = new THREE.InstancedMesh(cyl, new THREE.MeshStandardMaterial({ color: rgb(C.via), roughness: 0.4,
                                                                          metalness: 0.2 }), cu.vias.length);
    const M = new THREE.Matrix4();
    cu.vias.forEach(([x, y, size], i) => {
      const k = Math.max(size, 0.1);
      vm.setMatrixAt(i, M.makeScale(k, 1, k).setPosition(x, 0, y));
    });
    g.add(vm);
  }
  for (const z of cu.pours) g.add(pourMesh(z, C, d, n));
  return g;
}

function makeLabel(text, color) {
  const cv = document.createElement('canvas');
  cv.width = 1024; cv.height = 128;
  const x = cv.getContext('2d');
  x.fillStyle = 'rgba(0, 0, 0, 0.72)';
  x.beginPath(); x.roundRect(4, 8, 1016, 112, 24); x.fill();
  let px = 64;
  const font = () => `600 ${px}px -apple-system, "Helvetica Neue", Helvetica, Arial, sans-serif`;
  x.font = font();
  while (x.measureText(text).width > 960 && px > 20) { px -= 2; x.font = font(); }
  x.fillStyle = css(color);
  x.textAlign = 'center'; x.textBaseline = 'middle';
  x.fillText(text, 512, 66);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  sp.renderOrder = 10;
  return sp;
}

function buildGrid(sc, world) {
  const G = sc.grid, C = sc.colors, d = sc.thickness, n = sc.copper.layers.length;
  S.cells = [];
  G.cells.forEach((cell) => {
    const g = new THREE.Group();
    g.position.set(cell.off[0], 0, cell.off[1]);
    g.add(S.boardMesh.clone());
    S.parts.forEach((part, i) => {
      const q = cell.poses[i];
      if (!q) return;
      const c = part.grp.clone(true);
      c.position.set(q[0], q[3], q[1]);
      c.rotation.set(0, rad(q[2]), 0);
      g.add(c);
    });
    g.add(staticCopper(cell.copper, C, d, n));
    g.visible = false;
    world.add(g);
    const label = makeLabel(cell.label, cell.best ? C.best : C.text);
    label.position.set(cell.label_at[0], d + 1.0, cell.label_at[1]);
    label.scale.set(cell.label_w, cell.label_w / 8, 1);
    label.visible = false;
    world.add(label);
    S.cells.push({ g, label });
  });
  // the best seed's frame: a flat band round its board
  const b = G.cells[G.best], [x0, y0, x1, y1] = sc.bounds, m = 1.2, w = 0.8;
  const outer = new THREE.Shape([[x0 - m - w, y0 - m - w], [x1 + m + w, y0 - m - w], [x1 + m + w, y1 + m + w],
                                 [x0 - m - w, y1 + m + w]].map((q) => new THREE.Vector2(q[0], q[1])));
  outer.holes.push(new THREE.Path([[x0 - m, y0 - m], [x1 + m, y0 - m], [x1 + m, y1 + m], [x0 - m, y1 + m]]
    .map((q) => new THREE.Vector2(q[0], q[1]))));
  const fg = new THREE.ShapeGeometry(outer);
  fg.rotateX(Math.PI / 2);
  S.bestFrame = new THREE.Mesh(fg, new THREE.MeshBasicMaterial({ color: rgb(C.best), side: THREE.DoubleSide }));
  S.bestFrame.position.set(b.off[0], d + 0.08, b.off[1]);
  S.bestFrame.visible = false;
  world.add(S.bestFrame);
}

function setGrid(fr) {
  const G = S.sc.grid;
  const off = (G && fr.ro != null) ? G.cells[fr.ro].off : [0, 0];
  S.root.position.set(off[0], 0, off[1]);
  if (!S.cells) return;
  S.cells.forEach((c, k) => {
    c.g.visible = !!fr.grid && k !== fr.ro;
    c.label.visible = !!fr.labels;
  });
  S.bestFrame.visible = !!fr.best;
}

// ---------------------------------------------------------------- airwires
function buildAir(sc, root) {
  let n = 0;
  for (const net of sc.nets) n += net.length - 1;
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(Math.max(1, n) * 6), 3));
  const m = new THREE.LineBasicMaterial({ color: rgb(sc.colors.air), transparent: true, opacity: 0.75 });
  S.air = new THREE.LineSegments(g, m);
  S.air.frustumCulled = false;
  S.air.renderOrder = 2;
  root.add(S.air);
}

function padWorld(i, k, fr) {
  const part = S.parts[i], P = part.P, p = P.pads[k];
  const [x, y, rot, h] = fr.p[i];
  const t = rad(rot), c = Math.cos(t), s = Math.sin(t);
  // a plated hole's wire leaves from the board's top face, where it is seen
  const top = p[6] === 'T' && part.up < 0 ? S.sc.thickness : 0;
  return [x + p[0] * c + p[1] * s, h + top + (part.up > 0 || top ? 0.07 : -0.07), y - p[0] * s + p[1] * c];
}

function setAir(fr) {
  S.air.visible = fr.air !== false;
  if (!S.air.visible) return;
  const a = S.air.geometry.getAttribute('position');
  let v = 0;
  for (const net of S.sc.nets) {
    const pts = net.map(([i, k]) => padWorld(i, k, fr));
    // Prim's tree, Manhattan in the board plane (place_movie's airwires)
    const best = new Map();
    for (let j = 1; j < pts.length; j++) best.set(j, [Infinity, 0]);
    let at = 0;
    while (best.size) {
      const [ax, , az] = pts[at];
      let pick = -1, pd = Infinity;
      for (const [j, b] of best) {
        const dd = Math.abs(pts[j][0] - ax) + Math.abs(pts[j][2] - az);
        if (dd < b[0]) { b[0] = dd; b[1] = at; }
        if (b[0] < pd) { pd = b[0]; pick = j; }
      }
      const from = pts[best.get(pick)[1]], to = pts[pick];
      best.delete(pick);
      a.array.set(from, v); a.array.set(to, v + 3); v += 6;
      at = pick;
    }
  }
  a.needsUpdate = true;
  S.air.geometry.setDrawRange(0, v / 3);
}

// ---------------------------------------------------------------- camera
const FOV = 30;

// the camera that fits `corners` from direction `dir`, from a first guess:
// centre what is shown, then scale it to fill the frame with a margin
function fitTo(corners, target, dist, dir) {
  const cam = S.cam, tanh = Math.tan(rad(FOV / 2));
  for (let k = 0; k < 8; k++) {
    cam.position.copy(target).addScaledVector(dir, dist);
    cam.lookAt(target);
    cam.updateMatrixWorld(true);
    let mnx = Infinity, mxx = -Infinity, mny = Infinity, mxy = -Infinity;
    for (const c of corners) {
      const p = c.clone().project(cam);
      mnx = Math.min(mnx, p.x); mxx = Math.max(mxx, p.x); mny = Math.min(mny, p.y); mxy = Math.max(mxy, p.y);
    }
    const right = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 0);
    const upv = new THREE.Vector3().setFromMatrixColumn(cam.matrixWorld, 1);
    const hh = dist * tanh, hw = hh * cam.aspect;
    target.addScaledVector(right, (mnx + mxx) / 2 * hw).addScaledVector(upv, (mny + mxy) / 2 * hh);
    dist *= Math.max((mxx - mnx) / 2 / 0.92, (mxy - mny) / 2 / 0.86);
  }
  return { target, dist };
}

function fitCamera(fr) {
  const [x0, y0, x1, y1, lo, hi] = fr.view;
  const cam = S.cam, d = S.sc.thickness;
  // a grid frame names its four ground corners (a rectangle along the
  // screen's axes); any other frame its box
  const dir = new THREE.Vector3(Math.sin(rad(fr.az)) * Math.cos(rad(fr.el)), Math.sin(rad(fr.el)),
                                Math.cos(rad(fr.az)) * Math.cos(rad(fr.el)));
  const corners = [];
  const ground = fr.pts || [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
  for (const [x, z] of ground) for (const y of [lo, hi]) corners.push(new THREE.Vector3(x, y, z));
  let { target, dist } = fitTo(corners, new THREE.Vector3((x0 + x1) / 2, d / 2, (y0 + y1) / 2),
                               2 * Math.hypot(x1 - x0, y1 - y0), dir);
  let r = 0.6 * Math.hypot(x1 - x0, y1 - y0) + 4;
  if (S.flip) {
    // turning over: the board as it is posed, face up or face down; mid
    // turn, a view that holds the whole turn (the camera steps back to turn
    // it); from the frame's own view as the board rises
    const F = S.flip, b = fitTo(F.pts, F.centre.clone(), dist, dir);
    const n = fitTo(F.now, F.nowCentre.clone(), dist, dir);
    b.target.lerp(n.target, F.w);
    b.dist *= Math.pow(n.dist / b.dist, F.w);
    target.lerp(b.target, F.rise);
    dist *= Math.pow(b.dist / dist, F.rise);
    r = Math.max(r, F.reach + F.lift + 4);
  }
  cam.position.copy(target).addScaledVector(dir, dist);
  cam.lookAt(target);
  cam.updateMatrixWorld(true);
  // the sun follows the view so the shadows fall the same way all film
  S.sun.position.copy(target).add(new THREE.Vector3(-0.45, 1.0, 0.35).multiplyScalar(4 * dist));
  S.sun.target.position.copy(target);
  S.sun.target.updateMatrixWorld();
  const sh = S.sun.shadow.camera;
  sh.left = -r; sh.right = r; sh.top = r; sh.bottom = -r; sh.near = 0.1; sh.far = 12 * dist;
  sh.updateProjectionMatrix();
}

// ---------------------------------------------------------------- turning over
// A frame's `flip` [deg, rise, az]: the board, and everything on it, turned
// `deg` about the screen's horizontal at azimuth `az`, through the board's
// centre, after rising `rise` (0-1) of the way it must to turn clear of the
// table. The turn's view holds the whole turn, so the camera does not breathe.
function setFlip(fr) {
  S.root.quaternion.identity();
  S.flip = null;
  if (!fr.flip) return;
  const [deg, rise, az] = fr.flip;
  const sc = S.sc, d = sc.thickness, [bx0, by0, bx1, by1] = sc.bounds;
  const off = S.root.position.clone();               // the grid cell it stands in
  const c = new THREE.Vector3((bx0 + bx1) / 2, d / 2, (by0 + by1) / 2);
  const axis = new THREE.Vector3(Math.cos(rad(az)), 0, -Math.sin(rad(az)));
  // what turns: the board and the parts standing on it, as this frame poses them
  S.root.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(S.boardMesh);
  S.parts.forEach((part, i) => {
    const h = fr.p[i][3];
    if (Math.abs(h - (part.up > 0 ? d : 0)) < 1e-3) box.expandByObject(part.grp);
  });
  box.translate(off.clone().negate());
  const corners = [];
  for (const x of [box.min.x, box.max.x]) for (const y of [box.min.y, box.max.y])
    for (const z of [box.min.z, box.max.z]) corners.push(new THREE.Vector3(x, y, z).sub(c));
  // a corner turns in the vertical plane across the axis, at its distance from it
  let reach = 0;
  for (const q of corners) reach = Math.max(reach, q.clone().addScaledVector(axis, -q.dot(axis)).length());
  const lift = Math.max(0, reach - d / 2 + sc.table + 0.5);
  const q = new THREE.Quaternion().setFromAxisAngle(axis, rad(deg));
  S.root.quaternion.copy(q);
  const up = new THREE.Vector3(0, lift, 0);
  S.root.position.copy(off).add(c).sub(c.clone().applyQuaternion(q)).addScaledVector(up, rise);
  // the turn's view: every corner at every 15 degrees, fully risen; and the
  // corners as this frame poses them
  const pts = [], at = off.clone().add(c).add(up), t = new THREE.Quaternion();
  for (let a = 0; a < 360; a += 15) {
    t.setFromAxisAngle(axis, rad(a));
    for (const k of corners) pts.push(k.clone().applyQuaternion(t).add(at));
  }
  const nowCentre = off.clone().add(c).addScaledVector(up, rise);
  const now = corners.map((k) => k.clone().applyQuaternion(q).add(nowCentre));
  // the posed view face up or face down, the turn's view on edge
  const w = Math.cos(rad(deg)) ** 2;
  S.flip = { pts, centre: at, now, nowCentre, w, rise, lift, reach };
}

function orderBySide() {
  // which face the camera sees: the board's top, as this frame turns it
  const n = new THREE.Vector3(0, 1, 0).applyQuaternion(S.root.quaternion);
  const [bx0, by0, bx1, by1] = S.sc.bounds;
  const c = S.root.localToWorld(new THREE.Vector3((bx0 + bx1) / 2, S.sc.thickness / 2, (by0 + by1) / 2));
  const top = n.dot(S.cam.position.clone().sub(c)) > 0;
  for (const [o, order, side] of S.sided) o.renderOrder = order + (side !== 0 && (side > 0) === top ? NEAR : 0);
}

// ---------------------------------------------------------------- key
function buildKey(sc) {
  const key = document.getElementById('key');
  for (const k of sc.key) {
    const e = document.createElement('span');
    const sw = document.createElement('span');
    sw.className = 'sw' + (k.mark === 'ring' ? ' ring' : '');
    sw.style.background = css(k.rgb);
    sw.style.borderColor = css(k.rgb);
    e.appendChild(sw);
    e.appendChild(document.createTextNode(k.text));
    key.appendChild(e);
  }
  return key.offsetHeight;
}

// ---------------------------------------------------------------- entry
window.m3dInit = async function (o) {
  const sc = await (await fetch(o.sceneUrl)).json();
  const tl = await (await fetch(o.timelineUrl)).json();
  S.sc = sc; S.tl = tl;
  S.sided = [];
  document.body.style.background = css(sc.colors.ground);
  const W = o.width, H = o.height;
  const kh = buildKey(sc);
  // a stencil buffer: each fixed part's ghost blends a pixel once (loadModels)
  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true, stencil: true });
  renderer.setPixelRatio(1);
  renderer.setSize(W, H - kh);
  renderer.setClearColor(rgb(sc.colors.ground), 1);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  document.getElementById('wrap').prepend(renderer.domElement);
  const world = new THREE.Scene();
  world.add(new THREE.HemisphereLight(0xffffff, 0x30343c, 1.3));
  const sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.02;
  world.add(sun, sun.target);
  S.sun = sun;
  world.add(buildTable(sc));
  const root = new THREE.Group();
  world.add(root);
  root.add(buildBoard(sc));
  buildParts(sc, root);
  buildAir(sc, root);
  if (sc.copper) buildCopper(sc, root);
  let models = 0, misfit = [], modelError = null;
  if (o.glbUrl && sc.glb) {
    // the models are decoration: a missing or corrupt GLB leaves the boxes
    try { ({ n: models, misfit } = await loadModels(o.glbUrl, sc)); } catch (e) { modelError = String(e).slice(0, 200); }
  }
  S.root = root;
  if (sc.grid) buildGrid(sc, world);      // after the models: each cell clones the parts
  S.cam = new THREE.PerspectiveCamera(FOV, W / (H - kh), 0.5, 100000);
  Object.assign(S, { renderer, world, root });
  const gl = renderer.getContext();
  const ext = gl.getExtension('WEBGL_debug_renderer_info');
  return { renderer: ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER),
           three: THREE.REVISION, frames: tl.frames.length, models, misfit, modelError };
};

window.m3dFrame = function (i) {
  const fr = S.tl.frames[i];
  poseParts(fr);
  setAir(fr);
  setCopper(fr);
  setGrid(fr);
  setFlip(fr);
  S.boardMat.opacity = fr.alpha ?? S.sc.board_alpha ?? 1.0;
  fitCamera(fr);
  orderBySide();
  document.getElementById('cap').textContent = fr.label;
  S.renderer.render(S.world, S.cam);
  return true;
};

window.m3dReady = true;
