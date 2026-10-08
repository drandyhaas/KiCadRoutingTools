// The stage3d film's 3D board (#1081). A PURE FUNCTION of (scene, timeline,
// state index): `renderState(i)` sets every transform, visibility and colour
// from the timeline and renders once. Nothing reads a clock, so two renders
// of one state are the same pixels (pinned three.js, SwiftShader).
//
// Frames: KiCad's board (x, y-down) maps to three (x, z) with y UP out of the
// board's top face, so a KiCad rotation `rot` is `rotation.y = rad(rot)` --
// what kicad-cli's own GLB carries (F.Cu parts `Ry(rot)`, B.Cu `Ry(rot)
// Rx(180)`). The board turns over about its SCREEN-vertical axis (three z),
// the same flip the X-ray animates, so the far side comes up mirrored
// left-right exactly as the 2D film shows it.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const S = {};

function rad(d) { return d * Math.PI / 180; }

// Theme colours are sRGB bytes; say so, or three's colour management treats
// them as linear and the board box comes out lighter than the frame's ground.
function rgb(c) { return new THREE.Color().setRGB(c[0] / 255, c[1] / 255, c[2] / 255, THREE.SRGBColorSpace); }

// ---------------------------------------------------------------- board
function buildBoard(scene, colors) {
  const d = scene.thickness;
  const shape = new THREE.Shape(scene.outline.map(p => new THREE.Vector2(p[0], p[1])));
  for (const ring of scene.cutouts) {
    shape.holes.push(new THREE.Path(ring.map(p => new THREE.Vector2(p[0], p[1]))));
  }
  const geo = new THREE.ExtrudeGeometry(shape, { depth: d, bevelEnabled: false, curveSegments: 24 });
  geo.rotateX(Math.PI / 2);          // shape (x, y) -> three (x, z); depth -> -y
  geo.translate(0, d, 0);            // y in [0, d]: the top face is y = d
  const mat = new THREE.MeshStandardMaterial({ color: rgb(colors.board), roughness: 0.85,
                                               metalness: 0.0, transparent: true, opacity: 0.9 });
  const board = new THREE.Mesh(geo, mat);
  // the outline on both faces, in the theme's board edge: a WHITE board's
  // top face is close to the light ground, and without it the back and side
  // edges vanished into the frame
  if (colors.edge) {
    const lm = new THREE.LineBasicMaterial({ color: rgb(colors.edge) });
    for (const ring of [scene.outline, ...scene.cutouts]) {
      for (const y of [d + 0.02, -0.02]) {
        const pts = ring.map(p => new THREE.Vector3(p[0], y, p[1]));
        board.add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(pts), lm));
      }
    }
  }
  return board;
}

// ---------------------------------------------------------------- copper
const COPPER_VS = `
attribute float born; attribute float died; attribute float hidden;
uniform float uN; varying float vKeep;
void main() {
  vKeep = (born < uN && (died < 0.0 || died >= uN) && hidden < 0.5) ? 1.0 : 0.0;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;
const COPPER_FS = `
uniform vec3 uColor; uniform float uAlpha; varying float vKeep;
void main() { if (vKeep < 0.5) discard; gl_FragColor = vec4(uColor, uAlpha); }`;

function layerY(li, n, d) {
  if (li === 0) return d + 0.035;                 // F.Cu, on the top face
  if (li === n - 1) return -0.035;                // B.Cu, on the bottom face
  return d * (1 - li / (n - 1));                  // an inner layer, inside
}

// One flat quad per segment, lengthened by half its width at each end (a
// square-capped trace), with its item index, birth and death per vertex.
function segQuads(items, y, withLife) {
  const pos = [], born = [], died = [], idx = [], hid = [];
  const tri = [];
  let v = 0;
  for (const it of items) {
    const [sx, sy, ex, ey, w] = it.row;
    let dx = ex - sx, dy = ey - sy;
    const L = Math.hypot(dx, dy) || 1e-6;
    dx /= L; dy /= L;
    const hw = Math.max(w, 0.05) / 2;
    const nx = -dy * hw, ny = dx * hw;
    const ax = sx - dx * hw, ay = sy - dy * hw, bx = ex + dx * hw, by = ey + dy * hw;
    pos.push(ax + nx, y, ay + ny, ax - nx, y, ay - ny, bx - nx, y, by - ny, bx + nx, y, by + ny);
    tri.push(v, v + 2, v + 1, v, v + 3, v + 2, v, v + 1, v + 2, v, v + 2, v + 3);  // both faces
    for (let k = 0; k < 4; k++) {
      if (withLife) { born.push(it.born); died.push(it.died); hid.push(0); idx.push(it.i); }
    }
    v += 4;
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  if (withLife) {
    g.setAttribute('born', new THREE.Float32BufferAttribute(born, 1));
    g.setAttribute('died', new THREE.Float32BufferAttribute(died, 1));
    g.setAttribute('hidden', new THREE.Float32BufferAttribute(hid, 1));
  }
  g.setIndex(tri);
  g.userData.items = idx;
  return g;
}

function buildCopper(tl, scene, colors, root) {
  const n = tl.layers.length, d = scene.thickness;
  S.copper = [];
  S.segLayerOf = new Int32Array(tl.segs.length);
  S.segSlot = new Int32Array(tl.segs.length);
  const per = tl.layers.map(() => []);
  tl.segs.forEach((s, i) => {
    const li = s[5];
    S.segLayerOf[i] = li;
    S.segSlot[i] = per[li].length;
    per[li].push({ row: s, born: s[6], died: s[7], i });
  });
  per.forEach((items, li) => {
    if (!items.length) { S.copper.push(null); return; }
    const g = segQuads(items, layerY(li, n, d), true);
    const inner = li !== 0 && li !== n - 1;
    const m = new THREE.ShaderMaterial({
      vertexShader: COPPER_VS, fragmentShader: COPPER_FS, transparent: inner,
      uniforms: { uN: { value: 0 }, uColor: { value: rgb(colors.layers[li] || colors.pad) },
                  uAlpha: { value: inner ? 0.55 : 1.0 } } });
    const mesh = new THREE.Mesh(g, m);
    mesh.renderOrder = inner ? 1 : 2;
    root.add(mesh);
    S.copper.push(mesh);
  });
  // vias: one instanced barrel each, shown or scaled away per frame
  const cyl = new THREE.CylinderGeometry(0.5, 0.5, d + 0.09, 16);
  cyl.translate(0, d / 2, 0);
  const vm = new THREE.MeshStandardMaterial({ color: rgb(colors.via), roughness: 0.4, metalness: 0.6 });
  S.vias = new THREE.InstancedMesh(cyl, vm, Math.max(1, tl.vias.length));
  S.vias.count = tl.vias.length;
  S.viaRows = tl.vias;
  S.lastNv = -1;
  root.add(S.vias);
  S.hidden = new Set();
}

function setCopper(st) {
  for (const m of S.copper) if (m) m.material.uniforms.uN.value = st.ns;
  // a growth stage hides its finished self under it (`hide` = item indices)
  const want = new Set(st.hide);
  const touched = new Set();
  for (const i of S.hidden) if (!want.has(i)) touched.add(i);
  for (const i of want) if (!S.hidden.has(i)) touched.add(i);
  for (const i of touched) {
    const m = S.copper[S.segLayerOf[i]];
    if (!m) continue;
    const a = m.geometry.getAttribute('hidden');
    const base = S.segSlot[i] * 4;
    const v = want.has(i) ? 1 : 0;
    for (let k = 0; k < 4; k++) a.array[base + k] = v;
    a.needsUpdate = true;
  }
  S.hidden = want;
  if (st.nv !== S.lastNv) {
    const M = new THREE.Matrix4(), Z = new THREE.Matrix4().makeScale(0, 0, 0);
    S.viaRows.forEach((v, i) => {
      const on = v[6] < st.nv && (v[7] < 0 || v[7] >= st.nv);
      if (on) {
        const r = Math.max(v[2], 0.1);
        M.makeScale(r, 1, r).setPosition(v[0], 0, v[1]);
        S.vias.setMatrixAt(i, M);
      } else S.vias.setMatrixAt(i, Z);
    });
    S.vias.instanceMatrix.needsUpdate = true;
    S.lastNv = st.nv;
  }
}

// #1090: the plane pours -- one flat shape per zone on its layer, shown
// from the frame the film reveals that net's fill.
function buildPours(scene, tl, colors, root) {
  S.pours = [];
  const n = tl.layers.length, d = scene.thickness;
  for (const z of (scene.pours || [])) {
    const shape = new THREE.Shape(z.poly.map(p => new THREE.Vector2(p[0], p[1])));
    const g = new THREE.ShapeGeometry(shape);
    g.rotateX(Math.PI / 2);                     // shape (x, y) -> three (x, z)
    const li = z.layer;
    g.translate(0, li === 0 ? d + 0.02 : (li === n - 1 ? -0.02 : d * (1 - li / (n - 1))), 0);
    const base = rgb(colors.layers[li] || colors.pad), body = rgb(colors.board);
    const col = body.clone().lerp(base, 0.45);
    const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
      color: col, transparent: true, opacity: 0.75, side: THREE.DoubleSide, depthWrite: false }));
    m.renderOrder = 1;
    m.visible = false;
    m.userData.net = z.net;
    // a zone OUTLINE may run far past the board (KiCad clips the fill), so
    // it never sets the camera's fit
    m.userData.noFit = true;
    root.add(m);
    S.pours.push(m);
  }
}

function setPours(st) {
  const on = new Set(st.zones || []);
  for (const m of S.pours) m.visible = on.has(m.userData.net);
}

function setHighlight(st, tl, scene, colors, root) {
  if (S.hl) {
    root.remove(S.hl);
    S.hl.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
    S.hl = null;
  }
  if (!st.hl_s.length && !st.hl_v.length) return;
  const n = tl.layers.length, d = scene.thickness;
  const col = st.color ? rgb(st.color) : rgb(colors.hilite);
  const g = new THREE.Group();
  const byLayer = {};
  for (const h of st.hl_s) (byLayer[h[5]] = byLayer[h[5]] || []).push({ row: h });
  for (const li of Object.keys(byLayer)) {
    const y = layerY(+li, n, d) + (+li === n - 1 ? -0.03 : 0.03);
    const geo = segQuads(byLayer[li], y, false);
    g.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: col, side: THREE.DoubleSide })));
  }
  for (const v of st.hl_v) {
    const c = new THREE.Mesh(new THREE.CylinderGeometry(v[2] / 2 + 0.05, v[2] / 2 + 0.05, d + 0.15, 16),
                             new THREE.MeshBasicMaterial({ color: col }));
    c.position.set(v[0], d / 2, v[1]);
    g.add(c);
  }
  g.traverse(o => { o.renderOrder = 3; });
  S.hl = g;
  root.add(g);
}

// ---------------------------------------------------------------- parts
function buildParts(scene, colors, root) {
  S.parts = {};
  const d = scene.thickness;
  const bodyMat = new THREE.MeshStandardMaterial({ color: rgb(colors.body), roughness: 0.7 });
  const padMat = new THREE.MeshStandardMaterial({ color: rgb(colors.pad), roughness: 0.35, metalness: 0.7,
                                                  side: THREE.DoubleSide });
  const holeMat = new THREE.MeshBasicMaterial({ color: rgb(colors.hole || [20, 20, 22]) });
  for (const ref of Object.keys(scene.parts).sort()) {
    const P = scene.parts[ref];
    const grp = new THREE.Group();
    const faces = { F: new THREE.Group(), B: new THREE.Group() };
    for (const p of P.pads) {
      const [lx, ly, sx, sy, ang, shape, face, drill, hole, polys] = p;
      for (const f of (face === 'T' ? ['F', 'B'] : [face])) {
        const y = f === 'F' ? 0.03 : -0.03;
        if (polys) {
          // a custom pad's REAL outline, in the part's frame (#1090)
          for (const poly of polys) {
            const g = new THREE.ShapeGeometry(new THREE.Shape(poly.map(q => new THREE.Vector2(q[0], q[1]))));
            g.rotateX(Math.PI / 2);
            const m = new THREE.Mesh(g, padMat);
            m.position.y = y + (f === 'F' ? 0.025 : -0.025);
            m.userData.custom = true;
            faces[f].add(m);
          }
        } else {
          const geo = (shape === 'circle')
            ? new THREE.CylinderGeometry(Math.max(sx, sy) / 2, Math.max(sx, sy) / 2, 0.05, 20)
            : new THREE.BoxGeometry(Math.max(sx, 0.05), 0.05, Math.max(sy, 0.05));
          const m = new THREE.Mesh(geo, padMat);
          m.position.set(lx, y, ly);
          m.rotation.y = -rad(ang);
          faces[f].add(m);
        }
        if (drill > 0) {
          // the drill, at its OWN centre (an offset drill is not the copper's)
          const hx = hole ? hole[0] : lx, hz = hole ? hole[1] : ly;
          const h = new THREE.Mesh(new THREE.CircleGeometry(drill / 2, 18), holeMat);
          h.rotation.x = f === 'F' ? -Math.PI / 2 : Math.PI / 2;
          h.position.set(hx, y + (f === 'F' ? 0.03 : -0.03), hz);
          h.userData.hole = true;
          faces[f].add(h);
        }
      }
    }
    let body = null;
    if (P.body) {
      const [x0, y0, x1, y1, h] = P.body;
      body = new THREE.Mesh(new THREE.BoxGeometry(Math.max(x1 - x0, 0.1), h, Math.max(y1 - y0, 0.1)), bodyMat);
      body.position.set((x0 + x1) / 2, h / 2 + 0.06, (y0 + y1) / 2);
      body.userData.h = h;
      grp.add(body);
    }
    grp.add(faces.F, faces.B);
    // the part's own extent, for its halo and its ghost
    let ex0 = -0.5, ey0 = -0.5, ex1 = 0.5, ey1 = 0.5;
    if (P.pads.length) {
      ex0 = Math.min(...P.pads.map(p => p[0] - p[2] / 2)); ex1 = Math.max(...P.pads.map(p => p[0] + p[2] / 2));
      ey0 = Math.min(...P.pads.map(p => p[1] - p[3] / 2)); ey1 = Math.max(...P.pads.map(p => p[1] + p[3] / 2));
    }
    const plane = new THREE.PlaneGeometry(ex1 - ex0 + 1.2, ey1 - ey0 + 1.2);
    plane.rotateX(-Math.PI / 2);
    plane.translate((ex0 + ex1) / 2, 0, (ey0 + ey1) / 2);
    const halo = new THREE.Mesh(plane, new THREE.MeshBasicMaterial({
      color: rgb(colors.hilite), transparent: true, opacity: 0.55, depthWrite: false,
      side: THREE.DoubleSide }));
    halo.renderOrder = 4;
    halo.visible = false;
    grp.add(halo);
    const ghost = new THREE.Mesh(plane, new THREE.MeshBasicMaterial({
      color: rgb(colors.hilite), transparent: true, opacity: 0.22, depthWrite: false,
      side: THREE.DoubleSide }));
    ghost.renderOrder = 4;
    ghost.visible = false;
    // the glide's decorations switch on in one frame; fitting them made the
    // camera pop at every glide's start and end
    halo.userData.noFit = ghost.userData.noFit = true;
    root.add(ghost);
    root.add(grp);
    S.parts[ref] = { grp, faces, body, side: P.side, d, halo, ghost };
  }
}

function poseParts(st, tl) {
  const ep = tl.epochs[Math.max(0, Math.min(st.epoch, tl.epochs.length - 1))] || {};
  for (const ref of Object.keys(S.parts)) {
    const part = S.parts[ref];
    const e = ep[ref];
    if (!e) { part.grp.visible = false; continue; }
    part.grp.visible = true;
    const m = st.moving[ref];
    const x = m ? m[0] : e[0], y = m ? m[1] : e[1], rot = m ? m[2] : e[2];
    const back = String(e[3] || 'F.Cu').startsWith('B');
    // mid-glide: lifted clear of its neighbours (C26 slid THROUGH the
    // RP2350's model and was hidden for 6 of its 10 frames), haloed, and
    // its landing pose shown as a ghost on the face
    const lift = m ? 1.2 : 0;
    part.grp.position.set(x, back ? -lift : part.d + lift, y);
    part.grp.rotation.set(0, rad(rot), 0);
    part.halo.visible = !!m;
    part.halo.position.y = back ? -0.03 + lift : 0.03 - lift;
    part.ghost.visible = !!m;
    if (m) {
      part.ghost.position.set(e[0], back ? -0.04 : part.d + 0.04, e[1]);
      part.ghost.rotation.set(0, rad(e[2]), 0);
    }
    // pads drawn on the part's own face (THT pads carry both), and the body
    // hangs below a back-side part
    part.faces.F.position.y = back ? part.d : 0;
    part.faces.B.position.y = back ? 0 : -part.d;
    if (part.body) {
      // a back-side part's body hangs BELOW the board's bottom face (the
      // group sits at y = 0 there); mirroring a box about its own centre
      // moved nothing, so it used to sit inside the board -- and a 3 mm
      // connector stuck out of the TOP (the phase-6 verifier, orangecrab J4)
      const h = part.body.userData.h;
      part.body.position.y = back ? -(h / 2 + 0.06) : h / 2 + 0.06;
      part.body.visible = !part.glb;
    }
    if (part.glb) {
      const F = new THREE.Matrix4().makeRotationY(rad(rot));
      if (back) F.multiply(new THREE.Matrix4().makeRotationX(Math.PI));
      F.setPosition(x, back ? 0 : part.d, y);
      for (const n of part.glb) {
        n.obj.matrix.multiplyMatrices(F, n.rest);
        n.obj.matrixWorldNeedsUpdate = true;
      }
    }
  }
}

async function loadGlb(url, glb, root) {
  const buf = await (await fetch(url)).arrayBuffer();
  const gltf = await new GLTFLoader().parseAsync(buf, '');
  gltf.scene.updateMatrixWorld(true);
  const want = new Set(glb.matched);
  const mm = new THREE.Matrix4().makeScale(1000, 1000, 1000);   // glTF metres -> mm
  const picked = [];
  gltf.scene.traverse(o => {
    if (!want.has(o.name)) return;
    for (let a = o.parent; a; a = a.parent) if (want.has(a.name)) return;  // top node only
    picked.push(o);
  });
  // Duplicate references (orangecrab's three G***): kicad-cli names every
  // node by the BARE reference, the parser keys the later blocks `REF~2`...,
  // so the k-th node of a name goes to the k-th block when the counts agree
  // -- otherwise (one footprint, two models) every node is that part's.
  const byName = {};
  for (const o of picked) (byName[o.name] = byName[o.name] || []).push(o);
  const assigned = [];
  for (const name of Object.keys(byName)) {
    const nodes = byName[name];
    const keys = [name];
    for (let k = 2; S.parts[name + '~' + k]; k++) keys.push(name + '~' + k);
    nodes.forEach((o, i) => assigned.push([o, (nodes.length === keys.length) ? keys[i] : name]));
  }
  let n = 0;
  for (const [o, key] of assigned) {
    const pose = glb.poses[key];
    const part = S.parts[key];
    if (!pose || !part) continue;
    // rest = F(final)^-1 * node (in mm): the model in its part's own frame,
    // so any later pose is F(now) * rest -- no knowledge of the model's own
    // offset needed.
    // F sits ON the part's face (the top face at y = d, the bottom at 0):
    // with F at y = 0 on both sides, a part that changed side landed one
    // board thickness off (the phase-6 verifier)
    const F = new THREE.Matrix4().makeRotationY(rad(pose[2]));
    if (pose[3] === 'B') F.multiply(new THREE.Matrix4().makeRotationX(Math.PI));
    F.setPosition(pose[0], pose[3] === 'B' ? 0 : S.scene.thickness, pose[1]);
    const world = new THREE.Matrix4().multiplyMatrices(mm, o.matrixWorld);
    const rest = new THREE.Matrix4().copy(F).invert().multiply(world);
    const clone = o.clone(true);
    clone.matrixAutoUpdate = false;
    clone.matrix.identity();
    clone.traverse(c => { if (c !== clone) c.matrixAutoUpdate = true; });
    // children keep their own local transforms; the clone's matrix is the
    // whole world transform, so strip its scale ancestry by construction
    root.add(clone);
    (part.glb = part.glb || []).push({ obj: clone, rest });
    n += 1;
  }
  return n;
}

// ---------------------------------------------------------------- camera
function frameCamera(scene, tl, W, H) {
  let [x0, y0, x1, y1] = scene.bounds;
  // a part off the outline (a pile beside the board, a glide's source) is
  // in the film too: the fit covers every pose any part takes
  for (const ep of tl.epochs) for (const ref of Object.keys(ep)) {
    const p = scene.parts[ref];
    if (!p) continue;
    const r = p.pads.length ? Math.max(...p.pads.map(q => Math.hypot(q[0], q[1]) + Math.max(q[2], q[3]) / 2)) : 1;
    const [px, py] = ep[ref];
    x0 = Math.min(x0, px - r); x1 = Math.max(x1, px + r);
    y0 = Math.min(y0, py - r); y1 = Math.max(y1, py + r);
  }
  const cx = (x0 + x1) / 2, cz = (y0 + y1) / 2;
  const r = 0.5 * Math.hypot(x1 - x0, y1 - y0) + 2;
  const fov = 30;
  const cam = new THREE.PerspectiveCamera(fov, W / H, 0.5, 100000);
  const fv = rad(fov), fh = 2 * Math.atan(Math.tan(fv / 2) * W / H);
  let dist = r / Math.sin(Math.min(fv, fh) / 2);
  const el = rad(52);                   // a 3/4 view from the board's near edge
  const cy = scene.thickness / 2;
  // Fit the board's OWN corners, not a bounding sphere: a long board seen at
  // 3/4 wastes most of a sphere fit.
  const corners = [];
  for (const x of [x0, x1]) for (const z of [y0, y1]) for (const y of [-3, scene.thickness + 3])
    corners.push(new THREE.Vector3(x, y, z));
  // ...and the board STANDING on its edge, mid-flip (turned about the
  // screen-vertical axis, its x-extent becomes height): the flat fit clipped
  // a wide board's top and bottom at the turn (the phase-6 verifier)
  const half = (x1 - x0) / 2 + 3;
  for (const y of [cy - half, cy + half]) for (const z of [y0, y1])
    corners.push(new THREE.Vector3(cx, y, z));
  for (let k = 0; k < 4; k++) {
    cam.position.set(cx, cy + dist * Math.sin(el), cz + dist * Math.cos(el));
    cam.lookAt(cx, cy, cz);
    cam.updateMatrixWorld(true);
    let ext = 0;
    for (const c of corners) {
      const p = c.clone().project(cam);
      ext = Math.max(ext, Math.abs(p.x), Math.abs(p.y));
    }
    dist *= ext / 0.9;                  // 10 % margin
  }
  cam.position.set(cx, cy + dist * Math.sin(el), cz + dist * Math.cos(el));
  cam.lookAt(cx, cy, cz);
  return { cam, cx, cz };
}

// ---------------------------------------------------------------- entry
window.stage3dInit = async function (o) {
  const scene = await (await fetch(o.sceneUrl)).json();
  const tl = await (await fetch(o.timelineUrl)).json();
  const W = o.width, H = o.height;
  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: false });
  renderer.setPixelRatio(1);
  renderer.setSize(W, H);
  renderer.setClearColor(rgb(o.colors.ground), 1);
  document.body.appendChild(renderer.domElement);
  const world = new THREE.Scene();
  world.add(new THREE.HemisphereLight(0xffffff, 0x404048, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(0.35, 1.0, 0.7);
  world.add(sun);
  const { cam, cx, cz } = frameCamera(scene, tl, W, H);
  // the flip pivot: the board turns about the screen-vertical axis through
  // its centre, like the X-ray's flip
  const pivot = new THREE.Group();
  pivot.position.set(cx, scene.thickness / 2, cz);
  const root = new THREE.Group();
  root.position.set(-cx, -scene.thickness / 2, -cz);
  pivot.add(root);
  world.add(pivot);
  root.add(buildBoard(scene, o.colors));
  buildCopper(tl, scene, o.colors, root);
  buildParts(scene, o.colors, root);
  buildPours(scene, tl, o.colors, root);
  S.scene = scene;
  let glbParts = 0, glbError = null;
  if (o.glbUrl && scene.glb) {
    // a GLB is decoration: a missing or corrupt one leaves the boxes
    try { glbParts = await loadGlb(o.glbUrl, scene.glb, root); }
    catch (e) { glbError = String(e).slice(0, 160); }
  }
  Object.assign(S, { renderer, world, cam, pivot, root, tl, scene, colors: o.colors });
  const gl = renderer.getContext();
  const ext = gl.getExtension('WEBGL_debug_renderer_info');
  return { renderer: ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER),
           three: THREE.REVISION, states: tl.states.length, glbParts, glbError };
};

window.renderState = function (i) {
  const st = S.tl.states[i];
  setCopper(st);
  setPours(st);
  setHighlight(st, S.tl, S.scene, S.colors, S.root);
  poseParts(st, S.tl);
  S.pivot.rotation.set(0, 0, st.angle);
  fitCamera();
  S.renderer.render(S.world, S.cam);
  return true;
};

// Fit the camera to what THIS state shows: the board, the parts where they
// are now, copper and pours, as posed and turned. The film-wide fit
// (`frameCamera`) had to cover the pile beside the board and the board
// standing on its edge mid-flip, so every ordinary frame drew the board at
// about a third of its box (esp_prog, run 35). A per-state fit pulls back
// while a pile or a turn needs the room and comes in again after, and it is
// still a pure function of the state.
const _fitBox = new THREE.Box3(), _b = new THREE.Box3();
function fitCamera() {
  S.world.updateMatrixWorld(true);
  _fitBox.makeEmpty();
  S.pivot.traverseVisible(o => {
    if (!o.isMesh || o.isInstancedMesh || !o.geometry || o.userData.noFit) return;
    if (!o.geometry.boundingBox) o.geometry.computeBoundingBox();
    _b.copy(o.geometry.boundingBox).applyMatrix4(o.matrixWorld);
    _fitBox.union(_b);
  });
  if (_fitBox.isEmpty()) return;
  const c = _fitBox.getCenter(new THREE.Vector3());
  const corners = [];
  for (const x of [_fitBox.min.x, _fitBox.max.x])
    for (const y of [_fitBox.min.y, _fitBox.max.y])
      for (const z of [_fitBox.min.z, _fitBox.max.z])
        corners.push(new THREE.Vector3(x, y, z));
  const el = rad(52);
  let dist = 0.6 * _fitBox.getSize(new THREE.Vector3()).length() / Math.tan(rad(15));
  for (let k = 0; k < 5; k++) {
    S.cam.position.set(c.x, c.y + dist * Math.sin(el), c.z + dist * Math.cos(el));
    S.cam.lookAt(c);
    S.cam.updateMatrixWorld(true);
    let ext = 0;
    for (const q of corners) {
      const p = q.clone().project(S.cam);
      ext = Math.max(ext, Math.abs(p.x), Math.abs(p.y));
    }
    dist *= ext / 0.92;                 // 8 % margin
  }
  S.cam.position.set(c.x, c.y + dist * Math.sin(el), c.z + dist * Math.cos(el));
  S.cam.lookAt(c);
  S.cam.updateMatrixWorld(true);
}

// For tests: every drawn part body's world-space y range and side at
// state `i`, so a body on the wrong side of the board is a number, not a
// picture someone has to notice.
window.stage3dProbe = function (i) {
  window.renderState(i);
  S.world.updateMatrixWorld(true);
  const out = {};
  for (const ref of Object.keys(S.parts)) {
    const p = S.parts[ref];
    if (!p.body || !p.body.visible || !p.grp.visible) continue;
    const box = new THREE.Box3().setFromObject(p.body);
    out[ref] = [box.min.y, box.max.y];
  }
  let custom = 0, holes = 0;
  for (const ref of Object.keys(S.parts)) {
    S.parts[ref].grp.traverse(o => {
      if (o.userData.custom) custom += 1;
      if (o.userData.hole) holes += 1;
    });
  }
  return { bodies: out, thickness: S.scene.thickness,
           pours: S.pours.filter(m => m.visible).length, pours_total: S.pours.length,
           custom, holes };
};

window.stage3dReady = true;
