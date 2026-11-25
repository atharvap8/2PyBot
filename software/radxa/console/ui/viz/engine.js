/* viz/engine.js - the console's live 3D view of the robot.
 *
 * One scene, one robot (the real printed CAD from robot.json), three camera modes:
 *   parked  studio hero shot, floor reflection
 *   drive   chase camera over the mapped world, route ribbon, obstacles
 *   park    top-down with proximity arcs
 * Switching mode never cuts: the camera orbits from one rig to the next while the
 * studio floor, the world layers and the arcs cross-fade. Everything is drawn in
 * the browser; the robot only serves this file, the model and its usual data.
 *
 * Frames: the "robot frame" is x right / y forward / z up (the model's axes). The
 * world is the SLAM map frame (x, y on the floor, heading from +x). three.js is
 * y-up, so everything lives under one z-up group.
 */
import * as THREE from '../vendor/three/three.module.min.js';
import { RoomEnvironment } from '../vendor/three/RoomEnvironment.js';
import { RoundedBoxGeometry } from '../vendor/three/RoundedBoxGeometry.js';
import { toCreasedNormals } from '../vendor/three/BufferGeometryUtils.js';

const D2R = Math.PI / 180, TAU = Math.PI * 2;
const AXLE = 0.05;                       // wheel radius = axle height, metres
const WALL_H = 0.2;
const C = { fog: '#16120e', wall: '#3b3530', scan: '#4a433c', blob: '#4a433d', floor: '#ffffff', floorOp: 0.05,
            lane: '#8f857b', path: '#38bdf8', cliff: '#ff4d6d', shadow: 0.62,
            pool: ['52,43,35', '25,21,15', '14,12,11'], red: '#ff3d3d', amber: '#f0a53b', far: '#b9afa4' };

// Camera rigs, in the robot frame. az/r/h is a cylindrical position so a mode
// change orbits the robot instead of flying through it.
export const VIEWS = {
  parked: { az: 125.4 * D2R, r: 1.3125, h: 0.36, look: [0, 0, 0.135], up: [0, 0, 1], fov: 26, shift: 0.035,
            key: [-0.9, 1.4, 1.8], rim: [1.1, -1.2, 0.9], keyI: 4.4, fillI: 1.5, park: 0,
            studio: 1, world: 0, arcs: 0, shadow: 1, fog: [40, 60], lean: 0, near: 0.02 },
  drive:  { az: -90 * D2R, r: 1.55, h: 0.95, look: [0, 1.2, 0], up: [0, 0, 1], fov: 50, shift: 0.05,
            key: [-0.9, -1.2, 1.9], rim: [1.0, 1.4, 0.8], keyI: 3.0, fillI: 0.9, park: 0,
            studio: 0, world: 1, arcs: 0, shadow: 1, fog: [2.9, 6.3], lean: 1, near: 0.7 },
  park:   { az: -90 * D2R, r: 0.03, h: 1.55, look: [0, 0.03, 0], up: [0, 1, 0], fov: 22, shift: 0,
            key: [-1.1, -0.8, 2.2], rim: [0.9, 1.0, 0.9], keyI: 2.6, fillI: 0, park: 1,
            studio: 0, world: 0, arcs: 1, shadow: 0, fog: [40, 60], lean: 1, near: 0.02 },
};
// Named points on the robot that the shell hangs labels on (robot frame, metres).
const ANCHORS = { lidar: [0, 0, 0.283], camera: [0.026, 0.049, 0.203], wheel: [-0.105, 0, AXLE] };
// Proximity sectors: centre angle from +x (forward is 90 deg) and the hull radius that way.
export const SECTORS = [
  { a: 90, hull: 0.09 }, { a: 45, hull: 0.115 }, { a: 0, hull: 0.105 }, { a: -45, hull: 0.105 },
  { a: -90, hull: 0.06 }, { a: -135, hull: 0.105 }, { a: 180, hull: 0.105 }, { a: 135, hull: 0.115 },
];

const ease = (t) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);
const lerp = (a, b, t) => a + (b - a) * t;
const lerpN = (a, b, t) => a.map((v, i) => lerp(v, b[i], t));
const angDiff = (a, b) => ((b - a + Math.PI) % TAU + TAU) % TAU - Math.PI;
function mixView(a, b, t) {
  const o = {};
  for (const k of Object.keys(b)) {
    if (k === 'az') o.az = a.az + angDiff(a.az, b.az) * t;
    else if (Array.isArray(b[k])) o[k] = lerpN(a[k], b[k], t);
    else o[k] = lerp(a[k], b[k], t);
  }
  return o;
}

function canvasTex(w, h, paint) {
  const c = document.createElement('canvas'); c.width = w; c.height = h; paint(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}

/** Merge occupied grid cells into as few rectangles as possible: [i0, j0, w, h] in cells. */
export function mergeCells(cells, dropLonely = false) {
  const K = 16384, set = new Set();
  for (const c of cells) set.add(c[0] * K + c[1]);
  if (dropLonely) {
    for (const k of [...set]) {
      const i = Math.floor(k / K), j = k % K; let n = 0;
      for (let di = -1; di <= 1 && !n; di++) for (let dj = -1; dj <= 1; dj++) if ((di || dj) && set.has((i + di) * K + j + dj)) { n = 1; break; }
      if (!n) set.delete(k);
    }
  }
  const keys = [...set].sort((p, q) => (p % K) - (q % K) || p - q), out = [];
  for (const k of keys) {
    if (!set.has(k)) continue;
    const i0 = Math.floor(k / K), j0 = k % K; let w = 1, h = 1;
    while (set.has((i0 + w) * K + j0)) w++;
    grow: for (;;) { for (let d = 0; d < w; d++) if (!set.has((i0 + d) * K + j0 + h)) break grow; h++; }
    for (let d = 0; d < w; d++) for (let e = 0; e < h; e++) set.delete((i0 + d) * K + j0 + e);
    out.push([i0, j0, w, h]);
  }
  return out;
}

export async function createEngine(canvas, host, opts = {}) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
  renderer.setClearColor(0x000000, 0);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.3;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const scene = new THREE.Scene();
  scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
  scene.fog = new THREE.Fog(C.fog, 40, 60);
  const zup = new THREE.Group(); zup.rotation.x = -Math.PI / 2; scene.add(zup);
  const rig = new THREE.Group(); zup.add(rig);                 // follows the robot; cameras and lights hang off it
  const camera = new THREE.PerspectiveCamera(26, 1, 0.02, 80);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x4a3d32, 1.15));
  const fill = new THREE.DirectionalLight(0xffffff, 1.5);
  const aim = new THREE.Object3D(); aim.position.set(0, 0, 0.12); rig.add(aim);
  fill.target = aim; scene.add(fill);
  const key = new THREE.DirectionalLight(0xfff3e6, 4.4);
  key.castShadow = true; key.shadow.mapSize.set(2048, 2048);
  Object.assign(key.shadow.camera, { left: -0.45, right: 0.45, top: 0.45, bottom: -0.45, near: 0.2, far: 8 });
  key.shadow.bias = -0.0003; key.shadow.normalBias = 0.003; key.target = aim; rig.add(key);
  const rim = new THREE.DirectionalLight(0xcfe2ff, 2.6); rim.target = aim; rig.add(rim);

  // ------------------------------------------------------------ shared bits
  const blobTex = canvasTex(256, 256, (g) => {
    const gr = g.createRadialGradient(128, 128, 0, 128, 128, 128);
    gr.addColorStop(0, 'rgba(0,0,0,1)'); gr.addColorStop(0.45, 'rgba(0,0,0,.5)'); gr.addColorStop(1, 'rgba(0,0,0,0)');
    g.fillStyle = gr; g.fillRect(0, 0, 256, 256);
  });
  const contact = (w, d, opacity) => {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, d),
      new THREE.MeshBasicMaterial({ map: blobTex, transparent: true, opacity, depthWrite: false, toneMapped: false }));
    m.position.z = 0.0015; m.renderOrder = 2; return m;
  };
  const fadeTex = (a0, a1) => canvasTex(256, 4, (g, w, h) => {
    const gr = g.createLinearGradient(0, 0, w, 0);
    gr.addColorStop(0, `rgba(255,255,255,${a0})`); gr.addColorStop(1, `rgba(255,255,255,${a1})`);
    g.fillStyle = gr; g.fillRect(0, 0, w, h);
  });
  const worldMats = [];                              // every world material fades with the "world" factor
  const wm = (m, base = 1) => { m.transparent = true; m.userData.base = base; worldMats.push(m); return m; };

  // ------------------------------------------------------------ the robot
  const robot = { root: new THREE.Group(), body: new THREE.Group(), wheelL: new THREE.Group(), wheelR: new THREE.Group() };
  const mats = {
    shell: new THREE.MeshPhysicalMaterial({ color: 0x2b2b2f, roughness: 0.5, clearcoat: 0.35, clearcoatRoughness: 0.5, envMapIntensity: 1.5 }),
    base: new THREE.MeshStandardMaterial({ color: 0x232325, roughness: 0.85, envMapIntensity: 0.5 }),
    accent: new THREE.MeshPhysicalMaterial({ color: 0xff8226, roughness: 0.58, clearcoat: 0.15, clearcoatRoughness: 0.6, envMapIntensity: 1.0 }),
    tyre: new THREE.MeshStandardMaterial({ color: 0x1c1c1d, roughness: 0.88, envMapIntensity: 0.8 }),
    hub: new THREE.MeshStandardMaterial({ color: 0x38383c, roughness: 0.4, metalness: 0.55, envMapIntensity: 1.2 }),
    glass: new THREE.MeshPhysicalMaterial({ color: 0x1b1d23, roughness: 0.2, metalness: 0.45, clearcoat: 1, clearcoatRoughness: 0.04, envMapIntensity: 1.5 }),
    led: new THREE.MeshBasicMaterial({ color: 0x55f2d4, toneMapped: false }),
  };
  for (const m of Object.values(mats)) m.userData = { env: m.envMapIntensity, cc: m.clearcoat, rough: m.roughness, parkEnv: 0.2 };
  mats.glass.userData.parkEnv = 0.035; mats.glass.userData.parkRough = 0.4;

  const model = await (await fetch(opts.model || '/ui/viz/robot.json')).json();
  robot.body.position.z = AXLE;                      // the body pitches about the axle
  for (const p of model.parts) {
    let g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(p.vertices, 3));
    g.setIndex(p.faces);
    g = toCreasedNormals(g, 34 * D2R);               // CAD shading: smooth curves, crisp edges
    const mesh = new THREE.Mesh(g, mats[p.material] || mats.shell);
    mesh.castShadow = true; mesh.receiveShadow = true; mesh.position.z = -AXLE; mesh.name = p.name;
    robot.body.add(mesh);
  }
  { // lidar dome and LED ring: not in the CAD export, so parametric
    const top = 0.247 - AXLE;
    const ring = new THREE.Mesh(new THREE.CylinderGeometry(0.046, 0.046, 0.004, 56), mats.shell);
    ring.rotation.x = Math.PI / 2; ring.position.z = top + 0.002; ring.castShadow = true; robot.body.add(ring);
    const dome = new THREE.Mesh(new THREE.CylinderGeometry(0.0335, 0.036, 0.027, 56), mats.glass);
    dome.rotation.x = Math.PI / 2; dome.position.z = top + 0.0175; dome.castShadow = true; robot.body.add(dome);
    const led = new THREE.InstancedMesh(new THREE.BoxGeometry(0.0045, 0.0045, 0.0022), mats.led, 16), m4 = new THREE.Matrix4();
    for (let i = 0; i < 16; i++) { const a = i / 16 * TAU;
      m4.makeRotationZ(a).setPosition(Math.cos(a) * 0.0415, Math.sin(a) * 0.0415, top + 0.005); led.setMatrixAt(i, m4); }
    robot.body.add(led);
  }
  for (const [grp, side] of [[robot.wheelL, -1], [robot.wheelR, 1]]) {     // wheels: parametric, with tread
    grp.position.set(side * 0.092, 0, AXLE);
    const tyre = new THREE.Mesh(new THREE.CylinderGeometry(0.0468, 0.0468, 0.024, 44), mats.tyre);
    tyre.rotation.z = Math.PI / 2; tyre.castShadow = true; grp.add(tyre);
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.027, 0.027, 0.0255, 32), mats.hub);
    hub.rotation.z = Math.PI / 2; grp.add(hub);
    const N = 26, lugs = new THREE.InstancedMesh(new THREE.BoxGeometry(0.0118, 0.0034, 0.0078), mats.tyre, N * 2);
    const rx = new THREE.Matrix4(), tr = new THREE.Matrix4(), tw = new THREE.Matrix4(), m4 = new THREE.Matrix4();
    for (let row = 0; row < 2; row++) for (let i = 0; i < N; i++) {
      const s = row ? 1 : -1;
      rx.makeRotationX((i + row * 0.5) / N * TAU); tr.makeTranslation(s * 0.0062, 0.0483, 0); tw.makeRotationY(s * 0.5);
      lugs.setMatrixAt(row * N + i, m4.copy(rx).multiply(tr).multiply(tw));
    }
    lugs.castShadow = true; grp.add(lugs);
  }
  robot.root.add(robot.body, robot.wheelL, robot.wheelR, contact(0.44, 0.34, C.shadow));
  zup.add(robot.root);

  // ------------------------------------------------------------ studio (parked)
  const mirror = new THREE.Group();
  mirror.add(robot.body.clone(true), robot.wheelL.clone(true), robot.wheelR.clone(true));
  mirror.scale.z = -1;
  mirror.traverse((o) => { if (o.isMesh) { o.castShadow = false; o.receiveShadow = false; } });
  robot.root.add(mirror);
  const veil = new THREE.Mesh(new THREE.PlaneGeometry(8, 8), new THREE.MeshBasicMaterial({ transparent: true, depthWrite: false, toneMapped: false,
    map: canvasTex(512, 512, (g) => {
      const mix = (a, b, t) => a.split(',').map((v, i) => Math.round(+v + (+b.split(',')[i] - +v) * t)).join(',');
      const gr = g.createRadialGradient(256, 256, 0, 256, 256, 256), p = C.pool;
      gr.addColorStop(0, `rgba(${p[0]},.78)`); gr.addColorStop(0.06, `rgba(${mix(p[0], p[1], 0.38)},.91)`);
      gr.addColorStop(0.115, `rgba(${mix(p[0], p[1], 0.72)},1)`); gr.addColorStop(0.16, `rgba(${p[1]},1)`);
      gr.addColorStop(0.38, `rgba(${p[2]},1)`); gr.addColorStop(1, `rgba(${p[2]},1)`);
      g.fillStyle = gr; g.fillRect(0, 0, 512, 512);
    }) }));
  veil.renderOrder = -2; rig.add(veil);
  const catcher = new THREE.Mesh(new THREE.PlaneGeometry(4, 4), new THREE.ShadowMaterial({ opacity: C.shadow * 0.75 }));
  catcher.position.z = 0.001; catcher.receiveShadow = true; catcher.renderOrder = 1; rig.add(catcher);

  // ------------------------------------------------------------ world layers (drive)
  const world = new THREE.Group(); zup.add(world);
  const unitBox = new THREE.BoxGeometry(1, 1, 1); unitBox.translate(0.5, 0.5, 0.5);
  const wallMat = wm(new THREE.MeshStandardMaterial({ color: C.wall, roughness: 0.92, envMapIntensity: 0.55 }));
  const floorMat = wm(new THREE.MeshBasicMaterial({ color: C.floor, depthWrite: false, toneMapped: false }), C.floorOp);
  let walls = null, floor = null, mapVersion = null;
  const known = { set: new Set(), res: 0.05, origin: 0 }, KK = 16384;
  const m4 = new THREE.Matrix4();
  function setScene(sc) {
    if (!sc || sc.version === mapVersion) return;
    mapVersion = sc.version;
    for (const o of [walls, floor]) if (o) { world.remove(o); o.geometry !== unitBox && o.geometry.dispose(); }
    walls = floor = null;
    const r = sc.grid_res, o = sc.grid_origin;
    known.set = new Set((sc.walls || []).map((c) => c[0] * KK + c[1])); known.res = r; known.origin = o;
    const wr = mergeCells(sc.walls || [], true);
    if (wr.length) {
      walls = new THREE.InstancedMesh(unitBox, wallMat, wr.length);
      wr.forEach(([i, j, w, h], n) => { m4.makeScale(w * r, h * r, WALL_H).setPosition(o + i * r, o + j * r, 0); walls.setMatrixAt(n, m4); });
      world.add(walls);
    }
    const fr = mergeCells(sc.floor || []);
    if (fr.length) {
      const pos = new Float32Array(fr.length * 18);
      fr.forEach(([i, j, w, h], n) => { const x0 = o + i * r, y0 = o + j * r, x1 = x0 + w * r, y1 = y0 + h * r;
        pos.set([x0, y0, 0, x1, y0, 0, x1, y1, 0, x0, y0, 0, x1, y1, 0, x0, y1, 0], n * 18); });
      const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      floor = new THREE.Mesh(g, floorMat); floor.position.z = 0.0004; floor.renderOrder = -1; world.add(floor);
    }
    dirty();
  }

  // Live lidar returns the map does not already explain, joined into one low wall
  // (robot frame). Unmapped obstacles show up; mapped walls are not drawn twice.
  const SCAN_SEG = 460, BEHIND = -0.35;                // anything further behind sits between the robot and the chase camera
  const scanPos = new Float32Array(SCAN_SEG * 54), scanGeo = new THREE.BufferGeometry();
  scanGeo.setAttribute('position', new THREE.BufferAttribute(scanPos, 3).setUsage(THREE.DynamicDrawUsage)); scanGeo.setDrawRange(0, 0);
  const scan = new THREE.Mesh(scanGeo, wm(new THREE.MeshStandardMaterial({ color: C.wall, roughness: 0.92, envMapIntensity: 0.55, flatShading: true, side: THREE.DoubleSide })));
  scan.frustumCulled = false; robot.root.add(scan);
  function setScan(pts) {                              // [[forward, left], ...] metres, robot frame
    const c = Math.cos(pose.th), sn = Math.sin(pose.th), has = known.set.size > 0, n = pts ? pts.length : 0, keep = new Array(n);
    for (let k = 0; k < n; k++) {
      const f = pts[k][0], l = pts[k][1]; keep[k] = null;
      if (f < BEHIND) continue;
      if (has) {                                        // already a wall on the map: the map draws it
        const i = Math.floor((pose.x + c * f - sn * l - known.origin) / known.res), j = Math.floor((pose.y + sn * f + c * l - known.origin) / known.res);
        let hit = false;
        for (let di = -1; di <= 1 && !hit; di++) for (let dj = -1; dj <= 1; dj++) if (known.set.has((i + di) * KK + j + dj)) { hit = true; break; }
        if (hit) continue;
      }
      keep[k] = [-l, f, Math.hypot(f, l) || 1e-3];      // x right, y forward, range
    }
    let v = 0, segs = 0; const T = 0.03, used = new Uint8Array(n);
    const quad = (ax, ay, az, bx, by, bz, cx, cy, cz, dx, dy, dz) => { scanPos.set([ax, ay, az, bx, by, bz, cx, cy, cz, ax, ay, az, cx, cy, cz, dx, dy, dz], v); v += 18; };
    const wall = (ax, ay, ar, bx, by, br) => {          // front, top and back of one stretch, T thick away from the robot
      if (segs >= SCAN_SEG) return; segs++;
      const ox = ax + ax / ar * T, oy = ay + ay / ar * T, px = bx + bx / br * T, py = by + by / br * T;
      quad(ax, ay, 0, bx, by, 0, bx, by, WALL_H, ax, ay, WALL_H); quad(ax, ay, WALL_H, bx, by, WALL_H, px, py, WALL_H, ox, oy, WALL_H); quad(px, py, 0, ox, oy, 0, ox, oy, WALL_H, px, py, WALL_H);
    };
    for (let k = 0; k < n; k++) {
      const p = keep[k], q2 = keep[(k + 1) % n];
      if (!p || !q2 || Math.hypot(q2[0] - p[0], q2[1] - p[1]) > Math.max(0.1, 0.075 * p[2])) continue;
      wall(p[0], p[1], p[2], q2[0], q2[1], q2[2]); used[k] = used[(k + 1) % n] = 1;
    }
    for (let k = 0; k < n; k++) {                       // a lone return (a chair leg): a short stub so it is not lost
      const p = keep[k]; if (!p || used[k]) continue;
      const tx = -p[1] / p[2] * 0.025, ty = p[0] / p[2] * 0.025;
      wall(p[0] - tx, p[1] - ty, p[2], p[0] + tx, p[1] + ty, p[2]);
    }
    scanGeo.attributes.position.needsUpdate = true; scanGeo.setDrawRange(0, v / 3); dirty();
  }

  // depth-sensor obstacles (map frame) clustered into soft volumes; drop-offs as red discs
  const BLOB_MAX = 24, blobMat = wm(new THREE.MeshStandardMaterial({ color: C.blob, roughness: 0.78, envMapIntensity: 0.7 }));
  const blobs = new THREE.InstancedMesh(new RoundedBoxGeometry(1, 1, 1, 4, 0.22), blobMat, BLOB_MAX);
  const blobShadows = new THREE.InstancedMesh(new THREE.PlaneGeometry(1, 1), wm(new THREE.MeshBasicMaterial({ map: blobTex, depthWrite: false, toneMapped: false }), C.shadow * 0.8), BLOB_MAX);
  blobs.count = blobShadows.count = 0; blobs.frustumCulled = blobShadows.frustumCulled = false; blobShadows.renderOrder = 2;
  const cliffs = new THREE.InstancedMesh(new THREE.CircleGeometry(0.03, 20), wm(new THREE.MeshBasicMaterial({ color: C.cliff, depthWrite: false, toneMapped: false }), 0.85), 64);
  cliffs.count = 0; cliffs.frustumCulled = false; cliffs.renderOrder = 3;
  world.add(blobs, blobShadows, cliffs);
  function setObstacles(points, drops) {
    const cell = 0.12, groups = new Map();
    for (const p of points || []) {
      const k = Math.round(p[0] / cell) + ',' + Math.round(p[1] / cell), g = groups.get(k);
      if (g) { g[0] = Math.min(g[0], p[0]); g[1] = Math.max(g[1], p[0]); g[2] = Math.min(g[2], p[1]); g[3] = Math.max(g[3], p[1]); g[4] = Math.max(g[4], p[2] || 0); }
      else groups.set(k, [p[0], p[0], p[1], p[1], p[2] || 0]);
    }
    let n = 0; const ct = Math.cos(pose.th), st = Math.sin(pose.th);
    for (const g of groups.values()) {
      if (n >= BLOB_MAX) break;
      if (((g[0] + g[1]) / 2 - pose.x) * ct + ((g[2] + g[3]) / 2 - pose.y) * st < BEHIND) continue;   // behind the robot: in the chase camera's way
      const w = Math.max(0.09, g[1] - g[0] + 0.07), d = Math.max(0.09, g[3] - g[2] + 0.07), h = Math.max(0.07, g[4] + 0.02);
      const cx = (g[0] + g[1]) / 2, cy = (g[2] + g[3]) / 2;
      m4.makeScale(w, d, h).setPosition(cx, cy, h / 2); blobs.setMatrixAt(n, m4);
      m4.makeScale(w * 1.9, d * 1.9, 1).setPosition(cx, cy, 0.0015); blobShadows.setMatrixAt(n, m4); n++;
    }
    blobs.count = blobShadows.count = n; blobs.instanceMatrix.needsUpdate = blobShadows.instanceMatrix.needsUpdate = true;
    const c = Math.min(64, drops ? drops.length : 0);
    for (let i = 0; i < c; i++) { m4.makeTranslation(drops[i][0], drops[i][1], 0.005); cliffs.setMatrixAt(i, m4); }
    cliffs.count = c; cliffs.instanceMatrix.needsUpdate = true; dirty();
  }

  // planned route: a ribbon that is brightest at the robot, plus the swept corridor
  const routeGroup = new THREE.Group(); world.add(routeGroup);
  const pathMat = wm(new THREE.MeshBasicMaterial({ color: C.path, map: fadeTex(1, 0.6), depthWrite: false, side: THREE.DoubleSide, toneMapped: false }));
  const laneMat = wm(new THREE.MeshBasicMaterial({ color: C.lane, map: fadeTex(0.75, 0.1), depthWrite: false, side: THREE.DoubleSide, toneMapped: false }));
  const goalMat = wm(new THREE.MeshBasicMaterial({ color: C.path, depthWrite: false, side: THREE.DoubleSide, toneMapped: false }));
  function ribbon(pts, width, mat) {
    const n = pts.length, pos = [], uv = [], idx = [];
    for (let i = 0; i < n; i++) {
      const p = pts[i], q = pts[Math.min(n - 1, i + 1)], o = pts[Math.max(0, i - 1)];
      let tx = q[0] - o[0], ty = q[1] - o[1]; const L = Math.hypot(tx, ty) || 1; tx /= L; ty /= L;
      pos.push(p[0] - ty * width / 2, p[1] + tx * width / 2, 0.004, p[0] + ty * width / 2, p[1] - tx * width / 2, 0.004);
      uv.push(i / (n - 1), 0, i / (n - 1), 1);
      if (i < n - 1) idx.push(i * 2, i * 2 + 1, i * 2 + 2, i * 2 + 1, i * 2 + 3, i * 2 + 2);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2)); g.setIndex(idx);
    const m = new THREE.Mesh(g, mat); m.renderOrder = 3; m.frustumCulled = false; return m;
  }
  const offset = (pts, d) => pts.map((p, i) => { const q = pts[Math.min(pts.length - 1, i + 1)], o = pts[Math.max(0, i - 1)];
    let tx = q[0] - o[0], ty = q[1] - o[1]; const L = Math.hypot(tx, ty) || 1; return [p[0] - ty / L * d, p[1] + tx / L * d]; });
  let routeKey = '', goalAt = null;
  function setRoute(plan, goal) {
    const pts = (plan || []).filter((p) => Array.isArray(p) && isFinite(p[0]) && isFinite(p[1]));
    const key = pts.length + ':' + (pts.length ? pts[0].join() + pts[pts.length - 1].join() : '') + ':' + (goal ? goal.x + ',' + goal.y : '');
    if (key === routeKey) return; routeKey = key;
    for (const o of [...routeGroup.children]) { routeGroup.remove(o); o.geometry.dispose(); }
    goalAt = goal && isFinite(goal.x) ? [goal.x, goal.y] : null;
    if (pts.length >= 2) {
      const n = Math.min(240, Math.max(24, pts.length * 4));
      const p = new THREE.CatmullRomCurve3(pts.map((q) => new THREE.Vector3(q[0], q[1], 0)), false, 'centripetal').getPoints(n).map((v) => [v.x, v.y]);
      routeGroup.add(ribbon(p, 0.052, pathMat), ribbon(offset(p, 0.15), 0.012, laneMat), ribbon(offset(p, -0.15), 0.012, laneMat));
    }
    if (goalAt) {
      const ringM = new THREE.Mesh(new THREE.RingGeometry(0.085, 0.11, 48), goalMat), dot = new THREE.Mesh(new THREE.CircleGeometry(0.035, 32), goalMat);
      for (const o of [ringM, dot]) { o.position.set(goalAt[0], goalAt[1], 0.005); o.renderOrder = 3; routeGroup.add(o); }
    }
    dirty();
  }

  // ------------------------------------------------------------ proximity arcs (park)
  const ARC_R = 0.17;
  const arcs = SECTORS.map((s) => {
    const mk = (w, base) => { const m = new THREE.Mesh(new THREE.RingGeometry(ARC_R - w, ARC_R + w, 40, 1, (s.a - 19) * D2R, 38 * D2R),
      new THREE.MeshBasicMaterial({ color: C.red, transparent: true, opacity: 0, depthWrite: false, toneMapped: false, side: THREE.DoubleSide }));
      m.position.z = 0.006; m.renderOrder = 4; m.userData.base = base; robot.root.add(m); return m; };
    return { line: mk(0.006, 1), glow: mk(0.016, 0.2), hull: s.hull, alpha: 0 };
  });
  function setProximity(clear) {                       // clearance in metres per SECTORS entry, or null
    arcs.forEach((a, i) => {
      const c = clear ? clear[i] : null;
      if (c == null || !isFinite(c) || c > 0.6) { a.alpha = 0; return; }
      const col = c < 0.12 ? C.red : c < 0.3 ? C.amber : C.far;
      a.line.material.color.set(col); a.glow.material.color.set(col);
      a.alpha = c < 0.3 ? 1 : 0.55;
      const s = (a.hull + Math.min(0.14, Math.max(0.035, c * 0.5))) / ARC_R;
      a.line.scale.set(s, s, 1); a.glow.scale.set(s, s, 1);
    });
    dirty();
  }

  // ------------------------------------------------------------ state + frame loop
  let cur = { ...VIEWS.parked, look: [...VIEWS.parked.look] }, from = cur, to = VIEWS.parked, tT = 1, tDur = 1.15, modeName = 'parked';
  const pose = { x: 0, y: 0, th: Math.PI / 2 }, shown = { x: 0, y: 0, th: Math.PI / 2 }, camPose = { x: 0, y: 0, th: Math.PI / 2 };
  const tel = { pitch: 0, v: 0, rpmL: null, rpmR: null }, spin = { l: 0, r: 0 }, orbit = { az: 0, h: 0 };
  let needs = 4, last = 0, raf = 0, alive = true, slowT = 0;
  const frameCbs = [], q = { dpr: Math.min(2, window.devicePixelRatio || 1), ema: 16, bad: 0, level: 0 };
  const dirty = (n = 2) => { needs = Math.max(needs, n); };
  const v3 = new THREE.Vector3(), size = { w: 1, h: 1 };

  function setMode(name, o = {}) {
    if (!VIEWS[name] || name === modeName) return;
    from = cur; to = VIEWS[name]; modeName = name; tT = o.instant ? 1 : 0; tDur = (o.duration || 1.15) * (opts.slow || 1);
    if (o.instant) cur = mixView(from, to, 1);
    dirty();
  }
  function resize() {
    const w = Math.max(1, host.clientWidth), h = Math.max(1, host.clientHeight);
    if (w === size.w && h === size.h) return;
    size.w = w; size.h = h; renderer.setPixelRatio(q.dpr); renderer.setSize(w, h, false); camera.aspect = w / h; dirty();
  }
  const ro = new ResizeObserver(resize); ro.observe(host);

  function frame(now) {
    if (!alive) return;
    raf = requestAnimationFrame(frame);
    const dt = Math.min(0.1, (now - last) / 1000 || 0.016);
    const moving = tT < 1 || Math.abs(tel.v) > 0.004 || Math.abs(angDiff(shown.th, pose.th)) > 0.002
      || Math.hypot(pose.x - shown.x, pose.y - shown.y) > 0.002 || Math.abs(orbit.az) > 1e-4 && cur.studio < 0.99;
    if (!moving && needs <= 0) { slowT += dt; if (slowT < 0.5) { last = now; return; } }   // idle: about 2 fps
    slowT = 0; last = now; needs--;
    resize();

    if (tT < 1) { tT = Math.min(1, tT + dt / tDur); cur = mixView(from, to, ease(tT)); }
    // follow the robot: the model tracks the pose quickly, the camera a little slower
    const k1 = 1 - Math.exp(-dt / 0.12), k2 = 1 - Math.exp(-dt / 0.3);
    shown.x += (pose.x - shown.x) * k1; shown.y += (pose.y - shown.y) * k1; shown.th += angDiff(shown.th, pose.th) * k1;
    camPose.x += (pose.x - camPose.x) * k1; camPose.y += (pose.y - camPose.y) * k1; camPose.th += angDiff(camPose.th, pose.th) * k2;
    robot.root.position.set(shown.x, shown.y, 0); robot.root.rotation.z = shown.th - Math.PI / 2;
    rig.position.set(camPose.x, camPose.y, 0); rig.rotation.z = camPose.th - Math.PI / 2;
    // rig animation: lean with pitch (not in the studio shot), wheels from real rpm when present
    robot.body.rotation.x = -tel.pitch * D2R * cur.lean;
    const w0 = tel.v / AXLE;
    spin.l += (tel.rpmL != null ? tel.rpmL * TAU / 60 : w0) * dt; spin.r += (tel.rpmR != null ? tel.rpmR * TAU / 60 : w0) * dt;
    robot.wheelL.rotation.x = -spin.l; robot.wheelR.rotation.x = -spin.r;
    if (cur.studio < 0.5) orbit.az *= Math.exp(-dt / 0.25);

    rig.updateMatrixWorld(true);
    const az = cur.az + orbit.az, h = Math.max(0.08, cur.h + orbit.h * cur.studio);
    camera.position.copy(rig.localToWorld(v3.set(cur.r * Math.cos(az), cur.r * Math.sin(az), h)));
    camera.up.copy(v3.set(...cur.up).transformDirection(rig.matrixWorld));
    camera.lookAt(rig.localToWorld(v3.set(...cur.look)));
    camera.fov = cur.fov; camera.near = cur.near;
    camera.setViewOffset(size.w, size.h, 0, size.h * cur.shift, size.w, size.h);
    camera.updateProjectionMatrix();
    key.position.set(...cur.key); key.intensity = cur.keyI; rim.position.set(...cur.rim);
    fill.position.copy(camera.position); fill.intensity = cur.fillI;
    scene.fog.near = cur.fog[0]; scene.fog.far = cur.fog[1];

    for (const m of Object.values(mats)) {
      const u = m.userData;
      if (u.env != null) m.envMapIntensity = u.env * lerp(1, u.parkEnv, cur.park);
      if (u.cc) m.clearcoat = u.cc * lerp(1, 0.15, cur.park);
      if (u.parkRough) m.roughness = lerp(u.rough, u.parkRough, cur.park);
    }
    veil.visible = cur.studio > 0.004; veil.material.opacity = cur.studio;
    mirror.visible = cur.studio > 0.985 && q.level < 2;
    catcher.material.opacity = C.shadow * 0.75 * cur.shadow;
    const wv = cur.world > 0.004; world.visible = wv; scan.visible = wv;
    for (const m of worldMats) m.opacity = m.userData.base * cur.world;
    for (const a of arcs) { const o = a.alpha * cur.arcs; a.line.visible = a.glow.visible = o > 0.004;
      a.line.material.opacity = o; a.glow.material.opacity = o * a.glow.userData.base; }

    renderer.render(scene, camera);
    for (const cb of frameCbs) cb(cur);

    // adapt quality to the device: first resolution, then shadows and reflection
    q.ema += (dt * 1000 - q.ema) * 0.08;
    if (moving && q.ema > 30) { if (++q.bad > 50) { q.bad = 0;
      if (q.dpr > 1) { q.dpr = Math.max(1, q.dpr - 0.5); size.w = 0; }
      else if (q.level < 2) { q.level++; renderer.shadowMap.enabled = q.level < 1; key.castShadow = q.level < 1;
        for (const m of [...Object.values(mats), wallMat, blobMat]) m.needsUpdate = true; } } }
    else q.bad = Math.max(0, q.bad - 1);
  }
  raf = requestAnimationFrame(frame);

  // drag to orbit the studio shot
  let drag = null;
  canvas.addEventListener('pointerdown', (e) => { if (cur.studio > 0.9) { drag = [e.clientX, e.clientY]; canvas.setPointerCapture(e.pointerId); } });
  canvas.addEventListener('pointermove', (e) => { if (!drag) return;
    orbit.az -= (e.clientX - drag[0]) * 0.006; orbit.h = Math.max(-0.2, Math.min(0.9, orbit.h + (e.clientY - drag[1]) * 0.003));
    drag = [e.clientX, e.clientY]; dirty(); });
  for (const t of ['pointerup', 'pointercancel']) canvas.addEventListener(t, () => { drag = null; });
  canvas.addEventListener('webglcontextlost', (e) => { e.preventDefault(); });
  canvas.addEventListener('webglcontextrestored', () => dirty(6));

  function project(local, parent) {                    // -> [x, y] in CSS px of the host, or null if behind the camera
    v3.set(local[0], local[1], local[2]); parent.localToWorld(v3); v3.project(camera);
    return v3.z > 1 ? null : [(v3.x * 0.5 + 0.5) * size.w, (-v3.y * 0.5 + 0.5) * size.h];
  }
  return {
    setMode, mode: () => modeName, view: () => cur, transitioning: () => tT < 1,
    setPose(p) { if (p && isFinite(p[0]) && isFinite(p[1]) && isFinite(p[2])) { pose.x = p[0]; pose.y = p[1]; pose.th = p[2]; } },
    snapPose() { Object.assign(shown, pose); Object.assign(camPose, pose); dirty(); },
    setTelemetry(t) { Object.assign(tel, t); },
    setScene, setScan, setObstacles, setRoute, setProximity,
    anchors() { const o = {}; for (const k of Object.keys(ANCHORS)) o[k] = project(ANCHORS[k], robot.root);
      o.goal = goalAt ? project([goalAt[0], goalAt[1], 0.02], zup) : null; return o; },
    onFrame(fn) { frameCbs.push(fn); }, invalidate: dirty, resize,
    stats: () => ({ frameMs: +q.ema.toFixed(1), dpr: q.dpr, level: q.level, triangles: renderer.info.render.triangles, calls: renderer.info.render.calls }),
    dispose() { alive = false; cancelAnimationFrame(raf); ro.disconnect(); renderer.dispose(); },
  };
}
