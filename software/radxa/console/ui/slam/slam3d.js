// Tilted map view: the 2D lidar map seen from a top corner at 45 degrees,
// walls raised slightly, floor tiles flat, plus a proper little model of the
// robot (round wheels, body, camera head, lidar puck) placed at its pose.
// Self-contained WebGL1 + ANGLE_instanced_arrays; renders only when something
// changes, so an idle page costs no GPU time. Exposes window.Slam3D.
//
// Map cells use an instanced unit-cube pipeline: one draw call for every floor
// or wall tile. The robot is a separate, properly built triangle mesh (with
// cylinders for the wheels and lidar) drawn through a model matrix, so it reads
// as a rounded object and turns with the robot's heading instead of looking
// like a stack of boxes.
(function () {
  "use strict";
  // ---- instanced cube pipeline (floor + walls) ----
  const VS = `attribute vec3 aPos; attribute vec3 aNrm; attribute vec3 aOff; attribute vec3 aScl; attribute vec3 aCol;
uniform mat4 uMVP; varying vec3 vCol;
void main(){ vec3 p = aOff + aPos * aScl;
  float l = 0.55 + 0.45*max(dot(aNrm, normalize(vec3(0.4,0.3,0.85))), 0.0);
  vCol = aCol * l; gl_Position = uMVP * vec4(p, 1.0); }`;
  const FS = `precision mediump float; varying vec3 vCol; void main(){ gl_FragColor = vec4(vCol, 1.0); }`;
  // ---- plain mesh pipeline (the robot), positioned by a model matrix ----
  const MVS = `attribute vec3 aPos; attribute vec3 aNrm; attribute vec3 aCol;
uniform mat4 uVP; uniform mat4 uModel; varying vec3 vCol;
void main(){ vec4 wp = uModel * vec4(aPos, 1.0); vec3 wn = mat3(uModel) * aNrm;
  float l = 0.5 + 0.5*max(dot(normalize(wn), normalize(vec3(0.4,0.3,0.85))), 0.0);
  vCol = aCol * l; gl_Position = uVP * wp; }`;
  const LVS = `attribute vec3 aPos; uniform mat4 uMVP; void main(){ gl_Position = uMVP * vec4(aPos, 1.0); }`;
  const LFS = `precision mediump float; uniform vec4 uCol; void main(){ gl_FragColor = uCol; }`;

  function cubeGeometry() {
    // unit cube [0,1]^3 with per-face normals, 36 vertices
    const f = [
      [[0,0,1],[1,0,1],[1,1,1],[0,1,1],[0,0,1]], [[0,0,0],[0,1,0],[1,1,0],[1,0,0],[0,0,-1]],
      [[0,0,0],[1,0,0],[1,0,1],[0,0,1],[0,-1,0]], [[0,1,0],[0,1,1],[1,1,1],[1,1,0],[0,1,0]],
      [[0,0,0],[0,0,1],[0,1,1],[0,1,0],[-1,0,0]], [[1,0,0],[1,1,0],[1,1,1],[1,0,1],[1,0,0]]];
    const out = [];
    for (const [a, b, c, d, n] of f) for (const v of [a, b, c, a, c, d]) out.push(...v, ...n);
    return new Float32Array(out);
  }
  const mul = (a, b) => { const o = new Float32Array(16);
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
      let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + j] * b[i * 4 + k]; o[i * 4 + j] = s; } return o; };
  function persp(fov, asp, n, f) { const t = 1 / Math.tan(fov / 2), o = new Float32Array(16);
    o[0] = t / asp; o[5] = t; o[10] = (f + n) / (n - f); o[11] = -1; o[14] = 2 * f * n / (n - f); return o; }
  function lookAt(e, c, u) {
    const z = norm(sub(e, c)), x = norm(cross(u, z)), y = cross(z, x);
    return new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0,
      -dot(x, e), -dot(y, e), -dot(z, e), 1]); }
  // model matrix: translate to (px,py) then rotate about z by heading (column-major)
  function modelMat(px, py, th) { const c = Math.cos(th), s = Math.sin(th);
    return new Float32Array([c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, px, py, 0, 1]); }
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
  const hex = (h) => [parseInt(h.slice(1, 3), 16) / 255, parseInt(h.slice(3, 5), 16) / 255, parseInt(h.slice(5, 7), 16) / 255];
  const C_WALL = hex("#efe9e3"), C_FLOOR = hex("#2e2823");
  const WALL_H = 0.175;                  // visual wall height (3.5 map cells)

  // ---- robot mesh, built once in the robot frame (x forward, y left, z up),
  // metres, then scaled. A balancing robot reads mostly from its two big round
  // wheels, so those are real cylinders; the body, head and lidar sit between.
  const BOT_SCALE = 0.75;
  const C_BODY = hex("#f08a4b"), C_TOP = hex("#e07a3c"), C_TIRE = hex("#141210"),
        C_HUB = hex("#cfc8bf"), C_BELT = hex("#26201b"), C_HEAD = hex("#211d19"),
        C_LENS = hex("#0a0908"), C_LIDAR = hex("#cfc8bf"), C_BAND = hex("#1b1815"), C_NOSE = hex("#ffd9a8");
  const WR = 0.105;                       // wheel radius (sets ride height)

  function pushTri(a, p0, n0, p1, n1, p2, n2, col) {
    a.push(p0[0], p0[1], p0[2], n0[0], n0[1], n0[2], col[0], col[1], col[2]);
    a.push(p1[0], p1[1], p1[2], n1[0], n1[1], n1[2], col[0], col[1], col[2]);
    a.push(p2[0], p2[1], p2[2], n2[0], n2[1], n2[2], col[0], col[1], col[2]);
  }
  function addBox(a, c, s, col) {       // centre c, full size s
    const hx = s[0] / 2, hy = s[1] / 2, hz = s[2] / 2;
    const v = (sx, sy, sz) => [c[0] + sx * hx, c[1] + sy * hy, c[2] + sz * hz];
    const face = (n, q) => { pushTri(a, q[0], n, q[1], n, q[2], n, col); pushTri(a, q[0], n, q[2], n, q[3], n, col); };
    face([0,0,1],  [v(-1,-1,1), v(1,-1,1), v(1,1,1), v(-1,1,1)]);
    face([0,0,-1], [v(-1,1,-1), v(1,1,-1), v(1,-1,-1), v(-1,-1,-1)]);
    face([0,1,0],  [v(-1,1,-1), v(-1,1,1), v(1,1,1), v(1,1,-1)]);
    face([0,-1,0], [v(-1,-1,-1), v(1,-1,-1), v(1,-1,1), v(-1,-1,1)]);
    face([1,0,0],  [v(1,-1,-1), v(1,1,-1), v(1,1,1), v(1,-1,1)]);
    face([-1,0,0], [v(-1,-1,-1), v(-1,-1,1), v(-1,1,1), v(-1,1,-1)]);
  }
  function addCyl(a, axis, c, r, hl, seg, col) {  // axis 0=x 1=y 2=z, half-length hl
    const u = (axis + 1) % 3, v = (axis + 2) % 3;
    const at = (ring, end, sign) => { const p = [0, 0, 0], n = [0, 0, 0];
      p[axis] = c[axis] + end * hl; p[u] = c[u] + ring[0] * r; p[v] = c[v] + ring[1] * r;
      n[u] = ring[0] * sign; n[v] = ring[1] * sign; return [p, n]; };
    const ring = [];
    for (let i = 0; i < seg; i++) { const t = 2 * Math.PI * i / seg; ring.push([Math.cos(t), Math.sin(t)]); }
    for (let i = 0; i < seg; i++) {                 // side
      const r0 = ring[i], r1 = ring[(i + 1) % seg];
      const [a0, n0] = at(r0, -1, 1), [a1, n1] = at(r1, -1, 1), [b0, m0] = at(r0, 1, 1), [b1, m1] = at(r1, 1, 1);
      pushTri(a, a0, n0, a1, n1, b1, m1, col); pushTri(a, a0, n0, b1, m1, b0, m0, col);
    }
    for (const end of [-1, 1]) {                    // caps
      const cen = [0, 0, 0]; cen[axis] = c[axis] + end * hl; cen[u] = c[u]; cen[v] = c[v];
      const n = [0, 0, 0]; n[axis] = end;
      for (let i = 0; i < seg; i++) {
        const r0 = ring[i], r1 = ring[(i + 1) % seg];
        const e0 = [0, 0, 0], e1 = [0, 0, 0];
        e0[axis] = cen[axis]; e0[u] = c[u] + r0[0] * r; e0[v] = c[v] + r0[1] * r;
        e1[axis] = cen[axis]; e1[u] = c[u] + r1[0] * r; e1[v] = c[v] + r1[1] * r;
        if (end > 0) pushTri(a, cen, n, e0, n, e1, n, col); else pushTri(a, cen, n, e1, n, e0, n, col);
      }
    }
  }
  function robotMesh() {
    const a = [], k = BOT_SCALE, s = (arr) => arr.map((x) => x * k);
    const S = (x) => x * k;
    // wheels (axis = y, the lateral axis), with a lighter hub cap just outboard
    for (const side of [1, -1]) {
      addCyl(a, 1, s([0, side * 0.085, WR]), S(WR), S(0.018), 20, C_TIRE);
      addCyl(a, 1, s([0, side * 0.104, WR]), S(0.05), S(0.006), 16, C_HUB);
    }
    // body: a dark belt across the axle, the orange hull, a slightly tapered top
    addBox(a, s([0, 0, WR - 0.005]), s([0.142, 0.125, 0.045]), C_BELT);
    addBox(a, s([0, 0, WR + 0.055]), s([0.135, 0.115, 0.115]), C_BODY);
    addBox(a, s([0, 0, WR + 0.125]), s([0.10, 0.092, 0.03]), C_TOP);
    // camera head with a protruding lens, facing forward (+x)
    addBox(a, s([0.045, 0, WR + 0.12]), s([0.05, 0.085, 0.05]), C_HEAD);
    addCyl(a, 0, s([0.078, 0, WR + 0.12]), S(0.02), S(0.012), 16, C_LENS);
    // lidar puck on top, with a thin dark band
    addCyl(a, 2, s([0, 0, WR + 0.165]), S(0.05), S(0.018), 20, C_LIDAR);
    addCyl(a, 2, s([0, 0, WR + 0.155]), S(0.052), S(0.004), 20, C_BAND);
    // bright nose so the heading is obvious from any angle
    addBox(a, s([0.088, 0, WR + 0.015]), s([0.022, 0.03, 0.03]), C_NOSE);
    return new Float32Array(a);
  }

  function create(canvas) {
    const gl = canvas.getContext("webgl", { antialias: true, alpha: false });
    if (!gl) return null;
    const inst = gl.getExtension("ANGLE_instanced_arrays");
    if (!inst) return null;
    const prog = (vs, fs) => { const p = gl.createProgram();
      for (const [t, s] of [[gl.VERTEX_SHADER, vs], [gl.FRAGMENT_SHADER, fs]]) {
        const sh = gl.createShader(t); gl.shaderSource(sh, s); gl.compileShader(sh); gl.attachShader(p, sh); }
      gl.linkProgram(p); return p; };
    const P = prog(VS, FS), M = prog(MVS, FS), L = prog(LVS, LFS);   // robot reuses the colour FS
    const a = { pos: gl.getAttribLocation(P, "aPos"), nrm: gl.getAttribLocation(P, "aNrm"),
      off: gl.getAttribLocation(P, "aOff"), scl: gl.getAttribLocation(P, "aScl"), col: gl.getAttribLocation(P, "aCol"),
      mpos: gl.getAttribLocation(M, "aPos"), mnrm: gl.getAttribLocation(M, "aNrm"), mcol: gl.getAttribLocation(M, "aCol"),
      lpos: gl.getAttribLocation(L, "aPos") };
    const u = { mvp: gl.getUniformLocation(P, "uMVP"), vp: gl.getUniformLocation(M, "uVP"), model: gl.getUniformLocation(M, "uModel"),
      lmvp: gl.getUniformLocation(L, "uMVP"), lcol: gl.getUniformLocation(L, "uCol") };
    const cube = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, cube); gl.bufferData(gl.ARRAY_BUFFER, cubeGeometry(), gl.STATIC_DRAW);
    const bot = { buf: gl.createBuffer(), n: 0 };
    { const mesh = robotMesh(); gl.bindBuffer(gl.ARRAY_BUFFER, bot.buf); gl.bufferData(gl.ARRAY_BUFFER, mesh, gl.STATIC_DRAW); bot.n = mesh.length / 9; }
    const sets = { floor: { buf: gl.createBuffer(), n: 0 }, walls: { buf: gl.createBuffer(), n: 0 } };
    const lines = { grid: { buf: gl.createBuffer(), n: 0 }, path: { buf: gl.createBuffer(), n: 0 } };
    const cam = { yaw: -2.356, pitch: 0.785, dist: 7, tx: 0, ty: 0, follow: true };   // 45 deg, low-x/low-y corner
    let pose = [0, 0, 0], model = modelMat(0, 0, 0), dirty = true, raf = 0, visible = false;

    function upload(set, data) { gl.bindBuffer(gl.ARRAY_BUFFER, set.buf); gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW); set.n = data.length / 9; }
    function uploadLine(l, data) { gl.bindBuffer(gl.ARRAY_BUFFER, l.buf); gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW); l.n = data.length / 3; }

    function setScene(sc) {
      const r = sc.grid_res, o = sc.grid_origin;
      const f = new Float32Array(sc.floor.length * 9);
      sc.floor.forEach((c, i) => f.set([o + c[0] * r, o + c[1] * r, 0, r, r, 0.004, ...C_FLOOR], i * 9));
      upload(sets.floor, f);
      const w = new Float32Array(sc.walls.length * 9);
      sc.walls.forEach((c, i) => w.set([o + c[0] * r, o + c[1] * r, 0, r, r, WALL_H, ...C_WALL], i * 9));
      upload(sets.walls, w);
      uploadLine(lines.path, new Float32Array(sc.path.flatMap((p) => [p[0], p[1], 0.012])));
      if (!lines.grid.n) { const g = []; for (let i = -20; i <= 20; i++) g.push(i, -20, -0.001, i, 20, -0.001, -20, i, -0.001, 20, i, -0.001);
        uploadLine(lines.grid, new Float32Array(g)); }
      setPose(sc.pose);
    }
    function setPose(p) {
      if (!p) return; pose = p;
      model = modelMat(p[0], p[1], p[2]);
      if (cam.follow) { cam.tx = p[0]; cam.ty = p[1]; }
      request();
    }
    function resize() { const r = canvas.getBoundingClientRect(), d = Math.min(2, window.devicePixelRatio || 1);
      canvas.width = Math.max(1, r.width * d); canvas.height = Math.max(1, r.height * d); request(); }
    function request() { dirty = true; if (visible && !raf) raf = requestAnimationFrame(render); }
    function render() {
      raf = 0; if (!dirty || !visible) return; dirty = false;
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(0.059, 0.051, 0.047, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); gl.enable(gl.DEPTH_TEST);
      const eye = [cam.tx + cam.dist * Math.cos(cam.pitch) * Math.cos(cam.yaw), cam.ty + cam.dist * Math.cos(cam.pitch) * Math.sin(cam.yaw),
        cam.dist * Math.sin(cam.pitch)];
      const mvp = mul(persp(0.9, canvas.width / canvas.height, 0.05, 200), lookAt(eye, [cam.tx, cam.ty, 0.1], [0, 0, 1]));
      // ground grid + travelled path
      gl.useProgram(L); gl.uniformMatrix4fv(u.lmvp, false, mvp); gl.enableVertexAttribArray(a.lpos);
      for (const [l, col] of [[lines.grid, [0.16, 0.14, 0.12, 1]], [lines.path, [0.94, 0.54, 0.29, 1]]]) {
        if (!l.n) continue; gl.uniform4fv(u.lcol, col); gl.bindBuffer(gl.ARRAY_BUFFER, l.buf);
        gl.vertexAttribPointer(a.lpos, 3, gl.FLOAT, false, 0, 0); gl.drawArrays(l === lines.grid ? gl.LINES : gl.LINE_STRIP, 0, l.n); }
      gl.disableVertexAttribArray(a.lpos);
      // floor + wall tiles (instanced cubes)
      gl.useProgram(P); gl.uniformMatrix4fv(u.mvp, false, mvp);
      gl.bindBuffer(gl.ARRAY_BUFFER, cube);
      gl.enableVertexAttribArray(a.pos); gl.vertexAttribPointer(a.pos, 3, gl.FLOAT, false, 24, 0);
      gl.enableVertexAttribArray(a.nrm); gl.vertexAttribPointer(a.nrm, 3, gl.FLOAT, false, 24, 12);
      for (const set of [sets.floor, sets.walls]) {
        if (!set.n) continue; gl.bindBuffer(gl.ARRAY_BUFFER, set.buf);
        gl.enableVertexAttribArray(a.off); gl.vertexAttribPointer(a.off, 3, gl.FLOAT, false, 36, 0); inst.vertexAttribDivisorANGLE(a.off, 1);
        gl.enableVertexAttribArray(a.scl); gl.vertexAttribPointer(a.scl, 3, gl.FLOAT, false, 36, 12); inst.vertexAttribDivisorANGLE(a.scl, 1);
        gl.enableVertexAttribArray(a.col); gl.vertexAttribPointer(a.col, 3, gl.FLOAT, false, 36, 24); inst.vertexAttribDivisorANGLE(a.col, 1);
        inst.drawArraysInstancedANGLE(gl.TRIANGLES, 0, 36, set.n); }
      inst.vertexAttribDivisorANGLE(a.off, 0); inst.vertexAttribDivisorANGLE(a.scl, 0); inst.vertexAttribDivisorANGLE(a.col, 0);
      gl.disableVertexAttribArray(a.off); gl.disableVertexAttribArray(a.scl); gl.disableVertexAttribArray(a.col);
      gl.disableVertexAttribArray(a.pos); gl.disableVertexAttribArray(a.nrm);
      // the robot (plain mesh, placed by its model matrix)
      if (bot.n) {
        gl.useProgram(M); gl.uniformMatrix4fv(u.vp, false, mvp); gl.uniformMatrix4fv(u.model, false, model);
        gl.bindBuffer(gl.ARRAY_BUFFER, bot.buf);
        gl.enableVertexAttribArray(a.mpos); gl.vertexAttribPointer(a.mpos, 3, gl.FLOAT, false, 36, 0);
        gl.enableVertexAttribArray(a.mnrm); gl.vertexAttribPointer(a.mnrm, 3, gl.FLOAT, false, 36, 12);
        gl.enableVertexAttribArray(a.mcol); gl.vertexAttribPointer(a.mcol, 3, gl.FLOAT, false, 36, 24);
        gl.drawArrays(gl.TRIANGLES, 0, bot.n);
        gl.disableVertexAttribArray(a.mpos); gl.disableVertexAttribArray(a.mnrm); gl.disableVertexAttribArray(a.mcol);
      }
    }
    // orbit (1 finger / left drag), pan (2 fingers / right drag), zoom (pinch / wheel)
    const pts = new Map(); let last = null;
    canvas.addEventListener("pointerdown", (e) => { pts.set(e.pointerId, [e.clientX, e.clientY]); canvas.setPointerCapture(e.pointerId); last = null; });
    canvas.addEventListener("pointerup", (e) => { pts.delete(e.pointerId); last = null; });
    canvas.addEventListener("pointercancel", (e) => { pts.delete(e.pointerId); last = null; });
    canvas.addEventListener("contextmenu", (e) => e.preventDefault());
    canvas.addEventListener("pointermove", (e) => {
      if (!pts.has(e.pointerId)) return; const prev = pts.get(e.pointerId); pts.set(e.pointerId, [e.clientX, e.clientY]);
      const dx = e.clientX - prev[0], dy = e.clientY - prev[1];
      if (pts.size === 1 && !(e.buttons & 2)) { cam.yaw -= dx * 0.008; cam.pitch = Math.min(1.5, Math.max(0.08, cam.pitch + dy * 0.008)); }
      else if (pts.size === 1) pan(dx, dy);
      else { const [p, q] = [...pts.values()], d = Math.hypot(p[0] - q[0], p[1] - q[1]);
        if (last) { cam.dist = Math.min(60, Math.max(0.6, cam.dist * last / d)); pan(dx / 2, dy / 2); } last = d; }
      request(); });
    function pan(dx, dy) { cam.follow = false; const k = cam.dist * 0.0018, c = Math.cos(cam.yaw), s = Math.sin(cam.yaw);
      cam.tx += (dx * s - dy * c) * k; cam.ty += (-dx * c - dy * s) * k; }
    canvas.addEventListener("wheel", (e) => { e.preventDefault(); cam.dist = Math.min(60, Math.max(0.6, cam.dist * (e.deltaY > 0 ? 1.12 : 1 / 1.12))); request(); }, { passive: false });
    window.addEventListener("resize", () => visible && resize());
    return { setScene, setPose, resize,
      show(v) { visible = v; if (v) resize(); },
      center() { cam.follow = true; setPose(pose); },
      stats: () => ({ walls: sets.walls.n, floor: sets.floor.n }) };
  }
  window.Slam3D = { create };
})();
