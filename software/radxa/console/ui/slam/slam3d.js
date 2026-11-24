// Tilted map view: the 2D lidar map seen from a top corner at 45 degrees,
// walls raised slightly, floor tiles flat, plus a simple assembled model of the
// robot (body, two wheels, camera head, lidar puck) scaled to its real size.
// Self-contained WebGL1 + ANGLE_instanced_arrays; renders only when something
// changes, so an idle page costs no GPU time. Exposes window.Slam3D.
//
// Instance layout: [offx, offy, offz, sx, sy, sz, r, g, b] (9 floats). Each cube
// spans [off, off + scale]. Floor and wall cells share one size; the robot parts
// each carry their own size, which is how one pipeline draws all of them.
(function () {
  "use strict";
  const VS = `attribute vec3 aPos; attribute vec3 aNrm; attribute vec3 aOff; attribute vec3 aScl; attribute vec3 aCol;
uniform mat4 uMVP; varying vec3 vCol;
void main(){ vec3 p = aOff + aPos * aScl;
  float l = 0.55 + 0.45*max(dot(aNrm, normalize(vec3(0.4,0.3,0.85))), 0.0);
  vCol = aCol * l; gl_Position = uMVP * vec4(p, 1.0); }`;
  const FS = `precision mediump float; varying vec3 vCol; void main(){ gl_FragColor = vec4(vCol, 1.0); }`;
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
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
  const hex = (h) => [parseInt(h.slice(1, 3), 16) / 255, parseInt(h.slice(3, 5), 16) / 255, parseInt(h.slice(5, 7), 16) / 255];
  const C_WALL = hex("#efe9e3"), C_FLOOR = hex("#2e2823");
  const WALL_H = 0.175;                  // visual wall height (3.5 map cells)

  // Simple robot, parts in the robot frame (x forward, y left, z up), metres.
  // [cx, cy, cz, sx, sy, sz, colour]. Boxes stay axis-aligned; their centres are
  // rotated by the heading, which is enough to read the robot's pose and facing.
  const BOT_SCALE = 0.6;   // compact marker, smaller than life size
  const C_BODY = hex("#f08a4b"), C_DARK = hex("#1f1b18"), C_LIDAR = hex("#cfc8bf"), C_NOSE = hex("#ffd9a8");
  const BOT_PARTS = [
    [0.00,  0.000, 0.105, 0.145, 0.095, 0.150, C_BODY],   // body
    [0.00,  0.088, 0.052, 0.104, 0.028, 0.104, C_DARK],   // left wheel
    [0.00, -0.088, 0.052, 0.104, 0.028, 0.104, C_DARK],   // right wheel
    [0.052, 0.000, 0.205, 0.050, 0.075, 0.055, C_DARK],   // camera head
    [0.00,  0.000, 0.260, 0.062, 0.062, 0.030, C_LIDAR],  // lidar puck
    [0.088, 0.000, 0.150, 0.030, 0.030, 0.030, C_NOSE],   // front marker
  ];

  function create(canvas) {
    const gl = canvas.getContext("webgl", { antialias: true, alpha: false });
    if (!gl) return null;
    const inst = gl.getExtension("ANGLE_instanced_arrays");
    if (!inst) return null;
    const prog = (vs, fs) => { const p = gl.createProgram();
      for (const [t, s] of [[gl.VERTEX_SHADER, vs], [gl.FRAGMENT_SHADER, fs]]) {
        const sh = gl.createShader(t); gl.shaderSource(sh, s); gl.compileShader(sh); gl.attachShader(p, sh); }
      gl.linkProgram(p); return p; };
    const P = prog(VS, FS), L = prog(LVS, LFS);
    const a = { pos: gl.getAttribLocation(P, "aPos"), nrm: gl.getAttribLocation(P, "aNrm"),
      off: gl.getAttribLocation(P, "aOff"), scl: gl.getAttribLocation(P, "aScl"), col: gl.getAttribLocation(P, "aCol"),
      lpos: gl.getAttribLocation(L, "aPos") };
    const u = { mvp: gl.getUniformLocation(P, "uMVP"), lmvp: gl.getUniformLocation(L, "uMVP"), lcol: gl.getUniformLocation(L, "uCol") };
    const cube = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, cube); gl.bufferData(gl.ARRAY_BUFFER, cubeGeometry(), gl.STATIC_DRAW);
    const sets = { floor: { buf: gl.createBuffer(), n: 0 }, walls: { buf: gl.createBuffer(), n: 0 }, bot: { buf: gl.createBuffer(), n: 0 } };
    const lines = { grid: { buf: gl.createBuffer(), n: 0 }, path: { buf: gl.createBuffer(), n: 0 } };
    const cam = { yaw: -2.356, pitch: 0.785, dist: 7, tx: 0, ty: 0, follow: true };   // 45 deg, low-x/low-y corner
    let pose = [0, 0, 0], dirty = true, raf = 0, visible = false;

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
      const c = Math.cos(p[2]), sn = Math.sin(p[2]);
      const data = new Float32Array(BOT_PARTS.length * 9);
      BOT_PARTS.forEach((part, i) => {
        const k = BOT_SCALE;
        const cx = part[0] * k, cy = part[1] * k, cz = part[2] * k, sx = part[3] * k, sy = part[4] * k, sz = part[5] * k, col = part[6];
        const wx = p[0] + c * cx - sn * cy, wy = p[1] + sn * cx + c * cy;   // rotate centre by heading
        data.set([wx - sx / 2, wy - sy / 2, cz - sz / 2, sx, sy, sz, ...col], i * 9);
      });
      upload(sets.bot, data);
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
      gl.useProgram(L); gl.uniformMatrix4fv(u.lmvp, false, mvp); gl.enableVertexAttribArray(a.lpos);
      for (const [l, col] of [[lines.grid, [0.16, 0.14, 0.12, 1]], [lines.path, [0.94, 0.54, 0.29, 1]]]) {
        if (!l.n) continue; gl.uniform4fv(u.lcol, col); gl.bindBuffer(gl.ARRAY_BUFFER, l.buf);
        gl.vertexAttribPointer(a.lpos, 3, gl.FLOAT, false, 0, 0); gl.drawArrays(l === lines.grid ? gl.LINES : gl.LINE_STRIP, 0, l.n); }
      gl.useProgram(P); gl.uniformMatrix4fv(u.mvp, false, mvp);
      gl.bindBuffer(gl.ARRAY_BUFFER, cube);
      gl.enableVertexAttribArray(a.pos); gl.vertexAttribPointer(a.pos, 3, gl.FLOAT, false, 24, 0);
      gl.enableVertexAttribArray(a.nrm); gl.vertexAttribPointer(a.nrm, 3, gl.FLOAT, false, 24, 12);
      for (const set of [sets.floor, sets.walls, sets.bot]) {
        if (!set.n) continue; gl.bindBuffer(gl.ARRAY_BUFFER, set.buf);
        gl.enableVertexAttribArray(a.off); gl.vertexAttribPointer(a.off, 3, gl.FLOAT, false, 36, 0); inst.vertexAttribDivisorANGLE(a.off, 1);
        gl.enableVertexAttribArray(a.scl); gl.vertexAttribPointer(a.scl, 3, gl.FLOAT, false, 36, 12); inst.vertexAttribDivisorANGLE(a.scl, 1);
        gl.enableVertexAttribArray(a.col); gl.vertexAttribPointer(a.col, 3, gl.FLOAT, false, 36, 24); inst.vertexAttribDivisorANGLE(a.col, 1);
        inst.drawArraysInstancedANGLE(gl.TRIANGLES, 0, 36, set.n); }
      inst.vertexAttribDivisorANGLE(a.off, 0); inst.vertexAttribDivisorANGLE(a.scl, 0); inst.vertexAttribDivisorANGLE(a.col, 0);
      gl.disableVertexAttribArray(a.off); gl.disableVertexAttribArray(a.scl); gl.disableVertexAttribArray(a.col);
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
