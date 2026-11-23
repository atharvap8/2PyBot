/* 2PyBot "Drive" view: a chase-camera reality view in the style of a car
 * autopilot display. Canvas 2D perspective renderer, no WebGL, no assets.
 *   - map walls as grey columns, live lidar returns as glowing posts
 *   - the safety bubble on the floor (green / amber / red)
 *   - the planned path as a blue ribbon, the goal as a beacon
 *   - area labels floating above the floor
 * Robot frame / map frame: x forward, y left, z up (metres).
 */
"use strict";
(function () {
  const COL = {
    bg0: "#07090c", bg1: "#10151b", grid: "rgba(120,140,160,0.10)",
    wall: [176, 186, 198], scanNear: [255, 77, 109], scanFar: [80, 200, 255],
    path: "rgba(56,189,248,0.55)", pathEdge: "rgba(125,211,252,0.95)",
    trail: "rgba(240,138,75,0.35)", robot: "#f08a4b", label: "#efe9e3",
    ok: "#34d399", warn: "#f0b84b", bad: "#ff4d6d", mute: "#73695f",
  };

  function create(canvas) {
    const ctx = canvas.getContext("2d");
    let S = null, scene = null, walls = [], shown = false, raf = 0, hud = null;
    let pose = null;                                   // smoothed display pose
    const cam = { yaw: 0, dist: 1.9, height: 1.15, fov: 62 };
    let W = 1, H = 1, dpr = 1;

    function resize() {
      const r = canvas.getBoundingClientRect();
      dpr = window.devicePixelRatio || 1;
      W = Math.max(1, r.width); H = Math.max(1, r.height);
      canvas.width = W * dpr; canvas.height = H * dpr;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    window.addEventListener("resize", () => { if (shown) resize(); });

    // ------------------------------------------------------------ input: orbit + zoom
    let drag = null;
    canvas.addEventListener("pointerdown", (e) => { drag = { x: e.clientX, y: e.clientY }; canvas.setPointerCapture(e.pointerId); });
    canvas.addEventListener("pointermove", (e) => {
      if (!drag) return;
      cam.yaw -= (e.clientX - drag.x) * 0.006;
      cam.height = Math.max(0.35, Math.min(4, cam.height + (e.clientY - drag.y) * 0.006));
      drag = { x: e.clientX, y: e.clientY };
    });
    canvas.addEventListener("pointerup", () => { drag = null; });
    canvas.addEventListener("dblclick", () => { cam.yaw = 0; cam.dist = 1.9; cam.height = 1.15; });
    canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      cam.dist = Math.max(0.6, Math.min(8, cam.dist * (e.deltaY < 0 ? 0.9 : 1.1)));
    }, { passive: false });

    // ------------------------------------------------------------ data
    function setScene(sc) {
      scene = sc;
      // pool the 5 cm wall cells to 10 cm columns
      const seen = new Set(), out = [], k = sc.grid_res;
      for (const [c, r] of sc.walls || []) {
        const x = sc.grid_origin + (c + 0.5) * k, y = sc.grid_origin + (r + 0.5) * k;
        const key = Math.round(x * 10) + "," + Math.round(y * 10);
        if (seen.has(key)) continue;
        seen.add(key); out.push([Math.round(x * 10) / 10, Math.round(y * 10) / 10]);
      }
      walls = out;
      api.version = sc.version;
    }
    function update(state) { S = state; }

    // ------------------------------------------------------------ camera
    let C = null;
    function setupCamera() {
      const th = pose[2] + cam.yaw;
      const fx = Math.cos(th), fy = Math.sin(th);
      const eye = [pose[0] - fx * cam.dist, pose[1] - fy * cam.dist, cam.height];
      const look = [pose[0] + fx * 1.2, pose[1] + fy * 1.2, 0.15];
      let f = [look[0] - eye[0], look[1] - eye[1], look[2] - eye[2]];
      const n = Math.hypot(...f); f = f.map((v) => v / n);
      let r = [f[1], -f[0], 0]; const rn = Math.hypot(...r); r = r.map((v) => v / rn);
      const u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]];
      const focal = (H / 2) / Math.tan((cam.fov * Math.PI) / 360);
      C = { eye, f, r, u, focal };
    }
    function proj(x, y, z) {
      const dx = x - C.eye[0], dy = y - C.eye[1], dz = z - C.eye[2];
      const d = dx * C.f[0] + dy * C.f[1] + dz * C.f[2];
      if (d < 0.08) return null;
      const sx = dx * C.r[0] + dy * C.r[1] + dz * C.r[2];
      const sy = dx * C.u[0] + dy * C.u[1] + dz * C.u[2];
      return [W / 2 + (C.focal * sx) / d, H / 2 - (C.focal * sy) / d, d];
    }
    function lineW(pts, close) {
      let started = false;
      ctx.beginPath();
      for (const p of pts) {
        const q = proj(p[0], p[1], p[2] || 0);
        if (!q) { started = false; continue; }
        if (!started) { ctx.moveTo(q[0], q[1]); started = true; } else ctx.lineTo(q[0], q[1]);
      }
      if (close) ctx.closePath();
    }
    const mix = (a, b, t) => a.map((v, i) => Math.round(v + (b[i] - v) * t));

    // ------------------------------------------------------------ drawing
    function drawBackground() {
      const g = ctx.createLinearGradient(0, 0, 0, H);
      g.addColorStop(0, COL.bg1); g.addColorStop(0.55, COL.bg0); g.addColorStop(1, "#0b0e12");
      ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);
    }
    function drawGrid() {
      ctx.strokeStyle = COL.grid; ctx.lineWidth = 1;
      const cx = Math.round(pose[0]), cy = Math.round(pose[1]), R = 9;
      for (let i = -R; i <= R; i++) {
        lineW([[cx + i, cy - R, 0], [cx + i, cy + R, 0]]); ctx.stroke();
        lineW([[cx - R, cy + i, 0], [cx + R, cy + i, 0]]); ctx.stroke();
      }
    }
    function drawTrail() {
      const P = (S.path || []).slice(-400);
      if (P.length < 2) return;
      ctx.strokeStyle = COL.trail; ctx.lineWidth = 2;
      lineW(P.map((p) => [p[0], p[1], 0.005])); ctx.stroke();
    }
    function drawPlan() {
      const P = S.plan || [];
      if (P.length >= 2) {
        // ribbon: offset both sides by 9 cm
        const left = [], right = [];
        for (let i = 0; i < P.length; i++) {
          const a = P[Math.max(0, i - 1)], b = P[Math.min(P.length - 1, i + 1)];
          let tx = b[0] - a[0], ty = b[1] - a[1]; const n = Math.hypot(tx, ty) || 1; tx /= n; ty /= n;
          left.push([P[i][0] - ty * 0.09, P[i][1] + tx * 0.09, 0.01]);
          right.push([P[i][0] + ty * 0.09, P[i][1] - tx * 0.09, 0.01]);
        }
        ctx.fillStyle = COL.path; lineW(left.concat(right.reverse()), true); ctx.fill();
        ctx.strokeStyle = COL.pathEdge; ctx.lineWidth = 1.5; lineW(P.map((p) => [p[0], p[1], 0.012])); ctx.stroke();
      }
      if (S.goal) {
        const g = S.goal, t = (Date.now() % 1600) / 1600;
        ctx.strokeStyle = COL.pathEdge; ctx.lineWidth = 2;
        const ring = (rad, z) => lineW(Array.from({ length: 25 }, (_, i) => {
          const a = (i / 24) * 2 * Math.PI; return [g.x + rad * Math.cos(a), g.y + rad * Math.sin(a), z];
        }), true);
        ring(0.15, 0.01); ctx.stroke();
        ctx.globalAlpha = 1 - t; ring(0.15 + t * 0.35, 0.01); ctx.stroke(); ctx.globalAlpha = 1;
        lineW([[g.x, g.y, 0], [g.x, g.y, 0.7]]); ctx.stroke();
      }
    }
    function collectColumns(items) {
      // walls: grey columns within 7 m
      const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
      for (const [x, y] of walls) {
        const dx = x - pose[0], dy = y - pose[1];
        if (dx * dx + dy * dy > 49) continue;
        const b = proj(x, y, 0), t = proj(x, y, 0.32);
        if (!b || !t) continue;
        items.push({ d: b[2], kind: 0, b, t });
      }
      // live lidar returns: glowing posts at scan height
      for (const p of S.scan || []) {
        const x = pose[0] + c * p[0] - s * p[1], y = pose[1] + s * p[0] + c * p[1];
        const r = Math.hypot(p[0], p[1]);
        const b = proj(x, y, 0), t = proj(x, y, 0.265);
        if (!b || !t) continue;
        items.push({ d: b[2], kind: 1, b, t, near: Math.max(0, Math.min(1, (r - 0.25) / 1.5)) });
      }
    }
    function drawColumns(items) {
      items.sort((a, b) => b.d - a.d);
      for (const it of items) {
        const w = Math.max(1, Math.min(9, 40 / it.d));
        if (it.kind === 0) {
          const fade = Math.max(0.18, Math.min(0.9, 1.4 - it.d / 6));
          const [r, g, b] = COL.wall;
          ctx.strokeStyle = `rgba(${r},${g},${b},${fade * 0.55})`; ctx.lineWidth = w;
          ctx.beginPath(); ctx.moveTo(it.b[0], it.b[1]); ctx.lineTo(it.t[0], it.t[1]); ctx.stroke();
          ctx.fillStyle = `rgba(230,236,242,${fade})`; ctx.fillRect(it.t[0] - w / 2, it.t[1] - 1, w, 2);
        } else {
          const [r, g, b] = mix(COL.scanNear, COL.scanFar, it.near);
          ctx.strokeStyle = `rgba(${r},${g},${b},0.35)`; ctx.lineWidth = Math.max(1, w * 0.5);
          ctx.beginPath(); ctx.moveTo(it.b[0], it.b[1]); ctx.lineTo(it.t[0], it.t[1]); ctx.stroke();
          ctx.fillStyle = `rgb(${r},${g},${b})`;
          ctx.shadowColor = `rgb(${r},${g},${b})`; ctx.shadowBlur = 8;
          ctx.fillRect(it.t[0] - w * 0.4, it.t[1] - w * 0.4, w * 0.8, w * 0.8);
          ctx.shadowBlur = 0;
        }
      }
    }
    function drawBubble() {
      const B = S.bubble;
      if (!B || !B.footprint) return;
      const [x0, x1, y0, y1] = B.footprint, m = B.stop_margin || 0.06;
      const col = { clear: COL.ok, limiting: COL.warn, stale: COL.mute }[B.state] || COL.bad;
      const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
      const W_ = (px, py, z) => [pose[0] + c * px - s * py, pose[1] + s * px + c * py, z || 0.004];
      // margin zone (filled, translucent) and the travel corridor ahead/behind up to the stopping distance
      ctx.fillStyle = col + "33"; ctx.strokeStyle = col; ctx.lineWidth = 2;
      lineW([W_(x1 + m, y0 - 0.03), W_(x1 + m, y1 + 0.03), W_(x0 - m, y1 + 0.03), W_(x0 - m, y0 - 0.03)], true);
      ctx.fill(); ctx.stroke();
      const reach = (cap) => Math.min(1.5, cap * cap / (2 * 0.6) + cap * 0.25);
      if (B.front_m !== null && B.front_m !== undefined) {
        const f = Math.min(B.front_m, 3);
        ctx.strokeStyle = B.forward_cap < 0.05 ? COL.bad : col; ctx.setLineDash([6, 6]);
        lineW([W_(x1 + f, y0 - 0.03), W_(x1 + f, y1 + 0.03)]); ctx.stroke(); ctx.setLineDash([]);
      }
      const fr = reach(Math.min(B.forward_cap || 0, 1.0));
      if (fr > 0.02) {                                  // stopping distance at the current cap
        ctx.fillStyle = col + "22";
        lineW([W_(x1 + m, y0), W_(x1 + m + fr, y0), W_(x1 + m + fr, y1), W_(x1 + m, y1)], true); ctx.fill();
      }
    }
    function drawRobot() {
      const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
      const W_ = (px, py, z) => [pose[0] + c * px - s * py, pose[1] + s * px + c * py, z];
      const box = (xa, xb, ya, yb, za, zb, fill, edge) => {
        const v = [[xa, ya], [xb, ya], [xb, yb], [xa, yb]];
        const faces = [
          [0, 1], [1, 2], [2, 3], [3, 0],
        ].map(([i, j]) => [W_(v[i][0], v[i][1], za), W_(v[j][0], v[j][1], za), W_(v[j][0], v[j][1], zb), W_(v[i][0], v[i][1], zb)]);
        faces.push(v.map((p) => W_(p[0], p[1], zb)));
        const withD = faces.map((f) => {
          const q = f.map((p) => proj(...p)); if (q.some((x) => !x)) return null;
          return { f, d: q.reduce((a, x) => a + x[2], 0) / q.length };
        }).filter(Boolean).sort((a, b) => b.d - a.d);
        for (const it of withD) { ctx.fillStyle = fill; ctx.strokeStyle = edge; ctx.lineWidth = 1; lineW(it.f, true); ctx.fill(); ctx.stroke(); }
      };
      box(-0.052, 0.052, -0.125, -0.095, 0, 0.104, "#2a2f36", "#4b5563");   // right wheel
      box(-0.052, 0.052, 0.095, 0.125, 0, 0.104, "#2a2f36", "#4b5563");     // left wheel
      box(-0.05, 0.05, -0.09, 0.09, 0.06, 0.25, "rgba(240,138,75,0.92)", "#7c3f1c");   // body
      box(-0.035, 0.035, -0.066, 0.004, 0.25, 0.28, "#1f2937", "#60a5fa");   // lidar puck
      // heading chevron
      ctx.fillStyle = COL.robot;
      lineW([W_(0.22, 0, 0.005), W_(0.14, 0.05, 0.005), W_(0.16, 0, 0.005), W_(0.14, -0.05, 0.005)], true); ctx.fill();
    }
    function drawLabels() {
      ctx.font = "600 13px Inter, Segoe UI, sans-serif"; ctx.textAlign = "center";
      for (const l of S.labels || []) {
        const q = proj(l.x, l.y, 0.55), b = proj(l.x, l.y, 0);
        if (!q || !b || q[2] > 14) continue;
        ctx.strokeStyle = "rgba(240,138,75,0.6)"; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(b[0], b[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
        const w = ctx.measureText(l.name).width + 14;
        ctx.fillStyle = "rgba(15,18,22,0.85)"; ctx.fillRect(q[0] - w / 2, q[1] - 20, w, 20);
        ctx.strokeStyle = "rgba(240,138,75,0.8)"; ctx.strokeRect(q[0] - w / 2, q[1] - 20, w, 20);
        ctx.fillStyle = COL.label; ctx.fillText(l.name, q[0], q[1] - 6);
      }
      ctx.textAlign = "left";
    }
    function drawHud() {
      const B = S.bubble || {};
      const mode = { idle: "LIDAR ONLY", mapping: "MAPPING", localizing: "ON MAP" }[S.mode] || S.mode;
      const auto = hud && hud.state === "running";
      ctx.font = "700 12px Inter, Segoe UI, sans-serif";
      const badge = (x, text, col) => {
        const w = ctx.measureText(text).width + 18;
        ctx.fillStyle = "rgba(10,12,15,0.75)"; ctx.fillRect(x, 12, w, 24);
        ctx.strokeStyle = col; ctx.strokeRect(x, 12, w, 24);
        ctx.fillStyle = col; ctx.fillText(text, x + 9, 28); return x + w + 8;
      };
      let x = 12;
      x = badge(x, auto ? "AUTOPILOT · " + (hud.name || "") : "MANUAL", auto ? "#38bdf8" : "#aab4c0");
      x = badge(x, mode, S.match && S.match.ok ? COL.ok : "#aab4c0");
      const bs = { clear: "BUBBLE CLEAR", limiting: "BUBBLE SLOWING", front_blocked: "FRONT BLOCKED",
        rear_blocked: "REAR BLOCKED", boxed_in: "BOXED IN", stale: "NO LIDAR" }[B.state] || "BUBBLE";
      badge(x, bs, { clear: COL.ok, limiting: COL.warn, stale: COL.mute }[B.state] || COL.bad);
      // distance readouts
      ctx.font = "600 28px Inter, Segoe UI, sans-serif"; ctx.fillStyle = "#e5e7eb"; ctx.textAlign = "right";
      const f = B.front_m;
      ctx.fillText(f === null || f === undefined ? "--" : f.toFixed(2), W - 16, 40);
      ctx.font = "600 11px Inter, Segoe UI, sans-serif"; ctx.fillStyle = "#9ca3af";
      ctx.fillText("M CLEAR AHEAD", W - 16, 56);
      if (hud && hud.state === "running") {
        ctx.font = "600 13px Inter, Segoe UI, sans-serif"; ctx.fillStyle = "#7dd3fc";
        ctx.fillText(`${(hud.v || 0).toFixed(2)} m/s · ${(hud.w || 0).toFixed(2)} rad/s`, W - 16, 76);
      }
      ctx.textAlign = "left";
    }

    function frame() {
      raf = 0;
      if (!shown) return;
      if (S && S.pose) {
        const p = S.pose;
        if (!pose) pose = p.slice();
        else {
          const k = 0.25, dth = Math.atan2(Math.sin(p[2] - pose[2]), Math.cos(p[2] - pose[2]));
          pose = [pose[0] + (p[0] - pose[0]) * k, pose[1] + (p[1] - pose[1]) * k, pose[2] + dth * k];
        }
        drawBackground();
        setupCamera();
        drawGrid();
        drawTrail();
        drawBubble();
        drawPlan();
        const items = [];
        collectColumns(items);
        // robot drawn in depth order with columns: split columns at the robot's depth
        const rd = proj(pose[0], pose[1], 0.1);
        const far = items.filter((it) => !rd || it.d >= rd[2]), near = items.filter((it) => rd && it.d < rd[2]);
        drawColumns(far);
        drawRobot();
        drawColumns(near);
        drawLabels();
        drawHud();
      } else {
        drawBackground();
        ctx.fillStyle = "#9ca3af"; ctx.font = "14px sans-serif"; ctx.fillText("Waiting for the robot pose…", 20, 30);
      }
      raf = requestAnimationFrame(frame);
    }
    function show(on) {
      shown = !!on;
      if (shown) { resize(); if (!raf) raf = requestAnimationFrame(frame); }
    }
    function setHud(h) { hud = h; }
    const api = { setScene, update, show, resize, setHud, version: -1 };
    return api;
  }
  window.DriveView = { create };
})();
