/* 2PyBot Drive tab: live camera with the lidar projected into it (distance
 * posts and tags, floor distance rings, safety bubble, planned path ribbon,
 * goal beacon, area labels, HUD). No WebGL, no assets.
 *
 * Frames: robot x forward, y left, z up (m). Map frame from SLAM (or wheel
 * odometry while SLAM is idle). Camera: optical X right, Y down, Z forward.
 * Intrinsics come from /api/calibration/active when present; otherwise HFOV.
 * Mount (height, offsets, tilt, pan) is set in the Align panel (localStorage).
 */
"use strict";
(function () {
  const D2R = Math.PI / 180, R2D = 180 / Math.PI;
  const LS = "2pybot.drive.camera.v1";
  const DEF = { hfov: 66, x: 0.045, y: -0.023, z: 0.195, tilt: 0, yaw: 0, roll: 0, lean: 1, useCalib: true, walls: false };
  const LIDAR_Z = 0.265;
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

  // distance colour: near red -> amber -> green -> cyan far
  function distColor(r, a) {
    const stops = [[0.25, [255, 60, 90]], [0.6, [255, 150, 60]], [1.2, [250, 220, 90]], [2.0, [90, 230, 160]], [3.5, [80, 200, 255]]];
    let c = stops[stops.length - 1][1];
    for (let i = 0; i < stops.length - 1; i++) {
      const [r0, c0] = stops[i], [r1, c1] = stops[i + 1];
      if (r <= r0) { c = c0; break; }
      if (r <= r1) { const t = (r - r0) / (r1 - r0); c = c0.map((v, k) => Math.round(v + (c1[k] - v) * t)); break; }
    }
    return `rgba(${c[0]},${c[1]},${c[2]},${a === undefined ? 1 : a})`;
  }

  function create(root) {
    // ------------------------------------------------------------------ DOM
    root.innerHTML = `
      <video class="dv-video" muted playsinline autoplay></video>
      <canvas class="dv-canvas"></canvas>
      <div class="dv-bar">
        <button class="btn small" data-act="video">Reconnect video</button>
        <button class="btn small" data-act="align">Align camera</button>
        <span class="dv-note"></span>
      </div>
      <div class="dv-align" hidden></div>`;
    const video = root.querySelector(".dv-video"), cv = root.querySelector(".dv-canvas"), ctx = cv.getContext("2d");
    const note = root.querySelector(".dv-note"), alignBox = root.querySelector(".dv-align");

    // ------------------------------------------------------------------ state
    let S = null, hud = null, shown = false, raf = 0;
    let W = 1, H = 1, dpr = 1;
    let cfg = Object.assign({}, DEF);
    try { Object.assign(cfg, JSON.parse(localStorage.getItem(LS) || "{}")); } catch (e) { /* defaults */ }
    let calib = null;
    const hist = [];                                   // [t, x, y, heading] for speed / yaw rate
    let walls = [];
    let videoState = "idle";

    fetch("/api/calibration/active").then((r) => r.json()).then((j) => {
      if (j && j.camera_matrix && j.image_size) calib = j;
      buildAlign();
    }).catch(() => buildAlign());

    // ------------------------------------------------------------------ video (WHEP, same as the console)
    let pc = null, session = null;
    function stopVideo() {
      if (pc) { pc.close(); pc = null; }
      video.srcObject = null;
      if (session) { fetch(session, { method: "DELETE" }).catch(() => {}); session = null; }
      videoState = "idle";
    }
    async function startVideo() {
      stopVideo();
      const peer = new RTCPeerConnection({ iceServers: [] }); pc = peer;
      const url = `${location.protocol}//${location.hostname}:8889/cam/whep`;
      peer.addTransceiver("video", { direction: "recvonly" });
      const remote = new MediaStream(); video.srcObject = remote;
      peer.ontrack = (e) => { remote.addTrack(e.track); video.play().catch(() => {}); };
      peer.onconnectionstatechange = () => {
        videoState = peer.connectionState;
        if (pc === peer && (videoState === "failed" || videoState === "disconnected")) setTimeout(() => { if (pc === peer && shown) startVideo(); }, 2000);
      };
      videoState = "connecting";
      try {
        await peer.setLocalDescription(await peer.createOffer());
        await new Promise((res) => {
          if (peer.iceGatheringState === "complete") return res();
          const t = setTimeout(res, 2000);
          peer.onicegatheringstatechange = () => { if (peer.iceGatheringState === "complete") { clearTimeout(t); res(); } };
        });
        if (pc !== peer) return;
        const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/sdp" }, body: peer.localDescription.sdp });
        if (!r.ok) throw new Error(`camera WHEP ${r.status}`);
        const loc = r.headers.get("Location"); if (loc) session = new URL(loc, url).href;
        await peer.setRemoteDescription({ type: "answer", sdp: await r.text() });
      } catch (e) {
        if (pc === peer) { videoState = "error: " + e.message; setTimeout(() => { if (pc === peer && shown) startVideo(); }, 4000); }
      }
    }
    const videoReady = () => video.readyState >= 2 && video.videoWidth > 0;

    // ------------------------------------------------------------------ pose helpers
    function robotPose() {
      if (!S) return null;
      if (S.mode === "idle" && S.odom) return [S.odom.x || 0, S.odom.y || 0, (S.odom.yaw_deg || 0) * D2R];
      return S.pose;
    }
    function motion() {
      if (hist.length < 2) return { v: 0, w: 0 };
      const a = hist[0], b = hist[hist.length - 1], dt = Math.max(1e-3, b[0] - a[0]);
      const v = Math.hypot(b[1] - a[1], b[2] - a[2]) / dt;
      const fwd = (b[1] - a[1]) * Math.cos(b[3]) + (b[2] - a[2]) * Math.sin(b[3]);
      return { v: fwd < 0 ? -v : v, w: wrap(b[3] - a[3]) / dt };
    }
    const toRobot = (P, x, y) => {                     // map frame -> robot frame
      const dx = x - P[0], dy = y - P[1], c = Math.cos(P[2]), s = Math.sin(P[2]);
      return [c * dx + s * dy, -s * dx + c * dy];
    };
    const toMap = (P, x, y) => {
      const c = Math.cos(P[2]), s = Math.sin(P[2]);
      return [P[0] + c * x - s * y, P[1] + s * x + c * y];
    };

    // ------------------------------------------------------------------ camera model
    function camModel() {
      const vw = videoReady() ? video.videoWidth : 1280, vh = videoReady() ? video.videoHeight : 720;
      let fx, fy, cx, cy, dist = null, fisheye = false;
      if (cfg.useCalib && calib) {
        const [iw, ih] = calib.image_size, K = calib.camera_matrix, sx = vw / iw, sy = vh / ih;
        fx = K[0][0] * sx; fy = K[1][1] * sy; cx = K[0][2] * sx; cy = K[1][2] * sy;
        dist = calib.distortion || null; fisheye = calib.model === "fisheye";
      } else {
        fx = (vw / 2) / Math.tan(cfg.hfov * D2R / 2); fy = fx; cx = vw / 2; cy = vh / 2;
      }
      const lean = S && S.attitude ? (S.attitude.dev || 0) : 0;
      const p = (cfg.tilt + cfg.lean * lean) * D2R, yw = cfg.yaw * D2R, rl = cfg.roll * D2R;
      const fwd = [Math.cos(yw) * Math.cos(p), Math.sin(yw) * Math.cos(p), -Math.sin(p)];
      const left0 = [-Math.sin(yw), Math.cos(yw), 0], up0 = cross(fwd, left0);
      const left = left0.map((v, i) => v * Math.cos(rl) + up0[i] * Math.sin(rl));
      const up = up0.map((v, i) => v * Math.cos(rl) - left0[i] * Math.sin(rl));
      const scale = Math.min(W / vw, H / vh);
      return { vw, vh, fx, fy, cx, cy, dist, fisheye, fwd, left, up, pos: [cfg.x, cfg.y, cfg.z],
               hfov: 2 * Math.atan(vw / 2 / fx), vfov: 2 * Math.atan(vh / 2 / fy),
               scale, ox: (W - vw * scale) / 2, oy: (H - vh * scale) / 2 };
    }
    // direction / offset in the robot frame -> video pixel [u, v, depth]
    function dirPix(M, d, minZ) {
      const Z = dot(d, M.fwd);
      if (Z < (minZ || 0.05)) return null;
      let xn = -dot(d, M.left) / Z, yn = -dot(d, M.up) / Z;
      const r2 = xn * xn + yn * yn;
      if (r2 > 9) return null;
      if (M.dist) {
        const k = M.dist;
        if (M.fisheye) {
          const r = Math.sqrt(r2), th = Math.atan(r), t2 = th * th;
          const thd = th * (1 + (k[0] || 0) * t2 + (k[1] || 0) * t2 * t2 + (k[2] || 0) * t2 * t2 * t2 + (k[3] || 0) * t2 * t2 * t2 * t2);
          const s = r > 1e-8 ? thd / r : 1; xn *= s; yn *= s;
        } else if (r2 < 2.5) {
          const [k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0] = k;
          const rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
          const x2 = xn * rad + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn);
          const y2 = yn * rad + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn;
          xn = x2; yn = y2;
        }
      }
      return [M.cx + M.fx * xn, M.cy + M.fy * yn, Z];
    }
    const pix = (M, x, y, z) => dirPix(M, [x - M.pos[0], y - M.pos[1], z - M.pos[2]], 0.06);
    const scr = (M, p) => p && [M.ox + p[0] * M.scale, M.oy + p[1] * M.scale, p[2]];
    const ptS = (M, x, y, z) => scr(M, pix(M, x, y, z));

    // ------------------------------------------------------------------ shared overlay pieces
    function tag(x, y, text, color, sub) {
      ctx.font = "600 12px Inter, Segoe UI, sans-serif";
      const w = Math.max(ctx.measureText(text).width, sub ? ctx.measureText(sub).width * 0.9 : 0) + 14, h = sub ? 32 : 20;
      ctx.fillStyle = "rgba(8,10,14,0.78)"; ctx.strokeStyle = color; ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.roundRect ? ctx.roundRect(x - w / 2, y - h, w, h, 6) : ctx.rect(x - w / 2, y - h, w, h); ctx.fill(); ctx.stroke();
      ctx.fillStyle = color; ctx.textAlign = "center"; ctx.fillText(text, x, y - (sub ? 18 : 6));
      if (sub) { ctx.font = "500 10.5px Inter, Segoe UI, sans-serif"; ctx.fillStyle = "#cbd5e1"; ctx.fillText(sub, x, y - 5); }
      ctx.textAlign = "left";
    }
    function polyFloor(M, pts) {                                    // robot-frame floor polygon
      const q = pts.map(([x, y]) => ptS(M, x, y, 0));
      if (q.some((p) => !p)) return false;
      ctx.beginPath(); q.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath();
      return true;
    }
    function ribbon(M, pts, width, fill, edge) {                     // robot-frame polyline on the floor
      const L = [], Rr = [];
      for (let i = 0; i < pts.length; i++) {
        const a = pts[Math.max(0, i - 1)], b = pts[Math.min(pts.length - 1, i + 1)];
        let tx = b[0] - a[0], ty = b[1] - a[1]; const n = Math.hypot(tx, ty) || 1; tx /= n; ty /= n;
        const l = ptS(M, pts[i][0] - ty * width, pts[i][1] + tx * width, 0.003);
        const r = ptS(M, pts[i][0] + ty * width, pts[i][1] - tx * width, 0.003);
        if (!l || !r) break;
        L.push(l); Rr.push(r);
      }
      if (L.length < 2) return;
      const g = ctx.createLinearGradient(0, L[0][1], 0, L[L.length - 1][1]);
      g.addColorStop(0, fill); g.addColorStop(1, fill.replace(/[\d.]+\)$/, "0.08)"));
      ctx.fillStyle = g; ctx.beginPath();
      L.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
      for (let i = Rr.length - 1; i >= 0; i--) ctx.lineTo(Rr[i][0], Rr[i][1]);
      ctx.closePath(); ctx.fill();
      if (edge) {
        ctx.strokeStyle = edge; ctx.lineWidth = 1.5;
        for (const side of [L, Rr]) { ctx.beginPath(); side.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.stroke(); }
      }
    }

    // ------------------------------------------------------------------ CAMERA mode
    function drawCamera() {
      const M = camModel(), P = robotPose(), live = videoReady();
      ctx.clearRect(0, 0, W, H);
      if (!live) {
        const g = ctx.createLinearGradient(0, 0, 0, H); g.addColorStop(0, "#0b1018"); g.addColorStop(1, "#05070a");
        ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);
        // synthetic horizon so the overlay still reads as a view
        const hz = ptS(M, 50, 0, cfg.z);
        if (hz) { ctx.strokeStyle = "rgba(148,163,184,0.15)"; ctx.beginPath(); ctx.moveTo(0, hz[1]); ctx.lineTo(W, hz[1]); ctx.stroke(); }
        ctx.fillStyle = "#94a3b8"; ctx.font = "13px Inter, sans-serif";
        ctx.fillText(`Camera ${videoState}. Lidar overlay shown on a synthetic view.`, 16, H - 40);
      } else {
        ctx.fillStyle = "rgba(0,0,0,0.12)"; ctx.fillRect(0, 0, W, H);   // slight dim so the overlay pops
      }
      if (!S || !P) return;
      ctx.save();
      ctx.beginPath(); ctx.rect(M.ox, M.oy, M.vw * M.scale, M.vh * M.scale); ctx.clip();
      // floor distance rings
      ctx.lineWidth = 1; ctx.font = "600 11px Inter, sans-serif";
      for (const r of [0.5, 1, 2, 3]) {
        const pts = []; for (let a = -80; a <= 80; a += 4) pts.push([r * Math.cos(a * D2R), r * Math.sin(a * D2R)]);
        const q = pts.map(([x, y]) => ptS(M, x, y, 0));
        ctx.strokeStyle = "rgba(226,232,240,0.22)"; ctx.setLineDash([5, 6]); ctx.beginPath();
        let on = false; for (const p of q) { if (!p) { on = false; continue; } on ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); on = true; }
        ctx.stroke(); ctx.setLineDash([]);
        const lab = ptS(M, r * Math.cos(-28 * D2R), r * Math.sin(-28 * D2R), 0);
        if (lab) { ctx.fillStyle = "rgba(226,232,240,0.55)"; ctx.fillText(`${r} m`, lab[0] + 4, lab[1] - 3); }
      }
      // safety bubble on the floor
      const B = S.bubble;
      if (B && B.footprint) {
        const [x0, x1, y0, y1] = B.footprint, mg = B.stop_margin || 0.06;
        const col = { clear: "52,211,153", limiting: "240,184,75", stale: "115,105,95" }[B.state] || "255,77,109";
        const reach = Math.min(1.6, (B.forward_cap || 0) ** 2 / 1.2 + (B.forward_cap || 0) * 0.25);
        if (reach > 0.02 && polyFloor(M, [[x1 + mg, y0], [x1 + mg + reach, y0], [x1 + mg + reach, y1], [x1 + mg, y1]])) {
          ctx.fillStyle = `rgba(${col},0.16)`; ctx.fill();
        }
        if (B.front_m !== null && B.front_m !== undefined && B.front_m < 3) {
          const f = x1 + B.front_m;
          const a = ptS(M, f, y0 - 0.03, 0), b = ptS(M, f, y1 + 0.03, 0);
          if (a && b) { ctx.strokeStyle = `rgba(${col},0.95)`; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
        }
      }
      // predicted course (grey) or planned path (blue), like a car autopilot ribbon
      const plan = (S.plan || []).map((p) => toRobot(P, p[0], p[1])).filter((p) => p[0] > 0.08);
      if (plan.length > 1) ribbon(M, [[0.08, 0]].concat(plan), 0.09, "rgba(56,189,248,0.55)", "rgba(125,211,252,0.9)");
      else {
        const m = motion(), pts = [];
        if (Math.abs(m.v) > 0.03) {
          let x = 0, y = 0, th = 0; const dt = 0.1, v = Math.max(0.15, Math.abs(m.v));
          for (let i = 0; i < 18; i++) { x += v * Math.cos(th) * dt * Math.sign(m.v || 1); y += v * Math.sin(th) * dt; th += m.w * dt; pts.push([x, y]); }
          if (m.v > 0) ribbon(M, [[0.08, 0]].concat(pts), 0.08, "rgba(203,213,225,0.28)", null);
        }
      }
      // goal beacon
      if (S.goal) {
        const [gx, gy] = toRobot(P, S.goal.x, S.goal.y), b = ptS(M, gx, gy, 0), t = ptS(M, gx, gy, 0.7);
        if (b && t) {
          ctx.strokeStyle = "rgba(125,211,252,0.95)"; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(b[0], b[1]); ctx.lineTo(t[0], t[1]); ctx.stroke();
          tag(t[0], t[1] - 4, S.goal.name || "goal", "#7dd3fc", `${Math.hypot(gx, gy).toFixed(1)} m`);
        }
      }
      // map walls (optional): faint columns of the saved map
      if (cfg.walls) {
        ctx.strokeStyle = "rgba(186,198,214,0.22)";
        for (const [wx, wy] of walls) {
          const [x, y] = toRobot(P, wx, wy); if (x < 0.2 || Math.hypot(x, y) > 5) continue;
          const b = ptS(M, x, y, 0), t = ptS(M, x, y, LIDAR_Z); if (!b || !t) continue;
          ctx.lineWidth = clamp(14 / b[2], 1, 6); ctx.beginPath(); ctx.moveTo(b[0], b[1]); ctx.lineTo(t[0], t[1]); ctx.stroke();
        }
      }
      // ToF obstacles / drop-offs (map frame)
      const T = S.tof || {};
      for (const [list, colr, txt] of [[T.obstacles, "#fb923c", null], [T.cliffs, "#ff4d6d", "DROP"]]) {
        for (const o of list || []) {
          const [x, y] = toRobot(P, o[0], o[1]), z = o.length > 2 && txt === null ? Math.max(0, o[2]) : 0;
          const p = ptS(M, x, y, z); if (!p) continue;
          ctx.fillStyle = colr; ctx.fillRect(p[0] - 3, p[1] - 3, 6, 6);
          if (txt) { ctx.fillStyle = colr; ctx.font = "700 10px Inter, sans-serif"; ctx.fillText(txt, p[0] + 5, p[1]); }
        }
      }
      // lidar returns as posts with glow, far first
      const items = [];
      for (const p of S.scan || []) {
        const b = ptS(M, p[0], p[1], 0), t = ptS(M, p[0], p[1], LIDAR_Z);
        if (!b || !t) continue;
        items.push({ b, t, r: Math.hypot(p[0], p[1]), p });
      }
      items.sort((a, c) => c.b[2] - a.b[2]);
      for (const it of items) {
        const w = clamp(22 / it.b[2], 1.2, 7);
        ctx.strokeStyle = distColor(it.r, 0.28); ctx.lineWidth = w;
        ctx.beginPath(); ctx.moveTo(it.b[0], it.b[1]); ctx.lineTo(it.t[0], it.t[1]); ctx.stroke();
        ctx.fillStyle = distColor(it.r, 1); ctx.shadowColor = distColor(it.r, 1); ctx.shadowBlur = 10;
        ctx.fillRect(it.t[0] - w * 0.45, it.t[1] - w * 0.45, w * 0.9, w * 0.9); ctx.shadowBlur = 0;
      }
      // distance tags on the nearest returns per 8 degree sector
      const sectors = new Map();
      for (const it of items) {
        const k = Math.floor((Math.atan2(it.p[1], it.p[0]) * R2D + 360) / 8);
        const s = sectors.get(k); if (!s || it.r < s.r) sectors.set(k, it);
      }
      const chosen = [];
      for (const it of [...sectors.values()].filter((s) => s.r < 4).sort((a, c) => a.r - c.r)) {
        if (chosen.some((c) => Math.abs(c.t[0] - it.t[0]) < 78 && Math.abs(c.t[1] - it.t[1]) < 36)) continue;
        chosen.push(it); if (chosen.length >= 8) break;
      }
      for (const it of chosen) tag(it.t[0], it.t[1] - 6, `${it.r.toFixed(2)} m`, distColor(it.r, 1));
      // area labels
      for (const l of S.labels || []) {
        const [x, y] = toRobot(P, l.x, l.y), d = Math.hypot(x, y);
        const b = ptS(M, x, y, 0), t = ptS(M, x, y, 0.45);
        if (!b || !t || d > 15) continue;
        ctx.strokeStyle = "rgba(240,138,75,0.8)"; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(b[0], b[1]); ctx.lineTo(t[0], t[1]); ctx.stroke();
        tag(t[0], t[1], l.name, "#f08a4b", `${d.toFixed(1)} m`);
      }
      ctx.restore();
      drawHud(M, P);
    }

    // ------------------------------------------------------------------ HUD (car-autopilot style)
    function drawHud(M, P) {
      const m = motion(), B = (S && S.bubble) || {};
      const auto = hud && hud.state === "running";
      ctx.fillStyle = "rgba(5,7,10,0.55)"; ctx.fillRect(10, 54, 210, 96);
      ctx.fillStyle = "#f1f5f9"; ctx.font = "700 40px Inter, Segoe UI, sans-serif";
      const speed = Math.abs(m.v).toFixed(2), sw = ctx.measureText(speed).width;
      ctx.fillText(speed, 22, 100);
      ctx.font = "600 12px Inter, sans-serif"; ctx.fillStyle = "#94a3b8"; ctx.fillText("m/s", 28 + sw, 100);
      ctx.font = "700 13px Inter, sans-serif"; ctx.fillStyle = auto ? "#38bdf8" : "#cbd5e1";
      ctx.fillText(auto ? "Self-driving · " + (hud.name || "") : "Manual", 22, 122);
      ctx.font = "500 11px Inter, sans-serif"; ctx.fillStyle = "#94a3b8";
      ctx.fillText(`${{ idle: "lidar only", mapping: "mapping", localizing: "on map" }[S.mode] || S.mode} · ${(m.w * R2D).toFixed(0)}°/s`, 22, 140);
      const bs = { clear: ["clear", "#34d399"], limiting: ["slowing", "#f0b84b"], front_blocked: ["front blocked", "#ff4d6d"],
        rear_blocked: ["rear blocked", "#ff4d6d"], boxed_in: ["boxed in", "#ff4d6d"], stale: ["no lidar", "#73695f"] }[B.state] || ["–", "#94a3b8"];
      ctx.textAlign = "right"; ctx.font = "700 30px Inter, sans-serif"; ctx.fillStyle = "#f1f5f9";
      ctx.fillText(B.front_m === null || B.front_m === undefined ? "--" : B.front_m.toFixed(2), W - 18, 92);
      ctx.font = "600 11px Inter, sans-serif"; ctx.fillStyle = "#94a3b8"; ctx.fillText("m clear ahead", W - 18, 108);
      ctx.fillStyle = bs[1]; ctx.font = "700 12px Inter, sans-serif"; ctx.fillText("bubble " + bs[0], W - 18, 126);
      if (auto) { ctx.fillStyle = "#7dd3fc"; ctx.fillText(`cmd ${(hud.v || 0).toFixed(2)} m/s · ${(hud.w || 0).toFixed(2)} rad/s`, W - 18, 144); }
      ctx.textAlign = "left";
      if (!calib || !cfg.useCalib) {
        ctx.fillStyle = "rgba(240,184,75,0.85)"; ctx.font = "11px Inter, sans-serif";
        ctx.fillText(`Uncalibrated camera (HFOV ${cfg.hfov}°): use Align camera so the posts sit on real walls`, 14, H - 16);
      }
    }

    // ------------------------------------------------------------------ align panel
    function buildAlign() {
      const cal = !!calib;
      const rows = [
        ["hfov", "Horizontal FOV", 40, 130, 1, "°"], ["z", "Camera height", 0.08, 0.40, 0.005, " m"],
        ["x", "Forward of axle", -0.08, 0.15, 0.005, " m"], ["y", "Left of centre", -0.10, 0.10, 0.005, " m"],
        ["tilt", "Tilt down", -25, 35, 0.5, "°"], ["yaw", "Pan (left +)", -60, 60, 0.5, "°"], ["roll", "Roll", -15, 15, 0.5, "°"],
      ];
      alignBox.innerHTML = `<b>Align camera</b><p class="sub">Drive or hold the robot near a wall. Adjust until the lidar posts sit on the real wall bottoms. ${cal ? "Lens calibration found." : "No lens calibration: FOV is a guess."}</p>` +
        rows.map(([k, l, a, b, st, u]) => `<label><span>${l}</span><input type="range" min="${a}" max="${b}" step="${st}" data-k="${k}" value="${cfg[k]}"${k === "hfov" && cal && cfg.useCalib ? " disabled" : ""}><em data-v="${k}">${cfg[k]}${u}</em></label>`).join("") +
        `<label class="chk"><input type="checkbox" data-c="useCalib"${cfg.useCalib ? " checked" : ""}${cal ? "" : " disabled"}> Use lens calibration</label>` +
        `<label class="chk"><input type="checkbox" data-c="lean"${cfg.lean > 0 ? " checked" : ""}> Camera pitches with the robot lean</label>` +
        `<label class="chk"><input type="checkbox" data-c="walls"${cfg.walls ? " checked" : ""}> Show saved map walls in the camera</label>` +
        `<div class="row"><button class="btn small" data-a="reset">Reset</button><button class="btn small" data-a="close">Done</button></div>`;
      alignBox.querySelectorAll("input[type=range]").forEach((el) => (el.oninput = () => {
        cfg[el.dataset.k] = +el.value; alignBox.querySelector(`[data-v="${el.dataset.k}"]`).textContent = el.value + (rows.find((r) => r[0] === el.dataset.k)[5]);
        save();
      }));
      alignBox.querySelectorAll("input[type=checkbox]").forEach((el) => (el.onchange = () => {
        const k = el.dataset.c; cfg[k] = k === "lean" ? (el.checked ? 1 : 0) : el.checked; save(); if (k === "useCalib") buildAlign();
      }));
      alignBox.querySelector('[data-a="reset"]').onclick = () => { cfg = Object.assign({}, DEF); save(); buildAlign(); };
      alignBox.querySelector('[data-a="close"]').onclick = () => { alignBox.hidden = true; };
    }
    function save() { try { localStorage.setItem(LS, JSON.stringify(cfg)); } catch (e) { /* private mode */ } }

    // ------------------------------------------------------------------ input
    root.querySelector('[data-act="video"]').onclick = () => startVideo();
    root.querySelector('[data-act="align"]').onclick = () => { alignBox.hidden = !alignBox.hidden; };

    // ------------------------------------------------------------------ loop
    function resize() {
      const r = root.getBoundingClientRect(); dpr = window.devicePixelRatio || 1;
      W = Math.max(1, r.width); H = Math.max(1, r.height);
      cv.width = W * dpr; cv.height = H * dpr; ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    window.addEventListener("resize", () => { if (shown) resize(); });
    function frameLoop(now) {
      raf = 0; if (!shown) return;
      try {
        drawCamera();
      } catch (e) { note.textContent = "Drive view error: " + e.message; }
      raf = requestAnimationFrame(frameLoop);
    }
    function show(on) {
      shown = !!on;
      if (shown) { resize(); if (!pc) startVideo(); if (!raf) raf = requestAnimationFrame(frameLoop); }
      else stopVideo();
    }
    function update(state) {
      S = state;
      const P = robotPose();
      if (P) { hist.push([performance.now() / 1000, P[0], P[1], P[2]]); while (hist.length > 2 && hist[hist.length - 1][0] - hist[0][0] > 1.0) hist.shift(); }
    }
    function setScene(sc) {
      const k = sc.grid_res, o = sc.grid_origin, pool = (list) => {
        const seen = new Set(), out = [];
        for (const [c, r] of list || []) {
          const x = Math.round((o + (c + 0.5) * k) * 10) / 10, y = Math.round((o + (r + 0.5) * k) * 10) / 10, key = x + "," + y;
          if (!seen.has(key)) { seen.add(key); out.push([x, y]); }
        }
        return out;
      };
      walls = pool(sc.walls); api.version = sc.version;
    }
    function setHud(h) { hud = h; }
    const api = { show, update, setScene, setHud, resize, version: -1 };
    return api;
  }
  window.DriveView = { create };
})();
