/* 2PyBot Drive tab: camera-based reality view, no WebGL, no assets.
 *
 *   Camera    live WebRTC video with the lidar projected into it: distance posts
 *             and tags on every return, floor distance rings, the safety bubble,
 *             the planned path as a blue ribbon, goal beacon, area labels, HUD.
 *   Panorama  frames stitched by robot heading into a 360 degree cylinder; look
 *             around with drag; the 360 lidar is drawn into it everywhere.
 *   World     the lidar map rebuilt in 3D and coloured with real camera pixels
 *             (walls and floor), chase camera behind the robot.
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
  const PPD = 8;                                     // panorama pixels per degree
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const esc = (s) => String(s).replace(/[&<>"']/g, (c) => `&#${c.charCodeAt(0)};`);

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
        <div class="seg dv-modes" role="tablist">
          <button data-dmode="camera" aria-selected="true">Camera</button>
          <button data-dmode="pano" aria-selected="false">Panorama</button>
          <button data-dmode="world" aria-selected="false">World</button>
        </div>
        <button class="btn small" data-act="video">Reconnect video</button>
        <button class="btn small" data-act="align">Align camera</button>
        <button class="btn small" data-act="clear" title="Forget the stitched panorama and camera colours">Clear</button>
        <button class="btn small" data-act="save" title="Download the 360° panorama">Save 360°</button>
        <span class="dv-note"></span>
      </div>
      <div class="dv-align" hidden></div>`;
    const video = root.querySelector(".dv-video"), cv = root.querySelector(".dv-canvas"), ctx = cv.getContext("2d");
    const note = root.querySelector(".dv-note"), alignBox = root.querySelector(".dv-align");

    // ------------------------------------------------------------------ state
    let S = null, hud = null, shown = false, raf = 0, mode = "camera";
    let W = 1, H = 1, dpr = 1;
    let cfg = Object.assign({}, DEF);
    try { Object.assign(cfg, JSON.parse(localStorage.getItem(LS) || "{}")); } catch (e) { /* defaults */ }
    let calib = null;
    const hist = [];                                   // [t, x, y, heading] for speed / yaw rate
    let walls = [], floor = [], sceneVersion = -1;
    const wallColor = new Map(), floorColor = new Map();
    let pano = null;
    const frame = document.createElement("canvas"), fctx = frame.getContext("2d", { willReadFrequently: true });
    const sample = document.createElement("canvas"), sctx = sample.getContext("2d", { willReadFrequently: true });
    let videoState = "idle", lastCapture = 0;
    const view = { yaw: 0, follow: true, hfov: 95, pitch: 0, wcam: { yaw: 0, dist: 1.6, height: 0.9 } };

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

    // ------------------------------------------------------------------ panorama + colour capture
    function ensurePano(M) {
      const half = M.vfov * R2D / 2 + Math.abs(cfg.tilt) + 8;
      const e1 = Math.min(70, half), e0 = -Math.min(70, half);
      if (pano && Math.abs(pano.e1 - e1) < 3) return pano;
      const c = document.createElement("canvas");
      c.width = 360 * PPD; c.height = Math.round((e1 - e0) * PPD);
      pano = { cv: c, ctx: c.getContext("2d"), e0, e1, W: c.width, H: c.height, filled: new Uint8Array(360) };
      return pano;
    }
    const azToX = (phi) => ((((-phi * R2D) % 360) + 360) % 360) * PPD;     // clockwise to the right
    const xToAz = (x) => -(x / PPD) * D2R;
    const eToY = (P, e) => (P.e1 - e * R2D) * PPD;

    function capture(now) {
      if (!videoReady() || !S || now - lastCapture < 180) return;
      const P = robotPose(); if (!P) return;
      const m = motion();
      if (Math.abs(m.w) > 25 * D2R) return;                        // pose lags video: no stitching while spinning
      lastCapture = now;
      const M = camModel();
      const FW = 640, FH = Math.round(640 * M.vh / M.vw);
      if (frame.width !== FW || frame.height !== FH) { frame.width = FW; frame.height = FH; }
      fctx.drawImage(video, 0, 0, FW, FH);
      stitch(M, P, FW / M.vw);
      colourise(M, P);
    }

    function stitch(M, P, k) {
      const PN = ensurePano(M), c = PN.ctx;
      const heading = P[2], camAz = heading + cfg.yaw * D2R, half = M.hfov / 2 * 0.92;
      const x0 = Math.floor(azToX(camAz + half)), span = Math.ceil(2 * half * R2D * PPD);
      const segs = 6, de = (PN.e1 - PN.e0) / segs;
      for (let i = 0; i < span; i += 2) {
        const x = (x0 + i) % PN.W;
        const phi = xToAz(x + 1), beta = wrap(phi - heading), rel = Math.abs(wrap(phi - camAz));
        if (rel > half) continue;
        c.globalAlpha = 0.25 + 0.6 * Math.cos(rel / half * Math.PI / 2);
        for (let s = 0; s < segs; s++) {
          const eA = (PN.e1 - s * de) * D2R, eB = (PN.e1 - (s + 1) * de) * D2R;
          const dA = [Math.cos(eA) * Math.cos(beta), Math.cos(eA) * Math.sin(beta), Math.sin(eA)];
          const dB = [Math.cos(eB) * Math.cos(beta), Math.cos(eB) * Math.sin(beta), Math.sin(eB)];
          const a = dirPix(M, dA), b = dirPix(M, dB);
          if (!a || !b) continue;
          let vA = a[1] * k, vB = b[1] * k, yA = eToY(PN, eA * R2D), yB = eToY(PN, eB * R2D);
          const u = 0.5 * (a[0] + b[0]) * k;
          if (u < 0 || u >= frame.width || vB <= 0 || vA >= frame.height || vB - vA < 0.5) continue;
          if (vA < 0) { yA += (0 - vA) / (vB - vA) * (yB - yA); vA = 0; }
          if (vB > frame.height) { yB -= (vB - frame.height) / (vB - vA) * (yB - yA); vB = frame.height; }
          c.drawImage(frame, u, vA, 1, vB - vA, x, yA, 2, yB - yA);
        }
        PN.filled[Math.floor(x / PPD) % 360] = 1;
      }
      c.globalAlpha = 1;
    }

    function rangeByBearing(scan) {                                // min lidar range per 2 deg bucket (robot frame)
      const r = new Float32Array(180).fill(99);
      for (const p of scan || []) {
        const b = Math.floor(((Math.atan2(p[1], p[0]) * R2D + 360) % 360) / 2);
        const d = Math.hypot(p[0], p[1]);
        if (d < r[b]) r[b] = d;
      }
      return r;
    }
    const rangeAt = (R, x, y) => R[Math.floor(((Math.atan2(y, x) * R2D + 360) % 360) / 2)];

    function colourise(M, P) {
      if (!walls.length && !floor.length) return;
      const SW = 192, SH = Math.round(192 * M.vh / M.vw);
      if (sample.width !== SW || sample.height !== SH) { sample.width = SW; sample.height = SH; }
      sctx.drawImage(frame, 0, 0, SW, SH);
      const img = sctx.getImageData(0, 0, SW, SH).data, k = SW / M.vw;
      const R = rangeByBearing(S.scan);
      const get = (p) => {
        if (!p) return null;
        const u = Math.round(p[0] * k), v = Math.round(p[1] * k);
        if (u < 1 || v < 1 || u >= SW - 1 || v >= SH - 1) return null;
        const i = (v * SW + u) * 4; return [img[i], img[i + 1], img[i + 2]];
      };
      const mixIn = (map, key, idx, n, rgb) => {
        let e = map.get(key); if (!e) { e = new Float32Array(n * 4); map.set(key, e); }
        const o = idx * 4, w = e[o + 3] ? 0.35 : 1;
        e[o] += (rgb[0] - e[o]) * w; e[o + 1] += (rgb[1] - e[o + 1]) * w; e[o + 2] += (rgb[2] - e[o + 2]) * w; e[o + 3] = 1;
      };
      for (const [wx, wy] of walls) {
        const [x, y] = toRobot(P, wx, wy), d = Math.hypot(x, y);
        if (x < 0.15 || d > 4.5) continue;
        if (d > rangeAt(R, x, y) + 0.25) continue;                    // hidden behind something nearer
        [0.05, 0.15, 0.25, 0.42].forEach((z, i) => { const c = get(pix(M, x, y, z)); if (c) mixIn(wallColor, wx + "," + wy, i, 4, c); });
      }
      for (const [fx_, fy_] of floor) {
        const [x, y] = toRobot(P, fx_, fy_), d = Math.hypot(x, y);
        if (x < 0.12 || d > 3.2) continue;
        if (d > rangeAt(R, x, y) - 0.05) continue;
        const c = get(pix(M, x, y, 0)); if (c) mixIn(floorColor, fx_ + "," + fy_, 0, 1, c);
      }
    }

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
        ctx.fillText(`Camera ${videoState}. Lidar overlay shown on a synthetic view.`, 16, H - 110);
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
      drawStrip(M, P);
    }

    // ------------------------------------------------------------------ PANORAMA mode
    function drawPano() {
      const P = robotPose(), M = camModel();
      const g = ctx.createLinearGradient(0, 0, 0, H); g.addColorStop(0, "#0c121a"); g.addColorStop(1, "#05070a");
      ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);
      if (!S || !P) return;
      const PN = ensurePano(M);
      const heading = P[2], f = (W / 2) / Math.tan(view.hfov * D2R / 2);
      const psi = view.follow ? heading + cfg.yaw * D2R : view.yaw;
      if (view.follow) view.yaw = psi;
      const cy = H / 2 + view.pitch;
      const yOf = (e, a) => cy - f * Math.tan(e) / Math.cos(a);
      // horizon grid so unseen directions still read as space
      ctx.strokeStyle = "rgba(148,163,184,0.10)"; ctx.lineWidth = 1;
      for (let e = -30; e <= 30; e += 10) { ctx.beginPath(); for (let sx = 0; sx <= W; sx += 12) { const a = Math.atan((W / 2 - sx) / f); const y = yOf(e * D2R, a); sx ? ctx.lineTo(sx, y) : ctx.moveTo(sx, y); } ctx.stroke(); }
      // cylinder columns
      const segs = 4, de = (PN.e1 - PN.e0) / segs;
      for (let sx = 0; sx < W; sx += 3) {
        const a = Math.atan((W / 2 - (sx + 1.5)) / f), x = Math.floor(azToX(psi + a)) % PN.W;
        for (let s = 0; s < segs; s++) {
          const eA = PN.e1 - s * de, eB = eA - de, yA = yOf(eA * D2R, a), yB = yOf(eB * D2R, a);
          ctx.drawImage(PN.cv, x, eToY(PN, eA), 1, de * PPD, sx, yA, 3, yB - yA);
        }
      }
      // compass ticks
      ctx.fillStyle = "rgba(226,232,240,0.7)"; ctx.font = "600 11px Inter, sans-serif"; ctx.textAlign = "center";
      for (let d = 0; d < 360; d += 15) {
        const a = wrap(-d * D2R - psi); if (Math.abs(a) > view.hfov * D2R / 2) continue;
        const sx = W / 2 - f * Math.tan(a);
        ctx.fillRect(sx, 46, 1, d % 45 ? 5 : 10);
        if (d % 45 === 0) ctx.fillText(`${d}°`, sx, 70);
      }
      ctx.textAlign = "left";
      // lidar in the panorama: posts + tags, all around
      const items = [];
      for (const p of S.scan || []) {
        const dx = p[0] - cfg.x, dy = p[1] - cfg.y, h = Math.hypot(dx, dy);
        if (h < 0.05) continue;
        const a = wrap(heading + Math.atan2(dy, dx) - psi);
        if (Math.abs(a) > view.hfov * D2R / 2 + 0.05 || Math.abs(a) > 1.45) continue;
        const sx = W / 2 - f * Math.tan(a);
        const yt = yOf(Math.atan2(LIDAR_Z - cfg.z, h), a), yb = yOf(Math.atan2(-cfg.z, h), a);
        items.push({ sx, yt, yb, r: Math.hypot(p[0], p[1]), h });
      }
      items.sort((a, b) => b.h - a.h);
      for (const it of items) {
        const w = clamp(14 / it.h, 1, 6);
        ctx.strokeStyle = distColor(it.r, 0.3); ctx.lineWidth = w; ctx.beginPath(); ctx.moveTo(it.sx, it.yb); ctx.lineTo(it.sx, it.yt); ctx.stroke();
        ctx.fillStyle = distColor(it.r, 1); ctx.fillRect(it.sx - w / 2, it.yt - w / 2, w, w);
      }
      const chosen = [];
      for (const it of items.slice().sort((a, b) => a.r - b.r)) {
        if (it.r > 5 || chosen.some((c) => Math.abs(c.sx - it.sx) < 80)) continue;
        chosen.push(it); if (chosen.length >= 9) break;
      }
      for (const it of chosen) tag(it.sx, it.yt - 6, `${it.r.toFixed(2)} m`, distColor(it.r, 1));
      // labels
      for (const l of S.labels || []) {
        const [x, y] = toRobot(P, l.x, l.y), a = wrap(heading + Math.atan2(y - cfg.y, x - cfg.x) - psi);
        if (Math.abs(a) > view.hfov * D2R / 2) continue;
        const sx = W / 2 - f * Math.tan(a), h = Math.hypot(x, y);
        tag(sx, yOf(Math.atan2(0.45 - cfg.z, h), a), l.name, "#f08a4b", `${h.toFixed(1)} m`);
      }
      // live camera window outline
      const ca = wrap(heading + cfg.yaw * D2R - psi), hh = M.hfov / 2;
      if (Math.abs(ca) < view.hfov * D2R) {
        const l = W / 2 - f * Math.tan(clamp(ca + hh, -1.5, 1.5)), r = W / 2 - f * Math.tan(clamp(ca - hh, -1.5, 1.5));
        ctx.strokeStyle = "rgba(56,189,248,0.6)"; ctx.setLineDash([6, 5]); ctx.strokeRect(l, yOf(M.vfov / 2, 0), r - l, yOf(-M.vfov / 2, 0) - yOf(M.vfov / 2, 0)); ctx.setLineDash([]);
      }
      const seen = PN.filled.reduce((a, b) => a + b, 0);
      ctx.fillStyle = "#cbd5e1"; ctx.font = "12px Inter, sans-serif";
      ctx.fillText(`Panorama ${seen}° of 360° captured · turn the robot slowly to fill it · drag to look around · double-click to follow`, 16, H - 104);
      drawHud(M, P);
      drawStrip(M, P);
    }

    // 360 strip: the whole panorama centred on the robot heading + lidar ring underneath
    function drawStrip(M, P) {
      const PN = pano; const x0 = 14, w = W - 28, h = Math.min(64, Math.max(34, w / 18)), y = H - h - 30;
      ctx.fillStyle = "rgba(5,7,10,0.72)"; ctx.fillRect(x0 - 2, y - 2, w + 4, h + 22);
      if (PN) {
        const centre = azToX(P[2]), shift = ((centre - PN.W / 2) % PN.W + PN.W) % PN.W, k = w / PN.W;
        ctx.drawImage(PN.cv, shift, 0, PN.W - shift, PN.H, x0, y, (PN.W - shift) * k, h);
        if (shift > 0) ctx.drawImage(PN.cv, 0, 0, shift, PN.H, x0 + (PN.W - shift) * k, y, shift * k, h);
      }
      // lidar distance ring (robot-relative bearing, centre = straight ahead)
      const R = rangeByBearing(S.scan);
      for (let i = 0; i < 180; i++) {
        const bearing = i * 2, rel = ((-bearing + 180) % 360 + 360) % 360;      // left of centre = left of robot
        const sx = x0 + rel / 360 * w;
        ctx.fillStyle = R[i] < 90 ? distColor(R[i], 0.95) : "rgba(71,85,105,0.5)";
        ctx.fillRect(sx, y + h + 4, w / 180 + 0.6, 6);
      }
      const fw = M.hfov * R2D / 360 * w;
      ctx.strokeStyle = "#38bdf8"; ctx.lineWidth = 1.5; ctx.strokeRect(x0 + w / 2 - fw / 2 - (cfg.yaw / 360) * w, y, fw, h);
      ctx.fillStyle = "#f08a4b"; ctx.beginPath(); ctx.moveTo(x0 + w / 2, y + h + 2); ctx.lineTo(x0 + w / 2 - 5, y + h + 12); ctx.lineTo(x0 + w / 2 + 5, y + h + 12); ctx.fill();
      ctx.fillStyle = "#94a3b8"; ctx.font = "10px Inter, sans-serif";
      ctx.fillText("BEHIND", x0 + 4, y + h + 18); ctx.fillText("AHEAD", x0 + w / 2 + 8, y + h + 18);
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
        ctx.fillText(`Uncalibrated camera (HFOV ${cfg.hfov}°): use Align camera so the posts sit on real walls`, 14, H - 112 - (mode === "pano" ? 12 : 0));
      }
    }

    // ------------------------------------------------------------------ WORLD mode (camera-coloured map)
    let C = null;
    function wsetup(P) {
      const th = P[2] + view.wcam.yaw, fx = Math.cos(th), fy = Math.sin(th);
      const eye = [P[0] - fx * view.wcam.dist, P[1] - fy * view.wcam.dist, view.wcam.height];
      const look = [P[0] + fx * 1.0, P[1] + fy * 1.0, 0.12];
      let f = [look[0] - eye[0], look[1] - eye[1], look[2] - eye[2]]; const n = Math.hypot(...f); f = f.map((v) => v / n);
      let r = [f[1], -f[0], 0]; const rn = Math.hypot(...r) || 1; r = r.map((v) => v / rn);
      C = { eye, f, r, u: cross(r, f), focal: (H / 2) / Math.tan(32 * D2R) };
    }
    function wp(x, y, z) {
      const dx = x - C.eye[0], dy = y - C.eye[1], dz = z - C.eye[2], d = dx * C.f[0] + dy * C.f[1] + dz * C.f[2];
      if (d < 0.08) return null;
      return [W / 2 + C.focal * (dx * C.r[0] + dy * C.r[1] + dz * C.r[2]) / d, H / 2 - C.focal * (dx * C.u[0] + dy * C.u[1] + dz * C.u[2]) / d, d];
    }
    function wpoly(pts) {
      const q = pts.map((p) => wp(p[0], p[1], p[2] || 0)); if (q.some((x) => !x)) return null;
      ctx.beginPath(); q.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath(); return q;
    }
    let wpose = null;
    function drawWorld() {
      const P0 = robotPose();
      const g = ctx.createLinearGradient(0, 0, 0, H); g.addColorStop(0, "#1a2433"); g.addColorStop(0.45, "#0b1018"); g.addColorStop(1, "#06080b");
      ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);
      if (!S || !P0) return;
      if (!wpose) wpose = P0.slice();
      else { const k = 0.25; wpose = [wpose[0] + (P0[0] - wpose[0]) * k, wpose[1] + (P0[1] - wpose[1]) * k, wpose[2] + wrap(P0[2] - wpose[2]) * k]; }
      const P = wpose; wsetup(P);
      // floor: camera-coloured tiles (10 cm), unseen known floor dark slate
      const half = 0.05;
      for (const [x, y] of floor) {
        const dx = x - P[0], dy = y - P[1]; if (dx * dx + dy * dy > 49) continue;
        const c = floorColor.get(x + "," + y);
        if (!wpoly([[x - half, y - half], [x + half, y - half], [x + half, y + half], [x - half, y + half]])) continue;
        ctx.fillStyle = c ? `rgb(${c[0] | 0},${c[1] | 0},${c[2] | 0})` : "rgba(40,50,64,0.55)"; ctx.fill();
      }
      // grid
      ctx.strokeStyle = "rgba(148,163,184,0.08)"; ctx.lineWidth = 1;
      const gx = Math.round(P[0]), gy = Math.round(P[1]);
      for (let i = -7; i <= 7; i++) {
        let a = wp(gx + i, gy - 7, 0), b = wp(gx + i, gy + 7, 0); if (a && b) { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
        a = wp(gx - 7, gy + i, 0); b = wp(gx + 7, gy + i, 0); if (a && b) { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
      }
      // path ribbon (blue)
      const plan = S.plan || [];
      if (plan.length > 1) {
        const L = [], Rr = [];
        for (let i = 0; i < plan.length; i++) {
          const a = plan[Math.max(0, i - 1)], b = plan[Math.min(plan.length - 1, i + 1)];
          let tx = b[0] - a[0], ty = b[1] - a[1]; const n = Math.hypot(tx, ty) || 1; tx /= n; ty /= n;
          L.push([plan[i][0] - ty * 0.09, plan[i][1] + tx * 0.09, 0.01]); Rr.push([plan[i][0] + ty * 0.09, plan[i][1] - tx * 0.09, 0.01]);
        }
        if (wpoly(L.concat(Rr.reverse()))) { ctx.fillStyle = "rgba(56,189,248,0.55)"; ctx.fill(); }
      }
      // collect columns: walls (camera colours), live lidar
      const items = [], c = Math.cos(P0[2]), s = Math.sin(P0[2]);
      for (const [x, y] of walls) {
        const dx = x - P[0], dy = y - P[1]; if (dx * dx + dy * dy > 64) continue;
        const b = wp(x, y, 0); if (!b) continue;
        items.push({ d: b[2], x, y, kind: 0 });
      }
      for (const p of S.scan || []) {
        const x = P0[0] + c * p[0] - s * p[1], y = P0[1] + s * p[0] + c * p[1], b = wp(x, y, 0);
        if (b) items.push({ d: b[2], x, y, kind: 1, r: Math.hypot(p[0], p[1]) });
      }
      const rd = wp(P[0], P[1], 0.1);
      items.sort((a, b) => b.d - a.d);
      let robotDrawn = false;
      for (const it of items) {
        if (!robotDrawn && rd && it.d < rd[2]) { drawRobot(P); robotDrawn = true; }
        if (it.kind === 0) {
          const col = wallColor.get(it.x + "," + it.y), zs = [0, 0.1, 0.2, 0.32, 0.5];
          const w = clamp(60 / it.d, 1.5, 14);
          for (let i = 0; i < 4; i++) {
            const a = wp(it.x, it.y, zs[i]), b = wp(it.x, it.y, zs[i + 1]); if (!a || !b) continue;
            const fade = clamp(1.3 - it.d / 7, 0.25, 1);
            ctx.strokeStyle = col && col[i * 4 + 3] ? `rgba(${col[i * 4] | 0},${col[i * 4 + 1] | 0},${col[i * 4 + 2] | 0},${fade})` : `rgba(150,162,178,${0.45 * fade})`;
            ctx.lineWidth = w; ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1] - 0.5); ctx.stroke();
          }
        } else {
          const t = wp(it.x, it.y, LIDAR_Z); if (!t) continue;
          const w = clamp(30 / it.d, 1.5, 7);
          ctx.fillStyle = distColor(it.r, 1); ctx.shadowColor = distColor(it.r, 1); ctx.shadowBlur = 8;
          ctx.fillRect(t[0] - w / 2, t[1] - w / 2, w, w); ctx.shadowBlur = 0;
        }
      }
      if (!robotDrawn) drawRobot(P);
      // bubble footprint
      const B = S.bubble;
      if (B && B.footprint) {
        const [x0, x1, y0, y1] = B.footprint, mg = B.stop_margin || 0.06, cc = Math.cos(P[2]), ss = Math.sin(P[2]);
        const T = (px, py) => [P[0] + cc * px - ss * py, P[1] + ss * px + cc * py, 0.004];
        const col = { clear: "#34d399", limiting: "#f0b84b", stale: "#73695f" }[B.state] || "#ff4d6d";
        if (wpoly([T(x1 + mg, y0 - 0.03), T(x1 + mg, y1 + 0.03), T(x0 - mg, y1 + 0.03), T(x0 - mg, y0 - 0.03)])) {
          ctx.fillStyle = col + "33"; ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.fill(); ctx.stroke();
        }
      }
      // goal + labels
      if (S.goal) { const b = wp(S.goal.x, S.goal.y, 0), t = wp(S.goal.x, S.goal.y, 0.7); if (b && t) { ctx.strokeStyle = "#7dd3fc"; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(b[0], b[1]); ctx.lineTo(t[0], t[1]); ctx.stroke(); } }
      for (const l of S.labels || []) { const t = wp(l.x, l.y, 0.5); if (t && t[2] < 14) tag(t[0], t[1], l.name, "#f08a4b"); }
      const M = camModel();
      drawHud(M, P0);
      ctx.fillStyle = "#94a3b8"; ctx.font = "12px Inter, sans-serif";
      ctx.fillText(`World rebuilt from the lidar map, coloured by the camera: ${wallColor.size} wall / ${floorColor.size} floor patches seen · drag to orbit · wheel to zoom`, 16, H - 16);
    }
    function drawRobot(P) {
      const c = Math.cos(P[2]), s = Math.sin(P[2]);
      const T = (px, py, z) => [P[0] + c * px - s * py, P[1] + s * px + c * py, z];
      const box = (xa, xb, ya, yb, za, zb, fill, edge) => {
        const v = [[xa, ya], [xb, ya], [xb, yb], [xa, yb]];
        const faces = [[0, 1], [1, 2], [2, 3], [3, 0]].map(([i, j]) => [T(v[i][0], v[i][1], za), T(v[j][0], v[j][1], za), T(v[j][0], v[j][1], zb), T(v[i][0], v[i][1], zb)]);
        faces.push(v.map((p) => T(p[0], p[1], zb)));
        faces.map((f) => { const q = f.map((p) => wp(...p)); return q.some((x) => !x) ? null : { f, d: q.reduce((a, x) => a + x[2], 0) / 4 }; })
          .filter(Boolean).sort((a, b) => b.d - a.d)
          .forEach((it) => { if (wpoly(it.f)) { ctx.fillStyle = fill; ctx.strokeStyle = edge; ctx.lineWidth = 1; ctx.fill(); ctx.stroke(); } });
      };
      box(-0.052, 0.052, -0.125, -0.095, 0, 0.104, "#1f2937", "#475569");
      box(-0.052, 0.052, 0.095, 0.125, 0, 0.104, "#1f2937", "#475569");
      box(-0.05, 0.05, -0.09, 0.09, 0.06, 0.25, "rgba(226,232,240,0.95)", "#64748b");
      box(-0.035, 0.035, -0.066, 0.004, 0.25, 0.28, "#0f172a", "#38bdf8");
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
    root.querySelectorAll("[data-dmode]").forEach((b) => (b.onclick = () => setMode(b.dataset.dmode)));
    root.querySelector('[data-act="video"]').onclick = () => startVideo();
    root.querySelector('[data-act="align"]').onclick = () => { alignBox.hidden = !alignBox.hidden; };
    root.querySelector('[data-act="clear"]').onclick = () => { pano = null; wallColor.clear(); floorColor.clear(); };
    root.querySelector('[data-act="save"]').onclick = () => {
      if (!pano) return;
      const a = document.createElement("a"); a.download = `2pybot-panorama-${Date.now()}.png`; a.href = pano.cv.toDataURL("image/png"); a.click();
    };
    let drag = null;
    cv.addEventListener("pointerdown", (e) => { drag = { x: e.clientX, y: e.clientY }; cv.setPointerCapture(e.pointerId); });
    cv.addEventListener("pointermove", (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag = { x: e.clientX, y: e.clientY };
      if (mode === "pano") { view.follow = false; view.yaw += dx * (view.hfov * D2R) / W; view.pitch = clamp(view.pitch + dy, -H / 2, H / 2); }
      else if (mode === "world") { view.wcam.yaw -= dx * 0.006; view.wcam.height = clamp(view.wcam.height + dy * 0.005, 0.3, 4); }
    });
    cv.addEventListener("pointerup", () => { drag = null; });
    cv.addEventListener("dblclick", () => { view.follow = true; view.pitch = 0; view.wcam = { yaw: 0, dist: 1.6, height: 0.9 }; });
    cv.addEventListener("wheel", (e) => {
      e.preventDefault();
      if (mode === "pano") view.hfov = clamp(view.hfov * (e.deltaY < 0 ? 0.92 : 1.08), 35, 150);
      else if (mode === "world") view.wcam.dist = clamp(view.wcam.dist * (e.deltaY < 0 ? 0.9 : 1.1), 0.5, 8);
    }, { passive: false });

    function setMode(m) {
      mode = m;
      root.querySelectorAll("[data-dmode]").forEach((b) => b.setAttribute("aria-selected", b.dataset.dmode === m));
      video.style.visibility = m === "camera" ? "visible" : "hidden";          // keeps decoding for capture
      note.textContent = m === "camera" ? "Live camera with lidar distances"
        : m === "pano" ? "360° stitched from camera frames by heading" : "3D map coloured by the camera";
    }

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
        capture(now);
        if (mode === "camera") drawCamera(); else if (mode === "pano") drawPano(); else drawWorld();
      } catch (e) { note.textContent = "Drive view error: " + e.message; }
      raf = requestAnimationFrame(frameLoop);
    }
    function show(on) {
      shown = !!on;
      if (shown) { resize(); setMode(mode); if (!pc) startVideo(); if (!raf) raf = requestAnimationFrame(frameLoop); }
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
      walls = pool(sc.walls); floor = pool(sc.floor); sceneVersion = sc.version; api.version = sc.version;
    }
    function setHud(h) { hud = h; }
    const api = { show, update, setScene, setHud, resize, version: -1 };
    return api;
  }
  window.DriveView = { create };
})();
