/* 2PyBot Maps — plain JS, no build step, no external assets. */
"use strict";
const $ = (id) => document.getElementById(id);
const base = location.pathname.replace(/\/?$/, "/");          // works at / and /slam/
let S = null, mapImg = null, mapMeta = null, view = "map";
let cam = { x: 0, y: 0, scale: 60, follow: true };              // px per metre

// ---------------------------------------------------------------- data
function connect() {
  const ws = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + base + "ws");
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.map) {
      const img = new Image();
      img.onload = () => { mapImg = img; mapMeta = m.map; };
      img.src = "data:image/png;base64," + m.map.png;
    }
    if (m.state) { S = m.state; render(); }
  };
  ws.onclose = () => { setSay("Lost connection to the robot. Reconnecting…", "err"); setTimeout(connect, 1500); };
}
async function post(path, body) {
  const r = await fetch(base + path, { method: "POST", headers: { "Content-Type": "application/json" },
                                       body: body ? JSON.stringify(body) : null });
  return r.json().catch(() => ({ ok: r.ok }));
}
async function refreshMaps() {
  const maps = await (await fetch(base + "api/maps")).json().catch(() => []);
  const sel = $("map-list");
  sel.innerHTML = maps.length ? maps.map((m) => `<option>${m.name}</option>`).join("")
                              : `<option value="">No maps yet</option>`;
}

// ---------------------------------------------------------------- UI state
function setCheck(id, state) { $(id).className = state; }
function setSay(text, cls) { const e = $("say"); e.textContent = text; e.className = "say " + (cls || ""); }
function meter(id, frac, label, bad) {
  const f = Math.max(0, Math.min(1, frac || 0));
  $(id).style.width = (f * 100).toFixed(0) + "%";
  $(id).style.background = bad ? "var(--err)" : f < 0.4 ? "var(--warn)" : "var(--ok)";
  $(id + "-v").textContent = label;
}
const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v) ? "–" : Number(v).toFixed(d));

function render() {
  const L = S.lidar || {}, live = S.robot_live, scanning = L.state === "scanning";
  // header
  $("d-lidar").className = "dot " + (scanning ? "ok" : L.state === "error" ? "err" : "warn");
  $("h-lidar").textContent = scanning ? `${fmt(S.rates.scan, 0)} Hz` : (L.state || "off");
  $("d-robot").className = "dot " + (live ? "ok" : "warn");
  $("h-robot").textContent = live ? "connected" : "no data";
  $("h-mode").textContent = { idle: "ready", mapping: "mapping", localizing: "using map" }[S.mode] || S.mode;

  // step 1
  setCheck("c-lidar-conn", scanning ? "ok" : L.state === "error" ? "bad" : "wait");
  setCheck("c-lidar-health", L.health === "good" ? "ok" : L.health ? "bad" : "");
  const hz = S.rates.scan || 0;
  setCheck("c-lidar-rate", hz > 6 ? "ok" : scanning ? "wait" : "");
  $("h-step1").textContent = L.error ? "Problem: " + L.error : (L.note || "");
  // step 2
  setCheck("c-robot", live ? "ok" : "wait");
  setCheck("c-axis", S.odom.axis ? "ok" : live ? "wait" : "");
  $("c-axis").textContent = S.odom.axis ? `Gyro figured out (axis ${S.odom.axis})` :
                                          `Gyro figured out ("spin the robot a few times")`;
  for (const name of ["push", "turn"]) {
    const t = (S.tests || {})[name];
    if (!t) continue;
    const val = name === "push" ? `${fmt(t.distance_m)} m` : `${fmt(t.turn_deg, 0)}°`;
    $("t-" + name).innerHTML = `${val} — <span class="${t.pass ? "pass" : "fail"}">${t.pass ? "looks right" : t.hint}</span>`;
  }
  // step highlighting
  const s1 = scanning && hz > 6, s2 = live && !!S.odom.axis, s3 = S.map_version > 0;
  const active = !s1 ? 1 : !s2 ? 2 : S.mode === "mapping" || !s3 ? 3 : S.mode === "localizing" ? 5 : 4;
  document.querySelectorAll(".step").forEach((el) => {
    const n = +el.dataset.step;
    el.classList.toggle("active", n === active);
    el.classList.toggle("done", (n === 1 && s1) || (n === 2 && s2) || (n === 3 && s3 && S.mode !== "mapping")
                                || (n === 4 && !!S.map_name));
  });
  $("b-start").disabled = S.mode === "mapping";
  $("b-stop").disabled = S.mode === "idle";
  $("b-save").disabled = S.map_version === 0;

  // meters + numbers
  const pts = L.points || (S.scan || []).length;
  meter("m-scan", pts / 450, pts ? `${pts}` : "–");
  meter("m-match", S.mode === "idle" ? 0 : S.match.score / 0.5, S.mode === "idle" ? "–" : fmt(S.match.score),
        S.mode !== "idle" && !S.match.ok);
  const tilt = Math.abs(S.attitude.dev || 0);
  meter("m-tilt", 1 - tilt / 6, live ? `${fmt(tilt, 1)}°` : "–", tilt > 3);
  $("n-pose").textContent = `${fmt(S.pose[0])}, ${fmt(S.pose[1])} m · ${fmt((S.pose[2] * 180) / Math.PI, 0)}°`;
  $("n-odom").textContent = live ? `${fmt(Math.hypot(S.odom.x, S.odom.y))} m (${S.odom.source})` : "–";
  $("n-range").textContent = S.gate.usable_range ? `${fmt(S.gate.usable_range, 1)} m` : "–";
  $("n-gate").textContent = `${S.gate.passed} / ${S.gate.tilt + S.gate.rate}`;
  $("n-pts").textContent = pts || "–";
  $("n-fw").textContent = L.model !== null && L.model !== undefined ? `${L.model} / ${L.firmware}` : "–";
  $("adv").innerHTML = Object.entries(S.settings || {})
    .map(([k, v]) => `<div><span>${k}</span><b>${v}</b></div>`).join("");

  const warn = /skipping|not sure|can't|Problem|error/i.test(S.message);
  setSay(S.message, warn ? "warn" : "");
  draw();
}

// ---------------------------------------------------------------- drawing
const cv = $("cv"), ctx = cv.getContext("2d");
function resize() {
  const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  cv.width = Math.max(1, r.width * dpr); cv.height = Math.max(1, r.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); draw();
}
function w2s(x, y) {                                          // world (m) -> screen (css px)
  const r = cv.getBoundingClientRect();
  return [r.width / 2 + (x - cam.x) * cam.scale, r.height / 2 - (y - cam.y) * cam.scale];
}
function draw() {
  const r = cv.getBoundingClientRect();
  ctx.fillStyle = "#0f0d0c"; ctx.fillRect(0, 0, r.width, r.height);
  if (!S) return;
  const pose = S.pose;
  if (view === "radar") return drawRadar(r);
  if (cam.follow) { cam.x = pose[0]; cam.y = pose[1]; }
  // grid lines every metre
  ctx.strokeStyle = "#1e1a17"; ctx.lineWidth = 1;
  const span = Math.ceil(Math.max(r.width, r.height) / cam.scale / 2) + 1;
  for (let i = -span; i <= span; i++) {
    const gx = Math.round(cam.x) + i, gy = Math.round(cam.y) + i;
    let a = w2s(gx, cam.y - span), b = w2s(gx, cam.y + span);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    a = w2s(cam.x - span, gy); b = w2s(cam.x + span, gy);
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
  }
  if (mapImg && mapMeta) {
    const [sx, sy] = w2s(mapMeta.x0, mapMeta.y0 + mapMeta.h);
    ctx.imageSmoothingEnabled = cam.scale < 40;
    ctx.globalAlpha = 0.95;
    ctx.drawImage(mapImg, sx, sy, mapMeta.w * cam.scale, mapMeta.h * cam.scale);
    ctx.globalAlpha = 1;
  }
  // path
  if (S.path && S.path.length > 1) {
    ctx.strokeStyle = "rgba(240,138,75,.75)"; ctx.lineWidth = 2; ctx.beginPath();
    S.path.forEach((p, i) => { const [x, y] = w2s(p[0], p[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke();
  }
  // live scan in map frame
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  ctx.fillStyle = "#60a5fa";
  for (const p of S.scan || []) {
    const [x, y] = w2s(pose[0] + c * p[0] - s * p[1], pose[1] + s * p[0] + c * p[1]);
    ctx.fillRect(x - 1.2, y - 1.2, 2.4, 2.4);
  }
  // ToF 3D layer (already in map frame): low obstacles orange, drop-offs red
  const T = S.tof || {};
  ctx.fillStyle = "#f08a4b";
  for (const p of T.obstacles || []) { const [x, y] = w2s(p[0], p[1]); ctx.fillRect(x - 2, y - 2, 4, 4); }
  ctx.fillStyle = "#ff4d6d";
  for (const p of T.cliffs || []) { const [x, y] = w2s(p[0], p[1]); ctx.fillRect(x - 2.5, y - 2.5, 5, 5); }
  drawRobot(...w2s(pose[0], pose[1]), pose[2]);
  // scale bar
  ctx.fillStyle = "#aba196"; ctx.font = "12px sans-serif";
  ctx.fillRect(16, r.height - 22, cam.scale, 3); ctx.fillText("1 m", 16, r.height - 28);
}
function drawRobot(x, y, th) {
  ctx.save(); ctx.translate(x, y); ctx.rotate(-th);
  ctx.fillStyle = "#f08a4b"; ctx.strokeStyle = "#141210"; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(14, 0); ctx.lineTo(-9, 9); ctx.lineTo(-5, 0); ctx.lineTo(-9, -9); ctx.closePath();
  ctx.fill(); ctx.stroke(); ctx.restore();
}
function drawRadar(r) {
  const cx = r.width / 2, cy = r.height / 2, R = Math.min(cx, cy) - 20;
  const maxM = 6, k = R / maxM;
  ctx.strokeStyle = "#2a241f"; ctx.fillStyle = "#73695f"; ctx.font = "12px sans-serif";
  for (let m = 1; m <= maxM; m++) {
    ctx.beginPath(); ctx.arc(cx, cy, m * k, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillText(`${m} m`, cx + 4, cy - m * k + 14);
  }
  ctx.beginPath(); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.stroke();
  ctx.fillStyle = "#f08a4b"; ctx.fillText("FRONT", cx - 20, cy - R - 6);
  ctx.fillStyle = "#60a5fa";                                    // robot frame: x forward = up
  for (const p of S.scan || []) {
    const x = cx - p[1] * k, y = cy - p[0] * k;
    ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
  }
  drawRobot(cx, cy, Math.PI / 2);
}

// ---------------------------------------------------------------- input
let drag = null, pinch = null;
cv.addEventListener("wheel", (e) => {
  e.preventDefault(); cam.scale = Math.max(8, Math.min(400, cam.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15))); draw();
}, { passive: false });
cv.addEventListener("pointerdown", (e) => { drag = { x: e.clientX, y: e.clientY }; cv.setPointerCapture(e.pointerId); });
cv.addEventListener("pointermove", (e) => {
  if (!drag || pinch) return;
  cam.follow = false;
  cam.x -= (e.clientX - drag.x) / cam.scale; cam.y += (e.clientY - drag.y) / cam.scale;
  drag = { x: e.clientX, y: e.clientY }; draw();
});
cv.addEventListener("pointerup", () => { drag = null; });
cv.addEventListener("touchmove", (e) => {
  if (e.touches.length !== 2) return;
  const d = Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY);
  if (pinch) cam.scale = Math.max(8, Math.min(400, cam.scale * d / pinch));
  pinch = d; draw();
}, { passive: true });
cv.addEventListener("touchend", () => { pinch = null; });

function setView(v) {
  view = v;
  $("v-map").setAttribute("aria-selected", v === "map"); $("v-radar").setAttribute("aria-selected", v === "radar");
  $("view-note").textContent = v === "map" ? "Scroll or pinch to zoom · drag to move" : "Live scan around the robot (top = front)";
  draw();
}
$("v-map").onclick = () => setView("map");
$("v-radar").onclick = () => setView("radar");
$("b-center").onclick = () => { cam.follow = true; draw(); };
$("estop").onclick = () => post("api/estop");
$("b-start").onclick = () => post("api/mapping/start?fresh=true");
$("b-stop").onclick = () => post("api/mapping/stop");
document.querySelectorAll("[data-test]").forEach((b) => (b.onclick = () => post("api/test/" + b.dataset.test)));
$("b-save").onclick = async () => {
  const name = $("map-name").value.trim();
  const r = await post("api/maps/save", { name });
  $("h-save").textContent = r.ok ? `Saved “${name}”.` : (r.error || "Could not save.");
  refreshMaps();
};
$("b-load").onclick = () => { const n = $("map-list").value; if (n) post("api/maps/load", { name: n }); };
$("b-find").onclick = async () => {
  setSay("Looking around the map for a match…");
  const r = await post("api/relocalize");
  setSay(r.ok ? "Found a likely position." : "Couldn't find a match. Drive somewhere with more features.", r.ok ? "" : "warn");
};
window.addEventListener("resize", resize);
resize(); connect(); refreshMaps();
setView(new URLSearchParams(location.search).get("view") === "radar" ? "radar" : "map");
