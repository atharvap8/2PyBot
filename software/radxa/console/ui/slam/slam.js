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
    if (m.state) { S = m.state; render(); update3d(); }
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
  if (S.map_name && !$("map-name").value && document.activeElement !== $("map-name")) $("map-name").value = S.map_name;
  $("b-extend").disabled = S.mode === "mapping";
  renderLabels();
  if (!guard) renderBubble();
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
  drawOverlays(pose);
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
let drag = null, pinch = null, tap = null;
cv.addEventListener("wheel", (e) => {
  e.preventDefault(); cam.scale = Math.max(8, Math.min(400, cam.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15))); draw();
}, { passive: false });
cv.addEventListener("pointerdown", (e) => {
  drag = { x: e.clientX, y: e.clientY }; tap = { x: e.clientX, y: e.clientY, moved: 0 };
  cv.setPointerCapture(e.pointerId);
});
cv.addEventListener("pointermove", (e) => {
  if (!drag || pinch) return;
  if (tap) tap.moved += Math.abs(e.clientX - drag.x) + Math.abs(e.clientY - drag.y);
  if (tap && tap.moved < 6) return;                     // still a tap, not a pan
  cam.follow = false;
  cam.x -= (e.clientX - drag.x) / cam.scale; cam.y += (e.clientY - drag.y) / cam.scale;
  drag = { x: e.clientX, y: e.clientY }; draw();
});
cv.addEventListener("pointerup", (e) => {
  const t = tap; drag = null; tap = null;
  if (t && t.moved < 6 && !pinch && view === "map" && tool) {
    const r = cv.getBoundingClientRect();
    toolTap(...s2w(e.clientX - r.left, e.clientY - r.top));
  }
});

// ---------------------------------------------------------------- map tools
let tool = null;                                              // null | "label" | "erase" | "goal"
function s2w(px, py) {                                        // screen (css px) -> world (m)
  const r = cv.getBoundingClientRect();
  return [cam.x + (px - r.width / 2) / cam.scale, cam.y - (py - r.height / 2) / cam.scale];
}
function setTool(t) {
  tool = tool === t ? null : t;
  document.querySelectorAll(".btn.tool").forEach((b) => b.setAttribute("aria-pressed", b.dataset.tool === tool));
  if (tool && view !== "map") setView("map");
  $("view-note").textContent = tool === "label" ? "Tap a spot to name it"
    : tool === "erase" ? "Tap to erase (drag still pans)" : tool === "goal" ? "Tap where the robot should go"
    : "Scroll or pinch to zoom · drag to move";
}
document.querySelectorAll(".btn.tool").forEach((b) => (b.onclick = () => setTool(b.dataset.tool)));
async function toolTap(x, y) {
  if (tool === "label") {
    const name = (prompt(`Name this spot (${x.toFixed(2)}, ${y.toFixed(2)} m):`) || "").trim();
    if (!name) return;
    const r = await post("api/labels", { name, x, y });
    if (!r.ok) setSay(r.error || "Could not add the label.", "warn");
  } else if (tool === "erase") {
    const r = await post("api/map/erase", { x, y, r: +$("erase-r").value });
    if (!r.ok) setSay(r.error || "Could not erase.", "warn");
  } else if (tool === "goal") {
    picked = { x, y };                                        // chosen; Start sends it
    setKind("goto"); renderLabels.key = null; renderLabels();
    $("goal-label").value = "__picked";
    setTool("goal");                                          // one tap = one spot
    draw();
  }
}
function renderLabels() {
  const labels = S.labels || [];
  const key = JSON.stringify(labels);
  if (key === renderLabels.key) return;
  renderLabels.key = key;
  const esc = (s) => String(s).replace(/[&<>"']/g, (c) => `&#${c.charCodeAt(0)};`);
  $("label-list").innerHTML = labels.map((l) =>
    `<li><b>${esc(l.name)}</b><span class="sub">${l.x.toFixed(1)}, ${l.y.toFixed(1)}</span>` +
    `<button data-go="${esc(l.name)}" title="Drive there">Go</button><button data-del="${esc(l.name)}" title="Delete">✕</button></li>`).join("")
    || `<li class="sub">No labels yet.</li>`;
  const sel = $("goal-label"), cur = sel.value;
  sel.innerHTML = `<option value="">${labels.length ? "Choose a place…" : "No places yet: name one in step 7"}</option>`
    + labels.map((l) => `<option>${esc(l.name)}</option>`).join("")
    + (picked ? `<option value="__picked">Spot picked on map (${picked.x.toFixed(1)}, ${picked.y.toFixed(1)})</option>` : "");
  sel.value = cur === "__picked" && picked ? cur : labels.some((l) => l.name === cur) ? cur : "";
  $("label-list").querySelectorAll("[data-del]").forEach((b) => (b.onclick = () => post("api/labels/delete", { name: b.dataset.del })));
  $("label-list").querySelectorAll("[data-go]").forEach((b) => (b.onclick = () => {
    setKind("goto"); $("goal-label").value = b.dataset.go; startMission();
  }));
}
function drawOverlays(pose) {
  // planned path + goal
  if (S.plan && S.plan.length > 1) {
    ctx.strokeStyle = "#38bdf8"; ctx.lineWidth = 3; ctx.setLineDash([8, 6]); ctx.beginPath();
    S.plan.forEach((p, i) => { const [x, y] = w2s(p[0], p[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke(); ctx.setLineDash([]);
  }
  if (picked && !S.goal) {                                     // chosen but not started yet
    const [px, py] = w2s(picked.x, picked.y);
    ctx.strokeStyle = "#38bdf8"; ctx.setLineDash([4, 4]); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(px, py, 10, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
  }
  if (S.goal) {
    const [gx, gy] = w2s(S.goal.x, S.goal.y);
    ctx.strokeStyle = "#38bdf8"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(gx, gy, 9, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(gx, gy, 3, 0, 2 * Math.PI); ctx.fillStyle = "#38bdf8"; ctx.fill();
  }
  // area labels
  ctx.font = "600 12px sans-serif"; ctx.textBaseline = "bottom";
  for (const l of S.labels || []) {
    const [x, y] = w2s(l.x, l.y);
    ctx.fillStyle = "#f08a4b"; ctx.beginPath(); ctx.arc(x, y, 4, 0, 2 * Math.PI); ctx.fill();
    const w = ctx.measureText(l.name).width + 10;
    ctx.fillStyle = "rgba(20,18,16,.85)"; ctx.fillRect(x - w / 2, y - 24, w, 17);
    ctx.fillStyle = "#efe9e3"; ctx.fillText(l.name, x - w / 2 + 5, y - 9);
  }
  ctx.textBaseline = "alphabetic";
  // safety bubble footprint (+ stop margin), coloured by state
  const B = S.bubble;
  if (B && B.footprint) {
    const [x0, x1, y0, y1] = B.footprint, m = B.stop_margin || 0.06;
    const col = { clear: "#34d399", limiting: "#f0b84b", stale: "#73695f" }[B.state] || "#ff4d6d";
    const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
    const poly = (pts) => {
      ctx.beginPath();
      pts.forEach(([px, py], i) => { const [X, Y] = w2s(pose[0] + c * px - s * py, pose[1] + s * px + c * py); i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); });
      ctx.closePath();
    };
    ctx.strokeStyle = col; ctx.lineWidth = 1.5;
    poly([[x1, y0], [x1, y1], [x0, y1], [x0, y0]]); ctx.stroke();
    ctx.setLineDash([3, 3]);
    poly([[x1 + m, y0 - 0.03], [x1 + m, y1 + 0.03], [x0 - m, y1 + 0.03], [x0 - m, y0 - 0.03]]); ctx.stroke();
    ctx.setLineDash([]);
  }
}

// ---------------------------------------------------------------- autonomy
let sup = null, guard = null, picked = null, kind = "goto", lastPoll = null, flash = null;   // sup: {id, key}
async function api(path, body) {
  const r = await fetch(path, { method: body === undefined ? "GET" : "POST", headers: { "Content-Type": "application/json" },
                                 body: body === undefined ? undefined : JSON.stringify(body) });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { const e = new Error(j.detail || `HTTP ${r.status}`); e.status = r.status; throw e; }
  return j;
}
function setKind(k) {
  kind = k;
  document.querySelectorAll(".kinds [data-kind]").forEach((b) => b.setAttribute("aria-selected", b.dataset.kind === k));
  document.querySelectorAll(".kind-opts").forEach((el) => (el.hidden = !el.dataset.for.split(" ").includes(k)));
  renderMission();
}
document.querySelectorAll(".kinds [data-kind]").forEach((b) => (b.onclick = () => setKind(b.dataset.kind)));
function showFlash(text) { flash = { text, until: Date.now() + 6000 }; renderMission(); }
async function startMission() {
  const body = { kind, speed_mps: +$("m-speed").value };
  if (kind !== "goto") body.duration_s = +$("m-time").value;
  else {
    const v = $("goal-label").value;
    if (v === "__picked" && picked) body.goal = { x: picked.x, y: picked.y, name: "picked spot" };
    else if (v) body.goal = { label: v };
    else return showFlash("Choose a place, or press Pick on map and tap the map.");
  }
  try {
    const r = await api("/api/autonomy/mission/start", body);
    sup = { id: r.mission_id, key: r.lease_key }; flash = null; pollMission();
  } catch (e) { showFlash(e.message); }
}
async function heartbeat() {
  if (!sup) return;
  try { await api("/api/autonomy/heartbeat", { mission_id: sup.id, lease_key: sup.key }); }
  catch (e) { if (e.status === 409) sup = null; }            // network blips retry; only a stale lease ends supervision
}
const REASONS = [
  [/supervisor|backgrounded/, "The page was hidden or lost its connection."],
  [/gamepad takeover|lease lost/, "You took over with the gamepad."],
  [/feedback lost|telemetry/, "Lost contact with the controller."],
  [/disarmed|fell/, "The robot stopped balancing."],
  [/bubble stale/, "The lidar stopped seeing (lean or lidar off)."],
  [/boxed in/, "Boxed in: no room to turn or back out."],
  [/lost localization/, "Lost its place on the map."],
  [/path blocked|No mapped path|not on mapped/, null],
  [/operator|E-STOP/, "Stopped by you."],
];
function human(reason) {
  for (const [re, text] of REASONS) if (re.test(reason || "")) return text || reason;
  return reason || "";
}
function progressText(m) {
  const i = m.info || {};
  if (i.waiting) return { planning: "Planning a route…", localizing: "Finding itself on the map…", "finding frontiers": "Looking for unmapped edges…",
    "lidar bubble stale": "Waiting for the lidar…" }[i.waiting] || `Waiting: ${i.waiting}`;
  if (m.kind === "goto") return i.distance_m !== undefined ? `${fmt(i.distance_m, 1)} m to go` : "On the way";
  const mins = i.remaining_s !== undefined ? `${Math.floor(i.remaining_s / 60)}:${String(Math.max(0, i.remaining_s % 60)).padStart(2, "0")} left` : "";
  if (m.kind === "roam") return [`${fmt(i.distance_m, 1)} m driven`, `${i.turns || 0} turns`, mins].join(" · ");
  if (m.kind === "explore") return [`${i.frontiers_reached || 0} edges reached`, mins].join(" · ");
  return "";
}
function renderMission() {
  const r = lastPoll, m = r && r.mission, pill = $("m-pill"), card = $("m-card");
  const active = m && (m.state === "running" || m.state === "paused");
  $("m-setup").hidden = !!active;
  $("b-mstart").hidden = !!active;
  $("b-mpause").hidden = !(m && m.state === "running");
  $("b-mresume").hidden = !(m && m.state === "paused");
  $("b-mstop").hidden = !active;
  $("m-bar").hidden = true;
  let title, detail, tone;
  if (!r) { title = "Autonomy unavailable"; detail = "Open this page from the console (port 8080)."; tone = "off"; }
  else if (m && m.state === "running") {
    title = m.name; detail = progressText(m); tone = "run";
    if (m.kind === "goto" && m.info && m.info.distance_m !== undefined) {
      if (!renderMission.start || renderMission.id !== m.id) { renderMission.start = m.info.distance_m; renderMission.id = m.id; }
      $("m-bar").hidden = false;
      $("m-bar-fill").style.width = `${Math.round(100 * (1 - m.info.distance_m / Math.max(0.01, renderMission.start)))}%`;
    }
  } else if (m && m.state === "paused") { title = `Paused · ${m.name}`; detail = human(m.reason); tone = "warn"; }
  else {
    const reasons = (r.blocked_reasons || {})[kind] || [];
    const last = m && (m.state === "completed" || m.state === "aborted" || m.state === "stopped") ? m : null;
    if (reasons.length) { title = "Not ready"; detail = reasons[0]; tone = "off"; }
    else { title = "Ready"; detail = kind === "goto" ? "Choose a place and press Start." : "Press Start."; tone = "ok"; }
    if (last && Date.now() - (renderMission.lastSeen || 0) < 15000) {
      title = last.state === "completed" ? (last.kind === "goto" ? `Arrived · ${last.name.replace(/^Go to /, "")}` : `Done · ${last.name}`) : `Stopped · ${last.name}`;
      detail = human(last.reason); tone = last.state === "completed" ? "ok" : "warn";
    }
  }
  if (flash && Date.now() < flash.until && !(m && m.state === "running")) { detail = flash.text; tone = tone === "ok" ? "warn" : tone; }
  $("m-title").textContent = title; $("m-detail").textContent = detail;
  card.className = "auto-card " + tone;
  pill.textContent = { run: "Driving", warn: "Paused", ok: "Ready", off: "Not ready" }[tone] || "";
  if (m && m.state === "paused") pill.textContent = "Paused";
  pill.className = "pill " + tone;
}
async function pollMission() {
  try {
    const r = await api("/api/autonomy/mission");
    const m = r.mission;
    if (m && (m.state === "running" || m.state === "paused")) renderMission.lastSeen = Date.now();
    lastPoll = r; guard = r.bubble;
    if (m && m.state !== "running" && m.state !== "paused") sup = null;
  } catch (e) { /* console offline or separate SLAM port: no autonomy here */ }
  renderMission(); renderBubble();
}
function renderBubble() {
  const B = guard || (S && S.bubble);
  if (!B) return;
  const tone = { clear: "ok", limiting: "warn", stale: "off" }[B.state] || "bad";
  $("bubble-dot").className = "dot " + tone;
  const what = { clear: "clear", limiting: "slowing down", front_blocked: "blocked ahead", rear_blocked: "blocked behind",
    boxed_in: "boxed in", stale: "no lidar data" }[B.state] || B.state;
  const ahead = B.front_m !== null && B.front_m !== undefined ? ` · ${fmt(B.front_m, 2)} m ahead` : "";
  $("bubble-text").textContent = `Safety bubble ${what}${ahead}`;
  if (guard && document.activeElement !== $("bubble-on")) $("bubble-on").checked = !!guard.enabled;
}
$("bubble-on").onchange = async (e) => { try { guard = await api("/api/bubble", { enabled: e.target.checked }); renderBubble(); } catch (err) { showFlash(err.message); } };
$("b-mstart").onclick = () => startMission();
$("b-mpause").onclick = () => api("/api/autonomy/pause", {}).then(pollMission).catch((e) => showFlash(e.message));
$("b-mstop").onclick = () => { sup = null; api("/api/autonomy/stop", {}).then(pollMission).catch((e) => showFlash(e.message)); };
$("b-mresume").onclick = async () => {
  try { const r = await api("/api/autonomy/resume", {}); sup = { id: r.mission_id, key: r.lease_key }; pollMission(); }
  catch (e) { showFlash(e.message); }
};
setKind("goto");
$("b-extend").onclick = async () => {
  const n = $("map-list").value; if (!n) return;
  const r = await post("api/maps/extend", { name: n });
  if (r.ok) $("map-name").value = n; else setSay(r.error || "Could not load the map.", "warn");
};
$("b-delete").onclick = async () => {
  const n = $("map-list").value;
  if (n && confirm(`Delete map “${n}” permanently?`)) { await post("api/maps/delete", { name: n }); refreshMaps(); }
};
// Supervision: the robot only keeps driving while this page is visible.
const pauseNow = () => { if (sup) fetch("/api/autonomy/pause", { method: "POST", keepalive: true }); };
document.addEventListener("visibilitychange", () => { if (document.hidden) pauseNow(); });
window.addEventListener("pagehide", pauseNow);
setInterval(heartbeat, 700);
setInterval(pollMission, 700);
cv.addEventListener("touchmove", (e) => {
  if (e.touches.length !== 2) return;
  const d = Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY);
  if (pinch) cam.scale = Math.max(8, Math.min(400, cam.scale * d / pinch));
  pinch = d; draw();
}, { passive: true });
cv.addEventListener("touchend", () => { pinch = null; });

// ---------------------------------------------------------------- 3D view
const cv3 = $("cv3"), s3d = window.Slam3D ? window.Slam3D.create(cv3) : null;
let scene3dKey = "", scene3dAt = 0, scene3dBusy = false;
async function update3d(force) {
  if (view !== "3d" || !s3d || !S) return;
  s3d.setPose(S.pose);
  const key = `${S.map_version}`;
  const now = Date.now();
  if (scene3dBusy || (!force && (key === scene3dKey || now - scene3dAt < 2500))) return;
  scene3dBusy = true; scene3dAt = now;
  try {
    const sc = await (await fetch(base + "api/scene3d")).json();
    s3d.setScene(sc); scene3dKey = key;
    const st = s3d.stats();
    $("stat3d").textContent = `${st.walls} wall · ${st.floor} floor cells`;
  } catch (e) { /* the next state message retries */ }
  scene3dBusy = false;
}
function setView(v) {
  if (v === "3d" && !s3d) { setSay("3D needs WebGL, which this browser does not provide.", "warn"); v = "map"; }
  view = v;
  for (const k of ["map", "radar", "3d"]) $("v-" + k).setAttribute("aria-selected", v === k);
  cv.hidden = v === "3d"; cv3.hidden = v !== "3d";
  $("legend3d").hidden = v !== "3d";
  if (s3d) s3d.show(v === "3d");
  $("view-note").textContent = v === "map" ? "Scroll or pinch to zoom · drag to move"
    : v === "radar" ? "Live scan around the robot (top = front)"
    : "45° corner view · drag to rotate · two fingers or right-drag to pan · pinch or scroll to zoom";
  if (v === "3d") update3d(true);
  else resize();
}
$("v-map").onclick = () => setView("map");
$("v-radar").onclick = () => setView("radar");
$("v-3d").onclick = () => setView("3d");
$("b-center").onclick = () => { cam.follow = true; if (view === "3d" && s3d) s3d.center(); draw(); };
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
setView({ map: "map", radar: "radar", "3d": "3d" }[new URLSearchParams(location.search).get("view")] || "3d");
