/* shell/shell.js - the console shell.
 *
 * Left: the live 3D view (viz/engine.js) with its instruments. Right: an app pane
 * (Drive, Maps, Camera, ML, Controls, System, Terminal, Deploy). Bottom: the dock.
 *
 * The 3D view picks its own mode from live state, like a car does from its gear:
 *   parked  disarmed                      -> studio shot of the robot
 *   drive   balancing (gamepad or a mission) -> chase camera over the map
 *   park    balancing, slow, something close -> top-down with proximity arcs
 *
 * Safety: this page never starts motion. It shows state and can only stop things
 * (STOP, pause or end a mission, power down). Missions are started from the Maps
 * app, which keeps its own supervision lease exactly as before.
 */
import { API, msg, state, refreshParams, renderGroup } from '../core.js';
import { createEngine, SECTORS } from '../viz/engine.js';
import { liveVideo } from '../live_video.js';

const $ = (id) => document.getElementById(id);
const Q = new URLSearchParams(location.search);
const DEMO = Q.has('demo');
const D2R = Math.PI / 180;
const body = document.body;
const S = { slam: null, mission: null, guard: null, power: { asleep: false }, hub: {}, vp: null, stream: null,
            mapImg: null, mapMeta: null, linkUp: false, near: null, clear: null };
let engine = null, mode = 'parked', held = null, modeAt = 0, app = null, autoOpened = false;

// ------------------------------------------------------------------ helpers
const fin = (v) => typeof v === 'number' && isFinite(v);
const fmt = (v, d = 1) => (fin(v) ? v.toFixed(d) : '-');
const armed = () => !!state.tel.balancing && !state.tel.stale && S.linkUp;
const online = () => S.linkUp && state.tel.stale === false;
const missionActive = () => S.mission && (S.mission.state === 'running' || S.mission.state === 'paused');
const bubble = () => (S.slam && S.slam.bubble) || S.guard || null;

// ------------------------------------------------------------------ view mode
function wantedMode() {
  if (held) return held;
  if (!armed()) return 'parked';
  const v = Math.abs(state.tel.v_ms || 0), n = S.near;
  if (mode === 'park') return fin(n) && n < 0.42 && v < 0.2 ? 'park' : 'drive';
  return fin(n) && n < 0.3 && v < 0.12 ? 'park' : 'drive';
}
function updateMode(force) {
  const want = wantedMode(), now = performance.now();
  if (want === mode && !force) return;
  // do not flap between drive and park; dropping to parked is always immediate
  if (!force && !held && want !== 'parked' && mode !== 'parked' && now - modeAt < 1500) return;
  mode = want; modeAt = now; body.dataset.mode = mode;
  engine && engine.setMode(mode);
  if (mode !== 'parked' && body.dataset.pane === 'closed') { openApp('drive'); autoOpened = true; }
  else if (mode === 'parked' && autoOpened && app === 'drive') closePane();
  paint();
}
const VIEW_CYCLE = [null, 'parked', 'drive', 'park'], VIEW_NAME = { null: 'Auto', parked: 'Parked', drive: 'Drive', park: 'Close' };
$('sh-view').onclick = () => {
  held = VIEW_CYCLE[(VIEW_CYCLE.indexOf(held) + 1) % VIEW_CYCLE.length];
  $('sh-view').classList.toggle('held', !!held);
  $('sh-view').querySelector('span').textContent = VIEW_NAME[held];
  updateMode(true);
};

// ------------------------------------------------------------------ app pane
const TABS = {}, registerTab = (t) => { TABS[t.id] = t; return t; };
const TAB_FILES = ['telemetry', 'camera', 'calibration', 'balance', 'motor', 'drive', 'yaw', 'safety', 'imu', 'climb', 'led', 'payload', 'console'];
const PARAM_TABS = ['balance', 'motor', 'drive', 'yaw', 'imu', 'safety', 'climb', 'led', 'payload'];
const LIVE_TABS = ['telemetry', 'camera', 'calibration', 'console'];      // must not be torn down by a parameter dump
const ctx = { state, API, renderGroup, msg, refreshParams };
let curTab = null, curHost = null;
function leaveTab() {
  try { curTab && curTab.onLeave && curTab.onLeave(); } catch (e) { console.error(e); }
  if (curHost) curHost.innerHTML = '';
  curTab = curHost = null;
}
function showTab(id, host) {
  leaveTab();
  const t = TABS[id];
  if (!t) { host.innerHTML = '<div class="card"><h3>Unavailable</h3><p class="hint">This section failed to load.</p></div>'; return; }
  curTab = t; curHost = host; t.render(host, ctx);
}
function buildControlsNav() {
  const nav = $('sub-controls'); nav.innerHTML = '';
  const add = (id, label) => { if (!TABS[id]) return; const b = document.createElement('button');
    b.textContent = label || TABS[id].title; b.dataset.id = id; b.onclick = () => selectControl(id); nav.appendChild(b); };
  add('telemetry'); add('calibration'); add('console', 'Console log');
  const h = document.createElement('h5'); h.textContent = 'Parameters'; nav.appendChild(h);
  for (const id of PARAM_TABS) add(id);
}
function selectControl(id) {
  localStorage.setItem('sh.control', id);
  for (const b of $('sub-controls').querySelectorAll('button')) b.classList.toggle('sel', b.dataset.id === id);
  showTab(id, $('host-controls'));
}
const feed = liveVideo($('sh-video'), (note) => { $('d-feednote').textContent = note; });
let feedOn = false;
function setFeed(on) {
  if (on === feedOn || DEMO) return; feedOn = on;
  if (on) { $('d-feednote').hidden = false; $('d-feednote').textContent = 'Connecting to the camera'; feed.start(); } else feed.stop();
}
$('sh-video').addEventListener('playing', () => { $('d-feednote').hidden = true; });

function openApp(id) {
  if (id === app && body.dataset.pane === 'open') { if (id === 'drive' && mode === 'parked') closePane(); return; }
  autoOpened = false;
  if (app === 'camera' || app === 'controls') leaveTab();
  app = id;
  for (const el of document.querySelectorAll('.sh-app')) el.classList.toggle('on', el.dataset.app === id);
  for (const b of document.querySelectorAll('#sh-dock [data-open]')) b.classList.toggle('sel', b.dataset.open === id);
  const el = document.querySelector(`.sh-app[data-app="${id}"]`);
  if (el.classList.contains('sh-frame') && !el.firstChild) {         // sub-pages load once and stay alive (Maps keeps its mission lease)
    const f = document.createElement('iframe'); f.src = el.dataset.src; f.title = id; el.appendChild(f);
  }
  if (id === 'camera') showTab('camera', $('host-camera'));
  if (id === 'controls') selectControl(localStorage.getItem('sh.control') || 'telemetry');
  setFeed(id === 'drive');
  body.dataset.pane = 'open';
  if (id === 'drive') requestAnimationFrame(drawPanels);
}
function closePane() {
  if (app === 'camera' || app === 'controls') leaveTab();
  setFeed(false); app = null; autoOpened = false; body.dataset.pane = 'closed';
  for (const b of document.querySelectorAll('#sh-dock [data-open]')) b.classList.remove('sel');
}
for (const b of document.querySelectorAll('#sh-dock [data-open]')) b.onclick = () => openApp(b.dataset.open);
for (const id of ['q-nav', 'd-nav', 'd-open']) $(id).onclick = () => openApp('maps');

// ------------------------------------------------------------------ actions (stop-direction only)
$('sh-stop').onclick = () => API.estop().catch((e) => msg(e.message, true));
$('q-gyro').onclick = () => API.command('C').then(() => msg('Gyro calibration sent')).catch((e) => msg(e.message, true));
$('q-pose').onclick = () => API.post('/api/pose/reset').then(() => msg('Pose reset')).catch((e) => msg(e.message, true));
$('q-pad').onclick = () => {
  if (!confirm('Clear every gamepad pairing on the controller?\n\nThe controller restarts into pairing mode. Hold HOME + A on the pad to pair again.')) return;
  API.post('/api/gamepad/repair').then(() => msg('Pairings cleared. Hold HOME + A on the pad.')).catch((e) => msg(e.message, true));
};
$('q-power').onclick = async () => {
  const sleep = !S.power.asleep;
  if (sleep && !confirm('Power down the robot?\n\nCamera, lidar, depth sensor, LED ring, servos, torch and the gamepad switch off and the robot cannot be armed. Wake it again from this console.')) return;
  S.power = { ...S.power, busy: true, asleep: sleep }; paint();
  try { S.power = await API.power(sleep ? 'sleep' : 'wake'); msg(sleep ? 'Powered down' : 'Awake: camera and sensors starting'); }
  catch (e) { msg(e.message, true); }
  pollPower();
};
$('d-pause').onclick = () => API.post('/api/autonomy/pause', {}).then(pollMission).catch((e) => msg(e.message, true));
$('d-end').onclick = () => API.post('/api/autonomy/stop', {}).then(pollMission).catch((e) => msg(e.message, true));

// ------------------------------------------------------------------ live data
function onConsole(m) {
  if (m.type === 'telemetry') {
    state.tel = m.tel || {}; state.debug = m.debug || {}; onTel();
    curTab && curTab.onTelemetry && curTab.onTelemetry(state.tel, state.debug);
  } else if (m.type === 'tof') { state.tof = m.tof || {}; curTab && curTab.onToF && curTab.onToF(state.tof); }
  else if (m.type === 'hello') {
    const st = m.status || {}; state.tel = st.telemetry || {}; state.debug = st.debug || {}; state.tof = st.tof || {}; onTel();
    curTab && curTab.onToF && curTab.onToF(state.tof);
  } else if (m.type === 'hub') S.hub = m.hub || {};
  else if (m.type === 'params') {
    state.params = Object.fromEntries(m.params.map((p) => [p.key, p])); state.derived = m.derived || {};
    document.dispatchEvent(new CustomEvent('params'));
  } else if (m.type === 'visual_pair') S.vp = m.visual_pair;
  else if (m.type === 'param_msg' || m.type === 'log') {
    msg(m.text, /PE,|ERROR|FAIL/.test(m.text)); document.dispatchEvent(new CustomEvent('logline', { detail: m.text }));
  }
}
function onTel() {
  const t = state.tel;
  if (t.hub) S.hub = t.hub;
  if (engine) {
    engine.setTelemetry({ pitch: Math.max(-25, Math.min(25, t.pitch || 0)), v: t.v_ms || 0 });
    if (!S.slam || S.slam.mode === 'idle') engine.setPose([t.x_m || 0, t.y_m || 0, (t.heading || 0) * D2R + Math.PI / 2]);
  }
  updateMode();
}
function sectorClearance(scan) {                     // scan: [[forward, left], ...] in the robot frame
  const out = SECTORS.map(() => Infinity);
  for (const p of scan || []) {
    const x = -p[1], y = p[0], r = Math.hypot(x, y);
    if (r < 0.09) continue;                           // inside the robot's own outline
    const a = Math.atan2(y, x) / D2R; let best = 0, bd = 999;
    for (let i = 0; i < SECTORS.length; i++) { const d = Math.abs(((a - SECTORS[i].a + 540) % 360) - 180); if (d < bd) { bd = d; best = i; } }
    if (bd <= 22.5) out[best] = Math.min(out[best], r - SECTORS[best].hull);
  }
  return out.map((v) => (v === Infinity ? null : Math.max(0, v)));
}
let sceneVersion = null, sceneAt = 0, sceneBusy = false;
function onSlam(s) {
  S.slam = s;
  S.clear = sectorClearance(s.scan);
  const b = s.bubble || {}, mins = S.clear.filter(fin);
  S.near = !b.stale && fin(b.nearest_m) ? b.nearest_m : (mins.length ? Math.min(...mins) : null);
  if (engine) {
    if (s.mode !== 'idle') engine.setPose(s.pose);
    engine.setScan(s.scan || []);
    engine.setObstacles((s.tof || {}).obstacles, (s.tof || {}).cliffs);
    engine.setRoute(s.plan, s.goal);
    engine.setProximity(S.clear);
    const now = Date.now();
    if (s.map_version !== sceneVersion && !sceneBusy && now - sceneAt > 2500 && !DEMO) {
      sceneBusy = true; sceneAt = now;
      API.get('/slam/api/scene3d').then((sc) => { engine.setScene(sc); sceneVersion = s.map_version; })
        .catch(() => {}).finally(() => { sceneBusy = false; });
    }
  }
  updateMode();
}
function onMap(map) {
  const img = new Image();
  img.onload = () => { S.mapImg = img; S.mapMeta = map; $('sh-mini').classList.add('has'); };
  img.src = 'data:image/png;base64,' + map.png;
}
const wsUrl = (path) => (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + path;
function connectConsole() {
  const ws = new WebSocket(wsUrl('/ws'));
  ws.onopen = () => { S.linkUp = true; msg('connected'); };
  ws.onmessage = (ev) => { try { onConsole(JSON.parse(ev.data)); } catch (e) { console.error(e); } };
  ws.onclose = () => { S.linkUp = false; updateMode(); setTimeout(connectConsole, 1500); };
}
function connectSlam() {
  const ws = new WebSocket(wsUrl('/slam/ws'));
  ws.onmessage = (ev) => { try { const m = JSON.parse(ev.data); if (m.map) onMap(m.map); if (m.state) onSlam(m.state); } catch (e) { console.error(e); } };
  ws.onclose = () => setTimeout(connectSlam, 2000);
}
async function pollMission() {
  try { const r = await API.get('/api/autonomy/mission'); S.mission = r.mission; S.guard = r.bubble; } catch (e) { /* console restarting */ }
}
async function pollPower() { try { S.power = await API.get('/api/power'); } catch (e) { /* console restarting */ } }
async function pollStream() { try { S.stream = await API.get('/api/stream'); } catch (e) { S.stream = null; } }

// ------------------------------------------------------------------ painting
const REASONS = [
  [/supervisor|backgrounded/, 'The Maps page was hidden or lost its connection.'], [/gamepad takeover|lease lost/, 'You took over with the gamepad.'],
  [/feedback lost|telemetry/, 'Lost contact with the controller.'], [/disarmed|fell/, 'The robot stopped balancing.'],
  [/bubble stale/, 'The lidar stopped seeing (lean or lidar off).'], [/boxed in/, 'Boxed in: no room to turn or back out.'],
  [/lost localization/, 'Lost its place on the map.'], [/operator|E-STOP/, 'Stopped by you.'],
];
const human = (r) => { for (const [re, tx] of REASONS) if (re.test(r || '')) return tx; return r || ''; };
function missionLine(m) {
  const i = m.info || {};
  if (i.waiting) return { planning: 'Planning a route', localizing: 'Finding itself on the map', 'finding frontiers': 'Looking for unmapped edges',
    'lidar bubble stale': 'Waiting for the lidar' }[i.waiting] || 'Waiting: ' + i.waiting;
  if (m.kind === 'goto') return fin(i.distance_m) ? fmt(i.distance_m, 1) + ' m to go' : 'On the way';
  const left = fin(i.remaining_s) ? `${Math.floor(i.remaining_s / 60)}:${String(Math.max(0, Math.round(i.remaining_s % 60))).padStart(2, '0')} left` : '';
  if (m.kind === 'roam') return [fmt(i.distance_m, 1) + ' m driven', left].filter(Boolean).join(' · ');
  if (m.kind === 'explore') return [(i.frontiers_reached || 0) + ' edges reached', left].filter(Boolean).join(' · ');
  return '';
}
function pillState() {
  const t = state.tel, b = bubble(), m = S.mission;
  if (!S.linkUp) return ['err', 'i-warn', 'Lost connection to the console. Reconnecting'];
  if (t.stale !== false) return ['warn', 'i-warn', 'No telemetry from the controller'];
  if (m && m.state === 'running') return ['', 'i-nav', [m.name, missionLine(m)].filter(Boolean).join('  ·  ')];
  if (m && m.state === 'paused') return ['warn', 'i-warn', ['Paused', human(m.reason)].filter(Boolean).join('  ·  ')];
  if (armed() && b && !b.stale) {
    const tx = { front_blocked: 'Blocked ahead', rear_blocked: 'Blocked behind', boxed_in: 'Boxed in: no room to move' }[b.state];
    if (tx) return ['err', 'i-warn', tx];
  }
  const vp = S.vp;
  if (vp && vp.enabled && (vp.state === 'triggered' || (vp.hits > 0 && ['watching', 'seen'].includes(vp.state))))
    return ['', 'i-pad', vp.state === 'triggered' ? 'Gamepad recognised: clearing pairings. Hold HOME + A' : `Hold the gamepad to the camera  ·  ${Math.round((vp.progress || 0) * 100)}%`];
  if (S.power.asleep) return ['warn', 'i-pwr', 'Powered down. Wake it from Controls'];
  return null;
}
let placesKey = '';
function paint() {
  const t = state.tel, b = bubble(), m = S.mission, s = S.slam, isArmed = armed();
  // gear, speed, cap, distance
  const gear = m && m.state === 'running' ? 'A' : isArmed ? 'D' : 'P';
  for (const el of document.querySelectorAll('.sh-gear b')) el.classList.toggle('on', el.dataset.g === gear);
  $('sh-spd').textContent = Math.round(Math.abs(t.v_ms || 0) * 100);
  const cap = b && !b.stale ? b.forward_cap : null;
  const showCap = isArmed && fin(cap) && ['limiting', 'front_blocked', 'boxed_in'].includes(b.state);   // only while the bubble really limits
  $('sh-max').hidden = !showCap; if (showCap) $('sh-max').firstElementChild.textContent = Math.round(cap * 100);
  $('sh-dist').firstElementChild.textContent = fin(S.near) ? Math.round(S.near * 100) : '-';
  // status bar
  const startup = { WAIT_HOST: 'Waiting for the host', INITIALIZING: 'Starting up', CALIBRATING: 'Calibrating the IMU', ERROR: 'Startup error', FLASHING: 'Updating firmware' }[t.startup_state];
  const arm = !S.linkUp ? 'Offline' : t.stale !== false ? 'No link' : startup || (t.self_righting ? 'Self-righting' : t.balancing ? 'Balancing' : S.power.asleep || t.asleep ? 'Powered down' : 'Disarmed');
  $('sh-arm').textContent = arm;
  $('sh-lock').firstElementChild.setAttribute('href', isArmed ? '#i-open' : '#i-lock');
  const hz = t.rx_hz || 0, dot = !online() ? '' : hz < 30 ? 'warn' : 'ok';
  $('sh-link').innerHTML = `<i class="sh-dot ${dot}"></i> Link ${online() ? Math.round(hz) + ' Hz' : 'down'}`;
  const scanning = s && s.lidar && s.lidar.state === 'scanning';
  $('sh-lidar').textContent = s ? 'Lidar ' + (scanning ? Math.round((s.rates || {}).scan || 0) + ' Hz' : (s.lidar || {}).state || 'off') : '';
  // pill
  const p = pillState(), pill = $('sh-pill');
  pill.hidden = !p;
  if (p) { pill.className = p[0]; pill.querySelector('use').setAttribute('href', '#' + p[1]); pill.querySelector('.tx').textContent = p[2]; }
  // callouts
  const set = (k, v) => { document.querySelector(`.sh-callout[data-k="${k}"] b`).textContent = v; };
  set('lidar', s ? (scanning ? Math.round((s.rates || {}).scan || 0) + ' Hz' : (s.lidar || {}).state || 'off') : '-');
  const st = S.stream;
  set('camera', st ? (st.running ? (fin(st.fps) ? Math.round(st.fps) + ' fps' : 'live') : 'off') : '-');
  const temps = [S.hub.temp_l, S.hub.temp_r].filter(fin);
  set('wheel', temps.length ? Math.round(Math.max(...temps)) + ' °C' : '-');
  // parked cards
  const pw = $('q-power'); pw.classList.toggle('on', !!S.power.asleep); pw.disabled = !!S.power.busy;
  pw.lastElementChild.textContent = S.power.busy ? (S.power.asleep ? 'Powering down' : 'Waking') : S.power.asleep ? 'Wake up' : 'Power down';
  const labels = (s && s.labels) || [], key = labels.map((l) => l.name).join('|');
  if (key !== placesKey) {
    placesKey = key; const box = $('q-places'); box.innerHTML = '';
    if (!labels.length) box.innerHTML = '<span class="none">Places you name on the map appear here</span>';
    for (const l of labels.slice(0, 6)) { const c = document.createElement('button'); c.textContent = l.name; c.title = 'Open Maps to drive there'; c.onclick = () => openApp('maps'); box.appendChild(c); }
  }
  // autopilot card
  const active = missionActive();
  $('d-pause').hidden = !(m && m.state === 'running'); $('d-end').hidden = !active; $('d-open').hidden = !!active;
  $('d-open').textContent = m && m.state === 'paused' ? 'Resume in Maps' : 'Open Maps';
  if (m && m.state === 'paused') $('d-open').hidden = false;
  $('d-mt').textContent = active ? (m.state === 'paused' ? 'Paused · ' : '') + (m.name || 'Mission') : 'No mission';
  $('d-ms').textContent = active ? (m.state === 'paused' ? human(m.reason) : missionLine(m)) : 'Pick a place in Maps and press Start. This page can pause or end it.';
  const prog = $('d-prog'), dist = m && m.info ? m.info.distance_m : null;
  if (m && m.state === 'running' && m.kind === 'goto' && fin(dist)) {
    if (paint.id !== m.id) { paint.id = m.id; paint.start = Math.max(dist, 0.01); }
    prog.hidden = false; prog.firstElementChild.style.width = Math.round(100 * Math.max(0, Math.min(1, 1 - dist / paint.start))) + '%';
  } else prog.hidden = true;
}

// labels that hang off the robot: positioned every rendered frame
const LEAD = { lidar: [0, -118], camera: [-1, 0], wheel: [1, 0] };
function placeLabels(view) {
  const a = engine.anchors(), svg = $('sh-leaders'), w = $('sh-viz').clientWidth, alpha = Math.max(0, (view.studio - 0.6) / 0.4);
  let lines = '';
  for (const el of document.querySelectorAll('.sh-callout')) {
    const p = a[el.dataset.k], d = LEAD[el.dataset.k];
    if (!p || alpha <= 0) { el.style.opacity = 0; continue; }
    const reach = Math.min(200, w * 0.2), ex = p[0] + d[0] * reach, ey = p[1] + d[1];
    let x, y;
    if (d[1]) { x = ex + 9; y = ey - 2; } else if (d[0] < 0) { x = ex - el.offsetWidth - 6; y = ey - el.offsetHeight / 2; } else { x = ex + 6; y = ey - el.offsetHeight / 2; }
    el.style.opacity = alpha; el.style.transform = `translate(${x.toFixed(1)}px,${y.toFixed(1)}px)`;
    lines += `<line x1="${p[0].toFixed(1)}" y1="${p[1].toFixed(1)}" x2="${ex.toFixed(1)}" y2="${ey.toFixed(1)}"/><circle cx="${p[0].toFixed(1)}" cy="${p[1].toFixed(1)}" r="2.6"/>`;
  }
  svg.innerHTML = lines; svg.style.opacity = alpha;
  const g = $('sh-goal'), goal = S.slam && S.slam.goal;
  if (a.goal && goal && view.world > 0.5) { g.hidden = false; g.textContent = goal.name || goal.label || 'Goal';
    g.style.transform = `translate(${(a.goal[0] - g.offsetWidth / 2).toFixed(1)}px,${(a.goal[1] - g.offsetHeight - 14).toFixed(1)}px)`; g.style.opacity = view.world; }
  else g.hidden = true;
}

// 2D panels of the Drive app
function fit(cv) {
  const dpr = Math.min(2, window.devicePixelRatio || 1), w = cv.clientWidth, h = cv.clientHeight;
  if (!w || !h) return null;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const g = cv.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, h); return [g, w, h];
}
function arrow(g, x, y, th, s) {                      // th: heading from +x, screen y is flipped
  g.save(); g.translate(x, y); g.rotate(Math.PI / 2 - th); g.fillStyle = '#ff4d4d';
  g.beginPath(); g.moveTo(0, -14 * s); g.lineTo(10 * s, 11 * s); g.lineTo(0, 5 * s); g.lineTo(-10 * s, 11 * s); g.closePath(); g.fill(); g.restore();
}
function drawMap(cv, scale, big) {
  const f = fit(cv); if (!f) return; const [g, w, h] = f, s = S.slam;
  const pose = s && s.mode !== 'idle' ? s.pose : [state.tel.x_m || 0, state.tel.y_m || 0, (state.tel.heading || 0) * D2R + Math.PI / 2];
  let cx = pose[0], cy = pose[1];
  if (big && s && s.goal && fin(s.goal.x)) { cx = (cx + s.goal.x) / 2; cy = (cy + s.goal.y) / 2;
    scale = Math.max(40, Math.min(scale, 0.7 * Math.min(w, h) / Math.max(0.5, Math.hypot(s.goal.x - pose[0], s.goal.y - pose[1])))); }
  const X = (x) => w / 2 + (x - cx) * scale, Y = (y) => h / 2 - (y - cy) * scale;
  if (big) { g.fillStyle = '#181411'; g.fillRect(0, 0, w, h); }
  if (S.mapImg && S.mapMeta) {
    const mm = S.mapMeta; g.imageSmoothingEnabled = scale < 40; g.globalAlpha = big ? 0.95 : 0.8;
    g.drawImage(S.mapImg, X(mm.x0), Y(mm.y0 + mm.h), mm.w * scale, mm.h * scale); g.globalAlpha = 1;
  } else if (s) {                                      // no map yet: show what the lidar sees right now
    const c = Math.cos(pose[2]), sn = Math.sin(pose[2]); g.fillStyle = '#8f867c';
    for (const p of s.scan || []) g.fillRect(X(pose[0] + c * p[0] - sn * p[1]) - 1.5, Y(pose[1] + sn * p[0] + c * p[1]) - 1.5, 3, 3);
  }
  if (s && s.plan && s.plan.length > 1) {
    g.strokeStyle = '#38bdf8'; g.lineWidth = big ? 5 : 3; g.lineCap = g.lineJoin = 'round'; g.beginPath();
    s.plan.forEach((p, i) => (i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1])))); g.stroke();
  }
  if (big && s) {
    g.font = '500 12.5px Inter, "Segoe UI", sans-serif'; g.textAlign = 'center'; g.fillStyle = '#aba196';
    for (const l of s.labels || []) g.fillText(l.name, X(l.x), Y(l.y) - 9);
    g.fillStyle = '#f08a4b'; for (const l of s.labels || []) { g.beginPath(); g.arc(X(l.x), Y(l.y), 3.5, 0, 7); g.fill(); }
    if (s.goal && fin(s.goal.x)) { g.fillStyle = '#38bdf8'; g.beginPath(); g.arc(X(s.goal.x), Y(s.goal.y), 7, 0, 7); g.fill();
      g.fillStyle = '#06202e'; g.beginPath(); g.arc(X(s.goal.x), Y(s.goal.y), 2.6, 0, 7); g.fill(); }
  }
  arrow(g, X(pose[0]), Y(pose[1]), pose[2], big ? 1 : 0.8);
  if (big) $('d-mapnote').textContent = S.mapImg ? '' : s ? 'No map loaded. Live lidar shown; open Maps to build one.' : 'Waiting for the lidar';
}
function drawRadar(cv) {
  const f = fit(cv); if (!f) return; const [g, w, h] = f, s = S.slam, cx = w / 2, cy = h * 0.52, k = Math.min(w, h) * 0.27;
  g.strokeStyle = '#2f2823'; g.lineWidth = 1;
  for (let m = 0.5; m <= 2; m += 0.5) { g.beginPath(); g.arc(cx, cy, m * k, 0, 7); g.stroke(); }
  g.fillStyle = '#60a5fa';
  for (const p of (s && s.scan) || []) g.fillRect(cx - p[1] * k - 1.3, cy - p[0] * k - 1.3, 2.6, 2.6);     // forward is up
  arrow(g, cx, cy, Math.PI / 2, 0.7);
}
function drawDepth(cv) {
  const f = fit(cv); if (!f) return; const [g, w, h] = f, t = state.tof || {}, d = t.distance_mm || [], ok = t.valid || [];
  const n = 8, s = Math.min(w, h) / n, ox = (w - s * n) / 2, oy = (h - s * n) / 2;
  for (let i = 0; i < 64; i++) {
    const good = !t.stale && ok[i] && fin(d[i]), near = good ? Math.max(0, Math.min(1, (1500 - d[i]) / 1400)) : 0;
    g.fillStyle = good ? `rgba(240,138,75,${(0.12 + near * 0.85).toFixed(3)})` : 'rgba(111,102,93,.14)';
    g.beginPath(); g.roundRect(ox + (i % 8) * s + 1.5, oy + Math.floor(i / 8) * s + 1.5, s - 3, s - 3, 3); g.fill();
  }
}
function drawPanels() {
  if (body.dataset.mode === 'parked' && body.dataset.pane === 'closed') drawMap($('sh-mini-cv'), 26, false);
  if (app !== 'drive' || body.dataset.pane !== 'open') return;
  if (mode === 'park') { drawRadar($('sh-radar')); drawDepth($('sh-depth')); } else drawMap($('sh-map'), 96, true);
}

// ------------------------------------------------------------------ boot
let toastT = 0;
document.addEventListener('consolemsg', () => { const t = $('sh-toast'); t.classList.add('show'); clearTimeout(toastT); toastT = setTimeout(() => t.classList.remove('show'), 4000); });
document.addEventListener('params', () => { if (curTab && curHost && !LIVE_TABS.includes(curTab.id)) showTab(curTab.id, curHost); });
const clock = () => { $('sh-clock').textContent = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }); };

(async function boot() {
  clock(); setInterval(clock, 20000);
  try {
    const slow = Math.max(1, +Q.get('slow') || 1);                   // ?slow=N stretches the view change, for filming it
    if (slow > 1) $('sh').style.setProperty('--s-move', (1.1 * slow) + 's cubic-bezier(.65,0,.35,1)');
    engine = await createEngine($('sh-gl'), $('sh-viz'), { slow });
    engine.onFrame(placeLabels);
  } catch (e) { console.error('3D view failed to start', e); $('sh-fallback').hidden = false; $('sh-gl').hidden = true; }
  for (const f of TAB_FILES) {
    try { (await import(`/ui/tabs/${f}.js`)).default(registerTab); } catch (e) { console.error('tab failed:', f, e); }
  }
  buildControlsNav();
  if (DEMO) {
    $('sh-demo').hidden = false; S.linkUp = true; $('d-feednote').textContent = 'Live camera (off in demo)';
    (await import('./demo.js')).startDemo({ console: onConsole, slam: onSlam, mission: (r) => { S.mission = r.mission; S.guard = r.bubble; },
      scene: (sc) => engine && engine.setScene(sc), map: (img, meta) => { S.mapImg = img; S.mapMeta = meta; $('sh-mini').classList.add('has'); },
      stream: (st) => { S.stream = st; } }, Q);
  } else {
    connectConsole(); connectSlam();
    pollMission(); setInterval(pollMission, 1000);
    pollPower(); setInterval(pollPower, 3000);
    pollStream(); setInterval(pollStream, 5000);
    refreshParams().catch(() => {});
    setInterval(() => { if (!Object.keys(state.params).length) refreshParams().catch(() => {}); }, 4000);
  }
  if (['parked', 'drive', 'park'].includes(Q.get('view'))) { held = Q.get('view'); $('sh-view').classList.add('held'); $('sh-view').querySelector('span').textContent = VIEW_NAME[held]; }
  updateMode(true);
  if (Q.get('app') && document.querySelector(`.sh-app[data-app="${Q.get('app')}"]`)) openApp(Q.get('app'));
  setInterval(paint, 200); setInterval(drawPanels, 250); paint();
  window.__shell = { engine, S, state, mode: () => mode };
  window.__ready = true;
})();
