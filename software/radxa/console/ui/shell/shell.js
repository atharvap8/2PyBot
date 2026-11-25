/* shell/shell.js - the console shell.
 *
 * Left: the live 3D view (viz/engine.js) with its instruments. Right: an app pane
 * (Drive, Maps, Camera, ML, Controls, Terminal, Deploy). Bottom: the dock.
 *
 * The 3D view picks its own mode from live state, like a car does from its gear:
 *   parked  disarmed                         -> studio shot, live status labels
 *   drive   balancing (gamepad or a mission) -> chase camera over the map
 *   park    balancing, slow, something close -> top-down with proximity arcs
 *   system  the System button                -> front view with every part as a block
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
const D2R = Math.PI / 180, TAU = Math.PI * 2;
const body = document.body;
const S = { slam: null, mission: null, guard: null, power: { asleep: false }, hub: {}, vp: null, sys: null,
            mapImg: null, mapMeta: null, linkUp: false, near: null, clear: null, note: null, pausedAt: 0 };
let engine = null, mode = 'parked', held = null, sysView = false, modeAt = 0, app = null, autoOpened = false, wasArmed = false, hot = null;

// ------------------------------------------------------------------ helpers
const fin = (v) => typeof v === 'number' && isFinite(v);
const fmt = (v, d = 1) => (fin(v) ? v.toFixed(d) : '-');
const armed = () => !!state.tel.balancing && !state.tel.stale && S.linkUp;
const online = () => S.linkUp && state.tel.stale === false;
const missionActive = () => S.mission && (S.mission.state === 'running' || S.mission.state === 'paused');
const bubble = () => (S.slam && S.slam.bubble) || S.guard || null;
const angDiff = (a, b) => ((b - a + Math.PI) % TAU + TAU) % TAU - Math.PI;
const node = (k) => (S.sys && S.sys.nodes && S.sys.nodes[k]) || null;
const metric = (n, label) => { const m = n && (n.metrics || []).find((x) => x[0] === label); return m && m[1] && m[1] !== '–' ? String(m[1]) : null; };
const tone = (n) => (!n ? 'off' : n.status === 'ok' ? 'ok' : n.status === 'warn' ? 'warn' : n.status === 'nofb' ? 'off' : 'err');

// ------------------------------------------------------------------ view mode
function wantedMode() {
  if (sysView) return 'system';
  if (held) return held;
  if (!armed()) return 'parked';
  const v = Math.abs(state.tel.v_ms || 0), n = S.near;
  if (mode === 'park') return fin(n) && n < 0.42 && v < 0.2 ? 'park' : 'drive';
  return fin(n) && n < 0.3 && v < 0.12 ? 'park' : 'drive';
}
function updateMode(force) {
  const isArmed = armed();
  if (isArmed !== wasArmed) {                         // arming or disarming always takes the view back: balancing -> drive, disarmed -> parked
    wasArmed = isArmed; held = null; setHeldLabel();
    if (isArmed && sysView) leaveSystem();
    force = true;
  }
  const want = wantedMode(), now = performance.now();
  if (want === mode && !force) return;
  // do not flap between drive and park; every other change is immediate
  if (!force && (want === 'drive' || want === 'park') && (mode === 'drive' || mode === 'park') && now - modeAt < 1500) return;
  mode = want; modeAt = now; body.dataset.mode = mode;
  engine && engine.setMode(mode);
  if ((mode === 'drive' || mode === 'park') && body.dataset.pane === 'closed') { openApp('drive'); autoOpened = true; }
  else if (mode === 'parked' && autoOpened && app === 'drive') closePane();
  if (mode !== 'system') setHot(null);
  paint();
}
const VIEW_CYCLE = [null, 'parked', 'drive', 'park'], VIEW_NAME = { null: 'Auto', parked: 'Parked', drive: 'Drive', park: 'Close' };
function setHeldLabel() { $('sh-view').classList.toggle('held', !!held); $('sh-view').querySelector('span').textContent = VIEW_NAME[held]; }
$('sh-view').onclick = () => {
  if (sysView) leaveSystem();
  held = VIEW_CYCLE[(VIEW_CYCLE.indexOf(held) + 1) % VIEW_CYCLE.length]; setHeldLabel(); updateMode(true);
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

function selectDock(id) { for (const b of document.querySelectorAll('#sh-dock [data-open]')) b.classList.toggle('sel', b.dataset.open === id); }
function openApp(id) {
  if (id === 'system') { sysView ? leaveSystem() : enterSystem(); return; }
  if (sysView) leaveSystem();
  if (id === app && body.dataset.pane === 'open') { if (id === 'drive' && mode === 'parked') closePane(); return; }
  autoOpened = false;
  if (app === 'camera' || app === 'controls') leaveTab();
  app = id;
  for (const el of document.querySelectorAll('.sh-app')) el.classList.toggle('on', el.dataset.app === id);
  selectDock(id);
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
  selectDock(sysView ? 'system' : null);
}
function enterSystem() { closePane(); sysView = true; selectDock('system'); pollSys(); updateMode(true); }
function leaveSystem() { sysView = false; setHot(null); selectDock(app); updateMode(true); }
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
// The robot's pose comes from the robot 20 times a second: the last pose matched against
// the map, carried forward by the odometry measured since (so it is current, and it moves
// smoothly between matches). The engine eases towards it. The robot also counts the times
// it moved the pose in one step (a map was loaded, or it found itself somewhere else after
// checking twice); only then does the view cut instead of gliding across the map.
const live = { pose: [0, 0, Math.PI / 2], at: 0, jumps: null };
const mapPose = () => live.pose;
function onPose(p, jumps) {
  if (!p || !fin(p[0]) || !fin(p[1]) || !fin(p[2])) return;
  const cut = live.jumps === null || (jumps != null && jumps !== live.jumps);
  live.pose = p; live.at = performance.now(); if (jumps != null) live.jumps = jumps;
  if (engine) { engine.setPose(p); if (cut) engine.snapPose(); }
}
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
  // Raw controller lines go to the Console log only. They are never shown over the view.
  else if (m.type === 'param_msg' || m.type === 'log') document.dispatchEvent(new CustomEvent('logline', { detail: m.text }));
}
let speed = 0, lastTel = 0;
function onTel() {
  const t = state.tel, now = performance.now(), dt = Math.min(0.2, (now - lastTel) / 1000 || 0.02); lastTel = now;
  if (t.hub) S.hub = t.hub;
  speed += (Math.abs(t.v_ms || 0) - speed) * Math.min(1, dt / 0.18);      // steady digits, no flicker between neighbours
  if (engine) engine.setTelemetry({ pitch: Math.max(-25, Math.min(25, t.pitch || 0)), v: t.v_ms || 0, armed: armed() });
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
function routeFromRobot(plan) {                      // the planner's route starts at its nearest waypoint; draw it from the robot
  if (!plan || plan.length < 2) return plan;
  const p = mapPose(); let i = 0;
  while (i < plan.length - 2 && Math.hypot(plan[i][0] - p[0], plan[i][1] - p[1]) < 0.14) i++;
  return [[p[0], p[1]], ...plan.slice(i)];
}
let sceneVersion = null, sceneAt = 0, sceneBusy = false, vectorOk = true;
function fetchScene(version) {                       // version: map name and revision, so another map is always a new scene
  sceneBusy = true; sceneAt = Date.now();
  const cells = () => API.get('/slam/api/scene3d').then((sc) => engine.setScene({ ...sc, version }));
  (vectorOk ? API.get('/slam/api/scene_vector').then((v) => engine.setVector({ ...v, version })).catch(() => { vectorOk = false; return cells(); }) : cells())
    .then(() => { sceneVersion = version; }).catch(() => {}).finally(() => { sceneBusy = false; });
}
function onSlam(s) {
  S.slam = s;
  S.clear = sectorClearance(s.scan);
  const b = s.bubble || {}, mins = S.clear.filter(fin);
  S.near = !b.stale && fin(b.nearest_m) ? b.nearest_m : (mins.length ? Math.min(...mins) : null);
  if (performance.now() - live.at > 600) onPose(s.live_pose || s.pose, s.jumps);      // no fast pose stream: 5 Hz is still a pose
  if (s.map_version === 0 && S.mapImg) { S.mapImg = S.mapMeta = null; $('sh-mini').classList.remove('has'); }   // map unloaded
  if (engine) {
    engine.setScan(s.scan || []);
    engine.setObstacles((s.tof || {}).obstacles, (s.tof || {}).cliffs);
    engine.setRoute(routeFromRobot(s.plan), s.goal);
    engine.setProximity(S.clear);
    const scene = (s.map_name || '') + ':' + s.map_version;
    if (scene !== sceneVersion && !sceneBusy && Date.now() - sceneAt > 2500 && !DEMO) fetchScene(scene);
  }
  updateMode();
}
function onMap(map) {                                 // recolour the grey map image for the dark console: unknown clear, walls light
  const img = new Image();
  img.onload = () => {
    const c = document.createElement('canvas'); c.width = img.width; c.height = img.height;
    const g = c.getContext('2d'); g.drawImage(img, 0, 0);
    const d = g.getImageData(0, 0, c.width, c.height), px = d.data;
    for (let i = 0; i < px.length; i += 4) {
      const v = px[i];                                // 0 occupied ... 128 unknown ... 255 free
      if (v < 100) { px[i] = 176; px[i + 1] = 166; px[i + 2] = 154; px[i + 3] = Math.round(255 * Math.min(1, (128 - v) / 70)); }
      else if (v > 150) { px[i] = px[i + 1] = px[i + 2] = 255; px[i + 3] = Math.round(26 * Math.min(1, (v - 128) / 90)); }
      else px[i + 3] = 0;
    }
    g.putImageData(d, 0, 0); S.mapImg = c; S.mapMeta = map; $('sh-mini').classList.add('has');
  };
  img.src = map.png.startsWith('data:') ? map.png : 'data:image/png;base64,' + map.png;
}
const wsUrl = (path) => (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + path;
function connectConsole() {
  const ws = new WebSocket(wsUrl('/ws'));
  ws.onopen = () => { S.linkUp = true; };
  ws.onmessage = (ev) => { try { onConsole(JSON.parse(ev.data)); } catch (e) { console.error(e); } };
  ws.onclose = () => { S.linkUp = false; updateMode(); setTimeout(connectConsole, 1500); };
}
function connectSlam() {
  const ws = new WebSocket(wsUrl('/slam/ws?pose=1'));
  ws.onmessage = (ev) => { try { const m = JSON.parse(ev.data);
    if (m.type === 'pose') return onPose(m.pose, m.jumps);
    if (m.map) onMap(m.map); if (m.state) onSlam(m.state); } catch (e) { console.error(e); } };
  ws.onclose = () => setTimeout(connectSlam, 2000);
}
function onMission(r) {
  const was = S.mission && S.mission.state;
  S.mission = r.mission; S.guard = r.bubble;
  if (r.mission && r.mission.state === 'paused' && was !== 'paused') S.pausedAt = performance.now();
}
async function pollMission() { try { onMission(await API.get('/api/autonomy/mission')); } catch (e) { /* console restarting */ } }
async function pollPower() { try { S.power = await API.get('/api/power'); } catch (e) { /* console restarting */ } }
async function pollSys() {                            // part health: only while its labels or blocks are on screen
  if (DEMO || document.hidden || !(mode === 'parked' || mode === 'system')) return;
  try { S.sys = await API.get('/sysmap/api/snapshot'); paintParts(); } catch (e) { /* console restarting */ }
}

// ------------------------------------------------------------------ painting
const REASONS = [
  [/supervisor|backgrounded/, 'The Maps page was hidden or lost its connection'], [/gamepad takeover/, 'You took over with the gamepad'],
  [/lease lost|lease/, 'The controller ended the drive'], [/feedback lost|telemetry/, 'The controller stopped answering for over 2 s'],
  [/disarmed|fell/, 'The robot stopped balancing'], [/bubble stale/, 'The lidar stopped seeing (lean or lidar off)'],
  [/boxed in/, 'Boxed in: no room to turn or back out'], [/lost localization/, 'Lost its place on the map'], [/operator|E-STOP/, 'Stopped by you'],
];
const human = (r) => { for (const [re, tx] of REASONS) if (re.test(r || '')) return tx; return r || ''; };
function missionLine(m) {
  const i = m.info || {};
  if (i.waiting) return { planning: 'Planning a route', replanning: 'Finding a way round', localizing: 'Finding itself on the map',
    'finding frontiers': 'Looking for unmapped edges', 'lidar bubble stale': 'Waiting for the lidar', 'controller link': 'Waiting for the controller',
    gamepad: 'You are driving. Autopilot resumes when you let go' }[i.waiting] || 'Waiting: ' + i.waiting;
  if (m.kind === 'goto') return fin(i.distance_m) ? fmt(i.distance_m, 1) + ' m to go' : 'On the way';
  const left = fin(i.remaining_s) ? `${Math.floor(i.remaining_s / 60)}:${String(Math.max(0, Math.round(i.remaining_s % 60))).padStart(2, '0')} left` : '';
  if (m.kind === 'roam') return [fmt(i.distance_m, 1) + ' m driven', left].filter(Boolean).join(' · ');
  if (m.kind === 'explore') return [(i.frontiers_reached || 0) + ' edges reached', left].filter(Boolean).join(' · ');
  return '';
}
function pillState() {
  const t = state.tel, b = bubble(), m = S.mission, now = performance.now();
  if (!S.linkUp) return ['err', 'i-warn', 'Lost connection to the console. Reconnecting'];
  if (t.stale !== false) return ['warn', 'i-warn', 'No telemetry from the controller'];
  if (S.note && now < S.note.until) return [S.note.bad ? 'err' : '', S.note.bad ? 'i-warn' : 'i-check', S.note.text];
  if (m && m.state === 'running') return ['', 'i-nav', [m.name, missionLine(m)].filter(Boolean).join('  ·  ')];
  if (m && m.state === 'paused' && now - S.pausedAt < 9000) return ['warn', 'i-warn', ['Autopilot paused', human(m.reason)].filter(Boolean).join('  ·  ')];
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
  const gear = m && m.state === 'running' && !m.yielded ? 'A' : isArmed ? 'D' : 'P';
  for (const el of document.querySelectorAll('.sh-gear b')) el.classList.toggle('on', el.dataset.g === gear);
  $('sh-spd').textContent = Math.round(speed * 100);
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
  // one notification surface: the pill
  const p = pillState(), pill = $('sh-pill');
  pill.hidden = !p;
  if (p) { pill.className = p[0]; pill.querySelector('use').setAttribute('href', '#' + p[1]); pill.querySelector('.tx').textContent = p[2]; }
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
  $('d-pause').hidden = !(m && m.state === 'running'); $('d-end').hidden = !active;
  $('d-open').hidden = !!(m && m.state === 'running'); $('d-open').textContent = m && m.state === 'paused' ? 'Resume in Maps' : 'Open Maps';
  $('d-mt').textContent = active ? (m.state === 'paused' ? 'Paused · ' : '') + (m.name || 'Mission') : 'No mission';
  $('d-ms').textContent = active ? (m.state === 'paused' ? human(m.reason) : missionLine(m)) : 'Pick a place in Maps and press Start. This page can pause or end it.';
  const prog = $('d-prog'), dist = m && m.info ? m.info.distance_m : null;
  if (m && m.state === 'running' && m.kind === 'goto' && fin(dist)) {
    if (paint.id !== m.id) { paint.id = m.id; paint.start = Math.max(dist, 0.01); }
    prog.hidden = false; prog.firstElementChild.style.width = Math.round(100 * Math.max(0, Math.min(1, 1 - dist / paint.start))) + '%';
  } else prog.hidden = true;
}

// ---- part status: the labels on the parked robot and the blocks of the system view
// The robot faces you in both studio views, so its left side is on your right.
const LABELS = [                                       // sysmap node, anchor on the model, side, title, metrics to show
  ['camera', 'camera', 'l', 'Camera', ['Frame rate']], ['tof', 'tof', 'l', 'Depth sensor', ['Nearest']],
  ['esp32', 'esp32', 'l', 'Controller', ['State', 'Loop max']], ['wheel_r', 'wheel_r', 'l', 'Right motor', ['Temp']],
  ['lidar', 'lidar', 'r', 'Lidar', ['Scan rate']], ['radxa', 'host', 'r', 'Radxa', ['CPU temp', 'Load']], ['wheel_l', 'wheel_l', 'r', 'Left motor', ['Temp']],
];
const BLOCKS = {                                       // system view: column (top to bottom), anchor on the model, title, metrics
  lidar: ['l', 'lidar', 'Lidar', ['Scan rate', 'Points']], camera: ['l', 'camera', 'Camera', ['Frame rate', 'Capture to publish']],
  radxa: ['l', 'radxa', 'Radxa host', ['CPU temp', 'Load']], clients: ['l', 'radxa', 'Phones and browsers', ['Hotspot', 'Cockpit clients']],
  imu: ['l', 'imu', 'IMU', ['Pitch', 'Accel']],
  lights: ['r', 'lights', 'LED ring', ['Blue LED should be']], tof: ['r', 'tof', 'Depth sensor', ['Nearest', 'Valid zones']],
  compass: ['r', 'compass', 'Compass', ['Heading']], esp32: ['r', 'esp32', 'ESP32 controller', ['State', 'Loop max']],
  gamepad: ['r', 'esp32', 'Gamepad', ['Controller', 'Speed mode']],
  wheel_r: ['b', 'wheel_r', 'Right motor', ['Temp', 'Current']], payload: ['b', 'payload', 'Camera payload', ['Torch']],
  wheel_l: ['b', 'wheel_l', 'Left motor', ['Temp', 'Current']],
};
const valueOf = (n, metrics) => {
  if (!n) return '-';
  if (n.status === 'warn' || n.status === 'err' || n.status === 'stale') return n.reason || n.status;
  const v = metrics.map((l) => metric(n, l)).filter(Boolean);
  return v.length ? v.map((x, i) => (i && /^[\d.]+$/.test(x) ? metrics[i].toLowerCase() + ' ' + x : x)).join(' · ') : n.reason || '-';
};
function buildParts() {
  const lab = $('sh-labels');
  for (const [k, , side, title] of LABELS) lab.insertAdjacentHTML('beforeend',
    `<div class="sh-callout ${side}" data-k="${k}"><i class="sh-dot"></i><u>${title}</u><b>-</b></div>`);
  for (const [k, [col, , title]] of Object.entries(BLOCKS)) {
    const el = document.createElement('button'); el.className = 'sys-b'; el.dataset.k = k;
    el.innerHTML = `<i class="sh-dot"></i><u>${title}</u><b>-</b>`;
    let touch = false;                                 // a mouse highlights by hovering, a finger by tapping
    el.onpointerenter = (e) => { if (e.pointerType === 'mouse') setHot(k); };
    el.onpointerleave = (e) => { if (e.pointerType === 'mouse') setHot(null); };
    el.onpointerdown = (e) => { touch = e.pointerType !== 'mouse'; };
    el.onclick = () => { if (touch) setHot(hot === k ? null : k); };
    el.onfocus = () => { if (!touch) setHot(k); }; el.onblur = () => { if (!touch) setHot(null); };
    $('sys-' + col).appendChild(el);
  }
}
function setHot(k) {
  if (k === hot) return; hot = k;
  if (k) body.dataset.hot = k; else delete body.dataset.hot;
  for (const el of document.querySelectorAll('.sys-b')) el.classList.toggle('hot', el.dataset.k === k);
  engine && engine.highlight(k);
}
function paintParts() {
  for (const [k, , , , metrics] of LABELS) {
    const el = document.querySelector(`.sh-callout[data-k="${k}"]`), n = node(k);
    el.querySelector('i').className = 'sh-dot ' + tone(n); el.querySelector('b').textContent = valueOf(n, metrics);
  }
  for (const [k, [, , , metrics]] of Object.entries(BLOCKS)) {
    const el = document.querySelector(`.sys-b[data-k="${k}"]`), n = node(k);
    el.querySelector('i').className = 'sh-dot ' + tone(n); el.querySelector('b').textContent = valueOf(n, metrics);
    el.classList.toggle('bad', !!n && (n.status === 'warn' || n.status === 'err'));
  }
  const sum = (S.sys && S.sys.summary) || {}, bad = [...(sum.errors || []), ...(sum.warnings || [])];
  $('sys-sum').textContent = !S.sys ? 'Reading the robot' : bad.length ? `${bad.length} need${bad.length === 1 ? 's' : ''} attention: ` + bad.map((k) => (BLOCKS[k] || [0, 0, k])[2]).join(', ') : 'Every part is healthy';
  $('sys-sum').className = bad.length ? 'warn' : '';
}

// labels and blocks are tied to points on the 3D model, so they are placed every rendered frame
function placeLabels(view) {
  const a = engine.anchors(), viz = $('sh-viz'), W = viz.clientWidth, H = viz.clientHeight, svg = $('sh-leaders');
  let lines = '';
  const dotAt = (p) => `<circle cx="${p[0].toFixed(1)}" cy="${p[1].toFixed(1)}" r="2.6"/>`;
  // parked: two tidy columns beside the robot, each joined to its part
  const alpha = mode === 'parked' && W >= 720 ? Math.max(0, (view.studio - 0.6) / 0.4) : 0;
  const reach = Math.max(230, Math.min(330, W * 0.21)), top = 66, bottom = H - (body.dataset.pane === 'closed' ? 226 : 40);
  for (const side of ['l', 'r']) {
    const items = LABELS.filter((l) => l[2] === side).map((l) => ({ el: document.querySelector(`.sh-callout[data-k="${l[0]}"]`), p: a[l[1]] }))
      .filter((it) => it.p).sort((p, q) => p.p[1] - q.p[1]);
    let y = top;
    for (const it of items) { it.y = Math.max(y, it.p[1] - 16); y = it.y + 56; }
    for (let i = items.length - 1, lim = bottom; i >= 0; i--) { items[i].y = Math.min(items[i].y, lim - 44); lim = items[i].y - 12; }
    for (const it of items) {
      const el = it.el, x = side === 'l' ? W / 2 - reach - el.offsetWidth : W / 2 + reach, ly = it.y + el.offsetHeight / 2;
      el.style.opacity = alpha; el.style.transform = `translate(${x.toFixed(1)}px,${it.y.toFixed(1)}px)`;
      if (alpha <= 0) continue;
      const ex = side === 'l' ? W / 2 - reach + 10 : W / 2 + reach - 10, kx = it.p[0] + (side === 'l' ? -34 : 34);
      const bend = side === 'l' ? Math.max(ex, Math.min(kx, it.p[0])) : Math.min(ex, Math.max(kx, it.p[0]));
      lines += `<polyline points="${ex.toFixed(1)},${ly.toFixed(1)} ${bend.toFixed(1)},${ly.toFixed(1)} ${it.p[0].toFixed(1)},${it.p[1].toFixed(1)}"/>` + dotAt(it.p);
    }
  }
  // system: a wire from every block to its part; the hovered one lights up
  if (mode === 'system' && view.studio > 0.7) {
    const box = viz.getBoundingClientRect(), narrow = W < 720;      // a phone stacks the blocks under the robot and wires only the chosen one
    for (const [k, [col, anchor]] of Object.entries(BLOCKS)) {
      const el = document.querySelector(`.sys-b[data-k="${k}"]`), p = a[anchor]; if (!p || !el.offsetWidth || (narrow && hot !== k)) continue;
      const r = el.getBoundingClientRect(), cls = hot === k ? ' class="hot"' : '';
      let sx, sy, d;
      if (col === 'b' || narrow) { sx = r.left + r.width / 2 - box.left; sy = r.top - box.top; d = `M${sx} ${sy} C${sx} ${sy - 46},${p[0]} ${p[1] + 60},${p[0]} ${p[1]}`; }
      else { sx = (col === 'l' ? r.right : r.left) - box.left; sy = r.top + r.height / 2 - box.top; const mx = (sx + p[0]) / 2; d = `M${sx} ${sy} C${mx} ${sy},${mx} ${p[1]},${p[0]} ${p[1]}`; }
      lines += `<path d="${d}"${cls}/>` + `<circle cx="${p[0].toFixed(1)}" cy="${p[1].toFixed(1)}" r="${hot === k ? 4 : 2.6}"${cls}/>`;
    }
  }
  svg.innerHTML = lines; svg.style.opacity = mode === 'system' ? 1 : alpha;
  const g = $('sh-goal'), goal = S.slam && S.slam.goal;
  if (a.goal && goal && view.world > 0.5) { g.hidden = false; g.textContent = goal.name || goal.label || 'Goal';
    g.style.transform = `translate(${(a.goal[0] - g.offsetWidth / 2).toFixed(1)}px,${(a.goal[1] - g.offsetHeight - 14).toFixed(1)}px)`; g.style.opacity = view.world; }
  else g.hidden = true;
}

// ---- 2D panels of the Drive app
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
  const f = fit(cv); if (!f) return; const [g, w, h] = f, s = S.slam, pose = mapPose();
  let cx = pose[0], cy = pose[1];
  if (big && s && s.goal && fin(s.goal.x)) { cx = (cx + s.goal.x) / 2; cy = (cy + s.goal.y) / 2;
    scale = Math.max(40, Math.min(scale, 0.7 * Math.min(w, h) / Math.max(0.5, Math.hypot(s.goal.x - pose[0], s.goal.y - pose[1])))); }
  const X = (x) => w / 2 + (x - cx) * scale, Y = (y) => h / 2 - (y - cy) * scale;
  if (big) { g.fillStyle = '#181411'; g.fillRect(0, 0, w, h); }
  if (S.mapImg && S.mapMeta) {
    const mm = S.mapMeta; g.imageSmoothingEnabled = scale < 40;
    g.drawImage(S.mapImg, X(mm.x0), Y(mm.y0 + mm.h), mm.w * scale, mm.h * scale);
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
// Messages from an action you took ("Pose reset") show in the pill for a few seconds.
document.addEventListener('consolemsg', (e) => { const d = e.detail || {}; if (!d.text || d.text === 'connected') return;
  S.note = { text: d.text, bad: !!d.bad, until: performance.now() + 3600 }; paint(); });
document.addEventListener('params', () => { if (curTab && curHost && !LIVE_TABS.includes(curTab.id)) showTab(curTab.id, curHost); });
const clock = () => { $('sh-clock').textContent = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }); };

(async function boot() {
  clock(); setInterval(clock, 20000);
  buildParts();
  try {
    const slow = Math.max(1, +Q.get('slow') || 1);                   // ?slow=N stretches the view change, for filming it
    if (slow > 1) $('sh').style.setProperty('--s-move', (1.6 * slow) + 's cubic-bezier(.65,0,.35,1)');
    engine = await createEngine($('sh-gl'), $('sh-viz'), { slow });
    engine.onFrame(placeLabels);
  } catch (e) { console.error('3D view failed to start', e); $('sh-fallback').hidden = false; $('sh-gl').hidden = true; }
  for (const f of TAB_FILES) {
    try { (await import(`/ui/tabs/${f}.js`)).default(registerTab); } catch (e) { console.error('tab failed:', f, e); }
  }
  buildControlsNav();
  if (DEMO) {
    $('sh-demo').hidden = false; S.linkUp = true; $('d-feednote').textContent = 'Live camera (off in demo)';
    await (await import('./demo.js')).startDemo({ console: onConsole, slam: onSlam, mission: onMission, map: onMap,
      scene: (sc) => engine && engine.setScene(sc), vector: (v) => engine && engine.setVector(v), pose: onPose,
      sys: (snap) => { S.sys = snap; paintParts(); } }, Q);
  } else {
    connectConsole(); connectSlam();
    pollMission(); setInterval(pollMission, 1000);
    pollPower(); setInterval(pollPower, 3000);
    pollSys(); setInterval(pollSys, 2000);
    refreshParams().catch(() => {});
    setInterval(() => { if (!Object.keys(state.params).length) refreshParams().catch(() => {}); }, 4000);
  }
  if (['parked', 'drive', 'park'].includes(Q.get('view'))) { held = Q.get('view'); setHeldLabel(); }
  wasArmed = armed();
  updateMode(true);
  if (Q.get('app') && document.querySelector(`[data-open="${Q.get('app')}"]`)) openApp(Q.get('app'));
  if (Q.get('hot') && BLOCKS[Q.get('hot')]) setHot(Q.get('hot'));
  setInterval(paint, 200); setInterval(drawPanels, 250); paint();
  window.__shell = { engine, S, state, mode: () => mode };
  window.__ready = true;
})();
