/* app.js — console core.
 *
 * Tabs live in ./tabs/*.js, one file per tab, exactly as specified.
 * Each exports:  { id, title, order, render(el, ctx) }  and optionally
 * { onTelemetry(tel, dbg) } for live updates without a re-render.
 *
 * Config tabs don't hand-write their fields: the firmware's P? dump
 * carries a `group` per parameter, so a tab just says which group it
 * owns and the shared renderer builds the rows. Add a parameter in
 * firmware and it shows up here with no change to this file.
 */

import { API, msg, state, refreshParams, renderGroup, persistBar, markDirty } from './core.js';
export { API, msg, state, refreshParams, renderGroup, persistBar, markDirty };

// ---------------- tabs ----------------
const TABS = [];
export function registerTab(t) { TABS.push(t); }

let current = null;
function select(tab) {
  current?.onLeave?.();
  current = tab;
  for (const b of document.querySelectorAll('nav button')) b.classList.toggle('sel', b.dataset.id === tab.id);
  const view = document.getElementById('view');
  view.innerHTML = '';
  tab.render(view, { state, API, renderGroup, msg, refreshParams });
  localStorage.setItem('tab', tab.id);
}

// Firmware parameter tabs collapse into one "Parameters" group so the rail stays
// short; the perception/dashboard tabs stay at the top level.
const PARAM_TABS = new Set(['balance', 'motor', 'drive', 'yaw', 'imu', 'safety', 'climb', 'led', 'payload']);

function buildNav() {
  const nav = document.getElementById('tabs');
  nav.innerHTML = '';
  TABS.sort((a, b) => (a.order ?? 50) - (b.order ?? 50));
  const mkBtn = (t) => {
    const b = document.createElement('button');
    b.textContent = t.title; b.dataset.id = t.id;
    b.onclick = () => select(t);
    return b;
  };
  const params = [];
  for (const t of TABS) {
    if (PARAM_TABS.has(t.id)) params.push(t);
    else nav.appendChild(mkBtn(t));
  }
  if (params.length) {
    const group = document.createElement('details');
    group.className = 'navgroup'; group.id = 'navparams';
    const summary = document.createElement('summary');
    summary.textContent = 'Parameters';
    group.appendChild(summary);
    const box = document.createElement('div');
    box.className = 'navgroup-items';
    for (const t of params) box.appendChild(mkBtn(t));
    group.appendChild(box);
    nav.appendChild(group);
  }
  const want = TABS.find(t => t.id === localStorage.getItem('tab')) || TABS[0];
  if (want) {
    if (PARAM_TABS.has(want.id)) document.getElementById('navparams').open = true;
    select(want);
  }
}

// ---------------- websocket ----------------
function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === 'telemetry') {
      state.tel = m.tel; state.debug = m.debug || {};
      paintHeader();
      current?.onTelemetry?.(state.tel, state.debug);
    } else if (m.type === 'tof') {
      state.tof = m.tof;
      current?.onToF?.(state.tof);
    } else if (m.type === 'hello') {
      state.tel = m.status.telemetry || {};
      state.debug = m.status.debug || {};
      state.tof = m.status.tof || {};
      paintHeader();
      current?.onToF?.(state.tof);
    } else if (m.type === 'params') {
      state.params = Object.fromEntries(m.params.map(p => [p.key, p]));
      state.derived = m.derived || {};
      document.dispatchEvent(new CustomEvent('params'));
    } else if (m.type === 'visual_pair') {
      paintVisualPair(m.visual_pair);
    } else if (m.type === 'param_msg' || m.type === 'log') {
      msg(m.text, /PE,|ERROR|FAIL/.test(m.text));
      document.dispatchEvent(new CustomEvent('logline', { detail: m.text }));
    }
  };
  ws.onclose = () => { document.getElementById('dot').classList.remove('ok'); setTimeout(connectWS, 1500); };
  ws.onopen  = () => msg('connected');
}

function paintHeader() {
  const t = state.tel, d = state.debug;
  document.getElementById('dot').classList.toggle('ok', !t.stale);
  const waiting = state.derived.BOOTGATE === 1 && !t.host_ready;
   const startup = {WAIT_HOST:'WAIT RADXA ACK',INITIALIZING:'INITIALIZING',CALIBRATING:'IMU CALIBRATION',ERROR:'STARTUP ERROR'}[t.startup_state];
   const st = startup || (t.balancing ? 'BALANCING' : (t.stale ? 'NO LINK' : (waiting ? 'BOOT WAIT' : (t.asleep ? 'ASLEEP' : 'IDLE'))));
  const cs = document.getElementById('cState');
  cs.textContent = st;
  cs.className = 'chip' + (t.stale ? ' bad' : (t.balancing ? '' : ' warn'));
  document.getElementById('cPitch').textContent = `pitch ${(t.pitch ?? 0).toFixed(1)}°`;
  document.getElementById('cVel').textContent = `${(t.v_ms ?? 0).toFixed(2)} m/s`;
  const rx = document.getElementById('cRx');
  rx.textContent = `${(t.rx_hz ?? 0).toFixed(0)} Hz`;
  rx.className = 'chip' + ((t.rx_hz ?? 0) < 30 ? ' warn' : '');
}

// ---------------- visual re-pair ----------------
// Shows a progress banner while the gamepad is being held in front of the camera.
let vpTimer = null;
function paintVisualPair(vp) {
  if (!vp) return;
  let bar = document.getElementById('vpbar');
  const active = vp.enabled && ['watching', 'seen', 'triggered'].includes(vp.state) && (vp.hits > 0 || vp.state === 'triggered');
  if (!active) { if (bar) bar.remove(); return; }
  if (!bar) {
    bar = document.createElement('div'); bar.id = 'vpbar'; bar.className = 'vpbar';
    bar.innerHTML = '<span class="t"></span><div class="track"><div class="fill"></div></div>';
    (document.getElementById('sleepbar') || document.querySelector('header.top')).after(bar);
  }
  const fire = vp.state === 'triggered';
  bar.classList.toggle('fire', fire);
  bar.querySelector('.t').textContent = fire
    ? 'Gamepad recognised: clearing pairings and restarting. Hold HOME + A on the pad.'
    : `Hold the gamepad face-on to the camera… ${Math.round(vp.progress * 100)}%`;
  bar.querySelector('.fill').style.width = Math.round((fire ? 1 : vp.progress) * 100) + '%';
  clearTimeout(vpTimer);
  vpTimer = setTimeout(() => { const b = document.getElementById('vpbar'); if (b) b.remove(); }, fire ? 12000 : 4000);
}

// ---------------- power-down ----------------
// Camera, lidar, depth sensor and the controller's outputs off so the robot
// does not heat while unused. The console and Wi-Fi stay up to wake it.
let power = { asleep: false };
function paintPower() {
  const b = document.getElementById('pwr');
  if (!b) return;
  b.textContent = power.busy ? (power.asleep ? 'Powering down…' : 'Waking…') : (power.asleep ? 'Wake up' : 'Power down');
  b.classList.toggle('asleep', !!power.asleep);
  b.disabled = !!power.busy;
  let bar = document.getElementById('sleepbar');
  if (power.asleep && !bar) {
    bar = document.createElement('div'); bar.id = 'sleepbar'; bar.className = 'sleepbar';
    document.querySelector('header.top').after(bar);
  }
  if (bar) {
    bar.hidden = !power.asleep;
    const mins = Math.round((power.asleep_s || 0) / 60);
    bar.textContent = `Powered down${mins ? ' for ' + mins + ' min' : ''}: camera, lidar, depth sensor, ring, servos, torch and gamepad are off. ` +
      'Press Wake up to use the robot.' + (power.controller_supported === false ? ' (Controller firmware older than 4.7.7: only the host side is off.)' : '');
  }
}
async function refreshPower() {
  try { power = await API.get('/api/power'); paintPower(); } catch (e) { /* console restarting */ }
}
async function togglePower() {
  const sleep = !power.asleep;
  if (sleep && !confirm('Power down the robot?\n\nCamera, lidar, depth sensor, LED ring, servos, torch and the gamepad switch off ' +
                        'and the robot cannot be armed. Wake it again from this console.')) return;
  power = { ...power, busy: true, asleep: sleep }; paintPower();
  try { power = await API.power(sleep ? 'sleep' : 'wake'); msg(sleep ? 'Powered down' : 'Awake: camera and sensors starting'); }
  catch (e) { msg(e.message, true); }
  refreshPower();
}

// ---------------- boot ----------------
const TAB_FILES = ['telemetry', 'drive_view', 'camera', 'calibration', 'balance', 'motor', 'drive', 'yaw',
                   'safety', 'imu', 'climb', 'led', 'payload', 'console'];

(async function boot() {
  for (const f of TAB_FILES) {
    try { (await import(`/ui/tabs/${f}.js`)).default(registerTab); }
    catch (e) { console.error('tab failed:', f, e); }
  }
  buildNav();
  connectWS();
  document.getElementById('pwr')?.addEventListener('click', togglePower);
  refreshPower(); setInterval(refreshPower, 3000);
  try { await refreshParams(); } catch (e) { msg('waiting for ESP32…', true); }
  document.addEventListener('params', () => {
    paintHeader();
    // Parameter dumps must not tear down live video/grid/log views.
    if (current && !['telemetry', 'cockpit', 'camera', 'calibration', 'console'].includes(current.id)) select(current);
  });
  setInterval(() => { if (!Object.keys(state.params).length) refreshParams().catch(() => {}); }, 4000);
})();
