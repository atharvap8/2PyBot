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

// ---------------- API ----------------
export const API = {
  async get(p) { const r = await fetch(p); if (!r.ok) throw new Error(await r.text()); return r.json(); },
  async post(p, body) {
    const r = await fetch(p, { method: 'POST', headers: { 'Content-Type': 'application/json' },
                               body: body === undefined ? '{}' : JSON.stringify(body) });
    if (!r.ok) throw new Error((await r.text()).slice(0, 200));
    return r.json();
  },
  params()            { return API.get('/api/params'); },
  setParam(key, v)    { return API.post('/api/params/set', { key, value: v }); },
  paramAction(a)      { return API.post('/api/params/' + a); },
  command(line)       { return API.post('/api/command', { line }); },
  estop()             { return API.post('/api/estop').then(() => msg('E-STOP sent')); },
  status()            { return API.get('/api/status'); },
};
window.API = API;

export function msg(t, bad) {
  const el = document.getElementById('msg');
  el.textContent = t;
  el.style.color = bad ? 'var(--rd)' : 'var(--dim)';
}

// ---------------- shared state ----------------
export const state = { params: {}, derived: {}, tel: {}, debug: {}, pending: {} };

export async function refreshParams() {
  const d = await API.params();
  state.params = Object.fromEntries(d.params.map(p => [p.key, p]));
  state.derived = d.derived || {};
  document.dispatchEvent(new CustomEvent('params'));
  return state.params;
}

// ---------------- shared parameter renderer ----------------
const fmt = v => (Math.abs(v) >= 100 || Number.isInteger(v)) ? String(v) : v.toFixed(4).replace(/0+$/, '');

/** Build the rows for one firmware group into `el`. */
export function renderGroup(el, group, opts = {}) {
  const list = Object.values(state.params).filter(p => p.group === group);
  if (!list.length) {
    el.innerHTML = `<div class=card><h3>${group}</h3>
      <p class=hint>No parameters reported yet. Waiting for the ESP32 dump…</p></div>`;
    return;
  }
  const card = document.createElement('div');
  card.className = 'card';
  card.innerHTML = `<h3>${opts.title || group}</h3><p class=hint>${opts.hint || ''}</p>`;

  for (const p of list) {
    const row = document.createElement('div');
    row.className = 'row';
    const step = (p.hi - p.lo) / 500;
    row.innerHTML = `
      <label>${p.key}<small>${p.desc}</small></label>
      <input type=range min=${p.lo} max=${p.hi} step=${step} value=${p.value}>
      <input type=number min=${p.lo} max=${p.hi} step=${step} value=${fmt(p.value)}>`;
    const [rng, num] = row.querySelectorAll('input');

    const push = async (v) => {
      v = Math.min(p.hi, Math.max(p.lo, Number(v)));
      rng.value = v; num.value = fmt(v);
      row.classList.add('changed');
      try { await API.setParam(p.key, v); msg(`${p.key} = ${fmt(v)} (not saved)`); markDirty(true); }
      catch (e) { msg(`${p.key}: ${e.message}`, true); }
    };
    let t = null;
    rng.oninput = () => { num.value = fmt(Number(rng.value)); clearTimeout(t); t = setTimeout(() => push(rng.value), 120); };
    num.onchange = () => push(num.value);
    card.appendChild(row);
  }
  el.appendChild(card);

  if (opts.derived !== false) {
    const d = document.createElement('div');
    d.className = 'deriv';
    d.innerHTML = Object.entries(state.derived)
      .map(([k, v]) => `${k} <b>${Number(v).toFixed(2)}</b>`).join('');
    card.appendChild(d);
  }
  el.appendChild(persistBar());
}

export function persistBar() {
  const bar = document.createElement('div');
  bar.className = 'bar';
  bar.innerHTML = `
    <button class="act primary" data-a=save>Save to robot (NVS)</button>
    <button class=act data-a=load>Reload saved</button>
    <button class="act danger" data-a=defaults>Restore defaults</button>
    <button class=act data-a=refresh>Re-read</button>`;
  bar.onclick = async (e) => {
    const a = e.target.dataset.a; if (!a) return;
    if (a === 'defaults' && !confirm('Restore compiled defaults? This does not save.')) return;
    try {
      await API.paramAction(a);
      msg(a === 'save' ? 'Saved to NVS — survives reboot' : `${a} sent`);
      if (a === 'save') markDirty(false);
      setTimeout(refreshParams, 400);
    } catch (err) { msg(err.message, true); }
  };
  return bar;
}

let dirty = false;
export function markDirty(v) {
  dirty = v;
  document.getElementById('dirty').textContent = v ? '● unsaved changes' : '';
  document.getElementById('dirty').style.color = v ? 'var(--am)' : 'var(--dim)';
}

// ---------------- tabs ----------------
const TABS = [];
export function registerTab(t) { TABS.push(t); }

let current = null;
function select(tab) {
  current = tab;
  for (const b of document.querySelectorAll('nav button')) b.classList.toggle('sel', b.dataset.id === tab.id);
  const view = document.getElementById('view');
  view.innerHTML = '';
  tab.render(view, { state, API, renderGroup, msg, refreshParams });
  localStorage.setItem('tab', tab.id);
}

function buildNav() {
  const nav = document.getElementById('tabs');
  nav.innerHTML = '';
  TABS.sort((a, b) => (a.order ?? 50) - (b.order ?? 50));
  for (const t of TABS) {
    const b = document.createElement('button');
    b.textContent = t.title; b.dataset.id = t.id;
    b.onclick = () => select(t);
    nav.appendChild(b);
  }
  const want = TABS.find(t => t.id === localStorage.getItem('tab')) || TABS[0];
  if (want) select(want);
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
    } else if (m.type === 'params') {
      state.params = Object.fromEntries(m.params.map(p => [p.key, p]));
      state.derived = m.derived || {};
      document.dispatchEvent(new CustomEvent('params'));
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
  const st = t.balancing ? 'BALANCING' : (t.stale ? 'NO LINK' : 'IDLE');
  const cs = document.getElementById('cState');
  cs.textContent = st;
  cs.className = 'chip' + (t.stale ? ' bad' : (t.balancing ? '' : ' warn'));
  document.getElementById('cPitch').textContent = `pitch ${(t.pitch ?? 0).toFixed(1)}°`;
  document.getElementById('cVel').textContent = `${(t.v_ms ?? 0).toFixed(2)} m/s`;
  const rx = document.getElementById('cRx');
  rx.textContent = `${(t.rx_hz ?? 0).toFixed(0)} Hz`;
  rx.className = 'chip' + ((t.rx_hz ?? 0) < 30 ? ' warn' : '');
}

// ---------------- boot ----------------
const TAB_FILES = ['telemetry', 'camera', 'balance', 'stepper', 'drive', 'yaw',
                   'safety', 'imu', 'climb', 'led', 'payload', 'console'];

(async function boot() {
  for (const f of TAB_FILES) {
    try { (await import(`/ui/tabs/${f}.js`)).default(registerTab); }
    catch (e) { console.error('tab failed:', f, e); }
  }
  buildNav();
  connectWS();
  try { await refreshParams(); } catch (e) { msg('waiting for ESP32…', true); }
  document.addEventListener('params', () => { if (current) select(current); });
  setInterval(() => { if (!Object.keys(state.params).length) refreshParams().catch(() => {}); }, 4000);
})();
