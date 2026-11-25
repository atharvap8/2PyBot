/* core.js - shared console core: the HTTP API, live state and the parameter renderer.
 *
 * Used by both the console shell (shell/shell.js) and the classic page (app.js), so a
 * tab module gets the same context whichever page hosts it.
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
  power(mode)         { return API.post('/api/power', { mode }); },
  status()            { return API.get('/api/status'); },
};
window.API = API;

export function msg(t, bad) {
  const el = document.getElementById('msg');
  if (!el) return;
  el.textContent = t;
  el.style.color = bad ? 'var(--rd)' : 'var(--dim)';
  document.dispatchEvent(new CustomEvent('consolemsg', { detail: { text: t, bad: !!bad } }));
}

// ---------------- shared state ----------------
export const state = { params: {}, derived: {}, tel: {}, debug: {}, tof: {}, pending: {} };

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
    const step = p.hi > p.lo ? (p.hi - p.lo) / 500 : 1;
    row.innerHTML = `
      <label>${p.key}<small>${p.desc}</small></label>
      <input type=range min=${p.lo} max=${p.hi} step=${step} value=${p.value}>
      <input type=number min=${p.lo} max=${p.hi} step=${step} value=${fmt(p.value)}>`;
    const [rng, num] = row.querySelectorAll('input');
    rng.disabled = num.disabled = p.lo === p.hi;

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
  const el = document.getElementById('dirty');
  if (!el) return;
  el.textContent = v ? '● unsaved changes' : '';
  el.style.color = v ? 'var(--am)' : 'var(--dim)';
}
