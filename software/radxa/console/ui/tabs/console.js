/* tabs/console.js — raw serial console + derived values + link health. */
export default function register(registerTab) {
  return registerTab({
    id: 'console', title: 'Console', order: 95,
    render(el, c) {
      el.innerHTML = `
      <div class=card><h3>Derived (read-only, computed by firmware)</h3>
        <div class=grid id=dv></div>
        <p class=hint>These come from WHEELR and the stepper limits. If STEPSM
          disagrees with reality the whole control loop is scaled wrong —
          measure the loaded rolling radius and set WHEELR on the Stepper tab.</p>
      </div>
      <div class=card><h3>Send a command</h3>
        <p class=hint>Whitelisted only. A remote client may STOP the robot but
          never START it, so <code>E</code> is rejected by the console on purpose.</p>
        <div class=bar>
          <input id=cmd placeholder="e.g. G,dance   H,1   K4=-4.2854"
            style="flex:1;min-width:220px;background:rgba(255,255,255,.06);border:1px solid var(--line);
            color:var(--tx);border-radius:8px;padding:9px;font:inherit">
          <button class=act id=bSend>Send</button>
        </div>
        <div class=bar>
          <button class=act data-c="G,yes">nod yes</button>
          <button class=act data-c="G,no">nod no</button>
          <button class=act data-c="G,spin">spin</button>
          <button class=act data-c="G,dance">dance</button>
          <button class=act data-c="H,1">stiff on</button>
          <button class=act data-c="H,0">stiff off</button>
        </div>
      </div>
      <div class=card><h3>Log</h3><pre class=log id=log></pre></div>`;

      const dv = el.querySelector('#dv');
      const units = { STEPSM: 'steps/m', COUNTSM: 'counts/m', VMAX: 'm/s', AMAX: 'm/s²' };
      dv.innerHTML = Object.entries(c.state.derived).map(([k, v]) =>
        `<div class=stat><u>${k}</u><b>${Number(v).toFixed(2)}</b>
         <small style="color:var(--dim);font-size:10px">${units[k] || ''}</small></div>`).join('')
        || '<p class=hint>waiting for the parameter dump…</p>';

      const send = async (line) => {
        try { await c.API.command(line); c.msg(`sent ${line}`); }
        catch (e) { c.msg(e.message, true); }
      };
      el.querySelector('#bSend').onclick = () => send(el.querySelector('#cmd').value.trim());
      el.querySelector('#cmd').onkeydown = (e) => { if (e.key === 'Enter') send(e.target.value.trim()); };
      el.onclick = (e) => { if (e.target.dataset.c) send(e.target.dataset.c); };

      c.API.get('/api/log').then(d => {
        const p = el.querySelector('#log');
        p.textContent = d.log.map(l => l.text).join('\n'); p.scrollTop = p.scrollHeight;
      });
      document.addEventListener('logline', (ev) => {
        const p = el.querySelector('#log'); if (!p) return;
        p.textContent += '\n' + ev.detail; p.scrollTop = p.scrollHeight;
      });
    }
  });
}
