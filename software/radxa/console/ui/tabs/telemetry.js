/* tabs/telemetry.js — live state, dead-reckoned map, pad mirror, debug stream. */
export default function register(registerTab) {
  let cv, ctx2d, path = [];

  function drawMap(tel) {
    if (!cv) return;
    const w = cv.width = cv.clientWidth * devicePixelRatio;
    const h = cv.height = cv.clientHeight * devicePixelRatio;
    const g = ctx2d;
    g.clearRect(0, 0, w, h);
    // extent
    const pts = path.concat([[tel.x_m || 0, tel.y_m || 0]]);
    let mnx = Math.min(...pts.map(p => p[0])), mxx = Math.max(...pts.map(p => p[0]));
    let mny = Math.min(...pts.map(p => p[1])), mxy = Math.max(...pts.map(p => p[1]));
    const pad = 0.6, spanx = Math.max(1.2, mxx - mnx + pad * 2), spany = Math.max(1.2, mxy - mny + pad * 2);
    const span = Math.max(spanx, spany);
    const cx = (mnx + mxx) / 2, cy = (mny + mxy) / 2;
    const S = Math.min(w, h) / span;
    const X = x => w / 2 + (x - cx) * S, Y = y => h / 2 - (y - cy) * S;
    // grid, 0.5 m
    g.strokeStyle = 'rgba(255,255,255,.06)'; g.lineWidth = 1;
    for (let m = Math.floor(cx - span / 2); m <= cx + span / 2; m += 0.5) {
      g.beginPath(); g.moveTo(X(m), 0); g.lineTo(X(m), h); g.stroke();
    }
    for (let m = Math.floor(cy - span / 2); m <= cy + span / 2; m += 0.5) {
      g.beginPath(); g.moveTo(0, Y(m)); g.lineTo(w, Y(m)); g.stroke();
    }
    // trail
    if (path.length > 1) {
      g.strokeStyle = 'rgba(53,224,216,.85)'; g.lineWidth = 2 * devicePixelRatio;
      g.beginPath(); path.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1])));
      g.stroke();
    }
    // robot
    const th = (tel.heading || 0) * Math.PI / 180;
    const rx = X(tel.x_m || 0), ry = Y(tel.y_m || 0), r = 11 * devicePixelRatio;
    g.save(); g.translate(rx, ry); g.rotate(-th);
    g.fillStyle = tel.balancing ? '#4ade80' : '#fbbf24';
    g.beginPath(); g.moveTo(r * 1.6, 0); g.lineTo(-r, r * .85); g.lineTo(-r * .4, 0);
    g.lineTo(-r, -r * .85); g.closePath(); g.fill();
    g.restore();
    g.fillStyle = 'rgba(255,255,255,.45)';
    g.font = `${11 * devicePixelRatio}px ui-monospace,monospace`;
    g.fillText(`${span.toFixed(1)} m across`, 8 * devicePixelRatio, 16 * devicePixelRatio);
  }

  return registerTab({
    id: 'telemetry', title: 'Telemetry', order: 1,
    render(el, c) {
      el.innerHTML = `
      <div class=card><h3>Live state</h3>
        <div class=grid>
          <div class=stat><u>Pitch</u><b id=tPitch>—</b></div>
          <div class=stat><u>Pitch rate</u><b id=tRate>—</b></div>
          <div class=stat><u>Velocity</u><b id=tVel>—</b></div>
          <div class=stat><u>Pos error</u><b id=tEx>—</b></div>
          <div class=stat><u>Heading</u><b id=tHead>—</b></div>
          <div class=stat><u>O-stream</u><b id=tHz>—</b></div>
        </div>
        <div class=bar>
          <button class=act id=bDbg>Toggle D-stream (L)</button>
          <button class=act id=bCal>Calibrate gyro (C)</button>
          <button class=act id=bSet>Dump settings (S)</button>
          <button class="act danger" id=bStop>E-stop (X)</button>
        </div>
      </div>
      <div class=card><h3>Odometry map</h3>
        <p class=hint>Dead-reckoned from the wheel encoders — drift accumulates,
           it is not SLAM. Trail clears on reset.</p>
        <canvas class=map id=map></canvas>
        <div class=bar><button class=act id=bReset>Reset pose</button></div>
      </div>
      <div class=card><h3>Gamepad (mirrored from the ESP32)</h3>
        <p class=hint>The pad is paired to the ESP32 over Bluepad32 and always will be.
           This is a read-only mirror of what the firmware is receiving.</p>
        <div class=grid>
          <div class=stat><u>Drive stick</u><b id=tFwd>—</b></div>
          <div class=stat><u>Steer stick</u><b id=tStr>—</b></div>
          <div class=stat><u>Speed mode</u><b id=tSpd>—</b></div>
          <div class=stat><u>vCmd steps/s</u><b id=tVcmd>—</b></div>
          <div class=stat><u>satV / satA</u><b id=tSat>—</b></div>
          <div class=stat><u>loop max</u><b id=tLoop>—</b></div>
        </div>
      </div>
      <div class=card><h3>Robot log</h3><pre class=log id=log></pre></div>`;

      cv = el.querySelector('#map'); ctx2d = cv.getContext('2d');
      el.querySelector('#bDbg').onclick   = () => c.API.command('L');
      el.querySelector('#bCal').onclick   = () => c.API.command('C');
      el.querySelector('#bSet').onclick   = () => c.API.command('S');
      el.querySelector('#bStop').onclick  = () => c.API.estop();
      el.querySelector('#bReset').onclick = async () => {
        await c.API.post('/api/pose/reset'); path = []; c.msg('pose reset');
      };
      c.API.get('/api/path').then(d => { path = d.path || []; drawMap(c.state.tel); });
      c.API.get('/api/log').then(d => {
        const p = el.querySelector('#log');
        p.textContent = d.log.map(l => l.text).join('\n'); p.scrollTop = p.scrollHeight;
      });
      document.addEventListener('logline', (e) => {
        const p = el.querySelector('#log'); if (!p) return;
        p.textContent += '\n' + e.detail; p.scrollTop = p.scrollHeight;
      });
    },
    onTelemetry(t, d) {
      const S = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
      S('tPitch', `${(t.pitch ?? 0).toFixed(2)}°`);
      S('tRate',  `${(d.w ?? 0).toFixed(0)}°/s`);
      S('tVel',   `${(t.v_ms ?? 0).toFixed(2)}`);
      S('tEx',    d.ex !== undefined ? `${(d.ex * 100).toFixed(1)} cm` : '—');
      S('tHead',  `${(t.heading ?? 0).toFixed(0)}°`);
      S('tHz',    `${(t.rx_hz ?? 0).toFixed(0)}`);
      S('tFwd',   d.fwd !== undefined ? d.fwd.toFixed(2) : '—');
      S('tStr',   d.str !== undefined ? d.str.toFixed(2) : '—');
      S('tSpd',   d.spdHi !== undefined ? (d.spdHi ? 'HIGH' : 'LOW') : '—');
      S('tVcmd',  d.vCmd !== undefined ? d.vCmd.toFixed(0) : '—');
      S('tSat',   `${d.satV ?? 0} / ${d.satA ?? 0}`);
      S('tLoop',  d.loopMax !== undefined ? `${(d.loopMax / 1000).toFixed(1)} ms` : '—');
      if (t.x_m !== undefined) {
        const last = path[path.length - 1];
        if (!last || Math.hypot(t.x_m - last[0], t.y_m - last[1]) > 0.02) path.push([t.x_m, t.y_m]);
        if (path.length > 3000) path.shift();
        drawMap(t);
      }
    }
  });
}
