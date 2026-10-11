import { liveVideo } from '../live_video.js';

export default function register(registerTab) {
  let cells = [], root = null, viewer = null;
  function paint(t) {
    if (!root?.isConnected) return;
    const text = (id, value) => { root.querySelector(id).textContent = value; };
    text('#tofState', `${t.state || 'starting'} · ${t.stale ? 'STALE' : `${t.age_ms ?? 0} ms`} · ${t.valid_count ?? 0}/64 valid`);
    text('#tofError', t.error || t.quality_note || '');
    text('#tofNearest', t.nearest_m == null ? 'Unknown' : `${t.nearest_m.toFixed(2)} m`);
    text('#guardState', t.guard_enabled
      ? `${t.guard_active ? 'ON' : 'PENDING'} · ${t.guard_reason} · cap ${t.forward_limit_mps ?? 0} m/s` : 'OFF');
    const toggle = root.querySelector('#tofGuard');
    toggle.checked = !!t.guard_enabled;
    toggle.disabled = !t.guard_supported && !t.guard_enabled;
    text('#guardFirmware', t.guard_supported ? 'Firmware ready' : 'Guard requires updated ESP32 firmware');
    for (let i = 0; i < cells.length; i++) {
      const value = t.distance_mm?.[i];
      const good = !t.stale && t.valid?.[i];
      cells[i].textContent = value == null ? '—' : `${value}${good ? '' : '*'}`;
      cells[i].style.background = good ? `hsl(${Math.min(130, value / 8)} 60% 24%)` : '#252b36';
      cells[i].title = `Zone ${i} · status ${t.status?.[i] ?? '—'} · sigma ${t.sigma_mm?.[i] ?? '—'} mm`;
    }
  }
  registerTab({
    id: 'vision', title: 'Vision + ToF', order: 1.5,
    render(el, c) {
      root = el;
      el.innerHTML = `
        <div class=vision-layout>
          <div class=card><h3>Cubie camera</h3>
            <video autoplay playsinline muted id=tofVideo></video>
            <div class=bar><button class=act id=tofConnect>Reconnect video</button>
              <button class=act id=tofSnap>Snapshot</button><span class=chip id=tofCameraState>Connecting…</span></div>
          </div>
          <div class=card><h3>VL53L5CX · 8×8 depth</h3>
            <p class=hint>Millimetres, native zone order. * = invalid or stale; grey is unknown.</p>
            <div class=tof-grid id=tofGrid></div>
            <div class=bar><span class=chip id=tofState>Starting</span>
              <span class=chip>Central nearest: <b id=tofNearest>—</b></span></div>
            <p class=hint id=tofError></p>
          </div>
        </div>
        <div class=card><h3>Optional obstacle guard</h3>
          <label><input type=checkbox id=tofGuard> Limit forward driving near obstacles</label>
          <div class=bar><span class=chip id=guardState>OFF</span><span class=chip id=guardFirmware></span></div>
          <p class=hint>Off after every console restart. When enabled: central 4×6 zones slow requests
            below 0.80 m and stop forward requests at 0.35 m. Unknown/stale central depth blocks forward
            requests. Reverse, steering and balance recovery remain available. Grid orientation and
            stopping clearance need checking on the mounted bot; this is a reference limiter, not a hard wheel stop.</p>
        </div>`;
      cells = Array.from({ length: 64 }, (_, i) => {
        const cell = document.createElement('div');
        cell.className = 'tof-cell';
        if (Math.floor(i / 8) >= 2 && Math.floor(i / 8) < 6 && i % 8 >= 1 && i % 8 < 7)
          cell.classList.add('roi');
        el.querySelector('#tofGrid').appendChild(cell);
        return cell;
      });
      viewer = liveVideo(el.querySelector('#tofVideo'), value => {
        if (el.isConnected) el.querySelector('#tofCameraState').textContent = value;
      });
      viewer.start();
      el.querySelector('#tofConnect').onclick = () => viewer.start();
      el.querySelector('#tofSnap').onclick = () => window.open('/api/camera/snapshot', '_blank');
      el.querySelector('#tofGuard').onchange = async e => {
        try { paint(await c.API.post('/api/tof/guard', { enabled: e.target.checked })); }
        catch (error) { e.target.checked = !!c.state.tof.guard_enabled; c.msg(error.message, true); }
      };
      paint(c.state.tof);
      c.API.get('/api/tof').then(paint).catch(e => c.msg(e.message, true));
    },
    onToF: paint,
    onLeave() { viewer?.stop(); viewer = null; root = null; cells = []; }
  });
}
