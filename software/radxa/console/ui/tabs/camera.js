/* tabs/camera.js — WebRTC preview, v4l2 controls, low-light calibration.
 * Playback is WHEP (WebRTC-HTTP Egress) straight from MediaMTX, so there
 * is no MJPEG polling and no extra library: ~100 ms on 5 GHz.
 */
import { liveVideo } from '../live_video.js';

export default function register(registerTab) {
  let viewer = null, lumaTimer = null;

  return registerTab({
    id: 'camera', title: 'Camera', order: 2,
    async render(el, c) {
      el.innerHTML = `
      <div class=card><h3>Live feed <small style="color:var(--dim)">WebRTC / WHEP</small></h3>
        <video id=vid autoplay playsinline muted></video>
        <div class=bar>
          <button class="act primary" id=bPlay>Connect</button>
          <button class=act id=bSnap>Snapshot</button>
          <span class=chip id=cStream>—</span>
          <span class=chip id=cLuma>luma —</span>
        </div>
      </div>
      <div class=card><h3>Low-light calibration</h3>
        <p class=hint>Auto-exposure is why the frame rate collapses in dim rooms —
          the sensor lengthens exposure instead of dropping brightness. This locks
          exposure, then walks exposure and gain to hit a target mean luma.
          Exposure first (adds motion blur), gain second (adds noise). CSI exposure
          is in microseconds and capped to one sensor frame. No calibration is
          performed until you press the button.</p>
        <div class=bar>
          <button class="act primary" id=bAuto>Auto-calibrate</button>
          <label style="color:var(--dim);font-size:12px">target luma
            <input type=number id=tgt value=110 min=40 max=200 style="width:70px"></label>
        </div>
        <div class=bar id=profs></div>
        <pre class=log id=calLog style="max-height:150px">idle</pre>
      </div>
      <div class=card><h3>Stream</h3>
        <div class=bar>
          <label style="color:var(--dim);font-size:12px">max width
            <select id=selW><option>640</option><option>1280</option><option selected>1920</option></select></label>
          <label style="color:var(--dim);font-size:12px">bitrate
            <select id=selB><option>1M</option><option>2M</option><option selected>4M</option><option>8M</option></select></label>
          <button class=act id=bRestart>Restart pipeline</button>
        </div>
        <div class=deriv id=fmtInfo></div>
      </div>
      <div class=card><h3>Sensor controls</h3>
        <p class=hint>Enumerated live from v4l2 — these are the controls your
          specific camera actually exposes. CSI controls go through the owning
          publisher, so tuning works while live video is running. Changes persist
          across restarts. rpi_imx708 uses sensor-specific Raspberry Pi colour
          matrices and gamma in the ISP; Kelvin changes the white point and matrix.</p>
        <div id=ctrls></div>
        <div class=bar>
          <input id=profName placeholder="profile name" style="background:rgba(255,255,255,.06);
            border:1px solid var(--line);color:var(--tx);border-radius:8px;padding:8px">
          <button class=act id=bSaveProf>Save current as profile</button>
        </div>
      </div>`;

      const vid = el.querySelector('#vid');
      const live = liveVideo(vid, value => c.msg(value));
      viewer = live;
      const st = await c.API.get('/api/stream').catch(() => ({}));
      if (!el.isConnected) return;
      el.querySelector('#cStream').textContent =
        st.running ? `${st.mode?.w}x${st.mode?.h} ${st.encoder}` : (st.error || 'stopped');
      el.querySelector('#bPlay').onclick = async () => {
        await live.start();
      };
      live.start();
      el.querySelector('#bSnap').onclick = () => window.open('/api/camera/snapshot', '_blank');
      el.querySelector('#bRestart').onclick = async () => {
        c.msg('restarting pipeline…');
        try {
          const r = await c.API.post('/api/stream/restart', {
            width: +el.querySelector('#selW').value, bitrate: el.querySelector('#selB').value });
          c.msg(r.detail);
        } catch (e) { c.msg(e.message, true); }
      };

      // v4l2 controls
      const d = await c.API.get('/api/camera/controls').catch(() => null);
      if (!el.isConnected) return;
      if (d) {
        el.querySelector('#fmtInfo').innerHTML = (d.formats || []).slice(0, 6)
          .map(f => `${f.fmt} <b>${f.w}x${f.h}</b> @${Math.max(...(f.fps || [0]))}`).join(' · ');
        const box = el.querySelector('#ctrls');
        for (const ctl of d.controls) {
          if (!['int', 'bool', 'menu'].includes(ctl.type)) continue;
          const v = d.values[ctl.name];
          const row = document.createElement('div');
          row.className = 'row';
          const lo = ctl.min ?? 0, hi = ctl.max ?? 1;
          row.innerHTML = `<label>${ctl.name}<small>${ctl.unit || ctl.type}${ctl.flags ? ' · ' + ctl.flags : ''}</small></label>
            <input type=range min=${lo} max=${hi} step=${ctl.step || 1} value=${v ?? lo}>
            <input type=number min=${lo} max=${hi} step=${ctl.step || 1} value=${v ?? lo}>`;
          const [r1, n1] = row.querySelectorAll('input');
          const put = async (val) => {
            r1.value = val; n1.value = val;
            try { await c.API.post('/api/camera/set', { name: ctl.name, value: +val }); c.msg(`${ctl.name} = ${val}`); }
            catch (e) { c.msg(`${ctl.name}: ${e.message}`, true); }
          };
          let t = null;
          r1.oninput = () => { n1.value = r1.value; clearTimeout(t); t = setTimeout(() => put(r1.value), 150); };
          n1.onchange = () => put(n1.value);
          box.appendChild(row);
        }
        const pb = el.querySelector('#profs');
        for (const p of d.profiles) {
          const b = document.createElement('button');
          b.className = 'act'; b.textContent = p;
          b.onclick = async () => {
            try { const r = await c.API.post(`/api/camera/profile/${p}`);
                  c.msg(`${p}: applied ${Object.keys(r.applied || {}).length} controls`); }
            catch (e) { c.msg(e.message, true); }
          };
          pb.appendChild(b);
        }
      }

      el.querySelector('#bSaveProf').onclick = async () => {
        const name = el.querySelector('#profName').value.trim();
        if (!name) return c.msg('name the profile first', true);
        await c.API.post('/api/camera/profile/save', { name });
        c.msg(`saved profile "${name}"`);
      };

      el.querySelector('#bAuto').onclick = async () => {
        const log = el.querySelector('#calLog');
        log.textContent = 'calibrating — this takes ~15 s…\n';
        try {
          const t = +el.querySelector('#tgt').value;
          const r = await c.API.post(`/api/camera/autocalibrate?target=${t}`);
          log.textContent = (r.steps || []).map(s =>
            `iter ${s.iter}: luma ${s.luma}  exposure ${s.exposure}  gain ${s.gain}`).join('\n')
            + `\n\n${r.ok ? (r.converged ? 'converged' : (r.note || 'stopped')) : 'FAILED: ' + r.error}`;
          c.msg(r.converged ? 'camera calibrated' : 'calibration stopped short — see log');
        } catch (e) { log.textContent = e.message; c.msg(e.message, true); }
      };

      lumaTimer = setInterval(async () => {
        try { const l = await c.API.get('/api/camera/luma');
              const e = el.querySelector('#cLuma');
              if (e && l.luma != null) e.textContent = `luma ${l.luma.toFixed(0)}`; } catch {}
      }, 4000);
    },
    onLeave() { viewer?.stop(); viewer = null; clearInterval(lumaTimer); }
  });
}
