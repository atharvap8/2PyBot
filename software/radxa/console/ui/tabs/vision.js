import { liveVideo } from '../live_video.js';
import { prepareModel, drawScene, drawTrail } from '../driving_view.js';

export default function register(registerTab) {
  let root=null, viewer=null, triangles=[], path=[], context=null, resize=null;
  function draw() {
    if (!root?.isConnected) return;
    drawScene(root.querySelector('#robotScene'),triangles,context.state.tel,context.state.tof);
    drawTrail(root.querySelector('#driveMap'),path,context.state.tel);
  }
  function paint(t) {
    if (!root?.isConnected) return;
    context.state.tof=t;
    const text=(id,value)=>{ root.querySelector(id).textContent=value; };
    text('#tofState',`${t.stale?'STALE':t.state || 'starting'} · ${t.valid_count ?? 0}/64 usable`);
    text('#tofError',t.error || t.quality_note || '');
    text('#tofNearest',t.nearest_m==null?'Unknown':`${t.nearest_m.toFixed(2)} m`);
    text('#tofCoverage',`${Math.round((t.coverage || 0)*100)}% coverage`);
    text('#guardState',t.guard_enabled?`${t.guard_active?'ON':'PENDING'} · ${t.guard_reason}`:'Guard OFF');
    text('#guardCap',t.forward_limit_mps==null?'—':`${t.forward_limit_mps.toFixed(2)} m/s cap`);
    const toggle=root.querySelector('#tofGuard'); toggle.checked=!!t.guard_enabled;
    toggle.disabled=!t.guard_supported && !t.guard_enabled;
    draw();
  }
  function telemetry(t,d={}) {
    if (!root?.isConnected) return;
    context.state.tel=t;
    root.querySelector('#driveSpeed').textContent=(t.v_ms || 0).toFixed(2);
    root.querySelector('#drivePitch').textContent=`Pitch ${(t.pitch || 0).toFixed(1)}°`;
    root.querySelector('#driveBalance').textContent=t.stale?'NO LINK':t.balancing?'BALANCING':'IDLE';
    root.querySelector('#driveLoop').textContent=`Loop ${((d.loopMax || 0)/1000).toFixed(1)} ms`;
    const last=path[path.length-1];
    if (!t.stale && (!last || Math.hypot((t.x_m || 0)-last[0],(t.y_m || 0)-last[1])>.01)) path.push([t.x_m || 0,t.y_m || 0]);
    if (path.length>3000) path.shift();
    draw();
  }
  registerTab({
    id:'vision',title:'Driving view',order:1.5,
    render(el,c) {
      root=el; context=c;
      el.innerHTML=`
        <div class=driving-layout>
          <div class="drive-panel drive-map"><h3>Trail</h3><canvas id=driveMap></canvas><small>Encoder dead reckoning</small></div>
          <div class="drive-panel drive-scene"><canvas id=robotScene></canvas></div>
          <div class="drive-panel drive-stats"><strong id=driveSpeed>0.00</strong><small>m/s</small>
            <p id=driveBalance>IDLE</p><p id=drivePitch>Pitch —</p><p id=driveLoop>Loop —</p>
            <hr><p id=tofState>Starting</p><p id=tofCoverage>—</p><p>Corridor: <b id=tofNearest>—</b></p>
            <label><input type=checkbox id=tofGuard> Forward guard</label><p id=guardState>OFF</p><small id=guardCap>—</small>
          </div>
        </div>
        <p class=hint id=tofError></p>
        <div class=card><h3>Confidence-based reference guard</h3>
          <p class=hint>Confirmed returns in the projected corridor set a reaction/braking speed cap. Partial coverage allows reduced-speed requests; stale or wholly unknown depth holds forward requests.
            Reverse, steering and balance recovery remain available. Off after console restart. Approximate optics/mounting; reference floor is not measured terrain.</p></div>
        <details class=card><summary>Live camera</summary><video autoplay playsinline muted id=tofVideo></video>
          <div class=bar><button class=act id=tofConnect>Reconnect video</button><button class=act id=tofSnap>Snapshot</button><span class=chip id=tofCameraState>Connecting…</span></div>
        </details>`;
      fetch('/ui/robot-model.json').then(r=>{ if (!r.ok) throw Error('Model unavailable'); return r.json(); })
        .then(model=>{ if (el.isConnected) { triangles=prepareModel(model); draw(); } }).catch(e=>c.msg(e.message,true));
      c.API.get('/api/path').then(d=>{ if (el.isConnected) { path=d.path || []; draw(); } }).catch(e=>c.msg(e.message,true));
      viewer=liveVideo(el.querySelector('#tofVideo'),value=>{ if (el.isConnected) el.querySelector('#tofCameraState').textContent=value; });
      el.querySelector('details').ontoggle=e=>{ if (e.target.open) viewer?.start(); else viewer?.stop(); };
      el.querySelector('#tofConnect').onclick=()=>viewer.start();
      el.querySelector('#tofSnap').onclick=()=>window.open('/api/camera/snapshot','_blank');
      el.querySelector('#tofGuard').onchange=async e=>{
        try { paint(await c.API.post('/api/tof/guard',{enabled:e.target.checked})); }
        catch(error) { e.target.checked=!!c.state.tof.guard_enabled; c.msg(error.message,true); }
      };
      resize=new ResizeObserver(draw); resize.observe(el);
      telemetry(c.state.tel,c.state.debug); paint(c.state.tof);
      c.API.get('/api/tof').then(t=>{ if (el.isConnected) paint(t); }).catch(e=>c.msg(e.message,true));
    },
    onToF:paint,onTelemetry:telemetry,
    onLeave() { resize?.disconnect(); resize=null; viewer?.stop(); viewer=null; root=null; context=null; }
  });
}
