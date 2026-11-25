/* tabs/drive_view.js - Tesla-style driving cockpit.
 *
 * A dark, full-bleed visualisation: a perspective floor that scrolls with the
 * robot's speed, the robot drawn centred and leaning with pitch, the live ToF
 * returns as glowing blips inside the safety corridor, and a minimal HUD
 * (speed, state, heading, balance). Matches the console theme. Animates with
 * requestAnimationFrame and stops when you leave the tab. */
export default function register(registerTab) {
  let cv, g, raf = 0, active = false, last = 0, scroll = 0;
  let tel = {}, dbg = {}, tof = {};
  const C = { acc: "#f08a4b", accDim: "rgba(240,138,75,", ok: "#34d399", warn: "#f0b84b", err: "#ff4d6d",
              ink: "#efe9e3", dim: "#73695f", floor: "#1b1815", line: "rgba(240,138,75,.10)" };
  const botImg = new Image(); let botReady = false;
  botImg.onload = () => { botReady = true; };
  botImg.src = "/ui/sysmap/bot_front.png";

  function resize() {
    cv.width = Math.max(1, Math.round(cv.clientWidth * devicePixelRatio));
    cv.height = Math.max(1, Math.round(cv.clientHeight * devicePixelRatio));
  }

  function frame(now) {
    if (!active) return;
    const dt = Math.min(0.05, (now - last) / 1000 || 0); last = now;
    scroll = (scroll + (tel.v_ms || 0) * dt) % 0.5;      // road scroll phase (metres)
    draw();
    raf = requestAnimationFrame(frame);
  }

  function draw() {
    if (!cv.clientWidth) return;
    resize();
    const W = cv.width, H = cv.height, dpr = devicePixelRatio;
    const horizon = H * 0.40, groundY = H * 0.94, laneHalf = W * 0.46;
    // perspective helpers: robot frame (x lateral m, y forward m) -> screen
    const screenY = (y) => horizon + (groundY - horizon) / (1 + Math.max(0, y) * 0.95);
    const f = (y) => (screenY(y) - horizon) / (groundY - horizon);     // 1 near, 0 far
    const P = (x, y) => [W / 2 + x * laneHalf * f(y), screenY(y)];

    // background: dark radial
    const bg = g.createLinearGradient(0, 0, 0, H);
    bg.addColorStop(0, "#0e0c0b"); bg.addColorStop(0.4, "#141210"); bg.addColorStop(1, "#191512");
    g.fillStyle = bg; g.fillRect(0, 0, W, H);
    // sky glow at the horizon
    const sky = g.createRadialGradient(W / 2, horizon, 0, W / 2, horizon, W * 0.6);
    sky.addColorStop(0, "rgba(240,138,75,.10)"); sky.addColorStop(1, "rgba(240,138,75,0)");
    g.fillStyle = sky; g.fillRect(0, 0, W, horizon + 40 * dpr);
    g.strokeStyle = "rgba(240,138,75,.22)"; g.lineWidth = 1 * dpr;
    g.beginPath(); g.moveTo(0, horizon); g.lineTo(W, horizon); g.stroke();

    // perspective floor grid (forward lines scroll toward the viewer)
    g.lineWidth = 1 * dpr;
    for (let i = 0; i < 14; i++) {
      const y = i * 0.5 + (0.5 - scroll);
      const sy = screenY(y), a = Math.max(0, 0.18 * f(y));
      g.strokeStyle = `rgba(240,138,75,${a})`;
      g.beginPath(); g.moveTo(0, sy); g.lineTo(W, sy); g.stroke();
    }
    for (let x = -1; x <= 1; x += 0.25) {                 // lateral lines converge to vanishing point
      const a = P(x, 0), b = P(x, 12);
      g.strokeStyle = C.line; g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
    }

    // safety corridor ahead (Tesla-style path), coloured by the bubble guard
    const half = tof.corridor_half_width_m ?? 0.14;
    const stop = Math.min(2.2, tof.stopping_distance_m ?? 0.6);
    const guard = tof.guard_enabled !== false && !tof.stale;
    const corridor = (y) => half + Math.min(0.3, 0.5 * (tof.turn_envelope_curvature ?? 0) * y * y);
    g.beginPath();
    let started = false;
    for (let y = 0; y <= stop + 0.001; y += 0.1) { const p = P(-corridor(y), y); started ? g.lineTo(...p) : (g.moveTo(...p), started = true); }
    for (let y = stop; y >= 0; y -= 0.1) { g.lineTo(...P(corridor(y), y)); }
    g.closePath();
    g.fillStyle = guard ? "rgba(240,138,75,.12)" : "rgba(115,105,95,.10)";
    g.fill();
    g.strokeStyle = guard ? "rgba(240,138,75,.5)" : "rgba(115,105,95,.4)"; g.lineWidth = 2 * dpr;
    g.stroke();
    // stop line at the stopping distance
    const sl1 = P(-corridor(stop), stop), sl2 = P(corridor(stop), stop);
    g.strokeStyle = guard ? C.acc : C.dim; g.lineWidth = 2.5 * dpr;
    g.beginPath(); g.moveTo(...sl1); g.lineTo(...sl2); g.stroke();

    // ToF returns as glowing blips
    if (!tof.stale) for (const pt of (tof.points || [])) {
      if (pt.y_m < 0.02) continue;
      const [sx, sy] = P(pt.x_m, pt.y_m);
      const r = Math.max(2.2, 7 * f(pt.y_m)) * dpr;
      const danger = (pt.z_m ?? 0) < -0.03;               // drop-off
      const col = danger ? C.err : C.acc;
      const grd = g.createRadialGradient(sx, sy, 0, sx, sy, r * 3);
      grd.addColorStop(0, col); grd.addColorStop(1, (danger ? "rgba(255,77,109," : C.accDim) + "0)");
      g.fillStyle = grd; g.globalAlpha = Math.max(0.35, pt.confidence ?? 0.5);
      g.beginPath(); g.arc(sx, sy, r * 3, 0, 7); g.fill();
      g.globalAlpha = 1; g.fillStyle = col;
      g.beginPath(); g.arc(sx, sy, r, 0, 7); g.fill();
    }

    drawRobot(W / 2, groundY, H, dpr);
  }

  function drawRobot(cx, baseY, H, dpr) {
    // The real robot, upright and grounded (pitch is shown on the gauge, not by
    // tilting the whole robot - that read as "falling over" at rest).
    const hh = H * 0.46, asp = botReady && botImg.naturalHeight ? botImg.naturalWidth / botImg.naturalHeight : 0.82;
    const ww = hh * asp, top = baseY - hh;
    // contact shadow
    g.fillStyle = "rgba(0,0,0,.5)";
    g.beginPath(); g.ellipse(cx, baseY - 4 * dpr, ww * 0.42, hh * 0.07, 0, 0, 7); g.fill();
    // soft accent glow behind
    const glow = g.createRadialGradient(cx, top + hh * 0.5, 0, cx, top + hh * 0.5, ww * 0.85);
    glow.addColorStop(0, "rgba(240,138,75,.12)"); glow.addColorStop(1, "rgba(240,138,75,0)");
    g.fillStyle = glow; g.fillRect(cx - ww, top - hh * 0.1, ww * 2, hh * 1.2);
    if (botReady) {
      g.drawImage(botImg, cx - ww / 2, top, ww, hh);
    } else {                                               // fallback until the photo loads
      g.fillStyle = C.acc; g.beginPath(); g.arc(cx, baseY - hh * 0.4, hh * 0.3, 0, 7); g.fill();
    }
  }

  function paintHud() {
    const t = tel;
    const online = !t.stale;
    const state = t.startup_state && t.startup_state !== "READY" ? t.startup_state
      : (!online ? "NO LINK" : t.asleep ? "ASLEEP" : t.balancing ? "BALANCING" : "IDLE");
    const set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
    set("dv-state", state);
    const dot = document.getElementById("dv-dot");
    if (dot) dot.style.background = !online ? C.err : t.balancing ? C.ok : C.warn;
    set("dv-speed", Math.abs(t.v_ms ?? 0).toFixed(2));
    set("dv-head", `${Math.round(t.heading ?? 0)}°`);
    set("dv-pitch-v", `${(t.pitch ?? 0).toFixed(1)}°`);
    const fill = document.getElementById("dv-pitch-fill");
    if (fill) { const p = Math.max(-1, Math.min(1, (t.pitch ?? 0) / 25));
      fill.style.height = Math.abs(p) * 50 + "%"; fill.style.top = p < 0 ? "50%" : (50 - Math.abs(p) * 50) + "%";
      fill.style.background = Math.abs(t.pitch ?? 0) > 12 ? C.err : C.acc; }
    const guard = document.getElementById("dv-guard");
    if (guard) guard.textContent = tof.stale ? "bubble: no lidar" : tof.guard_enabled !== false ? "bubble: active" : "bubble: off";
    set("dv-hz", `${Math.round(t.rx_hz ?? 0)} Hz`);
  }

  return registerTab({
    id: "cockpit", title: "Drive", order: 2,
    render(el, c) {
      el.innerHTML = `
      <div class="card drive">
        <canvas id="dvcanvas"></canvas>
        <div class="dv-hud">
          <div class="dv-state"><i class="dot" id="dv-dot"></i><span id="dv-state">—</span></div>
          <div class="dv-speed"><b id="dv-speed">0.00</b><u>m/s</u></div>
          <div class="dv-head" id="dv-head">0°</div>
          <div class="dv-pitch"><div class="dv-pitch-track"><i id="dv-pitch-fill"></i></div><span id="dv-pitch-v">0°</span></div>
          <div class="dv-foot"><span id="dv-guard">bubble —</span><span id="dv-hz">— Hz</span></div>
          <div class="dv-ctrls">
            <button class="act" id="dv-cal">Calibrate gyro</button>
            <button class="act" id="dv-reset">Reset pose</button>
            <button class="act danger" id="dv-stop">E-stop</button>
          </div>
        </div>
      </div>`;
      cv = el.querySelector("#dvcanvas"); g = cv.getContext("2d");
      tel = c.state.tel || {}; dbg = c.state.debug || {}; tof = c.state.tof || {};
      el.querySelector("#dv-cal").onclick = () => c.API.command("C");
      el.querySelector("#dv-reset").onclick = () => c.API.post("/api/pose/reset").then(() => c.msg("pose reset"));
      el.querySelector("#dv-stop").onclick = () => c.API.estop();
      active = true; last = performance.now(); paintHud();
      raf = requestAnimationFrame(frame);
    },
    onLeave() { active = false; cancelAnimationFrame(raf); },
    onTelemetry(t, d) { tel = t; dbg = d; paintHud(); },
    onToF(x) { tof = x || {}; paintHud(); },
  });
}
