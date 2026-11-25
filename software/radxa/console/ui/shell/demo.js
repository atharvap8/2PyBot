/* shell/demo.js - synthetic data for /?demo, in exactly the shapes the robot serves.
 *
 * It lets you see every view (parked, driving a route, pulling up close to something)
 * without arming the robot. Nothing here talks to the robot: it only feeds the shell.
 *   &t=SECONDS   freeze the loop at one moment
 *   &data=URL    replay a capture of a real map instead of the built-in rooms:
 *                { vector | scene, map: { png, meta }, state: { labels }, route, goal, sys }
 */
const RES = 0.05, ORIGIN = -10, D2R = Math.PI / 180;
const ROOMS = [[-0.95, -0.6, 0.85, 4.4], [0.85, 1.2, 3.4, 4.4], [-3.6, 0.4, -0.95, 4.4], [-3.6, -2.6, -0.95, 0.4]];
const WALLS = [[-0.95, -0.6, 0.85, -0.6], [-0.95, -2.6, -0.95, 0.9], [-0.95, 1.6, -0.95, 4.4], [0.85, -0.6, 0.85, 2.0],
  [0.85, 2.75, 0.85, 4.4], [-3.6, 4.4, 3.4, 4.4], [3.4, 1.2, 3.4, 4.4], [0.85, 1.2, 3.4, 1.2],
  [-3.6, -2.6, -3.6, 4.4], [-3.6, 0.4, -2.6, 0.4], [-1.9, 0.4, -0.95, 0.4], [-3.6, -2.6, -0.95, -2.6]];
const BOXES = [[-0.5, 1.3, 0.34, 0.34, 0.3], [0.42, 3.55, 0.36, 0.3, 0.26], [2.37, 2.55, 0.34, 0.34, 0.3], [-2.6, 2.7, 1.1, 0.5, 0.4]];   // x, y, w, d, h
const ROUTE = [[0, 0.12], [0.03, 0.8], [0.06, 1.5], [0.3, 2.12], [0.9, 2.4], [1.4, 2.52], [1.7, 2.55]];
const LABELS = [{ name: 'Kitchen', x: 1.7, y: 2.55 }, { name: 'Sofa', x: -2.3, y: 1.9 }, { name: 'Desk', x: -2.3, y: -1.1 }, { name: 'Dock', x: 0, y: -0.3 }];
const TOF = [[-0.5, 1.3, 0.28], [-0.42, 1.38, 0.2], [0.42, 3.55, 0.24], [2.37, 2.55, 0.28], [2.3, 2.62, 0.22]];
const CRUISE = 0.25, CREEP = 0.05, CREEP_M = 0.26;
const SYS = { summary: { errors: [], warnings: [], state: 'ok' }, nodes: {
  radxa: { status: 'ok', reason: 'Console, video and ToF services running', metrics: [['CPU temp', '54 °C'], ['Load', '1.32'], ['Memory', '12 %']] },
  camera: { status: 'ok', reason: 'Streaming', metrics: [['Frame rate', '38 fps'], ['Capture to publish', '31 ms']] },
  tof: { status: 'ok', reason: 'Ranging', metrics: [['Nearest', '0.84 m'], ['Valid zones', '61/64']] },
  lidar: { status: 'ok', reason: 'Scanning', metrics: [['Scan rate', '20.1 Hz'], ['Points', '226']] },
  clients: { status: 'ok', reason: 'Connected', metrics: [['Cockpit clients', '1'], ['Hotspot', 'up']] },
  serial: { status: 'ok', reason: 'Telemetry flowing', metrics: [['Odometry rate', '48 Hz']] },
  esp32: { status: 'ok', reason: 'Control loop healthy', metrics: [['State', 'Ready'], ['Loop max', '5.0 ms']] },
  imu: { status: 'ok', reason: 'Attitude updating', metrics: [['Pitch', '1.5 °'], ['Accel', '0.98 g']] },
  compass: { status: 'ok', reason: 'Heading updating', metrics: [['Heading', '-6 °']] },
  gamepad: { status: 'ok', reason: 'Connected over Bluetooth', metrics: [['Controller', 'Android'], ['Speed mode', 'high']] },
  lights: { status: 'nofb', reason: 'Output only', metrics: [['Blue LED should be', 'steady on (ready)']] },
  wheel_l: { status: 'ok', reason: 'Replying, no faults', metrics: [['Temp', '36 °C'], ['Current', '0.21 / 0.00 A']] },
  wheel_r: { status: 'ok', reason: 'Replying, no faults', metrics: [['Temp', '37 °C'], ['Current', '0.19 / 0.00 A']] },
  payload: { status: 'nofb', reason: 'Output only', metrics: [['Torch', '0 %']] } } };

function chaikin(pts, n) {
  for (let k = 0; k < n; k++) {
    const o = [pts[0]];
    for (let i = 0; i < pts.length - 1; i++) { const a = pts[i], b = pts[i + 1];
      o.push([a[0] * 0.75 + b[0] * 0.25, a[1] * 0.75 + b[1] * 0.25], [a[0] * 0.25 + b[0] * 0.75, a[1] * 0.25 + b[1] * 0.75]); }
    o.push(pts[pts.length - 1]); pts = o;
  }
  return pts;
}
function scene() {                                    // same shape as GET /slam/api/scene3d
  const idx = (v) => Math.round((v - ORIGIN) / RES), walls = new Set(), floor = [];
  for (const [x1, y1, x2, y2] of WALLS) { const n = Math.max(1, Math.round(Math.hypot(x2 - x1, y2 - y1) / RES));
    for (let i = 0; i <= n; i++) walls.add(idx(x1 + (x2 - x1) * i / n) + ',' + idx(y1 + (y2 - y1) * i / n)); }
  for (const [x, y, w, d] of BOXES.slice(3)) for (let i = idx(x - w / 2); i <= idx(x + w / 2); i++) for (let j = idx(y - d / 2); j <= idx(y + d / 2); j++) walls.add(i + ',' + j);
  for (const [x0, y0, x1, y1] of ROOMS) for (let i = idx(x0) + 1; i < idx(x1); i++) for (let j = idx(y0) + 1; j < idx(y1); j++) if (!walls.has(i + ',' + j)) floor.push([i, j]);
  return { grid_res: RES, grid_origin: ORIGIN, walls: [...walls].map((k) => k.split(',').map(Number)), floor, pose: [0, 0.12, Math.PI / 2], path: [], version: 1 };
}
function mapImage(sc) {                               // the grey map image of the SLAM websocket: 0 wall, 255 free, 128 unknown
  const x0 = -3.8, y0 = -2.8, w = 7.4, h = 7.4, k = 1 / RES, c = document.createElement('canvas'); c.width = Math.round(w * k); c.height = Math.round(h * k);
  const g = c.getContext('2d'), X = (i) => Math.round((ORIGIN + i * RES - x0) * k), Y = (j) => c.height - 1 - Math.round((ORIGIN + j * RES - y0) * k);
  g.fillStyle = '#808080'; g.fillRect(0, 0, c.width, c.height);
  g.fillStyle = '#fff'; for (const [i, j] of sc.floor) g.fillRect(X(i), Y(j), 1, 1);
  g.fillStyle = '#000'; for (const [i, j] of sc.walls) g.fillRect(X(i), Y(j), 1, 1);
  return { x0, y0, w, h, version: 1, png: c.toDataURL() };
}

export async function startDemo(feed, query) {
  let real = null;
  if (query.get('data')) { try { real = await (await fetch(query.get('data'))).json(); } catch (e) { console.error('demo data failed to load', e); } }
  const segs = [];                                    // what the pretend lidar sees
  let labels = LABELS, goalName = 'Kitchen', tof = TOF, route = ROUTE;
  if (real) {
    for (const poly of (real.vector || {}).walls || []) for (const ring of poly) for (let i = 0; i < ring.length; i++) {
      const p = ring[i], q = ring[(i + 1) % ring.length]; segs.push([p[0], p[1], q[0], q[1]]); }
    labels = (real.state || {}).labels || []; goalName = (real.goal || {}).name || 'Goal'; tof = []; route = real.route;
    if (real.vector) feed.vector(real.vector); else feed.scene(real.scene);
    if (real.map) feed.map({ ...real.map.meta, png: real.map.png });
  } else {
    segs.push(...WALLS);
    for (const [x, y, w, d] of BOXES) { const a = x - w / 2, b = x + w / 2, c = y - d / 2, e = y + d / 2;
      segs.push([a, c, b, c], [b, c, b, e], [b, e, a, e], [a, e, a, c]); }
    const sc = scene(); feed.scene(sc); feed.map(mapImage(sc));
  }
  feed.sys((real && real.sys) || SYS);

  const PATH = chaikin(route, 3), CUM = [0];
  for (let i = 1; i < PATH.length; i++) CUM.push(CUM[i - 1] + Math.hypot(PATH[i][0] - PATH[i - 1][0], PATH[i][1] - PATH[i - 1][1]));
  const LEN = CUM[CUM.length - 1];
  function along(s) {                                 // -> [x, y, heading, index]
    s = Math.max(0, Math.min(LEN, s)); let i = 1; while (i < CUM.length - 1 && CUM[i] < s) i++;
    const a = PATH[i - 1], b = PATH[i], f = (s - CUM[i - 1]) / Math.max(1e-6, CUM[i] - CUM[i - 1]);
    return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, Math.atan2(b[1] - a[1], b[0] - a[0]), i];
  }
  function lidar(px, py, th) {                        // 360 returns, [forward, left] in the robot frame
    const out = [];
    for (let k = 0; k < 360; k++) {
      const rel = k * D2R, a = th + rel, rx = Math.cos(a), ry = Math.sin(a); let best = 6;
      for (const [ax, ay, bx, by] of segs) {
        const sx = bx - ax, sy = by - ay, den = rx * sy - ry * sx; if (Math.abs(den) < 1e-9) continue;
        const t = ((ax - px) * sy - (ay - py) * sx) / den, u = ((ax - px) * ry - (ay - py) * rx) / den;
        if (t > 0.1 && u >= 0 && u <= 1 && t < best) best = t;
      }
      if (best < 6) out.push([+(best * Math.cos(rel)).toFixed(3), +(best * Math.sin(rel)).toFixed(3)]);
    }
    return out;
  }

  const frozen = query.has('t') ? parseFloat(query.get('t')) : null, t0 = performance.now();
  const T_ARM = 5, T_GO = 7, T_ARRIVE = T_GO + LEN / CRUISE, T_STOP = T_ARRIVE + CREEP_M / CREEP, T_DISARM = T_STOP + 7, PERIOD = T_DISARM + 5;
  const start = along(0), end = along(LEN), th0 = real ? start[2] : Math.PI / 2;
  function sample() {
    const t = frozen != null ? frozen : ((performance.now() - t0) / 1000) % PERIOD;
    let x = start[0], y = start[1], th = th0, v = 0, balancing = t >= T_ARM && t < T_DISARM, mission = null, plan = [], goal = null;
    if (t >= T_GO && t < T_ARRIVE) {
      const s = (t - T_GO) * CRUISE, p = along(s); [x, y, th] = p; v = CRUISE;
      plan = [[x, y], ...PATH.slice(p[3])]; goal = { x: end[0], y: end[1], name: goalName };
      mission = { id: 'demo', state: 'running', kind: 'goto', name: 'Go to ' + goalName, info: { distance_m: +(LEN - s).toFixed(2) }, reason: '' };
    } else if (t >= T_ARRIVE) {
      const d = Math.min(CREEP_M, (t - T_ARRIVE) * CREEP); x = end[0] + d * Math.cos(end[2]); y = end[1] + d * Math.sin(end[2]); th = end[2];
      v = t < T_STOP ? CREEP : 0;
      if (t >= T_DISARM) { x = start[0]; y = start[1]; th = th0; }
    }
    return { t, x, y, th, v, balancing, mission, plan, goal };
  }
  function slamTick() {
    const d = sample(), scan = lidar(d.x, d.y, d.th);
    let front = 9, back = 9;
    for (const p of scan) { if (Math.abs(p[1]) < 0.13) { if (p[0] > 0) front = Math.min(front, p[0] - 0.09); else back = Math.min(back, -p[0] - 0.06); } }
    const cap = Math.min(3, Math.sqrt(1.2 * Math.max(0, front - 0.06)));
    const bubble = { state: front < 0.1 ? 'front_blocked' : cap < 0.45 ? 'limiting' : 'clear', front_m: +front.toFixed(3), back_m: +back.toFixed(3),
      nearest_m: +Math.min(front, back).toFixed(3), forward_cap: +cap.toFixed(3), reverse_cap: 3, stale: false, footprint: [-0.06, 0.09, -0.105, 0.105], stop_margin: 0.06 };
    feed.mission({ mission: d.mission, bubble });
    feed.slam({ mode: 'localizing', map_name: 'home', message: '', lidar: { state: 'scanning', points: scan.length }, rates: { scan: 20 }, robot_live: true,
      pose: [d.x, d.y, d.th], scan, path: [], plan: d.plan, goal: d.goal, labels, bubble, map_version: 1,
      tof: { obstacles: tof, cliffs: [] }, attitude: { dev: 0, balancing: d.balancing } });
    const near = Math.max(120, Math.min(1500, front * 1000 + 90));
    feed.console({ type: 'tof', tof: { state: 'ranging', stale: false, distance_mm: Array.from({ length: 64 }, (_, i) => Math.round(i < 48 ? near + (i % 8) * 6 : near * 0.7)), valid: Array(64).fill(true) } });
  }
  function telTick() {                                // wheel odometry has its own frame: x forward at power-on, turned a quarter from the map
    const d = sample();
    feed.console({ type: 'telemetry', debug: {}, tel: { stale: false, rx_hz: 50, balancing: d.balancing, asleep: false, startup_state: 'READY',
      pitch: d.balancing ? 1.5 + d.v * 9 : 0, v_ms: d.v, heading: (d.th - Math.PI / 2) / D2R, x_m: d.y, y_m: -d.x } });
  }
  feed.console({ type: 'hub', hub: { temp_l: 36, temp_r: 37 } });
  telTick(); slamTick();
  setInterval(telTick, 22); setInterval(slamTick, 200);
}
