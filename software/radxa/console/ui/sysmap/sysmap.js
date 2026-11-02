// sysmap.js — draws the map once from model.js, then repaints health from
// the server's snapshot stream. No framework, no build step, no network
// dependencies: it has to work on the robot's offline hotspot.
import { STAGE, BUSES, NODES, WIRES, SERIAL_INFO, PINS } from "./model.js";

const $ = (s) => document.querySelector(s);
const SVGNS = "http://www.w3.org/2000/svg";
const BADGE = { ok: "OK", warn: "WARN", error: "ERROR", blocked: "WAIT", nodata: "NO DATA", nofb: "NO FB" };
const PROBLEM = /ERROR|FATAL|NO REPLY|fault 0x(?!00)|timeout[1-9]|Disconnected|unavailable|failed/i;
const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)").matches;

const ICONS = {
  camera: '<rect x="2" y="6" width="15" height="12" rx="3"/><path d="M17 10l5-3v10l-5-3"/>',
  grid: '<rect x="3" y="3" width="18" height="18" rx="3"/><path d="M3 9h18M3 15h18M9 3v18M15 3v18"/>',
  phone: '<rect x="6" y="2" width="12" height="20" rx="3"/><path d="M11 18h2"/>',
  axis: '<path d="M12 2v20M2 12h20"/><circle cx="12" cy="12" r="4"/>',
  compass: '<circle cx="12" cy="12" r="9"/><path d="M12 5l3 7-3 7-3-7z"/>',
  pad: '<rect x="2" y="7" width="20" height="11" rx="5"/><path d="M7 11v3M5.5 12.5h3"/><circle cx="16" cy="11.5" r="1"/><circle cx="18" cy="13.5" r="1"/>',
  ring: '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="4" r="1.3"/><circle cx="20" cy="12" r="1.3"/><circle cx="12" cy="20" r="1.3"/><circle cx="4" cy="12" r="1.3"/>',
};
const ICON_BUS = { camera: "csi", grid: "i2c", phone: "wifi", axis: "i2c", compass: "i2c", pad: "bt", ring: "pwm" };

let snap = null, selected = null, userPicked = false, lastMsg = 0;
const els = {}, wireEls = {};

function esc(t) {
  return String(t ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

// ------------------------------------------------------------- build
function buildLegend() {
  $("#legend").innerHTML = "<span>Buses</span>" +
    BUSES.map(([k, label]) => `<span><i style="background:var(--${k})"></i>${label}</span>`).join("");
}

function buildWires() {
  const svg = $("#wires");
  svg.setAttribute("viewBox", `0 0 ${STAGE.w} ${STAGE.h}`);
  const stage = $("#stage");
  for (const w of WIRES) {
    const p = document.createElementNS(SVGNS, "path");
    p.setAttribute("d", w.d);
    p.setAttribute("class", "wire" + (w.mount ? " mount" : "") + (w.dashed ? " dashed" : ""));
    if (!w.mount) p.style.stroke = `var(--${w.bus})`;
    svg.appendChild(p);
    const rec = { w, path: p, len: p.getTotalLength(), pkts: [], phase: Math.random() };
    if (!w.mount) {
      const hit = document.createElementNS(SVGNS, "path");
      hit.setAttribute("d", w.d);
      hit.setAttribute("class", "hit");
      hit.addEventListener("click", () => select(w.node, true));
      svg.appendChild(hit);
      const n = 2;
      for (let i = 0; i < n; i++) {
        const c = document.createElementNS(SVGNS, "circle");
        c.setAttribute("r", "4");
        c.setAttribute("class", "pkt");
        const back = w.flow === "both" && i === 1;
        c.style.fill = back ? "var(--bg)" : `var(--${w.bus})`;
        if (back) { c.style.stroke = `var(--${w.bus})`; c.style.strokeWidth = "2"; }
        svg.appendChild(c);
        rec.pkts.push({ c, offset: i / n, back });
      }
      if (w.tag) {
        const t = document.createElement("div");
        t.className = "tag";
        t.style.left = w.tag[0] + "px"; t.style.top = w.tag[1] + "px";
        t.style.color = `var(--${w.bus})`;
        t.textContent = w.tag[2];
        stage.appendChild(t);
        rec.tag = t;
      }
    }
    wireEls[w.id] = rec;
  }
}

function icon(name) {
  return `<svg class="ic" viewBox="0 0 24 24" fill="none" stroke="var(--${ICON_BUS[name]})" stroke-width="2" aria-hidden="true">${ICONS[name]}</svg>`;
}

function buildNodes() {
  const host = $("#nodes");
  for (const [id, n] of Object.entries(NODES)) {
    const el = document.createElement("div");
    el.className = { leaf: "blk", hub: "hub", group: "grp" }[n.kind];
    el.tabIndex = 0;
    el.setAttribute("role", "button");
    el.addEventListener("keydown", (e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); select(id, true); } });
    Object.assign(el.style, { left: n.x + "px", top: n.y + "px", width: n.w + "px", height: n.h + "px" });
    el.setAttribute("aria-label", n.title);
    if (n.kind === "leaf") {
      el.innerHTML = `${icon(n.icon)}<div><h3>${esc(n.title)}</h3><p>${esc(n.sub)}</p><div class="line">–</div></div><span class="badge">–</span>`;
    } else if (n.kind === "hub") {
      el.innerHTML = `<span class="badge">–</span><h2>${esc(n.title)}</h2><div class="sub">${esc(n.sub)}</div><div class="line" hidden></div><ul></ul>`;
    } else if (n.stages) {
      el.innerHTML = `<div class="gh"><h3>${esc(n.title)}</h3><span>${esc(n.sub)}</span></div><span class="badge">–</span>
        <div class="chain">${n.stages.map((s, i) => `${i ? '<div class="ar">→</div>' : ""}<div class="st"><h4>${esc(s[0])}</h4><p>${esc(s[1])}</p></div>`).join("")}</div>
        <div class="kv"></div><div class="line">–</div>`;
    } else {
      el.innerHTML = `<div class="gh"><h3>${esc(n.title)}</h3><span>${esc(n.sub)}</span></div><span class="badge">–</span>
        <div class="outs">
          <div><span><h4>Pan servo</h4><p>GPIO 26, 50 Hz</p></span><b data-k="Pan">–</b></div>
          <div><span><h4>Zoom servo</h4><p>GPIO 0, strap pin</p></span><b data-k="Zoom">–</b></div>
          <div><span><h4>Torch</h4><p>GPIO 2 MOSFET, duty capped</p></span><b data-k="Torch">–</b></div>
        </div><div class="line">–</div>`;
    }
    el.addEventListener("click", () => select(id, true));
    host.appendChild(el);
    els[id] = el;
  }
}

function buildPins() {
  const row = (r) => `<tr><td>${esc(r[0])}</td><td>${esc(r[1])}</td><td>${esc(r[2])}</td></tr>`;
  $("#pins-esp32").innerHTML = "<tr><th>GPIO</th><th>Function</th><th>Connected to</th></tr>" + PINS.esp32.map(row).join("");
  $("#pins-radxa").innerHTML = "<tr><th>Port</th><th>Function</th><th>Connected to</th></tr>" + PINS.radxa.map(row).join("");
}

// ------------------------------------------------------------- render
const metric = (node, label) => (node.metrics || []).find((m) => m[0] === label)?.[1] ?? "–";

function lineFor(n, count) {
  if (n.status === "nofb" && n.metrics?.length) return `${esc(n.metrics[0][0])}: ${esc(n.metrics[0][1])}`;
  if (n.status === "ok") {
    return (n.metrics || []).slice(0, count).map((m) => `${esc(m[1])} <i>${esc(m[0].toLowerCase())}</i>`).join("&ensp; ");
  }
  return esc(n.reason);
}

function setStatus(el, status) {
  el.classList.remove("s-ok", "s-warn", "s-error", "s-blocked", "s-nodata", "s-nofb");
  el.classList.add("s-" + status);
}

function renderNode(id, n) {
  const el = els[id], def = NODES[id];
  if (!el) return;
  setStatus(el, n.status);
  el.querySelector(".badge").textContent = BADGE[n.status] || n.status;
  const line = el.querySelector(".line");
  if (def.kind === "hub") {
    el.querySelector("ul").innerHTML = (n.metrics || []).map((m) => `<li><span>${esc(m[0])}</span>${esc(m[1])}</li>`).join("");
    line.hidden = n.status === "ok";
    line.innerHTML = esc(n.reason);
  } else if (def.stages) {
    el.querySelector(".kv").innerHTML = (n.metrics || []).slice(0, 3)
      .map((m) => `<div><span>${esc(m[0])}</span><b>${esc(m[1])}</b></div>`).join("");
    line.innerHTML = n.status === "ok" ? `CRC errors ${esc(metric(n, "CRC errors"))}, timeouts ${esc(metric(n, "Timeouts"))}` : esc(n.reason);
  } else if (def.kind === "group") {
    el.querySelectorAll("[data-k]").forEach((b) => { b.textContent = metric(n, b.dataset.k); });
    line.innerHTML = esc(n.reason);
  } else {
    line.innerHTML = lineFor(n, def.lineCount ?? 2);
  }
}

function wireStatus(w) {
  const n = snap?.nodes?.[w.node];
  if (!n) return "nodata";
  if (n.status === "nofb") return snap.nodes.esp32?.status === "ok" ? "ok" : "blocked";
  return n.status;
}

function renderWires() {
  for (const rec of Object.values(wireEls)) {
    if (rec.w.mount) continue;
    const s = wireStatus(rec.w);
    rec.status = s;
    rec.path.classList.remove("s-ok", "s-warn", "s-error", "s-blocked", "s-nodata");
    rec.path.classList.add("s-" + s);
    rec.path.classList.toggle("sel", rec.w.node === selected);
    if (rec.tag) { setStatus(rec.tag, s); }
  }
}

function fmtTime(t) {
  return t ? new Date(t * 1000).toTimeString().slice(0, 8) : "";
}

function renderInspector() {
  const insp = $("#insp");
  const id = selected;
  const def = id === "serial" ? SERIAL_INFO : NODES[id];
  const n = snap?.nodes?.[id];
  if (!def) { insp.innerHTML = ""; return; }
  const status = n?.status || "nodata";
  insp.className = "insp s-" + status;
  const metrics = (n?.metrics || []).slice(0, 8);
  let heat = "";
  if (id === "tof" && snap?.tof_grid?.distance_mm?.length === 64) {
    const g = snap.tof_grid;
    heat = `<div class="heat" title="8×8 depth, near is brighter">${g.distance_mm.map((d, i) => {
      if (!g.valid[i]) return "<i></i>";
      const v = Math.max(0, Math.min(1, 1 - d / 2000));
      return `<i style="background:rgba(240,138,75,${(0.15 + v * 0.85).toFixed(2)})"></i>`;
    }).join("")}</div>`;
  }
  const logs = (snap?.log || []).filter((e) => def.log && def.log.test(e.text)).slice(-5);
  insp.innerHTML = `
    <div class="ih"><div><h2>${esc(def.title)}</h2><div class="path">${esc(def.path)}</div></div>
      <span class="badge">${BADGE[status] || status}</span></div>
    <div class="reason">${esc(n?.reason || "Waiting for the first snapshot")}</div>
    ${metrics.length ? `<div class="kv">${metrics.map((m) => `<div><span>${esc(m[0])}</span><b>${esc(m[1])}</b></div>`).join("")}</div>` : ""}
    <div class="body">${heat}<ul class="checks">${(def.checks || []).map((c) => `<li>${esc(c)}</li>`).join("")}
      ${n?.note ? `<li class="note">${esc(n.note)}</li>` : ""}</ul></div>
    ${logs.length ? `<div class="ilog">${logs.map((e) => `<div>${fmtTime(e.t)}  ${esc(e.text)}</div>`).join("")}</div>` : ""}`;
  Object.entries(els).forEach(([k, el]) => el.classList.toggle("sel", k === id));
}

function renderHeader() {
  const live = snap?.live || {};
  $("#h-port").textContent = live.port || "–";
  $("#h-baud").textContent = live.baud ? `${Number(live.baud).toLocaleString("en-US").replace(/,/g, " ")} baud` : "";
  const fresh = Date.now() - lastMsg < 3000;
  $("#h-dot").className = "dot " + (fresh ? "live" : "dead");
  $("#h-live").textContent = fresh ? "Live" : "Offline";
  $("#h-rate").textContent = fresh && live.rx_hz ? `${Math.round(live.rx_hz)} Hz` : "";
  const errs = snap?.summary?.errors || [];
  const pill = $("#h-errs");
  pill.hidden = !errs.length;
  pill.innerHTML = `<b>${errs.length} ${errs.length === 1 ? "error" : "errors"}</b> ${esc(errs.map(nameOf).join(", "))}`;
}

function nameOf(id) {
  return id === "serial" ? "serial link" : (NODES[id]?.title || id).split(",")[0];
}

function renderBot() {
  const live = snap?.nodes?.serial?.status === "ok";
  const l = live ? snap?.live || {} : {};
  const f = (v, d, u) => (typeof v === "number" ? v.toFixed(d) + u : "–");
  if (!live) { $("#b-state").textContent = "No telemetry"; }
  if (live) $("#b-state").textContent = l.balancing ? "Balancing" : l.startup_state ? l.startup_state.replace("_", " ").toLowerCase().replace(/^./, (c) => c.toUpperCase()) : "Idle";
  $("#b-pitch").textContent = f(l.pitch, 1, "°");
  $("#b-yaw").textContent = f(l.yaw, 0, "°");
  $("#b-speed").textContent = f(l.v_ms, 2, " m/s");
}

function renderLog() {
  if ($("#tab-log").hidden) return;
  const q = $("#log-filter").value.trim().toLowerCase();
  const only = $("#log-errors").checked;
  const rows = (snap?.log || []).filter((e) => (!q || e.text.toLowerCase().includes(q)) && (!only || PROBLEM.test(e.text)));
  $("#log").innerHTML = rows.slice().reverse().map((e) =>
    `<li class="${PROBLEM.test(e.text) ? "bad" : ""}"><time>${fmtTime(e.t)}</time><span>${esc(e.text)}</span></li>`).join("");
}

function render() {
  if (!snap) return;
  if (!userPicked) {
    const first = (snap.summary?.errors || [])[0] || (snap.summary?.warnings || [])[0];
    selected = first || selected || "esp32";
  }
  for (const [id, n] of Object.entries(snap.nodes || {})) renderNode(id, n);
  renderWires(); renderHeader(); renderBot(); renderInspector(); renderLog();
}

function select(id, byUser) {
  selected = id;
  if (byUser) userPicked = true;
  renderWires(); renderInspector();
}

// ------------------------------------------------------------- packets
let lastFrame = performance.now();
function animate(now) {
  const dt = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;
  for (const rec of Object.values(wireEls)) {
    if (rec.w.mount) continue;
    const s = rec.status;
    const moving = s === "ok" || s === "warn";
    const pps = 150 * (rec.w.speed || 1) * (s === "warn" ? 0.35 : 1);
    if (moving && !reduceMotion) rec.phase = (rec.phase + (pps * dt) / rec.len) % 1;
    for (const p of rec.pkts) {
      p.c.style.display = moving ? "" : "none";
      if (!moving) continue;
      let u = (rec.phase + p.offset) % 1;
      if (rec.w.flow === "rev" || p.back) u = 1 - u;
      const pt = rec.path.getPointAtLength(u * rec.len);
      p.c.setAttribute("cx", pt.x.toFixed(1));
      p.c.setAttribute("cy", pt.y.toFixed(1));
    }
  }
  requestAnimationFrame(animate);
}

// ------------------------------------------------------------- data
function apply(data) {
  snap = data;
  lastMsg = Date.now();
  $("#lost").hidden = true;
  render();
}

let ws = null, backoff = 500;
function connect() {
  const url = new URL("ws", location.href);
  url.protocol = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(url);
  ws.onmessage = (ev) => { backoff = 500; try { apply(JSON.parse(ev.data)); } catch (e) { console.warn(e); } };
  ws.onclose = () => { setTimeout(connect, backoff); backoff = Math.min(backoff * 2, 5000); };
  ws.onerror = () => ws.close();
}

setInterval(() => {
  if (Date.now() - lastMsg > 3000) {
    $("#lost").hidden = false;
    if (snap) {
      for (const n of Object.values(snap.nodes)) { n.status = "nodata"; n.reason = "No data from the console"; }
      snap.summary = { errors: [], warnings: [] };
      render();
    }
    renderHeader();
  }
}, 1000);

// ------------------------------------------------------------- layout & UI
function fit() {
  const box = $("#fit");
  const s = Math.max(0.55, Math.min(box.clientWidth / STAGE.w, box.clientHeight / STAGE.h));
  const st = $("#stage");
  st.style.transform = `scale(${s})`;
  st.style.marginRight = `${STAGE.w * (s - 1)}px`;
  st.style.marginBottom = `${STAGE.h * (s - 1)}px`;
  st.style.marginLeft = `${Math.max(0, (box.clientWidth - STAGE.w * s) / 2)}px`;
}

function initTabs() {
  document.querySelectorAll(".tabs button").forEach((b) => b.addEventListener("click", () => {
    document.querySelectorAll(".tabs button").forEach((x) => x.setAttribute("aria-selected", String(x === b)));
    document.querySelectorAll(".view").forEach((v) => { v.hidden = v.id !== "tab-" + b.dataset.tab; });
    if (b.dataset.tab === "map") fit();
    renderLog();
  }));
  $("#log-filter").addEventListener("input", renderLog);
  $("#log-errors").addEventListener("change", renderLog);
}

$("#h-errs").addEventListener("click", () => {
  const errs = snap?.summary?.errors || [];
  if (!errs.length) return;
  const i = errs.indexOf(selected);
  select(errs[(i + 1) % errs.length], true);
});

$("#estop").addEventListener("click", async () => {
  const b = $("#estop");
  try {
    const r = await fetch("api/estop", { method: "POST" });
    b.textContent = r.ok ? "Stop sent" : "Stop failed";
  } catch { b.textContent = "Stop failed"; }
  setTimeout(() => { b.textContent = "Stop robot"; }, 1500);
});

document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") { userPicked = false; render(); }
});

$("#h-host").textContent = location.host;
buildLegend(); buildWires(); buildNodes(); buildPins(); initTabs();
addEventListener("resize", fit); fit();
requestAnimationFrame(animate);
connect();
