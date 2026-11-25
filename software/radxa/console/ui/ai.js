/* 2PyBot AI inference page - plain JS, no build step. Aggregates the on-board
   inference features (object detection, backend/NPU status, gamepad re-pair,
   custom model) as cards, same controls and theme as the Maps page. */
"use strict";
const $ = (id) => document.getElementById(id);
let det = null, rep = null, busyDet = false, busyRep = false;

async function api(path, body) {
  const r = await fetch(path, { method: body === undefined ? "GET" : "POST",
    headers: body === undefined ? {} : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body) });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.detail || `HTTP ${r.status}`);
  return j;
}
const fmt = (v, s = "") => (v === null || v === undefined ? "–" : v + s);

// ---------------------------------------------------------------- object detection
function paintDetect() {
  if (!det) return;
  const pill = $("d-pill");
  const tone = det.state === "running" ? "run" : det.state === "error" ? "warn" : "off";
  pill.className = "pill " + tone;
  pill.textContent = (det.state || "off").toUpperCase();
  $("d-start").disabled = det.enabled || !det.model_installed || !det.runtime_available;
  $("d-stop").disabled = !det.enabled;
  $("d-cap").textContent = `${(det.state || "off").toUpperCase()} · ${det.corrected_applied ? "rectified" : "raw"} · ${det.message || ""}`;
  if (det.enabled) $("d-image").src = "/api/ml/preview.jpg?t=" + Date.now();
  else $("d-image").removeAttribute("src");
  $("d-note").textContent = det.state === "error" ? det.message : "";
  $("d-note").className = "note" + (det.state === "error" ? " err" : "");
  const rows = $("d-rows"); rows.replaceChildren();
  for (const d of det.detections || []) {
    const tr = document.createElement("tr");
    for (const v of ["#" + d.track_id, d.label, Math.round(d.score * 100) + "%", "– alignment pending"]) {
      const td = document.createElement("td"); td.textContent = v; tr.appendChild(td);
    }
    rows.appendChild(tr);
  }
  if (!(det.detections || []).length) {
    const tr = document.createElement("tr"), td = document.createElement("td");
    td.colSpan = 4; td.textContent = det.enabled ? "Nothing above the confidence threshold" : "Detection is off";
    tr.appendChild(td); rows.appendChild(tr);
  }
  paintBackend();
  $("m-cur").textContent = det.model || "–";
}
function paintBackend() {
  const npu = det.npu || {};
  const using = (det.provider || "").startsWith("NPU");
  const pill = $("b-pill");
  pill.className = "pill " + (using ? "ok" : "off");
  pill.textContent = det.provider || "–";
  const dot = (b) => `<span class="dot ${b ? "ok" : "off"}"></span>`;
  $("b-kv").innerHTML =
    `<div><span>Active provider</span><b>${fmt(det.provider)}</b></div>` +
    `<div><span>Model</span><b>${fmt(det.model)}</b></div>` +
    `<div><span>Inference</span><b>${fmt(det.inference_ms, " ms")}</b></div>` +
    `<div><span>Result age</span><b>${fmt(det.age_ms, " ms")}</b></div>` +
    `<div><span>NPU device</span><b>${dot(npu.device)} ${npu.device ? "present" : "absent"}</b></div>` +
    `<div><span>NPU driver</span><b>${dot(npu.driver)} ${npu.driver ? "loaded" : "missing"}</b></div>` +
    `<div><span>NPU model (NBG)</span><b>${dot(npu.model)} ${npu.model ? "installed" : "missing"}</b></div>` +
    `<div><span>NPU ready</span><b>${dot(npu.usable)} ${npu.usable ? "yes" : (npu.reason || "no")}</b></div>`;
}
async function pollDetect() {
  if (busyDet) return; busyDet = true;
  try { det = await api("/api/ml/status"); paintDetect(); }
  catch (e) { $("d-note").textContent = e.message; $("d-note").className = "note err"; }
  finally { busyDet = false; }
}
async function configure(enabled) {
  try {
    det = await api("/api/ml/configure", { enabled,
      threshold: Number($("d-thr").value), people_only: $("d-filter").value === "people", corrected: $("d-corr").checked });
    paintDetect();
  } catch (e) { $("d-note").textContent = e.message; $("d-note").className = "note err"; }
}
$("d-start").onclick = () => configure(true);
$("d-stop").onclick = () => configure(false);
$("d-apply").onclick = () => configure(!!(det && det.enabled));
$("m-apply").onclick = async () => {
  const path = $("m-path").value.trim(); if (!path) return;
  try { det = await api("/api/ml/model", { path }); paintDetect(); $("m-note").textContent = "Loaded " + path; $("m-note").className = "note"; }
  catch (e) { $("m-note").textContent = e.message; $("m-note").className = "note err"; }
};

// ---------------------------------------------------------------- gamepad re-pair
function paintRepair() {
  if (!rep) return;
  const pill = $("r-pill");
  const tone = { triggered: "run", seen: "run", watching: "warn", cooldown: "warn", unavailable: "off", idle: "off" }[rep.state] || "off";
  pill.className = "pill " + tone;
  pill.textContent = (rep.state || "idle").toUpperCase();
  if (document.activeElement !== $("r-on")) $("r-on").checked = !!rep.enabled;
  const p = Math.round((rep.progress || 0) * 100);
  $("r-bar").style.width = p + "%";
  $("r-bar").style.background = p >= 100 ? "var(--ok)" : "var(--warn)";
  $("r-barv").textContent = p + "%";
  $("r-note").textContent = rep.message || "Show the gamepad to the camera while disarmed to clear pairings. HOME + A to pair.";
}
async function pollRepair() {
  if (busyRep) return; busyRep = true;
  try { rep = await api("/api/visual_pair"); paintRepair(); }
  catch (e) { /* console may be offline */ }
  finally { busyRep = false; }
}
$("r-on").onchange = async (e) => {
  try { rep = await api("/api/visual_pair", { enabled: e.target.checked }); paintRepair(); }
  catch (err) { $("r-note").textContent = err.message; $("r-note").className = "note err"; }
};
$("r-now").onclick = async () => {
  $("r-now").disabled = true;
  try { await api("/api/gamepad/repair", {}); $("r-note").textContent = "Pairings cleared. HOME + A on the pad to pair."; $("r-note").className = "note"; }
  catch (e) { $("r-note").textContent = e.message; $("r-note").className = "note err"; }
  finally { setTimeout(() => ($("r-now").disabled = false), 2000); }
};

$("estop").onclick = () => api("/api/estop", {}).catch(() => {});

pollDetect(); pollRepair();
setInterval(pollDetect, 1200);
setInterval(pollRepair, 1500);
