export function mountDetection(root){
  root.innerHTML=`<div class="cal-suite"><div class="cal-layout"><section class="card"><h3>Object detection and tracking</h3>
    <div class="cal-preview"><img id="mlImage" alt="Camera image with detected object boxes"><div class="caption" id="mlCaption">Reading detector…</div></div>
    <p class="cal-small">Boxes and track IDs belong to this captured image. Inference runs on snapshots, not every WebRTC frame. Measured object distances need camera–ToF alignment.</p>
  </section><aside class="card"><h3>ONNX CPU detector</h3><p id="mlMetrics"></p><div class="bar"><button class="act primary" id="mlEnable">Start detection</button><button class="act" id="mlDisable">Stop</button></div>
    <div class="cal-form"><label>Confidence<input id="mlThreshold" type="number" min="0.15" max="0.95" step="0.05" value="0.5"></label>
    <label>Show<select id="mlFilter"><option value="all">Common objects</option><option value="people">People only</option></select></label></div>
    <label><input type="checkbox" id="mlCorrected" checked> Use saved lens correction</label><div class="bar"><button class="act" id="mlApply">Apply settings</button></div>
    <p class="cal-error" id="mlError"></p><p class="cal-small">This viewer does not arm or drive the bot. Track IDs are temporary object tracks, not person identities.</p>
  </aside></div><section class="card"><h3>Current detections</h3><table class="cal-result-table"><thead><tr><th>Track</th><th>Object</th><th>Confidence</th><th>Distance</th></tr></thead><tbody id="mlObjects"></tbody></table></section></div>`;
  const get=id=>root.querySelector('#'+id);let state=null,disposed=false,busy=false;
  async function request(path,body){const response=await fetch(path,{method:body?'POST':'GET',headers:body?{'Content-Type':'application/json'}:{},body:body?JSON.stringify(body):undefined});
    if(!response.ok){const data=await response.json();throw Error(data.detail || 'Request failed');}return response.json();}
  async function configure(enabled){try{get('mlError').textContent='';state=await request('/api/ml/configure',{enabled,threshold:Number(get('mlThreshold').value),people_only:get('mlFilter').value==='people',corrected:get('mlCorrected').checked});paint();}
    catch(error){get('mlError').textContent=error.message;}}
  get('mlEnable').onclick=()=>configure(true);get('mlDisable').onclick=()=>configure(false);get('mlApply').onclick=()=>configure(!!state?.enabled);
  function paint(){if(!state || disposed)return;
    get('mlEnable').disabled=state.enabled || !state.model_installed || !state.runtime_available;get('mlDisable').disabled=!state.enabled;
    get('mlCaption').textContent=`${state.state.toUpperCase()} · ${state.corrected_applied?'rectified':'raw'} image · ${state.message}`;
    get('mlMetrics').textContent=`${state.model} · ${state.provider} · ${state.inference_ms ?? '—'} ms inference · result age ${state.age_ms ?? '—'} ms`;
    if(state.state==='error')get('mlError').textContent=state.message;
    get('mlImage').src='/api/ml/preview.jpg?t='+Date.now();
    const rows=get('mlObjects');rows.replaceChildren();
    for(const item of state.detections || []){const row=document.createElement('tr');for(const value of ['#'+item.track_id,item.label,Math.round(item.score*100)+'%','Unknown — alignment pending']){const cell=document.createElement('td');cell.textContent=value;row.appendChild(cell);}rows.appendChild(row);}
    if(!(state.detections || []).length){const row=document.createElement('tr');const cell=document.createElement('td');cell.colSpan=4;cell.textContent=state.enabled?'No detections above the selected confidence':'Detection is off';row.appendChild(cell);rows.appendChild(row);}
  }
  async function poll(){if(disposed || busy)return;busy=true;try{state=await request('/api/ml/status');paint();}catch(error){get('mlError').textContent=error.message;}finally{busy=false;}}
  poll();const timer=setInterval(poll,1200);return ()=>{disposed=true;clearInterval(timer);};
}
