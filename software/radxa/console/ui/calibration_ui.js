const API={
  async request(path,body){const response=await fetch(path,{method:body===undefined?'GET':'POST',headers:body===undefined?{}:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body)});
    if(!response.ok){let message=await response.text();try{message=JSON.parse(message).detail || message;}catch{}throw Error(message);}return response.json();}
};

export function mountCalibration(root){
  root.innerHTML=`<div class="cal-suite">
    <div class="cal-steps"><span data-step="0">1 · Target</span><span data-step="1">2 · Guided capture</span><span data-step="2">3 · Automatic analysis</span><span data-step="3">4 · Results</span></div>
    <div class="cal-layout"><section class="card"><h3>Camera geometry</h3>
      <div class="cal-preview"><img id="calImage" alt="Live calibration camera preview"><div class="caption" id="calCaption">Waiting for camera…</div></div>
      <div class="bar"><button class="act" id="calPause">Pause</button><button class="act" id="calResume">Resume</button><button class="act" id="calAnalyze">Analyze now</button><button class="act" id="calCancel">End session</button></div>
      <p class="cal-error" id="calError"></p><p class="cal-small">The capture owner supplies snapshots; this suite does not change the camera's colour/exposure profile or drive the robot.</p>
    </section><aside class="card"><h3 id="calPhase">Prepare target</h3><div class="cal-guide" id="calGuide">Print the board at actual size, then show it to the camera.</div>
      <progress class="cal-progress" id="calProgress" max="32" value="0"></progress><p id="calCount">0 / 32 accepted views</p>
      <div class="cal-coverage" id="calCoverage"></div><p class="cal-small" id="calQuality"></p>
      <details id="calSetup" open><summary>Target setup</summary><div class="cal-form">
        <label>Pattern<select id="calKind"><option value="charuco">ChArUco (recommended)</option><option value="checkerboard">Checkerboard</option></select></label>
        <label>Square size (mm)<input id="calSquare" type="number" min="5" max="60" value="20"></label>
        <label>Square columns<input id="calColumns" type="number" min="4" max="14" value="8"></label>
        <label>Square rows<input id="calRows" type="number" min="4" max="12" value="6"></label>
        <label>Capture goal<input id="calGoal" type="number" min="24" max="60" value="32"></label>
      </div><p class="cal-small">Keep the bot idle and the camera at a fixed/home pose. Measure the printed square; enter its actual size. Automatic capture requires a sharp, steady target and varied positions/angles.</p>
      <div class="bar"><a class="act" id="calBoardPdf" target="_blank">Print PDF board</a><a class="act" id="calBoardSvg" target="_blank">SVG board</a></div>
      <div class="bar"><button class="act primary" id="calStart">Start automatic calibration</button></div></details>
    </aside></div>
    <section class="card" id="calResult" hidden><h3>Analysis and validation</h3><p id="calResultSummary"></p><div id="calModels"></div>
      <div class="cal-results"><div><h3>Raw</h3><img id="calRaw" alt="Raw camera image" style="width:100%"></div><div><h3>Corrected preview</h3><img id="calCorrected" alt="Undistorted camera preview" style="width:100%"></div></div>
      <p class="cal-small">Compare real straight edges near the centre and corners. Held-out reprojection checks are automatic; they are not camera/ToF mounting calibration. The normal WebRTC stream remains unchanged.</p>
      <div class="bar"><a id="calReport">PDF report</a><a id="calJson">Session JSON</a><a id="calZip">Images + data ZIP</a></div>
    </section><section class="card"><h3>Sensor health and frame timing</h3><p id="calTiming" class="cal-small">Reading camera/ToF diagnostics…</p>
      <p id="calDepthGuide"></p><div id="calDepthGrid" class="cal-depth-grid"></div><p class="cal-small">Raw millimetres; green = usable, amber = inside 50 mm, grey = rejected. Camera/ToF mounting alignment is not yet calibrated.</p>
    </section><section class="card"><h3>Saved sessions</h3><div id="calSessions" class="cal-session-list"></div></section>
  </div>`;
  const get=id=>root.querySelector('#'+id);
  const text=(id,value)=>{get(id).textContent=value;};
  let state=null,disposed=false,polling=false,lastHealth=0,lastPhase=null,lastSession=null;
  const settings=()=>({kind:get('calKind').value,columns:Number(get('calColumns').value),rows:Number(get('calRows').value),square_mm:Number(get('calSquare').value)});
  function boardLinks(){const query=new URLSearchParams(settings());get('calBoardPdf').href='/api/calibration/board.pdf?'+query;get('calBoardSvg').href='/api/calibration/board.svg?'+query;}
  for(const id of ['calKind','calColumns','calRows','calSquare'])get(id).onchange=boardLinks;
  boardLinks();
  async function operation(path,body={}){try{text('calError','');state=await API.request(path,body);paint();}catch(error){text('calError',error.message);}}
  get('calStart').onclick=()=>operation('/api/calibration/start',{board:settings(),goal:Number(get('calGoal').value)});
  for(const [id,action] of [['calPause','pause'],['calResume','resume'],['calAnalyze','analyze'],['calCancel','cancel']])get(id).onclick=()=>operation('/api/calibration/action/'+action);
  function image(id,corrected=false){const element=get(id);element.src='/api/calibration/preview.jpg?corrected='+corrected+'&t='+Date.now();}
  function paint(){
    if(disposed || !state)return;
    const phase=state.phase || 'idle',live=state.live || {},count=state.accepted || 0;
    if(state.id && state.id!==lastSession){
      const board=state.board;get('calKind').value=board.kind;get('calColumns').value=board.columns;
      get('calRows').value=board.rows;get('calSquare').value=board.square_mm;get('calGoal').value=state.goal;
      lastSession=state.id;boardLinks();
    }
    const step=phase==='idle'?0:phase==='analyzing'?2:['ready','failed'].includes(phase)?3:1;
    root.querySelectorAll('[data-step]').forEach(e=>{const n=Number(e.dataset.step);e.classList.toggle('current',n===step);e.classList.toggle('done',n<step);});
    text('calPhase',{idle:'Prepare target',capturing:'Automatic capture',paused:'Capture paused',analyzing:'Comparing lens models',ready:'Calibration passed',failed:'More views needed',cancelled:'Session saved'}[phase] || phase);
    text('calGuide',phase==='ready'?state.message:live.message || state.message || 'Show the printed target');
    text('calCaption',live.message || state.message || 'Live snapshot');
    get('calProgress').max=state.goal || 32;get('calProgress').value=count;text('calCount',`${count} / ${state.goal || 32} accepted views`);
    get('calStart').disabled=phase==='capturing' || phase==='analyzing' || state.available===false;
    get('calPause').disabled=phase!=='capturing';get('calResume').disabled=!['paused','failed'].includes(phase);
    get('calAnalyze').disabled=count<20 || ['analyzing','cancelled','idle'].includes(phase);get('calCancel').disabled=['analyzing','cancelled','idle'].includes(phase);
    if((phase==='capturing' || phase==='analyzing') && phase!==lastPhase)get('calSetup').open=false;
    lastPhase=phase;
    for(const id of ['calKind','calColumns','calRows','calSquare','calGoal'])get(id).disabled=phase==='capturing' || phase==='analyzing';
    const coverage=state.coverage || {cells:Array(9).fill(0),covered_cells:0,scales:[],tilted_views:0};
    const cells=get('calCoverage');cells.replaceChildren();
    coverage.cells.forEach((n,i)=>{const cell=document.createElement('span');cell.textContent=n?`${n} views`:'—';cell.classList.toggle('seen',n>0);cell.classList.toggle('target',state.guide?.cell===i && phase==='capturing');cells.appendChild(cell);});
    text('calQuality',`${coverage.covered_cells}/9 regions · ${coverage.scales.length} distances · ${coverage.tilted_views} tilted views`+
      (live.detected?` · sharpness ${live.sharpness} · ${live.corners} corners`:''));
    if(!['analyzing'].includes(phase))image('calImage');
    const result=state.result;get('calResult').hidden=!result;
    if(result){const q=result.quality;text('calResultSummary',`${result.model} · held-out mean ${q.heldout_mean_px.toFixed(3)} px · p95 ${q.heldout_p95_px.toFixed(3)} px · ${q.passed?'PASS':'MORE VIEWS NEEDED'}`);
      const table=document.createElement('table');table.className='cal-result-table';
      for(const candidate of q.model_comparison){const row=document.createElement('tr');for(const value of [candidate.model,candidate.eligible?`${candidate.heldout_mean_px.toFixed(3)} px held-out`:candidate.error]){const cell=document.createElement('td');cell.textContent=value;row.appendChild(cell);}table.appendChild(row);}get('calModels').replaceChildren(table);
      if(phase==='ready'){image('calRaw');image('calCorrected',true);}const base='/api/calibration/session/'+state.id;
      get('calReport').href=base+'/report.pdf';get('calJson').href=base+'.json';get('calZip').href=base+'/export.zip';
    }
    const sessions=get('calSessions');sessions.replaceChildren();for(const id of state.sessions || []){const link=document.createElement('a');link.href='/api/calibration/session/'+id+'/export.zip';link.textContent='Export '+id; sessions.appendChild(link);}
  }
  async function health(){try{
    const data=await API.request('/api/vision/diagnostics');if(disposed)return;
    const frame=data.camera_frame || {},pair=frame.tof_pair || {},depth=data.depth;
    text('calTiming',`${frame.timestamp_source || 'Camera timing unavailable'} · JPEG ${frame.checksum_verified?'verified':'publication timing only'} · ${pair.matched?`nearest depth ${pair.delta_ms.toFixed(1)} ms (${pair.timestamp_source}, ±${pair.uncertainty_ms ?? '?'} ms)`:'depth pairing not ready'}`);
    text('calDepthGuide',`${depth.valid_zones}/64 usable · ${depth.near_field_zones} near-field · ${depth.inconsistent_zones} inconsistent. ${depth.guidance.join('. ')}`);
    const grid=get('calDepthGrid');grid.replaceChildren();depth.raw_distance_mm.forEach((distance,i)=>{const cell=document.createElement('span');cell.textContent=distance;cell.className=depth.valid[i]?'valid':distance>0 && distance<50?'near':'';grid.appendChild(cell);});
  }catch{if(!disposed)text('calTiming','Sensor diagnostics will appear after the timing update is installed');}}
  async function poll(){if(disposed || polling)return;polling=true;try{state=await API.request('/api/calibration/status');paint();if(Date.now()-lastHealth>2500){lastHealth=Date.now();health();}}catch(error){text('calError','Connection: '+error.message);}finally{polling=false;}}
  poll();const timer=setInterval(poll,1000);
  return ()=>{disposed=true;clearInterval(timer);};
}
