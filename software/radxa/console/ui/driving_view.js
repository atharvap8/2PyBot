/* Shared measured-depth scene and current-STL robot renderer, with no CDN. */
export function prepareModel(model) {
  return model.parts.flatMap(part => {
    const v = part.vertices;
    const base = part.color.match(/[a-f\d]{2}/gi).map(x => parseInt(x, 16));
    const faces = [];
    for (let i = 0; i < part.faces.length; i += 3) {
      const xyz = part.faces.slice(i, i + 3).map(index => v.slice(index * 3, index * 3 + 3));
      const [a, b, c] = xyz;
      const u = b.map((x, k) => x - a[k]), w = c.map((x, k) => x - a[k]);
      const n = [u[1]*w[2]-u[2]*w[1], u[2]*w[0]-u[0]*w[2], u[0]*w[1]-u[1]*w[0]];
      const length = Math.hypot(...n) || 1;
      const light = .55 + .45 * Math.abs((n[0]*.3-n[1]*.4+n[2]*.85) / length);
      faces.push({ xyz, color: `rgb(${base.map(c => Math.round(c*light)).join(',')})` });
    }
    return faces;
  });
}

export function drawScene(canvas, triangles, tel, tof) {
  const w = canvas.width = Math.max(1, Math.round(canvas.clientWidth * devicePixelRatio));
  const h = canvas.height = Math.max(1, Math.round(canvas.clientHeight * devicePixelRatio));
  const g = canvas.getContext('2d'), scale = Math.min(w,h)*2.35;
  const project = ([x,y,z]) => {
    const perspective = 1 / Math.max(.65, 1 + y*.72);
    return [w*.5 + (x-y*.25)*scale*perspective, h*.85-(z+y*.27)*scale*perspective];
  };
  const line = (a,b,color,width=1) => {
    const p=project(a), q=project(b); g.strokeStyle=color; g.lineWidth=width*devicePixelRatio;
    g.beginPath(); g.moveTo(...p); g.lineTo(...q); g.stroke();
  };
  g.fillStyle='#dfe2e5'; g.fillRect(0,0,w,h);
  for (let y=0; y<=3; y+=.25) line([-.9,y,0],[.9,y,0],'#c9cdd1');
  for (let x=-.9; x<=.9; x+=.15) line([x,0,0],[x,3,0],'#c9cdd1');
  const stopping=tof.stopping_distance_m ?? .35;
  for (const sign of [-1,1]) {
    let previous=[sign*(tof.corridor_half_width_m ?? .14),0,.002];
    for (let y=.05; y<=Math.min(2,stopping)+.01; y+=.05) {
      const point=[sign*((tof.corridor_half_width_m ?? .14)+Math.min(.3,.5*(tof.turn_envelope_curvature ?? 0)*y*y)),y,.002];
      line(previous,point,tof.guard_enabled?'#778c9b':'#adb5bc',2); previous=point;
    }
  }
  // Draw only actual accepted returns. Ground reference is geometry, not sensed terrain.
  if (!tof.stale) for (const point of (tof.points || []).slice().sort((a,b)=>b.y_m-a.y_m)) {
    const p=project([point.x_m,point.y_m,point.z_m]);
    line([point.x_m,point.y_m,0],[point.x_m,point.y_m,point.z_m],'#b9a275');
    g.fillStyle=point.confirmed?'#89775a':'#bca989';
    g.globalAlpha=Math.max(.3,point.confidence ?? .4);
    g.beginPath(); g.arc(...p,Math.max(2,5/(1+point.y_m*.7))*devicePixelRatio,0,Math.PI*2); g.fill(); g.globalAlpha=1;
  }
  const angle=(tel.pitch || 0)*Math.PI/180, s=Math.sin(angle), c=Math.cos(angle);
  const faces=triangles.map(face=> {
    const xyz=face.xyz.map(([x,y,z])=>[x,y*c+(z-.05)*s,.05+(z-.05)*c-y*s]);
    return {...face,xyz,depth:xyz.reduce((sum,v)=>sum+v[1]-.25*v[2]+.1*v[0],0)};
  }).sort((a,b)=>b.depth-a.depth);
  for (const face of faces) {
    const points=face.xyz.map(project); g.beginPath(); g.moveTo(...points[0]);
    g.lineTo(...points[1]); g.lineTo(...points[2]); g.closePath(); g.fillStyle=face.color; g.fill();
  }
  g.fillStyle='#525b63'; g.font=`${11*devicePixelRatio}px system-ui`;
  g.fillText('Current CAD assembly · measured depth returns',12*devicePixelRatio,20*devicePixelRatio);
  g.fillText(tof.stale || !(tof.points?.length) ? 'Terrain unknown — no usable depth' : 'Approximate optical projection · reference floor',12*devicePixelRatio,h-12*devicePixelRatio);
}

export function drawTrail(canvas, path, tel) {
  const w=canvas.width=Math.max(1,canvas.clientWidth*devicePixelRatio);
  const h=canvas.height=Math.max(1,canvas.clientHeight*devicePixelRatio);
  const g=canvas.getContext('2d'), points=path.concat([[tel.x_m||0,tel.y_m||0]]);
  const minX=Math.min(...points.map(p=>p[0])), maxX=Math.max(...points.map(p=>p[0]));
  const minY=Math.min(...points.map(p=>p[1])), maxY=Math.max(...points.map(p=>p[1]));
  const span=Math.max(1.5,maxX-minX+1,maxY-minY+1), scale=Math.min(w,h)/span;
  const X=x=>w/2+(x-(minX+maxX)/2)*scale, Y=y=>h/2-(y-(minY+maxY)/2)*scale;
  g.fillStyle='#e8eaed'; g.fillRect(0,0,w,h); g.strokeStyle='#cdd1d5'; g.lineWidth=1;
  for (let i=.1; i<1; i+=.1) { g.beginPath(); g.moveTo(i*w,0); g.lineTo(i*w,h); g.moveTo(0,i*h); g.lineTo(w,i*h); g.stroke(); }
  g.strokeStyle='#728a99'; g.lineWidth=2*devicePixelRatio; g.beginPath();
  path.forEach((p,i)=>i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1]))); g.stroke();
  g.save(); g.translate(X(tel.x_m||0),Y(tel.y_m||0)); g.rotate(-(tel.heading||0)*Math.PI/180);
  g.fillStyle='#4d6576'; g.beginPath(); g.moveTo(12*devicePixelRatio,0); g.lineTo(-8*devicePixelRatio,7*devicePixelRatio); g.lineTo(-8*devicePixelRatio,-7*devicePixelRatio); g.closePath(); g.fill(); g.restore();
}
