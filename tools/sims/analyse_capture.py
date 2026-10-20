"""Compact read-only summary of a control capture; no network or actuation."""
import json
from pathlib import Path
import sys

folder=Path(sys.argv[1]);rows=[]
for line in (folder/'live-trace.jsonl').read_text().splitlines():
    try:r=json.loads(line)
    except ValueError:continue
    if r.get('debug'):rows.append(r)
if not rows:raise SystemExit('No samples')
origin=rows[0]['host_time']
def compact(r):
    d=r['debug'];h=r.get('hub') or {}
    return dict(t=round(r['host_time']-origin,3),bal=r.get('balancing'),
        **{k:d.get(k) for k in ['pF','w','v','vCmd','u','fwd','driveReq','driveRef','satV','satA','acPolicy','acBlend']},
        cmdL=h.get('current_cmd_l'),cmdR=h.get('current_cmd_r'),actualL=h.get('current_l'),actualR=h.get('current_r'),rpmL=h.get('rpm_l'),rpmR=h.get('rpm_r'))
print(json.dumps({'rows':len(rows),'seconds':round(rows[-1]['host_time']-origin,2),'max_stick':max(abs(r['debug'].get('fwd',0)) for r in rows)}))
selected={0,len(rows)-1}
for i,r in enumerate(rows):
    if i and r.get('balancing')!=rows[i-1].get('balancing'):
        selected.update(range(max(0,i-12),min(len(rows),i+3)))
full=[i for i,r in enumerate(rows) if abs(r['debug'].get('fwd',0))>.95]
if full:selected.update([full[0],full[-1],max(full,key=lambda i:abs(rows[i]['debug'].get('pF',0)))])
for i in sorted(selected):print(json.dumps(compact(rows[i])))
