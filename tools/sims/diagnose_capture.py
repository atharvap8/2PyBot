"""Decompose the recorded production velocity-control acceleration terms."""
import json
import math
from pathlib import Path
import sys

folder=Path(sys.argv[1])
rows=[json.loads(line) for line in (folder/'live-trace.jsonl').read_text().splitlines()]
rows=[r for r in rows if r.get('debug')]
p={x['key']:x['value'] for x in json.loads((folder/'live-params.json').read_text())['params']}
t0=rows[0]['host_time']
for t in (5.699,5.888,6.096,6.329,6.410):
    r=min(rows,key=lambda r:abs(r['host_time']-t0-t));d=r['debug'];h=r.get('hub') or {}
    velocity=-p['K2']*(d['v']-d['driveEff'])
    attitude=-(p['K3']*math.radians(d['pF'])+p['K4']*math.radians(d['w']))
    print(json.dumps({'t':round(r['host_time']-t0,3),'pitch':d['pF'],'rate':d['w'],'velocity':d['v'],
        'balance_gain_scale':d['balScale'],'velocity_term':round(velocity,3),'attitude_full':round(attitude,3),
        'calculated_old_accel':round(velocity+attitude*d['balScale'],3),'logged_accel':d['u'],
        'unsoftened_accel':round(velocity+attitude,3),'requested_A':h.get('current_cmd_l'),'reported_A':h.get('current_l')}))
