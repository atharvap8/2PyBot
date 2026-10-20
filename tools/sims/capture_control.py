"""Read-only 20 Hz API capture. Operator drives locally; this script sends no commands."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--go',action='store_true',help='Start recording only after the operator says GO')
if not parser.parse_args().go:
    print('Recorder prepared. NOT recording. Waiting for operator GO.');raise SystemExit(0)
base='http://10.42.0.1:8080'
def get(path):
    with urllib.request.urlopen(base+path,timeout=2) as response:return json.load(response)

parameters=get('/api/params')
values={p['key']:p['value'] for p in parameters['params']};derived=parameters.get('derived',{})
if derived.get('FALLPREVENT')!=0 or abs(derived.get('VDRIVE',0)-values.get('DRIVEV',1))>.001:
    raise SystemExit('Unrestricted firmware is not active; recorder did not start.')
folder=ROOT/'build/control-audit'/('capture-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S'))
folder.mkdir(parents=True,exist_ok=False)
print('Recording to '+str(folder),flush=True)
(folder/'live-params.json').write_text(json.dumps(parameters,indent=2))
deadline=time.monotonic()+600;was_balancing=False;stopped=None
with (folder/'live-trace.jsonl').open('w') as file:
    while time.monotonic()<deadline:
        start=time.monotonic()
        try:
            s=get('/api/status');tel=s['telemetry']
            row={'host_time':time.time(),'connected':s['connected'],'stale':tel.get('stale'),
                'debug':s['debug'],'hub':tel.get('hub'),'raw_pitch':tel.get('pitch'),
                'balancing':tel.get('balancing'),'mcu_ms':tel.get('ms')}
            file.write(json.dumps(row)+'\n');file.flush()
            if was_balancing and not row['balancing'] and stopped is None:stopped=time.monotonic()
            was_balancing=bool(row['balancing'])
            if stopped is not None and time.monotonic()-stopped>3:break
        except Exception as error:
            file.write(json.dumps({'host_time':time.time(),'error':str(error)})+'\n');file.flush()
        time.sleep(max(0,.05-(time.monotonic()-start)))
(folder/'live-log.json').write_text(json.dumps(get('/api/log?n=400'),indent=2))
