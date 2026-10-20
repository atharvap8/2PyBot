"""Deployment transaction tests with fake services/API, never a real controller."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SOURCE=Path(__file__).resolve().parents[1]/'scripts/radxa/deploy.py'
SPEC=importlib.util.spec_from_file_location('one_pull_test_deploy',SOURCE)
deploy=importlib.util.module_from_spec(SPEC);sys.modules[SPEC.name]=deploy;SPEC.loader.exec_module(deploy)


class Fake(deploy.Deployment):
    def __init__(self,root):
        root=Path(root);repo=root/'repo';repo.mkdir()
        state=root/'state';state.mkdir()
        config=root/'config.json';config.write_text(json.dumps({'one_pull':True,'auto_firmware':True,'run_dir':str(root/'run'),'firmware_root':str(root/'firmware')}))
        super().__init__(repo,'radxa',state,'http://localhost/api/status',root/'system',config)
        self.head='b'*40;self.events=[];self.armed=False;self.error=None;self.worker_job=None;self.head_changes=False
        self.installed={'build_id':'old'};self.change_after_lock=False
        self.state_file.write_text(json.dumps({'commit':'a'*40,'release':str(root/'previous'),'firmware_fingerprint':'old'}))
    def git(self,*args,check=True):
        if args==('rev-parse','--absolute-git-dir'):output=str(self.repo/'.git')
        elif args==('rev-parse','HEAD'):output=self.head
        else:output=''
        return subprocess.CompletedProcess(args,0,output,'')
    def changed_paths(self,previous):return [deploy.APP+'console/server.py'],False
    def fingerprint(self):return 'f'*64
    def preflight(self):
        self.events.append('preflight')
        if self.armed:raise deploy.DeploymentDeferred('local SELECT required')
    def stage_release(self,plan):self.events.append('stage');self.release=self.state_dir/'new-release'
    def prepare(self,plan):
        self.events.append('prepare')
        if self.error:raise RuntimeError(self.error)
        if self.head_changes:self.head='c'*40
    def prepare_firmware(self,fingerprint):self.events.append('build');return {'build_id':'1'*64}
    def switch_release(self,path):self.events.append(('switch',str(path)))
    def apply(self,plan):self.events.append('activate')
    def install_runtime_worker(self):self.events.append('worker')
    def wait_for_health(self,stream):self.events.append('health')
    def wait_controller(self):self.events.append('controller-ready');return {'connected':True}
    def stream_running(self):return True
    def run(self,args,**kwargs):self.events.append(list(args))
    def api(self,path,body=None):
        if path=='/api/internal/deployment/lock':
            self.events.append('lock')
            if self.change_after_lock:self.head='c'*40
            return {'locked':True}
        if path=='/api/internal/deployment/release':self.events.append('release');return {'released':True}
        if path=='/api/ready':return {'ready':True}
        if path=='/api/firmware/status':return {'worker_ready':True,'installed':self.installed,'job':self.worker_job}
        if path=='/api/internal/deployment/firmware':
            self.events.append('queue-flash');self.worker_job={'id':'job','status':'succeeded','stage':'complete','progress':100};self.installed={'build_id':body['build_id']};return self.worker_job
        raise AssertionError(path)


class OnePullTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.manager=Fake(self.temp.name)
    def test_armed_robot_defers_before_build_restart_or_flash(self):
        self.manager.armed=True
        with self.assertRaises(deploy.DeploymentDeferred):self.manager.deploy()
        self.assertEqual(self.manager.events,['preflight'])
        self.assertEqual(self.manager.read_state()['commit'],'a'*40)
        self.assertEqual(json.loads((self.manager.state_dir/'progress.json').read_text())['phase'],'waiting_idle')
    def test_failed_preparation_never_locks_or_changes_active_release(self):
        self.manager.error='bad dependency'
        with self.assertRaisesRegex(RuntimeError,'bad dependency'):self.manager.deploy()
        self.assertEqual(self.manager.events,['preflight','stage','prepare'])
        self.assertEqual(self.manager.read_state()['commit'],'a'*40)
    def test_second_pull_during_build_never_activates(self):
        self.manager.head_changes=True
        with self.assertRaises(deploy.GitBusy):self.manager.deploy()
        self.assertNotIn('activate',self.manager.events)
        self.assertNotIn('lock',self.manager.events)
    def test_second_pull_after_lock_releases_without_switching_or_restarting(self):
        self.manager.change_after_lock=True
        with self.assertRaises(deploy.GitBusy):self.manager.deploy()
        self.assertIn('release',self.manager.events)
        self.assertNotIn('activate',self.manager.events)
    def test_lock_precedes_restart_and_flash_and_handback_is_last(self):
        self.assertTrue(self.manager.deploy())
        events=self.manager.events
        self.assertLess(events.index('lock'),events.index('activate'))
        self.assertLess(events.index('activate'),events.index('queue-flash'))
        self.assertLess(events.index('queue-flash'),events.index('release'))
        self.assertEqual(events[-1],'controller-ready')
        self.assertEqual(self.manager.read_state()['commit'],'b'*40)
        self.assertEqual(json.loads((self.manager.state_dir/'progress.json').read_text())['phase'],'success')
    def test_failure_is_latched_instead_of_reflashing_every_timer_tick(self):
        self.manager.error='bad image'
        with self.assertRaises(RuntimeError):self.manager.deploy()
        count=len(self.manager.events)
        self.assertFalse(self.manager.deploy())
        self.assertEqual(len(self.manager.events),count)
    def test_bootstrap_marker_does_not_claim_firmware_was_already_deployed(self):
        state=self.manager.read_state();state.update(commit='b'*40,bootstrap_host_only=True)
        self.manager.state_file.write_text(json.dumps(state))
        self.assertTrue(self.manager.deploy())
        self.assertIn('build',self.manager.events)
        self.assertFalse(self.manager.read_state().get('bootstrap_host_only',False))
    def test_dry_run_has_no_api_build_or_state_changes(self):
        self.assertTrue(self.manager.deploy(dry_run=True));self.assertEqual(self.manager.events,[])
        self.assertFalse((self.manager.state_dir/'progress.json').exists())
    def test_serial_ready_alone_is_not_ready_if_motor_link_mode_or_fault_is_bad(self):
        status={'connected':True,'dump_complete':True,'derived':{'HUBMOTOR':1,'FLASHLOCK':1},
            'telemetry':{'stale':False,'startup_ready':True,'host_ready':True,'firmware_lock':False,'balancing':False,'hub':{
                'link_ok':1,'mode_ok':1,'fault_l':0,'fault_r':0,'current_cmd_l':0,'current_cmd_r':0}}}
        self.assertTrue(deploy.controller_ready(status))
        for key,value in [('link_ok',0),('mode_ok',0),('fault_l',2),('current_cmd_l',1)]:
            old=status['telemetry']['hub'][key];status['telemetry']['hub'][key]=value
            self.assertFalse(deploy.controller_ready(status));status['telemetry']['hub'][key]=old
        status['telemetry']['firmware_lock']=True
        self.assertFalse(deploy.controller_ready(status))
        status['telemetry']['firmware_lock']=False;status['telemetry']['host_ready']=False
        self.assertFalse(deploy.controller_ready(status))
    def test_interrupted_deployer_does_not_restart_an_active_flash_worker(self):
        root=Path(self.manager.config['firmware_root']);root.mkdir()
        token='d'*32
        (root/'deployment-hold.json').write_text(json.dumps({'commit':'b'*40,'token':token,'locked':True}))
        job={'id':'existing','deployment_token':token,'status':'running'}
        (root/'current.json').write_text(json.dumps(job));self.manager.worker_job=job
        def finish(identifier):
            self.assertEqual(identifier,'existing');self.manager.events.append('finish-existing')
            self.manager.installed={'build_id':'1'*64}
            self.manager.worker_job=dict(job,status='succeeded')
            (root/'current.json').write_text(json.dumps(self.manager.worker_job))
        self.manager.wait_firmware=finish
        self.assertTrue(self.manager.deploy())
        self.assertLess(self.manager.events.index('finish-existing'),self.manager.events.index('worker'))
        self.assertNotIn('queue-flash',self.manager.events)
    def test_unconfirmed_interrupted_lock_is_neither_released_nor_restarted(self):
        root=Path(self.manager.config['firmware_root']);root.mkdir()
        (root/'deployment-hold.json').write_text(json.dumps({'commit':'b'*40,'token':'d'*32,'locked':False}))
        with self.assertRaisesRegex(RuntimeError,'Interrupted maintenance acquisition'):self.manager.deploy()
        self.assertEqual(self.manager.events,[])
        self.assertTrue((root/'deployment-hold.json').exists())
    def test_host_rollback_is_verified_before_handing_back_serial(self):
        checks=[]
        def health(stream):
            checks.append(stream);self.manager.events.append('rollback-health' if len(checks)>1 else 'health')
            if len(checks)==1:raise RuntimeError('new host failed')
        self.manager.wait_for_health=health
        with self.assertRaisesRegex(RuntimeError,'new host failed'):self.manager.deploy()
        self.assertEqual(len(checks),2)
        self.assertLess(self.manager.events.index('rollback-health'),self.manager.events.index('release'))
        self.assertEqual(self.manager.read_state()['commit'],'a'*40)
    def test_unhealthy_rollback_keeps_controller_maintenance_held(self):
        def health(stream):raise RuntimeError('host unavailable')
        self.manager.wait_for_health=health
        with self.assertRaisesRegex(RuntimeError,'host unavailable'):self.manager.deploy()
        self.assertNotIn('release',self.manager.events)
        progress=json.loads((self.manager.state_dir/'progress.json').read_text())
        self.assertIn('rollback was not verified',progress['recovery'][0])


if __name__=='__main__':unittest.main()
