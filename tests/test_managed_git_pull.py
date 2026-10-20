"""Use real temporary Git repos to check the user's single-command interface."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

SOURCE=Path(__file__).resolve().parents[1]/'scripts/radxa/git_pull.py'
SPEC=importlib.util.spec_from_file_location('managed_pull_test',SOURCE)
pull=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(pull)


@unittest.skipUnless(shutil.which('git'),'Git is required')
class ManagedPullTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.origin=self.root/'origin';self.remote=self.root/'remote.git';self.bot=self.root/'bot'
        self.env=dict(os.environ,GIT_AUTHOR_NAME='Deployment Test',GIT_AUTHOR_EMAIL='test@example.invalid',
            GIT_COMMITTER_NAME='Deployment Test',GIT_COMMITTER_EMAIL='test@example.invalid')
        self.git('init','--bare',str(self.remote))
        self.git('init','--initial-branch=main',str(self.origin))
        (self.origin/'value.txt').write_text('one\n')
        self.git('-C',str(self.origin),'add','value.txt');self.git('-C',str(self.origin),'commit','-m','initial')
        self.git('-C',str(self.origin),'push',str(self.remote),'HEAD:refs/heads/main')
        self.git('clone','--branch','main',str(self.remote),str(self.bot))
        self.run=self.root/'run';self.run.mkdir();self.state=self.root/'state';self.state.mkdir()
        self.config=self.root/'config.json';self.config.write_text(json.dumps({'repo':str(self.bot),'real_git':shutil.which('git'),
            'run_dir':str(self.run),'state_dir':str(self.state),'wait_seconds':3}))
        self.handoff=self.bot/'handoff.md';self.handoff.write_text('preserve user notes\n')
    def git(self,*args):
        return subprocess.check_output(['git',*args],env=self.env,stderr=subprocess.STDOUT,text=True).strip()
    def test_pull_waits_for_matching_deployment_success_and_keeps_untracked_user_work(self):
        (self.origin/'value.txt').write_text('two\n');self.git('-C',str(self.origin),'add','value.txt')
        self.git('-C',str(self.origin),'commit','-m','update');self.git('-C',str(self.origin),'push',str(self.remote),'HEAD:refs/heads/main')
        target=self.git('-C',str(self.origin),'rev-parse','HEAD')
        # A retry of the same commit must not consume the previous failure.
        (self.state/'progress.json').write_text(json.dumps({'commit':target,'phase':'failed','message':'old failure','updated_at':1}))
        stop=threading.Event()
        def manager():
            while not stop.is_set():
                try:event=json.loads((self.run/'pull.json').read_text())
                except (OSError,ValueError):event={}
                if event.get('phase')=='queued':
                    (self.state/'progress.json').write_text(json.dumps({'commit':target,'phase':'success','message':'verified','updated_at':time.time()}));return
                time.sleep(.01)
        worker=threading.Thread(target=manager);worker.start()
        try:
            with patch.dict(os.environ,{'BOT_DEPLOY_CONFIG':str(self.config)}):code=pull.main(['-C',str(self.bot),'pull'])
        finally:stop.set();worker.join(timeout=2)
        self.assertEqual(code,0);self.assertEqual((self.bot/'value.txt').read_text(),'two\n')
        self.assertEqual(self.handoff.read_text(),'preserve user notes\n')
        self.assertTrue((self.run/'request').exists())
        self.assertEqual(json.loads((self.run/'pull.json').read_text())['commit'],target)
    def test_tracked_local_edits_are_rejected_without_reset_or_cleanup(self):
        (self.bot/'value.txt').write_text('user edit\n')
        with patch.dict(os.environ,{'BOT_DEPLOY_CONFIG':str(self.config)}):code=pull.main(['-C',str(self.bot),'pull'])
        self.assertEqual(code,2);self.assertEqual((self.bot/'value.txt').read_text(),'user edit\n')
        self.assertEqual(json.loads((self.run/'pull.json').read_text())['phase'],'failed')
    def test_global_option_parser_does_not_confuse_commit_message_with_pull(self):
        self.assertEqual(pull.command_position(['-C','somewhere','commit','-m','pull']),2)
    @unittest.skipUnless(os.name=='posix','Linux advisory-lock contract')
    def test_active_deployment_blocks_a_second_pull_without_overwriting_progress(self):
        import fcntl
        with (self.run/'operation.lock').open('a+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.dict(os.environ,{'BOT_DEPLOY_CONFIG':str(self.config)}):code=pull.main(['-C',str(self.bot),'pull'])
        self.assertEqual(code,75)
        self.assertFalse((self.run/'pull.json').exists())
        self.assertFalse((self.run/'request').exists())


if __name__=='__main__':unittest.main()
