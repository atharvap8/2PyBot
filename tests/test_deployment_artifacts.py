import getpass
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from urllib.request import Request

ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('artifact_deploy_test',ROOT/'scripts/radxa/deploy.py')
deploy=importlib.util.module_from_spec(SPEC);sys.modules[SPEC.name]=deploy;SPEC.loader.exec_module(deploy)
LED_SPEC=importlib.util.spec_from_file_location('artifact_led_test',ROOT/'scripts/radxa/deployment_leds.py')
led=importlib.util.module_from_spec(LED_SPEC);LED_SPEC.loader.exec_module(led)


class ArtifactTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.repo=self.root/'repo';self.repo.mkdir()
        self.manager=deploy.Deployment(self.repo,getpass.getuser(),self.root/'state','http://localhost/api/status',self.root/'system')
        self.manager.commit='a'*40
    def bundle(self):
        directory=self.root/'bundle';directory.mkdir()
        names=['firmware/application.bin','firmware/manifest.json','android/app.apk','android/manifest.json']
        checks={}
        for name in names:
            path=directory/name;path.parent.mkdir(exist_ok=True);path.write_bytes(name.encode())
            checks[name]=hashlib.sha256(path.read_bytes()).hexdigest()
        (directory/'bundle.json').write_text(json.dumps({'schema':1,'source_commit':self.manager.commit,'sha256':checks}))
        return directory
    def test_bundle_checks_every_payload_and_exact_commit(self):
        directory=self.bundle();self.manager.verify_bundle(directory)
        (directory/'android/app.apk').write_bytes(b'tampered')
        with self.assertRaisesRegex(RuntimeError,'checksum'):self.manager.verify_bundle(directory)
    def test_bundle_for_another_commit_is_not_accepted(self):
        directory=self.bundle();self.manager.commit='b'*40
        with self.assertRaisesRegex(RuntimeError,'identity'):self.manager.verify_bundle(directory)
    def test_asset_redirect_does_not_leak_the_repo_token(self):
        redirect=deploy.BundleRedirect();request=Request('https://api.github.com/repos/example/assets/1',headers={'Authorization':'Bearer TEST_ONLY'})
        other=redirect.redirect_request(request,None,302,'',{},'https://release-assets.githubusercontent.com/file')
        self.assertIsNone(other.get_header('Authorization'))
        same=redirect.redirect_request(request,None,302,'',{},'https://api.github.com/other')
        self.assertEqual(same.get_header('Authorization'),'Bearer TEST_ONLY')
    def test_spi_encoding_is_grb_with_reset_gaps(self):
        encoded=led.SpiBackend.encode([(255,0,0)])
        self.assertEqual(encoded[:24],[0]*24);self.assertEqual(encoded[-24:],[0]*24)
        self.assertEqual(bytes(encoded[24:27]),bytes.fromhex('924924')) # green=0
        self.assertEqual(bytes(encoded[27:30]),bytes.fromhex('db6db6')) # red=255
    @unittest.skipUnless(shutil.which('git'),'Git is required')
    def test_snapshot_contains_only_the_committed_tree_and_preserves_checkout_notes(self):
        env=dict(os.environ,GIT_AUTHOR_NAME='Deployment Test',GIT_AUTHOR_EMAIL='test@example.invalid',
            GIT_COMMITTER_NAME='Deployment Test',GIT_COMMITTER_EMAIL='test@example.invalid')
        def git(*args):return subprocess.check_output(['git','-C',str(self.repo),*args],env=env,stderr=subprocess.STDOUT,text=True).strip()
        git('init')
        path=self.repo/'software/radxa/console/requirements.txt';path.parent.mkdir(parents=True);path.write_text('')
        (self.repo/'tracked.txt').write_text('committed\n');git('add','.');git('commit','-m','snapshot')
        self.manager.commit=git('rev-parse','HEAD')
        note=self.repo/'handoff.md';note.write_text('keep this note\n')
        self.manager.state_dir.mkdir()
        self.manager.stage_release(deploy.Plan())
        self.assertEqual((self.manager.release/'tracked.txt').read_text(),'committed\n')
        self.assertFalse((self.manager.release/'handoff.md').exists())
        self.assertEqual(note.read_text(),'keep this note\n')
        self.assertEqual(json.loads((self.manager.release/'.deployment-snapshot.json').read_text())['commit'],self.manager.commit)


if __name__=='__main__':unittest.main()
