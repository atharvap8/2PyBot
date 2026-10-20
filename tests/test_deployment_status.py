import importlib
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"software/radxa/console"))
status=importlib.import_module("console.deployment_status")


class DeploymentStatusTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.run=self.root/'run';self.run.mkdir()

    def test_pull_progress_does_not_show_success_before_deployment(self):
        status.atomic(self.root/'progress.json',{'phase':'success','updated_at':1,'finished_at':1})
        status.atomic(self.run/'pull.json',{'phase':'queued','updated_at':10,'commit':'b'*40})
        self.assertEqual(status.indicator(self.root,self.run,now=11)['phase'],'checking')

    def test_completed_deployment_overrides_pull_and_releases_led_after_six_seconds(self):
        status.atomic(self.run/'pull.json',{'phase':'queued','updated_at':10})
        status.atomic(self.root/'progress.json',{'phase':'success','updated_at':11,'finished_at':11})
        self.assertTrue(status.indicator(self.root,self.run,now=12)['led_active'])
        self.assertFalse(status.indicator(self.root,self.run,now=18)['led_active'])

    def test_failed_pull_and_interrupted_build_are_visibly_distinct_from_success(self):
        status.atomic(self.run/'pull.json',{'phase':'failed','updated_at':10,'finished_at':10})
        self.assertEqual(status.indicator(self.root,self.run,now=11)['phase_code'],8)
        status.atomic(self.root/'progress.json',{'phase':'building','updated_at':20,'owner_pid':123})
        with patch.object(status,'process_alive',return_value=False):
            value=status.indicator(self.root,self.run,now=100)
        self.assertEqual(value['phase'],'failed')
        self.assertIn('unexpectedly',value['message'])

    def test_waiting_for_idle_can_survive_the_short_lived_deployer_process(self):
        status.atomic(self.root/'progress.json',{'phase':'waiting_idle','updated_at':20,'owner_pid':123})
        with patch.object(status,'process_alive',return_value=False):
            self.assertEqual(status.indicator(self.root,self.run,now=100)['phase'],'waiting_idle')

    def test_only_matching_worker_job_can_drive_flash_progress(self):
        firmware=self.root/'firmware';firmware.mkdir()
        status.atomic(self.root/'progress.json',{'phase':'flashing','updated_at':20,'firmware_job_id':'mine','progress':5})
        status.atomic(firmware/'current.json',{'id':'other','progress':90,'stage':'restoring'})
        self.assertEqual(status.indicator(self.root,self.run,firmware,now=21)['progress'],5)
        status.atomic(firmware/'current.json',{'id':'mine','progress':92,'stage':'restoring'})
        value=status.indicator(self.root,self.run,firmware,now=21)
        self.assertEqual((value['phase'],value['progress']),('calibrating',92))

    def test_all_animations_are_bounded_and_failure_is_not_green(self):
        for phase in status.PHASES:
            for seconds in (0,.2,1,3,100000):
                frame=status.led_frame(phase,50,seconds)
                self.assertEqual(len(frame),16)
                self.assertTrue(all(0<=channel<=153 for pixel in frame for channel in pixel))
        failure=status.led_frame('failed',0,0)
        success=status.led_frame('success',100,0)
        self.assertGreater(sum(p[0] for p in failure),sum(p[1] for p in failure))
        self.assertGreater(sum(p[1] for p in success),sum(p[0] for p in success))

    def test_deployment_is_one_blended_spin_independent_of_progress(self):
        start=status.led_frame('flashing',10,0)
        self.assertEqual(start,status.led_frame('flashing',90,0))
        self.assertEqual(start,status.led_frame('deploying',0,4))
        self.assertNotEqual(start,status.led_frame('flashing',10,1))
        self.assertTrue(all(sum(pixel)>0 for pixel in start))


if __name__=='__main__':unittest.main()
