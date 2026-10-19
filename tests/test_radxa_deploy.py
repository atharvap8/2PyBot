import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[1] / "scripts/radxa/deploy.py"
SPEC = importlib.util.spec_from_file_location("radxa_deploy", SOURCE)
deploy = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = deploy
SPEC.loader.exec_module(deploy)


class FakeDeployment(deploy.Deployment):
    def __init__(self, directory, changed):
        self.root = Path(directory)
        repo = self.root / "repo"
        repo.mkdir()
        (repo / ".git").mkdir()
        super().__init__(repo, "radxa", self.root / "state", "http://localhost/status", self.root / "system")
        self.paths = changed
        self.head = "b" * 40
        self.dirty = False
        self.events = []
        self.prepare_error = None
        self.health_error = None
        self.head_changes = False
        self.state_dir.mkdir()
        self.state_file.write_text(json.dumps({"commit": "a" * 40}), encoding="utf-8")

    def git(self, *arguments, check=True):
        if arguments == ("rev-parse", "--absolute-git-dir"):
            output = str(self.repo / ".git")
        elif arguments == ("status", "--porcelain", "--untracked-files=no"):
            output = " M file.py" if self.dirty else ""
        elif arguments == ("rev-parse", "HEAD"):
            output = self.head
        else:
            output = ""
        return subprocess.CompletedProcess(arguments, 0, output, "")

    def changed_paths(self, previous):
        return self.paths, not previous

    def prepare(self, plan):
        self.events.append("prepare")
        if self.prepare_error:
            raise self.prepare_error
        if self.head_changes:
            self.head = "c" * 40

    def apply(self, plan):
        self.events.append("apply")

    def stream_running(self):
        return True

    def wait_for_health(self, require_stream):
        self.events.append(("health", require_stream))
        if self.health_error:
            raise self.health_error

    def install_monitor(self):
        self.events.append("monitor")


class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=os.environ.get("PYBOT_TEST_TEMP"))
        self.addCleanup(self.temporary.cleanup)

    def instance(self, paths):
        return FakeDeployment(self.temporary.name, paths)

    def test_android_cad_firmware_and_docs_do_not_restart_handlers(self):
        instance = self.instance([
            "software/android/app/build.gradle.kts", "hardware/cad/enclosure/rev5/base.stl",
            "firmware/BaseLink/config.h", "README.md", "software/radxa/console/README.md",
            "software/radxa/deployment/mediamtx.yml",
        ])
        self.assertTrue(instance.deploy())
        self.assertEqual(instance.events, [])
        self.assertEqual(instance.read_state()["commit"], "b" * 40)

    def test_unchanged_commit_is_a_no_op(self):
        instance = self.instance([deploy.APP + "console/server.py"])
        instance.head = "a" * 40
        self.assertFalse(instance.deploy())
        self.assertEqual(instance.events, [])

    def test_dependency_failure_keeps_previous_marker_and_services(self):
        instance = self.instance([deploy.APP + "requirements.txt"])
        instance.prepare_error = RuntimeError("dependency installation failed")
        with self.assertRaisesRegex(RuntimeError, "dependency installation"):
            instance.deploy()
        self.assertEqual(instance.events, ["prepare"])
        self.assertEqual(instance.read_state()["commit"], "a" * 40)

    def test_failed_health_does_not_record_new_commit(self):
        instance = self.instance([deploy.APP + "console/server.py"])
        instance.health_error = RuntimeError("unhealthy")
        with self.assertRaisesRegex(RuntimeError, "unhealthy"):
            instance.deploy()
        self.assertEqual(instance.events, ["prepare", "apply", ("health", True)])
        self.assertEqual(instance.read_state()["commit"], "a" * 40)

    def test_completed_restart_records_new_commit(self):
        instance = self.instance([deploy.APP + "ui/app.js"])
        self.assertTrue(instance.deploy())
        self.assertEqual(instance.events, ["prepare", "apply", ("health", True)])
        self.assertTrue(instance.read_state()["actions"]["console"])

    def test_dirty_checkout_defers_without_installing(self):
        instance = self.instance([deploy.APP + "console/server.py"])
        instance.dirty = True
        with self.assertRaises(deploy.GitBusy):
            instance.deploy()
        self.assertEqual(instance.events, [])
        self.assertEqual(instance.read_state()["commit"], "a" * 40)

    def test_git_lock_defers_without_installing(self):
        instance = self.instance([deploy.APP + "console/server.py"])
        (instance.repo / ".git/index.lock").touch()
        with self.assertRaises(deploy.GitBusy):
            instance.deploy()
        self.assertEqual(instance.events, [])

    def test_second_pull_during_preparation_defers_apply(self):
        instance = self.instance([deploy.APP + "native/2pybot_hwstream.c"])
        instance.head_changes = True
        with self.assertRaises(deploy.GitBusy):
            instance.deploy()
        self.assertEqual(instance.events, ["prepare"])
        self.assertEqual(instance.read_state()["commit"], "a" * 40)

    def test_dry_run_does_not_update_services_or_state(self):
        instance = self.instance([deploy.APP + "mediamtx.yml"])
        self.assertTrue(instance.deploy(dry_run=True))
        self.assertEqual(instance.events, [])
        self.assertEqual(instance.read_state()["commit"], "a" * 40)

    def test_component_change_plans(self):
        plan = deploy.plan_changes([deploy.APP + "mediamtx.yml"])
        self.assertTrue(plan.mediamtx and plan.console)
        self.assertFalse(plan.native or plan.dependencies)
        plan = deploy.plan_changes([deploy.APP + "native/2pybot_hwstream.c"])
        self.assertTrue(plan.native and plan.console)
        self.assertFalse(plan.mediamtx)
        plan = deploy.plan_changes([deploy.MONITOR + "2pybot-deploy.timer"])
        self.assertTrue(plan.monitor)
        self.assertFalse(plan.console or plan.mediamtx)

    def test_config_install_backs_up_once_and_skips_identical_content(self):
        instance = self.instance([])
        instance.commit = "b" * 40
        target = instance.system_root / "etc/2pybot/mediamtx.yml"
        target.parent.mkdir(parents=True)
        target.write_bytes(b"original")
        self.assertTrue(instance.write_bytes(target, b"replacement"))
        self.assertFalse(instance.write_bytes(target, b"replacement"))
        self.assertTrue(instance.write_bytes(target, b"second replacement"))
        backup = instance.state_dir / "backups" / instance.commit / "etc/2pybot/mediamtx.yml"
        self.assertEqual(backup.read_bytes(), b"original")
        self.assertEqual(target.read_bytes(), b"second replacement")

    def test_boot_and_tof_assets_are_selected_for_repo_deployment(self):
        for path in (deploy.APP + "40-tof.conf", deploy.APP + "45-boot-ready.conf",
                     "scripts/radxa/boot_gate.py", deploy.MONITOR + "2pybot-boot-gate.service",
                     deploy.MONITOR + "86-2pybot-boot-gate.rules"):
            plan = deploy.plan_changes([path])
            self.assertTrue(plan.boot and plan.console, path)
            self.assertFalse(plan.native or plan.mediamtx, path)

    def test_boot_install_resolves_paths_without_resetting_live_robot(self):
        instance = self.instance([])
        instance.commit = "b" * 40
        monitor = instance.repo / deploy.MONITOR
        monitor.mkdir(parents=True)
        instance.app.mkdir(parents=True)
        (monitor / "2pybot-boot-gate.service").write_text("User=__USER__\nExecStart=__VENV__/bin/python3 __REPO__/scripts/radxa/boot_gate.py\n")
        (monitor / "86-2pybot-boot-gate.rules").write_text('ACTION=="add"\n')
        for name in ("40-tof.conf", "45-boot-ready.conf"):
            (instance.app / name).write_text("[Service]\n")
        instance.run = lambda command, **kwargs: instance.events.append(list(command))
        deploy.Deployment.apply(instance, deploy.Plan(boot=True))
        unit = (instance.system_root / "etc/systemd/system/2pybot-boot-gate.service").read_text()
        self.assertIn("User=radxa", unit)
        self.assertNotIn("__REPO__", unit)
        self.assertIn(str(instance.venv), unit)
        self.assertIn(["systemctl", "enable", "2pybot-boot-gate.service"], instance.events)
        self.assertFalse(any("start" in command or "restart" in command for command in instance.events))

    def test_root_monitor_reads_git_as_checkout_owner(self):
        instance = deploy.Deployment(self.temporary.name, "radxa", Path(self.temporary.name) / "state", "http://localhost")
        completed = subprocess.CompletedProcess([], 0, "commit\n", "")
        with patch.object(deploy.os, "geteuid", return_value=0, create=True), patch.object(deploy.subprocess, "run", return_value=completed) as run:
            self.assertEqual(instance.git("rev-parse", "HEAD").stdout, "commit\n")
        self.assertEqual(run.call_args.args[0][:4], ["runuser", "-u", "radxa", "--"])
        self.assertIn("git", run.call_args.args[0])

    def test_git_errors_include_the_original_diagnostic(self):
        instance = deploy.Deployment(self.temporary.name, "radxa", Path(self.temporary.name) / "state", "http://localhost")
        completed = subprocess.CompletedProcess([], 128, "", "fatal: detected dubious ownership\n")
        with patch.object(deploy.subprocess, "run", return_value=completed):
            with self.assertRaisesRegex(RuntimeError, "dubious ownership"):
                instance.git("rev-parse", "HEAD")
            self.assertEqual(instance.git("cat-file", "-e", "missing", check=False).returncode, 128)


if __name__ == "__main__":
    unittest.main()
