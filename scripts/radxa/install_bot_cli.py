#!/usr/bin/env python3
"""Install the bot command environment and directory-independent command launchers."""
import argparse
import json
import os
from pathlib import Path
import shlex
import stat
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
MARKER = "# 2pybot-managed command environment"
ALIASES = {"build-firmware": "build-firmware", "flash-fw": "flash-fw", "deploy-status": "deploy-status",
           "calibration": "calibration", "diagnostics": "diagnostics", "bot-backup": "backup", "bot-restore": "restore"}


def ssh_activation(path, environment, account):
    # Append one bounded block; retain the user's shell setup and symlinks.
    path = path.resolve()
    original = path.read_text(encoding="utf-8") if path.exists() else ""
    start = "# >>> 2pybot command environment >>>"
    end = "# <<< 2pybot command environment <<<"
    block = (start + '\ncase "$-" in\n  *i*)\n'
             '    if [ -n "${SSH_CONNECTION:-}${SSH_TTY:-}" ]; then\n'
             '      case "${VIRTUAL_ENV:-}" in\n'
             '        ""|' + shlex.quote(str(environment)) + ')\n'
             '          . ' + shlex.quote(str(environment / "bin/activate")) + '\n'
             '          ;;\n      esac\n    fi\n    ;;\nesac\n' + end + '\n')
    if start in original or end in original:
        if original.count(start) != 1 or original.count(end) != 1:
            raise RuntimeError("Incomplete/duplicate bot activation block; left untouched: " + str(path))
        first = original.index(start)
        last = original.index(end, first) + len(end)
        if original[last:last+1] == "\n":
            last += 1
        content = original[:first] + block + original[last:]
    else:
        content = original + ("\n" if original and not original.endswith("\n") else "") + "\n" + block
    if content == original:
        return
    temporary = path.with_name(path.name + ".2pybot-new")
    temporary.write_text(content, encoding="utf-8")
    temporary.chmod(stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o644)
    os.chown(temporary, account.pw_uid, account.pw_gid)
    os.replace(temporary, path)


def managed(path, content, executable=False):
    if path.exists():
        old = path.read_text(encoding="utf-8")
        if MARKER not in old:
            raise RuntimeError("Existing unmanaged launcher left untouched: " + str(path))
        if old == content:
            return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".2pybot-new")
    temporary.write_text(content, encoding="utf-8")
    temporary.chmod(0o755 if executable else 0o644)
    os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=ROOT)
    parser.add_argument("--user", default="radxa")
    parser.add_argument("--deploy-config", type=Path, default=Path("/etc/2pybot-deploy.json"))
    args = parser.parse_args()
    if os.name != "posix":
        parser.error("Install the command environment on the Linux bot")
    import pwd
    if os.geteuid() != 0:
        parser.error("Run the installer with sudo")
    repo = args.repo.resolve()
    if not (repo / "scripts/radxa/bot_cli.py").is_file():
        parser.error("The selected checkout has no bot_cli.py")
    account = pwd.getpwnam(args.user)
    home = Path(account.pw_dir)
    base = home / ".local/share/2pybot"
    environment = base / "command-env"
    config_path = Path("/etc/2pybot-cli.json")
    config = json.loads(args.deploy_config.read_text()) if args.deploy_config.exists() else {}
    existing = {}
    if config_path.exists():
        existing = json.loads(config_path.read_text())
        if existing.get("managed_by") != "2pybot-command-environment":
            raise RuntimeError("Existing unmanaged configuration left untouched: " + str(config_path))
    # Check every public command before installing anything over a user's tools.
    for name in ["bot", "bot-env", *ALIASES]:
        path = Path("/usr/local/bin") / name
        if path.exists() and MARKER not in path.read_text(errors="replace"):
            raise RuntimeError("Existing unmanaged command left untouched: " + str(path))
    for directory in [home / ".local", home / ".local/share", base, base / "backups"]:
        if not directory.exists():
            directory.mkdir()
            os.chown(directory, account.pw_uid, account.pw_gid)
    if not (environment / "bin/python3").is_file():
        subprocess.run(["runuser", "-u", args.user, "--", "env", "HOME=" + str(home), sys.executable,
                        "-m", "venv", "--without-pip", "--prompt", "2pybot", str(environment)], check=True)
    settings = {"managed_by": "2pybot-command-environment", "repo": str(repo),
                "api": config.get("api", "http://127.0.0.1:8080"),
                "firmware_root": config.get("firmware_root", "/var/lib/2pybot/firmware"),
                "deploy_run": config.get("run_dir", "/run/2pybot-deploy"),
                "backups": str(base / "backups"), "arduino_cli": config.get("arduino_cli", "arduino-cli"),
                "serial_port": config.get("serial_port", "/dev/ttyUSB0"),
                "real_git": config.get("real_git", "/usr/bin/git"),
                "flasher_python": config.get("flasher_python", str(home / "tools/2pybot-flasher/bin/python3"))}
    settings.update(existing)
    settings.update(managed_by="2pybot-command-environment", repo=str(repo))
    text = json.dumps(settings, indent=2) + "\n"
    if not config_path.exists() or config_path.read_text() != text:
        temporary = config_path.with_suffix(".json.new")
        temporary.write_text(text)
        temporary.chmod(0o644)
        os.replace(temporary, config_path)
    quote = shlex.quote
    launcher = "#!/bin/sh\n" + MARKER + "\nexec " + quote(str(environment / "bin/python3")) + " " + quote(str(repo / "scripts/radxa/bot_cli.py")) + ' "$@"\n'
    for directory in [Path("/usr/local/bin"), environment / "bin"]:
        managed(directory / "bot", launcher, True)
        for name, command in ALIASES.items():
            managed(directory / name, "#!/bin/sh\n" + MARKER + "\nexec /usr/local/bin/bot " + quote(command) + ' "$@"\n', True)
    shellrc = base / "shellrc"
    managed(shellrc, MARKER + '\n[ ! -f "$HOME/.bashrc" ] || . "$HOME/.bashrc"\n. ' +
            quote(str(environment / "bin/activate")) + '\nexport BOT_CLI_CONFIG=' + quote(str(config_path)) + '\n' +
            'complete -W "status check firmware-status build-firmware flash-fw deploy deploy-status update calibration diagnostics doctor backup restore params stop logs restart toolchain" bot\n')
    managed(Path("/usr/local/bin/bot-env"), "#!/bin/sh\n" + MARKER + "\nexec /bin/bash --rcfile " + quote(str(shellrc)) + ' -i "$@"\n', True)
    ssh_activation(home / ".bashrc", environment, account)
    login = next((home / name for name in (".bash_profile", ".bash_login", ".profile") if (home / name).exists()), home / ".profile")
    ssh_activation(login, environment, account)
    print("Bot commands installed. From any directory: bot --help")
    print("New interactive SSH logins automatically activate (2pybot).")
    print("Optional isolated shell: bot-env (exit returns to your previous shell).")


if __name__ == "__main__":
    main()
