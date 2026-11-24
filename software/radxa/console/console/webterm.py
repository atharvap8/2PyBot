"""Browser terminal: a PTY running the 2pybot TUI, bridged over a websocket.

The console serves the vendored xterm.js page at /terminal and this websocket at
/terminal/ws. Input arrives as binary (keystrokes), resize as a small JSON text
message. The child runs in its own session with TERM set so curses works.
"""
import asyncio
import fcntl
import os
import pty
import shutil
import struct
import subprocess
import termios


def _command():
    override = os.environ.get("BOT_TERMINAL_CMD")
    if override:
        return override.split()
    exe = shutil.which("2pybot") or ("/usr/local/bin/2pybot" if os.path.exists("/usr/local/bin/2pybot") else None)
    return [exe, "tui"] if exe else ["bash", "-l"]


def _spawn(cmd):
    master, slave = pty.openpty()
    env = dict(os.environ, TERM="xterm-256color", COLORTERM="truecolor")
    proc = subprocess.Popen(cmd, stdin=slave, stdout=slave, stderr=slave,
                            start_new_session=True, env=env, close_fds=True)
    os.close(slave)
    flags = fcntl.fcntl(master, fcntl.F_GETFL)
    fcntl.fcntl(master, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    return proc, master


def _resize(fd, rows, cols):
    try:
        fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", max(1, rows), max(1, cols), 0, 0))
    except OSError:
        pass


def register(app, enabled=True):
    import json

    from fastapi import WebSocket

    if not enabled:
        return

    @app.websocket("/terminal/ws")
    async def terminal_ws(ws: WebSocket):
        await ws.accept()
        proc, master = _spawn(_command())
        loop = asyncio.get_running_loop()
        alive = True

        def on_readable():
            nonlocal alive
            try:
                data = os.read(master, 65536)
            except (OSError, BlockingIOError):
                return
            if data:
                asyncio.ensure_future(ws.send_bytes(data))
            else:
                alive = False
                loop.remove_reader(master)
                asyncio.ensure_future(ws.close())

        loop.add_reader(master, on_readable)
        try:
            while alive:
                msg = await ws.receive()
                if msg.get("type") == "websocket.disconnect":
                    break
                if msg.get("bytes") is not None:
                    os.write(master, msg["bytes"])
                elif msg.get("text") is not None:
                    text = msg["text"]
                    try:
                        obj = json.loads(text)
                        if isinstance(obj, dict) and "resize" in obj:
                            cols, rows = obj["resize"]
                            _resize(master, int(rows), int(cols))
                            continue
                    except (ValueError, TypeError, KeyError):
                        pass
                    os.write(master, text.encode())
        except Exception:  # noqa: BLE001 -- client gone or pty closed; clean up below
            pass
        finally:
            try:
                loop.remove_reader(master)
            except (ValueError, OSError):
                pass
            try:
                proc.terminate()
            except (ProcessLookupError, OSError):
                pass
            try:
                os.close(master)
            except OSError:
                pass
