"""Preview the console UI on a laptop, without the robot.

Serves the UI with the same paths the robot uses, static files only (every /api
call answers 404). Open  http://127.0.0.1:8765/?demo  to see the console running on
synthetic data: parked, driving a route, and pulling up close to an obstacle.

    python scripts/preview_console.py [port]
"""
import http.server
import sys
from pathlib import Path

UI = Path(__file__).resolve().parents[1] / "software/radxa/console/ui"
PAGES = {"/": "index.html", "/deploy": "deployment.html", "/terminal": "terminal.html",
         "/slam/": "slam/index.html", "/sysmap/": "sysmap/index.html"}
PREFIXES = (("/ui/", ""), ("/slam/static/", "slam/"), ("/sysmap/static/", "sysmap/"))


def resolve(url_path):
    """Map a request path to a file under the UI directory, or None."""
    path = url_path.split("?", 1)[0].split("#", 1)[0]
    if path in PAGES:
        return UI / PAGES[path]
    for prefix, folder in PREFIXES:
        if path.startswith(prefix):
            target = (UI / folder / path[len(prefix):]).resolve()
            return target if UI.resolve() in target.parents and target.is_file() else None
    return None


class Handler(http.server.SimpleHTTPRequestHandler):
    def translate_path(self, path):
        target = resolve(path)
        return str(target) if target else str(UI / "__missing__")

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *args):
        pass


def serve(port=8765):
    return http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler)


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8765
    print(f"Console preview: http://127.0.0.1:{port}/?demo   (Ctrl+C to stop)")
    try:
        serve(port).serve_forever()
    except KeyboardInterrupt:
        pass
