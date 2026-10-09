"""Read-only, token-scoped loopback resource server for WebEngine ES modules.

Only viewer assets and the bundled model directory can be served. No request
can access application code, user files or a directory listing. A random port
and session token are used and all external WebEngine requests are blocked.
"""

from __future__ import annotations

import mimetypes
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit


def resource_path(resources: Path, request_path: str, token: str) -> Path | None:
    raw = unquote(urlsplit(request_path).path)
    prefix = f"/{token}/"
    if not raw.startswith(prefix) or "\\" in raw or "\x00" in raw:
        return None
    parts = raw[len(prefix):].split("/")
    if not parts or parts[0] not in {"viewer", "models"}:
        return None
    if any(part in {"..", ".", ""} for part in parts):
        return None
    allowed = (resources / parts[0]).resolve()
    path = resources.joinpath(*parts).resolve()
    if not path.is_relative_to(allowed) or not path.is_file():
        return None
    return path


class AssetServer:
    def __init__(self, resources: Path):
        self.resources = resources.resolve()
        self.token = secrets.token_urlsafe(24)
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass

            def do_GET(self):
                self._serve(False)

            def do_HEAD(self):
                self._serve(True)

            def _serve(self, head_only):
                if self.headers.get("Host") != owner.host:
                    self.send_error(403)
                    return
                origin = self.headers.get("Origin")
                if origin and origin != f"http://{owner.host}":
                    self.send_error(403)
                    return
                path = resource_path(owner.resources, self.path, owner.token)
                if path is None:
                    self.send_error(404)
                    return
                try:
                    data = path.read_bytes()
                except OSError:
                    self.send_error(404)
                    return
                media = {".js": "text/javascript", ".urdf": "application/xml",
                         ".stl": "application/octet-stream"}.get(path.suffix.lower())
                media = media or mimetypes.guess_type(path.name)[0] or "application/octet-stream"
                self.send_response(200)
                self.send_header("Content-Type", media)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Referrer-Policy", "no-referrer")
                self.send_header("Content-Security-Policy",
                    "default-src 'self'; script-src 'self' qrc: 'unsafe-inline'; "
                    "style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; "
                    "connect-src 'self'; frame-ancestors 'none'; object-src 'none'")
                self.end_headers()
                if not head_only:
                    try:
                        self.wfile.write(data)
                    except (ConnectionError, OSError):
                        # WebEngine can cancel a mesh request while closing.
                        pass

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self._server.daemon_threads = True
        self.host = f"127.0.0.1:{self._server.server_port}"
        self.base_url = f"http://{self.host}/{self.token}/"
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        kwargs={"poll_interval": 0.1}, daemon=True,
                                        name="dummy-urdf-assets")
        self._thread.start()
        self._closed = False

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=0.5)
